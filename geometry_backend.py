import os
import re
import json
import time
import hashlib
import polyscope as ps
import numpy as np
import trimesh

# FR5 link meshes, zero-pose world frame (see docs/FR5_Mesh_Convention.md)
MESH_DIR = "assets/fr5_meshes"
MESH_FILES = [f"Robot{i}.obj" for i in range(7)]  # Robot0 (base) .. Robot6

# Tool head: nozzle mesh + TCP point, same zero-pose convention, mounted on the flange (Delta_6)
PRINTER_HEAD_DIR = "assets/printerHead"
NOZZLE_FILE = "nozzle.obj"
TCP_FILE = "TCP.txt"

TRAJECTORY_SAMPLE_INTERVAL_S = 0.1  # Minimum seconds between recorded TCP trajectory points
TRAJECTORY_RADIUS_MM = 2.0  # Trajectory curve line thickness, world units (mm)
TCP_FRAME_SCALE_MM = 50.0  # TCP coordinate-axes length, world units (mm)

PLAYBACK_RENDER_STRIDE = 50  # Push arm/bead updates to Polyscope every Nth solved
# waypoint, not every frame -- precompute_joint_path itself is untouched (roadmap
# Stage5_README.md 5.9). Waypoints, not frames, since step_count varies 1-100 with
# the Speed slider. update_vertex_positions() re-uploads the FULL bead buffer every
# call, not just the changed slice (docs/Polyscope_Quickstart.md), so fewer/coarser
# pushes cut real GPU upload cost, not just Python-side work -- settled.md S1.18.

TRAJECTORY_CURVE_RENDER_STRIDE = 5  # Re-register the "Trajectory" curve network
# every Nth recorded sample, not every sample -- trajectory_points itself stays
# dense (roadmap Stage5_README.md 5.9); register_curve_network() has no incremental
# grow-node-count API, so this only throttles how often the O(n) rebuild fires.

PLAYBACK_LOOKAHEAD_BEADS = 5000  # How far ahead of current progress the
# registered "G-code Print" mesh is grown, in beads -- kept close to actual
# playback progress instead of registering the full K-bead mesh from frame 1
# (settled.md S1.20). Real per-frame render cost turned out to scale with the
# registered mesh size, not update frequency alone (S1.17-S1.19 measured this
# with a flawed screenshot-based proxy that masked it -- see S1.20). Value is
# an empirical tuning knob, balancing re-registration frequency against how
# far ahead of true progress the draw cost is allowed to run.

BUILD_PLATE_DIR = "assets/buildPlate"
BUILD_PLATE_FILE = "BambuLab_BuildPlate.obj"
PLATE_COLOR = (0.75, 0.75, 0.78)  # Light cool gray, visually distinct from the orange print
# Measured thickness of BambuLab_BuildPlate.obj (its local Z span is [-0.75, 0],
# origin at the top corner) -- position_mm marks the resting/bottom face, so the
# top/print surface sits this far above it. See BuildPlate_UserFrame.md.
PLATE_THICKNESS_MM = 0.75

# Placed in the (-X, -Y) quadrant to match the arm's natural zero/home-pose
# reach direction -- the opposite quadrant only reaches via a near-limit J1
# rotation, leaving little margin for the wrist to also orient freely
USER_FRAME_ORIGIN_MM = np.array([-600.0, -300.0, 0.0])
USER_FRAME_SCALE_MM = 50.0  # Fixed axes drawn at the user frame, world units (mm)
BUILD_PLATE_POSITION_FILE = os.path.join(BUILD_PLATE_DIR, "saved_position.json")  # GUI Save/Load Position buttons

GCODE_DIR = "assets/models/gcode"
GCODE_FILE = "model.gcode"  # Fixed name -- overwritten by each new Cura export, never hand-edited
GCODE_COLOR = (1.0, 0.55, 0.0)  # Orange, so it doesn't visually merge with the Trajectory curve
# Assumed, not parsed -- Cura's exported G-code carries no filament/nozzle-diameter
# comment to read instead. 1.75mm is the standard FDM default. Used to convert
# extruded filament length (E) into a deposited bead volume, see load_gcode().
FILAMENT_DIAMETER_MM = 1.75

# How straight (dot product of unit tangents) and width-matched (mm) two
# back-to-back bead segments must be for their shared cap faces to be treated
# as exactly coincident -- and thus safely dropped -- rather than meeting at a
# visible angle or ledge. See _build_gcode_beads, settled.md S1.19.
CAP_CULL_COLINEAR_DOT_MIN = 0.999  # ~2.6 degrees
CAP_CULL_WIDTH_TOL_MM = 0.01

GCODE_MOVE_RE = re.compile(r"([A-Za-z])\s*(-?\d+\.?\d*)")

PRECOMPUTE_CHUNK_SIZE = 25  # waypoints solved per step() call -- keeps each
# per-frame batch well under a 60fps budget. Measured ~0.5ms/waypoint for
# solve_ik_tcp_matrix + the ground-clearance filter at benchy scale (see
# settled.md S1.13's verification), so this is roughly a 12ms slice per frame.

GCODE_PRECOMPUTE_CACHE = os.path.join(GCODE_DIR, "model.precompute.npz")  # roadmap 5.10, settled.md S1.21
PRECOMPUTE_CACHE_VERSION = 1  # Bump to invalidate all existing caches on a schema change


class VisContent:
    """
    [Backend Logic Layer]
    Responsibilities:
    1. Maintain geometry data (Mesh, Point Cloud)
    2. Execute geometry algorithms (Registration, Optimisation)
    3. Call Polyscope to register data (ps.register_...)
    """
    def __init__(self):
        # State data
        self.mesh_data = None
        self.current_joint_angles = None

        self.tcp_world = None            # Current TCP world position, set each apply_delta_transform call
        self.trajectory_points = []      # Recorded TCP world positions (see record_trajectory_point)
        self._last_sample_time = time.time()
        self._trajectory_curve_sample_count = 0  # Throttles _update_trajectory_curve() re-registration, see TRAJECTORY_CURVE_RENDER_STRIDE
        self.trajectory_enabled = True
        self.trajectory_handle = None    # Set once a curve exists, see _update_trajectory_curve

        # Chunked toolpath IK precompute state -- mirrors gui_panel.py's
        # is_playing/playback_waypoint_index one-to-one (see settled.md S1.14),
        # just on the backend since real work (G-code parsing, chunked IK)
        # has to live there. See run_toolpath_ik_precompute().
        self.precompute_running = False
        self.precompute_index = 0
        self.precompute_total = 0
        self.precompute_waypoints = None
        self.precompute_R_target = None
        self.precompute_joint_limits = None
        self.precompute_ref = None
        self.precompute_joint_path = []
        self.precompute_status = ""
        self.precompute_cache_meta = None  # Cache key captured at precompute-start, see run_toolpath_ik_precompute()

        # Progressive-reveal playback state -- roadmap Stage5_README.md 5.7,
        # settled.md S1.16. Mirrors the precompute state above: playback_running
        # is is_playing's backend equivalent, playback_index persists across
        # pause, only reset_toolpath_playback() zeroes it.
        self.playback_running = False
        self.playback_index = 0
        self._last_rendered_playback_index = 0  # Throttles the Polyscope push in advance_toolpath_playback, see PLAYBACK_RENDER_STRIDE
        self.playback_total = 0
        self.playback_status = ""
        self.gcode_bead_verts_full = None       # (K*8,3) world space, real bead positions
        self.gcode_bead_faces = None
        self.gcode_bead_reveal_index = None     # (K,) sorted ascending, see _build_gcode_beads
        self.gcode_bead_face_prefix = None      # (K+1,) cumulative triangle count, see _build_gcode_beads
        self.gcode_bead_verts_current = None    # (K*8,3) working copy, mutated as beads reveal
        self.gcode_print_handle = None          # Polyscope handle, reused across advance() calls
        self._registered_bead_capacity = 0      # How many beads are actually registered with
        # Polyscope right now -- kept close to playback progress, not pinned at the full K from
        # frame 1, so draw/upload cost tracks progress (settled.md S1.20). See PLAYBACK_LOOKAHEAD_BEADS.

        # Initialise the scene
        self.create_coordinate_frame()
        self.load_build_plate()
        self.mesh_data = self.load_data()
        self.update_arm([0, 0, 0, 0, 0, 0])


    def create_coordinate_frame(self, scale=1.0, origin=(0, 0, 0), rotation=None, name="Coordinate Frame"):
        """Register an XYZ axis triad. Reused for the static world-origin
        frame (defaults) and, with an origin/scale/name override, for the
        TCP frame -- see load_data() and docs/FR5_Mesh_Convention.md. An
        optional rotation (3x3) tilts the triad's axes -- used by
        load_build_plate() so the "User Frame" triad matches the plate's
        orientation; defaults to identity (axis-aligned) for every other
        caller. Returns (handle, nodes) so callers can drive the nodes
        through the Delta transform like any other zero-pose-frame
        geometry."""
        origin = np.asarray(origin, dtype=float)
        R = np.eye(3) if rotation is None else rotation
        nodes = np.array([origin, origin + R @ [scale,0,0], origin + R @ [0,scale,0], origin + R @ [0,0,scale]])
        edges = np.array([[0,1], [0,2], [0,3]])

        ps_net = ps.register_curve_network(name, nodes, edges)

        # X=red, Y=green, Z=blue
        colors = np.array([[1,0,0], [0,1,0], [0,0,1]])
        ps_net.add_color_quantity("axis_colors", colors, defined_on='edges', enabled=True)

        return ps_net, nodes


    def load_build_plate(self, position_mm=USER_FRAME_ORIGIN_MM, rpy_deg=(0.0, 0.0, 0.0)):
        """Place the build plate at the user frame -- see GLOSSARY.md 'User
        frame' and settled.md S1.2/S1.6. Static geometry: registered once
        per call, never updated per-frame -- unlike the arm links, the
        plate isn't driven by any joint, so it needs no Delta transform,
        just a homogeneous transform applied here. Defaults reproduce the
        original translation-only placement exactly. rpy_deg is [roll,
        pitch, yaw] degrees, XYZ fixed-angle convention (R = Rz(yaw) @
        Ry(pitch) @ Rx(roll)) -- same convention as solve_ik_tcp. Safe to
        call repeatedly (e.g. from the GUI's Move/Reset buttons); Polyscope
        replaces the prior structures of the same names. position_mm marks
        the plate's resting/bottom face -- the raw mesh's local origin sits
        at its top face instead, so local vertices are shifted up by
        PLATE_THICKNESS_MM before the transform to compensate."""
        roll, pitch, yaw = np.deg2rad(rpy_deg)
        R = rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)

        self.T_user_frame = np.eye(4)
        self.T_user_frame[:3, :3] = R
        self.T_user_frame[:3, 3] = position_mm

        plate = self.load_mesh(os.path.join(BUILD_PLATE_DIR, BUILD_PLATE_FILE))
        plate_verts_local = plate.vertices + np.array([0.0, 0.0, PLATE_THICKNESS_MM])
        homo = np.hstack([plate_verts_local, np.ones((len(plate_verts_local), 1))])
        plate_verts_world = (self.T_user_frame @ homo.T).T[:, :3]
        plate_handle = ps.register_surface_mesh("Build Plate", plate_verts_world, plate.faces)
        plate_handle.set_color(PLATE_COLOR)

        self.create_coordinate_frame(scale=USER_FRAME_SCALE_MM, origin=position_mm, rotation=R, name="User Frame")


    def save_build_plate_position(self, position_mm, rpy_deg):
        """Write the given build-plate pose to assets/buildPlate/ so it can
        be recalled later via load_saved_build_plate_position() -- see the
        GUI's "Save Position" button. Only ever called on explicit user
        action, never automatically."""
        data = {
            "position_mm": np.asarray(position_mm, dtype=float).tolist(),
            "rpy_deg": np.asarray(rpy_deg, dtype=float).tolist(),
        }
        with open(BUILD_PLATE_POSITION_FILE, "w") as f:
            json.dump(data, f, indent=2)


    def load_saved_build_plate_position(self):
        """Read a previously saved build-plate pose (if any) and apply it
        immediately via load_build_plate(). Only ever called on explicit
        user action (the GUI's "Load Saved Position" button), never
        automatically at startup -- see settled.md S1.6. Returns
        (position_mm, rpy_deg, status_message); position_mm/rpy_deg are
        None on failure so the GUI knows not to update its input fields."""
        if not os.path.exists(BUILD_PLATE_POSITION_FILE):
            return None, None, "No saved position found"

        with open(BUILD_PLATE_POSITION_FILE) as f:
            data = json.load(f)

        position_mm = np.array(data["position_mm"], dtype=float)
        rpy_deg = np.array(data["rpy_deg"], dtype=float)
        self.load_build_plate(position_mm, rpy_deg)
        return position_mm, rpy_deg, "Loaded saved position"


    def parse_gcode(self, filepath):
        """Parse G0/G1 linear moves. Returns a list of ([x, y, z], e,
        is_feed_move) triples, plate-local mm -- G0 travel moves update
        position but are flagged is_feed_move=False so load_gcode() skips
        drawing them. Position and extrusion (E) are both modal: a G0
        travel line carries no E and simply leaves it unchanged. Modal
        position handling: see GLOSSARY.md 'G-code toolpath'. G0/G1-only by
        design (see settled.md S1.7) -- any other G/M-code, and any line
        that isn't G0/G1, is discarded in software here, not assumed
        absent from the input file."""
        x, y, z, e = 0.0, 0.0, 0.0, 0.0
        points = []
        with open(filepath) as f:
            for line in f:
                line = line.split(';', 1)[0]
                line = re.sub(r"\([^)]*\)", "", line)
                words = GCODE_MOVE_RE.findall(line)
                if not words:
                    continue

                letter0, value0 = words[0]
                if letter0.upper() != 'G':
                    continue
                try:
                    code = int(float(value0))
                except ValueError:
                    continue
                if code not in (0, 1):
                    continue

                for letter, value in words[1:]:
                    letter = letter.upper()
                    if letter == 'X':
                        x = float(value)
                    elif letter == 'Y':
                        y = float(value)
                    elif letter == 'Z':
                        z = float(value)
                    elif letter == 'E':
                        e = float(value)
                points.append(([x, y, z], e, code == 1))
        return points


    # One bead box's 12 triangles: corners 0-3 = bottom, 4-7 = top above them.
    # Reused per segment via a vertex-index offset, see load_gcode().
    _BEAD_BOX_FACE_TEMPLATE = np.array([
        [0, 1, 2], [0, 2, 3],       # bottom
        [4, 6, 5], [4, 7, 6],       # top (reversed winding vs. bottom)
        [0, 1, 5], [0, 5, 4],       # side 0-1
        [1, 2, 6], [1, 6, 5],       # side 1-2
        [2, 3, 7], [2, 7, 6],       # side 2-3
        [3, 0, 4], [3, 4, 7],       # side 3-0
    ])

    def _build_gcode_beads(self, gcode_points):
        """Compute the swept bead-mesh geometry for a parsed G-code point
        list -- the shared math behind both the always-visible static
        preview (load_gcode()) and the progressive-reveal playback mesh
        (reset_/advance_toolpath_playback(), roadmap Stage5_README.md 5.7).
        Bead height tracks the actual printed Z per layer rather than
        slicer metadata, since real files include non-extruding Z
        excursions (e.g. a startup clearance lift) that would corrupt a
        naive Z-change tracker. Bead width comes from extruded E assuming
        FILAMENT_DIAMETER_MM. Vectorised across all segments (no
        per-segment loop) for the ~180,000-segment scale of a real print
        (settled.md S1.9).

        Returns (verts_world, faces, reveal_waypoint_index, bead_face_prefix):
          verts_world: (K*8, 3) float, world-space bead-box corners,
            bead-major (8 contiguous rows per bead) -- matches
            _BEAD_BOX_FACE_TEMPLATE's indexing.
          faces: (<=K*12, 3) int, triangle indices into verts_world -- fewer
            than 12 per bead wherever settled.md S1.19's cap culling drops
            a hidden face, so no longer a fixed per-bead stride.
          reveal_waypoint_index: (K,) int, strictly increasing -- the
            0-based index into gcode_points at which bead k's segment ends
            (segment i connects point i -> point i+1, so it's revealed
            once playback reaches point i+1).
          bead_face_prefix: (K+1,) int, cumulative triangle count --
            `faces[:bead_face_prefix[n]]` is exactly the triangles for the
            first n beads, needed because `faces` no longer has a fixed
            per-bead stride (settled.md S1.20, right-sized playback
            registration). All four arrays are empty/trivial (K == 0) if
            there are no printed beads.
        """
        pts = np.array([p for p, _, _ in gcode_points])
        es = np.array([e for _, e, _ in gcode_points])
        is_feed = np.array([f for _, _, f in gcode_points])

        p0, p1 = pts[:-1], pts[1:]
        seg_vec = p1 - p0
        seg_len = np.linalg.norm(seg_vec, axis=1)
        delta_e = es[1:] - es[:-1]
        z_dest = p1[:, 2]

        # Only extruding segments advance the layer floor (settled.md S1.9) --
        # skips non-extruding Z excursions like a startup clearance lift.
        seg_is_print = is_feed[1:] & (delta_e > 1e-9)
        bead_bottom = np.empty(len(z_dest))
        prev_print_z = None
        cur_bottom = 0.0
        for i in range(len(z_dest)):
            if seg_is_print[i]:
                zi = z_dest[i]
                if prev_print_z is None or zi > prev_print_z:
                    if prev_print_z is not None:
                        cur_bottom = prev_print_z
                elif zi < prev_print_z:
                    cur_bottom = 0.0  # unexpected downward jump; rest on the plate
                prev_print_z = zi
            bead_bottom[i] = cur_bottom
        bead_top = z_dest
        bead_height = bead_top - bead_bottom

        safe_len = np.where(seg_len > 1e-9, seg_len, 1.0)
        u = seg_vec / safe_len[:, None]
        w_axis = np.cross(u, [0.0, 0.0, 1.0])  # horizontal, perpendicular to travel
        w_norm = np.linalg.norm(w_axis, axis=1)
        safe_w_norm = np.where(w_norm > 1e-9, w_norm, 1.0)
        w_axis = w_axis / safe_w_norm[:, None]

        filament_area = np.pi * (FILAMENT_DIAMETER_MM / 2.0) ** 2
        safe_denom = np.where((seg_len * bead_height) > 1e-9, seg_len * bead_height, 1.0)
        width = (delta_e * filament_area) / safe_denom

        valid = seg_is_print & (seg_len > 1e-6) & (w_norm > 1e-6) & (bead_height > 1e-9)
        if not np.any(valid):
            return (np.empty((0, 3)), np.empty((0, 3), dtype=int), np.empty(0, dtype=int),
                    np.zeros(1, dtype=int))

        # Capture segment indices before the `valid` filter overwrites p0/p1 --
        # segment i connects gcode_points[i] -> gcode_points[i+1], revealed once
        # playback reaches point i+1.
        reveal_waypoint_index = np.nonzero(valid)[0] + 1

        p0, p1 = p0[valid], p1[valid]
        w_axis = w_axis[valid]
        half_w = (width[valid] / 2.0)[:, None]
        bottom = (bead_bottom[valid] + PLATE_THICKNESS_MM)[:, None]
        top = (bead_top[valid] + PLATE_THICKNESS_MM)[:, None]

        offset = w_axis[:, :2] * half_w
        c0 = p0[:, :2] + offset
        c1 = p0[:, :2] - offset
        c2 = p1[:, :2] - offset
        c3 = p1[:, :2] + offset

        K = len(p0)
        verts_local = np.zeros((K, 8, 3))
        for idx, corner_xy in enumerate((c0, c1, c2, c3)):
            verts_local[:, idx, :2] = corner_xy
            verts_local[:, idx, 2] = bottom[:, 0]
            verts_local[:, idx + 4, :2] = corner_xy
            verts_local[:, idx + 4, 2] = top[:, 0]
        verts_local = verts_local.reshape(-1, 3)

        # Drop cap faces at bead-to-bead boundaries that are provably always
        # hidden: back-to-back in the G-code (no travel gap), colinear (same
        # travel direction -- at a turn the two cap planes meet at an angle,
        # not coincide), and width-matched (same cross-section, so no ledge
        # is exposed). ~8% of triangles on a real multi-layer print, since
        # most consecutive segments trace a curved surface and fail the
        # colinearity test -- settled.md S1.19.
        u_valid, width_valid = u[valid], width[valid]
        chained = np.diff(reveal_waypoint_index) == 1
        colinear = np.sum(u_valid[:-1] * u_valid[1:], axis=1) >= CAP_CULL_COLINEAR_DOT_MIN
        width_matched = np.abs(width_valid[:-1] - width_valid[1:]) <= CAP_CULL_WIDTH_TOL_MM
        cullable = chained & colinear & width_matched

        drop_end_cap = np.zeros(K, dtype=bool)    # bead k's end cap (template rows 8-9)
        drop_start_cap = np.zeros(K, dtype=bool)  # bead k's start cap (template rows 4-5)
        drop_end_cap[:-1] = cullable
        drop_start_cap[1:] = cullable

        keep_row = np.ones((K, 12), dtype=bool)
        keep_row[drop_end_cap, 8] = False
        keep_row[drop_end_cap, 9] = False
        keep_row[drop_start_cap, 4] = False
        keep_row[drop_start_cap, 5] = False

        faces_full = (self._BEAD_BOX_FACE_TEMPLATE[None, :, :] + (np.arange(K) * 8)[:, None, None])
        faces = faces_full[keep_row]
        bead_face_prefix = np.concatenate([[0], np.cumsum(keep_row.sum(axis=1))])

        homo = np.hstack([verts_local, np.ones((len(verts_local), 1))])
        verts_world = (self.T_user_frame @ homo.T).T[:, :3]

        return verts_world, faces, reveal_waypoint_index, bead_face_prefix


    def load_gcode(self):
        """Register the deposited G1 material as a swept bead mesh on the
        plate -- solid boxes, not a curve, so it reads as the printed
        object (settled.md S1.9). No-ops if the G-code file is missing;
        safe to call repeatedly, e.g. on plate reposition (settled.md
        S1.8). Geometry itself comes from _build_gcode_beads()."""
        filepath = os.path.join(GCODE_DIR, GCODE_FILE)
        if not os.path.exists(filepath):
            return

        waypoints = self.parse_gcode(filepath)
        if len(waypoints) < 2:
            return

        verts_world, faces, _reveal_waypoint_index, _bead_face_prefix = self._build_gcode_beads(waypoints)
        if len(verts_world) == 0:
            return

        handle = ps.register_surface_mesh("G-code Print", verts_world, faces)
        handle.set_color(GCODE_COLOR)


    def build_toolpath_waypoints_world(self, gcode_points):
        """
        Map parse_gcode()'s plate-local waypoints to world-space 6-DOF
        targets -- roadmap Stage5_README.md 5.4. gcode_points is
        parse_gcode()'s own return value (not re-parsed here, kept
        composable). Applies the same plate-local Z lift
        (PLATE_THICKNESS_MM) load_gcode()/load_build_plate() already use,
        then the same T_user_frame homogeneous multiply as load_gcode() --
        see settled.md S1.3.

        No subdivision: one returned waypoint per input point, in order --
        see settled.md S1.12.

        Returns (waypoints, R_target):
          waypoints: list of (pos_world_mm: np.ndarray[3], is_feed_move: bool),
            same length/order as gcode_points -- both G0 (travel) and G1
            (feed) points are included (unlike load_gcode()'s
            G1-extruding-only bead filter), so a caller can still tell them
            apart later.
          R_target: 3x3 np.ndarray, self.T_user_frame[:3,:3] snapshotted
            once here -- the constant TCP orientation for the whole path
            (the plate doesn't tilt mid-print, see settled.md S1.6/S1.8),
            not stored per-waypoint.
        """
        pts_local = np.array([p for p, _, _ in gcode_points], dtype=float)
        is_feed = [f for _, _, f in gcode_points]
        pts_local[:, 2] += PLATE_THICKNESS_MM
        homo = np.hstack([pts_local, np.ones((len(pts_local), 1))])
        pts_world = (self.T_user_frame @ homo.T).T[:, :3]
        R_target = self.T_user_frame[:3, :3].copy()
        return list(zip(pts_world, is_feed)), R_target


    # FR5 standard DH parameters: (a_mm, alpha_rad, d_mm, theta_offset_rad)
    # Source: docs/FR5_DH_Table.md
    DH_PARAMS = [
        (0,    np.pi / 2, 152, 0),
        (-425, 0,         0,   0),
        (-395, 0,         0,   0),
        (0,    np.pi / 2, 102, 0),
        (0,   -np.pi / 2, 102, 0),
        (0,    0,         100, 0),
    ]

       
    def compute_fk(self, joint_angles_deg):
        """
        joint_angles_deg: sequence of 6 joint angles in degrees [J1..J6]
        Returns [T_0_1, ..., T_0_6], each a 4x4 np.ndarray. T_0_6 (base->flange)
        is the last element.
        """
        T = np.eye(4)
        T_0_i = []
        for (a, alpha, d, theta_offset), joint_deg in zip(self.DH_PARAMS, joint_angles_deg):
            theta = np.deg2rad(joint_deg) + theta_offset
            T = T @ dh_transform(a, alpha, d, theta)
            T_0_i.append(T)
        return T_0_i


    def end_effector_position(self, joint_angles_deg):
        T_0_6 = (self.compute_fk(joint_angles_deg))[-1]
        print(T_0_6[:3, 3])


    def solve_ik(self, T_0_6_target):
        """
        Closed-form analytical IK for the FR5's specific DH geometry --
        NOT a literal port of a PUMA-style spherical wrist, since d4 and
        d5 are both nonzero here (same family as a UR5/UR10 wrist). Full
        derivation: docs/FR5_IK_Derivation.md.

        T_0_6_target: 4x4 np.ndarray, target flange pose (base frame).
        Returns a list of (joint_angles_deg: np.ndarray[6], is_wrist_singular: bool),
        one per geometrically valid branch (up to 8; branches that fail an
        acos/sqrt domain check -- pose unreachable along that branch -- are
        silently skipped).
        """
        d1 = self.DH_PARAMS[0][2]
        a2 = self.DH_PARAMS[1][0]
        a3 = self.DH_PARAMS[2][0]
        d4 = self.DH_PARAMS[3][2]
        d5 = self.DH_PARAMS[4][2]
        d6 = self.DH_PARAMS[5][2]

        R = T_0_6_target[:3, :3]
        p = T_0_6_target[:3, 3]
        P5 = p - d6 * R[:, 2]  # frame-5 origin, backing off the approach vector

        solutions = []
        R_xy_sq = P5[0] ** 2 + P5[1] ** 2
        if R_xy_sq < d4 ** 2:
            return solutions  # P5 can't reach the fixed shoulder-perpendicular offset

        for sign1 in (1, -1):
            theta1 = np.arctan2(P5[1], P5[0]) + np.arctan2(d4, sign1 * np.sqrt(R_xy_sq - d4 ** 2))
            c1, s1 = np.cos(theta1), np.sin(theta1)

            wrist_arg = (p[0] * s1 - p[1] * c1 - d4) / d6
            if abs(wrist_arg) > 1:
                continue

            R_0_1 = dh_transform(0, np.pi / 2, d1, theta1)[:3, :3]
            R_1_6 = R_0_1.T @ R

            for sign2 in (1, -1):
                theta5 = sign2 * np.arccos(wrist_arg)
                is_singular = abs(np.sin(theta5)) < 1e-6

                if is_singular:
                    theta6 = 0.0  # axes 4/6 aligned -- theta4/theta6 split is ambiguous
                else:
                    theta6 = np.arctan2(-R_1_6[2, 1] / np.sin(theta5), R_1_6[2, 0] / np.sin(theta5))

                K = rot_y(-theta5) @ rot_z(theta6)
                Rz_psi = R_1_6 @ K.T
                psi = np.arctan2(Rz_psi[1, 0], Rz_psi[0, 0])  # psi = theta2+theta3+theta4

                X_p = P5[0] * c1 + P5[1] * s1
                Y_p = P5[2] - d1
                X = X_p - d5 * np.sin(psi)
                Y = Y_p + d5 * np.cos(psi)

                cos_theta3 = (X ** 2 + Y ** 2 - a2 ** 2 - a3 ** 2) / (2 * a2 * a3)
                if abs(cos_theta3) > 1:
                    continue

                for sign3 in (1, -1):
                    theta3 = sign3 * np.arccos(cos_theta3)
                    theta2 = np.arctan2(Y, X) - np.arctan2(a3 * np.sin(theta3), a2 + a3 * np.cos(theta3))
                    theta4 = psi - theta2 - theta3

                    angles_rad = np.array([theta1, theta2, theta3, theta4, theta5, theta6])
                    solutions.append((np.rad2deg(angles_rad), is_singular))

        return solutions


    def solve_ik_tcp_matrix(self, target_pos_mm, R_target, joint_limits, reference_joint_angles=None):
        """
        Matrix-native IK entry point, targeting the TCP pose -- see settled.md
        S1.4. Converts via self.T_flange_to_tcp, solves, filters by
        joint_limits, then ranks every valid branch by closeness to a
        reference pose -- see settled.md S1.5.

        R_target: 3x3 np.ndarray, target TCP orientation (base frame).
        reference_joint_angles: joint_angles_deg (np.ndarray[6]) to rank
        branches against; defaults to self.current_joint_angles when None
        (the arm's live pose -- reproduces solve_ik_tcp's existing behavior).
        A future toolpath driver instead passes the previous waypoint's
        solved pose, for continuity between consecutive solves.

        Returns (solutions, status_message); solutions is a list of
        (joint_angles_deg, is_wrist_singular, raw_branch_index), sorted
        closest-to-reference first. Empty list on failure.
        """
        if reference_joint_angles is None:
            reference_joint_angles = self.current_joint_angles

        T_target_tcp = np.eye(4)
        T_target_tcp[:3, :3] = R_target
        T_target_tcp[:3, 3] = target_pos_mm
        T_target_flange = T_target_tcp @ np.linalg.inv(self.T_flange_to_tcp)

        branches = self.solve_ik(T_target_flange)
        if not branches:
            return [], "Unreachable: no geometric solution for this pose"

        def wrap_into_limits(angle_deg, lo, hi):
            # atan2-built angles come back in (-180,180]-ish ranges, but joints
            # like J2/J4 have physical limits past that (e.g. -264 deg) -- the
            # same physical angle can be +/-360 off and only one representation
            # falls inside the asymmetric limit window, so all three are tried.
            for k in (0, 360, -360):
                candidate = angle_deg + k
                if lo <= candidate <= hi:
                    return candidate
            return None

        valid = []
        for i, (angles, singular) in enumerate(branches):
            adjusted = [wrap_into_limits(a, lo, hi) for a, (lo, hi) in zip(angles, joint_limits)]
            if all(a is not None for a in adjusted):
                valid.append((np.array(adjusted), singular, i))
        if not valid:
            return [], f"Reachable but outside joint limits ({len(branches)} branch(es), none valid)"

        def wrapped_dist(angles):
            diff = (angles - reference_joint_angles + 180) % 360 - 180
            return np.sum(np.abs(diff))

        valid.sort(key=lambda item: wrapped_dist(item[0]))
        status = f"Solved ({len(valid)} valid solution{'s' if len(valid) != 1 else ''})"
        return valid, status


    def solve_ik_tcp(self, target_pos_mm, target_rpy_deg, joint_limits):
        """
        GUI-facing IK entry point, targeting the TCP pose via RPY orientation
        -- see settled.md S1.4. Thin wrapper: converts RPY to a rotation
        matrix, then delegates to solve_ik_tcp_matrix() (ranked against
        self.current_joint_angles, i.e. the arm's live pose -- see settled.md
        S1.5).

        target_rpy_deg: [roll, pitch, yaw] degrees, fixed-angle convention
        (R = Rz(yaw) @ Ry(pitch) @ Rx(roll)).
        Returns (solutions, status_message); solutions is a list of
        (joint_angles_deg, is_wrist_singular, raw_branch_index), sorted
        closest-to-current first. Empty list on failure.
        """
        roll, pitch, yaw = np.deg2rad(target_rpy_deg)
        R_target = rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)
        return self.solve_ik_tcp_matrix(target_pos_mm, R_target, joint_limits)


    def solve_toolpath_ik(self, waypoints, R_target, joint_limits, reference_joint_angles=None):
        """
        Solve IK across a whole toolpath, chaining continuity
        waypoint-to-waypoint -- see settled.md S1.11, roadmap
        Stage5_README.md 5.4. waypoints is
        build_toolpath_waypoints_world()'s first return value; R_target is
        its second (the constant TCP orientation for the whole path).

        For each waypoint, calls solve_ik_tcp_matrix() ranked against the
        previous waypoint's chosen solution (or reference_joint_angles /
        self.current_joint_angles for the first waypoint), then walks the
        ranked branches and takes the first one that clears the ground
        (_branch_clears_ground -- roadmap 5.5, settled.md S1.13), not
        blindly the top-ranked branch.

        Aborts the entire solve at the first waypoint with no valid branch,
        or where every valid branch dips below the ground plane -- no
        partial motion, matching the abort contract roadmap 5.6's chunked
        precompute will also need (settled.md S1.12).

        Returns (joint_path, status_message):
          joint_path: list of joint_angles_deg (np.ndarray[6]), one per
            waypoint, in order. Empty list on failure.
          status_message: "Solved N waypoint(s)" on success; on failure, the
            failing waypoint's index plus either solve_ik_tcp_matrix's own
            status string verbatim ("Unreachable: ..." / "Reachable but
            outside joint limits ...") or the ground-clearance failure
            message when every valid branch dips below z=0.
        """
        if reference_joint_angles is None:
            reference_joint_angles = self.current_joint_angles

        joint_path = []
        ref = reference_joint_angles
        for i, (pos_world_mm, _is_feed_move) in enumerate(waypoints):
            solutions, status = self.solve_ik_tcp_matrix(
                pos_world_mm, R_target, joint_limits, reference_joint_angles=ref)
            if not solutions:
                return [], f"Waypoint {i}/{len(waypoints)}: {status}"

            clear = next((angles for angles, *_ in solutions if self._branch_clears_ground(angles)), None)
            if clear is None:
                return [], f"Waypoint {i}/{len(waypoints)}: all {len(solutions)} valid branch(es) dip below the ground plane (z<0)"

            ref = clear
            joint_path.append(ref)
        return joint_path, f"Solved {len(joint_path)} waypoint(s)"


    def _toolpath_cache_meta(self, user_frame):
        """Cache-key dict for the toolpath precompute cache -- roadmap
        Stage5_README.md 5.10, settled.md S1.21. Compared by dict equality,
        not a hash-of-hash, so any single differing element is an
        unambiguous miss. Hashes the G-code file fresh from disk by content
        (SHA-256), not mtime -- a hand-edited-then-reverted file with an
        unchanged mtime still keys correctly. user_frame is the full 4x4
        build-plate pose, not just precompute_R_target's 3x3 rotation --
        waypoint XYZ positions are baked through T_user_frame's translation
        too (build_toolpath_waypoints_world). Rounded to 6 decimals to
        absorb float noise from repeated matrix ops without causing
        false-negative cache misses."""
        with open(os.path.join(GCODE_DIR, GCODE_FILE), "rb") as f:
            gcode_sha256 = hashlib.sha256(f.read()).hexdigest()
        return {
            "version": PRECOMPUTE_CACHE_VERSION,
            "gcode_sha256": gcode_sha256,
            "user_frame": np.round(np.asarray(user_frame, dtype=float), 6).tolist(),
        }


    def save_toolpath_precompute_cache(self):
        """Best-effort write of a just-completed precompute to
        GCODE_PRECOMPUTE_CACHE, tagged with the key captured at
        precompute-start (self.precompute_cache_meta) -- roadmap
        Stage5_README.md 5.10. Called only from step_toolpath_ik_precompute()'s
        successful-completion branch, never on an aborted/cancelled
        precompute. Wrapped in a bare except: a cache-write failure (disk
        full, permissions) must never surface as a failure of the
        precompute itself, which already succeeded in memory."""
        try:
            np.savez(
                GCODE_PRECOMPUTE_CACHE,
                joint_path=np.asarray(self.precompute_joint_path, dtype=np.float32),
                meta=np.array(json.dumps(self.precompute_cache_meta)))
        except Exception:
            pass


    def load_toolpath_precompute_cache(self):
        """Attempt to load a previously-saved precompute instead of
        re-solving -- roadmap Stage5_README.md 5.10. Rebuilds the cache key
        from the live self.T_user_frame (safe here since this only ever
        runs before any solving has started for the session) and compares
        it by dict equality against the cached meta. Any mismatch
        (different G-code content, moved plate, version bump) or any error
        (missing files, corrupt npz) is treated as a plain cache miss --
        fails open, letting the caller fall through to the normal
        parse/solve path; never raises."""
        filepath = os.path.join(GCODE_DIR, GCODE_FILE)
        if not (os.path.exists(GCODE_PRECOMPUTE_CACHE) and os.path.exists(filepath)):
            return False
        try:
            cached = np.load(GCODE_PRECOMPUTE_CACHE, allow_pickle=False)
            if json.loads(cached["meta"].item()) != self._toolpath_cache_meta(self.T_user_frame):
                return False
            joint_path = cached["joint_path"].astype(float)
        except Exception:
            return False

        self.precompute_joint_path = list(joint_path)
        self.precompute_index = len(joint_path)
        self.precompute_total = len(joint_path)
        self.precompute_running = False
        self.precompute_status = f"Loaded {len(joint_path)} waypoint(s) from cache"
        return True


    def run_toolpath_ik_precompute(self, joint_limits, reference_joint_angles=None):
        """Start or resume the chunked toolpath IK precompute -- roadmap
        Stage5_README.md 5.6. Mirrors the GUI's playback Run button
        (gui_panel.py): sets precompute_running True, and only (re-)parses
        the G-code and resets progress if nothing is loaded yet
        (precompute_waypoints is None -- true on the very first call, and
        again after cancel_toolpath_ik_precompute()). If a precompute is
        already loaded (i.e. paused), just resumes stepping from
        precompute_index -- no re-parsing, no restart. Reads the fixed
        G-code path (GCODE_DIR/GCODE_FILE, same convention as load_gcode())
        via parse_gcode() + build_toolpath_waypoints_world(), not a cached
        result from load_gcode() (which only keeps extruding-G1 preview
        beads, not the raw 1:1 waypoint list).

        Before parsing, tries load_toolpath_precompute_cache() -- on a hit,
        returns immediately with a completed path already loaded, skipping
        G-code parsing and IK entirely (roadmap Stage5_README.md 5.10).
        """
        if self.precompute_waypoints is None:
            if self.load_toolpath_precompute_cache():
                return

            filepath = os.path.join(GCODE_DIR, GCODE_FILE)
            if not os.path.exists(filepath):
                self.precompute_status = "No G-code file found"
                return

            gcode_points = self.parse_gcode(filepath)
            waypoints, R_target = self.build_toolpath_waypoints_world(gcode_points)
            if not waypoints:
                self.precompute_status = "No waypoints to solve"
                return

            self.precompute_waypoints = waypoints
            self.precompute_R_target = R_target
            self.precompute_joint_limits = joint_limits
            self.precompute_index = 0
            self.precompute_total = len(waypoints)
            self.precompute_joint_path = []
            self.precompute_ref = (
                reference_joint_angles if reference_joint_angles is not None else self.current_joint_angles)
            self.precompute_cache_meta = self._toolpath_cache_meta(self.T_user_frame)

        self.precompute_running = True
        self.precompute_status = f"Precomputing {self.precompute_index}/{self.precompute_total} waypoints"


    def pause_toolpath_ik_precompute(self):
        """Mirrors the GUI's playback Pause button: stop advancing the
        precompute without discarding progress. A following
        run_toolpath_ik_precompute() call continues from precompute_index."""
        self.precompute_running = False


    def cancel_toolpath_ik_precompute(self):
        """Mirrors the GUI's playback Reset button: stop and discard the
        precompute entirely, resetting progress back to zero -- a following
        run_toolpath_ik_precompute() call starts completely fresh
        (re-parses the G-code), matching Reset zeroing playback_waypoint_index."""
        self.precompute_running = False
        self.precompute_waypoints = None
        self.precompute_index = 0
        self.precompute_total = 0
        self.precompute_joint_path = []
        self.precompute_status = ""
        self.precompute_cache_meta = None


    def step_toolpath_ik_precompute(self):
        """Advance the in-progress precompute by up to
        PRECOMPUTE_CHUNK_SIZE waypoints -- call every frame from render()
        (roadmap Stage5_README.md 5.6). No-ops unless precompute_running.
        Uses the same per-waypoint logic as solve_toolpath_ik (solve, then
        walk ranked branches with _branch_clears_ground, settled.md S1.13):
        aborts the whole precompute -- no partial motion, settled.md S1.12
        -- at the first waypoint with no valid/ground-clearing branch."""
        if not self.precompute_running:
            return

        end = min(self.precompute_index + PRECOMPUTE_CHUNK_SIZE, self.precompute_total)
        for i in range(self.precompute_index, end):
            pos_world_mm, _is_feed_move = self.precompute_waypoints[i]
            solutions, status = self.solve_ik_tcp_matrix(
                pos_world_mm, self.precompute_R_target, self.precompute_joint_limits,
                reference_joint_angles=self.precompute_ref)
            if not solutions:
                self.precompute_running = False
                self.precompute_joint_path = []
                self.precompute_status = f"Waypoint {i}/{self.precompute_total}: {status}"
                return

            clear = next((angles for angles, *_ in solutions if self._branch_clears_ground(angles)), None)
            if clear is None:
                self.precompute_running = False
                self.precompute_joint_path = []
                self.precompute_status = (
                    f"Waypoint {i}/{self.precompute_total}: all {len(solutions)} valid branch(es) "
                    "dip below the ground plane (z<0)")
                return

            self.precompute_ref = clear
            self.precompute_joint_path.append(clear)

        self.precompute_index = end
        if self.precompute_index >= self.precompute_total:
            self.precompute_running = False
            self.precompute_status = f"Solved {self.precompute_total} waypoint(s)"
            self.save_toolpath_precompute_cache()
        else:
            self.precompute_status = f"Precomputing {self.precompute_index}/{self.precompute_total} waypoints"


    def _init_toolpath_playback(self):
        """Shared setup for reset_toolpath_playback() and the first
        run_toolpath_playback() call this session -- roadmap
        Stage5_README.md 5.7. Requires a completed precompute
        (self.precompute_joint_path); re-parses the fixed G-code path and
        rebuilds bead geometry via _build_gcode_beads() (not reused from
        load_gcode(), which doesn't return reveal_waypoint_index). Collapses
        every bead to its own first corner (zero-area, so nothing renders --
        no transparency involved, settled.md S1.16) and registers/replaces
        the "G-code Print" mesh with only the first PLAYBACK_LOOKAHEAD_BEADS
        beads' worth of that collapsed state, not the full K -- draw/upload
        cost tracks playback progress instead of being pinned at the full
        mesh's cost from frame 1 (settled.md S1.20). Snaps the arm to the
        first waypoint's solved pose. Returns True on success, False (with
        playback_status explaining why) otherwise."""
        if not self.precompute_joint_path:
            self.playback_status = "Run Precompute first"
            return False

        filepath = os.path.join(GCODE_DIR, GCODE_FILE)
        if not os.path.exists(filepath):
            self.playback_status = "No G-code file found"
            return False

        gcode_points = self.parse_gcode(filepath)
        verts_world, faces, reveal_index, face_prefix = self._build_gcode_beads(gcode_points)
        if len(verts_world) == 0:
            self.playback_status = "No printed beads to reveal"
            return False

        self.gcode_bead_verts_full = verts_world
        self.gcode_bead_faces = faces
        self.gcode_bead_reveal_index = reveal_index
        self.gcode_bead_face_prefix = face_prefix

        # Collapse every bead to its own first corner -- a zero-area box
        # renders nothing, revealed later by restoring real positions
        # (advance_toolpath_playback), never via transparency (settled.md S1.16).
        self.gcode_bead_verts_current = np.repeat(verts_world[0::8], 8, axis=0)

        K = len(reveal_index)
        self._registered_bead_capacity = min(PLAYBACK_LOOKAHEAD_BEADS, K)
        self.gcode_print_handle = ps.register_surface_mesh(
            "G-code Print",
            self.gcode_bead_verts_current[:self._registered_bead_capacity * 8],
            self.gcode_bead_faces[:self.gcode_bead_face_prefix[self._registered_bead_capacity]])
        self.gcode_print_handle.set_color(GCODE_COLOR)

        self.playback_index = 0
        self._last_rendered_playback_index = 0
        self.playback_total = len(self.precompute_joint_path)
        self.update_arm(self.precompute_joint_path[0])
        return True


    def reset_toolpath_playback(self):
        """Mirrors the GUI's playback Reset button: snaps to the first pose
        and empties the shape (roadmap Stage5_README.md 5.7) -- always a
        full re-init, discarding any in-progress reveal."""
        self.playback_running = False
        ok = self._init_toolpath_playback()
        if ok:
            self.playback_status = "Ready to play"


    def run_toolpath_playback(self):
        """Mirrors the GUI's playback Run button: start or resume. If
        playback was never initialized this session (or was reset),
        initializes fresh; otherwise resumes from wherever playback_index
        already is (a paused run continues, not restarts)."""
        if self.gcode_bead_verts_full is None:
            if not self._init_toolpath_playback():
                return
        self.playback_running = True


    def pause_toolpath_playback(self):
        """Mirrors the GUI's playback Pause button: stop advancing without
        discarding progress. A following run_toolpath_playback() call
        continues from playback_index."""
        self.playback_running = False


    def advance_toolpath_playback(self, step_count):
        """Advance playback by up to step_count waypoints -- call every
        frame from render() (roadmap Stage5_README.md 5.7). No-ops unless
        playback_running. The waypoint index always advances every call;
        only the Polyscope push (arm pose + bead reveal) is throttled to
        every PLAYBACK_RENDER_STRIDE waypoints -- or forced on the final
        waypoint, so playback never ends on a stale mid-stride pose
        (roadmap Stage5_README.md 5.9). Reveals beads via a sorted cutoff
        over gcode_bead_reveal_index (strictly increasing by construction,
        settled.md S1.16) rather than a per-bead scan or mask, accumulated
        from the last *rendered* index so no bead is skipped across
        throttled frames.

        The registered "G-code Print" mesh is kept right-sized to progress
        (PLAYBACK_LOOKAHEAD_BEADS ahead of the last revealed bead, capped at
        the full K), not pinned at the full K from frame 1 -- real per-frame
        render cost tracks the registered mesh size, not just how often it's
        pushed (settled.md S1.20). Growing capacity re-registers (the
        periodic, progress-proportional cost); staying within the current
        capacity is a cheap same-size update_vertex_positions call."""
        if not self.playback_running:
            return

        new_index = min(self.playback_index + step_count, self.playback_total - 1)
        self.playback_index = new_index

        finished = self.playback_index >= self.playback_total - 1
        if finished or self.playback_index - self._last_rendered_playback_index >= PLAYBACK_RENDER_STRIDE:
            self.update_arm(self.precompute_joint_path[self.playback_index])

            old_revealed = np.searchsorted(self.gcode_bead_reveal_index, self._last_rendered_playback_index, side='right')
            new_revealed = np.searchsorted(self.gcode_bead_reveal_index, self.playback_index, side='right')
            if new_revealed > old_revealed:
                self.gcode_bead_verts_current[old_revealed * 8:new_revealed * 8] = \
                    self.gcode_bead_verts_full[old_revealed * 8:new_revealed * 8]

                K = len(self.gcode_bead_reveal_index)
                if finished or new_revealed >= self._registered_bead_capacity:
                    target_capacity = K if finished else min(new_revealed + PLAYBACK_LOOKAHEAD_BEADS, K)
                    self._registered_bead_capacity = target_capacity
                    self.gcode_print_handle = ps.register_surface_mesh(
                        "G-code Print",
                        self.gcode_bead_verts_current[:target_capacity * 8],
                        self.gcode_bead_faces[:self.gcode_bead_face_prefix[target_capacity]])
                    self.gcode_print_handle.set_color(GCODE_COLOR)
                else:
                    self.gcode_print_handle.update_vertex_positions(
                        self.gcode_bead_verts_current[:self._registered_bead_capacity * 8])

            self._last_rendered_playback_index = self.playback_index

        if finished:
            self.playback_running = False
            self.playback_status = "Playback complete"
        else:
            self.playback_status = f"Playing {self.playback_index}/{self.playback_total - 1}"


    def load_mesh(self, filepath):
        """Load a single OBJ mesh with trimesh.

        force='mesh' collapses multi-group OBJs into one Trimesh; without it,
        trimesh.load can return a Scene (no .vertices/.faces -> AttributeError
        later). See docs/Polyscope_Quickstart.md.
        """
        return trimesh.load(filepath, force='mesh')


    def load_data(self):
        """Load all 7 FR5 link meshes (Robot0..Robot6), register them with
        Polyscope, and cache the zero-pose data needed for the Delta
        transform (see docs/FR5_Mesh_Convention.md).

        Returns a list of trimesh.Trimesh, index i == Robot{i}.obj. Robot0 is the
        static base; Robot1..Robot6 correspond to T_0_1..T_0_6 from compute_fk().
        """
        meshes = [self.load_mesh(os.path.join(MESH_DIR, fname)) for fname in MESH_FILES]

        # Compute zero-pose transforms
        zero_joints = [0, 0, 0, 0, 0, 0]
        self.T_zero = self.compute_fk(zero_joints)

        # Store the rest-pose vertices for each list
        self.rest_verts = [m.vertices.copy() for m in meshes[1:]]  # List of Nx3 arrays, one per link (Robot1..Robot6)
        self.mesh_handles = []
        self.update_fns = []  # matching per-object position-update method (meshes vs point clouds differ)

        ps.register_surface_mesh("Robot0", meshes[0].vertices, meshes[0].faces)  # Fixed base, no transforms

        for i, m in enumerate(meshes[1:]):
            handle = ps.register_surface_mesh(f"Robot{i+1}", m.vertices, m.faces)
            self.mesh_handles.append(handle)
            self.update_fns.append(handle.update_vertex_positions)

        # Nozzle rides on the flange -- same Delta_6 as Robot6, see docs/FR5_Mesh_Convention.md
        nozzle = self.load_mesh(os.path.join(PRINTER_HEAD_DIR, NOZZLE_FILE))
        self.rest_verts.append(nozzle.vertices.copy())
        nozzle_handle = ps.register_surface_mesh("Nozzle", nozzle.vertices, nozzle.faces)
        self.mesh_handles.append(nozzle_handle)
        self.update_fns.append(nozzle_handle.update_vertex_positions)

        # Zero-pose bbox corners for the moving-geometry set (Robot1..6 + nozzle,
        # rest_verts[0:7] -- excludes the TCP point/frame appended below, which
        # are visualization markers, not solid robot geometry). See
        # moving_geometry_bbox_min_z (roadmap Stage5_README.md 5.5).
        self.moving_geometry_rest_bbox_corners = [_bbox_corners(v) for v in self.rest_verts]

        self.tcp_local = np.loadtxt(os.path.join(PRINTER_HEAD_DIR, TCP_FILE))  # Zero-pose world frame [x, y, z]

        # Fixed flange->TCP transform for IK; rotation comes from inv(T_zero[5]),
        # not assumed identity -- see settled.md S1.4 for why.
        T_zero_flange_inv = np.linalg.inv(self.T_zero[5])
        self.T_flange_to_tcp = T_zero_flange_inv.copy()
        self.T_flange_to_tcp[:3, 3] = (T_zero_flange_inv @ np.append(self.tcp_local, 1.0))[:3]

        # TCP point, also Delta_6, but a Polyscope point cloud -- update_point_positions,
        # not update_vertex_positions, hence the per-object self.update_fns lookup
        tcp_point = self.tcp_local.reshape(1, 3)
        self.rest_verts.append(tcp_point)
        point_cloud = ps.register_point_cloud("TCP", tcp_point)
        self.mesh_handles.append(point_cloud)
        self.update_fns.append(point_cloud.update_point_positions)

        # TCP orientation triad, also Delta_6 -- axis tips defined in the zero-pose
        # world frame around tcp_local, so they rotate with the tool via the same
        # Delta transform (curve network -> update_node_positions)
        tcp_frame_handle, tcp_frame_rest_nodes = self.create_coordinate_frame(
            scale=TCP_FRAME_SCALE_MM, origin=self.tcp_local, name="TCP Frame")
        self.rest_verts.append(tcp_frame_rest_nodes)
        self.mesh_handles.append(tcp_frame_handle)
        self.update_fns.append(tcp_frame_handle.update_node_positions)

        return meshes


    def apply_delta_transform(self, joint_angles_deg):
        """Update link mesh vertex positions for the given joint angles.

        Delta_i = T_0_i(q) @ inv(T_0_i(0)) -- see docs/FR5_Mesh_Convention.md.
        Robot0 is the fixed base and is never updated. The nozzle (index 6),
        TCP point (index 7), and TCP frame (index 8) ride on the flange,
        reusing Delta_6 (index 5).
        """
        T_current = self.compute_fk(joint_angles_deg)
        for i in range(9):
            src = min(i, 5)
            Delta = T_current[src] @ np.linalg.inv(self.T_zero[src])

            # Convert rest verts to homogenous [x,y,z,1]
            N = self.rest_verts[i].shape[0]
            homo = np.hstack([self.rest_verts[i], np.ones((N, 1))])

            # Apply delta
            new_verts = (Delta @ homo.T).T[:, :3]

            # Update Polyscope structure (mesh or point cloud, per registration)
            self.update_fns[i](new_verts)

            if i == 7:
                self.tcp_world = new_verts[0]


    def update_arm(self, joint_angles_deg):
        """GUI-facing entry point: record the current joint state and move
        the rendered arm to match via the Delta transform."""
        self.current_joint_angles = joint_angles_deg
        self.apply_delta_transform(joint_angles_deg)


    def _moving_geometry_deltas(self, joint_angles_deg):
        """Delta_i for each moving mesh (Robot1..Robot6 + nozzle), reusing
        Delta_6 for the nozzle -- same src = min(i, 5) mapping as
        apply_delta_transform, but pure computation with no Polyscope side
        effects (used by the ground-clearance checks below, not per-frame
        rendering)."""
        T_current = self.compute_fk(joint_angles_deg)
        return [T_current[min(i, 5)] @ np.linalg.inv(self.T_zero[min(i, 5)]) for i in range(7)]


    def moving_geometry_bbox_min_z(self, joint_angles_deg):
        """Cheap ground-clearance pre-check (roadmap Stage5_README.md 5.5):
        transform each moving mesh's cached zero-pose bounding-box corners (8
        per mesh, not the full vertex set) by its Delta transform and return
        the minimum world z reached, mm. This is a guaranteed lower bound on
        moving_geometry_min_z's result -- a rigid transform of an AABB's 8
        corners always produces a convex hull enclosing the mesh's true
        transformed extent, and z is linear so its minimum is attained at a
        corner -- so a non-negative result here proves the branch clears
        without needing the exact check."""
        deltas = self._moving_geometry_deltas(joint_angles_deg)
        min_z = np.inf
        for delta, corners in zip(deltas, self.moving_geometry_rest_bbox_corners):
            homo = np.hstack([corners, np.ones((len(corners), 1))])
            world = (delta @ homo.T).T[:, :3]
            min_z = min(min_z, world[:, 2].min())
        return min_z


    def moving_geometry_min_z(self, joint_angles_deg):
        """Exact ground-clearance check (roadmap Stage5_README.md 5.5):
        transform every vertex of every moving mesh and return the true
        minimum world z reached, mm. Slower than moving_geometry_bbox_min_z --
        only called when the bbox check doesn't already prove clearance."""
        deltas = self._moving_geometry_deltas(joint_angles_deg)
        min_z = np.inf
        for delta, verts in zip(deltas, self.rest_verts[:7]):
            homo = np.hstack([verts, np.ones((len(verts), 1))])
            world = (delta @ homo.T).T[:, :3]
            min_z = min(min_z, world[:, 2].min())
        return min_z


    def _branch_clears_ground(self, joint_angles_deg):
        """True if this branch's moving geometry never dips below world
        z=0 (the robot's own base-mounting plane, roadmap Stage5_README.md
        5.5). Checks the cheap bbox bound first -- proven clear if
        non-negative -- and only escalates to the exact per-vertex check
        when the bbox result is negative (inconclusive: a rotated AABB
        corner can dip below ground even when the real mesh does not)."""
        if self.moving_geometry_bbox_min_z(joint_angles_deg) >= 0:
            return True
        return self.moving_geometry_min_z(joint_angles_deg) >= 0


    def record_trajectory_point(self):
        """Sample self.tcp_world at most once per TRAJECTORY_SAMPLE_INTERVAL_S;
        discard the sample if the TCP hasn't moved since the last recorded
        point. trajectory_points stays dense every accepted sample; only the
        Polyscope redraw (_update_trajectory_curve, a full re-registration)
        is throttled to every TRAJECTORY_CURVE_RENDER_STRIDE samples --
        roadmap Stage5_README.md 5.9 treats the curve as a decimatable debug
        overlay, not the exported path."""
        if not self.trajectory_enabled:
            return

        now = time.time()
        if now - self._last_sample_time < TRAJECTORY_SAMPLE_INTERVAL_S:
            return
        self._last_sample_time = now

        if self.trajectory_points and np.allclose(self.tcp_world, self.trajectory_points[-1]):
            return

        self.trajectory_points.append(self.tcp_world.copy())
        self._trajectory_curve_sample_count += 1
        if self._trajectory_curve_sample_count >= TRAJECTORY_CURVE_RENDER_STRIDE:
            self._trajectory_curve_sample_count = 0
            self._update_trajectory_curve()


    def _update_trajectory_curve(self):
        """Re-register the curve network -- Polyscope curve networks don't support
        growing node count in place, unlike update_vertex_positions."""
        nodes = np.array(self.trajectory_points)
        if len(nodes) < 2:
            return
        edges = np.array([[i, i + 1] for i in range(len(nodes) - 1)])
        self.trajectory_handle = ps.register_curve_network("Trajectory", nodes, edges)
        self.trajectory_handle.set_radius(TRAJECTORY_RADIUS_MM, relative=False)


    def set_trajectory_enabled(self, enabled):
        """Pause/resume TCP trajectory recording and show/hide the existing curve."""
        self.trajectory_enabled = enabled
        if self.trajectory_handle is not None:
            self.trajectory_handle.set_enabled(enabled)


    def clear_trajectory(self):
        """Discard recorded TCP trajectory points and remove the curve, if
        any -- emptying trajectory_points alone isn't enough, since
        _update_trajectory_curve() only re-registers when there are >=2
        points, which would otherwise leave the stale curve on screen."""
        self.trajectory_points = []
        self.trajectory_handle = None
        self._trajectory_curve_sample_count = 0
        ps.remove_curve_network("Trajectory", error_if_absent=False)



def _bbox_corners(verts):
    """The 8 corners of verts' axis-aligned bounding box, in whatever frame
    verts is already in. Used to cheaply bound a mesh's rotated extent
    without transforming every vertex -- see moving_geometry_bbox_min_z."""
    lo, hi = verts.min(axis=0), verts.max(axis=0)
    xs, ys, zs = np.meshgrid([lo[0], hi[0]], [lo[1], hi[1]], [lo[2], hi[2]], indexing='ij')
    return np.stack([xs.ravel(), ys.ravel(), zs.ravel()], axis=1)


def dh_transform(a, alpha, d, theta):
    """Standard DH homogeneous transform, frame {i-1} -> {i}"""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,   sa,       ca,      d],
        [0,   0,        0,       1],
    ])


# Bare rotation matrices used by solve_ik/solve_ik_tcp -- pure stateless
# helpers, same footing as dh_transform (see settled.md S1.1).
def rot_x(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


# Validation
if __name__ == "__main__":
    ps.init()
    vis = VisContent()
    vis.end_effector_position([0, 0, 0, 0, 0, 0])
    print(f"[Backend] Loaded {len(vis.mesh_data)} link meshes")

    gcode_waypoints = vis.parse_gcode(os.path.join(GCODE_DIR, GCODE_FILE))
    print(f"[Backend] Parsed {len(gcode_waypoints)} G-code waypoints")
    vis.load_gcode()

