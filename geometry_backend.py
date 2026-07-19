import os
import re
import json
import time
import heapq
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

PLAYBACK_RENDER_STRIDE = 50  # Push arm/bead updates to Polyscope every Nth
# solved waypoint, not every frame -- full-buffer re-uploads make coarser
# pushes cut real GPU cost, not just Python-side work.

TRAJECTORY_CURVE_RENDER_STRIDE = 5  # Re-register the "Trajectory" curve
# network every Nth recorded sample -- it has no incremental grow API, so
# this throttles how often the O(n) rebuild fires.

PLAYBACK_LOOKAHEAD_BEADS = 5000  # How far ahead of current progress the
# registered "G-code Print" mesh is grown, in beads -- render cost scales
# with registered mesh size, so this stays close to actual progress instead
# of registering the full mesh from frame 1.

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

GCODE_DIR = "assets/models/planar/gcode"
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

CURVED_MODEL_DIR = "assets/models/curved"
CURVED_RX_FILES = [f"RX_{i}.ply" for i in range(28)]  # RX_0..RX_27, all on Surface_RX_Offset
CURVED_TX_FILES = [f"TX_{i}.ply" for i in range(27)]  # TX_0..TX_26, all on Surface_TX_Base
CURVED_SURFACE_RX_OFFSET_FILE = "Surface_RX_Offset.obj"
CURVED_SURFACE_TX_BASE_FILE = "Surface_TX_Base.obj"
CURVED_SURFACE_BOT_FILE = "Surface_Bot.obj"  # underlying shoulder body, not a print surface -- collision body in 6.5

# Float export noise keeps true duplicate vertices apart past ~3dp -- verified
# on RX_0.ply (108 raw verts -> exactly 54 nodes, matching the asset survey).
CURVE_DEDUPE_DECIMALS = 3

CURVED_MODEL_ROTATE_X_DEG = 90.0  # CAD "+z up" assumption was wrong (Stage6_README.md
# open question) -- +90 about the plate's local X puts the printable ridge surface
# face-up; -90 was tested and puts it face-down into the plate, confirmed wrong.

RX_CURVE_COLOR = (0.85, 0.15, 0.15)  # red
TX_CURVE_COLOR = (0.15, 0.35, 0.85)  # blue
CURVE_RADIUS_MM = 0.5  # thin vs. TRAJECTORY_RADIUS_MM (2.0) -- 70 pieces shouldn't dominate the view
SURFACE_RX_OFFSET_COLOR = (0.93, 0.80, 0.80)  # pale rose, curves read clearly on top
SURFACE_TX_BASE_COLOR = (0.80, 0.85, 0.93)    # pale blue
SURFACE_BOT_COLOR = (0.55, 0.55, 0.55)        # neutral gray, not a print target

# Geodesic routing over the print surfaces -- roadmap 6.2. RX and TX are
# separate passes on separate surfaces (settled.md S1.30), so every geodesic
# structure is a 2-element list indexed by these rather than an _rx/_tx pair.
GEODESIC_LAYER_RX = 0
GEODESIC_LAYER_TX = 1
GEODESIC_LAYER_NAMES = ("RX", "TX")

GEODESIC_CHUNK_SOURCES = 1  # whole Dijkstra sources solved per step() call.
# Measured per source: ~50ms on Surface_RX_Offset (30,284 verts), ~85ms on
# Surface_TX_Base (45,430 verts / 135,518 edges) -- so ~12-20fps while running
# and ~8.4-9.1s wall for the full 113-source job. One whole source is the
# chunk granularity because sub-source chunking would mean carrying a live
# heap plus partial dist/prev across frames -- real complexity for a job that
# finishes in seconds.

GEODESIC_CURVE_COLOR = (0.10, 0.80, 0.20)  # green -- distinct from RX red, TX blue, and the pale surfaces
GEODESIC_CHORD_COLOR = (0.90, 0.20, 0.85)  # magenta -- the straight-line comparison, verification only
GEODESIC_CURVE_RADIUS_MM = 1.5  # 3x CURVE_RADIUS_MM so a geodesic reads over the 70 toolpath curves
GEODESIC_HOST_TRANSPARENCY = 0.55  # host surface is ghosted while a sample geodesic is shown, so
# the path reads against the shell instead of being buried in it -- see _isolate_geodesic_layer()

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

        # Chunked toolpath IK precompute state, see run_toolpath_ik_precompute()
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

        # Progressive-reveal playback state -- playback_index persists across
        # pause, only reset_toolpath_playback() zeroes it.
        self.playback_running = False
        self.playback_index = 0
        self._last_rendered_playback_index = 0  # Throttles the Polyscope push in advance_toolpath_playback, see PLAYBACK_RENDER_STRIDE
        self.playback_status = ""
        self.playback_waiting = False  # True when caught up to precompute's frontier but it isn't exhausted yet, see advance_toolpath_playback()
        self.gcode_bead_verts_full = None       # (K*8,3) world space, real bead positions
        self.gcode_bead_faces = None
        self.gcode_bead_reveal_index = None     # (K,) sorted ascending, see _build_gcode_beads
        self.gcode_bead_face_prefix = None      # (K+1,) cumulative triangle count, see _build_gcode_beads
        self.gcode_bead_verts_current = None    # (K*8,3) working copy, mutated as beads reveal
        self.gcode_print_handle = None          # Polyscope handle, reused across advance() calls
        self.gcode_preview_loaded = False       # True only while the static preview (not playback) owns "G-code Print"
        self._registered_bead_capacity = 0      # How many beads are actually registered
        # with Polyscope right now, see PLAYBACK_LOOKAHEAD_BEADS

        self.curved_model_loaded = False  # True once load_curved_model() has registered its structures -- roadmap 6.1/6.6

        # Retained curved-model geometry, world coordinates (already through
        # T_curved) -- roadmap 6.2 needs the per-piece curves and the print
        # surfaces that 6.1 previously computed and threw away. Both lists are
        # indexed by GEODESIC_LAYER_RX/GEODESIC_LAYER_TX. Surface_Bot is
        # deliberately absent: it's a 6.5 collision body, not a print surface.
        self.curved_pieces_world = None        # list of 2 lists of (Ni,3) polylines
        self.curved_surface_verts_world = None # list of 2 (V,3)
        self.curved_surface_faces = None       # list of 2 (F,3), placement-invariant
        self.T_curved = None                   # (4,4) placement actually used, for the staleness check
        self._T_user_frame_at_curved_load = None  # Plate pose the world state above was built against

        # Chunked geodesic precompute state, see run_geodesic_precompute()
        self.geodesic_running = False
        self.geodesic_index = 0
        self.geodesic_total = 0
        self.geodesic_status = ""
        self.geodesic_loaded = False       # True only once both cost matrices are complete -- what 6.3 gates on
        self.geodesic_graphs = None        # list of 2 CSR triples; None/not-None is the start-fresh/resume sentinel
        self.geodesic_snap_nodes = None    # list of 2 (70,) endpoint -> vertex id on its own surface
        self.geodesic_snap_dist = None     # list of 2 (70,) snap distances, evidence the snap is legitimate
        self.geodesic_sources = None       # list of 2 (S,) unique snapped vertex ids
        self.geodesic_source_row = None    # list of 2 (70,) endpoint -> row in sources/prev
        self.geodesic_queue = None         # list of (layer, row) tuples, the flat work list
        self.geodesic_prev = None          # list of 2 (S,V) int32 predecessor rows -- makes any path a walk-back
        self.geodesic_cost = None          # list of 2 (70,70) float64, inf = unreachable
        self.geodesic_unreachable = None   # list of 2 ints, count of inf entries per matrix
        self._geodesic_isolation_prior = None  # {structure_name: was_enabled} while a sample is isolated

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
        PLATE_THICKNESS_MM before the transform to compensate.

        Also invalidates any in-memory toolpath precompute/playback solved
        against the plate's previous pose -- roadmap Stage5_README.md 5.11,
        settled.md S1.22. The cross-session disk cache (5.10/S1.21) already
        keys on pose, but that's only checked once, at the start of a fresh
        precompute; without this, resuming/replaying an already-loaded
        precompute after a plate move would silently drive the arm through
        the old pose's joint path."""
        roll, pitch, yaw = np.deg2rad(rpy_deg)
        R = rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)

        self.T_user_frame = np.eye(4)
        self.T_user_frame[:3, :3] = R
        self.T_user_frame[:3, 3] = position_mm

        if self.precompute_cache_meta is not None:
            new_pose = np.round(self.T_user_frame, 6).tolist()
            if new_pose != self.precompute_cache_meta["user_frame"]:
                self.cancel_toolpath_ik_precompute()
                self.precompute_status = "Build plate moved -- precompute invalidated, run again"

        if self.T_curved is not None and not np.allclose(self.T_user_frame, self._T_user_frame_at_curved_load):
            # The geodesic *costs* survive a plate move -- distances in mm are
            # rigid-motion invariant. The retained world vertices and the
            # stored paths do not, and 6.3 consumes the paths, not just the
            # numbers, so invalidating is the honest call.
            self._abort_geodesic_precompute()
            self.geodesic_status = "Build plate moved -- geodesics invalidated, reload the curved model"

        plate = self.load_mesh(os.path.join(BUILD_PLATE_DIR, BUILD_PLATE_FILE))
        plate_verts_local = plate.vertices + np.array([0.0, 0.0, PLATE_THICKNESS_MM])
        plate_verts_world = transform_points(self.T_user_frame, plate_verts_local)
        plate_handle = ps.register_surface_mesh("Build Plate", plate_verts_world, plate.faces)
        plate_handle.set_color(PLATE_COLOR)

        self.create_coordinate_frame(scale=USER_FRAME_SCALE_MM, origin=position_mm, rotation=R, name="User Frame")


    def save_build_plate_position(self, position_mm, rpy_deg):
        """Write the given build-plate pose to assets/buildPlate/ so it can
        be recalled later via load_saved_build_plate_position() -- see the
        GUI's "Save Position" button. Only ever called on explicit user
        action, never automatically. Returns a status message; a write
        failure (missing directory, permissions) is reported back rather
        than left to raise out of the button callback, since silently
        claiming success here would be worse than the crash it avoids."""
        data = {
            "position_mm": np.asarray(position_mm, dtype=float).tolist(),
            "rpy_deg": np.asarray(rpy_deg, dtype=float).tolist(),
        }
        try:
            with open(BUILD_PLATE_POSITION_FILE, "w") as f:
                json.dump(data, f, indent=2)
        except OSError as e:
            return f"Failed to save position: {e}"
        return "Position saved"


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

        verts_world = transform_points(self.T_user_frame, verts_local)

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

        try:
            waypoints = self.parse_gcode(filepath)
        except OSError:
            # File can be overwritten mid-read by a Cura re-export between
            # the exists() check above and here -- no-op like the missing-
            # file case rather than crashing the per-frame callback.
            return
        if len(waypoints) < 2:
            return

        verts_world, faces, _reveal_waypoint_index, _bead_face_prefix = self._build_gcode_beads(waypoints)
        if len(verts_world) == 0:
            return

        self.gcode_print_handle = ps.register_surface_mesh("G-code Print", verts_world, faces)
        self.gcode_print_handle.set_color(GCODE_COLOR)
        self.gcode_preview_loaded = True


    def clear_gcode_preview(self):
        """Mirrors the GUI's toggled "Clear G-code preview" button: removes
        the "G-code Print" mesh and resets playback state, since the same
        structure is reused for the playback reveal animation -- leaving
        playback_index pointed at now-discarded bead arrays would break the
        next Run Toolpath. Safe to call with nothing loaded."""
        self._reset_toolpath_playback_state()


    def _register_curve_layer(self, name, pieces_local, T_placement, color):
        """Combine one layer's reconstructed pieces (open + closed) into a
        single Polyscope curve network -- one structure per layer, not per
        piece, so a future layer toggle (roadmap 6.6) can show/hide a whole
        pass at once."""
        nodes_local, edge_blocks, offset = [], [], 0
        for piece in pieces_local:
            n = len(piece)
            nodes_local.append(piece)
            edge_blocks.append(np.column_stack([np.arange(n - 1), np.arange(1, n)]) + offset)
            offset += n
        nodes_world = transform_points(T_placement, np.vstack(nodes_local))
        handle = ps.register_curve_network(name, nodes_world, np.vstack(edge_blocks))
        handle.set_color(color)
        handle.set_radius(CURVE_RADIUS_MM, relative=False)
        return handle


    def load_curved_model(self):
        """Load the 55 toolpath-curve PLY files and 3 surface OBJ meshes from
        CURVED_MODEL_DIR and place them above the build plate -- roadmap
        Stage6_README.md 6.1. Static workpiece geometry, same as
        load_build_plate()/load_gcode(): one-time T_user_frame multiply, no
        Delta transform (settled.md S1.2/S1.3). Safe to call repeatedly;
        Polyscope replaces the prior structures of the same names.

        Placement is translation plus one fixed rotation (Stage6_README.md's
        Open Questions: the CAD "+z up" assumption was unverified and turned
        out wrong -- CURVED_MODEL_ROTATE_X_DEG about local X puts the
        printable ridge surface face-up, confirmed by which side the
        surface faced when tested at +90 vs -90): rotate the raw-local
        points about the CAD-local origin, then center the rotated
        assembly's XY bbox over the plate mesh's own local XY bbox-center
        and lift so its lowest point sits at the plate-local print surface
        -- z=0 after the same PLATE_THICKNESS_MM compensation
        load_build_plate()/build_toolpath_waypoints_world() already apply
        (position_mm marks the plate's resting/bottom face, not its top).

        Retains the placed geometry in world coordinates
        (curved_pieces_world/curved_surface_verts_world/curved_surface_faces/
        T_curved) for roadmap 6.2's geodesic routing, which needs the
        per-piece curves and the two print surfaces in the frame the arm
        works in. Surface_Bot is rendered but not retained -- it's a
        collision body for 6.5, not a print surface."""
        # Every world vertex below is about to be re-derived, so any geodesic
        # solved against the previous load -- in flight or complete -- describes
        # geometry that no longer exists.
        self._abort_geodesic_precompute()
        self.geodesic_status = ""

        rx_pieces_local = [p for f in CURVED_RX_FILES
                            for p in reconstruct_polylines(*read_ply_polyline(os.path.join(CURVED_MODEL_DIR, f)))]
        tx_pieces_local = [p for f in CURVED_TX_FILES
                            for p in reconstruct_polylines(*read_ply_polyline(os.path.join(CURVED_MODEL_DIR, f)))]

        surface_rx = self.load_mesh(os.path.join(CURVED_MODEL_DIR, CURVED_SURFACE_RX_OFFSET_FILE))
        surface_tx = self.load_mesh(os.path.join(CURVED_MODEL_DIR, CURVED_SURFACE_TX_BASE_FILE))
        surface_bot = self.load_mesh(os.path.join(CURVED_MODEL_DIR, CURVED_SURFACE_BOT_FILE))

        R = rot_x(np.deg2rad(CURVED_MODEL_ROTATE_X_DEG))[:3, :3]

        def rotate(pts):
            return pts @ R.T

        rx_pieces_local = [rotate(p) for p in rx_pieces_local]
        tx_pieces_local = [rotate(p) for p in tx_pieces_local]
        surface_rx_verts = rotate(surface_rx.vertices)
        surface_tx_verts = rotate(surface_tx.vertices)
        surface_bot_verts = rotate(surface_bot.vertices)

        all_local = np.vstack(rx_pieces_local + tx_pieces_local
                               + [surface_rx_verts, surface_tx_verts, surface_bot_verts])
        assembly_min, assembly_max = all_local.min(axis=0), all_local.max(axis=0)

        plate = self.load_mesh(os.path.join(BUILD_PLATE_DIR, BUILD_PLATE_FILE))
        plate_min, plate_max = plate.bounds

        T_placement = np.eye(4)
        T_placement[:2, 3] = (plate_min[:2] + plate_max[:2]) / 2.0 - (assembly_min[:2] + assembly_max[:2]) / 2.0
        T_placement[2, 3] = -assembly_min[2] + PLATE_THICKNESS_MM
        T_curved = self.T_user_frame @ T_placement

        # Transform once, then both retain and render the same arrays -- 6.2
        # routes over these surfaces and needs them in the frame the arm works
        # in, so world is computed here rather than at each registration site.
        rx_pieces_world = [transform_points(T_curved, p) for p in rx_pieces_local]
        tx_pieces_world = [transform_points(T_curved, p) for p in tx_pieces_local]
        surface_rx_world = transform_points(T_curved, surface_rx_verts)
        surface_tx_world = transform_points(T_curved, surface_tx_verts)
        surface_bot_world = transform_points(T_curved, surface_bot_verts)

        self._register_curve_layer("Curved Toolpath RX", rx_pieces_world, np.eye(4), RX_CURVE_COLOR)
        self._register_curve_layer("Curved Toolpath TX", tx_pieces_world, np.eye(4), TX_CURVE_COLOR)

        for name, verts_world, mesh, color in (
            ("Surface RX Offset", surface_rx_world, surface_rx, SURFACE_RX_OFFSET_COLOR),
            ("Surface TX Base", surface_tx_world, surface_tx, SURFACE_TX_BASE_COLOR),
            ("Surface Bot", surface_bot_world, surface_bot, SURFACE_BOT_COLOR),
        ):
            handle = ps.register_surface_mesh(name, verts_world, mesh.faces)
            handle.set_color(color)

        self.curved_pieces_world = [rx_pieces_world, tx_pieces_world]
        self.curved_surface_verts_world = [surface_rx_world, surface_tx_world]
        self.curved_surface_faces = [np.asarray(surface_rx.faces), np.asarray(surface_tx.faces)]
        self.T_curved = T_curved
        self._T_user_frame_at_curved_load = self.T_user_frame.copy()
        self.curved_model_loaded = True


    def _layer_endpoints_world(self, layer):
        """The 70 curve endpoints of one layer, in the order roadmap 6.3
        indexes them: endpoint 2p is pieces[p][0] and 2p+1 is pieces[p][-1],
        for p over curved_pieces_world[layer].

        The 3 closed-loop pieces per layer (RX_0/22/27, TX_2/6/17) have
        coincident ends, so their two endpoints are the same point -- they
        snap to one vertex and their cost-matrix rows come out identical,
        with cost[2p, 2p+1] == 0.0. That is correct, but 6.3's ordering must
        not read that zero as free travel to somewhere else."""
        return np.array([e for piece in self.curved_pieces_world[layer]
                          for e in (piece[0], piece[-1])])


    def run_geodesic_precompute(self):
        """Start or resume the chunked geodesic precompute -- roadmap
        Stage6_README.md 6.2. Mirrors run_toolpath_ik_precompute(): only
        builds the graphs and resets progress if nothing is loaded yet
        (geodesic_graphs is None -- true on the first call and again after
        cancel_geodesic_precompute()); if a run is merely paused, resumes
        stepping from geodesic_index with no rebuild.

        Builds one graph per print surface, not one merged graph: RX travels
        on Surface_RX_Offset and TX on Surface_TX_Base, the passes never
        interleave (settled.md S1.30), and a geodesic between an RX and a TX
        endpoint is meaningless on either mesh.

        One Dijkstra runs per *unique snapped vertex*, not per endpoint --
        measured 58 unique for RX and 55 for TX rather than 70 each, since
        distinct endpoints often land on the same vertex, so this is 113
        runs and not the 140 the roadmap assumed."""
        if self.geodesic_graphs is None:
            if not self.curved_model_loaded:
                # Fail with a status message, never an exception: this runs
                # from a button inside the per-frame Polyscope callback.
                self.geodesic_status = "Load Curved Model first"
                return

            graphs, snap_nodes, snap_dist, sources, source_row, prev, cost = [], [], [], [], [], [], []
            for layer in (GEODESIC_LAYER_RX, GEODESIC_LAYER_TX):
                verts = self.curved_surface_verts_world[layer]
                graphs.append(build_surface_graph(verts, self.curved_surface_faces[layer]))

                idx, dist = nearest_vertex_index(verts, self._layer_endpoints_world(layer))
                snap_nodes.append(idx)
                snap_dist.append(dist)

                uniq, row = np.unique(idx, return_inverse=True)
                sources.append(uniq)
                source_row.append(row)

                prev.append(np.full((len(uniq), len(verts)), -1, dtype=np.int32))
                cost.append(np.full((len(idx), len(idx)), np.inf))

            self.geodesic_graphs = graphs
            self.geodesic_snap_nodes = snap_nodes
            self.geodesic_snap_dist = snap_dist
            self.geodesic_sources = sources
            self.geodesic_source_row = source_row
            self.geodesic_prev = prev
            self.geodesic_cost = cost
            self.geodesic_unreachable = [0, 0]
            self.geodesic_queue = [(layer, r)
                                    for layer in (GEODESIC_LAYER_RX, GEODESIC_LAYER_TX)
                                    for r in range(len(sources[layer]))]
            self.geodesic_index = 0
            self.geodesic_total = len(self.geodesic_queue)
            self.geodesic_loaded = False

        self.geodesic_running = True
        self.geodesic_status = f"Building geodesics {self.geodesic_index}/{self.geodesic_total} sources"


    def pause_geodesic_precompute(self):
        """Stop advancing the geodesic precompute without discarding
        progress. A following run_geodesic_precompute() continues from
        geodesic_index."""
        self.geodesic_running = False


    def cancel_geodesic_precompute(self):
        """Stop and discard the geodesic precompute entirely -- a following
        run_geodesic_precompute() starts fresh."""
        self._abort_geodesic_precompute()
        self.geodesic_status = ""


    def _abort_geodesic_precompute(self):
        """Shared discard used by cancel_geodesic_precompute() and
        load_curved_model(). Resets geodesic_index/total together, so a stale
        index can't outlive the arrays it counted (the same failure
        _abort_toolpath_ik_precompute() guards against, settled.md S1.24),
        and removes the sample curves, which render vertex arrays this just
        dropped. Does not touch geodesic_status, so a caller can set an
        explanatory message first."""
        self.geodesic_running = False
        self.geodesic_loaded = False
        self.geodesic_index = 0
        self.geodesic_total = 0
        self.geodesic_graphs = None
        self.geodesic_snap_nodes = None
        self.geodesic_snap_dist = None
        self.geodesic_sources = None
        self.geodesic_source_row = None
        self.geodesic_queue = None
        self.geodesic_prev = None
        self.geodesic_cost = None
        self.geodesic_unreachable = None
        ps.remove_curve_network("Geodesic Sample", error_if_absent=False)
        ps.remove_curve_network("Geodesic Chord", error_if_absent=False)
        self._restore_geodesic_isolation()


    def step_geodesic_precompute(self):
        """Advance the in-progress geodesic precompute by up to
        GEODESIC_CHUNK_SOURCES whole Dijkstra sources -- call every frame
        from render(). No-ops unless geodesic_running.

        Unlike step_toolpath_ik_precompute() there is no abort-on-failure
        branch, deliberately: Dijkstra cannot fail. An unreachable target is
        data (inf), not an error, and discarding a 70x70 matrix because one
        pair sits on a disconnected mesh fragment would throw away the 4,830
        other off-diagonal entries alongside it."""
        if not self.geodesic_running:
            return

        end = min(self.geodesic_index + GEODESIC_CHUNK_SOURCES, self.geodesic_total)
        for i in range(self.geodesic_index, end):
            layer, row = self.geodesic_queue[i]
            start, nbr, weight = self.geodesic_graphs[layer]
            dist, prev = dijkstra_surface(start, nbr, weight, int(self.geodesic_sources[layer][row]))
            self.geodesic_prev[layer][row] = prev

            # One solve fills every endpoint row that snapped to this vertex,
            # so the duplicate rows cost nothing and can't disagree with their
            # twin. Only the 70 endpoint columns are kept -- retaining full
            # dist rows would double the memory for data nothing reads.
            fill = (self.geodesic_source_row[layer] == row)
            self.geodesic_cost[layer][fill, :] = dist[self.geodesic_snap_nodes[layer]]

            if row == 0:
                # The first solve of a layer already covers the whole graph,
                # so it is the reachability oracle -- no separate flood fill
                # needed. Pausing (not just setting status) is what makes the
                # warning readable: a plain status here would be overwritten
                # by "Building geodesics N/M" one frame later. Build resumes
                # the run through the normal pause/resume path.
                n_bad = int(np.isinf(dist[self.geodesic_snap_nodes[layer]]).sum())
                if n_bad:
                    self.geodesic_index = i + 1
                    self.geodesic_running = False
                    self.geodesic_status = (f"{GEODESIC_LAYER_NAMES[layer]}: {n_bad}/"
                                             f"{len(self.geodesic_snap_nodes[layer])} endpoints unreachable "
                                             f"-- surface is fragmented (Build Geodesics resumes)")
                    return

        self.geodesic_index = end

        if self.geodesic_index >= self.geodesic_total:
            self.geodesic_running = False
            self.geodesic_loaded = True
            self.geodesic_unreachable = [int(np.isinf(c).sum()) for c in self.geodesic_cost]
            if any(self.geodesic_unreachable):
                self.geodesic_status = (f"Geodesics ready with {sum(self.geodesic_unreachable)} "
                                         f"unreachable pair(s) -- surface is fragmented")
            else:
                spans = [c[np.isfinite(c)].max() for c in self.geodesic_cost]
                self.geodesic_status = (f"Geodesics ready -- RX max {spans[GEODESIC_LAYER_RX]:.0f}mm, "
                                         f"TX max {spans[GEODESIC_LAYER_TX]:.0f}mm")
        else:
            self.geodesic_status = f"Building geodesics {self.geodesic_index}/{self.geodesic_total} sources"


    def _pick_sample_pair(self, layer, mode):
        """Choose two endpoints to demonstrate a geodesic with. Returns
        (endpoint_a, endpoint_b), or None if no valid pair exists.

        Both modes must exclude two classes of pair that a bare argmin/argmax
        would happily return:
          - the two ends of the *same* piece (that isn't a travel move), and
          - zero-cost pairs, where both endpoints snapped to one vertex
            (settled.md S1.31). Those reconstruct to a single-node "path".
            Counted as matrix entries (both (i,j) and (j,i)) there are 16 on
            RX and 18 on TX between *different* pieces -- i.e. 8 and 9
            distinct pairs.

        mode="representative" -- the most curved of the travel moves 6.3 will
        actually emit, i.e. over each endpoint's nearest other-piece endpoint,
        the one with the highest geodesic/chord ratio. Measured RX 14->43,
        26.1mm, ratio 1.11. The naive alternatives are both misleading: the
        *farthest* pair (317mm) is a traversal 6.3 will never emit, and the
        *shortest* hop is 2.95mm over 3 nodes at ratio 1.000 -- a straight
        line, because a curved surface is locally flat at that scale.

        mode="most_curved" -- the highest ratio at any distance (RX 48->6,
        250mm, ratio 1.72), for the "does it chord through the shell"
        question.

        Both defaults are deliberately chosen outliers, not typical: the
        median ratio is 1.08 over all ~4,744 valid pairs and ~1.003 over
        realistic hops, so most travel moves are very nearly straight."""
        cost = self.geodesic_cost[layer]
        n = cost.shape[0]
        verts = self.curved_surface_verts_world[layer][self.geodesic_snap_nodes[layer]]
        chord = np.linalg.norm(verts[:, None, :] - verts[None, :, :], axis=-1)

        piece = np.arange(n) // 2
        valid = (piece[:, None] != piece[None, :]) & (cost > 1e-9) & np.isfinite(cost)
        if not valid.any():
            return None
        ratio = np.where(valid, cost / np.maximum(chord, 1e-9), 0.0)

        if mode == "most_curved":
            return tuple(int(v) for v in np.unravel_index(np.argmax(ratio), ratio.shape))

        # Restrict to each endpoint's nearest other-piece endpoint -- the set
        # of hops a greedy nearest-endpoint chain would actually consider.
        best = None
        for i in range(n):
            cand = np.where(valid[i])[0]
            if not len(cand):
                continue
            j = int(cand[np.argmin(cost[i, cand])])
            if best is None or ratio[i, j] > ratio[best[0], best[1]]:
                best = (i, j)
        return best


    def show_sample_geodesic(self, layer=GEODESIC_LAYER_RX, mode="representative",
                              endpoint_a=None, endpoint_b=None):
        """Render one geodesic on its own surface -- roadmap 6.2's Verify
        step. See _pick_sample_pair() for how the default pair is chosen.

        Isolates the host surface first, because otherwise this shows
        nothing: Surface_TX_Base sits a uniform ~2mm *outside*
        Surface_RX_Offset (which is itself ~2mm outside Surface_Bot), so an
        RX geodesic renders sealed inside the TX shell and is invisible. The
        only thing that stayed visible was the straight comparison chord
        leaving the surface into open space, which read as a broken
        geodesic. Prior visibility is snapshotted and restored on clear, so
        this isn't a one-way trip through the user's view settings.

        The chord is drawn only in mode="most_curved", where the comparison
        is the point -- at representative scale it overlaps the geodesic
        almost exactly and adds only clutter."""
        if not self.geodesic_loaded:
            self.geodesic_status = "Build geodesics first"
            return

        cost = self.geodesic_cost[layer]
        if endpoint_a is None or endpoint_b is None:
            pair = self._pick_sample_pair(layer, mode)
            if pair is None:
                self.geodesic_status = f"{GEODESIC_LAYER_NAMES[layer]}: no valid sample pair"
                return
            endpoint_a, endpoint_b = pair
        endpoint_a, endpoint_b = int(endpoint_a), int(endpoint_b)

        row = int(self.geodesic_source_row[layer][endpoint_a])
        nodes = geodesic_path_nodes(self.geodesic_prev[layer][row],
                                     int(self.geodesic_snap_nodes[layer][endpoint_b]))
        if nodes is None:
            self.geodesic_status = f"{GEODESIC_LAYER_NAMES[layer]} {endpoint_a}->{endpoint_b}: unreachable"
            return
        if len(nodes) < 2:
            # Only reachable with manual endpoint args -- _pick_sample_pair()
            # excludes zero-cost pairs, but a caller-supplied pair snapping to
            # one vertex would hand register_curve_network a 1-node polyline.
            self.geodesic_status = (f"{GEODESIC_LAYER_NAMES[layer]} {endpoint_a}->{endpoint_b}: "
                                     f"endpoints snap to the same vertex -- no path to show")
            return

        self._isolate_geodesic_layer(layer)

        pts = self.curved_surface_verts_world[layer][nodes]  # already world, no transform needed
        handle = self._register_curve_layer("Geodesic Sample", [pts], np.eye(4), GEODESIC_CURVE_COLOR)
        handle.set_radius(GEODESIC_CURVE_RADIUS_MM, relative=False)

        # Drop any previous chord before deciding whether to draw one: without
        # this, switching most_curved -> representative leaves the old pair's
        # chord on screen next to an unrelated path -- exactly the "looks like
        # a broken geodesic" state this whole aid was rewritten to avoid.
        ps.remove_curve_network("Geodesic Chord", error_if_absent=False)

        chord_len = float(np.linalg.norm(pts[-1] - pts[0]))
        if mode == "most_curved":
            chord = np.array([pts[0], pts[-1]])
            chord_handle = ps.register_curve_network("Geodesic Chord", chord, np.array([[0, 1]]))
            chord_handle.set_color(GEODESIC_CHORD_COLOR)
            chord_handle.set_radius(GEODESIC_CURVE_RADIUS_MM, relative=False)

        # Report the ratio, not just the length: it's what makes "this hugs
        # the surface" checkable when the picture alone is ambiguous.
        length = float(cost[endpoint_a, endpoint_b])
        self.geodesic_status = (f"{GEODESIC_LAYER_NAMES[layer]} {endpoint_a}->{endpoint_b} ({mode}): "
                                 f"{length:.1f}mm over {len(nodes)} nodes vs {chord_len:.1f}mm chord "
                                 f"-- ratio {length / max(chord_len, 1e-9):.3f}")


    def _isolate_geodesic_layer(self, layer):
        """Hide everything that would occlude a geodesic on `layer`'s surface
        and ghost the host surface, snapshotting prior visibility into
        _geodesic_isolation_prior so _restore_geodesic_isolation() can put it
        all back. Re-entrant: an existing snapshot is left alone, so showing
        two samples in a row doesn't record the already-isolated state as if
        it were the user's."""
        host = f"Surface {'RX Offset' if layer == GEODESIC_LAYER_RX else 'TX Base'}"
        other = f"Surface {'TX Base' if layer == GEODESIC_LAYER_RX else 'RX Offset'}"

        if self._geodesic_isolation_prior is None:
            # Snapshot transparency alongside enabled state: the host gets
            # ghosted below, and restoring a hardcoded 1.0 would silently
            # undo any transparency the user had set themselves.
            prior = {}
            for name in (host, other, "Surface Bot"):
                if ps.has_surface_mesh(name):
                    h = ps.get_surface_mesh(name)
                    prior[name] = (h.is_enabled(), h.get_transparency())
            for name in ("Curved Toolpath RX", "Curved Toolpath TX"):
                if ps.has_curve_network(name):
                    prior[name] = (ps.get_curve_network(name).is_enabled(), None)
            self._geodesic_isolation_prior = prior

        for name in (other, "Surface Bot"):
            if ps.has_surface_mesh(name):
                ps.get_surface_mesh(name).set_enabled(False)
        if ps.has_surface_mesh(host):
            h = ps.get_surface_mesh(host)
            h.set_enabled(True)
            h.set_transparency(GEODESIC_HOST_TRANSPARENCY)

        other_curves = f"Curved Toolpath {'TX' if layer == GEODESIC_LAYER_RX else 'RX'}"
        if ps.has_curve_network(other_curves):
            ps.get_curve_network(other_curves).set_enabled(False)


    def _restore_geodesic_isolation(self):
        """Put back the visibility _isolate_geodesic_layer() changed, from its
        snapshot -- never to hardcoded defaults, which would clobber
        transparency the user set themselves on a structure isolation only
        ever enabled/disabled. Safe to call with nothing isolated."""
        if self._geodesic_isolation_prior is None:
            return
        for name, (was_enabled, was_transparency) in self._geodesic_isolation_prior.items():
            if ps.has_surface_mesh(name):
                h = ps.get_surface_mesh(name)
                h.set_enabled(was_enabled)
                h.set_transparency(was_transparency)
            elif ps.has_curve_network(name):
                ps.get_curve_network(name).set_enabled(was_enabled)
        self._geodesic_isolation_prior = None


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
        pts_world = transform_points(self.T_user_frame, pts_local)
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
            cached_meta = json.loads(cached["meta"].item())
            if cached_meta != self._toolpath_cache_meta(self.T_user_frame):
                return False
            joint_path = cached["joint_path"].astype(float)
        except Exception:
            return False

        self.precompute_joint_path = list(joint_path)
        self.precompute_index = len(joint_path)
        self.precompute_total = len(joint_path)
        self.precompute_running = False
        self.precompute_status = f"Loaded {len(joint_path)} waypoint(s) from cache"
        # Record the pose this load matched, so a later plate move can be
        # detected as staleness (roadmap 5.11, settled.md S1.22) even though
        # this path skipped run_toolpath_ik_precompute()'s own snapshot.
        self.precompute_cache_meta = cached_meta
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

            # filepath can be overwritten mid-read by a Cura re-export
            # between the exists() check above and here -- fail closed with
            # a status message rather than letting the exception escape the
            # per-frame Polyscope callback (settled.md notes model.gcode
            # "gets overwritten by each new Cura export").
            try:
                gcode_points = self.parse_gcode(filepath)
                waypoints, R_target = self.build_toolpath_waypoints_world(gcode_points)
                cache_meta = self._toolpath_cache_meta(self.T_user_frame)
            except OSError:
                self.precompute_status = "G-code file changed while loading -- try again"
                return
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
            self.precompute_cache_meta = cache_meta

        self.precompute_running = True
        self.precompute_status = f"Precomputing {self.precompute_index}/{self.precompute_total} waypoints"


    def pause_toolpath_ik_precompute(self):
        """Mirrors the GUI's playback Pause button: stop advancing the
        precompute without discarding progress. A following
        run_toolpath_ik_precompute() call continues from precompute_index."""
        self.precompute_running = False


    def cancel_toolpath_ik_precompute(self):
        """Stop and discard the precompute entirely, resetting progress to
        zero -- a following run_toolpath_ik_precompute() call starts fresh."""
        self._abort_toolpath_ik_precompute()
        self.precompute_status = ""


    def _abort_toolpath_ik_precompute(self):
        """Shared discard used by cancel_toolpath_ik_precompute() and
        step_toolpath_ik_precompute()'s failure branches -- resets all
        precompute progress (precompute_index/total included, so a stale
        index can't outlive the joint path it counted) and playback state,
        since playback indexes precompute_joint_path directly and can't be
        left pointing at a joint path this just emptied. Does not touch
        precompute_status, so a caller can set an explanatory message
        first."""
        self.precompute_running = False
        self.precompute_waypoints = None
        self.precompute_index = 0
        self.precompute_total = 0
        self.precompute_joint_path = []
        self.precompute_cache_meta = None
        self._reset_toolpath_playback_state()


    def step_toolpath_ik_precompute(self):
        """Advance the in-progress precompute by up to PRECOMPUTE_CHUNK_SIZE
        waypoints -- call every frame from render(). No-ops unless
        precompute_running. Uses the same per-waypoint solve + ground-
        clearance logic as solve_toolpath_ik, and aborts the whole
        precompute (no partial motion) at the first waypoint with no
        valid/clearing branch."""
        if not self.precompute_running:
            return

        end = min(self.precompute_index + PRECOMPUTE_CHUNK_SIZE, self.precompute_total)
        for i in range(self.precompute_index, end):
            pos_world_mm, _is_feed_move = self.precompute_waypoints[i]
            solutions, status = self.solve_ik_tcp_matrix(
                pos_world_mm, self.precompute_R_target, self.precompute_joint_limits,
                reference_joint_angles=self.precompute_ref)
            if not solutions:
                status_msg = f"Waypoint {i}/{self.precompute_total}: {status}"
                self._abort_toolpath_ik_precompute()
                self.precompute_status = status_msg
                return

            clear = next((angles for angles, *_ in solutions if self._branch_clears_ground(angles)), None)
            if clear is None:
                status_msg = (
                    f"Waypoint {i}/{self.precompute_total}: all {len(solutions)} valid branch(es) "
                    "dip below the ground plane (z<0)")
                self._abort_toolpath_ik_precompute()
                self.precompute_status = status_msg
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


    def _reset_toolpath_playback_state(self):
        """Shared playback reset used by cancel_toolpath_ik_precompute() and
        load_build_plate()'s invalidation branch, both of which discard
        precompute_joint_path that playback indexes into directly. Also
        removes the "G-code Print" mesh so a stale preview doesn't linger
        at the old pose."""
        self.playback_running = False
        self.playback_index = 0
        self.playback_waiting = False
        self.gcode_bead_verts_full = None
        self.playback_status = ""
        self.gcode_print_handle = None
        self.gcode_preview_loaded = False
        ps.remove_surface_mesh("G-code Print", error_if_absent=False)


    def _init_toolpath_playback(self):
        """Shared setup for reset_toolpath_playback() and the first
        run_toolpath_playback() call this session. Requires a completed
        precompute; re-parses the G-code and rebuilds bead geometry via
        _build_gcode_beads() (not load_gcode(), which doesn't return
        reveal_waypoint_index). Collapses every bead to its own first
        corner (zero-area, nothing renders) and registers only the first
        PLAYBACK_LOOKAHEAD_BEADS beads' worth, not the full mesh. Snaps
        the arm to the first waypoint's pose. Returns True on success,
        False (with playback_status explaining why) otherwise."""
        if not self.precompute_joint_path:
            self.playback_status = "Run Precompute first"
            return False

        filepath = os.path.join(GCODE_DIR, GCODE_FILE)
        if not os.path.exists(filepath):
            self.playback_status = "No G-code file found"
            return False

        try:
            gcode_points = self.parse_gcode(filepath)
        except OSError:
            # File can be overwritten mid-read by a Cura re-export between
            # the exists() check above and here.
            self.playback_status = "G-code file changed while loading -- try again"
            return False
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
        # From here "G-code Print" belongs to playback's reveal, not the
        # static preview -- the Clear button should stay hidden until an
        # explicit Load click claims the mesh again.
        self.gcode_preview_loaded = False
        self.gcode_print_handle = ps.register_surface_mesh(
            "G-code Print",
            self.gcode_bead_verts_current[:self._registered_bead_capacity * 8],
            self.gcode_bead_faces[:self.gcode_bead_face_prefix[self._registered_bead_capacity]])
        self.gcode_print_handle.set_color(GCODE_COLOR)

        self.playback_index = 0
        self._last_rendered_playback_index = 0
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
        frame from render(). No-ops unless playback_running. The index
        always advances every call; the Polyscope push (arm pose + bead
        reveal) is throttled to every PLAYBACK_RENDER_STRIDE waypoints,
        forced on the final one so playback never ends on a stale
        mid-stride pose. Beads reveal via a sorted cutoff over
        gcode_bead_reveal_index, accumulated from the last *rendered*
        index so none are skipped across throttled frames. The
        registered mesh grows in PLAYBACK_LOOKAHEAD_BEADS chunks instead
        of registering the full mesh from frame 1.

        Playback may start before precompute finishes: the advance is
        capped at the live frontier (len(precompute_joint_path)), not a
        snapshot. If playback catches the frontier before precompute is
        exhausted, it holds there with a "Waiting for precompute" status
        and playback_running stays True so the next frame rechecks the
        frontier automatically. self.playback_waiting mirrors this state
        -- gui_panel.py reads it to snap the Speed slider down the
        moment playback actually hits the compute limit."""
        if not self.playback_running:
            return

        frontier = len(self.precompute_joint_path)
        if self.playback_index >= frontier:
            # precompute_joint_path shrank or emptied under an active
            # playback -- refuse cleanly instead of an IndexError below.
            self.playback_running = False
            self.playback_waiting = False
            self.playback_status = "Toolpath data changed -- reset playback"
            return

        new_index = min(self.playback_index + step_count, frontier - 1)
        moved = new_index != self.playback_index
        self.playback_index = new_index

        exhausted = self.precompute_index >= self.precompute_total
        at_frontier = self.playback_index >= frontier - 1
        finished = exhausted and at_frontier
        waiting = at_frontier and not exhausted
        self.playback_waiting = waiting

        if finished or (waiting and moved) or self.playback_index - self._last_rendered_playback_index >= PLAYBACK_RENDER_STRIDE:
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
        elif waiting:
            self.playback_status = f"Waiting for precompute ({frontier}/{self.precompute_total} solved)"
        else:
            self.playback_status = f"Playing {self.playback_index}/{self.precompute_total - 1}"


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
        self.T_zero_inv = [np.linalg.inv(T) for T in self.T_zero]  # cached once,
        # not per-frame -- apply_delta_transform()/_moving_geometry_deltas() run every frame

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
        T_zero_flange_inv = self.T_zero_inv[5]
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
            Delta = T_current[src] @ self.T_zero_inv[src]

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
        return [T_current[min(i, 5)] @ self.T_zero_inv[min(i, 5)] for i in range(7)]


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


def transform_points(T, points):
    """Apply a 4x4 homogeneous transform to an Nx3 point array."""
    homo = np.hstack([points, np.ones((len(points), 1))])
    return (T @ homo.T).T[:, :3]


def read_ply_polyline(filepath):
    """Read an ASCII PLY containing only `element vertex` + `element edge`
    (no faces) -- these reject trimesh.load(force='mesh'), which needs
    faces to produce anything but a degenerate empty mesh. Returns
    (verts: Nx3 float64, edges: Mx2 int) exactly as declared in the header;
    edges are a disjoint segment soup in file order, not a walkable curve
    -- see reconstruct_polylines()."""
    with open(filepath) as f:
        lines = f.readlines()
    n_vertex = n_edge = header_end = None
    for i, line in enumerate(lines):
        if line.startswith("element vertex"):
            n_vertex = int(line.split()[-1])
        elif line.startswith("element edge"):
            n_edge = int(line.split()[-1])
        elif line.startswith("end_header"):
            header_end = i + 1
            break
    v_end = header_end + n_vertex
    e_end = v_end + n_edge
    verts = np.loadtxt(lines[header_end:v_end], dtype=np.float64).reshape(n_vertex, 3)
    edges = np.loadtxt(lines[v_end:e_end], dtype=int).reshape(n_edge, 2)
    return verts, edges


def reconstruct_polylines(verts, edges, decimals=CURVE_DEDUPE_DECIMALS):
    """Reassemble a PLY's disjoint edge soup into walkable polyline pieces.
    Coordinates are deduped by rounding -- float export noise keeps true
    duplicate points apart past ~3dp (verified: RX_0.ply's 108 raw vertices
    collapse to exactly 54 nodes at 3dp). A few files also carry one
    degenerate zero-length edge whose endpoints round to the same node --
    dropped, it carries no path information.

    Every node has degree <= 2 (no branching, confirmed across all 55
    files) but not every component has a degree-1 endpoint: 6 files
    (RX_0/RX_22/RX_27/TX_17/TX_2/TX_6) are single closed loops. Open
    components are walked from a degree-1 node; any component with none
    left is a closed loop, walked from an arbitrary node back to itself.
    Verified lossless (every non-self-loop edge consumed exactly once)
    across all 55 files -- 70 pieces total, matching the asset survey.

    Returns a list of Nx3 float arrays, one per piece; closed pieces repeat
    their start point as the last row so every piece can be treated
    identically as a consecutive-edge stride (see _register_curve_layer)."""
    rounded = np.round(verts, decimals)
    uniq, inv = np.unique(rounded, axis=0, return_inverse=True)
    inv = inv.reshape(-1)

    adjacency = {i: set() for i in range(len(uniq))}
    for a, b in edges:
        ua, ub = int(inv[a]), int(inv[b])
        if ua == ub:
            continue
        adjacency[ua].add(ub)
        adjacency[ub].add(ua)

    visited_nodes = set()
    n_edges_consumed = 0

    def walk(start):
        nonlocal n_edges_consumed
        chain = [start]
        visited_nodes.add(start)
        prev, cur = None, start
        while True:
            nbrs = [n for n in adjacency[cur] if n != prev]
            if not nbrs:
                break
            nxt = nbrs[0]
            n_edges_consumed += 1
            chain.append(nxt)
            if nxt == start:
                break
            visited_nodes.add(nxt)
            prev, cur = cur, nxt
        return uniq[chain]

    pieces = [walk(n) for n, adj in adjacency.items() if len(adj) == 1 and n not in visited_nodes]
    pieces += [walk(n) for n in adjacency if n not in visited_nodes]

    n_unique_edges = len({(min(int(a), int(b)), max(int(a), int(b)))
                           for a, b in ((inv[a], inv[b]) for a, b in edges) if a != b})
    assert n_edges_consumed == n_unique_edges, "reconstruct_polylines dropped an edge"
    return pieces


def build_surface_graph(verts, faces):
    """Undirected edge graph of a triangle mesh -- roadmap 6.2 step 1. Nodes
    are mesh vertices, edges are triangle edges, weights are Euclidean edge
    lengths in whatever frame verts is given in (world mm, as called).

    Returns CSR-style (neighbor_start, neighbor_index, neighbor_weight):
    node u's neighbours are the slots neighbor_start[u]:neighbor_start[u+1].

    Two representation choices worth not re-litigating:

    Flat CSR rather than a list-of-lists adjacency, because Surface_TX_Base
    has 271,036 directed entries -- a list[list[tuple]] allocates 45,430 lists
    plus 271,036 tuples (tens of MB) through an interpreted build loop, where
    this builds fully vectorised in ~0.27s.

    Python lists rather than numpy arrays on return, which looks backwards
    but is measured: identical algorithm and layout, only the container type
    differs, and the bare Dijkstra loop runs ~139ms with numpy element
    indexing vs ~81ms with lists on Surface_TX_Base -- about 1.7x. Numpy
    boxes a fresh scalar on every element access; a list already holds native
    ints/floats. (dijkstra_surface() itself costs ~50ms RX / ~85ms TX, above
    the bare loop, because it also allocates prev and converts to numpy on
    return.) The .tolist() below costs ~11ms, once."""
    faces = np.asarray(faces)
    e = np.sort(np.vstack([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]]), axis=1)
    e = np.unique(e, axis=0)  # each undirected edge once, however many triangles share it
    both = np.vstack([e, e[:, ::-1]])
    both = both[np.argsort(both[:, 0], kind='stable')]

    counts = np.bincount(both[:, 0], minlength=len(verts))
    start = np.concatenate([[0], np.cumsum(counts)])
    weight = np.linalg.norm(np.asarray(verts)[both[:, 0]] - np.asarray(verts)[both[:, 1]], axis=1)
    return start.tolist(), both[:, 1].tolist(), weight.tolist()


def nearest_vertex_index(verts, query_points):
    """Snap each query point to the nearest mesh vertex -- roadmap 6.2 step 2.
    Returns (indices (Q,), distances (Q,)).

    Brute force, deliberately: there is no scipy in this environment for a
    KD-tree, and at this size none is needed -- the worst case here is 70
    endpoints against Surface_TX_Base's 45,430 vertices, a 25MB intermediate
    solved in ~109ms, once per precompute. Distances are returned rather than
    discarded because they're the evidence the snap is legitimate: measured
    max 0.684mm (RX) / 0.580mm (TX), comfortably inside the ~1.24mm median
    mesh edge, so every endpoint lands on a vertex of the triangle it sits
    over."""
    verts = np.asarray(verts)
    query_points = np.asarray(query_points)
    d2 = ((verts[None, :, :] - query_points[:, None, :]) ** 2).sum(-1)
    idx = np.argmin(d2, axis=1)
    return idx, np.sqrt(d2[np.arange(len(query_points)), idx])


def dijkstra_surface(neighbor_start, neighbor_index, neighbor_weight, source):
    """Single-source shortest paths over a build_surface_graph() CSR graph --
    roadmap 6.2 step 3. Hand-rolled heapq rather than a scipy dependency, per
    AGENTS.md's from-scratch principle; standard lazy-deletion Dijkstra.

    Returns (dist (V,) float64, prev (V,) int32). Unreachable nodes are
    np.inf / -1. prev[source] is source itself -- a self-loop, not -1,
    because the obvious -1 would make "I am the source" and "I am
    unreachable" indistinguishable and leave geodesic_path_nodes() unable to
    tell a valid termination from a broken walk."""
    n = len(neighbor_start) - 1
    dist = [float('inf')] * n
    prev = [-1] * n
    dist[source] = 0.0
    prev[source] = source

    heap = [(0.0, source)]
    push, pop = heapq.heappush, heapq.heappop  # bound locally, this loop runs ~V log V times
    while heap:
        d, u = pop(heap)
        if d > dist[u]:
            continue  # stale entry left behind by a relaxation
        for k in range(neighbor_start[u], neighbor_start[u + 1]):
            v = neighbor_index[k]
            nd = d + neighbor_weight[k]
            if nd < dist[v]:
                dist[v] = nd
                prev[v] = u
                push(heap, (nd, v))

    return np.array(dist, dtype=np.float64), np.array(prev, dtype=np.int32)


def geodesic_path_nodes(prev_row, target):
    """Reconstruct one geodesic as a vertex-id list from a stored
    dijkstra_surface() predecessor row -- source-first, both ends inclusive.
    Returns [target] when target is the source, and None when unreachable.

    A pure walk-back: this never re-runs Dijkstra, which is the whole reason
    the (S,V) prev rows are retained rather than just the cost matrix.

    Note for roadmap 6.3: the returned path begins and ends at *snapped mesh
    vertices*, not at the curve endpoints themselves -- measured median
    ~0.36mm apart, max 0.68mm (see nearest_vertex_index). So a travel move
    built straight from these nodes leaves a sub-millimetre gap at both ends
    where it meets the piece it is travelling from/to. 6.3 must either append
    the true endpoints or accept that gap as within positioning tolerance."""
    if prev_row[target] < 0:
        return None
    path = [int(target)]
    while prev_row[path[-1]] != path[-1]:
        path.append(int(prev_row[path[-1]]))
    return path[::-1]


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

