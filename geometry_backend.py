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

TRAJECTORY_SAMPLE_INTERVAL_S = 0.01  # Minimum seconds between recorded TCP trajectory points
TRAJECTORY_RADIUS_MM = 0.28  # Trajectory curve line thickness, world units (mm)
TCP_FRAME_SCALE_MM = 50.0  # TCP coordinate-axes length, world units (mm)

BUILD_PLATE_DIR = "assets/buildPlate"
BUILD_PLATE_FILE = "BambuLab_BuildPlate.obj"
BUILD_PLATE_COLOR = (0.75, 0.77, 0.80)
BUILD_PLATE_THICKNESS_MM = 0.75

# Placed in the (-X, -Y) quadrant to match the arm's natural zero/home-pose
# reach direction -- the opposite quadrant only reaches via a near-limit J1
# rotation, leaving little margin for the wrist to also orient freely
USER_FRAME_ORIGIN_MM = np.array([-600.0, -300.0, 0.0])
USER_FRAME_SCALE_MM = 50.0  # Fixed axes drawn at the user frame, world units (mm)
BUILD_PLATE_POSITION_FILE = os.path.join(BUILD_PLATE_DIR, "saved_position.json")  # GUI Save/Load Position buttons

GCODE_DIR = "assets/models/gcode"
GCODE_FILE = "model.gcode"  # Fixed name -- overwritten by each new Cura export, never hand-edited
GCODE_COLOR = (1.0, 0.55, 0.0)  # Orange, so it doesn't visually merge with the Trajectory curve
GCODE_DEFAULT_LAYER_HEIGHT_MM = 0.1

# The G-code preview is a swept rectangular bead surface mesh, not a curve --
# each positive-extrusion G1 segment becomes a box whose width comes from the
# deposited volume (settled.md S1.11). Width = (dE * filament_area) / (L * h).
FILAMENT_DIAMETER_MM = 1.75
GCODE_BEAD_MIN_WIDTH_MM = 0.1   # Keep loose: bridge/overhang spans deposit thin, must not vanish
GCODE_BEAD_MAX_WIDTH_MM = 2.0   # Guard against priming blobs / degenerate short segments
# Progressive playback reveal re-registers a growing mesh prefix; only re-upload
# once this many new beads have appeared, so the near-complete (~1.4M-vert) mesh
# isn't re-sent every frame -- the segment-count/render-cost lever (S1.11).
GCODE_REVEAL_CHUNK = 200

GROUND_Z_MIN_MM = 0.0

# Toolpath IK is solved only at adaptive keyframes and interpolated in joint
# space for the ~0.65mm-median waypoints between them -- the selected IK branch
# is near-constant along a print, so per-waypoint solving is redundant for a
# visualization (settled.md S1.12). A keyframe is placed every STEP_MM of arc
# length OR at any vertex that turns more than ANGLE_DEG, so corners and G0
# travels stay crisp while straights are subsampled.
GCODE_IK_KEYFRAME_STEP_MM = 2.5
GCODE_IK_KEYFRAME_ANGLE_DEG = 15.0

# A completed precompute is cached to disk beside the (fixed) model.gcode and
# reloaded when an identical one is requested, rather than re-solving (~37s).
# The cache is keyed on everything the joint path depends on -- gcode content,
# plate pose, keyframe params -- plus a version that captures the IK/ground/
# joint-limit/robot-geometry code (bump it when any of those change). See S1.13.
PRECOMPUTE_CACHE_VERSION = 1
GCODE_PRECOMPUTE_CACHE = os.path.join(GCODE_DIR, "model.precompute.npz")

GCODE_MOVE_RE = re.compile(r"([A-Za-z])\s*(-?\d+\.?\d*)")


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
        self.trajectory_enabled = True
        self.trajectory_handle = None    # Set once a curve exists, see _update_trajectory_curve
        self.toolpath_joint_path = []
        self.toolpath_waypoints_world = []
        self.toolpath_status = "Toolpath IK not computed"
        self.toolpath_current_index = 0
        self.toolpath_precompute_active = False
        self.toolpath_precompute_paused = False
        self.toolpath_progress = 0.0
        self._toolpath_precompute_joint_limits = None
        self._toolpath_precompute_tcp_rotation = None
        self._toolpath_keyframe_indices = []      # waypoint indices IK is solved at (S1.12)
        self._toolpath_keyframe_angles = []        # solved joint angles, one per keyframe
        self._toolpath_precompute_previous_angles = None
        self._toolpath_precompute_start_time = None
        self._toolpath_precompute_user_frame = None  # plate pose the active job is solving for (cache key, S1.13)

        # G-code bead mesh cache (see build_print_beads / load_gcode / set_print_reveal).
        # Local geometry is built once per file; world verts are re-placed cheaply
        # on each plate reposition. bead_end_waypoint[k] = parsed-waypoint index at
        # which bead k finishes, so playback can reveal a growing prefix (S1.11).
        self._print_beads_source = None      # (filepath, mtime) the cached local beads were built from
        self._print_bead_verts_local = None  # (M*8, 3) plate-local box corners
        self._print_bead_faces = None        # (M*12, 3) triangle indices
        self._print_bead_end_waypoint = None # (M,) waypoint index each bead completes at
        self._print_bead_verts_world = None  # local verts placed through T_user_frame
        self._print_reveal_count = None      # beads currently uploaded (None = full mesh shown)

        # Initialise the scene
        self.create_coordinate_frame()
        self.load_build_plate()
        self.mesh_data = self.load_data()
        self.update_arm([0, 0, 0, 0, 0, 0])

        # Reuse a matching precomputed toolpath from a previous session if the
        # object + default plate pose + params are identical (S1.13). Best-effort.
        self.load_toolpath_cache()


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
        just a homogeneous transform applied here. position_mm marks where
        the plate's underside rests; the print
        surface is offset upward by the plate's own thickness. rpy_deg is
        [roll, pitch, yaw] degrees, XYZ fixed-angle convention
        (R = Rz(yaw) @ Ry(pitch) @ Rx(roll)) -- same convention as
        solve_ik_tcp. Safe to call repeatedly (e.g. from the GUI's
        Move/Reset buttons); Polyscope replaces the prior structures of
        the same names."""
        position_mm = np.asarray(position_mm, dtype=float)
        roll, pitch, yaw = np.deg2rad(rpy_deg)
        R = rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)

        self.T_user_frame = np.eye(4)
        self.T_user_frame[:3, :3] = R
        self.T_user_frame[:3, 3] = (
            position_mm + R @ [0.0, 0.0, BUILD_PLATE_THICKNESS_MM]
        )

        plate = self.load_mesh(os.path.join(BUILD_PLATE_DIR, BUILD_PLATE_FILE))
        homo = np.hstack([plate.vertices, np.ones((len(plate.vertices), 1))])
        plate_verts_world = (self.T_user_frame @ homo.T).T[:, :3]
        handle = ps.register_surface_mesh("Build Plate", plate_verts_world, plate.faces)
        handle.set_color(BUILD_PLATE_COLOR)

        self.create_coordinate_frame(
            scale=USER_FRAME_SCALE_MM,
            origin=self.T_user_frame[:3, 3],
            rotation=R,
            name="User Frame",
        )


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
        """Parse linear motion waypoints and mark deposited-material spans.

        Returns ([x, y, z], is_print_move, deposit) tuples in plate-local mm,
        where deposit is the E extruded on *this* move (e - previous_e), not the
        cumulative E. Using the per-move amount matters because retraction and
        the following un-retract happen on non-motion lines that create no
        waypoint, so a cumulative-E difference across the preceding travel move
        would wrongly include the whole retract distance and inflate the first
        bead of every region. All G0/G1 motion is preserved for execution
        continuity; only G1 moves with positive extrusion are print moves.
        """
        x, y, z = 0.0, 0.0, 0.0
        e = 0.0
        extrusion_absolute = True
        points = []
        with open(filepath) as f:
            for line in f:
                line = line.split(';', 1)[0]
                line = re.sub(r"\([^)]*\)", "", line)
                words = GCODE_MOVE_RE.findall(line)
                if not words:
                    continue

                letter0, value0 = words[0]
                letter0 = letter0.upper()
                try:
                    code = int(float(value0))
                except ValueError:
                    continue

                values = {}
                for letter, value in words[1:]:
                    values[letter.upper()] = float(value)

                if letter0 == 'M':
                    if code == 82:
                        extrusion_absolute = True
                    elif code == 83:
                        extrusion_absolute = False
                    continue

                if letter0 != 'G':
                    continue

                if code == 92:
                    if 'E' in values:
                        e = values['E']
                    continue

                if code not in (0, 1):
                    continue

                if 'X' in values:
                    x = values['X']
                if 'Y' in values:
                    y = values['Y']
                if 'Z' in values:
                    z = values['Z']

                has_motion_axis = any(axis in values for axis in ('X', 'Y', 'Z'))
                previous_e = e
                if 'E' in values:
                    if extrusion_absolute:
                        e = values['E']
                    else:
                        e += values['E']

                if not has_motion_axis:
                    continue

                is_print_move = code == 1 and e > previous_e
                points.append(([x, y, z], is_print_move, e - previous_e))
        return points


    def gcode_layer_height_mm(self, filepath):
        """Read Cura's layer-height header when present; fall back to 0.1 mm."""
        with open(filepath) as f:
            for line in f:
                match = re.match(r";Layer height:\s*(-?\d+\.?\d*)", line)
                if match:
                    return float(match.group(1))
                if line.lstrip().startswith(("G0", "G1")):
                    break
        return GCODE_DEFAULT_LAYER_HEIGHT_MM


    def build_print_beads(self, waypoints, layer_height):
        """Swept rectangular bead geometry for every deposited-material segment.

        Returns (verts_local, faces, bead_end_waypoint) in plate-local mm, or
        (None, None, None) if there is nothing to draw. A segment is deposited
        material iff its destination G1 increased E (is_print_move) -- there is
        deliberately no ;TYPE: filtering, so bridge/overhang spans across open
        air become solid bars rather than gaps (settled.md S1.11). Bead width
        comes from the deposited volume, height from the layer height. Fully
        vectorised; each bead is an 8-vertex / 12-triangle box.
        """
        pts = np.array([p for p, _, _ in waypoints], dtype=float)
        is_print = np.array([w[1] for w in waypoints], dtype=bool)
        deposit = np.array([w[2] for w in waypoints], dtype=float)  # E extruded on each move (per-move, not cumulative)

        end_idx = np.nonzero(is_print[1:])[0] + 1  # destination waypoint of each print segment
        if len(end_idx) == 0:
            return None, None, None

        a = pts[end_idx - 1]
        b = pts[end_idx]
        vec = b - a
        length = np.linalg.norm(vec, axis=1)

        keep = length > 1e-9  # drop zero-length print moves (e.g. an unretract with no XYZ change)
        end_idx, a, b, vec, length = end_idx[keep], a[keep], b[keep], vec[keep], length[keep]
        if len(end_idx) == 0:
            return None, None, None

        # width * height ~= deposited cross-section = (filament_area * dE) / segment_length
        filament_area = np.pi * (FILAMENT_DIAMETER_MM / 2.0) ** 2
        width = np.clip((deposit[end_idx] * filament_area) / (length * layer_height),
                        GCODE_BEAD_MIN_WIDTH_MM, GCODE_BEAD_MAX_WIDTH_MM)

        tangent = vec / length[:, None]
        up = np.array([0.0, 0.0, 1.0])              # plate-local build normal
        side = np.cross(tangent, up)
        side_norm = np.linalg.norm(side, axis=1)
        vertical = side_norm < 1e-6                 # near-vertical segment: pick an arbitrary in-plane side
        side[vertical] = np.array([1.0, 0.0, 0.0])
        side_norm[vertical] = 1.0
        side = side / side_norm[:, None]

        w_off = (width / 2.0)[:, None] * side        # half bead width, in the plate plane
        top_z = a[:, 2]
        first_layer_z = top_z.min()
        # Cura's initial layer can be thicker than the general layer height; hang
        # the first layer down to local Z=0 so it reaches the plate surface (S1.15).
        bead_height = np.where(np.isclose(top_z, first_layer_z), first_layer_z, layer_height)
        down = np.column_stack([np.zeros(len(bead_height)),
                                np.zeros(len(bead_height)),
                                -bead_height])

        a_tl, a_tr = a + w_off, a - w_off
        b_tl, b_tr = b + w_off, b - w_off
        verts = np.stack([
            a_tl, a_tr, a_tr + down, a_tl + down,    # 0..3  start face (tl, tr, br, bl)
            b_tl, b_tr, b_tr + down, b_tl + down,    # 4..7  end face
        ], axis=1).reshape(-1, 3)

        face_template = np.array([
            [0, 3, 7], [0, 7, 4],   # left
            [1, 2, 6], [1, 6, 5],   # right
            [0, 1, 5], [0, 5, 4],   # top
            [3, 2, 6], [3, 6, 7],   # bottom
            [0, 1, 2], [0, 2, 3],   # start cap
            [4, 5, 6], [4, 6, 7],   # end cap
        ])
        faces = (face_template[None] + (np.arange(len(end_idx)) * 8)[:, None, None]).reshape(-1, 3)
        return verts, faces, end_idx


    def _ensure_print_beads(self, filepath):
        """Build and cache plate-local bead geometry for filepath unless it is
        already cached for this file+mtime. Keeps a plate reposition cheap: the
        heavy parse/build runs once per file, and only the T_user_frame placement
        re-runs afterwards. Returns True if beads are available."""
        source = (filepath, os.path.getmtime(filepath))
        if self._print_beads_source == source and self._print_bead_verts_local is not None:
            return True

        waypoints = self.parse_gcode(filepath)
        if len(waypoints) < 2:
            return False
        verts, faces, ends = self.build_print_beads(waypoints, self.gcode_layer_height_mm(filepath))
        if verts is None:
            return False

        self._print_bead_verts_local = verts
        self._print_bead_faces = faces
        self._print_bead_end_waypoint = ends
        self._print_beads_source = source
        return True


    def load_gcode(self):
        """Register deposited material as a swept rectangular bead surface mesh.

        Positive-extrusion G1 segments become solid beads sized by deposited
        volume (width) and layer height, so the preview reads as the real
        printed object -- bridge/overhang spans included (settled.md S1.11).
        Plate-local geometry is built once per file (_ensure_print_beads) and
        placed into world with the full T_user_frame multiply, same placement as
        before (S1.3/S1.8). Shows the whole print; playback trims it via
        set_print_reveal. Safe to call repeatedly; no-ops if the file is absent.
        """
        filepath = os.path.join(GCODE_DIR, GCODE_FILE)
        if not os.path.exists(filepath):
            return
        if not self._ensure_print_beads(filepath):
            return

        homo = np.hstack([self._print_bead_verts_local,
                          np.ones((len(self._print_bead_verts_local), 1))])
        self._print_bead_verts_world = (self.T_user_frame @ homo.T).T[:, :3]
        self._print_reveal_count = None  # full mesh currently shown

        handle = ps.register_surface_mesh("G-code Print", self._print_bead_verts_world,
                                          self._print_bead_faces)
        handle.set_color(GCODE_COLOR)


    def set_print_reveal(self, waypoint_index):
        """Show only the beads deposited up to waypoint_index, for the playback
        build-up. Re-registers a growing prefix of the cached bead mesh, throttled
        by GCODE_REVEAL_CHUNK so the near-complete (~1.4M-vert) mesh is not
        re-uploaded every frame (settled.md S1.11). The empty and fully-complete
        endpoints are always honored exactly. No-op if no bead mesh is loaded."""
        if self._print_bead_verts_world is None or self._print_bead_end_waypoint is None:
            return

        n = int(np.searchsorted(self._print_bead_end_waypoint, waypoint_index, side='right'))
        total = len(self._print_bead_end_waypoint)
        if (self._print_reveal_count is not None and n not in (0, total)
                and abs(n - self._print_reveal_count) < GCODE_REVEAL_CHUNK):
            return
        self._print_reveal_count = n

        if n == 0:
            ps.remove_surface_mesh("G-code Print", error_if_absent=False)
            return
        handle = ps.register_surface_mesh("G-code Print",
                                          self._print_bead_verts_world[:n * 8],
                                          self._print_bead_faces[:n * 12])
        handle.set_color(GCODE_COLOR)


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


    def solve_ik_tcp(self, target_pos_mm, target_rpy_deg, joint_limits):
        """
        GUI-facing IK entry point, targeting the TCP pose rather than the
        flange -- see settled.md S1.4. Converts via self.T_flange_to_tcp,
        solves, filters by joint_limits, then ranks every valid branch by
        closeness to self.current_joint_angles -- see settled.md S1.5.

        target_rpy_deg: [roll, pitch, yaw] degrees, fixed-angle convention
        (R = Rz(yaw) @ Ry(pitch) @ Rx(roll)).
        Returns (solutions, status_message); solutions is a list of
        (joint_angles_deg, is_wrist_singular, raw_branch_index), sorted
        closest-to-current first. Empty list on failure.
        """
        roll, pitch, yaw = np.deg2rad(target_rpy_deg)
        R_target = rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)

        return self.solve_ik_tcp_matrix(target_pos_mm, R_target, joint_limits)


    def solve_ik_tcp_matrix(self, target_pos_mm, target_rotation, joint_limits, reference_joint_angles=None):
        """TCP IK entry point for callers that already have a rotation
        matrix. Used by toolpath precompute so plate-normal orientation
        does not need a matrix->RPY->matrix round trip."""
        T_target_tcp = np.eye(4)
        T_target_tcp[:3, :3] = np.asarray(target_rotation, dtype=float)
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

        reference = self.current_joint_angles if reference_joint_angles is None else reference_joint_angles

        def wrapped_dist(angles):
            diff = (angles - reference + 180) % 360 - 180
            return np.sum(np.abs(diff))

        valid.sort(key=lambda item: wrapped_dist(item[0]))
        status = f"Solved ({len(valid)} valid solution{'s' if len(valid) != 1 else ''})"
        return valid, status


    def moving_geometry_min_z(self, joint_angles_deg):
        """Minimum Z of Robot1-Robot6 plus nozzle after the Delta transform."""
        T_current = self.compute_fk(joint_angles_deg)
        min_z = np.inf
        for verts, src in zip(self.ground_check_verts, self.ground_check_sources):
            Delta = T_current[src] @ self.T_zero_inv[src]
            z = verts @ Delta[2, :3] + Delta[2, 3]
            min_z = min(min_z, float(np.min(z)))
            if min_z < GROUND_Z_MIN_MM:
                return min_z
        return min_z


    def moving_geometry_bbox_min_z(self, joint_angles_deg):
        """Cheap conservative minimum Z of moving geometry bounding boxes."""
        T_current = self.compute_fk(joint_angles_deg)
        min_z = np.inf
        for corners, src in zip(self.ground_check_bbox_corners, self.ground_check_sources):
            Delta = T_current[src] @ self.T_zero_inv[src]
            z = corners @ Delta[2, :3] + Delta[2, 3]
            min_z = min(min_z, float(np.min(z)))
            if min_z < GROUND_Z_MIN_MM:
                return min_z
        return min_z


    def _reset_toolpath_precompute(self, progress):
        """Clear the precomputed path and stop the job. Caller sets its own status."""
        self.toolpath_joint_path = []
        self.toolpath_current_index = 0
        self.toolpath_progress = progress
        self.toolpath_precompute_active = False
        self.toolpath_precompute_paused = False

    def _compute_keyframe_indices(self, world):
        """Waypoint indices to solve IK at: adaptive by arc-length + heading.

        A keyframe is placed every GCODE_IK_KEYFRAME_STEP_MM of arc length, and
        at any interior vertex that turns more than GCODE_IK_KEYFRAME_ANGLE_DEG
        so corners and long G0 travels keep their exact pose; the ~0.65mm-median
        waypoints between keyframes are joint-interpolated at finish (S1.12).
        """
        n = len(world)
        if n <= 2:
            return np.arange(n, dtype=int)
        seg = np.diff(world, axis=0)
        d = np.linalg.norm(seg, axis=1)
        unit = seg / np.where(d[:, None] > 1e-9, d[:, None], 1.0)
        cos = np.clip(np.sum(unit[:-1] * unit[1:], axis=1), -1.0, 1.0)  # turn at vertex j+1
        corner = np.degrees(np.arccos(cos)) > GCODE_IK_KEYFRAME_ANGLE_DEG
        keys = [0]
        accum = 0.0
        for i in range(1, n - 1):
            accum += d[i - 1]
            if accum >= GCODE_IK_KEYFRAME_STEP_MM or corner[i - 1]:
                keys.append(i)
                accum = 0.0
        keys.append(n - 1)
        return np.array(keys, dtype=int)

    def _finish_toolpath_precompute(self):
        """Interpolate the solved keyframes into a dense per-waypoint path and stop.

        Playback and reveal index toolpath_joint_path by waypoint, so the sparse
        keyframe solves are expanded here to one pose per waypoint (S1.12). Each
        joint is interpolated against cumulative arc length so motion tracks
        distance travelled; a degenerate (non-increasing) arc coordinate falls
        back to waypoint index.
        """
        world = self.toolpath_waypoints_world
        n = len(world)
        kf = self._toolpath_keyframe_indices
        q = np.array(self._toolpath_keyframe_angles)  # (K, 6)
        elapsed_s = time.time() - self._toolpath_precompute_start_time

        s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(world, axis=0), axis=1))])
        x, xp = (s, s[kf]) if np.all(np.diff(s[kf]) > 0) else (np.arange(n), kf.astype(float))
        dense = np.empty((n, 6))
        for j in range(6):
            dense[:, j] = np.interp(x, xp, q[:, j])

        self.toolpath_joint_path = dense
        self.toolpath_current_index = 0
        self.toolpath_progress = 1.0
        self.toolpath_precompute_active = False
        self.toolpath_precompute_paused = False
        self.toolpath_status = f"Toolpath IK ready: {n} waypoints via {len(kf)} keyframes ({elapsed_s:.1f}s)"
        self.save_toolpath_cache()

    def _toolpath_cache_meta(self, user_frame):
        """The cache key: everything the solved joint path depends on (S1.13).

        gcode is hashed by content (not mtime) so an identical re-export still
        hits; the plate pose is included because the path is only valid for the
        pose it was solved at; keyframe params and a version tag cover the rest.
        """
        with open(os.path.join(GCODE_DIR, GCODE_FILE), "rb") as f:
            gcode_sha256 = hashlib.sha256(f.read()).hexdigest()
        return {
            "version": PRECOMPUTE_CACHE_VERSION,
            "gcode_sha256": gcode_sha256,
            "user_frame": np.round(np.asarray(user_frame, dtype=float), 6).tolist(),
            "step_mm": GCODE_IK_KEYFRAME_STEP_MM,
            "angle_deg": GCODE_IK_KEYFRAME_ANGLE_DEG,
            "ground_z": GROUND_Z_MIN_MM,
        }

    def save_toolpath_cache(self):
        """Write the completed joint path + its key to disk (S1.13). Best-effort:
        a cache write must never break an otherwise-good precompute."""
        try:
            meta = self._toolpath_cache_meta(self._toolpath_precompute_user_frame)
            np.savez(GCODE_PRECOMPUTE_CACHE,
                     joint_path=self.toolpath_joint_path.astype(np.float32),
                     meta=np.array(json.dumps(meta)))
        except Exception:
            pass

    def load_toolpath_cache(self):
        """Load a cached joint path iff its key matches the current inputs (S1.13).

        Returns True on a hit (state left ready for playback), False on any miss,
        missing file, or corruption -- the caller then solves normally.
        """
        if not (os.path.exists(GCODE_PRECOMPUTE_CACHE) and
                os.path.exists(os.path.join(GCODE_DIR, GCODE_FILE))):
            return False
        try:
            cached = np.load(GCODE_PRECOMPUTE_CACHE, allow_pickle=False)
            if json.loads(cached["meta"].item()) != self._toolpath_cache_meta(self.T_user_frame):
                return False
            self.toolpath_joint_path = cached["joint_path"].astype(float)
        except Exception:
            return False
        self.toolpath_current_index = 0
        self.toolpath_progress = 1.0
        self.toolpath_precompute_active = False
        self.toolpath_precompute_paused = False
        self.toolpath_status = f"Toolpath IK loaded from cache: {len(self.toolpath_joint_path)} waypoints"
        return True

    def start_toolpath_ik_precompute(self, joint_limits):
        """Initialize a chunked toolpath IK precompute job."""
        self.cancel_toolpath_ik_precompute()
        filepath = os.path.join(GCODE_DIR, GCODE_FILE)
        if not os.path.exists(filepath):
            self.toolpath_status = f"Toolpath IK failed: {filepath} not found"
            self.toolpath_waypoints_world = []
            self._reset_toolpath_precompute(0.0)
            return False

        if self.load_toolpath_cache():
            return True

        waypoints = self.parse_gcode(filepath)
        if not waypoints:
            self.toolpath_status = "Toolpath IK failed: no G0/G1 waypoints found"
            self.toolpath_waypoints_world = []
            self._reset_toolpath_precompute(0.0)
            return False

        points_local = np.array([p for p, _, _ in waypoints])
        homo = np.hstack([points_local, np.ones((len(points_local), 1))])
        self.toolpath_waypoints_world = (self.T_user_frame @ homo.T).T[:, :3]
        self.toolpath_joint_path = []
        self.toolpath_current_index = 0
        self.toolpath_progress = 0.0
        self.toolpath_precompute_active = True
        self.toolpath_precompute_paused = False
        self._toolpath_precompute_joint_limits = joint_limits
        self._toolpath_precompute_tcp_rotation = self.T_user_frame[:3, :3].copy()
        self._toolpath_precompute_user_frame = self.T_user_frame.copy()
        self._toolpath_keyframe_indices = self._compute_keyframe_indices(self.toolpath_waypoints_world)
        self._toolpath_keyframe_angles = []
        self._toolpath_precompute_previous_angles = np.asarray(self.current_joint_angles, dtype=float)
        self._toolpath_precompute_start_time = time.time()
        self.toolpath_status = f"Toolpath IK precomputing: 0/{len(self._toolpath_keyframe_indices)} keyframes"
        return True


    def step_toolpath_ik_precompute(self, max_steps=10):
        """Solve a small batch of keyframes for an active precompute job."""
        if not self.toolpath_precompute_active:
            return False

        total = len(self._toolpath_keyframe_indices)
        n_waypoints = len(self.toolpath_waypoints_world)
        if self.toolpath_precompute_paused:
            self.toolpath_status = f"Toolpath IK paused: {len(self._toolpath_keyframe_angles)}/{total} keyframes"
            return True

        steps = max(1, int(max_steps))

        for _ in range(steps):
            k = len(self._toolpath_keyframe_angles)
            if k >= total:
                self._finish_toolpath_precompute()
                return False

            wp = int(self._toolpath_keyframe_indices[k])
            solutions, status = self.solve_ik_tcp_matrix(
                self.toolpath_waypoints_world[wp],
                self._toolpath_precompute_tcp_rotation,
                self._toolpath_precompute_joint_limits,
                self._toolpath_precompute_previous_angles)
            if not solutions:
                self.toolpath_status = f"Toolpath IK failed at waypoint {wp + 1}/{n_waypoints}: {status}"
                self._reset_toolpath_precompute(wp / n_waypoints)
                return False

            # bbox min-z is a lower bound on the exact mesh min-z, so bbox >= 0
            # is already a guarantee of ground clearance; only fall back to the
            # expensive exact check to adjudicate a branch whose bbox dips below
            # (it may still clear). See settled.md S1.12.
            selected_angles = None
            lowest_min_z = np.inf
            for angles, _, _ in solutions:
                if self.moving_geometry_bbox_min_z(angles) >= GROUND_Z_MIN_MM:
                    selected_angles = angles
                    break
                exact_min_z = self.moving_geometry_min_z(angles)
                lowest_min_z = min(lowest_min_z, exact_min_z)
                if exact_min_z >= GROUND_Z_MIN_MM:
                    selected_angles = angles
                    break

            if selected_angles is None:
                self.toolpath_status = (
                    f"Toolpath IK failed at waypoint {wp + 1}/{n_waypoints}: "
                    f"all branches cross z={GROUND_Z_MIN_MM:g} (lowest min z {lowest_min_z:.1f} mm)"
                )
                self._reset_toolpath_precompute(wp / n_waypoints)
                return False

            self._toolpath_precompute_previous_angles = selected_angles
            self._toolpath_keyframe_angles.append(selected_angles.copy())
            self.toolpath_progress = (k + 1) / total

        done = len(self._toolpath_keyframe_angles)
        if done >= total:
            self._finish_toolpath_precompute()
            return False

        self.toolpath_status = f"Toolpath IK precomputing: {done}/{total} keyframes"
        return True


    def cancel_toolpath_ik_precompute(self):
        """Stop an active precompute job without clearing the last ready path."""
        self.toolpath_precompute_active = False
        self.toolpath_precompute_paused = False
        self._toolpath_precompute_joint_limits = None
        self._toolpath_precompute_tcp_rotation = None
        self._toolpath_keyframe_indices = []
        self._toolpath_keyframe_angles = []
        self._toolpath_precompute_previous_angles = None
        self._toolpath_precompute_start_time = None
        self._toolpath_precompute_user_frame = None


    def pause_toolpath_ik_precompute(self):
        """Pause an active precompute job without discarding progress."""
        if not self.toolpath_precompute_active:
            return False
        self.toolpath_precompute_paused = True
        done = len(self._toolpath_keyframe_angles)
        total = len(self._toolpath_keyframe_indices)
        self.toolpath_status = f"Toolpath IK paused: {done}/{total} keyframes"
        return True


    def resume_toolpath_ik_precompute(self):
        """Resume a paused precompute job."""
        if not self.toolpath_precompute_active:
            return False
        self.toolpath_precompute_paused = False
        done = len(self._toolpath_keyframe_angles)
        total = len(self._toolpath_keyframe_indices)
        self.toolpath_status = f"Toolpath IK precomputing: {done}/{total} keyframes"
        return True


    def reset_toolpath_playback(self):
        """Return playback to the first precomputed waypoint."""
        self.toolpath_current_index = 0
        if len(self.toolpath_joint_path) == 0:
            self.toolpath_status = "Toolpath playback reset: no precomputed path"
            return False

        self.update_arm(self.toolpath_joint_path[0])
        self.set_print_reveal(0)  # empty the printed shape; it rebuilds as playback advances (S1.11)
        self.toolpath_status = f"Toolpath playback reset: 1/{len(self.toolpath_joint_path)}"
        return True


    def advance_toolpath_playback(self, step_count=1):
        """Apply cached toolpath joint poses in order."""
        if len(self.toolpath_joint_path) == 0:
            self.toolpath_status = "Toolpath playback failed: no precomputed path"
            return False

        if self.toolpath_current_index >= len(self.toolpath_joint_path):
            self.toolpath_status = f"Toolpath playback complete: {len(self.toolpath_joint_path)} waypoints"
            return False

        steps = max(1, int(step_count))
        next_index = min(self.toolpath_current_index + steps, len(self.toolpath_joint_path))
        self.update_arm(self.toolpath_joint_path[next_index - 1])
        self.toolpath_current_index = next_index
        self.set_print_reveal(next_index)  # grow the printed shape to match progress (S1.11)

        if self.toolpath_current_index >= len(self.toolpath_joint_path):
            self.toolpath_status = f"Toolpath playback complete: {len(self.toolpath_joint_path)} waypoints"
            return False

        self.toolpath_status = (
            f"Toolpath playback: {self.toolpath_current_index}/{len(self.toolpath_joint_path)}"
        )
        return True


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
        self.T_zero_inv = [np.linalg.inv(T) for T in self.T_zero]

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

        self.ground_check_verts = self.rest_verts[:7]
        self.ground_check_sources = [0, 1, 2, 3, 4, 5, 5]
        self.ground_check_bbox_corners = []
        for verts in self.ground_check_verts:
            lo = np.min(verts, axis=0)
            hi = np.max(verts, axis=0)
            self.ground_check_bbox_corners.append(np.array([
                [lo[0], lo[1], lo[2]],
                [lo[0], lo[1], hi[2]],
                [lo[0], hi[1], lo[2]],
                [lo[0], hi[1], hi[2]],
                [hi[0], lo[1], lo[2]],
                [hi[0], lo[1], hi[2]],
                [hi[0], hi[1], lo[2]],
                [hi[0], hi[1], hi[2]],
            ]))

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


    def record_trajectory_point(self):
        """Sample self.tcp_world at most once per TRAJECTORY_SAMPLE_INTERVAL_S;
        discard the sample if the TCP hasn't moved since the last recorded point."""
        if not self.trajectory_enabled:
            return

        now = time.time()
        if now - self._last_sample_time < TRAJECTORY_SAMPLE_INTERVAL_S:
            return
        self._last_sample_time = now

        if self.trajectory_points and np.allclose(self.tcp_world, self.trajectory_points[-1]):
            return

        self.trajectory_points.append(self.tcp_world.copy())
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
        ps.remove_curve_network("Trajectory", error_if_absent=False)



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

