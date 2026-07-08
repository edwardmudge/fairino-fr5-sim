import os
import re
import time
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

BUILD_PLATE_DIR = "assets/buildPlate"
BUILD_PLATE_FILE = "BambuLab_BuildPlate.obj"

# Placed in the (-X, -Y) quadrant to match the arm's natural zero/home-pose
# reach direction -- the opposite quadrant only reaches via a near-limit J1
# rotation, leaving little margin for the wrist to also orient freely
USER_FRAME_ORIGIN_MM = np.array([-600.0, -300.0, 0.0])
USER_FRAME_SCALE_MM = 50.0  # Fixed axes drawn at the user frame, world units (mm)

GCODE_DIR = "assets/gcode"
GCODE_FILE = "square_test.gcode"
GCODE_RADIUS_MM = 1.5  # Toolpath curve thickness, world units (mm) -- distinct from TRAJECTORY_RADIUS_MM
GCODE_COLOR = (1.0, 0.55, 0.0)  # Orange, so it doesn't visually merge with the Trajectory curve

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

        # Initialise the scene
        self.create_coordinate_frame()
        self.load_build_plate()
        self.mesh_data = self.load_data()
        self.update_arm([0, 0, 0, 0, 0, 0])


    def create_coordinate_frame(self, scale=1.0, origin=(0, 0, 0), name="Coordinate Frame"):
        """Register an XYZ axis triad. Reused for the static world-origin
        frame (defaults) and, with an origin/scale/name override, for the
        TCP frame -- see load_data() and docs/FR5_Mesh_Convention.md.
        Returns (handle, nodes) so callers can drive the nodes through the
        Delta transform like any other zero-pose-frame geometry."""
        origin = np.asarray(origin, dtype=float)
        nodes = np.array([origin, origin + [scale,0,0], origin + [0,scale,0], origin + [0,0,scale]])
        edges = np.array([[0,1], [0,2], [0,3]])

        ps_net = ps.register_curve_network(name, nodes, edges)

        # X=red, Y=green, Z=blue
        colors = np.array([[1,0,0], [0,1,0], [0,0,1]])
        ps_net.add_color_quantity("axis_colors", colors, defined_on='edges', enabled=True)

        return ps_net, nodes


    def load_build_plate(self):
        """Place the build plate at the user frame -- see GLOSSARY.md 'User
        frame' and settled.md S1.2. Static geometry: registered once, never
        updated per-frame -- unlike the arm links, the plate isn't driven by
        any joint, so it needs no Delta transform, just a one-time
        translation."""
        self.T_user_frame = np.eye(4)
        self.T_user_frame[:3, 3] = USER_FRAME_ORIGIN_MM

        plate = self.load_mesh(os.path.join(BUILD_PLATE_DIR, BUILD_PLATE_FILE))
        plate_verts_world = plate.vertices + USER_FRAME_ORIGIN_MM  # mesh origin == plate corner
        ps.register_surface_mesh("Build Plate", plate_verts_world, plate.faces)

        self.create_coordinate_frame(scale=USER_FRAME_SCALE_MM, origin=USER_FRAME_ORIGIN_MM, name="User Frame")


    def parse_gcode(self, filepath):
        """Parse G0/G1 linear moves. Returns a list of ([x, y, z],
        is_feed_move) pairs, plate-local mm -- G0 travel moves update
        position but are flagged is_feed_move=False so load_gcode() skips
        drawing them. Modal position handling: see GLOSSARY.md 'G-code
        toolpath'. Scoped to the square test fixture, not a general G-code
        interpreter."""
        x, y, z = 0.0, 0.0, 0.0
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
                points.append(([x, y, z], code == 1))
        return points


    def load_gcode(self):
        """Register the G1 feed-move segments of the parsed G-code as a
        static curve network on the plate -- G0 travel moves are parsed
        (for position tracking) but not drawn, see parse_gcode(). Points
        are plate-local (mm) and mapped to world with the full
        T_user_frame matrix multiply, not a raw vector add -- see
        settled.md S1.3. Safe to call repeatedly; Polyscope replaces the
        prior structure of the same name, same as _update_trajectory_curve."""
        waypoints = self.parse_gcode(os.path.join(GCODE_DIR, GCODE_FILE))
        if len(waypoints) < 2:
            return

        points_local = np.array([p for p, _ in waypoints])
        homo = np.hstack([points_local, np.ones((len(points_local), 1))])
        nodes = (self.T_user_frame @ homo.T).T[:, :3]

        edges = np.array([[i, i + 1] for i in range(len(nodes) - 1) if waypoints[i + 1][1]])
        if len(edges) == 0:
            return

        handle = ps.register_curve_network("G-code Toolpath", nodes, edges)
        handle.set_radius(GCODE_RADIUS_MM, relative=False)
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
            diff = (angles - self.current_joint_angles + 180) % 360 - 180
            return np.sum(np.abs(diff))

        valid.sort(key=lambda item: wrapped_dist(item[0]))
        status = f"Solved ({len(valid)} valid solution{'s' if len(valid) != 1 else ''})"
        return valid, status


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

