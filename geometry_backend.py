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

# User frame: base-frame -> build-plate corner. Translation-only for now.
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
        self.transformation = np.eye(4)
        self.point_cloud_data = None
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
        """Place the build plate at the user frame (base-frame -> plate
        corner). Translation-only for now -- see GLOSSARY.md 'User frame'.
        Static geometry: registered once, never updated per-frame -- unlike
        the arm links, the plate isn't driven by any joint, so it needs no
        Delta transform, just a one-time translation."""
        self.T_user_frame = np.eye(4)
        self.T_user_frame[:3, 3] = USER_FRAME_ORIGIN_MM

        plate = self.load_mesh(os.path.join(BUILD_PLATE_DIR, BUILD_PLATE_FILE))
        plate_verts_world = plate.vertices + USER_FRAME_ORIGIN_MM  # mesh origin == plate corner
        ps.register_surface_mesh("Build Plate", plate_verts_world, plate.faces)

        self.create_coordinate_frame(scale=USER_FRAME_SCALE_MM, origin=USER_FRAME_ORIGIN_MM, name="User Frame")


    def parse_gcode(self, filepath):
        """Parse G0/G1 linear moves, tracking modal position (an axis word
        missing from a line keeps its last value; the very first line
        defaults missing axes to 0). Comments (';...' and '(...)') and
        everything except G0/G1 are ignored. Returns a list of
        ([x, y, z], is_feed_move) pairs, plate-local mm -- G0 travel moves
        still update position (needed as the anchor for the next drawn
        edge) but are flagged is_feed_move=False so load_gcode() skips
        drawing them. Scoped to the square test fixture, not a general
        G-code interpreter."""
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

        # TCP point, also Delta_6, but a Polyscope point cloud -- update_point_positions,
        # not update_vertex_positions, hence the per-object self.update_fns lookup
        tcp_point = self.tcp_local.reshape(1, 3)
        self.rest_verts.append(tcp_point)
        self.point_cloud_data = ps.register_point_cloud("TCP", tcp_point)
        self.mesh_handles.append(self.point_cloud_data)
        self.update_fns.append(self.point_cloud_data.update_point_positions)

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


    def update_transformation(self, rotation, translation):
        """
        Handle the transformation logic
        :param rotation: np.array [3] (degrees)
        :param translation: np.array [3]
        """
        # The actual matrix computation logic goes here
        # self.transformation = ... 
        print(f"[Backend] Matrix Updated | Rot: {rotation} | Trans: {translation}")
        

    def run_algorithm(self, param_a, param_b):
        """Example algorithm interface"""
        print(f"[Backend] Running Algorithm with params: {param_a}, {param_b}")
        # Once the time-consuming computation is finished, call ps.register_... to update the display



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


# Validation
if __name__ == "__main__":
    ps.init()
    vis = VisContent()
    vis.end_effector_position([0, 0, 0, 0, 0, 0])
    print(f"[Backend] Loaded {len(vis.mesh_data)} link meshes")

    gcode_waypoints = vis.parse_gcode(os.path.join(GCODE_DIR, GCODE_FILE))
    print(f"[Backend] Parsed {len(gcode_waypoints)} G-code waypoints")
    vis.load_gcode()

