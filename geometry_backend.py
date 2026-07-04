import polyscope as ps
import numpy as np

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
        
        # Initialise the scene
        self.create_coordinate_frame()

    def create_coordinate_frame(self, scale=1.0):
        """Initialise a basic coordinate frame, to prevent an empty scene"""
        nodes = np.array([[0,0,0], [scale,0,0], [0,scale,0], [0,0,scale]])
        edges = np.array([[0,1], [0,2], [0,3]])
        
        ps_net = ps.register_curve_network("Coordinate Frame", nodes, edges)
        
        # X=red, Y=green, Z=blue
        colors = np.array([[1,0,0], [0,1,0], [0,0,1]])
        ps_net.add_color_quantity("axis_colors", colors, defined_on='edges', enabled=True)

    def load_dummy_data(self):
        """Example: load a test sphere"""
        # This can be replaced with trimesh.load() later
        print("[Backend] Generating dummy geometry...")
        pts = np.random.rand(100, 3)
        ps.register_point_cloud("Test Cloud", pts)

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
