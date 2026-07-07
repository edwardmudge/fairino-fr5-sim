import polyscope.imgui as psim
import numpy as np

# FR5 practical joint slider ranges (degrees), asymmetric per joint.
# Source: docs/FR5_Joint_Limits.md "Practical Slider Ranges"
JOINT_LIMITS = [
    (-170, 170),  # J1
    (-130, 80),   # J2
    (-155, 155),  # J3
    (-170, 80),   # J4
    (-170, 170),  # J5
    (-170, 170),  # J6
]

class UI_Menu:
    """
    [Frontend Interaction Layer]
    Responsibilities:
    1. Draw ImGui controls (Button, Slider, Input)
    2. Collect user input
    3. Call methods on self.content to perform actions
    """
    def __init__(self, content_instance):
        # Dependency injection: the UI holds a reference to the backend
        self.content = content_instance
        
        # UI internal state (View State)
        self.show_settings = True
        self.slider_val = 0.0
        self.trans_vec = np.array([0.0, 0.0, 0.0])
        self.rot_vec = np.array([0.0, 0.0, 0.0])
        self.joint_angles = np.zeros(6)

    def render(self):
        """This function needs to be called by Polyscope every frame"""
        
        # 1. Panel title
        psim.TextUnformatted("GeoProc Template Control")
        psim.Separator()

        # 2. Data loading section
        if psim.TreeNode("I/O Operations"):
            if psim.Button("Load Test Data"):
                self.content.load_dummy_data()
            psim.TreePop()

        # 3. Joint angle control section
        if psim.TreeNode("Joint Angles"):
            changed_any = False
            for i in range(6):
                lo, hi = JOINT_LIMITS[i]
                changed, self.joint_angles[i] = psim.SliderFloat(f"J{i+1}", self.joint_angles[i], lo, hi)
                changed_any = changed_any or changed

            if changed_any:
                self.content.update_arm(self.joint_angles)

            psim.TreePop()

        # 4. Transformation control section
        if psim.TreeNode("Transformation"):
            changed_t, self.trans_vec = psim.InputFloat3("Translate", self.trans_vec)
            changed_r, self.rot_vec = psim.SliderFloat3("Rotate", self.rot_vec, -180, 180)
            
            # If the user has interacted with the UI, notify the backend immediately
            if changed_t or changed_r:
                self.content.update_transformation(self.rot_vec, self.trans_vec)
            
            psim.TreePop()

        # 5. Algorithm parameters section
        if psim.TreeNode("Algorithm Settings"):
            _, self.show_settings = psim.Checkbox("Enable Advanced", self.show_settings)
            
            if self.show_settings:
                _, self.slider_val = psim.SliderFloat("Smoothness", self.slider_val, 0.0, 1.0)
                
                if psim.Button("Run Processing"):
                    self.content.run_algorithm(self.slider_val, "method_A")
            
            psim.TreePop()
