import polyscope.imgui as psim
import numpy as np

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

        # 3. Transformation control section
        if psim.TreeNode("Transformation"):
            changed_t, self.trans_vec = psim.InputFloat3("Translate", self.trans_vec)
            changed_r, self.rot_vec = psim.SliderFloat3("Rotate", self.rot_vec, -180, 180)
            
            # If the user has interacted with the UI, notify the backend immediately
            if changed_t or changed_r:
                self.content.update_transformation(self.rot_vec, self.trans_vec)
            
            psim.TreePop()

        # 4. Algorithm parameters section
        if psim.TreeNode("Algorithm Settings"):
            _, self.show_settings = psim.Checkbox("Enable Advanced", self.show_settings)
            
            if self.show_settings:
                _, self.slider_val = psim.SliderFloat("Smoothness", self.slider_val, 0.0, 1.0)
                
                if psim.Button("Run Processing"):
                    self.content.run_algorithm(self.slider_val, "method_A")
            
            psim.TreePop()
