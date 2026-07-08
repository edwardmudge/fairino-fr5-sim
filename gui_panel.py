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

HOME_JOINT_ANGLES = [0, 0, 0, 0, 90, 0]  # docs/FR5_Joint_Limits.md "Home Position"

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
        self.trajectory_enabled = True
        self.ik_target_pos = np.zeros(3)
        self.ik_target_rpy = np.zeros(3)
        self.ik_status = ""

    def render(self):
        """This function needs to be called by Polyscope every frame"""
        self.content.record_trajectory_point()

        # 1. Panel title
        psim.TextUnformatted("GeoProc Template Control")
        psim.Separator()

        # 2. Trajectory section
        changed, self.trajectory_enabled = psim.Checkbox("Enable Trajectory", self.trajectory_enabled)
        if changed:
            self.content.set_trajectory_enabled(self.trajectory_enabled)
        psim.Separator()

        # 3. Data loading section
        if psim.TreeNode("I/O Operations"):
            if psim.Button("Load G-code"):
                self.content.load_gcode()
            psim.TreePop()

        # 4. Joint angle control section
        if psim.TreeNode("Joint Angles"):
            changed_any = False
            for i in range(6):
                lo, hi = JOINT_LIMITS[i]
                changed, self.joint_angles[i] = psim.SliderFloat(f"J{i+1}", self.joint_angles[i], lo, hi)
                changed_any = changed_any or changed

            if changed_any:
                self.content.update_arm(self.joint_angles)

            if psim.Button("Reset"):
                self.joint_angles = np.array(HOME_JOINT_ANGLES, dtype=float)
                self.content.update_arm(self.joint_angles)
                self.content.clear_trajectory()

            psim.TreePop()

        # 5. Inverse kinematics section -- target is the TCP pose, not the
        # flange (see docs/FR5_IK_Derivation.md); RPY reuses the same
        # -180..180 slider convention as the Transformation section below.
        if psim.TreeNode("Inverse Kinematics"):
            _, self.ik_target_pos = psim.InputFloat3("Target Position (mm)", self.ik_target_pos)
            _, self.ik_target_rpy = psim.SliderFloat3("Target RPY (deg)", self.ik_target_rpy, -180, 180)

            if psim.Button("Solve IK"):
                solution, self.ik_status = self.content.solve_ik_tcp(
                    self.ik_target_pos, self.ik_target_rpy, JOINT_LIMITS)
                if solution is not None:
                    self.joint_angles = solution
                    self.content.update_arm(self.joint_angles)

            psim.TextUnformatted(self.ik_status)
            psim.TreePop()

        # 6. Transformation control section
        if psim.TreeNode("Transformation"):
            changed_t, self.trans_vec = psim.InputFloat3("Translate", self.trans_vec)
            changed_r, self.rot_vec = psim.SliderFloat3("Rotate", self.rot_vec, -180, 180)
            
            # If the user has interacted with the UI, notify the backend immediately
            if changed_t or changed_r:
                self.content.update_transformation(self.rot_vec, self.trans_vec)
            
            psim.TreePop()

        # 7. Algorithm parameters section
        if psim.TreeNode("Algorithm Settings"):
            _, self.show_settings = psim.Checkbox("Enable Advanced", self.show_settings)
            
            if self.show_settings:
                _, self.slider_val = psim.SliderFloat("Smoothness", self.slider_val, 0.0, 1.0)
                
                if psim.Button("Run Processing"):
                    self.content.run_algorithm(self.slider_val, "method_A")
            
            psim.TreePop()