import polyscope.imgui as psim
import numpy as np

from geometry_backend import USER_FRAME_ORIGIN_MM

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
        self.joint_angles = np.zeros(6)
        self.trajectory_enabled = True
        self.ik_target_pos = np.zeros(3)
        self.ik_target_rpy = np.zeros(3)
        self.ik_status = ""
        self.ik_solutions = []       # list of (angles, is_singular, raw_branch_index) from solve_ik_tcp
        self.ik_selected_index = 0
        self.bp_target_pos = np.array(USER_FRAME_ORIGIN_MM, dtype=float)
        self.bp_target_rpy = np.zeros(3)
        self.bp_status = ""
        self.is_playing = False
        self.playback_waypoint_index = 0
        self.playback_speed = 1.0
        self.toolpath_ik_solutions = []   # list of angle arrays, indexed [0]/[2]/[4] by the radio button loop
        self.selected_solution = 0

    def render(self):
        """This function needs to be called by Polyscope every frame"""
        self.content.record_trajectory_point()
        self.content.step_toolpath_ik_precompute()

        # 1. Panel title
        psim.TextUnformatted("Fairino FR5 Arm Control")
        psim.Separator()

        # 2. Trajectory section
        changed, self.trajectory_enabled = psim.Checkbox("Enable Trajectory", self.trajectory_enabled)
        if changed:
            self.content.set_trajectory_enabled(self.trajectory_enabled)
        psim.Separator()

        # 3. Data loading section
        if psim.TreeNode("I/O Operations"):
            if psim.Button("Load G-code preview"):
                self.content.load_gcode()
            
            psim.Spacing()
            psim.Spacing()

            if psim.Button("Run"):
                self.is_playing = True

            psim.SameLine()

            if psim.Button("Pause"):
                self.is_playing = False

            psim.SameLine()

            if psim.Button("Reset"):
                self.is_playing = False
                self.playback_waypoint_index = 0

            psim.SameLine()
            psim.TextUnformatted(f"Playback: {'Running' if self.is_playing else 'Paused'}")

            if psim.TreeNode("Toolpath Settings"):
                _, self.playback_speed = psim.SliderFloat("Speed", self.playback_speed, 0.1, 5.0)

                psim.Spacing()
                if self.content.precompute_running:
                    if psim.Button("Pause Precompute"):
                        self.content.pause_toolpath_ik_precompute()
                else:
                    if psim.Button("Run Precompute"):
                        self.content.run_toolpath_ik_precompute(JOINT_LIMITS)

                psim.SameLine()

                if psim.Button("Cancel Precompute"):
                    self.content.cancel_toolpath_ik_precompute()

                total = self.content.precompute_total
                fraction = (self.content.precompute_index / total) if total else 0.0
                psim.ProgressBar(fraction, overlay=f"{fraction * 100:.0f}%" if total else "")
                psim.TextUnformatted(self.content.precompute_status)

                psim.Spacing()
                for i, sol in enumerate(self.toolpath_ik_solutions):
                    changed, self.selected_solution = psim.RadioButton(
                        f"Solution {i+1}/{len(self.toolpath_ik_solutions)}: J1={sol[0]:.1f} J3={sol[2]:.1f} J5={sol[4]:.1f}",
                        self.selected_solution, i
                    )
                psim.TreePop()
            psim.TreePop()

        # 4. Build plate orientation section -- see settled.md S1.6.
        if psim.TreeNode("Build Plate Orientation"):
            _, self.bp_target_pos = psim.InputFloat3("Target Position (mm)", self.bp_target_pos)
            _, self.bp_target_rpy = psim.InputFloat3("Target RPY (deg)", self.bp_target_rpy)

            if psim.Button("Move"):
                self.content.load_build_plate(self.bp_target_pos, self.bp_target_rpy)
                self.content.load_gcode()  # Keep the toolpath preview in sync with the new pose
                self.bp_status = "Build plate moved"

            psim.SameLine()

            if psim.Button("Reset"):
                self.bp_target_pos = np.array(USER_FRAME_ORIGIN_MM, dtype=float)
                self.bp_target_rpy = np.zeros(3)
                self.content.load_build_plate()
                self.content.load_gcode()
                self.bp_status = "Reset to default"

            psim.Spacing()
            psim.Spacing()

            if psim.Button("Save Position"):
                self.content.save_build_plate_position(self.bp_target_pos, self.bp_target_rpy)
                self.bp_status = "Position saved"

            psim.SameLine()

            if psim.Button("Load Saved Position"):
                pos, rpy, status = self.content.load_saved_build_plate_position()
                if pos is not None:
                    self.bp_target_pos = pos
                    self.bp_target_rpy = rpy
                    self.content.load_gcode()
                else:
                    self.bp_status = status

            psim.Spacing()
            psim.Spacing()
            psim.TextUnformatted(self.bp_status)
            psim.TreePop()

        # 5. Forward kinematics section
        if psim.TreeNode("Forward Kinematics"):
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

            psim.Spacing()
            psim.Spacing()
            psim.TreePop()

        # 6. Inverse kinematics section -- target is the TCP pose, not the
        # flange (see docs/FR5_IK_Derivation.md).
        if psim.TreeNode("Inverse Kinematics"):
            _, self.ik_target_pos = psim.InputFloat3("Target Position (mm)", self.ik_target_pos)
            _, self.ik_target_rpy = psim.SliderFloat3("Target RPY (deg)", self.ik_target_rpy, -180, 180)

            if psim.Button("Solve IK"):
                self.ik_solutions, self.ik_status = self.content.solve_ik_tcp(
                    self.ik_target_pos, self.ik_target_rpy, JOINT_LIMITS)
                self.ik_selected_index = 0
                if self.ik_solutions:
                    self.joint_angles = self.ik_solutions[0][0]
                    self.content.update_arm(self.joint_angles)

            psim.TextUnformatted(self.ik_status)

            if self.ik_solutions:
                # No verified anatomical naming (shoulder/elbow/wrist left-right)
                # for this arm's branches -- label with ordinal + the three
                # sign-driven joints (J1/J3/J5) as a numeric fingerprint instead.
                n = len(self.ik_solutions)
                for i, (angles, singular, _) in enumerate(self.ik_solutions):
                    label = (
                        f"Solution {i + 1}/{n}: J1={angles[0]:6.1f} J3={angles[2]:6.1f} J5={angles[4]:6.1f}"
                        f"{'  [near singularity]' if singular else ''}"
                    )
                    changed, self.ik_selected_index = psim.RadioButton(label, self.ik_selected_index, i)
                    if changed:
                        self.joint_angles = angles
                        self.content.update_arm(self.joint_angles)

            psim.Spacing()
            psim.Spacing()
            psim.TreePop()
