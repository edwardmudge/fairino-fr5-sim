import polyscope.imgui as psim
import numpy as np

from geometry_backend import USER_FRAME_ORIGIN_MM, PRECOMPUTE_CHUNK_SIZE

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
        self.playback_speed = 1.0   # whole-steps-per-frame multiplier, 1-100
        # -- snapped down automatically if it ever outruns precompute
        self.geodesic_sample_layer = 0  # which layer "Show Sample Geodesic" draws on
        self.geodesic_sample_most_curved = False  # False = a realistic travel move, True = the highest-ratio pair

    def _section_gap(self):
        """Uniform small gap between the numbered top-level sections below."""
        psim.Spacing()
        psim.Spacing()

    def _clear_ik_solutions(self):
        """Discard the current IK solution list -- called wherever the pose
        or target it was solved for stops being current, so a stale
        solution can't be applied via a leftover radio button."""
        self.ik_solutions = []
        self.ik_status = ""
        self.ik_selected_index = 0

    def render(self):
        """This function needs to be called by Polyscope every frame"""
        self.content.record_trajectory_point()
        self.content.step_toolpath_ik_precompute()
        self.content.step_geodesic_precompute()
        self.content.advance_toolpath_playback(max(1, int(self.playback_speed)))
        if self.content.playback_waiting:
            # Snap down reactively the moment playback hits precompute's throughput
            self.playback_speed = min(self.playback_speed, float(PRECOMPUTE_CHUNK_SIZE))

        if not self.content.playback_running:
            # Keep the FK sliders following the arm's real pose whenever
            # something other than these sliders could be driving it
            # (playback). A no-op once the sliders themselves are the last
            # thing to call update_arm(), since that keeps the two in sync.
            self.joint_angles = np.array(self.content.current_joint_angles, dtype=float)

        # 1. Panel title
        psim.TextUnformatted("Fairino FR5 Arm Control")
        psim.Separator()
        self._section_gap()

        # 2. Trajectory section
        changed, self.trajectory_enabled = psim.Checkbox("Enable Trajectory", self.trajectory_enabled)
        if changed:
            self.content.set_trajectory_enabled(self.trajectory_enabled)
        psim.Separator()
        self._section_gap()

        # 3. Data loading section
        if psim.TreeNode("I/O Operations"):
            if psim.Button("Load G-code preview"):
                self.content.load_gcode()

            if self.content.gcode_preview_loaded:
                psim.SameLine()
                if psim.Button("Clear G-code preview"):
                    self.content.clear_gcode_preview()

            if psim.Button("Load Curved Model"):
                self.content.load_curved_model()

            # Minimal geodesic controls for roadmap 6.2's verify step, same
            # spirit as the bare "Load Curved Model" button above -- the layer
            # selector and Clear pair are 6.6.
            if self.content.curved_model_loaded:
                if self.content.geodesic_running:
                    if psim.Button("Pause Geodesics"):
                        self.content.pause_geodesic_precompute()
                else:
                    if psim.Button("Build Geodesics"):
                        self.content.run_geodesic_precompute()

                psim.SameLine()

                if psim.Button("Cancel Geodesics"):
                    self.content.cancel_geodesic_precompute()

                if self.content.geodesic_loaded:
                    psim.SameLine()
                    if psim.Button("Show Sample Geodesic"):
                        self.content.show_sample_geodesic(
                            layer=self.geodesic_sample_layer,
                            mode="most_curved" if self.geodesic_sample_most_curved else "representative")

                    # Which layer the sample draws on. The sample isolates its
                    # host surface, so RX is viewable despite sitting inside
                    # Surface_TX_Base. A selector gating load/precompute/
                    # playback is still 6.6's job -- this one only picks the
                    # sample.
                    for i, layer_name in enumerate(self.content.curved_layer_names):
                        if i:
                            psim.SameLine()
                        _, self.geodesic_sample_layer = psim.RadioButton(
                            layer_name, self.geodesic_sample_layer, i)

                    psim.SameLine()
                    _, self.geodesic_sample_most_curved = psim.Checkbox(
                        "Most-curved pair", self.geodesic_sample_most_curved)

                total = self.content.geodesic_total
                fraction = (self.content.geodesic_index / total) if total else 0.0
                psim.ProgressBar(fraction, overlay=f"{fraction * 100:.0f}%" if total else "")

            psim.TextWrapped(self.content.geodesic_status)

            psim.Spacing()
            psim.Spacing()

            if self.content.playback_running:
                if psim.Button("Pause"):
                    self.content.pause_toolpath_playback()
            else:
                if psim.Button("Run Toolpath"):
                    self.content.run_toolpath_playback()
                    if self.content.playback_running:
                        # Only clear IK view state if playback actually
                        # started (e.g. not "Run Precompute first") -- the
                        # FK sliders resync to the real pose automatically
                        # once playback stops (see the top of render()).
                        self.ik_target_pos = np.zeros(3)
                        self.ik_target_rpy = np.zeros(3)
                        self._clear_ik_solutions()

            psim.SameLine()

            if psim.Button("Reset Toolpath"):
                self.content.reset_toolpath_playback()

            psim.SameLine()
            psim.TextWrapped(self.content.playback_status)

            if psim.TreeNode("Toolpath Settings"):
                _, self.playback_speed = psim.SliderFloat("Speed", self.playback_speed, 1.0, 100.0)

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
                psim.TextWrapped(self.content.precompute_status)
                psim.TreePop()
            psim.TreePop()
        self._section_gap()

        # 4. Build plate orientation section -- see settled.md S1.6.
        if psim.TreeNode("Build Plate Orientation"):
            # Disabled during playback -- moving the plate mid-print
            # invalidates and cancels the running toolpath (settled.md
            # S1.22), same reason FK/IK are disabled below.
            psim.BeginDisabled(self.content.playback_running)

            _, self.bp_target_pos = psim.InputFloat3("Target Position (mm)", self.bp_target_pos)
            _, self.bp_target_rpy = psim.InputFloat3("Target RPY (deg)", self.bp_target_rpy)

            if psim.Button("Move"):
                self.content.load_build_plate(self.bp_target_pos, self.bp_target_rpy)
                self.bp_status = "Build plate moved"

            psim.SameLine()

            if psim.Button("Reset"):
                self.bp_target_pos = np.array(USER_FRAME_ORIGIN_MM, dtype=float)
                self.bp_target_rpy = np.zeros(3)
                self.content.load_build_plate()
                self.bp_status = "Reset to default"

            psim.Spacing()
            psim.Spacing()

            if psim.Button("Save Position"):
                self.bp_status = self.content.save_build_plate_position(self.bp_target_pos, self.bp_target_rpy)

            psim.SameLine()

            if psim.Button("Load Saved Position"):
                pos, rpy, status = self.content.load_saved_build_plate_position()
                if pos is not None:
                    self.bp_target_pos = pos
                    self.bp_target_rpy = rpy
                    self.bp_status = "Loaded saved position"
                else:
                    self.bp_status = status

            psim.Spacing()
            psim.Spacing()
            psim.TextWrapped(self.bp_status)
            psim.EndDisabled()
            psim.TreePop()
        self._section_gap()

        # 5. Forward kinematics section
        if psim.TreeNode("Forward Kinematics"):
            # Disabled during playback -- these sliders would otherwise
            # fight advance_toolpath_playback() for the arm's pose.
            psim.BeginDisabled(self.content.playback_running)

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
                # Any IK solutions on screen were solved for the pre-reset
                # pose -- applying one now would silently undo this reset.
                self._clear_ik_solutions()

            psim.EndDisabled()
            psim.TreePop()
        self._section_gap()

        # 6. Inverse kinematics section -- target is the TCP pose, not the
        # flange (see docs/FR5_IK_Derivation.md).
        if psim.TreeNode("Inverse Kinematics"):
            # Disabled during playback, same reason as Forward Kinematics above.
            psim.BeginDisabled(self.content.playback_running)

            target_changed_pos, self.ik_target_pos = psim.InputFloat3("Target Position (mm)", self.ik_target_pos)
            target_changed_rpy, self.ik_target_rpy = psim.SliderFloat3("Target RPY (deg)", self.ik_target_rpy, -180, 180)
            if target_changed_pos or target_changed_rpy:
                # Existing solutions were solved for the old target -- stop
                # showing them as if they applied to the one on screen now.
                self._clear_ik_solutions()

            if psim.Button("Solve IK"):
                self.ik_solutions, self.ik_status = self.content.solve_ik_tcp(
                    self.ik_target_pos, self.ik_target_rpy, JOINT_LIMITS)
                self.ik_selected_index = 0
                if self.ik_solutions:
                    self.joint_angles = self.ik_solutions[0][0].copy()
                    self.content.update_arm(self.joint_angles)

            psim.TextWrapped(self.ik_status)

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
                        self.joint_angles = angles.copy()
                        self.content.update_arm(self.joint_angles)

            psim.EndDisabled()
            psim.TreePop()
        self._section_gap()
