---
status: active
---

# Glossary

Only terms that are easy to confuse or get subtly wrong. Not a full
robotics dictionary.

## 1. Kinematics direction

| Term | Definition | Notes |
|------|-----------|-------|
| **FK (Forward Kinematics)** | Joint angles → end-effector pose. One unique answer. | `docs/FR5_DH_Table.md`, implemented in `geometry_backend.py` |
| **IK (Inverse Kinematics)** | Target end-effector pose → joint angles. Up to 8 valid solutions for the FR5 (redundancy). | Implemented — `solve_ik`/`solve_ik_tcp` in `geometry_backend.py`; see `settled.md` S1.4/S1.5 |

## 2. Transform frames — the mesh-rendering trap

| Term | Definition | Notes |
|------|-----------|-------|
| **`T_0_i`** | Absolute DH transform: base frame → link *i* frame, at the *current* joint angles. | From `compute_fk()`, per `docs/FR5_DH_Table.md` |
| **`T_zero`** | `T_0_i` evaluated at `joints = [0,0,0,0,0,0]`. | Computed once at init |
| **Delta transform** | `Delta_i = T_0_i(q) @ inv(T_0_i(0))`. The *only* correct transform to apply to the OBJ mesh vertices. | See `docs/FR5_Mesh_Convention.md` — **do not** apply `T_0_i(q)` directly to mesh vertices, they already encode `T_0_i(0)`; direct application double-transforms and produces garbage geometry |

## 3. Points of interest

| Term | Definition | Notes |
|------|-----------|-------|
| **TCP (Tool Centre Point)** | The functional tip of whatever's mounted on the flange (here: the nozzle tip), not the flange centre itself. | Coordinate in `assets/printerHead/TCP.txt`, transformed with `Delta_6` |
| **Flange** | The mounting face at the end of Link 6, where tools attach. | Frame = `T_0_6` |
| **Home position** | `[0, 0, 0, 0, 90, 0]` degrees — J5=90° points the tool straight down. | `docs/FR5_Joint_Limits.md` |
| **User frame** | A reference frame (position + rotation) from the base frame to a workpiece origin — here, the build-plate corner. Standard industrial-robot term (FANUC/UR/ABB all have one); Craig's *Introduction to Robotics* calls the same concept a **station frame**. Re-posable (position and XYZ fixed-angle rotation), stored as `self.T_user_frame`. | `USER_FRAME_ORIGIN_MM`, `load_build_plate()` in `geometry_backend.py`; see `settled.md` S1.2/S1.6 |
| **G-code toolpath** | Preview curve of a parsed G-code file's `G1` (feed) moves, registered per "Load G-code preview" click (not live per-frame like the TCP trajectory). Does **not** auto-refresh when the build plate is repositioned — Move/Reset/Load Saved Position all require an explicit follow-up "Load G-code preview" click (S1.8's auto-reload was removed by S1.23). Modal position convention: an axis omitted from a line keeps its *last* value, not 0 — only the very first line defaults omitted axes to 0. `G0` (travel) moves update this modal position but are not drawn. | `parse_gcode()`/`load_gcode()`/`clear_gcode_preview()` in `geometry_backend.py`, loads `assets/models/planar/gcode/model.gcode` (fixed name every Cura export overwrites); see `settled.md` S1.3/S1.7/S1.8/S1.23 and `wiki/003_Guides/Gcode_Toolpath.md` |

## 4. Degrees of freedom

| Term | Definition | Notes |
|------|-----------|-------|
| **6DOF** | The FR5 has 6 revolute joints (J1–J6), giving 6 degrees of freedom — enough to reach any position *and* orientation in its workspace. | Distinguish from a 6DOF *pose* `[x,y,z,rx,ry,rz]`, which is the IK target format, not the joint count |

## 5. Toolpath execution — precompute, cache, playback

| Term | Definition | Notes |
|------|-----------|-------|
| **Toolpath IK precompute** | Solving IK for every G-code waypoint ahead of playback, chunked across frames (`PRECOMPUTE_CHUNK_SIZE` waypoints/call) so it doesn't block the GUI. Pausable/resumable/cancellable; aborts the whole precompute (no partial motion) at the first waypoint with no valid, ground-clearing branch. | `run_/step_/pause_/cancel_toolpath_ik_precompute()` in `geometry_backend.py`; result stored in `precompute_joint_path`; see `settled.md` S1.14/S1.15 |
| **Precompute disk cache** | Cross-session cache of a completed precompute, keyed on `{version, gcode_sha256, user_frame}` (G-code file content hash + build-plate pose, not mtime). A hit skips parsing and IK entirely. | `assets/models/planar/gcode/model.precompute.npz`, `save_/load_toolpath_precompute_cache()`, `_toolpath_cache_meta()`; see `settled.md` S1.21 |
| **`precompute_cache_meta`** | The cache key dict captured once, at precompute-start (or on a disk-cache hit) — used later to detect a plate move that invalidates the in-memory precompute mid-session, not just across sessions. | `geometry_backend.py`; see `settled.md` S1.22 |
| **Toolpath playback** | Driving the arm through `precompute_joint_path` while progressively revealing the printed bead mesh — beads start collapsed to zero-area (invisible) and are restored to their real vertex positions as `playback_index` crosses each bead's reveal point. | `run_/pause_/reset_/advance_toolpath_playback()` in `geometry_backend.py`; see `settled.md` S1.16 |
| **`playback_index` / `playback_running`** | Current waypoint index into `precompute_joint_path`, and whether playback is actively advancing. `playback_index` persists across pause; only `reset_toolpath_playback()` zeroes it. Indexes `precompute_joint_path` directly, so both must be reset together whenever that list is discarded (`_reset_toolpath_playback_state()`) — a stale index into an emptied path was the project's one confirmed GUI crash bug (fixed; see `settled.md` S1.24). | `geometry_backend.py` |
| **`PLAYBACK_RENDER_STRIDE` / `PLAYBACK_LOOKAHEAD_BEADS`** | Two independent render-cost throttles: the first limits how often playback pushes a Polyscope update (vs. every step); the second limits how far ahead of actual progress the registered bead mesh is grown, so registered mesh size — not just push frequency — tracks progress. | `geometry_backend.py`; see `settled.md` S1.17–S1.20 |
