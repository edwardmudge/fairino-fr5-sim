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
| **G-code toolpath** | Preview curve of a parsed G-code file's `G1` (feed) moves, registered per "Load G-code preview" click (not live per-frame like the TCP trajectory) — also re-registered whenever the build plate is repositioned (Move/Reset/Load Saved Position), so it stays in sync. Modal position convention: an axis omitted from a line keeps its *last* value, not 0 — only the very first line defaults omitted axes to 0. `G0` (travel) moves update this modal position but are not drawn. | `parse_gcode()`/`load_gcode()` in `geometry_backend.py`, loads `assets/models/gcode/model.gcode` (fixed name every Cura export overwrites); fixture at `assets/models/gcode/square_test.gcode`; see `settled.md` S1.3/S1.7/S1.8 and `wiki/003_Guides/Gcode_Toolpath.md` |

## 4. Degrees of freedom

| Term | Definition | Notes |
|------|-----------|-------|
| **6DOF** | The FR5 has 6 revolute joints (J1–J6), giving 6 degrees of freedom — enough to reach any position *and* orientation in its workspace. | Distinguish from a 6DOF *pose* `[x,y,z,rx,ry,rz]`, which is the IK target format, not the joint count |
