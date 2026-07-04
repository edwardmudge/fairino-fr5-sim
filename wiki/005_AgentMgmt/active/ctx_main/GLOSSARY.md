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
| **IK (Inverse Kinematics)** | Target end-effector pose → joint angles. Up to 8 valid solutions for the FR5 (redundancy). | Not implemented yet (Stage 4) |

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

## 4. Degrees of freedom

| Term | Definition | Notes |
|------|-----------|-------|
| **6DOF** | The FR5 has 6 revolute joints (J1–J6), giving 6 degrees of freedom — enough to reach any position *and* orientation in its workspace. | Distinguish from a 6DOF *pose* `[x,y,z,rx,ry,rz]`, which is the IK target format, not the joint count |
