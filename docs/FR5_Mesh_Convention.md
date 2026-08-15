# FR5 Mesh Coordinate Convention

This is the **most important document** in this kit. Misunderstanding this will cause the robot to render incorrectly.

## Key Fact

**The OBJ mesh vertices are defined in the zero-configuration WORLD frame, NOT in each link's DH-local frame.**

The 7 OBJ files (`Robot0.obj` – `Robot6.obj`) were exported from a CAD assembly with the robot in its zero position (`joints = [0, 0, 0, 0, 0, 0]` degrees). Each link's vertices already include the cumulative DH transforms from the base to that link at zero pose.

## Wrong Approach

```python
# ❌ WRONG — "double transforms" the mesh, produces garbage
T_0_i = compute_fk(current_joints)[i]
new_verts = (T_0_i @ homogeneous_verts.T).T[:, :3]
```

This doesn't work because the vertices already encode `T_0_i(q=0)`. Applying `T_0_i(q)` on top gives `T_0_i(q) · T_0_i(0) · v_local` — one transform too many.

## Correct Approach: Delta Transform

```python
# ✅ CORRECT — use incremental (delta) transform

# Step 1: At init, compute zero-pose transforms ONCE
zero_joints = [0, 0, 0, 0, 0, 0]
T_zero = compute_fk(zero_joints)  # list of 6 matrices [T_0_1, ..., T_0_6]

# Step 2: Store the rest-pose vertices for each link
rest_verts = []  # list of Nx3 arrays, one per link
for mesh_file in ["Robot1.obj", ..., "Robot6.obj"]:
    m = trimesh.load(mesh_file, force='mesh')
    rest_verts.append(m.vertices.copy())

# Step 3: Each frame, compute delta and apply
T_current = compute_fk(current_joints)
for i in range(6):
    Delta = T_current[i] @ np.linalg.inv(T_zero[i])

    # Convert rest verts to homogeneous [x,y,z,1]
    N = rest_verts[i].shape[0]
    homo = np.hstack([rest_verts[i], np.ones((N, 1))])  # Nx4

    # Apply delta
    new_verts = (Delta @ homo.T).T[:, :3]  # Nx3

    # Update Polyscope mesh
    polyscope_mesh[i].update_vertex_positions(new_verts)
```

## Intuition

Think of the mesh as a "photograph" taken at zero pose:
1. `inv(T_zero[i])` — **undo** the zero-pose position (bring vertices back to link-local frame)
2. `T_current[i]` — **apply** the current-pose position

Combined: `Delta = T_current @ inv(T_zero)` does both in one step.

## Nozzle and TCP

The nozzle mesh (`nozzle.obj`) and TCP point (`TCP.txt`) follow the **same convention**:
- Their coordinates are in the zero-pose world frame
- Transform them using `Delta_flange = T_0_6(q) @ inv(T_0_6(0))`

```python
# Nozzle mesh
Delta_flange = T_current[5] @ np.linalg.inv(T_zero[5])
nozzle_new = (Delta_flange @ nozzle_homo.T).T[:, :3]

# TCP point from TCP.txt: [-798.137, -228.017, -109.903]
tcp_local = np.array([-798.137, -228.017, -109.903, 1.0])
tcp_world = (Delta_flange @ tcp_local)[:3]
```

Note: `TCP.txt` coordinates are near the zero-pose end-effector position `[-820, -202, 50]`, confirming they are in world-zero-pose space (not a small local offset).

## Robot0 (Base)

`Robot0.obj` is the static base. It does NOT move — do not apply any transform to it.
Just register it once and leave it.

## Mapping Summary

```
Robot0.obj → Fixed (no transform)
Robot1.obj → Delta_1 = T_0_1(q) @ inv(T_0_1(0))
Robot2.obj → Delta_2 = T_0_2(q) @ inv(T_0_2(0))
Robot3.obj → Delta_3 = T_0_3(q) @ inv(T_0_3(0))
Robot4.obj → Delta_4 = T_0_4(q) @ inv(T_0_4(0))
Robot5.obj → Delta_5 = T_0_5(q) @ inv(T_0_5(0))
Robot6.obj → Delta_6 = T_0_6(q) @ inv(T_0_6(0))
nozzle.obj → Same Delta_6 as Robot6
TCP.txt    → Delta_6 @ [x, y, z, 1]
```

---

## Changed in Stage 7.1 (2026-08-14)

Everything above still describes the Delta convention correctly — that is
unchanged and remains the hard rule. What changed is **what the tool is**.
The "Nozzle and TCP" section above is kept as the record of the old setup.

**`TCP.txt` is no longer read.** The TCP is now derived from the real
calibrated tool=1 offset, a module-level constant in `geometry_backend.py`:

```python
TCP_OFFSET_6D_MM_DEG = np.array([-134.777, 96.448, 106.334, 86.647, -13.136, 60.612])
T_flange_to_tcp = pose_to_matrix(*TCP_OFFSET_6D_MM_DEG)   # flange-local, mm/deg
T_zero_tcp = T_zero[5] @ T_flange_to_tcp                  # zero-pose world pose
```

Source: `docs/saved_coords_data_and_usage_EN.md` §1.2. This is a genuine
**flange-local 6D pose with rotation**, not a zero-pose world point — the
previous construction had to borrow its rotation from `inv(T_zero[5])`
(`settled.md` S1.4) and could not express a tool that is itself rotated
relative to the flange, which tool=1 is (~87° / −13° / 61°).

The zero-pose TCP world point therefore moves **310.97 mm**:

```
old (TCP.txt):  [-798.137, -228.017, -109.903]
new (tool=1) :  [-954.777, -308.334,  146.448]
```

`TCP.txt` is retained on disk with a legacy header, as a record.

**`nozzle.obj` is registered but hidden** (`set_enabled(False)`). The
supervisor confirmed on 2026-08-14 that the tool=1 calibration is correct, so
the 33.4 mm flange-to-tip disagreement (asset 163.47 mm vs tool=1 196.91 mm)
means this asset is simply not the head that was calibrated. Magnitude is
frame-independent, so it was never a convention problem. The mesh stays wired
into `rest_verts`/`update_fns` so a corrected asset only needs the flag
flipped.

**Two new flange-mounted structures**, both following the same Delta
convention as everything else here:

```
TCP point   → Delta_6, at T_zero_tcp[:3, 3]            (index 7)
TCP Frame   → Delta_6, triad now tilted by T_zero_tcp[:3, :3]
                       rather than world-aligned        (index 8)
Tool Axis   → Delta_6, 2-node line: flange origin → TCP (index 9, new)
```

`apply_delta_transform`'s loop is `range(10)`, not `range(9)`.

**Collision geometry is now a separate list from render geometry.** The tool's
collision body is the single **TCP point**, not the nozzle mesh — colliding
against a hidden asset of the wrong length would reject poses on geometry the
real head does not have. `rest_verts[6]` is still the nozzle's render buffer
(its `update_fns` entry needs the full vertex count), so the clearance checks
index `moving_geometry_rest_verts` = 6 arm links + the TCP point instead. The
Tool Axis stalk is **visual only** and deliberately absent from that set.

Full rationale and measurements: `settled.md` S1.43.
