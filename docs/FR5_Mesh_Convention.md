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
