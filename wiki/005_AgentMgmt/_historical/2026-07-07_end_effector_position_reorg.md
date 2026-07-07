---
status: retired
scope: historical-archive
supersedes: null
superseded_by: wiki/002_Architecture/settled.md#S1.1
---

# `end_effector_position` / module-level FK layout (pre-reorg)

Archived record of `geometry_backend.py`'s Stage-1 layout, before FK and
mesh-loading logic were consolidated onto `VisContent` as instance methods
(see [`settled.md#S1.1`](../../002_Architecture/settled.md)). Kept for
audit only — do not use as an implementation reference.

## Previous state

`compute_fk`, `DH_PARAMS`, and `dh_transform` were plain module-level
functions/constants (not attached to `VisContent`). `end_effector_position`
was a bare module-level function, and the bottom of the file ran it
unconditionally on every import:

```python
# FR5 standard DH parameters: (a_mm, alpha_rad, d_mm, theta_offset_rad)
# Source: docs/FR5_DH_Table.md
DH_PARAMS = [
    (0,    np.pi / 2, 152, 0),
    (-425, 0,         0,   0),
    (-395, 0,         0,   0),
    (0,    np.pi / 2, 102, 0),
    (0,   -np.pi / 2, 102, 0),
    (0,    0,         100, 0),
]

def dh_transform(a, alpha, d, theta):
    """Standard DH homogeneous transform, frame {i-1} -> {i}"""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,   sa,       ca,      d],
        [0,   0,        0,       1],
    ])

def compute_fk(joint_angles_deg):
    """
    joint_angles_deg: sequence of 6 joint angles in degrees [J1..J6]
    Returns [T_0_1, ..., T_0_6], each a 4x4 np.ndarray. T_0_6 (base->flange)
    is the last element
    """
    T = np.eye(4)
    T_0_i = []
    for (a, alpha, d, theta_offset), joint_deg in zip(DH_PARAMS, joint_angles_deg):
        theta = np.deg2rad(joint_deg) + theta_offset
        T = T @ dh_transform(a, alpha, d, theta)
        T_0_i.append(T)
    return T_0_i


def end_effector_position(joint_angles_deg):
    T_0_6 = (compute_fk(joint_angles_deg))[-1]
    print(T_0_6[:3, 3])


joint_angles_deg = [0, 0, 0, 0, 0, 0]
end_effector_position(joint_angles_deg)
```

This worked as long as `compute_fk`/`end_effector_position` stayed bare
module functions with no dependency on a `VisContent` instance — the
self-test ran fine on plain `import geometry_backend`.

## What changed and why

A manual edit moved `DH_PARAMS`, `compute_fk`, `end_effector_position`,
and the newly-added `load_mesh`/`load_data` (Stage 2 mesh loading) inside
the `VisContent` class body, but without adding `self` or qualifying
internal references (`DH_PARAMS`, `load_mesh`) with `self.` — which breaks,
since a class body is not an enclosing scope for its methods in Python.
The trailing self-test also broke at **import time**, since
`end_effector_position` no longer existed as a module-level name.

The fix (see `settled.md#S1.1`) made `compute_fk`, `end_effector_position`,
`load_mesh`, and `load_data` real `VisContent` instance methods (`self.*`),
and moved the self-test under `if __name__ == "__main__":` so
`python geometry_backend.py` still works as a standalone smoke test without
running on normal imports from `main.py`/`gui_panel.py`. `dh_transform`,
`MESH_DIR`, and `MESH_FILES` were kept as module-level (stateless/config),
unchanged from this previous layout.

`end_effector_position` itself was *not* deleted, despite compute_fk/mesh
loading being verified — it remains the only console feedback that
`compute_fk` responds to joint input until the Delta-transform wiring
(mesh rendering following joint angles) lands later in Stage 2.
