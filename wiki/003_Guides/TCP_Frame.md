---
status: active
---

# How to Read the TCP Coordinate Frame

## What it is

A small red/green/blue XYZ triad ("TCP Frame" curve network) rigidly
attached to the tool tip. Unlike the static "Coordinate Frame" triad at
the world origin, this one moves *and* rotates with the arm — it always
shows the current orientation of the flange/tool, not just its position.

## Why it rotates, not just translates

Craig's *Introduction to Robotics* describes the pose of a rigid body
relative to a reference frame as a single homogeneous transform combining
a rotation matrix (orientation) and a position vector (translation) —
neither one alone is a complete description of where a frame is. Forward
kinematics exists precisely to compute that whole transform,
`T_0_6(q)` (frame {6} relative to base frame {0}), not just the position
of a point.

`compute_fk()` already returns the full `T_0_6(q)` for every joint
configuration — the rotation submatrix comes for free alongside the
translation. Building the TCP frame as a translate-only marker (axes
always aligned to world X/Y/Z, just repositioned at `tcp_world`) would
silently throw that rotation away, even though it's already sitting in
`T_current[5]`. It would also break the project's Delta-transform
convention (`docs/FR5_Mesh_Convention.md`): `tcp_local`-adjacent points are
zero-pose-frame geometry and must move via `Delta_6`, not via a bespoke
translation.

The frame's initial basis (world-aligned axes at zero pose, centred on
`tcp_local`) is an arbitrary choice — per Craig, frame assignment is
always a convention, not a physical property of the link. Any three
orthonormal axes rigidly attached to the flange rotate identically once
carried through `Delta_6`, so a world-aligned zero-pose basis is just as
valid a "frame 6" as the tool's real CAD axes would be for the purpose of
visualising orientation change.

This will matter directly once Stage 4 (IK) starts: IK targets are a
desired *pose* (position + orientation), and being able to see the tool's
current orientation — not just its tip position — is what lets you
visually confirm an orientation error, not just a position error.

## How it's computed

`VisContent.create_coordinate_frame(scale, origin, rotation, name)`
(`geometry_backend.py`) builds an axis triad — the same node/edge/color
logic used for the static world-origin frame, generalised to take a
custom `origin`, `scale`, optional `rotation` (3x3, added for the build
plate's tiltable "User Frame" triad — see `settled.md` S1.6; defaults to
identity/axis-aligned for every other caller including this one), and
Polyscope structure `name`, and to return the raw `nodes` array alongside
the handle.

`load_data()` calls it once with `origin=self.tcp_local` (the same
zero-pose point the "TCP" point cloud uses) to register the "TCP Frame"
curve network, then appends its rest-pose nodes / handle /
`update_node_positions` to the same `rest_verts` / `mesh_handles` /
`update_fns` lists everything else in `apply_delta_transform()` uses.

`apply_delta_transform()`'s loop (`range(9)`) reaches the TCP frame at
index 8; `src = min(8, 5)` resolves to `T_current[5]` / `T_zero[5]`
(Delta_6) — exactly the same delta already applied to the nozzle (index
6) and TCP point (index 7). No special-casing needed: the frame's four
rest-pose nodes (origin + three axis tips, all in the zero-pose world
frame) go through `Delta_6 = T_0_6(q) @ inv(T_0_6(0))` like any other
flange-mounted geometry, which is what makes the triad rotate correctly.

## How to tune it

One module-level constant in `geometry_backend.py`:

| Constant | Effect |
|---|---|
| `TCP_FRAME_SCALE_MM` | Axis length, in world units (mm). Larger = easier to see at a distance, but can visually clutter the nozzle at close range. |

## Code anchors

- `geometry_backend.py`: `create_coordinate_frame()` (generalised),
  the "TCP Frame" registration block in `load_data()` (right after
  `self.tcp_local` is loaded), and index 8 in `apply_delta_transform()`'s
  loop.
