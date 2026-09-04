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

Since Stage 7.1 the frame's initial basis is the **tool's own rest
orientation** — `(T_zero[5] @ T_flange_to_tcp)[:3, :3]`, centred on that
transform's translation — so the triad is visibly tilted at zero pose
rather than world-aligned. Blue Z is the nozzle approach axis (the nozzle
approaches along −Z, into the surface), which is what the curved path
targets per `settled.md` S1.36.

Per Craig, frame assignment is always a convention, not a physical
property of the link: any three orthonormal axes rigidly attached to the
flange rotate identically once carried through `Delta_6`, so the earlier
world-aligned basis was equally valid *for visualising orientation
change*. What makes the tool's own axes the better choice now is that
there is a real tool orientation to show — see "Changed in Stage 7.1"
below.

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

`load_data()` calls it once with `origin=T_zero_tcp[:3, 3]` (the same
zero-pose point the "TCP" point cloud uses) and
`rotation=T_zero_tcp[:3, :3]` to register the "TCP Frame" curve network,
then appends its rest-pose nodes / handle / `update_node_positions` to
the same `rest_verts` / `mesh_handles` / `update_fns` lists everything
else in `apply_delta_transform()` uses.

`apply_delta_transform()`'s loop (`range(10)` — **`range(9)` since Stage
7.7, index 9 is gone**) reaches the TCP frame at
index 8; `src = min(8, 5)` resolves to `T_current[5]` / `T_zero[5]`
(Delta_6) — exactly the same delta already applied to the nozzle (index
6), TCP point (index 7) and Tool Axis stalk (index 9, deleted at 7.7 —
the frame's own index 8 is unaffected). No special-casing
needed: the frame's four rest-pose nodes (origin + three axis tips, all
in the zero-pose world frame) go through
`Delta_6 = T_0_6(q) @ inv(T_0_6(0))` like any other flange-mounted
geometry, which is what makes the triad rotate correctly.

## How to tune it

Module-level constants in `geometry_backend.py`:

| Constant | Effect |
|---|---|
| `TCP_FRAME_SCALE_MM` | Axis length, in world units (mm). Larger = easier to see at a distance, but can visually clutter the tool at close range. |

`TOOL_AXIS_COLOR`/`TOOL_AXIS_RADIUS_MM` no longer exist — see "Changed in
Stage 7.7" below.

## Code anchors

- `geometry_backend.py`: `create_coordinate_frame()` (generalised),
  the "TCP Frame" and Nozzle registration blocks in `load_data()` (right
  after `T_flange_to_tcp` is built), and index 8 in
  `apply_delta_transform()`'s loop (`range(9)`).

## Changed in Stage 7.1 (2026-08-14)

Two things above were previously true and are no longer, corrected in
place rather than left to mislead. The superseded reasoning is kept here.

**The triad used to be world-aligned at zero pose, centred on
`tcp_local`.** `tcp_local` was a bare zero-pose *world* point loaded from
`assets/printerHead/TCP.txt`, with no rotation of its own, so
`create_coordinate_frame` was called with the default identity rotation
and the "arbitrary but equally valid basis" argument above carried the
justification. That reasoning was sound for a tool with no measured
orientation. Stage 7.1 wired in the real calibrated tool=1 offset, which
*does* carry an orientation (~87° / −13° / 61°), so the triad now shows
the tool's actual rest axes and `tcp_local` no longer exists. The
zero-pose TCP moved 310.97 mm as a result.

**Index 8 used to be the last entry, and the loop was `range(9)`.**
Stage 7.1 appended the "Tool Axis" stalk at index 9 — a 2-node curve
network from the flange origin to the TCP, 196.91 mm long. It exists
because `nozzle.obj` is now hidden (it is not the head tool=1 was
calibrated against), leaving nothing on screen to show where the tool
points. It is **visual only**: the tool's collision body is the single
TCP point, and the stalk is deliberately excluded from the clearance
set.

Full rationale and measurements: `settled.md` S1.43.

## Changed in Stage 7.7 (2026-09-04)

**The "Tool Axis" stalk described above is gone -- deleted, not hidden** --
and the Nozzle mesh takes its place. `nozzle_handle.set_enabled(False)` is
removed, `TOOL_AXIS_COLOR`/`TOOL_AXIS_RADIUS_MM` no longer exist, and
`apply_delta_transform()`'s loop is `range(9)` again -- index 9 is gone, not
merely never-yet-added as it was pre-7.1. The TCP Frame stays at index 8,
unaffected, and still takes the tool's own rest rotation.

The mesh is re-aimed at load time rather than rendered as exported, since its
native CAD pose targets the retired `TCP.txt` point rather than the real
tool=1 offset. Its tip is pinned onto `tcp_point` and its **shaft's** long
axis (`_nozzle_shaft_mask` + `_obb_from_points()`) laid along **this frame's
own -Z** -- the approach axis every curved `R_target` is built around (S1.36).
So the triad sits at the tool's tip and the nozzle body runs back along the
blue Z axis, collinear with it by construction: what the triad claims and
what the mesh shows are now the same thing, which is the point. Viewed down
the tool the blue axis foreshortens to almost nothing, which is the quickest
visual check that the alignment is intact.

The shaft rather than the whole mesh because the mounting bracket skews a
whole-mesh fit by 6.59 degrees, which would put the rendered shaft that far
off the commanded approach axis. Note the tool visibly floats clear of the
flange (nearest approach 98.33mm) -- an artefact of the placeholder asset's
wrong length and compound mount angle, accepted deliberately in favour of
showing the true orientation.

Full rationale and measurements: `settled.md` S1.51.
