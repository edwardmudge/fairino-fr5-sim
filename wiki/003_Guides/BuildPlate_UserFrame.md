---
status: active
---

# How the Build Plate and User Frame Are Placed

## What it is

A static Bambu Lab build-plate mesh (`assets/buildPlate/BambuLab_BuildPlate.obj`)
and a fixed "User Frame" XYZ triad marking its corner. Unlike everything
else in the scene, neither one moves when the joint sliders change — the
plate isn't mounted on the arm, so there's no Delta transform to apply.

## Why a "user frame"

"User frame" is standard industrial-robot terminology (FANUC/UR/ABB) for
a reference frame, relative to the robot base, that workpiece geometry
and IK targets get defined against — see `GLOSSARY.md` §3. The same
concept appears in Craig's *Introduction to Robotics* under the name
**station frame** — a frame relating the robot's base frame to the task
(here, the build plate). Establishing one now, even translation-only,
gives Stage 4 (IK) a concrete point to target instead of raw base-frame
coordinates.

## Why this particular placement

`USER_FRAME_ORIGIN_MM` sits in the arm's natural (-X, -Y) reach
quadrant — the direction the tool already points at the zero pose and at
the `[0,0,0,0,90,0]` home pose (`GLOSSARY.md` §3) — rather than the
opposite quadrant. J1 only travels ±170°, so a target diametrically
opposite the rest direction can only be reached by winding J1 almost to
its limit, which leaves little room for the wrist joints to also pick a
free orientation. Keeping the plate on the same side as the natural rest
direction, at a radius well inside the reachable envelope, avoids that.

## How it's computed

`VisContent.load_build_plate()` (`geometry_backend.py`):

1. Builds `self.T_user_frame`, a 4x4 identity matrix with its translation
   column set to `USER_FRAME_ORIGIN_MM` — translation-only for now (see
   `settled.md` S1.2 for why a full matrix is stored anyway).
2. Loads the plate OBJ via the shared `load_mesh()` helper and adds
   `USER_FRAME_ORIGIN_MM` directly to its raw vertices. This works because
   the mesh's local origin `(0,0,0)` already sits almost exactly at the
   plate's front-left-top corner (checked directly with `trimesh` bounds:
   X:[0,258], Y:[-10,266], Z:[-0.75,0]) — no bounding-box correction
   needed, just a plain translate.
3. Registers the translated mesh once (`ps.register_surface_mesh`) — it's
   never updated again.
4. Calls `create_coordinate_frame(scale=USER_FRAME_SCALE_MM,
   origin=USER_FRAME_ORIGIN_MM, name="User Frame")` — the same
   generalised triad helper used for the TCP frame (`TCP_Frame.md`),
   reused here with no rotation since this transform doesn't have one yet.

`load_build_plate()` is defined directly below `create_coordinate_frame()`
in `geometry_backend.py`, and is called once from `__init__`, grouped
with the other static scene setup (before the articulated arm loads) —
and, critically, it is **not** one of the indices in
`apply_delta_transform()`'s loop, since it isn't attached to any joint.

## How to tune it

Two module-level constants in `geometry_backend.py`:

| Constant | Effect |
|---|---|
| `USER_FRAME_ORIGIN_MM` | Base-frame `[x, y, z]` translation to the plate's corner. Keep it in the same quadrant as the zero/home-pose TCP direction (see above) and within the ~830–1124mm reach envelope. |
| `USER_FRAME_SCALE_MM` | Length of the fixed axis triad drawn at the corner, world units (mm). |

## Code anchors

- `geometry_backend.py`: `load_build_plate()`, `USER_FRAME_ORIGIN_MM`,
  `USER_FRAME_SCALE_MM`, `self.T_user_frame`.
- `wiki/002_Architecture/settled.md` S1.2 — why this bypasses the Delta
  pipeline.
- `wiki/005_AgentMgmt/active/ctx_main/GLOSSARY.md` §3 — "User frame" term.
