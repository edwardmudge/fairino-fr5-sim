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

`VisContent.load_build_plate(position_mm=USER_FRAME_ORIGIN_MM,
rpy_deg=(0.0, 0.0, 0.0))` (`geometry_backend.py`) — re-posable, not a
one-time setup call (see `settled.md` S1.6):

1. Converts `rpy_deg` (`[roll, pitch, yaw]`, degrees) into a rotation
   matrix `R = Rz(yaw) @ Ry(pitch) @ Rx(roll)` — the **XYZ fixed-angle**
   convention, the same one `solve_ik_tcp` already uses for its target
   RPY, reusing the shared `rot_x`/`rot_y`/`rot_z` helpers.
2. Builds `self.T_user_frame`, a full 4x4 matrix: rotation submatrix `R`,
   translation column `position_mm`.
3. Loads the plate OBJ via the shared `load_mesh()` helper and maps its
   raw vertices to world coordinates with the full homogeneous multiply
   `T_user_frame @ [x,y,z,1]` (same pattern `load_gcode()` uses, S1.3) —
   no longer a raw vector add, since there's now a rotation to apply too.
   The mesh's local origin `(0,0,0)` still sits almost exactly at the
   plate's front-left-top corner (checked directly with `trimesh` bounds:
   X:[0,258], Y:[-10,266], Z:[-0.75,0]), so `position_mm` alone (at
   zero rotation) reproduces the old placement exactly.
4. Registers the mesh (`ps.register_surface_mesh("Build Plate", ...)`) —
   Polyscope replaces any prior structure of the same name, so calling
   this repeatedly (e.g. from GUI buttons) is safe.
5. Calls `create_coordinate_frame(scale=USER_FRAME_SCALE_MM,
   origin=position_mm, rotation=R, name="User Frame")` — the same
   generalised triad helper used for the TCP frame (`TCP_Frame.md`), now
   passing `rotation` so the triad tilts with the plate.

`load_build_plate()` is defined directly below `create_coordinate_frame()`
in `geometry_backend.py`. `__init__` still calls it once, argument-free,
grouped with the other static scene setup (before the articulated arm
loads) — the defaults reproduce the original translation-only placement
exactly. It is **not** one of the indices in `apply_delta_transform()`'s
loop, since it isn't attached to any joint.

### GUI panel ("Build Plate Orientation", `gui_panel.py`)

Four buttons, all **click-to-apply** (matching the "Solve IK" button
pattern, not the Forward Kinematics panel's live-drag `changed_any`
pattern) — position/RPY are plain `InputFloat3` fields, not sliders:

| Button | Effect |
|---|---|
| **Move** | `load_build_plate(bp_target_pos, bp_target_rpy)` with the current field values. |
| **Reset** | Resets the fields to `USER_FRAME_ORIGIN_MM`/zero-rotation, then calls `load_build_plate()` argument-free — back to the exact original placement. |
| **Save Position** | `save_build_plate_position(...)` writes the current field values to `assets/buildPlate/saved_position.json`. |
| **Load Saved Position** | `load_saved_build_plate_position()` reads that file (if present), applies it immediately, and syncs the input fields; shows a status message either way. |

Loading a saved position only ever happens on that explicit click —
`__init__` never reads the saved-position file automatically, so every
fresh start still begins from `USER_FRAME_ORIGIN_MM`/zero-rotation.

G-code loaded before a Move/Reset/Load click does **not** currently
re-transform with the plate — a known gap, deferred to roadmap Stage 5.3
(`wiki/001_Inbox/2026-07-09_2d3d_printing_roadmap.md`).

## How to tune it

Module-level constants in `geometry_backend.py`:

| Constant | Effect |
|---|---|
| `USER_FRAME_ORIGIN_MM` | Default base-frame `[x, y, z]` translation to the plate's corner, used when `load_build_plate()` is called argument-free (startup, Reset). Keep it in the same quadrant as the zero/home-pose TCP direction (see above) and within the ~830–1124mm reach envelope. |
| `USER_FRAME_SCALE_MM` | Length of the fixed axis triad drawn at the corner, world units (mm). |
| `BUILD_PLATE_POSITION_FILE` | Path to the saved-position JSON (`assets/buildPlate/saved_position.json`) read/written by the Save/Load Position buttons. |

## Code anchors

- `geometry_backend.py`: `load_build_plate()`, `save_build_plate_position()`,
  `load_saved_build_plate_position()`, `create_coordinate_frame()`
  (`rotation` param), `USER_FRAME_ORIGIN_MM`, `USER_FRAME_SCALE_MM`,
  `BUILD_PLATE_POSITION_FILE`, `self.T_user_frame`.
- `gui_panel.py`: "Build Plate Orientation" panel (`bp_target_pos`,
  `bp_target_rpy`, `bp_status`).
- `wiki/002_Architecture/settled.md` S1.2 — why this bypasses the Delta
  pipeline; S1.6 — the rotation/position/persistence decisions above.
- `wiki/005_AgentMgmt/active/ctx_main/GLOSSARY.md` §3 — "User frame" term.
