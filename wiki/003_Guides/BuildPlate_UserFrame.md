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

**The value moved in Stage 7.1: `[-600, -300, 0]` → `[-570, -300, -100]`.**
The quadrant reasoning above is unchanged; what changed is the *radius*
margin. Reach is a property of the **wrist centre**, not the TCP, so it
depends on the tool offset — and the real tool=1 offset sits ~109mm further
in −Y than the placeholder it replaced. That consumed the margin: the far
corner of the bed needed a wrist centre **835.35mm** out against an
**820mm** envelope, and only 3 of 181,375 planar waypoints failed — but one
was waypoint 0, and IK aborts the whole path on the first failure
(`settled.md` S1.12). +30mm X restores reach (19.4mm was the minimum);
−100mm Z clears the residual posed-plate rejection. All 181,375 solve there.

Practical consequence when re-tuning this constant: check it against the
**wrist centre under the current TCP offset**, not the plate corner's
distance from the base. See `settled.md` S1.43.

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
3. Loads the plate OBJ via the shared `load_mesh()` helper. The mesh's
   local origin `(0,0,0)` sits almost exactly at the plate's
   front-left-**top** corner (checked directly with `trimesh` bounds:
   X:[0,258], Y:[-10,266], Z:[-0.75,0]) — but `position_mm` is meant to
   mark the plate's **resting/bottom** face, not its top. So the local
   vertices are shifted up by `PLATE_THICKNESS_MM` (0.75mm) in Z first —
   local Z=-0.75 (bottom) becomes local Z=0 — before mapping to world
   coordinates with the full homogeneous multiply `T_user_frame @
   [x,y,z,1]` (same pattern `load_gcode()` uses, S1.3). `load_gcode()`
   applies the identical shift to its plate-local waypoints, so the
   printed path lands on the plate's real top surface instead of
   `PLATE_THICKNESS_MM` below it.
4. Registers the mesh (`ps.register_surface_mesh("Build Plate", ...)`)
   and sets its color to `PLATE_COLOR`, a light cool gray distinct from
   the orange G-code print — Polyscope replaces any prior structure of
   the same name, so calling this repeatedly (e.g. from GUI buttons) is
   safe.
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
| **Load Saved Position** | `load_saved_build_plate_position()` reads that file (if present), applies it immediately, and syncs the input fields; on failure (no saved file yet) or success alike, `gui_panel.py` sets a `bp_status` message (on success, prompting a follow-up G-code reload — see below). |

Loading a saved position only ever happens on that explicit click —
`__init__` never reads the saved-position file automatically, so every
fresh start still begins from `USER_FRAME_ORIGIN_MM`/zero-rotation.

> ⚠ **No longer true — changed at v1.0 (`settled.md` S1.58).** Startup now calls
> `_load_startup_build_plate()`, which applies `saved_position.json` when it is
> readable and falls back to `USER_FRAME_ORIGIN_MM` only when it is absent or
> malformed, reporting which through `startup_plate_status`. The reason is that
> the shipped curved precompute caches are keyed on the plate pose and were
> solved at the saved frame, so booting at the constant meant a ~30-minute
> re-solve per layer on every first run.
>
> **Reset** still means `USER_FRAME_ORIGIN_MM`, so the pose described above is
> one click away. The reference table's "used when `load_build_plate()` is called
> argument-free (startup, Reset)" should now read **Reset only**.

**Move, Reset, and Load Saved Position do *not* reload the G-code
preview.** Each just sets a `bp_status` message prompting an explicit
"Load G-code preview" click — an already-loaded preview mesh is left
showing the *old* plate pose until that click happens. This supersedes
S1.8's original button-triggered auto-reload, which was removed by
`settled.md` S1.23 specifically because it made the plate move look
fully in sync (preview jumped correctly) while the toolpath IK
precompute/playback state (a separate concern, unaffected by this page)
silently went stale underneath it — see `settled.md` S1.22/S1.23 and
`wiki/003_Guides/Gcode_Toolpath.md`.

## How to tune it

Module-level constants in `geometry_backend.py`:

| Constant | Effect |
|---|---|
| `USER_FRAME_ORIGIN_MM` | Default base-frame `[x, y, z]` translation to the plate's corner, used when `load_build_plate()` is called argument-free (startup, Reset). `[-570, -300, -100]` since Stage 7.1. Keep it in the same quadrant as the zero/home-pose TCP direction (see above), and check the **whole toolpath's worst wrist centre** against the 820mm envelope — not the plate corner's own distance from the base. The margin depends on the TCP offset, so it must be re-checked whenever that changes (see "Why this particular placement"). |
| `USER_FRAME_SCALE_MM` | Length of the fixed axis triad drawn at the corner, world units (mm). |
| `FRAME_AXIS_RADIUS_RATIO` | Line thickness, as a fraction of the triad's own `scale` (axis length). Shared with the TCP Frame — see `TCP_Frame.md`'s "How to tune it". |
| `BUILD_PLATE_POSITION_FILE` | Path to the saved-position JSON (`assets/buildPlate/saved_position.json`) read/written by the Save/Load Position buttons. |
| `PLATE_THICKNESS_MM` | Measured thickness (mm) of `BambuLab_BuildPlate.obj`. Shifts the plate mesh and G-code waypoints up in their local Z before the `T_user_frame` transform, so `position_mm` marks the resting/bottom face and the print lands on the real top surface. |
| `PLATE_COLOR` | RGB (0-1) color applied to the "Build Plate" mesh via `set_color()` — light cool gray, distinct from `GCODE_COLOR`. |

## Changed in Stage 7.3 — the saved position is now the real calibrated frame

Everything above still describes the mechanism correctly. What changed is the
**value** in `assets/buildPlate/saved_position.json`, and it is now the only
place in the project holding a *measured* User Frame:

| | position (mm) | rpy (deg) | provenance |
|---|---|---|---|
| `USER_FRAME_ORIGIN_MM` (startup / **Reset**) | `[-570, -300, -100]` | `[0, 0, 0]` | **chosen**, tuned for planar reachability at Stage 7.1 |
| `saved_position.json` (**Load Saved Position**) | `[649.456, 133.762, 322.778]` | `[-0.369, 0.329, -89.080]` | **measured** — `docs/saved_coords_data_and_usage_EN.md` §1.1, `user_index=1`, read from the physical robot 2026-05-28 |

The previous saved value was the Stage 6.8 *demo* pose `[-570, -300, -200]` /
`[0, 0, 0]`, retained in the file under the inert `_legacy_stage6_8_demo_pose`
key (the loader reads only `position_mm` / `rpy_deg`, so it is a record, not a
selectable second slot).

Three things worth knowing:

- **No conversion was needed.** The doc's §3 `pose_to_matrix` is
  `R = Rz(rz) @ Ry(ry) @ Rx(rx)` over `[rx, ry, rz]`; step 1 above builds
  `R = Rz(yaw) @ Ry(pitch) @ Rx(roll)` over `[roll, pitch, yaw]`. Identical —
  verified to `max |ΔT| = 0.0`. The six numbers go straight into the file.
- **This is the first saved pose with a real rotation** (a ~89° yaw). Everything
  downstream already handled it — `create_coordinate_frame`'s `rotation` param,
  the `bp_target_rpy` field sync, the full-4x4 cache key — so no code changed.
- **`USER_FRAME_ORIGIN_MM` deliberately did *not* move with it.** It is the
  startup default that keeps the planar path solving; the real frame is opt-in
  per session. See `settled.md` **S1.45** for why, and for the reachability
  finding that makes the distinction matter.

## Code anchors

- `geometry_backend.py`: `load_build_plate()`, `save_build_plate_position()`,
  `load_saved_build_plate_position()`, `create_coordinate_frame()`
  (`rotation` param), `USER_FRAME_ORIGIN_MM`, `USER_FRAME_SCALE_MM`,
  `BUILD_PLATE_POSITION_FILE`, `PLATE_THICKNESS_MM`, `PLATE_COLOR`,
  `self.T_user_frame`.
- `gui_panel.py`: "Build Plate Orientation" panel (`bp_target_pos`,
  `bp_target_rpy`, `bp_status`).
- `wiki/002_Architecture/settled.md` S1.2 — why this bypasses the Delta
  pipeline; S1.6 — the rotation/position/persistence decisions above.
- `wiki/005_AgentMgmt/active/ctx_main/GLOSSARY.md` §3 — "User frame" term.
