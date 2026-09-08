---
status: active
---

# How the Curved-Surface Model Is Loaded and Placed

`load_curved_model()` and the geodesic engine it feeds are generic,
project-agnostic simulator features (settled.md S1.33) — they operate on
whatever `CURVED_LAYERS` describes. The RX/TX specifics this guide's
examples use come from `examples/curved_surface_printing/study_config.py`,
the shoulder-sensor study's own configuration; a different curved-print job
would supply a different config module.

## What it is

The "Load Curved Model" button (I/O Operations panel) parses 55
toolpath-curve PLY files (`RX_0.ply`…`RX_27.ply`, `TX_0.ply`…`TX_26.ply`)
and 3 surface OBJ meshes (`Surface_RX_Offset.obj`, `Surface_TX_Base.obj`,
`Surface_Bot.obj`) from `assets/models/curved/`, reconstructs the PLYs'
disjoint edge soup into 70 walkable polylines, and places the whole
assembly above the build plate — two curve networks (`Curved Toolpath
RX`/`Curved Toolpath TX`) and three surface meshes (`Surface RX Offset`,
`Surface TX Base`, `Surface Bot`). This is roadmap `Stage6_README.md`
sub-stage 6.1 — getting the curved-surface print target on screen, nothing
about routing, print ordering, per-waypoint orientation, or IK/playback
(those are 6.2–6.5).

The physical context: an elastomeric sensor printed conformally onto a
mockup of a human shoulder — `Surface_RX_Offset`/`Surface_TX_Base` are the
two print surfaces (RX/TX read as two stacked electrode layers), and
`Surface_Bot` is the underlying shoulder body, not a print target itself.
See the asset survey,
`wiki/001_Inbox/2026-07-18_curved_surface_assets.md`, for the full
measured-property writeup this guide's numbers come from.

## How it's computed

> ⚠ **Read this before step 4 below.** Step 4's centering description used to
> say the workpiece is centred on the **build-plate mesh's** own bounding box.
> That was 6.1's original design, and it turned out to be a bug, found and
> fixed 2026-09-03 (roadmap 7.4 follow-up, `settled.md` **S1.48**):
> `BambuLab_BuildPlate.obj` is a stand-in asset with its local origin at a
> corner, not a centre, and centering on it silently shifted the workpiece
> **+105.6mm** outward — enough to make the curved toolpath unreachable at the
> real calibrated User Frame (past the FR5's 922mm flange reach).
>
> **The workpiece is now centred on the User Frame origin, not on the plate
> mesh.** Step 4 below has been corrected to describe this directly — read it
> as current, not historical. **If you are pointing this feature at a
> different curved-print job (a different `study_config.py`), this is the one
> placement fact to get right: your workpiece's own XY bounding-box centre
> lands at `CURVED_MODEL_XY_OFFSET_MM` relative to the User Frame origin, and
> the default is `(0.0, 0.0)` — i.e. centred exactly ON the origin.** Do not
> assume it will land on, or relative to, wherever the build-plate mesh is
> drawn; that mesh has no bearing on placement at all since this fix. Full
> record, including why this was decided by measurement rather than by asking
> which convention the User Frame uses:
> `wiki/001_Inbox/2026-09-03_curved_placement_plate_centring_offset.md`.

`VisContent.load_curved_model()` and its helpers (`geometry_backend.py`):

1. `read_ply_polyline(filepath)` — a hand-rolled ASCII PLY reader. These
   files declare `element vertex` + `element edge` and **no**
   `element face`, so `trimesh.load(..., force='mesh')` (the loader
   `load_mesh()` uses for ordinary meshes) would yield a degenerate empty
   mesh instead of erroring. The header is trivially parseable; the
   vertex and edge blocks read straight into `np.loadtxt` after skipping
   it.
2. `reconstruct_polylines(verts, edges)` — the edge list is a **disjoint
   segment soup in file order**, not a walkable curve (confirmed by direct
   inspection — see the asset survey's `RX_0` counter-example). Reassembly:
   - Dedupe vertices by rounding to `CURVE_DEDUPE_DECIMALS` (3 decimal
     places / 0.001mm). Float export noise keeps true duplicate points
     apart past ~3dp; verified directly on `RX_0.ply` (108 raw vertices
     collapse to exactly 54 unique nodes at 3dp, matching the survey).
   - Build an adjacency map from the deduped edges, dropping any
     degenerate zero-length edge whose two endpoints round to the same
     node (a few files carry one; it carries no path information).
   - Every node has degree ≤ 2 across all 55 files (no branching), but not
     every connected piece has a degree-1 endpoint: **6 of the 70 pieces
     are closed loops** with no degree-1 node at all
     (`RX_0`/`RX_22`/`RX_27`/`TX_17`/`TX_2`/`TX_6`). The roadmap's original
     "chain-walk from a degree-1 endpoint" description missed this — an
     endpoint-only walk silently drops these 6 pieces (64 instead of 70).
     `reconstruct_polylines()` walks degree-1 endpoints for open pieces,
     then walks any leftover node as a closed loop (repeating its start
     point as the last row). Verified lossless — every non-self-loop edge
     consumed exactly once — across all 55 files.
3. The 3 surface OBJs load via the existing `load_mesh()` — ordinary
   triangle meshes, no special handling needed.
4. Placement (`load_curved_model()`), one 4×4 (`T_placement`) composed with
   `T_user_frame`:
   - The raw CAD-local points (curves + all 3 surfaces) are rotated
     `CURVED_MODEL_ROTATE_X_DEG` (90°) about local X **first**. The
     roadmap's "does the CAD +z axis point up?" open question turned out
     to be **no** — confirmed by loading both `+90°` and `-90°` and
     checking which one put the printable ridge-pattern surface face-up
     vs. face-down into the plate (`-90°` put it face-down, unprintable —
     wrong; `+90°` is what's shipped).
   - The **rotated** assembly's combined XY bounding-box center is
     translated to `CURVED_MODEL_XY_OFFSET_MM` (`study_config.py`, default
     `(0.0, 0.0)`) **relative to the User Frame origin** — by default,
     centred directly on that origin. (Was: centred on the build-plate
     mesh's own local XY bbox-center, derived from `plate.bounds` at load
     time. Changed 2026-09-03, `settled.md` **S1.48** — see the callout
     above. The plate mesh's local origin is a corner, not a centre, so
     that old placement put the workpiece **+105.6mm** further from the
     robot base than the User Frame origin itself, at the real calibrated
     frame — enough to push it past the arm's reach.)
   - Z is translated so the rotated assembly's lowest point lands at
     `PLATE_THICKNESS_MM` in plate-local space — the same
     resting-face/top-face compensation `load_build_plate()`/
     `build_toolpath_waypoints_world()` already apply, since the plate
     mesh's local origin sits at its top face while `position_mm` marks
     its resting/bottom face.
   - `T_curved = T_user_frame @ T_placement` is applied once via
     `transform_points(T, points)`, a shared helper pulled out of what
     used to be three separate inlined copies of the same
     homogeneous-multiply pattern (`load_build_plate()`,
     `_build_gcode_beads()`, `build_toolpath_waypoints_world()`) — this
     was the fourth call site that made extracting it worthwhile.
5. Registration: `_register_curve_layer()` combines one layer's
   reconstructed pieces (open + closed) into a **single**
   `ps.register_curve_network(...)` call per layer — not one per piece —
   so a future layer toggle (roadmap 6.6) can show/hide a whole pass at
   once. The 3 surfaces each get their own `ps.register_surface_mesh(...)`.
   Static workpiece geometry, same as the build plate and G-code preview:
   one-time placement, no Delta transform (`settled.md` S1.2/S1.3 — the
   Delta pipeline exists for the arm's joints, and this has none). Fixed
   structure names mean repeat-clicking "Load Curved Model" is safe —
   Polyscope replaces the prior structures rather than accumulating
   duplicates.
6. Retention: the placed assembly is kept in **world coordinates**
   (`curved_pieces_world`, `curved_surface_verts_world`,
   `curved_surface_faces`, `T_curved`) for roadmap 6.2's geodesic routing,
   which needs the per-piece curves and the two print surfaces in the frame
   the arm works in. `Surface_Bot` is rendered but not retained — it is not
   a collision body. (6.5's stated reason, that a per-waypoint tangent-plane
   check replaced the obstacle-mesh idea, is superseded: that check was deleted
   at 7.2 and Stage 7.4 built a real mesh-vs-mesh check — filter 8 — against
   each layer's *own print surface*. `Surface_Bot` is still not among them.
   `settled.md` S1.37 → **S1.46/S1.47**.) Reloading
   aborts any geodesic solved against the previous load, since every world
   vertex is re-derived here (`settled.md` S1.31).

## Current scope and limitations

This is 6.1 only — the geometry loads and sits in the right place, nothing
more:

- **No layer selector or Clear button yet** (roadmap 6.6) — clicking "Load
  Curved Model" always loads both RX and TX together, and there's no way
  to unload it from the GUI yet.
- **Print ordering now exists** (roadmap 6.3, done) — the 70 pieces are
  stitched into a per-layer print sequence with hover travel moves, built
  on the 6.2 geodesic logic. See
  [`CurvedModel_PrintOrder.md`](CurvedModel_PrintOrder.md), `settled.md` S1.35.
- **Per-waypoint surface-normal orientation now exists** (roadmap 6.4, done)
  — each feed point gets a TCP orientation with Z along the outward surface
  normal (`build_orientation_frames()`). 6.4 itself only *computed and
  visualised* the frames; wiring them through IK was 6.5. See
  [`CurvedModel_Orientation.md`](CurvedModel_Orientation.md), `settled.md` S1.36.
- **IK precompute now reuses Stage 5's machinery per layer** (roadmap 6.5,
  done) — `run_curved_toolpath_ik_precompute(layer, ...)` feeds the same
  chunked solver from `build_curved_toolpath_waypoints_world(layer)`, with
  per-waypoint `R_target` and tool-tip tangent-plane clearance instead of
  an obstacle mesh (the tested body became the TCP point at Stage 7.1, and
  7.2 removes the check). `geometry_backend.py`-only; no GUI button and no curved
  playback yet (roadmap 6.6). See
  [`CurvedModel_IKPrecompute.md`](CurvedModel_IKPrecompute.md), `settled.md` S1.37.
- **The 90° rotation is a fixed constant**, not derived from the build
  plate's live orientation. This is fine today since the plate defaults to
  zero rotation and nothing currently re-tilts it before loading the
  curved model — if that ever changes, `CURVED_MODEL_ROTATE_X_DEG` would
  need to become relative to the plate's actual pose rather than world X.
- ~~**What RX/TX semantically denote, and whether the `_verify_*` dumps in
  `assets/models/curved/` are asset contract or scratch, are still open
  questions.**~~ **Both answered** — RX and TX are the two electrode layers
  of the sensor, printed as separate passes with **RX first** (`settled.md`
  S1.30, S1.32); the `_verify_*` dumps are scratch. Nothing in 6.1 reads the
  `_verify_*` folders. See `Stage6_README.md`'s Open Questions section for
  the one question that remains (what fills the 2.00 mm under
  `Surface_RX_Offset`).

## How to tune it

Study-specific config, `examples/curved_surface_printing/study_config.py`
(settled.md S1.33 — swap this module to point the feature at a different
curved-print job):

| Constant | Effect |
|---|---|
| `CURVED_MODEL_DIR` | Path to the curved-model assets — `assets/models/curved/`. |
| `CURVED_LAYERS` | List of per-layer config dicts (`name`, `curve_files`, `curve_structure_name`, `curve_color`, `surface_file`, `surface_structure_name`, `surface_color`) — one entry per print layer, RX and TX by default. |
| `CURVED_MODEL_ROTATE_X_DEG` | The fixed placement rotation about local X, degrees — 90° puts the printable surface face-up. |
| `CURVED_MODEL_XY_OFFSET_MM` | **Where the workpiece's own XY bbox centre lands, relative to the User Frame origin.** `(0.0, 0.0)` — centred directly on the origin — by default. Not derived from the build-plate mesh (see the callout above `load_curved_model()`'s step 4). Change only after a fresh reachability measurement at the real frame matching S1.48's — don't guess a new offset for a different workpiece without measuring. |
| `CURVED_OBSTACLE_FILE` / `CURVED_OBSTACLE_STRUCTURE_NAME` / `CURVED_OBSTACLE_COLOR` | The optional non-print collision body (`Surface_Bot.obj`) and its display name/color. |

Generic engine tuning, still module-level constants in `geometry_backend.py`:

| Constant | Effect |
|---|---|
| `CURVE_DEDUPE_DECIMALS` | Rounding precision (decimal places) for vertex dedup during polyline reconstruction — 3 is the verified-correct value, see above. |
| `CURVE_RADIUS_MM` | Curve-network line thickness, world units (mm) — kept thin relative to `TRAJECTORY_RADIUS_MM` since 70 pieces would otherwise dominate the view. |

## Code anchors

- `geometry_backend.py`: `read_ply_polyline()`, `reconstruct_polylines()`,
  `transform_points()`, `_register_curve_layer()`, `load_curved_model()` —
  all generic, project-agnostic (settled.md S1.33).
- `examples/curved_surface_printing/study_config.py`: `CURVED_LAYERS`,
  `CURVED_MODEL_DIR`, `CURVED_MODEL_ROTATE_X_DEG`, `CURVED_MODEL_XY_OFFSET_MM`,
  `CURVED_OBSTACLE_*` — the RX/TX-specific config `geometry_backend.py` imports.
- `gui_panel.py`: "Load Curved Model" button, "I/O Operations" section —
  the only caller of `load_curved_model()`.
- `wiki/002_Architecture/settled.md` S1.29 — the placement decision and its
  same-day rotation amendment (including the +90°/-90° test that settled
  the rotation sign); **S1.48 — the XY-placement fix (centred on the User
  Frame origin, not the plate mesh) and why it was necessary**; S1.1 — why
  `read_ply_polyline`/`reconstruct_polylines`/`transform_points` are
  module-level functions, not `VisContent` methods (stateless, no instance
  data touched); S1.2/S1.3 — why static workpiece geometry skips the Delta
  transform; S1.33 — the generic-mechanism / study-config split.
- `wiki/001_Inbox/2026-09-03_curved_placement_plate_centring_offset.md` —
  full measurements behind S1.48 (closed).
- `tutorials/Stage6_README.md` — sub-stage 6.1 and the Open Questions
  section.
- `wiki/001_Inbox/2026-07-18_curved_surface_assets.md` — the asset survey
  this guide's measured numbers (PLY format, dedup precision, closed-loop
  pieces, bounding boxes, RX/TX-to-surface distances) come from.
