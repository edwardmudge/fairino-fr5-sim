---
status: draft
---

# Exploring the Curved-Surface Assets (Stage 6 prep)

Non-authoritative exploration notes — see
[`TRUTH_LADDER.md`](../005_AgentMgmt/active/ctx_main/TRUTH_LADDER.md). If
anything here contradicts [`settled.md`](../002_Architecture/settled.md),
settled.md wins. This file records what we've inferred by inspecting new
asset files, not a design decision — check
[`ctx_system_current.md`](../005_AgentMgmt/active/ctx_main/ctx_system_current.md)
for actual build status.

**Revised 2026-07-18** after measuring the files directly rather than eyeballing
them in MeshLab. Two claims in the first version of this note were wrong and are
corrected below — see *Corrections*.

## Where this picks up

Two new asset directories landed in the repo: `assets/models/curved/` and
`assets/models/planar/`. `assets/models/planar/` turns out to just be the
existing Stage 5 gcode/precompute data, relocated from the old top-level
`assets/models/gcode/` during a "Stage 6-prep reorg" (see
[`Gcode_Toolpath.md`](../003_Guides/Gcode_Toolpath.md)) — already wired up
correctly in `geometry_backend.py` (`GCODE_DIR`, `GCODE_PRECOMPUTE_CACHE`)
and gitignored for its large binaries.

`assets/models/curved/` is new and unreferenced anywhere in the codebase.
Given that `tutorials/Stage6_README.md` existed as a deliberate stub and
Stage 5 is complete per `ctx_system_current.md`, this asset drop is the input
data for Stage 6 — printing on a curved (non-planar) surface, per the original
framing in
[`2026-07-09_2d3d_printing_roadmap.md`](2026-07-09_2d3d_printing_roadmap.md).

## Project context (from the user, 2026-07-18)

The goal is to 3D-print an **elastomeric material onto a surface that
represents a human shoulder**; the printed material acts as a **sensor**. So
the "workpiece" is a shoulder mockup and the print is a conformal sensor laid
onto it. This reframes Stage 6 from "print on an arbitrary curved surface" to a
specific, physically motivated target.

## What's in the folder

`assets/models/curved/` — 91 files, ~244 MB total, of which only **8.5 MB is
real input data**:

- **`RX_0.ply` … `RX_27.ply`** (28 files) and **`TX_0.ply` … `TX_26.ply`**
  (27 files) — ASCII PLY polylines (`element vertex` + `element edge`, no
  faces). Magnitudes are consistent with millimetres, matching the project's mm
  convention. **Both sets are toolpaths** (see *Corrections*).
- **`Surface_Bot.obj`**, **`Surface_RX_Offset.obj`**, **`Surface_TX_Base.obj`**
  — MeshLab-exported triangle meshes. `RX_Offset` and `TX_Base` are the two
  surfaces the curves lie on; `Bot` is neither, and reads as the underlying
  shoulder body.
- **`_verify_hardmin/`** (187 MB) and **`_verify_interpolation/`** (52 MB)
  — debug/verification dumps from whatever external pipeline generated the
  curves: point+normal `.txt` files and `.ply` files named `ridge_network`,
  `ridge_full_network`, `final_ridge`, `final_contours`, `final_combined`, and
  `T0`/`T1` variants. Reads as a ridge-network → interpolation/smoothing →
  final-contours pipeline, with "hardmin" checking a minimum-clearance
  constraint. Now gitignored (see *Storage*).

## Measured properties

Measured directly from the 55 PLY files and 3 OBJ meshes. Recorded here so
later sessions don't re-derive it.

| Property | Value |
|---|---|
| RX curves lie on | `Surface_RX_Offset.obj` — median point→surface 0.477 mm (vs 2.06 mm to the other two) |
| TX curves lie on | `Surface_TX_Base.obj` — median 0.368 mm (vs 2.07 / 4.04 mm) |
| Branching | None. Every node in all 55 files has degree ≤ 2; 0/55 files contain a junction |
| Piece count | 70 disjoint polylines — 35 components across 28 RX files, 35 across 27 TX files. ⚠ *Not all open: 6 are closed loops (`RX_0`/`RX_22`/`RX_27`, `TX_2`/`TX_6`/`TX_17`) — corrected during 6.1, see `settled.md` S1.29* |
| Gaps between pieces | endpoint → nearest other endpoint: median 8.87 mm, max 20.61 mm |
| Segment length | mean ≈ 1.5–2.0 mm, max ≈ 4.1 mm |
| Coordinate frame | CAD-local mm. bbox x∈[-225, -38] y∈[12, 169] z∈[-137, +24] — mostly below z=0 |
| Overall size | bbox diagonal ⚠ *287 mm as first recorded here doesn't reproduce; re-measured 2026-07-19 as **294.3 mm** for the curves alone and **296.4 mm** for curves + all 3 surfaces. `settled.md` S1.29's ~296 mm is the figure to use* |
| Mesh sizes | `Bot` / `RX_Offset` 30,284 v / 59,943 f; `TX_Base` 45,430 v / 90,089 f. None watertight — ⚠ *but **not disconnected**: 6.2 measured each print surface as exactly **one** connected component with every vertex reachable (`settled.md` S1.31). "Not watertight" here means an open shell with a rim (623 boundary edges on `RX_Offset`), not a fragmented one* |
| `Bot` vs `RX_Offset` | *Not* vertex-corresponding — face arrays differ, so per-vertex subtraction is meaningless |
| Real vs scratch size | 8.5 MB of input data; 239 MB of `_verify_*` dumps |

## Corrections

Two claims in the first version of this note were wrong.

**1. PLY edge order does not follow the curve.** The original note said
"walking the edge list in file order reconstructs the curve." It does not.
Edges are a *disjoint segment soup* with duplicated coincident endpoint
coordinates. Counter-example, `RX_0.ply`:

```
edge 0 = (0, 1)     v0 = (-84.808334, 75.438103, -126.235435)
edge 1 = (2, 3)     v3 = (-84.808334, 75.438103, -126.235435)   <- == v0
```

Edge 1 does not start where edge 0 ends; it *ends* where edge 0 *starts*.
Reconstructing a polyline needs coordinate dedupe → adjacency map → chain walk.
`RX_0` has 108 vertices collapsing to 54 unique nodes.

⚠ *This note originally said "chain walk **from a degree-1 endpoint**". That
is insufficient and was corrected during 6.1: 6 of the 70 pieces are closed
loops with no degree-1 node at all, and an endpoint-only walk silently drops
them (64 instead of 70). See `settled.md` S1.29 and
[`CurvedModel_Loading.md`](../003_Guides/CurvedModel_Loading.md).*

**2. RX is not "the toolpath" and TX is not "the surface".** Both are toolpath
curve sets — every one of the 55 PLY files is vertex+edge with zero faces. The
surfaces are the three `.obj` files. What actually distinguishes RX from TX is
*which surface it sits on* (see the table above): RX on `Surface_RX_Offset`, TX
on `Surface_TX_Base`.

## Working interpretation

RX and TX are two **stacked electrode layers**, not a path-and-its-surface
pair. RX sits on a surface offset outward from the TX base, i.e. two conductive
traces separated by a dielectric gap. Combined with the shoulder-sensor context
above, this reads as a **capacitive tactile sensor**: TX (transmit) and RX
(receive) electrode layers, printed conformally onto the shoulder mockup.

*The "capacitive" reading was inference from the geometry, not something
originally stated by the supervisor; confirmed 2026-07-19 (see Open
questions, item 3).* RX and TX are two separate ordered print passes rather
than one merged path, and `Surface_Bot.obj` is a collision body rather than
a print target.

**Corroborated 2026-07-20 by the fabrication sequence** (`settled.md` S1.32).
The supervisor's description of how the pad is made is: **RX layer → fill the
gaps with silicone → TX layer → fill the gaps with silicone**, the silicone
applied manually. A silicone dielectric deliberately filling the gap between
two electrode layers is exactly what the capacitive reading above predicted,
inferred at the time from geometry alone. It also settles the print order —
**RX first** (see item 3 below).

The earlier RF/ridge-waveguide guess in the first version of this note is
dropped — it was based on the "ridge network" filenames in `_verify_*`, which
more likely describe the curve-*generation* algorithm (ridge extraction on a
scalar field) than the end application.

## The shortest-path constraint

Recorded requirement from the supervisor: **use a shortest-path algorithm
(Dijkstra) in the implementation.** Stated goals — print the curves in the
easiest order, and don't drive the arm through the shoulder mockup.

Since no curve branches (degree ≤ 2 everywhere), Dijkstra is *not* needed to
untangle curve interiors — a plain chain walk does that. It applies to the
**travel moves between the 70 disjoint pieces**, where it serves both goals
with one algorithm used twice:

1. **Geodesic travel moves.** A straight-line hop from one curve's end to the
   next curve's start cuts *through* the shoulder shell. Dijkstra over the
   surface mesh's edge graph (weight = edge length) returns a path that hugs
   the surface instead. The gaps are real — median 8.87 mm, max 20.61 mm.
2. **Print ordering.** Those same geodesic distances form the cost matrix that
   decides which order the 70 pieces print in.

⚠ *Superseded in detail by 6.2 (`settled.md` S1.31): this is **two** 70×70
matrices, one per layer on its own surface — explicitly not one merged 140×140
matrix, since a geodesic between an RX and a TX endpoint is meaningless on
either mesh. And it needs 113 Dijkstra runs, not 140: distinct endpoints
frequently snap to the same mesh vertex (58 unique sources for RX, 55 for TX).*

Note there is no `scipy` in the `fairino-fr5-sim` environment (confirmed), so
this is either a new dependency or a `heapq` hand-roll. The from-scratch
principle in `AGENTS.md` favours `heapq`.

## Open questions for the supervisor

1. ~~**Coordinate frame and units.**~~ **Answered:** the curves load onto the
   build plate, above `z=0`. The source data is CAD-local mm (bbox reaches
   z = -137), so a placement transform is required — it is *not* already in
   robot world coordinates.
2. ~~**Are the `_verify_*` folders part of the asset contract, or scratch
   output?**~~ **Answered (2026-07-19):** scratch — safe to ignore. Confirms
   the gitignore assumption below; no reversal needed.
3. ~~**What do RX/TX denote, and which curve(s) get printed?**~~ **Answered
   (2026-07-19):** RX and TX are two layers of the printed sensor, offset
   from each other to represent the print's thickness — confirming the
   two-electrode-layer reading above. ~~TX is the underlying/base layer and
   prints first; RX (the offset layer) second.~~ See `settled.md` S1.30.

   ⚠ **The print-order half was reopened the same day** — the *two-layer*
   reading held, but the order was put in doubt. Measuring the three surfaces
   against each other during Stage 6.2 gives median nearest-surface gaps
   `Surface_Bot` → `Surface_RX_Offset` **2.00 mm** → `Surface_TX_Base`
   **4.02 mm**, with TX outside RX at 100% of 3,000 sampled points, and each
   layer's curves following its own surface. The physical stack is therefore
   **BOT → RX → TX**: `RX_Offset` is the layer against the shoulder body, so
   at face value **RX** is laid down first, not TX. The filenames said TX was
   the base; the geometry said otherwise.

   ✅ **Resolved 2026-07-20 — RX prints first** (`settled.md` S1.32). The
   supervisor gave the fabrication sequence: RX → manual silicone fill → TX →
   manual silicone fill. **The measurement was right and the filenames were
   the misleading signal** — `Surface_TX_Base` reads as "the base layer" but
   isn't one. Worth carrying forward: when a name and a measurement disagree
   here, the measurement won.
4. ~~**Storage/tracking plan.**~~ **Resolved** by the gitignore change below.
5. ⚠ **NEW (2026-07-20) — what fills the 2.00 mm between `Surface_Bot` and
   `Surface_RX_Offset`?** RX printing first means it isn't touching the
   mockup. **Working assumption:** a silicone base layer is applied to the
   shoulder before the RX pass, making `Surface_RX_Offset` that base's outer
   surface. Not yet confirmed. Blocks nothing in 6.3–6.6 — it changes how the
   gap is interpreted, not any transform, route or clearance.

## Storage

`.gitignore` now excludes the two verification dumps:

```gitignore
assets/models/curved/_verify_hardmin/
assets/models/curved/_verify_interpolation/
```

That drops what a `git add assets/models/curved/` would stage from ~244 MB to
~8.5 MB (55 PLY + 3 OBJ), which is comfortably committable. The two directories
are listed explicitly rather than as a `_verify_*` glob so that a
differently-named future dump shows up in `git status` instead of being
silently swallowed.

Confirmed correct by the answer to open question 2 above (2026-07-19) — was
trivially reversible either way, since gitignoring deletes nothing.

## Next step

`tutorials/Stage6_README.md` has been fleshed out from its stub following the
`tutorials/Stage5_README.md` template, with sub-stages 6.1–6.6 all marked
**planned**. It carries its own Open Questions section mirroring 2 and 3 above.
Nothing has been written to `settled.md` yet — in particular 6.4 (per-waypoint
orientation) contradicts S1.12 and needs a real decision first.

## Implementation notes for 6.3–6.6 (moved from Stage6_README, 2026-07-20)

`Stage6_README.md` was trimmed to match `Stage5_README.md`'s terser, human-
readable style. The forward-looking implementation detail for the still-
**planned** stages (6.3–6.6) is kept here instead of being deleted, since it
doesn't have a `settled.md` entry yet.

### 6.3 — Order the 70 Pieces

- **Tied zero-cost moves are real, not a bug to avoid**: 16 (RX) / 18 (TX)
  matrix entries — 8 and 9 endpoint pairs — sit on *different* pieces but
  have geodesic cost exactly 0.0 because they snap to the same mesh vertex.
  A bare `argmin` picks among them arbitrarily; tie-breaking is a genuine
  design decision (`settled.md` S1.31).
- **Snap gap**: a geodesic starts/ends at the snapped mesh vertex, median
  ~0.36 mm / max 0.68 mm from the true curve endpoint. Decide explicitly
  whether to append the true endpoints to close the gap or accept it as
  within tolerance.
- **Rim-hugging travel moves**: 6.2 measured a mean 18% of path nodes on the
  surface's open boundary across random endpoint pairs (11/60 pairs above
  20%). Geometrically correct (shortest path around a dome's rim can
  genuinely go along the rim) but may not be physically desirable — worth a
  sanity check once travel moves are rendered.
- **Hover offset mechanics**: emit each travel move as the 6.2 geodesic
  polyline offset outward along the local surface normal by a new
  `CURVED_TRAVEL_HOVER_MM` constant (~3–5 mm, tune empirically). The normal
  lookup (nearest-vertex `vertex_normals` via trimesh, no `scipy`) is pulled
  forward from 6.4 into this stage since the hover offset needs it first;
  6.4 then reuses the same lookup rather than re-implementing it. Outward =
  away from `Surface_Bot`.

### 6.4 — Per-Waypoint Orientation from Surface Normals

This is an **architecture decision, not just code** — don't write it to
`settled.md` until decided; it supersedes S1.12 and deserves its own entry
explaining why the flat-plate assumption was correct at the time and what
replaces it.

- `settled.md` S1.12 snapshots one constant `R_target` for the whole path —
  correct for a flat, non-tilting build plate, wrong for a curved one. It's
  baked into `build_toolpath_waypoints_world()` (`geometry_backend.py:1013`,
  returns a single `R_target` at `:1042`) and `precompute_R_target` (`:146`,
  assigned at `:1393`), consumed as one shared matrix per waypoint at
  `:1452`.
- A curved surface needs a per-waypoint rotation from the local surface
  normal, plus a decision about the remaining degree of freedom (rotation
  about the nozzle axis) — most likely aligned to the path tangent.
- Watch for normal flipping between adjacent waypoints. The resulting IK
  branch discontinuity is what the reference-pose ranking in
  `solve_ik_tcp_matrix` (`:1150`) exists to smooth, but it needs a sensible
  sequence fed into it first.
- **Outward = away from `Surface_Bot`** — fixed by the measured stack
  geometry (BOT → RX 2.00 mm → TX 4.02 mm), independent of pass order.
  Getting the sign wrong drives the nozzle *into* the mockup — verify
  `mesh.vertex_normals`' orientation against the away-from-Bot direction
  before trusting it; it's a property of the CAD export, not the physical
  stack.

### 6.5 — IK Precompute & Playback Reuse

- **Open the precompute seam**: `run_toolpath_ik_precompute(joint_limits, ...)`
  (`:1349`) hardcodes its own source data at `:1367-1400` (cache →
  `GCODE_DIR` → `parse_gcode` → `build_toolpath_waypoints_world`) with no
  parameter for injecting a waypoint list. Add a `waypoints=None` keyword, or
  split out a `_begin_precompute(waypoints, R_target, ...)` helper.
  Pre-populating `self.precompute_waypoints` from outside works but is an
  undocumented backdoor — don't ship that.
- Once 6.4 lands, `precompute_R_target` becomes an array (per-waypoint), not
  a single matrix, and that has to flow through the whole precompute path.
- **Bead rendering needs new constants**: the PLY curves carry no extrusion
  `E` and layer-from-Z is meaningless on a conformal path, so add assumed
  `CURVED_BEAD_WIDTH_MM` / `CURVED_BEAD_HEIGHT_MM` (same spirit as the
  existing assumed `FILAMENT_DIAMETER_MM`) and feed the existing bead-box
  template with a constant cross-section along feed segments.
- **Cache goes per-layer**: `GCODE_PRECOMPUTE_CACHE` is one fixed filename —
  can't serve TX, RX, and the planar benchy without thrashing. Use per-source
  files (`curved_rx.precompute.npz`, `curved_tx.precompute.npz`).
  `_toolpath_cache_meta()` (`:1276`) keys on G-code hash + plate pose — add a
  curve/surface hash field and bump `PRECOMPUTE_CACHE_VERSION` (`:118`) so
  old caches are rejected rather than silently misapplied.
- **Collision obstacle is per-pass**, because the two passes see a different
  physical world:

  | Pass | Obstacle | Why |
  | --- | --- | --- |
  | RX (first) | `Surface_Bot` | nothing is printed yet |
  | TX (second) | `Surface_RX_Offset` | stands in for the cured RX traces + silicone fill now present |

  Using `Surface_Bot` for both would let the TX pass drive the arm straight
  through 2 mm of already-printed sensor without complaint.
  `_branch_clears_ground()` (`geometry_backend.py:1815`) currently only
  tests literal world `z=0` (`settled.md` S1.13) — a shoulder mockup on the
  plate needs clearance against the obstacle mesh, not a ground plane.
- **Design caveat**: "no part of the moving geometry may contact the
  obstacle mesh" rejects *every valid feed waypoint*, since the nozzle tip
  touching the surface is exactly what printing is. The check must apply
  strict clearance to the arm links/nozzle body while exempting (or giving
  tolerance to) the tip region near the TCP.
- `Surface_Bot` is rendered but **not retained** — `load_curved_model()`
  keeps only the two print surfaces in world space
  (`geometry_backend.py:652-657`, per S1.31). This stage needs `Surface_Bot`
  retained too before it can collide against it.

### 6.6 — GUI Wiring

- New numbered `psim.TreeNode` section in `gui_panel.py`'s `render()`,
  following the existing structure. New per-frame pump line at the top of
  `render()` alongside the existing three (`gui_panel.py:60-62`).
- Layer selector: a radio pair (RX first, TX second) — 6.2 already added a
  minimal RX/TX `psim.RadioButton` pair for the sample geodesic
  (`gui_panel.py:131-135`); reuse that idiom. Check
  `docs/Polyscope_Quickstart.md` for the `(changed, value)` return signature
  before using it (`AGENTS.md` rule).
- Selecting a layer re-applies the visibility set from the "One live layer at
  a time" table in `Stage6_README.md`. Build this by **generalising**
  `_isolate_geodesic_layer()` / `_restore_geodesic_isolation()`
  (`geometry_backend.py:959` / `:996`), which already implement
  snapshot-and-restore of per-structure visibility for exactly this reason.
  Do not add a second visibility mechanism beside them — two would fight
  over the same Polyscope structures, and S1.31's amendment records a real
  bug from restore clobbering state it never captured.
- Load/Clear pair following the conditional-clear pattern at
  `gui_panel.py:88-94` — Clear gated on a **backend-owned** boolean flag, not
  UI state.
- Clear-sample button for the geodesic view: removes the sample/chord curves
  and calls `_restore_geodesic_isolation()` **without**
  `_abort_geodesic_precompute()` — as of 6.2 the only way back to normal
  visibility is Cancel Geodesics, which also discards the ~8.4 s cost
  matrices.
- Gate controls during playback with `psim.BeginDisabled(...)` (`settled.md`
  S1.27) — including the layer selector, so the live pass can't be switched
  mid-playback.
- Verify the *transitions*, not just the states: a user-set visibility or
  transparency should survive a load → select → clear round trip — S1.31's
  amendment records a real bug here.
