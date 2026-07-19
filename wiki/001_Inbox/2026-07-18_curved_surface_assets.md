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
   two-electrode-layer reading above. TX is the underlying/base layer and
   prints first; RX (the offset layer) second. See `settled.md` S1.30.

   ⚠ **Partially reopened, same day** — the *two-layer* reading holds, but
   the *print order* is now in doubt. Measuring the three surfaces against
   each other during Stage 6.2 gives median nearest-surface gaps
   `Surface_Bot` → `Surface_RX_Offset` **2.00 mm** → `Surface_TX_Base`
   **4.02 mm**, with TX outside RX at 100% of 3,000 sampled points, and each
   layer's curves following its own surface. The physical stack is therefore
   **BOT → RX → TX**: `RX_Offset` is the layer against the shoulder body, so
   at face value **RX** is laid down first, not TX. The filenames say TX is
   the base; the geometry says otherwise. Open for the supervisor — it
   decides roadmap 6.3's pass order and the sign of 6.4's hover-offset
   normal. See S1.30's caveat.
4. ~~**Storage/tracking plan.**~~ **Resolved** by the gitignore change below.

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
