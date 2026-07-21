---
status: active
---

# Geodesic Routing Over the Print Surfaces

## What it is

The "Build Geodesics" button (I/O Operations panel, shown once a curved
model is loaded) computes shortest paths that stay **on** the print
surfaces, rather than cutting straight chords through the shoulder mockup.
This is roadmap `Stage6_README.md` sub-stage 6.2.

The output is two 70×70 geodesic cost matrices — one per print layer — plus
retained predecessor rows from which any individual path can be
reconstructed. Roadmap 6.3 consumes these to decide the print order of the
70 curve pieces and to emit the travel moves between them.

Nothing here drives the arm. 6.2 produces routes and distances only;
per-waypoint orientation is 6.4 and IK/playback is 6.5.

The geodesic engine is generic and project-agnostic (settled.md S1.33): it
builds one graph per *configured* layer, however many `CURVED_LAYERS`
describes. RX/TX are the shoulder-sensor study's two layers, wired up in
`examples/curved_surface_printing/study_config.py` — the numbers and
examples below are specific to that config, not the mechanism.

## Why two graphs, not one

RX and TX are two stacked electrode layers printed as **separate ordered
passes** that never interleave (`settled.md` S1.30). RX travels on
`Surface_RX_Offset`, TX on `Surface_TX_Base`.

**RX prints first, TX second** (`settled.md` S1.32) — the fabrication
sequence is RX → manual silicone fill → TX → manual silicone fill. 6.2
doesn't depend on the order (each layer routes on its own surface
independently); 6.3 and 6.4 do.

A geodesic between an RX endpoint and a TX endpoint is computed on neither
mesh meaningfully and is never needed, so the two networks are kept fully
separate: two graphs, two endpoint sets, two cost matrices. Merging them into
one 140×140 matrix would produce a large block of numbers with no physical
meaning.

Because the two passes are structurally identical, every geodesic structure
on `VisContent` is a **2-element list** indexed by `GEODESIC_LAYER_RX` /
`GEODESIC_LAYER_TX`, rather than a duplicated `_rx`/`_tx` attribute pair.
This diverges from the flat `precompute_*` naming next door; see
`settled.md` S1.31 for why.

## How it's computed

1. **Retained world geometry.** `load_curved_model()` now keeps the placed
   assembly (`curved_pieces_world`, `curved_surface_verts_world`,
   `curved_surface_faces`, `T_curved`) instead of discarding it. Everything
   is in **world coordinates**, already through `T_curved`, because
   everything downstream — 6.3's hover offsets, 6.4's normals, 6.5's IK —
   works in the frame the arm works in. `Surface_Bot` is rendered but not
   retained: it is a collision body for 6.5, not a print surface.

2. **`build_surface_graph(verts, faces)`** — nodes are mesh vertices, edges
   are triangle edges, weights are Euclidean edge lengths. Returns a
   CSR-style `(neighbor_start, neighbor_index, neighbor_weight)` triple.

3. **`nearest_vertex_index(verts, query_points)`** — snaps each of the 70
   curve endpoints to the nearest vertex **of its own surface**. Brute
   force; there is no `scipy` in this environment and none is needed at this
   size.

4. **`dijkstra_surface(...)`** — hand-rolled `heapq` Dijkstra with lazy
   deletion, per `AGENTS.md`'s from-scratch principle. Returns `dist` and a
   `prev` predecessor array.

5. **`geodesic_path_nodes(prev_row, target)`** — walks a stored predecessor
   row back to the source. This never re-runs Dijkstra, which is the entire
   reason the `(S, V)` predecessor rows are retained rather than just the
   cost matrix.

6. **Chunked driver** — `run_/pause_/cancel_/_abort_/step_geodesic_precompute()`,
   mirroring the Stage 5 IK precompute (`settled.md` S1.14/S1.15) including
   the pause-resumes-without-restarting contract. `step_` is pumped once per
   frame from `gui_panel.py`'s `render()`.

## Measured properties

Measured directly against the shipped assets, 2026-07-19. Recorded here so
later sessions don't re-derive it.

| Property | RX (`Surface_RX_Offset`) | TX (`Surface_TX_Base`) |
|---|---|---|
| Vertices / faces | 30,284 / 59,943 | 45,430 / 90,089 |
| Unique undirected edges | 90,226 | 135,518 |
| **Connected components** | **1** | **1** |
| Pieces / endpoints | 35 / 70 | 35 / 70 |
| Unique snapped vertices | 58 | 55 |
| Snap distance min/med/max | 0.041 / 0.377 / 0.684 mm | 0.024 / 0.342 / 0.580 mm |
| Longest geodesic in matrix | 317.1 mm | 319.4 mm |
| Unreachable pairs | 0 | 0 |
| Graph build | ~145 ms | ~270 ms |
| One `dijkstra_surface()` | ~50 ms | ~85 ms |
| Sources to solve | 58 | 55 |

Full run: 113 sources, **~8.4–9.1 s** wall — which reconciles with the
per-source figures (58 × 50 ms + 55 × 85 ms ≈ 7.6 s, plus ~0.6 s setup).

The per-source figure is the **shipped function**, which also allocates
`prev` and converts to numpy on return; the bare Dijkstra loop alone is
~81 ms on TX. An earlier version of this table quoted ~170 ms for TX, which
was a measured *worst frame*, not the solve time — see `settled.md` S1.31's
amendment.

**The surfaces are not fragmented.** The asset survey records both meshes as
"not watertight", which is true but does **not** imply disconnected — each
is exactly one connected component, and every vertex is reachable from every
other. Consequently there are zero unreachable pairs on the shipped assets.
This is worth stating explicitly because "not watertight" reads like a
warning that would otherwise invite building component-handling machinery
that isn't needed.

## Conventions that 6.3 depends on

### Endpoint indexing

Endpoint `2p` is `pieces[p][0]` and endpoint `2p+1` is `pieces[p][-1]`, for
`p` over `curved_pieces_world[layer]` in order. So `geodesic_cost[layer]` is
indexed by endpoint, not by piece, and choosing "which end of piece `p` to
enter from" (roadmap 6.3 step 3) is a choice between rows `2p` and `2p+1`.

### Zeros in the cost matrix are real, and there are many

`geodesic_cost[layer][i][j] == 0.0` for `i != j` whenever endpoints `i` and
`j` snapped to the **same mesh vertex**. All legitimate.

**Mind the counting convention.** The matrix is symmetric, so one pair of
endpoints produces **two** entries, `(i,j)` and `(j,i)`. Counts below are
*matrix entries* unless labelled "pairs":

| | Off-diagonal zeros | From different pieces | From one piece's two ends |
|---|---|---|---|
| RX | **24** entries | 16 entries = **8 pairs** | 8 entries = **4 pieces** |
| TX | **30** entries | 18 entries = **9 pairs** | 12 entries = **6 pieces** |

- **Coincident ends of one piece** — 4 pieces in RX, 6 in TX have both ends
  on the same vertex. Six of these (three per layer) are the exact closed
  loops that `reconstruct_polylines()` handles (`RX_0`/`RX_22`/`RX_27`,
  `TX_2`/`TX_6`/`TX_17` — see `CurvedModel_Loading.md`); the rest are
  near-loops whose two ends sit ~0.001 mm apart, one dedupe quantum short of
  merging, and which therefore snap together anyway.
- **Distinct endpoints of *different* pieces sharing a vertex** — 8 pairs in
  RX, 9 in TX. Separate curve pieces whose ends genuinely lie within
  sub-millimetre of each other on the surface.

Both are correct: those endpoints really are ~0.5 mm apart, and travel
between them really is free. But 6.3's greedy nearest-endpoint chain will
face **many tied zero-cost moves**, so its tie-breaking is a real design
decision rather than an incidental one — an unqualified `argmin` picks
arbitrarily among them. And for a closed loop, `cost[2p][2p+1] == 0.0` must
not be read as "free travel to somewhere else"; it is the same point.

### Some endpoints look like they're mid-curve

The same abutting pieces have a visual consequence. **16 RX / 18 TX
endpoints sit within 0.5 mm of another piece's line** — and because all 35
pieces of a layer render as a *single* combined curve network
(`_register_curve_layer`), the join between two abutting pieces is
invisible. They read as one continuous curve, so a geodesic terminating at a
genuine piece end appears to stop halfway along a curve.

This is the same phenomenon as the different-piece zeros above, counted a
different way (8 pairs × 2 = 16 endpoints, 9 × 2 = 18). If the true piece
boundaries ever need to be visible, registering the 70 endpoints as a point
cloud over the curves makes them explicit.

### A geodesic does not quite touch the curve it connects

Paths begin and end at the **snapped mesh vertex**, not at the curve
endpoint — median ~0.36 mm away, max 0.68 mm. So a travel move built
straight from 6.2's path nodes leaves a sub-millimetre gap at both ends
where it meets the piece it travels from and to.

**6.3 (or 6.5's waypoint builder) has to decide** whether to close that gap
by appending the true endpoints, or accept it as within positioning
tolerance. Easy to miss, since at render scale the gap is invisible.

### Geodesics can track the mesh rim

Over 60 random endpoint pairs, a mean **18%** of path nodes lie on the
surface's open boundary, and 11/60 pairs spend >20% of their nodes there.
This is geometrically correct — around a dome's rim genuinely can be the
shortest surface path — but a travel move that tracks the shell's open edge
may not be what you want physically. **Check this when 6.3 emits real travel
moves.** The default sample pairs are unaffected (2/254 nodes on the rim for
the most-curved pair, median 2.1 mm from the printed curves).

### There is no "top surface" to constrain routing to

The printed curves span z 2.7–159.0 against surfaces spanning z 0.9–159.0,
with matching bounding boxes on all three axes — the pattern wraps the whole
dome rather than sitting on a distinct top face. So "only route over the top"
isn't a well-defined constraint on these assets. Each print surface is a
single open sheet (623 boundary edges, 0 non-manifold, ~39,900 mm² — not a
double-sided skin), so a path on it cannot pass through the interior.

### Unreachable pairs

Represented as `np.inf` in the cost matrix and `None` from
`geodesic_path_nodes()`. Not `-1` (which would win every `argmin`) and not
`NaN` (which poisons comparisons unpredictably) — `inf` is the only sentinel
that behaves correctly under both the `<` comparisons and the `argmin`
selection that 6.3's ordering does, and it propagates into any tour total
that depends on an impossible move.

The first Dijkstra of each layer already covers the whole graph, so it
doubles as the reachability oracle — no separate flood fill is run. If it
finds unreachable endpoints, the status line reports it within ~100 ms of
the click rather than at the end of the run, and the run continues so the
reachable majority still solves. On the shipped assets this never fires.

## The sample geodesic aid — removed

6.2 shipped a `show_sample_geodesic()` verification aid that drew one geodesic
(green) plus a comparison chord (magenta) on an isolated host surface. **It was
removed 2026-07-21** (`settled.md` S1.31's "Removed" note): 6.3's print-order
overlay and `apply_live_layer_visibility()` supersede both its purposes — seeing
a geodesic on the surface, and viewing one layer at a time. Its backend methods
(`show_sample_geodesic`, `_pick_sample_pair`, `_isolate_geodesic_layer`,
`_restore_geodesic_isolation`) and sample-only `GEODESIC_*` colour/radius/
transparency constants are gone.

The measurement that motivated its isolation still matters and drives 6.3's
`apply_live_layer_visibility()`: the layer stack is
**`Surface_Bot` → `Surface_RX_Offset` → `Surface_TX_Base`** (median gaps from
`Surface_Bot`: RX **2.00 mm**, TX **4.02 mm**, TX outside RX at 100% of sampled
points), so `RX_Offset` is **sealed inside** `TX_Base` and RX geometry is
invisible with TX shown — which is why one layer must be isolated at a time.

## Two implementation choices that look wrong without the measurement

**Flat CSR rather than a list-of-lists adjacency.** `Surface_TX_Base` has
271,036 directed adjacency entries; a `list[list[tuple]]` allocates 45,430
lists plus 271,036 tuples — tens of MB through an interpreted build loop.
Flat CSR builds fully vectorised in ~0.27 s.

**Python lists rather than numpy arrays returned from
`build_surface_graph()`.** This looks backwards in a numpy codebase, and is
measured, not assumed: identical algorithm, identical CSR layout, only the
container type differs, and Dijkstra runs **174 ms with numpy element
indexing vs 83 ms with Python lists**. Numpy constructs a boxed scalar on
every element access; a list already holds native ints and floats. The
`.tolist()` conversion costs ~11 ms, once per graph. This is what makes
one-source-per-frame chunking viable.

## Invalidation

The retained world geometry and the stored paths go stale if the geometry
moves, so two hooks abort any in-flight or completed geodesic run:

- **`load_curved_model()`** — reloading re-derives every world vertex.
- **`load_build_plate()`** — if the plate pose differs from the one captured
  when the curved model was loaded. Note the subtlety: the geodesic *costs*
  are rigid-motion invariant (distances in mm don't change when the plate
  moves), but the retained world vertices and stored paths are stale, and
  6.3 consumes the paths, not just the numbers.

Neither hook clears `curved_model_loaded` — unloading the model is 6.6's
Clear-button territory.

## Current scope and limitations

- **Print ordering is done** (6.3, `settled.md` S1.35) — `build_print_order()`
  consumes these cost matrices and predecessor rows: greedy + 2-opt per layer,
  emitting hover travel moves.
- **Travel-move hover offset is done** (6.3) — the nozzle follows each geodesic
  offset outward by `CURVED_TRAVEL_HOVER_MM` along the from-scratch surface
  normals (`compute_vertex_normals`), so it clears the mockup.
- **No disk cache** — the run takes ~8.4–9.1 s and is recomputed per session.
  Roadmap 6.5 already schedules per-layer cache files; that is the natural
  place to add one, keyed on a surface hash rather than the plate pose,
  since costs are pose-invariant.
- **Minimal GUI only** — Build/Pause, Cancel, plus an RX/TX radio that isolates
  the live layer (6.3, `apply_live_layer_visibility()`) and a Build Print Order
  button (6.3). There is **no Clear** and no selector gating which layer is
  loaded/precomputed/played — that is 6.6.

## How to tune it

Generic engine tuning, `geometry_backend.py`:

| Constant | Effect |
|---|---|
| `GEODESIC_CHUNK_SOURCES` | Whole Dijkstra sources solved per frame. 1 gives ~6-12 fps while running and ~8.4 s total. |

Which layers exist and their display names come from `CURVED_LAYERS` in
`examples/curved_surface_printing/study_config.py` (settled.md S1.33) — the
engine reads `VisContent.curved_layer_names` (populated by
`load_curved_model()`) rather than a hardcoded RX/TX pair.

## Code anchors

- `geometry_backend.py`: `build_surface_graph()`, `nearest_vertex_index()`,
  `dijkstra_surface()`, `geodesic_path_nodes()` (module-level, per
  `settled.md` S1.1); `VisContent._layer_endpoints_world()`,
  `run_/pause_/cancel_/_abort_/step_geodesic_precompute()`; the
  `GEODESIC_CHUNK_SOURCES` constant; the retention block at the end of
  `load_curved_model()` and the invalidation hook in `load_build_plate()`.
  All generic — no RX/TX-specific code (settled.md S1.33). (The
  `show_sample_geodesic` sample aid and its isolation helpers were removed —
  S1.31's "Removed 2026-07-21" note; 6.3's overlay + `apply_live_layer_visibility`
  replace them.)
- `examples/curved_surface_printing/study_config.py`: `CURVED_LAYERS`,
  `CURVED_OBSTACLE_*` — what the engine above iterates over.
- `gui_panel.py`: the `step_geodesic_precompute()` pump line in `render()`,
  and the geodesic controls (data-driven off `content.curved_layer_names`)
  in the "I/O Operations" section.
- `wiki/002_Architecture/settled.md` S1.31 — the 6.2 decisions and why each
  was a real fork; S1.29/S1.30 — placement and the RX/TX layer semantics;
  S1.33 — the generic-mechanism / study-config split.
- `wiki/003_Guides/CurvedModel_Loading.md` — how the geometry this routes
  over gets loaded and placed.
- `tutorials/Stage6_README.md` — sub-stage 6.2 and what 6.3-6.6 do with it.
