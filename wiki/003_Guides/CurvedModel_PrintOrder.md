---
status: active
---

# Print Ordering and Travel Moves Over the Print Surfaces

## What it is

The "Build Print Order" button (I/O Operations panel, shown once a layer's
geodesics are built) decides the sequence in which a layer's 35 curve pieces
print, and generates the non-printing hops between them. This is roadmap
`Stage6_README.md` sub-stage 6.3.

The output, per layer: an ordered list of `(piece, entry_end)`, a hover-offset
travel polyline between each consecutive pair, a print-order gradient overlay,
and — pulled forward from 6.4 because the travel hover needs it first — the
from-scratch outward surface normals every later stage reuses.

Nothing here drives the arm; 6.3 produces an order and travel geometry only.
Per-waypoint orientation is 6.4 and IK/playback reuse is 6.5 (both done,
`settled.md` S1.36/S1.37).

Generic and project-agnostic (`settled.md` S1.33) — `build_print_order()`
iterates over however many layers `CURVED_LAYERS` describes. RX/TX are the
shoulder-sensor study's two layers; the numbers below are specific to that
config.

## Why this is a TSP variant, not a plain sort

Every piece is printed exactly once (a feed move along its curve, cost fixed
regardless of order), so **order only changes the sum of the geodesic hops
between pieces** — that sum is the entire optimisation target. Each piece also
has two ends it could be entered from, so ordering is really choosing both a
*sequence* and, per piece, an *entry end*: endpoint `2p` and `2p+1` are piece
`p`'s two ends (`_layer_endpoints_world`), and a print order is a list of
`(piece, entry_end)`.

**RX prints first, TX second, ordered independently** (`settled.md` S1.32) —
no travel move stitches the last RX piece to the first TX piece, since the
manual silicone fill happens in that gap.

## How it's computed

`build_print_order()` and its helpers (`geometry_backend.py`):

1. **Greedy nearest-endpoint seed.** `greedy_piece_order()` (module-level pure
   function) builds an initial tour by always hopping to the nearest
   unvisited piece's nearest end, off the 6.2 cost matrix.
2. **2-opt improvement.** `two_opt()` repeatedly reverses a contiguous block of
   the tour if that lowers total travel (`travel_cost()`). A good order, not a
   proven-optimal one. Reversing a block also **flips each piece's entry/exit
   end** — because geodesic cost is symmetric, a reversed *internal* hop keeps
   the same two physical endpoints and is unchanged, so only the two cut edges
   at the block's boundary actually move. With only 35 pieces the tour is
   re-summed in full rather than tracked incrementally — trivial at this size,
   and immune to delta-sign bugs. A block length of 1 is a single-piece
   end-swap, so even one piece's entry end alone can improve.
3. **Zero-cost ties break to the lowest endpoint index** (a stable `argmin`),
   so the order is reproducible run to run. A zero-cost hop to a genuinely
   *different* piece is real free travel and is taken; a piece leaves the
   candidate set the moment it's entered, so a closed loop's
   `cost[2p, 2p+1] == 0` (the same point, not a destination) is never a
   candidate — see `CurvedModel_Geodesics.md`'s zero-cost-matrix-entries note.
4. **Surface normals, computed from scratch.** `compute_vertex_normals(verts,
   faces)` accumulates area-weighted face normals in numpy — trimesh's
   `vertex_normals` needs `scipy.sparse` and silently degrades without it
   (measured: only ~84% outward on these assets). `_orient_normals_outward()`
   fixes the sign as one global majority vote against the direction away from
   the nearest `Surface_Bot` vertex, baked into the retained
   `curved_surface_vnormals_world`. This is the normal lookup roadmap 6.4 was
   to introduce, pulled forward since 6.3's hover offset needs it first; 6.4
   reuses the exact same array. A wrong sign would drive the nozzle into the
   mockup.
5. **Travel moves hover.** Each travel polyline is the 6.2 geodesic, offset
   outward along the local surface normal by `CURVED_TRAVEL_HOVER_MM` (4.0mm,
   assumed) so the nozzle never scrapes the mockup or wet traces — rendered as
   one `Curved Travel <name>` network per layer in `CURVED_TRAVEL_COLOR`
   (solid warm red), deliberately off the print-order gradient ramp so
   "printing" vs "moving" read apart at a glance.
6. **Snap gap closed by appending true endpoints.** A geodesic starts/ends at
   the *snapped mesh vertex*, ~0.36mm from the true curve endpoint (S1.31).
   Each travel polyline is bookended with the true exit/entry endpoints
   (themselves lifted to hover height along their own snap-vertex normal), so
   the route actually connects end to end instead of leaving a sub-millimetre
   gap.
7. **Print-order gradient overlay.** A second curve network, `Curved Order
   Feed <name>`, draws the printed pieces *in print order* (each piece
   reversed when entered at `2p+1`), coloured by a sequence gradient
   (`_sequence_colors()` over the `CURVED_ORDER_CMAP` purple→teal→yellow ramp,
   applied per-edge via `add_color_quantity(defined_on='edges')`) so the order
   itself is legible, not just coverage. Supersedes the flat base curve when
   shown.
8. **Strict live-layer isolation.** `apply_live_layer_visibility(layer)` shows
   only the selected layer's surface/overlay/travel and hides every other
   layer — needed because RX is sealed inside the TX shell. This is currently
   the *sole* visibility mechanism (the sample-geodesic isolation it once
   composed with was removed, `settled.md` S1.31's "Removed" note); the
   eventual 6.6 rule is the S1.32 physical stack (TX shows RX beneath).
9. **Synchronous, not chunked.** Unlike 6.2's ~8.4s Dijkstra precompute, this
   stage only walks stored predecessor rows — no re-solve — so it finishes
   inside one frame off a direct button click, no per-frame stepper.

## Measured properties

Measured directly against the shipped assets, 2026-07-21.

| Property | RX | TX |
|---|---|---|
| Optimised travel total | 690 mm | 607 mm |
| Naive file-order travel total | 5157 mm | 4848 mm |
| Minimum travel-node clearance vs 4.0mm hover | 3.97 mm | 3.96 mm |
| Print-order overlay edge count | 2492 | 1965 |
| Max rim-hugging fraction on ordered travel | up to ~46% | up to ~46% |

**2-opt sanity, 20 random layers:** travel cost never increases across the
pass, and every resulting order is a valid piece permutation with valid entry
ends; a genuine zero-cost different-piece hop is taken when offered.

**Retained normals:** unit-length and 100% outward from `Surface_Bot` on both
layers (vs. ~84% via trimesh's degraded path).

## Conventions and gotchas

**Rim-hugging is reported, not gated.** 6.2 already warned a geodesic can
legitimately track a surface's open boundary (mean ~18% of nodes over random
pairs). On the *ordered* travel moves the max per-layer figure is
substantially higher (~46%), because the greedy chain favours short hops
between near-rim endpoints. `build_print_order()` reports the max rim-node
fraction in its status line; it's geometrically correct and left as a
physical judgement for a human (higher hover clearance, or a rim penalty in
the cost), not a hard reject.

**Zero-cost ties are common and real**, not a bug — see
`CurvedModel_Geodesics.md`'s write-up of the 8 (RX) / 9 (TX) endpoint pairs
that snap to the same mesh vertex. A bare `argmin` would pick among them
arbitrarily; breaking to the lowest endpoint index is what makes the order
reproducible.

## Current scope and limitations

- **No per-pass obstacle-mesh collision check.** 6.3 only offsets travel moves
  by a fixed hover clearance along the surface normal — it does not check the
  arm or nozzle against any obstacle mesh. That check (ultimately built
  differently than first planned) belongs to 6.5, see
  [`CurvedModel_IKPrecompute.md`](CurvedModel_IKPrecompute.md).
- **2-opt full re-sum, not incremental.** Fine at N=35 pieces; would need the
  two-cut-edge delta the symmetry argument already justifies if piece counts
  grew enough to matter.
- **Minimal GUI only** — a Build Print Order button and the RX/TX radio that
  drives `apply_live_layer_visibility()`. No Clear button (roadmap 6.6).

## How to tune it

Generic engine tuning, `geometry_backend.py`:

| Constant | Effect |
|---|---|
| `CURVED_TRAVEL_HOVER_MM` | Outward offset (mm) applied to every travel polyline along the local surface normal. 4.0 assumed; tune empirically for a different nozzle/material. Lives in `examples/curved_surface_printing/study_config.py` (nozzle/material-dependent — `settled.md` S1.41), not `geometry_backend.py`. |
| `CURVED_TRAVEL_COLOR` | Flat colour for travel moves — deliberately off the `CURVED_ORDER_CMAP` gradient ramp. |
| `CURVED_ORDER_CMAP` | Sequence-gradient colourmap for the print-order overlay (purple→teal→yellow). |
| `CURVED_ORDER_FEED_RADIUS_MM` | Line radius (mm) of the print-order overlay, drawn over the base curve. |

## Code anchors

- `geometry_backend.py`: `greedy_piece_order()`, `two_opt()`, `travel_cost()`,
  `compute_vertex_normals()`, `_orient_normals_outward()` (module-level, per
  `settled.md` S1.1); `VisContent.build_print_order()`,
  `apply_live_layer_visibility()`, `_sequence_colors()`,
  `_surface_boundary_vertices()` (the rim-node detector);
  `CURVED_TRAVEL_COLOR`/`CURVED_ORDER_CMAP`/`CURVED_ORDER_FEED_RADIUS_MM`
  presentation constants. All generic — no RX/TX-specific code
  (`settled.md` S1.33).
- `examples/curved_surface_printing/study_config.py`: `CURVED_TRAVEL_HOVER_MM`
  (moved here by `settled.md` S1.41 — it is nozzle/material-dependent).
- `gui_panel.py`: "Build Print Order" button, RX/TX layer selector driven off
  `content.curved_layer_names`, "I/O Operations" section.
- `wiki/002_Architecture/settled.md` **S1.35** — the full decision record and
  its same-day print-order-visualisation amendment; S1.31/S1.33 for the
  geodesics/config-split it builds on; S1.32 for the RX-first fabrication
  sequence.
- [`CurvedModel_Geodesics.md`](CurvedModel_Geodesics.md) — the cost matrices
  and predecessor rows this stage consumes, and the zero-cost/rim-hugging
  phenomena this guide reuses.
- `tutorials/Stage6_README.md` — sub-stage 6.3 and what 6.4/6.5 do next.
