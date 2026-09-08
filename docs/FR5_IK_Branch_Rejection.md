# FR5 IK Branch Rejection Criteria

What the toolpath planner rejects, in the order it rejects it, and why. Adapted for this project from a working implementation of the same task in another project; the deviations from it are recorded at the end. Decision of record: `wiki/002_Architecture/settled.md` **S1.46** (the design) and **S1.47** (as built, with the measurements).

## Overview

An FR5 waypoint does not have one solution. It has a *set*, and the planner's job is to pick one per waypoint such that the whole path is safe and continuous. Rejection happens in two phases:

1. **Candidate filter** — per candidate, before the graph exists. Hard reject.
2. **Edge filter** — between adjacent candidates, inside the graph search. Hard reject plus soft penalty.

Only candidates surviving phase 1 become graph nodes. Only edges surviving phase 2 are traversable.

Code path, all in `geometry_backend.py`:

```
_begin_toolpath_precompute      builds the per-run filter context once
  step_toolpath_ik_precompute   one chunk of waypoints per frame
    _waypoint_candidates        generate + filter one waypoint  (phase 1)
      orientation_candidates    the commanded frames to try
      solve_ik_tcp_matrix       IK branches per frame           (filter 1)
      _candidate_admissible     filters 2-9
    _relax_candidate_layer      edge costs into this layer      (phase 2)
  _finish_candidate_search      backtrack the DAG into a joint path
```

Nothing in the filter stack is rebuilt per candidate. `_filter_context()` computes the surface grid, the plate frame, the link pair list and the OBB proxies once per run and passes them down as `ctx`.

## Candidate Generation

`orientation_candidates(nominal_R)` returns the commanded TCP orientations tried at one waypoint. Two DOF are swept, for two different reasons:

| DOF | Range | Why |
|---|---|---|
| Tool axis | cone of half-angle `ORIENT_SEARCH_TILT_MAX_DEG` (20°) about the nominal surface normal, sampled as the normal plus `ORIENT_SEARCH_TILT_RING_AZIMUTHS` (8) directions at the full cap | The supervisor's relaxation: perpendicular to the surface *within* 20°, not exactly. This is the only part of the design that loosens anything |
| Roll about that axis | all `ORIENT_SEARCH_ROLL_SLOTS` (60), 6° apart, wrapping | Always free — the nozzle is rotationally symmetric. Sweeping it costs nothing physically and buys both reach (the flange→TCP offset is lateral, so rolling relocates the wrist centre) and continuity |

`ORIENT_SEARCH_FRAMES` = `(1 + 8) × 60` = **540** commanded frames, each yielding up to 8 IK branches, so **≤4,320 raw candidates per waypoint**.

Only the cap of the cone is sampled, not intermediate rings: the cap is where the reach leverage is, and intermediate rings multiply IK cost for very little extra coverage.

Two ordering properties are load-bearing, not incidental:

- **Tilt-major.** A candidate's index decomposes as `(frame // ROLL_SLOTS, frame % ROLL_SLOTS)` = `(tilt_idx, roll_idx)`. The edge cost reads `roll_idx` to charge for roll jumps.
- **Index 0 is tilt 0, roll 0** — the nominal axis. The pre-search commanded direction is always in the set.

Candidates are deduped on joints rounded to 0.01°. Distinct `(tilt, roll)` frames routinely map to the same arm pose — most obviously wherever the tool axis is near the wrist axis — and every duplicate would otherwise cost a full row and column in this layer's edge block. A pose is marked seen when **evaluated**, not when it passes: admissibility is a pure function of the pose, so a duplicate that failed will fail identically.

## Filter Order and Cost

`_candidate_admissible(joints, ctx)` returns `None` if the candidate is admissible, otherwise the short name of the **first** filter it failed. Order is deliberate and runs cheapest-first:

| Stage | Filters | Cost |
|---|---|---|
| Pure arithmetic | 2, 3 | two comparisons |
| One FK | — | `compute_fk` + `_moving_geometry_deltas_from_fk`, shared by everything below |
| FK-derived points | 4, 5 | vector arithmetic on three frame origins |
| Sampled link points | 6, 7, 8 | one `_link_sample_points` call shared by all three |
| Oriented boxes | 9 | bounding-sphere pre-test, then SAT on survivors |

Two measured optimisations are why this is affordable at ~4,300 candidates per waypoint:

- Sharing **one** FK across filters 4-9 rather than three: 3.677 → 0.742 ms per candidate.
- The bounding-sphere pre-test before filter 9's SAT: 0.533 → 0.05 ms per candidate.

Short names used in the per-waypoint rejection tally: `limits/reach`, `J5`, `J4`, `upper-branch`, `elbow-plate`, `under-plate`, `plate-slab`, `surface`, `self-collision`.

## Filter 1 — Joint Limits

**This filter is not in `_candidate_admissible`.** It is enforced upstream, inside `solve_ik_tcp_matrix`, which discards any solution outside `PHYSICAL_JOINT_LIMITS` (trying a ±360° wrap first, via `wrap_into_limits`) before a candidate ever reaches the filter stack. If you go looking for it in the filter function, it is not there.

| Detail | Value |
|---|---|
| Check | every joint within `PHYSICAL_JOINT_LIMITS` |
| Limits | J1 ±174, J2 −264/+84, J3 ±159, J4 −264/+84, J5 ±174, J6 ±174 (degrees) |
| Tallied as | `limits/reach` — counted when a frame yields *no* solutions at all |
| Why | Out-of-range joints are physically impossible for the FR5 |

`limits/reach` deliberately conflates "no geometric solution" with "no solution inside limits". The distinction that matters diagnostically is between this and everything else: a tally dominated by `limits/reach` means the arm genuinely cannot reach the point, while one dominated by a single filter name means that filter is mistuned.

Note `gui_panel.JOINT_LIMITS` is a **different, narrower** set used only for the sliders. Every solver call passes `PHYSICAL_JOINT_LIMITS`. See `FR5_Joint_Limits.md`.

## Filter 2 — J5 Minimum

| Detail | Value |
|---|---|
| Check | `joints[4] >= FILTER_J5_MIN_DEG` |
| Constant | `FILTER_J5_MIN_DEG = 2.0` (`geometry_backend.py`) |
| Default | always on, no flag |
| Cost | one comparison |
| Why | Negative J5 flips the wrist, giving an upside-down tool approach |

Set at 2.0 rather than the reference's 0.0 so it also subsumes the exchange spec's row 7, which WARNs on `|J5| < 2°` as a singular configuration — an exported job then cannot carry that warning. Measured cost of choosing 2.0 over 0.0: **0 candidates on RX, 2 of 8,834 on TX**.

## Filter 3 — J4 Minimum

| Detail | Value |
|---|---|
| Check | `joints[3] >= FILTER_J4_MIN_DEG` |
| Constants | `FILTER_J4_MIN_DEG = -60.0`, `FILTER_J4_ENABLED = False` (`geometry_backend.py`) |
| Default | **opt-in, off** — a module constant, not a GUI toggle |
| Cost | one comparison |
| Why | Restricts elbow range to avoid extreme wrist poses |

## Filter 4 — Upper Branch Configuration

| Detail | Value |
|---|---|
| Check | the elbow must stand above the shoulder→wrist chord by `FILTER_UPPER_BRANCH_TOL_MM` in world Z |
| Constant | `FILTER_UPPER_BRANCH_TOL_MM = 2.0` (`geometry_backend.py`) |
| Default | always on, no flag |
| Points used | origins of `T_0_1` (shoulder), `T_0_2` (elbow), `T_0_3` (wrist) |
| Cost | first use of the shared FK |
| Why | Rejects lower-elbow poses and, because a straight arm puts the elbow *on* the chord, near-singular ones too — those have unpredictable velocity and can flip suddenly |

```python
chord = wrist - shoulder
chord_dir = chord / np.linalg.norm(chord)
offset = (elbow - shoulder) - np.dot(elbow - shoulder, chord_dir) * chord_dir
if offset[2] < FILTER_UPPER_BRANCH_TOL_MM:   # world Z of the perpendicular offset
    reject
```

A degenerate chord (shoulder and wrist coincident, norm < 1e-9) is also rejected here.

## Filter 5 — Elbow Above the Plate Plane

| Detail | Value |
|---|---|
| Check | signed distance from the elbow to the build-plate top face >= `-FILTER_ELBOW_PLATE_TOL_MM` |
| Constants | `FILTER_ELBOW_PLATE_TOL_MM = 1.0`, plane built by `_plate_plane()` using `PLATE_THICKNESS_MM = 0.75` (`geometry_backend.py`) |
| Default | always on, no flag |
| Points used | the elbow origin only |
| Cost | reuses the FK from filter 4 |
| Why | Prevents the elbow dipping below the plate during travel moves |

The plane follows the **live posed** `T_user_frame`, so it moves with the build plate.

## Filter 6 — Under-Plate Footprint

| Detail | Value |
|---|---|
| Check | no sample point may sit inside the plate's XY shadow **and** below its print face |
| Constant | `FILTER_UNDER_PLATE_MARGIN_MM = 20.0` (`geometry_backend.py`) |
| Default | always on, no flag |
| Parts checked | arm links Robot1–Robot6 only (see below) |
| Cost | one `_link_sample_points` call, shared with filters 7 and 8 |
| Why | The arm reaching *under* the plate would hit the table or fixture. The XY margin catches near-misses at the plate edge |

Filters 6 and 7 together are the **finite** plate model. This matters: an infinite plane through the plate's top face is sound only while the plate sits below the whole arm, and at a User Frame 323.5 mm above the base it is not — it cuts through the shoulder and elbow, links nowhere near the print, and rejects every valid branch at planar waypoint 0. A real bed is finite and the arm legitimately reaches *around* it.

The test runs in **plate-local** coordinates (`_plate_box_frame()` returns `inv(T_user_frame)` plus the local bounds), so the footprint and slab tests are plain axis-aligned comparisons however the plate is posed.

## Filter 7 — Plate Volume Slab

| Detail | Value |
|---|---|
| Check | no sample point may lie inside the plate's bounding slab, expanded by the clearance |
| Constant | `FILTER_PLATE_SLAB_CLEARANCE_MM = 3.0` (`geometry_backend.py`) |
| Default | always on, no flag |
| Parts checked | same sample set as filter 6 |
| Cost | reuses filter 6's transformed points |
| Why | Catches link-through-plate cases the footprint test misses, e.g. a wrist passing through the plate edge from the side |

## Filter 8 — Surface Mesh Collision

| Detail | Value |
|---|---|
| Check | no sample point within the clearance of any triangle on the live layer's print surface |
| Constants | `CURVED_TIP_CLEARANCE_TOLERANCE_MM = 1.0` (**`examples/<study>/study_config.py`**), `SURFACE_GRID_CELL_MM = 8.0` (`geometry_backend.py`) |
| Default | **curved runs only** — gated on `ctx["surface_grid"] is not None` |
| Parts checked | same sample set as filters 6 and 7 |
| Cost | uniform-grid broadphase (`_build_surface_grid`) then point-to-triangle narrowphase (`_points_clear_surface`, `_point_triangle_distance2`) |
| Why | The arm must not drive through the workpiece it is printing on |

This is the **first mesh-vs-mesh check in the project**. It was declined at Stage 6.5 on the argument that a full-arm obstacle test "would reject every real printing pose" — true only while *one* orientation is commanded per waypoint. With 540 searched, it is both affordable and useful: it caught a real pose with an arm link **0.71 mm** into the TX surface that the pre-search path accepted.

The clearance lives in the study config, not here, because it is a nozzle/material property of a particular job (S1.41). The grid cell is ~6× the print surfaces' ~1.24 mm median edge, so a cell holds a handful of triangles and a query touches 27 of them. Only the layer's **own print surface** is a collision body; `CURVED_OBSTACLE_FILE` (`Surface_Bot.obj`) is not.

## Filter 9 — Robot/Tool Self-Collision

| Detail | Value |
|---|---|
| Check | no non-adjacent link-proxy pair closer than the clearance, by separating-axis test |
| Constants | `FILTER_SELF_COLLISION_CLEARANCE_MM = 5.0`, `SELF_COLLISION_PROXY_SEGMENT_MM = 80.0` (`geometry_backend.py`) |
| Default | always on, no flag |
| Pairs | `(i, j) for i in range(6) for j in range(i + 3, 6)`, plus `(6, k) for k in range(4)` |
| Cost | batch transform of all proxies → bounding-sphere pre-test → SAT on survivors |
| Why | Stops the arm folding onto itself — the forearm reaching the shoulder, the flange reaching the upper arm |

Two details here were forced by measurement, and both are easy to get wrong:

**One OBB per link is unusable.** Robot3's single box is 502 mm long and reported contact with Robot5/Robot6 in **all 8 branches** at planar waypoint 0, where the true mesh gap is 20–35 mm against a 5 mm clearance. Links are instead covered by a row of 80 mm boxes (`_obb_proxies`), which keeps the FR5's 425/395 mm links to 6–7 boxes each while staying tight enough not to invent collisions.

**Pairs must be at least three apart in the chain, not two.** Adjacent links share a joint and are in permanent contact, so nobody tests those. Links *two* apart are the subtle case: `(i, i+2)` is separated by one short link, and on the FR5's compact wrist (d4/d5/d6 = 102/102/100 mm) their meshes interpenetrate at every pose — their relative motion is a single joint rotation about a shared axis, so no joint value separates them. Robot4~Robot6 fired on every branch at a true 30 mm gap. Testing them rejects every pose the arm can hold, which is a broken filter, not a safe one.

Mind the index convention: these are **moving-geometry** indices, and `moving_geometry_rest_verts` is `rest_verts[:6] + [tcp_point]` — the static base Robot0 is not in it. Index `i` means **Robot(i+1)**, so `range(4)` is Robot1–Robot4.

## What the Filters Do Not Cover

Filters 6-8 sample moving-geometry indices `0-5` — the six **arm links only**. The tool point at index 6 is deliberately excluded (`sample_indices = tuple(range(6))`).

The reason is structural, not an oversight. The tool's entire collision body has been the single TCP point since the real TCP offset landed, and IK *pins that point to the commanded waypoint* — which lies **on** the print surface during a feed move, and at exactly the plate's top face on the planar first layer. Testing it against either would reject every legitimate printing pose, and against the plate would additionally turn float noise at `z == hi[2]` into a waypoint-0 abort.

This is the same trap that made the earlier tangent-plane nozzle check incapable of rejecting anything (measured: <1e-12 mm signed distance over all 5,863 cached waypoints). It is exactly what the reference implementation's `nozzle_tip_exclusion_mm` exists to avoid.

**State the consequence plainly: nothing here guards the nozzle against the workpiece — only the arm.** Recovering that needs a real tool body, which no asset currently provides (`nozzle.obj` is 163.47 mm against tool=1's 196.91 mm, and mounted at a compound angle). It is a corrected asset problem, not another filter.

## When Every Candidate Fails

If no candidate at a waypoint survives, `_fail_precompute` aborts the run and reports the waypoint index with its per-filter rejection tally, commonest first (`_reject_summary`). The planner does **not** relax a filter and does **not** fall back to a less-safe candidate. The tally is per waypoint, not cumulative — totals carried over from the thousands of waypoints that succeeded would drown out the one that failed.

The same is true at the edge level: if no traversable edge enters a layer, `_relax_candidate_layer` returns `(None, None)` and the run fails there.

## Edge Filter and Costs

`_relax_candidate_layer` relaxes one whole layer of the DAG at a time:

```python
D = np.abs(joints[None, :, :] - q_prev[:, None, :])
cost = D @ EDGE_JOINT_WEIGHTS
cost = cost + EDGE_BRANCH_CHANGE_PENALTY * (branches[None, :] != branch_prev[:, None])
roll_d = np.abs(rolls[None, :] - roll_prev[:, None])
roll_d = np.minimum(roll_d, ORIENT_SEARCH_ROLL_SLOTS - roll_d)   # wrapping
cost = cost + EDGE_ROLL_QUADRATIC_WEIGHT * np.maximum(0.0, roll_d - 1.0) ** 2

if <both waypoints are feed>:
    cost = np.where(D.max(axis=-1) > EDGE_MAX_JOINT_STEP_DEG, np.inf, cost)
```

| Term | Constant | Value | Effect |
|---|---|---|---|
| Max adjacent joint step | `EDGE_MAX_JOINT_STEP_DEG` | **30.0** | hard reject — edge cost set to `inf` |
| Weighted joint movement | `EDGE_JOINT_WEIGHTS` | `[3, 3, 2, 1, 1, 0.5]` | proximal joints cost more, pushing redundancy resolution out to the wrist |
| IK branch change | `EDGE_BRANCH_CHANGE_PENALTY` | **150.0** | flat, far above any plausible movement cost — switching elbow-up/elbow-down mid-path is legal but a last resort |
| Roll step ≤ 1 slot | — | 0 | adjacent roll slots are free |
| Roll step > 1 slot | `EDGE_ROLL_QUADRATIC_WEIGHT` | **2.0** | `2 × (excess)²` — a 5-slot jump costs 32, a 10-slot jump 162 |

Two things about the step limit are deliberate:

**It is an alias, not a fresh constant.** `EDGE_MAX_JOINT_STEP_DEG = JOINT_STEP_MAX_DEG`, the 30° threshold taken from the exchange spec's row 5. The reference implementation uses 35, and carrying that across would build a planner whose own edge filter admits jobs the receiving side rejects. Aliasing the spec value makes that mistake unavailable rather than merely discouraged.

**It applies feed-to-feed only.** Row 5 measures steps *within a continuous extrusion line*, and travel moves between disjoint toolpath pieces are legitimately large. Measured on planar (2026-09-08, from the shipped v7 cache): **15.49°** worst step overall against **4.43°** inside a feed segment, with **0** in-segment edges over the 30° limit. An unscoped E1 would abort the planar job at its first G0. Travel waypoints are dropped from export regardless.

These figures move when the solve changes, so treat them as a headroom indication rather than a constant — earlier measurements against different solves recorded 57.32°/5.85° (pre-orientation-search) and 4.58° in-segment (immediately after it). The conclusion has held throughout: nothing in a feed segment comes close to 30°.

## Path Selection

Surviving candidates form a layered DAG — nodes `(waypoint_index, candidate_index)`, edges between every pair of adjacent-waypoint candidates — searched for the minimum-cost path.

The implementation is a **vectorised layered relaxation**, not a heap frontier. It is the same algorithm as Dijkstra specialised to a layered DAG: because every edge goes from layer `i` to layer `i+1`, layers are already in topological order and one `argmin` over the previous layer relaxes all of them at once. A heap here would walk on the order of 5×10¹⁰ edges and never finish.

Relaxation is fused into the candidate sweep so the search can be chunked across frames and only the previous layer stays live. `_finish_candidate_search` then backtracks from the cheapest **finite** final cost, writes `precompute_joint_path` and `precompute_commanded_R`, and drops the per-layer candidate arrays — they are the precompute's peak memory (hundreds of MB for a curved layer) and nothing downstream reads them.

Note `precompute_commanded_R` (the orientation actually chosen) is kept separate from `precompute_R_target` (still the nominal surface normal), because the exchange spec's `normal_base` wants the nominal one and the printed beads stack on it.

> `dijkstra_candidate_path()` also exists in `geometry_backend.py` as the readable whole-graph statement of the same arithmetic. **It is never called.** Nothing keeps the two copies in step except reading them together — change one, change the other.

## Planar vs Curved

One parameter, `filter_mode` on `_begin_toolpath_precompute`, takes `"planar"` or `"curved"`:

| | Planar | Curved |
|---|---|---|
| Commanded frames | 1 — the constant plate frame | 540 from `orientation_candidates` |
| Candidates per waypoint | ≤8 | ≤4,320 |
| Filter 8 | off (no print surface exists) | on, against the live layer's surface |
| Filters 1-7, 9 | on | on |
| Chunk size | `PRECOMPUTE_CHUNK_SIZE` = 25 waypoints/frame | `SEARCH_CHUNK_SIZE` = 1 waypoint/frame |

The chunk sizes differ because 25 waypoints × 540 frames would be ~13,500 IK solves in a single render frame. Even at 1, the curved search is the slowest frame in the app — measured 437 ms/waypoint at the real User Frame.

## Tolerance Summary

| Parameter | Value | File |
|---|---|---|
| `ORIENT_SEARCH_TILT_MAX_DEG` | 20.0 | `geometry_backend.py` |
| `ORIENT_SEARCH_TILT_RING_AZIMUTHS` | 8 | `geometry_backend.py` |
| `ORIENT_SEARCH_ROLL_SLOTS` | 60 | `geometry_backend.py` |
| `ORIENT_SEARCH_FRAMES` | 540 (derived) | `geometry_backend.py` |
| `PHYSICAL_JOINT_LIMITS` | see `FR5_Joint_Limits.md` | `geometry_backend.py` |
| `FILTER_J5_MIN_DEG` | 2.0 | `geometry_backend.py` |
| `FILTER_J4_MIN_DEG` | −60.0 | `geometry_backend.py` |
| `FILTER_J4_ENABLED` | False | `geometry_backend.py` |
| `FILTER_UPPER_BRANCH_TOL_MM` | 2.0 | `geometry_backend.py` |
| `FILTER_ELBOW_PLATE_TOL_MM` | 1.0 | `geometry_backend.py` |
| `FILTER_UNDER_PLATE_MARGIN_MM` | 20.0 | `geometry_backend.py` |
| `FILTER_PLATE_SLAB_CLEARANCE_MM` | 3.0 | `geometry_backend.py` |
| `SURFACE_GRID_CELL_MM` | 8.0 | `geometry_backend.py` |
| `FILTER_SELF_COLLISION_CLEARANCE_MM` | 5.0 | `geometry_backend.py` |
| `SELF_COLLISION_PROXY_SEGMENT_MM` | 80.0 | `geometry_backend.py` |
| `LINK_SAMPLE_SPACING_MM` | 25.0 | `geometry_backend.py` |
| `PLATE_THICKNESS_MM` | 0.75 | `geometry_backend.py` |
| `EDGE_MAX_JOINT_STEP_DEG` | 30.0 (aliases `JOINT_STEP_MAX_DEG`) | `geometry_backend.py` |
| `EDGE_JOINT_WEIGHTS` | `[3, 3, 2, 1, 1, 0.5]` | `geometry_backend.py` |
| `EDGE_BRANCH_CHANGE_PENALTY` | 150.0 | `geometry_backend.py` |
| `EDGE_ROLL_QUADRATIC_WEIGHT` | 2.0 | `geometry_backend.py` |
| `CURVED_TIP_CLEARANCE_TOLERANCE_MM` | 1.0 | `examples/<study>/study_config.py` |

The split is a rule, not an accident: robot- and planner-level values live in `geometry_backend.py`; `study_config.py` holds only material- and nozzle-dependent job values (S1.41). Filter 8's clearance is the one filter value that is a property of the job rather than the arm.

## Where This Differs From the Reference Implementation

The design was adapted from another project's working implementation of this task. It is **not** a copy, and assuming the two match will mislead you. The deviations:

| | Reference | Here | Reason |
|---|---|---|---|
| Max adjacent joint step | 35° | **30°** | the exchange spec rejects steps >30°; 35 would admit jobs the receiver rejects |
| Step-filter scope | every adjacent pair | **feed-to-feed only** | travel steps are legitimately large (planar 15.49° overall vs 4.43° in-segment) |
| Surface clearance | 2.0 mm | **1.0 mm** | job-level value, lives in `study_config.py` |
| J5 minimum | 0° | **2°** | also subsumes spec row 7's `|J5| < 2°` WARN; costs 0/2,527 RX and 2/8,834 TX |
| Candidate set | 480 (60 roll × 8 IK, no cone) | **540 frames × ≤8 branches** | a 20° tilt cone was added |
| Tool in filters 6-8 | included, `nozzle_tip_exclusion_mm` 12.0 | **excluded entirely** | the whole tool body is one TCP point that IK pins to the surface being tested |
| Filter 9 proxies | one OBB per link | **80 mm proxy bands** | one box per link rejected all 8 branches at planar waypoint 0 |
| Filter 9 pairs | two apart | **three apart** | `(i, i+2)` meshes nest permanently on the FR5 wrist |
| Per-filter flags | one boolean each | **always on** except filter 3; filter 8 gated by `filter_mode` | fewer ways to ship a run with a filter silently off |
| Plain L1 / L2 edge costs | present as steering terms | **not implemented** | weighted-L1 alone was sufficient |
| Safe-branch mask post-pass | present | **not implemented** | no downstream consumer |
| Search | heap Dijkstra / A* (h=0) | **vectorised layered relaxation** | same algorithm; a heap cannot finish at this edge count |

Three of these were found **by measurement, not review**, and each would otherwise have been a filter that rejects everything: the tool exclusion, the multi-proxy boxes, and the three-apart pair rule. That is the pattern worth carrying forward — a filter that rejects every pose looks identical to a filter that is working, right up until you measure the rejection rate. Full record: `settled.md` **S1.47**.
