---
status: inbox
stage: pre-7
scope: geometry_backend.py, gui_panel.py
---

# Current Rejection Criteria, by Printing Mode

## Scope

**A snapshot of what the code rejects *today*, before Stage 7 changes it.**
Stage 7 replaces this wholesale — §7.2 adopts the exchange spec's seven-row
Rejection Criteria table and sets `check_collision=False` on the curved path,
and §7.1 reduces the tool to a single TCP point. Captured now because the
current behaviour is otherwise only readable by tracing four functions.

Non-authoritative (`TRUTH_LADDER.md`) — if this and the code disagree, the code
wins.

## The table

Three paths reach IK. Only two of them print.

| Check | Manual IK panel | Planar (G-code) | Curved (per layer) |
|---|---|---|---|
| Geometric reachability | rejects branch | **aborts run** | **aborts run** |
| Joint limits | rejects branch | **aborts run** | **aborts run** |
| Posed plate — arm links 0–5 | *not checked* | **always blocks** | **always blocks** |
| Posed plate — nozzle (mesh 6) | *not checked* | blocks unless `allow_tcp_through_plate` | blocks unless `allow_tcp_through_plate` |
| Surface tangent plane — nozzle only | n/a | n/a | **blocks**, 1.0mm inward slack |
| Wrist singularity | label only | flagged, never rejects | flagged, never rejects |
| Arm vs mockup / workpiece | ✗ never | ✗ never | ✗ never |
| Arm self-collision | ✗ never | ✗ never | ✗ never |
| Joint step between waypoints | n/a | ✗ not checked | ✗ not checked |
| On failure | returns 0 solutions | aborts whole path | aborts whole path |

**Curved = planar + one extra row.** The tangent-plane check is the only
difference between the two printing modes.

## Where each check lives

| Check | Implementation |
|---|---|
| Geometric reachability | `solve_ik()` — returns no branch for an unreachable pose |
| Joint limits | `solve_ik_tcp_matrix()` → `wrap_into_limits` (tries ±0/360 so asymmetric windows resolve) |
| Posed plate | `_branch_clears_ground()` → `_meshes_clear_plane()`, plane from `_plate_plane()` |
| Tangent plane | `_nozzle_clears_plane()` (S1.37) |
| Abort contract | `step_toolpath_ik_precompute()` |

**Planar vs curved is one argument.** `_begin_toolpath_precompute()`'s
`tip_tolerance_mm`: `None` from `run_toolpath_ik_precompute` (planar),
`CURVED_TIP_CLEARANCE_TOLERANCE_MM` (= 1.0) from
`run_curved_toolpath_ik_precompute`. Non-`None` is what enables the
tangent-plane row. That single value is the whole difference.

## Two things the table doesn't show

**Branch choice is not "first valid".** Branches are ranked by summed
wrapped-angle distance to the **previous waypoint's** solved pose (continuity,
S1.5/S1.11), then the first *ranked* branch that clears is taken. So clearance
filters an already-ordered list; it does not pick.

**Failure is all-or-nothing.** The first waypoint with no valid or no clearing
branch aborts the entire precompute — no partial joint path is kept (S1.12).
The status line names which check failed and at which waypoint.

## ⚠ Two limitations worth knowing before trusting a solve

### 1. Joint limits are the conservative range, not the physical one

Every mode passes `gui_panel.JOINT_LIMITS`, a deliberately **practical slider
range**, not the robot's real limits:

| Joint | Used here | Physical (`docs/FR5_Joint_Limits.md`) |
|---|---|---|
| J1 | −170 … +170 | −174 … +174 |
| **J2** | **−130 … +80** | **−264 … +84** |
| J3 | −155 … +155 | −159 … +159 |
| **J4** | **−170 … +80** | **−264 … +84** |
| J5 | −170 … +170 | −174 … +174 |
| J6 | −170 … +170 | −174 … +174 |

Strictly inside the physical limits on all six joints, so nothing unsafe is
accepted — but poses the real robot **could** reach are rejected here, J2 and J4
by a wide margin. Stage 7 §7.2's joint-limit row must use the physical values.

### 2. "Solved" does not mean "collision-free"

A completed precompute means **reachable and plate-clearing**. Nothing more.

- There is **no mesh-vs-mesh collision anywhere in this project.** Arm links are
  only ever tested against the plate *plane*.
- S1.37's tangent-plane check is **nozzle-only by deliberate design** — testing
  the arm links "would reject every real printing pose", since the arm
  legitimately sits inward of a local tangent plane while reaching its contact
  point.
- `CURVED_OBSTACLE_FILE` (`Surface_Bot.obj`) is loaded and displayed but used
  **only** to orient surface normals — it is *not* a collision body. The
  obstacle-mesh approach was rejected as too slow and replaced by the tangent
  plane (S1.37).

Observed consequence: a completed TX precompute (2,688 waypoints) drives the
**arm through the shoulder mockup**. Judge that by eye — see
[`../003_Guides/CurvedModel_PrintSetup.md`](../003_Guides/CurvedModel_PrintSetup.md).

## What Stage 7 changes

- **§7.1** — the tool becomes a single TCP point (the nozzle mesh is hidden as
  it is not the calibrated tool), so no tool-*body* check remains on any path.
  ⚠ **The "hidden" half is stale as of §7.7** (2026-09-04, `settled.md` S1.51):
  the mesh is visible again, re-aimed at load time onto the TCP frame's −Z. The
  load-bearing clause is unaffected — the tool is still a single TCP point and
  there is still no tool-*body* check on any path, because only the render pose
  changed, not the asset's uncalibrated shape.
- **§7.2** — this table is replaced by the exchange spec's seven rows (identity,
  TCP offset, joint limits, per-point FK, joint step, `num_points`, `|J5|<2°`
  warn), and the curved path gets `check_collision=False`: rows 3–5 disappear
  there entirely. Planar keeps them.
- Note the spec's rows validate **data**, not **geometry** — so after §7.2 an
  export can pass every check and still drive the arm through the workpiece.

See `2026-07-22_stage7_calibration_and_external_ik.md` §7.1/§7.2.

---

## Changed in Stage 7.2 — implemented 2026-08-15 (`settled.md` S1.44)

The table above is now **historical**. What actually landed, and where it
differed from the prediction:

**As predicted.** The curved path lost both clearance checks; planar kept Stage
6.8 behaviour exactly; the spec's seven rows went in verbatim as
`validate_job()`; `PRECOMPUTE_CACHE_VERSION` bumped (5 → 6, not 4 → 5 — §7.1
had already taken it to 5).

**Differed — the tangent-plane check was already dead.** This note describes it
as an active row that "blocks, 1.0mm inward slack". It had not blocked anything
since §7.1: that stage made the tool's collision body the single TCP point,
which IK pins to the very plane being tested. Measured before deletion — 7,471
evaluations, zero rejections, worst signed distance 3.4e-13mm against a 1.0mm
tolerance. So removing it changed no outcome, and the "Curved = planar + one
extra row" summary above was already untrue when written.

**Differed — the joint-limit limitation was fixed, not just noted.** §2's
warning that every mode passes the conservative slider range is resolved:
`PHYSICAL_JOINT_LIMITS` now feeds the precompute, the manual IK panel, and the
spec's row 3. `gui_panel.JOINT_LIMITS` governs the sliders only. Measured
effect: 425 valid branches vs 207 over an 80-pose sample.

**Still true, and now worse.** §2's "solved does not mean collision-free" holds
with more force: the curved path has *no* geometric rejection at all, and
nozzle-vs-workpiece protection has been absent since §7.1. See
[`../003_Guides/CurvedModel_PrintSetup.md`](../003_Guides/CurvedModel_PrintSetup.md)
"Changed in Stage 7.2".

**New, found by the criteria.** Curved solved paths **fail** the spec's
joint-step row — 23/35 RX and 15/35 TX segments contain >30° steps inside a feed
run, from a reference-axis flip in `_orientation_frames_for_points`. Planar
passes all seven rows cleanly (worst within-segment step 5.85°, 0/20,350
segments violating). See
[`2026-08-15_orientation_frame_flips_row5.md`](2026-08-15_orientation_frame_flips_row5.md).
