---
status: closed
closed: 2026-09-03
closed_by: CURVED_MODEL_XY_OFFSET_MM (study_config.py), settled.md S1.48
stage: post-7.4
scope: geometry_backend.py (load_curved_model), assets/buildPlate/BambuLab_BuildPlate.obj
blocks: re-running the curved pipeline at the real User Frame; roadmap 7.5 for the curved path
---

> ✅ **CLOSED 2026-09-03 — fixed the same day it was found, by measurement, per
> explicit direction not to gate on asking the supervisor a fresh question:**
> the User Frame and TCP calibration data are already measured and verified, so
> whichever placement makes the job reachable **is** correct.
>
> Fix option 1/3 hybrid, exactly as sketched below: `load_curved_model()` no
> longer derives XY placement from the build-plate mesh at all. A new
> `CURVED_MODEL_XY_OFFSET_MM = np.array([0.0, 0.0])` in `study_config.py`
> centers the workpiece directly on the User Frame origin — the offset
> confirmed below to work, made an explicit, named, study-level constant rather
> than a silent side effect of a stand-in asset's bounding box. Two-line diff in
> `geometry_backend.py`'s `T_placement` (see the fix sketch below, implemented
> verbatim); the `plate = self.load_mesh(...)` load for bounds is deleted
> outright, since nothing else in the function used it.
>
> **Full rebuild, real User Frame, headless, 2026-09-03:**
>
> | Layer | Feed points | Solved | `validate_job` |
> |---|---|---|---|
> | RX | 2,527 | **3,175 / 3,175 waypoints (100%)** | **ACCEPTED** |
> | TX | 2,000 | **2,688 / 2,688 waypoints (100%)** | **ACCEPTED** |
>
> Matches the "at the User Frame origin" test below exactly, now run through the
> *actual* 7.4 pipeline (540-orientation search, all nine filters, the candidate
> DAG) rather than the coarser IK-only sampling that test used — confirming the
> earlier result wasn't an artefact of that sampling.
>
> Geodesic travel totals matched the S1.35 baseline **exactly** (RX 690mm vs
> 5157mm file-order, TX 607mm vs 4848mm) as a sanity check that only XY moved —
> rigid translation, so intrinsic geodesic distances couldn't have changed if
> the fix were correct, and they didn't.
>
> One observation, not a problem: the max joint step **overall** on the solved
> path is large (81.74° RX, 275.33° TX) — but measured to occur only between two
> **travel** waypoints, never within a feed segment (worst within-segment step:
> 29.93° RX / 29.85° TX, both under the 30° limit, which is exactly why
> `validate_job` accepted). E1's hard rejection is scoped to feed-to-feed edges
> only (see `settled.md` S1.47), so this is architecturally expected, and travel
> waypoints are dropped from export regardless. Might read as a visual "jump" in
> playback; does not affect correctness or export.
>
> Fresh `.npz` caches saved to `assets/models/curved/curved_{rx,tx}.precompute.npz`.
> No `PRECOMPUTE_CACHE_VERSION` bump — the cache key already hashes waypoint
> positions, so the old caches missed automatically.
>
> **Visually confirmed, not just predicted:** a headless screenshot of "Load
> Curved Model" at the real frame shows the workpiece sitting at the User
> Frame's origin triad — the plate mesh's corner, since its own local origin is
> a corner — and extending past the visible plate's edge, rather than centred
> on it. Exactly the expected consequence, left as-is: the plate mesh's
> correspondence to the real fixture is a separate open question (see
> "Secondary" below), and moving it to visually match is not this fix's job.
>
> Full record: `settled.md` **S1.48**.

# Curved placement — the workpiece is centred on a stand-in plate mesh, 105.6mm too far out

## What happened

Roadmap 7.4's orientation search raised curved reachability at the real User
Frame **8.5×** (226/2,527 → 1,922/2,527 RX; 186/2,000 → 1,410/2,000 TX), but the
precompute still aborts: ~24% of feed waypoints have no IK solution at **any** of
the 540 searched orientations.

The supervisor has confirmed the 7.3 configuration is correct, so "the arm can't
reach it" needed an explanation that does not blame the frame. It has one, and
the frame is not at fault.

**`load_curved_model()` does not place the workpiece at the User Frame. It
centres it on the build plate MESH's bounding-box centre** —
`geometry_backend.py`, `load_curved_model()`:

```python
T_placement[:2, 3] = (plate_min[:2] + plate_max[:2]) / 2.0 - (assembly_min[:2] + assembly_max[:2]) / 2.0
T_curved = self.T_user_frame @ T_placement
```

`BambuLab_BuildPlate.obj` is 258 × 276mm with its local origin at a **corner**
(local bbox x `0..258`, y `-10..265.9`), so its centre is at plate-local
`(129, 128)`. Pushed through the real frame's ~−89° yaw:

| | distance from robot base |
|---|---|
| User Frame origin itself | **737.5 mm** |
| Where the model actually lands (plate centre) | **843.1 mm** |
| **Offset introduced by the centring** | **+105.6 mm** |

## Why 105.6mm is the whole story

The FR5's flange reach is `a2 + a3 + d5` = 425 + 395 + 102 = **922 mm**. The
curved assembly is ~140mm across, so a 105.6mm outward shift pushes its far half
straight through the envelope boundary. Reachability does not degrade gradually —
it falls off a cliff at exactly 922mm.

Measured 2026-09-03, real User Frame, `PHYSICAL_JOINT_LIMITS`, IK-reachability
only (no filters), every 3rd feed waypoint × every 5th of the 540 orientations:

| TCP distance from base | RX reachable | TX reachable |
|---|---|---|
| 800–850 mm | 100/100 (100%) | 129/129 (100%) |
| 850–900 mm | 179/179 (100%) | 98/98 (100%) |
| 900–920 mm | 219/233 (94.0%) | 136/144 (94.4%) |
| **920–950 mm** | **171/331 (51.7%)** | **126/296 (42.6%)** |

TCP distances span 808–945mm (RX, median 912) and 806–947mm (TX, median 916).
The `a2 + a3 = 820mm` chain plus the wrist offsets simply runs out, and the
196.91mm flange→TCP offset cannot rescue it: that offset is **lateral**, so it
only extends reach when the tool points outward, which fights the
perpendicular-within-20° constraint.

## The confirming test

Same real User Frame, same model, same solver, same everything — with **only the
centring offset removed**, so the assembly sits on the User Frame origin instead
of the plate-mesh centre:

| Layer | at plate centre (current code) | **at User Frame origin** |
|---|---|---|
| RX | 1,922 / 2,527 admissible (76.1%) | **843 / 843 reachable (100%)** |
| TX | 1,410 / 2,000 admissible (70.5%) | **667 / 667 reachable (100%)** |

Note the right-hand column used a **coarser** orientation sample (108 of 540)
than the failing runs, so it is a conservative result: with the full search it
can only be better.

Method: `vis.load_build_plate(position_mm - R @ [129, 128, 0], rpy_deg)` before
`load_curved_model()`, which makes the plate-centre centring land the assembly on
the original User Frame origin.

## What this rules out

Each of these was a live hypothesis and each is now measured false:

- **Not the frame.** `saved_position.json` is the real calibrated `user_index=1`
  and the supervisor confirms it. Moving the model relative to it fixes
  everything; the frame itself is untouched.
- **Not the arm's reach.** 100% of both layers is reachable when placed 105.6mm
  closer, at the same frame.
- **Not the commanded pose.** That was S1.45's reading, corrected by S1.46/S1.47
  — the orientation search already recovered 8.5× of it. The residual is pure
  geometry.
- **Not the filters.** A control run at the default plate pose
  `[-570, -300, -100]` gives **2,527/2,527 RX and 2,000/2,000 TX admissible**
  with all nine filters active (S1.47). Given a sensible placement they cost
  nothing. ⚠ **Do not tune the filters to "fix" curved reachability.**
- **Not the plate collision model.** 7.4 replaced S1.40's infinite plane with a
  finite footprint + slab, which fixed the *planar* abort outright
  (181,375/181,375). The curved failures are `"Unreachable: no geometric
  solution"`, before any collision test runs.

## The question for the supervisor

Narrower than "where should the model go":

> **Is user frame 1 defined at the CORNER of the print bed, or at its CENTRE?**

This project's code assumes **corner**: `load_build_plate()` treats
`position_mm` as the plate's resting/bottom face with the mesh's local origin at
a corner, and then `load_curved_model()` centres the workpiece 129/128mm away
from it. If the real frame is defined at the **centre** of the fixture — the more
common convention for a taught user frame — then the workpiece belongs on the
origin, and by the test above the entire job solves.

Secondary, and already open in `BOOT_MATRIX.md`: **does
`BambuLab_BuildPlate.obj` describe what physically sits at the User Frame at
all?** It was a stand-in chosen at the Stage 6.8 demo pose. Its 258 × 276mm
footprint is currently load-bearing for three separate things — the centring
offset above, filter 6's under-plate footprint, and filter 7's slab — and none of
them has been checked against the real cell.

## Fix sketch (NOT implemented — needs the answer above first)

Cheapest first, and the choice is a data question, not a coding one:

1. **Centre the workpiece on the User Frame origin**, not the plate mesh. One
   line in `load_curved_model()` (drop the plate-bbox term from
   `T_placement[:2, 3]`). Correct **if** the frame is centre-defined. Measured to
   give 100% on both layers.
2. **Keep the plate centring but supply the real fixture mesh.** Correct if the
   frame is corner-defined and the real bed simply differs from the Bambu Lab
   stand-in. Also fixes filters 6/7, which is the stronger argument for it.
3. **An explicit workpiece-placement offset in `study_config.py`**, separate from
   the plate. Most honest structurally — where the *workpiece* sits is a study
   fact, not a property of the build plate — and it stops a stand-in asset
   silently determining reachability. Probably the right end state whichever of
   1 or 2 is true.

Any of these invalidates every curved precompute cache and needs the full
6.1→6.4 rebuild before re-running, but **no `PRECOMPUTE_CACHE_VERSION` bump**:
`_curved_toolpath_cache_meta` hashes the waypoint positions, so a moved model
misses by construction.

## Scope note

Deliberately **not** fixed as part of 7.4. 7.4's remit was the orientation search
and the filter set; both are built, verified, and independently correct — the
planar path went from aborting at waypoint 0 to solving all 181,375 waypoints at
this same frame. 7.4 is also what made this diagnosable: with one commanded
orientation per waypoint the failure looked like "the arm can't reach", and only
after searching 540 could the residual be isolated to placement.

Guessing between options 1–3 would produce a curved pipeline rebuild against an
assumption, which is exactly what S1.45 said not to do.

See `settled.md` **S1.47** (the 7.4 measurements),
[`2026-08-15_real_user_frame_reachability.md`](2026-08-15_real_user_frame_reachability.md)
(S1.45's original measurements, still valid), and
[`../003_Guides/CurvedModel_PrintSetup.md`](../003_Guides/CurvedModel_PrintSetup.md).
