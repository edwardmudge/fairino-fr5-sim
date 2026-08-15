---
status: inbox
stage: post-7.2
scope: geometry_backend.py (_orientation_frames_for_points)
---

# Curved jobs fail the exchange spec's joint-step row — the reference-axis flip

## What happened

Roadmap 7.2 implemented the external IK exchange spec's Rejection Criteria.
Running the new `validate_job()` over the **existing** curved solved paths, the
joint-step row (>30° between adjacent points **within** a segment) fails on both
layers:

| Layer | Segments | Violating row 5 | Worst step inside a feed segment |
|---|---|---|---|
| RX | 35 | **23** | **180.10°** |
| TX | 35 | **15** | **127.96°** |

These are inside continuous extrusion runs, **not** at travel boundaries — the
segment builder was validated first (35 segments == 35 print-order pieces on
both layers, segment lengths summing exactly to the feed-waypoint count).

Measured 2026-08-15, headless `fairino-fr5-sim`, plate at the 6.8 adopted pose
`[-570, -300, -200]`, against the cached RX/TX joint paths.

**A curved job exported today would be rejected by the receiving side.**

## Root cause

`_orientation_frames_for_points()` (`geometry_backend.py`, S1.36) picks the
in-plane reference axis per waypoint:

```python
a = world_axes[np.argmin(np.abs(world_axes @ z))]  # most perpendicular to z
```

As the surface normal `z` sweeps along a curve, `argmin` switches from one world
axis to another **discretely**. That rotates the commanded TCP frame about its
own Z by a large angle between two adjacent waypoints. The nozzle is
rotationally symmetric about that axis, so the spin is physically free and
changes nothing about where the bead is laid — but IK tracks the commanded frame
faithfully, so J6 jumps, and sometimes the branch ranking follows it into a
different branch (hence the J3/J4 offenders too).

Correlation is close to 1:1:

| Layer | Reference-axis switches | Adjacent steps >30° |
|---|---|---|
| RX | 74 | 78 |
| TX | 62 | 63 |

The method's docstring states the intent as *"whichever world axis is most
perpendicular to Z, so the projection never collapses and adjacent frames don't
flip"*. It achieves the first half and causes the second: guarding against the
projection **collapsing** is not the same as keeping the choice **continuous**.

## Why it was invisible until now

Nothing before 7.2 looked at inter-waypoint joint continuity. The precompute
ranks branches for continuity against the previous waypoint (S1.5/S1.11), but
that ranks *branches for a given commanded frame* — it cannot undo a
discontinuity in the commanded frame itself. Playback shows the flip as a fast
wrist spin, which reads as cosmetic.

7.1 also measured **57.32°** as the max planar step and noted for 7.2 that the
large steps were "almost certainly G0 travel moves, which are segment
boundaries". Both halves are now measured:

| Path | Max step, whole path | Max step **within** a segment | Segments violating row 5 |
|---|---|---|---|
| Planar | 57.32° | **5.85°** | **0 / 20,350** |
| Curved RX | — | **180.10°** | **23 / 35** |
| Curved TX | — | **127.96°** | **15 / 35** |

So the hypothesis was right for planar and wrong for curved. The planar job
passes all seven rows and exports cleanly; the curved one does not. That
contrast is the evidence this is an orientation-frame problem specific to the
curved path, not something general about the solver or the segment definition —
planar uses a single constant `R_target` (S1.12) and never hits it.

## Fix sketch (not implemented — needs a decision)

The DOF is free, so the fix is to choose it *continuously* rather than
independently per waypoint. Options, cheapest first:

1. **Propagate the previous frame.** Project the previous waypoint's X axis into
   the current tangent plane and re-orthonormalise, falling back to the current
   world-axis rule only at the start of each piece. Parallel transport along the
   path; O(1) per waypoint, no new dependency. Frames stay continuous within a
   segment by construction, which is exactly the scope row 5 measures.
2. **Keep the world-axis rule but unwrap afterwards.** Post-process the solved
   joint path, adding ±360°/±180° to J6 where it would reduce the step. Cheaper
   to write, but only masks a genuinely discontinuous *commanded pose*, and
   `tcp_xyz_base_mm` still tells the receiving side the tool spun.
3. **Hysteresis on `argmin`.** Only switch reference axis when the new one beats
   the current by a margin. Reduces the switch count but does not remove the
   discontinuity when it does switch.

Option 1 is the real fix. It changes `R_target` for the curved path, so it
invalidates every curved precompute cache and needs another
`PRECOMPUTE_CACHE_VERSION` bump, and S1.36 would need superseding.

## Scope note

Deliberately **not** fixed as part of 7.2. 7.2's remit was to implement the
criteria; the criteria then found this. It is a Stage 6.4 (S1.36) defect, and
fixing it means re-running the whole curved pipeline — which 7.3 is about to
force anyway when the real User Frame lands. Sequencing it with 7.3 avoids
paying for two full curved rebuilds.

See `wiki/002_Architecture/settled.md` S1.44 and
`wiki/001_Inbox/2026-07-22_stage7_calibration_and_external_ik.md` §7.2.
