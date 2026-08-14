---
status: active
---

# 001_Inbox

Dated work notes, experiment logs, and plan drafts. **Non-authoritative** —
see [`TRUTH_LADDER.md`](../005_AgentMgmt/active/ctx_main/TRUTH_LADDER.md).
Ideas here may later be rejected or superseded; if this contradicts
`002_Architecture/settled.md`, settled.md wins.

## Entries

- [`2026-07-09_2d3d_printing_roadmap.md`](2026-07-09_2d3d_printing_roadmap.md) — Stage 5 (2D planar printer) & Stage 6 (curved-surface 3D printing) plan draft. **Superseded** by `tutorials/Stage5_README.md` and `tutorials/Stage6_README.md` — kept here as history per this folder's own convention, not corrected.
- [`2026-07-18_curved_surface_assets.md`](2026-07-18_curved_surface_assets.md) — Survey of the `assets/models/curved/` RX/TX toolpath assets: measured properties, two corrections to the first-pass reading, the Dijkstra constraint, and the two still-open supervisor questions. Fed into `tutorials/Stage6_README.md`.
- [`2026-07-22_stage6.7_playback_overlay_hide.md`](2026-07-22_stage6.7_playback_overlay_hide.md) — Hide guide overlays (geodesic path, travel moves, print-order gradient, orientation triads) during curved toolpath playback. Folded into `settled.md` S1.39 / `tutorials/Stage6_README.md` 6.7.
- [`2026-07-22_stage6.8_posed_plate_collision.md`](2026-07-22_stage6.8_posed_plate_collision.md) — Replace the crude world-z=0 collision proxy with a real posed-plate check: arm always blocked, TCP/nozzle optionally passes through. Folded into `settled.md` S1.40 / `tutorials/Stage6_README.md` 6.8.
- [`2026-07-22_stage7_calibration_and_external_ik.md`](2026-07-22_stage7_calibration_and_external_ik.md) — Stage 7 plan: real TCP offset, the exchange spec's rejection criteria, real User Frame calibration, and job export to the external IK exchange format. Feeds `tutorials/Stage7_README.md`.
- [`2026-08-14_current_rejection_criteria.md`](2026-08-14_current_rejection_criteria.md) — Snapshot of what the code rejects **today**, per printing mode (manual IK / planar / curved), with the two limitations that matter: joint limits use the conservative slider range, and "solved" is not "collision-free". Captured before Stage 7 §7.2 replaces it.
