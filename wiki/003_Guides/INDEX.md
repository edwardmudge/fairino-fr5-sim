---
status: active
---

# 003_Guides

User-facing operation guides — how to drive a feature, and how to change it for
your own use. Add one guide per file as features stabilise.

| Guide | Covers |
|-------|--------|
| [TCP_Trajectory.md](TCP_Trajectory.md) | How the TCP trajectory curve is recorded, and how to tune its sampling rate and line thickness |
| [TCP_Frame.md](TCP_Frame.md) | Why the TCP triad rotates with the tool (not just translates), and how to tune its size |
| [BuildPlate_UserFrame.md](BuildPlate_UserFrame.md) | How the build plate's re-posable user frame works (position + rotation, Move/Reset/Save/Load buttons), and why the default location was chosen |
| [Gcode_Toolpath.md](Gcode_Toolpath.md) | How the G-code toolpath preview is parsed and placed on the build plate, current scope, and where it could grow |
| [CurvedModel_Loading.md](CurvedModel_Loading.md) | How the curved-surface toolpath PLYs and surface meshes are parsed, reconstructed, and placed above the build plate (roadmap Stage 6.1) |
| [CurvedModel_Geodesics.md](CurvedModel_Geodesics.md) | How geodesic distances across each print surface are precomputed (Dijkstra over the surface graph), and how to read the cost matrices (Stage 6.2) |
| [CurvedModel_PrintOrder.md](CurvedModel_PrintOrder.md) | How per-layer print order and travel moves are derived from the geodesic costs, and the constants that tune the overlay (Stage 6.3) |
| [CurvedModel_Orientation.md](CurvedModel_Orientation.md) | How a target TCP orientation is attached to every feed point from the surface normal, holding the nozzle perpendicular (Stage 6.4) |
| [CurvedModel_IKPrecompute.md](CurvedModel_IKPrecompute.md) | How the chunked curved IK precompute works, what clearance it checks, and how its per-layer cache is keyed (Stage 6.5) |
| [CurvedModel_PrintSetup.md](CurvedModel_PrintSetup.md) | **Operating procedure** — the plate-then-model load order that produces a complete RX/TX solve, why the sequence matters, and why "solved" is not "collision-free" |
| [CurvedModel_AdaptingYourOwnJob.md](CurvedModel_AdaptingYourOwnJob.md) | **Start here to print your own part** — selecting a study config via `FR5_STUDY_CONFIG`, the PLY/OBJ asset formats, the placement reach constraint, which constants to re-tune, cache invalidation, and the export format |
