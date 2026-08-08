---
status: active
scope: current-truth
last_verified_against_code: 2026-07-22
---

# Agent Boot File — FR5 Simulator (Current State)

## Step 0: Who You Are

You're helping build an offline FK/IK simulator for a Fairino FR5 6-axis
arm, rendered in Polyscope. No real robot or hardware connection — this is
pure math + visualisation.

## Step 1: 30-Second Project Overview

### What the System Does

Takes 6 joint angles (or a target end-effector pose) and renders the
corresponding FR5 arm configuration in an interactive 3D window.

### Current Status

| Feature | Status | Notes |
|---------|--------|-------|
| Polyscope app skeleton | done | `main.py` wires `VisContent`/`UI_Menu` into Polyscope's per-frame callback |
| FK maths (`compute_fk`) | done | See `docs/FR5_DH_Table.md` |
| Mesh loading + rendering | done | Delta-transform pipeline — see `docs/FR5_Mesh_Convention.md` |
| Joint sliders | done | "Forward Kinematics" panel, `gui_panel.py` |
| Tool head + TCP tracking | done | See `wiki/003_Guides/TCP_Frame.md` |
| Analytical IK | done | Closed-form solver, multi-solution — see `settled.md` S1.4/S1.5 |
| TCP trajectory recording | done | See `wiki/003_Guides/TCP_Trajectory.md` |
| G-code toolpath preview | done | G0/G1-only parser, fixed `model.gcode` path, registered via an explicit "Load G-code preview" click — does **not** auto-reload on plate reposition (that auto-reload was removed, see `settled.md` S1.23); removable via a conditionally-shown "Clear G-code preview" button (`clear_gcode_preview()`) — see `wiki/003_Guides/Gcode_Toolpath.md`, `settled.md` S1.7/S1.8 |
| Build-plate position/orientation | done | Re-posable via Move/Reset/Save/Load Position buttons — see `wiki/003_Guides/BuildPlate_UserFrame.md`, `settled.md` S1.6 |
| Toolpath IK precompute | done | Chunked (`PRECOMPUTE_CHUNK_SIZE` waypoints/frame), pausable/resumable/cancellable, ground-clearance filtered — see `settled.md` S1.14/S1.15 |
| Toolpath precompute disk cache | done | `assets/models/planar/gcode/model.precompute.npz`, keyed on G-code SHA-256 + build-plate pose + version; loaded before re-solving on `run_toolpath_ik_precompute()` — see `settled.md` S1.21 |
| Toolpath playback | done | Progressive-reveal (beads start invisible, revealed as playback crosses them), render-throttled (`PLAYBACK_RENDER_STRIDE`, `PLAYBACK_LOOKAHEAD_BEADS`) — see `settled.md` S1.16/S1.17-S1.20 |
| Precompute/playback invalidation on plate move | done | In-session: `load_build_plate()` compares the new pose against the pose captured at precompute-start and invalidates both if it differs — see `settled.md` S1.22 |
| Curved-surface model loading (Stage 6.1) | done | 55 toolpath PLY files (reconstructing to 70 polylines) + 3 surface OBJ meshes placed above the plate via a "Load Curved Model" button; retains the placed geometry in world coordinates for 6.2 — see `wiki/003_Guides/CurvedModel_Loading.md`, `settled.md` S1.29/S1.30 |
| Geodesic routing over print surfaces (Stage 6.2) | done | Two per-surface CSR graphs + hand-rolled `heapq` Dijkstra producing two 70×70 geodesic cost matrices, chunked one source per frame; in-memory only, no disk cache — see `wiki/003_Guides/CurvedModel_Geodesics.md`, `settled.md` S1.31 |
| Curved-surface print ordering (Stage 6.3) | done | Per-layer TSP-variant ordering (greedy nearest-endpoint seed + 2-opt over oriented pieces) off the 6.2 cost matrices, RX first then TX, via a synchronous "Build Print Order" button. Travel moves are the 6.2 geodesics hovered `CURVED_TRAVEL_HOVER_MM` outward along from-scratch surface normals (`compute_vertex_normals`, no scipy; oriented away from `Surface_Bot`), bookended with true endpoints. Printed pieces render as a **print-order gradient** (`Curved Order Feed`), travel in a distinct flat colour, and the RX/TX selector isolates one layer at a time (`apply_live_layer_visibility`, strict for now — S1.32 stack rule later). Measured RX 690mm vs 5157mm / TX 607mm vs 4848mm file-order travel — see `wiki/003_Guides/CurvedModel_PrintOrder.md`, `settled.md` S1.35 |
| Per-waypoint tool orientation (Stage 6.4) | done | `build_orientation_frames()` attaches a per-feed-point TCP orientation (nozzle perpendicular to the shell): **Z = outward surface normal**, in-plane axes pinned to a fixed world reference (not the path tangent) so the symmetric nozzle doesn't spin as the path meanders — "stable and straight". Supersedes S1.12's single-constant `R_target`. Stored per layer as `curved_orient_frames` (the array 6.5 feeds to IK) and drawn as a downsampled triad overlay (`Curved Orient Frames`, X/Y/Z = red/green/blue) via a "Build Orientation Frames" button. Compute + visualise only; IK wiring is 6.5 — see `wiki/003_Guides/CurvedModel_Orientation.md`, `settled.md` S1.36 |
| Curved IK precompute (Stage 6.5) | done | Reuses Stage 5's chunked precompute through one shared seam, `_begin_toolpath_precompute()`; `run_curved_toolpath_ik_precompute(layer, ...)` feeds it from `build_curved_toolpath_waypoints_world(layer)` (6.3's ordered feed pieces + travel hops, each carrying 6.4's per-waypoint orientation). `precompute_R_target` is now an `(N,3,3)` array (planar path broadcasts its one constant, unchanged behaviour). Nozzle clearance uses no obstacle mesh: each waypoint's own outward tangent plane is a supporting hyperplane for the convex mockup stack, checked against the **nozzle tip only** (`_nozzle_clears_plane`, `CURVED_TIP_CLEARANCE_TOLERANCE_MM` inward slack); world `z=0` is dropped for the curved case. Per-layer disk caches (`curved_rx/tx.precompute.npz`, `PRECOMPUTE_CACHE_VERSION` bumped 1→2). `geometry_backend.py`-only; no GUI hookup yet (6.6) — see `wiki/003_Guides/CurvedModel_IKPrecompute.md`, `settled.md` S1.37 |
| Curved GUI wiring (Stage 6.6) | done | Wires 6.1-6.5 into the panel + adds curved playback. **One source-aware control set**, not a duplicate: a "Toolpath Source" selector sets `toolpath_source` (-1 planar / 0..N-1 layer) and the existing Run/Pause/Cancel/Reset controls dispatch via it (`run_active_toolpath_ik_precompute`, source-aware `*_toolpath_playback`); a layer-mixup guard force-cancels a paused run of a different source instead of silently resuming it. Per-layer curved bead playback (`_build_curved_beads`, fixed cross-section swept along each waypoint's surface normal) coexists across layers, so `apply_live_layer_visibility` now does the real S1.32 stack (`i <= layer`: TX shows the printed RX beneath). `clear_curved_model()` Load/Clear pair. **Toggleable z=0 ground check** (`reject_below_ground`, default ON, applies to both paths, layered on the tangent-plane check for curved; folded into the cache key, `PRECOMPUTE_CACHE_VERSION` 2→3). Top-down build panel + "Curved Model Properties" dropdown (`curved_model_summary()`) — see `settled.md` **S1.38** |
| Hide guide overlays during playback (Stage 6.7) | done | New `playback_active` flag, distinct from `playback_running` (survives Pause, cleared only by Reset). `run_toolpath_playback()` sets it and re-applies `apply_live_layer_visibility`, which now force-hides the order-feed/travel/orient curve networks + base toolpath curve while `playback_active` — surfaces and growing beads keep the `i <= layer` stack rule, so you can watch the object form. `reset_toolpath_playback()` clears it and restores the full guide view. Planar source is a no-op (the `G-code Print` mesh is the playback mesh). `gui_panel.py` untouched — see `settled.md` **S1.39** |
| Posed-plate collision (Stage 6.8) | done | Replaces the world-`z=0` proxy with a check against the **actual posed build plate** (`_plate_plane()` from `T_user_frame`, infinite plane through the top face). Arm links (0-5) **always** blocked below it; nozzle (6) blocked unless the new **`allow_tcp_through_plate`** toggle (default OFF). `_meshes_clear_plane()` generalizes the old nozzle signed-distance test; the two `moving_geometry_*min_z` helpers are deleted; the curved tangent-plane check (S1.37) still layers on top. GUI checkbox "Reject poses below ground (z<0)" → "Allow TCP through build plate". `PRECOMPUTE_CACHE_VERSION` **3→4** (`reject_below_ground`→`allow_tcp_through_plate` in both metas). **Consequence (spec):** a precompute at the default plate pose rejects early (the arm reaches below the plate) — reposition the plate lower via the Build Plate controls, don't disable the check. The working plate poses / print procedure live in the supervisor's print-setup docs; `saved_position.json` holds the adopted plate pose — see `settled.md` **S1.40** |

### S1.40 current setup amendment

The adopted RX setup moves the plate to `[-570, -300, 0]` at working height,
loads the curved model, then loads the saved pose `[-570, -300, -200]` with
TCP-through enabled. Rebuild geodesics/order/orientation before RX precompute.
This X+30 setup solves all 3,175 RX waypoints. The earlier S1.38
`reject_below_ground` description is historical and superseded by S1.40.

## Directory Structure

```
/
├── assets/          FR5 link meshes, nozzle, build plate
├── docs/            DH table, joint limits, mesh convention, Polyscope API
├── wiki/            you are here
├── main.py          entry point, wires backend + UI together
├── gui_panel.py      UI panel — joint sliders, IK controls, I/O buttons
├── geometry_backend.py  backend — FK/IK, mesh rendering, TCP/trajectory, G-code
└── requirements.txt
```

## Architecture Model

Model-View split: `geometry_backend.py` (VisContent) owns geometry state
and math; `gui_panel.py` (UI_Menu) owns ImGui widgets and calls into the
backend; `main.py` wires them together and drives Polyscope's per-frame
callback. See `wiki/002_Architecture/INDEX.md` as subsystems get built out.

## Key Constraints

- Mesh vertices are baked in zero-pose world coordinates — always apply the
  Delta transform (`GLOSSARY.md` §2), never `T_0_i(q)` directly.
- Requires a physical GPU with OpenGL >= 3.3 — won't run over Remote
  Desktop or in most VMs.

## Recent Decisions

See `wiki/002_Architecture/settled.md` (S1.1–S1.42).
