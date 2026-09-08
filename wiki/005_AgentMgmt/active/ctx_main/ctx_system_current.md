---
status: active
scope: current-truth
last_verified_against_code: 2026-09-08
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
| Tool head + TCP tracking | done | **Real calibrated tool=1 offset since Stage 7.1** (`TCP_OFFSET_6D_MM_DEG`, flange-relative 6D pose with a genuine ~87°/−13°/61° rotation) — replaces S1.4's world point + borrowed rotation. **Since Stage 7.7**, `nozzle.obj` is visible again and **replaces** the "Tool Axis" stalk, which is deleted. It is rigidly re-aimed at load time (native CAD placement discarded — it targeted the retired `TCP.txt` point): tip pinned onto the TCP point, shaft axis laid along the **TCP frame's −Z**, the approach axis the curved pipeline commands, so the render shows the orientation IK is solving for. The tool floats clear of the flange as a result (98.33mm) — a known placeholder-asset artefact, accepted over a 36.32° orientation error. The tool is still the **TCP point alone** for collision; the nozzle's own shape/length is still uncalibrated, only its render orientation changed. See `wiki/003_Guides/TCP_Frame.md`, `settled.md` **S1.43**/**S1.51** |
| Analytical IK | done | Closed-form solver, multi-solution — see `settled.md` S1.4/S1.5 |
| TCP trajectory recording | done | See `wiki/003_Guides/TCP_Trajectory.md` |
| G-code toolpath preview | done | G0/G1-only parser, fixed `model.gcode` path, registered via an explicit "Load G-code preview" click — does **not** auto-reload on plate reposition (that auto-reload was removed, see `settled.md` S1.23); removable via a conditionally-shown "Clear G-code preview" button (`clear_gcode_preview()`) — see `wiki/003_Guides/Gcode_Toolpath.md`, `settled.md` S1.7/S1.8 |
| Build-plate position/orientation | done | Re-posable via Move/Reset/Save/Load Position buttons — see `wiki/003_Guides/BuildPlate_UserFrame.md`, `settled.md` S1.6 |
| Toolpath IK precompute | done | Chunked (`PRECOMPUTE_CHUNK_SIZE` waypoints/frame), pausable/resumable/cancellable, ground-clearance filtered **(no longer true — "ground clearance" went with `_branch_clears_ground` at Stage 7.4; the gate is now the nine-filter `_candidate_admissible` stack, specified in `docs/FR5_IK_Branch_Rejection.md`. See S1.46/S1.47)** — see `settled.md` S1.14/S1.15 |
| Toolpath precompute disk cache | done | `assets/models/planar/gcode/model.precompute.npz`, keyed on G-code SHA-256 + build-plate pose + version; loaded before re-solving on `run_toolpath_ik_precompute()` — see `settled.md` S1.21 |
| Toolpath playback | done | Progressive-reveal (beads start invisible, revealed as playback crosses them), render-throttled (`playback_render_stride`, `PLAYBACK_LOOKAHEAD_BEADS`) — see `settled.md` S1.16/S1.17-S1.20. **(Updated 2026-09-04, S1.55: the throttle is no longer the fixed `PLAYBACK_RENDER_STRIDE = 50` — it is derived per playback from the path's own joint motion, so planar still gets 50 while a curved layer gets ~6. A fixed waypoint stride gave curved layers only ~64 arm poses for a whole print.)** **(S1.56/S1.57, 2026-09-05: Run now re-inits rather than resumes when the loaded precompute isn't the active source's; `playback_waiting` and the frontier-chasing paths are gone as unreachable since 7.4.)** |
| Precompute/playback invalidation on plate move | done | In-session: `load_build_plate()` compares the new pose against the pose captured at precompute-start and invalidates both if it differs — see `settled.md` S1.22 |
| Curved-surface model loading (Stage 6.1) | done | 55 toolpath PLY files (reconstructing to 70 polylines) + 3 surface OBJ meshes placed above the plate via a "Load Curved Model" button; retains the placed geometry in world coordinates for 6.2 — see `wiki/003_Guides/CurvedModel_Loading.md`, `settled.md` S1.29/S1.30. **XY placement since 2026-09-03 (S1.48): `CURVED_MODEL_XY_OFFSET_MM` (`study_config.py`, default `(0,0)`) relative to the User Frame origin — NOT centred on the build-plate mesh, which was a bug that made the workpiece unreachable at the real frame (measured +105.6mm outward shift, past the arm's 922mm flange reach)** |
| Geodesic routing over print surfaces (Stage 6.2) | done | Two per-surface CSR graphs + hand-rolled `heapq` Dijkstra producing two 70×70 geodesic cost matrices, chunked one source per frame; in-memory only, no disk cache — see `wiki/003_Guides/CurvedModel_Geodesics.md`, `settled.md` S1.31 |
| Curved-surface print ordering (Stage 6.3) | done | Per-layer TSP-variant ordering (greedy nearest-endpoint seed + 2-opt over oriented pieces) off the 6.2 cost matrices, RX first then TX, via a synchronous "Build Print Order" button. Travel moves are the 6.2 geodesics hovered `CURVED_TRAVEL_HOVER_MM` outward along from-scratch surface normals (`compute_vertex_normals`, no scipy; oriented away from `Surface_Bot`), bookended with true endpoints. Printed pieces render as a **print-order gradient** (`Curved Order Feed`), travel in a distinct flat colour, and the RX/TX selector isolates one layer at a time (`apply_live_layer_visibility`, strict for now — S1.32 stack rule later). Measured RX 690mm vs 5157mm / TX 607mm vs 4848mm file-order travel — see `wiki/003_Guides/CurvedModel_PrintOrder.md`, `settled.md` S1.35 |
| Per-waypoint tool orientation (Stage 6.4) | done | `build_orientation_frames()` attaches a per-feed-point TCP orientation (nozzle perpendicular to the shell): **Z = outward surface normal**, in-plane axes pinned to a fixed world reference (not the path tangent) so the symmetric nozzle doesn't spin as the path meanders — "stable and straight". Supersedes S1.12's single-constant `R_target`. Stored per layer as `curved_orient_frames` (the array 6.5 feeds to IK) and drawn as a downsampled triad overlay (`Curved Orient Frames`, X/Y/Z = red/green/blue) via a "Build Orientation Frames" button. Compute + visualise only; IK wiring is 6.5 — see `wiki/003_Guides/CurvedModel_Orientation.md`, `settled.md` S1.36 |
| Curved IK precompute (Stage 6.5) | done | Reuses Stage 5's chunked precompute through one shared seam, `_begin_toolpath_precompute()`; `run_curved_toolpath_ik_precompute(layer, ...)` feeds it from `build_curved_toolpath_waypoints_world(layer)` (6.3's ordered feed pieces + travel hops, each carrying 6.4's per-waypoint orientation). `precompute_R_target` is now an `(N,3,3)` array (planar path broadcasts its one constant, unchanged behaviour). Nozzle clearance uses no obstacle mesh: each waypoint's own outward tangent plane is a supporting hyperplane for the convex mockup stack, checked against the **nozzle tip only** (`_nozzle_clears_plane`, `CURVED_TIP_CLEARANCE_TOLERANCE_MM` inward slack); world `z=0` is dropped for the curved case **(no longer true — `_nozzle_clears_plane` was deleted at Stage 7.2 as dead code, having been incapable of rejecting anything since 7.1. `CURVED_TIP_CLEARANCE_TOLERANCE_MM` is live again since 7.4 but for a different consumer: filter 8, the ARM LINKS against the layer's own print surface, not the nozzle tip against a tangent plane. The tool is excluded from that check by construction, so the nozzle remains unguarded. See S1.44/S1.46/S1.47 and `docs/FR5_IK_Branch_Rejection.md`)**. Per-layer disk caches (`curved_rx/tx.precompute.npz`, `PRECOMPUTE_CACHE_VERSION` bumped 1→2). `geometry_backend.py`-only; no GUI hookup yet (6.6) — see `wiki/003_Guides/CurvedModel_IKPrecompute.md`, `settled.md` S1.37 |
| Curved GUI wiring (Stage 6.6) | done | Wires 6.1-6.5 into the panel + adds curved playback. **One source-aware control set**, not a duplicate: a "Toolpath Source" selector sets `toolpath_source` (-1 planar / 0..N-1 layer) and the existing Run/Pause/Cancel/Reset controls dispatch via it (`run_active_toolpath_ik_precompute`, source-aware `*_toolpath_playback`); a layer-mixup guard force-cancels a paused run of a different source instead of silently resuming it. Per-layer curved bead playback (`_build_curved_beads`, fixed cross-section swept along each waypoint's surface normal) coexists across layers, so `apply_live_layer_visibility` now does the real S1.32 stack (`i <= layer`: TX shows the printed RX beneath). `clear_curved_model()` Load/Clear pair. **Toggleable z=0 ground check** (`reject_below_ground`, default ON, applies to both paths, layered on the tangent-plane check for curved; folded into the cache key, `PRECOMPUTE_CACHE_VERSION` 2→3). Top-down build panel + "Curved Model Properties" dropdown (`curved_model_summary()`) — see `settled.md` **S1.38** |
| Hide guide overlays during playback (Stage 6.7) | done | New `playback_active` flag, distinct from `playback_running` (survives Pause, cleared only by Reset). `run_toolpath_playback()` sets it and re-applies `apply_live_layer_visibility`, which now force-hides the order-feed/travel/orient curve networks + base toolpath curve while `playback_active` — surfaces and growing beads keep the `i <= layer` stack rule, so you can watch the object form. `reset_toolpath_playback()` clears it and restores the full guide view. Planar source is a no-op (the `G-code Print` mesh is the playback mesh). `gui_panel.py` untouched — see `settled.md` **S1.39** |
| Posed-plate collision (Stage 6.8) | done | Replaces the world-`z=0` proxy with a check against the **actual posed build plate** (`_plate_plane()` from `T_user_frame`, infinite plane through the top face). Arm links (0-5) **always** blocked below it; nozzle (6) blocked unless the new **`allow_tcp_through_plate`** toggle (default OFF). `_meshes_clear_plane()` generalizes the old nozzle signed-distance test; the two `moving_geometry_*min_z` helpers are deleted; the curved tangent-plane check (S1.37) still layers on top **(no longer true — Stage 7.2 narrowed this whole row to the planar path and deleted the tangent check; see S1.44)**. GUI checkbox "Reject poses below ground (z<0)" → "Allow TCP through build plate". `PRECOMPUTE_CACHE_VERSION` **3→4** (`reject_below_ground`→`allow_tcp_through_plate` in both metas). **Consequence (spec):** a precompute at the default plate pose rejects early (the arm reaches below the plate) — reposition the plate lower via the Build Plate controls, don't disable the check. The working plate poses / print procedure live in the supervisor's print-setup docs; `saved_position.json` holds the adopted plate pose **(no longer true — Stage 7.3 replaced it with the real calibrated User Frame, where this very check rejects every branch at waypoint 0; the infinite-plane model assumes the plate sits below the whole arm, and the real frame is 323.5mm above the base. "Reposition the plate lower" is not available for a measured pose. See S1.45)** — see `settled.md` **S1.40** |

| Real TCP offset (Stage 7.1) | done | `T_flange_to_tcp` is now `pose_to_matrix(*TCP_OFFSET_6D_MM_DEG)` — the real tool=1 calibration, superseding S1.4. New module-level `pose_to_matrix`/`matrix_to_pose` helpers (a refactor of a convention already inline, not new maths). `Nozzle` registered but `set_enabled(False)`; collision geometry split from render geometry (`moving_geometry_rest_verts` = 6 arm links + the TCP point); visual-only "Tool Axis" stalk at index 9, loop now `range(10)` **(the visibility and stalk half is no longer true — Stage 7.7 un-hid the mesh, deleted the stalk outright and put the loop back to `range(9)`; the collision split is untouched and still holds. See S1.51)**. `PRECOMPUTE_CACHE_VERSION` **4→5**. `USER_FRAME_ORIGIN_MM` moved `[-600,-300,0]` → `[-570,-300,-100]` to keep the planar path reachable. Identity check passes at 0.000000mm/0.0003°, unblocking 7.2 — see `settled.md` **S1.43** |
| Rejection criteria (Stage 7.2) | done | The exchange spec's **seven-row table implemented verbatim** as `validate_job()` — and it applies to **both** toolpath sources, not just curved: nothing in the rows is study- or surface-specific. In exchange, this project's own pose rejection is narrowed to **planar only** — `_nozzle_clears_plane` deleted (it had been incapable of rejecting anything since 7.1: 7,471 evaluations, zero rejections), and curved runs skip the plate check via the new `check_collision` flag **(both halves superseded at Stage 7.4 — the discriminator is now `filter_mode` ("planar"/"curved"), and curved runs no longer skip anything: filters 5-9 apply on both paths, reversing this narrowing. See S1.46/S1.47 and row 7.4 below)**. `build_export_segments()` pulled forward from the export sub-stage (7.4 then, **7.5** since the 7.4 insertion) — one shared builder, a maximal `is_feed_move` run, verified 35 segments == 35 print-order pieces. Solver + manual IK panel moved off `gui_panel.JOINT_LIMITS` onto **`PHYSICAL_JOINT_LIMITS`** (425 valid branches vs 207); sliders keep the practical range. `PRECOMPUTE_CACHE_VERSION` **5→6**. ⚠ **Curved solved paths fail row 5** (23/35 RX, 15/35 TX) **(no longer true — the cause was S1.36's per-waypoint roll, and Stage 7.4 made the roll a searched variable resolved by graph cost. Both layers now pass row 5 — worst in-segment step 29.93° RX / 29.85° TX of 30° — and `validate_job` returns ACCEPTED on both. See S1.46/S1.47)** — see `settled.md` **S1.44** |
| Real User Frame (Stage 7.3) | done | `assets/buildPlate/saved_position.json` now holds the **real calibrated User Frame** — `[649.456, 133.762, 322.778]` / `[-0.369, 0.329, -89.080]`, `user_index=1` from `docs/saved_coords_data_and_usage_EN.md` §1.1, replacing the 6.8 demo pose (retained as the file's inert `_legacy_stage6_8_demo_pose` record). **Data + docs only: no code changed and none was needed** — the rotation convention already matched exactly (`max |ΔT| = 0.0`), every consumer already handled a rotated pose, and the full-4x4 cache key invalidates old caches on its own (**no `PRECOMPUTE_CACHE_VERSION` bump**, stays 6). `USER_FRAME_ORIGIN_MM` deliberately **unchanged** at `[-570,-300,-100]` — startup/Reset is the chosen pose, the file is the measured one, applied opt-in per session. ⚠ **Neither toolpath runs at the real frame:** planar aborts at waypoint 0 on the *plate check* (the plane sits 323.5mm above the base and cuts the shoulder — a modelling limit, not a reach limit; IK gives 8 valid branches), and curved is genuinely unreachable (226/2,527 RX, 186/2,000 TX). Recorded as the finding §7.3 asked for, **not** reverted — see `settled.md` **S1.45** and `wiki/001_Inbox/2026-08-15_real_user_frame_reachability.md`. ⚠ **The frame is confirmed correct and this diagnosis is superseded (S1.46):** "genuinely unreachable" measures *one commanded orientation per waypoint* (≤8 IK candidates), not the arm's envelope, and the planar abort is a *shape* problem in S1.40's infinite plane. Stage 7.4 addresses both; until it runs, reachability at the real frame is **unknown in both directions** |
| Orientation search / re-shaped rejection (Stage 7.4) | done | A waypoint is judged by whether **any** admissible pose reaches it. `orientation_candidates()` searches **540 commanded frames** per waypoint (9 tool-axis directions — the normal plus an 8-azimuth ring at the 20° cap — × 60 roll slots), up to 8 IK branches each; S1.36's `argmin \|a·z\|` frame survives **unchanged** but is now only the cone axis and the exported surface normal, never the commanded pose. Nine candidate filters (`_candidate_admissible`), including this project's **first mesh-vs-mesh collision** (surface **1.0mm** from `CURVED_TIP_CLEARANCE_TOLERANCE_MM`, live again; self **5.0mm**, multi-proxy OBB), on **both** paths — reversing 7.2's curved-only narrowing. S1.40's infinite plate plane and `allow_tcp_through_plate` **deleted**, replaced by a finite footprint (20mm) + slab (3.0mm). Selection is `dijkstra_candidate_path()` over a `(waypoint, candidate)` DAG — the same algorithm as S1.31 but a **layered relaxation, not a heapq frontier** (a heap would walk ~5×10¹⁰ edges). Edge step limit aliases `JOINT_STEP_MAX_DEG` = **30°**, and applies **feed-to-feed only**. `PRECOMPUTE_CACHE_VERSION` **6→7**, schema now carries waypoint positions/is_feed/normals (closing 7.5's cache gap early). ✅ **Planar fixed: 181,375/181,375 at the real User Frame** (was: abort at waypoint 0), 20,350 segments, `validate_job` ACCEPTED. At the time, curved improved 8.5× but was still not plannable at the real frame
— 1,922/2,527 RX and 1,410/2,000 TX admissible (was 226/186), yet ~24% of feed
points had no IK solution at *any* of the 540 orientations. A control run at the
default plate pose gave 100%/100%, isolating the cause to **placement**, not the
filters and not the arm. See `settled.md` **S1.47**. ✅ **Fixed the same day —
see the Stage 6.1 row above and S1.48.** With the corrected placement, both
curved layers now solve completely too: RX 3,175/3,175, TX 2,688/2,688, both
`validate_job` ACCEPTED |
| Job export / GUI (Stage 7.5–7.6) | done | `VisContent.export_active_job()` + `step_export_job()` write `job.json` + `segment_N_solution.json`/`toolpath_TN.ply` per segment + `surface.obj` (curved only) to `assets/export/<job_name>/`; an "Export IK Job" button (`gui_panel.py`) self-checks via `validate_job()` then writes, gated on a truthy (possibly partial) `precompute_joint_path`. Folded 7.6 into the same pass as 7.5 per direct user request. **Exported and round-trip verified on both sources** — planar's `validate_job` result was already established at 7.4 (181,375/181,375, not re-exported to avoid the ~156s solve); curved RX (35 segs/2,527 pts) and TX (35 segs/2,000 pts) were both exported and round-trip checked, loaded straight from `curved_rx/tx.precompute.npz` with no rebuild. ⚠ Review found and fixed a toolpath_source/precompute mismatch bug: switching the Toolpath Source radio doesn't clear `precompute_joint_path`, so exporting after a switch (without re-running precompute) would silently write the *previous* source's data under the *new* source's name/`surface.obj` — fixed with the same `precompute_cache_path` guard the playback init functions use. See `settled.md` **S1.49**. **7.5 follow-up (S1.50, 2026-09-04):** the write itself is now chunked across frames (`step_export_job()`, `EXPORT_CHUNK_SIZE` points/call; `write_job_export()` deleted) with a progress bar and a "Cancel Export" button, since the planar job's 181,375 points froze the GUI for the whole write. Chunking reopened the toolpath-source race for the write's whole duration (not just export-start) and left `export_running` stuck `True` forever on a write failure — both found in review and fixed the same pass. **7.5 follow-up (S1.54, 2026-09-04):** `_finish_export_job()` also writes `assets/export/<YYYYMMDD>-<name>.zip` (the job folder as one top-level entry), named from a free-text "Export Name" GUI field sanitized into `export_zip_name` at export-start. The zip has its **own** try/except: a zip failure is reported as a trailing `(zip failed: ...)` note on an otherwise-successful export, never as a failed export, since `job_dir` is already complete on disk |
| Nozzle render pose (Stage 7.7) | done | The `Nozzle` mesh is **visible again and replaces the "Tool Axis" stalk**, which is deleted outright along with `TOOL_AXIS_COLOR`/`TOOL_AXIS_RADIUS_MM`; `apply_delta_transform`'s loop is back to `range(9)`. The mesh is **re-aimed once at load time**, not rendered as CAD-exported (its native pose targets the retired `TCP.txt` point, 310.97mm from the real TCP): tip pinned onto `tcp_point`, and the **shaft's** long axis — via the new `_nozzle_shaft_mask()` + the existing `_obb_from_points()` — laid along the **TCP frame's −Z**, the approach axis every curved `R_target` is built around (S1.36). The shaft, not the whole mesh: the mounting bracket skews a whole-mesh PCA **6.59°**, versus **0.0000°** fitting the shaft parts alone. Accepted cost: the tool floats **98.33mm** clear of the flange, a placeholder-asset artefact (163.47mm vs tool=1's 196.91mm, compound mount angle) taken deliberately over a **36.32°** orientation error from the flange→TCP chord. **Collision geometry untouched** — the tool is still the TCP point alone, since only the render pose was corrected. **Also this stage:** `run_*_toolpath_ik_precompute()` gains a third "already complete" mode (S1.52) — re-clicking Run after a finished solve used to raise `IndexError` inside the render callback. ⚠ Review found and fixed two degenerate-case defects in the alignment before it landed: `-I` as the antiparallel rotation (det −1 — it would point-invert the mesh) and an all-False shaft mask falling through to NaN vertices; both fire only on a *swapped* asset, which is the open item S1.43 records. See `wiki/003_Guides/TCP_Frame.md`, `settled.md` **S1.51**/**S1.52** |

### S1.40 current setup amendment

The adopted setup moves the plate to `[-570, -300, 0]` at working height,
loads the curved model, then loads the saved pose `[-570, -300, -200]`.
Rebuild geodesics/order/orientation before precompute. This solved all
**3,175 RX** and **2,688 TX** waypoints — both with
`allow_tcp_through_plate` **False** (nozzle blocked), per the cache metadata;
an earlier note here said "TCP-through enabled", which the artifacts
contradict. Full procedure and its rationale:
`wiki/003_Guides/CurvedModel_PrintSetup.md`. ⚠ A completed precompute is
*not* a collision-free guarantee — the arm passes through the mockup on TX;
nothing checks arm-vs-mockup. The earlier S1.38 `reject_below_ground`
description is historical and superseded by S1.40.

⚠ **Stale since Stage 7.1.** S1.43 moved the TCP 310.97mm, so every solved
branch changes and the RX/TX counts above are pre-7.1 evidence. The curved path
has not been re-run. Re-validating is deliberately deferred: 7.2 removes the
curved clearance checks entirely and 7.3 replaces `saved_position.json` with the
real User Frame, so the setup is likely to change again first. The **planar**
path *was* re-validated — 181,375/181,375, at the new default plate pose.

⚠ **Dead as a procedure since Stage 7.3 (S1.45).** Steps 1 and 3 above name
`[-570, -300, 0]` / `[-570, -300, -200]`, and `saved_position.json` no longer
contains the latter — "Load Saved Position" now jumps to the real User Frame in
the opposite quadrant, 322mm up, yawed ~89°. Measured there: **226/2,527 RX** and
**186/2,000 TX** feed points reachable (91% unreachable, pure geometry), and the
planar path aborts at waypoint 0 because the plate plane sits 323.5mm above the
base and cuts through the shoulder. The clearance trick this amendment describes
has no meaning at the real frame. The curved pipeline was **deliberately not
re-run** — placement has to be answered first, and that rebuild is shared with
the pending S1.36 reference-axis fix. To reproduce the old setup you must type
the poses in by hand. See
`wiki/001_Inbox/2026-08-15_real_user_frame_reachability.md`.

✅ **This whole amendment is closed as of 2026-09-03 — do not act on any of it.**
Every ⚠ above describes a problem that Stage 7.4 and S1.48 resolved, and the
"deliberately not re-run / placement has to be answered first" sentence directly
above is the most misleading line in this file if read as current:

- **Placement was answered**, by measurement. The workpiece had been centred on
  the stand-in build-plate mesh, ~105.6mm outward of the User Frame origin and
  past the arm's 922mm flange reach. `CURVED_MODEL_XY_OFFSET_MM` (default
  `(0,0)`, `study_config.py`) now centres it on the frame — see the Stage 6.1 row
  above and **S1.48**.
- **The curved pipeline WAS re-run**, at the real calibrated User Frame, and
  solves completely: **RX 3,175/3,175, TX 2,688/2,688**, both `validate_job`
  **ACCEPTED**, both exported. The 226/2,527 and 186/2,000 counts above measured
  *one commanded orientation per waypoint*, which 7.4 replaced with a 540-frame
  search — they were never a statement about the arm's envelope.
- **The S1.36 reference-axis fix is not pending.** 7.4 subsumed it: the roll is a
  searched variable, and `_orientation_frames_for_points()` now supplies only the
  search cone's axis and the exported surface normal.
- **The plate model is no longer an infinite plane**, so "the plate plane cuts
  through the shoulder" no longer applies — filters 6 and 7 are a finite
  footprint (20mm margin) plus a bounding slab (3.0mm), and planar solves
  181,375/181,375 at the real frame.

There is no manual pose-typing step any more: since v1.0 the app starts at the
saved calibrated User Frame (**S1.58**). Current procedure:
`wiki/003_Guides/CurvedModel_PrintSetup.md`. Filters and tolerances:
`docs/FR5_IK_Branch_Rejection.md`.

## Directory Structure

```
/
├── assets/          FR5 link meshes, nozzle, build plate, curved-model PLY/OBJ
├── docs/            DH table, joint limits, mesh convention, Polyscope API,
│                    supervisor calibration data (saved_coords_*)
├── examples/        per-study config the generic engine reads (S1.33) —
│                    curved_surface_printing/study_config.py + the supervisor's
│                    external_ik_exchange_spec_EN.md
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

See `wiki/002_Architecture/settled.md` (S1.1–S1.69).

## Changed at v1.0 (2026-09-06) — repo-wide review before the curved-printing tag

Five changes an agent will trip over if it assumes the pre-v1.0 shape:

- **The build plate no longer starts at `USER_FRAME_ORIGIN_MM`** (S1.58). It
  loads `assets/buildPlate/saved_position.json` (the real calibrated User Frame)
  when readable, falling back to the constant otherwise, and reports which
  through `startup_plate_status`. This supersedes S1.6's "never automatically at
  startup" clause. Reason: the shipped curved caches are keyed on the plate pose
  and were solved at the saved frame, so booting at the constant meant a ~30
  min/layer re-solve on every first run. **`load_build_plate()` now also retains
  `build_plate_pose`**, which `UI_Menu` seeds its input fields from.
- **The precompute cache key now includes the tuned solver constants** (S1.59),
  via `_solver_cache_fields()` — `TCP_OFFSET_6D_MM_DEG`, `PHYSICAL_JOINT_LIMITS`,
  the `FILTER_*` set, the `EDGE_*` costs, plus `tip_clearance` on curved keys.
  Retuning any of them now correctly misses. `PRECOMPUTE_CACHE_VERSION` stays
  **7**: the existing caches were migrated in place, not invalidated.
- **The study config is selected by `FR5_STUDY_CONFIG`** (S1.60), not by editing
  the import in `geometry_backend.py`. Required names are listed in
  `_STUDY_CONFIG_NAMES`. Default behaviour is unchanged.
- **Asset paths are absolute**, anchored to the source directory via
  `_asset_path()` (S1.61) — do not assume CWD-relative paths any more.
- **`two_opt` scores by `_reverse_delta`**, not a full tour re-sum (S1.62). It
  provably returns the same orders (280/280 on random cost matrices), which is
  why the shipped caches still hit.

Two dead functions are now marked ⚠ LEGACY in their docstrings rather than
silently uncalled: `dijkstra_candidate_path()` (the live path is
`_relax_candidate_layer()` + `_finish_candidate_search()`) and
`_meshes_clear_plane()` (filter 5 uses `_plate_plane()` only). There are **no
tests in this repo** — an earlier docstring claiming `dijkstra_candidate_path`
was "unit-tested" was wrong and has been corrected.

New guide: `wiki/003_Guides/CurvedModel_AdaptingYourOwnJob.md`, the reference for
pointing the curved feature at a different part.

A second sweep then found defects the first pass had missed (S1.64–S1.68) — most
of them state-machine rather than maths:

- **Two crashes that killed the Polyscope window**, both reachable by ordinary
  clicks: *Cancel Geodesics → Run Toolpath* (the playback initialiser gated on
  the precompute but dereferenced the print order the cancel had nulled), and
  *Run Precompute on a motion-free G-code file* (`np.array([])` is 1-D). Both now
  decline with a status. **The lesson to carry: a function that subscripts
  retained state must validate that state itself** — `precompute_cache_path` says
  *who solved the path*, not whether the geometry it came from still exists.
- **Selecting Planar hid the entire curved workpiece** — `visible = (i <= layer)`
  is False for every layer at `layer == -1`, and nothing restored it. Now an
  explicit early return (S1.65).
- **Cancel Precompute destroyed a G-code preview it never owned**, because
  `precompute_cache_path is None` meant both "planar run" and "no run at all"
  (S1.68).
- **A plate move left the curved model usable at its old pose** (S1.66).
- **Export validation now runs in the first `step_export_job()`**, not the click,
  so its pause is visible — measured 0.12s curved, 6.32s planar (S1.67). Validate
  still strictly precedes the destructive prune.

⚠ One correction to the earlier v1.0 pass recorded above: the filter-9 comment in
`_filter_context` was "fixed" from Robot1..Robot4 to Robot0..Robot3, and that was
**wrong** — `moving_geometry_rest_verts` is `rest_verts[:6] + [tcp_point]`, so
index 0 is **Robot1** and `range(4)` is Robot1..Robot4. Reverted, with the index
convention spelled out in place.

`tutorials/` **is published** (2026-09-08, **S1.70**) — the `.gitignore` rule is
gone, so the many docstring/wiki citations to `tutorials/Stage*_README.md` can be
followed and every cited sub-stage number resolves to a section. One caveat when
cross-reading: the stage READMEs are a clean *reconstruction*, so a correction
found in a later stage is taught in the stage that needs it, and a sub-stage may
describe the corrected approach rather than the one originally built there. For
the chronology use `settled.md`. See `wiki/INDEX.md`.
