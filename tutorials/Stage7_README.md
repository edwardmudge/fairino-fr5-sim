# FR5 Stage 7 — Real Calibration & Export to the External IK Exchange Format

Stage 6 proved out curved-surface printing using this project's own analytical
solver — but against stand-ins throughout: a chosen build-plate pose, a stand-in
tool, and one commanded orientation per waypoint. The supervisor's docs mostly
confirm this project already has things right — DH table and zero-pose FK match
bit-for-bit, joint limits match. The role they put this project in is the
**Collaborator** side of the exchange spec: *"Collaborators independently provide
the complete print job package; we ingest and execute it directly."* The solver
already works. What's missing is the real-world numbers that make its output
*physically* correct, the pose search and collision filtering Stage 6 deferred,
and a way to **export** a solved path in the format the supervisor's team expects.

## What You Get

- `docs/saved_coords_data_and_usage_EN.md` — real User Frame + TCP-offset
  numbers, plus DH/joint-limit references (already confirmed to match this
  project's own docs — no FK/DH rework needed).
- `examples/curved_surface_printing/external_ik_exchange_spec_EN.md` — the
  `job.json` + `segment_N_solution.json` + `toolpath_T*.ply` package format, and
  the Rejection Criteria table this project's export must satisfy.
- A **working reference implementation** of this same task from another project:
  its 480-candidate orientation search and its nine candidate filters. The
  source for 7.4 — what this project ends up with is specified in
  [`docs/FR5_IK_Branch_Rejection.md`](../docs/FR5_IK_Branch_Rejection.md),
  including the places the reference's values had to be changed.
- Decisions on record — `settled.md` **S1.43** (real tool=1 TCP offset),
  **S1.44** (Rejection Criteria, physical joint limits), **S1.45** (real User
  Frame), **S1.46**/**S1.47** (orientation search, re-shaped rejection),
  **S1.49** (job export + output location), **S1.51**/**S1.52** (nozzle render
  pose, precompute guard) and **S1.54** (export archive).
- Design notes and per-sub-stage specs —
  [`2026-07-22_stage7_calibration_and_external_ik.md`](../wiki/001_Inbox/2026-07-22_stage7_calibration_and_external_ik.md).

As in Stage 6, this roadmap stays plan-level: the measured numbers and the
gotchas live in `settled.md` and the inbox notes linked per sub-stage.

## What You Build

Four focused pieces, not a big import/validation system:

1. **Real calibration, actually applied.** The TCP offset (7.1) and the User
   Frame (7.3) were both stand-ins; both become the measured values.
2. **The spec's Rejection Criteria, replacing ours.** The seven-row table
   becomes the definition of a valid job, implemented verbatim (7.2).
3. **An orientation search and a properly-shaped filter set** (7.4), so a
   waypoint is judged by whether *any* admissible tool pose reaches it rather
   than by one commanded frame — and so the collision hole Stage 6.8 left open
   is finally closed.
4. **A job exporter** (7.5). Given a solved `toolpath_source`, self-check it
   against those criteria, then write `job.json` + one
   `segment_N_solution.json`/`toolpath_T*.ply` pair per segment.

## Still Open

Two questions are genuinely unresolved at the end of Stage 7. Everything else
this stage raised is answered in the sub-stage that answered it.

- **The nozzle has no collision body.** 7.4's filters cover the arm links, and
  filter 8 caught a real pose with an arm link 0.71 mm inside the TX surface.
  The tool itself is one TCP point, excluded by construction, so nothing would
  catch the nozzle body fouling the mockup. Closing this needs a corrected tool
  asset, not another filter — see 7.7 for why the current one can't carry
  collision geometry. `settled.md` S1.46, S1.47.
- **What physically sits at the User Frame?** The Bambu Lab plate mesh was a
  stand-in chosen at the old demo pose. The filters in 7.4 that model the plate
  (6 and 7) are load-bearing and still rest on that asset. Ask before engineering
  further around it.

## Roadmap

### 7.1 — Real TCP Offset ✅ done

**Goal:** Give the print head its real flange-relative offset instead of a
borrowed-rotation hack. Leads the stage because 7.2's identity check is only
meaningful once this is in.

1. `T_flange_to_tcp = pose_to_matrix(*TCP_OFFSET_6D_MM_DEG)`, the real tool=1
   calibration — **done**, superseding S1.4's derived rotation.
2. Retire `TCP.txt` and represent the tool by the **TCP point alone** —
   **done**. The `nozzle.obj` asset is not the head tool=1 was calibrated
   against: it is 163.47 mm against tool=1's 196.91 mm, and mounted at a
   compound angle. The asset is at fault, not the calibration. It stays on
   screen as a visual reference and is excluded from collision geometry; its
   render pose is corrected in 7.7.
3. `PRECOMPUTE_CACHE_VERSION` 4 → 5 — **done**; every cached path was solved for
   a different flange→TCP transform.

**Note:** the offset moves the TCP 310.97 mm, far enough to break the planar
path at the old plate pose, so `USER_FRAME_ORIGIN_MM` moves to
`[-570, -300, -100]`. Expect every Stage 6-era solve to be invalidated — that is
the point of the version bump, not a symptom of a mistake.

**Verify:** the flange→TCP identity check passes at **0.000000 mm / 0.0003°**,
unblocking 7.2; every existing cache is rejected on load at v5.

Full record: `settled.md` **S1.43**, spec: inbox note §7.1.

### 7.2 — Rejection Criteria ✅ done

**Goal:** One definition of "rejected", taken from the exchange spec, rather than
this project's own ad-hoc notion of a bad pose.

1. **Split the joint limits first**, before anything reads them — **done**. New
   `PHYSICAL_JOINT_LIMITS` for both solver call sites; `gui_panel.JOINT_LIMITS`
   keeps the sliders. The limits the hardware enforces and the limits a slider
   exposes are two different things and should never have been one constant.
2. **Give the precompute seam a collision switch** — **done**.
   `_begin_toolpath_precompute` takes a `check_collision` boolean, so 7.4 has
   somewhere to hang the filter set without another signature change.
3. **Pull segments forward from job export** — **done**, rows 5 and 6 need them.
   A maximal `is_feed` run *is* the spec's continuous extrusion line, so it is
   **one shared builder** (`build_export_segments()`, `ExportSegment`), not two.
4. **Then the table** — **done**, `validate_job()` + `format_validation()`, plus
   an in-house **row 0, "job is non-empty"** ahead of the spec's seven. The spec
   assumes a job exists; a validator that accepts nothing at all is worse than
   useless when a cache miss can hand you an empty path.
5. **`PRECOMPUTE_CACHE_VERSION` 5 → 6** — **done**; the joint-limit switch
   changes which solves are admissible.

**Verify (headless):** seven fixtures, each failing its own row with all others
passing; row 7 WARNs without rejecting; an empty job is REJECTED by row 0; 35
segments == 35 print-order pieces on both curved layers; all stale caches
rejected at v6.

Full record: `settled.md` **S1.44**, spec: inbox note §7.2.

### 7.3 — Real User Frame ✅ done

**Goal:** Make the build plate's saved pose the real calibrated User Frame, not
a chosen demo pose.

1. `assets/buildPlate/saved_position.json` overwritten with `user_index=1` —
   `[649.456, 133.762, 322.778]` / `[-0.369, 0.329, -89.080]` — **done**. The old
   pose stays as the inert `_legacy_stage6_8_demo_pose` record.
2. **No code changed, and none was needed** — **done**. The rotation convention
   already matched at `max |ΔT| = 0.0`, and the 4×4 pose is already part of the
   cache key, so it invalidates old caches by itself and
   `PRECOMPUTE_CACHE_VERSION` stays **6**. Verify the convention rather than
   assuming it; this is the cheapest possible confirmation that Stage 5's
   `T_user_frame` was built right.
3. `USER_FRAME_ORIGIN_MM` **deliberately unchanged** at `[-570, -300, -100]` —
   **done**. Startup/Reset is the *chosen* pose, the file is the *measured* one,
   opt-in per session via Load Saved Position.

**What this exposes.** At the real frame the plate sits 323.5 mm above the base
and the workpiece sits correspondingly far out along the arm. With Stage 6.4's
single commanded orientation per waypoint, a waypoint gets at most 8 IK
candidates and is declared unreachable when none survives — and measured that
way, most curved feed points fail. Read that as a fact about *one commanded
pose*, not about the arm's envelope: the tool axis is pinned exactly on the
surface normal and the roll is pinned by a world-axis rule, so the solver is
being asked for one pose out of a continuum it is free to choose from. Recording
the measurement without reverting anything is the right move here; 7.4 is what
turns it around.

**Verify:** Load Saved Position puts the plate and its User Frame triad on the
calibrated pose; `max |ΔT|` between this project's matrix and the reference's is
**0.0**, confirming the rotation convention; every v6 cache built at the old pose
is rejected on the pose key alone, with no version bump.

Full record: `settled.md` **S1.45**, measurements:
[`2026-08-15_real_user_frame_reachability.md`](../wiki/001_Inbox/2026-08-15_real_user_frame_reachability.md).

### 7.4 — Orientation Search & Re-shaped Rejection ✅ done

**Goal:** Judge a waypoint by whether *any* admissible tool pose reaches it, and
reject on geometry that is actually shaped like the obstacle. Adapts a working
reference implementation of this task from another project; the criteria this
project settles on are specified in
[`docs/FR5_IK_Branch_Rejection.md`](../docs/FR5_IK_Branch_Rejection.md).

1. **Widen the commanded pose** — **done**. The tool axis need only be
   perpendicular to the surface **within 20°**, per the supervisor. The roll
   about the tool axis stays entirely free — the nozzle is rotationally
   symmetric, which is S1.36's own reasoning for pinning it in the first place.
2. **Search the orientation set** — **done**. A **20° tilt cone** about the
   surface normal × **60 roll slots** (6°, wrapping) × **8 IK branches** per
   waypoint. The supervisor phrased this as "all combinations of Rx, Ry and Rz";
   cone + roll is the same set, parameterised so the 20° cap lands only on the
   DOF it should constrain and the free DOF is swept in full.
3. **Adopt the reference's candidate filters**, cheapest first so expensive FK
   and collision tests run last — **done**, see the table below. Filters 6–9 are
   what close Stage 6.8's collision hole.
4. **Select the path globally, not greedily** — **done**. Candidates form a
   layered DAG (`(waypoint, candidate)` nodes) searched by Dijkstra, replacing a
   per-waypoint "rank branches against the previous pose, take the first that
   clears". Continuity becomes a **cost**, not a tie-break — which is what
   resolves the free-roll discontinuity 6.4 flagged, without patching 6.4's rule.
   A greedy ranking cannot recover from a dead end, and cannot undo a
   discontinuity in the commanded frame itself.
5. **`PRECOMPUTE_CACHE_VERSION` 6 → 7** — **done**; the candidate schema changes.

**Filters adopted, with this project's values:**

| Reference filter | Status here | Value |
|---|---|---|
| 1 Joint limits | already have | `PHYSICAL_JOINT_LIMITS` (S1.44) |
| 2 J5 non-negative | adopt — subsumes spec row 7's \|J5\|<2° WARN | ≥ 2° |
| 3 J4 minimum | opt-in, default off | −60° |
| 4 Upper branch (elbow above shoulder-wrist chord) | adopt | 2.0mm |
| 5 Elbow above plate plane | adopt | 1.0mm |
| 6 Under-plate footprint | adopt | 20mm margin |
| 7 Plate volume slab | adopt | 3.0mm |
| 8 Surface mesh collision | adopt — first mesh-vs-mesh in this project | 2.0mm |
| 9 Robot/tool self-collision | adopt | 5.0mm |
| Edge: max adjacent joint step | **adopt, retuned** | **30°, not the reference's 35°** |
| Edge: branch-change penalty | adopt | 150 per IK-ordinal change, 2.0 roll-quadratic |

Filters 6 and 7 together are the finite plate model Stage 6.8 argued for: a real
bed has a footprint and a thickness, and the arm reaches around it.

**Note — do not copy the 35°.** The reference uses
`max_adjacent_joint_step_deg = 35`, but the exchange spec's row 5 rejects steps
**> 30°**. Carrying 35 across would build a planner whose own edge filter admits
jobs the receiving side rejects.

**Note — this is not "less strict" overall.** Only the *commanded pose* is
loosened. Filters 4–9 are all additions: the reference is stricter than this
project everywhere except the shape of the plate model, where an infinite plane
becomes a finite footprint plus a bounding slab.

**Verify — all done:**
- ✅ planar solves all **181,375 / 181,375** waypoints (156 s) into 20,350
  segments, `validate_job` ACCEPTED
- ✅ curved solves completely at the real User Frame — RX **3,175 / 3,175**, TX
  **2,688 / 2,688**, `validate_job` ACCEPTED on both
- ✅ `dijkstra_candidate_path()` matches exhaustive search on 40 random DAGs, and
  takes the non-greedy branch in a hand-built dead-end case
- ✅ filter 8 rejects a real arm-through-mockup pose (TX waypoint 518, arm link
  **0.71mm** from the print surface) that an unfiltered curved path accepted
- ✅ row 5 passes on planar (worst in-segment step **4.43°** of 30°); row 7 never
  WARNs, since filter 2's J5 ≥ 2° subsumes it
- ✅ row 5 passes on both curved layers: worst in-segment step **29.93° RX /
  29.85° TX**. (Large steps of 81.74°/275.33° do occur, but only between two
  *travel* waypoints, never within a feed segment — the edge filter is scoped to
  feed-to-feed edges by design, and travel waypoints are dropped from export
  regardless.)
- ✅ v6 caches rejected at v7

Specification: [`docs/FR5_IK_Branch_Rejection.md`](../docs/FR5_IK_Branch_Rejection.md)
— every filter, its tolerance, and where it differs from the reference.
Full record: `settled.md` **S1.46** (the decision) and **S1.47** (as built, with
the measurements and the three places the reference's values had to be
corrected).

### 7.5 — Job Export ✅ done

**Goal:** Write the actual `job.json` / segment / ply files, and never write one
that the receiving side would reject.

7.2 already built `build_export_segments()`, `validate_job()` and
`ExportSegment` — 7.5 is their first caller.

1. `write_job_export(vis, segments, job_dir)` writes one `job.json` + one
   `segment_N_solution.json`/`toolpath_TN.ply` pair per segment — **done**, plus
   `surface.obj` for curved sources. Planar has no mesh asset for its plate
   (7.4's model is a footprint plus a slab, not a mesh), a known gap in spec
   coverage for that source rather than an oversight.
2. `VisContent.export_active_job()` is the self-check-then-write glue —
   **done**: `build_export_segments()` + `validate_job()` first, write only on
   ACCEPTED. Output at `assets/export/<job_name>/`, where `job_name` is
   `"planar"` for G-code or the curved layer name (`"RX"`/`"TX"`).
3. **Guard the source/data mismatch** — **done**. `export_active_job()` names the
   folder and picks `surface.obj` from `self.toolpath_source`, but reads the
   already-solved `precompute_joint_path` — and switching the "Toolpath Source"
   radio does not itself clear or re-check that path. Without a guard, a solved
   RX followed by a radio switch to TX would silently export RX's data
   mislabeled as TX. Reuse the same `precompute_cache_path` guard the playback
   init functions already use.
4. **Also write a dated archive** — **done**,
   `assets/export/<YYYYMMDD>-<name>.zip`, holding the job folder as one
   top-level entry. A single artifact is easier to hand off than the folder's
   file count (72 for a curved layer, ~40,000 for planar). The name comes from a
   free-text "Export Name" field in the GUI, sanitized and captured at
   export-start, falling back to the job folder's own name when left blank. Zip
   failures are isolated from the folder write, so they can never report a
   complete export as failed.

Nothing in `build_export_segments()`/`validate_job()`/`write_job_export()` is
source-specific — all three run over `ExportSegment`, generic across planar and
curved. The exported job's User Frame is a provenance fact stated in the
`job.json` header, and it is now the real calibrated one either way.

**Verify:** both paths export and validate clean at the real User Frame —
planar's 20,350 segments ACCEPTED, RX and TX both ACCEPTED and both round-trip
checked loading straight from `curved_rx/tx.precompute.npz` with no rebuild; a
cache hit exports the same 28 segments as a fresh solve, with identical
positions and normals; switching the toolpath source without re-running
precompute refuses rather than exporting mislabeled data.

Full record: `settled.md` **S1.49** and **S1.54**. Spec: inbox note §7.4
(pre-renumber — see the note at the top of that file).

### 7.6 — GUI Wiring ✅ done

**Goal:** An "Export IK Job" control in the panel.

Folded into the same pass as 7.5 rather than done separately, so the exporter is
never left uncallable from the app. An "Export IK Job" button sits in I/O
Operations beside Run/Reset Toolpath, calling `export_active_job()`. The gating
condition is a truthy (possibly partial) `precompute_joint_path` and not
`precompute_running` — **not** "complete", matching how the existing
precompute/playback controls gate and `build_export_segments()`'s own documented
"a partial precompute exports its solved prefix" behaviour. The status line
below it shows `format_validation()`'s full 8-row table on REJECT, but collapses
to one line (`"Passed all checks, exported N segment(s) to <path>"`) on ACCEPTED
— once every row has passed, the table adds nothing.

**Verify:** the button is greyed with no precompute and live after a partial one;
a rejected job prints all 8 rows with the failing one identifiable; an accepted
job collapses to the one-line summary and the named path contains the files.

Full record: `settled.md` **S1.49**. Spec: inbox note §7.5 (pre-renumber).

### 7.7 — Nozzle Render Pose ✅ done

**Goal:** Make the visible tool show the orientation the solver is actually
commanding. `nozzle.obj` was modelled around the `TCP.txt` point that 7.1
retired, so as-authored it points at empty space — the arm looks wrong even when
the solve is right.

1. Re-aim the mesh at load time instead of rendering it as CAD-exported —
   **done**. Its own tip is pinned onto `tcp_point`, and its **shaft's** long
   axis (new `_nozzle_shaft_mask()` + the existing `_obb_from_points()`) is
   rotated onto the **TCP frame's −Z** — the approach axis every curved
   `R_target` is built around (S1.36). Roll is left free: the nozzle is axially
   symmetric.
2. Collision geometry is untouched — **done**. The tool is still the TCP point
   alone (7.1); only the render pose changed. `apply_delta_transform`'s loop
   covers `range(9)`.
3. Fix the precompute re-entry crash — **done**, S1.52. Clicking "Run
   Precompute" on an already-finished solve raised `IndexError` inside the render
   callback; it now reports `"Already solved N waypoint(s)"`.

**Note — the shaft, not the whole mesh.** The mounting bracket drags a
whole-mesh PCA **6.59°** off the shaft's true axis, which would render the tool
that far off the commanded approach axis. The flange→TCP chord was tried first
and looks better attached, but sits **36.32°** off the tool's real axis; it was
rejected for exactly that reason. The accepted cost is that the tool floats
**98.33 mm** clear of the flange — an artefact of a placeholder asset that is
the wrong length and mounted at a compound angle (7.1), not something re-aiming
can fix. This is the same asset problem that keeps the nozzle out of collision
geometry.

**Verify:** shaft axis vs the TCP frame's −Z at **0.0000°** and tip-to-`tcp_point`
at **0.0000 mm**, measured against the *current* frame at six joint
configurations — the load-time alignment is exact at every pose, not just the
zero pose it is computed at. On screen the nozzle body is collinear with the TCP
triad's blue axis, foreshortening to almost nothing viewed down the tool.

Full record: `settled.md` **S1.51** (render pose) and **S1.52** (the precompute
guard).

---

## What's Next: Beyond Stage 7

Driving a real FR5. A job exported here, against the real User Frame and real
TCP, is meant to run on the physical arm without modification once that handoff
happens. The two items under **Still Open** above are what to settle first.
