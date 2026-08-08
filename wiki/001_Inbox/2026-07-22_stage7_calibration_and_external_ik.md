---
status: inbox
stage: 7.1-7.5
scope: geometry_backend.py, gui_panel.py, assets/buildPlate/saved_position.json, examples/curved_surface_printing/study_config.py
---

# Stage 7 — Real calibration + export to the external IK exchange format

## Why

Two supervisor docs landed (`docs/saved_coords_data_and_usage_EN.md`,
`examples/curved_surface_printing/external_ik_exchange_spec_EN.md`) once
the in-house solver was validated. First read of this treated it as an *import* feature (ingest
someone else's job); re-reading with the user corrected that: this
project is the **Collaborator** in the exchange spec's own language —
"Collaborators independently provide the complete 'print job' package; we
ingest and execute it directly." The solver already works. What's
missing is narrower than first drafted:

- **The build plate isn't posed at the real User Frame.**
  `assets/buildPlate/saved_position.json` currently holds
  `[-570, -300, -200]`, zero rotation — the Stage 6.8 "adopted RX setup"
  working pose, chosen purely so the arm could reach the curved-model
  waypoints on screen (see
  `wiki/001_Inbox/2026-07-22_stage6.8_posed_plate_collision.md`, "RX
  setup"). The real calibrated User Frame is
  `[649.456, 133.762, 322.778, -0.369, 0.329, -89.080]`
  (`docs/saved_coords_data_and_usage_EN.md` §1.1). These are very
  different poses (different sign on X/Y, real rotation vs. zero). Any
  exported Base-Frame coordinate is only physically meaningful if the
  plate — and the curved RX/TX model built above it — is actually posed
  at the real frame, not a demo layout picked for reachability.
- **TCP is a placeholder.** `tcp_local = np.loadtxt(TCP_FILE)`
  (`geometry_backend.py:2425`) is a single zero-pose *world point*,
  `[-798.137, -228.017, -109.903]`, no rotation, no tool-index concept.
  `T_flange_to_tcp`'s rotation is borrowed from `T_zero_flange_inv`
  (`geometry_backend.py:2429-2431`, `settled.md` S1.4's "not assumed
  identity" reasoning) — a hack that only works because the current tool
  has no real orientation offset from the flange. The real tool=1 offset
  (`[-134.777, 96.448, 106.334, 86.647, -13.136, 60.612]`) has a genuine
  ~87°/-13°/61° rotation this cannot represent.
- **No export path exists.** No file in the repo references `job.json`,
  `segment_id`, or `toolpath_T*` — new ground, not a partial feature.
- Confirmed by direct comparison: the DH table (`docs/FR5_DH_Table.md`)
  and zero-pose flange reference (`[-820, -202, 50]`) already match the
  new docs bit-for-bit. **No FK/DH change is in scope.**
  `docs/FR5_Joint_Limits.md`'s physical limits already match the new
  doc's §5 table exactly, but `gui_panel.py`'s `JOINT_LIMITS` constant
  (lines 8-15) is a separate, more conservative "practical slider range"
  — **not** interchangeable with the physical limits an export's
  self-check should use.

## Confirmed decisions (user, 2026-07-22)

1. Export whatever `toolpath_source` is currently active (planar or a
   curved layer) — reuse the existing selector, no new curved-only path.
2. Self-check the exported data against the Rejection Criteria table
   before writing files.
3. Single real TCP offset for tool=1 — not full multi-tool 0/1/2 — but
   structured so a second tool isn't a rewrite (a named constant + a
   `pose_to_matrix` helper, not values inlined into a hardcoded matrix).
4. Replace `saved_position.json` outright with the real User Frame; no
   dual demo/real slot. If arm reachability changes at the real pose,
   that's a finding to record, not a reason to keep the demo pose.

## Confirmed decisions (user, 2026-08-08) — sub-stage reorder

5. **The exchange spec's Rejection Criteria table is the definition of a
   valid job**, implemented **verbatim, all seven rows** — including the
   "TCP offset vs. our calibration" and "`num_points` != ply line count"
   rows this note originally dropped as self-referential.
6. **The in-house pose rejection is narrowed to the planar path.** The
   curved path loses both the posed-plate check (S1.40) and the
   tangent-plane check (S1.37); planar keeps Stage 6.8 behaviour
   unchanged. Accepted consequence: curved precompute will accept poses
   that drive the arm through the plate or the mockup shell, and nothing
   replaces that check — the spec's table validates data, not geometry.
7. **Reordered:** real TCP offset (§7.1) → rejection criteria (§7.2) →
   real User Frame (§7.3) → export (§7.4) → GUI (§7.5). The rejection
   criteria move to the front of the stage, but cannot lead it: §7.2's
   identity check compares against a reference TCP 6D pose that only
   exists once §7.1's real offset is in. Real User Frame drops to §7.3
   because nothing in the criteria depends on it.

## §7.1 — Real TCP Offset

**Leads the stage** because §7.2's identity check compares `FK(joints=0) +
TCP` against the exchange spec's reference TCP 6D pose, which is only
meaningful once this real offset is in place.

1. New module-level `pose_to_matrix(x, y, z, rx, ry, rz)` — port
   `docs/saved_coords_data_and_usage_EN.md` §3 verbatim
   (`R = Rz(rz) @ Ry(ry) @ Rx(rx)`). Only the forward `rot_x`/`rot_y`/
   `rot_z` (`geometry_backend.py:2991-3003`) exist today.
2. New matrix-to-Euler extraction helper, the exchange spec's own
   reference formulas (`ry = arcsin(-R[2,0])`, etc.) — needed by §7.2's
   identity and TCP-offset checks to report rotation error in degrees,
   not just a boolean.
3. New constant, e.g. `TCP_OFFSET_6D_MM_DEG = np.array([-134.777, 96.448,
   106.334, 86.647, -13.136, 60.612])`, commented as tool=1, the only
   tool in active use — extend to a `tool_index`-keyed dict here if a
   second tool is ever needed, not before.
4. `T_flange_to_tcp = pose_to_matrix(*TCP_OFFSET_6D_MM_DEG)` replaces the
   current construction (`geometry_backend.py:2429-2431`) outright — no
   more borrowing `T_zero_flange_inv`'s rotation.
5. The TCP point cloud / TCP Frame triad (`geometry_backend.py:2435-2448`)
   currently render at the zero-pose world point `tcp_local` directly.
   With a real offset, the equivalent zero-pose world point becomes
   `(self.T_zero[5] @ self.T_flange_to_tcp)[:3, 3]`, and the frame's
   rest-pose axes should take that transform's rotation submatrix
   (`create_coordinate_frame`'s `rotation` param already supports this,
   per `wiki/003_Guides/TCP_Frame.md` — no new mechanism, just a
   non-identity argument at this call site).

## §7.2 — Rejection Criteria (spec's table; in-house check narrowed to planar)

Two halves: retire this project's own pose rejection on the **curved**
path, and implement the exchange spec's Rejection Criteria table verbatim
as the definition of a valid exported job.

### Part 1 — narrow the in-house pose rejection to planar

**Kept, planar only.** `_plate_plane()`, `_meshes_clear_plane()`,
`_branch_clears_ground()`, the `allow_tcp_through_plate` toggle and its
GUI checkbox. The planar G-code path keeps Stage 6.8's behaviour exactly.

**Removed.**

- `_nozzle_clears_plane()` (`geometry_backend.py:2503`) — the S1.37
  tangent-plane check, curved-only by construction.
- `precompute_tip_tolerance_mm` — its only job was carrying that
  tolerance.
- `CURVED_TIP_CLEARANCE_TOLERANCE_MM` (`study_config.py`, moved there by
  S1.41) and its `geometry_backend.py` import — dead once the tangent
  check goes.

**The seam.** `_begin_toolpath_precompute()` already takes a
`tip_tolerance_mm=None` argument that is exactly the planar/curved
discriminator. Replace it with a boolean `check_collision`:
`run_toolpath_ik_precompute` passes **True**,
`run_curved_toolpath_ik_precompute` passes **False**.
`step_toolpath_ik_precompute` then runs the `_branch_clears_ground`
filter only when the flag is set and takes `solutions[0]` otherwise;
`_branch_clears_ground` loses its `plane` parameter and keeps only the
plate half. Prefer the flag over reading `self.toolpath_source` — the
precompute snapshots its own per-run state at begin, and a live-mutating
source field could change mid-solve.

`PRECOMPUTE_CACHE_VERSION` **4 → 5**: cached curved precomputes were
solved under the old accept/reject rule.

Keep `PLATE_THICKNESS_MM` (plate rendering + G-code placement, not just
collision) and `wrap_into_limits` branch filtering in
`solve_ik_tcp_matrix` — the solver still needs a limit window to pick a
representable branch; only its source changes (see the joint-limits row).

`solve_toolpath_ik()` (`geometry_backend.py:1678`) is **uncalled** — the
live path is the chunked precompute — so its `_branch_clears_ground` call
is already dead code. Worth deleting the whole method here.

### Part 2 — the spec's table, all seven rows

| Check | Rejection condition | Action |
|---|---|---|
| Identity check | pos ≥ 0.1mm OR rot ≥ 0.5° | REJECT |
| TCP offset vs. our calibration | pos ≥ 0.5mm OR rot ≥ 0.5° | REJECT |
| Joint limits | any joint out of range | REJECT |
| Per-point FK vs `tcp_xyz_base_mm` | error ≥ 0.1mm | REJECT |
| Joint step within a segment | > 30° between adjacent points | REJECT |
| `num_points` != ply line count | mismatch | REJECT |
| `\|J5\| < 2°` | singular configuration | **WARN** |

Implemented verbatim, including the "TCP offset vs. our calibration" and
"`num_points` != ply line count" rows. (An earlier draft of this note
dropped those two as meaningless for a self-check — no second calibration
source to compare against, and this project writes the ply itself. That
reasoning is **withdrawn**: the table is implemented as the receiving side
defines it.)

- **Identity check** — FK(`[0]*6`) + the new `T_flange_to_tcp` vs the
  doc's reference TCP 6D pose
  (`[-954.777, -308.334, 146.448, -161.378, -58.051, -25.434]`).
- **Joint limits means the real physical limits**
  (`docs/FR5_Joint_Limits.md`, J2/J4 −264..+84) — a new constant, *not*
  `gui_panel.py`'s `JOINT_LIMITS`, which is a deliberately conservative
  practical slider range.
- **Per-point FK** — FK(`joints_deg`) + `T_flange_to_tcp` vs the computed
  `tcp_xyz_base_mm` for every point about to be exported.
- **Joint step** — consecutive `precompute_joint_path` rows within one
  segment.
- **`|J5| < 2°` is a different notion from today's singularity flag.**
  `is_singular` is `|sin(theta5)| < 1e-6` — near-exact degeneracy. The
  spec's band is far wider and only warns.
- Rows 1 and 2 use §7.1's matrix-to-Euler helper, so rotation error
  reports in degrees.
- Produce a structured pass/fail-with-reasons result — export should
  abort (or, for the warn-only row, warn) and say *which* row failed, not
  refuse silently.

**Consequence.** Curved precompute no longer rejects on the mockup tangent
plane *or* the build plate, so it will accept branches Stage 6.8 rejected —
including ones that drive the arm through the plate or the shell. Planar is
unaffected. The spec's table does not cover collision, so nothing replaces
it on the curved path: the self-check catches bad *data*, not bad
*geometry*.

## §7.3 — Real User Frame

1. Overwrite `assets/buildPlate/saved_position.json`:
   `position_mm: [649.456, 133.762, 322.778]`,
   `rpy_deg: [-0.369, 0.329, -89.080]` — replacing
   `[-570, -300, -200]` / `[0, 0, 0]` entirely.
2. This changes `T_user_frame` for every downstream curved-model step
   that consumes it (`load_curved_model`'s placement, geodesics, print
   order travel-hover, orientation frames) — **all need re-running**
   against the new pose once it's loaded via the existing "Load Saved
   Position" button, not assumed to carry over from the old pose's
   results. Note §7.2 has by this point removed the plate/tangent
   clearance checks from the *curved* precompute, so the pose no longer
   feeds a collision test on that path — only the planar one.
3. **Known risk, not yet resolved:** the Stage 6.8 pose was specifically
   chosen because the *default* plate pose made the arm reach below the
   plate (BOOT_MATRIX's "Current 6.8 amendment"). The real User Frame is
   a different position and orientation entirely — reachability at this
   pose is unverified. If the arm cannot reach the curved model's
   waypoints here, that is real information to record (e.g. in a
   follow-up inbox note), not a signal to fall back to the demo pose.
   Reachability now means joint-limit and geometric reach only; with
   curved collision gone, "clears the plate" is no longer part of the
   test and must be judged by eye.

## §7.4 — Job Export

1. `job.json`: the header fields `format` (`"fr5_external_ik_job"`),
   `format_version` (`"2.0"`), `generator`, `generated_utc`; then
   `tool_index=1`, `tcp_offset_6d` (§7.1's constant),
   `identity_check.joints_zero_tcp_pose_base` (computed, not hand-typed),
   ordered `segments` list.
2. One `segment_N_solution.json` per segment: `segment_id`,
   `toolpath_file`, `num_points`, then `points[]` with `joints_deg`,
   `tcp_xyz_base_mm` (FK + TCP per point), `normal_base` — taken directly
   from each waypoint's `R_target`'s Z column (already the
   surface/plate outward normal by construction, `settled.md`
   S1.36/S1.12) — no new normal computation needed.
   Note the spec allows `tcp_xyz_base_mm` to differ from the ply's
   `x y z` by up to **2mm** ("optimization fine-tuning", spec §
   "Correspondence with the ply File"). That is the *receiving* side's
   tolerance — this project writes both from the same solve, so they
   should agree far more tightly than that; the 0.1mm per-point FK row of
   §7.2 is the binding check.
3. One `toolpath_T*.ply` per segment: same 6-column `x y z nx ny nz`
   format `read_ply_polyline()` already reads for the curved model —
   write it as the mirror of that reader, reusing its column order.
4. Segment boundaries: for curved layers, reuse Stage 6.3's print-order
   piece boundaries (`build_print_order`) directly — a segment is one
   continuous printed piece, matching the exchange spec's own definition
   ("one continuous extrusion line"). For planar G-code, use G1-run
   boundaries (a segment ends at a G0 travel move), the same underlying
   concept.
5. `surface.obj` — the spec's folder structure includes the print surface
   mesh. **No new work:** this is the active layer's existing
   `CURVED_LAYERS[l]["surface_file"]` (`Surface_TX_Base.obj` /
   `Surface_RX_Offset.obj`, already on disk in `assets/models/curved/` and
   already loaded), copied into the job folder under the spec's name. See
   the naming map in the conformance audit below.
6. Output location: still open (Stage7_README) — e.g.
   `assets/export/<job_name>/`. Decide during implementation; not
   load-bearing for the rest of the design.

## §7.5 — GUI Wiring

1. "Export IK Job" button in "I/O Operations" (`gui_panel.py`, alongside
   the existing "Load Curved Model" / "Load G-code preview" controls),
   enabled once the active `toolpath_source`'s precompute is complete
   (mirrors how playback controls already gate on precompute state).
2. Runs §7.2's self-check first; shows a pass/fail-with-reasons status
   line (same idiom as existing `precompute_status`/`playback_status`
   strings) before deciding whether to write files.

## Docs to update when implemented

- `settled.md`: new entries per sub-stage (numbers to confirm against
  whatever S1.x is latest at implementation time) — real TCP offset
  replacing S1.4's hack, the rejection criteria, real User Frame
  adoption, the exporter. §7.2 additionally needs an entry **superseding
  S1.37** (tangent-plane check removed) and **narrowing S1.40** to the
  planar path (not deleting it), plus a follow-up note on S1.41's table
  for the now-dead `CURVED_TIP_CLEARANCE_TOLERANCE_MM`.
- `ctx_system_current.md`: new status rows; the "Posed-plate collision
  (Stage 6.8)" row becomes planar-only, and the "S1.40 current setup
  amendment" needs revisiting — the RX setup it describes exists to
  satisfy a check curved runs no longer make.
- `BOOT_MATRIX.md`: new task-type row(s) for "real calibration / TCP" and
  "IK job export"; the curved-precompute rows describe the clearance
  checks as live.
- `wiki/003_Guides/CurvedModel_IKPrecompute.md`, `tutorials/Stage6_README.md`:
  same — both describe the curved clearance checks as live.
- `GLOSSARY.md`: consider a "TCP offset" vs "User Frame" disambiguation
  entry now that both have real, not placeholder, values.
- `tutorials/Stage7_README.md`: flip each roadmap item to done as it
  lands.

## Verification (for the implementing session — not run yet)

Headless, conda env `fairino-fr5-sim`
(`C:\Users\Edward\miniconda3\envs\fairino-fr5-sim\python.exe`):

1. §7.1: `pose_to_matrix` round-trips through the matrix-to-Euler helper
   for the real tool=1 offset; the new `T_flange_to_tcp` reproduces the
   doc's reference TCP 6D pose at zero joints within 0.01mm/0.01°.
2. §7.2: construct synthetic solved paths that fail each of the seven
   rows individually (bad identity, a TCP offset off by >0.5mm, an
   out-of-range joint, a corrupted per-point FK, a >30° joint jump, a
   `num_points`/ply mismatch, a `|J5|<2°` point) and confirm each is
   caught with the right reason and the right action (REJECT vs WARN);
   confirm a real valid path passes. Separately confirm the narrowing:
   a **planar** precompute still rejects a branch that dips below the
   posed plate, and a **curved** precompute no longer rejects on either
   the plate or the tangent plane. Confirm a stale cache at version 4 is
   invalidated.
3. §7.3: load the new saved position; confirm `T_user_frame` matches the
   real values; re-run geodesics/print-order/orientation/precompute and
   record whether they still complete (this is genuinely open — see
   "Known risk" above).
4. §7.4: export a real solved segment, re-parse the written `job.json`/
   segment/ply files, and confirm the re-parsed data matches what was
   exported (round-trip check) — including that the ply's line count
   equals the segment's `num_points`.

---

## Conformance audit (2026-08-08)

Run before implementation started, to answer two questions: does the Stage 7
plan cover the whole spec, and does the *existing* code already satisfy the
supervisor docs' assumptions? Headless under the `fairino-fr5-sim` env against
`docs/saved_coords_data_and_usage_EN.md` and
`examples/curved_surface_printing/external_ik_exchange_spec_EN.md`. Numbers
below are measured, not assumed.

### Conforms

| # | Assumption | Result |
|---|---|---|
| 1 | Standard DH, §5 table, `d4=102` | Exact match, all 6 rows. `d4=102`, not the deprecated 130 of §6 |
| 2 | FK(`[0]*6`) flange = `[-820, -202, 50]` | **0.000000 mm** error |
| 3 | `R = Rz(rz) @ Ry(ry) @ Rx(rx)` | Identical to this project's `rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)` (`geometry_backend.py:320`); `rot_x`/`rot_y`/`rot_z` match the doc's matrices element-for-element |
| 4 | Identity-check reference TCP pose | FK(0) + tool=1 reproduces `[-954.777, -308.334, 146.448]` to **0.000000 mm** and the rotation to **0.0003°** — against spec thresholds of 0.1mm / 0.5° |
| 5 | Nozzle axis = TCP local −Z | Already this project's stated convention (`geometry_backend.py:1144`): "Z = the outward surface normal (nozzle approaches along −Z, into the surface)" |
| 6 | Joint limits | `docs/FR5_Joint_Limits.md` matches §5 exactly. `gui_panel.py`'s `JOINT_LIMITS` is strictly *inside* the physical range on all six joints, so anything solved under the sliders passes §7.2's joint-limit row automatically |
| 7 | Normals: unit vectors, Base Frame | `R_target[:,2]` is exactly that |

Two consequences:

- **§7.1's `pose_to_matrix()` is a refactor, not new maths.** The convention is
  already implemented correctly, inline, in two places
  (`geometry_backend.py:320` and `:1674`). The helper extracts and names what
  is already there; it does not introduce a new rotation convention.
- **The identity check passes today on the maths alone.** It is gated only on
  wiring in the real tool=1 constant — which is precisely why §7.1 leads the
  stage.

### Does not conform — one real conflict

**Flange→TCP distance disagrees with tool=1 by 33.4mm.**

| | Vector (flange frame) | Magnitude |
|---|---|---|
| Current, from `assets/printerHead/` via the S1.4 construction | `[21.863, -159.903, 26.017]` | **163.47 mm** |
| Real tool=1, `saved_coords` §1.2 | `[-134.777, 96.448, 106.334]` | **196.91 mm** |

Magnitude is frame-independent, so this **cannot** be a rotation-convention
problem — the two describe tools of different physical length. Either
`assets/printerHead/` is a different tool than the one tool=1 was calibrated
against, or the calibration does not match the asset.

Consequence for §7.1: adopting the real offset moves the TCP by 311mm in vector
terms, after which the **rendered nozzle mesh will no longer sit at the TCP**.
That is a rendering/asset problem, not an IK one — the IK is correct either way,
since it targets the TCP frame. Open question against §7.1; do not reconcile the
asset blind.

### Naming map — the docs' terms vs this project's

The spec names several things that already exist here. Write the exporter
against this existing state rather than building new geometry:

| Spec term | Already in this project |
|---|---|
| `surface.obj` | `Surface_TX_Base.obj` / `Surface_RX_Offset.obj` — `CURVED_LAYERS[l]["surface_file"]`, on disk in `assets/models/curved/`, loaded already |
| segment ("one continuous extrusion line") | one Stage 6.3 print-order piece (`build_print_order`) |
| `normal_base` | `R_target[:,2]` |
| `tcp_xyz_base_mm` | FK + `T_flange_to_tcp` translation |
| `joints_deg` | a `precompute_joint_path` row |
| User Frame / WObj | `T_user_frame` / `assets/buildPlate/saved_position.json` |
| Flange Frame | `compute_fk()[5]` |
| Base Frame | the world frame |

`assets/calib/active_tcp_wobj.json`, named in the docs as the source of truth,
is the supervisor's file and is **not** present in this repo — the values are
transcribed into the docs, which is all this project needs.
