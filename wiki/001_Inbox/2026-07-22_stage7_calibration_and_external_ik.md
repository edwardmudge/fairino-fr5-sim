---
status: inbox
stage: 7.1-7.5
scope: geometry_backend.py, gui_panel.py, assets/buildPlate/saved_position.json
---

# Stage 7 — Real calibration + export to the external IK exchange format

## Why

Two supervisor docs landed (`docs/saved_coords_data_and_usage_EN.md`,
`docs/external_ik_exchange_spec_EN.md`) once the in-house solver was
validated. First read of this treated it as an *import* feature (ingest
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

## §7.1 — Real User Frame

1. Overwrite `assets/buildPlate/saved_position.json`:
   `position_mm: [649.456, 133.762, 322.778]`,
   `rpy_deg: [-0.369, 0.329, -89.080]` — replacing
   `[-570, -300, -200]` / `[0, 0, 0]` entirely.
2. This changes `T_user_frame` for every downstream curved-model step
   that consumes it (`load_curved_model`'s placement, geodesics, print
   order travel-hover, orientation frames, precompute's ground-clearance
   plate plane, S1.40's "posed plate" check) — **all need re-running**
   against the new pose once it's loaded via the existing "Load Saved
   Position" button, not assumed to carry over from the old pose's
   results.
3. **Known risk, not yet resolved:** the Stage 6.8 pose was specifically
   chosen because the *default* plate pose made the arm reach below the
   plate (BOOT_MATRIX's "Current 6.8 amendment"). The real User Frame is
   a different position and orientation entirely — reachability at this
   pose is unverified. If the arm cannot clear the plate or reach the
   curved model's waypoints here, that is real information to record
   (e.g. in a follow-up inbox note), not a signal to fall back to the
   demo pose.

## §7.2 — Real TCP Offset

1. New module-level `pose_to_matrix(x, y, z, rx, ry, rz)` — port
   `docs/saved_coords_data_and_usage_EN.md` §3 verbatim
   (`R = Rz(rz) @ Ry(ry) @ Rx(rx)`). Only the forward `rot_x`/`rot_y`/
   `rot_z` (`geometry_backend.py:2991-3003`) exist today.
2. New matrix-to-Euler extraction helper, the exchange spec's own
   reference formulas (`ry = arcsin(-R[2,0])`, etc.) — needed by §7.3's
   identity check to report rotation error in degrees, not just a
   boolean.
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

## §7.3 — Rejection-Criteria Self-Check

Run once on an already-solved path before export is allowed to proceed:

| Check | Rule | Source values |
|---|---|---|
| Identity check | pos error < 0.1mm, rot error < 0.5° at joints=0 | FK(`[0]*6`) + new `T_flange_to_tcp` vs the doc's reference TCP 6D pose (`[-954.777, -308.334, 146.448, -161.378, -58.051, -25.434]`) |
| Joint limits | every joint in range | **real physical limits**, `docs/FR5_Joint_Limits.md` — a new constant, not `gui_panel.py JOINT_LIMITS` |
| Per-point FK | error < 0.1mm | FK(`joints_deg`) + `T_flange_to_tcp` vs the computed `tcp_xyz_base_mm` for every point about to be exported |
| Joint step | adjacent-point step ≤ 30° per joint | consecutive `precompute_joint_path` rows within one segment |
| Singular (warn, not reject) | `\|J5\| < 2°` | per point |

Uses §7.2's matrix-to-Euler helper for the identity check's rotation
error. Produce a structured pass/fail-with-reasons result — export
should abort (or, for the singular-only row, warn) on failure, and say
*which* row failed, not just refuse silently.

Note: the exchange spec's own table also has a "TCP offset vs. our
calibration" row and a "num_points != ply line count" row — both are
meaningless for a self-check (there's no second calibration source to
compare against, and this project is producing the ply file itself, not
receiving one), so they're dropped here rather than carried over
unchanged from the import-shaped first draft.

## §7.4 — Job Export

1. `job.json`: `tool_index=1`, `tcp_offset_6d` (§7.2's constant),
   `identity_check.joints_zero_tcp_pose_base` (computed, not hand-typed),
   ordered `segments` list.
2. One `segment_N_solution.json` per segment: `joints_deg`,
   `tcp_xyz_base_mm` (FK + TCP per point), `normal_base` — taken directly
   from each waypoint's `R_target`'s Z column (already the
   surface/plate outward normal by construction, `settled.md`
   S1.36/S1.12) — no new normal computation needed.
3. One `toolpath_T*.ply` per segment: same 6-column `x y z nx ny nz`
   format `read_ply_polyline()` already reads for the curved model —
   write it as the mirror of that reader, reusing its column order.
4. Segment boundaries: for curved layers, reuse Stage 6.3's print-order
   piece boundaries (`build_print_order`) directly — a segment is one
   continuous printed piece, matching the exchange spec's own definition
   ("one continuous extrusion line"). For planar G-code, use G1-run
   boundaries (a segment ends at a G0 travel move), the same underlying
   concept.
5. Output location: still open (Stage7_README) — e.g.
   `assets/export/<job_name>/`. Decide during implementation; not
   load-bearing for the rest of the design.

## §7.5 — GUI Wiring

1. "Export IK Job" button in "I/O Operations" (`gui_panel.py`, alongside
   the existing "Load Curved Model" / "Load G-code preview" controls),
   enabled once the active `toolpath_source`'s precompute is complete
   (mirrors how playback controls already gate on precompute state).
2. Runs §7.3's self-check first; shows a pass/fail-with-reasons status
   line (same idiom as existing `precompute_status`/`playback_status`
   strings) before deciding whether to write files.

## Docs to update when implemented

- `settled.md`: new entries per sub-stage (numbers to confirm against
  whatever S1.x is latest at implementation time) — real User Frame
  adoption, real TCP offset replacing S1.4's hack, the self-check, the
  exporter.
- `ctx_system_current.md`: new status rows.
- `BOOT_MATRIX.md`: new task-type row(s) for "real calibration / TCP" and
  "IK job export".
- `GLOSSARY.md`: consider a "TCP offset" vs "User Frame" disambiguation
  entry now that both have real, not placeholder, values.
- `tutorials/Stage7_README.md`: flip each roadmap item to done as it
  lands.

## Verification (for the implementing session — not run yet)

Headless, conda env `fairino-fr5-sim`
(`C:\Users\Edward\miniconda3\envs\fairino-fr5-sim\python.exe`):

1. §7.1: load the new saved position; confirm `T_user_frame` matches the
   real values; re-run geodesics/print-order/orientation/precompute and
   record whether they still complete (this is genuinely open — see
   "Known risk" above).
2. §7.2: `pose_to_matrix` round-trips through the matrix-to-Euler helper
   for the real tool=1 offset; the new `T_flange_to_tcp` reproduces the
   doc's reference TCP 6D pose at zero joints within 0.01mm/0.01°.
3. §7.3: construct synthetic solved paths that fail each self-check row
   individually (bad identity, an out-of-range joint, a corrupted
   per-point FK, a >30° joint jump, a `|J5|<2°` point) and confirm each
   is caught with the right reason; confirm a real valid path passes.
4. §7.4: export a real solved segment, re-parse the written `job.json`/
   segment/ply files, and confirm the re-parsed data matches what was
   exported (round-trip check) — including that the ply's line count
   equals the segment's `num_points`.
