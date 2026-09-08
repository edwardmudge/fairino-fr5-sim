---
status: active
---

# Curved Print: The Working Setup

## ✅ Re-validated 2026-09-03 — read this before the banner below

**The banner that follows is out of date and is kept only as history.** It says
the curved path "has *not* been re-run since 7.1" and that the 3,175 RX / 2,688 TX
figures are pre-7.1 historical evidence. That has not been true since roadmap 7.4:

- Both layers solve **completely** at the real calibrated User Frame — RX
  3,175/3,175 and TX 2,688/2,688 — and both export `validate_job` **ACCEPTED**
  (`settled.md` S1.47/S1.48). The counts below are current, not historical.
- The placement that makes this work is `CURVED_MODEL_XY_OFFSET_MM = (0, 0)`
  (S1.48), which supersedes any plate-mesh-centred placement described later.
- Since v1.0 the app **starts at the saved calibrated User Frame** rather than
  `USER_FRAME_ORIGIN_MM` (S1.58), so the "move the plate first" step is already
  done for you on startup.

The order of operations below is still correct and is still the point of this
guide. To adapt the procedure to a *different* part, see
[`CurvedModel_AdaptingYourOwnJob.md`](CurvedModel_AdaptingYourOwnJob.md).

## ⚠ Scope — SUPERSEDED by Stage 7.1, not yet re-validated (HISTORICAL — see above)

This procedure was written against the pre-7.1 code: the original
`assets/printerHead/nozzle.obj` mesh and the `settled.md` S1.4 `tcp_local` TCP
construction.

**Stage 7.1 landed on 2026-08-14 and invalidated it** (`settled.md` S1.43). It
replaced the TCP with the real calibrated tool=1 offset and hid the nozzle
mesh (⚠ the nozzle is **visible again since 7.7**, re-aimed at load time onto
the TCP frame's −Z — `settled.md` S1.51; it is still not a collision body, so
nothing else in this banner changes), moving the TCP **310.97mm** (zero-pose
`[-798.137, -228.017, -109.903]` → `[-954.777, -308.334, 146.448]`).
Reachability is a direct function of where the TCP sits.

**The solve results recorded below (3,175 RX / 2,688 TX) are therefore
historical.** The curved path has *not* been re-run since 7.1 — treat every
number here as pre-7.1 evidence, not a current expectation. The procedure's
*shape* (the order of operations) is still expected to hold; only the poses and
counts are in question.

Two further reasons to re-derive rather than re-run verbatim:

- The tool's collision body is now the **single TCP point**, not the nozzle
  mesh, so the clearance this procedure was tuned to create is more permissive
  than it was.
- Stage **7.2** removes both the posed-plate and tangent-plane checks from the
  curved path entirely, and **7.3** replaces `saved_position.json` with the real
  User Frame. Re-validating now would likely be wasted work — this guide is best
  rewritten once 7.3 lands.

For reference, the *planar* path was re-validated at 7.1 and does solve
181,375/181,375, but only after `USER_FRAME_ORIGIN_MM` moved from
`[-600, -300, 0]` to `[-570, -300, -100]`. See
[`../001_Inbox/2026-07-22_stage7_calibration_and_external_ik.md`](../001_Inbox/2026-07-22_stage7_calibration_and_external_ik.md)
§7.1.

## What it is

The order of operations that produces a complete RX and TX solve. It is not
obvious, and the steps are **not** reorderable — the sequence is what creates
the clearance the arm needs.

## The procedure

1. Place the build plate at **`[-570, -300, 0]`** (Build Plate controls).
2. **Load the curved model.** It bakes to world coordinates against *that*
   plate pose.
3. Move the plate to **`[-570, -300, -200]`** — same X and Y, Z dropped 200mm.
   This is what `assets/buildPlate/saved_position.json` holds, so "Load Saved
   Position" does it.
4. **Rebuild geodesics → print order → orientation frames.** Required, not
   optional: the plate move invalidates them and the status line says so —
   *"Build plate moved -- geodesics invalidated, reload the curved model"*.
5. Run the RX precompute, then the TX precompute.

## Why the order matters

The plate move in step 3 invalidates the geodesics but **does not move the
model**. Nothing on the plate-move path calls `_reset_curved_model_state()` —
only `clear_curved_model()` does (plus `__init__`, to define the cleared
values) — so the retained world geometry stays anchored to the `Z = 0` pose
from step 2 while the plate — and with it the collision plane the arm is
tested against — drops 200mm.

That 200mm of clearance between the print target and the plate plane is what
lets the arm reach the waypoints. Step 4 then re-derives the geodesics from the
*unmoved* model.

**Corollary:** reloading the curved model *after* the plate has dropped
re-anchors it to the lowered plate and destroys the clearance. The model must
be loaded while the plate is at `Z = 0`.

## Evidence

Read from the precompute caches, not from memory:

| Cache | Waypoints solved | Plate pose | Cache ver | Toggle |
|---|---|---|---|---|
| `curved_rx.precompute.npz` | **3,175** | `[-570, -300, -200]` | 4 | `allow_tcp_through_plate = False` |
| `curved_tx.precompute.npz` | **2,688** | `[-570, -300, -200]` | 4 | `allow_tcp_through_plate = False` |

Both layers solve completely. Note both ran with `allow_tcp_through_plate`
**False** — the nozzle *is* blocked below the plate. Earlier prose in
`ctx_system_current.md` / `BOOT_MATRIX.md` described the adopted setup as
running with TCP-through *enabled*; the cache metadata is the evidence and the
prose has been corrected to match.

## ⚠ Known limitation — "solved" is not "collision-free"

A completed precompute means **reachable and plate-clearing**. It does *not*
mean safe. The TX run above completes, but **the arm passes through the
shoulder mockup**.

This is a long-standing gap, not a regression:

- **No mesh-vs-mesh collision exists anywhere in this project.** The arm links
  are only ever tested against the **plate plane** (`settled.md` S1.40). The
  mockup is never a collision body.
- S1.37's tangent-plane check is **nozzle-only, by deliberate design** —
  `_nozzle_clears_plane()`'s own docstring notes that testing the arm links
  "would reject every real printing pose", since the arm legitimately sits
  inward of a local tangent plane while reaching its contact point.
- `CURVED_OBSTACLE_FILE` (`Surface_Bot.obj`) is loaded and displayed, but used
  **only** by `_orient_normals_outward()` to fix normal direction — it is *not*
  a collision body. The per-pass obstacle-mesh approach was considered and
  rejected as too slow, replaced by the tangent-plane check (S1.37).

Judge arm-vs-mockup clearance **by eye in the viewer**, or by the real machine.
Nothing in the simulator will catch it.

**This section is the record of that open question.** It is open and
deliberately unscheduled: closing it needs a mesh-vs-mesh collision design
decision, which S1.37 already rejected once on performance grounds — not a task
that can be slotted into the roadmap without reopening that choice.

This matters for Stage 7: an exported job can pass all seven of the exchange
spec's Rejection Criteria and still drive the arm through the workpiece — those
criteria validate *data*, not *geometry*.

## Changed in Stage 7.2 — the gap is wider than the section above says

The section above is retained as the record, but it now understates the problem
in two ways. Both are settled facts, not predictions (`settled.md` **S1.44**).

**1. It is the arm *and the nozzle body*, and it has been since 7.1.**
The section says the tangent-plane check guards the nozzle. It no longer does,
and stopped when **7.1** reduced the tool's collision body to the single TCP
point: the check tested that point against the tangent plane through its own
waypoint, which IK pins it to, so the signed distance was identically zero.
Measured across all 5,863 cached RX+TX waypoints and 1,608 candidate branches,
it returned "clear" **every single time**, worst distance 3.4e-13mm against a
1.0mm tolerance. 7.2 deleted it as dead code; the protection was already gone.
So the thing to judge by eye is the arm **and** the tool.

**2. The curved path now has no geometric rejection at all.**
7.2 narrowed this project's own pose rejection to the **planar** path (confirmed
decision #6). Curved precomputes no longer run the posed-plate check either. A
curved solve now means *reachable and within joint limits*, nothing more —
"clears the plate" is no longer part of it. Planar is unchanged and keeps Stage
6.8 behaviour exactly.

**What this means in practice.** Before running a curved job on the real
machine, step through playback and watch both the arm and the tool against the
mockup. The simulator will not stop you, and after 7.2 it will not even stop you
driving through the build plate.

Closing this still needs the mesh-vs-mesh design decision S1.37 rejected once on
performance grounds. A cheaper option now exists than when that call was made:
`curved_surface_verts_world` and `nearest_vertex_index` are already computed per
waypoint for the orientation frames, so a point-to-surface distance test could
reuse them. It still needs real tool geometry — which no longer exists on any
path — and a performance budget. Logged as a Stage 8 candidate, not scheduled.

## Changed in Stage 7.3 — this whole procedure is now unrunnable as written

`assets/buildPlate/saved_position.json` no longer holds `[-570, -300, -200]`. It
holds the **real calibrated User Frame**, `[649.456, 133.762, 322.778]` /
`[-0.369, 0.329, -89.080]` (`settled.md` **S1.45**). Step 3 above — *"Load Saved
Position does it"* — therefore does something completely different now: it moves
the plate to the opposite quadrant, 322mm up, and yaws it ~89°.

**Measured consequence: the curved model is out of reach at the real frame.**
Headless, 2026-08-15, every feed point of both layers solved against
`PHYSICAL_JOINT_LIMITS` with no collision check (7.2 removed it here):

| Layer | Feed points | Solved at the **real** frame | Solved at the 6.8 demo pose | TCP distance from base (real frame) |
|---|---|---|---|---|
| RX | 2,527 | **226** (91.1% unreachable) | 2,527 (100%) | median **912mm**, max 945mm |
| TX | 2,000 | **186** (90.7% unreachable) | 2,000 (100%) | median **916mm**, max 947mm |

Every failure is `"Unreachable: no geometric solution for this pose"` — pure
geometry, not joint limits, and not collision. The cause is placement, not
calibration: `load_curved_model()` centres the assembly on the plate
(`T_placement`), and at the real frame the plate's centre sits ~844mm from the
base with the far corner ~980mm out, against a shoulder-to-wrist chain of
`a2 + a3 = 820mm`. At the demo pose the same points sit at a median of 475mm.

This was the "Known risk" §7.3 recorded in advance, and it resolved the way the
spec said to treat it: **as a finding, not as a reason to restore the demo
pose.** Full write-up, including what would have to change to print at the real
frame:
[`../001_Inbox/2026-08-15_real_user_frame_reachability.md`](../001_Inbox/2026-08-15_real_user_frame_reachability.md).

**Status of this guide:** the *order of operations* (load the model at working
height, then drop the plate, then rebuild) is still the correct mechanism and
worth keeping. The *poses* in it are dead — steps 1 and 3 name a pose the file
no longer contains, and step 3's clearance trick has no meaning at the real
frame. The placement question below this guide's rewrite was blocked on is now
**answered (S1.48)** — the numbers above (~844mm plate centre, `a2+a3=820mm`)
were themselves an artefact of the plate-mesh-centred placement that S1.48
replaced; re-derive this section's numbers against the corrected placement
before rewriting, not simply "re-run with new numbers" against the old one.

## Changed in Stage 7.4 — arm-vs-mockup is now guarded; nozzle-vs-mockup is not

The open question this guide has carried since 6.8 — *nothing stops the arm
driving through the shoulder mockup* — is **closed for the arm** as of
2026-09-03 (`settled.md` **S1.47**).

**Filter 8** tests the arm links against each layer's own print surface
(`Surface_RX_Offset` / `Surface_TX_Base`) at **1.0mm** clearance, and **filter 9**
tests the arm against itself at 5.0mm. These are the first mesh-vs-mesh checks
in the project — exactly the obstacle-mesh approach S1.37 declined to build, now
affordable because a rejected pose is no longer a rejected *waypoint*: with 540
commanded orientations searched, ~95% of reachable waypoints still yield an
admissible pose.

Demonstrated, not asserted: a TX pose at waypoint 518 places an arm link
**0.71mm** from the print surface, passes filters 2–7, is a valid IK solution
within the physical joint limits, and **the pre-7.4 code would have accepted it**
(7.2 left the curved path with no geometric test at all).

⚠ **The nozzle is still unguarded, and this section must not be read as closing
that.** The tool's entire collision body is the single TCP point (7.1), and it is
*deliberately excluded* from filters 6–8 — IK pins it to the commanded waypoint,
which lies on the print surface, so including it would reject every printing
pose. So "judge arm-vs-mockup clearance by eye" above is superseded; **judge
nozzle-vs-mockup clearance by eye still stands**. Closing it needs a corrected
tool asset (`nozzle.obj` is 163.47mm against tool=1's 196.91mm), not a filter.

✅ **And the curved path CAN now be run at the real frame** — a separate,
unrelated problem to the one above, found and fixed the same day (S1.48). 7.4
measured 1,922/2,527 RX and 1,410/2,000 TX feed points admissible (up 8.5× from
226/186), but ~24% had no IK solution at *any* of the 540 orientations, so the
precompute aborted.

The cause was `load_curved_model()` centring the workpiece on the build plate
MESH's bbox centre rather than on the User Frame — a **+105.6mm** outward
offset from `BambuLab_BuildPlate.obj`, a stand-in, 258×276mm with its origin at
a corner. That put the workpiece 843.1mm out against the FR5's 922mm flange
reach. Rather than ask the supervisor "corner or centre", the fix was decided by
measurement (explicit user direction — the calibration data is already
verified, so whatever's reachable is correct): a new
`CURVED_MODEL_XY_OFFSET_MM = (0,0)` places the workpiece directly on the User
Frame origin. **Re-measured after the fix, full pipeline: RX 3,175/3,175, TX
2,688/2,688, both `validate_job` ACCEPTED.** See
[`../001_Inbox/2026-09-03_curved_placement_plate_centring_offset.md`](../001_Inbox/2026-09-03_curved_placement_plate_centring_offset.md)
(closed) and `settled.md` **S1.48**.

## Code anchors

- `geometry_backend.py`: `load_build_plate()` (the plate-move invalidation and
  its status message), `load_curved_model()` (bakes `T_curved` against the plate
  pose at load; XY placement is `CURVED_MODEL_XY_OFFSET_MM` relative to the
  User Frame origin since S1.48, NOT the build-plate mesh's bbox), `_reset_curved_model_state()` /
  `_abort_geodesic_precompute()` (what a plate move does and does not clear),
  `run_curved_toolpath_ik_precompute()`.
- `examples/curved_surface_printing/study_config.py`:
  `CURVED_MODEL_XY_OFFSET_MM` — the workpiece placement constant, S1.48.
- `assets/buildPlate/saved_position.json`: the real User Frame since Stage 7.3
  (the `[-570, -300, -200]` pose this guide is written around survives only as
  the file's inert `_legacy_stage6_8_demo_pose` record).
- `settled.md` **S1.40** (posed-plate collision — the check itself **deleted** at
  7.4, replaced by the finite footprint + slab), **S1.37** (nozzle-only tangent
  check — gone at 7.2; its obstacle-mesh argument overturned at 7.4), **S1.42**
  (the reset helpers), **S1.47** (the filter set, and the measurements above).
- `geometry_backend.py`: `_candidate_admissible()` (the nine filters),
  `_filter_context()` (why the tool point is excluded), `orientation_candidates()`,
  `dijkstra_candidate_path()`.
- [`BuildPlate_UserFrame.md`](BuildPlate_UserFrame.md) — the plate controls
  themselves.
- [`CurvedModel_Loading.md`](CurvedModel_Loading.md) — how the model is placed.
