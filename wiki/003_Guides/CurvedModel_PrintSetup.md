---
status: active
---

# Curved Print: The Working Setup

## ⚠ Scope — this documents the *current* code

This procedure applies to the code **as it stands today**: the original
`assets/printerHead/nozzle.obj` mesh and the `settled.md` S1.4 `tcp_local` TCP
construction.

**Stage 7.1 will invalidate it.** That sub-stage replaces the TCP with the real
calibrated tool=1 offset and hides the nozzle mesh, moving the TCP **311mm**
(zero-pose `[-798.137, -228.017, -109.903]` → `[-954.777, -308.334, 146.448]`).
Reachability is a direct function of where the TCP sits, so the procedure must
be re-validated after 7.1 and **the solve results recorded below will not carry
over**. See
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

## Code anchors

- `geometry_backend.py`: `load_build_plate()` (the plate-move invalidation and
  its status message), `load_curved_model()` (bakes `T_curved` against the plate
  pose at load), `_reset_curved_model_state()` /
  `_abort_geodesic_precompute()` (what a plate move does and does not clear),
  `run_curved_toolpath_ik_precompute()`.
- `assets/buildPlate/saved_position.json`: the `[-570, -300, -200]` pose.
- `settled.md` **S1.40** (posed-plate collision), **S1.37** (nozzle-only
  tangent check), **S1.42** (the reset helpers).
- [`BuildPlate_UserFrame.md`](BuildPlate_UserFrame.md) — the plate controls
  themselves.
- [`CurvedModel_Loading.md`](CurvedModel_Loading.md) — how the model is placed.
