---
status: active
---

# Curved-Surface IK Precompute

## What it is

`run_curved_toolpath_ik_precompute(layer, ...)` wires a print layer's ordered,
oriented waypoints (6.3 + 6.4) through IK, reusing Stage 5's chunked
precompute/playback machinery rather than rewriting it. This is roadmap
`Stage6_README.md` sub-stage 6.5.

There is no GUI button for this yet, and no curved playback — both are 6.6.
This stage is `geometry_backend.py` only.

## Why reuse Stage 5's machinery instead of writing a new solver

Stage 5 already has a chunked, pausable IK precompute with a ground-clearance
branch filter and a disk cache (`settled.md` S1.14/S1.13/S1.21) — built for a
G-code path but structurally generic: solve waypoint `i`, rank branches
against the previous solved pose, filter by a clearance check, cache the
result. The curved case needs the exact same shape with two waypoint-level
differences (a per-waypoint target orientation instead of one constant, and a
different clearance obstacle) — reusing the shape and swapping only what
differs was cheaper and safer than a parallel implementation.

`_begin_toolpath_precompute(waypoints, R_target_array, ...)` is the shared
seam: both `run_toolpath_ik_precompute` (planar G-code) and
`run_curved_toolpath_ik_precompute(layer, ...)` (curved) load their own
waypoint source into the same state, and `step_toolpath_ik_precompute` is the
one solver loop for both.

## How it's computed

1. **Waypoint source.** `build_curved_toolpath_waypoints_world(layer)` merges
   6.3's ordered feed pieces and inter-piece travel hops into one flat
   `(waypoints, R_target_array)` list — the curved analogue of
   `build_toolpath_waypoints_world`. Each element carries an `is_feed_move`
   flag and a per-waypoint orientation from 6.4's
   `_orientation_frames_for_points()` (both feed *and* travel waypoints get an
   orientation, since the arm has to hold a sensible pose during travel too,
   not just while printing).
2. **Per-waypoint `R_target`.** `precompute_R_target` is now an `(N,3,3)`
   array indexed per waypoint, not one shared matrix. The planar path keeps
   its old behaviour unchanged by broadcasting its one constant rotation to
   `(N,3,3)` — a read-only view, no extra allocation, so nothing about the
   flat-plate solve actually changes.
3. **Nozzle clearance without an obstacle mesh.** This is the one genuine
   design fork in 6.5 — see *Why the obstacle-mesh plan was rejected* below.
4. **Per-layer disk caches.** `curved_precompute_cache_path(layer_name)` gives
   `curved_<layer>.precompute.npz`, so the planar benchy and each curved pass
   keep independent caches instead of thrashing one fixed file.
   `_curved_toolpath_cache_meta()` hashes the *derived* waypoint positions,
   feed flags, and orientation array (there's no single curved source file to
   hash the way there's one G-code file — these derived arrays are what drift
   on a re-order or re-orient), plus the build-plate pose.
   `PRECOMPUTE_CACHE_VERSION` bumped 1→2 for the schema change (a one-time
   silent rebuild of the existing planar cache too, not a bug).

## Why the obstacle-mesh plan was rejected

The original plan (asset survey, `settled.md` S1.37's own record) was a
literal per-pass obstacle mesh: check the arm/nozzle against `Surface_Bot` for
the RX pass and `Surface_RX_Offset` for the TX pass (standing in for the
already-printed, cured RX traces + silicone fill). This was rejected for two
reasons:

1. **Too slow.** `nearest_vertex_index()` is brute-force by design (no scipy,
   `settled.md` S1.31) — querying a moving mesh's full vertex set against a
   tens-of-thousands-vertex obstacle thousands of times per precompute doesn't
   fit the chunked-per-frame budget.
2. **A literal "never touch the obstacle" check rejects every real printing
   pose**, since the nozzle tip touching the print surface is exactly what
   printing *is*.

**What replaced it: a per-waypoint tangent-plane check.** Since each print
surface is treated as a convex-ish dome cap, **a tangent plane at a point on a
convex surface is a supporting hyperplane for the whole body**: everything on
its outward side provably clears the entire surface behind it. Because
`RX_Offset` sits outward of `Surface_Bot` and `TX_Base` outward of
`RX_Offset` everywhere (the measured stack, `settled.md` S1.30/S1.32/S1.34), a
point outward of *this waypoint's own* tangent plane (point = the waypoint,
normal = its `R_target[:,2]`, already computed by 6.4) also clears every
surface further inward — for either pass, with no obstacle mesh at all.

**The check applies to the tool tip only** (`_nozzle_clears_plane()` — since
Stage 7.1 this is the **TCP point**, not the nozzle mesh; see the note at the
end), **not the arm links.** The supporting-hyperplane proof bounds where the
*surface* is, not where the *arm* is — the arm must span from its base up to the
contact point, so its lower links legitimately sit far *inward* of a local
tangent plane (measured Robot1 ~-92mm, Robot2 ~-194mm at a real waypoint)
while the tip sits on the surface (~0). Testing the links would reject
every real printing pose. The tip gets `CURVED_TIP_CLEARANCE_TOLERANCE_MM`
(~1.0mm, assumed) of *inward* slack — it prints *on* the surface, so its
worst signed distance is ~0 and sometimes slightly negative after mesh
discretisation; the tolerance is *added* to the signed distance (subtracting
would demand the tip float outward and reject every feed waypoint). Like the
posed-plate check it layers with, it tries the cheap 8-corner bounding-box
bound first and only escalates to the exact per-vertex check when that's
inconclusive.

**The posed build-plate plane is now the physical clearance gate.** The plane
is derived live from `T_user_frame`: its point is the top face, lifted by
`PLATE_THICKNESS_MM`, and its normal is the plate's local +Z. It is modelled as
an infinite plane, so the plate must sit below the whole arm, including the
base and lower links. This replaces the old world-z=0 proxy and the removed
`reject_below_ground` toggle.

The six arm-link meshes are always rejected below the posed plate. The tool
(the TCP point since 7.1) is also rejected by default; the GUI's **Allow TCP
through build plate** toggle (`allow_tcp_through_plate`, default OFF) is the
only exception, and it never permits an arm link through the plate. On curved
runs, this plate check is layered with the existing tip-only tangent-plane
check. The toggle changes which IK branch is accepted, so it is included in
both precompute cache keys; `PRECOMPUTE_CACHE_VERSION` is 4 (**5 since Stage
7.1**). `_branch_clears_ground(angles, plane)` still
uses `plane=None` for planar paths and a waypoint tangent plane for curved paths.

**RX setup requires a decoupled load order.** `load_curved_model()` places the
mockup on the current plate. Move the plate to `[-570, -300, 0]` at working
height, load the curved model, then load the saved pose `[-570, -300, -200]`
without reloading the model. Rebuild geodesics, print order, and orientation
frames, enable `allow_tcp_through_plate`, and run RX precompute. Loading the
low plate before the model lowers the mockup with it and is invalid. This
adopted X+30 pose solves all 3,175 RX waypoints.

## Known limitation

**Full arm-vs-mockup collision beyond the tool tip is not checked.** The
supporting-hyperplane argument only bounds the tip; nothing here stops
an arm link from intersecting the mockup elsewhere. The posed-plate check
handles the plate as an infinite plane, not the finite curved obstacle. This is the same
simplification class as the old planar z=0 proxy (which also never checked
the full arm against the plate). Closing the mockup gap would need the rejected
obstacle-mesh (or per-triangle) check for the affected surface — left as a
future improvement, not attempted here because of the same performance
concern that ruled it out above.

## Reachability is a placement property, not a code concern

At the adopted X+30 pose, the curved run solves all 3,175 RX waypoints. The
unshifted placement stopped at the 1,809-waypoint joint-limit boundary; the X
shift moves the path clear of that reachability failure while retaining the
same low plate height.

The current cache was produced with `allow_tcp_through_plate` **False** — the
nozzle *is* blocked below the plate. (Earlier prose here said "TCP-through
enabled"; the cache metadata is the evidence. See
[`CurvedModel_PrintSetup.md`](CurvedModel_PrintSetup.md).)

## Measured properties

Verified headless, 2026-07-22.

| Check | Result |
|---|---|
| Planar regression | The planar path uses the same posed-plate clearance gate; stale v3 caches miss under version 4 and are rebuilt |
| RX precompute | All 3,175 RX waypoints solve with the adopted X+30 placement. The shipped cache records `allow_tcp_through_plate: false` |
| FK round-trip on solved poses | Target position matches to 6.7e-13mm; `R_target[i]` matches to 3e-15 — the per-waypoint orientation threads through IK exactly |
| Tip clearance | A solved branch clears its own tangent plane; shifting that plane outward past the tip by more than the tolerance correctly rejects it. Measured against the *nozzle mesh* — 7.1 reduced the tested body to the TCP point |
| Per-layer cache plumbing | `curved_rx.precompute.npz` / `curved_tx.precompute.npz` remain independent; both cache metas include `allow_tcp_through_plate` at version 4 |

**Remaining:** the RX path still stops at its genuine joint-limit boundary;
the interactive GUI checkbox/disable-state eyeball remains a physical-window
check.

---

## Changed in Stage 7.1 (2026-08-14)

The clearance *design* above is unchanged — the supporting-hyperplane argument
and the tip-only-not-links reasoning both still hold, and were always really
about the tip rather than the mesh. Two things it says are now stale:

**The tested body is the single TCP point, not the nozzle mesh.**
`_nozzle_clears_plane()` keeps its name (7.2 deletes it, so renaming would
churn every reference for one sub-stage) but now indexes
`moving_geometry_rest_verts[6]`, which is the TCP point. Consequences:

- The check is **strictly more permissive** than when measured above. The
  mesh's shoulders used to be able to fail a waypoint the tip itself cleared;
  they cannot now.
- `CURVED_TIP_CLEARANCE_TOLERANCE_MM` (~1.0mm) was chosen against mesh
  behaviour. Against a point it is doing a different job, and has not been
  re-tuned. It is moot in practice — see below.
- The 8-corner bbox bound degenerates to 8 coincident points, so the
  corners-first escalation is a no-op here.

**The measured results above predate 7.1 and have not been re-run.** The real
tool=1 offset moved the TCP 310.97mm, so every solved branch changes. Treat the
3,175 RX figure, the `X+30` adopted pose and the 1,809-waypoint boundary as
historical — see the warning at the top of
[`CurvedModel_PrintSetup.md`](CurvedModel_PrintSetup.md).

**`PRECOMPUTE_CACHE_VERSION` is 5**, not 4: the TCP offset is a constant rather
than a cache-key field, so the bump is what invalidates joint paths solved for
the old tool.

**Stage 7.2 removes this check entirely** on the curved path, along with
`_nozzle_clears_plane()`, `precompute_tip_tolerance_mm` and
`CURVED_TIP_CLEARANCE_TOLERANCE_MM`. The posed-plate check narrows to planar
only. Nothing replaces either on the curved path — the exchange spec's
Rejection Criteria validate *data*, not geometry. That is a deliberate,
recorded trade (`settled.md` S1.43 and the Stage 7 inbox note §7.2), which is
why re-tuning the tolerance now would be wasted work.

## Current scope and limitations

- **Shared GUI wiring is now landed in 6.6**, including the posed-plate toggle
  added in 6.8; `main.py` remains wiring only.
- **Curved playback is implemented** through the shared source-aware playback
  controls; the curved bead-size constants are active in that path.
- **The per-pass obstacle distinction is moot** under the tangent-plane
  design — `study_config.py` needed no new per-pass obstacle field.

## Non-revertible unless

The mockup stack turns out non-convex somewhere — then a waypoint's tangent
plane is no longer a global supporting hyperplane and the nozzle-clearance
argument fails, forcing a real obstacle-mesh (or per-triangle) check for the
affected surface (also the path to close the known arm-vs-mockup limitation
above).

## Code anchors

- `geometry_backend.py`: `_begin_toolpath_precompute()` — the shared seam;
  `run_curved_toolpath_ik_precompute(layer, ...)`,
  `build_curved_toolpath_waypoints_world()`,
  `_orientation_frames_for_points()` (shared with 6.4),
  `_nozzle_clears_plane()`, `_plate_plane()`, `_meshes_clear_plane()`,
  `_branch_clears_ground()` (now takes an optional `plane` arg),
  `curved_precompute_cache_path()`, `_curved_toolpath_cache_meta()`;
  `precompute_cache_path`/`precompute_tip_tolerance_mm` state.
- `examples/curved_surface_printing/study_config.py`:
  `CURVED_TIP_CLEARANCE_TOLERANCE_MM` (moved here by `settled.md` S1.41 — it
  is nozzle/material-dependent).
- `wiki/002_Architecture/settled.md` **S1.37** — the full decision record;
  S1.12/S1.13/S1.21 — the Stage-5 chunked-precompute/ground-clearance/cache
  machinery this reuses; S1.36 — the orientation frames this consumes;
  S1.30/S1.32/S1.34 — the measured layer stack the supporting-hyperplane
  argument depends on.
- [`CurvedModel_Orientation.md`](CurvedModel_Orientation.md) — where
  `R_target` per waypoint comes from.
- [`CurvedModel_PrintOrder.md`](CurvedModel_PrintOrder.md) — where the
  ordered feed pieces and travel hops this stage interleaves come from.
- `tutorials/Stage6_README.md` — sub-stage 6.5 and what 6.6 does next.
