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

**The check applies to the nozzle tip only** (`_nozzle_clears_plane()`), **not
the arm links.** The supporting-hyperplane proof bounds where the *surface*
is, not where the *arm* is — the arm must span from its base up to the
contact point, so its lower links legitimately sit far *inward* of a local
tangent plane (measured Robot1 ~-92mm, Robot2 ~-194mm at a real waypoint)
while the nozzle tip sits on the surface (~0). Testing the links would reject
every real printing pose. The nozzle gets `CURVED_TIP_CLEARANCE_TOLERANCE_MM`
(~1.0mm, assumed) of *inward* slack — it prints *on* the surface, so its
worst signed distance is ~0 and sometimes slightly negative after mesh
discretisation; the tolerance is *added* to the signed distance (subtracting
would demand the tip float outward and reject every feed waypoint). Like the
ground-clearance check it extends, it tries the cheap 8-corner bounding-box
bound first and only escalates to the exact per-vertex check when that's
inconclusive.

**World `z=0` is dropped for the curved case.** The curved mockup sits above
the plate in a frame where z=0 is not the physical floor, so valid printing
poses routinely put arm links below z=0 (measured z_min ~-60 to -300mm on the
only joint-limit-valid branches of a real waypoint) — retaining the z=0 gate
rejected every such pose. The planar path is unaffected —
`_branch_clears_ground(angles, plane=None)` is still exactly the old z=0
check.

## Known limitation

**Full arm-vs-mockup collision beyond the nozzle is not checked.** The
supporting-hyperplane argument only bounds the nozzle tip; nothing here stops
an arm link from intersecting the mockup elsewhere. This is the same
simplification class as the old planar z=0 proxy (which also never checked
the full arm against the plate). Closing it would need the rejected
obstacle-mesh (or per-triangle) check for the affected surface — left as a
future improvement, not attempted here because of the same performance
concern that ruled it out above.

## Reachability is a placement property, not a code concern

On the shipped assets at the default plate pose, 7 of 3175 RX waypoints and 6
of 2688 TX are geometrically reachable but have no joint-limit-valid IK
branch (verified solving each in isolation), so a full curved precompute
aborts at the first one (no partial motion, `settled.md` S1.12's contract).
This is expected: the build plate pose is a free variable
(`load_build_plate(rpy_deg=...)`), meant to be varied until a fully reachable
placement is found. Finding that pose is a setup step, out of this stage's
`geometry_backend.py`-only scope.

## Measured properties

Verified headless, 2026-07-21.

| Check | Result |
|---|---|
| Planar regression | All 181,375 G-code waypoints solve; v2 cache written and reloaded ("Loaded ... from cache"); `plane=None` clearance path byte-for-byte identical to the old z=0 check |
| RX reachable prefix | Full 1809-waypoint reachable prefix solves (6.4 orientation + nozzle clearance), before the expected abort at the first dead spot |
| FK round-trip on solved poses | Target position matches to 6.7e-13mm; `R_target[i]` matches to 3e-15 — the per-waypoint orientation threads through IK exactly |
| Nozzle clearance | A solved branch clears its own tangent plane; shifting that plane outward past the tip by more than the tolerance correctly rejects it |
| Per-layer cache plumbing | `curved_rx.precompute.npz` / `curved_tx.precompute.npz` written independently and each reload via a fresh `run_curved_toolpath_ik_precompute` call |

**Remaining:** an end-to-end full-curved solve + cache, which needs a plate
pose with no unreachable waypoints (the placement step above), and the
interactive GUI eyeball (roadmap 6.6).

## Current scope and limitations

- **`geometry_backend.py` only** — no `gui_panel.py`/`main.py` change; no
  curved playback yet (roadmap 6.6).
- **Curved-specific bead-size constants are deferred to 6.6**, where curved
  playback will actually call the bead builder — nothing renders a curved
  bead until then, and shipping the constant unused would be half-finished
  code (`AGENTS.md`).
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
  `_nozzle_clears_plane()`, `_branch_clears_ground()` (now takes an optional
  `plane` arg), `curved_precompute_cache_path()`, `_curved_toolpath_cache_meta()`;
  `CURVED_TIP_CLEARANCE_TOLERANCE_MM` constant; `precompute_cache_path`/
  `precompute_tip_tolerance_mm` state.
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
