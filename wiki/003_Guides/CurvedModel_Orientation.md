---
status: active
---

# Per-Waypoint Tool Orientation from Surface Normals

## What it is

The "Build Orientation Frames" button (I/O Operations panel, shown once a
layer's print order exists) attaches a target TCP orientation to every printed
feed point, holding the nozzle perpendicular to a surface that is no longer
flat. This is roadmap `Stage6_README.md` sub-stage 6.4.

6.4 only **computes and visualises** these frames — nothing here drives the
arm. Feeding the per-waypoint orientation through IK is 6.5 (done, see
[`CurvedModel_IKPrecompute.md`](CurvedModel_IKPrecompute.md)).

Generic and project-agnostic (`settled.md` S1.33) — `build_orientation_frames()`
runs per layer, however many `CURVED_LAYERS` describes.

## Why a single constant orientation no longer works

Stage 5's flat-plate path snapshots **one** TCP rotation
(`T_user_frame[:3,:3]`) for an entire G-code path (`settled.md` S1.12) —
correct there because the build plate is flat and doesn't tilt mid-print, so
the nozzle prints perpendicular everywhere with one orientation. A curved
shell has a *different* surface normal at every point; holding one constant
orientation would drive the nozzle into the mockup wherever the shell steepens
away from that one direction.

`build_orientation_frames()` replaces the single matrix with a **per-waypoint**
`R_target`, the direct generalisation of the flat-plate convention: the flat
plate's `R_target` third column was already the plate's outward `+Z`, and at
`rpy=0` that reduces to `R = I` with the nozzle pointing straight down. The
curved case just lets that third column vary per point.

## How it's computed

`build_orientation_frames()` and `_orientation_frames_for_points()`
(`geometry_backend.py`) build an orthonormal basis per feed waypoint:

1. **Z axis = the outward surface normal.** The nozzle approaches along `-Z`,
   into the surface. Normals come from the already-outward
   `curved_surface_vnormals_world` (6.3's from-scratch, sign-fixed array,
   `settled.md` S1.35), sampled per feed point by nearest surface vertex
   (`nearest_vertex_index()`) — the same normal source 6.3's travel hover
   uses. The outward sign was fixed against `Surface_Bot` at load time, so no
   re-check is needed here; a wrong sign would drive the nozzle into the
   mockup.
2. **In-plane axes (X, Y) pinned to a fixed world reference — NOT the path
   tangent.** This is the one real design decision in 6.4, and it inverts the
   roadmap's original tentative guess. A print nozzle is rotationally
   symmetric about its own axis, so the remaining spin degree of freedom is
   physically free — nothing about the print depends on it. Aligning it to
   the path tangent (the original guess) would spin joint 6 to chase every
   wiggle in the path. Pinning it to a **constant world direction** instead
   means the frame only *tilts* as the normal changes and never *spins* as the
   toolpath meanders — "as stable and straight as possible", minimising wrist
   travel.
3. **Reference-axis selection, per point.** The reference is whichever of
   world X/Y/Z is most perpendicular to Z at that point
   (`argmin |world_axis . z|`), so the projection `x = normalize(a - (a·z)z)`
   never collapses and adjacent frames stay close as the normal sweeps near a
   world axis — a branchless degeneracy guard, not a special case.
   `y = z × x` closes the right-handed basis.
4. **Storage.** Frames are stored per layer as `curved_orient_frames` — a list
   of `(pos_world, R_target)` in print order — and rendered as a downsampled
   XYZ-triad overlay (`Curved Orient Frames <name>`, every
   `ORIENT_FRAME_STRIDE`-th waypoint, X red / Y green / Z blue, the same
   batched-triad pattern the coordinate-frame gizmo uses). Gated on
   `curved_order_loaded`; a re-order or reload invalidates the frames (state
   cleared, triads removed) so a stale overlay can't outlive its order.
   `apply_live_layer_visibility()` toggles the triads with the rest of the
   live layer.

`_orientation_frames_for_points()` is the shared frame-math helper —
`build_orientation_frames()` (6.4, feed points only) and 6.5's
`build_curved_toolpath_waypoints_world()` (feed *and* travel) both call it, so
the orientation convention lives in exactly one place.

## Measured properties

Measured directly against the shipped assets, 2026-07-21 (headless — the
interactive eyeball, confirming a triad reads outward and perpendicular on a
steep part of the shell, needs the GUI window and remains a manual check).

| Property | RX | TX |
|---|---|---|
| Feed waypoints / triads drawn | 2527 / 211 | 2000 / 167 |
| Orthonormality, `\|RᵀR - I\|` | < 6.7e-16 | < 6.7e-16 |
| `det(R)` | +1 | +1 |
| `R[:,2]` vs. nearest-vertex outward normal | equal to 0.0 | equal to 0.0 |

**Basis-construction stress test:** 20,000 random normals, same tolerances —
the most-perpendicular-axis pick avoids projection collapse near every world
axis.

## Conventions and gotchas

**Tangent-alignment was the natural first guess and is wrong here.** It's
worth remembering *why* before reapplying this pattern to a different tool: the
nozzle's rotational symmetry is what frees the spin DOF, and stability (not
following the path) is what fills it. A future tool that is *not*
rotationally symmetric (a directional applicator, a blade) would need the
spin DOF to track the path tangent or a process-defined direction instead —
this convention is non-revertible under that condition.

**Z is a hard constraint, X/Y are a tie-breaker.** Don't read the fixed-world-
reference choice as arbitrary — any right-handed basis with the correct Z
prints correctly; the reference only decides *which* of the infinitely many
valid bases minimises wrist motion.

## Current scope and limitations

- **Compute + visualise only.** No IK is solved in this stage — see
  [`CurvedModel_IKPrecompute.md`](CurvedModel_IKPrecompute.md) for how the
  array here gets fed through `solve_ik_tcp_matrix`.
- **Normals are per-vertex, not per-face-interpolated.** Sampled by nearest
  surface vertex, not barycentric-interpolated across the containing face. A
  finer verify could motivate that refinement; it would be a normal-source
  change, not a convention change.
- **No GUI gating beyond `curved_order_loaded`** — a "Build Orientation
  Frames" button and the existing RX/TX radio; no Clear button (roadmap 6.6).

## How to tune it

Generic engine tuning, `geometry_backend.py`:

| Constant | Effect |
|---|---|
| `ORIENT_FRAME_SCALE_MM` | Triad arm length in the overlay. |
| `ORIENT_FRAME_STRIDE` | Draw every Nth waypoint's triad, not all of them (density vs. clutter). |
| `ORIENT_FRAME_COLORS` | X/Y/Z triad colours (red/green/blue). |

## Code anchors

- `geometry_backend.py`: `build_orientation_frames()`,
  `_orientation_frames_for_points()`, `_register_orientation_frames()`,
  `apply_live_layer_visibility()`; `ORIENT_FRAME_SCALE_MM`/
  `ORIENT_FRAME_STRIDE`/`ORIENT_FRAME_COLORS` constants; `curved_orient_frames`/
  `curved_orient_loaded` state.
- `gui_panel.py`: "Build Orientation Frames" button, "I/O Operations" section.
- `wiki/002_Architecture/settled.md` **S1.36** — the full decision record;
  S1.12 — the flat-plate single-constant convention this supersedes; S1.35 —
  the outward normals this reuses; S1.4/S1.5 — the TCP `R_target` convention
  and reference-pose ranking that 6.5's IK consumes this array through.
- [`CurvedModel_PrintOrder.md`](CurvedModel_PrintOrder.md) — where the
  reused normal lookup and print order come from.
- `tutorials/Stage6_README.md` — sub-stage 6.4 and what 6.5 does with the
  resulting array.
