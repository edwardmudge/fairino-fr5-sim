---
status: draft
---

# Roadmap: Stage 5 (Flat-Bed Printing: 2D & Multi-Layer) & Stage 6 (Curved-Surface 3D Printing)

Non-authoritative plan draft — see
[`TRUTH_LADDER.md`](../005_AgentMgmt/active/ctx_main/TRUTH_LADDER.md). As
pieces of this get implemented, the relevant decision should graduate into
[`settled.md`](../002_Architecture/settled.md), the same way this stage
itself grew out of `settled.md` S1.2/S1.3 (see below). This file describes
the *plan*, not a record of what's built — check
[`ctx_system_current.md`](../005_AgentMgmt/active/ctx_main/ctx_system_current.md)
for current status.

## Where this picks up

The original kit roadmap
([`tutorials/FR5_FK_Kit_README.md`](../../tutorials/FR5_FK_Kit_README.md))
is complete — FK, mesh rendering, TCP, and IK are all done, and its
optional "Stage 4 — Advanced Extensions" grab-bag (build plate, User
Frame, G-code preview) is done too. There are now two different "Stage 4"
labels floating around in the project's history (`CLAUDE.md`'s
FK→mesh→TCP→**IK** numbering, and the kit README's FK→mesh→TCP→**advanced
extensions** numbering) — both finished, both slightly different. To stop
perpetuating that ambiguity, this document starts fresh at **Stage 5**.

The supervisor's brief for this phase: 2D printing on the build plate,
then eventually 3D printing. For 2D printing specifically, three
ingredients were given:

1. A build-plate orientation slider.
2. Cura for slicing, and a general-purpose G-code solver to load what it
   produces.
3. Use the G-code toolpath on the (now tiltable) plate to find a good
   orientation by printing on the surface.

Stage 5 is scoped to printing on a **flat** build plate — this covers
both a single 2D layer and a full multi-layer 3D print (e.g. the
benchy): the plate itself stays flat/planar throughout either way, so
multi-layer is a natural extension of Stage 5, not a separate stage.
Stage 6 is the genuinely different, harder problem — printing on a
**curved** (non-planar) plate/surface; see Stage 6 below.

## Scope decisions already made (confirm before implementing if this file goes stale)

- **Stays a simulation.** The supervisor's longer-term intent is for this
  orientation work to eventually feed a real FR5 as an "orientation
  generator," but whether/how that connects to physical hardware is
  genuinely undecided. Keep the design hardware-agnostic — a clean
  `T_user_frame` matrix and a parser decoupled from rendering — rather
  than building toward a specific hardware API that doesn't exist yet.
- **Cura integration is manual export/import.** Slice in the standalone
  Cura GUI, export a `.gcode` file. This codebase never shells out to
  Cura or CuraEngine.
- **Build plate pose is full position + roll/pitch/yaw**, applied via
  click-to-apply `InputFloat3` fields and Move/Reset/Save/Load Position
  buttons (`settled.md` S1.6) — not live-drag sliders.
- **"Best location" search is manual exploration.** Reposition the plate,
  watch the toolpath, judge by eye. No automated reachability-scoring or
  optimizer function is planned.

---

## Stage 5 — Flat-Bed Printing (2D & Multi-Layer)

### 5.1 Build-plate position & orientation — done

**Shipped:** `T_user_frame` is a full 4x4 pose (position + rotation),
built each call from `load_build_plate(position_mm, rpy_deg)` using the
**XYZ fixed-angle convention** (`R = Rz(yaw) @ Ry(pitch) @ Rx(roll)`,
`settled.md` S1.6 — same convention `solve_ik_tcp` already used). The
"Build Plate Orientation" panel (`gui_panel.py`) exposes `InputFloat3`
position/RPY fields and four click-to-apply buttons:

- **Move** — apply the current fields.
- **Reset** — back to `USER_FRAME_ORIGIN_MM` / zero rotation exactly.
- **Save Position** — write the current pose to
  `assets/buildPlate/saved_position.json`.
- **Load Saved Position** — read it back and apply it.

**Fix (2026-07-10):** The build plate now sets an explicit cool-gray
`BUILD_PLATE_COLOR`, so it no longer collides visually with the warm-orange
deposited-bead mesh (`settled.md` S1.14). Its measured 0.75mm thickness is
also accounted for: `position_mm` remains the place where the plate rests,
while `T_user_frame` is lifted along the plate's local Z axis so the plate's
underside lands on that shared floor and the print surface sits above it
(`settled.md` S1.15).

See `settled.md` S1.6 and
[`BuildPlate_UserFrame.md`](../003_Guides/BuildPlate_UserFrame.md) for
the full record.

### 5.2 G-code toolpath: parser, loading, and live preview — done

**Shipped:**

- A custom G0/G1-only parser (`parse_gcode()`/`load_gcode()`,
  `geometry_backend.py`) — a third-party tokenizer
  (`AndyEveritt/GcodeParser`) was evaluated and rejected (generic
  tokenizer only, no modal-position/arc/relative-positioning handling).
  Arcs (`G2`/`G3`) and relative positioning (`G91`) are discarded **in
  software** by decision, not assumed absent from the input file
  (`settled.md` S1.7).
- A fixed load path, `assets/models/gcode/model.gcode` — every Cura
  export overwrites the same file; no file-picker UI.
- The toolpath preview stays in sync with the plate: Move, Reset, and
  Load Saved Position each reload and re-transform the curve against the
  plate's current pose, instead of only updating on a separate "Load
  G-code" click (`settled.md` S1.8).
- **Tried and reverted:** translucent rendering (`GCODE_TRANSPARENCY`)
  caused a constant frame-rate regression on a real multi-layer print's
  toolpath (~180,000 `G0`/`G1` segments — alpha-blending that many
  overlapping segments is expensive). Reverted to opaque; revisiting
  translucency needs a plan for the segment-count/rendering-cost problem
  itself, not just re-adding it.
- **Multi-layer already works, no code change needed.** `parse_gcode()`
  tracks Z modally with no single-layer assumption, so a full multi-layer
  3D print loads and previews correctly today — proven directly by the
  benchy (~180,000 `G0`/`G1` lines across many layers) rendering
  correctly. This was already true before this stage's scope was
  explicitly widened to include multi-layer; it just wasn't previously
  called out.

**Fix (2026-07-10):** The first deposited layer now hangs down by the
actual first-layer toolpath Z instead of the general Cura layer height, so
it reaches the plate-local top surface at `Z=0`. Later layers still use the
general layer height and remain stacked flush on the layer below
(`settled.md` S1.15).

See [`Gcode_Toolpath.md`](../003_Guides/Gcode_Toolpath.md) for the full
record, including gaps that are genuinely still unbuilt (not decisions):
unit switching (`G20`/`G21`) and malformed-line reporting.

**Toolpath execution — drive the arm through the print.** Subsections
5.3–5.8 below are the ordered build sequence for Stage 5.3: take the
already-loaded, already-positioned G-code toolpath — a single flat layer
*or* a full multi-layer print (e.g. the full benchy, ~180,000 waypoints) —
and actually move the FR5 through it via IK, not just preview it. This is
the step that makes this a printer instead of a viewer. Each subsection is
one small piece you build and verify before starting the next; together
they're the template Stage 6 (curved-surface printing) reuses. Built and
verified in this order; full decisions on record in `settled.md`
S1.9–S1.11.

### 5.3 Matrix IK target with a reference pose — done

**Build:** Add `solve_ik_tcp_matrix(target_pos, rotation, joint_limits,
reference_joint_angles)` taking the TCP orientation as a rotation matrix
plus an explicit reference pose, and make the existing `solve_ik_tcp`
delegate to it. This lets the toolpath solver hand IK a matrix directly
(no matrix→RPY→matrix round-trip) and rank branches against the *previous
waypoint's* accepted pose rather than the live arm pose.

**Verify:** `solve_ik_tcp` returns the same solutions it did before the
refactor — the RPY entry point is unchanged from the outside.

### 5.4 Fix the print orientation and place waypoints in the world — done

**Build:** Snapshot `T_user_frame[:3, :3]` once at the start of a run as a
constant TCP rotation (normal to the plate — the plate doesn't tilt
mid-print), and transform every plate-local waypoint through `T_user_frame`
into world coordinates. Execution parses both `G0` travel and `G1` feed
moves so the arm can reposition continuously between deposited segments —
unlike the `G1`-only preview (`settled.md` S1.9/S1.10).

**Verify:** A hand-checked waypoint solves to a pose whose TCP sits where
the preview bead is, held flat to the plate.

### 5.5 Ground-clearance branch filter — done

**Build:** For each waypoint, walk the continuity-ranked IK branches and
pick the first one whose moving geometry clears the plate. The transformed
bounding-box min-z (`moving_geometry_bbox_min_z`) is a *guaranteed lower
bound* on the exact full-mesh min-z, so `bbox ≥ 0` accepts the branch
outright; the exact `moving_geometry_min_z` runs only to adjudicate a branch
whose bbox dips below `z=0` (it may still clear). This bbox-first ordering is
both correct and ~5× cheaper than checking every branch's exact mesh
(`settled.md` S1.12).

**Verify:** A branch that would drive the arm or nozzle through the plate
is skipped in favour of one that clears; a waypoint where *every* branch
crosses `z=0` is reported as a failure.

### 5.6 Chunked, pausable IK precompute — done

**Build:** Precompute the whole IK path *before* any motion, in small
per-frame batches driven from `render()`, via `start_/step_/pause_/resume_/
cancel_toolpath_ik_precompute`. A live progress bar advances instead of the
window freezing; the first unreachable / out-of-limits / all-branches-
below-ground waypoint aborts the job and reports its index, with no partial
motion (`settled.md` S1.9). To keep the ~180k-waypoint solve to tens of
seconds, IK is solved only at adaptive keyframes (every ~2.5 mm of arc, plus
every real corner) and the dense per-waypoint joint path is interpolated at
finish; playback then replays cached joints and never pays the solve cost per
frame (`settled.md` S1.12).

**Verify:** The full benchy precomputes in ~40 s (down from ~220 s) with the
progress bar climbing; Pause/Resume holds and continues without losing
progress; Cancel stops it; the arm still traces the path with no visible
change.

### 5.7 Progressive-reveal playback — done

**Build:** Reuse the swept-bead preview mesh and `set_print_reveal`
(`settled.md` S1.11). `reset_toolpath_playback` snaps to the first pose and
empties the shape; `advance_toolpath_playback(step_count)` steps the cached
joint path and grows the revealed beads to match progress.

**Verify:** During playback the printed shape grows bead-by-bead in step
with the nozzle; Reset empties it back to nothing.

### 5.8 GUI wiring — done

**Build:** Wire Run / Pause / Reset plus the precompute Pause/Resume/Cancel
controls, a progress bar + status line, and a speed slider into
`gui_panel.py`. The slider is a whole-steps-per-frame multiplier (1–100):
playback advances `max(1, int(speed))` cached waypoints each frame.
**Real-time feedrate pacing is deferred** (`settled.md` S1.9) — playback is
a cached-joint stepper, not `F`-timed; `F` re-enters `parse_gcode` when that
feature is actually built.

**Verify:** With a flat shape and the full benchy loaded and the plate
positioned, Precompute then Run drives the TCP through the waypoints as the
shape reveals, and dragging Speed across 1–100 visibly changes the pace.

### 5.9 Persist precompute across sessions — done

**Build:** Cache the completed joint path to `model.precompute.npz` beside the
G-code, tagged with a key = G-code **content hash** + build-plate pose +
keyframe params + a version. At startup and when **Precompute** is pressed,
rebuild the key from the current inputs and load the path instantly on a match
instead of re-solving; any change to the object, plate pose, or keyframe
settings changes the key, so a stale cache is ignored and re-solved
(`settled.md` S1.13). Startup auto-load only fires at the default plate pose
(S1.6 forbids auto-restoring a saved position); the moved-plate case is served
by the button.

**Verify:** Precompute once (~37 s, writes the cache); restart — the path is
ready instantly ("loaded from cache"); move the plate and Precompute — it
re-solves because the pose is part of the key.

---

## Stage 6 — Curved-Surface 3D Printing

Intentionally left blank for now — to be filled in once Stage 5 lands.

**Goal (stated, not designed):** printing on a **curved** (non-planar)
surface, sliced by a **proprietary slicer**, not Cura — Cura only
produces flat-layer G-code, so a genuinely different slicing approach is
needed here. This is a harder, different problem than a straightforward
"add more Z-layers to a flat print," which an earlier draft of this
section assumed; that framing no longer applies.

No hardware-control design is proposed here — the point of keeping
`T_user_frame` as a clean matrix and the parser decoupled from rendering
(reiterated from the scope decisions above) is specifically so that a
later "drive a real FR5" step doesn't force a rearchitecture, whenever
and however that gets decided.

---

## Deferred, on purpose

Not doing yet, until the above is actually implemented and these
decisions are actually made:

- Dedicated `GLOSSARY.md` terms for **slicer** and **layer** — genuinely
  still not needed; nothing in the codebase names either concept
  directly yet. (Build-plate orientation is already covered by the
  existing "User frame" entry, and a "G-code toolpath" entry already
  exists too — those were done alongside 5.1/5.2, not deferred.)

Already done, no longer deferred: a `BOOT_MATRIX.md` row routing
build-plate/G-code tasks to this document, `BuildPlate_UserFrame.md`, and
`Gcode_Toolpath.md` now exists.

Per `CLAUDE.md`'s Documentation Updates rule, the remaining item above
gets added once that vocabulary is actually in use, not speculatively for
a plan that hasn't been built yet.
