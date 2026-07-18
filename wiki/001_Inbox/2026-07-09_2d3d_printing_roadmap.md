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
- A fixed load path, `assets/models/planar/gcode/model.gcode` — every Cura
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

See [`Gcode_Toolpath.md`](../003_Guides/Gcode_Toolpath.md) for the full
record, including gaps that are genuinely still unbuilt (not decisions):
unit switching (`G20`/`G21`) and malformed-line reporting.

### 5.3 Toolpath execution — drive the arm through the print (not started)

**Goal:** Take the already-loaded, already-positioned G-code toolpath —
a single flat layer *or* a full multi-layer print (e.g. the full benchy,
~180,000 waypoints) — and actually move the FR5 through it via IK,
rather than only previewing it, with a **playback speed slider** running
anywhere from real-time (paced to the G-code's `F` feedrate) up to as
fast as the sim can solve/render. This is the step that makes this a
printer instead of a viewer, and the thing to get working before Stage 6
starts.

**Open questions to resolve when implementing** (not decided here):

1. **Tool orientation during printing.** `parse_gcode()` only carries
   X/Y/Z; `solve_ik_tcp` needs a full 6-DOF target (`settled.md` S1.4).
   Natural default: hold the TCP orientation constant, normal to the
   plate (derived from `T_user_frame`'s rotation), since the plate
   doesn't tilt mid-print — true for a single layer and for every layer
   of a multi-layer print alike (unlike Stage 6's curved surface, which
   would need the orientation to actually vary).
2. **Per-waypoint IK + continuity.** `solve_ik_tcp` already ranks
   branches by proximity to `self.current_joint_angles` (`settled.md`
   S1.4/S1.5) — this is exactly the "path following" trigger S1.5's
   "Non-revertible unless" clause anticipated, and should work as-is
   per-waypoint (each solve continues from wherever the previous one
   left the arm). Worth adding: a whole-path reachability pre-check
   (solve every waypoint before moving anything, report the first
   unreachable one) reusing the same solver — no new reachability-
   checking code.
3. **Speed-slider animation control that doesn't exist yet.** Every
   current `gui_panel.py` control is an immediate one-shot action;
   this needs new Play/Pause state plus a speed slider spanning
   real-time (derive timing from each waypoint's most recent `F` value)
   to as-fast-as-possible (advance every waypoint each frame, no timing
   delay), and a per-frame advance through the waypoint list driven by
   whichever pace is selected.
4. **Unverified at ~180,000-waypoint scale: is per-waypoint IK-solving
   actually fast enough?** Even at the "as fast as possible" end of the
   slider, solving IK ~180,000 times has a real cost that hasn't been
   measured. Benchmark `solve_ik_tcp`'s per-call cost against the full
   benchy waypoint count early during implementation, before committing
   to the rest of the animation-control design — if it's too slow, that
   changes the design (e.g. batching, a coarser IK-solve stride with
   interpolation between solved points, etc.), so it needs an answer
   first, not last.

**Files:** `geometry_backend.py` (new driver logic, reusing
`solve_ik_tcp`), `gui_panel.py` (new Play/Pause + speed slider controls).

**Verify:** With a toolpath loaded (both a flat single-layer shape and
the full benchy) and the plate positioned, starting playback moves the
arm's TCP through the waypoints in sequence and the nozzle visibly
traces the loaded curve; dragging the speed slider from real-time to
as-fast-as-possible visibly changes playback pace.

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
