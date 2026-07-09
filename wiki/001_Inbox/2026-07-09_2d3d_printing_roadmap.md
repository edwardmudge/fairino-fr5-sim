---
status: draft
---

# Roadmap: Stage 5 (2D Printing) & Stage 6 (3D Printing)

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
- **Build plate orientation is full roll/pitch/yaw** (3 sliders), not a
  single tilt axis.
- **"Best location" search is manual exploration.** Drag the orientation
  sliders, watch the toolpath, judge by eye. No automated
  reachability-scoring or optimizer function is planned.

---

## Stage 5 — 2D Printing on the Build Plate

### 5.1 Build-plate orientation slider (roll, pitch, yaw)

**Goal:** Let the build plate be rotated in the scene, not just
translated.

This is the trigger condition
[`settled.md` S1.2](../002_Architecture/settled.md) already named as its
own future amendment: `T_user_frame` was deliberately stored as a full 4x4
matrix, even while only translation-populated, "to leave room to add a
rotation later without changing the storage shape." Implementing this
step means:

1. `T_user_frame` gains a populated rotation submatrix, built from three
   new slider values (roll, pitch, yaw) in `gui_panel.py`.
2. The build plate mesh's placement in `load_build_plate()`
   (`geometry_backend.py`) must switch from the current raw
   `+ USER_FRAME_ORIGIN_MM` vector add
   ([`BuildPlate_UserFrame.md`](../003_Guides/BuildPlate_UserFrame.md))
   to a full homogeneous matrix multiply — exactly the switch S1.2's own
   "non-revertible unless" clause anticipated.
3. **Open question, not decided here:** which Euler convention/rotation
   order combines roll, pitch, yaw into one matrix. Pick one when
   implementing and record it as a new dated entry in `settled.md`
   (e.g. S1.6), the same way every other transform convention in this
   project is on record.

**Files:** `geometry_backend.py` (`load_build_plate()`, rotation matrix
construction), `gui_panel.py` (3 new sliders). No new files — consistent
with `CLAUDE.md`'s Surgical Changes rule.

**Verify:** Dragging each of the three new sliders visibly tilts the
plate mesh and its User Frame triad together, independent of the other
two axes; setting all three back to zero returns the plate to its exact
current (translation-only) placement.

### 5.2 Generalize the G-code parser; Cura for slicing

**Goal:** Load a real Cura-exported `.gcode` file instead of the one
hardcoded test fixture.

Cura itself is external and out of scope for this codebase — the user
slices standalone and exports a file. What's in scope is turning
`parse_gcode()` / `load_gcode()` (`geometry_backend.py`) from what
[`Gcode_Toolpath.md`](../003_Guides/Gcode_Toolpath.md) already documents
as a deliberately minimal, single-fixture parser into something that
survives a real slicer's output. That doc's own "Current scope and
limitations" section is effectively a pre-written gap list for this step
— work through it directly rather than re-deriving requirements:

- Arc support (`G2`/`G3`) — interpolate into short line-segment chains.
- Unit switching (`G20`/`G21`) — currently mm is assumed always.
- Relative positioning (`G90`/`G91`) — currently absolute is assumed
  always.
- Dynamic file loading — replace the hardcoded `GCODE_DIR`/`GCODE_FILE`
  constants with a real file picker. Check
  [`Polyscope_Quickstart.md`](../../docs/Polyscope_Quickstart.md) for
  what ImGui file-dialog options actually exist before assuming one —
  per `CLAUDE.md`'s SDK Investigation Rule, don't guess an unfamiliar
  `psim.*` widget's behavior.
- Malformed-line reporting — currently unparseable/unsupported lines are
  silently skipped.

**Open question, flagged not decided:** Cura's FDM export includes
extrusion (`E`) and temperature (`M104`/`M109`) words that a 2D
single-layer preview doesn't need numerically. These should be
**parsed-but-ignored** for Stage 5 — recognized and retained on each
waypoint rather than silently dropped the way unsupported G/M-codes are
today — so Stage 6 (which does need `E`) doesn't require a second parser
rewrite.

**Files:** `geometry_backend.py` only.

**Verify:** Load an actual Cura-exported `.gcode` file (not the
`square_test.gcode` fixture) for a simple flat 2D shape and confirm the
toolpath preview renders correctly — including any arcs, and regardless
of whether the file uses relative or absolute positioning or explicit
units.

### 5.3 Print on the surface; use orientation to find the best fit

**Goal:** Combine 5.1 and 5.2 into the actual workflow the supervisor
described — tilt the plate under a loaded toolpath and visually judge fit.

Per the manual-exploration decision above, this isn't a new feature so
much as composing what 5.1 and 5.2 already provide: load G-code, tilt the
plate with the new sliders, watch the toolpath move with it, and manually
cross-check a few points against the existing IK panel if reachability is
in doubt (reusing the existing multi-branch solver and ranking from
`settled.md` S1.4/S1.5 — no new reachability-checking code).

**One real gap to resolve when implementing 5.1+5.2 together:**
[`settled.md` S1.3](../002_Architecture/settled.md) transforms the G-code
toolpath through `T_user_frame` **once, at load time** — correct for a
static plate, but it means tilting the plate *after* G-code is already
loaded won't move the already-drawn curve. This needs a decision: either
re-run the toolpath transform whenever the orientation sliders change, or
move the G-code curve onto a per-frame update path (S1.3's own
"non-revertible unless" clause already flags "live streaming" as the
trigger for needing a per-frame pipeline — this is a milder version of
that same trigger). Record whichever is chosen as a `settled.md` update
to S1.3.

**Files:** `geometry_backend.py` (whichever of the two options above is
chosen), `gui_panel.py` if the slider callback needs to trigger a
re-transform.

**Verify:** With a G-code file already loaded, drag any orientation
slider and confirm the drawn toolpath tilts with the plate in real time
(not just on the next "Load G-code" click).

---

## Stage 6 — 3D Printing

Kept intentionally lighter and more general here — mirroring how the
original kit's Stage 4 "Advanced Extensions" was a looser grab-bag than
Stages 1–3 — since this depends on how Stage 5 actually lands.

Builds on Stage 5's parser handling multiple Z-layers rather than one
flat pass, and is where the Stage 5.2 parsed-but-ignored `E` value
actually gets used (e.g. only rendering an edge where `E` increases, to
distinguish print moves from travel at a finer grain than `G0`/`G1`
already does). Layer-height changes are a natural place to add the
per-layer/Z-height color-coding `Gcode_Toolpath.md` already names as a
gap.

No hardware-control design is proposed here — the point of keeping
`T_user_frame` as a clean matrix and the parser decoupled from rendering
(reiterated from the scope decisions above) is specifically so that a
later "drive a real FR5" step doesn't force a rearchitecture, whenever
and however that gets decided.

---

## Deferred, on purpose

Not doing yet, until the above is actually implemented and these
decisions are actually made:

- New `GLOSSARY.md` terms (slicer, layer, build-plate orientation).
- A new `BOOT_MATRIX.md` row routing future G-code/slicing tasks to this
  document and to `Gcode_Toolpath.md`.

Per `CLAUDE.md`'s Documentation Updates rule, these get added once the
vocabulary is actually in use, not speculatively for a plan that hasn't
been built yet.
