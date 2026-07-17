---
status: active
---

# How the G-code Toolpath Preview Works

## What it is

A static preview of a parsed G-code file's deposited material, drawn on
the build plate as a solid **swept bead surface mesh** — a box per
extruding segment, sized from the actual extrusion and layer data, not a
thin wireframe. Clicking "Load G-code preview" (I/O Operations panel)
parses `assets/models/gcode/model.gcode` — the fixed name/location every
Cura export is saved to — maps its points into world space via the
plate's `T_user_frame`, and registers an orange mesh named `"G-code
Print"` covering only the `G1` (feed) segments that actually extrude —
`G0` (travel) moves and non-extruding `G1` moves (retraction, priming)
are tracked for position but produce no bead, so the mesh shows the
printed/cut object, not incidental repositioning. The Build Plate
Orientation panel's Move/Reset/Load Saved Position buttons do **not**
reload this mesh — an already-loaded preview is left showing the old
plate pose until "Load G-code preview" is clicked again explicitly.
(S1.8's original button-triggered auto-reload was removed by `settled.md`
S1.23 — see `wiki/003_Guides/BuildPlate_UserFrame.md` for why.)

A translucent rendering mode was tried twice, both times as groundwork
for an eventual layer-by-layer build-up preview, and reverted both times
— **for two different reasons**, see `settled.md` S1.10 for the measured
detail:

1. On the old curve-network preview: rendering ~180,000 overlapping
   translucent segments caused a constant (not just click-triggered)
   frame-rate regression.
2. On this bead mesh, re-attempted on the hypothesis that the mesh
   representation itself wouldn't have the old regression — measured
   directly (not assumed): `ps.set_transparency_mode("simple")` indeed
   has no measurable frame-cost even at this scale (confirming the old
   regression was specific to `"pretty"` mode's multi-pass blending), but
   `set_transparency_mode()` turned out to be a **scene-global** renderer
   switch — it made every other fully-opaque structure (the arm, the
   plate) render translucent too, not just this mesh. That's an
   unacceptable side effect for a default-on feature, independent of the
   good frame-time number.

The mesh stays opaque; the transparency code was removed after
measuring, since it's blocked on a Polyscope API limitation (no
per-structure transparency-mode opt-in found), not on rendering cost.

## How it's computed

`VisContent.parse_gcode()` and `VisContent.load_gcode()`
(`geometry_backend.py`):

1. `parse_gcode(filepath)` reads the file line by line, strips comments
   (`;...` to end of line, and inline `(...)`), and tokenizes each line
   with `GCODE_MOVE_RE` (`([A-Za-z])\s*(-?\d+\.?\d*)`). Only `G0`/`G1`
   lines are kept; any other G/M code, or a line with no G-word, is
   skipped.
2. Position and extrusion are **modal**: an axis word (X/Y/Z) or `E`
   missing from a line keeps its last known value, matching standard
   G-code semantics — only the very first line defaults missing axes/E
   to 0. This applies whether the line is `G0` or `G1`. (`G92`, which
   redefines the firmware's absolute E origin without moving anything,
   isn't a G0/G1 code, so it's discarded like any other unsupported code
   — the parser's own absolute `E` tracking doesn't need it, since
   `load_gcode()` only ever reasons about `E` *deltas* between
   consecutive waypoints, not absolute position, and every real
   `G92 E`-reset in a Cura export is immediately followed by a zero-length
   G1 that gets filtered out anyway — see point 5.)
3. Each recognized line appends `([x, y, z], e, is_feed_move)` to the
   returned list, where `is_feed_move` is `True` only for `G1`.
4. `load_gcode()` converts the parsed points (plate-local mm) to world
   coordinates with a single homogeneous multiply,
   `T_user_frame @ [x, y, z, 1]^T` — not the per-frame Delta pipeline,
   since the toolpath is static workpiece geometry with no joints (see
   `settled.md` S1.3, same reasoning as the plate mesh itself, S1.2).
5. A segment (waypoint `i` → `i+1`) gets a bead only if: the destination
   move is `G1`, extrusion increased (`E` delta > 0 — filters out travel,
   retraction, and the zero-length priming/un-retract moves Cura's
   standard start G-code and every travel jump emit), the segment has
   real length, its horizontal width axis is well-defined (not a
   near-vertical/degenerate direction), and its layer band has non-zero
   height. This is why, e.g., the real `model.gcode`'s startup sequence
   (`G1 Z15.0 ... ;Prime the extruder` at Z=15, before the first real
   layer at Z=0.3) produces no visible geometry — those moves are either
   zero-length E-only moves or non-extruding travel.
6. **Bead height** comes from each drawn segment's enclosing layer band,
   tracked as a running floor that only *extruding* segments advance:
   whenever a real print segment's destination Z exceeds the previous
   print Z, the previous print Z becomes the new floor; an unexpected
   downward jump resets the floor to 0 (the plate surface). This starts
   at 0 for the very first layer, so first-layer beads automatically
   reach the plate's top surface with no special-casing — see
   `settled.md` S1.9 for why this is geometric (derived from the actual
   printed Z sequence) rather than a parsed `;Layer height:` comment,
   which doesn't reliably describe the first layer.
7. **Bead width** comes from the extruded filament volume — `E` delta
   times a filament cross-section area assuming `FILAMENT_DIAMETER_MM`
   (1.75mm, a documented assumption, not parsed metadata — see
   `settled.md` S1.9) — divided by (segment length x bead height), the
   standard slicer-viewer formula.
8. Each valid segment becomes an independent box: 8 corner vertices (4 at
   each endpoint, offset ± half the bead width along a horizontal axis
   perpendicular to travel, at the layer's bottom and top Z) and up to 12
   triangles (6 faces). This is built fully vectorised across all
   segments at once (no per-segment Python loop or `trimesh.creation.box`
   + concatenate), since a real print is on the order of ~180,000
   segments.
9. **Cap-face culling**: at a bead-to-bead boundary that is back-to-back
   in the G-code (no travel gap between them), colinear (unit travel
   directions' dot product ≥ `CAP_CULL_COLINEAR_DOT_MIN`), and
   width-matched (cross-section widths differ by ≤ `CAP_CULL_WIDTH_TOL_MM`),
   the two beads' shared cap faces are provably always hidden inside the
   print and are dropped — 2 triangles off each side of the boundary
   (~8% fewer triangles on a real multi-layer print, since most
   consecutive segments trace a curved surface and fail the colinearity
   test). This means `faces` no longer has a fixed 12-triangles-per-bead
   stride; `_build_gcode_beads()` also returns `bead_face_prefix`, a
   `(K+1,)` cumulative-triangle-count array, so `faces[:bead_face_prefix[n]]`
   is exactly the triangles for the first `n` beads — see `settled.md`
   S1.19, S1.20.
10. The mesh is registered via `ps.register_surface_mesh("G-code Print",
    ...)` and colored `GCODE_COLOR` — same-name re-registration replaces
    the prior mesh, no explicit clear needed.
11. `load_gcode()` is called only from the "Load G-code preview" button
    (`gui_panel.py`) — **not** from the Build Plate Orientation panel's
    Move/Reset/Load Saved Position buttons. Repositioning the plate does
    not re-transform an already-loaded preview mesh; it's left showing
    the old `T_user_frame` until "Load G-code preview" is clicked again
    (`settled.md` S1.23 superseded S1.8's button-triggered reload). It
    no-ops if the G-code file doesn't exist yet, since the button is
    reachable before any G-code has ever been loaded.

## Current scope and limitations

This is a G0/G1-only line-segment parser, not a general G-code
interpreter — **by decision**, not just an unbuilt gap: see `settled.md`
S1.7. It's the project's accepted general-purpose G-code loader, used for
real Cura exports (the original `square_test.gcode` fixture still exists
at `assets/models/gcode/square_test.gcode` but isn't the default anymore
— see the dynamic file loading bullet below); a third-party tokenizer
(`AndyEveritt/GcodeParser`) was evaluated and
rejected (generic tokenizer only, no modal-position/G91/arc handling —
wouldn't have removed any of the actual parsing work). Known
gaps/non-goals, and where each would need to hook in if ever revisited:

- **No arc support (`G2`/`G3`) or relative positioning (`G91`) — out of
  scope by decision (settled.md S1.7), not pending work.** The
  supervisor ruled G0/G1-only sufficient; rather than rely on Cura never
  emitting these, `parse_gcode()`'s existing `if code not in (0, 1):
  continue` filter discards them in software if they ever appear (so a
  file that happens to contain an arc doesn't crash the loader — it just
  silently loses that segment). If ever revisited: arcs would need
  interpolation into a short line-segment chain, and `G91` would need
  `x, y, z` accumulated as deltas instead of overwritten, both in
  `parse_gcode()`.
- **No unit switching (`G20`/`G21`)** — all coordinates are assumed to
  already be millimeters (matches Cura's default output).
- **Bead width assumes a fixed 1.75mm filament diameter
  (`FILAMENT_DIAMETER_MM`)** — not parsed metadata, since Cura's export
  carries no filament/nozzle-diameter comment; see `settled.md` S1.9.
- **No visual distinction between separate `G1` runs** — if a file had
  multiple travel-separated print regions, they'd currently all render as
  one uniform-colored mesh. Could color-code by feed rate (`F`) or by
  Z-height/layer.
- **No dynamic file loading** — the path is the hardcoded `GCODE_DIR`/
  `GCODE_FILE` constants (`assets/models/gcode/model.gcode`), matching
  every other asset loader in this project (no file dialog anywhere in
  the codebase). `GCODE_FILE` is a **fixed** name, not hand-edited per
  session — Cura is configured to always export to `model.gcode`, so a
  fresh export just overwrites the previous one at the same path. A real
  file picker would replace the constant with a path held in
  `gui_panel.py`'s view state.
- **No live streaming through the arm (yet)** — this only previews the
  path. This parser's waypoint list is the planned input to Stage 5.3
  (`tutorials/Stage5_README.md`), which will drive
  the arm through it via `solve_ik_tcp` — not started yet, but no longer
  just a vague future idea.
- **No malformed-line reporting** — unparseable or unsupported lines are
  silently skipped, not logged or surfaced as an error.

## How to tune it

Module-level constants in `geometry_backend.py`:

| Constant | Effect |
|---|---|
| `GCODE_DIR` / `GCODE_FILE` | Path to the loaded G-code file — `assets/models/gcode/model.gcode`, a fixed name Cura always exports to. |
| `GCODE_COLOR` | RGB color of the bead mesh. |
| `FILAMENT_DIAMETER_MM` | Assumed filament diameter (mm) used to convert extruded `E` into a bead volume — see `settled.md` S1.9. |

## Code anchors

- `geometry_backend.py`: `parse_gcode()`, `load_gcode()`,
  `_build_gcode_beads()` (including its `bead_face_prefix` return value),
  `_BEAD_BOX_FACE_TEMPLATE`, `GCODE_DIR`, `GCODE_FILE`,
  `FILAMENT_DIAMETER_MM`, `GCODE_COLOR`, `GCODE_MOVE_RE`,
  `CAP_CULL_COLINEAR_DOT_MIN`, `CAP_CULL_WIDTH_TOL_MM`.
- `gui_panel.py`: "Load G-code preview" button ("I/O Operations" section) is
  the only caller of `load_gcode()` — the Move/Reset/Load Saved Position
  buttons ("Build Plate Orientation" section) do not call it (S1.23).
- `assets/models/gcode/square_test.gcode` — the original verification
  fixture; not loaded by default (that's now `model.gcode`), but usable
  by temporarily swapping `GCODE_FILE`.
- `wiki/002_Architecture/settled.md` S1.3 — why this bypasses the Delta
  pipeline and draws only `G1` segments; S1.7 — why the parser stays
  G0/G1-only and custom (not a third-party tokenizer); S1.8 — the original
  button-triggered reload-on-plate-reposition decision, since superseded;
  S1.9 — the swept bead mesh itself, geometric layer-height derivation,
  and the assumed filament diameter; S1.10 — the transparency re-attempt,
  its measured numbers, and the scene-global Polyscope limitation blocking
  it; S1.19 — the cap-face culling added in point 9 above; S1.20 — why
  `bead_face_prefix` exists (playback registers a variable-length slice of
  `faces`, not a fixed 12-per-bead stride); S1.23 — why the plate-reposition
  auto-reload from S1.8 was removed (an explicit "Load G-code preview"
  click is required instead).
- `wiki/005_AgentMgmt/active/ctx_main/GLOSSARY.md` §3 — "G-code toolpath"
  term.
