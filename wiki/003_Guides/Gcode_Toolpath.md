---
status: active
---

# How the G-code Toolpath Preview Works

## What it is

A static preview curve of a parsed G-code file's path, drawn on the build
plate. Clicking "Load G-code preview" (I/O Operations panel) parses
`assets/models/gcode/model.gcode` — the fixed name/location every Cura
export is saved to — maps its points into world space via the plate's
`T_user_frame`, and registers an orange curve network showing only the
`G1` (feed) segments — `G0` (travel) moves are tracked for position but
not drawn, so the curve shows the printed/cut path, not incidental
repositioning. The Build Plate Orientation panel's Move/Reset/Load Saved
Position buttons each reload this curve too, so it stays in sync
whenever the plate's pose changes (`settled.md` S1.8).

A translucent rendering mode (`GCODE_TRANSPARENCY`) was tried, as
groundwork for an eventual layer-by-layer build-up preview, but was
**reverted** — a real multi-layer print's toolpath is on the order of
~180,000 `G0`/`G1` segments, and rendering that many overlapping
translucent segments caused a constant (not just click-triggered) frame
rate regression. The curve is opaque again; revisit translucency only
alongside a plan for the segment-count/rendering-cost problem itself
(e.g. decimation or a cheaper render path), not on its own.

## How it's computed

`VisContent.parse_gcode()` and `VisContent.load_gcode()`
(`geometry_backend.py`):

1. `parse_gcode(filepath)` reads the file line by line, strips comments
   (`;...` to end of line, and inline `(...)`), and tokenizes each line
   with `GCODE_MOVE_RE` (`([A-Za-z])\s*(-?\d+\.?\d*)`). Only `G0`/`G1`
   lines are kept; any other G/M code, or a line with no G-word, is
   skipped.
2. Position is **modal**: an axis word (X/Y/Z) missing from a line keeps
   its last known value, matching standard G-code semantics — only the
   very first line defaults missing axes to 0. This applies whether the
   line is `G0` or `G1`.
3. Each recognized line appends `([x, y, z], is_feed_move)` to the
   returned list, where `is_feed_move` is `True` only for `G1`.
4. `load_gcode()` converts the parsed points (plate-local mm) to world
   coordinates with a single homogeneous multiply,
   `T_user_frame @ [x, y, z, 1]^T` — not the per-frame Delta pipeline,
   since the toolpath is static workpiece geometry with no joints (see
   `settled.md` S1.3, same reasoning as the plate mesh itself, S1.2).
5. Edges are built only between consecutive points where the destination
   point's move was `G1` — so a `G0` travel move still contributes a node
   (it's the anchor the next `G1` edge starts from) but is never itself
   drawn as a line. This is why the fixture's opening `G0 X20 Y20 Z0`
   produces no visible line from the plate origin to the square's start
   corner.
6. The curve is registered via `ps.register_curve_network(...)`,
   following the exact pattern used for the TCP trajectory
   (`TCP_Trajectory.md`) — same-name re-registration replaces the prior
   curve, no explicit clear needed.
7. `load_gcode()` is called not just from the "Load G-code preview" button but
   also from the Build Plate Orientation panel's Move/Reset/Load Saved
   Position buttons (`gui_panel.py`), so repositioning the plate
   re-transforms the curve against the new `T_user_frame` instead of
   leaving it stale (`settled.md` S1.8). It no-ops if the G-code file
   doesn't exist yet, since those buttons are reachable before any G-code
   has ever been loaded.

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
- **No visual distinction between separate `G1` runs** — if a file had
  multiple travel-separated print regions, they'd currently all render as
  one uniform-colored curve. Could color-code by feed rate (`F`) or by
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
| `GCODE_RADIUS_MM` | Toolpath curve thickness, world units (mm) — kept distinct from `TRAJECTORY_RADIUS_MM` so the two curves don't visually merge. |
| `GCODE_COLOR` | RGB color of the toolpath curve. |

## Code anchors

- `geometry_backend.py`: `parse_gcode()`, `load_gcode()`, `GCODE_DIR`,
  `GCODE_FILE`, `GCODE_RADIUS_MM`, `GCODE_COLOR`, `GCODE_MOVE_RE`.
- `gui_panel.py`: "Load G-code preview" button ("I/O Operations" section); Move/
  Reset/Load Saved Position buttons ("Build Plate Orientation" section)
  also call `load_gcode()`.
- `assets/models/gcode/square_test.gcode` — the original verification
  fixture; not loaded by default (that's now `model.gcode`), but usable
  by temporarily swapping `GCODE_FILE`.
- `wiki/002_Architecture/settled.md` S1.3 — why this bypasses the Delta
  pipeline and draws only `G1` segments; S1.7 — why the parser stays
  G0/G1-only and custom (not a third-party tokenizer); S1.8 — why the
  curve reloads on plate reposition via a button-triggered call, not a
  per-frame pipeline.
- `wiki/005_AgentMgmt/active/ctx_main/GLOSSARY.md` §3 — "G-code toolpath"
  term.
