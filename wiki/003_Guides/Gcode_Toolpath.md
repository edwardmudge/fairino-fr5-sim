---
status: active
---

# How the G-code Toolpath Preview Works

## What it is

A static preview curve of a parsed G-code file's path, drawn on the build
plate. Clicking "Load G-code" (I/O Operations panel) parses
`assets/gcode/square_test.gcode`, maps its points into world space via the
plate's `T_user_frame`, and registers an orange curve network showing only
the `G1` (feed) segments — `G0` (travel) moves are tracked for position
but not drawn, so the curve shows the printed/cut path, not incidental
repositioning.

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
6. The curve is registered once via `ps.register_curve_network(...)`,
   following the exact pattern used for the TCP trajectory
   (`TCP_Trajectory.md`) — same-name re-registration on a second "Load
   G-code" click replaces the prior curve, no explicit clear needed.

## Current scope and limitations

This is deliberately a minimal G0/G1 line-segment parser, not a general
G-code interpreter — verified against a single square test fixture. Known
gaps, and where each would need to hook in if extended:

- **No arc support (`G2`/`G3`)** — would need to interpolate each arc into
  a short chain of line segments before appending to the waypoint list in
  `parse_gcode()`.
- **No unit switching (`G20`/`G21`)** — all coordinates are assumed to
  already be millimeters.
- **No relative positioning (`G90`/`G91`)** — all coordinates are assumed
  absolute; a relative mode would mean accumulating `x, y, z` as deltas
  instead of overwriting them.
- **No visual distinction between separate `G1` runs** — if a file had
  multiple travel-separated print regions, they'd currently all render as
  one uniform-colored curve. Could color-code by feed rate (`F`) or by
  Z-height/layer.
- **No dynamic file loading** — the path is the hardcoded `GCODE_DIR`/
  `GCODE_FILE` constants, matching every other asset loader in this
  project (no file dialog anywhere in the codebase). A real file picker
  would replace the constant with a path held in `gui_panel.py`'s view
  state.
- **No live streaming through the arm** — this only previews the path;
  once IK exists (Stage 4), this parser's waypoint list is the natural
  input to a "drive the arm through this G-code" feature.
- **No malformed-line reporting** — unparseable or unsupported lines are
  silently skipped, not logged or surfaced as an error.

## How to tune it

Module-level constants in `geometry_backend.py`:

| Constant | Effect |
|---|---|
| `GCODE_DIR` / `GCODE_FILE` | Path to the loaded G-code file. |
| `GCODE_RADIUS_MM` | Toolpath curve thickness, world units (mm) — kept distinct from `TRAJECTORY_RADIUS_MM` so the two curves don't visually merge. |
| `GCODE_COLOR` | RGB color of the toolpath curve. |

## Code anchors

- `geometry_backend.py`: `parse_gcode()`, `load_gcode()`, `GCODE_DIR`,
  `GCODE_FILE`, `GCODE_RADIUS_MM`, `GCODE_COLOR`, `GCODE_MOVE_RE`.
- `gui_panel.py`: "Load G-code" button, "I/O Operations" section.
- `assets/gcode/square_test.gcode` — the verification fixture.
- `wiki/002_Architecture/settled.md` S1.3 — why this bypasses the Delta
  pipeline and draws only `G1` segments.
- `wiki/005_AgentMgmt/active/ctx_main/GLOSSARY.md` §3 — "G-code toolpath"
  term.
