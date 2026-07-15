---
status: active
---

# How the G-code Toolpath Preview Works

## What it is

A preview of the deposited G-code material as a **solid 3D shape**, drawn
on the build plate — the printed object as it would come off a 3D printer,
not just its centerline path. Clicking "Load G-code preview" (I/O
Operations panel) parses `assets/models/gcode/model.gcode`, the fixed
name/location every Cura export is saved to, builds a **swept rectangular
bead** for every positive-extrusion `G1` segment, maps them into world
space via the plate's `T_user_frame`, and registers them as one orange
`register_surface_mesh("G-code Print", ...)` (`settled.md` S1.11).

Each bead is a box whose **width** comes from the per-move extrusion volume
and whose **height** is the layer height, so walls and layers read as solid
material (over/under-extrusion shows as bead-width variation). `G0` travel
moves, retractions, unretractions, and non-extruding `G1` moves are tracked
for position but never become beads, so the mesh shows deposited material
rather than incidental motion. Detection is purely extrusion-based (no
`;TYPE:` filtering), so **bridge/overhang spans** — which this Cura export
emits as untagged extruding SKIN/WALL moves; it contains no `;BRIDGE`
markers — render as solid bars across the gap rather than disappearing.

The Build Plate Orientation panel's Move/Reset/Load Saved Position buttons
each reload the mesh too, so it stays in sync whenever the plate's pose
changes (`settled.md` S1.8). The heavy parse/build runs once per file and
is cached; a plate reposition only re-runs the world matmul.

During toolpath playback the shape **grows** as the nozzle traces the path
(`set_print_reveal`, below), so you watch the object build up layer by
layer.

A translucent rendering mode (`GCODE_TRANSPARENCY`) was tried on the old
curve preview, as groundwork for build-up, but was reverted: a real
multi-layer print is ~180,000 segments, and translucency over that many
overlapping structures caused a constant frame-rate regression. The opaque
bead mesh avoids that (no translucency), and the playback build-up is what
that experiment was reaching for — see the reveal throttle
(`GCODE_REVEAL_CHUNK`) for how re-upload cost is bounded.

## How it's computed

`VisContent.parse_gcode()` and `VisContent.load_gcode()`
(`geometry_backend.py`):

1. `parse_gcode(filepath)` reads the file line by line, strips comments
   (`;...` to end of line, and inline `(...)`), and tokenizes each line
   with `GCODE_MOVE_RE` (`([A-Za-z])\s*(-?\d+\.?\d*)`). Only `G0`/`G1`
   motion lines become waypoints; unsupported motion modes are skipped.
2. Position is modal: an axis word (X/Y/Z) missing from a line keeps its
   last known value. This applies whether the line is `G0` or `G1`.
3. Extrusion (`E`) and feedrate (`F`) are modal too. `M82`/`M83` switch
   absolute/relative extrusion tracking, and `G92 E...` resets the current
   extrusion value without creating a TCP waypoint.
4. Each recognized motion line appends
   `([x, y, z], is_print_move, deposit)` to the returned list.
   `is_print_move` is `True` only for `G1` moves where extrusion increases.
   `deposit` is the **per-move** extrusion `e - previous_e`, **not** the
   cumulative `E` — retraction/un-retract happen on non-motion lines that
   create no waypoint, so a cumulative difference across the preceding
   travel would fold the whole retract distance into the first bead of each
   region and blow it out to the max-width clamp.
5. `build_print_beads(waypoints, layer_height)` turns every print segment
   into a swept rectangular box (vectorised, ~0.25 s for the full print):
   - **width** `= clip((deposit · π(D/2)²) / (L · layer_height), min, max)`,
     `D = FILAMENT_DIAMETER_MM`, `L` = segment length. This is the deposited
     cross-section area divided by layer height.
   - **height** = layer height (Cura's `;Layer height:` header when present,
     else 0.1 mm), except the first layer hangs down by its actual first
     toolpath Z so it reaches the plate-local top surface at `Z=0`. Cura's
     initial layer is often thicker than the general layer height.
   - The box is oriented in the **plate-local** frame: width along the
     in-plane perpendicular to the segment, body hanging from the toolpath
     Z down by one layer height, so the bead top sits at the nozzle and the
     material is just below it. Near-vertical segments fall back to an
     arbitrary in-plane side axis.
   - Returns `(verts_local, faces, bead_end_waypoint)`; `bead_end_waypoint`
     records the waypoint index each bead completes at, for playback reveal.
6. `_ensure_print_beads()` caches those local arrays by file+mtime, so the
   parse/build runs once per file.
7. `load_gcode()` places the cached local verts into world with a single
   homogeneous multiply, `T_user_frame @ [x,y,z,1]^T` (not the per-frame
   Delta pipeline), and registers them via
   `ps.register_surface_mesh("G-code Print", ...)`. Same-name
   re-registration replaces the prior mesh. Because only the matmul re-runs
   on a plate move (not the parse/build), repositioning stays responsive.
8. **Progressive reveal:** `set_print_reveal(waypoint_index)` shows only the
   beads deposited up to that playback index by re-registering a growing
   **prefix** (`verts[:n*8]`, `faces[:n*12]`) of the cached world mesh —
   `n = searchsorted(bead_end_waypoint, waypoint_index)`. It is throttled by
   `GCODE_REVEAL_CHUNK` (only re-upload once that many new beads appear) so
   the near-complete ~1.0M-vertex mesh isn't re-sent every frame; the empty
   and fully-complete endpoints are always honored exactly. `Run` reveals
   from the current index (starts empty), `Reset` empties it, and
   `advance_toolpath_playback` grows it.
9. `load_gcode()` is called from "Load G-code preview", from `Run`, and from
   the Build Plate Orientation panel's Move/Reset/Load Saved Position
   buttons, so repositioning the plate re-transforms the mesh against the
   new `T_user_frame`. It no-ops if the G-code file doesn't exist yet.

## Current scope and limitations

This is still a G0/G1-only line-segment parser, not a general G-code
interpreter, by settled decision S1.7. It is the project's accepted
general-purpose G-code loader for Cura exports.

- No arc support (`G2`/`G3`) or relative XYZ positioning (`G91`). If ever
  revisited, arcs would need interpolation and `G91` would need XYZ delta
  accumulation.
- No unit switching (`G20`/`G21`). All coordinates are assumed to already
  be millimeters, matching Cura's default output.
- No visual distinction between separate printed runs. Multiple
  travel-separated print regions render as one uniform-colored mesh.
  Future color-coding could use feed rate (`F`), Z-height/layer, or the
  `;TYPE:` marker.
- Bridges/overhangs render **as solid material but are not classified**.
  Cura exports inspected so far carry no `;BRIDGE` markers, so deposited
  material is inferred from extrusion state: every positive-extrusion span
  becomes a bead (so a bridge across a hole shows as a solid bar);
  non-extruding moves across gaps do not. Bridge beads are genuinely thinner
  (low `E` over a long span) — the `GCODE_BEAD_MIN_WIDTH_MM` clamp is kept
  loose so they stay visible. Distinct bridge *highlighting* would still need
  a geometry/support heuristic or `;BRIDGE` markers.
- Rectangular (flat-topped) bead cross-section only — no rounded/stadium
  profile, and beads are independent boxes (not welded at segment joints),
  so tight corners can show small overlaps/gaps. Fine at print scale.
- Bead width assumes `FILAMENT_DIAMETER_MM` (1.75 mm) and 100% flow; a
  different filament diameter or flow-comp would scale every width.
- No dynamic file loading. The path is the hardcoded `GCODE_DIR`/
  `GCODE_FILE` constants (`assets/models/gcode/model.gcode`), matching
  every other asset loader in this project.
- No malformed-line reporting. Unparseable or unsupported lines are
  silently skipped, not logged or surfaced as an error.

## How to tune it

Module-level constants in `geometry_backend.py`:

| Constant | Effect |
|---|---|
| `GCODE_DIR` / `GCODE_FILE` | Path to the loaded G-code file: `assets/models/gcode/model.gcode`, a fixed name Cura always exports to. |
| `FILAMENT_DIAMETER_MM` | Filament diameter (1.75 mm) used to convert extrusion `E` into deposited volume, hence bead width. |
| `GCODE_BEAD_MIN_WIDTH_MM` / `GCODE_BEAD_MAX_WIDTH_MM` | Clamp on derived bead width. `MIN` is kept loose so thin bridge beads stay visible; `MAX` guards against priming blobs / degenerate short segments. |
| `GCODE_DEFAULT_LAYER_HEIGHT_MM` | Fallback bead height/offset if the Cura header lacks `;Layer height:`. |
| `GCODE_REVEAL_CHUNK` | Playback reveal throttle: minimum new beads before the growing prefix mesh is re-uploaded. Raise it if playback stutters on the full print. |
| `GCODE_COLOR` | RGB color of the deposited-bead mesh. |

## Code anchors

- `geometry_backend.py`: `parse_gcode()`, `build_print_beads()`,
  `_ensure_print_beads()`, `load_gcode()`, `set_print_reveal()`,
  `gcode_layer_height_mm()`; hooks in `reset_toolpath_playback()` /
  `advance_toolpath_playback()`. Constants `GCODE_DIR`, `GCODE_FILE`,
  `FILAMENT_DIAMETER_MM`, `GCODE_BEAD_MIN_WIDTH_MM`,
  `GCODE_BEAD_MAX_WIDTH_MM`, `GCODE_DEFAULT_LAYER_HEIGHT_MM`,
  `GCODE_REVEAL_CHUNK`, `GCODE_COLOR`, `GCODE_MOVE_RE`.
- `gui_panel.py`: "Load G-code preview" and `Run` buttons ("I/O
  Operations" section); Move/Reset/Load Saved Position buttons ("Build
  Plate Orientation" section) also call `load_gcode()`.
- `assets/models/gcode/square_test.gcode`: the original verification
  fixture; not loaded by default.
- `wiki/002_Architecture/settled.md` S1.3, S1.7, S1.8, S1.10, and S1.11.
- `wiki/005_AgentMgmt/active/ctx_main/GLOSSARY.md` section 3:
  "G-code toolpath" and "Deposited-bead mesh" terms.
