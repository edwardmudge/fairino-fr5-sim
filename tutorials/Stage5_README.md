# FR5 Stage 5 — Flat-Bed Printing (2D & Multi-Layer)

Build on the finished FK/IK kit to turn the arm into a build-plate
printer simulator: position a real build plate, load a real Cura-sliced
G-code toolpath, preview it as the solid printed object, and drive the arm
through the print while the shape builds up.

## What You Get

- Everything from the FK/IK kit (`FR5_FK_Kit_README.md`) — FK, mesh
  rendering, TCP, and analytical IK are all done.
- Build plate mesh + User Frame — `assets/buildPlate/`
- A working custom G0/G1 G-code parser — `geometry_backend.py`
- Reference guides — `../wiki/003_Guides/BuildPlate_UserFrame.md`,
  `../wiki/003_Guides/Gcode_Toolpath.md`
- Decisions on record — `../wiki/002_Architecture/settled.md` S1.2, S1.3,
  S1.6, S1.7, S1.8

Same environment as the FK/IK kit — see `FR5_FK_Kit_README.md`.

## What You Build

A flat build plate you can position and orient, a real G-code print
loaded on top of it as a solid bead mesh, and the arm actually tracing
that path while the shape builds up.

## Roadmap

### 5.1 — Build-Plate Position & Orientation ✅ done

**Goal:** Move and tilt the build plate instead of it being fixed.

1. `T_user_frame` is a full 4x4 pose (position + rotation), built from
   `load_build_plate(position_mm, rpy_deg)` — **done**
2. Rotation uses the XYZ fixed-angle convention — the same one IK's
   target RPY already uses — **done**
3. "Build Plate Orientation" panel: Move / Reset / Save Position / Load
   Saved Position — **done**
4. Plate color is explicit cool gray, visually distinct from the orange
   print — **done**
5. `position_mm` marks where the plate rests; the print surface is
   lifted by the measured 0.75mm plate thickness — **done**
   (`PLATE_THICKNESS_MM`, applied to both the plate mesh and the G-code
   waypoints before the `T_user_frame` transform)

**Verify:** Enter a position + RPY and click Move — the plate and its
User Frame triad move together. Click Reset — it returns to the default
resting position.

Full record: `settled.md` S1.6, `BuildPlate_UserFrame.md`.

### 5.2 — G-Code Toolpath: Parse, Load, Preview ✅ done

**Goal:** Load a real Cura-exported `.gcode` file and preview it as the
printed 3D shape.

1. Custom G0/G1-only parser — no external dependency (`settled.md` S1.7)
   — **done**
2. Fixed load path, `assets/models/planar/gcode/model.gcode`, overwritten
   by each new Cura export — **done**
3. Deposited material is drawn as a swept **bead surface mesh** — width
   from the extrusion `E`, height from layer height — so it reads as a
   solid printed object, bridges/overhangs included, not a thin wire —
   **done** (`load_gcode()`, fully vectorised box-per-segment
   construction; see `settled.md` S1.9, `Gcode_Toolpath.md`)
4. The preview does not auto-reload when the plate moves — an explicit
   "Load G-code preview" click is required to refresh it (`settled.md`
   S1.23) — **done**
5. First-layer beads hang down by the actual first-layer Z height, so
   they reach the plate-local top surface — **done** (bead height is
   derived from the actual printed Z sequence, not a parsed slicer
   comment, so the first layer's band always starts at the plate
   surface — see `settled.md` S1.9)

**Verify:** Load a real multi-layer print (e.g. a benchy) — it renders as
a solid extruded shape sitting flush on the plate, not a wireframe curve,
matching the sliced toolpath. Move the plate — the mesh stays put until
"Load G-code preview" is clicked again (see 5.11), then it jumps to match
the new pose.

Full record: `settled.md` S1.7/S1.8/S1.9, `Gcode_Toolpath.md`.

### Toolpath Execution: Drive the Arm Through the Print (5.3–5.8)

Take the loaded toolpath — a flat shape or a full multi-layer print — and
actually move the FR5 through it via IK, not just preview it. Build it in
these six small pieces, verifying each before the next. Together they're the
template Stage 6 (curved surfaces) reuses.

### 5.3 — Matrix IK Target with a Reference Pose ✅ done

**Goal:** Give IK a way to take a fixed orientation and a reference pose.

1. Add `solve_ik_tcp_matrix(...)` — TCP orientation as a rotation matrix,
   plus an explicit reference pose to rank branches against
2. Make the existing `solve_ik_tcp` delegate to it, so nothing else changes
3. This lets the toolpath solver skip a matrix→RPY→matrix round-trip and
   continue from the *previous* waypoint's pose, not the live arm

**Verify:** `solve_ik_tcp` returns the same solutions as before the
refactor — the RPY entry point is unchanged from the outside.

### 5.4 — Fix Print Orientation & Place Waypoints in the World ✅ done

**Goal:** Turn plate-local G-code into world-space 6-DOF targets.

1. Snapshot `T_user_frame[:3, :3]` once as the constant TCP rotation
   (normal to the plate — it doesn't tilt mid-print) — **done**
   (`build_toolpath_waypoints_world()`)
2. Transform every plate-local waypoint through `T_user_frame` to world —
   **done**, one waypoint per parsed G-code line, no subdivision (see
   `settled.md` S1.12 for the forward-looking caveat)
3. Execution keeps `G0` travel + `G1` feed moves for continuous
   repositioning — unlike the `G1`-only preview (`settled.md` S1.9/S1.10)
   — **done**; `solve_toolpath_ik()` chains IK continuity waypoint-to-
   waypoint via `solve_ik_tcp_matrix`'s `reference_joint_angles`, aborting
   the whole path (no partial motion) on the first unreachable/
   out-of-limits waypoint

**Verify:** A hand-checked waypoint solves to a TCP pose sitting where the
preview bead is, held flat to the plate.

Full record: `settled.md` S1.12.

### 5.5 — Ground-Clearance Branch Filter ✅ done

**Goal:** Never pick an IK branch that drives the arm through the plate.

1. For each waypoint, walk the continuity-ranked branches
2. Cheap transformed bounding-box check first
   (`moving_geometry_bbox_min_z`) to reject obvious below-ground branches
3. Exact full-mesh check (`moving_geometry_min_z`) confirms no vertex dips
   below `z=0`; take the first branch that clears

**Verify:** A branch that would punch through the plate is skipped for one
that clears; a waypoint where every branch crosses `z=0` is reported.

### 5.6 — Chunked, Pausable IK Precompute ✅ done

**Goal:** Solve the whole path before moving, without freezing the window,
and let the user drive it from the panel.

1. Precompute the full IK path up front in small per-frame batches driven
   from `render()` — `run_/step_/pause_/cancel_toolpath_ik_precompute`
2. A live progress bar advances instead of the UI hanging; the first
   unreachable / out-of-limits / all-below-ground waypoint aborts and
   reports its index, with no partial motion (`settled.md` S1.9/S1.12/S1.13)
3. Solving once up front is also the answer to "is per-waypoint IK fast
   enough at ~180k scale?" — playback replays cached joints, no per-frame
   solve
4. Run / Pause / Cancel buttons, a progress bar, and a status line wired
   into `gui_panel.py`'s "Toolpath Settings" panel, beneath the Speed
   slider — GUI wiring for the precompute itself lands in this stage, not
   deferred to 5.8 (which now covers playback controls only, once 5.7 exists)

**Verify:** The full benchy precomputes with the bar climbing after
clicking Run Precompute; Pause holds progress, clicking Run again resumes
from the same point with no lost progress; Cancel stops it and the panel
returns to its idle state.

Full record: `settled.md` S1.14.

### 5.7 — Progressive-Reveal Playback ✅ done

**Goal:** Watch the printed object build up as the arm traces the path,
driven from the panel.

1. Reuse the swept-bead preview mesh; reveal beads by growing them from a
   collapsed (invisible) position to their real one as the nozzle passes
   -- stays fully opaque, no transparency (`settled.md` S1.16)
2. `reset_toolpath_playback()` snaps to the first pose and empties the
   shape; `run_/pause_toolpath_playback()` start/resume and hold
3. `advance_toolpath_playback(step_count)` steps the cached joint path and
   grows the revealed beads to match progress, via a sorted cutoff over
   each bead's reveal-waypoint index (no per-frame scan)
4. Run / Pause / Reset wired into `gui_panel.py`, and the Speed slider is
   now the whole-steps-per-frame multiplier (1-100) roadmap 5.8 called
   for -- GUI wiring lands in this stage, not deferred

**Verify:** During playback the shape grows bead-by-bead in step with the
nozzle; Reset empties it back to nothing; dragging Speed changes pace.

Full record: `settled.md` S1.16.

### 5.8 — GUI Wiring ✅ done (covered by 5.6 and 5.7)

Playback and precompute GUI wiring are covered in 5.6 and 5.7. Real-time
`F`-feedrate pacing is out of scope -- playback replays cached joints on
a fixed step rate, not paced to the G-code's feedrate (`settled.md` S1.9).

### 5.9 — Throttled Playback Rendering ✅ done

**Goal:** Cut playback's rendering cost without dropping a single solved
waypoint from the cached path or a future export.

1. Advancing and rendering are separate: the playback index always steps
   forward every call, but the push to Polyscope (arm pose + bead reveal)
   is throttled to a fixed stride, except always forced on the final
   waypoint so playback never ends on a stale mid-stride pose. The TCP
   trail is throttled the same way, independently — **done**
2. Stride constants chosen by measuring the real render loop against a
   full-scale G-code file, not a toy fixture — **done**
3. Bead cap faces that are geometrically guaranteed hidden between
   chained, colinear, width-matched beads are culled — **done**
4. The registered print mesh grows in capacity chunks as playback
   progresses instead of registering the full mesh from frame 1 —
   **done**
5. None of the above touches the underlying solved joint path or bead
   geometry — every waypoint stays solved and every bead's true geometry
   stays available for a future export; only how often/how much gets
   pushed to the renderer is throttled — **done**

**Verify:** Playback always ends on the exact final pose, fully revealed;
frame time stays smooth on a full-scale print; fully-revealed and
partially-revealed mesh look correct from multiple angles.

Full record: `settled.md` S1.17-S1.20.

**Follow-up (2026-09-04, S1.55):** item 1's "fixed stride" and item 2's
"stride constants" no longer hold. The stride is derived per playback from
the path's own median joint motion, because a fixed *waypoint* stride is only
a fixed *visible* step when joint motion per waypoint matches — measured, it
does not: planar 0.095°/waypoint versus curved 0.90°, so the fixed 50 gave a
curved layer 45°/push (~64 arm poses for a whole print) against planar's
smooth 4.75°. Planar still derives exactly 50, so item 2's measurements stand
for that path. **S1.56/S1.57** additionally make Run re-initialize rather than
resume when the loaded precompute isn't the active source's, and remove the
unreachable "playback may start before precompute finishes" machinery.

### 5.10 — Persist the Precompute Across Sessions ✅ done

**Goal:** Cache the solved IK path and reload it instead of re-solving an
unchanged object.

1. On a completed precompute, `save_toolpath_precompute_cache()` saves the
   dense joint path to `model.precompute.npz` beside the G-code, tagged
   with a key: G-code **content hash** (SHA-256, not mtime) + full
   build-plate pose (`T_user_frame`) + a version number — **done**
   (no keyframe params — `main`'s waypoints are 1:1 with parsed G-code
   lines, settled.md S1.12; there's no keyframing concept to key on)
2. At the next **Run Precompute** press (including the first one after a
   restart), `load_toolpath_precompute_cache()` rebuilds that key from the
   current inputs; on a match, the path loads instantly instead of
   solving — **done**
3. Any change to the G-code file's contents or the plate pose changes the
   key, so a stale cache is ignored and the path is re-solved — **done**

**Verify:** Precompute once (~37 s, writes the cache); restart — the path is
ready instantly ("Loaded N waypoint(s) from cache"); move the plate and
Precompute — it re-solves, because the pose is part of the key.

Full record: `settled.md` S1.21.

### 5.11 — Invalidate Stale Precompute on Plate Move ✅ done

**Goal:** A precompute or playback run against a build plate pose that's
since changed should refuse/restart instead of silently using stale data.

1. `load_build_plate()` (`geometry_backend.py`) -- called by all three
   plate-moving buttons -- compares the new pose against
   `precompute_cache_meta["user_frame"]` (the pose captured at
   precompute-start, `settled.md` S1.21) and, on a mismatch, calls
   `cancel_toolpath_ik_precompute()` and clears the playback state, each
   with a status message explaining why -- **done**
2. The disk cache (5.10) already re-keys correctly on the next fresh
   precompute; this stage adds the equivalent in-memory/mid-session
   check -- **done**
3. Preview, precompute, and playback all require an explicit re-load/
   re-run after a plate move -- none auto-refresh (`settled.md` S1.23)
   -- **done**

**Verify:** Precompute, move the plate, run Precompute again -- it
restarts rather than resuming stale waypoints. Precompute, move the plate,
try playback Run/Reset -- refused instead of animating a mismatch.

Full record: `settled.md` S1.22, S1.23.

---

## What's Next: Beyond Stage 5

Stage 6 moves from a flat plate to a curved print surface — a different,
harder problem, using a proprietary (non-Cura) slicer. See
`Stage6_README.md`.
