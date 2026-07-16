# Settled Decisions

Architectural constraints that have been decided and should not be
re-discussed unless the listed condition changes. See
[`wiki-template/WIKI_CONSTRUCTION_GUIDE.md`](../../wiki-template/WIKI_CONSTRUCTION_GUIDE.md)
§3.2 for the format to use when adding entries:

```markdown
## S1.1 <Decision name>
**Decision:** ...
**Reason:** ...
**Non-revertible unless:** ...
**Verified on:** YYYY-MM-DD
```

## S1.1 Geometry logic lives on VisContent, not as bare module functions

**Decision:** `compute_fk`, `end_effector_position`, `load_mesh`, and
`load_data` are `VisContent` instance methods (`self.compute_fk(...)`,
etc.), not module-level functions. `dh_transform` stays a module-level
pure helper (stateless, no dependency on instance state). `MESH_DIR` and
`MESH_FILES` stay module-level constants (static path config, not instance
state).

**Reason:** `load_data` was about to start populating `self.mesh_data`
(then an unused slot set in `__init__`), and `update_transformation` (a
template placeholder, since removed along with the `self.transformation`
slot it used) already read/wrote instance state — consolidating all
stateful geometry operations behind the one object `gui_panel.py` already
holds a reference to avoids mixing bare module functions with class methods
for what is fundamentally the same backend responsibility. See the archived
previous layout at
[`_historical/2026-07-07_end_effector_position_reorg.md`](../005_AgentMgmt/_historical/2026-07-07_end_effector_position_reorg.md).

**Non-revertible unless:** the backend is split into multiple objects
(not currently planned).

**Verified on:** 2026-07-07

## S1.2 Static workpiece geometry uses a stored user-frame transform, not the Delta pipeline

**Decision:** The build plate (`assets/buildPlate/BambuLab_BuildPlate.obj`)
is placed by translating its raw vertices by `USER_FRAME_ORIGIN_MM` once,
in `load_build_plate()`, and is never touched again per-frame. The
translation is also stored as a full 4x4 homogeneous matrix,
`self.T_user_frame`, even though only its translation column is populated
today.

**Reason:** The plate has no joints, so unlike the arm links/nozzle/TCP it
does not need `apply_delta_transform`'s `Delta_i = T_0_i(q) @ inv(T_0_i(0))`
machinery — that pipeline exists specifically to re-pose meshes baked at a
non-zero joint configuration on every frame. A one-time static translation
is the simplest correct placement. Storing the transform as a 4x4 matrix
(vs. a bare xyz vector) keeps it representationally consistent with every
other frame in the codebase (`T_0_i`, `Delta_i`) and leaves room to add a
rotation later without changing the storage shape.

**Non-revertible unless:** the user frame needs a real orientation (e.g. a
3-point calibration against the physical plate), at which point
`T_user_frame` gains a rotation submatrix and the plate vertices must be
transformed with the full matrix instead of a raw vector add.

**Verified on:** 2026-07-08

## S1.3 G-code toolpath points transform via T_user_frame, not the Delta pipeline

**Decision:** `load_gcode()` parses waypoints in plate-local mm
(`parse_gcode()`) and maps them to world coordinates with a full
homogeneous multiply, `T_user_frame @ [x,y,z,1]`, once at load time — not
the per-frame Delta transform pipeline, and not a raw vector add. Only
`G1` (feed) segments are drawn as curve-network edges; `G0` (travel)
moves still update the parser's modal position but are not rendered.

**Reason:** The toolpath is static workpiece geometry fixed to the plate,
same as the plate mesh itself (S1.2) — it has no joints and never moves,
so `apply_delta_transform` doesn't apply. Unlike the plate mesh's current
raw `+ USER_FRAME_ORIGIN_MM` vector add, the toolpath uses the actual
matrix multiply because S1.2 already commits to `T_user_frame` eventually
gaining a rotation submatrix; doing the multiply now means `load_gcode()`
needs no changes when that happens, while the plate mesh's vector add
would need revisiting anyway. Drawing only `G1` segments keeps the
preview focused on the printed/cut path rather than incidental
repositioning moves.

**Non-revertible unless:** G-code loading becomes something other than a
one-time static preview (e.g. live streaming during a simulated print), at
which point it needs its own per-frame update pipeline.

**Verified on:** 2026-07-08

## S1.4 IK targets the TCP via a derived flange->TCP pose offset, not the flange directly

**Decision:** `solve_ik` solves for the flange pose (`T_0_6`), matching
Craig's worked example directly. `solve_ik_tcp` is a thin wrapper that
converts a TCP-pose target into a flange-pose target using a new cached
transform, `self.T_flange_to_tcp`, built once in `load_data()`. Its
rotation part is taken from `inv(T_zero[5])` (not assumed to be
identity/pure-translation) and its translation is `tcp_local` expressed
in that same rotated frame. See `docs/FR5_IK_Derivation.md`.

**Reason:** The project's actual purpose is a tool/printer simulator, so
IK naturally targets the TCP (nozzle tip), not the flange -- confirmed
with the user rather than assumed. No flange-relative TCP transform
existed before this; only `tcp_local`, a bare zero-pose *world* point
moved at render time via the Delta-transform trick (mesh-rendering-only
machinery, not reusable for IK). Deriving the rotation from
`inv(T_zero[5])` rather than assuming pure translation was necessary
because the FR5's zero-pose flange orientation is not identity (it comes
out to `Rot_x(-90°)` for this robot's actual mesh data) -- assuming
translation-only would have silently produced a TCP frame rotated 90°
from the one already rendered on screen (the "TCP Frame" triad).

**Non-revertible unless:** the TCP mount point changes to something with
its own independently-calibrated orientation not derivable from the
flange's zero-pose frame (e.g. a tool-changer with multiple swappable
heads), at which point `T_flange_to_tcp` would need to come from a
per-tool calibration file instead of being derived from `T_zero[5]`.

**Verified on:** 2026-07-08

## S1.5 IK multi-solution branches are filtered by joint limits, then ranked by proximity to the current pose -- all valid branches are returned, not just the closest

**Decision:** `solve_ik` returns every geometrically valid branch (up to
8, from 3 independent sign choices). `solve_ik_tcp` discards any branch
with a joint outside the caller-supplied `joint_limits`, then **sorts**
the rest by summed wrapped-angle distance to `self.current_joint_angles`
(closest first) and returns the **whole ranked list** (each entry tagged
with `raw_branch_index`, `solve_ik`'s own enumeration position) rather
than collapsing it to a single winner. `gui_panel.py`'s "Inverse
Kinematics" panel applies index 0 by default (reproducing the original
auto-pick behavior) but renders every valid branch as a `psim.RadioButton`
row so the user can select any other one, which immediately re-applies via
`update_arm`. See `docs/FR5_IK_Derivation.md` "Branch selection". Prior
single-winner behavior archived at
[`_historical/2026-07-08_ik_single_branch_autopick.md`](../005_AgentMgmt/_historical/2026-07-08_ik_single_branch_autopick.md).

**Reason:** Filtering by joint limits and ranking by proximity to
`self.current_joint_angles` is still standard practice for
redundant/multi-solution IK (explicitly stated in Craig's text) and still
reuses the arm's existing state rather than introducing a new "preferred
configuration" concept -- that part of the original decision holds.
What changed: the simulator's purpose includes letting the user inspect
and compare valid configurations, not just reach *a* pose, so
auto-collapsing to one branch and discarding the rest was throwing away
information the user explicitly wants. Branch labels use the ordinal
`raw_branch_index` plus the three sign-driven joint values (J1/J3/J5) as
a fingerprint rather than an anatomical name, since no "shoulder
left/right"-style naming has been geometrically verified for this arm.
Distinguishing "no branch survived the limit filter" from "no branch was
geometrically valid at all" is unchanged -- `solve_ik_tcp` still reports
them with distinct status messages, now alongside an empty list instead
of `None`.

**Non-revertible unless:** a specific application (e.g. G-code path
following, once built) needs continuity between consecutive solved
poses rather than just proximity to whatever the arm's current pose
happens to be -- at which point the *default* (index 0) would need to
consider the previous *target* in a path, not just the arm's live state;
this wouldn't require removing the full-list return, only changing which
entry is pre-selected.

**Verified on:** 2026-07-08

## S1.6 Build plate is fully re-posable (position + rotation), XYZ fixed-angle convention, via click-to-apply GUI buttons

**Decision:** `load_build_plate(position_mm, rpy_deg)` now takes both a
position and an `[roll, pitch, yaw]` (degrees) rotation, superseding S1.2's
translation-only description -- this is S1.2's own anticipated trigger
firing ("the user frame needs a real orientation"). Rotation uses the
**XYZ fixed-angle convention**, `R = Rz(yaw) @ Ry(pitch) @ Rx(roll)`,
reusing the existing module-level `rot_x`/`rot_y`/`rot_z` helpers -- the
same convention `solve_ik_tcp` already uses for its target RPY, not a
second one invented for the plate. The plate mesh and its "User Frame"
triad (`create_coordinate_frame`'s new optional `rotation` param) are
placed via one full homogeneous multiply of `self.T_user_frame`, replacing
the old raw `+ USER_FRAME_ORIGIN_MM` vector add. `gui_panel.py`'s "Build
Plate Orientation" panel exposes `InputFloat3` fields (not sliders --
accuracy over drag-feel) plus **Move**/**Reset** buttons: click-to-apply,
matching the "Solve IK" button pattern rather than the Forward Kinematics
panel's live-drag `changed_any` pattern. A pose can also be persisted with
**Save Position** (writes `assets/buildPlate/saved_position.json` via
`save_build_plate_position`) and recalled with **Load Saved Position**
(`load_saved_build_plate_position`) -- loading is only ever triggered by
that explicit button click, never automatically at startup, which still
always begins from `USER_FRAME_ORIGIN_MM`/zero-rotation.

**Reason:** Roadmap Stage 5.1 (2D printing) needed the plate to tilt so a
G-code toolpath's fit against the plate surface can be judged by eye --
see `tutorials/Stage5_README.md`. The user asked
to generalize position alongside rotation (one parameterized placement
function, not two separate mechanisms) and to make the applied pose
persistable across sessions once a good orientation is found by manual
exploration, without forcing every future startup to load it silently.

**Non-revertible unless:** G-code loaded before a Move/Reset/Load click
does not currently re-transform with the plate (deferred to roadmap 5.3)
-- if live toolpath-follows-plate is added, `load_build_plate()` (or its
callers) would need to also re-run `load_gcode()`'s transform.

**Verified on:** 2026-07-09

## S1.7 G-code scope is G0/G1 motion only, enforced in software, not by constraining Cura's output

**Decision:** `parse_gcode()`/`load_gcode()` (`geometry_backend.py`)
stay the project's general-purpose G-code loader -- no third-party
tokenizer (`AndyEveritt/GcodeParser` was evaluated and rejected: it's a
generic line tokenizer with no modal-position tracking, `G90`/`G91`
handling, or arc interpolation, so it would replace the ~5-line regex
tokenizing step this project already has working, at the cost of a new
dependency, without touching any of the actually hard parts). Scope is
**G0/G1 motion only**: any `G2`/`G3` (arcs), `G91` (relative
positioning), `G20` (inch units), or any other G/M/T-code is discarded
by the existing `if code not in (0, 1): continue` filter in
`parse_gcode()` -- a software-side filter, not an assumption that Cura
will never emit them. `GCODE_DIR`/`GCODE_FILE` remain hardcoded
constants, now `assets/models/gcode/model.gcode` -- a **fixed**
name/location every Cura export is saved to, not hand-edited per
session; no file-picker/text-input was added.

**Reason:** The supervisor ruled G0/G1-only sufficient for this
application. Configuring Cura to never emit `G2`/`G3` turned out to be
impractical, so rather than depend on slicer configuration, unsupported
codes are filtered in software -- which the parser already did (this
formalizes existing behavior as intentional rather than an unexamined
gap). See `wiki/003_Guides/Gcode_Toolpath.md` "Current scope and
limitations" for the full up-to-date gap list.

**Non-revertible unless:** the application later needs arcs, relative
positioning, or unit switching after all (e.g. a different slicer/export
path than Cura), at which point `parse_gcode()` would need real
interpolation/accumulation logic for whichever codes are reintroduced,
not just removing the discard.

**Verified on:** 2026-07-09

## S1.8 G-code toolpath re-transforms on plate reposition via a button-triggered reload, not a per-frame pipeline

**Decision:** `gui_panel.py`'s Build Plate Orientation panel now calls
`self.content.load_gcode()` again immediately after each of its
**Move**, **Reset**, and **Load Saved Position** buttons (all three
change `self.T_user_frame` via `load_build_plate()`). `load_gcode()`
re-parses the file from disk and re-registers the curve against the
current `T_user_frame`, so the already-drawn toolpath jumps to match the
plate's new pose without a separate "Load G-code" click. To make this
safe to call unconditionally (these buttons are reachable before any
G-code has ever been loaded), `load_gcode()` now no-ops if
`GCODE_DIR`/`GCODE_FILE` doesn't exist, instead of letting
`parse_gcode()`'s `open()` raise `FileNotFoundError`.

**Reason:** This is the gap S1.3's "Non-revertible unless" clause and
the roadmap's Stage 5.3 flagged in advance: a toolpath loaded once,
transformed once, doesn't follow the plate if it's repositioned
afterward. Of the two fixes that section anticipated (re-run the
transform on pose change, or move the curve onto a full per-frame update
pipeline), the button-triggered re-run is the one actually needed here —
the plate only ever changes pose on an explicit click (Move/Reset/Load),
never continuously, so there's no live-dragging case that would require
a real per-frame pipeline.

**Non-revertible unless:** plate repositioning becomes continuous/live
(e.g. a live-drag slider is added back for RPY/position, or live
streaming through the arm is built), at which point the toolpath would
need the per-frame pipeline S1.3 originally deferred, not just another
reload call.

**Verified on:** 2026-07-09

## S1.9 G-code preview renders as a swept bead surface mesh; bead height comes from the printed Z sequence, not parsed slicer metadata; bead width assumes a fixed filament diameter

**Decision:** `load_gcode()` (`geometry_backend.py`) now builds a solid
**bead mesh** (a box per extruding segment, `ps.register_surface_mesh`)
in place of the earlier `ps.register_curve_network` wireframe, satisfying
`Stage5_README.md` 5.2 items 3 and 5. Three sub-decisions:

1. **Bead height/first-layer band is derived from the actual printed Z
   sequence, not `model.gcode`'s `;Layer height: 0.1` header comment.**
   A running layer floor advances only on segments that actually extrude
   (`is_feed` and `E` delta > 0); it starts at 0 (the plate surface) and
   only moves up when a print segment's destination Z exceeds the
   previous print Z. This was necessary, not just a style preference: the
   real `model.gcode`'s own header layer-height doesn't describe its
   first layer (measured first print Z is 0.3mm, not 0.1mm — a thicker
   first layer for bed adhesion, standard slicer practice), and its
   startup sequence includes a non-extruding `G1 Z15.0 ... ;Prime the
   extruder` clearance move before the first real layer — a naive
   "previous distinct Z becomes the new floor" tracker (not filtered by
   whether the segment actually deposits material) gets corrupted by that
   Z=15 excursion. Restricting floor updates to extruding segments only
   sidesteps this without needing to special-case the startup sequence.
2. **Bead width assumes `FILAMENT_DIAMETER_MM = 1.75`** (the standard FDM
   default), converting extruded `E` into a volume via the standard
   slicer-viewer formula (`E delta x filament cross-section / (segment
   length x bead height)`). This is a documented assumption, not parsed
   metadata — `model.gcode`'s header has no filament/nozzle-diameter
   comment to read instead.
3. **Mesh construction is fully vectorised** (box corners/faces built via
   numpy array broadcasting across all segments at once, not a
   per-segment Python loop or `trimesh.creation.box` + concatenate),
   given a real multi-layer print is on the order of ~180,000 segments —
   the same scale that caused the earlier translucent-curve attempt
   (`Gcode_Toolpath.md`) to be reverted for frame-cost reasons.

**Reason:** Roadmap items 3 and 5 explicitly called for a solid,
first-layer-aware bead mesh instead of a thin wire. The thin curve
(`GCODE_RADIUS_MM = 1.5mm` fixed tube radius) visually clipped through
the plate at the first layer regardless of correct positioning, since a
1.5mm-radius tube dips well below a first layer only ~0.3mm above the
plate surface — a rendering artifact of representing deposited material
as a round wire, not a position bug (see S1.2's `PLATE_THICKNESS_MM`,
which remains correct and is reused here for the same plate-surface
offset).

**Non-revertible unless:** a future need for adaptive/per-layer filament
diameter (e.g. multi-material prints) requires parsing real filament
metadata instead of assuming one constant, or the segment-count/rendering
cost of the bead mesh itself becomes a problem at larger scale (in which
case decimation or LOD, not reverting to a wireframe, would be the fix).

**Verified on:** 2026-07-16

## S1.10 G-code print mesh transparency removed -- measured, not assumed, and blocked by a Polyscope limitation, not a performance one alone

**Decision:** The transparency re-attempt (`GCODE_TRANSPARENT`,
`GCODE_TRANSPARENCY_ALPHA`, and the `ps.set_transparency_mode(...)` +
`handle.set_transparency(...)` call in `load_gcode()`) was removed from
`geometry_backend.py` entirely after measuring it, rather than left in as
a default-off toggle -- see Reason.

**Reason:** This was a re-attempt at the translucency groundwork
mentioned in S1.9/`Gcode_Toolpath.md`, on the hypothesis that the old
frame-rate regression was specific to the old curve-network
representation and might not recur on the new bead mesh. Measured
directly on the real ~127,000-bead benchy (`screenshot_to_buffer()`
timing, since `frame_tick()` alone doesn't force an actual render in this
harness):

| Mode | Mean frame time | ~fps |
|---|---|---|
| Opaque (current default) | ~28ms | ~35 |
| `set_transparency_mode("pretty")` | ~69ms | ~14 |
| `set_transparency_mode("simple")` | ~28ms | ~35 |

Two findings, not one:
1. **The old regression is confirmed to be specific to `"pretty"` mode**
   (multi-pass order-independent transparency) — `"simple"` mode
   (single-pass blend) shows no measurable cost even at this segment
   count, on either representation.
2. **`ps.set_transparency_mode()` is a scene-global renderer switch, not
   a per-structure opt-in.** Confirmed directly: with zero transparent
   objects in the scene (every structure left at its default
   `transparency=1.0`), merely calling `set_transparency_mode("simple")`
   made every opaque structure (the arm links, the plate) render
   translucently in the output image. So there is currently no way to
   make only the G-code print mesh translucent without also ghosting the
   entire rest of the scene — an unacceptable default regardless of the
   good frame-time number.

**Non-revertible unless:** a Polyscope version or API is found that scopes
transparency mode to specific structures rather than the whole scene, or
the project decides whole-scene translucency (e.g. for a deliberate
"see-through" debug view, toggled on demand rather than default-on) is
actually desirable — at which point re-add the toggle using
`set_transparency_mode("simple")`, not `"pretty"` (measured ~2.5x slower
at this segment count, see the table above).

**Verified on:** 2026-07-16

## S1.11 IK gained a matrix-native entry point with an explicit reference pose; the RPY entry point is now a thin wrapper over it

**Decision:** `solve_ik_tcp_matrix(target_pos_mm, R_target, joint_limits,
reference_joint_angles=None)` (`geometry_backend.py`) now holds the shared
solve/filter/rank/status logic that used to live directly in
`solve_ik_tcp`. It takes the TCP orientation as a 3x3 rotation matrix
instead of RPY, and ranks branches against an explicit
`reference_joint_angles` instead of always `self.current_joint_angles` --
when omitted (`None`), it falls back to `self.current_joint_angles`,
reproducing the old behavior exactly. `solve_ik_tcp` is now a ~9-line
wrapper: convert RPY to a rotation matrix, delegate. Both functions were
verified to return bit-for-bit identical solutions (order, angles,
singular flags) for the same pose.

**Reason:** Roadmap `tutorials/Stage5_README.md` 5.3 -- a per-waypoint
toolpath driver (5.4+) needs to solve IK directly from a rotation matrix
it already holds (the plate's `T_user_frame[:3,:3]`) without a
matrix->RPY->matrix round-trip on every call, and needs to rank branches
against the *previous waypoint's* solved pose rather than the arm's live
pose, for continuity along the path. This is the exact generalization
S1.5's "Non-revertible unless" clause anticipated in advance.

**Non-revertible unless:** a future need arises to rank against something
other than a single reference joint-angle vector (e.g. a weighted
blend of the last N waypoints), at which point
`reference_joint_angles` would need to become a richer parameter than a
single `np.ndarray[6]`.

**Verified on:** 2026-07-16

## S1.12 Toolpath waypoints are 1:1 with parsed G-code lines, snapshot one constant TCP orientation, and IK solving aborts the whole path on the first failed waypoint

**Decision:** Two new `geometry_backend.py` functions implement roadmap
`Stage5_README.md` 5.4:

1. `build_toolpath_waypoints_world(gcode_points)` maps `parse_gcode()`'s
   output to world-space 1:1 -- one returned waypoint per input point,
   **no subdivision** of long segments into denser intermediate points.
   Both `G0` (travel) and `G1` (feed) points are included, unlike
   `load_gcode()`'s G1-extruding-only bead-mesh filter.
2. The constant TCP orientation (`T_user_frame[:3,:3]`) is snapshotted
   exactly once, inside `build_toolpath_waypoints_world` itself, and
   returned alongside the waypoint list rather than re-read or
   re-stored per-waypoint -- licensed by S1.6/S1.8 (the plate only
   repositions via discrete button clicks, never mid-print).
3. `solve_toolpath_ik(waypoints, R_target, joint_limits,
   reference_joint_angles)` chains `solve_ik_tcp_matrix`'s
   `reference_joint_angles` from each waypoint's top-ranked branch
   (`solutions[0][0]`) to the next, and **aborts the entire solve** at
   the first waypoint with no valid branch -- returns `([], status)`,
   reusing `solve_ik_tcp_matrix`'s own status wording verbatim (never
   inventing new text), no partial/silent motion.

**Reason:** One-waypoint-per-line keeps this stage simple (AGENTS.md
"Simplicity First") and matches the roadmap's literal wording; no
evidence yet that per-line resolution is visually or mechanically
insufficient. The orientation snapshot happening once, at path-build
time, avoids any window where position and orientation could be read
from two different plate poses, even though S1.6/S1.8 already guarantee
the plate cannot move mid-solve. Abort-on-first-failure matches roadmap
5.6's chunked-precompute contract exactly (`Stage5_README.md` 5.6 item
2: "the first unreachable / out-of-limits ... waypoint aborts and
reports its index, with no partial motion"), so this stage's behavior
does not need to change when chunking/pausing is layered on top later --
this was an explicit user requirement, not just convenience.
`solve_toolpath_ik` is the first real consumer of
`reference_joint_angles` (settled.md S1.11's stated reason for adding
it). Verified against the real ~187k-line `model.gcode`: full-scale
waypoint build completes in well under a second; a hand-checked
waypoint (flat and tilted plate) matches a manual `T_user_frame`
transform exactly; a 300-waypoint IK-chained subset FK-reproduces every
target pose to sub-micron error; a synthetic unreachable waypoint
correctly aborts with the right failing index and status text.

**Non-revertible unless:** a print needs higher path accuracy/smoothness
than one waypoint per G-code line provides (e.g. visible faceting on
long straight travel/feed moves solved as a single chord) -- at which
point `build_toolpath_waypoints_world` would need to subdivide long
segments into multiple sub-waypoints before IK solving, not just
increase point density upstream in the G-code itself.

**Verified on:** 2026-07-16

## S1.13 Ground-clearance filter checks literal world z=0, not the build plate's current surface height; the cheap bbox check proves clearance, the exact check only confirms when it's inconclusive

**Decision:** Three new `geometry_backend.py` functions implement roadmap
`Stage5_README.md` 5.5:

1. `moving_geometry_bbox_min_z(joint_angles_deg)` -- transforms each moving
   mesh's cached zero-pose bounding-box corners (`self.moving_geometry_rest_bbox_corners`,
   8 corners per mesh, cached once in `load_data()` via the new module-level
   `_bbox_corners()` helper) through that mesh's Delta transform and returns
   the minimum world z reached.
2. `moving_geometry_min_z(joint_angles_deg)` -- the same, but over every
   vertex of every moving mesh (`self.rest_verts[:7]`), for the true
   minimum z.
3. `_branch_clears_ground(joint_angles_deg)` -- returns `True` immediately
   if the bbox check is non-negative (a rigid transform of an AABB's 8
   corners always produces a convex hull enclosing the mesh's true
   transformed extent, and z is linear so its minimum is attained at a
   corner -- so a non-negative bbox result *proves* clearance); only calls
   the exact check when the bbox result is negative, since that's
   inconclusive (a rotated AABB corner can dip below ground even when the
   real mesh does not).

"Moving geometry" is `self.rest_verts[0:7]` (Robot1..Robot6 + nozzle) --
the same set `apply_delta_transform` drives, minus indices 7/8 (the TCP
point and TCP frame triad, which are visualization markers, not solid
robot geometry) and minus `Robot0` (the static base, never in `rest_verts`,
and not affected by any joint-angle branch choice).

"Ground" means literal **world z=0** -- the robot's own base-mounting
plane -- not the build plate's current top surface via `T_user_frame`.
`solve_toolpath_ik()` now walks each waypoint's continuity-ranked branch
list (settled.md S1.12) and takes the first branch `_branch_clears_ground`
accepts, aborting the whole solve (no partial motion, same contract as
S1.12) if every valid branch dips below z=0.

**Reason:** Roadmap 5.5's own wording is a literal `z=0` check, and the
user confirmed this reading explicitly when asked (the alternative --
checking against `T_user_frame`'s current Z + `PLATE_THICKNESS_MM` -- would
track the plate's real position after a reposition, but goes beyond both
the roadmap's literal spec and this stage's scope). Known simplification:
if the plate is moved away from its default z=0-resting pose (S1.6), this
filter no longer tracks the plate's actual surface height. The
bbox-proves/exact-confirms combination (rather than treating any negative
bbox result as an outright reject) was also confirmed with the user --
the mathematical guarantee only runs one direction (bbox is a lower bound
on the exact result), so a negative bbox result cannot be used to reject a
branch without risking a false rejection of one that actually clears.

**Non-revertible unless:** the build plate becomes reachable while
mid-print at a non-default pose in the same session the ground-clearance
filter needs to apply, or a future collision check needs to guard against
something other than the base mounting plane (e.g. the plate mesh itself,
or other scene geometry) -- at which point the filter would need to read
`T_user_frame` instead of assuming z=0.

**Verified on:** 2026-07-16

## S1.14 Toolpath IK precompute is chunked and driven from `render()`, with Run/Pause/Cancel state mirroring the existing playback controls one-to-one instead of the roadmap's original five-function split

**Decision:** Four new `geometry_backend.py` functions implement roadmap
`Stage5_README.md` 5.6, replacing its originally-drafted
`start_/step_/pause_/resume_/cancel_toolpath_ik_precompute` five-function
split with `run_/step_/pause_/cancel_toolpath_ik_precompute` (four):

1. `run_toolpath_ik_precompute(joint_limits, reference_joint_angles=None)`
   mirrors the GUI's playback **Run** button exactly: sets
   `precompute_running = True`, and only (re-)parses the fixed G-code path
   and resets progress counters if nothing is loaded yet
   (`precompute_waypoints is None` -- true on the first call, and again
   after `cancel_toolpath_ik_precompute()`). If a precompute is already
   loaded (i.e. paused), it just resumes stepping from `precompute_index`
   -- no re-parsing, no restart. There is no separate `resume_` function;
   `run_` does double duty for both "start fresh" and "resume", exactly as
   the existing UI's single "Run" button does for playback
   (`gui_panel.py:70-71`, which has never distinguished the two either).
2. `pause_toolpath_ik_precompute()` mirrors playback **Pause**: sets
   `precompute_running = False` only, leaving `precompute_index` and
   `precompute_joint_path` untouched.
3. `cancel_toolpath_ik_precompute()` mirrors playback **Reset**: sets
   `precompute_running = False` **and** zeroes `precompute_index`/
   `precompute_total`, discarding `precompute_waypoints`/
   `precompute_joint_path` -- the same relationship `Reset` already has to
   `playback_waypoint_index` (`gui_panel.py:80-82`).
4. `step_toolpath_ik_precompute()` -- a no-op unless `precompute_running`
   -- solves up to `PRECOMPUTE_CHUNK_SIZE = 25` waypoints per call (chosen
   from S1.13's ~0.5ms/waypoint benchmark at benchy scale, keeping each
   call to roughly a 12ms slice, comfortably under a 60fps frame budget),
   using the exact same per-waypoint logic `solve_toolpath_ik` uses
   (`solve_ik_tcp_matrix` then `_branch_clears_ground` over the ranked
   branches, S1.13) so the two never drift apart. Called unconditionally
   every frame from `gui_panel.py`'s `render()`, the same place
   `record_trajectory_point()` already runs per-frame backend work. Aborts
   the whole precompute (discarding `precompute_joint_path`, no partial
   motion -- same contract as `solve_toolpath_ik`, S1.12) at the first
   waypoint with no valid or ground-clearing branch.

GUI wiring (Run/Pause/Cancel buttons, a progress bar via
`psim.ProgressBar`, and a status line reading `precompute_status`) was
added directly in this stage, in `gui_panel.py`'s "Toolpath Settings"
panel beneath the Speed slider -- **not** deferred to roadmap 5.8 as
originally drafted (`Stage5_README.md` 5.8 is rewritten to cover playback
controls only, once 5.7 exists).

**Reason:** The user asked explicitly, across two follow-ups, first to
wire the GUI now (so the precompute is actually testable) rather than wait
for 5.8, then to make the Run/Pause/Cancel *logic* mirror the existing
playback Run/Pause/Reset buttons exactly rather than invent a separate
five-function vocabulary -- the existing UI only has three
buttons/concepts (a running flag set by Run/cleared by Pause, and a
progress counter zeroed only by Reset), so introducing a distinct
`start_`/`resume_` split on the backend would have been an asymmetry with
no corresponding UI concept, and "Simplicity First" (AGENTS.md) favors the
one `precompute_running` flag actually used everywhere else in this file
(`is_playing`) over a busier state machine. `_branch_clears_ground` reuse
(rather than re-implementing ground clearance inline) keeps the precompute
and the existing blocking `solve_toolpath_ik` from ever disagreeing about
what counts as reachable.

**Non-revertible unless:** the fixed 25-waypoint chunk size becomes too
coarse or too fine for a real frame budget once actually profiled inside
Polyscope's render loop (as opposed to the standalone benchmark this was
derived from) -- at which point it may need to become time-budgeted rather
than count-budgeted; or the playback stage (5.7) needs to distinguish
"paused mid-precompute" from "fully idle" in a way `precompute_running`
alone can't express, at which point the state would need a third value
instead of a bool.

**Verified on:** 2026-07-16

## S1.15 Precompute's Run/Pause buttons collapsed into one toggle; progress bar shows a percentage instead of a raw count

**Decision:** A GUI-presentation-only refinement on top of S1.14's
backend, requested once the 5.6 feature was visible and clickable in the
real app:

1. `gui_panel.py`'s "Toolpath Settings" panel no longer draws separate
   "Run Precompute"/"Pause Precompute" buttons side by side. One button
   now reads "Pause Precompute" and calls `pause_toolpath_ik_precompute()`
   while `self.content.precompute_running` is `True`, or "Run Precompute"
   and calls `run_toolpath_ik_precompute(JOINT_LIMITS)` otherwise --
   `precompute_running` already fully describes which action makes sense,
   so no new state was added, only which single button/handler is shown.
   "Cancel Precompute" stays its own separate button, unchanged.
2. The progress bar's overlay switched from the raw waypoint count
   (`"150/181375"`) to a percentage (`f"{fraction * 100:.0f}%"`, e.g.
   `"83%"`).
3. The in-progress status string, set in both
   `run_toolpath_ik_precompute` and `step_toolpath_ik_precompute`
   (`geometry_backend.py`), gained a `" waypoints"` suffix:
   `"Precomputing 150/181375 waypoints"`. The terminal
   `"Solved N waypoint(s)"` and abort-status strings were left alone --
   they already say "waypoint(s)".

**Reason:** User follow-up request for a tighter, more informative panel
after seeing the 5.6 controls running for real -- a single toggle button
reads more clearly than two buttons that are only ever meaningfully
different in one direction at a time, and a percentage is a more familiar
progress-bar convention than a raw fraction printed twice (once on the bar,
once in the status line below it).

**Non-revertible unless:** none identified -- pure presentation change
over already-settled backend state (S1.14), reversible by editing
`gui_panel.py` alone.

**Verified on:** 2026-07-16
