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
constants, now `assets/models/planar/gcode/model.gcode` -- a **fixed**
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

**Superseded by:** S1.23 -- the button-triggered `load_gcode()` calls
this decision added were removed; the preview no longer auto-reloads on
any of the three buttons.

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
   (`gui_panel.py`'s Run/Pause buttons, which have never distinguished
   the two either).
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

## S1.16 Progressive-reveal playback grows the opaque bead mesh via a sorted-cutoff vertex collapse, not transparency; Run/Pause/Reset GUI wiring pulled forward from 5.8

**Decision:** Four new `geometry_backend.py` functions implement roadmap
`Stage5_README.md` 5.7:

1. `_build_gcode_beads(gcode_points)` -- the bead-construction math
   extracted unchanged from `load_gcode()` into a shared instance method,
   with one addition: it now also returns `reveal_waypoint_index`, a
   `(K,)` array giving the 0-based `gcode_points` index each bead's
   segment ends at (`np.nonzero(valid)[0] + 1`, captured before the
   `valid` filter overwrites `p0`/`p1`). Strictly increasing by
   construction, since `valid` is a boolean mask over segments already in
   original line order. `load_gcode()` is now a thin wrapper: parse, call
   `_build_gcode_beads()`, register + color exactly as before -- verified
   unchanged (same bead count, same real benchy load) after the refactor.
2. `_init_toolpath_playback()` (private) -- requires a completed
   `precompute_joint_path` (`"Run Precompute first"` otherwise); re-parses
   the G-code and calls `_build_gcode_beads()` fresh (not reused from
   `load_gcode()`, which never returns `reveal_waypoint_index`); collapses
   every bead to its own first corner via
   `np.repeat(verts_world[0::8], 8, axis=0)` -- a zero-area box renders
   nothing, so beads start invisible without any transparency machinery;
   registers/replaces the same `"G-code Print"` structure `load_gcode()`
   uses (playback repurposes it rather than drawing a second overlapping
   mesh); snaps the arm to the first waypoint's pose.
3. `reset_toolpath_playback()`/`run_toolpath_playback()`/
   `pause_toolpath_playback()` mirror the existing playback Reset/Run/Pause
   button semantics exactly the way S1.14 established for precompute:
   Reset always re-initializes fully; Run initializes only if nothing's
   loaded yet, otherwise resumes from `playback_index`; Pause only clears
   the running flag.
4. `advance_toolpath_playback(step_count)` -- a no-op unless
   `playback_running`; moves the arm to
   `precompute_joint_path[new_index]`, then reveals newly-passed beads via
   a **sorted cutoff**, not a per-bead mask or scan:
   `np.searchsorted(gcode_bead_reveal_index, playback_index, side='right')`
   gives "how many beads are revealed so far" in one call, since
   `reveal_waypoint_index` is monotonic; only the newly-revealed slice
   `[old_revealed*8 : new_revealed*8]` is copied from the cached real
   positions into the working array before a single
   `update_vertex_positions` call.

GUI wiring landed in this stage rather than waiting for roadmap 5.8: the
existing (previously-dead) `gui_panel.py` Run/Pause/Reset buttons now call
`run_/pause_/reset_toolpath_playback()` directly (replacing the
UI-local-only `self.is_playing`/`self.playback_waypoint_index`, which were
set but never meaningfully read -- retired entirely, matching S1.14's
precedent of backend-owned state over UI-local shadows); the status line
reads `self.content.playback_status` instead of a hand-built
Running/Paused string. The existing Speed slider's range changed from a
placeholder `0.1-5.0` to roadmap 5.8's actual spec, a `1-100`
whole-steps-per-frame multiplier: `render()` calls
`advance_toolpath_playback(max(1, int(self.playback_speed)))`
unconditionally every frame, the same placement pattern as
`step_toolpath_ik_precompute()`.

**Reason:** The user initially asked for a transparency-based reveal
("as noted in the docs"), but neither `Stage5_README.md` 5.7 (which
literally says "grows the revealed beads") nor any other doc actually
specifies that -- confronted with S1.10's own measurement that
`ps.set_transparency_mode()` is scene-global (would ghost the arm and
plate too, not just the print, for as long as playback runs), the user
confirmed growing the opaque mesh instead. The vertex-collapse technique
was chosen over an alternative (re-registering a growing sub-mesh each
step) because Polyscope surface meshes already support cheap
same-topology `update_vertex_positions` calls (the exact mechanism
`apply_delta_transform` uses for the arm every frame) -- no new API
surface, and no per-step re-registration cost at up to ~180,000-bead
scale. The GUI-wiring-now decision mirrors the same call already made for
5.6 (S1.14): the user wants each stage testable in the real app
immediately rather than deferred to a later wiring-only stage.

**Non-revertible unless:** a future need arises to reveal beads out of
G-code line order (e.g. a re-sequenced/optimized print path), at which
point the sorted-cutoff technique (which assumes `reveal_waypoint_index`
is monotonic) would need to become a scattered-mask approach instead.

**Verified on:** 2026-07-16

## S1.17 Playback and trajectory-curve rendering are throttled by two independent fixed-constant strides, not a GUI slider

**Decision:** Roadmap `Stage5_README.md` 5.9 implemented as two separate
throttles in `geometry_backend.py`, both decoupling the *calculation* (which
must stay dense for a future export) from the *Polyscope push* (which is
purely visual and safe to decimate):

1. `advance_toolpath_playback(step_count)` -- `self.playback_index` still
   advances every call, unconditionally. The Polyscope push (`update_arm()`
   plus the bead-reveal `update_vertex_positions()` call) only fires once
   `self.playback_index - self._last_rendered_playback_index >=
   PLAYBACK_RENDER_STRIDE`, or unconditionally on the final waypoint
   (`finished`) so playback never freezes on a stale mid-stride pose. The
   bead-reveal cutoff (`np.searchsorted` over `gcode_bead_reveal_index`) is
   computed from `_last_rendered_playback_index`, not the previous frame's
   index, so beads revealed during throttled-away frames are never lost --
   they just appear in one larger batch at the next render.
2. `record_trajectory_point()` -- `self.trajectory_points.append(...)`
   fires on every accepted sample exactly as before (still gated by the
   pre-existing `TRAJECTORY_SAMPLE_INTERVAL_S` wall-clock throttle and the
   `np.allclose` no-movement dedup). Only the call to
   `_update_trajectory_curve()` -- a full `ps.register_curve_network()`
   re-registration, the actual cost center since curve networks have no
   incremental grow-node-count API -- is gated behind a new counter,
   `_trajectory_curve_sample_count`, firing every
   `TRAJECTORY_CURVE_RENDER_STRIDE` accepted samples.
3. Both strides are fixed `geometry_backend.py` module constants (default
   `5`), styled like the existing `PRECOMPUTE_CHUNK_SIZE` -- no GUI slider
   was added.

**Reason:** `advance_toolpath_playback` already used incremental
`update_vertex_positions` for bead reveal (S1.16) -- the actual full
re-registration offender was `_update_trajectory_curve()`, confirmed against
`docs/Polyscope_Quickstart.md` (no incremental API for curve networks), so
the roadmap's item 2 explicitly calls it out as the first target and a
"decimatable debug overlay" since, unlike `precompute_joint_path` and the
bead mesh's vertex/face arrays, it is never exported. The user confirmed a
fixed constant over a GUI slider: roadmap 5.9 (unlike 5.6/5.7) has no
GUI-wiring bullet, matching the project's existing precedent of only adding
GUI surface when a stage's roadmap text calls for it.

Checked with a standalone script driving `VisContent` against a real
(not mocked) Polyscope backend: a 41-waypoint fake playback path pushed to
Polyscope 8 times instead of 41 (throttled to every 5th waypoint, plus the
forced final frame), ended exactly on the full bead set and
`"Playback complete"`, and left `precompute_joint_path`'s length unaffected;
23 forced trajectory samples stayed fully dense in `trajectory_points` while
`_update_trajectory_curve()` only fired 4 times (`23 // 5`). `main.py` still
launches cleanly (OpenGL 3.3 context, no errors) after the change.

**Non-revertible unless:** profiling at real ~180,000-bead scale shows `5`
is too coarse (visible stutter) or too fine (rendering still the bottleneck)
for either stride, at which point the constants would need retuning or --
per the roadmap's original framing -- promotion to a GUI-exposed control.

**Verified on:** 2026-07-17

## S1.18 `PLAYBACK_RENDER_STRIDE` raised from 5 to 50, measured (not assumed) against the real benchy; `planar-printing-prototype` had nothing portable for rendering

**Decision:** S1.17's `5` was too conservative in practice -- the user
reported playback still felt laggy after that change. Raised
`PLAYBACK_RENDER_STRIDE` (`geometry_backend.py`) to `50`.
`TRAJECTORY_CURVE_RENDER_STRIDE` stays at `5`, unchanged -- see Reason.

**Reason:** Two things drove this, both checked rather than guessed:

1. **Branch comparison.** The user asked to check
   `planar-printing-prototype` (recalled as fast) for a portable technique.
   It isn't one: its `set_print_reveal()` re-registers a growing mesh
   *prefix* every reveal step (`ps.register_surface_mesh(...,
   verts[:n*8], faces[:n*12])`, throttled by a bead-count delta,
   `GCODE_REVEAL_CHUNK = 200`) -- exactly the "re-register a growing
   sub-mesh" approach S1.16 already rejected in favor of a fixed-size
   buffer + `update_vertex_positions`. Its own architecture doc even
   flags this as a known weak point needing a non-re-upload mechanism. Its
   trajectory-curve throttle is also worse than `main`'s (100Hz sampling,
   *no* stride on the full re-registration -- `main` already added the
   throttle it lacks). The prototype's real speed advantage is on the
   **precompute** side -- sparse keyframe IK (every 2.5mm/15° turn) +
   joint-space interpolation, plus a disk cache
   (`model.precompute.npz`) `main` completely lacks -- confirmed out of
   scope here since the user confirmed the lag is specifically in
   playback rendering, not precompute (candidate future work, roadmap
   5.10/5.11).
2. **Root-cause measurement**, using a script that loads the real
   ~187,000-line `model.gcode` and a cached keyframe-interpolated joint
   path (`assets/models/planar/gcode/model.precompute.npz`, sha256-verified
   against the current G-code + default plate pose, borrowed purely as a
   fast way to get a real dense joint path for a *rendering* benchmark --
   not evidence that keyframe interpolation was adopted). Real bead mesh:
   127,677 beads, 1,021,416 verts. `ps.screenshot_to_buffer()` per frame
   (S1.10's method -- `frame_tick()` alone doesn't force a render),
   300 frames per condition, `step_count=1` (Speed slider at its slowest,
   worst case):

   | Condition | Mean frame time | ~fps |
   |---|---|---|
   | `PLAYBACK_RENDER_STRIDE=5` (old) | 65.55ms | ~15.3 |
   | `PLAYBACK_RENDER_STRIDE=50` (new) | 36.10ms | ~27.7 |
   | Static floor (fully revealed, zero updates) | 34.15ms | ~29.3 |

   `update_vertex_positions()` re-uploads the *entire* vertex buffer every
   call, not just the changed slice (`docs/Polyscope_Quickstart.md`), so
   fewer/coarser pushes cut real GPU upload cost, not just Python-side
   work -- explaining why `50` (10x fewer pushes than `5`) very nearly
   closes the gap to the floor (36.10ms vs. 34.15ms) rather than scaling
   down only proportionally. `50` is therefore close to the practical
   ceiling for this lever: the residual ~2ms gap to the floor is
   Python-side/update overhead; the floor itself (~34ms, matching S1.10's
   ~28ms opaque measurement in the same ballpark) is raw GPU triangle-count
   draw cost that no update-frequency throttle can reduce further --
   closing it would require decimating the *displayed* bead geometry
   (S1.9's flagged future LOD/decimation fix), explicitly out of scope for
   this pass. `TRAJECTORY_CURVE_RENDER_STRIDE` was left at `5`: the
   reported lag is playback-specific, and during playback the TCP barely
   moves between throttled `update_arm()` calls, so
   `record_trajectory_point()`'s pre-existing `np.allclose` dedup already
   collapses most of that redundant work as a side effect of the
   `PLAYBACK_RENDER_STRIDE` increase.

**Non-revertible unless:** a future need for smoother-than-~28fps playback
on the full benchy arises, at which point the fix is bead-mesh
LOD/decimation (reducing displayed triangle count), not further stride
tuning -- this pass's own measurement shows stride is already within ~2ms
of its ceiling.

**Verified on:** 2026-07-17

## S1.19 Bead mesh cap faces culled at provably-hidden boundaries -- ~8% fewer triangles, no visual change

**Decision:** `_build_gcode_beads()` (`geometry_backend.py`) now drops the
two triangles of a bead's "end cap" (`_BEAD_BOX_FACE_TEMPLATE` rows 8-9) and
the next bead's "start cap" (rows 4-5) wherever a boundary between
consecutive beads satisfies all three:

1. **Index-chained** -- `reveal_waypoint_index[k+1] == reveal_waypoint_index[k] + 1`
   (no G0 travel move or non-print gap between the two segments).
2. **Colinear** -- unit-tangent dot product `>= CAP_CULL_COLINEAR_DOT_MIN`
   (`0.999`, ~2.6°). At a turn the two cap planes meet at an angle rather
   than coincide, so dropping both would expose a visible sliver.
3. **Width-matched** -- bead widths differ by `<= CAP_CULL_WIDTH_TOL_MM`
   (`0.01`mm). Width is derived per-segment from extrusion rate/length/layer
   height (S1.9) and can vary slightly even along a straight run; a
   mismatch would expose a stepped ledge on the wider bead's side wall.

`verts_world` and `reveal_waypoint_index` are untouched -- only which
triangles get built from those same vertices changes, so S1.16's
vertex-collapse playback reveal (which never touches `faces` after initial
registration) is unaffected. Fully vectorised (boolean mask over `(K-1,)`
boundaries, then `faces_full[keep_row]` fancy-indexing), matching the rest
of the function's style.

**Reason:** Requested by the user after S1.18 still felt laggy in
practice; investigated whether the remaining ~28fps ceiling on the full
benchy was inherent or fixable. It's mostly inherent (S1.18 already showed
a fully-static mesh of that size drawing at ~34ms/frame regardless of
update frequency), but cap culling is a genuine, zero-risk reduction in
what actually gets drawn -- not an approximation, since the culled faces
are geometrically proven to never be visible.

Measured on the real benchy (`assets/models/planar/gcode/model.gcode`,
127,677 beads): 107,939 boundaries (85%) are index-chained, but the
curved-hull shape means most consecutive segments turn slightly, so only
30,640 pass the full colinear+width-matched test -- **8.0% of triangles**
(122,560 of 1,532,124) dropped, confirmed by direct inspection of
`_build_gcode_beads`'s output (`1,532,124 -> 1,409,564` triangles,
`verts_world`/`reveal_waypoint_index` shapes unchanged). Initial back-of-
envelope estimate before measuring was 15-30%; actual measured savings was
meaningfully lower once the colinearity condition was checked against the
real curved geometry, not just the index-chain condition -- the gap
between "back-to-back in the G-code" and "actually straight" is the
benchy's curvature, and would be smaller on a print dominated by long
straight walls.

Re-profiled with the same `ps.screenshot_to_buffer()` method as S1.18,
300 frames, `PLAYBACK_RENDER_STRIDE=50`, `step_count=1`:

| Condition | Before (S1.18) | After (culled) |
|---|---|---|
| Static floor (fully revealed, zero updates) | 34.15ms (~29.3fps) | 29.12ms (~34.3fps) |
| Playback, stride=50 | 36.10ms (~27.7fps) | 34.36ms (~29.1fps) |

The floor improved ~15% (more than the 8% triangle reduction alone would
suggest -- some combination of fixed per-triangle GPU cost and measurement
noise, not further investigated). The playback figure improved less and
has high variance (std ~30ms) since most frames are cheap no-push draws at
the new floor, occasionally spiked by the real `update_vertex_positions`
push every `PLAYBACK_RENDER_STRIDE` waypoints.

Visually verified: fully-revealed culled mesh screenshotted from an
overview angle, a low grazing angle across the hull, an extreme close-up on
a curved hull section, and a from-below angle -- no gaps, slivers, or
missing faces in any of them.

**Non-revertible unless:** a print dominated by short, sharply-turning
segments makes the colinearity condition rarely fire (diminishing returns
for the added code complexity), or a future need arises to relax
`CAP_CULL_COLINEAR_DOT_MIN`/`CAP_CULL_WIDTH_TOL_MM` for a bigger (but
no-longer-provably-exact) reduction -- at which point this becomes a true
approximation/LOD decision, not a free win, and should be re-evaluated
against real visual inspection, not just the triangle count.

**Verified on:** 2026-07-17

## S1.20 `screenshot_to_buffer()` timing methodology retracted; playback registration right-sized to progress instead of registering the full mesh from frame 1

**Decision:** Two related corrections after the user reported playback
still felt laggy post-S1.19 and asked to compare against
`planar-printing-prototype` again:

1. **Methodology retraction.** Every frame-time number in S1.10/S1.17/
   S1.18/S1.19 was measured via `ps.screenshot_to_buffer()`, inherited from
   S1.10's own justification (`frame_tick()` "doesn't force an actual
   render in this harness"). Diagnosed directly: `screenshot_to_buffer()`
   costs ~29.5ms with *no print mesh loaded at all* (just the arm + plate)
   vs. ~38.3ms with the full 1.4M-triangle mesh -- only ~9ms attributable
   to the mesh; the rest is a mostly-fixed readback cost unrelated to scene
   complexity. Confirmed further: registering meshes from 1,000 to 127,677
   beads (0.8% to 100% of K) showed draw time flat at 28-35ms regardless of
   size when measured this way. `frame_tick()` was re-examined too --
   ~0.02-0.5ms, confirming S1.10's original read that it doesn't force a
   real render either. Neither is a valid proxy for real interactive frame
   cost.
2. **Real methodology**: drive the actual `ps.show()` render loop (real
   vsync/present, no CPU-side pixel readback) with an instrumented
   callback that logs `time.perf_counter()` deltas and calls `ps.unshow()`
   after a fixed duration. This revealed the *real* shape of the problem,
   which is nothing like what the screenshot-based numbers implied: median
   frame time ~15.8ms (**~63fps** -- smooth) with occasional spikes up to
   ~410ms. Playback isn't uniformly slow; it's smooth almost all the time,
   punctuated by periodic multi-hundred-millisecond stutters landing
   exactly at `PLAYBACK_RENDER_STRIDE` push boundaries, because
   `update_vertex_positions()` re-uploads the *entire* K*8 vertex buffer on
   every push regardless of playback progress (`main`'s mesh has been
   registered at full K size since S1.16, from frame 1 of playback).
3. **Fix**: stop registering the full K-bead mesh upfront. `_init_toolpath_
   playback()` now registers only `min(PLAYBACK_LOOKAHEAD_BEADS, K)` beads'
   worth; `advance_toolpath_playback()` keeps updating in place
   (`update_vertex_positions`, cheap) as long as revealed progress stays
   within the currently registered capacity, and only re-registers (grows
   capacity to `min(revealed + PLAYBACK_LOOKAHEAD_BEADS, K)`, or exactly
   `K` on the final frame) when progress outgrows it. This is deliberately
   close to the prototype's `set_print_reveal()` technique (register a
   slice sized to progress) rather than S1.16's "always full K, vertex-
   collapse only" -- S1.16's own rejection of the growing-submesh approach
   ("per-step re-registration cost... at ~180,000-bead scale") reasoned
   about cost near the *end* of a print without weighing it against
   steady-state draw-cost savings for most of the print's *duration*, an
   incomplete tradeoff analysis. `gcode_bead_faces` is no longer a fixed
   12-triangles-per-bead stride after S1.19's culling, so
   `_build_gcode_beads()` now also returns `bead_face_prefix`, a `(K+1,)`
   cumulative-triangle-count array, so `faces[:bead_face_prefix[n]]` slices
   correctly by bead count in O(1).

**Reason:** the user's recollection that the prototype played back
smoothly (including on what sounds like a real print, given "precompute
took longer") directly contradicted the "nothing worth porting" verdict
from the first prototype comparison (S1.18's write-up) -- prompting a
re-examination that found the real explanation wasn't a rendering
trick but a genuine difference in how much is registered at once, masked
until now by a flawed timing methodology.

Real `ps.show()`-loop measurements on the actual benchy (127,677 beads),
`PLAYBACK_RENDER_STRIDE=50`, `step_count=1`, before vs. after this fix:

| Checkpoint | Before (full K registered) | After (`PLAYBACK_LOOKAHEAD_BEADS=5000`) |
|---|---|---|
| Start (0%) | mean 20.30ms, std 31.01ms, max 410.77ms, ~49fps | mean 16.57ms, std 5.59ms, max 95.84ms, ~60fps |
| Mid (50%) | (not separately measured pre-fix) | mean 18.83ms, std 20.92ms, max 195.43ms, ~53fps |
| Near-end (95%) | (not separately measured pre-fix) | mean 20.77ms, std 32.56ms, max 277.34ms, ~48fps |

Median frame time stayed ~15.83ms (~63fps) at every checkpoint, both
before and after -- the fix doesn't change steady-state cost, it shrinks
the periodic spikes, most dramatically early/mid-playback where the
registered capacity is smallest relative to `K`. Spikes still grow as
capacity approaches `K` late in playback (expected -- the final state
genuinely needs the full mesh registered), converging toward the original
behavior by 100%. `PLAYBACK_LOOKAHEAD_BEADS` was tested at 1000 vs. 5000:
negligible difference at mid/near-end checkpoints (spike size there is
dominated by *revealed* progress, not the lookahead margin), so 5000 was
kept for fewer total re-registration events at no measured cost.

Visually verified at 50% and 100% revealed (overview + low-grazing angles):
no gaps, misalignment, or missing geometry at the capacity-growth boundary.

**Non-revertible unless:** a future profiling pass finds the remaining
near-end spikes still unacceptable, at which point the fix is reducing
displayed triangle count itself (bead-mesh LOD/decimation, S1.9's
long-flagged future option) rather than further lookahead tuning -- this
pass's checkpoint measurements show the spike near 100% progress is
already close to the original full-K behavior by construction, not a
tuning gap.

**Verified on:** 2026-07-17

**Addendum (2026-07-17):** a post-landing review found the regrowth
condition as first written (`target_capacity > self._registered_bead_
capacity`, with `target_capacity = new_revealed + PLAYBACK_LOOKAHEAD_BEADS`)
reduces to `new_revealed_now > new_revealed_at_last_growth` -- since
`PLAYBACK_LOOKAHEAD_BEADS` is constant, that's true on essentially every
push that reveals any new bead at all, not just when progress actually
outgrows the registered window. In practice this meant `register_surface_
mesh()` (the expensive path) fired on nearly every render-stride push once
printing started, growing toward `K` in lockstep with revealed progress
rather than jumping in rare `PLAYBACK_LOOKAHEAD_BEADS`-sized chunks --
`update_vertex_positions()` (the cheap path this fix was meant to make the
common case) was effectively dead code. This plausibly explains this
section's own "spikes still grow as capacity approaches K late in
playback" observation as the bug, not an inherent limit. Fixed by gating
growth on `finished or new_revealed >= self._registered_bead_capacity`
instead of comparing the shifted target against the old capacity
(`geometry_backend.py`, `advance_toolpath_playback()`). The frame-time
table above was measured against the buggy version and hasn't been
re-measured since.

## S1.21 Toolpath IK precompute persisted to disk, keyed on G-code content hash + build-plate pose

**Decision:** roadmap 5.10. `run_toolpath_ik_precompute()` now tries
`load_toolpath_precompute_cache()` before parsing the G-code; on a hit it
returns immediately with `precompute_joint_path` already populated,
skipping parsing and IK entirely. `step_toolpath_ik_precompute()` calls
the new `save_toolpath_precompute_cache()` only on its successful-completion
branch (never on an aborted or cancelled precompute), writing
`assets/models/planar/gcode/model.precompute.npz` (already covered by
`.gitignore`'s `assets/models/planar/gcode/*.npz` pattern).

The cache key (`_toolpath_cache_meta()`) is a plain dict compared by
equality, not a hash-of-hash: `{version, gcode_sha256, user_frame}`.
`gcode_sha256` is a SHA-256 of the G-code file's bytes, hashed fresh from
disk on every check -- content-based, not mtime-based, so a
hand-edited-then-reverted file with an unchanged mtime still keys
correctly. `user_frame` is the **full 4x4** `T_user_frame` (rounded to 6
decimals to absorb float noise), not just the 3x3 rotation that
`precompute_R_target` snapshots elsewhere -- waypoint XYZ positions are
baked through `T_user_frame`'s translation too, in
`build_toolpath_waypoints_world()`, so the translation has to be part of
the key or a plate move that only translates (no rotation) would false-hit.
`version` is a manual `PRECOMPUTE_CACHE_VERSION` bump point for future
schema changes. The key is captured once, at precompute-start (mirroring
`precompute_R_target`'s snapshot timing), not read live at save-time, so a
plate move mid-precompute doesn't retroactively change what gets written.

The roadmap's original draft text for 5.10 also listed "keyframe params"
as part of the key -- that refers to a keyframe-interpolation feature that
exists only on the divergent `planar-printing-prototype` branch. `main`'s
waypoints are 1:1 with parsed G-code lines (S1.12); there's no keyframing
concept on `main` to key on, so that element was dropped rather than
invented. Ground clearance (`_branch_clears_ground`) is a hardcoded
world-z=0 check with no tunable constant, so it's likewise not part of the
key.

Save is best-effort, wrapped in a bare `try/except: pass` -- a disk-write
failure (full disk, permissions) must never surface as a failure of the
precompute itself, which already succeeded in memory. Load is fail-open:
any mismatch or exception (missing file, corrupt `.npz`, schema mismatch)
is treated as a plain cache miss, falling through to the normal
parse/solve path, never raising.

This mechanism was prototyped first on the `planar-printing-prototype`
branch (different state-variable names -- `toolpath_joint_path`,
`toolpath_precompute_*` -- and an extra keyframe-params key element not
applicable here) before being ported to `main`'s actual precompute
architecture and variable names.

**Non-revertible unless:** roadmap 5.11 (invalidating a precompute or
playback run against a since-moved plate mid-session) turns out to need a
different key shape than what's captured here -- this pass only handles
the cross-session case (cache checked once, at the start of a fresh
precompute), not staleness introduced after a precompute is already
running or already loaded.

**Resolved by:** S1.22 -- 5.11 reused this key shape as-is
(`precompute_cache_meta["user_frame"]`) for the in-session check, so no
change was needed here after all.

## S1.22 In-session precompute/playback invalidation on plate move, compared against the same pose captured at precompute-start

**Decision:** roadmap 5.11. `load_build_plate()` (`geometry_backend.py`)
-- the single function all three Build Plate Orientation buttons (Move,
Reset, Load Saved Position) call -- now compares the freshly-set
`self.T_user_frame` (rounded to 6 decimals) against
`self.precompute_cache_meta["user_frame"]`, the pose snapshotted at
`run_toolpath_ik_precompute()`'s start (S1.21). On a mismatch:

1. `cancel_toolpath_ik_precompute()` is called (the same reset the
   Cancel button triggers), wiping `precompute_waypoints`/
   `precompute_joint_path`/`precompute_cache_meta`/`precompute_running`,
   followed by a clearer `precompute_status` explaining the plate moved.
2. Playback state is reset directly (`playback_running`,
   `playback_index`, `playback_total`, `gcode_bead_verts_full`) rather
   than relying on `precompute_joint_path` alone becoming empty --
   without this, a previously-initialized-but-idle playback would skip
   re-init (`run_toolpath_playback()`'s `gcode_bead_verts_full is None`
   guard) and index into the now-empty joint path with a stale
   `playback_index`, crashing instead of refusing cleanly.
3. The check is skipped entirely while `precompute_cache_meta is None`
   (no precompute has run yet this session), so moving the plate before
   ever precomputing is unchanged.

This exposed a real gap in `load_toolpath_precompute_cache()` (S1.21):
on a disk-cache hit it populated `precompute_joint_path` but never set
`precompute_cache_meta`, so a precompute loaded from disk (the common
path -- instant load at the start of a session) had no recorded pose for
this check to compare against, and a subsequent plate move would go
unnoticed. Fixed by setting `precompute_cache_meta = cached_meta` once
the cache's meta has matched the live pose (the dict that comparison
already validated, not a fresh `_toolpath_cache_meta()` call).

The comparison reads `precompute_cache_meta["user_frame"]` directly
instead of calling `_toolpath_cache_meta()` again, since that method also
re-hashes the G-code file from disk -- unnecessary I/O for a pure pose
check, and a needless failure point when no G-code is loaded yet.

**Reason:** S1.21 only checks the cache key once, at the start of a
fresh `run_toolpath_ik_precompute()` call -- a precompute or playback
already sitting in memory (completed, paused, or mid-progress) had no
mechanism to notice the plate moved out from under it, so pressing "Run
Precompute" again would silently no-op/resume against the old pose, and
playback would drive the arm through stale joint angles while the (then
still auto-reloading) preview mesh showed the new pose -- see S1.23 for
why that preview auto-reload was also removed as part of this stage.

**Verified on:** 2026-07-17

## S1.23 G-code preview no longer auto-reloads on plate move; supersedes S1.8's button-triggered reload

**Decision:** roadmap 5.11. `gui_panel.py`'s Move, Reset, and Load Saved
Position button handlers no longer call `self.content.load_gcode()`
after `load_build_plate()`. Each sets `bp_status` to prompt an explicit
"Load G-code preview" click instead.

**Reason:** S1.8's automatic reload meant the preview mesh always
tracked the plate while the precompute/playback state (S1.22's subject)
silently went stale -- an inconsistent mix of automatic and stale
behavior that made the staleness bug easy to miss (a user moving the
plate would see the preview jump correctly and reasonably assume
everything else followed). Removing the auto-reload makes plate-move
behavior consistent end-to-end: nothing -- preview, precompute, or
playback -- auto-refreshes on a plate move; every part requires an
explicit user action, each surfaced with a status message saying so.
This doesn't affect the correctness of a fresh precompute:
`run_toolpath_ik_precompute()` re-parses the G-code and reads
`self.T_user_frame` live, independent of whatever the preview mesh
currently displays.

**Non-revertible unless:** plate repositioning becomes continuous/live
(same condition S1.8 itself named), at which point a real per-frame
pipeline would be needed for the preview regardless of what triggers a
reload.

**Verified on:** 2026-07-17

## S1.24 Playback-reset factored into `_reset_toolpath_playback_state()`; fixes a real crash in the GUI's Cancel Precompute button

**Decision:** `load_build_plate()`'s invalidation branch (S1.22) reset
playback state inline (`playback_running`/`playback_index`/
`playback_total`/`gcode_bead_verts_full`) rather than through
`cancel_toolpath_ik_precompute()`, specifically because that function
alone had been found insufficient. But `cancel_toolpath_ik_precompute()`
is also what the GUI's "Cancel Precompute" button calls directly
(`gui_panel.py`) -- so that fix only ever covered the plate-move trigger,
not the button. Since `advance_toolpath_playback()` indexes
`precompute_joint_path` directly and doesn't stop being `playback_running`
just because that list got emptied, clicking "Cancel Precompute" while a
playback was active (or already finished, with a nonzero `playback_index`)
and then clicking "Run" again reached `self.precompute_joint_path[self.
playback_index]` against a now-empty list -- `IndexError`, raised from
inside the per-frame Polyscope callback. Confirmed by direct reproduction
(scripted, not just read) before this fix landed.

Fixed by extracting the playback-reset block into
`_reset_toolpath_playback_state()` (also removing the stale "G-code
Print" mesh registration, which previously stayed visible at the old pose
until the next Run/Reset) and calling it from both
`cancel_toolpath_ik_precompute()` and `load_build_plate()`'s invalidation
branch -- the latter now just calls the former, no longer duplicating the
reset inline. Also added a defensive bounds guard at the top of
`advance_toolpath_playback()` (refuse cleanly with a status message if
`playback_index >= len(precompute_joint_path)`) as a second line of
defense, since it also covers the G-code-content-changed-without-a-plate-
move gap S1.22 knowingly left open.

**Reason:** the state-reset logic existed in exactly one caller
(`load_build_plate`) when a second caller
(`cancel_toolpath_ik_precompute`, reachable directly from the GUI) needed
the same invariant and didn't get it -- a duplication-avoidance gap, not a
new design decision.

**Verified on:** 2026-07-17 -- scripted repro (drive precompute to
completion via disk cache, play back to completion, cancel precompute,
re-run playback) raised `IndexError` before this fix and refused cleanly
with `"Run Precompute first"` after it.

## S1.25 Playback starting during a still-running precompute now chases the live frontier instead of finishing on a frozen snapshot

**Decision:** `advance_toolpath_playback()` (`geometry_backend.py`) no
longer caps its advance against a stored `playback_total` -- that field is
removed entirely (`__init__`, `_reset_toolpath_playback_state()`,
`_init_toolpath_playback()`). The cap is now `len(self.precompute_joint_path)`
(the live frontier), re-read every call, so it grows as
`step_toolpath_ik_precompute()` appends more solved waypoints in the
background.

Reaching the frontier now branches on whether precompute is truly done
(`precompute_index >= precompute_total`):
- If exhausted, playback finishes for real (`"Playback complete"`,
  unchanged from before).
- If not exhausted (precompute still running, or merely paused with more
  waypoints left), playback holds at the frontier -- `playback_running`
  stays `True`, status becomes `"Waiting for precompute (n/N solved)"` --
  and resumes advancing on its own the moment the frontier grows, with no
  further user action. The Polyscope push (arm pose + bead reveal) is
  forced once when the wait begins (so the arm doesn't lag mid-stride at
  the parked pose) but not repeated on subsequent idle frames at the same
  index. The "Playing x/y" status denominator changed from the old
  snapshot to `precompute_total - 1`, the real final count, stable from
  the moment precompute starts.

No GUI or Speed-slider changes -- `gui_panel.py` already renders whatever
`playback_status` string the backend produces.

**Reason:** `playback_total` was set once, at whichever moment playback
first initialized (first "Run Toolpath" click, or "Reset"), and never
refreshed. Starting playback while precompute was still mid-way solved
against that one small snapshot forever, so playback declared
`"Playback complete"` and stopped once it worked through only the
waypoints solved *so far* -- regardless of how much more precompute later
produced. The user confirmed they want to keep starting playback early
(no requirement to block it), just fixed so it waits instead of falsely
finishing -- explicitly declining a slider-based "drop to 1x speed"
literal reading in favor of the cap naturally holding at the frontier.

The pre-existing defensive guard (`playback_index >= len(precompute_joint_path)`
-> `"Toolpath data changed -- reset playback"`, S1.24) already covers
precompute aborting mid-run, which forcibly empties `precompute_joint_path`
back to `[]` -- untouched by this change, still the correct behavior.

**Non-revertible unless:** none identified -- confined to
`geometry_backend.py`, reversible by editing that file alone.

**Verified on:** 2026-07-17 -- scripted repro (real solve path, disk cache
moved aside): started precompute on the real ~181k-waypoint `model.gcode`,
solved 3 chunks (75/181375), started playback, then drove
`advance_toolpath_playback(1000)` repeatedly. Playback held at index 74
with status `"Waiting for precompute (75/181375 solved)"` and
`playback_running` stayed `True` instead of finishing. One more precompute
chunk (100/181375) let playback advance to index 99 with no extra calls.
Ran precompute to full completion, then playback advanced to the real end
(index 181374) and reported `"Playback complete"`.

**Superseded by:** S1.26 -- the "explicitly declining a slider-based
'drop to 1x speed' literal reading" call above was reconsidered once the
user asked directly for a physical slider reaction to hitting the
frontier; S1.26 adds that as a reactive snap-down, not a re-litigation of
the frontier/exhaustion logic itself, which is unchanged.

## S1.26 Speed slider value snaps down to `PRECOMPUTE_CHUNK_SIZE` reactively, the instant playback actually hits the compute limit; the 1-100 range itself is untouched

**Decision:** `gui_panel.py`'s `render()` now checks
`self.content.playback_waiting` (a new public flag on `VisContent`, set
every `advance_toolpath_playback()` call in lockstep with the `waiting`
local S1.25 already computed) immediately after calling
`advance_toolpath_playback()`. Whenever it's `True`:
```python
self.playback_speed = min(self.playback_speed, float(PRECOMPUTE_CHUNK_SIZE))
```
The Speed slider itself (`psim.SliderFloat("Speed", ..., 1.0, 100.0)`) is
untouched -- its range stays 1-100 at all times. Only the current value
gets pulled down, and only reactively, the moment playback is actually
observed catching up to precompute's frontier -- not preemptively just
because `precompute_running` is `True`. Nothing raises the value back up
automatically once precompute finishes; `playback_waiting` simply stops
becoming `True` again, so the user is free to drag the slider back up
themselves whenever they like.

**Reason:** First draft of this proposed capping the slider's *max* to
`PRECOMPUTE_CHUNK_SIZE` for the whole time `precompute_running` was `True`
-- rejected by the user: they want the slider free to sit at 100x the
entire time precompute is running, and only want it physically pulled down
once playback actually goes past the computed waypoints and hits the
limit, snapping to whatever rate the waypoints are actually being computed
at (`PRECOMPUTE_CHUNK_SIZE`, the same per-frame unit `playback_speed`
already uses -- both `step_toolpath_ik_precompute()` and
`advance_toolpath_playback()` are called once per `render()` frame,
S1.14/S1.16). This directly builds on S1.25 (which introduced the
`waiting` state as a status message only); S1.26 is purely a GUI reaction
to that already-existing signal, promoted to a public field so
`gui_panel.py` doesn't need to re-derive it from `precompute_joint_path`
internals.

**Non-revertible unless:** none identified -- `playback_waiting` is a pure
addition to `VisContent`'s public surface, and the `gui_panel.py` change
is 4 lines; both revertible by editing those two files alone.

**Verified on:** 2026-07-17

## S1.27 Forward/Inverse Kinematics sections greyed out via `psim.BeginDisabled`/`EndDisabled` while a toolpath is playing

**Decision:** Both the Forward Kinematics (`gui_panel.py`, six `J1`-`J6`
sliders + Reset) and Inverse Kinematics (`gui_panel.py`, target
position/RPY + Solve IK + solution radio buttons) sections now wrap their
entire interactive body in `psim.BeginDisabled(self.content.playback_running)`
/ `psim.EndDisabled()`, immediately inside each `psim.TreeNode(...)` block
(the TreeNode header itself stays live -- the section can still be
expanded/collapsed, just its widgets go inert). `docs/Polyscope_Quickstart.md`
gained a short entry for this widget pair: unlike the slider/input/button
widgets already documented there, it returns `None`, not `(changed,
value)`, and disabled widgets automatically report `changed=False`, so no
extra `if playback_running: ...` guards were needed around the existing
`update_arm()` calls.

Gated specifically on `playback_running`, not `precompute_running`:
`step_toolpath_ik_precompute()` never calls `update_arm()`, so precompute
running on its own (before "Run Toolpath" is pressed) has nothing to
conflict with and both sections stay fully interactive during it. The
S1.25 "waiting for precompute" hold keeps `playback_running` `True`, so
it's correctly covered too (the arm is deliberately parked then, same as
mid-playback). Pausing (`playback_running` -> `False`) re-enables both
sections immediately.

**Reason:** Both sections call `self.content.update_arm(...)` directly
from user-driven widgets, and `advance_toolpath_playback()` also calls
`update_arm()` every frame while playback runs -- nothing previously
stopped a user from nudging an FK slider or clicking Solve IK mid-playback
and yanking the rendered arm to an unrelated pose, corrupting the running
toolpath animation. The user asked for the FK/IK sliders to have "no
effect on the simulation" while a toolpath plays; `BeginDisabled` was
chosen over ad-hoc `if playback_running: return` guards scattered across
each callback because it's the standard ImGui idiom, gives free visual
feedback (greyed-out widgets) that the section is inert, and needs no
special-casing per widget.

**Non-revertible unless:** none identified -- confined to `gui_panel.py`
(plus a docs addition), reversible by removing the four `BeginDisabled`/
`EndDisabled` calls.

**Verified on:** 2026-07-17 -- syntax-checked, then drove real `render()`
frames via `ps.frame_tick()` (same headless technique noted in S1.20) with
`playback_running` toggled `True`/`False` across frames to confirm the
`BeginDisabled`/`EndDisabled` pairing never mismatches in either branch;
no ImGui stack assertion or exception in either state.

## S1.28 Pre-v0.1 bug-fix pass: FK/IK view state, Build Plate gating, and precompute-failure cleanup

**Decision:** A code review ahead of the v0.1 commit surfaced seven real
bugs in how the GUI and backend interact; all seven are fixed:

1. `gui_panel.py`'s `render()` now resyncs `self.joint_angles` to
   `self.content.current_joint_angles` every frame `playback_running` is
   `False`. Previously the FK sliders only ever changed via direct user
   interaction, so pausing mid-playback left them showing a stale value;
   nudging one afterward would call `update_arm()` with that stale value
   and yank the arm off its correct paused pose. The resync is a no-op
   whenever the sliders themselves were the last thing to call
   `update_arm()`, so it never fights live interaction -- only the
   dead time right after playback stops.
2. Build Plate Orientation is now `BeginDisabled`-gated on
   `playback_running`, matching Forward/Inverse Kinematics (S1.27).
   Moving the plate mid-print invalidates and cancels the running
   toolpath (S1.22); it was previously the one panel not protected
   against triggering that mid-print.
3. The Forward Kinematics "Reset" button and editing the IK target
   position/RPY both now call a new `UI_Menu._clear_ik_solutions()`
   helper. Previously neither invalidated a previously-solved
   `ik_solutions` list, so a leftover radio button could apply a
   solution computed for a pose/target that no longer matches what's
   on screen.
4. The "Run Toolpath" click handler only clears IK view state when
   `self.content.playback_running` is actually `True` afterward, not
   unconditionally -- clicking it before ever running a precompute no
   longer zeroes the IK target/solutions for no reason (playback never
   started, so nothing needed clearing). The FK zeroing this handler
   used to do is gone entirely, superseded by fix 1's general resync.
5. `geometry_backend.py`: `step_toolpath_ik_precompute()`'s two failure
   branches now share a new `_abort_toolpath_ik_precompute()` helper
   (also used by `cancel_toolpath_ik_precompute()`) that resets
   `precompute_index`/`precompute_total`/`precompute_waypoints`/
   `precompute_cache_meta` and calls `_reset_toolpath_playback_state()`
   -- previously only `precompute_joint_path` was cleared on failure,
   leaving `precompute_index` stale (a misleading frozen progress bar),
   `precompute_waypoints` still set (so "Run Precompute" silently
   *resumed* into the emptied path instead of restarting), and any
   already-running playback (started mid-precompute per S1.25) with a
   stale `gcode_bead_verts_full` that made a subsequent "Run Toolpath"
   silently no-op.
6. `load_data()` now caches `self.T_zero_inv` once instead of
   `apply_delta_transform()`/`_moving_geometry_deltas()` each calling
   `np.linalg.inv()` on the same fixed zero-pose matrices every single
   frame (`apply_delta_transform` alone did it 9 times per call, 4
   redundantly on the same `T_zero[5]`). Not a bug, but the accompanying
   efficiency finding was fixed in the same pass since it touched the
   same functions.

**Reason:** All seven were found by an explicit code-review pass the user
requested ahead of the v0.1 commit, each verified against the actual code
(not just agent speculation) before being reported, then fixed and
re-verified. See the review's findings for the full failure scenarios.

**Non-revertible unless:** none identified -- confined to
`geometry_backend.py` and `gui_panel.py`.

**Verified on:** 2026-07-17 -- syntax-checked; two pre-existing scripted
repros (chase-precompute, speed-slider-snap) re-run unchanged; new
scripted repros directly exercising each fix (forced an unreachable
waypoint to trigger the precompute-failure path and confirmed
`precompute_index`/`total`/`waypoints`/`cache_meta` all reset correctly
and a follow-up `run_toolpath_ik_precompute()` does a fresh
reload/restart rather than a stale resume; confirmed `T_zero_inv` matches
a fresh `np.linalg.inv()` for all 6 entries and `apply_delta_transform`'s
output is bit-identical; drove real `render()` frames via `ps.frame_tick()`
to confirm the FK resync fires only while not playing and the new
Build Plate `BeginDisabled` stays balanced across every
playback/precompute state).

## S1.29 Curved-model placement (roadmap 6.1) uses a translation-only 4x4 centered on the plate's own local bbox, composed with T_user_frame -- same static-geometry pattern as S1.2/S1.3

**Decision:** `load_curved_model()` places the 55 toolpath-curve PLY files
(reconstructed into 70 polylines, one combined `register_curve_network` per
layer -- RX, TX) and 3 surface OBJ meshes with a `T_placement` computed from
measured bounding boxes, not fixed constants: the raw CAD-local points are
first rotated `CURVED_MODEL_ROTATE_X_DEG` (90°) about local X (the CAD "+z is
up" assumption -- see below -- turned out wrong), then the *rotated*
assembly's combined XY bbox-center (curves + all 3 surfaces) is translated
to the build plate mesh's own local XY bbox-center (derived from
`plate.bounds`, not hardcoded -- the plate mesh's local origin is a corner,
not its center), and Z is translated so the rotated assembly's lowest point
lands at `PLATE_THICKNESS_MM` in plate-local space, matching the same
resting-face/top-face compensation `load_build_plate()`/
`build_toolpath_waypoints_world()` already apply. `T_curved = T_user_frame @
T_placement` is then applied once via a new shared `transform_points()`
helper (also backfilled into the 3 pre-existing call sites that used to
inline the same two-line homogeneous-multiply pattern). Static workpiece
geometry: one-time placement, no Delta transform, same as S1.2/S1.3.

**Amendment (2026-07-19, same day):** the rotation was added after the
initial translation-only version shipped -- the user confirmed the curves/
surfaces loaded in the right place but the model itself needed rotating 90°
about the build plate's local X (red) axis, plate and arm staying put. The
sign was tested both ways (screenshotted, not guessed): `+90°` about local X
puts the printable ridge-pattern surface face-up/outward (physically
correct -- the arm has to reach it); `-90°` puts it face-down into the
plate (confirmed wrong, unprintable). The rotation is applied to the raw
local points about the CAD-local origin, before the centering/lift step, so
the existing bbox-based centering logic re-derives correctly from the
*rotated* bbox with no other changes needed.

Reconstructing the 70 polylines from the 55 files' disjoint edge-soup PLY
format needed one correction beyond the roadmap's original "chain-walk from
a degree-1 endpoint" description: 6 of the 70 pieces
(`RX_0`/`RX_22`/`RX_27`/`TX_17`/`TX_2`/`TX_6`) are closed loops with no
degree-1 node at all -- an endpoint-only walk silently drops them. Vertex
dedup rounds to 3 decimal places (0.001mm); coarser rounding under-merges
(float export noise), finer rounding fails to merge true duplicates.

**Reason:** The roadmap (`tutorials/Stage6_README.md` 6.1) calls for
translation-only placement, kept as a single cheap-to-fix 4x4, because the
CAD data's "+z is up" assumption is unverified (open question, see the
roadmap and `wiki/001_Inbox/2026-07-18_curved_surface_assets.md`). Deriving
both the plate-center target and the assembly bbox from measured geometry
(rather than hand-picked constants) means the placement stays correct if
either asset is ever re-exported with a different bbox. Verified against
the real assets: 70 total pieces (35 RX-file-groups, 35 TX-file-groups),
world min-Z lands exactly on the plate's print surface, combined bbox
diagonal ~296mm, and RX/TX median nearest-surface distance ~0.46mm/0.38mm
(asset survey measured 0.48mm/0.37mm on the same files independently).

**Non-revertible unless:** the CAD "+z up" assumption changes again (e.g. a
re-exported asset drop with a different orientation convention) --
`CURVED_MODEL_ROTATE_X_DEG` is the one constant to revisit, same
cheap-to-fix framing as before, now confirmed exercised once already.

**Verified on:** 2026-07-19 -- numeric script cross-checked piece count,
world-Z clearance, bbox diagonal, and RX/TX surface proximity against an
independent re-derivation of the same placement math (re-run after the
rotation amendment, all figures unchanged since rotation doesn't affect
size/distance); visual check via offscreen `ps.screenshot()` (arm hidden,
since its zero-pose links reach directly over the plate and occlude the
view) confirmed RX and TX curves each visibly trace a ridge pattern on
their own surface, sitting entirely above the plate with nothing poking
below it, and confirmed the rotated placement puts the printable surface
face-up with `Surface_Bot` naturally inside/beneath it.

## S1.30 RX/TX curve layers and `_verify_*` folders confirmed -- two sensor-print layers offset by print thickness (~~TX base first~~ -- print order superseded by S1.32: **RX** first), `_verify_*` is scratch

**Decision:** Per the user, the two remaining Stage 6 asset open questions
(roadmap 6.1 / `wiki/001_Inbox/2026-07-18_curved_surface_assets.md`) are
resolved:

- `_verify_hardmin/` and `_verify_interpolation/` are scratch debug dumps,
  not asset contract -- no Stage 6 code should read them. The existing
  gitignore stays as-is.
- RX and TX are two layers of the printed sensor, offset from each other to
  represent the print's thickness. ~~TX is the underlying/base layer --
  printing starts there, then RX (the offset layer) on top.~~ **← print
  order superseded 2026-07-20 by S1.32: RX prints first.** The two-layer
  reading stands and confirms the asset survey's "Working interpretation"
  section; only the order was wrong.

**Reason:** Both were flagged as open questions because the RX/TX-as-two-
passes and `_verify_*`-as-scratch readings were inferred from measured
geometry and file naming (S1.29, the asset survey), not confirmed. Roadmap
6.3 (print order) and 6.5 (`Surface_Bot.obj` as a collision body, not a
print target) were built on that inference and explicitly flagged as
subject to change if it turned out wrong.

**Non-revertible unless:** a future asset drop changes the RX/TX
relationship (e.g. a third layer, or the offset no longer represents
thickness) -- 6.3's two-pass ordering and 6.5's collision treatment of
`Surface_Bot.obj` are the code that would need to change.

**Verified on:** 2026-07-19 -- user confirmation; no new geometry
measurement needed, since the underlying geometry was already measured in
S1.29 and the asset survey.

**⚠ Caveat added 2026-07-19 (open question, decision NOT changed) --
measured geometry contradicts the recorded print order.** While
implementing 6.2, the three surfaces were measured against each other:
median nearest-surface gap `Surface_Bot` -> `Surface_RX_Offset` = **2.00mm**
-> `Surface_TX_Base` = **4.02mm**, with `TX_Base` outside `RX_Offset` for
100% of 3,000 sampled points, and each layer's own curves following its own
surface. So the physical stack is **BOT -> RX -> TX**: `RX_Offset` is the
layer sitting against the shoulder body, and `TX_Base` is 2mm outboard of
it. Taken at face value that implies **RX is laid down first**, the reverse
of this entry.

The filenames (`Surface_TX_Base`, `Surface_RX_Offset`) support this entry as
written; the geometry supports the opposite. This is left as an **open
question for the supervisor**, not a reversal -- the decision above came
from direct user confirmation, and measurement alone shouldn't silently
overturn it.

Nothing in 6.2 depends on the answer (each layer routes on its own surface
independently). It does decide roadmap **6.3**'s pass order and the sign of
**6.4**'s hover-offset surface normal, so it must be resolved before either
lands.

**✅ Caveat resolved 2026-07-20 -- see S1.32. The measured geometry was
right: RX prints first.** The supervisor gave the fabrication sequence (RX ->
manual silicone fill -> TX -> manual silicone fill), which supersedes the
print-order half of this entry. The two-layer reading above is unaffected and
stands. The caveat text is kept as the record of how the discrepancy was
found -- and as a reminder that the filenames, not the measurements, were the
misleading signal.


## S1.31 Geodesic routing (roadmap 6.2) -- two per-surface CSR graphs, hand-rolled heapq Dijkstra, one solve per unique snapped vertex, retained predecessor rows

**Decision:** Shortest paths that stay on the print surfaces are computed as
follows, and `load_curved_model()` now retains the geometry they run over:

- **Two graphs, one per print surface**, never merged. RX routes on
  `Surface_RX_Offset`, TX on `Surface_TX_Base`; the passes don't interleave
  (S1.30), so a cross-layer geodesic is meaningless and never computed. Two
  70x70 cost matrices, not one 140x140.
- **`load_curved_model()` retains world-space state** --
  `curved_pieces_world`, `curved_surface_verts_world`,
  `curved_surface_faces`, `T_curved`. Previously it discarded all of this.
  World rather than local because everything downstream (6.3 hover offsets,
  6.4 normals, 6.5 IK) works in the arm's frame. `Surface_Bot` is rendered
  but not retained -- it is a 6.5 collision body, not a print surface.
- **Hand-rolled `heapq` Dijkstra**, no `scipy` dependency added.
- **Flat CSR adjacency returned as Python lists, not numpy arrays.**
  Measured: identical algorithm and layout, only the container differs, and
  the bare Dijkstra loop runs ~139ms with numpy element indexing vs ~81ms
  with lists on Surface_TX_Base -- about 1.7x (numpy boxes a scalar per
  element access). `.tolist()` costs ~11ms once. *(Corrected 2026-07-19: an
  earlier version of this entry claimed 174ms vs 83ms / 2.1x, taken from a
  measurement that was never independently re-run. The decision stands; the
  margin is 1.7x, not 2.1x.)*
- **One Dijkstra per unique snapped vertex, not per endpoint** -- 58 unique
  for RX and 55 for TX, so 113 runs rather than the 140 the roadmap assumed.
  Duplicate endpoint rows are filled from the same solve by a mask.
- **Predecessor rows retained** ((S,V) int32, ~17MB) so any individual path
  is a walk-back via `geodesic_path_nodes()` and never a re-solve. Full
  `dist` rows are *not* retained -- each is sliced to its 70 endpoint
  columns and dropped.
- **Unreachable pairs are `inf`/`None`, reported not aborted.** Unlike
  `step_toolpath_ik_precompute()`, which aborts the whole job at its first
  bad waypoint, `step_geodesic_precompute()` has no failure branch:
  Dijkstra cannot fail, an unreachable target is data, and discarding a
  70x70 matrix over one disconnected pair would throw away the 4,830 other
  off-diagonal entries with it.
- **One whole Dijkstra source per frame** (`GEODESIC_CHUNK_SOURCES = 1`),
  pumped from `render()` like the Stage 5 precompute.
- **Geodesic state is a 2-element list** indexed by
  `GEODESIC_LAYER_RX`/`GEODESIC_LAYER_TX`, not an `_rx`/`_tx` attribute
  pair -- a divergence from the flat `precompute_*` naming next door.
- **No disk cache in 6.2.** The run is ~8.4s; 6.5 already schedules
  per-layer cache files and is the natural place for one.

**Reason:** Each was a real fork. Merged-vs-separate graphs decides whether
the cost matrices mean anything physically. World-vs-local retention decides
whether every downstream consumer pays a transform. The Python-list CSR
looks like a mistake in a numpy codebase without the measurement, and it is
what makes one-source-per-frame chunking viable at all. Per-unique-vertex
solving is a 19% saving that also guarantees duplicate rows agree with their
twin rather than being solved twice and hoping. Retaining predecessor rows
is what separates "we know the distances" from "we can emit the travel
move", which is what 6.3 actually needs. The `inf`-not-abort divergence is
the one a reader is most likely to "fix" into symmetry with its sibling.
The 2-element list shape is worth recording because it *is* inconsistent
with the neighbouring naming, deliberately.

**Also settled -- zeros in the cost matrix are real and plentiful.**
`cost[i][j] == 0.0` for `i != j` whenever two endpoints snap to the same
vertex. **Counting convention matters here** and an earlier version of this
entry mixed the two, making the arithmetic look wrong: the matrix is
symmetric, so one endpoint *pair* produces two *entries*. Measured, as
entries: 24 off-diagonal zeros in RX (16 from different pieces = 8 pairs,
8 from a piece's own two ends = 4 pieces), and 30 in TX (18 = 9 pairs, 12 =
6 pieces). Of the pieces with coincident ends, six across both layers are
the exact closed loops from S1.29; the rest are near-loops sitting ~0.001mm
apart, one dedupe quantum short of merging. All are correct, but 6.3's
greedy chain will face many tied zero-cost moves, so its tie-breaking is a
real design decision, and `cost[2p][2p+1] == 0.0` on a closed loop must not
be read as free travel to a different place.

**Also settled -- two consequences of snapping that 6.3 must handle.**

- *Endpoints that look mid-curve.* 16 RX / 18 TX endpoints lie within 0.5mm
  of another piece's line (the same abutting pieces as the different-piece
  zeros above, counted as endpoints rather than pairs). Since all 35 pieces
  of a layer render as one combined curve network, the join is invisible and
  a path terminating at a genuine piece end appears to stop halfway along a
  curve. Not a bug -- but it makes visual inspection of piece boundaries
  unreliable without rendering the endpoints explicitly.
- *Paths don't quite reach the curve.* A geodesic starts and ends at the
  snapped mesh vertex, median ~0.36mm and max 0.68mm from the actual curve
  endpoint. A travel move built straight from these nodes leaves a
  sub-millimetre gap at both ends. **6.3 (or 6.5's waypoint builder) must
  decide** whether to close it by appending the true endpoints or accept it
  as within positioning tolerance.

**Also settled -- the print surfaces are single connected components.** The
asset survey records both meshes as "not watertight", which is true but does
not imply disconnected. Measured: RX 30,284/30,284 vertices and TX
45,430/45,430 reachable, one component each, zero unreachable pairs. The
unreachability path exists so a future asset drop fails loudly, not because
the current one needs it -- do not build component-handling machinery.

**Non-revertible unless:** a future asset drop ships a surface large enough
that one Dijkstra no longer fits a frame budget (the escape hatch is a
pop-count budget inside `dijkstra_surface` plus carried partial state, which
is why one-source-per-frame is recorded as a choice rather than a default),
or a fragmented surface makes unreachable pairs common enough that
reporting-not-aborting is the wrong call. Adding `scipy` to the environment
would also reopen the hand-rolled-Dijkstra and brute-force-snap decisions.

**Verified on:** 2026-07-19 -- against the shipped assets. Both cost
matrices (70,70), symmetric to 1e-9, zero diagonal, all finite, and
elementwise >= the straight-line distance between the same snapped vertices
(a surface-constrained path can never beat the chord, so this catches a
chording bug arithmetically). Sample RX pair 36->30: geodesic 317.1mm over
254 nodes vs 218.9mm chord; every consecutive step 1.583mm or less, under
the mesh's 1.907mm max edge, with segment lengths summing exactly to the
reported cost -- so every step is a real triangle edge. A 400-point sample
along one chord dives up to 51.9mm into the shell interior while every
geodesic node sits on the surface by construction. Visually confirmed by
offscreen screenshot (arm hidden, as in S1.29): green geodesic draped over
`Surface RX Offset`, magenta chord passing through open space beneath it.
Interaction verified: build-without-model, pause-freezes, resume-without-
restart, cancel-clears-and-removes-curves, sample-before-ready,
reload-invalidates, plate-move-invalidates.

**Removed 2026-07-21 -- the sample-geodesic aid is gone.** `show_sample_geodesic`,
`_pick_sample_pair`, `_isolate_geodesic_layer`, `_restore_geodesic_isolation`,
the `_geodesic_isolation_prior` state and the sample-only constants
(`GEODESIC_CURVE_COLOR`/`GEODESIC_CHORD_COLOR`/`GEODESIC_CURVE_RADIUS_MM`/
`GEODESIC_HOST_TRANSPARENCY`) were deleted, with their two GUI widgets. 6.3's
print-order overlay and `apply_live_layer_visibility()` (S1.35) supersede both
its purposes -- seeing a geodesic on the surface, and viewing one layer at a
time. The geodesic *engine* (graphs, Dijkstra, cost matrices, `geodesic_path_nodes`)
is untouched; only the visual demo is removed. The measurements and reasoning
below stand as the 6.2 record.

**Amended 2026-07-19 -- sample-geodesic presentation (`show_sample_geodesic`).**
As first shipped, the verification aid was unusable and its own verification
was invalid. Both were fixed (before the aid was later removed, see above):

- **It rendered nothing visible.** The default layer is RX, and
  `Surface_RX_Offset` is sealed inside `Surface_TX_Base` (see S1.30's caveat
  -- TX sits a uniform 2mm outboard). The green geodesic drew *inside* the
  TX shell; the only visible artifact was the magenta comparison chord
  leaving the surface into open space, which reads as a broken geodesic.
  `show_sample_geodesic()` now **isolates its host surface** -- hides the
  other layer's surface and curves plus `Surface_Bot`, ghosts the host to
  `GEODESIC_HOST_TRANSPARENCY` -- snapshotting prior visibility into
  `_geodesic_isolation_prior` and restoring it in
  `_abort_geodesic_precompute()`, so the aid isn't a one-way trip through
  the user's view settings. The isolate step is re-entrant: a second sample
  doesn't record the already-isolated state as if it were the user's.
- **Its verification was staged.** The original screenshot was taken with
  the occluding surfaces manually disabled *by the verifying script*, so it
  confirmed the geometry while proving nothing about the feature. **A visual
  check must exercise the same code path the user triggers.** The replacement
  screenshots change nothing but hiding the arm (whose zero-pose links
  occlude the plate, as in S1.29).
- **Its default pair was unrepresentative.** It picked the farthest-apart
  endpoints -- 317mm, wrapping most of the dome, a traversal 6.3 will never
  emit; real travel moves measure median 11.32mm (RX) / 10.46mm (TX). But
  the naive correction is also wrong: the *shortest* inter-piece hop is
  2.95mm over 3 nodes at geodesic/chord ratio **1.000**, a straight line,
  because a curved surface is locally flat at that scale. `_pick_sample_pair()`
  resolves this with two modes -- `"representative"` (default) takes the
  most-curved of the hops a greedy chain would actually consider, giving
  RX 14->43 at 26.1mm and ratio 1.110; `"most_curved"` takes the highest
  ratio at any distance, RX 48->6 at 250mm and ratio 1.724. Both must mask
  out same-piece pairs **and** zero-cost pairs, of which there are 16 (RX) /
  18 (TX) entries between different pieces -- an unguarded argmin returns one
  and reconstructs a degenerate single-node path. **Both defaults are chosen
  outliers, not typical**: the median ratio is 1.08 over all ~4,744 valid
  pairs and ~1.003 over realistic hops, so a typical travel move is very
  nearly straight and neither sample should be read as showing typical
  curvature.
- The chord is drawn **only in most_curved mode**, where the comparison is
  the point; at representative scale it overlaps the geodesic and adds
  clutter. The status line now reports the **ratio**, which is what makes the
  hugs-the-surface claim checkable when the picture alone is ambiguous.

**Second amendment, 2026-07-19 -- pre-commit review.** A full audit of the
above found two defects in the fix itself, both from verifying states rather
than transitions:

- **Visibility restore clobbered state it never captured.**
  `_isolate_geodesic_layer()` snapshotted only `is_enabled()`, but the
  restore hardcoded `set_transparency(1.0)` on every surface in the
  snapshot -- including ones isolation merely enabled/disabled. A
  transparency the user had set themselves was silently reset. Now
  `(enabled, transparency)` is snapshotted and restored from the snapshot.
- **A stale chord survived a mode switch.** The chord was registered only in
  `most_curved` mode but never *removed* in `representative` mode, so
  switching modes left the previous pair's chord on screen beside an
  unrelated path -- reproducing the exact "reads as a broken geodesic"
  failure this amendment was written to eliminate. The chord is now removed
  unconditionally before the mode is consulted. The original verification
  line ("chord present in stress mode and absent in representative") tested
  each mode independently and never the transition, which is how it slipped
  through. **Verify transitions, not just states.**
- **`"stress"` renamed to `"most_curved"`** (GUI: "Most-curved pair"). The
  old name described why it was built, not what it selects.
- **Timing figures corrected** -- see the 1.7x note above, and:
  `dijkstra_surface()` as shipped costs ~50ms (RX) / ~85ms (TX), above the
  bare loop because it also allocates `prev` and converts to numpy on
  return. Full run ~8.4-9.1s. An earlier version of the guide's table quoted
  ~170ms for TX, which was a measured worst *frame*, not the solve.

**Also recorded -- geodesics can track the mesh rim.** Over 60 random
endpoint pairs a mean 18% of path nodes lie on the surface's open boundary,
and 11/60 pairs spend >20% of their nodes there. Geometrically correct --
around a dome's rim genuinely can be the shortest surface path -- but a
travel move that tracks the shell's open edge may not be physically
desirable, so **6.3 should check this when it emits real travel moves**. The
default sample pairs are unaffected.

**Also recorded -- there is no "top surface" to constrain routing to.** The
printed curves span z 2.7-159.0 against surfaces spanning z 0.9-159.0, with
matching bounding boxes on all three axes: the pattern wraps the whole dome
rather than sitting on a distinct top face. Each print surface is a single
open sheet (623 boundary edges, 0 non-manifold, ~39,900mm^2 -- not a
double-sided skin), so a path on it cannot pass through the interior.

**Verified on:** 2026-07-19 -- unstaged screenshots on both layers through
`show_sample_geodesic()` with no visibility changes but hiding the arm;
visibility snapshot/restore round-trip across show -> cancel and across two
successive samples; pair selection asserted distinct-piece and non-zero on
both layers in both modes, with the representative pair inside the measured
realistic hop range; reload-after-sample clears isolation.

Re-verified 2026-07-19 after the second amendment, by **transition**: the
chord round-trips most_curved -> representative -> most_curved as
present/absent/present; a user-set transparency on `Surface Bot` (0.3) and
`Surface TX Base` (0.7) survives show -> cancel byte-identical. Layer/surface
correspondence checked numerically -- selecting RX puts every path node at
0.0000mm from `Surface_RX_Offset` and 2.08mm from the TX surface, and vice
versa. The untouched core was re-checked unchanged: both matrices (70,70),
symmetric to 1e-9, zero diagonal, all finite, elementwise >= chord.


## S1.32 Fabrication sequence confirmed (roadmap 6.3) -- RX prints first, manual silicone fill between passes, one live layer at a time

**Decision:** The supervisor has given the physical fabrication sequence for
the dual-layer sensing pad:

> RX sensor layer -> fill the gaps with silicone -> TX sensor layer -> fill
> the gaps with silicone. Silicone application is a **manual** process.

Four things follow, and they close S1.30's open print-order caveat:

- **RX is the first robot pass, TX the second.** This **supersedes the
  print-order half of S1.30**, which recorded TX first. S1.30's substantive
  claim -- that RX and TX are two electrode layers offset by the print's
  thickness -- is unaffected and stands.
- **Two separate toolpaths, run one at a time.** Each layer is ordered on its
  own surface graph and printed as its own pass. There is never a toolpath,
  cost matrix, precompute run, cache file or playback spanning both layers,
  and never a travel move from the last RX piece to the first TX piece --
  each pass starts and ends at rest. The manual silicone fill sits in that
  gap and is **not modelled in the simulator**: no between-pass state
  machine, no rendered fill. Switching the live layer is the user action that
  represents it.
- **The live layer drives scene visibility.** RX shows itself and everything
  below it (`Curved Toolpath RX`, `Surface RX Offset`, `Surface Bot`) with
  `Surface TX Base` and `Curved Toolpath TX` hidden; TX shows the whole
  object, including the already-printed RX layer beneath it in its normal
  colour. This is **functional, not cosmetic**: S1.31 measured
  `Surface_RX_Offset` as sealed inside the `Surface_TX_Base` shell (a uniform
  ~2mm outboard, 100% of 3,000 sampled points), so with TX_Base shown the RX
  curves and their geodesics are not merely cluttered, they are *invisible* --
  which is exactly why `_isolate_geodesic_layer()` had to exist for 6.2's
  sample view. 6.6's layer selector generalises that existing
  snapshot/restore pair rather than adding a second visibility mechanism
  beside it.
- **The 6.5 collision obstacle is per-pass.** RX clears `Surface_Bot` (nothing
  is printed yet); TX clears `Surface_RX_Offset`, standing in for the cured RX
  traces plus silicone fill that are physically present by then. Both keep the
  nozzle-tip exemption S1.13's successor needs -- a strict "no contact with
  the obstacle" test rejects every valid feed waypoint, since tip contact is
  what printing is. `Surface_Bot` remains a collision body only, never a print
  target (S1.30 unchanged on that point).

**Assumption, not confirmed:** S1.31 measured a 2.00mm gap between
`Surface_Bot` and `Surface_RX_Offset`, so RX printing first does not put RX
in contact with the mockup. The working assumption is that a **silicone base
layer is applied to the shoulder before the RX pass**, making
`Surface_RX_Offset` that base's outer surface. This is an **open question for
the supervisor**, recorded so the discrepancy isn't rediscovered later.
Nothing in 6.3-6.6 depends on the answer -- it changes the interpretation of
the gap, not any transform, route or clearance.

✅ **Confirmed by the supervisor -- see S1.34.**

**Reason:** Direct supervisor answer, and it resolves the contradiction
S1.30's caveat left open in favour of the measured geometry. The stack
measures BOT -> RX (2.00mm) -> TX (4.02mm) with `TX_Base` outside
`RX_Offset` for 100% of sampled points, which is precisely a first-laid RX.
The filenames (`Surface_TX_Base` reading as "the base layer") were the
misleading signal, and the silicone dielectric fill between two electrode
layers independently corroborates the capacitive-tactile-sensor reading the
asset survey had inferred from geometry alone.

The existing architecture needed no reversal to accommodate any of this: 6.2
already built two per-surface graphs and two 70x70 cost matrices (S1.31), and
6.5 already scheduled per-layer cache files. Only the pass *order*, the
per-pass collision surface, and the visibility rule are new.

**Non-revertible unless:** a future asset drop changes the layer relationship
-- a third layer, or an offset that no longer represents print thickness. The
code that would change is 6.3's pass sequencing, 6.5's per-pass collision
surface, and 6.6's layer selector and visibility sets.

**Verified on:** 2026-07-20 -- supervisor statement. No new geometry
measurement needed: the stack ordering, the 2.00mm/4.02mm gaps and the
enclosure of `RX_Offset` by `TX_Base` were all measured in S1.29/S1.31, and
this entry adopts those numbers rather than re-deriving them.


## S1.33 Curved-surface printing generalized to a configurable layer list; RX/TX study config moved to examples/curved_surface_printing/

**Decision:** Curved-surface printing (roadmap Stage 6) is a core,
project-agnostic simulator feature -- same standing as flat-plate G-code
printing -- and stays in `geometry_backend.py`/`gui_panel.py`. What was
specific to one project (printing an elastomeric capacitive sensor onto a
shoulder mockup) was the hardcoded RX/TX wiring: which PLY/OBJ files, how
many layers, their names, colors, and the 90-degree CAD-rotation constant.
That wiring moved to `examples/curved_surface_printing/study_config.py`
(`CURVED_LAYERS`, `CURVED_MODEL_DIR`, `CURVED_MODEL_ROTATE_X_DEG`,
`CURVED_OBSTACLE_*`), imported by `geometry_backend.py` with one
clearly-commented import block.

The mechanism itself was generalized to consume that config rather than
assume exactly two layers named RX and TX:

- `load_curved_model()` now loops over `CURVED_LAYERS` (collapsing what were
  two near-identical `rx_*`/`tx_*` code paths into one), and populates a new
  `curved_layer_names` list other code and the GUI read instead of a
  hardcoded RX/TX pair.
- `run_geodesic_precompute()`/`step_geodesic_precompute()` iterate
  `range(len(CURVED_LAYERS))` instead of `(GEODESIC_LAYER_RX,
  GEODESIC_LAYER_TX)`; status strings read `curved_layer_names[layer]`
  instead of a `GEODESIC_LAYER_NAMES` constant.
- `_isolate_geodesic_layer()`'s RX-or-TX ternary became a loop hiding every
  *other* configured layer's surface and curve network (plus the optional
  obstacle mesh), so it isolates correctly for however many layers are
  configured.
- `gui_panel.py`'s layer-selector radio buttons read
  `content.curved_layer_names` instead of importing a `GEODESIC_LAYER_NAMES`
  constant -- the GUI has no RX/TX-specific code left at all.

Behaviour is unchanged: same structure names (`Curved Toolpath RX`/`TX`,
`Surface RX Offset`/`TX Base`/`Bot`), same placement math, same geodesic
results -- verified by re-running the load -> build geodesics -> show sample
(RX and TX) -> move plate sequence headlessly and confirming identical
output to before the refactor (26.1mm/ratio 1.110 for the RX representative
sample, matching S1.31's recorded figure).

**Reason:** The user wants a repo viewer to see curved-surface printing as a
real capability, not a bolted-on example -- but also wants cloning this repo
for a different curved-print job to mean writing a new `study_config.py`,
not extracting RX/TX-specific code out of `geometry_backend.py`/`gui_panel.py`
by hand. Genericizing the mechanism and isolating only the concrete config
achieves both: `assets/models/curved/` and the RX/TX asset wiring stay the
default configuration this simulator ships with, swappable via one import.

**Non-revertible unless:** a future need re-hardcodes a fixed two-layer
assumption somewhere the generic list-based code doesn't already cover --
none identified. Roadmap 6.3-6.6's fabrication-specific behaviour (per-pass
collision-obstacle swap, "manual silicone fill" messaging, RX-before-TX
ordering) should continue this split: express it as additional optional
`CURVED_LAYERS` fields the generic code reads with `.get(...)` defaults,
living in `study_config.py`, not as new bare module constants in
`geometry_backend.py`.

**Verified on:** 2026-07-20 -- `python -c` headless smoke test: load curved
model, build geodesics to completion, show sample geodesics on both layers,
move the build plate and confirm geodesic invalidation still fires via the
(unchanged, already-generic) `load_build_plate()` hook. All matched
pre-refactor behaviour and S1.31's recorded sample figures.


## S1.34 Silicone base layer under RX confirmed (roadmap 6.5) -- fills the measured 2.00mm gap between Surface_Bot and Surface_RX_Offset

**Decision:** The supervisor has confirmed that a silicone base layer is
applied to the shoulder mockup before the RX pass. `Surface_RX_Offset` is
that base layer's outer surface, not the bare shoulder body -- closing the
open question S1.32 recorded.

This is the *third* silicone application in the fabrication sequence,
distinct from the two S1.32 already recorded: base layer (before RX) -> RX
pass -> gap fill (silicone, manual) -> TX pass -> gap fill (silicone,
manual).

**Reason:** Direct supervisor confirmation. It also further corroborates the
capacitive-tactile-sensor reading the asset survey inferred from geometry
alone (S1.30) -- silicone as a dielectric now appears at the base of the
stack as well as between the two electrode layers.

**Non-revertible unless:** N/A in practice -- per the original open question
and S1.31's measurement, the answer changes only how the 2.00mm gap is
*interpreted*, not any transform, route, clearance, or collision-obstacle
assignment in the code (`Surface_Bot` was already, and remains, the fixed
RX-pass collision body regardless of what physically sits above it). This
entry exists purely so the question isn't reopened or rediscovered later.

**Verified on:** 2026-07-20 -- direct supervisor confirmation.


## S1.35 Print ordering (roadmap 6.3) -- per-layer greedy + 2-opt over oriented pieces, synchronous, hover-offset travel moves bookended with true endpoints, from-scratch outward normals

**Decision:** Each layer's 35 curve pieces are ordered and the travel moves
between them emitted by `build_print_order()` (`geometry_backend.py`), consuming
6.2's per-layer geodesic cost matrices and predecessor rows. Generic over
`CURVED_LAYERS` (S1.33) -- no RX/TX-specific code.

- **A TSP variant, objective = total inter-piece travel.** Every piece is
  printed once (a feed move along its curve, cost fixed regardless of order), so
  order changes only the sum of the geodesic hops between pieces. Each piece has
  two entry ends, so ordering also chooses which end to enter -- endpoint `2p`
  and `2p+1` are the two ends of piece `p` (`_layer_endpoints_world`), a piece's
  other end is `e ^ 1`, and a print order is a list of `(piece, entry_end)`.
- **Greedy nearest-endpoint seed + 2-opt**, module-level pure functions
  (`greedy_piece_order`, `two_opt`, `travel_cost`; S1.1). A good order, not
  proven-optimal (`Stage6_README.md` 6.3). 2-opt reverses a contiguous block
  *and flips each block piece's entry/exit end*; because geodesic cost is
  symmetric a reversed internal hop keeps the same two physical endpoints and is
  unchanged, so only the two cut edges move -- but with N=35 the tour is
  re-summed in full (trivial, and immune to delta-sign slips). Block length 1 is
  a single-piece end-swap, so a piece's entry end can be improved on its own.
- **Zero-cost ties break to the lowest endpoint index** (stable `argmin`), so
  the order is reproducible. A zero-cost hop to a *different* piece is real free
  travel and is taken; a piece leaves the candidate set the moment it is entered,
  so a closed loop's `cost[2p, 2p+1] == 0` is never a candidate (S1.31's trap).
- **RX first, TX second, ordered independently** (S1.32). No travel move stitches
  the last RX piece to the first TX piece -- the manual silicone fill sits there.
- **Synchronous, not chunked.** Unlike 6.2's ~8.4s Dijkstra precompute, this only
  walks stored predecessor rows (no re-solve), so it finishes inside one frame
  and runs straight off the button -- no per-frame stepper.
- **Travel moves hover.** Each travel polyline is the 6.2 geodesic offset outward
  by `CURVED_TRAVEL_HOVER_MM` (= 4.0, assumed) along the local surface normal, so
  the nozzle never scrapes the mockup or wet traces. Rendered as one combined
  curve network `Curved Travel <name>` per layer in `CURVED_TRAVEL_COLOR` (amber)
  -- feed curves keep their per-layer colour, giving the two-colour feed/travel
  view. Both constants are generic job-tuning values and live in
  `geometry_backend.py` (S1.33 licenses this, same as `FILAMENT_DIAMETER_MM`).
- **Snap gap closed by appending true endpoints.** A geodesic starts/ends at the
  snapped mesh vertex, ~0.36mm from the true curve endpoint (S1.31). Each travel
  polyline is bookended with the true exit/entry endpoints (also lifted to hover
  height along their snap-vertex normal), so the route connects end-to-end.

**Also settled -- surface normals are computed from scratch, sign fixed against
Surface_Bot at load time.** trimesh's `vertex_normals` needs `scipy.sparse` and
silently degrades to poor normals without it (measured: only ~84% outward), so
`compute_vertex_normals(verts, faces)` accumulates area-weighted face normals in
numpy (no scipy, AGENTS.md). The outward sign is one global bit, decided in
`_orient_normals_outward()` by a majority vote against the direction away from
the nearest `Surface_Bot` vertex -- the obstacle mesh is in scope during
`load_curved_model()`, so the sign is baked into the retained
`curved_surface_vnormals_world` and Bot is *not* retained (that stays a 6.5
concern). This is the normal lookup 6.4 was to introduce, pulled forward because
6.3's hover needs it first (asset survey's 6.3 notes); 6.4 reuses it. A wrong
sign would drive the nozzle into the mockup.

**Also settled -- the rim-hugging diagnostic is reported, not gated.** 6.2 warned
a geodesic can legitimately track the surface's open boundary (mean 18% of nodes
over random pairs; `CurvedModel_Geodesics.md`). On the *ordered* travel moves the
max per-layer figure is higher (measured ~46%), because the greedy chain favours
short hops between near-rim endpoints. `build_print_order()` reports the max
rim-node fraction in its status line; it is geometrically correct and left as a
physical judgement for 6.4/6.5 (higher hover or a rim penalty), not a hard reject
here.

**Reason:** Feed cost being order-invariant makes travel-sum the whole objective,
which is why the two matrices from 6.2 are exactly what ordering needs. The
oriented-piece 2-opt with cost symmetry is what lets a standard segment-reversal
also choose entry ends without special-casing. Synchronous is correct because the
expensive part (Dijkstra) already ran in 6.2 -- 6.3 is pure walk-backs. Computing
normals from scratch removes both a hidden scipy dependency and the measured
84%-outward degradation; orienting against Bot at load avoids retaining it.

**Non-revertible unless:** the piece count grows enough that full-tour re-summing
in 2-opt stops fitting a frame (escape hatch: the two-cut-edge delta the symmetry
argument already justifies), or a future asset drop's surface has genuinely
inconsistent winding that a single global normal sign can't correct (then per-
vertex sign against Bot, not a global vote). Rim-hugging becoming unacceptable
would change 6.4/6.5's hover/penalty, not this ordering.

**Verified on:** 2026-07-21 -- headless. Pure functions: across 20 random layers
2-opt never increases travel cost and every order is a piece permutation with
valid entry ends; a zero-cost different-piece hop is taken. Full pipeline on the
shipped RX/TX assets: retained normals unit-length and 100% outward from Bot on
both layers (was 84% via trimesh); optimized travel RX 690mm vs 5157mm file-order
and TX 607mm vs 4848mm; every travel-node nearest-surface clearance >= hover
(3.97/3.96mm vs 4.0); reload removes the `Curved Travel` networks and clears
`curved_order_loaded`.

**Amended 2026-07-21 -- print-order visualisation.** The first cut drew only the
amber travel hops, which didn't let the user *verify the order* and left both
layers rendered at once (RX invisible inside the TX shell). Three additions:

- **Ordered-feed gradient overlay.** `build_print_order()` now also registers a
  `Curved Order Feed <name>` curve network per layer -- the printed pieces in
  print order, each oriented by its entry end (piece reversed when entered at
  `2p+1`) -- coloured by a **sequence gradient** (`_sequence_colors` over the
  `CURVED_ORDER_CMAP` purple->teal->yellow ramp, applied per-edge via
  `add_color_quantity(defined_on='edges')`, the proven pattern from the coord
  frame -- *not* Polyscope's scalar colourmap, undocumented in this version).
  Edge index runs along the print order, so the gradient reads as the sequence.
  Drawn at `CURVED_ORDER_FEED_RADIUS_MM` (0.8) over the base curve.
- **Travel recoloured** to a solid warm red (`CURVED_TRAVEL_COLOR`), off the
  gradient ramp, so printing (gradient) vs moving (flat red) read apart.
- **Strict live-layer isolation.** `apply_live_layer_visibility(layer)` shows
  only the selected layer's surface / overlay / travel (base curve hidden while
  its overlay is present); every other layer is hidden. Driven by the GUI's
  existing RX/TX radio -- applied on change and after each build. **Strict
  isolation is deliberate for verification**; the eventual 6.6 rule is the S1.32
  physical stack (TX shows the RX layer beneath), a localised change to the
  `visible`/base-curve logic. It is now the **sole** visibility mechanism: the
  sample-geodesic isolation helpers it once composed with were removed with the
  rest of that aid (see S1.31's "Removed 2026-07-21" note).

Verified headless: overlays registered with the right per-order edge counts
(RX 2492 / TX 1965) and oriented at the entry end; selecting either layer
enables only its structures and disables the others (base curves off while
overlays exist); reload removes both overlay and travel networks.


## S1.36 Per-waypoint tool orientation (roadmap 6.4) -- TCP Z = outward surface normal, in-plane axes pinned to a fixed world reference, not the path tangent

**Supersedes S1.12's single-constant `R_target` for the curved path.** S1.12
snapshots one TCP orientation (`T_user_frame[:3,:3]`) for a whole G-code path --
correct then, because the build plate is flat and doesn't tilt mid-print, so the
nozzle prints perpendicular everywhere with one orientation. A curved shell has a
different surface normal at every point, so one orientation would drive the nozzle
into the mockup on the steep parts. `build_orientation_frames()`
(`geometry_backend.py`, roadmap 6.4) replaces the single matrix with a
**per-waypoint** `R_target`.

**Decision:** For each printed feed point, the target TCP orientation (base frame,
3x3) is built from an orthonormal basis:

- **Z axis = the outward surface normal.** The nozzle approaches along `-Z`, into
  the surface. This *is* the S1.12 convention generalised: the flat plate's
  `R_target` third column is the plate's outward `+Z`, and at rpy=0 that reduces to
  `R = I` with the nozzle pointing straight down. Normals come from the
  already-outward `curved_surface_vnormals_world` (S1.35), sampled per feed point
  by nearest surface vertex (`nearest_vertex_index`, the same normal source 6.3's
  hover uses). The outward sign is fixed against `Surface_Bot` at load, so no
  re-check is needed -- a wrong sign would drive the nozzle into the mockup.
- **In-plane axes (X, Y) pinned to a fixed world reference, NOT the path tangent.**
  A print nozzle is rotationally symmetric about its own axis, so the spin DOF is
  physically free. Pinning it to a constant world direction means the frame only
  *tilts* as the normal changes and never *spins* as the toolpath meanders --
  minimising wrist (J6) travel, i.e. "as stable and straight as possible" (user
  directive). Tangent-alignment (the README's tentative suggestion) would be the
  opposite: J6 chasing every wiggle. The reference axis is chosen **per point as
  whichever of world X/Y/Z is most perpendicular to Z** (`argmin |a . z|`), so the
  projection `x = normalize(a - (a.z)z)` never collapses and adjacent frames stay
  close -- no flip as a normal sweeps past a world axis. `y = z x x` closes the
  right-handed basis.

**Scope -- 6.4 computes and visualises only; IK is 6.5.** The frames are stored per
layer as `curved_orient_frames` (list of `(pos_world, R_target)` in print order) --
exactly the array 6.5 will feed to `run_toolpath_ik_precompute` -- and rendered as
a downsampled triad overlay `Curved Orient Frames <name>` (every
`ORIENT_FRAME_STRIDE`-th waypoint, X red / Y green / Z blue, the coord-frame edge-
colour pattern batched across origins like `_register_curve_layer`). No IK is
solved here. Gated on `curved_order_loaded`; a re-order or reload invalidates the
frames (state cleared, triads removed) so a stale overlay can't outlive its order.
`apply_live_layer_visibility` toggles the triads with the rest of the live layer.

**Reason:** Z = outward normal is the direct meaning of "nozzle perpendicular to
the surface" and keeps continuity with the flat-plate convention it replaces.
Fixing the free spin DOF to a world reference (rather than the tangent) is the
right call *because* the nozzle is symmetric: nothing about the print depends on
it, so the tie-breaker is arm-motion stability, and a constant reference minimises
tool roll. Per-point most-perpendicular axis selection is a branchless degeneracy
guard that also keeps neighbours smooth, feeding the S1.5 reference-pose ranking a
sensible sequence when 6.5 wires in IK.

**Non-revertible unless:** a future tool is *not* rotationally symmetric about its
axis (a directional applicator, a blade), in which case the spin DOF becomes
physically meaningful and would track the path tangent (or a process-defined
direction) instead of a world reference; or nearest-vertex normals prove too
faceted on a finer verify, warranting barycentric interpolation across the
containing face (a normal-source refinement, not a convention change).

**Verified on:** 2026-07-21 -- headless. Basis construction over 20000 random
normals: `|R^T R - I| < 1e-15`, `det(R) = +1`, `R[:,2]` equals the normal exactly,
and the most-perpendicular axis pick avoids projection collapse near every world
axis. Full pipeline on the shipped RX/TX assets (load -> geodesics -> order ->
orient): RX 2527 waypoints / 211 triads, TX 2000 / 167; every stored `R`
orthonormal to 6.7e-16 with `det = +1` and `R[:,2]` equal to the nearest-vertex
outward normal to 0.0 -- i.e. Z is numerically normal to the surface at every
waypoint; live-layer selection enables only the selected layer's triads; a
re-order clears `curved_orient_loaded` and removes both triad networks. Remaining:
the interactive eyeball (README 6.4 verify -- triad Z reads outward and
perpendicular on a steep part of the shell), which needs the GUI window.


## S1.37 Curved IK precompute (roadmap 6.5) -- Stage 5's machinery reused per layer, per-waypoint `R_target`, and nozzle-tip clearance against each waypoint's own tangent plane instead of an obstacle mesh

**Reuses Stage 5's chunked precompute (S1.11/S1.12/S1.21) rather than rewriting
it.** The chunked solver now loads from either source through one shared seam,
`_begin_toolpath_precompute(waypoints, R_target_array, ...)`. The G-code entry
point `run_toolpath_ik_precompute` is behaviourally unchanged (planar benchy); a
new sibling `run_curved_toolpath_ik_precompute(layer, ...)` feeds the same
machine from `build_curved_toolpath_waypoints_world(layer)` -- the S1.35 ordered
feed pieces interleaved with their travel hops, each waypoint carrying an S1.36
surface-normal orientation. `step_toolpath_ik_precompute` is the one solver for
both.

**Three decisions:**

- **Per-waypoint `R_target`, superseding S1.12's single constant.**
  `precompute_R_target` is now an `(N,3,3)` array indexed per waypoint. The
  planar path keeps its one constant orientation via `np.broadcast_to(R, (N,3,3))`
  -- a read-only view, no per-waypoint copy -- so nothing about the flat-plate
  solve changes; the curved path passes the real per-waypoint array from S1.36.

- **Nozzle-tip clearance against the waypoint's own tangent plane -- NOT the whole
  arm, and NOT world `z=0`.** The inbox note's literal suggestion was a real
  obstacle-mesh proximity check (`Surface_Bot` for RX, `Surface_RX_Offset` for
  TX). Rejected: `nearest_vertex_index` is brute-force by design (no scipy,
  S1.31), and querying a moving mesh's full vertex set against a
  tens-of-thousands-vertex obstacle thousands of times per precompute is too slow.
  Instead -- since S1.35 already treats each print surface as a convex-ish dome
  cap -- **a tangent plane at a point on a convex surface is a supporting
  hyperplane for the whole body**: everything on its outward side provably clears
  the entire surface behind it, and since RX_Offset sits outward of `Surface_Bot`
  and TX_Base outward of RX_Offset everywhere (the measured stack, S1.30/S1.32/
  S1.34), a point outward of *this waypoint's own* tangent plane (point = the
  waypoint, normal = its `R_target[:,2]`, already computed by S1.36) also clears
  every surface further inward -- no obstacle mesh, for either pass.

  **The check applies to the nozzle tip only (`_nozzle_clears_plane`), not the
  arm links.** This corrects the original plan, which would have tested all 6 arm
  links against the plane. The supporting-hyperplane proof bounds where the
  *surface* is, not where the *arm* is: the arm must span from its base up to the
  contact point, so its lower links legitimately sit far *inward* of a local
  tangent plane (measured Robot1 ~-92mm, Robot2 ~-194mm at a real waypoint) while
  the nozzle tip sits on the surface (~0). Testing the links would reject every
  real printing pose. The nozzle is the only part required to stay outward, and it
  gets `CURVED_TIP_CLEARANCE_TOLERANCE_MM` (assumed ~1.0mm) of **inward** slack --
  it prints *on* the surface, so its worst signed distance is ~0 and must be
  allowed to dip slightly in; the tolerance is *added* to the signed distance
  (subtracting would demand the tip float outward and reject every feed waypoint).
  The check keeps the cheap-corners-first / exact-vertices-fallback escalation
  (signed distance is linear, so its min over a rigid-transformed AABB's 8 corners
  is a lower bound on its min over the mesh).

  **World `z=0` is dropped for the curved case** (kept unchanged for the planar
  path via `_branch_clears_ground(angles, plane=None)`). The curved mockup sits
  above the plate in a frame where z=0 is not the physical floor (inbox note), so
  valid printing poses routinely put arm links below z=0 (measured z_min ~-60 to
  -300mm on the only joint-limit-valid branches of a real waypoint) -- retaining
  the z=0 gate rejected every such pose. Full arm-vs-mockup collision beyond the
  nozzle is a **known limitation**, the same simplification class as the old
  planar z=0 proxy; closing it would need the rejected obstacle-mesh (or
  per-triangle) check and is left as a future improvement.

- **Per-layer caches, versioned.** `curved_precompute_cache_path(layer_name)`
  gives `curved_<layer>.precompute.npz` per pass, so the planar benchy, RX, and
  TX keep independent caches. `save_/load_toolpath_precompute_cache` gained a
  `cache_path` parameter (default `GCODE_PRECOMPUTE_CACHE`) and `load_` a
  `meta_builder` callable, so the shared machinery no longer hardcodes the planar
  file or meta function. `_curved_toolpath_cache_meta` keys on a SHA-256 of the
  *derived* waypoint positions + feed flags + orientation array (there's no single
  curved source file to hash the way there's one G-code file; the derived arrays
  are what drift on a re-order/re-orient), plus the build-plate pose.
  `PRECOMPUTE_CACHE_VERSION` bumped 1->2 -- a one-time silent rebuild of the
  existing planar cache, not a bug.

**Scope -- `geometry_backend.py` only; no GUI, no bead constants.** The curved
bead-size constants (inbox note 6.5 item 3) are deferred to 6.6, where curved
playback will actually call the bead builder -- shipping the constant now, unused,
would be half-finished code (AGENTS.md). No `gui_panel.py`/`main.py`/`study_config.py`
change; the per-pass obstacle distinction `study_config.py` would have needed is
moot under the plane design.

**Reachability is a placement property, not a code concern.** On the shipped
assets at the default plate pose, 7 of 3175 RX waypoints and 6 of 2688 TX are
geometrically reachable but have no joint-limit-valid IK branch (verified solving
each in isolation), so a full curved precompute aborts at the first (no partial
motion, S1.12). This is expected: the build plate pose is a free variable
(`load_build_plate(rpy_deg=...)`) meant to be varied until a fully reachable
placement is found -- finding that pose is a setup step, out of 6.5's
`geometry_backend.py`-only scope. The precompute machinery itself is complete and
correct (below).

**Non-revertible unless:** the mockup stack turns out non-convex somewhere -- then
a waypoint's tangent plane is no longer a global supporting hyperplane and the
nozzle clearance argument fails, forcing a real obstacle-mesh (or per-triangle)
check for the affected surface (also the path to close the arm-vs-mockup
limitation above). `build_curved_toolpath_waypoints_world` asserts
`len(travel_moves) == len(pieces) - 1` so a future multi-component surface (where
`build_print_order` could skip an unreachable geodesic gap) fails loud rather than
silently misaligning travel with pieces.

**Verified on:** 2026-07-21 -- headless. **Planar regression:** the G-code
precompute solves all 181,375 waypoints, writes the v2 cache, and reloads from it
("Loaded ... from cache"); the `plane=None` clearance path is byte-for-byte the
old z=0 check. **Curved machinery:** RX solves its full 1809-waypoint reachable
prefix (with 6.4 per-waypoint orientation + nozzle clearance) before the expected
abort at the first dead-spot; FK-reproducing the solved poses matches each target
position to 6.7e-13mm and `R_target[i]` to 3e-15 -- i.e. the per-waypoint
orientation threads through IK exactly. **Nozzle clearance:** a solved branch
clears its own tangent plane, and shifting that plane outward past the tip by more
than the tolerance rejects it. **Per-layer cache plumbing:** writing a completed
path under each layer's key produces independent `curved_rx.precompute.npz` /
`curved_tx.precompute.npz`, and a fresh `run_curved_toolpath_ik_precompute` reloads
each. **Remaining:** an end-to-end full-curved solve + cache, which needs a plate
pose with no unreachable waypoints (the placement step above), and the interactive
GUI eyeball (6.6).


## S1.38 Curved GUI wiring (roadmap 6.6) -- one source-aware set of precompute/playback controls, per-layer coexisting bead playback, a stack-rule live view, a toggleable z=0 ground check, and a top-down build panel

**Roadmap 6.6, the last curved-printing stage. `geometry_backend.py` +
`gui_panel.py` only.** Wires the 6.1-6.5 backend into the panel and adds curved
playback. Built in two passes (the wiring, then two follow-up fixes); recorded
together here.

**One shared, source-aware control set -- not a duplicated curved panel.** A new
`toolpath_source` field (-1 = planar G-code; 0..N-1 = curved layer index) is the
single source of truth for what the existing Run/Pause/Cancel Precompute and
Run/Pause/Reset Toolpath controls target. `run_active_toolpath_ik_precompute()`
dispatches to `run_toolpath_ik_precompute` or `run_curved_toolpath_ik_precompute(layer)`;
`run_/reset_/advance_toolpath_playback()` read `toolpath_source` internally and
act on the planar bead slot or the correct layer's curved bead slot. The GUI's
old RX/TX-only radio is replaced by a "Toolpath Source" selector (`Planar (G-code)`
+ one entry per layer). No second copy of the controls (user directive: reuse, do
not duplicate).

**Layer-mixup guard.** The two `run_*_ik_precompute` entry points only consult
their `layer`/source inside `if precompute_waypoints is None:`, so switching the
active source while a run is paused would otherwise *silently resume the wrong
one*. Both now force-cancel (`_abort_toolpath_ik_precompute`) a loaded run whose
`precompute_cache_path` doesn't match the requested source before starting -- the
guard sits at the one seam both callers pass through, so it holds regardless of
GUI state. **Also fixed:** `load_toolpath_precompute_cache()` never recorded which
cache a *hit* came from, so `precompute_cache_path` was `None` after a cache load;
every 6.6 source-identity check depends on it, so it's now set on the hit path
too.

**Per-layer bead playback that coexists (the S1.32 stack rule made functional).**
Stage 5's playback was hardcoded to the single G-code print mesh. 6.6 adds
`_build_curved_beads(layer)` (the curved analogue of `_build_gcode_beads`: fixed
`CURVED_BEAD_WIDTH_MM`/`CURVED_BEAD_HEIGHT_MM` cross-section swept along each
waypoint's own surface normal `R_target[:,2]`, since a conformal path has no
extrusion `E` and no single "layer Z") and `_init_curved_toolpath_playback(layer)`,
storing bead state in per-layer lists (`curved_bead_*`) registered as
`Curved Print {name}`. Because the meshes are per-layer, a *completed* layer's
printed mesh survives switching to another layer -- so `apply_live_layer_visibility`
now implements the real S1.32 stack (`i <= layer`: layer k's view shows layers
0..k, incl. their bead meshes), replacing S1.35's provisional strict isolation.
Teardown was reworked so this coexistence holds: `_clear_gcode_print_mesh()` is
the G-code-only slice, called unconditionally by `clear_gcode_preview()` but only
by `_abort_toolpath_ik_precompute()` when the discarded run was the planar one
(keyed on `precompute_cache_path`, read before it's nulled); a generic precompute
abort never touches curved bead meshes -- only `clear_curved_model()` or a
re-order/re-orient cascade (`_abort_geodesic_precompute`) does. New
`clear_curved_model()` is the Load/Clear pair's backend (removes every registered
curved structure incl. the `Curved Print` meshes, resets all `curved_*` state).

**Toggleable z=0 ground check (`reject_below_ground`, default ON, applies to BOTH
paths).** S1.13's planar z=0 arm-vs-table check was always-on; S1.37 dropped z=0
for curved entirely (valid curved poses routinely go below z=0 at the default
plate pose). A user asked for a toggle -- "sometimes we may be able to put the
plate lower than the arm". `_branch_clears_ground` now gates the z=0 check on
`reject_below_ground` and *layers* it: with the toggle ON (default), planar does
the exact old z=0 check and curved does z=0 **and** its tangent-plane nozzle check
(S1.37); with it OFF, planar rejects nothing and curved does tangent-plane only
(the pre-toggle S1.37 behaviour). **Consequence:** with the default ON, a fresh
curved precompute now aborts early on z=0 at the default plate pose -- the user
unchecks the toggle for curved/low-plate work, which is exactly the case the
toggle exists for. **The toggle is folded into the precompute cache key** (both
`_toolpath_cache_meta` and `_curved_toolpath_cache_meta`) because it changes which
IK branch is accepted per waypoint, so the solved joint path depends on it;
`PRECOMPUTE_CACHE_VERSION` bumped 2->3 (one-time rebuild of existing caches, same
class as S1.37's 1->2). Exposed as a "Reject poses below ground (z<0)" checkbox in
Toolpath Settings, disabled mid-solve so one run can't be solved half under each
rule.

**Top-down build panel + properties dropdown.** The curved panel was reordered so
it reads down the page as the user progresses: Load Curved Model / Clear -> a
collapsible "Curved Model Properties" dropdown (`curved_model_summary()` --
backend-owned property lines: source dir, per-layer piece/vertex/face counts, and
travel figures once ordered) -> Toolpath Source selector -> Build Geodesics + its
progress bar + status -> Build Print Order + status -> Build Orientation Frames +
status. The only structural move was lifting the geodesic progress bar/status from
*below* the print-order/orientation buttons to *above* them, so each stage sits
under the previous stage's bar.

**Non-revertible unless:** the shared-precompute-state design (one flat
`precompute_*` set, one active run at a time) stops holding -- e.g. if curved
layers ever need to precompute concurrently, the source-identity-via-cache-path
scheme would need per-layer precompute state instead. The z=0 toggle being global
(one default for both paths) is the user's explicit choice (2026-07-22); a future
need for per-path defaults would split it.

**Verified on:** 2026-07-22 -- headless, plus the shipped-asset regressions.
**Source dispatch + mixup guard:** pausing an RX precompute and starting TX
force-cancels RX (cache path flips to TX's, index resets to 0) rather than
resuming it. **Coexistence:** after switching RX->TX and playing TX, `Curved Print
RX` stays registered alongside `Curved Print TX`. **G-code isolation:** a curved
layer switch leaves an unrelated loaded G-code preview untouched. **Clear/reload:**
`clear_curved_model()` removes every curved structure incl. both bead meshes and
`Surface Bot`, and an immediate reload re-registers cleanly. **z=0 toggle:** a
scanned below-ground pose is rejected with the toggle ON and accepted with it OFF;
curved RX aborts at waypoint 0 on the ground plane with the toggle ON but reaches
the S1.37 1809-waypoint tangent-only prefix with it OFF; the toggle changes both
cache metas (fields present, version 3). **Planar regression:** the full 181,375-
waypoint G-code precompute still solves and round-trips its (v3) cache; playback
grows the bead mesh and Reset re-inits. **Remaining:** the interactive GUI eyeball
-- the stack-rule view (TX showing the printed RX beneath it) and the bead reveal
during curved playback need the Polyscope window.

## S1.39 Hide guide overlays during playback (roadmap 6.7) -- `playback_active`, distinct from `playback_running`

**Roadmap 6.7. `geometry_backend.py` only.** 6.6 shipped the progressive curved
bead reveal, but the geodesic order-feed/travel curves, the base toolpath curve,
and the orientation-frame triads stayed drawn on top of the growing beads --
`apply_live_layer_visibility` toggled overlays only on layer *selection*, never on
playback *state*, so pressing Run left every guide up and you couldn't see the
object form. 6.7 gates the overlays on playback.

**New `playback_active` flag, distinct from `playback_running`.** `playback_running`
flips off on Pause; the overlays must stay hidden through a pause and restore only
on Reset, so a second flag was needed. `run_toolpath_playback()` sets
`playback_active = True` after the run actually starts and calls
`apply_live_layer_visibility(self.toolpath_source)` so the guides hide on the click;
`reset_toolpath_playback()` sets it False and re-applies visibility to restore the
full guide view. Pause touches neither flag's overlay state.

**`apply_live_layer_visibility` now reads playback state.** While `playback_active`,
the three overlay curve networks (order-feed/travel/orient) and the base toolpath
curve are force-hidden regardless of the `i <= layer` stack rule
(`overlay_visible = visible and not self.playback_active`); the print **surfaces**
and the growing **bead** meshes keep following the plain stack rule. Net effect
during playback: stacked surfaces + growing beads only.

**Planar (source -1) is a no-op:** `apply_live_layer_visibility` early-returns
unless `curved_model_loaded`, and the planar `G-code Print` mesh *is* the playback
mesh -- no separate overlays to hide. No GUI change: the coupling rides inside the
existing Run/Reset backend calls (`gui_panel.py` untouched).

**Verified on:** 2026-07-22 -- headless import/flow check only. The observable
result (guides vanish on Run, stay hidden through Pause, return on Reset; beads
still grow; planar unaffected) needs the Polyscope window on a physical GPU.

## S1.40 Posed-plate collision replacing the world-z=0 proxy (roadmap 6.8) -- arm always blocked, TCP optional; cache 3->4

**Roadmap 6.8. `geometry_backend.py` + `gui_panel.py`.** Replaces the crude
world-`z=0` clearance proxy (`reject_below_ground`, S1.13/S1.38) with a check
against the **actual posed build plate**. The plate is modelled as the infinite
plane through its top/print face, derived live from `self.T_user_frame`
(`_plate_plane()`: point = origin lifted `PLATE_THICKNESS_MM` along local +Z,
normal = plate local +Z), so it tracks wherever the Build Plate controls put the
plate.

**The rule:** the 6 arm-link meshes (0-5) may **never** dip below the plate (zero
tolerance); the nozzle (mesh 6) may, **only** when the new `allow_tcp_through_plate`
toggle is set (default **False** = nozzle also blocked, the safe default). A new
`_meshes_clear_plane(joint_angles, indices, point, normal, tol)` generalizes the
old `_nozzle_clears_plane` corners-first/exact-verts signed-distance test to an
arbitrary moving-geometry index set; `_branch_clears_ground` calls it for `range(6)`
(always) and `(6,)` (unless the toggle). The curved tangent-plane nozzle check
(S1.37) is unchanged and still layers on top when a `plane` is supplied. The two
world-`z=0` min-z helpers (`moving_geometry_bbox_min_z`/`moving_geometry_min_z`) are
deleted -- the generalized plane path supersedes them. Cache: `reject_below_ground`
-> `allow_tcp_through_plate` in both `_toolpath_cache_meta`/`_curved_toolpath_cache_meta`;
`PRECOMPUTE_CACHE_VERSION` 3 -> 4 (the toggle changes which IK branch is accepted).
GUI: the "Reject poses below ground (z<0)" checkbox becomes "Allow TCP through build
plate", bound to `allow_tcp_through_plate`, same disabled-mid-solve node.

**Consequence (from the spec).** Because the plane is infinite and the arm links
are always blocked, a precompute at the **default** plate pose (plate top ~z=0.75)
rejects early -- the arm reaches below the plate. The fix is to reposition the plate
lower via the Build Plate controls / saved position, not to disable the check. The
working plate poses and the operating procedure for the curved print live in the
supervisor's print-setup docs and the RX-setup steps in
`tutorials/Stage6_README.md` (6.8), not here; `assets/buildPlate/saved_position.json`
holds the adopted plate pose.

**Verified on:** 2026-07-22 -- headless code logic (all pose-independent): an arm
pose below the plate is rejected regardless of the toggle; a nozzle-only dip is
rejected with the toggle OFF and accepted with it ON; the tangent-plane check still
rejects a nozzle inward of its own plane while the plate check passes; both cache
metas carry `allow_tcp_through_plate` (not `reject_below_ground`) at version 4.
**Remaining:** the interactive GUI eyeball (the "Allow TCP through build plate"
checkbox driving the toggle and greying out mid-solve) needs the Polyscope window.

## S1.41 Assumed job constants move to study_config.py (amends S1.33) -- material/nozzle values follow the study, not the engine

**`geometry_backend.py` + `examples/curved_surface_printing/study_config.py`.**
The four "assumed, not measured" curved-print job constants move out of
`geometry_backend.py` and into the study config, joining the asset wiring
S1.33 already put there:

| Constant | Value | Consumer |
|---|---|---|
| `CURVED_TRAVEL_HOVER_MM` | 4.0 | `build_print_order()` |
| `CURVED_TIP_CLEARANCE_TOLERANCE_MM` | 1.0 | `_branch_clears_ground()` |
| `CURVED_BEAD_WIDTH_MM` | 1.5 | `_build_curved_beads()` |
| `CURVED_BEAD_HEIGHT_MM` | 0.5 | `_build_curved_beads()` |

`geometry_backend.py`'s existing single import from `study_config` grows to
carry them; no call site changes -- the names are unchanged, only where they
are bound. Values are unchanged, so no behaviour, cache-version or GUI change.

**Reason:** S1.33 split the curved-printing feature into a generic mechanism
(`geometry_backend.py`/`gui_panel.py`) and one study's concrete config, so that
cloning the repo for a different curved-print job means writing a new
`study_config.py` rather than editing the engine. These four constants sat on
the wrong side of that line: all four are **material- and nozzle-dependent**
(their own comments justified them as "a plausible elastomer trace width for
this nozzle", "a soft elastomer bead", "a plausible nozzle-contact depth"), so
a different job with a different material or nozzle had to edit
`geometry_backend.py` -- exactly the outcome S1.33 set out to avoid. S1.33's
closing guidance had named hover as generic job-tuning that could stay; that
carve-out is withdrawn here, since hover is elastomer-specific on the same
evidence as the other three.

Kept as **module-level constants**, not the optional `CURVED_LAYERS` `.get(...)`
fields S1.33 prescribes. That pattern is for genuinely *per-layer* behaviour
(the per-pass obstacle swap); these four are job-wide, so per-layer entries
would duplicate identical values across RX and TX. Module-level matches how
`CURVED_MODEL_ROTATE_X_DEG` and the `CURVED_OBSTACLE_*` values are already
configured.

**Non-revertible unless:** one of these four becomes genuinely per-layer (an
RX bead and a TX bead of different width, say) -- at which point it becomes a
`CURVED_LAYERS` field per S1.33, moving further into the config, not back to
`geometry_backend.py`.

**Verified on:** 2026-08-08 -- headless under the `fairino-fr5-sim` env:
`geometry_backend` imports cleanly and all four constants resolve to the same
values as `study_config`'s; no module-level assignment for any of the four
remains in `geometry_backend.py`; all call sites
(`build_print_order`/`_branch_clears_ground`/`_build_curved_beads`) reference
them unchanged. **Remaining:** no runtime eyeball -- values are identical, so
geodesics/print-order/precompute behaviour is unchanged by construction.

## S1.42 Pre-commit sweep -- dead `solve_toolpath_ik` removed, curved state resets and bead-face culling deduplicated

**`geometry_backend.py` only.** A duplication sweep before committing the
curved-printing branch. **No behaviour change on either path** -- every edit is
code motion or dead-code removal, proven below.

**1. `solve_toolpath_ik()` deleted (47 lines).** The Stage 5.4 synchronous
whole-path solver. Superseded by the chunked `step_toolpath_ik_precompute`
(S1.12) and left with **zero call sites** -- verified by AST across
`geometry_backend.py`/`gui_panel.py`/`main.py`, and the codebase contains no
`getattr`/`eval`/`exec`/`globals()`, so no dynamic-dispatch path could reach
it either. `step_toolpath_ik_precompute`'s docstring no longer cross-references
it. `docs/FR5_IK_Derivation.md` updated for the same reason (it described the
method as the live continuity consumer).

**2. Five `_reset_*_state()` helpers.** `__init__` and the two reset paths each
re-declared the same curved-model fields -- **37 assignments duplicated
verbatim**. Each group now has one definition:

| Helper | Fields | Callers |
|---|---|---|
| `_reset_curved_model_state()` | 8 | `__init__`, `clear_curved_model()` |
| `_reset_geodesic_state()` | 13 | `__init__`, `_abort_geodesic_precompute()` |
| `_reset_print_order_state()` | 6 | `__init__`, `_abort_geodesic_precompute()` |
| `_reset_orientation_state()` | 3 | `__init__`, `_abort_geodesic_precompute()` |
| `_reset_curved_bead_state()` | 7 | `__init__`, `_abort_geodesic_precompute()` |

Two deltas deliberately left at their call sites, not folded in:
`clear_curved_model()` also sets `toolpath_source = -1`, and `__init__` sets
`geodesic_status` (the abort path sets its own explanatory message first).

The helpers are **pure state assignment -- no Polyscope calls** -- which is what
makes them safe from `__init__` before any structure is registered; the
`ps.remove_*` calls stay at the call sites where the structure names are known.
The pre-existing `_reset_toolpath_playback_state()` is the naming precedent but
**not** the structural one: it calls `_clear_gcode_print_mesh()`, so it is not
`__init__`-safe.

**Consequence for future work:** add a new curved state field to its helper,
not to `__init__` -- otherwise the clear/abort paths leave it stale. That
add-one-forget-the-other hazard is the whole reason for the extraction.

**3. `bead_faces()` extracted (module-level).** `_build_gcode_beads` and
`_build_curved_beads` shared ~14 lines of S1.19 cap-culling (cap-cull ->
`keep_row` -> `faces_full` -> `bead_face_prefix`), differing only in the
planar caller's width-match term. Now one function taking `width_valid=None`
for callers with a fixed cross-section (the curved path). Module-level beside
`transform_points`/`_bbox_corners`, per S1.1, so it is testable without a
`VisContent`.

**Comments were reviewed and deliberately left alone** -- dense but accurate,
with no stale or dangling references, and the 10 guides already carry the
background. Length alone was not treated as a defect.

**Verified on:** 2026-08-08 -- headless, `fairino-fr5-sim` env, three
equivalence proofs rather than inspection:
(a) an AST snapshot of every `self.X = V` assignment reachable from
`__init__`/`clear_curved_model()`/`_abort_geodesic_precompute()` (following the
new helper calls) is **identical** before and after -- 75/11/29 assignments
respectively;
(b) `bead_faces()` is **bit-identical** to verbatim copies of both original
inline blocks across 303 randomised cases (varying K, chained/colinear/
width-matched patterns, plus `K==0`, single-bead, all-cullable and none-chained
edges), asserted with `np.array_equal` on both returned arrays;
(c) zero remaining references to `solve_toolpath_ik`, all three modules import
clean, no never-called functions remain, and FK(0) still returns
`[-820, -202, 50]`. `geometry_backend.py` 2995 -> 2952 lines.
**Remaining:** the GUI eyeball -- planar preview/precompute/playback and the
curved load -> geodesics -> order -> orientation -> precompute -> playback
cycle, plus a clear-and-reload of the curved model (the path the reset helpers
touch) -- needs the Polyscope window.


## S1.43 Real tool=1 TCP offset (roadmap 7.1) -- supersedes S1.4's derived rotation; nozzle mesh hidden, tool reduced to the TCP point, default plate pose moved to keep planar reachable

**Decision:** `T_flange_to_tcp` is now the real calibrated tool=1 offset,
`pose_to_matrix(*TCP_OFFSET_6D_MM_DEG)` with
`[-134.777, 96.448, 106.334, 86.647, -13.136, 60.612]` (mm, deg) from
`docs/saved_coords_data_and_usage_EN.md` 1.2. This **supersedes S1.4's
construction** -- a `TCP.txt` world point with rotation borrowed from
`inv(T_zero[5])` -- which could not represent a tool with its own orientation
offset from the flange. `tcp_local` is gone; the TCP is derived, not loaded.

**Reason:** S1.4's rotation-borrowing was explicitly a stand-in that "only
works because the current tool has no real orientation offset". tool=1 has one
(~87 / -13 / 61 deg). Roadmap 7.2's identity check compares against a reference
TCP pose that is only meaningful once this is wired in, which is why 7.1 leads
the stage.

**Two module-level helpers**, beside `rot_x`/`rot_y`/`rot_z` per S1.1:
`pose_to_matrix(x, y, z, rx, ry, rz)` (the docs' one convention for every 6D
pose they publish, `R = Rz @ Ry @ Rx`) and `matrix_to_pose(T)` (the exchange
spec's extraction formulas, for 7.2's degree-valued rotation error).
`pose_to_matrix` is a **refactor, not new maths** -- the same composition was
already inline in `solve_ik_tcp` and `load_build_plate`.

**The asset is wrong, not the calibration.** The supervisor confirmed on
2026-08-14 that tool=1 is correct, so the 33.4mm flange-to-tip conflict
(`nozzle.obj` 163.47mm vs tool=1 196.91mm) means `nozzle.obj` is not the head
that was calibrated. Magnitude is frame-independent, so this was never a
convention mismatch. Consequences:

1. **The `Nozzle` structure is registered but `set_enabled(False)`.** Not
   deleted, and still wired into `rest_verts`/`update_fns`, so a corrected
   asset needs one flag flipped. `nozzle.obj` stays on disk.
2. **The tool's collision body is the single TCP point**, not the mesh --
   colliding against a hidden asset of the wrong length would reject poses on
   geometry the real head does not have. *Considered and rejected:* a
   flange->TCP line (2 points) or the flange-frame AABB (8 points). Both are
   exact against a plane, but neither buys anything for the curved deliverable
   and the real bracket shape is unknown.
3. **A visual-only "Tool Axis" stalk** (flange origin -> TCP, 196.91mm, a
   2-node curve network at index 9) replaces the mesh *on screen*, since
   otherwise nothing shows where the tool points. Deliberately **not** in the
   collision set -- see the rejection above.

**Render vs. collision geometry are now separate lists.** `rest_verts[6]` is
still the nozzle's render buffer (`update_fns[6]` needs its full vertex count),
so the collision set became its own `moving_geometry_rest_verts` =
6 arm links + the TCP point. `_meshes_clear_plane`/`_nozzle_clears_plane` index
that; `_moving_geometry_deltas` is unchanged, since `Delta_6` is correct for
the point exactly as it was for the mesh. Index 6's bbox is 8 coincident
corners -- degenerate but harmless, the corners bound is then exact.
`_nozzle_clears_plane` is thereby a literal tip test and more permissive than
before; 7.2 removes it outright.

**`PRECOMPUTE_CACHE_VERSION` 4 -> 5.** Neither cache meta hashes the TCP
offset, so every cached joint path would otherwise load as a hit having been
solved for a different tool.

**`USER_FRAME_ORIGIN_MM` moved `[-600, -300, 0]` -> `[-570, -300, -100]`.**
Not cosmetic and not optional: the real offset puts the flange at
TCP + `[-41.6, -108.95, 158.66]` instead of `[-21.9, 26.0, 159.9]`, and that
extra ~109mm in -Y pushed the far corner of the bed outside the arm's 820mm
envelope. Measured: **3 of 181,375** planar waypoints needed a wrist centre up
to **835.35mm** out (worst over-extension 15.35mm), and waypoint 0 was one of
them -- so S1.12's abort-on-first-failure killed the whole path at index 0.
+19.4mm X restores reach; -100 Z clears the residual posed-plate rejection.
The old pose had 646mm max wrist-centre distance, i.e. a margin the new tool
consumed. `assets/buildPlate/saved_position.json` was **not** touched -- it
still holds the 6.8 curved setup, and 7.3 replaces it with the real User Frame.

**Verified on:** 2026-08-14 -- headless, `fairino-fr5-sim` env:
(a) `matrix_to_pose(pose_to_matrix(...))` round-trips to 0.0, and
`pose_to_matrix` matches the docs' 3 listing element-for-element;
(b) the identity check -- FK(0) + TCP reproduces
`[-954.777, -308.334, 146.448, -161.378, -58.051, -25.434]` to **0.000000 mm /
0.0003 deg**, against 7.2's 0.1mm / 0.5deg thresholds, and flange->TCP measures
196.911mm;
(c) IK/FK round-trip over 200 seeded poses -- all solved, FK of **every**
returned branch reproduces the target TCP pose to 5e-11 mm. Seed recovery is
1992/2000 over a wider sweep, but the **same 8 seeds miss identically with no
tool at all, with S1.4's transform, and with this one** -- it is `solve_ik`'s
8-branch enumeration, unrelated to the offset, and the miss set being identical
across all three is itself evidence the TCP layer is an exact change of
variables;
(d) the full planar precompute at the new default solves **181,375 / 181,375**
in 111s, matching the pre-7.1 baseline exactly, after correctly rejecting the
stale v4 cache. Joint ranges stay well inside the `gui_panel` sliders (widest
is J3, 14.4..84.2 deg against +/-155).
**GUI eyeball done:** hidden nozzle, tilted TCP triad, and the Tool Axis stalk
tracking the arm all confirmed.

**Note for 7.2:** max joint step between adjacent waypoints measures **57.32
deg** against the spec's 30 deg rejection row -- but that was measured across
the whole path, so the large steps are almost certainly G0 travel moves, which
are segment *boundaries*. 7.2 must measure **within** a segment before treating
this as a violation.
