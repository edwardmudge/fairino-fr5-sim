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
   path (`assets/models/gcode/model.precompute.npz`, sha256-verified
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

Measured on the real benchy (`assets/models/gcode/model.gcode`,
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
`assets/models/gcode/model.precompute.npz` (already covered by
`.gitignore`'s `assets/models/gcode/*.npz` pattern).

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
