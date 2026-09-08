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

> ⚠ **Amended 2026-09-06 (S1.58).** The final clause below -- that loading a
> saved position is "only ever triggered by that explicit button click, never
> automatically at startup, which still always begins from
> `USER_FRAME_ORIGIN_MM`/zero-rotation" -- **no longer holds.** Startup now
> applies `saved_position.json` when readable, falling back to
> `USER_FRAME_ORIGIN_MM` otherwise, because the shipped curved caches are keyed
> on the plate pose and were solved at the saved frame. Everything else in this
> entry (the re-posable plate, the RPY convention, Move/Reset/Save/Load) stands.

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
   > ⚠ **Symbol gone (noted 2026-09-08).** `playback_waypoint_index` no longer
   > exists anywhere in the codebase, and that line range now points at unrelated
   > code. The analogy stands as written; only the pointer is dead.
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

**Amended by S1.52 (2026-09-04):** item 1's two modes ("start fresh" /
"resume") became **three** at Stage 7.7 -- `run_` now also early-returns
when the loaded precompute is already *complete*, rather than resuming a
finished run into a crash. Pause/resume itself is unchanged.

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

> ⚠ **Changed 2026-09-04 (S1.55).** `PLAYBACK_RENDER_STRIDE` no longer exists.
> The stride is derived per playback from the path's own joint motion
> (`_derive_playback_render_stride()`), because a fixed *waypoint* stride is
> only a fixed *visible* step when joint motion per waypoint matches -- it does
> not across toolpath sources. The planar path still derives exactly 50, so the
> measurements below stand as recorded for that path; a curved layer derives ~6.

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

> ⚠ **Partly changed 2026-09-04 (S1.55).** The *methodology* here is still
> current and was reused to measure S1.55. What changed is the throttle it
> describes: `PLAYBACK_RENDER_STRIDE` is gone, replaced by a per-playback
> derived stride. The `PLAYBACK_LOOKAHEAD_BEADS` registration scheme in item 3
> is unchanged.

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

**Verified on:** not recorded. This entry was closed by the forward reference
above rather than by a verification pass, and it states no date and describes no
test. Left as an honest gap rather than back-filled with a guessed date (audited
2026-09-08, S1.74) -- the mechanism it describes is exercised by S1.22's and
S1.59's cache-key verifications.

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

> ⚠ **Amended 2026-09-08 (S1.60).** The split described below stands, but the
> *mechanism* has changed: the study config is no longer selected by editing
> `geometry_backend.py`'s import. It resolves
> `os.environ.get("FR5_STUDY_CONFIG", DEFAULT_STUDY_CONFIG)` via `importlib`, so
> pointing the feature at another job needs no source edit at all. Read every
> "swappable via one import" phrasing below as "swappable via `FR5_STUDY_CONFIG`".

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
  > ⚠ **Amended 2026-09-08 (S1.62).** The full re-sum is gone: `two_opt()` now
  > scores candidates with `_reverse_delta()` -- exactly the two cut edges this
  > paragraph identifies -- making a sweep O(N^2) rather than O(N^3), because
  > this stage runs synchronously off a button click and a study config with a
  > few hundred pieces froze the GUI for minutes. The "immune to delta-sign
  > slips" caution was answered by measurement, not argument: 280/280 identical
  > orders on random symmetric cost matrices, worst delta error 1.7e-13 against a
  > 1e-9 threshold. Everything else in this bullet still holds.
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

> ⚠ **Changed in Stage 7.4 (S1.46, built and measured in S1.47).** Both halves
> of this entry's orientation rule are superseded as the **commanded** pose:
>
> - **Z = the outward normal** is now an inequality, not an equality -- the tool
>   axis need only be perpendicular within **20 deg** (supervisor's instruction).
> - **The in-plane axes are no longer pinned** by `argmin |a . z|`. That rule's
>   discrete switching is what produced the row-5 flips (23/35 RX, 15/35 TX
>   segments with >30 deg steps inside a feed run); the roll is now **searched**
>   over 60 slots and resolved globally by continuity cost in the candidate DAG.
>   This is why `2026-08-15_orientation_frame_flips_row5.md`'s option 1 was
>   deliberately **not** implemented -- do not add it.
>
> What this entry still describes correctly, and what the code still does:
> `_orientation_frames_for_points()` is **unchanged** and still returns exactly
> the frame below. Its role changed rather than its content -- it is now the
> **axis of the search cone**, and its Z column is the surface normal that
> `build_export_segments()` exports as the spec's `normal_base` and that
> `_build_curved_beads()` stacks beads along. The `argmin` reference-axis flip
> is therefore now cosmetic: it only orients the 6.4 display triads.

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

> ⚠ **Changed in Stage 7.2 (2026-08-15) -- the clearance half of this decision
> is superseded by S1.44. The reuse half still stands.**
>
> - The **tangent-plane nozzle check is gone**: `_nozzle_clears_plane()` is
>   deleted, `CURVED_TIP_CLEARANCE_TOLERANCE_MM` is legacy and unimported, and
>   `_branch_clears_ground()` no longer takes a `plane`. It had also been
>   *incapable of rejecting anything* since 7.1, which made the tool a single TCP
>   point that IK pins to the very plane being tested -- measured 7,471
>   evaluations, zero rejections, worst 3.4e-13mm against a 1.0mm tolerance.
> - The curved path now runs **no clearance check whatsoever** -- it lost the
>   posed-plate check too. A curved solve means "reachable and within joint
>   limits", nothing more.
> - `_begin_toolpath_precompute`'s planar/curved discriminator is now the boolean
>   `check_collision`, not `tip_tolerance_mm`.
>
> **Everything below describing the tangent-plane check as live describes 6.5's
> design, not current behaviour.** The shared-seam reuse, the per-waypoint
> `(N,3,3)` `R_target`, and the per-layer caching are all unchanged. See **S1.44**.

> ⚠ **Changed again in Stage 7.4 (S1.46, built in S1.47) -- the obstacle mesh
> this entry declined to build now exists.** The curved path is no longer
> check-free: **filter 8** tests the arm links against each layer's own print
> surface (`_build_surface_grid` over `Surface_RX_Offset`/`Surface_TX_Base`),
> which is the first mesh-vs-mesh collision check in the project.
>
> This entry's argument for avoiding it -- that a full-arm obstacle test "would
> reject every real printing pose" -- held only while ONE orientation was
> commanded per waypoint. With 540 searched, it does not: where a waypoint is
> reachable at all, ~95% of it survives the whole nine-filter stack, and filter 8
> accounts for just 234 (RX) / 237 (TX) candidate rejections.
>
> `CURVED_TIP_CLEARANCE_TOLERANCE_MM` is **imported and live again** as filter
> 8's clearance -- keeping it under a legacy marker rather than deleting it at
> 7.2 is what made that possible. `_begin_toolpath_precompute`'s discriminator is
> now `filter_mode` ("planar"/"curved"), not the `check_collision` boolean noted
> above.
>
> Still true, and still not fixed by 7.4: the **nozzle** is unguarded. The tool
> is a single TCP point that IK pins to the commanded waypoint, so it is
> deliberately excluded from filters 6-8 -- including it would reject every
> printing pose, which is the same trap described above.

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

> ⚠ **Narrowed in Stage 7.2 (2026-08-15) -- this check is now PLANAR ONLY.**
> The mechanism below is unchanged and still correct for the planar path
> (`_plate_plane`, `_meshes_clear_plane`, `allow_tcp_through_plate`, arm always
> blocked / TCP gated). What changed is *when it runs*: curved precomputes skip
> it entirely via `check_collision=False`. Where this section says "both paths"
> or layers the S1.37 tangent-plane check on top, read **S1.44** -- that check is
> deleted and the curved path has no geometric rejection left at all.

> ⚠ **REMOVED in Stage 7.4 (S1.46, built in S1.47). This entry is history, not
> code.** `_branch_clears_ground()` and the `allow_tcp_through_plate` toggle --
> field, cache-key entry and GUI checkbox -- are **deleted**. The infinite plane
> is replaced by **filter 6** (under-plate footprint, 20mm XY margin) plus
> **filter 7** (plate bounding slab, 3.0mm), and both apply to **both** paths, so
> 7.2's narrowing above is reversed too.
>
> Why it could not simply be layered under: the plate model here is an
> **infinite** plane, sound only while the plate sits below the whole arm. At the
> real User Frame (S1.45) it is **323.5mm above the base**, where it cuts through
> the shoulder and elbow -- links nowhere near the print -- and rejected all 8
> valid branches at planar waypoint 0 (deepest link signed distance -253.2mm).
> This entry's own prescribed fix, "move the plate lower", became unavailable
> once the plate height was a measurement rather than a knob.
>
> Measured result of the replacement: planar goes from **aborting at waypoint 0**
> to solving **181,375 / 181,375** at that same plate pose. A real bed is finite
> and the arm legitimately reaches around it.
>
> `_plate_plane()` and `_meshes_clear_plane()` **survive** -- filter 5 (elbow
> above the plate plane) still uses the former.

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

> ⚠ **Changed in Stage 7.7 (S1.51).** The **rendering** half of this entry is
> reversed: the nozzle mesh is visible again, re-aimed at load time onto the TCP
> frame's -Z, and the "Tool Axis" stalk that stood in for it is deleted, not
> hidden (`apply_delta_transform`'s loop is `range(9)` again). Everything else
> here stands unchanged -- the tool=1 offset, the retirement of `TCP.txt`, the
> moved default plate pose, and above all the **collision** decision: the tool's
> collision body is still the single TCP point, because the asset's own
> shape/length is exactly as uncalibrated as it was while hidden. This entry's
> open item ("needs a corrected tool asset, not another filter") is **not**
> resolved by 7.7.

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

> **Answered by 7.2 (2026-08-15).** The hypothesis holds for **planar** but is
> **wrong for curved**: there the >30 deg steps sit *inside* feed segments, from
> a reference-axis flip in `_orientation_frames_for_points`, not at boundaries.
> See S1.44 and `wiki/001_Inbox/2026-08-15_orientation_frame_flips_row5.md`.


## S1.44 Exchange-spec Rejection Criteria adopted (roadmap 7.2) -- seven rows on both toolpath sources, in-house collision narrowed to planar, solver moved to the physical joint limits; cache 5->6

**Decision:** the external IK exchange spec's seven-row Rejection Criteria table
(`examples/curved_surface_printing/external_ik_exchange_spec_EN.md`) is the
definition of a valid print job, implemented verbatim as module-level
`validate_job(vis, segments)` returning `(ok, [CheckResult])`. In exchange, this
project's own pose rejection is **narrowed to the planar path**.

### Scope: the two halves answer the scoping question differently

This is the load-bearing distinction and the easiest thing to get wrong.

**The seven rows are universal** -- both toolpath sources, and any future one.
Not one row reads a surface, a normal's provenance, RX/TX, or anything in
`study_config.py`: rows 1-2 are properties of the solver and the calibration and
do not even read the toolpath; rows 3-7 are properties of the robot and the
data. The only shoulder-specific content in the spec is cosmetic (its example
folder name `print_job_TX_sensors/`). One export path (confirmed decision #1,
2026-07-22) therefore gets **one source-agnostic validator**.

**The collision narrowing is curved-only** -- deliberately asymmetric. Planar
keeps Stage 6.8 behaviour exactly; the flat plate is a real, cheap, already
validated constraint and removing it there would buy nothing. Do not let the
table's symmetry pull this symmetric.

**These rows validate data, not geometry.** A job can pass all seven and still
drive the arm through the plate or the workpiece.

### Part 1 -- the narrowing

`_begin_toolpath_precompute`'s `tip_tolerance_mm=None` argument (which *was* the
planar/curved discriminator) becomes a boolean `check_collision`:
`run_toolpath_ik_precompute` passes **True**, `run_curved_toolpath_ik_precompute`
passes **False**. `step_toolpath_ik_precompute` filters the ranked branches
through `_branch_clears_ground` only when set, and takes `solutions[0]`
otherwise. A flag rather than a read of `self.toolpath_source`, because the
precompute snapshots its own per-run state at begin and a live-mutating source
field could change mid-solve. `_abort_toolpath_ik_precompute` resets it to
**True**, the checked direction.

`_branch_clears_ground` loses its `plane` parameter and keeps only the plate
half. `_plate_plane`, `_meshes_clear_plane` and `allow_tcp_through_plate` are
untouched.

**`_nozzle_clears_plane` deleted -- it superseded S1.37, which was already
dead.** Not a judgement call: **7.1 had made the check incapable of rejecting
anything.** It tested the tool against the tangent plane through a waypoint, but
7.1 reduced the tool's collision body to the single TCP point, which IK pins to
that exact waypoint. Signed distance was therefore identically zero.

**Measured before deleting** (2026-08-15, headless): over all **5,863** cached
RX+TX waypoints and **1,608** re-solved candidate branches -- including branches
the precompute did not choose -- it returned `True` **7,471 / 7,471** times, with
worst \|signed distance\| **3.4e-13 mm** against a 1.0mm tolerance. Twelve orders
of magnitude of margin. Removing it changed no accept/reject outcome.

**Consequence, recorded where it will be read:** nozzle-vs-workpiece protection
was lost at **7.1**, not here. Combined with the plate check going, the curved
path now has *no* geometric rejection whatsoever. `CurvedModel_PrintSetup.md`'s
open question is widened accordingly, from "the arm passes through the mockup"
to "the arm **and the nozzle body** pass through the mockup".
`CURVED_TIP_CLEARANCE_TOLERANCE_MM` stays in `study_config.py` under a legacy
marker -- a tuned material value, unimported.

### Part 2 -- segments, pulled forward from the export sub-stage (7.4 then, 7.5 now)

Rows 5 and 6 need a segment concept neither path had. **Both sources already
emit `(pos, is_feed_move)` waypoints**, and on each a segment is exactly a
maximal run of consecutive `is_feed=True` -- precisely the spec's "one
continuous extrusion line". So `build_export_segments()` is **one shared
function**, not two per-path builders, and needs no reference to
`build_print_order`: the piece boundaries are already in the flags. Travel runs
are dropped, not exported (the spec has the receiving side re-insert a travel
MoveJ between segments). Only the solved prefix is used, so a paused precompute
yields the segments solved so far.

Verified against real data: **35 segments == 35 print-order pieces** on both RX
and TX, with segment lengths summing exactly to the feed-waypoint count
(2,527 RX / 2,000 TX).

### Part 3 -- the seven rows

Constants live in `geometry_backend.py`, not `study_config.py`: they are
robot/spec-level, and S1.41 reserves that module for material- and
nozzle-dependent values.

**Nothing calls `validate_job` or `build_export_segments` yet.** Both are
module/class-level API with no caller: the export writer is 7.5 and the GUI
hookup is 7.6. "Implemented" here means "built and verified headless", not
"reachable from the app".

- **Row 2 is circular** for a single-source project -- we *are* the calibration.
  Kept per confirmed decision #5 (implement verbatim), and it earns its place:
  `TCP_CALIBRATION_REFERENCE_6D` is transcribed separately from the supervisor
  doc, so the pair catches a mistyped digit in either constant.
- **Row 5 measures within a segment only.** A large jump *between* segments is
  legal -- the receiving side inserts a travel MoveJ there.
- **Row 6** has no ply to count until 7.5, so it checks the structural invariant
  the line count will inherit: positions, joints and normals must agree. Note
  this is **tautological for segments `build_export_segments` produced** -- it
  slices all three arrays with the same `sl`, so they cannot disagree. Same
  circularity as row 2, and kept for the same reason: it starts earning its keep
  the moment 7.5 writes a real line count, or anything hand-builds a segment.
- **Row 7 is WARN, and must not reject.** It is also a *different* notion from
  `solve_ik`'s `is_singular` (`|sin(theta5)| < 1e-6`, near-exact degeneracy);
  the spec's 2 deg band is far wider.

### Part 4 -- physical vs practical joint limits

`gui_panel.JOINT_LIMITS` was never drift: it cites `docs/FR5_Joint_Limits.md`,
but that doc's separate **"Practical Slider Ranges"** section. The real defect
was that **the solver borrowed a slider constant**. New
`PHYSICAL_JOINT_LIMITS` in `geometry_backend.py` (J2/J4 -264..+84) now feeds the
precompute call site, the manual IK panel, and row 3. `JOINT_LIMITS` keeps the
sliders, where the conservative range is honest -- hand-dragging a joint has no
continuity or clearance check behind it.

**Accepted risk:** J2 gains ~134 deg and J4 ~94 deg of deep-negative travel on
both print paths, at the same moment the curved path loses its last geometric
check. J2's shallow floor was documented as a collision-safety proxy, and
nothing replaces it. Measured effect on branch availability: **425 valid
branches vs 207** over an 80-pose sample -- the solver was rejecting more than
half of what the arm can reach.

Note also that `wrap_into_limits` returns the first of `k in (0, +/-360)` inside
the window, so a wider window can change which *representation* is selected,
which changes `wrapped_dist` ranking. Solved paths shift even where they already
succeeded -- which is why the planar path was re-run rather than assumed.

### Cache

`PRECOMPUTE_CACHE_VERSION` **5 -> 6**. One bump covers both changes: neither
cache meta hashes the collision rule or the joint limits.

### Verified on 2026-08-15 -- headless, `fairino-fr5-sim` env

- **Each of the seven rows fires alone.** Seven fixtures, each FK-consistent by
  construction so only the row under test breaks: every one failed its own row
  with the right message while **all six others still passed**, and row 7 warned
  without rejecting. A clean job passes all seven.
- **Segment builder**: counts, lengths, travel exclusion, sequential indices,
  `R_target` Z column as `normal_base`, partial-precompute prefix, and the
  empty case. Plus the real-data match above.
- **The narrowing**: `_nozzle_clears_plane` and `precompute_tip_tolerance_mm`
  gone, `_branch_clears_ground` signature reduced, planar passing
  `check_collision=True` and curved `False`, the step loop bypassing the filter,
  and the plate check still discriminating (rejects an arm below the plate,
  accepts one above).
- **Limits**: both solver call sites on `PHYSICAL_JOINT_LIMITS`, sliders still
  on `JOINT_LIMITS`, physical strictly containing practical.
- **Cache**: all three on-disk caches (two v4, one v5) correctly rejected at v6.
- **Planar re-run end to end** under the new physical limits: **181,375 /
  181,375** solved in 120s at `USER_FRAME_ORIGIN_MM = [-570, -300, -100]`,
  after correctly rejecting the stale v5 cache. Segments into **20,350** runs
  (134,618 feed / 46,757 travel waypoints, lengths summing exactly). The full
  seven-row validator returns **ACCEPTED**, worst per-point FK error
  **0.000000mm**. Solved joint ranges are unchanged and sit well inside even the
  slider window (widest J3, 14.4..84.2 deg), so widening the solver's window did
  not move the planar solution -- it only stopped rejecting reachable poses
  elsewhere.

  This also **settles 7.1's open note**: max step across the whole path
  reproduces at **57.32 deg**, but within a feed segment it is **5.85 deg**,
  with **0 of 20,350** segments violating row 5. The large steps really were G0
  travel boundaries, exactly as hypothesised -- for planar.

### Found by this work -- curved jobs currently fail row 5

Running the new validator over the existing curved solved paths: **23 of 35 RX
segments and 15 of 35 TX segments** contain >30 deg joint steps *inside* a feed
run, worst **180.10 deg**. Cause is a discrete reference-axis switch in
`_orientation_frames_for_points` (S1.36): `argmin(|world_axes @ z|)` flips as the
normal sweeps, spinning the commanded frame about its own Z. The spin is
physically free (the nozzle is rotationally symmetric) but IK tracks it, so J6
jumps. Correlation is near 1:1 -- RX 74 switches / 78 large steps, TX 62 / 63.

**Not fixed here** -- 7.2's remit was to implement the criteria, and the criteria
found it. It is an S1.36 defect whose fix re-runs the whole curved pipeline,
which 7.3 forces anyway. Full diagnosis and three fix options:
`wiki/001_Inbox/2026-08-15_orientation_frame_flips_row5.md`.

### Known gap -- a cached precompute exports zero segments

Found in 7.2's pre-commit review, *not* by the verification above.
`load_toolpath_precompute_cache()` restores `precompute_joint_path` but **not**
`precompute_waypoints`/`precompute_R_target` -- both runners return on a cache
hit before `_begin_toolpath_precompute()`, the only place those are assigned. So
after any cache hit `build_export_segments()` returns `[]`: without the
`is_feed_move` flags there is nothing to segment on.

The verification above missed it because the planar end-to-end run happened
**after correctly rejecting the stale v5 cache** -- i.e. only ever on the
fresh-solve path.

**Consequence, and why it needed a fix before this commit:** rows 3-7 are all
"no offender found" tests, so they passed vacuously at `n_points = 0` while rows
1-2 never read the toolpath. An empty job reported **ACCEPTED**.

**Fixed here, the safety half only.** `validate_job` gained an **in-house row 0,
"job is non-empty" (REJECT)** -- explicitly *not* one of the spec's seven; the
spec never contemplates exporting nothing. `results` is therefore **8 long**,
row 0 first, then the seven in table order. A cached job still exports zero
segments; it now says so loudly instead of passing.

**Deferred to 7.5:** persisting `waypoint_positions`, `waypoint_is_feed` and the
`R_target` Z column in the npz. That is a schema change
(`PRECOMPUTE_CACHE_VERSION` 6 -> 7, every cache invalidated) and 7.5 needs those
positions in the exported ply anyway, so it pays for itself there and not here.
`PRECOMPUTE_CACHE_VERSION` stays at **6** for this stage. Full diagnosis,
including the cheaper curved-only half:
`wiki/001_Inbox/2026-08-15_export_segments_cache_gap.md`.

**Verified on:** 2026-08-15 -- headless, `fairino-fr5-sim` env. Not a second
pass: this is the labelled field for the `### Verified on 2026-08-15` section
above, which carries the detail (seven fixtures each failing only its own row,
row 7 warning without rejecting, planar re-run 181,375/181,375 under the
physical limits). Labelled 2026-09-08 (S1.74) so the field census matches the
template; no content changed.

---

## S1.45 Real calibrated User Frame adopted (roadmap 7.3) -- `saved_position.json` replaces the 6.8 demo pose; neither toolpath runs there, and that is the finding

> ⚠ **The frame is confirmed correct; the *diagnosis* below is superseded by
> S1.46 (roadmap 7.4).** The supervisor has confirmed the 7.3 configuration. The
> measurements here stand -- 226/2,527 RX, 186/2,000 TX, planar aborting at
> waypoint 0 -- but two conclusions drawn from them do not:
>
> - **"Placement, not calibration"** and **"genuine reach"**: these measure a
>   *single commanded orientation per waypoint* (S1.36 pins tool Z to the exact
>   normal and fixes the roll), which yields at most 8 IK candidates. They are
>   not statements about the arm's envelope. S1.46 searches ~480 candidates per
>   waypoint instead.
> - **The planar abort as a reason to question the plate's pose**: it is a
>   *shape* problem. S1.40's infinite plane is sound only while the plate sits
>   below the whole arm; at +323.5mm it is not. S1.46 makes the model finite.
>
> Consequently the "not settled" list at the end of this entry is narrower than
> it reads: the placement question is no longer blocking, it is a hypothesis
> S1.46 can now test.

**Decision:** `assets/buildPlate/saved_position.json` holds the **real
calibrated User Frame**, `user_index=1` from
`docs/saved_coords_data_and_usage_EN.md` §1.1 -- read from the physical robot
(192.168.58.2) on 2026-05-28. It replaces the Stage 6.8 demo pose outright, per
confirmed decision #4 (2026-07-22): no dual demo/real slot.

| | position (mm) | rpy (deg) |
|---|---|---|
| was -- 6.8 demo pose | `[-570, -300, -200]` | `[0, 0, 0]` |
| now -- real, `user_index=1` | `[649.456, 133.762, 322.778]` | `[-0.369, 0.329, -89.080]` |

**Data and docs only. No code changed, and none needed to.** This is the whole
implementation surface of 7.3:

- The rotation convention already matched exactly. `load_build_plate()` builds
  `R = Rz(yaw) @ Ry(pitch) @ Rx(roll)` from `[roll, pitch, yaw]`; the doc's §3
  `pose_to_matrix` is `R = Rz(rz) @ Ry(ry) @ Rx(rx)` from `[rx, ry, rz]`.
  Verified `max |ΔT| = 0.0`, and `matrix_to_pose` round-trips to the six input
  digits. The numbers go straight into the file.
- This is the first saved pose carrying a **real rotation** (~89° yaw), and
  every consumer already handled one: `create_coordinate_frame`'s `rotation`
  param (S1.6), `gui_panel`'s `bp_target_rpy` sync, and the full-4x4
  `user_frame` cache key.
- **No `PRECOMPUTE_CACHE_VERSION` bump.** Both cache metas already hash the
  whole 4x4, so every existing cache misses at the new pose by construction.
  Stays at **6**.

**The old pose is retained, not deleted**, under the file's inert
`_legacy_stage6_8_demo_pose` key. `load_saved_build_plate_position()` reads only
`position_mm` / `rpy_deg`, so nothing can select it -- it is a record, which is
what the no-dual-slot decision rules out selecting, not recording.

### `USER_FRAME_ORIGIN_MM` deliberately did NOT move with it

The startup/**Reset** default stays `[-570, -300, -100]`. It is a *chosen* pose,
tuned at 7.1 so all 181,375 planar waypoints solve; the file is the *measured*
one. Keeping them separate means the real frame is opt-in per session ("Load
Saved Position", S1.6 -- never read at startup), so nothing regressed for
anyone not clicking that button. Given the finding below, that separation is
now load-bearing rather than incidental.

### The finding: neither toolpath runs at the real frame, for two different reasons

§7.3 recorded reachability here as a known, unverified risk, and said a failure
was to be recorded rather than reverted. Both paths fail. Measured 2026-08-15,
headless, `PHYSICAL_JOINT_LIMITS`, `allow_tcp_through_plate = False`.

**Planar aborts at waypoint 0 -- on the plate check, not on reach.**
`Waypoint 0/181375: all 8 valid branch(es) hit the build plate (arm + nozzle)`.
IK is fine: waypoint 0 yields **8 valid branches**, and an IK-only sweep found
**0 / 13,952** sampled waypoints unreachable. The posed-plate check (S1.40,
still live on this path after 7.2) rejects all of them.

| | plate top plane | deepest arm-link signed distance, best branch | branches clearing |
|---|---|---|---|
| default `[-570,-300,-100]` | z = **-99.2mm** | **+125.4mm** | 8 / 8 |
| real User Frame | z = **+323.5mm** | **-253.2mm** | **0 / 8** |

S1.40 models the plate as an **infinite plane**, which is sound only while the
plate sits below the whole arm. The real frame is **323.5mm above the base
origin**, so the plane cuts through the shoulder and elbow -- links nowhere near
the print. `allow_tcp_through_plate` cannot rescue it: it gates the tool point,
and links 0-5 are blocked unconditionally. **This is a modelling limitation, not
a robot limitation** -- and S1.40's prescribed fix ("move the plate lower") is
unavailable, because the height is now a measurement rather than a knob.

**Curved is genuinely out of reach.** It never meets the above, having lost all
collision checking at 7.2, and fails on plain geometry instead:

| Layer | Feed points | Solved, real frame | Solved, demo pose | TCP distance from base |
|---|---|---|---|---|
| RX | 2,527 | **226** (91.1% unreachable) | 2,527 (100%) | median 912mm, max 945mm |
| TX | 2,000 | **186** (90.7% unreachable) | 2,000 (100%) | median 916mm, max 947mm |

All failures are `"Unreachable: no geometric solution"` -- the `a2 + a3 = 820mm`
chain does not extend that far. **Placement, not calibration:**
`load_curved_model()` centres the assembly on the plate (S1.29), and at the real
frame the plate centre is ~844mm out and its far corner ~980mm, while the corner
itself is only 738mm. That is exactly why the *planar* G-code, which sits near
the plate's local origin, has no reach problem at all.

### What this settles, and what it deliberately leaves open

**Settled:** the real frame stays. It is a measurement; the demo pose is not
coming back, and 7.5's exporter is unblocked because it targets the *planar*
path at the default pose.

**Not settled, and not to be guessed at:** whether the Bambu Lab plate mesh
describes what physically sits at the User Frame; whether S1.40's plane should
become finite or optional; and where the curved model actually belongs in the
real cell. The curved pipeline was **deliberately not re-run** -- re-running it
before the placement question is answered produces a meaningless result, and it
would burn the same rebuild the S1.36 reference-axis fix
(`2026-08-15_orientation_frame_flips_row5.md`) is waiting to share. The two stay
correctly paired, just later than 7.3 planned.

Full measurements, method and the open questions in priority order:
`wiki/001_Inbox/2026-08-15_real_user_frame_reachability.md`.

**Verified on:** 2026-08-15 -- headless, `PHYSICAL_JOINT_LIMITS`,
`allow_tcp_through_plate = False`. The rotation convention round-trips at
`max \|ΔT\| = 0.0` and `matrix_to_pose` returns the six input digits, which is
what licensed the data-only change; the reachability counts in the tables above
were measured in the same pass. Labelled 2026-09-08 (S1.74) from the dates
already stated in this entry; no content changed, and the **diagnosis** here
remains superseded by S1.46/S1.48 per the notice at the top.

---

## S1.46 Orientation search and a re-shaped filter set (roadmap 7.4) -- a waypoint is judged by whether *any* admissible pose reaches it, not by one commanded frame

**Decision:** the curved planner stops commanding a single `R_target` per
waypoint. It searches an orientation set, filters candidates with the nine
checks adapted from a working external implementation
(`examples/curved_surface_printing/IK_BRANCH_REJECTION_GUIDE.md`), and selects
the trajectory by global graph search rather than greedy per-waypoint ranking.

This supersedes the *interpretation* in S1.45, not its measurements.

### Why -- the failure was never reach

S1.45 measured 226/2,527 RX and 186/2,000 TX feed points solving at the real
User Frame and concluded "genuine reach ... placement, not calibration". The
supervisor has since confirmed the 7.3 configuration is **correct**, which makes
that conclusion untenable, and the mechanism is visible in S1.36: one commanded
frame per waypoint (tool Z exactly on the outward normal, roll fixed by
`argmin |a . z|`) yields **at most 8 IK candidates**. When none survives, the
point reports `"Unreachable: no geometric solution for this pose"` -- which is a
fact about that pose, not about the arm. The reference implementation searches
**480 candidates per point** (60 roll slots x 8 branches) and succeeds at the
same class of task.

### Part 1 -- the relaxation (the only part that is a relaxation)

- **Tool axis perpendicular within 20 deg**, not exactly. Supervisor's
  instruction. Supersedes S1.36's "Z = the outward surface normal" as a hard
  equality.
- **Roll about the tool axis is searched, not pinned.** 60 slots, 6 deg apart,
  wrapping. S1.36 established this DOF is *free* (the nozzle is rotationally
  symmetric) and then spent it on a fixed world reference; that choice is what
  produced the row-5 flips in S1.44. Searching it uses the same premise for a
  better end.
- **Parameterisation.** The supervisor phrased the search as "all combinations
  of Rx, Ry and Rz". A 20 deg tilt cone x full 360 deg roll is the same set,
  parameterised so the 20 deg cap constrains only the DOF it should and the free
  DOF is swept entirely. Recorded because the phrasing will resurface.

### Part 2 -- the filter set (this is *stricter*, not looser)

Adopted from the reference guide, cheapest arithmetic first, FK and collision
last, rejecting on first failure:

| # | Filter | Status here | Value |
|---|---|---|---|
| 1 | Joint limits | already have | `PHYSICAL_JOINT_LIMITS` (S1.44) |
| 2 | J5 non-negative | **open decision** -- interacts with spec row 7's \|J5\|<2deg WARN | -- |
| 3 | J4 minimum | opt-in, default off | -60 deg |
| 4 | Upper branch (elbow above shoulder-wrist chord) | adopt, new | 2.0mm |
| 5 | Elbow above plate plane | adopt, new | 1.0mm |
| 6 | Under-plate footprint | **adopt -- this is the S1.40 fix** | 20mm XY margin |
| 7 | Plate volume slab | adopt, new | 3.0mm |
| 8 | Surface mesh collision | adopt -- **first mesh-vs-mesh in this project** | 2.0mm |
| 9 | Robot/tool self-collision | adopt, new | 5.0mm |
| E1 | Max adjacent joint step (edge) | **adopt, retuned to 30 deg** | see below |
| E2 | Branch-change penalty (edge) | adopt | 150 / 2.0 quadratic |

**E1 must be 30 deg, not the reference's 35 deg.** The exchange spec's row 5
rejects steps **> 30 deg** (S1.44). Carrying 35 across would build a planner
whose own edge filter admits jobs the receiving side rejects.

**Filter 8 maps onto `CURVED_TIP_CLEARANCE_TOLERANCE_MM`**, kept as legacy at
S1.44 precisely because it is a tuned material value rather than junk. Prefer it
over the reference's 2.0mm default if the two disagree.

### Part 3 -- what this supersedes

- **S1.36** -- the in-plane reference axis. `argmin |a . z|` stops being a
  per-waypoint choice; the DOF is searched and resolved by continuity cost. This
  **subsumes** the fix sketched in `2026-08-15_orientation_frame_flips_row5.md`
  (option 1, propagate the previous frame) -- do not implement both. It also
  supersedes Z-equals-normal as an equality, per Part 1.
- **S1.37** -- the tangent-plane rationale. Its argument for avoiding an
  obstacle mesh, that a full-arm check "would reject every real printing pose",
  holds only for a single commanded orientation. Filters 8 and 9 are the
  obstacle-mesh check it declined to build.
- **S1.40** -- the infinite plate plane. Replaced by filters 6 and 7 (finite
  footprint plus bounding slab). S1.40's own prescription, "if the arm reaches
  below the plate the fix is to move the plate lower", is unavailable once the
  plate height is a measurement (S1.45); a real bed is finite and the arm reaches
  around it. `allow_tcp_through_plate` is superseded.
- **S1.44's collision narrowing** -- "this project's own pose rejection narrows
  to the planar path" is reversed. Both paths get the filter set. **S1.44's seven
  rows are untouched**; the narrowing and the table were always two different
  questions (S1.44's own scoping-trap note).
- **S1.45** -- diagnosis only, per the marker on that entry.

### Part 4 -- selection becomes a graph search

Candidates form a layered DAG over `(waypoint, candidate)` nodes; edges carry
the joint-movement costs plus E2, with E1 as a hard `inf`. Searched by Dijkstra.

**This is not new machinery.** S1.31 (roadmap 6.2) already built a hand-rolled
`heapq` Dijkstra over flat CSR lists for the geodesic cost matrices, chosen
because there is no `scipy` in the `fairino-fr5-sim` environment. Same primitive,
different graph. Reuse it rather than adding a dependency.

The greedy alternative currently in place -- rank branches by wrapped-angle
distance to the previous waypoint, take the first that clears (S1.5/S1.11) --
cannot recover from a dead end and cannot undo a discontinuity in the commanded
frame itself. That is exactly the failure mode
`2026-08-15_orientation_frame_flips_row5.md` documents.

### Cache

`PRECOMPUTE_CACHE_VERSION` **6 -> 7**. One bump shared with the export cache-gap
fix deferred to 7.5, since both invalidate every cache regardless.

### NOT YET MEASURED -- do not report these as results

Nothing in this entry has been run. In particular, **whether the orientation
search restores curved reachability is a prediction.** The honest arithmetic:
the flange->TCP offset is **196.91mm** and sits *laterally* off the flange, not
along its Z, so varying the commanded orientation relocates the wrist centre.

- A +/-20 deg tilt alone buys ~**68mm** (`2 * 196.91 * sin 10deg`).
- The median shortfall at the real frame is ~**92mm** (912mm TCP distance against
  the `a2 + a3 = 820mm` chain).
- The **full 360 deg roll sweep is the larger lever**, since the offset's
  component perpendicular to the tool axis is swept entirely.

Combined the swing is the same order as the shortfall -- plausibly enough, but
the placement question (S1.45) must not be declared closed until this is
measured. The measurement to run is in `Stage7_README.md` §7.4 "Verify".

Reference: `examples/curved_surface_printing/IK_BRANCH_REJECTION_GUIDE.md` --
external, describing another project's code, with its file paths and its 35 deg
default. Roadmap: `tutorials/Stage7_README.md` §7.4.

---

## S1.47 Orientation search, the nine filters and the candidate DAG, as built (roadmap 7.4) -- the search works, and it does not close the placement question

**Decision:** S1.46 as specified is implemented. This entry records what was
built and, more importantly, **what it measured** -- S1.46 ended with a "NOT YET
MEASURED" block, and this is the answer to it.

**Verified on:** 2026-09-03. Headless, `PHYSICAL_JOINT_LIMITS`, the real
calibrated User Frame from `saved_position.json`.

### What was built

- `orientation_candidates()` -- 9 tool-axis directions (the exact surface normal
  plus one 8-azimuth ring at the 20 deg cap) x 60 roll slots = **540 commanded
  frames** per waypoint, up to 8 IK branches each. S1.46's "~480" was the
  reference guide's 60x8 with no cone at all; the cone is a separate factor and
  its density was a choice, taken as one ring at the cap because that is where
  the reach leverage is.
- Nine candidate filters in `_candidate_admissible()`, cheapest first, rejecting
  on first failure, with a per-filter rejection tally reported on abort.
- `dijkstra_candidate_path()` -- module-level, beside `dijkstra_surface()`.
- `PRECOMPUTE_CACHE_VERSION` 6 -> **7**, and the cache schema grew
  `waypoint_positions` / `waypoint_is_feed` / `waypoint_normals`, which closes
  the export gap deferred from 7.2.
- `allow_tcp_through_plate` and `_branch_clears_ground()` **removed**, GUI
  checkbox included.

### The Dijkstra is a layered relaxation, not a heapq frontier

S1.46 said to reuse S1.31's `heapq` primitive. **It is the same algorithm but
not the same code, deliberately.** This graph is strictly layered -- every edge
runs from waypoint i to i+1 -- so the topological order IS the waypoint index,
every node settles exactly once when its layer is reached, and the heap has
nothing left to decide. Dropping it changes no result, and it lets a whole
layer's edge block be relaxed as one vectorised numpy operation.

That is not an optimisation, it is the difference between running and not:
curved RX is ~2,900 waypoints at up to ~4,320 candidates, so a heapq frontier
would walk ~12.5M nodes and ~5x10^10 edges in interpreted Python.

Verified against exhaustive search on 40 random DAGs, plus a hand-built case
where the optimum requires an early non-greedy choice to avoid a dead end two
layers later -- the exact failure mode S1.46 says the superseded greedy ranking
cannot recover from. It takes the dearer branch.

### E1 applies only between two FEED waypoints

The 30 deg step limit is scoped to pairs where both endpoints extrude, because
that is what the exchange spec's row 5 measures -- steps *within* a continuous
extrusion line. Travel moves are legitimately large. Measured on the planar
path: **57.32 deg** max step overall against **4.58 deg** within a segment. An
unscoped E1 would abort the planar job at its first G0.

### Three things the reference guide's values had to be corrected on

Each was found by measurement rather than review, and each would otherwise have
been a filter that rejects everything:

1. **The tool point must be excluded from filters 6, 7 and 8.** Its whole
   collision body has been the single TCP point since 7.1, and IK pins that
   point to the commanded waypoint -- which lies ON the print surface during a
   feed move, and at exactly the plate's top face on the planar first layer.
   This is the same trap that made S1.37's nozzle check inert (S1.44 measured
   <1e-12mm over 7,471 evaluations). The reference guide's
   `nozzle_tip_exclusion_mm` exists for precisely this.
2. **One OBB per link is unusable; filter 9 needs multi-proxy boxes.** Robot3's
   single OBB is 502mm long and reported contact with Robot5/Robot6 in **all 8
   branches** at planar waypoint 0, where the true mesh gap is 20-35mm against a
   5mm clearance. Links are now covered by rows of 80mm boxes (`_obb_proxies`).
3. **Filter 9's pairs must be at least THREE apart in the chain, not two.**
   `(i, i+2)` is separated by one short link, and on the FR5's compact wrist
   (d4/d5/d6 = 102/102/100mm) those meshes interpenetrate at every pose -- their
   relative motion is a single joint rotation about a shared axis, so no joint
   value separates them. Robot4~Robot6 fired on every branch at a true 30mm gap.

### Performance -- three fixes, 33x

The filter stack started at **3.677 ms/candidate** and the curved path at
**8,748 ms/waypoint**, which is ~13 hours per layer. Final: **0.266
ms/candidate** and **437 ms/waypoint** (~20 min per layer).

| Fix | Effect |
|---|---|
| One FK per candidate (was three: filter 4, the sample transform, filter 9) | part of 3.677 -> 0.742 |
| Bounding-sphere pre-test before filter 9's SAT | 0.533 -> 0.047 ms |
| Dense dilated occupancy array screening filter 8's points in one vectorised lookup, replacing a Python loop hashing 27 cells per sample point | ~8.5 -> ~0.2 s/waypoint |

### THE MEASUREMENT -- planar is fixed, curved is improved but not fixed

**Planar, at the real User Frame: completely fixed.**

| | before (S1.45) | now |
|---|---|---|
| Planar | **aborts at waypoint 0** of 181,375 | **181,375 / 181,375 solved**, 156s |

20,350 export segments, `validate_job` ACCEPTED on all 8 rows. The plate plane
still sits 323.5mm above the base; filters 6/7 simply let the arm reach *around*
a finite bed, which S1.40's infinite plane forbade.

**Curved: 8.5x better, and still not plannable.**

| Layer | Feed points | IK-reachable (any of 540) | Admissible (all 9 filters) | 7.3 baseline (1 orientation) |
|---|---|---|---|---|
| RX | 2,527 | 2,033 (80.5%) | **1,922 (76.1%)** | 226 (8.9%) |
| TX | 2,000 | 1,484 (74.2%) | **1,410 (70.5%)** | 186 (9.3%) |

**S1.46's prediction was half right, and the honest reading matters.** The "91%
unreachable" figure was indeed largely an artefact of commanding one orientation
-- reachability rises 8.5x once the pose is searched. But **494 RX and 516 TX
feed points have no IK solution at ANY of the 540 orientations**, so a complete
curved job still cannot be planned at the real frame, and the precompute
correctly aborts rather than relaxing a filter or falling back.

**This sharpens the S1.45 placement question rather than closing it.** The
residual failure is now known to be *pure geometric reach* -- not the commanded
pose, not the filter set, and not the plate model. Where a waypoint is reachable
at all, the filters admit ~95% of them. "Where should the curved model actually
sit in the real cell?" is now the only remaining explanation, and it is a
question for the supervisor.

### The control run settles that it IS placement

Same model, same 540-orientation search, same nine filters -- only the plate
pose changed, from the real calibrated frame to the startup default
`USER_FRAME_ORIGIN_MM = [-570, -300, -100]`:

| Layer | Real User Frame | **Default plate pose** | 7.3 baseline |
|---|---|---|---|
| RX | 1,922 / 2,527 (76.1%) | **2,527 / 2,527 (100.0%)** | 226 (8.9%) |
| TX | 1,410 / 2,000 (70.5%) | **2,000 / 2,000 (100.0%)** | 186 (9.3%) |

**Every feed point on both layers is admissible at the default pose**, filters
and all. So the nine filters do not over-reject -- given a sensible placement
they cost nothing -- and the arm is not short of reach. What fails at the real
frame is *where the workpiece is put*: `load_curved_model()` centres the assembly
on the plate (S1.29), and at the real frame that puts the plate centre ~844mm out
against an `a2 + a3 = 820mm` chain.

This is the cleanest evidence yet for S1.45's placement question, and it is now
the **only** unexplained thing about the curved path. It is also a warning: do
not tune the filters to "fix" curved reachability -- they are demonstrably not
the cause.

### Root cause found the same day: a 105.6mm centring offset from a stand-in asset

`load_curved_model()` centres the workpiece on the **build plate mesh's** bbox
centre, not on the User Frame. `BambuLab_BuildPlate.obj` is 258 x 276mm with its
origin at a corner, so the centre sits at plate-local `(129, 128)` -- which
through the real frame's ~-89 deg yaw puts the workpiece **843.1mm** from the
base instead of the User Frame origin's **737.5mm**. The FR5's flange reach is
`a2 + a3 + d5` = **922mm**, and reachability falls off a cliff exactly there:
100% below 900mm, 94% at 900-920mm, **~50% at 920-950mm**.

Removing only that offset -- same frame, same model, same solver -- gives
**843/843 RX and 667/667 TX reachable (100%)**, on a *coarser* orientation
sample than the failing runs.

So the frame is right, the arm's reach is right, and the filters are right. What
is wrong is this project's assumption about where the workpiece sits relative to
user frame 1, inherited from an asset that was only ever a stand-in. The question
to settle is narrow: **is user frame 1 defined at the corner of the print bed or
at its centre?** The code assumes corner.

Not fixed here -- choosing between the fixes is a data question for the
supervisor, and guessing would rebuild the whole curved pipeline against an
assumption. Full measurements, the confirming test and three fix options:
**`wiki/001_Inbox/2026-09-03_curved_placement_plate_centring_offset.md`**.

**Do not read the filters as the curved blocker.** The RX rejection tally is J5
26,401 / upper-branch 7,676 / self-collision 3,744 / elbow-plate 2,755 / surface
234 / under-plate 40 -- but those count *candidates*, not waypoints, and only
111 RX waypoints (2,033 reachable minus 1,922 admissible) are lost to filters at
all.

### Filter 2 set at J5 >= 2 deg costs nothing, and subsumes spec row 7

S1.46 left this open, flagging the interaction with row 7. Measured over 8,304
RX / 8,834 TX sampled branches, the J5 distribution is symmetric with
essentially nothing in the +/-2 band:

| Threshold | RX admits | TX admits |
|---|---|---|
| J5 >= -2 | 50.7% | 51.3% |
| J5 >= 0 (reference default) | 50.7% | 51.3% |
| **J5 >= 2 (chosen)** | **50.7%** | **51.2%** |

Choosing 2.0 over the reference's 0.0 costs **0 candidates on RX and 2 of 8,834
on TX**, and in exchange the exchange spec's row 7 `|J5| < 2deg` singularity WARN
becomes unreachable by construction. An exported job cannot carry that warning.

### Filter 8 works -- the first mesh-vs-mesh guard this project has had

Found a TX pose (waypoint 518, joints `[-170.75, -47.07, 50.02, -166.37, 155.11,
12.23]`) whose nearest arm-link sample sits **0.71mm** from the print surface
against a 1.0mm clearance. It passes filters 2-7, it is a valid IK solution
inside the physical joint limits, and **the pre-7.4 curved path would have
accepted it** -- 7.2 set `check_collision=False`, so a curved solve applied no
geometric test whatsoever (S1.44). The standing warning that "a solved TX run
drives the arm through the shoulder mockup" is now closed **for the arm**.

**It is NOT closed for the nozzle**, and that must not be overstated: the tool is
still a single TCP point, deliberately excluded from the collision filters, so
nothing guards the *nozzle* against the workpiece. Closing that needs a corrected
tool asset (7.1 found `nozzle.obj` is 163.47mm against tool=1's 196.91mm), not
another filter.

**Non-revertible unless:** the exchange spec's row 5 threshold changes (E1
aliases `JOINT_STEP_MAX_DEG` for exactly this reason), the FR5's wrist geometry
changes (filter 9's three-apart rule), or a real tool body arrives (which would
put the tool back into filters 6-8).

---

## S1.48 Curved workpiece placement decoupled from the build-plate mesh (roadmap 7.4 follow-up) -- both curved layers now solve 100% at the real User Frame

**Decision:** `load_curved_model()` no longer derives the workpiece's XY
placement from the build-plate MESH's bounding box. A new study-level constant,
`CURVED_MODEL_XY_OFFSET_MM = np.array([0.0, 0.0])` in
`examples/curved_surface_printing/study_config.py`, centers the workpiece
directly on the **User Frame origin** instead.

### Why

S1.47 measured curved reachability at the real User Frame improved 8.5x by the
roadmap 7.4 orientation search (76.1% RX, 70.5% TX admissible) but not fixed --
~24% of feed points had no IK solution at any of 540 searched orientations, and
a control run at the default plate pose gave 100% on both layers with the
identical filters and search, isolating the cause to **placement**, not the
arm, the filters, or the commanded pose.

The mechanism, found the same day: `load_curved_model()` centered the workpiece
on `BambuLab_BuildPlate.obj`'s own bbox center -- a stand-in asset, 258x276mm,
local origin at a corner. Through the real frame's ~-89 deg yaw, that added a
measured **+105.6mm** outward shift (User Frame origin 737.5mm from the base;
plate-mesh-centered placement 843.1mm). The FR5's flange reach is
`a2+a3+d5` = **922mm**, and reachability was measured to fall off a cliff
exactly there (100% below 900mm -> ~50% at 920-950mm).

A confirming test -- same real frame, same model, same solver, only the
centering offset removed -- measured 100% reachability on both layers (843/843
RX, 667/667 TX, coarse IK-only sampling). Full record and every ruled-out
alternative: `wiki/001_Inbox/2026-09-03_curved_placement_plate_centring_offset.md`.

### The decision itself: don't ask, measure

The inbox note framed this as an open question for the supervisor -- "is user
frame 1 defined at the corner of the print bed or at its centre?" -- because the
code's corner-relative assumption was unverified. Explicit user direction:
don't gate the fix on that question. The User Frame and TCP calibration data are
already measured and verified (S1.43, S1.45), so **whichever placement makes the
job reachable is the one the supervisor's data actually supports**. The offset
that was measured to work -- zero, i.e. the workpiece centered on the origin --
is therefore the answer, made an explicit, named constant rather than left as a
silent side effect of an unrelated asset's geometry.

### What changed

`geometry_backend.py`, `load_curved_model()`:

```python
# before
plate = self.load_mesh(os.path.join(BUILD_PLATE_DIR, BUILD_PLATE_FILE))
plate_min, plate_max = plate.bounds
T_placement[:2, 3] = (plate_min[:2] + plate_max[:2]) / 2.0 - (assembly_min[:2] + assembly_max[:2]) / 2.0

# after
T_placement[:2, 3] = CURVED_MODEL_XY_OFFSET_MM - (assembly_min[:2] + assembly_max[:2]) / 2.0
```

The `plate = self.load_mesh(...)` line for bounds is deleted outright --
confirmed nothing else in the function or file reads `plate_min`/`plate_max`.
**Not the same `plate` as `load_build_plate()`'s** (a separate load inside
[`load_build_plate()`](../../geometry_backend.py)), which registers the
visible plate mesh and sets `self.plate_local_bounds` for roadmap 7.4's
collision filters 6/7 -- fully independent, untouched by this change.
`T_placement[2, 3]` (Z anchoring) is untouched; this fix is XY-only.

### Measured result: full rebuild, real User Frame, headless, 2026-09-03

| Layer | Feed points | Solved (real 7.4 pipeline: search + 9 filters + DAG) | `validate_job` |
|---|---|---|---|
| RX | 2,527 | **3,175 / 3,175 waypoints (100%)** | **ACCEPTED** |
| TX | 2,000 | **2,688 / 2,688 waypoints (100%)** | **ACCEPTED** |

Both curved layers now solve completely and validate cleanly at the real
calibrated User Frame -- matching the coarse-sampled confirming test exactly, now
through the actual pipeline. **Curved is therefore no longer blocked for
roadmap 7.5's job export** -- 7.5 can target either toolpath source, not only
planar.

Geodesic travel totals were checked against the S1.35 baseline as a sanity test
that only XY moved (a rigid translation preserves intrinsic mm distances): RX
690mm vs 5157mm file-order, TX 607mm vs 4848mm -- **matched exactly**.

One non-blocking observation: the max joint step **across the whole solved
path** is large (81.74 deg RX, 275.33 deg TX), but measured to occur only
between two **travel** waypoints -- never within a feed segment (worst
within-segment step: 29.93 deg RX / 29.85 deg TX, both under the spec's 30 deg
limit, which is exactly why `validate_job` accepts). E1's hard rejection is
deliberately scoped to feed-to-feed edges only (S1.47), so a large travel-hop
step is architecturally expected, not a defect -- and travel waypoints are
dropped from export regardless (`build_export_segments()`). May read as a
visual "jump" during playback; does not affect correctness.

Fresh `.npz` caches were saved to
`assets/models/curved/curved_{rx,tx}.precompute.npz`, replacing the stale
plate-centered ones. No `PRECOMPUTE_CACHE_VERSION` bump -- `_curved_toolpath_cache_meta()`
already hashes waypoint positions, so the old caches missed by construction the
moment placement changed.

**Non-revertible unless:** a fresh measurement at a different offset
outperforms (0,0) at the real frame -- don't guess a replacement value without
one -- or the real fixture's geometry becomes known and the offset needs to
target something other than the origin.

**Verified on:** 2026-09-03.

## S1.49 Job export writer + GUI trigger (roadmap 7.5 + 7.6, done together) -- `write_job_export()`/`export_active_job()` write the exchange spec's package; an "Export IK Job" button triggers it

**Decision:** `write_job_export(vis, segments, job_dir)` (module-level,
alongside `validate_job`/`format_validation`) writes one solved
`toolpath_source` to `assets/export/<job_name>/` as `job.json` +
`segment_N_solution.json` + `toolpath_TN.ply` per segment, plus `surface.obj`
for curved sources. `VisContent.export_active_job()` is the thin glue: self-check
via `build_export_segments()`/`validate_job()` first, write only on ACCEPTED.
`gui_panel.py` gets an "Export IK Job" button (I/O Operations, beside Run/Reset
Toolpath) calling it. 7.6 (the GUI trigger) was explicitly folded into the same
pass as 7.5 rather than left for later, per direct user instruction -- the
roadmap's own sub-stage split was a convenience, not a hard boundary.

**Reason:** 7.2 built `build_export_segments()`/`validate_job()`/`ExportSegment`
with no caller (S1.44); 7.4 closed the cache-export gap and got both toolpath
sources to `validate_job` ACCEPTED (S1.47, S1.48). Nothing was left to decide
about *whether* the job is valid -- only how to write it, and how to trigger
that from the running app.

### Decisions settled during implementation

- **Output location** (`Stage7_README.md`'s Open Question, previously
  unanswered): `assets/export/<job_name>/`. `job_name` is `"planar"` for
  G-code, or `vis.curved_layer_names[vis.toolpath_source]` (`"RX"`/`"TX"`) for
  curved -- not the spec's own cosmetic `print_job_TX_sensors`-style naming,
  since nothing reads that string back.
- **`toolpath_T*.ply` has no PLY header.** `read_ply_polyline()` reads a
  proper PLY (`ply` / `format ascii 1.0` / `element vertex` / `element edge`
  header, x/y/z only, no normals) for this project's *own* curve assets
  (`assets/models/curved/RX_*.ply` etc.) -- a different, unrelated format that
  happens to share an extension. `external_ik_exchange_spec_EN.md`'s
  `toolpath_T*.ply` is its own plain format: every line is
  `x y z nx ny nz`, no header at all, confirmed against the spec text itself
  (its "toolpath_T*.ply Format" section shows only data lines). Do not
  "mirror `read_ply_polyline()`'s format" literally -- mirror its *column
  order* only, per the inbox note's own wording.
- **`surface.obj` is curved-only.** Planar's "plate" is S1.40/S1.47's modelled
  plane (infinite, then finite footprint + slab), never a mesh asset -- there
  is nothing to copy for it. Documented as a known gap in spec coverage for
  that source, not an oversight (docstring in `write_job_export()`, and now
  here).
- **Stale segment files are pruned on re-export.** A re-export with fewer
  segments than a prior run (e.g. after switching sources, or from a shorter
  partial precompute) deletes any `toolpath_T*.ply`/`segment_*_solution.json`
  whose index is `>= len(segments)` before writing -- otherwise a receiving
  parser would find orphaned higher-numbered files with no `job.json` entry
  pointing at them.
- **GUI success message is one line, not the 8-row table.** Direct user
  request: `format_validation()`'s per-row table is shown on REJECT (needed to
  see which row failed and why), but collapsed to `"Passed all checks[ (with
  warnings)], exported N segment(s) to <path>"` on ACCEPTED -- the table adds
  nothing once every row already passed.
- **Export button gates on a truthy (possibly partial) `precompute_joint_path`**,
  not full completion -- same idiom the existing precompute/playback controls
  use, and matches `build_export_segments()`'s own documented behaviour of
  exporting a paused precompute's solved prefix.

### Bug found in review, fixed before this entry: toolpath_source/precompute mismatch

`export_active_job()` reads `precompute_joint_path`/`precompute_waypoints` but
names the job folder and picks `surface.obj` from `self.toolpath_source`.
Switching the GUI's "Toolpath Source" radio does **not** touch
`precompute_joint_path` -- only pressing "Run Precompute" again does (via the
existing layer-mixup guard already inside `run_curved_toolpath_ik_precompute`/
`run_toolpath_ik_precompute`). So: solve RX, switch the radio to TX *without*
re-running precompute, click "Export IK Job" -- it was silently writing RX's
actual solved joints/positions into `assets/export/TX/` together with TX's own
`surface.obj`, reporting success. `validate_job()`'s rows never catch this --
they only check a job's internal consistency, never that it's the source the
button claims it is.

**Fixed** by adding the same `precompute_cache_path` comparison
`_init_toolpath_playback()`/`_init_curved_toolpath_playback()` already use, as
the first statement in `export_active_job()`:
```python
expected_cache_path = (GCODE_PRECOMPUTE_CACHE if self.toolpath_source == -1
                        else curved_precompute_cache_path(self.curved_layer_names[self.toolpath_source]))
if self.precompute_cache_path != expected_cache_path:
    self.export_status = "Run Precompute for the active toolpath source first"
    return
```
Verified: the exact repro above now reports "Run Precompute for the active
toolpath source first" and writes nothing; the matching-source case still
exports normally.

### Verified without the ~20min/layer geodesic rebuild

Both curved layers' full pipeline (geodesics -> print order -> orientation
frames -> IK precompute) only needs re-running to reach a **fresh** solve;
`curved_rx/tx.precompute.npz` (S1.48's fresh caches) already hold a valid one.
Loading a cache directly -- `load_toolpath_precompute_cache(cache_path,
meta_builder=lambda: json.loads(np.load(cache_path)["meta"].item()))`, i.e. a
`meta_builder` that trivially matches the cache's own stored meta -- restores
`precompute_joint_path`/`precompute_waypoints`/`precompute_R_target` without
rebuilding geodesics/print order/orientation frames at all (those have no
cache of their own; only the IK precompute does). Used to verify this stage in
under a minute instead of ~40 (both layers), at the direct cost of skipping
the cache-key equality check the GUI path always runs -- acceptable for a
verification run, since the cache's provenance was already established by
S1.48's own fresh solve.

Results: RX (35 segments, 2,527 points) and TX (35 segments, 2,000 points)
both exported `ACCEPTED`. Round-trip checked: `segment_0_solution.json`'s
`tcp_xyz_base_mm`/`normal_base` matched `toolpath_T0.ply`'s row to
<0.0001mm, `num_points` matched the `.ply` line count, `identity_check`
matched the spec's published reference pose. Pruning verified by re-exporting
a 3-segment subset and confirming files 3-34 were removed, then restoring the
full 35-segment export.

**Non-revertible unless:** the exchange spec's folder structure or field names
change, or a different export-folder convention is explicitly requested --
`assets/export/<job_name>/` was a choice made to unblock this stage, not
derived from anything in the spec itself.

**Verified on:** 2026-09-03.

## S1.50 Job export chunked across frames + Cancel button (roadmap 7.5 follow-up) -- `write_job_export()` replaced by `step_export_job()`/`_flush_export_segment()`/`_finish_export_job()`

**Decision:** `write_job_export()` (S1.49's single blocking call) is deleted.
`export_active_job()` now only self-checks (`build_export_segments()` +
`validate_job()`, unchanged) and, on ACCEPTED, seeds chunked write state
instead of writing. `VisContent.step_export_job()` -- called every frame from
`gui_panel.UI_Menu.render()`, mirroring `step_toolpath_ik_precompute()` --
advances the write `EXPORT_CHUNK_SIZE` (2000, unmeasured default) points at a
time, flushing each segment's `toolpath_TN.ply`/`segment_N_solution.json` via
`_flush_export_segment()` as soon as its points are done, then writing
`job.json` + `surface.obj` via `_finish_export_job()` once every segment is
flushed. `gui_panel.py` gets a `psim.ProgressBar` (same idiom as
Precompute/Geodesics) and a "Cancel Export" button
(`VisContent.cancel_export_job()`, mirroring `cancel_toolpath_ik_precompute()`
-- prunes whatever segment files were already flushed via
`_prune_stale_export_files(job_dir, keep=0)`).

**Reason:** the planar job is 181,375 points (S1.47's measured count); writing
it was one `Button("Export IK Job")` click freezing the GUI for the whole
duration, with no feedback and no way to tell it apart from a hang.

### Bugs found in review, fixed before this entry

**1. Toolpath-source race.** `_finish_export_job()` read the *live*
`self.toolpath_source` to pick which layer's `surface.obj` to copy, while
`export_job_dir`/`export_segments` were captured from the toolpath source at
`export_active_job()`-start. Because the write now spans many frames instead
of one call, a user could solve RX, click Export, switch the "Toolpath
Source" radio to TX before the chunked write finished, and get RX's solved
path shipped with TX's `surface.obj` in `assets/export/RX/` -- silently
wrong, reported as success. Same bug *class* S1.49 already found and fixed
once (see above), but that fix only guards the moment `export_active_job()`
starts; chunking reopened the window for the whole write's duration.
**Fixed** by capturing `self.export_toolpath_source = self.toolpath_source`
alongside the other `export_*` state at export-start and reading that (never
live `toolpath_source`) inside `_finish_export_job()`; the "Toolpath Source"
radio's `BeginDisabled` in `gui_panel.py` now also gates on `export_running`,
belt-and-suspenders so the switch can't even be attempted mid-export.
Verified: started an RX export, flipped `toolpath_source` to TX mid-write,
confirmed the finished job's `surface.obj` still matched RX's source file,
not TX's.

**2. `export_running` stuck `True` on a write failure.** `_finish_export_job()`
did the `surface.obj` copy and `job.json` write *before* clearing
`export_running`. A failure in either (e.g. a missing `surface_file`) left
`export_running` `True`; the next frame's `step_export_job()` saw
`export_seg_index >= len(export_segments)` and re-entered
`_finish_export_job()` immediately, failing identically forever -- once per
frame, nothing surfaced to `export_status`, the Export button stuck
disabled. `step_toolpath_ik_precompute()`'s equivalent tail avoids this by
clearing `precompute_running` before its own I/O. **Fixed** by wrapping the
copy + `job.json` write in `try/except OSError`, the same "fail closed with a
status message" convention `run_toolpath_ik_precompute()` already uses for
its G-code file I/O: on exception, `export_running` is cleared and
`export_status` reports the failure; on success, unchanged. Verified: forced
the `job.json` write to fail by pointing `export_job_dir` at a nonexistent
path, confirmed `export_running` became `False` with a failure message and a
second `step_export_job()` call was a no-op (no retry loop).

### Unrelated, same pass: precompute terminal status shortened

Direct user request, not a bug: `_finish_candidate_search()`'s terminal
`precompute_status` was `"Solved N waypoint(s) -- A-B candidates/waypoint,
path cost C"`; it's now just `"Solved N waypoint(s)"`. The candidate-count/
path-cost detail is no longer computed or shown.

**Verified on:** 2026-09-04.

## S1.51 Nozzle mesh replaces the "Tool Axis" line, aimed along the TCP frame's Z -- the axis the curved pipeline actually commands

**Decision:** `load_data()` no longer hides the "Nozzle" mesh
(`nozzle_handle.set_enabled(False)`, S1.43), and no longer registers the
"Tool Axis" stalk that stood in for it while it was hidden. That curve
network is deleted outright, along with `TOOL_AXIS_COLOR`/
`TOOL_AXIS_RADIUS_MM`, so `apply_delta_transform`'s loop is `range(9)`, not
`range(10)`.

The mesh is not rendered as exported. `nozzle.obj`'s native CAD pose was
modelled against the retired `TCP.txt` point rather than the real tool=1
offset, so as-authored it points at empty space -- its own tip lands on the
old `TCP.txt` world point, 310.97mm from where the calibrated TCP actually
is. It is therefore rigidly re-aimed once, at load time:

- **Direction:** the **shaft's** long axis is rotated onto
  `-T_zero_tcp[:3, 2]` -- the TCP frame's **-Z**. `-Z` and not `+Z` because Z
  points *out* of the print surface: the tip goes in along -Z and the body
  trails behind it along +Z (checked, not assumed: `(centroid - tip) . Z =
  +67.22`). Unit by construction, being a rotation-matrix column.
- **Anchor:** the vertex farthest from the flange origin (the asset's own
  tip) is pinned exactly onto `tcp_point`, and is the rotation pivot.
- **Roll** about that axis is not pinned -- the nozzle is rotationally
  symmetric about its own axis, the same reasoning as S1.36's frames.

**Why Z, and not the flange-centre -> TCP line.** The flange->TCP chord was
tried first and looks better -- it puts the tool body against the flange --
but it is not an axis the robot has any notion of, and it sits **36.32
degrees** off the TCP frame's Z. Z *is* the tool axis by this project's own
convention: `_orientation_frames_for_points` builds every curved `R_target`
with "Z is the outward surface normal" and S1.36 has the nozzle approaching
along -Z. So on the chord the render contradicted the commanded orientation
by 36 degrees, in a study whose whole subject is holding the nozzle
perpendicular to a curved shell. The rendered tool now shows the orientation
IK is actually solving for; the nozzle body and the TCP triad's blue axis are
collinear by construction, which makes the commanded approach direction
readable straight off the screen.

**Accepted cost: the tool floats.** Closest nozzle-to-`Robot6` approach goes
from 14.64mm to **98.33mm**, and the shaft centreline now misses the flange
face centre by 116.6mm. This is an artefact of the placeholder asset rather
than of the alignment: `nozzle.obj` is 163.47mm flange-to-tip against
tool=1's measured 196.91mm, and the real head is mounted at a compound angle
(~87/-13/61 degrees), so no re-aiming makes this asset both faithful and
attached. A corrected asset would satisfy both at once -- the same open item
S1.43 records ("needs a corrected tool asset, not another filter"). Judged
the right trade: a 36-degree orientation error is substantive, a visual gap
is cosmetic.

**The shaft's axis, not the whole mesh's** (`_nozzle_shaft_mask` +
`_obb_from_points`, the same PCA helper the 7.4 collision proxies use). This
distinction is load-bearing and was measured: the mounting bracket is a slab
alongside the shaft and drags a whole-mesh PCA **6.59 degrees** off the
shaft's true axis -- which would leave the rendered shaft 6.59 degrees off
the commanded approach axis, a smaller version of exactly the error the
alignment exists to remove. Fitting the shaft parts alone lands it at
**0.0000 degrees**, at every pose.

`nozzle.obj` fuses **7 rigid parts** into one mesh with no OBJ groups, so
`_nozzle_shaft_mask` labels components with a union-find over the faces
(`trimesh.split()` needs scipy or networkx, neither of which this environment
has -- same constraint as S1.47) and keeps the ones whose oriented box is
slender *and* narrower in cross-section than any bracket component -- the
signature of a turned part. (The shaft parts are also round in cross-section
and the bracketry mostly isn't, but that is an observation, not a test: the
width cut alone separates the two groups, so there is no roundness check in
the code.) Measured, the split is unambiguous: shaft half-extents
`[6.25, 6.25, 41.50]`,
`[11.00, 11.00, 40.75]`, `[5.48, 5.48, 12.71]` against bracketry
`[8.86, 34.66, 48.84]`, `[7.20, 15.04, 16.04]`, `[15.00, 17.51, 48.69]`, so
`NOZZLE_SHAFT_MAX_HALF_WIDTH_MM = 12.5` sits in a wide gap rather than on a
boundary.

**Both degenerate branches fail visibly, not silently** -- neither fires on
this asset, and both fire the moment a *different* one is dropped in, which is
the whole open item S1.43 records. If no component looks like a turned part,
`_nozzle_shaft_mask` returns the whole mesh rather than an empty selection,
which would hand `_obb_from_points` an empty array (`mean()` of nothing is
NaN, and the tool would vanish from the render entirely instead of merely
sitting 6.59 degrees askew). And if the native axis lands *antiparallel* to
the target, the 180-degree turn is built as a real rotation about an arbitrary
perpendicular axis, not as `-I`: `-I` aims the axis correctly but has
determinant -1, so it would point-invert the mesh and reverse every face
winding. Both were review findings on this stage's own first cut, fixed
before it landed.

**The flange face centre IS the DH frame-6 origin** -- worth recording,
because "anchor it to the outer surface rather than the middle of the flange"
is an intuitive-sounding lever on where the tool sits, and it is a no-op.
`Robot6` spans **-46.800mm to 0.000mm** along the flange Z axis relative to
that origin: the body extends *backwards* and the outer face is a flat
annulus at exactly 0 (bore radius 16.25mm, outer 31.00mm). That is the
convention that makes `TCP_OFFSET_6D_MM_DEG` a face-relative tool offset.
Anchoring is not adjustable; the axis choice is the only real lever.

The output is still zero-pose world-frame rest data, so `Delta_6` and
`apply_delta_transform` carry it unchanged, exactly like every other
flange-mounted structure. Because both the tool offset and the alignment
target are fixed in the flange frame, computing the alignment once at load
is exactly equivalent to recomputing it every frame -- a cache, not an
approximation.

**Reason:** the tool needed to be visible again, and the stalk was only ever
a stand-in for it. Aiming the mesh down the tool's own axis is what makes the
render *mean* something: the nozzle now points where the solver says the tool
points.

**This does not resolve S1.43's asset mismatch.** The nozzle's own
shape/length is exactly as uncalibrated as it was while hidden (163.47mm
flange-to-tip against tool=1's real 196.91mm) -- only its render pose
changed. It therefore stays excluded from `moving_geometry_rest_verts`/
collision filtering, unchanged from S1.43: the tool's collision body is still
the TCP point alone. Nothing about the TCP itself moved --
`TCP_OFFSET_6D_MM_DEG`, the TCP point, the TCP frame and IK are all
untouched.

**Verified on:** 2026-09-04. Shaft axis vs the TCP frame's Z **0.0000
degrees** and tip pinned to `tcp_point` at **0.0000mm** -- both measured
against the *current* frame at six joint configurations (zero, J6 spin,
wrist-only and three whole-arm poses), confirming the load-time alignment is
exact at every pose and not merely at the zero pose it is computed at.
Closest nozzle-to-`Robot6` approach 98.33mm, as expected. `tcp_world` still
matches `compute_fk(q)[5] @ T_flange_to_tcp` under a moved pose. Confirmed on
screen: the shaft renders collinear with the triad's blue axis, which
foreshortens to almost nothing when viewed down the tool.

## S1.52 `run_*_toolpath_ik_precompute()` gains a third mode -- "already complete" returns a status instead of resuming a finished run into a crash

**Decision:** both precompute runners early-return with
`"Already solved N waypoint(s)"` when `precompute_index >= precompute_total`,
after the fresh-start branch and before `precompute_running = True`. S1.14
item 1 described two modes ("start fresh" when `precompute_waypoints is None`,
"resume from `precompute_index`" otherwise); *complete* is a third state that
neither covers, and it was falling into the resume path.

**The crash this prevents.** Clicking "Run Precompute" a second time after a
solve finished was an unhandled exception, not a no-op:
`_finish_candidate_search()` empties `precompute_cand_joints` (roadmap 7.4 --
a curved layer's candidate arrays are hundreds of MB and nothing reads them
after the backtrack), so a re-run set `precompute_running = True`, stepped a
zero-length chunk (`end = min(index + chunk, total) == index == total`),
re-entered `_finish_candidate_search()` with `n = 0` and raised `IndexError`
on `chosen[-1] = chosen_last` against an empty list -- inside the per-frame
Polyscope callback. `precompute_waypoints` is deliberately left set after
completion (S1.49's export path reads it), so "loaded" could not distinguish
"paused" from "finished" on its own; the index/total comparison can.

**Accepted consequence: a completed solve is not re-checked for staleness.**
`precompute_cache_meta` is only compared against a freshly-built meta on the
fresh-start branch, so after a build-plate move the finished solve is stale
but the guard still reports "Already solved". The escape is **Cancel, then
Run** -- `cancel_toolpath_ik_precompute()` clears `precompute_waypoints`, so
the next Run rebuilds waypoints and re-keys the cache. Deliberately not
closed in code: pre-7.7 that same click raised, so the guard is a strict
improvement, and folding a staleness re-check into it would widen a crash fix
into a behaviour change with its own verification burden. Recorded here
rather than fixed.

**Reason:** a finished precompute is a normal state a user lands in (solve,
then look at the result, then click Run again out of habit), and it should
report what happened rather than raise.

**Verified on:** 2026-09-04. Re-running a completed solve reports "Already
solved N waypoint(s)" with no traceback; Pause mid-run still resumes from
`precompute_index` and Cancel -> Run still starts fresh, so S1.14's two
original modes are untouched.

## S1.53 Coordinate-frame triad radius is relative to its own axis length, not a flat absolute constant

**Decision:** `create_coordinate_frame()`'s curve-network radius is
`scale * FRAME_AXIS_RADIUS_RATIO` (`FRAME_AXIS_RADIUS_RATIO = 0.05`), not a
flat `set_radius(N_MM, relative=False)`.

**The bug this fixes.** The TCP/User Frame triads originally had no explicit
radius at all, leaving Polyscope's default *relative* radius in effect --
`radius_relative * automatic_length_scale`, where `automatic_length_scale` is
recomputed from the bounding box of every registered structure. During
toolpath playback the "G-code Print" bead mesh is repeatedly re-registered
with more geometry (growing the scene bounding box), so the triads visibly
changed thickness as playback progressed even though their own geometry
never changed. Pinning an absolute radius (matching `CURVE_RADIUS_MM`,
`TRAJECTORY_RADIUS_MM`, `CURVED_ORDER_FEED_RADIUS_MM`'s existing pattern)
fixes that -- but a single flat constant is wrong here, because
`create_coordinate_frame()` is shared by callers at different `scale`s and a
radius tuned for one is wrong for the others. A flat 2.5mm (right at 50mm
axes) renders a 1mm triad as a blob, radius wider than the axes are long.

**Two follow-ons found in review, both the same root cause.** Pinning the
triads exposed structures still on the scene-scaled default:

1. **The world-origin frame was left invisible.** It used
   `create_coordinate_frame()`'s bare `scale=1.0` default -- 1mm axes, which
   are sub-pixel in a ~2400mm scene. It had only ever been visible because
   the default relative radius inflated it to a 12mm-thick blob; given a
   proportionate 0.05mm radius it disappeared entirely. Fixed by giving it a
   real length, `WORLD_FRAME_SCALE_MM = 100.0` -- a 100mm triad at 5mm thick.
   The 1mm default was never a viewable triad, only an accidental blob.
2. **The "TCP" point cloud was a 24mm ball.** Registered with no explicit
   radius, so it drew at the same scene-scaled 12mm -- previously masked by
   the equally fat default-radius triad sitting on top of it, and left
   standing proud once the triad thinned. Pinned to
   `TCP_FRAME_SCALE_MM * FRAME_AXIS_RADIUS_RATIO` (2.5mm), matching the
   triad's own tube thickness so it reads as the triad's origin. Kept
   visible and registered: it is the tool's collision body and holds a fixed
   index in `rest_verts`/`update_fns`/`mesh_handles`.

**Reason:** shared triad infrastructure serving callers at different scales
needs a scale-relative visual parameter, not a constant tuned for one of
them -- and every structure's radius wants pinning absolute, since a relative
one is a function of the whole scene's bounding box and therefore changes as
the print mesh grows during playback.

**Verified on:** 2026-09-05. `scale * FRAME_AXIS_RADIUS_RATIO` gives 2.5mm at
the TCP/User frames' `scale=50.0` (unchanged from the value visually tuned
there) and 5mm at the world-origin frame's `scale=100.0`. Polyscope's default
was measured at 12mm radius in a 2400mm scene, confirming both the original
pulsing and why the TCP ball and the world triad behaved as they did.

## S1.54 Job export also writes a dated .zip of the job folder; its failure is reported separately from the folder write

**Decision:** `_finish_export_job()` writes the zip in its own
try/except OSError, after (not inside) the try/except that guards
`surface.obj`/`job.json`. GUI adds a free-text "Export Name" field
(`gui_panel.py`), sanitized and captured into `export_zip_name` at
`export_active_job()`-start (same reasoning as `export_toolpath_source`'s
capture, S1.50) -- falling back to the job folder's own name
(`os.path.basename(job_dir)`) if the field is blank *or* sanitizes down to
only underscores (i.e. an input made *entirely* of the reserved characters
`\x00-\x1f&lt;&gt;:"/\|?*` -- ordinary punctuation like `!!!` is not sanitized and is
kept as typed). Filename:
`EXPORT_DIR/<YYYYMMDD>-<name>.zip`, UTC date (matching `job.json`'s
`generated_utc` convention) -- built via `shutil.make_archive(zip_base,
"zip", root_dir=dirname(job_dir), base_dir=basename(job_dir))` so the job
folder lands as one top-level directory inside the zip rather than loose files
at the archive root -- 72 for a curved layer, ~40,000 for the planar job.

**Why the zip's try/except is separate.** A first pass wrapped the zip call
in the *same* try/except as the `job.json`/`surface.obj` write (both fail
closed the same way per S1.50's convention). Caught in review: by the time
the zip step runs, `job_dir` is already a fully complete, valid job --
zipping it is a convenience, not a second critical write. A zip-only
failure (disk full, AV lock on the just-written files) doesn't destroy
anything, so reusing the critical-write's fail-closed handler wrongly
reported a successful export as "Export failed writing `<job_dir>`" (the
wrong path, too) and discarded `export_segments`/`export_job_meta` state
over an artifact that was never lost. The zip's own try/except instead
folds a failure into `export_status` as a trailing note (e.g. "... exported
to `<job_dir>` (zip failed: ...)") while still reporting the export itself
as passed.

**Reason:** a convenience step layered on top of an already-durable write
must not be able to make that write look like it failed.

**Verified on:** 2026-09-04. `shutil.make_archive` with `root_dir`/`base_dir`
confirmed (standalone test) to produce a zip containing the job folder as a
single top-level entry. Sanitize+fallback confirmed against blank,
whitespace-only, all-invalid-character, and normal inputs.

## S1.55 Playback render stride is derived per playback from the path's own joint motion, superseding S1.18's fixed `PLAYBACK_RENDER_STRIDE = 50`

**Decision:** `PLAYBACK_RENDER_STRIDE` is gone. `advance_toolpath_playback()`
now throttles on `self.playback_render_stride`, computed once per playback by
`_derive_playback_render_stride()` from the solved joint path:

```python
deg_per_waypoint = median(|diff(joint_path)|.max(axis=1))
stride = clip(round(PLAYBACK_RENDER_DEG_PER_PUSH / deg_per_waypoint), 1, PLAYBACK_RENDER_STRIDE_MAX)
```
with `PLAYBACK_RENDER_DEG_PER_PUSH = 5.0` and `PLAYBACK_RENDER_STRIDE_MAX = 50`.

**The bug this fixes.** The user reported curved playback "steps a lot" at 1x
while planar looks smooth. S1.18's stride is a fixed *waypoint count*, which
is only a fixed *visible* step if joint motion per waypoint matches. Measured
against the real cached joint paths, it doesn't:

| | waypoints | deg/waypoint (median) | deg per 50-waypoint push | pushes per print |
|---|---|---|---|---|
| planar | 181,375 | 0.095 | 4.75 | 3,628 |
| curved RX | 3,175 | 0.903 | **45.13** | **64** |
| curved TX | 2,688 | 0.828 | 41.40 | 54 |

Two compounding causes, both measured: curved has ~57x fewer waypoints, and
its tool must reorient continuously to stay normal to the surface, so each
waypoint carries ~10x the joint motion. Per-waypoint *spacing* is nearly
identical (~1.6mm both), so distance is not the discriminator -- joint motion
is, which is what the new derivation normalises on. Planar additionally hides
its stride because the slicer zigzags: 50 points of zigzag is only ~15mm net
displacement versus ~51mm on a curved monotone sweep.

**Why not simply push every frame.** Benchmarked first (S1.20 methodology --
real `ps.show()` loop, `perf_counter()` deltas, `ps.unshow()` after 600
frames, `step_count=1` worst case; explicitly *not* the `screenshot_to_buffer`
approach S1.20 retracted). Curved layer; each run is 600 frames, of which the
569 after a 30-frame warm-up are timed. Push counts are over all 600:

| condition | stride | pushes | median | p99 | max |
|---|---|---|---|---|---|
| S1.18 fixed | 50 | 13 (2%) | 15.83ms (63.2fps) | 40.5ms | 47.1ms |
| **derived (adopted)** | **6** | **101 (17%)** | **15.86ms (63.0fps)** | 44.4ms | 75.3ms |
| every frame | 1 | 600 (100%) | 39.44ms (25.4fps) | 46.0ms | 48.9ms |
| arm every frame, bead on 50 | -- | 13 | 38.25ms (26.1fps) | 74.6ms | 87.1ms |

Pushing every frame costs ~24ms/frame and halves the framerate to 25fps, so it
was rejected. The harness was checked against S1.20's ~15.8ms figure on the
**planar** path it was originally measured on -- the planar run here
reproduced 15.83ms with its derived stride of 50, i.e. the unchanged
configuration. The curved stride-50 row landing on the same 15.83ms is a
coincidence of two different scenes, not the cross-check.

**Accepted consequence: splitting the arm push from the bead push buys
nothing.** The obvious fallback -- arm every frame (cheap, fixed-size link
meshes) with the expensive growing bead buffer left on the coarse stride --
was measured and is *not* cheaper: 38.25ms vs 39.44ms. The arm is ~23ms of
the ~24ms push cost (241k verts across 9 link meshes, versus only ~25k for
curved's entire bead buffer). The conditional is therefore left unsplit, and
the tunable is push *frequency*, not which half pushes.

**Reason:** the throttle's purpose is bounding render cost, but its parameter
should be the thing the eye actually sees -- motion per push -- not waypoint
count, which is a proxy that holds only for one toolpath source.

**Verified on:** 2026-09-04. Derived strides against the real caches: planar
50 (4.75 deg/push -- **bit-for-bit unchanged**, the cap does the work),
curved RX 6 (5.42 deg/push, was 45.13), curved TX 6 (4.97, was 41.40) -- an
~8x smoothness gain on curved at an unchanged 63fps median. Degenerate paths
guarded: empty and single-pose return 1, a never-moving path returns the cap,
a NaN path returns the cap rather than raising `round(nan)` out of the frame
callback, and a path that is >half duplicate points still derives from its
*moving* steps (a plain median would read 0 there and pick the coarsest
stride -- the worst stepping, exactly backwards). Reviewed 2026-09-05.

## S1.56 Playback resume re-validates the loaded precompute against the active toolpath source

**Decision:** `run_toolpath_playback()` re-initializes, rather than resumes,
whenever `precompute_cache_path` is not the one the active `toolpath_source`
solves into. That mapping now lives in one place,
`_expected_precompute_cache_path(source=None)`, called by the resume guard,
`_init_curved_toolpath_playback()` and `export_active_job()`.

**The bug this fixes.** `run_toolpath_playback()` only called an initialiser
when the bead arrays were `None`, but the source guard *and* (since S1.55) the
render-stride derivation both live inside those initialisers. Reproduction:
play RX -> Pause -> switch Toolpath Source (the radio is gated on
`playback_running`, which Pause clears, so this is reachable) -> Run
Precompute, which loads TX from cache -> switch back to RX -> Run. RX's beads
already exist, so no initialiser ran and nothing revalidated: the arm followed
**TX's** 2,688-pose joint path while revealing **RX's** beads, ending ~500
waypoints early with most of RX's beads still hidden. Pre-existing since 6.6;
S1.55's per-path stride would have added a second stale value on top.

**Reason:** "the beads exist" answered a different question than "the loaded
solve belongs to what is selected". Three hand-written copies of the
source->cache-path expression is how the guard came to be enforced in some
entry points and not others.

**Verified on:** 2026-09-05. The mapping returns the planar cache for source
-1 and the per-layer curved caches for 0/1; the stale test is True for
"RX active, TX loaded" (forces re-init) and False for a matched pair, so an
ordinary Pause -> Run still resumes from `playback_index`. A `None` cache path
also reads stale, which is harmless: it only occurs with an empty joint path,
which the initialisers already refuse with "Run Precompute first".

## S1.57 `playback_waiting` and the frontier-chasing paths removed as unreachable since 7.4

**Decision:** deleted `playback_waiting`, the `waiting` local, the
`(waiting and moved)` clause in the render conditional, the
"Waiting for precompute" status branch, and the Speed slider snap-down in
`gui_panel.render()` (with its now-unused `PRECOMPUTE_CHUNK_SIZE` import).
`advance_toolpath_playback()`'s docstring is rewritten to match.

**Why they were dead.** S1.14's model -- playback starts against a partially
solved path and chases a live frontier -- stopped being reachable at 7.4.
`precompute_joint_path` is only ever assigned whole: `[]` on reset/abort,
`list(joint_path)` on a cache load, and one atomic comprehension in
`_finish_candidate_search()`. Nothing appends to it. `_finish_candidate_search()`
runs only under `precompute_index >= precompute_total`, and a cache load sets
`index = total = len`; a solve that fails, fails the whole run rather than
leaving a prefix. So `exhausted` is true whenever the path is non-empty, and
the initialisers refuse an empty one -- making `waiting = at_frontier and not
exhausted` permanently `False`, and the GUI snap-down that read it dead too.

**Reason:** unreachable state that looks live is worse than no state. This
machinery actively misled the S1.55 investigation, which read the
`(waiting and moved)` clause as a real per-frame path and had to rule it out
by tracing every writer of `precompute_joint_path`.

**Restoration condition -- read this before reintroducing chunked path
filling.** If `precompute_joint_path` is ever filled incrementally again (e.g.
to let playback start against a partial solve, S1.14's original intent), this
entry must be reverted *and* `playback_render_stride` must be re-derived as
the path grows -- S1.55 derives it once at playback init, which is only sound
because the path is complete by then. A note to that effect sits on
`_derive_playback_render_stride()` itself.

**Verified on:** 2026-09-05. Traced every write to `precompute_joint_path`
(5 sites, all wholesale) and to `precompute_index` before removing anything.
Post-removal the curved benchmark is unchanged (stride 6, 63fps median) and
playback still terminates on the `finished` branch with "Playback complete".

## S1.58 Build plate loads the saved calibrated User Frame at startup, superseding S1.6's "never automatically at startup"

**Decision:** `VisContent.__init__` now calls `_load_startup_build_plate()`,
which applies `assets/buildPlate/saved_position.json` when it is readable and
falls back to `USER_FRAME_ORIGIN_MM` when it is absent or malformed. The chosen
pose is reported through `startup_plate_status`, which the GUI seeds `bp_status`
from, and `load_build_plate()` now retains the applied `(position_mm, rpy_deg)`
in `build_plate_pose` so `UI_Menu.__init__` seeds its Target Position/RPY fields
from the pose actually applied. The **Reset** button still means
`USER_FRAME_ORIGIN_MM`. `load_saved_build_plate_position()` gained a `try/except`
around the file read and returns a status instead of raising.

This supersedes the final clause of **S1.6**, which specified that loading is
"only ever triggered by that explicit button click, never automatically at
startup, which still always begins from `USER_FRAME_ORIGIN_MM`/zero-rotation".

**Reason:** the shipped curved precompute caches are keyed on the plate pose and
were solved at the real calibrated User Frame ([649.456, 133.762, 322.778], rpy
[-0.369, 0.329, -89.080]) adopted by S1.45. Booting at `USER_FRAME_ORIGIN_MM`
meant every cache check missed, so a first run re-solved ~3,175 (RX) / ~2,688
(TX) waypoints at the measured 437-749ms each -- around half an hour per layer at
1.3-2.3fps -- with nothing on screen indicating that a cached solve existed. It
was also not the same job: `CURVED_MODEL_XY_OFFSET_MM = (0,0)` is measured for
100% reachability at the saved frame specifically (S1.48), so the default pose
silently offered an unvalidated configuration.

S1.6's stated objection was to loading the pose *silently* ("without forcing
every future startup to load it silently"). The status message is the direct
answer to that objection, and Reset keeps the demo pose one click away -- so this
supersedes the mechanism while honouring the reason behind it.

**Non-revertible unless:** the shipped caches are regenerated at
`USER_FRAME_ORIGIN_MM`, or the plate pose leaves the cache key. Note the GUI
field seeding is load-bearing, not cosmetic: fields hard-coded to
`USER_FRAME_ORIGIN_MM` while the plate sits elsewhere would make an unedited
**Move** click silently teleport the plate back to the demo pose.

**Verified on:** 2026-09-06. Cache metas read directly from
`curved_rx/tx.precompute.npz` confirm the stored `user_frame` is the saved pose,
not `USER_FRAME_ORIGIN_MM`.

## S1.59 Precompute cache key includes the tuned solver constants

**Decision:** both `_toolpath_cache_meta()` and `_curved_toolpath_cache_meta()`
gained a `solver` field from the shared `_solver_cache_fields()` helper --
`TCP_OFFSET_6D_MM_DEG`, `PHYSICAL_JOINT_LIMITS`, the `FILTER_*` values plus
`SELF_COLLISION_PROXY_SEGMENT_MM` / `LINK_SAMPLE_SPACING_MM` /
`SURFACE_GRID_CELL_MM`, and the `EDGE_*` costs. The curved meta additionally
carries `tip_clearance` (`CURVED_TIP_CLEARANCE_TOLERANCE_MM`, filter 8's
clearance), deliberately kept out of the planar key because filter 8 never runs
on the planar path -- a curved-only retune must not invalidate a planar cache.

Existing caches were **migrated in place** (meta rewritten, arrays untouched)
rather than invalidated, so `PRECOMPUTE_CACHE_VERSION` stays 7.

**Reason:** the key covered the source geometry, the plate pose, the filter mode
and the orientation-search shape, but none of the constants that decide *which
candidate survives*. Retuning a filter threshold, a joint limit, an edge cost or
the TCP offset therefore left the key identical, and the next Run Precompute
served the joint path solved under the OLD values with no warning. That is worst
for exactly the person adapting this to their own job, whose first action is to
retune those constants. `PRECOMPUTE_CACHE_VERSION` was the only lever and it is
manual and undocumented for users.

**Non-revertible unless:** the filter stack stops reading these constants. If a
new tuned constant is added to the filter or edge machinery, it belongs in
`_solver_cache_fields()` -- otherwise this class of silent staleness returns.

**Verified on:** 2026-09-06. Migration re-read each cache and confirmed the
stored `solver` block equals `_solver_cache_fields()` (so the caches still hit)
with `joint_path` shapes intact at (3175, 6) and (2688, 6); mutating
`FILTER_J5_MIN_DEG` or `EDGE_BRANCH_CHANGE_PENALTY` changes the key, and
restoring the value restores it.

## S1.60 Study config is selected by the `FR5_STUDY_CONFIG` environment variable

**Decision:** `geometry_backend.py`'s single hard-coded
`from examples.curved_surface_printing.study_config import (...)` is replaced by
an `importlib` resolution of
`os.environ.get("FR5_STUDY_CONFIG", DEFAULT_STUDY_CONFIG)`. The required names
are listed in `_STUDY_CONFIG_NAMES` and read off the module by name, so a config
missing one fails at import naming the offender. `CURVED_MODEL_DIR` is then
anchored via `_asset_path()`, leaving an absolute path untouched so a study may
keep assets outside the repo. Unset, behaviour is byte-for-byte unchanged.

**Reason:** S1.33 established the generic-engine / study-config split, and the
README already told users to "swap it for a different curved-print job" -- but
the only way to do so was to edit `geometry_backend.py`, one of the two files
AGENTS.md section 3 designates as the simulator core. An environment variable
keeps the single import seam S1.33 intended while making the documented action
actually possible without a source edit, and `main.py` stays untouched.

**Non-revertible unless:** more than one study must be loaded simultaneously, at
which point this becomes an instance parameter rather than a module-level
constant -- a much larger change, since `CURVED_LAYERS` and friends are read as
module globals throughout.

**Verified on:** 2026-09-06.

## S1.61 Asset paths are anchored to the source directory, not the process CWD

**Decision:** `MESH_DIR`, `PRINTER_HEAD_DIR`, `BUILD_PLATE_DIR`, `GCODE_DIR`,
`EXPORT_DIR` and the study config's `CURVED_MODEL_DIR` are resolved through
`_asset_path()` against `os.path.dirname(os.path.abspath(__file__))`. An
already-absolute path is returned unchanged.

**Reason:** every asset path was relative to the working directory, so the app
only started when launched from the repo root. An IDE Run button, a
`python /path/to/main.py` invocation, or any wrapper script produced a bare
`FileNotFoundError` before the window opened -- a poor first impression with a
non-obvious cause.

**Non-revertible unless:** assets must be relocatable at runtime, which would
want a config entry rather than a return to CWD-relative paths.

**Verified on:** 2026-09-06.

## S1.62 `two_opt` scores candidate reversals by the two cut edges instead of re-summing the tour

**Decision:** `two_opt()` now calls the new `_reverse_delta()` (the change in
total travel from the edge entering the block and the edge leaving it) rather
than building each candidate order and re-summing it with `travel_cost()`. A
sweep is O(N^2) instead of O(N^3). The scan order, the
apply-immediately-and-keep-scanning behaviour, and the acceptance threshold are
unchanged: `delta < -1e-9` is exactly the previous
`candidate_total < best - 1e-9`.

**Reason:** `build_print_order()` calls this synchronously from a button click
with no chunking, progress bar or cancel, so its cost lands as a frozen GUI. At
the shipped N=35 the full re-sum was genuinely trivial, as its docstring said --
but the curved pipeline is generic over whatever a study config describes, and a
job with a few hundred pieces made the freeze minutes long. This is the cheapest
fix that removes the scaling problem without restructuring the call into the
chunked-step pattern.

**Non-revertible unless:** the cost matrix stops being symmetric. The delta
derivation depends on it: reversing a block flips each piece's entry/exit ends,
so an internal hop keeps the same two physical endpoints and is unchanged in cost
*only* because `cost[a,b] == cost[b,a]`. Geodesic distance is symmetric; a
directional cost (e.g. an asymmetric travel-time model) would break this and
require the full re-sum.

**Verified on:** 2026-09-06. Both implementations run against 280 random
symmetric cost matrices (2-35 pieces, including coincident-endpoint closed loops)
returned **identical orders in all 280 cases**; over 300 random reversals the
worst |delta - exact re-sum| was 1.7e-13, four orders of magnitude below the
1e-9 acceptance threshold, so a flipped near-tie is not a practical risk. This
mattered because the print order feeds the waypoint positions hashed into the
curved cache key -- a changed order would have silently invalidated the shipped
RX/TX caches.

## S1.63 Cancelling an export removes the stale job.json, and is a no-op when no export is running

**Decision:** `cancel_export_job()` now also deletes `job.json` from the job
directory alongside the segment files it already pruned, **and** returns early
with "No export in progress" unless `export_running` is true.

**Reason:** two halves of the same hazard, found in the v1.0 review.

The first: cancel pruned every `toolpath_T*.ply` / `segment_*_solution.json`
with `keep=0`, and the old docstring justified leaving `job.json` alone on the
grounds that "job.json is only written by `_finish_export_job()`, so none exists
yet". That holds only for a first-ever export into a fresh directory. On a
**re-export**, the previous run's `job.json` is still present, and the prune has
just deleted every segment file it references -- handing a receiving parser a
manifest pointing at files that no longer exist. Cancelling must leave no job,
not a broken one.

The second follows directly from the first: once cancel deletes `job.json`, a
cancel with nothing in flight would destroy a **completed, valid** export. The
GUI only shows the Cancel Export button while `export_running`, so this is not a
reachable click today -- but a destructive operation must not depend on a GUI
gate for its safety.

**Non-revertible unless:** `job.json` stops being the manifest that names the
segment files, or export gains a resume-from-partial mode -- in which case
cancel would need to distinguish "abandon" from "pause" rather than always
clearing.

**Verified on:** 2026-09-08. With `EXPORT_CHUNK_SIZE` temporarily lowered to 50
so the export stays genuinely mid-flight, cancel leaves no `job.json`, no
partial segment files, and no new zip, while the two zips from previously
completed exports survive; a subsequent stray `cancel_export_job()` after a
completed export leaves that export's manifest and all 35 `.ply` files intact.

## S1.64 Playback/precompute initialisers gate on the state they dereference, not only on the precompute's existence

**Decision:** `_init_curved_toolpath_playback()` now refuses unless the curved
model is loaded AND `curved_order_loaded`/`curved_orient_loaded` are both true,
returning False with a `playback_status` like its sibling guards.
`build_toolpath_waypoints_world()` returns `([], R_target)` for an empty
`gcode_points`, and `_build_gcode_beads()` returns its empty-result tuple for
fewer than two points. `build_curved_toolpath_waypoints_world()`'s travel/piece
pairing check raises `ValueError` instead of `assert`, and both callers catch it.

**Reason:** two of these were crashes reachable by ordinary clicks, and every one
of them escaped into the per-frame Polyscope callback, which kills the window
rather than showing a message.

1. `cancel_geodesic_precompute()` nulls `curved_print_order` (via
   `_reset_print_order_state`) while leaving `precompute_joint_path` and
   `precompute_cache_path` fully populated. The initialiser gated only on those
   two, so Cancel Geodesics -> Run Toolpath reached
   `self.curved_print_order[layer]` on `None`. Reproduced:
   `TypeError: 'NoneType' object is not subscriptable`. Cancel Geodesics is
   always enabled and sits beside Build Geodesics; clicking Load Curved Model a
   second time after a completed precompute reaches the same state.
2. `parse_gcode()` returns `[]` for any file with no G0/G1 line -- a header-only
   or truncated Cura export. `np.array([])` is 1-D, so `pts_local[:, 2]` raised
   `IndexError: too many indices for array`, and the caller's `if not waypoints`
   guard ran *after* the call while its `except OSError` could not catch it.
   `assets/models/planar/gcode/*.gcode` is gitignored, so every user supplies
   this file themselves. `load_gcode()` already guarded the same input.
3. The `assert` additionally evaporates under `python -O`, which is exactly when
   a silent wrong-travel-move stitch would be worst.

**Non-revertible unless:** the guards move somewhere that provably runs earlier.
The principle to keep: a function that is about to subscript retained state
validates that state itself, rather than trusting a caller's unrelated check --
`precompute_cache_path` says who solved the path, not whether the geometry that
path was built from still exists.

**Verified on:** 2026-09-08. Both crashes reproduced against the running backend
before the fix and re-run after; all four entry points (Run Toolpath, Reset
Toolpath, Run Precompute, Load G-code preview) now decline with a status.

## S1.65 `apply_live_layer_visibility(-1)` is an explicit early return, not a fall-through

**Decision:** the method returns immediately for `layer is None or layer < 0`.

**Reason:** the body computes `visible = (i <= layer)`, which at `layer == -1` is
False for every configured layer -- so selecting the **Planar** toolpath source
with a curved model loaded disabled the entire workpiece: every surface,
ordered-feed overlay, travel network, orientation triad and printed bead mesh.
Nothing restored it, because `gui_panel` only calls this for a non-planar
selection, so re-selecting Planar never helped; the mockup stayed invisible until
the user picked a curved layer again. Two call sites documented the -1 case as "a
safe no-op", which was true only while no curved model was loaded.

**Non-revertible unless:** -1 stops meaning "planar" as the toolpath-source
sentinel.

**Verified on:** 2026-09-08. Reproduced (`Surface RX Offset` enabled True -> False
on `apply_live_layer_visibility(-1)`), and confirmed it stays enabled after.

## S1.66 A plate move marks the curved model stale, and the build chain refuses to run on it

**Decision:** `load_build_plate()` sets `curved_model_stale = True` when the plate
moves under a loaded curved model (alongside the geodesic abort it already did);
`load_curved_model()` clears it, and `run_geodesic_precompute()` refuses to start
while it is set, directing the user to reload.

**Reason:** the pre-existing guard aborted the geodesics and advised "reload the
curved model", but left `curved_model_loaded` True with the retained world
vertices and `T_curved` at the **pre-move** pose, and the structures still
rendered there. Nothing stopped Build Geodesics -> Build Print Order -> Build
Orientation Frames -> Run Precompute being re-run on that stale geometry. The
result would be a solve whose filter 8 bins its surface grid from the old
workpiece position while filters 5-7 use the new plate pose -- a collision test
against a surface that is not where the arm thinks it is. The advisory status was
the only thing in the way, and the next Build click overwrote it.

**Non-revertible unless:** `load_build_plate()` gains the ability to re-place the
curved geometry itself, at which point staleness would be resolved rather than
reported.

**Verified on:** 2026-09-08.

## S1.67 Export validation is deferred to the first `step_export_job()` call

**Decision:** `export_active_job()` builds the segments, sets `export_running`
with `export_phase = "validate"` and a "Validating N point(s)..." status, and
returns. The first `step_export_job()` runs `validate_job()`, and only on ACCEPT
does it `makedirs` + `_prune_stale_export_files` and switch to `export_phase =
"write"`. `export_job_dir` is cleared at the start so a Cancel during validation
cannot prune a previous job's folder.

**Reason:** validation does one `compute_fk` per exported point. Measured: 0.12s
over curved RX (35 segments, 2,527 points) but **6.32s** over planar (20,350
segments, 134,618 points). Running it inside the button click meant nothing
repainted until it finished -- the same freeze `step_export_job()` was created to
eliminate, reintroduced ahead of it. Deferring by one frame lets the status line
and progress bar paint first. Validation itself stays monolithic: 6s does not
justify restructuring `validate_job` into a chunked walk.

**Non-revertible unless:** the ordering is preserved. Validate must stay strictly
before `makedirs`/prune -- the prune is destructive, and a REJECT must write
nothing and delete nothing, including a previously completed export sharing the
job folder. Both moved into the validate phase together for exactly this reason.

**Verified on:** 2026-09-08. A forced REJECT (one joint driven outside
`PHYSICAL_JOINT_LIMITS`) reports the failing row, leaves a previously completed
export's `job.json` and all 35 `.ply` files intact, and writes no zip.

## S1.68 Destructive cancels confirm what they discarded; `_abort_toolpath_ik_precompute` no longer treats "no run" as "planar run"

**Decision:** `cancel_toolpath_ik_precompute()` reports "Precompute cancelled" and
`cancel_geodesic_precompute()` names the cascade it triggered, instead of blanking
their status to empty. `_abort_toolpath_ik_precompute()` tests
`precompute_cache_path == GCODE_PRECOMPUTE_CACHE` rather than
`in (None, GCODE_PRECOMPUTE_CACHE)`. `_clear_gcode_print_mesh()` clears all five
bead arrays plus `gcode_status`; `_abort_toolpath_ik_precompute()` and
`_reset_toolpath_playback_state()` clear `playback_active`; `build_print_order()`
cascades into the curved precompute and bead meshes.

**Reason:** `precompute_cache_path` is None in two unrelated situations -- "a
planar run" and "no run has ever started" -- and conflating them meant Cancel
Precompute tore down a freshly loaded G-code **preview** that no precompute had
anything to do with, then blanked the status so nothing said why. The rest are the
same class of defect S1.42 set the grouped reset helpers up to prevent: state
cleared in one path and forgotten in another. `playback_active` gates the 6.7
overlay force-hide, so leaving it True stranded the guide overlays hidden with
nothing playing; `build_print_order()` invalidated the orientation frames but left
the joint path and bead arrays derived from the order it was replacing, and
`run_toolpath_playback()`'s staleness test only compares cache paths.

**Non-revertible unless:** a status field is added to the G-code or curved
subsystems without also being cleared in its subsystem's reset helper -- the
recurring failure this entry exists to close.

**Verified on:** 2026-09-08. Nine regression groups covering all of the above.

## S1.69 Trajectory curve redraw stride is derived from the point count, not fixed at 5

**Decision:** `record_trajectory_point()` now tests against
`_trajectory_render_stride()`, which returns
`max(TRAJECTORY_CURVE_RENDER_STRIDE, len(trajectory_points) // TRAJECTORY_CURVE_NODES_PER_STRIDE)`
with the new `TRAJECTORY_CURVE_NODES_PER_STRIDE = 1000`.
`TRAJECTORY_CURVE_RENDER_STRIDE` (5) becomes the floor of that derivation rather
than the value itself. No recorded point is discarded; only the redraw interval
changes.

**Reason:** `trajectory_points` is unbounded -- `record_trajectory_point()` appends
up to 10 points/sec while the TCP moves, and only `clear_trajectory()` (the FK
panel's Reset) empties it -- while `_update_trajectory_curve()` is an O(n) full
re-registration, Polyscope curve networks having no incremental grow API. A FIXED
stride therefore fired that growing rebuild at a constant ~2/sec, so redraw work
grew O(n^2) over a session.

Measured at ~0.31us/node: 0.22ms at 500 points, 2.99ms at 10,000, **9.40ms at
30,000**, 18.89ms at 60,000. 30,000 points corresponds to a ~50-minute planar
playback at speed 1. The scenarios that actually occur are far smaller -- curved
RX at speed 1 reaches ~529 points and planar at speed 100 about 302 -- so this was
a real but narrow problem, which is why the response is a derived stride and not a
redesign of the overlay.

Deriving the interval in step with the cost holds the amortised redraw cost flat:
measured 0.05% of wall time at 500 points, 0.37% at 10,000, 0.39% at 30,000 and
0.46% at 60,000.

Directly analogous to **S1.55**, which replaced the fixed `PLAYBACK_RENDER_STRIDE`
with a stride derived from the solved path's own joint motion, for the same reason:
a fixed count only means a fixed cost if the thing being counted has fixed cost.

**Non-revertible unless:** Polyscope gains an incremental grow API for curve
networks, at which point the rebuild stops being O(n) and the whole derivation is
unnecessary.

**Deliberate properties, both load-bearing:**
- The `max()` floor keeps the stride at exactly 5 below 5,000 points, so every
  realistic session -- all curved playback, all speed-100 planar -- behaves
  bit-for-bit as before. This is a tail-case fix that must not perturb the common
  case.
- **No point is discarded.** Capping the list was rejected: the trail is a debug
  overlay whose entire value is showing where the TCP has been, and silent
  truncation would be a behaviour change rather than a performance one.

**Verified on:** 2026-09-08. Derived stride is 5 at 0/500/4,999/5,000 points and
10/30/60 at 10,000/30,000/60,000; amortised cost flat as tabulated above; 2,000
recorded samples all retained in order, and `clear_trajectory()` still empties.

## S1.70 `tutorials/` is published, and the stage READMEs are a reconstruction

**Decision:** `tutorials/` is removed from `.gitignore` and published. Stages 6
and 7 are rewritten to `Stage5_README.md`'s granularity, and corrections that
Stage 7 discovered about Stage 6 are folded back into Stage 6 so the stages read
in build order. The stage READMEs are therefore a **clean reconstruction, not a
chronology**: this file remains the sole authority on what was decided when and
what superseded what. No existing entry in this file, and no `001_Inbox/` note,
was altered.

Specifically migrated into Stage 6: the User Frame-origin placement
(`CURVED_MODEL_XY_OFFSET_MM`, S1.48) into 6.1; the study-config home for job
constants (S1.41) into 6.3/6.6; the reframing of 6.4's frame as *nominal* (the
axis of S1.46's search cone) rather than commanded; the removal of the
tangent-plane clearance check (S1.37/S1.44) from 6.5; and the cache's storage of
waypoints and orientation frames alongside the joint path. Stage 6.8 keeps its
number but becomes a scope statement — what Stage 6 does not check, and why the
three obvious collision models fail — since S1.40's plate plane and
`allow_tcp_through_plate` no longer exist in the code. Correspondingly removed
from Stage 7: the placement root-cause narrative, the tangent-check deletion, the
cache-gap fix, the S1.40-plane framing of filter 6, the 7.1→7.7 nozzle-render
reversal, and the measurement tables taken under superseded conditions
(226/2,527 RX, the planar waypoint-0 abort, the 105.6 mm control run).

No exception: the `tutorials/` rule is removed from `.gitignore` outright, so the
directory is published whole -- seven files, nothing excluded.

The wiki construction guide is not among them. `tutorials/` had carried an
earlier FR5-specific fork ~95% identical to `wiki-template/WIKI_CONSTRUCTION_GUIDE.md`,
differing only in worked examples; that fork was folded out rather than committed
as a second copy, leaving the generalised `wiki-template/` version as the single
canonical guide. `tutorials/README.md` links it under "Method" for anyone wanting
to run the same working method on another project.

**Reason:** ~40 wiki pages and 15 `geometry_backend.py` docstrings cite the stage
READMEs as the roadmap of record, so gitignoring them meant a clone got dangling
citations — `README.md` and `wiki/INDEX.md` each carried a section apologising
for it. Publishing closes that gap.

The reconstruction is the harder half. Stage 6 was built entirely against
stand-ins (a chosen plate pose, a stand-in plate mesh, a stand-in tool, one
commanded orientation per waypoint) and Stage 7 replaced each with a measurement,
so Stage 6's README had accumulated a 25-line superseding block plus 6 inline
amendment banners and Stage 7 a further 9. A reader following that in order would
build code that a later stage deletes. Since these files exist to be *followed*,
teaching order beats chronological order — provided the chronology survives
intact somewhere, which is this file's job.

Where a wrong turn is instructive it survives in the tutorials as a short
`Pitfall:` note rather than a dated reversal — notably the two general lessons:
*pinning a free DOF per-waypoint is a defect* (S1.36 → S1.46), and *a check that
can never reject is not a check* (S1.37's tangent-plane test: 7,471 evaluations,
zero rejections).

**Non-revertible unless:** the tutorials need to double as a dated record, at
which point they would be duplicating this file and should be deleted instead.

**Verified on:** 2026-09-08. `.gitignore` carries no `tutorials/` rule and
`git check-ignore` matches nothing in the directory; zero amendment banners
remain in Stages 5-7; `Verify:` coverage is 10/10, 8/8, 7/7 sub-stages; every
relative link in the stage READMEs resolves on disk; every `settled.md` S-number
they cite exists here; and every sub-stage number cited from
`geometry_backend.py` and from this file (5.4, 5.6, 5.7, 5.9-5.11, 6.1, 6.2,
6.5, 6.8, 7.4) still resolves to a section.

## S1.71 The IK branch rejection criteria are a `docs/` spec, not an external reference

**Decision:** `examples/curved_surface_printing/IK_BRANCH_REJECTION_GUIDE.md`
moves to **`docs/FR5_IK_Branch_Rejection.md`** and is rewritten as *this
project's* specification: the orientation search, the nine candidate filters in
run order with their real constant names and values, the edge filter and cost
terms, the DAG selection, the failure behaviour, and a closing section listing
every deviation from the external implementation it was adapted from.

The external document's own text is not preserved in the working tree. It is in
git history at the old path, and it was never this project's code -- its preamble
said so ("nothing here exists in `geometry_backend.py` today"). Nothing depended
on its verbatim wording; what mattered was which values were adopted, and that is
recorded in **S1.46**/**S1.47** and now restated in the new doc.

**S1.46 and S1.47 are deliberately left unedited.** Both cite the guide at its old
path, and S1.47 describes it as "external, describing another project's code, with
its file paths and its 35 deg default" -- an accurate description of the file as it
stood when that decision was made. They are append-only decision history; this
entry is the forward pointer.

Deviations the new doc records, so they are not lost if it is ever re-derived:
35 deg -> **30 deg** joint step (aliased to `JOINT_STEP_MAX_DEG` so the exchange
spec's own value cannot drift); the step filter scoped **feed-to-feed only**;
2.0mm -> **1.0mm** surface clearance; J5 0 deg -> **2 deg**; 480 -> **540**
commanded frames (a 20 deg tilt cone added); the tool point **excluded** from
filters 6-8 rather than given a tip-exclusion radius; **multi-proxy 80mm** OBB
bands rather than one box per link; filter 9 pairs **three** apart rather than
two; no per-filter flags; no plain L1/L2 edge terms; no safe-branch mask; and a
vectorised layered relaxation rather than a heap frontier.

**Reason:** the file was the only thing left in `examples/curved_surface_printing/`
that is not tied to the shoulder-sensor study. Everything it describes is
robot/planner-level and applies to the planar path too --
`geometry_backend.py`'s own filter-block comment already says exactly that, which
is why those constants live there and not in `study_config.py` (S1.41). The
folder's README states the rule that decides it: general FR5 data belongs in
`docs/`, study-specific material stays here.

Keeping it as an external reference also had a live cost. Its values are *not*
this project's, and the differences are the kind that silently break an export
(35 deg admits jobs the receiver rejects) or reject every pose (one OBB per link,
pairs two apart). A reader reaching for "the rejection criteria" needs this
project's numbers first, with the reference's as a footnote -- not the reverse.

**Non-revertible unless:** a second consumer of the same criteria appears that
needs them stated project-agnostically, at which point the `docs/` copy would
split into a spec and a project binding.

**Verified on:** 2026-09-08. No reference to the old filename survives outside
this entry and the S1.46/S1.47 history; every constant name and value in the new
doc matches `geometry_backend.py`/`study_config.py`, checked mechanically; every
function it names exists; and it carries no line numbers, so it does not rot
against edits to a 5,000-line module.

## S1.72 BOOT_MATRIX's 7.4 row no longer says curved reachability is unsolved

**Decision:** The Stage 7.4 row's "Do NOT Treat as Current" cell said, as its
emphatic closing claim, that the orientation search did not fix curved
reachability and that "curved still aborts and the S1.45 **placement question is
now the blocker**". Both halves are false and have been since **S1.48**, the same
day. Rewritten to state the true two-cause story: the search bought 8.5x
(8.9%/9.3% -> 76%/70%), the placement fix closed the rest, and both layers now
solve 100% at the real User Frame with `validate_job` ACCEPTED.

**Reason:** BOOT_MATRIX is a routing table read at the *start* of a task, and the
"Do NOT Treat as Current" column exists precisely to stop an agent acting on a
stale belief. A false entry there is worse than a missing one -- this one told a
reader that the curved pipeline aborts and that placement is unresolved, which
would have sent them to re-open a closed question. Unlike `settled.md`, this file
describes present state rather than history, so it is corrected in place.

The useful half of the original warning is kept: do not credit the orientation
search alone with the 100% result. They are two separate causes and only the pair
gets there.

**Non-revertible unless:** the placement or the search changes, in which case the
row is re-measured rather than reverted.

**Verified on:** 2026-09-08. Row 28 read end to end; it no longer asserts that
curved aborts or that placement is open. Noted but **not** corrected in this pass:
row 26 (Stage 7.2) still carries clauses falsified by 7.4/7.5 -- curved paths as
failing row 5 and unexportable, `PRECOMPUTE_CACHE_VERSION` "now 6" (7),
`build_export_segments` returning `[]` after a cache hit, and two inbox notes
marked open that are closed. Row 26 is a stage-scoped snapshot; a full sweep of it
is its own task. **Swept 2026-09-08** -- see the row itself; every clause listed
above is now either corrected or explicitly marked as superseded by 7.4/7.5.

## S1.73 The planar row-5 step figures are re-measured, and are solve-dependent

**Decision:** The planar path's max joint step, measured 2026-09-08 directly from
the shipped `model.precompute.npz`, is **15.49 deg overall** and **4.43 deg within
a feed segment**, with **0** in-segment edges over the 30 deg limit. Live docs now
carry these: `geometry_backend.py` (both the `_relax_candidate_layer` comment and
`dijkstra_candidate_path`'s docstring), `docs/FR5_IK_Branch_Rejection.md`,
`GLOSSARY.md`, `BOOT_MATRIX.md` and `tutorials/Stage7_README.md`.

**S1.44's and S1.47's figures are left exactly as they are.** They are dated
measurements and were correct when taken; this entry is the forward pointer.

**Reason:** three different numbers were in circulation and the disagreement was
not a transcription error -- each was measured against a different solve:

| Measured | Overall | In-segment | Conditions |
|---|---|---|---|
| 7.2 (2026-08-15, S1.44) | 57.32 deg | **5.85 deg** | one commanded orientation per waypoint, pre-search |
| 7.4 (2026-09-03, S1.47) | 57.32 deg *(carried over)* | **4.58 deg** | after the orientation search |
| Now (2026-09-08) | **15.49 deg** | **4.43 deg** | shipped v7 cache, written 2026-09-06 |

Two things follow. First, **S1.47's 57.32 deg was never re-measured** -- it was
carried forward from S1.44 while the in-segment figure beside it was updated. The
orientation search changes which pose is chosen at every waypoint, including
travel waypoints, so the overall step had no reason to survive unchanged, and it
did not: it is 3.7x smaller. That is the specific defect this entry fixes.

Second, the shipped cache disagrees with the 7.4 figures too, though only
slightly (4.43 vs 4.58). It was written 2026-09-06, during the v1.0 review pass
(S1.58-S1.69), and its metadata records the same configuration the 7.4 run used
-- v7, `filter_mode` planar, real User Frame, the same TCP offset, filters and
edge costs, and a matching g-code hash. Something in that pass changed the solved
path without changing the cache key, and the row-5 statistics were not re-taken.
Not chased further: the numbers moved in the safe direction and the conclusion is
unaffected.

**These figures are indicative, not constants.** They describe a particular
solved path, and any change to the filters, the edge costs, the search or the
plate pose moves them. What is stable, and what row 5 actually needs, is the
qualitative result: no feed-to-feed edge comes near 30 deg, so an unscoped E1 is
what breaks the planar job (at its first G0), not a genuine violation.

**Non-revertible unless:** the numbers are re-measured from a newer cache, in
which case this entry gets the same treatment it gives S1.44 and S1.47 -- a new
entry, not an edit.

**Verified on:** 2026-09-08. Computed from `model.precompute.npz` as
`max|diff(joint_path)|` per edge, masked to `waypoint_is_feed[:-1] &
waypoint_is_feed[1:]`: 114,268 feed-to-feed edges of 181,374, max 4.43 deg, none
over 30 deg. Cache meta confirms version 7, planar, User Frame
`[649.456, 133.762, 322.778]`, and `gcode_sha256` matching the `model.gcode` on
disk. L1 and L2 interpretations were also computed (61.41/28.72 overall) and
reproduce none of 57.32, 5.85 or 4.58, ruling out a units or metric mismatch.

## S1.74 Stage-scoped guides get a current-state head, not another banner layer

**Decision:** A post-change audit of the whole doc set found ten stale claims in
six files, all the same failure mode as S1.72's BOOT_MATRIX rows: text written
before 7.4/7.5/S1.48 that those stages falsified. All corrected.

`ctx_system_current.md` (`scope: current-truth`, the agent boot file) carried
five, in its own inline `**(no longer true -- see S1.4x)**` convention where rows
46/47/48/50 already used it and rows 35/43/49 did not. The worst was the S1.40
amendment's closing line -- "the curved pipeline was **deliberately not re-run**
-- placement has to be answered first" -- which is the *last word* of that
section, so a reader left believing placement was open. It has been closed since
S1.48. The file also still said `tutorials/` was gitignored, and
`CurvedModel_AdaptingYourOwnJob.md` said the same; both cited `wiki/INDEX.md` as
the authority for a claim `wiki/INDEX.md` now refutes.

**`CurvedModel_IKPrecompute.md` and `CurvedModel_PrintSetup.md` are restructured
rather than re-bannered:** a **Current behaviour** section at the head, the
superseded material demoted under an explicit **How it got here** heading, and
the "Code anchors" list -- a live reference -- corrected in place. Nothing is
deleted.

**Reason:** the append-a-banner convention is right for `settled.md` and for
`001_Inbox/`, which are dated ledgers. It fails for an *operating guide*, and
`CurvedModel_IKPrecompute.md` is the proof: it had reached a banner whose first
line was "the banner above is itself out of date", and even that third layer did
not reach four body sites -- including a literal instruction to "enable
`allow_tcp_through_plate`", a GUI checkbox deleted at 7.4, and a "Known
limitation" section asserting nothing checks the arm against the mockup, which
**filter 8 closed** (it caught a real pose with an arm link 0.71mm inside the TX
surface). A reader following that guide would have looked for a control that does
not exist and believed a safety gap that no longer exists.

The distinction worth keeping: a *ledger* records what was decided and when, so
it is append-only. A *guide* answers "what do I do now", so its first screen must
be true, and history belongs below the fold. Both are honoured here -- nothing
was deleted, only reordered.

`CurvedModel_Orientation.md` was the only Stage 6 guide with no 7.4 marker at
all, while S1.36 itself carries one; its central claim -- that the frame it
builds is the commanded pose with the roll pinned for stability -- is precisely
what 7.4 replaced. `CurvedModel_PrintSetup.md`'s evidence table was re-read from
the shipped `.npz` metadata rather than hand-edited: three of its five columns
(plate pose, cache version, `allow_tcp_through_plate`) were dead, under a banner
that endorsed them as "current".

**Code was audited and needed nothing.** The session's `geometry_backend.py` diff
is 43 lines, every one a comment or docstring.

**Two follow-ups, same audit (2026-09-08).**

**A render defect, not just a content one.** `BOOT_MATRIX.md` row 27 carried
`` `max |ΔT| = 0.0` `` with **unescaped** pipes. A markdown table row splits on
any unescaped `|` regardless of code spans, so that row rendered as 7 cells
against a 5-cell header -- the Stage 7.1-7.3 row displayed wrong wherever the
wiki is read rendered rather than raw. Escaped to `\|ΔT\|`, matching row 28's
existing `\|J5\|`. Worth stating because a content audit that only greps text
will never see this class of defect; the check is to split each row on unescaped
pipes and assert the cell count matches the header.

**The verification census, and two entries that must stay as they are.** 74
entries against 71 `**Verified on:**` lines. Five deviations, and only one was a
plain omission:

- **S1.31** has two, correctly -- one verification per revision: the first closes
  the original 6.2 decision, the second closes its same-day amendments, and a
  third unlabelled pass covers the second amendment. Not a duplicate.
- **S1.46** has none, ***deliberately***. It contains `### NOT YET MEASURED -- do
  not report these as results` and states "Nothing in this entry has been run";
  S1.47 is the answer to it and carries the date. **Do not add a `Verified on`
  line here** -- a future census will flag it again, and it must be left alone.
- **S1.44** and **S1.45** had the verification and the date present -- S1.44 as
  an `###` heading among its other `###` parts, S1.45 as prose ("Measured
  2026-08-15") -- but no labelled field. Both now carry one, worded only from
  what those entries already stated.
- **S1.21** genuinely has no verification record: no date, no test described in
  60 lines. Given `**Verified on:** not recorded` rather than a guessed date.

The rule this follows: a missing verification is information, and back-filling it
with a plausible date destroys that information. Label what the entry already
proves; say "not recorded" when it proves nothing.

**Non-revertible unless:** a guide's history section grows past the point of
being worth carrying inline, at which point it moves to `001_Inbox/` as a dated
note and the guide keeps a pointer.

**Verified on:** 2026-09-08. All 143 markdown links repo-wide resolve, anchors
and case included; no `INDEX.md` lists a deleted file or omits a present one; no
live doc names `allow_tcp_through_plate`, `_branch_clears_ground`,
`_nozzle_clears_plane`, `precompute_tip_tolerance_mm` or `check_collision` as
current; every remaining `226/2,527`, `186/2,000`, `23/35`, `15/35` and `X+30`
sits inside a history section, an inbox note or a `settled.md` entry;
`PrintSetup.md`'s rebuilt table matches the cache metadata exactly (RX 3,175 /
2,527 feed, TX 2,688 / 2,000 feed, v7, `filter_mode` curved, User Frame
`[649.456, 133.762, 322.778]`); `geometry_backend` imports.
