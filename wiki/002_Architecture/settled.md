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

**Refined by:** S1.15 -- the plate pose now includes the measured
thickness offset, while `position_mm` keeps meaning "where the plate
rests."

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

## S1.9 Toolpath execution precomputes a continuous IK path before playback

**Decision:** Stage 5.3 toolpath execution uses the existing G-code parser
but treats the parsed waypoint list differently from the preview. Playback
execution includes both `G0` travel moves and `G1` feed moves so the arm
can reposition continuously between printed segments; the preview remains
`G1`-only so it shows the deposited/tool-contact pattern rather than
incidental travel.

Before playback starts, `geometry_backend.py` precomputes the whole IK
path in GUI-visible chunks, so Polyscope can repaint a progress bar
instead of appearing frozen during long solves. The chunked job can also
be paused/resumed without discarding progress. Each parsed plate-local
waypoint is transformed through the current `T_user_frame`, then solved
as a TCP target whose rotation is fixed to `T_user_frame[:3, :3]` for the
full run. That keeps the TCP normal to the build plate while the plate
itself stays static during playback.

Branch selection is continuity-driven and ground-filtered: each waypoint
starts from the valid IK branches ranked closest to the previous accepted
joint solution, then chooses the first branch whose moving physical
geometry (Robot1-Robot6 plus nozzle, transformed with the same Delta
pipeline used for rendering) has no vertex below `z=0`. For speed, branch
clearance is staged: a cheap transformed bounding-box check rejects
obviously below-ground branches before the exact full-mesh vertex check
runs. Static Robot0, the TCP point, and TCP frame axes are not part of the
clearance check. The first unreachable, out-of-limits, or
all-branches-below-ground waypoint aborts the precompute and reports its
index; no partial motion starts.

Playback itself is a simple cached-joint-path stepper for now. The GUI
decides how many cached waypoints to advance per frame; real feedrate
timing remains a later refinement.

**Reason:** This separates three concerns cleanly for the first working
version: parsing/placement stays shared with the preview, reachability and
basic ground clearance are known before motion begins, and playback does
not pay the IK solve cost on every render frame. Including `G0` in
execution avoids impossible jumps between disconnected `G1` drawing
segments, while keeping the preview focused on the visible print path.

**Non-revertible unless:** full-path precompute proves too slow or too
memory-heavy for representative multi-layer prints, at which point the
same continuity rule should be preserved in a chunked or streaming solver
rather than reverting to independent per-waypoint branch choices.

**Refined by:** S1.12 — the precompute hot path (ground clearance via the
bbox lower bound; adaptive keyframe IK + joint interpolation) was made ~6×
faster within this same macro-architecture.

**Verified on:** 2026-07-09

## S1.10 G-code preview draws positive extrusion, not raw G1 motion

**Decision:** The G-code preview is a deposited-material visual. The parser
still preserves every real `G0`/`G1` motion waypoint for Stage 5.3
playback, but preview edges are drawn only when the destination `G1` move
increases extrusion (`E`). `M82`/`M83` and `G92 E...` are tracked only so
that positive extrusion is classified correctly; they do not broaden the
motion scope beyond S1.7's G0/G1 line-segment parser. The bead preview is
offset by half the detected layer height along negative build-plate normal
and uses `GCODE_RADIUS_MM = 0.16` for a 0.28 mm nozzle visual.

**Reason:** Cura can emit non-extruding `G1` moves for travel,
retraction/unretraction, or repositioning. Drawing every `G1` therefore
creates false material across windows/openings. Positive extrusion is the
smallest reliable signal already present in the file for "material is
being deposited", while preserving all motion waypoints keeps robot
playback continuous.

**Non-revertible unless:** the project starts consuming slicer metadata or
a richer path format that explicitly labels printed, travel, bridge, and
support spans more accurately than extrusion deltas.

**Verified on:** 2026-07-09

## S1.11 G-code preview is a swept rectangular bead surface mesh sized by extrusion, revealed progressively during playback

**Decision:** The G-code preview renders deposited material as a **surface
mesh** (`register_surface_mesh("G-code Print", ...)`), not a curve network.
Each positive-extrusion `G1` segment (`is_print_move`, per S1.10) becomes a
swept rectangular box: **width** derived per-segment from the deposited volume
`width = (dE · π(D/2)²) / (L · layer_height)` (clamped by
`GCODE_BEAD_MIN_WIDTH_MM`/`GCODE_BEAD_MAX_WIDTH_MM`, `D = FILAMENT_DIAMETER_MM`),
**height** = layer height, oriented with width in the plate plane and the body
hanging below the toolpath Z so the bead top sits at the nozzle. This
supersedes the *rendering mechanism* of S1.3/S1.10 (curve-network edges,
`GCODE_RADIUS_MM`, both removed); their **parsing and `T_user_frame`
placement** decisions still stand — the mesh is still placed by one
`T_user_frame` multiply and reloaded on plate reposition (S1.8). Detection is
purely extrusion-based with **no `;TYPE:` filtering**, so bridge/overhang spans
(which this Cura export emits as untagged extruding SKIN/WALL moves — it
contains no `;BRIDGE` markers) render as solid bars, not gaps.

`parse_gcode()` now stores the **per-move** extrusion `deposit = e - previous_e`
(not cumulative `E`) at tuple index 2, because retraction/un-retract happen on
non-motion lines that create no waypoint; a cumulative-E difference across the
preceding travel would fold the whole retract distance into the first bead of
each region and inflate it to the max clamp.

Plate-local bead geometry is built once per file (`build_print_beads` via
`_ensure_print_beads`, cached by file+mtime); a plate reposition only re-runs
the world matmul (~0.08 s for the full print), not the parse/build (~1.6 s).
During playback the shape **grows**: `set_print_reveal(waypoint_index)`
re-registers a growing prefix of the cached mesh (`searchsorted` on the
per-bead completion index), throttled by `GCODE_REVEAL_CHUNK` so the
near-complete ~1.0M-vertex mesh is not re-uploaded every frame. `Run` reveals
from the current index, `Reset` empties it, and playback advance grows it.

**Reason:** The curve network was a constant-radius centerline tube with no real
bead width or layer height, so it read as thin wires rather than a printed
object. A swept rectangular bead is how slicers model deposited material and is
the smallest change that makes the preview represent the real 3D printed shape,
including bridges. Deriving width from `E` (rather than a constant) makes
over/under-extrusion and thin bridge spans visible. Progressive reveal was the
user's explicit request ("show the printed shape as it would be printed"); the
prefix-reupload approach is used because Polyscope has no native
show-first-N-faces, and the throttle is the same segment-count/render-cost lever
`Gcode_Toolpath.md` already flags for the ~180k-segment print.

**Non-revertible unless:** the reveal's per-frame re-upload proves too heavy at
full print scale even with `GCODE_REVEAL_CHUNK` (then reveal needs a
transparency/visibility mechanism that doesn't re-send geometry), or the project
adopts a rounded/stadium bead cross-section or slicer-metadata-driven widths.

**Refined by:** S1.15 -- first-layer bead height now uses the actual initial
layer Z so the bead reaches the plate-local top surface.

**Verified on:** 2026-07-09

## S1.12 Toolpath IK precompute is keyframed and joint-interpolated; ground clearance uses the bbox lower bound

**Decision:** Refines S1.9's precompute step (macro-architecture unchanged:
whole path solved before motion, chunked, pausable, then cheap cached
playback). Two changes to the hot path, prompted by a measured ~220 s
full-benchy (179,070-waypoint) precompute:

1. **Ground clearance is decided by the bounding-box check alone in the common
   case.** The transformed bbox min-z is a *guaranteed lower bound* on the exact
   full-mesh min-z (a linear map sends the mesh inside the transformed box, and
   z is linear so its extreme is at a box corner). So `bbox_min_z >= 0` already
   guarantees clearance — accept the branch with no exact check. The exact
   `moving_geometry_min_z` runs only to *adjudicate* a branch whose bbox dips
   below ground (it may still clear). The previous code had this inverted (ran
   the exact check only after bbox already passed, where it is guaranteed to
   pass, and rejected any branch whose bbox dipped without ever consulting the
   exact geometry). The fix is both correct and removes ~80% of precompute cost.

2. **IK is solved at adaptive keyframes and interpolated for the waypoints
   between.** Along a print the selected IK branch is near-constant and segments
   are ~0.65 mm (median), so per-waypoint solving is redundant for a
   visualization. `_compute_keyframe_indices` places a keyframe every
   `GCODE_IK_KEYFRAME_STEP_MM` (2.5) of arc length and at any vertex turning
   more than `GCODE_IK_KEYFRAME_ANGLE_DEG` (15°), so corners and long `G0`
   travels keep their exact pose while gentle curves and straights subsample.
   Keyframes are solved (with the continuity + ground rule above); the dense
   per-waypoint `toolpath_joint_path` is filled at finish by interpolating each
   joint against cumulative arc length. Playback and reveal are unchanged —
   they still index the dense path by waypoint.

Result on the benchy: ~220 s → ~37 s (5.9×), with faithful tracking — nozzle
TCP error at interpolated waypoints is ~0.05 mm median (worst ~1.3 mm only at
sub-millimetre-radius fillets), and the printed bead mesh is exact because
reveal stays per-segment.

**Accepted simplifications:** intermediate (non-keyframe) waypoints are not
individually reachability- or ground-checked, and joint-space interpolation
traces a slightly curved Cartesian path between keyframes rather than the exact
segment. Both are invisible at 2.5 mm spacing and appropriate for a
visualization.

**Non-revertible unless:** driving a real arm (not a sim) makes the
between-keyframe path deviation or the un-checked intermediate poses
unacceptable, at which point tighten `GCODE_IK_KEYFRAME_STEP_MM`/`ANGLE_DEG`
toward per-waypoint solving rather than abandoning keyframing; or a future
non-plate (curved, Stage 6) surface needs a varying TCP orientation, which
changes what is being interpolated.

**Verified on:** 2026-07-10

## S1.13 Toolpath IK precompute is cached to disk and auto-loaded when identical

**Decision:** A completed precompute (the dense per-waypoint joint path, S1.12)
is saved to `assets/models/gcode/model.precompute.npz` — one cache beside the
one fixed `model.gcode` (S1.7) — and reloaded instead of re-solving whenever an
*identical* precompute is requested, both at startup (`VisContent.__init__`) and
when the "Precompute Toolpath IK" button is pressed. The joint path is stored as
`float32` (halves the ~8.6 MB; 0.001° resolution is ample) alongside a JSON key.

A cached path is used only when its stored key equals the key rebuilt from the
current inputs. The key is everything the path depends on: the **SHA-256 of the
`model.gcode` bytes** (content hash, so an identical re-export still hits and a
different object misses), the **build-plate pose** `T_user_frame` (the path is
only valid for the pose it was solved at — moving the plate correctly misses),
the keyframe params (`GCODE_IK_KEYFRAME_STEP_MM`/`ANGLE_DEG`) and
`GROUND_Z_MIN_MM`, and `PRECOMPUTE_CACHE_VERSION` (bumped by hand when the
IK/ground-check/joint-limit/robot-geometry code changes, since those are not in
the runtime key). Save and load are best-effort: any write error, missing file,
or corruption falls back to a normal solve.

The runtime seed (`current_joint_angles`) is deliberately **excluded** from the
key: it only nudges the waypoint-0 branch, both branches are valid
ground-clearing paths, and playback snaps to `joint_path[0]` regardless.
Including it would make the cache brittle (a startup at the default pose could
never match a path solved from a HOME pose) for no correctness gain.

**Reason:** The keyframed solve is ~37 s for a full multi-layer print; for an
unchanged object that is pure repeated work. Content-hash + pose + params keying
guarantees a loaded path is actually valid for the current scene, so the reuse
is safe rather than a stale shortcut.

**Consequences (correct by design, not bugs):** moving the plate invalidates the
cache; startup auto-load only fires at the default plate pose, because S1.6
forbids auto-restoring a saved position at startup — the moved-plate case is
served by the Precompute button, which loads from cache on a hit too.

**Non-revertible unless:** the fixed single-`model.gcode` convention (S1.7) is
replaced by a file picker / multiple concurrent objects, at which point the
single fixed cache filename should become per-object (e.g. keyed by the gcode
hash) rather than reverting to no cache.

**Verified on:** 2026-07-10

## S1.14 Build-plate color is explicit and distinct from the print

**Decision:** `load_build_plate()` sets the registered "Build Plate" surface
mesh to `BUILD_PLATE_COLOR = (0.75, 0.77, 0.80)`. The deposited G-code bead
mesh keeps `GCODE_COLOR = (1.0, 0.55, 0.0)`.

**Reason:** Leaving the build plate on Polyscope's automatic color assignment
made it depend on registration order. In the real app's current init sequence
that produced approximately `(0.89, 0.61, 0.11)`, visually close to the orange
print color; a captured screenshot showed the plate and print blending into the
same gold/olive hue. An explicit cool gray keeps the bed readable against the
warm deposited material.

**Non-revertible unless:** the app grows a deliberate material/theme system
that assigns both plate and print colors explicitly from one palette.

**Verified on:** 2026-07-10

## S1.15 The build plate accounts for physical thickness, and the first layer reaches its top surface

**Decision:** The Bambu Lab build-plate mesh is treated as a 0.75mm-thick
object whose local origin is on the top/print surface. `load_build_plate()`
keeps `position_mm` as the place where the plate rests, but stores
`T_user_frame[:3, 3] = position_mm + R @ [0, 0, BUILD_PLATE_THICKNESS_MM]`,
so the mesh's local bottom (`Z=-0.75`) lands on the shared floor and local
top (`Z=0`) becomes the corrected print surface. The User Frame triad uses
that corrected `T_user_frame` origin.

`build_print_beads()` also treats the first layer specially: beads whose
toolpath top Z matches the first printed layer hang down by that first-layer
Z, not by the general Cura layer height. Later layers still hang down by the
general layer height, so their stacking behavior is unchanged.

**Reason:** Direct `trimesh` measurement of `BambuLab_BuildPlate.obj` showed
local bounds `Z in [-0.75, 0]`: the mesh's material extends downward from its
local top surface. Placing local `Z=0` at world `position_mm.z` therefore sunk
the plate body below the shared floor. The same measurement pass showed the
first real print moves in `model.gcode` are at `Z=0.3`, while the general layer
height header is `0.1`; using only the general height made the first bead span
`Z=[0.2, 0.3]` in plate-local coordinates, leaving a visible gap above the
plate-local `Z=0` surface.

**Known simplification:** `GROUND_Z_MIN_MM` remains the shared-floor clearance
check at world `Z=0`. It does not yet model arm-vs-plate-body collision against
the plate footprint and 0.75mm thickness; that is a separate future collision
problem.

**Cache note:** This intentionally changes `T_user_frame`. Any S1.13
precompute cache made before this decision misses automatically because the
cache key includes the full user-frame matrix; no special invalidation code is
needed.

**Non-revertible unless:** the build-plate mesh is replaced by one whose local
origin and measured thickness are different, or slicer metadata becomes rich
enough to provide per-layer bead heights more directly.

**Verified on:** 2026-07-10
