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
