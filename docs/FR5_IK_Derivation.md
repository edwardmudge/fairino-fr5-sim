# FR5 Closed-Form Inverse Kinematics

Reference derivation for `VisContent.solve_ik`, `solve_ik_tcp`, and
`solve_ik_tcp_matrix` in `geometry_backend.py`. Written in the style of
Craig's *Introduction to Robotics* §4.7 PUMA560 worked example, but
**not** a verbatim port of it -- see the structural note below.

## Why Craig's PUMA560 equations don't transfer directly

Craig's PUMA560 has a true *common-point* spherical wrist: `d5 = d6 = 0`
in his table, so frames 4, 5, and 6 all share one origin, and "the wrist
center" is simply that point, reached by backing `d6` off the target
position.

The FR5's own DH table (`docs/FR5_DH_Table.md`) has **both** `d4 = 102mm`
and `d5 = 102mm` nonzero (only `a4 = a5 = a6 = 0`). Structurally this is
the same family as a Universal Robots UR5/UR10
(`alpha1=+90°, alpha4=+90°, alpha5=-90°`), not a PUMA wrist. Working
through the DH chain shows axis4(=z3)∩axis5(=z4) = frame-4's origin, and
axis5(=z4)∩axis6(=z5) = frame-5's origin -- two *different* points, since
`d5 != 0`. Pieper's more general solvability theorem still applies
(`a4=0` alone guarantees frame-4's origin depends only on θ1,θ2,θ3), but
the position/orientation decoupling has to be re-derived around
frame-5's origin, not frame-4's.

## Derivation

Constants from `DH_PARAMS`: `a2=-425, a3=-395, d1=152, d4=102, d5=102, d6=100`.

Given target flange pose `T_0_6 = [R | p]`, `â = R[:, 2]` (approach
vector), back off the last link to get the frame-5 origin:

```
P5 = p − d6 · â
```

**1) θ1 -- shoulder (2 branches, sign1 = ±1):**
```
θ1 = atan2(P5y, P5x) + atan2(d4, sign1 · sqrt(P5x² + P5y² − d4²))
```
`d4` plays the role Craig's `d3` plays for PUMA: it's the fixed
perpendicular offset of `P5` (not `P4`) off the shoulder plane. This is
the one place FR5's geometry genuinely differs from a direct port --
the offset shows up one joint later in the chain because `d5 != 0`.

**2) θ5 -- wrist bend (2 branches, sign2 = ±1)**, using the raw target
`px, py` (not `P5`):
```
θ5 = sign2 · acos((px·sinθ1 − py·cosθ1 − d4) / d6)
```

**3) θ6** -- with `R_0_1 = dh_transform(0, π/2, d1, θ1)[:3,:3]`,
`R_1_6 = R_0_1ᵀ · R`:
```
θ6 = atan2(−R_1_6[2,1] / sinθ5, R_1_6[2,0] / sinθ5)
```
Undefined at the wrist singularity (θ5 ≈ 0, axes 4/6 aligned, θ4/θ6
split is ambiguous); `solve_ik` falls back to `θ6 = 0` and flags the
branch `is_singular=True`.

**4) ψ = θ2+θ3+θ4 (auxiliary):**
```
K = Rot_y(−θ5) · Rot_z(θ6)
Rot_z(ψ) = R_1_6 · Kᵀ
ψ = atan2(Rot_z(ψ)[1,0], Rot_z(ψ)[0,0])
```

**5) θ2, θ3 -- elbow (2 branches, sign3 = ±1)**, planar 2-link IK for the
point backed off by `d5` along the ψ direction:
```
X' = P5x·cosθ1 + P5y·sinθ1,   Y' = P5z − d1
X = X' − d5·sinψ,             Y = Y' + d5·cosψ
cosθ3 = (X² + Y² − a2² − a3²) / (2·a2·a3)
θ3 = sign3 · acos(cosθ3)
θ2 = atan2(Y, X) − atan2(a3·sinθ3, a2 + a3·cosθ3)
```

**6) θ4 = ψ − θ2 − θ3**

Up to `2×2×2 = 8` branches total. Branches that fail an acos/sqrt domain
check (target unreachable along that branch) are skipped -- Craig's "up
to 8 solutions" framing, not always exactly 8.

## Flange -> TCP offset

`self.tcp_local` is a single zero-pose *world* point (no orientation of
its own) -- see `FR5_Mesh_Convention.md`. `solve_ik_tcp` targets the TCP,
so it needs a genuine flange-local *pose* offset, not just a point.
`T_flange_to_tcp` is built in `load_data()`:

```python
T_zero_flange_inv = np.linalg.inv(self.T_zero[5])
T_flange_to_tcp = T_zero_flange_inv.copy()
T_flange_to_tcp[:3, 3] = (T_zero_flange_inv @ [tcp_local, 1])[:3]
```

The rotation part is exactly `inv(T_zero[5])`'s rotation -- this matches
the rendered "TCP Frame" triad exactly, since its axis-tip points are
defined in world coordinates at zero pose and driven by the same
`Delta_6` as everything else (see `docs/FR5_Mesh_Convention.md`); the
zero-pose orientation of those world-aligned axis tips relative to the
flange is `inv(T_zero[5])`'s rotation by construction. Numerically this
comes out to `Rot_x(-90°)` for the FR5's actual mesh/TCP data -- a real
rotation, not pure translation.

`solve_ik_tcp` converts the target once: `T_target_flange = T_target_tcp
@ inv(T_flange_to_tcp)`, then runs `solve_ik` unchanged.

## Branch selection

`solve_ik_tcp_matrix` is the shared entry point -- `solve_ik_tcp` is a
thin RPY-to-matrix wrapper over it. It discards branches with any joint
outside the caller-supplied `joint_limits`, then **ranks** (not picks)
the rest by summed wrapped-angle distance to a reference pose --
`self.current_joint_angles` by default, or an explicit
`reference_joint_angles` -- and returns the whole ranked list, standard
practice per Craig's text (prefer the solution closest to the reference
configuration). Choosing *which* branch to apply is left to the caller:
`gui_panel.py`'s "Inverse Kinematics" panel defaults to index 0 but lets
the user pick any other valid branch from a list. The chunked toolpath
precompute (`step_toolpath_ik_precompute`) instead passes the *previous*
waypoint's solved pose as `reference_joint_angles` and takes the first
ranked branch that clears the plate, giving continuous motion
waypoint-to-waypoint instead of each one independently chasing the live
arm's pose.

Each returned entry carries `raw_branch_index` -- `solve_ik`'s own
enumeration position -- purely as a stable ordinal for disambiguating
branches in a UI label. Branches are labeled with that ordinal plus the
three sign-driven joint values (J1, J3, J5 -- the joints whose sign
choices `solve_ik` branches over) rather than an anatomical name
("shoulder left/right", "elbow up/down"), since no such naming has been
geometrically verified against this arm's actual poses.

## Verification

Verified against `compute_fk`: 1000/1000 random in-limit joint configs
round-trip through `compute_fk -> solve_ik -> compute_fk` within 1e-6;
12/12 joint-limit boundary cases pass; 100/100 near-singularity (θ5
within 2° of 0) cases pass at a relaxed 1e-4 tolerance; TCP-targeting
round-trip 100/100.
