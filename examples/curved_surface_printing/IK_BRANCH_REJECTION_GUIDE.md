# IK Branch Rejection Guide for Toolpath Generation

> **External reference — not this project's code.** This describes a *separate,
> working* implementation of the same curved-printing task, supplied as the
> model for roadmap **7.4**. The file paths and defaults below
> (`surface_traj_batch.py`, `robot_collision.py`, …) belong to that project;
> nothing here exists in `geometry_backend.py` today. What this project adopts,
> and with which values, is `settled.md` **S1.46** and `Stage7_README.md` §7.4 —
> note in particular that its 35° joint-step limit is **not** carried across,
> because the exchange spec rejects steps >30°.
>
> Covers every rejection criterion, the order they run in, and why each exists.

## Overview

The toolpath planner picks one IK solution per point along a surface trajectory.
Each point has up to **480 candidates** (60 roll angles × 8 IK solutions).
Rejection happens in two phases:

1. **Candidate filter** (per-candidate, before graph construction) — hard reject.
2. **Edge filter** (between adjacent candidates, inside graph search) — hard reject + soft penalty.

Only candidates that survive Phase 1 enter the graph.
Only edges that survive Phase 2 are traversed by the optimizer.

---

## Phase 1 — Candidate-Level Filter (`build_admissible_mask`)

**File:** `FR5_FK_Kit/surface_traj_batch.py`, line 710.

Candidates are tested in the order below. The order is deliberate: cheap
arithmetic checks run first; expensive FK + collision checks run last. A
candidate is rejected on the **first** test it fails (`continue`), so later
checks never execute for it.

### Filter 1: Joint Limits

| Detail | Value |
|--------|-------|
| Check | Every joint angle must be within `JOINT_LIMITS` (from `ik_solver`) |
| Tolerance | ±1e-9° |
| Cost | Pure arithmetic, no FK |
| Why | Out-of-range joints are physically impossible for the FR5 |

### Filter 2: J5 Non-Negative

| Detail | Value |
|--------|-------|
| Check | `q[4] >= j5_min_deg` (default 0°) |
| Flag | `require_j5_nonnegative` (default True) |
| Cost | Pure arithmetic |
| Why | Negative J5 flips the wrist/nozzle orientation. For surface printing with nozzle pointing down, this produces an upside-down tool approach |

### Filter 3: J4 Minimum Angle

| Detail | Value |
|--------|-------|
| Check | `q[3] >= j4_min_deg` (default -60°) |
| Flag | `require_j4_ge_minus_60` (default False — opt-in) |
| Cost | Pure arithmetic |
| Why | Restricts elbow range to avoid extreme wrist poses |

### Filter 4: Upper Branch Configuration

| Detail | Value |
|--------|-------|
| Check | Elbow must be above the shoulder-wrist chord by at least `upper_branch_tol_mm` (default 2 mm) |
| Flag | `use_upper_branch` (default True) |
| Cost | Requires FK (first FK call for this candidate) |
| Why | Rejects lower-elbow and near-straight-arm configurations. These are kinematically dangerous — close to singularity, unpredictable velocity, and risk sudden flips |

### Filter 5: Elbow Above Build Plate Plane

| Detail | Value |
|--------|-------|
| Check | Signed distance from physical elbow to the build plate top surface plane must be >= `-z_tol_mm` (default 1 mm) |
| Flag | `keep_above_world_xy` (default True) |
| Cost | Reuses FK from Filter 4 |
| Why | Prevents the robot's elbow from dipping below the plate. Elbow-below-plate configs lead to collisions during travel moves |

### Filter 6: Under Build Plate Footprint

| Detail | Value |
|--------|-------|
| Check | No robot/nozzle sample point may fall inside the XY shadow of the build plate AND below its top surface |
| Flag | `keep_out_under_build_plate` (default True) |
| Margin | `under_plate_footprint_margin_mm` (default 20 mm) expands the XY bounds |
| Cost | FK + sampled link points transformed to plate frame |
| Parts checked | Nozzle, Robot2–Robot6 |
| Why | The robot reaching *under* the plate would collide with the table/fixture. XY expansion catches near-misses |

### Filter 7: Build Plate Volume Collision

| Detail | Value |
|--------|-------|
| Check | No link sample point may penetrate the plate's conservative bounding slab |
| Flag | `check_build_plate_collision` (default True) |
| Clearance | `collision_clearance_mm` (default 3 mm) |
| Cost | FK + voxel-downsampled link points vs. plate AABB slab |
| Why | Catches link-through-plate collisions that the footprint test misses (e.g., wrist passing through plate edge) |

### Filter 8: Surface Mesh Collision

| Detail | Value |
|--------|-------|
| Check | No link sample point may be within `surface_collision_clearance_mm` (default 2 mm) of any triangle on the workpiece surface |
| Flag | `check_surface_tx_base_collision` (default True) |
| Cost | FK + spatial-grid broadphase + point-to-triangle narrowphase |
| Why | The robot/nozzle must not collide with the surface being printed on. Uses a uniform grid over all surface triangles for fast queries |

### Filter 9: Robot/Tool Self-Collision

| Detail | Value |
|--------|-------|
| Check | OBB-based link-vs-link proximity check (e.g., Robot3 vs Robot5) |
| Flag | `check_robot_tool_self_collision` (default True) |
| Clearance | `self_collision_clearance_mm` (default 5 mm) |
| Cost | FK + multi-proxy OBB distance |
| Why | Prevents the robot from hitting itself — especially at extreme joint configurations where the forearm can approach the upper arm |

### What happens when all candidates for a point are rejected

If every candidate at a point fails, the function records a
`NoAdmissibleCandidateError` with the per-reason breakdown for that point.
The entire TX is marked failed. The planner does **not** relax filters or
fall back to a less-safe candidate.

---

## Phase 2 — Edge-Level Rejection (Inside Graph Search)

After Phase 1, surviving candidates form a layered DAG:
- **Nodes:** `(point_index, candidate_index)` for each admissible candidate.
- **Edges:** every pair `(point i, candidate a) → (point i+1, candidate b)`.

The graph is searched by Dijkstra (A* with h=0) in `surface_astar_planner.py`.

### Hard Reject: Max Adjacent Joint Step (`forbidden_edges`)

**File:** `surface_traj_batch.py`, line 815.

| Detail | Value |
|--------|-------|
| Check | `max(|q_a[j] - q_b[j]|) for j in 0..5` must be ≤ `max_adjacent_joint_step_deg` (default 35°) |
| Effect | Edge cost is set to `inf` inside the A* planner, so the edge is never traversed |
| Why | A single joint jumping 35°+ between two adjacent toolpath points means dangerously fast motion. Even if the total weighted cost is low, one joint spiking is physically unsafe |

### Soft Reject: Branch Change Penalty

**File:** `surface_trajectory_costs.py`, line 64.

The graph edge cost is the sum of all enabled cost functions. One of them
penalizes switching IK solution families:

```
penalty = 150 * (ik_ordinal changed)
        + 2 * max(0, circular_roll_delta - 1)²
```

Where:
- `candidate_index = roll_index * 8 + ik_ordinal`
- `circular_roll_delta = min(|roll_a - roll_b|, 60 - |roll_a - roll_b|)`

| Component | Value | Effect |
|-----------|-------|--------|
| IK ordinal change | 150 | Massive penalty for switching IK family (e.g., elbow-up ↔ elbow-down). Makes the optimizer strongly prefer staying on the same IK branch |
| Roll step ≤ 1 | 0 | Adjacent roll slots are free (smooth rotation) |
| Roll step > 1 | 2 × (excess)² | Quadratic penalty for large roll jumps. A jump of 5 slots costs 2×16=32; a jump of 10 costs 2×81=162 |

### Other Edge Costs (Not Rejection, But Steering)

These don't reject edges but steer the optimizer toward smooth trajectories:

- **Joint Angle Movement (L1):** `sum(|q_curr - q_prev|)` — total joint travel in degrees.
- **Joint L2 Movement:** `||q_curr - q_prev||₂` — Euclidean joint-space distance.
- **Weighted Joint Movement:** `sum(w × |q_curr - q_prev|)` with weights `[3, 3, 2, 1, 1, 0.5]` — penalizes proximal (base/shoulder) motion more heavily, pushes redundancy resolution to the wrist.

---

## Post-Search: Safe Branch Mask

**File:** `surface_traj_batch.py`, line 863.

After the optimal path is found, the planner builds a **safe branch mask** — 
the set of alternative roll candidates the robot could switch to without
exceeding `safe_branch_max_step_deg` (default 35°) per joint. This is not a
rejection step; it's metadata for downstream consumers (e.g., the gradient
refiner) that tells them which roll angles are reachable from the selected
solution without a dangerous jump.

---

## Default Tolerances Summary

| Parameter | Default | Source |
|-----------|---------|--------|
| `upper_branch_tol_mm` | 2.0 | `robot_collision.py` |
| `world_xy_z_tol_mm` | 1.0 | `robot_collision.py` |
| `under_plate_footprint_margin_mm` | 20.0 | `robot_collision.py` |
| `collision_clearance_mm` | 3.0 | `robot_collision.py` |
| `link_sample_spacing_mm` | 25.0 | `robot_collision.py` |
| `nozzle_tip_exclusion_mm` | 12.0 | `robot_collision.py` |
| `surface_collision_clearance_mm` | 2.0 | `robot_collision.py` |
| `surface_grid_cell_size_mm` | 8.0 | `robot_collision.py` |
| `self_collision_clearance_mm` | 5.0 | `robot_collision.py` |
| `max_adjacent_joint_step_deg` | 35.0 | `surface_traj_batch.py` |
| `j5_min_deg` | 0.0 | `surface_traj_batch.py` |
| `j4_min_deg` | -60.0 | `surface_traj_batch.py` |
| `BRANCH_CHANGE_IK_ORDINAL_PENALTY` | 150.0 | `surface_trajectory_costs.py` |
| `BRANCH_CHANGE_ROLL_QUADRATIC_WEIGHT` | 2.0 | `surface_trajectory_costs.py` |
| `WEIGHTED_JOINT_MOVEMENT_WEIGHTS` | [3, 3, 2, 1, 1, 0.5] | `surface_trajectory_costs.py` |

---

## Key Files

| File | Role |
|------|------|
| `surface_traj_batch.py` | Orchestrates filtering + planning per TX |
| `surface_astar_planner.py` | Dijkstra/A* graph search over the candidate DAG |
| `surface_trajectory_costs.py` | All cost functions (L1, L2, weighted, branch penalty) |
| `surface_trajectory_planner.py` | Unified entry point: builds layers, dispatches DP or A* |
| `robot_collision.py` | Collision primitives (plate volume, surface mesh, self-collision, upper branch, footprint) |
| `toolpath_branch_select.py` | G-code pipeline variant (Viterbi per chain + build plate collision) |
| `toolpath_ik_batch.py` | Upstream: generates the raw IK candidates that get filtered here |
