---
status: inbox
stage: post-7.3
scope: assets/buildPlate/saved_position.json, geometry_backend.py (_plate_plane / load_curved_model placement)
---

# Neither toolpath runs at the real User Frame — and for two different reasons

> ⚠ **The measurements stand; the diagnosis is superseded by `settled.md`
> S1.46 (roadmap 7.4).** The supervisor has confirmed the **7.3 configuration is
> correct**, so "Result 2 — curved is genuinely out of reach" and "the cause is
> **placement, not calibration**" no longer hold as written. Both results measure
> a **single commanded orientation per waypoint** (S1.36 pins tool Z to the exact
> normal and fixes the roll, giving at most 8 IK candidates before a point
> reports `"Unreachable"`); roadmap 7.4 searches ~480 per waypoint. Result 1 is
> already identified below as a *modelling* limitation, and 7.4 replaces that
> infinite plane with a finite footprint + slab.
>
> Open question 2 ("should the plate plane become finite?") is therefore
> **answered: yes**. Open question 3 (where the curved model should sit) is no
> longer blocking — it becomes a hypothesis 7.4 can test. Open question 4 is
> answered by subsumption: the S1.36 fix is folded into the orientation search,
> so its option 1 should **not** be implemented separately.

## What happened

Roadmap 7.3 replaced `assets/buildPlate/saved_position.json` with the real
calibrated User Frame (`settled.md` **S1.45**):

| | position (mm) | rpy (deg) |
|---|---|---|
| was — Stage 6.8 demo pose | `[-570, -300, -200]` | `[0, 0, 0]` |
| now — real, `user_index=1` | `[649.456, 133.762, 322.778]` | `[-0.369, 0.329, -89.080]` |

§7.3 flagged reachability at that pose as a **known, unverified risk** and said
that if the arm cannot reach, it is a finding to record rather than a reason to
restore the demo pose. It cannot reach. Both toolpath sources fail, and neither
failure is the one the risk note anticipated.

Measured 2026-08-15, headless `fairino-fr5-sim`, `PHYSICAL_JOINT_LIMITS`,
`allow_tcp_through_plate = False` (the default).

## Result 1 — planar aborts at waypoint 0, on the *plate check*, not on reach

Running the real precompute (`run_/step_toolpath_ik_precompute`, which since 7.2
still carries `check_collision=True` on this path):

```
Waypoint 0/181375: all 8 valid branch(es) hit the build plate (arm + nozzle)
```

**IK is not the problem.** Waypoint 0 solves with **8 valid branches**, and a
strided IK-only sweep found **0 / 13,952** waypoints unreachable — pure reach and
joint limits are fine across the whole bed. Every one of those branches is then
thrown out by `_branch_clears_ground`.

The reason is structural:

| | plate top plane | best branch's deepest arm-link signed distance | branches clearing |
|---|---|---|---|
| Stage 7.1 default `[-570,-300,-100]` | z = **-99.2mm** | **+125.4mm** (clear) | 8 / 8 |
| real User Frame | z = **+323.5mm** | **-253.2mm** (inside) | **0 / 8** |

S1.40 models the plate as the **infinite plane** through its top face. That is
sound only while the plate sits below the whole arm. The real User Frame is
**323.5mm above the robot base origin**, so the plane cuts straight through the
shoulder and elbow — links that are nowhere near the print and cannot be moved
out of the way. `allow_tcp_through_plate` does not help: it gates the tool point
only, and arm links 0-5 are blocked unconditionally.

**So this is a modelling limitation, not a robot limitation.** S1.40's own
docstring prescribes the fix — *"if the arm reaches below the plate the fix is to
move the plate lower"* — and that prescription is unavailable here, because the
plate's height is now a **measurement**, not a tuning knob. A real bed is finite;
the arm in the real cell reaches over and around it.

## Result 2 — curved is genuinely out of reach

The curved path has no collision check at all since 7.2, so it never meets the
problem above. It fails on plain geometry instead. Every feed point of both
layers, solved with per-point surface-normal orientation frames:

| Layer | Feed points | Solved, **real** frame | Solved, 6.8 demo pose | TCP distance from base (real frame) |
|---|---|---|---|---|
| RX | 2,527 | **226** — 91.1% unreachable | 2,527 (100%) | median **912mm**, max **945mm** |
| TX | 2,000 | **186** — 90.7% unreachable | 2,000 (100%) | median **916mm**, max **947mm** |

Every failure returns `"Unreachable: no geometric solution for this pose"` — the
`a2 + a3 = 820mm` shoulder-to-wrist chain simply does not extend that far. At the
demo pose the same points sit at a median of **475mm**.

The cause is **placement, not calibration**. `load_curved_model()` centres the
assembly on the plate (`T_placement`, S1.29). At the real frame the plate's
centre lands ~844mm from the base and its far corner ~980mm, so centring the
model puts it in the worst available spot. The plate corner itself is only 738mm
out — which is why the *planar* G-code, whose model sits near the plate's local
origin, has no reach trouble at all.

## What this does and does not mean

- **The real User Frame is not wrong.** It is a measurement from the physical
  robot and it stays. Nothing here argues for restoring the demo pose.
- **The 6.8 demo pose was doing more work than its name suggests.** It was not
  just "somewhere the model is visible" — it was simultaneously satisfying the
  infinite-plane plate model *and* centring the workpiece inside the envelope.
  Two constraints, one hand-tuned number, and replacing it exposed both.
- **`USER_FRAME_ORIGIN_MM` is untouched and still works.** Startup/Reset remains
  `[-570, -300, -100]`, where planar still solves 181,375/181,375. The real frame
  is opt-in per session via "Load Saved Position", so nothing regressed for
  anyone not clicking that button.
- **§7.4 is not blocked by this.** The exporter is written against the *planar*
  path (Stage7_README's note), which works at the default pose. Exporting a job
  solved at a non-real User Frame is a data-provenance problem to flag in the
  export header, not a reason to stall the writer.

## Open questions, in the order they need answering

1. **Is the plate mesh even the right object at the real frame?** The User Frame
   is the *workpiece* origin. At the demo pose the Bambu Lab plate was a stand-in
   for it. If the real cell's fixture is not a 258×276mm bed 323mm up, the plate
   mesh is now scenery attached to a frame it does not describe — and the
   infinite-plane collision model inherits that error. **Ask the supervisor what
   physically sits at the User Frame** before engineering around Result 1.
2. **Should the plate plane become finite (or optional)?** The cheapest honest
   fix for Result 1 is to bound the plane to the plate's actual footprint, so
   links far from the bed stop being rejected. That is a real design change to
   S1.40, not a constant tweak, and it re-opens the mesh-vs-mesh question
   `CurvedModel_PrintSetup.md` already parks. A cheaper interim: a "check against
   the plate" master toggle, defaulting on, off when the plate is above the base.
3. **Where should the curved model actually sit?** Centring on the plate is a
   Stage 6.1 convenience (S1.29), not a requirement. The real job presumably
   places the shoulder mockup at a known offset in the User Frame. Until that
   offset is known, no amount of re-running the curved pipeline produces a
   meaningful result — which is why 7.3 deliberately did **not** re-run it.
4. **Does the S1.36 reference-axis fix still get folded into a curved re-run?**
   [`2026-08-15_orientation_frame_flips_row5.md`](2026-08-15_orientation_frame_flips_row5.md)
   sequenced itself with 7.3's forced curved rebuild. That rebuild has not
   happened and should not until (3) is answered, so the two are still correctly
   paired — just later than planned.

## Reproducing

Headless, `C:\Users\Edward\miniconda3\envs\fairino-fr5-sim\python.exe`, from the
repo root, with the plate at the real frame
(`content.load_build_plate([649.456, 133.762, 322.778], [-0.369, 0.329, -89.080])`):

- **Planar:** `run_toolpath_ik_precompute(PHYSICAL_JOINT_LIMITS)` then loop
  `step_toolpath_ik_precompute()`. Redirect `precompute_cache_path` to a scratch
  file first — a completed run writes the cache. Aborts immediately.
- **Plate plane:** `_plate_plane()`, then `_meshes_clear_plane(angles, range(6),
  point, normal, 0.0)` per branch of `solve_ik_tcp_matrix` at waypoint 0.
- **Curved:** `load_curved_model()`, then per layer `np.vstack(curved_pieces_world[l])`
  and `_orientation_frames_for_points(l, points)` — no geodesics or print order
  needed, since only reach is being measured.

See `settled.md` **S1.45**,
[`2026-07-22_stage7_calibration_and_external_ik.md`](2026-07-22_stage7_calibration_and_external_ik.md) §7.3,
and [`../003_Guides/CurvedModel_PrintSetup.md`](../003_Guides/CurvedModel_PrintSetup.md)
("Changed in Stage 7.3").
