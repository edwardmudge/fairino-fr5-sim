# Curved-Surface Printing: Study Configuration

Curved-surface printing (loading a toolpath + surface, routing geodesics
over it) is a core, project-agnostic simulator feature — it lives in
`geometry_backend.py`/`gui_panel.py`, same as flat-plate G-code printing.

What's specific to one project is only the asset wiring in
[`study_config.py`](study_config.py): printing an elastomeric capacitive
sensor conformally onto a mockup of a human shoulder. `CURVED_LAYERS`
describes two offset electrode layers (RX/TX) with their PLY toolpath
files, host surface, and display colors; `CURVED_OBSTACLE_FILE` is the
non-print collision body underneath them.

To point the same feature at a different curved-print job, write a new module
shaped like `study_config.py` and select it with an environment variable — **no
source edit is needed**:

```bash
FR5_STUDY_CONFIG=mystudy.study_config python main.py
```

`geometry_backend.py` resolves that name with `importlib`, defaulting to this
module when the variable is unset, and fails at import naming any required
constant the module is missing (the list is `_STUDY_CONFIG_NAMES`). The
loading/geodesic/GUI mechanism itself needs no changes.

**The full guide is
[`wiki/003_Guides/CurvedModel_AdaptingYourOwnJob.md`](../../wiki/003_Guides/CurvedModel_AdaptingYourOwnJob.md)** —
asset formats, the placement reach constraint, the build order, which constants
to re-tune, cache invalidation, and the export format.

> ⚠ Earlier wording here said to "change that one import" in
> `geometry_backend.py`. That was accurate until v1.0; the environment variable
> replaced it (`settled.md` **S1.60**), precisely so adapting the tool no longer
> means editing the simulator core.

The physical context and the reasoning behind this layer configuration is
recorded in `wiki/001_Inbox/2026-07-18_curved_surface_assets.md` and
`wiki/002_Architecture/settled.md` (S1.29–S1.33).

## Supervisor-provided reference

[`external_ik_exchange_spec_EN.md`](external_ik_exchange_spec_EN.md) is
**not this project's own doc** — it is the receiving team's spec for the
`job.json` + `segment_N_solution.json` + `toolpath_T*.ply` package this
project produces as the Collaborator. It lives here rather than in `docs/`
because it describes this study's print job specifically
(`print_job_TX_sensors/`). Treat it as read-only: it defines what an export
must satisfy, and it is the authority behind Stage 7's Rejection Criteria.

Its companion, `docs/saved_coords_data_and_usage_EN.md`, stays in `docs/` —
that one is general FR5 data (DH table, joint limits, FK verification) the
whole simulator depends on, not just this study.

`IK_BRANCH_REJECTION_GUIDE.md` used to sit here too, and left under that same
rule (`settled.md` **S1.71**). What it describes — joint limits, elbow branch,
plate footprint, self-collision, edge costs — is robot- and planner-level and
applies to the planar path as much as this one, so it now lives at
[`docs/FR5_IK_Branch_Rejection.md`](../../docs/FR5_IK_Branch_Rejection.md),
rewritten as this project's own specification rather than the external
implementation it was adapted from.
