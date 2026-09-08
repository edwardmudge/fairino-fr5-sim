# Project Principles

## 1. Think Before Coding
Read `wiki/005_AgentMgmt/active/ctx_main/BOOT_MATRIX.md` and load only the
documents relevant to the task at hand before writing code.

## 2. Simplicity First
This is a from-scratch learning project — prefer the direct implementation
over a general one. Don't add abstractions for stages that haven't been
reached yet (e.g. don't generalise the FK code for arbitrary DOF arms; this
is a 6-axis FR5, hard-code that).

## 3. Surgical Changes
`geometry_backend.py` and `gui_panel.py` are the two files that carry the
simulator itself; `main.py` is wiring and shouldn't need edits.

The one standing exception: **job-specific constants belong in a study config**
(`examples/<study>/study_config.py`), not in `geometry_backend.py` — settled.md
S1.33/S1.41. Material and nozzle values (bead size, hover height, tip clearance)
and asset wiring go there; robot- and planner-level values (joint limits, filter
thresholds, edge costs) stay in `geometry_backend.py`.

## 4. Goal-Driven Execution
The original 4-stage roadmap (FK maths → mesh rendering → tool head/TCP → IK) is
**complete**, as are Stage 5 (planar G-code printing), Stage 6 (curved-surface
printing) and Stage 7 (calibration + external IK job export). New work extends
that base rather than following the stage order.

The dependency the old rule protected still holds and is worth stating directly:
the Delta transform depends on a correct `compute_fk()`, and every curved-path
feature depends on both. Check `wiki/005_AgentMgmt/active/ctx_main/ctx_system_current.md`
for what is actually built before assuming a stage is still ahead of you.

## Mesh Rendering Hard Rule

**Never apply `T_0_i(q)` directly to mesh vertices. Always apply the Delta
transform: `Delta_i = T_0_i(q) @ inv(T_0_i(0))`.**

Why: the OBJ mesh vertices (`assets/fr5_meshes/Robot0-6.obj`) are exported
from CAD already in the zero-pose world frame — they already encode
`T_0_i(0)`. Applying `T_0_i(q)` on top double-transforms them and produces
garbage geometry. Full explanation and code: `docs/FR5_Mesh_Convention.md`.
This applies to `geometry_backend.py` and any future script that renders
the arm meshes.

## SDK / API Investigation Rule

Polyscope's ImGui widgets return `(changed, value)` tuples, not bare
values — check `docs/Polyscope_Quickstart.md` before using an unfamiliar
`psim.*` widget rather than guessing its return signature.

## Documentation Updates

When an architecture decision is made, add it to
`wiki/002_Architecture/settled.md`. When a term causes confusion, add it to
`wiki/005_AgentMgmt/active/ctx_main/GLOSSARY.md`. Don't leave decisions only
in conversation.

## Language

English. Keep code comments minimal — explain *why*, not *what*.

# Project Agent Memory

See `wiki/005_AgentMgmt/active/ctx_main/ctx_system_current.md` for current
build status and `wiki/002_Architecture/settled.md` for locked-in decisions.

## Python environment

This project uses the conda environment `fairino-fr5-sim`. Always use its interpreter directly rather than relying on `conda activate`, since shell activation doesn't persist between commands.

- Python: `C:\Users\Edward\miniconda3\envs\fairino-fr5-sim\python.exe`
- pip: `C:\Users\Edward\miniconda3\envs\fairino-fr5-sim\Scripts\pip.exe`

Do not call bare `python`, `python3`, or `pip`. They resolve to system Python, not this environment.

This is the maintainer's local setup. `README.md` gives the portable
`pip install -r requirements.txt` route for anyone else; the two are not in
conflict — use the conda interpreter when working in this repo.

<!-- The interpreter paths above are machine-specific and should be generalised
     if this repo is handed to someone else. -->
