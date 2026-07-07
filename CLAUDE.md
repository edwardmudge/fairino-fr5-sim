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
`geometry_backend.py` and `gui_panel.py` are the only files that should
change during FK/IK development. `main.py` is wiring and shouldn't need
edits once Stage 1 starts.

## 4. Goal-Driven Execution
Follow the 4-stage roadmap: FK maths → mesh rendering → tool head/TCP → IK.
Don't jump ahead to IK before FK visually works — the Delta transform in
Stage 2 depends on a correct `compute_fk()` from Stage 1.

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

<!-->
(Note to self: change this in final version)
-->