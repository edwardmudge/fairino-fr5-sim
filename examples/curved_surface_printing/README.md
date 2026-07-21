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

`geometry_backend.py` imports this module with one clearly-commented import
block. To point the same feature at a different curved-print job, write a
new module shaped like `study_config.py` and change that one import — the
loading/geodesic/GUI mechanism itself needs no changes.

The physical context and the reasoning behind this layer configuration is
recorded in `wiki/001_Inbox/2026-07-18_curved_surface_assets.md` and
`wiki/002_Architecture/settled.md` (S1.29–S1.33).
