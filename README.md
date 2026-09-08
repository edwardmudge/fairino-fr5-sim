# Fairino FR5 Simulation

A Python-based 3D visualisation tool for a 6-degree-of-freedom Fairino FR5 manipulator that simulates 3D-printing toolpaths on a repositionable build plate — both flat-bed G-code printing and conformal printing onto a curved surface.

## Demo

### Planar printing

![Planar printing demo](docs/media/planar_printing.gif)

## Requirements

* Python 3.12 (developed on 3.12.13)
* NumPy 2.5.1, Polyscope 2.6.1, Trimesh 4.12.2 — see `requirements.txt`
* **A physical GPU with OpenGL 3.3 or newer.** Polyscope opens a real OpenGL
  window, so this will not run over Remote Desktop, in most VMs, or in a headless
  container. This is the most common reason the app fails to start.

## Usage

```bash
git clone https://github.com/edwardmudge/fairino-fr5-sim.git
cd fairino-fr5-sim
pip install -r requirements.txt
python main.py
```

If you use conda (the maintainer's setup is an env named `fairino-fr5-sim`),
install into that env and run its interpreter directly rather than relying on
shell activation — see `AGENTS.md`.

Assets are located relative to the source files, not the working directory, so
`python main.py` also works when launched from elsewhere (e.g. an IDE Run button).

### First run

The build plate starts at the **saved calibrated User Frame**
(`assets/buildPlate/saved_position.json`), which is the pose the shipped curved
toolpath solutions were computed at. Leave it there for the bundled demo: the
precompute caches only match that pose, and at any other pose the curved job is
re-solved from scratch (roughly half an hour per layer). The Build Plate
Orientation panel's **Reset** button moves to the older demo pose if you want it.

A working curved run is:

1. **Build Plate Orientation** — leave at the startup pose (or Move, then continue)
2. **I/O Operations → Load Curved Model**
3. **Build Geodesics → Build Print Order → Build Orientation Frames**
4. **Toolpath Source → RX** (or TX), then **Toolpath Settings → Run Precompute**
5. **Run Toolpath** to animate, **Export IK Job** to write the robot job

The order matters — moving the plate after step 2 invalidates the geodesics and
the model must be reloaded. See
[`wiki/003_Guides/CurvedModel_AdaptingYourOwnJob.md`](wiki/003_Guides/CurvedModel_AdaptingYourOwnJob.md).

The planar demo additionally needs a `model.gcode` in
`assets/models/planar/gcode/`; that file is not committed, so "Load G-code
preview" reports that it is missing on a fresh clone.

## Printing your own curved part

The curved-surface-printing feature is project-agnostic — it operates on whatever
a *study config* describes. To point it at your own job, write a module shaped
like `examples/curved_surface_printing/study_config.py` and select it with an
environment variable; no source edit is needed:

```bash
FR5_STUDY_CONFIG=mystudy.study_config python main.py
```

**[`wiki/003_Guides/CurvedModel_AdaptingYourOwnJob.md`](wiki/003_Guides/CurvedModel_AdaptingYourOwnJob.md)
is the guide** — asset formats, the reach constraint that governs where your part
can sit, which constants to re-tune, and how to export the result.

## Project layout

* `main.py`, `gui_panel.py`, `geometry_backend.py` — Polyscope app (Model-View architecture), including the project-agnostic curved-surface-printing feature
* `examples/` — the concrete dataset/config that feature ships with by default (an elastomeric sensor printed conformally onto a shoulder mockup)
* `assets/` — robot arm, tool head, build plate, and curved-print-surface meshes
* `docs/` — technical reference (DH parameters, joint limits, mesh coordinate convention)
* `wiki/` — project knowledge base: operation guides, architecture decisions, glossary, agent boot protocol
* `wiki-template/` — the generic, project-agnostic methodology behind `wiki/`, reusable to bootstrap this documentation system on other projects

### A note on roadmap references

Source docstrings and wiki pages frequently cite `tutorials/Stage5_README.md`,
`Stage6_README.md`, `Stage7_README.md` and similar. That directory is local
assignment scaffolding and is **not published** — a clone will not contain it, and
those citations are historical provenance rather than links you can follow. The
published equivalents are `wiki/003_Guides/` for how a feature is operated and
`wiki/002_Architecture/settled.md` for why it is built the way it is.
