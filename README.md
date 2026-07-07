# Fairino FR5 Simulation

A Python-based 3D visualisation tool for a 6-degree-of-freedom Fairino FR5 manipulator.

## Dependencies

* Python 3.12
* Polyscope 2.6.1
* NumPy 2.5.0
* Trimesh 4.12.2

## Project layout

* `main.py`, `gui_panel.py`, `geometry_backend.py` — Polyscope app (Model-View architecture)
* `assets/` — robot arm, tool head, and build plate meshes
* `docs/` — technical reference (DH parameters, joint limits, mesh coordinate convention)
* `wiki/` — project knowledge base: architecture decisions, glossary, agent boot protocol
* `wiki-template/` — the generic, project-agnostic methodology behind `wiki/`, reusable to bootstrap this documentation system on other projects

## Usage

```bash
git clone https://github.com/edwardmudge/fairino-fr5-sim.git
cd fairino-fr5-sim
pip install -r requirements.txt
python main.py
```
