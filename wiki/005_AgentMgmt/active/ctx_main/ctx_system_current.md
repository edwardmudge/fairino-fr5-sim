---
status: active
scope: current-truth
last_verified_against_code: 2026-07-04
---

# Agent Boot File — FR5 Simulator (Current State)

## Step 0: Who You Are

You're helping build an offline FK/IK simulator for a Fairino FR5 6-axis
arm, rendered in Polyscope. No real robot or hardware connection — this is
pure math + visualisation.

## Step 1: 30-Second Project Overview

### What the System Does

Takes 6 joint angles (or a target end-effector pose) and renders the
corresponding FR5 arm configuration in an interactive 3D window.

### Current Status

| Feature | Status | Notes |
|---------|--------|-------|
| Polyscope app skeleton | done | `main.py` opens a window with just a coordinate frame |
| FK maths (`compute_fk`) | not started | Stage 1 — see `docs/FR5_DH_Table.md` |
| Mesh loading + rendering | not started | Stage 2 — see `docs/FR5_Mesh_Convention.md` (Delta transform) |
| Joint sliders | not started | Stage 2 |
| Tool head + TCP tracking | not started | Stage 3 |
| Analytical IK | not started | Stage 4 |

## Directory Structure

```
/
├── assets/          FR5 link meshes, nozzle, build plate
├── docs/            DH table, joint limits, mesh convention, Polyscope API
├── wiki/            you are here
├── main.py          entry point, already working
├── gui_panel.py      UI panel — stub, add sliders here
├── geometry_backend.py  backend — stub, add FK/IK here
└── requirements.txt
```

## Architecture Model

Model-View split: `geometry_backend.py` (VisContent) owns geometry state
and math; `gui_panel.py` (UI_Menu) owns ImGui widgets and calls into the
backend; `main.py` wires them together and drives Polyscope's per-frame
callback. See `wiki/002_Architecture/INDEX.md` as subsystems get built out.

## Key Constraints

- Mesh vertices are baked in zero-pose world coordinates — always apply the
  Delta transform (`GLOSSARY.md` §2), never `T_0_i(q)` directly.
- Requires a physical GPU with OpenGL >= 3.3 — won't run over Remote
  Desktop or in most VMs.

## Recent Decisions

None yet — see `wiki/002_Architecture/settled.md`.
