---
status: active
scope: current-truth
last_verified_against_code: 2026-07-08
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
| Polyscope app skeleton | done | `main.py` wires `VisContent`/`UI_Menu` into Polyscope's per-frame callback |
| FK maths (`compute_fk`) | done | See `docs/FR5_DH_Table.md` |
| Mesh loading + rendering | done | Delta-transform pipeline — see `docs/FR5_Mesh_Convention.md` |
| Joint sliders | done | "Forward Kinematics" panel, `gui_panel.py` |
| Tool head + TCP tracking | done | See `wiki/003_Guides/TCP_Frame.md` |
| Analytical IK | done | Closed-form solver, multi-solution — see `settled.md` S1.4/S1.5 |
| TCP trajectory recording | done | See `wiki/003_Guides/TCP_Trajectory.md` |
| G-code toolpath preview | done | See `wiki/003_Guides/Gcode_Toolpath.md` |

## Directory Structure

```
/
├── assets/          FR5 link meshes, nozzle, build plate
├── docs/            DH table, joint limits, mesh convention, Polyscope API
├── wiki/            you are here
├── main.py          entry point, wires backend + UI together
├── gui_panel.py      UI panel — joint sliders, IK controls, I/O buttons
├── geometry_backend.py  backend — FK/IK, mesh rendering, TCP/trajectory, G-code
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

See `wiki/002_Architecture/settled.md` (S1.1–S1.5).
