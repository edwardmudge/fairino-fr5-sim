---
status: active
scope: current-truth
last_verified_against_code: 2026-07-17
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
| G-code toolpath preview | done | G0/G1-only parser, fixed `model.gcode` path, registered via an explicit "Load G-code preview" click — does **not** auto-reload on plate reposition (that auto-reload was removed, see `settled.md` S1.23) — see `wiki/003_Guides/Gcode_Toolpath.md`, `settled.md` S1.7/S1.8 |
| Build-plate position/orientation | done | Re-posable via Move/Reset/Save/Load Position buttons — see `wiki/003_Guides/BuildPlate_UserFrame.md`, `settled.md` S1.6 |
| Toolpath IK precompute | done | Chunked (`PRECOMPUTE_CHUNK_SIZE` waypoints/frame), pausable/resumable/cancellable, ground-clearance filtered — see `settled.md` S1.14/S1.15 |
| Toolpath precompute disk cache | done | `assets/models/gcode/model.precompute.npz`, keyed on G-code SHA-256 + build-plate pose + version; loaded before re-solving on `run_toolpath_ik_precompute()` — see `settled.md` S1.21 |
| Toolpath playback | done | Progressive-reveal (beads start invisible, revealed as playback crosses them), render-throttled (`PLAYBACK_RENDER_STRIDE`, `PLAYBACK_LOOKAHEAD_BEADS`) — see `settled.md` S1.16/S1.17-S1.20 |
| Precompute/playback invalidation on plate move | done | In-session: `load_build_plate()` compares the new pose against the pose captured at precompute-start and invalidates both if it differs — see `settled.md` S1.22 |

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

See `wiki/002_Architecture/settled.md` (S1.1–S1.24).
