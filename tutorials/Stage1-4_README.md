# FR5 Robot Arm Kinematics Tutorial Project

## What This Project Does

Build a **kinematics simulation system** for the Fairino FR5 six-axis robot arm from scratch — fully offline, no real robot or simulator connection needed.

You will accomplish two things:

**Forward Kinematics (FK)**
Given 6 joint angles → compute the spatial pose of each arm segment → see the robot arm "come alive" in a 3D viewer.
Drag any joint slider and the entire arm from that joint onward follows — the TCP (Tool Centre Point, nozzle tip) tracks in real time.

**Inverse Kinematics (IK) — Analytical Solution**
Given a target end-effector position → solve for all possible joint angle combinations (up to 8 solutions for the FR5).
Switch between different solutions in the UI to intuitively experience how one target point maps to completely different arm configurations — this is IK redundancy.

Final result: an interactive Polyscope program with a control panel on the left, a 3D robot arm on the right, and free switching between FK and IK modes.

---

## Why Use the Human-Wiki-Agent Collaboration Approach

Before you start writing code, open **[wiki-beginner-guide.html](wiki-beginner-guide.html)** and read it through.

That guide explains a core idea:

> A person's working memory can handle only 4–7 items at once. But a robot control system has hundreds of tightly coupled technical details.
> **You don't need to keep all of them in your head.** Crystallise them into a knowledge network — you focus on the current node, and the wiki + AI agent manages the rest.

This tutorial project puts that method into practice:

- **You (Human)** — make high-level judgements: Is the FK formula derivation correct? What is the physical meaning of each IK solution? Which solution is most reasonable?
- **Wiki / Docs** — manage the details: DH parameter table, mesh coordinate conventions, Polyscope API usage, joint limit ground truth. You don't need to memorise them — just look them up.
- **AI Agent** — helps you write code, debug, and explain concepts. But it doesn't make decisions for you — you understand the principles, it helps you implement efficiently.

This is not "let the AI write everything and submit it as your homework." This is you learning **how to engineer solutions together with AI** — defining requirements, reviewing plans, verifying results. This is the core skill of future engineers.

---

## Materials Checklist

All raw data is in this project's root folder, including:

| Content | Path | Description |
|---------|------|-------------|
| Polyscope blank framework | `main.py` + `gui_panel.py` + `geometry_backend.py` | Three-layer architecture already working — `python main.py` opens the 3D window |
| Robot arm meshes | `assets/fr5_meshes/Robot0~6.obj` | 7 OBJ files: Base + 6 Links |
| Printer head | `assets/printerHead/nozzle.obj` + `TCP.txt` | Tool head mesh + nozzle tip coordinate |
| Build plate | `assets/buildPlate/` | Used in the advanced stage |
| DH parameter table | `docs/FR5_DH_Table.md` | Standard DH formula + parameters + zero-position verification values |
| Joint limits | `docs/FR5_Joint_Limits.md` | 6-axis ground truth limits + Home pose |
| Mesh coordinate convention | `docs/FR5_Mesh_Convention.md` | **Must read** — skipping this guarantees incorrect mesh rendering |
| Polyscope quick reference | `docs/Polyscope_Quickstart.md` | Core API + common pitfalls |

**There is no pre-built FK/IK code.** You and the AI build everything from scratch.

---

## Learning Path

### Step 1: Reading

1. Open **[wiki-beginner-guide.html](wiki-beginner-guide.html)** to understand the Human-Wiki-Agent collaboration approach
2. Open **[FR5_FK_Kit_README.md](FR5_FK_Kit_README.md)** to learn the kit structure and 4-stage technical roadmap
3. Run `python main.py` to confirm the Polyscope window opens

### Step 2: FK Build (Stages 1–3)

Follow the Stage 1 → 2 → 3 order in `FR5_FK_Kit_README.md`, collaborating with AI:

**Stage 1 — FK Maths**
- Implement the DH homogeneous transform matrix
- Chain-multiply to get `T_0_1` through `T_0_6`
- Verify zero-position end-effector = `[-820, -202, 50]` mm

**Stage 2 — 3D Rendering**
- Load the 7 OBJ meshes into Polyscope
- Drive meshes using Delta incremental transforms (NOT direct `T_0_i` multiplication!)
- 6 joint angle sliders with real-time interaction

**Stage 3 — Tool Head + TCP**
- Load the nozzle mesh, attached to Link 6
- Real-time TCP point tracking, optional trajectory drawing

### Step 3: IK Build (Stage 4+)

With FK running stably, add Analytical IK:

**IK Core Functionality**
- Given a target pose `[x, y, z, rx, ry, rz]` → analytical solution (up to 8 sets of joint angles)
- Filter out solutions that exceed joint limits, keeping only feasible ones
- Forward-verify each solution with FK: `compute_fk(solution) ≈ target`

**IK Interactive UI**
- Input fields: specify target end-effector pose
- Solution list: display all feasible solutions (e.g., "Solution 1/5", "Solution 2/5")
- Toggle button or dropdown: select a different solution and the arm instantly jumps to that configuration
- **Intuitive experience**: for the same end-effector position, "elbow up" and "elbow down" look completely different — that's redundancy

---

## Final Result

After completion, you will have a fully interactive simulator:

```
┌─────────────────────────────────────────────────────┐
│  Polyscope 3D Window                                │
│                                                     │
│  ┌──────────────┐    ┌─────────────────────────┐   │
│  │ Control Panel │    │                         │   │
│  │              │    │    FR5 Robot Arm 3D      │   │
│  │ [FK Mode]    │    │    (7 meshes + nozzle)   │   │
│  │ J1: ──●───── │    │                         │   │
│  │ J2: ────●─── │    │         ↗ TCP trail      │   │
│  │ J3: ─●────── │    │        ●                 │   │
│  │ J4: ──────●─ │    │       /                  │   │
│  │ J5: ───●──── │    │      /                   │   │
│  │ J6: ─────●── │    │     ■ Robot arm           │   │
│  │              │    │    /|                     │   │
│  │ [IK Mode]    │    │   / |                    │   │
│  │ Target:      │    │  ■  |                    │   │
│  │  x: -500     │    │  |  ■                    │   │
│  │  y: -200     │    │  ■──■──■                 │   │
│  │  z:  300     │    │  ┃ Base                  │   │
│  │              │    │  ═══════                 │   │
│  │ Solutions:   │    │     Build plate           │   │
│  │ ● Sol 1/5   │    │                         │   │
│  │ ○ Sol 2/5   │    └─────────────────────────┘   │
│  │ ○ Sol 3/5   │                                   │
│  │ ...          │                                   │
│  └──────────────┘                                   │
└─────────────────────────────────────────────────────┘
```

**FK Mode**: drag sliders → arm moves → TCP tracks
**IK Mode**: enter target point → list all solutions → click to switch → observe redundancy

---

## Environment Requirements

- Python >= 3.12
- Physical GPU + OpenGL >= 3.3 (no Remote Desktop / VM support)
- `pip install numpy polyscope trimesh`

See `FR5_FK_Kit_README.md` for details.

---

## What's Next: Beyond the Kit

All 4 stages above are complete — FK, mesh rendering, TCP, and IK are
done. The project's next phase, set by the supervisor, moves from pure
kinematics into using the arm for 2D printing on the build plate, then
onto curved surfaces, then out to a real controller:

- [`Stage5_README.md`](Stage5_README.md) — flat-bed printing from real
  Cura-sliced G-code
- [`Stage6_README.md`](Stage6_README.md) — curved-surface (conformal) printing
- [`Stage7_README.md`](Stage7_README.md) — real calibration and job export

Read Stage 5 before starting Stage 5 work; each stage builds on the last.
[`README.md`](README.md) in this folder gives the full reading order.
