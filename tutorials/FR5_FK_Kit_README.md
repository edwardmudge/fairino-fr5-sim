# FR5 Robot FK & Polyscope Visualisation Kit

Build a real-time 3D visualisation of the Fairino FR5 6-axis robot arm from scratch.

## What You Get

- **Polyscope blank framework** — `main.py` + `gui_panel.py` + `geometry_backend.py` (Model-View architecture, already wired up)
- **FR5 mesh files** — 7 OBJ files for the robot arm (base + 6 links), ~19 MB total
- **Nozzle** — 3D printer tool head mesh + TCP tip coordinate
- **Build plate** — print platform mesh (Stage 4)
- **Reference docs** — DH parameters, joint limits, mesh coordinate convention, Polyscope API

## What You Build

Using AI as your collaborator, implement Forward Kinematics and bring the robot to life in 3D.

## Environment Setup

**Requirements:**
- Python >= 3.12
- Physical GPU with OpenGL >= 3.3 (no VM / Remote Desktop)
- Windows / macOS / Linux

**Install:**
```bash
pip install -r requirements.txt
```

Or with conda:
```bash
conda activate FastIKD          # or your own env name
pip install numpy polyscope trimesh
```

**Run:**
```bash
python main.py
```

You should see a Polyscope window with a coordinate frame. This is your starting point.

---

## Learning Roadmap (4 Stages)

### Stage 1 — FK Maths Engine (~2-3 hours)

> Pure maths, no visualisation yet.

**Goal:** Implement FR5 Forward Kinematics using numpy.

1. Read `docs/FR5_DH_Table.md` for the DH parameters and transform formula
2. Implement `dh_transform(a, alpha, d, theta)` → 4×4 matrix
3. Implement `compute_fk(joints_deg)` → list of 6 transforms `[T_0_1, ..., T_0_6]`
4. Extract end-effector position from `T_0_6[:3, 3]`

**Verify:** `compute_fk([0, 0, 0, 0, 0, 0])` → end-effector at **[-820, -202, 50] mm**

**Where to put it:** Add your FK class to `geometry_backend.py`.

---

### Stage 2 — Mesh Loading & Visualisation (~4-6 hours)

> Make the robot move in 3D. **This is the hardest stage.**

**Goal:** Load the robot mesh files and drive them with FK.

1. Load `assets/fr5_meshes/Robot0–6.obj` with `trimesh.load(path, force='mesh')`
2. Register them in Polyscope as surface meshes
3. **★ Read `docs/FR5_Mesh_Convention.md` carefully** — the meshes use a Delta transform pattern, NOT direct `T_0_i` multiplication
4. Add 6 joint angle sliders in `gui_panel.py` using `psim.SliderFloat`
5. Each frame: recompute FK → compute Delta → update vertex positions

**Verify:** Drag the J1 slider — the entire arm from shoulder down should rotate together. All joints should stay connected with no gaps or overlap.

**⚠️ Common pitfalls:**
- `psim.SliderFloat()` returns `(changed, value)` tuple — unpack it!
- `trimesh.load()` needs `force='mesh'` to avoid getting a Scene object
- See `docs/Polyscope_Quickstart.md` for the complete gotcha list

---

### Stage 3 — Tool Head & TCP (~2-3 hours)

> Attach the 3D printer nozzle and track its tip.

**Goal:** Add the nozzle mesh and display the TCP (Tool Centre Point).

1. Load `assets/printerHead/nozzle.obj` and register it
2. Use the same Delta transform as Link 6: `Delta_flange = T_0_6(q) @ inv(T_0_6(0))`
3. Read `assets/printerHead/TCP.txt` → `[-798.137, -228.017, -109.903]`
4. Transform TCP point: `tcp_world = (Delta_flange @ [x, y, z, 1])[:3]`
5. Display TCP as a point cloud in Polyscope
6. (Optional) Record TCP positions to draw a trajectory curve

**Verify:** When you move the sliders, the nozzle follows the flange, and the TCP point stays at the nozzle tip.

---

### Stage 4 — Advanced Extensions (~3-5 hours, optional)

Pick any of these:
- **Coordinate frame axes** — draw RGB (X/Y/Z) axes at the TCP using curve networks
- **Build plate** — load `assets/buildPlate/BambuLab_BuildPlate.obj`, position with a User Frame transform
- **User Frame** — implement `T_base_to_user` coordinate transform, display Work Object frame
- **G-code preview** — load a simple G-code file and visualise the toolpath
- **Home button** — add a button that sets joints to `[0, 0, 0, 0, 90, 0]` (tool pointing down)

---

## File Structure

```
main.py                 ← Entry point (already working)
gui_panel.py            ← ImGui UI panel (add your sliders here)
geometry_backend.py     ← Backend logic (add your FK here)
requirements.txt
.polyscope.ini          ← Polyscope window state
imgui.ini               ← ImGui layout state

assets/
├── fr5_meshes/         ← Robot arm meshes (7 OBJ files)
├── printerHead/        ← Nozzle mesh + TCP point
└── buildPlate/         ← Print platform (Stage 4)

docs/
├── FR5_DH_Table.md     ← DH parameters + transform formula + verification
├── FR5_Joint_Limits.md ← Joint angle limits + Home position
├── FR5_Mesh_Convention.md ← ★ READ THIS — mesh transform pattern
└── Polyscope_Quickstart.md ← API reference + common pitfalls
```

## Estimated Total Time

~11–17 hours with AI assistance across all 4 stages.

## What's Next

With FK, mesh rendering, TCP and analytical IK working, the project moves from
pure kinematics into printing: [`Stage5_README.md`](Stage5_README.md) (flat-bed
G-code printing), then [`Stage6_README.md`](Stage6_README.md) (curved surfaces)
and [`Stage7_README.md`](Stage7_README.md) (real calibration and job export).
[`README.md`](README.md) in this folder gives the full reading order.
