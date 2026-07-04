# Polyscope + ImGui Quick Reference

## Requirements

- **Python >= 3.9** (Polyscope's C++ bindings require it)
- **Physical GPU + OpenGL >= 3.3 core profile**
- ⚠️ Does NOT work on: Remote Desktop (RDP/VNC), most VMs (VMware, VirtualBox without GPU passthrough)
- Install: `pip install polyscope`

## Minimal Application Template

```python
import polyscope as ps
import polyscope.imgui as psim
import time

ps.init()                          # ★ Must be called FIRST, before any register_*
ps.set_up_dir("z_up")             # ★ FR5 meshes use Z-up; default is Y-up (robot sideways)
ps.set_ground_plane_mode("none")   # Clean look

def my_callback():
    # --- All ImGui widgets and mesh updates go here ---
    psim.TextUnformatted("Hello Polyscope!")
    time.sleep(0.02)               # Throttle to ~50 FPS (prevents 100% CPU)

ps.set_user_callback(my_callback)  # Register the per-frame callback
ps.show()                          # ★ BLOCKING — enters the render loop, never returns
```

Key points:
- `ps.show()` is blocking — all logic must be inside the callback
- The callback runs every frame (~50 FPS)
- `ps.init()` before everything, `ps.show()` after everything

## Core APIs

### Register Geometry

```python
# Surface mesh (robot links, nozzle)
mesh = ps.register_surface_mesh("Link1", vertices, faces)
# vertices: Nx3 float array
# faces: Mx3 int array (triangle indices)

# Point cloud (TCP position)
pc = ps.register_point_cloud("TCP", points)
# points: Nx3 float array

# Curve network (trajectories, coordinate axes)
cn = ps.register_curve_network("Trajectory", nodes, edges)
# nodes: Nx3 float array
# edges: Mx2 int array (pairs of node indices)
```

### Update Geometry (each frame)

```python
mesh.update_vertex_positions(new_vertices)  # Nx3, same N as registration
```

### Remove Geometry

```python
ps.remove_surface_mesh("Link1")
ps.remove_all_structures()
```

### Visual Properties

```python
mesh.set_transparency(0.8)           # 0=invisible, 1=solid
mesh.set_color([0.8, 0.8, 0.8])      # RGB [0,1]

# Per-vertex colours
colors = np.tile([0.7, 0.7, 0.7], (N, 1))  # Nx3
mesh.add_color_quantity("color", colors, enabled=True)

# Edge colours for curve networks
edge_colors = np.array([[1,0,0], [0,1,0], [0,0,1]])  # RGB per edge
cn.add_color_quantity("axis_colors", edge_colors, defined_on='edges', enabled=True)
```

## ImGui Widgets (inside callback)

### ★ IMPORTANT: All widgets return (changed, value), not just value!

```python
# ❌ WRONG
angle = psim.SliderFloat("J1", angle, -174, 174)  # angle becomes a tuple!

# ✅ CORRECT
changed, angle = psim.SliderFloat("J1", angle, -174, 174)
```

### Common Widgets

```python
# Slider
changed, val = psim.SliderFloat("Name", current_val, min_val, max_val)

# 3-component slider
changed, vec3 = psim.SliderFloat3("Rotate", vec3, -180, 180)

# Input field
changed, val = psim.InputFloat("Name", current_val)
changed, vec3 = psim.InputFloat3("Position", vec3)

# Button (returns True when clicked)
if psim.Button("Click Me"):
    do_something()

# Checkbox
changed, is_on = psim.Checkbox("Enable Feature", is_on)

# Text
psim.TextUnformatted("Hello")

# Layout
psim.Separator()                    # Horizontal line
psim.SameLine()                     # Next widget on same line
psim.PushItemWidth(150)             # Set widget width
psim.PopItemWidth()

# Collapsible section
if psim.TreeNode("Section Name"):
    # ... widgets ...
    psim.TreePop()                  # ★ Must call TreePop() to close!
```

## Loading Meshes with trimesh

```python
import trimesh

# ★ Use force='mesh' — without it, multi-group OBJ files return a Scene object
#   (which has no .vertices or .faces, causing AttributeError)
mesh = trimesh.load("./assets/fr5_meshes/Robot1.obj", force='mesh')

vertices = mesh.vertices   # Nx3 numpy array (float64)
faces = mesh.faces          # Mx3 numpy array (int)
```

## Working Directory

All asset paths (`./assets/...`) are relative. Always run `main.py` from the
repo root:

```bash
python main.py
```
