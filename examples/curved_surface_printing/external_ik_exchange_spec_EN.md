# External IK Solution Exchange Spec

> Version: 2.0 | Date: 2026-07-19
>
> Collaborators independently provide the complete "print job" package; we ingest and execute it directly.

---

## Convention

- Positions: **Robot Base Frame**, mm
- Angles: **degrees**
- Joints: J1–J6
- Rotation: `R = Rz(rz) @ Ry(ry) @ Rx(rx)` (intrinsic XYZ / extrinsic ZYX)
- DH: **Standard FR5, d4 = 102mm** (see `saved_coords_data_and_usage_EN.md` §5, ignore §6)
- Nozzle axis: **TCP local -Z**
- Normals: **unit vectors**, Robot Base Frame
- File: **strict UTF-8 JSON**

---

## Folder Structure

```
print_job_TX_sensors/
├── surface.obj                    ← print surface mesh
├── toolpath_T0.ply                ← ply curve for segment 0
├── toolpath_T1.ply                ← ply curve for segment 1
├── toolpath_T2.ply                ← ply curve for segment 2
├── ...
├── segment_0_solution.json        ← joint angle solution for segment 0
├── segment_1_solution.json        ← joint angle solution for segment 1
├── segment_2_solution.json        ← joint angle solution for segment 2
├── ...
└── job.json                       ← master control file, references all segments
```

Each `toolpath_T*.ply` corresponds to one continuous extrusion line (one segment). Each segment has its own independent solution JSON. `job.json` organizes them into the execution sequence.

---

## job.json (Master Control File)

```jsonc
{
  "format": "fr5_external_ik_job",
  "format_version": "2.0",
  "generator": "graph_search_optimizer_v2",
  "generated_utc": "2026-07-19T12:00:00Z",

  // ═══ TCP + Identity ═══
  "tool_index": 1,
  "tcp_offset_6d": [-134.777, 96.448, 106.334, 86.647, -13.136, 60.612],
  "identity_check": {
    "joints_zero_tcp_pose_base": [-954.777, -308.334, 146.448, -161.378, -58.051, -25.434]
  },

  // ═══ Execution Sequence ═══
  "segments": [
    {"segment_id": 0, "toolpath": "toolpath_T0.ply", "solution": "segment_0_solution.json"},
    {"segment_id": 1, "toolpath": "toolpath_T1.ply", "solution": "segment_1_solution.json"},
    {"segment_id": 2, "toolpath": "toolpath_T2.ply", "solution": "segment_2_solution.json"}
  ]
}
```

The order of the `segments` array = the print execution order. A travel MoveJ is automatically inserted between adjacent segments by us.

---

## segment_N_solution.json (Joint Angle Solution for Each Segment)

```jsonc
{
  "segment_id": 0,
  "toolpath_file": "toolpath_T0.ply",
  "num_points": 800,
  "points": [
    {
      "joints_deg": [58.114, -114.874, -121.430, 72.659, 68.246, -99.604],
      "tcp_xyz_base_mm": [531.699, 310.076, 236.433],
      "normal_base": [-0.795, -0.552, 0.252]
    },
    {
      "joints_deg": [58.996, -112.518, -120.379, 64.222, 63.289, -96.553],
      "tcp_xyz_base_mm": [532.100, 310.500, 236.200],
      "normal_base": [-0.795, -0.552, 0.251]
    }
    // ... corresponds 1:1 with the lines of toolpath_T0.ply
  ]
}
```

### Field Descriptions

| Field | Description |
|------|------|
| `segment_id` | Corresponds to the one in job.json |
| `toolpath_file` | Filename of the referenced ply file |
| `num_points` | Redundancy check = length of the `points` array = number of lines in the ply file |
| `points[].joints_deg` | 6 joint angles, degrees |
| `points[].tcp_xyz_base_mm` | FK+TCP position (for verification) |
| `points[].normal_base` | Surface normal unit vector (Base Frame) |

### Correspondence with the ply File

- `points[i]` corresponds to line `i` of `toolpath_T*.ply`
- `tcp_xyz_base_mm` should match the ply's `x y z` (or be within 2mm due to optimization fine-tuning)
- `normal_base` should match the ply's `nx ny nz` (the same surface normal)

---

## toolpath_T*.ply Format

Each line has 6 space-separated columns:
```
x y z nx ny nz
```

- `x y z`: TCP target position (Base Frame, mm)
- `nx ny nz`: Surface normal (Base Frame, unit vector)
- Line order = print direction (the nozzle moves along this path)

---

## What We Provide to Collaborators

1. `saved_coords_data_and_usage_EN.md` — DH, TCP, rotation convention, FK verification
2. TX surface mesh (OBJ)
3. TX sensor ply curves (original design curves)

Collaborators independently complete: surface projection → segment division → graph search → optimization → output of the entire job folder.

---

## Rejection Criteria

| Check | Rejection Condition | Action |
|------|----------|------|
| Identity check | pos >= 0.1mm OR rot >= 0.5° | REJECT |
| TCP offset vs. our calibration | pos >= 0.5mm OR rot >= 0.5° | REJECT |
| Joint limits | Any joint out of range | REJECT |
| Per-point FK vs `tcp_xyz_base_mm` | error >= 0.1mm | REJECT |
| Joint step between adjacent points within a segment > 30° | Not allowed | REJECT |
| `num_points` != ply line count | Mismatch | REJECT |
| \|J5\| < 2° | Singular configuration | WARN |

---

## Reference Values

```
Standard DH (d4=102), joints = [0,0,0,0,0,0]:
  Flange xyz (Base):  [-820.000, -202.000, 50.000]

With TCP tool=1 [-134.777, 96.448, 106.334, 86.647, -13.136, 60.612]:
  TCP 6D (Base):      [-954.777, -308.334, 146.448, -161.378, -58.051, -25.434]

Euler extraction:
  ry = arcsin(-R[2,0])
  rx = arctan2(R[2,1]/cos(ry), R[2,2]/cos(ry))
  rz = arctan2(R[1,0]/cos(ry), R[0,0]/cos(ry))
```
