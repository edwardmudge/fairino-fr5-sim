---
status: active
scope: How to point the curved-surface-printing feature at a different part
---

# Curved Printing: Adapting It To Your Own Job

The curved-surface-printing feature is project-agnostic. Loading, geodesic travel
routing, print ordering, tool orientation, the IK search and job export all
operate on whatever a **study config** describes. The shipped config prints an
elastomeric capacitive sensor conformally onto a shoulder mockup; nothing about
that job is baked into the engine.

This guide is what you need to substitute your own part. It is self-contained on
purpose — source docstrings cite `tutorials/Stage6_README.md` /
`Stage7_README.md`, which are not published (see `wiki/INDEX.md`).

Related: [`CurvedModel_PrintSetup.md`](CurvedModel_PrintSetup.md) (operating the
shipped job), [`CurvedModel_Loading.md`](CurvedModel_Loading.md),
[`CurvedModel_Geodesics.md`](CurvedModel_Geodesics.md),
[`CurvedModel_PrintOrder.md`](CurvedModel_PrintOrder.md).

---

## 1. Selecting your config

Write a module with the same names as
`examples/curved_surface_printing/study_config.py` and select it with an
environment variable — no source edit:

```bash
FR5_STUDY_CONFIG=mystudy.study_config python main.py
```

The module must be importable (on `sys.path`, with `__init__.py` files if it
sits in a package). Unset, it defaults to the shipped study.

Every name listed in `_STUDY_CONFIG_NAMES` in `geometry_backend.py` must be
defined. A missing one fails at import with a message naming it, rather than as
an `AttributeError` hundreds of frames later inside a render callback.

## 2. Asset formats

Everything lives in `CURVED_MODEL_DIR` (relative paths are resolved against the
repo; an absolute path is used as given, so assets may live outside the repo).

### Toolpath curves — ASCII PLY, vertices + edges, **no faces**

One file per curve piece, listed in each layer's `curve_files`. The reader
(`read_ply_polyline`) expects exactly this shape:

```
ply
format ascii 1.0
element vertex 108
property float x
property float y
property float z
element edge 54
property int vertex1
property int vertex2
end_header
-84.808334 75.438103 -126.235435
...            <- n_vertex lines of "x y z"
0 1
...            <- n_edge lines of "i j"
```

Constraints that actually bite:

- **ASCII only, and no `element face`.** These files are deliberately faceless;
  `trimesh.load(force='mesh')` cannot read them, which is why there is a
  hand-rolled reader.
- **Edges are a disjoint segment soup**, not an ordered walk.
  `reconstruct_polylines()` reassembles them by deduping coordinates rounded to
  `CURVE_DEDUPE_DECIMALS` (3 dp) — float export noise keeps genuinely distinct
  points apart well past that. If your exporter writes more precision than 3 dp
  of *real* separation between distinct points, raise that constant.
- **Closed loops are fine.** A piece whose two ends coincide snaps to one vertex;
  the print-order code expects and handles that.
- Units are **millimetres**, matching the rest of the project.

### Surfaces — OBJ

Each layer names a `surface_file`: the mesh its toolpath lies on and its
geodesics are routed over. Loaded with trimesh, so any OBJ it reads works.

- **Each layer routes on its own surface.** Travel moves never cross between
  layers, so a geodesic between two layers' endpoints is meaningless.
- The surface must be **connected** where you expect travel. A geodesic between
  endpoints in different connected components is unreachable; that gap is skipped,
  and `build_curved_toolpath_waypoints_world` asserts the 1:1 pairing of travel
  moves to piece gaps, so it fails loudly rather than mis-stitching.
- Denser is not better. `Surface_TX_Base` is ~90k triangles and is binned once
  into a broadphase grid at `SURFACE_GRID_CELL_MM` (8 mm, ≈6× the shipped meshes'
  ~1.24 mm median edge). If your mesh is much coarser or finer, scale that
  constant so a cell holds a handful of triangles.

### `CURVED_OBSTACLE_FILE` — normals only, optional

This mesh **is not a collision body**. Its only job is to decide which way is
"outward" so surface normals point away from the part rather than into it
(`_orient_normals_outward`). Getting the sign wrong drives the nozzle into the
workpiece. Set it to a body *underneath* your print surfaces. If you omit it
(`None`), outward is decided against each surface's own centroid, which is
correct for a convex-ish dome and unreliable otherwise.

## 3. Placement — the reach constraint

This is the part most likely to go wrong, and it is a **reach** constraint, not a
convention.

`load_curved_model()` rotates your assembly by `CURVED_MODEL_ROTATE_X_DEG` about
local X, centres its XY bounding box on `CURVED_MODEL_XY_OFFSET_MM` **relative to
the User Frame origin**, and lifts it so its lowest point rests on the plate's
print face.

Keep `CURVED_MODEL_XY_OFFSET_MM = (0, 0)` unless you have measured otherwise.
The worked example of getting this wrong is recorded in
`wiki/001_Inbox/2026-09-03_curved_placement_plate_centring_offset.md`: placement
was once centred on the build-plate *mesh's* bbox centre, but
`BambuLab_BuildPlate.obj` is a stand-in whose origin sits at a corner, so this
added a **+105.6 mm** outward shift — pushing the workpiece past the arm's
`a2 + a3 + d5 = 922 mm` flange reach. IK reachability fell to 76 % (RX) and 70 %
(TX) even with the full orientation search; at `(0, 0)` both are 100 %.

`CURVED_MODEL_ROTATE_X_DEG` exists because the CAD "+Z up" assumption was wrong
for the shipped assets. Check which way your printable surface faces and set it
accordingly — if normals point into the part, you will see it immediately in the
orientation triads (step 3 of the build sequence).

**A large part may simply not fit.** The arm's envelope is ~820 mm and the
flange–TCP offset sits laterally off the flange, so the usable region is smaller
than the raw reach suggests. If waypoints fail, that is the first thing to check.

## 4. Build order — it is not reorderable

```
Build Plate Orientation  (set the plate pose FIRST)
  → I/O Operations → Load Curved Model
    → Build Geodesics
      → Build Print Order
        → Build Orientation Frames
          → Toolpath Source → <layer>
            → Toolpath Settings → Run Precompute
              → Run Toolpath  /  Export IK Job
```

Each stage consumes the previous stage's output, and the GUI gates them in this
order. The trap is the plate: **moving it after loading the model invalidates the
geodesics**, because the retained world vertices and stored geodesic paths are
built against the plate pose. You will be told to reload the curved model, and
you must — the geodesic *costs* survive a rigid move but the *paths* do not.

The panel lists Build Plate Orientation *below* I/O Operations, so top-down
reading gives you the wrong order. Set the plate first.

Since v1.0 the plate starts at the saved calibrated User Frame
(`assets/buildPlate/saved_position.json`), so for the shipped job this step is
already done at startup.

## 5. Constants to re-tune

**Material and nozzle values — in your study config** (settled.md S1.41):

| Constant | Shipped | What it does |
|---|---|---|
| `CURVED_TRAVEL_HOVER_MM` | 4.0 | How far a travel move lifts off along the local normal, so the nozzle clears wet traces |
| `CURVED_TIP_CLEARANCE_TOLERANCE_MM` | 1.0 | Filter 8's clearance between **arm links** and the print surface |
| `CURVED_BEAD_WIDTH_MM` | 1.5 | Deposited bead cross-section (visual only — PLY curves carry no extrusion data) |
| `CURVED_BEAD_HEIGHT_MM` | 0.5 | As above |

All four are assumptions, not measurements. Tune them empirically for your
material.

**Robot and planner values — in `geometry_backend.py`**, and correct as they
stand for any FR5: `PHYSICAL_JOINT_LIMITS`, `TCP_OFFSET_6D_MM_DEG` (the
calibrated tool=1 offset), the nine `FILTER_*` values, the `EDGE_*` costs, and
the `ORIENT_SEARCH_*` shape. Change these only for a different robot, a different
tool, or a deliberate planner change — and see §6, because they are now part of
the cache key.

If waypoints fail to solve, the status line reports a per-filter breakdown. Read
it: a failure dominated by `limits/reach` means the arm genuinely cannot get
there (a placement problem, §3); one dominated by a single filter name means that
filter is mistuned for your geometry.

## 6. Caching — and why retuning now re-solves

A completed precompute is cached per layer at
`CURVED_MODEL_DIR/curved_<layer>.precompute.npz`. A hit skips the whole search,
which matters: the curved search runs 540 commanded orientations × up to 8 IK
branches per waypoint, and measures ~0.44–0.75 s **per waypoint** — around half
an hour for a 3,000-waypoint layer.

The key covers the layer name, a hash of the waypoint positions / feed flags /
normals, the build-plate pose, the orientation-search shape, filter 8's
clearance, and the tuned solver constants (`_solver_cache_fields()`). So:

- Re-ordering, re-orienting, moving the plate, or **retuning any filter, joint
  limit, edge cost or the TCP offset** correctly produces a **miss** and re-solves.
- Editing a value that does not affect the solve does not invalidate anything.

⚠ Before v1.0 the solver constants were *not* in the key, so retuning a filter
silently served the stale joint path. If you are carrying an older cache from
before that change, delete it rather than trusting it.

`PRECOMPUTE_CACHE_VERSION` remains the blunt instrument: bump it to invalidate
every cache everywhere, and do so if you change the *structure* of what is cached.

## 7. Exporting

**Export IK Job** writes the active toolpath source to
`assets/export/<job_name>/` and zips it to
`assets/export/<YYYYMMDD>-<name>.zip`. The job folder contains:

- `job.json` — the manifest: format version, generator, tool index, TCP offset,
  an identity check, and the segment list
- `toolpath_T<N>.ply` — one per segment, six space-separated columns
  `x y z nx ny nz`, headerless (the receiving side's spec, deliberately)
- `segment_<N>_solution.json` — per point: `joints_deg`, `tcp_xyz_base_mm`,
  `normal_base`
- `surface.obj` — the layer's print surface (curved jobs only)

A **segment is one continuous extrusion run**; travel moves are dropped, because
the receiving side re-inserts a travel `MoveJ` between segments. That means file
count scales with segment count: a curved layer yields ~35 segments (~72 files),
while the planar benchy yields ~20,350 (~40,700 files). The status line reports
the count before the write starts.

Export runs `validate_job()` first — an in-house "job is non-empty" row plus the
exchange spec's seven Rejection Criteria — and a REJECT **gates the write**. On
failure you get the full per-row table naming the offending segment and point.

⚠ **These rows validate data, not geometry.** A job can pass every one of them
and still drive the arm through the plate or the workpiece. In particular
**nothing guards the nozzle** against your part: the tool's entire collision body
is the single TCP point, which IK pins to the commanded waypoint, so it is
deliberately excluded from filters 6–8. Filter 8 protects the *arm links* only.

## 8. Checklist

1. Write your study config; select it with `FR5_STUDY_CONFIG`.
2. Curves as faceless ASCII PLY (vertex + edge), surfaces as OBJ, all in mm.
3. Set `CURVED_MODEL_ROTATE_X_DEG` so the printable face points away from the plate.
4. Leave `CURVED_MODEL_XY_OFFSET_MM` at `(0, 0)`; re-measure before changing it.
5. Point `CURVED_OBSTACLE_FILE` at a body under your surfaces (normals only).
6. Set the plate pose **before** loading the model.
7. Load → Geodesics → Print Order → Orientation Frames → Run Precompute.
8. Read the reject breakdown if it fails; tune material constants in your config.
9. Export, and read the validation table rather than trusting ACCEPTED alone.
