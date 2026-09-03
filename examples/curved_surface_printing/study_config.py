"""Study-specific configuration for the curved-surface-printing feature in
geometry_backend.py/gui_panel.py -- see this folder's README.md.

The feature itself (load_curved_model(), the geodesic engine, the GUI layer
selector) is generic and project-agnostic: it operates on whatever CURVED_LAYERS
describes. This module is where one specific study -- printing an elastomeric
capacitive sensor conformally onto a shoulder mockup -- wires the generic
feature to its own assets. A different curved-print job would supply its own
version of this module and geometry_backend.py's one import would point at it
instead.
"""

CURVED_MODEL_DIR = "assets/models/curved"
CURVED_MODEL_ROTATE_X_DEG = 90.0  # CAD "+z up" assumption was wrong -- see settled.md S1.29

CURVED_LAYERS = [
    {
        "name": "RX",
        "curve_files": [f"RX_{i}.ply" for i in range(28)],
        "curve_structure_name": "Curved Toolpath RX",
        "curve_color": (0.85, 0.15, 0.15),
        "surface_file": "Surface_RX_Offset.obj",
        "surface_structure_name": "Surface RX Offset",
        "surface_color": (0.93, 0.80, 0.80),
    },
    {
        "name": "TX",
        "curve_files": [f"TX_{i}.ply" for i in range(27)],
        "curve_structure_name": "Curved Toolpath TX",
        "curve_color": (0.15, 0.35, 0.85),
        "surface_file": "Surface_TX_Base.obj",
        "surface_structure_name": "Surface TX Base",
        "surface_color": (0.80, 0.85, 0.93),
    },
]
# Print order defaults to list order -- RX first, per settled.md S1.32.

# Optional non-print body under the print surfaces. Used to orient surface
# normals outward (_orient_normals_outward) -- NOT a collision body: the
# obstacle-mesh clearance approach was rejected as too slow and replaced by the
# per-waypoint tangent-plane check (settled.md S1.37). Roadmap 7.2 then removed
# that check too, so the curved path has no clearance test at all -- this mesh
# is still normals-only, and now nothing else guards the workpiece (S1.44).
#
# Changed in Stage 7.4 (settled.md S1.46): the curved path DOES have a
# collision test again -- filter 8 checks the arm links against each layer's own
# PRINT surface (Surface_RX_Offset / Surface_TX_Base, via _build_surface_grid),
# which is the mesh-vs-mesh check S1.37 declined to build. This obstacle mesh is
# still normals-only and is still not an obstacle: it is Surface_Bot, under the
# print surfaces, and the arm has no reason to approach it that filter 8 does
# not already catch. The "nothing guards the workpiece" line above is therefore
# out of date for the ARM; it remains true for the nozzle, which has no body.
CURVED_OBSTACLE_FILE = "Surface_Bot.obj"
CURVED_OBSTACLE_STRUCTURE_NAME = "Surface Bot"
CURVED_OBSTACLE_COLOR = (0.55, 0.55, 0.55)

# The four below are assumptions, not measurements -- they depend on this
# study's material and nozzle, so they live here rather than in
# geometry_backend.py. Tune empirically. (Still four, as settled.md S1.41 says;
# one of them is legacy since 7.2 but is still a tuned material assumption.)

# How far a travel move between two curve pieces is offset outward along the
# local surface normal, so the nozzle hovers over the mockup and any wet traces
# instead of scraping them. Used by build_print_order().
CURVED_TRAVEL_HOVER_MM = 4.0

# LIVE AGAIN since roadmap 7.4 -- this is filter 8's surface-mesh clearance.
#
# History, because the round trip is the point: it began as the nozzle-tip
# inward slack against a waypoint's surface tangent plane (_nozzle_clears_plane,
# settled.md S1.37). Roadmap 7.2 removed that check -- 7.1 had already reduced
# the tool's collision body to the single TCP point, which IK pins to the very
# plane being tested, so it could no longer reject anything -- and marked this
# LEGACY, kept rather than deleted on the grounds that it is a tuned material
# value that a real surface-collision test would want back.
#
# 7.4 is that test. settled.md S1.46 directs preferring this 1.0mm over the
# reference guide's 2.0mm default where the two disagree, which they do.
#
# Note what it now guards and what it does not: filter 8 tests the ARM LINKS
# against the print surface, not the nozzle. The tool is still a single point
# pinned to the commanded waypoint, so including it would reject every printing
# pose -- the same trap that made the 7.2-era check inert.
CURVED_TIP_CLEARANCE_TOLERANCE_MM = 1.0

# The PLY toolpath curves carry no extrusion (E) data, and "layer height from Z"
# is meaningless on a conformal path -- this fixed cross-section stands in for
# both. Assumed plausible for an elastomer trace. Used by _build_curved_beads().
CURVED_BEAD_WIDTH_MM = 1.5
CURVED_BEAD_HEIGHT_MM = 0.5
