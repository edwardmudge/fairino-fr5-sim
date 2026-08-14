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
# per-waypoint tangent-plane check (settled.md S1.37).
CURVED_OBSTACLE_FILE = "Surface_Bot.obj"
CURVED_OBSTACLE_STRUCTURE_NAME = "Surface Bot"
CURVED_OBSTACLE_COLOR = (0.55, 0.55, 0.55)

# The four below are assumptions, not measurements -- they depend on this
# study's material and nozzle, so they live here rather than in
# geometry_backend.py. Tune empirically.

# How far a travel move between two curve pieces is offset outward along the
# local surface normal, so the nozzle hovers over the mockup and any wet traces
# instead of scraping them. Used by build_print_order().
CURVED_TRAVEL_HOVER_MM = 4.0

# Nozzle-tip inward slack against a waypoint's surface tangent plane during the
# curved precompute clearance check. Assumed plausible contact depth; the 6 arm
# links get zero tolerance. Used by _branch_clears_ground().
CURVED_TIP_CLEARANCE_TOLERANCE_MM = 1.0

# The PLY toolpath curves carry no extrusion (E) data, and "layer height from Z"
# is meaningless on a conformal path -- this fixed cross-section stands in for
# both. Assumed plausible for an elastomer trace. Used by _build_curved_beads().
CURVED_BEAD_WIDTH_MM = 1.5
CURVED_BEAD_HEIGHT_MM = 0.5
