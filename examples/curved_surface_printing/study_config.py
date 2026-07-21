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

CURVED_OBSTACLE_FILE = "Surface_Bot.obj"  # optional non-print collision body
CURVED_OBSTACLE_STRUCTURE_NAME = "Surface Bot"
CURVED_OBSTACLE_COLOR = (0.55, 0.55, 0.55)
