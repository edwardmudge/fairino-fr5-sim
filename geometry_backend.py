import os
import re
import json
import time
import heapq
import hashlib
import polyscope as ps
import numpy as np
import trimesh

# FR5 link meshes, zero-pose world frame (see docs/FR5_Mesh_Convention.md)
MESH_DIR = "assets/fr5_meshes"
MESH_FILES = [f"Robot{i}.obj" for i in range(7)]  # Robot0 (base) .. Robot6

# Tool head: nozzle mesh + TCP point, same zero-pose convention, mounted on the flange (Delta_6)
PRINTER_HEAD_DIR = "assets/printerHead"
NOZZLE_FILE = "nozzle.obj"
TCP_FILE = "TCP.txt"

TRAJECTORY_SAMPLE_INTERVAL_S = 0.1  # Minimum seconds between recorded TCP trajectory points
TRAJECTORY_RADIUS_MM = 2.0  # Trajectory curve line thickness, world units (mm)
TCP_FRAME_SCALE_MM = 50.0  # TCP coordinate-axes length, world units (mm)

PLAYBACK_RENDER_STRIDE = 50  # Push arm/bead updates to Polyscope every Nth
# solved waypoint, not every frame -- full-buffer re-uploads make coarser
# pushes cut real GPU cost, not just Python-side work.

TRAJECTORY_CURVE_RENDER_STRIDE = 5  # Re-register the "Trajectory" curve
# network every Nth recorded sample -- it has no incremental grow API, so
# this throttles how often the O(n) rebuild fires.

PLAYBACK_LOOKAHEAD_BEADS = 5000  # How far ahead of current progress the
# registered "G-code Print" mesh is grown, in beads -- render cost scales
# with registered mesh size, so this stays close to actual progress instead
# of registering the full mesh from frame 1.

BUILD_PLATE_DIR = "assets/buildPlate"
BUILD_PLATE_FILE = "BambuLab_BuildPlate.obj"
PLATE_COLOR = (0.75, 0.75, 0.78)  # Light cool gray, visually distinct from the orange print
# Measured thickness of BambuLab_BuildPlate.obj (its local Z span is [-0.75, 0],
# origin at the top corner) -- position_mm marks the resting/bottom face, so the
# top/print surface sits this far above it. See BuildPlate_UserFrame.md.
PLATE_THICKNESS_MM = 0.75

# Placed in the (-X, -Y) quadrant to match the arm's natural zero/home-pose
# reach direction -- the opposite quadrant only reaches via a near-limit J1
# rotation, leaving little margin for the wrist to also orient freely
USER_FRAME_ORIGIN_MM = np.array([-600.0, -300.0, 0.0])
USER_FRAME_SCALE_MM = 50.0  # Fixed axes drawn at the user frame, world units (mm)
BUILD_PLATE_POSITION_FILE = os.path.join(BUILD_PLATE_DIR, "saved_position.json")  # GUI Save/Load Position buttons

GCODE_DIR = "assets/models/planar/gcode"
GCODE_FILE = "model.gcode"  # Fixed name -- overwritten by each new Cura export, never hand-edited
GCODE_COLOR = (1.0, 0.55, 0.0)  # Orange, so it doesn't visually merge with the Trajectory curve
# Assumed, not parsed -- Cura's exported G-code carries no filament/nozzle-diameter
# comment to read instead. 1.75mm is the standard FDM default. Used to convert
# extruded filament length (E) into a deposited bead volume, see load_gcode().
FILAMENT_DIAMETER_MM = 1.75

# How straight (dot product of unit tangents) and width-matched (mm) two
# back-to-back bead segments must be for their shared cap faces to be treated
# as exactly coincident -- and thus safely dropped -- rather than meeting at a
# visible angle or ledge. See _build_gcode_beads, settled.md S1.19.
CAP_CULL_COLINEAR_DOT_MIN = 0.999  # ~2.6 degrees
CAP_CULL_WIDTH_TOL_MM = 0.01

GCODE_MOVE_RE = re.compile(r"([A-Za-z])\s*(-?\d+\.?\d*)")

# Float export noise keeps true duplicate vertices apart past ~3dp -- verified
# on RX_0.ply (108 raw verts -> exactly 54 nodes, matching the asset survey).
CURVE_DEDUPE_DECIMALS = 3

CURVE_RADIUS_MM = 0.5  # thin vs. TRAJECTORY_RADIUS_MM (2.0) -- 70 pieces shouldn't dominate the view

# Curved-surface printing (roadmap Stage 6) is a generic, project-agnostic
# feature: load an arbitrary set of toolpath-curve layers + their host
# surfaces, place them above the build plate, and route geodesics over each
# layer's own surface (settled.md S1.30 -- geodesics never cross layers).
# What's specific to one project -- which files, how many layers, their
# names/colors -- is imported from a study config rather than hardcoded
# here. See examples/curved_surface_printing/ to point this feature at a
# different curved-print job.
from examples.curved_surface_printing.study_config import (
    CURVED_MODEL_DIR, CURVED_MODEL_ROTATE_X_DEG, CURVED_LAYERS,
    CURVED_OBSTACLE_FILE, CURVED_OBSTACLE_STRUCTURE_NAME, CURVED_OBSTACLE_COLOR,
)

GEODESIC_CHUNK_SOURCES = 1  # whole Dijkstra sources solved per step() call.
# Measured per source: ~50ms on Surface_RX_Offset (30,284 verts), ~85ms on
# Surface_TX_Base (45,430 verts / 135,518 edges) -- so ~12-20fps while running
# and ~8.4-9.1s wall for the full 113-source job. One whole source is the
# chunk granularity because sub-source chunking would mean carrying a live
# heap plus partial dist/prev across frames -- real complexity for a job that
# finishes in seconds.

# Assumed, not measured -- the toolpath curves carry no clearance data. A travel
# move between two curve pieces follows the 6.2 geodesic offset this far outward
# along the local surface normal, so the nozzle hovers over the mockup and any
# wet traces instead of scraping them. ~3-5mm is a plausible non-contact hop for
# a soft elastomer bead; tune empirically. Used by build_print_order().
CURVED_TRAVEL_HOVER_MM = 4.0
# Assumed, not measured -- how deep the nozzle tip may sit *inward* of its own
# waypoint's surface tangent plane during a curved precompute clearance check.
# The tip prints on the surface (signed distance ~0, sometimes slightly negative
# after mesh discretisation), so it alone gets this inward slack; the 6 arm-link
# meshes get zero tolerance. ~1mm is a plausible nozzle-contact depth; tune
# empirically like the other assumed job constants. Used by _branch_clears_ground().
CURVED_TIP_CLEARANCE_TOLERANCE_MM = 1.0
# Assumed, not measured -- the curved-print PLY toolpath curves carry no
# extrusion (E) data, and "layer height from Z" is meaningless on a
# conformal path -- a fixed cross-section stands in for both, same spirit as
# FILAMENT_DIAMETER_MM. ~1.5mm is a plausible elastomer trace width for this
# nozzle; tune empirically. Used by _build_curved_beads().
CURVED_BEAD_WIDTH_MM = 1.5
# Assumed, not measured -- same reasoning as CURVED_BEAD_WIDTH_MM. ~0.5mm is
# a plausible single-pass bead height for a conformal elastomer trace. Used
# by _build_curved_beads().
CURVED_BEAD_HEIGHT_MM = 0.5
# Solid warm red -- the non-printing hops. Deliberately outside the ordered-feed
# gradient's purple->teal->yellow ramp below, so "printing" (gradient) vs
# "moving" (this flat colour) read apart at a glance. Used by build_print_order().
CURVED_TRAVEL_COLOR = (0.90, 0.20, 0.15)
# The ordered feed is drawn as a sequence gradient over the printed pieces (piece
# 1 -> piece N), so the print order itself is legible. Anchor RGB stops of a
# viridis-like ramp, interpolated by _sequence_colors(); dark start, bright end.
CURVED_ORDER_CMAP = np.array([
    [0.267, 0.005, 0.329],   # deep purple  -- first printed
    [0.128, 0.567, 0.551],   # teal
    [0.993, 0.906, 0.144],   # yellow       -- last printed
])
CURVED_ORDER_FEED_RADIUS_MM = 0.8  # slightly over CURVE_RADIUS_MM (0.5) so the overlay reads on the base curve

# Per-waypoint TCP orientation triads -- roadmap 6.4. Smaller than the
# 50mm TCP frame: hundreds are drawn along the shell, so they must read as
# local surface normals, not clutter. Only every ORIENT_FRAME_STRIDE-th
# waypoint gets a triad, keeping the overlay legible.
ORIENT_FRAME_SCALE_MM = 6.0
ORIENT_FRAME_STRIDE = 12
ORIENT_FRAME_COLORS = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])  # X red, Y green, Z blue

PRECOMPUTE_CHUNK_SIZE = 25  # waypoints solved per step() call -- keeps each
# per-frame batch well under a 60fps budget. Measured ~0.5ms/waypoint for
# solve_ik_tcp_matrix + the ground-clearance filter at benchy scale (see
# settled.md S1.13's verification), so this is roughly a 12ms slice per frame.

GCODE_PRECOMPUTE_CACHE = os.path.join(GCODE_DIR, "model.precompute.npz")  # roadmap 5.10, settled.md S1.21
PRECOMPUTE_CACHE_VERSION = 3  # Bump to invalidate all existing caches on a schema change (2: per-waypoint R_target, roadmap 6.5; 3: reject_below_ground in key, roadmap 6.6)


def curved_precompute_cache_path(layer_name):
    """Per-layer precompute cache file for the curved passes -- roadmap 6.5.
    One file per print layer (RX, TX) so the planar benchy and each curved
    pass keep independent caches instead of thrashing a single fixed file."""
    return os.path.join(CURVED_MODEL_DIR, f"curved_{layer_name.lower()}.precompute.npz")


class VisContent:
    """
    [Backend Logic Layer]
    Responsibilities:
    1. Maintain geometry data (Mesh, Point Cloud)
    2. Execute geometry algorithms (Registration, Optimisation)
    3. Call Polyscope to register data (ps.register_...)
    """
    def __init__(self):
        # State data
        self.mesh_data = None
        self.current_joint_angles = None

        self.tcp_world = None            # Current TCP world position, set each apply_delta_transform call
        self.trajectory_points = []      # Recorded TCP world positions (see record_trajectory_point)
        self._last_sample_time = time.time()
        self._trajectory_curve_sample_count = 0  # Throttles _update_trajectory_curve() re-registration, see TRAJECTORY_CURVE_RENDER_STRIDE
        self.trajectory_enabled = True
        self.trajectory_handle = None    # Set once a curve exists, see _update_trajectory_curve

        # Chunked toolpath IK precompute state, see run_toolpath_ik_precompute()
        self.precompute_running = False
        self.precompute_index = 0
        self.precompute_total = 0
        self.precompute_waypoints = None
        self.precompute_R_target = None
        self.precompute_joint_limits = None
        self.precompute_ref = None
        self.precompute_joint_path = []
        self.precompute_status = ""
        self.precompute_cache_meta = None  # Cache key captured at precompute-start, see run_toolpath_ik_precompute()
        self.precompute_cache_path = None  # Which cache file this precompute writes to (per-layer for curved, roadmap 6.5)
        self.precompute_tip_tolerance_mm = None  # Non-None -> curved run; enables per-waypoint tangent-plane clearance
        self.toolpath_source = -1  # -1 = planar G-code; 0..len(CURVED_LAYERS)-1 = that curved layer.
                                    # Single source of truth for what the shared Run/Pause/Cancel/Reset
                                    # precompute+playback controls currently target -- roadmap 6.6.
        self.reject_below_ground = True  # Toggle: reject IK branches whose moving geometry dips
                                         # below world z=0. Default ON (planar's historical behaviour);
                                         # applies to BOTH paths -- roadmap 6.6. Unchecked for a
                                         # low-plate/mockup setup where sub-z=0 poses are physically
                                         # fine. Folded into the precompute cache key (it changes which
                                         # branch is accepted, so the solved path depends on it).

        # Progressive-reveal playback state -- playback_index persists across
        # pause, only reset_toolpath_playback() zeroes it.
        self.playback_running = False
        self.playback_active = False  # True from when a run actually starts until Reset. Distinct
        # from playback_running (which flips off on Pause) so the guide overlays stay hidden through a
        # pause and only reset_toolpath_playback() restores them -- roadmap 6.7.
        self.playback_index = 0
        self._last_rendered_playback_index = 0  # Throttles the Polyscope push in advance_toolpath_playback, see PLAYBACK_RENDER_STRIDE
        self.playback_status = ""
        self.playback_waiting = False  # True when caught up to precompute's frontier but it isn't exhausted yet, see advance_toolpath_playback()
        self.gcode_bead_verts_full = None       # (K*8,3) world space, real bead positions
        self.gcode_bead_faces = None
        self.gcode_bead_reveal_index = None     # (K,) sorted ascending, see _build_gcode_beads
        self.gcode_bead_face_prefix = None      # (K+1,) cumulative triangle count, see _build_gcode_beads
        self.gcode_bead_verts_current = None    # (K*8,3) working copy, mutated as beads reveal
        self.gcode_print_handle = None          # Polyscope handle, reused across advance() calls
        self.gcode_preview_loaded = False       # True only while the static preview (not playback) owns "G-code Print"
        self._registered_bead_capacity = 0      # How many beads are actually registered
        # with Polyscope right now, see PLAYBACK_LOOKAHEAD_BEADS

        self.curved_model_loaded = False  # True once load_curved_model() has registered its structures -- roadmap 6.1/6.6

        # Retained curved-model geometry, world coordinates (already through
        # T_curved) -- roadmap 6.2 needs the per-piece curves and the print
        # surfaces that 6.1 previously computed and threw away. All lists are
        # indexed positionally by CURVED_LAYERS (examples/curved_surface_printing/
        # study_config.py). The obstacle mesh is deliberately absent from
        # these: it's a 6.5 collision body, not a print surface.
        self.curved_pieces_world = None        # list of len(CURVED_LAYERS) lists of (Ni,3) polylines
        self.curved_surface_verts_world = None # list of len(CURVED_LAYERS) (V,3)
        self.curved_surface_vnormals_world = None  # list of len(CURVED_LAYERS) (V,3) outward unit normals -- 6.3 hover, 6.4 orientation
        self.curved_surface_faces = None       # list of len(CURVED_LAYERS) (F,3), placement-invariant
        self.curved_layer_names = None         # list of len(CURVED_LAYERS) display names, e.g. ["RX", "TX"]
        self.T_curved = None                   # (4,4) placement actually used, for the staleness check
        self._T_user_frame_at_curved_load = None  # Plate pose the world state above was built against

        # Chunked geodesic precompute state, see run_geodesic_precompute()
        self.geodesic_running = False
        self.geodesic_index = 0
        self.geodesic_total = 0
        self.geodesic_status = ""
        self.geodesic_loaded = False       # True only once both cost matrices are complete -- what 6.3 gates on
        self.geodesic_graphs = None        # list of 2 CSR triples; None/not-None is the start-fresh/resume sentinel
        self.geodesic_snap_nodes = None    # list of 2 (70,) endpoint -> vertex id on its own surface
        self.geodesic_snap_dist = None     # list of 2 (70,) snap distances, evidence the snap is legitimate
        self.geodesic_sources = None       # list of 2 (S,) unique snapped vertex ids
        self.geodesic_source_row = None    # list of 2 (70,) endpoint -> row in sources/prev
        self.geodesic_queue = None         # list of (layer, row) tuples, the flat work list
        self.geodesic_prev = None          # list of 2 (S,V) int32 predecessor rows -- makes any path a walk-back
        self.geodesic_cost = None          # list of 2 (70,70) float64, inf = unreachable
        self.geodesic_unreachable = None   # list of 2 ints, count of inf entries per matrix

        # Print-order + travel moves, see build_print_order() -- roadmap 6.3.
        # All per-layer, derived from the geodesic cost/prev above, so they are
        # cleared alongside them in _abort_geodesic_precompute().
        self.curved_order_loaded = False       # True once build_print_order() has run -- what 6.5 gates on
        self.curved_print_order = None         # list of len(CURVED_LAYERS) lists of (piece, entry_end)
        self.curved_travel_moves = None        # list of len(CURVED_LAYERS) lists of (M,3) hover polylines
        self.curved_travel_total = None        # list of len(CURVED_LAYERS) optimized inter-piece travel (mm)
        self.curved_travel_naive = None        # list of len(CURVED_LAYERS) file-order travel (mm), the baseline
        self.curved_order_status = ""

        # Per-waypoint TCP orientation frames, see build_orientation_frames()
        # -- roadmap 6.4. Derived from the print order above, so cleared with
        # it in _abort_geodesic_precompute().
        self.curved_orient_loaded = False      # True once build_orientation_frames() has run -- what 6.5 gates on
        self.curved_orient_frames = None       # list of len(CURVED_LAYERS) lists of (pos_world (3,), R_target (3,3)), print order
        self.curved_orient_status = ""

        # Per-layer printed-bead playback state, see _build_curved_beads() /
        # _init_curved_toolpath_playback() -- roadmap 6.6. Mirrors the flat
        # gcode_bead_* fields above, but indexed per layer (lazily sized to
        # len(CURVED_LAYERS) on first use) so RX's and TX's printed meshes can
        # coexist -- the S1.32 stack rule requires TX's view to keep showing
        # RX's already-printed layer beneath it, not just whichever was last
        # built. Cleared only by clear_curved_model() or a re-order/re-orient
        # cascade (_abort_geodesic_precompute()) -- never by a generic
        # precompute abort/cancel, so switching the active toolpath source
        # can't make a completed layer's printed mesh disappear.
        self.curved_bead_verts_full = None
        self.curved_bead_faces = None
        self.curved_bead_reveal_index = None
        self.curved_bead_face_prefix = None
        self.curved_bead_verts_current = None
        self.curved_print_handle = None
        self.curved_bead_registered_capacity = None

        # Initialise the scene
        self.create_coordinate_frame()
        self.load_build_plate()
        self.mesh_data = self.load_data()
        self.update_arm([0, 0, 0, 0, 0, 0])


    def create_coordinate_frame(self, scale=1.0, origin=(0, 0, 0), rotation=None, name="Coordinate Frame"):
        """Register an XYZ axis triad. Reused for the static world-origin
        frame (defaults) and, with an origin/scale/name override, for the
        TCP frame -- see load_data() and docs/FR5_Mesh_Convention.md. An
        optional rotation (3x3) tilts the triad's axes -- used by
        load_build_plate() so the "User Frame" triad matches the plate's
        orientation; defaults to identity (axis-aligned) for every other
        caller. Returns (handle, nodes) so callers can drive the nodes
        through the Delta transform like any other zero-pose-frame
        geometry."""
        origin = np.asarray(origin, dtype=float)
        R = np.eye(3) if rotation is None else rotation
        nodes = np.array([origin, origin + R @ [scale,0,0], origin + R @ [0,scale,0], origin + R @ [0,0,scale]])
        edges = np.array([[0,1], [0,2], [0,3]])

        ps_net = ps.register_curve_network(name, nodes, edges)

        # X=red, Y=green, Z=blue
        colors = np.array([[1,0,0], [0,1,0], [0,0,1]])
        ps_net.add_color_quantity("axis_colors", colors, defined_on='edges', enabled=True)

        return ps_net, nodes


    def load_build_plate(self, position_mm=USER_FRAME_ORIGIN_MM, rpy_deg=(0.0, 0.0, 0.0)):
        """Place the build plate at the user frame -- see GLOSSARY.md 'User
        frame' and settled.md S1.2/S1.6. Static geometry: registered once
        per call, never updated per-frame -- unlike the arm links, the
        plate isn't driven by any joint, so it needs no Delta transform,
        just a homogeneous transform applied here. Defaults reproduce the
        original translation-only placement exactly. rpy_deg is [roll,
        pitch, yaw] degrees, XYZ fixed-angle convention (R = Rz(yaw) @
        Ry(pitch) @ Rx(roll)) -- same convention as solve_ik_tcp. Safe to
        call repeatedly (e.g. from the GUI's Move/Reset buttons); Polyscope
        replaces the prior structures of the same names. position_mm marks
        the plate's resting/bottom face -- the raw mesh's local origin sits
        at its top face instead, so local vertices are shifted up by
        PLATE_THICKNESS_MM before the transform to compensate.

        Also invalidates any in-memory toolpath precompute/playback solved
        against the plate's previous pose -- roadmap Stage5_README.md 5.11,
        settled.md S1.22. The cross-session disk cache (5.10/S1.21) already
        keys on pose, but that's only checked once, at the start of a fresh
        precompute; without this, resuming/replaying an already-loaded
        precompute after a plate move would silently drive the arm through
        the old pose's joint path."""
        roll, pitch, yaw = np.deg2rad(rpy_deg)
        R = rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)

        self.T_user_frame = np.eye(4)
        self.T_user_frame[:3, :3] = R
        self.T_user_frame[:3, 3] = position_mm

        if self.precompute_cache_meta is not None:
            new_pose = np.round(self.T_user_frame, 6).tolist()
            if new_pose != self.precompute_cache_meta["user_frame"]:
                self.cancel_toolpath_ik_precompute()
                self.precompute_status = "Build plate moved -- precompute invalidated, run again"

        if self.T_curved is not None and not np.allclose(self.T_user_frame, self._T_user_frame_at_curved_load):
            # The geodesic *costs* survive a plate move -- distances in mm are
            # rigid-motion invariant. The retained world vertices and the
            # stored paths do not, and 6.3 consumes the paths, not just the
            # numbers, so invalidating is the honest call.
            self._abort_geodesic_precompute()
            self.geodesic_status = "Build plate moved -- geodesics invalidated, reload the curved model"

        plate = self.load_mesh(os.path.join(BUILD_PLATE_DIR, BUILD_PLATE_FILE))
        plate_verts_local = plate.vertices + np.array([0.0, 0.0, PLATE_THICKNESS_MM])
        plate_verts_world = transform_points(self.T_user_frame, plate_verts_local)
        plate_handle = ps.register_surface_mesh("Build Plate", plate_verts_world, plate.faces)
        plate_handle.set_color(PLATE_COLOR)

        self.create_coordinate_frame(scale=USER_FRAME_SCALE_MM, origin=position_mm, rotation=R, name="User Frame")


    def save_build_plate_position(self, position_mm, rpy_deg):
        """Write the given build-plate pose to assets/buildPlate/ so it can
        be recalled later via load_saved_build_plate_position() -- see the
        GUI's "Save Position" button. Only ever called on explicit user
        action, never automatically. Returns a status message; a write
        failure (missing directory, permissions) is reported back rather
        than left to raise out of the button callback, since silently
        claiming success here would be worse than the crash it avoids."""
        data = {
            "position_mm": np.asarray(position_mm, dtype=float).tolist(),
            "rpy_deg": np.asarray(rpy_deg, dtype=float).tolist(),
        }
        try:
            with open(BUILD_PLATE_POSITION_FILE, "w") as f:
                json.dump(data, f, indent=2)
        except OSError as e:
            return f"Failed to save position: {e}"
        return "Position saved"


    def load_saved_build_plate_position(self):
        """Read a previously saved build-plate pose (if any) and apply it
        immediately via load_build_plate(). Only ever called on explicit
        user action (the GUI's "Load Saved Position" button), never
        automatically at startup -- see settled.md S1.6. Returns
        (position_mm, rpy_deg, status_message); position_mm/rpy_deg are
        None on failure so the GUI knows not to update its input fields."""
        if not os.path.exists(BUILD_PLATE_POSITION_FILE):
            return None, None, "No saved position found"

        with open(BUILD_PLATE_POSITION_FILE) as f:
            data = json.load(f)

        position_mm = np.array(data["position_mm"], dtype=float)
        rpy_deg = np.array(data["rpy_deg"], dtype=float)
        self.load_build_plate(position_mm, rpy_deg)
        return position_mm, rpy_deg, "Loaded saved position"


    def parse_gcode(self, filepath):
        """Parse G0/G1 linear moves. Returns a list of ([x, y, z], e,
        is_feed_move) triples, plate-local mm -- G0 travel moves update
        position but are flagged is_feed_move=False so load_gcode() skips
        drawing them. Position and extrusion (E) are both modal: a G0
        travel line carries no E and simply leaves it unchanged. Modal
        position handling: see GLOSSARY.md 'G-code toolpath'. G0/G1-only by
        design (see settled.md S1.7) -- any other G/M-code, and any line
        that isn't G0/G1, is discarded in software here, not assumed
        absent from the input file."""
        x, y, z, e = 0.0, 0.0, 0.0, 0.0
        points = []
        with open(filepath) as f:
            for line in f:
                line = line.split(';', 1)[0]
                line = re.sub(r"\([^)]*\)", "", line)
                words = GCODE_MOVE_RE.findall(line)
                if not words:
                    continue

                letter0, value0 = words[0]
                if letter0.upper() != 'G':
                    continue
                try:
                    code = int(float(value0))
                except ValueError:
                    continue
                if code not in (0, 1):
                    continue

                for letter, value in words[1:]:
                    letter = letter.upper()
                    if letter == 'X':
                        x = float(value)
                    elif letter == 'Y':
                        y = float(value)
                    elif letter == 'Z':
                        z = float(value)
                    elif letter == 'E':
                        e = float(value)
                points.append(([x, y, z], e, code == 1))
        return points


    # One bead box's 12 triangles: corners 0-3 = bottom, 4-7 = top above them.
    # Reused per segment via a vertex-index offset, see load_gcode().
    _BEAD_BOX_FACE_TEMPLATE = np.array([
        [0, 1, 2], [0, 2, 3],       # bottom
        [4, 6, 5], [4, 7, 6],       # top (reversed winding vs. bottom)
        [0, 1, 5], [0, 5, 4],       # side 0-1
        [1, 2, 6], [1, 6, 5],       # side 1-2
        [2, 3, 7], [2, 7, 6],       # side 2-3
        [3, 0, 4], [3, 4, 7],       # side 3-0
    ])

    def _build_gcode_beads(self, gcode_points):
        """Compute the swept bead-mesh geometry for a parsed G-code point
        list -- the shared math behind both the always-visible static
        preview (load_gcode()) and the progressive-reveal playback mesh
        (reset_/advance_toolpath_playback(), roadmap Stage5_README.md 5.7).
        Bead height tracks the actual printed Z per layer rather than
        slicer metadata, since real files include non-extruding Z
        excursions (e.g. a startup clearance lift) that would corrupt a
        naive Z-change tracker. Bead width comes from extruded E assuming
        FILAMENT_DIAMETER_MM. Vectorised across all segments (no
        per-segment loop) for the ~180,000-segment scale of a real print
        (settled.md S1.9).

        Returns (verts_world, faces, reveal_waypoint_index, bead_face_prefix):
          verts_world: (K*8, 3) float, world-space bead-box corners,
            bead-major (8 contiguous rows per bead) -- matches
            _BEAD_BOX_FACE_TEMPLATE's indexing.
          faces: (<=K*12, 3) int, triangle indices into verts_world -- fewer
            than 12 per bead wherever settled.md S1.19's cap culling drops
            a hidden face, so no longer a fixed per-bead stride.
          reveal_waypoint_index: (K,) int, strictly increasing -- the
            0-based index into gcode_points at which bead k's segment ends
            (segment i connects point i -> point i+1, so it's revealed
            once playback reaches point i+1).
          bead_face_prefix: (K+1,) int, cumulative triangle count --
            `faces[:bead_face_prefix[n]]` is exactly the triangles for the
            first n beads, needed because `faces` no longer has a fixed
            per-bead stride (settled.md S1.20, right-sized playback
            registration). All four arrays are empty/trivial (K == 0) if
            there are no printed beads.
        """
        pts = np.array([p for p, _, _ in gcode_points])
        es = np.array([e for _, e, _ in gcode_points])
        is_feed = np.array([f for _, _, f in gcode_points])

        p0, p1 = pts[:-1], pts[1:]
        seg_vec = p1 - p0
        seg_len = np.linalg.norm(seg_vec, axis=1)
        delta_e = es[1:] - es[:-1]
        z_dest = p1[:, 2]

        # Only extruding segments advance the layer floor (settled.md S1.9) --
        # skips non-extruding Z excursions like a startup clearance lift.
        seg_is_print = is_feed[1:] & (delta_e > 1e-9)
        bead_bottom = np.empty(len(z_dest))
        prev_print_z = None
        cur_bottom = 0.0
        for i in range(len(z_dest)):
            if seg_is_print[i]:
                zi = z_dest[i]
                if prev_print_z is None or zi > prev_print_z:
                    if prev_print_z is not None:
                        cur_bottom = prev_print_z
                elif zi < prev_print_z:
                    cur_bottom = 0.0  # unexpected downward jump; rest on the plate
                prev_print_z = zi
            bead_bottom[i] = cur_bottom
        bead_top = z_dest
        bead_height = bead_top - bead_bottom

        safe_len = np.where(seg_len > 1e-9, seg_len, 1.0)
        u = seg_vec / safe_len[:, None]
        w_axis = np.cross(u, [0.0, 0.0, 1.0])  # horizontal, perpendicular to travel
        w_norm = np.linalg.norm(w_axis, axis=1)
        safe_w_norm = np.where(w_norm > 1e-9, w_norm, 1.0)
        w_axis = w_axis / safe_w_norm[:, None]

        filament_area = np.pi * (FILAMENT_DIAMETER_MM / 2.0) ** 2
        safe_denom = np.where((seg_len * bead_height) > 1e-9, seg_len * bead_height, 1.0)
        width = (delta_e * filament_area) / safe_denom

        valid = seg_is_print & (seg_len > 1e-6) & (w_norm > 1e-6) & (bead_height > 1e-9)
        if not np.any(valid):
            return (np.empty((0, 3)), np.empty((0, 3), dtype=int), np.empty(0, dtype=int),
                    np.zeros(1, dtype=int))

        # Capture segment indices before the `valid` filter overwrites p0/p1 --
        # segment i connects gcode_points[i] -> gcode_points[i+1], revealed once
        # playback reaches point i+1.
        reveal_waypoint_index = np.nonzero(valid)[0] + 1

        p0, p1 = p0[valid], p1[valid]
        w_axis = w_axis[valid]
        half_w = (width[valid] / 2.0)[:, None]
        bottom = (bead_bottom[valid] + PLATE_THICKNESS_MM)[:, None]
        top = (bead_top[valid] + PLATE_THICKNESS_MM)[:, None]

        offset = w_axis[:, :2] * half_w
        c0 = p0[:, :2] + offset
        c1 = p0[:, :2] - offset
        c2 = p1[:, :2] - offset
        c3 = p1[:, :2] + offset

        K = len(p0)
        verts_local = np.zeros((K, 8, 3))
        for idx, corner_xy in enumerate((c0, c1, c2, c3)):
            verts_local[:, idx, :2] = corner_xy
            verts_local[:, idx, 2] = bottom[:, 0]
            verts_local[:, idx + 4, :2] = corner_xy
            verts_local[:, idx + 4, 2] = top[:, 0]
        verts_local = verts_local.reshape(-1, 3)

        # Drop cap faces at bead-to-bead boundaries that are provably always
        # hidden: back-to-back in the G-code (no travel gap), colinear (same
        # travel direction -- at a turn the two cap planes meet at an angle,
        # not coincide), and width-matched (same cross-section, so no ledge
        # is exposed). ~8% of triangles on a real multi-layer print, since
        # most consecutive segments trace a curved surface and fail the
        # colinearity test -- settled.md S1.19.
        u_valid, width_valid = u[valid], width[valid]
        chained = np.diff(reveal_waypoint_index) == 1
        colinear = np.sum(u_valid[:-1] * u_valid[1:], axis=1) >= CAP_CULL_COLINEAR_DOT_MIN
        width_matched = np.abs(width_valid[:-1] - width_valid[1:]) <= CAP_CULL_WIDTH_TOL_MM
        cullable = chained & colinear & width_matched

        drop_end_cap = np.zeros(K, dtype=bool)    # bead k's end cap (template rows 8-9)
        drop_start_cap = np.zeros(K, dtype=bool)  # bead k's start cap (template rows 4-5)
        drop_end_cap[:-1] = cullable
        drop_start_cap[1:] = cullable

        keep_row = np.ones((K, 12), dtype=bool)
        keep_row[drop_end_cap, 8] = False
        keep_row[drop_end_cap, 9] = False
        keep_row[drop_start_cap, 4] = False
        keep_row[drop_start_cap, 5] = False

        faces_full = (self._BEAD_BOX_FACE_TEMPLATE[None, :, :] + (np.arange(K) * 8)[:, None, None])
        faces = faces_full[keep_row]
        bead_face_prefix = np.concatenate([[0], np.cumsum(keep_row.sum(axis=1))])

        verts_world = transform_points(self.T_user_frame, verts_local)

        return verts_world, faces, reveal_waypoint_index, bead_face_prefix


    def load_gcode(self):
        """Register the deposited G1 material as a swept bead mesh on the
        plate -- solid boxes, not a curve, so it reads as the printed
        object (settled.md S1.9). No-ops if the G-code file is missing;
        safe to call repeatedly, e.g. on plate reposition (settled.md
        S1.8). Geometry itself comes from _build_gcode_beads()."""
        filepath = os.path.join(GCODE_DIR, GCODE_FILE)
        if not os.path.exists(filepath):
            return

        try:
            waypoints = self.parse_gcode(filepath)
        except OSError:
            # File can be overwritten mid-read by a Cura re-export between
            # the exists() check above and here -- no-op like the missing-
            # file case rather than crashing the per-frame callback.
            return
        if len(waypoints) < 2:
            return

        verts_world, faces, _reveal_waypoint_index, _bead_face_prefix = self._build_gcode_beads(waypoints)
        if len(verts_world) == 0:
            return

        self.gcode_print_handle = ps.register_surface_mesh("G-code Print", verts_world, faces)
        self.gcode_print_handle.set_color(GCODE_COLOR)
        self.gcode_preview_loaded = True


    def clear_gcode_preview(self):
        """Mirrors the GUI's toggled "Clear G-code preview" button: removes
        the "G-code Print" mesh and resets playback state, since the same
        structure is reused for the playback reveal animation -- leaving
        playback_index pointed at now-discarded bead arrays would break the
        next Run Toolpath. Safe to call with nothing loaded."""
        self._reset_toolpath_playback_state()


    def _register_curve_layer(self, name, pieces_local, T_placement, color):
        """Combine one layer's reconstructed pieces (open + closed) into a
        single Polyscope curve network -- one structure per layer, not per
        piece, so a future layer toggle (roadmap 6.6) can show/hide a whole
        pass at once."""
        nodes_local, edge_blocks, offset = [], [], 0
        for piece in pieces_local:
            n = len(piece)
            nodes_local.append(piece)
            edge_blocks.append(np.column_stack([np.arange(n - 1), np.arange(1, n)]) + offset)
            offset += n
        nodes_world = transform_points(T_placement, np.vstack(nodes_local))
        handle = ps.register_curve_network(name, nodes_world, np.vstack(edge_blocks))
        handle.set_color(color)
        handle.set_radius(CURVE_RADIUS_MM, relative=False)
        return handle


    def load_curved_model(self):
        """Load the toolpath-curve PLY files and surface OBJ meshes described
        by CURVED_LAYERS (plus the optional CURVED_OBSTACLE_FILE collision
        body) and place them above the build plate -- roadmap
        Stage6_README.md 6.1. Generic over however many layers the study
        config describes -- see examples/curved_surface_printing/. Static
        workpiece geometry, same as load_build_plate()/load_gcode():
        one-time T_user_frame multiply, no Delta transform (settled.md
        S1.2/S1.3). Safe to call repeatedly; Polyscope replaces the prior
        structures of the same names.

        Placement is translation plus one fixed rotation (Stage6_README.md's
        Open Questions: the CAD "+z up" assumption was unverified and turned
        out wrong -- CURVED_MODEL_ROTATE_X_DEG about local X puts the
        printable ridge surface face-up, confirmed by which side the
        surface faced when tested at +90 vs -90): rotate the raw-local
        points about the CAD-local origin, then center the rotated
        assembly's XY bbox over the plate mesh's own local XY bbox-center
        and lift so its lowest point sits at the plate-local print surface
        -- z=0 after the same PLATE_THICKNESS_MM compensation
        load_build_plate()/build_toolpath_waypoints_world() already apply
        (position_mm marks the plate's resting/bottom face, not its top).

        Retains the placed geometry in world coordinates
        (curved_pieces_world/curved_surface_verts_world/curved_surface_faces/
        T_curved) for roadmap 6.2's geodesic routing, which needs the
        per-piece curves and each layer's print surface in the frame the arm
        works in. The obstacle mesh is rendered but not retained -- it's a
        collision body for 6.5, not a print surface."""
        # Every world vertex below is about to be re-derived, so any geodesic
        # solved against the previous load -- in flight or complete -- describes
        # geometry that no longer exists.
        self._abort_geodesic_precompute()
        self.geodesic_status = ""

        layers_local = [
            [p for f in layer["curve_files"]
             for p in reconstruct_polylines(*read_ply_polyline(os.path.join(CURVED_MODEL_DIR, f)))]
            for layer in CURVED_LAYERS
        ]
        surfaces = [self.load_mesh(os.path.join(CURVED_MODEL_DIR, layer["surface_file"]))
                    for layer in CURVED_LAYERS]
        obstacle = (self.load_mesh(os.path.join(CURVED_MODEL_DIR, CURVED_OBSTACLE_FILE))
                    if CURVED_OBSTACLE_FILE else None)

        R = rot_x(np.deg2rad(CURVED_MODEL_ROTATE_X_DEG))[:3, :3]

        def rotate(pts):
            return pts @ R.T

        layers_local = [[rotate(p) for p in pieces] for pieces in layers_local]
        surface_verts_local = [rotate(s.vertices) for s in surfaces]
        obstacle_verts_local = rotate(obstacle.vertices) if obstacle is not None else None

        all_local = np.vstack(
            [p for pieces in layers_local for p in pieces] + surface_verts_local
            + ([obstacle_verts_local] if obstacle_verts_local is not None else [])
        )
        assembly_min, assembly_max = all_local.min(axis=0), all_local.max(axis=0)

        plate = self.load_mesh(os.path.join(BUILD_PLATE_DIR, BUILD_PLATE_FILE))
        plate_min, plate_max = plate.bounds

        T_placement = np.eye(4)
        T_placement[:2, 3] = (plate_min[:2] + plate_max[:2]) / 2.0 - (assembly_min[:2] + assembly_max[:2]) / 2.0
        T_placement[2, 3] = -assembly_min[2] + PLATE_THICKNESS_MM
        T_curved = self.T_user_frame @ T_placement

        # Transform once, then both retain and render the same arrays -- 6.2
        # routes over these surfaces and needs them in the frame the arm works
        # in, so world is computed here rather than at each registration site.
        layers_world = [[transform_points(T_curved, p) for p in pieces] for pieces in layers_local]
        surface_verts_world = [transform_points(T_curved, v) for v in surface_verts_local]

        for layer, pieces_world in zip(CURVED_LAYERS, layers_world):
            self._register_curve_layer(layer["curve_structure_name"], pieces_world,
                                        np.eye(4), layer["curve_color"])

        for layer, verts_world, mesh in zip(CURVED_LAYERS, surface_verts_world, surfaces):
            handle = ps.register_surface_mesh(layer["surface_structure_name"], verts_world, mesh.faces)
            handle.set_color(layer["surface_color"])

        obstacle_verts_world_or_none = None
        if obstacle is not None:
            obstacle_verts_world_or_none = transform_points(T_curved, obstacle_verts_local)
            handle = ps.register_surface_mesh(CURVED_OBSTACLE_STRUCTURE_NAME, obstacle_verts_world_or_none, obstacle.faces)
            handle.set_color(CURVED_OBSTACLE_COLOR)

        # Per-vertex outward normals, retained for 6.3's travel-move hover offset
        # (and reused by 6.4's per-waypoint orientation). Computed from the world
        # verts directly (rot_x + translation is rigid and orientation-preserving,
        # so no separate direction transform is needed). Outward = away from
        # Surface_Bot: the obstacle mesh is in scope here, so orient against it now
        # and bake the sign into the retained array rather than retaining Bot (a
        # 6.5 concern). Getting the sign wrong drives the nozzle into the mockup
        # (asset survey, 6.4 notes).
        surface_vnormals_world = []
        for verts_world, mesh in zip(surface_verts_world, surfaces):
            n = compute_vertex_normals(verts_world, np.asarray(mesh.faces))
            surface_vnormals_world.append(self._orient_normals_outward(verts_world, n, obstacle_verts_world_or_none))
        self.curved_surface_vnormals_world = surface_vnormals_world

        self.curved_pieces_world = layers_world
        self.curved_surface_verts_world = surface_verts_world
        self.curved_surface_faces = [np.asarray(m.faces) for m in surfaces]
        self.curved_layer_names = [layer["name"] for layer in CURVED_LAYERS]
        self.T_curved = T_curved
        self._T_user_frame_at_curved_load = self.T_user_frame.copy()
        self.curved_model_loaded = True


    def clear_curved_model(self):
        """Load/Clear pair for the curved model -- same idiom as
        clear_gcode_preview() (roadmap 6.6). Force-cancels an in-flight
        curved precompute first (its waypoints reference geometry about to
        be deleted), then removes every structure load_curved_model() and
        everything derived from it registered, and resets all curved_*
        state back to pre-load values. Safe to call with nothing loaded."""
        if self.precompute_waypoints is not None and self.precompute_cache_path not in (None, GCODE_PRECOMPUTE_CACHE):
            self._abort_toolpath_ik_precompute()
            self.precompute_status = "Curved model cleared -- precompute cancelled"

        # Cascades order/orient/bead state and their registered structures --
        # see _abort_geodesic_precompute()'s roadmap-6.6 extension.
        self._abort_geodesic_precompute()
        self.geodesic_status = ""

        if self.curved_layer_names is not None:
            for cfg in CURVED_LAYERS:
                ps.remove_curve_network(cfg["curve_structure_name"], error_if_absent=False)
                ps.remove_surface_mesh(cfg["surface_structure_name"], error_if_absent=False)
            if CURVED_OBSTACLE_FILE:
                ps.remove_surface_mesh(CURVED_OBSTACLE_STRUCTURE_NAME, error_if_absent=False)

        self.curved_pieces_world = None
        self.curved_surface_verts_world = None
        self.curved_surface_vnormals_world = None
        self.curved_surface_faces = None
        self.curved_layer_names = None
        self.T_curved = None
        self._T_user_frame_at_curved_load = None
        self.curved_model_loaded = False
        self.toolpath_source = -1


    def curved_model_summary(self):
        """Human-readable property lines for the loaded curved model -- the
        GUI's 'Curved Model Properties' dropdown (roadmap 6.6). Backend-owned
        so the panel just renders the strings. Returns [] if nothing is
        loaded; per-layer travel figures only appear once a print order
        exists (curved_travel_total is populated by build_print_order)."""
        if not self.curved_model_loaded:
            return []
        lines = [
            f"Source: {CURVED_MODEL_DIR}",
            f"Layers: {len(self.curved_layer_names)}",
        ]
        for i, name in enumerate(self.curved_layer_names):
            pieces = len(self.curved_pieces_world[i])
            verts = len(self.curved_surface_verts_world[i])
            faces = len(self.curved_surface_faces[i])
            lines.append(f"  {name}: {pieces} pieces, {verts} verts, {faces} faces")
            if self.curved_travel_total is not None:
                lines.append(f"     travel {self.curved_travel_total[i]:.0f} mm "
                             f"(file-order {self.curved_travel_naive[i]:.0f} mm)")
        built = lambda flag: "built" if flag else "not built"
        lines.append(f"Geodesics: {built(self.geodesic_loaded)}")
        lines.append(f"Print order: {built(self.curved_order_loaded)}")
        lines.append(f"Orientation frames: {built(self.curved_orient_loaded)}")
        return lines


    def _orient_normals_outward(self, verts, normals, obstacle_verts):
        """Flip `normals` as a whole so they point outward -- away from the
        Surface_Bot obstacle if one is configured, else away from the surface's
        own centroid (each print surface is a convex-ish dome cap, so the two
        agree on the shipped assets). Outward is one global sign, decided by a
        majority vote over a vertex sample rather than per-vertex, since the
        trimesh winding is already internally consistent -- only the whole-array
        sense can be wrong. A wrong sign would drive 6.3's hover offset (and
        6.4's nozzle) into the mockup instead of away from it."""
        n = len(verts)
        sample = np.arange(n) if n <= 2000 else np.linspace(0, n - 1, 2000).astype(int)
        if obstacle_verts is not None:
            idx, _ = nearest_vertex_index(obstacle_verts, verts[sample])
            outward = verts[sample] - obstacle_verts[idx]  # away from the nearest Bot point
        else:
            outward = verts[sample] - verts.mean(axis=0)   # away from the surface centroid
        vote = float(np.einsum('ij,ij->i', normals[sample], outward).sum())
        return -normals if vote < 0 else normals


    def _layer_endpoints_world(self, layer):
        """The 70 curve endpoints of one layer, in the order roadmap 6.3
        indexes them: endpoint 2p is pieces[p][0] and 2p+1 is pieces[p][-1],
        for p over curved_pieces_world[layer].

        The 3 closed-loop pieces per layer (RX_0/22/27, TX_2/6/17) have
        coincident ends, so their two endpoints are the same point -- they
        snap to one vertex and their cost-matrix rows come out identical,
        with cost[2p, 2p+1] == 0.0. That is correct, but 6.3's ordering must
        not read that zero as free travel to somewhere else."""
        return np.array([e for piece in self.curved_pieces_world[layer]
                          for e in (piece[0], piece[-1])])


    def run_geodesic_precompute(self):
        """Start or resume the chunked geodesic precompute -- roadmap
        Stage6_README.md 6.2. Mirrors run_toolpath_ik_precompute(): only
        builds the graphs and resets progress if nothing is loaded yet
        (geodesic_graphs is None -- true on the first call and again after
        cancel_geodesic_precompute()); if a run is merely paused, resumes
        stepping from geodesic_index with no rebuild.

        Builds one graph per print surface, not one merged graph: each layer
        travels on its own surface and the passes never interleave
        (settled.md S1.30), so a geodesic between two different layers'
        endpoints is meaningless on either mesh.

        One Dijkstra runs per *unique snapped vertex*, not per endpoint --
        measured 58 unique for RX and 55 for TX rather than 70 each, since
        distinct endpoints often land on the same vertex, so this is 113
        runs and not the 140 the roadmap assumed (on the shipped RX/TX
        study config; the ratio depends on whichever layers are configured)."""
        if self.geodesic_graphs is None:
            if not self.curved_model_loaded:
                # Fail with a status message, never an exception: this runs
                # from a button inside the per-frame Polyscope callback.
                self.geodesic_status = "Load Curved Model first"
                return

            n_layers = len(CURVED_LAYERS)
            graphs, snap_nodes, snap_dist, sources, source_row, prev, cost = [], [], [], [], [], [], []
            for layer in range(n_layers):
                verts = self.curved_surface_verts_world[layer]
                graphs.append(build_surface_graph(verts, self.curved_surface_faces[layer]))

                idx, dist = nearest_vertex_index(verts, self._layer_endpoints_world(layer))
                snap_nodes.append(idx)
                snap_dist.append(dist)

                uniq, row = np.unique(idx, return_inverse=True)
                sources.append(uniq)
                source_row.append(row)

                prev.append(np.full((len(uniq), len(verts)), -1, dtype=np.int32))
                cost.append(np.full((len(idx), len(idx)), np.inf))

            self.geodesic_graphs = graphs
            self.geodesic_snap_nodes = snap_nodes
            self.geodesic_snap_dist = snap_dist
            self.geodesic_sources = sources
            self.geodesic_source_row = source_row
            self.geodesic_prev = prev
            self.geodesic_cost = cost
            self.geodesic_unreachable = [0] * n_layers
            self.geodesic_queue = [(layer, r)
                                    for layer in range(n_layers)
                                    for r in range(len(sources[layer]))]
            self.geodesic_index = 0
            self.geodesic_total = len(self.geodesic_queue)
            self.geodesic_loaded = False

        self.geodesic_running = True
        self.geodesic_status = f"Building geodesics {self.geodesic_index}/{self.geodesic_total} sources"


    def pause_geodesic_precompute(self):
        """Stop advancing the geodesic precompute without discarding
        progress. A following run_geodesic_precompute() continues from
        geodesic_index."""
        self.geodesic_running = False


    def cancel_geodesic_precompute(self):
        """Stop and discard the geodesic precompute entirely -- a following
        run_geodesic_precompute() starts fresh."""
        self._abort_geodesic_precompute()
        self.geodesic_status = ""


    def _abort_geodesic_precompute(self):
        """Shared discard used by cancel_geodesic_precompute() and
        load_curved_model(). Resets geodesic_index/total together, so a stale
        index can't outlive the arrays it counted (the same failure
        _abort_toolpath_ik_precompute() guards against, settled.md S1.24).
        Does not touch geodesic_status, so a caller can set an explanatory
        message first."""
        self.geodesic_running = False
        self.geodesic_loaded = False
        self.geodesic_index = 0
        self.geodesic_total = 0
        self.geodesic_graphs = None
        self.geodesic_snap_nodes = None
        self.geodesic_snap_dist = None
        self.geodesic_sources = None
        self.geodesic_source_row = None
        self.geodesic_queue = None
        self.geodesic_prev = None
        self.geodesic_cost = None
        self.geodesic_unreachable = None

        # The 6.3 print order and its travel moves are derived from the cost
        # matrices and predecessor rows just dropped, so they go stale with
        # them -- clear the state and remove the rendered travel networks.
        if self.curved_layer_names is not None:
            for name in self.curved_layer_names:
                ps.remove_curve_network(f"Curved Travel {name}", error_if_absent=False)
                ps.remove_curve_network(f"Curved Order Feed {name}", error_if_absent=False)
                ps.remove_curve_network(f"Curved Orient Frames {name}", error_if_absent=False)
                ps.remove_surface_mesh(f"Curved Print {name}", error_if_absent=False)
        self.curved_order_loaded = False
        self.curved_print_order = None
        self.curved_travel_moves = None
        self.curved_travel_total = None
        self.curved_travel_naive = None
        self.curved_order_status = ""

        # The 6.4 orientation frames derive from the print order just dropped.
        self.curved_orient_loaded = False
        self.curved_orient_frames = None
        self.curved_orient_status = ""

        # The 6.6 printed-bead meshes are built from the waypoints derived
        # above (print order + orientation), across every layer -- not just
        # whichever is currently active -- so they go stale with them too.
        self.curved_bead_verts_full = None
        self.curved_bead_faces = None
        self.curved_bead_reveal_index = None
        self.curved_bead_face_prefix = None
        self.curved_bead_verts_current = None
        self.curved_print_handle = None
        self.curved_bead_registered_capacity = None


    def step_geodesic_precompute(self):
        """Advance the in-progress geodesic precompute by up to
        GEODESIC_CHUNK_SOURCES whole Dijkstra sources -- call every frame
        from render(). No-ops unless geodesic_running.

        Unlike step_toolpath_ik_precompute() there is no abort-on-failure
        branch, deliberately: Dijkstra cannot fail. An unreachable target is
        data (inf), not an error, and discarding a 70x70 matrix because one
        pair sits on a disconnected mesh fragment would throw away the 4,830
        other off-diagonal entries alongside it."""
        if not self.geodesic_running:
            return

        end = min(self.geodesic_index + GEODESIC_CHUNK_SOURCES, self.geodesic_total)
        for i in range(self.geodesic_index, end):
            layer, row = self.geodesic_queue[i]
            start, nbr, weight = self.geodesic_graphs[layer]
            dist, prev = dijkstra_surface(start, nbr, weight, int(self.geodesic_sources[layer][row]))
            self.geodesic_prev[layer][row] = prev

            # One solve fills every endpoint row that snapped to this vertex,
            # so the duplicate rows cost nothing and can't disagree with their
            # twin. Only the 70 endpoint columns are kept -- retaining full
            # dist rows would double the memory for data nothing reads.
            fill = (self.geodesic_source_row[layer] == row)
            self.geodesic_cost[layer][fill, :] = dist[self.geodesic_snap_nodes[layer]]

            if row == 0:
                # The first solve of a layer already covers the whole graph,
                # so it is the reachability oracle -- no separate flood fill
                # needed. Pausing (not just setting status) is what makes the
                # warning readable: a plain status here would be overwritten
                # by "Building geodesics N/M" one frame later. Build resumes
                # the run through the normal pause/resume path.
                n_bad = int(np.isinf(dist[self.geodesic_snap_nodes[layer]]).sum())
                if n_bad:
                    self.geodesic_index = i + 1
                    self.geodesic_running = False
                    self.geodesic_status = (f"{self.curved_layer_names[layer]}: {n_bad}/"
                                             f"{len(self.geodesic_snap_nodes[layer])} endpoints unreachable "
                                             f"-- surface is fragmented (Build Geodesics resumes)")
                    return

        self.geodesic_index = end

        if self.geodesic_index >= self.geodesic_total:
            self.geodesic_running = False
            self.geodesic_loaded = True
            self.geodesic_unreachable = [int(np.isinf(c).sum()) for c in self.geodesic_cost]
            if any(self.geodesic_unreachable):
                self.geodesic_status = (f"Geodesics ready with {sum(self.geodesic_unreachable)} "
                                         f"unreachable pair(s) -- surface is fragmented")
            else:
                spans = [c[np.isfinite(c)].max() for c in self.geodesic_cost]
                self.geodesic_status = "Geodesics ready -- " + ", ".join(
                    f"{name} max {span:.0f}mm" for name, span in zip(self.curved_layer_names, spans))
        else:
            self.geodesic_status = f"Building geodesics {self.geodesic_index}/{self.geodesic_total} sources"


    def _surface_boundary_vertices(self, layer):
        """Vertex ids on the open boundary of `layer`'s surface -- the ends of
        edges used by a single face. Feeds only the rim-hugging travel
        diagnostic (settled.md S1.31 / CurvedModel_Geodesics.md): a geodesic
        can legitimately track the shell's open edge, worth surfacing
        numerically since rim-hugging travel may not be physically desirable."""
        f = self.curved_surface_faces[layer]
        e = np.sort(np.concatenate([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]]), axis=1)
        uniq, counts = np.unique(e, axis=0, return_counts=True)
        return np.unique(uniq[counts == 1])


    def build_print_order(self):
        """Order each layer's pieces and emit the travel moves between them --
        roadmap 6.3. Synchronous: unlike the 6.2 Dijkstra precompute this only
        walks stored predecessor rows (no re-solve), so it finishes well inside
        one frame. Gated on geodesic_loaded.

        Each layer is ordered independently on its own surface (settled.md
        S1.30/S1.32) -- RX first, TX second by CURVED_LAYERS order, with no
        travel move stitching the last RX piece to the first TX piece (the
        manual silicone fill sits in that gap). Travel polylines follow the 6.2
        geodesic offset outward by CURVED_TRAVEL_HOVER_MM along the local
        surface normal, bookended with the true curve endpoints (also lifted)
        so the ~0.36mm snap gap is closed and the route meets the feed moves."""
        if not self.geodesic_loaded:
            self.curved_order_status = "Build geodesics first"
            return

        # A fresh order invalidates any 6.4 orientation frames built against the
        # previous one -- drop their state and rendered triads so a re-order
        # can't leave a stale overlay behind.
        if self.curved_orient_loaded:
            for name in self.curved_layer_names:
                ps.remove_curve_network(f"Curved Orient Frames {name}", error_if_absent=False)
            self.curved_orient_loaded = False
            self.curved_orient_frames = None
            self.curved_orient_status = ""

        n_layers = len(self.curved_layer_names)
        self.curved_print_order = [None] * n_layers
        self.curved_travel_moves = [None] * n_layers
        self.curved_travel_total = [0.0] * n_layers
        self.curved_travel_naive = [0.0] * n_layers
        rim_frac = []

        for layer in range(n_layers):
            cost = self.geodesic_cost[layer]
            n_pieces = cost.shape[0] // 2
            order = two_opt(cost, greedy_piece_order(cost))
            self.curved_print_order[layer] = order
            self.curved_travel_total[layer] = travel_cost(order, cost)
            self.curved_travel_naive[layer] = travel_cost(
                [(p, 2 * p) for p in range(n_pieces)], cost)

            endpoints = self._layer_endpoints_world(layer)
            verts = self.curved_surface_verts_world[layer]
            vnormals = self.curved_surface_vnormals_world[layer]
            snap = self.geodesic_snap_nodes[layer]
            boundary = self._surface_boundary_vertices(layer)

            def lift_endpoint(ep):  # true curve endpoint, lifted along its snap-vertex normal
                return endpoints[ep] + CURVED_TRAVEL_HOVER_MM * vnormals[snap[ep]]

            polylines, n_nodes, n_rim = [], 0, 0
            for (_, entry_a), (_, entry_b) in zip(order, order[1:]):
                exit_ep = entry_a ^ 1
                row = int(self.geodesic_source_row[layer][exit_ep])
                nodes = geodesic_path_nodes(self.geodesic_prev[layer][row], int(snap[entry_b]))
                if nodes is None:
                    continue  # unreachable -- never on the shipped single-component surfaces
                hover = verts[nodes] + CURVED_TRAVEL_HOVER_MM * vnormals[nodes]
                polylines.append(np.vstack([lift_endpoint(exit_ep), hover, lift_endpoint(entry_b)]))
                n_nodes += len(nodes)
                n_rim += int(np.isin(nodes, boundary).sum())
            self.curved_travel_moves[layer] = polylines
            rim_frac.append(n_rim / n_nodes if n_nodes else 0.0)

            name = f"Curved Travel {self.curved_layer_names[layer]}"
            if polylines:
                self._register_curve_layer(name, polylines, np.eye(4), CURVED_TRAVEL_COLOR)

            # Ordered-feed overlay: the printed pieces in print order, each
            # oriented by its entry end (reversed when entered at 2p+1), drawn as
            # a sequence gradient so the order itself is legible. _register_curve_layer
            # builds edges piece-by-piece in the order given, so edge index runs
            # along the print order and _sequence_colors() maps straight onto it.
            ordered_pieces = [self.curved_pieces_world[layer][p][::-1] if entry & 1
                              else self.curved_pieces_world[layer][p]
                              for p, entry in order]
            feed_name = f"Curved Order Feed {self.curved_layer_names[layer]}"
            handle = self._register_curve_layer(feed_name, ordered_pieces, np.eye(4),
                                                 CURVED_ORDER_CMAP[0])
            handle.set_radius(CURVED_ORDER_FEED_RADIUS_MM, relative=False)
            n_edges = sum(len(p) - 1 for p in ordered_pieces)
            handle.add_color_quantity("print order", _sequence_colors(n_edges),
                                       defined_on='edges', enabled=True)

        self.curved_order_loaded = True
        summary = "; ".join(
            f"{self.curved_layer_names[l]}: travel {self.curved_travel_total[l]:.0f}mm "
            f"vs {self.curved_travel_naive[l]:.0f}mm file-order ({len(self.curved_print_order[l])} pieces)"
            for l in range(n_layers))
        self.curved_order_status = (f"Print order ready -- {summary}; "
                                     f"max {max(rim_frac) * 100:.0f}% travel nodes on rim")


    def build_orientation_frames(self):
        """Attach a per-waypoint TCP orientation to every printed feed point,
        holding the nozzle perpendicular to the curved surface -- roadmap 6.4.
        Synchronous, gated on curved_order_loaded (walks the stored print
        order, no re-solve). Supersedes the flat-plate single-constant
        R_target of settled.md S1.12 for the curved path.

        Per feed point the target TCP orientation R (base frame, 3x3) is:
          - Z = the outward surface normal (nozzle approaches along -Z, into
            the surface). Matches the planar convention where R_target's
            third column is the plate's outward +Z; the outward sign is
            already fixed away from Surface_Bot at load (S1.35), so it's safe.
          - X, Y = a fixed world reference projected into the tangent plane,
            NOT the path tangent. The nozzle is rotationally symmetric about
            its axis, so this DOF is free; pinning it to a constant world
            direction keeps the frame from spinning as the toolpath meanders
            (only the normal tilts it), minimising wrist travel. The
            reference axis is chosen per point as whichever world axis is most
            perpendicular to Z, so the projection never collapses and adjacent
            frames stay close (no flip as the normal sweeps past an axis)."""
        if not self.curved_order_loaded:
            self.curved_orient_status = "Build print order first"
            return

        n_layers = len(self.curved_layer_names)
        self.curved_orient_frames = [None] * n_layers

        for layer in range(n_layers):
            # Feed points in print order -- the exact derivation build_print_order
            # uses for its ordered-feed overlay (reversed when entered at 2p+1).
            ordered_pieces = [self.curved_pieces_world[layer][p][::-1] if entry & 1
                              else self.curved_pieces_world[layer][p]
                              for p, entry in self.curved_print_order[layer]]
            points = np.vstack(ordered_pieces)  # (W,3) all feed waypoints

            R_array = self._orientation_frames_for_points(layer, points)
            frames = list(zip(points, R_array))
            self.curved_orient_frames[layer] = frames

            self._register_orientation_frames(
                f"Curved Orient Frames {self.curved_layer_names[layer]}", frames)

        self.curved_orient_loaded = True
        counts = "; ".join(f"{self.curved_layer_names[l]}: {len(self.curved_orient_frames[l])} waypoints"
                           for l in range(n_layers))
        self.curved_orient_status = f"Orientation frames ready -- {counts}"


    def _orientation_frames_for_points(self, layer, points):
        """Per-point TCP orientation matrices holding the nozzle perpendicular
        to the curved surface -- the frame math shared by build_orientation_frames
        (6.4, feed points only) and build_curved_toolpath_waypoints_world (6.5,
        feed + travel). points is (N,3) world positions; returns (N,3,3).

        Z is the outward surface normal from the nearest surface vertex (the
        verts array is only the nearest-vertex query target; the normals live in
        the separate curved_surface_vnormals_world array). X/Y are a fixed world
        reference projected into the tangent plane (whichever world axis is most
        perpendicular to Z, so the projection never collapses and adjacent frames
        don't flip) -- the nozzle is rotationally symmetric about its axis, so
        this DOF is free and pinning it keeps the wrist from spinning."""
        world_axes = np.eye(3)
        snap, _ = nearest_vertex_index(self.curved_surface_verts_world[layer], points)
        normals = self.curved_surface_vnormals_world[layer][snap]  # (N,3), outward unit
        frames = np.empty((len(points), 3, 3))
        for i, z in enumerate(normals):
            z = z / np.linalg.norm(z)
            a = world_axes[np.argmin(np.abs(world_axes @ z))]  # most perpendicular to z
            x = a - np.dot(a, z) * z
            x /= np.linalg.norm(x)
            y = np.cross(z, x)
            frames[i] = np.column_stack([x, y, z])
        return frames


    def build_curved_toolpath_waypoints_world(self, layer):
        """Merge a layer's ordered feed pieces and inter-piece travel hops into
        one oriented waypoint list -- the curved analogue of
        build_toolpath_waypoints_world (roadmap 6.5). Walks curved_print_order,
        interleaving each oriented feed piece (curved_pieces_world[p], reversed
        when entered at 2p+1, same convention build_print_order/
        build_orientation_frames use) with the travel polyline that follows it
        (curved_travel_moves[k], already bookended with lifted true endpoints).

        Returns (waypoints, R_target_array), the same tuple shape as
        build_toolpath_waypoints_world so it's a drop-in alternate precompute
        source: waypoints is a list of (pos_world_mm (3,), is_feed_move bool);
        R_target_array is (N,3,3), one orientation per waypoint (feed and travel
        both get a surface-normal frame)."""
        order = self.curved_print_order[layer]
        travel = self.curved_travel_moves[layer]
        # build_print_order appends one travel polyline per consecutive-piece gap
        # (zip(order, order[1:])) but skips a gap whose geodesic is unreachable
        # (never on the shipped single-component surfaces). Assert the 1:1 pairing
        # so a future non-trivial surface fails loud rather than stitching the
        # wrong travel move to the wrong gap.
        assert len(travel) == len(order) - 1, (
            f"layer {layer}: {len(travel)} travel moves for {len(order)} pieces "
            f"(expected {len(order) - 1}); a geodesic gap was skipped")

        positions, is_feed = [], []
        for k, (p, entry) in enumerate(order):
            piece = self.curved_pieces_world[layer][p]
            piece = piece[::-1] if entry & 1 else piece
            positions.append(piece)
            is_feed.extend([True] * len(piece))
            if k < len(travel):
                hop = travel[k]
                positions.append(hop)
                is_feed.extend([False] * len(hop))

        points = np.vstack(positions)
        R_target_array = self._orientation_frames_for_points(layer, points)
        waypoints = [(pos, feed) for pos, feed in zip(points, is_feed)]
        return waypoints, R_target_array


    def _build_curved_beads(self, layer):
        """Curved analogue of _build_gcode_beads() -- roadmap 6.6. The PLY
        toolpath curves carry no extrusion data, and there's no single
        "layer Z" on a conformal path, so width/height are fixed constants
        (CURVED_BEAD_WIDTH_MM/HEIGHT_MM) instead of derived from E/Z, and the
        box's stacking axis is each waypoint's own local surface normal
        (R_target[:,2], averaged per segment) instead of world Z -- a curved
        surface has no single "up". Waypoints from
        build_curved_toolpath_waypoints_world() are already world-space
        (unlike gcode_points, which are plate-local), so no
        transform_points()/PLATE_THICKNESS_MM step is needed here. Reuses
        _BEAD_BOX_FACE_TEMPLATE and the colinear cap-cull test verbatim;
        drops the width_matched test (trivially true since width is
        constant here).

        Returns the same (verts_world, faces, reveal_waypoint_index,
        bead_face_prefix) tuple shape as _build_gcode_beads()."""
        waypoints, R_target_array = self.build_curved_toolpath_waypoints_world(layer)
        pts = np.array([p for p, _ in waypoints])
        is_feed = np.array([f for _, f in waypoints])

        p0, p1 = pts[:-1], pts[1:]
        seg_vec = p1 - p0
        seg_len = np.linalg.norm(seg_vec, axis=1)
        # Both endpoints feed -- excludes lift-off/touch-down transition
        # segments into/out of a travel hop (no extrusion signal to key off
        # instead, unlike _build_gcode_beads' delta_e check).
        seg_is_print = is_feed[:-1] & is_feed[1:]

        safe_len = np.where(seg_len > 1e-9, seg_len, 1.0)
        u = seg_vec / safe_len[:, None]

        # Per-segment "up" = the two waypoints' own R_target Z columns
        # (local surface normal), averaged and re-normalized -- not world Z.
        n0, n1 = R_target_array[:-1, :, 2], R_target_array[1:, :, 2]
        normal_seg = n0 + n1
        safe_normal_len = np.where(np.linalg.norm(normal_seg, axis=1, keepdims=True) > 1e-9,
                                    np.linalg.norm(normal_seg, axis=1, keepdims=True), 1.0)
        normal_seg = normal_seg / safe_normal_len

        w_axis = np.cross(u, normal_seg)  # width direction: perpendicular to both travel and normal
        w_norm = np.linalg.norm(w_axis, axis=1)
        safe_w_norm = np.where(w_norm > 1e-9, w_norm, 1.0)
        w_axis = w_axis / safe_w_norm[:, None]

        valid = seg_is_print & (seg_len > 1e-6) & (w_norm > 1e-6)
        if not np.any(valid):
            return (np.empty((0, 3)), np.empty((0, 3), dtype=int), np.empty(0, dtype=int),
                    np.zeros(1, dtype=int))

        reveal_waypoint_index = np.nonzero(valid)[0] + 1
        p0v, p1v = p0[valid], p1[valid]
        w_axis_v, normal_v = w_axis[valid], normal_seg[valid]
        half_w, half_h = CURVED_BEAD_WIDTH_MM / 2.0, CURVED_BEAD_HEIGHT_MM / 2.0

        c0 = p0v + w_axis_v * half_w
        c1 = p0v - w_axis_v * half_w
        c2 = p1v - w_axis_v * half_w
        c3 = p1v + w_axis_v * half_w

        K = len(p0v)
        verts_world = np.zeros((K, 8, 3))
        for idx, corner in enumerate((c0, c1, c2, c3)):
            verts_world[:, idx, :] = corner - normal_v * half_h
            verts_world[:, idx + 4, :] = corner + normal_v * half_h
        verts_world = verts_world.reshape(-1, 3)

        u_valid = u[valid]
        chained = np.diff(reveal_waypoint_index) == 1
        colinear = np.sum(u_valid[:-1] * u_valid[1:], axis=1) >= CAP_CULL_COLINEAR_DOT_MIN
        cullable = chained & colinear

        drop_end_cap = np.zeros(K, dtype=bool)
        drop_start_cap = np.zeros(K, dtype=bool)
        drop_end_cap[:-1] = cullable
        drop_start_cap[1:] = cullable

        keep_row = np.ones((K, 12), dtype=bool)
        keep_row[drop_end_cap, 8] = False
        keep_row[drop_end_cap, 9] = False
        keep_row[drop_start_cap, 4] = False
        keep_row[drop_start_cap, 5] = False

        faces_full = (self._BEAD_BOX_FACE_TEMPLATE[None, :, :] + (np.arange(K) * 8)[:, None, None])
        faces = faces_full[keep_row]
        bead_face_prefix = np.concatenate([[0], np.cumsum(keep_row.sum(axis=1))])

        return verts_world, faces, reveal_waypoint_index, bead_face_prefix


    def _init_curved_toolpath_playback(self, layer):
        """Curved analogue of _init_toolpath_playback() -- roadmap 6.6.
        Requires a completed precompute for this exact layer (checked via
        cache_path, since precompute_joint_path alone doesn't say which
        source solved it). Builds via _build_curved_beads() and registers
        under this layer's own name/slot so a different layer's already-
        printed mesh is untouched. Lazily sizes the per-layer bead-state
        lists to len(curved_layer_names) on first use. Returns True on
        success, False (with playback_status explaining why) otherwise."""
        if self.curved_bead_verts_full is None:
            n = len(self.curved_layer_names)
            self.curved_bead_verts_full = [None] * n
            self.curved_bead_faces = [None] * n
            self.curved_bead_reveal_index = [None] * n
            self.curved_bead_face_prefix = [None] * n
            self.curved_bead_verts_current = [None] * n
            self.curved_print_handle = [None] * n
            self.curved_bead_registered_capacity = [None] * n

        if (not self.precompute_joint_path
                or self.precompute_cache_path != curved_precompute_cache_path(self.curved_layer_names[layer])):
            self.playback_status = "Run Precompute for this layer first"
            return False

        verts_world, faces, reveal_index, face_prefix = self._build_curved_beads(layer)
        if len(verts_world) == 0:
            self.playback_status = "No printed beads to reveal"
            return False

        self.curved_bead_verts_full[layer] = verts_world
        self.curved_bead_faces[layer] = faces
        self.curved_bead_reveal_index[layer] = reveal_index
        self.curved_bead_face_prefix[layer] = face_prefix
        # Collapse every bead to its own first corner -- a zero-area box
        # renders nothing, revealed later by restoring real positions
        # (advance_toolpath_playback), never via transparency (settled.md S1.16).
        self.curved_bead_verts_current[layer] = np.repeat(verts_world[0::8], 8, axis=0)

        K = len(reveal_index)
        capacity = min(PLAYBACK_LOOKAHEAD_BEADS, K)
        self.curved_bead_registered_capacity[layer] = capacity
        name = f"Curved Print {self.curved_layer_names[layer]}"
        handle = ps.register_surface_mesh(
            name, self.curved_bead_verts_current[layer][:capacity * 8],
            self.curved_bead_faces[layer][:self.curved_bead_face_prefix[layer][capacity]])
        handle.set_color(CURVED_LAYERS[layer]["curve_color"])  # reuse the layer's curve color, no new constant
        self.curved_print_handle[layer] = handle

        self.playback_index = 0
        self._last_rendered_playback_index = 0
        self.update_arm(self.precompute_joint_path[0])
        return True


    def _register_orientation_frames(self, name, frames):
        """Draw a downsampled batch of TCP orientation triads as one curve
        network (every ORIENT_FRAME_STRIDE-th frame), X red / Y green / Z blue
        -- same colour scheme as create_coordinate_frame, batched across many
        origins like _register_curve_layer."""
        sampled = frames[::ORIENT_FRAME_STRIDE]
        nodes, edges, colors, offset = [], [], [], 0
        for pos, R in sampled:
            tips = pos + ORIENT_FRAME_SCALE_MM * R.T  # rows: +X, +Y, +Z tips
            nodes.append(np.vstack([pos, tips]))
            edges.append(np.array([[0, 1], [0, 2], [0, 3]]) + offset)
            colors.append(ORIENT_FRAME_COLORS)
            offset += 4
        handle = ps.register_curve_network(name, np.vstack(nodes), np.vstack(edges))
        handle.add_color_quantity("axis_colors", np.vstack(colors),
                                  defined_on='edges', enabled=True)
        handle.set_radius(CURVE_RADIUS_MM, relative=False)
        return handle


    def apply_live_layer_visibility(self, layer):
        """Show `layer`'s geometry and every layer beneath it in the physical
        print stack -- the S1.32 stack rule (roadmap 6.6), driven by the GUI's
        toolpath-source selector. Necessary because Surface_RX_Offset is
        sealed inside the Surface_TX_Base shell (settled.md S1.32), so RX is
        invisible with TX shown; conversely TX's view should show the already-
        printed RX layer beneath it, since RX -> silicone fill -> TX is a
        real, physically stacked sequence, not three independent views.

        Per configured layer, the surface / ordered-feed overlay / travel
        network / orientation-frame triads / printed bead mesh are enabled
        for every layer at or before `layer` in CURVED_LAYERS order (index 0
        = first pass = innermost); the base toolpath curve shows only when
        its overlay is absent (the gradient overlay supersedes it, so they
        don't z-fight). The obstacle mesh is left as-is -- it's shared mockup
        context. Every structure is guarded, since overlays/travel/bead
        meshes don't exist until their building stage has run.

        During playback (self.playback_active, roadmap 6.7) the guide overlays
        -- the order-feed/travel/orientation curve networks and the base
        toolpath curve -- are force-hidden regardless of the stack rule, so the
        growing bead mesh is actually visible; surfaces and beads keep following
        the i <= layer stack rule."""
        if not self.curved_model_loaded:
            return
        for i, cfg in enumerate(CURVED_LAYERS):
            visible = (i <= layer)  # layer k's view shows layers 0..k, the physical stack.
            overlay_visible = visible and not self.playback_active  # guides hide during playback -- 6.7
            surface = cfg["surface_structure_name"]
            base_curve = cfg["curve_structure_name"]
            overlay = f"Curved Order Feed {self.curved_layer_names[i]}"
            travel = f"Curved Travel {self.curved_layer_names[i]}"
            orient = f"Curved Orient Frames {self.curved_layer_names[i]}"
            bead = f"Curved Print {self.curved_layer_names[i]}"

            if ps.has_surface_mesh(surface):
                ps.get_surface_mesh(surface).set_enabled(visible)
            for name in (overlay, travel, orient):
                if ps.has_curve_network(name):
                    ps.get_curve_network(name).set_enabled(overlay_visible)
            if ps.has_surface_mesh(bead):
                ps.get_surface_mesh(bead).set_enabled(visible)
            if ps.has_curve_network(base_curve):
                # Overlay wins when present; the base curve is the fallback view.
                ps.get_curve_network(base_curve).set_enabled(overlay_visible and not ps.has_curve_network(overlay))


    def build_toolpath_waypoints_world(self, gcode_points):
        """
        Map parse_gcode()'s plate-local waypoints to world-space 6-DOF
        targets -- roadmap Stage5_README.md 5.4. gcode_points is
        parse_gcode()'s own return value (not re-parsed here, kept
        composable). Applies the same plate-local Z lift
        (PLATE_THICKNESS_MM) load_gcode()/load_build_plate() already use,
        then the same T_user_frame homogeneous multiply as load_gcode() --
        see settled.md S1.3.

        No subdivision: one returned waypoint per input point, in order --
        see settled.md S1.12.

        Returns (waypoints, R_target):
          waypoints: list of (pos_world_mm: np.ndarray[3], is_feed_move: bool),
            same length/order as gcode_points -- both G0 (travel) and G1
            (feed) points are included (unlike load_gcode()'s
            G1-extruding-only bead filter), so a caller can still tell them
            apart later.
          R_target: 3x3 np.ndarray, self.T_user_frame[:3,:3] snapshotted
            once here -- the constant TCP orientation for the whole path
            (the plate doesn't tilt mid-print, see settled.md S1.6/S1.8),
            not stored per-waypoint.
        """
        pts_local = np.array([p for p, _, _ in gcode_points], dtype=float)
        is_feed = [f for _, _, f in gcode_points]
        pts_local[:, 2] += PLATE_THICKNESS_MM
        pts_world = transform_points(self.T_user_frame, pts_local)
        R_target = self.T_user_frame[:3, :3].copy()
        return list(zip(pts_world, is_feed)), R_target


    # FR5 standard DH parameters: (a_mm, alpha_rad, d_mm, theta_offset_rad)
    # Source: docs/FR5_DH_Table.md
    DH_PARAMS = [
        (0,    np.pi / 2, 152, 0),
        (-425, 0,         0,   0),
        (-395, 0,         0,   0),
        (0,    np.pi / 2, 102, 0),
        (0,   -np.pi / 2, 102, 0),
        (0,    0,         100, 0),
    ]

       
    def compute_fk(self, joint_angles_deg):
        """
        joint_angles_deg: sequence of 6 joint angles in degrees [J1..J6]
        Returns [T_0_1, ..., T_0_6], each a 4x4 np.ndarray. T_0_6 (base->flange)
        is the last element.
        """
        T = np.eye(4)
        T_0_i = []
        for (a, alpha, d, theta_offset), joint_deg in zip(self.DH_PARAMS, joint_angles_deg):
            theta = np.deg2rad(joint_deg) + theta_offset
            T = T @ dh_transform(a, alpha, d, theta)
            T_0_i.append(T)
        return T_0_i


    def end_effector_position(self, joint_angles_deg):
        T_0_6 = (self.compute_fk(joint_angles_deg))[-1]
        print(T_0_6[:3, 3])


    def solve_ik(self, T_0_6_target):
        """
        Closed-form analytical IK for the FR5's specific DH geometry --
        NOT a literal port of a PUMA-style spherical wrist, since d4 and
        d5 are both nonzero here (same family as a UR5/UR10 wrist). Full
        derivation: docs/FR5_IK_Derivation.md.

        T_0_6_target: 4x4 np.ndarray, target flange pose (base frame).
        Returns a list of (joint_angles_deg: np.ndarray[6], is_wrist_singular: bool),
        one per geometrically valid branch (up to 8; branches that fail an
        acos/sqrt domain check -- pose unreachable along that branch -- are
        silently skipped).
        """
        d1 = self.DH_PARAMS[0][2]
        a2 = self.DH_PARAMS[1][0]
        a3 = self.DH_PARAMS[2][0]
        d4 = self.DH_PARAMS[3][2]
        d5 = self.DH_PARAMS[4][2]
        d6 = self.DH_PARAMS[5][2]

        R = T_0_6_target[:3, :3]
        p = T_0_6_target[:3, 3]
        P5 = p - d6 * R[:, 2]  # frame-5 origin, backing off the approach vector

        solutions = []
        R_xy_sq = P5[0] ** 2 + P5[1] ** 2
        if R_xy_sq < d4 ** 2:
            return solutions  # P5 can't reach the fixed shoulder-perpendicular offset

        for sign1 in (1, -1):
            theta1 = np.arctan2(P5[1], P5[0]) + np.arctan2(d4, sign1 * np.sqrt(R_xy_sq - d4 ** 2))
            c1, s1 = np.cos(theta1), np.sin(theta1)

            wrist_arg = (p[0] * s1 - p[1] * c1 - d4) / d6
            if abs(wrist_arg) > 1:
                continue

            R_0_1 = dh_transform(0, np.pi / 2, d1, theta1)[:3, :3]
            R_1_6 = R_0_1.T @ R

            for sign2 in (1, -1):
                theta5 = sign2 * np.arccos(wrist_arg)
                is_singular = abs(np.sin(theta5)) < 1e-6

                if is_singular:
                    theta6 = 0.0  # axes 4/6 aligned -- theta4/theta6 split is ambiguous
                else:
                    theta6 = np.arctan2(-R_1_6[2, 1] / np.sin(theta5), R_1_6[2, 0] / np.sin(theta5))

                K = rot_y(-theta5) @ rot_z(theta6)
                Rz_psi = R_1_6 @ K.T
                psi = np.arctan2(Rz_psi[1, 0], Rz_psi[0, 0])  # psi = theta2+theta3+theta4

                X_p = P5[0] * c1 + P5[1] * s1
                Y_p = P5[2] - d1
                X = X_p - d5 * np.sin(psi)
                Y = Y_p + d5 * np.cos(psi)

                cos_theta3 = (X ** 2 + Y ** 2 - a2 ** 2 - a3 ** 2) / (2 * a2 * a3)
                if abs(cos_theta3) > 1:
                    continue

                for sign3 in (1, -1):
                    theta3 = sign3 * np.arccos(cos_theta3)
                    theta2 = np.arctan2(Y, X) - np.arctan2(a3 * np.sin(theta3), a2 + a3 * np.cos(theta3))
                    theta4 = psi - theta2 - theta3

                    angles_rad = np.array([theta1, theta2, theta3, theta4, theta5, theta6])
                    solutions.append((np.rad2deg(angles_rad), is_singular))

        return solutions


    def solve_ik_tcp_matrix(self, target_pos_mm, R_target, joint_limits, reference_joint_angles=None):
        """
        Matrix-native IK entry point, targeting the TCP pose -- see settled.md
        S1.4. Converts via self.T_flange_to_tcp, solves, filters by
        joint_limits, then ranks every valid branch by closeness to a
        reference pose -- see settled.md S1.5.

        R_target: 3x3 np.ndarray, target TCP orientation (base frame).
        reference_joint_angles: joint_angles_deg (np.ndarray[6]) to rank
        branches against; defaults to self.current_joint_angles when None
        (the arm's live pose -- reproduces solve_ik_tcp's existing behavior).
        A future toolpath driver instead passes the previous waypoint's
        solved pose, for continuity between consecutive solves.

        Returns (solutions, status_message); solutions is a list of
        (joint_angles_deg, is_wrist_singular, raw_branch_index), sorted
        closest-to-reference first. Empty list on failure.
        """
        if reference_joint_angles is None:
            reference_joint_angles = self.current_joint_angles

        T_target_tcp = np.eye(4)
        T_target_tcp[:3, :3] = R_target
        T_target_tcp[:3, 3] = target_pos_mm
        T_target_flange = T_target_tcp @ np.linalg.inv(self.T_flange_to_tcp)

        branches = self.solve_ik(T_target_flange)
        if not branches:
            return [], "Unreachable: no geometric solution for this pose"

        def wrap_into_limits(angle_deg, lo, hi):
            # atan2-built angles come back in (-180,180]-ish ranges, but joints
            # like J2/J4 have physical limits past that (e.g. -264 deg) -- the
            # same physical angle can be +/-360 off and only one representation
            # falls inside the asymmetric limit window, so all three are tried.
            for k in (0, 360, -360):
                candidate = angle_deg + k
                if lo <= candidate <= hi:
                    return candidate
            return None

        valid = []
        for i, (angles, singular) in enumerate(branches):
            adjusted = [wrap_into_limits(a, lo, hi) for a, (lo, hi) in zip(angles, joint_limits)]
            if all(a is not None for a in adjusted):
                valid.append((np.array(adjusted), singular, i))
        if not valid:
            return [], f"Reachable but outside joint limits ({len(branches)} branch(es), none valid)"

        def wrapped_dist(angles):
            diff = (angles - reference_joint_angles + 180) % 360 - 180
            return np.sum(np.abs(diff))

        valid.sort(key=lambda item: wrapped_dist(item[0]))
        status = f"Solved ({len(valid)} valid solution{'s' if len(valid) != 1 else ''})"
        return valid, status


    def solve_ik_tcp(self, target_pos_mm, target_rpy_deg, joint_limits):
        """
        GUI-facing IK entry point, targeting the TCP pose via RPY orientation
        -- see settled.md S1.4. Thin wrapper: converts RPY to a rotation
        matrix, then delegates to solve_ik_tcp_matrix() (ranked against
        self.current_joint_angles, i.e. the arm's live pose -- see settled.md
        S1.5).

        target_rpy_deg: [roll, pitch, yaw] degrees, fixed-angle convention
        (R = Rz(yaw) @ Ry(pitch) @ Rx(roll)).
        Returns (solutions, status_message); solutions is a list of
        (joint_angles_deg, is_wrist_singular, raw_branch_index), sorted
        closest-to-current first. Empty list on failure.
        """
        roll, pitch, yaw = np.deg2rad(target_rpy_deg)
        R_target = rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)
        return self.solve_ik_tcp_matrix(target_pos_mm, R_target, joint_limits)


    def solve_toolpath_ik(self, waypoints, R_target, joint_limits, reference_joint_angles=None):
        """
        Solve IK across a whole toolpath, chaining continuity
        waypoint-to-waypoint -- see settled.md S1.11, roadmap
        Stage5_README.md 5.4. waypoints is
        build_toolpath_waypoints_world()'s first return value; R_target is
        its second (the constant TCP orientation for the whole path).

        For each waypoint, calls solve_ik_tcp_matrix() ranked against the
        previous waypoint's chosen solution (or reference_joint_angles /
        self.current_joint_angles for the first waypoint), then walks the
        ranked branches and takes the first one that clears the ground
        (_branch_clears_ground -- roadmap 5.5, settled.md S1.13), not
        blindly the top-ranked branch.

        Aborts the entire solve at the first waypoint with no valid branch,
        or where every valid branch dips below the ground plane -- no
        partial motion, matching the abort contract roadmap 5.6's chunked
        precompute will also need (settled.md S1.12).

        Returns (joint_path, status_message):
          joint_path: list of joint_angles_deg (np.ndarray[6]), one per
            waypoint, in order. Empty list on failure.
          status_message: "Solved N waypoint(s)" on success; on failure, the
            failing waypoint's index plus either solve_ik_tcp_matrix's own
            status string verbatim ("Unreachable: ..." / "Reachable but
            outside joint limits ...") or the ground-clearance failure
            message when every valid branch dips below z=0.
        """
        if reference_joint_angles is None:
            reference_joint_angles = self.current_joint_angles

        joint_path = []
        ref = reference_joint_angles
        for i, (pos_world_mm, _is_feed_move) in enumerate(waypoints):
            solutions, status = self.solve_ik_tcp_matrix(
                pos_world_mm, R_target, joint_limits, reference_joint_angles=ref)
            if not solutions:
                return [], f"Waypoint {i}/{len(waypoints)}: {status}"

            clear = next((angles for angles, *_ in solutions if self._branch_clears_ground(angles)), None)
            if clear is None:
                return [], f"Waypoint {i}/{len(waypoints)}: all {len(solutions)} valid branch(es) dip below the ground plane (z<0)"

            ref = clear
            joint_path.append(ref)
        return joint_path, f"Solved {len(joint_path)} waypoint(s)"


    def _toolpath_cache_meta(self, user_frame):
        """Cache-key dict for the toolpath precompute cache -- roadmap
        Stage5_README.md 5.10, settled.md S1.21. Compared by dict equality,
        not a hash-of-hash, so any single differing element is an
        unambiguous miss. Hashes the G-code file fresh from disk by content
        (SHA-256), not mtime -- a hand-edited-then-reverted file with an
        unchanged mtime still keys correctly. user_frame is the full 4x4
        build-plate pose, not just precompute_R_target's 3x3 rotation --
        waypoint XYZ positions are baked through T_user_frame's translation
        too (build_toolpath_waypoints_world). Rounded to 6 decimals to
        absorb float noise from repeated matrix ops without causing
        false-negative cache misses."""
        with open(os.path.join(GCODE_DIR, GCODE_FILE), "rb") as f:
            gcode_sha256 = hashlib.sha256(f.read()).hexdigest()
        return {
            "version": PRECOMPUTE_CACHE_VERSION,
            "gcode_sha256": gcode_sha256,
            "user_frame": np.round(np.asarray(user_frame, dtype=float), 6).tolist(),
            # The ground toggle changes which IK branch is accepted, so the
            # solved joint path depends on it -- roadmap 6.6.
            "reject_below_ground": self.reject_below_ground,
        }


    def _curved_toolpath_cache_meta(self, layer, waypoints, R_target_array, user_frame):
        """Cache-key dict for a curved-layer precompute -- roadmap 6.5, the
        curved analogue of _toolpath_cache_meta. There's no single curved
        source file to hash (unlike the one G-code file), so this hashes the
        *derived* arrays that actually drive the solve -- waypoint positions,
        feed flags, and per-waypoint orientations -- rounded to 6dp to absorb
        float noise. Any re-order or re-orient changes that hash and correctly
        invalidates the cache. user_frame is folded in the same way as the
        planar meta so a plate move is also caught."""
        positions = np.array([p for p, _ in waypoints], dtype=float)
        feed = np.array([f for _, f in waypoints], dtype=bool)
        h = hashlib.sha256()
        h.update(np.round(positions, 6).tobytes())
        h.update(feed.tobytes())
        h.update(np.round(np.asarray(R_target_array, dtype=float), 6).tobytes())
        return {
            "version": PRECOMPUTE_CACHE_VERSION,
            "layer_name": self.curved_layer_names[layer],
            "curve_sha256": h.hexdigest(),
            "user_frame": np.round(np.asarray(user_frame, dtype=float), 6).tolist(),
            # The ground toggle changes which IK branch is accepted, so the
            # solved joint path depends on it -- roadmap 6.6.
            "reject_below_ground": self.reject_below_ground,
        }


    def save_toolpath_precompute_cache(self, cache_path=GCODE_PRECOMPUTE_CACHE):
        """Best-effort write of a just-completed precompute to cache_path,
        tagged with the key captured at precompute-start
        (self.precompute_cache_meta) -- roadmap Stage5_README.md 5.10 (planar,
        default path), 6.5 (curved, per-layer path). Called only from
        step_toolpath_ik_precompute()'s successful-completion branch, never on
        an aborted/cancelled precompute. Wrapped in a bare except: a cache-write
        failure (disk full, permissions) must never surface as a failure of the
        precompute itself, which already succeeded in memory."""
        try:
            np.savez(
                cache_path,
                joint_path=np.asarray(self.precompute_joint_path, dtype=np.float32),
                meta=np.array(json.dumps(self.precompute_cache_meta)))
        except Exception:
            pass


    def load_toolpath_precompute_cache(self, cache_path=GCODE_PRECOMPUTE_CACHE, meta_builder=None):
        """Attempt to load a previously-saved precompute from cache_path instead
        of re-solving -- roadmap Stage5_README.md 5.10 (planar), 6.5 (curved).
        Compares the cached meta by dict equality against the key from
        meta_builder() (default: the planar _toolpath_cache_meta from the live
        T_user_frame; the curved caller passes its own already-computed meta).
        Any mismatch (different source, moved plate, version bump) or any error
        (missing cache file, corrupt npz, missing source file) is treated as a
        plain cache miss -- fails open, letting the caller fall through to the
        normal parse/solve path; never raises. No explicit os.path.exists gate:
        np.load and meta_builder() both raise on a missing file and the broad
        except below turns that into a clean miss."""
        if meta_builder is None:
            meta_builder = lambda: self._toolpath_cache_meta(self.T_user_frame)
        try:
            cached = np.load(cache_path, allow_pickle=False)
            cached_meta = json.loads(cached["meta"].item())
            if cached_meta != meta_builder():
                return False
            joint_path = cached["joint_path"].astype(float)
        except Exception:
            return False

        self.precompute_joint_path = list(joint_path)
        self.precompute_index = len(joint_path)
        self.precompute_total = len(joint_path)
        self.precompute_running = False
        self.precompute_status = f"Loaded {len(joint_path)} waypoint(s) from cache"
        # Record the pose this load matched, so a later plate move can be
        # detected as staleness (roadmap 5.11, settled.md S1.22) even though
        # this path skipped run_toolpath_ik_precompute()'s own snapshot.
        self.precompute_cache_meta = cached_meta
        # Also record which cache this came from -- roadmap 6.6's toolpath-
        # source tracking (the layer-mixup guard, and _init_toolpath_playback()'s
        # / _init_curved_toolpath_playback()'s source gates) all identify "who
        # owns precompute_joint_path" via precompute_cache_path, and a cache
        # hit is a legitimate way for that path to get populated, not just a
        # fresh chunked solve.
        self.precompute_cache_path = cache_path
        return True


    def _begin_toolpath_precompute(self, waypoints, R_target_array, joint_limits,
                                   reference_joint_angles, cache_meta, cache_path,
                                   tip_tolerance_mm=None):
        """Load a freshly-built waypoint source into precompute state -- the
        shared seam behind run_toolpath_ik_precompute (planar) and
        run_curved_toolpath_ik_precompute (curved), roadmap 6.5. R_target_array
        is (N,3,3), one target orientation per waypoint (settled.md S1.12's
        constant becomes a per-waypoint array). tip_tolerance_mm None keeps the
        z=0 clearance check; a value switches step_ to the per-waypoint tangent-
        plane check. cache_path is where a completed solve is written."""
        self.precompute_waypoints = waypoints
        self.precompute_R_target = R_target_array
        self.precompute_joint_limits = joint_limits
        self.precompute_index = 0
        self.precompute_total = len(waypoints)
        self.precompute_joint_path = []
        self.precompute_ref = (
            reference_joint_angles if reference_joint_angles is not None else self.current_joint_angles)
        self.precompute_cache_meta = cache_meta
        self.precompute_cache_path = cache_path
        self.precompute_tip_tolerance_mm = tip_tolerance_mm


    def run_toolpath_ik_precompute(self, joint_limits, reference_joint_angles=None):
        """Start or resume the chunked toolpath IK precompute -- roadmap
        Stage5_README.md 5.6. Mirrors the GUI's playback Run button
        (gui_panel.py): sets precompute_running True, and only (re-)parses
        the G-code and resets progress if nothing is loaded yet
        (precompute_waypoints is None -- true on the very first call, and
        again after cancel_toolpath_ik_precompute()). If a precompute is
        already loaded (i.e. paused), just resumes stepping from
        precompute_index -- no re-parsing, no restart. Reads the fixed
        G-code path (GCODE_DIR/GCODE_FILE, same convention as load_gcode())
        via parse_gcode() + build_toolpath_waypoints_world(), not a cached
        result from load_gcode() (which only keeps extruding-G1 preview
        beads, not the raw 1:1 waypoint list).

        The cache key (_toolpath_cache_meta) only hashes the G-code file, not a
        full parse, so it's computed up front and the cache is checked *before*
        parsing -- a hit skips the 187k-line parse and IK entirely (roadmap
        Stage5_README.md 5.10). parse_gcode runs on the miss path only.

        Layer-mixup guard (roadmap 6.6): if a curved-layer precompute is
        currently loaded (paused or mid-run), force-cancel it first rather
        than silently resuming it -- run_curved_toolpath_ik_precompute()'s
        fresh-start branch only fires when precompute_waypoints is None, so
        without this guard switching the active toolpath source wouldn't be
        noticed here.
        """
        if self.precompute_waypoints is not None and self.precompute_cache_path != GCODE_PRECOMPUTE_CACHE:
            self._abort_toolpath_ik_precompute()
            self.precompute_status = "Switched to planar toolpath -- previous precompute cancelled"

        if self.precompute_waypoints is None:
            filepath = os.path.join(GCODE_DIR, GCODE_FILE)
            if not os.path.exists(filepath):
                self.precompute_status = "No G-code file found"
                return

            # The G-code file can be overwritten mid-read by a Cura re-export
            # (settled.md notes model.gcode "gets overwritten by each new Cura
            # export") -- guard both the cheap hash and the full parse, failing
            # closed with a status message rather than letting the exception
            # escape the per-frame Polyscope callback.
            try:
                cache_meta = self._toolpath_cache_meta(self.T_user_frame)
            except OSError:
                self.precompute_status = "G-code file changed while loading -- try again"
                return

            if self.load_toolpath_precompute_cache(GCODE_PRECOMPUTE_CACHE, lambda: cache_meta):
                return

            try:
                gcode_points = self.parse_gcode(filepath)
                waypoints, R_target = self.build_toolpath_waypoints_world(gcode_points)
            except OSError:
                self.precompute_status = "G-code file changed while loading -- try again"
                return
            if not waypoints:
                self.precompute_status = "No waypoints to solve"
                return

            # The plate doesn't tilt mid-print (settled.md S1.6/S1.8), so the one
            # R_target applies to every waypoint -- broadcast to the (N,3,3) shape
            # step_ now indexes, a read-only view with no extra allocation.
            R_target_array = np.broadcast_to(R_target, (len(waypoints), 3, 3))
            self._begin_toolpath_precompute(
                waypoints, R_target_array, joint_limits, reference_joint_angles,
                cache_meta, cache_path=GCODE_PRECOMPUTE_CACHE, tip_tolerance_mm=None)

        self.precompute_running = True
        self.precompute_status = f"Precomputing {self.precompute_index}/{self.precompute_total} waypoints"


    def run_curved_toolpath_ik_precompute(self, layer, joint_limits, reference_joint_angles=None):
        """Start or resume the chunked IK precompute for a curved print layer --
        roadmap 6.5, the curved sibling of run_toolpath_ik_precompute. Gated on
        curved_orient_loaded so it can't run ahead of the 6.1-6.4 pipeline
        (load -> geodesics -> print order -> orientation frames). Feeds the
        shared precompute machinery from build_curved_toolpath_waypoints_world
        instead of a G-code parse, with a per-waypoint R_target and the
        tangent-plane clearance tolerance.

        Unlike the G-code path, the cache is checked *after* rebuilding
        waypoints: there's no single source file to hash cheaply, and rebuilding
        from the already-in-memory retained arrays is cheap (one
        nearest_vertex_index over a few thousand points), so there's no parse to
        avoid on a hit.

        Layer-mixup guard (roadmap 6.6): if a *different* source (the planar
        path, or a different curved layer) is currently loaded, force-cancel
        it first -- otherwise this would silently resume that stale run
        instead of starting layer's, since the fresh-start branch below only
        fires when precompute_waypoints is None."""
        if self.precompute_waypoints is not None:
            intended_cache_path = curved_precompute_cache_path(self.curved_layer_names[layer])
            if self.precompute_cache_path != intended_cache_path:
                self._abort_toolpath_ik_precompute()
                self.precompute_status = f"Switched to {self.curved_layer_names[layer]} -- previous precompute cancelled"

        if self.precompute_waypoints is None:
            if not self.curved_orient_loaded:
                self.precompute_status = "Build orientation frames first"
                return
            waypoints, R_target_array = self.build_curved_toolpath_waypoints_world(layer)
            if not waypoints:
                self.precompute_status = "No waypoints to solve"
                return
            cache_path = curved_precompute_cache_path(self.curved_layer_names[layer])
            cache_meta = self._curved_toolpath_cache_meta(layer, waypoints, R_target_array, self.T_user_frame)
            if self.load_toolpath_precompute_cache(cache_path, lambda: cache_meta):
                return
            self._begin_toolpath_precompute(
                waypoints, R_target_array, joint_limits, reference_joint_angles,
                cache_meta, cache_path=cache_path,
                tip_tolerance_mm=CURVED_TIP_CLEARANCE_TOLERANCE_MM)

        self.precompute_running = True
        self.precompute_status = f"Precomputing {self.precompute_index}/{self.precompute_total} waypoints"


    def run_active_toolpath_ik_precompute(self, joint_limits, reference_joint_angles=None):
        """Single dispatch entry point for the GUI's one shared Run Precompute
        button (roadmap 6.6) -- routes on toolpath_source so gui_panel.py
        doesn't need to know the planar/curved entry points take different
        arguments."""
        if self.toolpath_source == -1:
            self.run_toolpath_ik_precompute(joint_limits, reference_joint_angles)
        else:
            self.run_curved_toolpath_ik_precompute(self.toolpath_source, joint_limits, reference_joint_angles)


    def pause_toolpath_ik_precompute(self):
        """Mirrors the GUI's playback Pause button: stop advancing the
        precompute without discarding progress. A following
        run_toolpath_ik_precompute() call continues from precompute_index."""
        self.precompute_running = False


    def cancel_toolpath_ik_precompute(self):
        """Stop and discard the precompute entirely, resetting progress to
        zero -- a following run_toolpath_ik_precompute() call starts fresh."""
        self._abort_toolpath_ik_precompute()
        self.precompute_status = ""


    def _abort_toolpath_ik_precompute(self):
        """Shared discard used by cancel_toolpath_ik_precompute(),
        step_toolpath_ik_precompute()'s failure branches, and the
        layer-mixup guards in run_toolpath_ik_precompute()/
        run_curved_toolpath_ik_precompute() (roadmap 6.6) -- resets all
        precompute progress (precompute_index/total included, so a stale
        index can't outlive the joint path it counted) and playback state,
        since playback indexes precompute_joint_path directly and can't be
        left pointing at a joint path this just emptied. Does not touch
        precompute_status, so a caller can set an explanatory message
        first.

        Reads precompute_cache_path BEFORE clearing it to decide whether the
        run being discarded was the planar one -- only then is the G-code
        print mesh torn down (_clear_gcode_print_mesh()). Curved per-layer
        bead meshes are deliberately NOT cleared here: they must persist
        across switching the active toolpath source (roadmap 6.6's S1.32
        stack rule -- a completed layer's printed mesh stays visible while a
        different layer's precompute is discarded/restarted); only
        clear_curved_model() or a re-order/re-orient cascade
        (_abort_geodesic_precompute()) removes those."""
        was_gcode = self.precompute_cache_path in (None, GCODE_PRECOMPUTE_CACHE)
        self.playback_running = False
        self.playback_index = 0
        self.playback_waiting = False
        self.playback_status = ""
        if was_gcode:
            self._clear_gcode_print_mesh()

        self.precompute_running = False
        self.precompute_waypoints = None
        self.precompute_index = 0
        self.precompute_total = 0
        self.precompute_joint_path = []
        self.precompute_cache_meta = None
        # Clear the curved-run markers too, so cancelling a curved precompute and
        # starting the planar one can't carry over the wrong cache path or the
        # tangent-plane tolerance (roadmap 6.5).
        self.precompute_cache_path = None
        self.precompute_tip_tolerance_mm = None


    def step_toolpath_ik_precompute(self):
        """Advance the in-progress precompute by up to PRECOMPUTE_CHUNK_SIZE
        waypoints -- call every frame from render(). No-ops unless
        precompute_running. Uses the same per-waypoint solve + ground-
        clearance logic as solve_toolpath_ik, and aborts the whole
        precompute (no partial motion) at the first waypoint with no
        valid/clearing branch."""
        if not self.precompute_running:
            return

        end = min(self.precompute_index + PRECOMPUTE_CHUNK_SIZE, self.precompute_total)
        for i in range(self.precompute_index, end):
            pos_world_mm, _is_feed_move = self.precompute_waypoints[i]
            R_i = self.precompute_R_target[i]
            solutions, status = self.solve_ik_tcp_matrix(
                pos_world_mm, R_i, self.precompute_joint_limits,
                reference_joint_angles=self.precompute_ref)
            if not solutions:
                status_msg = f"Waypoint {i}/{self.precompute_total}: {status}"
                self._abort_toolpath_ik_precompute()
                self.precompute_status = status_msg
                return

            # Planar run (tip_tolerance None): the original z=0 clearance check.
            # Curved run: this waypoint's own outward tangent plane (point = the
            # waypoint, normal = R_i's Z column), the supporting-hyperplane
            # clearance of settled.md S1.37.
            plane = (None if self.precompute_tip_tolerance_mm is None
                     else (pos_world_mm, R_i[:, 2], self.precompute_tip_tolerance_mm))
            clear = next((angles for angles, *_ in solutions
                          if self._branch_clears_ground(angles, plane)), None)
            if clear is None:
                # Name the checks actually active (roadmap 6.6): the z=0 ground
                # check is toggle-gated, the tangent plane is curved-only.
                checks = []
                if self.reject_below_ground:
                    checks.append("the ground plane (z<0)")
                if plane is not None:
                    checks.append("their surface tangent plane")
                status_msg = (
                    f"Waypoint {i}/{self.precompute_total}: all {len(solutions)} valid branch(es) "
                    f"hit {' or '.join(checks)}")
                self._abort_toolpath_ik_precompute()
                self.precompute_status = status_msg
                return

            self.precompute_ref = clear
            self.precompute_joint_path.append(clear)

        self.precompute_index = end
        if self.precompute_index >= self.precompute_total:
            self.precompute_running = False
            self.precompute_status = f"Solved {self.precompute_total} waypoint(s)"
            self.save_toolpath_precompute_cache(self.precompute_cache_path)
        else:
            self.precompute_status = f"Precomputing {self.precompute_index}/{self.precompute_total} waypoints"


    def _clear_gcode_print_mesh(self):
        """The G-code-specific slice of playback teardown -- bead arrays,
        registered mesh, and the preview/playback ownership flag. Shared by
        _reset_toolpath_playback_state() (clear_gcode_preview()'s
        unconditional reset -- Clear always means "wipe G-code", regardless
        of what else is active) and _abort_toolpath_ik_precompute() (only
        when G-code was actually the source being discarded, roadmap 6.6) --
        split out so those two call sites can apply it under different
        conditions without duplicating the four lines."""
        self.gcode_bead_verts_full = None
        self.gcode_print_handle = None
        self.gcode_preview_loaded = False
        ps.remove_surface_mesh("G-code Print", error_if_absent=False)


    def _reset_toolpath_playback_state(self):
        """Playback reset used only by clear_gcode_preview() -- unconditionally
        discards G-code's own playback/bead state and the shared playback
        pointer, regardless of which toolpath source is currently active,
        since the Clear button's whole point is "wipe G-code now"."""
        self.playback_running = False
        self.playback_index = 0
        self.playback_waiting = False
        self.playback_status = ""
        self._clear_gcode_print_mesh()


    def _init_toolpath_playback(self):
        """Shared setup for reset_toolpath_playback() and the first
        run_toolpath_playback() call this session. Requires a completed
        precompute; re-parses the G-code and rebuilds bead geometry via
        _build_gcode_beads() (not load_gcode(), which doesn't return
        reveal_waypoint_index). Collapses every bead to its own first
        corner (zero-area, nothing renders) and registers only the first
        PLAYBACK_LOOKAHEAD_BEADS beads' worth, not the full mesh. Snaps
        the arm to the first waypoint's pose. Returns True on success,
        False (with playback_status explaining why) otherwise.

        Guards precompute_cache_path too (roadmap 6.6), not just emptiness:
        without it, switching toolpath_source to Planar while a curved
        precompute_joint_path is still loaded would build G-code beads
        against the wrong (curved) joint angles."""
        if not self.precompute_joint_path:
            self.playback_status = "Run Precompute first"
            return False
        if self.precompute_cache_path not in (None, GCODE_PRECOMPUTE_CACHE):
            self.playback_status = "Run Precompute for the planar toolpath first"
            return False

        filepath = os.path.join(GCODE_DIR, GCODE_FILE)
        if not os.path.exists(filepath):
            self.playback_status = "No G-code file found"
            return False

        try:
            gcode_points = self.parse_gcode(filepath)
        except OSError:
            # File can be overwritten mid-read by a Cura re-export between
            # the exists() check above and here.
            self.playback_status = "G-code file changed while loading -- try again"
            return False
        verts_world, faces, reveal_index, face_prefix = self._build_gcode_beads(gcode_points)
        if len(verts_world) == 0:
            self.playback_status = "No printed beads to reveal"
            return False

        self.gcode_bead_verts_full = verts_world
        self.gcode_bead_faces = faces
        self.gcode_bead_reveal_index = reveal_index
        self.gcode_bead_face_prefix = face_prefix

        # Collapse every bead to its own first corner -- a zero-area box
        # renders nothing, revealed later by restoring real positions
        # (advance_toolpath_playback), never via transparency (settled.md S1.16).
        self.gcode_bead_verts_current = np.repeat(verts_world[0::8], 8, axis=0)

        K = len(reveal_index)
        self._registered_bead_capacity = min(PLAYBACK_LOOKAHEAD_BEADS, K)
        # From here "G-code Print" belongs to playback's reveal, not the
        # static preview -- the Clear button should stay hidden until an
        # explicit Load click claims the mesh again.
        self.gcode_preview_loaded = False
        self.gcode_print_handle = ps.register_surface_mesh(
            "G-code Print",
            self.gcode_bead_verts_current[:self._registered_bead_capacity * 8],
            self.gcode_bead_faces[:self.gcode_bead_face_prefix[self._registered_bead_capacity]])
        self.gcode_print_handle.set_color(GCODE_COLOR)

        self.playback_index = 0
        self._last_rendered_playback_index = 0
        self.update_arm(self.precompute_joint_path[0])
        return True


    def reset_toolpath_playback(self):
        """Mirrors the GUI's playback Reset button: snaps to the first pose
        and empties the shape (roadmap Stage5_README.md 5.7) -- always a
        full re-init, discarding any in-progress reveal. Dispatches on
        toolpath_source (roadmap 6.6) -- re-inits the planar path or the
        active curved layer, whichever is currently selected; a different,
        already-completed curved layer's printed mesh is untouched."""
        self.playback_running = False
        self.playback_active = False  # roadmap 6.7 -- Reset restores the full guide view.
        ok = (self._init_toolpath_playback() if self.toolpath_source == -1
              else self._init_curved_toolpath_playback(self.toolpath_source))
        if ok:
            self.playback_status = "Ready to play"
        self.apply_live_layer_visibility(self.toolpath_source)  # restore overlays (curved; planar no-op)


    def run_toolpath_playback(self):
        """Mirrors the GUI's playback Run button: start or resume. If
        playback was never initialized this session (or was reset),
        initializes fresh; otherwise resumes from wherever playback_index
        already is (a paused run continues, not restarts). Dispatches on
        toolpath_source (roadmap 6.6) -- the planar path or a specific
        curved layer, without duplicating this Run/Pause/Reset control set."""
        if self.toolpath_source == -1:
            if self.gcode_bead_verts_full is None:
                if not self._init_toolpath_playback():
                    return
        else:
            layer = self.toolpath_source
            if self.curved_bead_verts_full is None or self.curved_bead_verts_full[layer] is None:
                if not self._init_curved_toolpath_playback(layer):
                    return
        self.playback_running = True
        self.playback_active = True  # roadmap 6.7 -- survives Pause, cleared only by Reset.
        # Hide the guide overlays on the click so the growing beads are visible. Planar (-1) is a
        # safe no-op: apply_live_layer_visibility early-returns unless curved_model_loaded.
        self.apply_live_layer_visibility(self.toolpath_source)


    def pause_toolpath_playback(self):
        """Mirrors the GUI's playback Pause button: stop advancing without
        discarding progress. A following run_toolpath_playback() call
        continues from playback_index."""
        self.playback_running = False


    def advance_toolpath_playback(self, step_count):
        """Advance playback by up to step_count waypoints -- call every
        frame from render(). No-ops unless playback_running. The index
        always advances every call; the Polyscope push (arm pose + bead
        reveal) is throttled to every PLAYBACK_RENDER_STRIDE waypoints,
        forced on the final one so playback never ends on a stale
        mid-stride pose. Beads reveal via a sorted cutoff over
        gcode_bead_reveal_index, accumulated from the last *rendered*
        index so none are skipped across throttled frames. The
        registered mesh grows in PLAYBACK_LOOKAHEAD_BEADS chunks instead
        of registering the full mesh from frame 1.

        Playback may start before precompute finishes: the advance is
        capped at the live frontier (len(precompute_joint_path)), not a
        snapshot. If playback catches the frontier before precompute is
        exhausted, it holds there with a "Waiting for precompute" status
        and playback_running stays True so the next frame rechecks the
        frontier automatically. self.playback_waiting mirrors this state
        -- gui_panel.py reads it to snap the Speed slider down the
        moment playback actually hits the compute limit.

        Dispatches on toolpath_source (roadmap 6.6): resolves which bead
        arrays/structure name/color/capacity to reveal into once at the top,
        then runs the same reveal math either way. verts_current is the same
        array object as gcode_bead_verts_current/curved_bead_verts_current[layer],
        so the in-place slice assignment below still mutates the real
        backing array through the alias -- only the two scalar/handle fields
        (registered capacity, Polyscope handle) need an explicit write-back,
        done at the bottom since they aren't mutated in place."""
        if not self.playback_running:
            return

        frontier = len(self.precompute_joint_path)
        if self.playback_index >= frontier:
            # precompute_joint_path shrank or emptied under an active
            # playback -- refuse cleanly instead of an IndexError below.
            self.playback_running = False
            self.playback_waiting = False
            self.playback_status = "Toolpath data changed -- reset playback"
            return

        new_index = min(self.playback_index + step_count, frontier - 1)
        moved = new_index != self.playback_index
        self.playback_index = new_index

        exhausted = self.precompute_index >= self.precompute_total
        at_frontier = self.playback_index >= frontier - 1
        finished = exhausted and at_frontier
        waiting = at_frontier and not exhausted
        self.playback_waiting = waiting

        if finished or (waiting and moved) or self.playback_index - self._last_rendered_playback_index >= PLAYBACK_RENDER_STRIDE:
            self.update_arm(self.precompute_joint_path[self.playback_index])

            curved = self.toolpath_source != -1
            layer = self.toolpath_source
            if curved:
                bead_faces = self.curved_bead_faces[layer]
                reveal_index = self.curved_bead_reveal_index[layer]
                face_prefix = self.curved_bead_face_prefix[layer]
                bead_verts_full = self.curved_bead_verts_full[layer]
                verts_current = self.curved_bead_verts_current[layer]
                structure_name = f"Curved Print {self.curved_layer_names[layer]}"
                color = CURVED_LAYERS[layer]["curve_color"]
                capacity = self.curved_bead_registered_capacity[layer]
            else:
                bead_faces = self.gcode_bead_faces
                reveal_index = self.gcode_bead_reveal_index
                face_prefix = self.gcode_bead_face_prefix
                bead_verts_full = self.gcode_bead_verts_full
                verts_current = self.gcode_bead_verts_current
                structure_name = "G-code Print"
                color = GCODE_COLOR
                capacity = self._registered_bead_capacity

            old_revealed = np.searchsorted(reveal_index, self._last_rendered_playback_index, side='right')
            new_revealed = np.searchsorted(reveal_index, self.playback_index, side='right')
            if new_revealed > old_revealed:
                verts_current[old_revealed * 8:new_revealed * 8] = \
                    bead_verts_full[old_revealed * 8:new_revealed * 8]

                K = len(reveal_index)
                if finished or new_revealed >= capacity:
                    target_capacity = K if finished else min(new_revealed + PLAYBACK_LOOKAHEAD_BEADS, K)
                    handle = ps.register_surface_mesh(
                        structure_name,
                        verts_current[:target_capacity * 8],
                        bead_faces[:face_prefix[target_capacity]])
                    handle.set_color(color)
                    if curved:
                        self.curved_bead_registered_capacity[layer] = target_capacity
                        self.curved_print_handle[layer] = handle
                    else:
                        self._registered_bead_capacity = target_capacity
                        self.gcode_print_handle = handle
                else:
                    handle = self.curved_print_handle[layer] if curved else self.gcode_print_handle
                    handle.update_vertex_positions(verts_current[:capacity * 8])

            self._last_rendered_playback_index = self.playback_index

        if finished:
            self.playback_running = False
            self.playback_status = "Playback complete"
        elif waiting:
            self.playback_status = f"Waiting for precompute ({frontier}/{self.precompute_total} solved)"
        else:
            self.playback_status = f"Playing {self.playback_index}/{self.precompute_total - 1}"


    def load_mesh(self, filepath):
        """Load a single OBJ mesh with trimesh.

        force='mesh' collapses multi-group OBJs into one Trimesh; without it,
        trimesh.load can return a Scene (no .vertices/.faces -> AttributeError
        later). See docs/Polyscope_Quickstart.md.
        """
        return trimesh.load(filepath, force='mesh')


    def load_data(self):
        """Load all 7 FR5 link meshes (Robot0..Robot6), register them with
        Polyscope, and cache the zero-pose data needed for the Delta
        transform (see docs/FR5_Mesh_Convention.md).

        Returns a list of trimesh.Trimesh, index i == Robot{i}.obj. Robot0 is the
        static base; Robot1..Robot6 correspond to T_0_1..T_0_6 from compute_fk().
        """
        meshes = [self.load_mesh(os.path.join(MESH_DIR, fname)) for fname in MESH_FILES]

        # Compute zero-pose transforms
        zero_joints = [0, 0, 0, 0, 0, 0]
        self.T_zero = self.compute_fk(zero_joints)
        self.T_zero_inv = [np.linalg.inv(T) for T in self.T_zero]  # cached once,
        # not per-frame -- apply_delta_transform()/_moving_geometry_deltas() run every frame

        # Store the rest-pose vertices for each list
        self.rest_verts = [m.vertices.copy() for m in meshes[1:]]  # List of Nx3 arrays, one per link (Robot1..Robot6)
        self.mesh_handles = []
        self.update_fns = []  # matching per-object position-update method (meshes vs point clouds differ)

        ps.register_surface_mesh("Robot0", meshes[0].vertices, meshes[0].faces)  # Fixed base, no transforms

        for i, m in enumerate(meshes[1:]):
            handle = ps.register_surface_mesh(f"Robot{i+1}", m.vertices, m.faces)
            self.mesh_handles.append(handle)
            self.update_fns.append(handle.update_vertex_positions)

        # Nozzle rides on the flange -- same Delta_6 as Robot6, see docs/FR5_Mesh_Convention.md
        nozzle = self.load_mesh(os.path.join(PRINTER_HEAD_DIR, NOZZLE_FILE))
        self.rest_verts.append(nozzle.vertices.copy())
        nozzle_handle = ps.register_surface_mesh("Nozzle", nozzle.vertices, nozzle.faces)
        self.mesh_handles.append(nozzle_handle)
        self.update_fns.append(nozzle_handle.update_vertex_positions)

        # Zero-pose bbox corners for the moving-geometry set (Robot1..6 + nozzle,
        # rest_verts[0:7] -- excludes the TCP point/frame appended below, which
        # are visualization markers, not solid robot geometry). See
        # moving_geometry_bbox_min_z (roadmap Stage5_README.md 5.5).
        self.moving_geometry_rest_bbox_corners = [_bbox_corners(v) for v in self.rest_verts]

        self.tcp_local = np.loadtxt(os.path.join(PRINTER_HEAD_DIR, TCP_FILE))  # Zero-pose world frame [x, y, z]

        # Fixed flange->TCP transform for IK; rotation comes from inv(T_zero[5]),
        # not assumed identity -- see settled.md S1.4 for why.
        T_zero_flange_inv = self.T_zero_inv[5]
        self.T_flange_to_tcp = T_zero_flange_inv.copy()
        self.T_flange_to_tcp[:3, 3] = (T_zero_flange_inv @ np.append(self.tcp_local, 1.0))[:3]

        # TCP point, also Delta_6, but a Polyscope point cloud -- update_point_positions,
        # not update_vertex_positions, hence the per-object self.update_fns lookup
        tcp_point = self.tcp_local.reshape(1, 3)
        self.rest_verts.append(tcp_point)
        point_cloud = ps.register_point_cloud("TCP", tcp_point)
        self.mesh_handles.append(point_cloud)
        self.update_fns.append(point_cloud.update_point_positions)

        # TCP orientation triad, also Delta_6 -- axis tips defined in the zero-pose
        # world frame around tcp_local, so they rotate with the tool via the same
        # Delta transform (curve network -> update_node_positions)
        tcp_frame_handle, tcp_frame_rest_nodes = self.create_coordinate_frame(
            scale=TCP_FRAME_SCALE_MM, origin=self.tcp_local, name="TCP Frame")
        self.rest_verts.append(tcp_frame_rest_nodes)
        self.mesh_handles.append(tcp_frame_handle)
        self.update_fns.append(tcp_frame_handle.update_node_positions)

        return meshes


    def apply_delta_transform(self, joint_angles_deg):
        """Update link mesh vertex positions for the given joint angles.

        Delta_i = T_0_i(q) @ inv(T_0_i(0)) -- see docs/FR5_Mesh_Convention.md.
        Robot0 is the fixed base and is never updated. The nozzle (index 6),
        TCP point (index 7), and TCP frame (index 8) ride on the flange,
        reusing Delta_6 (index 5).
        """
        T_current = self.compute_fk(joint_angles_deg)
        for i in range(9):
            src = min(i, 5)
            Delta = T_current[src] @ self.T_zero_inv[src]

            # Convert rest verts to homogenous [x,y,z,1]
            N = self.rest_verts[i].shape[0]
            homo = np.hstack([self.rest_verts[i], np.ones((N, 1))])

            # Apply delta
            new_verts = (Delta @ homo.T).T[:, :3]

            # Update Polyscope structure (mesh or point cloud, per registration)
            self.update_fns[i](new_verts)

            if i == 7:
                self.tcp_world = new_verts[0]


    def update_arm(self, joint_angles_deg):
        """GUI-facing entry point: record the current joint state and move
        the rendered arm to match via the Delta transform."""
        self.current_joint_angles = joint_angles_deg
        self.apply_delta_transform(joint_angles_deg)


    def _moving_geometry_deltas(self, joint_angles_deg):
        """Delta_i for each moving mesh (Robot1..Robot6 + nozzle), reusing
        Delta_6 for the nozzle -- same src = min(i, 5) mapping as
        apply_delta_transform, but pure computation with no Polyscope side
        effects (used by the ground-clearance checks below, not per-frame
        rendering)."""
        T_current = self.compute_fk(joint_angles_deg)
        return [T_current[min(i, 5)] @ self.T_zero_inv[min(i, 5)] for i in range(7)]


    def moving_geometry_bbox_min_z(self, joint_angles_deg):
        """Cheap ground-clearance pre-check (roadmap Stage5_README.md 5.5):
        transform each moving mesh's cached zero-pose bounding-box corners (8
        per mesh, not the full vertex set) by its Delta transform and return
        the minimum world z reached, mm. This is a guaranteed lower bound on
        moving_geometry_min_z's result -- a rigid transform of an AABB's 8
        corners always produces a convex hull enclosing the mesh's true
        transformed extent, and z is linear so its minimum is attained at a
        corner -- so a non-negative result here proves the branch clears
        without needing the exact check."""
        deltas = self._moving_geometry_deltas(joint_angles_deg)
        min_z = np.inf
        for delta, corners in zip(deltas, self.moving_geometry_rest_bbox_corners):
            homo = np.hstack([corners, np.ones((len(corners), 1))])
            world = (delta @ homo.T).T[:, :3]
            min_z = min(min_z, world[:, 2].min())
        return min_z


    def moving_geometry_min_z(self, joint_angles_deg):
        """Exact ground-clearance check (roadmap Stage5_README.md 5.5):
        transform every vertex of every moving mesh and return the true
        minimum world z reached, mm. Slower than moving_geometry_bbox_min_z --
        only called when the bbox check doesn't already prove clearance."""
        deltas = self._moving_geometry_deltas(joint_angles_deg)
        min_z = np.inf
        for delta, verts in zip(deltas, self.rest_verts[:7]):
            homo = np.hstack([verts, np.ones((len(verts), 1))])
            world = (delta @ homo.T).T[:, :3]
            min_z = min(min_z, world[:, 2].min())
        return min_z


    def _nozzle_clears_plane(self, joint_angles_deg, point, normal, tip_tolerance_mm):
        """True if the nozzle mesh stays outward of the given plane, allowing
        tip_tolerance_mm of inward slack -- the surface-penetration half of the
        curved clearance check (roadmap 6.5, settled.md S1.37). The plane passes
        through `point` with unit outward `normal`; a vertex's signed distance
        is (world - point) @ normal, positive outward. The nozzle clears iff its
        worst (min) signed distance is >= -tip_tolerance_mm.

        Only the nozzle (moving-geometry index 6) is tested, NOT the arm links.
        The plane is a supporting hyperplane for the convex mockup stack, so a
        point on its outward side provably clears every surface behind it -- but
        that bounds where the *surface* is, not where the *arm* is. The arm must
        span from its base up to the contact point, so its lower links
        legitimately sit far *inward* of a local tangent plane; testing them
        would reject every real printing pose. The nozzle is the only part
        required to stay on the surface it prints. (Arm-vs-table clearance is
        handled separately by the retained world z=0 check.)

        Cheap 8-corner bbox bound first, exact vertices only if inconclusive:
        signed distance is linear, so its min over the rigid-transformed AABB
        corners is a lower bound on its min over the true mesh -- a non-negative
        corner result proves clearance."""
        delta = self._moving_geometry_deltas(joint_angles_deg)[6]
        for verts in (self.moving_geometry_rest_bbox_corners[6], self.rest_verts[6]):
            homo = np.hstack([verts, np.ones((len(verts), 1))])
            world = (delta @ homo.T).T[:, :3]
            if ((world - point) @ normal).min() + tip_tolerance_mm >= 0:
                return True
        return False


    def _branch_clears_ground(self, joint_angles_deg, plane=None):
        """True if this branch's moving geometry stays clear of its obstacle(s).

        Two independent checks, layered:

        1. **World z=0 ground check**, gated by the reject_below_ground toggle
           (roadmap 6.6). When enabled it applies to BOTH the planar and the
           curved path: cheap transformed-bbox bound first -- proven clear if
           non-negative -- escalating to the exact per-vertex min only when the
           bbox result is negative (inconclusive: a rotated AABB corner can dip
           below ground even when the real mesh does not, settled.md S1.13).
           Default ON = planar's historical always-reject behaviour; the user
           unchecks it for a low-plate/mockup setup where sub-z=0 arm poses are
           physically fine (the curved mockup sits above the plate in a frame
           where z=0 is not the physical floor, S1.37).

        2. **Tangent-plane nozzle check** (curved path only, plane not None):
           the nozzle tip must stay outward of that waypoint's own surface
           tangent plane, within tip_tolerance_mm of inward slack (roadmap 6.5,
           settled.md S1.37). Only the nozzle is checked (see
           _nozzle_clears_plane); full arm-vs-mockup collision would need a real
           obstacle-mesh check, the expensive path 6.5 deliberately avoided.

        With the toggle OFF and no plane (planar), nothing is rejected."""
        if self.reject_below_ground:
            if self.moving_geometry_bbox_min_z(joint_angles_deg) < 0:
                if self.moving_geometry_min_z(joint_angles_deg) < 0:
                    return False

        if plane is not None:
            point, normal, tip_tolerance_mm = plane
            return self._nozzle_clears_plane(joint_angles_deg, point, normal, tip_tolerance_mm)

        return True


    def record_trajectory_point(self):
        """Sample self.tcp_world at most once per TRAJECTORY_SAMPLE_INTERVAL_S;
        discard the sample if the TCP hasn't moved since the last recorded
        point. trajectory_points stays dense every accepted sample; only the
        Polyscope redraw (_update_trajectory_curve, a full re-registration)
        is throttled to every TRAJECTORY_CURVE_RENDER_STRIDE samples --
        roadmap Stage5_README.md 5.9 treats the curve as a decimatable debug
        overlay, not the exported path."""
        if not self.trajectory_enabled:
            return

        now = time.time()
        if now - self._last_sample_time < TRAJECTORY_SAMPLE_INTERVAL_S:
            return
        self._last_sample_time = now

        if self.trajectory_points and np.allclose(self.tcp_world, self.trajectory_points[-1]):
            return

        self.trajectory_points.append(self.tcp_world.copy())
        self._trajectory_curve_sample_count += 1
        if self._trajectory_curve_sample_count >= TRAJECTORY_CURVE_RENDER_STRIDE:
            self._trajectory_curve_sample_count = 0
            self._update_trajectory_curve()


    def _update_trajectory_curve(self):
        """Re-register the curve network -- Polyscope curve networks don't support
        growing node count in place, unlike update_vertex_positions."""
        nodes = np.array(self.trajectory_points)
        if len(nodes) < 2:
            return
        edges = np.array([[i, i + 1] for i in range(len(nodes) - 1)])
        self.trajectory_handle = ps.register_curve_network("Trajectory", nodes, edges)
        self.trajectory_handle.set_radius(TRAJECTORY_RADIUS_MM, relative=False)


    def set_trajectory_enabled(self, enabled):
        """Pause/resume TCP trajectory recording and show/hide the existing curve."""
        self.trajectory_enabled = enabled
        if self.trajectory_handle is not None:
            self.trajectory_handle.set_enabled(enabled)


    def clear_trajectory(self):
        """Discard recorded TCP trajectory points and remove the curve, if
        any -- emptying trajectory_points alone isn't enough, since
        _update_trajectory_curve() only re-registers when there are >=2
        points, which would otherwise leave the stale curve on screen."""
        self.trajectory_points = []
        self.trajectory_handle = None
        self._trajectory_curve_sample_count = 0
        ps.remove_curve_network("Trajectory", error_if_absent=False)



def _bbox_corners(verts):
    """The 8 corners of verts' axis-aligned bounding box, in whatever frame
    verts is already in. Used to cheaply bound a mesh's rotated extent
    without transforming every vertex -- see moving_geometry_bbox_min_z."""
    lo, hi = verts.min(axis=0), verts.max(axis=0)
    xs, ys, zs = np.meshgrid([lo[0], hi[0]], [lo[1], hi[1]], [lo[2], hi[2]], indexing='ij')
    return np.stack([xs.ravel(), ys.ravel(), zs.ravel()], axis=1)


def transform_points(T, points):
    """Apply a 4x4 homogeneous transform to an Nx3 point array."""
    homo = np.hstack([points, np.ones((len(points), 1))])
    return (T @ homo.T).T[:, :3]


def read_ply_polyline(filepath):
    """Read an ASCII PLY containing only `element vertex` + `element edge`
    (no faces) -- these reject trimesh.load(force='mesh'), which needs
    faces to produce anything but a degenerate empty mesh. Returns
    (verts: Nx3 float64, edges: Mx2 int) exactly as declared in the header;
    edges are a disjoint segment soup in file order, not a walkable curve
    -- see reconstruct_polylines()."""
    with open(filepath) as f:
        lines = f.readlines()
    n_vertex = n_edge = header_end = None
    for i, line in enumerate(lines):
        if line.startswith("element vertex"):
            n_vertex = int(line.split()[-1])
        elif line.startswith("element edge"):
            n_edge = int(line.split()[-1])
        elif line.startswith("end_header"):
            header_end = i + 1
            break
    v_end = header_end + n_vertex
    e_end = v_end + n_edge
    verts = np.loadtxt(lines[header_end:v_end], dtype=np.float64).reshape(n_vertex, 3)
    edges = np.loadtxt(lines[v_end:e_end], dtype=int).reshape(n_edge, 2)
    return verts, edges


def reconstruct_polylines(verts, edges, decimals=CURVE_DEDUPE_DECIMALS):
    """Reassemble a PLY's disjoint edge soup into walkable polyline pieces.
    Coordinates are deduped by rounding -- float export noise keeps true
    duplicate points apart past ~3dp (verified: RX_0.ply's 108 raw vertices
    collapse to exactly 54 nodes at 3dp). A few files also carry one
    degenerate zero-length edge whose endpoints round to the same node --
    dropped, it carries no path information.

    Every node has degree <= 2 (no branching, confirmed across all 55
    files) but not every component has a degree-1 endpoint: 6 files
    (RX_0/RX_22/RX_27/TX_17/TX_2/TX_6) are single closed loops. Open
    components are walked from a degree-1 node; any component with none
    left is a closed loop, walked from an arbitrary node back to itself.
    Verified lossless (every non-self-loop edge consumed exactly once)
    across all 55 files -- 70 pieces total, matching the asset survey.

    Returns a list of Nx3 float arrays, one per piece; closed pieces repeat
    their start point as the last row so every piece can be treated
    identically as a consecutive-edge stride (see _register_curve_layer)."""
    rounded = np.round(verts, decimals)
    uniq, inv = np.unique(rounded, axis=0, return_inverse=True)
    inv = inv.reshape(-1)

    adjacency = {i: set() for i in range(len(uniq))}
    for a, b in edges:
        ua, ub = int(inv[a]), int(inv[b])
        if ua == ub:
            continue
        adjacency[ua].add(ub)
        adjacency[ub].add(ua)

    visited_nodes = set()
    n_edges_consumed = 0

    def walk(start):
        nonlocal n_edges_consumed
        chain = [start]
        visited_nodes.add(start)
        prev, cur = None, start
        while True:
            nbrs = [n for n in adjacency[cur] if n != prev]
            if not nbrs:
                break
            nxt = nbrs[0]
            n_edges_consumed += 1
            chain.append(nxt)
            if nxt == start:
                break
            visited_nodes.add(nxt)
            prev, cur = cur, nxt
        return uniq[chain]

    pieces = [walk(n) for n, adj in adjacency.items() if len(adj) == 1 and n not in visited_nodes]
    pieces += [walk(n) for n in adjacency if n not in visited_nodes]

    n_unique_edges = len({(min(int(a), int(b)), max(int(a), int(b)))
                           for a, b in ((inv[a], inv[b]) for a, b in edges) if a != b})
    assert n_edges_consumed == n_unique_edges, "reconstruct_polylines dropped an edge"
    return pieces


def build_surface_graph(verts, faces):
    """Undirected edge graph of a triangle mesh -- roadmap 6.2 step 1. Nodes
    are mesh vertices, edges are triangle edges, weights are Euclidean edge
    lengths in whatever frame verts is given in (world mm, as called).

    Returns CSR-style (neighbor_start, neighbor_index, neighbor_weight):
    node u's neighbours are the slots neighbor_start[u]:neighbor_start[u+1].

    Two representation choices worth not re-litigating:

    Flat CSR rather than a list-of-lists adjacency, because Surface_TX_Base
    has 271,036 directed entries -- a list[list[tuple]] allocates 45,430 lists
    plus 271,036 tuples (tens of MB) through an interpreted build loop, where
    this builds fully vectorised in ~0.27s.

    Python lists rather than numpy arrays on return, which looks backwards
    but is measured: identical algorithm and layout, only the container type
    differs, and the bare Dijkstra loop runs ~139ms with numpy element
    indexing vs ~81ms with lists on Surface_TX_Base -- about 1.7x. Numpy
    boxes a fresh scalar on every element access; a list already holds native
    ints/floats. (dijkstra_surface() itself costs ~50ms RX / ~85ms TX, above
    the bare loop, because it also allocates prev and converts to numpy on
    return.) The .tolist() below costs ~11ms, once."""
    faces = np.asarray(faces)
    e = np.sort(np.vstack([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]]), axis=1)
    e = np.unique(e, axis=0)  # each undirected edge once, however many triangles share it
    both = np.vstack([e, e[:, ::-1]])
    both = both[np.argsort(both[:, 0], kind='stable')]

    counts = np.bincount(both[:, 0], minlength=len(verts))
    start = np.concatenate([[0], np.cumsum(counts)])
    weight = np.linalg.norm(np.asarray(verts)[both[:, 0]] - np.asarray(verts)[both[:, 1]], axis=1)
    return start.tolist(), both[:, 1].tolist(), weight.tolist()


def nearest_vertex_index(verts, query_points):
    """Snap each query point to the nearest mesh vertex -- roadmap 6.2 step 2.
    Returns (indices (Q,), distances (Q,)).

    Brute force, deliberately: there is no scipy in this environment for a
    KD-tree, and at this size none is needed -- the worst case here is 70
    endpoints against Surface_TX_Base's 45,430 vertices, a 25MB intermediate
    solved in ~109ms, once per precompute. Distances are returned rather than
    discarded because they're the evidence the snap is legitimate: measured
    max 0.684mm (RX) / 0.580mm (TX), comfortably inside the ~1.24mm median
    mesh edge, so every endpoint lands on a vertex of the triangle it sits
    over."""
    verts = np.asarray(verts)
    query_points = np.asarray(query_points)
    d2 = ((verts[None, :, :] - query_points[:, None, :]) ** 2).sum(-1)
    idx = np.argmin(d2, axis=1)
    return idx, np.sqrt(d2[np.arange(len(query_points)), idx])


def dijkstra_surface(neighbor_start, neighbor_index, neighbor_weight, source):
    """Single-source shortest paths over a build_surface_graph() CSR graph --
    roadmap 6.2 step 3. Hand-rolled heapq rather than a scipy dependency, per
    AGENTS.md's from-scratch principle; standard lazy-deletion Dijkstra.

    Returns (dist (V,) float64, prev (V,) int32). Unreachable nodes are
    np.inf / -1. prev[source] is source itself -- a self-loop, not -1,
    because the obvious -1 would make "I am the source" and "I am
    unreachable" indistinguishable and leave geodesic_path_nodes() unable to
    tell a valid termination from a broken walk."""
    n = len(neighbor_start) - 1
    dist = [float('inf')] * n
    prev = [-1] * n
    dist[source] = 0.0
    prev[source] = source

    heap = [(0.0, source)]
    push, pop = heapq.heappush, heapq.heappop  # bound locally, this loop runs ~V log V times
    while heap:
        d, u = pop(heap)
        if d > dist[u]:
            continue  # stale entry left behind by a relaxation
        for k in range(neighbor_start[u], neighbor_start[u + 1]):
            v = neighbor_index[k]
            nd = d + neighbor_weight[k]
            if nd < dist[v]:
                dist[v] = nd
                prev[v] = u
                push(heap, (nd, v))

    return np.array(dist, dtype=np.float64), np.array(prev, dtype=np.int32)


def geodesic_path_nodes(prev_row, target):
    """Reconstruct one geodesic as a vertex-id list from a stored
    dijkstra_surface() predecessor row -- source-first, both ends inclusive.
    Returns [target] when target is the source, and None when unreachable.

    A pure walk-back: this never re-runs Dijkstra, which is the whole reason
    the (S,V) prev rows are retained rather than just the cost matrix.

    Note for roadmap 6.3: the returned path begins and ends at *snapped mesh
    vertices*, not at the curve endpoints themselves -- measured median
    ~0.36mm apart, max 0.68mm (see nearest_vertex_index). So a travel move
    built straight from these nodes leaves a sub-millimetre gap at both ends
    where it meets the piece it is travelling from/to. 6.3 must either append
    the true endpoints or accept that gap as within positioning tolerance."""
    if prev_row[target] < 0:
        return None
    path = [int(target)]
    while prev_row[path[-1]] != path[-1]:
        path.append(int(prev_row[path[-1]]))
    return path[::-1]


def compute_vertex_normals(verts, faces):
    """Area-weighted per-vertex normals, from scratch -- roadmap 6.3/6.4.
    trimesh's own vertex_normals needs scipy.sparse and silently degrades to
    poor normals without it (no scipy in this env, AGENTS.md), so accumulate
    unnormalised face normals (larger faces weigh more) onto their vertices and
    unitise. The sign is whatever the mesh winding gives; _orient_normals_outward
    fixes it globally against Surface_Bot."""
    vn = np.zeros(verts.shape, dtype=np.float64)
    tri = verts[faces]
    fn = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    for k in range(3):
        np.add.at(vn, faces[:, k], fn)
    return vn / np.maximum(np.linalg.norm(vn, axis=1, keepdims=True), 1e-12)


# --- Print ordering (roadmap 6.3) -----------------------------------------
# The 35 disjoint pieces of a layer must each be printed once (a feed move
# along the curve, cost fixed regardless of order), so ordering only changes
# the sum of the travel hops between them -- a TSP variant where each "city" is
# a piece with two possible entry ends. `cost` throughout is one layer's
# (2N,2N) geodesic cost matrix; endpoints 2p and 2p+1 are the two ends of piece
# p (_layer_endpoints_world convention), and a piece's other end is `e ^ 1`.

def travel_cost(order, cost):
    """Total inter-piece travel of a print `order` (the 6.3 objective). Only
    the hops between consecutive pieces count -- from each piece's exit end
    (entry ^ 1) to the next piece's entry end -- since the feed moves along the
    curves are order-invariant."""
    return sum(cost[entry_a ^ 1, entry_b]
               for (_, entry_a), (_, entry_b) in zip(order, order[1:]))


def greedy_piece_order(cost):
    """Seed a print order by nearest-endpoint chaining. Returns a list of
    (piece, entry_end) of length N; entry_end is the endpoint the nozzle
    arrives at, so the piece is printed from entry_end to entry_end ^ 1.

    From the current exit endpoint, hop to the nearest endpoint of any
    unvisited piece. Ties -- plentiful, since abutting pieces snap to one
    vertex at cost 0.0 (settled.md S1.31) -- break to the lowest endpoint
    index via a stable argmin, so the order is reproducible. A zero-cost hop
    to a *different* piece is real free travel and is taken; a piece leaves the
    candidate set the moment it is entered, so a closed loop's
    cost[2p, 2p+1] == 0 is never a candidate and never misread as travel."""
    n_pieces = cost.shape[0] // 2
    visited = np.zeros(n_pieces, dtype=bool)
    visited[0] = True
    order = [(0, 0)]      # start at piece 0, entered at end 0, exiting end 1
    exit_ep = 1
    for _ in range(n_pieces - 1):
        cand = np.array([e for p in range(n_pieces) if not visited[p]
                         for e in (2 * p, 2 * p + 1)])
        entry = int(cand[np.argmin(cost[exit_ep, cand])])
        visited[entry // 2] = True
        order.append((entry // 2, entry))
        exit_ep = entry ^ 1
    return order


def _sequence_colors(n):
    """`n` RGB colours evenly spaced along the CURVED_ORDER_CMAP ramp -- roadmap
    6.3's ordered-feed gradient. Piecewise-linear interpolation of the anchor
    stops in numpy (no matplotlib), so colour `k` encodes position `k/(n-1)`
    along the print order. `n == 1` returns the ramp's start."""
    if n <= 1:
        return CURVED_ORDER_CMAP[:1].copy()
    stops = CURVED_ORDER_CMAP
    t = np.linspace(0.0, 1.0, n) * (len(stops) - 1)
    lo = np.clip(np.floor(t).astype(int), 0, len(stops) - 2)
    frac = (t - lo)[:, None]
    return stops[lo] * (1 - frac) + stops[lo + 1] * frac


def _reverse_block(order, i, j):
    """order with block [i..j] reversed and each of its pieces' entry/exit ends
    flipped -- the oriented-piece form of a 2-opt segment reversal."""
    return order[:i] + [(p, e ^ 1) for p, e in reversed(order[i:j + 1])] + order[j + 1:]


def two_opt(cost, order):
    """Improve a greedy order by 2-opt: repeatedly reverse the contiguous block
    that reduces total travel, until a full sweep finds none. Reversing
    order[i:j] flips each block piece's entry/exit end as well as the block
    order. Because geodesic cost is symmetric a reversed internal hop keeps the
    same two physical endpoints and is unchanged in cost, so only the two cut
    edges actually move -- but with N=35 the tour is re-summed in full, trivial
    and immune to delta-sign slips. Block length 1 is a single-piece end-swap,
    included so a piece's entry end can be improved on its own. A good order,
    not proven-optimal (Stage6_README 6.3)."""
    order = list(order)
    best = travel_cost(order, cost)
    improved = True
    while improved:
        improved = False
        n = len(order)
        for i in range(n):
            for j in range(i, n):
                cand = _reverse_block(order, i, j)
                c = travel_cost(cand, cost)
                if c < best - 1e-9:
                    order, best, improved = cand, c, True
    return order


def dh_transform(a, alpha, d, theta):
    """Standard DH homogeneous transform, frame {i-1} -> {i}"""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,   sa,       ca,      d],
        [0,   0,        0,       1],
    ])


# Bare rotation matrices used by solve_ik/solve_ik_tcp -- pure stateless
# helpers, same footing as dh_transform (see settled.md S1.1).
def rot_x(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


# Validation
if __name__ == "__main__":
    ps.init()
    vis = VisContent()
    vis.end_effector_position([0, 0, 0, 0, 0, 0])
    print(f"[Backend] Loaded {len(vis.mesh_data)} link meshes")

    gcode_waypoints = vis.parse_gcode(os.path.join(GCODE_DIR, GCODE_FILE))
    print(f"[Backend] Parsed {len(gcode_waypoints)} G-code waypoints")
    vis.load_gcode()

