import os
import re
import json
import time
import heapq
import shutil
import hashlib
from datetime import datetime, timezone
from collections import namedtuple
import polyscope as ps
import numpy as np
import trimesh

# FR5 link meshes, zero-pose world frame (see docs/FR5_Mesh_Convention.md)
MESH_DIR = "assets/fr5_meshes"
MESH_FILES = [f"Robot{i}.obj" for i in range(7)]  # Robot0 (base) .. Robot6

# Tool head, mounted on the flange (Delta_6). The nozzle mesh follows the
# zero-pose world convention like the link meshes, but is hidden since roadmap
# 7.1 -- it is not the head tool=1 was calibrated against. The TCP is no longer
# a zero-pose world point read from TCP.txt (now legacy, kept as a record); it
# is derived from the flange-local offset below.
PRINTER_HEAD_DIR = "assets/printerHead"
NOZZLE_FILE = "nozzle.obj"

# Real calibrated tool offset, flange frame: [x, y, z, rx, ry, rz] (mm, deg).
# Source: docs/saved_coords_data_and_usage_EN.md 1.2, tool_index=1 -- the only
# tool in active use. Supersedes the TCP.txt world point + borrowed rotation
# (settled.md S1.4); see roadmap 7.1. A second tool becomes a tool_index-keyed
# dict here, not before.
TCP_OFFSET_6D_MM_DEG = np.array([-134.777, 96.448, 106.334, 86.647, -13.136, 60.612])

# The robot's real limits, from the teach pendant (docs/FR5_Joint_Limits.md
# "Physical Joint Limits"). Every solver call and the export self-check use
# these. NOT gui_panel.JOINT_LIMITS, which is the same doc's separate
# "Practical Slider Ranges" -- a conservative hand-driving range that governs
# the sliders only. Roadmap 7.2 split the two: the solver had been borrowing
# the slider constant, which rejected poses the arm can physically reach
# (J2/J4 by ~134 and ~94 degrees).
PHYSICAL_JOINT_LIMITS = [
    (-174, 174),  # J1
    (-264, 84),   # J2  asymmetric
    (-159, 159),  # J3
    (-264, 84),   # J4  asymmetric
    (-174, 174),  # J5
    (-174, 174),  # J6
]

# --- External IK exchange spec: Rejection Criteria thresholds (roadmap 7.2) ---
# Source: examples/curved_surface_printing/external_ik_exchange_spec_EN.md.
# Robot/format-level, so they live here rather than in study_config.py, which
# S1.41 reserves for material- and nozzle-dependent job values.

# Expected TCP 6D pose at joints=[0]*6, from the spec's "Reference Values".
# FK(0) + TCP_OFFSET_6D_MM_DEG must reproduce this or the whole calibration
# chain is wrong -- the spec's first and most fundamental row.
IDENTITY_REFERENCE_TCP_POSE_6D = np.array(
    [-954.777, -308.334, 146.448, -161.378, -58.051, -25.434])
IDENTITY_POS_TOL_MM = 0.1
IDENTITY_ROT_TOL_DEG = 0.5

# The tool=1 offset as transcribed independently from saved_coords_data_and_usage_EN.md
# 1.2. Deliberately a second copy of TCP_OFFSET_6D_MM_DEG: comparing the two
# is the spec's "TCP offset vs. our calibration" row, which reads circular for a
# single-source project but does catch a mistyped digit in either constant.
TCP_CALIBRATION_REFERENCE_6D = np.array(
    [-134.777, 96.448, 106.334, 86.647, -13.136, 60.612])
TCP_OFFSET_POS_TOL_MM = 0.5
TCP_OFFSET_ROT_TOL_DEG = 0.5

PER_POINT_FK_TOL_MM = 0.1   # FK(joints) + TCP vs the exported tcp_xyz_base_mm
JOINT_STEP_MAX_DEG = 30.0   # Max change in any joint between adjacent points
SINGULARITY_WARN_J5_DEG = 2.0  # |J5| below this warns (does NOT reject)

# --- External IK exchange spec: job export destination (roadmap 7.5) ---
# Output location was explicitly open in Stage7_README.md/the inbox note;
# settled during 7.5 implementation.
EXPORT_DIR = "assets/export"
EXPORT_GENERATOR = "fairino-fr5-sim stage7 exporter"

TRAJECTORY_SAMPLE_INTERVAL_S = 0.1  # Minimum seconds between recorded TCP trajectory points
TRAJECTORY_RADIUS_MM = 2.0  # Trajectory curve line thickness, world units (mm)
TCP_FRAME_SCALE_MM = 50.0  # TCP coordinate-axes length, world units (mm)
# Flange->TCP stalk standing in for the hidden nozzle mesh (roadmap 7.1) --
# yellow, so it reads against the TCP triad's red/green/blue and the orange
# G-code preview.
TOOL_AXIS_COLOR = (0.95, 0.85, 0.15)
TOOL_AXIS_RADIUS_MM = 4.0

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
# rotation, leaving little margin for the wrist to also orient freely.
#
# Moved from [-600, -300, 0] by roadmap 7.1: the real tool=1 offset puts the
# flange at TCP + [-41.6, -108.95, 158.66] instead of [-21.9, 26.0, 159.9]
# (world, for the planar R_target = I -- the offset is orientation-dependent),
# and that extra ~109mm in -Y pushed the far corner of the bed past the arm's
# 820mm envelope -- 3 of 181,375 waypoints needed a wrist centre up to 835.35mm
# out, and waypoint 0 was one of them. +30 X buys the reach back (19.4 needed);
# -100 Z clears the residual posed-plate rejection. Measured, not guessed: all
# 181,375 planar waypoints solve here under the gui_panel slider limits.
USER_FRAME_ORIGIN_MM = np.array([-570.0, -300.0, -100.0])
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
    CURVED_MODEL_DIR, CURVED_MODEL_ROTATE_X_DEG, CURVED_MODEL_XY_OFFSET_MM, CURVED_LAYERS,
    CURVED_OBSTACLE_FILE, CURVED_OBSTACLE_STRUCTURE_NAME, CURVED_OBSTACLE_COLOR,
    CURVED_TRAVEL_HOVER_MM, CURVED_TIP_CLEARANCE_TOLERANCE_MM,
    CURVED_BEAD_WIDTH_MM, CURVED_BEAD_HEIGHT_MM,
)
# CURVED_TIP_CLEARANCE_TOLERANCE_MM is imported again as of roadmap 7.4: it is
# filter 8's surface-mesh clearance. It went unused between 7.2 (which removed
# the tangent-plane check it originally fed) and 7.4 -- kept in study_config.py
# under a legacy marker throughout rather than deleted, which is exactly why it
# was still there to reuse. settled.md S1.46 directs preferring this tuned
# 1.0mm over the reference guide's 2.0mm default where the two disagree.

GEODESIC_CHUNK_SOURCES = 1  # whole Dijkstra sources solved per step() call.
# Measured per source: ~50ms on Surface_RX_Offset (30,284 verts), ~85ms on
# Surface_TX_Base (45,430 verts / 135,518 edges) -- so ~12-20fps while running
# and ~8.4-9.1s wall for the full 113-source job. One whole source is the
# chunk granularity because sub-source chunking would mean carrying a live
# heap plus partial dist/prev across frames -- real complexity for a job that
# finishes in seconds.

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

# ===========================================================================
# Orientation search and candidate filters -- roadmap 7.4, settled.md S1.46.
#
# Adapted from examples/curved_surface_printing/IK_BRANCH_REJECTION_GUIDE.md, a
# working implementation of this task in ANOTHER project. Its file paths do not
# exist here and its 35 deg joint-step default is deliberately not carried
# across (see EDGE_* below). Values are robot/planner-level, so they live here;
# study_config.py is reserved for material- and nozzle-dependent job values
# (S1.41) -- which is why filter 8's clearance is imported from there instead.
# ===========================================================================

# The commanded tool axis need only be perpendicular to the surface WITHIN this
# angle, per the supervisor -- superseding S1.36's "Z = the outward surface
# normal" as a hard equality. Sampled as the nominal normal plus one ring of
# ORIENT_SEARCH_TILT_RING_AZIMUTHS directions at the full cap: the cap is where
# the reach leverage is, and intermediate rings multiply IK cost for very little
# extra coverage. The supervisor phrased the search as "all combinations of Rx,
# Ry and Rz"; a tilt cone x a full roll sweep is the same set, parameterised so
# the 20 deg cap constrains only the DOF it should.
ORIENT_SEARCH_TILT_MAX_DEG = 20.0
ORIENT_SEARCH_TILT_RING_AZIMUTHS = 8

# Roll about the commanded tool axis, searched rather than pinned. S1.36
# established this DOF is free (the nozzle is rotationally symmetric) and then
# spent it on a fixed world reference, which is what produced the row-5 flips
# (S1.44). 60 slots, 6 deg apart, wrapping. This sweep is also the LARGER reach
# lever of the two: the flange->TCP offset sits laterally off the flange, so its
# perpendicular component is swept entirely.
ORIENT_SEARCH_ROLL_SLOTS = 60

# 1 nominal + 8 ring directions, x 60 roll slots = 540 commanded frames per
# waypoint, each yielding up to 8 IK branches (<=4,320 raw candidates). The
# reference guide's 480 is 60 x 8 with no cone at all.
ORIENT_SEARCH_FRAMES = (1 + ORIENT_SEARCH_TILT_RING_AZIMUTHS) * ORIENT_SEARCH_ROLL_SLOTS

# --- Candidate filters, in the order they run (cheap arithmetic -> FK -> collision) ---
# Filter 1 is joint limits, already enforced inside solve_ik_tcp_matrix against
# PHYSICAL_JOINT_LIMITS (S1.44), so it has no constant of its own here.

# Filter 2 -- J5 minimum. The reference's rule is q[4] >= 0 (negative J5 flips
# the wrist, giving an upside-down tool approach). Set to 2.0 rather than 0.0
# so it ALSO subsumes the exchange spec's row 7, which WARNs on |J5| < 2 deg as
# a singular configuration: an exported job then cannot carry that warning.
# S1.46 left the interaction between the two as an open decision; this is it.
FILTER_J5_MIN_DEG = 2.0

# Filter 3 -- J4 minimum. Opt-in, default off, as in the reference.
FILTER_J4_MIN_DEG = -60.0
FILTER_J4_ENABLED = False

# Filter 4 -- upper-branch configuration. The elbow must stand above the
# shoulder->wrist chord. Rejects lower-elbow AND near-straight-arm poses, which
# sit close to singularity with unpredictable velocity and can flip suddenly.
FILTER_UPPER_BRANCH_TOL_MM = 2.0

# Filter 5 -- elbow above the build-plate plane, with a little slack.
FILTER_ELBOW_PLATE_TOL_MM = 1.0

# Filters 6/7 -- the finite plate model that replaces S1.40's infinite plane.
# 6 is the XY shadow under the plate (expanded, to catch near-misses); 7 is the
# plate's own bounding slab. Together they are why the arm may now legitimately
# reach *around* a plate mounted high, which the infinite plane forbade.
FILTER_UNDER_PLATE_MARGIN_MM = 20.0
FILTER_PLATE_SLAB_CLEARANCE_MM = 3.0

# Filter 8 -- surface-mesh collision. Clearance comes from study_config's
# CURVED_TIP_CLEARANCE_TOLERANCE_MM (imported above). Broadphase cell size:
# ~6x the print surfaces' ~1.24mm median edge, so a cell holds a handful of
# triangles and a query touches 27 of them.
SURFACE_GRID_CELL_MM = 8.0

# Filter 9 -- robot/tool self-collision, OBB vs OBB.
FILTER_SELF_COLLISION_CLEARANCE_MM = 5.0

# Length of each oriented-box proxy along a link's principal axis. ONE box per
# link is unusably loose -- measured, Robot3's single 502mm box reports contact
# with Robot5/Robot6 in all 8 branches at planar waypoint 0 where the true mesh
# gap is 20-35mm -- so links are covered by a row of shorter boxes instead
# (the reference guide's "multi-proxy OBB"). 80mm keeps the FR5's 425/395mm
# links to 6-7 boxes each while staying tight enough not to invent collisions.
SELF_COLLISION_PROXY_SEGMENT_MM = 80.0

# Link surface sampling for filters 6-8. The link meshes are far denser than a
# clearance test needs; one representative point per 25mm voxel keeps the
# per-candidate point count in the hundreds rather than the tens of thousands.
LINK_SAMPLE_SPACING_MM = 25.0

# --- Edge (adjacent-waypoint) costs and rejection, inside the graph search ---
# EDGE_MAX_JOINT_STEP is deliberately JOINT_STEP_MAX_DEG (30.0, defined above
# from the exchange spec) rather than a fresh constant. The reference guide uses
# 35, and carrying that across would build a planner whose own edge filter
# admits jobs the receiving side rejects -- aliasing the spec value here makes
# that mistake unavailable rather than merely discouraged.
EDGE_MAX_JOINT_STEP_DEG = JOINT_STEP_MAX_DEG

# Weighted-L1 joint movement. Proximal joints cost more, pushing redundancy
# resolution out to the wrist where it is cheap and safe.
EDGE_JOINT_WEIGHTS = np.array([3.0, 3.0, 2.0, 1.0, 1.0, 0.5])

# Switching IK branch family (elbow-up <-> elbow-down) mid-path is legal but
# should be a last resort, so it carries a flat penalty far above any plausible
# joint-movement cost. Adjacent roll slots are free; larger roll jumps grow
# quadratically in the excess.
EDGE_BRANCH_CHANGE_PENALTY = 150.0
EDGE_ROLL_QUADRATIC_WEIGHT = 2.0

PRECOMPUTE_CHUNK_SIZE = 25  # waypoints solved per step() call -- keeps each
# per-frame batch well under a 60fps budget. Measured ~0.5ms/waypoint for
# solve_ik_tcp_matrix + the ground-clearance filter at benchy scale (see
# settled.md S1.13's verification), so this is roughly a 12ms slice per frame.

SEARCH_CHUNK_SIZE = 1  # waypoints per step() call on the ORIENTATION-SEARCH
# path (roadmap 7.4). PRECOMPUTE_CHUNK_SIZE's 25 assumes ~8 IK solves per
# waypoint; the curved search runs ORIENT_SEARCH_FRAMES (540) of them plus the
# filter stack, so 25 would be ~13,500 solves in one frame.
#
# Even at 1 this is the slowest frame in the app: measured 437ms/waypoint at the
# real User Frame and 749ms at the default plate pose (more candidates survive
# there), i.e. ~1.3-2.3 fps while a curved precompute runs. That is deliberate
# and is not a freeze -- the progress bar advances and Pause responds within a
# frame -- but it is well short of interactive. Going lower means splitting a
# single waypoint's search across frames, which would mean carrying the partial
# candidate set and the half-relaxed layer across callbacks; not worth the
# complexity for a batch operation that is cached afterwards.

GCODE_PRECOMPUTE_CACHE = os.path.join(GCODE_DIR, "model.precompute.npz")  # roadmap 5.10, settled.md S1.21
PRECOMPUTE_CACHE_VERSION = 7  # Bump to invalidate all existing caches on a schema change (2: per-waypoint R_target, roadmap 6.5; 3: reject_below_ground in key, roadmap 6.6; 4: allow_tcp_through_plate in key + posed-plate clearance, roadmap 6.8; 5: real tool=1 TCP offset, roadmap 7.1 -- every cached joint path was solved for a different flange->TCP transform, and the offset is a constant rather than a cache-key field; 6: roadmap 7.2 -- curved runs no longer reject on clearance at all, and every run now solves against PHYSICAL_JOINT_LIMITS rather than the narrower slider range, which changes both which branches are valid and which representation wrap_into_limits picks; 7: roadmap 7.4 -- the orientation search + nine-filter set changes every solved branch on both paths, AND the schema itself grew the waypoint positions/is_feed/normals arrays that build_export_segments() needs after a cache hit. settled.md S1.46 pre-authorised sharing this one bump with 7.5's cache-gap fix, since either alone invalidates every cache)


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
        self.export_status = ""  # roadmap 7.5 -- validate_job()'s table, plus the written path on success
        self.precompute_cache_meta = None  # Cache key captured at precompute-start, see run_toolpath_ik_precompute()
        self.precompute_cache_path = None  # Which cache file this precompute writes to (per-layer for curved, roadmap 6.5)
        self.precompute_filter_mode = "planar"  # "planar" | "curved" -- selects the candidate
                                    # filter set AND whether the orientation search runs
                                    # (roadmap 7.4). Was a check_collision bool at 7.2.
        self.precompute_filter_ctx = None   # Per-run filter constants, see _filter_context()
        # Candidate DAG (roadmap 7.4). Per-waypoint lists of the surviving
        # candidates and the backpointers dijkstra_candidate_path's relaxation
        # leaves behind; consumed by the backtrack at the end of the sweep.
        self.precompute_cand_joints = []
        self.precompute_cand_roll = []
        self.precompute_cand_branch = []
        self.precompute_dag_dist = None     # Running shortest-path cost to each live candidate
        self.precompute_dag_back = []
        self.precompute_commanded_R = []    # The orientation actually chosen per waypoint --
                                    # distinct from precompute_R_target, which stays the
                                    # NOMINAL surface normal frame the export reads.
        self.precompute_reject_tally = {}   # filter name -> count, for the failure diagnostic
        self.toolpath_source = -1  # -1 = planar G-code; 0..len(CURVED_LAYERS)-1 = that curved layer.
                                    # Single source of truth for what the shared Run/Pause/Cancel/Reset
                                    # precompute+playback controls currently target -- roadmap 6.6.
        # allow_tcp_through_plate lived here until roadmap 7.4. It gated the tool
        # point against S1.40's INFINITE plate plane, which filters 6 and 7 have
        # replaced with a finite footprint plus a bounding slab -- and those
        # exclude the tool point entirely, since IK pins it to the commanded
        # waypoint (see _filter_context). With nothing left for the toggle to
        # gate, settled.md S1.46 supersedes it outright; it is gone from the
        # cache keys and the GUI too.

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

        # Curved-model state, in dependency order -- each _reset_* helper below
        # is the single definition of its group's cleared values, shared with
        # the clear/abort paths (settled.md S1.42).
        self._reset_curved_model_state()
        self.geodesic_status = ""   # not in _reset_geodesic_state(): the abort
                                    # path sets its own explanatory message
        self._reset_geodesic_state()
        self._reset_print_order_state()
        self._reset_orientation_state()
        self._reset_curved_bead_state()

        # Initialise the scene
        self.create_coordinate_frame()
        self.load_build_plate()
        self.mesh_data = self.load_data()
        self.update_arm([0, 0, 0, 0, 0, 0])


    # --- Grouped curved-model state resets ---------------------------------
    # Each helper is the one place a subsystem's cleared state is defined, so
    # __init__ and the clear/abort paths cannot drift apart (adding a field to
    # one and forgetting the other was the hazard -- settled.md S1.42).
    # Deliberately pure state assignment, no Polyscope calls: that is what makes
    # them safe to call from __init__ before any structure is registered. The
    # ps.remove_* calls stay at the call sites, where the structure names are
    # known.

    def _reset_curved_model_state(self):
        """Retained curved-model geometry, world coordinates (already through
        T_curved) -- roadmap 6.2 needs the per-piece curves and the print
        surfaces that 6.1 previously computed and threw away. All lists are
        indexed positionally by CURVED_LAYERS (examples/curved_surface_printing/
        study_config.py). The obstacle mesh is deliberately absent from these:
        it's a 6.5 collision body, not a print surface."""
        self.curved_model_loaded = False  # True once load_curved_model() has registered its structures -- roadmap 6.1/6.6
        self.curved_pieces_world = None        # list of len(CURVED_LAYERS) lists of (Ni,3) polylines
        self.curved_surface_verts_world = None # list of len(CURVED_LAYERS) (V,3)
        self.curved_surface_vnormals_world = None  # list of len(CURVED_LAYERS) (V,3) outward unit normals -- 6.3 hover, 6.4 orientation
        self.curved_surface_faces = None       # list of len(CURVED_LAYERS) (F,3), placement-invariant
        self.curved_layer_names = None         # list of len(CURVED_LAYERS) display names, e.g. ["RX", "TX"]
        self.T_curved = None                   # (4,4) placement actually used, for the staleness check
        self._T_user_frame_at_curved_load = None  # Plate pose the world state above was built against


    def _reset_geodesic_state(self):
        """Chunked geodesic precompute state, see run_geodesic_precompute().
        Resets geodesic_index/total together, so a stale index can't outlive
        the arrays it counted (settled.md S1.24). Deliberately excludes
        geodesic_status, so a caller can set an explanatory message first."""
        self.geodesic_running = False
        self.geodesic_index = 0
        self.geodesic_total = 0
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


    def _reset_print_order_state(self):
        """Print-order + travel moves, see build_print_order() -- roadmap 6.3.
        All per-layer, derived from the geodesic cost/prev, so they are cleared
        alongside them in _abort_geodesic_precompute()."""
        self.curved_order_loaded = False       # True once build_print_order() has run -- what 6.5 gates on
        self.curved_print_order = None         # list of len(CURVED_LAYERS) lists of (piece, entry_end)
        self.curved_travel_moves = None        # list of len(CURVED_LAYERS) lists of (M,3) hover polylines
        self.curved_travel_total = None        # list of len(CURVED_LAYERS) optimized inter-piece travel (mm)
        self.curved_travel_naive = None        # list of len(CURVED_LAYERS) file-order travel (mm), the baseline
        self.curved_order_status = ""


    def _reset_orientation_state(self):
        """Per-waypoint TCP orientation frames, see build_orientation_frames()
        -- roadmap 6.4. Derived from the print order, so cleared with it in
        _abort_geodesic_precompute()."""
        self.curved_orient_loaded = False      # True once build_orientation_frames() has run -- what 6.5 gates on
        self.curved_orient_frames = None       # list of len(CURVED_LAYERS) lists of (pos_world (3,), R_target (3,3)), print order
        self.curved_orient_status = ""


    def _reset_curved_bead_state(self):
        """Per-layer printed-bead playback state, see _build_curved_beads() /
        _init_curved_toolpath_playback() -- roadmap 6.6. Mirrors the flat
        gcode_bead_* fields, but indexed per layer (lazily sized to
        len(CURVED_LAYERS) on first use) so RX's and TX's printed meshes can
        coexist -- the S1.32 stack rule requires TX's view to keep showing
        RX's already-printed layer beneath it, not just whichever was last
        built. Cleared only by clear_curved_model() or a re-order/re-orient
        cascade (_abort_geodesic_precompute()) -- never by a generic
        precompute abort/cancel, so switching the active toolpath source
        can't make a completed layer's printed mesh disappear."""
        self.curved_bead_verts_full = None
        self.curved_bead_faces = None
        self.curved_bead_reveal_index = None
        self.curved_bead_face_prefix = None
        self.curved_bead_verts_current = None
        self.curved_print_handle = None
        self.curved_bead_registered_capacity = None


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

        # The plate's own extent in plate-local coordinates, retained for roadmap
        # 7.4's filters 6 and 7 -- the finite footprint + bounding slab that
        # replace S1.40's infinite plane. Taken AFTER the PLATE_THICKNESS_MM
        # lift, so local z spans [0, PLATE_THICKNESS_MM] with the print face on
        # top, matching what _plate_plane() reports. Recomputed on every call
        # because it is cheap and because a future non-uniform plate asset must
        # not be able to drift from the mesh actually registered above.
        self.plate_local_bounds = (plate_verts_local.min(axis=0), plate_verts_local.max(axis=0))

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

        # Width varies per segment here (it comes from extruded E), so the
        # width-match term applies -- see bead_faces(), settled.md S1.19.
        faces, bead_face_prefix = bead_faces(
            self._BEAD_BOX_FACE_TEMPLATE, K, reveal_waypoint_index,
            u[valid], width_valid=width[valid])

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
        by CURVED_LAYERS (plus the optional CURVED_OBSTACLE_FILE, which orients
        normals outward -- not a collision body, S1.37) and place them above
        the build plate -- roadmap
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
        assembly's XY bbox on CURVED_MODEL_XY_OFFSET_MM relative to the User
        Frame origin, and lift so its lowest point sits at the plate-local
        print surface -- z=0 after the same PLATE_THICKNESS_MM compensation
        load_build_plate()/build_toolpath_waypoints_world() already apply
        (position_mm marks the plate's resting/bottom face, not its top).

        XY placement is deliberately NOT derived from the build-plate mesh
        (roadmap 7.4 follow-up, settled.md S1.48, superseding the original
        "center over the plate mesh's own bbox-center" design): BambuLab_BuildPlate.obj
        is a stand-in asset whose bbox-center offset was measured to push the
        workpiece +105.6mm outward at the real calibrated User Frame -- enough
        to fail IK on ~24% of feed points that a zero offset (the study default)
        solves. See
        wiki/001_Inbox/2026-09-03_curved_placement_plate_centring_offset.md.

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

        T_placement = np.eye(4)
        T_placement[:2, 3] = CURVED_MODEL_XY_OFFSET_MM - (assembly_min[:2] + assembly_max[:2]) / 2.0
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

        self._reset_curved_model_state()
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
        self._reset_geodesic_state()

        # The 6.3 print order and its travel moves are derived from the cost
        # matrices and predecessor rows just dropped, so they go stale with
        # them -- clear the state and remove the rendered travel networks.
        if self.curved_layer_names is not None:
            for name in self.curved_layer_names:
                ps.remove_curve_network(f"Curved Travel {name}", error_if_absent=False)
                ps.remove_curve_network(f"Curved Order Feed {name}", error_if_absent=False)
                ps.remove_curve_network(f"Curved Orient Frames {name}", error_if_absent=False)
                ps.remove_surface_mesh(f"Curved Print {name}", error_if_absent=False)
        self._reset_print_order_state()

        # The 6.4 orientation frames derive from the print order just dropped.
        self._reset_orientation_state()

        # The 6.6 printed-bead meshes are built from the waypoints derived
        # above (print order + orientation), across every layer -- not just
        # whichever is currently active -- so they go stale with them too.
        self._reset_curved_bead_state()


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

        # Fixed cross-section here (CURVED_BEAD_WIDTH_MM), so no width-match
        # term -- see bead_faces(), settled.md S1.19.
        faces, bead_face_prefix = bead_faces(
            self._BEAD_BOX_FACE_TEMPLATE, K, reveal_waypoint_index, u[valid])

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
            # Which filter set solved this path -- roadmap 7.4. Replaces the
            # allow_tcp_through_plate entry, whose toggle S1.46 superseded.
            "filter_mode": "planar",
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
            "filter_mode": "curved",
            # The orientation search's shape -- roadmap 7.4. Widening the cone,
            # adding ring azimuths or changing the roll resolution all change
            # which candidates exist and therefore which path is optimal, so a
            # cache solved under different search parameters must miss.
            "orient_search": [ORIENT_SEARCH_TILT_MAX_DEG,
                              ORIENT_SEARCH_TILT_RING_AZIMUTHS,
                              ORIENT_SEARCH_ROLL_SLOTS],
        }


    def save_toolpath_precompute_cache(self, cache_path=GCODE_PRECOMPUTE_CACHE):
        """Best-effort write of a just-completed precompute to cache_path,
        tagged with the key captured at precompute-start
        (self.precompute_cache_meta) -- roadmap Stage5_README.md 5.10 (planar,
        default path), 6.5 (curved, per-layer path). Called only from
        step_toolpath_ik_precompute()'s successful-completion branch, never on
        an aborted/cancelled precompute. Wrapped in a bare except: a cache-write
        failure (disk full, permissions) must never surface as a failure of the
        precompute itself, which already succeeded in memory.

        Since roadmap 7.4 the waypoint positions, their is_feed flags and the
        nominal surface normals are persisted alongside the joint path. Without
        them build_export_segments() returns [] after every cache hit -- the
        joint path alone carries no segment boundaries -- which is the gap
        recorded in wiki/001_Inbox/2026-08-15_export_segments_cache_gap.md. The
        normals are stored as the (N,3) Z column rather than the full (N,3,3)
        R_target: it is all the exporter reads, and on the planar path the
        (N,3,3) is a broadcast_to view of one constant anyway."""
        try:
            positions = np.array([p for p, _ in self.precompute_waypoints], dtype=np.float32)
            is_feed = np.array([bool(f) for _, f in self.precompute_waypoints])
            normals = np.asarray(self.precompute_R_target, dtype=np.float32)[:, :, 2]
            np.savez(
                cache_path,
                joint_path=np.asarray(self.precompute_joint_path, dtype=np.float32),
                waypoint_positions=positions,
                waypoint_is_feed=is_feed,
                waypoint_normals=normals,
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
            # Roadmap 7.4: restore what the segment builder needs. A v7 cache
            # always carries these (save_ writes them unconditionally), so a
            # KeyError here means a hand-made or truncated file and is correctly
            # treated as a plain miss by the except below.
            positions = cached["waypoint_positions"].astype(float)
            is_feed = cached["waypoint_is_feed"]
            normals = cached["waypoint_normals"].astype(float)
        except Exception:
            return False

        # Rebuild precompute_waypoints/_R_target rather than leaving them None.
        # Before 7.4 both runners returned on a cache hit before
        # _begin_toolpath_precompute ever assigned them, so build_export_segments
        # tripped its own guard and exported nothing -- silently reporting a
        # clean job until 7.2's in-house row 0 made it loud. Only the normal (the
        # R_target Z column) is persisted, so the restored R_target is a normal-
        # only stand-in: its X/Y columns are not reconstructed, because nothing
        # downstream of a cache hit reads them.
        self.precompute_waypoints = list(zip(positions, [bool(f) for f in is_feed]))
        R_restored = np.zeros((len(normals), 3, 3))
        R_restored[:, :, 2] = normals
        self.precompute_R_target = R_restored

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
                                   filter_mode="planar", layer=None):
        """Load a freshly-built waypoint source into precompute state -- the
        shared seam behind run_toolpath_ik_precompute (planar) and
        run_curved_toolpath_ik_precompute (curved), roadmap 6.5. R_target_array
        is (N,3,3), one NOMINAL orientation per waypoint -- Z is the exact
        surface (or plate) normal. Since roadmap 7.4 that is the axis of the
        search cone rather than the pose actually commanded; the chosen frames
        land in precompute_commanded_R. cache_path is where a completed solve is
        written.

        filter_mode is the planar/curved difference, and since roadmap 7.4 it
        selects a filter SET rather than toggling one check (it was a
        check_collision boolean at 7.2, and a tip tolerance before that):

          "planar"  one commanded orientation per waypoint (the plate does not
                    tilt, settled.md S1.12), so <=8 candidates; filters 2-7 and 9.
          "curved"  the full ORIENT_SEARCH_FRAMES orientation search, so <=4,320
                    candidates; filters 2-9, filter 8 against this layer's own
                    print surface.

        This reverses 7.2's collision narrowing, which had left the curved path
        with no geometric rejection at all (settled.md S1.44 -> S1.46). S1.44's
        seven exchange-spec rows are untouched by that -- the narrowing and the
        table were always separate questions.

        A snapshotted argument rather than a read of self.toolpath_source,
        because the precompute captures its own per-run state here at begin and
        a live-mutating source field could change mid-solve."""
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
        self.precompute_filter_mode = filter_mode

        # Per-run filter constants (surface grid, plate frame, link pair list),
        # built once here rather than per candidate -- roadmap 7.4.
        self.precompute_filter_ctx = self._filter_context(filter_mode, layer)

        # Candidate-DAG accumulators. Only the PREVIOUS layer's joints and dist
        # stay live during the sweep; the per-layer candidate arrays and
        # backpointers accumulate for the final backtrack.
        self.precompute_cand_joints = []
        self.precompute_cand_roll = []
        self.precompute_cand_branch = []
        self.precompute_dag_dist = None
        self.precompute_dag_back = []
        self.precompute_commanded_R = []
        self.precompute_reject_tally = {}


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
                cache_meta, cache_path=GCODE_PRECOMPUTE_CACHE, filter_mode="planar")

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
                cache_meta, cache_path=cache_path, filter_mode="curved", layer=layer)

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
        # wrong filter set (roadmap 6.5/7.2/7.4). Resets to "planar", the
        # narrower search -- a stale "curved" would run a 540-orientation sweep
        # over the 181k-waypoint G-code path.
        self.precompute_cache_path = None
        self.precompute_filter_mode = "planar"
        self.precompute_filter_ctx = None
        # Candidate-DAG state (roadmap 7.4). Dropped here rather than left to be
        # overwritten at the next begin: these are the largest arrays the
        # precompute holds -- hundreds of MB on a curved layer -- and a cancelled
        # run should not keep them alive until the next one starts.
        self.precompute_cand_joints = []
        self.precompute_cand_roll = []
        self.precompute_cand_branch = []
        self.precompute_dag_dist = None
        self.precompute_dag_back = []
        self.precompute_commanded_R = []
        self.precompute_reject_tally = {}


    def _waypoint_candidates(self, i):
        """Every admissible (joints, roll_slot, ik_branch) candidate at waypoint
        i -- roadmap 7.4 steps 2 and 3. Returns
        (joints (C,6), roll (C,), branch (C,), frames (C,3,3)), all empty when
        nothing survives; the per-filter rejection counts land in
        self.precompute_reject_tally.

        Curved runs search ORIENT_SEARCH_FRAMES commanded orientations about the
        nominal surface normal; planar runs command the single constant plate
        frame (S1.12) and so evaluate at most 8 branches. Either way the branches
        then go through the same filter stack.

        Candidates are deduped on joints rounded to 0.01 deg. Distinct
        (tilt, roll) frames routinely map to the same arm pose -- most obviously
        wherever the tool axis is near the wrist axis -- and every duplicate
        would otherwise cost a full row and column in this layer's edge block.
        """
        pos_world_mm, _is_feed = self.precompute_waypoints[i]
        nominal_R = self.precompute_R_target[i]

        # Per-waypoint, not cumulative. The tally exists to answer "why did THIS
        # waypoint have nothing admissible", and totals carried over from the
        # thousands of waypoints that succeeded would drown that out.
        self.precompute_reject_tally = {}

        if self.precompute_filter_mode == "curved":
            frames = orientation_candidates(nominal_R)
        else:
            frames = np.asarray(nominal_R)[None, :, :]

        ctx = self.precompute_filter_ctx
        joints, rolls, branches, chosen_frames = [], [], [], []
        seen = set()
        for f_idx, R in enumerate(frames):
            roll_slot = f_idx % ORIENT_SEARCH_ROLL_SLOTS
            solutions, _status = self.solve_ik_tcp_matrix(
                pos_world_mm, R, self.precompute_joint_limits,
                reference_joint_angles=self.precompute_ref)
            if not solutions:
                # No geometric solution, or none inside PHYSICAL_JOINT_LIMITS --
                # filter 1, counted so the diagnostic can distinguish "the arm
                # cannot reach this point" from "a filter is mistuned".
                self.precompute_reject_tally["limits/reach"] = (
                    self.precompute_reject_tally.get("limits/reach", 0) + 1)
                continue
            for angles, _singular, branch_idx in solutions:
                key = tuple(np.round(angles, 2))
                if key in seen:
                    continue
                reason = self._candidate_admissible(angles, ctx)
                if reason is not None:
                    self.precompute_reject_tally[reason] = (
                        self.precompute_reject_tally.get(reason, 0) + 1)
                    continue
                seen.add(key)
                joints.append(angles)
                rolls.append(roll_slot)
                branches.append(branch_idx)
                chosen_frames.append(R)

        if not joints:
            return (np.empty((0, 6)), np.empty(0, dtype=np.int32),
                    np.empty(0, dtype=np.int32), np.empty((0, 3, 3)))
        return (np.asarray(joints, dtype=np.float64),
                np.asarray(rolls, dtype=np.int32),
                np.asarray(branches, dtype=np.int32),
                np.asarray(chosen_frames))

    def _reject_summary(self):
        """The current waypoint's per-filter rejection counts, commonest first --
        the diagnostic that distinguishes "the arm genuinely cannot reach this
        point" (dominated by limits/reach) from "one filter is mistuned"
        (dominated by a single filter name). Roadmap 7.4."""
        return ", ".join(f"{k} {v}" for k, v in
                         sorted(self.precompute_reject_tally.items(), key=lambda kv: -kv[1]))

    def _fail_precompute(self, i, message):
        """Abort the whole precompute at waypoint i with an explanatory status.
        No partial motion is kept and no filter is relaxed -- matching the
        reference implementation, a job that cannot be planned inside the
        filters fails rather than falling back to a less-safe candidate."""
        total = self.precompute_total
        self._abort_toolpath_ik_precompute()
        self.precompute_status = f"Waypoint {i}/{total}: {message}"

    def step_toolpath_ik_precompute(self):
        """Advance the in-progress precompute by one chunk of waypoints -- call
        every frame from render(). No-ops unless precompute_running.

        Since roadmap 7.4 this is a single fused pass rather than a greedy walk:
        per waypoint it generates the admissible candidate set
        (_waypoint_candidates) and immediately relaxes the previous layer into it
        with dijkstra_candidate_path's edge block. Fusing the two is what keeps
        the search chunkable across frames AND bounded in memory -- only the
        previous layer's joints stay live, and the layered DAG's topological
        order is exactly the waypoint order this loop already walks. On the final
        waypoint it backtracks to fill precompute_joint_path.

        The greedy per-waypoint ranking this replaces (S1.5/S1.11 -- rank
        branches by wrapped distance to the previous pose, take the first that
        clears) could not recover from a dead end, nor undo a discontinuity in
        the commanded frame itself. That is the documented cause of the curved
        row-5 failures (23/35 RX, 15/35 TX segments with >30deg steps inside a
        feed run).

        Aborts the whole precompute at the first waypoint with no admissible
        candidate or no traversable edge into it, reporting the per-filter
        breakdown."""
        if not self.precompute_running:
            return

        chunk = SEARCH_CHUNK_SIZE if self.precompute_filter_mode == "curved" else PRECOMPUTE_CHUNK_SIZE
        end = min(self.precompute_index + chunk, self.precompute_total)
        for i in range(self.precompute_index, end):
            joints, rolls, branches, frames = self._waypoint_candidates(i)
            if len(joints) == 0:
                n_frames = (ORIENT_SEARCH_FRAMES
                            if self.precompute_filter_mode == "curved" else 1)
                self._fail_precompute(
                    i, f"no admissible candidate over {n_frames} commanded "
                       f"orientation(s) ({self._reject_summary()})")
                return

            if i == 0:
                self.precompute_dag_dist = np.zeros(len(joints))
            else:
                dist, back = self._relax_candidate_layer(i, joints, rolls, branches)
                if dist is None:
                    self._fail_precompute(
                        i, f"{len(joints)} candidate(s) are admissible here, but every "
                           f"edge from the previous waypoint moves some joint more than "
                           f"{EDGE_MAX_JOINT_STEP_DEG:.0f}deg")
                    return
                self.precompute_dag_dist = dist
                self.precompute_dag_back.append(back)

            self.precompute_cand_joints.append(joints.astype(np.float32))
            self.precompute_cand_roll.append(rolls)
            self.precompute_cand_branch.append(branches)
            self.precompute_commanded_R.append(frames)
            # Rank the NEXT waypoint's IK branches against this waypoint's
            # cheapest-so-far candidate. Only an ordering hint now -- the graph
            # search, not this reference, decides what is actually taken -- but
            # it keeps solve_ik_tcp_matrix's wrap_into_limits picking a
            # representation near the path being walked.
            self.precompute_ref = joints[int(np.argmin(self.precompute_dag_dist))]

        self.precompute_index = end
        if self.precompute_index >= self.precompute_total:
            self._finish_candidate_search()
        else:
            self.precompute_status = (
                f"Precomputing {self.precompute_index}/{self.precompute_total} waypoints"
                f"{f' ({len(self.precompute_cand_joints[-1])} candidates)' if self.precompute_filter_mode == 'curved' else ''}")

    def _relax_candidate_layer(self, i, joints, rolls, branches):
        """One layer of dijkstra_candidate_path's relaxation, applied live during
        the sweep -- roadmap 7.4. Returns (dist, back), or (None, None) when no
        edge into this layer is traversable.

        The edge cost and the E1 hard rejection are defined and justified in
        dijkstra_candidate_path's docstring; this is the same arithmetic applied
        incrementally so the search can be chunked across frames. The two are
        kept deliberately in step: dijkstra_candidate_path is the readable,
        unit-tested statement of the algorithm, this is its streaming form."""
        q_prev = self.precompute_cand_joints[-1].astype(np.float64)
        roll_prev, branch_prev = self.precompute_cand_roll[-1], self.precompute_cand_branch[-1]

        D = np.abs(joints[None, :, :] - q_prev[:, None, :])
        cost = D @ EDGE_JOINT_WEIGHTS
        cost = cost + EDGE_BRANCH_CHANGE_PENALTY * (branches[None, :] != branch_prev[:, None])
        roll_d = np.abs(rolls[None, :].astype(float) - roll_prev[:, None].astype(float))
        roll_d = np.minimum(roll_d, ORIENT_SEARCH_ROLL_SLOTS - roll_d)
        cost = cost + EDGE_ROLL_QUADRATIC_WEIGHT * np.maximum(0.0, roll_d - 1.0) ** 2

        # E1, and ONLY between two feed waypoints -- the exchange spec's row 5
        # measures steps within a continuous extrusion line, and travel moves are
        # legitimately large (planar: 57.32deg overall vs 5.85deg inside a segment).
        if bool(self.precompute_waypoints[i - 1][1]) and bool(self.precompute_waypoints[i][1]):
            cost = np.where(D.max(axis=-1) > EDGE_MAX_JOINT_STEP_DEG, np.inf, cost)

        total = self.precompute_dag_dist[:, None] + cost
        back = np.argmin(total, axis=0)
        dist = total[back, np.arange(total.shape[1])]
        if not np.any(np.isfinite(dist)):
            return None, None
        return dist, back.astype(np.int32)

    def _finish_candidate_search(self):
        """Backtrack the candidate DAG into precompute_joint_path and write the
        cache -- roadmap 7.4, the tail of step_toolpath_ik_precompute.

        Candidates whose running cost is infinite are unreachable through the
        graph, so the final choice is the cheapest FINITE one; _relax_candidate_layer
        has already failed the run if a whole layer went infinite."""
        chosen_last = int(np.argmin(self.precompute_dag_dist))
        n = len(self.precompute_cand_joints)
        chosen = [0] * n
        chosen[-1] = chosen_last
        for i in range(n - 1, 0, -1):
            chosen[i - 1] = int(self.precompute_dag_back[i - 1][chosen[i]])

        self.precompute_joint_path = [
            self.precompute_cand_joints[i][chosen[i]].astype(np.float64) for i in range(n)]
        # The orientation actually commanded per waypoint, kept separate from
        # precompute_R_target (still the nominal surface normal, which is what
        # the exchange spec's normal_base wants and what the beads stack on).
        self.precompute_commanded_R = np.array(
            [self.precompute_commanded_R[i][chosen[i]] for i in range(n)])

        self.precompute_running = False
        cand_counts = [len(c) for c in self.precompute_cand_joints]
        self.precompute_status = (
            f"Solved {self.precompute_total} waypoint(s) -- "
            f"{min(cand_counts)}-{max(cand_counts)} candidates/waypoint, "
            f"path cost {self.precompute_dag_dist[chosen_last]:.1f}")
        self.save_toolpath_precompute_cache(self.precompute_cache_path)

        # The per-layer candidate arrays are the precompute's peak memory (a
        # curved layer can hold hundreds of MB) and nothing downstream reads
        # them once the path is backtracked. Drop them here rather than at the
        # next run, so an idle session isn't sitting on them.
        self.precompute_cand_joints = []
        self.precompute_cand_roll = []
        self.precompute_cand_branch = []
        self.precompute_dag_back = []


    def build_export_segments(self):
        """Split the solved path into the exchange spec's "segments" -- roadmap
        7.2. A segment is one continuous extrusion line, which on both toolpath
        sources is exactly a maximal run of consecutive is_feed_move waypoints:

          planar  build_toolpath_waypoints_world -- is_feed is G1 vs G0
          curved  build_curved_toolpath_waypoints_world -- True per print-order
                  piece, False per inter-piece travel hop

        So this is one shared function rather than two per-path builders, and it
        needs no reference to build_print_order: the piece boundaries are already
        encoded in the flags. Travel runs are dropped, not exported -- the spec
        states the receiving side re-inserts a travel MoveJ between adjacent
        segments.

        Returns a list of ExportSegment. Only the solved prefix is used, so a
        partial (paused) precompute yields the segments solved so far.

        Returns [] only when there is genuinely no waypoint source to split.
        Until roadmap 7.4 that also happened after any cache HIT -- the normal
        case -- because load_toolpath_precompute_cache() restored the joint path
        but not the waypoints, so this tripped its own guard and exported
        nothing. 7.4's cache schema persists the positions, is_feed flags and
        normals, closing that gap
        (wiki/001_Inbox/2026-08-15_export_segments_cache_gap.md).
        validate_job()'s in-house row 0 stays regardless: an empty job must never
        read as ACCEPTED, whatever emptied it.
        """
        if not self.precompute_joint_path or self.precompute_waypoints is None:
            return []

        solved = len(self.precompute_joint_path)
        segments, run_start = [], None
        for i in range(solved + 1):
            # One past the end closes any run still open at the frontier.
            is_feed = i < solved and bool(self.precompute_waypoints[i][1])
            if is_feed and run_start is None:
                run_start = i
            elif not is_feed and run_start is not None:
                sl = slice(run_start, i)
                segments.append(ExportSegment(
                    index=len(segments),
                    positions=np.array([p for p, _ in self.precompute_waypoints[sl]]),
                    joints=np.array(self.precompute_joint_path[sl]),
                    # settled.md S1.36/S1.12: R_target's Z column is already the
                    # outward surface (or plate) normal, a unit vector in the base
                    # frame -- exactly the spec's normal_base. Nothing to recompute.
                    # Read-only and 0-strided on the planar path (that R_target is
                    # a np.broadcast_to view of one constant) -- 7.5's writer must
                    # copy before mutating.
                    normals=np.asarray(self.precompute_R_target[sl])[:, :, 2],
                ))
                run_start = None
        return segments


    def export_active_job(self):
        """Roadmap 7.5 -- self-check the active toolpath_source against the
        exchange spec's Rejection Criteria, then write it. GUI glue: reads
        build_export_segments()/validate_job() (both source-agnostic already).
        Writes nothing on REJECT -- the self-check gates the write, not just
        the display. On REJECT, self.export_status carries the full per-row
        table (needed to see which row failed and why); on ACCEPTED it's
        collapsed to one line -- the table added nothing once every row
        already passed.

        Guards precompute_cache_path against toolpath_source first, the same
        layer-mixup check _init_toolpath_playback()/_init_curved_toolpath_playback()
        already use. Without it, switching the Toolpath Source radio after a
        completed solve (which doesn't itself touch precompute_joint_path --
        only pressing Run Precompute again does) would export whichever
        source actually solved, silently mislabeled and mis-surfaced under
        the newly-selected source's job_name/surface.obj."""
        expected_cache_path = (GCODE_PRECOMPUTE_CACHE if self.toolpath_source == -1
                                else curved_precompute_cache_path(self.curved_layer_names[self.toolpath_source]))
        if self.precompute_cache_path != expected_cache_path:
            self.export_status = "Run Precompute for the active toolpath source first"
            return
        segments = self.build_export_segments()
        ok, results = validate_job(self, segments)
        if not ok:
            self.export_status = format_validation(ok, results)
            return
        job_name = ("planar" if self.toolpath_source == -1
                    else self.curved_layer_names[self.toolpath_source])
        job_dir = write_job_export(self, segments, f"{EXPORT_DIR}/{job_name}")
        warned = any(not r.passed for r in results if r.action == "WARN")
        self.export_status = (
            f"Passed all checks{' (with warnings)' if warned else ''}, exported "
            f"{len(segments)} segment(s) to {job_dir}")


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

        # Nozzle rides on the flange -- same Delta_6 as Robot6, see docs/FR5_Mesh_Convention.md.
        # Registered but hidden (roadmap 7.1): the supervisor confirmed the tool=1
        # calibration is correct, so this asset -- 163.47mm flange-to-tip against
        # tool=1's 196.91mm -- is simply not the head that was calibrated. Kept
        # wired into rest_verts/update_fns so a corrected asset only needs the
        # set_enabled flag flipped back.
        nozzle = self.load_mesh(os.path.join(PRINTER_HEAD_DIR, NOZZLE_FILE))
        self.rest_verts.append(nozzle.vertices.copy())
        nozzle_handle = ps.register_surface_mesh("Nozzle", nozzle.vertices, nozzle.faces)
        nozzle_handle.set_enabled(False)
        self.mesh_handles.append(nozzle_handle)
        self.update_fns.append(nozzle_handle.update_vertex_positions)

        # Fixed flange->TCP transform for IK -- the real calibrated tool=1 offset
        # (roadmap 7.1), replacing S1.4's world point + rotation borrowed from
        # inv(T_zero[5]), which could not express a tool with its own orientation.
        self.T_flange_to_tcp = pose_to_matrix(*TCP_OFFSET_6D_MM_DEG)
        T_zero_tcp = self.T_zero[5] @ self.T_flange_to_tcp  # TCP pose at zero joints
        # .copy() so this is rest-pose data in its own right, not a view into
        # T_zero_tcp -- it is shared by the collision set and rest_verts, same
        # as every other entry in those lists (which all copy).
        tcp_point = T_zero_tcp[:3, 3].reshape(1, 3).copy()

        # Zero-pose bbox corners for the moving-geometry set (Robot1..6 + the tool).
        # The tool's collision body is the TCP point alone, NOT the nozzle mesh
        # (roadmap 7.1): colliding against a hidden asset of the wrong length would
        # reject poses on geometry the real head doesn't have. Its bbox is therefore
        # 8 coincident corners -- degenerate but harmless, the corners bound is then
        # exact. Excludes the TCP frame/axis appended below, which are visualization
        # markers, not solid geometry. See _meshes_clear_plane (roadmap 6.8).
        self.moving_geometry_rest_verts = self.rest_verts[:6] + [tcp_point]
        self.moving_geometry_rest_bbox_corners = [
            _bbox_corners(v) for v in self.moving_geometry_rest_verts]

        # Sampled link points and per-link oriented boxes for roadmap 7.4's
        # filters 6-9. Both are rest-frame data carried by the same rigid
        # Delta_i as the meshes, so neither is ever re-derived per candidate --
        # which is what makes a nine-filter stack affordable inside a search
        # that evaluates thousands of candidates per waypoint.
        #
        # Sampling rather than full vertex sets: filters 6-8 are clearance
        # tests at 1-20mm, and one representative point per LINK_SAMPLE_SPACING_MM
        # voxel resolves that comfortably while cutting the per-candidate point
        # count by orders of magnitude. Index 6 is the single TCP point, which
        # passes through both helpers unchanged.
        self.moving_geometry_rest_samples = [
            _voxel_downsample(v, LINK_SAMPLE_SPACING_MM) for v in self.moving_geometry_rest_verts]
        self.moving_geometry_rest_proxies = [
            _obb_proxies(v, SELF_COLLISION_PROXY_SEGMENT_MM)
            for v in self.moving_geometry_rest_verts]

        # TCP point, also Delta_6, but a Polyscope point cloud -- update_point_positions,
        # not update_vertex_positions, hence the per-object self.update_fns lookup
        self.rest_verts.append(tcp_point)
        point_cloud = ps.register_point_cloud("TCP", tcp_point)
        self.mesh_handles.append(point_cloud)
        self.update_fns.append(point_cloud.update_point_positions)

        # TCP orientation triad, also Delta_6 -- axis tips defined in the zero-pose
        # world frame around the TCP point, so they rotate with the tool via the same
        # Delta transform (curve network -> update_node_positions). The triad takes
        # the tool's own rest orientation, not world-aligned axes: with a real offset
        # the tool is genuinely rotated relative to the flange, and blue Z is the
        # nozzle approach axis the curved path targets.
        tcp_frame_handle, tcp_frame_rest_nodes = self.create_coordinate_frame(
            scale=TCP_FRAME_SCALE_MM, origin=T_zero_tcp[:3, 3],
            rotation=T_zero_tcp[:3, :3], name="TCP Frame")
        self.rest_verts.append(tcp_frame_rest_nodes)
        self.mesh_handles.append(tcp_frame_handle)
        self.update_fns.append(tcp_frame_handle.update_node_positions)

        # Flange->TCP stalk, also Delta_6 -- the only thing left showing where the
        # tool sits once the nozzle mesh is hidden (roadmap 7.1). Visual only: it is
        # deliberately absent from moving_geometry_rest_verts, since the flange->TCP
        # line was considered and rejected as a collision proxy in favour of the
        # point alone.
        tool_axis_nodes = np.array([self.T_zero[5][:3, 3], T_zero_tcp[:3, 3]])
        tool_axis_handle = ps.register_curve_network(
            "Tool Axis", tool_axis_nodes, np.array([[0, 1]]))
        tool_axis_handle.set_color(TOOL_AXIS_COLOR)
        tool_axis_handle.set_radius(TOOL_AXIS_RADIUS_MM, relative=False)
        self.rest_verts.append(tool_axis_nodes)
        self.mesh_handles.append(tool_axis_handle)
        self.update_fns.append(tool_axis_handle.update_node_positions)

        return meshes


    def apply_delta_transform(self, joint_angles_deg):
        """Update link mesh vertex positions for the given joint angles.

        Delta_i = T_0_i(q) @ inv(T_0_i(0)) -- see docs/FR5_Mesh_Convention.md.
        Robot0 is the fixed base and is never updated. The nozzle (index 6),
        TCP point (index 7), TCP frame (index 8) and flange->TCP tool axis
        (index 9) ride on the flange, reusing Delta_6 (index 5).
        """
        T_current = self.compute_fk(joint_angles_deg)
        for i in range(10):
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


    def _moving_geometry_deltas_from_fk(self, T_current):
        """_moving_geometry_deltas' body, given an already-computed FK. Split out
        for roadmap 7.4's filter stack, which needs the same FK for the elbow
        tests and the two collision tests and must not pay for it three times --
        compute_fk is six 4x4 multiplies in a Python loop and the stack runs on
        thousands of candidates per waypoint."""
        return [T_current[min(i, 5)] @ self.T_zero_inv[min(i, 5)] for i in range(7)]


    def _moving_geometry_deltas(self, joint_angles_deg):
        """Delta_i for each moving collision body (Robot1..Robot6 + the TCP
        point), reusing Delta_6 for the tool -- same src = min(i, 5) mapping as
        apply_delta_transform, but pure computation with no Polyscope side
        effects (used by the ground-clearance checks below, not per-frame
        rendering). Indexes moving_geometry_rest_verts, not rest_verts: index 6
        is the TCP point, not the hidden nozzle mesh (roadmap 7.1)."""
        return self._moving_geometry_deltas_from_fk(self.compute_fk(joint_angles_deg))


    def _meshes_clear_plane(self, joint_angles_deg, indices, point, normal, tol):
        """True if every moving mesh in `indices` stays outward of the plane
        through `point` with unit `normal`, allowing `tol` mm of inward slack
        (roadmap 6.8). A vertex's signed distance is (world - point) @ normal,
        positive on the outward (+normal) side; a mesh clears iff its worst
        (min) signed distance is >= -tol. `indices` are moving-geometry indices
        (0..5 = Robot1..6 arm links, 6 = the TCP point), matching
        _moving_geometry_deltas.

        Corners-first, exact vertices only if inconclusive: signed distance is
        linear, so its min over the rigid-transformed 8 AABB corners is a lower
        bound on its min over the true mesh -- a non-negative corner result
        proves clearance without touching the full vertex set. Each mesh is
        tested independently and a single penetrating mesh fails the whole set."""
        deltas = self._moving_geometry_deltas(joint_angles_deg)
        for i in indices:
            delta = deltas[i]
            for verts in (self.moving_geometry_rest_bbox_corners[i],
                          self.moving_geometry_rest_verts[i]):
                homo = np.hstack([verts, np.ones((len(verts), 1))])
                world = (delta @ homo.T).T[:, :3]
                if ((world - point) @ normal).min() + tol >= 0:
                    break  # this mesh clears (proven by corners, or by exact verts)
            else:
                return False  # neither bound cleared -> this mesh penetrates
        return True


    # _nozzle_clears_plane() lived here until roadmap 7.2 -- the S1.37
    # tangent-plane check, curved-only. Removed because roadmap 7.1 had already
    # made it incapable of rejecting anything: it tested the tool against the
    # plane through that waypoint, but 7.1 reduced the tool's collision body to
    # the single TCP point, which IK pins to that exact waypoint. Its signed
    # distance was therefore identically zero (measured: <1e-12 mm over all
    # 5,863 cached RX+TX waypoints and every candidate branch, against a 1.0mm
    # tolerance), so it returned True unconditionally. Deleting it changed no
    # accept/reject outcome. Nozzle-vs-workpiece protection was lost at 7.1, not
    # here -- see wiki/003_Guides/CurvedModel_PrintSetup.md.


    def _plate_plane(self):
        """The posed build-plate plane (roadmap 6.8), derived live from
        self.T_user_frame so it tracks wherever the Build Plate controls put
        the plate. Returns (point, normal): point on the plate's top/print face
        (T_user_frame origin lifted PLATE_THICKNESS_MM along local +Z, the same
        offset load_build_plate applies to the plate mesh), normal = plate local
        +Z (up). A vertex clears iff (world - point) @ normal >= -tol."""
        point = self.T_user_frame[:3, 3] + PLATE_THICKNESS_MM * self.T_user_frame[:3, 2]
        normal = self.T_user_frame[:3, 2]
        return point, normal


    # _branch_clears_ground() lived here until roadmap 7.4 -- the S1.40 posed-plate
    # check, which modelled the plate as an INFINITE plane and blocked arm links
    # 0-5 below it unconditionally, gating only the tool point on the
    # allow_tcp_through_plate toggle. Both it and the toggle are superseded by
    # filters 6 and 7 below (finite footprint + bounding slab), per settled.md
    # S1.46.
    #
    # It had to go rather than merely being layered under: S1.45 measured the
    # real User Frame sitting 323.5mm ABOVE the base, where the infinite plane
    # cuts through the shoulder and elbow -- links nowhere near the print -- and
    # rejected all 8 valid branches at planar waypoint 0 (deepest link signed
    # distance -253.2mm). S1.40's own prescribed fix, "move the plate lower", is
    # unavailable once the plate height is a measurement rather than a knob. A
    # real bed is finite and the arm legitimately reaches around it.
    #
    # _meshes_clear_plane() and _plate_plane() are both retained: filter 5 uses
    # the latter, and the former is still the cheap-corners/exact-verts bound.

    def _link_sample_points(self, deltas, indices):
        """World-space sampled surface points for the given moving-geometry
        indices, given this pose's Delta transforms -- the shared input to
        filters 6, 7 and 8 (roadmap 7.4). Returns (P,3).

        Computed once per candidate and passed to all three filters, rather than
        each transforming its own copy: the Delta multiply is the dominant cost
        of the collision half of the filter stack."""
        return np.vstack([transform_points(deltas[i], self.moving_geometry_rest_samples[i])
                          for i in indices])

    def _self_collision(self, deltas, ctx):
        """True if any non-adjacent link-proxy pair is closer than
        FILTER_SELF_COLLISION_CLEARANCE_MM -- filter 9 (roadmap 7.4).

        Every proxy box in the arm is transformed in one batch, then the pairs
        are narrowed by bounding spheres before the full separating-axis test
        runs on whatever survives. The pair list itself is built in
        _filter_context and excludes links less than three apart in the chain,
        which are in permanent contact by construction.

        The sphere pre-test is what makes filter 9 affordable: a box's bounding
        radius is |half|, so two proxies whose centres are further apart than the
        sum of their radii plus the clearance cannot possibly touch. It is one
        vectorised norm over the pair list, and in normal poses it eliminates all
        or nearly all of them -- measured 0.533ms -> 0.05ms per candidate, on a
        filter that runs thousands of times per waypoint."""
        prox_c, prox_a, prox_h, prox_link = ctx["proxies"]
        R = np.array([deltas[k][:3, :3] for k in range(7)])
        t = np.array([deltas[k][:3, 3] for k in range(7)])
        Rp, tp = R[prox_link], t[prox_link]
        centers = np.einsum('mij,mj->mi', Rp, prox_c) + tp

        ia, ib = ctx["proxy_pairs"]
        radii = ctx["proxy_radii"]
        gap = np.linalg.norm(centers[ib] - centers[ia], axis=1)
        near = gap < radii[ia] + radii[ib] + FILTER_SELF_COLLISION_CLEARANCE_MM
        if not np.any(near):
            return False
        ia, ib = ia[near], ib[near]

        axes = np.einsum('maj,mij->mai', prox_a, Rp)  # rows stay unit directions
        separated = _obbs_separated_batch(
            centers[ia], axes[ia], prox_h[ia],
            centers[ib], axes[ib], prox_h[ib],
            FILTER_SELF_COLLISION_CLEARANCE_MM)
        return not np.all(separated)

    def _plate_box_frame(self):
        """(inverse plate transform, local min, local max) for filters 6 and 7 --
        the finite plate model that replaces S1.40's infinite plane (roadmap
        7.4). Working in plate-local coordinates means the footprint and slab
        tests are plain axis-aligned comparisons however the plate is posed."""
        return np.linalg.inv(self.T_user_frame), self.plate_local_bounds[0], self.plate_local_bounds[1]

    def _candidate_admissible(self, joints, ctx):
        """The nine candidate filters of roadmap 7.4 / settled.md S1.46, adapted
        from IK_BRANCH_REJECTION_GUIDE.md. Returns None if the candidate is
        admissible, otherwise the SHORT NAME of the first filter it failed (for
        the per-reason tally the precompute reports on failure).

        Filter 1 (joint limits) is not here: solve_ik_tcp_matrix has already
        enforced PHYSICAL_JOINT_LIMITS before a candidate reaches this point.

        Order is load-bearing and matches the reference: pure arithmetic first,
        then a single FK, then the collision tests, rejecting on the FIRST
        failure so the expensive tail rarely runs. ctx carries the per-run
        constants (surface grid, plate frame) so nothing is rebuilt per
        candidate.
        """
        # --- Filter 2: J5 minimum (arithmetic) ---
        # Negative J5 flips the wrist, giving an upside-down tool approach. Set
        # at 2.0 rather than the reference's 0.0 so the exchange spec's row 7
        # |J5| < 2deg singularity WARN is also unreachable -- see FILTER_J5_MIN_DEG.
        if joints[4] < FILTER_J5_MIN_DEG:
            return "J5"

        # --- Filter 3: J4 minimum (arithmetic, opt-in) ---
        if FILTER_J4_ENABLED and joints[3] < FILTER_J4_MIN_DEG:
            return "J4"

        # --- The one FK, shared by filters 4-9 ---
        T = self.compute_fk(joints)
        deltas = self._moving_geometry_deltas_from_fk(T)
        shoulder, elbow, wrist = T[0][:3, 3], T[1][:3, 3], T[2][:3, 3]

        # --- Filter 4: upper-branch configuration ---
        # The elbow must stand above the shoulder->wrist chord. Rejects
        # lower-elbow poses and, because a straight arm puts the elbow ON the
        # chord, near-singular ones too -- those have unpredictable velocity and
        # can flip suddenly.
        chord = wrist - shoulder
        chord_len = np.linalg.norm(chord)
        if chord_len < 1e-9:
            return "upper-branch"  # shoulder and wrist coincident: degenerate
        chord_dir = chord / chord_len
        offset = (elbow - shoulder) - np.dot(elbow - shoulder, chord_dir) * chord_dir
        if offset[2] < FILTER_UPPER_BRANCH_TOL_MM:
            return "upper-branch"

        # --- Filter 5: elbow above the build-plate plane ---
        point, normal = ctx["plate_plane"]
        if np.dot(elbow - point, normal) < -FILTER_ELBOW_PLATE_TOL_MM:
            return "elbow-plate"

        # --- Filters 6-8 share one transformed sample set ---
        samples = self._link_sample_points(deltas, ctx["sample_indices"])
        T_inv, lo, hi = ctx["plate_box"]
        local = transform_points(T_inv, samples)

        # --- Filter 6: under-plate footprint ---
        # No sample may sit inside the plate's XY shadow AND below its print
        # face. This is the S1.40 fix: unlike an infinite plane it says nothing
        # about an arm link that is merely lower than the plate but well outside
        # its footprint, which is the normal situation for a plate mounted high.
        # The XY margin catches near-misses at the plate edge.
        m = FILTER_UNDER_PLATE_MARGIN_MM
        under = ((local[:, 0] >= lo[0] - m) & (local[:, 0] <= hi[0] + m) &
                 (local[:, 1] >= lo[1] - m) & (local[:, 1] <= hi[1] + m) &
                 (local[:, 2] < hi[2]))
        if np.any(under):
            return "under-plate"

        # --- Filter 7: plate volume slab ---
        # Catches link-through-plate cases the footprint test misses, e.g. a
        # wrist passing through the plate edge from the side.
        c = FILTER_PLATE_SLAB_CLEARANCE_MM
        inside = np.all((local >= lo - c) & (local <= hi + c), axis=1)
        if np.any(inside):
            return "plate-slab"

        # --- Filter 8: surface mesh collision (curved runs only) ---
        # The FIRST mesh-vs-mesh check in this project. S1.37 declined to build
        # it, arguing a full-arm obstacle test "would reject every real printing
        # pose" -- true only while ONE orientation is commanded per waypoint, and
        # 7.4 searches 540. Until now nothing at all stopped a solved TX run
        # driving the arm through the shoulder mockup.
        if ctx["surface_grid"] is not None:
            if not _points_clear_surface(ctx["surface_grid"], samples,
                                         CURVED_TIP_CLEARANCE_TOLERANCE_MM):
                return "surface"

        # --- Filter 9: robot/tool self-collision ---
        if self._self_collision(deltas, ctx):
            return "self-collision"

        return None

    def _filter_context(self, filter_mode, layer=None):
        """Per-RUN constants the filter stack reads -- built once at precompute
        begin, never per candidate (roadmap 7.4). filter_mode is "planar" or
        "curved"; the only difference is filter 8, which needs a print surface
        the planar path does not have.

        The surface grid is the expensive one: Surface_TX_Base is ~90k triangles,
        trivial to bin once and ruinous to rebin per waypoint."""
        grid = None
        if filter_mode == "curved" and layer is not None and self.curved_model_loaded:
            grid = _build_surface_grid(self.curved_surface_verts_world[layer],
                                       self.curved_surface_faces[layer],
                                       SURFACE_GRID_CELL_MM)
        # Moving-geometry indices sampled for filters 6-8: the six ARM LINKS
        # only (0-5), deliberately NOT the tool point at index 6.
        #
        # The tool's whole collision body has been the single TCP point since
        # roadmap 7.1, and IK pins that point to the commanded waypoint -- which
        # lies ON the print surface during a feed move, and at exactly the
        # plate's top face on the planar path's first layer. Testing it against
        # either would reject every legitimate printing pose, and against the
        # plate would additionally turn float noise at z == hi[2] into a
        # waypoint-0 abort. This is the same trap that made S1.37's nozzle check
        # incapable of rejecting anything (measured: <1e-12mm signed distance
        # over all 5,863 cached waypoints, roadmap 7.2), and it is what the
        # reference guide's nozzle_tip_exclusion_mm exists to avoid.
        #
        # Consequence worth stating plainly: nothing here guards the NOZZLE
        # against the workpiece -- only the arm. Recovering that needs a real
        # tool body, which no asset currently provides (7.1 found nozzle.obj is
        # 163.47mm against tool=1's 196.91mm and hid it).
        sample_indices = tuple(range(6))

        # Filter 9's link pairs: at least THREE apart in the kinematic chain.
        #
        # Adjacent links (i, i+1) share a joint and are permanently in contact,
        # so nobody tests those. Links two apart are the subtle case, and the
        # answer here is measured rather than assumed: (i, i+2) is separated by
        # just one short link, and on the FR5's compact wrist (d4/d5/d6 =
        # 102/102/100mm) their meshes interpenetrate at every pose. Robot4~Robot6
        # was reported as colliding in ALL 8 branches at planar waypoint 0 with a
        # true mesh gap of ~30mm, and no joint value separates them -- their
        # relative motion is one joint rotation about a shared axis. Testing them
        # rejects every pose the arm can hold, which is a broken filter, not a
        # safe one.
        #
        # Three or more apart is where a link can actually fold back onto another
        # (the forearm reaching the shoulder, the flange reaching the upper arm),
        # which is what filter 9 is for. The tool point sits at chain position 6,
        # so the same rule pairs it with Robot1..Robot4.
        link_pairs = [(i, j) for i in range(6) for j in range(i + 3, 6)]
        link_pairs += [(6, k) for k in range(4)]

        # Flatten every link's proxy boxes into one array so a candidate can
        # transform them all in a single batch, and pre-expand link_pairs into
        # the proxy-index pairs the batched SAT consumes. Both are fixed for the
        # whole run -- only the transforms change per candidate.
        centers, axes, halfs, link_of = [], [], [], []
        offset = {}
        for k in range(7):
            c, a, h = self.moving_geometry_rest_proxies[k]
            offset[k] = len(centers)
            centers.extend(c); axes.extend(a); halfs.extend(h)
            link_of.extend([k] * len(c))
        proxies = (np.array(centers), np.array(axes), np.array(halfs),
                   np.array(link_of, dtype=np.int64))
        # Bounding-sphere radius per proxy box, for _self_collision's pre-test.
        proxy_radii = np.linalg.norm(np.array(halfs), axis=1)

        ia, ib = [], []
        for i, j in link_pairs:
            ni = len(self.moving_geometry_rest_proxies[i][0])
            nj = len(self.moving_geometry_rest_proxies[j][0])
            for p in range(ni):
                for q in range(nj):
                    ia.append(offset[i] + p); ib.append(offset[j] + q)

        return {
            "plate_plane": self._plate_plane(),
            "plate_box": self._plate_box_frame(),
            "surface_grid": grid,
            "sample_indices": sample_indices,
            "proxies": proxies,
            "proxy_radii": proxy_radii,
            "proxy_pairs": (np.array(ia, dtype=np.int64), np.array(ib, dtype=np.int64)),
        }


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
    without transforming every vertex -- see _meshes_clear_plane."""
    lo, hi = verts.min(axis=0), verts.max(axis=0)
    xs, ys, zs = np.meshgrid([lo[0], hi[0]], [lo[1], hi[1]], [lo[2], hi[2]], indexing='ij')
    return np.stack([xs.ravel(), ys.ravel(), zs.ravel()], axis=1)


def transform_points(T, points):
    """Apply a 4x4 homogeneous transform to an Nx3 point array."""
    homo = np.hstack([points, np.ones((len(points), 1))])
    return (T @ homo.T).T[:, :3]


# ===========================================================================
# Collision primitives -- roadmap 7.4, filters 6-9 (settled.md S1.46).
#
# All from scratch in numpy: there is no scipy in the fairino-fr5-sim
# environment (confirmed 2026-09-03: numpy/polyscope/trimesh only), and adding
# it was considered and rejected for 7.4 -- its one real win here would have
# been a cKDTree broadphase for filter 8, against a new runtime dependency and
# ten "no scipy" decisions across the code and wiki. See _build_surface_grid.
# ===========================================================================


def _voxel_downsample(verts, spacing):
    """One representative vertex per `spacing`-mm cubic cell -- the link sample
    points filters 6-8 test (roadmap 7.4). Deterministic: cells are visited in
    np.unique's sorted order and the first vertex falling in each is kept, so
    the same mesh always yields the same samples and a cached solve stays
    reproducible.

    Sampling rather than using every vertex is what makes the filters
    affordable: the FR5 link meshes carry tens of thousands of vertices each,
    and a clearance test at 1-5mm does not need them. A degenerate input (a
    single point, e.g. the TCP) passes straight through."""
    verts = np.asarray(verts, dtype=float)
    if len(verts) <= 1:
        return verts.copy()
    cell = np.floor(verts / spacing).astype(np.int64)
    _, first = np.unique(cell, axis=0, return_index=True)
    return verts[np.sort(first)]


def _obb_from_points(verts):
    """Oriented bounding box of a point set, as (center, axes (3,3), half (3,)).
    Axes are the principal directions (unit rows), half the extents along them.

    Fitted in the points' OWN frame and used that way: filter 9 transforms the
    box by the link's rigid Delta rather than re-fitting per candidate, which is
    exact for a rigid motion and is the whole reason this is affordable inside a
    per-candidate loop. PCA via np.linalg.eigh on the 3x3 covariance -- no scipy
    needed for a problem this size.

    A degenerate set (<2 points, or a flat/linear cloud) still returns a valid
    box: eigh gives an orthonormal basis regardless, and zero half-extents are
    harmless to the SAT test below."""
    verts = np.asarray(verts, dtype=float)
    mean = verts.mean(axis=0)
    if len(verts) < 2:
        return mean, np.eye(3), np.zeros(3)
    centered = verts - mean
    # eigh (not eig) -- the covariance is symmetric, so this returns real,
    # orthonormal eigenvectors in ascending eigenvalue order.
    _, vecs = np.linalg.eigh(centered.T @ centered)
    axes = vecs.T  # rows are the principal directions
    projected = centered @ axes.T
    lo, hi = projected.min(axis=0), projected.max(axis=0)
    # Centre on the box, not on the mean: a lopsided point distribution puts the
    # mean off-centre, and half-extents taken about the mean would then bound a
    # larger box than the points actually occupy.
    return mean + ((lo + hi) / 2.0) @ axes, axes, (hi - lo) / 2.0


def _obb_proxies(verts, segment_mm):
    """A link's collision proxy: several oriented boxes along its length rather
    than one -- filter 9 (roadmap 7.4). Returns (centers (K,3), axes (K,3,3),
    halfs (K,3)).

    ONE box per link is far too loose to be usable, and this is measured rather
    than assumed: Robot3's single OBB is 502mm long, so it reports contact with
    Robot5 and Robot6 in *every* IK branch at planar waypoint 0, where the true
    mesh gap is 20-35mm against a 5mm clearance. A whole-arm filter that rejects
    every pose is worse than no filter, because it looks like a safety result.

    So the points are split into `segment_mm` bands along the link's principal
    axis and a box is fitted per band, which is what the reference guide means
    by its "multi-proxy OBB distance". Bands, not a fixed count, so a long link
    gets more boxes than a short one and the tightness is uniform.

    Empty bands are skipped; a link shorter than one band yields a single box,
    identical to _obb_from_points."""
    verts = np.asarray(verts, dtype=float)
    if len(verts) < 2:
        c, a, h = _obb_from_points(verts)
        return c[None, :], a[None, :, :], h[None, :]

    _, axes, half = _obb_from_points(verts)
    principal = axes[np.argmax(half)]  # split along the link's longest direction
    t = verts @ principal
    span = t.max() - t.min()
    n_bands = max(1, int(np.ceil(span / segment_mm)))
    edges = np.linspace(t.min(), t.max(), n_bands + 1)
    edges[-1] += 1e-6  # so the extreme point lands in the last band

    centers, all_axes, halfs = [], [], []
    for b in range(n_bands):
        band = verts[(t >= edges[b]) & (t < edges[b + 1])]
        if len(band) == 0:
            continue
        c, a, h = _obb_from_points(band)
        centers.append(c); all_axes.append(a); halfs.append(h)
    return np.array(centers), np.array(all_axes), np.array(halfs)


def _obbs_separated_batch(ca, aa, ha, cb, ab, hb, clearance):
    """Vectorised separating-axis test over P box PAIRS at once -- filter 9's
    hot path (roadmap 7.4). Each argument is stacked per pair: centers (P,3),
    axes (P,3,3) with unit rows, halfs (P,3). Returns a (P,) bool, True where
    that pair is separated by more than `clearance`.

    Standard 15-axis SAT (3 face normals per box, 9 edge cross-products),
    vectorised over every pair and every axis in one numpy pass rather than a
    Python loop: the per-pair form costs ~0.2ms/pair, and filter 9 tests ~100
    proxy pairs per candidate with thousands of candidates per waypoint, so
    looping would dominate the entire precompute. `clearance` inflates one
    box's extents, so pairs that merely come close count as touching. Verified
    by hand against a plain per-pair SAT loop during development (identical
    results on 300 random pairs, both clearances)."""
    P = len(ca)
    ha = ha + clearance  # inflate one box of each pair; separation is symmetric

    # (P,15,3) candidate axes: 3 faces of A, 3 of B, 9 edge cross-products.
    axes = np.empty((P, 15, 3))
    axes[:, 0:3] = aa
    axes[:, 3:6] = ab
    k = 6
    for i in range(3):
        for j in range(3):
            axes[:, k] = np.cross(aa[:, i], ab[:, j])
            k += 1

    norms = np.linalg.norm(axes, axis=2)
    valid = norms > 1e-9           # near-parallel edges: covered by the face normals
    axes = axes / np.where(valid[..., None], norms[..., None], 1.0)

    t = (cb - ca)[:, None, :]
    reach_a = np.einsum('pna,pa->pn', np.abs(np.einsum('pnk,pak->pna', axes, aa)), ha)
    reach_b = np.einsum('pna,pa->pn', np.abs(np.einsum('pnk,pak->pna', axes, ab)), hb)
    gap = np.abs(np.einsum('pnk,pnk->pn', t, axes)) - (reach_a + reach_b)
    return np.any(valid & (gap > 0.0), axis=1)


def _build_surface_grid(verts, faces, cell_size):
    """Uniform-grid broadphase over a triangle mesh -- filter 8 (roadmap 7.4).
    Returns a dict the query below consumes: triangle vertex arrays plus a
    {cell -> triangle indices} map, keyed by integer cell coordinate.

    Each triangle is registered in every cell its AABB touches, so a query point
    only has to look at its own cell and the 26 around it. Built ONCE per
    precompute run (the print surface does not move mid-solve), not per waypoint
    -- Surface_TX_Base is 45,430 verts / ~90k triangles, which would be
    ruinous per candidate and is trivial once.

    A dict-of-lists rather than the flat CSR layout build_surface_graph() uses:
    that one is walked ~V log V times inside a hot Dijkstra loop where the list
    indexing measurably won, whereas this is a handful of hash lookups per
    query point. Different access pattern, different structure.

    Alongside it, a DENSE boolean occupancy array over the surface's own
    bounding box, dilated by one cell. That is the part the query actually leans
    on: it turns "is this point anywhere near the surface?" into one vectorised
    array lookup for all points at once, so the dict -- and the Python loop over
    it -- is only ever touched for the handful of points that are genuinely
    close. The array is small (the print surfaces span a few hundred mm, so a
    few thousand cells at 8mm) and dilation is 27 slice-ORs, no scipy."""
    verts = np.asarray(verts, dtype=float)
    faces = np.asarray(faces)
    tri = verts[faces]  # (F,3,3)

    lo = np.floor(tri.min(axis=1) / cell_size).astype(np.int64)   # (F,3)
    hi = np.floor(tri.max(axis=1) / cell_size).astype(np.int64)

    grid = {}
    for f in range(len(faces)):
        for x in range(lo[f, 0], hi[f, 0] + 1):
            for y in range(lo[f, 1], hi[f, 1] + 1):
                for z in range(lo[f, 2], hi[f, 2] + 1):
                    grid.setdefault((x, y, z), []).append(f)

    # Dense occupancy over the cell bounding box, with a one-cell margin so the
    # dilation below has somewhere to grow into.
    origin = lo.min(axis=0) - 1
    shape = tuple(hi.max(axis=0) - origin + 2)
    occupied = np.zeros(shape, dtype=bool)
    keys = np.array(list(grid.keys())) - origin
    occupied[keys[:, 0], keys[:, 1], keys[:, 2]] = True

    # Dilate by one cell in every direction: a point sitting in a dilated cell is
    # within one cell of a triangle, and one cell (8mm) comfortably exceeds the
    # 1mm clearance, so nothing within range can be screened out.
    dilated = np.zeros_like(occupied)
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                dilated[max(dx, 0):shape[0] + min(dx, 0),
                        max(dy, 0):shape[1] + min(dy, 0),
                        max(dz, 0):shape[2] + min(dz, 0)] |= \
                    occupied[max(-dx, 0):shape[0] + min(-dx, 0),
                             max(-dy, 0):shape[1] + min(-dy, 0),
                             max(-dz, 0):shape[2] + min(-dz, 0)]

    return {"tri": tri, "grid": {k: np.array(v) for k, v in grid.items()},
            "cell_size": cell_size, "origin": origin, "near": dilated}


def _segment_distance2(points, p0, p1):
    """Squared distance from each of P points to each of F segments p0->p1.
    points (P,3), p0/p1 (F,3); returns (P,F). Helper for
    _point_triangle_distance2."""
    d = p1 - p0                                        # (F,3)
    dd = np.maximum(np.einsum('fk,fk->f', d, d), 1e-20)[None, :]
    w = points[:, None, :] - p0[None, :, :]            # (P,F,3)
    u = np.clip(np.einsum('pfk,fk->pf', w, d) / dd, 0.0, 1.0)
    diff = w - u[..., None] * d[None, :, :]
    return np.einsum('pfk,pfk->pf', diff, diff)


def _point_triangle_distance2(points, tri):
    """Squared distance from each of P points to each of F triangles --
    filter 8's narrowphase (roadmap 7.4). points (P,3), tri (F,3,3); returns
    (P,F).

    Exact, and deliberately structured to be obviously so. The closest point of
    a triangle is either interior -- in which case it is the perpendicular
    footpoint, and the barycentric coordinates of that footpoint are all
    non-negative and sum to <= 1 -- or it lies on the boundary, which is exactly
    the union of the three edge segments. So: take the perpendicular distance
    where the footpoint is inside, and the best of the three edges everywhere
    else. The three edges already cover the vertices, so there is no separate
    vertex case.

    This is the min-of-four formulation rather than the seven-region case split
    (Ericson). It does strictly more arithmetic, but every branch is a whole-
    array np.where instead of a per-region index dance, and getting a region
    wrong here means silently MISSING a collision -- the unsafe direction for a
    filter whose whole job is to stop the arm entering the workpiece."""
    a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]          # (F,3) each
    ab, ac = b - a, c - a
    ap = points[:, None, :] - a[None, :, :]            # (P,F,3)

    aa = np.einsum('fk,fk->f', ab, ab)[None, :]
    bb = np.einsum('fk,fk->f', ac, ac)[None, :]
    ab_ac = np.einsum('fk,fk->f', ab, ac)[None, :]
    d1 = np.einsum('pfk,fk->pf', ap, ab)
    d2 = np.einsum('pfk,fk->pf', ap, ac)

    det = aa * bb - ab_ac ** 2
    safe_det = np.where(np.abs(det) > 1e-20, det, 1.0)
    s = (bb * d1 - ab_ac * d2) / safe_det
    t = (aa * d2 - ab_ac * d1) / safe_det

    # Interior footpoint: distance to the triangle's plane. Degenerate (zero
    # area) triangles have no interior, so they fall through to the edges.
    inside = (np.abs(det) > 1e-20) & (s >= 0.0) & (t >= 0.0) & (s + t <= 1.0)
    foot = a[None, :, :] + s[..., None] * ab[None, :, :] + t[..., None] * ac[None, :, :]
    diff = points[:, None, :] - foot
    plane_d2 = np.einsum('pfk,pfk->pf', diff, diff)

    edge_d2 = np.minimum(np.minimum(_segment_distance2(points, a, b),
                                    _segment_distance2(points, b, c)),
                         _segment_distance2(points, a, c))
    return np.where(inside, plane_d2, edge_d2)


def _points_clear_surface(surface_grid, points, clearance):
    """True if every point stays further than `clearance` mm from the meshed
    surface -- filter 8 (roadmap 7.4). points is (P,3) world mm.

    Two stages, and the split is what makes this affordable. First a single
    vectorised lookup in the dilated occupancy array screens ALL points at once:
    anything outside the surface's cell bounding box, or in a cell with no
    triangle within one cell, is clear and is dropped. Only survivors reach the
    dict-and-narrowphase loop, and in a normal pose there are none -- the arm
    links are mostly nowhere near the print surface.

    Doing it the other way round -- looping in Python over every sample point
    and hashing 27 keys each -- was measured at ~8.5s per WAYPOINT on the curved
    path, against ~0.9s for everything else combined. Same answer, 100x the cost.

    Correctness of the screen: the occupancy array is dilated by one cell, and
    one cell (SURFACE_GRID_CELL_MM = 8mm) is far larger than `clearance` (1mm),
    so no point within range of a triangle can be screened out.

    Fails on the FIRST offending point rather than measuring them all -- this is
    a reject/accept gate inside a per-candidate loop, not a distance report."""
    cell_size = surface_grid["cell_size"]
    grid, tri = surface_grid["grid"], surface_grid["tri"]
    near, origin = surface_grid["near"], surface_grid["origin"]
    limit2 = clearance ** 2

    pts = np.atleast_2d(points)
    cells = np.floor(pts / cell_size).astype(np.int64) - origin
    inside = np.all((cells >= 0) & (cells < np.array(near.shape)), axis=1)
    if not np.any(inside):
        return True
    candidates = np.zeros(len(pts), dtype=bool)
    ci = cells[inside]
    candidates[inside] = near[ci[:, 0], ci[:, 1], ci[:, 2]]
    if not np.any(candidates):
        return True

    offsets = [(dx, dy, dz) for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)]
    for p, base in zip(pts[candidates], (cells[candidates] + origin)):
        nearby = [grid[key] for key in
                  ((base[0] + dx, base[1] + dy, base[2] + dz) for dx, dy, dz in offsets)
                  if key in grid]
        if not nearby:
            continue
        faces = np.unique(np.concatenate(nearby))
        if _point_triangle_distance2(p[None, :], tri[faces]).min() < limit2:
            return False
    return True


def bead_faces(face_template, K, reveal_index, u_valid, width_valid=None):
    """Triangle indices for K bead boxes, with always-hidden cap faces culled.

    Drop cap faces at bead-to-bead boundaries that are provably always hidden:
    back-to-back in the source path (no travel gap), colinear (same travel
    direction -- at a turn the two cap planes meet at an angle, not coincide),
    and, where widths vary, width-matched (same cross-section, so no ledge is
    exposed). ~8% of triangles on a real multi-layer print, since most
    consecutive segments trace a curved surface and fail the colinearity test
    -- settled.md S1.19.

    width_valid=None skips the width-match term, for callers whose beads have a
    fixed cross-section (the curved path, _build_curved_beads).

    Returns (faces, bead_face_prefix):
      faces: (<=K*12, 3) int, indices into a bead-major (8 rows per bead)
        vertex array -- fewer than 12 per bead wherever a cap was culled, so
        not a fixed per-bead stride.
      bead_face_prefix: (K+1,) int cumulative triangle count, so
        `faces[:bead_face_prefix[n]]` is exactly the first n beads
        (settled.md S1.20).
    """
    chained = np.diff(reveal_index) == 1
    colinear = np.sum(u_valid[:-1] * u_valid[1:], axis=1) >= CAP_CULL_COLINEAR_DOT_MIN
    cullable = chained & colinear
    if width_valid is not None:
        cullable = cullable & (np.abs(width_valid[:-1] - width_valid[1:]) <= CAP_CULL_WIDTH_TOL_MM)

    drop_end_cap = np.zeros(K, dtype=bool)    # bead k's end cap (template rows 8-9)
    drop_start_cap = np.zeros(K, dtype=bool)  # bead k's start cap (template rows 4-5)
    drop_end_cap[:-1] = cullable
    drop_start_cap[1:] = cullable

    keep_row = np.ones((K, 12), dtype=bool)
    keep_row[drop_end_cap, 8] = False
    keep_row[drop_end_cap, 9] = False
    keep_row[drop_start_cap, 4] = False
    keep_row[drop_start_cap, 5] = False

    faces_full = (face_template[None, :, :] + (np.arange(K) * 8)[:, None, None])
    return faces_full[keep_row], np.concatenate([[0], np.cumsum(keep_row.sum(axis=1))])


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


def orientation_candidates(nominal_R):
    """The commanded TCP orientations searched at one waypoint -- roadmap 7.4,
    settled.md S1.46. nominal_R is the (3,3) frame whose Z column is the exact
    outward surface normal (what _orientation_frames_for_points still returns);
    returns (ORIENT_SEARCH_FRAMES, 3, 3).

    Two DOF are swept, for two different reasons:

    - **Tool axis**, over a cone of half-angle ORIENT_SEARCH_TILT_MAX_DEG about
      the nominal normal. This is the supervisor's relaxation: perpendicular
      *within* 20 deg rather than exactly. It is the only part of 7.4 that
      loosens anything.
    - **Roll about that axis**, all ORIENT_SEARCH_ROLL_SLOTS of it. This DOF was
      always free -- the nozzle is rotationally symmetric, which is S1.36's own
      reasoning for pinning it -- so sweeping it costs nothing physically and
      buys both reach (the flange->TCP offset is lateral, so rolling relocates
      the wrist centre) and continuity (the caller resolves it by graph cost
      instead of a per-waypoint argmin, which is what caused the row-5 flips).

    Emitted in a fixed order, tilt-major, so a candidate's index decomposes as
    (frame // ROLL_SLOTS, frame % ROLL_SLOTS) = (tilt_idx, roll_idx). The edge
    cost reads roll_idx to charge for roll jumps, so the ordering is load-bearing
    rather than incidental. Index 0 is tilt 0 / roll 0, i.e. the nominal axis --
    so the pre-7.4 commanded direction is always in the set."""
    z = np.asarray(nominal_R)[:, 2]
    z = z / np.linalg.norm(z)
    x = np.asarray(nominal_R)[:, 0]
    x = x - np.dot(x, z) * z
    nx = np.linalg.norm(x)
    if nx < 1e-9:
        # nominal_R's X was (numerically) parallel to Z -- pick any perpendicular.
        x = np.cross(z, np.eye(3)[np.argmin(np.abs(z))])
        nx = np.linalg.norm(x)
    x = x / nx
    y = np.cross(z, x)

    tilt = np.deg2rad(ORIENT_SEARCH_TILT_MAX_DEG)
    axes = [z]
    for k in range(ORIENT_SEARCH_TILT_RING_AZIMUTHS):
        phi = 2.0 * np.pi * k / ORIENT_SEARCH_TILT_RING_AZIMUTHS
        # Tilt z by `tilt` toward the in-plane direction at azimuth phi.
        axes.append(np.cos(tilt) * z + np.sin(tilt) * (np.cos(phi) * x + np.sin(phi) * y))

    frames = np.empty((ORIENT_SEARCH_FRAMES, 3, 3))
    i = 0
    for axis in axes:
        axis = axis / np.linalg.norm(axis)
        # A reference perpendicular for this axis, taken from the nominal X so
        # roll slot 0 means the same thing across the cone.
        ref = x - np.dot(x, axis) * axis
        if np.linalg.norm(ref) < 1e-9:
            ref = y - np.dot(y, axis) * axis
        ref = ref / np.linalg.norm(ref)
        perp = np.cross(axis, ref)
        for r in range(ORIENT_SEARCH_ROLL_SLOTS):
            theta = 2.0 * np.pi * r / ORIENT_SEARCH_ROLL_SLOTS
            cx = np.cos(theta) * ref + np.sin(theta) * perp
            frames[i] = np.column_stack([cx, np.cross(axis, cx), axis])
            i += 1
    return frames


def dijkstra_candidate_path(layer_joints, layer_roll, layer_branch, layer_is_feed):
    """Globally optimal joint trajectory through the candidate DAG -- roadmap
    7.4 step 4, settled.md S1.46 part 4.

    Nodes are `(waypoint i, candidate c)`; edges run only from layer i to layer
    i+1. Every argument is a per-waypoint list, all four the same length W, with
    entry i describing that waypoint's C_i surviving candidates:
      layer_joints[i]   (C_i, 6) float, joint angles in degrees
      layer_roll[i]     (C_i,)   int, roll slot index (for the roll-step cost)
      layer_branch[i]   (C_i,)   int, raw IK branch ordinal (for the family cost)
      layer_is_feed[i]  bool, whether this waypoint extrudes

    Returns (chosen (W,) int, total_cost float), or (None, dead_layer_index)
    when no path exists -- either a layer has no admissible candidate at all, or
    every edge into it is forbidden. There is deliberately NO relaxation and no
    fallback to a less-safe candidate: matching the reference implementation,
    a job that cannot be planned within the filters fails loudly.

    ## Why this is a second Dijkstra rather than a call to dijkstra_surface()

    Same primitive, different graph -- settled.md S1.31 built the geodesic one
    over a *general* CSR mesh graph, where the frontier order is genuinely
    unknown and a heap is the only way to settle nodes cheaply. This graph is
    strictly layered: every edge goes from layer i to layer i+1, so the
    topological order IS the waypoint index, each node settles exactly once when
    its layer is reached, and the heap has nothing left to decide. Dropping it
    is not an approximation -- the path returned is identical -- it just lets a
    whole layer's edge block be relaxed as ONE vectorised numpy operation.

    That difference is the whole reason this runs. Curved RX is ~2,900 waypoints
    at up to ~4,320 candidates each: a heapq frontier over ~12.5M nodes and
    ~5x10^10 edges in interpreted Python does not finish, whereas the same
    arithmetic as ~2,900 numpy block operations does.

    ## The edge cost

    Weighted-L1 joint movement (EDGE_JOINT_WEIGHTS, proximal joints dearest, so
    redundancy resolves out at the wrist), plus a flat EDGE_BRANCH_CHANGE_PENALTY
    for switching IK family, plus a quadratic penalty on roll jumps beyond one
    slot. Continuity is therefore a *cost*, which is what lets an early
    non-greedy choice be taken to avoid a later dead end -- the failure mode the
    superseded per-waypoint ranking (S1.5/S1.11) could not recover from.

    ## The one hard rejection, and its scope

    An edge is forbidden (inf) when any joint moves more than
    EDGE_MAX_JOINT_STEP_DEG -- but ONLY between two feed waypoints. The exchange
    spec's row 5 measures steps *within* a continuous extrusion line, and travel
    moves are legitimately large: the planar path's max step is 57.32 deg overall
    against 5.85 deg inside a segment. Applying this across a G0 boundary would
    abort a job that the receiving side would happily accept."""
    n_layers = len(layer_joints)
    if n_layers == 0:
        return None, 0
    if len(layer_joints[0]) == 0:
        return None, 0

    dist = np.zeros(len(layer_joints[0]))
    back = []  # back[i-1][c] = chosen candidate index in layer i-1 for layer i's c

    for i in range(1, n_layers):
        q_prev, q_curr = layer_joints[i - 1], layer_joints[i]
        if len(q_curr) == 0:
            return None, i

        # (Ca, Cb, 6) per-joint absolute movement -- the one big allocation.
        D = np.abs(q_curr[None, :, :] - q_prev[:, None, :])
        cost = D @ EDGE_JOINT_WEIGHTS

        cost = cost + EDGE_BRANCH_CHANGE_PENALTY * (
            layer_branch[i][None, :] != layer_branch[i - 1][:, None])

        roll_d = np.abs(layer_roll[i][None, :].astype(float)
                        - layer_roll[i - 1][:, None].astype(float))
        roll_d = np.minimum(roll_d, ORIENT_SEARCH_ROLL_SLOTS - roll_d)  # slots wrap
        cost = cost + EDGE_ROLL_QUADRATIC_WEIGHT * np.maximum(0.0, roll_d - 1.0) ** 2

        if layer_is_feed[i - 1] and layer_is_feed[i]:
            cost = np.where(D.max(axis=-1) > EDGE_MAX_JOINT_STEP_DEG, np.inf, cost)

        total = dist[:, None] + cost
        best = np.argmin(total, axis=0)
        dist = total[best, np.arange(total.shape[1])]
        back.append(best)

        if not np.any(np.isfinite(dist)):
            return None, i

    chosen = np.empty(n_layers, dtype=np.int64)
    chosen[-1] = int(np.argmin(dist))
    for i in range(n_layers - 1, 0, -1):
        chosen[i - 1] = back[i - 1][chosen[i]]
    return chosen, float(dist[chosen[-1]])


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


def pose_to_matrix(x, y, z, rx, ry, rz):
    """6D pose [mm, mm, mm, deg, deg, deg] -> 4x4 homogeneous transform.

    The supervisor docs' one conversion for every 6D pose they publish (TCP
    offsets, User Frame) -- docs/saved_coords_data_and_usage_EN.md 3,
    R = Rz(rz) @ Ry(ry) @ Rx(rx). Not a new convention: that is already what
    solve_ik_tcp and load_build_plate compose inline from rot_x/rot_y/rot_z,
    so this builds on those rather than restating the matrices.
    """
    T = np.eye(4)
    T[:3, 3] = [x, y, z]
    rx_r, ry_r, rz_r = np.radians([rx, ry, rz])
    T[:3, :3] = rot_z(rz_r) @ rot_y(ry_r) @ rot_x(rx_r)
    return T


def matrix_to_pose(T):
    """4x4 homogeneous transform -> 6D pose [mm, mm, mm, deg, deg, deg].

    Inverse of pose_to_matrix, using the exchange spec's own extraction
    formulas. Degenerate at ry = +/-90 deg (gimbal lock: rx and rz stop being
    separable and the atan2 pair collapses onto their sum) -- untreated,
    because the poses this reports on are real tool/waypoint orientations
    where that is a measure-zero case, and silently picking a branch there
    would hide the ambiguity rather than surface it.
    """
    R = np.asarray(T)[:3, :3]
    ry = np.arcsin(-R[2, 0])
    rx = np.arctan2(R[2, 1], R[2, 2])
    rz = np.arctan2(R[1, 0], R[0, 0])
    return np.concatenate([np.asarray(T)[:3, 3], np.degrees([rx, ry, rz])])


# ============================================================================
# External IK exchange spec -- Rejection Criteria (roadmap 7.2)
#
# The spec's seven-row table, implemented verbatim as the definition of a valid
# print job. Source: external_ik_exchange_spec_EN.md "Rejection Criteria".
#
# Scope: BOTH toolpath sources and any future one. Not one of the seven rows
# reads a surface, a normal's provenance, or anything study-specific -- they
# validate the robot and the data, so planar G-code and curved layers go through
# the same validator. (The collision *narrowing* of 7.2 is the asymmetric half:
# that one is curved-only. Different question, different answer.)
#
# These rows validate DATA, NOT GEOMETRY. A job can pass every row here and
# still drive the arm through the build plate or the workpiece -- see
# wiki/003_Guides/CurvedModel_PrintSetup.md.
# ============================================================================

# One continuous extrusion line plus its solved joints -- see
# VisContent.build_export_segments(). positions (N,3) mm base frame, joints
# (N,6) deg, normals (N,3) unit base frame.
ExportSegment = namedtuple("ExportSegment", "index positions joints normals")

# One row's outcome. action is "REJECT" or "WARN"; detail carries the measured
# value and where it was found, so a caller can say which row failed and why
# rather than refusing silently.
CheckResult = namedtuple("CheckResult", "row passed action detail")


def _pose_delta(pose_a, pose_b):
    """(position error mm, rotation error deg) between two 6D poses. Rotation
    error is the largest per-axis difference, wrapped into (-180, 180] so a
    179 deg / -179 deg pair reads as 2 deg rather than 358."""
    pos = float(np.linalg.norm(np.asarray(pose_a)[:3] - np.asarray(pose_b)[:3]))
    d = (np.asarray(pose_a)[3:] - np.asarray(pose_b)[3:] + 180.0) % 360.0 - 180.0
    return pos, float(np.abs(d).max())


def validate_job(vis, segments):
    """Run the exchange spec's seven Rejection Criteria over a job about to be
    exported. Returns (ok, results): ok is False if any REJECT row failed; WARN
    rows never affect it. results is a list of CheckResult, always 8 long -- an
    in-house "job is non-empty" row first, then the spec's seven in table order,
    so a caller can render the whole table.

    vis is a VisContent (for compute_fk and T_flange_to_tcp); segments is
    build_export_segments()'s output.
    """
    results = []

    # --- Row 0: job is non-empty (IN-HOUSE, not one of the spec's seven) ---
    # Rows 3-7 are all "no offender found" tests, so they pass vacuously over an
    # empty segment list and an empty job would report ACCEPTED. The spec never
    # states this because it never contemplates exporting nothing. Reachable
    # today: build_export_segments() returns [] after a precompute cache hit --
    # see wiki/001_Inbox/2026-08-15_export_segments_cache_gap.md.
    n_export_points = sum(len(s.joints) for s in segments)
    results.append(CheckResult(
        "job is non-empty (in-house)", n_export_points > 0, "REJECT",
        f"{len(segments)} segment(s), {n_export_points} point(s)" if n_export_points
        else f"nothing to export ({len(segments)} segment(s), 0 point(s)) -- "
             "the remaining rows pass vacuously on an empty job"))

    # --- Row 1: identity check -------------------------------------------
    # FK(0) + the real tool offset must reproduce the spec's published TCP pose.
    # If this fails, the DH table, the rotation convention or the tool offset is
    # wrong and nothing downstream is trustworthy.
    actual = matrix_to_pose(vis.compute_fk([0] * 6)[5] @ vis.T_flange_to_tcp)
    dp, dr = _pose_delta(actual, IDENTITY_REFERENCE_TCP_POSE_6D)
    results.append(CheckResult(
        "identity check", dp < IDENTITY_POS_TOL_MM and dr < IDENTITY_ROT_TOL_DEG, "REJECT",
        f"FK(joints=0)+TCP vs spec reference: {dp:.6f}mm / {dr:.4f}deg "
        f"(limits {IDENTITY_POS_TOL_MM}mm / {IDENTITY_ROT_TOL_DEG}deg)"))

    # --- Row 2: TCP offset vs our calibration -----------------------------
    # Circular for a single-source project -- we are the calibration. Kept
    # because TCP_CALIBRATION_REFERENCE_6D is transcribed separately from the
    # supervisor doc, so the pair does catch a mistyped digit in either.
    dp, dr = _pose_delta(TCP_OFFSET_6D_MM_DEG, TCP_CALIBRATION_REFERENCE_6D)
    results.append(CheckResult(
        "TCP offset vs calibration", dp < TCP_OFFSET_POS_TOL_MM and dr < TCP_OFFSET_ROT_TOL_DEG, "REJECT",
        f"tool=1 offset vs saved_coords 1.2: {dp:.6f}mm / {dr:.6f}deg "
        f"(limits {TCP_OFFSET_POS_TOL_MM}mm / {TCP_OFFSET_ROT_TOL_DEG}deg)"))

    # Rows 3/4/7 are per-point and 5/6 per-segment; walk once, keeping one
    # offender per row so the message names a concrete location rather than just
    # a count. Rows 3/4/6 keep the first offender found; row 5 keeps the worst
    # step inside the first violating segment, which is the more useful pointer.
    bad_limit = bad_fk = bad_step = bad_count = None
    worst_fk = 0.0
    worst_step = None  # None -> no segment had >1 point, so row 5 never ran
    singular = []
    n_points = 0

    for seg in segments:
        n_points += len(seg.joints)

        # Row 6 here: no ply exists until 7.4 writes one, so the check is the
        # structural invariant its line count will inherit -- the three arrays
        # must describe the same number of points.
        if bad_count is None and not (len(seg.positions) == len(seg.joints) == len(seg.normals)):
            bad_count = (f"segment {seg.index}: positions={len(seg.positions)}, "
                         f"joints={len(seg.joints)}, normals={len(seg.normals)}")

        for i, angles in enumerate(seg.joints):
            # Row 3 -- the real physical envelope, NOT gui_panel.JOINT_LIMITS.
            if bad_limit is None:
                for j, (a, (lo, hi)) in enumerate(zip(angles, PHYSICAL_JOINT_LIMITS)):
                    if not (lo <= a <= hi):
                        bad_limit = (f"segment {seg.index} point {i}: "
                                     f"J{j+1}={a:.3f}deg outside [{lo}, {hi}]")
                        break

            # Row 4 -- does FK actually put the TCP where we claim it does?
            err = float(np.linalg.norm(
                (vis.compute_fk(angles)[5] @ vis.T_flange_to_tcp)[:3, 3] - seg.positions[i]))
            worst_fk = max(worst_fk, err)
            if bad_fk is None and err >= PER_POINT_FK_TOL_MM:
                bad_fk = f"segment {seg.index} point {i}: {err:.6f}mm"

            # Row 7 -- WARN only. Wider than solve_ik's own is_singular
            # (|sin(theta5)| < 1e-6, near-exact degeneracy); this is a band
            # around it where the wrist is merely ill-conditioned.
            if abs(angles[4]) < SINGULARITY_WARN_J5_DEG:
                singular.append(f"segment {seg.index} point {i} (J5={angles[4]:.3f}deg)")

        # Row 5 -- adjacent points WITHIN a segment. Deliberately not across
        # segment boundaries: the receiving side re-inserts a travel MoveJ
        # there, so a large jump between segments is expected and legal.
        if len(seg.joints) > 1:
            steps = np.abs(np.diff(np.asarray(seg.joints), axis=0))
            worst_step = max(worst_step or 0.0, float(steps.max()))
            if bad_step is None and steps.max() > JOINT_STEP_MAX_DEG:
                k, j = np.unravel_index(steps.argmax(), steps.shape)
                bad_step = (f"segment {seg.index} points {k}->{k+1}: "
                            f"J{j+1} moves {steps[k, j]:.3f}deg")

    results.append(CheckResult(
        "joint limits", bad_limit is None, "REJECT",
        bad_limit or f"all {n_points} point(s) within physical limits"))
    results.append(CheckResult(
        "per-point FK", bad_fk is None, "REJECT",
        (bad_fk + f" (limit {PER_POINT_FK_TOL_MM}mm)") if bad_fk
        else f"worst error {worst_fk:.6f}mm over {n_points} point(s) (limit {PER_POINT_FK_TOL_MM}mm)"))
    results.append(CheckResult(
        "joint step within segment", bad_step is None, "REJECT",
        (bad_step + f" (limit {JOINT_STEP_MAX_DEG}deg)") if bad_step
        else (f"worst step {worst_step:.3f}deg (limit {JOINT_STEP_MAX_DEG}deg)"
              if worst_step is not None
              else "not evaluated -- no segment has two adjacent points")))
    results.append(CheckResult(
        "num_points consistency", bad_count is None, "REJECT",
        bad_count or f"{len(segments)} segment(s), {n_points} point(s), arrays agree"))
    if singular:
        detail = (f"{len(singular)} near-singular point(s): {', '.join(singular[:3])}"
                  + (" ..." if len(singular) > 3 else ""))
    else:
        detail = f"no point within {SINGULARITY_WARN_J5_DEG}deg of wrist singularity"
    results.append(CheckResult("|J5| < 2deg singularity", not singular, "WARN", detail))

    ok = all(r.passed for r in results if r.action == "REJECT")
    return ok, results


def format_validation(ok, results):
    """One line per row, table order, for a status pane or a console run."""
    lines = [f"{'PASS' if r.passed else ('WARN' if r.action == 'WARN' else 'FAIL')}  "
             f"{r.row}: {r.detail}" for r in results]
    lines.append(f"==> job {'ACCEPTED' if ok else 'REJECTED'}"
                 + (" (with warnings)" if ok and any(
                     not r.passed for r in results if r.action == "WARN") else ""))
    return "\n".join(lines)


def write_job_export(vis, segments, job_dir):
    """Write one job.json + segment_N_solution.json + toolpath_TN.ply per
    ExportSegment, plus surface.obj where one exists, to job_dir --
    external_ik_exchange_spec_EN.md's folder structure. Roadmap 7.5.

    Assumes the caller already ran validate_job() and got ok=True; this
    function only writes, it does not re-check. Called from
    VisContent.export_active_job().

    surface.obj is curved-only: the planar path's "plate" is S1.40's
    infinite plane, not a mesh asset, so there is nothing to copy for it --
    a known gap in spec coverage for that source, not an oversight.
    """
    os.makedirs(job_dir, exist_ok=True)

    # Prune stale files from a previous, larger export first -- otherwise a
    # re-export with fewer segments than last time leaves orphaned
    # higher-numbered files a receiving parser would try to load.
    keep = len(segments)
    for fname in os.listdir(job_dir):
        m = re.match(r"^(?:toolpath_T|segment_)(\d+)(?:_solution)?\.(?:ply|json)$", fname)
        if m and int(m.group(1)) >= keep:
            os.remove(os.path.join(job_dir, fname))

    for seg in segments:
        ply_name = f"toolpath_T{seg.index}.ply"
        with open(os.path.join(job_dir, ply_name), "w", encoding="utf-8") as f:
            for pos, normal in zip(seg.positions, seg.normals):
                f.write(f"{pos[0]:.6f} {pos[1]:.6f} {pos[2]:.6f} "
                        f"{normal[0]:.6f} {normal[1]:.6f} {normal[2]:.6f}\n")

        points = []
        for angles, normal in zip(seg.joints, seg.normals):
            tcp_xyz = (vis.compute_fk(angles)[5] @ vis.T_flange_to_tcp)[:3, 3]
            points.append({
                "joints_deg": [float(a) for a in angles],
                "tcp_xyz_base_mm": [float(v) for v in tcp_xyz],
                "normal_base": [float(v) for v in normal],
            })
        solution = {
            "segment_id": seg.index,
            "toolpath_file": ply_name,
            "num_points": len(points),
            "points": points,
        }
        with open(os.path.join(job_dir, f"segment_{seg.index}_solution.json"), "w", encoding="utf-8") as f:
            json.dump(solution, f, indent=2)

    if vis.toolpath_source != -1:
        surface_src = os.path.join(CURVED_MODEL_DIR, CURVED_LAYERS[vis.toolpath_source]["surface_file"])
        shutil.copyfile(surface_src, os.path.join(job_dir, "surface.obj"))

    identity_pose = matrix_to_pose(vis.compute_fk([0] * 6)[5] @ vis.T_flange_to_tcp)
    job = {
        "format": "fr5_external_ik_job",
        "format_version": "2.0",
        "generator": EXPORT_GENERATOR,
        "generated_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "tool_index": 1,
        "tcp_offset_6d": [float(v) for v in TCP_OFFSET_6D_MM_DEG],
        "identity_check": {
            "joints_zero_tcp_pose_base": [float(v) for v in identity_pose],
        },
        "segments": [
            {"segment_id": seg.index, "toolpath": f"toolpath_T{seg.index}.ply",
             "solution": f"segment_{seg.index}_solution.json"}
            for seg in segments
        ],
    }
    with open(os.path.join(job_dir, "job.json"), "w", encoding="utf-8") as f:
        json.dump(job, f, indent=2)

    return job_dir


# Validation
if __name__ == "__main__":
    ps.init()
    vis = VisContent()
    vis.end_effector_position([0, 0, 0, 0, 0, 0])
    print(f"[Backend] Loaded {len(vis.mesh_data)} link meshes")

    gcode_waypoints = vis.parse_gcode(os.path.join(GCODE_DIR, GCODE_FILE))
    print(f"[Backend] Parsed {len(gcode_waypoints)} G-code waypoints")
    vis.load_gcode()

