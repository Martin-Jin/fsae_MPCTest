"""
simulation.py — Interactive Closed-Loop MPC Simulator (v6)

PURPOSE
-------
Provides an interactive matplotlib-based GUI for testing, visualising, and
scoring the MPC path-tracking controller against the nonlinear vehicle plant.
The user draws a path (or loads a synthetic one), configures initial conditions
via sliders, runs a closed-loop simulation, and scrubs through the full history.

This is the main integration point for the whole codebase: it wires together
every other module into a single runnable loop.

ARCHITECTURE WITHIN THIS FILE
------------------------------
The file is structured in six sections:

  1. Configuration        — dt, horizon, weight matrices, global state variables
  2. GUI Layout           — matplotlib figure, axes, lines, sliders, buttons
  3. Helper Mathematics   — triangle renderer, angle wrapping, closest-point search
  4. Event Handlers       — mouse draw, button callbacks, path loading
  5. Simulation Engine    — simulate_closed_loop(): the core closed-loop loop
  6. Playback             — timeline scrubber, telemetry panel update

SIMULATION LOOP SUMMARY (simulate_closed_loop)
----------------------------------------------
At each 20 Hz step:
  1.  Record current plant state to history
  2.  Filter visible cones via SimPerception
  3.  Update SimPlanner (centreline + speed profile rebuild)
  4.  Compute tracking errors (e_y, e_psi) from planner centreline
      (falls back to drawn path if planner hasn't converged yet)
  5.  Assemble 8-state MPC error vector
  6.  Check early-exit conditions (off-track, solver failure, path end)
  7.  Solve MPC with adaptive gain scaling
  8.  Roll out N-step horizon prediction for visualisation
  9.  Advance the nonlinear plant one timestep

PLANT / CONTROLLER MISMATCH (intentional)
------------------------------------------
The plant (vehicle_physics.step_nonlinear_plant) is a 24-state nonlinear model
with Pacejka tyres, suspension, and aerodynamics. The controller's internal
model (bicycle_model.get_8state_discrete_model) is a linearised 8-state bicycle
model. The controller never observes the plant's internal states directly — it
only receives the tracking errors computed from the plant's global position each
step. This closed-loop feedback is what makes the MPC robust to model mismatch.

USED BY
-------
  Standalone: run with `python simulation.py`
  Imports from: bicycle_model, optimiser, vehicle_physics, performance_stats,
                speed_profile, offline_tuner, sim_track, model_utils

DOES NOT USE
------------
  offline_tuner.py (at runtime — only imports pre-computed SYNTHETIC_PATHS
  and PATH_NAMES constants at import time; no tuner logic runs during simulation)
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, Slider
from scipy.interpolate import CubicSpline
from bicycle_model import get_8state_discrete_model
from optimiser import solve_mpc
from vehicle_physics import VehicleParams, step_nonlinear_plant, init_plant_state
import performance_stats
import speed_profile
from offline_tuner import SYNTHETIC_PATHS, PATH_NAMES
from sim_track import place_cones, SimPerception, SimPlanner, calculate_dynamic_max_steps
from model_utils import curvature_estimate, adaptive_R_rate, adaptive_R_scaling
import math


# ==========================================
# SETUP AND CONFIGURATION
# ==========================================

dt        = 0.05    # Simulation timestep (s) — 20 Hz, matches vehicle_physics sub-stepping
N_horizon = 25      # MPC prediction horizon (steps = 1.25 s of look-ahead at 20 Hz)
v_ref     = 7.0     # Fallback constant speed (m/s); only used if path_v_profile is empty

# ── MPC Cost Weight Matrices ───────────────────────────────────────────────────
# States penalised by Q: [e_y, ė_y, e_ψ, ψ̇, e_v, e_a, δ_act, a_act]
# The last three entries are zero: actuator states are not penalised here;
# R_rate handles smoothness indirectly through Δu costs.
# These values are the output of the most recent offline_tuner.py run.
# To update: paste Q_diag, R_diag, R_rate_diag printed by offline_tuner.py.
Q_diag      = [1.0183759459339357, 0.976697255260965, 28.738153874481, 0.5306028079715992, 4.38214845893275, 0.0, 0.0, 0.0]
R_diag      = [44.803475493970396, 49.84407709101785]
R_rate_diag = [41.29277347324602, 2.14748612384217]

Q      = np.diag(Q_diag)       # State cost matrix (8×8 diagonal)
R      = np.diag(R_diag)       # Input cost matrix (2×2 diagonal)
R_rate = np.diag(R_rate_diag)  # Input rate-of-change cost matrix (2×2 diagonal)

# Speed profile limits passed to SimPlanner and speed_profile.compute_speed_profile()
V_MAX = 20.0   # Absolute speed cap (m/s); planner and profiler respect this
V_MIN = 1.5    # Minimum speed floor (m/s); prevents near-zero speed targets

# ── Global GUI State ────────────────────────────────────────────────────────────
is_drawing          = False          # True while user is dragging a path
is_simulated        = False          # True after a simulation has been run
flip_heading_180    = False          # True if "Flip Heading" was pressed
drawn_points        = []             # Raw mouse points before spline fitting
path_X, path_Y, path_Psi = [], [], []  # Resampled path arrays (after spline fit)
path_v_profile      = np.array([])   # Per-point target speed from speed_profile.py
sim_history         = {}             # Result dict from the most recent simulation

# Full static cone arrays (populated by place_cones after path creation/load)
_blue_cones_all   = np.empty((0, 2))
_yellow_cones_all = np.empty((0, 2))

# Vehicle parameters (loaded once; shared across all simulation calls)
vehicle_params = VehicleParams()
u_bounds_min   = [-vehicle_params.max_steer, vehicle_params.max_accel_brake]
u_bounds_max   = [ vehicle_params.max_steer, vehicle_params.max_accel]

# Tracks which synthetic path is currently loaded (-1 = none)
current_test_path_idx = -1


# ==========================================
# INTERACTIVE GUI LAYOUT
# ==========================================
# 6-row, 2-column gridspec:
#   Row 0:       main map (col 0) | telemetry panel (col 1)
#   Rows 1-2:    sliders (col 0) | buttons (col 1)
#   Rows 3-5:    time-scrub slider (col 0, spans 3 rows) | buttons (col 1)
fig = plt.figure(figsize=(15, 9.2))
gs  = fig.add_gridspec(
    6, 2,
    width_ratios=[3.8, 1.2],
    height_ratios=[12, 1, 1, 1, 1, 1],
    left=0.06, right=0.94, top=0.94, bottom=0.06,
    wspace=0.15, hspace=0.45,
)

ax_map  = fig.add_subplot(gs[0, 0])   # Main map view (path drawing + simulation display)
ax_info = fig.add_subplot(gs[0, 1])   # Telemetry text panel
ax_info.axis("off")

# Plot lines — updated each scrub frame and during simulation
(path_line,)        = ax_map.plot([], [], "r--",  label="Target Path",           linewidth=2)
(trail_line,)       = ax_map.plot([], [], "b-",   label="Actual Vehicle Trail",  alpha=0.6)
(pred_line,)        = ax_map.plot([], [], "c-o",  label="MPC Horizon Prediction", markersize=3, alpha=0.8)
(vehicle_marker,)   = ax_map.plot([], [], "g-",   linewidth=2.0,                  label="Vehicle")
(blue_cones_line,)  = ax_map.plot([], [], color="blue", marker="o", linestyle="None", markersize=4, label="Blue Cones")
(yellow_cones_line,)= ax_map.plot([], [], color="gold", marker="o", linestyle="None", markersize=4, label="Yellow Cones")

ax_map.set_xlim(0, 100)
ax_map.set_ylim(0, 60)
ax_map.set_aspect("equal")
ax_map.grid(True)
ax_map.set_title("Robust High-Speed Path MPC Sandbox", fontweight="bold")
ax_map.legend(loc="upper right")

# Telemetry panel: monospaced text box updated each scrub frame
telemetry_text = ax_info.text(
    0.0, 0.95, "",
    family="monospace", fontsize=10.5, verticalalignment="top",
    bbox=dict(facecolor="#f8f9fa", edgecolor="#ccced1", boxstyle="round,pad=0.7"),
)

# Slider axes
ax_ey0   = fig.add_subplot(gs[1, 0])   # Initial lateral error slider
ax_epsi0 = fig.add_subplot(gs[2, 0])   # Initial yaw error slider
ax_scrub = fig.add_subplot(gs[3:6, 0]) # Timeline scrub slider (spans 3 rows)

slider_ey0   = Slider(ax_ey0,   "Initial Lat Error", -4.0, 4.0,    valinit=0.0, valfmt="%0.1f m",  color="orange")
slider_epsi0 = Slider(ax_epsi0, "Initial Yaw Error", -30.0, 30.0,  valinit=0.0, valfmt="%0.1f°",  color="orange")
slider_scrub = Slider(ax_scrub, "Time",               0,    1,      valinit=0,   valfmt="%d",       color="teal")
ax_scrub.set_visible(False)   # Hidden until a simulation completes

# Button column
ax_btn_load     = fig.add_subplot(gs[1, 1])
ax_btn_start    = fig.add_subplot(gs[2, 1])
ax_btn_flip     = fig.add_subplot(gs[3, 1])
ax_btn_reset    = fig.add_subplot(gs[4, 1])
ax_btn_optimize = fig.add_subplot(gs[5, 1])

btn_load     = Button(ax_btn_load,     "Load Test Path",      color="thistle",   hovercolor="plum")
btn_start    = Button(ax_btn_start,    "Start Sim",           color="lightgreen",hovercolor="limegreen")
btn_flip     = Button(ax_btn_flip,     "Flip Heading (180°)", color="lightgreen",hovercolor="khaki")
btn_reset    = Button(ax_btn_reset,    "Reset Environment",   color="tomato",    hovercolor="crimson")
btn_optimize = Button(ax_btn_optimize, "Show Metrics",        color="lightblue", hovercolor="deepskyblue")


# ==========================================
# HELPER MATHEMATICS AND GEOMETRY
# ==========================================

def get_car_triangle(x, y, heading, size=2.2):
    """
    Compute the (X, Y) vertices of a triangle representing the vehicle at a
    given position and heading, for rendering on the map axes.

    The triangle has its apex at the front and two rear corners, scaled by
    `size`. Vertices are rotated by the heading angle before translating to
    (x, y), giving a direction-indicating marker at any orientation.

    Parameters
    ----------
    x, y : float   Vehicle position in global frame (m).
    heading : float Vehicle yaw angle (rad).
    size : float    Triangle scale factor (display units = metres on map).

    Returns
    -------
    (tx, ty) : tuple of np.ndarray, shape (4,)
        Closed polygon vertices (4 points, last = first) for ax.plot().

    Called by: on_release(), toggle_heading_flip(), update_scrub_frame(),
               load_test_path()
    """
    corners = np.array([
        [ size,          0         ],   # Front apex
        [-size / 1.5,    size / 1.5],   # Rear left
        [-size / 1.5,   -size / 1.5],   # Rear right
        [ size,          0         ],   # Close polygon
    ])
    rot = np.array([
        [np.cos(heading), -np.sin(heading)],
        [np.sin(heading),  np.cos(heading)],
    ])
    rotated = (rot @ corners.T).T
    return rotated[:, 0] + x, rotated[:, 1] + y


def normalize_angle(angle):
    """
    Wrap an angle to (−π, π] using atan2.

    Parameters
    ----------
    angle : float   Angle in radians (any range).

    Returns
    -------
    float : Equivalent angle in (−π, π].

    Called by: simulate_closed_loop(), toggle_heading_flip(), load_test_path()
    """
    return np.arctan2(np.sin(angle), np.cos(angle))


def find_closest_reference_bounded(x_g, y_g, last_idx, window=40):
    """
    Find the closest point on the reference path (path_X, path_Y) to the
    given global position, searching within a bounded window around last_idx.

    The windowed search prevents the tracker from jumping backward on paths
    that double back on themselves (e.g. after a hairpin). At the start of a
    simulation (last_idx ≤ 5), a wider initial window prevents the tracker
    from locking onto index 0 if the vehicle has already moved forward.

    This function reads the module-level path_X, path_Y, path_Psi arrays.

    Parameters
    ----------
    x_g, y_g : float   Vehicle global position (m).
    last_idx : int      Previously found closest index (search anchor).
    window : int        Forward search range in path indices. Default 40.

    Returns
    -------
    (global_idx, ref_x, ref_y, ref_psi) : (int, float, float, float)
        Index and coordinates of the nearest path point, plus path heading there.

    Called by: simulate_closed_loop() — fallback path when SimPlanner has no centreline,
               and for path-end detection (idx ≥ len(path_X) - 2)
    """
    if last_idx <= 5:
        start_search = 0
        end_search   = min(len(path_X), 100)   # Wide initial window
    else:
        start_search = max(0, last_idx - 5)
        end_search   = min(len(path_X), last_idx + window)

    distances  = np.hypot(
        path_X[start_search:end_search] - x_g,
        path_Y[start_search:end_search] - y_g,
    )
    local_idx  = np.argmin(distances)
    global_idx = start_search + local_idx

    return global_idx, path_X[global_idx], path_Y[global_idx], path_Psi[global_idx]


# ==========================================
# INTERACTIVE EVENT HANDLERS
# ==========================================

def reset_environment(event):
    """
    Reset all global state, clear all plot lines, and return the GUI to its
    initial pre-draw state.

    Called by: btn_reset ("Reset Environment" button)
    """
    global is_simulated, flip_heading_180, drawn_points, path_X, path_Y, \
           path_Psi, path_v_profile, sim_history, current_test_path_idx

    is_simulated          = False
    flip_heading_180      = False
    drawn_points          = []
    path_X, path_Y, path_Psi = [], [], []
    path_v_profile        = np.array([])
    sim_history           = {}
    current_test_path_idx = -1

    path_line.set_data([], [])
    trail_line.set_data([], [])
    pred_line.set_data([], [])
    vehicle_marker.set_data([], [])
    telemetry_text.set_text("")
    blue_cones_line.set_data([], [])
    yellow_cones_line.set_data([], [])

    ax_map.set_xlim(0, 100)
    ax_map.set_ylim(0, 60)

    ax_ey0.set_visible(True)
    ax_epsi0.set_visible(True)
    ax_scrub.set_visible(False)

    btn_load.set_active(True)
    btn_start.set_active(True)
    btn_flip.set_active(True)
    btn_optimize.set_active(True)
    slider_ey0.set_val(0.0)
    slider_epsi0.set_val(0.0)
    ax_map.set_title(
        "Environment Reset. Draw a new path or Load Test Path.",
        fontweight="bold", color="black",
    )
    fig.canvas.draw_idle()


def load_test_path(event):
    """
    Cycle through the synthetic path library and load the next path.

    Each click advances current_test_path_idx by 1, wrapping around after
    the last path. The path is read from SYNTHETIC_PATHS (pre-computed at
    import time in offline_tuner.py), cones are placed, and the camera is
    framed around the new path with a 15 m margin.

    Does nothing if a simulation is already running (is_simulated = True).

    Called by: btn_load ("Load Test Path" button)
    """
    global path_X, path_Y, path_Psi, path_v_profile, flip_heading_180, \
           current_test_path_idx, _blue_cones_all, _yellow_cones_all

    if is_simulated:
        return

    # Advance cycle index
    current_test_path_idx = (current_test_path_idx + 1) % len(PATH_NAMES)
    path_name = PATH_NAMES[current_test_path_idx]

    # Unpack the pre-computed geometry and speed profile
    path_X, path_Y, path_Psi, path_v_profile, _, _ = SYNTHETIC_PATHS[path_name]
    flip_heading_180 = False

    # Generate and render cones
    _blue_cones_all, _yellow_cones_all = place_cones(path_X, path_Y)
    if len(_blue_cones_all) > 0:
        blue_cones_line.set_data(_blue_cones_all[:, 0], _blue_cones_all[:, 1])
        yellow_cones_line.set_data(_yellow_cones_all[:, 0], _yellow_cones_all[:, 1])

    path_line.set_data(path_X, path_Y)

    # Place vehicle marker at path start
    car_x, car_y = get_car_triangle(path_X[0], path_Y[0], path_Psi[0])
    vehicle_marker.set_data(car_x, car_y)

    ax_map.set_title(f"Loaded: {path_name} | Click 'Start Sim'", fontweight="bold", color="blue")

    # Frame camera around path with margin
    margin = 15.0
    ax_map.set_xlim(np.min(path_X) - margin, np.max(path_X) + margin)
    ax_map.set_ylim(np.min(path_Y) - margin, np.max(path_Y) + margin)
    fig.canvas.draw_idle()


def on_press(event):
    """
    Start recording mouse points when the user presses the mouse button on
    the map axes. Ignored if a simulation has already been run (path locked).

    Called by: fig.canvas.mpl_connect("button_press_event", on_press)
    """
    global is_drawing, drawn_points, is_simulated
    if event.inaxes != ax_map or is_simulated:
        return
    is_drawing    = True
    drawn_points  = [[event.xdata, event.ydata]]


def on_motion(event):
    """
    Append mouse position to drawn_points and update the path line in real time
    while the user drags. Provides immediate visual feedback of the drawn path.

    Called by: fig.canvas.mpl_connect("motion_notify_event", on_motion)
    """
    global drawn_points
    if not is_drawing or event.inaxes != ax_map:
        return
    drawn_points.append([event.xdata, event.ydata])
    pts = np.array(drawn_points)
    path_line.set_data(pts[:, 0], pts[:, 1])
    fig.canvas.draw_idle()


def on_release(event):
    """
    Finalise a drawn path on mouse release. Filters jitter, fits a clamped
    CubicSpline, resamples to 600+ dense points, computes heading and speed
    profile, and places cones.

    SPLINE FITTING
    --------------
    Raw drawn points are first deduplicated (min 0.5 m gap between consecutive
    points) to remove mouse sampling noise. A clamped CubicSpline is then fit:
    the derivative at each endpoint is pinned to the direction of the first/last
    chord. This prevents the "not-a-knot" default from creating an overshoot
    curvature spike at the path ends which would generate unrealistically low
    corner speeds and mislead the speed profiler.

    Path heading path_Psi is derived from arctan2(dy/dt, dx/dt) of the spline
    derivative, giving a smooth, continuous heading array without finite-difference
    noise from the raw drawn points.

    Requires at least 6 raw points (4 filtered); returns early otherwise.

    Called by: fig.canvas.mpl_connect("button_release_event", on_release)
    """
    global is_drawing, path_X, path_Y, path_Psi, path_v_profile, flip_heading_180
    global _blue_cones_all, _yellow_cones_all

    if not is_drawing:
        return
    is_drawing = False
    if len(drawn_points) < 6:
        return

    # Deduplicate: remove points within 0.5 m of the previous to reduce jitter
    raw_pts      = np.array(drawn_points)
    filtered_pts = [raw_pts[0]]
    for p in raw_pts[1:]:
        if np.linalg.norm(p - filtered_pts[-1]) > 0.5:
            filtered_pts.append(p)
    if len(filtered_pts) < 4:
        filtered_pts = list(raw_pts)   # Fallback for very slowly drawn paths

    pts = np.array(filtered_pts)
    t   = np.linspace(0, 1, len(pts))

    # Clamped boundary conditions: endpoint derivatives pinned to chord direction
    d0   = (pts[1]  - pts[0])  / (t[1]  - t[0])
    dN   = (pts[-1] - pts[-2]) / (t[-1] - t[-2])
    cs_x = CubicSpline(t, pts[:, 0], bc_type=((1, d0[0]), (1, dN[0])))
    cs_y = CubicSpline(t, pts[:, 1], bc_type=((1, d0[1]), (1, dN[1])))

    # Resample at min 600 points (finer for long drawn paths)
    t_fine  = np.linspace(0, 1, max(600, len(pts) * 6))
    path_X  = cs_x(t_fine)
    path_Y  = cs_y(t_fine)
    # Heading from spline derivative: smooth, no finite-difference noise
    dx       = cs_x.derivative()(t_fine)
    dy       = cs_y.derivative()(t_fine)
    path_Psi = np.arctan2(dy, dx)

    # Compute and smooth curvature-based speed profile
    raw_profile    = speed_profile.compute_speed_profile(path_X, path_Y)
    path_v_profile = speed_profile.smooth_profile(raw_profile, window=9)

    # Place cones for perception
    flip_heading_180      = False
    _blue_cones_all, _yellow_cones_all = place_cones(path_X, path_Y)
    if len(_blue_cones_all) > 0:
        blue_cones_line.set_data(_blue_cones_all[:, 0], _blue_cones_all[:, 1])
        yellow_cones_line.set_data(_yellow_cones_all[:, 0], _yellow_cones_all[:, 1])

    path_line.set_data(path_X, path_Y)
    car_x, car_y = get_car_triangle(path_X[0], path_Y[0], path_Psi[0])
    vehicle_marker.set_data(car_x, car_y)
    fig.canvas.draw_idle()


def toggle_heading_flip(event):
    """
    Toggle whether the vehicle starts pointing 180° away from the path direction.

    When flipped, the initial heading becomes path_Psi[0] + π (pointing backward).
    This tests the MPC's recovery capability from a worst-case heading mismatch.
    The vehicle marker on the map is updated immediately to show the new heading.

    Called by: btn_flip ("Flip Heading (180°)" button)
    """
    global flip_heading_180
    if len(path_X) == 0:
        return
    flip_heading_180 = not flip_heading_180

    base_heading = path_Psi[0] + (np.pi if flip_heading_180 else 0.0)
    current_psi  = normalize_angle(base_heading + np.radians(slider_epsi0.val))

    X_g = path_X[0] - slider_ey0.val * np.sin(base_heading)
    Y_g = path_Y[0] + slider_ey0.val * np.cos(base_heading)

    car_x, car_y = get_car_triangle(X_g, Y_g, current_psi)
    vehicle_marker.set_data(car_x, car_y)
    fig.canvas.draw_idle()


# Register event handlers
fig.canvas.mpl_connect("button_press_event",   on_press)
fig.canvas.mpl_connect("motion_notify_event",  on_motion)
fig.canvas.mpl_connect("button_release_event", on_release)
btn_load.on_clicked(load_test_path)
btn_flip.on_clicked(toggle_heading_flip)
btn_reset.on_clicked(reset_environment)


# ==========================================
# SIMULATION ENGINE
# ==========================================

def simulate_closed_loop(Q_w, R_w, ey0, epsi0, flip, rng_seed=None, max_steps=400, R_rate_w=None):
    """
    Run one closed-loop simulation rollout on the currently loaded path.

    This is the core simulation function. It integrates the nonlinear vehicle
    plant (vehicle_physics.step_nonlinear_plant) driven by the MPC solver
    (optimiser.solve_mpc), which internally predicts using the linear bicycle
    model (bicycle_model.get_8state_discrete_model).

    The plant and controller use deliberately different models (model-plant
    mismatch): the MPC never observes the plant's 24 internal states directly.
    It only receives the tracking errors derived from the plant's global
    position each step, exactly as a real controller does from state estimates.

    PERCEPTION AND PLANNING
    -----------------------
    SimPerception filters the full static cone map to the car's forward FOV
    each step. SimPlanner accumulates observations and rebuilds the centreline
    and speed profile on every step. Tracking errors are computed against the
    planner's centreline (matching the real ROS2 pipeline). While the planner
    warms up (insufficient cones to build a path yet), errors fall back to
    the drawn reference path.

    ADAPTIVE GAIN SCHEDULING
    ------------------------
    Before each MPC solve, two gain-scheduling functions from model_utils.py
    modify the weight matrices:
      - adaptive_R_scaling(vx, R): increases steering cost at high speed
        (Hill function, saturates at ~2.5× base) to prevent destabilising
        large steering commands where the linear model is less accurate.
      - adaptive_R_rate(kappa, R_rate): softens steering jerk penalty in
        tight corners (floor at 35% of base) so the controller can steer
        aggressively enough to track the corner without understeering.

    INITIAL CONDITION JITTER
    ------------------------
    If rng_seed is not None, small zero-mean Gaussian perturbations are added
    to ey0 (σ=0.05 m) and epsi0 (σ=1°). This is used when running multiple
    rollouts for averaging; individual simulations (rng_seed=None) use the
    exact slider values.

    TERMINATION CONDITIONS
    ----------------------
    The loop ends on the first of:
      - Reaching path end: idx ≥ len(path_X) - 2  OR  dist_to_end ≤ 3.0 m
      - Off-track:  |e_y| > OFFTRACK_LIMIT (5.0 m) → history["failed"] = True
      - Solver failure: consecutive_solver_failures ≥ MAX_CONSECUTIVE_FAILURES (5)
      - Step budget: step ≥ max_steps

    Parameters
    ----------
    Q_w : np.ndarray, shape (8, 8)
        State cost matrix. Elements penalise each component of the MPC error
        state [e_y, ė_y, e_ψ, ψ̇, e_v, e_a, δ_act, a_act].
    R_w : np.ndarray, shape (2, 2)
        Input cost matrix. Penalises [δ_cmd, a_cmd] magnitude each step.
    ey0 : float
        Initial lateral offset of the vehicle from the path start (m).
        Applied perpendicular to the path heading.
    epsi0 : float
        Initial heading offset from path direction (degrees).
    flip : bool
        If True, vehicle starts facing path_Psi[0] + π (away from path).
    rng_seed : int or None, optional
        Seed for initial condition jitter RNG. None = deterministic.
    max_steps : int, optional
        Maximum simulation steps. Overridden by calculate_dynamic_max_steps()
        in run_simulation(). Default 400.
    R_rate_w : np.ndarray, shape (2, 2), optional
        Rate-of-change cost matrix. Defaults to module-level R_rate if None.

    Returns
    -------
    history : dict
        Simulation history with keys:
          "X", "Y", "psi"     : list of float — global pose at each step
          "v"                 : list of float — longitudinal speed (m/s)
          "v_target"          : list of float — MPC's target speed (m/s)
          "u_steer"           : list of float — applied steering command (rad)
          "u_accel"           : list of float — applied acceleration command (m/s²)
          "e_y"               : list of float — lateral tracking error (m)
          "e_psi"             : list of float — heading tracking error (rad)
          "pred_X", "pred_Y"  : list of list  — N-step horizon prediction per step
          "failed"            : bool          — True if off-track or solver failed
          "fail_reason"       : str or None   — description of failure cause
          "reached_end"       : bool          — True if path completed cleanly
          "completion_frac"   : float         — fraction of path completed [0, 1]
          "time_bonus"        : float         — speed-completion bonus [0, 1]
          "peak_lateral_error": float         — max |e_y| over the run (m)

    Called by: run_simulation() (triggered by btn_start "Start Sim")
    """
    if R_rate_w is None:
        R_rate_w = R_rate

    # Optional initial condition jitter (for multi-rollout averaging)
    rng         = np.random.default_rng(rng_seed)
    jitter_ey   = rng.normal(0, 0.05) if rng_seed is not None else 0.0
    jitter_epsi = rng.normal(0, 1.0)  if rng_seed is not None else 0.0   # degrees

    # Compute initial vehicle position in global frame
    base_path_heading = path_Psi[0] + (np.pi if flip else 0.0)
    ey0_eff           = ey0   + jitter_ey
    epsi0_eff         = epsi0 + jitter_epsi

    # Lateral offset applied perpendicular to path heading
    X_g   = path_X[0] - ey0_eff * np.sin(base_path_heading)
    Y_g   = path_Y[0] + ey0_eff * np.cos(base_path_heading)
    psi_g = normalize_angle(base_path_heading + np.radians(epsi0_eff))

    v_start = 0.0   # Always start from standstill

    # Initialise nonlinear plant (24-state) at true static equilibrium
    plant_state = init_plant_state(X_g, Y_g, psi_g, vx0=v_start)

    # Initialise perception and planning pipeline
    perception = SimPerception(_blue_cones_all, _yellow_cones_all)
    planner    = SimPlanner(
        v_max=V_MAX if "V_MAX" in dir() else 20.0,
        v_min=V_MIN if "V_MIN" in dir() else 1.5,
    )
    # Warm-start the planner with the initial cone observations
    _b0, _y0 = perception.visible_cones(X_g, Y_g, psi_g)
    planner.update(_b0, _y0, np.array([X_g, Y_g]), psi_g)

    u_prev                     = np.zeros(2)
    consecutive_solver_failures = 0
    MAX_CONSECUTIVE_FAILURES    = 5
    OFFTRACK_LIMIT              = 5.0   # metres; well beyond the MPC's ±3.5 m soft corridor

    history = {
        "X": [], "Y": [], "psi": [], "v": [], "v_target": [],
        "u_steer": [], "u_accel": [],
        "e_y": [], "e_psi": [],
        "pred_X": [], "pred_Y": [],
        "failed": False,
        "fail_reason": None,
    }

    idx = 0   # Current closest reference path index

    for step in range(max_steps):

        # ── 1. Record current plant state ─────────────────────────────────────
        history["X"].append(plant_state[0])
        history["Y"].append(plant_state[1])
        history["psi"].append(plant_state[2])
        history["v"].append(plant_state[3])

        # ── 2. Perception + planning update ───────────────────────────────────
        X_g, Y_g, psi_g = plant_state[0], plant_state[1], plant_state[2]
        car_pos_np       = np.array([X_g, Y_g])

        b_vis, y_vis = perception.visible_cones(X_g, Y_g, psi_g)
        planner.update(b_vis, y_vis, car_pos_np, psi_g)

        # Track progress on the original reference path for path-end detection
        idx, _, _, _ = find_closest_reference_bounded(X_g, Y_g, idx, window=40)

        # ── 3. Tracking error from planner centreline (primary) ───────────────
        # Uses SimPlanner's accumulated centreline if available; falls back to
        # the drawn reference path while the planner warms up (first ~0.5 s).
        cl = planner.centreline
        if cl is not None and len(cl) >= 2:
            dists  = np.linalg.norm(cl - car_pos_np, axis=1)
            cl_idx = int(np.argmin(dists))
            seg    = (cl[cl_idx + 1] - cl[cl_idx]) if cl_idx < len(cl) - 1 else (cl[cl_idx] - cl[cl_idx - 1])
            seg_len = float(np.linalg.norm(seg))
            if seg_len > 1e-6:
                t_hat   = seg / seg_len                     # Unit tangent along centreline
                right_n = np.array([t_hat[1], -t_hat[0]])  # Right-pointing normal
                rpsi    = math.atan2(t_hat[1], t_hat[0])   # Path heading at cl_idx
                # Lateral error: positive = vehicle is to the LEFT of the centreline
                e_y     = -float(np.dot(car_pos_np - cl[cl_idx], right_n))
            else:
                rpsi = psi_g; e_y = 0.0
        else:
            # Fallback: use drawn reference path directly
            idx, rx, ry, rpsi = find_closest_reference_bounded(X_g, Y_g, idx, window=40)
            if flip:
                rpsi = normalize_angle(rpsi + np.pi)
            dx_err = X_g - rx; dy_err = Y_g - ry
            e_y    = dy_err * np.cos(rpsi) - dx_err * np.sin(rpsi)

        e_psi = normalize_angle(psi_g - rpsi)    # Heading error (wrapped to ±π)

        _, v_target = planner.get_target(car_pos_np, psi_g)   # Desired speed from planner
        history["v_target"].append(v_target)
        history["e_y"].append(e_y)
        history["e_psi"].append(e_psi)

        # ── 4. Assemble 8-state MPC error vector ──────────────────────────────
        # [e_y, ė_y, e_ψ, ψ̇, e_v, 0, δ_act, a_act]
        # e_y_dot: lateral velocity projected onto the path-normal direction
        # e_v:     speed error relative to the planner's desired speed
        x_current = np.array([
            e_y,
            plant_state[3] * np.sin(e_psi) + plant_state[4] * np.cos(e_psi),  # ė_y
            e_psi,
            plant_state[5],              # yaw rate r (= ψ̇ for the MPC)
            plant_state[3] - v_target,   # speed error (vx - v_target)
            0.0,                         # e_a unused; R_rate handles smoothness
            plant_state[6],              # delta_act (steering lag state)
            plant_state[7],              # a_act (acceleration lag state)
        ])

        # ── 5. Early-exit checks ───────────────────────────────────────────────
        if consecutive_solver_failures >= MAX_CONSECUTIVE_FAILURES:
            history["failed"]      = True
            history["fail_reason"] = (
                f"solver failed {consecutive_solver_failures} consecutive steps at step {step}"
            )
            break

        if abs(e_y) > OFFTRACK_LIMIT:
            history["failed"]      = True
            history["fail_reason"] = f"off-track (|e_y|={abs(e_y):.2f} m) at step {step}"
            break

        dist_to_finish = math.hypot(X_g - path_X[-1], Y_g - path_Y[-1])
        if idx >= len(path_X) - 2 or dist_to_finish <= 3.0:
            history["reached_end"]       = True
            history["remaining_steps"]   = max_steps - step
            break

        # ── 6. MPC solve ───────────────────────────────────────────────────────
        # Linearise the bicycle model at the current speed
        current_v    = plant_state[3]
        Ad, Bd       = get_8state_discrete_model(max(current_v, 0.5), dt)

        # Adaptive gain scheduling (model_utils.py)
        kappa         = curvature_estimate(plant_state)     # Instantaneous κ = r/vx
        R_rate_scaled = adaptive_R_rate(kappa, R_rate_w)    # Soften jerk cost in corners
        R_scaled      = adaptive_R_scaling(current_v, R_w)  # Stiffen steering cost at speed

        u_opt = solve_mpc(
            x_current, Ad, Bd, N_horizon,
            Q_w, R_scaled,
            u_bounds_min, u_bounds_max,
            R_rate=R_rate_scaled,
            u_prev=u_prev,
        )

        if u_opt is not None:
            consecutive_solver_failures = 0
        else:
            # Hold previous command on failure; increment failure counter
            consecutive_solver_failures += 1
            u_opt = u_prev.copy()

        u_prev = u_opt.copy()
        history["u_steer"].append(u_opt[0])
        history["u_accel"].append(u_opt[1])

        # ── 7. N-step horizon prediction (visualisation only) ─────────────────
        # Propagates the error state forward with the linear model to give
        # the MPC's look-ahead trajectory for display on the map. This is
        # purely cosmetic — the plant does not use this prediction.
        px, py         = [], []
        x_p_tmp        = x_current.copy()
        current_ref_psi = rpsi

        for k in range(N_horizon):
            e_y_pred = x_p_tmp[0]
            # Project Frenet lateral error back to global XY for display:
            # global_X ≈ X + k*vx*cos(ψ)*dt − e_y*sin(ψ_ref)
            px.append(X_g + (k + 1) * plant_state[3] * np.cos(psi_g) * dt
                      - e_y_pred * np.sin(current_ref_psi))
            py.append(Y_g + (k + 1) * plant_state[3] * np.sin(psi_g) * dt
                      + e_y_pred * np.cos(current_ref_psi))
            x_p_tmp = Ad @ x_p_tmp + Bd @ u_opt   # Propagate error state

        history["pred_X"].append(px)
        history["pred_Y"].append(py)

        # ── 8. Advance the nonlinear plant ────────────────────────────────────
        # The plant is advanced using the full nonlinear model (Pacejka tyres,
        # suspension, aerodynamics). The controller never sees any of this
        # directly — all that feeds back is the tracking error computed from
        # the plant's global X, Y, psi at the next step.
        plant_state = step_nonlinear_plant(plant_state, u_opt, dt, vehicle_params)

    # ── Post-loop: compute completion and bonus fields ─────────────────────────
    history.setdefault("reached_end", False)
    history["peak_lateral_error"] = float(np.max(np.abs(history["e_y"]))) if history["e_y"] else 0.0

    if history["reached_end"]:
        history["completion_frac"] = 1.0
        # Time bonus: how much earlier than max_steps did the vehicle finish?
        expected_time = max_steps * dt
        sim_time      = len(history["X"]) * dt
        history["time_bonus"] = max(0.0, 1.0 - (sim_time / expected_time))
    else:
        # Partial completion: fraction of max_steps used
        history["completion_frac"] = len(history["X"]) / max(max_steps, 1)
        history["time_bonus"]      = 0.0

    return history


def run_simulation(event):
    """
    Callback for the "Start Sim" button. Reads the current sliders and global
    path, computes a dynamic step budget, calls simulate_closed_loop(), stores
    the result, and sets up the time-scrub viewer.

    After the simulation completes, the scrub slider is made visible and
    update_scrub_frame(0) is called to render the first frame. The "Show Metrics"
    and "Start Sim" buttons are managed appropriately.

    Called by: btn_start ("Start Sim" button)
    """
    global is_simulated, sim_history
    if len(path_X) == 0:
        ax_map.set_title("ERROR: Draw a path first!", color="red", fontweight="bold")
        fig.canvas.draw_idle()
        return

    is_simulated = True
    btn_start.set_active(False)
    btn_flip.set_active(False)
    btn_optimize.set_active(False)
    ax_ey0.set_visible(False)
    ax_epsi0.set_visible(False)

    # Dynamic step budget: based on path length at a conservative fallback speed
    dynamic_steps = calculate_dynamic_max_steps(path_X, path_Y, dt=dt)

    history = simulate_closed_loop(
        Q, R, slider_ey0.val, slider_epsi0.val, flip_heading_180,
        rng_seed=None, max_steps=dynamic_steps, R_rate_w=R_rate,
    )
    sim_history = history

    ax_map.set_title(
        "Simulation Complete! Review tracking via Slider below.",
        fontweight="bold", color="darkgreen",
    )

    # Set up time-scrub slider
    ax_scrub.set_visible(True)
    slider_scrub.valmax = len(history["X"]) - 1
    slider_scrub.ax.set_xlim(0, len(history["X"]) - 1)
    slider_scrub.on_changed(update_scrub_frame)
    update_scrub_frame(0)
    btn_optimize.set_active(True)


btn_start.on_clicked(run_simulation)


# ==========================================
# PERFORMANCE METRICS (SHOW METRICS BUTTON)
# ==========================================

def run_optimize(event):
    """
    Score the most recently completed simulation and print a full performance
    breakdown to the console.

    No rollouts are re-run and no weights are modified. All scoring uses
    performance_stats.report_performance_metrics(), which imports SCORE_WEIGHTS
    and bonus constants directly from offline_tuner.py, ensuring live and
    offline scores are computed with the same formula and are directly
    comparable.

    After scoring, the plot title is updated with a summary line showing the
    composite score, lateral RMSE, heading RMSE, and completion percentage.
    The title colour is green for successful runs, crimson for failures.

    Requires a simulation to have been run first (is_simulated = True).

    Called by: btn_optimize ("Show Metrics" button)
    """
    if not is_simulated or not sim_history:
        ax_map.set_title(
            "Run a simulation first (Start Sim), then Show Metrics.",
            fontweight="bold", color="darkorange",
        )
        fig.canvas.draw_idle()
        return

    print("=" * 58)
    metrics = performance_stats.report_performance_metrics(sim_history, log_fn=print)

    ax_map.set_title(
        f"Metrics: composite={metrics['composite_score']:.4f}  "
        f"lat={metrics['lateral_rmse_m']:.3f} m  "
        f"hdg={metrics['heading_rmse_deg']:.2f}°  "
        f"completion={metrics['completion_pct']:.0f}%  "
        f"(see console)",
        fontweight="bold",
        color="darkgreen" if not metrics["failed"] else "crimson",
    )
    fig.canvas.draw_idle()


btn_optimize.on_clicked(run_optimize)


# ==========================================
# TIMELINE SCRUBBING
# ==========================================

def update_scrub_frame(val):
    """
    Redraw the map and telemetry panel for a given history frame index.

    Called every time the scrub slider is moved. Updates:
      - trail_line:      full vehicle trail up to (and including) `frame`
      - pred_line:       MPC horizon prediction at `frame`
      - vehicle_marker:  triangle marker at (X[frame], Y[frame], psi[frame])
      - telemetry_text:  speed, position, heading, errors, and commands at `frame`

    Handles the case where pred_X/pred_Y may be shorter than the full trail
    (e.g. if the simulation failed before the last step produced a prediction).

    Parameters
    ----------
    val : float   Current scrub slider value (converted to int frame index).

    Called by: slider_scrub.on_changed(), run_simulation() (initial frame 0)
    """
    frame = int(val)
    h     = sim_history

    trail_line.set_data(h["X"][: frame + 1], h["Y"][: frame + 1])

    # Guard: pred_X may be shorter than trail on failed/early-exit runs
    safe_frame_post = min(frame, len(h["pred_X"]) - 1) if h["pred_X"] else 0
    if h["pred_X"]:
        pred_line.set_data(h["pred_X"][safe_frame_post], h["pred_Y"][safe_frame_post])
    else:
        pred_line.set_data([], [])

    car_x, car_y = get_car_triangle(h["X"][frame], h["Y"][frame], h["psi"][frame])
    vehicle_marker.set_data(car_x, car_y)

    v_target_str = (
        f"{h['v_target'][frame]:6.2f} m/s"
        if "v_target" in h and len(h["v_target"]) > frame
        else "   n/a"
    )

    # Telemetry panel text — monospaced for column alignment
    text = (
        f"     HISTORIC FRAME: {frame:03d}\n"
        f"=======================\n"
        f"Speed     : {h['v'][frame]:6.2f} m/s\n"
        f"Target Spd: {v_target_str}\n"
        f"Pos X     : {h['X'][frame]:6.2f} m\n"
        f"Pos Y     : {h['Y'][frame]:6.2f} m\n"
        f"Heading   : {np.degrees(h['psi'][frame]):6.1f} deg\n"
        f"-----------------------\n"
        f"Lat Error : {h['e_y'][frame]:6.2f} m\n"
        f"Yaw Error : {np.degrees(h['e_psi'][frame]):6.2f} deg\n"
        f"-----------------------\n"
        f"Steer Cmd : {np.degrees(h['u_steer'][safe_frame_post]):6.1f} deg\n"
        f"Accel Cmd : {h['u_accel'][safe_frame_post]:6.2f} m/s²"
    )
    telemetry_text.set_text(text)
    fig.canvas.draw_idle()


plt.show()