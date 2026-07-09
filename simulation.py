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
  4.  Compute tracking errors (e_y, e_psi) 
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
  Imports from: vehicle_physics, performance_stats, speed_profile, offline_tuner
                (SYNTHETIC_PATHS/PATH_NAMES at import time, get_cached_model at
                runtime), sim_track, rollout_core, settings.

DOES NOT USE (at runtime, beyond what's listed above)
-------------------------------------------------------
  No tuner/CMA-ES optimisation logic from offline_tuner.py runs during
  simulation. Note: this file does call offline_tuner.get_cached_model() every
  simulation step (via rollout_core's model_lookup parameter) — a plain
  model-cache lookup, not tuning logic, but a genuine runtime dependency,
  unlike SYNTHETIC_PATHS/PATH_NAMES which are read once at import.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, Slider
from scipy.interpolate import CubicSpline
from vehicle_physics import VehicleParams
from performance_stats import benchmark_weights, report_performance_metrics
import speed_profile
from offline_tuner import SYNTHETIC_PATHS, PATH_NAMES, get_cached_model
from sim_track import place_cones
from rollout_core import run_core_rollout, compute_step_budget

from settings import (
    USE_PLANNER,
    ROLLOUT_MAX_ITER,
    ROLLOUT_EPS,
    Q_diag,
    R_diag,
    R_rate_diag
)

# ==========================================
# SETUP AND CONFIGURATION
# ==========================================
N_horizon = 25      # MPC prediction horizon (steps = 1.25 s of look-ahead at 20 Hz)
v_ref     = 7.0     # Fallback constant speed (m/s); only used if path_v_profile is empty

# ── MPC Cost Weight Matrices ───────────────────────────────────────────────────

Q      = np.diag(Q_diag)       # State cost matrix (8×8 diagonal)
R      = np.diag(R_diag)       # Input cost matrix (2×2 diagonal)
R_rate = np.diag(R_rate_diag)  # Input rate-of-change cost matrix (2×2 diagonal)

# ── Global GUI State ────────────────────────────────────────────────────────────
is_drawing          = False          # True while user is dragging a path
is_simulated        = False          # True after a simulation has been run
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
# Layout: 8-row × 2-column gridspec.
#   Left column : row 0     = map (tall)
#                 rows 1-3  = sliders (ey0, epsi0, scrub)
#   Right column: rows 0-4  = buttons (one per row)
#                 rows 5-7  = telemetry text panel
# Sliders share the left column beneath the map; the right column runs
# buttons at the top and the telemetry box at the bottom independently.
# ── Figure and gridspec ────────────────────────────────────────────────────────
# 9 uniform rows × 2 columns.
#   Left  col: rows 0-5 = map, rows 6-8 = three sliders
#   Right col: rows 0-4 = five equal-height buttons, rows 5-8 = telemetry box
# All rows share the same height so buttons are uniform and the map simply
# spans more of them.
fig = plt.figure(figsize=(13.0, 8.0))
gs  = fig.add_gridspec(
    9, 2,
    width_ratios=[3.8, 1.4],
    height_ratios=[1, 1, 1, 1, 1, 1, 1, 1, 1],   # 9 equal rows
    left=0.06, right=0.97, top=0.95, bottom=0.04,
    wspace=0.20, hspace=0.45,
)

ax_map = fig.add_subplot(gs[0:6, 0])   # Map spans rows 0-5 of left column

# Plot lines — updated each scrub frame and during simulation
(path_line,)         = ax_map.plot([], [], "r--",  label="Target Path",            linewidth=2)
(trail_line,)        = ax_map.plot([], [], "b-",   label="Actual Vehicle Trail",   alpha=0.6)
(pred_line,)         = ax_map.plot([], [], "c-o",  label="MPC Horizon Prediction", markersize=3, alpha=0.8)
(vehicle_marker,)    = ax_map.plot([], [], "g-",   linewidth=2.0,                  label="Vehicle")
(blue_cones_line,)   = ax_map.plot([], [], color="blue", marker="o", linestyle="None", markersize=4, label="Blue Cones")
(yellow_cones_line,) = ax_map.plot([], [], color="gold", marker="o", linestyle="None", markersize=4, label="Yellow Cones")

ax_map.set_xlim(0, 75)
ax_map.set_ylim(0, 45)
ax_map.set_aspect("equal")
ax_map.grid(True)
ax_map.set_title("Robust High-Speed Path MPC Sandbox", fontweight="bold", fontsize=10)
ax_map.legend(loc="upper right", fontsize=8)

# ── Sliders — left column rows 6, 7, 8 (below map) ───────────────────────────
ax_ey0   = fig.add_subplot(gs[6, 0])
ax_epsi0 = fig.add_subplot(gs[7, 0])
ax_scrub = fig.add_subplot(gs[8, 0])

pos_map = ax_map.get_position()
slider_w = pos_map.width * 0.9
slider_x = pos_map.x0 + 0.08

ax_ey0.set_position([slider_x, pos_map.y0 - 0.08, slider_w, 0.03])
ax_epsi0.set_position([slider_x, pos_map.y0 - 0.14, slider_w, 0.03])

slider_ey0   = Slider(ax_ey0,   "Initial Lat Error", -4.0,  4.0,  valinit=0.0, valfmt="%0.1f m", color="orange")
slider_epsi0 = Slider(ax_epsi0, "Initial Yaw Error", -30.0, 30.0, valinit=0.0, valfmt="%0.1f°",  color="orange")
slider_scrub = Slider(ax_scrub, "Time",               0,    1,    valinit=0,   valfmt="%d",       color="teal")

for s in [slider_ey0, slider_epsi0, slider_scrub]:
    s.label.set_fontsize(10.5)
    s.valtext.set_fontsize(10.5)

# All sliders hidden on startup; revealed when a path is loaded/drawn
ax_ey0.set_visible(False)
ax_epsi0.set_visible(False)
ax_scrub.set_visible(False)

# ── Buttons — right column rows 0-4 (one per row, equal height) ───────────────
ax_btn_load      = fig.add_subplot(gs[0, 1])
ax_btn_start     = fig.add_subplot(gs[1, 1])
ax_btn_reset     = fig.add_subplot(gs[2, 1])
ax_btn_optimize  = fig.add_subplot(gs[3, 1])
ax_btn_benchmark = fig.add_subplot(gs[4, 1])

btn_load      = Button(ax_btn_load,      "Load Test Path",      color="thistle",    hovercolor="plum")
btn_start     = Button(ax_btn_start,     "Start Sim",           color="lightgreen", hovercolor="limegreen")
btn_reset     = Button(ax_btn_reset,     "Reset Environment",   color="tomato",     hovercolor="crimson")
btn_optimize  = Button(ax_btn_optimize,  "Show Metrics",        color="lightblue",  hovercolor="deepskyblue")
btn_benchmark = Button(ax_btn_benchmark, "Benchmark All Paths", color="lightyellow",hovercolor="gold")

# ── Telemetry panel — right column rows 5-8 (top-anchored below buttons) ──────
# Spans four rows; text is top-anchored so it fills downward from just below
# the last button, matching the map's vertical extent on the left.
ax_info = fig.add_subplot(gs[5:9, 1])
ax_info.axis("off")

# Telemetry text — full axes width, top-anchored, centred horizontally
telemetry_text = ax_info.text(
    0.5, 1.0, "",
    family="monospace", fontsize=9.5, verticalalignment="top",
    horizontalalignment="center",
    transform=ax_info.transAxes,
    bbox=dict(facecolor="#f8f9fa", edgecolor="#ccced1", boxstyle="round,pad=0.8"),
)

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

    Called by: on_release(), update_scrub_frame(),
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

    Called by: simulate_closed_loop(), load_test_path()
    """
    return np.arctan2(np.sin(angle), np.cos(angle))


# ==========================================
# INTERACTIVE EVENT HANDLERS
# ==========================================

def reset_environment(event):
    """
    Reset all global state, clear all plot lines, and return the GUI to its
    initial pre-draw state.

    Called by: btn_reset ("Reset Environment" button)
    """
    global is_simulated, drawn_points, path_X, path_Y, \
           path_Psi, path_v_profile, sim_history, current_test_path_idx

    is_simulated          = False
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

    ax_map.set_xlim(0, 75)
    ax_map.set_ylim(0, 45)

    # Hide all sliders back to the pristine environment state
    ax_ey0.set_visible(False)
    ax_epsi0.set_visible(False)
    ax_scrub.set_visible(False)

    btn_load.set_active(True)
    btn_start.set_active(True)
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
    global path_X, path_Y, path_Psi, path_v_profile, \
           current_test_path_idx, _blue_cones_all, _yellow_cones_all

    if is_simulated:
        return

    # Advance cycle index
    current_test_path_idx = (current_test_path_idx + 1) % len(PATH_NAMES)
    path_name = PATH_NAMES[current_test_path_idx]

    # Unpack the pre-computed geometry and speed profile
    path_X, path_Y, path_Psi, path_v_profile, _, _ = SYNTHETIC_PATHS[path_name]

    # Generate and render cones
    _blue_cones_all, _yellow_cones_all = place_cones(path_X, path_Y)
    if len(_blue_cones_all) > 0:
        blue_cones_line.set_data(_blue_cones_all[:, 0], _blue_cones_all[:, 1])
        yellow_cones_line.set_data(_yellow_cones_all[:, 0], _yellow_cones_all[:, 1])

    path_line.set_data(path_X, path_Y)

    # Place vehicle marker at path start
    car_x, car_y = get_car_triangle(path_X[0], path_Y[0], path_Psi[0])
    vehicle_marker.set_data(car_x, car_y)

    # Reveal the condition sliders now that a path exists
    ax_ey0.set_visible(True)
    ax_epsi0.set_visible(True)

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
    global is_drawing, path_X, path_Y, path_Psi, path_v_profile
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
    _blue_cones_all, _yellow_cones_all = place_cones(path_X, path_Y)
    if len(_blue_cones_all) > 0:
        blue_cones_line.set_data(_blue_cones_all[:, 0], _blue_cones_all[:, 1])
        yellow_cones_line.set_data(_yellow_cones_all[:, 0], _yellow_cones_all[:, 1])

    path_line.set_data(path_X, path_Y)
    car_x, car_y = get_car_triangle(path_X[0], path_Y[0], path_Psi[0])
    vehicle_marker.set_data(car_x, car_y)

    # Reveal the condition sliders now that a path has been drawn
    ax_ey0.set_visible(True)
    ax_epsi0.set_visible(True)

    fig.canvas.draw_idle()

# Register event handlers
fig.canvas.mpl_connect("button_press_event",   on_press)
fig.canvas.mpl_connect("motion_notify_event",  on_motion)
fig.canvas.mpl_connect("button_release_event", on_release)
btn_load.on_clicked(load_test_path)
btn_reset.on_clicked(reset_environment)


# ==========================================
# SIMULATION ENGINE
# ==========================================
 
def simulate_closed_loop(Q_w, R_w, ey0, epsi0, rng_seed=None, max_steps=None, R_rate_w=None, use_planner=USE_PLANNER):
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

    Furthermore, this simulation also simulates controller input delay. How much delay
    can be changed with the DELAY_STEPS constant at the top of the file.

    PERCEPTION AND PLANNING
    -----------------------
    Controlled by the `use_planner` parameter. When True, SimPerception filters
    the static cone map to the car's FOV and SimPlanner accumulates observations
    to rebuild the centreline and speed profile each step, matching the real ROS2
    pipeline. When False (default), tracking errors and speed targets are derived
    directly from the true reference path — faster and deterministic, which is
    preferred for tuning. The planner-in-the-loop path falls back to the reference
    path while the planner warms up (insufficient cones to build a centreline).

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
      - Off-track:  |e_y| > OFFTRACK_LIMIT (TRACK_HALF_WIDTH * 1.3) → history["failed"] = True
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
    rng_seed : int or None, optional
        Seed for initial condition jitter RNG. None = deterministic.
    max_steps : int, optional
        Maximum simulation steps. Overridden by calculate_dynamic_max_steps()
        in run_simulation(). Default 400.
    R_rate_w : np.ndarray, shape (2, 2), optional
        Rate-of-change cost matrix. Defaults to module-level R_rate if None.
    use_planner : bool, optional
        If True, use SimPerception + SimPlanner centreline for tracking errors
        and speed profile. If False (default), use the true reference path
        directly. Toggling allows comparison of planner-in-the-loop vs oracle.

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

    # Optional initial condition jitter (for multi-rollout averaging) —
    # this is a simulation.py-only feature; offline_tuner always runs
    # deterministic ICs, so the jitter stays here rather than in rollout_core.
    rng         = np.random.default_rng(rng_seed)
    jitter_ey   = rng.normal(0, 0.05) if rng_seed is not None else 0.0
    jitter_epsi = rng.normal(0, 1.0)  if rng_seed is not None else 0.0   # degrees

    ey0_eff   = ey0   + jitter_ey
    epsi0_eff = epsi0 + jitter_epsi   # degrees

    if max_steps is None:
        # Match offline_tuner.py exactly: compute the budget internally
        # rather than requiring the caller to duplicate the formula.
        dynamic_max_steps, max_steps = compute_step_budget(path_X, path_Y, path_v_profile)
    else:
        dynamic_max_steps, _ = compute_step_budget(path_X, path_Y, path_v_profile)

    rollout = run_core_rollout(
        path_X, path_Y, path_Psi, path_v_profile,
        _blue_cones_all, _yellow_cones_all,
        Q_w, R_w, R_rate_w, u_bounds_min, u_bounds_max, vehicle_params,
        ey0=ey0_eff, epsi0=np.radians(epsi0_eff),
        max_steps=max_steps, dynamic_max_steps=dynamic_max_steps,
        use_planner=use_planner, model_lookup=get_cached_model,
        n_horizon=N_horizon, eps=ROLLOUT_EPS, max_iter=ROLLOUT_MAX_ITER,
        want_history=True, want_horizon_pred=True,
    )

    return rollout["history"]


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
    btn_optimize.set_active(False)
    ax_ey0.set_visible(False)
    ax_epsi0.set_visible(False)

    # Step budget is now computed inside simulate_closed_loop() (via
    # rollout_core.compute_step_budget) — identical formula to offline_tuner.
    history = simulate_closed_loop(
        Q, R, slider_ey0.val, slider_epsi0.val,
        rng_seed=None, R_rate_w=R_rate,
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
    and bonus constants directly from scoring.py, ensuring live and offline scores are computed
    with the same formula and are directly comparable.

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
    metrics = report_performance_metrics(sim_history, log_fn=print)

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


def run_benchmark(event):
    """
    Callback for "Benchmark All Paths". Runs every synthetic path 3 times
    and prints a full score breakdown to the console. No simulation needs
    to have been run first — uses the current weight matrices directly.

    This is a blocking call (~minutes); the GUI will be unresponsive until
    all rollouts complete. The plot title updates when done.

    Called by: btn_benchmark ("Benchmark All Paths" button)
    """
    ax_map.set_title(
        "Benchmarking all paths (3× each) — see console. GUI is busy...",
        fontweight="bold", color="darkorange",
    )
    fig.canvas.draw_idle()
    plt.pause(0.05)   # Flush the title update before the blocking loop starts

    results = benchmark_weights(Q, R, R_rate, n_repeats=3, log_fn=print)

    ax_map.set_title(
        f"Benchmark complete: mean score = {results['mean_score']:.4f}  "
        f"({len(PATH_NAMES)} paths × 3 repeats — see console)",
        fontweight="bold", color="darkgreen",
    )
    fig.canvas.draw_idle()


btn_benchmark.on_clicked(run_benchmark)


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
        f"    HISTORIC FRAME: {frame:03d}\n"
        f"========================\n"
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