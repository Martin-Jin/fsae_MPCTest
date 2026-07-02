# Language: python
# Title: Offline Auto-Tuner with Synthetic Path Library (offline_tuner.py)

import os
import numpy as np
import multiprocessing as mp
import time
from collections import Counter
from scipy.interpolate import CubicSpline

from vehicle_physics import VehicleParams, step_nonlinear_plant, init_plant_state
from model import get_8state_discrete_model
from optimiser import solve_mpc
import speed_profile as sp
import cvxpy as cp
import cma


# --- TUNING WEIGHT INDEXES ---
TUNABLE_Q_IDX = [0, 1, 2, 3, 4]  # e_y, e_y_dot, e_psi, e_psi_dot, e_v
TUNABLE_R_IDX = [0, 1]  # delta_cmd, a_cmd
TUNABLE_R_RATE_IDX = [0, 1]  # d(delta_cmd)/dt, d(a_cmd)/dt

# --- PROGRAMMATIC MULTIPLIER BOUNDARIES ---
# Adjusted to prevent explosive values while still allowing sufficient flexibility.
Q_MULTIPLIER_BOUNDS = {
    0: (0.1, 400.0),
    1: (0.1, 200.0),
    2: (0.1, 350.0),
    3: (0.1, 600.0),
    4: (0.1, 200.0),
}

R_MULTIPLIER_BOUNDS = {
    0: (0.1, 100.0),
    1: (0.1, 100.0),
}

# Raised the floor from 0.1 to 0.25 to prevent numerical ill-conditioning against Q
R_RATE_MULTIPLIER_BOUNDS = {
    0: (0.1, 100.0),
    1: (0.1, 50.0),
}

# --- PROGRAMMATIC BOUNDS GENERATION ---
bounds = []
for idx in TUNABLE_Q_IDX:
    bounds.append(Q_MULTIPLIER_BOUNDS.get(idx))

for idx in TUNABLE_R_IDX:
    bounds.append(R_MULTIPLIER_BOUNDS.get(idx))

for idx in TUNABLE_R_RATE_IDX:
    bounds.append(R_RATE_MULTIPLIER_BOUNDS.get(idx))

# MPC horizon must match simulation.py exactly so tuned weights transfer
# cleanly to the live simulator.
N_HORIZON = 25

# --- GRADED DNF (did-not-finish) PENALTY ---
# Added to a failed rollout's score, scaled by the fraction of the path NOT
# covered. Sized large enough that finishing is clearly preferred, while the
# (1 - progress) slope still guides CMA-ES toward candidates that get further.
DNF_PENALTY = 10.0
# Extra penalty on how far past the off-track limit the car was when it failed.
DNF_OFFTRACK_WEIGHT = 1.0

# --- SOLVER SETTINGS FOR HEADLESS ROLLOUTS ---
# The tuner runs millions of QP solves, so it trades a little precision for
# speed vs. the live simulator's defaults: a looser tolerance and a lower
# iteration ceiling let hard/ill-conditioned QPs bail out sooner instead of
# grinding to max_iter. The live sim (simulation.py) omits these args and keeps
# solve_mpc's tighter defaults.
ROLLOUT_EPS = 1e-4
ROLLOUT_MAX_ITER = 5000

# Speed profile parameters — identical to simulation.py's constants so the
# headless rollouts see the same v_target distribution as the live sim.
SP_V_MAX = 16.0
SP_MU = 0.6
SP_A_ACCEL_MAX = 2.5
SP_A_BRAKE_MAX = 4.0
SP_V_MIN = 2.5

# ==========================================
# SCORING WEIGHTS
# ==========================================
# One array, one place to edit. Indices map to metrics in this order:
#   0: rmse               — primary lateral tracking error (RMS)
#   1: yaw_rms            — yaw rate stability
#   2: smooth_rms         — control smoothness (delta-u RMS)
#   3: steer_rms          — steering effort (RMS)
#   4: accel_rms          — acceleration effort (RMS)
#   5: max_steering       — peak steering command
#   6: steering_sat_ratio — fraction of steps at steering saturation
#   7: jerk_rms           — control jerk (second derivative of u)
#   8: max_yaw_rate       — peak yaw rate
#   9: steering_reversals — count of steering direction changes
#  10: peak_lateral_error — worst single-step lateral error
#
# Positive weights sum to 1.0. Completion/time bonuses are subtracted separately.
SCORE_WEIGHTS = np.array([
    0.35,  # 0  rmse
    0.10,  # 1  yaw_rms
    0.08,  # 2  smooth_rms
    0.03,  # 3  steer_rms
    0.01,  # 4  accel_rms
    0.02,  # 5  max_steering
    0.12,  # 6  steering_sat_ratio
    0.09,  # 7  jerk_rms
    0.03,  # 8  max_yaw_rate
    0.02,  # 9  steering_reversals
    0.15,  # 10 peak_lateral_error
], dtype=float)

COMPLETION_BONUS_WEIGHT = 0.40   # subtracted — reward for finishing
TIME_BONUS_WEIGHT       = 0.05   # subtracted — reward for finishing quickly

assert abs(SCORE_WEIGHTS.sum() - 1.0) < 1e-9, \
    f"SCORE_WEIGHTS must sum to 1.0, got {SCORE_WEIGHTS.sum():.6f}"

# Module-level dictionary to share initial parameters safely across processes
_init_context: dict = {}

_model_cache = {}

def get_cached_model(vx, dt):
    # Bin speed to 0.1 m/s. The 8-state ZOH model (a matrix exponential per
    # speed) varies smoothly with vx, so 0.1 m/s bins are indistinguishable
    # from finer keys for control purposes while cutting the number of expm
    # builds ~10x over a full tuning run.
    key = np.round(vx, 1)
    if key not in _model_cache:
        _model_cache[key] = get_8state_discrete_model(key, dt)
    return _model_cache[key]


# ==========================================
# ADAPTIVE MPC GAIN HELPERS
# ==========================================
# NOTE: These must be defined *before* run_headless_rollout uses them.
# When the tuner runs under the default "fork" start method, worker
# processes are forked at Pool() creation time, so any function defined
# below the __main__ block would be missing from the child namespace and
# every rollout would raise NameError. Keep them up here.
def curvature_estimate(state):
    """Simple yaw-rate / speed curvature proxy from the plant state vector.
    state: [X, Y, psi, vx, vy, r, delta_act, a_act]
    """
    vx = max(state[3], 0.5)
    r = state[5]
    return abs(r / vx)


def adaptive_R_rate(kappa, R_rate_base):
    """
    Curvature-dependent steering jerk softening.

    In tight corners we allow smoother (less penalized) steering transitions
    so the controller can unwind quickly without fighting the rate penalty.
    """
    R = np.array(R_rate_base, copy=True)

    # Steering: soften in high curvature (scale → 0 as kappa → large)
    scale = max(
    0.35,
    1/(1+3*kappa))
    R[0, 0] *= scale

    # Accel/brake: keep full baseline penalty in all conditions
    # R[1, 1] unchanged

    return R


def adaptive_R_scaling(vx, R_base):
    """
    Speed-dependent steering cost shaping with a saturating scale.

    CHANGE: Replaced the old linear scale (1 + 0.25*vx) with a saturating
    (Michaelis-Menten) function:
        steer_scale = 1 + (A * vx) / (vx_half + vx)
    where A=1.5 and vx_half=6.0, giving:
        vx=0  → scale=1.0x  (baseline)
        vx=6  → scale=1.75x (50% of asymptote)
        vx=10 → scale=~2.0x (approaching asymptote at 2.5x)

    The old linear formula gave 3.5x at 10 m/s, which over-penalized
    steering corrections at the top of the speed profile and caused
    the controller to under-respond to heading errors at high speed.

    Accel scale remains a mild linear function of vx (unchanged).
    """
    vx = max(vx, 0.5)

    A = 1.5  # asymptotic gain above baseline
    vx_half = 6.0  # speed at which scale = 1 + A/2
    steer_scale = 1.0 + (A * vx) / (vx_half + vx)

    accel_scale = 1.0 + 0.05 * vx

    R_scaled = np.array(R_base, copy=True)
    R_scaled[0, 0] *= steer_scale
    R_scaled[1, 1] *= accel_scale

    return R_scaled


# ==========================================
# SYNTHETIC PATH LIBRARY
# ==========================================
def _resample_path(waypoints_x, waypoints_y, n_points=600):
    """
    Fit a clamped cubic spline through the given waypoints and resample to
    n_points, returning (path_X, path_Y, path_Psi, path_v_profile).
    Identical spline/heading logic to simulation.py's on_release().
    """
    wx = np.asarray(waypoints_x, dtype=float)
    wy = np.asarray(waypoints_y, dtype=float)
    t = np.linspace(0.0, 1.0, len(wx))

    d0 = np.array([wx[1] - wx[0], wy[1] - wy[0]]) / (t[1] - t[0])
    dN = np.array([wx[-1] - wx[-2], wy[-1] - wy[-2]]) / (t[-1] - t[-2])

    cs_x = CubicSpline(t, wx, bc_type=((1, d0[0]), (1, dN[0])))
    cs_y = CubicSpline(t, wy, bc_type=((1, d0[1]), (1, dN[1])))

    t_fine = np.linspace(0.0, 1.0, n_points)
    path_X = cs_x(t_fine)
    path_Y = cs_y(t_fine)
    dx = cs_x.derivative()(t_fine)
    dy = cs_y.derivative()(t_fine)
    path_Psi = np.arctan2(dy, dx)

    raw_v = sp.compute_speed_profile(
        path_X,
        path_Y,
        v_max=SP_V_MAX,
        mu=SP_MU,
        a_accel_max=SP_A_ACCEL_MAX,
        a_brake_max=SP_A_BRAKE_MAX,
        v_min=SP_V_MIN,
    )
    path_v = sp.smooth_profile(raw_v, window=9)
    return path_X, path_Y, path_Psi, path_v


def _make_arc(cx, cy, radius, theta_start_deg, theta_end_deg, n=20):
    """Helper: points along a circular arc."""
    angles = np.linspace(np.radians(theta_start_deg), np.radians(theta_end_deg), n)
    return cx + radius * np.cos(angles), cy + radius * np.sin(angles)


def vector_to_weights(vec, Q_template, R_template, R_rate_template):
    """
    Interpret the optimization vector elements as dynamic multipliers
    applied directly to the baseline template weights.

    This keeps the optimizer search spaces normalized while maintaining
    the absolute physical scaling ratios for the vehicle.
    """
    Q = Q_template.copy()
    R = R_template.copy()
    R_rate = R_rate_template.copy()

    n_q = len(TUNABLE_Q_IDX)
    n_r = len(TUNABLE_R_IDX)

    # Apply dynamic multipliers to Q
    for j, i in enumerate(TUNABLE_Q_IDX):
        # Scale relative to the template's initial value.
        # If the template value is 0.0, fall back to 1.0 to prevent zero-lock.
        base_val = Q_template[i, i] if Q_template[i, i] != 0.0 else 1.0
        Q[i, i] = vec[j] * base_val

    # Apply dynamic multipliers to R
    for j, i in enumerate(TUNABLE_R_IDX):
        base_val = R_template[i, i] if R_template[i, i] != 0.0 else 1.0
        R[i, i] = vec[n_q + j] * base_val

    # Apply dynamic multipliers to R_rate
    for j, i in enumerate(TUNABLE_R_RATE_IDX):
        base_val = R_rate_template[i, i] if R_rate_template[i, i] != 0.0 else 1.0
        R_rate[i, i] = vec[n_q + n_r + j] * base_val

    return Q, R, R_rate

def build_synthetic_paths():
    """
    Returns a dict of named (path_X, path_Y, path_Psi, path_v) tuples.

    PATH_GENTLE_CURVE  — 150 m straight followed by a wide 90° left bend
                         (R=40 m). Tests steady-state lateral tracking and
                         gentle speed reduction into a gradual corner.

    PATH_HAIRPIN       — 80 m straight, 180° right hairpin (R=8 m), then
                         80 m exit straight. Tests heavy braking, maximum
                         curvature lateral demand, and re-acceleration.

    PATH_SUDDEN_TURN   — 100 m full-speed straight, then an abrupt 90° left
                         turn (R=12 m). Tests emergency steering response and
                         the controller's ability to track a direction change
                         it had very little horizon preview of.

    PATH_S_BEND        — symmetric S-shape: right curve then left curve,
                         each 90° at R=18 m separated by a 20 m straight.
                         Tests lateral direction reversal and the transition
                         between left and right cornering loads.

    PATH_CHICANE       — rapid left-right-left chicane with tight R=10 m
                         arcs and short 15 m linking straights. Tests high-
                         frequency steering input and R_rate interaction with
                         the adaptive gain in rapid direction changes.

    PATH_MIXED         — one continuous path that chains a straight, gentle
                         curve, S-bend, and hairpin in sequence. This is the
                         primary scoring path for the DE objective because it
                         exercises all regimes in a single rollout, avoiding
                         the risk of over-fitting weights to a single shape.
    """
    paths = {}

    # --- PATH_GENTLE_CURVE ---
    # Long straight east, then sweep north via a wide arc
    straight_x = np.linspace(0, 150, 30)
    straight_y = np.zeros(30)
    arc_x, arc_y = _make_arc(150, 40, 40, -90, 0, n=25)
    wx = np.concatenate([straight_x, arc_x[1:]])
    wy = np.concatenate([straight_y, arc_y[1:]])
    paths["PATH_GENTLE_CURVE"] = _resample_path(wx, wy)

    # --- PATH_HAIRPIN ---
    # East straight, tight 180° right hairpin, west exit
    s1x = np.linspace(0, 80, 20)
    s1y = np.zeros(20)
    arc_x, arc_y = _make_arc(80, -8, 8, 90, -90, n=30)  # right (clockwise)
    s2x = np.linspace(80, 0, 20)
    s2y = np.full(20, -16.0)
    wx = np.concatenate([s1x, arc_x[1:], s2x[1:]])
    wy = np.concatenate([s1y, arc_y[1:], s2y[1:]])
    paths["PATH_HAIRPIN"] = _resample_path(wx, wy)

    # --- PATH_SUDDEN_TURN ---
    # Long straight, then sharp 90° left turn
    s1x = np.linspace(0, 100, 25)
    s1y = np.zeros(25)
    arc_x, arc_y = _make_arc(100, 12, 12, -90, 0, n=20)
    s2x = np.full(10, 112.0)
    s2y = np.linspace(12, 50, 10)
    wx = np.concatenate([s1x, arc_x[1:], s2x[1:]])
    wy = np.concatenate([s1y, arc_y[1:], s2y[1:]])
    paths["PATH_SUDDEN_TURN"] = _resample_path(wx, wy)

    # --- PATH_S_BEND ---
    # Short straight, right curve, short link, left curve, exit straight
    s0x = np.linspace(0, 20, 10)
    s0y = np.zeros(10)
    arc1x, arc1y = _make_arc(20, -18, 18, 90, 0, n=20)  # right
    lx = np.linspace(38, 58, 8)
    ly = np.full(8, -18.0)
    arc2x, arc2y = _make_arc(58, -36, 18, 90, 180, n=20)  # left
    s1x = np.linspace(40, 20, 8)
    s1y = np.full(8, -54.0)
    wx = np.concatenate([s0x, arc1x[1:], lx[1:], arc2x[1:], s1x[1:]])
    wy = np.concatenate([s0y, arc1y[1:], ly[1:], arc2y[1:], s1y[1:]])
    paths["PATH_S_BEND"] = _resample_path(wx, wy)

    # --- PATH_CHICANE ---
    # Left-right-left three-element chicane with tight R=10 m arcs
    s0x = np.linspace(0, 30, 10)
    s0y = np.zeros(10)
    arc1x, arc1y = _make_arc(30, 10, 10, -90, 0, n=15)  # left up
    l1x = np.linspace(40, 55, 6)
    l1y = np.full(6, 10.0)
    arc2x, arc2y = _make_arc(55, 0, 10, 90, 0, n=15)  # right down (return)
    l2x = np.linspace(65, 80, 6)
    l2y = np.zeros(6)
    arc3x, arc3y = _make_arc(80, 10, 10, -90, 0, n=15)  # left up again
    s1x = np.linspace(90, 120, 8)
    s1y = np.full(8, 10.0)
    wx = np.concatenate(
        [s0x, arc1x[1:], l1x[1:], arc2x[1:], l2x[1:], arc3x[1:], s1x[1:]]
    )
    wy = np.concatenate(
        [s0y, arc1y[1:], l1y[1:], arc2y[1:], l2y[1:], arc3y[1:], s1y[1:]]
    )
    paths["PATH_CHICANE"] = _resample_path(wx, wy)

    # --- PATH_MIXED (primary scoring path) ---
    # Straight → gentle curve → S-style reverse → hairpin → exit
    # Built as a continuous sequence of waypoints so the spline flows
    # naturally between segments without kinks.
    s0x = np.linspace(0, 80, 20)
    s0y = np.zeros(20)
    # Wide left bend (R=30)
    arc1x, arc1y = _make_arc(80, 30, 30, -90, 0, n=20)
    # Short north straight
    l1x = np.full(8, 110.0)
    l1y = np.linspace(30, 60, 8)
    # Right sweep (R=20)
    arc2x, arc2y = _make_arc(90, 60, 20, 0, 90, n=15)
    # Link west
    l2x = np.linspace(90, 50, 10)
    l2y = np.full(10, 80.0)
    # Tight left hairpin (R=8)
    arc3x, arc3y = _make_arc(50, 72, 8, 90, 270, n=25)
    # Exit south-east
    s1x = np.linspace(58, 120, 15)
    s1y = np.linspace(72, 40, 15)
    wx = np.concatenate(
        [s0x, arc1x[1:], l1x[1:], arc2x[1:], l2x[1:], arc3x[1:], s1x[1:]]
    )
    wy = np.concatenate(
        [s0y, arc1y[1:], l1y[1:], arc2y[1:], l2y[1:], arc3y[1:], s1y[1:]]
    )
    paths["PATH_MIXED"] = _resample_path(wx, wy)

    # --- PATH_FIGURE8 ---
    t = np.linspace(0, 2 * np.pi, 120)

    wx = 15 * np.sin(t)
    wy = 8 * np.sin(2 * t)

    paths["PATH_FIGURE8"] = _resample_path(wx, wy)

    # --- PATH_SPIRAL ---
    theta = np.linspace(0, 4 * np.pi, 120)

    r = np.linspace(40, 4, len(theta))

    wx = r * np.cos(theta)
    wy = r * np.sin(theta)

    paths["PATH_SPIRAL"] = _resample_path(wx, wy)

    # --- PATH_MICRO_SLALOM ---
    wx = [0, 5, 10, 15, 20, 25, 30, 35, 40]
    wy = [0, 2.5, -2.5, 2.5, -2.5, 2.5, -2.5, 2.5, 0]

    paths["PATH_MICRO_SLALOM"] = _resample_path(wx, wy)

    # --- PATH_DOUBLE_70 ---
    wx = [0, 5, 10, 13, 15, 16, 17, 18, 20, 23, 26, 29, 31, 32, 33, 34, 35, 40, 45]

    wy = [
        0,
        0,
        0,
        0.5,
        1.5,
        3.0,
        5.0,
        6.2,
        7.0,
        7.0,
        6.2,
        5.0,
        3.0,
        1.5,
        0.5,
        0,
        -1.5,
        -1.5,
        -1.5,
    ]

    paths["PATH_DOUBLE_70"] = _resample_path(wx, wy)

    # Offset chicane for testing lateral acceleration and R_rate scaling
    wx = [0, 10, 20, 25, 30, 35, 40, 45, 50, 60]

    wy = [0, 0, 0, 3, -3, 3, -3, 0, 0, 0]

    paths["PATH_OFFSET_CHICANE"] = _resample_path(wx, wy)

    # Tightening path for testing extreme curvature and lateral acceleration
    theta = np.linspace(-90, 0, 40)

    radii = np.linspace(18, 5, len(theta))

    x = 80 + radii * np.cos(np.radians(theta))
    y = radii * np.sin(np.radians(theta))

    wx = np.concatenate([np.linspace(0, 80, 25), x])
    wy = np.concatenate([np.zeros(25), y])

    paths["PATH_TIGHTENING"] = _resample_path(wx, wy)

    return paths


# Build paths once at import time so workers share them without re-computing
SYNTHETIC_PATHS = build_synthetic_paths()
PATH_LENGTHS = {
    name: np.sum(np.hypot(np.diff(x), np.diff(y)))
    for name, (x, y, _, _) in SYNTHETIC_PATHS.items()
}
PATH_NAMES = list(SYNTHETIC_PATHS.keys())

# ==========================================
# TRACKING ERROR HELPER
# ==========================================
def _normalize_angle(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def _find_closest(path_X, path_Y, x, y, last_idx, window=40):
    """Bounded closest-point search matching simulation.py's logic."""
    n = len(path_X)
    if last_idx <= 5:
        start, end = 0, min(n, 100)
    else:
        start = max(0, last_idx - 5)
        end = min(n, last_idx + window)
    dx = path_X[start:end] - x
    dy = path_Y[start:end] - y
    local = int(np.argmin(dx * dx + dy * dy))  # squared dist, no sqrt needed
    return start + local


def _tracking_errors(plant_state, path_X, path_Y, path_Psi, last_idx):
    """
    Compute (e_y, e_psi, v_target_idx, new_idx) from global plant state
    and a reference path, identical to simulation.py's error computation.
    """
    X, Y, psi = plant_state[0], plant_state[1], plant_state[2]
    idx = _find_closest(path_X, path_Y, X, Y, last_idx)
    rx, ry, rpsi = path_X[idx], path_Y[idx], path_Psi[idx]

    dx = X - rx
    dy = Y - ry
    e_y = dy * np.cos(rpsi) - dx * np.sin(rpsi)
    e_psi = _normalize_angle(psi - rpsi)
    return e_y, e_psi, idx


# ==========================================
# WORKER INITIALIZER
# ==========================================
def init_worker(Q_init, R_init, R_rate_init):
    """
    Runs immediately when a new worker process is spawned.
    Populates the global memory context for that child process and
    pre-warms the model cache across the expected speed range so
    there is zero model-build latency during rollouts.
    """
    np.random.seed()
    global _init_context, _model_cache
    _init_context["Q"] = Q_init
    _init_context["R"] = R_init
    _init_context["R_rate"] = R_rate_init
    _init_context["vehicle_params"] = VehicleParams()

    # Pre-warm: build all ZOH models for vx in [2.5, 16.0] at 0.1 m/s bins.
    # Cost is paid once at worker startup rather than on the critical path.
    dt = 0.05
    for vx in np.arange(2.5, 16.1, 0.1):
        key = np.round(vx, 1)
        if key not in _model_cache:
            _model_cache[key] = get_8state_discrete_model(key, dt)


# ==========================================
# HEADLESS SIMULATION ROLLOUT
# ==========================================
def run_headless_rollout(
    weights_vector,
    path_name=None,
    num_steps=350,
    ey0=0.0,
    epsi0=0.0,
):
    """
    Run one closed-loop rollout on a synthetic reference path with the
    given weight vector.
    """
    Q_init = _init_context["Q"]
    R_init = _init_context["R"]
    R_rate_init = _init_context["R_rate"]

    Q, R, R_rate = vector_to_weights(weights_vector, Q_init, R_init, R_rate_init)

    p = _init_context["vehicle_params"]
    dt = 0.05

    u_min = [-0.4, -10.0]
    u_max = [0.4, 4.0]

    if path_name is None:
        raise ValueError("path_name must be provided")

    path_X, path_Y, path_Psi, path_v = SYNTHETIC_PATHS[path_name]

    base_heading = path_Psi[0]
    X0 = path_X[0] - ey0 * np.sin(base_heading)
    Y0 = path_Y[0] + ey0 * np.cos(base_heading)
    psi0 = _normalize_angle(base_heading + epsi0)
    vx0 = float(path_v[0])

    state = init_plant_state(X0, Y0, psi0, vx0=vx0)

    error_cost = 0.0
    yaw_rate_cost = 0.0
    control_smooth = 0.0

    # Additional control effort penalties
    steering_reversals = 0
    last_sign = 0
    max_yaw_rate = 0.0
    steering_effort = 0.0
    steering_saturation = 0.0
    accel_effort = 0.0
    max_steering = 0.0
    max_accel = 0.0
    peak_lateral_error = 0.0
    inaccurate_count = 0  # Track solver degradation
    u_prev = np.zeros(2) 
    du_prev = np.zeros(2)
    jerk_cost = 0.0
    idx = 0
    consecutive_fails = 0

    cumulative_distance = 0.0
    last_idx = idx

    # DNF (did-not-finish) bookkeeping. Instead of returning a flat 1e3 the
    # moment a rollout goes wrong, we flag it, break, and fold a *graded*
    # penalty into the final score (see scoring block). A flat cliff makes the
    # CMA-ES landscape a plateau with no gradient toward "get further along the
    # path"; a progress-scaled penalty gives the optimiser a slope to descend.
    dnf = False
    offtrack_excess = 0.0

    MAX_FAILS = 5
    OFFTRACK_LIMIT = 3.0  # Tightened slightly for racing tolerances

    # Precompute reference segment distances (path arc-length per index step)
    path_seg_dist = np.hypot(
        np.diff(path_X),
        np.diff(path_Y)
    )

    for step in range(num_steps):
        e_y, e_psi, idx = _tracking_errors(state, path_X, path_Y, path_Psi, idx)
        # accumulate distance along reference path
        if idx > last_idx:
            cumulative_distance += np.sum(path_seg_dist[last_idx:idx])

        last_idx = idx
        v_target = float(path_v[idx])
        vx = max(state[3], 0.5)

        e_y_dot = state[3] * np.sin(e_psi) + state[4] * np.cos(e_psi)
        x0_mpc = np.array(
            [e_y, e_y_dot, e_psi, state[5], vx - v_target, 0.0, state[6], state[7]]
        )

        kappa = curvature_estimate(state)
        R_rate_scaled = adaptive_R_rate(kappa, R_rate)
        R_scaled = adaptive_R_scaling(vx, R)
        Ad, Bd = get_cached_model(vx, dt)

        # ----------------------------
        # MPC Solve: Muted & Status Tracked
        # ----------------------------
        mpc_result = solve_mpc(
            x0_mpc,
            Ad,
            Bd,
            N_HORIZON,
            Q,
            R_scaled,
            u_min,
            u_max,
            R_rate=R_rate_scaled,
            u_prev=u_prev,
            silent=True,
            return_status=True,
            eps_abs=ROLLOUT_EPS,
            eps_rel=ROLLOUT_EPS,
            max_iter=ROLLOUT_MAX_ITER,
            # Cold-start the first solve so this rollout is independent of any
            # solver state left by a previous rollout (keeps scoring order-free).
            warm_start=(step != 0),
        )

        if mpc_result is None:
            consecutive_fails += 1
            u_opt = u_prev.copy()
            control_smooth += 5
        else:
            u_opt, status = mpc_result
            consecutive_fails = 0

            # Catch cvxpy's exact string representation of OPTIMAL_INACCURATE
            if status == cp.OPTIMAL_INACCURATE or status == "optimal_inaccurate":
                inaccurate_count += 1

        if consecutive_fails >= MAX_FAILS:
            dnf = True
            num_steps = step + 1
            break

        if step > 60 and cumulative_distance < 3.0:
            dnf = True
            num_steps = step + 1
            break

        current_sign = np.sign(u_opt[0])

        if current_sign != 0:
            if last_sign != 0 and current_sign != last_sign:
                threshold = 0.02
                if abs(u_opt[0]) > threshold:
                    steering_reversals += 1

            last_sign = current_sign

        max_yaw_rate = max(max_yaw_rate, abs(state[5]))
        error_cost += e_y**2 + 0.5 * e_psi**2
        yaw_rate_cost += 0.8 * state[5] ** 2
        control_smooth += np.sum((u_opt - u_prev) ** 2)

        du = u_opt - u_prev
        jerk = du - du_prev
        jerk_cost += np.sum(jerk**2)
        du_prev = du

        # Penalise excessive control effort
        steering_effort += u_opt[0] ** 2
        accel_effort += u_opt[1] ** 2
        if abs(u_opt[0]) > 0.95 * u_max[0]:
            steering_saturation += 1.0

        peak_lateral_error = max(peak_lateral_error, abs(e_y))

        max_steering = max(max_steering, abs(u_opt[0]))
        max_accel = max(max_accel, abs(u_opt[1]))

        if abs(e_y) > OFFTRACK_LIMIT:
            offtrack_excess = abs(e_y) - OFFTRACK_LIMIT
            dnf = True
            num_steps = step + 1
            break

        if idx >= len(path_X) - 2:
            num_steps = step + 1
            break

        u_prev = u_opt.copy()
        state = step_nonlinear_plant(state, u_opt, dt, p)

    rmse               = np.sqrt(error_cost    / max(num_steps, 1))
    yaw_rms            = np.sqrt(yaw_rate_cost / max(num_steps, 1))
    smooth_rms         = np.sqrt(control_smooth / max(num_steps, 1))
    steer_rms          = np.sqrt(steering_effort / max(num_steps, 1))
    accel_rms          = np.sqrt(accel_effort  / max(num_steps, 1))
    jerk_rms           = np.sqrt(jerk_cost     / max(num_steps, 1))
    steering_sat_ratio = steering_saturation   / max(num_steps, 1)

    metrics = np.array([
        rmse,
        yaw_rms,
        smooth_rms,
        steer_rms,
        accel_rms,
        max_steering,
        steering_sat_ratio,
        jerk_rms,
        max_yaw_rate,
        float(steering_reversals),
        peak_lateral_error,
    ])

    score = float(SCORE_WEIGHTS @ metrics)

    # Reward completing the course
    progress = cumulative_distance / PATH_LENGTHS[path_name]
    progress = np.clip(progress, 0.0, 1.0)

    target_speed_mean = np.mean(path_v)
    expected_time = PATH_LENGTHS[path_name] / max(target_speed_mean, 1.0)
 
    if dnf:
        time_bonus = 0.0
    else:
        sim_time = step * dt
        time_bonus = max(0.0, 1.0 - (sim_time / expected_time))

    score -= COMPLETION_BONUS_WEIGHT * progress + TIME_BONUS_WEIGHT * time_bonus

    if dnf:
        score += DNF_PENALTY * (1.0 - progress)
        score += DNF_OFFTRACK_WEIGHT * offtrack_excess ** 2

    if inaccurate_count > 0:
        factor = 1.0 + min(5, inaccurate_count) * 0.1
        score = score * factor if score > 0 else score / factor

    return score


# ==========================================
# DETERMINISTIC OBJECTIVE WRAPPER
# ==========================================
# Deterministic evaluation profile representing diverse conditions. Paths listed
# more than once are intentionally up-weighted (e.g. PATH_MIXED is the primary
# scoring path). run_headless_rollout is deterministic in (vec, path, ey0,
# epsi0), so instead of re-simulating duplicated entries we collapse the suite
# to unique (path, ic) tasks with integer weights and reproduce the original
# mean exactly via a weighted average.
VALIDATION_SUITE = [
    "PATH_MIXED",
    "PATH_MIXED",
    "PATH_MICRO_SLALOM",
    "PATH_MICRO_SLALOM",
    "PATH_DOUBLE_70",
    "PATH_DOUBLE_70",
    "PATH_TIGHTENING",
    "PATH_TIGHTENING",
    "PATH_HAIRPIN",
    "PATH_CHICANE",
    "PATH_OFFSET_CHICANE",
    "PATH_SPIRAL",
    "PATH_FIGURE8",
    "PATH_GENTLE_CURVE",
    "PATH_SUDDEN_TURN",
    "PATH_S_BEND",
]

INITIAL_CONDITIONS = [
    (0.00, 0.00),
    (0.15, 0.05),
]

def _build_task_table(suite, ics):
    """Collapse (suite x ics) into unique (path, ey0, epsi0) tasks + weights."""
    counts = Counter((p, ey0, epsi0) for p in suite for (ey0, epsi0) in ics)
    tasks = list(counts.keys())
    weights = np.array([counts[t] for t in tasks], dtype=float)
    return tasks, weights

QUICK_SUITE = ["PATH_MIXED", "PATH_HAIRPIN", "PATH_MICRO_SLALOM"]

if os.environ.get("TUNER_QUICK") == "1":
    EVAL_TASKS, EVAL_WEIGHTS = _build_task_table(QUICK_SUITE, INITIAL_CONDITIONS)
    print("[Offline Tuner] QUICK mode: using reduced validation suite.")
else:
    EVAL_TASKS, EVAL_WEIGHTS = _build_task_table(VALIDATION_SUITE, INITIAL_CONDITIONS)


def _aggregate_task_scores(task_scores):
    """Combine per-task scores into a single fitness (weighted mean + worst).

    Matches the original 0.7*mean + 0.3*worst objective; the weighted mean
    reproduces the duplicate-entry up-weighting of VALIDATION_SUITE.
    """
    s = np.asarray(task_scores, dtype=float)
    weighted_mean = float(np.sum(EVAL_WEIGHTS * s) / np.sum(EVAL_WEIGHTS))
    worst = float(np.max(s))
    return 0.7 * weighted_mean + 0.3 * worst


def _score_task(args):
    """Worker entry point: run a single rollout for one (vec, path, ic) task."""
    vec, path_name, ey0, epsi0 = args
    return run_headless_rollout(vec, path_name=path_name, ey0=ey0, epsi0=epsi0)


def evaluate_candidate(vec):
    """
    Evaluate a candidate weight vector deterministically over the unique
    validation tasks. Serial fallback used when not flattening across a pool.
    """
    scores = [
        run_headless_rollout(vec, path_name=p, ey0=ey0, epsi0=epsi0)
        for (p, ey0, epsi0) in EVAL_TASKS
    ]
    return float(_aggregate_task_scores(scores))


# ==========================================
# MAIN EXECUTION
# ==========================================
Q = np.diag(
    [10.0, 10.0, 10.0, 10.0, 10.0, 0.0, 0.0, 0.0]
)
R = np.diag(
    [10.0, 5.0]
)
R_rate = np.diag(
    [10.0, 1]
)

if __name__ == "__main__":
    Q_init = Q
    R_init = R
    R_rate_init = R_rate

    print("[Offline Tuner] Building synthetic path library...")
    for name, (px, py, _, _) in SYNTHETIC_PATHS.items():
        print(
            f"  {name}: {len(px)} points "
            f"X=[{px.min():.1f},{px.max():.1f}] "
            f"Y=[{py.min():.1f},{py.max():.1f}]"
        )

    num_params = len(bounds)
    x0 = np.ones(num_params)

    sigma0 = 1.0 # good default for multiplier tuning

    # Runtime knobs (env-overridable) so a full sweep and a quick smoke test
    # share one entry point:
    #   TUNER_POPSIZE   — CMA-ES population size (default: CMA's heuristic)
    #   TUNER_MAX_GEN   — generation cap (default 20)
    #   TUNER_QUICK=1   — use the reduced QUICK_SUITE (see EVAL_TASKS above)
    default_popsize = 4 + int(3 * np.log(len(x0)))
    popsize = int(os.environ.get("TUNER_POPSIZE", default_popsize))
    max_gen = int(os.environ.get("TUNER_MAX_GEN", 10))

    print("\n[Offline Tuner] Initializing CMA-ES...")
    print(f"  Parameters: {num_params}")
    print(f"  Initial sigma: {sigma0}")
    print(f"  Population: {popsize} | max generations: {max_gen}")
    print(f"  Eval tasks/candidate: {len(EVAL_TASKS)}")

    num_cores = max(1, mp.cpu_count() - 1)
    print(f"[Offline Tuner] Using {num_cores} workers")

    with mp.Pool(
        processes=num_cores,
        initializer=init_worker,
        initargs=(Q_init, R_init, R_rate_init),
    ) as pool:

        # -----------------------------
        # CMA-ES CONFIG
        # -----------------------------
        lower = np.array([b[0] for b in bounds])
        upper = np.array([b[1] for b in bounds])
        param_ranges = upper - lower
        # Set per-parameter initial step size to ~30% of each bound's width.
        # This explores more aggressively early than a flat sigma=0.35, while
        # still respecting the tighter R_rate floor (0.25) vs the wider Q range.
        cma_stds = 0.3 * param_ranges

        es = cma.CMAEvolutionStrategy(
            x0,
            sigma0, 
            {
                "bounds": [lower, upper],
                "CMA_stds": cma_stds,
                "popsize": popsize,
                "seed": 42,
                "verb_disp": False,   # you print your own per-generation line
                "verb_log": 0,        # no file I/O
                "verbose": -9,        # suppress internal output
                "CMA_active": True,
                "tolstagnation": 0,   # disable: weight tuning landscapes plateau legitimately
                "tolconditioncov": 1e14,  # prevent alleviate_conditioning from perturbing CMA_stds
            },
        )

        start_time = time.time()
        generation = 0
        n_tasks = len(EVAL_TASKS)
        running_best = float('inf')

        while not es.stop():

            solutions = es.ask()

            # -----------------------------
            # PARALLEL EVALUATION
            # -----------------------------
            # Flatten to (candidate x task) work units so all workers stay busy
            # even when popsize < num_cores. Each candidate's tasks form a
            # contiguous block in the flat result list, then are aggregated.
            flat_tasks = [
                (vec, p, ey0, epsi0)
                for vec in solutions
                for (p, ey0, epsi0) in EVAL_TASKS
            ]
            chunksize = max(1, len(flat_tasks) // (num_cores * 4))
            flat_scores = pool.map(_score_task, flat_tasks, chunksize=chunksize)

            scores = [
                _aggregate_task_scores(flat_scores[i * n_tasks:(i + 1) * n_tasks])
                for i in range(len(solutions))
            ]

            es.tell(solutions, scores)

            # -----------------------------
            # EARLY TERMINATION LOGIC
            # -----------------------------
            gen_best = min(scores)
            running_best = min(running_best, gen_best)
            print(f"[CMA-ES] Gen {generation} | gen_best: {gen_best:.4f} | overall_best: {running_best:.4f} | sigma: {es.sigma:.4e}")

            generation += 1

            # optional safety stop
            if generation >= max_gen:
                print("[CMA-ES] Max generations reached.")
                break

        result = es.result

    end_time = time.time()

    # After the pool closes, populate context in the main process for evaluate_candidate
    _init_context["Q"] = Q_init
    _init_context["R"] = R_init
    _init_context["R_rate"] = R_rate_init
    _init_context["vehicle_params"] = VehicleParams()

    # Also pre-warm the model cache in the main process
    dt = 0.05
    for vx in np.arange(2.5, 16.1, 0.1):
        key = np.round(vx, 1)
        if key not in _model_cache:
            _model_cache[key] = get_8state_discrete_model(key, dt)

    best_vec = result.xbest
    mean_vec = result.xfavorite
    score_best = evaluate_candidate(best_vec)
    score_mean = evaluate_candidate(mean_vec)
    final_vec = best_vec if score_best < score_mean else mean_vec

    best_Q, best_R, best_R_rate = vector_to_weights(
        final_vec, Q_init, R_init, R_rate_init
    )

    print("\n" + "=" * 50)
    print(f"OPTIMIZATION COMPLETE in {(end_time - start_time) / 60:.2f} min")
    print(f"Best Score: {result.fbest:.4f}")
    print("=" * 50)

    print("Replace your simulation.py weights with:")
    print("Q_diag      =", np.diag(best_Q).tolist())
    print("R_diag      =", np.diag(best_R).tolist())
    print("R_rate_diag =", np.diag(best_R_rate).tolist())
    print("=" * 50)
