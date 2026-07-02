import numpy as np
import multiprocessing as mp
import time
from collections import Counter
from scipy.interpolate import CubicSpline
import signal

from vehicle_physics import VehicleParams, step_nonlinear_plant, init_plant_state
from model import get_8state_discrete_model
from optimiser import solve_mpc
import speed_profile as sp
import cvxpy as cp
import cma
from sim_track import place_cones, SimPerception, SimPlanner, calculate_dynamic_max_steps
import math


# --- TUNING WEIGHT INDEXES ---
TUNABLE_Q_IDX = [0, 1, 2, 3, 4]  # e_y, e_y_dot, e_psi, e_psi_dot, e_v
TUNABLE_R_IDX = [0, 1]  # delta_cmd, a_cmd
TUNABLE_R_RATE_IDX = [0, 1]  # d(delta_cmd)/dt, d(a_cmd)/dt

# --- PROGRAMMATIC MULTIPLIER BOUNDARIES ---
# Floor lowered to 0.1 so the optimizer can REDUCE weights below the template,
# not only increase them. Previous floor of 1.0 made reduction impossible.
Q_BOUNDS = {
    0: (0.1, 10.0),
    1: (0.1, 10.0),
    2: (0.1, 10.0),
    3: (0.1, 10.0),
    4: (0.1, 10.0),
}

R_BOUNDS = {
    0: (0.1, 10.0),
    1: (0.1, 10.0),
}

R_RATE_BOUNDS = {
    0: (0.1, 10.0),
    1: (0.1, 10.0),
}

# --- PROGRAMMATIC BOUNDS GENERATION ---
bounds = []
for idx in TUNABLE_Q_IDX:
    bounds.append(Q_BOUNDS.get(idx))

for idx in TUNABLE_R_IDX:
    bounds.append(R_BOUNDS.get(idx))

for idx in TUNABLE_R_RATE_IDX:
    bounds.append(R_RATE_BOUNDS.get(idx))

# MPC horizon must match simulation.py exactly so tuned weights transfer
# cleanly to the live simulator.
N_HORIZON = 25

# --- GRADED DNF (did-not-finish) PENALTY ---
# Scaled to ~2-3x the typical finishing score range (~[-0.4, 1.0]) so CMA-ES
# still has a gradient slope toward "get further" while DNF is clearly worse
# than finishing. Previous value of 10.0 created a 10-20x cliff that dominated
# the covariance update and masked the continuous metric gradient.
DNF_PENALTY = 3.0
# Extra penalty on how far past the off-track limit the car was when it failed.
DNF_OFFTRACK_WEIGHT = 1.0

# --- SOLVER SETTINGS FOR HEADLESS ROLLOUTS ---
ROLLOUT_EPS = 1e-4
ROLLOUT_MAX_ITER = 5000

# --- Graceful shutdown flag ---
_stop_requested = False
# Max number of true evaluations (rollouts) to allow before stopping. Can be overridden by env var TUNER_MAX_EVALS.
MAX_EVALS = 1000

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
# NOTE: steer_rms (3), max_steering (5), and steering_sat_ratio (6) previously
# triple-counted the same signal. Weights redistributed: steer_rms removed
# (correlated with smooth_rms already), max_steering trimmed, freed weight
# moved to rmse and peak_lateral_error which are the primary tracking signals.
# smooth_rms (Δu) and jerk_rms (Δ²u) are also correlated but measure different
# timescales of roughness so both are retained with reduced individual weights.
SCORE_WEIGHTS = np.array([
    0.40,  # 0  rmse                (increased — primary tracking signal)
    0.06,  # 1  yaw_rms
    0.07,  # 2  smooth_rms
    0.04,  # 3  steer_rms
    0.03,  # 4  accel_rms
    0.03,  # 5  max_steering
    0.09,  # 6  steering_sat_ratio
    0.10,  # 7  jerk_rms
    0.03,  # 8  max_yaw_rate
    0.02,  # 9  steering_reversals
    0.13,  # 10 peak_lateral_error
], dtype=float)

COMPLETION_BONUS_WEIGHT = 0.30   # subtracted — reward for finishing
TIME_BONUS_WEIGHT       = 0.05   # subtracted — reward for finishing quickly

assert abs(SCORE_WEIGHTS.sum() - 1.0) < 1e-9,
    f"SCORE_WEIGHTS must sum to 1.0, got {SCORE_WEIGHTS.sum():.6f}"

# Module-level dictionary to share initial parameters safely across processes
_init_context: dict = {}

_model_cache = {}

def get_cached_model(vx, dt):
    key = np.round(vx, 1)
    if key not in _model_cache:
        _model_cache[key] = get_8state_discrete_model(key, dt)
    return _model_cache[key]


# ==========================================
# ADAPTIVE MPC GAIN HELPERS
# ==========================================
def curvature_estimate(state):
    """Simple yaw-rate / speed curvature proxy from the plant state vector."""
    vx = max(state[3], 0.5)
    r = state[5]
    return abs(r / vx)


def adaptive_R_rate(kappa, R_rate_base):
    """Curvature-dependent steering jerk softening."""
    R = np.array(R_rate_base, copy=True)
    scale = max(0.35, 1 / (1 + 3 * kappa))
    R[0, 0] *= scale
    return R


def adaptive_R_scaling(vx, R_base):
    """Speed-dependent steering cost shaping with a saturating scale."""
    vx = max(vx, 0.5)
    A = 1.5
    vx_half = 6.0
    steer_scale = 1.0 + (A * vx) / (vx_half + vx)
    accel_scale = 1.0 + 0.05 * vx
    R_scaled = np.array(R_base, copy=True)
    R_scaled[0, 0] *= steer_scale
    R_scaled[1, 1] *= accel_scale
    return R_scaled


# ==========================================
# SYNTHETIC PATH LIBRARY
# ==========================================
def _resample_path(waypoints_x, waypoints_y, n_points=MAX_EVALS):
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
        path_X, path_Y
    )
    path_v = sp.smooth_profile(raw_v, window=9)
    blue_all, yellow_all = place_cones(path_X, path_Y)
    return path_X, path_Y, path_Psi, path_v, blue_all, yellow_all


def _make_arc(cx, cy, radius, theta_start_deg, theta_end_deg, n=20):
    angles = np.linspace(np.radians(theta_start_deg), np.radians(theta_end_deg), n)
    return cx + radius * np.cos(angles), cy + radius * np.sin(angles)


def vector_to_weights(vec, Q_template, R_template, R_rate_template):
    Q = Q_template.copy()
    R = R_template.copy()
    R_rate = R_rate_template.copy()

    n_q = len(TUNABLE_Q_IDX)
    n_r = len(TUNABLE_R_IDX)

    for j, i in enumerate(TUNABLE_Q_IDX):
        base_val = Q_template[i, i] if Q_template[i, i] != 0.0 else 1.0
        Q[i, i] = vec[j] * base_val

    for j, i in enumerate(TUNABLE_R_IDX):
        base_val = R_template[i, i] if R_template[i, i] != 0.0 else 1.0
        R[i, i] = vec[n_q + j] * base_val

    for j, i in enumerate(TUNABLE_R_RATE_IDX):
        base_val = R_rate_template[i, i] if R_rate_template[i, i] != 0.0 else 1.0
        R_rate[i, i] = vec[n_q + n_r + j] * base_val

    return Q, R, R_rate


# Language: Python
# Title: Formula Student Compliant Synthetic Path Library

def build_synthetic_paths():
    """
    Returns a dict mapping path names to (path_X, path_Y, path_Psi, path_v) tuples.
    All geometry is scaled strictly for Formula Student: straights 5-10m, corners R=5-12m.
    No excess straight line distance padding is included to ensure clean scoring.
    """
    paths = {}

    # --- PATH_SUDDEN_TURN ---
    # 5m straight then sharp 90° left (R=6m) — tests late-apex response
    s1x = np.linspace(0, 5, 10)
    s1y = np.zeros(10)
    arc_x, arc_y = _make_arc(5, 6, 6, -90, 0, n=20)
    s2x = np.full(10, 11.0)
    s2y = np.linspace(6, 11, 10)
    wx = np.concatenate([s1x, arc_x[1:], s2x[1:]])
    wy = np.concatenate([s1y, arc_y[1:], s2y[1:]])
    paths["PATH_SUDDEN_TURN"] = _resample_path(wx, wy)

    # --- PATH_S_BEND --- 
    # 5m straight (East), right 90° (R=10m), 5m link (South), left 90° (R=10m), 10m exit (East)
    s0x = np.linspace(15, 20, 10)
    s0y = np.zeros(10)
    arc1x, arc1y = _make_arc(20, -10, 10, 90, 0, n=20)
    lx = np.full(8, 30.0)
    ly = np.linspace(-10, -15, 8)
    arc2x, arc2y = _make_arc(40, -15, 10, 180, 270, n=20)
    s1x = np.linspace(40, 50, 10)
    s1y = np.full(10, -25.0)
    wx = np.concatenate([s0x, arc1x[1:], lx[1:], arc2x[1:], s1x[1:]])
    wy = np.concatenate([s0y, arc1y[1:], ly[1:], arc2y[1:], s1y[1:]])
    paths["PATH_S_BEND"] = _resample_path(wx, wy)

    # --- PATH_SKIDPAD ---
    # Two perfectly tangent circles R=9.125m (FS centerline specification) with 5m entry/exit
    R_skid = 9.125
    s0x = np.zeros(8)
    s0y = np.linspace(-5, 0, 8)
    # Right circle (Clockwise loop around center (R, 0))
    arc1x, arc1y = _make_arc(R_skid, 0, R_skid, 180, -180, n=60)
    # Left circle (Counter-Clockwise loop around center (-R, 0))
    arc2x, arc2y = _make_arc(-R_skid, 0, R_skid, 0, 360, n=60)
    s1x = np.zeros(8)
    s1y = np.linspace(0, 5, 8)
    wx = np.concatenate([s0x, arc1x[1:], arc2x[1:], s1x[1:]])
    wy = np.concatenate([s0y, arc1y[1:], arc2y[1:], s1y[1:]])
    paths["PATH_SKIDPAD"] = _resample_path(wx, wy)

    # --- PATH_SPIRAL ---
    # Continuous clothoid spiral tightening from R=15m down to R=5.5m over a 60m arc length
    s0x = np.linspace(0, 5, 10)
    s0y = np.zeros(10)
    L_spiral = 60.0
    ds_spiral = 0.1
    s_spiral = np.arange(0, L_spiral + ds_spiral, ds_spiral)
    kappa_spiral = (1.0 / 15.0) + ((1.0 / 5.5) - (1.0 / 15.0)) * (s_spiral / L_spiral)
    psi_spiral = -np.cumsum(kappa_spiral * ds_spiral)
    psi_spiral = np.concatenate([[0.0], psi_spiral[:-1]])
    curve_sx = 5.0 + np.cumsum(np.cos(psi_spiral) * ds_spiral)
    curve_sy = 0.0 + np.cumsum(np.sin(psi_spiral) * ds_spiral)
    curve_sx = np.concatenate([[5.0], curve_sx])
    curve_sy = np.concatenate([[0.0], curve_sy])
    wx = np.concatenate([s0x, curve_sx[1:]])
    wy = np.concatenate([s0y, curve_sy[1:]])
    paths["PATH_SPIRAL"] = _resample_path(wx, wy)

    # --- PATH_MICRO_SLALOM ---
    # Cones at tight 9m spacing, matching a technical FS slalom spec with a clean 1.2m amplitude deviation
    wx = np.linspace(0, 45, 7)
    wy = [0, 1.2, -1.2, 1.2, -1.2, 1.2, 0]
    paths["PATH_MICRO_SLALOM"] = _resample_path(wx, wy)

    # --- PATH_OFFSET_CHICANE ---
    # Lateral offset gates at crisp 10m intervals with no padded trailing straights
    wx = [0, 5, 15, 25, 35, 45, 50]
    wy = [0, 0, 2.0, -2.0, 2.0, 0, 0]
    paths["PATH_OFFSET_CHICANE"] = _resample_path(wx, wy)

    # --- PATH_HAIRPIN ---
    # 5m entry straight, ultra-tight 180° right turn centerline (R=5m), 5m exit
    s1x = np.linspace(0, 5, 10)
    s1y = np.zeros(10)
    arc_hp_x, arc_hp_y = _make_arc(5, -5, 5, 90, -90, n=25)
    s2x = np.linspace(5, 0, 10)
    s2y = np.full(10, -10.0)
    wx = np.concatenate([s1x, arc_hp_x[1:], s2x[1:]])
    wy = np.concatenate([s1y, arc_hp_y[1:], s2y[1:]])
    paths["PATH_HAIRPIN"] = _resample_path(wx, wy)

    # --- PATH_CHICANE ---
    # Direct pure tangent S-transition chicane utilizing matching R=6m radii
    s0x = np.linspace(0, 5, 10)
    s0y = np.zeros(10)
    arc_ch1x, arc_ch1y = _make_arc(5, 6, 6, -90, 0, n=15)
    arc_ch2x, arc_ch2y = _make_arc(17, 6, 6, 180, 90, n=15)
    s1x = np.linspace(17, 22, 10)
    s1y = np.full(10, 12.0)
    wx = np.concatenate([s0x, arc_ch1x[1:], arc_ch2x[1:], s1x[1:]])
    wy = np.concatenate([s0y, arc_ch1y[1:], arc_ch2y[1:], s1y[1:]])
    paths["PATH_CHICANE"] = _resample_path(wx, wy)

    # --- PATH_FS_CORNER ---
    # Classic single 90° corner configuration with 5m bounding entries
    s1x = np.linspace(0, 5, 10)
    s1y = np.zeros(10)
    arc_fsc_x, arc_fsc_y = _make_arc(5, -6, 6, 90, 0, n=20)
    s2x = np.full(10, 11.0)
    s2y = np.linspace(-6, -11, 10)
    wx = np.concatenate([s1x, arc_fsc_x[1:], s2x[1:]])
    wy = np.concatenate([s1y, arc_fsc_y[1:], s2y[1:]])
    paths["PATH_FS_CORNER"] = _resample_path(wx, wy)

    # --- PATH_MIXED ---
    # Master track sequence without unneeded straight line padding: 
    # 5m straight -> Left Turn (R=6m) -> 5m North link -> Right Turn (R=6m) -> 5m East link -> Hairpin (R=5m) -> 5m West exit
    s_mix0x = np.linspace(0, 5, 10)
    s_mix0y = np.zeros(10)
    arc_m1x, arc_m1y = _make_arc(5, 6, 6, -90, 0, n=15)
    l_m1x = np.full(6, 11.0)
    l_m1y = np.linspace(6, 11, 6)
    arc_m2x, arc_m2y = _make_arc(17, 11, 6, 180, 90, n=15)
    l_m2x = np.linspace(17, 22, 6)
    l_m2y = np.full(6, 17.0)
    arc_m3x, arc_m3y = _make_arc(22, 12, 5, 90, -90, n=25)
    s_mix1x = np.linspace(22, 17, 10)
    s_mix1y = np.full(10, 7.0)
    wx = np.concatenate([s_mix0x, arc_m1x[1:], l_m1x[1:], arc_m2x[1:], l_m2x[1:], arc_m3x[1:], s_mix1x[1:]])
    wy = np.concatenate([s_mix0y, arc_m1y[1:], l_m1y[1:], arc_m2y[1:], l_m2y[1:], arc_m3y[1:], s_mix1y[1:]])
    paths["PATH_MIXED"] = _resample_path(wx, wy)

    return paths

# Build paths once at import time so workers share them without re-computing
SYNTHETIC_PATHS = build_synthetic_paths()
PATH_LENGTHS = {
    name: np.sum(np.hypot(np.diff(x), np.diff(y)))
    for name, (x, y, _, _, _, _) in SYNTHETIC_PATHS.items()
}
PATH_NAMES = list(SYNTHETIC_PATHS.keys())

# ==========================================
# TRACKING ERROR HELPER
# ==========================================
def _normalize_angle(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def _find_closest(path_X, path_Y, x, y, last_idx, window=40):
    n = len(path_X)
    if last_idx <= 5:
        start, end = 0, min(n, 100)
    else:
        start = max(0, last_idx - 5)
        end = min(n, last_idx + window)
    dx = path_X[start:end] - x
    dy = path_Y[start:end] - y
    local = int(np.argmin(dx * dx + dy * dy))
    return start + local


def _tracking_errors(plant_state, path_X, path_Y, path_Psi, last_idx):
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
    signal.signal(signal.SIGINT, signal.SIG_IGN)

    np.random.seed()
    global _init_context, _model_cache
    _init_context["Q"] = Q_init
    _init_context["R"] = R_init
    _init_context["R_rate"] = R_rate_init
    _init_context["vehicle_params"] = VehicleParams()

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
    Q_init = _init_context["Q"]
    R_init = _init_context["R"]
    R_rate_init = _init_context["R_rate"]

    Q, R, R_rate = vector_to_weights(weights_vector, Q_init, R_init, R_rate_init)

    p = _init_context["vehicle_params"]
    dt = 0.05

    u_min = np.array([-p.max_steer, p.max_accel_brake])
    u_max = np.array([p.max_steer, p.max_accel])

    if path_name is None:
        raise ValueError("path_name must be provided")

    path_X, path_Y, path_Psi, path_v, blue_all, yellow_all = SYNTHETIC_PATHS[path_name]
    dynamic_steps = calculate_dynamic_max_steps(path_X, path_Y, dt=0.05)
    num_steps = dynamic_steps
    perception = SimPerception(blue_all, yellow_all)
    planner    = SimPlanner(v_max=18.0, v_min=2.5)
    _b0, _y0   = perception.visible_cones(float(path_X[0]), float(path_Y[0]), float(path_Psi[0]))
    planner.update(_b0, _y0, np.array([path_X[0], path_Y[0]]), float(path_Psi[0]))

    base_heading = path_Psi[0]
    X0 = path_X[0] - ey0 * np.sin(base_heading)
    Y0 = path_Y[0] + ey0 * np.cos(base_heading)
    psi0 = _normalize_angle(base_heading + epsi0)
    vx0 = float(path_v[0])

    state = init_plant_state(X0, Y0, psi0, vx0=vx0)

    error_cost = 0.0
    yaw_rate_cost = 0.0
    control_smooth = 0.0

    steering_reversals = 0
    last_sign = 0
    max_yaw_rate = 0.0
    steering_effort = 0.0
    steering_saturation = 0.0
    accel_effort = 0.0
    max_steering = 0.0
    max_accel = 0.0
    peak_lateral_error = 0.0
    inaccurate_count = 0
    u_prev = np.zeros(2)
    du_prev = np.zeros(2)
    jerk_cost = 0.0
    idx = 0
    consecutive_fails = 0

    cumulative_distance = 0.0
    last_idx = idx

    dnf = False
    offtrack_excess = 0.0

    MAX_FAILS = 5
    OFFTRACK_LIMIT = 2.5

    path_seg_dist = np.hypot(np.diff(path_X), np.diff(path_Y))

    for step in range(num_steps):
        car_pos_np = np.array([state[0], state[1]])
        b_vis, y_vis = perception.visible_cones(state[0], state[1], state[2])
        planner.update(b_vis, y_vis, car_pos_np, state[2])

        cl = planner.centreline
        if cl is not None and len(cl) >= 2:
            dists = np.linalg.norm(cl - car_pos_np, axis=1)
            cl_idx = int(np.argmin(dists))
            seg = cl[cl_idx + 1] - cl[cl_idx] if cl_idx < len(cl) - 1 else cl[cl_idx] - cl[cl_idx - 1]
            seg_len = float(np.linalg.norm(seg))
            if seg_len > 1e-6:
                t_hat   = seg / seg_len
                right_n = np.array([t_hat[1], -t_hat[0]])
                rpsi    = math.atan2(t_hat[1], t_hat[0])
                e_y     = -float(np.dot(car_pos_np - cl[cl_idx], right_n))
                e_psi   = _normalize_angle(state[2] - rpsi)
            else:
                e_y, e_psi, _ = _tracking_errors(state, path_X, path_Y, path_Psi, idx)
        else:
            e_y, e_psi, idx_new = _tracking_errors(state, path_X, path_Y, path_Psi, idx)
            idx = idx_new

        # Progress tracking still uses original path for consistent scoring
        _, _, idx_ref = _tracking_errors(state, path_X, path_Y, path_Psi, idx)
        if idx_ref > last_idx:
            cumulative_distance += np.sum(path_seg_dist[last_idx:idx_ref])
        last_idx = idx_ref

        _, v_target = planner.get_target(car_pos_np, state[2])
        v_target = float(v_target)
        vx = max(state[3], 0.5)

        e_y_dot = state[3] * np.sin(e_psi) + state[4] * np.cos(e_psi)
        x0_mpc = np.array(
            [e_y, e_y_dot, e_psi, state[5], vx - v_target, 0.0, state[6], state[7]]
        )

        kappa = curvature_estimate(state)
        R_rate_scaled = adaptive_R_rate(kappa, R_rate)
        R_scaled = adaptive_R_scaling(vx, R)
        Ad, Bd = get_cached_model(vx, dt)

        mpc_result = solve_mpc(
            x0_mpc, Ad, Bd, N_HORIZON, Q, R_scaled, u_min, u_max,
            R_rate=R_rate_scaled, u_prev=u_prev, silent=True,
            return_status=True, eps_abs=ROLLOUT_EPS, eps_rel=ROLLOUT_EPS,
            max_iter=ROLLOUT_MAX_ITER,
            warm_start=(step != 0),
        )

        if mpc_result is None:
            consecutive_fails += 1
            u_opt = u_prev.copy()
            control_smooth += 5
        else:
            u_opt, status = mpc_result
            consecutive_fails = 0
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
                if abs(u_opt[0]) > 0.02:
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

    # FIX: preserve sign so good runs (negative score) are not rewarded for
    # solver degradation. Original code divided negative scores which reduced
    # the penalty; now abs(score) is scaled and the sign re-applied.
    if inaccurate_count > 0:
        factor = 1.0 + min(5, inaccurate_count) * 0.1
        score = np.sign(score) * abs(score) * factor

    return score


# ==========================================
# DETERMINISTIC OBJECTIVE WRAPPER
# ==========================================
# PATH_MIXED appears once (equal weight to others). Previously 2x, which
# over-specialised weights toward mixed-path geometry at the cost of transfer.
VALIDATION_SUITE = [
    "PATH_MICRO_SLALOM",
    #"PATH_OFFSET_CHICANE",
    "PATH_SPIRAL",
    "PATH_SUDDEN_TURN",
    #"PATH_SKIDPAD",
    "PATH_S_BEND",
    #"PATH_MIXED",
    #"PATH_HAIRPIN",
    "PATH_CHICANE",
]

INITIAL_CONDITIONS = [
    (0.00, 0.00),
    (0.15, 0.05),
]

def _build_task_table(suite, ics):
    counts = Counter((p, ey0, epsi0) for p in suite for (ey0, epsi0) in ics)
    tasks = list(counts.keys())
    weights = np.array([counts[t] for t in tasks], dtype=float)
    return tasks, weights

EVAL_TASKS, EVAL_WEIGHTS = _build_task_table(VALIDATION_SUITE, INITIAL_CONDITIONS)


def _aggregate_task_scores(task_scores):
    s = np.asarray(task_scores, dtype=float)
    weighted_mean = float(np.sum(EVAL_WEIGHTS * s) / np.sum(EVAL_WEIGHTS))
    worst = float(np.max(s))
    return 0.7 * weighted_mean + 0.3 * worst


def _score_task(args):
    vec, path_name, ey0, epsi0 = args
    return run_headless_rollout(vec, path_name=path_name, ey0=ey0, epsi0=epsi0)


def evaluate_candidate(vec):
    scores = [
        run_headless_rollout(vec, path_name=p, ey0=ey0, epsi0=epsi0)
        for (p, ey0, epsi0) in EVAL_TASKS
    ]
    return float(_aggregate_task_scores(scores))


# ==========================================
# PARALLEL OBJECTIVE (pool-aware)
# ==========================================
# The surrogate calls our objective for one candidate at a time. We evaluate
# all tasks for that candidate in parallel across the process pool so the
# surrogate's serial outer loop still saturates all CPU cores.
# Pool handle is injected into this function before the optimiser starts.
_eval_pool = None
_n_tasks = None


def parallel_evaluate_candidate(vec):
    """
    Objective function passed to fmin_lq_surr2.
    Evaluates all tasks for a single candidate in parallel.
    Falls back to serial evaluation if the pool is not available.
    """
    if _eval_pool is None:
        return evaluate_candidate(vec)

    flat_tasks = [
        (vec, p, ey0, epsi0)
        for (p, ey0, epsi0) in EVAL_TASKS
    ]
    chunksize = max(1, len(flat_tasks) // (_n_tasks * 4))
    task_scores = _eval_pool.map(_score_task, flat_tasks, chunksize=chunksize)
    return float(_aggregate_task_scores(task_scores))


# ==========================================
# MAIN EXECUTION
# ==========================================
Q = np.diag([5.0, 5.0, 5.0, 5.0, 5.0, 0.0, 0.0, 0.0])
R = np.diag([5.0, 5.0])
R_rate = np.diag([5.0, 5.0])

if __name__ == "__main__":
    Q_init = Q
    R_init = R
    R_rate_init = R_rate

    print("[Offline Tuner] Building synthetic path library...")
    for name, (px, py, _, _, _, _) in SYNTHETIC_PATHS.items():
        print(
            f"  {name}: {len(px)} points "
            f"X=[{px.min():.1f},{px.max():.1f}] "
            f"Y=[{py.min():.1f},{py.max():.1f}]"
        )

    num_params = len(bounds)
    lower = np.array([b[0] for b in bounds])
    upper = np.array([b[1] for b in bounds])
    param_ranges = upper - lower

    # Initial point: midpoint of each bound (avoids bias toward floor/ceiling)
    x0 = (lower + upper) / 2.0

    # CMA-ES initial step size: 23% of the parameter range (per-dim)
    # initial step size is a critical hyperparameter for CMA-ES; too small and the search stagnates, too large and the search is inefficient.
    # signma0 references the initial standard deviation for the CMA-ES algorithm, which controls how much exploration occurs in the parameter space. 
    sigma0 = 0.75
    cma_stds = 0.23 * param_ranges

    # Runtime knobs (env-overridable):
    #   TUNER_POPSIZE   — CMA-ES base population size (default: heuristic)
    #   TUNER_MAX_EVALS — total true-evaluation budget across all restarts
    #   TUNER_RESTARTS  — max BIPOP restarts (default: 9)
    default_popsize = int(5 + np.floor(3 * np.log(num_params)))
    popsize = default_popsize

    # Budget in true evaluations (surrogate skips many; this controls wall time)
    max_evals = MAX_EVALS
    max_restarts = 9

    num_cores = max(1, mp.cpu_count() - 1)

    print("\n[Offline Tuner] Strategy: BIPOP + lq-CMA-ES (surrogate-assisted)")
    print(f"  Parameters:    {num_params}")
    print(f"  x0 (midpoint): {np.round(x0, 2).tolist()}")
    print(f"  sigma0:        {sigma0}  |  per-dim stds: {np.round(cma_stds, 3).tolist()}")
    print(f"  Base popsize:  {popsize}  |  max restarts: {max_restarts}")
    print(f"  True-eval budget: {max_evals}  (surrogate reduces actual rollouts ~3-10x)")
    print(f"  Eval tasks/candidate: {len(EVAL_TASKS)}")
    print(f"  Workers: {num_cores}")

    # ------------------------------------------------------------------
    # CMA-ES OPTIONS
    # All restarts share these options. BIPOP internally multiplies
    # popsize by incpopsize=2 for large restarts and uses small populations
    # for the interleaved local restarts, so popsize here is just the seed.
    # ------------------------------------------------------------------
    cma_options = {
        "bounds":           [lower.tolist(), upper.tolist()],
        "CMA_stds":         cma_stds,
        "popsize":          popsize,
        "seed":             42,
        "verb_disp":        False,
        "verb_log":         0,
        "verbose":          -9,
        "CMA_active":       True,
        # Stagnation detection re-enabled: with restarts, a stagnated run
        # should trigger a restart rather than grinding to the eval budget.
        # tolstagnation default (~100+100*n/popsize) is fine for this problem.
        "tolconditioncov":  1e14,
        "maxfevals":        max_evals,
    }

    start_time = time.time()

    # ------------------------------------------------------------------
    # POOL + SURROGATE LAUNCH
    # The pool is opened here and injected into parallel_evaluate_candidate
    # via module globals. fmin_lq_surr2 calls our objective serially for
    # each candidate it decides to truly evaluate (the surrogate predicts
    # the rest); each true-eval call parallelises across tasks internally.
    # ------------------------------------------------------------------
    with mp.Pool(
        processes=num_cores,
        initializer=init_worker,
        initargs=(Q_init, R_init, R_rate_init),
    ) as pool:

        # Inject pool into the module-level objective so the surrogate can
        # reach it without passing arguments through pycma's interface.
        _eval_pool = pool
        _n_tasks = num_cores

        # Populate the main-process context so evaluate_candidate works for
        # the post-optimisation comparison step.
        _init_context["Q"] = Q_init
        _init_context["R"] = R_init
        _init_context["R_rate"] = R_rate_init
        _init_context["vehicle_params"] = VehicleParams()
        dt = 0.05
        for vx in np.arange(2.5, 16.1, 0.1):
            key = np.round(vx, 1)
            if key not in _model_cache:
                _model_cache[key] = get_8state_discrete_model(key, dt)

        def _handle_sigint(sig, frame):
            global _stop_requested
            if not _stop_requested:
                print("\n[Tuner] Ctrl+C caught — finishing current generation then stopping...")
                _stop_requested = True

        signal.signal(signal.SIGINT, _handle_sigint)

        generation_log = []

        def _log_callback(es):
            """Per-generation logging callback passed to fmin_lq_surr2."""
            gen_best = es.best.f if es.best.f is not None else float("inf")
            generation_log.append({
                "gen":   es.countiter,
                "evals": es.countevals,
                "best":  gen_best,
                "sigma": es.sigma,
            })
            running_best = min(e["best"] for e in generation_log)
            print(
                f"[lq-CMA-ES] gen {es.countiter:4d} | "
                f"true_evals {es.countevals:5d} | "
                f"gen_best {gen_best:.4f} | "
                f"overall_best {running_best:.4f} | "
                f"sigma {es.sigma:.4e}"
            )
            if _stop_requested:
                es.opts["maxfevals"] = 0  # tells pycma to stop after this generation

        # ------------------------------------------------------------------
        # fmin_lq_surr2 — BIPOP + quadratic surrogate restarts
        #
        # incpopsize=2: large restarts double population (IPOP schedule).
        # restarts=max_restarts: total restart budget.
        # inject=True: re-inject surrogate's predicted optimum each gen.
        # keep_model=False: fresh surrogate per restart (avoids stale fits).
        # ------------------------------------------------------------------
        best_vec, es = cma.fmin_lq_surr2(
            parallel_evaluate_candidate,
            x0,
            sigma0,
            options=cma_options,
            restarts=max_restarts,
            incpopsize=2,
            inject=True,
            keep_model=False,
            callback=_log_callback,
        )

    # Pool is closed; clear the reference.
    _eval_pool = None

    end_time = time.time()

    # ------------------------------------------------------------------
    # POST-OPTIMISATION: compare xbest vs xfavorite (distribution mean)
    # and pick whichever scores lower on a fresh serial evaluation.
    # ------------------------------------------------------------------
    mean_vec = es.result.xfavorite
    score_best = evaluate_candidate(best_vec)
    score_mean = evaluate_candidate(mean_vec)
    final_vec = best_vec if score_best <= score_mean else mean_vec

    best_Q, best_R, best_R_rate = vector_to_weights(
        final_vec, Q_init, R_init, R_rate_init
    )

    total_true_evals = es.result.evaluations

    print("\n" + "=" * 60)
    print(f"OPTIMIZATION COMPLETE in {(end_time - start_time) / 60:.2f} min")
    print(f"True evaluations used: {total_true_evals}  (budget: {max_evals})")
    print(f"Restarts completed:    {es.result.stop.get('maxrestarts', '?')}")
    print(f"Best score (xbest):    {score_best:.4f}")
    print(f"Best score (xfavorite): {score_mean:.4f}")
    print(f"Selected:              {'xbest' if score_best <= score_mean else 'xfavorite'}")
    print("=" * 60)
    print("\nReplace your simulation.py weights with:")
    print("Q_diag      =", np.diag(best_Q).tolist())
    print("R_diag      =", np.diag(best_R).tolist())
    print("R_rate_diag =", np.diag(best_R_rate).tolist())
    print("=" * 60)

    # ------------------------------------------------------------------
    # GENERATION LOG SUMMARY
    # ------------------------------------------------------------------
    if generation_log:
        evals_arr = [e["evals"] for e in generation_log]
        best_arr  = [e["best"]  for e in generation_log]
        running_best = np.minimum.accumulate(best_arr)
        # Print improvement milestones
        print("\nImprovement milestones (true-eval count → score):")
        last_reported = None
        for ev, rb in zip(evals_arr, running_best):
            if last_reported is None or rb < last_reported * 0.99:
                print(f"  evals={ev:5d}  →  {rb:.5f}")
                last_reported = rb