"""
offline_tuner.py — Offline MPC Weight Optimisation via Surrogate-Assisted CMA-ES

PURPOSE
-------
Automatically searches for the best MPC cost weight matrices (Q, R, R_rate) by
running thousands of headless closed-loop simulations and minimising a composite
performance score. The result is a set of weight diagonals that can be pasted
directly into simulation.py to improve live simulator performance.

HOW IT WORKS — THE OPTIMISATION LOOP
--------------------------------------
The tuner uses CMA-ES (Covariance Matrix Adaptation Evolution Strategy), a
black-box derivative-free optimiser well-suited to noisy, non-convex objective
functions like closed-loop vehicle performance. The specific variant used is
BIPOP + lq-CMA-ES (via the `cma` library's fmin_lq_surr2):

  BIPOP (Bi-Population):
      Interleaves "large" restarts (doubling population each time, broad
      exploration) with "small" restarts (reduced population, local refinement).
      This escapes local minima while still exploiting promising regions.

  Surrogate assistance (lq = local quadratic):
      A local quadratic surrogate model is fitted to the most recently
      evaluated candidates. The surrogate predicts the objective for new
      candidates without running a full rollout (~3-10× speedup). True
      rollouts are only run for candidates that the surrogate predicts are
      promising, or periodically to keep the surrogate accurate.

  Parallel evaluation:
      Each candidate's score is computed by running it across all EVAL_TASKS
      (path × initial condition combinations) in parallel using a
      multiprocessing Pool. The surrogate's serial outer loop therefore still
      saturates all CPU cores.

HOW SCORING WORKS
-----------------
Each rollout produces 12 performance metrics (RMSE, yaw stability, control
smoothness, etc. — see scoring.py's IDX_* constants). These are combined
into a single scalar "composite score" via a weighted dot product
(SCORE_WEIGHTS). Lower is better.

  Completion bonus: subtracted if the vehicle finishes the path.
  Time bonus:       subtracted if the vehicle finishes quickly.
  DNF penalty:      Added if vehicle does not the finish the track.

The objective function evaluated by CMA-ES is a weighted combination of the
mean score across all tasks and the worst-case score (70% mean + 30% worst),
ensuring the tuned weights generalise across path types and don't over-fit to
one scenario.

SEARCH SPACE
------------
CMA-ES searches over multiplicative scaling factors (one per tunable weight):
    Q[i,i]        = vec[j] * Q_template[i,i]
    R[i,i]        = vec[j] * R_template[i,i]
    R_rate[i,i]   = vec[j] * R_rate_template[i,i]

Each factor is bounded in [0.1, 10.0] (Q_BOUNDS, R_BOUNDS, R_RATE_BOUNDS),
so the search explores ±1 decade around the template values. The floor of 0.1
allows weights to be reduced below the template — a crucial capability that
was missing when the floor was 1.0.

SYNTHETIC PATH LIBRARY
----------------------
The tuner evaluates candidates on a library of synthetic FS-spec paths that
cover representative corner types:
  PATH_SUDDEN_TURN  — single sharp 90° corner, tests late-apex response
  PATH_S_BEND       — paired corners (right then left), tests weight transfer
  PATH_SKIDPAD      — two full circles (currently disabled/commented out)
  PATH_SPIRAL       — continuously tightening corner, tests progressive response
  PATH_MICRO_SLALOM — tight slalom gates, tests rapid direction changes
  PATH_OFFSET_CHICANE — lateral offset gates
  PATH_HAIRPIN      — ultra-tight 180° turn
  PATH_ACCELERATION — straight-line acceleration run
  PATH_CHICANE      — S-transition between matched-radius arcs
  PATH_FS_CORNER    — classic single 90° corner
  PATH_MIXED        — combined sequence: corner + link + corner + hairpin

Only the VALIDATION_SUITE subset is used for evaluation
to balance coverage vs. computation time. The full library is available for
manual testing.

USED BY
-------
  Standalone script: run with `python offline_tuner.py` to start optimisation.
  simulation.py: imports SYNTHETIC_PATHS, PATH_NAMES, curvature_estimate,
                 adaptive_R_rate, adaptive_R_scaling, SCORE_WEIGHTS,
                 COMPLETION_BONUS_WEIGHT, TIME_BONUS_WEIGHT, DNF_PENALTY
  performance_stats.py: imports SCORE_WEIGHTS, COMPLETION_BONUS_WEIGHT,
                         TIME_BONUS_WEIGHT, DNF_PENALTY

DOES NOT USE (as module)
-----------------------
  performance_stats.py (performance_stats imports from this file, not vice versa)
"""

import numpy as np
import multiprocessing as mp
import time
from collections import Counter
from scipy.interpolate import CubicSpline
import signal
from rollout_core import run_core_rollout, compute_step_budget
import subprocess
from settings import (
    SCORE_WEIGHTS,
    PATH_N_POINTS,
    USE_PLANNER,
    ROLLOUT_EPS,
    ROLLOUT_MAX_ITER,
    MAX_EVALS,
    N_HORIZON,
    DT
)

from vehicle_physics import (
    VehicleParams,
)
from bicycle_model import get_8state_discrete_model
import speed_profile as sp
import cma
from sim_track import (
    place_cones,
)
import datetime

# ==========================================
# TUNABLE WEIGHT CONFIGURATION
# ==========================================
# These index lists define WHICH diagonal entries of Q, R, R_rate are
# handed to CMA-ES as free parameters. Entries not listed are held fixed
# at their template values (e.g. Q[5,5]=0 stays zero — no cost on e_a).
TUNABLE_Q_IDX = [0, 1, 2, 3, 4]  # e_y, e_y_dot, e_psi, e_psi_dot, e_v
TUNABLE_R_IDX = [0, 1]  # delta_cmd, a_cmd
TUNABLE_R_RATE_IDX = [0, 1]  # d(delta_cmd)/dt, d(a_cmd)/dt

# Multiplicative bounds on each tunable weight (multiplier, not absolute value).
# Floor of 0.1 allows reduction below the template; ceiling of 10.0 prevents
# weights from becoming so large they dominate and mask tracking quality.
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

# Graceful shutdown flag: set by SIGINT handler; checked each CMA generation.
_stop_requested = False

# Build the flat bounds list in the same order as the parameter vector
# [Q_0, Q_1, Q_2, Q_3, Q_4, R_0, R_1, R_rate_0, R_rate_1]
bounds = []
for idx in TUNABLE_Q_IDX:
    bounds.append(Q_BOUNDS.get(idx))
for idx in TUNABLE_R_IDX:
    bounds.append(R_BOUNDS.get(idx))
for idx in TUNABLE_R_RATE_IDX:
    bounds.append(R_RATE_BOUNDS.get(idx))

# Sanity check: weights must sum to 1 so the composite score is interpretable
assert (
    abs(SCORE_WEIGHTS.sum() - 1.0) < 1e-9
), f"SCORE_WEIGHTS must sum to 1.0, got {SCORE_WEIGHTS.sum():.6f}"

# Module-level dict: shared initial parameters passed to worker processes.
# Set in the main process before the Pool is opened; each worker reads it
# from its own copy after fork via init_worker().
_init_context: dict = {}

# Cache for discretised linear models: keyed by rounded vx (1 decimal place).
# Populated in init_worker() at worker startup to amortise model computation
# across the many rollouts each worker runs.
_model_cache = {}


def get_cached_model(vx, dt):
    """
    Return the discrete-time (Ad, Bd) pair for a given speed from the
    module-level cache, computing and storing it on first access.

    Avoids repeated calls to get_8state_discrete_model() — which involves
    a matrix exponential — during the inner simulation loop. Speed is rounded
    to 1 decimal place so that speeds differing by < 0.05 m/s share the same
    cached model (acceptable linearisation error for the MPC horizon).

    Parameters
    ----------
    vx : float   Current longitudinal speed (m/s).
    dt : float   Discretisation timestep (s). Must be 0.05 s to match simulation.

    Returns
    -------
    (Ad, Bd) : tuple of np.ndarray, shapes (8,8) and (8,2)
        Discrete-time state and input matrices for use in solve_mpc().

    Called by: run_headless_rollout() (inner loop, every simulation step)
    """
    key = np.round(vx, 1)
    if key not in _model_cache:
        _model_cache[key] = get_8state_discrete_model(key, dt)
    return _model_cache[key]


# ==========================================
# SYNTHETIC PATH LIBRARY — INTERNAL HELPERS
# ==========================================


def _resample_path(waypoints_x, waypoints_y, n_points=PATH_N_POINTS):
    """
    Fit a clamped cubic spline through the given waypoints and resample to
    n_points uniformly-spaced points. Computes path heading, speed profile,
    and cone placement for the resulting dense path.

    Clamped boundary conditions (bc_type=((1, d0), (1, dN))) pin the spline
    derivative at each end to the direction of the first/last chord. This
    prevents the "not-a-knot" default from creating an upswing at path ends
    that would generate unrealistic curvature and corner speeds.

    Parameters
    ----------
    waypoints_x : array-like   Sparse X waypoints defining the path shape.
    waypoints_y : array-like   Sparse Y waypoints defining the path shape.
    n_points : int             Number of output points (default: PATH_N_POINTS=1000).

    Returns
    -------
    (path_X, path_Y, path_Psi, path_v, blue_all, yellow_all) : tuple
        path_X, path_Y : np.ndarray, shape (n_points,)   Resampled coordinates.
        path_Psi       : np.ndarray, shape (n_points,)   Path heading at each point (rad).
        path_v         : np.ndarray, shape (n_points,)   Smoothed speed profile (m/s).
        blue_all       : np.ndarray, shape (m, 2)        Left cone positions.
        yellow_all     : np.ndarray, shape (m, 2)        Right cone positions.

    Called by: build_synthetic_paths() — once at import time for each path
    """
    wx = np.asarray(waypoints_x, dtype=float)
    wy = np.asarray(waypoints_y, dtype=float)
    t = np.linspace(0.0, 1.0, len(wx))

    # Clamped end derivatives: direction of first/last chord
    d0 = np.array([wx[1] - wx[0], wy[1] - wy[0]]) / (t[1] - t[0])
    dN = np.array([wx[-1] - wx[-2], wy[-1] - wy[-2]]) / (t[-1] - t[-2])

    cs_x = CubicSpline(t, wx, bc_type=((1, d0[0]), (1, dN[0])))
    cs_y = CubicSpline(t, wy, bc_type=((1, d0[1]), (1, dN[1])))

    t_fine = np.linspace(0.0, 1.0, n_points)
    path_X = cs_x(t_fine)
    path_Y = cs_y(t_fine)
    # Path heading: atan2 of the spline derivative vector
    dx = cs_x.derivative()(t_fine)
    dy = cs_y.derivative()(t_fine)
    path_Psi = np.arctan2(dy, dx)

    # Curvature-based speed profile + smoothing
    raw_v = sp.compute_speed_profile(path_X, path_Y)
    path_v = sp.smooth_profile(raw_v, window=9)

    # Cone placement for SimPerception / SimPlanner
    blue_all, yellow_all = place_cones(path_X, path_Y)
    return path_X, path_Y, path_Psi, path_v, blue_all, yellow_all


def _make_arc(cx, cy, radius, theta_start_deg, theta_end_deg, n=20):
    """
    Generate (x, y) coordinates for a circular arc segment.

    Parameters
    ----------
    cx, cy : float        Centre of the circle (m).
    radius : float        Radius of the arc (m).
    theta_start_deg : float   Start angle measured from +X axis (degrees).
    theta_end_deg : float     End angle measured from +X axis (degrees).
    n : int               Number of points along the arc.

    Returns
    -------
    (x, y) : tuple of np.ndarray, shape (n,)
        Coordinates of the arc points.

    Called by: build_synthetic_paths() — used to construct corner geometry
    """
    angles = np.linspace(np.radians(theta_start_deg), np.radians(theta_end_deg), n)
    return cx + radius * np.cos(angles), cy + radius * np.sin(angles)


def vector_to_weights(vec, Q_template, R_template, R_rate_template):
    """
    Convert a CMA-ES parameter vector to (Q, R, R_rate) weight matrices.

    The parameter vector contains multiplicative scale factors for each tunable
    weight, ordered as [Q scales..., R scales..., R_rate scales...]. Each
    factor is applied to the corresponding diagonal element of the template:
        Q[i,i] = vec[j] * Q_template[i,i]   (or 1.0 if template entry is 0)

    Using multiplicative factors rather than absolute values keeps the search
    space dimensionally consistent and allows CMA-ES to reason about relative
    scaling regardless of the template's magnitude.

    Parameters
    ----------
    vec : array-like, shape (n_q + n_r + n_r_rate,)
        CMA-ES candidate parameter vector. Length = len(TUNABLE_Q_IDX) +
        len(TUNABLE_R_IDX) + len(TUNABLE_R_RATE_IDX) = 5 + 2 + 2 = 9.
    Q_template : np.ndarray, shape (8, 8)   Base Q matrix.
    R_template : np.ndarray, shape (2, 2)   Base R matrix.
    R_rate_template : np.ndarray, shape (2, 2)   Base R_rate matrix.

    Returns
    -------
    (Q, R, R_rate) : tuple of np.ndarray
        Weight matrices with tunable entries replaced by scaled values.
        Non-tunable entries are copied unchanged from templates.

    Called by: run_headless_rollout() (converts each CMA-ES candidate to weights),
               main block (converts best found vector to final weights for display)
    """
    Q = Q_template.copy()
    R = R_template.copy()
    R_rate = R_rate_template.copy()

    n_q = len(TUNABLE_Q_IDX)
    n_r = len(TUNABLE_R_IDX)

    for j, i in enumerate(TUNABLE_Q_IDX):
        # Use template value as base; substitute 1.0 if template entry is zero
        base_val = Q_template[i, i] if Q_template[i, i] != 0.0 else 1.0
        Q[i, i] = vec[j] * base_val

    for j, i in enumerate(TUNABLE_R_IDX):
        base_val = R_template[i, i] if R_template[i, i] != 0.0 else 1.0
        R[i, i] = vec[n_q + j] * base_val

    for j, i in enumerate(TUNABLE_R_RATE_IDX):
        base_val = R_rate_template[i, i] if R_rate_template[i, i] != 0.0 else 1.0
        R_rate[i, i] = vec[n_q + n_r + j] * base_val

    return Q, R, R_rate


def build_synthetic_paths():
    """
    Build the full synthetic path library and return it as a name→tuple dict.

    All paths are scaled to Formula Student track geometry:
      - Straight segments: 5-10 m
      - Corner radii: 5-12 m (except skidpad at R=9.125 m per FS spec)
      - No excess straight padding: paths start and end at the feature being tested

    Each path is resampled to n_points=MAX_EVALS dense points with:
      - A clamped cubic spline (smooth, no end-point artefacts)
      - A curvature-based speed profile (from speed_profile.py)
      - Cone placement (from sim_track.place_cones)

    Returns
    -------
    paths : dict
        Keys: path name strings (e.g. "PATH_SUDDEN_TURN").
        Values: (path_X, path_Y, path_Psi, path_v, blue_all, yellow_all) tuples.

    Called at module import time: SYNTHETIC_PATHS = build_synthetic_paths()
    Used by: run_headless_rollout() (looks up path by name),
             simulation.py (load_test_path cycles through PATH_NAMES)
    """
    paths = {}

    # --- PATH_SUDDEN_TURN ---
    # Long straight then sharp 90° left (R=6 m). Tests late-apex cornering response:
    # the vehicle must slow and turn simultaneously from a straight-line approach.
    s1x = np.linspace(-55, 5, 10)
    s1y = np.zeros(10)
    arc_x, arc_y = _make_arc(5, 4.5, 4.5, -90, 0, n=20)
    s2x = np.full(10, 9.5)
    s2y = np.linspace(4.5, 24, 10)
    wx = np.concatenate([s1x, arc_x[1:], s2x[1:]])
    wy = np.concatenate([s1y, arc_y[1:], s2y[1:]])
    paths["PATH_SUDDEN_TURN"] = _resample_path(wx, wy)

    # --- PATH_S_BEND ---
    # Long straight (East) → right 90° (R=10 m) → 5 m link (South) →
    # left 90° (R=10 m) → 10 m exit (East).
    # Tests consecutive direction changes and weight transfer between corners.
    s0x = np.linspace(10, 20, 10)
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
    # Two tangent circles R=9.125 m (FS skidpad centreline specification) with
    # 5 m entry/exit straight. Tests sustained constant-radius cornering at speed —
    # the most common FS dynamic event and the hardest for the linear model.
    # R_skid = 9.125
    # s0x    = np.zeros(8)
    # s0y    = np.linspace(-5, 0, 8)
    # # Right circle: clockwise, centre at (R_skid, 0)
    # arc1x, arc1y = _make_arc(R_skid, 0, R_skid, 180, -180, n=60)
    # # Left circle: counter-clockwise, centre at (-R_skid, 0)
    # arc2x, arc2y = _make_arc(-R_skid, 0, R_skid, 0, 360, n=60)
    # s1x    = np.zeros(8)
    # s1y    = np.linspace(0, 5, 8)
    # wx     = np.concatenate([s0x, arc1x[1:], arc2x[1:], s1x[1:]])
    # wy     = np.concatenate([s0y, arc1y[1:], arc2y[1:], s1y[1:]])
    # paths["PATH_SKIDPAD"] = _resample_path(wx, wy)

    # --- PATH_SPIRAL ---
    # Continuously tightening clothoid (Euler spiral): curvature increases linearly
    # from κ=1/15 m⁻¹ (R=15 m) to κ=1/5.5 m⁻¹ (R=5.5 m) over 60 m arc length.
    # Tests progressive speed reduction and gradual steering increase. The clothoid
    # is integrated numerically from the curvature profile:
    #   ψ(s) = -∫₀ˢ κ(t) dt  (accumulated heading change)
    #   x(s) = ∫₀ˢ cos(ψ) dt, y(s) = ∫₀ˢ sin(ψ) dt
    s0x = np.linspace(0, 5, 10)
    s0y = np.zeros(10)
    L_spiral = 60.0
    ds_spiral = 0.1
    s_spiral = np.arange(0, L_spiral + ds_spiral, ds_spiral)
    # Linearly varying curvature: κ = κ_start + (κ_end - κ_start) * s/L
    kappa_spiral = (1.0 / 15.0) + ((1.0 / 5.5) - (1.0 / 15.0)) * (s_spiral / L_spiral)
    psi_spiral = -np.cumsum(kappa_spiral * ds_spiral)  # Integrated heading (clockwise)
    psi_spiral = np.concatenate([[0.0], psi_spiral[:-1]])  # Shift to start at ψ=0
    curve_sx = 5.0 + np.cumsum(np.cos(psi_spiral) * ds_spiral)
    curve_sy = 0.0 + np.cumsum(np.sin(psi_spiral) * ds_spiral)
    curve_sx = np.concatenate([[5.0], curve_sx])
    curve_sy = np.concatenate([[0.0], curve_sy])
    wx = np.concatenate([s0x, curve_sx[1:]])
    wy = np.concatenate([s0y, curve_sy[1:]])
    paths["PATH_SPIRAL"] = _resample_path(wx, wy)

    # --- PATH_MICRO_SLALOM ---
    # 7 gates at 7.5 m spacing, ±1.2 m lateral amplitude.
    # Tests rapid alternating direction changes at close spacing.
    wx = np.linspace(0, 45, 7)
    wy = [0, 1.2, -1.2, 1.2, -1.2, 1.2, 0]
    paths["PATH_MICRO_SLALOM"] = _resample_path(wx, wy)

    # --- PATH_OFFSET_CHICANE ---
    # Lateral gate offsets of ±2.0 m at 10 m intervals.
    # Similar to slalom but with cleaner step-input geometry.
    wx = [-20, 5, 15, 25, 35, 45, 50]
    wy = [0, 0, 2.0, -2.0, 2.0, 0, 0]
    paths["PATH_OFFSET_CHICANE"] = _resample_path(wx, wy)

    # --- PATH_ACCELERATION---
    # Straight path for acceleration testing
    wx = np.linspace(0, 75, 50)
    wy = np.zeros(50)
    paths["PATH_ACCELERATION"] = _resample_path(wx, wy)

    # --- PATH_HAIRPIN ---
    # 5 m entry → 180° turn (R=5 m, the tightest FS-legal corner) → 5 m exit.
    # Tests maximum steering demand and slowest-speed tracking.
    s1x = np.linspace(0, 5, 10)
    s1y = np.zeros(10)
    arc_hp_x, arc_hp_y = _make_arc(5, -5, 5, 90, -90, n=25)
    s2x = np.linspace(5, 0, 10)
    s2y = np.full(10, -10.0)
    wx = np.concatenate([s1x, arc_hp_x[1:], s2x[1:]])
    wy = np.concatenate([s1y, arc_hp_y[1:], s2y[1:]])
    paths["PATH_HAIRPIN"] = _resample_path(wx, wy)

    # --- PATH_CHICANE ---
    # Two matched-radius (R=6 m) arcs forming a pure S-transition without a
    # straight link between them. Tests the controller's ability to reverse
    # lateral error sign while still in a corner.
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
    # Classic single 90° right-hand corner (R=6 m) with 5 m approach/exit.
    # Symmetric to PATH_SUDDEN_TURN but turning right; tests directional parity.
    s1x = np.linspace(0, 5, 10)
    s1y = np.zeros(10)
    arc_fsc_x, arc_fsc_y = _make_arc(5, -6, 6, 90, 0, n=20)
    s2x = np.full(10, 11.0)
    s2y = np.linspace(-6, -11, 10)
    wx = np.concatenate([s1x, arc_fsc_x[1:], s2x[1:]])
    wy = np.concatenate([s1y, arc_fsc_y[1:], s2y[1:]])
    paths["PATH_FS_CORNER"] = _resample_path(wx, wy)

    # --- PATH_MIXED ---
    # Compound sequence: 5 m straight → left 90° (R=6 m) → 5 m N link →
    # right 90° (R=6 m) → 5 m E link → 180° hairpin (R=5 m) → 5 m W exit.
    # Tests generalisation: the controller must handle all corner types in sequence.
    s_mix0x = np.linspace(-25, 5, 10)
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
    wx = np.concatenate(
        [
            s_mix0x,
            arc_m1x[1:],
            l_m1x[1:],
            arc_m2x[1:],
            l_m2x[1:],
            arc_m3x[1:],
            s_mix1x[1:],
        ]
    )
    wy = np.concatenate(
        [
            s_mix0y,
            arc_m1y[1:],
            l_m1y[1:],
            arc_m2y[1:],
            l_m2y[1:],
            arc_m3y[1:],
            s_mix1y[1:],
        ]
    )
    paths["PATH_MIXED"] = _resample_path(wx, wy)

    return paths


# Build the path library once at import time so all workers (forked from
# this process) share the same pre-computed data without re-running the splines.
SYNTHETIC_PATHS = build_synthetic_paths()
PATH_LENGTHS = {
    name: np.sum(np.hypot(np.diff(x), np.diff(y)))
    for name, (x, y, _, _, _, _) in SYNTHETIC_PATHS.items()
}
PATH_NAMES = list(SYNTHETIC_PATHS.keys())


# ==========================================
# TRACKING ERROR HELPERS
# ==========================================
def _normalize_angle(angle):
    """
    Wrap an angle to the range (−π, π] using atan2.

    Parameters
    ----------
    angle : float   Angle in radians (any value).
    Returns
    -------
    float : Equivalent angle in (−π, π].
    """
    return np.arctan2(np.sin(angle), np.cos(angle))


# ==========================================
# WORKER INITIALIZER
# ==========================================
def init_worker(Q_init, R_init, R_rate_init):
    """
    Initialise each worker process in the multiprocessing Pool.

    Called once per worker at Pool creation time. Pre-computes and caches
    the linear vehicle models for all expected speed values so the inner
    rollout loop never waits on a matrix exponential.

    Also ignores SIGINT in worker processes: Ctrl+C is handled in the main
    process only (via _handle_sigint), preventing workers from being killed
    mid-rollout which would leave the pool in an inconsistent state.

    Parameters
    ----------
    Q_init : np.ndarray, shape (8,8)   Template Q matrix from main process.
    R_init : np.ndarray, shape (2,2)   Template R matrix from main process.
    R_rate_init : np.ndarray, shape (2,2)   Template R_rate matrix from main process.

    Called by: mp.Pool(initializer=init_worker, initargs=(...))
    """
    signal.signal(signal.SIGINT, signal.SIG_IGN)  # Workers ignore Ctrl+C

    np.random.seed()  # Re-seed RNG in each worker (fork inherits parent's state)

    global _init_context, _model_cache
    _init_context["Q"] = Q_init
    _init_context["R"] = R_init
    _init_context["R_rate"] = R_rate_init
    _init_context["vehicle_params"] = VehicleParams()

    # Pre-cache all models from 0.5 m/s to 20.0 m/s in 0.1 m/s steps
    # Range covers vx clamp floor (0.5) through V_MAX (20.0) to avoid
    # mid-rollout cache misses during startup and high-speed phases.
    for vx in np.arange(0.5, 20.1, 0.1):
        key = np.round(vx, 1)
        if key not in _model_cache:
            _model_cache[key] = get_8state_discrete_model(key, DT)


# ==========================================
# HEADLESS SIMULATION ROLLOUT
# ==========================================

def run_headless_rollout(
    weights_vector,
    path_name=None,
    num_steps=350,
    ey0=0.0,
    epsi0=0.0,
    use_planner=USE_PLANNER,
):
    """
    Run a single closed-loop simulation rollout without graphics and return
    a scalar performance score.

    This is the innermost function that CMA-ES's objective evaluates.
    It mirrors simulate_closed_loop() from simulation.py but without any
    matplotlib rendering, with looser solver tolerances (ROLLOUT_EPS) for
    speed, and with deterministic initial conditions (no rng jitter, unlike
    the live simulator's noise option).

    ROLLOUT PIPELINE (per step):
      1.  Get visible cones from SimPerception (FOV filter)
      2.  Update SimPlanner (cone accumulation → centreline + speed profile)
      3.  Compute tracking errors
      4.  Track progress along the reference path (for completion scoring)
      5.  Build MPC state vector x0_mpc from tracking errors + plant state
      6.  Apply adaptive gain scaling (curvature and speed dependent)
      7.  Solve MPC (warm-start on all steps except first)
      8.  Accumulate performance metrics
      9.  Check DNF conditions (off-track, solver failures, no progress)
     10.  Step the nonlinear plant

    SCORING (after rollout):
      Metrics → RMSE normalisation → weighted dot product with SCORE_WEIGHTS
      → subtract completion + time bonuses → add DNF penalty if applicable
      → multiply by inaccuracy factor if solver quality was poor

    Parameters
    ----------
    weights_vector : array-like, shape (9,)
        CMA-ES candidate: multiplicative scale factors for the 9 tunable weights
        [Q_0..Q_4, R_0, R_1, R_rate_0, R_rate_1]. Converted to (Q, R, R_rate)
        by vector_to_weights().
    path_name : str
        Name of the synthetic path to use (must be a key in SYNTHETIC_PATHS).
        Raises ValueError if None.
    num_steps : int
        Initial step budget (overridden by calculate_dynamic_max_steps).
    ey0 : float
        Initial lateral offset from path start (m). Used to test robustness
        to imperfect initial positioning.
    epsi0 : float
        Initial heading offset from path tangent (rad).
    use_planner : bool, optional
        If True, use SimPerception + SimPlanner for errors/speed profile,
        matching the real ROS2 pipeline. If False (default), use the true
        reference path directly for faster, noise-free tuning rollouts.

    Returns
    -------
    score : float
        Composite performance score (lower is better). Typical range:
          Good finish:  Around -0.5
          Poor finish:   -0.1 to 1
          DNF:           >= 1 (depending on how early the DNF)

    Called by: _score_task() (from pool.map in parallel_evaluate_candidate),
               evaluate_candidate() (serial fallback)
    """
    Q_init = _init_context["Q"]
    R_init = _init_context["R"]
    R_rate_init = _init_context["R_rate"]

    Q, R, R_rate = vector_to_weights(weights_vector, Q_init, R_init, R_rate_init)

    p = _init_context["vehicle_params"]

    u_min = np.array([-p.max_steer, p.max_accel_brake])
    u_max = np.array([p.max_steer, p.max_accel])

    if path_name is None:
        raise ValueError("path_name must be provided")

    path_X, path_Y, path_Psi, path_v, blue_all, yellow_all = SYNTHETIC_PATHS[path_name]

    dynamic_max_steps, num_steps = compute_step_budget(path_X, path_Y, path_v)

    rollout = run_core_rollout(
        path_X, path_Y, path_Psi, path_v, blue_all, yellow_all,
        Q, R, R_rate, u_min, u_max, p,
        ey0=ey0, epsi0=epsi0,
        max_steps=num_steps, dynamic_max_steps=dynamic_max_steps,
        use_planner=use_planner, model_lookup=get_cached_model,
        n_horizon=N_HORIZON, eps=ROLLOUT_EPS, max_iter=ROLLOUT_MAX_ITER,
        want_history=False,
    )
    return rollout["composite_score"]


# ==========================================
# OBJECTIVE FUNCTION WRAPPERS
# ==========================================
def evaluate_all_paths(weights_vector, n_repeats=3, ey0=0.0, epsi0=0.0):
    """
    Evaluate a weights vector across every path in PATH_NAMES (not just
    VALIDATION_SUITE), repeated n_repeats times, and return the mean
    composite score and per-path breakdown.

    Intended for post-tuning benchmarking from performance_stats.py.
    Must be called after init_worker() has populated _init_context, or
    after manually setting _init_context in the calling process.

    Initial conditions to be used for path testing can be specified as well.

    Parameters
    ----------
    weights_vector : array-like, shape (9,)
        CMA-ES parameter vector (multiplicative scale factors).
    n_repeats : int
        Number of independent rollouts per path (scores are averaged).

    Returns
    -------
    dict with keys:
        'mean_score'    : float  — mean composite score across all paths × repeats
        'per_path'      : dict   — {path_name: mean_score} for each path
        'all_scores'    : list   — flat list of every individual rollout score
    """
    per_path = {}
    all_scores = []

    for path_name in PATH_NAMES:
        path_scores = []
        for _ in range(n_repeats):
            path_scores.append(
                run_headless_rollout(
                    weights_vector, path_name=path_name, ey0=ey0, epsi0=epsi0
                )
            )

        per_path[path_name] = float(np.mean(path_scores))
        all_scores.extend(path_scores)

    return {
        "mean_score": float(np.mean(all_scores)),
        "per_path": per_path,
        "all_scores": all_scores,
    }


# Active validation suite: subset of paths used for CMA-ES evaluation.
# Commented-out paths are available but excluded to balance coverage vs. speed.
VALIDATION_SUITE = [
    # "PATH_OFFSET_CHICANE",
    "PATH_SPIRAL",
    "PATH_SUDDEN_TURN",
    # "PATH_SKIDPAD",
    # "PATH_S_BEND",
    # "PATH_MIXED",
    "PATH_HAIRPIN",
    # "PATH_CHICANE",
    "PATH_FS_CORNER",
    "PATH_MICRO_SLALOM",
    # "PATH_ACCELERATION"
]

# Initial condition perturbations tested for each path.
# (ey0, epsi0): lateral offset (m), heading offset (rad).
INITIAL_CONDITIONS = [
    (0.00, 0.00),  # Nominal: start exactly on path
    (0.2, 0.05),  # Perturbed: slight lateral/heading offset
]


def _build_task_table(suite, ics):
    """
    Build the flat evaluation task list from the validation suite × ICs.

    Returns a list of (path_name, ey0, epsi0) tuples and a corresponding
    weight array (all 1.0 for unique combinations; Counter handles duplicates
    if any path/IC appears twice in suite or ics).

    Called at module import time: EVAL_TASKS, EVAL_WEIGHTS = _build_task_table(...)
    """
    counts = Counter((p, ey0, epsi0) for p in suite for (ey0, epsi0) in ics)
    tasks = list(counts.keys())
    weights = np.array([counts[t] for t in tasks], dtype=float)
    return tasks, weights


EVAL_TASKS, EVAL_WEIGHTS = _build_task_table(VALIDATION_SUITE, INITIAL_CONDITIONS)


def _aggregate_task_scores(task_scores):
    """
    Combine per-task scores into a single objective value.

    Uses a 70/30 blend of weighted mean and worst-case score:
        objective = 0.7 * weighted_mean + 0.3 * max(scores)

    The worst-case term (30%) prevents CMA-ES from finding weights that perform
    well on average but catastrophically fail on one path type — a real risk
    when the validation suite has diverse track geometries.

    Parameters
    ----------
    task_scores : list of float   Score for each task in EVAL_TASKS.

    Returns
    -------
    float : Combined objective (lower is better).

    Called by: evaluate_candidate(), parallel_evaluate_candidate()
    """
    s = np.asarray(task_scores, dtype=float)
    weighted_mean = float(np.sum(EVAL_WEIGHTS * s) / np.sum(EVAL_WEIGHTS))
    worst = float(np.max(s))
    return 0.7 * weighted_mean + 0.3 * worst


def _score_task(args):
    """
    Unpack a (vec, path_name, ey0, epsi0) tuple and call run_headless_rollout().
    This wrapper exists because pool.map() only accepts a single iterable argument.

    Called by: pool.map() inside parallel_evaluate_candidate()
    """
    vec, path_name, ey0, epsi0 = args
    return run_headless_rollout(vec, path_name=path_name, ey0=ey0, epsi0=epsi0)


def evaluate_candidate(vec):
    """
    Evaluate a single CMA-ES candidate serially across all EVAL_TASKS.

    Used as a fallback when the process pool is unavailable, and for the
    post-optimisation comparison step (xbest vs. xfavorite) where we want
    a clean, pool-independent evaluation.

    Parameters
    ----------
    vec : array-like, shape (9,)   CMA-ES parameter vector.

    Returns
    -------
    float : Aggregated objective value.

    Called by: main block (post-optimisation comparison),
               parallel_evaluate_candidate() (fallback when pool is None)
    """
    scores = [
        run_headless_rollout(vec, path_name=p, ey0=ey0, epsi0=epsi0)
        for (p, ey0, epsi0) in EVAL_TASKS
    ]
    return float(_aggregate_task_scores(scores))


# ==========================================
# PARALLEL OBJECTIVE
# ==========================================

# Pool handle injected into the module before the surrogate starts.
# fmin_lq_surr2 calls parallel_evaluate_candidate serially per candidate;
# each call parallelises across tasks internally via this pool.
_eval_pool = None
_n_tasks = None


def parallel_evaluate_candidate(vec):
    """
    Objective function passed to fmin_lq_surr2 (CMA-ES surrogate).

    Evaluates all EVAL_TASKS for a single candidate vector in parallel using
    _eval_pool. Each task is one path × initial condition rollout. Results are
    aggregated via _aggregate_task_scores (70% mean + 30% worst-case).

    Falls back to serial evaluation (evaluate_candidate) if _eval_pool is None.

    Parameters
    ----------
    vec : array-like, shape (9,)   CMA-ES candidate parameter vector.

    Returns
    -------
    float : Aggregated objective value (lower is better).

    Called by: cma.fmin_lq_surr2() — the surrogate-assisted CMA-ES optimiser
    """
    if _eval_pool is None:
        return evaluate_candidate(vec)

    flat_tasks = [(vec, p, ey0, epsi0) for (p, ey0, epsi0) in EVAL_TASKS]
    chunksize = max(1, len(flat_tasks) // (_n_tasks * 4))
    task_scores = _eval_pool.map(_score_task, flat_tasks, chunksize=chunksize)
    return float(_aggregate_task_scores(task_scores))


# ==========================================
# LOGGING
# ==========================================
def get_git_revision_hash():
    """
    Retrieve the current git commit hash for logging alongside tuning results.

    Including the commit hash in the log allows a tuned weight set to be
    traced back to the exact code version it was generated with, which is
    important when the model or scoring function changes between runs.

    Returns
    -------
    str : 40-character hex commit hash, or a fallback string if git is
          unavailable (not a repository, or git not installed).

    Called by: log_results_to_history()
    """
    try:
        return (
            subprocess.check_output(["git", "rev-parse", "HEAD"])
            .decode("ascii")
            .strip()
        )
    except (subprocess.CalledProcessError, FileNotFoundError):
        return "Unknown (not a git repository)"


def log_results_to_history(Q, R, R_rate, duration, score):
    """
    Append the best-found weight matrices and metadata to tuning history.txt.

    The log file provides a persistent record of all tuning runs. Each entry
    includes a timestamp, the weight diagonals (copy-pasteable into simulation.py),
    the run duration, and the git commit hash so results can be reproduced.
    The "Score" field is left as a placeholder — it is filled in manually after
    the weights have been tested in FSDS (the real simulator), since the offline
    score does not perfectly correlate with FSDS performance.

    Parameters
    ----------
    Q : np.ndarray, shape (8,8)       Best-found Q matrix.
    R : np.ndarray, shape (2,2)       Best-found R matrix.
    R_rate : np.ndarray, shape (2,2)  Best-found R_rate matrix.
    duration : float                  Total optimisation wall time (seconds).

    Called by: main block (after optimisation completes or is interrupted)
    """
    timestamp = datetime.datetime.now().strftime("%d/%m/%y %H:%M")
    commit_hash = get_git_revision_hash()
    with open("tuning history.txt", "a") as f:
        f.write(f"\n# {timestamp} - [Pending Description: yet to be tested]\n")
        f.write(f"Q_diag      = {np.diag(Q).tolist()}\n")
        f.write(f"R_diag      = {np.diag(R).tolist()}\n")
        f.write(f"R_rate_diag = {np.diag(R_rate).tolist()}\n")
        f.write(f"Duration    = {duration / 60:.2f} minutes\n")
        f.write(
            f"Overall score (avged from all testing scenarios)  = Haven't been tested.\n"
        )
        f.write(f"Tuner score (tuning scenarios / validation suite) = {score}\n")
        f.write(f"Commit hash = {commit_hash}\n")


# ==========================================
# TEMPLATE WEIGHT MATRICES
# ==========================================
# These define the search space centre. CMA-ES multiplies each diagonal entry
# by a factor from bounds (0.1-10.0). Setting all to 1.0 starts the search
# at the midpoint of the multiplicative range on a log scale.
Q = np.diag([1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0])
R = np.diag([1.0, 1.0])
R_rate = np.diag([1.0, 1.0])

# ==========================================
# MAIN EXECUTION
# ==========================================
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

    # Start at geometric (log-scale) midpoint: sqrt(lower * upper) = 1.0 for [0.1, 10.0].
    # This is the correct neutral point for a multiplicative search space — it means
    # "start with the template weights unscaled."
    x0 = np.sqrt(lower * upper)

    # sigma0: initial CMA-ES step size.
    # Too small → slow exploration (stagnates in local minimum near x0).
    # Too large → poor exploitation (misses fine structure near optima).
    sigma0 = 0.65
    # Per-dimension std scaled to log-space range: ln(upper/lower)
    log_ranges = np.log(upper / lower)
    cma_stds = 0.23 * log_ranges

    # Default population size: pycma heuristic 5 + 3*ln(n) ≈ 12 for n=9 params
    default_popsize = int(5 + np.floor(3 * np.log(num_params)))
    popsize = default_popsize

    max_evals = MAX_EVALS
    max_restarts = 7  # BIPOP restart budget
    num_cores = max(1, mp.cpu_count() - 1)  # Leave one core for the OS

    print("\n[Offline Tuner] Strategy: BIPOP + lq-CMA-ES (surrogate-assisted)")
    print(f"  Parameters:    {num_params}")
    print(f"  x0 (midpoint): {np.round(x0, 2).tolist()}")
    print(
        f"  sigma0:        {sigma0}  |  per-dim stds: {np.round(cma_stds, 3).tolist()}"
    )
    print(f"  Base popsize:  {popsize}  |  max restarts: {max_restarts}")
    print(
        f"  True-eval budget: {max_evals}  (surrogate reduces actual rollouts ~3-10x)"
    )
    print(f"  Eval tasks/candidate: {len(EVAL_TASKS)}")
    print(f"  Workers: {num_cores}")

    # ── CMA-ES configuration ────────────────────────────────────────────────────
    # CMA_active=True: active negative covariance update — improves convergence on
    # multi-modal landscapes by learning which directions to avoid, not just which
    # to prefer. Recommended for n > 10 but beneficial here too.
    # tolconditioncov=1e14: allows the covariance matrix to become moderately
    # ill-conditioned before terminating (avoids premature stops in flat regions).
    cma_options = {
        "bounds": [lower.tolist(), upper.tolist()],
        "CMA_stds": cma_stds,
        "popsize": popsize,
        "seed": 42,  # Fixed seed for reproducibility
        "verb_disp": False,
        "verb_log": 0,
        "verbose": -9,
        "CMA_active": True,
        "tolconditioncov": 1e14,
        "maxfevals": max_evals,
    }

    start_time = time.time()

    # ── Process pool + surrogate launch ─────────────────────────────────────────
    # Workers are pre-initialised with the template weights and the full model
    # cache so they start fast. The pool handle is injected into the module-level
    # parallel_evaluate_candidate() via _eval_pool so pycma's interface (which
    # only accepts a scalar function) doesn't need modification.
    with mp.Pool(
        processes=num_cores,
        initializer=init_worker,
        initargs=(Q_init, R_init, R_rate_init),
    ) as pool:

        _eval_pool = pool
        _n_tasks = num_cores

        # Also initialise the main process context (used for post-opt evaluation)
        _init_context["Q"] = Q_init
        _init_context["R"] = R_init
        _init_context["R_rate"] = R_rate_init
        _init_context["vehicle_params"] = VehicleParams()

        # 196 steps ensures exactly 0.1 increments between 0.5 and 20.0 inclusive
        for vx in np.linspace(0.5, 20.0, 196):
            key = np.round(vx, 1)
            if key not in _model_cache:
                _model_cache[key] = get_8state_discrete_model(key, DT)

        def _handle_sigint(sig, frame):
            """
            SIGINT (Ctrl+C) handler for the main process.
            Sets _stop_requested, which _log_callback() checks each generation.
            The current generation completes before stopping, ensuring the best
            found solution is available for post-processing.
            """
            global _stop_requested
            if not _stop_requested:
                print(
                    "\n[Tuner] Ctrl+C caught — finishing current generation then stopping..."
                )
                _stop_requested = True

        signal.signal(signal.SIGINT, _handle_sigint)

        generation_log = []

        def _log_callback(es):
            """
            Per-generation callback passed to fmin_lq_surr2.
            Logs the best score and sigma each generation, and checks the
            _stop_requested flag to enable graceful shutdown.

            Parameters
            ----------
            es : cma.CMAEvolutionStrategy   The active CMA-ES instance.
            """
            gen_best = es.best.f if es.best.f is not None else float("inf")
            generation_log.append(
                {
                    "gen": es.countiter,
                    "evals": es.countevals,
                    "best": gen_best,
                    "sigma": es.sigma,
                }
            )
            running_best = min(e["best"] for e in generation_log)
            print(
                f"[lq-CMA-ES] gen {es.countiter:4d} | "
                f"true_evals {es.countevals:5d} | "
                f"gen_best {gen_best:.4f} | "
                f"overall_best {running_best:.4f} | "
                f"sigma {es.sigma:.4e}"
            )
            if _stop_requested:
                es.opts["maxfevals"] = 0  # Signal pycma to stop after this gen

        # ── fmin_lq_surr2 — BIPOP + quadratic surrogate ─────────────────────────
        # incpopsize=2:  large restarts double population (IPOP-CMA-ES schedule)
        # inject=True:   re-inject surrogate's predicted optimum each generation
        #                (keeps the surrogate's prediction honest against true evals)
        # keep_model=False: discard surrogate at each restart (avoids stale fits
        #                   that accumulated data from the previous regime biasing
        #                   the new population's predictions)
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

    _eval_pool = None  # Pool is now closed; remove reference

    end_time = time.time()

    # ── Post-optimisation: pick the better of xbest and xfavorite ───────────────
    # xbest:     the single best candidate observed across all evaluations
    # xfavorite: the distribution mean (more robust average of recent good candidates)
    # Fresh serial evaluation of both avoids noise from the parallel pool.
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
    print(
        f"Selected:              {'xbest' if score_best <= score_mean else 'xfavorite'}"
    )
    print("=" * 60)
    print("\nReplace your simulation.py weights with:")
    print("Q_diag      =", np.diag(best_Q).tolist())
    print("R_diag      =", np.diag(best_R).tolist())
    print("R_rate_diag =", np.diag(best_R_rate).tolist())
    print("=" * 60)

    # ── Generation log summary ───────────────────────────────────────────────────
    if generation_log:
        evals_arr = [e["evals"] for e in generation_log]
        best_arr = [e["best"] for e in generation_log]
        running_best = np.minimum.accumulate(best_arr)
        print("\nImprovement milestones (true-eval count → score):")
        last_reported = None
        for ev, rb in zip(evals_arr, running_best):
            if last_reported is None or rb < last_reported * 0.99:
                print(f"  evals={ev:5d}  →  {rb:.5f}")
                last_reported = rb

    try:
        print("\n" + "=" * 60)
        print("Optimization finished or interrupted. Saving results...")
        duration = end_time - start_time
        log_results_to_history(best_Q, best_R, best_R_rate, duration, score_best)
        print("Results successfully appended to tuning history.txt")
        print("=" * 60)
    except KeyboardInterrupt:
        print("\n[!] Optimization interrupted by user. Saving best-found state...")
        exit()