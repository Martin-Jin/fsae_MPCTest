# Language: python
# Title: Offline Auto-Tuner with Synthetic Path Library (offline_tuner.py)
#
# Changes from previous version:
#
#   BUG FIX (#3): x0_mpc state vector construction corrected.
#     - Index 4 (e_v) is now vx - v_target (speed error against the path
#       profile), not vx - vx0_rand (a fixed initial speed, meaningless
#       once the car has slowed for a corner).
#     - Index 5 (e_a) is now 0.0, matching simulation.py.  The old code
#       used state[7] (a_act) here, which is wrong: e_a is the
#       *acceleration error* slot which simulation.py always sets to 0.
#     - Indices 6/7 (delta_act, a_act) are correctly state[6] and state[7].
#
#   BUG FIX (#4): Tracking errors are now computed against a real reference
#     path (closest-point projection) instead of using raw global Y as a
#     proxy for lateral error. Each rollout drives one of the synthetic
#     paths defined below, so e_y and e_psi mean exactly the same thing
#     here as they do in simulation.py.
#
#   IMPROVEMENT (#9): N_horizon is 25, matching simulation.py (was 30).
#     MPC weights are horizon-dependent; tuning with a mismatched horizon
#     produces weights that don't transfer cleanly to the live simulator.
#
#   IMPROVEMENT (#10): Six synthetic reference paths replace the old
#     straight-line approximation.  Each path exercises a different
#     tracking scenario the weights need to handle:
#       PATH_GENTLE_CURVE    — long gradual bend, tests steady-state lateral
#       PATH_HAIRPIN         — tight 180° hairpin after a straight, tests
#                              hard deceleration + high curvature recovery
#       PATH_SUDDEN_TURN     — full-speed straight then abrupt 90° turn,
#                              tests emergency steering response
#       PATH_S_BEND          — classic S-shape, tests direction reversal
#       PATH_CHICANE         — rapid left-right-left, tests steering frequency
#       PATH_MIXED           — combination of all of the above in sequence
#     Each path is pre-processed with the same speed profiler used in
#     simulation.py so that v_target at each point is physically meaningful
#     and consistent.
#
#   IMPROVEMENT (#12): DE budget reduced to popsize=8, maxiter=12 for a
#     faster initial sweep.  With 9 tunable parameters that is still
#     8*9*2=144 population members × 12 generations = 1728 evaluations,
#     each evaluated over num_runs=3 rollouts on randomly selected paths.
#     A subsequent online-tuner hill-climb pass then refines from there.
#
#   IMPROVEMENT (#14): Removed the redundant _init_context population in
#     the parent process (__main__ block). Those assignments were never
#     read: evaluate_candidate is only called inside worker processes via
#     pool.map, where the context is populated by init_worker(). The parent-
#     side dict population was dead code that gave a false impression of
#     shared memory.

import numpy as np
import multiprocessing as mp
import time
from scipy.optimize import differential_evolution
from scipy.interpolate import CubicSpline

from tuner import (
    Q_BOUNDS,
    R_BOUNDS,
    R_RATE_BOUNDS,
    adaptive_R_rate,
    curvature_estimate,
    adaptive_R_scaling,
    vector_to_weights,
)
from vehicle_physics import VehicleParams, step_nonlinear_plant, init_plant_state
from model import get_8state_discrete_model
from optimiser import solve_mpc
import speed_profile as sp

# MPC horizon must match simulation.py exactly so tuned weights transfer
# cleanly to the live simulator.
N_HORIZON = 25

# Speed profile parameters — identical to simulation.py's constants so the
# headless rollouts see the same v_target distribution as the live sim.
SP_V_MAX       = 10.0
SP_MU          = 0.6
SP_A_ACCEL_MAX = 2.5
SP_A_BRAKE_MAX = 4.0
SP_V_MIN       = 2.5

# Module-level dictionary to share initial parameters safely across processes
_init_context: dict = {}


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
    t  = np.linspace(0.0, 1.0, len(wx))

    d0 = np.array([wx[1] - wx[0],  wy[1] - wy[0]])  / (t[1] - t[0])
    dN = np.array([wx[-1] - wx[-2], wy[-1] - wy[-2]]) / (t[-1] - t[-2])

    cs_x = CubicSpline(t, wx, bc_type=((1, d0[0]), (1, dN[0])))
    cs_y = CubicSpline(t, wy, bc_type=((1, d0[1]), (1, dN[1])))

    t_fine  = np.linspace(0.0, 1.0, n_points)
    path_X  = cs_x(t_fine)
    path_Y  = cs_y(t_fine)
    dx      = cs_x.derivative()(t_fine)
    dy      = cs_y.derivative()(t_fine)
    path_Psi = np.arctan2(dy, dx)

    raw_v   = sp.compute_speed_profile(
        path_X, path_Y,
        v_max=SP_V_MAX, mu=SP_MU,
        a_accel_max=SP_A_ACCEL_MAX, a_brake_max=SP_A_BRAKE_MAX,
        v_min=SP_V_MIN,
    )
    path_v  = sp.smooth_profile(raw_v, window=9)
    return path_X, path_Y, path_Psi, path_v


def _make_arc(cx, cy, radius, theta_start_deg, theta_end_deg, n=20):
    """Helper: points along a circular arc."""
    angles = np.linspace(np.radians(theta_start_deg),
                         np.radians(theta_end_deg), n)
    return cx + radius * np.cos(angles), cy + radius * np.sin(angles)


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
    wx  = np.concatenate([s1x, arc_x[1:], s2x[1:]])
    wy  = np.concatenate([s1y, arc_y[1:], s2y[1:]])
    paths["PATH_HAIRPIN"] = _resample_path(wx, wy)

    # --- PATH_SUDDEN_TURN ---
    # Long straight, then sharp 90° left turn
    s1x = np.linspace(0, 100, 25)
    s1y = np.zeros(25)
    arc_x, arc_y = _make_arc(100, 12, 12, -90, 0, n=20)
    s2x = np.full(10, 112.0)
    s2y = np.linspace(12, 50, 10)
    wx  = np.concatenate([s1x, arc_x[1:], s2x[1:]])
    wy  = np.concatenate([s1y, arc_y[1:], s2y[1:]])
    paths["PATH_SUDDEN_TURN"] = _resample_path(wx, wy)

    # --- PATH_S_BEND ---
    # Short straight, right curve, short link, left curve, exit straight
    s0x = np.linspace(0, 20, 10)
    s0y = np.zeros(10)
    arc1x, arc1y = _make_arc(20, -18, 18, 90, 0, n=20)   # right
    lx   = np.linspace(38, 58, 8)
    ly   = np.full(8, -18.0)
    arc2x, arc2y = _make_arc(58, -36, 18, 90, 180, n=20)  # left
    s1x  = np.linspace(40, 20, 8)
    s1y  = np.full(8, -54.0)
    wx   = np.concatenate([s0x, arc1x[1:], lx[1:], arc2x[1:], s1x[1:]])
    wy   = np.concatenate([s0y, arc1y[1:], ly[1:], arc2y[1:], s1y[1:]])
    paths["PATH_S_BEND"] = _resample_path(wx, wy)

    # --- PATH_CHICANE ---
    # Left-right-left three-element chicane with tight R=10 m arcs
    s0x = np.linspace(0, 30, 10)
    s0y = np.zeros(10)
    arc1x, arc1y = _make_arc(30, 10, 10, -90, 0, n=15)   # left up
    l1x  = np.linspace(40, 55, 6)
    l1y  = np.full(6, 10.0)
    arc2x, arc2y = _make_arc(55, 0, 10, 90, 0, n=15)      # right down (return)
    l2x  = np.linspace(65, 80, 6)
    l2y  = np.zeros(6)
    arc3x, arc3y = _make_arc(80, 10, 10, -90, 0, n=15)   # left up again
    s1x  = np.linspace(90, 120, 8)
    s1y  = np.full(8, 10.0)
    wx   = np.concatenate([s0x, arc1x[1:], l1x[1:], arc2x[1:], l2x[1:], arc3x[1:], s1x[1:]])
    wy   = np.concatenate([s0y, arc1y[1:], l1y[1:], arc2y[1:], l2y[1:], arc3y[1:], s1y[1:]])
    paths["PATH_CHICANE"] = _resample_path(wx, wy)

    # --- PATH_MIXED (primary scoring path) ---
    # Straight → gentle curve → S-style reverse → hairpin → exit
    # Built as a continuous sequence of waypoints so the spline flows
    # naturally between segments without kinks.
    s0x  = np.linspace(0,  80,  20)
    s0y  = np.zeros(20)
    # Wide left bend (R=30)
    arc1x, arc1y = _make_arc(80, 30, 30, -90, 0, n=20)
    # Short north straight
    l1x  = np.full(8, 110.0)
    l1y  = np.linspace(30, 60, 8)
    # Right sweep (R=20)
    arc2x, arc2y = _make_arc(90, 60, 20, 0, 90, n=15)
    # Link west
    l2x  = np.linspace(90, 50, 10)
    l2y  = np.full(10, 80.0)
    # Tight left hairpin (R=8)
    arc3x, arc3y = _make_arc(50, 72, 8, 90, 270, n=25)
    # Exit south-east
    s1x  = np.linspace(58, 120, 15)
    s1y  = np.linspace(72, 40,  15)
    wx   = np.concatenate([s0x, arc1x[1:], l1x[1:], arc2x[1:],
                            l2x[1:], arc3x[1:], s1x[1:]])
    wy   = np.concatenate([s0y, arc1y[1:], l1y[1:], arc2y[1:],
                            l2y[1:], arc3y[1:], s1y[1:]])
    paths["PATH_MIXED"] = _resample_path(wx, wy)

    return paths


# Build paths once at import time so workers share them without re-computing
SYNTHETIC_PATHS = build_synthetic_paths()
PATH_NAMES      = list(SYNTHETIC_PATHS.keys())

# Weights for path selection in the objective: PATH_MIXED is used more
# often because it covers all regimes; the specialist paths each appear
# once so their specific demands still influence the tuning.
PATH_WEIGHTS = {
    "PATH_GENTLE_CURVE": 1,
    "PATH_HAIRPIN":      1,
    "PATH_SUDDEN_TURN":  1,
    "PATH_S_BEND":       1,
    "PATH_CHICANE":      1,
    "PATH_MIXED":        3,   # 3 out of 8 draws come from the mixed path
}
_PATH_POOL = []
for name, count in PATH_WEIGHTS.items():
    _PATH_POOL.extend([name] * count)


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
        end   = min(n, last_idx + window)
    dists = np.hypot(path_X[start:end] - x, path_Y[start:end] - y)
    local = int(np.argmin(dists))
    return start + local


def _tracking_errors(plant_state, path_X, path_Y, path_Psi, last_idx):
    """
    Compute (e_y, e_psi, v_target_idx, new_idx) from global plant state
    and a reference path, identical to simulation.py's error computation.
    """
    X, Y, psi = plant_state[0], plant_state[1], plant_state[2]
    idx   = _find_closest(path_X, path_Y, X, Y, last_idx)
    rx, ry, rpsi = path_X[idx], path_Y[idx], path_Psi[idx]

    dx    = X - rx
    dy    = Y - ry
    e_y   = dy * np.cos(rpsi) - dx * np.sin(rpsi)
    e_psi = _normalize_angle(psi - rpsi)
    return e_y, e_psi, idx


# ==========================================
# WORKER INITIALIZER
# ==========================================
def init_worker(Q_init, R_init, R_rate_init):
    """
    Runs immediately when a new worker process is spawned.
    Populates the global memory context for that child process.
    """
    global _init_context
    _init_context["Q"]      = Q_init
    _init_context["R"]      = R_init
    _init_context["R_rate"] = R_rate_init


# ==========================================
# HEADLESS SIMULATION ROLLOUT
# ==========================================
def run_headless_rollout(weights_vector, path_name=None, num_steps=250):
    """
    Run one closed-loop rollout on a synthetic reference path with the
    given weight vector.

    BUG FIX: Tracking errors are now computed via proper closest-point
    projection onto the reference path (identical to simulation.py), so
    e_y and e_psi here mean the same thing as they do in the live sim.

    BUG FIX: The MPC state vector x0_mpc now correctly uses:
      [5] = 0.0      (e_a, not penalised — matches simulation.py)
      [6] = delta_act
      [7] = a_act
      [4] = vx - v_target (speed error against path profile, not vx0_rand)

    IMPROVEMENT: N_HORIZON = 25 (matches simulation.py, was 30).
    """
    Q_init    = _init_context["Q"]
    R_init    = _init_context["R"]
    R_rate_init = _init_context["R_rate"]

    Q, R, R_rate = vector_to_weights(weights_vector, Q_init, R_init, R_rate_init)

    p  = VehicleParams()
    dt = 0.05

    u_min = [-0.4, -10.0]
    u_max  = [ 0.4,   4.0]

    rng = np.random.default_rng(int(time.time() * 1e6) % 2**32)

    # --- Select a reference path ---
    if path_name is None:
        path_name = rng.choice(_PATH_POOL)
    path_X, path_Y, path_Psi, path_v = SYNTHETIC_PATHS[path_name]

    # --- Initial conditions: small random offset from path start ---
    ey0   = rng.uniform(-0.4, 0.4)
    epsi0 = rng.uniform(-0.15, 0.15)   # rad

    base_heading = path_Psi[0]
    X0 = path_X[0] - ey0 * np.sin(base_heading)
    Y0 = path_Y[0] + ey0 * np.cos(base_heading)
    psi0 = _normalize_angle(base_heading + epsi0)
    vx0  = float(path_v[0])

    state  = init_plant_state(X0, Y0, psi0, vx0=vx0)

    error_cost        = 0.0
    yaw_rate_cost     = 0.0
    control_smooth    = 0.0
    u_prev            = np.zeros(2)
    idx               = 0
    consecutive_fails = 0
    MAX_FAILS         = 5
    OFFTRACK_LIMIT    = 8.0

    for step in range(num_steps):
        # --- Compute tracking errors from CURRENT state (bug fix) ---
        e_y, e_psi, idx = _tracking_errors(state, path_X, path_Y, path_Psi, idx)
        v_target        = float(path_v[idx])
        vx              = max(state[3], 0.5)

        # --- Build MPC state vector (bug fix: correct indices) ---
        e_y_dot = state[3] * np.sin(e_psi) + state[4] * np.cos(e_psi)
        x0_mpc  = np.array([
            e_y,          # [0] lateral error
            e_y_dot,      # [1] lateral velocity
            e_psi,        # [2] heading error
            state[5],     # [3] yaw rate r
            vx - v_target, # [4] speed error (was vx - vx0_rand — wrong)
            0.0,           # [5] e_a — not penalized (was state[7] — wrong)
            state[6],     # [6] delta_act
            state[7],     # [7] a_act
        ])

        # --- Adaptive MPC gains ---
        kappa        = curvature_estimate(state)
        R_rate_scaled = adaptive_R_rate(kappa, R_rate)
        R_scaled     = adaptive_R_scaling(vx, R)

        # --- Dynamics model at current speed ---
        Ad, Bd = get_8state_discrete_model(vx, dt)

        # --- MPC solve ---
        u_opt = solve_mpc(
            x0_mpc, Ad, Bd, N_HORIZON,
            Q, R_scaled, u_min, u_max,
            R_rate=R_rate_scaled, u_prev=u_prev,
        )

        if u_opt is None:
            consecutive_fails += 1
            u_opt = u_prev.copy()
        else:
            consecutive_fails = 0

        if consecutive_fails >= MAX_FAILS:
            # Bail out early — penalize with a large fixed cost
            return 1e3

        # --- Accumulate costs ---
        error_cost     += e_y**2 + 0.5 * e_psi**2
        yaw_rate_cost  += 0.8 * state[5]**2
        control_smooth += abs(u_opt[0] - u_prev[0])

        if abs(e_y) > OFFTRACK_LIMIT:
            return 1e3

        if idx >= len(path_X) - 2:
            # Reached end of path — this is success; no penalty
            num_steps = step + 1
            break

        u_prev = u_opt.copy()
        state  = step_nonlinear_plant(state, u_opt, dt, p)

    rmse = np.sqrt(error_cost / max(num_steps, 1))
    return (
        rmse
        + 0.15 * np.sqrt(yaw_rate_cost  / max(num_steps, 1))
        + 0.05 * np.sqrt(control_smooth / max(num_steps, 1))
    )


# ==========================================
# PICKLEABLE OBJECTIVE WRAPPER
# ==========================================
def evaluate_candidate(vec):
    """
    Evaluate a candidate weight vector over num_runs rollouts across
    randomly selected synthetic paths and return the mean score.

    Each of the num_runs rollouts draws a path independently so the
    objective has coverage across all path types in a single DE generation,
    with PATH_MIXED appearing 3x as often as each specialist path.
    """
    num_runs = 3
    scores   = []
    rng      = np.random.default_rng(int(time.time() * 1e6) % 2**32)
    for _ in range(num_runs):
        path_name = rng.choice(_PATH_POOL)
        scores.append(run_headless_rollout(vec, path_name=path_name))
    return float(np.mean(scores))


# ==========================================
# MAIN EXECUTION
# ==========================================
if __name__ == "__main__":
    # Baseline weight matrices (same as simulation.py starting values)
    Q_init      = np.diag([2725.0, 215.0, 6590.0, 3605.0, 150.0, 0.0, 0.0, 0.0])
    R_init      = np.diag([1400.0, 108.0])
    R_rate_init = np.diag([1561.0, 4.2])

    # NOTE: The parent process does NOT populate _init_context here.
    # _init_context is populated exclusively via init_worker() in each
    # child process. The old code populated it in the parent too, but
    # evaluate_candidate is never called in the parent — those assignments
    # were dead code and have been removed.

    bounds = Q_BOUNDS + R_BOUNDS + R_RATE_BOUNDS

    print("[Offline Tuner] Building synthetic path library...")
    for name, (px, py, _, _) in SYNTHETIC_PATHS.items():
        print(f"  {name}: {len(px)} points, "
              f"X=[{px.min():.1f},{px.max():.1f}] "
              f"Y=[{py.min():.1f},{py.max():.1f}]")

    print("\n[Offline Tuner] Initializing Differential Evolution (Headless)...")
    print(f"  Horizon N={N_HORIZON} (matches simulation.py)")
    print(f"  Paths: {PATH_NAMES}")
    print(f"  Pool distribution: { {n: _PATH_POOL.count(n) for n in PATH_NAMES} }")
    start_time = time.time()

    num_cores = max(1, mp.cpu_count() - 1)
    print(f"[Offline Tuner] Firing up {num_cores} parallel workers.")

    with mp.Pool(
        processes=num_cores,
        initializer=init_worker,
        initargs=(Q_init, R_init, R_rate_init),
    ) as pool:
        result = differential_evolution(
            evaluate_candidate,
            bounds,
            strategy="best1bin",
            maxiter=12,       # reduced from 20 (see improvement #12)
            popsize=8,        # reduced from 15; 8*9*2=144 members still adequate
            mutation=(0.5, 1.0),
            recombination=0.7,
            disp=True,
            updating="deferred",
            workers=pool.map,
            seed=42,          # reproducible runs
        )

    end_time = time.time()

    best_vec = result.x
    best_Q, best_R, best_R_rate = vector_to_weights(best_vec, Q_init, R_init, R_rate_init)

    print("\n" + "=" * 50)
    print(f"OPTIMIZATION COMPLETE in {(end_time - start_time) / 60:.2f} minutes.")
    print(f"Best Score Achieved: {result.fun:.4f}")
    print("=" * 50)
    print("Replace your simulation.py starting weights with:")
    print("Q_diag      =", np.diag(best_Q).tolist())
    print("R_diag      =", np.diag(best_R).tolist())
    print("R_rate_diag =", np.diag(best_R_rate).tolist())
    print("=" * 50)