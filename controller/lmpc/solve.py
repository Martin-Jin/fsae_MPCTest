"""
controller/lmpc/solve.py — per-tick LMPC solve, plus the module-level
compiled-problem cache init_parameterized_mpc() builds and this rebuilds
on a shape/bound change. See build.py's module docstring for the design
(parameterized CVXPY problem, warm start, cost function) this solve
executes each tick.

USED BY
-------
  gui/simulation.py      — calls solve_mpc() at every simulation step
  tuner/offline_tuner.py — calls solve_mpc() inside run_headless_rollout()
                     with relaxed tolerances (ROLLOUT_EPS) for speed

DOES NOT USE
------------
  model/vehicle_physics.py (directly), model/bicycle_model.py (receives Ad/Bd as arguments),
  sim/speed_profile.py, sim/perception.py, sim/planner.py, tuner/performance_stats.py
"""

import cvxpy as cp
import numpy as np

from controller.lmpc.build import init_parameterized_mpc

# Module-level cache: stores the compiled CVXPY problem and Parameter references.
# Avoids rebuilding the expression graph on every call, which would be ~10x slower.
_mpc_cache = None


def solve_mpc(x0, Ad, Bd, N, Q, R, u_min, u_max, R_rate=None, u_prev=None,
              silent=False, return_status=False,
              eps_abs=1e-5, eps_rel=1e-5, max_iter=8000, warm_start=True,
              du_max=None, terminal_scale=1.0,
              r_a_accel=None, r_a_brake=None):
    """
    Execute the parameterized MPC solve for the current timestep.

    This function:
      1. Rebuilds the cached problem if N has changed (rare).
      2. Injects the current state and dynamics matrices into CVXPY Parameters.
      3. Extracts and square-roots the diagonal weight matrices for the
         sum_squares formulation.
      4. Solves with OSQP; falls back to Clarabel if OSQP fails.
      5. Returns u[0] (the first step's control action to apply to the plant).

    SOLVER STRATEGY: OSQP primary (sparse, warm-startable, ~1-5 ms at N=25),
    Clarabel fallback on a non-optimal OSQP status. See architecture.md's
    "The solver" section for the full OSQP/Clarabel/OPTIMAL_INACCURATE
    reasoning — not repeated here.

    warm_start=False is used by the offline tuner for the FIRST step of each
    rollout only, to avoid inheriting stale state from a previous rollout —
    every other caller wants the default True.

    eps_abs/eps_rel/max_iter: both gui/simulation.py and tuner/offline_tuner.py
    pass settings.ROLLOUT_EPS/ROLLOUT_MAX_ITER rather than this function's own
    defaults, so live and offline runs solve to the same tolerance and stay
    comparable.

    Parameters
    ----------
    x0 : np.ndarray, shape (8,)
        Current MPC state vector [e_y, e_y_dot, e_psi, e_psi_dot, e_v, 0,
        delta_act, a_act]. Built in gui/simulation.py or tuner/offline_tuner.py
        from the plant's current state and tracking errors.
    Ad : np.ndarray, shape (8, 8)
        Discrete-time A matrix from get_8state_discrete_model(vx, dt).
    Bd : np.ndarray, shape (8, 2)
        Discrete-time B matrix from get_8state_discrete_model(vx, dt).
    N : int
        MPC prediction horizon (number of steps).
        Must be consistent with the cached problem — a change triggers rebuild.
    Q : np.ndarray, shape (8,8) or (8,)
        State cost matrix or diagonal vector. Penalises tracking errors.
    R : np.ndarray, shape (2,2) or (2,)
        Input cost matrix or diagonal vector. Penalises control effort.
    u_min : array-like, shape (2,)
        Lower bounds on control inputs [delta_min, a_min].
    u_max : array-like, shape (2,)
        Upper bounds on control inputs [delta_max, a_max].
    R_rate : np.ndarray, shape (2,2) or (2,), optional
        Rate-of-change cost matrix. Penalises Δu between timesteps.
        If None, rate cost is zero (no smoothness penalty).
    r_a_accel, r_a_brake : float, optional
        Independent effort weights for u[1,:] (a_cmd) by sign, applied via
        cp.pos/cp.neg in the cost instead of R's own [1,1] entry (R[1,1]
        is NOT used for a_cmd -- only R[0,0], delta_cmd, is read from it).
        Defaults to R[1,1] for both when omitted, exactly reproducing the
        pre-split single-R[1,1] behaviour. See settings.py's
        R_A_ACCEL/R_A_BRAKE and `docs/reference/README.md`'s "Accel/brake
        effort weight split".
    u_prev : array-like, shape (2,), optional
        Previously applied control input. Used as anchor for the step-0
        rate cost, and (if du_max is set) the step-0 hard slew constraint.
        If None, zeros are assumed.
    silent : bool, optional
        If True, suppress OPTIMAL_INACCURATE warnings. Used by offline tuner
        where warning noise would flood the console during mass rollouts.
    return_status : bool, optional
        If True, return (u_sol, status) tuple instead of just u_sol.
        Used by tuner/offline_tuner.py to count OPTIMAL_INACCURATE occurrences.
    eps_abs, eps_rel : float, optional
        OSQP absolute and relative convergence tolerances.
    max_iter : int, optional
        OSQP maximum iteration count.
    warm_start : bool, optional
        Whether to warm-start OSQP from the previous solution.
    terminal_scale : float, optional
        See init_parameterized_mpc's docstring. 1.0 (default) is a no-op.

    Returns
    -------
    u_sol : np.ndarray, shape (2,) or None
        Optimal first-step control action [delta_cmd, a_cmd] to apply.
        Returns None if both OSQP and Clarabel fail — caller should hold
        the previous command.
    (u_sol, status) if return_status=True.

    Called by: gui/simulation.py (simulate_closed_loop),
               tuner/offline_tuner.py (run_headless_rollout)
    """
    global _mpc_cache

    nx, nu = 8, 2

    if R_rate is None:
        R_rate = np.zeros((nu, nu))
    if u_prev is None:
        u_prev = np.zeros(nu)

    # ── Build or rebuild cache if horizon N has changed ───────────────────────
    # In normal operation the cache is built once; N is fixed across the session.
    # Rebuild cache if horizon N changed, OR if u_min/u_max differ from what
    # the cache was built with. u_min/u_max are baked into the cached QP as
    # plain constants (see init_parameterized_mpc), so — unlike Q/R/R_rate,
    # which flow through cp.Parameters every call — a caller passing
    # different bounds with the same N would otherwise silently keep getting
    # the stale, originally-cached bounds enforced. 
    # du_max is baked in the same way and needs the same staleness check.
    u_min_arr = np.asarray(u_min, dtype=float)
    u_max_arr = np.asarray(u_max, dtype=float)
    du_max_arr = None if du_max is None else np.asarray(du_max, dtype=float)
    cached_du = None if _mpc_cache is None else _mpc_cache.get('du_max')
    du_changed = (
        (cached_du is None) != (du_max_arr is None)
        or (du_max_arr is not None and not np.array_equal(cached_du, du_max_arr))
    )
    # terminal_scale is baked in like du_max/u_min/u_max -- needs the same check.
    terminal_changed = (
        _mpc_cache is not None
        and _mpc_cache.get('terminal_scale', 1.0) != float(terminal_scale)
    )
    needs_rebuild = (
        _mpc_cache is None
        or _mpc_cache['u'].shape[1] != N
        or not np.array_equal(_mpc_cache['u_min'], u_min_arr)
        or not np.array_equal(_mpc_cache['u_max'], u_max_arr)
        or du_changed
        or terminal_changed
    )
    if needs_rebuild:
        _mpc_cache = init_parameterized_mpc(nx, nu, N, u_min, u_max, du_max,
                                             terminal_scale=terminal_scale)

    # ── Inject dynamics matrices (change every timestep as vx changes) ────────
    _mpc_cache['A'].value  = Ad
    _mpc_cache['B'].value  = Bd
    _mpc_cache['x0'].value = x0

    # ── Extract diagonal weights (handle both 2D matrix and 1D vector inputs) ─
    Q_diag      = np.diag(Q)      if Q.ndim      == 2 else np.asarray(Q)
    R_diag      = np.diag(R)      if R.ndim      == 2 else np.asarray(R)
    R_rate_diag = np.diag(R_rate) if np.ndim(R_rate) == 2 else np.asarray(R_rate)

    # Clip to safe range: avoid numerical issues from extreme weight values.
    # Lower bound 1e-6 prevents near-zero weights from causing ill-conditioning.
    # Upper bound 1e6 prevents overflow in the QP's P matrix.
    Q_diag      = np.clip(Q_diag,      1e-6, 1e6)
    R_diag      = np.clip(R_diag,      1e-6, 1e6)
    R_rate_diag = np.clip(R_rate_diag, 1e-6, 1e6)

    # Convert to sqrt form: cp.sum_squares(sqrt(w) * x) = w * x²
    # This is equivalent to the standard x^T Q x but avoids forming Q explicitly.
    sqrtQ      = np.sqrt(Q_diag)
    sqrtR      = np.sqrt(R_diag)
    sqrtR_rate = np.sqrt(R_rate_diag)

    _mpc_cache['sqrtQ'].value          = sqrtQ[:, None]          # (nx, 1) for broadcasting
    _mpc_cache['sqrtR'].value          = sqrtR[:, None]          # (nu, 1)
    _mpc_cache['sqrtR_rate'].value     = sqrtR_rate[:, None]     # (nu, 1)
    # a_cmd effort split by sign -- defaults to R_diag[1] for both when the
    # caller doesn't pass r_a_accel/r_a_brake, reproducing pre-split behaviour.
    _mpc_cache['r_a_accel'].value = float(np.clip(
        R_diag[1] if r_a_accel is None else r_a_accel, 1e-6, 1e6))
    _mpc_cache['r_a_brake'].value = float(np.clip(
        R_diag[1] if r_a_brake is None else r_a_brake, 1e-6, 1e6))
    # Pre-multiply u_prev by sqrtR_rate so the rate cost at step 0 is:
    # ||sqrtR_rate * u[:,0] - sqrtR_rate * u_prev||² = ||sqrtR_rate ⊙ Δu_0||²
    _mpc_cache['weighted_u_prev'].value = sqrtR_rate * np.asarray(u_prev)
    _mpc_cache['u_prev'].value           = np.asarray(u_prev, dtype=float)

    # ── Solve: OSQP primary, Clarabel fallback ─────────────────────────────────
    try:
        _mpc_cache['prob'].solve(
            solver=cp.OSQP,
            warm_start=warm_start,   # Reuse previous solution as initial guess
            eps_abs=eps_abs,         # Absolute convergence tolerance
            eps_rel=eps_rel,         # Relative convergence tolerance
            max_iter=max_iter,       # Iteration cap
        )
        status = _mpc_cache['prob'].status

        if status == cp.OPTIMAL_INACCURATE and not silent:
            # Usable despite inaccuracy; better than falling back to a held command.
            print(f"[MPC] Warning: OSQP returned OPTIMAL_INACCURATE "
                  f"(consider tightening eps or checking weight magnitudes)")

        if status not in (cp.OPTIMAL, cp.OPTIMAL_INACCURATE):
            _mpc_cache['prob'].solve(solver=cp.CLARABEL)
            status = _mpc_cache['prob'].status

    except cp.error.SolverError as e:
        if not silent:
            print(f"[MPC] Warning: Solver error: {e}")
        return None

    if status not in (cp.OPTIMAL, cp.OPTIMAL_INACCURATE):
        # Both solvers failed; return None so caller can hold the previous command.
        return None

    # ── Extract solution ────────────────────────────────────────────────────────
    # u[:,0] is the first-step control action (only this is applied).
    u_sol = _mpc_cache['u'][:, 0].value

    if u_sol is None or not np.all(np.isfinite(u_sol)):
        # Solver returned None values or NaN/Inf — treat as failure.
        return None

    if return_status:
        return u_sol, status

    return u_sol
