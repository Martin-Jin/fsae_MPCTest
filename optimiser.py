# Language: python
# Title: Vectorized optimiser.py

import cvxpy as cp
import numpy as np

# Global problem cache
_mpc_cache = None

def init_parameterized_mpc(nx, nu, N, u_min, u_max):
    """
    Vectorized CVXPY formulation.
    Reduces the expression graph size for significantly faster backend parsing.
    """
    A_param = cp.Parameter((nx, nx))
    B_param = cp.Parameter((nx, nu))
    x0_param = cp.Parameter(nx)

    # Use 2D shapes for safe broadcasting across the N horizon
    sqrtQ_param     = cp.Parameter((nx, 1), nonneg=True)
    sqrtR_param     = cp.Parameter((nu, 1), nonneg=True)
    sqrtR_rate_param = cp.Parameter((nu, 1), nonneg=True)
    weighted_u_prev_param = cp.Parameter(nu)

    x     = cp.Variable((nx, N + 1))
    u     = cp.Variable((nu, N))
    slack = cp.Variable(N)

    W_slack = 10000.0

    # Vectorized Cost
    cost  = cp.sum(cp.sum_squares(cp.multiply(sqrtQ_param, x)))
    cost += cp.sum(cp.sum_squares(cp.multiply(sqrtR_param, u)))
    cost += W_slack * cp.sum_squares(slack)

    # Vectorized Rate-of-Change Penalty
    # First step vs previous applied input
    cost += cp.sum_squares(
        cp.multiply(sqrtR_rate_param[:, 0], u[:, 0]) - weighted_u_prev_param
    )

    # Remaining steps using cp.diff
    if N > 1:
        du = cp.diff(u, axis=1)
        cost += cp.sum(cp.sum_squares(cp.multiply(sqrtR_rate_param, du)))

    # Vectorized Constraints
    constraints = [
        x[:, 0] == x0_param,
        x[:, 1:] == A_param @ x[:, :-1] + B_param @ u,
        u >= np.array(u_min)[:, None],
        u <= np.array(u_max)[:, None],
        x[0, :-1] <=  3.5 + slack,
        x[0, :-1] >= -3.5 - slack,
    ]

    prob = cp.Problem(cp.Minimize(cost), constraints)

    return {
        'prob': prob, 'A': A_param, 'B': B_param, 'x0': x0_param,
        'sqrtQ': sqrtQ_param, 'sqrtR': sqrtR_param,
        'sqrtR_rate': sqrtR_rate_param,
        'weighted_u_prev': weighted_u_prev_param,
        'u': u,
    }


def solve_mpc(x0, Ad, Bd, N, Q, R, u_min, u_max, R_rate=None, u_prev=None,
              silent=False, return_status=False,
              eps_abs=1e-5, eps_rel=1e-5, max_iter=8000, warm_start=True):
    """
    Executes the parameterized MPC solver using OSQP with Clarabel fallback.

    eps_abs / eps_rel / max_iter tune the OSQP stopping criteria. They default
    to the tight, live-simulator settings; the offline tuner passes looser
    values so its many rollouts run faster (see offline_tuner.ROLLOUT_EPS).

    warm_start: reuse the previous solve's solution as the initial guess. This
    is what makes the receding-horizon solves fast in the live sim. The offline
    tuner passes warm_start=False on the FIRST step of each rollout so that a
    rollout never inherits solver state left behind by a previous, unrelated
    rollout — that carryover made the "deterministic" objective depend on
    evaluation order.

    Solver status handling:
      OPTIMAL            — accepted silently.
      OPTIMAL_INACCURATE — accepted but a warning is printed so callers can
                           monitor how often the QP is near the edge of
                           feasibility. The control action is still used
                           rather than discarding it at 20 Hz.
      anything else      — Clarabel fallback is attempted; if that also
                           fails, None is returned and the caller should
                           hold the previous command.
    """
    global _mpc_cache

    nx, nu = 8, 2

    if R_rate is None:
        R_rate = np.zeros((nu, nu))
    if u_prev is None:
        u_prev = np.zeros(nu)

    # ----------------------------
    # Cache build / rebuild on horizon change
    # ----------------------------
    if _mpc_cache is None or _mpc_cache['u'].shape[1] != N:
        _mpc_cache = init_parameterized_mpc(nx, nu, N, u_min, u_max)

    # ----------------------------
    # Assign dynamics
    # ----------------------------
    _mpc_cache['A'].value  = Ad
    _mpc_cache['B'].value  = Bd
    _mpc_cache['x0'].value = x0

    # ----------------------------
    # Safe weight extraction
    # ----------------------------
    Q_diag      = np.diag(Q)      if Q.ndim      == 2 else np.asarray(Q)
    R_diag      = np.diag(R)      if R.ndim      == 2 else np.asarray(R)
    R_rate_diag = np.diag(R_rate) if np.ndim(R_rate) == 2 else np.asarray(R_rate)

    # Clipping (must happen AFTER extraction)
    Q_diag      = np.clip(Q_diag,      1e-6, 1e6)
    R_diag      = np.clip(R_diag,      1e-6, 1e6)
    R_rate_diag = np.clip(R_rate_diag, 1e-6, 1e6)

    # Convert to sqrt form required by the CVXPY sum_squares formulation
    sqrtQ      = np.sqrt(Q_diag)
    sqrtR      = np.sqrt(R_diag)
    sqrtR_rate = np.sqrt(R_rate_diag)

    _mpc_cache['sqrtQ'].value          = sqrtQ[:, None]
    _mpc_cache['sqrtR'].value          = sqrtR[:, None]
    _mpc_cache['sqrtR_rate'].value     = sqrtR_rate[:, None]
    _mpc_cache['weighted_u_prev'].value = sqrtR_rate * np.asarray(u_prev)

    # ----------------------------
    # Solve: OSQP primary, Clarabel fallback
    # ----------------------------
    try:
        _mpc_cache['prob'].solve(
            solver=cp.OSQP,
            warm_start=warm_start,
            eps_abs=eps_abs,
            eps_rel=eps_rel,
            max_iter=max_iter,
        )
        status = _mpc_cache['prob'].status

        if status == cp.OPTIMAL_INACCURATE and not silent:
            # Warn but continue — the solution is usable at 20 Hz even if
            # the QP stopped before reaching full convergence tolerances.
            print(f"[MPC] Warning: OSQP returned OPTIMAL_INACCURATE "
                  f"(consider tightening eps or checking weight magnitudes)")

        if status not in (cp.OPTIMAL, cp.OPTIMAL_INACCURATE):
            # OSQP failed outright — try Clarabel before giving up
            _mpc_cache['prob'].solve(solver=cp.CLARABEL)
            status = _mpc_cache['prob'].status

    except cp.error.SolverError as e:
        if not silent:
            print(f"[MPC] Warning: Solver error: {e}")
        return None

    if status not in (cp.OPTIMAL, cp.OPTIMAL_INACCURATE):
        return None

    u_sol = _mpc_cache['u'][:, 0].value

    if u_sol is None or not np.all(np.isfinite(u_sol)):
        return None

    # Return the status tuple if requested (used strictly by offline tuner)
    if return_status:
        return u_sol, status

    return u_sol