# Language: python
# Title: Vectorized optimiser.py Replacement
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
    sqrtQ_param = cp.Parameter((nx, 1), nonneg=True)
    sqrtR_param = cp.Parameter((nu, 1), nonneg=True)
    sqrtR_rate_param = cp.Parameter((nu, 1), nonneg=True) 
    weighted_u_prev_param = cp.Parameter(nu)

    x = cp.Variable((nx, N + 1))
    u = cp.Variable((nu, N))
    slack = cp.Variable(N)

    W_slack = 10000.0

    # Vectorized Cost
    cost = cp.sum(cp.sum_squares(cp.multiply(sqrtQ_param, x)))
    cost += cp.sum(cp.sum_squares(cp.multiply(sqrtR_param, u)))
    cost += W_slack * cp.sum_squares(slack)

    # Vectorized Rate-of-Change Penalty
    # First step vs previous applied input
    cost += cp.sum_squares(cp.multiply(sqrtR_rate_param[:, 0], u[:, 0]) - weighted_u_prev_param)
    
    # Remaining steps using cp.diff
    if N > 1:
        du = cp.diff(u, axis=1)
        cost += cp.sum(cp.sum_squares(cp.multiply(sqrtR_rate_param, du)))

    # Vectorized Constraints collapsing N objects into a single mathematical block
    constraints = [
        x[:, 0] == x0_param,
        x[:, 1:] == A_param @ x[:, :-1] + B_param @ u,
        u >= np.array(u_min)[:, None],
        u <= np.array(u_max)[:, None],
        x[0, :-1] <= 3.5 + slack,
        x[0, :-1] >= -3.5 - slack
    ]

    prob = cp.Problem(cp.Minimize(cost), constraints)

    return {
        'prob': prob, 'A': A_param, 'B': B_param, 'x0': x0_param,
        'sqrtQ': sqrtQ_param, 'sqrtR': sqrtR_param, 'sqrtR_rate': sqrtR_rate_param,
        'weighted_u_prev': weighted_u_prev_param, 'u': u,
    }

def solve_mpc(x0, Ad, Bd, N, Q, R, u_min, u_max, R_rate=None, u_prev=None):
    """
    Executes the parameterized MPC solver using OSQP with a Clarabel fallback.
    """
    global _mpc_cache
    nx, nu = 8, 2

    if R_rate is None:
        R_rate = np.zeros(nu)
    if u_prev is None:
        u_prev = np.zeros(nu)

    if _mpc_cache is None or _mpc_cache['u'].shape[1] != N:
        _mpc_cache = init_parameterized_mpc(nx, nu, N, u_min, u_max)

    _mpc_cache['A'].value = Ad
    _mpc_cache['B'].value = Bd
    _mpc_cache['x0'].value = x0
    
    Q_diag = np.diag(Q) if Q.ndim == 2 else np.asarray(Q)
    R_diag = np.diag(R) if R.ndim == 2 else np.asarray(R)
    R_rate_diag = np.diag(R_rate) if np.ndim(R_rate) == 2 else np.asarray(R_rate)
    
    # Inject values reshaped to (dim, 1) for CVXPY broadcasting
    _mpc_cache['sqrtQ'].value = np.sqrt(np.maximum(Q_diag, 0.0))[:, None]
    _mpc_cache['sqrtR'].value = np.sqrt(np.maximum(R_diag, 0.0))[:, None]
    sqrtR_rate = np.sqrt(np.maximum(R_rate_diag, 0.0))
    _mpc_cache['sqrtR_rate'].value = sqrtR_rate[:, None]
    _mpc_cache['weighted_u_prev'].value = sqrtR_rate * np.asarray(u_prev)

    try:
        # Rely on OSQP's ADMM for rapid warm starts
        _mpc_cache['prob'].solve(solver=cp.OSQP, warm_start=True, eps_abs=1e-3, eps_rel=1e-3)
        status = _mpc_cache['prob'].status
        
        if status not in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
            # Clarabel cleanly catches ill-conditioned dynamic scaling failures
            _mpc_cache['prob'].solve(solver=cp.CLARABEL)
            status = _mpc_cache['prob'].status
            
    except cp.error.SolverError as e:
        print(f"Warning: Solver raised an error ({e}).")
        return None

    if status not in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
        print(f"Warning: Solver failed with status {status}.")
        return None

    u_sol = _mpc_cache['u'][:, 0].value
    if u_sol is None or not np.all(np.isfinite(u_sol)):
        print("Warning: Solver returned non-finite control.")
        return None

    return u_sol