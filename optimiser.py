"""
High-Performance Parameterized MPC Optimizer with Slack Variables (v2)
File Name: optimiser.py
"""
import cvxpy as cp
import numpy as np

# Global problem cache to persist the compiled model across simulation steps
_mpc_cache = None

def init_parameterized_mpc(nx, nu, N, Q, R, u_min, u_max):
    """
    Compiles the optimization structure once using cvxpy Parameters.
    """
    # Dynamic system parameters
    A_param = cp.Parameter((nx, nx))
    B_param = cp.Parameter((nx, nu))
    x0_param = cp.Parameter(nx)
    
    # Optimization Decision Variables
    x = cp.Variable((nx, N + 1))
    u = cp.Variable((nu, N))
    slack = cp.Variable(N)  # Slack variable to handle transient lateral errors safely
    
    cost = 0
    constraints = [x[:, 0] == x0_param]
    
    # Large penalty weight for violating tracking boundaries
    W_slack = 10000.0
    
    for k in range(N):
        # Base LQR Cost + Soft Constraint Penalty
        cost += cp.quad_form(x[:, k], Q) + cp.quad_form(u[:, k], R) + W_slack * cp.square(slack[k])
        
        # State transitions mapped using parameters
        constraints += [x[:, k+1] == A_param @ x[:, k] + B_param @ u[:, k]]
        
        # Actuator saturation boundaries
        constraints += [u[:, k] >= u_min, u[:, k] <= u_max]
        
        # Soft-bound on lateral deviation to absorb sharp corner spikes
        constraints += [x[0, k] <= 3.5 + slack[k], x[0, k] >= -3.5 - slack[k]]
        
    # Terminal step cost
    cost += cp.quad_form(x[:, N], Q)
    
    prob = cp.Problem(cp.Minimize(cost), constraints)
    
    return {
        'prob': prob, 'A': A_param, 'B': B_param, 'x0': x0_param, 'u': u
    }

def solve_mpc(x0, Ad, Bd, N, Q, R, u_min, u_max):
    """
    Executes the parameterized MPC solver using warm starts.
    """
    global _mpc_cache
    nx, nu = 8, 2
    
    # Lazy compilation optimization step
    if _mpc_cache is None or _mpc_cache['u'].shape[1] != N:
        _mpc_cache = init_parameterized_mpc(nx, nu, N, Q, R, u_min, u_max)
        
    # Inject current matrices directly into problem variables without rebuilding
    _mpc_cache['A'].value = Ad
    _mpc_cache['B'].value = Bd
    _mpc_cache['x0'].value = x0
    
    # Solve leveraging OSQP's warm start capabilities
    _mpc_cache['prob'].solve(solver=cp.OSQP, warm_start=True, eps_abs=1e-3, eps_rel=1e-3)
    
    if _mpc_cache['prob'].status not in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
        print(f"Warning: Solver failed with status {_mpc_cache['prob'].status}. Yielding zero control.")
        return np.zeros(nu)
        
    return _mpc_cache['u'][:, 0].value