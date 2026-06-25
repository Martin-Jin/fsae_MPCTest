import cvxpy as cp

# ==========================================
# 2. MPC OPTIMIZER SETUP
# ==========================================
def solve_mpc(x0, Ad, Bd, N, Q, R, u_min, u_max):
    """
    Constructs and solves the LQR/Tracking cost optimization problem over horizon N.
    Returns the optimal control input for the first time step.
    """
    nx = 8 # Number of states
    nu = 2 # Number of inputs
    
    # CVXPY Variables representing future states and inputs
    x = cp.Variable((nx, N + 1))
    u = cp.Variable((nu, N))
    
    cost = 0
    constraints = [x[:, 0] == x0] # Initial state constraint
    
    # Build cost function and constraints over the finite horizon
    for k in range(N):
        # LQR Cost: J = x^T Q x + u^T R u
        cost += cp.quad_form(x[:, k], Q) + cp.quad_form(u[:, k], R)
        
        # Subject to Bound Dynamics: x(k+1) = A x(k) + B u(k)
        constraints += [x[:, k+1] == Ad @ x[:, k] + Bd @ u[:, k]]
        
        # Actuator Constraints
        constraints += [u[:, k] >= u_min, u[:, k] <= u_max]
        
    # Terminal Cost: Penelizes error at the final prediction step N
    cost += cp.quad_form(x[:, N], Q)
    
    # Configure and execute solver
    prob = cp.Problem(cp.Minimize(cost), constraints)
    prob.solve(solver=cp.OSQP, warm_start=True)
    
    # Fallback safety if the solver fails to find a feasible path
    if prob.status != cp.OPTIMAL:
        print("Warning: Solver failed. Outputting zero control.")
        return np.zeros(nu)
        
    return u[:, 0].value # Return only the first optimal control action