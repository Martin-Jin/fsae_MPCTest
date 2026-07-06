"""
optimiser.py — Parameterized MPC QP Solver

PURPOSE
-------
Formulates and solves the Model Predictive Control (MPC) optimisation problem
at each timestep. The MPC solves a finite-horizon optimal control problem:
choose a sequence of N control inputs {u[0], ..., u[N-1]} that minimise a
quadratic cost over the predicted trajectory, subject to linear input bounds
and a soft lane-boundary constraint.

The primary solver is OSQP (Operator Splitting QP), with Clarabel as an
automatic fallback when OSQP fails or returns an infeasible status.

HOW MPC WORKS (brief)
---------------------
At each timestep k, given the current state x[k]:
  1. Predict N steps forward using the linear model: x[k+i+1] = A*x[k+i] + B*u[k+i]
  2. Minimise: Σ ||Q^0.5 * x[i]||² + ||R^0.5 * u[i]||² + ||R_rate^0.5 * Δu[i]||²
     subject to input bounds and soft lateral corridor constraint.
  3. Apply only u[0] to the vehicle (receding horizon principle).
  4. Repeat at k+1 with the new measured state.

The receding horizon means the MPC constantly re-plans: even if the first few
steps were suboptimal, the next solve corrects for any mismatch. This is what
makes MPC robust to model-plant mismatch and disturbances.

PARAMETERIZED FORMULATION
--------------------------
Rather than rebuilding the CVXPY problem from scratch each timestep (slow),
the problem is built once with CVXPY Parameter objects as placeholders,
then only the parameter values are updated each step. This "warm start"
approach allows OSQP to reuse its factorisation from the previous solve,
dramatically reducing computation time — critical at 20 Hz.

COST FUNCTION
-------------
  State cost:        Σ_i ||sqrtQ ⊙ x[:,i]||²   (element-wise scaling, all N+1 steps)
  Input cost:        Σ_i ||sqrtR ⊙ u[:,i]||²   (penalise control effort)
  Rate-of-change:    Σ_i ||sqrtR_rate ⊙ Δu[:,i]||²  (penalise jerk/roughness)
  Slack:             W_slack * ||slack||²         (soft corridor violation penalty)

The sqrt formulations allow using cp.sum_squares which maps directly to OSQP's
internal P matrix (positive semidefinite quadratic form), which is more
numerically stable than passing Q directly.

USED BY
-------
  simulation.py    — calls solve_mpc() at every simulation step
  offline_tuner.py — calls solve_mpc() inside run_headless_rollout()
                     with relaxed tolerances (ROLLOUT_EPS) for speed

DOES NOT USE
------------
  vehicle_physics.py (directly), bicycle_model.py (receives Ad/Bd as arguments),
  speed_profile.py, sim_track.py, performance_stats.py
"""

import cvxpy as cp
import numpy as np

# Module-level cache: stores the compiled CVXPY problem and Parameter references.
# Avoids rebuilding the expression graph on every call, which would be ~10× slower.
_mpc_cache = None


def init_parameterized_mpc(nx, nu, N, u_min, u_max):
    """
    Build and compile a parameterized CVXPY MPC problem.

    This function is called once (or when the horizon N changes) and stores
    the compiled problem in _mpc_cache. Subsequent calls to solve_mpc() only
    update Parameter values and re-invoke the already-compiled problem, which
    OSQP solves via warm-start in a fraction of the initial build time.

    VECTORIZATION
    -------------
    All cost and constraint expressions operate on (nx, N+1) and (nu, N)
    matrices simultaneously, rather than looping over timesteps. This keeps
    the CVXPY expression graph small and avoids O(N) Python-level overhead.

    VARIABLES
    ---------
    x      : cp.Variable (nx, N+1)   Predicted state trajectory
    u      : cp.Variable (nu, N)     Control input sequence
    slack  : cp.Variable (N,)        Soft constraint violation (lane boundary)

    PARAMETERS (updated each solve without recompilation)
    ---------
    A_param            : (nx, nx)  Discrete-time A matrix from bicycle_model.py
    B_param            : (nx, nu)  Discrete-time B matrix from bicycle_model.py
    x0_param           : (nx,)     Current state (MPC initial condition)
    sqrtQ_param        : (nx, 1)   Element-wise sqrt of diagonal Q weights
    sqrtR_param        : (nu, 1)   Element-wise sqrt of diagonal R weights
    sqrtR_rate_param   : (nu, 1)   Element-wise sqrt of diagonal R_rate weights
    weighted_u_prev    : (nu,)     sqrtR_rate * u_prev (for rate cost at first step)

    Parameters
    ----------
    nx : int      State dimension (always 8 in this system)
    nu : int      Input dimension (always 2: [delta_cmd, a_cmd])
    N  : int      Prediction horizon (number of steps)
    u_min : array-like, shape (nu,)   Lower input bounds
    u_max : array-like, shape (nu,)   Upper input bounds

    Returns
    -------
    dict with keys:
        'prob'           : cp.Problem  — compiled problem object
        'A', 'B'         : cp.Parameter — dynamics matrices
        'x0'             : cp.Parameter — initial state
        'sqrtQ', 'sqrtR', 'sqrtR_rate' : cp.Parameter — cost weights
        'weighted_u_prev': cp.Parameter — rate cost anchor at step 0
        'u'              : cp.Variable  — control variable (for extracting u[0])

    Called by: solve_mpc() — on first call or when N changes
    """
    # ── CVXPY Parameters (updated each solve, not recompiled) ────────────────
    A_param = cp.Parameter((nx, nx))            # Discrete-time A from bicycle_model.py
    B_param = cp.Parameter((nx, nu))            # Discrete-time B from bicycle_model.py
    x0_param = cp.Parameter(nx)                 # Current MPC state

    # (nx,1) and (nu,1) shapes allow broadcasting across N horizon columns
    sqrtQ_param      = cp.Parameter((nx, 1), nonneg=True)   # State weight sqrt
    sqrtR_param      = cp.Parameter((nu, 1), nonneg=True)   # Input weight sqrt
    sqrtR_rate_param = cp.Parameter((nu, 1), nonneg=True)   # Rate weight sqrt
    weighted_u_prev_param = cp.Parameter(nu)   # sqrtR_rate * u_prev for step-0 rate cost

    # ── CVXPY Variables ────────────────────────────────────────────────────────
    x     = cp.Variable((nx, N + 1))   # Predicted states: x[:,0] = x0, x[:,N] = terminal
    u     = cp.Variable((nu, N))       # Control inputs: u[:,0] applied, u[:,1:] discarded
    slack = cp.Variable(N)             # Non-negative slack for soft lane constraint

    W_slack = 10000.0   # Hard penalty on lane-boundary violation; effectively enforces it

    # ── COST FUNCTION ─────────────────────────────────────────────────────────
    # State cost: sum over all N+1 states (including x[:,0] and x[:,N]).
    # cp.multiply(sqrtQ_param, x) broadcasts (nx,1) across (nx,N+1) columns.
    # cp.sum_squares computes Σ_ij (sqrtQ_i * x_ij)² = Σ_i Q_ii * Σ_j x_ij²
    cost  = cp.sum(cp.sum_squares(cp.multiply(sqrtQ_param, x)))

    # Input magnitude cost: penalises large control commands.
    cost += cp.sum(cp.sum_squares(cp.multiply(sqrtR_param, u)))

    # Slack penalty: quadratic to keep the corridor soft but strongly penalised.
    cost += W_slack * cp.sum_squares(slack)

    # Rate-of-change cost (step 0 vs. last applied input):
    # ||sqrtR_rate * u[:,0] - sqrtR_rate * u_prev||²
    # = ||sqrtR_rate ⊙ u[:,0] - weighted_u_prev||²
    # weighted_u_prev_param = sqrtR_rate * u_prev is pre-computed in solve_mpc().
    cost += cp.sum_squares(
        cp.multiply(sqrtR_rate_param[:, 0], u[:, 0]) - weighted_u_prev_param
    )

    # Rate-of-change cost (subsequent steps): penalise Δu = u[:,i+1] - u[:,i]
    # cp.diff computes first differences along axis=1: Δu[:,i] = u[:,i+1] - u[:,i]
    if N > 1:
        du    = cp.diff(u, axis=1)                                    # Shape (nu, N-1)
        cost += cp.sum(cp.sum_squares(cp.multiply(sqrtR_rate_param, du)))

    # ── CONSTRAINTS ───────────────────────────────────────────────────────────
    constraints = [
        # Initial condition: force predicted trajectory to start at current state
        x[:, 0] == x0_param,

        # Dynamics: x[k+1] = A*x[k] + B*u[k]  (vectorized over all N steps)
        # x[:,1:] is (nx,N); A_param @ x[:,:-1] is (nx,N); B_param @ u is (nx,N)
        x[:, 1:] == A_param @ x[:, :-1] + B_param @ u,

        # Hard input bounds: applied to all N control steps simultaneously
        u >= np.array(u_min)[:, None],   # Broadcasting: (nu,1) vs (nu,N)
        u <= np.array(u_max)[:, None],

        # Soft lane corridor: lateral error (state[0] = e_y) within ±3.5 m
        # Slack is added to the hard limit so violations are penalised but not
        # infeasible — essential when the vehicle is already off-track during recovery.
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
    Execute the parameterized MPC solve for the current timestep.

    This function:
      1. Rebuilds the cached problem if N has changed (rare).
      2. Injects the current state and dynamics matrices into CVXPY Parameters.
      3. Extracts and square-roots the diagonal weight matrices for the
         sum_squares formulation.
      4. Solves with OSQP; falls back to Clarabel if OSQP fails.
      5. Returns u[0] (the first step's control action to apply to the plant).

    SOLVER STRATEGY
    ---------------
    OSQP is the primary solver: it exploits the problem's sparsity, supports
    warm-starting (reusing the previous solution as the initial guess), and
    runs in ~1-5 ms for this problem size at N=25.

    Clarabel is the fallback: a newer interior-point solver, slightly slower
    but more robust to poorly conditioned problems. Used when OSQP returns
    a non-optimal status (e.g. time limit hit, numerical difficulties).

    WARM STARTING
    -------------
    warm_start=True (default) reuses the previous solve's primal/dual variables
    as the initial guess for OSQP. In the receding horizon, consecutive solves
    differ by only one step, so the previous solution is an excellent warm start
    and typically converges in 50-200 iterations vs. 500-2000 cold.

    warm_start=False is used by the offline tuner for the FIRST step of each
    rollout to avoid inheriting stale state from a previous rollout.

    SOLVER TOLERANCES
    -----------------
    eps_abs / eps_rel: OSQP convergence criteria. Tighter = more accurate but
    slower. The live simulator uses the default 1e-5; offline_tuner.py uses
    1e-4 (ROLLOUT_EPS) for ~2× faster rollouts at negligible accuracy cost.

    max_iter: OSQP iteration cap. 8000 for live use; 5000 in offline tuner.

    Parameters
    ----------
    x0 : np.ndarray, shape (8,)
        Current MPC state vector [e_y, e_y_dot, e_psi, e_psi_dot, e_v, 0,
        delta_act, a_act]. Built in simulation.py or offline_tuner.py
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
    u_prev : array-like, shape (2,), optional
        Previously applied control input. Used as anchor for the step-0
        rate cost. If None, zeros are assumed.
    silent : bool, optional
        If True, suppress OPTIMAL_INACCURATE warnings. Used by offline tuner
        where warning noise would flood the console during mass rollouts.
    return_status : bool, optional
        If True, return (u_sol, status) tuple instead of just u_sol.
        Used by offline_tuner.py to count OPTIMAL_INACCURATE occurrences.
    eps_abs, eps_rel : float, optional
        OSQP absolute and relative convergence tolerances.
    max_iter : int, optional
        OSQP maximum iteration count.
    warm_start : bool, optional
        Whether to warm-start OSQP from the previous solution.

    Returns
    -------
    u_sol : np.ndarray, shape (2,) or None
        Optimal first-step control action [delta_cmd, a_cmd] to apply.
        Returns None if both OSQP and Clarabel fail — caller should hold
        the previous command.
    (u_sol, status) if return_status=True.

    Called by: simulation.py (simulate_closed_loop),
               offline_tuner.py (run_headless_rollout)
    """
    global _mpc_cache

    nx, nu = 8, 2

    if R_rate is None:
        R_rate = np.zeros((nu, nu))
    if u_prev is None:
        u_prev = np.zeros(nu)

    # ── Build or rebuild cache if horizon N has changed ───────────────────────
    # In normal operation the cache is built once; N is fixed across the session.
    if _mpc_cache is None or _mpc_cache['u'].shape[1] != N:
        _mpc_cache = init_parameterized_mpc(nx, nu, N, u_min, u_max)

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
    # Pre-multiply u_prev by sqrtR_rate so the rate cost at step 0 is:
    # ||sqrtR_rate * u[:,0] - sqrtR_rate * u_prev||² = ||sqrtR_rate ⊙ Δu_0||²
    _mpc_cache['weighted_u_prev'].value = sqrtR_rate * np.asarray(u_prev)

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
            # OPTIMAL_INACCURATE: OSQP converged but not to full tolerance.
            # The solution is still usable at 20 Hz — better to use it than
            # to fall back to holding the previous command.
            print(f"[MPC] Warning: OSQP returned OPTIMAL_INACCURATE "
                  f"(consider tightening eps or checking weight magnitudes)")

        if status not in (cp.OPTIMAL, cp.OPTIMAL_INACCURATE):
            # OSQP failed outright (infeasible, unbounded, or numerical error).
            # Attempt Clarabel — a more robust but slower interior-point solver.
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