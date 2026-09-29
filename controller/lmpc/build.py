"""
controller/lmpc/build.py — builds the parameterized MPC QP problem
(the linear time-varying MPC, "LMPC" in the docs)

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
  controller/lmpc/solve.py — solve_mpc() calls this to (re)build the cached
    problem on the first call and whenever N/u_min/u_max/du_max/terminal_scale
    change; not called directly by gui/simulation.py or tuner/offline_tuner.py.

DOES NOT USE
------------
  model/vehicle_physics.py (directly), model/bicycle_model.py (receives Ad/Bd as arguments),
  sim/speed_profile.py, sim/sim_track.py, tuner/performance_stats.py
"""

import cvxpy as cp
import numpy as np


def init_parameterized_mpc(nx, nu, N, u_min, u_max, du_max=None, terminal_scale=1.0):
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
    A_param            : (nx, nx)  Discrete-time A matrix from model/bicycle_model.py
    B_param            : (nx, nu)  Discrete-time B matrix from model/bicycle_model.py
    x0_param           : (nx,)     Current state (MPC initial condition)
    sqrtQ_param        : (nx, 1)   Element-wise sqrt of diagonal Q weights
    sqrtR_param        : (nu, 1)   Element-wise sqrt of diagonal R weights
    sqrtR_rate_param   : (nu, 1)   Element-wise sqrt of diagonal R_rate weights
    weighted_u_prev    : (nu,)     sqrtR_rate * u_prev (for rate cost at first step)
    u_prev_param       : (nu,)     raw u_prev (for the step-0 hard slew constraint)

    Parameters
    ----------
    nx : int      State dimension (always 8 in this system)
    nu : int      Input dimension (always 2: [delta_cmd, a_cmd])
    N  : int      Prediction horizon (number of steps)
    u_min : array-like, shape (nu,)   Lower input bounds
    u_max : array-like, shape (nu,)   Upper input bounds
    du_max : array-like, shape (nu,), optional
        Hard per-step slew-rate limit on u, including step 0 against u_prev.
        None disables the constraint (legacy behaviour). Must match the live
        mpc_core.py du_max for offline-tuned weights to transfer.
    terminal_scale : float, optional
        Extra multiplier on the state cost applied ONLY to the terminal state
        x[:,N], on top of the existing (unscaled) per-step cost it already
        gets from the uniform sum over all N+1 columns. 1.0 (default) is a
        no-op: no terminal weighting at all. This closes a structural gap:
        with no terminal cost or constraint, the MPC has no reason to prefer
        trajectories that leave it in a good position at the end of the
        horizon, which can show up as myopic behaviour right at the horizon
        boundary. >1.0 adds weight; NOT the same knob as settings.Q_diag,
        which scales every step equally including this one.

    Returns
    -------
    dict of cp.Problem/Parameter/Variable handles — see the inline comments
    at each cp.Parameter(...)/cp.Variable(...) declaration below for what
    each one is.

    Called by: solve_mpc() — on first call or when N changes
    """
    # ── CVXPY Parameters (updated each solve, not recompiled) ────────────────
    A_param = cp.Parameter((nx, nx))            # Discrete-time A from model/bicycle_model.py
    B_param = cp.Parameter((nx, nu))            # Discrete-time B from model/bicycle_model.py
    x0_param = cp.Parameter(nx)                 # Current MPC state

    # (nx,1) and (nu,1) shapes allow broadcasting across N horizon columns
    sqrtQ_param      = cp.Parameter((nx, 1), nonneg=True)   # State weight sqrt
    sqrtR_param      = cp.Parameter((nu, 1), nonneg=True)   # Input weight sqrt
    sqrtR_rate_param = cp.Parameter((nu, 1), nonneg=True)   # Rate weight sqrt
    weighted_u_prev_param = cp.Parameter(nu)   # sqrtR_rate * u_prev for step-0 rate cost
    u_prev_param = cp.Parameter(nu)            # raw u_prev, for the step-0 hard slew constraint
    # u[1,:] (a_cmd) effort cost is split by sign instead of going through
    # sqrtR_param[1] -- see the cost-function comment below for why.
    # sqrtR_param[1] is left unused (only row 0, delta_cmd, is read from it).
    r_a_accel_param = cp.Parameter(nonneg=True)
    r_a_brake_param = cp.Parameter(nonneg=True)

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

    # Terminal cost: EXTRA weight on the final predicted state x[:,N], on top
    # of the per-step weight it already receives above. terminal_scale=1.0
    # (default) makes this term's coefficient zero -- a pure no-op, i.e.
    # every column including the terminal one is weighted identically.
    # See init_parameterized_mpc's docstring.
    if terminal_scale != 1.0:
        cost += (terminal_scale - 1.0) * cp.sum_squares(
            cp.multiply(sqrtQ_param[:, 0], x[:, N])
        )

    # Input magnitude cost. delta_cmd (row 0) uses the shared sqrtR_param
    # path. a_cmd (row 1) is split by sign: cp.pos(u)^2 and cp.neg(u)^2 are
    # each individually convex (composition of the convex increasing
    # `square` with convex pos/neg), so summing them with independent
    # weights is still DCP, and pos(x)^2+neg(x)^2 == x^2 identically
    # (exactly one of pos/neg is nonzero for any real x) -- so
    # r_a_accel==r_a_brake reproduces the old single-R[1,1] cost
    # bit-for-bit. This lets braking be tuned independently from
    # acceleration without new QP variables or constraints: see
    # `docs/reference/control_mechanisms.md`'s "Accel/brake effort weight split".
    cost += cp.sum_squares(cp.multiply(sqrtR_param[0, 0], u[0, :]))
    cost += r_a_accel_param * cp.sum_squares(cp.pos(u[1, :]))
    cost += r_a_brake_param * cp.sum_squares(cp.neg(u[1, :]))

    # Slack penalty: quadratic to keep the corridor soft but strongly penalised.
    cost += W_slack * cp.sum_squares(slack)

    # Rate-of-change cost (step 0 vs. last applied input):
    # ||sqrtR_rate * u[:,0] - sqrtR_rate * u_prev||²
    # = ||sqrtR_rate ⊙ u[:,0] - weighted_u_prev||²
    # weighted_u_prev_param = sqrtR_rate * u_prev is pre-computed in solve_mpc().
    cost += cp.sum_squares(
        cp.multiply(sqrtR_rate_param[:, 0], u[:, 0]) - weighted_u_prev_param
    )

    # Rate-of-change cost (subsequent steps): Δu = u[:,i+1] - u[:,i], vectorized
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

    # Hard per-step slew-rate limit on [delta_cmd, a_cmd].
    #
    # PARITY: this constraint must exist here too, matching live mpc_core.py,
    # or the offline tuner would be optimising against a plant that can
    # change steering arbitrarily fast while the real car is clamped —
    # weights tuned here would not transfer faithfully. Mirrors mpc_core.py's
    # du_max (see that file for how the 180 deg/s figure was measured).
    #
    # Step-0 constraint against u_prev closes a second parity gap:
    # mpc_core.py hard-constrains `u[:,0] - uprev_p` (its own
    # separate raw-u_prev Parameter, not the sqrtR_rate-weighted one used in
    # the cost), so live can never jump more than du_max from the last
    # applied command on the very first predicted step. A SOFT-only penalty
    # via weighted_u_prev in the rate cost is not equivalent: a large
    # tracking-error gradient could still push u[:,0] arbitrarily far from
    # u_prev if the cost tradeoff favoured it. u_prev_param carries the
    # unweighted value for exactly this purpose, matching the hard constraint.
    if du_max is not None:
        du0 = u[:, 0] - u_prev_param
        constraints += [
            du0 <=  np.array(du_max),
            du0 >= -np.array(du_max),
        ]
    if du_max is not None and N > 1:
        du_hard = cp.diff(u, axis=1)
        constraints += [
            du_hard <=  np.array(du_max)[:, None],
            du_hard >= -np.array(du_max)[:, None],
        ]

    prob = cp.Problem(cp.Minimize(cost), constraints)

    return {
        'prob': prob, 'A': A_param, 'B': B_param, 'x0': x0_param,
        'sqrtQ': sqrtQ_param, 'sqrtR': sqrtR_param,
        'sqrtR_rate': sqrtR_rate_param,
        'r_a_accel': r_a_accel_param, 'r_a_brake': r_a_brake_param,
        'weighted_u_prev': weighted_u_prev_param,
        'u_prev': u_prev_param,
        'u': u,
        # u_min/u_max are baked into the constraints above as plain numpy
        # constants (not cp.Parameters), so they can't be updated later the
        # way Q/R/R_rate can. Stashing the values the cache was built with so
        # solve_mpc() can detect a caller requesting different bounds and
        # rebuild, instead of silently keeping stale bounds.
        'u_min': np.array(u_min, dtype=float),
        'u_max': np.array(u_max, dtype=float),
        'du_max': None if du_max is None else np.array(du_max, dtype=float),
        # terminal_scale is baked into the cost as a plain constant (like
        # u_min/u_max/du_max above), so it needs the same staleness check.
        'terminal_scale': float(terminal_scale),
    }
