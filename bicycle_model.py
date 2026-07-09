"""
bicycle_model.py — 8-State Linear Discrete-Time Vehicle Model for MPC

PURPOSE
-------
Builds the linearised, discretised vehicle model that the MPC optimiser
(optimiser.py) uses for its horizon predictions. This is deliberately simpler
than the nonlinear plant (vehicle_physics.py): it is a linearised bicycle
model that can be solved as a convex QP. The gap between this model and the
real plant (model-plant mismatch) is what makes the closed-loop feedback
controller necessary.

HOW IT WORKS
------------
The model blends two linearisations based on vehicle speed:
  - Below ~1 m/s: kinematic bicycle model (tyre forces not developed yet;
    the car steers purely by geometry, like a shopping trolley)
  - Above ~2.5 m/s: dynamic bicycle model (tyre cornering forces dominate)
  - Between 1 and 2.5 m/s: a weighted blend of both

The continuous-time state matrix A_c and input matrix B_c are then
discretised using the Zero-Order Hold (ZOH) method via a matrix exponential
(scipy.linalg.expm). ZOH is exact for a linear system driven by a
piecewise-constant input — exactly how MPC applies commands — unlike Euler
discretisation which introduces O(dt²) error per step.

STATE VECTOR (8 states) — same as MPC's internal prediction horizon
---------------------------------------------------------------------------
  [0] e_y       Lateral deviation from path centreline (m)
  [1] e_y_dot   Rate of change of lateral deviation (m/s)
  [2] e_psi     Heading error relative to path tangent (rad)
  [3] e_psi_dot Yaw rate (rad/s)
  [4] e_v       Speed error: vx − v_target (m/s)
  [5] e_a       Unused acceleration error (held at zero, exists for consistency only)
  [6] delta_act Actual steering angle after first-order lag (rad)
  [7] a_act     Actual acceleration command after first-order lag (m/s²)

CONTROL INPUTS (2 inputs)
---------------------------------------------------------------------------
  [0] delta_cmd  Steering command sent to the actuator lag filter (rad)
  [1] a_cmd      Acceleration command sent to the actuator lag filter (m/s²)

USED BY
-------
  optimiser.py  — solve_mpc() calls get_8state_discrete_model() each step
                  to populate the QP's A and B matrices.
  offline_tuner.py — get_cached_model() wraps this to avoid redundant calls
                     during mass parallel rollouts.

DOES NOT USE
------------
  simulation.py, vehicle_physics.py, speed_profile.py, sim_track.py,
  performance_stats.py
"""

import numpy as np
from scipy.linalg import expm
import vehicle_physics as vp


def get_8state_discrete_model(v_x, dt):
    """
    Compute the 8-state discrete-time matrices (Ad, Bd) for a given
    longitudinal speed and timestep, using a kinematic-dynamic blend
    discretised by Zero-Order Hold.

    The output matrices satisfy the one-step prediction:
        x[k+1] = Ad @ x[k] + Bd @ u[k]

    These are passed directly to the QP in optimiser.py as the prediction
    model for the N-step horizon rollout.

    Parameters
    ----------
    v_x : float
        Current longitudinal vehicle speed (m/s). Used to:
          1. Compute speed-dependent dynamic model coefficients (cornering
             stiffness effects scale with 1/vx).
          2. Set the blending weight between kinematic and dynamic models.
    dt : float
        Discrete timestep (s). Must match the simulation timestep exactly
        (0.05 s at 20 Hz) so that the MPC's prediction horizon aligns with
        real elapsed time.

    Returns
    -------
    Ad : np.ndarray, shape (8, 8)
        Discrete-time state transition matrix.
    Bd : np.ndarray, shape (8, 2)
        Discrete-time input matrix.

    Notes on OSQP sparsity
    ----------------------
    OSQP (the QP solver used by optimiser.py) pre-analyses the sparsity pattern
    of the problem matrices on the first solve and caches it. If a subsequent
    solve presents a different sparsity pattern — e.g. a zero that was previously
    nonzero — OSQP throws a reallocation error. To prevent this, all matrices are
    initialised with epsilon (1e-12) rather than exact zeros, forcing a consistent
    "dense" sparsity pattern at every speed.

    Called by: optimiser.py (solve_mpc), offline_tuner.py (get_cached_model)
    """
    # Load vehicle parameters — same source of truth as the nonlinear plant
    vehicle_parameters = vp.VehicleParams()
    lf, lr = vehicle_parameters.lf, vehicle_parameters.lr     # Axle distances from CoM
    L  = lf + lr                                               # Total wheelbase
    m  = vehicle_parameters.m                                  # Vehicle mass (kg)
    Iz = vehicle_parameters.Iz                                 # Yaw inertia (kg·m²)
    Cf = vehicle_parameters.Cf                                 # Front cornering stiffness (N/rad)
    Cr = vehicle_parameters.Cr                                 # Rear  cornering stiffness (N/rad)
    tau_delta = vehicle_parameters.tau_delta                   # Steering lag time constant (s)
    tau_a     = vehicle_parameters.tau_a                       # Acceleration lag time constant (s)

    # Guard against exact zero speed — prevents 1/v_x divisions from blowing up
    v_x_safe = max(0.01, v_x)

    # ── Force dense matrices for OSQP sparsity consistency ──────────────────
    # All entries initialised to epsilon (1e-12) rather than 0.0 so that the
    # sparsity pattern (set of nonzero locations) is the same at every call,
    # regardless of speed. This prevents OSQP's cached factorisation from
    # becoming invalid when a previously nonzero entry rounds to zero.
    A_kin = np.ones((8, 8)) * 1e-12   # Kinematic model A matrix (blank canvas)
    A_dyn = np.ones((8, 8)) * 1e-12   # Dynamic model A matrix (blank canvas)
    B     = np.ones((8, 2)) * 1e-12   # Input matrix (shared by both models)

    # ── 1. KINEMATIC BICYCLE MODEL ────────────────────────────────────────────
    # Valid at low speeds where tyre slip angles are negligible. The car steers
    # like a rigid linkage: heading rate = vx * tan(delta) / L ≈ vx * delta / L.
    #
    # State equations (continuous time):
    #   ė_y       = v_x * e_psi   (lateral drift proportional to heading error and speed)
    #   ė_psi     = v_x * delta_act / L   (yaw rate from Ackermann steer geometry)
    #   ẋ_4       = e_v  (placeholder; e_v integrates through e_a)
    #   ẋ_5       = e_a  (placeholder; a_act integrates through command)
    #   δ̇_act     = −delta_act / tau_delta   (first-order lag toward commanded δ)
    #   ȧ_act     = −a_act / tau_a            (first-order lag toward commanded a)
    A_kin[0, 2] = v_x_safe           # ė_y = v_x * e_psi
    A_kin[2, 6] = v_x_safe / L       # ė_psi = v_x/L * delta_act  (Ackermann geometry)
    A_kin[4, 5] = 1.0                 # ė_v = e_a  (acceleration integrates to speed error)
    A_kin[5, 7] = 1.0                 # ė_a = a_act (unused but structurally present)
    A_kin[6, 6] = -1.0 / tau_delta   # First-order lag: dδ/dt = -δ/tau (self-decay)
    A_kin[7, 7] = -1.0 / tau_a       # First-order lag: da/dt = -a/tau (self-decay)

    # ── 2. DYNAMIC BICYCLE MODEL ──────────────────────────────────────────────
    # Valid at higher speeds where tyre lateral forces (cornering stiffness × slip
    # angle) dominate over geometric steering. The key difference: ė_y_dot and
    # ė_psi_dot have damping terms that depend on the cornering stiffnesses Cf, Cr
    # and their moment arms about the CoM.
    #
    # From the linearised bicycle model (derived by linearising Euler's equations
    # for planar rigid body motion with small slip angle approximation):
    #
    # ë_y = −(2Cf+2Cr)/(m*vx) * ė_y + (2Cf+2Cr)/m * e_psi
    #        + (−2Cf*lf+2Cr*lr)/(m*vx) * e_psi_dot + (2Cf)/m * delta_act
    #
    # ë_psi = (−2Cf*lf+2Cr*lr)/(Iz*vx) * ė_y + (2Cf*lf−2Cr*lr)/Iz * e_psi
    #          − (2Cf*lf²+2Cr*lr²)/(Iz*vx) * e_psi_dot + (2Cf*lf)/Iz * delta_act
    #
    # The 1/vx terms reflect that at higher speeds, each unit of lateral
    # velocity produces a smaller slip angle, so cornering forces build more slowly.
    A_dyn[0, 1] = 1.0                                              # ė_y = e_y_dot
    A_dyn[1, 1] = -(2*Cf + 2*Cr) / (m * v_x_safe)                # Lateral damping: cornering forces damp e_y_dot
    A_dyn[1, 2] = (2*Cf + 2*Cr) / m                               # Lateral restoring: heading error → lateral accel
    A_dyn[1, 3] = (-2*Cf*lf + 2*Cr*lr) / (m * v_x_safe)          # Coupling: yaw rate → lateral accel (understeer gradient)
    A_dyn[1, 6] = (2*Cf) / m                                       # Steering → lateral force: delta_act drives e_y_dot
    A_dyn[2, 3] = 1.0                                              # ė_psi = e_psi_dot (yaw rate is derivative of heading error)
    A_dyn[3, 1] = (-2*Cf*lf + 2*Cr*lr) / (Iz * v_x_safe)         # Coupling: lateral velocity → yaw moment
    A_dyn[3, 2] = (2*Cf*lf - 2*Cr*lr) / Iz                        # Yaw restoring: heading error → yaw moment
    A_dyn[3, 3] = -(2*Cf*lf**2 + 2*Cr*lr**2) / (Iz * v_x_safe)   # Yaw damping: yaw rate → opposing moment (both axles)
    A_dyn[3, 6] = (2*Cf * lf) / Iz                                 # Steering → yaw moment: front force × moment arm
    A_dyn[4, 5] = 1.0                                              # ė_v = e_a
    A_dyn[5, 7] = 1.0                                              # ė_a = a_act
    A_dyn[6, 6] = -1.0 / tau_delta                                 # Steering lag (same in both models)
    A_dyn[7, 7] = -1.0 / tau_a                                     # Acceleration lag (same in both models)

    # Input matrix B: shared by both kinematic and dynamic models.
    # The actuator lag ODEs take the commanded values as inputs:
    #   dδ/dt = (δ_cmd − δ_act) / tau_delta  →  B[6,0] = 1/tau_delta
    #   da/dt = (a_cmd  − a_act) / tau_a     →  B[7,1] = 1/tau_a
    B[6, 0] = 1.0 / tau_delta   # Steering command drives steering lag integrator
    B[7, 1] = 1.0 / tau_a       # Acceleration command drives acceleration lag integrator

    # ── 3. SPEED-BASED BLENDING ────────────────────────────────────────────────
    # alpha = 0 at v_x ≤ 1.0 m/s (pure kinematic)
    # alpha = 1 at v_x ≥ 2.5 m/s (pure dynamic)
    # Linear interpolation between: captures the transition region where
    # both geometry and tyre forces are significant.
    alpha = np.clip((v_x - 1.0) / (2.5 - 1.0), 0.0, 1.0)
    A_c   = (1.0 - alpha) * A_kin + alpha * A_dyn  # Blended continuous-time A
    B_c   = B                                        # B is identical for both models

    # ── 4. ZERO-ORDER HOLD (ZOH) DISCRETISATION ───────────────────────────────
    # ZOH exactly discretises a continuous LTI system for piecewise-constant inputs:
    #   Ad = exp(A_c * dt)
    #   Bd = A_c⁻¹ * (Ad − I) * B_c   [in general]
    #
    # The standard trick computes both simultaneously via the matrix exponential:
    #   exp([A_c  B_c] * dt) = [Ad  Bd]
    #       [0    0  ]         [0   I ]
    #
    # This avoids explicitly inverting A_c (which may be ill-conditioned) and
    # handles the case where A_c is singular gracefully via the expm algorithm.
    nx, nu = 8, 2
    M = np.zeros((nx + nu, nx + nu))  # Augmented matrix: (10×10)
    M[:nx, :nx] = A_c                  # Top-left: continuous A
    M[:nx, nx:] = B_c                  # Top-right: continuous B
    # Bottom rows remain zero — they produce the [0 I] block structure

    Md = expm(M * dt)        # Matrix exponential: key ZOH computation
    Ad = Md[:nx, :nx]        # Discrete A: top-left block of exp result
    Bd = Md[:nx, nx:]        # Discrete B: top-right block of exp result

    return Ad, Bd