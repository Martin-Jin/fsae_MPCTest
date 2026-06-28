"""
control_utils.py — MPC path-tracking controller for FSDS.

Replaces the Stanley controller with a linear time-varying MPC built on an
8-state bicycle model with first-order actuator lag.

  States  x : [e_y, e_yd, e_ψ, e_ψd, e_v, e_a, δ_act, a_act]
  Inputs  u : [δ_cmd (rad), a_cmd (m/s²)]

Sign convention (FSDS ENU — x forward, y left):
  e_y  > 0 → vehicle is to the LEFT  of the reference path
  e_ψ  > 0 → path heading is to the LEFT of the vehicle heading
  δ    > 0 → steer left   (front wheels deflect to port)
  FSDS steering > 0 → steer RIGHT (FSDS sign is opposite to δ)

Improvements over the standalone simulation MPC:
  • Zero-order hold (matrix-exponential) discretisation — more accurate than Euler,
    especially for the fast actuator states (τ ≈ 0.2 – 0.3 s).
  • Path curvature feedforward — adds the term –κ·v_x to the heading-error dynamics
    so the linearised model has a zero-error equilibrium on curved paths.
  • Input-rate constraints — penalises and bounds Δu per step, eliminating the
    left–right oscillation that pure CTE feedback is prone to.
  • Heavier terminal cost — provides an implicit Lyapunov bound on closed-loop
    stability without a terminal set constraint.
  • Combined longitudinal control — the MPC outputs a_cmd as well as δ_cmd, so
    no separate P-controller for speed is needed.
  • Graceful solver fallback — holds the previous command (not zero) on failure,
    which is safer at racing speed.

Dependencies: numpy, scipy, cvxpy (with OSQP backend).
"""

import math

import cvxpy as cp
import numpy as np
from scipy.linalg import expm

# FSDS: maximum physical steering deflection per ros-bridge.md
MAX_STEER_RAD: float = math.radians(25.0)


class MPCController:
    """
    Linear time-varying MPC for combined lateral and longitudinal path tracking.

    The model is re-linearised every control step around the current longitudinal
    speed v_x, making it an LTV-MPC (gain-scheduled).  The QP is small enough
    (≈ 200 variables) to solve in well under 10 ms with OSQP warm-starting,
    leaving comfortable headroom inside a 50 ms control period.

    Tuning quick-reference
    ----------------------
    Increase Q[0,0]   (e_y weight)    → tighter lateral tracking, may oscillate more.
    Increase Q[3,3]   (e_ψd weight)   → heavier yaw-rate damping, reduces sway.
    Increase R[0,0]   (δ_cmd effort)  → smoother steering, more cross-track error.
    Increase R_rate   (Δu penalty)    → smoother commands, slower response.
    Increase N        (horizon steps) → better preview, slower QP solve.
    """

    # ------------------------------------------------------------------
    # Construction
    # ------------------------------------------------------------------

    def __init__(
        self,
        dt: float = 0.05,
        N:  int   = 20,
    ) -> None:
        """
        Parameters
        ----------
        dt : Control period (s).  Must match the ROS2 timer period.
        N  : Prediction horizon length (steps).  N·dt seconds of preview.
             N = 20 gives 1 s at 20 Hz; keep N ≤ 30 for real-time operation.
        """
        self.dt = dt
        self.N  = N

        # ── Vehicle geometry & dynamics (Formula Student car) ──────────
        self.lf  = 0.9      # CoM → front axle (m)
        self.lr  = 0.7      # CoM → rear  axle (m) - Total wheelbase ~1.6m
        self.m   = 280.0    # Real FSDS vehicle mass profile (kg)
        self.Iz  = 140.0    # Adjusted yaw inertia for light formula car (kg·m²)
        self.Cf  = 28000.0  # Front cornering stiffness (N/rad)
        self.Cr  = 32000.0  # Rear cornering stiffness (N/rad)

        # First-order actuator time constants (s)
        self.tau_delta = 0.15  # FSDS steering actuators are highly responsive
        self.tau_a     = 0.10

        self.nx = 8
        self.nu = 2

        # ── Cost weight matrices ───────────────────────────────────────
        # State order: [e_y, e_yd, e_ψ, e_ψd, e_v, e_a, δ_act, a_act]
        # Title: Tuned MPC Weights for Formula Student track drive
        # State tracking weights: [e_y, e_yd, e_ψ, e_ψd, e_v, e_a, δ_act, a_act]
        self.Q = np.diag([
            15.0,   # e_y   - Lowered to match lighter mass framework
            1.0,    # e_yd  
            12.0,   # e_ψ   
            2.0,    # e_ψd  
            8.0,    # e_v   
            0.1,    # e_a   
            0.1,    # δ_act 
            0.1,    # a_act 
        ])
        self.Q_terminal = 2.5 * self.Q

        # Absolute control efforts
        self.R = np.diag([
            40.0,   # δ_cmd
            2.0,    # a_cmd
        ])

        # Slew rate limits (change in input per time-step)
        self.R_rate = np.diag([
            150.0,  # Δδ_cmd - Provides smooth steering actuation
            5.0,    # Δa_cmd
        ])

        # ── Hard actuator limits ───────────────────────────────────────
        self.u_min  = np.array([-MAX_STEER_RAD, -6.0])  # Cap braking deceleration
        self.u_max  = np.array([ MAX_STEER_RAD,  4.0])  # Cap acceleration

        # Per-step rate limits  (positive bound applied symmetrically)
        self.du_max = np.array([math.radians(8.0), 0.8])   # [rad, m/s²]

        # ── Continuity memory (updated every solve) ────────────────────
        self._delta_act: float     = 0.0           # Estimated actuator steer angle (rad)
        self._a_act:     float     = 0.0           # Estimated actuator acceleration (m/s²)
        self._u_prev:    np.ndarray = np.zeros(self.nu)   # Last applied command

    # ------------------------------------------------------------------
    # Discrete-time model  (ZOH via matrix exponential)
    # ------------------------------------------------------------------

    def _discrete_model(
        self,
        v_x:   float,
        kappa: float,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Zero-order-hold discretisation of the linearised bicycle + actuator model.

        Parameters
        ----------
        v_x   : Longitudinal speed (m/s).  Clipped to ≥ 0.5 m/s for stability.
        kappa : Local path curvature (1/m), positive = left-hand turn.
                Enters as a constant disturbance on the heading-error derivative
                so that straight-ahead driving on a curve is the zero-cost solution.

        Returns
        -------
        Ad : (8, 8)  Discrete-time state matrix
        Bd : (8, 2)  Discrete-time input matrix
        dd : (8,)    Discrete-time curvature disturbance vector
        """
        v_x = max(0.5, abs(v_x))
        m, Iz, lf, lr = self.m, self.Iz, self.lf, self.lr
        Cf, Cr         = self.Cf, self.Cr
        td, ta, dt     = self.tau_delta, self.tau_a, self.dt

        # ── Continuous A_c  (8 × 8) ───────────────────────────────────
        A_c = np.zeros((self.nx, self.nx))

        # Lateral / yaw bicycle dynamics (linearised Rajamani bicycle model)
        A_c[0, 1] = 1.0                                         # ė_y  = e_yd
        A_c[1, 1] = -(2*Cf + 2*Cr)        / (m * v_x)          # e_yd from lateral vel
        A_c[1, 2] =  (2*Cf + 2*Cr)        /  m                  # e_yd from heading err
        A_c[1, 3] = (-2*Cf*lf + 2*Cr*lr)  / (m * v_x)          # e_yd from yaw rate
        A_c[1, 6] =  (2*Cf)               /  m                  # e_yd from front steer
        A_c[2, 3] = 1.0                                         # ė_ψ  = e_ψd
        A_c[3, 1] = (-2*Cf*lf + 2*Cr*lr)  / (Iz * v_x)         # e_ψd from lateral vel
        A_c[3, 2] =  (2*Cf*lf - 2*Cr*lr)  /  Iz                 # e_ψd from heading err
        A_c[3, 3] = -(2*Cf*lf**2 + 2*Cr*lr**2) / (Iz * v_x)    # e_ψd from yaw rate
        A_c[3, 6] =  (2*Cf * lf)          /  Iz                 # e_ψd from front steer

        # Longitudinal integrator chain: e_v → e_a → a_act
        A_c[4, 5] = -1.0    # ė_v = e_a
        A_c[5, 7] = 1.0    # ė_a = a_act

        # First-order actuator models
        A_c[6, 6] = -1.0 / td   # δ_act decays toward δ_cmd
        A_c[7, 7] = -1.0 / ta   # a_act decays toward a_cmd

        # ── Continuous B_c  (8 × 2) ───────────────────────────────────
        B_c = np.zeros((self.nx, self.nu))
        B_c[6, 0] = 1.0 / td   # δ_cmd drives δ_act
        B_c[7, 1] = 1.0 / ta   # a_cmd drives a_act

        # ── Curvature feedforward disturbance vector ───────────────────
        # Continuous heading-error dynamics:
        #   ė_ψ = e_ψd − κ·v_x
        # Without this term the model assumes a straight reference path, causing
        # a persistent heading error on corners.  We absorb it into a constant
        # disturbance d_c and discretise it alongside A_c and B_c.
        d_c = np.zeros(self.nx)
        d_c[2] = -kappa * v_x

        # ── Zero-order hold via augmented matrix exponential ──────────
        #
        # Augment the state with u and d treated as piecewise-constant over [0, dt]:
        #
        #   d/dt [x]   [A_c  B_c  d_c] [x]
        #        [u] = [ 0    0    0  ] [u]
        #        [1]   [ 0    0    0  ] [1]
        #
        # Then expm of this (times dt) gives the exact ZOH solution:
        #   x(k+1) = Ad·x(k) + Bd·u(k) + dd
        n_aug = self.nx + self.nu + 1
        M = np.zeros((n_aug, n_aug))
        M[:self.nx, :self.nx]                    = A_c
        M[:self.nx,  self.nx:self.nx + self.nu]  = B_c
        M[:self.nx, -1]                          = d_c

        eM = expm(M * dt)   # (11 × 11) matrix exponential — very fast

        Ad = eM[:self.nx, :self.nx]
        Bd = eM[:self.nx,  self.nx:self.nx + self.nu]
        dd = eM[:self.nx, -1]

        return Ad, Bd, dd

    # ------------------------------------------------------------------
    # State extraction
    # ------------------------------------------------------------------

    def _error_state(
        self,
        path:          np.ndarray,
        car_pos:       np.ndarray,
        car_yaw:       float,
        car_speed:     float,
        car_yaw_rate:  float,
        desired_speed: float,
    ) -> tuple[np.ndarray, float]:
        """
        Project odometry and path geometry into the 8-dimensional error state.

        Convention: e_y > 0 when the vehicle is to the LEFT of the path.
        (This is the LEFT-positive convention of the Rajamani bicycle model.)
        The Stanley helper used RIGHT-positive, so we flip the sign of the
        raw cross-track dot product.

        Returns
        -------
        x0    : (8,) initial MPC state for this step
        kappa : Local curvature estimate (1/m) for the disturbance feedforward
        """
        # Control point: front axle
        fa = car_pos + self.lf * np.array([math.cos(car_yaw), math.sin(car_yaw)])

        # Nearest waypoint index (to front axle)
        dists = np.linalg.norm(path - fa, axis=1)
        idx   = int(np.argmin(dists))

        # Path tangent at nearest point
        if idx < len(path) - 1:
            seg = path[idx + 1] - path[idx]
        else:
            seg = path[idx] - path[idx - 1]
        seg_len = float(np.linalg.norm(seg))
        if seg_len < 1e-6:
            return np.zeros(self.nx), 0.0

        t       = seg / seg_len                      # unit tangent
        right_n = np.array([t[1], -t[0]])            # 90° CW = right-hand normal

        # ── Heading error (path_yaw − car_yaw), positive = path turns LEFT ──
        path_yaw = math.atan2(t[1], t[0])
        e_psi = math.atan2(
            math.sin(path_yaw - car_yaw),
            math.cos(path_yaw - car_yaw),
        )

        # ── Cross-track error  (LEFT-positive) ────────────────────────
        # dot(fa − nearest, right_n) > 0 means car is RIGHT of path → negate for model
        e_y = -float(np.dot(fa - path[idx], right_n))

        # ── Lateral velocity approximation ────────────────────────────
        # v_y ≈ v_x · sin(e_ψ)  (kinematic small-angle approximation)
        e_yd = car_speed * math.sin(e_psi)

        # ── Local curvature ───────────────────────────────────────────
        kappa = _curvature(path, idx)

        # ── Heading error rate ────────────────────────────────────────
        # ė_ψ = ω − κ·v_x   (yaw rate corrected for path curvature)
        e_psi_d = car_yaw_rate - kappa * car_speed

        # ── Speed error (positive = car too slow) ─────────────────────
        e_v = desired_speed - car_speed

        # Build 8-state vector; actuator states carried from last solve
        x0 = np.array([
            e_y,
            e_yd,
            e_psi,
            e_psi_d,
            e_v,
            0.0,              # e_a  — not directly observable; reset each step
            self._delta_act,  # δ_act — continuity from previous step
            self._a_act,      # a_act — continuity from previous step
        ])
        return x0, kappa

    # ------------------------------------------------------------------
    # QP solver
    # ------------------------------------------------------------------

    def _solve_qp(
        self,
        x0: np.ndarray,
        Ad: np.ndarray,
        Bd: np.ndarray,
        dd: np.ndarray,
    ) -> np.ndarray:
        """
        Build and solve the finite-horizon QP.

        Returns u[:, 0] — the first optimal control action.
        On solver failure, returns the previous command (safe fallback).
        """
        nx, nu, N = self.nx, self.nu, self.N

        x = cp.Variable((nx, N + 1))
        u = cp.Variable((nu, N))

        cost        = 0.0
        constraints = [x[:, 0] == x0]

        for k in range(N):
            # Stage cost: J += x'Qx + u'Ru
            cost += cp.quad_form(x[:, k], self.Q)
            cost += cp.quad_form(u[:, k], self.R)

            # Dynamics: x(k+1) = Ad·x(k) + Bd·u(k) + dd
            constraints += [x[:, k + 1] == Ad @ x[:, k] + Bd @ u[:, k] + dd]

            # Hard actuator limits
            constraints += [u[:, k] >= self.u_min, u[:, k] <= self.u_max]

            # Input-rate penalty and hard rate limits
            u_prev_k = self._u_prev if k == 0 else u[:, k - 1]
            du = u[:, k] - u_prev_k
            cost        += cp.quad_form(du, self.R_rate)
            constraints += [du >= -self.du_max, du <= self.du_max]

        # Terminal cost (heavier than stage cost — implicit stability weight)
        cost += cp.quad_form(x[:, N], self.Q_terminal)

        prob = cp.Problem(cp.Minimize(cost), constraints)
        prob.solve(
            solver=cp.OSQP,
            verbose=False,
            warm_start=True,
            eps_abs=1e-3,       # Relax absolute tolerance from default 1e-4
            eps_rel=1e-3,       # Relax relative tolerance from default 1e-4
            max_iter=4000       # Allow more iterations to ensure clean convergence
        )

        if (prob.status in (cp.OPTIMAL, cp.OPTIMAL_INACCURATE)
                and u[:, 0].value is not None):
            return u[:, 0].value

        # Safe fallback: hold last steer, coast (zero accel)
        return np.array([self._u_prev[0], 0.0])

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def compute(
        self,
        path:          np.ndarray,
        car_pos:       np.ndarray,
        car_yaw:       float,
        car_speed:     float,
        desired_speed: float,
        car_yaw_rate:  float = 0.0,
    ) -> tuple[float, float, float]:
        """
        Compute FSDS control commands via MPC.

        Parameters
        ----------
        path          : (N_wp, 2) planned waypoints in map frame (needs ≥ 2 points)
        car_pos       : (2,)  vehicle position in map frame
        car_yaw       : Vehicle heading (rad)
        car_speed     : Longitudinal speed (m/s)
        desired_speed : Reference speed  (m/s)
        car_yaw_rate  : Yaw rate (rad/s), positive = CCW / left turn

        Returns
        -------
        steering : float ∈ [-1, 1]   positive = steer RIGHT  (FSDS convention)
        throttle : float ∈  [0, 1]
        brake    : float ∈  [0, 1]
        """
        if len(path) < 2:
            return 0.0, 0.0, 0.5    # gentle brake if no path yet

        # ── Step 1: extract error state ───────────────────────────────
        x0, kappa = self._error_state(
            path, car_pos, car_yaw, car_speed, car_yaw_rate, desired_speed,
        )

        # ── Step 2: linearise + discretise model at current speed ─────
        Ad, Bd, dd = self._discrete_model(max(car_speed, 1.0), kappa)

        # ── Step 3: solve MPC QP ──────────────────────────────────────
        u_opt = self._solve_qp(x0, Ad, Bd, dd)

        # ── Step 4: propagate actuator states for next step (Euler) ───
        self._delta_act += (self.dt / self.tau_delta) * (u_opt[0] - self._delta_act)
        self._a_act     += (self.dt / self.tau_a)     * (u_opt[1] - self._a_act)
        self._u_prev     = u_opt

        # ── Step 5: convert to FSDS commands ─────────────────────────
        delta_cmd = float(np.clip(u_opt[0], -MAX_STEER_RAD, MAX_STEER_RAD))
        a_cmd     = float(u_opt[1])

        # FSDS sign: positive δ (steer left) → FSDS negative steering value
        # Title: Update steering direction normalization in control_utils.py
        steering = float(np.clip(-delta_cmd / MAX_STEER_RAD, -1.0, 1.0))

        # Split signed acceleration into throttle / brake
        # Calibration: 5.0 m/s² ≡ full throttle,  4.0 m/s² ≡ full brake
        if a_cmd >= 0.0:
            throttle = float(np.clip(a_cmd / 5.0, 0.0, 1.0))
            brake    = 0.0
        else:
            throttle = 0.0
            brake    = float(np.clip(-a_cmd / 4.0, 0.0, 1.0))

        return steering, throttle, brake

    def reset(self) -> None:
        """
        Reset all internal state.
        Call after large discontinuities (e.g. car teleportation, warm-up phase).
        """
        self._delta_act = 0.0
        self._a_act     = 0.0
        self._u_prev    = np.zeros(self.nu)


# ------------------------------------------------------------------
# Module-level helpers
# ------------------------------------------------------------------

def _curvature(path: np.ndarray, idx: int) -> float:
    """
    Estimate signed curvature (1/m) at waypoint idx using the finite-difference
    of the path heading angle.  Positive = left-hand turn.
    Returns 0 for boundary indices.
    """
    if idx <= 0 or idx >= len(path) - 1:
        return 0.0
    s_prev = path[idx]     - path[idx - 1]
    s_next = path[idx + 1] - path[idx]
    yaw_p  = math.atan2(s_prev[1], s_prev[0])
    yaw_n  = math.atan2(s_next[1], s_next[0])
    dpsi   = math.atan2(math.sin(yaw_n - yaw_p), math.cos(yaw_n - yaw_p))
    ds     = (np.linalg.norm(s_prev) + np.linalg.norm(s_next)) * 0.5
    return dpsi / ds if ds > 1e-6 else 0.0


# import math

# import numpy as np

# # FSDS: max steering angle 25 degrees per ros-bridge.md
# MAX_STEER_RAD = math.radians(25.0)


# def _heading_error(car_pos, car_yaw, target_global) -> float:
#     """Return heading error in radians: positive when target is left of car."""
#     dx = target_global[0] - car_pos[0]
#     dy = target_global[1] - car_pos[1]
#     cos_y, sin_y = math.cos(car_yaw), math.sin(car_yaw)
#     x_car =  dx * cos_y + dy * sin_y
#     y_car = -dx * sin_y + dy * cos_y
#     return math.atan2(y_car, x_car)


# def compute_steering(car_pos, car_yaw, target_global) -> float:
#     """Pure-proportional steering (legacy helper, kept for reference)."""
#     return float(np.clip(-_heading_error(car_pos, car_yaw, target_global)
#                          / MAX_STEER_RAD, -1.0, 1.0))


# class StanleyController:
#     """
#     Stanley path-tracking steering controller (Thrun et al., DARPA 2005).

#     δ = θ_e + arctan(k_cte · e / (v + k_soft))

#     θ_e — heading error: path tangent angle minus car yaw (rad).
#           Positive when path turns left relative to the car.
#     e   — cross-track error: signed lateral distance from the front axle
#           to the nearest path point (m), positive when the axle is to
#           the RIGHT of the path.
#     v   — car speed (m/s); k_soft prevents division by zero at standstill.

#     Sign convention (FSDS ENU: x forward, y left):
#       output > 0  → steer right
#       output < 0  → steer left
#       output ∈ [-1, 1]

#     Tuning:
#       k_cte     — cross-track gain.  Higher values correct lateral error faster
#                   but cause oscillation on a high-speed straight.
#       k_soft    — speed softening (m/s).  Set to ~walking speed so the CTE
#                   term doesn't saturate the steering at low speeds.
#       k_d       — yaw-rate damper gain.  Subtracts k_d·ω from the Stanley
#                   angle before normalising, opposing rapid heading changes.
#                   This is the primary fix for left-right sway: the CTE term
#                   alone has no memory of how fast the heading is already
#                   changing, so it overshoots; k_d·ω counters each swing.
#       wheelbase — distance from rear to front axle (m).  Used to project
#                   the control point to the front axle, which is where Stanley
#                   measures cross-track error.
#     """

#     def __init__(
#         self,
#         k_cte: float = 1.0,
#         k_soft: float = 1.0,
#         k_d: float = 0.1,
#         wheelbase: float = 1.5,
#     ):
#         self.k_cte     = k_cte
#         self.k_soft    = k_soft
#         self.k_d       = k_d
#         self.wheelbase = wheelbase

#     def compute(
#         self,
#         path: np.ndarray,
#         car_pos: np.ndarray,
#         car_yaw: float,
#         car_speed: float,
#         car_yaw_rate: float = 0.0,
#     ) -> float:
#         """
#         Return a steering command in [-1, 1].

#         path         — (N, 2) array of waypoints in map frame (must have N ≥ 2)
#         car_pos      — (2,) car position in map frame
#         car_yaw      — car heading in radians
#         car_speed    — car speed in m/s
#         car_yaw_rate — yaw rate in rad/s (positive = left / CCW); used by the
#                        damper term to oppose rapid heading changes
#         """
#         if len(path) < 2:
#             return 0.0

#         # Project control point to front axle
#         fa = car_pos + self.wheelbase * np.array([math.cos(car_yaw), math.sin(car_yaw)])

#         # Nearest waypoint index to front axle
#         idx = int(np.argmin(np.linalg.norm(path - fa, axis=1)))

#         # Unit tangent in direction of travel at that waypoint
#         if idx < len(path) - 1:
#             seg = path[idx + 1] - path[idx]
#         else:
#             seg = path[idx] - path[idx - 1]
#         seg_len = float(np.linalg.norm(seg))
#         if seg_len < 1e-6:
#             return 0.0
#         t = seg / seg_len

#         # Heading error: path_yaw - car_yaw, normalised to (-π, π)
#         path_yaw = math.atan2(t[1], t[0])
#         theta_e = math.atan2(
#             math.sin(path_yaw - car_yaw),
#             math.cos(path_yaw - car_yaw),
#         )

#         # Cross-track error: right-normal of path, positive = axle right of path
#         right_n = np.array([t[1], -t[0]])   # 90° CW rotation of tangent
#         e = float(np.dot(fa - path[idx], right_n))

#         # Stanley angle — positive in standard convention = left turn = FSDS negative.
#         # Damper subtracts k_d·ω: when the car is already swinging left (ω > 0),
#         # this reduces δ so the next tick steers less left, preventing overshoot.
#         delta = (theta_e
#                  + math.atan2(self.k_cte * e, car_speed + self.k_soft)
#                  - self.k_d * car_yaw_rate)

#         return float(np.clip(-delta / MAX_STEER_RAD, -1.0, 1.0))