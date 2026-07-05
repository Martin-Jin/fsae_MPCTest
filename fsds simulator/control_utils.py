# Title: control_utils.py

"""
control_utils.py — MPC path-tracking controller for FSDS.

Linear time-varying MPC built on an 8-state bicycle model with first-order
actuator lag.

  States  x : [e_y, e_yd, e_ψ, e_ψd, e_v, e_a, δ_act, a_act]
  Inputs  u : [δ_cmd (rad), a_cmd (m/s²)]

Sign convention (FSDS ENU — x forward, y left):
  e_y  > 0 → vehicle is to the LEFT  of the reference path
  e_ψ  > 0 → vehicle heading is to the LEFT of path heading
             (atan2(sin(yaw_car - yaw_path), cos(...)) > 0)
  δ    > 0 → steer left   (front wheels deflect to port)
  FSDS steering > 0 → steer RIGHT (FSDS sign is opposite to δ)

Dependencies: numpy, scipy, cvxpy (with OSQP + Clarabel backends).
"""

import math

import cvxpy as cp
import numpy as np
from scipy.linalg import expm

# FSDS: maximum physical steering deflection aligned with simulation.py bounds
MAX_STEER_RAD: float = math.radians(35.0)


# ---------------------------------------------------------------------------
# Adaptive gain helpers  (mirrors tuner.py so offline weights transfer cleanly)
# ---------------------------------------------------------------------------

def _adaptive_R_scaling(vx: float, R_base: np.ndarray) -> np.ndarray:
    """
    Speed-dependent steering cost with a saturating (Michaelis-Menten) scale.
    Dynamically scales the cost of control actions based on current velocity.

    steer_scale = 1 + (1.5 * vx) / (6.0 + vx)
        vx=0  → 1.0x  (baseline)
        vx=6  → 1.75x (halfway to asymptote)
        vx=10 → ~2.0x (approaching 2.5x asymptote)
    """
    vx = max(vx, 0.5)
    steer_scale = 1.0 + (1.5 * vx) / (6.0 + vx)
    accel_scale = 1.0 + 0.05 * vx
    R = R_base.copy()
    R[0, 0] *= steer_scale
    R[1, 1] *= accel_scale
    return R


def _adaptive_R_rate(kappa: float, R_rate_base: np.ndarray) -> np.ndarray:
    """
    Curvature-dependent steering-jerk softening.
    Reduces the penalty on steering slew rates in sharp corners to prevent
    understeering due to excessive damping.
    """
    scale = max(0.35, 1.0 / (1.0 + 3.0 * abs(kappa)))
    R = R_rate_base.copy()
    R[0, 0] *= scale
    # R[1, 1] unchanged
    return R


def _curvature(path: np.ndarray, idx: int) -> float:
    """
    Estimate signed path curvature (1/m) at waypoint idx via finite-difference
    of the heading angle. Positive = left-hand turn. Returns 0 at boundaries.
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


# ---------------------------------------------------------------------------
# MPC Controller
# ---------------------------------------------------------------------------

class MPCController:
    """
    Linear time-varying MPC for combined lateral and longitudinal path tracking.
    """
    # ------------------------------------------------------------------
    # Construction
    # ------------------------------------------------------------------

    def __init__(
        self,
        dt: float = 0.05, 
        N:  int   = 25, 
    ) -> None:
        """
        Initializes the MPC controller with physical vehicle parameters,
        cost weight matrices, and hardware limits.

        Parameters:
        dt: Control period (s) — must match the ROS 2 timer period.
        N:  Prediction horizon (steps). N=25 gives 1.25 s preview at 20 Hz.
        """
        self.dt = dt
        self.N  = N

        # ── Vehicle geometry & dynamics (Matched to model.py) ─────
        self.lf = 0.85   # CoM -> front axle (m)
        self.lr = 0.70   # CoM -> rear  axle (m)
        self.m  = 255.0  # Vehicle mass (kg)
        self.Iz = 110.0  # Yaw inertia (kg m^2)
        self.Cf = 11500.0  # Front cornering stiffness (N/rad)
        self.Cr = 12500.0  # Rear  cornering stiffness (N/rad)

        # First-order actuator time constants (s)
        self.tau_delta = 0.08  # Steering lag
        self.tau_a     = 0.05  # Throttle/brake lag

        self.nx = 8
        self.nu = 2

        # For tuning copy and paste purposes
        Q_diag      = [0.8272234497643787, 0.12414925328330728, 4.4135201967414694, 0.44809502633366416, 0.6767584308766955, 0.0, 0.0, 0.0]
        R_diag      = [5.109150728883122, 5.752126672173076]
        R_rate_diag = [6.482078562228737, 1.844043028681526]

        # ── Cost weight matrices (Matched to simulation.py tuner defaults) ───
        # State order: [e_y, e_yd, e_psi, e_psi_d, e_v, e_a, delta_act, a_act]
        self.Q = np.diag(Q_diag)

        # Terminal cost: 3x running cost for implicit Lyapunov stability.
        self.Q_terminal = 3.0 * self.Q

        # Absolute control-effort weights
        self.R = np.diag(R_diag)

        # Slew-rate penalty weights (change per time-step).
        self.R_rate = np.diag(R_rate_diag)

        # ── Hard actuator limits ───────────────────────────────────────
        self.u_min = np.array([-MAX_STEER_RAD, -6.0])  # [rad, m/s²]; matches VehicleParams.max_accel_brake
        self.u_max = np.array([ MAX_STEER_RAD, 12.0])  # matches VehicleParams.max_accel

        # Per-step rate limits (symmetric)
        self.du_max = np.array([math.radians(4.0), 0.6])  # [rad/step, m/s^2/step]

        # ── Continuity memory ─────────────────────────────────────────
        self._delta_act:      float      = 0.0
        self._a_act:          float      = 0.0
        self._u_prev:         np.ndarray = np.zeros(self.nu)
        self._v_des_filtered: float      = 0.0

        # Per-tick telemetry snapshot
        self.last_telemetry: dict = {}

        # ── Parameterised QP (built lazily on first call) ──────────────
        self._qp: dict | None = None

    # ------------------------------------------------------------------
    # Parameterised QP — built ONCE, solved every step
    # ------------------------------------------------------------------

    def _build_qp(self) -> None:
        """
        Constructs the CVXPY problem using parameters. 
        This is built only once to avoid overhead; during runtime, only the 
        parameter values (matrices, initial state) are updated before calling solve().
        """
        nx, nu, N = self.nx, self.nu, self.N

        Ad_p    = cp.Parameter((nx, nx), name="Ad")
        Bd_p    = cp.Parameter((nx, nu), name="Bd")
        dd_p    = cp.Parameter(nx,       name="dd")
        x0_p    = cp.Parameter(nx,       name="x0")
        uprev_p = cp.Parameter(nu,       name="u_prev")
        sqrtR_param = cp.Parameter((self.nu, self.nu), name="sqrtR")
        sqrtRr_param = cp.Parameter((self.nu, self.nu), name="sqrtRr")

        x = cp.Variable((nx, N + 1))
        u = cp.Variable((nu, N))

        constraints = [x[:, 0] == x0_p]
        constraints += [
            x[:, 1:] == Ad_p @ x[:, :-1] + Bd_p @ u + dd_p[:, None]
        ]

        constraints += [
            u >= self.u_min[:, None],
            u <= self.u_max[:, None],
        ]

        du0 = u[:, 0] - uprev_p
        constraints += [
            du0 >= -self.du_max,
            du0 <=  self.du_max,
        ]
        
        if N > 1:
            du_rest = cp.diff(u, axis=1)
            constraints += [
                du_rest >= -self.du_max[:, None],
                du_rest <=  self.du_max[:, None],
            ]

        sqrtQ = np.diag(np.sqrt(np.diag(self.Q)))
        cost  = cp.sum_squares(sqrtQ @ x[:, :N])

        sqrtQ_T = np.diag(np.sqrt(np.diag(self.Q_terminal)))
        cost  += cp.sum_squares(sqrtQ_T @ x[:, N])

        cost  += cp.sum_squares(sqrtR_param @ u)
        cost  += cp.sum_squares(sqrtRr_param @ du0)

        if N > 1:
            cost += cp.sum_squares(sqrtRr_param @ du_rest)

        prob = cp.Problem(cp.Minimize(cost), constraints)

        self._qp = {
            "prob":  prob,
            "Ad":    Ad_p,
            "Bd":    Bd_p,
            "dd":    dd_p,
            "x0":    x0_p,
            "uprev": uprev_p,
            "u":     u,
            "sqrtR": sqrtR_param,
            "sqrtRr": sqrtRr_param
        }

    # ------------------------------------------------------------------
    # Discrete-time model (ZOH via matrix exponential)
    # ------------------------------------------------------------------

    def _discrete_model(
        self,
        v_x:   float,
        kappa: float,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Discretizes the model using velocity-blending while forcing a constant
        dense sparsity pattern (8x8) to prevent OSQP reallocation crashes.
        """
        # Safe speed for matrix denominators
        v_x_safe = max(0.01, abs(v_x))
        m, Iz, lf, lr = self.m, self.Iz, self.lf, self.lr
        Cf, Cr        = self.Cf, self.Cr
        td, ta, dt    = self.tau_delta, self.tau_a, self.dt

        # FORCE DENSE: Initialize all 64 entries with a tiny epsilon to ensure
        # that even 'zero' entries are treated as non-zero by the solver.
        A_kin = np.ones((self.nx, self.nx)) * 1e-12
        A_dyn = np.ones((self.nx, self.nx)) * 1e-12

        # --- 1. Kinematic Model Setup ---
        A_kin[0, 1] = 1.0
        A_kin[2, 6] = v_x_safe / (lf + lr) 
        A_kin[4, 5] = 1.0
        A_kin[5, 7] = 1.0
        A_kin[6, 6] = -1.0 / td
        A_kin[7, 7] = -1.0 / ta

        # --- 2. Dynamic Model Setup ---
        A_dyn[0, 1] = 1.0
        A_dyn[1, 1] = -(2 * Cf + 2 * Cr) / (m * v_x_safe)
        A_dyn[1, 2] = (2 * Cf + 2 * Cr) / m
        A_dyn[1, 3] = (-2 * Cf * lf + 2 * Cr * lr) / (m * v_x_safe)
        A_dyn[1, 6] = (2 * Cf) / m
        A_dyn[2, 3] = 1.0
        A_dyn[3, 1] = (-2 * Cf * lf + 2 * Cr * lr) / (Iz * v_x_safe)
        A_dyn[3, 2] = (2 * Cf * lf - 2 * Cr * lr) / Iz
        A_dyn[3, 3] = -(2 * Cf * lf**2 + 2 * Cr * lr**2) / (Iz * v_x_safe)
        A_dyn[3, 6] = (2 * Cf * lf) / Iz
        A_dyn[4, 5] = 1.0   
        A_dyn[5, 7] = 1.0   
        A_dyn[6, 6] = -1.0 / td   
        A_dyn[7, 7] = -1.0 / ta   

        # --- 3. B-Matrices (Force dense) ---
        B = np.ones((self.nx, self.nu)) * 1e-12
        B[6, 0] = 1.0 / td
        B[7, 1] = 1.0 / ta

        # --- 4. Blending ---
        alpha = np.clip((v_x - 1.0) / (2.5 - 1.0), 0.0, 1.0)
        A_c = (1.0 - alpha) * A_kin + alpha * A_dyn
        B_c = B 
        
        # Disturbance vector
        d_c = np.zeros(self.nx)
        d_c[2] = -kappa * v_x_safe

        # --- 5. ZOH Discretization ---
        n_aug = self.nx + self.nu + 1
        M     = np.zeros((n_aug, n_aug))
        M[: self.nx, : self.nx]                   = A_c
        M[: self.nx, self.nx : self.nx + self.nu] = B_c
        M[: self.nx, -1]                          = d_c

        eM = expm(M * dt)
        return eM[: self.nx, : self.nx], eM[: self.nx, self.nx : self.nx + self.nu], eM[: self.nx, -1]

    # ------------------------------------------------------------------
    # Error state extraction
    # ------------------------------------------------------------------

    def _error_state(
        self,
        path:          np.ndarray,
        car_pos:       np.ndarray,
        car_yaw:       float,
        car_speed:     float,
        car_yaw_rate:  float,
        desired_speed: float,
    ) -> tuple[np.ndarray, float, dict]:
        """
        Calculates the lateral and longitudinal tracking error state of the vehicle 
        relative to the planned trajectory. Projects the front axle position onto the path.

        Returns:
            x0: Initial state vector for the QP [e_y, e_yd, e_psi, e_psi_d, e_v, e_a, δ_act, a_act]
            kappa: Approximated path curvature at a preview distance
            dbg: Dictionary containing telemetry data for logging
        """
        # Project the rear/center coordinate to the front axle by shifting 'lf' meters along the yaw vector [cos(yaw), sin(yaw)]
        fa = car_pos + self.lf * np.array([math.cos(car_yaw), math.sin(car_yaw)])

        # Compute the L2 norm (Euclidean distance) from the front axle to every individual waypoint coordinate along the path array
        base_dists = np.linalg.norm(path - fa, axis=1)

        # Find the index of the minimum scalar value in the distance array to identify the closest waypoint
        base_idx   = int(np.argmin(base_dists))

        # If not at the terminal waypoint, compute the forward displacement vector for the current path segment
        if base_idx < len(path) - 1:
            seg = path[base_idx + 1] - path[base_idx]
        # If at the last waypoint, backward extrapolate the segment vector using the preceding waypoint
        else:
            seg = path[base_idx]     - path[base_idx - 1]

        # Calculate the Euclidean scalar length (L2 norm) of the 2D segment vector
        seg_len = float(np.linalg.norm(seg))

        # Zero-division safety guard check to prevent numeric instabilities on overlapping path waypoints
        if seg_len < 1e-6:
            return np.zeros(self.nx), 0.0, {}

        # Normalize the segment vector to construct a unit tangent vector 't' pointing along the path track direction
        t       = seg / seg_len

        # Create a perpendicular unit normal vector rotated 90 degrees clockwise (pointing to the right of the track direction)
        right_n = np.array([t[1], -t[0]])

        # Extract the orientation angle of the path segment relative to the global grid using the four-quadrant inverse tangent
        path_yaw = math.atan2(t[1], t[0])

        # Compute heading error by wrapping the angular delta inside sin and cos to bound the output cleanly within [-pi, pi]
        e_psi    = math.atan2(
            math.sin(car_yaw - path_yaw),
            math.cos(car_yaw - path_yaw),
        )

        # Project the tracking offset vector onto the right normal vector via dot product; invert sign so left of track yields e_y > 0
        e_y  = -float(np.dot(fa - path[base_idx], right_n))

        # Derive lateral velocity error by resolving the vehicle's forward speed vector along the heading error direction
        e_yd = car_speed * math.sin(e_psi)

        # Define the forward look-ahead distance in meters for tracking upcoming track curvature
        preview_dist = 1.0
        preview_idx  = base_idx
        accumulated  = 0.0

        # Integrate segment lengths forward along the path to find the target look-ahead waypoint index
        for i in range(base_idx, len(path) - 1):
            # Determine the Euclidean length of the upcoming segment
            seg_d = float(np.linalg.norm(path[i + 1] - path[i]))
            # Add to the accumulated arc length
            accumulated += seg_d
            # Stop searching once the accumulated preview window threshold has been surpassed
            if accumulated >= preview_dist:
                preview_idx = i + 1
                break

        # Compute local path curvature (kappa = d_psi / d_s) via numerical differentiation at the preview index
        kappa = _curvature(path, preview_idx)

        # Compute heading rate error by subtracting the structural path rotation rate (kappa * speed) from actual yaw rate
        e_psi_d = car_yaw_rate - kappa * car_speed

        # Calculate longitudinal tracking velocity error by finding the simple difference against the filtered target speed
        e_v = car_speed - desired_speed

        # Pack the calculated errors and actuator states directly into the final 8-dimensional initial condition state vector x0
        x0 = np.array([
            e_y,
            e_yd,
            e_psi,
            e_psi_d,
            e_v,
            0.0,             
            self._delta_act,
            self._a_act,
        ])


        # print(f"\n[MPC LIVE ERRORS]\n"
        #     f"  e_y     (Lateral Dev) : {e_y:7.3f} m\n"
        #     f"  e_yd    (Lat Velocity): {e_yd:7.3f} m/s\n"
        #     f"  e_psi   (Heading Err) : {math.degrees(e_psi):7.2f} deg\n"
        #     f"  e_psi_d (Yaw Rate Err): {math.degrees(e_psi_d):7.2f} deg/s\n"
        #     f"  e_v     (Speed Delta) : {e_v:7.3f} m/s", flush=True)
        
        # Map metrics to descriptive keys within a dictionary structure for downstream ROS2 debugging logs and telemetry tracking
        dbg = {
            "e_y":        e_y,
            "e_psi":      e_psi,
            "e_psi_d":    e_psi_d,
            "e_v":        e_v,
            "kappa":      kappa,
            "base_idx":   base_idx,
            "preview_idx": preview_idx,
        }
        return x0, kappa, dbg

    # ------------------------------------------------------------------
    # QP solve (parameterised — no rebuild per step)
    # ------------------------------------------------------------------

    def _solve_qp(
        self,
        x0: np.ndarray,
        Ad: np.ndarray,
        Bd: np.ndarray,
        dd: np.ndarray,
        R_scaled:      np.ndarray,
        R_rate_scaled: np.ndarray,
    ) -> np.ndarray:
        """
        Injects the current matrices into the parameterised CVXPY problem and solves it.
        Defaults to OSQP, and falls back to Clarabel if numerical issues arise.

        Returns:
            Optimal control sequence vector (only returns the immediate next step [δ_cmd, a_cmd])
        """
        if self._qp is None:
            self._build_qp()

        qp = self._qp
        qp["Ad"].value    = Ad
        qp["Bd"].value    = Bd
        qp["dd"].value    = dd
        qp["x0"].value    = x0
        qp["uprev"].value = self._u_prev.copy()

        sqrtR  = np.diag(np.sqrt(np.clip(np.diag(R_scaled),      1e-6, 1e6)))
        sqrtRr = np.diag(np.sqrt(np.clip(np.diag(R_rate_scaled),  1e-6, 1e6)))
        qp["sqrtR"].value  = sqrtR
        qp["sqrtRr"].value = sqrtRr

        # ── Primary solve: OSQP (Matched to live sim tolerances) ──────
        qp["prob"].solve(
            solver=cp.OSQP,
            verbose=False,
            warm_start=True,
            eps_abs=1e-5,
            eps_rel=1e-5,
            max_iter=8000,
        )

        status = qp["prob"].status
        u_val  = qp["u"][:, 0].value

        if status == cp.OPTIMAL_INACCURATE and u_val is not None:
            print("[MPC] Warning: OSQP OPTIMAL_INACCURATE — "
                  "solution used but weights/feasibility should be checked.")
            return u_val.copy()

        if status == cp.OPTIMAL and u_val is not None:
            return u_val.copy()

        # ── Fallback: Clarabel ────────────────────────────────────────
        try:
            qp["prob"].solve(solver=cp.CLARABEL, verbose=False)
            status_fb = qp["prob"].status
            u_val_fb  = qp["u"][:, 0].value
            if status_fb in (cp.OPTIMAL, cp.OPTIMAL_INACCURATE) and u_val_fb is not None:
                print("[MPC] Warning: OSQP failed, Clarabel succeeded.")
                return u_val_fb.copy()
        except cp.error.SolverError as exc:
            print(f"[MPC] Warning: Clarabel also failed: {exc!r}")

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
        High-level function to trigger the MPC pipeline. Given vehicle state and 
        desired path, extracts error, discretizes the model, solves the QP, and 
        maps physical limits to normalized outputs.

        Returns:
            Tuple of [steering (-1 to 1), throttle (0 to 1), brake (0 to 1)]
        """
        if len(path) < 2:
            return 0.0, 0.0, 0.5   

        alpha = 0.08
        if self._v_des_filtered == 0.0:
            self._v_des_filtered = desired_speed
        self._v_des_filtered += alpha * (desired_speed - self._v_des_filtered)
        desired_speed = self._v_des_filtered

        x0, kappa, dbg = self._error_state(
            path, car_pos, car_yaw, car_speed, car_yaw_rate, desired_speed,
        )

        Ad, Bd, dd = self._discrete_model(car_speed, kappa)

        R_scaled      = _adaptive_R_scaling(car_speed, self.R)
        R_rate_scaled = _adaptive_R_rate(kappa, self.R_rate)

        u_opt = self._solve_qp(x0, Ad, Bd, dd, R_scaled, R_rate_scaled)

        self._delta_act += (self.dt / self.tau_delta) * (u_opt[0] - self._delta_act)
        self._a_act     += (self.dt / self.tau_a)     * (u_opt[1] - self._a_act)
        self._u_prev     = u_opt.copy()

        delta_cmd = float(np.clip(u_opt[0], -MAX_STEER_RAD, MAX_STEER_RAD))
        a_cmd     = float(u_opt[1])

        steering = float(np.clip(-delta_cmd / MAX_STEER_RAD, -1.0, 1.0))

        # Vehicle constants for FSDS mapping
        mass = 255.0
        max_fsds_force = 4000.0  # Must match the Unreal Engine max torque / wheel radius
        cda = 0.7
        rho = 1.225
        crr = 20.0

        if a_cmd >= 0.0:
            # Calculate opposing forces at current speed
            f_drag = 0.5 * rho * cda * (car_speed ** 2)
            
            # Feed-forward force requirement
            f_req = (mass * a_cmd) + f_drag + crr
            
            # Map required force to a 0.0 to 1.0 throttle percentage
            throttle = float(np.clip(f_req / max_fsds_force, 0.0, 1.0))
            brake    = 0.0
        else:
            throttle = 0.0
            
            # Drag assists braking. We want a negative acceleration, 
            # so the brakes need to do less work at high speeds.
            f_drag = 0.5 * rho * cda * (car_speed ** 2)
            f_req_brake = (mass * abs(a_cmd)) - f_drag - crr
            
            # Max FSDS braking force (tune to match your FSDS vehicle setup)
            max_fsds_brake = 2000.0 
            brake = float(np.clip(f_req_brake / max_fsds_brake, 0.0, 1.0))

        self.last_telemetry = {
            **dbg,
            "car_speed":    car_speed,
            "desired_speed": desired_speed,
            "steering":     steering,
            "throttle":     throttle,
            "brake":        brake,
            "delta_cmd":    delta_cmd,
            "a_cmd":        a_cmd,
            "delta_act":    self._delta_act,
            "a_act":        self._a_act,
        }

        return steering, throttle, brake

    def reset(self) -> None:
        """
        Clears the controller's internal state history, forcing the QP solver
        to discard its warm start and actuator lag tracking. Useful after 
        large disruptions like path loss or emergency stops.
        """
        self._delta_act       = 0.0
        self._a_act           = 0.0
        self._u_prev          = np.zeros(self.nu)
        self._v_des_filtered  = 0.0
        if self._qp is not None:
            self._qp["uprev"].value = np.zeros(self.nu)