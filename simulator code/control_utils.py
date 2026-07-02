# Language: python
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
    """
    scale = max(0.35, 1.0 / (1.0 + 3.0 * abs(kappa)))
    R = R_rate_base.copy()
    R[0, 0] *= scale
    # R[1, 1] unchanged
    return R


def _curvature(path: np.ndarray, idx: int) -> float:
    """
    Estimate signed path curvature (1/m) at waypoint idx via finite-difference
    of the heading angle.  Positive = left-hand turn.  Returns 0 at boundaries.
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
        Parameters
        ----------
        dt : Control period (s) — must match the ROS 2 timer period.
        N  : Prediction horizon (steps).  N=25 gives 1.25 s preview at 20 Hz.
        """
        self.dt = dt
        self.N  = N

        # ── Vehicle geometry & dynamics (Matched to model.py) ─────
        self.lf = 0.9    # CoM -> front axle (m)
        self.lr = 0.6    # CoM -> rear  axle (m)
        self.m  = 255.0  # Vehicle mass (kg)
        self.Iz = 110.0  # Yaw inertia (kg m^2)
        self.Cf = 13500.0  # Front cornering stiffness (N/rad)
        self.Cr = 14500.0  # Rear  cornering stiffness (N/rad)

        # First-order actuator time constants (s)
        self.tau_delta = 0.30  # Steering lag
        self.tau_a     = 0.20  # Throttle/brake lag

        self.nx = 8
        self.nu = 2

        # ── Cost weight matrices (Matched to simulation.py tuner defaults) ───
        # State order: [e_y, e_yd, e_psi, e_psi_d, e_v, e_a, delta_act, a_act]
        self.Q = np.diag([
            669.7320546229485, 
            38.98747288066656, 
            1247.9929146654895, 
            72.00226306885786, 
            44.18749700337984, 
            0.0, 
            0.0, 
            0.0
        ])

        # Terminal cost: 3x running cost for implicit Lyapunov stability.
        self.Q_terminal = 3.0 * self.Q

        # Absolute control-effort weights
        self.R = np.diag([42.73504824590047, 3.0166628936483875])

        # Slew-rate penalty weights (change per time-step).
        self.R_rate = np.diag([45.22626090681843, 0.9368120619766566])

        # ── Hard actuator limits ───────────────────────────────────────
        self.u_min = np.array([-MAX_STEER_RAD, -5.0])  # [rad, m/s^2]
        self.u_max = np.array([ MAX_STEER_RAD,  5.0])

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
        
        v_x = max(0.5, abs(v_x))
        m, Iz, lf, lr = self.m, self.Iz, self.lf, self.lr
        Cf, Cr        = self.Cf, self.Cr
        td, ta, dt    = self.tau_delta, self.tau_a, self.dt

        A_c = np.zeros((self.nx, self.nx))
        A_c[0, 1] = 1.0
        A_c[1, 1] = -(2 * Cf + 2 * Cr) / (m * v_x)
        A_c[1, 2] = (2 * Cf + 2 * Cr) / m
        A_c[1, 3] = (-2 * Cf * lf + 2 * Cr * lr) / (m * v_x)
        A_c[1, 6] = (2 * Cf) / m
        A_c[2, 3] = 1.0
        A_c[3, 1] = (-2 * Cf * lf + 2 * Cr * lr) / (Iz * v_x)
        A_c[3, 2] = (2 * Cf * lf - 2 * Cr * lr) / Iz
        A_c[3, 3] = -(2 * Cf * lf**2 + 2 * Cr * lr**2) / (Iz * v_x)
        A_c[3, 6] = (2 * Cf * lf) / Iz

        A_c[4, 5] = 1.0   
        A_c[5, 7] = 1.0   

        A_c[6, 6] = -1.0 / td   
        A_c[7, 7] = -1.0 / ta   

        B_c = np.zeros((self.nx, self.nu))
        B_c[6, 0] = 1.0 / td  
        B_c[7, 1] = 1.0 / ta  

        d_c = np.zeros(self.nx)
        d_c[2] = -kappa * v_x

        n_aug = self.nx + self.nu + 1
        M     = np.zeros((n_aug, n_aug))
        M[: self.nx, : self.nx]                   = A_c
        M[: self.nx, self.nx : self.nx + self.nu] = B_c
        M[: self.nx, -1]                          = d_c

        eM = expm(M * dt)

        Ad = eM[: self.nx, : self.nx]
        Bd = eM[: self.nx, self.nx : self.nx + self.nu]
        dd = eM[: self.nx, -1]

        return Ad, Bd, dd

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
        
        fa = car_pos + self.lf * np.array([math.cos(car_yaw), math.sin(car_yaw)])
        base_dists = np.linalg.norm(path - fa, axis=1)
        base_idx   = int(np.argmin(base_dists))

        if base_idx < len(path) - 1:
            seg = path[base_idx + 1] - path[base_idx]
        else:
            seg = path[base_idx]     - path[base_idx - 1]
        
        seg_len = float(np.linalg.norm(seg))
        if seg_len < 1e-6:
            return np.zeros(self.nx), 0.0, {}

        t       = seg / seg_len
        right_n = np.array([t[1], -t[0]])

        path_yaw = math.atan2(t[1], t[0])
        e_psi    = math.atan2(
            math.sin(car_yaw - path_yaw),
            math.cos(car_yaw - path_yaw),
        )

        e_y  = -float(np.dot(fa - path[base_idx], right_n))
        e_yd = car_speed * math.sin(e_psi)

        preview_dist = 1.5
        preview_idx  = base_idx
        accumulated  = 0.0
        for i in range(base_idx, len(path) - 1):
            seg_d = float(np.linalg.norm(path[i + 1] - path[i]))
            accumulated += seg_d
            if accumulated >= preview_dist:
                preview_idx = i + 1
                break

        kappa = _curvature(path, preview_idx)
        e_psi_d = car_yaw_rate - kappa * car_speed

        desired_speed = max(10, desired_speed)
        e_v = car_speed - desired_speed

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

        Ad, Bd, dd = self._discrete_model(max(car_speed, 1.0), kappa)

        R_scaled      = _adaptive_R_scaling(car_speed, self.R)
        R_rate_scaled = _adaptive_R_rate(kappa, self.R_rate)

        u_opt = self._solve_qp(x0, Ad, Bd, dd, R_scaled, R_rate_scaled)

        self._delta_act += (self.dt / self.tau_delta) * (u_opt[0] - self._delta_act)
        self._a_act     += (self.dt / self.tau_a)     * (u_opt[1] - self._a_act)
        self._u_prev     = u_opt.copy()

        delta_cmd = float(np.clip(u_opt[0], -MAX_STEER_RAD, MAX_STEER_RAD))
        a_cmd     = float(u_opt[1])

        steering = float(np.clip(-delta_cmd / MAX_STEER_RAD, -1.0, 1.0))

        if a_cmd >= 0.0:
            throttle = float(np.clip(a_cmd / 5.0, 0.0, 0.85))
            brake    = 0.0
        else:
            throttle = 0.0
            brake    = float(np.clip(-a_cmd / 8.0, 0.0, 0.50))

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
        self._delta_act       = 0.0
        self._a_act           = 0.0
        self._u_prev          = np.zeros(self.nu)
        self._v_des_filtered  = 0.0
        if self._qp is not None:
            self._qp["uprev"].value = np.zeros(self.nu)