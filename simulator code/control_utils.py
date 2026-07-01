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

Key design decisions
--------------------
  • Zero-order hold (matrix-exponential) discretisation for accuracy on the
    fast actuator states (τ ≈ 0.15–0.20 s).
  • Parameterised CVXPY problem — compiled once at first call, then only the
    parameter values (Ad, Bd, dd, x0, u_prev) are updated each step.
    This eliminates the ~5-20 ms problem-build overhead that would otherwise
    occur every 50 ms control tick.
  • Path curvature feedforward — adds −κ·v_x to heading-error dynamics so
    the linearised model has a zero-error equilibrium on curved paths.
  • Input-rate constraints — penalises and hard-bounds Δu per step.
  • Heavier terminal cost — implicit Lyapunov stability weight.
  • Combined longitudinal control — MPC outputs a_cmd as well as δ_cmd.
  • Adaptive R / R_rate scaling — steering cost saturates with speed;
    R_rate steering term softens in corners while accel stays firm.
  • Graceful solver fallback — tries Clarabel before holding previous steer.

Dependencies: numpy, scipy, cvxpy (with OSQP + Clarabel backends).
"""

import math

import cvxpy as cp
import numpy as np
from scipy.linalg import expm

# FSDS: maximum physical steering deflection per ros-bridge.md
MAX_STEER_RAD: float = math.radians(25.0)


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

    The old linear 1+0.25*vx gave 3.5x at 10 m/s, over-penalising steering
    corrections at exactly the speeds where heading errors are hardest to recover.

    Accel scale stays mild and linear (unchanged from prior version).
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

    In tight corners the steering rate penalty is reduced so the controller can
    unwind quickly without fighting the slew limiter. The accel/brake jerk penalty
    is deliberately NOT reduced in corners — firm braking timing is most important
    exactly when the vehicle is most load-sensitive.
    """
    scale = 1.0 / (1.0 + 3.0 * abs(kappa))
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

    The model is re-linearised every control step around the current speed v_x
    (LTV / gain-scheduled MPC).  The QP is compiled once into a parameterised
    CVXPY problem; subsequent solves only inject new matrix values, keeping
    solve time well under 10 ms with OSQP warm-starting.

    Tuning quick-reference
    ----------------------
    Q[0,0]  (e_y weight)     up → tighter lateral tracking, may oscillate more.
    Q[2,2]  (e_psi weight)   up → faster heading correction.
    Q[3,3]  (e_psi_d weight) up → heavier yaw-rate damping, reduces sway.
    Q[4,4]  (e_v weight)     up → tighter speed tracking.
    R[0,0]  (delta effort)   up → smoother steering, more cross-track error.
    R_rate  (du penalty)     up → smoother commands, slower response.
    N       (horizon steps)  up → better preview, slightly slower QP solve.
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
             Matches simulation.py so offline-tuned weights transfer cleanly.
             (Previous default was 30; see model-mismatch note in control_node.py.)
        """
        self.dt = dt
        self.N  = N

        # ── Vehicle geometry & dynamics (matched to FSDS TechnionCar) ─────
        # Source: https://fs-driverless.github.io/Formula-Student-Driverless-Simulator/v2.2.0/vehicle_model/
        #   Mass: 255 kg, Length: 180 cm, CoG height: 25 cm above ground
        #
        # Cornering stiffness: PhysX uses a Pacejka-like tyre model tuned for
        # a lightweight FS car on slicks. FS slick tyres on a 255 kg car
        # produce peak lateral ~3 kN → effective stiffness ~15000 N/rad.
        #
        # tau_delta = 0.20 s: PhysX applies steering gradually through its own
        # internal filter — a longer lag better matches observed sim behaviour.
        self.lf = 0.9    # CoM -> front axle (m)  (wheelbase 1.5 m)
        self.lr = 0.6    # CoM -> rear  axle (m)
        self.m  = 255.0  # Vehicle mass (kg)  [FSDS spec]
        self.Iz = 110.0  # Yaw inertia (kg m^2)  [255 kg * 0.43^2 ≈ 110]
        self.Cf = 13500.0  # Front cornering stiffness (N/rad)  [FS slick estimate]
        self.Cr = 14500.0  # Rear  cornering stiffness (N/rad)  [slightly stiffer rear]

        # First-order actuator time constants (s)
        self.tau_delta = 0.30  # Steering lag
        self.tau_a     = 0.20  # Throttle/brake lag

        self.nx = 8
        self.nu = 2

        # ── Cost weight matrices ───────────────────────────────────────
        # State order: [e_y, e_yd, e_psi, e_psi_d, e_v, e_a, delta_act, a_act]
        #
        # These match simulation.py exactly so weights tuned offline (offline_tuner.py)
        # or by the online hill-climber (tuner.py) transfer without rescaling.
        #
        # Q[4,4] raised from 90 → 150: the old value was too weak on straights
        # where lateral/heading errors are small and the speed term needs to matter.
        self.Q = np.diag([
            1738.0,  # e_y       — lateral error
            426.0,   # e_yd      — lateral velocity
            5718.0,  # e_psi     — heading error
            2006.0,  # e_psi_d   — heading rate damping
            165.0,   # e_v       — speed error  (raised from 90)
            0.0,     # e_a       — acceleration error (not penalised; R_rate handles smoothness)
            0.0,     # delta_act — actuator steer (regularisation only)
            0.0,     # a_act     — actuator accel (regularisation only)
        ])

        # Terminal cost: 3x running cost for implicit Lyapunov stability.
        # The multiplier is larger than 1x (steady-state Riccati) to give the
        # horizon a strong incentive to converge the state to near-zero by step N,
        # without paying the computational cost of solving a DARE every tick.
        self.Q_terminal = 3.0 * self.Q

        # Absolute control-effort weights
        self.R = np.diag([
            689.2,  # delta_cmd
            126.5,   # a_cmd
        ])

        # Slew-rate penalty weights (change per time-step).
        # Steering rate: directly limits how fast consecutive steering commands
        # can swing, which is the primary mechanism controlling oscillation.
        # Raise toward 2000-2500 if left-right hunting persists; lower toward
        # 1000 if turn-in on tight corners feels sluggish.
        self.R_rate = np.diag([
            1076.0,  # d(delta_cmd)/dt — penalize sharp steering jerk
            60.0,     # d(a_cmd)/dt     — penalize sharp accel/brake jerk
        ])

        # ── Hard actuator limits ───────────────────────────────────────
        # u_max[1] raised from 2.0 → 5.0 to match simulation.py u_bounds_max.
        # The old 2.0 cap prevented the MPC from commanding full acceleration
        # authority and made the a_cmd → throttle normalisation asymmetric.
        self.u_min = np.array([-MAX_STEER_RAD, -5.0])  # [rad, m/s^2]
        self.u_max = np.array([ MAX_STEER_RAD,  5.0])

        # Per-step rate limits (symmetric)
        # 4°/step @ 20 Hz = 80°/s max slew — prevents single-tick direction reversal.
        # 0.6 m/s² per step = 12 m/s³ jerk — comfortable for FSDS PhysX.
        self.du_max = np.array([math.radians(4.0), 0.6])  # [rad/step, m/s^2/step]

        # ── Continuity memory ─────────────────────────────────────────
        self._delta_act:      float      = 0.0
        self._a_act:          float      = 0.0
        self._u_prev:         np.ndarray = np.zeros(self.nu)
        self._v_des_filtered: float      = 0.0

        # Per-tick telemetry snapshot — populated by compute(), read by control_node.py.
        self.last_telemetry: dict = {}

        # ── Parameterised QP (built lazily on first call) ──────────────
        self._qp: dict | None = None

    # ------------------------------------------------------------------
    # Parameterised QP — built ONCE, solved every step
    # ------------------------------------------------------------------

    def _build_qp(self) -> None:
        """
        Compile the finite-horizon QP into a parameterised CVXPY problem.

        Called exactly once (lazy init on first compute() call).  All
        subsequent solves only update cp.Parameter values; the symbolic graph
        is never rebuilt.  This removes the dominant latency of the original
        per-step problem-construction approach (~5-20 ms saved per tick).

        IMPROVEMENT: Uses a vectorised formulation (matrix operations over the
        full horizon) instead of a Python for-loop over N steps.  This reduces
        the CVXPY expression-graph size and cuts first-call compilation time,
        particularly for large N.
        """
        nx, nu, N = self.nx, self.nu, self.N

        # ── CVXPY Parameters (data injected each step, no recompilation) ─
        Ad_p    = cp.Parameter((nx, nx), name="Ad")
        Bd_p    = cp.Parameter((nx, nu), name="Bd")
        dd_p    = cp.Parameter(nx,       name="dd")
        x0_p    = cp.Parameter(nx,       name="x0")
        uprev_p = cp.Parameter(nu,       name="u_prev")
        sqrtR_param = cp.Parameter((self.nu, self.nu), name="sqrtR")
        sqrtRr_param = cp.Parameter((self.nu, self.nu), name="sqrtRr")

        # ── Decision variables ─────────────────────────────────────────
        x = cp.Variable((nx, N + 1))   # states  x[:,0..N]
        u = cp.Variable((nu, N))       # inputs  u[:,0..N-1]

        # ── Dynamics constraints (vectorised over all steps) ───────────
        # x[:,k+1] = Ad @ x[:,k] + Bd @ u[:,k] + dd  for k=0..N-1
        constraints = [x[:, 0] == x0_p]
        constraints += [
            x[:, 1:] == Ad_p @ x[:, :-1] + Bd_p @ u + dd_p[:, None]
        ]

        # ── Input box constraints ──────────────────────────────────────
        constraints += [
            u >= self.u_min[:, None],
            u <= self.u_max[:, None],
        ]

        # ── Rate constraints ───────────────────────────────────────────
        # Step 0: rate vs previous applied input (u_prev parameter)
        du0 = u[:, 0] - uprev_p
        constraints += [
            du0 >= -self.du_max,
            du0 <=  self.du_max,
        ]
        # Steps 1..N-1: rate vs previous step
        if N > 1:
            du_rest = cp.diff(u, axis=1)          # shape (nu, N-1)
            constraints += [
                du_rest >= -self.du_max[:, None],
                du_rest <=  self.du_max[:, None],
            ]

# ── Cost (vectorised) ──────────────────────────────────────────
        # Running state cost: sum_{k=0}^{N-1} x[:,k]^T Q x[:,k]
        sqrtQ = np.diag(np.sqrt(np.diag(self.Q)))
        cost  = cp.sum_squares(sqrtQ @ x[:, :N])

        # Terminal state cost: x[:,N]^T Q_terminal x[:,N]
        sqrtQ_T = np.diag(np.sqrt(np.diag(self.Q_terminal)))
        cost  += cp.sum_squares(sqrtQ_T @ x[:, N])

        # Running input cost: sum_{k=0}^{N-1} u[:,k]^T R u[:,k]
        cost  += cp.sum_squares(sqrtR_param @ u)

        # Rate cost step 0
        cost  += cp.sum_squares(sqrtRr_param @ du0)

        # Rate cost steps 1..N-1
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
        Zero-order-hold discretisation of the linearised bicycle + actuator model.

        Parameters
        ----------
        v_x   : Longitudinal speed (m/s), clipped to >= 0.5 m/s for numerical
                stability of the lateral tyre-force terms.
        kappa : Local path curvature (1/m), positive = left-hand turn.
                Enters as a constant disturbance on the heading-error derivative
                so that steady-state cornering produces zero heading error.

        Returns
        -------
        Ad : (8, 8)  Discrete-time state matrix
        Bd : (8, 2)  Discrete-time input matrix
        dd : (8,)    Discrete-time curvature disturbance vector

        Longitudinal state chain (matches simulation.py model.py exactly):
          e_v_dot = e_a          (A_c[4, 5] = 1.0)
          e_a_dot = a_act        (A_c[5, 7] = 1.0)
          a_act_dot = -a_act/τ_a + a_cmd/τ_a  (first-order lag)

        This two-integrator chain (e_v → e_a → a_act) means speed error takes
        two steps to observe the effect of a_cmd through both integrators, which
        produces smoother speed tracking than a direct e_v → a_act connection.
        The previous version had A_c[4, 7] = 1.0 (direct), which is inconsistent
        with simulation.py; unifying to the two-integrator form ensures that
        weights tuned in simulation transfer cleanly here.
        """
        v_x = max(0.5, abs(v_x))
        m, Iz, lf, lr = self.m, self.Iz, self.lf, self.lr
        Cf, Cr        = self.Cf, self.Cr
        td, ta, dt    = self.tau_delta, self.tau_a, self.dt

        # ── Continuous A_c (8x8) ───────────────────────────────────────
        A_c = np.zeros((self.nx, self.nx))

        # Lateral / yaw dynamics (Rajamani bicycle model)
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

        # Longitudinal two-integrator chain: e_v -> e_a -> a_act
        # BUG FIX: was A_c[4, 7] = 1.0 (direct e_v → a_act, bypassing e_a).
        # Now matches simulation.py model.py exactly.
        A_c[4, 5] = 1.0   # e_v_dot  = e_a
        A_c[5, 7] = 1.0   # e_a_dot  = a_act

        # First-order actuator lag
        A_c[6, 6] = -1.0 / td   # delta_act
        A_c[7, 7] = -1.0 / ta   # a_act

        # ── Continuous B_c (8x2) ───────────────────────────────────────
        B_c = np.zeros((self.nx, self.nu))
        B_c[6, 0] = 1.0 / td  # delta_cmd drives delta_act
        B_c[7, 1] = 1.0 / ta  # a_cmd drives a_act

        # ── Curvature feedforward disturbance ─────────────────────────
        # Continuous heading-error dynamics:
        #   ė_ψ = e_ψd - κ·v_x
        # Without this term the model assumes a straight reference, giving a
        # persistent heading error on corners. Absorbed into a constant
        # disturbance d_c and discretised with A_c and B_c via ZOH.
        d_c = np.zeros(self.nx)
        d_c[2] = -kappa * v_x

        # ── ZOH via augmented matrix exponential ──────────────────────
        # Augmented system (nx + nu + 1):
        #
        #   d/dt [x]   [A_c  B_c  d_c] [x]
        #        [u] = [ 0    0    0  ] [u]
        #        [1]   [ 0    0    0  ] [1]
        #
        # expm(M*dt) gives the exact ZOH: x(k+1) = Ad x(k) + Bd u(k) + dd
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
        """
        Project odometry + path geometry into the 8-D error state.

        Tracking error is measured at the CLOSEST point on the path to the
        front axle (not a lookahead point). A separate short forward walk
        (preview_dist = 1.5 m) is used only to sample curvature for the ZOH
        feedforward term, so the feedforward and the tracking error origin are
        decoupled — changing preview_dist no longer biases tracking error.

        Sign convention for e_psi (CORRECTED AND DOCUMENTED):
          path_yaw = atan2(tangent_y, tangent_x)
          e_psi    = atan2(sin(yaw_car - path_yaw), cos(yaw_car - path_yaw))
          e_psi > 0 → vehicle is pointing MORE LEFT than the path
                      (vehicle has over-rotated CCW relative to the path tangent)
          Corrective action: δ > 0 (steer left) pushes the front axle left,
          generating a rightward yaw moment — reducing positive e_psi. ✓

        Returns
        -------
        x0    : (8,) error state
        kappa : curvature at the preview index (for ZOH feedforward)
        dbg   : dict of intermediate values for the telemetry log
        """
        # Control point: front axle position
        fa = car_pos + self.lf * np.array([math.cos(car_yaw), math.sin(car_yaw)])

        # Closest waypoint to the front axle
        base_dists = np.linalg.norm(path - fa, axis=1)
        base_idx   = int(np.argmin(base_dists))

        # Path tangent at the closest point
        if base_idx < len(path) - 1:
            seg = path[base_idx + 1] - path[base_idx]
        else:
            seg = path[base_idx]     - path[base_idx - 1]
        seg_len = float(np.linalg.norm(seg))
        if seg_len < 1e-6:
            return np.zeros(self.nx), 0.0, {}

        t       = seg / seg_len
        right_n = np.array([t[1], -t[0]])   # 90° CW = right-hand normal

        # Heading error (see sign convention above)
        path_yaw = math.atan2(t[1], t[0])
        e_psi    = math.atan2(
            math.sin(car_yaw - path_yaw),
            math.cos(car_yaw - path_yaw),
        )

        # Cross-track error: LEFT-positive (car left of path → e_y > 0)
        e_y  = -float(np.dot(fa - path[base_idx], right_n))

        # Lateral velocity approximation from small-angle kinematics
        e_yd = car_speed * math.sin(e_psi)

        # ── Curvature preview (separate from tracking-error point) ────────
        preview_dist = 1.5   # metres — short for stable curvature estimate
        preview_idx  = base_idx
        accumulated  = 0.0
        for i in range(base_idx, len(path) - 1):
            seg_d = float(np.linalg.norm(path[i + 1] - path[i]))
            accumulated += seg_d
            if accumulated >= preview_dist:
                preview_idx = i + 1
                break

        kappa = _curvature(path, preview_idx)

        # Heading error rate relative to path curvature
        e_psi_d = car_yaw_rate - kappa * car_speed

        # Speed error: positive when car is too fast (sign matches simulation.py)
        e_v = car_speed - desired_speed

        x0 = np.array([
            e_y,
            e_yd,
            e_psi,
            e_psi_d,
            e_v,
            0.0,             # e_a: initialised to zero each step
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
        """
        Inject current data into the pre-compiled CVXPY problem and solve.

        IMPROVEMENT: Clarabel fallback added. When OSQP returns an outright
        failure (not just OPTIMAL_INACCURATE), Clarabel is tried before
        falling back to hold-steer. OPTIMAL_INACCURATE is accepted (the
        solution is usable at 20 Hz) but logged as a warning once so the
        operator can see when the QP is near the edge of feasibility.

        NOTE: R_scaled and R_rate_scaled are applied by rebuilding sqrtR /
        sqrtRr from the scaled matrices before injecting them into the static
        cost. Because the QP is parameterised with baked-in sqrt weight
        matrices, adaptive scaling requires either (a) weight cp.Parameters
        (adds overhead) or (b) a light re-inject of the sqrt coefficient
        matrices into the fixed cost terms. We use option (b): the sqrtR and
        sqrtRr matrices in the cost are replaced by re-extracting them from
        R_scaled / R_rate_scaled and updating the _qp["sqrtR"] / _qp["sqrtRr"]
        cp.Parameter values. See _build_qp for the matching cp.Parameter setup.

        Returns the first optimal control action u[:,0].
        Falls back to (last_steer, 0.0) on solver failure.
        """
        if self._qp is None:
            self._build_qp()

        qp = self._qp
        qp["Ad"].value    = Ad
        qp["Bd"].value    = Bd
        qp["dd"].value    = dd
        qp["x0"].value    = x0
        qp["uprev"].value = self._u_prev.copy()

        # Inject adaptive sqrt weight matrices
        sqrtR  = np.diag(np.sqrt(np.clip(np.diag(R_scaled),      1e-6, 1e6)))
        sqrtRr = np.diag(np.sqrt(np.clip(np.diag(R_rate_scaled),  1e-6, 1e6)))
        qp["sqrtR"].value  = sqrtR
        qp["sqrtRr"].value = sqrtRr

        # ── Primary solve: OSQP ───────────────────────────────────────
        qp["prob"].solve(
            solver=cp.OSQP,
            verbose=False,
            warm_start=True,
            eps_abs=1e-3,
            eps_rel=1e-3,
            max_iter=4000,
        )

        status = qp["prob"].status
        u_val  = qp["u"][:, 0].value

        if status == cp.OPTIMAL_INACCURATE and u_val is not None:
            # Log once per occurrence — at 20 Hz this would otherwise flood
            # the ROS log. throttle_duration_sec is set in the calling node;
            # here we just print so the module stays ROS-agnostic.
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

        # ── Final fallback: hold last steer, zero acceleration ────────
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
        path          : (N_wp, 2) planned waypoints in map frame (>= 2 points)
        car_pos       : (2,)  vehicle position in map frame
        car_yaw       : Vehicle heading (rad)
        car_speed     : Longitudinal speed (m/s)
        desired_speed : Reference speed (m/s)
        car_yaw_rate  : Yaw rate (rad/s), positive = CCW / left turn

        Returns
        -------
        steering : float in [-1, 1]   positive = steer RIGHT (FSDS convention)
        throttle : float in  [0, 1]
        brake    : float in  [0, 1]
        """
        if len(path) < 2:
            return 0.0, 0.0, 0.5   # gentle brake if no path

        # Low-pass filter the desired speed to avoid step-input changes from
        # the planner updating every cone-map cycle (~1 Hz).
        # α=0.08 → time constant ≈ 0.6 s at 20 Hz — smooths jerky targets
        # while still following genuine speed reductions within ~1 s.
        alpha = 0.08
        if self._v_des_filtered == 0.0:
            self._v_des_filtered = desired_speed   # initialise on first call
        self._v_des_filtered += alpha * (desired_speed - self._v_des_filtered)
        desired_speed = self._v_des_filtered

        # Step 1: extract 8-state error vector and local curvature
        x0, kappa, dbg = self._error_state(
            path, car_pos, car_yaw, car_speed, car_yaw_rate, desired_speed,
        )

        # Step 2: linearise + ZOH-discretise model at current speed
        Ad, Bd, dd = self._discrete_model(max(car_speed, 1.0), kappa)

        # Step 3: compute adaptive gain matrices
        R_scaled      = _adaptive_R_scaling(car_speed, self.R)
        R_rate_scaled = _adaptive_R_rate(kappa, self.R_rate)

        # Step 4: solve parameterised MPC QP (no problem rebuild)
        u_opt = self._solve_qp(x0, Ad, Bd, dd, R_scaled, R_rate_scaled)

        # Step 5: propagate internal actuator states for next step (Euler)
        self._delta_act += (self.dt / self.tau_delta) * (u_opt[0] - self._delta_act)
        self._a_act     += (self.dt / self.tau_a)     * (u_opt[1] - self._a_act)
        self._u_prev     = u_opt.copy()

        # Step 6: convert to FSDS commands
        delta_cmd = float(np.clip(u_opt[0], -MAX_STEER_RAD, MAX_STEER_RAD))
        a_cmd     = float(u_opt[1])

        # FSDS steering sign convention:
        #   Bicycle model: delta > 0 = steer LEFT
        #   FSDS bridge:   steering > 0 = steer RIGHT
        # Therefore negate when normalising.
        steering = float(np.clip(-delta_cmd / MAX_STEER_RAD, -1.0, 1.0))

        # Throttle / brake from MPC acceleration command.
        # Throttle: normalised against u_max[1] = 5.0 m/s², capped at 0.85
        # to avoid flooring the throttle (FSDS PhysX responds non-linearly
        # near full throttle; keeping headroom avoids overshoot on short straights).
        # Brake: normalised against 8.0 m/s² deceleration capacity, capped at 0.50
        # to avoid wheel-lock in PhysX.
        if a_cmd >= 0.0:
            throttle = float(np.clip(a_cmd / 5.0, 0.0, 0.85))
            brake    = 0.0
        else:
            throttle = 0.0
            brake    = float(np.clip(-a_cmd / 8.0, 0.0, 0.50))

        # Stash telemetry for the control node to log.
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
        Reset all internal continuity state.

        Call after hard stops, teleportation, or long braking phases where the
        previous warm-start solution would be a poor initial guess.
        The compiled CVXPY problem is retained so no recompilation is triggered.
        """
        self._delta_act       = 0.0
        self._a_act           = 0.0
        self._u_prev          = np.zeros(self.nu)
        self._v_des_filtered  = 0.0
        if self._qp is not None:
            self._qp["uprev"].value = np.zeros(self.nu)