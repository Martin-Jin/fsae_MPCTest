"""
control_utils.py — MPC path-tracking controller for FSDS.

Linear time-varying MPC built on an 8-state bicycle model with first-order
actuator lag.

  States  x : [e_y, e_yd, e_ψ, e_ψd, e_v, e_a, δ_act, a_act]
  Inputs  u : [δ_cmd (rad), a_cmd (m/s²)]

Sign convention (FSDS ENU — x forward, y left):
  e_y  > 0 → vehicle is to the LEFT  of the reference path
  e_ψ  > 0 → path heading is to the LEFT of the vehicle heading
  δ    > 0 → steer left   (front wheels deflect to port)
  FSDS steering > 0 → steer RIGHT (FSDS sign is opposite to δ)

Key design decisions
--------------------
  • Zero-order hold (matrix-exponential) discretisation for accuracy on the
    fast actuator states (τ ≈ 0.1–0.12 s).
  • Parameterised CVXPY problem — compiled once at first call, then only the
    parameter values (Ad, Bd, dd, x0, u_prev) are updated each step.
    This eliminates the ~5–20 ms problem-build overhead that would otherwise
    occur every 50 ms control tick.
  • Path curvature feedforward — adds −κ·v_x to heading-error dynamics so
    the linearised model has a zero-error equilibrium on curved paths.
  • Input-rate constraints — penalises and hard-bounds Δu per step.
  • Heavier terminal cost — implicit Lyapunov stability weight.
  • Combined longitudinal control — MPC outputs a_cmd as well as δ_cmd.
  • Graceful solver fallback — holds previous steer, coasts on failure.

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

    The model is re-linearised every control step around the current speed v_x
    (LTV / gain-scheduled MPC).  The QP is compiled once into a parameterised
    CVXPY problem; subsequent solves only inject new matrix values, keeping
    solve time well under 10 ms with OSQP warm-starting.

    Tuning quick-reference
    ----------------------
    Q[0,0]  (e_y weight)    up -> tighter lateral tracking, may oscillate more.
    Q[2,2]  (e_psi weight)  up -> faster heading correction.
    Q[3,3]  (e_psi_d weight) up -> heavier yaw-rate damping, reduces sway.
    Q[4,4]  (e_v weight)    up -> tighter speed tracking.
    R[0,0]  (delta effort)  up -> smoother steering, more cross-track error.
    R_rate  (du penalty)    up -> smoother commands, slower response.
    N       (horizon steps) up -> better preview, slightly slower QP solve.
    """

    # ------------------------------------------------------------------
    # Construction
    # ------------------------------------------------------------------

    def __init__(
        self,
        dt: float = 0.05,
        N:  int   = 30,
    ) -> None:
        """
        Parameters
        ----------
        dt : Control period (s) — must match the ROS 2 timer period.
        N  : Prediction horizon (steps).  N=30 gives 1.5 s of preview at 20 Hz.
             Keep N <= 30 for reliable real-time operation.
        """
        self.dt = dt
        self.N  = N

        # ── Vehicle geometry & dynamics (matched to FSDS TechnionCar) ─────
        # Source: https://fs-driverless.github.io/Formula-Student-Driverless-Simulator/v2.2.0/vehicle_model/
        #   Mass: 255 kg, Length: 180 cm, CoG height: 25 cm above ground
        #
        # Cornering stiffness: PhysX uses a Pacejka-like tyre model tuned for
        # a lightweight FS car on slicks.  Road-car values (30000-80000 N/rad)
        # massively overestimate lateral force, making the linearised model
        # predict much sharper response than PhysX actually delivers — causing
        # the MPC to under-command then overcorrect.  FS slick tyres on a 255 kg
        # car produce peak lateral ~3 kN, giving effective stiffness ~15000 N/rad.
        #
        # tau_delta = 0.20 s: PhysX applies steering gradually through its own
        # internal filter — a longer lag better matches observed sim behaviour.
        self.lf  = 0.9       # CoM -> front axle (m)  (wheelbase 1.5 m)
        self.lr  = 0.6       # CoM -> rear  axle (m)
        self.m   = 255.0     # Vehicle mass (kg)  [FSDS spec]
        self.Iz  = 110.0     # Yaw inertia (kg m^2)  [255 kg * 0.43^2 ≈ 110]
        self.Cf  = 15000.0   # Front cornering stiffness (N/rad)  [FS slick estimate]
        self.Cr  = 17000.0   # Rear  cornering stiffness (N/rad)  [slightly stiffer rear]

        # First-order actuator time constants (s)
        # REVERTED: tau_delta was bumped to 0.12s in the previous revision
        # alongside the Q[e_psi_d] cut, in violation of my own "change one
        # variable at a time" advice. Combining a much lower yaw-rate damping
        # weight with a slower-believed actuator is a plausible direct cause
        # of worse oscillation and failed-sharp-corner behaviour: less
        # resistance to yaw rate + a model that thinks corrections arrive
        # later = larger, later corrections = more overshoot.
        # Reverted to the original 0.05s so Q[e_psi_d] is tested in
        # isolation. Re-introduce a tau_delta change separately, AFTER
        # measuring the actual step response (see tuning log at bottom of
        # this file) -- don't guess at it again.
        self.tau_delta = 0.2   # Steering — reverted, see note above
        self.tau_a     = 0.15  # Throttle/brake

        self.nx = 8
        self.nu = 2

        # ── Cost weight matrices ───────────────────────────────────────
        # State order: [e_y, e_yd, e_psi, e_psi_d, e_v, e_a, delta_act, a_act]
        self.Q = np.diag([
            400.0,   # e_y       — lateral error (closest-point, post-fix)
            120.0,   # e_yd      — lateral velocity
            730.0,   # e_psi     — heading error (closest-point, post-fix)
            5200.0,  # e_psi_d   — heading rate damping (was 4850, see note)
            4.5,     # e_v       — speed error
            0.01,    # e_a       — acceleration error
            0.01,    # delta_act — actuator steer (regularisation)
            0.01,    # a_act     — actuator accel (regularisation)
        ])
        self.Q_terminal = 1 * self.Q

        # Absolute control-effort weights
        self.R = np.diag([
            470.0,   # delta_cmd
            15.0,   # a_cmd   
        ])

        # Slew-rate penalty weights (change per time-step).
        # d(delta_cmd) raised 10 -> 25: this is the surgical fix for the
        # left-right-left-right oscillation -- it directly limits how fast
        # consecutive steering commands can swing, which is the actual
        # mechanism that turns "oscillatory plan" into "oscillatory motion".
        # Raise further (30-40) if oscillation persists after the e_psi_d cut;
        # lower back toward 15 if turn-in now feels sluggish on tight corners.
        self.R_rate = np.diag([
            40.0,  # d(delta_cmd)  (was 10.0)
            5.0,   # d(a_cmd)
        ])

        # ── Hard actuator limits ───────────────────────────────────────
        self.u_min  = np.array([-MAX_STEER_RAD, -5.0])   # [rad, m/s^2]
        self.u_max  = np.array([ MAX_STEER_RAD,  2.0])

        # Per-step rate limits (symmetric)
        # 4°/step @ 20 Hz = 80°/s max slew — prevents single-tick direction reversal
        self.du_max = np.array([math.radians(4.0), 0.6])  # [rad/step, m/s^2/step]

        # ── Continuity memory ─────────────────────────────────────────
        self._delta_act: float      = 0.0
        self._a_act:     float      = 0.0
        self._u_prev:    np.ndarray = np.zeros(self.nu)
        # Low-pass filtered desired speed — smooths out planner speed target
        # changes (which jump every ~1 s as new cones are mapped) so the MPC
        # sees a gradually changing reference rather than step inputs.
        self._v_des_filtered: float = 0.0

        # Per-tick telemetry snapshot, populated by compute(). Read this from
        # control_node.py after calling compute() to feed the CSV logger.
        self.last_telemetry: dict = {}

        # ── Parameterised QP (built lazily on first call) ──────────────
        self._qp: dict | None = None   # populated by _build_qp()

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
        """
        nx, nu, N = self.nx, self.nu, self.N

        # ── CVXPY Parameters (data injected each step, no recompilation) ─
        Ad_p    = cp.Parameter((nx, nx), name='Ad')
        Bd_p    = cp.Parameter((nx, nu), name='Bd')
        dd_p    = cp.Parameter(nx,       name='dd')
        x0_p    = cp.Parameter(nx,       name='x0')
        uprev_p = cp.Parameter(nu,       name='u_prev')  # for k=0 rate constraint

        # ── Decision variables ─────────────────────────────────────────
        x = cp.Variable((nx, N + 1))
        u = cp.Variable((nu, N))

        cost        = 0.0
        constraints = [x[:, 0] == x0_p]

        # 2. REPLACE the k loop with direct numpy matrix references
        for k in range(N):
            cost += cp.quad_form(x[:, k], self.Q)
            cost += cp.quad_form(u[:, k], self.R)

            constraints += [
                x[:, k + 1] == Ad_p @ x[:, k] + Bd_p @ u[:, k] + dd_p
            ]
            constraints += [u[:, k] >= self.u_min, u[:, k] <= self.u_max]

            u_prev_k = uprev_p if k == 0 else u[:, k - 1]
            du = u[:, k] - u_prev_k
            cost        += cp.quad_form(du, self.R_rate)
            constraints += [du >= -self.du_max, du <= self.du_max]

        cost += cp.quad_form(x[:, N], self.Q_terminal)

        prob = cp.Problem(cp.Minimize(cost), constraints)

        self._qp = {
            'prob':   prob,
            'Ad':     Ad_p,
            'Bd':     Bd_p,
            'dd':     dd_p,
            'x0':     x0_p,
            'uprev':  uprev_p,
            # Weight parameters have been removed since they are now
            # safely baked directly into the problem graph.
            'u':      u,
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
        """
        v_x = max(0.5, abs(v_x))
        m, Iz, lf, lr = self.m, self.Iz, self.lf, self.lr
        Cf, Cr         = self.Cf, self.Cr
        td, ta, dt     = self.tau_delta, self.tau_a, self.dt

        # ── Continuous A_c (8x8) ───────────────────────────────────────
        # Linearised Rajamani bicycle model + first-order actuator states.
        A_c = np.zeros((self.nx, self.nx))

        # Lateral / yaw dynamics
        A_c[0, 1] = 1.0
        A_c[1, 1] = -(2*Cf + 2*Cr)               / (m  * v_x)
        A_c[1, 2] =  (2*Cf + 2*Cr)               /  m
        A_c[1, 3] = (-2*Cf*lf + 2*Cr*lr)         / (m  * v_x)
        A_c[1, 6] =  (2*Cf)                       /  m
        A_c[2, 3] = 1.0
        A_c[3, 1] = (-2*Cf*lf + 2*Cr*lr)         / (Iz * v_x)
        A_c[3, 2] =  (2*Cf*lf - 2*Cr*lr)         /  Iz
        A_c[3, 3] = -(2*Cf*lf**2 + 2*Cr*lr**2)   / (Iz * v_x)
        A_c[3, 6] =  (2*Cf * lf)                  /  Iz

        # Longitudinal integrator chain.
        A_c[4, 7] = 1.0    # e_v_dot = a_act 
        A_c[5, 7] = 0.0    # Deactivate the unused intermediate state

        # First-order actuator lag
        A_c[6, 6] = -1.0 / td
        A_c[7, 7] = -1.0 / ta

        # ── Continuous B_c (8x2) ───────────────────────────────────────
        B_c = np.zeros((self.nx, self.nu))
        B_c[6, 0] = 1.0 / td   # delta_cmd drives delta_act
        B_c[7, 1] = 1.0 / ta   # a_cmd drives a_act

        # ── Curvature feedforward disturbance ─────────────────────────
        # Continuous heading-error dynamics include e_psi_dot = e_psi_d - kappa*v_x.
        # Without this term the model assumes a straight reference, giving a
        # persistent heading error on corners. Absorbed into a constant
        # disturbance d_c and discretised with A_c and B_c via ZOH.
        d_c = np.zeros(self.nx)
        d_c[2] = -kappa * v_x

        # ── ZOH via augmented matrix exponential ──────────────────────
        # Build the (nx+nu+1) x (nx+nu+1) augmented matrix:
        #
        #   d/dt [x]   [A_c  B_c  d_c] [x]
        #        [u] = [ 0    0    0  ] [u]
        #        [1]   [ 0    0    0  ] [1]
        #
        # expm(M*dt) gives the exact ZOH solution: x(k+1) = Ad x(k)+Bd u(k)+dd
        n_aug = self.nx + self.nu + 1
        M = np.zeros((n_aug, n_aug))
        M[:self.nx, :self.nx]                   = A_c
        M[:self.nx,  self.nx:self.nx + self.nu] = B_c
        M[:self.nx, -1]                         = d_c

        eM = expm(M * dt)

        Ad = eM[:self.nx, :self.nx]
        Bd = eM[:self.nx,  self.nx:self.nx + self.nu]
        dd = eM[:self.nx, -1]

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

        TUNING FIX (see tuning log): e_y/e_psi are now measured at the
        CLOSEST point on the path (base_idx), not at a forward-shifted
        lookahead point. The previous version measured tracking error against
        a point `look_ahead_dist` further along the path's arc length. On a
        curving path, drawing a straight chord from the car to a point further
        along the curve is geometrically biased toward the inside of the turn
        -- the controller was correctly minimizing error to a target that was
        itself wrong, which is the root cause of cutting/running over the
        inside cones on gradual corners.

        A short forward offset (curvature_preview_idx) is still used, but only
        to sample local curvature for the feedforward term -- it no longer
        determines where tracking error is measured.

        Returns
        -------
        x0    : (8,) error state
        kappa : curvature at the closest point (used for ZOH feedforward)
        dbg   : dict of intermediate values for logging (e_y, e_psi, idx, etc.)
        """
        # Control point: front axle position
        fa = car_pos + self.lf * np.array([math.cos(car_yaw), math.sin(car_yaw)])

        # Closest waypoint to the front axle -- this is now ALSO the point
        # tracking error is measured against, not just an anchor for the walk.
        base_dists = np.linalg.norm(path - fa, axis=1)
        base_idx   = int(np.argmin(base_dists))

        # Path tangent AT THE CLOSEST POINT (for e_y / e_psi)
        if base_idx < len(path) - 1:
            seg = path[base_idx + 1] - path[base_idx]
        else:
            seg = path[base_idx] - path[base_idx - 1]
        seg_len = float(np.linalg.norm(seg))
        if seg_len < 1e-6:
            return np.zeros(self.nx), 0.0, {}

        t       = seg / seg_len
        right_n = np.array([t[1], -t[0]])   # 90 deg CW = right-hand normal

        # Heading error: path_yaw - car_yaw, positive means path turns LEFT
        path_yaw = math.atan2(t[1], t[0])
        e_psi = math.atan2(
            math.sin(car_yaw - path_yaw),
            math.cos(car_yaw - path_yaw),
        )

        # Cross-track error: LEFT-positive (car left of path -> e_y > 0)
        # Measured against base_idx (closest point), not a lookahead point.
        e_y = -float(np.dot(fa - path[base_idx], right_n))

        # Lateral velocity approximation
        e_yd = car_speed * math.sin(e_psi)

        # ── Curvature preview (separate from tracking-error point) ────────
        # Walk a short distance forward from base_idx purely to sample
        # curvature for the ZOH feedforward (-kappa*v_x term). This is
        # independent of where e_y/e_psi are measured, so shortening or
        # lengthening it no longer changes the inside-cutting bias.
        # Kept short (curvature changes fast on tight corners; a long preview
        # smooths over exactly the sharp corners we need to react to).
        preview_dist = 1.5  # metres -- short, just enough for a stable estimate
        preview_idx  = base_idx
        accumulated  = 0.0
        for i in range(base_idx, len(path) - 1):
            seg_d = float(np.linalg.norm(path[i + 1] - path[i]))
            accumulated += seg_d
            if accumulated >= preview_dist:
                preview_idx = i + 1
                break

        # Local path curvature at the preview index
        kappa = _curvature(path, preview_idx)

        # Heading error rate relative to path curvature
        e_psi_d = car_yaw_rate - kappa * car_speed

        # Speed error: positive when car is too slow
        desired_speed = min(10.0, desired_speed)   # clip negative speeds to zero
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
            'e_y': e_y, 'e_psi': e_psi, 'e_psi_d': e_psi_d, 'e_v': e_v,
            'kappa': kappa, 'base_idx': base_idx, 'preview_idx': preview_idx,
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
    ) -> np.ndarray:
        """
        Inject current data into the pre-compiled CVXPY problem and solve.

        The CVXPY problem graph is built exactly once (_build_qp) and reused
        every call.  Only cp.Parameter values change, so OSQP can warm-start
        from the previous solution — typically 1-4 ms per call vs 10-20 ms
        for a full rebuild.

        Returns the first optimal control action u[:,0].
        Falls back to (last_steer, 0.0) on solver failure.
        """
        if self._qp is None:
            self._build_qp()

        qp = self._qp
        qp['Ad'].value     = Ad
        qp['Bd'].value     = Bd
        qp['dd'].value     = dd
        qp['x0'].value     = x0
        qp['uprev'].value  = self._u_prev.copy()

        qp['prob'].solve(
            solver=cp.OSQP,
            verbose=False,
            warm_start=True,
            eps_abs=1e-3,
            eps_rel=1e-3,
            max_iter=4000,
        )

        status = qp['prob'].status
        u_val  = qp['u'][:, 0].value

        if status in (cp.OPTIMAL, cp.OPTIMAL_INACCURATE) and u_val is not None:
            return u_val.copy()

        # Safe fallback: hold last steer, zero acceleration (coast)
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
            return 0.0, 0.0, 0.5    # gentle brake if no path

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

        # Step 3: solve parameterised MPC QP (no problem rebuild)
        u_opt = self._solve_qp(x0, Ad, Bd, dd)

        # Step 4: propagate internal actuator states for next step (Euler)
        self._delta_act += (self.dt / self.tau_delta) * (u_opt[0] - self._delta_act)
        self._a_act     += (self.dt / self.tau_a)     * (u_opt[1] - self._a_act)
        self._u_prev     = u_opt.copy()

        # Step 5: convert to FSDS commands
        delta_cmd = float(np.clip(u_opt[0], -MAX_STEER_RAD, MAX_STEER_RAD))
        a_cmd     = float(u_opt[1])

        # FSDS steering sign convention:
        #   Bicycle model: delta > 0 = steer LEFT
        #   FSDS bridge:   steering > 0 = steer RIGHT
        # Therefore negate when normalising.
        steering = float(np.clip(-delta_cmd / MAX_STEER_RAD, -1.0, 1.0))

        # Throttle / brake from MPC acceleration command.
        # if car_speed > desired_speed + 1.25:
        #     throttle = 0.0
        #     brake    = float(np.clip((car_speed - desired_speed - 1.25) * 0.2, 0.0, 0.55))
        if a_cmd >= 0.0:
            # Cap at 0.50 throttle — prevents the MPC from flooring it every
            # low-speed step (u_max=2.0 → 2.0/4.0=0.50 max).
            throttle = float(np.clip(a_cmd / 4.0, 0.0, 0.50))
            brake    = 0.0
        else:
            # Gentle deceleration — cap at 0.50 to avoid wheel-lock in PhysX
            throttle = 0.0
            brake    = float(np.clip(-a_cmd / 8.0, 0.0, 0.50))

        # Stash last-tick telemetry for the control node to log. Cheap dict
        # assembly only -- no file I/O happens here, so this doesn't add
        # per-tick latency. control_node.py reads self.last_telemetry after
        # calling compute() and writes it to disk on its own cadence.
        self.last_telemetry = {
            **dbg,
            'car_speed': car_speed,
            'desired_speed': desired_speed,
            'steering': steering,
            'throttle': throttle,
            'brake': brake,
            'delta_cmd': delta_cmd,
            'a_cmd': a_cmd,
            'delta_act': self._delta_act,
            'a_act': self._a_act,
        }

        return steering, throttle, brake

    def reset(self) -> None:
        """
        Reset all internal continuity state.

        Call after hard stops, teleportation, or long braking phases where the
        previous warm-start solution would be a poor initial guess.
        The compiled CVXPY problem is retained so no recompilation is triggered.
        """
        self._delta_act      = 0.0
        self._a_act          = 0.0
        self._u_prev         = np.zeros(self.nu)
        self._v_des_filtered = 0.0   # re-initialise on next compute() call
        # Neutralise the u_prev parameter so the warm-start doesn't inherit
        # a stale actuator history after a discontinuity.
        if self._qp is not None:
            self._qp['uprev'].value = np.zeros(self.nu)


# ------------------------------------------------------------------
# Module-level helpers
# ------------------------------------------------------------------

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
