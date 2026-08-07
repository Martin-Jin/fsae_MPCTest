# Title: mpc_core.py

# NOTE (fsae_control port): this is a near-verbatim copy of
# fsae_MPCTest/"fsds simulator"/control_utils.py, kept byte-for-byte in the MPC
# math so the offline-tuned weights (Q/R/R_rate) still transfer.  The ONLY
# intentional change is MAX_STEER_RAD 35deg -> 25deg to match this stack's
# physical steering limit (see fsae_control.control_utils.MAX_STEER_RAD and
# fsds_bridge.MAX_STEER_RAD); the MPC now plans steering the car can actually
# deliver.  If you re-sync from upstream, re-apply that one change.

"""
mpc_core.py — Live MPC Path-Tracking Controller for FSDS

PURPOSE
-------
Provides MPCController, the single class both mpc_controller.py and
mpc_controller_standalone.py use to turn a planner path + current vehicle
state into steering/throttle/brake at 20 Hz (mpc_controller.py forwards only
steering through the shared cmd_vel interface; mpc_controller_standalone.py
uses the full (steering, throttle, brake) triple directly — see that file's
own docstring for why). It is a self-contained, "live-solve" re-implementation
of the same linear time-varying MPC formulated generically in optimiser.py /
bicycle_model.py for the offline tuner and simulator (both in the
fsae_MPCTest repo), designed for 100% numerical parity with that offline
pipeline so that weights tuned there transfer directly to the real/simulated
vehicle.

  States  x : [e_y, e_yd, e_psi, r, e_v, e_a, delta_act, a_act]   (8,)
  Inputs  u : [delta_cmd (rad), a_cmd (m/s2)]                      (2,)

HOW IT WORKS
------------
Each call to MPCController.compute() runs the full MPC pipeline:
  1. Low-pass filter the incoming desired_speed (_v_des_filtered) to avoid
     feeding step changes into the MPC's speed-error state.
  2. _error_state() — project the vehicle's front axle onto the nearest
     path segment to get Frenet-style tracking errors (e_y, e_psi, e_v) and
     a short-lookahead curvature estimate (kappa), then assemble the 8-state
     vector x0 (reusing the controller's own actuator-lag memory for the
     delta_act/a_act entries, since those aren't directly measurable).
  3. _discrete_model() — build the speed-blended kinematic/dynamic bicycle
     model and ZOH-discretise it (mirrors bicycle_model.get_8state_discrete_model,
     duplicated locally so the live controller has no simulation dependencies).
  4. Gain-schedule R and R_rate via the module-level _adaptive_R_scaling /
     _adaptive_R_rate helpers (mirrors model_utils.py's adaptive_R_scaling /
     adaptive_R_rate — duplicated here for the same reason).
  5. _solve_qp() — inject the above into a persistent, parameterised CVXPY
     problem (built once in _build_qp, reused via warm-start) and solve with
     OSQP, falling back to Clarabel, then to a full-brake command (holding
     the last steering angle) if both solvers fail.
  6. Integrate the actuator lag states exactly (ZOH, not Euler) so
     delta_act/a_act stay consistent even though dt (0.05s) is comparable
     to tau_a (0.02s).
  7. Convert [delta_cmd, a_cmd] into FSDS's normalised
     [steering, throttle, brake] command triple and populate
     self.last_telemetry for the caller's telemetry logging.

PARITY WITH THE OFFLINE PIPELINE
---------------------------------
_adaptive_R_scaling/_adaptive_R_rate/_discrete_model here are intentionally
near-identical duplicates of model_utils.py / bicycle_model.py, and
_build_qp's cost/constraint formulation is a near-identical duplicate of
optimiser.py's init_parameterized_mpc (same +/-3.5 m soft lane bound, same
W_slack=10000, same step-0/subsequent rate-cost split), plus a hard
per-step slew-rate constraint on [delta_cmd, a_cmd] (self.du_max) enforced
in addition to the soft R_rate cost. Any change to the cost/constraint
structure in one location should be mirrored in the other, or the weights
tuned by offline_tuner.py will no longer transfer faithfully to the live
controller.

USED BY
-------
  mpc_controller.py and mpc_controller_standalone.py — each constructs its
                    own MPCController(dt=0.05, N=25) in __init__ and calls
                    .compute() every 20 Hz tick, .reset() on stale path /
                    cone-brake fail-safes.
"""

import math
import time
from collections import deque

import cvxpy as cp
import numpy as np
from scipy.linalg import expm

# Maximum physical steering deflection.  25deg matches this stack's limit
# (fsae_control.control_utils / fsds_bridge); upstream used 35deg.
MAX_STEER_RAD: float = math.radians(25.0)
MAX_ACCEL: float = 12.0
MAX_BRAKE: float = 9.0

# ── Reference-heading rate limit ─────────────────────────────────────────────
# Mirrors fsae_MPCTest/sim/rollout_core.py's REF_HEADING_RATE_LIMIT_ENABLED /
# REF_HEADING_RISE_RATE / _rate_limit_ref_psi — keep all three in sync.
# See sim_to_real_investigation.md S26-S28: the planner's published centreline
# sometimes anticipates a sharp corner earlier than the car has actually
# yawed yet, a sustained (not single-frame) effect strongly linked to
# steering saturation. This caps how fast the tracked reference heading
# (path_yaw in _error_state) may change per tick, symmetric in both
# directions — unlike the speed-target rise limiter, there is no "always
# safe" direction for a heading reference. MEASURED value: 90 deg/s is the
# tightest rate with no DNF across settings.VALIDATION_SUITE offline; do not
# tighten this without re-running tuner/ref_heading_limiter_suite_check.py —
# tighter values look better on the recorded map alone but DNF a fast slalom.
# TRIED LIVE 2026-08-07 at 90 deg/s (see sim_to_real_investigation.md S29):
# made saturation WORSE (21.1%/26.4% baselines -> 28.0%), via the same
# failure mode S28 found offline on PATH_MICRO_SLALOM (holding the reference
# back during turn-in leaves a larger heading deficit to claw back later).
# Do not re-enable without a new offline test against a synthetic path
# shaped like that failure first — see S29 for what to check.
REF_HEADING_RATE_LIMIT_ENABLED: bool = False
REF_HEADING_RISE_RATE_DEG_S: float = 90.0

# ── Delay compensation ──────────────────────────────────────────────────────
# Real delay (perception + planning + control + actuation latency) is
# unknown and time-varying, unlike fsae_MPCTest's simulator-only fixed
# DELAY_STEPS. compute() is instead told how OLD the pose it's solving
# against is (pose_age_s, measured from the pose message's own timestamp —
# see mpc_controller.py/_pose_cb) and converts that into a step count itself.
# See predict_ahead() below for the same small-angle-clip rollforward
# validated in fsae_MPCTest/sim/rollout_core.py.
MAX_DELAY_COMPENSATION_STEPS: int = 6   # cap: a bigger measured age is clamped, not trusted blindly
_PREDICT_EPSI_CLIP: float = 0.5         # rad (~28.6°) — small-angle bound, see predict_ahead

# ── n_delay stabilisation ───────────────────────────────────────────────────
# pose_age_s is noisy: the control loop's own jitter (measured dt median
# 0.050 s but max 0.121 s in mpc_standalone_control_1785976976.csv) makes a
# raw round(pose_age_s / dt) flip between adjacent step counts tick to tick.
# Each flip changes how many commands predict_ahead() rolls x0 through, so x0
# jumps discontinuously between rollforward depths on consecutive solves —
# injecting step changes into the state the QP sees at exactly the frequency
# of the observed steering chatter. Two guards, applied in compute():
#   1. Low-pass pose_age_s so a single late message can't move the step count.
#   2. Hysteresis on the resulting integer: only change n_delay when the
#      filtered estimate is clearly past the midpoint of the current bin, so
#      an age hovering near a boundary doesn't dither.
_POSE_AGE_LP_ALPHA: float = 0.15   # per-tick low-pass on pose_age_s (~0.3 s settle at 20 Hz)
_N_DELAY_HYSTERESIS: float = 0.25  # steps of deadband either side of a bin boundary


def predict_ahead(x0: np.ndarray, Ad: np.ndarray, Bd: np.ndarray, pending_cmds) -> np.ndarray:
    """
    Roll the linear error-state model forward through commands already
    issued but not yet reflected in the measured pose, so the MPC solves
    against the state it will actually face instead of a stale x0.

    pending_cmds must be ordered oldest-first (the order they were issued).
    Mirrors fsae_MPCTest/sim/rollout_core.py's predict_ahead() exactly,
    including the e_psi clip — see that function's docstring for why the
    clip is needed (the e_psi -> e_y_dot coupling in Ad is only valid for
    small angles, and this rollforward has no per-step ground-truth
    correction the way the closed-loop MPC horizon does).
    """
    x_p = x0.copy()
    for u in pending_cmds:
        x_p[2] = np.clip(x_p[2], -_PREDICT_EPSI_CLIP, _PREDICT_EPSI_CLIP)
        x_p = Ad @ x_p + Bd @ u
    return x_p

# ---------------------------------------------------------------------------
# Adaptive gain helpers
# ---------------------------------------------------------------------------

def _adaptive_R_scaling(vx: float, R_base: np.ndarray) -> np.ndarray:
    """
    Speed-dependent steering cost with a saturating (Michaelis-Menten) scale.
    steer_scale = 1 + (1.5 * vx) / (6.0 + vx)
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
    Softens slew penalty in sharp corners (floor at 0.55 — raised from 0.35
    on 2026-08-05, mirroring model_utils.adaptive_R_rate in fsae_MPCTest: the
    old floor relaxed the rate-of-change cost — the one term that directly
    discourages rapid steer-sign-flipping — exactly where live standalone-ROS
    test data showed reversal chatter getting worse. See that function's
    docstring for the full rationale; keep both floors in sync manually).
    """
    scale = max(0.55, 1.0 / (1.0 + 3.0 * abs(kappa)))
    R = R_rate_base.copy()
    R[0, 0] *= scale
    return R


def _curvature(path: np.ndarray, idx: int) -> float:
    """
    Estimate signed path curvature (1/m) at waypoint idx via finite-difference.
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
    def __init__(
        self,
        dt: float = 0.05, 
        N:  int   = 25, 
    ) -> None:
        """
        Parameters
        ----------
        dt : float
            Control/prediction timestep (s). Must equal the calling node's
            control timer period (0.05 s / 20 Hz in both mpc_controller.py
            and mpc_controller_standalone.py) so the discretised model's
            predictions align with real elapsed time.
        N : int
            MPC horizon length in steps (25 -> 1.25 s lookahead at dt=0.05).
            Must match settings.N_HORIZON for tuned weights to transfer.

        Vehicle geometry/dynamics constants (lf, lr, m, Iz, Cf, Cr,
        tau_delta, tau_a) are hardcoded here rather than imported from
        vehicle_physics.VehicleParams — keep these in sync manually if the
        plant model is retuned, since this is the "live" copy used on the
        real/simulated vehicle.
        """
        self.dt = dt
        self.N  = N

        # ── Vehicle geometry & dynamics  ─────
        # FSDS-matched values.  Mass 255 kg is confirmed from the sim
        # (docs/vehicle_model.md).  The true lf/lr, Iz and Cf/Cr are NOT in the
        # FSDS repo (they live in git-LFS .uasset binaries), so these are chosen
        # from physical reasoning rather than read off:
        #   - lf < lr (CoG biased toward the front axle) makes the bicycle model
        #     UNDERSTEER (understeer gradient K_us > 0), i.e. stable at every
        #     speed.  The upstream lf=0.85 > lr=0.70 was oversteer-prone
        #     (K_us < 0, v_crit ~35 m/s) — a needless stability risk on a car
        #     whose real balance we can't measure.  Swapped for margin.
        #   - Iz ~= m*lf*lr (~152) is the standard yaw-inertia estimate; the
        #     upstream 110 under-estimated it, making the model expect a twitchier
        #     car than reality.
        # Wheelbase L = lf + lr = 1.55 m is an estimate for a FS car (the FSDS
        # collision box is 1.80 m long — an upper bound on car length, not the
        # wheelbase).  Refine all of these via system-ID on the running sim.
        #
        # 2026-08-08: vehicle_physics.py's own lf/lr/Iz were still the OLD,
        # oversteer-prone values (0.85/0.70/110) when this swap was made here
        # 2026-08-07 -- a one-sided fix that violated this project's plant/
        # model parity rule (see CLAUDE.md) and left THIS file's own Cf/Cr
        # below stale (computed against the pre-swap geometry). Both are now
        # fixed together: vehicle_physics.py's lf/lr/Iz match this file, and
        # Cf/Cr below are recomputed from the corrected geometry.
        self.lf = 0.70
        self.lr = 0.85
        self.m  = 255.0
        self.Iz = 150.0
        # Cf/Cr mirror vehicle_physics.VehicleParams.Cf/Cr — the linear
        # cornering stiffness matched to the Pacejka curve's initial slope
        # (C_eff = mu_eff * Fz_nominal * B * C * D). If the tyre model in
        # vehicle_physics.py changes, recompute these from VehicleParams()
        # and paste the new values here; don't hand-edit them independently.
        # Recomputed 2026-08-08 after fixing vehicle_physics.py's lf/lr (see
        # above) -- the previous values here were stale, computed against
        # vehicle_physics.py's pre-fix geometry (lf=0.85, lr=0.70).
        self.Cf = 29155.47766921484
        self.Cr = 19512.3421655211
        self.tau_delta = 0.08
        self.tau_a     = 0.02

        self.nx = 8
        self.nu = 2

        # Tuned parameters — offline-tuner run of 08/07/26 11:16 (see
        # fsae_MPCTest/"tuning history.txt"): "Retuned with unified scoring and
        # simulation. Decent tracking performance and speed, just not the best at
        # sudden corners."  Chosen over the later 10/07/26 set, whose own note
        # flagged braking triggering much later in FSDS than in the MPC sim.
        # Q_diag[3] (yaw-rate/e_psi_dot damping) manually corrected 2026-08-05,
        # mirroring the same fix in fsae_MPCTest/settings.py: live standalone-ROS
        # test data (mpc_standalone_control_*.csv, this file's own output) showed
        # steering sign-reversal chatter almost every ~0.05s tick, worst in
        # corners. The old value (0.1009...) was ~42:1 smaller than Q_diag[2]
        # (heading error), giving the controller almost no cost on the yaw rate
        # it uses to correct heading — a classic recipe for oscillation. Raised
        # to 2.5 (just above Q_diag[1]=2.4068) so it's no longer the smallest of
        # the five active Q entries. See settings.py's Q_diag comment for the
        # full rationale — keep these two values in sync manually.
        Q_diag      = [5.652309254831446, 0.3161236925233666, 2.798244246741331, 0.2546694567259241, 0.683532104837636, 0.0, 0.0, 0.0]
        R_diag      = [9.217407925832218, 0.3382032665811773]
        R_rate_diag = [2.9495178296071587, 9.460759229883873]

        self.Q      = np.diag(Q_diag)
        self.R      = np.diag(R_diag)
        self.R_rate = np.diag(R_rate_diag)

        # ── Hard actuator limits ───────────────────────────────────────
        # Matched exactly to the offline tuner/vehicle plant capabilities
        self.a_max = MAX_ACCEL
        self.a_max_brake = MAX_BRAKE
        self.u_min = np.array([-MAX_STEER_RAD, -self.a_max_brake]) 
        self.u_max = np.array([ MAX_STEER_RAD,  self.a_max])
        
        # Hard per-step slew-rate limit on [delta_cmd, a_cmd], enforced in
        # _build_qp in addition to the soft R_rate cost.
        #
        # Steering: expressed as a RATE (deg/s) x dt rather than a fixed
        # per-step angle, so the physical meaning survives a change of dt.
        # The old value was a bare radians(4.0)/step = 80 deg/s at dt=0.05,
        # which was far below what the plant can actually do and became the
        # binding constraint: analysis of mpc_standalone_control_1785976976.csv
        # showed 41% of steps pinned exactly on this limit, with the command
        # reversing sign at ~8 Hz — a rate-limit-induced limit cycle, not a
        # weight-tuning problem. Inverting the logged yaw rate through the
        # kinematic bicycle (delta = atan(L*r/v)) puts the ACHIEVED roadwheel
        # rate at p99 ~138 deg/s and max ~218 deg/s, so the true actuator is
        # at least ~200 deg/s. 180 deg/s is set just under that measured
        # floor: enough headroom that the constraint stops binding in normal
        # driving, while still bounding the commanded jerk. Re-measure via
        # system-ID on the running sim to tighten this estimate.
        MAX_STEER_RATE_RAD_S: float = math.radians(180.0)
        self.du_max = np.array([MAX_STEER_RATE_RAD_S * self.dt, 0.6])

        # ── Continuity memory ─────────────────────────────────────────
        self._delta_act:      float      = 0.0
        self._a_act:          float      = 0.0
        self._u_prev:         np.ndarray = np.zeros(self.nu)
        self._v_des_filtered: float | None = None

        # Previous tick's LIMITED reference heading (rad, unwrapped-compatible
        # — see _rate_limit_ref_psi), for REF_HEADING_RATE_LIMIT_ENABLED. None
        # on the first tick after start/reset: the raw value passes through
        # unlimited, mirroring _v_des_filtered's None handling above.
        self._ref_psi_prev: float | None = None

        # Rolling history of recently issued u_opt, oldest-first, used by
        # predict_ahead() to roll x0 forward through however many steps the
        # measured pose_age_s indicates (see compute()). Sized to the delay
        # cap since older entries are never needed.
        self._u_history: deque = deque(maxlen=MAX_DELAY_COMPENSATION_STEPS)

        # Filtered pose age and the currently-committed rollforward depth.
        # See the _POSE_AGE_LP_ALPHA / _N_DELAY_HYSTERESIS notes above.
        self._pose_age_filtered: float | None = None
        self._n_delay: int = 0

        self.last_telemetry: dict = {}
        self._qp: dict | None = None

    def _build_qp(self) -> None:
        """
        Constructs the CVXPY problem using parameters. Built once to maximize 20Hz throughput.
        Matches optimiser.py exactly, including soft track boundaries.
        """
        nx, nu, N = self.nx, self.nu, self.N

        Ad_p    = cp.Parameter((nx, nx), name="Ad")
        Bd_p    = cp.Parameter((nx, nu), name="Bd")
        x0_p    = cp.Parameter(nx,       name="x0")
        uprev_p = cp.Parameter(nu,       name="u_prev")
        
        sqrtQ_param  = cp.Parameter((nx, 1), nonneg=True, name="sqrtQ")
        sqrtR_param  = cp.Parameter((nu, 1), nonneg=True, name="sqrtR")
        sqrtRr_param = cp.Parameter((nu, 1), nonneg=True, name="sqrtRr")
        weighted_u_prev_param = cp.Parameter(nu, name="weighted_u_prev")

        x     = cp.Variable((nx, N + 1))
        u     = cp.Variable((nu, N))
        slack = cp.Variable(N)  # Soft lane boundary constraint

        W_slack = 10000.0

        # Dynamics constraints
        constraints = [
            x[:, 0] == x0_p,
            x[:, 1:] == Ad_p @ x[:, :-1] + Bd_p @ u,
            u >= self.u_min[:, None],
            u <= self.u_max[:, None],
            x[0, :-1] <=  3.5 + slack,
            x[0, :-1] >= -3.5 - slack,
            u[:, 0] - uprev_p <=  self.du_max,
            u[:, 0] - uprev_p >= -self.du_max,
        ]

        if N > 1:
            du_hard = cp.diff(u, axis=1)
            constraints += [
                du_hard <=  self.du_max[:, None],
                du_hard >= -self.du_max[:, None],
            ]

        # Cost Formulation (Exact match to optimiser.py)
        cost  = cp.sum(cp.sum_squares(cp.multiply(sqrtQ_param, x)))
        cost += cp.sum(cp.sum_squares(cp.multiply(sqrtR_param, u)))
        cost += W_slack * cp.sum_squares(slack)
        
        # Step-0 rate cost
        cost += cp.sum_squares(cp.multiply(sqrtRr_param[:, 0], u[:, 0]) - weighted_u_prev_param)

        # Subsequent rate cost
        if N > 1:
            du = cp.diff(u, axis=1)
            cost += cp.sum(cp.sum_squares(cp.multiply(sqrtRr_param, du)))

        prob = cp.Problem(cp.Minimize(cost), constraints)

        self._qp = {
            "prob":  prob,
            "Ad":    Ad_p,
            "Bd":    Bd_p,
            "x0":    x0_p,
            "u_prev": uprev_p,
            "sqrtQ": sqrtQ_param,
            "sqrtR": sqrtR_param,
            "sqrtRr": sqrtRr_param,
            "weighted_u_prev": weighted_u_prev_param,
            "u":     u,
        }

    def _discrete_model(self, v_x: float) -> tuple[np.ndarray, np.ndarray]:
        """
        ZOH exact discretization of the bicycle model.
        Forces dense sparsity pattern with epsilon to prevent OSQP reallocation.
        """
        v_x_safe = max(0.01, abs(v_x))
        m, Iz, lf, lr = self.m, self.Iz, self.lf, self.lr
        Cf, Cr        = self.Cf, self.Cr
        td, ta, dt    = self.tau_delta, self.tau_a, self.dt

        A_kin = np.ones((self.nx, self.nx)) * 1e-12
        A_dyn = np.ones((self.nx, self.nx)) * 1e-12

        A_kin[0, 2] = v_x_safe
        A_kin[2, 6] = v_x_safe / (lf + lr) 
        A_kin[4, 5] = 1.0
        A_kin[5, 7] = 1.0
        A_kin[6, 6] = -1.0 / td
        A_kin[7, 7] = -1.0 / ta

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

        B = np.ones((self.nx, self.nu)) * 1e-12
        B[6, 0] = 1.0 / td
        B[7, 1] = 1.0 / ta

        alpha = np.clip((v_x - 1.0) / (2.5 - 1.0), 0.0, 1.0)
        A_c = (1.0 - alpha) * A_kin + alpha * A_dyn
        
        n_aug = self.nx + self.nu 
        M     = np.zeros((n_aug, n_aug))
        M[: self.nx, : self.nx] = A_c
        M[: self.nx, self.nx :] = B 

        eM = expm(M * dt)
        return eM[: self.nx, : self.nx], eM[: self.nx, self.nx :]

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
        Calculates exact Frenet tracking errors to match offline evaluation.
        """
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

        # Orientation of the path segment
        path_yaw = math.atan2(seg[1], seg[0])

        # Robust Euclidean projection for lateral error (matches vehicle_physics.py)
        dx = fa[0] - path[base_idx][0]
        dy = fa[1] - path[base_idx][1]
        e_y_proj = dy * math.cos(path_yaw) - dx * math.sin(path_yaw)
        true_dist = math.hypot(dx, dy)
        e_y = true_dist * (1.0 if e_y_proj >= 0 else -1.0)

        # ── Reference-heading rate limit (REF_HEADING_RATE_LIMIT_ENABLED) ──
        # Mirrors fsae_MPCTest/sim/rollout_core.py's planner branch exactly:
        # only e_psi is recomputed from the limited reference; e_y above is
        # left untouched (matches the offline choice not to also limit
        # lateral tracking). See the module-level comment above
        # REF_HEADING_RATE_LIMIT_ENABLED for the mechanism and measurements.
        if REF_HEADING_RATE_LIMIT_ENABLED:
            if self._ref_psi_prev is None:
                path_yaw_limited = path_yaw
            else:
                max_step = math.radians(REF_HEADING_RISE_RATE_DEG_S) * self.dt
                delta = math.atan2(
                    math.sin(path_yaw - self._ref_psi_prev),
                    math.cos(path_yaw - self._ref_psi_prev),
                )
                delta = max(-max_step, min(max_step, delta))
                path_yaw_limited = self._ref_psi_prev + delta
            self._ref_psi_prev = path_yaw_limited
            path_yaw = path_yaw_limited

        # Heading error wrapped to [-pi, pi]
        e_psi = math.atan2(math.sin(car_yaw - path_yaw), math.cos(car_yaw - path_yaw))
        e_yd  = car_speed * math.sin(e_psi)

        # Preview curvature lookup
        preview_dist = 1.0
        preview_idx  = base_idx
        accumulated  = 0.0
        for i in range(base_idx, len(path) - 1):
            accumulated += float(np.linalg.norm(path[i + 1] - path[i]))
            if accumulated >= preview_dist:
                preview_idx = i + 1
                break
        kappa = _curvature(path, preview_idx)

        x0 = np.array([
            e_y,
            e_yd,
            e_psi,
            car_yaw_rate,    
            car_speed - desired_speed,
            0.0,             
            self._delta_act,
            self._a_act,
        ])
        
        dbg = {
            "e_y":        e_y,
            "e_psi":      e_psi,
            "e_v":        x0[4],
            "kappa":      kappa,
            "base_idx":   base_idx,
            "preview_idx": preview_idx,
        }
        return x0, kappa, dbg

    def _update_n_delay(self, pose_age_s: float) -> int:
        """
        Convert a noisy measured pose age into a stable rollforward depth.

        Low-passes pose_age_s, then moves the committed integer step count
        only when the filtered estimate has clearly crossed a bin boundary
        (by more than _N_DELAY_HYSTERESIS steps). Without this, ordinary
        control-loop jitter flips n_delay between adjacent values every few
        ticks, and each flip discontinuously changes how far predict_ahead()
        rolls x0 forward — feeding a step disturbance into the QP at the
        control rate. Returns the step count to use this tick.
        """
        age = max(0.0, float(pose_age_s))

        if self._pose_age_filtered is None:
            # First sample: adopt it outright rather than easing up from zero,
            # so startup doesn't spend ~0.3 s under-compensating.
            self._pose_age_filtered = age
            self._n_delay = int(np.clip(
                round(age / self.dt), 0, MAX_DELAY_COMPENSATION_STEPS))
            return self._n_delay

        self._pose_age_filtered += _POSE_AGE_LP_ALPHA * (age - self._pose_age_filtered)

        steps_f = self._pose_age_filtered / self.dt
        # Only leave the current bin once the estimate is past its edge plus
        # the deadband; otherwise hold, so an age sitting near a boundary
        # produces a constant n_delay instead of dithering.
        if abs(steps_f - self._n_delay) > 0.5 + _N_DELAY_HYSTERESIS:
            self._n_delay = int(np.clip(
                round(steps_f), 0, MAX_DELAY_COMPENSATION_STEPS))

        return self._n_delay

    def _solve_qp(
        self,
        x0: np.ndarray,
        Ad: np.ndarray,
        Bd: np.ndarray,
        R_scaled:      np.ndarray,
        R_rate_scaled: np.ndarray,
    ) -> np.ndarray:
        """
        Solves the MPC optimization problem utilizing warm starts.
        """
        if self._qp is None:
            self._build_qp()

        qp = self._qp
        qp["Ad"].value = Ad
        qp["Bd"].value = Bd
        qp["x0"].value = x0
        qp["u_prev"].value = self._u_prev

        # Format arrays for cp.sum_squares element-wise multiplication
        sqrtQ  = np.sqrt(np.clip(np.diag(self.Q), 1e-6, 1e6))
        sqrtR  = np.sqrt(np.clip(np.diag(R_scaled), 1e-6, 1e6))
        sqrtRr = np.sqrt(np.clip(np.diag(R_rate_scaled), 1e-6, 1e6))
        
        qp["sqrtQ"].value = sqrtQ[:, None]
        qp["sqrtR"].value = sqrtR[:, None]
        qp["sqrtRr"].value = sqrtRr[:, None]
        qp["weighted_u_prev"].value = sqrtRr * self._u_prev

        # ── Primary solve: OSQP ──────
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
            print("[MPC] Warning: OSQP OPTIMAL_INACCURATE — Proceeding with viable solution.")
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

        return np.array([self._u_prev[0], -self.a_max_brake])

    def compute(
        self,
        path:          np.ndarray,
        car_pos:       np.ndarray,
        car_yaw:       float,
        car_speed:     float,
        desired_speed: float,
        car_yaw_rate:  float = 0.0,
        pose_age_s:    float = 0.0,
    ) -> tuple[float, float, float]:
        """
        Run one full MPC control step: extract tracking error -> discretise
        the plant model at the current speed -> gain-schedule R/R_rate ->
        solve the QP -> integrate actuator lag -> convert to FSDS units.

        Parameters
        ----------
        path : np.ndarray, shape (n, 2)
            Planner centreline waypoints [x, y] in the global frame.
        car_pos : np.ndarray, shape (2,)
            Vehicle rear-axle-reference position [x, y] (global frame);
            front axle position is derived inside _error_state via self.lf.
        car_yaw : float
            Vehicle heading (rad, global frame).
        car_speed : float
            Vehicle forward speed magnitude (m/s); see _odom_cb note on how
            this is measured upstream.
        desired_speed : float
            Planner's requested speed (m/s); low-pass filtered internally.
        car_yaw_rate : float, optional
            Measured yaw rate (rad/s), defaults to 0.0 if unavailable.
        pose_age_s : float, optional
            How long ago (s) the pose above was actually measured (from the
            pose message's own timestamp, not callback receipt time — see
            mpc_controller.py's _pose_cb/_control_step). Converted to a step
            count and used to roll x0 forward through the commands already
            issued but not yet reflected in this pose (predict_ahead()),
            compensating for real, unknown/time-varying delay instead of
            assuming the measured state is current. Defaults to 0.0 (no
            compensation) for callers that don't measure it.

        Returns
        -------
        (steering, throttle, brake) : tuple of float
            steering in [-1, 1] (FSDS ControlCommand convention),
            throttle in [0, 1], brake in [0, 1] (throttle/brake mutually
            exclusive, split by the sign of a_cmd).

        Guard: if the path has fewer than 2 points, immediately returns a
        neutral/mild-braking command (0.0, 0.0, 0.5) without touching the
        QP or any internal state — the calling node's own path-staleness
        check is expected to normally catch this first
        (mpc_controller_standalone.py's Phase 2, or mpc_controller.py's
        equivalent stale-path guard).
        """
        if len(path) < 2:
            return 0.0, 0.0, 0.5   

        # Filter target speed to prevent impulse requests.
        alpha = 0.08
        if self._v_des_filtered is None:
            self._v_des_filtered = desired_speed
        self._v_des_filtered += alpha * (desired_speed - self._v_des_filtered)
        desired_speed = self._v_des_filtered

        x0, kappa, dbg = self._error_state(
            path, car_pos, car_yaw, car_speed, car_yaw_rate, desired_speed,
        )

        Ad, Bd = self._discrete_model(car_speed)

        # ── Delay compensation ───────────────────────────────────────────
        # x0 reflects the pose as measured pose_age_s seconds ago. Roll it
        # forward through however many of the recently-issued commands
        # (_u_history) fall within that window, so the QP solves against
        # the state it will actually face rather than a stale one. Clamp
        # the step count rather than trusting an arbitrarily large measured
        # age blindly (e.g. a perception hiccup) — see MAX_DELAY_COMPENSATION_STEPS.
        # The step count is filtered and hysteresis-gated rather than taken
        # raw from this tick's pose_age_s — see the _POSE_AGE_LP_ALPHA /
        # _N_DELAY_HYSTERESIS notes at module scope for why a jittering
        # n_delay is itself a source of oscillation.
        n_delay = self._update_n_delay(pose_age_s)
        if n_delay > 0 and len(self._u_history) > 0:
            pending_cmds = list(self._u_history)[-n_delay:]
            x0 = predict_ahead(x0, Ad, Bd, pending_cmds)

        R_scaled      = _adaptive_R_scaling(car_speed, self.R)
        R_rate_scaled = _adaptive_R_rate(kappa, self.R_rate)

        # Wall-clock the QP so the log can distinguish "the solver is slow"
        # from "the pipeline upstream of us is slow" — see solve_ms in
        # telemetry_logger's column reference.
        _t_solve0 = time.perf_counter()
        u_opt = self._solve_qp(x0, Ad, Bd, R_scaled, R_rate_scaled)
        solve_ms = (time.perf_counter() - _t_solve0) * 1e3
        self._u_history.append(u_opt.copy())

        # ── EXACT ZOH ACTUATOR INTEGRATION ────────────────────────────
        # Prevents explicit Euler instability when dt > tau_a
        exp_delta = math.exp(-self.dt / self.tau_delta)
        exp_a     = math.exp(-self.dt / self.tau_a)
        
        self._delta_act = self._delta_act * exp_delta + u_opt[0] * (1.0 - exp_delta)
        self._a_act     = self._a_act * exp_a         + u_opt[1] * (1.0 - exp_a)
        
        self._u_prev    = u_opt.copy()
        # ──────────────────────────────────────────────────────────────

        delta_cmd = float(np.clip(u_opt[0], -MAX_STEER_RAD, MAX_STEER_RAD))
        a_cmd     = float(u_opt[1])
        steering  = float(np.clip(-delta_cmd / MAX_STEER_RAD, -1.0, 1.0))

        if a_cmd >= 0.0:
            throttle = float(np.clip(a_cmd / self.a_max, 0.0, 1.0))
            brake    = 0.0
        else:
            throttle = 0.0
            brake    = float(np.clip(-a_cmd / self.a_max_brake, 0.0, 1.0))

        self.last_telemetry = {
            **dbg,
            # Delay diagnostics — the controller's own view of how stale its
            # inputs were and how far it rolled the state forward to compensate.
            # Logged so a live run can be checked against the offline sim's
            # DELAY_STEPS assumption instead of it being taken on trust.
            "pose_age_s":    float(pose_age_s),
            "n_delay":       int(n_delay),
            "solve_ms":      float(solve_ms),
            "car_speed":     car_speed,
            "desired_speed": desired_speed,
            "steering":      steering,
            "throttle":      throttle,
            "brake":         brake,
            "delta_cmd":     delta_cmd,
            "a_cmd":         a_cmd,
            "delta_act":     self._delta_act,
            "a_act":         self._a_act,
        }

        return steering, throttle, brake

    def reset(self) -> None:
        """
        Clears the controller's internal state history, forcing the QP solver
        to discard its warm start and actuator lag tracking. 
        """
        self._delta_act       = 0.0
        self._a_act           = 0.0
        self._u_prev          = np.zeros(self.nu)
        self._v_des_filtered  = None
        self._ref_psi_prev    = None
        self._u_history.clear()
        self._pose_age_filtered = None
        self._n_delay           = 0