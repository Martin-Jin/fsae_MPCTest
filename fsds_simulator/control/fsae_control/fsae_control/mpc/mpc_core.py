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
Provides MPCController, the class mpc_controller.py uses to turn a planner
path + current vehicle state into steering/throttle/brake at 20 Hz. That
node's `standalone_output` parameter picks how the result is used:
false forwards only steering through the shared cmd_vel interface;
true uses the full (steering, throttle, brake) triple directly — see that
file's own docstring for why. It is a self-contained, "live-solve" re-implementation
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
  mpc_controller.py — constructs an MPCController(dt=0.05, N=35) in
                    __init__ and calls .compute() every 20 Hz tick,
                    .reset() on stale path / cone-brake fail-safes.
"""

import math
import time
from collections import deque

import cvxpy as cp
import numpy as np
from scipy.linalg import expm

from fsae_control.mpc.mpc_params import DEFAULT_MPC_PARAMS, MPCParams

# Maximum physical steering deflection.  25deg matches this stack's limit
# (fsae_control.control_utils / fsds_bridge); upstream used 35deg.
MAX_STEER_RAD: float = math.radians(25.0)
MAX_ACCEL: float = 12.0
MAX_BRAKE: float = 7.0

# ── Reference-heading rate limit ─────────────────────────────────────────────
# Mirrors fsae_MPCTest/sim/rollout_core.py's REF_HEADING_RATE_LIMIT_ENABLED /
# REF_HEADING_RISE_RATE / _rate_limit_ref_psi — keep all three in sync.
# Caps how fast the tracked reference heading (path_yaw in _error_state) may
# change per tick, symmetric in both directions — unlike the speed-target
# rise limiter, there is no "always safe" direction for a heading reference.
# Disabled: holding the reference back during turn-in leaves a larger
# heading deficit to claw back later, which makes steering saturation worse.
# Re-test against a synthetic slalom path offline before re-enabling.
# Moved to MPCParams.ref_heading_rate_limit_enabled /
# .ref_heading_rise_rate_deg_s (mpc_params.py) — see those fields for the
# current values.

# ── Delay compensation ──────────────────────────────────────────────────────
# Real delay (perception + planning + control + actuation latency) is
# unknown and time-varying, unlike fsae_MPCTest's simulator-only fixed
# DELAY_STEPS. compute() is instead told how OLD the pose it's solving
# against is (pose_age_s, measured from the pose message's own timestamp —
# see mpc_controller.py/_pose_cb) and converts that into a step count itself.
# See predict_ahead() below for the same small-angle-clip rollforward
# validated in fsae_MPCTest/sim/rollout_core.py.
# The rollforward depth (n_delay) is capped at a small value because
# predict_ahead() iterates the linear model n_delay times with NO
# ground-truth correction, so pose noise compounds through every extra
# matrix multiply and the QP faithfully tracks the resulting jitter into the
# steering command. This is the same mechanism the DELAY_COMPENSATION_ENABLED
# note below records (elevated reversal rate in high-n_delay windows): a
# deep rollforward can turn measurement noise into steering thrash that
# consumes the whole corner-approach phase (steer swinging +-5-10 deg per
# tick at e_y/e_psi ~0), causing late turn-in and running wide -- a failure
# no Q/R weight change can fix, because the command is already saturated by
# noise before the corner starts.
# Disabling compensation outright was tried and was WORSE (see
# DELAY_COMPENSATION_ENABLED's RESULT note), so this caps rollforward DEPTH
# instead: still compensates typical latency, but bounds the noise
# compounding to a level measured to clearly improve peak/rms lateral error
# and steering reversal rate over deeper caps, across a wide range of
# measured pose_age_s.
#
# CAVEAT: this validates the n_delay -> reversal-rate mechanism, NOT the
# separate (and unsupported) claim that pose latency explains a
# sudden-corner regression -- a same-settings control run showed peak |e_y|
# essentially unchanged across a wide pose_age swing. The real fix is still
# stabilising pose_age_s upstream, not living on a truncated rollforward.
# Moved to MPCParams.max_delay_compensation_steps (the cap: a bigger measured
# age is clamped, not trusted blindly) and MPCParams.predict_epsi_clip (rad,
# the small-angle bound used in predict_ahead) — see mpc_params.py for the
# current values.

# ── n_delay stabilisation ───────────────────────────────────────────────────
# pose_age_s is noisy: control-loop jitter makes a raw round(pose_age_s / dt)
# flip between adjacent step counts tick to tick. Each flip changes how many
# commands predict_ahead() rolls x0 through, so x0 jumps discontinuously
# between rollforward depths on consecutive solves — injecting step changes
# into the state the QP sees at the control rate. Two guards, applied in
# compute():
#   1. Low-pass pose_age_s so a single late message can't move the step count.
#   2. Hysteresis on the resulting integer: only change n_delay when the
#      filtered estimate is clearly past the midpoint of the current bin, so
#      an age hovering near a boundary doesn't dither.
# Moved to MPCParams.pose_age_lp_alpha (per-tick low-pass on pose_age_s,
# ~0.3 s settle at 20 Hz) and MPCParams.n_delay_hysteresis (steps of deadband
# either side of a bin boundary) — see mpc_params.py for the current values.

# ── Lookahead gain-scheduling family: removed ──────────────────────────────
# Removed ~15 mechanisms that scanned forward along the path and reweighted
# today's Q/R cost based on a corner not yet reached (approach/exit boosts,
# demand normalisation, U-turn detector, straight-line adjustments, curvature
# forcing, the precomputed CornerMap fast path — full list in
# `docs/reference/README.md`'s "Corner-factor scheduler rewrite" section)
# because this MPC formulation already predicts state error at each future
# horizon step; reweighting TODAY's near-zero cost based on a forward scan
# doesn't change what the horizon predicts once the car gets there. Replaced
# by _corner_factor/_low_speed_corner_boost below (current-state only) plus
# an independent heading-error-driven accel/brake asymmetry. Mirror any
# change here into fsae_MPCTest/controller/model_utils.py per CLAUDE.md's
# parity rule.


def predict_ahead(
    x0: np.ndarray,
    Ad: np.ndarray,
    Bd: np.ndarray,
    pending_cmds,
    epsi_clip: float = 0.5,
) -> np.ndarray:
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

    epsi_clip (rad, ~28.6 deg at the default) is that small-angle bound; the
    default matches MPCParams.predict_epsi_clip, which MPCController passes
    explicitly.
    """
    x_p = x0.copy()
    for u in pending_cmds:
        x_p[2] = np.clip(x_p[2], -epsi_clip, epsi_clip)
        x_p = Ad @ x_p + Bd @ u
    return x_p

# ---------------------------------------------------------------------------
# Adaptive gain helpers
# ---------------------------------------------------------------------------

def _adaptive_R_scaling(vx: float, R_base: np.ndarray) -> np.ndarray:
    """
    Speed-dependent steering cost with a saturating (Michaelis-Menten) scale.
    steer_scale = 1 + (1.5 * vx) / (6.0 + vx)

    accel_scale is disabled (fixed at 1.0): a speed-dependent accel_scale
    (e.g. 1 + 0.05*vx) makes R[1,1] rise with speed exactly where
    corner-entry braking needs to be strongest, and relax again as the car
    decelerates mid-approach even though heading error/curvature are still
    climbing -- fighting the braking R_diag[1] tuning is trying to loosen.
    R[1,1] is governed by R_diag[1] alone, independent of vx.
    """
    vx = max(vx, 0.5)
    steer_scale = 1.0 + (1.5 * vx) / (6.0 + vx)
    accel_scale = 1.0
    R = R_base.copy()
    R[0, 0] *= steer_scale
    R[1, 1] *= accel_scale
    return R


def _adaptive_R_rate(
    kappa: float,
    R_rate_base: np.ndarray,
    enable_in_corners: bool = True,
    during_floor: float = 0.625,
) -> np.ndarray:
    """
    Curvature-dependent steering-jerk softening: relaxes the steering
    rate-of-change cost in sharp corners (floor during_floor, default
    MPCParams.adaptive_r_rate_during_floor), driven by CURRENT-position
    kappa, so the controller isn't over-penalised for the extra steering
    rate a tight corner demands. Mirrors model_utils.adaptive_R_rate in
    fsae_MPCTest — keep both floors in sync manually.

    enable_in_corners: TEMPORARY/EXPERIMENTAL, NOT VALIDATED -- re-verify
    before relying on the False branch below. True
    (default) preserves the softening above -- R_rate reduction stays
    active in corners. False uses a kappa_straight=0.03 "cornering" cutoff:
    once |kappa| exceeds it, softening is switched off and R_rate[0,0] gets
    the full, unscaled baseline cost instead -- deliberately undoing the
    softening this function exists to provide. (Renamed from
    disable_in_corners, whose True/False polarity was inverted from what
    the name suggested.)

    during_floor is kept relatively shallow (not deeper): a deeper floor
    lets steering oscillate through zero several times per second
    mid-corner (e.g. +25 -> -20 -> +14 -> -9 deg across ~0.3s) while
    e_y/e_psi stay small -- classic under-damped steering-rate hunt, not a
    tracking-error problem. The fix is less softening of the rate cost
    while actually turning, not more lateral/heading authority -- do not
    deepen this floor without re-checking for that oscillation.

    Only the current-position during-floor is implemented; there is no
    forward-scan entering-floor.
    """
    kappa_straight = 0.03
    if not enable_in_corners and abs(kappa) > kappa_straight:
        scale = 1.0
    else:
        scale = max(during_floor, 1.0 / (1.0 + 3.0 * abs(kappa)))
    R = R_rate_base.copy()
    R[0, 0] *= scale
    return R


def _steer_rate_anti_hunt(
    kappa: float,
    e_y: float,
    R_rate_base: np.ndarray,
    enabled: bool,
    e_psi: float = 0.0,
    boost_max: float = 6.0,
) -> np.ndarray:
    """
    TEMPORARY/EXPERIMENTAL, NOT VALIDATED: heavily penalise steering
    rate-of-change on top of _adaptive_R_rate's existing curvature softening,
    strongest when the car is centred (|e_y| small), well-aligned (|e_psi|
    small), AND not currently curving (kappa small). Mirrors
    model_utils.steer_rate_anti_hunt in fsae_MPCTest -- keep both constants
    in sync. enabled=False returns R_rate_base untouched.

    Continuous, not a hard AND-gated threshold: a discontinuous step would
    risk the same QP-solver-iteration-spike problem the
    enable_in_corners/kappa_straight history above already found from
    threshold cutoffs on curvature, so straight-line hunting is instead
    penalised more strongly via a higher continuous ceiling.
    boost_kappa, boost_ey, and boost_epsi each saturate independently
    toward 1.0 as their input shrinks toward 0 (same saturating-curve style
    as _adaptive_R_rate's own floor); their product is the applied scale,
    so the full boost_max (default MPCParams.anti_hunt_boost_max) only
    applies when all three are near their "straight, centred, and aligned"
    ideal, and it fades smoothly -- never snaps -- as any one of them
    grows.

    e_psi (radians, NOT the degrees used in telemetry/logging -- same units
    _error_state's x0[2]/e_psi already use internally) is included because
    without it, a car that enters a straight MISALIGNED (large |e_psi|,
    small |e_y| -- e.g. just exited a corner still pointed the wrong way)
    would get the full straight-line boost anyway, since kappa/e_y alone
    can't distinguish "straight and correctly aligned" from "straight but
    needs to yaw back into line" -- making exactly the correction it needs
    artificially expensive. k_epsi=23.0 sets half-fade at ~2.5 deg of e_psi.

    boost_kappa/boost_ey/boost_epsi are current-state signals only (no
    forward-scan term).
    """
    if not enabled:
        return R_rate_base
    # Relaxed 2026-08-19 (halved from 60.0/30.0/23.0): the original constants
    # faded the boost out too fast on genuinely gentle curves -- boost_kappa
    # was already down to ~0.45 by |kappa|=0.02 (a ~50 m-radius bend), so
    # R_rate[0,0] had mostly relaxed back toward baseline exactly where
    # residual steering jitter was still visible. Halving each k_* doubles
    # the |kappa|/|e_y|/|e_psi| each factor reaches before dropping to half
    # its max contribution (kappa: ~0.017 -> ~0.033 1/m; e_y: ~3.3 -> ~6.7 cm;
    # e_psi: ~2.5 -> ~5.0 deg). Applies to both controllers -- nmpc_core.py
    # imports this function verbatim, not a separate copy.
    k_kappa, k_ey, k_epsi = 30.0, 15.0, 11.5
    boost_kappa = 1.0 / (1.0 + k_kappa * abs(kappa))
    boost_ey    = 1.0 / (1.0 + k_ey * abs(e_y))
    boost_epsi  = 1.0 / (1.0 + k_epsi * abs(e_psi))
    scale = 1.0 + (boost_max - 1.0) * boost_kappa * boost_ey * boost_epsi
    R = R_rate_base.copy()
    R[0, 0] *= scale
    return R


def _reversal_penalty_boost(
    u_prev_steer: float,
    R_rate_base: np.ndarray,
    enabled: bool,
    boost_max: float = 4.0,
    k: float = 8.0,
) -> np.ndarray:
    """
    TEMPORARY/EXPERIMENTAL, NOT VALIDATED: soft constraint against steering
    REVERSALS (a tick-to-tick sign flip), approximated inside the convex QP
    by boosting R_rate[0,0] whenever LAST tick's steering command
    (u_prev_steer, rad) was already close to zero -- the one state a
    reversal must pass through, since delta_cmd is continuous. A reversal
    can't be detected directly inside one solve (it depends on this tick's
    OWN decision, the thing being optimised), so this penalises the
    precondition instead: the closer steering already sits to zero, the
    more it costs to change it further this tick, making a full sign flip
    specifically (as opposed to a same-side ramp toward/away from zero)
    disproportionately expensive relative to a swing of the same size made
    from a large starting angle.

    Same saturating-curve style as _steer_rate_anti_hunt (single input here,
    not a product of several) so it fades continuously rather than snapping,
    and composes the same way: applied multiplicatively on top of whatever
    _adaptive_R_rate/_steer_rate_anti_hunt/the corner blend already produced,
    never replacing them. enabled=False returns R_rate_base untouched.

    k=8.0 (rad^-1) sets half-boost at ~7.2 deg of PREVIOUS steering (a
    reversal starting from near-centre gets close to the full boost_max;
    one starting from a large existing angle -- already unlikely to flip
    sign in one 50ms tick without an equally large du -- is barely
    affected). Deliberately keyed on u_prev, not the CURRENT solve's u[0,0]
    (a QP variable): using the variable itself would make the cost
    non-convex (a rational function of the decision), whereas u_prev is a
    known constant by solve time, keeping this an ordinary quadratic term.
    """
    if not enabled:
        return R_rate_base
    boost_near_zero = 1.0 / (1.0 + k * abs(u_prev_steer))
    scale = 1.0 + (boost_max - 1.0) * boost_near_zero
    R = R_rate_base.copy()
    R[0, 0] *= scale
    return R


def _adaptive_Q_scaling(
    e_y: float, Q_base: np.ndarray, enabled: bool,
) -> np.ndarray:
    """
    Soften the lateral-error cost Q[0,0] when already close to the
    centreline, to reduce small-error hunting/chatter. Mirrors
    model_utils.adaptive_Q_scaling in fsae_MPCTest — see that function's
    docstring for the full mechanism and why this is disabled by default.
    enabled=False returns Q_base untouched.

    Only the current-state ey_lo/ey_hi/floor softening on CURRENT |e_y|
    below is implemented; there is no forward-scan relaxation term.
    """
    if not enabled:
        return Q_base
    ey_lo, ey_hi, floor = 0.05, 0.3, 0.5
    ey_abs = abs(e_y)
    if ey_abs >= ey_hi:
        scale = 1.0
    elif ey_abs <= ey_lo:
        scale = floor
    else:
        scale = floor + (1.0 - floor) * (ey_abs - ey_lo) / (ey_hi - ey_lo)
    Q = Q_base.copy()
    Q[0, 0] *= scale
    return Q


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


def _corner_factor(kappa: float, k: float) -> float:
    """
    0 (straight) -> 1 (full corner), a single continuous saturating curve
    of the CURRENT |kappa| (the ~1m-preview curvature _error_state already
    computes every tick, same signal _adaptive_R_rate/_steer_rate_anti_hunt
    use). Deliberately the SAME functional shape for both rising (entry)
    and falling (exit) curvature -- no separate decay-distance timer, no
    hysteresis state: this is a pure function of the current instantaneous
    signal, replacing the whole deleted lookahead approach/exit-boost
    family (see the module comment near the top of this file). k is
    MPCParams.corner_factor_k, the curve's sharpness.
    """
    return 1.0 - 1.0 / (1.0 + k * abs(kappa))


def _blend(straight_val: float, corner_val: float, corner_frac: float) -> float:
    """
    Simple linear interpolation from straight_val (corner_frac=0) to
    corner_val (corner_frac=1). Shared helper for every current-state
    Q/R_rate weight schedule in compute() -- see _corner_factor/
    _low_speed_corner_boost for how corner_frac itself is built.
    """
    return straight_val + (corner_val - straight_val) * corner_frac


def _low_speed_corner_boost(
    car_speed: float, corner_factor: float,
    v_half: float, max_extra: float,
) -> float:
    """
    Extra push in the SAME direction as corner_factor's "full corner"
    endpoint (i.e. an ADDITIONAL fraction, not a separate blend), active
    ONLY when corner_factor > 0 AND speed is low -- multiplicatively gated
    on corner_factor so this is an exact no-op on a straight regardless of
    speed. This is what makes it safe where the deleted
    _low_speed_steer_rate_boost was NOT: that function fired on low speed
    ALONE, with no curvature/lookahead signal to distinguish wanted
    low-speed turn-in from unwanted post-exit wobble, and live testing
    found it taxed both indistinguishably. Gating on corner_factor instead
    of a forward scan keeps this current-state: it only ever pushes harder
    on a corner the car is ALREADY turning through, never anticipates one.

    car_speed=0 gives the full max_extra (scaled by corner_factor); the
    boost falls off toward 0 as speed rises past v_half (the speed at
    which half of max_extra remains), same saturating-curve style used
    throughout this file.
    """
    v = max(abs(car_speed), 0.0)
    speed_frac = v_half / (v_half + v) if v_half > 0.0 else 0.0
    return corner_factor * max_extra * speed_frac


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
        N:  int   = 35,
        params: MPCParams = None,
    ) -> None:
        """
        Parameters
        ----------
        dt : float
            Control/prediction timestep (s). Must equal the calling node's
            control timer period (0.05 s / 20 Hz in mpc_controller.py) so
            the discretised model's predictions align with real elapsed
            time.
        N : int
            MPC horizon length in steps (35 -> 1.75 s lookahead at dt=0.05).
            Must match settings.N_HORIZON for tuned weights to transfer.
        params : MPCParams, optional
            Every tunable weight/gain/flag (see mpc_params.py). Defaults to
            DEFAULT_MPC_PARAMS, whose field defaults are numerically identical
            to the values this constructor used to hardcode inline — so
            MPCController() with no params behaves exactly as before.

        Vehicle geometry/dynamics constants (lf, lr, m, Iz, Cf, Cr,
        tau_delta, tau_a) are hardcoded here rather than imported from
        vehicle_physics.VehicleParams — keep these in sync manually if the
        plant model is retuned, since this is the "live" copy used on the
        real/simulated vehicle.
        """
        self.dt = dt
        self.N  = N
        self.params = params if params is not None else DEFAULT_MPC_PARAMS

        # ── Vehicle geometry & dynamics  ─────
        # FSDS-matched values. Mass 255 kg is confirmed from the sim
        # (docs/vehicle_model.md). The true lf/lr, Iz and Cf/Cr are NOT in the
        # FSDS repo (they live in git-LFS .uasset binaries), so these are chosen
        # from physical reasoning rather than read off:
        #   - lf < lr (CoG biased toward the front axle) makes the bicycle model
        #     UNDERSTEER (understeer gradient K_us > 0), i.e. stable at every
        #     speed — a needless oversteer stability risk otherwise, on a car
        #     whose real balance we can't measure.
        #   - Iz ~= m*lf*lr (~152) is the standard yaw-inertia estimate.
        # Wheelbase L = lf + lr = 1.55 m is an estimate for a FS car (the FSDS
        # collision box is 1.80 m long — an upper bound on car length, not the
        # wheelbase). Refine all of these via system-ID on the running sim.
        # Must match vehicle_physics.VehicleParams' lf/lr/Iz (CLAUDE.md's
        # plant/model parity rule) — keep in sync manually.
        self.lf = 0.70
        self.lr = 0.85
        self.m  = 255.0
        self.Iz = 150.0
        # Cf/Cr mirror vehicle_physics.VehicleParams.Cf/Cr — the linear
        # cornering stiffness matched to the Pacejka curve's initial slope
        # (C_eff = mu_eff * Fz_nominal * B * C * D). If the tyre model in
        # vehicle_physics.py changes, recompute these from VehicleParams()
        # and paste the new values here; don't hand-edit them independently.
        self.Cf = 29155.47766921484
        self.Cr = 19512.3421655211
        self.tau_delta = 0.08
        self.tau_a     = 0.02

        self.nx = 8
        self.nu = 2

        # Values live in mpc_params.py (MPCParams.q_*/r_*/r_rate_*); keep them
        # numerically identical to fsae_MPCTest/settings.py's
        # Q_diag/R_diag/R_rate_diag and the same three lists in
        # fsds_simulator's copy of this file (see CLAUDE.md's parity rule).
        #
        # Q_diag index -> state penalised (states x are [e_y, e_yd, e_psi, r,
        # e_v, e_a, delta_act, a_act], see the module docstring above):
        #   [0] e_y        lateral deviation from path centreline (m)
        #   [1] e_yd       rate of change of lateral deviation (m/s)
        #   [2] e_psi      heading error relative to path tangent (rad)
        #   [3] r          yaw rate (rad/s)
        #   [4] e_v        speed error: car_speed - desired_speed (m/s)
        #   [5] e_a        unused (always 0.0, kept for structural consistency only)
        #   [6] delta_act  actuator-lagged steering angle (rad) -- always 0.0
        #   [7] a_act      actuator-lagged acceleration (m/s^2) -- always 0.0
        Q_diag      = [
            self.params.q_e_y,
            self.params.q_e_yd,
            self.params.q_e_psi,
            self.params.q_r,
            self.params.q_e_v,
            0.0,   # e_a       — always 0.0, not parameterised (see above)
            0.0,   # delta_act — always 0.0, not parameterised
            0.0,   # a_act     — always 0.0, not parameterised
        ]
        # R_diag index -> input penalised (inputs u are [delta_cmd, a_cmd]):
        #   [0] delta_cmd  steering command effort (rad)
        #   [1] a_cmd      acceleration command effort (m/s^2) -- nominal
        #       value only (r_a_accel); the QP itself splits this into
        #       independent accel/brake weights via cp.pos/cp.neg in
        #       _build_qp, since a_cmd's SIGN determines which of
        #       params.r_a_accel/r_a_brake actually applies. self.R[1,1]
        #       is never read by _adaptive_R_scaling (index-0-only) or
        #       _solve_qp (which takes r_a_accel/r_a_brake directly) --
        #       it exists only for telemetry/reporting parity with R[0,0].
        R_diag      = [self.params.r_delta, self.params.r_a_accel]
        # R_rate_diag index -> input RATE-OF-CHANGE penalised (tick-to-tick
        # jerk, not the input itself):
        #   [0] delta_cmd  steering rate of change
        #   [1] a_cmd      acceleration rate of change
        R_rate_diag = [self.params.r_rate_delta, self.params.r_rate_a]

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
        # 180 deg/s was set just under the plant's measured achievable
        # roadwheel rate (~200 deg/s, via system-ID) — see
        # fsae_MPCTest/`docs/reference/README.md`'s "Slew-rate limit
        # (du_max)" section for the full history.
        #
        # Do not raise without re-measuring; a higher value previously
        # regressed smoothness metrics. UNMEASURED either way -- the sync
        # doc explicitly says to refine this "via system-ID on the running
        # sim", not by picking a number; re-measure properly before
        # trusting either value long-term, and update both sides (this file
        # + fsae_MPCTest's controller/optimiser.py / vehicle_physics.py
        # du_max) together per that doc's parity rule.
        MAX_STEER_RATE_RAD_S: float = math.radians(180.0)
        self.du_max = np.array([MAX_STEER_RATE_RAD_S * self.dt, 0.6])

        # TERMINAL_Q_SCALE (settings.py, fsae_MPCTest) — extra weight on the
        # final predicted state x[:,N], on top of the uniform per-step weight
        # it already gets. 1.0 = no-op, matching every weight set tuned
        # against this controller so far. Inlined here per the standing
        # no-settings.py-on-the-car rule (now via MPCParams.terminal_q_scale
        # in mpc_params.py); must be kept numerically identical to
        # fsae_MPCTest's copy.
        self.terminal_scale = self.params.terminal_q_scale

        # ADAPTIVE_Q_SCALING_ENABLED (settings.py, fsae_MPCTest) — see
        # _adaptive_Q_scaling above. Gated by MPCParams.adaptive_q_scaling_enabled.
        self.adaptive_q_scaling_enabled = self.params.adaptive_q_scaling_enabled

        # STEER_RATE_ANTI_HUNT_ENABLED (settings.py, fsae_MPCTest) — see
        # _steer_rate_anti_hunt above. Experimental, not validated against
        # a live log or VALIDATION_SUITE. Default off, inlined per the
        # standing no-settings.py-on-the-car rule; keep in sync with
        # fsae_MPCTest's copy.
        self.steer_rate_anti_hunt_enabled = self.params.steer_rate_anti_hunt_enabled

        # _reversal_penalty_boost above. EXPERIMENTAL, not validated against
        # a live log or VALIDATION_SUITE. Default off, inlined per the
        # standing no-settings.py-on-the-car rule; keep in sync with
        # fsae_MPCTest's copy.
        self.reversal_penalty_enabled = self.params.reversal_penalty_enabled

        # ADAPTIVE_R_RATE_ENABLE_IN_CORNERS (settings.py, fsae_MPCTest) —
        # see _adaptive_R_rate's enable_in_corners param above (renamed from
        # disable_in_corners, whose polarity was inverted from what the name
        # suggested). True (default) keeps R_rate reduction ACTIVE in
        # corners via the continuous curve (no threshold, no discontinuity).
        # False uses the kappa_straight cutoff to switch softening off past
        # it, restoring full baseline R_rate[0,0] -- raising kappa_straight
        # causes severe lag specifically in corners: the discontinuous
        # R_rate[0,0] jump at the kappa_straight crossing likely spikes QP
        # solver iterations / invalidates warm-starts every tick near the
        # threshold. Do not switch this to False without addressing that.
        # Inlined per the standing no-settings.py-on-the-car rule; keep in
        # sync with fsae_MPCTest's copy.
        self.adaptive_r_rate_enable_in_corners = (
            self.params.adaptive_r_rate_enable_in_corners
        )

        # DELAY_COMPENSATION_ENABLED — TEMPORARY/EXPERIMENTAL, NOT VALIDATED.
        # Master switch for _update_n_delay()/predict_ahead() (see both
        # above). Exists to test a hypothesis: n_delay has been observed
        # swinging over a wide range in short (~10s) blocks on live
        # standalone logs, and steering-reversal rate
        # on straight sections was far higher during high-n_delay windows
        # despite e_y barely moving -- i.e. predict_ahead()'s
        # rollforward may itself be injecting the chatter rather than
        # compensating for real lag. False skips
        # _update_n_delay()/predict_ahead() entirely: x0 is solved raw, with
        # zero delay compensation, to isolate whether this mechanism is the
        # cause. Not a fix either way -- if disabling it removes the
        # chatter, the next step is to find why pose_age_s/n_delay swing so
        # widely in the first place, not to leave compensation off
        # permanently.
        #
        # RESULT: disabling delay compensation outright measured WORSE, not
        # better -- rmse, steering saturation ratio and |e_psi| all got
        # substantially worse, with much of the run pinned at the 25 deg
        # steer limit. pose_age_s did NOT stabilise with compensation off
        # (still swinging over the same wide range in the same short
        # blocks) -- it just went uncorrected instead of jittering the
        # correction. Steering-reversal rate did
        # drop, so the mechanism isn't imaginary, but the net
        # effect of disabling compensation outright is worse than the
        # chatter it was meant to isolate. Keep this True; the real fix is
        # stabilising pose_age_s upstream, not leaving this off.
        self.delay_compensation_enabled = self.params.delay_compensation_enabled

        # ── Continuity memory ─────────────────────────────────────────
        self._delta_act:      float      = 0.0
        self._a_act:          float      = 0.0
        self._u_prev:         np.ndarray = np.zeros(self.nu)
        self._v_des_filtered: float | None = None

        # Previous tick's LIMITED reference heading (rad, unwrapped-compatible
        # — see _rate_limit_ref_psi), for
        # MPCParams.ref_heading_rate_limit_enabled. None
        # on the first tick after start/reset: the raw value passes through
        # unlimited, mirroring _v_des_filtered's None handling above.
        self._ref_psi_prev: float | None = None

        # Precomputed shaped heading-lead profile (see
        # late_turn_in_investigation.md Part 8/9, set_heading_profile()
        # below). None until set by the owning node, gated on the
        # node-level use_precomputed_heading_profile parameter -- when
        # None, _error_state falls back to today's geometric path tangent
        # for e_psi's reference, a byte-for-byte no-op.
        self._heading_profile: np.ndarray | None = None

        # Rolling history of recently issued u_opt, oldest-first, used by
        # predict_ahead() to roll x0 forward through however many steps the
        # measured pose_age_s indicates (see compute()). Sized to the delay
        # cap since older entries are never needed.
        self._u_history: deque = deque(
            maxlen=self.params.max_delay_compensation_steps
        )

        # Filtered pose age and the currently-committed rollforward depth.
        # See the MPCParams.pose_age_lp_alpha / .n_delay_hysteresis notes above.
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
        # a_cmd (u[1,:]) effort cost is split by sign instead of going
        # through sqrtR_param[1] -- see the "Cost Formulation" block below
        # for why. sqrtR_param[1] is left unused/unset for this row (only
        # row 0, delta_cmd, is actually read from it).
        r_a_accel_param = cp.Parameter(nonneg=True, name="r_a_accel")
        r_a_brake_param = cp.Parameter(nonneg=True, name="r_a_brake")

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
        # Terminal cost: extra weight on x[:,N] only. self.terminal_scale=1.0
        # makes this a no-op (see __init__ for the full explanation).
        if self.terminal_scale != 1.0:
            cost += (self.terminal_scale - 1.0) * cp.sum_squares(
                cp.multiply(sqrtQ_param[:, 0], x[:, N])
            )
        # Input effort cost. delta_cmd (row 0) uses the shared sqrtR_param
        # path as before. a_cmd (row 1) is split by sign: cp.pos(u)^2 and
        # cp.neg(u)^2 are each individually convex (composition of the
        # convex increasing `square` with convex pos/neg), so summing them
        # with independent weights is still DCP, and pos(x)^2+neg(x)^2 ==
        # x^2 identically (exactly one of pos/neg is nonzero for any real
        # x) -- so r_a_accel==r_a_brake reproduces the old single-r_a cost
        # bit-for-bit. This lets braking be tuned independently from
        # acceleration without new QP variables or constraints: see
        # `docs/reference/control_mechanisms.md`'s "Accel/brake effort weight split".
        cost += cp.sum_squares(cp.multiply(sqrtR_param[0, 0], u[0, :]))
        cost += r_a_accel_param * cp.sum_squares(cp.pos(u[1, :]))
        cost += r_a_brake_param * cp.sum_squares(cp.neg(u[1, :]))
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
            "r_a_accel": r_a_accel_param,
            "r_a_brake": r_a_brake_param,
            "weighted_u_prev": weighted_u_prev_param,
            "u":     u,
        }

    def _discrete_model(self, v_x: float) -> tuple[np.ndarray, np.ndarray]:
        """
        ZOH exact discretization of the speed-blended kinematic/dynamic
        bicycle model. Forces dense sparsity pattern with epsilon to prevent
        OSQP reallocation. Byte-for-byte the same model as
        bicycle_model.get_8state_discrete_model — see that function's
        docstring for the full kinematic/dynamic blend derivation and the
        ZOH/sparsity reasoning; kept here uncommented-per-line only to avoid
        two independently-maintained comment sets drifting apart.
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

        # Dynamic (tyre-force) model — see bicycle_model.py's DYNAMIC BICYCLE
        # MODEL section for the linearised-bicycle derivation each entry below
        # comes from; same terms, same axle convention (2*Cf/2*Cr).
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
        B[6, 0] = 1.0 / td   # steering command drives steering-lag integrator
        B[7, 1] = 1.0 / ta   # accel command drives accel-lag integrator

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
        car_vy:        float = 0.0,
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

        # Perpendicular projection for lateral error (matches vehicle_physics.py's
        # plant_to_tracking_error(): e_y = e_y_proj directly) — NOT the full
        # Euclidean distance to the nearest path point, which only equals the
        # correct value when that point happens to be exactly abeam of the
        # car (otherwise overstates e_y, e.g. a 5 m along-track / 0.2 m
        # lateral offset would read back as e_y ~= 5.0 instead of 0.2).
        dx = fa[0] - path[base_idx][0]
        dy = fa[1] - path[base_idx][1]
        e_y_proj = dy * math.cos(path_yaw) - dx * math.sin(path_yaw)
        e_y = e_y_proj

        # ── Precomputed shaped heading-lead profile (Part 8/9) ──────────────
        # Substitutes ONLY the reference e_psi is measured against -- e_y
        # above already used the GEOMETRIC path_yaw for its projection and
        # is unaffected, same precedent as ref_heading_rate_limit_enabled's
        # own "only e_psi changes" pattern immediately below. Length-checked
        # per-tick (not just at set_heading_profile() time) since `path`
        # itself isn't known until compute() is called with it.
        if (self._heading_profile is not None
                and len(self._heading_profile) == len(path)):
            path_yaw = float(self._heading_profile[base_idx])

        # ── Reference-heading rate limit (ref_heading_rate_limit_enabled) ──
        # Mirrors fsae_MPCTest/sim/rollout_core.py's planner branch exactly:
        # only e_psi is recomputed from the limited reference; e_y above is
        # left untouched (matches the offline choice not to also limit
        # lateral tracking). See the module-level comment about the
        # reference-heading rate limit for the mechanism and measurements.
        if self.params.ref_heading_rate_limit_enabled:
            if self._ref_psi_prev is None:
                path_yaw_limited = path_yaw
            else:
                max_step = (
                    math.radians(self.params.ref_heading_rise_rate_deg_s) * self.dt
                )
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
        # Matches vehicle_physics.py's plant_to_tracking_error /
        # rollout_core.py's identical inline formula: e_y_dot needs both
        # body-frame velocity components, not just forward speed — omitting
        # car_vy silently drops this term whenever the car has real sideslip
        # (car_vy defaults to 0.0 for callers that don't measure it).
        e_yd  = car_speed * math.sin(e_psi) + car_vy * math.cos(e_psi)

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
        (by more than MPCParams.n_delay_hysteresis steps). Without this, ordinary
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
                round(age / self.dt),
                0, self.params.max_delay_compensation_steps))
            return self._n_delay

        self._pose_age_filtered += (
            self.params.pose_age_lp_alpha * (age - self._pose_age_filtered)
        )

        steps_f = self._pose_age_filtered / self.dt
        # Only leave the current bin once the estimate is past its edge plus
        # the deadband; otherwise hold, so an age sitting near a boundary
        # produces a constant n_delay instead of dithering.
        if abs(steps_f - self._n_delay) > 0.5 + self.params.n_delay_hysteresis:
            self._n_delay = int(np.clip(
                round(steps_f), 0, self.params.max_delay_compensation_steps))

        return self._n_delay

    def set_heading_profile(self, psi_target: np.ndarray | None) -> None:
        """
        Store a precomputed, per-waypoint SHAPED heading reference (see
        late_turn_in_investigation.md Part 8/9's design), row-aligned with
        `path` (the SAME array `set_static_path` was given, from the SAME
        CSV load_path_heading_profile_csv read). Called ONCE by the owning
        node, gated on the node-level `use_precomputed_heading_profile`
        parameter -- NEVER called from compute() itself.

        This array requires no computation here (the shaping already
        happened offline in
        tuner/tools/raceline_optimizer.py) -- this is a plain store, not a
        build step. `_error_state` looks it up by `base_idx` in place of
        the geometric `path_yaw` it computes today, ONLY for e_psi's
        reference; e_y's projection is unaffected (mirrors
        ref_heading_rate_limit_enabled's existing "only e_psi changes"
        precedent).

        psi_target=None (or a length mismatch against whatever `path` is
        passed to compute()/`_error_state` later) falls back to today's
        geometric heading, a byte-for-byte no-op -- the length check
        happens per-tick in `_error_state` since `path` itself isn't known
        until then.
        """
        self._heading_profile = psi_target

    def _solve_qp(
        self,
        x0: np.ndarray,
        Ad: np.ndarray,
        Bd: np.ndarray,
        R_scaled:      np.ndarray,
        R_rate_scaled: np.ndarray,
        Q_scaled:      np.ndarray | None = None,
        r_a_accel:     float | None = None,
        r_a_brake:     float | None = None,
    ) -> np.ndarray:
        """
        Solves the MPC optimization problem utilizing warm starts.

        r_a_accel/r_a_brake: independent effort weights for u[1,:] (a_cmd)
        by sign, applied via _build_qp's cp.pos/cp.neg cost split. R_scaled's
        own [1,1] entry is NOT used for a_cmd (only [0,0], delta_cmd, is
        read from sqrtR) -- callers still pass a full R_scaled for shape/
        parity but must supply r_a_accel/r_a_brake to actually set the
        accel/brake weights. compute() passes the heading-error-scaled
        r_a_accel_eff/r_a_brake_eff (see MPCParams.epsi_ra_half_rad) rather
        than the raw params directly.
        """
        if self._qp is None:
            self._build_qp()

        qp = self._qp
        qp["Ad"].value = Ad
        qp["Bd"].value = Bd
        qp["x0"].value = x0
        qp["u_prev"].value = self._u_prev

        # Format arrays for cp.sum_squares element-wise multiplication
        Q_for_solve = self.Q if Q_scaled is None else Q_scaled
        sqrtQ  = np.sqrt(np.clip(np.diag(Q_for_solve), 1e-6, 1e6))
        sqrtR  = np.sqrt(np.clip(np.diag(R_scaled), 1e-6, 1e6))
        sqrtRr = np.sqrt(np.clip(np.diag(R_rate_scaled), 1e-6, 1e6))

        qp["sqrtQ"].value = sqrtQ[:, None]
        qp["sqrtR"].value = sqrtR[:, None]
        qp["sqrtRr"].value = sqrtRr[:, None]
        qp["r_a_accel"].value = float(np.clip(
            self.params.r_a_accel if r_a_accel is None else r_a_accel, 1e-6, 1e6))
        qp["r_a_brake"].value = float(np.clip(
            self.params.r_a_brake if r_a_brake is None else r_a_brake, 1e-6, 1e6))
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

        # Both solvers failed: hold the last commanded steering angle (avoids
        # a sudden straighten-out mid-corner) and command full brake (a
        # fail-safe deceleration) rather than coasting or repeating a
        # possibly-bad last accel command.
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
        car_vy:        float = 0.0,
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
            Vehicle forward (body-frame vx) speed (m/s); see _odom_cb note on
            how this is measured upstream. Used for the plant discretisation
            and gain scheduling, which are both parameterised on forward
            speed alone — see rollout_core.py's identical vx_true usage.
        desired_speed : float
            Planner's requested speed (m/s); low-pass filtered internally.
        car_yaw_rate : float, optional
            Measured yaw rate (rad/s), defaults to 0.0 if unavailable.
        car_vy : float, optional
            Body-frame lateral velocity (m/s), defaults to 0.0. Used only in
            _error_state's e_yd = vx*sin(e_psi) + vy*cos(e_psi) (matches
            vehicle_physics.py's plant_to_tracking_error / rollout_core.py's
            identical inline formula exactly). Callers that only have a
            speed magnitude (no separate vx/vy) should leave this at 0.0
            rather than passing the magnitude here.
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
        check is expected to normally catch this first (mpc_controller.py's
        Phase 2).
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
            path, car_pos, car_yaw, car_speed, car_yaw_rate, desired_speed, car_vy,
        )

        Ad, Bd = self._discrete_model(car_speed)

        # ── Delay compensation ───────────────────────────────────────────
        # x0 reflects the pose as measured pose_age_s seconds ago. Roll it
        # forward through however many of the recently-issued commands
        # (_u_history) fall within that window, so the QP solves against
        # the state it will actually face rather than a stale one. Clamp
        # the step count rather than trusting an arbitrarily large measured
        # age blindly (e.g. a perception hiccup) — see
        # MPCParams.max_delay_compensation_steps.
        # The step count is filtered and hysteresis-gated rather than taken
        # raw from this tick's pose_age_s — see the MPCParams.pose_age_lp_alpha /
        # .n_delay_hysteresis notes at module scope for why a jittering
        # n_delay is itself a source of oscillation.
        if self.delay_compensation_enabled:
            n_delay = self._update_n_delay(pose_age_s)
            if n_delay > 0 and len(self._u_history) > 0:
                pending_cmds = list(self._u_history)[-n_delay:]
                x0 = predict_ahead(
                    x0, Ad, Bd, pending_cmds,
                    epsi_clip=self.params.predict_epsi_clip,
                )
        else:
            n_delay = 0

        # ── Adaptive-feature trace ────────────────────────────────────────
        # Every adaptive multiplier below is recorded into `adapt` as it is
        # applied, then merged into last_telemetry so a live log shows WHICH
        # feature fired and BY HOW MUCH on every tick -- not just the
        # resulting error. These are pure observations of values already
        # computed -- adding or removing a trace entry must never change a
        # control output.
        adapt: dict[str, float] = {}

        # ── CURRENT-STATE corner factor ─────────────────────────────────────
        # Replaces the deleted lookahead gain-scheduling family (see the
        # module comment near the top of this file): 0 (straight) -> 1 (full
        # corner), a single continuous saturating curve of the CURRENT
        # ~1m-preview curvature `kappa` -- the same signal _adaptive_R_rate/
        # _steer_rate_anti_hunt already use. No forward scan, no separate
        # decay-distance timer/hysteresis state: entry and exit are
        # symmetric, driven purely by how `kappa` itself rises and falls.
        corner_factor = _corner_factor(kappa, self.params.corner_factor_k)
        adapt["corner_factor"] = corner_factor

        # Extra push in the SAME direction as corner_factor's "full corner"
        # endpoint, active only when BOTH corner_factor > 0 AND speed is
        # low -- gated on corner_factor (multiplicatively) so this cannot
        # fire on low speed alone with no corner, unlike the deleted
        # _low_speed_steer_rate_boost (which fired on speed alone and ended
        # up taxing wanted low-speed turn-in indistinguishably from
        # unwanted post-exit wobble).
        low_speed_boost = _low_speed_corner_boost(
            car_speed, corner_factor,
            v_half=self.params.low_speed_corner_boost_v_half,
            max_extra=self.params.low_speed_corner_boost_max_extra,
        )
        adapt["low_speed_corner_boost"] = low_speed_boost
        # Combined corner fraction driving every blend below: corner_factor
        # itself, boosted further (never past 1.0) at low speed in-corner.
        corner_frac = float(np.clip(corner_factor + low_speed_boost, 0.0, 1.0))
        adapt["corner_frac"] = corner_frac

        R_scaled = _adaptive_R_scaling(car_speed, self.R)
        adapt["m_R_speed"] = float(R_scaled[0, 0] / self.R[0, 0]) if self.R[0, 0] else 1.0

        R_rate_scaled = _adaptive_R_rate(
            kappa, self.R_rate,
            enable_in_corners=self.adaptive_r_rate_enable_in_corners,
            during_floor=self.params.adaptive_r_rate_during_floor,
        )
        adapt["m_Rrate_corner"] = (
            float(R_rate_scaled[0, 0] / self.R_rate[0, 0]) if self.R_rate[0, 0] else 1.0
        )
        _rr_before_hunt = float(R_rate_scaled[0, 0])
        R_rate_scaled = _steer_rate_anti_hunt(
            kappa, x0[0], R_rate_scaled, self.steer_rate_anti_hunt_enabled,
            e_psi=x0[2],
            boost_max=self.params.anti_hunt_boost_max,
        )
        adapt["m_Rrate_antihunt"] = (
            float(R_rate_scaled[0, 0] / _rr_before_hunt) if _rr_before_hunt else 1.0
        )
        _rr_before_reversal = float(R_rate_scaled[0, 0])
        R_rate_scaled = _reversal_penalty_boost(
            float(self._u_prev[0]), R_rate_scaled, self.reversal_penalty_enabled,
            boost_max=self.params.reversal_penalty_boost_max,
            k=self.params.reversal_penalty_k,
        )
        adapt["m_Rrate_reversal"] = (
            float(R_rate_scaled[0, 0] / _rr_before_reversal) if _rr_before_reversal else 1.0
        )

        # ── Current-state Q[0,0]/Q[2,2]/Q[3,3] and R_rate[0,0] blend ───────
        # Straight-line-blend, PER WEIGHT, between a "straight" endpoint and
        # a "full corner" endpoint, driven by corner_frac above. Replaces
        # the deleted per-mechanism multiplicative lookahead gates with one
        # shared current-state schedule. R[0,0] (steering effort) is a
        # special case: blended toward a MIDDLE value, not the same
        # corner-floor extreme as R_rate/Q[3,3], per the user's own framing
        # ("should be somewhere in between the two extremes to discourage
        # saturation") -- see _blend/the R_steer_corner_mid field comment.
        Q_base = self.Q.copy()
        Q_base[0, 0] = _blend(
            self.params.q_ey_straight, self.params.q_ey_corner, corner_frac)
        Q_base[2, 2] = _blend(
            self.params.q_epsi_straight, self.params.q_epsi_corner, corner_frac)
        Q_base[3, 3] = _blend(
            self.params.q_r_straight, self.params.q_r_corner, corner_frac)
        adapt["Q_ey_base"]   = float(Q_base[0, 0])
        adapt["Q_epsi_base"] = float(Q_base[2, 2])
        adapt["Q_r_base"]    = float(Q_base[3, 3])

        # CAUTION: this line sets R_rate_scaled[0,0]'s BASE value (the
        # current-curvature straight/corner schedule), so every multiplier
        # computed above (m_Rrate_antihunt, m_Rrate_reversal, and any future
        # one added before this point) must be explicitly reapplied here too
        # -- an assignment that doesn't multiply in all of them silently
        # discards whichever one it omits, even though that multiplier's own
        # value is still correctly logged to adapt[...]. This exact class of
        # bug has recurred more than once; do not add a new R_rate[0,0]
        # multiplier without threading it through this line.
        R_rate_scaled = R_rate_scaled.copy()
        R_rate_scaled[0, 0] = _blend(
            self.params.rrate_steer_straight, self.params.rrate_steer_corner,
            corner_frac,
        ) * adapt["m_Rrate_antihunt"] * adapt["m_Rrate_reversal"]
        adapt["Rrate_steer_corner_blend"] = float(R_rate_scaled[0, 0])

        R_scaled = R_scaled.copy()
        R_scaled[0, 0] = _blend(
            R_scaled[0, 0], self.params.r_steer_corner_mid, corner_frac)
        adapt["R_steer_corner_blend"] = float(R_scaled[0, 0])

        # x0[0] is e_y (delay-compensated, if n_delay>0 above rolled it
        # forward) -- compute() has no bare e_y in scope, only x0.
        Q_scaled = _adaptive_Q_scaling(x0[0], Q_base, self.adaptive_q_scaling_enabled)
        adapt["m_Q_ey_soften"] = (
            float(Q_scaled[0, 0] / Q_base[0, 0]) if Q_base[0, 0] else 1.0
        )

        # Absolute post-scaling weights actually handed to the QP. The
        # multipliers above say how hard each feature pushed; these say where
        # the weights ended up, which is what makes two runs comparable even
        # if a base weight in self.Q changed between them.
        adapt["Q_ey_eff"]   = float(Q_scaled[0, 0])
        adapt["Q_epsi_eff"] = float(Q_scaled[2, 2])
        adapt["Q_r_eff"]    = float(Q_scaled[3, 3])
        adapt["R_steer_eff"]      = float(R_scaled[0, 0])
        adapt["Rrate_steer_eff"]  = float(R_rate_scaled[0, 0])

        # ── Heading-error-driven accel/brake asymmetry ──────────────────────
        # Always-on, independent of the corner_frac scheduler above: a
        # continuous 0->1 fraction of CURRENT |e_psi| (x0[2]) scales
        # r_a_accel toward accel_boost_max (more expensive, so the MPC
        # doesn't keep accelerating through a heading error it should be
        # correcting) and r_a_brake toward brake_floor (cheaper, so braking
        # authority is freed up specifically when heading error is large).
        # Not a replacement for _adaptive_R_scaling (current-speed-driven
        # R[0,0] scaling), which is left untouched.
        epsi_abs = abs(x0[2])
        epsi_half = max(self.params.epsi_ra_half_rad, 1e-6)
        frac_epsi = epsi_abs / (epsi_abs + epsi_half)
        r_a_accel_eff = self.params.r_a_accel * (
            1.0 + (self.params.epsi_ra_accel_boost_max - 1.0) * frac_epsi)
        r_a_brake_eff = self.params.r_a_brake * (
            1.0 - (1.0 - self.params.epsi_ra_brake_floor) * frac_epsi)
        adapt["R_a_accel_eff"] = float(r_a_accel_eff)
        adapt["R_a_brake_eff"] = float(r_a_brake_eff)

        # Wall-clock the QP so the log can distinguish "the solver is slow"
        # from "the pipeline upstream of us is slow" — see solve_ms in
        # telemetry_logger's column reference.
        _t_solve0 = time.perf_counter()
        u_opt = self._solve_qp(
            x0, Ad, Bd, R_scaled, R_rate_scaled, Q_scaled,
            r_a_accel=r_a_accel_eff, r_a_brake=r_a_brake_eff,
        )
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
            # Per-feature adaptive multipliers + demand diagnostics; see the
            # "Adaptive-feature trace" comment above compute()'s R_scaled.
            **adapt,
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