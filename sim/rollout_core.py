"""
sim/rollout_core.py — Single Source of Truth for the MPC Closed-Loop Rollout

PURPOSE
-------
tuner/offline_tuner.run_headless_rollout() and gui/simulation.simulate_closed_loop() used
to independently reimplement the exact same per-step logic: tracking-error
computation, progress tracking, MPC solve with adaptive gains, delay queue,
termination checks, and metric accumulation. Any tweak to one silently drifted
from the other — which is exactly why offline-tuner scores and the live
simulator's "Show Metrics" scores stopped matching (e.g. the planner-fallback
branch existed in tuner/offline_tuner but was missing in gui/simulation.py).

This module is now the ONLY place that runs the actual rollout loop. Both
tuner/offline_tuner.py and gui/simulation.py call run_core_rollout() and only differ in
what they do with the result:
  - tuner/offline_tuner.py:  want_history=False → just the composite score
  - gui/simulation.py:      want_history=True  → full step history for the GUI

WHY NOT IN gui/simulation.py
-------------------------
gui/simulation.py builds a matplotlib GUI at import time. tuner/offline_tuner.py runs
rollouts inside multiprocessing worker processes — importing gui/simulation.py
there would try to open a GUI window in every worker. This module imports
nothing GUI-related, so it's safe to import from anywhere.
"""

import math
import numpy as np
from collections import deque

from model.vehicle_physics import (
    step_nonlinear_plant, init_plant_state, plant_to_tracking_error,
    find_closest_reference_bounded,
)
from controller.optimiser import solve_mpc
from sim.sim_track import SimPerception, SimPlanner, calculate_dynamic_max_steps
from controller.model_utils import (
    curvature_estimate, adaptive_R_rate, adaptive_R_scaling, adaptive_Q_scaling,
    steer_rate_anti_hunt, lookahead_curvature_profile, update_lookahead_peak,
    adaptive_Q_lookahead, steer_effort_straight_boost,
)
from sim.scoring import RolloutMetrics
import sim.speed_profile as sp
import cvxpy as cp

from settings import (
    USE_PLANNER, DELAY_STEPS, OFFTRACK_LIMIT, MAX_FAILS, DT,
    ROLLOUT_EPS, ROLLOUT_MAX_ITER, N_HORIZON,
    DELAY_JITTER_STEPS, DELAY_JITTER_SEED,
    SLAM_NOISE_ENABLED, SLAM_POS_JITTER_STD, SLAM_YAW_JITTER_STD,
    SLAM_POS_DRIFT_STD, SLAM_YAW_DRIFT_STD, SLAM_DRIFT_TAU, SLAM_NOISE_SEED,
    POSE_HOLD_ENABLED, POSE_HOLD_PROB, POSE_HOLD_MEAN_TICKS,
    POSE_HOLD_MAX_TICKS, POSE_HOLD_SEED,
    CONE_NOISE_ENABLED, CONE_POS_JITTER_STD, CONE_NOISE_SEED,
    REF_HEADING_RATE_LIMIT_ENABLED, REF_HEADING_RISE_RATE,
    TERMINAL_Q_SCALE, ADAPTIVE_Q_SCALING_ENABLED,
    USE_PRECOMPUTED_SPEED_PROFILE, STEER_RATE_ANTI_HUNT_ENABLED,
    ENABLE_DYNAMIC_SPEED_CAP, DYNAMIC_CAP_A_LAT_MAX, DYNAMIC_CAP_SAFETY,
    ADAPTIVE_R_RATE_ENABLE_IN_CORNERS,
    ADAPTIVE_Q_LOOKAHEAD_ENABLED, ADAPTIVE_Q_DEMAND_NORMALISED,
    STEER_EFFORT_STRAIGHT_BOOST_ENABLED,
    ADAPTIVE_R_RATE_DURING_FLOOR, ADAPTIVE_R_RATE_ENTERING_FLOOR,
    ADAPTIVE_R_RATE_K_ENTERING,
    ALAT_CEILING_FLAT, ALAT_CEILING_SLOPE, ALAT_CEILING_INTERCEPT,
    ADAPTIVE_Q_DEMAND_HALF,
    ADAPTIVE_Q_STRAIGHT_EY_FLOOR, ADAPTIVE_Q_STRAIGHT_EY_K,
    ADAPTIVE_Q_STRAIGHT_EPSI_BOOST_MAX, ADAPTIVE_Q_STRAIGHT_R_BOOST_MAX,
    ADAPTIVE_Q_STRAIGHT_K,
    ADAPTIVE_Q_UTURN_HEADING_THRESH_RAD, ADAPTIVE_Q_UTURN_HEADING_SAT_RAD,
    ADAPTIVE_Q_UTURN_EY_BOOST_MAX, ADAPTIVE_Q_UTURN_EPSI_BOOST_MAX,
    ADAPTIVE_Q_UTURN_R_RELAX_FLOOR,
)

STALL_CHECK_INTERVAL = 60   # Steps between rolling stall checks (3 s at 20 Hz)
STALL_MIN_DISTANCE = 3.0    # Minimum distance (m) expected per interval

# v_max/v_min for the live-planner branch's speed_profile.curvature_speed() call.
# Mirror fsds_simulator/control/fsae_control/fsae_control/mpc_controller_standalone.py's
# v_max/v_min ROS parameters (default V_MAX/V_MIN) — and the old SimPlanner
# defaults these replace — so offline-tuned weights see the same speed targets
# the live node will command.
PLANNER_V_MAX = 20.0
PLANNER_V_MIN = 1.5

# Max rate (m/s^2) at which the speed TARGET may rise. Mirrors
# mpc_controller_standalone.SPEED_TARGET_RISE_RATE — keep the two in sync.
# Decreases are never rate-limited; only the rise is damped, to suppress the
# planner's frame-to-frame curvature jitter without capping real acceleration.
SPEED_TARGET_RISE_RATE = 7.0

# Max rate (gate-units/s) at which tracking_error_speed_gate()'s output may
# change per tick, in either direction. Mirrors
# mpc_controller_standalone.GATE_RATE_LIMIT — keep the two in sync. See that
# constant's own comment for the full rationale.
GATE_RATE_LIMIT = 2.0


def _normalize_angle(angle):
    """Wrap an angle to (−π, π] using atan2."""
    return np.arctan2(np.sin(angle), np.cos(angle))


def _rate_limit_ref_psi(ref_psi_raw, ref_psi_prev, max_rate_rad_per_s, dt):
    """
    Cap how fast the reference heading (ref_psi) is allowed to change per
    tick, same shape as SPEED_TARGET_RISE_RATE's cap on v_target.

    Why this exists: most of the planner's reference-heading swing is real
    track geometry, but a tail-concentrated excess — the reference correctly
    anticipating a sharp corner earlier than the car has actually yawed — is
    strongly linked to steering saturation. Limiting only the RATE (never
    the final direction — once the car catches up, the raw reference is
    reached again) trades slightly later turn-in commitment for not asking
    the controller to snap onto a heading the car has no chance of reaching
    yet.

    Symmetric (limits swings in either direction) — unlike
    SPEED_TARGET_RISE_RATE, which only limits increases because slowing down
    is always safe. There is no equivalent "always safe" direction for a
    heading reference: swinging the target toward straight ahead just as
    hard as toward the apex can be equally premature relative to where the
    car has actually turned.

    Parameters
    ----------
    ref_psi_raw : float
        This tick's actual reference heading (rad), unwrapped-compatible with
        ref_psi_prev (i.e. already continuous, not wrapped to [-pi, pi]).
    ref_psi_prev : float or None
        Previous tick's LIMITED reference heading. None on the first tick
        after start/reset, in which case the raw value passes through
        unlimited (mirrors v_des_prev's None handling).
    max_rate_rad_per_s : float
        Maximum |d(ref_psi)/dt|, rad/s.
    dt : float
        Tick period, s.

    Returns
    -------
    float — the limited reference heading (rad, unwrapped-compatible).
    """
    if ref_psi_prev is None:
        return ref_psi_raw
    max_step = max_rate_rad_per_s * dt
    delta = _normalize_angle(ref_psi_raw - ref_psi_prev)
    delta = np.clip(delta, -max_step, max_step)
    return ref_psi_prev + delta


class SlamNoise:
    """
    Corrupts the pose the controller/planner SEE, leaving the true plant
    state untouched.

    Why this exists
    ---------------
    FSDS has no real SLAM. Its `sim_perception` node republishes ground-truth
    `/fsds/testing_only/odom` straight onto `/fsae/slam/car_position`, and the
    cone map is a latched oracle map cropped to a forward window — the only
    modelled limitation is sensor RANGE, not accuracy. This rollout mirrored
    that by feeding exact plant state back into the planner and the
    tracking-error maths, which makes the simulator systematically easier than
    the real car, whose pose comes from ZED visual odometry + cone_mapper.

    Localisation error matters here specifically because it lands directly in
    e_y/e_psi — the signals the MPC steers on. A pose that jitters makes the
    measured error jitter, and an under-damped controller chases it.

    NOT the cause of the observed FSDS chatter: FSDS's pose is already exact,
    so pose noise cannot explain steering reversal chatter seen there. That
    was instead caused by (a) the steering slew limit binding on a large
    fraction of steps and (b) sim_perception publishing the pose at a lower
    rate than the control loop, so many MPC solves re-used an unchanged
    pose. Both are fixed elsewhere; this class is for the REAL car's
    localisation error, and defaults to off.

    Model
    -----
    Two additive components, matching how real SLAM misbehaves:

      jitter — zero-mean white noise, redrawn every step. Causes chatter.
      drift  — a first-order (Ornstein-Uhlenbeck) process pulled back toward
               zero with time constant SLAM_DRIFT_TAU. Wanders over seconds
               and self-corrects, like a SLAM estimate between loop closures,
               instead of random-walking away over a long rollout.

    The OU update uses the exact discrete-time form
    ``d <- a*d + sqrt(1-a^2)*sigma*w`` with ``a = exp(-dt/tau)``, whose
    stationary standard deviation is exactly ``sigma`` regardless of dt — so
    SLAM_POS_DRIFT_STD means what it says and does not change meaning if DT
    changes.

    IMPORTANT: this is deliberately NOT applied to the state handed to the
    plant or to scoring. The car is judged on where it actually ended up, not
    where it believed it was — the same asymmetry real localisation error has.
    """

    def __init__(self, dt, seed, pos_jitter_std, yaw_jitter_std,
                 pos_drift_std, yaw_drift_std, drift_tau):
        self._rng = np.random.default_rng(seed)
        self._pos_jitter_std = float(pos_jitter_std)
        self._yaw_jitter_std = float(yaw_jitter_std)
        self._pos_drift_std = float(pos_drift_std)
        self._yaw_drift_std = float(yaw_drift_std)

        tau = max(float(drift_tau), 1e-6)
        self._a = float(np.exp(-float(dt) / tau))
        # Scale that makes the OU process's stationary std equal *_drift_std.
        self._q = float(np.sqrt(max(0.0, 1.0 - self._a * self._a)))

        # Start the drift at a stationary sample rather than 0, so step 0 isn't
        # artificially better-localised than the rest of the run.
        self._drift = self._rng.normal(0.0, 1.0, size=3)

    def corrupt(self, x, y, yaw):
        """Return (x_est, y_est, yaw_est) as the controller would measure them."""
        self._drift = self._a * self._drift + self._q * self._rng.normal(0.0, 1.0, size=3)
        jitter = self._rng.normal(0.0, 1.0, size=3)

        x_est = x + self._drift[0] * self._pos_drift_std + jitter[0] * self._pos_jitter_std
        y_est = y + self._drift[1] * self._pos_drift_std + jitter[1] * self._pos_jitter_std
        yaw_est = yaw + self._drift[2] * self._yaw_drift_std + jitter[2] * self._yaw_jitter_std
        return float(x_est), float(y_est), float(_normalize_angle(yaw_est))


class ConeNoise:
    """
    Corrupts cone positions AFTER SimPerception's FOV filter, modelling real
    per-detection vision noise on top of FSDS's noise-free oracle cone map.

    Why this exists
    ---------------
    sim_track.SimPerception.visible_cones() returns exact ground-truth cone
    coordinates, only cropped by range/FOV — see docs/planning_control_sync.md,
    "Simulator fidelity limits": the cone map was, until this class existed,
    the one aspect of the sim/real gap with literally no model at all. This
    adds the minimum needed to make perception-side hypotheses testable
    offline: independent per-cone, per-frame position jitter. It does NOT
    model false positives/negatives or range-dependent noise growth — both
    are real, both are still unmodelled, this only closes the position-jitter
    gap. See settings.CONE_NOISE_ENABLED for the full rationale.

    Unlike SlamNoise, there is no drift/bias component: a vision cone detector
    re-estimates each cone's position independently every frame rather than
    tracking one belief over time, so there is nothing that should carry over
    between frames the way SLAM's OU drift does. If a future measurement shows
    real cone detections DO carry frame-to-frame correlated error (e.g. from a
    slowly-drifting camera calibration), add a drift term the same way
    SlamNoise does rather than repurposing jitter for it.

    Deliberately applied to EACH CONE INDEPENDENTLY, not once per frame as a
    shared offset — SlamNoise corrupts a single pose shared by the whole cone
    set, which is correct for localisation error, but detection error is
    per-object (each cone has its own range/angle/occlusion to the sensor).
    """

    def __init__(self, seed, pos_jitter_std):
        self._rng = np.random.default_rng(seed)
        self._pos_jitter_std = float(pos_jitter_std)

    def corrupt(self, cones):
        """Return a copy of `cones` (n, 2) with independent per-point jitter."""
        if len(cones) == 0 or self._pos_jitter_std <= 0.0:
            return cones
        return cones + self._rng.normal(0.0, self._pos_jitter_std, size=cones.shape)


class PoseFeedHold:
    """
    Models the live pose feed REPEATING the previous measurement instead of
    delivering a fresh one — the dominant sim-to-real gap.

    WHY THIS EXISTS
    ---------------
    The offline rollout gave the controller a brand-new, exact pose every
    single tick. DELAY_STEPS models a fixed 50 ms lag, but the controller
    still learns something new every step, so its heading error can never
    accumulate. The real car does not work that way: `/fsae/slam/car_position`
    intermittently stops publishing, and the controller re-uses the last pose
    it received while the car keeps moving.

    Measured from live telemetry across runs on the same track and same
    tuned gains, differing only in how badly the feed stalled: a healthy run
    has a low repeated-tick rate and short holds, while a degraded run can
    see the majority of ticks repeating with holds approaching a second. In
    one such degraded run the pose froze for roughly a second at speed —
    enough distance travelled with no positional update that, when the feed
    resumed, the heading error was unrecoverable and the car spun. Both runs
    used identical weights on identical track; the only difference was the
    feed.

    This is NOT the same thing as DELAY_STEPS or DELAY_JITTER_STEPS:
      - DELAY_STEPS delays a pose that is still FRESH each tick.
      - DELAY_JITTER_STEPS perturbs only the controller's BELIEF about the lag.
      - This repeats the DATA, so pose_age genuinely ramps and the controller
        is flying blind. Nothing in the previous model produced that.

    MODEL
    -----
    Two-state Markov chain over "fresh" and "held":
      - each tick, with probability `p_hold`, a hold begins
      - hold length is drawn geometrically, mean `mean_hold_ticks`, capped at
        `max_hold_ticks`
    A geometric hold length reproduces the measured histogram shape well: many
    short 2-tick holds, a thin tail of long ones. Fitting anything more
    elaborate to two runs would be overfitting.

    The whole ESTIMATED state is frozen, not just x/y — the live log shows
    v_actual and yaw repeating alongside position, because they come from the
    same odometry message. Freezing position while letting speed update would
    model a failure mode that does not exist.

    Seeded, so rollouts stay reproducible and CMA-ES still gets a stable score
    per candidate (see settings.POSE_HOLD_SEED).
    """

    def __init__(self, p_hold, mean_hold_ticks, max_hold_ticks, seed):
        self._rng = np.random.default_rng(seed)
        self._p_hold = float(np.clip(p_hold, 0.0, 1.0))
        # Geometric success prob giving the requested mean hold length.
        # Repeats per hold average mean_hold - 1 (see apply()); guard the
        # degenerate mean_hold <= 1 case, which means "never actually hold".
        self._mean_hold = max(float(mean_hold_ticks), 1.0)
        self._q_repeat = 1.0 / max(self._mean_hold - 1.0, 1e-6)
        self._q_repeat = float(np.clip(self._q_repeat, 1e-6, 1.0))
        self._max_hold = int(max(1, max_hold_ticks))
        self._remaining = 0          # ticks still to hold
        self._held = None            # the frozen (state, X, Y, psi) tuple

    def apply(self, state_est, X_est, Y_est, psi_est):
        """
        Return the pose the controller actually sees this tick, plus how many
        ticks old it is.

        Returns (state_est, X, Y, psi, age_ticks). age_ticks is 0 on a fresh
        sample and increments through a hold, mirroring the live pose_age_s.
        """
        if self._remaining > 0:
            self._remaining -= 1
            s, x, y, psi, age = self._held
            self._held = (s, x, y, psi, age + 1)
            return s, x, y, psi, age + 1

        # Fresh sample this tick; decide whether the NEXT ticks are held.
        # A "hold of length L" spans L ticks TOTAL (this fresh one plus L-1
        # repeats), matching how the live histogram was counted, so the number
        # of repeat ticks to schedule is L-1. np.random.geometric returns >= 1,
        # hence the -1: without it every hold ran one tick long and the mean
        # came out at 2.94 against a measured 2.08.
        if self._rng.random() < self._p_hold:
            # A hold spans `mean_hold_ticks` ticks TOTAL (this fresh one plus
            # the repeats), matching how the live histogram was counted, so the
            # number of REPEATS to schedule averages mean_hold - 1. A geometric
            # draw has mean 1/q, hence q = 1/(mean_hold - 1).
            self._remaining = int(np.clip(
                self._rng.geometric(self._q_repeat), 1, self._max_hold - 1))

        self._held = (state_est.copy(), float(X_est), float(Y_est), float(psi_est), 0)
        return state_est, X_est, Y_est, psi_est, 0


_PREDICT_EPSI_CLIP = 0.5   # rad (~28.6°) — small-angle bound, see predict_ahead below


def predict_ahead(x0, Ad, Bd, pending_cmds):
    """
    Roll the linear error-state model forward through commands already
    committed but not yet applied to the plant, so the MPC solves against
    the state it will actually face when its new output takes effect
    instead of the stale current state (delay compensation).

    pending_cmds must be ordered oldest-first (the order they will be
    applied to the plant). Same x_p = Ad @ x_p + Bd @ u mechanics as the
    horizon-prediction preview below.

    Ad's e_psi -> e_y_dot coupling is the kinematic relation e_y_dot ~= vx *
    sin(e_psi), linearised to vx * e_psi (bicycle_model.py). That's only
    valid for small e_psi (sin(x) ~= x). Unlike the closed-loop MPC horizon
    (which re-measures every real step), this rollforward is open-loop over
    several steps with no ground-truth correction in between, so a large
    e_psi here (sharp corner + a perturbed initial heading) compounds every
    step instead of getting corrected — observed to blow up e_y_dot and
    saturate steering on PATH_SUDDEN_TURN. Clip e_psi to a small-angle range
    before each step's matrix multiply so the rollforward can't leave the
    regime the linear model is actually valid in; the real (unclipped) e_psi
    is still what the QP solves against afterwards via x0_mpc.
    """
    x_p = x0.copy()
    for u in pending_cmds:
        x_p[2] = np.clip(x_p[2], -_PREDICT_EPSI_CLIP, _PREDICT_EPSI_CLIP)
        x_p = Ad @ x_p + Bd @ u
    return x_p


def compute_step_budget(path_X, path_Y, path_v_profile):
    """
    Single source of truth for the dynamic step budget. Both callers used to
    duplicate this formula exactly — arc-length/fallback-speed estimate vs.
    a speed-profile-aware estimate, taking the larger of the two.

    Returns
    -------
    (dynamic_max_steps, max_steps) : tuple of int
        dynamic_max_steps : from calculate_dynamic_max_steps() alone — used
                             for the time-bonus "expected time" baseline.
        max_steps         : max(dynamic_max_steps, profile_max_steps) — the
                             actual step budget for the rollout loop.
    """
    path_length = float(np.sum(np.hypot(np.diff(path_X), np.diff(path_Y))))
    dynamic_max_steps = calculate_dynamic_max_steps(path_X, path_Y, dt=DT)
    mean_v_profile = float(np.mean(path_v_profile)) if len(path_v_profile) > 0 else 1.5
    profile_max_steps = int(
        math.ceil((path_length / max(mean_v_profile * 0.6, 1.5)) * 1.5 / DT)
    )
    max_steps = max(dynamic_max_steps, profile_max_steps)
    return dynamic_max_steps, max_steps


def run_core_rollout(
    path_X, path_Y, path_Psi, path_v_profile, blue_cones, yellow_cones,
    Q, R, R_rate, u_min, u_max, vehicle_params,
    ey0=0.0, epsi0=0.0, max_steps=400, dynamic_max_steps=None,
    use_planner=USE_PLANNER, model_lookup=None,
    n_horizon=N_HORIZON, eps=ROLLOUT_EPS, max_iter=ROLLOUT_MAX_ITER,
    want_history=False, want_horizon_pred=False,
    optimal_time=None, continue_after_dnf=False,
):
    """
    Run one closed-loop MPC rollout: nonlinear plant + MPC controller.

    THE single implementation of the rollout loop, shared by
    offline_tuner.run_headless_rollout() and simulation.simulate_closed_loop().

    Parameters
    ----------
    path_X, path_Y, path_Psi, path_v_profile : arrays
        Reference path geometry and speed profile.
    optimal_time : float or None
        Quasi-steady-state minimum traversal time for this path (s), from
        speed_profile.optimal_lap_time(). Anchors `time_bonus` to the physical
        optimum so it means "how close to fastest-possible" and is comparable
        across paths. None falls back to the old arc_length/2.5 m/s heuristic
        (kept only so external callers that don't supply it still run).
    blue_cones, yellow_cones : arrays
        Static cone map for SimPerception (used only if use_planner=True).
    Q, R, R_rate : np.ndarray
        MPC cost matrices at their template/tuned values (this function
        applies adaptive_R_scaling / adaptive_R_rate internally each step).
    u_min, u_max : array-like, shape (2,)
        Actuator bounds.
    vehicle_params : VehicleParams
    ey0 : float
        Initial lateral offset (m), Frenet frame.
    epsi0 : float
        Initial heading offset in **radians**. (gui/simulation.py's slider is in
        degrees — convert with np.radians() before calling this function.)
    max_steps : int
        Step budget for the loop (use compute_step_budget()'s second value).
    dynamic_max_steps : int
        From compute_step_budget()'s first value — used only for the time
        bonus's "expected time" baseline. Required if you want a nonzero
        time bonus on a clean finish.
    use_planner : bool
        Planner-in-the-loop vs. oracle tracking against the global path.
    model_lookup : callable(vx, dt) -> (Ad, Bd)
        Bicycle-model lookup. Pass offline_tuner.get_cached_model — both
        callers already share this cache.
    want_history : bool
        If True, populate and return a full step-by-step history dict for
        the GUI. If False (CMA-ES scoring path), skip all the list-append
        overhead and just accumulate RolloutMetrics.
    want_horizon_pred : bool
        If True (and want_history=True), also compute the cosmetic N-step
        horizon prediction used by the GUI's cyan prediction line.
    continue_after_dnf : bool
        If True, a DNF trigger (solver-fail streak, stall, or off-track) sets
        the dnf/offtrack flags and history["fail_reason"] as usual but does
        NOT stop the loop -- the plant keeps stepping to max_steps. For
        inspecting what the car does AFTER the moment that would normally end
        the rollout (e.g. does it recover, does it stay off-track, how does
        the rest of a recorded map compare to a live log that also doesn't
        stop at first excursion). Only the FIRST trigger's reason is recorded
        in fail_reason; dnf/offtrack stay True from that point on even if the
        car re-enters OFFTRACK_LIMIT afterward. Default False preserves the
        existing stop-on-DNF behaviour used by scoring and tuning.

    Returns
    -------
    dict with keys:
        "composite_score" : float — final score (see sim/scoring.py)
        "metrics_result"  : dict  — full RolloutMetrics.finalize() output
        "progress"        : float — continuous completion fraction [0,1]
        "reached_end"     : bool
        "dnf"             : bool
        "offtrack"        : bool
        "time_bonus"      : float
        "history"         : dict or None — populated iff want_history=True
    """
    if model_lookup is None:
        raise ValueError("model_lookup must be provided (e.g. offline_tuner.get_cached_model)")
    if dynamic_max_steps is None:
        dynamic_max_steps = max_steps

    # ── Initial condition (Frenet frame → global pose) ────────────────────────
    base_heading = path_Psi[0]
    X0 = path_X[0] - ey0 * np.sin(base_heading)
    Y0 = path_Y[0] + ey0 * np.cos(base_heading)
    psi0 = _normalize_angle(base_heading + epsi0)

    state = init_plant_state(X0, Y0, psi0, vx0=0.0)

    # ── Cone-detection noise ───────────────────────────────────────────────
    # Corrupts only what SimPerception reports as visible; the plant, the
    # oracle centreline (use_planner=False) and the score are unaffected. See
    # ConeNoise / settings.CONE_NOISE_ENABLED.
    cone_noise = None
    if CONE_NOISE_ENABLED:
        cone_noise = ConeNoise(seed=CONE_NOISE_SEED, pos_jitter_std=CONE_POS_JITTER_STD)

    if use_planner:
        perception = SimPerception(blue_cones, yellow_cones)
        planner = SimPlanner()
        _b0, _y0 = perception.visible_cones(float(X0), float(Y0), float(psi0))
        if cone_noise is not None:
            _b0, _y0 = cone_noise.corrupt(_b0), cone_noise.corrupt(_y0)
        planner.update(_b0, _y0, np.array([X0, Y0]), float(psi0))

    command_queue = deque([np.zeros(2) for _ in range(DELAY_STEPS + 1)], maxlen=DELAY_STEPS + 1)
    u_prev = np.zeros(2)

    # Hard per-step slew-rate limit handed to the MPC, mirroring the live
    # mpc_core.py's du_max so offline-tuned weights transfer. Derived from the
    # vehicle's physical steering rate rather than hardcoded, and scaled by DT
    # so it stays a rate. The acceleration entry (0.6 per step at DT=0.05 =
    # 12 m/s^3) matches the live controller's second du_max element.
    du_max = np.array([
        vehicle_params.max_steer_rate * DT,
        0.6,
    ])

    # ── Delay-estimation error ────────────────────────────────────────────
    # The plant always applies the true DELAY_STEPS lag. What varies is how
    # many pending commands the CONTROLLER believes it must roll forward
    # through. Live, that count comes from a noisy pose timestamp divided by
    # a jittering loop period, so it is regularly wrong by a step; offline it
    # used to be exactly right on every tick, which made predict_ahead()
    # strictly more effective in the tuner than it can ever be on the car.
    # Seeded so each rollout is reproducible and CMA-ES still gets a stable
    # score per candidate (see settings.DELAY_JITTER_SEED).
    delay_rng = np.random.default_rng(DELAY_JITTER_SEED)

    # Previous step's speed target, for the rise-rate limiter above.
    v_des_prev = None

    # Stacked (N,2) path array for lookahead_curvature_profile's forward scan
    # (ADAPTIVE_Q_LOOKAHEAD) -- built once, not per-step.
    path_xy = np.column_stack([path_X, path_Y])

    # ADAPTIVE_Q_LOOKAHEAD peak-tracker state -- see model_utils.py's
    # update_lookahead_peak. dist_since_peak starts at +inf so
    # lookahead_exit_boost is a no-op until the first corner peak is seen.
    lookahead_peak_state = {
        "last_peak_kappa_abs": 0.0,
        "dist_since_peak": float("inf"),
        "armed_for_next_peak": True,
    }

    # Previous step's tracking-error speed gate, for GATE_RATE_LIMIT above.
    gate_prev = None

    # Previous step's LIMITED reference heading, for REF_HEADING_RATE_LIMIT.
    # Unwrapped/continuous (not [-pi, pi]) so consecutive limiting steps
    # compose correctly across the wrap boundary.
    ref_psi_prev = None

    # ── SLAM / localisation noise ─────────────────────────────────────────
    # Corrupts only the pose fed to perception/planner/tracking-error; the
    # plant and the score always see the true state. See SlamNoise.
    slam_noise = None
    if SLAM_NOISE_ENABLED:
        slam_noise = SlamNoise(
            dt=DT, seed=SLAM_NOISE_SEED,
            pos_jitter_std=SLAM_POS_JITTER_STD,
            yaw_jitter_std=SLAM_YAW_JITTER_STD,
            pos_drift_std=SLAM_POS_DRIFT_STD,
            yaw_drift_std=SLAM_YAW_DRIFT_STD,
            drift_tau=SLAM_DRIFT_TAU,
        )

    # ── Pose-feed hold ────────────────────────────────────────────────────
    # Models the live pose feed repeating its last measurement instead of
    # delivering a fresh one. See PoseFeedHold — this is the measured dominant
    # sim-to-real gap, and it is deliberately applied AFTER SLAM noise so a
    # held tick repeats the corrupted pose the controller actually saw, not a
    # freshly-corrupted one (re-drawing noise during a freeze would leak new
    # information into a period when the controller should be blind).
    pose_hold = None
    if POSE_HOLD_ENABLED:
        pose_hold = PoseFeedHold(
            p_hold=POSE_HOLD_PROB,
            mean_hold_ticks=POSE_HOLD_MEAN_TICKS,
            max_hold_ticks=POSE_HOLD_MAX_TICKS,
            seed=POSE_HOLD_SEED,
        )

    metrics = RolloutMetrics()
    idx = 0
    last_idx = 0
    cumulative_distance = 0.0
    consecutive_fails = 0
    dnf = False
    offtrack = False
    reached_end = False
    inaccurate_count_total = 0
    dist_at_last_stall_check = 0.0

    path_seg_dist = np.hypot(np.diff(path_X), np.diff(path_Y))
    path_length = float(np.sum(path_seg_dist))

    history = None
    if want_history:
        history = {
            "X": [], "Y": [], "psi": [], "v": [], "r": [], "v_target": [],
            "u_steer": [], "u_accel": [], "e_y": [], "e_psi": [],
            # Ground-truth tracking error (what the score uses). Identical to
            # e_y/e_psi unless SLAM noise is enabled — see SlamNoise.
            "e_y_true": [], "e_psi_true": [],
            "pred_X": [], "pred_Y": [], "solver_failed": [],
            "failed": False, "offtrack": False, "fail_reason": None,
        }
        if use_planner:
            # Per-step snapshot of SimPlanner's live centreline (distinct from
            # the true reference path_X/path_Y) — GUI-only, cosmetic, for
            # visualising what the planner actually built vs. the ground truth.
            history["planner_X"] = []
            history["planner_Y"] = []

    n_ran = max_steps

    for step in range(max_steps):
        X_g, Y_g, psi_g = state[0], state[1], state[2]

        # ── Estimated (SLAM) pose vs true pose ────────────────────────────
        # Everything the controller and planner consume below uses the
        # ESTIMATED pose; the plant integration and the score keep using the
        # true `state`. With SLAM noise disabled the two are identical, so
        # this is a no-op relative to the previous behaviour.
        if slam_noise is not None:
            X_est, Y_est, psi_est = slam_noise.corrupt(X_g, Y_g, psi_g)
        else:
            X_est, Y_est, psi_est = X_g, Y_g, psi_g

        # `state_est` is `state` with only the pose entries replaced, so the
        # existing tracking-error helpers keep working unchanged (they read
        # velocity/yaw-rate entries from the same vector).
        state_est = state.copy()
        state_est[0], state_est[1], state_est[2] = X_est, Y_est, psi_est

        # Freeze the estimated pose for the duration of a hold. pose_age_ticks
        # mirrors the live pose_age_s telemetry column.
        pose_age_ticks = 0
        if pose_hold is not None:
            state_est, X_est, Y_est, psi_est, pose_age_ticks = pose_hold.apply(
                state_est, X_est, Y_est, psi_est
            )

        car_pos_np = np.array([X_est, Y_est])

        if want_history:
            # History records the TRUE trajectory — that's what "where the car
            # actually went" means for plotting and for the score.
            history["X"].append(X_g)
            history["Y"].append(Y_g)
            history["psi"].append(psi_g)
            history["v"].append(state[3])
            history["r"].append(state[5])

        # ── Tracking error + speed target (planner or oracle) ─────────────────
        rpsi = None
        if use_planner:
            # Skip perception/planning entirely while the pose is held. On the
            # car, a stalled pose feed stalls everything downstream of it: the
            # planner is triggered by car_position, so no new pose means no new
            # centreline AND no new tracking error. Re-planning here from a
            # frozen pose would still hand the controller a subtly different
            # centreline each tick (the fit is not a pure function of pose), so
            # e_y would keep changing and the controller would never actually
            # be blind — which is exactly what the first version of this model
            # got wrong (measured: e_y repeated on 0.0% of ticks instead of the
            # intended ~5%).
            if pose_age_ticks == 0:
                b_vis, y_vis = perception.visible_cones(X_est, Y_est, psi_est)
                if cone_noise is not None:
                    b_vis, y_vis = cone_noise.corrupt(b_vis), cone_noise.corrupt(y_vis)
                planner.update(b_vis, y_vis, car_pos_np, psi_est)

            cl = planner.centreline
            if cl is not None and len(cl) >= 2:
                cl_x, cl_y = cl[:, 0], cl[:, 1]
                cl_psi = np.zeros_like(cl_x)
                cl_psi[:-1] = np.arctan2(np.diff(cl_y), np.diff(cl_x))
                cl_psi[-1] = cl_psi[-2] if len(cl_psi) > 1 else state[2]

                e_y, _, e_psi, _, _, _, _ = plant_to_tracking_error(
                    state_est, path_x=cl_x, path_y=cl_y, path_psi=cl_psi
                )
                rpsi = psi_est - e_psi

                # ── Reference-heading rate limit (settings.REF_HEADING_RATE_LIMIT_ENABLED) ──
                # See _rate_limit_ref_psi's own docstring for the mechanism.
                # Only applied here (the live planner branch) — the
                # fallback/oracle branches below reference path_X/path_Y/
                # path_Psi, the fixed geometric path that does NOT carry this
                # excess, so there is nothing to limit there.
                if REF_HEADING_RATE_LIMIT_ENABLED:
                    rpsi_limited = _rate_limit_ref_psi(
                        rpsi, ref_psi_prev, np.radians(REF_HEADING_RISE_RATE), DT
                    )
                    ref_psi_prev = rpsi_limited
                    e_psi = _normalize_angle(psi_est - rpsi_limited)
                    rpsi = rpsi_limited
                else:
                    ref_psi_prev = rpsi

                if USE_PRECOMPUTED_SPEED_PROFILE:
                    # Track is already fully mapped (settings.py's
                    # USE_PRECOMPUTED_SPEED_PROFILE) -- use the oracle speed
                    # profile computed once from the WHOLE path (path_v_profile,
                    # non-causal, see speed_profile.compute_speed_profile()) at
                    # the car's current position, instead of re-deriving from
                    # only the live-built sub-path. Bypasses the perception-FOV
                    # lookahead shortfall entirely (the live centreline is
                    # typically shorter than curvature_speed()'s own scan
                    # horizon), since it needs no live cone visibility at all
                    # for the speed target. idx is one step stale here
                    # (updated later this loop, same as the path_v_profile[idx]
                    # fallback below) -- accepted, not new.
                    v_target = float(path_v_profile[idx])

                    # The oracle lookup above has no notion of the car's
                    # actual current speed relative to how much runway is
                    # left to brake for the upcoming corner — see
                    # settings.ENABLE_DYNAMIC_SPEED_CAP's docstring. Layer a
                    # live curvature-lookahead cap under it (min, never above
                    # the oracle target) so a corner reached faster than
                    # planned still gets braked for in time. Mirrors
                    # mpc_controller(_standalone).py's identical logic.
                    if ENABLE_DYNAMIC_SPEED_CAP:
                        dists = np.linalg.norm(cl - car_pos_np, axis=1)
                        cl_idx = int(np.argmin(dists))
                        v_cap = sp.curvature_speed(
                            cl[cl_idx:], v_max=PLANNER_V_MAX, v_min=PLANNER_V_MIN,
                            a_lat_max=DYNAMIC_CAP_A_LAT_MAX, safety=DYNAMIC_CAP_SAFETY,
                        )
                        v_target = min(v_target, v_cap)
                else:
                    # No pre-computed profile exists for a live-built centreline
                    # (see SimPlanner) -- derive the target speed on-demand each
                    # step from the sub-path ahead of the car, exactly as the
                    # live ROS node does via control_utils.curvature_speed().
                    dists = np.linalg.norm(cl - car_pos_np, axis=1)
                    cl_idx = int(np.argmin(dists))
                    v_target = sp.curvature_speed(
                        cl[cl_idx:], v_max=PLANNER_V_MAX, v_min=PLANNER_V_MIN
                    )

                if want_history:
                    history["planner_X"].append(cl_x)
                    history["planner_Y"].append(cl_y)
            else:
                # Planner not yet ready — fall back to the global reference path.
                # (This fallback was previously missing in gui/simulation.py, which
                # meant e_y/e_psi/v_target silently reused stale values from
                # the previous step whenever the planner wasn't ready.)
                e_y, _, e_psi, _, _, _, _ = plant_to_tracking_error(
                    state_est, path_x=path_X, path_y=path_Y, path_psi=path_Psi
                )
                v_target = float(path_v_profile[idx])

                if want_history:
                    # No planner centreline yet this step — record an empty
                    # snapshot rather than skipping the index, so history["planner_X"]
                    # stays aligned index-for-index with history["X"]/pred_X.
                    history["planner_X"].append(np.empty(0))
                    history["planner_Y"].append(np.empty(0))
        else:
            e_y, _, e_psi, _, _, _, _ = plant_to_tracking_error(
                state_est, path_x=path_X, path_y=path_Y, path_psi=path_Psi
            )
            v_target = float(path_v_profile[idx])

        # ── Tracking-error speed gate + rise-rate limit ────────────────────
        # Mirrors mpc_controller_standalone.py's Phase 3 exactly (see
        # control_utils.tracking_error_speed_gate for the rationale and the
        # live measurements behind the thresholds). curvature_speed() reads
        # only path SHAPE, so without this the target stays high — and can even
        # command acceleration — while the car is badly off-line with steering
        # already saturated, which is unrecoverable.
        #
        # Applied to the oracle branch via GATE_RATE_LIMIT -- see
        # mpc_controller_standalone.py's identical comment for the full
        # rationale (disabling the gate outright trades away its whole
        # purpose; smoothing its rate of change removes the sharp-cliff
        # side effect that motivated disabling it in the first place).
        raw_gate = sp.tracking_error_speed_gate(e_y, e_psi)
        if gate_prev is not None:
            max_step = GATE_RATE_LIMIT * DT
            raw_gate = float(np.clip(raw_gate, gate_prev - max_step, gate_prev + max_step))
        gate_prev = raw_gate
        gate = raw_gate
        v_target = max(PLANNER_V_MIN, v_target * gate)
        if v_des_prev is not None:
            v_target = min(v_target, v_des_prev + SPEED_TARGET_RISE_RATE * DT)
        v_des_prev = v_target

        # ── TRUE tracking error, for scoring only ──────────────────────────
        # e_y/e_psi above are what the CONTROLLER perceives, and they are not
        # where the car actually is whenever its reference differs from the
        # true path. Scoring and the off-track check must use ground truth,
        # otherwise a car could score well by tracking its own wrong belief —
        # the exact asymmetry real perception/localisation error has. Always
        # measured against the true reference path, never the planner's
        # estimated centreline.
        #
        # TWO independent sources make the controller's view diverge from
        # ground truth, and both must trigger this:
        #   1. SLAM noise      — the pose fed to the tracking-error helper is
        #                        corrupted (state_est != state).
        #   2. USE_PLANNER     — the REFERENCE is the planner's cone-derived,
        #                        FOV-limited, EMA-blended centreline rather
        #                        than path_X/path_Y, so e_y is a distance to
        #                        an estimated line even with a perfect pose.
        # Case 2 was previously missed: with SLAM noise off (the default),
        # e_y_true aliased the planner-relative error, so ~60% of the score
        # (rmse + peak_lateral_error) measured controller-vs-planner
        # agreement with no ground-truth anchor, and a drifting planner read
        # as good tracking while also suppressing the off-track trigger.
        if slam_noise is not None or use_planner:
            e_y_true, _, e_psi_true, _, _, _, _ = plant_to_tracking_error(
                state, path_x=path_X, path_y=path_Y, path_psi=path_Psi
            )
        else:
            e_y_true, e_psi_true = e_y, e_psi

        # ── Progress tracking (unconditional, every step) ──────────────────────
        idx, _, _, idx_rpsi = find_closest_reference_bounded(
            path_X, path_Y, path_Psi, state[0], state[1], idx, window=40
        )
        if rpsi is None:
            rpsi = idx_rpsi
        if idx > last_idx:
            cumulative_distance += np.sum(path_seg_dist[last_idx:idx])
            last_idx = idx

        if want_history:
            history["v_target"].append(v_target)
            # "e_y"/"e_psi" stay the CONTROLLER's view so existing plots keep
            # showing what it was reacting to; the *_true series is what the
            # score uses. With SLAM noise off the two are identical.
            history["e_y"].append(e_y)
            history["e_psi"].append(e_psi)
            history["e_y_true"].append(e_y_true)
            history["e_psi_true"].append(e_psi_true)

        # ── MPC state vector ────────────────────────────────────────────────
        vx_true = state[3]
        vx = max(vx_true, 0.5)
        e_y_dot = vx_true * np.sin(e_psi) + state[4] * np.cos(e_psi)
        x0_mpc = np.array([
            e_y, e_y_dot, e_psi, state[5], vx_true - v_target, 0.0, state[6], state[7],
        ])

        # ── Lookahead curvature scan (ADAPTIVE_Q_LOOKAHEAD) ─────────────────
        # Distinct from the single-point curvature_estimate() below (which
        # drives adaptive_R_rate/steer_rate_anti_hunt): scans the whole
        # speed-scaled window ahead for the largest |curvature| and the
        # total accumulated heading change, so the Q boost can anticipate a
        # corner before the car reaches it, and a long gradual U-turn is
        # recognised even with unremarkable peak curvature.
        lookahead_dist = float(np.clip(vx_true * 1.13, 3.0, 17.0))
        (kappa_max_abs, _lookahead_idx, _lookahead_peak_dist,
         lookahead_heading_change) = lookahead_curvature_profile(
            path_xy, idx, lookahead_dist
        )

        # ── Adaptive gain scaling ────────────────────────────────────────────
        kappa = curvature_estimate(state)
        R_rate_scaled = adaptive_R_rate(
            kappa, R_rate, enable_in_corners=ADAPTIVE_R_RATE_ENABLE_IN_CORNERS,
            kappa_max_abs=kappa_max_abs,
            during_floor=ADAPTIVE_R_RATE_DURING_FLOOR,
            entering_floor=ADAPTIVE_R_RATE_ENTERING_FLOOR,
            k_entering=ADAPTIVE_R_RATE_K_ENTERING,
        )
        R_rate_scaled = steer_rate_anti_hunt(
            kappa, e_y, R_rate_scaled, enabled=STEER_RATE_ANTI_HUNT_ENABLED, e_psi=e_psi
        )
        R_scaled = adaptive_R_scaling(vx, R)
        if STEER_EFFORT_STRAIGHT_BOOST_ENABLED:
            R_scaled = R_scaled.copy()
            R_scaled[0, 0] *= steer_effort_straight_boost(kappa_max_abs)

        # ── Lookahead corner-anticipation Q-boost (ADAPTIVE_Q_LOOKAHEAD) ────
        # Applied to Q FIRST, then adaptive_Q_scaling's centred-softening
        # multiplies on top of the result below -- see model_utils.py's
        # module docstring for why this ordering avoids the corner boost
        # being silently cancelled by the centred-softening floor while
        # keeping both continuous.
        if ADAPTIVE_Q_LOOKAHEAD_ENABLED:
            lookahead_peak_state = update_lookahead_peak(
                lookahead_peak_state, kappa, vx_true, DT
            )
        Q_base = adaptive_Q_lookahead(
            Q, kappa_max_abs, vx_true,
            lookahead_peak_state["last_peak_kappa_abs"],
            lookahead_peak_state["dist_since_peak"],
            lookahead_heading_change,
            enabled=ADAPTIVE_Q_LOOKAHEAD_ENABLED,
            demand_normalised=ADAPTIVE_Q_DEMAND_NORMALISED,
            ey_straight_floor=ADAPTIVE_Q_STRAIGHT_EY_FLOOR,
            ey_straight_k=ADAPTIVE_Q_STRAIGHT_EY_K,
            epsi_straight_boost_max=ADAPTIVE_Q_STRAIGHT_EPSI_BOOST_MAX,
            epsi_straight_k=ADAPTIVE_Q_STRAIGHT_K,
            r_straight_boost_max=ADAPTIVE_Q_STRAIGHT_R_BOOST_MAX,
            r_straight_k=ADAPTIVE_Q_STRAIGHT_K,
            uturn_ey_boost_max=ADAPTIVE_Q_UTURN_EY_BOOST_MAX,
            uturn_epsi_boost_max=ADAPTIVE_Q_UTURN_EPSI_BOOST_MAX,
            uturn_r_relax_floor=ADAPTIVE_Q_UTURN_R_RELAX_FLOOR,
            uturn_thresh_rad=ADAPTIVE_Q_UTURN_HEADING_THRESH_RAD,
            uturn_sat_rad=ADAPTIVE_Q_UTURN_HEADING_SAT_RAD,
            demand_half=ADAPTIVE_Q_DEMAND_HALF,
            alat_flat=ALAT_CEILING_FLAT,
            alat_slope=ALAT_CEILING_SLOPE,
            alat_intercept=ALAT_CEILING_INTERCEPT,
        )
        Q_scaled = adaptive_Q_scaling(e_y, Q_base, enabled=ADAPTIVE_Q_SCALING_ENABLED)
        Ad, Bd = model_lookup(vx, DT)

        # ── Delay compensation ───────────────────────────────────────────────
        # command_queue[0] is applied to the plant THIS step; everything after
        # it (DELAY_STEPS commands) is already committed but still in transit
        # and will land before the u_opt computed below ever reaches the
        # plant. Predict the state forward through those so the solve isn't
        # reacting to a stale x0 (see settings.py DELAY_STEPS note).
        pending_cmds = list(command_queue)[1:]
        if DELAY_JITTER_STEPS > 0.0:
            # Perturb only the controller's BELIEF about how many commands are
            # in flight — the queue itself (and so the plant's real lag) is
            # untouched. Rounding a Gaussian gives the live failure mode: the
            # estimate is usually right, occasionally off by a step, which is
            # what makes x0 jump between rollforward depths on the real car.
            n_true = len(pending_cmds)
            n_believed = int(round(n_true + delay_rng.normal(0.0, DELAY_JITTER_STEPS)))
            # Cap over-estimates at the live MAX_DELAY_COMPENSATION_STEPS
            # equivalent so a tail draw can't roll forward absurdly far.
            n_believed = int(np.clip(n_believed, 0, max(n_true, 0) + 2))
            if n_believed <= n_true:
                pending_cmds = pending_cmds[n_true - n_believed:] if n_believed else []
            else:
                # Over-estimating: the controller thinks more commands are in
                # flight than there are, so it rolls forward through the
                # oldest one extra times — the same over-compensation a
                # too-large pose_age_s produces live.
                pad = n_believed - n_true
                oldest = pending_cmds[0] if pending_cmds else u_prev
                pending_cmds = [oldest] * pad + pending_cmds
        if pending_cmds:
            x0_mpc = predict_ahead(x0_mpc, Ad, Bd, pending_cmds)

        # ── MPC solve ─────────────────────────────────────────────────────────
        mpc_result = solve_mpc(
            x0_mpc, Ad, Bd, n_horizon, Q_scaled, R_scaled, u_min, u_max,
            R_rate=R_rate_scaled, u_prev=u_prev, silent=True,
            return_status=True, eps_abs=eps, eps_rel=eps,
            max_iter=max_iter, warm_start=(step != 0),
            du_max=du_max, terminal_scale=TERMINAL_Q_SCALE,
        )

        solver_failed = mpc_result is None
        inaccurate = False
        if solver_failed:
            consecutive_fails += 1
            u_opt = u_prev.copy()
        else:
            u_opt, status = mpc_result
            consecutive_fails = 0
            inaccurate = status in (cp.OPTIMAL_INACCURATE, "optimal_inaccurate")
        if inaccurate:
            inaccurate_count_total += 1

        if want_history:
            history["solver_failed"].append(solver_failed)
            history["u_steer"].append(u_opt[0])
            history["u_accel"].append(u_opt[1])

        # ── Apply transport delay ────────────────────────────────────────────
        command_queue.append(u_opt)
        delayed_u_cmd = command_queue[0]

        # ── Horizon prediction (GUI-only, cosmetic — plant never uses this) ───
        if want_history and want_horizon_pred:
            px, py = [], []
            x_p_tmp = x0_mpc.copy()
            for k in range(n_horizon):
                e_y_pred = x_p_tmp[0]
                px.append(X_g + (k + 1) * state[3] * np.cos(psi_g) * DT - e_y_pred * np.sin(rpsi))
                py.append(Y_g + (k + 1) * state[3] * np.sin(psi_g) * DT + e_y_pred * np.cos(rpsi))
                x_p_tmp = Ad @ x_p_tmp + Bd @ u_opt
            history["pred_X"].append(px)
            history["pred_Y"].append(py)

        # ── Termination checks ──────────────────────────────────────────────
        # idx (from find_closest_reference_bounded's forward-bounded search,
        # above) only ever advances through nearby array indices — it cannot
        # jump to a spatially-close-but-far-away-in-the-path point, so
        # idx >= len(path_X) - 2 alone is a reliable "reached the end of the
        # reference array" signal regardless of the path's shape.
        #
        # The raw-distance fallback below exists only to close out the last
        # stretch when idx's bounded search hasn't quite caught up to the
        # tail (e.g. the car cuts a corner near the very end). Gating it on
        # idx already being near the end keeps it from firing anywhere else
        # the path happens to pass close to its own last point — which a
        # closed-loop recorded lap does routinely (a start/finish straight,
        # a figure-eight crossing) well before the lap is actually done.
        near_end = idx >= len(path_X) - int(0.1 * len(path_X)) - 2
        dist_to_finish = math.hypot(state[0] - path_X[-1], state[1] - path_Y[-1])
        if idx >= len(path_X) - 2 or (near_end and dist_to_finish <= 3.0):
            reached_end = True
            n_ran = step + 1
            break

        if consecutive_fails >= MAX_FAILS:
            dnf = True
            n_ran = step + 1
            if want_history:
                history["failed"] = True
                history["fail_reason"] = (
                    f"solver failed {consecutive_fails} consecutive steps at step {step}"
                )
            break

        if step > 0 and step % STALL_CHECK_INTERVAL == 0 and step > STALL_CHECK_INTERVAL:
            dist_since = cumulative_distance - dist_at_last_stall_check
            if dist_since < STALL_MIN_DISTANCE:
                first_trigger = not dnf
                dnf = True
                n_ran = step + 1
                if want_history and first_trigger:
                    history["failed"] = True
                    history["fail_reason"] = (
                        f"stalled (< {STALL_MIN_DISTANCE} m in {STALL_CHECK_INTERVAL} steps) at step {step}"
                    )
                if not continue_after_dnf:
                    break
            dist_at_last_stall_check = cumulative_distance

        # ── Metric accumulation (single source of truth: scoring.RolloutMetrics) ──
        # Scored on the TRUE error (see "TRUE tracking error" above), not the
        # controller's possibly-mislocalised belief.
        metrics.add_step(
            e_y=e_y_true, e_psi=e_psi_true, r=state[5], u_opt=u_opt,
            v_target=v_target, v_actual=state[3], u_max_steer=u_max[0],
            solver_failed=solver_failed, inaccurate=inaccurate,
        )

        if abs(e_y_true) > OFFTRACK_LIMIT:
            first_trigger = not dnf
            offtrack = True
            dnf = True
            n_ran = step + 1
            if want_history and first_trigger:
                history["failed"] = True
                history["offtrack"] = True
                history["fail_reason"] = f"off-track (|e_y|={abs(e_y_true):.2f} m) at step {step}"
            if not continue_after_dnf:
                break

        u_prev = u_opt.copy()
        state = step_nonlinear_plant(state, delayed_u_cmd, DT, vehicle_params)

    # ── Completion / time bonus (identical formula for both callers) ──────────
    progress = cumulative_distance / path_length if path_length > 0 else 0.0
    progress = float(np.clip(progress, 0.0, 1.0))

    sim_time = n_ran * DT
    if reached_end:
        # Anchor the time bonus to the path's PHYSICAL optimum where available.
        #
        # This used to divide by `dynamic_max_steps * DT`, i.e.
        # arc_length / 2.5 m/s * 1.5 — a placeholder step budget with no
        # physical meaning, measured to be 2.7x-6.7x the actual optimum and
        # varying by path. That made TIME_BONUS_WEIGHT (0.25, the
        # second-largest score term) a reward against an arbitrary constant,
        # and made the bonus non-comparable BETWEEN paths: the same driving
        # earned wildly different bonuses depending on which track it was on.
        #
        # optimal_lap_time() is a quasi-steady-state bound (corner limit ->
        # forward accel pass -> backward brake pass -> integrate ds/v), so
        # time_bonus is now "how close to physically-fastest", in [0, 1] and
        # directly comparable across paths. Because the bound ignores transient
        # dynamics it is not quite attainable, so a real run scores below 1.0.
        # Ratio form, NOT (1 - sim/optimal): sim_time is always >= optimal_time
        # (it's a lower bound), so that subtraction would clip to 0 on every
        # run and the term would carry no information at all. optimal/sim is 1.0
        # at the physical limit and decays toward 0 as the run gets slower —
        # e.g. taking twice the optimal time scores 0.5.
        #
        # optimal_time covers the WHOLE path, but the rollout stops as soon as
        # the car is within 3 m of the finish and past ~90% of the points (see
        # the reached_end check above), so it is only timed over `progress` of
        # the distance. Comparing a 95%-of-the-path run against a 100%-of-the-
        # path reference makes the car look faster than physically possible —
        # measured, 7 of 10 paths saturated at exactly 1.000 before this scaling
        # was applied, destroying all discrimination in the primary objective.
        # Scale the reference to the distance actually covered.
        if optimal_time is not None and optimal_time > 0.0 and sim_time > 0.0:
            ref_time = optimal_time * max(progress, 1e-6)
            time_bonus = float(np.clip(ref_time / sim_time, 0.0, 1.0))
        else:
            expected_time = dynamic_max_steps * DT
            time_bonus = max(0.0, 1.0 - (sim_time / expected_time))
    else:
        time_bonus = 0.0

    metrics_result = metrics.finalize(
        progress=progress, time_bonus=time_bonus, dnf=dnf, offtrack=offtrack,
        reached_end=reached_end,
    )
    # inaccurate_count from metrics.finalize() only counts steps that made it
    # through add_step(); include steps that were skipped by an early break too.
    metrics_result["inaccurate_count"] = max(
        metrics_result["inaccurate_count"], inaccurate_count_total
    )

    if want_history:
        history.setdefault("reached_end", False)
        history["reached_end"] = reached_end
        history["peak_lateral_error"] = metrics.peak_lateral_error
        history["completion_frac"] = progress
        history["time_bonus"] = time_bonus
        history["inaccurate_count"] = inaccurate_count_total
        if not reached_end and not history["failed"]:
            # Ran out of steps without reaching the end or triggering a DNF
            # condition — treat as a failure for scoring/labelling purposes,
            # matching the previous gui/simulation.py behaviour.
            history["failed"] = True

    return {
        "composite_score": metrics_result["composite_score"],
        "metrics_result": metrics_result,
        "progress": progress,
        "reached_end": reached_end,
        "dnf": dnf,
        "offtrack": offtrack,
        "time_bonus": time_bonus,
        "history": history,
    }