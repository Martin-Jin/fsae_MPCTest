"""
sim/rollout_phases.py — the per-tick phases of run_core_rollout(), lifted out
of its loop so the loop itself reads as a sequence of named steps.

Every function here is stateless: any value the loop carries from one tick to
the next (v_des_prev, gate_prev, ref_psi_prev, ...) is passed in and returned
explicitly rather than stored, so the loop in sim/rollout_core.py remains the
one place that owns rollout state.

settings.py constants are bound here by name at import time, the same as in
rollout_core.py. Callers that override settings (tuner/steering_chatter_check.py)
must do so before the first import of sim.rollout_core, which imports this
module.
"""

import numpy as np
import cvxpy as cp

from model.vehicle_physics import plant_to_tracking_error
from controller.lmpc import solve_mpc
from controller.nmpc import NMPCController
from controller.model_utils import (
    curvature_estimate, adaptive_R_scaling, adaptive_Q_scaling,
    steer_rate_anti_hunt, reversal_penalty_boost, _corner_factor, _blend,
    _low_speed_corner_boost,
)
import sim.speed_profile as sp

from settings import (
    DT, DELAY_JITTER_STEPS, REF_HEADING_RATE_LIMIT_ENABLED,
    REF_HEADING_RISE_RATE, TERMINAL_Q_SCALE, ADAPTIVE_Q_SCALING_ENABLED,
    USE_PRECOMPUTED_SPEED_PROFILE, STEER_RATE_ANTI_HUNT_ENABLED,
    REVERSAL_PENALTY_ENABLED, REVERSAL_PENALTY_BOOST_MAX, REVERSAL_PENALTY_K,
    ENABLE_DYNAMIC_SPEED_CAP, DYNAMIC_CAP_A_LAT_MAX, DYNAMIC_CAP_SAFETY,
    ALAT_CEILING_FLAT, ALAT_CEILING_SLOPE, ALAT_CEILING_INTERCEPT, R_A_ACCEL,
    R_A_BRAKE, CORNER_FACTOR_K, Q_EY_STRAIGHT, Q_EY_CORNER, Q_EPSI_STRAIGHT,
    Q_EPSI_CORNER, Q_R_STRAIGHT, Q_R_CORNER, RRATE_STEER_STRAIGHT,
    RRATE_STEER_CORNER, R_STEER_CORNER_MID, LOW_SPEED_CORNER_BOOST_V_HALF,
    LOW_SPEED_CORNER_BOOST_MAX_EXTRA, EPSI_RA_HALF_RAD,
    EPSI_RA_ACCEL_BOOST_MAX, EPSI_RA_BRAKE_FLOOR, NMPC_HORIZON,
    NMPC_SQP_ITERS, NMPC_SOLVE_BUDGET_MS, NMPC_RK_SUBSTEPS, NMPC_JAC_SUBSTEPS,
    NMPC_JAC_GATE_SPEED, NMPC_JAC_SUBSTEPS_FAST, NMPC_RK_GATE_SPEED,
    NMPC_RK_SUBSTEPS_FAST, NMPC_STANDSTILL_STEER_DAMP_ENABLED,
    NMPC_STANDSTILL_SPEED, NMPC_STANDSTILL_FADE_SPEED,
    NMPC_STANDSTILL_STEER_R_SCALE, NMPC_TRUST_DELTA_RAD, NMPC_TRUST_A,
    NMPC_BACKTRACK_MAX, NMPC_TRACK_HALFWIDTH, NMPC_SLACK_WEIGHT,
    NMPC_CURVATURE_DENSE_STEP, NMPC_CURVATURE_SMOOTH_W, NMPC_KAPPA_CLIP,
    NMPC_KAPPA_RATE_MAX, NMPC_OSQP_MAX_ITER, NMPC_OSQP_EPS,
    NMPC_ALAT_CEILING_ENABLED, NMPC_Q_E_Y, NMPC_Q_E_YD, NMPC_Q_E_PSI,
    NMPC_Q_EPSI_DOT, NMPC_Q_E_V, NMPC_R_DELTA, NMPC_R_A_ACCEL, NMPC_R_A_BRAKE,
    NMPC_R_RATE_DELTA, NMPC_R_RATE_A, NMPC_TERMINAL_SCALE,
    NMPC_SPLINE_REFERENCE_ENABLED, NMPC_FRICTION_CIRCLE_ENABLED,
    NMPC_LATENCY_COMPENSATION_ENABLED, NMPC_LATENCY_COMPENSATION_MS,
    NMPC_STEER_RATE_ANTI_HUNT_ENABLED, NMPC_CORNER_RRATE_BLEND_ENABLED,
    NMPC_CORNER_FACTOR_K, NMPC_RRATE_STEER_STRAIGHT, NMPC_RRATE_STEER_CORNER,
    NMPC_REVERSAL_PENALTY_ENABLED, NMPC_REVERSAL_PENALTY_BOOST_MAX,
    NMPC_REVERSAL_PENALTY_K, NMPC_RRATE_STAGE_RAMP_ENABLED,
    NMPC_RRATE_STAGE_NEAR, NMPC_RJERK_DELTA, NMPC_RJERK_A,
    NMPC_RRATE_ZONE_ENABLED, NMPC_RRATE_ZONE_BOOST_STRAIGHT,
    NMPC_RRATE_ZONE_EASE_APPROACH, NMPC_RRATE_ZONE_FLOOR_CORNER,
    NMPC_PROGRESS_ENABLED, NMPC_Q_PROGRESS, NMPC_PROGRESS_REACH,
    NMPC_PROGRESS_V_MIN, NMPC_SLACK_LINEAR_WEIGHT, SPEED_TARGET_DEFICIT_MAX,
)

# v_max/v_min for the live-planner branch's speed_profile.curvature_speed() call.
# Mirror fsds_simulator/control/fsae_control/fsae_control/mpc/mpc_controller.py's
# v_max/v_min ROS parameters (default V_MAX/V_MIN) — and the old SimPlanner
# defaults these replace — so offline-tuned weights see the same speed targets
# the live node will command.
PLANNER_V_MAX = 20.0
PLANNER_V_MIN = 1.5

# Max rate (m/s^2) at which the speed TARGET may rise. Mirrors
# mpc_controller.SPEED_TARGET_RISE_RATE — keep the two in sync.
# Decreases are never rate-limited; only the rise is damped, to suppress the
# planner's frame-to-frame curvature jitter without capping real acceleration.
SPEED_TARGET_RISE_RATE = 7.0

# Max speed error (m/s) the rise limiter is allowed to open up before it stops
# ramping and waits for the car. Promoted to MPCParams.speed_target_deficit_max
# on the live side (ROS param/YAML/launch arg/GUI tunable); imported here from
# settings.py's SPEED_TARGET_DEFICIT_MAX, which is the offline mirror of that
# same field, kept in sync by hand like every other entry in that file.
#
# SPEED_TARGET_RISE_RATE alone assumes the car can accelerate at that rate. From
# a standing start it cannot: the car does not break static friction for ~1 s,
# so the target ramps to ~7 m/s while the car is still stationary and banks a
# deficit it spends the next second chasing. The NMPC minimises one scalar cost
# over the horizon, so a speed error that large swamps the lateral term and the
# optimiser trades e_y away for speed it was never going to get — measured as a
# sideways excursion at launch that self-corrects once the car is rolling.
#
# Capping the DEFICIT rather than gating on measured speed is deliberate. A gate
# of the form "hold the target while v_actual is near zero" deadlocks: no target
# means no speed error, which means no throttle, which means the car never moves
# and the gate never opens. Holding at v_actual + DEFICIT_MAX always leaves a
# real speed error, so throttle still commands and the launch still happens; the
# ramp resumes by itself as the car closes the gap.
#
# Not specific to launch: the same rule stops the target running away after a
# spin or a heavy brake, for the same reason.
#
# 5.0, not the original 2.5. At 2.5 the clamp is not a launch/recovery guard
# at all, it is the binding constraint on acceleration for a THIRD of a
# normal lap: measured 36.8% of ticks pinned at exactly the limit, holding
# a_cmd to 4.45 against a plant that delivers ~12. Raising it to 5.0 drops
# the pinned fraction to 2.3%, nearly doubles peak a_cmd to 8.32, and
# improves every metric at once rather than trading any against another:
#
#   DEFICIT_MAX   score (3 runs)        lap steps   a_cmd max   |e_y| mean   steer sat
#   2.5           0.757/0.804/0.757     1081-1117   4.45        0.418        4.71%
#   5.0           0.693/0.692/0.693     1033-1034   8.32        0.402        3.77%
#
# Lower score is better. The launch behaviour the clamp exists to protect is
# unchanged (launch at step 9 either way, launch-phase |e_y| 0.27 m against
# a 3.5 m boundary), which is why the guard still does its job at 5.0. Run
# to run spread also collapses (0.001 vs 0.047), because the clamp is no
# longer arbitrating most of the lap.
#
# Values above ~5 buy nothing further (10.0 and 100.0 both plateau at
# a_cmd 8.87 and score no better), so this is the knee, not a ceiling to
# keep raising. Do not read it as "the clamp was wrong": it is a real guard
# and still needed, it was simply set tight enough to bind far outside the
# regime it was designed for.

# Max rate (gate-units/s) at which tracking_error_speed_gate()'s output may
# change per tick, in either direction. Mirrors
# mpc_controller.GATE_RATE_LIMIT — keep the two in sync. See that
# constant's own comment for the full rationale.
GATE_RATE_LIMIT = 2.0

# Max rate (m/s^2) at which curvature_speed()'s OWN output (the live,
# per-step centreline-derived target, NOT the precomputed-profile oracle
# lookup) may fall. Mirrors mpc_controller.V_CURV_FALL_RATE — keep the two
# in sync. See that constant's own comment for the full history: an initial
# 5.0 (speed_profile.A_BRAKE_PLAN, the PLANNING-time deceleration
# curvature_speed()'s braking-distance propagation assumes) capped genuine
# hard braking below what the car can do (measured live 2026-09-15: car
# entered the first corner at ~17 m/s, took 3+ s to reach the real ~2.5 m/s
# target, spun out before arriving). Now 7.0, matching mpc_core.MAX_BRAKE /
# vehicle_physics.max_accel_brake, the car's actual achievable braking
# deceleration rather than a conservative planning assumption.
V_CURV_FALL_RATE = 7.0


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


def _nmpc_pick(override, base):
    """-1.0 (or None) = inherit `base`; otherwise use `override`. See
    settings.py's "NMPC weight overrides" comment."""
    return base if override is None or override < 0.0 else override


def build_nmpc(Q, R, R_rate, u_min, u_max, du_max, vehicle_params, nmpc_overrides):
    """
    Construct the rollout's NMPCController. Called ONCE per rollout, so its
    warm-started SQP solution persists tick to tick exactly like the LTV
    path's u_prev/command_queue do. See controller/nmpc/'s package docstring
    for the full design, and settings.py's "NMPC weight overrides" comment
    for why Q/R/R_rate (the CURRENT weight set: settings.py's tuned values or
    a CMA-ES candidate, whichever this rollout was called with) rather than
    settings.py's constants directly are what the overrides inherit from.

    `nmpc_overrides` is run_core_rollout()'s per-call override dict, keyed by
    the controller's own kwarg name; see that function's docstring.
    """
    _nmpc_ov = nmpc_overrides or {}

    def _ov(name, default):
        return _nmpc_ov.get(name, default)

    return NMPCController(
        dt=DT, N=NMPC_HORIZON, vehicle_params=vehicle_params,
        u_min=u_min, u_max=u_max, du_max=du_max,
        q_e_y=_nmpc_pick(NMPC_Q_E_Y, Q[0, 0]),
        q_e_yd=_nmpc_pick(NMPC_Q_E_YD, Q[1, 1]),
        q_e_psi=_nmpc_pick(NMPC_Q_E_PSI, Q[2, 2]),
        q_epsi_dot=_nmpc_pick(NMPC_Q_EPSI_DOT, Q[3, 3]),
        q_e_v=_nmpc_pick(NMPC_Q_E_V, Q[4, 4]),
        r_delta=_nmpc_pick(NMPC_R_DELTA, R[0, 0]),
        r_a_accel=_nmpc_pick(NMPC_R_A_ACCEL, R_A_ACCEL),
        r_a_brake=_nmpc_pick(NMPC_R_A_BRAKE, R_A_BRAKE),
        r_rate_delta=_nmpc_pick(NMPC_R_RATE_DELTA, R_rate[0, 0]),
        r_rate_a=_nmpc_pick(NMPC_R_RATE_A, R_rate[1, 1]),
        terminal_scale=_nmpc_pick(NMPC_TERMINAL_SCALE, TERMINAL_Q_SCALE),
        sqp_iters=NMPC_SQP_ITERS, solve_budget_ms=NMPC_SOLVE_BUDGET_MS,
        rk_substeps=NMPC_RK_SUBSTEPS, jac_substeps=NMPC_JAC_SUBSTEPS,
        jac_gate_speed=NMPC_JAC_GATE_SPEED, jac_substeps_fast=NMPC_JAC_SUBSTEPS_FAST,
        rk_gate_speed=NMPC_RK_GATE_SPEED, rk_substeps_fast=NMPC_RK_SUBSTEPS_FAST,
        standstill_steer_damp_enabled=NMPC_STANDSTILL_STEER_DAMP_ENABLED,
        standstill_speed=NMPC_STANDSTILL_SPEED,
        standstill_fade_speed=NMPC_STANDSTILL_FADE_SPEED,
        standstill_steer_r_scale=NMPC_STANDSTILL_STEER_R_SCALE,
        trust_delta_rad=NMPC_TRUST_DELTA_RAD, trust_a=NMPC_TRUST_A,
        backtrack_max=NMPC_BACKTRACK_MAX,
        track_halfwidth=NMPC_TRACK_HALFWIDTH, slack_weight=NMPC_SLACK_WEIGHT,
        slack_linear_weight=_ov('slack_linear_weight', NMPC_SLACK_LINEAR_WEIGHT),
        osqp_max_iter=NMPC_OSQP_MAX_ITER, osqp_eps=NMPC_OSQP_EPS,
        alat_ceiling_enabled=NMPC_ALAT_CEILING_ENABLED,
        alat_flat=ALAT_CEILING_FLAT, alat_slope=ALAT_CEILING_SLOPE,
        alat_intercept=ALAT_CEILING_INTERCEPT,
        spline_reference_enabled=NMPC_SPLINE_REFERENCE_ENABLED,
        friction_circle_enabled=NMPC_FRICTION_CIRCLE_ENABLED,
        steer_rate_anti_hunt_enabled=NMPC_STEER_RATE_ANTI_HUNT_ENABLED,
        corner_rrate_blend_enabled=NMPC_CORNER_RRATE_BLEND_ENABLED,
        corner_factor_k=_ov('corner_factor_k',
                            _nmpc_pick(NMPC_CORNER_FACTOR_K, CORNER_FACTOR_K)),
        rrate_steer_straight=_nmpc_pick(NMPC_RRATE_STEER_STRAIGHT, RRATE_STEER_STRAIGHT),
        rrate_steer_corner=_nmpc_pick(NMPC_RRATE_STEER_CORNER, RRATE_STEER_CORNER),
        reversal_penalty_enabled=NMPC_REVERSAL_PENALTY_ENABLED,
        reversal_penalty_boost_max=_nmpc_pick(
            NMPC_REVERSAL_PENALTY_BOOST_MAX, REVERSAL_PENALTY_BOOST_MAX),
        reversal_penalty_k=_nmpc_pick(NMPC_REVERSAL_PENALTY_K, REVERSAL_PENALTY_K),
        rrate_stage_ramp_enabled=_ov('rrate_stage_ramp_enabled', NMPC_RRATE_STAGE_RAMP_ENABLED),
        rrate_stage_near=_ov('rrate_stage_near', NMPC_RRATE_STAGE_NEAR),
        rrate_zone_enabled=_ov('rrate_zone_enabled', NMPC_RRATE_ZONE_ENABLED),
        rrate_zone_boost_straight=_ov('rrate_zone_boost_straight', NMPC_RRATE_ZONE_BOOST_STRAIGHT),
        rrate_zone_ease_approach=_ov('rrate_zone_ease_approach', NMPC_RRATE_ZONE_EASE_APPROACH),
        rrate_zone_floor_corner=_ov('rrate_zone_floor_corner', NMPC_RRATE_ZONE_FLOOR_CORNER),
        rjerk_delta=_ov('rjerk_delta', NMPC_RJERK_DELTA),
        rjerk_a=_ov('rjerk_a', NMPC_RJERK_A),
        latency_compensation_enabled=NMPC_LATENCY_COMPENSATION_ENABLED,
        latency_compensation_ms=NMPC_LATENCY_COMPENSATION_MS,
        kappa_rate_max=NMPC_KAPPA_RATE_MAX,
        progress_enabled=_ov('progress_enabled', NMPC_PROGRESS_ENABLED),
        q_progress=_ov('q_progress', NMPC_Q_PROGRESS),
        progress_reach=_ov('progress_reach', NMPC_PROGRESS_REACH),
        progress_v_min=_ov('progress_v_min', NMPC_PROGRESS_V_MIN),
    )


def compute_reference(
    use_planner, perception, planner, cone_noise, pose_age_ticks,
    state, state_est, X_est, Y_est, psi_est, car_pos_np,
    path_X, path_Y, path_Psi, ref_psi_prev, history,
):
    """
    The CONTROLLER's view of tracking error, from the live planner's
    centreline when one is ready, otherwise the oracle path.

    Returns (e_y, e_psi, rpsi, planner_cl, ref_psi_prev, cl_idx). `rpsi` is
    None unless the planner branch produced one. `planner_cl` is the planner
    centreline this tick's error was measured against, or None when the
    oracle path was used (the NMPC must track the same source). `cl_idx` is
    the nearest-point index into `planner_cl` (None when `planner_cl` is
    None), reused by compute_speed_target() so it isn't recomputed twice per
    tick. Appends the planner-centreline snapshot to `history` when it is
    not None.
    """
    rpsi = None
    planner_cl = None
    cl_idx = None
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
            planner_cl = cl
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

            dists = np.linalg.norm(cl - car_pos_np, axis=1)
            cl_idx = int(np.argmin(dists))

            if history is not None:
                history["planner_X"].append(cl_x)
                history["planner_Y"].append(cl_y)
        else:
            # Planner not yet ready — fall back to the global reference path.
            # Without this fallback, e_y/e_psi would silently reuse stale
            # values from the previous step whenever the planner isn't ready.
            e_y, _, e_psi, _, _, _, _ = plant_to_tracking_error(
                state_est, path_x=path_X, path_y=path_Y, path_psi=path_Psi
            )

            if history is not None:
                # No planner centreline yet this step — record an empty
                # snapshot rather than skipping the index, so history["planner_X"]
                # stays aligned index-for-index with history["X"]/pred_X.
                history["planner_X"].append(np.empty(0))
                history["planner_Y"].append(np.empty(0))
    else:
        e_y, _, e_psi, _, _, _, _ = plant_to_tracking_error(
            state_est, path_x=path_X, path_y=path_Y, path_psi=path_Psi
        )

    return e_y, e_psi, rpsi, planner_cl, ref_psi_prev, cl_idx


def compute_speed_target(
    planner_cl, cl_idx, car_pos_np, path_v_profile, idx, v_curv_prev,
):
    """
    The raw speed target: the oracle profile plus a live curvature cap when
    a planner centreline is available, otherwise the oracle profile alone.

    `planner_cl`/`cl_idx` come from compute_reference()'s return (None when
    no planner centreline was ready this tick, which folds into the "no
    live centreline" branch below exactly like the pre-split function's
    fallback/no-planner cases did — both only ever read path_v_profile[idx]).

    Returns (v_target, v_curv_prev).
    """
    if planner_cl is None:
        return float(path_v_profile[idx]), v_curv_prev

    cl = planner_cl
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
        # (updated later in the loop, same as the path_v_profile[idx]
        # fallback below) -- accepted, not new.
        v_target = float(path_v_profile[idx])

        # The oracle lookup above has no notion of the car's
        # actual current speed relative to how much runway is
        # left to brake for the upcoming corner — see
        # settings.ENABLE_DYNAMIC_SPEED_CAP's docstring. Layer a
        # live curvature-lookahead cap under it (min, never above
        # the oracle target) so a corner reached faster than
        # planned still gets braked for in time. Mirrors
        # mpc_controller.py's identical logic.
        if ENABLE_DYNAMIC_SPEED_CAP:
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
        v_target = sp.curvature_speed(
            cl[cl_idx:], v_max=PLANNER_V_MAX, v_min=PLANNER_V_MIN
        )
        # curvature_speed() has no memory of its own last output
        # and the live centreline is rebuilt every step, so a
        # single noisy sample can swing v_target down far faster
        # than any real corner's own braking-distance curve would
        # ask for -- see V_CURV_FALL_RATE's own comment. Mirrors
        # mpc_controller.py's identical fix.
        if v_curv_prev is not None:
            max_fall = V_CURV_FALL_RATE * DT
            v_target = max(v_target, v_curv_prev - max_fall)
        v_curv_prev = v_target

    return v_target, v_curv_prev


def gate_and_rate_limit_speed_target(v_target, e_y, e_psi, v_actual, gate_prev, v_des_prev):
    """
    Tracking-error speed gate plus the target's rise-rate and deficit limits.
    Returns (v_target, gate_prev, v_des_prev).

    Mirrors mpc_controller.py's Phase 3 exactly (see
    control_utils.tracking_error_speed_gate for the rationale and the live
    measurements behind the thresholds). curvature_speed() reads only path
    SHAPE, so without the gate the target stays high, and can even command
    acceleration, while the car is badly off-line with steering already
    saturated, which is unrecoverable.

    The gate's own rate of change is limited by GATE_RATE_LIMIT, see
    mpc_controller.py's identical comment for the full rationale (disabling
    the gate outright trades away its whole purpose; smoothing its rate of
    change removes the sharp-cliff side effect that motivated disabling it).
    """
    raw_gate = sp.tracking_error_speed_gate(e_y, e_psi)
    if gate_prev is not None:
        max_step = GATE_RATE_LIMIT * DT
        raw_gate = float(np.clip(raw_gate, gate_prev - max_step, gate_prev + max_step))
    gate_prev = raw_gate
    gate = raw_gate
    v_target = max(PLANNER_V_MIN, v_target * gate)
    # Seed the ramp from the car's ACTUAL speed on the first tick, not
    # from an unlimited jump straight to v_target -- see mpc_controller.py's
    # identical fix and CLAUDE.md's standstill steering-saturation note.
    # Without this, a standing start (vx0=0.0) asks the NMPC to track the
    # full-speed target from tick 0 via its e_v cost term, which is the
    # actual root cause of the "steers hard at startup" symptom -- not a
    # plant/tyre-force bug.
    if v_des_prev is None:
        v_des_prev = v_actual
    v_target = min(v_target, v_des_prev + SPEED_TARGET_RISE_RATE * DT)
    # Stop ramping once the target has run this far ahead of the car; see
    # SPEED_TARGET_DEFICIT_MAX. Never DROPS the target (max against
    # v_des_prev), so a car that is merely slow does not get the target
    # dragged down to meet it, and a genuine brake request still passes
    # through the min() above untouched.
    if v_target - v_actual > SPEED_TARGET_DEFICIT_MAX:
        v_target = min(v_target, max(v_des_prev, v_actual + SPEED_TARGET_DEFICIT_MAX))
    v_des_prev = v_target
    return v_target, gate_prev, v_des_prev


def true_tracking_error(state, e_y, e_psi, path_X, path_Y, path_Psi, diverged):
    """
    Ground-truth tracking error, for scoring and the off-track check only.

    The controller's e_y/e_psi are not where the car actually is whenever its
    reference differs from the true path. Scoring must use ground truth,
    otherwise a car could score well by tracking its own wrong belief, the
    exact asymmetry real perception/localisation error has. Always measured
    against the true reference path, never the planner's centreline.

    `diverged` must be True whenever EITHER source of divergence is active:
      1. SLAM noise:  the pose fed to the tracking-error helper is corrupted.
      2. use_planner: the REFERENCE is the planner's cone-derived, FOV-limited,
                      EMA-blended centreline, so e_y is a distance to an
                      estimated line even with a perfect pose.
    Case 2 must count even with SLAM noise off (the default): otherwise
    e_y_true would alias the planner-relative error, so most of the score
    (rmse + peak_lateral_error) would measure controller-vs-planner agreement
    with no ground-truth anchor, and a drifting planner would read as good
    tracking while also suppressing the off-track trigger.
    """
    if diverged:
        e_y_true, _, e_psi_true, _, _, _, _ = plant_to_tracking_error(
            state, path_x=path_X, path_y=path_Y, path_psi=path_Psi
        )
        return e_y_true, e_psi_true
    return e_y, e_psi


def believed_pending_cmds(command_queue, delay_rng, u_prev):
    """
    Commands the CONTROLLER believes are still in transit to the plant.

    command_queue[0] is applied to the plant THIS step; everything after it
    (DELAY_STEPS commands) is already committed and will land before a new
    solve's output ever reaches the plant, so the controller rolls its state
    forward through them (see settings.py DELAY_STEPS note).

    With DELAY_JITTER_STEPS > 0, only the controller's BELIEF about how many
    commands are in flight is perturbed; the queue itself (and so the plant's
    real lag) is untouched. Rounding a Gaussian gives the live failure mode:
    the estimate is usually right, occasionally off by a step, which is what
    makes x0 jump between rollforward depths on the real car. Draws exactly
    one sample from `delay_rng` per call, so call it once per tick.
    """
    pending_cmds = list(command_queue)[1:]
    if DELAY_JITTER_STEPS > 0.0:
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
    return pending_cmds


def solve_nmpc_tick(nmpc, planner_cl, path_xy, car_pos_np, psi_est, state_est,
                    vx_true, v_target, pending_cmds, step):
    """
    One NMPC solve. Returns (u_opt, nmpc_diag).

    Deliberately skips the whole adaptive-gain schedule solve_ltv_tick()
    runs (current-state corner-factor scheduler, adaptive_R_scaling,
    steer_rate_anti_hunt, adaptive_Q_scaling, heading-error accel/brake
    asymmetry): all of it exists to synthesise anticipation the LINEAR model
    cannot produce on its own. The nonlinear model anticipates a bend
    structurally (kappa(s) is looked up from a STATE, arc length, not
    reweighted after the fact), so applying the same mechanisms on top would
    double-count an effect that is now built into the prediction. See
    settings.py's USE_NMPC comment.

    The warm-start-projection invariant (see NMPCController._project_feasible)
    means compute_step() always ships a feasible u_opt, even on a tick where
    the SQP subproblem itself did not solve, so "solver failed" in the LTV
    sense does not apply; nmpc_diag['status']/['solved'] carry the SQP's own
    per-tick outcome instead.
    """
    # The NMPC tracks its own Frenet path reference and needs the SAME path
    # source e_y/e_psi were computed against (the live planner's centreline
    # when one is ready, otherwise the oracle path).
    if planner_cl is not None:
        nmpc_path_xy = np.column_stack([planner_cl[:, 0], planner_cl[:, 1]])
    else:
        nmpc_path_xy = path_xy

    return nmpc.compute_step(
        nmpc_path_xy, car_pos_np, psi_est, vx_true, v_target,
        car_yaw_rate=state_est[5], car_vy=state_est[4],
        pending_cmds=(pending_cmds if pending_cmds else None),
        dense_step=NMPC_CURVATURE_DENSE_STEP,
        smooth_w=NMPC_CURVATURE_SMOOTH_W, kappa_clip=NMPC_KAPPA_CLIP,
        step_index=step,
    )


def solve_ltv_tick(
    state, x0_mpc, e_y, e_psi, vx, vx_true, u_prev, pending_cmds,
    Q, R, R_rate, u_min, u_max, du_max, model_lookup,
    n_horizon, eps, max_iter, step,
):
    """
    One linear time-varying QP solve, with the current-state gain schedule
    and delay compensation applied first.

    Returns (u_opt, solver_failed, inaccurate, x0_mpc, Ad, Bd). On a solver
    failure u_opt is a copy of u_prev. The returned x0_mpc is the
    delay-compensated state the QP solved against (the GUI's horizon preview
    starts from it).
    """
    # ── Current-state corner factor ───────────────────────────────────
    # Replaces the deleted lookahead gain-scheduling family (see
    # model_utils.py's module docstring): 0 (straight) -> 1 (full
    # corner), a single continuous saturating curve of the CURRENT
    # ~instantaneous curvature `kappa` -- the same signal
    # steer_rate_anti_hunt already uses. No forward
    # scan, no separate decay-distance timer/hysteresis state: entry
    # and exit are symmetric, driven purely by how `kappa` itself
    # rises and falls.
    kappa = curvature_estimate(state)
    corner_factor = _corner_factor(kappa, CORNER_FACTOR_K)

    # Extra push in the SAME direction as corner_factor's "full
    # corner" endpoint, active only when BOTH corner_factor > 0 AND
    # speed is low -- gated on corner_factor (multiplicatively) so
    # this cannot fire on low speed alone with no corner, unlike the
    # deleted low_speed_steer_rate_boost (which fired on speed alone
    # and ended up taxing wanted low-speed turn-in indistinguishably
    # from unwanted post-exit wobble).
    low_speed_boost = _low_speed_corner_boost(
        vx_true, corner_factor,
        v_half=LOW_SPEED_CORNER_BOOST_V_HALF,
        max_extra=LOW_SPEED_CORNER_BOOST_MAX_EXTRA,
    )
    # Combined corner fraction driving every blend below:
    # corner_factor itself, boosted further (never past 1.0) at low
    # speed in-corner.
    corner_frac = float(np.clip(corner_factor + low_speed_boost, 0.0, 1.0))

    # ── Adaptive gain scaling ────────────────────────────────────────
    # R_rate_scaled[0,0]'s base value is set below by the corner-factor
    # blend, so it starts as a plain copy here (the removed adaptive_R_rate
    # current-curvature floor used to scale it first, but that scale was
    # always overwritten by the blend before it could reach the QP, see
    # docs/removed_mechanisms.md).
    R_rate_scaled = np.array(R_rate, copy=True)
    _rr_before_hunt = float(R_rate_scaled[0, 0])
    R_rate_scaled = steer_rate_anti_hunt(
        kappa, e_y, R_rate_scaled, enabled=STEER_RATE_ANTI_HUNT_ENABLED, e_psi=e_psi,
    )
    m_rrate_antihunt = (
        float(R_rate_scaled[0, 0] / _rr_before_hunt) if _rr_before_hunt else 1.0
    )
    _rr_before_reversal = float(R_rate_scaled[0, 0])
    R_rate_scaled = reversal_penalty_boost(
        float(u_prev[0]), R_rate_scaled, enabled=REVERSAL_PENALTY_ENABLED,
        boost_max=REVERSAL_PENALTY_BOOST_MAX, k=REVERSAL_PENALTY_K,
    )
    m_rrate_reversal = (
        float(R_rate_scaled[0, 0] / _rr_before_reversal) if _rr_before_reversal else 1.0
    )
    R_scaled = adaptive_R_scaling(vx, R)

    # ── Current-state Q[0,0]/Q[2,2]/Q[3,3] and R_rate[0,0] blend ─────
    # Straight-line-blend, PER WEIGHT, between a "straight" endpoint
    # and a "full corner" endpoint, driven by corner_frac above.
    # Replaces the deleted per-mechanism multiplicative lookahead
    # gates with one shared current-state schedule. R[0,0] (steering
    # effort) is a special case: blended toward a MIDDLE value, not
    # the same corner-floor extreme as R_rate/Q[3,3], per the user's
    # own framing ("should be somewhere in between the two extremes
    # to discourage saturation").
    Q_base = np.array(Q, copy=True)
    Q_base[0, 0] = _blend(Q_EY_STRAIGHT, Q_EY_CORNER, corner_frac)
    Q_base[2, 2] = _blend(Q_EPSI_STRAIGHT, Q_EPSI_CORNER, corner_frac)
    Q_base[3, 3] = _blend(Q_R_STRAIGHT, Q_R_CORNER, corner_frac)

    # CAUTION: this line sets R_rate_scaled[0,0]'s BASE value, so
    # every multiplier computed above (m_rrate_antihunt,
    # m_rrate_reversal, and any future one) must be explicitly
    # reapplied here too -- an assignment that omits one silently
    # discards its effect even though the multiplier's own value is
    # still correctly logged elsewhere. See mpc_core.py's matching
    # comment; this exact class of bug has recurred more than once.
    R_rate_scaled = R_rate_scaled.copy()
    R_rate_scaled[0, 0] = _blend(
        RRATE_STEER_STRAIGHT, RRATE_STEER_CORNER, corner_frac
    ) * m_rrate_antihunt * m_rrate_reversal

    R_scaled = R_scaled.copy()
    R_scaled[0, 0] = _blend(R_scaled[0, 0], R_STEER_CORNER_MID, corner_frac)

    Q_scaled = adaptive_Q_scaling(e_y, Q_base, enabled=ADAPTIVE_Q_SCALING_ENABLED)
    Ad, Bd = model_lookup(vx, DT)

    # ── Delay compensation ───────────────────────────────────────────
    # Solve against the state the car will be in when this solve's output
    # lands, not the stale current one. See believed_pending_cmds().
    if pending_cmds:
        x0_mpc = predict_ahead(x0_mpc, Ad, Bd, pending_cmds)

    # ── Heading-error-driven accel/brake asymmetry ────────────────────
    # Always-on, independent of the corner_frac scheduler above: a
    # continuous 0->1 fraction of CURRENT |e_psi| scales r_a_accel
    # toward EPSI_RA_ACCEL_BOOST_MAX (more expensive, so the MPC
    # doesn't keep accelerating through a heading error it should be
    # correcting) and r_a_brake toward EPSI_RA_BRAKE_FLOOR (cheaper,
    # so braking authority is freed up specifically when heading
    # error is large). Not a replacement for adaptive_R_scaling
    # (current-speed-driven R[0,0] scaling), which is left untouched.
    epsi_abs = abs(e_psi)
    epsi_half = max(EPSI_RA_HALF_RAD, 1e-6)
    frac_epsi = epsi_abs / (epsi_abs + epsi_half)
    r_a_accel_eff = R_A_ACCEL * (
        1.0 + (EPSI_RA_ACCEL_BOOST_MAX - 1.0) * frac_epsi)
    r_a_brake_eff = R_A_BRAKE * (
        1.0 - (1.0 - EPSI_RA_BRAKE_FLOOR) * frac_epsi)

    # ── MPC solve ─────────────────────────────────────────────────────
    mpc_result = solve_mpc(
        x0_mpc, Ad, Bd, n_horizon, Q_scaled, R_scaled, u_min, u_max,
        R_rate=R_rate_scaled, u_prev=u_prev, silent=True,
        return_status=True, eps_abs=eps, eps_rel=eps,
        max_iter=max_iter, warm_start=(step != 0),
        du_max=du_max, terminal_scale=TERMINAL_Q_SCALE,
        r_a_accel=r_a_accel_eff, r_a_brake=r_a_brake_eff,
    )

    solver_failed = mpc_result is None
    inaccurate = False
    if solver_failed:
        u_opt = u_prev.copy()
    else:
        u_opt, status = mpc_result
        inaccurate = status in (cp.OPTIMAL_INACCURATE, "optimal_inaccurate")
    return u_opt, solver_failed, inaccurate, x0_mpc, Ad, Bd


_NMPC_HISTORY_KEYS = (
    ("nmpc_iters", "iters"), ("nmpc_status", "status"), ("nmpc_cost", "cost"),
    ("nmpc_e_y", "e_y"), ("nmpc_e_psi", "e_psi"), ("nmpc_solve_ms", "solve_ms"),
)


def record_solve_history(history, u_opt, solver_failed, nmpc_diag):
    """Append this tick's command and solver diagnostics. `nmpc_diag` is
    None on an LTV-QP tick, and every nmpc_* series gets None for it."""
    history["solver_failed"].append(solver_failed)
    history["u_steer"].append(u_opt[0])
    history["u_accel"].append(u_opt[1])
    for hist_key, diag_key in _NMPC_HISTORY_KEYS:
        history[hist_key].append(None if nmpc_diag is None else nmpc_diag[diag_key])


def record_horizon_prediction(history, x0_mpc, Ad, Bd, u_opt, n_horizon,
                              X_g, Y_g, psi_g, v, rpsi):
    """
    GUI-only, cosmetic N-step prediction line; the plant never uses it.

    `x0_mpc` is None on an NMPC tick: no linear Ad/Bd model exists to build
    the line from, so an empty snapshot is recorded (same precedent as the
    "planner not ready" case) rather than a misleading LTV-based prediction.
    The NMPC predicts in Frenet coordinates, not global X/Y, so its own
    trajectory is not plotted here.
    """
    if x0_mpc is None:
        history["pred_X"].append(np.empty(0))
        history["pred_Y"].append(np.empty(0))
        return
    px, py = [], []
    x_p_tmp = x0_mpc.copy()
    for k in range(n_horizon):
        e_y_pred = x_p_tmp[0]
        px.append(X_g + (k + 1) * v * np.cos(psi_g) * DT - e_y_pred * np.sin(rpsi))
        py.append(Y_g + (k + 1) * v * np.sin(psi_g) * DT + e_y_pred * np.cos(rpsi))
        x_p_tmp = Ad @ x_p_tmp + Bd @ u_opt
    history["pred_X"].append(px)
    history["pred_Y"].append(py)


def compute_time_bonus(reached_end, sim_time, progress, optimal_time, dynamic_max_steps):
    """
    Time bonus for a finished run, 0.0 otherwise (identical formula for both
    rollout callers).

    Anchored to the path's PHYSICAL optimum where available. Dividing by a
    placeholder step budget (e.g. some multiple of arc_length /
    assumed_speed) has no physical meaning, can be several times the actual
    optimum, and varies by path, which would make TIME_BONUS_WEIGHT (0.25,
    the second-largest score term) a reward against an arbitrary constant
    and make the bonus non-comparable BETWEEN paths.

    optimal_lap_time() is a quasi-steady-state bound (corner limit ->
    forward accel pass -> backward brake pass -> integrate ds/v), so
    time_bonus is "how close to physically-fastest", in [0, 1] and directly
    comparable across paths. Because the bound ignores transient dynamics it
    is not quite attainable, so a real run scores below 1.0. Ratio form, NOT
    (1 - sim/optimal): sim_time is always >= optimal_time (it's a lower
    bound), so that subtraction would clip to 0 on every run and carry no
    information. optimal/sim is 1.0 at the physical limit and decays toward
    0 as the run gets slower, e.g. twice the optimal time scores 0.5.

    optimal_time covers the WHOLE path, but the rollout stops as soon as the
    car is within 3 m of the finish and past ~90% of the points, so it is
    only timed over `progress` of the distance. Comparing a partial-path run
    against a full-path reference makes the car look faster than physically
    possible and can saturate the bonus at exactly 1.000 for many runs,
    destroying all discrimination in the primary objective. The reference is
    scaled to the distance actually covered.
    """
    if not reached_end:
        return 0.0
    if optimal_time is not None and optimal_time > 0.0 and sim_time > 0.0:
        ref_time = optimal_time * max(progress, 1e-6)
        return float(np.clip(ref_time / sim_time, 0.0, 1.0))
    expected_time = dynamic_max_steps * DT
    return max(0.0, 1.0 - (sim_time / expected_time))
