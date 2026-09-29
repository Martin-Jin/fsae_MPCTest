"""
sim/rollout/tick_solve.py — the controller-solve phase of run_core_rollout(),
lifted out of its loop so the loop itself reads as a sequence of named
steps.

Stateless: any value the loop carries from one tick to the next
(u_prev, command_queue, ...) is passed in and returned explicitly rather
than stored, so the loop in sim/rollout/core.py remains the one place that
owns rollout state.

settings.py constants are bound here by name at import time, the same as
elsewhere in this package. Callers that override settings
(tuner/investigations/steering_chatter_check.py) must do so before the
first import of sim.rollout.core, which imports this module.
"""

import numpy as np
import cvxpy as cp

from controller.lmpc import solve_mpc
from controller.nmpc import NMPCController
from controller.model_utils import (
    curvature_estimate, adaptive_R_scaling, adaptive_Q_scaling,
    steer_rate_anti_hunt, reversal_penalty_boost, _corner_factor, _blend,
    _low_speed_corner_boost,
)
from sim.rollout.delay import predict_ahead

import settings

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
        dt=settings.DT, N=settings.NMPC_HORIZON, vehicle_params=vehicle_params,
        u_min=u_min, u_max=u_max, du_max=du_max,
        q_e_y=_nmpc_pick(settings.NMPC_Q_E_Y, Q[0, 0]),
        q_e_yd=_nmpc_pick(settings.NMPC_Q_E_YD, Q[1, 1]),
        q_e_psi=_nmpc_pick(settings.NMPC_Q_E_PSI, Q[2, 2]),
        q_epsi_dot=_nmpc_pick(settings.NMPC_Q_EPSI_DOT, Q[3, 3]),
        q_e_v=_nmpc_pick(settings.NMPC_Q_E_V, Q[4, 4]),
        r_delta=_nmpc_pick(settings.NMPC_R_DELTA, R[0, 0]),
        r_a_accel=_nmpc_pick(settings.NMPC_R_A_ACCEL, settings.R_A_ACCEL),
        r_a_brake=_nmpc_pick(settings.NMPC_R_A_BRAKE, settings.R_A_BRAKE),
        r_rate_delta=_nmpc_pick(settings.NMPC_R_RATE_DELTA, R_rate[0, 0]),
        r_rate_a=_nmpc_pick(settings.NMPC_R_RATE_A, R_rate[1, 1]),
        terminal_scale=_nmpc_pick(settings.NMPC_TERMINAL_SCALE, settings.TERMINAL_Q_SCALE),
        sqp_iters=settings.NMPC_SQP_ITERS, solve_budget_ms=settings.NMPC_SOLVE_BUDGET_MS,
        rk_substeps=settings.NMPC_RK_SUBSTEPS, jac_substeps=settings.NMPC_JAC_SUBSTEPS,
        jac_gate_speed=settings.NMPC_JAC_GATE_SPEED, jac_substeps_fast=settings.NMPC_JAC_SUBSTEPS_FAST,
        rk_gate_speed=settings.NMPC_RK_GATE_SPEED, rk_substeps_fast=settings.NMPC_RK_SUBSTEPS_FAST,
        standstill_steer_damp_enabled=settings.NMPC_STANDSTILL_STEER_DAMP_ENABLED,
        standstill_speed=settings.NMPC_STANDSTILL_SPEED,
        standstill_fade_speed=settings.NMPC_STANDSTILL_FADE_SPEED,
        standstill_steer_r_scale=settings.NMPC_STANDSTILL_STEER_R_SCALE,
        trust_delta_rad=settings.NMPC_TRUST_DELTA_RAD, trust_a=settings.NMPC_TRUST_A,
        backtrack_max=settings.NMPC_BACKTRACK_MAX,
        track_halfwidth=settings.NMPC_TRACK_HALFWIDTH, slack_weight=settings.NMPC_SLACK_WEIGHT,
        slack_linear_weight=_ov('slack_linear_weight', settings.NMPC_SLACK_LINEAR_WEIGHT),
        osqp_max_iter=settings.NMPC_OSQP_MAX_ITER, osqp_eps=settings.NMPC_OSQP_EPS,
        alat_ceiling_enabled=settings.NMPC_ALAT_CEILING_ENABLED,
        alat_flat=settings.ALAT_CEILING_FLAT, alat_slope=settings.ALAT_CEILING_SLOPE,
        alat_intercept=settings.ALAT_CEILING_INTERCEPT,
        spline_reference_enabled=settings.NMPC_SPLINE_REFERENCE_ENABLED,
        friction_circle_enabled=settings.NMPC_FRICTION_CIRCLE_ENABLED,
        steer_rate_anti_hunt_enabled=settings.NMPC_STEER_RATE_ANTI_HUNT_ENABLED,
        corner_rrate_blend_enabled=settings.NMPC_CORNER_RRATE_BLEND_ENABLED,
        corner_factor_k=_ov('corner_factor_k',
                            _nmpc_pick(settings.NMPC_CORNER_FACTOR_K, settings.CORNER_FACTOR_K)),
        rrate_steer_straight=_nmpc_pick(settings.NMPC_RRATE_STEER_STRAIGHT, settings.RRATE_STEER_STRAIGHT),
        rrate_steer_corner=_nmpc_pick(settings.NMPC_RRATE_STEER_CORNER, settings.RRATE_STEER_CORNER),
        reversal_penalty_enabled=settings.NMPC_REVERSAL_PENALTY_ENABLED,
        reversal_penalty_boost_max=_nmpc_pick(
            settings.NMPC_REVERSAL_PENALTY_BOOST_MAX, settings.REVERSAL_PENALTY_BOOST_MAX),
        reversal_penalty_k=_nmpc_pick(settings.NMPC_REVERSAL_PENALTY_K, settings.REVERSAL_PENALTY_K),
        rrate_stage_ramp_enabled=_ov('rrate_stage_ramp_enabled', settings.NMPC_RRATE_STAGE_RAMP_ENABLED),
        rrate_stage_near=_ov('rrate_stage_near', settings.NMPC_RRATE_STAGE_NEAR),
        rrate_zone_enabled=_ov('rrate_zone_enabled', settings.NMPC_RRATE_ZONE_ENABLED),
        rrate_zone_boost_straight=_ov('rrate_zone_boost_straight', settings.NMPC_RRATE_ZONE_BOOST_STRAIGHT),
        rrate_zone_ease_approach=_ov('rrate_zone_ease_approach', settings.NMPC_RRATE_ZONE_EASE_APPROACH),
        rrate_zone_floor_corner=_ov('rrate_zone_floor_corner', settings.NMPC_RRATE_ZONE_FLOOR_CORNER),
        rjerk_delta=_ov('rjerk_delta', settings.NMPC_RJERK_DELTA),
        rjerk_a=_ov('rjerk_a', settings.NMPC_RJERK_A),
        latency_compensation_enabled=settings.NMPC_LATENCY_COMPENSATION_ENABLED,
        latency_compensation_ms=settings.NMPC_LATENCY_COMPENSATION_MS,
        kappa_rate_max=settings.NMPC_KAPPA_RATE_MAX,
        progress_enabled=_ov('progress_enabled', settings.NMPC_PROGRESS_ENABLED),
        q_progress=_ov('q_progress', settings.NMPC_Q_PROGRESS),
        progress_reach=_ov('progress_reach', settings.NMPC_PROGRESS_REACH),
        progress_v_min=_ov('progress_v_min', settings.NMPC_PROGRESS_V_MIN),
    )


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
        dense_step=settings.NMPC_CURVATURE_DENSE_STEP,
        smooth_w=settings.NMPC_CURVATURE_SMOOTH_W, kappa_clip=settings.NMPC_KAPPA_CLIP,
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
    corner_factor = _corner_factor(kappa, settings.CORNER_FACTOR_K)

    # Extra push in the SAME direction as corner_factor's "full
    # corner" endpoint, active only when BOTH corner_factor > 0 AND
    # speed is low -- gated on corner_factor (multiplicatively) so
    # this cannot fire on low speed alone with no corner, unlike the
    # deleted low_speed_steer_rate_boost (which fired on speed alone
    # and ended up taxing wanted low-speed turn-in indistinguishably
    # from unwanted post-exit wobble).
    low_speed_boost = _low_speed_corner_boost(
        vx_true, corner_factor,
        v_half=settings.LOW_SPEED_CORNER_BOOST_V_HALF,
        max_extra=settings.LOW_SPEED_CORNER_BOOST_MAX_EXTRA,
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
        kappa, e_y, R_rate_scaled, enabled=settings.STEER_RATE_ANTI_HUNT_ENABLED, e_psi=e_psi,
    )
    m_rrate_antihunt = (
        float(R_rate_scaled[0, 0] / _rr_before_hunt) if _rr_before_hunt else 1.0
    )
    _rr_before_reversal = float(R_rate_scaled[0, 0])
    R_rate_scaled = reversal_penalty_boost(
        float(u_prev[0]), R_rate_scaled, enabled=settings.REVERSAL_PENALTY_ENABLED,
        boost_max=settings.REVERSAL_PENALTY_BOOST_MAX, k=settings.REVERSAL_PENALTY_K,
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
    Q_base[0, 0] = _blend(settings.Q_EY_STRAIGHT, settings.Q_EY_CORNER, corner_frac)
    Q_base[2, 2] = _blend(settings.Q_EPSI_STRAIGHT, settings.Q_EPSI_CORNER, corner_frac)
    Q_base[3, 3] = _blend(settings.Q_R_STRAIGHT, settings.Q_R_CORNER, corner_frac)

    # CAUTION: this line sets R_rate_scaled[0,0]'s BASE value, so
    # every multiplier computed above (m_rrate_antihunt,
    # m_rrate_reversal, and any future one) must be explicitly
    # reapplied here too -- an assignment that omits one silently
    # discards its effect even though the multiplier's own value is
    # still correctly logged elsewhere. See mpc_core.py's matching
    # comment; this exact class of bug has recurred more than once.
    R_rate_scaled = R_rate_scaled.copy()
    R_rate_scaled[0, 0] = _blend(
        settings.RRATE_STEER_STRAIGHT, settings.RRATE_STEER_CORNER, corner_frac
    ) * m_rrate_antihunt * m_rrate_reversal

    R_scaled = R_scaled.copy()
    R_scaled[0, 0] = _blend(R_scaled[0, 0], settings.R_STEER_CORNER_MID, corner_frac)

    Q_scaled = adaptive_Q_scaling(e_y, Q_base, enabled=settings.ADAPTIVE_Q_SCALING_ENABLED)
    Ad, Bd = model_lookup(vx, settings.DT)

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
    epsi_half = max(settings.EPSI_RA_HALF_RAD, 1e-6)
    frac_epsi = epsi_abs / (epsi_abs + epsi_half)
    r_a_accel_eff = settings.R_A_ACCEL * (
        1.0 + (settings.EPSI_RA_ACCEL_BOOST_MAX - 1.0) * frac_epsi)
    r_a_brake_eff = settings.R_A_BRAKE * (
        1.0 - (1.0 - settings.EPSI_RA_BRAKE_FLOOR) * frac_epsi)

    # ── MPC solve ─────────────────────────────────────────────────────
    mpc_result = solve_mpc(
        x0_mpc, Ad, Bd, n_horizon, Q_scaled, R_scaled, u_min, u_max,
        R_rate=R_rate_scaled, u_prev=u_prev, silent=True,
        return_status=True, eps_abs=eps, eps_rel=eps,
        max_iter=max_iter, warm_start=(step != 0),
        du_max=du_max, terminal_scale=settings.TERMINAL_Q_SCALE,
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
        px.append(X_g + (k + 1) * v * np.cos(psi_g) * settings.DT - e_y_pred * np.sin(rpsi))
        py.append(Y_g + (k + 1) * v * np.sin(psi_g) * settings.DT + e_y_pred * np.cos(rpsi))
        x_p_tmp = Ad @ x_p_tmp + Bd @ u_opt
    history["pred_X"].append(px)
    history["pred_Y"].append(py)
