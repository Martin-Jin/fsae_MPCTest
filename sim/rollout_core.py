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

LAYOUT
------
  sim/rollout_core.py    this file: rollout state, the tick loop, termination
  sim/rollout_phases.py  the loop's per-tick phases (reference + speed target,
                         LTV/NMPC solves, history recording, time bonus)
  sim/sensor_noise.py    SLAM noise, cone noise, pose-feed hold models
"""

import math
import numpy as np
from collections import deque

from model.vehicle_physics import (
    step_nonlinear_plant, init_plant_state, find_closest_reference_bounded,
)
from sim.perception import SimPerception
from sim.planner import SimPlanner, calculate_dynamic_max_steps
from sim.scoring import RolloutMetrics

from settings import (
    USE_PLANNER, DELAY_STEPS, OFFTRACK_LIMIT, MAX_FAILS, DT,
    ROLLOUT_EPS, ROLLOUT_MAX_ITER, N_HORIZON, DELAY_JITTER_SEED,
    SLAM_NOISE_ENABLED, SLAM_POS_JITTER_STD, SLAM_YAW_JITTER_STD,
    SLAM_POS_DRIFT_STD, SLAM_YAW_DRIFT_STD, SLAM_DRIFT_TAU, SLAM_NOISE_SEED,
    POSE_HOLD_ENABLED, POSE_HOLD_PROB, POSE_HOLD_MEAN_TICKS,
    POSE_HOLD_MAX_TICKS, POSE_HOLD_SEED,
    CONE_NOISE_ENABLED, CONE_POS_JITTER_STD, CONE_NOISE_SEED,
    USE_NMPC,
)
from sim.rollout_phases import (  # noqa: F401 (predict_ahead is public API)
    _normalize_angle, predict_ahead, build_nmpc, compute_reference,
    compute_speed_target, gate_and_rate_limit_speed_target, true_tracking_error,
    believed_pending_cmds, solve_nmpc_tick, solve_ltv_tick,
    record_solve_history, record_horizon_prediction, compute_time_bonus,
)
from sim.sensor_noise import SlamNoise, ConeNoise, PoseFeedHold


# Conservative defaults, not independently measured/tuned: 3 s / 3 m is slow
# even for a car recovering from a bad line, so this only catches a genuine
# stall (stuck oscillating, not actually progressing), not normal driving.
STALL_CHECK_INTERVAL = 60   # Steps between rolling stall checks (3 s at 20 Hz)
STALL_MIN_DISTANCE = 3.0    # Minimum distance (m) expected per interval


def compute_step_budget(path_X, path_Y, path_v_profile):
    """
    Single source of truth for the dynamic step budget, so both callers stay
    consistent — arc-length/fallback-speed estimate vs. a speed-profile-aware
    estimate, taking the larger of the two.

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
    use_nmpc=USE_NMPC,
    nmpc_overrides=None,
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
        applies adaptive_R_scaling internally each step).
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
    use_nmpc : bool
        False (default, = settings.USE_NMPC) -> controller/lmpc/solve.py's
        linear time-varying QP, as always. True -> controller/nmpc/'s
        Frenet-frame nonlinear MPC instead -- see that package's
        docstring. Explicit parameter (not read from settings.py
        at call time) so a caller (e.g. an A/B script) can toggle it without
        relying on module-attribute mutation after settings.py has already
        been imported elsewhere, which has no effect on an already-bound
        `from settings import USE_NMPC` name -- see tuner/
        nmpc_offline_check.py's closed-loop test for exactly this gotcha,
        found by testing this parameter's own first version.
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
    nmpc_overrides : dict, optional
        Per-call overrides for the NMPC rate-shaping fields, by the controller's
        own kwarg name (e.g. {'rjerk_delta': 250.0, 'rrate_zone_ease_approach':
        0.35}). Anything absent falls back to the settings.py constant.

        This exists because settings.py's constants are imported into this
        module BY NAME at import time, so a caller that mutates
        settings.NMPC_* after this module is imported has no effect -- the
        usual workaround is a fresh subprocess per configuration. Passing a
        dict here lets one process evaluate many configurations, which is what
        an optimiser sweeping these fields needs. Keys are not validated
        against the controller's signature; a typo is silently ignored, so
        check a swept field actually moves before trusting a null result.

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

    perception = planner = None
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
    # a jittering loop period, so it is regularly wrong by a step; this jitter
    # is modelled offline too, so predict_ahead() cannot look more effective
    # in the tuner than it can ever be on the car. See believed_pending_cmds.
    # Seeded so each rollout is reproducible and CMA-ES still gets a stable
    # score per candidate (see settings.DELAY_JITTER_SEED).
    delay_rng = np.random.default_rng(DELAY_JITTER_SEED)

    # Stacked (N,2) path array, built once (not per-step), for the NMPC's
    # oracle-path reference.
    path_xy = np.column_stack([path_X, path_Y])

    # Values carried from one tick to the next by the per-tick phases, each
    # None until its first tick:
    #   v_des_prev    speed target, for SPEED_TARGET_RISE_RATE
    #   gate_prev     tracking-error speed gate, for GATE_RATE_LIMIT
    #   v_curv_prev   live curvature_speed() output, for V_CURV_FALL_RATE
    #                 (live-planner, non-precomputed-profile branch only)
    #   ref_psi_prev  LIMITED reference heading, for REF_HEADING_RATE_LIMIT;
    #                 unwrapped/continuous so consecutive limiting steps
    #                 compose correctly across the wrap boundary
    v_des_prev = gate_prev = v_curv_prev = ref_psi_prev = None

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

    nmpc = None
    if use_nmpc:
        nmpc = build_nmpc(Q, R, R_rate, u_min, u_max, du_max, vehicle_params, nmpc_overrides)

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
            # NMPC-only diagnostics (see controller/nmpc/solver.py's
            # compute_step() diag dict) -- always present, None on every
            # LTV-QP-controlled step, exactly like "planner_X"/"planner_Y"
            # are only meaningful in planner mode. Distinct from "e_y"/
            # "e_psi" above, which stay the SAME tracking-error-helper value
            # regardless of controller so existing plots/scoring keep
            # meaning what they always have; these are the NMPC's OWN
            # Frenet e_y/e_psi and solver state.
            "nmpc_iters": [], "nmpc_status": [], "nmpc_cost": [],
            "nmpc_e_y": [], "nmpc_e_psi": [], "nmpc_solve_ms": [],
        }
        if use_planner:
            # Per-step snapshot of SimPlanner's live centreline (distinct from
            # the true reference path_X/path_Y) — GUI-only, cosmetic, for
            # visualising what the planner actually built vs. the ground truth.
            history["planner_X"] = []
            history["planner_Y"] = []

    n_ran = max_steps
    # Step at which the car first exceeds LAUNCH_SPEED_MPS, so sim_time (and
    # therefore time_bonus) is measured from LAUNCH rather than from tick 0.
    # Mirrors telemetry_logger.LapProgressTracker.LAUNCH_SPEED_MPS on the live
    # side -- keep the two equal. Without this the standstill before the car
    # gets moving is folded into the lap time, deflating time_bonus and making
    # runs with different pre-launch holds non-comparable.
    LAUNCH_SPEED_MPS = 0.5
    launch_step = None

    for step in range(max_steps):
        X_g, Y_g, psi_g = state[0], state[1], state[2]

        # ── Estimated (SLAM) pose vs true pose ────────────────────────────
        # Everything the controller and planner consume below uses the
        # ESTIMATED pose; the plant integration and the score keep using the
        # true `state`. With SLAM noise disabled the two are identical.
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

        # ── Tracking error + speed target ─────────────────────────────────
        e_y, e_psi, rpsi, planner_cl, ref_psi_prev, cl_idx = compute_reference(
            use_planner, perception, planner, cone_noise, pose_age_ticks,
            state, state_est, X_est, Y_est, psi_est, car_pos_np,
            path_X, path_Y, path_Psi, ref_psi_prev, history,
        )
        v_target, v_curv_prev = compute_speed_target(
            planner_cl, cl_idx, car_pos_np, path_v_profile, idx, v_curv_prev,
        )
        v_target, gate_prev, v_des_prev = gate_and_rate_limit_speed_target(
            v_target, e_y, e_psi, state[3], gate_prev, v_des_prev,
        )
        e_y_true, e_psi_true = true_tracking_error(
            state, e_y, e_psi, path_X, path_Y, path_Psi,
            diverged=(slam_noise is not None or use_planner),
        )

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
        if launch_step is None and abs(vx_true) >= LAUNCH_SPEED_MPS:
            launch_step = step
        vx = max(vx_true, 0.5)
        e_y_dot = vx_true * np.sin(e_psi) + state[4] * np.cos(e_psi)
        x0_mpc = np.array([
            e_y, e_y_dot, e_psi, state[5], vx_true - v_target, 0.0, state[6], state[7],
        ])

        # ── Solve ─────────────────────────────────────────────────────────
        pending_cmds = believed_pending_cmds(command_queue, delay_rng, u_prev)
        nmpc_diag = None
        if use_nmpc:
            u_opt, nmpc_diag = solve_nmpc_tick(
                nmpc, planner_cl, path_xy, car_pos_np, psi_est, state_est,
                vx_true, v_target, pending_cmds, step,
            )
            solver_failed = False
            inaccurate = False
            x0_mpc = Ad = Bd = None   # no linear model -- see record_horizon_prediction
        else:
            u_opt, solver_failed, inaccurate, x0_mpc, Ad, Bd = solve_ltv_tick(
                state, x0_mpc, e_y, e_psi, vx, vx_true, u_prev, pending_cmds,
                Q, R, R_rate, u_min, u_max, du_max, model_lookup,
                n_horizon, eps, max_iter, step,
            )
            consecutive_fails = consecutive_fails + 1 if solver_failed else 0
            if inaccurate:
                inaccurate_count_total += 1

        if want_history:
            record_solve_history(history, u_opt, solver_failed, nmpc_diag)

        # ── Apply transport delay ────────────────────────────────────────────
        command_queue.append(u_opt)
        delayed_u_cmd = command_queue[0]

        if want_history and want_horizon_pred:
            record_horizon_prediction(
                history, x0_mpc, Ad, Bd, u_opt, n_horizon,
                X_g, Y_g, psi_g, state[3], rpsi,
            )

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
        # 10% window / 3 m radius: conservative defaults, not independently
        # measured — wide enough to reliably catch idx lagging the true
        # finish, narrow enough not to false-trigger on a lap's own
        # start/finish straight or figure-eight crossing (see above).
        # Mirrored in telemetry_logger.py's LapProgressTracker.
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
        # Scored on the TRUE error (see true_tracking_error), not the
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

    # Timed from LAUNCH, not from tick 0 -- see launch_step's comment above.
    # Falls back to the full count if the car never moved, so a stalled run
    # still reports the whole elapsed time rather than 0.
    sim_time = (n_ran - (launch_step or 0)) * DT
    time_bonus = compute_time_bonus(
        reached_end, sim_time, progress, optimal_time, dynamic_max_steps,
    )

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
