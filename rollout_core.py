"""
rollout_core.py — Single Source of Truth for the MPC Closed-Loop Rollout

PURPOSE
-------
offline_tuner.run_headless_rollout() and simulation.simulate_closed_loop() used
to independently reimplement the exact same per-step logic: tracking-error
computation, progress tracking, MPC solve with adaptive gains, delay queue,
termination checks, and metric accumulation. Any tweak to one silently drifted
from the other — which is exactly why offline-tuner scores and the live
simulator's "Show Metrics" scores stopped matching (e.g. the planner-fallback
branch existed in offline_tuner but was missing in simulation.py).

This module is now the ONLY place that runs the actual rollout loop. Both
offline_tuner.py and simulation.py call run_core_rollout() and only differ in
what they do with the result:
  - offline_tuner.py:  want_history=False → just the composite score
  - simulation.py:      want_history=True  → full step history for the GUI

WHY NOT IN simulation.py
-------------------------
simulation.py builds a matplotlib GUI at import time. offline_tuner.py runs
rollouts inside multiprocessing worker processes — importing simulation.py
there would try to open a GUI window in every worker. This module imports
nothing GUI-related, so it's safe to import from anywhere.
"""

import math
import numpy as np
from collections import deque

from vehicle_physics import (
    step_nonlinear_plant, init_plant_state, plant_to_tracking_error,
    find_closest_reference_bounded,
)
from optimiser import solve_mpc
from sim_track import SimPerception, SimPlanner, calculate_dynamic_max_steps
from model_utils import curvature_estimate, adaptive_R_rate, adaptive_R_scaling
from scoring import RolloutMetrics
import cvxpy as cp

from settings import (
    USE_PLANNER, DELAY_STEPS, OFFTRACK_LIMIT, MAX_FAILS, DT,
    ROLLOUT_EPS, ROLLOUT_MAX_ITER, N_HORIZON,
)

STALL_CHECK_INTERVAL = 60   # Steps between rolling stall checks (3 s at 20 Hz)
STALL_MIN_DISTANCE = 3.0    # Minimum distance (m) expected per interval


def _normalize_angle(angle):
    """Wrap an angle to (−π, π] using atan2."""
    return np.arctan2(np.sin(angle), np.cos(angle))


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
):
    """
    Run one closed-loop MPC rollout: nonlinear plant + MPC controller.

    THE single implementation of the rollout loop, shared by
    offline_tuner.run_headless_rollout() and simulation.simulate_closed_loop().

    Parameters
    ----------
    path_X, path_Y, path_Psi, path_v_profile : arrays
        Reference path geometry and speed profile.
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
        Initial heading offset in **radians**. (simulation.py's slider is in
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

    Returns
    -------
    dict with keys:
        "composite_score" : float — final score (see scoring.py)
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

    if use_planner:
        perception = SimPerception(blue_cones, yellow_cones)
        planner = SimPlanner(v_max=20.0, v_min=1.5)
        _b0, _y0 = perception.visible_cones(float(X0), float(Y0), float(psi0))
        planner.update(_b0, _y0, np.array([X0, Y0]), float(psi0))

    command_queue = deque([np.zeros(2) for _ in range(DELAY_STEPS + 1)], maxlen=DELAY_STEPS + 1)
    u_prev = np.zeros(2)

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
            "pred_X": [], "pred_Y": [], "solver_failed": [],
            "failed": False, "offtrack": False, "fail_reason": None,
        }

    n_ran = max_steps

    for step in range(max_steps):
        car_pos_np = np.array([state[0], state[1]])
        X_g, Y_g, psi_g = state[0], state[1], state[2]

        if want_history:
            history["X"].append(X_g)
            history["Y"].append(Y_g)
            history["psi"].append(psi_g)
            history["v"].append(state[3])
            history["r"].append(state[5])

        # ── Tracking error + speed target (planner or oracle) ─────────────────
        rpsi = None
        if use_planner:
            b_vis, y_vis = perception.visible_cones(state[0], state[1], state[2])
            planner.update(b_vis, y_vis, car_pos_np, state[2])

            cl = planner.centreline
            if cl is not None and len(cl) >= 2:
                cl_x, cl_y = cl[:, 0], cl[:, 1]
                cl_psi = np.zeros_like(cl_x)
                cl_psi[:-1] = np.arctan2(np.diff(cl_y), np.diff(cl_x))
                cl_psi[-1] = cl_psi[-2] if len(cl_psi) > 1 else state[2]

                e_y, _, e_psi, _, _, _, _ = plant_to_tracking_error(
                    state, path_x=cl_x, path_y=cl_y, path_psi=cl_psi
                )
                rpsi = psi_g - e_psi

                dists = np.linalg.norm(cl - car_pos_np, axis=1)
                cl_idx = int(np.argmin(dists))
                v_target = (
                    float(np.interp(
                        float(cl_idx), np.arange(len(planner.v_profile)), planner.v_profile,
                    ))
                    if len(planner.v_profile) > 0
                    else float(path_v_profile[idx])
                )
            else:
                # Planner not yet ready — fall back to the global reference path.
                # (This fallback was previously missing in simulation.py, which
                # meant e_y/e_psi/v_target silently reused stale values from
                # the previous step whenever the planner wasn't ready.)
                e_y, _, e_psi, _, _, _, _ = plant_to_tracking_error(
                    state, path_x=path_X, path_y=path_Y, path_psi=path_Psi
                )
                v_target = float(path_v_profile[idx])
        else:
            e_y, _, e_psi, _, _, _, _ = plant_to_tracking_error(
                state, path_x=path_X, path_y=path_Y, path_psi=path_Psi
            )
            v_target = float(path_v_profile[idx])

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
            history["e_y"].append(e_y)
            history["e_psi"].append(e_psi)

        # ── MPC state vector ────────────────────────────────────────────────
        vx_true = state[3]
        vx = max(vx_true, 0.5)
        e_y_dot = vx_true * np.sin(e_psi) + state[4] * np.cos(e_psi)
        x0_mpc = np.array([
            e_y, e_y_dot, e_psi, state[5], vx_true - v_target, 0.0, state[6], state[7],
        ])

        # ── Adaptive gain scaling ────────────────────────────────────────────
        kappa = curvature_estimate(state)
        R_rate_scaled = adaptive_R_rate(kappa, R_rate)
        R_scaled = adaptive_R_scaling(vx, R)
        Ad, Bd = model_lookup(vx, DT)

        # ── MPC solve ─────────────────────────────────────────────────────────
        mpc_result = solve_mpc(
            x0_mpc, Ad, Bd, n_horizon, Q, R_scaled, u_min, u_max,
            R_rate=R_rate_scaled, u_prev=u_prev, silent=True,
            return_status=True, eps_abs=eps, eps_rel=eps,
            max_iter=max_iter, warm_start=(step != 0),
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
        dist_to_finish = math.hypot(state[0] - path_X[-1], state[1] - path_Y[-1])
        if idx >= len(path_X) - 2 or dist_to_finish <= 3.0:
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
                dnf = True
                n_ran = step + 1
                if want_history:
                    history["failed"] = True
                    history["fail_reason"] = (
                        f"stalled (< {STALL_MIN_DISTANCE} m in {STALL_CHECK_INTERVAL} steps) at step {step}"
                    )
                break
            dist_at_last_stall_check = cumulative_distance

        # ── Metric accumulation (single source of truth: scoring.RolloutMetrics) ──
        metrics.add_step(
            e_y=e_y, e_psi=e_psi, r=state[5], u_opt=u_opt,
            v_target=v_target, v_actual=state[3], u_max_steer=u_max[0],
            solver_failed=solver_failed, inaccurate=inaccurate,
        )

        if abs(e_y) > OFFTRACK_LIMIT:
            offtrack = True
            dnf = True
            n_ran = step + 1
            if want_history:
                history["failed"] = True
                history["offtrack"] = True
                history["fail_reason"] = f"off-track (|e_y|={abs(e_y):.2f} m) at step {step}"
            break

        u_prev = u_opt.copy()
        state = step_nonlinear_plant(state, delayed_u_cmd, DT, vehicle_params)

    # ── Completion / time bonus (identical formula for both callers) ──────────
    progress = cumulative_distance / path_length if path_length > 0 else 0.0
    progress = float(np.clip(progress, 0.0, 1.0))

    if reached_end:
        sim_time = n_ran * DT
        expected_time = dynamic_max_steps * DT
        time_bonus = max(0.0, 1.0 - (sim_time / expected_time))
    else:
        time_bonus = 0.0

    metrics_result = metrics.finalize(
        progress=progress, time_bonus=time_bonus, dnf=dnf, offtrack=offtrack,
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
            # matching the previous simulation.py behaviour.
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