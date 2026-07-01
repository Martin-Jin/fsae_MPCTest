"""
Performance Metric Reporter
File Name: tuner.py

All weight-optimisation logic has been removed — tuning is done exclusively
via offline_tuner.py (differential evolution on synthetic paths).

This module's only job is to score a completed simulation history dict using
the identical cost decomposition that offline_tuner.run_headless_rollout uses,
so results from the live simulator are directly comparable to offline scores.

Public API (called by simulation.py's "Show Metrics" button):
    report_performance_metrics(history, log_fn=print) -> metrics dict
"""

import numpy as np


def report_performance_metrics(history, log_fn=print):
    """
    Score a completed simulate_closed_loop() history dict and print a full
    breakdown to the console.

    Cost terms mirror offline_tuner.run_headless_rollout exactly:

        error_cost     = sum(e_y**2 + 0.5 * e_psi**2)
        yaw_rate_cost  = 0.8 * sum(r**2)          [r = plant_state[5]]
        control_smooth = sum(||u_k - u_{k-1}||^2)
        steering_effort= sum(u_steer**2)
        accel_effort   = sum(u_accel**2)
        max_steering   = max(|u_steer|)

        rmse           = sqrt(error_cost / N)
        yaw_rms        = sqrt(yaw_rate_cost / N)   (after the 0.8 factor)
        smooth_rms     = sqrt(control_smooth / N)
        steer_rms      = sqrt(steering_effort / N)
        accel_rms      = sqrt(accel_effort / N)

        composite = rmse
                  + 0.10 * yaw_rms
                  + 0.05 * smooth_rms
                  + 0.03 * steer_rms
                  + 0.01 * accel_rms
                  + 0.02 * max_steering
                  - 0.20 * completion_frac   (progress bonus)

    Parameters
    ----------
    history : dict
        Returned by simulate_closed_loop(). Must contain keys:
        'e_y', 'e_psi', 'v', 'u_steer', 'u_accel',
        'failed', 'completion_frac'.
        'v_target' is used for speed-error reporting only (optional).
    log_fn : callable
        Sink for the printed report. Defaults to print (console).

    Returns
    -------
    dict with keys matching the printed rows, for programmatic use.
    """
    e_y     = np.asarray(history.get("e_y",     []), dtype=float)
    e_psi   = np.asarray(history.get("e_psi",   []), dtype=float)
    v       = np.asarray(history.get("v",       []), dtype=float)
    u_steer = np.asarray(history.get("u_steer", []), dtype=float)
    u_accel = np.asarray(history.get("u_accel", []), dtype=float)

    n = max(len(e_y), 1)
    failed          = bool(history.get("failed", False))
    completion_frac = float(history.get("completion_frac", 1.0))

    # ------------------------------------------------------------------
    # Replicate offline_tuner accumulation exactly
    # ------------------------------------------------------------------
    error_cost = float(np.sum(e_y**2 + 0.5 * e_psi**2))

    # Yaw-rate: offline_tuner uses state[5] (plant yaw rate r). The live
    # simulator doesn't store r separately, but e_psi_dot ≈ r in small-
    # angle tracking, and v * sin(e_psi) / wheelbase is also an option.
    # The closest available proxy in history is the rate of change of
    # e_psi between steps (same dt=0.05 s as the plant).
    if len(e_psi) > 1:
        r_proxy = np.diff(e_psi) / 0.05          # rad/s
        yaw_rate_cost = 0.8 * float(np.sum(r_proxy**2))
        yaw_rms = float(np.sqrt(yaw_rate_cost / n))
    else:
        yaw_rate_cost = 0.0
        yaw_rms = 0.0

    # Control smoothness: successive-difference norm, same as offline_tuner
    if len(u_steer) > 1 and len(u_accel) > 1:
        du_steer = np.diff(u_steer)
        du_accel = np.diff(u_accel)
        control_smooth = float(np.sum(du_steer**2 + du_accel**2))
    else:
        control_smooth = 0.0

    steering_effort = float(np.sum(u_steer**2))
    accel_effort    = float(np.sum(u_accel**2))
    max_steering    = float(np.max(np.abs(u_steer))) if len(u_steer) else 0.0

    # RMS forms (divided by N, matching offline_tuner's per-step accumulation)
    rmse       = float(np.sqrt(error_cost    / n))
    smooth_rms = float(np.sqrt(control_smooth / n))
    steer_rms  = float(np.sqrt(steering_effort / n))
    accel_rms  = float(np.sqrt(accel_effort    / n))

    # Decomposed tracking RMSEs (for readability)
    lateral_rmse = float(np.sqrt(np.mean(e_y**2)))   if len(e_y)   else 0.0
    heading_rmse = float(np.sqrt(np.mean(e_psi**2))) if len(e_psi) else 0.0

    # Speed tracking (informational only — not in offline_tuner composite)
    v_target = np.asarray(history.get("v_target", []), dtype=float)
    if len(v_target) == len(v) and len(v) > 0:
        speed_rmse = float(np.sqrt(np.mean((v - v_target)**2)))
    else:
        speed_rmse = float("nan")

    # Composite score — identical formula to offline_tuner
    composite = (
          rmse
        + 0.10 * yaw_rms
        + 0.05 * smooth_rms
        + 0.03 * steer_rms
        + 0.01 * accel_rms
        + 0.02 * max_steering
        - 0.20 * completion_frac
    )

    # ------------------------------------------------------------------
    # Console report
    # ------------------------------------------------------------------
    status_str = "FAILED / OFF-TRACK" if failed else "completed"
    log_fn("=" * 58)
    log_fn(f"[Performance] Rollout {status_str} "
           f"({completion_frac * 100:.1f}% of path, {n} steps)")
    log_fn("-" * 58)
    log_fn(f"  Composite score    : {composite:8.4f}  "
           f"(offline_tuner compatible — lower is better)")
    log_fn("-" * 58)
    log_fn(f"  Lateral RMSE       : {lateral_rmse:8.4f} m")
    log_fn(f"  Heading RMSE       : {np.degrees(heading_rmse):8.4f} deg")
    log_fn(f"  Speed RMSE         : "
           + (f"{speed_rmse:8.4f} m/s" if not np.isnan(speed_rmse) else "     n/a"))
    log_fn("-" * 58)
    log_fn(f"  Yaw-rate RMS       : {yaw_rms:8.4f} rad/s  (x0.10 in composite)")
    log_fn(f"  Control smooth RMS : {smooth_rms:8.4f}        (x0.05)")
    log_fn(f"  Steering RMS       : {np.degrees(steer_rms):8.4f} deg    (x0.03)")
    log_fn(f"  Accel RMS          : {accel_rms:8.4f} m/s²   (x0.01)")
    log_fn(f"  Max steering cmd   : {np.degrees(max_steering):8.4f} deg    (x0.02)")
    log_fn(f"  Path completion    : {completion_frac * 100:8.1f} %      (-0.20 bonus)")
    log_fn("=" * 58)

    return {
        "composite_score":    composite,
        "lateral_rmse_m":     lateral_rmse,
        "heading_rmse_deg":   np.degrees(heading_rmse),
        "speed_rmse_mps":     speed_rmse,
        "yaw_rms_radps":      yaw_rms,
        "control_smooth_rms": smooth_rms,
        "steering_rms_deg":   np.degrees(steer_rms),
        "accel_rms_mps2":     accel_rms,
        "max_steering_deg":   np.degrees(max_steering),
        "completion_pct":     completion_frac * 100.0,
        "failed":             failed,
        "n_steps":            n,
    }