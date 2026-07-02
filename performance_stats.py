"""
Performance Metric Reporter
Scores a completed simulation history dict using the identical cost
decomposition as offline_tuner.run_headless_rollout, so live simulator
results are directly comparable to offline scores.

Composite weights are imported directly from offline_tuner.SCORE_WEIGHTS
and offline_tuner.COMPLETION_BONUS_WEIGHT / TIME_BONUS_WEIGHT so there is
a single source of truth — editing offline_tuner constants automatically
updates this report.

Public API (called by simulation.py's "Show Metrics" button):
    report_performance_metrics(history, log_fn=print) -> metrics dict

NOTE on yaw_rms: the offline tuner uses the plant's actual yaw rate
state[5] accumulated during simulation. This module approximates it from
diff(e_psi)/dt, which is a proxy — expect small numerical differences on
this term only. All other terms are exact replicas.
"""

import numpy as np
from offline_tuner import (
    SCORE_WEIGHTS,
    COMPLETION_BONUS_WEIGHT,
    TIME_BONUS_WEIGHT,
    DNF_PENALTY,
)

# Metric index constants — must stay in sync with offline_tuner.SCORE_WEIGHTS
_IDX_RMSE               = 0
_IDX_YAW_RMS            = 1
_IDX_SMOOTH_RMS         = 2
_IDX_STEER_RMS          = 3
_IDX_ACCEL_RMS          = 4
_IDX_MAX_STEERING       = 5
_IDX_STEER_SAT_RATIO    = 6
_IDX_JERK_RMS           = 7
_IDX_MAX_YAW_RATE       = 8
_IDX_STEER_REVERSALS    = 9
_IDX_PEAK_LATERAL_ERROR = 10

# Saturation limit must match offline_tuner u_max[0]
_U_STEER_MAX = 0.4
_DT = 0.05


def report_performance_metrics(history, log_fn=print):
    """
    Score a completed simulate_closed_loop() history dict and print a full
    breakdown to the console.

    Cost terms mirror offline_tuner.run_headless_rollout exactly.
    Weights are imported from offline_tuner to guarantee parity.
    """
    e_y     = np.asarray(history.get("e_y",     []), dtype=float)
    e_psi   = np.asarray(history.get("e_psi",   []), dtype=float)
    v       = np.asarray(history.get("v",        []), dtype=float)
    u_steer = np.asarray(history.get("u_steer", []), dtype=float)
    u_accel = np.asarray(history.get("u_accel", []), dtype=float)

    n = max(len(e_y), 1)
    failed          = bool(history.get("failed", False))
    completion_frac = float(history.get("completion_frac", 1.0))

    # ------------------------------------------------------------------
    # Replicate offline_tuner accumulation exactly
    # ------------------------------------------------------------------

    # error_cost mirrors: error_cost += e_y**2 + 0.5 * e_psi**2
    error_cost = float(np.sum(e_y**2 + 0.5 * e_psi**2))
    rmse = float(np.sqrt(error_cost / n))

    # Yaw rate: approximated from diff(e_psi)/dt (see module note)
    if len(e_psi) > 1:
        r_proxy      = np.diff(e_psi) / _DT
        yaw_rate_cost = 0.8 * float(np.sum(r_proxy**2))
        yaw_rms      = float(np.sqrt(yaw_rate_cost / n))
        max_yaw_rate = float(np.max(np.abs(r_proxy)))
    else:
        yaw_rms = max_yaw_rate = 0.0

    # Control smoothness: mirrors control_smooth += sum((u_opt - u_prev)**2)
    if len(u_steer) > 1 and len(u_accel) > 1:
        du_steer      = np.diff(u_steer)
        du_accel      = np.diff(u_accel)
        control_smooth = float(np.sum(du_steer**2 + du_accel**2))
        smooth_rms    = float(np.sqrt(control_smooth / n))

        # Jerk: mirrors jerk = du - du_prev; jerk_cost += sum(jerk**2)
        jerk_cost = float(np.sum(np.diff(du_steer)**2 + np.diff(du_accel)**2))
        jerk_rms  = float(np.sqrt(jerk_cost / n))
    else:
        smooth_rms = jerk_rms = 0.0

    # Steering effort: mirrors steering_effort += u_opt[0]**2
    steering_effort = float(np.sum(u_steer**2))
    steer_rms       = float(np.sqrt(steering_effort / n))

    # Accel effort: mirrors accel_effort += u_opt[1]**2
    accel_effort = float(np.sum(u_accel**2))
    accel_rms    = float(np.sqrt(accel_effort / n))

    # Peak steering: mirrors max_steering = max(max_steering, abs(u_opt[0]))
    max_steering = float(np.max(np.abs(u_steer))) if len(u_steer) else 0.0

    # Saturation: mirrors if abs(u_opt[0]) > 0.95 * u_max[0]: steering_saturation += 1
    steering_saturation = float(np.sum(np.abs(u_steer) > 0.95 * _U_STEER_MAX)) \
        if len(u_steer) else 0.0
    steering_sat_ratio = steering_saturation / n

    # Reversals: mirrors the sign-change + threshold logic in the rollout loop
    steering_reversals = 0
    if len(u_steer) > 0:
        last_sign = 0
        for val in u_steer:
            current_sign = int(np.sign(val))
            if current_sign != 0:
                if last_sign != 0 and current_sign != last_sign and abs(val) > 0.02:
                    steering_reversals += 1
                last_sign = current_sign

    # Peak lateral error: mirrors peak_lateral_error = max(peak_lateral_error, abs(e_y))
    peak_lateral_error = float(np.max(np.abs(e_y))) if len(e_y) else 0.0

    # ------------------------------------------------------------------
    # Composite score — dot product with shared SCORE_WEIGHTS
    # ------------------------------------------------------------------
    metrics = np.array([
        rmse,
        yaw_rms,
        smooth_rms,
        steer_rms,
        accel_rms,
        max_steering,
        steering_sat_ratio,
        jerk_rms,
        max_yaw_rate,
        float(steering_reversals),
        peak_lateral_error,
    ])

    composite = float(SCORE_WEIGHTS @ metrics)

    # Progress and time bonuses
    progress = np.clip(completion_frac, 0.0, 1.0)

    if failed:
        time_bonus = 0.0
    else:
        v_target = np.asarray(history.get("v_target", []), dtype=float)
        target_speed_mean = float(np.mean(v_target)) if len(v_target) else 12.0
        sim_time      = n * _DT
        expected_time = sim_time / max(target_speed_mean, 1.0)
        time_bonus    = max(0.0, 1.0 - (sim_time / expected_time))

    composite -= COMPLETION_BONUS_WEIGHT * progress + TIME_BONUS_WEIGHT * time_bonus

    if failed:
        composite += DNF_PENALTY * (1.0 - progress)

    # ------------------------------------------------------------------
    # Informational-only terms (not in composite)
    # ------------------------------------------------------------------
    lateral_rmse  = float(np.sqrt(np.mean(e_y**2)))   if len(e_y)   else 0.0
    heading_rmse  = float(np.sqrt(np.mean(e_psi**2)))  if len(e_psi) else 0.0
    v_target_arr  = np.asarray(history.get("v_target", []), dtype=float)
    speed_rmse    = float(np.sqrt(np.mean((v - v_target_arr)**2))) \
        if len(v_target_arr) == len(v) and len(v) > 0 else float("nan")

    # ------------------------------------------------------------------
    # Console report — inline weights from SCORE_WEIGHTS for transparency
    # ------------------------------------------------------------------
    W = SCORE_WEIGHTS
    status_str = "FAILED / OFF-TRACK" if failed else "completed"
    log_fn("=" * 60)
    log_fn(
        f"[Performance] Rollout {status_str} "
        f"({completion_frac * 100:.1f}% of path, {n} steps)"
    )
    log_fn("-" * 60)
    log_fn(f"  Composite score    : {composite:8.4f}  (lower is better)")
    log_fn("-" * 60)
    log_fn(f"  Lateral RMSE       : {lateral_rmse:8.4f} m")
    log_fn(f"  Heading RMSE       : {np.degrees(heading_rmse):8.4f} deg")
    log_fn(f"  Speed RMSE         : " +
           (f"{speed_rmse:8.4f} m/s" if not np.isnan(speed_rmse) else "     n/a"))
    log_fn("-" * 60)
    log_fn(f"  rmse               : {rmse:8.4f}        (x{W[_IDX_RMSE]:.2f})")
    log_fn(f"  Yaw-rate RMS       : {yaw_rms:8.4f} rad/s  (x{W[_IDX_YAW_RMS]:.2f}) [approx]")
    log_fn(f"  Control smooth RMS : {smooth_rms:8.4f}        (x{W[_IDX_SMOOTH_RMS]:.2f})")
    log_fn(f"  Steering RMS       : {np.degrees(steer_rms):8.4f} deg    (x{W[_IDX_STEER_RMS]:.2f})")
    log_fn(f"  Accel RMS          : {accel_rms:8.4f} m/s²   (x{W[_IDX_ACCEL_RMS]:.2f})")
    log_fn(f"  Max steering cmd   : {np.degrees(max_steering):8.4f} deg    (x{W[_IDX_MAX_STEERING]:.2f})")
    log_fn(f"  Steer Sat Ratio    : {steering_sat_ratio*100:8.2f} %      (x{W[_IDX_STEER_SAT_RATIO]:.2f})")
    log_fn(f"  Jerk RMS           : {jerk_rms:8.4f}        (x{W[_IDX_JERK_RMS]:.2f})")
    log_fn(f"  Max yaw rate       : {max_yaw_rate:8.4f} rad/s  (x{W[_IDX_MAX_YAW_RATE]:.2f}) [approx]")
    log_fn(f"  Steer Reversals    : {steering_reversals:8d}           (x{W[_IDX_STEER_REVERSALS]:.2f})")
    log_fn(f"  Peak Lateral Error : {peak_lateral_error:8.4f} m        (x{W[_IDX_PEAK_LATERAL_ERROR]:.2f})")
    log_fn("-" * 60)
    log_fn(f"  Path completion    : {completion_frac*100:8.1f} %      (-{COMPLETION_BONUS_WEIGHT:.2f} bonus)")
    log_fn(f"  Time bonus         : {time_bonus:8.4f}        (-{TIME_BONUS_WEIGHT:.2f} bonus)")
    log_fn("=" * 60)

    return {
        "composite_score":      composite,
        "lateral_rmse_m":       lateral_rmse,
        "heading_rmse_deg":     np.degrees(heading_rmse),
        "speed_rmse_mps":       speed_rmse,
        "yaw_rms_radps":        yaw_rms,
        "control_smooth_rms":   smooth_rms,
        "steering_rms_deg":     np.degrees(steer_rms),
        "accel_rms_mps2":       accel_rms,
        "jerk_rms":             jerk_rms,
        "max_steering_deg":     np.degrees(max_steering),
        "steering_sat_ratio":   steering_sat_ratio,
        "steering_reversals":   steering_reversals,
        "peak_lateral_error_m": peak_lateral_error,
        "completion_pct":       completion_frac * 100.0,
        "failed":               failed,
        "n_steps":              n,
    }