"""
performance_stats.py — Live Simulator Performance Metric Reporter

PURPOSE
-------
Scores a completed simulation history dict using the same cost decomposition
as offline_tuner.run_headless_rollout(), allowing direct comparison between
offline tuning scores and live simulator results. This is the "Show Metrics"
button output in simulation.py.

The scoring is deliberately kept in a separate file rather than inlined into
simulation.py so that:
  1. Weights and metric definitions have a single source of truth in scoring.py
     (which itself sources the weight values from settings.py)
  2. The console report can be updated without touching the simulation engine
  3. The returned dict can be used programmatically (e.g. logging, plotting)

PARITY WITH offline_tuner.py / rollout_core.py
------------------------------------------------
All metric computations mirror the accumulation loop in
rollout_core.run_core_rollout() exactly, by replaying the stored history
through the identical scoring.RolloutMetrics accumulator that
run_core_rollout() itself uses (see the "single source of truth" comment
above rm = RolloutMetrics() below).

SCORE_WEIGHTS, COMPLETION_BONUS_WEIGHT, and TIME_BONUS_WEIGHT are defined in
settings.py and re-exported via scoring.py (imported directly from scoring.py
below, not from offline_tuner.py), so any change to the scoring formula in
settings.py automatically propagates to this report. DNF_PENALTY is not used
in this file — failure is instead signalled via the `dnf`/`offtrack` booleans
already recorded in the history dict.

USED BY
-------
  simulation.py — btn_optimize "Show Metrics" callback calls
                  report_performance_metrics(sim_history, log_fn=print)

DOES NOT USE
------------
  vehicle_physics.py (beyond VehicleParams for u_max_steer), bicycle_model.py,
  optimiser.py, speed_profile.py, sim_track.py
"""

from vehicle_physics import VehicleParams
import numpy as np
from scoring import SCORE_WEIGHTS, COMPLETION_BONUS_WEIGHT, TIME_BONUS_WEIGHT, RolloutMetrics
from offline_tuner import (
    PATH_NAMES,
    INITIAL_CONDITIONS,
    evaluate_all_paths,
    _init_context,
    get_cached_model,
    TUNABLE_Q_IDX, TUNABLE_R_IDX, TUNABLE_R_RATE_IDX
)
from settings import DT

# Metric index constants — must stay in sync with SCORE_WEIGHTS order in offline_tuner.py
_IDX_RMSE               = 0   # Combined tracking RMSE (e_y² + 0.4*e_psi²)
_IDX_YAW_RMS            = 1   # Yaw rate RMS
_IDX_SMOOTH_RMS         = 2   # Control smoothness RMS (Δu)
_IDX_STEER_RMS          = 3   # Steering effort RMS
_IDX_ACCEL_RMS          = 4   # Acceleration effort RMS
_IDX_MAX_STEERING       = 5   # Peak steering command magnitude
_IDX_STEER_SAT_RATIO    = 6   # Fraction of steps near steering saturation
_IDX_JERK_RMS           = 7   # Control jerk RMS (Δ²u)
_IDX_MAX_YAW_RATE       = 8   # Peak yaw rate
_IDX_STEER_REVERSALS    = 9   # Count of steering direction reversals
_IDX_PEAK_LATERAL_ERROR = 10  # Worst single-step lateral error
_IDX_SPEED_RMSE         = 11

def report_performance_metrics(history, log_fn=print):
    """
    Score a completed simulate_closed_loop() history dict and print a detailed
    breakdown to the console. Returns the same metrics dict for programmatic use.

    All cost terms are computed to match offline_tuner.run_headless_rollout()
    exactly (see module docstring for the one approximation on yaw terms).

    METRIC COMPUTATION PIPELINE
    ----------------------------
    The function replicates the accumulation that run_headless_rollout() does
    step-by-step, but operates on the already-stored history arrays:

      rmse:               sqrt( Σ(e_y² + 0.4*e_psi²) / n )
      yaw_rms:            sqrt( 0.8 * Σ(r_proxy²) / n )         [proxy: Δe_psi/dt]
      smooth_rms:         sqrt( Σ(Δu_steer² + Δu_accel²) / n )   [Δu from u[-1]=0]
      steer_rms:          sqrt( Σ(u_steer²) / n )
      accel_rms:          sqrt( Σ(u_accel²) / n )
      max_steering:       max(|u_steer|)
      steering_sat_ratio: count(|u_steer| > 0.95 * max steer) / n
      jerk_rms:           sqrt( Σ(Δ²u_steer² + Δ²u_accel²) / n ) [Δ²u from u[-1]=du[-1]=0]
      max_yaw_rate:       max(|r_proxy|)                          [proxy: Δe_psi/dt]
      steering_reversals: count of sign changes > 0.02 rad threshold
      peak_lateral_error: max(|e_y|)

    These 12 metrics are then combined.

    Parameters
    ----------
    history : dict
        Simulation history dict as populated by simulate_closed_loop() in
        simulation.py. Expected keys:
          "e_y"             : list of float — lateral error at each step (m)
          "e_psi"           : list of float — heading error at each step (rad)
          "v"               : list of float — vehicle speed at each step (m/s)
          "v_target"        : list of float — target speed at each step (m/s)
          "u_steer"         : list of float — applied steering command (rad)
          "u_accel"         : list of float — applied acceleration command (m/s²)
          "failed"          : bool — True if the vehicle went off-track or failed
          "completion_frac" : float — fraction of path completed [0, 1]
        Missing keys are handled gracefully (empty arrays or default values).
    log_fn : callable, optional
        Output function for the console report. Defaults to print().
        Pass a custom logger (e.g. file write, GUI text widget) to redirect.

    Returns
    -------
    metrics_dict : dict
        Dictionary of all computed metrics with descriptive keys:
          "composite_score"      : float — main objective (lower is better)
          "lateral_rmse_m"       : float — RMS lateral error (m)
          "heading_rmse_deg"     : float — RMS heading error (deg)
          "speed_rmse_mps"       : float — RMS speed error (m/s), NaN if unavailable
          "yaw_rms_radps"        : float — RMS yaw rate proxy (rad/s)
          "control_smooth_rms"   : float — RMS control rate-of-change
          "steering_rms_deg"     : float — RMS steering command (deg)
          "accel_rms_mps2"       : float — RMS acceleration command (m/s²)
          "jerk_rms"             : float — RMS control jerk
          "max_steering_deg"     : float — Peak steering command (deg)
          "steering_sat_ratio"   : float — Fraction of steps at saturation
          "steering_reversals"   : int   — Count of steering direction changes
          "peak_lateral_error_m" : float — Worst lateral error (m)
          "completion_pct"       : float — Path completion percentage
          "failed"               : bool  — Whether the run ended in a failure
          "n_steps"              : int   — Total steps completed

    Called by: simulation.py (btn_optimize "Show Metrics" callback)
    """
    e_y     = np.asarray(history.get("e_y",     []), dtype=float)
    e_psi   = np.asarray(history.get("e_psi",   []), dtype=float)
    v       = np.asarray(history.get("v",        []), dtype=float)
    v_target_arr = np.asarray(history.get("v_target", []), dtype=float)
    u_steer = np.asarray(history.get("u_steer", []), dtype=float)
    u_accel = np.asarray(history.get("u_accel", []), dtype=float)
    r_arr   = np.asarray(history.get("r",        []), dtype=float)
    solver_failed_arr = history.get("solver_failed", [])

    n_hist          = len(e_y)
    failed          = bool(history.get("failed", False))
    completion_frac = float(history.get("completion_frac", 1.0))
    u_max_steer     = VehicleParams().max_steer

    # ── Replay every step through the SAME accumulator offline_tuner uses ─────
    # This is what guarantees "Show Metrics" and the offline benchmark produce
    # identical numbers for the identical trajectory: there is only one
    # implementation of the metric math (scoring.RolloutMetrics), not two.
    rm = RolloutMetrics()
    for i in range(n_hist):
        u_opt = np.array([
            u_steer[i] if i < len(u_steer) else 0.0,
            u_accel[i] if i < len(u_accel) else 0.0,
        ])
        r_i = r_arr[i] if i < len(r_arr) else 0.0
        v_t = v_target_arr[i] if i < len(v_target_arr) else (v[i] if i < len(v) else 0.0)
        v_a = v[i] if i < len(v) else 0.0
        s_failed = bool(solver_failed_arr[i]) if i < len(solver_failed_arr) else False
        rm.add_step(
            e_y=e_y[i], e_psi=e_psi[i], r=r_i,
            u_opt=u_opt, v_target=v_t, v_actual=v_a,
            u_max_steer=u_max_steer, solver_failed=s_failed,
        )

    # inaccurate_count isn't stored per-step in history; pass through the total
    rm.inaccurate_count = int(history.get("inaccurate_count", 0))

    progress = float(np.clip(completion_frac, 0.0, 1.0))
    time_bonus = 0.0 if failed else float(history.get("time_bonus") or 0.0)

    result = rm.finalize(
        progress=progress, time_bonus=time_bonus, dnf=failed,
        offtrack=history.get("offtrack", False),
    )
    composite = result["composite_score"]

    # ── Informational-only metrics (not in composite score) ───────────────────
    lateral_rmse = float(np.sqrt(np.mean(e_y**2)))   if len(e_y)   else 0.0
    heading_rmse = float(np.sqrt(np.mean(e_psi**2))) if len(e_psi) else 0.0

    rmse                = result["rmse"]
    yaw_rms             = result["yaw_rms_radps"]
    smooth_rms          = result["control_smooth_rms"]
    steer_rms           = result["steer_rms"]
    accel_rms           = result["accel_rms_mps2"]
    max_steering        = result["max_steering_rad"]
    steering_sat_ratio  = result["steering_sat_ratio"]
    jerk_rms            = result["jerk_rms"]
    max_yaw_rate        = result["max_yaw_rate_radps"]
    steering_reversals  = result["steering_reversals"]
    peak_lateral_error  = result["peak_lateral_error_m"]
    speed_rmse          = result["speed_rmse_mps"]
    n                   = result["n_steps"]

    # ── Console report ────────────────────────────────────────────────────────
    W          = SCORE_WEIGHTS
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
    log_fn(f"  Speed RMS          : {speed_rmse:8.4f} m/s        (x{W[_IDX_SPEED_RMSE]:.2f})")
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

def benchmark_weights(Q_w, R_w, R_rate_w, n_repeats=3, log_fn=print):
    """
    Run every path in PATH_NAMES n_repeats times using evaluate_all_paths()
    and report a full per-path and aggregate score breakdown.

    Mirrors the offline tuner's evaluation approach but covers all paths
    (not just VALIDATION_SUITE), giving a comprehensive view of weight
    generalisation. Scores are computed by run_headless_rollout() so they
    are directly comparable to offline tuning results.

    Uses offline tuner's initial conditions to bench mark all paths.

    Parameters
    ----------
    Q_w : np.ndarray, shape (8, 8)     State cost matrix from simulation.py.
    R_w : np.ndarray, shape (2, 2)     Input cost matrix.
    R_rate_w : np.ndarray, shape (2, 2) Rate-of-change cost matrix.
    n_repeats : int
        Number of rollouts per path (scores averaged). Default 3.
    log_fn : callable
        Output function. Defaults to print().

    Returns
    -------
    dict with keys:
        'mean_score' : float  — aggregate mean across all paths × repeats
        'per_path'   : dict   — {path_name: mean_score}
        'all_scores' : list   — every individual rollout score

    Called by: simulation.py (btn_benchmark "Benchmark All Paths" callback)
    """
    # Populate _init_context in this process so evaluate_all_paths() can call
    # run_headless_rollout() without a worker pool.
    _init_context["Q"]              = Q_w
    _init_context["R"]              = R_w
    _init_context["R_rate"]         = R_rate_w
    _init_context["vehicle_params"] = VehicleParams()

    # Pre-populate the model cache to avoid matrix-exponential overhead per step.
    for vx in np.arange(0.5, 20.1, 0.1):
        get_cached_model(round(float(vx), 1), 0.05)

    identity_vec = []
    
    for idx in TUNABLE_Q_IDX:
        identity_vec.append(1.0 if Q_w[idx, idx] != 0.0 else 0.0)
        
    for idx in TUNABLE_R_IDX:
        identity_vec.append(1.0 if R_w[idx, idx] != 0.0 else 0.0)
        
    for idx in TUNABLE_R_RATE_IDX:
        identity_vec.append(1.0 if R_rate_w[idx, idx] != 0.0 else 0.0)

    vec = np.array(identity_vec, dtype=float)

    # Temporarily override the context templates so evaluate_all_paths uses Q_w etc.
    eye0 = INITIAL_CONDITIONS[0][0]
    epsi0 = INITIAL_CONDITIONS[0][1]
    results = evaluate_all_paths(vec, n_repeats=n_repeats, epsi0=epsi0, ey0=eye0)

    # ── Console report ────────────────────────────────────────────────────────
    log_fn("=" * 60)
    log_fn(f"[Benchmark] All paths  ×{n_repeats} repeats each")
    log_fn(f"  Paths evaluated : {len(PATH_NAMES)}")
    log_fn(f"  Total rollouts  : {len(results['all_scores'])}")
    log_fn("-" * 60)
    for path_name, score in sorted(results['per_path'].items(), key=lambda x: x[1]):
        log_fn(f"  {path_name:<30s}: {score:8.4f}")
    log_fn("-" * 60)
    log_fn(f"  Mean composite score : {results['mean_score']:8.4f}")
    log_fn("=" * 60)

    return results