"""
tuner/performance_stats.py — Live Simulator Performance Metric Reporter

PURPOSE
-------
Scores a completed simulation history dict using the same cost decomposition
as tuner/offline_tuner.run_headless_rollout(), allowing direct comparison between
offline tuning scores and live simulator results. This is the "Show Metrics"
button output in gui/simulation.py.

The scoring is deliberately kept in a separate file rather than inlined into
gui/simulation.py so that:
  1. Weights and metric definitions have a single source of truth in sim/scoring.py
     (which itself sources the weight values from settings.py)
  2. The console report can be updated without touching the simulation engine
  3. The returned dict can be used programmatically (e.g. logging, plotting)

PARITY WITH tuner/offline_tuner.py / sim/rollout_core.py
------------------------------------------------
All metric computations mirror the accumulation loop in
sim/rollout_core.run_core_rollout() exactly, by replaying the stored history
through the identical sim/scoring.RolloutMetrics accumulator that
run_core_rollout() itself uses (see the "single source of truth" comment
above rm = RolloutMetrics() below).

SCORE_WEIGHTS, COMPLETION_BONUS_WEIGHT, and TIME_BONUS_WEIGHT are defined in
settings.py and re-exported via sim/scoring.py (imported directly from sim/scoring.py
below, not from tuner/offline_tuner.py), so any change to the scoring formula in
settings.py automatically propagates to this report. DNF_PENALTY is not used
in this file — failure is instead signalled via the `dnf`/`offtrack` booleans
already recorded in the history dict.

USED BY
-------
  gui/simulation.py — btn_optimize "Show Metrics" callback calls
                  report_performance_metrics(sim_history, log_fn=print)

DOES NOT USE
------------
  model/vehicle_physics.py (beyond VehicleParams for u_max_steer), model/bicycle_model.py,
  controller/optimiser.py, sim/speed_profile.py, sim/sim_track.py
"""

from model.vehicle_physics import VehicleParams
import numpy as np
from sim.scoring import (
    SCORE_WEIGHTS, METRIC_SCALES, COMPLETION_BONUS_WEIGHT, TIME_BONUS_WEIGHT,
    COMPLETION_THRESHOLD, TIME_OBJECTIVE_WEIGHT, QUALITY_WEIGHT, RolloutMetrics,
)
from tuner.offline_tuner import (
    PATH_NAMES,
    INITIAL_CONDITIONS,
    evaluate_all_paths,
    _init_context,
    get_cached_model,
    TUNABLE_Q_IDX, TUNABLE_R_IDX, TUNABLE_R_RATE_IDX
)
from settings import DT

# Metric index constants — must stay in sync with SCORE_WEIGHTS order in tuner/offline_tuner.py
_IDX_RMSE               = 0   # Combined tracking RMSE (e_y² + 0.4*e_psi²)
_IDX_YAW_RMS            = 1   # Yaw rate RMS
_IDX_SMOOTH_RMS         = 2   # Control smoothness RMS (Δu)
_IDX_STEER_RMS          = 3   # Steering effort RMS
_IDX_ACCEL_RMS          = 4   # Acceleration effort RMS
_IDX_MAX_STEERING       = 5   # Peak steering command magnitude
_IDX_STEER_SAT_RATIO    = 6   # Fraction of steps near steering saturation
_IDX_JERK_RMS           = 7   # Control jerk RMS (Δ²u)
_IDX_MAX_YAW_RATE       = 8   # Peak yaw rate
_IDX_STEER_REVERSAL_RMS = 9   # Magnitude-weighted RMS of steering reversal swings
_IDX_PEAK_LATERAL_ERROR = 10  # Worst single-step lateral error
_IDX_SPEED_RMSE         = 11
_IDX_ACCEL_REVERSAL_RMS = 12  # Magnitude-weighted RMS of throttle/brake reversal swings

def report_performance_metrics(history, log_fn=print):
    """
    Score a completed simulate_closed_loop() history dict and print a detailed
    breakdown to the console. Returns the same metrics dict for programmatic use.

    All cost terms are computed to match offline_tuner.run_headless_rollout()
    exactly — both replay through the identical scoring.RolloutMetrics
    accumulator, including the true plant yaw rate (history["r"]), not an
    approximation.

    METRIC COMPUTATION PIPELINE
    ----------------------------
    The function replicates the accumulation that run_headless_rollout() does
    step-by-step, but operates on the already-stored history arrays:

      rmse:               sqrt( Σ(e_y² + 0.4*e_psi²) / n )
      yaw_rms:            sqrt( 0.8 * Σ(r²) / n )                [r = true plant yaw rate, history["r"]]
      smooth_rms:         sqrt( Σ(Δu_steer² + Δu_accel²) / n )   [Δu from u[-1]=0]
      steer_rms:          sqrt( Σ(u_steer²) / n )
      accel_rms:          sqrt( Σ(u_accel²) / n )
      max_steering:       max(|u_steer|)
      steering_sat_ratio: count(|u_steer| > 0.95 * max steer) / n
      jerk_rms:           sqrt( Σ(Δ²u_steer² + Δ²u_accel²) / n ) [Δ²u from u[-1]=du[-1]=0]
      max_yaw_rate:       max(|r|)                                [r = true plant yaw rate, history["r"]]
      steering_reversal_rms: sqrt( Σ swing² / n ) for each sign change > 0.02
                          rad threshold, where swing = |u_steer| + |u_steer_prev|
                          [fed into the composite score; magnitude-weighted so a
                          tiny trim wiggle contributes almost nothing while a
                          large swing dominates — this is what distinguishes
                          controller hunting from a twisty path legitimately
                          needing more frequent small direction changes. The raw
                          reversal count and its per-step rate are also reported
                          separately as "steering_reversals"/"steering_reversal_rate",
                          informational only — not scored directly, since an
                          unnormalised count mechanically grows with rollout
                          length / shrinks with DT and can't tell swing size apart]
      peak_lateral_error: max(|e_y|)
      accel_reversal_rms: sqrt( Σ swing² / n ) for each a_cmd sign change > 0.02
                          m/s² threshold, where swing = |u_accel| + |u_accel_prev|
                          [identical construction to steering_reversal_rms above,
                          applied to a_cmd instead of u_steer]

    These 13 metrics are then combined.

    Parameters
    ----------
    history : dict
        Simulation history dict as populated by simulate_closed_loop() in
        gui/simulation.py. Expected keys:
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
          "yaw_rms_radps"        : float — RMS yaw rate (rad/s)
          "control_smooth_rms"   : float — RMS control rate-of-change
          "steering_rms_deg"     : float — RMS steering command (deg)
          "accel_rms_mps2"       : float — RMS acceleration command (m/s²)
          "jerk_rms"             : float — RMS control jerk
          "max_steering_deg"     : float — Peak steering command (deg)
          "steering_sat_ratio"   : float — Fraction of steps at saturation
          "steering_reversal_rms": float — Magnitude-weighted RMS of steering
                                    reversal swings (scored metric)
          "steering_reversal_rate": float — Reversals per step, informational
                                    only (not scored)
          "steering_reversals"   : int   — Raw count of steering direction
                                    changes, informational only (not scored)
          "accel_reversal_rms"   : float — Magnitude-weighted RMS of throttle/
                                    brake reversal swings (scored metric)
          "accel_reversal_rate"  : float — Reversals per step, informational
                                    only (not scored)
          "accel_reversals"      : int   — Raw count of accel/brake direction
                                    changes, informational only (not scored)
          "peak_lateral_error_m" : float — Worst lateral error (m)
          "completion_pct"       : float — Path completion percentage
          "failed"               : bool  — Whether the run ended in a failure
          "n_steps"              : int   — Total steps completed

    Called by: gui/simulation.py (btn_optimize "Show Metrics" callback)
    """
    # Prefer the GROUND-TRUTH tracking error when it's present. Under SLAM
    # noise, history["e_y"]/["e_psi"] are what the controller PERCEIVED, which
    # is not where the car actually was — scoring on those would credit a
    # mislocalised car for tracking its own wrong belief. run_core_rollout()
    # scores on the *_true series for exactly this reason, so replaying the
    # perceived ones here would silently disagree with it. Falls back to the
    # perceived series for histories recorded before *_true existed (and they
    # are identical anyway when SLAM noise is off).
    e_y     = np.asarray(history.get("e_y_true",   history.get("e_y",   [])), dtype=float)
    e_psi   = np.asarray(history.get("e_psi_true", history.get("e_psi", [])), dtype=float)
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
    steering_reversal_rms  = result["steering_reversal_rms"]
    steering_reversal_rate = result["steering_reversal_rate"]  # raw rate, informational only
    steering_reversals  = result["steering_reversals"]  # raw count, informational only
    accel_reversal_rms  = result["accel_reversal_rms"]
    accel_reversal_rate = result["accel_reversal_rate"]  # raw rate, informational only
    accel_reversals     = result["accel_reversals"]  # raw count, informational only
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
    log_fn(f"  Yaw-rate RMS       : {yaw_rms:8.4f} rad/s  (x{W[_IDX_YAW_RMS]:.2f})")
    log_fn(f"  Control smooth RMS : {smooth_rms:8.4f}        (x{W[_IDX_SMOOTH_RMS]:.2f})")
    log_fn(f"  Steering RMS       : {np.degrees(steer_rms):8.4f} deg    (x{W[_IDX_STEER_RMS]:.2f})")
    log_fn(f"  Accel RMS          : {accel_rms:8.4f} m/s²   (x{W[_IDX_ACCEL_RMS]:.2f})")
    log_fn(f"  Max steering cmd   : {np.degrees(max_steering):8.4f} deg    (x{W[_IDX_MAX_STEERING]:.2f})")
    log_fn(f"  Steer Sat Ratio    : {steering_sat_ratio*100:8.2f} %      (x{W[_IDX_STEER_SAT_RATIO]:.2f})")
    log_fn(f"  Jerk RMS           : {jerk_rms:8.4f}        (x{W[_IDX_JERK_RMS]:.2f})")
    log_fn(f"  Max yaw rate       : {max_yaw_rate:8.4f} rad/s  (x{W[_IDX_MAX_YAW_RATE]:.2f})")
    log_fn(f"  Steer Reversals    : {steering_reversals:8d} (raw)  {steering_reversal_rate:8.4f} /step  {steering_reversal_rms:8.4f} rms (x{W[_IDX_STEER_REVERSAL_RMS]:.2f})")
    log_fn(f"  Accel Reversals    : {accel_reversals:8d} (raw)  {accel_reversal_rate:8.4f} /step  {accel_reversal_rms:8.4f} rms (x{W[_IDX_ACCEL_REVERSAL_RMS]:.2f})")
    log_fn(f"  Peak Lateral Error : {peak_lateral_error:8.4f} m        (x{W[_IDX_PEAK_LATERAL_ERROR]:.2f})")
    log_fn(f"  Speed RMS          : {speed_rmse:8.4f} m/s        (x{W[_IDX_SPEED_RMSE]:.2f})")
    log_fn("-" * 60)
    # Completion is now a hard CONSTRAINT, not a bonus: a run below
    # COMPLETION_THRESHOLD is scored above CONSTRAINT_FLOOR and can't be
    # rescued by good quality metrics. time_bonus is optimal_lap_time /
    # actual_time, so 1.0 is the physical limit.
    log_fn(f"  Path completion    : {completion_frac*100:8.1f} %      "
           f"(constraint, must be >= {COMPLETION_THRESHOLD*100:.0f}%)")
    log_fn(f"  Time vs optimal    : {time_bonus:8.4f}        "
           f"(1.0 = physical limit; cost = {TIME_OBJECTIVE_WEIGHT:.2f} x (1-this))")
    log_fn(f"  Quality group      : {'':8s}        (x{QUALITY_WEIGHT:.2f} of the metrics below)")
    log_fn("-" * 60)
    # Effective contribution = weight * (metric / reference scale). This is
    # what each metric actually adds to the composite, and it is the number
    # to look at when deciding whether a metric is pulling its weight — the
    # "(x0.05)" annotations above are the raw weights, which say nothing on
    # their own about influence. Before METRIC_SCALES existed, several terms
    # here were 2-3 orders of magnitude below the tracking terms despite
    # respectable-looking weights (see settings.METRIC_SCALES).
    _contrib = [
        ("rmse",                  rmse,                  _IDX_RMSE),
        ("yaw_rms",               yaw_rms,               _IDX_YAW_RMS),
        ("smooth_rms",            smooth_rms,            _IDX_SMOOTH_RMS),
        ("steer_rms",             steer_rms,             _IDX_STEER_RMS),
        ("accel_rms",             accel_rms,             _IDX_ACCEL_RMS),
        ("max_steering",          max_steering,          _IDX_MAX_STEERING),
        ("steering_sat_ratio",    steering_sat_ratio,    _IDX_STEER_SAT_RATIO),
        ("jerk_rms",              jerk_rms,              _IDX_JERK_RMS),
        ("max_yaw_rate",          max_yaw_rate,          _IDX_MAX_YAW_RATE),
        ("steering_reversal_rms", steering_reversal_rms, _IDX_STEER_REVERSAL_RMS),
        ("peak_lateral_error",    peak_lateral_error,    _IDX_PEAK_LATERAL_ERROR),
        ("speed_rmse",            speed_rmse,            _IDX_SPEED_RMSE),
        ("accel_reversal_rms",    accel_reversal_rms,    _IDX_ACCEL_REVERSAL_RMS),
    ]
    _rows = []
    for _name, _val, _i in _contrib:
        _v = 0.0 if (_val is None or np.isnan(_val)) else float(_val)
        _rows.append((_name, W[_i] * (_v / METRIC_SCALES[_i])))
    _total = sum(c for _, c in _rows) or 1.0
    log_fn("  Effective contribution to score (weight x metric/scale):")
    for _name, _c in sorted(_rows, key=lambda r: -abs(r[1])):
        log_fn(f"    {_name:<22s} {_c:8.4f}  ({100.0 * _c / _total:5.1f}%)")
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
        "steering_reversal_rms": steering_reversal_rms,
        "steering_reversal_rate": steering_reversal_rate,  # raw rate, informational only
        "steering_reversals":   steering_reversals,  # raw count, informational only
        "accel_reversal_rms":   accel_reversal_rms,
        "accel_reversal_rate":  accel_reversal_rate,  # raw rate, informational only
        "accel_reversals":      accel_reversals,  # raw count, informational only
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
    Q_w : np.ndarray, shape (8, 8)     State cost matrix from gui/simulation.py.
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

    Called by: gui/simulation.py (btn_benchmark "Benchmark All Paths" callback)
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