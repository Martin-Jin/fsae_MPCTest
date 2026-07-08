"""
scoring.py — Single Source of Truth for MPC Rollout Scoring

PURPOSE
-------
Both offline_tuner.run_headless_rollout() and performance_stats's live-sim
report used to hand-implement the same 12-metric accumulation independently.
That duplication is exactly how the two silently drift apart over time.

This module is now the ONLY place that defines:
  1. How raw per-step signals (e_y, e_psi, r, u, v, v_target) get turned
     into the 12 score metrics (RolloutMetrics)
  2. How those 12 metrics get combined into the final composite score
     (compute_composite_score)

USED BY
-------
  offline_tuner.py      — run_headless_rollout() accumulates live, step-by-step
  performance_stats.py  — report_performance_metrics() replays stored history
                           arrays through the identical accumulator
  simulation.py          — should call RolloutMetrics the same way
                           offline_tuner does (see integration note at bottom)

NOTE: review pass found no functional bugs in this file. Comments only
lightly tightened for clarity; logic is unchanged from the original.
"""

import numpy as np
from settings import (
    SCORE_WEIGHTS,
    COMPLETION_BONUS_WEIGHT,
    TIME_BONUS_WEIGHT,
    DNF_PENALTY,
    DNF_OFFTRACK_PENALTY,
)

# Metric index constants — must stay in sync with SCORE_WEIGHTS order in settings.py
IDX_RMSE               = 0
IDX_YAW_RMS            = 1
IDX_SMOOTH_RMS         = 2
IDX_STEER_RMS          = 3
IDX_ACCEL_RMS          = 4
IDX_MAX_STEERING       = 5
IDX_STEER_SAT_RATIO    = 6
IDX_JERK_RMS           = 7
IDX_MAX_YAW_RATE       = 8
IDX_STEER_REVERSALS    = 9
IDX_PEAK_LATERAL_ERROR = 10
IDX_SPEED_RMSE         = 11


def compute_composite_score(
    rmse,
    yaw_rms,
    smooth_rms,
    steer_rms,
    accel_rms,
    max_steering,
    steering_sat_ratio,
    jerk_rms,
    max_yaw_rate,
    steering_reversals,
    peak_lateral_error,
    speed_rmse,
    progress,
    time_bonus=0.0,
    dnf=False,
    offtrack=False,
    inaccurate_count=0,
):
    """
    Single source of truth for the composite performance score.
    Combines the 12 metrics with SCORE_WEIGHTS, applies completion/time
    bonuses, DNF penalties, and the inaccurate-solver factor.
    Lower is better.

    Parameter order here MUST match the IDX_* constants above / the order
    of SCORE_WEIGHTS in settings.py — the metrics array below is built
    positionally, not by name.
    """
    metrics = np.array(
        [
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
            speed_rmse,
        ]
    )
    score = float(SCORE_WEIGHTS @ metrics)

    # Bonuses reward progress/time; clip progress defensively in case a
    # caller passes a slightly out-of-range value (e.g. 1.0000001 from
    # floating-point arc-length accumulation).
    progress = float(np.clip(progress, 0.0, 1.0))
    score -= COMPLETION_BONUS_WEIGHT * progress + TIME_BONUS_WEIGHT * time_bonus

    if dnf:
        score += DNF_PENALTY
    if offtrack:
        score += DNF_OFFTRACK_PENALTY
    if inaccurate_count > 0:
        # OPTIMAL_INACCURATE solves are still usable but less trustworthy;
        # inflate the score proportionally (capped at 5 occurrences -> 50%)
        # rather than rejecting the rollout outright.
        factor = min(5, inaccurate_count) * 0.1
        score = score + abs(score) * factor

    return score


class RolloutMetrics:
    """
    Accumulates the 12 raw score-metric sums one simulation step at a time.

    This is THE canonical per-step accumulation logic. Any rollout loop
    (offline tuner, live simulator, or a metrics-replay over stored history)
    must funnel its per-step signals through add_step() rather than
    re-deriving the formulas. That is what guarantees offline-tuner scores
    and live-simulator "Show Metrics" scores are computed identically.

    Usage
    -----
        m = RolloutMetrics()
        for step in rollout:
            m.add_step(e_y, e_psi, r, u_opt, v_target, v_actual,
                       u_max_steer, solver_failed=..., inaccurate=...)
        result = m.finalize(progress=..., time_bonus=..., dnf=..., offtrack=...)
    """

    def __init__(self):
        self.n_steps = 0
        self.error_cost = 0.0
        self.yaw_rate_cost = 0.0
        self.control_smooth = 0.0
        self.jerk_cost = 0.0
        self.steering_effort = 0.0
        self.accel_effort = 0.0
        self.steering_saturation = 0.0
        self.steering_reversals = 0
        self._last_sign = 0
        self.max_yaw_rate = 0.0
        self.max_steering = 0.0
        self.max_accel = 0.0
        self.peak_lateral_error = 0.0
        self.speed_cost = 0.0
        self.inaccurate_count = 0
        self.u_prev = np.zeros(2)
        self.du_prev = np.zeros(2)

    def add_step(self, e_y, e_psi, r, u_opt, v_target, v_actual, u_max_steer,
                 solver_failed=False, inaccurate=False):
        """
        Accumulate one timestep's contribution to all 12 metrics.

        Parameters
        ----------
        e_y, e_psi : float   Lateral / heading tracking error this step.
        r          : float   True yaw rate (plant state[5]).
        u_opt      : array (2,)  Applied [delta_cmd, a_cmd] this step
                     (== previous command if the solver failed this step).
        v_target, v_actual : float  Planner target speed / true vx.
        u_max_steer : float  Vehicle's max_steer bound (for saturation check).
        solver_failed : bool  True if the MPC solve failed this step
                       (adds the flat 5.0 smoothness penalty).
        inaccurate : bool  True if the solver returned OPTIMAL_INACCURATE.
        """
        u_opt = np.asarray(u_opt, dtype=float)
        self.n_steps += 1

        if solver_failed:
            self.control_smooth += 5.0
        if inaccurate:
            self.inaccurate_count += 1

        current_sign = int(np.sign(u_opt[0]))
        if current_sign != 0:
            if (self._last_sign != 0 and current_sign != self._last_sign
                    and abs(u_opt[0]) > 0.02):
                self.steering_reversals += 1
            self._last_sign = current_sign

        self.max_yaw_rate = max(self.max_yaw_rate, abs(r))
        e_v_now = v_actual - v_target
        self.speed_cost += e_v_now ** 2
        self.error_cost += 1.2 * e_y ** 2 + 0.4 * e_psi ** 2
        self.yaw_rate_cost += 0.8 * r ** 2

        self.control_smooth += float(np.sum((u_opt - self.u_prev) ** 2))
        du = u_opt - self.u_prev
        jerk = du - self.du_prev
        self.jerk_cost += float(np.sum(jerk ** 2))
        self.du_prev = du

        self.steering_effort += u_opt[0] ** 2
        self.accel_effort += u_opt[1] ** 2
        if abs(u_opt[0]) > 0.95 * u_max_steer:
            self.steering_saturation += 1.0

        self.peak_lateral_error = max(self.peak_lateral_error, abs(e_y))
        self.max_steering = max(self.max_steering, abs(u_opt[0]))
        self.max_accel = max(self.max_accel, abs(u_opt[1]))

        self.u_prev = u_opt.copy()

    def finalize(self, progress, time_bonus=0.0, dnf=False, offtrack=False):
        """
        Normalise accumulated sums to RMS/ratio metrics and compute the
        final composite score. Returns a dict usable both for CMA-ES
        (just read ["composite_score"]) and for human-readable reports.
        """
        n = max(self.n_steps, 1)
        rmse = float(np.sqrt(self.error_cost / n))
        yaw_rms = float(np.sqrt(self.yaw_rate_cost / n))
        smooth_rms = float(np.sqrt(self.control_smooth / n))
        steer_rms = float(np.sqrt(self.steering_effort / n))
        accel_rms = float(np.sqrt(self.accel_effort / n))
        jerk_rms = float(np.sqrt(self.jerk_cost / n))
        steering_sat_ratio = self.steering_saturation / n
        speed_rmse = float(np.sqrt(self.speed_cost / n))

        score = compute_composite_score(
            rmse, yaw_rms, smooth_rms, steer_rms, accel_rms,
            self.max_steering, steering_sat_ratio, jerk_rms, self.max_yaw_rate,
            self.steering_reversals, self.peak_lateral_error, speed_rmse,
            progress=progress, time_bonus=time_bonus, dnf=dnf, offtrack=offtrack,
            inaccurate_count=self.inaccurate_count,
        )

        return {
            "composite_score": score,
            "rmse": rmse,
            "yaw_rms_radps": yaw_rms,
            "control_smooth_rms": smooth_rms,
            "steer_rms": steer_rms,
            "accel_rms_mps2": accel_rms,
            "jerk_rms": jerk_rms,
            "max_steering_rad": self.max_steering,
            "max_accel_mps2": self.max_accel,
            "steering_sat_ratio": steering_sat_ratio,
            "steering_reversals": self.steering_reversals,
            "peak_lateral_error_m": self.peak_lateral_error,
            "speed_rmse_mps": speed_rmse,
            "max_yaw_rate_radps": self.max_yaw_rate,
            "inaccurate_count": self.inaccurate_count,
            "n_steps": n,
        }