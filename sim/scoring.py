"""
sim/scoring.py — Single Source of Truth for MPC Rollout Scoring

PURPOSE
-------
Both offline_tuner.run_headless_rollout() and performance_stats's live-sim
report used to hand-implement the same 13-metric accumulation independently.
That duplication is exactly how the two silently drift apart over time.

This module is now the ONLY place that defines:
  1. How raw per-step signals (e_y, e_psi, r, u, v, v_target) get turned
     into the 13 score metrics (RolloutMetrics)
  2. How those 13 metrics get combined into the final composite score
     (compute_composite_score)

USED BY
-------
  tuner/offline_tuner.py      — run_headless_rollout() accumulates live, step-by-step
  tuner/performance_stats.py  — report_performance_metrics() replays stored history
                           arrays through the identical accumulator
  gui/simulation.py          — should call RolloutMetrics the same way
                           tuner/offline_tuner does (see integration note at bottom)
"""

import numpy as np
from settings import (
    SCORE_WEIGHTS,
    METRIC_SCALES,
    COMPLETION_BONUS_WEIGHT,
    TIME_BONUS_WEIGHT,
    DNF_PENALTY,
    DNF_OFFTRACK_PENALTY,
    CONSTRAINT_FLOOR,
    COMPLETION_THRESHOLD,
    TIME_OBJECTIVE_WEIGHT,
    QUALITY_WEIGHT,
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
IDX_STEER_REVERSAL_RMS = 9
IDX_PEAK_LATERAL_ERROR = 10
IDX_SPEED_RMSE         = 11
IDX_ACCEL_REVERSAL_RMS = 12


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
    steering_reversal_rms,
    peak_lateral_error,
    speed_rmse,
    progress,
    accel_reversal_rms=0.0,
    time_bonus=0.0,
    dnf=False,
    offtrack=False,
    inaccurate_count=0,
    reached_end=None,
):
    """
    Single source of truth for the composite performance score.
    Combines the 13 metrics with SCORE_WEIGHTS, applies completion/time
    bonuses, DNF penalties, and the inaccurate-solver factor.
    Lower is better.

    Parameter order here MUST match the IDX_* constants above / the order
    of SCORE_WEIGHTS in settings.py — the metrics array below is built
    positionally, not by name. accel_reversal_rms is keyword-only with a
    default so existing positional callers (which predate this metric)
    don't break; new callers should pass it explicitly.

    steering_reversal_rms is an RMS, magnitude-weighted measure of steering
    direction reversals (sqrt(Σ swing² / n steps)) — see
    RolloutMetrics.finalize(). A reversal's "swing" is how far the steering
    command travelled from its previous value before flipping sign, so a
    tiny back-and-forth trim correction near zero contributes almost nothing
    while a large aggressive swing contributes proportionally (squared) more.
    This also keeps a twistier path (which legitimately needs more direction
    changes) from being penalised the same as controller-induced dithering,
    since a path-demanded reversal is typically a small, deliberate swing
    while hunting/chatter tends to produce many small-to-moderate swings that
    still individually average out much lower than one genuine large
    correction. Normalising by n steps keeps it on a comparable per-step
    scale to the other RMS metrics instead of mechanically growing with
    rollout length or shrinking with DT.

    accel_reversal_rms is the identical construction applied to u_opt[1]
    (a_cmd) instead of u_opt[0] (delta_cmd): steering_reversal_rms only ever
    looks at u_opt[0], so without this nothing in the score discourages
    a_cmd from oscillating across zero even though the same
    magnitude-weighted-swing rationale applies identically to accel/brake
    effort.
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
            float(steering_reversal_rms),
            peak_lateral_error,
            speed_rmse,
            float(accel_reversal_rms),
        ]
    )
    # Normalise each metric by its reference scale BEFORE weighting, so a
    # weight expresses priority rather than silently doing unit conversion
    # too. Without this the effective influence of a metric is
    # weight x typical magnitude, which made the smoothness/oscillation terms
    # 2-3 orders of magnitude too small to affect the outcome — see
    # settings.METRIC_SCALES for the measurement. A metric sitting exactly at
    # its reference scale now contributes exactly its weight.
    normalised = metrics / METRIC_SCALES
    quality = float(SCORE_WEIGHTS @ normalised)

    progress = float(np.clip(progress, 0.0, 1.0))

    # ── TIER 1: hard constraints ──────────────────────────────────────────
    # A run that crashed, left the track, or never finished is INFEASIBLE. It
    # is not a run with a bad score — it is not a valid data point about how
    # the car drives, because the metrics were accumulated over a trajectory
    # that ended in failure.
    #
    # Previously these were flat additive penalties (+3.0 DNF, +3.0 more for
    # off-track) competing on the same axis as the twelve continuous metrics.
    # That is a scalarisation, and it let a run BUY its way out of a crash:
    # a set of gains that tracked the line tightly enough could out-earn the
    # penalty on the quality terms. It also made the objective discontinuous
    # in a way that dominated the tuner — measured, a single DNF in ten tasks
    # moved the aggregate objective by ~0.9, swamping every quality signal.
    #
    # Feasible runs now occupy a band strictly below CONSTRAINT_FLOOR, and
    # infeasible ones strictly above it, so no amount of good driving in the
    # quality terms can promote an infeasible run above a feasible one. Within
    # the infeasible band, ordering still rewards getting further (progress)
    # before failing, which keeps a gradient for the optimiser to climb
    # instead of a flat wall of equally-bad scores.
    if dnf or offtrack:
        severity = DNF_PENALTY + (DNF_OFFTRACK_PENALTY if offtrack else 0.0)
        # Deeper progress -> less bad, but never good enough to cross the floor.
        return float(CONSTRAINT_FLOOR + severity * (1.0 - progress))

    # An unfinished-but-not-DNF run (ran out of steps mid-path) is also not a
    # valid measurement of a lap. Same treatment, scaled by how far it got.
    # `reached_end` is the authoritative "did it finish" signal from the
    # rollout. progress is NOT usable for this: it comes from a bounded
    # nearest-index search that stops short of the final path point, so a
    # fully-completed run typically reports ~0.90, not 1.0. Thresholding on
    # progress therefore marked every successful run infeasible. Fall back to
    # the progress threshold only when the caller can't supply reached_end
    # (e.g. the live car, which has no known path end).
    finished = reached_end if reached_end is not None else (progress >= COMPLETION_THRESHOLD)
    if not finished:
        return float(CONSTRAINT_FLOOR + DNF_PENALTY * (1.0 - progress))

    # ── TIER 2: primary objective — time ──────────────────────────────────
    # time_bonus is optimal_lap_time / actual_time (see rollout_core), so it
    # is 1.0 at the physical limit and decays as the run gets slower. The
    # objective is its complement: 0.0 is a perfect lap, 1.0 is infinitely
    # slow. This is the term that should dominate a FEASIBLE run, and it is
    # in real units — "0.15" means the lap took ~18% longer than physically
    # possible, not an arbitrary composite figure.
    time_cost = float(np.clip(1.0 - time_bonus, 0.0, 1.0))

    # ── TIER 3: quality / smoothness ──────────────────────────────────────
    # Kept as a weighted sum because these genuinely ARE preferences with no
    # natural priority ordering among them, which is exactly the case
    # scalarisation handles well. Scaled down relative to the time objective
    # so it shapes the solution rather than driving it: it should decide
    # between two laps of similar speed, not make a slow-but-smooth lap beat
    # a fast one. This is what kills the hunting exploit — hunting cannot buy
    # lap time, so it now only costs.
    score = TIME_OBJECTIVE_WEIGHT * time_cost + QUALITY_WEIGHT * quality

    # Completion is a precondition now (see tier 1), not something to reward,
    # so the old COMPLETION_BONUS_WEIGHT * progress term is gone: every run
    # reaching this line has progress >= COMPLETION_THRESHOLD.

    if inaccurate_count > 0:
        # OPTIMAL_INACCURATE solves are still usable but less trustworthy;
        # inflate the score proportionally (capped at 5 occurrences -> 50%)
        # rather than rejecting the rollout outright.
        factor = min(5, inaccurate_count) * 0.1
        score = score + abs(score) * factor

    return float(score)


class RolloutMetrics:
    """
    Accumulates the 13 raw score-metric sums one simulation step at a time.

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
        self.steering_reversal_cost = 0.0
        self.accel_reversals = 0
        self.accel_reversal_cost = 0.0
        self._last_sign = 0
        self._last_accel_sign = 0
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
        Accumulate one timestep's contribution to all 13 metrics.

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
                # Magnitude-weighted reversal cost: a flat +1 per sign flip
                # can't tell a ±1.5deg trim wiggle apart from a ±40deg swing,
                # and can't tell a path-demanded direction change (S-bends,
                # slaloms) apart from controller hunting/dithering. Weight by
                # how far the steering command actually swung (previous value
                # -> current value, both magnitudes since they're opposite
                # sign) and square it, consistent with this file's other RMS
                # accumulators (error_cost, yaw_rate_cost, jerk_cost) — a tiny
                # wiggle then contributes almost nothing while a large swing
                # dominates. The 0.02 rad gate is kept only to reject float
                # noise as a sign flip (see docstring below); it is not doing
                # the small-magnitude suppression anymore, that's now the
                # squared weighting's job.
                self.steering_reversals += 1
                delta_swing = abs(u_opt[0]) + abs(self.u_prev[0])
                self.steering_reversal_cost += delta_swing ** 2
            self._last_sign = current_sign

        # Identical construction to the steering block above, applied to
        # u_opt[1] (a_cmd) instead of u_opt[0] (delta_cmd) — see
        # compute_composite_score's accel_reversal_rms docstring for why
        # this exists. Gate threshold is 0.02 m/s^2 rather than 0.02 rad
        # since a_cmd's units/typical magnitude differ from steering's, but
        # serves the same purpose (reject float noise as a sign flip).
        current_accel_sign = int(np.sign(u_opt[1]))
        if current_accel_sign != 0:
            if (self._last_accel_sign != 0 and current_accel_sign != self._last_accel_sign
                    and abs(u_opt[1]) > 0.02):
                self.accel_reversals += 1
                accel_delta_swing = abs(u_opt[1]) + abs(self.u_prev[1])
                self.accel_reversal_cost += accel_delta_swing ** 2
            self._last_accel_sign = current_accel_sign

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

    def finalize(self, progress, time_bonus=0.0, dnf=False, offtrack=False,
                 reached_end=None):
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
        # RMS of magnitude-weighted reversal swings, not a raw count — see
        # compute_composite_score's docstring for why (a raw count can't
        # distinguish a harmless trim wiggle from an aggressive swing, or a
        # path-demanded direction change from controller hunting; and an
        # unnormalised sum would mechanically inflate with rollout length /
        # shrink with DT).
        steering_reversal_rms = float(np.sqrt(self.steering_reversal_cost / n))
        # Reversals-per-step raw-count rate, kept informational only (not
        # scored) — see the "steering_reversals" raw count below for the
        # unnormalised version.
        steering_reversal_rate = self.steering_reversals / n
        # Same construction, applied to a_cmd — see add_step's comment.
        accel_reversal_rms = float(np.sqrt(self.accel_reversal_cost / n))
        accel_reversal_rate = self.accel_reversals / n

        score = compute_composite_score(
            rmse, yaw_rms, smooth_rms, steer_rms, accel_rms,
            self.max_steering, steering_sat_ratio, jerk_rms, self.max_yaw_rate,
            steering_reversal_rms, self.peak_lateral_error, speed_rmse,
            progress=progress, accel_reversal_rms=accel_reversal_rms,
            time_bonus=time_bonus, dnf=dnf, offtrack=offtrack,
            inaccurate_count=self.inaccurate_count, reached_end=reached_end,
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
            "steering_reversal_rms": steering_reversal_rms,
            "steering_reversal_rate": steering_reversal_rate,  # informational-only rate
            "steering_reversals": self.steering_reversals,  # informational-only raw count
            "accel_reversal_rms": accel_reversal_rms,
            "accel_reversal_rate": accel_reversal_rate,  # informational-only rate
            "accel_reversals": self.accel_reversals,  # informational-only raw count
            "peak_lateral_error_m": self.peak_lateral_error,
            "speed_rmse_mps": speed_rmse,
            "max_yaw_rate_radps": self.max_yaw_rate,
            "inaccurate_count": self.inaccurate_count,
            "n_steps": n,
        }