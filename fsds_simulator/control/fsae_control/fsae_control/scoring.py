"""
scoring.py — Live-run copy of fsae_MPCTest/sim/scoring.py

PURPOSE
-------
Scores a LIVE (real/simulated-car) run with the exact same maths the offline
tuner uses to score a simulated rollout, so a number produced on the car is
directly comparable to a number produced by offline_tuner.py.  Without this,
"the sim says 42, the car feels worse" is unfalsifiable.

PARITY — THIS IS A VERBATIM COPY
--------------------------------
compute_composite_score(), RolloutMetrics.add_step() and
RolloutMetrics.finalize() below are copied byte-for-byte (modulo the
settings import, see below) from fsae_MPCTest/sim/scoring.py.  Do NOT
"improve" them here.  Any change to scoring must be made in
fsae_MPCTest/sim/scoring.py first and then re-copied across, exactly like
mpc_core.py mirrors the offline MPC — otherwise live and offline scores stop
being comparable, which defeats the entire point of the file.

The ONE intentional difference: fsae_MPCTest imports SCORE_WEIGHTS and the
bonus/penalty constants from its settings.py, which is not on the live car's
PYTHONPATH.  They are inlined below as module constants instead, and must be
kept numerically identical to fsae_MPCTest/settings.py.

WHAT THE LIVE CAR CANNOT MEASURE
--------------------------------
Two inputs to the offline score have no faithful live equivalent:
  - time_bonus  — needs a known expected lap time / step budget.
  - offtrack    — the offline rollout knows ground-truth track edges.
Both default to 0.0 / False here.  progress and dnf ARE supplied by the
caller where known.  Live scores are therefore comparable to offline scores
in their weighted-metric component; the bonus terms are only comparable when
the caller supplies them.  score_is_partial in the emitted header records
this so a reader can't mistake one for the other.

USED BY
-------
  telemetry_logger.ControlLogger — accumulates per control step and writes the
                                   finalised score as a header on the CSV.
"""

import numpy as np

# ── Inlined from fsae_MPCTest/settings.py — keep numerically identical ───────
# Order MUST match the IDX_* constants below.
# These must sum to ~1.0 (offline_tuner.py asserts this) so the composite
# score's scale stays comparable across runs.
SCORE_WEIGHTS = np.array([
    0.505,  # 0  rmse                  (lateral + heading tracking; primary)
    0.09,   # 1  yaw_rms
    0.065,  # 2  smooth_rms
    0.02,   # 3  steer_rms
    0.015,  # 4  accel_rms
    0.03,   # 5  max_steering
    0.045,  # 6  steering_sat_ratio
    0.045,  # 7  jerk_rms
    0.02,   # 8  max_yaw_rate
    0.05,   # 9  steering_reversal_rms
    0.10,   # 10 peak_lateral_error
    0.015,  # 11 speed_rmse
])

# METRIC_SCALES — each metric is divided by its entry here BEFORE being
# multiplied by its SCORE_WEIGHTS entry, so a weight expresses priority
# rather than silently doing unit conversion as well. Without this, a
# metric's real influence is weight x typical magnitude, which made the
# smoothness/oscillation terms 2-3 orders of magnitude too small to affect
# the outcome (steering_reversal_rms had an effective contribution of
# ~0.0003 despite a nominal weight of 0.05). See fsae_MPCTest/settings.py's
# METRIC_SCALES block for the measurement this came from.
# Order MUST match SCORE_WEIGHTS / the IDX_* constants below.
METRIC_SCALES = np.array([
    0.40,    # 0  rmse
    0.45,    # 1  yaw_rms
    0.30,    # 2  smooth_rms
    0.18,    # 3  steer_rms
    1.50,    # 4  accel_rms
    0.40,    # 5  max_steering
    0.02,    # 6  steering_sat_ratio
    0.30,    # 7  jerk_rms
    1.00,    # 8  max_yaw_rate
    0.015,   # 9  steering_reversal_rms
    0.70,    # 10 peak_lateral_error
    2.30,    # 11 speed_rmse
])

COMPLETION_BONUS_WEIGHT = 0.5
TIME_BONUS_WEIGHT = 0.25
DNF_PENALTY = 3.0
DNF_OFFTRACK_PENALTY = 3.0

# Constrained scoring structure — see fsae_MPCTest/settings.py for the full
# rationale. Summary: the score is three tiers, not one weighted sum.
#   1. Hard constraints (crash/off-track/unfinished) -> above CONSTRAINT_FLOOR,
#      where no amount of good driving in the quality terms can rescue them.
#   2. Primary objective: lap time vs. the path's physical optimum.
#   3. Quality group: the 12 metrics as a weighted sum, scaled to shape the
#      result rather than drive it.
# A weighted sum alone can only reach the convex hull of the trade-off surface,
# which is why a deliberately-hunting set of gains kept outscoring a sane one
# no matter how the weights were set.
CONSTRAINT_FLOOR = 10.0
COMPLETION_THRESHOLD = 0.98
TIME_OBJECTIVE_WEIGHT = 1.0
QUALITY_WEIGHT = 0.35

# Metric index constants — must stay in sync with SCORE_WEIGHTS order
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
    time_bonus=0.0,
    dnf=False,
    offtrack=False,
    inaccurate_count=0,
    reached_end=None,
):
    """
    Single source of truth for the composite performance score.
    Combines the 12 metrics with SCORE_WEIGHTS, applies completion/time
    bonuses, DNF penalties, and the inaccurate-solver factor.
    Lower is better.

    Parameter order here MUST match the IDX_* constants above / the order
    of SCORE_WEIGHTS — the metrics array below is built positionally, not
    by name.
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
        ]
    )
    # Normalise by reference scale before weighting — see METRIC_SCALES above.
    normalised = metrics / METRIC_SCALES
    quality = float(SCORE_WEIGHTS @ normalised)

    progress = float(np.clip(progress, 0.0, 1.0))

    # ── TIER 1: hard constraints ──────────────────────────────────────────
    # Infeasible runs are pushed above CONSTRAINT_FLOOR so no quality score
    # can promote them above a feasible run. Ordering within the band still
    # rewards getting further before failing, to keep a usable gradient.
    if dnf or offtrack:
        severity = DNF_PENALTY + (DNF_OFFTRACK_PENALTY if offtrack else 0.0)
        return float(CONSTRAINT_FLOOR + severity * (1.0 - progress))

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
    # time_bonus is optimal_time / actual_time, so this is "fraction slower
    # than physically possible". NOTE: live runs have no optimal-lap reference,
    # so time_bonus defaults to 0.0 and this term saturates at 1.0 — the live
    # score is flagged score_is_partial=1 for exactly this reason. The quality
    # component below remains directly comparable to an offline score.
    time_cost = float(np.clip(1.0 - time_bonus, 0.0, 1.0))

    # ── TIER 3: quality / smoothness ──────────────────────────────────────
    score = TIME_OBJECTIVE_WEIGHT * time_cost + QUALITY_WEIGHT * quality

    if inaccurate_count > 0:
        factor = min(5, inaccurate_count) * 0.1
        score = score + abs(score) * factor

    return float(score)


class RolloutMetrics:
    """
    Accumulates the 12 raw score-metric sums one control step at a time.

    Verbatim copy of fsae_MPCTest/sim/scoring.py's RolloutMetrics — see this
    module's docstring before changing anything here.
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
        e_y, e_psi : float   Lateral / heading tracking error this step (m, rad).
        r          : float   Measured yaw rate (rad/s).
        u_opt      : array (2,)  Applied [delta_cmd (rad), a_cmd (m/s2)].
        v_target, v_actual : float  Target speed / measured speed (m/s).
        u_max_steer : float  Vehicle's max_steer bound (for saturation check).
        solver_failed : bool  True if the MPC solve failed this step.
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
                delta_swing = abs(u_opt[0]) + abs(self.u_prev[0])
                self.steering_reversal_cost += delta_swing ** 2
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

    def finalize(self, progress, time_bonus=0.0, dnf=False, offtrack=False,
                 reached_end=None):
        """
        Normalise accumulated sums to RMS/ratio metrics and compute the
        final composite score.
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
        steering_reversal_rms = float(np.sqrt(self.steering_reversal_cost / n))
        steering_reversal_rate = self.steering_reversals / n

        score = compute_composite_score(
            rmse, yaw_rms, smooth_rms, steer_rms, accel_rms,
            self.max_steering, steering_sat_ratio, jerk_rms, self.max_yaw_rate,
            steering_reversal_rms, self.peak_lateral_error, speed_rmse,
            progress=progress, time_bonus=time_bonus, dnf=dnf, offtrack=offtrack,
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
            "steering_reversal_rate": steering_reversal_rate,
            "steering_reversals": self.steering_reversals,
            "peak_lateral_error_m": self.peak_lateral_error,
            "speed_rmse_mps": speed_rmse,
            "max_yaw_rate_radps": self.max_yaw_rate,
            "inaccurate_count": self.inaccurate_count,
            "n_steps": n,
        }
