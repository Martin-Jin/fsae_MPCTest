"""
settings/scoring.py — the composite score: per-metric scaling
(METRIC_SCALES), weights (SCORE_WEIGHTS), the validation suite, and the
constrained (hard-floor / primary-objective / quality-group) scoring
structure's own constants.
"""
import numpy as np

# ------------------------------------------------------------------------------
# COST FUNCTION SCORING WEIGHTS
# ------------------------------------------------------------------------------

# METRIC_SCALES — "What counts as a NORMAL amount of each thing being
# measured?"
# Each of the 13 metrics below is divided by its entry here before being
# multiplied by its SCORE_WEIGHTS entry. That turns every metric into a
# roughly unitless "multiples of a typical value" number, so a weight of
# 0.05 next to a weight of 0.015 really does mean "this matters ~3x more".
#
# WHY THIS EXISTS (measured)
# ---------------------------
# Without it, a metric's real influence is weight x typical magnitude, not
# weight — and the 13 metrics have wildly different natural magnitudes
# (steering_reversal_rms ~0.007, accel_rms ~1.3, speed_rmse ~2.5). A probe
# batch of 6 hand-constructed gain sets spanning known failure modes found
# that this made the score effectively SINGLE-objective:
#
#   Comparing a deliberately-hunting gain set against a neutral baseline,
#   the total score difference of -0.2605 decomposed as
#       rmse                            -0.2031
#       peak_lateral_error              -0.0618
#       ALL TEN other metrics combined  +0.0064   <-- 3% of the tracking term
#
#   Every anti-hunting metric DID register the misbehaviour (steering_sat_
#   ratio was 7x higher, yaw_rms higher), but their combined contribution
#   was noise. steering_reversal_rms, nominally the 4th-largest weight at
#   0.05, had an effective contribution of 0.0003 — it could not influence
#   the outcome at all. The hunting set therefore SCORED BETTER than the
#   baseline purely by tracking the line more tightly.
#
# That is also why hand-raising yaw_rms 0.06 -> 0.09 (see the SCORE_WEIGHTS
# comments below) failed to suppress the oscillation seen in live logs: it
# moved that metric's effective contribution by about 0.001. And it explains
# the ~10x spread in tuned gains across historical runs — with ~97% of the
# discrimination coming from two correlated tracking metrics, CMA-ES roamed
# freely in every other dimension because they cost nothing.
#
# HOW THESE NUMBERS WERE CHOSEN
# -----------------------------
# Median-ish observed magnitude across CLEAN (no-DNF, non-pathological)
# rollouts in both planner and oracle modes, rounded to 1-2 significant
# figures so they read as deliberate reference points rather than
# false-precision measurements. They are REFERENCE SCALES, not targets or
# limits — a metric equal to its scale contributes exactly its weight.
#
#   - Set one too LARGE: that metric is suppressed, contributes less than
#     its weight implies.
#   - Set one too SMALL: that metric is amplified and can dominate.
#   - Typical adjustment: only change these if a metric's typical magnitude
#     genuinely shifts (e.g. after a plant/planner change). To change
#     PRIORITY, change SCORE_WEIGHTS instead — that is now what it means.
#
# Order MUST match SCORE_WEIGHTS / the IDX_* constants in sim/scoring.py.
METRIC_SCALES = np.array(
    [
        0.40,    # 0  rmse                   (clean runs 0.26-0.52)
        0.45,    # 1  yaw_rms                (0.39-0.56 rad/s)
        0.30,    # 2  smooth_rms             (0.21-0.30 in clean, higher when jerky)
        0.18,    # 3  steer_rms              (0.168-0.188 rad — very stable)
        1.50,    # 4  accel_rms              (1.3-1.8 m/s^2)
        0.40,    # 5  max_steering           (0.34-0.43 rad)
        0.02,    # 6  steering_sat_ratio     (0.001-0.06; small but real range)
        0.30,    # 7  jerk_rms               (0.24-0.31 in clean runs)
        1.00,    # 8  max_yaw_rate           (0.84-1.44 rad/s)
        0.015,   # 9  steering_reversal_rms  (0.0002-0.027; the worst-scaled
                 #    metric of the 12 — near-zero effective contribution at
                 #    a naively larger scale, hence the small value here)
        0.70,    # 10 peak_lateral_error     (0.57-0.82 m)
        2.30,    # 11 speed_rmse             (1.86-2.83 m/s)
        0.08,    # 12 accel_reversal_rms     (0.037-0.123 measured directly on
                 #    VALIDATION_SUITE at current tuned weights, use_planner=False)
    ],
    dtype=float,
)

# SCORE_WEIGHTS — "How much does each aspect of driving quality matter when
# grading a test run?"
# Every test run is graded on 12 different things (see the list below), and
# each grade is multiplied by its corresponding weight here, then added
# together into one final score (lower is better). This list is what the
# automated tuner is actually trying to minimise — it is the definition of
# "good driving" for this whole project.
#
# These weights are applied to each metric AFTER it has been divided by its
# METRIC_SCALES entry above. That normalisation is what makes a weight mean
# what it says: without it, weights would hit each metric's *raw* value, and
# since the 13 metrics have wildly different natural magnitudes
# (mixed m²/rad² RMS terms, radians, m/s², unitless ratios), a metric's real
# influence was weight x typical magnitude rather than weight. See the
# METRIC_SCALES block above for the measurement that showed this had made the
# score effectively single-objective (~97% of discrimination from rmse +
# peak_lateral_error alone).
#
# So: to change PRIORITY, change the weight here. To correct for a metric's
# typical magnitude having genuinely shifted, change METRIC_SCALES instead.
# Those are now two separate jobs, which is the point of the split.
#
# offline_tuner.py still asserts these 12 numbers sum to ~1.0, which keeps the
# composite score's overall scale stable/comparable across tuning runs. With
# normalisation in place, summing to 1.0 now DOES carry real meaning: a run
# where every metric sits exactly at its reference scale scores 1.0 before
# bonuses/penalties. If you adjust one weight, take the offsetting change
# from another so the sum is preserved.
#
# What each of the 12 numbers grades, in order:
#   0: rmse               — how far off the racing line the car drives on
#                            average (the single most important measure —
#                            has the largest weight for that reason)
#   1: yaw_rms             — how much the car's direction wobbles/oscillates
#   2: smooth_rms          — how jerky the steering/throttle changes are
#                            step-to-step
#   3: steer_rms            — how much steering effort is used overall
#   4: accel_rms            — how much acceleration/braking effort is used
#                            overall
#   5: max_steering         — the single sharpest steering movement made
#                            during the run
#   6: steering_sat_ratio   — how often the car steers at (or very near) its
#                            maximum possible steering angle
#   7: jerk_rms             — how abruptly steering/throttle changes speed
#                            up or slow down (a "smoothness of smoothness"
#                            measure)
#   8: max_yaw_rate         — the fastest the car's direction ever spun
#                            during the run
#   9: steering_reversal_rms — how large the car's steering direction
#                            flips are, magnitude-weighted (an RMS of each
#                            reversal's swing size, sqrt(Σ swing² / n
#                            steps)), not a flat per-flip count. A tiny
#                            back-and-forth trim wiggle (e.g. ±1.5deg on a
#                            straight) contributes almost nothing, while a
#                            large aggressive swing (e.g. ±40deg) dominates
#                            — this is what actually distinguishes
#                            controller hunting/dithering from a twisty
#                            path (S-bends, slaloms) legitimately demanding
#                            more frequent-but-small direction changes,
#                            which a flat count could not tell apart. The
#                            raw reversal count and its per-step rate are
#                            still reported separately (informational only)
#                            alongside this in performance_stats.py.
#  10: peak_lateral_error   — the single worst moment the car was off the
#                            racing line, even briefly
#  11: speed_rmse           — how far off the intended speed the car drives
#                            on average
#  12: accel_reversal_rms   — how large the car's throttle/brake direction
#                            flips are, magnitude-weighted, same construction
#                            as steering_reversal_rms above but applied to
#                            a_cmd instead of delta_cmd. Distinguishes a car
#                            that's genuinely flip-flopping between throttle
#                            and brake from one that's simply using a lot of
#                            accel/brake effort (accel_rms, metric 4) or
#                            changing it jerkily but monotonically (jerk_rms,
#                            metric 7) — neither of those isolates a sign
#                            flip the way this does.
#
# Increasing any one weight makes the tuner prioritise fixing that aspect
# of driving more, even if it makes other aspects slightly worse.
#   - Typical adjustment: change a weight by roughly 20-30% of its own value
#     at a time, then re-tune and compare.
SCORE_WEIGHTS = np.array(
    [
        0.505,  # 0  rmse                    (lateral + heading tracking; primary)
        0.09,   # 1  yaw_rms                 (live standalone-ROS test data showed
                #    yaw_rate swinging +0.9/-1.1 rad/s within a few hundred ms in
                #    corners; this weight gives CMA-ES real pressure to avoid that
                #    via the composite score, so oscillatory yaw actually costs the
                #    tuner something. Offset by the trims below.)
        0.040,  # 2  smooth_rms               (kept modest because this metric is a
                #    blunt instrument for accel/brake flip-flopping — it reacts via
                #    (u_opt-u_prev)^2 without isolating reversal count/magnitude the
                #    way accel_reversal_rms below does; weight funds that metric
                #    instead)
        0.02,   # 3  steer_rms
        0.015,  # 4  accel_rms               (kept above the floor needed to give
                #    CMA-ES a real gradient on throttle/brake effort)
        0.03,   # 5  max_steering
        0.045,  # 6  steering_sat_ratio       (kept modest relative to yaw_rms/
                #    steering_reversal_rms — less directly related to the
                #    oscillation/chatter symptom those two terms target)
        0.020,  # 7  jerk_rms                 (kept modest: jerk_cost reacts to a
                #    sign flip's large du but conflates it with any other jerky-
                #    but-monotonic accel change, unlike the dedicated reversal
                #    metrics)
        0.02,   # 8  max_yaw_rate
        0.05,   # 9  steering_reversal_rms  (live standalone-ROS test data showed
                #    steering sign reversals almost every ~0.05s tick, worst in
                #    corners — this metric directly measures that "hunting"
                #    behaviour, so it carries a meaningful weight to discourage it.
                #    Constructed as a magnitude-weighted RMS (see sim/scoring.py)
                #    so a tiny trim wiggle and a path-demanded direction change
                #    don't score the same as an aggressive hunting swing.)
        0.10,   # 10 peak_lateral_error
        0.015,  # 11 speed_rmse              (same rationale as accel_rms above —
                #    kept above the floor needed for a usable CMA-ES gradient)
        0.05,   # 12 accel_reversal_rms      (the identical
                #    magnitude-weighted-swing construction as steering_reversal_rms
                #    above, applied to a_cmd instead of delta_cmd. Exists because
                #    live logs showed persistent throttle/brake sign-flip chatter
                #    with NO corresponding cost term anywhere in the score:
                #    smooth_rms/jerk_cost react to a_cmd's tick-to-tick delta but
                #    can't distinguish a reversal from any other jerky-but-same-sign
                #    change, and nothing else even looks at u_opt[1]'s sign. Given
                #    the same weight as steering_reversal_rms since the two are the
                #    same behaviour on the two different actuators; funded by
                #    trimming smooth_rms/jerk_rms (see their comments) rather than
                #    the tracking terms. METRIC_SCALES entry is a PLACEHOLDER (1.0)
                #    until measured on VALIDATION_SUITE — see that
                #    entry's comment.)
    ],
    dtype=float,
)

# VALIDATION_SUITE — "Which practice tracks does the tuner actually test the
# car on?"
# The tuner has a larger library of possible practice tracks (defined in
# offline_tuner.py) covering different corner types (sharp turns, S-bends,
# hairpins, etc.), but only tests against the tracks listed here (the
# commented-out ones are skipped to keep tuning faster). The tuner tries to
# find one set of driving weights that works reasonably well across *all*
# of the tracks listed here at once, not just one.
#   - Add a track (uncomment or add a name): tuning takes longer per test,
#     but the result generalises to more corner shapes and is less likely
#     to be "overfit" to only the tracks currently listed.
#   - Remove a track: faster tuning, but risk producing weights that drive
#     well on the remaining tracks and poorly on the removed one.
#   - Typical adjustment: add or remove one track at a time and observe
#     how much longer/shorter tuning runs take before removing/adding more.
VALIDATION_SUITE = [
    "PATH_SPIRAL",
    "PATH_SUDDEN_TURN",
    "PATH_HAIRPIN",
    "PATH_FS_CORNER",
    "PATH_MICRO_SLALOM",
    # "PATH_OFFSET_CHICANE",
    # "PATH_S_BEND",
    # "PATH_MIXED",
    # "PATH_CHICANE",
    # "PATH_ACCELERATION"
]

# ------------------------------------------------------------------------------
# PERFORMANCE BONUS WEIGHTS
# ------------------------------------------------------------------------------

# COMPLETION_BONUS_WEIGHT / TIME_BONUS_WEIGHT — NO LONGER USED BY THE SCORE.
# Under the constrained scoring structure (see CONSTRAINT_FLOOR below),
# completion is a hard requirement rather than a reward, and time is the
# primary objective rather than a bonus. Both constants are retained only so
# the live scoring copy's CSV header and tuning-history logging keep their
# existing fields. Changing them has no effect on the score — use
# TIME_OBJECTIVE_WEIGHT / QUALITY_WEIGHT instead.
#
# Historical description follows.
#
# COMPLETION_BONUS_WEIGHT — "How much of a reward (score reduction) does the
# car get simply for finishing the track?"
# This is subtracted from the score in proportion to how much of the track
# was completed (fully finishing = the full bonus subtracted; finishing
# half the track = half the bonus). It exists to make sure "finish the
# track" is always worth pursuing even if driving isn't perfect along the
# way.
#   - Increase it: tuner favours weights that reliably finish tracks, even
#     if the driving along the way is a bit rougher.
#   - Decrease it: tuner cares relatively more about precision/smoothness
#     than simply finishing.
#   - Typical adjustment: change by 0.1-0.2 at a time.
COMPLETION_BONUS_WEIGHT = 0.5

# TAIL_QUANTILE — "When grading a candidate across all the practice tracks,
# how much should its WORST track count?"
# The tuner scores each candidate on every track x starting-condition
# combination (10 tasks by default), then combines them as
#     0.7 * weighted_average + 0.3 * quantile(scores, TAIL_QUANTILE)
# The second term is there so the tuner can't pick weights that drive well on
# average but crash on one particular track.
#
# A hard worst-case (TAIL_QUANTILE = 1.0) is too brittle: a DNF adds a flat
# +3.0 (+6.0 off-track), so ONE unlucky task out of ten shifts the objective
# by ~0.9 and swamps all thirteen continuous quality metrics. Measured effect:
# a plausible hand-picked gain set scored 3rd-WORST of six — below two
# deliberately pathological sets — purely because one of its ten tasks DNF'd.
#   - 1.0  = the single worst task decides the tail term entirely.
#   - 0.8  = around the 2nd-worst of 10 tasks. One bad task still hurts a
#            lot; two bad tasks hurt much more.
#   - 0.5  = the median; effectively stops punishing rare failures at all
#            (not recommended — that's what DNF_PENALTY is for).
#   - Typical adjustment: 0.05-0.1 at a time. Lower it if tuning results
#     swing wildly between runs; raise it if the tuner starts accepting
#     weights that reliably fail one track.
TAIL_QUANTILE = 0.8

# ==============================================================================
# CONSTRAINED SCORING STRUCTURE
# ==============================================================================
# A single weighted sum of 13 metrics plus additive bonuses and penalties is
# "linear scalarisation", and it has a structural limit: a
# weighted sum can only ever reach solutions on the CONVEX HULL of the
# trade-off surface. If that surface is non-convex — normal for vehicle
# dynamics — entire regions of good behaviour are unreachable by ANY weight
# vector, so no amount of weight tuning finds them.
#
# Measured evidence: a deliberately-hunting set of gains outscored a sane one
# because it tracked the line more tightly, and it kept winning even after
# METRIC_SCALES made the smoothness terms bite (normalisation amplified the
# tracking terms too). The hunting set is genuinely better on the dominant
# term, so re-weighting cannot fix it.
#
# The score is now three tiers instead of one sum:
#   1. HARD CONSTRAINTS  — crash / off-track / didn't finish. Infeasible, and
#      pushed above CONSTRAINT_FLOOR where no quality score can rescue them.
#   2. PRIMARY OBJECTIVE — lap time vs. the path's physical optimum.
#   3. QUALITY GROUP     — the 13 metrics, kept as a weighted sum (they really
#      are preferences), scaled to shape rather than drive the result.

# CONSTRAINT_FLOOR — "the line between a valid run and a failed one."
# Any run that crashes, leaves the track, or doesn't finish scores at least
# this much; any run that completes scores less. Set well above the worst
# realistic feasible score so the two bands can never overlap — that
# separation is the whole point, and it's what stops a tight-tracking run
# from buying its way out of a crash.
#   - Raise it: more headroom if feasible scores ever grow past it.
#   - Lower it: risks the bands touching. Don't go below ~2.0.
CONSTRAINT_FLOOR = 10.0

# COMPLETION_THRESHOLD — "how much of the path counts as finishing?"
# Below this fraction the run is treated as infeasible (tier 1). Slightly
# below 1.0 because arc-length accumulation can stop a hair short of the end
# on a completed lap.
COMPLETION_THRESHOLD = 0.98

# TIME_OBJECTIVE_WEIGHT / QUALITY_WEIGHT — the tier-2 vs tier-3 balance.
# time_cost is in [0,1] ("fraction slower than physically optimal") and the
# quality term is ~O(1) when metrics sit at their reference scales, so these
# are directly comparable. Time leads at 1.0 and quality shapes at 0.35.
#   - Raise QUALITY_WEIGHT: smoother but slower driving.
#   - Lower it: faster but rougher; below ~0.15 the smoothness terms stop
#     mattering again and hunting can creep back in.
#   - Typical adjustment: 0.05 at a time on QUALITY_WEIGHT.
TIME_OBJECTIVE_WEIGHT = 1.0

QUALITY_WEIGHT = 0.35

# TIME_BONUS_WEIGHT — "How much of a reward (score reduction) does the car
# get for finishing quickly?"
# Similar to the completion bonus above, but rewards speed specifically —
# a run that finishes faster gets more of this bonus subtracted from its
# score.
#   - Increase it: tuner favours weights that drive faster overall, even
#     if that costs some precision.
#   - Decrease it: tuner cares relatively more about precision/smoothness
#     than raw speed.
#   - Typical adjustment: change by 0.05-0.1 at a time.
TIME_BONUS_WEIGHT = 0.25
