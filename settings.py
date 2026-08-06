"""
settings.py — Central Configuration File

PURPOSE
-------
This is the one file you should look at to change how the simulator and
the offline tuner behave, without touching any of the maths or control
code elsewhere.

Nothing physical about the car (its weight, tyre grip, engine power etc.)
lives here — that's all in vehicle_physics.py. This file only controls
how the *controller* is scored, tuned, and configured to drive.
"""

import numpy as np
from sim.sim_track import TRACK_HALF_WIDTH

# ==============================================================================
# GENERAL SYSTEM CONFIGURATION (TUNER + SIMULATOR)
# ==============================================================================

# N_HORIZON — "How far ahead does the car plan?"
# The controller doesn't just react to what's happening right now — it plans
# a short sequence of future steering/throttle moves and only acts on the
# first one, then re-plans next tick. This number is how many 0.05-second 
# (since simulator runs at 20Hz) steps ahead it plans each time 
# (25 steps = 1.25 seconds of look-ahead).
#   - Increase it: the car "sees" further ahead, which can smooth out
#     reactions to corners it hasn't reached yet, but each planning step
#     takes noticeably longer to compute (the difficulty roughly squares).
#   - Decrease it: faster to compute, but the car becomes more short-sighted
#     and can react late to corners.
#   - Typical adjustment: change by 5 steps (0.25 s) at a time. Must match
#     N_horizon in simulation.py and N in control_utils.py exactly, or the
#     weights tuned here won't behave the same on the real car.
N_HORIZON = 25

# USE_PLANNER — "Does the tuner pretend to have real cone-vision, or cheat
# and use the perfect track outline?"
# True  = the tuner simulates a car that can only see nearby cones and has
#         to build its own idea of the track from them (like the real car).
#         This is slower but tests the whole system, including mistakes the
#         perception/planning code might make.
# False = the tuner gives the car the exact, perfect racing line to follow.
#         Much faster, useful for quickly testing whether the driving style
#         itself (speed, smoothness) is good, but won't catch planner bugs.
# Recommendation: leave as True. Only set False temporarily if you want much
# faster tuning runs and are only tweaking driving feel, not perception.
USE_PLANNER = True

# DELAY_STEPS — "How much lag is there between the car deciding to steer and
# the wheels actually moving?"
# Real hardware (radios, motors, computers) has a small delay before a
# command takes effect. Each unit here is one 0.05 s simulation step. Set
# this above 0 to make the simulator more pessimistic/realistic if you know
# your real car has noticeable lag; leave at 0 for an "ideal" simulation.
#   - Increase it: makes the simulated car more cautious/twitchy to
#     compensate for pretend lag — good for testing robustness.
#   - Adjustment: change by 1 step (0.05 s) at a time; 2-4 steps
#     (0.1-0.2 s) is a realistic amount of lag for most small robots.
DELAY_STEPS = 1

# DELAY_JITTER_STEPS — "Is the lag always exactly the same, or does it vary?"
# DELAY_STEPS above is a single fixed number, and predict_ahead() compensates
# for it EXACTLY — the simulated controller knows the lag perfectly. The real
# car never does: it estimates the lag from a pose timestamp, and its control
# loop jitters. Measured on live standalone-ROS telemetry
# (mpc_standalone_control_1785976976.csv): loop period median 0.0498 s but
# p99 0.0741 s and max 0.1205 s, i.e. jitter std ~0.0092 s = ~0.18 steps.
# With this at 0 the tuner is optimising against a delay model that is
# strictly easier than reality, which is exactly how a set of weights can
# score well offline and still wobble on the car.
# This value is the standard deviation, in steps, of the error between the
# TRUE delay applied to the plant and the delay the controller THINKS it has
# (i.e. how many commands predict_ahead() rolls forward). The plant's own
# delay stays DELAY_STEPS; only the controller's belief is perturbed.
#   - 0.0 : perfect knowledge (previous behaviour, optimistic).
#   - 0.2 : roughly matches the measured live loop jitter. Recommended.
#   - >0.5: pessimistic; useful for robustness testing.
# Rollouts stay deterministic — the perturbation is drawn from a seeded RNG
# (DELAY_JITTER_SEED) so CMA-ES still sees a repeatable score per candidate.
DELAY_JITTER_STEPS = 0.2

# DELAY_JITTER_SEED — fixed seed for the delay-jitter draw above. Keeping it
# fixed is what lets the tuner compare two candidate weight sets fairly: both
# see the identical sequence of delay perturbations, so a score difference is
# attributable to the weights and not to luck. Change it only to check that a
# tuned result isn't overfitted to one particular jitter sequence.
DELAY_JITTER_SEED = 12345

# ------------------------------------------------------------------------------
# SLAM / localisation noise
# ------------------------------------------------------------------------------
# "Does the car know exactly where it is?"
#
# In FSDS it currently does, perfectly. The simulator has no real SLAM: the
# `sim_perception` node republishes FSDS's ground-truth `/fsds/testing_only/odom`
# straight onto `/fsae/slam/car_position`, and the cone map is a latched oracle
# map cropped to a forward window. The only realism is limited sensor RANGE.
# This offline rollout mirrors that — it feeds the exact plant state back into
# the planner and the tracking-error maths.
#
# The REAL car's pose comes from actual SLAM (ZED visual odometry +
# cone_mapper), which jitters frame to frame and drifts slowly. That error
# lands directly in e_y/e_psi, which is what the MPC steers on, so a
# noise-free pose makes the tuner blind to weights that are fragile under
# localisation error.
#
# IMPORTANT — what this is NOT for. It does not explain the steering chatter
# seen in mpc_standalone_control_1785976976.csv. That log came from FSDS,
# where localisation is already perfect (ground-truth odom, see above), so
# pose noise cannot have caused it. The measured cause of that chatter was the
# steering slew limit (the command sat pinned on du_max for 41% of steps); the
# secondary contributor was delay-estimation jitter, which comes from control
# loop timing rather than localisation and so exists in FSDS too. Don't reach
# for this knob to reproduce that behaviour — it won't.
#
# The noise below is applied ONLY to the pose the controller/planner SEE. The
# true plant state still drives the physics and the score, exactly like real
# SLAM error: the car is punished for where it actually ends up, not for where
# it thought it was.
#
# DEFAULT IS OFF, deliberately. The platform currently being validated against
# is FSDS, which has no localisation error, so enabling this by default would
# make offline scores pessimistic relative to the very runs they're compared
# to. Turn it ON when tuning for the real car, or to check that a candidate
# weight set doesn't fall apart once the pose stops being perfect.
SLAM_NOISE_ENABLED = False

# Two components, because they behave differently and the controller reacts to
# them differently:
#
#  1. JITTER — zero-mean, independent every step ("white"). This is the one
#     that provokes chattering: it moves e_y/e_psi randomly each tick, and a
#     controller with too little damping chases it. Sub-centimetre per-frame
#     jitter is typical of a well-behaved visual-odometry front-end.
#  2. DRIFT/BIAS — slowly-varying, correlated over seconds. This is what real
#     SLAM does between loop closures: the estimate wanders off and comes
#     back. It produces a slow steady-state offset from the true centreline
#     rather than chatter.
#
# Units are metres for position, radians for yaw.
#   - Increase jitter: more chatter pressure; the tuner will favour smoother,
#     better-damped weights.
#   - Increase drift: tests robustness to a mis-localised car.
#   - Typical adjustment: change by ~2x at a time and re-check that rollouts
#     still complete without DNF.
SLAM_POS_JITTER_STD = 0.02          # m,   per-step white noise on x/y (2 cm)
SLAM_YAW_JITTER_STD = np.radians(0.3)   # rad, per-step white noise on yaw (0.3 deg)

SLAM_POS_DRIFT_STD = 0.05           # m,   std of the slow position drift (5 cm)
SLAM_YAW_DRIFT_STD = np.radians(0.5)    # rad, std of the slow yaw drift (0.5 deg)

# SLAM_DRIFT_TAU — how many SECONDS the drift takes to wander appreciably.
# Implemented as a first-order (Ornstein-Uhlenbeck) random walk that is pulled
# back toward zero, so the estimate wanders and self-corrects instead of
# running away over a long rollout. 5 s is a reasonable stand-in for the
# timescale between loop closures / re-observations.
SLAM_DRIFT_TAU = 5.0

# SLAM_NOISE_SEED — fixed so rollouts stay reproducible and CMA-ES compares
# candidate weight sets against the identical noise sequence (same rationale
# as DELAY_JITTER_SEED). Change it only to check a tuned result isn't
# overfitted to one particular noise realisation.
SLAM_NOISE_SEED = 24680
# Note: rollout_core.py now predicts the state forward through the commands
# already queued (predict_ahead()) before solving, so the MPC no longer
# reacts to a stale x0 at DELAY_STEPS > 0 — the large-oscillation/DNF
# behavior this note used to warn about is fixed. Validated across
# DELAY_STEPS 1-8 and several initial-offset perturbations on every
# SYNTHETIC_PATHS track without DNF or added oscillation. Still left at 0
# by default since it's a deliberate "how much lag do I want to simulate"
# knob, not because delay is unsafe to enable.

# MAX_FAILS — "How many times in a row can the maths solver fail before we
# give up on this test run?"
# Occasionally the underlying optimisation (the maths that decides steering/
# throttle) can fail to find an answer in time. One failure isn't a big
# deal — the car just repeats its last command. But many in a row usually
# means something is badly wrong (bad weights, impossible situation), so the
# run is abandoned as a "Did Not Finish" (DNF).
#   - Increase it: more tolerant of temporary solver hiccups, but risks
#     letting a genuinely broken run continue for longer before giving up.
#   - Decrease it: fails faster/stricter.
#   - Typical adjustment: change by 1-2 at a time. 5 is a sensible default.
MAX_FAILS = 5

# OFFTRACK_LIMIT — "How far sideways off the centre of the track can the car
# go before we count it as having left the track?"
# Calculated automatically as 1.3× the track's half-width, i.e. a bit more
# than the distance from the centreline to the cones — the car has to be
# meaningfully outside the cone boundary, not just close to it, to be
# flagged. You normally shouldn't need to touch this directly; if you want
# to change it, change TRACK_HALF_WIDTH in sim_track.py instead, which also
# affects cone placement.
OFFTRACK_LIMIT = TRACK_HALF_WIDTH * 1.3  # Lateral error threshold for DNF (m)

# DT — "How often does the car make a new decision?"
# 0.05 seconds = 20 times per second (20 Hz). This must match the real
# controller's update rate and the physics simulation's timestep exactly,
# or the tuned numbers will not behave the same on the real car. Do not
# change this unless you are also changing the real controller's timer
# rate and understand the consequences — it affects almost every other
# calculation in the project.
DT = 0.05

# ------------------------------------------------------------------------------
# Cost function weights (for simulator only)
# ------------------------------------------------------------------------------
# These three lists are the "driving personality" of the car — how much it
# cares about being exactly on the line vs. driving smoothly vs. saving
# steering effort, etc. You do not need to understand the numbers
# individually: they are not meant to be hand-edited. Instead, run
# offline_tuner.py, let it search for a few minutes to hours, and paste the
# three lists it prints out at the end here, replacing the old ones.
#
# If you do want to nudge one manually: each list has one number per "thing
# the car cares about" (see bicycle_model.py's STATE VECTOR comment for what
# each position in Q_diag means). Bigger number = the car tries harder to
# fix that particular error, at the cost of everything else. Change any
# single number by no more than 20-30% at a time and re-test — small changes
# can have surprisingly large effects because they interact with each other.
# Q_diag[3] (yaw-rate/e_psi_dot damping) manually corrected 2026-08-05: live
# standalone-ROS test data (mpc_standalone_control_*.csv) showed steering
# sign-reversal chatter almost every ~0.05s tick, worst in corners (steer
# swinging +-40-57 deg, yaw_rate swinging +0.9/-1.1 rad/s within a few
# hundred ms), but present even on near-straight sections with e_psi ~0-3 deg.
# The old value (0.1009...) was ~42:1 smaller than Q_diag[2] (heading error)
# and ~65:1 smaller than Q_diag[4] (speed error) — by far the smallest of the
# five active Q entries, so the controller had almost no cost on the yaw rate
# it uses to correct heading, a classic recipe for state-feedback overshoot/
# oscillation. This is a large jump rather than the usual 20-30% nudge
# because the prior value was disproportionately small, not just mistuned —
# a small nudge would have left the same qualitative imbalance. Raised to
# 2.5: just above Q_diag[1] (e_y_dot, 2.4068) so yaw-rate damping is no
# longer the smallest term, while staying below Q_diag[2]/Q_diag[4] so
# heading/speed tracking aren't sacrificed outright. CMA-ES should still
# re-tune this properly in a full run — this is a manual corrective starting
# point, not a final value.
Q_diag      = [5.732257553913991, 1.185291265595073, 5.336061825548227, 8.551974642936917, 2.347470167485916, 0.0, 0.0, 0.0]
R_diag      = [1.3813886877086599, 2.058720365852702]
R_rate_diag = [3.5730821419788663, 4.207449882607239]


# ==============================================================================
# TUNER ENGINE & CONSTRAINT SETTINGS
# ==============================================================================

# ------------------------------------------------------------------------------
# DNF (DID-NOT-FINISH) PENALTY CONFIGURATION
# ------------------------------------------------------------------------------

# DNF_PENALTY — "How harshly do we punish a test run where the car never
# finishes the track?"
# This is a flat number added to the run's score if it didn't finish (lower
# score is always better in this project, so a penalty makes the score
# worse/bigger). Without this, the tuner might discover it can get a
# deceptively good-looking score by having the car sit still or crawl very
# slowly and carefully forever without ever finishing.
#   - Increase it: the tuner becomes more strongly biased toward "finish the
#     lap, whatever it takes" over "drive perfectly but risk not finishing."
#   - Decrease it: the tuner cares more about precision/smoothness even if
#     that occasionally means not finishing.
#   - Typical adjustment: change by 0.5-1.0 at a time.
DNF_PENALTY = 3.0

# DNF_OFFTRACK_PENALTY — same idea as above, but specifically an *extra*
# penalty added on top of DNF_PENALTY if the reason the car didn't finish
# was that it left the track (as opposed to, say, running out of time).
# This lets you punish "left the track" more harshly than "just too slow."
#   - Typical adjustment: change by 0.5-1.0 at a time, same as DNF_PENALTY.
DNF_OFFTRACK_PENALTY = 3.0


# ------------------------------------------------------------------------------
# SOLVER SETTINGS FOR HEADLESS ROLLOUTS
# ------------------------------------------------------------------------------

# ROLLOUT_EPS — "How precise does the maths solver need to be during
# automated tuning?"
# A smaller number means the solver has to find a more exact answer before
# it's satisfied, which takes longer. During tuning, thousands of test runs
# happen, so a slightly looser (larger) tolerance here is used to make each
# one faster, at a very small cost to accuracy — the difference is not
# noticeable in how the car actually drives.
#   - Decrease it (more precise): slower tuning, marginally more accurate
#     results.
#   - Increase it (less precise): faster tuning, but if raised too much the
#     car's simulated driving in the tuner may not match how it actually
#     drives.
#   - Typical adjustment: change by a factor of 2-10x at a time (e.g. from
#     1e-4 to 5e-4), since this value works on an exponential/scientific
#     scale, not a simple linear one.
ROLLOUT_EPS = 1e-4

# ROLLOUT_MAX_ITER — "How many attempts does the solver get to find an
# answer before giving up for this step, during tuning?"
# If the solver can't find a good answer within this many internal
# attempts, it gives up for that step (which may count toward MAX_FAILS).
#   - Increase it: solver gets more chances to find an answer, runs may be
#     slightly slower but more likely to succeed on hard corners.
#   - Decrease it: faster but more likely to give up on tricky moments.
#   - Typical adjustment: change by 1000-2000 at a time.
ROLLOUT_MAX_ITER = 8000

# Graceful shutdown flag: set by SIGINT handler; checked each CMA generation.
# (Internal bookkeeping — not a setting you should change.)
_stop_requested = False

# MAX_EVALS — "How long should the automated tuner run for before stopping
# and giving you its best answer?"
# This is the total number of real test-drives the tuner is allowed to run
# across the whole tuning session before it must stop and report its best
# result. A "real test-drive" here means one full attempt at one of the
# validation tracks — the tuner also runs many cheaper approximate guesses
# in between, so actual wall-clock time is not directly proportional to
# this number, but roughly is.
#   - Increase it: tuner searches for longer and will likely (but not
#     guaranteed to) find better driving weights, at the cost of more time
#     (minutes to hours depending on your computer).
#   - Decrease it: faster but rougher tuning results, useful for quick
#     iteration while testing changes to the tracks or scoring.
#   - Typical adjustment: double or halve it (e.g. 2500 → 5000 or → 1250)
#     to meaningfully change tuning time; small changes won't be noticeable.
MAX_EVALS = 2500

# PATH_N_POINTS — "How finely detailed are the practice tracks the tuner
# drives on?"
# Each synthetic test track is built from a smooth curve and then broken
# into this many small dots/points for the car to follow. More points =
# smoother, more precise track shape, but slightly more computation per
# test run. This is unrelated to MAX_EVALS/tuning time budget.
#   - Increase it: smoother, more realistic-looking test tracks.
#   - Decrease it: coarser tracks, marginally faster per-run computation.
#   - Typical adjustment: change by 200-500 at a time; 1000 is already
#     quite fine detail and rarely needs increasing.
PATH_N_POINTS = 1000


# ------------------------------------------------------------------------------
# OPTUNA TPE PRE-SEARCH (optional warm-start for CMA-ES)
# ------------------------------------------------------------------------------

# USE_OPTUNA_PRESEARCH — "Should the tuner spend a short exploratory phase
# with a different search algorithm (Optuna's TPE sampler) before handing
# off to CMA-ES?"
# CMA-ES currently always starts its search at a fixed point (the geometric
# midpoint of each weight's allowed range) and has to spend some of its own
# budget finding out which general area of the 9-dimensional search space is
# promising before it can start refining within it. TPE (Tree-structured
# Parzen Estimator) is a cheaper, more sample-efficient method for narrowing
# down "which general area is promising" — it doesn't refine as precisely as
# CMA-ES, but gets there in fewer evaluations. Running it first and handing
# CMA-ES a better starting point (instead of the fixed midpoint) can mean
# CMA-ES spends more of its budget on fine refinement instead of coarse
# search.
#   True  = run the Optuna pre-pass, then start CMA-ES from its best result.
#   False = skip it entirely and use the original fixed-midpoint start —
#           the exact previous behaviour, useful for a clean before/after
#           comparison.
# Requires the `optuna` package (`pip install optuna`) — not part of this
# repo's own code, only installed if you actually use this feature. Defaults
# to False so a fresh checkout behaves exactly as before this feature existed
# until you deliberately opt in (and confirm `optuna` is installed).
USE_OPTUNA_PRESEARCH = True

# OPTUNA_PRE_PASS_EVALS — "How many test-drives does the Optuna pre-pass get
# to use, out of the tuner's total budget?"
# This comes out of a separate mini-budget, not out of MAX_EVALS — the two
# phases run one after another, so total wall-clock time is roughly the sum
# of both. Keeping this meaningfully smaller than MAX_EVALS is what keeps
# the pre-pass "cheap": it only needs to find a good general area, not the
# precise optimum (that's still CMA-ES's job afterwards).
#   - Increase it: better/more reliable starting point for CMA-ES, at the
#     cost of extra wall-clock time before CMA-ES even begins.
#   - Decrease it: faster pre-pass, but a noisier/less-informed starting
#     point (CMA-ES may need to do more of the coarse-search work itself).
#   - Typical adjustment: keep it in the 10-20% of MAX_EVALS range; the
#     default below is 10%.
OPTUNA_PRE_PASS_EVALS = max(10, int(0.1 * MAX_EVALS))


# ------------------------------------------------------------------------------
# COST FUNCTION SCORING WEIGHTS
# ------------------------------------------------------------------------------

# METRIC_SCALES — "What counts as a NORMAL amount of each thing being
# measured?"
# Each of the 12 metrics below is divided by its entry here before being
# multiplied by its SCORE_WEIGHTS entry. That turns every metric into a
# roughly unitless "multiples of a typical value" number, so a weight of
# 0.05 next to a weight of 0.015 really does mean "this matters ~3x more".
#
# WHY THIS EXISTS (measured, 2026-08-06)
# --------------------------------------
# Without it, a metric's real influence is weight x typical magnitude, not
# weight — and the 12 metrics have wildly different natural magnitudes
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
                 #    metric of the 12 — previously ~0.0003 effective)
        0.70,    # 10 peak_lateral_error     (0.57-0.82 m)
        2.30,    # 11 speed_rmse             (1.86-2.83 m/s)
    ],
    dtype=float,
)
assert len(METRIC_SCALES) == 12
assert np.all(METRIC_SCALES > 0.0), "METRIC_SCALES must be strictly positive (used as a divisor)"


# SCORE_WEIGHTS — "How much does each aspect of driving quality matter when
# grading a test run?"
# Every test run is graded on 12 different things (see the list below), and
# each grade is multiplied by its corresponding weight here, then added
# together into one final score (lower is better). This list is what the
# automated tuner is actually trying to minimise — it is the definition of
# "good driving" for this whole project.
#
# As of 2026-08-06 these weights are applied to each metric AFTER it has been
# divided by its METRIC_SCALES entry above. That normalisation is what makes
# a weight mean what it says: previously the weights hit each metric's *raw*
# value, and since the 12 metrics have wildly different natural magnitudes
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
#
# Increasing any one weight makes the tuner prioritise fixing that aspect
# of driving more, even if it makes other aspects slightly worse.
#   - Typical adjustment: change a weight by roughly 20-30% of its own value
#     at a time, then re-tune and compare.
SCORE_WEIGHTS = np.array(
    [
        0.505,  # 0  rmse                    (lateral + heading tracking; primary)
        0.09,   # 1  yaw_rms                 (was 0.06 — live standalone-ROS test data
                #    2026-08-05 showed yaw_rate swinging +0.9/-1.1 rad/s within a few
                #    hundred ms in corners; CMA-ES had too little pressure to avoid
                #    this via the composite score, raised so oscillatory yaw actually
                #    costs the tuner something. Offset below.)
        0.065,  # 2  smooth_rms               (was 0.085 — trimmed to offset the yaw_rms/
                #    steering_reversal_rate raise; already has partial overlap with
                #    those two as a general jerkiness measure)
        0.02,   # 3  steer_rms
        0.015,  # 4  accel_rms               (was 0.005 — too small to give CMA-ES
                #    any real gradient on throttle/brake effort; nudged up)
        0.03,   # 5  max_steering
        0.045,  # 6  steering_sat_ratio       (was 0.075 — trimmed to offset the
                #    yaw_rms/steering_reversal_rate raise; less directly related to
                #    the oscillation/chatter symptom than the two raised terms)
        0.045,  # 7  jerk_rms
        0.02,   # 8  max_yaw_rate
        0.05,   # 9  steering_reversal_rms  (was 0.03 on the old flat-count-based
                #    "steering_reversal_rate", originally 0.005 on a raw count of
                #    ~0-30; live standalone-ROS test data 2026-08-05 showed steering
                #    sign reversals almost every ~0.05s tick, worst in corners — this
                #    metric directly measures that "hunting" behaviour and had almost
                #    no weight to discourage it, so raised further. Offset below.
                #    2026-08-06: the metric itself was replaced with a magnitude-
                #    weighted RMS (see sim/scoring.py) so a tiny trim wiggle and a
                #    path-demanded direction change no longer score the same as an
                #    aggressive hunting swing; this weight value carries over as-is
                #    since the two metrics are both O(reversal-related, per-step-
                #    normalised) in scale, but re-tuning may want to revisit it.)
        0.10,   # 10 peak_lateral_error
        0.015,  # 11 speed_rmse              (was 0.005 — same issue as accel_rms)
    ],
    dtype=float,
)
assert len(SCORE_WEIGHTS) == 12

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
    # "PATH_SKIDPAD",
    # "PATH_S_BEND",
    # "PATH_MIXED",
    # "PATH_CHICANE",
    # "PATH_ACCELERATION"
]

# ------------------------------------------------------------------------------
# PERFORMANCE BONUS WEIGHTS
# ------------------------------------------------------------------------------

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
# This used to be a hard worst-case (equivalent to TAIL_QUANTILE = 1.0). The
# problem, measured 2026-08-06: a DNF adds a flat +3.0 (+6.0 off-track), so
# ONE unlucky task out of ten shifted the objective by ~0.9 and swamped all
# twelve continuous quality metrics. A plausible hand-picked gain set scored
# 3rd-WORST of six — below two deliberately pathological sets — purely
# because one of its ten tasks DNF'd.
#   - 1.0  = old behaviour, the single worst task decides the tail term.
#   - 0.8  = around the 2nd-worst of 10 tasks. One bad task still hurts a
#            lot; two bad tasks hurt much more.
#   - 0.5  = the median; effectively stops punishing rare failures at all
#            (not recommended — that's what DNF_PENALTY is for).
#   - Typical adjustment: 0.05-0.1 at a time. Lower it if tuning results
#     swing wildly between runs; raise it if the tuner starts accepting
#     weights that reliably fail one track.
TAIL_QUANTILE = 0.8

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


# ==============================================================================
# FAST TEST MODE (for validating tuner/benchmark code changes quickly)
# ==============================================================================
# FAST_TEST_MODE — "Am I checking that a code change to the tuner/benchmark
# still runs correctly, or am I actually trying to find good driving weights?"
# A real offline_tuner.py run (MAX_EVALS=2500 across a 5-path validation
# suite) or a full performance_stats.py benchmark (11 paths x multiple
# initial conditions/repeats) takes minutes to hours. That cost is fine when
# the result is meant to be used, but wasteful when you only need to confirm
# "does this still import/run/converge" after an unrelated code change.
#   - Leave False for any run whose output you intend to actually use (a real
#     tuning session, or a benchmark compared against tuning history.txt).
#   - Set True only for quick development smoke-tests: cuts a tuning run down
#     to roughly a minute by shrinking the eval budget, path count, track
#     resolution, and solver precision. Never paste weights produced with
#     this on into settings.py's Q_diag/R_diag/R_rate_diag — they're a
#     correctness check, not a tuned result.
FAST_TEST_MODE = False

if FAST_TEST_MODE:
    MAX_EVALS = 150                                        # was 2500
    VALIDATION_SUITE = ["PATH_SUDDEN_TURN", "PATH_HAIRPIN"]  # was 5 paths
    ROLLOUT_EPS = 1e-3                                      # was 1e-4, looser/faster OSQP
    ROLLOUT_MAX_ITER = 2000                                 # was 8000
    PATH_N_POINTS = 300                                     # was 1000
    USE_PLANNER = False                                     # skip perception/planner overhead
    OPTUNA_PRE_PASS_EVALS = max(5, int(0.1 * MAX_EVALS))  # match the 0.1 ratio above, keep pre-pass proportionally tiny too