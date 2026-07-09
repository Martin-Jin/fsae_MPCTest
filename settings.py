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
from sim_track import TRACK_HALF_WIDTH

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
DELAY_STEPS = 0

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
Q_diag      = [0.9638529433528358, 0.16917546433555822, 0.8412084423109519, 0.6719136934634028, 1.3722642626759542, 0.0, 0.0, 0.0]
R_diag      = [1.0732323890203437, 0.6986142210105707]
R_rate_diag = [2.2731056206565956, 3.8354972983644497]


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
#     1e-5 to 5e-5), since this value works on an exponential/scientific
#     scale, not a simple linear one.
ROLLOUT_EPS = 1e-5

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
# COST FUNCTION SCORING WEIGHTS
# ------------------------------------------------------------------------------

# SCORE_WEIGHTS — "How much does each aspect of driving quality matter when
# grading a test run?"
# Every test run is graded on 12 different things (see the list below), and
# each grade is multiplied by its corresponding weight here, then added
# together into one final score (lower is better). This list is what the
# automated tuner is actually trying to minimise — it is the definition of
# "good driving" for this whole project. All 12 numbers must add up to
# exactly 1.0 (there's a check below that enforces this), so if you increase
# one weight you must decrease others by the same total amount to compensate.
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
#   9: steering_reversals   — how many times the car flips from steering
#                            left to steering right or vice versa
#                            (a "hunting"/indecisiveness measure)
#  10: peak_lateral_error   — the single worst moment the car was off the
#                            racing line, even briefly
#  11: speed_rmse           — how far off the intended speed the car drives
#                            on average
#
# Increasing any one weight makes the tuner prioritise fixing that aspect
# of driving more, even if it makes other aspects slightly worse.
#   - Typical adjustment: move 0.01-0.03 from one weight to another at a
#     time, then re-tune and compare — because everything must sum to 1.0,
#     even small shifts noticeably change priorities.
SCORE_WEIGHTS = np.array(
    [
        0.505,  # 0  rmse               (lateral + heading tracking; primary)
        0.06,   # 1  yaw_rms
        0.07,   # 2  smooth_rms
        0.02,   # 3  steer_rms
        0.005,  # 4  accel_rms
        0.06,   # 5  max_steering
        0.09,   # 6  steering_sat_ratio
        0.06,   # 7  jerk_rms
        0.02,   # 8  max_yaw_rate
        0.005,  # 9  steering_reversals
        0.10,   # 10 peak_lateral_error
        0.005,  # 11 speed_rmse
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