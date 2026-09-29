"""
settings/solver.py — headless-rollout solver settings (OSQP epsilon/max
iterations), DNF penalties, the CMA-ES/Optuna tuning-engine budget, and
FAST_TEST_MODE.
"""

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
MAX_EVALS = 1500

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
