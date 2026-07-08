import numpy as np
from sim_track import TRACK_HALF_WIDTH

# ==============================================================================
# GENERAL SYSTEM CONFIGURATION (TUNER + SIMULATOR)
# ==============================================================================

# MPC horizon: must match N_horizon in simulation.py exactly so that the weights
# tuned here are valid when transferred to the live simulator.
N_HORIZON = 25

# Whether to use perception and planner in tuner
USE_PLANNER = True

# How steps of delay to simulate between controller commands
DELAY_STEPS = 0

MAX_FAILS = 5  # Consecutive solve failures before DNF

OFFTRACK_LIMIT = TRACK_HALF_WIDTH * 1.3  # Lateral error threshold for DNF (m)

DT        = 0.05    # Simulation timestep (s) — 20 Hz, matches vehicle_physics sub-stepping


# ==============================================================================
# TUNER ENGINE & CONSTRAINT SETTINGS
# ==============================================================================

# ------------------------------------------------------------------------------
# DNF (DID-NOT-FINISH) PENALTY CONFIGURATION
# ------------------------------------------------------------------------------

# Penalty for not completing the track, used to discourage
# stationary vehicle behaviour
DNF_PENALTY = 3.0

# Penalty if the vehicle went off track
DNF_OFFTRACK_PENALTY = 3.0


# ------------------------------------------------------------------------------
# SOLVER SETTINGS FOR HEADLESS ROLLOUTS
# ------------------------------------------------------------------------------

# Looser than the live simulator (1e-5) for ~2x faster rollouts at negligible
# accuracy cost. These are passed to solve_mpc() in run_headless_rollout().
ROLLOUT_EPS = 1e-5
ROLLOUT_MAX_ITER = 8000

# Graceful shutdown flag: set by SIGINT handler; checked each CMA generation.
_stop_requested = False

# Total true-evaluation budget (surrogate skips many; this controls wall time).
MAX_EVALS = 2500

# Path resampling resolution — independent of CMA-ES budget
PATH_N_POINTS = 1000


# ------------------------------------------------------------------------------
# COST FUNCTION SCORING WEIGHTS
# ------------------------------------------------------------------------------

# One array, one place to edit. Shared with performance_stats.py via import.
# Indices correspond to the metrics array built by scoring.RolloutMetrics /
# scoring.compute_composite_score (see IDX_* constants in scoring.py, which
# must stay in sync with this ordering):
#   0: rmse              — primary lateral+heading tracking error (combined RMSE)
#   1: yaw_rms           — yaw rate stability (damps oscillations)
#   2: smooth_rms        — control smoothness (delta-u RMS; penalises jitter)
#   3: steer_rms         — steering effort (RMS magnitude)
#   4: accel_rms         — acceleration effort (RMS magnitude)
#   5: max_steering      — peak steering command (prevents saturation events)
#   6: steering_sat_ratio — fraction of steps where steering hit 95% of limit
#   7: jerk_rms          — control jerk (delta^2-u RMS; penalises rapid changes)
#   8: max_yaw_rate      — peak yaw rate (limits cornering aggression)
#   9: steering_reversals — count of steering sign reversals (penalises hunting)
#  10: peak_lateral_error — worst single-step lateral deviation (safety margin)
#  11: speed_rmse         - difference between current and planner speed
#
# Design note: steer_rms (3), max_steering (5), and steering_sat_ratio (6)
# are correlated (all measure the same signal at different aggregation levels).
# Weight was redistributed to rmse and peak_lateral_error which are the primary
# tracking quality signals and previously under-weighted.
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


# ------------------------------------------------------------------------------
# PERFORMANCE BONUS WEIGHTS
# ------------------------------------------------------------------------------

COMPLETION_BONUS_WEIGHT = 0.5  # Subtracted from score when vehicle finishes path
TIME_BONUS_WEIGHT = 0.25       # Subtracted from score for fast completion