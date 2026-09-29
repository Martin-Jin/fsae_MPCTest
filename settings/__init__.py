"""
settings/__init__.py — Central Configuration Package

PURPOSE
-------
This is the one place you should look at to change how the simulator and
the offline tuner behave, without touching any of the maths or control
code elsewhere.

Nothing physical about the car (its weight, tyre grip, engine power etc.)
lives here -- that's all in vehicle_physics.py. This package only controls
how the *controller* is scored, tuned, and configured to drive.

LAYOUT
------
  settings/general.py   core config: horizon, planner mode, delay,
                        offtrack/failure thresholds, pose-feed-hold
  settings/noise.py     SLAM + cone detection noise
  settings/planner.py   sim/planner.py::SimPlanner's tunables
  settings/lmpc.py      LTV-QP (LMPC) weights and adaptive-gain shaping
  settings/nmpc.py      Nonlinear MPC weights, solver internals, feature flags
  settings/solver.py    headless-rollout solver settings, DNF penalties,
                        the tuning engine's own budget, FAST_TEST_MODE
  settings/scoring.py   composite-score weights and the constrained
                        (hard-floor / objective / quality) scoring structure

Every name below is re-exported at package scope so BOTH `import settings;
settings.X` and `from settings import X` keep working -- but only the
former sees a runtime override. `setattr(settings, name, value)`
(tuner/investigations/steering_chatter_check.py's --set mechanism) rebinds
the name in THIS namespace; a consumer that did `from settings.lmpc import
Q_diag` directly (bypassing this re-export) would hold its own reference
and never see the override. Every in-repo consumer accesses settings via
`import settings; settings.X`, not a submodule import, for exactly this
reason -- keep it that way when adding a new consumer.
"""

from settings.general import (
    N_HORIZON, TERMINAL_Q_SCALE, USE_PLANNER, USE_PRECOMPUTED_SPEED_PROFILE,
    ENABLE_DYNAMIC_SPEED_CAP, DYNAMIC_CAP_A_LAT_MAX, DYNAMIC_CAP_SAFETY,
    DELAY_STEPS, DELAY_JITTER_STEPS, DELAY_JITTER_SEED, MAX_FAILS,
    OFFTRACK_LIMIT, DT, REF_HEADING_RATE_LIMIT_ENABLED,
    REF_HEADING_RISE_RATE, POSE_HOLD_ENABLED, POSE_HOLD_PROB,
    POSE_HOLD_MEAN_TICKS, POSE_HOLD_MAX_TICKS, POSE_HOLD_SEED
)

from settings.noise import (
    SLAM_NOISE_ENABLED, SLAM_POS_JITTER_STD, SLAM_YAW_JITTER_STD,
    SLAM_POS_DRIFT_STD, SLAM_YAW_DRIFT_STD, SLAM_DRIFT_TAU, SLAM_NOISE_SEED,
    CONE_NOISE_ENABLED, CONE_POS_JITTER_STD, CONE_NOISE_SEED
)

from settings.planner import (
    PLANNER_SMOOTH_PER_PT, PLANNER_LOOK_RADIUS, PLANNER_PLAN_HORIZON,
    PLANNER_PATH_BLEND
)

from settings.lmpc import (
    ADAPTIVE_Q_SCALING_ENABLED, STEER_RATE_ANTI_HUNT_ENABLED,
    REVERSAL_PENALTY_ENABLED, REVERSAL_PENALTY_BOOST_MAX, REVERSAL_PENALTY_K,
    CORNER_FACTOR_K, Q_EY_STRAIGHT, Q_EY_CORNER, Q_EPSI_STRAIGHT,
    Q_EPSI_CORNER, Q_R_STRAIGHT, Q_R_CORNER, RRATE_STEER_STRAIGHT,
    RRATE_STEER_CORNER, R_STEER_CORNER_MID, LOW_SPEED_CORNER_BOOST_V_HALF,
    LOW_SPEED_CORNER_BOOST_MAX_EXTRA, EPSI_RA_HALF_RAD,
    EPSI_RA_ACCEL_BOOST_MAX, EPSI_RA_BRAKE_FLOOR, SPEED_TARGET_DEFICIT_MAX,
    Q_diag, R_diag, R_rate_diag, R_A_ACCEL, R_A_BRAKE
)

from settings.nmpc import (
    USE_NMPC, NMPC_Q_E_Y, NMPC_Q_E_YD, NMPC_Q_E_PSI, NMPC_Q_EPSI_DOT,
    NMPC_Q_E_V, NMPC_R_DELTA, NMPC_R_A_ACCEL, NMPC_R_A_BRAKE,
    NMPC_R_RATE_DELTA, NMPC_R_RATE_A, NMPC_TERMINAL_SCALE,
    NMPC_STEER_RATE_ANTI_HUNT_ENABLED, NMPC_CORNER_RRATE_BLEND_ENABLED,
    NMPC_CORNER_FACTOR_K, NMPC_RRATE_STEER_STRAIGHT, NMPC_RRATE_STEER_CORNER,
    NMPC_RJERK_DELTA, NMPC_RJERK_A, NMPC_RRATE_ZONE_ENABLED,
    NMPC_RRATE_ZONE_BOOST_STRAIGHT, NMPC_RRATE_ZONE_EASE_APPROACH,
    NMPC_RRATE_ZONE_FLOOR_CORNER, NMPC_RRATE_STAGE_RAMP_ENABLED,
    NMPC_RRATE_STAGE_NEAR, NMPC_REVERSAL_PENALTY_ENABLED,
    NMPC_REVERSAL_PENALTY_BOOST_MAX, NMPC_REVERSAL_PENALTY_K, NMPC_HORIZON,
    NMPC_SQP_ITERS, NMPC_SOLVE_BUDGET_MS, NMPC_RK_SUBSTEPS,
    NMPC_JAC_SUBSTEPS, NMPC_JAC_GATE_SPEED, NMPC_JAC_SUBSTEPS_FAST,
    NMPC_RK_GATE_SPEED, NMPC_RK_SUBSTEPS_FAST,
    NMPC_STANDSTILL_STEER_DAMP_ENABLED, NMPC_STANDSTILL_SPEED,
    NMPC_STANDSTILL_FADE_SPEED, NMPC_STANDSTILL_STEER_R_SCALE,
    NMPC_TRUST_DELTA_RAD, NMPC_TRUST_A, NMPC_BACKTRACK_MAX,
    NMPC_TRACK_HALFWIDTH, NMPC_SLACK_WEIGHT, NMPC_CURVATURE_DENSE_STEP,
    NMPC_CURVATURE_SMOOTH_W, NMPC_KAPPA_CLIP, NMPC_KAPPA_RATE_MAX,
    NMPC_OSQP_MAX_ITER, NMPC_OSQP_EPS, NMPC_ALAT_CEILING_ENABLED,
    NMPC_SPLINE_REFERENCE_ENABLED, NMPC_FRICTION_CIRCLE_ENABLED,
    NMPC_PROGRESS_ENABLED, NMPC_Q_PROGRESS, NMPC_PROGRESS_REACH,
    NMPC_PROGRESS_V_MIN, NMPC_SLACK_LINEAR_WEIGHT,
    NMPC_LATENCY_COMPENSATION_ENABLED, NMPC_LATENCY_COMPENSATION_MS,
    NMPC_V_DES_FILTER_ALPHA, ALAT_CEILING_FLAT, ALAT_CEILING_SLOPE,
    ALAT_CEILING_INTERCEPT
)

from settings.solver import (
    DNF_PENALTY, DNF_OFFTRACK_PENALTY, ROLLOUT_EPS, ROLLOUT_MAX_ITER,
    _stop_requested, MAX_EVALS, PATH_N_POINTS, USE_OPTUNA_PRESEARCH,
    OPTUNA_PRE_PASS_EVALS, FAST_TEST_MODE
)

from settings.scoring import (
    METRIC_SCALES, SCORE_WEIGHTS, VALIDATION_SUITE, COMPLETION_BONUS_WEIGHT,
    TAIL_QUANTILE, CONSTRAINT_FLOOR, COMPLETION_THRESHOLD,
    TIME_OBJECTIVE_WEIGHT, QUALITY_WEIGHT, TIME_BONUS_WEIGHT
)

# ==============================================================================
# FAST TEST MODE (for validating tuner/benchmark code changes quickly)
# ==============================================================================
# Overrides constants from MULTIPLE submodules above (general/solver/scoring),
# so it lives here, run after every submodule is imported, rather than inside
# any one of them -- see FAST_TEST_MODE's own docstring in settings/solver.py
# for what it does and why.
if FAST_TEST_MODE:
    MAX_EVALS = 150                                        # was 2500
    VALIDATION_SUITE = ["PATH_SUDDEN_TURN", "PATH_HAIRPIN"]  # was 5 paths
    ROLLOUT_EPS = 1e-3                                      # was 1e-4, looser/faster OSQP
    ROLLOUT_MAX_ITER = 2000                                 # was 8000
    PATH_N_POINTS = 300                                     # was 1000
    USE_PLANNER = False                                     # skip perception/planner overhead
    OPTUNA_PRE_PASS_EVALS = max(5, int(0.1 * MAX_EVALS))  # match the 0.1 ratio above, keep pre-pass proportionally tiny too
