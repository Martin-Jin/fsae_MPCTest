"""
settings/lmpc.py — LTV-QP (LMPC) controller: cost weights (Q_diag/R_diag/
R_rate_diag), adaptive-gain shaping (corner-factor scheduler, low-speed/
accel-brake boosts), and the flags that gate them.
"""
import numpy as np

# [LTV-QP only] ADAPTIVE_Q_SCALING_ENABLED — "Should the controller relax its lateral-error
# penalty when it's already close to the centreline, to stop small-error
# hunting?" See controller/model_utils.py::adaptive_Q_scaling for the full
# mechanism. Not reproduced on the offline recorded-map rollout as currently
# tuned (there, steering-reversal rate rises WITH |e_y|, the opposite trend
# seen live) — may be a live-only symptom. Kept enabled to match the live
# controller; re-run VALIDATION_SUITE/recorded-map for new DNFs if
# re-tuning around this.
ADAPTIVE_Q_SCALING_ENABLED = True

# [LTV-QP only] STEER_RATE_ANTI_HUNT_ENABLED — TEMPORARY/EXPERIMENTAL, fsds sim only.
# Heavily penalises steering-rate-of-change, but only when the car is
# already centred (|e_y| small) AND not currently curving (kappa small) --
# see controller/model_utils.py::steer_rate_anti_hunt for the exact
# thresholds and mechanism. "Corner ahead" is NOT detected via path
# lookahead here -- it reuses the same causal, current-curvature signal, so
# it cannot anticipate a corner before the car is already turning into it.
# NOT VALIDATED against VALIDATION_SUITE/recorded-map or any live log.
# Kept enabled to match the live controller.
STEER_RATE_ANTI_HUNT_ENABLED = True

# [LTV-QP only, EXPERIMENTAL] Soft constraint against
# steering REVERSALS (tick-to-tick sign flip), approximated by boosting
# R_rate[0,0] whenever LAST tick's steering was already close to zero -- see
# controller/model_utils.py::reversal_penalty_boost's docstring for why a
# reversal can't be detected directly inside a convex QP and this
# approximates it. Composes multiplicatively with STEER_RATE_ANTI_HUNT
# above (mirrors mpc_core.py's own composition fix), does not replace it.
# Default False: genuine experiment, not yet validated.
REVERSAL_PENALTY_ENABLED = False

REVERSAL_PENALTY_BOOST_MAX = 4.0   # ceiling multiplier, applied when u_prev steer == 0

REVERSAL_PENALTY_K = 8.0           # 1/rad; half-boost at ~7.2deg of previous steering

# ── Lookahead gain-scheduling family: removed ────────────────────────────────
# This section used to carry ~15 interacting mechanisms
# (ADAPTIVE_Q_LOOKAHEAD_ENABLED, ADAPTIVE_Q_DEMAND_NORMALISED, the exit-decay
# constants, STEER_EFFORT_STRAIGHT_BOOST_ENABLED,
# LOOKAHEAD_STEER_EFFORT_RELAX_ENABLED/_FLOOR, CURVATURE_FORCING_ENABLED/
# _GAIN, ANTI_HUNT_K_LOOKAHEAD, and the ADAPTIVE_Q_STRAIGHT_*/
# ADAPTIVE_Q_UTURN_*/ALAT_CEILING_*/ADAPTIVE_Q_DEMAND_HALF constants further
# below) that scanned forward along the path (kappa_max_abs = peak curvature
# within a lookahead window) and reweighted today's Q/R cost based on what's
# coming up. Removed because this MPC formulation already predicts state
# error against the reference at each future horizon step; reweighting
# TODAY's (usually near-zero) cost based on a forward scan doesn't change
# what the horizon predicts when the car actually gets there -- see
# controller/model_utils.py's module docstring for the full removal
# rationale, and mpc_core.py's mirrored removal (CLAUDE.md's parity rule).
# Replaced by CORNER_FACTOR_K and the Q/R_rate/R straight/corner blend
# endpoints below, plus LOW_SPEED_CORNER_BOOST_*/EPSI_RA_* — all driven by
# CURRENT curvature/speed/heading-error, never a forward scan. Also removed
# as unused/didn't-work: CURVATURE_FORCING_ENABLED/_GAIN (structurally
# unsound, see docs/logs) and LOW_SPEED_STEER_RATE_BOOST_* (see above --
# disabled, gated on speed alone with no way to distinguish wanted
# low-speed turn-in from unwanted post-exit wobble).

# ── Current-state corner-factor scheduler ────────────────────────────────────
# [LTV-QP only] _corner_factor(kappa, CORNER_FACTOR_K) is a single continuous 0 (straight)
# -> 1 (full corner) curve of CURRENT |kappa| only -- no forward scan,
# symmetric on entry/exit. k=8.0 matches the deleted lookahead mechanisms'
# own default sharpness (8.0) -- same curve shape, now applied to the
# current-position signal instead of a forward-scanned one. Mirrors
# MPCParams.corner_factor_k.
CORNER_FACTOR_K = 8.0

# [LTV-QP only] Q[0,0] (e_y) / Q[2,2] (e_psi) / Q[3,3] (r) / R_rate[0,0] straight/corner
# blend endpoints, and R[0,0]'s special MIDDLE blend target -- see
# mpc_core.py's compute() for the exact _blend() wiring these feed. Q[3,3]/
# R_rate[0,0] RELAX in-corner (corner value LOWER than straight) so the MPC
# can rotate/steer fast enough to hit the tighter Q[0,0]/Q[2,2] targets;
# R[0,0] blends toward a MIDDLE value, not the same low extreme, so
# steering effort sits "somewhere in between the two extremes to discourage
# saturation" rather than becoming cheap enough to overshoot. Mirrors
# MPCParams.q_ey_straight/_corner, q_epsi_straight/_corner,
# q_r_straight/_corner, rrate_steer_straight/_corner, r_steer_corner_mid.
Q_EY_STRAIGHT = 4.5

Q_EY_CORNER = 9.0

Q_EPSI_STRAIGHT = 1.5

Q_EPSI_CORNER = 3.0

Q_R_STRAIGHT = 1.0

Q_R_CORNER = 0.5

RRATE_STEER_STRAIGHT = 2.0

RRATE_STEER_CORNER = 1.25

R_STEER_CORNER_MID = 1.35

# ── Low-speed-in-corner extra boost ──────────────────────────────────────────
# [LTV-QP only] A NEW, corner-GATED mechanism -- distinct from the removed
# LOW_SPEED_STEER_RATE_BOOST_* (which fired on speed ALONE with no way to
# tell wanted low-speed turn-in from unwanted post-exit wobble). This only
# ever adds to corner_frac (see model_utils._low_speed_corner_boost), so it
# is an exact no-op on any straight regardless of speed -- "turn even more
# at low speed during turning", not "penalise steering rate whenever slow".
# Mirrors MPCParams.low_speed_corner_boost_v_half/_max_extra.
LOW_SPEED_CORNER_BOOST_V_HALF = 4.0

LOW_SPEED_CORNER_BOOST_MAX_EXTRA = 0.3

# ── Heading-error-driven accel/brake asymmetry ───────────────────────────────
# [LTV-QP only] Always-on, independent of the corner-factor scheduler above: scales
# R_A_ACCEL/R_A_BRAKE (below) by a continuous 0->1 fraction of CURRENT
# |e_psi| -- see mpc_core.py's compute() for the exact blend. Not
# gain-scheduled off a forward scan; purely reactive to the car's own
# current heading error. Mirrors MPCParams.epsi_ra_half_rad/
# _accel_boost_max/_brake_floor.
EPSI_RA_HALF_RAD = np.radians(10.0)

EPSI_RA_ACCEL_BOOST_MAX = 2.0

EPSI_RA_BRAKE_FLOOR = 0.5

# ── Speed-target deficit clamp ────────────────────────────────────────────────
# [Both controllers] Caps how far the ramped speed target may run ahead of the
# car's own current speed, before either the LTV-QP's q_e_v row or the NMPC's
# e_v/progress-cap row ever sees it. Measured 2026-09-20: at 2.5 this was the
# binding constraint on acceleration for 36.8% of a lap, not the launch/
# recovery guard it was written as. Raised to 5.0 (faster lap, lower |e_y|,
# lower steering saturation, no measured trade-off offline); NOT yet
# live-validated at this value. See docs/logs/nmpc_progress_term_investigation.md.
# Mirrors MPCParams.speed_target_deficit_max.
SPEED_TARGET_DEFICIT_MAX = 2.55

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
# the car cares about". Bigger number = the car tries harder to fix that
# particular error, at the cost of everything else. Change any single number
# by no more than 20-30% at a time and re-test — small changes can have
# surprisingly large effects because they interact with each other.
#
# [shared] Q_diag index -> state penalised (see bicycle_model.py's STATE VECTOR comment
# for the full state definitions):
#   [0] e_y        lateral deviation from path centreline (m)
#   [1] e_y_dot    rate of change of lateral deviation (m/s)
#   [2] e_psi      heading error relative to path tangent (rad)
#   [3] e_psi_dot  yaw rate (rad/s). Shared base value the NMPC also reads
#                  (NMPC_Q_EPSI_DOT below), but under the NMPC it weights
#                  HEADING-ERROR rate, not absolute yaw rate -- same slot,
#                  different regressor
#   [4] e_v        speed error: vx - v_target (m/s)
#   [5] e_a        unused (always 0.0, kept for structural consistency only)
#   [6] delta_act  actuator-lagged steering angle (rad) -- always 0.0, no
#                  tuned weight sets this state
#   [7] a_act      actuator-lagged acceleration (m/s^2) -- always 0.0, ditto
# Q_diag[4]=5.0 is car-tuned (not offline) — a measured optimum on the
# corner-approach phase; do not raise further expecting gains. See docs/logs
# for the sweep.
#
# Whole-run averages hide this: they are dominated by the ~50% of ticks on
# straights, where a speed-error weight does little. Compare on the approach
# phase when re-tuning this.
# Mirrors mpc_params.py's Q_diag.
Q_diag      = [6.4, 0.0, 1.65, 1.0, 1.5, 0.0, 0.0, 0.0]

# [shared] R_diag index -> input penalised:
#   [0] delta_cmd  steering command effort (rad)
#   [1] a_cmd      acceleration command effort (m/s^2)
# R_diag[1]=0.77 is car-tuned toward cheap braking AND acceleration effort.
# Motivation: a_cmd floors around -2.2 m/s^2 against a -7.0 limit on every
# logged run, while the car demonstrably sustains -6.3, so the MPC uses
# under a third of its braking authority and arrives ~2 m/s hot at corner
# entry. The cause is structural: a_cmd is a RATE, so one 50 ms step of
# braking (or accelerating) at magnitude 6 changes speed by only 0.30 m/s,
# while the effort cost R[1,1]*a^2 is paid immediately -- at these weights
# |a|=6 is only worth it if it removes >2.9 m/s of error per step. Lowering
# R[1,1] is the cheap lever; raising Q_diag[4] instead would need ~460.
#
# The same effort/benefit mismatch applies symmetrically to ACCELERATION,
# not just braking -- live telemetry showed a_cmd topping out at ~3 m/s^2
# during a clean, well-tracked corner-exit straight with a large (3-9 m/s)
# speed deficit and zero competing lateral demand, well under the 12 m/s^2
# ceiling the same lap demonstrably used elsewhere. A sweep around this
# value confirmed 0.77 is a local optimum on this metric set (see docs/logs);
# re-sweep rather than assume further cuts help.
#
# R_diag[1] itself is now a NOMINAL value only (kept for shape/API parity
# with every R_diag consumer -- solve_mpc's needs_rebuild check, tuner
# scripts that don't know about the split, etc). The QP's actual a_cmd
# effort cost no longer reads it: see R_A_ACCEL/R_A_BRAKE below.
# Mirrors mpc_params.py's R_diag. R_diag[1] is nominal-only (see comment
# above) -- the live side's R_A_ACCEL/R_A_BRAKE split (below) is what
# actually matters for a_cmd's effort cost.
R_diag      = [1.8, 0.77]

# [shared] R_rate_diag index -> input RATE-OF-CHANGE penalised (tick-to-tick jerk, not
# the input itself):
#   [0] delta_cmd  steering rate of change
#   [1] a_cmd      acceleration rate of change
# Mirrors mpc_params.py's R_rate_diag.
R_rate_diag = [100.0, 2.0]

# [shared] R_A_ACCEL / R_A_BRAKE — separate effort weights for acceleration and
# braking. solve_mpc()'s a_cmd effort cost is r_a_accel*pos(a_cmd)^2 +
# r_a_brake*neg(a_cmd)^2 (see controller/optimiser.py), not R_diag[1]*a_cmd^2
# -- R_diag[1] is read only as the fallback default when a caller omits
# these. A single shared r_a weight cannot be tuned independently for
# acceleration vs. braking: lowering it to free up acceleration authority
# also weakens braking by the same amount, and live telemetry has shown the
# resulting asymmetry -- corners entered hot, steering saturating, unstable
# post-exit recovery -- because the same weight that frees up acceleration
# also caps how hard the QP is willing to brake. See `docs/reference/control_mechanisms.md`'s
# "Accel/brake effort weight split" for the diagnosis and retuning history.
# Mirrors mpc_params.py's R_A_ACCEL/R_A_BRAKE.
R_A_ACCEL = 0.9

R_A_BRAKE = 0.6
