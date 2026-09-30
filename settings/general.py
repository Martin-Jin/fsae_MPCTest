"""
settings/general.py — core simulator/tuner configuration: horizon length,
planner mode, delay compensation, offtrack/failure thresholds, and the
pose-feed-hold model.
"""
import numpy as np
from sim.perception import TRACK_HALF_WIDTH

# ==============================================================================
# GENERAL SYSTEM CONFIGURATION (TUNER + SIMULATOR)
# ==============================================================================

# N_HORIZON — "How far ahead does the car plan?"
# The controller doesn't just react to what's happening right now — it plans
# a short sequence of future steering/throttle moves and only acts on the
# first one, then re-plans next tick. This number is how many 0.05-second 
# (since simulator runs at 20Hz) steps ahead it plans each time 
# (35 steps = 1.75 seconds of look-ahead).
#   - Increase it: the car "sees" further ahead, which can smooth out
#     reactions to corners it hasn't reached yet, but each planning step
#     takes noticeably longer to compute (the difficulty roughly squares).
#   - Decrease it: faster to compute, but the car becomes more short-sighted
#     and can react late to corners.
#   - Typical adjustment: change by 5 steps (0.25 s) at a time. Must match
#     N_horizon in simulation.py and N in control_utils.py exactly, or the
#     weights tuned here won't behave the same on the real car.
N_HORIZON = 35

# [shared] TERMINAL_Q_SCALE — "How much extra does the controller care about where it
# ends up at the very end of its plan, compared to every other step?"
# With no terminal cost or constraint, the MPC has exactly the same incentive
# to track well at the last predicted step as at every other step, and no
# incentive to leave itself in a good position for what happens just past the
# horizon. This affects both stacks identically (lmpc/controller.py has the same gap).
# 1.0 = no-op (the only value ever validated against the current Q_diag/
#       R_diag/R_rate_diag tuning -- this is what every existing tuned
#       weight set assumes).
#   - Increase it: the last predicted step is penalised more heavily than
#     the others, which should reduce end-of-horizon myopic behaviour, at
#     the cost of some responsiveness earlier in the plan.
#   - Typical adjustment: try 2-5x as a starting point if tuning this;
#     re-validate against VALIDATION_SUITE and the recorded map for new
#     DNFs before trusting it, the same as any other weight change.
TERMINAL_Q_SCALE = 1.0

# USE_PLANNER — "Does the tuner pretend to have real cone-vision, or cheat
# and use the perfect track outline?"
# True  = the tuner simulates a car that can only see nearby cones and has
#         to build its own idea of the track from them (like the real car).
#         This is slower but tests the whole system, including mistakes the
#         perception/planning code might make.
# False = the tuner gives the car the exact, perfect racing line to follow.
#         Much faster, useful for quickly testing whether the driving style
#         itself (speed, smoothness) is good, but won't catch planner bugs.
# Default is False to match the live ROS side's path_map_path mode
# (precomputed path, no planner/perception in the loop) — see the "Offline
# parity note" comment below. Set True to re-enable the planner-in-loop
# rollout (perception mistakes, live-built centreline).
USE_PLANNER = False

# USE_PRECOMPUTED_SPEED_PROFILE — "For a track that's already been mapped
# (cone positions known), skip the live per-tick speed re-derivation and use
# the full-map speed profile computed once, up front, from the whole path."
#
# curvature_speed() (sim/speed_profile.py) re-derives the target speed every
# tick from only the sub-path currently visible/built — by design, since a
# real car doesn't have the map on its first lap. But the live-built
# centreline is frequently shorter than curvature_speed()'s own assumed
# scan_end=24m (the perception FOV's lateral window clips before its forward
# window does at sharper corners), silently giving the speed planner less
# runway than its own braking-distance design assumes.
#
# True  = once a recorded map is loaded (sim/track_io.load_recorded_track()),
#         look up the target speed from that map's own oracle profile
#         (path_v, computed non-causally from the WHOLE path via
#         compute_speed_profile()) at the car's current position, instead
#         of calling curvature_speed() on the live-built centreline.
#         Only valid when the whole track is already mapped — this is
#         explicitly the pre-mapped-track case, not a general fix for a
#         car exploring an unknown track live.
# False = unchanged: curvature_speed() on the live planner centreline
#         every tick, as before.
#
# No effect while USE_PLANNER=False above: with the planner disabled,
# run_core_rollout() already always uses the oracle path_v profile for
# speed regardless of this flag (sim/rollout/core.py). Set True here only
# matters if USE_PLANNER is switched back to True and you still want
# precomputed speed instead of live curvature_speed().
USE_PRECOMPUTED_SPEED_PROFILE = True

# ENABLE_DYNAMIC_SPEED_CAP — "On top of the oracle speed profile above, also
# cap the target speed in real time from the car's OWN current position and
# the live path curvature ahead of it."
#
# precomputed_speed_at() (the oracle lookup used when
# USE_PRECOMPUTED_SPEED_PROFILE=True) is a static, position-indexed lookup:
# it has no notion of the car currently running faster than the profile's
# own plan and the corner being too close to brake down to the profile's
# target in time. That mismatch is what shows up as late, hard braking and
# steering saturation right at corner entry — see
# fsae_MPCTest/`docs/reference/control_mechanisms.md`'s dynamic speed cap section for
# the log evidence.
#
# True  = also compute speed_profile.curvature_speed() (renamed at the call
#         site dynamic_speed_cap() on the live side) on the live path each
#         tick, using DYNAMIC_CAP_A_LAT_MAX/DYNAMIC_CAP_SAFETY below (NOT
#         curvature_speed()'s own defaults — this cap is meant to be a
#         tighter safety net under an already-trusted oracle target, not a
#         second opinion on the racing line), and take
#         min(oracle_v_target, dynamic_cap) every tick.
# False = unchanged: the oracle profile alone, exactly as before this flag
#         existed.
#
# No effect when USE_PRECOMPUTED_SPEED_PROFILE=False — that branch already
# calls curvature_speed() directly every tick with no oracle target to cap.
# Mirrors the live ROS side's enable_dynamic_speed_cap parameter
# (mpc_controller.py) — keep both in sync.
ENABLE_DYNAMIC_SPEED_CAP = True

# DYNAMIC_CAP_A_LAT_MAX / DYNAMIC_CAP_SAFETY — curvature_speed()'s own
# a_lat_max/safety parameters, but used ONLY by the dynamic cap above, kept
# deliberately tighter than curvature_speed()'s defaults (4.0 / 1.0) used
# elsewhere (e.g. the live-planner branch below) so the cap engages a little
# before the oracle profile would actually be violated, rather than exactly
# at the edge. Mirror the live ROS side's dynamic_cap_a_lat_max /
# dynamic_cap_safety parameters — keep all four in sync.
DYNAMIC_CAP_A_LAT_MAX = 3.2   # m/s^2

DYNAMIC_CAP_SAFETY = 0.9

# Offline parity note for the live ROS side's path_map_path param
# (fsae_planning's mpc_controller.py, which tracks a precomputed path
# instead of subscribing to the live planner's
# centreline): the equivalent offline experiment is USE_PLANNER=False above
# (or, for a recorded real track specifically, `python3 -m
# tuner.validation.recorded_map_rollout <map.json> --oracle`) — see that flag's own
# comment for what it does. No separate flag needed here; USE_PLANNER=False
# already removes the planner from the rollout and tracks path_X/path_Y/
# path_Psi (the same oracle path tuner/tools/export_speed_profile.py exports for
# path_map_path) directly.

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
# to change it, change TRACK_HALF_WIDTH in sim/perception.py instead, which also
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

# [LTV-QP only] REF_HEADING_RISE_RATE — "How fast is the planner's steering target allowed
# to swing before we start holding it back?"
# The planner's published centreline can point further into an upcoming
# corner than the car has actually turned yet ("anticipating" a corner
# early), which is strongly linked to steering saturation. This limiter caps
# how fast the reference heading the controller tracks (ref_psi) is allowed
# to change per second, exactly like SPEED_TARGET_RISE_RATE does for the
# speed target — the raw direction is still used once the car catches up,
# this only slows how fast the target moves.
#   - Increase it (or disable): the controller reacts to the planner's full
#     corner-anticipation immediately — may mean earlier, more confident turn-in.
#   - Decrease it: smoother, later turn-in, but risks entering a tight corner
#     with too little heading correction already applied ("understeering in").
#   - Units: deg/s. Only the magnitude of change is capped; sign (turning
#     left vs. right) is never touched, so this cannot reverse a correction.
# Lowering below ~85 risks DNFing a fast, tight slalom off-track (the
# reference is held back so hard the car cannot keep up) — re-run
# tuner/investigations/ref_heading_limiter_suite_check.py before changing this.
# Default off until validated live.
REF_HEADING_RATE_LIMIT_ENABLED = False

REF_HEADING_RISE_RATE = 90.0   # deg/s — only used when the flag above is True

# ==============================================================================
# POSE FEED HOLD (sim-to-real: the pose sometimes stops updating)
# ==============================================================================
# POSE_HOLD_ENABLED — "Does the simulated controller sometimes get handed the
# SAME pose it got last tick, instead of a fresh one?"
# On the real car, /fsae/slam/car_position intermittently stops publishing and
# the controller re-uses its last known pose while the car keeps moving.
# Without this flag, the offline rollout hands the controller a brand-new
# exact pose every single tick, so heading error can never accumulate this
# way — which is exactly why the simulator can show smooth driving while the
# car wobbles on the same track with the same weights.
#
# Measured on live telemetry (two runs, same track, same tuned weights,
# differing only in how badly the feed stalled):
#
#                          normal run       failed run
#     fresh-pose rate      18.9 Hz          6.4 Hz
#     repeated ticks       5.3%             60.7%
#     longest hold         5 ticks (0.25s)  20 ticks (0.99s)
#     peak pose_age        347 ms           1242 ms
#
# In the failed run the pose froze for ~1 s at 14 m/s — the car covered ~17 m
# blind, and when the feed resumed the heading error was unrecoverable
# (105 deg) and it spun. This is NOT DELAY_STEPS (which delays a pose that is
# still fresh each tick) nor DELAY_JITTER_STEPS (which perturbs only the
# controller's belief about the lag). It repeats the DATA.
#   - True : the tuner sees a controller that must survive going briefly blind.
#            Recommended, and the whole point of the model.
#   - False: previous behaviour, optimistic; the sim will keep flattering
#            weights that cannot cope on the car.
POSE_HOLD_ENABLED = True

# POSE_HOLD_PROB — chance, on a tick that delivered a FRESH pose, that a hold
# begins. Verified against the logs: 0.05 with the mean/max below reproduces
# 5.1% repeated ticks / mean hold 2.10 against the normal run's measured
# 5.3% / 2.08.
# To reproduce the FAILED run instead (60.7% repeated, mean hold 5.05, max 20)
# set POSE_HOLD_PROB=0.40, POSE_HOLD_MEAN_TICKS=5.05, POSE_HOLD_MAX_TICKS=20 —
# that config measures 61.2% / 4.99 / 20. Useful as a recovery stress test,
# but do NOT tune against it as the normal case; it is a fault condition, not
# the expected operating point.
#   - Typical adjustment: 0.01 at a time.
POSE_HOLD_PROB = 0.05

# POSE_HOLD_MEAN_TICKS — average length of a hold, in control ticks (0.05 s
# each). Measured 2.08 ticks on the normal run. Hold length is drawn
# geometrically, which reproduces the observed shape: mostly 2-tick holds with
# a thin tail of longer ones.
POSE_HOLD_MEAN_TICKS = 2.1

# POSE_HOLD_MAX_TICKS — hard cap on a single hold, counted as total ticks
# including the fresh one. 5 (0.25 s) matches the worst hold in the normal run.
POSE_HOLD_MAX_TICKS = 5

# POSE_HOLD_SEED — fixed so each rollout is reproducible and CMA-ES still gets
# a stable score per candidate. Change it only to check that a tuned result
# isn't overfitted to one particular hold sequence.
POSE_HOLD_SEED = 24680
