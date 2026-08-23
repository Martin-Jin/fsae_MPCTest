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
N_HORIZON = 35

# [shared] TERMINAL_Q_SCALE — "How much extra does the controller care about where it
# ends up at the very end of its plan, compared to every other step?"
# With no terminal cost or constraint, the MPC has exactly the same incentive
# to track well at the last predicted step as at every other step, and no
# incentive to leave itself in a good position for what happens just past the
# horizon. This affects both stacks identically (mpc_core.py has the same gap).
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
# speed regardless of this flag (rollout_core.py). Set True here only
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
# fsae_MPCTest/docs/planning_control_sync.md's speed-governor section for
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
# (mpc_controller.py / mpc_controller_standalone.py) — keep both in sync.
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
# (fsae_planning's mpc_controller.py / mpc_controller_standalone.py, which
# tracks a precomputed path instead of subscribing to the live planner's
# centreline): the equivalent offline experiment is USE_PLANNER=False above
# (or, for a recorded real track specifically, `python3 -m
# tuner.recorded_map_rollout <map.json> --oracle`) — see that flag's own
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
# Left OFF: with SLAM noise enabled, reversals/s measured 3.98-4.56 depending
# on N_HORIZON against live's 1.62 (2.5-2.8x too high); with it off,
# reversals/s=1.58 -- near-exact live parity. SLAM jitter (not cone noise,
# tested separately) is the dominant contributor to the excess. The mean|e_y|
# gap (stayed ~0.06-0.08 across every multiplier tried vs live's 0.346) is a
# separate, still-open issue this flag was never meant to close.
# Turn back ON only after re-calibrating SLAM_POS_JITTER_STD/
# SLAM_YAW_JITTER_STD against a current live log's reversals/s.
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
# Left at 0 by default as a deliberate "how much lag to simulate" knob, not
# because delay is unsafe (predict_ahead() rolls x0 forward through queued
# commands, so a nonzero DELAY_STEPS is safe to enable).

# ── Cone-detection noise ────────────────────────────────────────────────
# FSDS's cone map is a latched ORACLE: sim_track.SimPerception returns exact
# ground-truth cone positions, cropped only by range/FOV (see
# docs/planning_control_sync.md, "Simulator fidelity limits" — "Cone map...
# Not modelled anywhere" until this was added). Real cone detection has
# per-detection position error from the vision pipeline, which this models as
# jitter applied at the SimPerception boundary, independently per cone per
# frame (not per-frame-global, since real detection noise is dominated by
# each cone's own range/angle to the sensor, not a single shared offset).
#
# Why this exists: planning/cone_map.py's ConeMap._absorb() merges each new
# detection into whichever existing map entry is nearest, if within
# MERGE_DIST — otherwise it is appended as a brand-new permanent cone. Two
# jittered detections of the SAME physical cone, on the far side of MERGE_DIST
# from each other before either has been merged into the map, would be added
# as two separate permanent entries; nothing currently exercises that path
# because there is no detection noise. This flag exists to make that (and any
# other perception-noise-dependent behaviour) testable offline at all.
#
# Left OFF, same as SLAM_NOISE_ENABLED above. Isolated from SLAM noise
# (tuner.recorded_map_rollout --planner, SLAM off/cone on only):
# reversals/s=2.28, moderately above live's current 1.62 but nowhere near
# SLAM jitter's 4.56 alone -- cone noise is a smaller contributor to the
# excess, not the dominant one.
CONE_NOISE_ENABLED = False

# Per-cone, per-frame position jitter (independent white noise, redrawn every
# time a cone is returned as visible — NOT correlated frame-to-frame, unlike
# SLAM_POS_DRIFT_STD, because a vision cone detector re-estimates each cone's
# position from scratch every frame rather than tracking/filtering it).
# 0.05 m is a placeholder magnitude (typical stereo-vision cone-detection
# error at short range), not a measured FSDS or real-car constant — there is
# no measurement to fit this to yet. Treat it as tunable, same as
# SLAM_POS_JITTER_STD, and prefer measuring the real detector's noise before
# trusting any conclusion drawn from a specific value.
CONE_POS_JITTER_STD = 0.05   # m, per-cone-per-frame white noise on x/y

# CONE_NOISE_SEED — fixed so rollouts stay reproducible and CMA-ES compares
# candidate weight sets against the identical noise sequence (same rationale
# as SLAM_NOISE_SEED/DELAY_JITTER_SEED).
CONE_NOISE_SEED = 97531

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
# Planner tunables — MUST mirror fsae_bringup/config/fsae_params.yaml's
# centerline_planner block (live ROS params: smooth, look_radius, plan_horizon,
# path_blend). sim/sim_track.py::SimPlanner must pass these four values as
# explicit keyword args to build_path_walls()/blend_paths() — those functions
# have their own hardcoded defaults that silently diverge from the live-tuned
# values below (e.g. PLANNER_SMOOTH_PER_PT's live 0.015 vs. build_path_walls'
# default 0.05, and blend_paths' internal horizon default of 15.0 vs. the
# live-tuned 25.0), which is a real parity break if the keyword args are ever
# dropped. See sim_to_real_investigation.md S31 for the mechanism and its
# measured effect.
# ------------------------------------------------------------------------------
PLANNER_SMOOTH_PER_PT = 0.015   # m^2 smoothing budget per input point (splprep s = this * n_pts)
PLANNER_LOOK_RADIUS = 25.0      # m; omni-directional cone-map crop radius
PLANNER_PLAN_HORIZON = 25.0     # m; arc-length the published centreline is clamped to
PLANNER_PATH_BLEND = 0.4        # 0<a<=1; temporal EMA weight toward each freshly-planned path

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
# tuner/checks/ref_heading_limiter_suite_check.py before changing this.
# Default off until validated live.
REF_HEADING_RATE_LIMIT_ENABLED = False
REF_HEADING_RISE_RATE = 90.0   # deg/s — only used when the flag above is True

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
# Heavily penalises steering-rate-of-change on top of adaptive_R_rate's
# existing curvature softening, but only when the car is already centred
# (|e_y| small) AND not currently curving (kappa small) -- see
# controller/model_utils.py::steer_rate_anti_hunt for the exact thresholds
# and mechanism. "Corner ahead" is NOT detected via path lookahead here --
# it reuses the same causal, current-curvature signal as adaptive_R_rate, so
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

# [shared, both controllers, EXPERIMENTAL] Post-solve
# moving-average filter on the FINAL steering command, applied identically
# regardless of which controller (LTV-QP or NMPC) produced it -- mirrors
# ros2/.../mpc_controller_standalone.py's node-level Output smoothing block.
# Deliberately NOT a QP weight change: the solver's own Q/R/R_rate are
# untouched, so this can't silently override separately-tuned weights the
# way a cost-scheduling mechanism can (see NMPC_CORNER_RRATE_BLEND_ENABLED's
# history -- enabling that dropped an already-tuned NMPC_R_RATE_DELTA down
# to the LTV-QP's own unrelated, lower RRATE_STEER_STRAIGHT). This IS a
# genuine temporal filter and DOES add lag, unlike every QP-weight-based
# mechanism above (re-derived fresh from the current state each tick, no
# cross-tick memory) -- traded off by weighting it down (never fully off)
# as CURRENT curvature rises: full weight on a clean straight, fading toward
# OUTPUT_SMOOTHING_CORNER_FLOOR (never below it) as the car actually turns,
# so a sharp corner still gets a mostly-instant response.
OUTPUT_SMOOTHING_ENABLED = False
OUTPUT_SMOOTHING_ALPHA = 0.425        # EMA coefficient; lower = more smoothing/more lag
# CAUTION: this is the live-validated value. A lower alpha (heavier
# smoothing) can look strictly better on an offline reversals/s sweep, but
# the offline recorded-map rollout has almost no genuine disturbance to
# correct -- a slow filter reads as pure improvement there while live it
# produces a real sustained lateral drift once the EMA's own settle time
# (~3/(alpha*CONTROL_HZ) seconds) is too slow to track an actual
# correction. Re-tune only against a live A/B, not this offline metric.
OUTPUT_SMOOTHING_CORNER_FLOOR = 0.1   # min smoothing weight retained even at full curvature

# [shared, both controllers, EXPERIMENTAL] ALSO fade
# smoothing down (never below OUTPUT_SMOOTHING_CORNER_FLOOR) as CURRENT
# tracking error grows, on top of the curvature-based fade above -- large
# |e_y| or |e_psi| means the car needs its raw, fast-reacting command right
# now regardless of curvature (e.g. recovering from a disturbance on a
# straight, where curvature alone would keep smoothing at full strength).
# Same saturating-curve style as model_utils.steer_rate_anti_hunt
# (1/(1+k*|x|), independent per input, multiplied together), just fading the
# OUTPUT weight down instead of boosting a QP cost up. 0.0 disables the
# corresponding factor (saturates at 1.0, i.e. no extra fade from that term).
OUTPUT_SMOOTHING_K_EY = 0.8      # 1/m; higher = fades out faster per metre of |e_y|
OUTPUT_SMOOTHING_K_EPSI = 1.115  # 1/rad; higher = fades out faster per radian of |e_psi|

# [shared, both controllers, EXPERIMENTAL] Fade smoothing
# down BEFORE the car reaches a corner already visible in the path, not only
# once the car's own CURRENT curvature has risen. Motivated by a track
# (comp_test_map_3) where 61% of "straights" between corners are under 2s --
# shorter than this filter's own settle time at typical alpha values, so a
# purely current-curvature corner_frac only starts fading smoothing after
# the straight has mostly already ended, producing a sustained sway right
# where two corners are close together.
# OUTPUT_SMOOTHING_LOOKAHEAD_LEAD_S is a TIME lead (converted to a scan
# distance via the car's own current speed each tick, so the lead stays
# consistent across speed rather than a fixed metres value giving less
# warning at high speed exactly when more is needed), sized to roughly this
# filter's own ~95% settle time (3 / (alpha * CONTROL_HZ) seconds) so the
# fade has time to complete before the corner arrives. The scanned peak
# curvature (speed_profile.peak_kappa_ahead) goes through the SAME
# _corner_factor curve as the current-curvature signal, and the larger of
# the two wins -- so whichever fires first drives the fade.
# 0.0 disables (pure current-curvature corner_frac, unchanged behaviour).
# Numeric parity: mpc_controller(_standalone).py's
# 'output_smoothing_lookahead_lead_s' ROS2 parameter.
OUTPUT_SMOOTHING_LOOKAHEAD_LEAD_S = 0.5   # s of lead; 0.0 disables

# [LTV-QP only] ADAPTIVE_R_RATE_ENABLE_IN_CORNERS — TEMPORARY/EXPERIMENTAL, fsds sim only.
# (Renamed from ADAPTIVE_R_RATE_DISABLE_IN_CORNERS, whose True/False
# polarity was inverted from what the name suggested.) adaptive_R_rate
# (above STEER_RATE_ANTI_HUNT_ENABLED's mechanism, see
# controller/model_utils.py::adaptive_R_rate) normally SOFTENS the steering
# rate-of-change cost continuously as curvature rises, so the controller
# isn't over-penalised for the extra steering rate a corner demands. Keep
# this True to keep that reduction ACTIVE in corners via the continuous
# curve (no threshold, no discontinuity) -- this is the setting you want if
# the goal is "reduce R_rate when turning". Setting this False switches
# softening off once kappa exceeds adaptive_R_rate's own kappa_straight
# cutoff (0.03): in a corner, R_rate[0,0] gets the full, unscaled baseline
# cost instead of being relaxed -- the opposite of reduction. NOT VALIDATED.
# Must stay True — disabling causes severe lag specifically in corners, because
# the discontinuous R_rate[0,0] jump at the kappa_straight crossing likely
# spikes QP solver iterations / invalidates warm-starts every tick near the
# threshold.
ADAPTIVE_R_RATE_ENABLE_IN_CORNERS = True

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
Q_diag      = [6.0, 0.8, 1.65, 1.20, 5.5, 0.0, 0.0, 0.0]
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
R_rate_diag = [52.5, 5.0]

# [shared] R_A_ACCEL / R_A_BRAKE — separate effort weights for acceleration and
# braking. solve_mpc()'s a_cmd effort cost is r_a_accel*pos(a_cmd)^2 +
# r_a_brake*neg(a_cmd)^2 (see controller/optimiser.py), not R_diag[1]*a_cmd^2
# -- R_diag[1] is read only as the fallback default when a caller omits
# these. A single shared r_a weight cannot be tuned independently for
# acceleration vs. braking: lowering it to free up acceleration authority
# also weakens braking by the same amount, and live telemetry has shown the
# resulting asymmetry -- corners entered hot, steering saturating, unstable
# post-exit recovery -- because the same weight that frees up acceleration
# also caps how hard the QP is willing to brake. See planning_control_sync.md's
# "Accel/brake effort weight split" for the diagnosis and retuning history.
# Mirrors mpc_params.py's R_A_ACCEL/R_A_BRAKE. R_A_ACCEL being large is a
# big change from the diagnosis above (which argued for CHEAPER accel
# effort, not more expensive) -- this may be a response to a
# separately-diagnosed speed-tracking-lag issue (car slow to convert a
# falling v_desired into actual braking, see
# late_turn_in_investigation.md Part 11), not a reversal of that reasoning.
# Re-read Part 11 before assuming this number is settled.
R_A_ACCEL = 2.25
R_A_BRAKE = 0.5


# ------------------------------------------------------------------------------
# Nonlinear MPC (NMPC) — a SECOND controller (controller/nmpc_optimiser.py)
# ------------------------------------------------------------------------------
# [NMPC only] USE_NMPC — "Which controller does the closed-loop rollout actually solve?"
# False (default) = controller/optimiser.py's solve_mpc(), the linear
# time-varying QP everything above this section tunes. True =
# controller/nmpc_optimiser.py's NMPCController, a Frenet-frame NONLINEAR MPC
# (arc length is a state, path curvature kappa(s) is looked up from it
# directly, instead of the linear model's e_psi_dot = yaw-rate-only). See
# docs/junior_project_mpc_docs.md's §4.2 for the plain-language explanation
# of why the linear model needs this at all, and docs/tuning.md's NMPC
# section for the tuning surface.
#
# Closes a structural gap the linear model has: with the car exactly on-line
# and on-heading approaching a corner, the linear model's own horizon
# rollout predicts staying at 0 forever (checked directly, not assumed —
# see tuner/nmpc_offline_check.py's turn-in test), so no amount of cost
# reweighting can make it commit to steering before real tracking error
# exists. The WHOLE adaptive-gain-shape section below (and every
# ADAPTIVE_*_ENABLED flag) exists to synthesise anticipation this linear
# model cannot produce on its own -- none of it applies when USE_NMPC=True
# (the nonlinear model anticipates structurally, so layering the same
# mechanisms on top would double-count an effect that's now built in).
#
# Mirrors the live side's `use_nmpc` node parameter (mpc_params.py's "NMPC
# weight overrides" section handles the WEIGHTS below; this file's NMPC_*
# constants mirror nmpc_params.py's remaining structural/solver fields) --
# kept numerically identical by hand, the same discipline as every other
# constant in this file, per CLAUDE.md's parity rule. Land this off; prove
# it live before flipping the live side's default.
USE_NMPC = False

# ── NMPC weight overrides ────────────────────────────────────────────────
# [NMPC only] -1.0 = inherit the corresponding Q_diag/R_diag/R_rate_diag/TERMINAL_Q_SCALE
# entry above (the SAME weight set a tuner run passes to run_core_rollout,
# so a CMA-ES sweep reaches the NMPC's weights exactly the way it reaches
# the LTV-QP's). Set a real value to diverge only that one weight for the
# NMPC without touching the LTV-QP's tuned set.
#
# NMPC_Q_EPSI_DOT is the one weight whose MEANING differs from its LTV-QP
# counterpart (Q_diag[3]): the nonlinear model's 4th output is heading-error
# RATE (r - kappa(s)*s_dot), not absolute yaw rate. Penalising absolute yaw
# rate in a curvature-aware model would penalise the yaw rate the car MUST
# hold to follow a corner (r = kappa*v) -- the exact opposite of what's
# wanted. Same slot, different regressor: expect this one to need its own
# sweep rather than inheriting Q_diag[3] unchanged.
NMPC_Q_E_Y       = -1.0
NMPC_Q_E_YD      = -1.0
NMPC_Q_E_PSI     = -1.0
NMPC_Q_EPSI_DOT  = -1.0
NMPC_Q_E_V       = -1.0
NMPC_R_DELTA     = -1.0
NMPC_R_A_ACCEL   = -1.0
NMPC_R_A_BRAKE   = -1.0
NMPC_R_RATE_DELTA = -1.0
NMPC_R_RATE_A    = -1.0
NMPC_TERMINAL_SCALE = -1.0

# [NMPC only, EXPERIMENTAL] Reuses model_utils.steer_rate_anti_hunt
# (the same function STEER_RATE_ANTI_HUNT_ENABLED above already gates for the
# LTV-QP) on the NMPC too -- independent flag, not inherited, since the live
# nmpc_core.py module docstring documents a deliberate decision NOT to port
# mpc_core's adaptive gain-schedule family onto the curvature-aware NMPC
# (double-count risk); anti-hunt is offered separately because it only ever
# makes steering-rate MORE expensive when already centred/aligned/uncurving,
# the opposite direction from anticipation. UNVALIDATED for the NMPC. Note:
# model_utils.steer_rate_anti_hunt hardcodes boost_max=6.0 internally (no
# settings.py override exists for EITHER controller's use of it, pre-existing
# limitation, not introduced here) -- ANTI_HUNT_BOOST_MAX has no live
# offline-side constant to parameterise this with yet.
NMPC_STEER_RATE_ANTI_HUNT_ENABLED = False

# [NMPC only, EXPERIMENTAL] A narrower, ALTERNATIVE port of
# the LTV-QP's corner_factor family -- blends R_rate[0,0] between
# NMPC_RRATE_STEER_STRAIGHT/_CORNER by CURRENT curvature alone
# (model_utils._corner_factor/_blend, imported not reimplemented). Unlike the
# rest of that family (Q_EY/Q_EPSI/Q_R/R_STEER, deliberately excluded above),
# only R_rate[steer] is touched, to limit how much of the "no adaptive gain
# schedule" reasoning this overrides. NOT composed with
# NMPC_STEER_RATE_ANTI_HUNT_ENABLED -- takes priority over it when both are
# set; use one or the other, not both. UNVALIDATED for the NMPC.
NMPC_CORNER_RRATE_BLEND_ENABLED = False
NMPC_CORNER_FACTOR_K = -1.0        # -1 = inherit CORNER_FACTOR_K
NMPC_RRATE_STEER_STRAIGHT = -1.0   # -1 = inherit RRATE_STEER_STRAIGHT
NMPC_RRATE_STEER_CORNER = -1.0     # -1 = inherit RRATE_STEER_CORNER

# [NMPC only, EXPERIMENTAL] Applies
# model_utils.reversal_penalty_boost (the same function
# REVERSAL_PENALTY_ENABLED above already gates for the LTV-QP) to the NMPC too
# -- gain-scheduled per tick from LAST tick's steering command and applied
# uniformly across the horizon, exactly like the two flags above. Unlike those
# two, this one COMPOSES with either of them rather than replacing them: it is
# keyed on u_prev, not curvature/e_y/e_psi, so all three multipliers stack onto
# the same R_rate[0,0] (see nmpc_optimiser.compute_step's rrate_steer_current).
# Mirrors the live MPCParams.nmpc_reversal_penalty_* override fields.
# UNVALIDATED on the car; offline-A/B'd only.
# [NMPC only, EXPERIMENTAL] Discount the steering-RATE cost at the NEAR
# horizon stages (a linear ramp from NMPC_RRATE_STAGE_NEAR at stage 0 to 1.0
# at the last stage), so a first turn-in input is cheap while a sustained
# oscillation still pays close to full price -- see
# controller/nmpc_optimiser.py::_rrate_stage_ramp for the reasoning, and
# docs/steering_turn_in_upgrade_options.md (Option 1) for why this is keyed
# on horizon POSITION rather than measured curvature/error.
# NEAR = 1.0 is an exact no-op. Composes with the three flags below (they set
# the rate weight's magnitude; this shapes it across stages).
# [NMPC only, EXPERIMENTAL] Continuous three-zone schedule on the
# steering-RATE cost, driven by CURRENT curvature and the peak curvature the
# HORIZON predicts ahead: boost on a true straight, ease on the approach to a
# corner the horizon can see, floor through the corner itself. Smooth surface,
# no thresholds -- degrades to the corner value on a continuously-winding
# road. See controller/nmpc_optimiser.py::_rrate_zone_scale.
# Multiplies whatever r_rate_delta is (unlike NMPC_CORNER_RRATE_BLEND_ENABLED,
# which OVERWRITES it), so it composes with the shipped 52.5 rather than
# discarding it.
# [NMPC only, EXPERIMENTAL] Steering-JERK weight: penalises the SECOND
# difference of the steering command (steering acceleration) instead of only
# the first. A steady ramp into a corner has near-zero second difference and
# is nearly free; an alternating wiggle is expensive. Measured on live data,
# reversals carry ~4.3x the |d2| of same-direction ramps vs only ~1.9x the
# |d1|, so this separates chatter from turn-in about twice as sharply as the
# rate cost can. 0.0 disables the term entirely (no Hessian contribution).
# Intended to eventually let R_rate_diag[0] come back down from 52.5.
# See controller/nmpc_optimiser.py::_build_qp's _E2 comment.
NMPC_RJERK_DELTA = 0.0
NMPC_RJERK_A = 0.0

NMPC_RRATE_ZONE_ENABLED = False
NMPC_RRATE_ZONE_BOOST_STRAIGHT = 2.0    # x r_rate on a true straight
NMPC_RRATE_ZONE_EASE_APPROACH = 0.35    # x r_rate when a corner is AHEAD but not here yet
NMPC_RRATE_ZONE_FLOOR_CORNER = 0.15     # x r_rate mid-corner

NMPC_RRATE_STAGE_RAMP_ENABLED = False
NMPC_RRATE_STAGE_NEAR = 0.15

NMPC_REVERSAL_PENALTY_ENABLED = False
NMPC_REVERSAL_PENALTY_BOOST_MAX = -1.0  # -1 = inherit REVERSAL_PENALTY_BOOST_MAX
NMPC_REVERSAL_PENALTY_K = -1.0          # -1 = inherit REVERSAL_PENALTY_K

# ── Structural / solver settings ─────────────────────────────────────────
# [NMPC only] No LTV-QP counterpart to inherit from (there's no "linear horizon length"
# concept these could default to) -- these are genuine NMPC-only constants,
# each measured rather than guessed; see the live repo's
# late_turn_in_investigation.md Part 16 §16.7 for the sweep behind the
# horizon/iteration defaults specifically.
NMPC_HORIZON = 20                          # steps (x DT=0.05s = 1.0s). Measured BETTER than
                                            # N_HORIZON=35 on tracking: the model is optimistic
                                            # (linear tyres, no suspension), and that mismatch
                                            # compounds over a longer horizon.
NMPC_SQP_ITERS = 1                         # Gauss-Newton iterations/tick (real-time-iteration
                                            # style -- the warm start carries convergence
                                            # across ticks). Measured better AND ~2x cheaper
                                            # than 2.
NMPC_SOLVE_BUDGET_MS = 25.0                # wall-clock budget/tick; ships the best feasible
                                            # iterate rather than overrunning DT=0.05s.
NMPC_RK_SUBSTEPS = 2                       # RK4 substeps in the prediction rollout -- needed
                                            # because tau_a=0.02s is stiff against DT=0.05s.
NMPC_JAC_SUBSTEPS = 1                       # RK4 substeps for the QP's sensitivity Jacobians
                                            # only (never the prediction itself) -- the
                                            # dominant per-iteration cost, deliberately coarser.
NMPC_TRUST_DELTA_RAD = np.radians(9.0)      # per-iteration steering trust region = MAX_STEER's
                                            # own slew-rate limit per tick (180 deg/s * DT) --
                                            # reused, not invented.
NMPC_TRUST_A = 0.6                          # per-iteration accel trust region = du_max[1].
NMPC_BACKTRACK_MAX = 2                      # step halvings if a full SQP step increases the
                                            # true nonlinear cost (divergence guard).
NMPC_TRACK_HALFWIDTH = 3.5                  # soft |e_y| bound with slack, matching
                                            # controller/optimiser.py's own +-3.5m literal.
NMPC_SLACK_WEIGHT = 10000.0                 # matches controller/optimiser.py's W_SLACK.
NMPC_CURVATURE_DENSE_STEP = 0.5             # kappa(s)/heading-reference smoothing -- same
NMPC_CURVATURE_SMOOTH_W = 3                 # denoise precedent as sim/speed_profile.py's
                                            # curvature_speed() (dense_step=0.5, w=3), not new
                                            # smoothing constants.
NMPC_KAPPA_CLIP = 0.5                       # hard |kappa(s)| clamp -- a 2m-radius guard,
                                            # inert on any real track line, only catches a
                                            # degenerate/spiking path.
NMPC_OSQP_MAX_ITER = 500                    # bounded well below solve_mpc's ~8000: this is a
                                            # step DIRECTION validated by the backtracking cost
                                            # check before being kept, so a hard subproblem
                                            # should cost bounded time and be retried next tick.
NMPC_OSQP_EPS = 1e-4                        # looser than solve_mpc's 1e-5 on purpose -- a step
                                            # direction the next iteration corrects doesn't need
                                            # sub-1e-4 accuracy.
NMPC_ALAT_CEILING_ENABLED = True            # model FSDS's measured sustained a_lat ceiling
                                            # (ALAT_CEILING_FLAT/_SLOPE/_INTERCEPT below) inside
                                            # the NMPC's own prediction. NOT optional on FSDS:
                                            # without it the linear-tyre prediction believes it
                                            # can hold any corner at any speed, and the car spins
                                            # mid-lap (see Part 16 §16.6, live repo). False only
                                            # for real-vehicle work where that ceiling doesn't
                                            # exist.

# [NMPC only] NMPC_SPLINE_REFERENCE_ENABLED -- True (default): PathReference builds kappa(s)/
# psi_ref(s) from an analytic CubicSpline fit to the raw waypoints (x(s), y(s)
# each independently splined over cumulative arc length), instead of the old
# dense-resample + moving-average + finite-difference pipeline. A strict
# numerical-quality improvement to the documented "centreline curvature
# spikes" defect (see CLAUDE.md) with no new coupling to solver dynamics, so
# it defaults ON unlike the two flags below -- but is still flagged so the
# old moving-average path (kept, not deleted) can be A/B'd against it if a
# regression shows up. False restores the pre-existing behaviour exactly.
NMPC_SPLINE_REFERENCE_ENABLED = True

# [NMPC only] NMPC_HORIZON_SPEED_PROFILE_ENABLED -- False (default, EXPERIMENTAL): sample a
# precomputed per-lap speed profile v(s) at each horizon stage's own
# PREDICTED arc length s_k (PathReference.v_ref_at), instead of holding a
# single scalar v_ref constant across the whole horizon. Mirrors kappa(s)'s
# own non-schedulable, state-keyed lookup on purpose -- see PathReference's
# docstring and nmpc_core.py's module docstring on why curvature-as-
# exogenous-horizon-data produced wrong-direction transients in three earlier
# attempts; v_ref(s) is wired the same way specifically to inherit that
# property. Only takes effect when a speed-profile array is actually
# supplied to PathReference (see run_core_rollout's NMPC construction) --
# with no such array, or with this False, v_ref is the exact same frozen
# scalar as before. NMPC-only; the LTV-QP (mpc_core.py) is untouched.
NMPC_HORIZON_SPEED_PROFILE_ENABLED = False

# [NMPC only] NMPC_FRICTION_CIRCLE_ENABLED -- False (default, EXPERIMENTAL): add a HARD
# per-axle |F_yf|/|F_yr| bound to the condensed QP (on top of, not instead
# of, the existing SOFT alat-ceiling tanh saturation inside _f/_f_scalar --
# see CLAUDE.md's strong warning against touching that mechanism, which this
# does not). The bound is derived from the SAME measured ceiling law
# (ALAT_CEILING_FLAT/_SLOPE/_INTERCEPT) via F_max = m * ceiling(v_x) / 2 per
# axle (see nmpc_optimiser.py's _fmax_flat/_fmax_slope/_fmax_intercept for
# the exact conversion). When False, _build_qp/_outputs/_output_jacobians/
# _solve_step are all IDENTICAL (same array shapes, same QP dimensions) to
# before this feature existed -- not just "the extra rows are empty".
NMPC_FRICTION_CIRCLE_ENABLED = False

# [NMPC only] NMPC_SPEED_LIMIT_ENABLED -- False (default, EXPERIMENTAL): add a
# SOFT (slack-backed) v_x_k <= v_ref_at(s_k) + NMPC_SPEED_LIMIT_MARGIN +
# slack_v_k row to the condensed QP at EVERY horizon stage, using the same
# state-keyed PathReference.v_ref_at(s_k) lookup as
# NMPC_HORIZON_SPEED_PROFILE_ENABLED and the same slack-with-weight pattern as
# the existing soft track-bound rows. Never a HARD bound like
# NMPC_FRICTION_CIRCLE_ENABLED -- that one's zero-slack hard bound went
# infeasible under ordinary cornering and stalled the car (see
# docs/planning_control_sync.md), so this one always carries slack and can
# never make the subproblem infeasible.
#
# Added 2026-08-19 because NMPC_HORIZON_SPEED_PROFILE_ENABLED's COST term
# alone was live-tested and REJECTED: the QP just SUMS (v_x - v_ref)^2 across
# stages with no ordering constraint, so the solver can trade a bad early
# (in-corner) residual against a good late (post-corner) one in the SAME
# solve -- it never has to actually slow down in time, only make the total
# look good. Measured live: v_actual ~16.5 m/s against v_ref already down to
# ~7-9 m/s on corner entry, with worse off-track excursions than doing
# nothing. A per-stage INEQUALITY cannot be traded away that way: each
# stage's bound must individually hold.
#
# Only takes effect when a speed-profile array is actually supplied to
# PathReference (see run_core_rollout's NMPC construction / ref.v_target),
# exactly like NMPC_HORIZON_SPEED_PROFILE_ENABLED's own gating; the two flags
# are independent and either, both or neither can be enabled. Default False:
# genuine experiment, not yet live-tested.
NMPC_SPEED_LIMIT_ENABLED = False
NMPC_SPEED_LIMIT_MARGIN = 0.5               # m/s added on top of v_ref_at(s_k) before the
                                            # bound applies, so ordinary tracking noise around
                                            # the profile doesn't constantly engage slack.
                                            # 0 = bound exactly at the profile's own value.
NMPC_SPEED_LIMIT_SLACK_WEIGHT = 200.0       # penalty on the speed-limit slack -- same role as
                                            # NMPC_SLACK_WEIGHT for the track bound, but a
                                            # separate, much smaller constant: a few m/s of
                                            # overshoot for a tick or two while braking is
                                            # expected and should cost noticeably less than
                                            # actually leaving the track (10000), not be pinned
                                            # to zero as aggressively.


# ------------------------------------------------------------------------------
# Adaptive-gain SHAPE constants
# ------------------------------------------------------------------------------
# The *_ENABLED flags further up decide WHETHER each adaptive-gain mechanism
# runs; these decide the SHAPE of the curve it applies — the floors, ceilings
# and ramp sharpnesses. Each is a keyword argument of the function that uses
# it in controller/model_utils.py, defaulting to the value below, so this
# file is the single place any of them gets tuned. Read the referenced
# function's docstring for the mechanism before changing one — these are all
# tuned values, not arbitrary defaults.
#
# Mirrors the live side's MPCParams (mpc_params.py) field-for-field; keep the
# numbers identical across both per CLAUDE.md's planning/control parity rule.

# [LTV-QP only] adaptive_R_rate's softening floor on the steering rate-of-change cost
# R_rate[0,0] — "how much of the rate penalty survives in a corner?" Driven
# by the car's CURRENT curvature. Raising it means less softening (more
# damping, but a controller more penalised for the steering rate a corner
# needs); lowering it too far is what let steering sign-reversal chatter
# grow mid-corner. See controller/model_utils.py::adaptive_R_rate.
#
# Only the current-curvature floor is implemented (no forward-scan
# entering-floor).
ADAPTIVE_R_RATE_DURING_FLOOR = 0.625

# FSDS's fitted sustained lateral-acceleration ceiling law,
# a_lat_max(v) = max(FLAT, SLOPE * |v| + INTERCEPT) in m/s^2. This is a
# MEASURED property of the simulator (see CLAUDE.md's "dynamically-enforced
# lateral-acceleration ceiling"), not a free tuning knob. Still load-bearing
# for the NMPC's own in-prediction ceiling model (NMPC_ALAT_CEILING_ENABLED
# above) even though the LTV-QP no longer has a lookahead
# demand-normalisation reading these directly. Must stay in sync with
# model/vehicle_physics.py's alat_ceiling_at().
ALAT_CEILING_FLAT = 7.5
ALAT_CEILING_SLOPE = 0.47
ALAT_CEILING_INTERCEPT = 2.46


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
# WHY THIS EXISTS (measured)
# ---------------------------
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
                 #    metric of the 12 — near-zero effective contribution at
                 #    a naively larger scale, hence the small value here)
        0.70,    # 10 peak_lateral_error     (0.57-0.82 m)
        2.30,    # 11 speed_rmse             (1.86-2.83 m/s)
        0.08,    # 12 accel_reversal_rms     (0.037-0.123 measured directly on
                 #    VALIDATION_SUITE at current tuned weights, use_planner=False)
    ],
    dtype=float,
)
assert len(METRIC_SCALES) == 13
assert np.all(METRIC_SCALES > 0.0), "METRIC_SCALES must be strictly positive (used as a divisor)"


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
# since the 12 metrics have wildly different natural magnitudes
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
assert len(SCORE_WEIGHTS) == 13

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
# by ~0.9 and swamps all twelve continuous quality metrics. Measured effect:
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


# ==============================================================================
# CONSTRAINED SCORING STRUCTURE
# ==============================================================================
# A single weighted sum of 12 metrics plus additive bonuses and penalties is
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
#   3. QUALITY GROUP     — the 12 metrics, kept as a weighted sum (they really
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