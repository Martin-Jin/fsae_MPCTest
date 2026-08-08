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
# Changed 25 -> 35 (2026-08-08): swept on PATH_SUDDEN_TURN (long straight ->
# sharp 90 deg, R=4.5m) with use_planner=False, current Q_diag/R_diag/
# R_rate_diag -- N=35 was the peak (peak lateral error 0.527m -> 0.450m,
# score 0.474 -> 0.441; N=50/70 drift slightly worse again, so this is a
# real optimum, not "higher is always better"). Neutral on the full recorded
# map (score 0.411 -> 0.412, sat% unchanged at 0.00) -- helps the sharp-corner
# case, doesn't cost anything elsewhere. Does NOT fix the late-turn-in/
# saturation failure mode itself: commit timing barely moved (5.95s -> 6.0s)
# because the reference heading is genuinely ~0 until the car's own
# arc-length position reaches the corner's start -- there's no earlier
# signal for a longer horizon to react to. See sim_to_real_investigation.md
# for the fuller writeup (terminal_scale and R_rate were swept too and
# ruled out as the commit-timing driver).
N_HORIZON = 35

# TERMINAL_Q_SCALE — "How much extra does the controller care about where it
# ends up at the very end of its plan, compared to every other step?"
# Added 2026-08-08 (sim_to_real_investigation.md S40): with no
# terminal cost or constraint, the MPC has exactly the same incentive to
# track well at the last predicted step as at every other step, and no
# incentive at all to leave itself in a good position for what happens just
# past the horizon. This affects BOTH stacks identically (mpc_core.py has
# the same gap) since it's a genuine structural omission, not a sim/live
# mismatch.
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
# Default changed to False (2026-08-08) to match the live ROS side's
# path_map_path mode (precomputed path, no planner/perception in the loop) —
# see the "Offline parity note" comment below. Set True to re-enable the
# planner-in-loop rollout (perception mistakes, live-built centreline).
USE_PLANNER = False

# USE_PRECOMPUTED_SPEED_PROFILE — "For a track that's already been mapped
# (cone positions known), skip the live per-tick speed re-derivation and use
# the full-map speed profile computed once, up front, from the whole path."
#
# curvature_speed() (sim/speed_profile.py) re-derives the target speed every
# tick from only the sub-path currently visible/built — by design, since a
# real car doesn't have the map on its first lap. But measured directly on
# the recorded comp-test map (2026-08-08): the live-built centreline is
# SHORTER than curvature_speed()'s own assumed scan_end=24m on 100% of
# steps (median 21.6m, <15m on ~20% of steps, <10m on ~8% — almost
# certainly at the sharper corners, where the perception FOV's lateral
# window clips before its forward window does). That silently gives the
# speed planner less runway than its own braking-distance design assumes,
# on every tick — a second, independent contributor to "brakes too late"
# alongside the MPC's own fixed-time (not fixed-distance) horizon (see
# docs/sim_to_real_investigation.md S48).
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

# Offline parity note for the live ROS side's path_map_path param
# (fsae_planning's mpc_controller.py / mpc_controller_standalone.py, which
# tracks a precomputed path instead of subscribing to the live planner's
# centreline): the equivalent offline experiment is USE_PLANNER=False above
# (or, for a recorded real track specifically, `python3 -m
# tuner.recorded_map_rollout <map.json> --oracle`) — see that flag's own
# comment for what it does. No separate flag needed here; USE_PLANNER=False
# already removes the planner from the rollout and tracks path_X/path_Y/
# path_Psi (the same oracle path tuner/export_speed_profile.py exports for
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
# CHANGED TO ON 2026-08-08 (sim_to_real_investigation.md S43), then back OFF
# 2026-08-08 (same day, later session): measured against that day's live log
# it moved reversals/s 0.99->3.06, "right in live's 3.48" AT THE TIME. Live
# has since been re-measured at 1.62 -- the calibration target moved and this
# magnitude no longer matches it. Re-measured directly on the recorded map
# (tuner.recorded_map_rollout --planner): with this ON, reversals/s=3.98-4.56
# depending on N_HORIZON, vs live's current 1.62 (2.5-2.8x too high); with it
# OFF, reversals/s=1.58 -- near-exact live parity. SLAM jitter (not cone
# noise, tested separately) is the dominant contributor to the excess.
# The mean|e_y| gap this was never meant to close (stayed ~0.06-0.08 across
# every multiplier tried vs live's 0.346) is still separate and still open.
# Turn back ON only after re-calibrating SLAM_POS_JITTER_STD/
# SLAM_YAW_JITTER_STD against a current live log's reversals/s, not the
# stale 3.48 target above.
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
# CHANGED TO ON 2026-08-08, then back OFF 2026-08-08 (same day, later
# session) alongside SLAM_NOISE_ENABLED above -- see that flag's comment for
# the re-measurement against a current live log. Isolated from SLAM noise
# this time (tuner.recorded_map_rollout --planner, SLAM off/cone on only):
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
# path_blend). Found 2026-08-08: sim/sim_track.py::SimPlanner called
# build_path_walls()/blend_paths() with NO keyword args at all, silently
# falling back to those functions' own hardcoded defaults instead of the
# live-tuned values below — a real, previously undetected parity break, live
# the whole time this investigation's offline rollouts have been run. Two of
# four values (look_radius, path_blend's alpha) happened to already match by
# coincidence; PLANNER_SMOOTH_PER_PT (0.015 live vs 0.05 hardcoded default)
# and blend_paths' internal horizon (25.0 live vs 15.0 hardcoded default) did
# not. See sim_to_real_investigation.md S31 for the fix and its measured
# effect on every prior finding in this document.
# ------------------------------------------------------------------------------
PLANNER_SMOOTH_PER_PT = 0.015   # m^2 smoothing budget per input point (splprep s = this * n_pts)
PLANNER_LOOK_RADIUS = 25.0      # m; omni-directional cone-map crop radius
PLANNER_PLAN_HORIZON = 25.0     # m; arc-length the published centreline is clamped to
PLANNER_PATH_BLEND = 0.4        # 0<a<=1; temporal EMA weight toward each freshly-planned path

# REF_HEADING_RISE_RATE — "How fast is the planner's steering target allowed
# to swing before we start holding it back?"
# sim_to_real_investigation.md S26/S27 found the planner's published centreline
# sometimes points much further into an upcoming corner than the car has
# actually turned yet ("anticipating" a corner early) — a real but sustained
# effect (not a single bad frame), concentrated at sharp braking corners, and
# strongly linked to steering saturation (up to 18x more likely on the ticks
# where this happens). This limiter caps how fast the reference heading the
# controller tracks (ref_psi) is allowed to change per second, exactly like
# SPEED_TARGET_RISE_RATE already does for the speed target — the raw
# direction is still used once the car catches up, this only slows how fast
# the TARGET moves, so the controller isn't asked to snap onto a heading the
# car has no chance of reaching yet.
#   - Increase it (or disable): the controller reacts to the planner's full
#     corner-anticipation immediately — can drive the reported saturation gap
#     in S26/S27 but may also mean earlier, more confident turn-in.
#   - Decrease it: smoother, later turn-in, but risks entering a tight corner
#     with too little heading correction already applied ("understeering in").
#   - Units: deg/s. Only the MAGNITUDE of change is capped; sign (turning
#     left vs. right) is never touched, so this cannot reverse a correction.
#
# MEASURED (sim_to_real_investigation.md S28): 90 is the tightest value with
# NO DNF anywhere in settings.VALIDATION_SUITE. It cuts recorded-map
# saturation 4.62%->3.07% and suite-mean saturation 8.99%->6.02% with no
# per-path regression. Values below ~85 look even better on the recorded map
# (65 reaches 0.00% saturation there) but DNF PATH_MICRO_SLALOM off-track —
# the reference is held back so hard the car cannot keep up on a fast, tight
# slalom. Do not lower this without re-running
# tuner/ref_heading_limiter_suite_check.py; the recorded map alone hid this
# failure mode completely.
# Default OFF until validated live — see S28 for the full sweep before
# enabling on the car.
REF_HEADING_RATE_LIMIT_ENABLED = False
REF_HEADING_RISE_RATE = 90.0   # deg/s — only used when the flag above is True

# ADAPTIVE_Q_SCALING_ENABLED — "Should the controller relax its lateral-error
# penalty when it's already close to the centreline, to stop small-error
# hunting?"
# Added 2026-08-08 after a live log showed steering-reversal rate
# INCREASING as |e_y| got smaller (35.6% of ticks at |e_y|<0.05 m, down to
# 2.4% at |e_y|>0.6 m) — the car darting back and forth across the
# centreline instead of settling onto it. See
# controller/model_utils.py::adaptive_Q_scaling for the full mechanism and
# sim_to_real_investigation.md S42 for the measurement.
# NOT REPRODUCED on the offline recorded-map rollout as currently tuned
# (there, reversal rate rises WITH |e_y|, the opposite trend) — this may be
# a live-only symptom. Default OFF until validated: re-run
# VALIDATION_SUITE/recorded-map for new DNFs before enabling, then validate
# on a live log the same way S28's reference-heading limiter was.
ADAPTIVE_Q_SCALING_ENABLED = False

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
Q_diag      = [8.835061533166446, 0.10074710969078902, 3.121860429243342, 0.10204777472070867, 9.944842732101566, 0.0, 0.0, 0.0]
R_diag      = [0.96036050771207, 2.7854960715243156]
R_rate_diag = [1.10425012508917786, 2.60031073385853]
# Three further hand edits 2026-08-08 (live edits, resynced here), all part
# of chasing live accel/brake jitter on straights (accel_rms_mps2 2.67 after
# R_diag[1]'s 0.25x cut, 2.06 after Q_diag[4]'s cut below, 1.81 after this
# round -- measurably better each time but not eliminated; not yet
# reproduced offline on the recorded map/oracle profile, so these are live
# hand-tuning steps, not offline-validated optima like R_diag[1]):
#   Q_diag[4] (speed error): 9.715386646449979 -> 3.715386646449979
#   R_diag[0] (steering effort): 0.44113286130397317 -> 0.84113286130397317
#   R_rate_diag[0] (steering-rate): 5.856634028761815 -> 1.856634028761815
# CMA-ES should re-tune properly in a full run once the jitter's root cause
# is understood -- these are corrective, not final, values.
# R_diag[1] (accel/brake effort cost) lowered 2.11423973420019 -> 0.25x
# (2026-08-08): swept on PATH_SUDDEN_TURN with use_planner=False, N_HORIZON=35
# (see that setting's own comment) -- the MPC was only commanding ~-3.0 to
# -3.2 m/s^2 of braking approaching the corner, about a third of the
# vehicle's actual -9.0 m/s^2 limit, leaving a ~2 m/s speed-tracking gap
# (v vs v_target) right at corner entry. 0.25x closes about half that gap
# (2.13 -> 1.15 m/s), peak e_y 0.450 -> 0.398m, score 0.441 -> 0.420. Below
# 0.25x it plateaus/slightly reverses (0.05x-0.02x actually re-widened the
# gap to ~1.0-1.06m and score ticked back up), so this is a real optimum,
# not "lower is always better" -- unlike R_rate and terminal_scale, which
# were also swept on the same corner and had no effect on commit timing.


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
        0.09,   # 1  yaw_rms                 (was 0.06 — live standalone-ROS test data
                #    2026-08-05 showed yaw_rate swinging +0.9/-1.1 rad/s within a few
                #    hundred ms in corners; CMA-ES had too little pressure to avoid
                #    this via the composite score, raised so oscillatory yaw actually
                #    costs the tuner something. Offset below.)
        0.040,  # 2  smooth_rms               (was 0.085, then 0.065 — trimmed again,
                #    0.025 taken to help fund accel_reversal_rms below, on the same
                #    logic as the original trim: this metric only blunt-instrument
                #    reacts to accel/brake flip-flopping via (u_opt-u_prev)^2 without
                #    isolating reversal count/magnitude the way the new metric does)
        0.02,   # 3  steer_rms
        0.015,  # 4  accel_rms               (was 0.005 — too small to give CMA-ES
                #    any real gradient on throttle/brake effort; nudged up)
        0.03,   # 5  max_steering
        0.045,  # 6  steering_sat_ratio       (was 0.075 — trimmed to offset the
                #    yaw_rms/steering_reversal_rate raise; less directly related to
                #    the oscillation/chatter symptom than the two raised terms)
        0.020,  # 7  jerk_rms                 (was 0.045 — trimmed 0.025, same
                #    reasoning as smooth_rms above: jerk_cost reacts to a sign flip's
                #    large du but conflates it with any other jerky-but-monotonic
                #    accel change, unlike the new dedicated reversal metric)
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
        0.05,   # 12 accel_reversal_rms      (NEW 2026-08-08 — the identical
                #    magnitude-weighted-swing construction as steering_reversal_rms
                #    above, applied to a_cmd instead of delta_cmd. Added after live
                #    logs showed persistent throttle/brake sign-flip chatter with NO
                #    corresponding cost term anywhere in the score: smooth_rms/
                #    jerk_cost react to a_cmd's tick-to-tick delta but can't
                #    distinguish a reversal from any other jerky-but-same-sign
                #    change, and nothing else even looks at u_opt[1]'s sign. Given
                #    the same weight as steering_reversal_rms since the two are the
                #    same behaviour on the two different actuators; funded by
                #    trimming smooth_rms/jerk_rms by 0.025 each (see their comments)
                #    rather than the tracking terms. METRIC_SCALES entry is a
                #    PLACEHOLDER (1.0) until measured on VALIDATION_SUITE — see that
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
# As of the 2026-08-06 constrained restructure (see CONSTRAINT_FLOOR below),
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


# ==============================================================================
# POSE FEED HOLD (sim-to-real: the pose sometimes stops updating)
# ==============================================================================
# POSE_HOLD_ENABLED — "Does the simulated controller sometimes get handed the
# SAME pose it got last tick, instead of a fresh one?"
# On the real car, /fsae/slam/car_position intermittently stops publishing and
# the controller re-uses its last known pose while the car keeps moving. The
# offline rollout used to hand it a brand-new exact pose every single tick, so
# heading error could never accumulate — which is exactly why the simulator
# showed smooth driving while the car wobbled on the same track with the same
# weights.
#
# Measured on live telemetry (2026-08-06, two runs, same track, same tuned
# weights, differing only in how badly the feed stalled):
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
# CONSTRAINED SCORING STRUCTURE (added 2026-08-06)
# ==============================================================================
# The score used to be one weighted sum of 12 metrics plus additive bonuses and
# penalties. That is "linear scalarisation", and it has a structural limit: a
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