"""
settings/noise.py — SLAM localisation noise and cone-detection noise models.
"Does the car know exactly where it is, and does it see the cones exactly
where they are?"
"""
import numpy as np

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
# FSDS's cone map is a latched ORACLE: perception.SimPerception returns exact
# ground-truth cone positions, cropped only by range/FOV (see
# `docs/reference/simulator_fidelity.md`, "Simulator fidelity limits" — "Cone map...
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
# (tuner.validation.recorded_map_rollout --planner, SLAM off/cone on only):
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
