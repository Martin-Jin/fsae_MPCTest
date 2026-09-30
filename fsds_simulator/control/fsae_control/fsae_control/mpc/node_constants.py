"""
fsae_control/mpc/node_constants.py — tuning constants of the MPC control node

Cone-brake corridor, path timeout and the speed-target rate limits. Shared by
mpc_controller.py and control_step.py. Each value mirrors the offline
sim/rollout constant of the same name where one exists; keep them in sync.
"""

CONTROL_HZ = 20.0   # must match MPCController(dt=0.05); dt = 1 / CONTROL_HZ

# CONE_BRAKE_DIST is also the ceiling on the dynamic corridor computed in
# _check_cone_proximity() (car_speed * 0.25, clipped to [0.6, CONE_BRAKE_DIST]).
# Only used when standalone_output=true (cone braking is fsds_bridge's job
# otherwise).
CONE_BRAKE_DIST      = 2.0    # m — forward corridor depth for cone proximity brake
CONE_BRAKE_WIDTH     = 0.18   # m — lateral half-width of braking corridor (36 cm total)
CONE_RESET_THRESHOLD = 0.3    # s — continuous cone-brake duration before one MPC reset
PATH_TIMEOUT         = 0.5    # s — reset the MPC if no fresh trajectory within this window

# Max rate (m/s^2) at which the speed TARGET may rise. Mirrors
# sim/rollout/core.SPEED_TARGET_RISE_RATE — keep both in sync. Decreases are
# never rate-limited; delaying a genuine brake request is the failure this is
# meant to prevent.
SPEED_TARGET_RISE_RATE = 7.0

# Max speed error (m/s) the rise limiter is allowed to open up before it stops
# ramping and waits for the car. Mirrors sim/rollout/core.py's constant of the
# same name — keep in sync.
#
# SPEED_TARGET_RISE_RATE alone assumes the car can accelerate at that rate. From
# a standing start it cannot: the car does not break static friction for ~1 s,
# so the target ramps to ~7 m/s while the car is still stationary and banks a
# deficit it spends the next second chasing. The NMPC minimises one scalar cost
# over the horizon, so a speed error that large swamps the lateral term and the
# optimiser trades e_y away for speed it was never going to get — measured live
# as a sideways excursion at launch that self-corrects once the car is rolling.
#
# Capping the DEFICIT rather than gating on measured speed is deliberate. A gate
# of the form "hold the target while v_actual is near zero" deadlocks: no target
# means no speed error, which means no throttle, which means the car never moves
# and the gate never opens. Holding at v_actual + DEFICIT_MAX always leaves a
# real speed error, so throttle still commands and the launch still happens; the
# ramp resumes by itself as the car closes the gap.
#
# Not specific to launch: the same rule stops the target running away after a
# spin or a heavy brake, for the same reason.
#
# 5.0, not the original 2.5. At 2.5 the clamp is not a launch/recovery guard
# at all, it is the binding constraint on acceleration for a THIRD of a
# normal lap: measured OFFLINE 36.8% of ticks pinned at exactly the limit,
# holding a_cmd to 4.45 against a plant that delivers ~12. Raising it to 5.0
# drops the pinned fraction to 2.3%, nearly doubles peak a_cmd to 8.32, and
# improves every metric at once rather than trading any against another:
#
#   DEFICIT_MAX   score (3 runs)        lap steps   a_cmd max   |e_y| mean   steer sat
#   2.5           0.757/0.804/0.757     1081-1117   4.45        0.418        4.71%
#   5.0           0.693/0.692/0.693     1033-1034   8.32        0.402        3.77%
#
# Lower score is better. The launch behaviour the clamp exists to protect is
# unchanged (launch at step 9 either way, launch-phase |e_y| 0.27 m against
# a 3.5 m boundary). Values above ~5 buy nothing further (10.0 and 100.0
# both plateau at a_cmd 8.87), so this is the knee, not a ceiling to keep
# raising.
#
# NOT YET LIVE-VALIDATED. This is an offline-only result, and the documented
# sim-to-real gap (live saturates ~4x more often than the sim) is exactly
# the failure mode of trusting one. Faster corner entry is the specific risk
# to watch on the car. See docs/logs/nmpc_progress_term_investigation.md.
#
# Promoted to a real MPCParams field (params.speed_target_deficit_max,
# tunable via ROS param/YAML/launch arg/GUI) rather than this module
# constant; see mpc_params.py's "Speed-target deficit clamp" section for
# the current default and rationale. No module-level constant remains.
# Max rate (m/s^2) at which curvature_speed()'s OWN output (v_curv, the live
# per-tick geometry-derived target, NOT the precomputed-track oracle lookup)
# may fall, applied before the tracking-error gate. curvature_speed() is a
# pure per-tick function with no memory of its own last output, and its
# docstring already documents that the live planner path (re-fit every
# frame) carries a few cm of lateral wiggle that survives its internal
# denoising often enough to swing v_curv by 3-10 m/s in a single 50 ms tick
# even on a straight or gentle bend (measured live 2026-09-15, see
# planner_only_speed_target_oscillation.md) -- SPEED_TARGET_RISE_RATE does
# not catch this, it only bounds the composed target's RISE, and this same
# noise is the actual DROP.
#
# FIRST attempt (2026-09-15) sized this at A_BRAKE_PLAN (control_utils.py,
# 5.0 m/s^2), reasoning that curvature_speed()'s own braking-distance
# propagation already assumes that deceleration is enough to plan a genuine
# corner's slowdown, so a cap at that rate should never bind on real
# braking. That reasoning had a gap: it assumes the target had the full
# scan-window distance to ramp down over, but the corner speed can firm up
# to its true low value only once the car is already close (after the noisy
# early-window estimate settles), leaving less runway than the planning
# assumption presupposes. Measured live the same day: with the 5.0 cap in
# place, the car entered the first corner at ~17 m/s and took 3+ seconds to
# reach the ~2.5 m/s target, spinning out well before it got there
# (e_psi -> -98 deg, stalled). 5.0 m/s^2 was capping GENUINE required
# braking, not just noise.
#
# Sized instead at MAX_BRAKE (lmpc/controller.py, 7.0 m/s^2, matching
# vehicle_physics.max_accel_brake): the car's actual achievable braking
# deceleration, not a conservative planning-time assumption. This still
# smooths a single noisy tick's collapse (which asks for far more than 7.0
# m/s^2 worth of change) across a few ticks, but no longer throttles a
# genuine hard-braking need down below what the car can physically do.
V_CURV_FALL_RATE = 7.0
# Max rate (gate-units/s, gate in [floor, 1.0]) at which
# tracking_error_speed_gate()'s output may change per tick, in EITHER
# direction. Without this, a fast-growing e_y sweeping through the gate's
# active band can compound with a simultaneously falling curvature-based
# speed target into a sharp single-tick v_desired drop that bypasses
# SPEED_TARGET_RISE_RATE (that limiter only bounds RISES), producing erratic
# a_cmd right after. Rate-limiting the gate itself spreads the same total
# slowdown over several ticks instead of one, keeping the safety response
# (the car DOES still slow down when tracking badly) while removing the
# single-tick cliff. Sized to the same order of magnitude as
# SPEED_TARGET_RISE_RATE by design choice, not measurement.
GATE_RATE_LIMIT = 2.0
