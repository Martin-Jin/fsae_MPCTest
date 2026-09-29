"""
sim/rollout/speed_target.py — the speed-target phase of run_core_rollout(),
lifted out of its loop so the loop itself reads as a sequence of named
steps.

Stateless: any value the loop carries from one tick to the next
(gate_prev, v_des_prev, v_curv_prev, ...) is passed in and returned
explicitly rather than stored, so the loop in sim/rollout/core.py remains
the one place that owns rollout state.

settings.py constants are bound here by name at import time, the same as
elsewhere in this package. Callers that override settings
(tuner/investigations/steering_chatter_check.py) must do so before the
first import of sim.rollout.core, which imports this module.
"""

import numpy as np

import sim.speed_profile as sp

from settings import (
    DT, USE_PRECOMPUTED_SPEED_PROFILE, ENABLE_DYNAMIC_SPEED_CAP,
    DYNAMIC_CAP_A_LAT_MAX, DYNAMIC_CAP_SAFETY, SPEED_TARGET_DEFICIT_MAX,
)

# v_max/v_min for the live-planner branch's speed_profile.curvature_speed() call.
# Mirror fsds_simulator/control/fsae_control/fsae_control/mpc/mpc_controller.py's
# v_max/v_min ROS parameters (default V_MAX/V_MIN) — and the old SimPlanner
# defaults these replace — so offline-tuned weights see the same speed targets
# the live node will command.
PLANNER_V_MAX = 20.0
PLANNER_V_MIN = 1.5

# Max rate (m/s^2) at which the speed TARGET may rise. Mirrors
# mpc_controller.SPEED_TARGET_RISE_RATE — keep the two in sync.
# Decreases are never rate-limited; only the rise is damped, to suppress the
# planner's frame-to-frame curvature jitter without capping real acceleration.
SPEED_TARGET_RISE_RATE = 7.0

# Max speed error (m/s) the rise limiter is allowed to open up before it stops
# ramping and waits for the car. Promoted to MPCParams.speed_target_deficit_max
# on the live side (ROS param/YAML/launch arg/GUI tunable); imported here from
# settings.py's SPEED_TARGET_DEFICIT_MAX, which is the offline mirror of that
# same field, kept in sync by hand like every other entry in that file.
#
# SPEED_TARGET_RISE_RATE alone assumes the car can accelerate at that rate. From
# a standing start it cannot: the car does not break static friction for ~1 s,
# so the target ramps to ~7 m/s while the car is still stationary and banks a
# deficit it spends the next second chasing. The NMPC minimises one scalar cost
# over the horizon, so a speed error that large swamps the lateral term and the
# optimiser trades e_y away for speed it was never going to get — measured as a
# sideways excursion at launch that self-corrects once the car is rolling.
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
# normal lap: measured 36.8% of ticks pinned at exactly the limit, holding
# a_cmd to 4.45 against a plant that delivers ~12. Raising it to 5.0 drops
# the pinned fraction to 2.3%, nearly doubles peak a_cmd to 8.32, and
# improves every metric at once rather than trading any against another:
#
#   DEFICIT_MAX   score (3 runs)        lap steps   a_cmd max   |e_y| mean   steer sat
#   2.5           0.757/0.804/0.757     1081-1117   4.45        0.418        4.71%
#   5.0           0.693/0.692/0.693     1033-1034   8.32        0.402        3.77%
#
# Lower score is better. The launch behaviour the clamp exists to protect is
# unchanged (launch at step 9 either way, launch-phase |e_y| 0.27 m against
# a 3.5 m boundary), which is why the guard still does its job at 5.0. Run
# to run spread also collapses (0.001 vs 0.047), because the clamp is no
# longer arbitrating most of the lap.
#
# Values above ~5 buy nothing further (10.0 and 100.0 both plateau at
# a_cmd 8.87 and score no better), so this is the knee, not a ceiling to
# keep raising. Do not read it as "the clamp was wrong": it is a real guard
# and still needed, it was simply set tight enough to bind far outside the
# regime it was designed for.

# Max rate (gate-units/s) at which tracking_error_speed_gate()'s output may
# change per tick, in either direction. Mirrors
# mpc_controller.GATE_RATE_LIMIT — keep the two in sync. See that
# constant's own comment for the full rationale.
GATE_RATE_LIMIT = 2.0

# Max rate (m/s^2) at which curvature_speed()'s OWN output (the live,
# per-step centreline-derived target, NOT the precomputed-profile oracle
# lookup) may fall. Mirrors mpc_controller.V_CURV_FALL_RATE — keep the two
# in sync. See that constant's own comment for the full history: an initial
# 5.0 (speed_profile.A_BRAKE_PLAN, the PLANNING-time deceleration
# curvature_speed()'s braking-distance propagation assumes) capped genuine
# hard braking below what the car can do (measured live 2026-09-15: car
# entered the first corner at ~17 m/s, took 3+ s to reach the real ~2.5 m/s
# target, spun out before arriving). Now 7.0, matching mpc_core.MAX_BRAKE /
# vehicle_physics.max_accel_brake, the car's actual achievable braking
# deceleration rather than a conservative planning assumption.
V_CURV_FALL_RATE = 7.0



def compute_speed_target(
    planner_cl, cl_idx, car_pos_np, path_v_profile, idx, v_curv_prev,
):
    """
    The raw speed target: the oracle profile plus a live curvature cap when
    a planner centreline is available, otherwise the oracle profile alone.

    `planner_cl`/`cl_idx` come from compute_reference()'s return (None when
    no planner centreline was ready this tick, which folds into the "no
    live centreline" branch below exactly like the pre-split function's
    fallback/no-planner cases did — both only ever read path_v_profile[idx]).

    Returns (v_target, v_curv_prev).
    """
    if planner_cl is None:
        return float(path_v_profile[idx]), v_curv_prev

    cl = planner_cl
    if USE_PRECOMPUTED_SPEED_PROFILE:
        # Track is already fully mapped (settings.py's
        # USE_PRECOMPUTED_SPEED_PROFILE) -- use the oracle speed
        # profile computed once from the WHOLE path (path_v_profile,
        # non-causal, see speed_profile.compute_speed_profile()) at
        # the car's current position, instead of re-deriving from
        # only the live-built sub-path. Bypasses the perception-FOV
        # lookahead shortfall entirely (the live centreline is
        # typically shorter than curvature_speed()'s own scan
        # horizon), since it needs no live cone visibility at all
        # for the speed target. idx is one step stale here
        # (updated later in the loop, same as the path_v_profile[idx]
        # fallback below) -- accepted, not new.
        v_target = float(path_v_profile[idx])

        # The oracle lookup above has no notion of the car's
        # actual current speed relative to how much runway is
        # left to brake for the upcoming corner — see
        # settings.ENABLE_DYNAMIC_SPEED_CAP's docstring. Layer a
        # live curvature-lookahead cap under it (min, never above
        # the oracle target) so a corner reached faster than
        # planned still gets braked for in time. Mirrors
        # mpc_controller.py's identical logic.
        if ENABLE_DYNAMIC_SPEED_CAP:
            v_cap = sp.curvature_speed(
                cl[cl_idx:], v_max=PLANNER_V_MAX, v_min=PLANNER_V_MIN,
                a_lat_max=DYNAMIC_CAP_A_LAT_MAX, safety=DYNAMIC_CAP_SAFETY,
            )
            v_target = min(v_target, v_cap)
    else:
        # No pre-computed profile exists for a live-built centreline
        # (see SimPlanner) -- derive the target speed on-demand each
        # step from the sub-path ahead of the car, exactly as the
        # live ROS node does via control_utils.curvature_speed().
        v_target = sp.curvature_speed(
            cl[cl_idx:], v_max=PLANNER_V_MAX, v_min=PLANNER_V_MIN
        )
        # curvature_speed() has no memory of its own last output
        # and the live centreline is rebuilt every step, so a
        # single noisy sample can swing v_target down far faster
        # than any real corner's own braking-distance curve would
        # ask for -- see V_CURV_FALL_RATE's own comment. Mirrors
        # mpc_controller.py's identical fix.
        if v_curv_prev is not None:
            max_fall = V_CURV_FALL_RATE * DT
            v_target = max(v_target, v_curv_prev - max_fall)
        v_curv_prev = v_target

    return v_target, v_curv_prev


def gate_and_rate_limit_speed_target(v_target, e_y, e_psi, v_actual, gate_prev, v_des_prev):
    """
    Tracking-error speed gate plus the target's rise-rate and deficit limits.
    Returns (v_target, gate_prev, v_des_prev).

    Mirrors mpc_controller.py's Phase 3 exactly (see
    control_utils.tracking_error_speed_gate for the rationale and the live
    measurements behind the thresholds). curvature_speed() reads only path
    SHAPE, so without the gate the target stays high, and can even command
    acceleration, while the car is badly off-line with steering already
    saturated, which is unrecoverable.

    The gate's own rate of change is limited by GATE_RATE_LIMIT, see
    mpc_controller.py's identical comment for the full rationale (disabling
    the gate outright trades away its whole purpose; smoothing its rate of
    change removes the sharp-cliff side effect that motivated disabling it).
    """
    raw_gate = sp.tracking_error_speed_gate(e_y, e_psi)
    if gate_prev is not None:
        max_step = GATE_RATE_LIMIT * DT
        raw_gate = float(np.clip(raw_gate, gate_prev - max_step, gate_prev + max_step))
    gate_prev = raw_gate
    gate = raw_gate
    v_target = max(PLANNER_V_MIN, v_target * gate)
    # Seed the ramp from the car's ACTUAL speed on the first tick, not
    # from an unlimited jump straight to v_target -- see mpc_controller.py's
    # identical fix and CLAUDE.md's standstill steering-saturation note.
    # Without this, a standing start (vx0=0.0) asks the NMPC to track the
    # full-speed target from tick 0 via its e_v cost term, which is the
    # actual root cause of the "steers hard at startup" symptom -- not a
    # plant/tyre-force bug.
    if v_des_prev is None:
        v_des_prev = v_actual
    v_target = min(v_target, v_des_prev + SPEED_TARGET_RISE_RATE * DT)
    # Stop ramping once the target has run this far ahead of the car; see
    # SPEED_TARGET_DEFICIT_MAX. Never DROPS the target (max against
    # v_des_prev), so a car that is merely slow does not get the target
    # dragged down to meet it, and a genuine brake request still passes
    # through the min() above untouched.
    if v_target - v_actual > SPEED_TARGET_DEFICIT_MAX:
        v_target = min(v_target, max(v_des_prev, v_actual + SPEED_TARGET_DEFICIT_MAX))
    v_des_prev = v_target
    return v_target, gate_prev, v_des_prev


def compute_time_bonus(reached_end, sim_time, progress, optimal_time, dynamic_max_steps):
    """
    Time bonus for a finished run, 0.0 otherwise (identical formula for both
    rollout callers).

    Anchored to the path's PHYSICAL optimum where available. Dividing by a
    placeholder step budget (e.g. some multiple of arc_length /
    assumed_speed) has no physical meaning, can be several times the actual
    optimum, and varies by path, which would make TIME_BONUS_WEIGHT (0.25,
    the second-largest score term) a reward against an arbitrary constant
    and make the bonus non-comparable BETWEEN paths.

    optimal_lap_time() is a quasi-steady-state bound (corner limit ->
    forward accel pass -> backward brake pass -> integrate ds/v), so
    time_bonus is "how close to physically-fastest", in [0, 1] and directly
    comparable across paths. Because the bound ignores transient dynamics it
    is not quite attainable, so a real run scores below 1.0. Ratio form, NOT
    (1 - sim/optimal): sim_time is always >= optimal_time (it's a lower
    bound), so that subtraction would clip to 0 on every run and carry no
    information. optimal/sim is 1.0 at the physical limit and decays toward
    0 as the run gets slower, e.g. twice the optimal time scores 0.5.

    optimal_time covers the WHOLE path, but the rollout stops as soon as the
    car is within 3 m of the finish and past ~90% of the points, so it is
    only timed over `progress` of the distance. Comparing a partial-path run
    against a full-path reference makes the car look faster than physically
    possible and can saturate the bonus at exactly 1.000 for many runs,
    destroying all discrimination in the primary objective. The reference is
    scaled to the distance actually covered.
    """
    if not reached_end:
        return 0.0
    if optimal_time is not None and optimal_time > 0.0 and sim_time > 0.0:
        ref_time = optimal_time * max(progress, 1e-6)
        return float(np.clip(ref_time / sim_time, 0.0, 1.0))
    expected_time = dynamic_max_steps * DT
    return max(0.0, 1.0 - (sim_time / expected_time))
