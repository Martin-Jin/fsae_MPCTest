"""
controller/model_utils.py — Adaptive MPC Gain Helpers

PURPOSE
-------
Provides two runtime gain-shaping functions that modify the MPC's cost weight
matrices (R and R_rate) on a per-step basis based on the vehicle's current
speed and estimated path curvature. This allows the MPC to behave differently
in corners versus straights, and at low speed versus high speed, without
requiring a separate set of tuned weights for each regime.

These functions implement a form of gain scheduling: the base weights (Q, R,
R_rate) are tuned offline by tuner/offline_tuner.py as if for a single operating
point, then these helpers scale them at runtime to compensate for the known
nonlinear dependence of required control authority on speed and curvature.

HOW THE SCALING WORKS
---------------------
adaptive_R_rate (curvature-based):
    In a tight corner the vehicle must change steering direction quickly to
    track the path, so penalising steering rate-of-change (R_rate[0,0]) too
    heavily would prevent the needed responsiveness. The scale factor
    1/(1 + 3*κ) is a saturating function: at zero curvature (straight) it
    equals 1.0 (no softening); at high curvature it approaches 0.33 (floored
    at 0.55 — see below). This lets the controller be more aggressive with
    steering changes when the path demands it, without softening the rate
    cost so far that it stops damping oscillation.

adaptive_R_scaling (speed-based):
    At higher speeds, a unit of steering angle produces a much larger lateral
    force and path deviation than at low speed (because lateral acceleration
    ≈ vx² * κ). The steering cost is therefore increased with speed to
    discourage large steering commands that would be destabilising at high
    speed. The Hill-function form A*vx/(vx_half + vx) is a saturating
    ramp: it rises steeply at low speeds and asymptotes to A=1.5 (so the
    maximum steer scale factor is 1 + 1.5 = 2.5). Acceleration cost is
    scaled more gently (linear, 0.05*vx) since longitudinal dynamics are
    less speed-sensitive in the tracking error framework.

USED BY
-------
  sim/rollout_core.py — called once per step inside run_core_rollout(), the
                    single shared rollout loop used by both gui/simulation.py
                    and tuner/offline_tuner.py.

DOES NOT USE
------------
  model/vehicle_physics.py, model.py, controller/optimiser.py, sim/speed_profile.py,
  sim/sim_track.py, tuner/performance_stats.py
"""

import numpy as np


def curvature_estimate(state):
    """
    Estimate the current path curvature from the vehicle's yaw rate and speed.

    Physics: For a vehicle following a circular arc of radius R at speed vx,
    the yaw rate r = vx / R, therefore curvature κ = 1/R = r / vx.
    This is an instantaneous estimate based on the plant's measured state —
    it captures the curvature the vehicle is currently experiencing rather
    than the path geometry ahead, which makes it a causal (non-predictive)
    curvature signal suitable for real-time gain adjustment.

    Parameters
    ----------
    state : array-like, length ≥ 6
        Plant state vector. Reads:
          state[3] — longitudinal speed vx (m/s)
          state[5] — yaw rate r (rad/s)
        Compatible with both the 8-state MPC vector and the 24-state plant vector
        since both share indices 3 and 5.

    Returns
    -------
    kappa : float
        Estimated path curvature magnitude (1/m = rad/m). Always non-negative.
        Minimum effective vx of 0.5 m/s prevents division by near-zero speed.

    Called by: tuner/offline_tuner.py (run_headless_rollout),
               gui/simulation.py (simulate_closed_loop)
    """
    vx = max(state[3], 0.5)   # Guard: avoid division by near-zero speed
    r  = state[5]              # Yaw rate (rad/s)
    return abs(r / vx)         # |κ| = |r| / vx  (always positive)


def adaptive_R_rate(kappa, R_rate_base, disable_in_corners=False):
    """
    Scale the steering rate-of-change cost R_rate[0,0] based on path curvature.

    In straight-line driving (κ ≈ 0), the full R_rate steering penalty applies,
    discouraging unnecessary steering jitter. In tight corners (large κ), the
    penalty is softened so the controller can make the larger steering rate
    changes needed to track the curve.

    Scaling formula:
        scale = max(0.55, 1 / (1 + 3 * κ))

    At κ = 0.0 (straight):         scale = 1.00 → no change to R_rate
    At κ = 0.1 (R=10 m corner):    scale = 0.77 → moderate softening
    At κ = 0.2 (R=5 m tight turn): scale = 0.63 → more softening
    At κ → ∞:                       scale → 0.55 → floor (55% of base)

    The floor is kept fairly high (0.55, not lower) because over-relaxing
    this cost is exactly what lets steering sign-reversal chatter grow in
    corners: R_rate[0,0] is the one cost term that directly discourages
    rapid steer-sign-flipping, so cutting it hard at high curvature works
    against damping instead of with it. 0.55 keeps some softening available
    in genuinely tight corners (where the vehicle does need to change
    steering direction quickly) while still ensuring the rate cost never
    fully vanishes, which would allow arbitrarily rapid steering
    oscillations.

    Only R_rate[0,0] (steering rate penalty) is modified. R_rate[1,1]
    (acceleration rate penalty) is unchanged: longitudinal jerk is less
    affected by curvature, and aggressive acceleration changes in corners
    destabilise traction regardless of curvature.

    Parameters
    ----------
    kappa : float
        Current path curvature estimate from curvature_estimate() (1/m).
    R_rate_base : np.ndarray, shape (2, 2)
        Base rate-of-change cost matrix, typically the tuned R_rate from
        tuner/offline_tuner.py or gui/simulation.py. Not modified in-place.
    disable_in_corners : bool, optional
        TEMPORARY/EXPERIMENTAL, NOT VALIDATED. False (default) preserves the
        original continuous softening above. True uses a kappa_straight=0.1
        cutoff (raised 2026-08-09 from 0.02 so only sharp corners trigger
        this) to mean "not cornering": below it, softening still applies as
        normal (it barely does anything that close to straight anyway); at
        or above it ("cornering"), softening is switched off entirely and
        R_rate[0,0] gets the full, unscaled baseline cost -- deliberately
        undoing the softening this function exists to provide, so only
        enable this to test the effect, not as a validated tuning choice.

    Returns
    -------
    R : np.ndarray, shape (2, 2)
        Modified R_rate with R[0,0] scaled by the curvature factor.
        A copy of R_rate_base — the original is not mutated.

    Called by: tuner/offline_tuner.py (run_headless_rollout),
               gui/simulation.py (simulate_closed_loop)
    """
    R = np.array(R_rate_base, copy=True)          # Never mutate the caller's matrix
    kappa_straight = 0.1
    if disable_in_corners and abs(kappa) > kappa_straight:
        scale = 1.0                                     # Cornering -> no softening, full baseline cost
    else:
        scale = max(0.55, 1.0 / (1.0 + 3.0 * kappa))    # Saturating softening: 1.0 at κ=0, floor at 0.55
    R[0, 0] *= scale                                    # Apply only to steering rate cost
    return R


def steer_rate_anti_hunt(kappa, e_y, R_rate_base, enabled=False):
    """
    TEMPORARY/EXPERIMENTAL (fsds sim only, off by default): heavily penalise
    steering-rate-of-change when the car is already centred on the path AND
    not currently in a corner, to suppress small-error steering hunt/chatter.

    This stacks on top of (does not replace) adaptive_R_rate: adaptive_R_rate
    only ever softens R_rate[0,0] below the tuned baseline for corners; this
    function multiplies in an additional boost ABOVE 1.0 when both of the
    following hold:
      - kappa is near zero (car isn't currently curving -- proxy for "no
        corner right now"; this is the SAME causal, current-state curvature
        signal adaptive_R_rate uses, not a path lookahead, so it cannot
        anticipate a corner the car hasn't reached yet)
      - |e_y| is small (car is already close to the centreline)

    NOT VALIDATED. Added as a quick experiment, gated behind
    STEER_RATE_ANTI_HUNT_ENABLED (default False in settings.py) so it can be
    tried and ripped out without affecting any existing tuned behaviour.
    Mirrors adaptive_Q_scaling's opt-in pattern: disabled callers get
    R_rate_base back completely unmodified.

    Parameters
    ----------
    kappa : float
        Current path curvature estimate from curvature_estimate() (1/m).
    e_y : float
        Current lateral deviation from the path centreline (m). Sign does
        not matter; only magnitude is used.
    R_rate_base : np.ndarray, shape (2, 2)
        Rate-of-change cost matrix to boost -- pass the ALREADY
        curvature-softened output of adaptive_R_rate so the two compose
        rather than one undoing the other.
    enabled : bool, optional
        Master off-switch. False (default) returns R_rate_base completely
        unmodified -- not even copied.

    Returns
    -------
    R : np.ndarray
        R_rate_base unchanged if enabled=False. Otherwise a copy with
        R[0,0] boosted when kappa and |e_y| are both small.

    Called by: sim/rollout_core.py (run_core_rollout), opt-in only
    """
    if not enabled:
        return R_rate_base

    kappa_straight, ey_low, boost = 0.02, 0.05, 3.0
    if kappa <= kappa_straight and abs(e_y) <= ey_low:
        scale = boost
    else:
        scale = 1.0

    R = np.array(R_rate_base, copy=True)
    R[0, 0] *= scale
    return R


def adaptive_Q_scaling(e_y, Q_base, enabled=False):
    """
    Soften the lateral-error cost Q[0,0] when the car is already close to the
    path centreline, to reduce small-error hunting/chatter.

    WHY THIS EXISTS
    ----------------
    Motivated by a live-only symptom: steering sign-reversal rate rising as
    |e_y| gets smaller, i.e. the car darts across the centreline rather than
    settling onto it. A quadratic cost with no dead zone has the same
    proportional "pull" toward zero error regardless of how small the error
    already is, which is a plausible contributor to a self-reinforcing
    correct-overcorrect cycle right at the point the controller should be
    settling, not correcting.

    This trend has NOT been reproduced on the offline recorded-map rollout —
    there, reversal rate instead increases with |e_y|, the opposite trend.
    This may be a live-only symptom (sensor/state noise, delay-compensation
    dynamics, or the plant behaving differently from the model near zero
    slip) rather than something the offline plant reproduces. Implemented
    here anyway, DISABLED BY DEFAULT, so it exists to test against a live
    log without risking any offline-tuned behaviour changing silently.

    SHAPE
    -----
    Mirrors adaptive_R_rate's saturating-floor style: linear ramp between
    ey_lo and ey_hi, 1.0 (no change) at and above ey_hi, floor below ey_lo.
        scale = floor                              |e_y| <= ey_lo
        scale = floor + (1-floor)*(|e_y|-ey_lo)/(ey_hi-ey_lo)   ey_lo < |e_y| < ey_hi
        scale = 1.0                                 |e_y| >= ey_hi
    Defaults (ey_lo=0.05, ey_hi=0.3, floor=0.5) are a starting point chosen
    to bracket the exact regime the live log's reversal-rate spike sits in
    (compare tracking_error_speed_gate's ey_lo=0.5/ey_hi=2.0, which gates a
    completely different, much larger-error regime -- speed reduction during
    recovery from being badly off-line, not steering softening near-centre).
    NOT validated against live data yet; must not be enabled without
    re-running VALIDATION_SUITE/recorded-map for new DNFs first, same as any
    other weight change.

    Parameters
    ----------
    e_y : float
        Current lateral deviation from the path centreline (m). Sign does
        not matter; only magnitude is used.
    Q_base : np.ndarray, shape (8,) or (8,8)
        Base state cost, typically the tuned Q from tuner/offline_tuner.py
        or gui/simulation.py. Not modified in-place.
    enabled : bool, optional
        Master off-switch. False (default) returns Q_base completely
        unmodified -- not even copied -- so callers that don't opt in pay
        zero cost and get byte-identical behaviour to before this function
        existed.

    Returns
    -------
    Q : np.ndarray
        Q_base unchanged if enabled=False. Otherwise a copy with Q[0,0]
        (or Q[0] if Q_base is a 1-D diagonal vector) scaled down when
        |e_y| < ey_hi.

    Called by: sim/rollout_core.py (run_core_rollout), opt-in only
    """
    if not enabled:
        return Q_base

    ey_lo, ey_hi, floor = 0.05, 0.3, 0.5
    ey_abs = abs(e_y)
    if ey_abs >= ey_hi:
        scale = 1.0
    elif ey_abs <= ey_lo:
        scale = floor
    else:
        scale = floor + (1.0 - floor) * (ey_abs - ey_lo) / (ey_hi - ey_lo)

    Q = np.array(Q_base, copy=True)
    if Q.ndim == 1:
        Q[0] *= scale
    else:
        Q[0, 0] *= scale
    return Q


def adaptive_R_scaling(vx, R_base):
    """
    Scale the steering and acceleration input costs R[0,0] and R[1,1] based
    on longitudinal speed.

    At higher speeds, the same steering command produces much larger lateral
    force and path deviation. The steering cost is therefore increased with
    speed to make the controller more conservative with steering commands,
    improving stability at speed. Acceleration cost is scaled more gently.

    Steering scale formula (Hill / Michaelis-Menten saturation):
        steer_scale = 1 + (A * vx) / (vx_half + vx)

    Where:
        A = 1.5        → asymptotic maximum additional scale (steer_scale → 2.5 at vx → ∞)
        vx_half = 6.0  → speed at which half the maximum additional scale is reached

    At vx = 0.5 m/s:  steer_scale ≈ 1.11  (barely changed from base)
    At vx = 6.0 m/s:  steer_scale = 1.75  (half-maximum: 75% increase)
    At vx = 15.0 m/s: steer_scale ≈ 2.07  (71% of the way to the 2.5 asymptote)

    The Hill function was chosen over a linear ramp because:
      1. It saturates at high speeds, preventing the steering cost from
         growing without bound and eventually locking out all steering.
      2. The half-saturation point (vx_half=6 m/s) places rapid scaling
         in the regime where stability most benefits from conservative steering
         (the transition from kinematic to dynamic lateral behaviour, which
         the linear model captures around 1-2.5 m/s).

    Acceleration scale (linear, gentler):
        accel_scale = 1 + 0.05 * vx

    At vx=15 m/s: accel_scale = 1.75 (75% increase). Longitudinal control
    is inherently less speed-sensitive in the tracking error framework, so
    a lighter scale suffices.

    Parameters
    ----------
    vx : float
        Current longitudinal vehicle speed (m/s). Floored at 0.5 m/s
        internally to avoid undefined behaviour at exact zero.
    R_base : np.ndarray, shape (2, 2)
        Base input cost matrix, typically the tuned R from tuner/offline_tuner.py
        or gui/simulation.py. Not modified in-place.

    Returns
    -------
    R_scaled : np.ndarray, shape (2, 2)
        Modified R with R[0,0] scaled by steer_scale and R[1,1] scaled by
        accel_scale. A copy of R_base — the original is not mutated.

    Called by: tuner/offline_tuner.py (run_headless_rollout),
               gui/simulation.py (simulate_closed_loop)
    """
    vx = max(vx, 0.5)              # Guard: avoid undefined behaviour at zero speed

    A        = 1.5                 # Asymptotic maximum additional steer scale factor
    vx_half  = 6.0                 # Speed at which half of A is reached (m/s)

    # Hill-function saturating ramp: rises quickly below vx_half, flattens above it
    steer_scale = 1.0 + (A * vx) / (vx_half + vx)

    # Linear scale for acceleration: gentler than steering since longitudinal
    # dynamics are less sensitive to speed in the Frenet-frame error model
    accel_scale = 1.0 + 0.05 * vx

    R_scaled = np.array(R_base, copy=True)   # Never mutate the caller's matrix
    R_scaled[0, 0] *= steer_scale            # Scale steering input cost
    R_scaled[1, 1] *= accel_scale            # Scale acceleration input cost
    return R_scaled