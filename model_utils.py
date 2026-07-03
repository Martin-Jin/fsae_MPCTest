"""
model_utils.py — Adaptive MPC Gain Helpers

PURPOSE
-------
Provides two runtime gain-shaping functions that modify the MPC's cost weight
matrices (R and R_rate) on a per-step basis based on the vehicle's current
speed and estimated path curvature. This allows the MPC to behave differently
in corners versus straights, and at low speed versus high speed, without
requiring a separate set of tuned weights for each regime.

These functions implement a form of gain scheduling: the base weights (Q, R,
R_rate) are tuned offline by offline_tuner.py as if for a single operating
point, then these helpers scale them at runtime to compensate for the known
nonlinear dependence of required control authority on speed and curvature.

HOW THE SCALING WORKS
---------------------
adaptive_R_rate (curvature-based):
    In a tight corner the vehicle must change steering direction quickly to
    track the path, so penalising steering rate-of-change (R_rate[0,0]) too
    heavily would prevent the needed responsiveness. The scale factor
    1/(1 + 3*κ) is a saturating function: at zero curvature (straight) it
    equals 1.0 (no softening); at high curvature it approaches 0.33 (floors
    at 0.35). This lets the controller be more aggressive with steering
    changes when the path demands it.

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
  offline_tuner.py — called inside run_headless_rollout() before each MPC solve
  simulation.py    — called inside simulate_closed_loop() before each MPC solve

Note: curvature_estimate() is also defined directly in offline_tuner.py and
simulation.py for historical reasons; model_utils.py is the single canonical
source and those copies should be removed when refactoring is complete.

DOES NOT USE
------------
  vehicle_physics.py, model.py, optimiser.py, speed_profile.py,
  sim_track.py, performance_stats.py
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

    Called by: offline_tuner.py (run_headless_rollout),
               simulation.py (simulate_closed_loop)
    """
    vx = max(state[3], 0.5)   # Guard: avoid division by near-zero speed
    r  = state[5]              # Yaw rate (rad/s)
    return abs(r / vx)         # |κ| = |r| / vx  (always positive)


def adaptive_R_rate(kappa, R_rate_base):
    """
    Scale the steering rate-of-change cost R_rate[0,0] based on path curvature.

    In straight-line driving (κ ≈ 0), the full R_rate steering penalty applies,
    discouraging unnecessary steering jitter. In tight corners (large κ), the
    penalty is softened so the controller can make the larger steering rate
    changes needed to track the curve.

    Scaling formula:
        scale = max(0.35, 1 / (1 + 3 * κ))

    At κ = 0.0 (straight):         scale = 1.00 → no change to R_rate
    At κ = 0.1 (R=10 m corner):    scale = 0.77 → moderate softening
    At κ = 0.2 (R=5 m tight turn): scale = 0.63 → more softening
    At κ → ∞:                       scale → 0.35 → floor (35% of base)

    The floor at 0.35 ensures the rate cost never fully vanishes, which would
    allow arbitrarily rapid steering oscillations.

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
        offline_tuner.py or simulation.py. Not modified in-place.

    Returns
    -------
    R : np.ndarray, shape (2, 2)
        Modified R_rate with R[0,0] scaled by the curvature factor.
        A copy of R_rate_base — the original is not mutated.

    Called by: offline_tuner.py (run_headless_rollout),
               simulation.py (simulate_closed_loop)
    """
    R     = np.array(R_rate_base, copy=True)          # Never mutate the caller's matrix
    scale = max(0.35, 1.0 / (1.0 + 3.0 * kappa))     # Saturating softening: 1.0 at κ=0, floor at 0.35
    R[0, 0] *= scale                                    # Apply only to steering rate cost
    return R


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
    At vx = 15.0 m/s: steer_scale ≈ 2.33  (near asymptote: 133% increase)

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
        Base input cost matrix, typically the tuned R from offline_tuner.py
        or simulation.py. Not modified in-place.

    Returns
    -------
    R_scaled : np.ndarray, shape (2, 2)
        Modified R with R[0,0] scaled by steer_scale and R[1,1] scaled by
        accel_scale. A copy of R_base — the original is not mutated.

    Called by: offline_tuner.py (run_headless_rollout),
               simulation.py (simulate_closed_loop)
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