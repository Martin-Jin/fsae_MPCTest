"""
fsae_control/nmpc/weight_schedule.py — R_rate zone scheduling

Curvature-zone and per-stage scaling of the steering-rate weight.
"""

import numpy as np

from fsae_control.lmpc import _corner_factor


def _rrate_zone_scale(kappa_now, kappa_ahead, k, boost_straight, ease_approach,
                      floor_corner):
    """
    Continuous three-zone multiplier on the steering-rate cost, driven by
    CURRENT curvature and the peak curvature the HORIZON predicts ahead:

        straight  (nothing now, nothing ahead)  -> boost_straight  (>= 1)
        approach  (nothing now, corner ahead)   -> ease_approach
        corner    (turning now)                 -> floor_corner    (<= 1)

    Both inputs go through the same saturating `_corner_factor` curve, so
    this is a smooth surface with no thresholds or hysteresis -- it degrades
    gracefully on a continuously-winding road (where `now` and `ahead` are
    both high, giving the corner value throughout) and on a lone kink
    (where `ahead` leads `now` by a tick or two, giving a brief approach
    ease).

    Blend order matters: the approach ease is applied FIRST against the
    straight boost, then the corner floor takes over as `now` rises. That
    ordering means a corner entered from a straight passes
    boost -> ease -> floor in that sequence, which is the intended
    "release the brake before you need to turn" behaviour.

    CAUTION on `k`: the ease/floor endpoints are only REACHED as
    `_corner_factor` approaches 1, so this schedule is only as strong as `k`
    lets it saturate over the TRACK's curvature range. At the LTV-QP's
    inherited k=8.0 a track whose tightest corner is |kappa|~0.2 tops
    `_corner_factor` out near 0.63, which silently degrades this from a
    three-zone schedule into a mild global rate boost -- the multiplier never
    leaves the boost band. Check a run's `m_Rrate_zone` column against
    `floor_corner` before concluding the endpoints did anything; if it never
    approaches the floor, raise `k` rather than lowering the endpoints
    (k ~= target/((1 - target) * kappa_max)).
    CAUTION on `kappa_ahead`: this is the peak |kappa| the horizon predicts,
    which leads current curvature by roughly the horizon length. Against a
    static raceline that is a clean signal. Against a LIVE planner path it
    inherits the open centreline curvature-spike defect, and unlike a speed
    target there is no rate limiter downstream to absorb a spurious spike --
    re-validate before trusting this with use_planner=True.
    """
    now = _corner_factor(abs(kappa_now), k)
    ahead = _corner_factor(abs(kappa_ahead), k)
    # Lead-only component: how much more corner is COMING than is here now.
    lead = max(0.0, ahead - now)
    s = boost_straight + (ease_approach - boost_straight) * lead
    return s + (floor_corner - s) * now


def _rrate_stage_ramp(N, near):
    """
    Per-stage multiplier on the steering-rate cost: `near` at horizon stage 0,
    rising linearly to 1.0 at the last stage. Returns shape (N,).

    WHY: the plain rate cost is uniform across the horizon
    (`np.tile(self.r_rate, N)`), so it charges the same price for a steering
    change whether that change is the FIRST move into a corner or the tenth
    tick of an oscillation. One weight cannot be both stiff enough to kill
    tick-to-tick hunting and compliant enough for a gentle corner's small
    early input, which is why a high flat weight makes the solver defer
    turn-in until it must catch up at the actuator slew limit.

    Ramping by STAGE is keyed on horizon POSITION, not measured state -- a
    measured-state schedule (curvature/error) was live-tested and failed
    because ~27% of jerk events show no curvature or heading-error signal at
    all one second beforehand, while the horizon already predicts the corner.

    CAUTION: offline-rejected as a fix for that jerk -- it moved
    slew-limited ticks the WRONG way (8.4% -> 12-15%) because a cheaper
    near-stage rate simply spends more of the slew budget every tick. It IS
    the only change found so far that clears the offline nmpc_offline_check
    DNF, which is why it is kept. See
    fsae_MPCTest/docs/steering_turn_in_upgrade_options.md (Option 1).

    `near` = 1.0 is an exact no-op, so the flag-off path is byte-identical.
    """
    if N <= 1:
        return np.ones(max(N, 1))
    return np.linspace(float(near), 1.0, N)
