"""
controller/model_utils.py — Adaptive MPC Gain Helpers

PURPOSE
-------
Provides runtime gain-shaping functions that modify the MPC's cost weight
matrices (Q, R and R_rate) on a per-step basis based on the vehicle's current
speed, estimated path curvature, and (for the lookahead functions) a forward
scan of the path ahead. This allows the MPC to behave differently in corners
versus straights, entering a corner versus already turning through it, and
at low speed versus high speed, without requiring a separate set of tuned
weights for each regime.

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
    equals 1.0 (no softening); at high curvature it floors at a "during a
    corner" value driven by the car's CURRENT-position curvature. A second,
    shallower floor driven by kappa_max_abs (the lookahead scan's peak
    curvature, see below) softens the cost slightly BEFORE the car reaches
    the corner too; the two floors combine via min() (whichever is more
    aggressive wins).

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

adaptive_Q_scaling (current lateral error):
    Softens Q[0,0] (lateral-error cost) when the car is already close to the
    centreline, to reduce small-error hunting/chatter. Reactive only — it
    looks at the car's CURRENT |e_y|, not the path ahead.

steer_rate_anti_hunt (current curvature + lateral + heading error):
    Stacks on top of adaptive_R_rate: boosts R_rate[0,0] ABOVE the tuned
    baseline (rather than softening it) when the car is simultaneously
    straight, centred, and well-aligned, to suppress residual steering
    hunt/chatter in that specific regime.

adaptive_Q_lookahead (forward-looking, corner anticipation and U-turns):
    Unlike the reactive functions above, this scans a speed-scaled window of
    path AHEAD of the car (lookahead_curvature_profile) for the sharpest
    curvature and the total accumulated heading change coming up, and uses
    both to anticipate corners before the car reaches them: boosting
    lateral/heading-error cost on approach, relaxing yaw-rate cost on
    approach, continuing a heading-error boost for a short distance after
    the corner (exit), softening cost on genuinely clear straights, and
    adding an extra boost for long, gradual U-turns that peak curvature
    alone would under-boost. All of the boost curves are DEMAND-normalised
    by default (corner curvature relative to how much curvature the car can
    actually hold at its current speed, via _corner_demand), rather than
    driven by raw curvature — see adaptive_Q_lookahead's own docstring for
    why raw curvature made the configured boost ceilings unreachable on real
    corners.

steer_effort_straight_boost (forward-looking, straight-line steering cost):
    The R[0,0] (steering effort, not its rate of change) counterpart of
    adaptive_Q_lookahead's straight-line softening: boosts R[0,0] on a clear
    straight, fading sharply back to baseline as a corner enters the
    lookahead window.

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


def adaptive_R_rate(kappa, R_rate_base, enable_in_corners=True, kappa_max_abs=0.0,
                     during_floor=0.625, entering_floor=0.85, k_entering=4.0):
    """
    Scale the steering rate-of-change cost R_rate[0,0] based on path curvature.

    In straight-line driving (κ ≈ 0), the full R_rate steering penalty applies,
    discouraging unnecessary steering jitter. In tight corners (large κ), the
    penalty is softened so the controller can make the larger steering rate
    changes needed to track the curve.

    Two floors compose via min() (the more aggressive reduction wins):
      - "during" a corner, driven by the CURRENT-position kappa:
            during_scale = max(0.625, 1 / (1 + 3 * κ))
        At κ = 0.0 (straight):         scale = 1.00 → no change to R_rate
        At κ = 0.1 (R=10 m corner):    scale = 0.77 → moderate softening
        At κ → ∞:                       scale → 0.625 → floor
      - "entering" a corner, driven by kappa_max_abs (a lookahead curvature
        signal -- see adaptive_Q_lookahead -- rather than the car's current
        position), shallower than the during-floor since the car hasn't
        reached the corner yet:
            entering_scale = max(0.85, 1 / (1 + 4 * kappa_max_abs))
    kappa_max_abs=0.0 (default) makes the entering floor a no-op, so callers
    that don't pass it get the original current-curvature-only behaviour.

    The during-floor is kept fairly high (0.625, not lower) because
    over-relaxing this cost is exactly what lets steering sign-reversal
    chatter grow in corners: R_rate[0,0] is the one cost term that directly
    discourages rapid steer-sign-flipping, so cutting it hard at high
    curvature works against damping instead of with it. It keeps some
    softening available in genuinely tight corners (where the vehicle does
    need to change steering direction quickly) while still ensuring the
    rate cost never fully vanishes, which would allow arbitrarily rapid
    steering oscillations. Raised from 0.55 after a live run with the
    entering floor at 0.85 showed steering oscillating through zero several
    times per second mid-corner while e_y/e_psi stayed small -- classic
    under-damped steering-rate hunt, not a tracking-error problem, so the
    fix was less softening of the rate cost while actually turning, not
    more lateral/heading authority. The entering floor's gain (4.0, lower
    than the during-floor's implicit 3.0-equivalent sharpness) was likewise
    lowered from 8.0 after even mild upcoming curvature triggered
    meaningful reduction, reported as R_rate "swinging a bit too much
    prematurely before corners".

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
    enable_in_corners : bool, optional
        TEMPORARY/EXPERIMENTAL, NOT VALIDATED. (Renamed from
        disable_in_corners, whose True/False polarity was inverted from
        what the name suggested.) True (default) preserves the original
        continuous softening above, keeping R_rate reduction ACTIVE in
        corners with no threshold/discontinuity -- this is what you want if
        the goal is "reduce R_rate when turning". False uses a
        kappa_straight=0.03 cutoff to mean "not cornering": below it,
        softening still applies as normal (it barely does anything that
        close to straight anyway); at or above it ("cornering"), softening
        is switched off entirely and R_rate[0,0] gets the full, unscaled
        baseline cost -- deliberately undoing the softening this function
        exists to provide. Tried disabled on 2026-08-09 but caused severe
        lag specifically in corners (likely the discontinuous cost jump
        spiking QP solver iterations); only disable this to re-test that
        effect, not as a validated tuning choice.
    kappa_max_abs : float, optional
        Lookahead curvature (same signal adaptive_Q_lookahead uses); 0.0
        (default) makes the entering floor a no-op.
    during_floor, entering_floor, k_entering : float, optional
        The two floors and the entering ramp sharpness described above.
        Defaults match settings.py's ADAPTIVE_R_RATE_DURING_FLOOR /
        _ENTERING_FLOOR / _K_ENTERING (and the live side's
        MPCParams.adaptive_r_rate_during_floor / _entering_floor /
        _k_entering), so a caller that passes nothing gets the tuned
        behaviour unchanged. The during-floor's own ramp sharpness (3.0) and
        the kappa_straight=0.03 cutoff are deliberately NOT parameters --
        they are not tuning knobs on either side.

    Returns
    -------
    R : np.ndarray, shape (2, 2)
        Modified R_rate with R[0,0] scaled by the curvature factor.
        A copy of R_rate_base — the original is not mutated.

    Called by: tuner/offline_tuner.py (run_headless_rollout),
               gui/simulation.py (simulate_closed_loop)
    """
    R = np.array(R_rate_base, copy=True)          # Never mutate the caller's matrix
    kappa_straight = 0.03
    if not enable_in_corners and abs(kappa) > kappa_straight:
        scale = 1.0                                     # Cornering -> no softening, full baseline cost
    else:
        during_scale   = max(during_floor, 1.0 / (1.0 + 3.0 * abs(kappa)))
        entering_scale = max(entering_floor, 1.0 / (1.0 + k_entering * kappa_max_abs))
        scale = min(during_scale, entering_scale)       # more aggressive reduction wins
    R[0, 0] *= scale                                    # Apply only to steering rate cost
    return R


def steer_rate_anti_hunt(kappa, e_y, R_rate_base, enabled=False, e_psi=0.0):
    """
    TEMPORARY/EXPERIMENTAL (fsds sim only, off by default): heavily penalise
    steering-rate-of-change on top of adaptive_R_rate's existing curvature
    softening, strongest when the car is centred (|e_y| small), well-aligned
    (|e_psi| small), AND not currently curving (kappa small). "Corner ahead"
    is NOT a path lookahead here -- it reuses the same causal, current-
    curvature kappa as adaptive_R_rate, so it cannot anticipate a corner
    before the car is already turning into it.

    Continuous, not a hard AND-gated threshold: boost_kappa, boost_ey, and
    boost_epsi each saturate independently toward 1.0 as their input shrinks
    toward 0 (same saturating-curve style as adaptive_R_rate's own floor);
    their product is the applied scale, so the full boost_max only applies
    when ALL THREE are near their "straight, centred, and aligned" ideal,
    and it fades smoothly -- never snaps -- as any one of them grows.

    e_psi (radians -- same units _error_state's e_psi already uses
    internally) guards against a car that enters a straight MISALIGNED
    (large |e_psi|, small |e_y| -- e.g. just exited a corner still pointed
    the wrong way): without it, kappa/e_y alone can't distinguish "straight
    and correctly aligned" from "straight but needs to yaw back into line",
    making exactly the correction it needs artificially expensive.
    e_psi=0.0 (default) makes this term a no-op for callers that don't pass
    it. k_epsi=23.0 sets half-fade at ~2.5 deg of e_psi.

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
    e_psi : float, optional
        Current heading error (rad). Sign does not matter; only magnitude
        is used. 0.0 (default) makes the alignment guard a no-op.

    Returns
    -------
    R : np.ndarray
        R_rate_base unchanged if enabled=False. Otherwise a copy with
        R[0,0] boosted when kappa, |e_y| and |e_psi| are all small.

    Called by: sim/rollout_core.py (run_core_rollout), opt-in only
    """
    if not enabled:
        return R_rate_base

    k_kappa, k_ey, k_epsi = 60.0, 30.0, 23.0
    boost_max = 6.0
    boost_kappa = 1.0 / (1.0 + k_kappa * abs(kappa))
    boost_ey    = 1.0 / (1.0 + k_ey * abs(e_y))
    boost_epsi  = 1.0 / (1.0 + k_epsi * abs(e_psi))
    scale = 1.0 + (boost_max - 1.0) * boost_kappa * boost_ey * boost_epsi

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


def _alat_ceiling_at(v, flat=7.5, slope=0.47, intercept=2.46):
    """
    FSDS's sustained lateral-acceleration ceiling at speed v (m/s^2).
    Mirrors model/vehicle_physics.py's alat_ceiling_at() — keep in sync
    (CLAUDE.md's plant/model parity rule). flat/slope/intercept default to
    settings.py's ALAT_CEILING_FLAT / _SLOPE / _INTERCEPT (and the live
    side's MPCParams.alat_ceiling_flat / _slope / _intercept).
    """
    return max(flat, slope * abs(v) + intercept)


def _corner_demand(kappa_max_abs, car_speed,
                    alat_flat=7.5, alat_slope=0.47, alat_intercept=2.46):
    """
    "How much of the car's available cornering does the path ahead demand at
    the current speed?" -- kappa_max_abs / kappa_limit(v), where
    kappa_limit = a_lat_ceiling(v) / v^2 is the tightest curvature holdable
    before the ceiling binds.

    0 = straight, ~1 = the corner needs everything available at this speed,
    >1 = cannot be held at this speed (must slow). Scale-free and
    speed-aware, so one set of constants covers gradual sweepers, tight
    corners and U-turns instead of needing a per-corner-type threshold --
    see adaptive_Q_lookahead's docstring for why raw kappa was the wrong
    parameterisation. Returns 0.0 below a small speed so a stationary or
    crawling car does not report infinite demand.

    alat_flat/alat_slope/alat_intercept parameterise the a_lat ceiling law
    passed through to _alat_ceiling_at; defaults match settings.py's
    ALAT_CEILING_FLAT / _SLOPE / _INTERCEPT.
    """
    v = abs(car_speed)
    if v < 1.0 or kappa_max_abs <= 0.0:
        return 0.0
    kappa_limit = _alat_ceiling_at(
        v, flat=alat_flat, slope=alat_slope, intercept=alat_intercept
    ) / (v * v)
    if kappa_limit <= 1e-9:
        return 0.0
    return float(kappa_max_abs / kappa_limit)


def _demand_frac(demand, demand_half=0.5):
    """
    Map a 0..inf corner demand onto a 0..1 boost fraction via the same
    saturating shape used elsewhere, but with the half-response point set in
    DEMAND units rather than raw curvature -- so the configured maxima are
    actually reachable on real corners.
    """
    h = max(demand_half, 1e-6)
    return demand / (demand + h)


def curvature(path, idx):
    """
    Estimate signed path curvature (1/m) at waypoint idx via finite-difference.
    Used by adaptive_Q_lookahead's lookahead scan -- distinct from
    curvature_estimate() above, which reads the plant's current yaw
    rate/speed rather than scanning the path geometry.
    """
    if idx <= 0 or idx >= len(path) - 1:
        return 0.0
    s_prev = path[idx]     - path[idx - 1]
    s_next = path[idx + 1] - path[idx]
    yaw_p  = np.arctan2(s_prev[1], s_prev[0])
    yaw_n  = np.arctan2(s_next[1], s_next[0])
    dpsi   = np.arctan2(np.sin(yaw_n - yaw_p), np.cos(yaw_n - yaw_p))
    ds     = (np.linalg.norm(s_prev) + np.linalg.norm(s_next)) * 0.5
    return dpsi / ds if ds > 1e-6 else 0.0


def lookahead_curvature_profile(path, base_idx, lookahead_dist):
    """
    Scan forward from base_idx up to lookahead_dist (arc length, m) and
    return (kappa_max_abs, idx_at_peak, dist_at_peak, heading_change_abs):
    the largest-MAGNITUDE signed curvature found in the window, the waypoint
    index it occurred at, how far ahead (m) that point is, and the total
    accumulated |heading change| across the whole window (the U-turn
    discriminator -- see adaptive_Q_lookahead's docstring).

    Uses |kappa|, not signed kappa, for kappa_max_abs so an S-curve's second
    corner (opposite sign to the first) contributes exactly as much as
    either corner alone -- sign never cancels magnitude here.

    Returns (0.0, base_idx, 0.0, 0.0) if the window can't be scanned (path
    too short / already at the end).
    """
    accumulated        = 0.0
    kappa_max_abs       = 0.0
    idx_at_peak         = base_idx
    dist_at_peak        = 0.0
    heading_change_abs  = 0.0
    for i in range(base_idx, len(path) - 1):
        ds = float(np.linalg.norm(path[i + 1] - path[i]))
        accumulated += ds
        if accumulated > lookahead_dist:
            break
        k = abs(curvature(path, i + 1))
        heading_change_abs += k * ds
        if k > kappa_max_abs:
            kappa_max_abs = k
            idx_at_peak   = i + 1
            dist_at_peak  = accumulated
    return kappa_max_abs, idx_at_peak, dist_at_peak, heading_change_abs


def update_lookahead_peak(state, kappa, car_speed, dt, peak_hysteresis=0.01):
    """
    Track distance travelled since the last LOCAL peak in CURRENT-POSITION
    curvature, for lookahead_exit_boost's decay.

    Keyed on `kappa` (the near-instantaneous curvature at the car's own
    position, the same signal adaptive_R_rate/steer_rate_anti_hunt use),
    NOT kappa_max_abs (the full speed-scaled lookahead window, default
    10-17m ahead at speed) -- changed 2026-08-11, numeric-parity mirror of
    the live mpc_core._update_lookahead_peak() fix. See that function's
    docstring for the full rationale: kappa_max_abs detects a corner the
    moment it enters the far lookahead window, not when the car is actually
    in it, so the decay clock (lookahead_exit_boost's 5m default window)
    had already elapsed by the time the car reached the physical apex on
    essentially every real corner taken at speed -- the exit-heading boost
    was structurally inert, not merely mistimed. kappa peaks at the car's
    own physical apex by construction, so decay now starts from the moment
    that matters.

    state is a dict with keys 'last_peak_kappa_abs', 'dist_since_peak',
    'armed_for_next_peak', mutated in place and also returned -- callers
    keep this dict per-controller-instance the same way mpc_core.py's
    MPCController keeps it as instance attributes. Initialise with
    {'last_peak_kappa_abs': 0.0, 'dist_since_peak': float('inf'),
    'armed_for_next_peak': True} so lookahead_exit_boost is a no-op until
    the first corner peak is ever seen.

    This is a rising-edge-after-a-clear detector, not "any new global
    maximum": armed_for_next_peak only goes True again once |kappa| has
    dropped back to <= peak_hysteresis (i.e. the car itself is genuinely on
    a straight, not just that the far lookahead window is clear), and a
    peak only latches while armed. Comparing against the running maximum
    instead would silently fail to re-trigger on a second corner of equal
    or LESSER curvature than an earlier one -- an ordinary case (most
    tracks reuse corner radii), not just a rare S-curve edge case. This way
    each distinct corner gets its own fresh decay cycle regardless of how
    its curvature compares to any previous corner's.

    Returns the updated state dict.
    """
    kappa_abs = abs(kappa)
    if kappa_abs <= peak_hysteresis:
        state['armed_for_next_peak'] = True
    elif state['armed_for_next_peak']:
        state['last_peak_kappa_abs'] = kappa_abs
        state['dist_since_peak'] = 0.0
        state['armed_for_next_peak'] = False
    elif kappa_abs > state['last_peak_kappa_abs']:
        state['last_peak_kappa_abs'] = kappa_abs
    state['dist_since_peak'] += abs(car_speed) * dt
    return state


def lookahead_approach_boost(kappa_max_abs, car_speed=0.0, boost_max=2.0, k=8.0,
                              demand_normalised=True, demand_half=0.5,
                              alat_flat=7.5, alat_slope=0.47, alat_intercept=2.46):
    """
    Continuous (no threshold/discontinuity) multiplier on Q[0,0] (lateral
    error) that rises smoothly as the corner ahead gets more demanding, so
    the controller commits lateral authority BEFORE the car is off-centre
    approaching a corner -- see adaptive_Q_lookahead's docstring for the
    motivating failure mode. 1.0 with no corner ahead; saturates toward
    boost_max.

    With demand_normalised=True (default) the input is corner DEMAND (see
    _corner_demand) rather than raw curvature, which is what makes one set
    of constants work for gradual/sharp/U-turn corners and makes boost_max
    actually reachable. False restores the legacy raw-kappa curve
    (1 - 1/(1 + k*kappa_max_abs)) for A/B comparison.

    alat_flat/alat_slope/alat_intercept are forwarded to _corner_demand's
    ceiling law; defaults match settings.py's ALAT_CEILING_* constants.
    """
    if demand_normalised:
        frac = _demand_frac(
            _corner_demand(kappa_max_abs, car_speed, alat_flat=alat_flat,
                           alat_slope=alat_slope, alat_intercept=alat_intercept),
            demand_half,
        )
    else:
        frac = 1.0 - 1.0 / (1.0 + k * kappa_max_abs)
    return 1.0 + (boost_max - 1.0) * frac


def lookahead_epsi_approach_boost(kappa_max_abs, car_speed=0.0, boost_max=1.5, k=8.0,
                                   demand_normalised=True, demand_half=0.5,
                                   alat_flat=7.5, alat_slope=0.47, alat_intercept=2.46):
    """
    Continuous (no threshold/discontinuity) multiplier on Q[2,2] (heading
    error) that rises smoothly as the corner ahead gets more demanding --
    same shape and signal as lookahead_approach_boost, applied to heading
    instead of lateral error. Addresses short/sudden corners showing
    insufficient commitment on BOTH lateral and heading error: without this,
    Q[2,2] only gets anticipatory help from lookahead_exit_boost (which only
    activates AFTER a peak has already been recorded, decaying through the
    exit), so heading error would have no boost at all before/during
    turn-in. Composes multiplicatively with lookahead_exit_boost on the same
    Q[2,2] entry -- the two are never both large at once in practice
    (approach rises while still approaching a peak; exit only starts
    contributing once a peak has been latched and decays afterward) but
    multiplying rather than taking a max keeps this continuous either way.

    alat_flat/alat_slope/alat_intercept are forwarded to _corner_demand's
    ceiling law; defaults match settings.py's ALAT_CEILING_* constants.
    """
    if demand_normalised:
        frac = _demand_frac(
            _corner_demand(kappa_max_abs, car_speed, alat_flat=alat_flat,
                           alat_slope=alat_slope, alat_intercept=alat_intercept),
            demand_half,
        )
    else:
        frac = 1.0 - 1.0 / (1.0 + k * kappa_max_abs)
    return 1.0 + (boost_max - 1.0) * frac


def lookahead_exit_boost(last_peak_kappa_abs, dist_since_peak, decay_dist=5.0,
                          k_exit_norm=0.05, boost_max=1.5):
    """
    Continuous multiplier on Q[2,2] (heading error) that is largest right at
    a corner's peak curvature and decays linearly to 1.0 over decay_dist
    metres of travel afterward, scaled by how sharp that corner was (a
    gentle bend gets less exit correction than a hairpin). Returns 1.0
    (no-op) once fully decayed or if no peak has been recorded yet
    (dist_since_peak == inf).
    """
    if dist_since_peak >= decay_dist or not np.isfinite(dist_since_peak):
        return 1.0
    decay_frac = 1.0 - dist_since_peak / decay_dist
    sharpness = last_peak_kappa_abs / (last_peak_kappa_abs + k_exit_norm)
    return 1.0 + (boost_max - 1.0) * decay_frac * sharpness


def lookahead_yaw_rate_relax(kappa_max_abs, car_speed=0.0, floor=0.5, k=8.0,
                              demand_normalised=True, demand_half=0.5,
                              alat_flat=7.5, alat_slope=0.47, alat_intercept=2.46):
    """
    Continuous (no threshold/discontinuity) multiplier on Q[3,3] (yaw rate r)
    that FALLS smoothly as the corner ahead gets more demanding -- the
    mirror image of lookahead_approach_boost's rise, applied to yaw-rate
    penalty instead of lateral-error penalty. 1.0 with no corner ahead (full
    yaw-rate damping as tuned); floors toward floor as demand rises, so the
    MPC is less penalised for the fast rotation a real turn-in needs, and --
    because this uses the same forward lookahead as the Q[0,0] boost, not
    the car's current-position kappa -- the relaxation is already in effect
    before the car reaches the corner, addressing "turns late/slowly" rather
    than only reacting once already mid-turn.

    alat_flat/alat_slope/alat_intercept are forwarded to _corner_demand's
    ceiling law; defaults match settings.py's ALAT_CEILING_* constants.
    """
    if demand_normalised:
        frac = _demand_frac(
            _corner_demand(kappa_max_abs, car_speed, alat_flat=alat_flat,
                           alat_slope=alat_slope, alat_intercept=alat_intercept),
            demand_half,
        )
    else:
        frac = 1.0 - 1.0 / (1.0 + k * kappa_max_abs)
    return 1.0 - (1.0 - floor) * frac


def lookahead_straight_boost(kappa_max_abs, boost_max, k):
    """
    Continuous (no threshold/discontinuity) multiplier that FALLS from
    boost_max toward 1.0 as the largest curvature within the lookahead
    window grows -- the mirror image of lookahead_yaw_rate_relax's fall
    (which relaxes a cost approaching a corner), used here to instead RAISE
    a cost on straights and fade it back to baseline as a corner is detected
    ahead. Shared helper for Q[2,2] (heading error), Q[3,3] (yaw rate), and
    R[0,0] (steering effort) straight-line boosts, keeping the car pointed
    straighter, damping yaw wander, and discouraging steering deflection
    when no corner is near. A heading-error (Q[2,2]) boost_max should be
    kept small relative to Q[3,3]/R[0,0]'s: a stronger heading-error weight
    on straights amplifies the QP's reaction to ordinary heading noise, the
    exact small-error hunting adaptive_Q_scaling exists to fight elsewhere.
    k controls how sharply the boost fades as curvature is detected ahead --
    a much higher k for R[0,0] than Q[2,2]/Q[3,3] collapses it to baseline
    almost as soon as a real turn is asked for, rather than lingering into
    it.

    With a boost_max below 1.0 this instead REDUCES a cost on a clear
    straight (e.g. Q[0,0]'s lateral-error floor) -- the derivation makes no
    assumption boost_max > 1.0, it is simply "blend from boost_max at
    kappa_max_abs=0 to 1.0 as kappa_max_abs grows".
    """
    return 1.0 + (boost_max - 1.0) * (1.0 / (1.0 + k * kappa_max_abs))


def steer_effort_straight_boost(kappa_max_abs, boost_max=1.5, k=20.0):
    """
    R[0,0] (steering EFFORT -- how far the wheel is turned -- NOT
    rate-of-change; that's R_rate[0,0], boosted separately by
    steer_rate_anti_hunt) instance of lookahead_straight_boost: boost_max on
    a clear straight, fading sharply (k defaults much sharper than the
    Q-side straight boosts) toward 1.0 as a corner enters the lookahead
    window. Composes with adaptive_R_scaling's existing speed-dependent
    scaling on R[0,0] via multiplication.
    """
    return lookahead_straight_boost(kappa_max_abs, boost_max, k)


def uturn_severity(heading_change_abs, thresh_rad=np.radians(60.0), sat_rad=np.radians(120.0)):
    """
    Continuous 0..1 "how much of a U-turn is coming" score from the
    accumulated |heading change| over the lookahead window (see
    lookahead_curvature_profile). 0 below thresh_rad -- ordinary corners
    score nothing, so this never disturbs already-working sudden-corner
    behaviour -- ramping linearly to 1.0 at sat_rad. Clamped both ends, so
    it is continuous everywhere and bounded regardless of how extreme the
    path is.

    Exists because every other lookahead mechanism above keys off
    kappa_max_abs (peak curvature MAGNITUDE), and a long, GRADUAL U-turn has
    unremarkable peak curvature (large radius) even though it demands a huge
    total rotation -- accumulated heading change catches that case
    regardless of how sharp any single point is.
    """
    if sat_rad <= thresh_rad:
        return 0.0
    return float(np.clip((abs(heading_change_abs) - thresh_rad) / (sat_rad - thresh_rad), 0.0, 1.0))


def uturn_boost(severity, boost_max):
    """
    Blend 1.0 -> boost_max by a 0..1 severity from uturn_severity. Used for
    both the Q[0,0]/Q[2,2] U-turn boosts (boost_max > 1) and the Q[3,3]
    yaw-rate relaxation (boost_max < 1, blending downward instead).
    """
    return 1.0 + (boost_max - 1.0) * severity


def adaptive_Q_lookahead(Q_base, kappa_max_abs, car_speed, last_peak_kappa_abs,
                          dist_since_peak, heading_change_abs, enabled=False,
                          demand_normalised=True,
                          ey_straight_floor=0.7, ey_straight_k=20.0,
                          epsi_straight_boost_max=1.1, epsi_straight_k=8.0,
                          r_straight_boost_max=1.5, r_straight_k=8.0,
                          uturn_ey_boost_max=1.6, uturn_epsi_boost_max=1.6,
                          uturn_r_relax_floor=0.6,
                          uturn_thresh_rad=np.radians(60.0),
                          uturn_sat_rad=np.radians(120.0),
                          demand_half=0.5,
                          alat_flat=7.5, alat_slope=0.47, alat_intercept=2.46,
                          exit_decay_dist_floor=5.0, exit_decay_time_s=2.5,
                          exit_decay_dist_max=25.0):
    """
    Full lookahead corner-anticipation Q-boost: combines
    lookahead_approach_boost, lookahead_epsi_approach_boost,
    lookahead_exit_boost, lookahead_yaw_rate_relax, lookahead_straight_boost
    (Q[2,2]/Q[3,3] straight-line variants) and the U-turn boosts into a
    single Q matrix, in the same order and composition mpc_core.py's
    MPCController.compute() applies them live -- see this module's docstring
    for why: the corner boost is applied FIRST, then any centred-softening
    (adaptive_Q_scaling) the caller applies afterward multiplies on top of
    this result, so a corner boost is never silently cancelled by the
    centred-softening floor while both stay continuous.

    Motivated by adaptive_Q_scaling's documented late-turn-in failure mode:
    that function only looks at the CURRENT |e_y|, so a well-tracked
    straight right before a corner looks identical to a straight that stays
    straight -- it can discount Q[0,0] right as the car needs full lateral
    authority to turn in. This instead uses a forward-looking curvature scan
    (lookahead_curvature_profile) so the boost is already in effect before
    the car is off-centre.

    enabled=False returns Q_base unmodified -- not even copied.

    Parameters
    ----------
    Q_base : np.ndarray, shape (8,8)
        Base state cost to boost. Not modified in-place.
    kappa_max_abs, heading_change_abs : float
        This tick's lookahead_curvature_profile() output.
    car_speed : float
        Current longitudinal speed (m/s), used by the demand-normalised
        boost curves.
    last_peak_kappa_abs, dist_since_peak : float
        This controller instance's persistent peak-tracker state -- the
        caller owns update_lookahead_peak()'s state dict and passes its
        current values in each tick, AFTER calling update_lookahead_peak
        for this tick (mirrors mpc_core.py's ordering).
    demand_normalised : bool
        See lookahead_approach_boost. Default True; set False to A/B against
        the legacy raw-curvature curve.
    ey_straight_floor, ey_straight_k : float, optional
        lookahead_straight_boost shape for Q[0,0]'s clear-straight lateral
        REDUCTION (floor < 1.0, so this lowers Q[0,0] on a straight).
    epsi_straight_boost_max, epsi_straight_k : float, optional
        lookahead_straight_boost shape for Q[2,2]'s clear-straight boost.
    r_straight_boost_max, r_straight_k : float, optional
        lookahead_straight_boost shape for Q[3,3]'s clear-straight boost.
    uturn_ey_boost_max, uturn_epsi_boost_max, uturn_r_relax_floor : float, optional
        uturn_boost maxima for Q[0,0]/Q[2,2] (>1, extra commitment) and
        Q[3,3] (<1, yaw-rate relaxation) at full U-turn severity.
    uturn_thresh_rad, uturn_sat_rad : float, optional
        uturn_severity's engage/saturate accumulated-heading-change bounds.
    demand_half : float, optional
        Corner demand at which a demand-normalised boost reaches half its
        max; forwarded to the three lookahead_*_boost/relax calls below.
    alat_flat, alat_slope, alat_intercept : float, optional
        The a_lat ceiling law, forwarded through those same three calls into
        _corner_demand/_alat_ceiling_at.
    exit_decay_dist_floor, exit_decay_time_s, exit_decay_dist_max : float, optional
        lookahead_exit_boost's decay_dist is now car_speed * exit_decay_time_s,
        clamped to [exit_decay_dist_floor, exit_decay_dist_max] -- same shape
        as lookahead_dist above. Added 2026-08-11 (numeric-parity mirror of
        the live mpc_core.py fix): a FIXED decay_dist undershoots at speed --
        |e_y|/|e_psi| don't peak right at the apex, they peak several seconds
        of travel after it (the car is still sliding wide/yawing back through
        the exit), so a short fixed window had already fully decayed by the
        time tracking error was at its worst. Mirrors MPCParams.
        adaptive_q_lookahead_exit_decay_dist/_time_s/_dist_max.

    All of the above default to the previously-hardcoded values, so a caller
    that passes none of them gets behaviour identical to before they became
    parameters. settings.py's ADAPTIVE_Q_STRAIGHT_* / ADAPTIVE_Q_UTURN_* /
    ADAPTIVE_Q_DEMAND_HALF / ALAT_CEILING_* mirror them 1:1 (as does the
    live side's MPCParams).

    Returns
    -------
    Q : np.ndarray, shape (8,8)
        Q_base unchanged if enabled=False. Otherwise a copy with
        Q[0,0]/Q[2,2]/Q[3,3] scaled by the composed lookahead + U-turn
        boosts.
    """
    if not enabled:
        return Q_base

    # The a_lat ceiling law + demand half-point, forwarded identically into
    # each of the three demand-normalised boost/relax curves below.
    _demand_kw = dict(demand_half=demand_half, alat_flat=alat_flat,
                      alat_slope=alat_slope, alat_intercept=alat_intercept)

    Q = np.array(Q_base, copy=True)
    Q[0, 0] *= lookahead_approach_boost(kappa_max_abs, car_speed,
                                        demand_normalised=demand_normalised, **_demand_kw)
    Q[0, 0] *= lookahead_straight_boost(kappa_max_abs, ey_straight_floor, ey_straight_k)
    Q[2, 2] *= lookahead_epsi_approach_boost(kappa_max_abs, car_speed,
                                             demand_normalised=demand_normalised, **_demand_kw)
    exit_decay_dist = float(np.clip(
        car_speed * exit_decay_time_s, exit_decay_dist_floor, exit_decay_dist_max
    ))
    Q[2, 2] *= lookahead_exit_boost(last_peak_kappa_abs, dist_since_peak,
                                    decay_dist=exit_decay_dist)
    Q[2, 2] *= lookahead_straight_boost(kappa_max_abs, epsi_straight_boost_max, epsi_straight_k)
    Q[3, 3] *= lookahead_yaw_rate_relax(kappa_max_abs, car_speed,
                                        demand_normalised=demand_normalised, **_demand_kw)
    Q[3, 3] *= lookahead_straight_boost(kappa_max_abs, r_straight_boost_max, r_straight_k)

    uturn = uturn_severity(heading_change_abs, thresh_rad=uturn_thresh_rad,
                           sat_rad=uturn_sat_rad)
    if uturn > 0.0:
        Q[0, 0] *= uturn_boost(uturn, uturn_ey_boost_max)
        Q[2, 2] *= uturn_boost(uturn, uturn_epsi_boost_max)
        Q[3, 3] *= uturn_boost(uturn, uturn_r_relax_floor)

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

    Acceleration scale: disabled (fixed at 1.0) as of 2026-08-10 — see the
    accel_scale assignment below for why. R[1,1] is governed by R_diag[1]
    alone, independent of vx.

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

    # accel_scale disabled (fixed at 1.0) 2026-08-10, mirroring the live
    # _adaptive_R_scaling fix: scaling R[1,1] with vx made braking effort
    # more expensive exactly at corner-entry speed and cheaper again as the
    # car decelerated mid-approach, fighting the R_diag[1] braking tuning.
    # R[1,1] is now governed by R_diag[1] alone, independent of vx.
    accel_scale = 1.0

    R_scaled = np.array(R_base, copy=True)   # Never mutate the caller's matrix
    R_scaled[0, 0] *= steer_scale            # Scale steering input cost
    R_scaled[1, 1] *= accel_scale            # Scale acceleration input cost
    return R_scaled