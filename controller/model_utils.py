"""
controller/model_utils.py — Adaptive MPC Gain Helpers

PURPOSE
-------
Provides runtime gain-shaping functions that modify the MPC's cost weight
matrices (Q, R and R_rate) on a per-step basis based on the vehicle's CURRENT
speed and estimated path curvature. This allows the MPC to behave
differently in corners versus straights, and at low speed versus high speed,
without requiring a separate set of tuned weights for each regime.

These functions implement a form of gain scheduling: the base weights (Q, R,
R_rate) are tuned offline by tuner/offline_tuner.py as if for a single operating
point, then these helpers scale them at runtime to compensate for the known
nonlinear dependence of required control authority on speed and curvature.

── Lookahead gain-scheduling family: removed ──────────────────────────────────
This file used to carry ~15 interacting mechanisms that scanned forward along
the path (producing a scalar kappa_max_abs = peak curvature within a
lookahead window) and reweighted today's Q/R cost matrices based on what's
coming up: lookahead_curvature_profile, adaptive_Q_lookahead,
lookahead_approach_boost, lookahead_epsi_approach_boost, lookahead_exit_boost,
lookahead_yaw_rate_relax, lookahead_steer_effort_relax,
lookahead_straight_boost, steer_effort_straight_boost, uturn_severity/
uturn_boost, _corner_demand/_demand_frac/_alat_ceiling_at, and the
kappa_max_abs-driven terms inside steer_rate_anti_hunt/adaptive_Q_scaling/
adaptive_R_rate. Removed because this MPC formulation already predicts state
error against the reference at each future horizon step; reweighting TODAY's
(usually near-zero) cost based on a forward scan doesn't change what the
horizon predicts when the car actually gets there, so the mechanism did
roughly nothing useful. Replaced by three CURRENT-STATE-driven factors — see
mpc_core.py's _corner_factor/_low_speed_corner_boost and their use in
compute(), mirrored here via rollout_core.py's call sites — plus a fourth,
independent heading-error-driven accel/brake asymmetry. Also removed as
unused/didn't-work: the curvature-forcing QP-disturbance term
(CURVATURE_FORCING_ENABLED, curvature_horizon_profile, the `w` parameter in
optimiser.py — structurally unsound, see docs/logs) and
low_speed_steer_rate_boost (disabled by default, gated on speed alone with no
way to distinguish wanted low-speed turn-in from unwanted post-exit wobble).
Mirrors mpc_core.py's identical removal per CLAUDE.md's parity rule.

HOW THE SCALING WORKS
---------------------
adaptive_R_rate (curvature-based):
    In a tight corner the vehicle must change steering direction quickly to
    track the path, so penalising steering rate-of-change (R_rate[0,0]) too
    heavily would prevent the needed responsiveness. The scale factor
    1/(1 + 3*κ) is a saturating function: at zero curvature (straight) it
    equals 1.0 (no softening); at high curvature it floors at a "during a
    corner" value driven by the car's CURRENT-position curvature.

adaptive_R_scaling (speed-based):
    At higher speeds, a unit of steering angle produces a much larger lateral
    force and path deviation than at low speed (because lateral acceleration
    ≈ vx² * κ). The steering cost is therefore increased with speed to
    discourage large steering commands that would be destabilising at high
    speed. The Hill-function form A*vx/(vx_half + vx) is a saturating
    ramp: it rises steeply at low speeds and asymptotes to A=1.5 (so the
    maximum steer scale factor is 1 + 1.5 = 2.5). Acceleration effort
    (see settings.R_A_ACCEL/R_A_BRAKE below) is NOT speed-scaled here at all
    (accel_scale fixed at 1.0) -- a speed-dependent accel_scale would make
    braking authority rise with speed exactly where corner-entry braking
    needs to be strongest, then relax again as the car decelerates
    mid-approach even though heading error/curvature are still climbing,
    fighting whatever R_A_BRAKE tuning is trying to achieve. See
    adaptive_R_scaling's own docstring below for the full reasoning.

adaptive_Q_scaling (current lateral error):
    Softens Q[0,0] (lateral-error cost) when the car is already close to the
    centreline, to reduce small-error hunting/chatter. Reactive only — it
    looks at the car's CURRENT |e_y|, not the path ahead.

steer_rate_anti_hunt (current curvature + lateral + heading error):
    Stacks on top of adaptive_R_rate: boosts R_rate[0,0] ABOVE the tuned
    baseline (rather than softening it) when the car is simultaneously
    straight, centred, and well-aligned, to suppress residual steering
    hunt/chatter in that specific regime.

corner_factor / low_speed_corner_boost (current curvature, replaces the
lookahead family):
    corner_factor is a single continuous 0 (straight) -> 1 (full corner)
    curve driven by CURRENT |kappa|, symmetric for both rising (entry) and
    falling (exit) curvature — no forward scan, no decay-distance timer.
    low_speed_corner_boost adds an extra push in the same direction, gated
    multiplicatively on corner_factor so it cannot fire on low speed alone.
    Both feed a shared _blend(straight_val, corner_val, corner_frac) lerp per
    weight in rollout_core.py, mirroring mpc_core.py's compute().

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


def adaptive_R_rate(kappa, R_rate_base, enable_in_corners=True,
                     during_floor=0.625):
    """
    Scale the steering rate-of-change cost R_rate[0,0] based on path curvature.

    In straight-line driving (κ ≈ 0), the full R_rate steering penalty applies,
    discouraging unnecessary steering jitter. In tight corners (large κ), the
    penalty is softened so the controller can make the larger steering rate
    changes needed to track the curve, driven by the CURRENT-position kappa:
        scale = max(0.625, 1 / (1 + 3 * κ))
    At κ = 0.0 (straight):         scale = 1.00 → no change to R_rate
    At κ = 0.1 (R=10 m corner):    scale = 0.77 → moderate softening
    At κ → ∞:                       scale → 0.625 → floor

    The floor is kept fairly high (0.625, not lower) because over-relaxing
    this cost is exactly what lets steering sign-reversal chatter grow in
    corners: R_rate[0,0] is the one cost term that directly discourages
    rapid steer-sign-flipping, so cutting it hard at high curvature works
    against damping instead of with it. It keeps some softening available
    in genuinely tight corners (where the vehicle does need to change
    steering direction quickly) while still ensuring the rate cost never
    fully vanishes, which would allow arbitrarily rapid steering
    oscillations. A floor set too low here shows up as steering oscillating
    through zero several times per second mid-corner while e_y/e_psi stay
    small -- an under-damped steering-rate hunt, not a tracking-error
    problem, which needs less softening of the rate cost while actually
    turning rather than more lateral/heading authority.

    Only the current-position floor is implemented; there is no
    forward-scan entering-floor.

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
        TEMPORARY/EXPERIMENTAL, NOT VALIDATED. True (default) preserves the
        original continuous softening above, keeping R_rate reduction ACTIVE
        in corners with no threshold/discontinuity -- this is what you want
        if the goal is "reduce R_rate when turning". False uses a
        kappa_straight=0.03 cutoff to mean "not cornering": below it,
        softening still applies as normal (it barely does anything that
        close to straight anyway); at or above it ("cornering"), softening
        is switched off entirely and R_rate[0,0] gets the full, unscaled
        baseline cost -- deliberately undoing the softening this function
        exists to provide. The discontinuous cost jump this introduces at
        the cutoff can spike QP solver iterations and cause severe lag
        specifically in corners; only disable this to investigate that
        failure mode, not as a validated tuning choice.
    during_floor : float, optional
        The floor described above. Defaults match settings.py's
        ADAPTIVE_R_RATE_DURING_FLOOR (and the live side's
        MPCParams.adaptive_r_rate_during_floor), so a caller that passes
        nothing gets the tuned behaviour unchanged. The ramp sharpness (3.0)
        and the kappa_straight=0.03 cutoff are deliberately NOT parameters --
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
        scale = max(during_floor, 1.0 / (1.0 + 3.0 * abs(kappa)))
    R[0, 0] *= scale                                    # Apply only to steering rate cost
    return R


def steer_rate_anti_hunt(kappa, e_y, R_rate_base, enabled=False, e_psi=0.0):
    """
    TEMPORARY/EXPERIMENTAL (fsds sim only, off by default): heavily penalise
    steering-rate-of-change on top of adaptive_R_rate's existing curvature
    softening, strongest when the car is centred (|e_y| small), well-aligned
    (|e_psi| small), AND not currently curving (kappa small). Mirrors
    mpc_core.py's _steer_rate_anti_hunt, keep both in sync.

    Continuous, not a hard AND-gated threshold: boost_kappa, boost_ey, and
    boost_epsi each saturate independently toward 1.0 as their input
    shrinks toward 0 (same saturating-curve style as adaptive_R_rate's own
    floor); their product is the applied scale, so the full boost_max only
    applies when all three are near their "straight, centred, and aligned"
    ideal, and it fades smoothly -- never snaps -- as any one of them grows.

    boost_kappa/boost_ey/boost_epsi are current-state signals only (no
    forward-scan term).

    e_psi (radians -- same units _error_state's e_psi already uses
    internally) guards against a car that enters a straight MISALIGNED
    (large |e_psi|, small |e_y| -- e.g. just exited a corner still pointed
    the wrong way): without it, kappa/e_y alone can't distinguish "straight
    and correctly aligned" from "straight but needs to yaw back into line",
    making exactly the correction it needs artificially expensive.
    e_psi=0.0 (default) makes this term a no-op for callers that don't pass
    it. k_epsi=23.0 sets half-fade at ~2.5 deg of e_psi.

    NOT VALIDATED. Gated behind STEER_RATE_ANTI_HUNT_ENABLED (default False
    in settings.py) so it can be evaluated without affecting any existing
    tuned behaviour. Mirrors adaptive_Q_scaling's opt-in pattern: disabled
    callers get R_rate_base back completely unmodified.

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

    # Relaxed 2026-08-19 (halved from 60.0/30.0/23.0) -- see mpc_core.py's
    # _steer_rate_anti_hunt for the full reasoning (faded out too fast on
    # gentle curves, leaving residual jitter under-damped). Keep in sync.
    k_kappa, k_ey, k_epsi = 30.0, 15.0, 11.5
    boost_max = 6.0
    boost_kappa = 1.0 / (1.0 + k_kappa * abs(kappa))
    boost_ey    = 1.0 / (1.0 + k_ey * abs(e_y))
    boost_epsi  = 1.0 / (1.0 + k_epsi * abs(e_psi))
    scale = 1.0 + (boost_max - 1.0) * boost_kappa * boost_ey * boost_epsi

    R = np.array(R_rate_base, copy=True)
    R[0, 0] *= scale
    return R


def reversal_penalty_boost(u_prev_steer, R_rate_base, enabled=False,
                            boost_max=4.0, k=8.0):
    """
    TEMPORARY/EXPERIMENTAL (fsds sim only, off by default): soft constraint
    against steering REVERSALS (a tick-to-tick sign flip), approximated by
    boosting R_rate[0,0] whenever LAST tick's steering command was already
    close to zero -- the one state a reversal must pass through. Mirrors
    mpc_core.py's _reversal_penalty_boost, keep both in sync.

    A reversal can't be detected directly inside one solve (it depends on
    THIS tick's own decision, the thing being optimised), so this penalises
    the precondition instead: the closer steering already sits to zero, the
    more expensive it is to change further this tick, making a full sign
    flip specifically disproportionately costly relative to a same-side
    ramp of the same magnitude made from a large starting angle.

    Same saturating-curve style as steer_rate_anti_hunt (single input here).
    k=8.0 (rad^-1) sets half-boost at ~7.2 deg of PREVIOUS steering.
    Deliberately keyed on u_prev (a known constant by solve time), not the
    current solve's own steering variable -- using the variable itself would
    make the cost non-convex.

    Parameters
    ----------
    u_prev_steer : float
        LAST tick's actual commanded steering (rad). Sign does not matter;
        only magnitude is used.
    R_rate_base : np.ndarray, shape (2, 2)
        Rate-of-change cost matrix to boost -- pass the ALREADY
        curvature/anti-hunt-adjusted output so mechanisms compose rather
        than one undoing the other.
    enabled : bool, optional
        Master off-switch. False (default) returns R_rate_base completely
        unmodified -- not even copied.
    boost_max : float, optional
        Ceiling multiplier, applied when u_prev_steer is exactly zero.
    k : float, optional
        Fade rate (1/rad) of the boost as |u_prev_steer| grows.

    Returns
    -------
    R : np.ndarray
        R_rate_base unchanged if enabled=False. Otherwise a copy with
        R[0,0] boosted when the previous steering command was near zero.

    Called by: sim/rollout_core.py (run_core_rollout, LTV-QP path) and
    controller/nmpc_optimiser.py (compute_step, NMPC path) -- opt-in only,
    via REVERSAL_PENALTY_ENABLED / NMPC_REVERSAL_PENALTY_ENABLED respectively.
    """
    if not enabled:
        return R_rate_base

    boost_near_zero = 1.0 / (1.0 + k * abs(u_prev_steer))
    scale = 1.0 + (boost_max - 1.0) * boost_near_zero

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


def _corner_factor(kappa, k):
    """
    0 (straight) -> 1 (full corner), a single continuous saturating curve
    of the CURRENT |kappa| (the ~1m-preview curvature the caller already
    computes every tick via curvature_estimate(), same signal
    adaptive_R_rate/steer_rate_anti_hunt use). Deliberately the SAME
    functional shape for both rising (entry) and falling (exit) curvature --
    no separate decay-distance timer, no hysteresis state: this is a pure
    function of the current instantaneous signal, replacing the whole
    deleted lookahead approach/exit-boost family (see this module's
    docstring). Numeric-parity mirror of mpc_core.py's _corner_factor -- k
    is settings.CORNER_FACTOR_K.
    """
    return 1.0 - 1.0 / (1.0 + k * abs(kappa))


def _blend(straight_val, corner_val, corner_frac):
    """
    Simple linear interpolation from straight_val (corner_frac=0) to
    corner_val (corner_frac=1). Shared helper for every current-state
    Q/R_rate weight schedule in rollout_core.py -- see _corner_factor/
    _low_speed_corner_boost for how corner_frac itself is built. Numeric-
    parity mirror of mpc_core.py's _blend.
    """
    return straight_val + (corner_val - straight_val) * corner_frac


def _low_speed_corner_boost(car_speed, corner_factor, v_half, max_extra):
    """
    Extra push in the SAME direction as corner_factor's "full corner"
    endpoint (i.e. an ADDITIONAL fraction, not a separate blend), active
    ONLY when corner_factor > 0 AND speed is low -- multiplicatively gated
    on corner_factor so this is an exact no-op on a straight regardless of
    speed. This is what makes it safe where the deleted
    low_speed_steer_rate_boost was NOT: that function fired on low speed
    ALONE, with no curvature/lookahead signal to distinguish wanted
    low-speed turn-in from unwanted post-exit wobble, and live testing
    found it taxed both indistinguishably. Gating on corner_factor instead
    of a forward scan keeps this current-state: it only ever pushes harder
    on a corner the car is ALREADY turning through, never anticipates one.

    car_speed=0 gives the full max_extra (scaled by corner_factor); the
    boost falls off toward 0 as speed rises past v_half (the speed at
    which half of max_extra remains), same saturating-curve style used
    throughout this file. Numeric-parity mirror of mpc_core.py's
    _low_speed_corner_boost.
    """
    v = max(abs(car_speed), 0.0)
    speed_frac = v_half / (v_half + v) if v_half > 0.0 else 0.0
    return corner_factor * max_extra * speed_frac


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

    Acceleration scale: disabled (fixed at 1.0) — see the accel_scale
    assignment below for why. R[1,1] is governed by R_diag[1] alone,
    independent of vx.

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

    # accel_scale disabled (fixed at 1.0), mirroring the live
    # _adaptive_R_scaling: scaling R[1,1] with vx would make braking effort
    # more expensive exactly at corner-entry speed and cheaper again as the
    # car decelerates mid-approach, fighting the R_diag[1] braking tuning.
    # R[1,1] is governed by R_diag[1] alone, independent of vx.
    accel_scale = 1.0

    R_scaled = np.array(R_base, copy=True)   # Never mutate the caller's matrix
    R_scaled[0, 0] *= steer_scale            # Scale steering input cost
    R_scaled[1, 1] *= accel_scale            # Scale acceleration input cost
    return R_scaled