"""
fsae_control/lmpc/adaptive_gains.py — current-state gain scheduling

R/Q scaling, steering-rate anti-hunt, reversal-penalty boost and the
corner-factor family. Numerically identical to fsae_MPCTest/controller/
model_utils.py, kept in sync by hand (CLAUDE.md, live/offline parity).
"""

import math

import numpy as np

# ── Lookahead gain-scheduling family: removed ──────────────────────────────
# Removed ~15 mechanisms that scanned forward along the path and reweighted
# today's Q/R cost based on a corner not yet reached (approach/exit boosts,
# demand normalisation, U-turn detector, straight-line adjustments, curvature
# forcing, the precomputed CornerMap fast path — full list in
# `docs/reference/control_mechanisms.md`'s "Corner-factor scheduler (LTV-QP)" section)
# because this MPC formulation already predicts state error at each future
# horizon step; reweighting TODAY's near-zero cost based on a forward scan
# doesn't change what the horizon predicts once the car gets there. Replaced
# by _corner_factor/_low_speed_corner_boost below (current-state only) plus
# an independent heading-error-driven accel/brake asymmetry. Mirror any
# change here into fsae_MPCTest/controller/model_utils.py per CLAUDE.md's
# parity rule.

# ---------------------------------------------------------------------------
# Adaptive gain helpers
# ---------------------------------------------------------------------------

def _adaptive_R_scaling(vx: float, R_base: np.ndarray) -> np.ndarray:
    """
    Speed-dependent steering cost with a saturating (Michaelis-Menten) scale.
    steer_scale = 1 + (1.5 * vx) / (6.0 + vx)

    accel_scale is disabled (fixed at 1.0): a speed-dependent accel_scale
    (e.g. 1 + 0.05*vx) makes R[1,1] rise with speed exactly where
    corner-entry braking needs to be strongest, and relax again as the car
    decelerates mid-approach even though heading error/curvature are still
    climbing -- fighting the braking R_diag[1] tuning is trying to loosen.
    R[1,1] is governed by R_diag[1] alone, independent of vx.
    """
    vx = max(vx, 0.5)
    steer_scale = 1.0 + (1.5 * vx) / (6.0 + vx)
    accel_scale = 1.0
    R = R_base.copy()
    R[0, 0] *= steer_scale
    R[1, 1] *= accel_scale
    return R


def _steer_rate_anti_hunt(
    kappa: float,
    e_y: float,
    R_rate_base: np.ndarray,
    enabled: bool,
    e_psi: float = 0.0,
    boost_max: float = 6.0,
) -> np.ndarray:
    """
    TEMPORARY/EXPERIMENTAL, NOT VALIDATED: heavily penalise steering
    rate-of-change, strongest when the car is centred (|e_y| small),
    well-aligned (|e_psi| small), AND not currently curving (kappa small).
    Mirrors model_utils.steer_rate_anti_hunt in fsae_MPCTest -- keep both
    constants in sync. enabled=False returns R_rate_base untouched.

    Continuous, not a hard AND-gated threshold: a discontinuous step would
    risk the same QP-solver-iteration-spike problem a threshold cutoff on
    curvature already found once (see the removed adaptive-R_rate
    enable_in_corners/kappa_straight cutoff in docs/reference/retired_mechanisms.md),
    so straight-line hunting is instead penalised more strongly via a
    higher continuous ceiling.
    boost_kappa, boost_ey, and boost_epsi each saturate independently
    toward 1.0 as their input shrinks toward 0 (a saturating-curve style);
    their product is the applied scale,
    so the full boost_max (default MPCParams.anti_hunt_boost_max) only
    applies when all three are near their "straight, centred, and aligned"
    ideal, and it fades smoothly -- never snaps -- as any one of them
    grows.

    e_psi (radians, NOT the degrees used in telemetry/logging -- same units
    _error_state's x0[2]/e_psi already use internally) is included because
    without it, a car that enters a straight MISALIGNED (large |e_psi|,
    small |e_y| -- e.g. just exited a corner still pointed the wrong way)
    would get the full straight-line boost anyway, since kappa/e_y alone
    can't distinguish "straight and correctly aligned" from "straight but
    needs to yaw back into line" -- making exactly the correction it needs
    artificially expensive. k_epsi=23.0 sets half-fade at ~2.5 deg of e_psi.

    boost_kappa/boost_ey/boost_epsi are current-state signals only (no
    forward-scan term).
    """
    if not enabled:
        return R_rate_base
    # Relaxed 2026-08-19 (halved from 60.0/30.0/23.0): the original constants
    # faded the boost out too fast on genuinely gentle curves -- boost_kappa
    # was already down to ~0.45 by |kappa|=0.02 (a ~50 m-radius bend), so
    # R_rate[0,0] had mostly relaxed back toward baseline exactly where
    # residual steering jitter was still visible. Halving each k_* doubles
    # the |kappa|/|e_y|/|e_psi| each factor reaches before dropping to half
    # its max contribution (kappa: ~0.017 -> ~0.033 1/m; e_y: ~3.3 -> ~6.7 cm;
    # e_psi: ~2.5 -> ~5.0 deg). Applies to both controllers -- nmpc/solver.py
    # imports this function verbatim, not a separate copy.
    k_kappa, k_ey, k_epsi = 30.0, 15.0, 11.5
    boost_kappa = 1.0 / (1.0 + k_kappa * abs(kappa))
    boost_ey    = 1.0 / (1.0 + k_ey * abs(e_y))
    boost_epsi  = 1.0 / (1.0 + k_epsi * abs(e_psi))
    scale = 1.0 + (boost_max - 1.0) * boost_kappa * boost_ey * boost_epsi
    R = R_rate_base.copy()
    R[0, 0] *= scale
    return R


def _reversal_penalty_boost(
    u_prev_steer: float,
    R_rate_base: np.ndarray,
    enabled: bool,
    boost_max: float = 4.0,
    k: float = 8.0,
) -> np.ndarray:
    """
    TEMPORARY/EXPERIMENTAL, NOT VALIDATED: soft constraint against steering
    REVERSALS (a tick-to-tick sign flip), approximated inside the convex QP
    by boosting R_rate[0,0] whenever LAST tick's steering command
    (u_prev_steer, rad) was already close to zero -- the one state a
    reversal must pass through, since delta_cmd is continuous. A reversal
    can't be detected directly inside one solve (it depends on this tick's
    OWN decision, the thing being optimised), so this penalises the
    precondition instead: the closer steering already sits to zero, the
    more it costs to change it further this tick, making a full sign flip
    specifically (as opposed to a same-side ramp toward/away from zero)
    disproportionately expensive relative to a swing of the same size made
    from a large starting angle.

    Same saturating-curve style as _steer_rate_anti_hunt (single input here,
    not a product of several) so it fades continuously rather than snapping,
    and composes the same way: applied multiplicatively on top of whatever
    _steer_rate_anti_hunt/the corner blend already produced, never
    replacing them. enabled=False returns R_rate_base untouched.

    k=8.0 (rad^-1) sets half-boost at ~7.2 deg of PREVIOUS steering (a
    reversal starting from near-centre gets close to the full boost_max;
    one starting from a large existing angle -- already unlikely to flip
    sign in one 50ms tick without an equally large du -- is barely
    affected). Deliberately keyed on u_prev, not the CURRENT solve's u[0,0]
    (a QP variable): using the variable itself would make the cost
    non-convex (a rational function of the decision), whereas u_prev is a
    known constant by solve time, keeping this an ordinary quadratic term.
    """
    if not enabled:
        return R_rate_base
    boost_near_zero = 1.0 / (1.0 + k * abs(u_prev_steer))
    scale = 1.0 + (boost_max - 1.0) * boost_near_zero
    R = R_rate_base.copy()
    R[0, 0] *= scale
    return R


def _adaptive_Q_scaling(
    e_y: float, Q_base: np.ndarray, enabled: bool,
) -> np.ndarray:
    """
    Soften the lateral-error cost Q[0,0] when already close to the
    centreline, to reduce small-error hunting/chatter. Mirrors
    model_utils.adaptive_Q_scaling in fsae_MPCTest — see that function's
    docstring for the full mechanism and why this is disabled by default.
    enabled=False returns Q_base untouched.

    Only the current-state ey_lo/ey_hi/floor softening on CURRENT |e_y|
    below is implemented; there is no forward-scan relaxation term.
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
    Q = Q_base.copy()
    Q[0, 0] *= scale
    return Q


def _curvature(path: np.ndarray, idx: int) -> float:
    """
    Estimate signed path curvature (1/m) at waypoint idx via finite-difference.
    """
    if idx <= 0 or idx >= len(path) - 1:
        return 0.0
    s_prev = path[idx]     - path[idx - 1]
    s_next = path[idx + 1] - path[idx]
    yaw_p  = math.atan2(s_prev[1], s_prev[0])
    yaw_n  = math.atan2(s_next[1], s_next[0])
    dpsi   = math.atan2(math.sin(yaw_n - yaw_p), math.cos(yaw_n - yaw_p))
    ds     = (np.linalg.norm(s_prev) + np.linalg.norm(s_next)) * 0.5
    return dpsi / ds if ds > 1e-6 else 0.0


def _corner_factor(kappa: float, k: float) -> float:
    """
    0 (straight) -> 1 (full corner), a single continuous saturating curve
    of the CURRENT |kappa| (the ~1m-preview curvature _error_state already
    computes every tick, same signal _steer_rate_anti_hunt uses).
    Deliberately the SAME functional shape for both rising (entry)
    and falling (exit) curvature -- no separate decay-distance timer, no
    hysteresis state: this is a pure function of the current instantaneous
    signal, replacing the whole deleted lookahead approach/exit-boost
    family (see the module comment near the top of this file). k is
    MPCParams.corner_factor_k, the curve's sharpness.
    """
    return 1.0 - 1.0 / (1.0 + k * abs(kappa))


def _blend(straight_val: float, corner_val: float, corner_frac: float) -> float:
    """
    Simple linear interpolation from straight_val (corner_frac=0) to
    corner_val (corner_frac=1). Shared helper for every current-state
    Q/R_rate weight schedule in compute() -- see _corner_factor/
    _low_speed_corner_boost for how corner_frac itself is built.
    """
    return straight_val + (corner_val - straight_val) * corner_frac


def _low_speed_corner_boost(
    car_speed: float, corner_factor: float,
    v_half: float, max_extra: float,
) -> float:
    """
    Extra push in the SAME direction as corner_factor's "full corner"
    endpoint (i.e. an ADDITIONAL fraction, not a separate blend), active
    ONLY when corner_factor > 0 AND speed is low -- multiplicatively gated
    on corner_factor so this is an exact no-op on a straight regardless of
    speed. This is what makes it safe where the deleted
    _low_speed_steer_rate_boost was NOT: that function fired on low speed
    ALONE, with no curvature/lookahead signal to distinguish wanted
    low-speed turn-in from unwanted post-exit wobble, and live testing
    found it taxed both indistinguishably. Gating on corner_factor instead
    of a forward scan keeps this current-state: it only ever pushes harder
    on a corner the car is ALREADY turning through, never anticipates one.

    car_speed=0 gives the full max_extra (scaled by corner_factor); the
    boost falls off toward 0 as speed rises past v_half (the speed at
    which half of max_extra remains), same saturating-curve style used
    throughout this file.
    """
    v = max(abs(car_speed), 0.0)
    speed_frac = v_half / (v_half + v) if v_half > 0.0 else 0.0
    return corner_factor * max_extra * speed_frac
