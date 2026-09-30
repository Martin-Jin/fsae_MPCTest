"""
fsae_control/nmpc/dynamics.py — NMPC prediction model

Frenet-frame vehicle model: `_Plant` constants, tyre forces, the continuous
dynamics `_f`/`_f_scalar` and their RK4 steps `_step`/`_step_scalar`.
"""

import math
from dataclasses import dataclass

import numpy as np

from fsae_control.nmpc.layout import (
    IDX_A,
    IDX_DELTA,
    IDX_EPSI,
    IDX_EY,
    IDX_R,
    IDX_S,
    IDX_VX,
    IDX_VY,
    NX,
    _DENOM_FLOOR,
)


@dataclass
class _Plant:
    """
    Vehicle constants for the prediction model.

    Every value defaults to MPCController.__init__'s own hardcoded value, kept
    here so this module is self-contained but MUST be kept in sync with that
    constructor (and, per CLAUDE.md, with vehicle_physics.VehicleParams) if the
    plant is ever re-identified. These are NOT independent tunables.
    """
    lf: float = 0.70
    lr: float = 0.85
    m: float = 255.0
    Iz: float = 150.0
    Cf: float = 29155.47766921484
    Cr: float = 19512.3421655211
    tau_delta: float = 0.08
    tau_a: float = 0.02
    # Kinematic->dynamic blend breakpoints, identical to _discrete_model's
    # alpha = clip((v_x - 1.0)/(2.5 - 1.0), 0, 1).
    v_blend_lo: float = 1.0
    v_blend_hi: float = 2.5
    # FSDS's measured sustained lateral-acceleration ceiling law, from
    # the _Plant.alat_ceiling_flat/_slope/_intercept constants below (the SAME
    # law model/vehicle_physics's VehicleParams.alat_ceiling_at uses on the
    # offline side — measured by open-loop system-ID, see CLAUDE.md's
    # "MECHANISM: a dynamically-enforced lateral-acceleration ceiling").
    # NMPCController.__init__ builds _Plant with these defaults and only sets
    # alat_ceiling_enabled (from nmpc_alat_ceiling_enabled).
    #
    # Why the PREDICTION needs it: linear tyres produce unbounded lateral
    # force, so without this the model believes it can hold any corner at any
    # speed. The plant cannot (FSDS clamps sustained a_lat at ~7.5-9 m/s^2),
    # so the NMPC would command a yaw rate that never arrives, see the error
    # persist, and command more — which offline showed up as a large-amplitude
    # steering oscillation through the tight corners, and eventually a spin
    # (late_turn_in_investigation.md Part 16 §16.6). The LTV-QP has the same
    # optimistic tyre model but is shielded from it by heavy steering-effort /
    # steering-rate damping and by never predicting the corner at all.
    alat_ceiling_enabled: bool = True
    alat_ceiling_flat: float = 7.5
    alat_ceiling_slope: float = 0.47
    alat_ceiling_intercept: float = 2.46


def _tyre_forces(X, p):
    """
    Front/rear axle lateral tyre force (N), vectorised over stages, AFTER
    the existing soft alat-ceiling tanh saturation (if p.alat_ceiling_enabled)
    -- i.e. the SAME F_yf/F_yr that actually enter _f's dynamics, not the
    raw linear-tyre value. A function of STATE only (v_y, r, delta are all
    states; steering/accel COMMAND only affects delta's rate, not the
    instantaneous force at this stage), which is what lets these ride along
    as extra _outputs() rows with no extra rollout cost -- see NH_FRICTION's
    comment. Kept a line-by-line mirror of _f's own force computation; if
    that changes, update this too.
    """
    v_x = X[:, IDX_VX]
    v_y = X[:, IDX_VY]
    r   = X[:, IDX_R]
    d   = X[:, IDX_DELTA]

    v_safe = np.maximum(np.abs(v_x), p.v_blend_hi)
    alpha_f = np.arctan((v_y + p.lf * r) / v_safe) - d
    alpha_r = np.arctan((v_y - p.lr * r) / v_safe)
    F_yf = -2.0 * p.Cf * alpha_f
    F_yr = -2.0 * p.Cr * alpha_r

    if p.alat_ceiling_enabled:
        cos_d = np.cos(d)
        a_y = (F_yf * cos_d + F_yr) / p.m
        ceil = np.maximum(p.alat_ceiling_flat,
                          p.alat_ceiling_slope * np.abs(v_x) + p.alat_ceiling_intercept)
        ratio = np.abs(a_y) / ceil
        sat = np.where(ratio > 1e-6, np.tanh(ratio) / np.maximum(ratio, 1e-6), 1.0)
        F_yf = F_yf * sat
        F_yr = F_yr * sat
    return F_yf, F_yr


def _f(X, U, ref, p):
    """
    Continuous-time dynamics, vectorised over horizon stages.

    X: (M, NX), U: (M, NU) -> (M, NX) state derivatives. `ref` supplies
    kappa(s); `p` is a _Plant. See the module docstring for the equations.
    """
    s     = X[:, IDX_S]
    e_y   = X[:, IDX_EY]
    e_psi = X[:, IDX_EPSI]
    v_x   = X[:, IDX_VX]
    v_y   = X[:, IDX_VY]
    r     = X[:, IDX_R]
    d     = X[:, IDX_DELTA]
    a     = X[:, IDX_A]

    kap = ref.kappa_at(s)

    denom = 1.0 - kap * e_y
    # Keep the denominator away from zero WITHOUT flipping its sign (see
    # _DENOM_FLOOR): a sign flip would reverse the predicted direction of
    # travel along the path.
    denom = np.where(denom >= 0.0,
                     np.maximum(denom, _DENOM_FLOOR),
                     np.minimum(denom, -_DENOM_FLOOR))

    cos_ep = np.cos(e_psi)
    sin_ep = np.sin(e_psi)
    s_dot   = (v_x * cos_ep - v_y * sin_ep) / denom
    e_y_dot = v_x * sin_ep + v_y * cos_ep
    e_psi_dot = r - kap * s_dot

    # Tyre slip angles: floor the denominator at the top of the
    # kinematic/dynamic blend band so the linear-tyre expressions stay finite
    # as v_x -> 0 (the blend below is what actually handles low speed).
    v_safe = np.maximum(np.abs(v_x), p.v_blend_hi)
    alpha_f = np.arctan((v_y + p.lf * r) / v_safe) - d
    alpha_r = np.arctan((v_y - p.lr * r) / v_safe)
    F_yf = -2.0 * p.Cf * alpha_f
    F_yr = -2.0 * p.Cr * alpha_r
    cos_d = np.cos(d)

    if p.alat_ceiling_enabled:
        # Smooth saturation of the lateral forces so the predicted lateral
        # acceleration cannot exceed the measured FSDS ceiling at this speed.
        # tanh(x)/x is 1 to second order at x=0, so small-signal handling (and
        # therefore the linear-region cornering stiffness the weights were
        # tuned against) is unchanged; only the demanded-beyond-possible region
        # is bent over. Both axle forces are scaled by the same factor, which
        # preserves the yaw-moment balance and hence the model's understeer
        # character while limiting its magnitude.
        a_y = (F_yf * cos_d + F_yr) / p.m
        ceil = np.maximum(p.alat_ceiling_flat,
                          p.alat_ceiling_slope * np.abs(v_x) + p.alat_ceiling_intercept)
        ratio = np.abs(a_y) / ceil
        sat = np.where(ratio > 1e-6, np.tanh(ratio) / np.maximum(ratio, 1e-6), 1.0)
        F_yf = F_yf * sat
        F_yr = F_yr * sat

    blend = np.clip((v_x - p.v_blend_lo) / (p.v_blend_hi - p.v_blend_lo), 0.0, 1.0)

    # Fade tyre forces out at low speed, same `blend` the kinematic/dynamic
    # mix already uses. WITHOUT this, alpha_f/alpha_r's speed-floored
    # denominator (v_safe, needed to avoid a divide-by-zero as v_x -> 0) makes
    # a stationary tyre's slip angle track the STEERING COMMAND directly
    # (alpha_f ~= -d when v_y, r are small), so the model predicts a large
    # cornering force from steering alone at v_x = 0 -- backwards from a real
    # tyre, which generates ~zero force with no rolling contact velocity.
    # Uncommanded, this manufactured force propagates through v_y_dot_dyn/
    # r_dot_dyn into a large, entirely fictitious predicted e_y/e_psi
    # excursion over the horizon while the car has not physically moved,
    # which is what caused the NMPC's steering command to snap to the full
    # +-25 deg mechanical lock in the first ~0.5-0.7s of every run from a
    # standing start (measured 2026-08-13: nmpc_pred_ey_end reached -1.9 m
    # while v_actual was still ~0). The dynamic branch's own OUTPUT
    # (v_y_dot_dyn/r_dot_dyn) is already blended out by `blend` below, but
    # that happens too late to stop the force itself from existing --
    # F_yf/F_yr must be scaled here, at the source, not just downstream.
    F_yf = F_yf * blend
    F_yr = F_yr * blend

    v_x_dot = a + blend * r * v_y

    v_y_dot_dyn = (F_yf * cos_d + F_yr) / p.m - r * v_x
    r_dot_dyn   = (p.lf * F_yf * cos_d - p.lr * F_yr) / p.Iz

    # Kinematic branch, differentiated: r_kin = v_x*tan(d)/L, v_y_kin = lr*r_kin,
    # with d_dot from the steering actuator lag and v_x_dot = a (the r*v_y term
    # is itself blended out at low speed).
    L = p.lf + p.lr
    d_dot = (U[:, 0] - d) / p.tau_delta
    tan_d = np.tan(d)
    sec2_d = 1.0 / np.maximum(cos_d * cos_d, 1e-3)
    r_dot_kin = (a * tan_d + v_x * sec2_d * d_dot) / L
    v_y_dot_kin = p.lr * r_dot_kin

    v_y_dot = (1.0 - blend) * v_y_dot_kin + blend * v_y_dot_dyn
    r_dot   = (1.0 - blend) * r_dot_kin   + blend * r_dot_dyn

    out = np.empty_like(X)
    out[:, IDX_S]     = s_dot
    out[:, IDX_EY]    = e_y_dot
    out[:, IDX_EPSI]  = e_psi_dot
    out[:, IDX_VX]    = v_x_dot
    out[:, IDX_VY]    = v_y_dot
    out[:, IDX_R]     = r_dot
    out[:, IDX_DELTA] = d_dot
    out[:, IDX_A]     = (U[:, 1] - a) / p.tau_a
    return out


def _f_scalar(x, u, kap, p):
    """
    Scalar (single-state) form of _f, returning a tuple of 8 derivatives.

    EXISTS ONLY FOR SPEED, and is a line-by-line mirror of _f above — keep the
    two identical. The horizon rollout is inherently sequential (N steps, default 20, x 4
    RK stages x n_sub substeps), and at that size numpy's per-call overhead
    dominates completely: the vectorised _f costs ~75 us on a 1x8 array, making
    one rollout 17 ms, versus ~1 ms for this form. The Jacobian pass, which
    batches all stages into one array, still uses the vectorised _f.

    `kap` is passed in (already looked up) rather than read from `ref` so the
    caller can use PathReference.kappa_scalar's O(1) uniform-grid lookup
    instead of np.interp's ~4 us call overhead.

    the scalar-versus-vectorised check in tuner/validation/nmpc_offline_check.py
    (offline) asserts the two forms agree on randomised states — a divergence
    between them is a silent-wrong-prediction bug, so that check is not optional.
    """
    s, e_y, e_psi, v_x, v_y, r, d, a = x

    denom = 1.0 - kap * e_y
    if denom >= 0.0:
        denom = denom if denom > _DENOM_FLOOR else _DENOM_FLOOR
    else:
        denom = denom if denom < -_DENOM_FLOOR else -_DENOM_FLOOR

    cos_ep = math.cos(e_psi)
    sin_ep = math.sin(e_psi)
    s_dot = (v_x * cos_ep - v_y * sin_ep) / denom
    e_y_dot = v_x * sin_ep + v_y * cos_ep
    e_psi_dot = r - kap * s_dot

    v_safe = abs(v_x)
    if v_safe < p.v_blend_hi:
        v_safe = p.v_blend_hi
    alpha_f = math.atan((v_y + p.lf * r) / v_safe) - d
    alpha_r = math.atan((v_y - p.lr * r) / v_safe)
    F_yf = -2.0 * p.Cf * alpha_f
    F_yr = -2.0 * p.Cr * alpha_r
    cos_d = math.cos(d)

    if p.alat_ceiling_enabled:
        # Mirror of _f's lateral-acceleration ceiling saturation.
        a_y = (F_yf * cos_d + F_yr) / p.m
        ceil = p.alat_ceiling_slope * abs(v_x) + p.alat_ceiling_intercept
        if ceil < p.alat_ceiling_flat:
            ceil = p.alat_ceiling_flat
        ratio = abs(a_y) / ceil
        if ratio > 1e-6:
            sat = math.tanh(ratio) / ratio
            F_yf *= sat
            F_yr *= sat

    blend = (v_x - p.v_blend_lo) / (p.v_blend_hi - p.v_blend_lo)
    blend = 0.0 if blend < 0.0 else (1.0 if blend > 1.0 else blend)

    # Mirror of _f's low-speed tyre-force fade -- see that copy's comment for
    # why this must scale F_yf/F_yr at the source, not just blend v_y_dot_dyn/
    # r_dot_dyn downstream.
    F_yf *= blend
    F_yr *= blend

    v_x_dot = a + blend * r * v_y
    v_y_dot_dyn = (F_yf * cos_d + F_yr) / p.m - r * v_x
    r_dot_dyn = (p.lf * F_yf * cos_d - p.lr * F_yr) / p.Iz

    L = p.lf + p.lr
    d_dot = (u[0] - d) / p.tau_delta
    sec2_d = cos_d * cos_d
    sec2_d = 1.0 / (sec2_d if sec2_d > 1e-3 else 1e-3)
    r_dot_kin = (a * math.tan(d) + v_x * sec2_d * d_dot) / L
    v_y_dot_kin = p.lr * r_dot_kin

    return (
        s_dot,
        e_y_dot,
        e_psi_dot,
        v_x_dot,
        (1.0 - blend) * v_y_dot_kin + blend * v_y_dot_dyn,
        (1.0 - blend) * r_dot_kin + blend * r_dot_dyn,
        d_dot,
        (u[1] - a) / p.tau_a,
    )


def _step_scalar(x, u, ref, p, dt, n_sub):
    """
    Scalar form of _step (one dt, RK4 with n_sub substeps, exact ZOH overwrite
    of the two actuator states, v_x floored at 0). Mirror of _step — see
    _f_scalar's docstring.
    """
    h = dt / n_sub
    xk = tuple(float(v) for v in x)
    d0, a0 = xk[IDX_DELTA], xk[IDX_A]
    for _ in range(n_sub):
        k1 = _f_scalar(xk, u, ref.kappa_scalar(xk[IDX_S]), p)
        x2 = tuple(xk[i] + 0.5 * h * k1[i] for i in range(NX))
        k2 = _f_scalar(x2, u, ref.kappa_scalar(x2[IDX_S]), p)
        x3 = tuple(xk[i] + 0.5 * h * k2[i] for i in range(NX))
        k3 = _f_scalar(x3, u, ref.kappa_scalar(x3[IDX_S]), p)
        x4 = tuple(xk[i] + h * k3[i] for i in range(NX))
        k4 = _f_scalar(x4, u, ref.kappa_scalar(x4[IDX_S]), p)
        xk = tuple(
            xk[i] + (h / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i])
            for i in range(NX)
        )
    out = list(xk)
    exp_d = math.exp(-dt / p.tau_delta)
    exp_a = math.exp(-dt / p.tau_a)
    out[IDX_DELTA] = d0 * exp_d + u[0] * (1.0 - exp_d)
    out[IDX_A] = a0 * exp_a + u[1] * (1.0 - exp_a)
    if out[IDX_VX] < 0.0:
        out[IDX_VX] = 0.0
    return out


def _step(X, U, ref, p, dt, n_sub):
    """
    One dt of the discretised model, vectorised over stages: RK4 with n_sub
    substeps, then the two actuator states overwritten with their EXACT
    zero-order-hold values.

    The overwrite matters: tau_a = 0.02 s against dt = 0.05 s puts lambda*dt at
    -2.5, close to RK4's real-axis stability edge, so RK4 alone leaves a_act
    visibly short of its true value at the end of a step. The lag states are
    linear, decoupled and driven by a constant input over the step, so their
    exact solution is available — and it is the same expression compute() uses
    to integrate the real actuator state.
    """
    h = dt / n_sub
    Xk = X
    for _ in range(n_sub):
        k1 = _f(Xk, U, ref, p)
        k2 = _f(Xk + (0.5 * h) * k1, U, ref, p)
        k3 = _f(Xk + (0.5 * h) * k2, U, ref, p)
        k4 = _f(Xk + h * k3, U, ref, p)
        Xk = Xk + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
    Xk = Xk.copy()
    exp_d = math.exp(-dt / p.tau_delta)
    exp_a = math.exp(-dt / p.tau_a)
    Xk[:, IDX_DELTA] = X[:, IDX_DELTA] * exp_d + U[:, 0] * (1.0 - exp_d)
    Xk[:, IDX_A]     = X[:, IDX_A]     * exp_a + U[:, 1] * (1.0 - exp_a)
    # The car cannot be predicted into reverse; a negative v_x would also flip
    # the sign of every slip angle and make the prediction meaningless.
    np.maximum(Xk[:, IDX_VX], 0.0, out=Xk[:, IDX_VX])
    return Xk
