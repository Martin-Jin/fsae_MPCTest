"""
fsae_control/nmpc/outputs.py — cost residual rows

`_outputs`: the residual vector h(x) the NMPC cost weighs, plus the optional
friction-circle rows.
"""

import numpy as np

from fsae_control.nmpc.layout import (
    IDX_EPSI,
    IDX_EY,
    IDX_PSI,
    IDX_R,
    IDX_S,
    IDX_VX,
    IDX_VY,
    IDX_X,
    IDX_Y,
    NH_FRICTION,
    NH_PROGRESS,
    NH_TRACKING,
    _DENOM_FLOOR,
)
from fsae_control.nmpc.dynamics import _tyre_forces


def _frenet_stages(X, ref, p, s_anchor=None):
    """
    Path-relative quantities per stage, in either state frame:
    (s, e_y, e_psi, e_y_dot, s_dot, kappa), all at the front axle.

    Frenet model: read straight off the state, exactly the expressions the
    cost has always used. Cartesian model (p.cartesian): the front axle
    (CoG + lf along the heading) is projected onto the path with
    PathReference.frenet_batch, seeded from `s_anchor` (stage 0's projected
    arc length, set once per tick by the solver) plus the distance the
    predicted front axle travels, and the rates come from the front axle's
    world-frame velocity, which includes the lf*r term the Frenet model's
    e_y_dot leaves out. This is the one place either frame is translated
    into the path-relative terms the cost, constraints and telemetry use.
    """
    v_x = X[:, IDX_VX]
    v_y = X[:, IDX_VY]
    if not p.cartesian:
        s = X[:, IDX_S]
        e_y = X[:, IDX_EY]
        e_psi = X[:, IDX_EPSI]
        kap = ref.kappa_at(s)
        denom = 1.0 - kap * e_y
        denom = np.where(denom >= 0.0,
                         np.maximum(denom, _DENOM_FLOOR),
                         np.minimum(denom, -_DENOM_FLOOR))
        cos_ep = np.cos(e_psi)
        sin_ep = np.sin(e_psi)
        s_dot = (v_x * cos_ep - v_y * sin_ep) / denom
        e_y_dot = v_x * sin_ep + v_y * cos_ep
        return s, e_y, e_psi, e_y_dot, s_dot, kap

    psi = X[:, IDX_PSI]
    r = X[:, IDX_R]
    c, sn = np.cos(psi), np.sin(psi)
    P = np.column_stack([X[:, IDX_X] + p.lf * c, X[:, IDX_Y] + p.lf * sn])
    travelled = np.concatenate(
        [[0.0], np.cumsum(np.hypot(np.diff(P[:, 0]), np.diff(P[:, 1])))])
    s, e_y, e_psi, th = ref.frenet_batch(P, psi, float(s_anchor) + travelled)
    v_lat = v_y + p.lf * r
    vx_w = v_x * c - v_lat * sn
    vy_w = v_x * sn + v_lat * c
    ct, st = np.cos(th), np.sin(th)
    kap = ref.kappa_at(s)
    denom = 1.0 - kap * e_y
    denom = np.where(denom >= 0.0,
                     np.maximum(denom, _DENOM_FLOOR),
                     np.minimum(denom, -_DENOM_FLOOR))
    s_dot = (vx_w * ct + vy_w * st) / denom
    e_y_dot = vy_w * ct - vx_w * st
    return s, e_y, e_psi, e_y_dot, s_dot, kap


def _outputs(X, ref, p, v_ref, friction_circle_enabled=False,
             progress_enabled=False, v_cap=None, s_target_N=None,
             progress_v_min=0.0, s_anchor=None):
    """
    Stage output h(x), vectorised over stages. Two mutually exclusive
    layouts for row 4 onward, fixed per NMPCController instance (see
    NH_TRACKING/NH_PROGRESS):

    Default (progress_enabled=False), NH_TRACKING=5 rows:
        [e_y, e_y_dot, e_psi, e_psi_dot, v_x - v_ref]
    Row 4 is the original two-sided speed-error residual, v_ref broadcast
    to every stage. e_psi_dot = r - kappa(s)*s_dot is the heading-error
    RATE — see nmpc_params.py on why q_r weights this rather than absolute
    yaw rate.

    Progress mode (progress_enabled=True), NH_PROGRESS=6 rows:
        [e_y, e_y_dot, e_psi, e_psi_dot, hinge(v_x), h_prog]
    Row 4 combines TWO one-sided hinges in one residual (sum, not stacked
    as separate rows -- both are weighted by the same q_e_v, and neither is
    ever simultaneously nonzero since v_cap > progress_v_min always):
        max(0, v_x - v_cap) - max(0, progress_v_min - v_x)
    The first term is the speed CAP: zero penalty below v_cap, quadratic
    above it once squared by the caller's weight. This replaces the
    two-sided residual because a two-sided cost pulls the car UP to v_ref;
    a bare upper hinge only ever pushes it back DOWN, leaving nothing to
    stop the car slowing down except the progress reward below. The second
    term is a LOW-SPEED FLOOR guard, zero above progress_v_min, growing as
    v_x drops below it: defence-in-depth against the standstill trivial
    solution (v_x=0 locally optimal, compounded by the known v_x=0
    tyre-force behaviour in this model) -- Liniger's MPCC reference
    implementation uses a hard Vx>=0.05 bound for the same reason.
    Subtracting rather than adding the second term keeps both hinges in a
    SINGLE signed residual so squaring it still produces a pure
    hinge-quadratic penalty on each side independently, with no cross term
    (the two conditions are mutually exclusive, one is always exactly
    zero). `v_cap` may be a per-stage array or a scalar broadcast like
    v_ref above.

    Row 5 (h_prog) is the progress reward, written as an UNREACHABLE-
    TARGET least-squares residual rather than a bare linear -q_s*s_N: a
    linear term contributes nothing to the Gauss-Newton Hessian (only the
    gradient), which starves the QP of curvature in the one direction
    nothing else opposes and lets a single SQP step bang straight to a
    bound (Zanon, "A Gauss-Newton-Like Hessian Approximation for Economic
    NMPC", IEEE TAC, arXiv:2007.13519; acados documents the identical
    zero-Hessian-block case). `s_target_N` is set by the caller to
    something always out of reach, so minimising (s_target_N - s)^2 is
    monotone-equivalent to maximising s while staying GN-native. h_prog is
    zero at every stage except the terminal one (index M-1 of this call's
    X) -- the reward is a horizon-end goal, not a per-stage summed cost, so
    it cannot reproduce the rejected nmpc_horizon_speed_profile_enabled
    loophole where a later stage's target paid for an earlier stage's
    violation (that mechanism summed a per-stage residual across the WHOLE
    horizon; this one only ever scores the last stage).

    The path-relative terms come from _frenet_stages, so the rows mean the
    same in both state frames. `s_anchor` is required for the Cartesian
    model (see _frenet_stages) and ignored by the Frenet one.

    When `friction_circle_enabled` is True, TWO EXTRA rows (F_yf, F_yr, see
    NH_FRICTION) are appended after whichever layout is active. These ride
    along through the exact same finite-difference Jacobian pass
    _output_jacobians already runs for the cost rows, but are NEVER part of
    the cost themselves (see NMPCController._solve_step's w_out slicing) --
    only used to build the friction-circle QP constraint. Shape is
    IDENTICAL to before either feature existed when both flags are False
    (not just "the extra rows are empty").
    """
    v_x   = X[:, IDX_VX]
    r     = X[:, IDX_R]
    s, e_y, e_psi, e_y_dot, s_dot, kap = _frenet_stages(X, ref, p, s_anchor)
    nh = NH_PROGRESS if progress_enabled else NH_TRACKING
    n_cols = nh + NH_FRICTION if friction_circle_enabled else nh
    H = np.empty((X.shape[0], n_cols))
    H[:, 0] = e_y
    H[:, 1] = e_y_dot
    H[:, 2] = e_psi
    H[:, 3] = r - kap * s_dot
    if progress_enabled:
        H[:, 4] = (np.maximum(0.0, v_x - v_cap)
                   - np.maximum(0.0, progress_v_min - v_x))
        h_prog = np.zeros(X.shape[0])
        h_prog[-1] = s_target_N - s[-1]
        H[:, 5] = h_prog
    else:
        H[:, 4] = v_x - v_ref
    if friction_circle_enabled:
        F_yf, F_yr = _tyre_forces(X, p)
        H[:, nh] = F_yf
        H[:, nh + 1] = F_yr
    return H
