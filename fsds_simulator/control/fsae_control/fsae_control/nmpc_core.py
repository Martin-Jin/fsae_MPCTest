# Title: nmpc_core.py

"""
nmpc_core.py — NONLINEAR model-predictive path-tracking controller
(Frenet-frame / curvilinear coordinates, Gauss-Newton SQP, condensed dense QP
subproblem solved by OSQP).

This is a SECOND, independently selectable controller. It does not modify,
subclass or import behaviour from mpc_core.MPCController's solve path: that
LTV-QP controller remains the default and is completely untouched. Selection
happens at node construction time via NMPCParams.use_nmpc (default False) —
see mpc_controller.py / mpc_controller_standalone.py.

WHY IT EXISTS (the structural gap this closes)
---------------------------------------------
MPCController's prediction model (`_discrete_model`) is the bicycle model in
ERROR coordinates with the reference frame's own rotation dropped. The exact
missing term is one line of the Frenet kinematics:

    e_psi_dot = r - kappa(s) * s_dot        <-- the "- kappa(s)*s_dot" is absent

Consequence, confirmed repeatedly in late_turn_in_investigation.md: with
e_y = e_psi = 0 (car dead on-line, corner ahead) the QP's whole 35-step rollout
predicts staying at 0 forever, so NO cost weighting can produce turn-in before
real error exists. Three separate attempts to bolt the term on as exogenous,
horizon-indexed data — a dynamics disturbance `w[k]` (Part 2), a shifting cost
target (Part 7), a per-step reference array (Part 15) — all produced a
WRONG-DIRECTION transient, because a future obligation known at solve time
lets the solver choose when to pay it, and an early wrong-way dip can integrate
to lower total quadratic cost.

Here `kappa` is not indexed by horizon step: it is `kappa(s)` where `s` is a
STATE driven by the car's own predicted motion. The obligation is therefore not
schedulable — steering the wrong way makes e_psi, and then e_y, grow from the
very next step, and every later step inherits the worse state through the
dynamics. That is the argument; it is checked empirically (synthetic
dead-centre corner approach, both "does it turn in early" and "does it dip the
wrong way first") in late_turn_in_investigation.md Part 16 §16.6, not assumed.

MODEL
-----
    x = [s, e_y, e_psi, v_x, v_y, r, delta_act, a_act]     (nx = 8)
    u = [delta_cmd, a_cmd]                                 (nu = 2)

    s_dot         = (v_x*cos(e_psi) - v_y*sin(e_psi)) / (1 - kappa(s)*e_y)
    e_y_dot       =  v_x*sin(e_psi) + v_y*cos(e_psi)
    e_psi_dot     =  r - kappa(s)*s_dot
    v_x_dot       =  a_act + alpha*r*v_y
    v_y_dot       =  (1-alpha)*v_y_dot_kin + alpha*(F_yf*cos(d) + F_yr)/m - alpha*r*v_x
    r_dot         =  (1-alpha)*r_dot_kin   + alpha*(lf*F_yf*cos(d) - lr*F_yr)/Iz
    delta_act_dot = (delta_cmd - delta_act)/tau_delta
    a_act_dot     = (a_cmd     - a_act    )/tau_a

with linear tyres F_yf = -2*Cf*alpha_f, F_yr = -2*Cr*alpha_r (the same 2*Cf /
2*Cr axle convention `_discrete_model` uses), and the SAME kinematic->dynamic
blend breakpoints MPCController already uses (alpha = clip((v_x-1.0)/1.5, 0, 1)),
whose kinematic branch is r = v_x*tan(delta_act)/(lf+lr), v_y = lr*r,
differentiated to give r_dot_kin / v_y_dot_kin.

EVERY vehicle constant (lf, lr, m, Iz, Cf, Cr, tau_delta, tau_a, MAX_STEER_RAD,
MAX_ACCEL, MAX_BRAKE, du_max) is taken from MPCController unchanged — imported
from mpc_core where module-level, copied with a comment where it is set in
MPCController.__init__. No new physical constant is introduced.

WHERE THE ERRORS ARE MEASURED
-----------------------------
Identical to `MPCController._error_state`: nearest waypoint to the FRONT AXLE
(car_pos + lf*heading), e_y as the perpendicular projection onto that segment
(not Euclidean distance), e_psi wrapped to [-pi, pi], e_y_dot =
v_x*sin(e_psi) + v_y*cos(e_psi). That mixture of a front-axle measurement point
with a CoG-frame e_y_dot is inherited deliberately, not "fixed" here: it is
what vehicle_physics.plant_to_tracking_error / rollout_core.py and every logged
run already use, so e_y / e_psi in an NMPC log mean exactly what they mean in
an LTV-QP log.

COST
----
Gauss-Newton least squares on the stage output

    h(x) = [e_y,  e_y_dot,  e_psi,  (r - kappa(s)*s_dot),  v_x - v_ref]

weighted by (q_e_y, q_e_yd, q_e_psi, q_r, q_e_v) from the SAME MPCParams the
LTV-QP uses, plus input effort (r_delta; r_a_accel/r_a_brake selected per stage
by the sign of the current iterate's a_cmd) and input rate (r_rate_delta,
r_rate_a), plus MPCParams.terminal_q_scale on the final stage and the same soft
+-3.5 m |e_y| slack penalty (W_slack = 10000) `_build_qp` carries. See
nmpc_params.py for which weights can be overridden per-NMPC and for the ONE
weight whose meaning changes (q_r -> heading-error rate, not absolute yaw rate).

WHAT THIS CONTROLLER DELIBERATELY DOES NOT DO
---------------------------------------------
* No adaptive gain schedule. Every mechanism in mpc_core's ~17-multiplier stack
  (_lookahead_approach_boost, _lookahead_epsi_approach_boost,
  _lookahead_yaw_rate_relax, _steer_rate_anti_hunt, ...) exists to synthesise
  anticipation the curvature-blind model could not produce. A curvature-aware
  model anticipates in the prediction itself, so layering those on top would
  double-count an effect that is now structural. They are left available and
  untouched on the LTV-QP path.
* No shaped heading-lead profile. set_heading_profile() is accepted (so the
  nodes need no branch) and IGNORED with a one-time log line: the shaped
  psi_target of Part 8/9 is a workaround for the same missing curvature term.
* No speed profile over the horizon. v_ref is the caller's single, already
  low-passed desired_speed held constant across the horizon — the same
  information the LTV-QP gets. Part 11's braking-lag problem is NOT addressed
  here on purpose; changing the longitudinal reference at the same time as the
  lateral model would make a live A/B unreadable.

REAL-TIME STRUCTURE
-------------------
Per tick: warm-start the input trajectory from the previous tick (shifted one
step), then up to NMPCParams.nmpc_sqp_iters Gauss-Newton iterations, each:
(1) roll the nonlinear model forward from the measured state under the current
input guess — so the linearisation point is always dynamically FEASIBLE and the
QP's dynamics defect is exactly zero; (2) finite-difference the one-step
Jacobians A_k/B_k and the output Jacobians C_k, vectorised across all horizon
stages at once; (3) condense to a dense QP in the input deviations only
(nu*N + N variables); (4) solve with OSQP, warm-started, fixed sparsity so only
the data arrays are updated; (5) take the step with a box trust region and
optional backtracking if the true nonlinear cost got worse. A wall-clock budget
(nmpc_solve_budget_ms) stops the loop early and ships the best iterate rather
than overrunning the 50 ms tick.

Jacobians are finite-differenced rather than hand-derived on purpose: a model
change cannot silently desynchronise from its derivative. Accuracy is verified
against complex-step/central differences and against CasADi+IPOPT offline —
see late_turn_in_investigation.md Part 16 §16.5/§16.7 for both the check and
the measured solve times.
"""

import math
import time
from dataclasses import dataclass

import numpy as np
import scipy.sparse as sp

try:
    import osqp
except ImportError as _exc:      # pragma: no cover - see package.xml
    osqp = None
    _OSQP_IMPORT_ERROR = _exc

from fsae_control.mpc_core import MAX_ACCEL, MAX_BRAKE, MAX_STEER_RAD
from fsae_control.mpc_params import DEFAULT_MPC_PARAMS, MPCParams
from fsae_control.nmpc_params import DEFAULT_NMPC_PARAMS, NMPCParams

# ── State/input/output layout ────────────────────────────────────────────
IDX_S     = 0   # arc length along the reference path (m)
IDX_EY    = 1   # lateral deviation, + = front axle LEFT of the path (m)
IDX_EPSI  = 2   # heading error, car_yaw - path_yaw, wrapped (rad)
IDX_VX    = 3   # body-frame forward speed (m/s)
IDX_VY    = 4   # body-frame lateral speed (m/s)
IDX_R     = 5   # yaw rate (rad/s)
IDX_DELTA = 6   # actuator-lagged steering angle (rad)
IDX_A     = 7   # actuator-lagged acceleration (m/s^2)
NX = 8
NU = 2
NH = 5          # h() rows: e_y, e_y_dot, e_psi, e_psi_dot, e_v

# Finite-difference perturbations, one per state then per input. Sized per
# variable so every column of the Jacobian has a comparable truncation/roundoff
# balance: eps_j ~ 1e-6 * (typical magnitude of that variable). Verified
# against central differences in Part 16 §16.5.
_FD_EPS_X = np.array([1e-6, 1e-6, 1e-7, 1e-6, 1e-6, 1e-7, 1e-7, 1e-6])
_FD_EPS_U = np.array([1e-7, 1e-6])

# Guard on the Frenet denominator (1 - kappa*e_y). It is singular at
# e_y = 1/kappa; on this car that is 4.8 m at the tightest logged corner
# (kappa 0.21) against a 3.5 m track half-width, so this floor is inert in
# normal operation and only prevents a sign flip if the car is somehow far
# off-track on a very tight bend. Not a tunable.
_DENOM_FLOOR = 0.25


def _wrap(a):
    """Wrap an angle (or array of angles) to [-pi, pi]."""
    return np.arctan2(np.sin(a), np.cos(a))


class PathReference:
    """
    Arc-length parameterisation of a waypoint path, plus the smoothed
    curvature profile kappa(s) the NMPC's prediction needs.

    Built once per DISTINCT path: for a precomputed/static path that is once
    per run (see NMPCController.set_static_path / the per-tick signature cache
    in compute()); in live-planner mode the path changes every tick and this is
    rebuilt each time (measured cost in Part 16 §16.7).

    kappa(s) is built by resampling at `dense_step` metres, moving-averaging
    the resampled x/y with a width-`smooth_w` kernel, then differencing
    consecutive headings — the SAME denoise precedent
    control_utils.curvature_speed() already uses (dense_step 0.5 m, w 3), for
    the same reason: the planner's refit centreline carries centimetre-scale
    lateral wiggle that a raw three-point curvature turns into a spurious bend.
    Here that matters more than it does for a speed cap, because this kappa
    enters the PREDICTION and would be steered for.
    """

    def __init__(self, path, dense_step=0.5, smooth_w=3, kappa_clip=0.5):
        path = np.asarray(path, dtype=float)
        self.path = path
        seg = np.diff(path, axis=0)
        seg_len = np.hypot(seg[:, 0], seg[:, 1])
        self.arc = np.concatenate([[0.0], np.cumsum(seg_len)])
        self.total = float(self.arc[-1]) if len(self.arc) else 0.0

        # Dense resample + smooth + heading difference -> kappa(s).
        s_k = np.array([0.0, max(self.total, 1e-3)])
        kappa = np.zeros(2)
        psi_ref = None
        if self.total > 4.0 * max(dense_step, 1e-6):
            dense = np.arange(0.0, self.total, dense_step)
            dx = np.interp(dense, self.arc, path[:, 0])
            dy = np.interp(dense, self.arc, path[:, 1])
            w = int(max(1, min(smooth_w, max(1, len(dense) - 4))))
            if w > 1:
                ker = np.ones(w) / w
                sx = np.convolve(dx, ker, mode='valid')
                sy = np.convolve(dy, ker, mode='valid')
                s0 = (w - 1) / 2.0 * dense_step
            else:
                sx, sy, s0 = dx, dy, 0.0
            if len(sx) >= 3:
                d_x = np.diff(sx)
                d_y = np.diff(sy)
                ds = np.hypot(d_x, d_y)
                psi = np.arctan2(d_y, d_x)
                dpsi = _wrap(np.diff(psi))
                ds_mid = 0.5 * (ds[:-1] + ds[1:])
                good = ds_mid > 1e-6
                k = np.zeros_like(ds_mid)
                k[good] = dpsi[good] / ds_mid[good]
                # kappa[i] is centred on the smoothed sample i+1.
                s_k = s0 + dense_step * (np.arange(len(k)) + 1.0)
                kappa = np.clip(k, -kappa_clip, kappa_clip)
                # Reference HEADING on the same grid and from the same
                # smoothed samples the curvature came from, unwrapped so
                # interpolation across the +-pi seam is well defined.
                #
                # This matters as much as the curvature itself: the reference
                # heading and the reference curvature must describe ONE
                # reference, or the measured e_psi and the model's own
                # e_psi_dot = r - kappa*s_dot disagree. Measuring e_psi off the
                # RAW segment tangent (as the LTV-QP does) quantises it in
                # steps of ds/R — 5.7 deg per 0.5 m waypoint on a 5 m-radius
                # hairpin — and the NMPC reads each of those steps as a real
                # state error to be corrected within a tick or two. Offline
                # that produced a hard period-2 steering limit cycle
                # (+25 deg / -25 deg alternating) through the tight corners on
                # comp_test_map_3; see late_turn_in_investigation.md Part 16
                # §16.6. The LTV-QP does not show it because its own
                # anti-hunt/R_rate machinery damps exactly this, and because it
                # never predicts heading forward at all.
                psi_mid = np.unwrap(psi)
                psi_ref = 0.5 * (psi_mid[:-1] + psi_mid[1:])

        self.s_kappa = s_k
        self.kappa = kappa
        # Scalar-lookup fast path for the sequential rollout (see
        # kappa_scalar): s_kappa is a uniform grid by construction, so the
        # index is arithmetic and a Python list beats numpy indexing at this
        # size. _k_uniform is False only for the degenerate short-path
        # fallback above (all-zero curvature), where kappa_scalar falls back
        # to np.interp.
        self._k_list = [float(v) for v in np.atleast_1d(kappa)]
        self._k_n = len(self._k_list)
        self._k_s0 = float(s_k[0])
        self._k_ds = float(dense_step)
        self._k_uniform = self._k_n >= 3

        # Reference heading psi_ref(s) on the SAME grid as kappa (see above).
        # When the path was too short to smooth, fall back to the raw
        # per-waypoint tangent so this is always populated.
        if psi_ref is None:
            d = np.diff(path, axis=0)
            raw = np.unwrap(np.arctan2(d[:, 1], d[:, 0]))
            self.s_psi = 0.5 * (self.arc[:-1] + self.arc[1:])
            self.psi_ref = raw
        else:
            self.s_psi = s_k
            self.psi_ref = psi_ref
        # Signature used to decide whether a cached PathReference still
        # describes the array compute() was handed this tick. Cheap (no
        # full-array compare) and sufficient: the planner republishes a new
        # array object with different endpoints/length when the path changes.
        self.signature = (
            len(path),
            float(path[0, 0]), float(path[0, 1]),
            float(path[-1, 0]), float(path[-1, 1]),
            round(self.total, 6),
        )

    def kappa_at(self, s):
        """
        Signed curvature (1/m) at arc length(s) `s`, clamped to the path's own
        extent at both ends (np.interp's default edge behaviour) rather than
        returning 0 past the end: holding the last known curvature is the
        conservative choice for a horizon that runs off the end of a partial
        planner path, where dropping to 0 would predict the corner simply
        stopping.
        """
        return np.interp(s, self.s_kappa, self.kappa)

    def kappa_scalar(self, s):
        """
        Single-point kappa(s), O(1) on the uniform curvature grid. Same values
        (and same end-clamping) as kappa_at, without np.interp's per-call
        overhead — this is called ~1100 times per rollout, so that overhead is
        the difference between a 1 ms and a 5 ms rollout.
        """
        if not self._k_uniform:
            return float(np.interp(s, self.s_kappa, self.kappa))
        t = (s - self._k_s0) / self._k_ds
        if t <= 0.0:
            return self._k_list[0]
        i = int(t)
        if i >= self._k_n - 1:
            return self._k_list[-1]
        k0 = self._k_list[i]
        return k0 + (t - i) * (self._k_list[i + 1] - k0)

    def psi_ref_at(self, s):
        """
        Reference heading (rad, unwrapped and hence continuous) at arc
        length(s) `s`, from the same smoothed samples kappa comes from. Used
        for BOTH the e_y projection direction and e_psi, so the measured
        Frenet state and the predicted one describe one identical reference.
        """
        return np.interp(s, self.s_psi, self.psi_ref)

    def project(self, front_axle, car_yaw):
        """
        Frenet projection of a front-axle position onto the path.

        Returns (s0, e_y, e_psi, base_idx, path_yaw). Deliberately identical
        arithmetic to MPCController._error_state (nearest waypoint, segment
        tangent, perpendicular projection, wrapped heading error) so e_y/e_psi
        are directly comparable between the two controllers' logs.
        """
        path = self.path
        base_idx = int(np.argmin(np.linalg.norm(path - front_axle, axis=1)))
        s_base = float(self.arc[base_idx])
        # Reference direction from the SMOOTHED profile, not the raw segment
        # tangent — see psi_ref_at / the psi_ref construction comment.
        path_yaw = float(self.psi_ref_at(s_base))
        dx = front_axle[0] - path[base_idx][0]
        dy = front_axle[1] - path[base_idx][1]
        cos_y, sin_y = math.cos(path_yaw), math.sin(path_yaw)
        e_y = dy * cos_y - dx * sin_y
        along = dx * cos_y + dy * sin_y
        s0 = s_base + along
        # Re-evaluate the heading at the refined station: on a tight corner the
        # along-track correction can be a metre or more, over which the
        # reference heading genuinely changes.
        path_yaw = float(self.psi_ref_at(s0))
        e_psi = float(_wrap(car_yaw - path_yaw))
        return s0, float(e_y), e_psi, base_idx, path_yaw


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
    # MPCParams.alat_ceiling_flat/_slope/_intercept (the SAME law
    # mpc_core._alat_ceiling_at and model/vehicle_physics.alat_ceiling_at
    # already use — measured by open-loop system-ID, see CLAUDE.md's
    # "MECHANISM: a dynamically-enforced lateral-acceleration ceiling").
    # NMPCController.__init__ overwrites these from its MPCParams instance.
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
    two identical. The horizon rollout is inherently sequential (35 steps x 4
    RK stages x n_sub substeps), and at that size numpy's per-call overhead
    dominates completely: the vectorised _f costs ~75 us on a 1x8 array, making
    one rollout 17 ms, versus ~1 ms for this form. The Jacobian pass, which
    batches all stages into one array, still uses the vectorised _f.

    `kap` is passed in (already looked up) rather than read from `ref` so the
    caller can use PathReference.kappa_scalar's O(1) uniform-grid lookup
    instead of np.interp's ~4 us call overhead.

    test_nmpc_core.py::test_scalar_matches_vectorised asserts the two forms
    agree to 1e-12 on randomised states — a divergence between them is a
    silent-wrong-prediction bug, so that test is not optional.
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


def _outputs(X, ref, p, v_ref):
    """
    Stage output h(x) = [e_y, e_y_dot, e_psi, e_psi_dot, v_x - v_ref],
    vectorised over stages. e_psi_dot = r - kappa(s)*s_dot is the
    heading-error RATE — see nmpc_params.py on why q_r weights this rather
    than absolute yaw rate.
    """
    e_y   = X[:, IDX_EY]
    e_psi = X[:, IDX_EPSI]
    v_x   = X[:, IDX_VX]
    v_y   = X[:, IDX_VY]
    r     = X[:, IDX_R]
    kap = ref.kappa_at(X[:, IDX_S])
    denom = 1.0 - kap * e_y
    denom = np.where(denom >= 0.0,
                     np.maximum(denom, _DENOM_FLOOR),
                     np.minimum(denom, -_DENOM_FLOOR))
    cos_ep = np.cos(e_psi)
    sin_ep = np.sin(e_psi)
    s_dot = (v_x * cos_ep - v_y * sin_ep) / denom
    H = np.empty((X.shape[0], NH))
    H[:, 0] = e_y
    H[:, 1] = v_x * sin_ep + v_y * cos_ep
    H[:, 2] = e_psi
    H[:, 3] = r - kap * s_dot
    H[:, 4] = v_x - v_ref
    return H


def _csc_pattern(mask):
    """
    Build a CSC matrix with an explicit, fixed sparsity pattern from a boolean
    mask, plus the (row, col) index arrays that write into its .data in CSC
    order.

    OSQP requires every update() to keep the pattern it was set up with, so the
    pattern is fixed once and only the data array is refilled per solve. Going
    through a mask (rather than converting a dense value array) is what
    guarantees a structural zero stays structurally present instead of being
    dropped by scipy.
    """
    m = sp.csc_matrix(mask.astype(np.float64))
    rows = m.indices.copy()
    cols = np.zeros_like(rows)
    for j in range(m.shape[1]):
        cols[m.indptr[j]:m.indptr[j + 1]] = j
    return m, rows, cols


class NMPCController:
    """
    Frenet-frame nonlinear MPC, drop-in compatible with
    mpc_core.MPCController's node-facing surface: compute(), reset(),
    set_static_path(), set_heading_profile(), last_telemetry, a_max,
    a_max_brake.
    """

    def __init__(
        self,
        dt: float = 0.05,
        N: int | None = None,
        params: MPCParams | None = None,
        nmpc: NMPCParams | None = None,
        logger=None,
    ) -> None:
        """
        dt must equal the calling node's control period (0.05 s), as for
        MPCController. N defaults to NMPCParams.nmpc_horizon (35, matching
        MPCController's own horizon so horizon length is never a confound when
        comparing the two); an explicit N overrides it. `logger` is an optional
        rclpy logger for the one-time informational messages; print() is used
        when it is None so this module stays importable/testable outside ROS.
        """
        if osqp is None:      # pragma: no cover - dependency guard
            raise ImportError(
                'nmpc_core requires osqp (already a documented dependency of '
                f'mpc_core via cvxpy — see package.xml): {_OSQP_IMPORT_ERROR!r}'
            )

        self.dt = float(dt)
        self.params = params if params is not None else DEFAULT_MPC_PARAMS
        self.nmpc = nmpc if nmpc is not None else DEFAULT_NMPC_PARAMS
        self._logger = logger
        self.N = int(self.nmpc.nmpc_horizon if N is None else N)
        if self.N < 2:
            raise ValueError(f'NMPC horizon must be >= 2 (got {self.N})')

        self.nx, self.nu = NX, NU
        nm0 = nmpc if nmpc is not None else DEFAULT_NMPC_PARAMS
        # alat_ceiling_flat/_slope/_intercept are FSDS's measured plant
        # constant (the sustained lateral-accel ceiling law), not a tuning
        # weight -- MPCParams no longer carries them (removed along with the
        # whole corner_demand/lookahead-gain-scheduling family they used to
        # parameterise), so _Plant's own hardcoded defaults (7.5/0.47/2.46,
        # the same measured values) are used directly here, the same
        # precedent this class already follows for lf/lr/m/Iz/Cf/Cr. Only
        # nmpc_alat_ceiling_enabled (a genuine NMPC-only on/off switch, not a
        # shared plant constant) still comes from NMPCParams.
        self.plant = _Plant(alat_ceiling_enabled=bool(nm0.nmpc_alat_ceiling_enabled))
        # Convenience aliases so telemetry/geometry code reads like mpc_core's.
        self.lf, self.lr = self.plant.lf, self.plant.lr

        # ── Limits: identical to MPCController's ────────────────────────
        self.a_max = MAX_ACCEL
        self.a_max_brake = MAX_BRAKE
        self.u_min = np.array([-MAX_STEER_RAD, -self.a_max_brake])
        self.u_max = np.array([MAX_STEER_RAD, self.a_max])
        # 180 deg/s * dt, matching MPCController's du_max exactly (see that
        # constructor's long note on why it is 180 and not 190).
        self.du_max = np.array([math.radians(180.0) * self.dt, 0.6])

        # ── Weights: MPCParams, with per-NMPC overrides ─────────────────
        # The override fields (nmpc_q_e_y, ...) live in MPCParams itself
        # (moved there 2026-08-13 from NMPCParams — see mpc_params.py's
        # "NMPC weight overrides" section for why), alongside the base
        # fields they inherit from when left at -1.0. NMPCParams (self.nmpc)
        # no longer carries any weight field at all — only structural/
        # solver settings.
        def _pick(override, inherited):
            return float(inherited if override is None or override < 0.0 else override)

        pm = self.params
        self.w_out = np.array([
            _pick(pm.nmpc_q_e_y,      pm.q_e_y),
            _pick(pm.nmpc_q_e_yd,     pm.q_e_yd),
            _pick(pm.nmpc_q_e_psi,    pm.q_e_psi),
            _pick(pm.nmpc_q_epsi_dot, pm.q_r),
            _pick(pm.nmpc_q_e_v,      pm.q_e_v),
        ])
        self.r_delta = _pick(pm.nmpc_r_delta, pm.r_delta)
        self.r_a_accel = _pick(pm.nmpc_r_a_accel, pm.r_a_accel)
        self.r_a_brake = _pick(pm.nmpc_r_a_brake, pm.r_a_brake)
        self.r_rate = np.array([
            _pick(pm.nmpc_r_rate_delta, pm.r_rate_delta),
            _pick(pm.nmpc_r_rate_a, pm.r_rate_a),
        ])
        self.terminal_scale = _pick(pm.nmpc_terminal_scale, pm.terminal_q_scale)

        # ── Continuity memory (mirrors MPCController's) ─────────────────
        self._delta_act = 0.0
        self._a_act = 0.0
        self._u_prev = np.zeros(NU)
        self._v_des_filtered: float | None = None
        self._U = np.zeros((self.N, NU))     # warm-start input trajectory
        self._have_warm_start = False
        self._u_history: list[np.ndarray] = []
        self._pose_age_filtered: float | None = None
        self._n_delay = 0

        self._ref: PathReference | None = None
        self._ref_signature = None
        self._static_ref: PathReference | None = None
        self._heading_profile_warned = False

        self.last_telemetry: dict = {}
        self._qp = None
        self._build_qp()

    # ------------------------------------------------------------------
    # Node-facing hooks (same names/semantics as MPCController's)
    # ------------------------------------------------------------------
    def _log(self, msg: str) -> None:
        if self._logger is not None:      # pragma: no cover - ROS path
            self._logger.info(msg)
        else:
            print(f'[nmpc_core] {msg}')

    def set_static_path(self, path) -> None:
        """
        Precompute the arc-length/curvature reference for a fixed path, once,
        at load time. Called by the owning node exactly where it calls
        MPCController.set_static_path(); compute() never builds this itself for
        a static path (it looks the cached one up by signature).

        path=None clears it (live-planner mode), in which case compute() builds
        a reference per tick from whatever path it is handed.
        """
        if path is None or len(np.asarray(path)) < 3:
            self._static_ref = None
            return
        try:
            self._static_ref = PathReference(
                path,
                dense_step=self.nmpc.nmpc_curvature_dense_step,
                smooth_w=self.nmpc.nmpc_curvature_smooth_w,
                kappa_clip=self.nmpc.nmpc_kappa_clip,
            )
            k = self._static_ref.kappa
            self._log(
                f'NMPC path reference built: {self._static_ref.total:.1f} m, '
                f'{len(k)} curvature samples, |kappa| max {np.abs(k).max():.4f} 1/m.'
            )
        except (ValueError, IndexError) as exc:   # pragma: no cover - defensive
            self._log(f'PathReference build failed ({exc}); will rebuild per tick.')
            self._static_ref = None

    def set_heading_profile(self, psi_target) -> None:
        """
        Accepted and IGNORED — see the module docstring. The shaped
        heading-lead profile (Part 8/9) is a workaround for the missing
        curvature term this controller models directly, so applying both would
        double-count the anticipation. Logged once so a launch config that
        enables use_precomputed_heading_profile alongside use_nmpc does not
        silently do nothing.
        """
        if psi_target is not None and not self._heading_profile_warned:
            self._heading_profile_warned = True
            self._log(
                'use_precomputed_heading_profile is IGNORED by the NMPC: its '
                'Frenet model already carries the path curvature the shaped '
                'heading lead exists to approximate.'
            )

    def reset(self) -> None:
        """Clear continuity state, as MPCController.reset() does."""
        self._delta_act = 0.0
        self._a_act = 0.0
        self._u_prev = np.zeros(NU)
        self._v_des_filtered = None
        self._U = np.zeros((self.N, NU))
        self._have_warm_start = False
        self._u_history.clear()
        self._pose_age_filtered = None
        self._n_delay = 0

    # ------------------------------------------------------------------
    # QP subproblem (condensed, dense, fixed sparsity)
    # ------------------------------------------------------------------
    def _build_qp(self) -> None:
        """
        Allocate the condensed QP once: variables z = [dU (nu*N); slack (N)],
        with the constraint rows

            (1) box/trust region on dU                       nu*N rows
            (2) input slew rate |u_k - u_{k-1}| <= du_max     nu*N rows
            (3) e_y_k - slack_k <= +halfwidth                 N rows
            (4) e_y_k + slack_k >= -halfwidth                 N rows
            (5) slack >= 0                                    N rows

        Rows (3)-(5) and the slack variables are omitted entirely when
        nmpc_track_halfwidth <= 0. The pattern never changes after this, so
        each solve only rewrites P.data / A.data / q / l / u.
        """
        N = self.N
        n_du = NU * N
        self._use_slack = self.nmpc.nmpc_track_halfwidth > 0.0
        n_slack = N if self._use_slack else 0
        nz = n_du + n_slack

        # First-difference operator E: diff_k = u_k - u_{k-1} (u_{-1} = u_prev).
        E = np.zeros((n_du, n_du))
        for k in range(N):
            E[k * NU:(k + 1) * NU, k * NU:(k + 1) * NU] = np.eye(NU)
            if k > 0:
                E[k * NU:(k + 1) * NU, (k - 1) * NU:k * NU] = -np.eye(NU)
        self._E = E
        self._Rr_flat = np.tile(self.r_rate, N)
        self._ErE = E.T @ (self._Rr_flat[:, None] * E)

        # P pattern: dense upper triangle over dU, plus the slack diagonal.
        p_mask = np.zeros((nz, nz), dtype=bool)
        p_mask[:n_du, :n_du] = np.triu(np.ones((n_du, n_du), dtype=bool))
        if n_slack:
            idx = np.arange(n_du, nz)
            p_mask[idx, idx] = True
        P, p_rows, p_cols = _csc_pattern(p_mask)

        # A pattern.
        n_rows = 2 * n_du + (3 * N if self._use_slack else 0)
        a_mask = np.zeros((n_rows, nz), dtype=bool)
        a_mask[:n_du, :n_du] = np.eye(n_du, dtype=bool)
        a_mask[n_du:2 * n_du, :n_du] = E != 0.0
        if self._use_slack:
            r0 = 2 * n_du
            # Track rows are structurally dense in dU: stage k's e_y depends on
            # every earlier input, and marking the (structurally zero) later
            # columns present too keeps this pattern trivially fixed.
            a_mask[r0:r0 + 2 * N, :n_du] = True
            for k in range(N):
                a_mask[r0 + k, n_du + k] = True                 # -slack_k
                a_mask[r0 + N + k, n_du + k] = True              # +slack_k
                a_mask[r0 + 2 * N + k, n_du + k] = True          # slack_k >= 0
        A, a_rows, a_cols = _csc_pattern(a_mask)

        q = np.zeros(nz)
        l = np.full(n_rows, -np.inf)
        u = np.full(n_rows, np.inf)

        prob = osqp.OSQP()
        settings = dict(
            verbose=False,
            eps_abs=self.nmpc.nmpc_osqp_eps,
            eps_rel=self.nmpc.nmpc_osqp_eps,
            max_iter=int(self.nmpc.nmpc_osqp_max_iter),
        )
        try:
            prob.setup(P, q, A, l, u, warm_starting=True, polishing=False,
                       **settings)
        except TypeError:      # pragma: no cover - osqp < 1.0 naming
            prob.setup(P, q, A, l, u, warm_start=True, polish=False, **settings)

        self._qp = dict(
            prob=prob, P=P, A=A,
            p_rows=p_rows, p_cols=p_cols,
            a_rows=a_rows, a_cols=a_cols,
            n_du=n_du, n_slack=n_slack, nz=nz, n_rows=n_rows,
        )

    # ------------------------------------------------------------------
    # Prediction / linearisation
    # ------------------------------------------------------------------
    def _rollout(self, x0, U, ref):
        """
        Nonlinear forward simulation of the whole horizon from x0 under U.
        Returns X with shape (N+1, NX). Sequential by construction (each stage
        depends on the previous one), so this is the one part of a Gauss-Newton
        iteration that cannot be vectorised across stages — hence the scalar
        _step_scalar fast path (see its docstring: 1 ms here versus 17 ms
        through the vectorised form).
        """
        N = self.N
        X = np.empty((N + 1, NX))
        X[0] = x0
        xk = [float(v) for v in x0]
        p, dt, n_sub = self.plant, self.dt, self.nmpc.nmpc_rk_substeps
        for k in range(N):
            xk = _step_scalar(xk, U[k], ref, p, dt, n_sub)
            X[k + 1] = xk
        return X

    def _project_feasible(self, U):
        """
        Project an input trajectory onto the set the QP's own constraints
        describe: input bounds, and the per-step slew limit measured from
        self._u_prev forward.

        This is what makes the subproblem UNCONDITIONALLY FEASIBLE, and that
        matters more than it looks. The QP's slew rows are
        `-du_max - e <= E dU <= du_max - e` with `e` the current iterate's own
        differences, so dU = 0 is feasible if and only if the iterate already
        respects the slew limit. A warm start that does not (shifting the
        previous solution and clipping it to the input bounds can produce one)
        can make the whole subproblem primal-infeasible — after which OSQP
        returns a finite but meaningless x. Projecting first removes that
        failure mode by construction rather than trying to detect it.
        """
        Up = np.clip(np.asarray(U, dtype=float), self.u_min, self.u_max)
        prev = self._u_prev
        for k in range(Up.shape[0]):
            Up[k] = np.clip(Up[k], prev - self.du_max, prev + self.du_max)
            prev = Up[k]
        return Up

    def _jacobians(self, X, U, ref):
        """
        One-step Jacobians A_k = d x_{k+1}/d x_k and B_k = d x_{k+1}/d u_k for
        every stage, by forward finite differences — vectorised over stages, so
        each perturbation direction costs ONE batched one-step integration of
        all N stages rather than N scalar ones (10 batched steps total).

        These use nmpc_jac_substeps (default 1) rather than the rollout's
        nmpc_rk_substeps (default 2), which halves the cost of the dominant
        term in a Gauss-Newton iteration. That is a deliberate, safe
        asymmetry: A_k/B_k only supply the QP's STEP DIRECTION, they never
        define the predicted trajectory (the rollout does, and it is exact to
        RK4 x n_sub). A slightly coarser sensitivity costs at most a slightly
        worse step, which the next iteration and the trust region absorb; the
        prediction itself is unaffected.
        """
        N = self.N
        Xs = X[:N]
        p, dt = self.plant, self.dt
        n_sub = max(1, int(self.nmpc.nmpc_jac_substeps))
        F0 = _step(Xs, U, ref, p, dt, n_sub)
        A = np.empty((N, NX, NX))
        B = np.empty((N, NX, NU))
        for j in range(NX):
            Xp = Xs.copy()
            Xp[:, j] += _FD_EPS_X[j]
            A[:, :, j] = (_step(Xp, U, ref, p, dt, n_sub) - F0) / _FD_EPS_X[j]
        for j in range(NU):
            Up = U.copy()
            Up[:, j] += _FD_EPS_U[j]
            B[:, :, j] = (_step(Xs, Up, ref, p, dt, n_sub) - F0) / _FD_EPS_U[j]
        return A, B

    def _output_jacobians(self, X, ref, v_ref):
        """
        Output Jacobians C_k = d h/d x at every stage (including the terminal
        one), same vectorised forward-difference scheme as _jacobians.
        """
        H0 = _outputs(X, ref, self.plant, v_ref)
        C = np.empty((X.shape[0], NH, NX))
        for j in range(NX):
            Xp = X.copy()
            Xp[:, j] += _FD_EPS_X[j]
            C[:, :, j] = (_outputs(Xp, ref, self.plant, v_ref) - H0) / _FD_EPS_X[j]
        return H0, C

    def _cost(self, X, U, H):
        """
        True (nonlinear) cost of a candidate trajectory — used only by the
        backtracking test, so it must match the QP's objective term for term:
        weighted output residuals with the terminal scale, input effort with
        the accel/brake split, input rate against u_prev, and the soft-track
        slack penalty at its optimal value for this trajectory (max(0, |e_y| -
        halfwidth), which is what the QP's slack would be).
        """
        w = self.w_out
        stage = float(np.sum(w * H[:-1] ** 2)) + float(
            self.terminal_scale * np.sum(w * H[-1] ** 2))
        a = U[:, 1]
        eff = float(self.r_delta * np.sum(U[:, 0] ** 2)
                    + self.r_a_accel * np.sum(np.maximum(a, 0.0) ** 2)
                    + self.r_a_brake * np.sum(np.minimum(a, 0.0) ** 2))
        du = np.vstack([U[0] - self._u_prev, np.diff(U, axis=0)])
        rate = float(np.sum(self.r_rate * du ** 2))
        slack = 0.0
        if self._use_slack:
            over = np.maximum(np.abs(X[1:, IDX_EY]) - self.nmpc.nmpc_track_halfwidth,
                              0.0)
            slack = float(self.nmpc.nmpc_slack_weight * np.sum(over ** 2))
        return stage + eff + rate + slack

    def _solve_step(self, X, U, ref, v_ref):
        """
        One Gauss-Newton SQP iteration: condense, solve the QP, return the
        input-deviation trajectory dU (N, NU) and the OSQP status string.

        Because X was produced by rolling the nonlinear model forward from the
        measured state under U, the linearised dynamics have ZERO defect, so
        the condensed sensitivities alone describe the subproblem exactly:
        dx_k = sum_{j<k} Phi_{k,j} du_j.
        """
        N = self.N
        qp = self._qp
        n_du, n_slack, nz, n_rows = qp['n_du'], qp['n_slack'], qp['nz'], qp['n_rows']

        A_k, B_k = self._jacobians(X, U, ref)
        H, C = self._output_jacobians(X, ref, v_ref)

        # Condensing: S[k] = d x_k / d U_flat, (NX, n_du).
        S = np.zeros((N + 1, NX, n_du))
        for k in range(N):
            S[k + 1] = A_k[k] @ S[k]
            S[k + 1][:, k * NU:(k + 1) * NU] += B_k[k]

        # Weighted output sensitivities: G = sqrt(W) C S, stacked over stages,
        # with the terminal stage scaled by sqrt(terminal_scale).
        sw = np.sqrt(self.w_out)
        scale = np.ones(N + 1)
        scale[N] = math.sqrt(max(self.terminal_scale, 0.0))
        WC = (sw[None, :, None] * C) * scale[:, None, None]
        G = np.einsum('kij,kjl->kil', WC, S).reshape((N + 1) * NH, n_du)
        g = ((sw[None, :] * H) * scale[:, None]).reshape(-1)

        # Input effort: accel/brake weight chosen per stage by the sign of the
        # current iterate's a_cmd (the linearisation point) — the SQP analogue
        # of _build_qp's cp.pos/cp.neg split, which needs no extra variables
        # because the sign is known at linearisation time.
        ru = np.empty((N, NU))
        ru[:, 0] = self.r_delta
        ru[:, 1] = np.where(U[:, 1] >= 0.0, self.r_a_accel, self.r_a_brake)
        ru_flat = ru.reshape(-1)
        u_flat = U.reshape(-1)

        # Input rate: diff = E dU + e, e = E u_flat - [u_prev, 0, ...].
        e_rate = self._E @ u_flat
        e_rate[:NU] -= self._u_prev

        Hess = G.T @ G + np.diag(ru_flat) + self._ErE
        grad = G.T @ g + ru_flat * u_flat + self._E.T @ (self._Rr_flat * e_rate)

        P_dense = np.zeros((nz, nz))
        P_dense[:n_du, :n_du] = 2.0 * Hess
        q = np.zeros(nz)
        q[:n_du] = 2.0 * grad
        if n_slack:
            idx = np.arange(n_du, nz)
            P_dense[idx, idx] = 2.0 * self.nmpc.nmpc_slack_weight

        A_dense = np.zeros((n_rows, nz))
        l = np.empty(n_rows)
        u = np.empty(n_rows)

        # (1) box + trust region on dU.
        A_dense[:n_du, :n_du] = np.eye(n_du)
        tr = np.tile(np.array([self.nmpc.nmpc_trust_delta_rad,
                               self.nmpc.nmpc_trust_a]), N)
        lo = np.maximum(np.tile(self.u_min, N) - u_flat, -tr)
        hi = np.minimum(np.tile(self.u_max, N) - u_flat, tr)
        # A trust region tighter than the distance to a violated bound would
        # make the box infeasible; keep lo <= hi in that (transient) case.
        lo = np.minimum(lo, hi)
        l[:n_du], u[:n_du] = lo, hi

        # (2) slew rate.
        A_dense[n_du:2 * n_du, :n_du] = self._E
        du_flat = np.tile(self.du_max, N)
        l[n_du:2 * n_du] = -du_flat - e_rate
        u[n_du:2 * n_du] = du_flat - e_rate

        # (3)-(5) soft track bound with slack.
        if n_slack:
            r0 = 2 * n_du
            hw = self.nmpc.nmpc_track_halfwidth
            S_ey = S[1:, IDX_EY, :]              # (N, n_du)
            ey = X[1:, IDX_EY]
            A_dense[r0:r0 + N, :n_du] = S_ey
            A_dense[r0:r0 + N, n_du:] = -np.eye(N)
            l[r0:r0 + N] = -np.inf
            u[r0:r0 + N] = hw - ey
            A_dense[r0 + N:r0 + 2 * N, :n_du] = S_ey
            A_dense[r0 + N:r0 + 2 * N, n_du:] = np.eye(N)
            l[r0 + N:r0 + 2 * N] = -hw - ey
            u[r0 + N:r0 + 2 * N] = np.inf
            A_dense[r0 + 2 * N:r0 + 3 * N, n_du:] = np.eye(N)
            l[r0 + 2 * N:r0 + 3 * N] = 0.0
            u[r0 + 2 * N:r0 + 3 * N] = np.inf

        qp['prob'].update(
            Px=P_dense[qp['p_rows'], qp['p_cols']],
            Ax=A_dense[qp['a_rows'], qp['a_cols']],
            q=q, l=l, u=u,
        )
        res = qp['prob'].solve()
        status = str(res.info.status).lower()
        # Status MUST be checked, not just finiteness: on a primal-infeasible
        # or otherwise failed subproblem OSQP returns a finite but MEANINGLESS
        # x, and taking it as a step direction is exactly how this controller
        # first diverged in offline testing (a wrong-way full-lock ramp built
        # up over ~20 ticks, each moving the full slew limit in the garbage
        # direction — see late_turn_in_investigation.md Part 16 §16.6).
        # 'solved inaccurate' / 'maximum iterations reached' ARE accepted:
        # they are still descent directions in practice, and the caller
        # validates every step against the true nonlinear cost before keeping
        # it, so a poor direction costs one wasted iteration, never a bad
        # command.
        ok = ('solved' in status) or ('maximum iterations' in status)
        if not ok or res.x is None or not np.all(np.isfinite(res.x[:n_du])):
            return None, status
        return res.x[:n_du].reshape(N, NU), status

    # ------------------------------------------------------------------
    # Delay compensation
    # ------------------------------------------------------------------
    def _update_n_delay(self, pose_age_s: float) -> int:
        """
        Filtered, hysteresis-gated rollforward depth from a measured pose age.

        Deliberate duplicate of MPCController._update_n_delay (same MPCParams
        fields, same arithmetic): that method is bound to the LTV-QP
        controller's own state, and refactoring it out would mean editing
        mpc_core.py's live solve path, which this feature is specifically
        designed not to touch. Keep the two in sync if either changes.
        """
        age = max(0.0, float(pose_age_s))
        cap = self.params.max_delay_compensation_steps
        if self._pose_age_filtered is None:
            self._pose_age_filtered = age
            self._n_delay = int(np.clip(round(age / self.dt), 0, cap))
            return self._n_delay
        self._pose_age_filtered += (
            self.params.pose_age_lp_alpha * (age - self._pose_age_filtered))
        steps_f = self._pose_age_filtered / self.dt
        if abs(steps_f - self._n_delay) > 0.5 + self.params.n_delay_hysteresis:
            self._n_delay = int(np.clip(round(steps_f), 0, cap))
        return self._n_delay

    # ------------------------------------------------------------------
    # Main entry point
    # ------------------------------------------------------------------
    def compute(
        self,
        path: np.ndarray,
        car_pos: np.ndarray,
        car_yaw: float,
        car_speed: float,
        desired_speed: float,
        car_yaw_rate: float = 0.0,
        pose_age_s: float = 0.0,
        car_vy: float = 0.0,
    ) -> tuple[float, float, float]:
        """
        Run one NMPC control step. Signature, units, return convention
        ((steering in [-1,1], throttle in [0,1], brake in [0,1])) and the
        short-path guard are all identical to MPCController.compute(), so the
        calling nodes need no branch beyond which object they construct.
        """
        if path is None or len(path) < 3:
            return 0.0, 0.0, 0.5

        t0 = time.perf_counter()

        # Same first-order target-speed filter as MPCController.compute().
        alpha = 0.08
        if self._v_des_filtered is None:
            self._v_des_filtered = desired_speed
        self._v_des_filtered += alpha * (desired_speed - self._v_des_filtered)
        v_ref = float(self._v_des_filtered)

        ref = self._path_reference(path)
        if ref is None or ref.total < 1e-3:
            return 0.0, 0.0, 0.5

        # ── Measured Frenet state ──────────────────────────────────────
        fa = np.asarray(car_pos, dtype=float) + self.lf * np.array(
            [math.cos(car_yaw), math.sin(car_yaw)])
        s0, e_y, e_psi, base_idx, path_yaw = ref.project(fa, car_yaw)
        x0 = np.array([
            s0, e_y, e_psi,
            max(float(car_speed), 0.0), float(car_vy), float(car_yaw_rate),
            self._delta_act, self._a_act,
        ])

        # ── Delay compensation (nonlinear rollforward) ──────────────────
        # Same trigger/depth logic as the LTV-QP path, but rolled forward
        # through the NONLINEAR model instead of predict_ahead()'s linear
        # Ad/Bd — the model is right here, so there is no reason to use a
        # linearisation for it.
        if self.params.delay_compensation_enabled:
            n_delay = self._update_n_delay(pose_age_s)
            if n_delay > 0 and self._u_history:
                xk = [float(v) for v in x0]
                for u_hist in self._u_history[-n_delay:]:
                    xk = _step_scalar(xk, u_hist, ref, self.plant, self.dt,
                                      self.nmpc.nmpc_rk_substeps)
                x0 = np.array(xk)
        else:
            n_delay = 0

        # ── Warm start: shift the previous solution one step ────────────
        if self._have_warm_start:
            U = np.vstack([self._U[1:], self._U[-1:]])
        else:
            U = np.tile(self._u_prev, (self.N, 1))
        U = self._project_feasible(U)

        # ── Gauss-Newton SQP ───────────────────────────────────────────
        budget_s = self.nmpc.nmpc_solve_budget_ms * 1e-3
        X = self._rollout(x0, U, ref)
        H = _outputs(X, ref, self.plant, v_ref)
        cost = self._cost(X, U, H)
        iters = 0
        status = 'warm-start-only'
        for _ in range(max(1, int(self.nmpc.nmpc_sqp_iters))):
            if time.perf_counter() - t0 > budget_s:
                status = 'budget'
                break
            dU, status = self._solve_step(X, U, ref, v_ref)
            if dU is None:
                break
            step = 1.0
            accepted = False
            for _bt in range(max(0, int(self.nmpc.nmpc_backtrack_max)) + 1):
                U_try = np.clip(U + step * dU, self.u_min, self.u_max)
                X_try = self._rollout(x0, U_try, ref)
                H_try = _outputs(X_try, ref, self.plant, v_ref)
                cost_try = self._cost(X_try, U_try, H_try)
                # ONLY a genuine improvement is accepted. Accepting the
                # smallest trial regardless (an earlier version of this loop
                # did, "so a tick always makes progress") means a bad search
                # direction is written into the warm start and carried into the
                # next tick — which is how a single failed subproblem grew into
                # a divergent wrong-way full-lock ramp in offline testing.
                # Rejecting leaves U at the previous tick's shifted solution,
                # which is always feasible and always sane.
                if cost_try <= cost:
                    U, X, H, cost = U_try, X_try, H_try, cost_try
                    accepted = True
                    break
                step *= 0.5
            iters += 1
            if not accepted:
                status = 'rejected'
                break
            # U + Delta is slew-feasible by construction (Delta = 0 is feasible
            # and the QP's rate rows are linear, so any fraction of an accepted
            # step is too), and the clip above cannot break that when both
            # endpoints already respect the input bounds.

        self._U = U
        self._have_warm_start = True

        # ── Ship u[0], hard-limited exactly as the LTV-QP path is ──────
        u_opt = np.clip(U[0], self.u_min, self.u_max)
        u_opt = np.clip(u_opt, self._u_prev - self.du_max, self._u_prev + self.du_max)
        self._u_history.append(u_opt.copy())
        if len(self._u_history) > self.params.max_delay_compensation_steps:
            del self._u_history[:-self.params.max_delay_compensation_steps]

        exp_delta = math.exp(-self.dt / self.plant.tau_delta)
        exp_a = math.exp(-self.dt / self.plant.tau_a)
        self._delta_act = self._delta_act * exp_delta + u_opt[0] * (1.0 - exp_delta)
        self._a_act = self._a_act * exp_a + u_opt[1] * (1.0 - exp_a)
        self._u_prev = u_opt.copy()

        delta_cmd = float(np.clip(u_opt[0], -MAX_STEER_RAD, MAX_STEER_RAD))
        a_cmd = float(u_opt[1])
        steering = float(np.clip(-delta_cmd / MAX_STEER_RAD, -1.0, 1.0))
        if a_cmd >= 0.0:
            throttle = float(np.clip(a_cmd / self.a_max, 0.0, 1.0))
            brake = 0.0
        else:
            throttle = 0.0
            brake = float(np.clip(-a_cmd / self.a_max_brake, 0.0, 1.0))

        solve_ms = (time.perf_counter() - t0) * 1e3

        # Telemetry: the keys mpc_core publishes keep their exact meaning (so
        # every existing offline analysis script and telemetry_logger column
        # still works), plus nmpc_* diagnostics. kappa/kappa_max_abs are
        # recomputed here from the SAME kappa(s) reference the prediction used
        # — kappa at the car, and peak |kappa| over the horizon's own predicted
        # arc length, which is the NMPC's natural analogue of the LTV-QP's
        # speed-scaled lookahead window.
        kap_horizon = ref.kappa_at(X[:, IDX_S])
        self.last_telemetry = {
            'e_y': float(e_y),
            'e_psi': float(e_psi),
            'e_v': float(car_speed - v_ref),
            'kappa': float(ref.kappa_at(np.array([s0]))[0]),
            'base_idx': int(base_idx),
            'kappa_max_abs': float(np.abs(kap_horizon).max()),
            'pose_age_s': float(pose_age_s),
            'n_delay': int(n_delay),
            'solve_ms': float(solve_ms),
            'car_speed': float(car_speed),
            'desired_speed': float(v_ref),
            'steering': steering,
            'throttle': throttle,
            'brake': brake,
            'delta_cmd': delta_cmd,
            'a_cmd': a_cmd,
            'delta_act': float(self._delta_act),
            'a_act': float(self._a_act),
            # NMPC-specific: see telemetry_logger.NMPC_COLUMNS.
            'nmpc_iters': int(iters),
            'nmpc_cost': float(cost),
            'nmpc_status': 1.0 if status.lower().startswith('solved') else 0.0,
            'nmpc_s0': float(s0),
            'nmpc_kappa_horizon_end': float(kap_horizon[-1]),
            'nmpc_pred_ey_end': float(X[-1, IDX_EY]),
            'nmpc_pred_epsi_end': float(X[-1, IDX_EPSI]),
            'nmpc_pred_ey_max_abs': float(np.abs(X[:, IDX_EY]).max()),
        }
        return steering, throttle, brake

    def _path_reference(self, path) -> PathReference | None:
        """
        Return the PathReference for this tick's path: the one built at load
        time by set_static_path() when its signature still matches (the static
        case — zero per-tick cost), otherwise a cached per-tick rebuild that is
        only redone when the path actually changes (live-planner mode).
        """
        path = np.asarray(path, dtype=float)
        sig = (
            len(path),
            float(path[0, 0]), float(path[0, 1]),
            float(path[-1, 0]), float(path[-1, 1]),
        )
        if self._static_ref is not None and self._static_ref.signature[:5] == sig:
            return self._static_ref
        if self._ref is not None and self._ref_signature == sig:
            return self._ref
        try:
            self._ref = PathReference(
                path,
                dense_step=self.nmpc.nmpc_curvature_dense_step,
                smooth_w=self.nmpc.nmpc_curvature_smooth_w,
                kappa_clip=self.nmpc.nmpc_kappa_clip,
            )
        except (ValueError, IndexError):    # pragma: no cover - defensive
            return None
        self._ref_signature = sig
        return self._ref
