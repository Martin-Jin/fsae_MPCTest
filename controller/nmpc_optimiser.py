"""
controller/nmpc_optimiser.py — Nonlinear MPC (NMPC), offline counterpart of
the live ROS 2 side's `fsae_control.nmpc_core.NMPCController`.

PURPOSE
-------
`controller/optimiser.py`'s `solve_mpc()` is a linear time-varying MPC: its
prediction model (`model/bicycle_model.py`) is the bicycle model in ERROR
coordinates with the reference path's own rotation entirely absent from it
(`e_psi_dot` = yaw rate only, never `r - kappa(s)*s_dot`). With the car
exactly on-line and on-heading approaching a corner, that model's own N-step
rollout predicts staying at zero forever — no cost weighting can produce
turn-in before real tracking error exists. See `model/bicycle_model.py`'s own
docstring and `docs/junior_project_mpc_docs.md` §4.2 for the plain-language
version, and `late_turn_in_investigation.md` (live repo) Parts 1-15 for the
full investigation this is downstream of.

This module is a SECOND controller that closes that gap: a Frenet-frame
(curvilinear-coordinate) nonlinear model, where arc length `s` is a STATE and
the path's curvature `kappa(s)` is looked up from it directly, so a bend
ahead is part of the dynamics rather than reweighted cost. Solved by
Gauss-Newton SQP (repeated re-linearisation + a condensed dense QP per
iteration, via OSQP), not one convex QP.

RELATIONSHIP TO THE LIVE SIDE
------------------------------
This is a faithful, independent PORT of `nmpc_core.py`'s `NMPCController` —
same model, same SQP/condensing/OSQP scheme, same variable/function names
where they carry over — NOT an import (`fsae_MPCTest` cannot import from the
live `fsae_planning` checkout and vice versa; CLAUDE.md's standing
"no settings.py-on-the-car" rule, from the other direction here). Kept
numerically identical BY HAND, the same discipline as every other
live/offline pair in this project (Q_diag/R_diag/R_rate_diag, etc.) — see
`settings.py`'s NMPC_* constants and `docs/tuning.md`'s NMPC section for the
field-by-field mapping this needs to be kept in sync with if either side's
model or solver changes.

Two differences from the live module, both because this is the offline side:
  - Vehicle constants (lf, lr, m, Iz, Cf, Cr, tau_delta, tau_a) are read
    directly from the `VehicleParams` instance already passed around this
    repo (the SAME source `model/bicycle_model.py`'s linear model and the
    24-state nonlinear plant both use), rather than a hardcoded copy — this
    repo has no "no cross-import" constraint against its own `model/` package.
  - Cost weights are NOT read from a dataclass. `sim/rollout_core.py`'s
    `run_core_rollout()` already receives the CURRENT weight set (whether
    from `settings.py` or a CMA-ES tuning candidate) as `Q`/`R`/`R_rate`
    arrays; `NMPCController` here is constructed with those same arrays (plus
    the NMPC-only override scalars from `settings.py`) so a tuner sweep
    reaches the NMPC's weights exactly the way it reaches the LTV-QP's.

USED BY
-------
  sim/rollout_core.py — run_core_rollout(), when settings.USE_NMPC is True.
    Constructed once per rollout (outside the step loop, so its warm start
    persists across ticks exactly like the LTV path's u_prev/command_queue).

DOES NOT USE
------------
  controller/optimiser.py, model/bicycle_model.py (this module's own model
  replaces both when active), gui/simulation.py (imported from
  rollout_core.py only, same reasoning as controller/optimiser.py's own
  "DOES NOT USE" note).
"""

import math

import numpy as np
import scipy.sparse as sp

try:
    import osqp
except ImportError as _exc:      # pragma: no cover - see README's dependency list
    osqp = None
    _OSQP_IMPORT_ERROR = _exc


# ── State/input/output layout (identical to the live module) ────────────
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

_FD_EPS_X = np.array([1e-6, 1e-6, 1e-7, 1e-6, 1e-6, 1e-7, 1e-7, 1e-6])
_FD_EPS_U = np.array([1e-7, 1e-6])

# Guard on the Frenet denominator (1 - kappa*e_y): singular at e_y = 1/kappa.
# Inert on any real track line (see the live module's identical comment).
_DENOM_FLOOR = 0.25


def _wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


class PathReference:
    """
    Arc-length parameterisation of a waypoint path, plus the smoothed
    curvature/reference-heading profile the prediction needs.

    Identical design and reasoning to the live module's `PathReference` —
    see that docstring for the full "why smoothed, not raw tangent" story
    (a raw per-waypoint tangent steps by ds/R, which the NMPC reads as real
    tracking error and turns into a steering limit cycle; confirmed on this
    project's own recorded track, see `late_turn_in_investigation.md`
    Part 16 §16.6, live repo).
    """

    def __init__(self, path, dense_step=0.5, smooth_w=3, kappa_clip=0.5):
        path = np.asarray(path, dtype=float)
        self.path = path
        seg = np.diff(path, axis=0)
        seg_len = np.hypot(seg[:, 0], seg[:, 1])
        self.arc = np.concatenate([[0.0], np.cumsum(seg_len)])
        self.total = float(self.arc[-1]) if len(self.arc) else 0.0

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
                s_k = s0 + dense_step * (np.arange(len(k)) + 1.0)
                kappa = np.clip(k, -kappa_clip, kappa_clip)
                psi_mid = np.unwrap(psi)
                psi_ref = 0.5 * (psi_mid[:-1] + psi_mid[1:])

        self.s_kappa = s_k
        self.kappa = kappa
        self._k_list = [float(v) for v in np.atleast_1d(kappa)]
        self._k_n = len(self._k_list)
        self._k_s0 = float(s_k[0])
        self._k_ds = float(dense_step)
        self._k_uniform = self._k_n >= 3

        if psi_ref is None:
            d = np.diff(path, axis=0)
            raw = np.unwrap(np.arctan2(d[:, 1], d[:, 0]))
            self.s_psi = 0.5 * (self.arc[:-1] + self.arc[1:])
            self.psi_ref = raw
        else:
            self.s_psi = s_k
            self.psi_ref = psi_ref

        self.signature = (
            len(path),
            float(path[0, 0]), float(path[0, 1]),
            float(path[-1, 0]), float(path[-1, 1]),
            round(self.total, 6),
        )

    def kappa_at(self, s):
        return np.interp(s, self.s_kappa, self.kappa)

    def kappa_scalar(self, s):
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
        return np.interp(s, self.s_psi, self.psi_ref)

    def project(self, front_axle, car_yaw):
        """
        Frenet projection of a front-axle position onto the path — identical
        arithmetic to `model/vehicle_physics.plant_to_tracking_error`'s
        nearest-waypoint + perpendicular-projection scheme, except the
        heading reference is the SMOOTHED `psi_ref(s)`, not the raw per-
        waypoint tangent (see this class's docstring).
        """
        path = self.path
        base_idx = int(np.argmin(np.linalg.norm(path - front_axle, axis=1)))
        s_base = float(self.arc[base_idx])
        path_yaw = float(self.psi_ref_at(s_base))
        dx = front_axle[0] - path[base_idx][0]
        dy = front_axle[1] - path[base_idx][1]
        cos_y, sin_y = math.cos(path_yaw), math.sin(path_yaw)
        e_y = dy * cos_y - dx * sin_y
        along = dx * cos_y + dy * sin_y
        s0 = s_base + along
        path_yaw = float(self.psi_ref_at(s0))
        e_psi = float(_wrap(car_yaw - path_yaw))
        return s0, float(e_y), e_psi, base_idx, path_yaw


class _Plant:
    """Vehicle constants for the prediction model, read from VehicleParams."""

    def __init__(self, vp, alat_ceiling_enabled=True,
                 alat_flat=7.5, alat_slope=0.47, alat_intercept=2.46):
        self.lf = vp.lf
        self.lr = vp.lr
        self.m = vp.m
        self.Iz = vp.Iz
        self.Cf = vp.Cf
        self.Cr = vp.Cr
        self.tau_delta = vp.tau_delta
        self.tau_a = vp.tau_a
        self.v_blend_lo = 1.0
        self.v_blend_hi = 2.5
        self.alat_ceiling_enabled = alat_ceiling_enabled
        self.alat_ceiling_flat = alat_flat
        self.alat_ceiling_slope = alat_slope
        self.alat_ceiling_intercept = alat_intercept


def _f(X, U, ref, p):
    """Continuous-time dynamics, vectorised over horizon stages. See the
    live nmpc_core.py's `_f` for the full equations/derivation; identical
    here."""
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
    denom = np.where(denom >= 0.0,
                     np.maximum(denom, _DENOM_FLOOR),
                     np.minimum(denom, -_DENOM_FLOOR))

    cos_ep = np.cos(e_psi)
    sin_ep = np.sin(e_psi)
    s_dot = (v_x * cos_ep - v_y * sin_ep) / denom
    e_y_dot = v_x * sin_ep + v_y * cos_ep
    e_psi_dot = r - kap * s_dot

    v_safe = np.maximum(np.abs(v_x), p.v_blend_hi)
    alpha_f = np.arctan((v_y + p.lf * r) / v_safe) - d
    alpha_r = np.arctan((v_y - p.lr * r) / v_safe)
    F_yf = -2.0 * p.Cf * alpha_f
    F_yr = -2.0 * p.Cr * alpha_r
    cos_d = np.cos(d)

    if p.alat_ceiling_enabled:
        # Smooth saturation of the lateral forces at FSDS's measured
        # sustained a_lat ceiling. Without this the linear-tyre prediction
        # believes it can hold any corner at any speed; the simulated
        # nonlinear plant cannot, and the car spins mid-lap without it (see
        # late_turn_in_investigation.md Part 16 §16.6, live repo).
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
    r_dot_dyn = (p.lf * F_yf * cos_d - p.lr * F_yr) / p.Iz

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
    """Scalar mirror of `_f` — see the live module's identical function for
    why this exists (numpy per-call overhead dominates the sequential
    rollout at this array size). Must stay in lockstep with `_f`; see
    `test_nmpc_offline_check.py`'s parity test."""
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
    """Scalar RK4 step + exact ZOH actuator overwrite. See the live module's
    `_step_scalar` for why the actuator states are overwritten exactly
    rather than left to RK4 (tau_a=0.02s is stiff against dt=0.05s)."""
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
    """Vectorised RK4 step over M stages at once — used only for the
    finite-difference Jacobians (see `NMPCController._jacobians`)."""
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
    np.maximum(Xk[:, IDX_VX], 0.0, out=Xk[:, IDX_VX])
    return Xk


def _outputs(X, ref, p, v_ref):
    """Stage output h(x) = [e_y, e_y_dot, e_psi, e_psi_dot, v_x - v_ref].
    e_psi_dot = r - kappa(s)*s_dot is the heading-error RATE, not absolute
    yaw rate -- see settings.py's NMPC_Q_EPSI_DOT comment for why this is
    the one weight whose meaning differs from the LTV-QP's Q_diag[3]."""
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
    """Fixed-sparsity-pattern CSC matrix + (row,col) index arrays for
    writing into its .data in CSC order (OSQP requires a stable pattern
    across update() calls)."""
    m = sp.csc_matrix(mask.astype(np.float64))
    rows = m.indices.copy()
    cols = np.zeros_like(rows)
    for j in range(m.shape[1]):
        cols[m.indptr[j]:m.indptr[j + 1]] = j
    return m, rows, cols


class NMPCController:
    """
    Offline counterpart of the live side's `nmpc_core.NMPCController`. See
    this module's own docstring for the relationship between the two, and
    the live module's docstring for the full model/solver explanation
    (identical here) — not repeated per-method here to avoid the two
    docstrings drifting apart in wording while the code itself is kept in
    sync by hand.
    """

    def __init__(
        self, dt, N, vehicle_params,
        u_min, u_max, du_max,
        q_e_y, q_e_yd, q_e_psi, q_epsi_dot, q_e_v,
        r_delta, r_a_accel, r_a_brake, r_rate_delta, r_rate_a,
        terminal_scale=1.0,
        sqp_iters=1, solve_budget_ms=25.0,
        rk_substeps=2, jac_substeps=1,
        trust_delta_rad=math.radians(9.0), trust_a=0.6, backtrack_max=2,
        track_halfwidth=3.5, slack_weight=10000.0,
        osqp_max_iter=500, osqp_eps=1e-4,
        alat_ceiling_enabled=True,
        alat_flat=7.5, alat_slope=0.47, alat_intercept=2.46,
    ):
        if osqp is None:      # pragma: no cover - dependency guard
            raise ImportError(
                f'nmpc_optimiser requires osqp (already a dependency of '
                f'controller/optimiser.py via cvxpy): {_OSQP_IMPORT_ERROR!r}'
            )
        self.dt = float(dt)
        self.N = int(N)
        if self.N < 2:
            raise ValueError(f'NMPC horizon must be >= 2 (got {self.N})')

        self.plant = _Plant(
            vehicle_params, alat_ceiling_enabled=alat_ceiling_enabled,
            alat_flat=alat_flat, alat_slope=alat_slope, alat_intercept=alat_intercept,
        )
        self.lf, self.lr = self.plant.lf, self.plant.lr

        self.u_min = np.asarray(u_min, dtype=float)
        self.u_max = np.asarray(u_max, dtype=float)
        self.du_max = np.asarray(du_max, dtype=float)

        self.w_out = np.array([q_e_y, q_e_yd, q_e_psi, q_epsi_dot, q_e_v], dtype=float)
        self.r_delta = float(r_delta)
        self.r_a_accel = float(r_a_accel)
        self.r_a_brake = float(r_a_brake)
        self.r_rate = np.array([r_rate_delta, r_rate_a], dtype=float)
        self.terminal_scale = float(terminal_scale)

        self.sqp_iters = int(sqp_iters)
        self.solve_budget_ms = float(solve_budget_ms)
        self.rk_substeps = int(rk_substeps)
        self.jac_substeps = max(1, int(jac_substeps))
        self.trust_delta_rad = float(trust_delta_rad)
        self.trust_a = float(trust_a)
        self.backtrack_max = int(backtrack_max)
        self.track_halfwidth = float(track_halfwidth)
        self.slack_weight = float(slack_weight)
        self.osqp_max_iter = int(osqp_max_iter)
        self.osqp_eps = float(osqp_eps)

        self._delta_act = 0.0
        self._a_act = 0.0
        self._u_prev = np.zeros(NU)
        self._U = np.zeros((self.N, NU))
        self._have_warm_start = False

        self._ref = None
        self._ref_signature = None

        self._qp = None
        self._build_qp()

    # ------------------------------------------------------------------
    def path_reference(self, path, dense_step=0.5, smooth_w=3, kappa_clip=0.5):
        """
        Return the PathReference for `path`, rebuilding only when the path's
        signature (endpoints/length) has changed since the last call — so a
        fixed oracle path (USE_PLANNER=False) costs this once, while a
        planner-built centreline (USE_PLANNER=True, which changes every
        tick) is rebuilt each time it actually changes. Mirrors the live
        module's per-tick caching exactly.
        """
        path = np.asarray(path, dtype=float)
        sig = (
            len(path),
            float(path[0, 0]), float(path[0, 1]),
            float(path[-1, 0]), float(path[-1, 1]),
        )
        if self._ref is not None and self._ref_signature == sig:
            return self._ref
        self._ref = PathReference(
            path, dense_step=dense_step, smooth_w=smooth_w, kappa_clip=kappa_clip,
        )
        self._ref_signature = sig
        return self._ref

    def reset(self):
        self._delta_act = 0.0
        self._a_act = 0.0
        self._u_prev = np.zeros(NU)
        self._U = np.zeros((self.N, NU))
        self._have_warm_start = False

    # ------------------------------------------------------------------
    def _build_qp(self):
        N = self.N
        n_du = NU * N
        self._use_slack = self.track_halfwidth > 0.0
        n_slack = N if self._use_slack else 0
        nz = n_du + n_slack

        E = np.zeros((n_du, n_du))
        for k in range(N):
            E[k * NU:(k + 1) * NU, k * NU:(k + 1) * NU] = np.eye(NU)
            if k > 0:
                E[k * NU:(k + 1) * NU, (k - 1) * NU:k * NU] = -np.eye(NU)
        self._E = E
        self._Rr_flat = np.tile(self.r_rate, N)
        self._ErE = E.T @ (self._Rr_flat[:, None] * E)

        p_mask = np.zeros((nz, nz), dtype=bool)
        p_mask[:n_du, :n_du] = np.triu(np.ones((n_du, n_du), dtype=bool))
        if n_slack:
            idx = np.arange(n_du, nz)
            p_mask[idx, idx] = True
        P, p_rows, p_cols = _csc_pattern(p_mask)

        n_rows = 2 * n_du + (3 * N if self._use_slack else 0)
        a_mask = np.zeros((n_rows, nz), dtype=bool)
        a_mask[:n_du, :n_du] = np.eye(n_du, dtype=bool)
        a_mask[n_du:2 * n_du, :n_du] = E != 0.0
        if self._use_slack:
            r0 = 2 * n_du
            a_mask[r0:r0 + 2 * N, :n_du] = True
            for k in range(N):
                a_mask[r0 + k, n_du + k] = True
                a_mask[r0 + N + k, n_du + k] = True
                a_mask[r0 + 2 * N + k, n_du + k] = True
        A, a_rows, a_cols = _csc_pattern(a_mask)

        q = np.zeros(nz)
        l = np.full(n_rows, -np.inf)
        u = np.full(n_rows, np.inf)

        prob = osqp.OSQP()
        settings = dict(
            verbose=False, eps_abs=self.osqp_eps, eps_rel=self.osqp_eps,
            max_iter=self.osqp_max_iter,
        )
        try:
            prob.setup(P, q, A, l, u, warm_starting=True, polishing=False, **settings)
        except TypeError:      # pragma: no cover - osqp < 1.0 naming
            prob.setup(P, q, A, l, u, warm_start=True, polish=False, **settings)

        self._qp = dict(
            prob=prob, P=P, A=A,
            p_rows=p_rows, p_cols=p_cols,
            a_rows=a_rows, a_cols=a_cols,
            n_du=n_du, n_slack=n_slack, nz=nz, n_rows=n_rows,
        )

    # ------------------------------------------------------------------
    def _rollout(self, x0, U, ref):
        N = self.N
        X = np.empty((N + 1, NX))
        X[0] = x0
        xk = [float(v) for v in x0]
        p, dt, n_sub = self.plant, self.dt, self.rk_substeps
        for k in range(N):
            xk = _step_scalar(xk, U[k], ref, p, dt, n_sub)
            X[k + 1] = xk
        return X

    def _jacobians(self, X, U, ref):
        N = self.N
        Xs = X[:N]
        p, dt, n_sub = self.plant, self.dt, self.jac_substeps
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
        H0 = _outputs(X, ref, self.plant, v_ref)
        C = np.empty((X.shape[0], NH, NX))
        for j in range(NX):
            Xp = X.copy()
            Xp[:, j] += _FD_EPS_X[j]
            C[:, :, j] = (_outputs(Xp, ref, self.plant, v_ref) - H0) / _FD_EPS_X[j]
        return H0, C

    def _cost(self, X, U, H):
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
            over = np.maximum(np.abs(X[1:, IDX_EY]) - self.track_halfwidth, 0.0)
            slack = float(self.slack_weight * np.sum(over ** 2))
        return stage + eff + rate + slack

    def _project_feasible(self, U):
        """Project onto input bounds + per-step slew feasibility from
        `self._u_prev` forward, so `dU=0` is always feasible and the SQP
        subproblem is unconditionally feasible by construction — see the
        live module's identical method for why this matters (a warm start
        that violates the slew limit can make the whole subproblem
        primal-infeasible, and OSQP returns a finite-but-meaningless answer
        in that case)."""
        Up = np.clip(np.asarray(U, dtype=float), self.u_min, self.u_max)
        prev = self._u_prev
        for k in range(Up.shape[0]):
            Up[k] = np.clip(Up[k], prev - self.du_max, prev + self.du_max)
            prev = Up[k]
        return Up

    def _solve_step(self, X, U, ref, v_ref):
        N = self.N
        qp = self._qp
        n_du, n_slack, nz, n_rows = qp['n_du'], qp['n_slack'], qp['nz'], qp['n_rows']

        A_k, B_k = self._jacobians(X, U, ref)
        H, C = self._output_jacobians(X, ref, v_ref)

        S = np.zeros((N + 1, NX, n_du))
        for k in range(N):
            S[k + 1] = A_k[k] @ S[k]
            S[k + 1][:, k * NU:(k + 1) * NU] += B_k[k]

        sw = np.sqrt(self.w_out)
        scale = np.ones(N + 1)
        scale[N] = math.sqrt(max(self.terminal_scale, 0.0))
        WC = (sw[None, :, None] * C) * scale[:, None, None]
        G = np.einsum('kij,kjl->kil', WC, S).reshape((N + 1) * NH, n_du)
        g = ((sw[None, :] * H) * scale[:, None]).reshape(-1)

        ru = np.empty((N, NU))
        ru[:, 0] = self.r_delta
        ru[:, 1] = np.where(U[:, 1] >= 0.0, self.r_a_accel, self.r_a_brake)
        ru_flat = ru.reshape(-1)
        u_flat = U.reshape(-1)

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
            P_dense[idx, idx] = 2.0 * self.slack_weight

        A_dense = np.zeros((n_rows, nz))
        l = np.empty(n_rows)
        u = np.empty(n_rows)

        A_dense[:n_du, :n_du] = np.eye(n_du)
        tr = np.tile(np.array([self.trust_delta_rad, self.trust_a]), N)
        lo = np.maximum(np.tile(self.u_min, N) - u_flat, -tr)
        hi = np.minimum(np.tile(self.u_max, N) - u_flat, tr)
        lo = np.minimum(lo, hi)
        l[:n_du], u[:n_du] = lo, hi

        A_dense[n_du:2 * n_du, :n_du] = self._E
        du_flat = np.tile(self.du_max, N)
        l[n_du:2 * n_du] = -du_flat - e_rate
        u[n_du:2 * n_du] = du_flat - e_rate

        if n_slack:
            r0 = 2 * n_du
            hw = self.track_halfwidth
            S_ey = S[1:, IDX_EY, :]
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
        ok = ('solved' in status) or ('maximum iterations' in status)
        if not ok or res.x is None or not np.all(np.isfinite(res.x[:n_du])):
            return None, status
        return res.x[:n_du].reshape(N, NU), status

    # ------------------------------------------------------------------
    def compute_step(
        self, path, car_pos, car_yaw, car_speed, desired_speed,
        car_yaw_rate=0.0, car_vy=0.0, pending_cmds=None,
        dense_step=0.5, smooth_w=3, kappa_clip=0.5,
        step_index=0,
    ):
        """
        One NMPC control step. Deliberately DIFFERENT calling convention from
        the live module's `compute()`: returns raw `(u_opt, status_dict)`
        with `u_opt = [delta_cmd (rad), a_cmd (m/s^2)]` — matching
        `controller/optimiser.solve_mpc()`'s own return convention — rather
        than normalised (steering, throttle, brake) FSDS units, since
        `sim/rollout_core.py`'s `step_nonlinear_plant()` (unlike the live
        ROS node) wants the raw physical command directly.

        `pending_cmds` (list of `[delta_cmd, a_cmd]` arrays, oldest first) is
        used for delay compensation via a NONLINEAR rollforward, exactly
        like the live module's `_u_history`-based rollforward — but is taken
        as an explicit argument here rather than derived internally from a
        `pose_age_s` measurement, because `sim/rollout_core.py`'s existing
        delay-JITTER model (settings.DELAY_JITTER_STEPS) already perturbs
        the BELIEVED pending-command list directly (see that module's
        "Delay-estimation error" comment) — reusing that list is more
        faithful to what the live car's noisy pose-timestamp model is
        actually standing in for than re-deriving a second, independent
        noise source here.

        `step_index`: 0 on the rollout's first tick (skips warm-start, same
        as `solve_mpc`'s own `warm_start=(step != 0)`).
        """
        import time
        t0 = time.perf_counter()

        ref = self.path_reference(
            path, dense_step=dense_step, smooth_w=smooth_w, kappa_clip=kappa_clip,
        )
        if ref.total < 1e-3:
            return np.array([self._u_prev[0], self.u_min[1]]), {
                'iters': 0, 'status': 'no-path', 'cost': float('nan'),
            }

        fa = np.asarray(car_pos, dtype=float) + self.lf * np.array(
            [math.cos(car_yaw), math.sin(car_yaw)])
        s0, e_y, e_psi, base_idx, path_yaw = ref.project(fa, car_yaw)
        x0 = np.array([
            s0, e_y, e_psi, max(float(car_speed), 0.0), float(car_vy),
            float(car_yaw_rate), self._delta_act, self._a_act,
        ])

        if pending_cmds:
            xk = [float(v) for v in x0]
            for u_hist in pending_cmds:
                xk = _step_scalar(xk, u_hist, ref, self.plant, self.dt, self.rk_substeps)
            x0 = np.array(xk)

        if self._have_warm_start and step_index != 0:
            U = np.vstack([self._U[1:], self._U[-1:]])
        else:
            U = np.tile(self._u_prev, (self.N, 1))
        U = self._project_feasible(U)

        budget_s = self.solve_budget_ms * 1e-3
        X = self._rollout(x0, U, ref)
        H = _outputs(X, ref, self.plant, desired_speed)
        cost = self._cost(X, U, H)
        iters = 0
        status = 'warm-start-only'
        for _ in range(max(1, self.sqp_iters)):
            if time.perf_counter() - t0 > budget_s:
                status = 'budget'
                break
            dU, status = self._solve_step(X, U, ref, desired_speed)
            if dU is None:
                break
            step = 1.0
            accepted = False
            for _bt in range(max(0, self.backtrack_max) + 1):
                U_try = np.clip(U + step * dU, self.u_min, self.u_max)
                X_try = self._rollout(x0, U_try, ref)
                H_try = _outputs(X_try, ref, self.plant, desired_speed)
                cost_try = self._cost(X_try, U_try, H_try)
                if cost_try <= cost:
                    U, X, H, cost = U_try, X_try, H_try, cost_try
                    accepted = True
                    break
                step *= 0.5
            iters += 1
            if not accepted:
                status = 'rejected'
                break

        self._U = U
        self._have_warm_start = True

        u_opt = np.clip(U[0], self.u_min, self.u_max)
        u_opt = np.clip(u_opt, self._u_prev - self.du_max, self._u_prev + self.du_max)

        exp_delta = math.exp(-self.dt / self.plant.tau_delta)
        exp_a = math.exp(-self.dt / self.plant.tau_a)
        self._delta_act = self._delta_act * exp_delta + u_opt[0] * (1.0 - exp_delta)
        self._a_act = self._a_act * exp_a + u_opt[1] * (1.0 - exp_a)
        self._u_prev = u_opt.copy()

        diag = {
            'iters': iters,
            'status': status,
            'solved': status.lower().startswith('solved'),
            'cost': float(cost),
            's0': float(s0),
            'e_y': float(e_y),
            'e_psi': float(e_psi),
            'kappa': float(ref.kappa_scalar(s0)),
            'kappa_max_abs': float(np.abs(ref.kappa_at(X[:, IDX_S])).max()),
            'pred_ey_end': float(X[-1, IDX_EY]),
            'pred_epsi_end': float(X[-1, IDX_EPSI]),
            'pred_ey_max_abs': float(np.abs(X[:, IDX_EY]).max()),
            'solve_ms': (time.perf_counter() - t0) * 1e3,
        }
        return u_opt, diag
