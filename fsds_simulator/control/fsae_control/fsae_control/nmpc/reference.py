"""
fsae_control/nmpc/reference.py — PathReference

Arc-length/curvature reference for a path: spline curvature, projection of the
front axle onto the path, and xy/heading lookups by arc length.
"""

import math

import numpy as np
from scipy.interpolate import CubicSpline

from fsae_control.nmpc.layout import _wrap


class PathReference:
    """
    Arc-length parameterisation of a waypoint path, plus the curvature
    profile kappa(s) the NMPC's prediction needs.

    Built once per DISTINCT path: for a precomputed/static path that is once
    per run (see NMPCController.set_static_path / the per-tick signature cache
    in compute()); in live-planner mode the path changes every tick and this is
    rebuilt each time (measured cost in Part 16 §16.7).

    kappa(s)/psi_ref(s) construction (NMPCParams.nmpc_spline_reference_enabled,
    default True): x(s) and y(s) are each fit as an independent
    `scipy.interpolate.CubicSpline` over the raw (not resampled) arc-length
    knots, and kappa/psi_ref are the spline's own analytic first/second
    derivatives (psi_ref = atan2(y', x'), kappa = (x'y'' - y'x'')/(x'^2+y'^2)^1.5),
    evaluated on a dense `dense_step` grid. This replaces the previous
    dense-resample + moving-average + finite-difference pipeline — the SAME
    denoise precedent control_utils.curvature_speed() uses (dense_step 0.5 m,
    w 3) — which is still present and used verbatim when the flag is False,
    kept for A/B comparison against the known "centreline curvature spikes"
    defect this addresses (see CLAUDE.md). Either way this matters more here
    than for a speed cap, because kappa enters the PREDICTION and would be
    steered for.
    """

    def __init__(self, path, dense_step=0.5, smooth_w=3, kappa_clip=0.5,
                 spline_reference_enabled=True):
        path = np.asarray(path, dtype=float)
        seg = np.diff(path, axis=0)
        seg_len = np.hypot(seg[:, 0], seg[:, 1])

        # A live planner path can arrive padded with the last real point
        # repeated to a fixed array length (seen live 2026-09-15, see
        # nmpc_planner_only_corner_failure.md): those trailing zero-length
        # segments look like valid flat geometry to everything below, so the
        # horizon predicts the corner simply stopping. Drop them before any
        # arc-length/kappa/psi_ref math runs, so the real end of data becomes
        # this path's actual last point and the existing edge-hold behaviour
        # in kappa_at/psi_ref_at (see their docstrings) takes over from there,
        # holding the last REAL sample instead of a frozen duplicate one.
        real_n = len(path)
        while real_n > 2 and seg_len[real_n - 2] < 1e-6:
            real_n -= 1
        if real_n < len(path):
            path = path[:real_n]
            seg_len = seg_len[:real_n - 1]

        self.path = path
        self.arc = np.concatenate([[0.0], np.cumsum(seg_len)])
        self.total = float(self.arc[-1]) if len(self.arc) else 0.0

        s_k = np.array([0.0, max(self.total, 1e-3)])
        kappa = np.zeros(2)
        psi_ref = None
        if self.total > 4.0 * max(dense_step, 1e-6):
            if spline_reference_enabled:
                s_k, kappa, psi_ref = self._spline_kappa_psi(
                    path, self.arc, dense_step, kappa_clip)
            else:
                # Dense resample + smooth + heading difference -> kappa(s).
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

    @staticmethod
    def _spline_kappa_psi(path, arc, dense_step, kappa_clip):
        """
        Analytic kappa(s)/psi_ref(s) from independent CubicSpline fits of
        x(s), y(s) over the RAW arc-length knots `arc` (not a dense-
        resampled grid — the spline itself is the smoothing step, so no
        separate resample/moving-average is needed). Not-a-knot boundary
        conditions (scipy's default): these are open racing lines, not
        closed loops (self.total/self.arc are treated as a plain open
        interval everywhere else in this class), so no periodic wraparound
        is added.
        """
        cs_x = CubicSpline(arc, path[:, 0])
        cs_y = CubicSpline(arc, path[:, 1])
        dx1, dy1 = cs_x.derivative(1), cs_y.derivative(1)
        dx2, dy2 = cs_x.derivative(2), cs_y.derivative(2)

        s_k = np.arange(0.0, arc[-1], dense_step)
        if len(s_k) < 3 or s_k[-1] < arc[-1]:
            s_k = np.append(s_k, arc[-1])

        xp, yp = dx1(s_k), dy1(s_k)
        xpp, ypp = dx2(s_k), dy2(s_k)
        denom = np.maximum(xp ** 2 + yp ** 2, 1e-9) ** 1.5
        kappa = (xp * ypp - yp * xpp) / denom
        kappa = np.clip(kappa, -kappa_clip, kappa_clip)
        psi_ref = np.unwrap(np.arctan2(yp, xp))
        return s_k, kappa, psi_ref

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

    def xy_at(self, s, e_y):
        """
        Cartesian (x, y) for arc length(s) `s` with perpendicular offset(s)
        `e_y`, the exact inverse of project()'s e_y projection: the path
        point at `s` (interpolated off the raw waypoints, matching arc/path
        elsewhere in this class) offset by e_y along the LEFT normal of
        psi_ref_at(s), so xy_at(*project(front_axle, yaw)[:2]) recovers
        front_axle. Used to convert the NMPC's own Frenet-frame horizon
        prediction (s, e_y per stage) into a plottable Cartesian trajectory
        for live visualisation, see live_viz.py.
        """
        s = np.atleast_1d(np.asarray(s, dtype=float))
        e_y = np.atleast_1d(np.asarray(e_y, dtype=float))
        x_path = np.interp(s, self.arc, self.path[:, 0])
        y_path = np.interp(s, self.arc, self.path[:, 1])
        psi = self.psi_ref_at(s)
        x = x_path - e_y * np.sin(psi)
        y = y_path + e_y * np.cos(psi)
        return x, y

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
