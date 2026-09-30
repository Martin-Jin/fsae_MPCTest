"""
controller/nmpc/reference.py — `PathReference`: arc-length parameterisation
of the waypoint path plus the curvature/reference-heading profile the NMPC
prediction looks up by `s`.
"""

import math

import numpy as np
from scipy.interpolate import CubicSpline

from controller.nmpc.layout import _wrap


class PathReference:
    """
    Arc-length parameterisation of a waypoint path, plus the
    curvature/reference-heading profile the prediction needs.

    Identical design and reasoning to the live module's `PathReference` —
    see that docstring for the full "why not the raw tangent" story
    (a raw per-waypoint tangent steps by ds/R, which the NMPC reads as real
    tracking error and turns into a steering limit cycle; confirmed on this
    project's own recorded track, see `late_turn_in_investigation.md`
    Part 16 §16.6, live repo).

    kappa(s)/psi_ref(s) construction (settings.NMPC_SPLINE_REFERENCE_ENABLED,
    default True): x(s) and y(s) are each fit as an independent
    `scipy.interpolate.CubicSpline` over the raw (not resampled) arc-length
    knots, and kappa/psi_ref are the spline's own analytic first/second
    derivatives (psi_ref = atan2(y', x'), kappa = (x'y'' - y'x'')/(x'^2+y'^2)^1.5),
    evaluated on a dense `dense_step` grid — replacing the previous
    dense-resample + moving-average + finite-difference pipeline (the known
    "centreline curvature spikes" defect, see CLAUDE.md), which is still
    present and used verbatim when the flag is False.
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
        # horizon predicts the corner simply stopping. They also break
        # CubicSpline outright (repeated arc-length values are not strictly
        # increasing) when this offline copy is driven by SimPlanner's own
        # live-built centreline. Drop them before any arc-length/kappa/psi_ref
        # math runs, so the real end of data becomes this path's actual last
        # point and the existing edge-hold behaviour in kappa_at/psi_ref_at
        # (see their docstrings) takes over from there, holding the last REAL
        # sample instead of a frozen duplicate one. Mirrors the live
        # nmpc/reference.py fix (same commit/investigation).
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
