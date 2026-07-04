"""
Path computation, smoothing, speed estimation, and control helpers.

Depends on cone_sorting for the low-level NN fallback path builder.
"""
import math

import numpy as np
from scipy.interpolate import splev, splprep

from planning.cone_sorting import (
    filter_cones_forward,
    pair_cones_nn,
    sort_cones_nn,
)

# Narrow window for direction checking — kept deliberately small so that
# outer-arc cones at corners (which appear on the wrong side at distance)
# are not mistaken for a wrong-way violation.
_DIR_LOOK_AHEAD = 8.0   # metres
_DIR_LOOK_WIDE  = 5.0   # metres lateral half-width


def compute_centreline(pairs):
    """
    Compute midpoints of (left, right) cone pairs.
    Returns (N, 2) float64 array.
    """
    if not pairs:
        return np.empty((0, 2), dtype=np.float64)
    return np.array([(l + r) * 0.5 for l, r in pairs], dtype=np.float64)


def _remove_reversals(pts: np.ndarray, min_cos: float = -0.5,
                      max_removals: int = 3) -> np.ndarray:
    """
    Remove midpoints that cause sharp direction reversals (dot < min_cos ≈ 120°).
    Capped at max_removals to prevent cascading elimination of a legitimate corner.
    """
    for _ in range(max_removals):
        if len(pts) < 3:
            break
        segs = np.diff(pts, axis=0)
        norms = np.linalg.norm(segs, axis=1, keepdims=True)
        norms = np.where(norms < 1e-6, 1.0, norms)
        d = segs / norms
        dots = np.einsum('ij,ij->i', d[:-1], d[1:])
        worst_local = int(np.argmin(dots))
        if dots[worst_local] >= min_cos:
            break
        pts = np.delete(pts, worst_local + 1, axis=0)
    return pts


def smooth_centreline(waypoints, n_out=None, smooth=0.0):
    """
    Fit a parametric cubic spline through cone-pair midpoints and resample.

    Pipeline:
      1. Drop duplicate consecutive points.
      2. Remove midpoints that cause direction reversals > 120°.
      3. Fit a cubic spline with chord-length parameterisation (arc-length as
         the knot parameter) to prevent backwards tangents at unevenly-spaced
         midpoints.
      4. Resample at n_out uniform parameter values.
    """
    pts = np.asarray(waypoints, dtype=np.float64)
    if len(pts) < 2:
        return pts.copy()

    gaps = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    pts = pts[np.concatenate([[True], gaps > 1e-4])]
    pts = _remove_reversals(pts)

    n = len(pts)
    if n_out is None:
        n_out = n * 8

    if n < 4:
        u_in  = np.linspace(0.0, 1.0, n)
        u_out = np.linspace(0.0, 1.0, n_out)
        return np.column_stack([
            np.interp(u_out, u_in, pts[:, 0]),
            np.interp(u_out, u_in, pts[:, 1]),
        ])

    arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))])
    if arc[-1] < 1e-6:
        return pts.copy()
    u_knots = arc / arc[-1]

    try:
        tck, _ = splprep([pts[:, 0], pts[:, 1]], u=u_knots, s=smooth, k=3)
        u_out = np.linspace(0.0, 1.0, n_out)
        x_new, y_new = splev(u_out, tck)
        return np.column_stack([x_new, y_new])
    except Exception:
        u_out = np.linspace(0.0, 1.0, n_out)
        return np.column_stack([
            np.interp(u_out, u_knots, pts[:, 0]),
            np.interp(u_out, u_knots, pts[:, 1]),
        ])


def build_local_path(blue_cones, yellow_cones, car_pos, car_yaw,
                     max_ahead=25.0, max_lateral=10.0):
    """
    Simple fallback planner: NN-sort each boundary then pair midpoints.

    Only forward cones are used so the NN sort is monotonically forward and
    cannot zigzag back across the car.  car_pos is prepended as the near
    anchor so the spline starts at the car's current position.

    Returns a smoothed (N, 2) array or None if there are not enough cones.
    """
    blue_fwd   = filter_cones_forward(blue_cones,   car_pos, car_yaw,
                                       min_ahead=0.5,
                                       max_ahead=max_ahead, max_lateral=max_lateral)
    yellow_fwd = filter_cones_forward(yellow_cones, car_pos, car_yaw,
                                       min_ahead=0.5,
                                       max_ahead=max_ahead, max_lateral=max_lateral)

    if len(blue_fwd) < 1 or len(yellow_fwd) < 1:
        return None

    blue_sorted   = sort_cones_nn(blue_fwd,   start=car_pos)
    yellow_sorted = sort_cones_nn(yellow_fwd, start=car_pos)
    pairs = pair_cones_nn(blue_sorted, yellow_sorted)

    if not pairs:
        return None

    raw = compute_centreline(pairs)
    anchored = np.vstack([car_pos.reshape(1, 2), raw])
    return smooth_centreline(anchored, n_out=max(20, len(raw) * 5))


def get_lookahead_waypoint(waypoints, car_pos, car_yaw,
                            lookahead_dist=5.0, min_ahead=1.0):
    """
    Project the car onto the nearest path segment, then walk lookahead_dist
    forward along the path from that projection.

    min_ahead: the returned target is guaranteed to have at least this many
    metres of forward (x_car) component.  If the arc-length walk ends up with
    less, the lookahead is extended until the constraint is satisfied or the
    path is exhausted.

    Returns a (2,) array, or None if waypoints is empty.
    """
    n = len(waypoints)
    if n == 0:
        return None
    if n == 1:
        return waypoints[0].copy()

    cos_y = math.cos(car_yaw)
    sin_y = math.sin(car_yaw)

    def _x_fwd(pt):
        rel = pt - car_pos
        return rel[0] * cos_y + rel[1] * sin_y

    def _walk(dist):
        best_seg = 0
        best_t   = 0.0
        best_d2  = np.inf

        for i in range(n - 1):
            ab  = waypoints[i + 1] - waypoints[i]
            ab2 = float(np.dot(ab, ab))
            if ab2 < 1e-12:
                continue
            t = float(np.dot(car_pos - waypoints[i], ab)) / ab2
            t = max(0.0, min(1.0, t))
            proj = waypoints[i] + t * ab
            d2   = float(np.dot(car_pos - proj, car_pos - proj))
            if d2 < best_d2:
                best_d2  = d2
                best_seg = i
                best_t   = t

        remaining = dist
        for i in range(best_seg, n - 1):
            a       = waypoints[i]
            b       = waypoints[i + 1]
            seg     = b - a
            seg_len = float(np.linalg.norm(seg))
            if seg_len < 1e-6:
                continue
            t0        = best_t if i == best_seg else 0.0
            available = (1.0 - t0) * seg_len
            if remaining <= available:
                return a + (t0 + remaining / seg_len) * seg
            remaining -= available

        n_back = min(max(1, n // 8), n - 1)
        last_dir = waypoints[-1] - waypoints[-1 - n_back]
        last_len = float(np.linalg.norm(last_dir))
        if last_len > 1e-6:
            return waypoints[-1] + (remaining / last_len) * last_dir
        return waypoints[-1].copy()

    target = _walk(lookahead_dist)

    step = 1.0
    while _x_fwd(target) < min_ahead and lookahead_dist < 50.0:
        lookahead_dist += step
        target = _walk(lookahead_dist)

    if _x_fwd(target) < min_ahead:
        fwds = np.array([_x_fwd(wp) for wp in waypoints])
        return waypoints[int(np.argmax(fwds))].copy()

    return target


# def compute_desired_speed(waypoints, v_max=5.0, v_min=1.5, a_lat_max=4.0,
#                            scan_start=1.5, scan_end=14.0, step=2.0,
#                            safety: float = 1):
#     """
#     Estimate peak curvature over the next scan_end metres and return a safe speed.

#     Short-path cap: when the visible path is shorter than scan_end, v_max is
#     scaled down proportionally (can't see what's coming → don't accelerate).

#     Safety multiplier: curvature-derived speed is multiplied by `safety` to
#     compensate for spline smoothing underestimating true curvature at corners.

#     waypoints[0] is assumed to be the car's current position.
#     """
#     n = len(waypoints)
#     if n < 3:
#         return float(v_max)

#     segs  = np.linalg.norm(np.diff(waypoints, axis=0), axis=1)
#     arc   = np.concatenate([[0.0], np.cumsum(segs)])
#     total = arc[-1]

#     v_max_eff = max(v_min, v_max * min(1.0, total / scan_end))

#     if total < scan_start + step:
#         return float(v_max_eff)

#     sample_arcs = np.arange(scan_start, min(scan_end, total), step)
#     if len(sample_arcs) < 3:
#         return float(v_max_eff)

#     sx  = np.interp(sample_arcs, arc, waypoints[:, 0])
#     sy  = np.interp(sample_arcs, arc, waypoints[:, 1])
#     pts = np.column_stack([sx, sy])

#     max_kappa = 0.0
#     for i in range(1, len(pts) - 1):
#         p1, p2, p3 = pts[i - 1], pts[i], pts[i + 1]
#         d12 = float(np.linalg.norm(p2 - p1))
#         d23 = float(np.linalg.norm(p3 - p2))
#         d31 = float(np.linalg.norm(p1 - p3))
#         denom = d12 * d23 * d31
#         if denom < 1e-9:
#             continue
#         v1    = p2 - p1
#         v2    = p3 - p1
#         cross = abs(float(v1[0] * v2[1] - v1[1] * v2[0]))
#         kappa = 2.0 * cross / denom
#         if kappa > max_kappa:
#             max_kappa = kappa

#     if max_kappa < 1e-4:
#         return float(v_max_eff)

#     v_target = safety * math.sqrt(a_lat_max / max_kappa)
#     return float(max(v_min, min(v_max_eff, v_target)))
