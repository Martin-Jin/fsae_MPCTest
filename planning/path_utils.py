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

# Per-point spline smoothing (splprep's s scales as smooth_per_pt * n_points).
# 0.0 forces an interpolating spline that reproduces every cone-pairing wobble;
# a small positive value makes the spline *approximate* the midpoints, removing
# the left-right kinks on straights.  Tunable live via each planner's `smooth`
# ROS parameter.
DEFAULT_SMOOTH_PER_PT = 0.05   # m² of smoothing budget per input point
# Cap on the point count that feeds the smoothing budget s = smooth_per_pt * n.
# Without a cap, a longer lookahead (more midpoints) inflates s and reshapes the
# *near* field — the local path in front of the car then changes as the far
# horizon grows.  Capping n decouples the near-field spline shape from how many
# far midpoints happen to be in view (see smooth_centreline / issue: lookahead
# consistency).
_SMOOTH_N_CAP = 40
# Weight applied to the prepended car-anchor point so the smoothed path still
# starts at the car instead of bowing away from it (see smooth_centreline).
_ANCHOR_WEIGHT = 100.0


def compute_centreline(pairs):
    """
    Compute midpoints of (left, right) cone pairs.
    Returns (N, 2) float64 array.
    """
    if not pairs:
        return np.empty((0, 2), dtype=np.float64)
    return np.array([(l + r) * 0.5 for l, r in pairs], dtype=np.float64)


def _remove_reversals(pts: np.ndarray, min_cos: float = -0.9,
                      max_removals: int = 3) -> np.ndarray:
    """
    Remove midpoints that cause near-180° direction spikes (dot < min_cos).

    min_cos = -0.9 (~154°) targets genuine back-and-forth spikes from a mispaired
    midpoint, NOT legitimate corners.  A looser threshold risks deleting the
    apex midpoint at tight corners and leaving the spline to cut a wide,
    rounded line across the gap (knocking outer cones).  With approximating
    spline smoothing (see smooth_centreline) absorbing moderate jitter, only
    the true reversal spikes still need explicit removal.  Capped at
    max_removals to prevent cascading elimination of a legitimate corner.
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


def smooth_centreline(waypoints, n_out=None, smooth=None,
                      smooth_per_pt=DEFAULT_SMOOTH_PER_PT, pin_start=True):
    """
    Fit a parametric cubic *approximating* spline through cone-pair midpoints.

    Pipeline:
      1. Drop duplicate consecutive points.
      2. Remove midpoints that cause near-180° direction spikes.
      3. Fit a cubic spline with chord-length parameterisation (arc-length as
         the knot parameter) to prevent backwards tangents at unevenly-spaced
         midpoints.
      4. Resample at n_out uniform parameter values.

    smooth : total splprep smoothing factor s.  When None it is auto-scaled as
             smooth_per_pt * n_points, so the spline *approximates* the midpoints
             (removing left-right kinks) rather than interpolating every one of
             them.  Pass 0.0 to force exact interpolation (legacy behaviour).
    smooth_per_pt : per-point smoothing used when `smooth` is None.
    pin_start : weight the first point heavily (callers prepend the car position)
                so the smoothed path still begins at the car instead of bowing
                away from it — otherwise approximating smoothing would let the
                start drift off the car and inject a near-field steering step.
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

    # Cap the point count feeding the budget so the near-field shape does not
    # drift as the far lookahead (and hence n) grows — see _SMOOTH_N_CAP.
    s = smooth_per_pt * min(n, _SMOOTH_N_CAP) if smooth is None else smooth

    w = None
    if pin_start:
        w = np.ones(n)
        w[0] = _ANCHOR_WEIGHT   # keep the spline pinned to the car anchor

    try:
        tck, _ = splprep([pts[:, 0], pts[:, 1]], u=u_knots, w=w, s=s, k=3)
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


def _resample_forward(path, car_pos, ds, n_samples):
    """
    Resample `path` into `n_samples` points spaced `ds` metres apart, starting
    from the car's projection onto the path and walking forward.

    Projecting onto the path (rather than starting at path[0]) re-anchors a
    previously-published path to the car's *current* position, so an old path
    and a fresh one can be compared sample-for-sample.  Samples past the end of
    the path clamp to its last point.

    Returns an (n_samples, 2) array, or None if the path is degenerate.
    """
    path = np.asarray(path, dtype=np.float64)
    m = len(path)
    if m < 2:
        return None

    # Nearest-segment projection of the car onto the path.
    best_seg, best_t, best_d2 = 0, 0.0, np.inf
    for i in range(m - 1):
        ab  = path[i + 1] - path[i]
        ab2 = float(np.dot(ab, ab))
        if ab2 < 1e-12:
            continue
        t    = max(0.0, min(1.0, float(np.dot(car_pos - path[i], ab)) / ab2))
        proj = path[i] + t * ab
        d2   = float(np.dot(car_pos - proj, car_pos - proj))
        if d2 < best_d2:
            best_d2, best_seg, best_t = d2, i, t

    start   = path[best_seg] + best_t * (path[best_seg + 1] - path[best_seg])
    fwd_pts = np.vstack([start.reshape(1, 2), path[best_seg + 1:]])
    seg     = np.linalg.norm(np.diff(fwd_pts, axis=0), axis=1)
    arc     = np.concatenate([[0.0], np.cumsum(seg)])
    total   = float(arc[-1])
    if total < 1e-6:
        return None

    targets = np.minimum(np.arange(n_samples) * ds, total)
    return np.column_stack([
        np.interp(targets, arc, fwd_pts[:, 0]),
        np.interp(targets, arc, fwd_pts[:, 1]),
    ])


def blend_paths(prev, new, car_pos, alpha=0.4, ds=0.5, horizon=15.0,
                reset_dist=2.0):
    """
    Temporally blend the previously-published path with the freshly-planned one.

    The planner rebuilds the centreline from scratch every pose tick, so
    successive paths can jump — and the controller's target jumps with them,
    producing a jerk in the steering.  Blending each new path toward the previous
    one (an exponential moving average in the map frame) removes those steps
    while still tracking genuine changes.

    Both paths are re-anchored to the car's current position and resampled onto a
    common forward grid (see _resample_forward) so they align sample-for-sample,
    then combined as  out = (1 - alpha)·prev + alpha·new.

      alpha      — blend weight toward the new path (0 < alpha ≤ 1).  1.0 disables
                   blending (pure new path); smaller values are smoother/laggier.
      ds         — resample spacing (m).
      horizon    — how far ahead to blend/publish (m).  Kept ≥ the controller's
                   curvature speed-scan window.
      reset_dist — if the mean sample distance between the two paths exceeds this,
                   the paths have genuinely diverged (a new track section appeared),
                   so snap to the new path instead of lagging toward the old one.

    Returns the blended (K, 2) path.  Falls back to `new` unchanged when there is
    no usable previous path.
    """
    new = np.asarray(new, dtype=np.float64)
    if prev is None or len(new) < 2 or alpha >= 1.0:
        return new

    n = int(horizon / ds) + 1
    r_new  = _resample_forward(new,  car_pos, ds, n)
    r_prev = _resample_forward(prev, car_pos, ds, n)
    if r_new is None or r_prev is None:
        return new

    if float(np.mean(np.linalg.norm(r_new - r_prev, axis=1))) > reset_dist:
        return r_new

    blended = (1.0 - alpha) * r_prev + alpha * r_new
    # Drop trailing near-duplicate samples (path shorter than the horizon clamps
    # its tail to the last point).
    keep = np.concatenate([[True],
                           np.linalg.norm(np.diff(blended, axis=0), axis=1) > 1e-3])
    return blended[keep]


def roll_loop_to_car(
    loop: np.ndarray,
    car_pos: np.ndarray,
    car_yaw: float,
    ahead: float = 35.0,
    wall_segs: list[tuple[np.ndarray, np.ndarray]] | None = None,
    tangent_entry: bool = False,
) -> np.ndarray:
    """
    Reorder a closed loop so the segment ahead of the car comes first.

    Finds the loop point nearest the car, rolls the loop to start there, orients
    it in the car's heading direction, and wraps `ahead` metres of the loop tail
    back onto the end so downstream lookahead / speed scans never run off the
    array at the wrap seam.  car_pos is prepended as the near anchor (matching
    the convention used by build_local_path / build_path_walls).

    When `wall_segs` (same-colour cone-wall segments, see boundary.build_wall_
    segments) is given, the entry point is the nearest loop point whose straight
    connection from the car crosses no wall.  This stops the car latching onto a
    loop point on the far side of a cone wall — e.g. on a skidpad it must reach
    the figure-8 through the opening rather than cutting across the cone rings.

    With `tangent_entry`, the entry point is not the *nearest* point but the
    reachable point ahead whose loop tangent is most aligned with the approach
    direction.  The car therefore merges onto the circle along a tangent — a
    smooth join into the turn — instead of driving at the closest point and
    cornering hard onto it.  This is a mode the caller turns on only while the
    car is still approaching (see the skidpad planner, which drops it once the
    car reaches the crossing); it should be off during normal loop following, or
    it would keep steering the car back toward the tangent target.

    This is a generic closed-loop geometry helper: the skidpad planner uses it to
    follow its known figure-8.  (It formerly lived in a lap-localisation module
    used by a raceline planner that has since been removed.)
    """
    pts = np.asarray(loop, dtype=np.float64)
    n = len(pts)
    if n < 3:
        return pts.copy()

    # Drop the duplicate closing point so rolling does not repeat it.
    if float(np.linalg.norm(pts[0] - pts[-1])) < 1e-6:
        pts = pts[:-1]
        n -= 1

    car = np.asarray(car_pos, dtype=np.float64)
    heading = np.array([math.cos(car_yaw), math.sin(car_yaw)])
    rel = pts - car
    dist = np.linalg.norm(rel, axis=1)
    order = np.argsort(dist)

    reachable = None
    idx = int(order[0])
    if wall_segs:
        from planning.boundary import segment_crosses_walls
        reachable = np.array(
            [not segment_crosses_walls(car, pts[i], wall_segs) for i in range(n)]
        )
        for cand in order:                       # nearest reachable point (via opening)
            if reachable[cand]:
                idx = int(cand)
                break

    # Tangent entry: join the circle where the approach direction is tangent to
    # it (smooth merge) rather than at the nearest point.  The caller enables
    # this only while approaching; near the skidpad crossing a circle arc passes
    # within a lane-width of the entry lane, so a distance test cannot tell an
    # approaching car from a following one — the mode is owned by the planner.
    if tangent_entry:
        with np.errstate(invalid='ignore'):
            direction = rel / dist[:, None]
        # Loop tangent (central difference), oriented toward the car's heading.
        tang = np.roll(pts, -1, axis=0) - np.roll(pts, 1, axis=0)
        tang /= np.linalg.norm(tang, axis=1, keepdims=True) + 1e-12
        tang *= np.sign(tang @ heading)[:, None]
        align = np.einsum('ij,ij->i', direction, tang)     # 1 = tangent, 0 = radial
        eligible = (rel @ heading) > 0.0                    # ahead of the car
        if reachable is not None:
            eligible &= reachable
        if np.any(eligible):
            align = np.where(eligible, align, -np.inf)
            idx = int(np.argmax(align))

    rolled = np.vstack([pts[idx:], pts[:idx]])

    # Orient in travel direction: if the next point is behind the car relative
    # to its heading, the loop is wound the wrong way — reverse it.
    if float(np.dot(rolled[1] - rolled[0], heading)) < 0.0:
        rolled = np.vstack([rolled[:1], rolled[1:][::-1]])

    # Wrap `ahead` metres of the loop back onto the tail for seamless lookahead.
    seg = np.linalg.norm(np.diff(rolled, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    m = int(np.searchsorted(cum, ahead)) + 1
    tail = rolled[: min(m, n)]

    return np.vstack([np.asarray(car_pos, dtype=np.float64).reshape(1, 2), rolled, tail])
