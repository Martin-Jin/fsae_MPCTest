"""
Boundary detection: cone-wall mesh planner and ft-fsd trace-sort planner.

Two public planners are provided:

build_path_walls  — active planner.  Connects same-colour cones into a wall
                    mesh, generates candidate midpoints between valid
                    blue-yellow pairs, then chains them with a greedy walk
                    that penalises steps crossing the wall mesh.

build_path_trace  — ft-fsd-inspired planner (reference / fallback).  Sorts
                    each boundary via a same-colour adjacency graph with a
                    cross-track guard, then matches pairs using an oriented
                    ellipse gate with monotonicity.
"""
import math

import numpy as np

from planning.cone_sorting import filter_cones_forward, pair_cones_nn
from planning.path_utils import (
    build_local_path,
    compute_centreline,
    smooth_centreline,
)

# ---------------------------------------------------------------------------
# Cone-wall barrier planner
# ---------------------------------------------------------------------------

_WALL_MAX_DIST      = 7.0      # metres — max dist to link same-colour cones into wall
_WALL_MID_DIST      = 4.0      # metres — max blue-yellow dist for midpoint candidates
_WALL_CROSS_PENALTY = 5000.0   # cost per wall segment crossed by a path step
_WALL_PATH_MAX_STEP = 10.0     # metres — max step between consecutive path midpoints
_WALL_PATH_MAX_WALK = 18       # max midpoints in the constructed path


def _build_wall_segments(
    cones: np.ndarray,
    max_dist: float = _WALL_MAX_DIST,
) -> list[tuple[np.ndarray, np.ndarray]]:
    """Return (p1, p2) segments connecting every same-colour cone pair within max_dist."""
    n = len(cones)
    segs: list[tuple[np.ndarray, np.ndarray]] = []
    for i in range(n):
        for j in range(i + 1, n):
            if float(np.linalg.norm(cones[i] - cones[j])) <= max_dist:
                segs.append((cones[i], cones[j]))
    return segs


def _seg_intersect(
    a1: np.ndarray, a2: np.ndarray,
    b1: np.ndarray, b2: np.ndarray,
) -> bool:
    """True if segment a1→a2 properly intersects segment b1→b2 (endpoints excluded)."""
    d1 = a2 - a1
    d2 = b2 - b1
    denom = float(d1[0] * d2[1] - d1[1] * d2[0])
    if abs(denom) < 1e-10:
        return False
    diff = b1 - a1
    t = float(diff[0] * d2[1] - diff[1] * d2[0]) / denom
    u = float(diff[0] * d1[1] - diff[1] * d1[0]) / denom
    return 0.0 < t < 1.0 and 0.0 < u < 1.0


def _gen_midpoints(
    blue: np.ndarray,
    yellow: np.ndarray,
    car_pos: np.ndarray,
    car_yaw: float,
    max_dist: float = _WALL_MID_DIST,
) -> np.ndarray:
    """
    Return midpoints of valid blue-yellow pairs within max_dist metres.

    Validity filter: the blue cone must be laterally to the LEFT of the yellow
    cone in the car's current frame (lat_blue > lat_yellow).  This eliminates
    midpoints that would land inside a boundary wall, which arise when a
    same-colour cone from an adjacent parallel track is incorrectly paired.
    """
    if len(blue) == 0 or len(yellow) == 0:
        return np.empty((0, 2), dtype=np.float64)

    cos_y, sin_y = math.cos(car_yaw), math.sin(car_yaw)

    def lat(pt: np.ndarray) -> float:
        rel = pt - car_pos
        return float(-rel[0] * sin_y + rel[1] * cos_y)  # positive = left of car

    mids = []
    for b in blue:
        b_lat = lat(b)
        for y in yellow:
            if lat(y) >= b_lat:                          # yellow must be right of blue
                continue
            if float(np.linalg.norm(b - y)) <= max_dist:
                mids.append((b + y) * 0.5)

    return np.array(mids, dtype=np.float64) if mids else np.empty((0, 2), dtype=np.float64)


def _build_wall_path(
    midpoints: np.ndarray,
    car_pos: np.ndarray,
    car_yaw: float,
    wall_segs: list[tuple[np.ndarray, np.ndarray]],
) -> np.ndarray:
    """
    Sort midpoints by forward distance, then chain them greedily.

    Step cost:
        distance  +  _WALL_CROSS_PENALTY × crossings  +  2.0 × heading_change(rad)

    Sorting by forward distance before the walk enforces monotonic forward
    progression.  The heading-change term prefers steps that continue the
    current travel direction, smoothing the raw chain before spline fitting.
    The wall-crossing penalty blocks jumps to adjacent parallel tracks.
    """
    n = len(midpoints)
    if n == 0:
        return np.empty((0, 2), dtype=np.float64)

    cos_y, sin_y = math.cos(car_yaw), math.sin(car_yaw)

    def x_fwd(pt: np.ndarray) -> float:
        rel = pt - car_pos
        return float(rel[0] * cos_y + rel[1] * sin_y)

    fwd      = np.array([x_fwd(m) for m in midpoints])
    sort_idx = np.argsort(fwd)
    midpoints = midpoints[sort_idx]
    fwd       = fwd[sort_idx]

    forward_start = int(np.searchsorted(fwd, 0.3))
    if forward_start >= n:
        return np.empty((0, 2), dtype=np.float64)

    pool_fwd = midpoints[forward_start:]
    seed = forward_start + int(np.argmin(np.linalg.norm(pool_fwd - car_pos, axis=1)))

    ordered = [seed]
    cur_dir = np.array([cos_y, sin_y], dtype=np.float64)

    for _ in range(_WALL_PATH_MAX_WALK - 1):
        curr_idx = ordered[-1]
        curr     = midpoints[curr_idx]
        best_nb, best_cost = None, math.inf

        for idx in range(curr_idx + 1, n):
            cand = midpoints[idx]
            d = float(np.linalg.norm(cand - curr))
            if d > _WALL_PATH_MAX_STEP:
                continue
            step_dir = (cand - curr) / (d + 1e-9)
            angle    = math.acos(float(np.clip(np.dot(cur_dir, step_dir), -1.0, 1.0)))
            n_cross  = sum(1 for (w1, w2) in wall_segs if _seg_intersect(curr, cand, w1, w2))
            cost     = d + _WALL_CROSS_PENALTY * n_cross + 2.0 * angle
            if cost < best_cost:
                best_cost = cost
                best_nb   = idx

        if best_nb is None:
            break
        step    = midpoints[best_nb] - curr
        cur_dir = step / (float(np.linalg.norm(step)) + 1e-9)
        ordered.append(best_nb)

    return midpoints[ordered]


def build_path_walls(
    blue_cones: np.ndarray,
    yellow_cones: np.ndarray,
    car_pos: np.ndarray,
    car_yaw: float,
    max_ahead: float = 25.0,
    max_lateral: float = 10.0,
) -> tuple[np.ndarray | None,
           list[tuple[np.ndarray, np.ndarray]],
           list[tuple[np.ndarray, np.ndarray]],
           np.ndarray]:
    """
    Build a centreline using cone-wall segments as a path barrier.

    Same-colour cones within _WALL_MAX_DIST are connected into a wall mesh.
    Candidate midpoints are generated between valid blue-yellow pairs within
    _WALL_MID_DIST.  A greedy walk picks the cheapest chain through those
    midpoints; every wall-segment crossing adds _WALL_CROSS_PENALTY to the
    step cost, blocking jumps to adjacent parallel tracks.

    Wall cones extend 5 m behind the car (min_ahead = -5) so recently-passed
    cones continue contributing as barriers after leaving the forward window.

    Returns
    -------
    centreline  : (N, 2) smoothed path, or None on failure
    blue_segs   : wall segments from blue cones  (for visualisation)
    yellow_segs : wall segments from yellow cones (for visualisation)
    midpoints   : (M, 2) all candidate midpoints  (for visualisation)
    """
    blue_wall = filter_cones_forward(
        blue_cones, car_pos, car_yaw,
        min_ahead=-5.0, max_ahead=max_ahead, max_lateral=max_lateral,
    )
    yellow_wall = filter_cones_forward(
        yellow_cones, car_pos, car_yaw,
        min_ahead=-5.0, max_ahead=max_ahead, max_lateral=max_lateral,
    )
    blue_fwd = filter_cones_forward(
        blue_cones, car_pos, car_yaw,
        min_ahead=0.5, max_ahead=max_ahead, max_lateral=max_lateral,
    )
    yellow_fwd = filter_cones_forward(
        yellow_cones, car_pos, car_yaw,
        min_ahead=0.5, max_ahead=max_ahead, max_lateral=max_lateral,
    )

    blue_segs   = _build_wall_segments(blue_wall)
    yellow_segs = _build_wall_segments(yellow_wall)
    all_segs    = blue_segs + yellow_segs

    midpoints = _gen_midpoints(blue_fwd, yellow_fwd, car_pos, car_yaw)

    if len(midpoints) < 2:
        cl = build_local_path(blue_cones, yellow_cones, car_pos, car_yaw,
                               max_ahead, max_lateral)
        return cl, blue_segs, yellow_segs, midpoints

    ordered = _build_wall_path(midpoints, car_pos, car_yaw, all_segs)

    if len(ordered) < 2:
        cl = build_local_path(blue_cones, yellow_cones, car_pos, car_yaw,
                               max_ahead, max_lateral)
        return cl, blue_segs, yellow_segs, midpoints

    anchored = np.vstack([car_pos.reshape(1, 2), ordered])
    cl = smooth_centreline(anchored, n_out=max(20, len(ordered) * 5))
    return cl, blue_segs, yellow_segs, midpoints


# ---------------------------------------------------------------------------
# ft-fsd-inspired trace-sort planner (reference / fallback)
# ---------------------------------------------------------------------------

_TS_K_NEIGHBOURS  = 5      # max k-NN per cone (same colour only)
_TS_MAX_EDGE_M    = 6.5    # metres — max edge length in the adjacency graph
_TS_MAX_WALK      = 14     # max cones to chain per boundary
_TS_ELLIPSE_MAJOR = 7.5    # metres — major axis along inward direction
_TS_ELLIPSE_MINOR = 3.0    # metres — minor axis ⊥ inward (≈ min track width)


def _build_same_color_adj(cones: np.ndarray) -> list[list[int]]:
    """
    Build a k-NN same-colour adjacency list restricted to _TS_MAX_EDGE_M.

    Only same-colour cones are in the graph so opposite-colour cones (and
    adjacent-track cones that are too far away) can never be reached through
    graph edges.
    """
    n = len(cones)
    adj: list[list[int]] = [[] for _ in range(n)]
    if n < 2:
        return adj
    diff  = cones[:, None, :] - cones[None, :, :]
    dists = np.linalg.norm(diff, axis=2)
    np.fill_diagonal(dists, np.inf)
    for i in range(n):
        within = np.where(dists[i] <= _TS_MAX_EDGE_M)[0]
        if len(within):
            adj[i] = within[np.argsort(dists[i, within])][:_TS_K_NEIGHBOURS].tolist()
    return adj


def _local_tangent(wall: np.ndarray, idx: int) -> np.ndarray:
    """Unit tangent at wall[idx] via chord between its immediate neighbours."""
    n = len(wall)
    if n < 2:
        return np.array([1.0, 0.0])
    if idx == 0:
        t = wall[1] - wall[0]
    elif idx == n - 1:
        t = wall[-1] - wall[-2]
    else:
        t = wall[idx + 1] - wall[idx - 1]
    length = float(np.linalg.norm(t))
    return t / length if length > 1e-6 else np.array([1.0, 0.0])


def _sort_boundary(
    cones: np.ndarray,
    car_pos: np.ndarray,
    car_yaw: float,
    opposite_cones: np.ndarray,
    is_left: bool,
) -> np.ndarray:
    """
    Order same-colour boundary cones with a greedy walk on the same-colour
    adjacency graph.

    Step cost: angle_cost + 2 × cross_cost

    cross_cost penalises steps where opposite-colour cones appear on the
    geometrically wrong lateral side, which happens when the walk starts
    drifting toward an adjacent track's boundary.
    """
    n = len(cones)
    if n == 0:
        return cones.copy()

    adj   = _build_same_color_adj(cones)
    cos_y = math.cos(car_yaw)
    sin_y = math.sin(car_yaw)

    def x_fwd(pt: np.ndarray) -> float:
        rel = pt - car_pos
        return float(rel[0] * cos_y + rel[1] * sin_y)

    fwd  = np.array([x_fwd(cones[i]) for i in range(n)])
    pool = np.where(fwd > 0.5)[0]
    if not len(pool):
        pool = np.arange(n)
    seed = int(pool[np.argmin(np.linalg.norm(cones[pool] - car_pos, axis=1))])

    ordered = [seed]
    visited = {seed}
    d0      = cones[seed] - car_pos
    cur_dir = d0 / (np.linalg.norm(d0) + 1e-9)

    for _ in range(_TS_MAX_WALK - 1):
        current    = ordered[-1]
        candidates = [nb for nb in adj[current] if nb not in visited]
        if not candidates:
            break

        best_nb, best_score = None, math.inf
        for nb in candidates:
            step     = cones[nb] - cones[current]
            step_len = float(np.linalg.norm(step))
            if step_len < 1e-6:
                continue
            step_dir = step / step_len

            if float(np.dot(step_dir, cur_dir)) < -0.3:
                continue

            angle_cost = math.acos(float(np.clip(np.dot(cur_dir, step_dir), -1.0, 1.0)))

            cross_cost = 0.0
            if len(opposite_cones):
                right_dir = np.array([step_dir[1], -step_dir[0]])
                rel_opp   = opposite_cones - cones[nb]
                near      = rel_opp[np.linalg.norm(rel_opp, axis=1) < 6.0]
                if len(near):
                    lat   = np.dot(near, right_dir)
                    wrong = int(np.sum(lat < 0)) if is_left else int(np.sum(lat > 0))
                    cross_cost = wrong / len(near)

            score = angle_cost + 2.0 * cross_cost
            if score < best_score:
                best_score = score
                best_nb    = nb

        if best_nb is None:
            break

        step    = cones[best_nb] - cones[ordered[-1]]
        cur_dir = step / (np.linalg.norm(step) + 1e-9)
        ordered.append(best_nb)
        visited.add(best_nb)

    return cones[ordered]


def _match_cones_ellipse(
    left_wall: np.ndarray,
    right_wall: np.ndarray,
) -> list[tuple[np.ndarray, np.ndarray]]:
    """
    Match ordered left (blue) to right (yellow) cones with an oriented ellipse
    gate and a strict monotonicity constraint.

    For each left cone: inward direction = rightward perpendicular to the local
    tangent; major axis = _TS_ELLIPSE_MAJOR along inward; minor axis =
    _TS_ELLIPSE_MINOR along-track (≈ min track width).  Only candidates in the
    inward half-space AND inside the ellipse are considered; the closest wins.
    The matched right-cone index must not decrease (monotonicity).
    """
    if not len(left_wall) or not len(right_wall):
        return []

    n_right = len(right_wall)
    pairs   = []
    last_ri = 0

    for li in range(len(left_wall)):
        lc      = left_wall[li]
        tang    = _local_tangent(left_wall, li)
        inward  = np.array([ tang[1], -tang[0]])
        perp_in = np.array([-inward[1], inward[0]])

        best_dist, best_ri = math.inf, None

        for ri in range(last_ri, n_right):
            rel    = right_wall[ri] - lc
            along  = float(np.dot(rel, inward))
            if along <= 0.0:
                continue
            across = float(np.dot(rel, perp_in))
            if (along  / _TS_ELLIPSE_MAJOR) ** 2 + \
               (across / _TS_ELLIPSE_MINOR) ** 2 > 1.0:
                continue

            dist = float(np.linalg.norm(rel))
            if dist < best_dist:
                best_dist = dist
                best_ri   = ri

        if best_ri is not None:
            pairs.append((lc.copy(), right_wall[best_ri].copy()))
            last_ri = best_ri

    return pairs


def build_path_trace(
    blue_cones: np.ndarray,
    yellow_cones: np.ndarray,
    car_pos: np.ndarray,
    car_yaw: float,
    max_ahead: float = 25.0,
    max_lateral: float = 10.0,
) -> np.ndarray | None:
    """
    Build a centreline using the ft-fsd trace-sort approach.

    1. Forward-filter cones to the planning window.
    2. Sort each boundary via a same-colour adjacency graph with an integrated
       cross-track guard.
    3. Match left↔right with an oriented ellipse gate + monotonicity.
    4. Midpoints of matched pairs → cubic spline centreline.

    Falls back to build_local_path() if sorting or matching fails.
    """
    blue_fwd = filter_cones_forward(
        blue_cones, car_pos, car_yaw,
        min_ahead=0.5, max_ahead=max_ahead, max_lateral=max_lateral,
    )
    yellow_fwd = filter_cones_forward(
        yellow_cones, car_pos, car_yaw,
        min_ahead=0.5, max_ahead=max_ahead, max_lateral=max_lateral,
    )

    if len(blue_fwd) < 1 or len(yellow_fwd) < 1:
        return build_local_path(
            blue_cones, yellow_cones, car_pos, car_yaw, max_ahead, max_lateral
        )

    blue_sorted   = _sort_boundary(blue_fwd,   car_pos, car_yaw, yellow_fwd, is_left=True)
    yellow_sorted = _sort_boundary(yellow_fwd, car_pos, car_yaw, blue_fwd,   is_left=False)

    if not len(blue_sorted) or not len(yellow_sorted):
        return build_local_path(
            blue_cones, yellow_cones, car_pos, car_yaw, max_ahead, max_lateral
        )

    pairs = _match_cones_ellipse(blue_sorted, yellow_sorted)

    if not pairs:
        pairs = pair_cones_nn(blue_sorted, yellow_sorted)

    if not pairs:
        return build_local_path(
            blue_cones, yellow_cones, car_pos, car_yaw, max_ahead, max_lateral
        )

    raw      = compute_centreline(pairs)
    anchored = np.vstack([car_pos.reshape(1, 2), raw])
    return smooth_centreline(anchored, n_out=max(20, len(raw) * 5))
