"""
Boundary detection: cone-wall mesh centreline planner.

build_path_walls  — the planner.  Connects same-colour cones into a wall mesh,
                    generates one midpoint per anchor-side cone via exclusive
                    nearest-neighbour matching, then chains them with a greedy
                    walk that penalises steps crossing the wall mesh.  The chain
                    is clamped to a fixed arc-length horizon and smoothed into a
                    centreline.
"""
import math

import numpy as np

from fsae_planning.cone_sorting import filter_cones_window
from fsae_planning.path_utils import (
    build_local_path,
    DEFAULT_SMOOTH_PER_PT,
    smooth_centreline,
)

# ---------------------------------------------------------------------------
# Cone-wall barrier planner
# ---------------------------------------------------------------------------

_WALL_MAX_DIST      = 7.0      # metres — max dist to link same-colour cones into wall
_WALL_MID_DIST      = 4.0      # metres — max blue-yellow dist for midpoint candidates
_WALL_CROSS_PENALTY = 100000.0   # cost per wall segment crossed by a path step
_WALL_PATH_MAX_STEP = 10.0     # metres — max step between consecutive path midpoints
_WALL_PATH_MAX_WALK = 18       # max midpoints in the constructed path
# Softest per-step turn the walk will accept, as cos(max turn).  The old walk
# used a hard 0.0 (a 90° per-step ceiling): at a tight hairpin every next
# midpoint sits >90° off the current travel direction, so the walk stalled and
# the path truncated into the corner (car then drove straight off).  -0.5 (~120°)
# lets the chain follow a genuine hairpin; the angle cost + wall-cross penalty
# still keep it from doubling back or hopping to a parallel track.
_WALL_MAX_TURN_COS  = -0.5
# Default arc-length horizon (m) the published centreline is clamped to before
# smoothing.  Far midpoints beyond this are dropped so the near path in front of
# the car does not change as the lookahead grows, and the global spline is not
# dragged by distant apex points.  Kept >= the controller's ~24 m speed scan
# (control_utils.curvature_speed's scan_end) — a tight hairpin (~2 m radius,
# v_target ~2.7 m/s) approached at v_max=15 m/s needs ~24 m to brake for at a
# realistic achieved deceleration (~4.5 m/s2, well under the 9 m/s2 hard limit
# once the MPC's own speed-request low-pass and rate limits are accounted for);
# the previous 15 m horizon only revealed such a corner a couple of car-lengths
# before the car needed to already be nearly stopped, causing steering
# saturation and a spin-out.
_WALL_PLAN_HORIZON  = 25.0


def build_wall_segments(
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


def segment_crosses_walls(
    p1: np.ndarray,
    p2: np.ndarray,
    wall_segs: list[tuple[np.ndarray, np.ndarray]],
) -> bool:
    """True if the segment p1→p2 crosses any cone-wall segment."""
    return any(_seg_intersect(p1, p2, w1, w2) for (w1, w2) in wall_segs)


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
    Return midpoints from an exclusive nearest-neighbour match between blue
    and yellow cones within max_dist metres.

    The denser side (more cones in view) is used as the anchor: every anchor
    cone claims its single nearest still-unclaimed opposite-colour cone, so no
    opposite-colour cone can be shared between multiple midpoints.  This
    matters most in corners, where the tighter-radius boundary has cones
    closer together than the outer one — under plain all-pairs distance
    matching, one outer cone falls within range of several inner cones and
    fans out into several midpoints pulling in different directions, which
    stays jagged even after spline smoothing.  Anchor cones are matched in
    forward order (nearest to the car first) so exclusivity resolves in
    favour of the more immediately relevant matches.

    Validity filter: the blue cone must be to the LEFT of the yellow cone
    relative to the LOCAL track direction (not the car's instantaneous heading).
    This eliminates midpoints that would land inside a boundary wall, which arise
    when a same-colour cone from an adjacent parallel track is incorrectly paired.

    Using the local track direction rather than the car heading is what keeps
    CORNER midpoints: around a bend the boundary rotates away from the car's
    current heading, so a car-frame left/right test wrongly rejected the very
    apex cones and left the spline to cut a wide line across the gap.

    The local direction is estimated per anchor cone from its two SPATIALLY
    nearest same-colour neighbours (not its neighbours in the forward-sorted
    order).  This matters at tight corners with a narrow infield (hairpins,
    chicanes): the forward-sort mixes cones from the two legs of the bend, so a
    sort-order neighbour can be a cone on the OPPOSITE leg several metres across
    the infield.  The tangent then points across the track instead of along it,
    the left/right cross-product test inverts, and cross-infield cone pairs are
    wrongly accepted — producing midpoints that cut diagonally across the track
    (car drives off the map at the apex).  Spatially-nearest neighbours are
    always on the same leg, so the tangent stays along-track and the test holds.
    """
    if len(blue) == 0 or len(yellow) == 0:
        return np.empty((0, 2), dtype=np.float64)

    cos_y, sin_y = math.cos(car_yaw), math.sin(car_yaw)

    def fwd(pt: np.ndarray) -> float:
        rel = pt - car_pos
        return float(rel[0] * cos_y + rel[1] * sin_y)

    blue_is_anchor = len(blue) >= len(yellow)
    anchor, other  = (blue, yellow) if blue_is_anchor else (yellow, blue)

    anchor_order = np.argsort([fwd(pt) for pt in anchor])
    n_anchor     = len(anchor_order)
    car_dir      = np.array([cos_y, sin_y], dtype=np.float64)
    claimed      = np.zeros(len(other), dtype=bool)

    def local_dir(idx: int) -> np.ndarray:
        """
        Along-track direction at anchor cone `idx`, from its two spatially
        nearest same-colour neighbours (within _WALL_MAX_DIST).  The chord
        between them gives the tangent line; the sign is oriented so it points
        forward (increasing fwd projection), falling back to the car heading
        when the cone is isolated.
        """
        if n_anchor < 2:
            return car_dir
        a = anchor[idx]
        d2 = np.linalg.norm(anchor - a, axis=1)
        d2[idx] = np.inf
        near = [k for k in np.argsort(d2)[:2] if d2[k] <= _WALL_MAX_DIST]
        if not near:
            return car_dir
        if len(near) == 1:
            d = anchor[near[0]] - a
            if float(d[0] * cos_y + d[1] * sin_y) < 0.0:
                d = -d
        else:
            k0, k1 = near
            if fwd(anchor[k0]) > fwd(anchor[k1]):
                k0, k1 = k1, k0
            d = anchor[k1] - anchor[k0]
        dn = float(np.linalg.norm(d))
        return d / dn if dn > 1e-6 else car_dir

    mids = []
    for pos, idx in enumerate(anchor_order):
        a  = anchor[idx]
        ld = local_dir(idx)

        # Signed cross product of the local track direction with (cone - anchor):
        # > 0 → cone is left of the track, < 0 → right.
        rel   = other - a
        cross = ld[0] * rel[:, 1] - ld[1] * rel[:, 0]
        if blue_is_anchor:
            valid = (~claimed) & (cross < 0.0)   # yellow must be right of blue
        else:
            valid = (~claimed) & (cross > 0.0)   # blue must be left of yellow

        cand_idx = np.where(valid)[0]
        if len(cand_idx) == 0:
            continue

        dists      = np.linalg.norm(other[cand_idx] - a, axis=1)
        best_local = int(np.argmin(dists))
        if dists[best_local] > max_dist:
            continue

        best_idx = cand_idx[best_local]
        claimed[best_idx] = True
        mids.append((a + other[best_idx]) * 0.5)

    return np.array(mids, dtype=np.float64) if mids else np.empty((0, 2), dtype=np.float64)


def _build_wall_path(
    midpoints: np.ndarray,
    car_pos: np.ndarray,
    car_yaw: float,
    wall_segs: list[tuple[np.ndarray, np.ndarray]],
) -> np.ndarray:
    """
    Chain midpoints into a path by following the local track direction.

    Step cost:
        distance  +  _WALL_CROSS_PENALTY × crossings  +  2.0 × heading_change(rad)

    The walk seeds at the nearest midpoint ahead of the car and, at each step,
    picks the cheapest unvisited midpoint whose bearing is within _WALL_MAX_TURN_COS
    of the current travel direction, where cur_dir rotates as the walk turns.

    This is the key to not truncating at corners: an earlier version sorted
    midpoints by forward distance in the *car's fixed heading frame* and only
    chained to ever-greater forward distance, so as soon as the track curved away
    from the initial heading the chain stalled (the apex/exit midpoints have
    smaller heading-frame forward distance) — producing a stub path into corners.
    Following cur_dir instead lets the chain turn with the track.

    The per-step gate is a relaxed turn allowance (_WALL_MAX_TURN_COS, ~120°) not
    a hard 90° forward test: at an extreme bend the next along-track midpoint can
    sit well past 90° from the current heading, and a 90° ceiling dropped it and
    truncated the path into the corner.  The angle cost still favours straighter
    chains and the wall-crossing penalty still blocks jumps to adjacent parallel
    tracks, so relaxing the gate wraps hairpins without doubling back.
    """
    n = len(midpoints)
    if n == 0:
        return np.empty((0, 2), dtype=np.float64)

    cos_y, sin_y = math.cos(car_yaw), math.sin(car_yaw)
    heading = np.array([cos_y, sin_y], dtype=np.float64)

    # Seed: nearest midpoint that isn't clearly behind the car.
    #
    # A hard `fwd > 0.3` gate discards a midpoint outright the instant its
    # heading-frame forward projection dips at or below the cutoff, which
    # happens as the car's heading rotates through a corner even while the
    # midpoint is still the closest point on the track and hasn't moved.
    # Losing the nearest midpoint then forces the seed to jump to the next
    # surviving one, which in a corner (where midpoints are sparser) can be
    # several metres further away — a discontinuous jump in the published
    # path's own near-field anchor right as the car needs to react to it.
    #
    # Fix: reject only midpoints clearly behind the car (a small negative
    # margin, not a positive one), so a near midpoint stays eligible through
    # the heading range where the old cutoff discarded it, and among eligible
    # points always take the nearest. Falls back to the nearest midpoint
    # overall if literally everything is behind (a sharp bend right at the car).
    fwd = (midpoints - car_pos) @ heading
    forward_idx = np.where(fwd > -0.5)[0]
    if len(forward_idx) == 0:
        forward_idx = np.arange(n)
    seed = int(forward_idx[np.argmin(np.linalg.norm(midpoints[forward_idx] - car_pos, axis=1))])

    ordered  = [seed]
    visited  = {seed}
    cur_dir  = heading.copy()

    for _ in range(_WALL_PATH_MAX_WALK - 1):
        curr = midpoints[ordered[-1]]
        best_nb, best_cost = None, math.inf

        for idx in range(n):
            if idx in visited:
                continue
            cand = midpoints[idx]
            step = cand - curr
            d = float(np.linalg.norm(step))
            if d < 1e-6 or d > _WALL_PATH_MAX_STEP:
                continue
            step_dir = step / d
            fwd_dot  = float(np.dot(cur_dir, step_dir))
            if fwd_dot <= _WALL_MAX_TURN_COS:   # reject only sharp doublings-back
                continue
            angle    = math.acos(max(-1.0, min(1.0, fwd_dot)))
            n_cross  = sum(1 for (w1, w2) in wall_segs if _seg_intersect(curr, cand, w1, w2))
            cost     = d + _WALL_CROSS_PENALTY * n_cross + 2.0 * angle
            if cost < best_cost:
                best_cost = cost
                best_nb   = idx

        if best_nb is None:
            break
        cur_dir = (midpoints[best_nb] - curr)
        cur_dir = cur_dir / (float(np.linalg.norm(cur_dir)) + 1e-9)
        visited.add(best_nb)
        ordered.append(best_nb)

    return midpoints[ordered]


def build_path_walls(
    blue_cones: np.ndarray,
    yellow_cones: np.ndarray,
    car_pos: np.ndarray,
    car_yaw: float,
    max_ahead: float = 25.0,
    max_lateral: float = 10.0,
    smooth_per_pt: float = DEFAULT_SMOOTH_PER_PT,
    look_radius: float = 25.0,   # kept >= _WALL_PLAN_HORIZON — see that constant's comment
    plan_horizon: float = _WALL_PLAN_HORIZON,
) -> tuple[np.ndarray | None,
           list[tuple[np.ndarray, np.ndarray]],
           list[tuple[np.ndarray, np.ndarray]],
           np.ndarray]:
    """
    Build a centreline using cone-wall segments as a path barrier.

    Same-colour cones within _WALL_MAX_DIST are connected into a wall mesh.
    Candidate midpoints are generated by exclusively matching each cone on the
    denser boundary to its nearest unclaimed opposite-colour cone within
    _WALL_MID_DIST (see _gen_midpoints).  A greedy walk picks the cheapest
    chain through those midpoints; every wall-segment crossing adds
    _WALL_CROSS_PENALTY to the step cost, blocking jumps to adjacent parallel
    tracks.

    Wall cones extend 5 m behind the car (min_ahead = -5) so recently-passed
    cones continue contributing as barriers after leaving the forward window.

    Returns
    -------
    centreline  : (N, 2) smoothed path, or None on failure
    blue_segs   : wall segments from blue cones  (for visualisation)
    yellow_segs : wall segments from yellow cones (for visualisation)
    midpoints   : (M, 2) all candidate midpoints  (for visualisation)
    """
    # Radius (omni) OR forward box.  The radius keeps the cones around a bend
    # (which a heading-aligned box drops as the track curves away), so the path
    # no longer truncates at corners; the box keeps long-range preview straight
    # ahead.  Walls extend a little further (look_radius + 4) so barrier segments
    # stay complete slightly beyond the midpoint horizon.
    blue_wall = filter_cones_window(
        blue_cones, car_pos, car_yaw, radius=look_radius + 4.0,
        min_ahead=-5.0, max_ahead=max_ahead, max_lateral=max_lateral,
    )
    yellow_wall = filter_cones_window(
        yellow_cones, car_pos, car_yaw, radius=look_radius + 4.0,
        min_ahead=-5.0, max_ahead=max_ahead, max_lateral=max_lateral,
    )
    blue_fwd = filter_cones_window(
        blue_cones, car_pos, car_yaw, radius=look_radius,
        min_ahead=0.5, max_ahead=max_ahead, max_lateral=max_lateral,
    )
    yellow_fwd = filter_cones_window(
        yellow_cones, car_pos, car_yaw, radius=look_radius,
        min_ahead=0.5, max_ahead=max_ahead, max_lateral=max_lateral,
    )

    blue_segs   = build_wall_segments(blue_wall)
    yellow_segs = build_wall_segments(yellow_wall)
    all_segs    = blue_segs + yellow_segs

    midpoints = _gen_midpoints(blue_fwd, yellow_fwd, car_pos, car_yaw)

    if len(midpoints) < 1:
        cl = build_local_path(blue_cones, yellow_cones, car_pos, car_yaw,
                               max_ahead, max_lateral)
        return cl, blue_segs, yellow_segs, midpoints

    ordered = _build_wall_path(midpoints, car_pos, car_yaw, all_segs)

    # A single ordered midpoint is enough: anchored to the car below it becomes a
    # short but ON-TRACK forward segment, which the controller happily extrapolates.
    # Only fall back to build_local_path when the walk is genuinely empty.  The old
    # `< 2` gate was the corner-cut culprit: at a loop's pinch (the descending leg
    # passing close to the returning leg) most midpoints sit BEHIND the car on the
    # return leg, so the forward-seeded walk yields exactly one midpoint — and the
    # `< 2` gate then discarded it for build_local_path, whose naive nearest-cone
    # pairing links straight across the pinch (a ~3 m cross-infield chord that
    # mowed down the apex cones).  Keeping the one-point walk avoids that entirely.
    if len(ordered) < 1:
        cl = build_local_path(blue_cones, yellow_cones, car_pos, car_yaw,
                               max_ahead, max_lateral)
        return cl, blue_segs, yellow_segs, midpoints

    # Clamp the chain to a fixed arc-length horizon (measured from the car through
    # the midpoints) before smoothing.  This makes the near path in front of the
    # car independent of how far the lookahead reaches — extra far midpoints no
    # longer extend the chain or drag the global spline — and keeps the corner
    # line on the true centreline instead of letting distant apex points pull it
    # inward.  At least three points are retained so the spline stays well-posed.
    anchored = np.vstack([car_pos.reshape(1, 2), ordered])
    seg = np.linalg.norm(np.diff(anchored, axis=0), axis=1)
    arc = np.concatenate([[0.0], np.cumsum(seg)])
    keep = arc <= plan_horizon
    if int(keep.sum()) < 3:
        keep[:min(3, len(anchored))] = True
    anchored = anchored[keep]

    cl = smooth_centreline(anchored, n_out=max(20, (len(anchored) - 1) * 5),
                           smooth_per_pt=smooth_per_pt)
    return cl, blue_segs, yellow_segs, midpoints
