import math

import numpy as np

# FSDS: max steering angle 25 degrees per ros-bridge.md
MAX_STEER_RAD = math.radians(25.0)


def _heading_error(car_pos, car_yaw, target_global) -> float:
    """Return heading error in radians: positive when target is left of car."""
    dx = target_global[0] - car_pos[0]
    dy = target_global[1] - car_pos[1]
    cos_y, sin_y = math.cos(car_yaw), math.sin(car_yaw)
    x_car =  dx * cos_y + dy * sin_y
    y_car = -dx * sin_y + dy * cos_y
    return math.atan2(y_car, x_car)


def compute_steering(car_pos, car_yaw, target_global) -> float:
    """Pure-proportional steering (legacy helper, kept for reference)."""
    return float(np.clip(-_heading_error(car_pos, car_yaw, target_global)
                         / MAX_STEER_RAD, -1.0, 1.0))


class StanleyController:
    """
    Stanley path-tracking steering controller (Thrun et al., DARPA 2005).

    δ = θ_e + arctan(k_cte · e / (v + k_soft))

    θ_e — heading error: path tangent angle minus car yaw (rad).
          Positive when path turns left relative to the car.
    e   — cross-track error: signed lateral distance from the front axle
          to the nearest path point (m), positive when the axle is to
          the RIGHT of the path.
    v   — car speed (m/s); k_soft prevents division by zero at standstill.

    Sign convention (FSDS ENU: x forward, y left):
      output > 0  → steer right
      output < 0  → steer left
      output ∈ [-1, 1]

    Tuning:
      k_cte     — cross-track gain.  Higher values correct lateral error faster
                  but cause oscillation on a high-speed straight.
      k_soft    — speed softening (m/s).  Set to ~walking speed so the CTE
                  term doesn't saturate the steering at low speeds.
      k_d       — yaw-rate damper gain.  Subtracts k_d·ω from the Stanley
                  angle before normalising, opposing rapid heading changes.
                  This is the primary fix for left-right sway: the CTE term
                  alone has no memory of how fast the heading is already
                  changing, so it overshoots; k_d·ω counters each swing.
      wheelbase — distance from rear to front axle (m).  Used to project
                  the control point to the front axle, which is where Stanley
                  measures cross-track error.
    """

    def __init__(
        self,
        k_cte: float = 1.0,
        k_soft: float = 1.0,
        k_d: float = 0.1,
        wheelbase: float = 1.5,
    ):
        self.k_cte     = k_cte
        self.k_soft    = k_soft
        self.k_d       = k_d
        self.wheelbase = wheelbase

    def compute(
        self,
        path: np.ndarray,
        car_pos: np.ndarray,
        car_yaw: float,
        car_speed: float,
        car_yaw_rate: float = 0.0,
    ) -> float:
        """
        Return a steering command in [-1, 1].

        path         — (N, 2) array of waypoints in map frame (must have N ≥ 2)
        car_pos      — (2,) car position in map frame
        car_yaw      — car heading in radians
        car_speed    — car speed in m/s
        car_yaw_rate — yaw rate in rad/s (positive = left / CCW); used by the
                       damper term to oppose rapid heading changes
        """
        if len(path) < 2:
            return 0.0

        # Project control point to front axle
        fa = car_pos + self.wheelbase * np.array([math.cos(car_yaw), math.sin(car_yaw)])

        # Nearest waypoint index to front axle
        idx = int(np.argmin(np.linalg.norm(path - fa, axis=1)))

        # Unit tangent in direction of travel at that waypoint
        if idx < len(path) - 1:
            seg = path[idx + 1] - path[idx]
        else:
            seg = path[idx] - path[idx - 1]
        seg_len = float(np.linalg.norm(seg))
        if seg_len < 1e-6:
            return 0.0
        t = seg / seg_len

        # Heading error: path_yaw - car_yaw, normalised to (-π, π)
        path_yaw = math.atan2(t[1], t[0])
        theta_e = math.atan2(
            math.sin(path_yaw - car_yaw),
            math.cos(path_yaw - car_yaw),
        )

        # Cross-track error: right-normal of path, positive = axle right of path
        right_n = np.array([t[1], -t[0]])   # 90° CW rotation of tangent
        e = float(np.dot(fa - path[idx], right_n))

        # Stanley angle — positive = left turn (standard convention).  Damper
        # subtracts k_d·ω: when the car is already swinging left (ω > 0), this
        # reduces δ so the next tick steers less left, preventing overshoot.
        delta = (theta_e
                 + math.atan2(self.k_cte * e, car_speed + self.k_soft)
                 - self.k_d * car_yaw_rate)

        # Return the steering ANGLE in radians (positive = left), clamped to the
        # physical limit.  FSDS normalisation (+1 = right) is done by fsds_bridge.
        return float(np.clip(delta, -MAX_STEER_RAD, MAX_STEER_RAD))


def curvature_speed(waypoints, v_max=15.0, v_min=1.5, a_lat_max=4.0,
                    scan_start=1.5, scan_end=24.0, step=2.0, safety=1.0):
    """
    Curvature-limited target speed over the next scan_end metres of the path.

    Ported from the planner's speed logic so the controller can set cmd_vel.speed
    without a cross-package import.  v_target = safety·√(a_lat_max / κ_peak), with a
    short-path cap that scales v_max down when the visible path is shorter than
    scan_end.  waypoints[0] is assumed to be the car's current position.

    scan_end=24 m (was 14 m): a tight hairpin (~2 m radius, v_target ~2.7 m/s)
    approached at v_max needs ~24 m to brake for at a realistic achieved
    deceleration, not the raw a_max_brake limit. A shorter scan sees the
    corner too late for the car to shed enough speed, saturating steering and
    spinning out at corner entry (observed on the live fsae_planning stack;
    kept in sync with the planner's own lookahead there).
    """
    n = len(waypoints)
    if n < 3:
        return float(v_max)

    segs  = np.linalg.norm(np.diff(waypoints, axis=0), axis=1)
    arc   = np.concatenate([[0.0], np.cumsum(segs)])
    total = arc[-1]

    v_max_eff = max(v_min, v_max * min(1.0, total / scan_end))
    if total < scan_start + step:
        return float(v_max_eff)

    hi = min(scan_end, total)
    # The planner re-fits the centreline every frame, so on a straight the
    # published path carries a few cm of per-point lateral wiggle.  Computing the
    # MAX Menger curvature over raw ~2 m triples turns that noise into a large
    # spurious kappa (true kappa is ~0 on a straight, so the max is pure noise),
    # collapsing v_target and making it oscillate frame-to-frame — the "rapid
    # accel/decel" on straights.  Fix at the source: densely resample the scan
    # window and moving-average denoise it before measuring curvature.  A real
    # corner is a sustained bend that survives the ~5 m smoothing; only the
    # cm-scale wiggle is removed (this also stops noise from over-slowing corners).
    pts = None
    dense = np.arange(scan_start, hi, 1.0)
    if len(dense) >= 7:                       # room to smooth and still leave >=3 triples
        dx = np.interp(dense, arc, waypoints[:, 0])
        dy = np.interp(dense, arc, waypoints[:, 1])
        w  = min(5, len(dense) - 4)           # 'valid' conv keeps len-w+1 >= 3 points
        ker = np.ones(w) / w
        sx = np.convolve(dx, ker, mode='valid')
        sy = np.convolve(dy, ker, mode='valid')
        pts = np.column_stack([sx, sy])[::2]  # back to ~2 m spacing for the triples
    if pts is None or len(pts) < 3:
        # Short scan window: no headroom to denoise — fall back to coarse sampling.
        sample_arcs = np.arange(scan_start, hi, step)
        if len(sample_arcs) < 3:
            return float(v_max_eff)
        sx  = np.interp(sample_arcs, arc, waypoints[:, 0])
        sy  = np.interp(sample_arcs, arc, waypoints[:, 1])
        pts = np.column_stack([sx, sy])

    max_kappa = 0.0
    for i in range(1, len(pts) - 1):
        p1, p2, p3 = pts[i - 1], pts[i], pts[i + 1]
        d12 = float(np.linalg.norm(p2 - p1))
        d23 = float(np.linalg.norm(p3 - p2))
        d31 = float(np.linalg.norm(p1 - p3))
        denom = d12 * d23 * d31
        if denom < 1e-9:
            continue
        v1    = p2 - p1
        v2    = p3 - p1
        cross = abs(float(v1[0] * v2[1] - v1[1] * v2[0]))
        kappa = 2.0 * cross / denom
        if kappa > max_kappa:
            max_kappa = kappa

    if max_kappa < 1e-4:
        return float(v_max_eff)

    v_target = safety * math.sqrt(a_lat_max / max_kappa)
    return float(max(v_min, min(v_max_eff, v_target)))
