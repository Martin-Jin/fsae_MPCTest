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

        # Stanley angle — positive in standard convention = left turn = FSDS negative.
        # Damper subtracts k_d·ω: when the car is already swinging left (ω > 0),
        # this reduces δ so the next tick steers less left, preventing overshoot.
        delta = (theta_e
                 + math.atan2(self.k_cte * e, car_speed + self.k_soft)
                 - self.k_d * car_yaw_rate)

        return float(np.clip(-delta / MAX_STEER_RAD, -1.0, 1.0))