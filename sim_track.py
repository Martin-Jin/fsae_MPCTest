"""
sim_track.py — cone placement and sim-side perception/planning nodes.
Mirrors perception_node.py and planner_node.py for the 2-D simulator.
"""
import math
import numpy as np
from planning.cone_map import ConeMap
from planning.boundary import build_path_walls
from planning.path_utils import build_local_path, get_lookahead_waypoint, compute_desired_speed
import speed_profile as sp

# ── Track geometry constants (FSG / FSUK spec) ──────────────────────────────
CONE_SPACING      = 3.0    # m between cones along each boundary
TRACK_HALF_WIDTH  = 1.75   # m from centreline to each boundary (3.5 m total)
LOOK_AHEAD        = 25.0   # m perception window (matches perception_node.py)
LOOK_WIDE         = 10.0   # m lateral half-width
MIN_AHEAD         = 0.5    # m


def place_cones(path_X, path_Y):
    """
    Place blue (left) and yellow (right) cones at CONE_SPACING intervals
    along the path boundaries. Returns (blue, yellow) as (N,2) float64 arrays.
    """
    px = np.asarray(path_X)
    py = np.asarray(path_Y)

    ds = np.hypot(np.diff(px), np.diff(py))
    arc = np.concatenate([[0.0], np.cumsum(ds)])
    total = arc[-1]

    s_samples = np.arange(0.0, total, CONE_SPACING)
    cx = np.interp(s_samples, arc, px)
    cy = np.interp(s_samples, arc, py)

    # Tangent → left normal
    dx = np.gradient(np.interp(s_samples, arc, px), s_samples)
    dy = np.gradient(np.interp(s_samples, arc, py), s_samples)
    norms = np.hypot(dx, dy)
    norms = np.where(norms < 1e-6, 1.0, norms)
    nx = -dy / norms   # left normal
    ny =  dx / norms

    blue   = np.column_stack([cx + nx * TRACK_HALF_WIDTH,
                               cy + ny * TRACK_HALF_WIDTH])
    yellow = np.column_stack([cx - nx * TRACK_HALF_WIDTH,
                               cy - ny * TRACK_HALF_WIDTH])
    return blue, yellow


class SimPerception:
    """
    Filters the full static cone map to the car's forward FOV.
    Mirrors perception_node.py._publish_visible_cones().
    """
    def __init__(self, blue_all, yellow_all):
        self._blue   = np.asarray(blue_all,   dtype=np.float64)
        self._yellow = np.asarray(yellow_all, dtype=np.float64)

    def visible_cones(self, car_x, car_y, car_yaw):
        cos_y = math.cos(car_yaw)
        sin_y = math.sin(car_yaw)

        def _filter(cones):
            if len(cones) == 0:
                return cones
            rel  = cones - np.array([car_x, car_y])
            x_c  =  rel[:, 0] * cos_y + rel[:, 1] * sin_y
            y_c  = -rel[:, 0] * sin_y + rel[:, 1] * cos_y
            mask = (x_c > MIN_AHEAD) & (x_c < LOOK_AHEAD) & (np.abs(y_c) < LOOK_WIDE)
            return cones[mask]

        return _filter(self._blue), _filter(self._yellow)


class SimPlanner:
    """
    Accumulates cone observations and builds a centreline + speed profile.
    Mirrors planner_node.py._planning_loop() logic.
    """
    def __init__(self, v_max=20.0, v_min=1.5, lookahead_dist=4.0):
        self._cone_map      = ConeMap()
        self._v_max         = v_max
        self._v_min         = v_min
        self._lookahead     = lookahead_dist
        self.centreline     = None
        self.v_profile      = np.array([])

    def update(self, blue_obs, yellow_obs, car_pos, car_yaw):
        """Ingest new cone observations, rebuild path and speed profile."""
        self._cone_map.update(blue_obs, yellow_obs)

        try:
            cl, _, _, _ = build_path_walls(
                self._cone_map.blue, self._cone_map.yellow, car_pos, car_yaw
            )
        except Exception:
            cl = build_local_path(
                self._cone_map.blue, self._cone_map.yellow, car_pos, car_yaw
            )

        self.centreline = cl

        if cl is not None and len(cl) >= 3:
            raw = sp.compute_speed_profile(
                cl[:, 0], cl[:, 1], v_max=self._v_max, v_min=self._v_min
            )
            self.v_profile = sp.smooth_profile(raw, window=9)
        else:
            self.v_profile = np.array([])

    def get_target(self, car_pos, car_yaw):
        """
        Returns (lookahead_xy, desired_speed) or (None, v_min) if no path.
        Mirrors planner_node._planning_loop() target/speed outputs.
        """
        if self.centreline is None or len(self.centreline) == 0:
            return None, self._v_min

        target = get_lookahead_waypoint(
            self.centreline, car_pos, car_yaw, self._lookahead
        )

        if len(self.v_profile) > 0:
            v_des = compute_desired_speed(
                self.centreline, v_max=self._v_max, v_min=self._v_min
            )
        else:
            v_des = self._v_min

        return target, v_des

    def reset(self):
        self._cone_map.reset()
        self.centreline = None
        self.v_profile  = np.array([])

    
def calculate_dynamic_max_steps(path_X, path_Y, dt=0.05, fallback_speed=3.0, buffer=1.5):
    """
    Dynamically calculates the maximum simulation steps required to finish a path.
    Assumes worst-case conservative speed to ensure the car reaches the end.
    """
    px = np.asarray(path_X)
    py = np.asarray(path_Y)

    # Return default if path is uninitialized
    if len(px) < 2:
        return 400

    # Calculate total track arc length (meters)
    ds = np.hypot(np.diff(px), np.diff(py))
    total_length = np.sum(ds)

    # Estimate max time needed: (Length / conservative speed) * buffer
    max_time = (total_length / fallback_speed) * buffer
    max_steps = int(math.ceil(max_time / dt))

    return max_steps