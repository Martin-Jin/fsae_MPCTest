"""
sim/perception.py — Cone Placement and Sim-Side Perception

PURPOSE
-------
The simulator-side counterpart of fsae_planning's sim_perception.py ROS 2
node. This allows the offline 2-D simulator to feed the same cone-visibility
pipeline that runs on the real vehicle into planner.py's SimPlanner.

  1. place_cones()    — Generate a static cone map from a path (track layout)
  2. SimPerception    — Filter visible cones from the car's current position/FOV

RELATIONSHIP TO ROS2 STACK
---------------------------
  place_cones()   →  static track layout (not in ROS2; the real track has real cones)
  SimPerception   →  mirrors sim_perception.py's SimPerception._visible() + _publish()

USED BY
-------
  gui/simulation.py    — calls place_cones() after path creation; instantiates
                     SimPerception inside simulate_closed_loop().
  tuner/offline_tuner.py — instantiates SimPerception inside run_headless_rollout().

DOES NOT USE (directly)
-----------------------
  model/vehicle_physics.py, model/bicycle_model.py, controller/lmpc/solve.py, tuner/performance_stats.py, sim/speed_profile.py
"""

import math
import numpy as np

# NOTE: settings.py imports TRACK_HALF_WIDTH from this module at module
# scope, so `from settings import PLANNER_*` cannot be a top-level import
# here without a circular import.

# ── Track geometry constants (FSG / FSUK specification) ─────────────────────
CONE_SPACING      = 3.0    # Distance between cones along each boundary (m)
TRACK_HALF_WIDTH  = 1.75   # Distance from centreline to boundary cones (m) → 3.5 m total width
LOOK_AHEAD        = 25.0   # Perception forward distance: cones visible ahead (m)
LOOK_WIDE         = 10.0   # Perception lateral half-width: cones visible to each side (m)
MIN_AHEAD         = 0.5    # Minimum forward distance to include a cone (m); filters behind-car cones


def place_cones(path_X, path_Y):
    """
    Generate blue (left) and yellow (right) boundary cones at regular intervals
    along the path, offset laterally by TRACK_HALF_WIDTH on each side.

    The lateral direction at each sample point is determined by rotating the
    path tangent 90° counterclockwise (left normal) or clockwise (right normal).
    This matches FS convention: blue cones mark the left boundary, yellow the right.

    Parameters
    ----------
    path_X : array-like, shape (n,)
        X coordinates of the path centreline (m).
    path_Y : array-like, shape (n,)
        Y coordinates of the path centreline (m).

    Returns
    -------
    blue : np.ndarray, shape (m, 2)
        [X, Y] positions of blue (left) boundary cones.
    yellow : np.ndarray, shape (m, 2)
        [X, Y] positions of yellow (right) boundary cones.

    Note: m < n because cones are placed at CONE_SPACING intervals along the
    arc, not at every path point.

    Called by: gui/simulation.py (on_release, load_test_path),
               tuner/offline_tuner.py (_resample_path)
    """
    px = np.asarray(path_X)
    py = np.asarray(path_Y)

    # Compute cumulative arc length along the path
    ds  = np.hypot(np.diff(px), np.diff(py))
    arc = np.concatenate([[0.0], np.cumsum(ds)])  # Arc length at each point (m)
    total = arc[-1]

    # Sample uniformly every CONE_SPACING metres along the arc
    s_samples = np.arange(0.0, total, CONE_SPACING)

    # Interpolate X and Y coordinates at each sample arc length
    cx = np.interp(s_samples, arc, px)
    cy = np.interp(s_samples, arc, py)

    # Compute tangent direction at each sample point
    dx = np.gradient(np.interp(s_samples, arc, px), s_samples)  # dx/ds
    dy = np.gradient(np.interp(s_samples, arc, py), s_samples)  # dy/ds
    norms = np.hypot(dx, dy)
    norms = np.where(norms < 1e-6, 1.0, norms)   # Avoid zero-division on duplicate points

    # Left normal: rotate tangent (dx, dy) by +90°: nx = -dy, ny = dx
    nx = -dy / norms   # Left normal X component
    ny =  dx / norms   # Left normal Y component

    # Place cones at ±TRACK_HALF_WIDTH from centreline along the normal
    blue   = np.column_stack([cx + nx * TRACK_HALF_WIDTH,
                               cy + ny * TRACK_HALF_WIDTH])
    yellow = np.column_stack([cx - nx * TRACK_HALF_WIDTH,
                               cy - ny * TRACK_HALF_WIDTH])
    return blue, yellow


class SimPerception:
    """
    Simulates the vehicle's cone perception by filtering the full static
    cone map to only those cones within the vehicle's forward field of view.

    Mirrors sim_perception.py's SimPerception._visible() from the ROS2 stack
    (fsae_sim_perception package): the real node filters the oracle cone map
    to a forward window + omni radius; this class does the same on the
    pre-placed static cone map.

    The filtering is done in the vehicle's local body frame:
      - Transform cones from global frame to vehicle frame (rotation by -yaw)
      - Keep cones with:  MIN_AHEAD < x_local < LOOK_AHEAD  and  |y_local| < LOOK_WIDE

    Used by: gui/simulation.py (simulate_closed_loop),
             tuner/offline_tuner.py (run_headless_rollout)
    """

    def __init__(self, blue_all, yellow_all):
        """
        Parameters
        ----------
        blue_all : array-like, shape (n, 2)
            Full set of blue (left) cone positions [X, Y] in global frame.
        yellow_all : array-like, shape (n, 2)
            Full set of yellow (right) cone positions [X, Y] in global frame.
        """
        self._blue   = np.asarray(blue_all,   dtype=np.float64)
        self._yellow = np.asarray(yellow_all, dtype=np.float64)

    def visible_cones(self, car_x, car_y, car_yaw):
        """
        Return the subset of cones visible from the vehicle's current pose.

        Transforms each cone set to the vehicle body frame and applies the
        forward-FOV mask. Vectorised over all cones simultaneously.

        Parameters
        ----------
        car_x : float   Vehicle X position in global frame (m).
        car_y : float   Vehicle Y position in global frame (m).
        car_yaw : float Vehicle yaw angle (rad), measured from global X-axis.

        Returns
        -------
        (visible_blue, visible_yellow) : tuple of np.ndarray, each shape (k, 2)
            Subsets of the blue and yellow cone arrays that fall within the FOV.
            May be empty arrays if no cones are visible.

        Called by: gui/simulation.py (simulate_closed_loop),
                   tuner/offline_tuner.py (run_headless_rollout)
        """
        cos_y = math.cos(car_yaw)
        sin_y = math.sin(car_yaw)

        def _filter(cones):
            if len(cones) == 0:
                return cones
            # Translate to vehicle-centred frame
            rel  = cones - np.array([car_x, car_y])
            # Rotate to vehicle body frame:  [x_local, y_local] = R(-yaw) * rel
            x_c  =  rel[:, 0] * cos_y + rel[:, 1] * sin_y   # Forward distance
            y_c  = -rel[:, 0] * sin_y + rel[:, 1] * cos_y   # Lateral distance
            # Apply forward-FOV mask
            mask = (x_c > MIN_AHEAD) & (x_c < LOOK_AHEAD) & (np.abs(y_c) < LOOK_WIDE)
            return cones[mask]

        return _filter(self._blue), _filter(self._yellow)
