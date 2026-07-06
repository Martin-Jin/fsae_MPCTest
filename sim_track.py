"""
sim_track.py — Cone Placement and Sim-Side Perception/Planning

PURPOSE
-------
Provides the simulator-side implementations of the perception and planning
pipeline, mirroring the behaviour of perception_node.py and planner_node.py
from the ROS2 stack. This allows the offline 2-D simulator to use the same
cone-based path building logic that runs on the real vehicle, ensuring that
tuned MPC weights transfer cleanly.

The three public classes/functions serve distinct pipeline stages:
  1. place_cones()    — Generate a static cone map from a path (track layout)
  2. SimPerception    — Filter visible cones from the car's current position/FOV
  3. SimPlanner       — Accumulate cone observations, build centreline + speed profile

RELATIONSHIP TO ROS2 STACK
---------------------------
  place_cones()   →  static track layout (not in ROS2; the real track has real cones)
  SimPerception   →  mirrors perception_node.py._publish_visible_cones()
  SimPlanner      →  mirrors planner_node.py._planning_loop()

Both SimPerception and SimPlanner are stateful: SimPerception holds the full
cone map and filters it; SimPlanner accumulates a ConeMap and rebuilds the
centreline+speed profile on every update.

USED BY
-------
  simulation.py    — instantiates SimPerception and SimPlanner inside
                     simulate_closed_loop(); calls place_cones() after path creation.
  offline_tuner.py — instantiates both inside run_headless_rollout(); uses
                     calculate_dynamic_max_steps() to size the step budget.

DOES NOT USE (directly)
-----------------------
  vehicle_physics.py, bicycle_model.py, optimiser.py, performance_stats.py
  (uses speed_profile.py indirectly via SimPlanner.update)
"""

import math
import numpy as np
from planning.cone_map import ConeMap
from planning.boundary import build_path_walls
from planning.path_utils import build_local_path
import speed_profile as sp

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

    Called by: simulation.py (on_release, load_test_path),
               offline_tuner.py (_resample_path)
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

    Mirrors perception_node.py._publish_visible_cones() from the ROS2 stack:
    the real perception node filters LiDAR/camera detections to a forward cone;
    this class does the same on the pre-placed static cone map.

    The filtering is done in the vehicle's local body frame:
      - Transform cones from global frame to vehicle frame (rotation by -yaw)
      - Keep cones with:  MIN_AHEAD < x_local < LOOK_AHEAD  and  |y_local| < LOOK_WIDE

    Used by: simulation.py (simulate_closed_loop),
             offline_tuner.py (run_headless_rollout)
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

        Called by: simulation.py (simulate_closed_loop),
                   offline_tuner.py (run_headless_rollout)
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


class SimPlanner:
    """
    Accumulates cone observations from SimPerception and builds a centreline
    path and speed profile, mirroring planner_node.py._planning_loop().

    On each update() call, the planner:
      1. Adds the new visible cones to its persistent ConeMap
      2. Attempts to rebuild the boundary walls and centreline
      3. Computes a curvature-based speed profile along the new centreline

    The stateful accumulation (ConeMap) means the planner's centreline improves
    as the vehicle moves forward and more cones enter the FOV — matching the
    behaviour of the real ROS2 planner which also accumulates observations.

    Used by: simulation.py (simulate_closed_loop),
             offline_tuner.py (run_headless_rollout)
    """

    def __init__(self, v_max=20.0, v_min=1.5, lookahead_dist=4.0):
        """
        Parameters
        ----------
        v_max : float
            Maximum speed cap for the speed profile (m/s).
            Passed to speed_profile.compute_speed_profile().
        v_min : float
            Minimum speed floor for the speed profile (m/s).
        """
        self._cone_map      = ConeMap()       # Persistent accumulator for cone observations
        self._v_max         = v_max
        self._v_min         = v_min
        self.centreline     = None            # Current best-estimate centreline (n, 2) or None
        self.v_profile      = np.array([])   # Speed profile along centreline (n,) or empty

    def update(self, blue_obs, yellow_obs, car_pos, car_yaw):
        """
        Ingest new cone observations and rebuild the centreline + speed profile.

        Called every simulation step with the cones currently visible to
        SimPerception. The ConeMap de-duplicates repeated cone observations,
        so calling this frequently with overlapping FOVs is safe.

        Path building is attempted via build_path_walls() first (which uses
        cone-to-cone boundary matching). If that fails (e.g. too few cones),
        it falls back to build_local_path() which uses a simpler heuristic.

        Speed profile is computed only if the centreline has ≥ 3 points.

        Parameters
        ----------
        blue_obs : np.ndarray, shape (k, 2)
            Currently visible blue cone positions [X, Y] in global frame.
        yellow_obs : np.ndarray, shape (k, 2)
            Currently visible yellow cone positions [X, Y] in global frame.
        car_pos : np.ndarray, shape (2,)
            Vehicle position [X, Y] in global frame (m).
        car_yaw : float
            Vehicle yaw angle (rad).

        Called by: simulation.py (simulate_closed_loop),
                   offline_tuner.py (run_headless_rollout)
        """
        self._cone_map.update(blue_obs, yellow_obs)

        # Attempt primary path builder (cone-boundary matching + centreline extraction)
        try:
            cl, _, _, _ = build_path_walls(
                self._cone_map.blue, self._cone_map.yellow, car_pos, car_yaw
            )
        except Exception:
            # Fallback: simple local path from cone midpoints
            cl = build_local_path(
                self._cone_map.blue, self._cone_map.yellow, car_pos, car_yaw
            )

        self.centreline = cl

        if cl is not None and len(cl) >= 3:
            # Compute curvature-based speed profile along the new centreline
            raw = sp.compute_speed_profile(
                cl[:, 0], cl[:, 1], v_max=self._v_max, v_min=self._v_min
            )
            self.v_profile = sp.smooth_profile(raw, window=9)
        else:
            self.v_profile = np.array([])   # Insufficient points; use v_min fallback

    def reset(self):
        """
        Clear accumulated cone map and reset centreline/profile.

        Called when the simulation environment is reset (new path drawn or
        Reset button pressed in simulation.py).
        """
        self._cone_map.reset()
        self.centreline = None
        self.v_profile  = np.array([])


def calculate_dynamic_max_steps(path_X, path_Y, dt=0.05, fallback_speed=2.50, buffer=1.5):
    """
    Compute the maximum number of simulation steps required to traverse the path.

    Rather than using a fixed step budget (which either wastes time on short
    paths or prematurely ends long ones), this dynamically estimates the step
    count from the path's arc length and a conservative fallback speed.

    Formula:
        max_time  = (arc_length / fallback_speed) * buffer
        max_steps = ceil(max_time / dt)

    The fallback_speed is intentionally conservative (3 m/s by default) to
    ensure the budget is sufficient even if the vehicle is slow (e.g. after
    a bad initial condition or near-DNF recovery). The buffer (1.5×) adds
    additional margin for transient slow periods.

    Parameters
    ----------
    path_X : array-like, shape (n,)
        Path X coordinates (m).
    path_Y : array-like, shape (n,)
        Path Y coordinates (m).
    dt : float
        Simulation timestep (s). Default 0.05 s (20 Hz).
    fallback_speed : float
        Conservative speed estimate for time calculation (m/s). Default 3.0 m/s.
    buffer : float
        Safety multiplier on the time estimate. Default 1.5×.

    Returns
    -------
    max_steps : int
        Maximum simulation steps. Returns 400 as a default if the path has
        fewer than 2 points.

    Called by: offline_tuner.py (run_headless_rollout)
    """
    px = np.asarray(path_X)
    py = np.asarray(path_Y)

    if len(px) < 2:
        return 400   # Default fallback for uninitialised paths

    # Total arc length of the path
    ds = np.hypot(np.diff(px), np.diff(py))
    total_length = np.sum(ds)

    # Time budget at worst-case conservative speed, with safety buffer
    max_time  = (total_length / fallback_speed) * buffer
    max_steps = int(math.ceil(max_time / dt))

    return max_steps