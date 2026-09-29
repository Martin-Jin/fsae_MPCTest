"""
sim/planner.py — Sim-Side Planning

PURPOSE
-------
The simulator-side counterpart of fsae_planning's centerline_planner.py ROS 2
node. Consumes the cone observations sim/perception.py's SimPerception
produces and builds/blends a centreline, using the same cone-based path
building logic (planning/boundary.py, planning/path_utils.py) that runs on
the real vehicle, so tuned MPC weights transfer cleanly.

  1. SimPlanner                  — Accumulate cone observations, build + temporally blend centreline
  2. calculate_dynamic_max_steps — Size a rollout's step budget from a path's arc length

RELATIONSHIP TO ROS2 STACK
---------------------------
  SimPlanner      →  mirrors centerline_planner.py's CenterlinePlanner._planning_loop()

USED BY
-------
  gui/simulation.py    — instantiates SimPlanner inside simulate_closed_loop().
  tuner/offline_tuner.py — instantiates SimPlanner inside run_headless_rollout();
                     uses calculate_dynamic_max_steps() to size the step budget.

DOES NOT USE (directly)
-----------------------
  model/vehicle_physics.py, model/bicycle_model.py, controller/lmpc/solve.py, tuner/performance_stats.py, sim/speed_profile.py
"""

import math
import numpy as np
from planning.cone_map import ConeMap
from planning.boundary import build_path_walls
from planning.path_utils import blend_paths, build_local_path
import settings

# The circular-import hazard this deferred import used to guard against
# (settings/general.py imports sim.perception, not sim.planner, but the two
# lived in one sim/sim_track.py module before that split) no longer exists
# now that perception and planner are separate files -- settings/general.py
# never reaches sim.planner. See settings.py's PLANNER_* comment for why
# these must mirror fsae_params.yaml.


class SimPlanner:
    """
    Accumulates cone observations from SimPerception and builds a centreline
    path, mirroring centerline_planner.py's CenterlinePlanner._planning_loop().

    On each update() call, the planner:
      1. Adds the new visible cones to its persistent ConeMap
      2. Attempts to rebuild the boundary walls and centreline
      3. Temporally blends the fresh centreline with the previous one

    The stateful accumulation (ConeMap) means the planner's centreline improves
    as the vehicle moves forward and more cones enter the FOV — matching the
    behaviour of the real ROS2 planner which also accumulates observations.

    Planning emits path only (no speed field) — matching the upstream
    CenterlinePlanner ROS node, which publishes x,y waypoints and leaves speed
    targeting to the controller (see rollout_core.run_core_rollout()'s
    use_planner branch, which calls speed_profile.curvature_speed() on this
    centreline each step).

    Used by: gui/simulation.py (simulate_closed_loop),
             tuner/offline_tuner.py (run_headless_rollout)
    """

    def __init__(self):
        self._cone_map        = ConeMap()   # Persistent accumulator for cone observations
        self.centreline        = None       # Current best-estimate centreline (n, 2) or None
        self._prev_centreline  = None       # Last blended centreline, for next update()'s blend

    def update(self, blue_obs, yellow_obs, car_pos, car_yaw):
        """
        Ingest new cone observations and rebuild the centreline.

        Called every simulation step with the cones currently visible to
        SimPerception. The ConeMap de-duplicates repeated cone observations,
        so calling this frequently with overlapping FOVs is safe.

        Path building is attempted via build_path_walls() first (which uses
        cone-to-cone boundary matching). If that fails (e.g. too few cones),
        it falls back to build_local_path() which uses a simpler heuristic.

        The planner rebuilds the centreline from scratch every step, so
        successive raw centrelines can jump; blend_paths() (an EMA in the map
        frame) eases the published centreline between steps instead of jumping,
        mirroring centerline_planner.py's _planning_loop() in the ROS2 stack.

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

        Called by: gui/simulation.py (simulate_closed_loop),
                   tuner/offline_tuner.py (run_headless_rollout)
        """
        self._cone_map.update(blue_obs, yellow_obs)

        # Attempt primary path builder (cone-boundary matching + centreline extraction).
        # Keyword args mirror centerline_planner.py's _compute_path() exactly, sourced
        # from the same settings.PLANNER_* constants that mirror fsae_params.yaml's
        # live-tuned ROS params — see settings.py's comment for why this matters.
        # Omitting these here would silently fall back to build_path_walls'/
        # blend_paths' own hardcoded defaults instead of the live-tuned values.
        try:
            cl, _, _, _ = build_path_walls(
                self._cone_map.blue, self._cone_map.yellow, car_pos, car_yaw,
                smooth_per_pt=settings.PLANNER_SMOOTH_PER_PT,
                look_radius=settings.PLANNER_LOOK_RADIUS,
                plan_horizon=settings.PLANNER_PLAN_HORIZON,
            )
        except Exception:
            # Fallback: simple local path from cone midpoints
            cl = build_local_path(
                self._cone_map.blue, self._cone_map.yellow, car_pos, car_yaw
            )

        self.centreline = cl

        if self.centreline is not None and len(self.centreline) >= 2:
            self.centreline = blend_paths(
                self._prev_centreline, self.centreline, car_pos,
                alpha=settings.PLANNER_PATH_BLEND, horizon=settings.PLANNER_PLAN_HORIZON,
            )
            self._prev_centreline = self.centreline
        else:
            self._prev_centreline = None

    def reset(self):
        """
        Clear accumulated cone map and reset the centreline.

        Called when the simulation environment is reset (new path drawn or
        Reset button pressed in gui/simulation.py).
        """
        self._cone_map.reset()
        self.centreline       = None
        self._prev_centreline = None


def calculate_dynamic_max_steps(path_X, path_Y, dt=0.05, fallback_speed=2.50, buffer=1.5):
    """
    Compute the maximum number of simulation steps required to traverse the path.

    Rather than using a fixed step budget (which either wastes time on short
    paths or prematurely ends long ones), this dynamically estimates the step
    count from the path's arc length and a conservative fallback speed.

    Formula:
        max_time  = (arc_length / fallback_speed) * buffer
        max_steps = ceil(max_time / dt)

    The fallback_speed is intentionally conservative (2.5 m/s by default) to
    ensure the budget is sufficient even if the vehicle is slow (e.g. after
    a bad initial condition or near-DNF recovery). The buffer (1.5x) adds
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
        Conservative speed estimate for time calculation (m/s). Default 2.5 m/s.
    buffer : float
        Safety multiplier on the time estimate. Default 1.5x.

    Returns
    -------
    max_steps : int
        Maximum simulation steps. Returns 400 as a default if the path has
        fewer than 2 points.

    Called by: tuner/offline_tuner.py (run_headless_rollout)
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
