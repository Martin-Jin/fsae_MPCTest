"""
settings/planner.py — sim/planner.py::SimPlanner's tunables. MUST mirror
fsae_bringup/config/fsae_params.yaml's centerline_planner block (live ROS
params: smooth, look_radius, plan_horizon, path_blend).
"""

# ------------------------------------------------------------------------------
# Planner tunables — MUST mirror fsae_bringup/config/fsae_params.yaml's
# centerline_planner block (live ROS params: smooth, look_radius, plan_horizon,
# path_blend). sim/planner.py::SimPlanner must pass these four values as
# explicit keyword args to build_path_walls()/blend_paths() — those functions
# have their own hardcoded defaults that silently diverge from the live-tuned
# values below (e.g. PLANNER_SMOOTH_PER_PT's live 0.015 vs. build_path_walls'
# default 0.05, and blend_paths' internal horizon default of 15.0 vs. the
# live-tuned 25.0), which is a real parity break if the keyword args are ever
# dropped. See sim_to_real_investigation.md S31 for the mechanism and its
# measured effect.
# ------------------------------------------------------------------------------
PLANNER_SMOOTH_PER_PT = 0.015   # m^2 smoothing budget per input point (splprep s = this * n_pts)

PLANNER_LOOK_RADIUS = 25.0      # m; omni-directional cone-map crop radius

PLANNER_PLAN_HORIZON = 25.0     # m; arc-length the published centreline is clamped to

PLANNER_PATH_BLEND = 0.4        # 0<a<=1; temporal EMA weight toward each freshly-planned path
