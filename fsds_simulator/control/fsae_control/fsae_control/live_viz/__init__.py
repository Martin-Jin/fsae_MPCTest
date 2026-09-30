"""
Live, close-up debug visualiser for the simulator: car (as a triangle), cone
map, planner/reference path, NMPC's predicted horizon (NMPC only), the car's
driven trail, and the controller's current output/stats, all redrawn from
live ROS2 topics on a timer.

Sim-only debug tool, not part of the car's autonomy stack: never launched by
`fsae_autonomous`, only by `ros2/launch_all.sh` alongside the simulator, the
same way the periodic-teleport diagnostics are (see that script's "TEMPORARY"
block). Standalone matplotlib window, no rqt/rviz dependency, run:

    ros2 run fsae_control live_viz

Topics subscribed (see this repo's fsae_planning per-file docstrings for the
authoritative topic table):
    /fsae/slam/left_track            fsae_interfaces/Track          blue boundary, global frame
    /fsae/slam/right_track           fsae_interfaces/Track          yellow boundary, global frame
    /fsae/slam/car_position           geometry_msgs/PoseStamped      car pose, global frame.
                                                                     orientation is NOT a real
                                                                     quaternion: .w carries raw
                                                                     yaw (radians) directly, see
                                                                     sim_perception.py's
                                                                     _car_pose_msg()
    /fsae/slam/car_odom               nav_msgs/Odometry              car speed/yaw rate
    /fsae/planning/selected_trajectory  geometry_msgs/PoseArray      live planner's centreline
                                                                     (empty in precomputed-path
                                                                     mode -- the planner does not
                                                                     even run then, see
                                                                     sim.launch.py)
    /fsae/control/static_reference_path geometry_msgs/PoseArray      one-shot, TRANSIENT_LOCAL:
                                                                     the precomputed path
                                                                     (path_map_path), when set --
                                                                     see mpc_controller.py. Drawn
                                                                     INSTEAD OF the topic above
                                                                     when populated, never both
    /fsae/control/nmpc_predicted_path geometry_msgs/PoseArray        NMPC's predicted horizon,
                                                                     only published when
                                                                     use_nmpc=true (see
                                                                     nmpc/reference.py's xy_at())
    /fsds/control_command             fs_msgs/ControlCommand         steering/throttle/brake
                                                                     (standalone_output=true)
    /fsae/control/cmd_vel             ackermann_msgs/AckermannDriveStamped  (standalone_output=false)
    /fsae/control/debug_weights       std_msgs/String                 JSON: per-tick weighted
                                                                       tracking-error breakdown
                                                                       (e_y/e_psi/e_v: error,
                                                                       weight, cost, pct of total)
                                                                       + solve_ms, debug-only, see
                                                                       mpc_controller.py's
                                                                       _publish_debug_weights()
    /fsae/control/debug_stanley       std_msgs/String                 JSON: Stanley's own three
                                                                       control-law terms (heading
                                                                       error, atan2 cross-track,
                                                                       yaw-rate damping) + e_y/
                                                                       e_psi/steering, debug-only.
                                                                       Also this node's only
                                                                       positive "Stanley is
                                                                       active" signal, see
                                                                       stanley_controller.py's
                                                                       _publish_debug_stanley()
    /fsae/control/lap_summary         std_msgs/String                 JSON: per-lap composite
                                                                       score + horizon-accuracy
                                                                       %, published once per
                                                                       completed lap by either
                                                                       controller node (see
                                                                       ControlLogger.finish_lap()
                                                                       in telemetry/control_logger.py).
                                                                       pred_acc_pct is n/a
                                                                       (absent) for Stanley/
                                                                       LTV-QP runs -- NMPC-only,
                                                                       see HorizonAccuracyTracker

Both control-output topics are subscribed; whichever one is actually being
published (depends on the `standalone_output` launch arg) is the one that
updates the stats panel, the other simply never fires.

MODULE MAP
----------
  panels.py  view/refresh constants and the debug-panel term tables
  node.py    LiveVizNode (topic subscriptions) and get_car_triangle
  app.py     the matplotlib window and main()
"""

from fsae_control.live_viz.app import main  # noqa: F401
from fsae_control.live_viz.node import LiveVizNode, get_car_triangle  # noqa: F401
