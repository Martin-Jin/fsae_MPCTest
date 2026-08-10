import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    EqualsSubstitution, IfElseSubstitution, LaunchConfiguration, PythonExpression,
)
from launch_ros.actions import Node


# Control subsystem: a path-tracking controller + the FSDS command bridge.
# The controller is selectable with `controller:=stanley|mpc|mpc_standalone`.
#   stanley, mpc       — publish the shared cmd_vel interface; fsds_bridge
#                         converts it (speed/steering -> ControlCommand) and
#                         owns GO-gating + cone e-braking, identically for both.
#   mpc_standalone     — publishes fs_msgs/ControlCommand directly, using the
#                         MPC's own throttle/brake instead of fsds_bridge's
#                         speed-error P-loop (preserves the offline-tuned
#                         longitudinal behaviour from the fsae_MPCTest repo).
#                         Owns GO-gating + cone e-braking itself, so
#                         fsds_bridge is skipped for this mode — see
#                         mpc_controller_standalone.py's module docstring.
# The skidpad planner drives the car itself (it publishes cmd_vel directly),
# so the controller is skipped in skidpad mode; fsds_bridge still runs then
# (skidpad also uses the shared cmd_vel interface).
def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('fsae_bringup'), 'config', 'fsae_params.yaml'
    )
    planner = LaunchConfiguration('planner')
    controller = LaunchConfiguration('controller')
    log_csv = LaunchConfiguration('log_csv')
    log_dir = LaunchConfiguration('log_dir')
    map_path = LaunchConfiguration('map_path')
    use_precomputed_speed = LaunchConfiguration('use_precomputed_speed')
    path_map_path = LaunchConfiguration('path_map_path')
    use_precomputed_path = LaunchConfiguration('use_precomputed_path')
    v_max = LaunchConfiguration('v_max')
    v_min = LaunchConfiguration('v_min')
    stanley_gain = LaunchConfiguration('stanley_gain')
    # Effective map_path handed to the node: '' whenever the feature is
    # switched off, regardless of what map_path itself is set to -- so
    # use_precomputed_speed:=false is a reliable one-flag disable without
    # having to also clear map_path (map_path alone doubles as "where's the
    # file" and, implicitly, "is this on"; this makes "is this on" explicit).
    # IfElseSubstitution (not PythonExpression) deliberately: map_path is an
    # arbitrary filesystem path that could contain backslashes (Windows) or
    # quotes, which would corrupt/break a PythonExpression string built by
    # concatenating it into Python source text -- IfElseSubstitution passes
    # it through as data instead of evaluating it.
    effective_map_path = IfElseSubstitution(
        EqualsSubstitution(use_precomputed_speed, 'true'),
        map_path,
        '',
    )
    # Same pattern as effective_map_path, for the path-import toggle.
    effective_path_map_path = IfElseSubstitution(
        EqualsSubstitution(use_precomputed_path, 'true'),
        path_map_path,
        '',
    )
    # Map the friendly name to the package entry point; node name stays
    # 'controller' either way so all three read the `controller:` params block.
    controller_exec = PythonExpression([
        "'mpc_controller_standalone' if '", controller, "' == 'mpc_standalone' "
        "else 'mpc_controller' if '", controller, "' == 'mpc' "
        "else 'controller'"
    ])
    # fsds_bridge is redundant (and would race mpc_standalone for
    # /fsds/control_command) when the standalone MPC node is selected.
    run_bridge = IfCondition(
        PythonExpression(["'", controller, "' != 'mpc_standalone'"])
    )
    # map_path is an MPC-only param (mpc_controller.py / mpc_controller_standalone.py
    # both declare_parameters it; stanley_controller.py does not) -- passing it
    # unconditionally would raise ParameterNotDeclaredException on the default
    # controller:=stanley. Only include it in the params list for mpc/mpc_standalone,
    # via a separate Node() entry rather than branching one entry's params.
    run_controller_mpc = IfCondition(
        PythonExpression([
            "'", planner, "' != 'skidpad_planner' and '", controller,
            "' in ('mpc', 'mpc_standalone')"
        ])
    )
    run_controller_non_mpc = IfCondition(
        PythonExpression([
            "'", planner, "' != 'skidpad_planner' and '", controller,
            "' not in ('mpc', 'mpc_standalone')"
        ])
    )

    return LaunchDescription([
        DeclareLaunchArgument('planner', default_value='centerline_planner'),
        DeclareLaunchArgument(
            'controller', default_value='stanley',
            description='stanley | mpc | mpc_standalone — path-tracking controller to run'),
        DeclareLaunchArgument(
            'log_csv', default_value='false',
            description='Write controller CSV telemetry (e_y/e_psi/steer/...) to log_dir'),
        DeclareLaunchArgument(
            'log_dir', default_value='',
            description="Controller CSV telemetry output dir ('' -> ~/fsae_logs)"),
        DeclareLaunchArgument(
            # Default points into fsae_MPCTest's tracks/<name>/ layout, where
            # `python3 -m tuner.export_speed_profile <name>` writes -- so the
            # exporter and this default agree on where the CSV lives, with no
            # path to pass on the command line for the common case.
            #
            # To drive a DIFFERENT track, don't edit this line: set TRACK= in
            # ros2/launch_all.sh, which expands to map_path/path_map_path for
            # both args at once. This default is only the fallback for a bare
            # `ros2 launch fsae_bringup control.launch.py`.
            #
            # Hardcoded absolute path, not derived from __file__ or
            # get_package_share_directory(): both resolve to the INSTALLED
            # copy under ros2/install/... at runtime (confirmed: this launch
            # file is itself copied there by colcon build), which has no
            # relationship to fsae_MPCTest's location -- that repo lives
            # entirely outside the ROS workspace, so there is no path from
            # anything ROS-visible to it. This matches WHERE launch_all.sh
            # runs `ros2 launch` FROM (inside WSL/the Docker container, not
            # Windows) -- update this line if the repo root ever moves.
            'map_path',
            default_value='/home/Formula-Student-Driverless-Simulator/fsae_MPCTest/tracks/comp_test_map_3/speed_profile.csv',
            description=(
                "Path to a fsae_MPCTest tuner/export_speed_profile.py CSV, "
                "exported from a recorded cone map. Has no effect unless "
                "use_precomputed_speed:=true. Applies to both `mpc` and "
                "`mpc_standalone`; ignored by `stanley`. If the file doesn't "
                "exist yet (e.g. a fresh checkout before the first "
                "export_speed_profile.py run), the node logs an error at "
                "startup and falls back to live curvature_speed() -- it does "
                "not crash."
            )),
        DeclareLaunchArgument(
            'use_precomputed_speed', default_value='true',
            description=(
                "true (default) -> look up the target speed from map_path's "
                "precomputed oracle profile instead of live curvature_speed() "
                "every tick. Only valid for a track that's already been fully "
                "mapped -- see mpc_controller_standalone.py's map_path param. "
                "Set to false here (or on the command line) to go back to "
                "unchanged live curvature_speed() behaviour regardless of "
                "map_path."
            )),
        DeclareLaunchArgument(
            # Same x,y,psi,v_target file FORMAT as map_path, but a different
            # default FILE: the track's raceline.csv (tuner/raceline_optimizer.py's
            # minimum-time line) rather than its speed_profile.csv (the
            # centreline). The MPC's optimum is always e_y=0 on whatever path
            # it is given, so it can never invent a racing line from a
            # centreline reference no matter how Q/R are tuned -- the tracked
            # geometry has to contain the widen-entry/clip-apex shape itself.
            # Kept as a separate launch arg from map_path so the path and speed
            # bypasses can be toggled independently.
            'path_map_path',
            default_value='/home/Formula-Student-Driverless-Simulator/fsae_MPCTest/tracks/comp_test_map_3/raceline.csv',
            description=(
                "Path to a fsae_MPCTest tuner/export_speed_profile.py CSV, used "
                "as the tracked PATH (not just speed) -- see "
                "mpc_controller_standalone.py's path_map_path param. Has no "
                "effect unless use_precomputed_path:=true. Applies to both "
                "`mpc` and `mpc_standalone`; ignored by `stanley`."
            )),
        DeclareLaunchArgument(
            'use_precomputed_path', default_value='true',
            description=(
                "true -> track path_map_path's precomputed path instead of "
                "subscribing to the live planner's "
                "/fsae/planning/selected_trajectory -- removes "
                "centerline_planner.py from the control loop entirely, to "
                "isolate controller/plant tracking error from planner-induced "
                "path error. Only valid for a track that's already been fully "
                "mapped. On by default, matching "
                "use_precomputed_speed -- set false for the planner-in-loop "
                "diagnostic/experiment mode instead."
            )),
        # v_max/v_min/stanley_gain: overrides the shared controller.ros__parameters
        # block in fsae_params.yaml (applies to stanley, mpc, and mpc_standalone
        # alike -- the node name is 'controller' for all three). Defaults match
        # that file's current values exactly, so leaving these unset on the
        # command line changes nothing; pass e.g. v_max:=3.0 for a one-off slow
        # lap (cone-map recording, characterisation runs) without editing the
        # shared config file.
        DeclareLaunchArgument(
            'v_max', default_value='15.0',
            description='m/s -- top speed on straights (overrides fsae_params.yaml controller.v_max)'),
        DeclareLaunchArgument(
            'v_min', default_value='1.5',
            description='m/s -- minimum speed through tight corners (overrides fsae_params.yaml controller.v_min)'),
        DeclareLaunchArgument(
            'stanley_gain', default_value='1.0',
            description='cross-track gain k_cte (overrides fsae_params.yaml controller.stanley_gain)'),
        Node(
            package='fsae_control',
            executable=controller_exec,
            name='controller',
            output='screen',
            parameters=[config, {
                'log_csv': log_csv, 'log_dir': log_dir,
                'map_path': effective_map_path,
                'path_map_path': effective_path_map_path,
                'v_max': v_max, 'v_min': v_min,
                # stanley_gain deliberately omitted here: neither
                # mpc_controller.py nor mpc_controller_standalone.py declares
                # it, so passing it would raise ParameterNotDeclaredException.
            }],
            condition=run_controller_mpc,
        ),
        Node(
            package='fsae_control',
            executable=controller_exec,
            name='controller',
            output='screen',
            parameters=[config, {
                'log_csv': log_csv, 'log_dir': log_dir,
                'v_max': v_max, 'v_min': v_min, 'stanley_gain': stanley_gain,
            }],
            condition=run_controller_non_mpc,
        ),
        Node(
            package='fsae_control',
            executable='fsds_bridge',
            name='fsds_bridge',
            output='screen',
            parameters=[config],
            condition=run_bridge,
        ),
    ])
