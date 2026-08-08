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
            # Default matches launch_all.sh's fixed cone_out_path (repo-root
            # cone_map.json) + tuner/export_speed_profile.py's own default
            # output path -- so `python3 -m tuner.export_speed_profile` (run
            # from fsae_MPCTest after a mapping lap) and this default agree
            # on where the CSV lives, with no path to pass on the command line
            # for the common case.
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
            default_value='/home/Formula-Student-Driverless-Simulator/fsae_MPCTest/tuner/speed_profile_export.csv',
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
                "mapped -- see mpc_controller_standalone.py's map_path param "
                "and S48 in fsae_MPCTest/docs/sim_to_real_investigation.md. "
                "Set to false here (or on the command line) to go back to "
                "unchanged live curvature_speed() behaviour regardless of "
                "map_path."
            )),
        DeclareLaunchArgument(
            # Same file format/default location as map_path (in fact the same
            # CSV works for both -- export_speed_profile.py writes x,y,psi,
            # v_target together). Kept as a separate launch arg from map_path
            # so the path and speed bypasses can be toggled independently.
            'path_map_path',
            default_value='/home/Formula-Student-Driverless-Simulator/fsae_MPCTest/tuner/speed_profile_export.csv',
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
                "mapped. Off by default, unlike use_precomputed_speed: this is "
                "a diagnostic/experiment mode, not a standing behaviour change."
            )),
        Node(
            package='fsae_control',
            executable=controller_exec,
            name='controller',
            output='screen',
            parameters=[config, {
                'log_csv': log_csv, 'log_dir': log_dir,
                'map_path': effective_map_path,
                'path_map_path': effective_path_map_path,
            }],
            condition=run_controller_mpc,
        ),
        Node(
            package='fsae_control',
            executable=controller_exec,
            name='controller',
            output='screen',
            parameters=[config, {'log_csv': log_csv, 'log_dir': log_dir}],
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
