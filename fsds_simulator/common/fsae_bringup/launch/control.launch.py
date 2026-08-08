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
            'map_path', default_value='',
            description=(
                "Path to a fsae_MPCTest tuner/export_speed_profile.py CSV, "
                "exported from a recorded cone map. Has no effect unless "
                "use_precomputed_speed:=true. Applies to both `mpc` and "
                "`mpc_standalone`; ignored by `stanley`."
            )),
        DeclareLaunchArgument(
            'use_precomputed_speed', default_value='false',
            description=(
                "true  -> look up the target speed from map_path's precomputed "
                "oracle profile instead of live curvature_speed() every tick. "
                "Only valid for a track that's already been fully mapped -- see "
                "mpc_controller_standalone.py's map_path param and S48 in "
                "fsae_MPCTest/docs/sim_to_real_investigation.md. "
                "false (default) -> unchanged live curvature_speed() behaviour, "
                "regardless of what map_path is set to."
            )),
        Node(
            package='fsae_control',
            executable=controller_exec,
            name='controller',
            output='screen',
            parameters=[config, {'log_csv': log_csv, 'log_dir': log_dir, 'map_path': effective_map_path}],
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
