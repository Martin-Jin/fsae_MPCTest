import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
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
    run_controller = IfCondition(
        PythonExpression(["'", planner, "' != 'skidpad_planner'"])
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

    return LaunchDescription([
        DeclareLaunchArgument('planner', default_value='centerline_planner'),
        DeclareLaunchArgument(
            'controller', default_value='stanley',
            description='stanley | mpc | mpc_standalone — path-tracking controller to run'),
        Node(
            package='fsae_control',
            executable=controller_exec,
            name='controller',
            output='screen',
            parameters=[config],
            condition=run_controller,
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
