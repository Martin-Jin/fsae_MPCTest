import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


# Top-level simulator bring-up: composes perception + planning + control.
# Pick the planner with `planner:=…`; the mode wires the rest automatically.
#
#   ros2 launch fsae_bringup sim.launch.py                              # mpc_standalone (default)
#   ros2 launch fsae_bringup sim.launch.py planner:=skidpad_planner
#   ros2 launch fsae_bringup sim.launch.py controller:=stanley          # use the Stanley controller
#   ros2 launch fsae_bringup sim.launch.py record_cones:=false          # skip cone_recorder
def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory('fsae_bringup'), 'launch')
    planner = LaunchConfiguration('planner')
    controller = LaunchConfiguration('controller')
    record_cones = LaunchConfiguration('record_cones')
    cone_out_path = LaunchConfiguration('cone_out_path')
    log_csv = LaunchConfiguration('log_csv')
    log_dir = LaunchConfiguration('log_dir')

    # Skidpad needs the whole cone map up front to reconstruct the figure-8.
    full_track = PythonExpression(
        ["'true' if '", planner, "' == 'skidpad_planner' else 'false'"]
    )

    def include(name, args):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, name)),
            launch_arguments=args.items(),
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            'planner',
            default_value='centerline_planner',
            description='centerline_planner | skidpad_planner'),
        DeclareLaunchArgument(
            'controller',
            default_value='mpc_standalone',
            description='stanley | mpc | mpc_standalone — path-tracking controller to run'),
        DeclareLaunchArgument(
            'record_cones',
            default_value='true',
            description='Also launch cone_recorder to log one lap of boundary cones'),
        DeclareLaunchArgument(
            'cone_out_path',
            default_value='',
            description="cone_recorder output path ('' -> ~/fsae_logs/cone_map_<timestamp>.json)"),
        DeclareLaunchArgument(
            'log_csv',
            default_value='true',
            description='Write controller CSV telemetry (e_y/e_psi/steer/...) to log_dir'),
        DeclareLaunchArgument(
            'log_dir',
            default_value='',
            description="Controller CSV telemetry output dir ('' -> ~/fsae_logs). "
                        "launch_all.sh passes the repo root here so logs land in "
                        "<repo>/fsae_logs instead."),
        include('perception.launch.py', {'full_track': full_track}),
        include('planning.launch.py',   {'planner': planner}),
        include('control.launch.py',    {
            'planner': planner, 'controller': controller,
            'log_csv': log_csv, 'log_dir': log_dir,
        }),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'cone_recorder.launch.py')),
            launch_arguments={'out_path': cone_out_path}.items(),
            condition=IfCondition(record_cones),
        ),
    ])
