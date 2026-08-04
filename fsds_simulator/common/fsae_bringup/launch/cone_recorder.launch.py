from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Standalone add-on: records one lap's worth of boundary cones to a JSON file
# for later replay outside FSDS (see fsae_MPCTest/sim/track_io.py). Launch
# alongside a normal run (any planner/controller) — this node only subscribes,
# it does not affect the pipeline.
def generate_launch_description():
    out_path = LaunchConfiguration('out_path')

    return LaunchDescription([
        DeclareLaunchArgument(
            'out_path', default_value='',
            description="JSON output path ('' -> ~/fsae_logs/cone_map_<timestamp>.json)"),
        Node(
            package='fsae_sim_perception',
            executable='cone_recorder',
            name='cone_recorder',
            output='screen',
            parameters=[{'out_path': out_path}],
        ),
    ])
