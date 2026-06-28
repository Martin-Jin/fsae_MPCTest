from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fsae_planning',
            executable='perception_node',
            name='perception',
            output='screen',
        ),
        Node(
            package='fsae_planning',
            executable='planner_node',
            name='centreline_planner',
            output='screen',
        ),
        Node(
            package='fsae_planning',
            executable='control_node',
            name='controller',
            output='screen',
        ),
    ])
