import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Simulator perception stand-in: FSDS oracle map + odom → /fsae/* inputs.
# `full_track:=true` publishes the entire map every frame (used by skidpad).
def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('fsae_bringup'), 'config', 'fsae_params.yaml'
    )
    full_track = LaunchConfiguration('full_track')

    return LaunchDescription([
        DeclareLaunchArgument('full_track', default_value='false',
                              description='publish the whole cone map instead of a FOV window'),
        Node(
            package='fsae_sim_perception',
            executable='sim_perception',
            name='sim_perception',
            output='screen',
            parameters=[config, {'full_track': full_track}],
        ),
    ])
