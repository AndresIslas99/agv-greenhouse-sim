"""
Launch the ODrive-realistic drive shaping node.

Sits between cmd_vel and the Gazebo diff_drive plugin (shaped_cmd_vel).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('agv_sim_drive')
    params_file = os.path.join(pkg, 'config', 'drive_shaping_params.yaml')

    ns = LaunchConfiguration('namespace')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='agv'),

        Node(
            package='agv_sim_drive',
            executable='sim_drive_shaping_node',
            name='sim_drive_shaping_node',
            namespace=ns,
            parameters=[params_file],
            output='screen',
        ),
    ])
