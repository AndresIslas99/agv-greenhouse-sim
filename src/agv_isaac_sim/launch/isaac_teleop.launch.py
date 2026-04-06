"""
AGV Isaac Sim Teleop Mode — drive robot with keyboard in Isaac Sim.

Launches:
  - isaac_sim.launch.py (base)
  - teleop_twist_keyboard

Usage:
  ros2 launch agv_isaac_sim isaac_teleop.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Base Isaac Sim launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('agv_isaac_sim'), 'launch', 'isaac_sim.launch.py',
                ]),
            ),
        ),

        # Keyboard teleop
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop',
            remappings=[('cmd_vel', '/agv/cmd_vel')],
            output='screen',
            prefix='xterm -e',
        ),
    ])
