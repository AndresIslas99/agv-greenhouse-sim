"""
AGV Isaac Sim standalone teleop — smoke test the sim with keyboard.

Includes isaac_sim base, adds:
  - sim_drive_shaping_node subscribed directly to /agv/cmd_vel (no motor gate)
  - teleop_twist_keyboard publishing /agv/cmd_vel

This bypasses the ODrive arm/disarm emulation since there is no brain around
to publish motor_enable. For production-equivalent wiring (arm/disarm + full
ODrive contract) use isaac_hil.launch.py.

Usage:
  ros2 launch agv_isaac_sim isaac_teleop.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    drive_pkg = get_package_share_directory('agv_sim_drive')
    drive_params = os.path.join(drive_pkg, 'config', 'drive_shaping_params.yaml')

    return LaunchDescription([
        # Sensor base. standalone_mode=true so the sensor base brings up
        # the odom→base_link TF broadcaster (no brain ekf_local in teleop).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('agv_isaac_sim'), 'launch', 'isaac_sim.launch.py',
                ]),
            ),
            launch_arguments={'standalone_mode': 'true'}.items(),
        ),

        # Drive shaping: /agv/cmd_vel → /agv/shaped_cmd_vel
        # No motor gate in teleop mode — keyboard output is always "live".
        Node(
            package='agv_sim_drive',
            executable='sim_drive_shaping_node',
            name='sim_drive_shaping_node',
            namespace='agv',
            parameters=[drive_params, {'use_sim_time': True}],
            output='screen',
        ),

        # Keyboard teleop → /agv/cmd_vel
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop',
            remappings=[('cmd_vel', '/agv/cmd_vel')],
            output='screen',
            prefix='xterm -e',
        ),
    ])
