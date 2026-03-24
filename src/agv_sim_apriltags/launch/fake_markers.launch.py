"""
Launch the fake AprilTag marker detector for simulation.

This is a sim-only development tool that publishes marker_pose
messages when the robot is within detection range of known
AprilTag positions.

Usage:
  ros2 launch agv_sim_apriltags fake_markers.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('agv_sim_apriltags')
    markers_file = os.path.join(pkg_dir, 'config', 'markers_registry.yaml')

    ns = LaunchConfiguration('namespace')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='agv'),
        DeclareLaunchArgument('detection_range', default_value='2.0'),

        Node(
            package='agv_sim_apriltags',
            executable='fake_marker_detector.py',
            name='fake_marker_detector',
            namespace=ns,
            parameters=[{
                'markers_file': markers_file,
                'detection_range': LaunchConfiguration('detection_range'),
                'publish_rate': 5.0,
                'map_frame': 'map',
                'base_frame': 'base_link',
            }],
            output='screen',
        ),
    ])
