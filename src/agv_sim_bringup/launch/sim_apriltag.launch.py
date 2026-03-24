"""
Launch apriltag_ros detection configured for the simulated ZED 2i camera.

Run alongside any sim mode that has the ZED camera active.

Usage:
  ros2 launch agv_sim_bringup sim_apriltag.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ns = LaunchConfiguration('namespace')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='agv'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('agv_sim_apriltags'), 'launch',
                    'apriltag_detection.launch.py',
                ]),
            ),
            launch_arguments={
                'namespace': ns,
            }.items(),
        ),
    ])
