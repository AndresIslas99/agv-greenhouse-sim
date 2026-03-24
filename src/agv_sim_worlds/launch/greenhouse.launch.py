"""
Launch Gazebo with the greenhouse world.

Usage:
  ros2 launch agv_sim_worlds greenhouse.launch.py
  ros2 launch agv_sim_worlds greenhouse.launch.py world:=nav_test.world
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    worlds_pkg = get_package_share_directory('agv_sim_worlds')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    world_file = LaunchConfiguration('world')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(worlds_pkg, 'worlds', 'greenhouse_simple.world'),
            description='Path to world file',
        ),

        # Add our models to the Gazebo model path
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            os.path.join(worlds_pkg, 'models'),
        ),

        # Start Gazebo server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_pkg, 'launch', 'gzserver.launch.py'),
            ),
            launch_arguments={'world': world_file}.items(),
        ),

        # Start Gazebo client
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_pkg, 'launch', 'gzclient.launch.py'),
            ),
        ),
    ])
