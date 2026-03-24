"""
AGV Sim Teleop Mode — drive robot in simulation with keyboard.

Launches:
  - Gazebo with greenhouse_simple.world
  - Robot spawned at starting position
  - Robot state publisher (sim URDF)
  - Diff drive plugin provides wheel_odom and accepts cmd_vel
  - IMU sensor publishes imu/data
  - Lidar publishes scan
  - teleop_twist_keyboard for manual control

In teleop mode, the diff_drive plugin publishes odom->base_link TF directly.

Usage:
  ros2 launch agv_sim_bringup sim_teleop.launch.py
  ros2 launch agv_sim_bringup sim_teleop.launch.py world:=nav_test
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

    ns = LaunchConfiguration('namespace')
    world_name = LaunchConfiguration('world')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='agv'),
        DeclareLaunchArgument('world', default_value='greenhouse_simple',
                              description='World name without .world extension'),
        DeclareLaunchArgument('x', default_value='2.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        # Gazebo model path
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            os.path.join(worlds_pkg, 'models'),
        ),

        # Start Gazebo server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_pkg, 'launch', 'gzserver.launch.py'),
            ),
            launch_arguments={
                'world': PathJoinSubstitution([
                    FindPackageShare('agv_sim_worlds'), 'worlds',
                    [world_name, '.world'],
                ]),
            }.items(),
        ),

        # Start Gazebo client
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_pkg, 'launch', 'gzclient.launch.py'),
            ),
        ),

        # Spawn robot — in teleop mode, diff_drive publishes odom TF
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('agv_sim_bringup'), 'launch', 'spawn_robot.launch.py',
                ]),
            ),
            launch_arguments={
                'namespace': ns,
                'x': LaunchConfiguration('x'),
                'y': LaunchConfiguration('y'),
                'yaw': LaunchConfiguration('yaw'),
                'publish_odom_tf': 'true',
            }.items(),
        ),
    ])
