"""
AGV Sim Teleop Mode — drive robot in simulation with keyboard.

Launches:
  - Gazebo with greenhouse_simple.world
  - Robot spawned at starting position
  - Robot state publisher (sim URDF)
  - ODrive-realistic drive shaping node (cmd_vel → shaped_cmd_vel)
  - Diff drive plugin accepts shaped_cmd_vel
  - ZED 2i stereo camera (RGB, depth, point cloud)
  - ZED 2i IMU (400Hz) publishes /zed/zed_node/imu/data
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
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    worlds_pkg = get_package_share_directory('agv_sim_worlds')
    drive_pkg = get_package_share_directory('agv_sim_drive')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    ns = LaunchConfiguration('namespace')
    world_name = LaunchConfiguration('world')

    drive_params = os.path.join(drive_pkg, 'config', 'drive_shaping_params.yaml')

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

        # ODrive-realistic drive shaping: cmd_vel → shaped_cmd_vel
        Node(
            package='agv_sim_drive',
            executable='sim_drive_shaping_node',
            name='sim_drive_shaping_node',
            namespace=ns,
            parameters=[drive_params],
            output='screen',
        ),
    ])
