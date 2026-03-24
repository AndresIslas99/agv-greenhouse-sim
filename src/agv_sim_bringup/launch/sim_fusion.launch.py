"""
AGV Sim Fusion Mode — teleop + slam_toolbox (localization) + dual EKF.

Launches:
  - Gazebo with greenhouse_simple.world
  - Robot spawned at starting position
  - Robot state publisher (sim URDF)
  - Diff drive plugin: publish_odom_tf=false (EKF owns odom->base_link)
  - slam_toolbox in localization mode (loads a saved map)
  - Dual EKF:
    - ekf_local: wheel_odom + simulated IMU -> odom->base_link
    - ekf_global: local output + slam_toolbox pose -> map->odom

Usage:
  ros2 launch agv_sim_bringup sim_fusion.launch.py map:=/path/to/greenhouse_map.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    worlds_pkg = get_package_share_directory('agv_sim_worlds')
    bringup_pkg = get_package_share_directory('agv_sim_bringup')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    ns = LaunchConfiguration('namespace')
    world_name = LaunchConfiguration('world')
    map_file = LaunchConfiguration('map')

    ekf_local_config = os.path.join(bringup_pkg, 'config', 'sim_ekf_local.yaml')
    ekf_global_config = os.path.join(bringup_pkg, 'config', 'sim_ekf_global.yaml')
    slam_params = os.path.join(bringup_pkg, 'config', 'slam_toolbox_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='agv'),
        DeclareLaunchArgument('world', default_value='greenhouse_simple'),
        DeclareLaunchArgument('x', default_value='2.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('map', default_value='',
                              description='Path to map YAML for localization (without extension)'),

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

        # Spawn robot — EKF owns odom TF in fusion mode
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
                'publish_odom_tf': 'false',
            }.items(),
        ),

        # slam_toolbox in localization mode
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='slam_toolbox',
                    executable='localization_slam_toolbox_node',
                    name='slam_toolbox',
                    namespace=ns,
                    parameters=[
                        slam_params,
                        {'mode': 'localization'},
                        {'map_file_name': map_file},
                    ],
                    remappings=[
                        ('scan', 'scan'),
                    ],
                    output='screen',
                ),
            ],
        ),

        # Dual EKF — local: wheel_odom + IMU -> odom->base_link
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='robot_localization',
                    executable='ekf_node',
                    name='ekf_local',
                    namespace=ns,
                    parameters=[ekf_local_config],
                    remappings=[
                        ('odometry/filtered', 'odometry/local'),
                    ],
                    output='screen',
                ),
            ],
        ),

        # Dual EKF — global: local + slam_toolbox -> map->odom
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='robot_localization',
                    executable='ekf_node',
                    name='ekf_global',
                    namespace=ns,
                    parameters=[ekf_global_config],
                    remappings=[
                        ('odometry/filtered', 'odometry/global'),
                    ],
                    output='screen',
                ),
            ],
        ),
    ])
