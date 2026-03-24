"""
AGV Sim Mapping Mode — teleop + slam_toolbox for map creation.

Launches:
  - Gazebo with greenhouse_simple.world
  - Robot spawned at starting position
  - Robot state publisher (sim URDF)
  - slam_toolbox in online_async mapping mode
  - In this mode, slam_toolbox owns map->odom TF
  - Diff drive publishes odom->base_link TF directly

Usage:
  ros2 launch agv_sim_bringup sim_mapping.launch.py
  # Drive robot with: ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/agv/cmd_vel
  # Save map: ros2 run nav2_map_server map_saver_cli -f ~/greenhouse_map --ros-args -r map:=/agv/map
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

    slam_params = os.path.join(bringup_pkg, 'config', 'slam_toolbox_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='agv'),
        DeclareLaunchArgument('world', default_value='greenhouse_simple'),
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

        # Spawn robot — in mapping mode, diff_drive publishes odom TF
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

        # slam_toolbox — online async mapping mode
        # Delayed to let TF settle after robot spawn
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='slam_toolbox',
                    executable='async_slam_toolbox_node',
                    name='slam_toolbox',
                    namespace=ns,
                    parameters=[slam_params],
                    remappings=[
                        ('scan', 'scan'),
                    ],
                    output='screen',
                ),
            ],
        ),
    ])
