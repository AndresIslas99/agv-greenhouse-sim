"""
AGV Sim Mapping Mode — teleop + slam_toolbox for map creation.

Launches:
  - Gz Sim with greenhouse_simple.sdf
  - Robot spawned at starting position
  - Robot state publisher (sim URDF)
  - ros_gz_bridge (Gz Transport <-> ROS 2 topics)
  - ODrive-realistic drive shaping node (cmd_vel -> shaped_cmd_vel)
  - slam_toolbox in online_async mapping mode
  - In this mode, slam_toolbox owns map->odom TF
  - Diff drive TF bridged for odom->base_link

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
    drive_pkg = get_package_share_directory('agv_sim_drive')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    ns = LaunchConfiguration('namespace')
    world_name = LaunchConfiguration('world')

    slam_params = os.path.join(bringup_pkg, 'config', 'slam_toolbox_params.yaml')
    drive_params = os.path.join(drive_pkg, 'config', 'drive_shaping_params.yaml')
    bridge_config = os.path.join(bringup_pkg, 'config', 'gz_bridge.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='agv'),
        DeclareLaunchArgument('world', default_value='greenhouse_simple'),
        DeclareLaunchArgument('x', default_value='2.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        # Gz transport over loopback (avoids multicast issues)
        SetEnvironmentVariable('GZ_IP', '127.0.0.1'),

        # Gz resource path
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(worlds_pkg, 'models'),
        ),

        # Start Gz Sim
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py'),
            ),
            launch_arguments={
                'gz_args': [
                    '-r ',
                    PathJoinSubstitution([
                        FindPackageShare('agv_sim_worlds'), 'worlds',
                        [world_name, '.sdf'],
                    ]),
                ],
            }.items(),
        ),

        # Spawn robot
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
            }.items(),
        ),

        # ros_gz_bridge — sensor/odom/clock bridges
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            parameters=[{'config_file': bridge_config}],
            output='screen',
        ),

        # ros_gz_bridge — odom TF (mapping mode: diff_drive publishes odom->base_link)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge_tf',
            arguments=['/agv/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
            remappings=[('/agv/tf', '/tf')],
            output='screen',
        ),

        # ODrive-realistic drive shaping: cmd_vel -> shaped_cmd_vel
        Node(
            package='agv_sim_drive',
            executable='sim_drive_shaping_node',
            name='sim_drive_shaping_node',
            namespace=ns,
            parameters=[drive_params, {'use_sim_time': True}],
            output='screen',
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
                    parameters=[slam_params, {'use_sim_time': True}],
                    remappings=[
                        ('scan', 'scan'),
                    ],
                    output='screen',
                ),
            ],
        ),
    ])
