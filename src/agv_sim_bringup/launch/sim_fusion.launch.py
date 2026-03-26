"""
AGV Sim Fusion Mode — teleop + slam_toolbox (localization) + dual EKF.

Launches:
  - Gz Sim with greenhouse_simple.sdf
  - Robot spawned at starting position
  - Robot state publisher (sim URDF)
  - ros_gz_bridge (Gz Transport <-> ROS 2 topics, NO odom TF)
  - ODrive-realistic drive shaping node (cmd_vel -> shaped_cmd_vel)
  - slam_toolbox in localization mode (loads a saved map)
  - Dual EKF:
    - ekf_local: wheel_odom + ZED 2i IMU -> odom->base_link
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
    drive_pkg = get_package_share_directory('agv_sim_drive')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    ns = LaunchConfiguration('namespace')
    world_name = LaunchConfiguration('world')
    map_file = LaunchConfiguration('map')

    ekf_local_config = os.path.join(bringup_pkg, 'config', 'sim_ekf_local.yaml')
    ekf_global_config = os.path.join(bringup_pkg, 'config', 'sim_ekf_global.yaml')
    slam_params = os.path.join(bringup_pkg, 'config', 'slam_toolbox_params.yaml')
    drive_params = os.path.join(drive_pkg, 'config', 'drive_shaping_params.yaml')
    bridge_config = os.path.join(bringup_pkg, 'config', 'gz_bridge.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='agv'),
        DeclareLaunchArgument('world', default_value='greenhouse_simple'),
        DeclareLaunchArgument('x', default_value='2.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('map', default_value='',
                              description='Path to map YAML for localization (without extension)'),

        # Force NVIDIA GPU for Gz rendering (PRIME offload on hybrid laptops)
        SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
        SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
        SetEnvironmentVariable('__VK_LAYER_NV_optimus', 'NVIDIA_only'),
        SetEnvironmentVariable('__EGL_VENDOR_LIBRARY_FILENAMES',
                               '/usr/share/glvnd/egl_vendor.d/10_nvidia.json'),

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

        # Spawn robot — EKF owns odom TF in fusion mode (no TF bridge)
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

        # ros_gz_bridge — sensor/odom/clock bridges (NO odom TF bridge in EKF mode)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            parameters=[{'config_file': bridge_config}],
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
                        {'use_sim_time': True},
                        # EKF global owns map->odom TF; slam_toolbox must NOT compete
                        {'publish_tf': False},
                    ],
                    remappings=[
                        ('scan', 'scan'),
                    ],
                    output='screen',
                ),
            ],
        ),

        # Dual EKF — local: wheel_odom + ZED 2i IMU -> odom->base_link
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='robot_localization',
                    executable='ekf_node',
                    name='ekf_local',
                    namespace=ns,
                    parameters=[ekf_local_config, {'use_sim_time': True}],
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
                    parameters=[ekf_global_config, {'use_sim_time': True}],
                    remappings=[
                        ('odometry/filtered', 'odometry/global'),
                    ],
                    output='screen',
                ),
            ],
        ),
    ])
