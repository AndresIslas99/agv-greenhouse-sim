"""
AGV Sim Navigation Mode — full Nav2 stack with localization + EKF.

This is the key deliverable: robot can launch in sim, localize,
and execute Nav2 A->B routes.

Launches:
  - Gz Sim with greenhouse_simple.sdf
  - Robot spawned at starting position
  - Robot state publisher (sim URDF)
  - ros_gz_bridge (Gz Transport <-> ROS 2 topics, NO odom TF)
  - ODrive-realistic drive shaping node (cmd_vel -> shaped_cmd_vel)
  - slam_toolbox in localization mode
  - Dual EKF (local + global)
  - Full Nav2 stack:
    - bt_navigator
    - planner_server (SmacPlannerHybrid)
    - controller_server (RegulatedPurePursuit)
    - behavior_server (spin, backup, wait)
    - costmap (global + local)
    - lifecycle manager
  - Optional: apriltag_ros detection on ZED left camera

Usage:
  ros2 launch agv_sim_bringup sim_nav.launch.py map:=/path/to/greenhouse_map.yaml
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
    nav_pkg = get_package_share_directory('agv_sim_nav')
    drive_pkg = get_package_share_directory('agv_sim_drive')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    ns = LaunchConfiguration('namespace')
    world_name = LaunchConfiguration('world')
    map_file = LaunchConfiguration('map')
    map_yaml = LaunchConfiguration('map_yaml')

    ekf_local_config = os.path.join(bringup_pkg, 'config', 'sim_ekf_local.yaml')
    ekf_global_config = os.path.join(bringup_pkg, 'config', 'sim_ekf_global.yaml')
    slam_params = os.path.join(bringup_pkg, 'config', 'slam_toolbox_params.yaml')
    nav2_params = os.path.join(nav_pkg, 'config', 'nav2_params.yaml')
    drive_params = os.path.join(drive_pkg, 'config', 'drive_shaping_params.yaml')
    bridge_config = os.path.join(bringup_pkg, 'config', 'gz_bridge.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='agv'),
        DeclareLaunchArgument('world', default_value='greenhouse_simple'),
        DeclareLaunchArgument('x', default_value='2.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('map', default_value='',
                              description='Path to slam_toolbox serialized map (without extension)'),
        DeclareLaunchArgument('map_yaml',
                              default_value=os.path.join(bringup_pkg, 'maps', 'greenhouse_simple.yaml'),
                              description='Path to .yaml map file for Nav2 map_server'),

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

        # Spawn robot — EKF owns odom TF
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

        # Dual EKF — local filter
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

        # Dual EKF — global filter
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

        # apriltag_ros detection on ZED left camera
        TimerAction(
            period=6.0,
            actions=[
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
            ],
        ),

        # Map server — serves the static map for Nav2 global costmap
        TimerAction(
            period=7.0,
            actions=[
                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='map_server',
                    namespace=ns,
                    parameters=[{
                        'yaml_filename': map_yaml,
                        'use_sim_time': True,
                    }],
                    output='screen',
                ),
            ],
        ),

        # Nav2 stack
        TimerAction(
            period=8.0,
            actions=[
                # Planner server (SmacPlannerHybrid)
                Node(
                    package='nav2_planner',
                    executable='planner_server',
                    name='planner_server',
                    namespace=ns,
                    parameters=[nav2_params, {'use_sim_time': True}],
                    output='screen',
                ),

                # Controller server (RegulatedPurePursuit)
                Node(
                    package='nav2_controller',
                    executable='controller_server',
                    name='controller_server',
                    namespace=ns,
                    parameters=[nav2_params, {'use_sim_time': True}],
                    output='screen',
                ),

                # Behavior server (spin, backup, wait)
                Node(
                    package='nav2_behaviors',
                    executable='behavior_server',
                    name='behavior_server',
                    namespace=ns,
                    parameters=[nav2_params, {'use_sim_time': True}],
                    output='screen',
                ),

                # BT Navigator
                Node(
                    package='nav2_bt_navigator',
                    executable='bt_navigator',
                    name='bt_navigator',
                    namespace=ns,
                    parameters=[nav2_params, {'use_sim_time': True}],
                    output='screen',
                ),

                # Lifecycle manager for Nav2 nodes
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_navigation',
                    namespace=ns,
                    parameters=[{
                        'autostart': True,
                        'use_sim_time': True,
                        'node_names': [
                            'map_server',
                            'planner_server',
                            'controller_server',
                            'behavior_server',
                            'bt_navigator',
                        ],
                    }],
                    output='screen',
                ),
            ],
        ),
    ])
