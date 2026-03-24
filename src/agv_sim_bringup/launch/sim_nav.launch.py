"""
AGV Sim Navigation Mode — full Nav2 stack with localization + EKF.

This is the key deliverable: robot can launch in sim, localize,
and execute Nav2 A->B routes.

Launches:
  - Gazebo with greenhouse_simple.world
  - Robot spawned at starting position
  - Robot state publisher (sim URDF)
  - ODrive-realistic drive shaping node (cmd_vel → shaped_cmd_vel)
  - Diff drive plugin: publish_odom_tf=false (EKF owns it)
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
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    ns = LaunchConfiguration('namespace')
    world_name = LaunchConfiguration('world')
    map_file = LaunchConfiguration('map')

    ekf_local_config = os.path.join(bringup_pkg, 'config', 'sim_ekf_local.yaml')
    ekf_global_config = os.path.join(bringup_pkg, 'config', 'sim_ekf_global.yaml')
    slam_params = os.path.join(bringup_pkg, 'config', 'slam_toolbox_params.yaml')
    nav2_params = os.path.join(nav_pkg, 'config', 'nav2_params.yaml')
    drive_params = os.path.join(drive_pkg, 'config', 'drive_shaping_params.yaml')

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
                'publish_odom_tf': 'false',
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

        # Dual EKF — local filter
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

        # Dual EKF — global filter
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
                    parameters=[nav2_params],
                    output='screen',
                ),

                # Controller server (RegulatedPurePursuit)
                Node(
                    package='nav2_controller',
                    executable='controller_server',
                    name='controller_server',
                    namespace=ns,
                    parameters=[nav2_params],
                    output='screen',
                ),

                # Behavior server (spin, backup, wait)
                Node(
                    package='nav2_behaviors',
                    executable='behavior_server',
                    name='behavior_server',
                    namespace=ns,
                    parameters=[nav2_params],
                    output='screen',
                ),

                # BT Navigator
                Node(
                    package='nav2_bt_navigator',
                    executable='bt_navigator',
                    name='bt_navigator',
                    namespace=ns,
                    parameters=[nav2_params],
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
                        'node_names': [
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
