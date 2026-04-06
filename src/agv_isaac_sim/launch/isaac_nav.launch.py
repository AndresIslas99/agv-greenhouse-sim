"""
AGV Isaac Sim Navigation Mode — full Nav2 stack with Isaac Sim.

Launches:
  - isaac_sim.launch.py (base)
  - sim_global_odom.py (simulated cuVSLAM)
  - sim_motor_gate.py (ODrive gate)
  - Dual EKF (local + global)
  - SLAM Toolbox
  - Nav2 stack
  - depthimage_to_laserscan (ZED depth → /agv/scan)

Usage:
  ros2 launch agv_isaac_sim isaac_nav.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_pkg = get_package_share_directory('agv_sim_bringup')
    nav_pkg = get_package_share_directory('agv_sim_nav')

    ekf_local_cfg = os.path.join(bringup_pkg, 'config', 'sim_ekf_local.yaml')
    ekf_global_cfg = os.path.join(bringup_pkg, 'config', 'sim_ekf_global.yaml')
    slam_cfg = os.path.join(bringup_pkg, 'config', 'slam_toolbox_params.yaml')
    nav2_cfg = os.path.join(nav_pkg, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        # Base Isaac Sim launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('agv_isaac_sim'), 'launch', 'isaac_sim.launch.py',
                ]),
            ),
        ),

        # Simulated cuVSLAM global odometry
        Node(
            package='agv_sim_bringup',
            executable='sim_global_odom.py',
            name='sim_global_odom',
            namespace='agv',
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),

        # ODrive arm/disarm gate
        Node(
            package='agv_sim_bringup',
            executable='sim_motor_gate.py',
            name='sim_motor_gate',
            namespace='agv',
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),

        # Depth image to laser scan (ZED depth → /agv/scan)
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[{
                'use_sim_time': True,
                'target_frame': 'base_link',
                'min_height': -0.1,
                'max_height': 0.5,
                'range_min': 0.3,
                'range_max': 10.0,
                'angle_min': -1.5708,
                'angle_max': 1.5708,
            }],
            remappings=[
                ('cloud_in', '/zed/zed_node/point_cloud/cloud_registered'),
                ('scan', '/agv/scan'),
            ],
            output='screen',
        ),

        # Local EKF (odom → base_link)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            parameters=[ekf_local_cfg, {'use_sim_time': True}],
            output='screen',
        ),

        # Global EKF (map → odom)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            parameters=[ekf_global_cfg, {'use_sim_time': True}],
            output='screen',
        ),

        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[slam_cfg, {'use_sim_time': True}],
            output='screen',
        ),

        # Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py',
                ]),
            ),
            launch_arguments={
                'params_file': nav2_cfg,
                'use_sim_time': 'true',
            }.items(),
        ),
    ])
