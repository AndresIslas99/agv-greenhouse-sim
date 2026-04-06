"""
AGV Isaac Sim External (HIL) Mode — Jetson brain drives the sim.

Launches:
  - isaac_sim.launch.py (base)
  - sim_global_odom.py (simulated cuVSLAM output)
  - sim_motor_gate.py (ODrive arm/disarm emulation)
  - CycloneDDS configured for network transport

The Jetson runs EKF, Nav2, SLAM, and cuVSLAM — this launch only
provides the simulated body (sensors + actuators).

Usage:
  ros2 launch agv_isaac_sim isaac_external.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_pkg = get_package_share_directory('agv_sim_bringup')

    # CycloneDDS config for network (same as Gazebo external mode)
    cyclone_cfg = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(bringup_pkg))),
        'cyclonedds.xml')

    return LaunchDescription([
        # CycloneDDS for cross-machine ROS 2 (PC ↔ Jetson)
        SetEnvironmentVariable('CYCLONEDDS_URI', f'file://{cyclone_cfg}'),

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
    ])
