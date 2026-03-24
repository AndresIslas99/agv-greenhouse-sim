"""
Spawn the AGV robot URDF into Gazebo.

Reusable robot spawner used by all sim launch files.
Publishes robot_description and spawns the model at a configurable pose.

Usage:
  ros2 launch agv_sim_bringup spawn_robot.launch.py
  ros2 launch agv_sim_bringup spawn_robot.launch.py x:=2.0 y:=-3.0 yaw:=1.57
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_pkg = get_package_share_directory('agv_sim_description')
    xacro_file = os.path.join(desc_pkg, 'urdf', 'agv_sim.urdf.xacro')

    ns = LaunchConfiguration('namespace')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    publish_odom_tf = LaunchConfiguration('publish_odom_tf')

    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' publish_odom_tf:=', publish_odom_tf,
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='agv'),
        DeclareLaunchArgument('x', default_value='2.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.2'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('publish_odom_tf', default_value='false',
                              description='Let diff_drive publish odom->base_link TF (true for teleop, false for EKF modes)'),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=ns,
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),

        # Spawn robot into Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_agv',
            namespace=ns,
            arguments=[
                '-entity', 'agv',
                '-topic', 'robot_description',
                '-x', x,
                '-y', y,
                '-z', z,
                '-Y', yaw,
            ],
            output='screen',
        ),
    ])
