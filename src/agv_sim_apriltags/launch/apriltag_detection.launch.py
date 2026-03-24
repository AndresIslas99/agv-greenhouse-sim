"""
Launch apriltag_ros for camera-based tag detection on simulated ZED left camera.

Subscribes to the ZED left camera image and publishes tag detections.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('agv_sim_apriltags')
    params_file = os.path.join(pkg, 'config', 'apriltag_ros_params.yaml')

    ns = LaunchConfiguration('namespace')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='agv'),

        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            namespace=ns,
            parameters=[params_file],
            remappings=[
                ('image_rect', '/zed/zed_node/left/image_rect_color'),
                ('camera_info', '/zed/zed_node/left/camera_info'),
            ],
            output='screen',
        ),
    ])
