"""
Launch Gz Sim with the greenhouse world.

Usage:
  ros2 launch agv_sim_worlds greenhouse.launch.py
  ros2 launch agv_sim_worlds greenhouse.launch.py world:=nav_test.sdf
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    worlds_pkg = get_package_share_directory('agv_sim_worlds')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    world_file = LaunchConfiguration('world')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(worlds_pkg, 'worlds', 'greenhouse_simple.sdf'),
            description='Path to world file',
        ),

        # Force NVIDIA GPU for Gz rendering (PRIME offload on hybrid laptops)
        SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
        SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
        SetEnvironmentVariable('__VK_LAYER_NV_optimus', 'NVIDIA_only'),
        SetEnvironmentVariable('__EGL_VENDOR_LIBRARY_FILENAMES',
                               '/usr/share/glvnd/egl_vendor.d/10_nvidia.json'),

        # Gz transport over loopback (avoids multicast issues)
        SetEnvironmentVariable('GZ_IP', '127.0.0.1'),

        # Add our models to the Gz resource path
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(worlds_pkg, 'models'),
        ),

        # Start Gz Sim (server + GUI)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py'),
            ),
            launch_arguments={
                'gz_args': ['-r ', world_file],
            }.items(),
        ),
    ])
