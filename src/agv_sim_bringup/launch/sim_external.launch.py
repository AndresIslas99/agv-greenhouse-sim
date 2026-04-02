"""
AGV Sim External Brain Mode — PC provides world/body/sensors, Jetson runs the brain.

The PC acts as a virtual robot body:
  - Gz Sim with greenhouse world (physics + sensors)
  - ros_gz_bridge exposes sensor topics to the ROS 2 network
  - drive_shaping_node shapes cmd_vel for the Gz DiffDrive plugin
  - robot_state_publisher publishes URDF TF tree

The Jetson (or any external machine on the same ROS 2 network) runs:
  - EKF sensor fusion (odom->base_link, map->odom)
  - SLAM (slam_toolbox or cuVSLAM)
  - Nav2 autonomous navigation
  - AprilTag detection
  - Mission logic / UI

Topics published by this launch (PC -> network):
  Lightweight (always):
    /clock                              ~16 KB/s
    /agv/wheel_odom                     ~15 KB/s
    /zed/zed_node/imu/data              ~60 KB/s
    /agv/scan                           ~60 KB/s
    /agv/joint_states                   ~10 KB/s
    /tf_static                          negligible
  Heavy (lazy — only when subscribed):
    /zed/zed_node/left/image_rect_color ~81 MB/s (NOT WiFi safe)
    /zed/zed_node/depth/depth_registered ~108 MB/s (NOT WiFi safe)
    /zed/zed_node/point_cloud/...       ~330 MB/s (NOT WiFi safe)

Topics consumed from network (Jetson -> PC):
    /agv/cmd_vel    (geometry_msgs/Twist, from Nav2 or teleop)
    /agv/e_stop     (std_msgs/Bool)

Usage:
  PC:     ros2 launch agv_sim_bringup sim_external.launch.py
  Jetson: ros2 topic list  # should see /agv/wheel_odom, /zed/zed_node/imu/data, etc.
  Jetson: ros2 topic pub /agv/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}" -r 10
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
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    worlds_pkg = get_package_share_directory('agv_sim_worlds')
    bringup_pkg = get_package_share_directory('agv_sim_bringup')
    drive_pkg = get_package_share_directory('agv_sim_drive')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    ns = LaunchConfiguration('namespace')
    world_name = LaunchConfiguration('world')

    drive_params = os.path.join(drive_pkg, 'config', 'drive_shaping_params.yaml')
    bridge_config = os.path.join(bringup_pkg, 'config', 'gz_bridge.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='agv'),
        DeclareLaunchArgument('world', default_value='greenhouse_simple',
                              description='World name without .sdf extension'),
        DeclareLaunchArgument('x', default_value='2.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

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

        # Spawn robot (includes robot_state_publisher for URDF TF + model spawn)
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

        # ros_gz_bridge — all sensor/odom/clock bridges
        # Bridges are lazy: heavy topics (cameras) only stream when subscribed
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            parameters=[{'config_file': bridge_config}],
            output='screen',
        ),

        # NO odom TF bridge — the external brain (Jetson) owns odom->base_link via EKF

        # Motor arming gate — emulates ODrive arm/disarm for the real GUI
        # Subscribes: motor_enable (Bool), cmd_vel (Twist)
        # Publishes: motor_state (String/JSON), cmd_vel_armed (Twist)
        Node(
            package='agv_sim_bringup',
            executable='sim_motor_gate.py',
            name='sim_motor_gate',
            namespace=ns,
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),

        # Drive shaping: cmd_vel_armed (from motor gate) -> shaped_cmd_vel (to Gz DiffDrive)
        # Remapped from cmd_vel to cmd_vel_armed so motor gate controls access
        Node(
            package='agv_sim_drive',
            executable='sim_drive_shaping_node',
            name='sim_drive_shaping_node',
            namespace=ns,
            parameters=[drive_params, {'use_sim_time': True}],
            remappings=[('cmd_vel', 'cmd_vel_armed')],
            output='screen',
        ),

        # Simulated global odometry — replaces cuVSLAM for HIL
        # Relays wheel_odom with frame_id='map' on /visual_slam/tracking/odometry
        # so the Jetson's global EKF gets the same topic contract as real hardware
        Node(
            package='agv_sim_bringup',
            executable='sim_global_odom.py',
            name='sim_global_odom',
            namespace=ns,
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),

        # ZED 2i point cloud -> LaserScan (matches real robot pipeline)
        # Real: ZED → point cloud → pointcloud_to_laserscan (on Jetson) → /agv/scan
        # Sim:  ZED → point cloud → pointcloud_to_laserscan (on PC)     → /agv/scan → network
        # Same algorithm, same height filter params. Only ~10KB/frame over network.
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            namespace=ns,
            parameters=[{
                'min_height': 0.05,
                'max_height': 1.20,
                'range_min': 0.3,
                'range_max': 10.0,
                'angle_min': -1.5708,
                'angle_max': 1.5708,
                'angle_increment': 0.00436,
                'scan_time': 0.033,
                'target_frame': 'base_link',
                'use_sim_time': True,
            }],
            remappings=[
                ('cloud_in', '/zed/zed_node/point_cloud/cloud_registered'),
                ('scan', 'scan'),
            ],
            output='screen',
        ),
    ])
