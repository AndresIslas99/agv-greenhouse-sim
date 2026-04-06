"""
AGV Isaac Sim Base Launch — GPU-accelerated greenhouse simulation.

Launches:
  - Isaac Sim (standalone or connects to running instance)
  - Greenhouse world USD
  - Robot USD (diff-drive, ZED 2i, IMU — all sensors publishing ROS 2)
  - Robot state publisher (URDF → /tf_static)
  - Drive shaping node (cmd_vel → shaped_cmd_vel)
  - Isaac ROS bridge helper (IMU relay)

Prerequisites:
  - Isaac Sim 4.x installed
  - USD files generated (run build_greenhouse_usd.py + import_robot_usd.py)

Usage:
  ros2 launch agv_isaac_sim isaac_sim.launch.py
  ros2 launch agv_isaac_sim isaac_sim.launch.py headless:=true
  ros2 launch agv_isaac_sim isaac_sim.launch.py x:=5.0 y:=-4.4
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    isaac_pkg = get_package_share_directory('agv_isaac_sim')
    description_pkg = get_package_share_directory('agv_sim_description')
    drive_pkg = get_package_share_directory('agv_sim_drive')

    drive_params = os.path.join(drive_pkg, 'config', 'drive_shaping_params.yaml')
    world_usd = os.path.join(isaac_pkg, 'worlds', 'greenhouse_simple.usd')
    robot_usd = os.path.join(isaac_pkg, 'robot', 'agv_sim.usd')

    # Process URDF for robot_state_publisher
    xacro_file = os.path.join(description_pkg, 'urdf', 'agv_sim.urdf.xacro')

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('namespace', default_value='agv'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run Isaac Sim in headless mode'),
        DeclareLaunchArgument('x', default_value='2.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        # Ensure use_sim_time for all nodes
        SetEnvironmentVariable('ROS_PARAM_FILE', ''),

        # Isaac Sim launcher
        # Isaac Sim is expected to be running separately or launched via
        # the isaacsim command. This launch file starts the ROS 2 side.
        # To auto-launch Isaac Sim, uncomment the ExecuteProcess below
        # and set ISAACSIM_PATH environment variable.
        #
        # ExecuteProcess(
        #     cmd=[
        #         os.environ.get('ISAACSIM_PATH', 'isaacsim'),
        #         '--open-usd', world_usd,
        #         '--ros2',
        #     ],
        #     output='screen',
        # ),

        # Robot state publisher (URDF → /tf_static + /robot_description)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': Command(['xacro ', xacro_file]),
                'use_sim_time': True,
            }],
            output='screen',
        ),

        # ODrive-realistic drive shaping: cmd_vel → shaped_cmd_vel
        Node(
            package='agv_sim_drive',
            executable='sim_drive_shaping_node',
            name='sim_drive_shaping_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=[drive_params, {'use_sim_time': True}],
            output='screen',
        ),

        # Isaac ROS bridge helper (IMU relay)
        Node(
            package='agv_isaac_sim',
            executable='isaac_ros_bridge_node.py',
            name='isaac_ros_bridge',
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),

        # Left camera lens distortion (ZED 2i barrel distortion)
        Node(
            package='agv_isaac_sim',
            executable='sim_lens_distortion.py',
            name='sim_lens_distortion_left',
            parameters=[{
                'use_sim_time': True,
                'k1': -0.0425, 'k2': 0.0105,
                'p1': 0.0, 'p2': 0.0, 'k3': 0.0,
                'fx': 527.5, 'fy': 527.5, 'cx': 640.0, 'cy': 360.0,
                'width': 1280, 'height': 720,
            }],
            remappings=[
                ('image_clean', '/zed/zed_node/left/image_clean'),
                ('image_raw', '/zed/zed_node/left/image_raw'),
                ('image_rect_color', '/zed/zed_node/left/image_rect_color'),
                ('camera_info', '/zed/zed_node/left/camera_info'),
            ],
            output='screen',
        ),

        # Right camera lens distortion
        Node(
            package='agv_isaac_sim',
            executable='sim_lens_distortion.py',
            name='sim_lens_distortion_right',
            parameters=[{
                'use_sim_time': True,
                'k1': -0.0425, 'k2': 0.0105,
                'p1': 0.0, 'p2': 0.0, 'k3': 0.0,
                'fx': 527.5, 'fy': 527.5, 'cx': 640.0, 'cy': 360.0,
                'width': 1280, 'height': 720,
            }],
            remappings=[
                ('image_clean', '/zed/zed_node/right/image_clean'),
                ('image_raw', '/zed/zed_node/right/image_raw'),
                ('image_rect_color', '/zed/zed_node/right/image_rect_color'),
                ('camera_info', '/zed/zed_node/right/camera_info'),
            ],
            output='screen',
        ),

        # Depth noise injector — adds calibrated distance-dependent noise
        # to match real ZED 2i depth accuracy: σ(d) = 0.002 + 0.005×d
        Node(
            package='agv_isaac_sim',
            executable='sim_depth_noise.py',
            name='sim_depth_noise',
            parameters=[{
                'use_sim_time': True,
                'noise_base': 0.002,
                'noise_scale': 0.005,
                'dropout_distance': 15.0,
                'dropout_rate': 0.3,
            }],
            remappings=[
                ('depth_raw', '/zed/zed_node/depth/raw'),
                ('depth_registered', '/zed/zed_node/depth/depth_registered'),
            ],
            output='screen',
        ),
    ])
