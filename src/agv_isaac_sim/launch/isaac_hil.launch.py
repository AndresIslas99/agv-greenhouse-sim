"""
AGV Isaac Sim HIL — virtual body for the real brain running on the Jetson.

Publishes exactly the topic contract the brain's agv_hil_full.launch.py expects.
No EKF, no SLAM, no Nav2, no AprilTag detection — those are the brain's job.

Topics this launch guarantees to the brain network:
  /clock                            (from Isaac sim time)
  /agv/wheel_odom                   50 Hz  (from OmniGraph)
  /agv/joint_states                 50 Hz  (from OmniGraph)
  /agv/imu/data                     200 Hz (from OmniGraph → isaac_ros_bridge_node bias drift)
  /agv/zed/left/image_rect_color    15 Hz  (from OmniGraph)
  /agv/zed/left/camera_info         15 Hz
  /agv/zed/right/image_rect_color   15 Hz
  /agv/zed/right/camera_info        15 Hz
  /agv/zed/depth/depth_registered   15 Hz  (from OmniGraph → sim_depth_noise)
  /agv/zed/point_cloud/cloud_registered  15 Hz  (from OmniGraph)
  /agv/scan                         ~10 Hz (from pointcloud_to_laserscan running HERE,
                                     with the brain's exact production params)
  /visual_slam/tracking/odometry    ~10 Hz (from sim_global_odom — replaces cuVSLAM
                                     for HIL since there is no ZED GPU pipeline)
  /agv/motor_state                  10 Hz  (from sim_motor_gate JSON)
  /agv/drive_debug                  10 Hz

The brain subscribes to /agv/cmd_vel (mapping-first mode) or /agv/cmd_vel_safe
(has_map mode) to close the control loop.

Network setup (matches brain/specs/launch_sequence.yaml):
  - ROS_DOMAIN_ID=42 required (excludes rogue dev-PC publishers)
  - CYCLONEDDS_URI should point to cyclonedds.xml for cross-machine discovery

NOT launched here (brain owns these):
  - robot_state_publisher (brain's agv_description publishes the URDF)
  - static TFs for imu_link, zed_camera_link (brain's agv_slam.launch.py publishes them)
  - Any EKF / SLAM / Nav2 / apriltag_ros node

Usage:
  export ROS_DOMAIN_ID=42
  export CYCLONEDDS_URI=file://$(pwd)/cyclonedds.xml
  ros2 launch agv_isaac_sim isaac_hil.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    drive_pkg = get_package_share_directory('agv_sim_drive')
    drive_params = os.path.join(drive_pkg, 'config', 'drive_shaping_params.yaml')

    ns = LaunchConfiguration('namespace')
    has_map = LaunchConfiguration('has_map')

    # The drive shaping node reads cmd_vel from the motor gate (arm/disarm).
    # Motor gate subscribes to /agv/cmd_vel (mapping-first) or /agv/cmd_vel_safe (has_map).
    # The gate's output cmd_vel_armed feeds the drive shaping input.

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='agv'),
        DeclareLaunchArgument(
            'has_map', default_value='false',
            description='If true, motor gate subscribes to /agv/cmd_vel_safe '
                        '(output of brain safety gate) instead of /agv/cmd_vel. '
                        'Must match the brain launch has_map arg.'),

        # Force the production ROS_DOMAIN_ID. Brain requires this.
        SetEnvironmentVariable('ROS_DOMAIN_ID', '42'),

        # IMU bias drift relay (bias walk on top of clean Isaac IMU)
        Node(
            package='agv_isaac_sim',
            executable='isaac_ros_bridge_node.py',
            name='isaac_ros_bridge',
            namespace=ns,
            parameters=[{'use_sim_time': True}],
            remappings=[
                ('imu/data_clean', '/agv/imu/data_clean'),
                ('imu/data', '/agv/imu/data'),
            ],
            output='screen',
        ),

        # Depth noise relay (distance-dependent Gaussian + dropout beyond 15m)
        Node(
            package='agv_isaac_sim',
            executable='sim_depth_noise.py',
            name='sim_depth_noise',
            namespace=ns,
            parameters=[{
                'use_sim_time': True,
                'noise_base': 0.002,
                'noise_scale': 0.005,
                'dropout_distance': 15.0,
                'dropout_rate': 0.3,
            }],
            remappings=[
                ('depth_raw', '/agv/zed/depth/depth_clean'),
                ('depth_registered', '/agv/zed/depth/depth_registered'),
            ],
            output='screen',
        ),

        # ODrive arm/disarm + motor_state/drive_debug JSON
        Node(
            package='agv_isaac_sim',
            executable='sim_motor_gate.py',
            name='sim_motor_gate',
            namespace=ns,
            parameters=[{'use_sim_time': True}],
            remappings=[
                # In mapping-first mode the brain publishes /agv/cmd_vel directly.
                # In has_map mode the brain publishes /agv/cmd_vel_safe (after the
                # safety gate) and /agv/cmd_vel is a fallback we also accept.
            ],
            output='screen',
        ),
        # Alternate: in has_map mode, additionally subscribe motor gate to cmd_vel_safe
        # via a pass-through relay (topic_tools relay).
        Node(
            package='topic_tools',
            executable='relay',
            name='cmd_vel_safe_relay',
            namespace=ns,
            arguments=['cmd_vel_safe', 'cmd_vel'],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(has_map),
            output='screen',
        ),

        # ODrive acceleration ramp emulation: cmd_vel_armed → shaped_cmd_vel
        Node(
            package='agv_sim_drive',
            executable='sim_drive_shaping_node',
            name='sim_drive_shaping_node',
            namespace=ns,
            parameters=[drive_params, {'use_sim_time': True}],
            remappings=[('cmd_vel', 'cmd_vel_armed')],
            output='screen',
        ),

        # cuVSLAM replacement: wheel_odom relayed with frame_id=map.
        # Brain's ekf_global consumes this as odom1 in differential mode, so
        # absolute drift does not matter — only the per-tick deltas.
        Node(
            package='agv_isaac_sim',
            executable='sim_global_odom.py',
            name='sim_global_odom',
            namespace=ns,
            parameters=[{'use_sim_time': True}],
            remappings=[('wheel_odom', '/agv/wheel_odom')],
            output='screen',
        ),

        # pointcloud_to_laserscan runs on the sim PC in HIL mode, using the
        # brain's EXACT production parameters (agv_full.launch.py). This
        # keeps /agv/scan identical in sim and on the real Jetson.
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            namespace=ns,
            parameters=[{
                'use_sim_time':     True,
                'target_frame':     'base_link',
                'min_height':       0.01,
                'max_height':       2.0,
                'range_min':        0.3,
                'range_max':        8.0,
                'angle_min':        -1.5708,
                'angle_max':        1.5708,
                'angle_increment':  0.00436,
                'scan_time':        0.1,
                'queue_size':       10,
            }],
            remappings=[
                ('cloud_in', '/agv/zed/point_cloud/cloud_registered'),
                ('scan',     '/agv/scan'),
            ],
            output='screen',
        ),
    ])
