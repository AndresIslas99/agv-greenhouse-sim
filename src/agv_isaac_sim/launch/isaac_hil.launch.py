"""
AGV Isaac Sim HIL — virtual body for the brain running on the Jetson.

Composes three pieces:
  1. isaac_sim.launch.py (sensor base: robot_state_publisher, static TFs,
     IMU bias drift relay, depth noise relay)
  2. Drive chain: sim_motor_gate → sim_drive_shaping_node → /agv/shaped_cmd_vel
  3. Brain-facing shims: sim_global_odom (cuVSLAM replacement),
     sim_wheel_odom_publisher (encoder-style /agv/wheel_odom)

Topics this launch guarantees to the brain network:
  /clock                                 (from Isaac sim time)
  /tf, /tf_static                        (robot_state_publisher + static TFs)
  /agv/wheel_odom                        50 Hz  (sim_wheel_odom_publisher)
  /agv/joint_states                      50 Hz  (OmniGraph)
  /agv/imu/data                          200 Hz (relay with bias drift)
  /agv/zed/left/image_rect_color         15 Hz  (OmniGraph)
  /agv/zed/left/camera_info              15 Hz
  /agv/zed/depth/depth_registered        15 Hz  (relay with distance noise)
  /agv/zed/point_cloud/cloud_registered  15 Hz  (OmniGraph)
  /visual_slam/tracking/odometry         (sim_global_odom — cuVSLAM replacement)
  /agv/motor_state                       10 Hz  (sim_motor_gate JSON)
  /agv/drive_debug                       10 Hz

NOT published here (brain owns it on its side, same as real robot):
  /agv/scan         pointcloud_to_laserscan from cloud_registered.
                    Must run on the Jetson next to Nav2.

Subscribes (brain publishes):
  /agv/cmd_vel           (mapping-first mode) OR
  /agv/cmd_vel_safe      (has_map mode — via topic_tools relay)
  /agv/motor_enable
  /agv/e_stop

Network:
  ROS_DOMAIN_ID=42 is enforced — matches brain/specs/launch_sequence.yaml.

Usage:
  # Launcher wrapper already sets ROS_DOMAIN_ID=42:
  ros2 launch agv_isaac_sim isaac_hil.launch.py

  # In has_map mode:
  ros2 launch agv_isaac_sim isaac_hil.launch.py has_map:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    drive_pkg = get_package_share_directory('agv_sim_drive')
    drive_params = os.path.join(drive_pkg, 'config', 'drive_shaping_params.yaml')

    ns = LaunchConfiguration('namespace')
    has_map = LaunchConfiguration('has_map')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='agv'),
        DeclareLaunchArgument(
            'has_map', default_value='false',
            description='If true, motor gate also accepts /agv/cmd_vel_safe via topic_tools relay.'),
        DeclareLaunchArgument(
            'validation', default_value='false',
            description='If true, also launch the AI validation overlay (ground truth + events + foxglove).'),

        # Force the production ROS_DOMAIN_ID so sim + relays + brain share one domain.
        SetEnvironmentVariable('ROS_DOMAIN_ID', '42'),

        # ── Sensor base: URDF + static TFs + realism relays ──
        # isaac_sim.launch.py provides robot_state_publisher, static TFs for
        # imu_link / zed_* frames, isaac_ros_bridge (IMU bias drift), and
        # sim_depth_noise (distance-dependent Gaussian).
        #
        # In a production setup where the brain also runs agv_description's
        # description.launch.py and agv_slam.launch.py's static TFs, there will
        # be duplicate URDF TF publishers. This is intentional for testing
        # without a brain; for production, disable this include or launch the
        # HIL on a different machine than the brain.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('agv_isaac_sim'), 'launch', 'isaac_sim.launch.py',
                ]),
            ),
            launch_arguments={'namespace': ns}.items(),
        ),

        # ── Drive chain ────────────────────────────────────────────────────

        # ODrive arm/disarm emulator: /agv/cmd_vel → /agv/cmd_vel_armed
        # Also publishes /agv/motor_state and /agv/drive_debug JSON at 10 Hz
        # matching the real agv_odrive contract.
        Node(
            package='agv_isaac_sim',
            executable='sim_motor_gate.py',
            name='sim_motor_gate',
            namespace=ns,
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),

        # In has_map mode the brain publishes to /agv/cmd_vel_safe after its
        # own safety gate. Bridge it into /agv/cmd_vel so the motor gate
        # sees a single input.
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
        # (Isaac's DifferentialController subscribes to shaped_cmd_vel.)
        Node(
            package='agv_sim_drive',
            executable='sim_drive_shaping_node',
            name='sim_drive_shaping_node',
            namespace=ns,
            parameters=[drive_params, {'use_sim_time': True}],
            remappings=[('cmd_vel', 'cmd_vel_armed')],
            output='screen',
        ),

        # ── Brain-facing shims ─────────────────────────────────────────────

        # Wheel-encoder odometry from /agv/joint_states (replaces the broken
        # OmniGraph IsaacComputeOdometry → ROS2PublishOdometry chain that
        # was producing wrong-signed pose.x and zero covariance, which
        # collapsed the brain's ekf_local). The OmniGraph publisher now
        # publishes to /agv/_sim_internal/isaac_compute_odom_raw for
        # diagnostic comparison only — /agv/wheel_odom comes from here.
        Node(
            package='agv_isaac_sim',
            executable='sim_wheel_odom_publisher.py',
            name='sim_wheel_odom_publisher',
            namespace=ns,
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),

        # cuVSLAM replacement for HIL: wheel_odom republished with frame_id=map.
        # Brain's ekf_global consumes this as odom1 with differential:true so
        # absolute drift doesn't matter — only per-tick deltas.
        Node(
            package='agv_isaac_sim',
            executable='sim_global_odom.py',
            name='sim_global_odom',
            namespace=ns,
            parameters=[{'use_sim_time': True}],
            remappings=[('wheel_odom', '/agv/wheel_odom')],
            output='screen',
        ),

        # NOTE: pointcloud_to_laserscan and any /agv/scan publisher are
        # explicitly NOT here. That conversion is brain work — on the real
        # robot it runs on the Jetson next to Nav2, so it must run on the
        # Jetson in HIL mode too. The sim's only job is to publish the raw
        # /agv/zed/point_cloud/cloud_registered topic; the brain consumes
        # it and produces /agv/scan with whatever params Nav2 expects.
        # See CLAUDE.md "the sim is the body, not the brain".

        # ── AI validation overlay ─────────────────────────────────────────
        # Optional: enable with validation:=true. Adds ground-truth pose,
        # event detector, localization monitor and foxglove_bridge under
        # /agv/sim/* topics. See agv_sim_validation/README.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('agv_sim_validation'), 'launch',
                    'validation_overlay.launch.py',
                ]),
            ),
            condition=IfCondition(LaunchConfiguration('validation')),
        ),
    ])
