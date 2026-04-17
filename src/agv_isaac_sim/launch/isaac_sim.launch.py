"""
AGV Isaac Sim sensor base — the "virtual body".

This launch file only provides what the brain's HIL contract needs from the
hardware side. It does NOT wire up drive shaping or teleop; those are the
concern of the mode-specific launches that include this one:

  - isaac_teleop.launch.py  — local keyboard control (smoke test)
  - isaac_hil.launch.py     — production HIL: brain on Jetson drives sim

Components:
  - robot_state_publisher       URDF → /tf_static + /robot_description
  - static TFs                  imu_link, zed_camera_link, zed_left/right_camera_frame
  - isaac_ros_bridge_node       IMU bias drift: /agv/imu/data_clean → /agv/imu/data
  - sim_depth_noise             depth noise:    /agv/zed/depth/depth_clean → depth_registered
  - sim_odom_tf_broadcaster     odom → base_link TF (STANDALONE ONLY,
                                gated on standalone_mode:=true)

Launch args:
  namespace        ROS namespace (default 'agv').
  standalone_mode  When true, brings up TF/odom sources that the brain
                   normally owns in HIL. isaac_teleop.launch.py passes
                   true; isaac_hil.launch.py leaves it false.

Isaac Sim itself is started separately with:
    isaacsim --open-usd src/agv_isaac_sim/worlds/greenhouse_simple.usd --ros2
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    isaac_pkg = get_package_share_directory('agv_isaac_sim')
    xacro_file = os.path.join(isaac_pkg, 'urdf', 'agv_sim.urdf.xacro')

    ns = LaunchConfiguration('namespace')
    standalone_mode = LaunchConfiguration('standalone_mode')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='agv'),
        DeclareLaunchArgument(
            'standalone_mode', default_value='false',
            description=('When true, brings up sim-side replacements for '
                         'topics that the brain owns in HIL (e.g. the '
                         'odom→base_link TF that ekf_local provides). '
                         'isaac_teleop.launch.py sets this to true. HIL '
                         'mode (isaac_hil.launch.py) leaves it false so '
                         'the brain is authoritative.')),

        # Robot state publisher — URDF → /tf_static + /robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=ns,
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro ', xacro_file]), value_type=str),
                'use_sim_time': True,
            }],
            output='screen',
        ),

        # base_link → imu_link (BMI088 inside ZED 2i)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_imu_link',
            namespace=ns,
            arguments=['0.70', '0.0', '-0.055', '0', '0', '0', 'base_link', 'imu_link'],
            parameters=[{'use_sim_time': True}],
        ),

        # base_link → zed_camera_link (ZED 2i body)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_zed_camera_link',
            namespace=ns,
            arguments=['0.70', '0.0', '-0.055', '0', '0', '0', 'base_link', 'zed_camera_link'],
            parameters=[{'use_sim_time': True}],
        ),

        # zed_camera_link → zed_left_camera_frame (+Y half-baseline)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_zed_left',
            namespace=ns,
            arguments=['0', '0.06', '0', '0', '0', '0', 'zed_camera_link', 'zed_left_camera_frame'],
            parameters=[{'use_sim_time': True}],
        ),

        # zed_camera_link → zed_right_camera_frame (−Y half-baseline)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_zed_right',
            namespace=ns,
            arguments=['0', '-0.06', '0', '0', '0', '0', 'zed_camera_link', 'zed_right_camera_frame'],
            parameters=[{'use_sim_time': True}],
        ),

        # Optical frames — ROS convention: +Z forward, +X right, +Y down.
        # Rotation from body frame (x-forward, y-left, z-up) is rpy=(-π/2, 0, -π/2).
        # Isaac's CameraHelper publishes images/clouds in the *_optical frame.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_zed_left_optical',
            namespace=ns,
            arguments=['0', '0', '0', '-1.5707963', '0', '-1.5707963',
                       'zed_left_camera_frame', 'zed_left_camera_frame_optical'],
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_zed_right_optical',
            namespace=ns,
            arguments=['0', '0', '0', '-1.5707963', '0', '-1.5707963',
                       'zed_right_camera_frame', 'zed_right_camera_frame_optical'],
            parameters=[{'use_sim_time': True}],
        ),

        # IMU bias drift relay: /agv/imu/data_clean → /agv/imu/data
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

        # odom → base_link TF broadcaster (STANDALONE MODE ONLY).
        # In HIL mode the brain's ekf_local publishes this TF — running
        # this node would create a TF conflict. Gated on standalone_mode.
        Node(
            package='agv_isaac_sim',
            executable='sim_odom_tf_broadcaster.py',
            name='sim_odom_tf_broadcaster',
            namespace=ns,
            parameters=[{'use_sim_time': True}],
            remappings=[('wheel_odom', '/agv/wheel_odom')],
            condition=IfCondition(standalone_mode),
            output='screen',
        ),

        # Depth noise relay: /agv/zed/depth/depth_clean → /agv/zed/depth/depth_registered
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
    ])
