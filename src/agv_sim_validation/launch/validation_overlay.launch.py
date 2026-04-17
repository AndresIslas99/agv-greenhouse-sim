"""
AGV validation overlay — adds ground-truth, events, episode tracking,
foxglove bridge and (later) a FastAPI control surface to the running HIL sim.

Included from isaac_hil.launch.py with `validation:=true`. All topics live
under /agv/sim/* so the brain contract is unaffected.

Components:
  ground_truth_publisher  reads /tf and publishes /agv/sim/ground_truth/*
  localization_monitor    compares GT vs /agv/odometry/global, publishes error JSON
  events_detector         emits discrete events on /agv/sim/events
  episode_tracker         tracks Nav2 missions, publishes summaries (Phase 3)
  visible_markers         AprilTag GT visibility (Phase 4)
  obstacles_publisher     static obstacle GT, latched (Phase 4)
  sim_api                 FastAPI :8090 (Phase 3, opt-in)
  foxglove_bridge         WebSocket :8765 for remote LLM/UI access

Note: collision events on /agv/sim/events and physical teleport via
/agv/sim/reset_request are produced by the in-sim handler that lives in
scripts/open_greenhouse.py — that script is loaded into the Isaac Sim
process by run_isaac_sim.sh, not by this launch file.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('agv_sim_validation')
    params = os.path.join(pkg, 'config', 'validation_params.yaml')

    enable_api = LaunchConfiguration('enable_api')
    enable_foxglove = LaunchConfiguration('enable_foxglove')

    return LaunchDescription([
        DeclareLaunchArgument('enable_api', default_value='false',
                              description='Launch the FastAPI :8090 service'),
        DeclareLaunchArgument('enable_foxglove', default_value='true',
                              description='Launch foxglove_bridge :8765'),

        # Ground-truth pose from /tf (world -> base_link)
        Node(package='agv_sim_validation', executable='ground_truth_publisher',
             name='ground_truth_publisher', namespace='agv',
             parameters=[params, {'use_sim_time': True}],
             output='screen'),

        # Localization error monitor
        Node(package='agv_sim_validation', executable='localization_monitor',
             name='localization_monitor', namespace='agv',
             parameters=[params],
             output='screen'),

        # Discrete event detector
        Node(package='agv_sim_validation', executable='events_detector',
             name='events_detector', namespace='agv',
             parameters=[params],
             output='screen'),

        # Episode tracker (Nav2 missions → /agv/sim/episode_summary)
        Node(package='agv_sim_validation', executable='episode_tracker',
             name='episode_tracker', namespace='agv',
             parameters=[params],
             output='screen'),

        # Visible AprilTags GT (camera frustum test)
        Node(package='agv_sim_validation', executable='visible_markers',
             name='visible_markers', namespace='agv',
             parameters=[params, {'use_sim_time': True}],
             output='screen'),

        # Static obstacles GT (walls + crates + props, latched)
        Node(package='agv_sim_validation', executable='obstacles_publisher',
             name='obstacles_publisher', namespace='agv',
             parameters=[params],
             output='screen'),

        # FastAPI control surface :8090 for remote LLM agent
        Node(package='agv_sim_validation', executable='sim_api',
             name='sim_api',
             parameters=[params],
             condition=IfCondition(enable_api),
             output='screen'),

        # Foxglove bridge — WebSocket on :8765 for remote subscribers
        Node(package='foxglove_bridge', executable='foxglove_bridge',
             name='foxglove_bridge',
             parameters=[{'port': 8765, 'use_sim_time': True,
                          'send_buffer_limit': 100_000_000,  # 100 MB for camera images
                          'use_compression': True}],
             condition=IfCondition(enable_foxglove),
             output='screen'),
    ])
