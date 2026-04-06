#!/usr/bin/env python3
"""Simulated global odometry for HIL — replaces cuVSLAM.

Subscribes to wheel_odom (namespace-relative) and republishes as
/visual_slam/tracking/odometry with frame_id='map'.

In simulation there is no drift, so odom == map and the wheel
odometry IS the ground-truth pose in the map frame. This gives
the Jetson-side global EKF the same topic contract as the real
cuVSLAM visual odometry.

Configurable Gaussian noise can be added to simulate real-world
cuVSLAM drift and tracking imperfections.

Topic contract:
  Input:  /agv/wheel_odom (nav_msgs/Odometry, frame_id=odom)
  Output: /visual_slam/tracking/odometry (nav_msgs/Odometry, frame_id=map)
  Output: /slam/quality (std_msgs/String, JSON, 1Hz)

Parameters:
  noise_xy:       std-dev of XY position noise in meters (default 0.0)
  noise_yaw:      std-dev of yaw orientation noise in radians (default 0.0)
  dropout_rate:   probability [0,1) of dropping a message (default 0.0)
"""

import json
import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class SimGlobalOdom(Node):
    def __init__(self):
        super().__init__('sim_global_odom')

        # Noise parameters (declare with defaults)
        self.declare_parameter('noise_xy', 0.0)
        self.declare_parameter('noise_yaw', 0.0)
        self.declare_parameter('dropout_rate', 0.0)

        self.noise_xy = self.get_parameter('noise_xy').value
        self.noise_yaw = self.get_parameter('noise_yaw').value
        self.dropout_rate = self.get_parameter('dropout_rate').value

        self.get_logger().info(
            f'Noise: xy={self.noise_xy:.4f}m, yaw={self.noise_yaw:.4f}rad, '
            f'dropout={self.dropout_rate:.2f}')

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(
            Odometry, '/visual_slam/tracking/odometry', qos)
        self.sub = self.create_subscription(
            Odometry, 'wheel_odom', self._on_odom, qos)

        # Simulated SLAM quality — dashboard expects this for tracking status
        self.pub_quality = self.create_publisher(String, '/slam/quality', 10)
        self.create_timer(1.0, self._publish_quality)

    def _on_odom(self, msg: Odometry):
        # Simulate message dropout
        if self.dropout_rate > 0.0 and random.random() < self.dropout_rate:
            return

        msg.header.frame_id = 'map'

        # Add Gaussian noise to position
        if self.noise_xy > 0.0:
            msg.pose.pose.position.x += random.gauss(0.0, self.noise_xy)
            msg.pose.pose.position.y += random.gauss(0.0, self.noise_xy)

        # Add Gaussian noise to yaw (rotate quaternion around Z)
        if self.noise_yaw > 0.0:
            q = msg.pose.pose.orientation
            # Extract current yaw from quaternion
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            # Add noise and reconstruct quaternion (assuming flat ground)
            yaw += random.gauss(0.0, self.noise_yaw)
            msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
            msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
            msg.pose.pose.orientation.x = 0.0
            msg.pose.pose.orientation.y = 0.0

        self.pub.publish(msg)

    def _publish_quality(self):
        data = json.dumps({'tracking': {'confidence': 'good'}})
        self.pub_quality.publish(String(data=data))


def main():
    rclpy.init()
    node = SimGlobalOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
