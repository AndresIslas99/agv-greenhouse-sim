#!/usr/bin/env python3
"""Simulated global odometry for HIL — replaces cuVSLAM.

Subscribes to wheel_odom (namespace-relative) and republishes as
/visual_slam/tracking/odometry with frame_id='map'.

In simulation there is no drift, so odom == map and the wheel
odometry IS the ground-truth pose in the map frame. This gives
the Jetson-side global EKF the same topic contract as the real
cuVSLAM visual odometry.

Topic contract:
  Input:  /agv/wheel_odom (nav_msgs/Odometry, frame_id=odom)
  Output: /visual_slam/tracking/odometry (nav_msgs/Odometry, frame_id=map)
  Output: /slam/quality (std_msgs/String, JSON, 1Hz)
"""

import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class SimGlobalOdom(Node):
    def __init__(self):
        super().__init__('sim_global_odom')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(
            Odometry, '/visual_slam/tracking/odometry', qos)
        self.sub = self.create_subscription(
            Odometry, 'wheel_odom', self._on_odom, qos)

        # Simulated SLAM quality — dashboard expects this for tracking status
        self.pub_quality = self.create_publisher(String, '/slam/quality', 10)
        self.create_timer(1.0, self._publish_quality)

    def _on_odom(self, msg: Odometry):
        msg.header.frame_id = 'map'
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
