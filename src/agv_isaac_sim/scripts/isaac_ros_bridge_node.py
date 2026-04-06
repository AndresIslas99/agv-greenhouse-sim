#!/usr/bin/env python3
"""Isaac Sim ROS 2 bridge helper node.

Handles topics that Isaac Sim OmniGraph doesn't publish natively:
- /clock (sim time → ROS clock) — only needed if Isaac Sim's built-in
  clock publisher is not enabled
- Duplicate IMU on /agv/imu/data (relay from /zed/zed_node/imu/data)

This node is a lightweight relay — most topics are published directly
by Isaac Sim's OmniGraph nodes configured in import_robot_usd.py.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu


class IsaacBridgeNode(Node):
    def __init__(self):
        super().__init__('isaac_ros_bridge')

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Relay IMU to alternate topic (matching Gazebo dual-path bridge)
        self.imu_pub = self.create_publisher(Imu, '/agv/imu/data', qos)
        self.imu_sub = self.create_subscription(
            Imu, '/zed/zed_node/imu/data', self._relay_imu, qos)

        self.get_logger().info('Isaac ROS bridge started (IMU relay: /zed/zed_node/imu/data → /agv/imu/data)')

    def _relay_imu(self, msg: Imu):
        self.imu_pub.publish(msg)


def main():
    rclpy.init()
    node = IsaacBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
