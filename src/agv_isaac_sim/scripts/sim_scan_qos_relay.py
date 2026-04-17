#!/usr/bin/env python3
"""QoS relay BEST_EFFORT -> RELIABLE for /agv/scan.

`pointcloud_to_laserscan_node` (the standard ROS package that converts the
ZED point cloud into a planar LaserScan) hardcodes its publisher QoS to
SensorDataQoS = BEST_EFFORT + VOLATILE + KEEP_LAST(5). The brain's Nav2
stack (collision_monitor, local/global_costmap, controller_server)
subscribes with RELIABLE QoS by default, which is incompatible with a
BEST_EFFORT publisher — DDS refuses the connection and the brain sees
the topic as silent even though messages are being produced.

This relay subscribes /agv/_sim_internal/scan_be (the renamed
pointcloud_to_laserscan output) with BEST_EFFORT and republishes on
/agv/scan with RELIABLE so the brain's Nav2 subscribers connect normally.

The launch passes the renamed input via parameter to keep both topics
configurable from outside.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


class ScanQosRelay(Node):
    def __init__(self):
        super().__init__('sim_scan_qos_relay')
        self.declare_parameter('input_topic',  '/agv/_sim_internal/scan_be')
        self.declare_parameter('output_topic', '/agv/scan')
        in_topic  = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value

        sub_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST)
        pub_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST)

        self.pub = self.create_publisher(LaserScan, out_topic, pub_qos)
        self.create_subscription(LaserScan, in_topic, self._on_scan, sub_qos)
        self.get_logger().info(
            f'QoS relay {in_topic} (BEST_EFFORT) -> {out_topic} (RELIABLE)')

    def _on_scan(self, msg: LaserScan):
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ScanQosRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
