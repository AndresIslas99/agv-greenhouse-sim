#!/usr/bin/env python3
"""Broadcast odom → base_link TF from /agv/wheel_odom.

Isaac Sim's ROS2PublishOdometry publishes the topic but does not broadcast
the matching TF transform. On the real robot this TF is owned by the brain's
ekf_local. When the sim runs without a brain (standalone validation), nothing
fills in that edge of the TF tree — which breaks pointcloud_to_laserscan and
any consumer of map/odom/base_link chains.

This tiny broadcaster fills the gap. It should be DISABLED when the brain is
actually running (the brain's ekf_local owns odom → base_link).
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomTfBroadcaster(Node):
    def __init__(self):
        super().__init__('sim_odom_tf_broadcaster')
        self.br = TransformBroadcaster(self)
        self.create_subscription(Odometry, 'wheel_odom', self._on_odom, 10)
        self.get_logger().info('Broadcasting odom -> base_link from /agv/wheel_odom')

    def _on_odom(self, msg: Odometry):
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = OdomTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
