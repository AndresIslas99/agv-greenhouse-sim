#!/usr/bin/env python3
"""Publish ground-truth robot pose from Isaac's /tf tree.

Isaac Sim's OmniGraph PublishTransformTree emits `world → agv` every physics
step. Because Isaac reads the pose straight from PhysX (no encoder drift, no
fusion error), that transform IS the ground truth.

This node listens on tf2, composes `world → base_link` (via the URDF chain
agv/base_link which robot_state_publisher inserts), and republishes as a
plain PoseStamped + TwistStamped for validation consumers.

Publishes:
  /agv/sim/ground_truth/pose   PoseStamped   10 Hz, frame_id=world
  /agv/sim/ground_truth/twist  TwistStamped  10 Hz (numeric derivative)
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException


def yaw_from_quat(x, y, z, w):
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


class GroundTruthPublisher(Node):
    def __init__(self):
        super().__init__('ground_truth_publisher')
        self.declare_parameter('source_frame', 'world')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('rate_hz', 10.0)

        self._src = self.get_parameter('source_frame').value
        self._dst = self.get_parameter('target_frame').value

        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pose_pub = self.create_publisher(PoseStamped, '/agv/sim/ground_truth/pose', qos)
        self.twist_pub = self.create_publisher(TwistStamped, '/agv/sim/ground_truth/twist', qos)

        self._last = None  # (t_sec, x, y, yaw)
        self.create_timer(1.0 / self.get_parameter('rate_hz').value, self._tick)
        self.get_logger().info(f'GT publisher: {self._src} -> {self._dst}')

    def _tick(self):
        try:
            tf = self.tf_buf.lookup_transform(
                self._src, self._dst, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05))
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            # Throttle warning to once every ~5s so we don't spam
            self.get_logger().warning(
                f'TF lookup {self._src} -> {self._dst} failed: {type(e).__name__}: {e}',
                throttle_duration_sec=5.0)
            return

        t = tf.transform.translation
        r = tf.transform.rotation
        now = self.get_clock().now()

        pose = PoseStamped()
        pose.header.stamp = now.to_msg()
        pose.header.frame_id = self._src
        pose.pose.position.x = t.x
        pose.pose.position.y = t.y
        pose.pose.position.z = t.z
        pose.pose.orientation = r
        self.pose_pub.publish(pose)

        # Numeric differentiation for twist (world-frame velocities)
        t_s = now.nanoseconds * 1e-9
        yaw = yaw_from_quat(r.x, r.y, r.z, r.w)
        if self._last is not None:
            dt = t_s - self._last[0]
            if dt > 1e-6:
                tw = TwistStamped()
                tw.header.stamp = now.to_msg()
                tw.header.frame_id = self._src
                tw.twist.linear.x = (t.x - self._last[1]) / dt
                tw.twist.linear.y = (t.y - self._last[2]) / dt
                d_yaw = yaw - self._last[3]
                while d_yaw > math.pi:
                    d_yaw -= 2 * math.pi
                while d_yaw < -math.pi:
                    d_yaw += 2 * math.pi
                tw.twist.angular.z = d_yaw / dt
                self.twist_pub.publish(tw)
        self._last = (t_s, t.x, t.y, yaw)


def main():
    rclpy.init()
    node = GroundTruthPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
