#!/usr/bin/env python3
"""
Fake AprilTag Marker Detector (sim-only development tool).

Uses TF to check robot proximity to known AprilTag positions from
markers_registry.yaml. When within detection range, publishes a
geometry_msgs/PoseWithCovarianceStamped on /{ns}/marker_pose.

This allows the agv_markers package to be developed and tested
without a real camera or apriltag_ros pipeline.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener, TransformException


class FakeMarkerDetector(Node):
    def __init__(self):
        super().__init__('fake_marker_detector')

        # Parameters
        self.declare_parameter('markers_file', '')
        self.declare_parameter('detection_range', 2.0)
        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        self._detection_range = self.get_parameter('detection_range').value
        self._map_frame = self.get_parameter('map_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        rate = self.get_parameter('publish_rate').value

        # Load markers from parameter or file
        self._markers = []
        self._load_markers()

        # TF
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Publisher
        self._pub = self.create_publisher(
            PoseWithCovarianceStamped,
            'marker_pose',
            QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE),
        )

        # Timer
        self.create_timer(1.0 / rate, self._timer_callback)

        self.get_logger().info(
            f'Fake marker detector started: {len(self._markers)} markers, '
            f'range={self._detection_range}m'
        )

    def _load_markers(self):
        """Load marker positions from YAML via parameters."""
        # Markers are loaded as ROS parameters from the YAML file
        # We try to read them from the 'markers.tags' parameter namespace
        try:
            markers_file = self.get_parameter('markers_file').value
            if markers_file:
                self._load_markers_from_file(markers_file)
                return
        except Exception:
            pass

        # Fallback: declare markers inline (matching greenhouse_simple.world)
        self._markers = [
            {'id': 0, 'x': 1.0, 'y': 0.0, 'z': 1.0,
             'qx': 0.0, 'qy': 0.0, 'qz': 0.7071068, 'qw': 0.7071068},
            {'id': 1, 'x': 29.0, 'y': 0.0, 'z': 1.0,
             'qx': 0.0, 'qy': 0.0, 'qz': -0.7071068, 'qw': 0.7071068},
            {'id': 2, 'x': 6.0, 'y': -4.4, 'z': 1.0,
             'qx': 0.0, 'qy': 0.0, 'qz': 0.7071068, 'qw': 0.7071068},
            {'id': 3, 'x': 6.0, 'y': 0.0, 'z': 1.0,
             'qx': 0.0, 'qy': 0.0, 'qz': 0.7071068, 'qw': 0.7071068},
            {'id': 4, 'x': 6.0, 'y': 4.4, 'z': 1.0,
             'qx': 0.0, 'qy': 0.0, 'qz': 0.7071068, 'qw': 0.7071068},
            {'id': 5, 'x': 2.5, 'y': -5.0, 'z': 1.0,
             'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0},
        ]
        self.get_logger().info('Using built-in marker positions (greenhouse_simple)')

    def _load_markers_from_file(self, filepath):
        """Parse markers_registry.yaml."""
        import yaml
        with open(filepath, 'r') as f:
            data = yaml.safe_load(f)

        markers_data = data.get('markers', {})
        self._detection_range = markers_data.get('detection_range', self._detection_range)

        for tag in markers_data.get('tags', []):
            pos = tag['pose']['position']
            ori = tag['pose']['orientation']
            self._markers.append({
                'id': tag['id'],
                'x': pos['x'], 'y': pos['y'], 'z': pos['z'],
                'qx': ori['x'], 'qy': ori['y'], 'qz': ori['z'], 'qw': ori['w'],
            })

        self.get_logger().info(f'Loaded {len(self._markers)} markers from {filepath}')

    def _timer_callback(self):
        """Check proximity to each marker and publish if in range."""
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, rclpy.time.Time()
            )
        except TransformException:
            return

        robot_x = tf.transform.translation.x
        robot_y = tf.transform.translation.y

        for marker in self._markers:
            dx = marker['x'] - robot_x
            dy = marker['y'] - robot_y
            distance = math.sqrt(dx * dx + dy * dy)

            if distance <= self._detection_range:
                msg = PoseWithCovarianceStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self._map_frame

                msg.pose.pose.position.x = marker['x']
                msg.pose.pose.position.y = marker['y']
                msg.pose.pose.position.z = marker['z']
                msg.pose.pose.orientation.x = marker['qx']
                msg.pose.pose.orientation.y = marker['qy']
                msg.pose.pose.orientation.z = marker['qz']
                msg.pose.pose.orientation.w = marker['qw']

                # Covariance: higher uncertainty further from marker
                cov_scale = 0.01 + 0.04 * (distance / self._detection_range)
                msg.pose.covariance[0] = cov_scale   # x
                msg.pose.covariance[7] = cov_scale   # y
                msg.pose.covariance[35] = cov_scale   # yaw

                self._pub.publish(msg)
                self.get_logger().debug(
                    f'Detected tag {marker["id"]} at distance {distance:.2f}m'
                )


def main(args=None):
    rclpy.init(args=args)
    node = FakeMarkerDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
