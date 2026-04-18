#!/usr/bin/env python3
"""Pointcloud downsampler — keeps the WiFi link to the brain alive.

The Isaac OmniGraph CameraHelper publishes the dense ZED 2i pointcloud
at HD720 resolution: 1280×720 = 921 600 points × 12 B (xyz float32) =
~11 MB per message. At the 10 Hz rate the brain expects, that is
~880 Mbps sustained. WiFi to the Jetson tops out around 300 Mbps real
throughput, so the link saturates and CycloneDDS drops samples (round 33
saw 0 msgs/s on the brain side).

In the real robot this is a non-issue: the ZED SDK runs on the Jetson
and produces the cloud locally — it never traverses the network. HIL
has to ship the cloud over WiFi because the ZED SDK can't run on the
brain (no hardware).

Fix: subsample the cloud sim-side BEFORE it crosses the WiFi link.
We take every Nth row × every Nth column of the organized cloud,
preserving the structured (height × width) layout so consumers like
pointcloud_to_laserscan still treat it as a depth-camera cloud.

Stride 4 → 16× fewer points → 230 400 pts × 12 B ≈ 2.7 MB/msg →
~220 Mbps sustained at 10 Hz. Comfortably within WiFi headroom.

Topology:
  OmniGraph publishes /agv/zed/point_cloud/cloud_raw   (HD720, ~11 MB)
  this node subscribes raw, downsamples, publishes
                       /agv/zed/point_cloud/cloud_registered  (subsampled)

Parameters:
  stride: keep every Nth point in u and v (default 4)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2


class PointcloudDownsampleNode(Node):
    def __init__(self):
        super().__init__('sim_pointcloud_downsample')

        self.declare_parameter('stride', 4)
        self.stride = int(self.get_parameter('stride').value)
        if self.stride < 1:
            self.stride = 1

        # Match the Isaac CameraHelper publisher QoS: RELIABLE.
        # Downstream subs (collision_monitor, brain pcl_to_scan) negotiate
        # to BEST_EFFORT if they want; RELIABLE pub is compatible with both.
        qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.pub = self.create_publisher(
            PointCloud2, 'cloud_registered', qos)
        self.sub = self.create_subscription(
            PointCloud2, 'cloud_raw', self._on_cloud, qos)

        self.get_logger().info(
            f'Pointcloud downsample relay up — stride={self.stride} '
            f'(keep every {self.stride}th row × col) → '
            f'{1.0/(self.stride*self.stride)*100:.1f}% of points retained')

    def _on_cloud(self, msg: PointCloud2):
        # The OmniGraph depth_pcl helper publishes an organized cloud
        # (height = render_height, width = render_width), so we can
        # subsample by 2D stride without losing the structured layout.
        # If a future producer publishes an unorganized cloud (height=1)
        # we fall back to a flat 1D stride.

        s = self.stride
        if s == 1:
            self.pub.publish(msg)
            return

        h, w = msg.height, msg.width
        point_step = msg.point_step
        if h <= 1 or w <= 1:
            # Unorganized — stride along the single axis.
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            buf = buf.reshape(-1, point_step)
            sub = buf[::s, :]
            new_h, new_w = 1, sub.shape[0]
            new_data = sub.tobytes()
        else:
            # Organized — stride 2D so we keep the (h', w') grid.
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            buf = buf.reshape(h, w, point_step)
            sub = buf[::s, ::s, :]
            new_h, new_w = sub.shape[0], sub.shape[1]
            new_data = np.ascontiguousarray(sub).tobytes()

        out = PointCloud2()
        out.header = msg.header
        out.height = int(new_h)
        out.width = int(new_w)
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = int(msg.point_step * new_w)
        out.data = new_data
        out.is_dense = msg.is_dense
        self.pub.publish(out)


def main():
    rclpy.init()
    node = PointcloudDownsampleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
