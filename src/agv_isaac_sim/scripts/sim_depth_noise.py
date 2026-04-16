#!/usr/bin/env python3
"""Distance-dependent depth noise injector for ZED 2i simulation.

Subscribes to the raw depth image from Isaac Sim and republishes with
calibrated Gaussian noise that scales with distance — matching the real
ZED 2i published accuracy curve:

    σ(d) = NOISE_BASE + NOISE_SCALE × d

    At 1m:  σ ≈ 7mm    (0.002 + 0.005×1)
    At 5m:  σ ≈ 27mm   (0.002 + 0.005×5)
    At 10m: σ ≈ 52mm   (0.002 + 0.005×10)

The raw depth topic is remapped to an internal topic, and this node
publishes the noisy version on the original topic name.

Usage in launch:
    Remap OmniGraph depth output to /agv/zed/depth/depth_clean, then
    this node subscribes to it and publishes /agv/zed/depth/depth_registered.

Parameters:
    noise_base:  minimum noise floor in meters (default 0.002)
    noise_scale: noise per meter of distance (default 0.005)
    dropout_distance: depth values beyond this distance randomly drop out (default 15.0)
    dropout_rate: probability of dropout beyond dropout_distance (default 0.3)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class DepthNoiseNode(Node):
    def __init__(self):
        super().__init__('sim_depth_noise')

        self.declare_parameter('noise_base', 0.002)
        self.declare_parameter('noise_scale', 0.005)
        self.declare_parameter('dropout_distance', 15.0)
        self.declare_parameter('dropout_rate', 0.3)
        self.declare_parameter('fog_factor', 0.0)

        self.noise_base = self.get_parameter('noise_base').value
        self.noise_scale = self.get_parameter('noise_scale').value
        self.dropout_dist = self.get_parameter('dropout_distance').value
        self.dropout_rate = self.get_parameter('dropout_rate').value
        self.fog_factor = self.get_parameter('fog_factor').value

        # Apply fog: increases noise and dropout to simulate humidity
        # fog_factor=0 → clear, 0.5 → light haze, 2.0 → heavy misting
        self.effective_noise_scale = self.noise_scale * (1.0 + self.fog_factor)
        self.effective_dropout_rate = min(
            0.95, self.dropout_rate + self.fog_factor * 0.15)

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.pub = self.create_publisher(
            Image, 'depth_registered', qos)
        self.sub = self.create_subscription(
            Image, 'depth_raw', self._on_depth, qos)

        self.get_logger().info(
            f'Depth noise: base={self.noise_base}m, scale={self.noise_scale}/m, '
            f'dropout>{self.dropout_dist}m @{self.dropout_rate*100:.0f}%, '
            f'fog={self.fog_factor} (effective scale={self.effective_noise_scale:.4f}, '
            f'dropout={self.effective_dropout_rate:.2f})')

    def _on_depth(self, msg: Image):
        # Decode depth image (32FC1 = float32, meters)
        if msg.encoding not in ('32FC1', 'TYPE_32FC1'):
            self.pub.publish(msg)
            return

        h, w = msg.height, msg.width
        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(h, w).copy()

        # Distance-dependent Gaussian noise: σ(d) = base + effective_scale × d
        valid = np.isfinite(depth) & (depth > 0)
        sigma = np.where(valid, self.noise_base + self.effective_noise_scale * depth, 0.0)
        noise = np.random.normal(0.0, 1.0, depth.shape).astype(np.float32) * sigma
        depth[valid] += noise[valid]

        # Dropout simulation for far distances (ZED loses tracking beyond ~15m)
        if self.effective_dropout_rate > 0:
            far_mask = valid & (depth > self.dropout_dist)
            dropout_mask = far_mask & (
                np.random.random(depth.shape) < self.effective_dropout_rate)
            depth[dropout_mask] = np.nan

        # Clamp negative depths
        depth[depth < 0] = 0.0

        # Republish
        msg.data = depth.tobytes()
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = DepthNoiseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
