#!/usr/bin/env python3
"""ZED 2i lens distortion simulation for Isaac Sim.

Isaac Sim cameras render perfect pinhole images. Real ZED 2i has
barrel distortion from the wide-angle 110° FOV lens. This node:

1. Subscribes to clean rendered images from Isaac Sim
2. Applies Brown-Conrady radial+tangential distortion via cv2.remap()
3. Publishes distorted image as 'image_raw' (what real camera sees)
4. Passes through clean image as 'image_rect_color' (rectified)
5. Publishes camera_info with distortion coefficients

The distortion map is precomputed once at startup for efficiency.

Parameters:
    calibration_file: path to zed2i_calibration.yaml
    (or individual k1, k2, p1, p2, k3, fx, fy, cx, cy params)
"""

import numpy as np
import cv2
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo


class LensDistortionNode(Node):
    def __init__(self):
        super().__init__('sim_lens_distortion')

        # Distortion coefficients (ZED 2i typical factory calibration)
        self.declare_parameter('k1', -0.0425)
        self.declare_parameter('k2', 0.0105)
        self.declare_parameter('p1', 0.0)
        self.declare_parameter('p2', 0.0)
        self.declare_parameter('k3', 0.0)
        self.declare_parameter('fx', 527.5)
        self.declare_parameter('fy', 527.5)
        self.declare_parameter('cx', 640.0)
        self.declare_parameter('cy', 360.0)
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)

        k1 = self.get_parameter('k1').value
        k2 = self.get_parameter('k2').value
        p1 = self.get_parameter('p1').value
        p2 = self.get_parameter('p2').value
        k3 = self.get_parameter('k3').value
        fx = self.get_parameter('fx').value
        fy = self.get_parameter('fy').value
        cx = self.get_parameter('cx').value
        cy = self.get_parameter('cy').value
        w = self.get_parameter('width').value
        h = self.get_parameter('height').value

        # Build camera matrix and distortion vector
        self.K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
        self.D = np.array([k1, k2, p1, p2, k3], dtype=np.float64)

        # Precompute distortion remap for efficiency (done once)
        # We need the INVERSE map: for each pixel in distorted image,
        # where does it come from in the undistorted (clean) image?
        self.map_x, self.map_y = self._build_distortion_map(w, h)

        # Camera info template
        self.camera_info = CameraInfo()
        self.camera_info.width = w
        self.camera_info.height = h
        self.camera_info.distortion_model = 'plumb_bob'
        self.camera_info.d = [k1, k2, p1, p2, k3]
        self.camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        self.camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Publishers
        self.pub_raw = self.create_publisher(Image, 'image_raw', qos)
        self.pub_rect = self.create_publisher(Image, 'image_rect_color', qos)
        self.pub_info = self.create_publisher(CameraInfo, 'camera_info', qos)

        # Subscriber (clean image from Isaac Sim)
        self.sub = self.create_subscription(Image, 'image_clean', self._on_image, qos)

        self.get_logger().info(
            f'Lens distortion: k1={k1:.4f}, k2={k2:.4f}, '
            f'fx={fx:.1f}, fy={fy:.1f}')

    def _build_distortion_map(self, w, h):
        """Precompute pixel remap for applying distortion to clean images.

        For each output (distorted) pixel, compute where it maps FROM
        in the input (clean) image using the inverse distortion model.
        """
        # Generate grid of normalized coordinates
        coords_x = np.arange(w, dtype=np.float32)
        coords_y = np.arange(h, dtype=np.float32)
        grid_x, grid_y = np.meshgrid(coords_x, coords_y)

        # Normalize to camera coordinates
        x_norm = (grid_x - self.K[0, 2]) / self.K[0, 0]
        y_norm = (grid_y - self.K[1, 2]) / self.K[1, 1]

        r2 = x_norm**2 + y_norm**2
        r4 = r2**2
        r6 = r2**3

        k1, k2, p1, p2, k3 = self.D

        # Radial distortion
        radial = 1 + k1 * r2 + k2 * r4 + k3 * r6

        # Tangential distortion
        x_dist = x_norm * radial + 2 * p1 * x_norm * y_norm + p2 * (r2 + 2 * x_norm**2)
        y_dist = y_norm * radial + p1 * (r2 + 2 * y_norm**2) + 2 * p2 * x_norm * y_norm

        # Back to pixel coordinates
        map_x = (x_dist * self.K[0, 0] + self.K[0, 2]).astype(np.float32)
        map_y = (y_dist * self.K[1, 1] + self.K[1, 2]).astype(np.float32)

        self.get_logger().info(
            f'Distortion map built: {w}x{h}, '
            f'max shift={np.max(np.abs(map_x - grid_x)):.1f}px')

        return map_x, map_y

    def _on_image(self, msg: Image):
        # Decode image
        if msg.encoding in ('rgb8', 'bgr8'):
            channels = 3
            dtype = np.uint8
        elif msg.encoding in ('rgba8', 'bgra8'):
            channels = 4
            dtype = np.uint8
        else:
            # Unsupported encoding — pass through
            self.pub_rect.publish(msg)
            self.pub_raw.publish(msg)
            return

        img = np.frombuffer(msg.data, dtype=dtype).reshape(
            msg.height, msg.width, channels)

        # Apply distortion via remap
        distorted = cv2.remap(img, self.map_x, self.map_y,
                              interpolation=cv2.INTER_LINEAR,
                              borderMode=cv2.BORDER_CONSTANT,
                              borderValue=0)

        # Publish distorted (raw) image
        raw_msg = Image()
        raw_msg.header = msg.header
        raw_msg.height = msg.height
        raw_msg.width = msg.width
        raw_msg.encoding = msg.encoding
        raw_msg.step = msg.step
        raw_msg.data = distorted.tobytes()
        self.pub_raw.publish(raw_msg)

        # Publish clean (rectified) image — passthrough
        self.pub_rect.publish(msg)

        # Publish camera info with distortion coefficients
        self.camera_info.header = msg.header
        self.pub_info.publish(self.camera_info)


def main():
    rclpy.init()
    node = LensDistortionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
