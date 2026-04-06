#!/usr/bin/env python3
"""Isaac Sim ROS 2 bridge helper node.

Functions:
1. Relay IMU to alternate topic (/zed/.../imu/data → /agv/imu/data)
2. Inject realistic IMU bias drift (Wiener process random walk)

Real BMI055 IMU (in ZED 2i) exhibits slowly-drifting bias on top of
white noise. Without this, the EKF performs unrealistically well in sim.

Bias model (Wiener process):
    bias(t+dt) = bias(t) + N(0, σ_walk × √dt)

Parameters:
    gyro_bias_walk:   gyro bias random walk σ (rad/s/√s), default 0.0001
    accel_bias_walk:  accel bias random walk σ (m/s²/√s), default 0.0005
    gyro_turnon_bias: max random offset at startup (rad/s), default 0.001
    accel_turnon_bias: max random offset at startup (m/s²), default 0.01
"""

import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu


class IsaacBridgeNode(Node):
    def __init__(self):
        super().__init__('isaac_ros_bridge')

        # IMU bias drift parameters
        self.declare_parameter('gyro_bias_walk', 0.0001)
        self.declare_parameter('accel_bias_walk', 0.0005)
        self.declare_parameter('gyro_turnon_bias', 0.001)
        self.declare_parameter('accel_turnon_bias', 0.01)

        self.gyro_walk = self.get_parameter('gyro_bias_walk').value
        self.accel_walk = self.get_parameter('accel_bias_walk').value

        # Turn-on bias: random initial offset (simulates power-on calibration error)
        turnon_g = self.get_parameter('gyro_turnon_bias').value
        turnon_a = self.get_parameter('accel_turnon_bias').value
        self.gyro_bias = [random.uniform(-turnon_g, turnon_g) for _ in range(3)]
        self.accel_bias = [random.uniform(-turnon_a, turnon_a) for _ in range(3)]

        self.last_time = None

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Relay IMU to alternate topic (matching Gazebo dual-path bridge)
        self.imu_pub = self.create_publisher(Imu, '/agv/imu/data', qos)
        self.imu_sub = self.create_subscription(
            Imu, '/zed/zed_node/imu/data', self._relay_imu, qos)

        self.get_logger().info(
            f'IMU bridge: bias walk gyro={self.gyro_walk:.5f}, accel={self.accel_walk:.5f}, '
            f'turn-on gyro={[f"{b:.5f}" for b in self.gyro_bias]}, '
            f'accel={[f"{b:.5f}" for b in self.accel_bias]}')

    def _update_bias(self, dt):
        """Update bias via Wiener process random walk."""
        sqrt_dt = math.sqrt(dt) if dt > 0 else 0.0

        for i in range(3):
            self.gyro_bias[i] += random.gauss(0, self.gyro_walk * sqrt_dt)
            self.accel_bias[i] += random.gauss(0, self.accel_walk * sqrt_dt)

    def _relay_imu(self, msg: Imu):
        # Compute dt for bias walk
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_time is not None:
            dt = now - self.last_time
            if 0 < dt < 1.0:  # sanity check
                self._update_bias(dt)
        self.last_time = now

        # Inject bias into measurements
        msg.angular_velocity.x += self.gyro_bias[0]
        msg.angular_velocity.y += self.gyro_bias[1]
        msg.angular_velocity.z += self.gyro_bias[2]
        msg.linear_acceleration.x += self.accel_bias[0]
        msg.linear_acceleration.y += self.accel_bias[1]
        msg.linear_acceleration.z += self.accel_bias[2]

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
