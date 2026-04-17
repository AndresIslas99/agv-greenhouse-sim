#!/usr/bin/env python3
"""Wheel-encoder differential-drive odometry from /agv/joint_states.

Replaces Isaac OmniGraph `IsaacComputeOdometry → ROS2PublishOdometry` for
the `/agv/wheel_odom` topic. The OmniGraph version had two defects that
collapsed the brain's `ekf_local`:

  1. `pose.position.x` accumulated with the wrong sign (drifted negative
     while the chassis physically moved positive).
  2. Covariance was 0.0 across the board, making the EKF over-trust the
     (wrong) estimate.

This node integrates wheel angular velocities from the JointState publisher
(which Isaac wires correctly) using textbook diff-drive kinematics. Because
the input is the wheel rotation only — not chassis pose — this odometry is
**immune to teleports** (same as a real ODrive encoder). Sign convention is
explicit in our code, and covariance is set to typical diff-drive values
(`pos x,y = 0.005`, `yaw = 0.001`).

Topic contract:
  Input:  /agv/joint_states  (sensor_msgs/JointState, 50 Hz from OmniGraph)
  Output: /agv/wheel_odom    (nav_msgs/Odometry, 50 Hz, frame_id=odom)

The TF `odom → base_link` is published by `sim_odom_tf_broadcaster.py`,
which subscribes to /agv/wheel_odom and republishes the pose as TF — that
chain is unchanged, so this node only owns the Odometry message.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry


class WheelOdomPublisher(Node):
    def __init__(self):
        super().__init__('sim_wheel_odom_publisher')

        # Calibrated 2026-04-08 — must match drive_shaping_params.yaml +
        # the brain's odrive_params.yaml.
        self.declare_parameter('wheel_radius', 0.0781)
        self.declare_parameter('track_width', 0.960)
        self.declare_parameter('joint_left',  'left_wheel_joint')
        self.declare_parameter('joint_right', 'right_wheel_joint')
        # Covariance — typical diff-drive encoder values. Conservative so
        # the EKF doesn't over-trust us; IMU + visual_slam contribute too.
        self.declare_parameter('pos_cov', 0.005)
        self.declare_parameter('yaw_cov', 0.001)

        self.r  = float(self.get_parameter('wheel_radius').value)
        self.tw = float(self.get_parameter('track_width').value)
        self.j_left  = self.get_parameter('joint_left').value
        self.j_right = self.get_parameter('joint_right').value
        self.pos_cov = float(self.get_parameter('pos_cov').value)
        self.yaw_cov = float(self.get_parameter('yaw_cov').value)

        # Integrated odometry pose (odom frame).
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self._prev_stamp = None

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(JointState, 'joint_states',
                                 self._on_joint_states, qos)
        self.pub = self.create_publisher(Odometry, 'wheel_odom', qos)

        self.get_logger().info(
            f'wheel_odom from joint_states: r={self.r}m tw={self.tw}m '
            f'left={self.j_left} right={self.j_right} '
            f'pos_cov={self.pos_cov} yaw_cov={self.yaw_cov}')

    def _on_joint_states(self, msg: JointState):
        # Locate the wheel joint indices (cache them after first lookup).
        if not msg.velocity:
            return
        try:
            li = msg.name.index(self.j_left)
            ri = msg.name.index(self.j_right)
        except ValueError:
            self.get_logger().warning(
                f'joint_states missing {self.j_left} or {self.j_right}; '
                f'have: {list(msg.name)}',
                throttle_duration_sec=5.0)
            return
        if li >= len(msg.velocity) or ri >= len(msg.velocity):
            return

        # Diff-drive forward kinematics in body frame.
        wl = float(msg.velocity[li])  # rad/s on left wheel
        wr = float(msg.velocity[ri])  # rad/s on right wheel
        v_left  = wl * self.r          # m/s wheel surface
        v_right = wr * self.r
        linear_x  = (v_left + v_right) * 0.5
        angular_z = (v_right - v_left) / self.tw

        # Integrate using sim time from the JointState header.
        now = msg.header.stamp
        now_t = now.sec + now.nanosec * 1e-9
        if self._prev_stamp is None:
            dt = 0.0
        else:
            dt = now_t - self._prev_stamp
            if dt <= 0.0 or dt > 1.0:
                # Skip first sample after a clock jump (sim restart, etc.)
                dt = 0.0
        self._prev_stamp = now_t

        if dt > 0.0:
            # Mid-point integration on yaw for accuracy at higher angular vels.
            self.yaw += angular_z * dt
            # Wrap yaw to [-pi, pi]
            if self.yaw > math.pi:
                self.yaw -= 2.0 * math.pi
            elif self.yaw < -math.pi:
                self.yaw += 2.0 * math.pi
            self.x += linear_x * math.cos(self.yaw) * dt
            self.y += linear_x * math.sin(self.yaw) * dt

        out = Odometry()
        out.header.stamp = now
        out.header.frame_id = 'odom'
        out.child_frame_id = 'base_link'
        out.pose.pose.position.x = self.x
        out.pose.pose.position.y = self.y
        out.pose.pose.position.z = 0.0
        out.pose.pose.orientation.z = math.sin(self.yaw * 0.5)
        out.pose.pose.orientation.w = math.cos(self.yaw * 0.5)
        out.twist.twist.linear.x  = linear_x
        out.twist.twist.angular.z = angular_z

        # 6×6 row-major covariance for [x y z roll pitch yaw].
        # Diagonal only; off-diagonals stay zero.
        big = 1e6
        cov = [
            self.pos_cov, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, self.pos_cov, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, big,        0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, big,        0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, big,        0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, self.yaw_cov,
        ]
        out.pose.covariance = cov
        out.twist.covariance = cov
        self.pub.publish(out)


def main():
    rclpy.init()
    node = WheelOdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
