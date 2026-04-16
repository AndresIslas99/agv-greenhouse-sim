#!/usr/bin/env python3
"""Compare ground-truth pose vs brain's estimated pose, publish error JSON.

Subscribes:
  /agv/sim/ground_truth/pose   geometry_msgs/PoseStamped  (oracle, from Isaac)
  /agv/odometry/global         nav_msgs/Odometry          (brain's ekf_global)

Publishes:
  /agv/sim/localization_error  std_msgs/String (JSON), 1 Hz

The error is the brain's localization quality — how far off its EKF is from
the true pose. An LLM agent reads this to spot drift, sudden jumps after
loop closures, or systematic bias.
"""
import json
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def angle_diff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


class LocalizationMonitor(Node):
    def __init__(self):
        super().__init__('localization_monitor')

        self.declare_parameter('rmse_window_s', 5.0)
        self.declare_parameter('publish_rate_hz', 1.0)
        self._window_s = self.get_parameter('rmse_window_s').value
        rate = self.get_parameter('publish_rate_hz').value

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.gt = None
        self.est = None
        self.history = deque()  # (t_sim, pos_err, yaw_err)

        self.create_subscription(PoseStamped, '/agv/sim/ground_truth/pose',
                                 self._on_gt, qos)
        self.create_subscription(Odometry, '/agv/odometry/global',
                                 self._on_est, qos)
        self.pub = self.create_publisher(String, '/agv/sim/localization_error', 10)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info('Localization monitor up')

    def _on_gt(self, msg: PoseStamped):
        self.gt = msg

    def _on_est(self, msg: Odometry):
        self.est = msg

    def _now_s(self):
        # Use sim time when available
        return self.get_clock().now().nanoseconds * 1e-9

    def _tick(self):
        if self.gt is None or self.est is None:
            return

        gx = self.gt.pose.position.x
        gy = self.gt.pose.position.y
        gyaw = yaw_from_quat(self.gt.pose.orientation)

        ex = self.est.pose.pose.position.x
        ey = self.est.pose.pose.position.y
        eyaw = yaw_from_quat(self.est.pose.pose.orientation)

        pos_err = math.hypot(ex - gx, ey - gy)
        yaw_err = abs(angle_diff(eyaw, gyaw))

        t = self._now_s()
        self.history.append((t, pos_err, yaw_err))
        # Trim window
        while self.history and (t - self.history[0][0]) > self._window_s:
            self.history.popleft()

        rmse_pos = math.sqrt(sum(p * p for _, p, _ in self.history) / len(self.history))
        rmse_yaw = math.sqrt(sum(y * y for _, _, y in self.history) / len(self.history))

        out = {
            't_sim': t,
            'pos_err_m': round(pos_err, 4),
            'yaw_err_rad': round(yaw_err, 4),
            'rmse_pos_m': round(rmse_pos, 4),
            'rmse_yaw_rad': round(rmse_yaw, 4),
            'window_s': self._window_s,
            'samples': len(self.history),
            'gt_pose': [round(gx, 3), round(gy, 3), round(gyaw, 3)],
            'est_pose': [round(ex, 3), round(ey, 3), round(eyaw, 3)],
        }
        self.pub.publish(String(data=json.dumps(out)))


def main():
    rclpy.init()
    node = LocalizationMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
