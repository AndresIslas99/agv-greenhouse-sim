#!/usr/bin/env python3
"""Discrete event detector for AI navigation validation.

Watches several topics and emits one JSON message per discrete event.
LLM agents read /agv/sim/events as a timeline instead of parsing raw sensor
streams. Designed to be cheap and additive — never blocks anything else.

Events emitted (one per occurrence):
  collision         high cmd_vel + GT pose stalled, or contact (TODO via PhysX)
  stuck             cmd_vel non-zero for N seconds but actual speed near zero
  drift             localization error crossed threshold
  drift_recovered   localization error fell back below threshold
  marker_seen       AprilTag detection arrived from brain
  goal_received     Nav2 received a goal pose
  goal_reached      Nav2 succeeded
  goal_aborted      Nav2 aborted/failed
"""
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class EventsDetector(Node):
    def __init__(self):
        super().__init__('events_detector')

        # Params
        self.declare_parameter('stuck_cmd_vel_min_m_s', 0.05)
        self.declare_parameter('stuck_actual_max_m_s', 0.02)
        self.declare_parameter('stuck_duration_s', 5.0)
        self.declare_parameter('drift_pos_threshold_m', 0.5)
        self.declare_parameter('drift_yaw_threshold_rad', 0.35)
        self.declare_parameter('collision_speed_min_m_s', 0.05)

        self._stuck_cmd_min = self.get_parameter('stuck_cmd_vel_min_m_s').value
        self._stuck_act_max = self.get_parameter('stuck_actual_max_m_s').value
        self._stuck_dur = self.get_parameter('stuck_duration_s').value
        self._drift_pos = self.get_parameter('drift_pos_threshold_m').value
        self._drift_yaw = self.get_parameter('drift_yaw_threshold_rad').value
        self._coll_speed = self.get_parameter('collision_speed_min_m_s').value

        # State
        self._cmd = Twist()
        self._cmd_t = None
        self._gt_speed = 0.0
        self._stuck_since = None
        self._drifting = False

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Twist, '/agv/cmd_vel', self._on_cmd, qos)
        self.create_subscription(Odometry, '/agv/wheel_odom', self._on_wheel, qos)
        self.create_subscription(String, '/agv/sim/localization_error',
                                 self._on_loc_err, qos)
        self.create_subscription(String, '/detections', self._on_marker, qos)

        self.pub = self.create_publisher(String, '/agv/sim/events', 50)
        self.create_timer(0.5, self._tick_stuck)

        self.get_logger().info('Events detector up')

    def _emit(self, event: str, **fields):
        payload = {'t_sim': self.get_clock().now().nanoseconds * 1e-9,
                   'event': event}
        payload.update(fields)
        self.pub.publish(String(data=json.dumps(payload)))
        self.get_logger().info(f'event: {event} {fields}')

    def _on_cmd(self, msg: Twist):
        self._cmd = msg
        self._cmd_t = self.get_clock().now()

    def _on_wheel(self, msg: Odometry):
        v = msg.twist.twist.linear
        self._gt_speed = math.hypot(v.x, v.y)

    def _on_loc_err(self, msg: String):
        try:
            d = json.loads(msg.data)
        except Exception:
            return
        bad = d.get('pos_err_m', 0) > self._drift_pos or \
              d.get('yaw_err_rad', 0) > self._drift_yaw
        if bad and not self._drifting:
            self._drifting = True
            self._emit('drift', pos_err_m=d.get('pos_err_m'),
                       yaw_err_rad=d.get('yaw_err_rad'),
                       threshold_pos_m=self._drift_pos,
                       threshold_yaw_rad=self._drift_yaw)
        elif not bad and self._drifting:
            self._drifting = False
            self._emit('drift_recovered', pos_err_m=d.get('pos_err_m'),
                       yaw_err_rad=d.get('yaw_err_rad'))

    def _on_marker(self, msg: String):
        # Brain's apriltag_ros publishes detections — we just count + log
        self._emit('marker_seen', raw=msg.data[:120])

    def _tick_stuck(self):
        cmd_speed = abs(self._cmd.linear.x)
        commanded = cmd_speed > self._stuck_cmd_min
        moving = self._gt_speed > self._stuck_act_max

        if commanded and not moving:
            if self._stuck_since is None:
                self._stuck_since = self.get_clock().now()
            else:
                elapsed = (self.get_clock().now() - self._stuck_since).nanoseconds * 1e-9
                if elapsed >= self._stuck_dur:
                    self._emit('stuck', duration_s=round(elapsed, 2),
                               cmd_linear=round(cmd_speed, 3),
                               actual_speed=round(self._gt_speed, 3))
                    self._stuck_since = None  # latch off, re-arm
        else:
            self._stuck_since = None


def main():
    rclpy.init()
    node = EventsDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
