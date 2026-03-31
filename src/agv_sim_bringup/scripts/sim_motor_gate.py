#!/usr/bin/env python3
"""Simulated motor arming gate — emulates ODrive arm/disarm for the real GUI.

Replicates the motor_enable / motor_state contract from agv_odrive so the
Jetson-side teleop_server (UI backend) can arm/disarm the simulated robot
exactly as it would the real one.

Contract:
  Subscribe: motor_enable (std_msgs/Bool)  — true=arm, false=disarm
  Subscribe: cmd_vel     (geometry_msgs/Twist) — from Nav2 or teleop
  Publish:   motor_state (std_msgs/String) — JSON, 2Hz
  Publish:   cmd_vel_armed (geometry_msgs/Twist) — forwarded only when armed

When disarmed, cmd_vel is silently dropped (same as real ODrive behavior).
Drive shaping node must be remapped to subscribe to cmd_vel_armed instead of cmd_vel.
"""

import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist

# ODrive axis states (matching odrive_protocol.hpp)
IDLE = 1
CLOSED_LOOP_CONTROL = 8


class SimMotorGate(Node):
    def __init__(self):
        super().__init__('sim_motor_gate')
        self.armed = False

        # Motor state publisher (2Hz, matches real ODrive node)
        self.pub_state = self.create_publisher(String, 'motor_state', 10)
        self.create_timer(0.5, self._publish_state)

        # Motor enable subscriber
        self.create_subscription(Bool, 'motor_enable', self._on_enable, 10)

        # cmd_vel gate: input → output only when armed
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel_armed', qos)
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, qos)

        self.get_logger().info('Motor gate started (disarmed)')

    def _on_enable(self, msg: Bool):
        if msg.data and not self.armed:
            self.armed = True
            self.get_logger().info('Motors ARMED (CLOSED_LOOP_CONTROL)')
        elif not msg.data and self.armed:
            self.armed = False
            # Send zero velocity before disarming
            self.pub_cmd.publish(Twist())
            self.get_logger().info('Motors DISARMED (IDLE)')

    def _on_cmd_vel(self, msg: Twist):
        if self.armed:
            self.pub_cmd.publish(msg)

    def _publish_state(self):
        state = CLOSED_LOOP_CONTROL if self.armed else IDLE
        data = json.dumps({
            'left_state': state,
            'right_state': state,
            'left_errors': 0,
            'right_errors': 0,
            'armed': self.armed,
        })
        self.pub_state.publish(String(data=data))


def main():
    rclpy.init()
    node = SimMotorGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
