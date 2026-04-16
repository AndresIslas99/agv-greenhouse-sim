#!/usr/bin/env python3
"""Emulates the real agv_odrive node's arm/disarm + motor_state/drive_debug contract.

This node replaces the real ODrive CAN driver from the brain's point of view:

  Subscribes (relative names, namespaced by launch under /agv/):
    - motor_enable  (std_msgs/Bool)         true=arm, false=disarm
    - cmd_vel       (geometry_msgs/Twist)   input from Nav2 / teleop / rail_approach
    - e_stop        (std_msgs/Bool)         hardware/software stop

  Publishes (relative names, namespaced by launch under /agv/):
    - cmd_vel_armed (geometry_msgs/Twist)   forwarded only when armed and not e-stopped.
                                            sim_drive_shaping_node remaps its input
                                            from cmd_vel to cmd_vel_armed.
    - motor_state   (std_msgs/String JSON)  10 Hz, matches odrive_can_node.cpp:629
    - drive_debug   (std_msgs/String JSON)  10 Hz, matches odrive_can_node.cpp:647

JSON payloads reproduce exactly the field names of the real driver so the
brain's agv_ui_backend (and any dashboards) see identical data in sim and real.
"""

import json
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist

# ODrive axis states (AxisState enum from odrive_protocol.hpp)
IDLE = 1
CLOSED_LOOP_CONTROL = 8

# Mock hardware values that don't change in sim
MOCK_BUS_VOLTAGE = 48.0       # nominal 12S LiFePO4 at rest
MOCK_BUS_CURRENT = 0.0        # idle
MOCK_FET_TEMP = 25.0          # ambient
MOCK_MOTOR_TEMP = 25.0        # ambient
MOCK_THERMAL_STATE = "ok"     # ok / warning / throttle / critical


class SimMotorGate(Node):
    def __init__(self):
        super().__init__('sim_motor_gate')
        self.armed = False
        self.e_stop_active = False
        self.last_linear = 0.0
        self.last_angular = 0.0

        # Publishers (10 Hz, matching real odrive driver).
        # These timers MUST use wall clock. The real ODrive CAN node emits at
        # 10 Hz wall-time regardless of any simulator. Using sim clock here
        # would slow the output whenever the sim runs below real-time (which
        # happens under heavy GPU load). The brain's heartbeat/timeout checks
        # expect wall-clock cadence.
        self.pub_state = self.create_publisher(String, 'motor_state', 10)
        self.pub_debug = self.create_publisher(String, 'drive_debug', 10)
        wall_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.create_timer(0.1, self._publish_state, clock=wall_clock)
        self.create_timer(0.1, self._publish_debug, clock=wall_clock)

        # Subscribers
        self.create_subscription(Bool, 'motor_enable', self._on_enable, 10)
        self.create_subscription(Bool, 'e_stop', self._on_e_stop, 10)

        # cmd_vel gate: input → output only when armed AND not e-stopped
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
            self.pub_cmd.publish(Twist())  # send zero before disarming
            self.get_logger().info('Motors DISARMED (IDLE)')

    def _on_e_stop(self, msg: Bool):
        if msg.data and not self.e_stop_active:
            self.e_stop_active = True
            self.pub_cmd.publish(Twist())
            self.get_logger().warn('E-stop ACTIVE')
        elif not msg.data and self.e_stop_active:
            self.e_stop_active = False
            self.get_logger().info('E-stop cleared')

    def _on_cmd_vel(self, msg: Twist):
        self.last_linear = msg.linear.x
        self.last_angular = msg.angular.z
        if self.armed and not self.e_stop_active:
            self.pub_cmd.publish(msg)

    def _publish_state(self):
        state = CLOSED_LOOP_CONTROL if self.armed else IDLE
        data = {
            "left_state":       state,
            "right_state":      state,
            "left_errors":      0,
            "right_errors":     0,
            "armed":            self.armed,
            "bus_voltage":      MOCK_BUS_VOLTAGE,
            "bus_current":      MOCK_BUS_CURRENT,
            "left_fet_temp":    MOCK_FET_TEMP,
            "left_motor_temp":  MOCK_MOTOR_TEMP,
            "right_fet_temp":   MOCK_FET_TEMP,
            "right_motor_temp": MOCK_MOTOR_TEMP,
            "thermal_state":    MOCK_THERMAL_STATE,
        }
        self.pub_state.publish(String(data=json.dumps(data)))

    def _publish_debug(self):
        data = {
            "cmd_linear":         self.last_linear,
            "cmd_angular":        self.last_angular,
            "left_target":        0.0,
            "right_target":       0.0,
            "left_meas":          0.0,
            "right_meas":         0.0,
            "armed":              self.armed,
            "e_stop":             self.e_stop_active,
            "cmd_valid":          True,
            "zero_cmd":           abs(self.last_linear) < 1e-6 and abs(self.last_angular) < 1e-6,
            "caster_disturbance": 0.0,
        }
        self.pub_debug.publish(String(data=json.dumps(data)))


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
