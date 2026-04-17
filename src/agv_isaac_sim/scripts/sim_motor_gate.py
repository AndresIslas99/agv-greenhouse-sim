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
import math
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

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

        # Calibration mirrors drive_shaping_params.yaml so drive_debug
        # publishes real per-wheel motor turns/s. Without these, debug
        # fields are misleading zeros and the LLM agent can't diagnose
        # the drive chain via telemetry.
        self.declare_parameter('wheel_radius', 0.0781)
        self.declare_parameter('track_width', 0.960)
        self.declare_parameter('gear_ratio', 10.0)
        self.declare_parameter('invert_left', True)
        self.declare_parameter('invert_right', False)
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.track_width  = self.get_parameter('track_width').value
        self.gear_ratio   = self.get_parameter('gear_ratio').value
        self.left_sign    = -1.0 if self.get_parameter('invert_left').value else 1.0
        self.right_sign   = -1.0 if self.get_parameter('invert_right').value else 1.0

        # Latest shaped (post-ramp) motor commands and measured wheel rates.
        self.shaped_linear = 0.0
        self.shaped_angular = 0.0
        self.left_meas_rps = 0.0
        self.right_meas_rps = 0.0

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

        # Telemetry sources for accurate drive_debug. shaped_cmd_vel is the
        # post-ramp body-frame Twist from sim_drive_shaping_node; joint_states
        # gives us actual wheel angular velocities from Isaac.
        self.create_subscription(Twist, 'shaped_cmd_vel', self._on_shaped, qos)
        self.create_subscription(JointState, 'joint_states', self._on_joints, 10)

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

    def _on_shaped(self, msg: Twist):
        self.shaped_linear = msg.linear.x
        self.shaped_angular = msg.angular.z

    def _on_joints(self, msg: JointState):
        # Wheel joints are usually named "left_wheel_joint" / "right_wheel_joint"
        # in our URDF. Fall back to first/second velocity if not found.
        if not msg.velocity:
            return
        left_idx = right_idx = None
        for i, name in enumerate(msg.name):
            n = name.lower()
            if 'left' in n and 'wheel' in n:
                left_idx = i
            elif 'right' in n and 'wheel' in n:
                right_idx = i
        if left_idx is not None and left_idx < len(msg.velocity):
            # Joint velocity is rad/s on the wheel axis. Convert to motor turns/s.
            wheel_rps = msg.velocity[left_idx] / (2.0 * math.pi)
            self.left_meas_rps = wheel_rps * self.gear_ratio
        if right_idx is not None and right_idx < len(msg.velocity):
            wheel_rps = msg.velocity[right_idx] / (2.0 * math.pi)
            self.right_meas_rps = wheel_rps * self.gear_ratio

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
        # Inverse kinematics on shaped_cmd_vel → per-wheel motor turns/s
        # (matches the real ODrive driver's drive_debug fields). The sign is
        # the chassis-frame motor direction; Isaac and the brain both apply
        # their own per-wheel inversion downstream.
        v_left_chassis  = self.shaped_linear - self.shaped_angular * self.track_width / 2.0
        v_right_chassis = self.shaped_linear + self.shaped_angular * self.track_width / 2.0
        denom = self.wheel_radius * 2.0 * math.pi
        left_target  = (v_left_chassis  / denom) * self.gear_ratio * self.left_sign
        right_target = (v_right_chassis / denom) * self.gear_ratio * self.right_sign

        data = {
            "cmd_linear":         self.last_linear,
            "cmd_angular":        self.last_angular,
            "left_target":        round(left_target, 4),
            "right_target":       round(right_target, 4),
            "left_meas":          round(self.left_meas_rps, 4),
            "right_meas":         round(self.right_meas_rps, 4),
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
