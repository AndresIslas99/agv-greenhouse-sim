#!/usr/bin/env python3
"""Ground-truth list of AprilTags currently visible to the front camera.

Subscribes:
  /agv/sim/ground_truth/pose      PoseStamped (world frame)

Publishes:
  /agv/sim/ground_truth/visible_markers   std_msgs/String JSON, ~5 Hz

Geometry (matches the ZED 2i mount in the URDF):
  camera origin = base_link + (cam_dx, cam_dy, cam_dz)  in robot frame
  camera optical axis = robot +X (after yaw)
  HFOV = 110° (configurable)

A tag is reported as visible when:
  1. It's a vertical (wall-mounted) tag — floor/ceiling tags are skipped here.
  2. Distance camera→tag is below `max_distance_m`.
  3. Tag direction from camera lies within ±HFOV/2 of optical axis.
  4. Tag's outward normal points back toward the camera (incidence < 80°).

The LLM compares this list against the brain's actual /detections topic to
catch perception bugs (tag is visible in GT but apriltag_ros missed it).
"""
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from agv_sim_validation.markers_db import (
    APRILTAG_PLACEMENTS, tag_normal_xy, is_floor_tag,
)


def _yaw_from_quat(x, y, z, w):
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def _wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class VisibleMarkers(Node):
    def __init__(self):
        super().__init__('visible_markers')

        self.declare_parameter('rate_hz', 5.0)
        self.declare_parameter('max_distance_m', 5.0)
        self.declare_parameter('hfov_deg', 110.0)
        self.declare_parameter('max_incidence_deg', 80.0)
        # Camera position relative to base_link (matches URDF ZED mount)
        self.declare_parameter('cam_offset_x', 0.70)
        self.declare_parameter('cam_offset_y', 0.0)
        self.declare_parameter('cam_offset_z', -0.055)

        self._max_d = float(self.get_parameter('max_distance_m').value)
        self._hfov_half = math.radians(float(self.get_parameter('hfov_deg').value)) * 0.5
        self._max_inc = math.radians(float(self.get_parameter('max_incidence_deg').value))
        self._cam_dx = float(self.get_parameter('cam_offset_x').value)
        self._cam_dy = float(self.get_parameter('cam_offset_y').value)
        self._cam_dz = float(self.get_parameter('cam_offset_z').value)

        self._pose = None  # (x, y, z, yaw)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PoseStamped, '/agv/sim/ground_truth/pose',
                                 self._on_pose, qos)
        self.pub = self.create_publisher(
            String, '/agv/sim/ground_truth/visible_markers', qos)

        rate = float(self.get_parameter('rate_hz').value)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'visible_markers up — {len(APRILTAG_PLACEMENTS)} tags in DB, '
            f'max_d={self._max_d:.1f}m hfov={math.degrees(2*self._hfov_half):.0f}°')

    def _on_pose(self, msg: PoseStamped):
        p = msg.pose.position
        q = msg.pose.orientation
        self._pose = (p.x, p.y, p.z, _yaw_from_quat(q.x, q.y, q.z, q.w))

    def _tick(self):
        if self._pose is None:
            return
        rx, ry, rz, yaw = self._pose
        # Camera world position: base_link + R(yaw) * offset
        c = math.cos(yaw)
        s = math.sin(yaw)
        cam_x = rx + c * self._cam_dx - s * self._cam_dy
        cam_y = ry + s * self._cam_dx + c * self._cam_dy
        cam_z = rz + self._cam_dz

        visible = []
        for (tag_id, tx, ty, tz, trx, try_, trz) in APRILTAG_PLACEMENTS:
            if is_floor_tag(trx, try_, trz):
                # Floor/ceiling tags aren't in the front camera frustum
                continue
            dx = tx - cam_x
            dy = ty - cam_y
            dz = tz - cam_z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if dist > self._max_d or dist < 1e-3:
                continue
            # Bearing from camera optical axis (robot +X after yaw)
            bearing = _wrap(math.atan2(dy, dx) - yaw)
            if abs(bearing) > self._hfov_half:
                continue
            # Tag's outward normal in XY plane (unit vector)
            nx, ny = tag_normal_xy(trx, try_, trz)
            if nx == 0.0 and ny == 0.0:
                continue
            # Vector from tag to camera (must align with tag normal)
            inv = 1.0 / dist
            vx = -dx * inv
            vy = -dy * inv
            dot = max(-1.0, min(1.0, nx * vx + ny * vy))
            incidence = math.acos(dot)
            if incidence > self._max_inc:
                continue
            visible.append({
                'id': int(tag_id),
                'distance_m': round(dist, 3),
                'bearing_rad': round(bearing, 4),
                'incidence_deg': round(math.degrees(incidence), 1),
            })

        # Sort by distance for stability
        visible.sort(key=lambda d: d['distance_m'])

        payload = {
            't_sim': self.get_clock().now().nanoseconds * 1e-9,
            'robot_pose': [round(rx, 3), round(ry, 3), round(yaw, 4)],
            'camera_pose': [round(cam_x, 3), round(cam_y, 3), round(cam_z, 3)],
            'count': len(visible),
            'visible': visible,
        }
        self.pub.publish(String(data=json.dumps(payload)))


def main():
    rclpy.init()
    node = VisibleMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
