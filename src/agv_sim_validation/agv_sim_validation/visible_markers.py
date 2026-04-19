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


def _quat_from_euler_xyz_deg(rx_deg, ry_deg, rz_deg):
    """Intrinsic XYZ Euler (degrees) → quaternion (qx, qy, qz, qw).

    Matches the rotation convention the USD builder uses for AprilTag
    placements (RotateXYZ on the parent Xform).
    """
    rx = math.radians(rx_deg) * 0.5
    ry = math.radians(ry_deg) * 0.5
    rz = math.radians(rz_deg) * 0.5
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    # XYZ intrinsic = Rx * Ry * Rz applied as p' = R p
    qx = sx * cy * cz + cx * sy * sz
    qy = cx * sy * cz - sx * cy * sz
    qz = cx * cy * sz + sx * sy * cz
    qw = cx * cy * cz - sx * sy * sz
    return (qx, qy, qz, qw)


class VisibleMarkers(Node):
    def __init__(self):
        super().__init__('visible_markers')

        self.declare_parameter('rate_hz', 5.0)
        self.declare_parameter('max_distance_m', 5.0)
        self.declare_parameter('hfov_deg', 110.0)
        self.declare_parameter('vfov_deg', 70.0)         # ZED 2i HD720
        self.declare_parameter('max_incidence_deg', 80.0)
        # Floor tags use a relaxed incidence — a forward-mounted camera
        # always sees them obliquely (camera at z=0.145, floor at z=0).
        # 88° lets the rail-entry floor tags stay visible up to ~2 m
        # while still rejecting tags so far away the angle is essentially
        # edge-on. Tune separately if real apriltag_ros has a stricter
        # detection envelope.
        self.declare_parameter('max_incidence_floor_deg', 88.0)
        # Camera position relative to base_link (matches URDF ZED mount)
        self.declare_parameter('cam_offset_x', 0.70)
        self.declare_parameter('cam_offset_y', 0.0)
        self.declare_parameter('cam_offset_z', -0.055)

        self._max_d = float(self.get_parameter('max_distance_m').value)
        self._hfov_half = math.radians(float(self.get_parameter('hfov_deg').value)) * 0.5
        self._vfov_half = math.radians(float(self.get_parameter('vfov_deg').value)) * 0.5
        self._max_inc = math.radians(float(self.get_parameter('max_incidence_deg').value))
        self._max_inc_floor = math.radians(
            float(self.get_parameter('max_incidence_floor_deg').value))
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
            dx = tx - cam_x
            dy = ty - cam_y
            dz = tz - cam_z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if dist > self._max_d or dist < 1e-3:
                continue

            # Horizontal bearing from camera optical axis (robot +X after yaw).
            # Use only the XY projection; vertical handled separately.
            xy_dist = math.sqrt(dx * dx + dy * dy)
            if xy_dist < 1e-6:
                continue  # tag directly under/over camera — degenerate
            bearing = _wrap(math.atan2(dy, dx) - yaw)
            if abs(bearing) > self._hfov_half:
                continue

            # Vertical (pitch) angle from optical axis. Negative = above
            # camera, positive = below. ZED's optical axis is horizontal
            # so the FoV is symmetric around 0.
            pitch = math.atan2(-dz, xy_dist)  # -dz: tag below camera => +pitch
            if abs(pitch) > self._vfov_half:
                continue

            floor = is_floor_tag(trx, try_, trz)
            if floor:
                # Floor tag normal = +Z. Vector from tag UP to camera =
                # (cam_z - tz)/dist along +Z. Incidence is the angle
                # between this vector and tag's +Z normal.
                vz_to_cam = (cam_z - tz) / dist
                if vz_to_cam <= 0:
                    continue   # camera below floor, can't see tag
                incidence = math.acos(max(-1.0, min(1.0, vz_to_cam)))
                if incidence > self._max_inc_floor:
                    continue
            else:
                # Wall tag normal in XY plane.
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
            # Tag world quaternion from the placement's intrinsic XYZ.
            # The brain shim takes this + the camera world TF (computed
            # locally on the Jetson from /tf) to derive the tag's pose
            # in camera_optical frame for AprilTagDetectionArray.
            tqx, tqy, tqz, tqw = _quat_from_euler_xyz_deg(trx, try_, trz)
            visible.append({
                'id': int(tag_id),
                'distance_m': round(dist, 3),
                'bearing_rad': round(bearing, 4),
                'incidence_deg': round(math.degrees(incidence), 1),
                'tag_world_pose': {
                    'x': round(tx, 4), 'y': round(ty, 4), 'z': round(tz, 4),
                    'qx': round(tqx, 6), 'qy': round(tqy, 6),
                    'qz': round(tqz, 6), 'qw': round(tqw, 6),
                },
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
