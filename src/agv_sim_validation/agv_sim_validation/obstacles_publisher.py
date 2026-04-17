#!/usr/bin/env python3
"""Publish a ground-truth list of static obstacles in the greenhouse.

Reads `world_config.yaml` from the `agv_isaac_sim` package share directory
(by default) and emits one latched JSON message with walls, crates and
NVIDIA asset props. Consumers:

  * LLM agent — correlates collision events (`{"with": "Crate2"}`) with the
    obstacle pose so it can tell the navigator what to avoid next time.
  * Offline analysis — join this with `/agv/sim/events` to compute closest
    approach distances per episode.

Publishes (latched, once):
  /agv/sim/ground_truth/obstacles   std_msgs/String JSON

Each entry carries: name, type, pose (x, y, z), yaw_deg, bbox_m (aabb in
local frame — width/depth/height before yaw). For walls and crates this is
the USD geometry, not inflated.

Parameters:
  world_config_path    override path to world_config.yaml (default: ament share)
  include_walls        default true
  include_crates       default true
  include_props        default true — the NVIDIA pallets/KLTs
  include_crop_rows    default false — visible to depth camera but not rigid
"""
import json
import os

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


# Rough bbox defaults for NVIDIA asset props (not defined in world_config).
# Good enough for collision correlation; exact values come from the USD.
PROP_BBOXES = {
    'pallet':    [1.20, 0.80, 0.15],
    'small_klt': [0.40, 0.30, 0.30],
}


class ObstaclesPublisher(Node):
    def __init__(self):
        super().__init__('obstacles_publisher')

        self.declare_parameter('world_config_path', '')
        self.declare_parameter('include_walls', True)
        self.declare_parameter('include_crates', True)
        self.declare_parameter('include_props', True)
        self.declare_parameter('include_crop_rows', False)

        self._path = self._resolve_path(self.get_parameter('world_config_path').value)
        self.get_logger().info(f'loading world_config from {self._path}')

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(
            String, '/agv/sim/ground_truth/obstacles', latched)

        # Publish once at startup; the latched QoS delivers it to late subscribers.
        self.create_timer(0.5, self._publish_once)
        self._published = False

    def _resolve_path(self, override: str) -> str:
        if override:
            return override
        try:
            share = get_package_share_directory('agv_isaac_sim')
            return os.path.join(share, 'config', 'world_config.yaml')
        except PackageNotFoundError:
            # Fall back to source tree layout (development)
            here = os.path.abspath(os.path.dirname(__file__))
            for _ in range(6):
                here = os.path.dirname(here)
                candidate = os.path.join(here, 'src', 'agv_isaac_sim',
                                         'config', 'world_config.yaml')
                if os.path.exists(candidate):
                    return candidate
            raise

    def _build_payload(self) -> dict:
        with open(self._path) as f:
            cfg = yaml.safe_load(f)

        items = []

        if self.get_parameter('include_walls').value:
            for w in cfg.get('walls', {}).get('specs', []):
                pos = w['position']
                size = w['size']
                items.append({
                    'name': f"wall_{w['name'].lower()}",
                    'type': 'wall',
                    'pose':    [float(pos[0]), float(pos[1]), float(pos[2])],
                    'yaw_deg': 0.0,
                    'bbox_m':  [float(size[0]), float(size[1]), float(size[2])],
                })

        if self.get_parameter('include_crates').value:
            for c in cfg.get('obstacles', {}).get('crates', []):
                pos = c['position']
                size = c['size']
                items.append({
                    'name': c['name'],
                    'type': 'crate',
                    'pose':    [float(pos[0]), float(pos[1]), float(pos[2])],
                    'yaw_deg': float(c.get('yaw', 0.0)),
                    'bbox_m':  [float(size[0]), float(size[1]), float(size[2])],
                })

        if self.get_parameter('include_props').value:
            props = cfg.get('props', {})
            if props.get('enabled', False):
                for i, p in enumerate(props.get('items', [])):
                    pos = p['position']
                    asset = p['asset']
                    bbox = PROP_BBOXES.get(asset, [0.5, 0.5, 0.3])
                    items.append({
                        'name': f"{asset}_{i}",
                        'type': 'prop',
                        'asset': asset,
                        'pose':    [float(pos[0]), float(pos[1]), float(pos[2])],
                        'yaw_deg': float(p.get('yaw', 0.0)),
                        'bbox_m':  bbox,
                    })

        if self.get_parameter('include_crop_rows').value:
            cr = cfg.get('crop_rows', {})
            length = float(cr.get('length', 0.0))
            width = float(cr.get('width', 0.0))
            height = float(cr.get('height', 0.0))
            cx = float(cr.get('center_x', 0.0))
            for y in cr.get('y_positions', []):
                items.append({
                    'name': f"crop_row_{y:+.1f}",
                    'type': 'crop_row',
                    'pose':    [cx, float(y), height * 0.5],
                    'yaw_deg': 0.0,
                    'bbox_m':  [length, width, height],
                })

        return {
            't_sim': self.get_clock().now().nanoseconds * 1e-9,
            'source': os.path.basename(self._path),
            'count': len(items),
            'static_obstacles': items,
        }

    def _publish_once(self):
        if self._published:
            return
        try:
            payload = self._build_payload()
        except Exception as e:
            self.get_logger().error(f'failed to build obstacles payload: {e}')
            return
        self.pub.publish(String(data=json.dumps(payload)))
        self.get_logger().info(
            f"published {payload['count']} obstacles "
            f"(walls+crates+props+rows) on /agv/sim/ground_truth/obstacles")
        self._published = True


def main():
    rclpy.init()
    node = ObstaclesPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
