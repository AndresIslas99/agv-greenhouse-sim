#!/usr/bin/env python3
"""FastAPI control surface for an LLM agent on another LAN machine.

Exposes a small JSON HTTP API on :8090 that wraps live ROS state and lets
the agent inject Nav2 goals. The agent doesn't need rclpy or DDS — just
HTTP. Foxglove handles the visual side; this handles programmatic access.

Endpoints:
  GET  /state                 latest snapshot: pose, gt_pose, error, last events
  GET  /metrics               session totals: distance, collisions, episodes
  GET  /events?since=<ts>     event timeline since a sim-time stamp
  GET  /episodes              list of completed episode summaries
  GET  /viz_url               foxglove URL hint for the human
  POST /goal {x,y,yaw}        send a Nav2 goal via NavigateToPose action
  POST /pause / /play         not implemented yet (would call Isaac via ZMQ)
  POST /reset {x,y,yaw}       not implemented yet
"""
import json
import math
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool

try:
    from nav2_msgs.action import NavigateToPose
    NAV2_AVAILABLE = True
except ImportError:
    NAV2_AVAILABLE = False

import cv2
import numpy as np
from cv_bridge import CvBridge

from fastapi import FastAPI, HTTPException, Query, Response
from pydantic import BaseModel
import uvicorn


# ─── Data model for POST /goal ────────────────────────────────────────────

class Goal(BaseModel):
    x: float
    y: float
    yaw: float = 0.0


class ResetPose(BaseModel):
    x: float = 5.5
    y: float = 0.0
    yaw: float = 0.0
    z: float = 0.2


# ─── ROS-side state aggregator ────────────────────────────────────────────

class SimState(Node):
    """Subscribes to /agv/* and /agv/sim/* and keeps the latest values."""

    def __init__(self):
        super().__init__('sim_api')
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8090)
        self.declare_parameter('foxglove_url', 'http://localhost:8765')

        self.lock = threading.RLock()
        self.gt_pose = None       # PoseStamped
        self.est_pose = None      # Odometry
        self.loc_error = None     # dict
        self.events = deque(maxlen=500)  # list of dicts (most recent last)
        self.episodes = deque(maxlen=200)
        self.session_start = time.time()
        self.cumulative_distance = 0.0
        self.cumulative_collisions = 0
        self._last_xy = None
        self._latest_jpeg = None      # bytes, last encoded frame
        self._latest_jpeg_t = None    # wall time when encoded
        self._bridge = CvBridge()

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        # Camera images come on best-effort QoS from Isaac
        cam_qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(PoseStamped, '/agv/sim/ground_truth/pose',
                                 self._on_gt, qos)
        self.create_subscription(Odometry, '/agv/odometry/global',
                                 self._on_est, qos)
        self.create_subscription(String, '/agv/sim/localization_error',
                                 self._on_loc, qos)
        self.create_subscription(String, '/agv/sim/events', self._on_event, qos)
        self.create_subscription(String, '/agv/sim/episode_summary',
                                 self._on_episode, latched)
        self.create_subscription(Image, '/agv/zed/left/image_rect_color',
                                 self._on_image, cam_qos)

        # Reset request publisher (latched). An OmniGraph reset handler in
        # Isaac is required to actually teleport — currently the topic is
        # advertised so an external script can listen, but no in-sim consumer
        # exists yet. POST /reset returns 202 Accepted with a hint.
        self.reset_pub = self.create_publisher(
            PoseStamped, '/agv/sim/reset_request', latched)
        # E-stop helper
        self.estop_pub = self.create_publisher(Bool, '/agv/e_stop', qos)
        self.motor_enable_pub = self.create_publisher(Bool, '/agv/motor_enable', qos)

        if NAV2_AVAILABLE:
            self.nav_client = ActionClient(self, NavigateToPose,
                                           '/agv/navigate_to_pose')
        else:
            self.nav_client = None

        self.get_logger().info('sim_api state aggregator up')

    # ── topic callbacks ──

    def _on_gt(self, msg: PoseStamped):
        with self.lock:
            self.gt_pose = msg
            x, y = msg.pose.position.x, msg.pose.position.y
            if self._last_xy is not None:
                self.cumulative_distance += math.hypot(
                    x - self._last_xy[0], y - self._last_xy[1])
            self._last_xy = (x, y)

    def _on_est(self, msg: Odometry):
        with self.lock:
            self.est_pose = msg

    def _on_loc(self, msg: String):
        try:
            d = json.loads(msg.data)
        except Exception:
            return
        with self.lock:
            self.loc_error = d

    def _on_event(self, msg: String):
        try:
            d = json.loads(msg.data)
        except Exception:
            return
        with self.lock:
            self.events.append(d)
            if d.get('event') == 'collision':
                self.cumulative_collisions += 1

    def _on_episode(self, msg: String):
        try:
            d = json.loads(msg.data)
        except Exception:
            return
        with self.lock:
            self.episodes.append(d)

    def _on_image(self, msg: Image):
        # Encode at most every ~200ms to limit CPU; the latest frame is
        # always available regardless of how often /snapshot.jpg is hit.
        now = time.time()
        if self._latest_jpeg_t and (now - self._latest_jpeg_t) < 0.2:
            return
        try:
            cv = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            ok, buf = cv2.imencode('.jpg', cv,
                                   [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if ok:
                with self.lock:
                    self._latest_jpeg = buf.tobytes()
                    self._latest_jpeg_t = now
        except Exception as e:
            self.get_logger().warning(f'image encode failed: {e}',
                                      throttle_duration_sec=10.0)

    # ── snapshots for HTTP ──

    def state(self):
        with self.lock:
            return {
                'session_uptime_s': round(time.time() - self.session_start, 2),
                'gt_pose':  self._pose_dict(self.gt_pose),
                'est_pose': self._odom_dict(self.est_pose),
                'localization_error': self.loc_error,
                'last_event': self.events[-1] if self.events else None,
            }

    def metrics(self):
        with self.lock:
            return {
                'session_uptime_s':   round(time.time() - self.session_start, 2),
                'cumulative_distance_m': round(self.cumulative_distance, 3),
                'cumulative_collisions': self.cumulative_collisions,
                'episodes_completed': len(self.episodes),
                'event_count':        len(self.events),
            }

    def events_since(self, t):
        with self.lock:
            return [e for e in self.events if e.get('t_sim', 0) >= t]

    def episodes_list(self):
        with self.lock:
            return list(self.episodes)

    def send_goal(self, goal: Goal) -> dict:
        if self.nav_client is None:
            return {'ok': False, 'error': 'nav2_msgs not available'}
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            return {'ok': False, 'error': 'NavigateToPose action server not available'}

        from nav2_msgs.action import NavigateToPose
        msg = NavigateToPose.Goal()
        msg.pose.header.frame_id = 'map'
        msg.pose.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = goal.x
        msg.pose.pose.position.y = goal.y
        # yaw → quaternion
        cy = math.cos(goal.yaw * 0.5)
        sy = math.sin(goal.yaw * 0.5)
        msg.pose.pose.orientation.z = sy
        msg.pose.pose.orientation.w = cy
        future = self.nav_client.send_goal_async(msg)
        return {'ok': True, 'sent': {'x': goal.x, 'y': goal.y, 'yaw': goal.yaw}}

    def snapshot(self):
        """Return (jpeg_bytes, age_seconds) or (None, None) if no frame yet."""
        with self.lock:
            if self._latest_jpeg is None:
                return None, None
            age = time.time() - self._latest_jpeg_t
            return self._latest_jpeg, age

    def request_reset(self, pose: ResetPose) -> dict:
        """Publish a reset request. The in-sim handler (open_greenhouse.py)
        consumes /agv/sim/reset_request and physically teleports the robot
        via omni.isaac.dynamic_control, zeroing linear/angular velocity.

        We also disarm the motor gate and pulse e-stop so the brain doesn't
        try to drive into the new pose with stale commands.
        """
        msg = PoseStamped()
        msg.header.frame_id = 'world'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = pose.x
        msg.pose.position.y = pose.y
        msg.pose.position.z = pose.z
        cy = math.cos(pose.yaw * 0.5)
        sy = math.sin(pose.yaw * 0.5)
        msg.pose.orientation.z = sy
        msg.pose.orientation.w = cy
        self.reset_pub.publish(msg)
        # Disarm + e-stop so the robot stops moving while teleport applies
        self.motor_enable_pub.publish(Bool(data=False))
        self.estop_pub.publish(Bool(data=True))
        return {
            'ok': True,
            'requested': {'x': pose.x, 'y': pose.y, 'yaw': pose.yaw, 'z': pose.z},
            'note': ('Teleport request sent on /agv/sim/reset_request. '
                     'In-sim handler will apply within 1 physics step and '
                     'confirm on /agv/sim/reset_done. Motors disarmed; '
                     're-arm via POST /motor/enable {"on": true} after.'),
        }

    def set_motor_enable(self, on: bool) -> dict:
        self.motor_enable_pub.publish(Bool(data=on))
        return {'ok': True, 'motor_enable': on}

    def set_estop(self, on: bool) -> dict:
        self.estop_pub.publish(Bool(data=on))
        return {'ok': True, 'e_stop': on}

    # ── helpers ──

    @staticmethod
    def _pose_dict(p):
        if p is None:
            return None
        q = p.pose.orientation
        return {
            'frame': p.header.frame_id,
            'x': round(p.pose.position.x, 4),
            'y': round(p.pose.position.y, 4),
            'z': round(p.pose.position.z, 4),
            'yaw': round(_yaw(q.x, q.y, q.z, q.w), 4),
        }

    @staticmethod
    def _odom_dict(o):
        if o is None:
            return None
        q = o.pose.pose.orientation
        return {
            'frame': o.header.frame_id,
            'x': round(o.pose.pose.position.x, 4),
            'y': round(o.pose.pose.position.y, 4),
            'yaw': round(_yaw(q.x, q.y, q.z, q.w), 4),
            'vx': round(o.twist.twist.linear.x, 4),
            'wz': round(o.twist.twist.angular.z, 4),
        }


def _yaw(x, y, z, w):
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


# ─── FastAPI app ──────────────────────────────────────────────────────────

def make_app(state: SimState) -> FastAPI:
    app = FastAPI(title='AGV Sim Validation API', version='0.1.0')

    @app.get('/')
    def root():
        return {
            'name': 'agv_sim_validation',
            'endpoints': {
                'GET /state':            'snapshot of pose, GT, error, last event',
                'GET /metrics':          'session totals',
                'GET /events?since=':    'event timeline since a sim-time',
                'GET /episodes':         'completed episode summaries',
                'GET /snapshot.jpg':     'last ZED left frame, JPEG',
                'GET /viz_url':          'Foxglove WebSocket endpoint',
                'POST /goal':            'Nav2 goal: {x, y, yaw}',
                'POST /reset':           'request teleport: {x, y, yaw, z}',
                'POST /motor/enable':    'arm/disarm: {on: bool}',
                'POST /e_stop':          'set/clear e-stop: {on: bool}',
            },
        }

    @app.get('/state')
    def get_state():
        return state.state()

    @app.get('/metrics')
    def get_metrics():
        return state.metrics()

    @app.get('/events')
    def get_events(since: float = Query(0.0)):
        return {'since': since, 'events': state.events_since(since)}

    @app.get('/episodes')
    def get_episodes():
        return {'count': len(state.episodes), 'episodes': state.episodes_list()}

    @app.get('/episodes/{episode_id}/bag')
    def get_episode_bag(episode_id: int):
        """Return the path to the rosbag for an episode (server-local path).

        Useful when the LLM agent runs on the same host (or an NFS mount).
        For remote retrieval, the agent should `scp` or `rsync` the path.
        """
        import os
        for ep in state.episodes_list():
            if ep.get('episode_id') == episode_id:
                bag = ep.get('bag_path')
                if not bag:
                    raise HTTPException(status_code=404,
                                        detail='No bag recorded for this episode')
                exists = os.path.isdir(bag) or os.path.isfile(bag + '.db3')
                return {
                    'episode_id': episode_id,
                    'bag_path': bag,
                    'exists': exists,
                    'host_hint': 'rsync from this path; LLM-side download not implemented.',
                }
        raise HTTPException(status_code=404, detail='Episode not found')

    @app.get('/snapshot.jpg')
    def get_snapshot():
        jpeg, age = state.snapshot()
        if jpeg is None:
            raise HTTPException(status_code=404,
                                detail='No camera frame received yet')
        return Response(
            content=jpeg, media_type='image/jpeg',
            headers={'X-Frame-Age-Seconds': f'{age:.3f}'},
        )

    @app.get('/viz_url')
    def get_viz():
        url = state.get_parameter('foxglove_url').value
        return {'foxglove_websocket': url.replace('http://', 'ws://').rstrip('/'),
                'foxglove_studio': 'https://studio.foxglove.dev'}

    @app.post('/goal')
    def post_goal(goal: Goal):
        result = state.send_goal(goal)
        if not result.get('ok'):
            raise HTTPException(status_code=503, detail=result.get('error'))
        return result

    @app.post('/reset')
    def post_reset(pose: ResetPose):
        return state.request_reset(pose)

    class OnOff(BaseModel):
        on: bool

    @app.post('/motor/enable')
    def post_motor_enable(payload: OnOff):
        return state.set_motor_enable(payload.on)

    @app.post('/e_stop')
    def post_estop(payload: OnOff):
        return state.set_estop(payload.on)

    return app


def main():
    rclpy.init()
    state = SimState()
    app = make_app(state)

    host = state.get_parameter('host').value
    port = state.get_parameter('port').value

    # Run rclpy spin in a background thread
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(state)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    state.get_logger().info(f'FastAPI listening on http://{host}:{port}')

    try:
        uvicorn.run(app, host=host, port=port, log_level='warning')
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        state.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
