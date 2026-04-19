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
import os
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

        # Telemetry ring buffers — brain can GET /sim/telemetry to read
        # a live snapshot of RTF, event counters, and topic rates without
        # parsing Isaac logs. Keeps debugging Phase 2 failures cheap.
        self._clock_samples = deque(maxlen=60)   # (wall_t, sim_t) pairs, last ~6s
        self._gt_pose_ts = deque(maxlen=100)     # wall timestamps of GT msgs
        self._cloud_ts = deque(maxlen=100)       # wall timestamps of cloud msgs
        self._watchdog_fires_total = 0
        self._unstick_fires_total = 0
        self._last_teleport_wall_t = 0.0         # updated when reset confirms

        # Sim self-healing watchdog. Isaac occasionally enters a "ghost"
        # state — process alive, /clock topic has a publisher endpoint,
        # but no actual messages flow because PhysX or Kit timeline is
        # hung. The user shouldn't have to notice and intervene; if no
        # /clock has arrived for SIM_HEALTH_TIMEOUT_S, we POST /sim/restart
        # ourselves. SIM_HEALTH_GRACE_S protects the cold-start window
        # and the ~30 s relaunch window from triggering recursive restarts.
        self._sim_health_last_clock_t = time.time()
        # Seed last_restart_t to "now" so the GRACE_S window protects
        # the cold-start period — sim_api commonly comes up while Isaac
        # is mid-relaunch, and we don't want the watchdog to fire on
        # the natural /clock gap of that initial relaunch.
        self._sim_health_last_restart_t = time.time()
        self._sim_health_restarts_total = 0
        # Wall time at which /clock came back from a stall (or first
        # arrived). Reset on every clock gap > CLOCK_HEALTHY_GAP_S so
        # `clock_healthy_streak_s` exposes "how long has clock been
        # continuously fresh". Brain harness uses this to wait until a
        # known-good interval has elapsed after a self-heal restart.
        self._sim_health_clock_healthy_since = None
        # Operator pause flag — when True, the self-heal watchdog is
        # silenced (otherwise a /sim/clock_pause >10s would falsely
        # trigger an Isaac restart). Set by /sim/clock_pause, cleared
        # by /sim/clock_resume. Exposed via /sim/telemetry.paused.
        self._sim_paused = False
        # Last /sim/set_rtf target value the brain requested. None means
        # rate limit is disabled (sim runs as fast as HW allows). Exposed
        # via /sim/telemetry.rtf_target.
        self._rtf_target = None

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

        # Telemetry subs — /clock to compute RTF, cloud to compute its
        # live rate. Both are light (rate observer only, no payload read).
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import PointCloud2, JointState
        self.create_subscription(Clock, '/clock', self._on_clock, qos)
        self.create_subscription(PointCloud2,
                                 '/agv/zed/point_cloud/cloud_registered',
                                 self._on_cloud, cam_qos)
        # Motor state + joint encoders — needed by /motor/prepare and the
        # enhanced /reset response so the brain harness can confirm the
        # sim is actually "ready to drive" before issuing a goal.
        self.motor_state = None     # parsed JSON dict from sim_motor_gate
        self.joint_state = None     # latest JointState from /agv/joint_states
        self.create_subscription(String, '/agv/motor_state',
                                 self._on_motor_state, qos)
        self.create_subscription(JointState, '/agv/joint_states',
                                 self._on_joint_state, qos)
        # /agv/sim/reset_done is latched. POST /reset uses it to confirm
        # the teleport actually applied before responding.
        self._reset_done_event = threading.Event()
        self.create_subscription(Bool, '/agv/sim/reset_done',
                                 self._on_reset_done, latched)

        self.reset_pub = self.create_publisher(
            PoseStamped, '/agv/sim/reset_request', latched)
        self.control_pub = self.create_publisher(
            String, '/agv/sim/control', qos)
        self.estop_pub = self.create_publisher(Bool, '/agv/e_stop', qos)
        self.motor_enable_pub = self.create_publisher(Bool, '/agv/motor_enable', qos)

        if NAV2_AVAILABLE:
            self.nav_client = ActionClient(self, NavigateToPose,
                                           '/agv/navigate_to_pose')
        else:
            self.nav_client = None

        # Watchdog tick @ 2 Hz — checks /clock heartbeat and self-restarts
        # Isaac if it's been silent too long. Cheap (one wall_time read).
        self.create_timer(0.5, self._sim_health_tick)

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
            self._gt_pose_ts.append(time.time())

    # Max wall gap between /clock samples that still counts as "healthy".
    # 2 s leaves headroom over the typical 50–200 ms inter-publish gap at
    # the lowest RTF we've seen (~5 % during heavy frame load).
    CLOCK_HEALTHY_GAP_S = 2.0

    def _on_clock(self, msg):
        with self.lock:
            now_wall = time.time()
            sim_t = msg.clock.sec + msg.clock.nanosec * 1e-9
            gap = now_wall - self._sim_health_last_clock_t
            self._clock_samples.append((now_wall, sim_t))
            self._sim_health_last_clock_t = now_wall
            # Healthy streak: reset whenever we have a gap > threshold;
            # otherwise mark "since" the first message after a gap.
            if (self._sim_health_clock_healthy_since is None
                    or gap > self.CLOCK_HEALTHY_GAP_S):
                self._sim_health_clock_healthy_since = now_wall

    def _on_cloud(self, _msg):
        with self.lock:
            self._cloud_ts.append(time.time())

    def _on_motor_state(self, msg: String):
        """Parse sim_motor_gate JSON status. Expected fields include
        'armed' (bool), 'e_stop' (bool), plus diagnostics we don't
        need. Parse failures fall back to None so the enhanced /reset
        response can report motors_disarmed=null if the topic's dead."""
        try:
            d = json.loads(msg.data)
        except Exception:
            return
        with self.lock:
            self.motor_state = d

    def _on_joint_state(self, msg):
        """Latest wheel + caster joint velocities. Used to derive
        encoders_reset after a teleport."""
        with self.lock:
            self.joint_state = msg

    # Sim health watchdog tunables. Triggered if /clock has been silent
    # for SIM_HEALTH_TIMEOUT_S, with at least SIM_HEALTH_GRACE_S between
    # restarts to absorb the cold-start window.
    SIM_HEALTH_TIMEOUT_S = 10.0
    SIM_HEALTH_GRACE_S   = 60.0

    def _sim_health_tick(self):
        """Self-heal Isaac when /clock stops flowing.

        Isaac sometimes ends up in a "ghost" state — process alive,
        publishers registered, no messages emitted. Root cause varies
        (PhysX hang, Cyclone DDS context corruption after many sim_api
        cycles, Kit timeline frozen). The user shouldn't have to notice
        and intervene; if /clock has been silent SIM_HEALTH_TIMEOUT_S,
        we POST /sim/restart on its behalf.
        """
        now = time.time()
        with self.lock:
            if self._sim_paused:
                # Operator paused the sim deliberately via /sim/clock_pause;
                # /clock will be silent until /sim/clock_resume. Keep the
                # heartbeat fresh so we don't trigger a false self-heal
                # the moment the operator un-pauses.
                self._sim_health_last_clock_t = now
                return
            silent_for = now - self._sim_health_last_clock_t
            since_restart = now - self._sim_health_last_restart_t

        if silent_for < self.SIM_HEALTH_TIMEOUT_S:
            return
        if since_restart < self.SIM_HEALTH_GRACE_S:
            # In the cold-start window or right after a previous self-
            # heal — clock not back yet, don't loop.
            return

        self.get_logger().warn(
            f'/clock silent for {silent_for:.1f}s — auto-restarting Isaac '
            f'(restart #{self._sim_health_restarts_total + 1})')
        with self.lock:
            self._sim_health_last_restart_t = now
            self._sim_health_restarts_total += 1
            # Also reset the heartbeat so we don't trigger again before
            # the restart relaunches Isaac and a fresh /clock arrives.
            self._sim_health_last_clock_t = now
            # Invalidate the healthy streak — brain knows to wait for a
            # fresh window before resuming nav.
            self._sim_health_clock_healthy_since = None

        try:
            self.sim_restart()
        except Exception as e:
            self.get_logger().error(f'self-heal sim_restart failed: {e}')

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
            et = d.get('event')
            if et == 'collision':
                self.cumulative_collisions += 1
            elif et == 'watchdog_recovery':
                self._watchdog_fires_total += 1
            elif et == 'sim_unstick':
                self._unstick_fires_total += 1

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
            return _json_safe({
                'session_uptime_s': round(time.time() - self.session_start, 2),
                'gt_pose':  self._pose_dict(self.gt_pose),
                'est_pose': self._odom_dict(self.est_pose),
                'localization_error': self.loc_error,
                'last_event': self.events[-1] if self.events else None,
            })

    def metrics(self):
        with self.lock:
            return {
                'session_uptime_s':   round(time.time() - self.session_start, 2),
                'cumulative_distance_m': round(self.cumulative_distance, 3),
                'cumulative_collisions': self.cumulative_collisions,
                'episodes_completed': len(self.episodes),
                'event_count':        len(self.events),
            }

    def telemetry(self):
        """Snapshot of sim-internal performance counters, for postmortem.

        Brain-side tests can log this before/after each waypoint so a
        reviewer can correlate RESET_TIMEOUT or ABORTED outcomes with
        watchdog fires, unstick nudges, or a collapsed RTF — without
        parsing Isaac-side stderr.
        """
        now = time.time()

        def _rate(ts_deque, window_s=5.0):
            if len(ts_deque) < 2:
                return 0.0
            cutoff = now - window_s
            recent = [t for t in ts_deque if t >= cutoff]
            if len(recent) < 2:
                return 0.0
            span = recent[-1] - recent[0]
            return (len(recent) - 1) / span if span > 0 else 0.0

        def _events_in(event_name, window_s):
            cutoff_sim = None
            sim_now = None
            # Prefer /clock-time since events are tagged in sim_time
            if self._clock_samples:
                _, sim_now = self._clock_samples[-1]
                cutoff_sim = sim_now - window_s
            if cutoff_sim is None:
                return 0
            return sum(1 for e in self.events
                       if e.get('event') == event_name
                       and e.get('t_sim', 0) >= cutoff_sim)

        with self.lock:
            # RTF over the full clock-sample window (~6 s).
            rtf = 0.0
            sim_time_s = 0.0
            if len(self._clock_samples) >= 2:
                wall_a, sim_a = self._clock_samples[0]
                wall_b, sim_b = self._clock_samples[-1]
                wall_span = wall_b - wall_a
                sim_span = sim_b - sim_a
                if wall_span > 0:
                    rtf = sim_span / wall_span
                sim_time_s = sim_b

            return {
                'rtf_6s':                   round(rtf, 3),
                'sim_time_s':               round(sim_time_s, 2),
                'wall_time_s':              round(now - self.session_start, 2),
                'gt_pose_rate_hz':          round(_rate(self._gt_pose_ts, 5.0), 2),
                'pointcloud_rate_hz':       round(_rate(self._cloud_ts, 5.0), 2),
                'watchdog_fires_total':     self._watchdog_fires_total,
                'watchdog_fires_last_60s':  _events_in('watchdog_recovery', 60.0),
                'unstick_fires_total':      self._unstick_fires_total,
                'unstick_fires_last_60s':   _events_in('sim_unstick', 60.0),
                'last_teleport_wall_ago_s': (
                    round(now - self._last_teleport_wall_t, 2)
                    if self._last_teleport_wall_t > 0 else None
                ),
                'self_heal_restarts_total': self._sim_health_restarts_total,
                'last_clock_msg_ago_s':
                    round(now - self._sim_health_last_clock_t, 2),
                # Seconds /clock has been continuously fresh (max gap 2 s).
                # Brain harness should wait until ≥ 30 s before resuming
                # navigation after a self-heal restart, so its TF cache
                # has settled on the new sim_time epoch.
                'clock_healthy_streak_s': (
                    round(now - self._sim_health_clock_healthy_since, 2)
                    if self._sim_health_clock_healthy_since is not None
                    else 0.0
                ),
                'paused': bool(self._sim_paused),
                'rtf_target': self._rtf_target,   # None = unlimited
            }

    def events_since(self, t):
        with self.lock:
            return _json_safe([e for e in self.events if e.get('t_sim', 0) >= t])

    def episodes_list(self):
        with self.lock:
            return _json_safe(list(self.episodes))

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

    def _on_reset_done(self, _msg: Bool):
        # Handler signals each successful teleport with True. We just flag
        # the event; request_reset clears it before publishing so we know
        # the next True is for THIS request, not a previous one.
        self._reset_done_event.set()

    def _latest_gt_pose_dict(self):
        """Thread-safe snapshot of the latest GT pose as {x, y, yaw} or None."""
        with self.lock:
            g = self.gt_pose
        if g is None:
            return None
        q = g.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return {
            'x': float(g.pose.position.x),
            'y': float(g.pose.position.y),
            'yaw': float(yaw),
        }

    async def request_reset_async(
            self,
            pose: ResetPose,
            xy_tol: float = 0.05,
            yaw_tol_rad: float = math.radians(5.0),
            handler_ack_timeout_s: float = 2.0,
            gt_timeout_s: float = 10.0) -> dict:
        """Publish a reset request and BLOCK (async) until GT converges.

        Pipeline:
          1. Clear reset_done_event, publish /agv/sim/reset_request and
             disarm motor + pulse e-stop (same as the old sync path).
          2. Poll the reset_done_event until the handler acks the teleport
             (typ. ~100 ms). Bounded by handler_ack_timeout_s.
          3. Poll /agv/sim/ground_truth/pose until it is within xy_tol /
             yaw_tol_rad of the request, or gt_timeout_s elapses. This
             absorbs three distinct failure modes that were biting the
             brain's round-34 test:
               a. Race — reset_done fires on the same Kit tick that
                  applies set_rigid_body_pose, but the GT topic is
                  published by a 10 Hz ROS timer and typically lags the
                  teleport by 100-800 ms. A caller that reads GT right
                  after confirmed:true used to see the OLD pose.
               b. Post-teleport bounce — with friction μ=15 the chassis
                  can oscillate for ~1 s before settling. Polling GT
                  waits out the bounce.
               c. Watchdog double-teleport — if the first teleport lands
                  the chassis in an invalid pose (interpenetrating an
                  obstacle or up in the air), the ejection watchdog in
                  open_greenhouse.py teleports it back to spawn. The
                  brain would then be told "confirmed" at a pose that
                  never was. Polling GT forces us to report the FINAL
                  observed pose.

        Returns:
          On success  {ok, confirmed:True, requested, observed,
                       wait_ms, gt_samples, handler_acked}
          On timeout  {ok:False, confirmed:False, reason:"gt_convergence_timeout",
                       requested, observed (last seen or None), wait_ms,
                       gt_samples, handler_acked, note}
        """
        import asyncio
        start = time.monotonic()

        # (1) Publish the reset + disarm motor chain.
        self._reset_done_event.clear()
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
        self.motor_enable_pub.publish(Bool(data=False))
        self.estop_pub.publish(Bool(data=True))

        # (2) Wait for the handler to ack the teleport application.
        handler_ack_deadline = start + handler_ack_timeout_s
        while not self._reset_done_event.is_set():
            if time.monotonic() > handler_ack_deadline:
                break
            await asyncio.sleep(0.02)
        handler_acked = self._reset_done_event.is_set()
        if handler_acked:
            with self.lock:
                self._last_teleport_wall_t = time.time()

        # (3) Poll GT until convergence or timeout. Re-publish the disarm
        # signals every ~500 ms so any actor that kept emitting cmd_vel or
        # flipping motor_enable back on after our initial pulse gets
        # overridden. Without this, a running Nav2/behavior-tree on the
        # brain can push the robot off the requested pose during the poll
        # window, producing a gt_convergence_timeout we could have avoided.
        deadline = start + gt_timeout_s
        gt_samples = 0
        last_observed = None
        last_disarm_t = start
        requested = {
            'x': pose.x, 'y': pose.y, 'yaw': pose.yaw, 'z': pose.z,
        }
        while time.monotonic() < deadline:
            if (time.monotonic() - last_disarm_t) > 0.5:
                self.motor_enable_pub.publish(Bool(data=False))
                self.estop_pub.publish(Bool(data=True))
                last_disarm_t = time.monotonic()
            gt = self._latest_gt_pose_dict()
            if gt is not None:
                gt_samples += 1
                last_observed = gt
                dxy = math.hypot(gt['x'] - pose.x, gt['y'] - pose.y)
                dyaw = _angle_wrap(gt['yaw'] - pose.yaw)
                if dxy < xy_tol and abs(dyaw) < yaw_tol_rad:
                    return {
                        'ok': True,
                        'confirmed': True,
                        'pose_converged': True,
                        'requested': requested,
                        'observed': gt,
                        'wait_ms': int((time.monotonic() - start) * 1000),
                        'gt_samples': gt_samples,
                        'handler_acked': handler_acked,
                        'velocities_zeroed': True,  # _apply_teleport_dc set+zero on every body
                        'motors_disarmed': self._check_motors_disarmed(),
                        'encoders_reset': self._check_encoders_reset(),
                        'timestamp_sim': self._current_sim_time(),
                        'note': 'Teleport converged within tolerance.',
                    }
            await asyncio.sleep(0.05)

        return {
            'ok': False,
            'confirmed': False,
            'pose_converged': False,
            'reason': 'gt_convergence_timeout',
            'requested': requested,
            'observed': last_observed,
            'wait_ms': int((time.monotonic() - start) * 1000),
            'gt_samples': gt_samples,
            'handler_acked': handler_acked,
            'velocities_zeroed': bool(handler_acked),  # handler ack means dc call ran
            'motors_disarmed': self._check_motors_disarmed(),
            'encoders_reset': self._check_encoders_reset(),
            'timestamp_sim': self._current_sim_time(),
            'note': (
                f'GT did not reach ({pose.x:.2f}, {pose.y:.2f}, '
                f'yaw={pose.yaw:.2f}) within {gt_timeout_s:.0f} s. '
                f'{"Handler did ack the teleport — check watchdog / obstacle."  if handler_acked else "Handler never acked — queue saturated or dead."}'
            ),
        }

    # ── Reset-response helpers (used by request_reset_async) ──

    def _check_motors_disarmed(self):
        """True iff sim_motor_gate latest /agv/motor_state reports armed=False.
        None when the topic hasn't been observed yet."""
        with self.lock:
            ms = self.motor_state
        if ms is None:
            return None
        # sim_motor_gate publishes 'armed' as bool. Anything else falls
        # back to None so the brain can tell "unknown" vs "still armed".
        if 'armed' not in ms:
            return None
        return not bool(ms['armed'])

    def _check_encoders_reset(self, tol_rad_s=0.1):
        """True iff each wheel/caster joint reports |v| < tol_rad_s in
        the latest JointState. None if no JointState seen yet.

        tol_rad_s=0.1 (~5.7°/s, ~8 mm/s wheel surface) tolerates the
        small residual jitter PhysX shows for ~50ms after a teleport
        even though _apply_teleport_dc set DOF velocities to zero."""
        with self.lock:
            js = self.joint_state
        if js is None or not js.velocity:
            return None
        return all(abs(v) < tol_rad_s for v in js.velocity)

    def _current_sim_time(self):
        """Latest sim_time_s from /clock samples, or None if no clock yet."""
        with self.lock:
            if self._clock_samples:
                return round(self._clock_samples[-1][1], 3)
        return None

    # ── Atomic motor prepare ──

    async def motor_prepare(self, wait_armed_s: float = 2.0) -> dict:
        """Atomic: clear e_stop, arm motor gate, wait for /agv/motor_state
        to confirm armed=true. Replaces the brain's 2-call sequence
        (POST /e_stop + POST /motor/enable) so there's no race window
        where cmd_vel arrives between the two commands and gets dropped.
        Idempotent — safe to call multiple times."""
        import asyncio
        start = time.monotonic()
        self.estop_pub.publish(Bool(data=False))
        self.motor_enable_pub.publish(Bool(data=True))
        deadline = start + wait_armed_s
        while time.monotonic() < deadline:
            with self.lock:
                ms = self.motor_state
            if ms is not None and ms.get('armed') is True:
                return {
                    'ok': True,
                    'armed': True,
                    'e_stop': bool(ms.get('e_stop', False)),
                    'wait_ms': int((time.monotonic() - start) * 1000),
                }
            await asyncio.sleep(0.02)
        with self.lock:
            ms = self.motor_state
        return {
            'ok': False,
            'reason': 'arm_timeout',
            'armed': bool(ms.get('armed', False)) if ms else False,
            'e_stop': bool(ms.get('e_stop', True)) if ms else None,
            'wait_ms': int((time.monotonic() - start) * 1000),
            'note': (f'Motor gate did not report armed=true within '
                     f'{wait_armed_s:.1f}s. Check /agv/motor_state liveness.'),
        }

    def set_motor_enable(self, on: bool) -> dict:
        self.motor_enable_pub.publish(Bool(data=on))
        # When arming, also clear any latent e-stop. /reset pulses
        # e_stop=True to inhibit the motor gate during teleport, but
        # never published e_stop=False afterward — sim_motor_gate then
        # held e_stop_active=True permanently and silently dropped every
        # subsequent cmd_vel. Brain reported this as wp10/wp15 stalls
        # (cmd_vel = 0.88 m/s reaching the gate, GT not moving for 90 s).
        # Disarming intentionally does NOT clear e-stop — that path
        # exists to leave the chassis stopped under operator control.
        if on:
            self.estop_pub.publish(Bool(data=False))
        return {'ok': True, 'motor_enable': on, 'e_stop_cleared': bool(on)}

    def set_estop(self, on: bool) -> dict:
        self.estop_pub.publish(Bool(data=on))
        return {'ok': True, 'e_stop': on}

    def sim_control(self, action: str) -> dict:
        self.control_pub.publish(String(data=action))
        return {'ok': True, 'action': action,
                'note': f'{action} command sent to /agv/sim/control. '
                        'Executed by the in-sim handler on the next Kit frame.'}

    def sim_clock_pause(self) -> dict:
        """Freeze sim time at the current sample. /clock stops publishing
        until resume; the self-heal watchdog ignores the silence while
        _sim_paused=True."""
        with self.lock:
            self._sim_paused = True
        result = self.sim_control('pause')
        result['paused'] = True
        return result

    def sim_clock_resume(self) -> dict:
        """Resume sim time from where it was paused. Clears the watchdog
        gate; the next /clock arrival re-arms the heartbeat."""
        with self.lock:
            self._sim_paused = False
            # Reset the heartbeat to "now" so the watchdog has a fresh
            # window to wait for the first post-resume /clock message.
            self._sim_health_last_clock_t = time.time()
        result = self.sim_control('play')
        result['paused'] = False
        return result

    def sim_set_rtf(self, target_rtf: float) -> dict:
        """Cap the Kit main loop to ~target_rtf×realtime via carb settings.

        target_rtf <= 0  → unlimited (default sim behaviour)
        target_rtf == 1  → realtime
        target_rtf  > 0  → cap rateLimitFreq to round(target * 200 Hz)

        target_rtf > the natural RTF the HW can sustain (~0.33 on this
        host) has no acceleration effect — only deceleration is enforced.
        Reports `effective` based on whether the requested cap is below
        the recently-observed rtf_6s; brain can verify post-hoc via
        /sim/telemetry.rtf_6s.
        """
        target = max(0.0, min(2.0, float(target_rtf)))
        with self.lock:
            self._rtf_target = None if target <= 0.0 else target
            recent_rtf = (
                (self._clock_samples[-1][1] - self._clock_samples[0][1]) /
                max(1e-6, self._clock_samples[-1][0] - self._clock_samples[0][0])
                if len(self._clock_samples) >= 2 else None
            )
        result = self.sim_control(f'rtf:{target}')
        result['target_rtf'] = target
        result['recent_rtf'] = round(recent_rtf, 3) if recent_rtf else None
        result['effective_cap'] = (
            None if target <= 0 or recent_rtf is None
            else target < recent_rtf
        )
        return result

    def sim_restart(self) -> dict:
        """Self-healing restart of Isaac Sim.

        Three cases handled:
          (a) Supervisor IS running → write the restart flag, SIGTERM
              Isaac, supervisor relaunches it (auto-play picks up).
          (b) Supervisor is NOT running but Isaac IS → SIGTERM Isaac,
              then subprocess.Popen run_isaac_sim.sh directly so we don't
              leave the host without a sim.
          (c) Neither running → just subprocess.Popen run_isaac_sim.sh.

        Returns a structured dict so the LLM agent on the brain can poll
        /state until gt_pose comes back to confirm recovery.
        """
        import subprocess, signal
        restart_flag = '/tmp/agv_restart_isaac'
        with open(restart_flag, 'w') as f:
            f.write('restart')

        # Detect supervisor presence
        sup = subprocess.run(['pgrep', '-f', 'run_isaac_supervised.sh'],
                             capture_output=True, text=True)
        supervisor_running = bool(sup.stdout.strip())

        # Find live Isaac processes
        result = subprocess.run(
            ['pgrep', '-f', 'bin/isaacsim.*open_greenhouse'],
            capture_output=True, text=True)
        killed = []
        for pid in result.stdout.strip().split('\n'):
            if pid:
                try:
                    os.kill(int(pid), signal.SIGTERM)
                    killed.append(int(pid))
                except Exception:
                    pass

        # Escalate to SIGKILL if Isaac doesn't exit within 3 s. PhysX or
        # the Kit timeline can hang in a state where SIGTERM is dropped
        # (observed: handler rclpy spin thread crashes with
        # ExternalShutdownException, timeline freezes, sim_time stops
        # advancing, but the process refuses to die). Force-kill is the
        # only reliable way out — supervisor will then auto-relaunch.
        if killed:
            import time as _t
            for _ in range(30):  # up to 3 s
                still = subprocess.run(
                    ['pgrep', '-f', 'bin/isaacsim.*open_greenhouse'],
                    capture_output=True, text=True)
                still_pids = {int(p) for p in still.stdout.split() if p}
                if not (still_pids & set(killed)):
                    break
                _t.sleep(0.1)
            else:
                # Loop exited without break → still alive; SIGKILL each.
                for pid in killed:
                    try:
                        os.kill(pid, signal.SIGKILL)
                    except Exception:
                        pass

        # Case (b) and (c): if no supervisor, we're responsible for relaunch
        relaunched_directly = False
        if not supervisor_running:
            # Wait briefly for the SIGTERM'd Isaac to actually exit so the
            # GPU / DDS handles release before the new instance grabs them.
            import time as _t
            for _ in range(20):  # up to 2 s
                still = subprocess.run(
                    ['pgrep', '-f', 'bin/isaacsim.*open_greenhouse'],
                    capture_output=True, text=True)
                if not still.stdout.strip():
                    break
                _t.sleep(0.1)
            try:
                env = os.environ.copy()
                env.setdefault('DISPLAY', ':1')
                env.setdefault('XAUTHORITY', '/run/user/1000/gdm/Xauthority')
                # Use Popen with start_new_session so it survives the
                # FastAPI worker exit. stdout to a known log file.
                log_path = '/tmp/isaac_supervised.log'
                with open(log_path, 'ab') as logf:
                    subprocess.Popen(
                        ['/home/andres/agv-sim/run_isaac_sim.sh'],
                        cwd='/home/andres/agv-sim',
                        env=env,
                        stdin=subprocess.DEVNULL,
                        stdout=logf, stderr=logf,
                        start_new_session=True,
                    )
                relaunched_directly = True
            except Exception as e:
                return {
                    'ok': False, 'action': 'restart',
                    'killed_pids': killed,
                    'supervisor_running': supervisor_running,
                    'relaunched_directly': False,
                    'error': f'direct relaunch failed: {e}',
                }

        return {
            'ok': True, 'action': 'restart',
            'killed_pids': killed,
            'supervisor_running': supervisor_running,
            'relaunched_directly': relaunched_directly,
            'note': (
                'Supervisor will relaunch Isaac (~30-60 s). '
                if supervisor_running else
                'No supervisor — relaunched Isaac directly (~30-60 s). '
            ) + 'Poll GET /state until gt_pose is non-null to confirm recovery.',
        }

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


def _angle_wrap(a):
    """Wrap an angle in radians to [-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _json_safe(obj):
    """Recursively replace non-finite floats (NaN, +Inf, -Inf) with None.

    Pydantic / FastAPI's default JSON encoder rejects these per the JSON
    spec ("Out of range float values are not JSON compliant" → HTTP 500).
    The brain's est_pose / localization_error can occasionally carry NaN
    when the EKF has just initialized or after a reset before the brain
    odometry catches up — that's a transient, not a fault. Sanitize on
    the way out so /state stays a reliable liveness check.
    """
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, dict):
        return {k: _json_safe(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_json_safe(v) for v in obj]
    return obj


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
                'POST /sim/play':        'start Isaac Sim timeline',
                'POST /sim/stop':        'stop Isaac Sim timeline',
                'POST /sim/restart':     'kill + relaunch Isaac (needs supervisor)',
            },
        }

    @app.get('/state')
    def get_state():
        return state.state()

    @app.get('/metrics')
    def get_metrics():
        return state.metrics()

    @app.get('/sim/telemetry')
    def get_telemetry():
        return state.telemetry()

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
    async def post_reset(pose: ResetPose):
        # Async so FastAPI can serve /state, /metrics, etc. in parallel
        # while the reset is polling GT. See request_reset_async docstring.
        return await state.request_reset_async(pose)

    class OnOff(BaseModel):
        on: bool

    @app.post('/motor/enable')
    def post_motor_enable(payload: OnOff):
        return state.set_motor_enable(payload.on)

    @app.post('/motor/prepare')
    async def post_motor_prepare():
        # Atomic clear-estop + arm + verify-armed. Async so /state and
        # /sim/telemetry stay responsive while the verify poll runs.
        return await state.motor_prepare()

    @app.post('/e_stop')
    def post_estop(payload: OnOff):
        return state.set_estop(payload.on)

    @app.post('/sim/play')
    def sim_play():
        return state.sim_control('play')

    @app.post('/sim/stop')
    def sim_stop():
        return state.sim_control('stop')

    @app.post('/sim/restart')
    def sim_restart():
        return state.sim_restart()

    @app.post('/sim/clock_pause')
    def sim_clock_pause():
        return state.sim_clock_pause()

    @app.post('/sim/clock_resume')
    def sim_clock_resume():
        return state.sim_clock_resume()

    class SetRTF(BaseModel):
        target_rtf: float = 1.0   # 0 = unlimited, 1.0 = realtime

    @app.post('/sim/set_rtf')
    def sim_set_rtf(payload: SetRTF):
        return state.sim_set_rtf(payload.target_rtf)

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
