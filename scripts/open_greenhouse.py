"""Open the greenhouse USD AND run the in-sim validation handler.

Responsibilities (all run inside the Isaac Sim Kit process):

  1. Open `greenhouse_with_robot.usd`.
  2. **Auto-play**: start the timeline automatically once the stage is
     loaded — no manual Play button needed. Disable with env var
     `AGV_AUTO_PLAY=0`.
  3. Boot a ROS 2 node (`sim_isaac_handler`) with:
       * `/agv/sim/reset_request` subscriber → queued teleport
       * `/agv/sim/control` subscriber → play / stop / pause the timeline
       * PhysX contact reports → `collision` events on `/agv/sim/events`
       * `/agv/sim/reset_done` publisher (latched Bool)

Threading model:
  * rclpy `spin()` runs in a daemon thread so callbacks fire continuously.
  * Teleport + timeline control land in queues; both are drained from the
    Isaac main thread inside a post-update callback (`omni.kit.app`).
  * PhysX contact reports fire on the simulation thread; publishing from
    there is safe.
"""
import importlib
import json
import math
import os
import sys
import threading
from collections import deque

import omni.usd
import omni.kit.app
import carb

USD_PATH = "/home/andres/agv-sim/src/agv_isaac_sim/worlds/greenhouse_with_robot.usd"
ROBOT_PRIM = "/agv"
ARTICULATION_PRIM = "/agv/base_link"

omni.usd.get_context().open_stage(USD_PATH)
print(f"[AGV] Opened {USD_PATH}")


# ─── ROS imports (Isaac's bundled rclpy) ─────────────────────────────────

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from geometry_msgs.msg import PoseStamped, Twist
    from std_msgs.msg import String, Bool
    ROS_OK = True
except Exception as exc:
    print(f"[AGV][handler] rclpy not available: {exc}", file=sys.stderr)
    ROS_OK = False


# ─── Module-level handler state (shared across threads) ──────────────────

_teleport_q = deque()
_teleport_q_lock = threading.Lock()
_control_q = deque()
_control_q_lock = threading.Lock()
_gt_pose_cache = None  # 7-tuple (x,y,z,qx,qy,qz,qw); writes only from Kit main thread
_gt_pose_lock = threading.Lock()
_node = None

# Multi-tick settle — after a teleport, re-apply pose + zero velocities
# for N consecutive post-update ticks so PhysX fully commits the new
# state. Without this, the first 30–80 ms of contact resolution can
# drift the chassis several cm / rotate it tens of degrees before
# friction damps out. sim_api /reset polls GT until convergence, so a
# chassis that drifts > 5 cm during the settle window triggers
# RESET_TIMEOUT even though the requested pose was applied correctly.
_SETTLE_TICKS = 12              # ~60 ms @ 200 Hz sim step
_settle_ticks_remaining = 0
_settle_target = None           # (x, y, z, yaw)
_contact_sub = None
_post_update_sub = None
_last_collision_t = {}
_COLL_DEBOUNCE_S = 0.5
_AUTO_PLAY = os.environ.get('AGV_AUTO_PLAY', '1') != '0'

# Auto-unstick: when the brain is commanding non-trivial motion but the
# chassis isn't actually moving (PhysX friction lock, common after teleport
# or under heavy load), nudge the robot ±5cm to break the contact state.
_UNSTICK_CMD_THRESH = 0.1     # m/s — only consider stuck if commanded > this
_UNSTICK_GT_THRESH  = 0.01    # m/s — actual chassis speed below this counts
_UNSTICK_DURATION_S = 3.0     # how long the above must hold to trigger
_UNSTICK_NUDGE_M    = 0.05    # micro-teleport magnitude (body x)
_UNSTICK_COOLDOWN_S = 5.0     # don't unstick again within this window
_unstick_state = {'cmd_lin': 0.0, 'last_pose_t': None, 'last_pose_xy': None,
                  'stuck_since': None, 'last_unstick_t': 0.0,
                  'nudge_dir': 1,
                  'motor_enabled': False}  # latest /agv/motor_enable; guards nudging


def _yaw_from_quat(qx, qy, qz, qw):
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


def _basename(path):
    return path.rstrip('/').rsplit('/', 1)[-1] or path


# ─── ROS node ────────────────────────────────────────────────────────────

if ROS_OK:
    class IsaacHandlerNode(Node):
        def __init__(self):
            super().__init__('sim_isaac_handler')
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
            latched = QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)

            self.create_subscription(
                PoseStamped, '/agv/sim/reset_request', self._on_reset, qos)
            self.create_subscription(
                String, '/agv/sim/control', self._on_control, qos)
            # Watch /agv/cmd_vel for the auto-unstick detector. We only
            # need linear.x — the rest is irrelevant for friction-lock
            # detection.
            self.create_subscription(
                Twist, '/agv/cmd_vel', self._on_cmd_vel, qos)
            # Track motor_enable so auto-unstick can gate on it. Without
            # this the nudge fires on every cooldown tick whenever the
            # brain has EVER published a non-zero cmd_vel, even while the
            # motor is disarmed (e.g. during /reset). That was silently
            # translating the robot 5 cm every 5 s.
            #
            # Use VOLATILE QoS (the default `qos` profile) instead of the
            # latched one — sim_api and the brain's teleop_server both
            # publish motor_enable as VOLATILE, so a TRANSIENT_LOCAL sub
            # is incompatible (Cyclone DDS warning: "incompatible policy:
            # DURABILITY") and never receives any message. Without
            # receiving motor_enable=True the unstick stays gated off
            # forever — independent bug from the original silent-nudge.
            self.create_subscription(
                Bool, '/agv/motor_enable', self._on_motor_enable, qos)
            self.events_pub = self.create_publisher(
                String, '/agv/sim/events', 50)
            self.reset_done_pub = self.create_publisher(
                Bool, '/agv/sim/reset_done', latched)
            # Override the external ground_truth_publisher with PhysX truth.
            # The external one reads world->agv from OmniGraph, but /agv is a
            # static Xform marker — only /agv/base_link moves with physics.
            self.gt_pose_pub = self.create_publisher(
                PoseStamped, '/agv/sim/ground_truth/pose', qos)
            self.create_timer(0.1, self._publish_gt)
            self.get_logger().info(
                'sim_isaac_handler ready (reset + control + GT @ 10 Hz)')

        def _publish_gt(self):
            with _gt_pose_lock:
                cached = _gt_pose_cache
            if cached is None:
                return
            x, y, z, qx, qy, qz, qw = cached
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'world'
            msg.pose.position.x = float(x)
            msg.pose.position.y = float(y)
            msg.pose.position.z = float(z)
            msg.pose.orientation.x = float(qx)
            msg.pose.orientation.y = float(qy)
            msg.pose.orientation.z = float(qz)
            msg.pose.orientation.w = float(qw)
            self.gt_pose_pub.publish(msg)

        def _on_reset(self, msg: PoseStamped):
            p = msg.pose.position
            q = msg.pose.orientation
            yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)
            with _teleport_q_lock:
                _teleport_q.append((p.x, p.y, p.z, yaw))
            self.get_logger().info(
                f'reset_request queued: x={p.x:.2f} y={p.y:.2f} '
                f'z={p.z:.2f} yaw={yaw:.2f}')

        def _on_control(self, msg: String):
            cmd = msg.data.strip().lower()
            with _control_q_lock:
                _control_q.append(cmd)
            self.get_logger().info(f'control command queued: {cmd}')

        def _on_cmd_vel(self, msg: Twist):
            _unstick_state['cmd_lin'] = float(msg.linear.x)
            if msg.linear.x < 0:
                _unstick_state['nudge_dir'] = -1
            elif msg.linear.x > 0:
                _unstick_state['nudge_dir'] = 1

        def _on_motor_enable(self, msg: Bool):
            _unstick_state['motor_enabled'] = bool(msg.data)
            if not msg.data:
                # Disarming the motor invalidates the last cmd_vel — the
                # brain's desired linear velocity is effectively 0. Reset
                # the stuck-accumulator so a stale non-zero cmd_lin from
                # before the disarm doesn't keep the unstick armed.
                _unstick_state['cmd_lin'] = 0.0
                _unstick_state['stuck_since'] = None


# ─── Teleport (runs on Isaac main thread) ────────────────────────────────

def _read_base_link_pose_dc():
    """Return (x, y, z, qx, qy, qz, qw) of the articulation root via PhysX.

    MUST be called on the Isaac Kit main thread — dynamic_control isn't
    thread-safe and segfaults from the rclpy daemon thread. Use the cached
    value via `_gt_pose_cache` from other threads instead.
    """
    try:
        dc = importlib.import_module('omni.isaac.dynamic_control._dynamic_control')
    except Exception:
        return None
    dci = dc.acquire_dynamic_control_interface()
    art = dci.get_articulation(ARTICULATION_PRIM)
    if art == dc.INVALID_HANDLE:
        return None
    root = dci.get_articulation_root_body(art)
    if root == dc.INVALID_HANDLE:
        return None
    try:
        tf = dci.get_rigid_body_pose(root)
        return (tf.p.x, tf.p.y, tf.p.z,
                tf.r.x, tf.r.y, tf.r.z, tf.r.w)
    except Exception:
        return None


def _refresh_gt_cache():
    """Read base_link pose on the Isaac main thread and cache it for the
    rclpy daemon thread to publish."""
    global _gt_pose_cache
    pose = _read_base_link_pose_dc()
    if pose is None:
        return
    with _gt_pose_lock:
        _gt_pose_cache = pose


def _apply_teleport_dc(x, y, z, yaw, arm_settle=True):
    """Preferred path: omni.isaac.dynamic_control. Fast and physics-correct.

    Zeroes linear AND angular velocity on EVERY body in the articulation
    (chassis + 2 drive wheels + 2 casters), not just the root. With the
    high-friction binding (effective μ=15), wheels left spinning from a
    previous drive command would couple back into chassis translation
    via ground friction and produce a residual ~5–10 mm/s drift after
    the teleport — visible from the brain side as `/reset` "didn't put
    me where I asked". Per-body zeroing eliminates this.

    When arm_settle=True (the default), arms the multi-tick settle
    mechanism — the next _SETTLE_TICKS post-update callbacks re-apply
    the pose + zero velocities so PhysX commits the state across
    several substeps before physics runs free. When called from the
    settle loop itself, arm_settle=False to avoid re-arming.
    """
    global _settle_ticks_remaining, _settle_target
    try:
        from omni.isaac.dynamic_control import _dynamic_control as dc
    except Exception:
        return False
    dci = dc.acquire_dynamic_control_interface()
    art = dci.get_articulation(ARTICULATION_PRIM)
    if art == dc.INVALID_HANDLE:
        return False
    dci.wake_up_articulation(art)
    root = dci.get_articulation_root_body(art)
    if root == dc.INVALID_HANDLE:
        return False
    qx, qy, qz = 0.0, 0.0, math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    pose = dc.Transform([float(x), float(y), float(z)], [qx, qy, qz, qw])
    dci.set_rigid_body_pose(root, pose)

    # Zero velocities on every body (root + each link). Wheels are
    # joint-driven, so leaving them spinning translates the chassis via
    # contact friction even when the brain has disarmed the motor.
    zero = [0.0, 0.0, 0.0]
    try:
        body_count = dci.get_articulation_body_count(art)
    except Exception:
        body_count = 0
    if body_count > 0:
        for i in range(body_count):
            try:
                body = dci.get_articulation_body(art, i)
                if body and body != dc.INVALID_HANDLE:
                    dci.set_rigid_body_linear_velocity(body, zero)
                    dci.set_rigid_body_angular_velocity(body, zero)
            except Exception:
                pass
    else:
        # Fallback when body count isn't available — at minimum kill root.
        dci.set_rigid_body_linear_velocity(root, zero)
        dci.set_rigid_body_angular_velocity(root, zero)

    # Also zero joint velocities directly (covers DOF state in addition to
    # rigid-body state). On articulations these can be set in one call.
    try:
        dof_count = dci.get_articulation_dof_count(art)
        if dof_count > 0:
            dci.set_articulation_dof_velocities(art, [0.0] * dof_count)
    except Exception:
        pass

    if arm_settle:
        # Arm the multi-tick settle so the next _SETTLE_TICKS post-update
        # callbacks re-apply this exact pose and re-zero velocities.
        # This absorbs the PhysX contact-resolve transient (~30–80 ms
        # after a rigid-body pose change) where tangential friction
        # impulses otherwise drift the chassis several cm.
        _settle_ticks_remaining = _SETTLE_TICKS
        _settle_target = (x, y, z, yaw)

    return True


def _apply_teleport_usd(x, y, z, yaw):
    """Fallback: rewrite the USD xformOps on /agv. Loses physics state."""
    from pxr import UsdGeom, Gf
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return False
    prim = stage.GetPrimAtPath(ROBOT_PRIM)
    if not prim or not prim.IsValid():
        return False
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(float(x), float(y), float(z)))
    qx, qy, qz = 0.0, 0.0, math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    xform.AddOrientOp().Set(Gf.Quatf(qw, Gf.Vec3f(qx, qy, qz)))
    return True


def _drain_control():
    """Execute timeline commands (play/stop/pause) queued from ROS."""
    if not _control_q:
        return
    try:
        tl = importlib.import_module('omni.timeline').get_timeline_interface()
    except Exception as e:
        print(f'[AGV][handler] omni.timeline unavailable: {e}', file=sys.stderr)
        return
    while True:
        with _control_q_lock:
            if not _control_q:
                return
            cmd = _control_q.popleft()
        if cmd == 'play':
            tl.play()
            print('[AGV][handler] timeline → PLAY')
        elif cmd == 'stop':
            tl.stop()
            print('[AGV][handler] timeline → STOP')
        elif cmd == 'pause':
            tl.pause()
            print('[AGV][handler] timeline → PAUSE')
        else:
            print(f'[AGV][handler] unknown control command: {cmd}',
                  file=sys.stderr)


def _check_auto_unstick():
    """Detect friction-lock (cmd > threshold but chassis static for >Ns)
    and queue a micro-teleport nudge to break it. Runs on the Isaac main
    thread inside _drain_teleport so it has access to the cached GT pose.

    Gated on /agv/motor_enable — if the brain has the motor disarmed
    (e.g. mid /sim/reset) the chassis being "static under a non-zero
    commanded velocity" is the expected behavior, not a stuck condition.
    Without this gate the unstick nudges a stationary robot +5 cm every
    5 s during the /reset convergence window, making POST /reset fail
    to converge.
    """
    if not _unstick_state.get('motor_enabled', False):
        _unstick_state['stuck_since'] = None
        return
    import time as _t
    cached = _gt_pose_cache
    if cached is None:
        return
    now = _t.time()
    x, y = cached[0], cached[1]
    last_t = _unstick_state['last_pose_t']
    last_xy = _unstick_state['last_pose_xy']
    _unstick_state['last_pose_t'] = now
    _unstick_state['last_pose_xy'] = (x, y)
    if last_t is None or last_xy is None:
        return
    dt = now - last_t
    if dt <= 0.0 or dt > 1.0:
        return
    speed = math.hypot(x - last_xy[0], y - last_xy[1]) / dt
    cmd_lin = abs(_unstick_state['cmd_lin'])
    is_stuck_now = cmd_lin > _UNSTICK_CMD_THRESH and speed < _UNSTICK_GT_THRESH
    if not is_stuck_now:
        _unstick_state['stuck_since'] = None
        return
    if _unstick_state['stuck_since'] is None:
        _unstick_state['stuck_since'] = now
        return
    stuck_dur = now - _unstick_state['stuck_since']
    if stuck_dur < _UNSTICK_DURATION_S:
        return
    if now - _unstick_state['last_unstick_t'] < _UNSTICK_COOLDOWN_S:
        return
    # Nudge in commanded direction by ±5cm in body x. Keep yaw, z.
    yaw = _yaw_from_quat(cached[3], cached[4], cached[5], cached[6])
    sign = _unstick_state['nudge_dir']
    nx = x + sign * _UNSTICK_NUDGE_M * math.cos(yaw)
    ny = y + sign * _UNSTICK_NUDGE_M * math.sin(yaw)
    nz = cached[2]
    with _teleport_q_lock:
        _teleport_q.append((nx, ny, nz, yaw))
    _unstick_state['last_unstick_t'] = now
    _unstick_state['stuck_since'] = None
    if _node is not None:
        try:
            _node.events_pub.publish(String(data=json.dumps({
                't_sim': _node.get_clock().now().nanoseconds * 1e-9,
                'event': 'sim_unstick',
                'cmd_lin': cmd_lin,
                'measured_speed': speed,
                'stuck_duration_s': round(stuck_dur, 2),
                'nudge_m': sign * _UNSTICK_NUDGE_M,
            })))
        except Exception:
            pass


def _drain_teleport(_event):
    global _last_teleport_wall_t, _settle_ticks_remaining
    _drain_control()
    _refresh_gt_cache()
    _check_ejection_watchdog()
    _fix_friction()
    _check_auto_unstick()

    # Multi-tick settle pass — if a recent teleport armed this, re-apply
    # the same pose and re-zero velocities for a few ticks before letting
    # physics run free. This must happen BEFORE we pop the next queued
    # teleport so an incoming /reset can't cancel the settle of a prior
    # one. Running on the Kit main thread so dci calls are safe.
    if _settle_ticks_remaining > 0 and _settle_target is not None:
        sx, sy, sz, syaw = _settle_target
        _apply_teleport_dc(sx, sy, sz, syaw, arm_settle=False)
        _settle_ticks_remaining -= 1
        # Do NOT publish reset_done again — the original teleport handler
        # already did so. Refresh GT so the watchdog sees the settled
        # pose on its next pass.
        _refresh_gt_cache()

    if _node is None or not _teleport_q:
        return
    import time as _t
    while True:
        with _teleport_q_lock:
            if not _teleport_q:
                return
            x, y, z, yaw = _teleport_q.popleft()

        ok = _apply_teleport_dc(x, y, z, yaw)
        method = 'dynamic_control'
        if not ok:
            ok = _apply_teleport_usd(x, y, z, yaw)
            method = 'usd_xform'

        # Stamp the teleport time so the ejection watchdog gives the
        # physics solver a grace period to settle before checking z.
        _last_teleport_wall_t = _t.time()

        try:
            _node.reset_done_pub.publish(Bool(data=bool(ok)))
            _node.get_logger().info(
                f'teleport via {method}: ({x:.2f}, {y:.2f}, {z:.2f}) '
                f'yaw={yaw:.2f} ok={ok}')
        except Exception as e:
            print(f'[AGV][handler] reset_done publish failed: {e}',
                  file=sys.stderr)


# ─── Runtime friction fix (no USD regeneration required) ────────────────

# Idealistic high-grip values. Real rubber-on-soil tops out around μ=0.8;
# we use higher because the simulator was getting massive wheel slip
# (chassis at 5-10% of commanded speed) under the original μ=5/2 with
# combine='max' (effective contact μ=5). Switching combine to 'multiply'
# with both sides at μ=5 gave effective μ=25, which fixed the slip but
# created a new failure mode: the brain reproduced three chassis ejections
# under continuous Nav2 driving (round 33 GT.z→-200 m, round 34 pre-test
# GT.z→-235 m, round 34 post-test instant ejection on /sim/reset). The
# PhysX TGS solver becomes unstable at very high friction with the
# chassis CoM offset (+0.2 m) plus caster bouncing — contact-resolve
# impulses eject the rigid body.
#
# Stable middle ground: ground × wheel = 3 × 5 = 15. Still 3× the original
# 'max' value, more than enough grip (μ=15 × 75 N normal = 1125 N per
# wheel × 4 wheels / 30 kg = 150 m/s² available, vs 0.5 m/s² needed).
# The accompanying ejection watchdog (_drain_teleport) catches the rare
# residual case and auto-recovers.
#
# Note on PhysX combine priority: max > multiply > min > avg. If only one
# side is 'multiply' and the other 'max', 'max' wins. Both sides MUST be
# set to 'multiply' for the multiplicative combine to take effect.
HI_FRICTION_WHEEL_STATIC  = 5.0
HI_FRICTION_WHEEL_DYNAMIC = 4.5
HI_FRICTION_GROUND_STATIC  = 3.0   # 5.0 → 3.0 (effective static  μ = 5×3 = 15)
HI_FRICTION_GROUND_DYNAMIC = 2.7   # 4.5 → 2.7 (effective dynamic μ = 4.5×2.7 ≈ 12)
HI_FRICTION_COMBINE = 'multiply'   # priority above 'min'/'avg'; both sides set


# Ejection watchdog. The friction reduction above eliminates the bulk of
# the TGS solver instability, but residual edge cases can still throw
# the chassis through the floor or up into the air. The watchdog runs
# on the Kit main thread (inside _drain_teleport) and pushes a synthetic
# teleport to spawn whenever the cached GT pose looks impossible.
#
# Spawn (5.5, 0.0, 0.2) is the value baked into greenhouse_with_robot.usd
# by setup_omnigraph_standalone.py:89 (ROBOT_START_POS). Keep in sync.
WATCHDOG_SPAWN      = (5.5, 0.0, 0.2, 0.0)   # x, y, z, yaw
WATCHDOG_Z_MIN      = -0.5     # below this z = robot fell through floor
WATCHDOG_Z_DRIFT    =  1.0     # |z - spawn_z| above this = ejected upward
WATCHDOG_COOLDOWN_S =  2.0     # min seconds between auto-recoveries
WATCHDOG_POST_TELEPORT_GRACE_S = 1.5  # quiet the watchdog right after a teleport
_watchdog_last_fire_t = 0.0    # wall-clock of last auto-teleport
_last_teleport_wall_t = 0.0    # wall-clock of last applied teleport (grace period)


def _check_ejection_watchdog():
    """Detect impossible chassis poses and queue an auto-recovery teleport.

    Runs every Kit post-update on the main thread. Reads the GT cache
    populated by _refresh_gt_cache() and, if the z-height is below the
    floor or far above spawn, pushes a synthetic teleport to WATCHDOG_SPAWN
    onto _teleport_q (the same queue /sim/reset_request uses, so the
    rest of the pipeline is unchanged). Rate-limited to one fire per
    WATCHDOG_COOLDOWN_S to absorb the 1-2 ticks of propagation latency
    between dynamic_control.set_rigid_body_pose() and the next GT sample.
    """
    global _watchdog_last_fire_t

    with _gt_pose_lock:
        gt = _gt_pose_cache
    if gt is None:
        return
    z = gt[2]
    spawn_z = WATCHDOG_SPAWN[2]
    if not (z < WATCHDOG_Z_MIN or abs(z - spawn_z) > WATCHDOG_Z_DRIFT):
        return

    import time
    now = time.time()
    # Post-teleport grace: a fresh set_rigid_body_pose can briefly show
    # transient solver states (z spikes while casters resolve contact)
    # that the watchdog would interpret as ejection. Skip the check for
    # WATCHDOG_POST_TELEPORT_GRACE_S after the most recent teleport so
    # the legit user-requested pose has time to settle.
    if (now - _last_teleport_wall_t) < WATCHDOG_POST_TELEPORT_GRACE_S:
        return
    if (now - _watchdog_last_fire_t) <= WATCHDOG_COOLDOWN_S:
        return
    _watchdog_last_fire_t = now

    sx, sy, sz, syaw = WATCHDOG_SPAWN
    sys.stderr.write(
        f'[AGV][watchdog] ejection detected '
        f'(GT x={gt[0]:.1f} y={gt[1]:.1f} z={z:.2f}) — '
        f'auto-teleporting to spawn ({sx},{sy},{sz})\n')
    sys.stderr.flush()

    with _teleport_q_lock:
        _teleport_q.append((sx, sy, sz, syaw))

    # Best-effort event for sim_api / brain. Falls back silently — the
    # watchdog must never propagate an exception into the post-update.
    if _node is not None:
        try:
            _node.events_pub.publish(String(data=json.dumps({
                't_sim': _node.get_clock().now().nanoseconds * 1e-9,
                'event': 'watchdog_recovery',
                'gt_x': gt[0], 'gt_y': gt[1], 'gt_z': z,
                'spawn': list(WATCHDOG_SPAWN),
            })))
        except Exception:
            pass


def _fix_friction():
    """Create high-friction physics materials and bind them to wheels +
    ground. Runs once on the Isaac main thread before Play."""
    if getattr(_fix_friction, 'done', False):
        return
    sys.stderr.write('[AGV][friction] starting binding...\n')
    sys.stderr.flush()
    try:
        from pxr import UsdPhysics, UsdShade
    except Exception as e:
        sys.stderr.write(f'[AGV][friction] pxr import failed: {e}\n')
        sys.stderr.flush()
        _fix_friction.done = True
        return
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        sys.stderr.write('[AGV][friction] no stage yet, will retry\n')
        sys.stderr.flush()
        return

    def _make_mat(path, mu_s, mu_d, combine='max'):
        from pxr import PhysxSchema
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            mat = UsdShade.Material.Define(stage, path)
            prim = mat.GetPrim()
        api = UsdPhysics.MaterialAPI.Apply(prim)
        api.CreateStaticFrictionAttr(float(mu_s))
        api.CreateDynamicFrictionAttr(float(mu_d))
        api.CreateRestitutionAttr(0.0)
        try:
            px_api = PhysxSchema.PhysxMaterialAPI.Apply(prim)
            px_api.CreateFrictionCombineModeAttr(combine)
            px_api.CreateRestitutionCombineModeAttr('avg')
        except Exception:
            pass
        return prim

    try:
        wheel_mat = _make_mat('/agv/Materials/HiFrictionWheel',
                              HI_FRICTION_WHEEL_STATIC, HI_FRICTION_WHEEL_DYNAMIC,
                              combine=HI_FRICTION_COMBINE)
        ground_mat = _make_mat('/World/Materials/HiFrictionGround',
                               HI_FRICTION_GROUND_STATIC, HI_FRICTION_GROUND_DYNAMIC,
                               combine=HI_FRICTION_COMBINE)
        # Low-friction "obstacle" material for crop_rows / walls / crates.
        # Without this binding, side contact between a wheel and one of
        # those geometries used the PhysX default friction (~0.5), which
        # — combined with the wheel's μ=5 via combine='max' — produced
        # huge tangential force spikes on glancing contact. That spike
        # showed up as IMU jolts and triggered EKF divergence in tight
        # aisles (wp10 at y=4.4 was the canonical victim). Binding μ=0.3
        # with combine='min' caps the contact at the obstacle side, so a
        # wheel scrape just slides past instead of locking.
        obstacle_mat = _make_mat('/World/Materials/LoFrictionObstacle',
                                 0.3, 0.25, 'min')
    except Exception as e:
        sys.stderr.write(f'[AGV][friction] material creation failed: {e}\n')
        sys.stderr.flush()
        _fix_friction.done = True
        return

    def _bind(prim, mat_prim):
        try:
            binding = UsdShade.MaterialBindingAPI.Apply(prim)
            binding.Bind(UsdShade.Material(mat_prim),
                         UsdShade.Tokens.weakerThanDescendants,
                         'physics')
            return True
        except Exception:
            return False

    # Caster gets a SEPARATE low-friction material with combine='min'.
    # Caster-vs-ground = min(0.05, ground μ) = 0.05 → caster slides freely
    # on the ground without dragging the chassis. This makes caster collision
    # safe to keep ENABLED — earlier versions disabled caster collision as a
    # workaround when the bindings weren't taking effect, but that left the
    # 30 kg chassis (CoM at +0.2 m forward of the wheel axis) supported only
    # by the two drive wheels, which made it tip forward and look "fallen"
    # in the viewport. Casters enabled = chassis sits flat on 4 contacts.
    caster_mat = _make_mat('/agv/Materials/HiFrictionCaster', 0.05, 0.04, 'min')

    wheel_targets = []
    caster_targets = []
    ground_targets = []
    obstacle_targets = []
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if path.startswith(ROBOT_PRIM):
            lower = path.lower()
            if '/joints/' in lower:
                continue
            if 'caster' in lower and ('caster/' in lower or 'caster_link' in lower):
                caster_targets.append((prim, path))
            elif 'wheel' in lower and ('wheel/' in lower or 'wheel_link' in lower):
                wheel_targets.append((prim, path))
            elif prim.HasAPI(UsdPhysics.CollisionAPI):
                if 'caster' in lower:
                    caster_targets.append((prim, path))
                elif 'wheel' in lower:
                    wheel_targets.append((prim, path))
        elif 'ground' in path.lower() or path == '/World/Plane':
            ground_targets.append((prim, path))
        elif (path.startswith('/World/CropRows/') or
              path.startswith('/World/Walls/') or
              path.startswith('/World/Obstacles/') or
              path.startswith('/World/Props/')):
            # Bind only to prims that actually have collision geometry —
            # the parent xforms don't matter for PhysX contact pairs.
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                obstacle_targets.append((prim, path))

    bound_wheels = sum(1 for prim, _ in wheel_targets if _bind(prim, wheel_mat))
    bound_casters = sum(1 for prim, _ in caster_targets if _bind(prim, caster_mat))
    bound_ground = sum(1 for prim, _ in ground_targets if _bind(prim, ground_mat))
    bound_obstacles = sum(1 for prim, _ in obstacle_targets if _bind(prim, obstacle_mat))

    _fix_friction.done = True
    sys.stderr.write(
        f'[AGV][friction] wheels(μ={HI_FRICTION_WHEEL_STATIC}/'
        f'{HI_FRICTION_WHEEL_DYNAMIC},{HI_FRICTION_COMBINE}) '
        f'ground(μ={HI_FRICTION_GROUND_STATIC}/'
        f'{HI_FRICTION_GROUND_DYNAMIC},{HI_FRICTION_COMBINE}) '
        f'effective_contact_μ='
        f'{HI_FRICTION_WHEEL_STATIC * HI_FRICTION_GROUND_STATIC:.1f}/'
        f'{HI_FRICTION_WHEEL_DYNAMIC * HI_FRICTION_GROUND_DYNAMIC:.1f} '
        f'casters(μ=0.05/0.04,min) obstacles(μ=0.3/0.25,min): bound to '
        f'{bound_wheels}/{len(wheel_targets)} wheel + '
        f'{bound_casters}/{len(caster_targets)} caster + '
        f'{bound_ground}/{len(ground_targets)} ground + '
        f'{bound_obstacles}/{len(obstacle_targets)} obstacle\n')
    if wheel_targets:
        sys.stderr.write(f'[AGV][friction] wheel paths: '
                         f'{", ".join(p for _, p in wheel_targets[:4])}\n')
    if caster_targets:
        sys.stderr.write(f'[AGV][friction] caster paths: '
                         f'{", ".join(p for _, p in caster_targets[:4])}\n')
    sys.stderr.flush()


# ─── PhysX contact reports → /agv/sim/events ────────────────────────────

def _decode_actor(actor_int):
    """Convert PhysX actor int handle to its SdfPath string."""
    try:
        from pxr import PhysicsSchemaTools
        return str(PhysicsSchemaTools.intToSdfPath(actor_int))
    except Exception:
        return f'<unknown:{actor_int}>'


def _on_contact_report(contact_headers, contact_data):
    if _node is None:
        return
    import time
    now_t = time.time()
    for h in contact_headers:
        try:
            a0 = _decode_actor(h.actor0)
            a1 = _decode_actor(h.actor1)
        except Exception:
            continue

        a0_is_robot = a0.startswith(ROBOT_PRIM)
        a1_is_robot = a1.startswith(ROBOT_PRIM)
        if not (a0_is_robot or a1_is_robot):
            continue
        if a0_is_robot and a1_is_robot:
            continue  # robot self-contact (wheel against base) — ignore

        robot_link = a0 if a0_is_robot else a1
        other = a1 if a0_is_robot else a0

        key = tuple(sorted([robot_link, other]))
        last = _last_collision_t.get(key, 0.0)
        if now_t - last < _COLL_DEBOUNCE_S:
            continue
        _last_collision_t[key] = now_t

        impulse = 0.0
        try:
            n = h.num_contact_data
            start = h.contact_data_offset
            for i in range(n):
                imp = contact_data[start + i].impulse
                impulse += math.sqrt(imp[0] ** 2 + imp[1] ** 2 + imp[2] ** 2)
        except Exception:
            pass

        payload = {
            't_sim': _node.get_clock().now().nanoseconds * 1e-9,
            'event': 'collision',
            'with':       _basename(other),
            'with_path':  other,
            'robot_link': _basename(robot_link),
            'impulse_n_s': round(impulse, 4),
        }
        try:
            _node.events_pub.publish(String(data=json.dumps(payload)))
            _node.get_logger().info(
                f"collision: {payload['robot_link']} <-> {payload['with']} "
                f"(J={payload['impulse_n_s']})")
        except Exception as e:
            print(f'[AGV][handler] event publish failed: {e}', file=sys.stderr)


def _enable_contact_reports_on_robot_and_obstacles():
    """Apply PhysxContactReportAPI to all rigid bodies under /agv and to
    static obstacles so that contacts actually fire events.

    URDF imports don't enable contact reports by default. We enable them
    here at runtime so the user doesn't have to regenerate the USD.
    """
    try:
        from pxr import UsdPhysics, PhysxSchema
    except Exception as e:
        print(f'[AGV][handler] pxr import for contact API failed: {e}',
              file=sys.stderr)
        return 0

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return 0

    targets_prefixes = (ROBOT_PRIM, '/World/Crate', '/World/wall',
                        '/World/Wall', '/Crate', '/wall', '/Wall',
                        '/World/pallet', '/World/small_klt')
    count = 0
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if not any(path.startswith(p) for p in targets_prefixes):
            continue
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI) and \
           not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        if not prim.HasAPI(PhysxSchema.PhysxContactReportAPI):
            PhysxSchema.PhysxContactReportAPI.Apply(prim)
        api = PhysxSchema.PhysxContactReportAPI(prim)
        try:
            api.GetThresholdAttr().Set(0.0)
        except Exception:
            pass
        count += 1
    return count


# ─── Boot the handler one Kit frame after the stage is open ──────────────

def _boot_handler():
    global _node, _contact_sub, _post_update_sub
    if not ROS_OK:
        print('[AGV][handler] ROS unavailable — handler NOT booted',
              file=sys.stderr)
        return
    if _node is not None:
        return  # already booted

    if not rclpy.ok():
        rclpy.init()
    _node = IsaacHandlerNode()

    spin_thread = threading.Thread(
        target=rclpy.spin, args=(_node,), daemon=True,
        name='sim_isaac_handler_spin')
    spin_thread.start()

    app = omni.kit.app.get_app()
    _post_update_sub = app.get_post_update_event_stream(
    ).create_subscription_to_pop(
        _drain_teleport, name='agv_handler_post_update')

    n_armed = _enable_contact_reports_on_robot_and_obstacles()
    print(f'[AGV][handler] PhysxContactReportAPI applied to {n_armed} prims')

    try:
        physx_mod = importlib.import_module('omni.physx')
        pxi = physx_mod.get_physx_simulation_interface()
        _contact_sub = pxi.subscribe_to_contact_report_events(
            _on_contact_report)
        print('[AGV][handler] subscribed to PhysX contact reports')
    except Exception as e:
        print(f'[AGV][handler] PhysX contact subscription FAILED: {e}',
              file=sys.stderr)


# Defer one Kit update so all extensions are fully initialised
def _delayed_boot(_e):
    if getattr(_delayed_boot, 'done', False):
        return
    _delayed_boot.done = True
    try:
        _boot_handler()
    except Exception as e:
        print(f'[AGV][handler] boot failed: {e}', file=sys.stderr)
        import traceback
        traceback.print_exc()


_boot_sub = omni.kit.app.get_app().get_update_event_stream(
).create_subscription_to_pop(_delayed_boot, name='agv_handler_boot')
print('[AGV][handler] boot scheduled')


# ─── Auto-play: start the timeline once the stage is fully loaded ────────

if _AUTO_PLAY:
    _auto_play_frames = [0]

    def _auto_play(_e):
        if getattr(_auto_play, 'done', False):
            return
        _auto_play_frames[0] += 1
        # Wait ~60 frames (~1 s at 60 fps) for stage + extensions to settle
        if _auto_play_frames[0] < 60:
            return
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return
        prim = stage.GetPrimAtPath(ROBOT_PRIM)
        if not prim or not prim.IsValid():
            return
        _auto_play.done = True
        try:
            tl = importlib.import_module('omni.timeline').get_timeline_interface()
            tl.play()
            print('[AGV][auto-play] simulation started automatically')
        except Exception as e:
            print(f'[AGV][auto-play] failed: {e}', file=sys.stderr)

    _auto_play_sub = omni.kit.app.get_app().get_update_event_stream(
    ).create_subscription_to_pop(_auto_play, name='agv_auto_play')
    print('[AGV][auto-play] scheduled (disable with AGV_AUTO_PLAY=0)')
else:
    print('[AGV][auto-play] DISABLED (AGV_AUTO_PLAY=0)')
