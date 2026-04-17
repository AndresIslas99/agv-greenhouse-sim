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
    from geometry_msgs.msg import PoseStamped
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
_contact_sub = None
_post_update_sub = None
_last_collision_t = {}
_COLL_DEBOUNCE_S = 0.5
_AUTO_PLAY = os.environ.get('AGV_AUTO_PLAY', '1') != '0'


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


def _apply_teleport_dc(x, y, z, yaw):
    """Preferred path: omni.isaac.dynamic_control. Fast and physics-correct."""
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
    dci.set_rigid_body_linear_velocity(root, [0.0, 0.0, 0.0])
    dci.set_rigid_body_angular_velocity(root, [0.0, 0.0, 0.0])
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


def _drain_teleport(_event):
    _drain_control()
    _refresh_gt_cache()
    _fix_friction()
    if _node is None or not _teleport_q:
        return
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
# we go to 2.0 because the simulator was getting massive wheel slip
# (chassis at 15% of commanded speed) due to physics materials being
# CREATED by import_robot_usd.py but never BOUND to the wheel collision
# geometry. Until the USD is regenerated with proper MaterialBindingAPI
# bindings, we bind them here at runtime.
HI_FRICTION_WHEEL_STATIC  = 5.0
HI_FRICTION_WHEEL_DYNAMIC = 4.5
HI_FRICTION_GROUND_STATIC  = 2.0
HI_FRICTION_GROUND_DYNAMIC = 1.8


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
                              HI_FRICTION_WHEEL_STATIC, HI_FRICTION_WHEEL_DYNAMIC)
        ground_mat = _make_mat('/World/Materials/HiFrictionGround',
                               HI_FRICTION_GROUND_STATIC, HI_FRICTION_GROUND_DYNAMIC)
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
    caster_mat = _make_mat('/agv/Materials/HiFrictionCaster', 0.05, 0.04, 'min')

    # ALSO disable caster collision entirely. This was the only way to stop
    # them from pinning the chassis even with low μ + min combine — with
    # 30 kg chassis weight distributed across 2 wheels + 2 casters, the
    # casters carry significant normal force and even μ=0.05 produces enough
    # drag to slow the wheel-driven chassis to ~5% efficiency. The robot
    # will tilt forward/backward briefly under accel/decel but stays on
    # its 2 drive wheels in steady state. Real ODrive AGV has rolling
    # casters — this is a sim-side approximation.
    try:
        for caster_name in ('front_caster', 'rear_caster'):
            for sub in ('collisions',):
                p = stage.GetPrimAtPath(f'/agv/{caster_name}/{sub}')
                if p and p.IsValid():
                    UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Set(False)
    except Exception as e:
        sys.stderr.write(f'[AGV][friction] caster disable warning: {e}\n')

    wheel_targets = []
    caster_targets = []
    ground_targets = []
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

    bound_wheels = sum(1 for prim, _ in wheel_targets if _bind(prim, wheel_mat))
    bound_casters = sum(1 for prim, _ in caster_targets if _bind(prim, caster_mat))
    bound_ground = sum(1 for prim, _ in ground_targets if _bind(prim, ground_mat))

    _fix_friction.done = True
    sys.stderr.write(
        f'[AGV][friction] wheels(μ={HI_FRICTION_WHEEL_STATIC}/'
        f'{HI_FRICTION_WHEEL_DYNAMIC},max) ground(μ={HI_FRICTION_GROUND_STATIC}/'
        f'{HI_FRICTION_GROUND_DYNAMIC},max) casters(μ=0.05/0.04,avg): bound to '
        f'{bound_wheels}/{len(wheel_targets)} wheel + '
        f'{bound_casters}/{len(caster_targets)} caster + '
        f'{bound_ground}/{len(ground_targets)} ground\n')
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
