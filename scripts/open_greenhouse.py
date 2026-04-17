"""Open the greenhouse USD AND run the in-sim validation handler.

Two responsibilities, both running inside the Isaac Sim Kit process:

  1. Open `greenhouse_with_robot.usd` (the original behaviour of this script).
  2. Boot a small ROS 2 node + PhysX hooks that:
       * Subscribe to /agv/sim/reset_request (PoseStamped) and apply the
         teleport on the Isaac main thread (USD writes are not thread-safe).
       * Subscribe to PhysX contact-report events and republish them as
         JSON `collision` events on /agv/sim/events.
       * Publish /agv/sim/reset_done (Bool, latched) once the teleport has
         been applied.

The handler is what closes Phase 4 deferred items 1 and 2 of the
`agv_sim_validation` overlay. Items 3 and 4 (visible_markers, obstacles)
are pure ROS nodes shipped from the validation package — they don't need
Isaac internals.

Threading model:
  * rclpy `spin()` runs in a daemon thread so callbacks fire continuously.
  * Reset requests land in a queue; the queue is drained from the Isaac
    main thread inside a post-update callback (`omni.kit.app`).
  * PhysX contact reports already fire on the simulation thread, but
    `rclpy.publisher.publish` is safe to call from there.
"""
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
_node = None
_contact_sub = None
_post_update_sub = None
_last_collision_t = {}
_COLL_DEBOUNCE_S = 0.5


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
            self.events_pub = self.create_publisher(
                String, '/agv/sim/events', 50)
            self.reset_done_pub = self.create_publisher(
                Bool, '/agv/sim/reset_done', latched)
            self.get_logger().info(
                'sim_isaac_handler ready (reset_request + PhysX collisions)')

        def _on_reset(self, msg: PoseStamped):
            p = msg.pose.position
            q = msg.pose.orientation
            yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)
            with _teleport_q_lock:
                _teleport_q.append((p.x, p.y, p.z, yaw))
            self.get_logger().info(
                f'reset_request queued: x={p.x:.2f} y={p.y:.2f} '
                f'z={p.z:.2f} yaw={yaw:.2f}')


# ─── Teleport (runs on Isaac main thread) ────────────────────────────────

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


def _drain_teleport(_event):
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
        import omni.physx
        pxi = omni.physx.get_physx_simulation_interface()
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
