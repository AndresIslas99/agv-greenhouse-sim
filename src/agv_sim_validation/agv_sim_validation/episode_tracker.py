#!/usr/bin/env python3
"""Track Nav2 missions, accumulate metrics per episode, publish summaries.

Subscribes:
  /agv/sim/ground_truth/pose      PoseStamped     (oracle path)
  /agv/sim/localization_error     std_msgs/String (drift over time)
  /agv/sim/events                 std_msgs/String (collisions, marker_seen, etc.)
  /agv/navigate_to_pose/_action/* nav2_msgs       (goal lifecycle)

Publishes:
  /agv/sim/episode_summary        std_msgs/String JSON, latched, one per episode

Per-episode metrics:
  duration_s, path_length_m, optimal_path_m, efficiency,
  collisions, mean_localization_error_m, max_localization_error_m,
  result (succeeded|aborted|canceled|timeout), bag_path (Phase 4)
"""
import json
import math
import os
import shutil
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from action_msgs.msg import GoalStatusArray


# Topics included in every per-episode bag. Image topics are excluded by
# default because they're megabytes per second and rarely needed for analysis.
BASE_BAG_TOPICS = [
    '/clock',
    '/tf', '/tf_static',
    '/agv/cmd_vel', '/agv/cmd_vel_armed', '/agv/shaped_cmd_vel',
    '/agv/wheel_odom', '/agv/joint_states',
    '/agv/imu/data', '/agv/imu/data_clean',
    '/agv/odometry/local', '/agv/odometry/global',
    '/agv/scan',
    '/visual_slam/tracking/odometry',
    '/agv/motor_state', '/agv/drive_debug', '/agv/motor_enable', '/agv/e_stop',
    '/agv/sim/ground_truth/pose', '/agv/sim/ground_truth/twist',
    '/agv/sim/localization_error', '/agv/sim/events', '/agv/sim/episode_summary',
    '/agv/navigate_to_pose/_action/status',
]

IMAGE_BAG_TOPICS = [
    '/agv/zed/left/image_rect_color', '/agv/zed/left/camera_info',
    '/agv/zed/depth/depth_registered',
    '/agv/zed/point_cloud/cloud_registered',
]


class EpisodeTracker(Node):
    def __init__(self):
        super().__init__('episode_tracker')

        self.declare_parameter('success_position_tol_m', 0.30)
        self.declare_parameter('success_yaw_tol_rad', 0.30)
        self.declare_parameter('bag_dir', '/tmp/agv_runs')
        self.declare_parameter('record_images', False)
        self.declare_parameter('record_bags', True)

        self._bag_dir = self.get_parameter('bag_dir').value
        self._record_images = self.get_parameter('record_images').value
        self._record_bags = self.get_parameter('record_bags').value

        if self._record_bags:
            os.makedirs(self._bag_dir, exist_ok=True)

        # Episode state
        self.active = False
        self.episode_id = 0
        self.start_t = None
        self.start_pose = None
        self.last_pose = None
        self.path_length = 0.0
        self.collisions = 0
        self.errs = []  # accumulated pos errors
        self.events = []  # raw events JSON list
        self.goal = None
        self.result = None
        self._bag_proc = None
        self._bag_path = None

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(PoseStamped, '/agv/sim/ground_truth/pose',
                                 self._on_pose, qos)
        self.create_subscription(String, '/agv/sim/localization_error',
                                 self._on_loc_err, qos)
        self.create_subscription(String, '/agv/sim/events', self._on_event, qos)

        # Nav2 action status (we don't act on it, just observe)
        self.create_subscription(GoalStatusArray,
                                 '/agv/navigate_to_pose/_action/status',
                                 self._on_goal_status, qos)

        self.summary_pub = self.create_publisher(String,
                                                 '/agv/sim/episode_summary', latched)
        self.get_logger().info('Episode tracker up')

    def _on_pose(self, msg: PoseStamped):
        x, y = msg.pose.position.x, msg.pose.position.y
        if self.active and self.last_pose is not None:
            dx = x - self.last_pose[0]
            dy = y - self.last_pose[1]
            self.path_length += math.hypot(dx, dy)
        self.last_pose = (x, y)

    def _on_loc_err(self, msg: String):
        if not self.active:
            return
        try:
            d = json.loads(msg.data)
            self.errs.append(d.get('pos_err_m', 0.0))
        except Exception:
            pass

    def _on_event(self, msg: String):
        try:
            d = json.loads(msg.data)
        except Exception:
            return
        if not self.active:
            return
        self.events.append(d)
        if d.get('event') == 'collision':
            self.collisions += 1

    def _on_goal_status(self, msg: GoalStatusArray):
        # Status codes: 1=accepted, 2=executing, 3=canceling, 4=succeeded,
        # 5=canceled, 6=aborted
        if not msg.status_list:
            return
        latest = msg.status_list[-1]
        st = latest.status
        if st in (1, 2) and not self.active:
            self._begin_episode()
        elif st in (4, 5, 6) and self.active:
            result_map = {4: 'succeeded', 5: 'canceled', 6: 'aborted'}
            self._end_episode(result_map.get(st, 'unknown'))

    def _begin_episode(self):
        self.active = True
        self.episode_id += 1
        self.start_t = time.time()
        self.start_pose = self.last_pose
        self.path_length = 0.0
        self.collisions = 0
        self.errs = []
        self.events = []
        self._start_bag()
        self.get_logger().info(f'episode {self.episode_id} started')

    def _start_bag(self):
        if not self._record_bags or not shutil.which('ros2'):
            self._bag_proc = None
            self._bag_path = None
            return
        ts = int(self.start_t)
        bag_name = f'episode_{self.episode_id:04d}_{ts}'
        self._bag_path = os.path.join(self._bag_dir, bag_name)
        topics = list(BASE_BAG_TOPICS)
        if self._record_images:
            topics += IMAGE_BAG_TOPICS
        cmd = ['ros2', 'bag', 'record', '-o', self._bag_path] + topics
        try:
            self._bag_proc = subprocess.Popen(
                cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid)
            self.get_logger().info(f'recording bag → {self._bag_path}')
        except Exception as e:
            self.get_logger().warning(f'bag start failed: {e}')
            self._bag_proc = None
            self._bag_path = None

    def _stop_bag(self):
        if self._bag_proc is None:
            return
        try:
            os.killpg(os.getpgid(self._bag_proc.pid), signal.SIGINT)
            self._bag_proc.wait(timeout=5.0)
        except Exception as e:
            self.get_logger().warning(f'bag stop failed: {e}')
            try:
                self._bag_proc.kill()
            except Exception:
                pass
        self._bag_proc = None

    def _end_episode(self, result: str):
        if not self.active:
            return
        self.active = False
        duration = time.time() - self.start_t
        self._stop_bag()

        if self.start_pose and self.last_pose:
            optimal = math.hypot(self.last_pose[0] - self.start_pose[0],
                                 self.last_pose[1] - self.start_pose[1])
        else:
            optimal = 0.0
        eff = (optimal / self.path_length) if self.path_length > 1e-3 else 0.0
        mean_err = sum(self.errs) / len(self.errs) if self.errs else 0.0
        max_err = max(self.errs) if self.errs else 0.0

        summary = {
            'episode_id':                   self.episode_id,
            'start_ts':                     self.start_t,
            'duration_s':                   round(duration, 2),
            'result':                       result,
            'path_length_m':                round(self.path_length, 3),
            'optimal_path_m':               round(optimal, 3),
            'efficiency':                   round(eff, 3),
            'collisions':                   self.collisions,
            'mean_localization_error_m':    round(mean_err, 4),
            'max_localization_error_m':     round(max_err, 4),
            'event_count':                  len(self.events),
            'last_pose':                    list(self.last_pose) if self.last_pose else None,
            'bag_path':                     self._bag_path,
        }
        self.summary_pub.publish(String(data=json.dumps(summary)))
        self.get_logger().info(f'episode {self.episode_id} done: {result} '
                               f'len={self.path_length:.2f}m '
                               f'eff={eff:.2f} '
                               f'collisions={self.collisions}')


def main():
    rclpy.init()
    node = EpisodeTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop any in-flight bag recording cleanly
        if node.active:
            node._stop_bag()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
