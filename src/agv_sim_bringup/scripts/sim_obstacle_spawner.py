#!/usr/bin/env python3
"""Dynamic obstacle spawner for testing Nav2 obstacle avoidance.

Spawns box obstacles in the Gz Sim greenhouse world and moves them with
simple bounce dynamics. Useful for testing the robot's reactive navigation.

Uses ros_gz_interfaces services (Gz Harmonic / Gz Sim):
  - /world/<world_name>/create       (ros_gz_interfaces/srv/SpawnEntity)
  - /world/<world_name>/set_pose     (ros_gz_interfaces/srv/SetEntityPose)

Parameters:
  world_name:     Gz world name (default 'greenhouse_simple')
  num_obstacles:  number of obstacles to spawn (default 3)
  speed:          obstacle speed in m/s (default 0.3)
  bounds_x:       [min, max] X bounds in meters (default [5.0, 25.0])
  bounds_y:       [min, max] Y bounds in meters (default [-6.0, 6.0])
  obstacle_size:  [x, y, z] box dimensions in meters (default [0.4, 0.4, 0.5])
  update_rate:    position update rate in Hz (default 10.0)
"""

import math
import random

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion

from ros_gz_interfaces.srv import SpawnEntity, SetEntityPose
from ros_gz_interfaces.msg import Entity


# SDF template for a simple box obstacle
BOX_SDF_TEMPLATE = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{name}">
    <static>false</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <box><size>{sx} {sy} {sz}</size></box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box><size>{sx} {sy} {sz}</size></box>
        </geometry>
        <material>
          <ambient>0.8 0.2 0.2 1</ambient>
          <diffuse>0.8 0.2 0.2 1</diffuse>
        </material>
      </visual>
      <inertial>
        <mass>5.0</mass>
      </inertial>
    </link>
  </model>
</sdf>"""


class ObstacleSpawner(Node):
    def __init__(self):
        super().__init__('sim_obstacle_spawner')

        # Parameters
        self.declare_parameter('world_name', 'greenhouse_simple')
        self.declare_parameter('num_obstacles', 3)
        self.declare_parameter('speed', 0.3)
        self.declare_parameter('bounds_x', [5.0, 25.0])
        self.declare_parameter('bounds_y', [-6.0, 6.0])
        self.declare_parameter('obstacle_size', [0.4, 0.4, 0.5])
        self.declare_parameter('update_rate', 10.0)

        self.world_name = self.get_parameter('world_name').value
        self.num_obstacles = self.get_parameter('num_obstacles').value
        self.speed = self.get_parameter('speed').value
        self.bounds_x = self.get_parameter('bounds_x').value
        self.bounds_y = self.get_parameter('bounds_y').value
        self.obs_size = self.get_parameter('obstacle_size').value
        update_rate = self.get_parameter('update_rate').value

        # Gz Sim service endpoints (ros_gz_interfaces)
        spawn_topic = f'/world/{self.world_name}/create'
        set_pose_topic = f'/world/{self.world_name}/set_pose'

        self.spawn_client = self.create_client(SpawnEntity, spawn_topic)
        self.set_pose_client = self.create_client(SetEntityPose, set_pose_topic)

        # Obstacle state: position + velocity direction
        # States: 'pending' -> 'spawning' -> 'spawned' (or back to 'pending' on failure)
        self.obstacles = []
        for i in range(self.num_obstacles):
            angle = random.uniform(0, 2 * math.pi)
            self.obstacles.append({
                'name': f'dyn_obstacle_{i}',
                'x': random.uniform(self.bounds_x[0], self.bounds_x[1]),
                'y': random.uniform(self.bounds_y[0], self.bounds_y[1]),
                'vx': math.cos(angle) * self.speed,
                'vy': math.sin(angle) * self.speed,
                'state': 'pending',
            })

        self.get_logger().info(
            f'Spawning {self.num_obstacles} dynamic obstacles '
            f'in world "{self.world_name}" (speed={self.speed} m/s)')
        self.get_logger().info(f'Spawn service: {spawn_topic}')

        # Timer for spawning (retries pending obstacles every 2s)
        self.create_timer(2.0, self._try_spawn)
        # Timer for movement
        self.dt = 1.0 / update_rate
        self.create_timer(self.dt, self._update_positions)

    def _try_spawn(self):
        """Attempt to spawn any pending obstacles."""
        if not self.spawn_client.service_is_ready():
            self.get_logger().info(
                f'Waiting for spawn service on /world/{self.world_name}/create...',
                throttle_duration_sec=5.0)
            return

        for obs in self.obstacles:
            if obs['state'] != 'pending':
                continue

            sdf = BOX_SDF_TEMPLATE.format(
                name=obs['name'],
                sx=self.obs_size[0],
                sy=self.obs_size[1],
                sz=self.obs_size[2])

            req = SpawnEntity.Request()
            req.entity_factory.name = obs['name']
            req.entity_factory.sdf = sdf
            req.entity_factory.pose = Pose(
                position=Point(
                    x=obs['x'], y=obs['y'], z=self.obs_size[2] / 2.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))

            # Mark as spawning (in-flight) to avoid duplicate requests
            obs['state'] = 'spawning'
            future = self.spawn_client.call_async(req)
            future.add_done_callback(
                lambda f, o=obs: self._on_spawn(f, o))

    def _on_spawn(self, future, obs):
        try:
            result = future.result()
            if result is not None and result.success:
                obs['state'] = 'spawned'
                self.get_logger().info(f'Spawned {obs["name"]}')
            else:
                # Revert to pending so _try_spawn can retry
                obs['state'] = 'pending'
                msg = result.status_message if result else 'no response'
                self.get_logger().warn(
                    f'Failed to spawn {obs["name"]}: {msg} (will retry)')
        except Exception as e:
            obs['state'] = 'pending'
            self.get_logger().error(
                f'Spawn service call failed for {obs["name"]}: {e} (will retry)')

    def _update_positions(self):
        """Move spawned obstacles with bounce dynamics."""
        if not self.set_pose_client.service_is_ready():
            return

        for obs in self.obstacles:
            if obs['state'] != 'spawned':
                continue

            # Update position
            obs['x'] += obs['vx'] * self.dt
            obs['y'] += obs['vy'] * self.dt

            # Bounce off bounds
            if obs['x'] <= self.bounds_x[0] or obs['x'] >= self.bounds_x[1]:
                obs['vx'] *= -1.0
                obs['x'] = max(self.bounds_x[0],
                               min(self.bounds_x[1], obs['x']))
            if obs['y'] <= self.bounds_y[0] or obs['y'] >= self.bounds_y[1]:
                obs['vy'] *= -1.0
                obs['y'] = max(self.bounds_y[0],
                               min(self.bounds_y[1], obs['y']))

            # Set new pose via ros_gz_interfaces
            req = SetEntityPose.Request()
            req.entity = Entity(name=obs['name'], type=Entity.MODEL)
            req.pose = Pose(
                position=Point(
                    x=obs['x'], y=obs['y'], z=self.obs_size[2] / 2.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))

            self.set_pose_client.call_async(req)


def main():
    rclpy.init()
    node = ObstacleSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
