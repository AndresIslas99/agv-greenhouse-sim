# Simulation Validation Checklist

Daily regression tests for each simulation mode. Run after any code change.
All commands assume `source install/setup.bash` and `export GZ_IP=127.0.0.1`.

---

## 1. sim_teleop

```bash
ros2 launch agv_sim_bringup sim_teleop.launch.py
```

| Check | Command | Pass Criteria |
|-------|---------|---------------|
| Topics present | `ros2 topic list \| wc -l` | >= 19 topics |
| Odom flowing | `ros2 topic hz /agv/wheel_odom` | ~44 Hz |
| IMU flowing | `ros2 topic hz /zed/zed_node/imu/data` | ~300 Hz |
| Joint states | `ros2 topic hz /agv/joint_states` | > 100 Hz |
| Drive forward | `ros2 topic pub /agv/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}" -r 10` then `ros2 topic echo /agv/wheel_odom --once \| grep "x:"` | x increases over 4s |
| TF: odom->base_link | `ros2 run tf2_ros tf2_echo odom base_link` | Translation changes when driving |
| Drive shaping | `ros2 param get /agv/sim_drive_shaping_node invert_right` | `Boolean value is: False` |

---

## 2. sim_mapping

```bash
ros2 launch agv_sim_bringup sim_mapping.launch.py
```

| Check | Command | Pass Criteria |
|-------|---------|---------------|
| slam_toolbox running | `ros2 node list \| grep slam` | `/agv/slam_toolbox` present |
| TF: map->odom | `ros2 run tf2_ros tf2_echo map odom` | Transform available |
| TF: odom->base_link | `ros2 run tf2_ros tf2_echo odom base_link` | From diff_drive bridge |
| Map topic | `ros2 topic list \| grep map` | `/agv/map` present |

**Note:** Requires working lidar (GPU renderer). If lidar is unavailable, slam_toolbox will not produce a map.

---

## 3. sim_fusion

```bash
ros2 launch agv_sim_bringup sim_fusion.launch.py map:=/path/to/slam_map
```

| Check | Command | Pass Criteria |
|-------|---------|---------------|
| EKF local running | `ros2 node list \| grep ekf_local` | Present |
| EKF global running | `ros2 node list \| grep ekf_global` | Present |
| Local odom output | `ros2 topic echo /agv/odometry/local --once` | Has data |
| Global odom output | `ros2 topic echo /agv/odometry/global --once` | Has data |
| TF: odom->base_link | `ros2 run tf2_ros tf2_echo odom base_link` | From EKF local (NOT diff_drive) |
| TF: map->odom | `ros2 run tf2_ros tf2_echo map odom` | From EKF global (NOT slam_toolbox) |
| No TF conflict | `ros2 run tf2_ros tf2_monitor 2>&1 \| grep "conflicting"` | No "conflicting" messages |

**Critical:** slam_toolbox must have `publish_tf: false` in this mode. EKF owns both TF transforms.

---

## 4. sim_nav

```bash
ros2 launch agv_sim_bringup sim_nav.launch.py map_yaml:=/path/to/greenhouse_simple.yaml
```

| Check | Command | Pass Criteria |
|-------|---------|---------------|
| Map server running | `ros2 node list \| grep map_server` | Present |
| Nav2 lifecycle active | `ros2 lifecycle list /agv/planner_server` | State: active |
| Global costmap | `ros2 topic echo /agv/global_costmap/costmap --once` | Has data |
| Local costmap | `ros2 topic echo /agv/local_costmap/costmap --once` | Has data |
| Send goal | See command below | Robot reaches goal |

**Nav2 goal test (spawn→aisle 3-4 entrance):**
```bash
ros2 action send_goal /agv/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 0.0}, orientation: {w: 1.0}}}}"
```
Pass: Robot moves toward goal. Failure modes: "No valid path" (missing map), "Transform timeout" (broken TF tree).

---

## 5. sim_apriltag

Run alongside any mode:
```bash
ros2 launch agv_sim_bringup sim_apriltag.launch.py
```

| Check | Command | Pass Criteria |
|-------|---------|---------------|
| apriltag_ros running | `ros2 node list \| grep apriltag` | Present |
| Detection topic | `ros2 topic list \| grep tag` | `/agv/tag_detections` exists |

**Note:** Camera-based detection requires working GPU renderer. Use fake detector as fallback:
```bash
ros2 launch agv_sim_apriltags fake_markers.launch.py
```

---

## 6. sim_external (PC as world/sensor provider, Jetson as brain)

**PC:**
```bash
ros2 launch agv_sim_bringup sim_external.launch.py
```

**Jetson (verify topics arrive):**
```bash
ros2 topic list | grep -E "wheel_odom|imu|scan|joint_states|clock"
```

| Check | Where | Command | Pass Criteria |
|-------|-------|---------|---------------|
| Topics visible | Jetson | `ros2 topic list` | /agv/wheel_odom, /zed/zed_node/imu/data, /clock present |
| Odom data flows | Jetson | `ros2 topic echo /agv/wheel_odom --once` | Has data with timestamps |
| IMU data flows | Jetson | `ros2 topic hz /zed/zed_node/imu/data` | ~300 Hz |
| cmd_vel round-trip | Jetson | `ros2 topic pub /agv/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}" -r 10` | PC robot moves, Jetson odom.x increases |
| No odom TF from PC | Jetson | `ros2 run tf2_ros tf2_echo odom base_link` | Should FAIL (no publisher — Jetson EKF must own this) |
| Static TF present | Jetson | `ros2 run tf2_ros tf2_echo base_link laser_frame` | Transform available (from robot_state_publisher) |
| No brain on PC | PC | `ros2 node list` | No ekf, slam, nav2, or map_server nodes |

**See `TOPIC_CONTRACT.md` for the full PC-Jetson topic specification.**

---

## Sim-to-Real Parity Reference

| Parameter | SIM | REAL | Intentional? |
|-----------|-----|------|--------------|
| Namespace | `/agv` | `/agv` | Match |
| wheel_radius | 0.0625 | 0.0625 | Match |
| track_width | 0.735 | 0.735 | Match |
| invert_right | false | true | Yes — Gz DiffDrive vs ODrive CAN |
| SLAM | slam_toolbox | cuVSLAM | Yes — cuVSLAM unavailable in Gz |
| EKF global source | `pose` (slam_toolbox) | `/visual_slam/tracking/odometry` | Yes — follows SLAM choice |
| All other EKF params | identical | identical | Match |
| All frame IDs | base_link, odom, map | base_link, odom, map | Match |
