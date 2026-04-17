# Simulation validation checklist (Isaac Sim only)

Smoke tests to run after any change. All commands assume
`source install/setup.bash` and Isaac Sim already open with the
generated USD.

---

## 1. Sensor base (isaac_sim.launch.py)

Usually included by teleop/hil, but can be run standalone to check
that the static TFs and realism relays come up:

```bash
ros2 launch agv_isaac_sim isaac_sim.launch.py
```

| Check | Command | Pass criteria |
|---|---|---|
| robot_state_publisher up | `ros2 node list \| grep robot_state_publisher` | present |
| imu_link static TF | `ros2 run tf2_ros tf2_echo base_link imu_link` | `(0.70, 0.0, -0.055)` |
| zed_camera_link static TF | `ros2 run tf2_ros tf2_echo base_link zed_camera_link` | `(0.70, 0.0, -0.055)` |
| stereo baseline | `ros2 run tf2_ros tf2_echo zed_left_camera_frame zed_right_camera_frame` | y = -0.12 |
| IMU relay alive | `ros2 node list \| grep isaac_ros_bridge` | present |
| Depth noise relay alive | `ros2 node list \| grep sim_depth_noise` | present |

---

## 2. Standalone teleop (isaac_teleop.launch.py)

Drive the robot with a keyboard, verify the brain-facing topics.

```bash
# Terminal 1: Isaac Sim
isaacsim --open-usd src/agv_isaac_sim/worlds/greenhouse_simple.usd --ros2
# Terminal 2: launch
ros2 launch agv_isaac_sim isaac_teleop.launch.py
# Drive with i/k/j/l in the xterm window that opens.
```

| Check | Command | Pass criteria |
|---|---|---|
| Wheel odom rate | `ros2 topic hz /agv/wheel_odom` | ~50 Hz |
| Joint state rate | `ros2 topic hz /agv/joint_states` | ~50 Hz |
| IMU rate | `ros2 topic hz /agv/imu/data` | ~200 Hz (target; actual depends on OmniGraph tick) |
| Left RGB | `ros2 topic hz /agv/zed/left/image_rect_color` | ~15 Hz |
| Right RGB | `ros2 topic hz /agv/zed/right/image_rect_color` | ~15 Hz |
| Depth | `ros2 topic hz /agv/zed/depth/depth_registered` | ~15 Hz |
| Point cloud | `ros2 topic hz /agv/zed/point_cloud/cloud_registered` | ~15 Hz |
| Drive forward | `ros2 topic pub /agv/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}" -r 10` then `ros2 topic echo /agv/wheel_odom --once` | `pose.position.x` increases over 4 s |
| TF: odom → base_link | `ros2 run tf2_ros tf2_echo odom base_link` | translation changes when driving |
| Physical params | `ros2 param get /agv/sim_drive_shaping_node wheel_radius` | `0.0781` |
| Track width | `ros2 param get /agv/sim_drive_shaping_node track_width` | `0.960` |

---

## 3. HIL production (isaac_hil.launch.py)

Requires the brain running on another machine (or localhost) with
`ROS_DOMAIN_ID=42`.

```bash
# Terminal 1: Isaac Sim
isaacsim --open-usd src/agv_isaac_sim/worlds/greenhouse_simple.usd --ros2
# Terminal 2: sim side (PC)
export ROS_DOMAIN_ID=42
ros2 launch agv_isaac_sim isaac_hil.launch.py
# Terminal 3: brain side (Jetson or localhost)
export ROS_DOMAIN_ID=42
ros2 launch agv_bringup agv_hil_full.launch.py map:=/path/to/greenhouse_map.yaml
```

| Check | Where | Command | Pass criteria |
|---|---|---|---|
| Brain sees sim topics | brain | `ros2 topic list \| grep -E "wheel_odom\|imu/data\|zed/left"` | all present |
| /clock flowing | brain | `ros2 topic hz /clock` | >0 Hz |
| motor_state JSON format | brain | `ros2 topic echo /agv/motor_state --once` | 13 fields: left_state, right_state, left_errors, right_errors, armed, bus_voltage, bus_current, left_fet_temp, left_motor_temp, right_fet_temp, right_motor_temp, thermal_state |
| /visual_slam/tracking/odometry | brain | `ros2 topic hz /visual_slam/tracking/odometry` | ~50 Hz |
| /agv/scan flowing | brain | `ros2 topic hz /agv/scan` | ~10 Hz (brain runs pointcloud_to_laserscan on its side) |
| ekf_local up | brain | `ros2 node list \| grep ekf_local` | present |
| /agv/odometry/local | brain | `ros2 topic hz /agv/odometry/local` | ~50 Hz |
| /agv/odometry/global | brain | `ros2 topic hz /agv/odometry/global` | ~10-20 Hz |
| TF: odom → base_link (brain) | brain | `ros2 run tf2_ros tf2_echo odom base_link` | Published by brain ekf_local |
| TF: map → odom | brain | `ros2 run tf2_ros tf2_echo map odom` | Published by brain ekf_global |
| No sim-side /tf from odom/map | sim PC | `ros2 run tf2_ros tf2_monitor 2>&1 \| grep conflicting` | no conflict warnings |
| No brain nodes on sim PC | sim PC | `ros2 node list` | NO ekf, slam_toolbox, nav2_*, apriltag, map_server |
| Round-trip cmd_vel | brain | `ros2 topic pub /agv/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}" -r 10` | Isaac robot moves, `/agv/odometry/local.x` on brain increases |

---

## Sim-to-real parity reference

| Parameter | Sim | Real robot (brain) | Must match |
|---|---|---|---|
| Namespace | `/agv` | `/agv` | ✓ |
| wheel_radius | 0.0781 | 0.0781 | ✓ |
| track_width | 0.960 | 0.960 | ✓ |
| gear_ratio | 10.0 | 10.0 | ✓ |
| invert_left | true | true | ✓ |
| invert_right | false | false | ✓ |
| chassis dim | 1.0×0.6×0.15 | (same, box) | ✓ |
| base_footprint z | -0.200 | -0.200 | ✓ |
| camera_mount | (0.70, 0, -0.055) | (0.70, 0, -0.055) | ✓ |
| All frame IDs | base_link, odom, map, imu_link, zed_* | same | ✓ |
