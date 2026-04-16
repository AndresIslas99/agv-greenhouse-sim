# Topic Contract — what `agv-sim` publishes to the brain

Authoritative source: [agv-greenhouse/specs/interfaces.yaml](https://github.com/AndresIslas99/agv-greenhouse/blob/main/specs/interfaces.yaml)
and [agv_hil_full.launch.py](https://github.com/AndresIslas99/agv-greenhouse/blob/main/src/agv_bringup/launch/agv_hil_full.launch.py).

The sim must publish every topic in the "Sim publishes" table below, with
the exact name, message type, frame_id and rate the brain expects. Anything
missing or mis-typed will break EKF / Nav2 / AprilTag detection silently.

## Sim publishes (to the brain)

| Topic | Type | Rate | Frame | Producer in sim | Notes |
|---|---|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | sim | — | Isaac Sim internal | Required when `use_sim_time=true` on the brain side |
| `/agv/wheel_odom` | `nav_msgs/Odometry` | 50 Hz | `odom`→`base_link` | OmniGraph ComputeOdometry | Covariance is zero from sim; brain's `covariance_override_node` patches it |
| `/agv/joint_states` | `sensor_msgs/JointState` | 50 Hz | — | OmniGraph JointStatePublisher | Joints `left_wheel_joint`, `right_wheel_joint` in radians |
| `/agv/imu/data` | `sensor_msgs/Imu` | 200 Hz | `imu_link` | `isaac_ros_bridge_node.py` (reads `_clean` from OmniGraph and injects BMI088 bias drift) | Covariance is zero; brain's `covariance_override_node` patches |
| `/agv/zed/left/image_rect_color` | `sensor_msgs/Image` | 15 Hz | `zed_left_camera_frame` | OmniGraph CameraHelper | Real ZED SDK publishes rectified — no distortion in sim output |
| `/agv/zed/left/camera_info` | `sensor_msgs/CameraInfo` | 15 Hz | `zed_left_camera_frame` | OmniGraph | K matrix from `config/zed2i_calibration.yaml` |
| `/agv/zed/right/image_rect_color` | `sensor_msgs/Image` | 15 Hz | `zed_right_camera_frame` | OmniGraph | |
| `/agv/zed/right/camera_info` | `sensor_msgs/CameraInfo` | 15 Hz | `zed_right_camera_frame` | OmniGraph | |
| `/agv/zed/depth/depth_registered` | `sensor_msgs/Image` | 15 Hz | `zed_left_camera_frame` | `sim_depth_noise.py` (reads `_clean` from OmniGraph and adds σ(d) = 0.002 + 0.005·d Gaussian + dropout beyond 15 m) | 32FC1, meters |
| `/agv/zed/point_cloud/cloud_registered` | `sensor_msgs/PointCloud2` | 15 Hz | `zed_left_camera_frame` | OmniGraph | Used by sim-side `pointcloud_to_laserscan` in HIL |
| `/agv/motor_state` | `std_msgs/String` (JSON) | 10 Hz | — | `sim_motor_gate.py` | JSON format matches `odrive_can_node.cpp:629` exactly |
| `/agv/drive_debug` | `std_msgs/String` (JSON) | 10 Hz | — | `sim_motor_gate.py` | JSON format matches `odrive_can_node.cpp:647` |
| `/agv/scan` | `sensor_msgs/LaserScan` | ~10 Hz | `base_link` | `pointcloud_to_laserscan` inside `isaac_hil.launch.py` | Same production params as the brain's `agv_full.launch.py`: `min_height=0.01, max_height=2.0, range_min=0.3, range_max=8.0`. **HIL only** — teleop mode doesn't publish this |
| `/visual_slam/tracking/odometry` | `nav_msgs/Odometry` | 50 Hz | `map`→`base_link` | `sim_global_odom.py` | cuVSLAM replacement for HIL. Brain's `ekf_global` consumes as `odom1` with `differential: true`, so absolute drift does not matter. **HIL only** |

## Sim subscribes (from the brain)

| Topic | Type | Consumer in sim | Notes |
|---|---|---|---|
| `/agv/cmd_vel` | `geometry_msgs/Twist` | `sim_motor_gate` → `cmd_vel_armed` → `sim_drive_shaping_node` → `/agv/shaped_cmd_vel` → OmniGraph DifferentialController | Mapping-first mode |
| `/agv/cmd_vel_safe` | `geometry_msgs/Twist` | `topic_tools::relay` → `/agv/cmd_vel` → motor gate | HIL `has_map:=true` mode (brain's safety gate output) |
| `/agv/motor_enable` | `std_msgs/Bool` | `sim_motor_gate` | Arm/disarm ODrive |
| `/agv/e_stop` | `std_msgs/Bool` | `sim_motor_gate` | Hardware/software e-stop |

## What the sim does NOT publish (brain owns these)

Any of these appearing from the sim is a bug:

- `/agv/odometry/local`, `/agv/odometry/global` — brain's dual EKF
- `/agv/imu/filtered` — brain's Butterworth filter
- `/agv/cmd_vel_smoothed`, `/agv/cmd_vel_collision_safe` — brain's Nav2 chain
- `/agv/cmd_vel_safe` — brain's `cmd_vel_gate`
- `/agv/pose`, `/agv/live_map`, `/agv/map` — brain's fusion_monitor / map_manager / map_server
- `/agv/marker_pose`, `/detections` — brain's `apriltag_ros` + `marker_correction_node`
- `/agv/navigate_to_pose` (action) — brain's Nav2 BT navigator
- `/tf` edges for `map→odom` and `odom→base_link` — brain's EKFs own them in HIL
- `/agv/zed/pose_with_covariance` — ZED SDK area memory (real only; in HIL the brain tolerates it missing)
- `/agv/battery` — planned real driver output, not yet published by anyone

## Frame tree published by the sim

```
Standalone (isaac_sim / isaac_teleop):
  base_link                      (from URDF)
    ├── left_wheel                (from URDF, continuous)
    ├── right_wheel               (from URDF, continuous)
    ├── base_footprint            (from URDF, z=-0.200)
    ├── front_caster, rear_caster (from URDF, Isaac physics only)
    ├── imu_link                  (static TF, (0.70, 0, -0.055))
    └── zed_camera_link           (static TF, (0.70, 0, -0.055))
          ├── zed_left_camera_frame   (static TF, +0.06 Y)
          └── zed_right_camera_frame  (static TF, -0.06 Y)

HIL (isaac_hil):
  The sim publishes NO TF — the brain's robot_state_publisher +
  agv_slam.launch.py own the full tree. The sim only publishes
  /agv/wheel_odom (as a topic, not as TF) and sensor topics.
```

## Physical params (must match `agv-greenhouse/src/agv_bringup/config/robot/agv_geometry.yaml`)

| | Calibrated value | Source |
|---|---|---|
| `wheel_radius` | `0.0781` m | `agv_odrive/config/odrive_params.yaml` |
| `track_width` | `0.960` m | same |
| `gear_ratio` | `10.0` | same |
| `invert_left` | `true` | same |
| `invert_right` | `false` | same |
| `camera_mount` | `(0.70, 0, -0.055)` rpy `(0,0,0)` | `agv_geometry.yaml#camera_mount` |
| `chassis_mass` | `30.0` kg | brain URDF `agv_base.xacro` |
| `base_footprint z` | `-0.200` m | brain URDF |
