# AGV Greenhouse Simulation — CLAUDE.md

This repository is the **virtual body** of the AGV for NVIDIA Isaac Sim.
It emulates the physics, sensors and actuators of the real robot so that
the production brain at [agv-greenhouse](https://github.com/AndresIslas99/agv-greenhouse)
can run unmodified against it.

## Architecture Rule

> **The sim is the body, not the brain.**
>
> Whatever runs on the Jetson on the real robot must also run on the
> Jetson in HIL. The sim only emulates hardware that physically requires
> the chip itself in the loop and therefore cannot be moved Jetson-side.
>
> If the real ZED 2i / BMI088 / ODrive S1 doesn't produce a topic, we
> don't produce it either. EKF, SLAM, cuVSLAM, Nav2, AprilTag detection,
> pointcloud_to_laserscan, costmaps, safety gates, waypoint management
> — none of these belong here. If you find one in the sim, file it as a
> leak and migrate it.

### What the sim IS allowed to publish

**(1) Hardware emulation — forced sim-side because the real chip is not
in the loop in HIL:**

| Topic | Real-robot source | Why sim-side |
|---|---|---|
| `/clock` | — | Sim time master |
| `/tf`, `/tf_static` | `robot_state_publisher` from URDF | Brain runs same RSP, but in HIL the sim runs it too because it owns the URDF (no double-publish issue: identical TFs deduplicate) |
| `/agv/joint_states` | ODrive encoders via CAN | OmniGraph emulates the encoders |
| `/agv/imu/data` | BMI088 chip + on-chip bias drift | OmniGraph + isaac_ros_bridge emulate the chip |
| `/agv/zed/left/image_rect_color` | **ZED SDK on Jetson** (rectification) | ZED SDK requires the actual ZED hardware. In HIL the brain CANNOT run the SDK, so Isaac generates the rectified images directly. |
| `/agv/zed/right/image_rect_color` | ZED SDK | Same |
| `/agv/zed/left/camera_info` | ZED SDK | Same |
| `/agv/zed/right/camera_info` | ZED SDK | Same |
| `/agv/zed/depth/depth_registered` | ZED SDK (stereo matching) | Same |
| `/agv/zed/point_cloud/cloud_registered` | ZED SDK (depth → 3D) | Same |
| `/agv/motor_state`, `/agv/drive_debug` | ODrive CAN driver | Emulated firmware behavior |
| `/agv/cmd_vel_armed`, `/agv/shaped_cmd_vel` | ODrive arm gate + accel ramp (firmware) | Emulated |

**(2) Validation oracle — separate `/agv/sim/*` namespace, never collides
with the brain contract:**

| Topic | Purpose |
|---|---|
| `/agv/sim/ground_truth/pose` | PhysX truth pose for grading |
| `/agv/sim/ground_truth/twist` | PhysX truth velocity |
| `/agv/sim/ground_truth/visible_markers` | AprilTag frustum oracle |
| `/agv/sim/ground_truth/obstacles` | Static obstacle catalog (latched) |
| `/agv/sim/localization_error` | GT vs brain est_pose comparison |
| `/agv/sim/events` | Discrete event timeline |
| `/agv/sim/episode_summary` | Nav2 mission metrics (latched per episode) |
| `/agv/sim/reset_request`, `/agv/sim/reset_done`, `/agv/sim/control` | Remote control surface |

### What the sim is NOT allowed to publish (brain-owned in HIL)

The brain MUST run an equivalent for each of these on its HIL launch.
On the real robot every one of them runs on the Jetson — HIL must mirror
that or it isn't a real test.

| Topic | Real-robot source | Brain HIL action |
|---|---|---|
| `/agv/wheel_odom` | `agv_odrive_node` (Jetson, integrates CAN encoder ticks) | Subscribe `/agv/joint_states`, integrate. Reference: `scripts/sim_wheel_odom_publisher.py` |
| `/agv/scan` | `pointcloud_to_laserscan` (Jetson, next to Nav2) | Subscribe `/agv/zed/point_cloud/cloud_registered`, run pcl_to_scan with same params as `agv_full.launch.py` |
| `/visual_slam/tracking/odometry` | cuVSLAM (`isaac_ros_visual_slam` on Jetson) | Run cuVSLAM with sim's stereo+IMU, OR (faster fallback) relay `/agv/wheel_odom` with `frame_id=map` |
| `/agv/odometry/local` | `ekf_local` (Robot Localization, Jetson) | Already runs Jetson-side in production |
| `/agv/odometry/global` | `ekf_global` | Same |
| TF `odom → base_link` | `ekf_local` | Same |
| TF `map → odom` | `ekf_global` | Same |
| `/detections` (AprilTag) | `apriltag_ros` (Jetson) | Same |
| Costmaps, planners, behavior trees | Nav2 (Jetson) | Same |

### How to spot a leak

Any new sim-side node you write is suspect. Ask:

1. *Is the input something only sim has* (joint_states, raw cloud)? OK to consume.
2. *Is the output something the real Jetson would publish*? If yes, it's
   a leak. The Jetson should consume the same input you would have.
3. *Could you run this same node unchanged on the Jetson if a brain dev
   asked you to*? If yes, ship it to the brain instead of running it here.

## Packages

| Package | Purpose |
|---------|---------|
| `agv_isaac_sim` | Full Isaac Sim integration: USD world builders, URDF, launches, OmniGraph setup, IMU/depth realism relays, ODrive arm/disarm emulation, assets |
| `agv_sim_drive` | C++ node that emulates the ODrive internal acceleration ramp (cmd_vel → shaped_cmd_vel). Standalone because it's C++; could be merged into `agv_isaac_sim` if we drop the ament_cmake dep |

That's it. Two packages. No Gazebo. No shared-utility packages.

## Brain contract — what the sim publishes

See [TOPIC_CONTRACT.md](TOPIC_CONTRACT.md) for the full table. The
hardware-emulation topics the sim ships to the brain in HIL:

| Topic | Type | Rate | Frame | Source |
|---|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | sim time | — | Isaac Sim |
| `/agv/joint_states` | `sensor_msgs/JointState` | 50 Hz | — | OmniGraph JointStatePublisher |
| `/agv/imu/data` | `sensor_msgs/Imu` | 200 Hz | `imu_link` | OmniGraph ReadIMU → isaac_ros_bridge_node (bias drift) |
| `/agv/zed/left/image_rect_color` | `sensor_msgs/Image` | 15 Hz | `zed_left_camera_frame` | OmniGraph CameraHelper |
| `/agv/zed/left/camera_info` | `sensor_msgs/CameraInfo` | 15 Hz | `zed_left_camera_frame` | OmniGraph |
| `/agv/zed/right/image_rect_color` | `sensor_msgs/Image` | 15 Hz | `zed_right_camera_frame` | OmniGraph |
| `/agv/zed/right/camera_info` | `sensor_msgs/CameraInfo` | 15 Hz | `zed_right_camera_frame` | OmniGraph |
| `/agv/zed/depth/depth_registered` | `sensor_msgs/Image` | 15 Hz | `zed_left_camera_frame` | OmniGraph → sim_depth_noise (Gaussian σ(d) = 0.002 + 0.005·d) |
| `/agv/zed/point_cloud/cloud_registered` | `sensor_msgs/PointCloud2` | 15 Hz | `zed_left_camera_frame` | OmniGraph |
| `/agv/motor_state` | `std_msgs/String` (JSON) | 10 Hz | — | sim_motor_gate (matches `odrive_can_node.cpp:629`) |
| `/agv/drive_debug` | `std_msgs/String` (JSON) | 10 Hz | — | sim_motor_gate |

**Brain owns in HIL** (NOT published by the sim — see Architecture Rule
above for the full list and rationale):
`/agv/wheel_odom`, `/agv/scan`, `/visual_slam/tracking/odometry`,
TF `odom→base_link`, TF `map→odom`, costmaps, AprilTag detections, etc.

**Sim subscribes** to `/agv/cmd_vel` (mapping-first) or `/agv/cmd_vel_safe` (has_map mode, via topic_tools relay), plus `/agv/motor_enable` and `/agv/e_stop`.

## Physical parameters

All values come from the brain's canonical config — calibrated 2026-04-08.

| Parameter | Value | Source |
|---|---|---|
| wheel_radius | **0.0781 m** | [odrive_params.yaml](https://github.com/AndresIslas99/agv-greenhouse/blob/main/src/agv_odrive/config/odrive_params.yaml) |
| track_width | **0.960 m** | same |
| gear_ratio | 10.0 | same |
| chassis | 1.0 × 0.6 × 0.15 m, mass 30 kg | brain URDF |
| base_footprint z | -0.200 m | brain URDF |
| camera mount | `base_link → (0.70, 0, -0.055)` | [agv_geometry.yaml](https://github.com/AndresIslas99/agv-greenhouse/blob/main/src/agv_bringup/config/robot/agv_geometry.yaml) |
| stereo baseline | 0.12 m | ZED 2i spec |
| invert_left / invert_right | true / false | odrive_params.yaml |

**Do not change these without re-measuring on the real robot.**

## Launch modes

| Mode | Command | Purpose |
|---|---|---|
| Sensor base | `ros2 launch agv_isaac_sim isaac_sim.launch.py` | Just the virtual body (robot_state_publisher, static TFs, IMU/depth realism relays). Intended to be included by teleop/hil. Pass `standalone_mode:=true` if running alone (brings up the odom→base_link TF broadcaster that the brain ekf_local would normally own in HIL). |
| Standalone teleop | `ros2 launch agv_isaac_sim isaac_teleop.launch.py` | Sensor base (with `standalone_mode:=true`) + direct keyboard→/agv/cmd_vel→drive shaping. Bypasses the motor gate. |
| HIL production | `ros2 launch agv_isaac_sim isaac_hil.launch.py` | Brain-compatible mode: motor gate + drive shaping + sim_wheel_odom_publisher + sim_global_odom. The brain owns `pointcloud_to_laserscan` on its side. Requires `ROS_DOMAIN_ID=42` and a running brain on the other side. |

Isaac Sim itself must be started separately with the generated USD:
```bash
isaacsim --open-usd src/agv_isaac_sim/worlds/greenhouse_simple.usd --ros2
```

## USD generators (one-time, run inside Isaac Sim Python)

```bash
isaacsim --exec src/agv_isaac_sim/scripts/build_greenhouse_usd.py   # world
isaacsim --exec src/agv_isaac_sim/scripts/import_robot_usd.py       # robot
```

The USD files under `src/agv_isaac_sim/worlds/` and `robot/` must be
regenerated whenever `world_config.yaml`, `import_robot_usd.py` or the
URDF changes. **After the 2026-04-15 brain-parity refactor, the USD is
stale and must be regenerated before first run.**

## Root scripts

| Script | Purpose | Output |
|---|---|---|
| `scripts/generate_textures.py` | Procedural PBR textures (ground, walls, leaves, rails, wood) | `src/agv_isaac_sim/assets/greenhouse_textures/` |
| `scripts/generate_apriltag_textures.py` | tag36h11 PNGs at 1024×1024 | `src/agv_isaac_sim/assets/tag_textures/` |
| `src/agv_isaac_sim/scripts/generate_hdri.py` | Greenhouse HDRI for Isaac DomeLight | `src/agv_isaac_sim/assets/greenhouse_sky.hdr` |
| `src/agv_isaac_sim/scripts/validate_apriltag_detection.py` | Isaac AprilTag visibility report | stdout + JSON |

## Conventions

- **Commit style**: `type: short description`
- **Python**: ROS 2 Humble style, `#!/usr/bin/env python3`, snake_case
- **C++**: C++17, `-Wall -Wextra -Wpedantic`
- **xacro**: one file per subsystem under `agv_isaac_sim/urdf/`

## Common pitfalls

1. **`use_sim_time`** — all nodes must set it in HIL mode.
2. **URDF out of date** — if you change physical params, regenerate USDs.
3. **Static TFs in HIL** — `isaac_hil.launch.py` does NOT publish static TFs
   for `imu_link`/`zed_*` because the brain's `agv_slam.launch.py` publishes
   them. Only `isaac_sim.launch.py` (standalone) publishes them.
4. **`ROS_DOMAIN_ID=42`** — mandatory in HIL (brain excludes rogue dev-PC
   publishers from DDS discovery).
5. **Topic rename** — all sensor topics are under `/agv/zed/*` (not
   `/zed/zed_node/*`). The rename happened 2026-04-15 during the brain-parity
   refactor; any remaining legacy references are bugs.
