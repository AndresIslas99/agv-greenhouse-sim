# AGV Greenhouse Simulation — CLAUDE.md

This repository is the **virtual body** of the AGV for NVIDIA Isaac Sim.
It emulates the physics, sensors and actuators of the real robot so that
the production brain at [agv-greenhouse](https://github.com/AndresIslas99/agv-greenhouse)
can run unmodified against it.

## Architecture Rule

> **The sim is the body, not the brain.**
>
> Never add EKF, Nav2, SLAM, cuVSLAM, AprilTag detection, safety gates,
> waypoint management or any other brain component here. The sim's only
> job is to publish the topics the brain expects from real hardware —
> same names, same types, same frames, same rates.
>
> If the real ZED 2i / BMI088 / ODrive S1 doesn't produce a topic,
> we don't produce it either. If the brain is responsible for a
> computation (visual SLAM, point-cloud-to-laserscan, EKF fusion, etc.),
> we do not reimplement it in the sim — except where the HIL launch
> from the brain explicitly delegates it to the sim PC.

## Packages

| Package | Purpose |
|---------|---------|
| `agv_isaac_sim` | Full Isaac Sim integration: USD world builders, URDF, launches, OmniGraph setup, IMU/depth realism relays, ODrive arm/disarm emulation, assets |
| `agv_sim_drive` | C++ node that emulates the ODrive internal acceleration ramp (cmd_vel → shaped_cmd_vel). Standalone because it's C++; could be merged into `agv_isaac_sim` if we drop the ament_cmake dep |

That's it. Two packages. No Gazebo. No shared-utility packages.

## Brain contract — what the sim publishes

See [TOPIC_CONTRACT.md](TOPIC_CONTRACT.md) for the full table. The critical
topics the brain consumes in HIL mode
([agv_hil_full.launch.py](https://github.com/AndresIslas99/agv-greenhouse/blob/main/src/agv_bringup/launch/agv_hil_full.launch.py)):

| Topic | Type | Rate | Frame | Source |
|---|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | sim time | — | Isaac Sim |
| `/agv/wheel_odom` | `nav_msgs/Odometry` | 50 Hz | `odom`→`base_link` | OmniGraph ComputeOdometry |
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
| `/agv/scan` | `sensor_msgs/LaserScan` | ~10 Hz | `base_link` | pointcloud_to_laserscan with brain's production params (HIL only) |
| `/visual_slam/tracking/odometry` | `nav_msgs/Odometry` | 50 Hz | `map`→`base_link` | sim_global_odom (wheel_odom relay — HIL cuVSLAM replacement) |

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
| Sensor base | `ros2 launch agv_isaac_sim isaac_sim.launch.py` | Just the virtual body (robot_state_publisher, static TFs, IMU/depth realism relays). Intended to be included by teleop/hil, not run alone. |
| Standalone teleop | `ros2 launch agv_isaac_sim isaac_teleop.launch.py` | Sensor base + direct keyboard→/agv/cmd_vel→drive shaping. Bypasses the motor gate for easy smoke testing. |
| HIL production | `ros2 launch agv_isaac_sim isaac_hil.launch.py` | Brain-compatible mode: motor gate + drive shaping + sim_global_odom + pointcloud_to_laserscan. Requires `ROS_DOMAIN_ID=42` and a running brain on the other side. |

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
