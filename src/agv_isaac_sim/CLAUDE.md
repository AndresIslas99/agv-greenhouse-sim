# agv_isaac_sim

NVIDIA Isaac Sim integration for the AGV greenhouse virtual body. The only
simulation target — Gazebo was removed 2026-04-15.

## Architecture

> Rule: **body, not brain.**
>
> Isaac Sim provides physics + sensors + actuators. The Jetson brain at
> [agv-greenhouse](https://github.com/AndresIslas99/agv-greenhouse) runs
> EKF, SLAM, Nav2, AprilTag detection, safety and waypoint management.

## Key files

- `urdf/` — URDF/xacro. Calibrated physical params match the real robot.
- `scripts/build_greenhouse_usd.py` — Procedural USD world builder. Reads textures from `assets/greenhouse_textures/` and `assets/tag_textures/`
- `scripts/import_robot_usd.py` — URDF → USD import with sensor setup
- `scripts/setup_omnigraph.py` / `setup_omnigraph_standalone.py` — OmniGraph publishers/subscribers
- `scripts/isaac_ros_bridge_node.py` — IMU bias drift relay: `/agv/imu/data_clean` → `/agv/imu/data`
- `scripts/sim_depth_noise.py` — Depth noise relay: `/agv/zed/depth/depth_clean` → `/agv/zed/depth/depth_registered`
- `scripts/sim_motor_gate.py` — ODrive arm/disarm emulation + `/agv/motor_state` + `/agv/drive_debug`
- `scripts/sim_global_odom.py` — HIL cuVSLAM replacement (wheel_odom → `/visual_slam/tracking/odometry`)
- `scripts/sim_domain_randomizer.py` — Lighting, friction, clutter randomization (sim-to-real transfer)
- `scripts/generate_hdri.py` — Greenhouse HDRI for Isaac DomeLight
- `scripts/validate_apriltag_detection.py` — AprilTag visibility report
- `launch/isaac_sim.launch.py` — Sensor base (TFs + relays, no drive wiring)
- `launch/isaac_teleop.launch.py` — Standalone: base + direct keyboard teleop
- `launch/isaac_hil.launch.py` — Production HIL: base + motor gate + drive shaping + sim_global_odom + pointcloud_to_laserscan
- `assets/` — Shared textures (greenhouse_textures, tag_textures, greenhouse_sky.hdr)
- `worlds/greenhouse_simple.usd` — Generated USD world
- `robot/agv_sim.usd` — Generated robot USD

## Dependencies

- `agv_sim_drive` — C++ drive shaping node (separate package because C++)

## Topic parity

All topics match the brain's contract in
[agv-greenhouse/specs/interfaces.yaml](https://github.com/AndresIslas99/agv-greenhouse/blob/main/specs/interfaces.yaml).
See `../../TOPIC_CONTRACT.md` for the table.

## Prerequisites

- NVIDIA Isaac Sim 4.x
- NVIDIA driver 535+
- RTX GPU with 8+ GB VRAM
