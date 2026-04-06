# AGV Greenhouse Simulation — CLAUDE.md

This repository is the **virtual body** of the AGV — it contains only the
simulated physics, sensors, and actuators. The real brain (EKF, Nav2, SLAM,
cuVSLAM) lives in [agv-greenhouse](https://github.com/AndresIslas99/agv-greenhouse).

## Architecture Rule

> **The sim is the body, not the brain.**
>
> Never add EKF, Nav2, SLAM, or cuVSLAM nodes here.
> The sim publishes raw sensor data; the brain subscribes and drives.

## Packages

| Package | Purpose |
|---------|---------|
| `agv_sim_description` | URDF/xacro robot model (diff-drive, ZED 2i, IMU) |
| `agv_sim_worlds` | Gazebo SDF worlds, AprilTag models, PBR textures |
| `agv_sim_bringup` | Launch files (7 modes), EKF configs, bridge YAML |
| `agv_sim_drive` | C++ drive-shaping node (ODrive-realistic) |
| `agv_sim_nav` | Nav2 parameter files |
| `agv_sim_apriltags` | Marker registry + fake proximity detector |
| `agv_isaac_sim` | **Isaac Sim** integration (GPU physics + RTX rendering) |

## Topic Parity

Sim topics must match the real robot 1:1. See `TOPIC_CONTRACT.md` for the
full table. Key topics:

| Sim | Real source | Must match |
|-----|-------------|------------|
| `/agv/wheel_odom` | agv_odrive | name + type |
| `/zed/zed_node/imu/data` | ZED 2i SDK | name + type |
| `/agv/cmd_vel` | Nav2/teleop | name + type |
| `/agv/joint_states` | agv_odrive | name + type |

## Launch Modes

| Mode | Command | What it runs |
|------|---------|-------------|
| Teleop | `ros2 launch agv_sim_bringup sim_teleop.launch.py` | Gz + robot + keyboard |
| Mapping | `ros2 launch agv_sim_bringup sim_mapping.launch.py` | + SLAM Toolbox |
| Fusion | `ros2 launch agv_sim_bringup sim_fusion.launch.py` | + dual EKF |
| Nav | `ros2 launch agv_sim_bringup sim_nav.launch.py` | + Nav2 |
| AprilTag | `ros2 launch agv_sim_bringup sim_apriltag.launch.py` | + fake marker detection |
| External | `ros2 launch agv_sim_bringup sim_external.launch.py` | Gz + bridge (Jetson drives) |

### Isaac Sim Modes (GPU-accelerated, RTX 4080)

| Mode | Command | What it runs |
|------|---------|-------------|
| Teleop | `ros2 launch agv_isaac_sim isaac_teleop.launch.py` | Isaac Sim + robot + keyboard |
| Nav | `ros2 launch agv_isaac_sim isaac_nav.launch.py` | + EKF + SLAM + Nav2 |
| External | `ros2 launch agv_isaac_sim isaac_external.launch.py` | Isaac Sim + bridge (Jetson drives) |

## Conventions

- **Commit style**: `type: short description` (feat, fix, refactor, docs, test, ci)
- **Python**: ROS2 Humble style, `#!/usr/bin/env python3`, snake_case
- **C++**: C++17, `-Wall -Wextra -Wpedantic`
- **Xacro**: one file per subsystem (base, sensors, wheels)
- **SDF**: all models under `agv_sim_worlds/models/`
- **Tests**: pytest under `test/` dirs, run with `colcon test`
- **CI**: GitHub Actions (`.github/workflows/ci.yaml`)

## Common Pitfalls

1. **use_sim_time** — all nodes consuming sim topics must set `use_sim_time: true`
2. **GPU topics** — `/agv/scan` and all camera topics need GPU rendering (ogre2 / RTX)
3. **Frame names** — must match real ZED 2i frame hierarchy exactly
4. **Namespace** — robot topics use `/agv/` namespace
5. **Isaac Sim USD** — run `build_greenhouse_usd.py` + `import_robot_usd.py` before first launch
