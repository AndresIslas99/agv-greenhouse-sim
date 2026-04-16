# AGV Greenhouse Simulation — Isaac Sim

NVIDIA Isaac Sim virtual body for the AGV greenhouse robot. Publishes the
exact topic contract that the production brain at [agv-greenhouse](https://github.com/AndresIslas99/agv-greenhouse)
expects from the real ZED 2i + ODrive S1 + BMI088 hardware.

The sim is deliberately **dumb**: it provides physics + sensors + actuators,
nothing else. EKF, SLAM, Nav2, AprilTag detection and every other brain
component runs on the Jetson, not here.

## Prerequisites

- **ROS 2 Humble Hawksbill**
- **NVIDIA Isaac Sim 4.x** (standalone installation)
- **NVIDIA driver 535+** with an RTX GPU (8 GB+ VRAM)
- System packages:

```bash
sudo apt install \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-teleop-twist-keyboard \
  ros-humble-tf2-ros \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-topic-tools
```

## Build

```bash
cd ~/agv-sim
colcon build --symlink-install
source install/setup.bash
```

Two packages compile: `agv_isaac_sim` and `agv_sim_drive`.

## First-time setup — generate the USD files

The procedural greenhouse world and robot are built from config + URDF at
generation time. This must be done once after cloning and again whenever
`world_config.yaml`, `import_robot_usd.py`, or the URDF change:

```bash
cd ~/agv-sim
isaacsim --exec src/agv_isaac_sim/scripts/build_greenhouse_usd.py
isaacsim --exec src/agv_isaac_sim/scripts/import_robot_usd.py
```

Output:
- `src/agv_isaac_sim/worlds/greenhouse_simple.usd`
- `src/agv_isaac_sim/robot/agv_sim.usd`

## Launch modes

### Standalone teleop (smoke test)

Starts Isaac Sim ROS components with keyboard control. Good for verifying
that the sim is publishing the expected topics and the robot responds to
commands.

```bash
# Terminal 1 — Isaac Sim itself
isaacsim --open-usd src/agv_isaac_sim/worlds/greenhouse_simple.usd --ros2

# Terminal 2 — ROS launch (robot_state_publisher, realism relays, drive shaping, teleop)
ros2 launch agv_isaac_sim isaac_teleop.launch.py
```

An `xterm` window opens with teleop_twist_keyboard. Drive the robot and
observe topics:

```bash
ros2 topic hz /agv/wheel_odom                  # 50 Hz
ros2 topic hz /agv/imu/data                    # 200 Hz (target)
ros2 topic hz /agv/zed/left/image_rect_color   # 15 Hz
ros2 topic echo /agv/motor_state --once        # JSON with 13 fields
ros2 run tf2_tools view_frames                 # should include imu_link, zed_*
```

### HIL production (brain on Jetson drives the sim)

Starts exactly the components the brain's `agv_hil_full.launch.py`
expects. No brain logic runs in the sim — only the hardware-equivalent
publishers plus the minimum HIL shims (motor gate, cuVSLAM replacement,
pointcloud_to_laserscan with production params).

```bash
# Terminal 1 — Isaac Sim
isaacsim --open-usd src/agv_isaac_sim/worlds/greenhouse_simple.usd --ros2

# Terminal 2 — HIL launch on the sim PC
export ROS_DOMAIN_ID=42                           # required
export CYCLONEDDS_URI=file://$(pwd)/cyclonedds.xml  # for cross-machine discovery
ros2 launch agv_isaac_sim isaac_hil.launch.py

# Terminal 3 — the brain on the Jetson (or locally for testing)
export ROS_DOMAIN_ID=42
ros2 launch agv_bringup agv_hil_full.launch.py map:=/path/to/greenhouse_map.yaml
```

Optional: pass `has_map:=true` to `isaac_hil.launch.py` when running against
a brain configured with `has_map:=true` (the brain then publishes velocity
commands on `/agv/cmd_vel_safe` instead of `/agv/cmd_vel`).

## Topic contract

See [TOPIC_CONTRACT.md](TOPIC_CONTRACT.md). Every topic the sim publishes
mirrors the real hardware contract in [agv-greenhouse/specs/interfaces.yaml](https://github.com/AndresIslas99/agv-greenhouse/blob/main/specs/interfaces.yaml).

## Physical parameters

Calibrated 2026-04-08 on the real robot:

| | Value | Source |
|---|---|---|
| wheel_radius | 0.0781 m | `agv_odrive/config/odrive_params.yaml` |
| track_width | 0.960 m | same |
| gear_ratio | 10.0 | same |
| camera_mount | `base_link → (0.70, 0, -0.055)` | `agv_geometry.yaml` |

These live in [src/agv_isaac_sim/urdf/agv_sim.urdf.xacro](src/agv_isaac_sim/urdf/agv_sim.urdf.xacro)
and [src/agv_sim_drive/config/drive_shaping_params.yaml](src/agv_sim_drive/config/drive_shaping_params.yaml).
**Do not change without re-measuring on the real robot.**

## Asset generation (optional)

```bash
python3 scripts/generate_textures.py            # PBR → agv_isaac_sim/assets/greenhouse_textures/
python3 scripts/generate_apriltag_textures.py   # tag36h11 PNGs → agv_isaac_sim/assets/tag_textures/
```

## Project layout

```
src/
├── agv_isaac_sim/
│   ├── launch/      isaac_sim, isaac_teleop, isaac_hil
│   ├── urdf/        agv_sim.urdf.xacro, agv_base.xacro, wheel.xacro, caster.xacro
│   ├── config/      isaac_bridge.yaml, physics_params.yaml, world_config.yaml, domain_randomization.yaml, zed2i_calibration.yaml
│   ├── assets/      greenhouse_textures/ (11 PBR), tag_textures/ (38 tag36h11), greenhouse_sky.hdr
│   ├── scripts/     build_greenhouse_usd.py, import_robot_usd.py, setup_omnigraph.py,
│   │                isaac_ros_bridge_node.py, sim_depth_noise.py, sim_motor_gate.py,
│   │                sim_global_odom.py, sim_domain_randomizer.py, generate_hdri.py,
│   │                validate_apriltag_detection.py, robot/, world/
│   ├── worlds/      greenhouse_simple.usd (generated)
│   ├── robot/       agv_sim.usd (generated)
│   └── extensions/  agv_diff_drive (Isaac kit extension)
│
└── agv_sim_drive/
    ├── src/sim_drive_shaping_node.cpp
    ├── config/drive_shaping_params.yaml
    └── launch/drive_shaping.launch.py
```
