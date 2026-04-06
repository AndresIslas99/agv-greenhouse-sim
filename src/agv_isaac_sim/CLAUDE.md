# agv_isaac_sim

NVIDIA Isaac Sim integration for the AGV greenhouse simulation.
Replaces Gazebo Harmonic with GPU-accelerated physics (PhysX 5) and
rendering (RTX) for the Lenovo Legion 7 Pro (RTX 4080, 12GB VRAM).

## Architecture

> Same rule as the rest of the sim: **body, not brain.**
> Isaac Sim provides physics + sensors. The Jetson brain drives.

## Key Files

- `scripts/build_greenhouse_usd.py` — Procedural USD world builder (run inside Isaac Sim)
- `scripts/import_robot_usd.py` — URDF→USD import with sensor/actuator setup
- `scripts/isaac_ros_bridge_node.py` — Clock publisher + topic remapping helper
- `launch/isaac_sim.launch.py` — Base launch (Isaac Sim + world + robot)
- `launch/isaac_teleop.launch.py` — + keyboard teleop
- `launch/isaac_external.launch.py` — HIL mode (Jetson drives over network)
- `launch/isaac_nav.launch.py` — + Nav2 stack
- `worlds/greenhouse_simple.usd` — Generated USD world
- `robot/agv_sim.usd` — Generated robot USD

## Usage

```bash
# Generate world and robot USD (one-time, run inside Isaac Sim Python)
cd src/agv_isaac_sim
isaacsim --exec scripts/build_greenhouse_usd.py
isaacsim --exec scripts/import_robot_usd.py

# Launch simulation
ros2 launch agv_isaac_sim isaac_teleop.launch.py
ros2 launch agv_isaac_sim isaac_nav.launch.py
ros2 launch agv_isaac_sim isaac_external.launch.py  # HIL
```

## Topic Parity

All topics match TOPIC_CONTRACT.md exactly — the Jetson brain requires
zero changes when switching from Gazebo to Isaac Sim.

## Prerequisites

- NVIDIA Isaac Sim 4.x (standalone installation)
- NVIDIA driver 535+
- RTX GPU with 8+ GB VRAM
