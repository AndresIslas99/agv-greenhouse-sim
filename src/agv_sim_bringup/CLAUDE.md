# agv_sim_bringup

Launch files and configuration for 7 simulation modes.

## Key Files

- `launch/sim_teleop.launch.py` — Base mode: Gz + robot + keyboard
- `launch/sim_external.launch.py` — HIL mode: Gz + bridge, Jetson brain drives
- `launch/spawn_robot.launch.py` — Robot spawner (x, y, yaw params)
- `config/sim_ekf_local.yaml` — Local EKF (odom frame)
- `config/sim_ekf_global.yaml` — Global EKF (map frame)
- `config/gz_bridge.yaml` — Gazebo-ROS bridge topic mapping
- `scripts/sim_global_odom.py` — Simulated cuVSLAM output
- `scripts/sim_motor_gate.py` — ODrive arm/disarm emulation

## Rules

- Launch files compose incrementally (teleop < mapping < fusion < nav)
- EKF configs must match the real Jetson EKF configs in topic names
- sim_global_odom publishes to `/visual_slam/tracking/odometry` (matching cuVSLAM)
- sim_motor_gate publishes `motor_state` JSON matching real ODrive protocol
