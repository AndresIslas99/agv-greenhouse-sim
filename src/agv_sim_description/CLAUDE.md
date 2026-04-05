# agv_sim_description

Robot URDF/xacro model for the AGV greenhouse simulation.

## Key Files

- `urdf/agv_sim.urdf.xacro` — Main entry point, includes base + sensors
- `urdf/agv_base_sim.xacro` — Chassis, casters, diff_drive plugin
- `urdf/sensors_sim.xacro` — ZED 2i stereo camera, IMU (400Hz), gpu_lidar
- `urdf/wheel_sim.xacro` — Wheel geometry macro
- `config/robot_params.yaml` — Parametric dimensions

## Rules

- Frame names MUST match the real ZED 2i SDK frame hierarchy
- Sensor rates must match real hardware (IMU 400Hz, camera 30fps)
- All visual meshes need PBR materials for ZED depth to work in sim
- One xacro file per subsystem — do not merge base/sensors/wheels
