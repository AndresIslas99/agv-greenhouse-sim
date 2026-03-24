# AGV Greenhouse Simulation

Gazebo Classic simulation for the AGV greenhouse robot. An **accuracy-focused simulation** that replicates the real hardware's ZED 2i stereo camera, ODrive motor shaping, real AprilTag textures, and greenhouse heating pipe rails — letting the software team develop and test Nav2, EKF, vision, and mission logic with high-fidelity sensor and actuator behavior.

## Prerequisites

- **ROS 2 Humble Hawksbill**
- **Gazebo Classic (gazebo11)** with `gazebo_ros` packages
- Required ROS 2 packages:

```bash
sudo apt install \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-xacro \
  ros-humble-robot-localization \
  ros-humble-slam-toolbox \
  ros-humble-nav2-bringup \
  ros-humble-nav2-smac-planner \
  ros-humble-nav2-regulated-pure-pursuit-controller \
  ros-humble-nav2-collision-monitor \
  ros-humble-teleop-twist-keyboard \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs
```

Optional (for camera-based AprilTag detection):
```bash
sudo apt install ros-humble-apriltag-ros
```

## Build

```bash
cd ~/agv-greenhouse-sim
colcon build --symlink-install
source install/setup.bash
```

## Packages

| Package | Description |
|---------|-------------|
| `agv_sim_description` | Simulation URDF with Gazebo plugins (diff-drive, ZED 2i stereo camera, IMU, lidar) |
| `agv_sim_worlds` | Gazebo world files: greenhouse (with heating pipe rails) and nav_test environments |
| `agv_sim_bringup` | Launch files for all simulation modes |
| `agv_sim_nav` | Nav2 configuration (SmacPlannerHybrid + RegulatedPurePursuit) |
| `agv_sim_apriltags` | Real tag36h11 textures, apriltag_ros config, and fake proximity detector |
| `agv_sim_drive` | ODrive-realistic wheel shaping node (C++17) |

## Launch Modes

### 1. Teleop Mode
Drive the robot manually in the greenhouse world.

```bash
ros2 launch agv_sim_bringup sim_teleop.launch.py
```

In another terminal, drive with keyboard:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/agv/cmd_vel
```

### 2. Mapping Mode
Create a map of the greenhouse using slam_toolbox.

```bash
ros2 launch agv_sim_bringup sim_mapping.launch.py
```

Drive the robot around, then save the map:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/greenhouse_map --ros-args -r map:=/agv/map
```

### 3. Fusion Mode
Test dual EKF sensor fusion with slam_toolbox localization.

```bash
ros2 launch agv_sim_bringup sim_fusion.launch.py map:=~/greenhouse_map
```

### 4. Navigation Mode (Full Stack)
Complete Nav2 with localization, autonomous A-to-B navigation, and apriltag_ros detection.

```bash
ros2 launch agv_sim_bringup sim_nav.launch.py map:=~/greenhouse_map
```

Send a Nav2 goal:
```bash
ros2 action send_goal /agv/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 8.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

### 5. AprilTag Detection
Camera-based detection using apriltag_ros (run alongside any mode):

```bash
ros2 launch agv_sim_bringup sim_apriltag.launch.py
```

Or use the fake proximity-based detector:
```bash
ros2 launch agv_sim_apriltags fake_markers.launch.py
```

### World Selection
Use the nav_test world for simple A-to-B testing:

```bash
ros2 launch agv_sim_bringup sim_teleop.launch.py world:=nav_test
```

### Custom Spawn Position
```bash
ros2 launch agv_sim_bringup sim_teleop.launch.py x:=5.0 y:=-3.0 yaw:=1.57
```

## ZED 2i Stereo Camera Simulation

The simulation replicates the real ZED 2i camera with matching specs:

| Parameter | Value |
|-----------|-------|
| Baseline | 120mm (0.12m) |
| FOV | 110° H × 70° V |
| Resolution | 1280×720 @30fps |
| Depth range | 0.3m – 20.0m |
| IMU rate | 400 Hz |
| IMU gyro noise | 0.00279 rad/s |
| IMU accel noise | 0.0314 m/s² |

### Camera Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/zed/zed_node/left/image_rect_color` | sensor_msgs/Image | Left camera RGB |
| `/zed/zed_node/left/camera_info` | sensor_msgs/CameraInfo | Left camera intrinsics |
| `/zed/zed_node/right/image_rect_color` | sensor_msgs/Image | Right camera RGB |
| `/zed/zed_node/right/camera_info` | sensor_msgs/CameraInfo | Right camera intrinsics |
| `/zed/zed_node/depth/depth_registered` | sensor_msgs/Image | Depth image |
| `/zed/zed_node/point_cloud/cloud_registered` | sensor_msgs/PointCloud2 | 3D point cloud |
| `/zed/zed_node/imu/data` | sensor_msgs/Imu | IMU (400Hz) |

## ODrive-Realistic Drive Shaping

The `sim_drive_shaping_node` replicates the real ODrive CAN node's wheel command pipeline:

```
cmd_vel → [inverse kinematics] → [per-wheel shaping] → shaped_cmd_vel → Gazebo diff_drive
```

**Shaping stages per wheel:**
1. **Zero-velocity epsilon** (0.03 turns/s): Below this threshold, send exact zero — prevents creep
2. **Acceleration limiter** (1.0 turns/s²): Rate-limits velocity changes per timestep
3. **Min effective velocity**: Stiction compensation (disabled by default)
4. **Per-side inversion and scaling**: Matches real motor wiring

**Parameters** (from `drive_shaping_params.yaml`):

| Parameter | Value | Description |
|-----------|-------|-------------|
| `wheel_radius` | 0.0625 m | 125mm diameter wheels |
| `track_width` | 0.735 m | Center-to-center |
| `publish_rate_hz` | 50 | Loop rate |
| `cmd_vel_timeout_ms` | 500 | Safety timeout |
| `zero_vel_epsilon` | 0.03 turns/s | Dead zone |
| `max_wheel_accel` | 1.0 turns/s² | Accel limit |

## AprilTag Detection

### Real Textures (apriltag_ros)
The simulation uses real tag36h11 PNG textures on the marker models. When `apriltag_ros` is installed, camera-based detection works:

```bash
# Launches automatically in sim_nav mode, or standalone:
ros2 launch agv_sim_bringup sim_apriltag.launch.py
```

Detections published on `/agv/tag_detections`.

### Fake Proximity Detector (fallback)
When apriltag_ros is not available or the camera is not active, the fake detector publishes poses based on TF proximity:

```bash
ros2 launch agv_sim_apriltags fake_markers.launch.py
```

Publishes `PoseWithCovarianceStamped` on `/{ns}/marker_pose` when the robot is within 2m of any marker.

### Tag Texture Generation
To regenerate the tag textures:
```bash
python3 scripts/generate_apriltag_textures.py
```

## Greenhouse World Layout

The world includes 6 crop rows, 5 aisles with heating pipe rails, walls, crates, and 6 AprilTag markers.

```
     0     5    10    15    20    25    30
  +--+-----+-----+-----+-----+-----+-----+
  |  |     | Row6 ========================|  7.5
  |  |     | ═══ heating rails ═══        |
  |  |     | Row5 ========================|
  |  |     | ═══ heating rails ═══        |
  |  | T3  | Row4 ========================|
  |  |     | ═══ heating rails ═══        |  0.0
  | T0     | Row3 ========================|
  |  |     | ═══ heating rails ═══        |
  |  | T2  | Row2 ========================|
  |  |     | ═══ heating rails ═══        |
  | T5     | Row1 ========================| -7.5
  +--+-----+-----+-----+-----+-----+-----+
   ^               ^                    ^
   Start area      Crop rows (20m)     T1
   (clear)
```

**Heating pipe rails:** 2 parallel 51mm steel pipes per aisle, 450mm apart, 20m long, at ground level.

T0-T5 = AprilTag positions. Robot spawns at (2.0, 0.0) in the start area.

## Configuration Parity with Real Robot

| Parameter | Value |
|-----------|-------|
| Namespace | `agv` |
| Wheel radius | 0.0625 m |
| Track width | 0.735 m |
| Chassis | 1.0 x 0.6 x 0.15 m |
| cmd_vel topic | `/agv/cmd_vel` |
| Shaped cmd_vel topic | `/agv/shaped_cmd_vel` |
| Wheel odom topic | `/agv/wheel_odom` |
| IMU topic | `/zed/zed_node/imu/data` |
| Odom frame | `odom` |
| Base frame | `base_link` |
| Map frame | `map` |
| Publish rate | 50 Hz |
| Left wheel joint | `left_wheel_joint` |
| Right wheel joint | `right_wheel_joint` |

## TF Tree

```
map
 └── odom          (global EKF in fusion/nav modes, slam_toolbox in mapping mode)
      └── base_link    (local EKF in fusion/nav modes, diff_drive in teleop/mapping)
           ├── left_wheel    (continuous, driven by Gazebo diff_drive)
           ├── right_wheel   (continuous, driven by Gazebo diff_drive)
           ├── base_footprint (fixed, -0.2m Z)
           ├── zed_camera_link (fixed, at ZED 2i position)
           │    ├── zed_camera_center
           │    ├── zed_left_camera_frame
           │    │    └── zed_left_camera_optical_frame
           │    ├── zed_right_camera_frame
           │    │    └── zed_right_camera_optical_frame
           │    └── imu_link
           ├── laser_frame    (fixed, front-mounted)
           ├── front_caster   (fixed)
           └── rear_caster    (fixed)
```

## Simulation Architecture

- **Drive shaping**: All modes include the ODrive-realistic shaping node (`cmd_vel` → `shaped_cmd_vel`)
- **Teleop mode**: Gazebo diff_drive publishes odom->base_link TF directly
- **Mapping mode**: diff_drive publishes odom->base_link, slam_toolbox publishes map->odom
- **Fusion mode**: EKF local owns odom->base_link, EKF global owns map->odom (diff_drive TF disabled)
- **Nav mode**: Same as fusion + full Nav2 stack + apriltag_ros detection

## AprilTag Markers

6 AprilTag markers (tag36h11 family) with real textures are placed in the greenhouse world:

| Tag ID | Location | Position (x, y) |
|--------|----------|------------------|
| 0 | West corridor end | (1.0, 0.0) |
| 1 | East corridor end | (29.0, 0.0) |
| 2 | Aisle 1-2 entrance | (6.0, -4.4) |
| 3 | Aisle 3-4 entrance | (6.0, 0.0) |
| 4 | Aisle 5-6 entrance | (6.0, 4.4) |
| 5 | Starting area | (2.5, -5.0) |
