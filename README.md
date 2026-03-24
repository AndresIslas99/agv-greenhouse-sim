# AGV Greenhouse Simulation

Gazebo Classic simulation for the AGV greenhouse robot. A **development bench** that lets the software team advance Nav2, EKF wiring, AprilTags, mission flow, and repeatable tests while the mechanical team works on the physical AGV.

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

## Build

```bash
cd ~/agv-greenhouse-sim
colcon build --symlink-install
source install/setup.bash
```

## Packages

| Package | Description |
|---------|-------------|
| `agv_sim_description` | Simulation URDF with Gazebo plugins (diff-drive, IMU, lidar) |
| `agv_sim_worlds` | Gazebo world files: greenhouse and nav_test environments |
| `agv_sim_bringup` | Launch files for all simulation modes |
| `agv_sim_nav` | Nav2 configuration (SmacPlannerHybrid + RegulatedPurePursuit) |
| `agv_sim_apriltags` | AprilTag models, markers registry, and fake proximity detector |

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
Complete Nav2 with localization and autonomous A-to-B navigation.

```bash
ros2 launch agv_sim_bringup sim_nav.launch.py map:=~/greenhouse_map
```

Send a Nav2 goal:
```bash
ros2 action send_goal /agv/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 8.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

### 5. Fake AprilTag Detector
Run alongside any mode to simulate marker detection:

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

## Configuration Parity with Real Robot

| Parameter | Value |
|-----------|-------|
| Namespace | `agv` |
| Wheel radius | 0.0625 m |
| Track width | 0.735 m |
| Chassis | 1.0 x 0.6 x 0.15 m |
| cmd_vel topic | `/agv/cmd_vel` |
| Wheel odom topic | `/agv/wheel_odom` |
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
           ├── imu_link       (fixed, at ZED position)
           ├── laser_frame    (fixed, front-mounted)
           ├── front_caster   (fixed)
           └── rear_caster    (fixed)
```

## Simulation Architecture

- **Teleop mode**: Gazebo diff_drive publishes odom->base_link TF directly
- **Mapping mode**: diff_drive publishes odom->base_link, slam_toolbox publishes map->odom
- **Fusion mode**: EKF local owns odom->base_link, EKF global owns map->odom (diff_drive TF disabled)
- **Nav mode**: Same as fusion + full Nav2 stack (planner, controller, behaviors)

## AprilTag Markers

6 AprilTag markers (tag36h11 family) are placed in the greenhouse world:

| Tag ID | Location | Position (x, y) |
|--------|----------|------------------|
| 0 | West corridor end | (1.0, 0.0) |
| 1 | East corridor end | (29.0, 0.0) |
| 2 | Aisle 1-2 entrance | (6.0, -4.4) |
| 3 | Aisle 3-4 entrance | (6.0, 0.0) |
| 4 | Aisle 5-6 entrance | (6.0, 4.4) |
| 5 | Starting area | (2.5, -5.0) |

The `fake_marker_detector` node publishes `PoseWithCovarianceStamped` on `/{ns}/marker_pose` when the robot is within 2m of any marker.

## Greenhouse World Layout

```
     0     5    10    15    20    25    30
  +--+-----+-----+-----+-----+-----+-----+
  |  |     | Row6 ========================|  7.5
  |  |     |     aisle (~1.2m)            |
  |  |     | Row5 ========================|
  |  |     |     aisle (~1.2m)            |
  |  | T3  | Row4 ========================|
  |  |     |     aisle (~1.2m)            |  0.0
  | T0     | Row3 ========================|
  |  |     |     aisle (~1.2m)            |
  |  | T2  | Row2 ========================|
  |  |     |     aisle (~1.2m)            |
  | T5     | Row1 ========================| -7.5
  +--+-----+-----+-----+-----+-----+-----+
   ^               ^                    ^
   Start area      Crop rows (20m)     T1
   (clear)
```

T0-T5 = AprilTag positions. Robot spawns at (2.0, 0.0) in the start area.
