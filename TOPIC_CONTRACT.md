# PC-Jetson Topic Contract

When running `sim_external.launch.py`, the PC publishes simulated sensor data and the Jetson consumes it as if it were real hardware.

## PC Publishes (Simulation -> Network)

### Lightweight (always active, WiFi-safe)

| Topic | Type | Rate | ~BW | Source |
|-------|------|------|-----|--------|
| `/clock` | rosgraph_msgs/Clock | ~1000Hz | 16 KB/s | Gz sim time |
| `/agv/wheel_odom` | nav_msgs/Odometry | 50Hz | 15 KB/s | Gz DiffDrive plugin |
| `/zed/zed_node/imu/data` | sensor_msgs/Imu | 300Hz | 60 KB/s | Gz IMU sensor |
| `/agv/scan` | sensor_msgs/LaserScan | 10Hz | 60 KB/s | Gz gpu_lidar (needs GPU) |
| `/agv/joint_states` | sensor_msgs/JointState | 50Hz | 10 KB/s | Gz JointStatePublisher |
| `/tf_static` | tf2_msgs/TFMessage | latched | ~1 KB | robot_state_publisher (URDF frames) |
| **Total** | | | **~160 KB/s** | |

### Heavy (lazy-bridged, only stream when subscribed)

| Topic | Type | Rate | ~BW | Note |
|-------|------|------|-----|------|
| `/zed/zed_node/left/image_rect_color` | sensor_msgs/Image | 30Hz | 81 MB/s | 1280x720 RGB |
| `/zed/zed_node/right/image_rect_color` | sensor_msgs/Image | 30Hz | 81 MB/s | 1280x720 RGB |
| `/zed/zed_node/left/camera_info` | sensor_msgs/CameraInfo | 30Hz | ~1 KB/s | Lightweight |
| `/zed/zed_node/right/camera_info` | sensor_msgs/CameraInfo | 30Hz | ~1 KB/s | Lightweight |
| `/zed/zed_node/depth/depth_registered` | sensor_msgs/Image | 30Hz | 108 MB/s | 1280x720 float32 |
| `/zed/zed_node/point_cloud/cloud_registered` | sensor_msgs/PointCloud2 | 30Hz | 330 MB/s | Dense cloud |

Camera/depth/pointcloud exceed WiFi bandwidth. Do NOT subscribe from Jetson over WiFi unless using wired Ethernet or compressed transport.

## Jetson Publishes (Brain -> PC)

| Topic | Type | Rate | ~BW | Purpose |
|-------|------|------|-----|---------|
| `/agv/cmd_vel` | geometry_msgs/Twist | 10-50Hz | ~1 KB/s | Drive commands (Nav2 or teleop) |
| `/agv/e_stop` | std_msgs/Bool | on event | ~1 B | Emergency stop |

The PC's drive_shaping_node subscribes to `/agv/cmd_vel`, shapes it, and feeds `/agv/shaped_cmd_vel` to the Gz DiffDrive plugin.

## Frame IDs (TF)

The PC publishes `/tf_static` with the URDF frame tree:
```
base_link -> left_wheel, right_wheel, base_footprint,
             zed_camera_link -> zed_camera_center,
                                zed_left_camera_frame -> zed_left_camera_optical_frame,
                                zed_right_camera_frame -> zed_right_camera_optical_frame,
                                imu_link
             laser_frame, front_caster, rear_caster
```

The Jetson owns the dynamic TF transforms:
- `odom -> base_link` (EKF local)
- `map -> odom` (EKF global)

## Topic Name Parity with Real Robot

| Sim Topic | Real Hardware Source | Match? |
|-----------|-------------------|--------|
| `/agv/wheel_odom` | agv_odrive (ODrive CAN) | Same name, same type |
| `/zed/zed_node/imu/data` | ZED 2i SDK | Same name, same type |
| `/agv/scan` | (no lidar on real robot yet) | Sim-only for now |
| `/agv/cmd_vel` | Nav2 / teleop_web | Same name, same type |
| `/agv/joint_states` | agv_odrive (wheel encoders) | Same name, same type |

The Jetson brain (`agv_fusion.launch.py`) can consume sim topics without any topic remapping.

## GPU-Dependent Topics

These topics require the Gz ogre2 renderer (GPU/EGL):
- `/agv/scan` (gpu_lidar sensor)
- All camera/depth/pointcloud topics

Without GPU rendering, only `/agv/wheel_odom`, `/zed/zed_node/imu/data`, `/agv/joint_states`, and `/clock` are available. This is sufficient for EKF development but not for Nav2 costmap or SLAM.

## use_sim_time

When the Jetson consumes sim topics, all Jetson nodes must set `use_sim_time: true` to synchronize with the sim `/clock`. Without this, message timestamps from the sim will appear in the past and be dropped by TF, EKF, and Nav2.
