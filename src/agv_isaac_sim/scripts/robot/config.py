"""Robot configuration for Isaac Sim — aligned with brain agv-greenhouse.

Authoritative source for physical values (calibrated 2026-04-08):
  agv-greenhouse/src/agv_bringup/config/robot/agv_geometry.yaml
  agv-greenhouse/src/agv_odrive/config/odrive_params.yaml

Topic contract comes from:
  agv-greenhouse/src/agv_bringup/launch/agv_hil_full.launch.py header
  agv-greenhouse/specs/interfaces.yaml
"""

import os

# Paths
SCRIPT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PACKAGE_DIR = os.path.dirname(SCRIPT_DIR)
SRC_DIR = os.path.dirname(PACKAGE_DIR)

# URDF lives inside agv_isaac_sim now (no separate description package)
URDF_DIR = os.path.join(PACKAGE_DIR, "urdf")
MAIN_XACRO = os.path.join(URDF_DIR, "agv_sim.urdf.xacro")
OUTPUT_PATH = os.path.join(PACKAGE_DIR, "robot", "agv_sim.usd")
TEMP_URDF = os.path.join(PACKAGE_DIR, "robot", "agv_sim_processed.urdf")

# Robot kinematics — calibrated values from agv_odrive/config/odrive_params.yaml
WHEEL_RADIUS = 0.0781        # meters (diameter 156.2mm) — calibrated 2026-04-08
WHEEL_SEPARATION = 0.960     # meters — effective track width (calibrated)
GEAR_RATIO = 10.0            # motor turns / wheel turns (10:1 planetary)

# ZED 2i camera mount — base_link → zed_camera_link
# Source: agv_geometry.yaml#camera_mount
CAMERA_MOUNT_XYZ = (0.70, 0.0, -0.055)
CAMERA_MOUNT_RPY = (0.0, 0.0, 0.0)
CAMERA_STEREO_BASELINE = 0.12  # 120mm

# Chassis (for Isaac physics)
CHASSIS_MASS = 30.0

# Publish rates (Hz) — match the real hardware drivers
ODOM_RATE = 50               # agv_odrive publishes wheel_odom at 50 Hz
JOINT_STATE_RATE = 50        # agv_odrive publishes joint_states at 50 Hz
CAMERA_RATE = 15             # ZED SDK default ROS publish rate
IMU_RATE = 200               # ZED 2i BMI088 publishes at 200 Hz

# ZED 2i camera specs — 1280×720 matches real ZED 2i output
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
CAMERA_HFOV_DEG = 110.0   # 1.9199 rad
CAMERA_CLIP_NEAR = 0.3    # hardware blind zone from agv_geometry.yaml
CAMERA_CLIP_FAR = 20.0

# Camera exposure (fixed, matched to greenhouse lighting)
CAMERA_AUTO_EXPOSURE = False
CAMERA_ISO = 200.0
CAMERA_SHUTTER_SPEED = 60.0
CAMERA_MOTION_BLUR = False    # ZED 2i has global shutter

# IMU noise — BMI088 datasheet typical
IMU_GYRO_NOISE = 0.00279   # rad/s stddev
IMU_ACCEL_NOISE = 0.0314   # m/s² stddev

# Depth camera noise model (ZED 2i calibrated)
DEPTH_NOISE_BASE = 0.002    # meters (noise floor)
DEPTH_NOISE_SCALE = 0.005   # meters per meter of distance

# ROS 2 topic names — must match brain's interfaces.yaml
#
# Note: the brain's HIL launch (agv_hil_full.launch.py) expects the IMU on
# /agv/imu/data (simple namespace), not /agv/zed/imu/data. We publish on
# /agv/imu/data for HIL parity. The camera topics use /agv/zed/* per the
# production interfaces.yaml contract.
TOPICS = {
    "cmd_vel":           "/agv/shaped_cmd_vel",
    "wheel_odom":        "/agv/wheel_odom",
    "joint_states":      "/agv/joint_states",
    "imu":               "/agv/imu/data",
    "left_rgb":          "/agv/zed/left/image_rect_color",
    "left_depth":        "/agv/zed/depth/depth_registered",
    "left_camera_info":  "/agv/zed/left/camera_info",
    "left_pointcloud":   "/agv/zed/point_cloud/cloud_registered",
    "right_rgb":         "/agv/zed/right/image_rect_color",
    "right_camera_info": "/agv/zed/right/camera_info",
    "tf":                "/tf",
}
