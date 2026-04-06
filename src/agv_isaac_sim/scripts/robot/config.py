"""Robot configuration: dimensions, sensor specs, topic names."""

import os

# Paths
SCRIPT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PACKAGE_DIR = os.path.dirname(SCRIPT_DIR)
SRC_DIR = os.path.dirname(PACKAGE_DIR)
DESCRIPTION_PKG = os.path.join(SRC_DIR, "agv_sim_description")
MAIN_XACRO = os.path.join(DESCRIPTION_PKG, "urdf", "agv_sim.urdf.xacro")
OUTPUT_PATH = os.path.join(PACKAGE_DIR, "robot", "agv_sim.usd")
TEMP_URDF = os.path.join(PACKAGE_DIR, "robot", "agv_sim_processed.urdf")

# Robot kinematics (from robot_params.yaml)
WHEEL_RADIUS = 0.0625      # 125mm diameter
WHEEL_SEPARATION = 0.735   # 735mm track width

# Publish rates (Hz)
ODOM_RATE = 50
JOINT_STATE_RATE = 50
CAMERA_RATE = 30
IMU_RATE = 400

# ZED 2i camera specs
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
CAMERA_HFOV_DEG = 110.0   # 1.9199 rad
CAMERA_CLIP_NEAR = 0.3
CAMERA_CLIP_FAR = 20.0

# IMU noise (matching Gazebo sensors_sim.xacro, BMI055 datasheet)
IMU_GYRO_NOISE = 0.00279   # rad/s stddev
IMU_ACCEL_NOISE = 0.0314   # m/s² stddev

# Depth camera noise model (ZED 2i calibrated)
# Real ZED 2i: noise scales with distance squared
# σ_depth(d) = DEPTH_NOISE_BASE + DEPTH_NOISE_SCALE * d
# At 1m: ~7mm, at 5m: ~30mm, at 10m: ~55mm
DEPTH_NOISE_BASE = 0.002    # meters (minimum noise floor)
DEPTH_NOISE_SCALE = 0.005   # meters per meter of distance

# ROS 2 topic names (must match TOPIC_CONTRACT.md)
TOPICS = {
    "cmd_vel": "/agv/shaped_cmd_vel",
    "wheel_odom": "/agv/wheel_odom",
    "joint_states": "/agv/joint_states",
    "left_rgb": "/zed/zed_node/left/image_rect_color",
    "left_depth": "/zed/zed_node/depth/depth_registered",
    "left_camera_info": "/zed/zed_node/left/camera_info",
    "left_pointcloud": "/zed/zed_node/point_cloud/cloud_registered",
    "right_rgb": "/zed/zed_node/right/image_rect_color",
    "right_camera_info": "/zed/zed_node/right/camera_info",
    "imu": "/zed/zed_node/imu/data",
    "imu_alt": "/agv/imu/data",
    "tf": "/tf",
}
