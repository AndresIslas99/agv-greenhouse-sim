#!/usr/bin/env python3
"""Import AGV URDF into Isaac Sim and configure sensors + actuators.

Run inside Isaac Sim standalone Python:
    isaacsim --exec import_robot_usd.py

Steps:
1. Process xacro → plain URDF
2. Import URDF → USD via Isaac Sim URDF Importer
3. Configure diff-drive controller (OmniGraph)
4. Configure odometry publisher (50Hz)
5. Configure joint state publisher (50Hz)
6. Configure ZED 2i stereo cameras (left: depth+RGB, right: RGB)
7. Configure IMU sensor (100Hz)
8. Apply physics materials (wheel/caster friction)
9. Save robot USD
"""

import os

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.usd

from robot.config import TOPICS, OUTPUT_PATH, TEMP_URDF
from robot.urdf_import import process_xacro, import_urdf
from robot.diff_drive import setup_diff_drive_controller, setup_odometry_publisher
from robot.joint_states import setup_joint_state_publisher
from robot.sensors import setup_camera, setup_imu
from robot.physics_materials import setup_physics_materials


def main():
    urdf_path = process_xacro()
    stage = omni.usd.get_context().get_stage()
    robot_path = import_urdf(urdf_path)

    # Actuators
    setup_diff_drive_controller(robot_path)
    setup_odometry_publisher(robot_path)
    setup_joint_state_publisher(robot_path)

    # Left camera (depth + RGB + point cloud)
    setup_camera(stage, robot_path,
                 cam_name="ZedLeftCamera",
                 parent_link="zed_left_camera_frame",
                 frame_id="zed_left_camera_frame_optical",
                 topics={
                     "rgb": TOPICS["left_rgb"],
                     "depth": TOPICS["left_depth"],
                     "camera_info": TOPICS["left_camera_info"],
                     "pointcloud": TOPICS["left_pointcloud"],
                 },
                 enable_depth=True)

    # Right camera (RGB only)
    setup_camera(stage, robot_path,
                 cam_name="ZedRightCamera",
                 parent_link="zed_right_camera_frame",
                 frame_id="zed_right_camera_frame_optical",
                 topics={
                     "rgb": TOPICS["right_rgb"],
                     "camera_info": TOPICS["right_camera_info"],
                 },
                 enable_depth=False)

    # IMU
    setup_imu(stage, robot_path)

    # Physics materials
    setup_physics_materials(stage, robot_path)

    # Save
    stage.GetRootLayer().Export(OUTPUT_PATH)
    print(f"\nRobot USD saved to: {OUTPUT_PATH}")

    if os.path.exists(TEMP_URDF):
        os.remove(TEMP_URDF)

    simulation_app.close()


if __name__ == "__main__":
    main()
