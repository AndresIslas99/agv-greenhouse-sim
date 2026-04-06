#!/usr/bin/env python3
"""Import AGV URDF into Isaac Sim and configure sensors + actuators.

Run inside Isaac Sim standalone Python:
    isaacsim --exec import_robot_usd.py

This script:
1. Processes the xacro URDF to plain URDF
2. Imports the URDF into a USD stage via Isaac Sim URDF Importer
3. Configures the diff-drive controller (DifferentialController)
4. Adds odometry publisher (wheel encoders)
5. Adds joint state publisher
6. Configures ZED 2i stereo cameras (left depth+RGB, right RGB)
7. Configures IMU sensor
8. Sets physics materials (wheels, casters)
9. Saves the robot as a standalone USD asset

The resulting USD can be loaded into the greenhouse world USD.
"""

import os
import subprocess
import math

# Isaac Sim imports
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.usd
import omni.kit.commands
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf, PhysxSchema

# Isaac Sim extensions
import omni.isaac.core.utils.stage as stage_utils
from omni.isaac.urdf import _urdf as urdf_importer

# Paths
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.dirname(SCRIPT_DIR)
SIM_ROOT = os.path.dirname(PACKAGE_DIR)
DESCRIPTION_PKG = os.path.join(SIM_ROOT, "agv_sim_description")
MAIN_XACRO = os.path.join(DESCRIPTION_PKG, "urdf", "agv_sim.urdf.xacro")
OUTPUT_PATH = os.path.join(PACKAGE_DIR, "robot", "agv_sim.usd")
TEMP_URDF = os.path.join(PACKAGE_DIR, "robot", "agv_sim_processed.urdf")

# Robot parameters (from robot_params.yaml)
WHEEL_RADIUS = 0.0625
WHEEL_SEPARATION = 0.735
ODOM_RATE = 50  # Hz
JOINT_STATE_RATE = 50  # Hz
CAMERA_RATE = 15  # Hz
IMU_RATE = 100  # Hz

# Camera parameters (ZED 2i specs)
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
CAMERA_HFOV = 110.0  # degrees (1.9199 rad)

# IMU noise (matching Gazebo config)
IMU_GYRO_NOISE = 0.00279   # rad/s stddev
IMU_ACCEL_NOISE = 0.0314   # m/s² stddev

# Topic names (matching TOPIC_CONTRACT.md)
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
}


def process_xacro():
    """Run xacro to produce a plain URDF file."""
    print(f"Processing xacro: {MAIN_XACRO}")
    result = subprocess.run(
        ["xacro", MAIN_XACRO],
        capture_output=True, text=True, timeout=30)

    if result.returncode != 0:
        raise RuntimeError(f"xacro failed: {result.stderr}")

    with open(TEMP_URDF, "w") as f:
        f.write(result.stdout)

    print(f"URDF written to: {TEMP_URDF}")
    return TEMP_URDF


def import_urdf(urdf_path):
    """Import URDF into the current USD stage."""
    print("Importing URDF into Isaac Sim...")

    # Configure URDF import settings
    import_config = urdf_importer.ImportConfig()
    import_config.merge_fixed_joints = False  # Keep all frames for TF
    import_config.fix_base = False  # Robot should be free to move
    import_config.import_inertia_tensor = True
    import_config.default_drive_type = 1  # Velocity drive for wheels
    import_config.default_drive_strength = 1e4
    import_config.default_position_drive_damping = 1e3
    import_config.create_physics_scene = False  # World has its own

    # Import
    result = urdf_importer.import_robot(
        urdf_path,
        import_config,
        "/Robot")

    if not result:
        raise RuntimeError("URDF import failed")

    print("URDF imported as /Robot")
    return "/Robot"


def setup_diff_drive_controller(stage, robot_path):
    """Configure the differential drive controller via OmniGraph.

    Creates an action graph that:
    - Subscribes to /agv/shaped_cmd_vel (Twist)
    - Computes per-wheel velocities using diff-drive kinematics
    - Drives left_wheel_joint and right_wheel_joint
    """
    print("Setting up differential drive controller...")

    # Use OmniGraph to create the controller
    # The exact API depends on Isaac Sim version; this uses the standard pattern
    import omni.graph.core as og

    keys = og.Controller.Keys

    (graph, nodes, _, _) = og.Controller.edit(
        {"graph_path": f"{robot_path}/DiffDriveController", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("SubscribeTwist", "omni.isaac.ros2_bridge.ROS2SubscribeTwist"),
                ("DiffController", "omni.isaac.wheeled_robots.DifferentialController"),
                ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
            ],
            keys.SET_VALUES: [
                ("SubscribeTwist.inputs:topicName", TOPICS["cmd_vel"]),
                ("DiffController.inputs:wheelRadius", WHEEL_RADIUS),
                ("DiffController.inputs:wheelDistance", WHEEL_SEPARATION),
                ("ArticulationController.inputs:robotPath", robot_path),
                ("ArticulationController.inputs:jointNames", ["left_wheel_joint", "right_wheel_joint"]),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
                ("SubscribeTwist.outputs:execOut", "DiffController.inputs:execIn"),
                ("SubscribeTwist.outputs:linearVelocity", "DiffController.inputs:linearVelocity"),
                ("SubscribeTwist.outputs:angularVelocity", "DiffController.inputs:angularVelocity"),
                ("DiffController.outputs:execOut", "ArticulationController.inputs:execIn"),
                ("DiffController.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
            ],
        }
    )
    print("  Diff-drive controller graph created")
    return graph


def setup_odometry_publisher(stage, robot_path):
    """Publish wheel odometry on /agv/wheel_odom + TF odom→base_link."""
    print("Setting up odometry publisher...")

    import omni.graph.core as og
    keys = og.Controller.Keys

    (graph, nodes, _, _) = og.Controller.edit(
        {"graph_path": f"{robot_path}/OdometryPublisher", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ComputeOdom", "omni.isaac.core_nodes.IsaacComputeOdometry"),
                ("PublishOdom", "omni.isaac.ros2_bridge.ROS2PublishOdometry"),
                ("PublishTF", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
            ],
            keys.SET_VALUES: [
                ("ComputeOdom.inputs:chassisPrim", robot_path),
                ("PublishOdom.inputs:topicName", TOPICS["wheel_odom"]),
                ("PublishOdom.inputs:odomFrameId", "odom"),
                ("PublishOdom.inputs:chassisFrameId", "base_link"),
                ("PublishTF.inputs:topicName", "/tf"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "ComputeOdom.inputs:execIn"),
                ("ComputeOdom.outputs:execOut", "PublishOdom.inputs:execIn"),
                ("ComputeOdom.outputs:position", "PublishOdom.inputs:position"),
                ("ComputeOdom.outputs:orientation", "PublishOdom.inputs:orientation"),
                ("ComputeOdom.outputs:linearVelocity", "PublishOdom.inputs:linearVelocity"),
                ("ComputeOdom.outputs:angularVelocity", "PublishOdom.inputs:angularVelocity"),
                ("PublishOdom.outputs:execOut", "PublishTF.inputs:execIn"),
            ],
        }
    )
    print(f"  Publishing to {TOPICS['wheel_odom']}")
    return graph


def setup_joint_state_publisher(stage, robot_path):
    """Publish joint states on /agv/joint_states."""
    print("Setting up joint state publisher...")

    import omni.graph.core as og
    keys = og.Controller.Keys

    (graph, nodes, _, _) = og.Controller.edit(
        {"graph_path": f"{robot_path}/JointStatePublisher", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadJoints", "omni.isaac.core_nodes.IsaacReadJointState"),
                ("PublishJointStates", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
            ],
            keys.SET_VALUES: [
                ("ReadJoints.inputs:prim", robot_path),
                ("PublishJointStates.inputs:topicName", TOPICS["joint_states"]),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "ReadJoints.inputs:execIn"),
                ("ReadJoints.outputs:execOut", "PublishJointStates.inputs:execIn"),
                ("ReadJoints.outputs:jointNames", "PublishJointStates.inputs:jointNames"),
                ("ReadJoints.outputs:positionCommand", "PublishJointStates.inputs:position"),
                ("ReadJoints.outputs:velocityCommand", "PublishJointStates.inputs:velocity"),
            ],
        }
    )
    print(f"  Publishing to {TOPICS['joint_states']}")
    return graph


def setup_camera(stage, robot_path, cam_name, parent_link, frame_id, topics, enable_depth=False):
    """Set up an Isaac Sim camera sensor with ROS 2 publishers."""
    print(f"Setting up camera: {cam_name} on {parent_link}...")

    import omni.graph.core as og
    keys = og.Controller.Keys

    cam_prim_path = f"{robot_path}/{parent_link}/Camera"

    # Create camera prim
    camera = UsdGeom.Camera.Define(stage, cam_prim_path)
    # Convert hFOV to focal length (for 1280px width sensor)
    hfov_rad = math.radians(CAMERA_HFOV)
    focal_length = (CAMERA_WIDTH / 2.0) / math.tan(hfov_rad / 2.0)
    # USD uses mm for focal length with 36mm horizontal aperture default
    camera.CreateFocalLengthAttr(focal_length * 36.0 / CAMERA_WIDTH)
    camera.CreateHorizontalApertureAttr(36.0)
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.3, 20.0))

    # Create OmniGraph for ROS 2 publishing
    node_list = [
        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
        ("IsaacCreateRenderProduct", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
        ("PublishRGB", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
    ]
    set_values = [
        ("IsaacCreateRenderProduct.inputs:cameraPrim", cam_prim_path),
        ("IsaacCreateRenderProduct.inputs:width", CAMERA_WIDTH),
        ("IsaacCreateRenderProduct.inputs:height", CAMERA_HEIGHT),
        ("PublishRGB.inputs:topicName", topics["rgb"]),
        ("PublishRGB.inputs:frameId", frame_id),
        ("PublishRGB.inputs:type", "rgb"),
    ]
    connections = [
        ("OnPlaybackTick.outputs:tick", "IsaacCreateRenderProduct.inputs:execIn"),
        ("IsaacCreateRenderProduct.outputs:execOut", "PublishRGB.inputs:execIn"),
        ("IsaacCreateRenderProduct.outputs:renderProductPath", "PublishRGB.inputs:renderProductPath"),
    ]

    # Camera info publisher
    node_list.append(("PublishCameraInfo", "omni.isaac.ros2_bridge.ROS2CameraHelper"))
    set_values.extend([
        ("PublishCameraInfo.inputs:topicName", topics["camera_info"]),
        ("PublishCameraInfo.inputs:frameId", frame_id),
        ("PublishCameraInfo.inputs:type", "camera_info"),
    ])
    connections.extend([
        ("PublishRGB.outputs:execOut", "PublishCameraInfo.inputs:execIn"),
        ("IsaacCreateRenderProduct.outputs:renderProductPath", "PublishCameraInfo.inputs:renderProductPath"),
    ])

    prev_node = "PublishCameraInfo"

    if enable_depth:
        # Depth publisher
        node_list.append(("PublishDepth", "omni.isaac.ros2_bridge.ROS2CameraHelper"))
        set_values.extend([
            ("PublishDepth.inputs:topicName", topics["depth"]),
            ("PublishDepth.inputs:frameId", frame_id),
            ("PublishDepth.inputs:type", "depth"),
        ])
        connections.extend([
            (f"{prev_node}.outputs:execOut", "PublishDepth.inputs:execIn"),
            ("IsaacCreateRenderProduct.outputs:renderProductPath", "PublishDepth.inputs:renderProductPath"),
        ])
        prev_node = "PublishDepth"

        # Point cloud publisher
        if "pointcloud" in topics:
            node_list.append(("PublishPC", "omni.isaac.ros2_bridge.ROS2CameraHelper"))
            set_values.extend([
                ("PublishPC.inputs:topicName", topics["pointcloud"]),
                ("PublishPC.inputs:frameId", frame_id),
                ("PublishPC.inputs:type", "depth_pcl"),
            ])
            connections.extend([
                (f"{prev_node}.outputs:execOut", "PublishPC.inputs:execIn"),
                ("IsaacCreateRenderProduct.outputs:renderProductPath", "PublishPC.inputs:renderProductPath"),
            ])

    (graph, nodes, _, _) = og.Controller.edit(
        {"graph_path": f"{robot_path}/{cam_name}Graph", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: node_list,
            keys.SET_VALUES: set_values,
            keys.CONNECT: connections,
        }
    )

    for topic_name, topic_value in topics.items():
        print(f"  {topic_name}: {topic_value}")

    return graph


def setup_imu(stage, robot_path):
    """Set up IMU sensor with ROS 2 publisher."""
    print("Setting up IMU sensor...")

    import omni.graph.core as og
    keys = og.Controller.Keys

    imu_prim_path = f"{robot_path}/imu_link"

    (graph, nodes, _, _) = og.Controller.edit(
        {"graph_path": f"{robot_path}/IMUGraph", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("IsaacReadIMU", "omni.isaac.sensor.IsaacReadIMU"),
                ("PublishIMU", "omni.isaac.ros2_bridge.ROS2PublishImu"),
            ],
            keys.SET_VALUES: [
                ("IsaacReadIMU.inputs:imuPrim", imu_prim_path),
                ("PublishIMU.inputs:topicName", TOPICS["imu"]),
                ("PublishIMU.inputs:frameId", "imu_link"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "IsaacReadIMU.inputs:execIn"),
                ("IsaacReadIMU.outputs:execOut", "PublishIMU.inputs:execIn"),
                ("IsaacReadIMU.outputs:angVel", "PublishIMU.inputs:angularVelocity"),
                ("IsaacReadIMU.outputs:linAcc", "PublishIMU.inputs:linearAcceleration"),
                ("IsaacReadIMU.outputs:orientation", "PublishIMU.inputs:orientation"),
            ],
        }
    )
    print(f"  Publishing to {TOPICS['imu']}")
    return graph


def setup_physics_materials(stage, robot_path):
    """Apply friction materials to wheels and casters."""
    print("Setting up physics materials...")

    # Wheel material (high friction)
    wheel_mat = UsdPhysics.MaterialAPI.Apply(
        UsdShade.Material.Define(stage, f"{robot_path}/Materials/WheelMaterial").GetPrim())
    wheel_mat.CreateStaticFrictionAttr(1.0)
    wheel_mat.CreateDynamicFrictionAttr(1.0)
    wheel_mat.CreateRestitutionAttr(0.0)

    # Caster material (low friction)
    caster_mat = UsdPhysics.MaterialAPI.Apply(
        UsdShade.Material.Define(stage, f"{robot_path}/Materials/CasterMaterial").GetPrim())
    caster_mat.CreateStaticFrictionAttr(0.001)
    caster_mat.CreateDynamicFrictionAttr(0.001)
    caster_mat.CreateRestitutionAttr(0.0)

    # Bind materials to collision geometry
    for wheel in ["left_wheel", "right_wheel"]:
        wheel_prim = stage.GetPrimAtPath(f"{robot_path}/{wheel}")
        if wheel_prim:
            PhysxSchema.PhysxMaterialAPI.Apply(wheel_prim)

    for caster in ["front_caster", "rear_caster"]:
        caster_prim = stage.GetPrimAtPath(f"{robot_path}/{caster}")
        if caster_prim:
            PhysxSchema.PhysxMaterialAPI.Apply(caster_prim)

    print("  Wheels: friction=1.0, Casters: friction=0.001")


def main():
    # Step 1: Process xacro
    urdf_path = process_xacro()

    # Step 2: Create stage and import URDF
    stage = omni.usd.get_context().get_stage()
    robot_path = import_urdf(urdf_path)

    # Step 3: Configure actuators and sensors
    setup_diff_drive_controller(stage, robot_path)
    setup_odometry_publisher(stage, robot_path)
    setup_joint_state_publisher(stage, robot_path)

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
    from pxr import UsdShade
    setup_physics_materials(stage, robot_path)

    # Step 4: Save
    stage.GetRootLayer().Export(OUTPUT_PATH)
    print(f"\nRobot USD saved to: {OUTPUT_PATH}")

    # Cleanup temp URDF
    if os.path.exists(TEMP_URDF):
        os.remove(TEMP_URDF)

    simulation_app.close()


if __name__ == "__main__":
    main()
