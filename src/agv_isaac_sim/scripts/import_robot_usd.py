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
import omni.kit.app
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf, PhysxSchema

# Enable URDF importer extension (renamed in Isaac Sim 4.5)
ext_manager = omni.kit.app.get_app().get_extension_manager()
ext_manager.set_extension_enabled_immediate("isaacsim.asset.importer.urdf", True)

import isaacsim.core.utils.stage as stage_utils

# Paths — URDF lives inside agv_isaac_sim itself (no separate description package)
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.dirname(SCRIPT_DIR)
URDF_DIR = os.path.join(PACKAGE_DIR, "urdf")
MAIN_XACRO = os.path.join(URDF_DIR, "agv_sim.urdf.xacro")
OUTPUT_PATH = os.path.join(PACKAGE_DIR, "robot", "agv_sim.usd")
TEMP_URDF = os.path.join(PACKAGE_DIR, "robot", "agv_sim_processed.urdf")

# Robot parameters — calibrated 2026-04-08 (brain agv_odrive/config/odrive_params.yaml)
WHEEL_RADIUS = 0.0781        # meters, diameter 156.2mm
WHEEL_SEPARATION = 0.960     # meters, effective track width
GEAR_RATIO = 10.0
ODOM_RATE = 50               # Hz (matches real agv_odrive)
JOINT_STATE_RATE = 50        # Hz
CAMERA_RATE = 15             # Hz (ZED SDK)
IMU_RATE = 200               # Hz (BMI088 in ZED 2i)

# Camera mount — base_link → zed_camera_link (agv_geometry.yaml#camera_mount)
CAMERA_MOUNT_XYZ = (0.70, 0.0, -0.055)
CAMERA_STEREO_BASELINE = 0.12

# Camera parameters (ZED 2i specs)
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
CAMERA_HFOV = 110.0  # degrees (1.9199 rad)

# IMU noise — BMI088 datasheet typical
IMU_GYRO_NOISE = 0.00279   # rad/s stddev
IMU_ACCEL_NOISE = 0.0314   # m/s² stddev

# Topic names — must match brain contract
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

    # Configure URDF import settings via commands API (Isaac Sim 4.5+)
    from isaacsim.asset.importer.urdf._urdf import UrdfJointTargetType
    status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    import_config.merge_fixed_joints = False  # Keep all frames for TF
    import_config.fix_base = False  # Robot should be free to move
    import_config.import_inertia_tensor = True
    import_config.default_drive_type = UrdfJointTargetType.JOINT_DRIVE_VELOCITY
    import_config.default_drive_strength = 1e4
    import_config.default_position_drive_damping = 1e3
    import_config.create_physics_scene = False  # World has its own

    # Import via commands
    status, result = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=urdf_path,
        import_config=import_config,
    )

    if not result:
        raise RuntimeError("URDF import failed")

    print(f"URDF imported at: {result}")
    return result


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
                ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("DiffController", "isaacsim.robot.wheeled_robots.DifferentialController"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
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
                ("SubscribeTwist.outputs:execOut", "ArticulationController.inputs:execIn"),
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
                ("ComputeOdom", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("PublishOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
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
                ("ReadJoints", "isaacsim.core.nodes.IsaacReadJointState"),
                ("PublishJointStates", "isaacsim.ros2.bridge.ROS2PublishJointState"),
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
        ("IsaacCreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
        ("PublishRGB", "isaacsim.ros2.bridge.ROS2CameraHelper"),
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
    node_list.append(("PublishCameraInfo", "isaacsim.ros2.bridge.ROS2CameraHelper"))
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
        node_list.append(("PublishDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"))
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
            node_list.append(("PublishPC", "isaacsim.ros2.bridge.ROS2CameraHelper"))
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
                ("IsaacReadIMU", "isaacsim.sensors.physics.IsaacReadIMU"),
                ("PublishIMU", "isaacsim.ros2.bridge.ROS2PublishImu"),
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

    # Wheel material — calibrated: rubber on packed soil/concrete (μs=0.6-0.8)
    wheel_mat = UsdPhysics.MaterialAPI.Apply(
        UsdShade.Material.Define(stage, f"{robot_path}/Materials/WheelMaterial").GetPrim())
    wheel_mat.CreateStaticFrictionAttr(0.7)
    wheel_mat.CreateDynamicFrictionAttr(0.6)
    wheel_mat.CreateRestitutionAttr(0.0)

    # Caster material — calibrated: nylon ball caster on hard floor (μs=0.02-0.05)
    caster_mat = UsdPhysics.MaterialAPI.Apply(
        UsdShade.Material.Define(stage, f"{robot_path}/Materials/CasterMaterial").GetPrim())
    caster_mat.CreateStaticFrictionAttr(0.03)
    caster_mat.CreateDynamicFrictionAttr(0.025)
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

    print("  Wheels: μs=0.7/μd=0.6 (rubber on soil), Casters: μs=0.03/μd=0.025 (nylon)")


def apply_visual_materials(stage, robot_path):
    """Apply OmniPBR MDL materials to robot prims for RTX realism.

    Only changes visual appearance — collision geometry, TF frames,
    and inertial properties are preserved exactly as imported from URDF.
    """
    print("Applying OmniPBR visual materials to robot...")

    # Import the material creator from the world primitives module
    import sys
    world_scripts = os.path.join(PACKAGE_DIR, "scripts")
    if world_scripts not in sys.path:
        sys.path.insert(0, world_scripts)
    from world.primitives import create_omnipbr_material

    MAT = f"{robot_path}/Materials"

    # Chassis: brushed aluminum
    mat_chassis = create_omnipbr_material(
        stage, f"{MAT}/Chassis",
        roughness=0.5, metallic=0.4,
        diffuse_color=Gf.Vec3f(0.40, 0.42, 0.45),
        specular_level=0.5)

    # Wheels: matte black rubber
    mat_wheel = create_omnipbr_material(
        stage, f"{MAT}/Wheel",
        roughness=0.9, metallic=0.0,
        diffuse_color=Gf.Vec3f(0.08, 0.08, 0.08),
        specular_level=0.1)

    # Casters: painted steel
    mat_caster = create_omnipbr_material(
        stage, f"{MAT}/Caster",
        roughness=0.6, metallic=0.3,
        diffuse_color=Gf.Vec3f(0.30, 0.30, 0.30),
        specular_level=0.4)

    # ZED 2i camera body: dark anodized aluminum
    mat_camera = create_omnipbr_material(
        stage, f"{MAT}/Camera",
        roughness=0.4, metallic=0.2,
        diffuse_color=Gf.Vec3f(0.15, 0.15, 0.15),
        specular_level=0.5)

    # Bind materials to visual geometry
    material_map = {
        "base_link": mat_chassis,
        "left_wheel": mat_wheel,
        "right_wheel": mat_wheel,
        "front_caster": mat_caster,
        "rear_caster": mat_caster,
        "zed_camera_link": mat_camera,
    }

    for link_name, material in material_map.items():
        link_prim = stage.GetPrimAtPath(f"{robot_path}/{link_name}")
        if link_prim:
            # Bind to the link itself and any visual mesh children
            UsdShade.MaterialBindingAPI.Apply(link_prim)
            UsdShade.MaterialBindingAPI(link_prim).Bind(material)
            for child in link_prim.GetAllChildren():
                if child.IsA(UsdGeom.Gprim):
                    UsdShade.MaterialBindingAPI.Apply(child)
                    UsdShade.MaterialBindingAPI(child).Bind(material)

    print("  Chassis: brushed aluminum (metallic=0.4)")
    print("  Wheels: matte rubber (roughness=0.9)")
    print("  ZED 2i: dark anodized aluminum (metallic=0.2)")


def setup_camera_exposure(stage, robot_path):
    """Configure fixed camera exposure matching real ZED 2i in greenhouse.

    The real ZED 2i uses auto-exposure, but Isaac Sim's auto-exposure
    algorithm differs. Fixed exposure tuned for greenhouse lighting
    provides more consistent sim-to-real transfer.
    """
    print("Configuring camera exposure...")

    for cam_name in ["ZedLeftCamera", "ZedRightCamera"]:
        cam_prim = stage.GetPrimAtPath(f"{robot_path}/{cam_name}")
        if not cam_prim:
            # Try alternative paths
            for prefix in ["zed_camera_link", "zed_camera_center",
                          "zed_left_camera_frame", "zed_right_camera_frame"]:
                cam_prim = stage.GetPrimAtPath(f"{robot_path}/{prefix}/{cam_name}")
                if cam_prim:
                    break

        if cam_prim:
            cam_prim.CreateAttribute(
                "omni:camera:autoExposure", Sdf.ValueTypeNames.Bool).Set(False)
            cam_prim.CreateAttribute(
                "omni:camera:iso", Sdf.ValueTypeNames.Float).Set(200.0)
            cam_prim.CreateAttribute(
                "omni:camera:shutterSpeed", Sdf.ValueTypeNames.Float).Set(60.0)
            cam_prim.CreateAttribute(
                "omni:camera:motionBlur", Sdf.ValueTypeNames.Bool).Set(False)
            cam_prim.CreateAttribute(
                "omni:camera:fStop", Sdf.ValueTypeNames.Float).Set(0.0)  # no DoF

    print("  Auto-exposure: OFF, ISO: 200, Shutter: 1/60s, Motion blur: OFF")


def main():
    # Step 1: Process xacro
    urdf_path = process_xacro()

    # Step 2: Create stage and import URDF
    stage = omni.usd.get_context().get_stage()
    robot_path = import_urdf(urdf_path)

    # Step 3: Physics materials
    setup_physics_materials(stage, robot_path)

    # Step 4: Visual materials (OmniPBR for RTX realism)
    apply_visual_materials(stage, robot_path)

    # Step 4b: Enhanced visual geometry (multi-panel chassis, wheel hubs)
    from robot.visual_enhancements import enhance_robot_visuals
    enhance_robot_visuals(stage, robot_path)

    # Step 5: Camera exposure configuration
    setup_camera_exposure(stage, robot_path)

    # OmniGraph (diff-drive, odometry, cameras, IMU) is configured at
    # runtime by setup_omnigraph.py because the ROS 2 bridge extension
    # requires the full Isaac Sim application with ROS 2 middleware.

    # Step 6: Save
    stage.GetRootLayer().Export(OUTPUT_PATH)
    print(f"\nRobot USD saved to: {OUTPUT_PATH}")

    # Cleanup temp URDF
    if os.path.exists(TEMP_URDF):
        os.remove(TEMP_URDF)

    simulation_app.close()


if __name__ == "__main__":
    main()
