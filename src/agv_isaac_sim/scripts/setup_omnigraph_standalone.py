#!/usr/bin/env python3
"""Configure OmniGraph for AGV robot — standalone mode.

Usage:
    source install/setup.bash
    python3 src/agv_isaac_sim/scripts/setup_omnigraph_standalone.py
"""

import os
import sys

# Must be first — before any omni/pxr imports
from isaacsim import SimulationApp

PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD_USD = os.path.join(PACKAGE_DIR, "worlds", "greenhouse_simple.usd")
ROBOT_USD = os.path.join(PACKAGE_DIR, "robot", "agv_sim.usd")

OUTPUT_USD = os.path.join(PACKAGE_DIR, "worlds", "greenhouse_with_robot.usd")

simulation_app = SimulationApp({"headless": True})

# Performance settings — applied at Kit app level so they bake into sessions
# that open the generated USD. These decouple rendering from the sim loop,
# let PhysX write back asynchronously, and keep rendering in Real-Time mode.
import carb.settings
_carb = carb.settings.get_settings()
_carb.set_bool("/app/asyncRendering", True)
_carb.set_bool("/app/asyncRenderingLowLatency", True)
_carb.set_bool("/app/hydraEngine/waitIdle", False)
_carb.set_bool("/physics/suppressReadback", True)          # skip CPU readback of physics state
_carb.set_bool("/physics/updateToUsd", False)              # don't write physics back to USD each step
_carb.set_bool("/physics/updateVelocitiesToUsd", False)
_carb.set_bool("/physics/updateParticlesToUsd", False)
_carb.set_bool("/rtx/ecoMode/enabled", False)              # disable RTX power saving
_carb.set_string("/rtx/rendermode", "RaytracedLighting")   # Real-Time (not PathTracing)
# Lower RTX quality to free GPU time for camera renders → higher FPS
_carb.set_bool("/rtx/reflections/enabled", False)
_carb.set_bool("/rtx/translucency/enabled", False)
_carb.set_bool("/rtx/indirectDiffuse/enabled", False)
_carb.set_bool("/rtx/ambientOcclusion/enabled", False)
_carb.set_bool("/rtx/post/aa/op", False)                    # disable anti-aliasing pass
_carb.set_bool("/rtx/post/dlss/enabled", False)
_carb.set_bool("/rtx/shadows/enabled", True)                # keep shadows for AprilTag detection
_carb.set_int("/rtx/directLighting/sampledLighting/samplesPerPixel", 1)
_carb.set_int("/rtx/post/histogram/filterType", 0)

# Now enable extensions one by one, checking each
import omni.kit.app
import omni.usd

ext_manager = omni.kit.app.get_app().get_extension_manager()

REQUIRED_EXTENSIONS = [
    "isaacsim.core.nodes",
    "isaacsim.robot.wheeled_robots",
    "isaacsim.sensors.physics",
    "isaacsim.ros2.bridge",
]

for ext_name in REQUIRED_EXTENSIONS:
    print(f"Enabling extension: {ext_name}...")
    try:
        result = ext_manager.set_extension_enabled_immediate(ext_name, True)
        print(f"  -> {'OK' if result else 'FAILED'}")
    except Exception as e:
        print(f"  -> ERROR: {e}")
        sys.exit(1)

# Give the app a moment to initialize extensions
import omni.kit.app as app
for _ in range(10):
    app.get_app().update()

print("\nAll extensions loaded. Setting up composed stage...")

import omni.graph.core as og
from pxr import Usd, UsdGeom, Sdf, Gf

# Robot parameters — calibrated 2026-04-08 (agv_odrive/config/odrive_params.yaml)
WHEEL_RADIUS = 0.0781
WHEEL_SEPARATION = 0.960
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
ROBOT_PATH = "/agv"
ARTICULATION_PATH = "/agv/base_link"  # ArticulationRootAPI is on base_link, not /agv

# Initial robot position: center of free corridor, wheels on ground
ROBOT_START_POS = Gf.Vec3d(5.5, 0.0, 0.2)

# Topic names. IMU and depth go through realism relays (see setup_omnigraph.py
# comment). Camera images are published directly — real ZED SDK gives rectified.
TOPICS = {
    "cmd_vel":           "/agv/shaped_cmd_vel",
    "wheel_odom":        "/agv/wheel_odom",
    "joint_states":      "/agv/joint_states",
    "imu":               "/agv/imu/data_clean",
    "left_depth":        "/agv/zed/depth/depth_clean",
    "left_rgb":          "/agv/zed/left/image_rect_color",
    "left_camera_info":  "/agv/zed/left/camera_info",
    "left_pointcloud":   "/agv/zed/point_cloud/cloud_registered",
    "right_rgb":         "/agv/zed/right/image_rect_color",
    "right_camera_info": "/agv/zed/right/camera_info",
}

# Create a new composed stage: greenhouse world as sublayer + robot as reference
print(f"Creating composed stage at {OUTPUT_USD}...")
stage = Usd.Stage.CreateNew(OUTPUT_USD)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

# Add the greenhouse world as a sublayer (brings in /World and all its children)
stage.GetRootLayer().subLayerPaths.append(WORLD_USD)
print(f"  Sublayer: {WORLD_USD}")

# Add the robot as a reference under /agv with initial position
robot_prim = stage.DefinePrim(ROBOT_PATH, "Xform")
robot_prim.GetReferences().AddReference(ROBOT_USD)

# Set initial position — the referenced USD already has xformOps from URDF import,
# so we override the existing translate op instead of adding a new one
xformable = UsdGeom.Xformable(robot_prim)
for op in xformable.GetOrderedXformOps():
    if op.GetOpName() == "xformOp:translate":
        op.Set(ROBOT_START_POS)
        break
else:
    # No existing translate op — add one on the outer prim (override layer)
    robot_prim.CreateAttribute("xformOp:translate", Sdf.ValueTypeNames.Double3).Set(ROBOT_START_POS)
    existing_order = xformable.GetXformOpOrderAttr().Get() or []
    xformable.GetXformOpOrderAttr().Set(["xformOp:translate"] + list(existing_order))

print(f"  Robot ref: {ROBOT_USD} -> {ROBOT_PATH} at {ROBOT_START_POS}")

# Save so far, then re-open via omni.usd so OmniGraph can work on it
stage.GetRootLayer().Save()
omni.usd.get_context().open_stage(OUTPUT_USD)

# Wait for stage to load
for _ in range(10):
    app.get_app().update()

stage = omni.usd.get_context().get_stage()

# Update the stage
for _ in range(5):
    app.get_app().update()

# --- Create sensor prims that OmniGraphs need ---
# The URDF import only creates link Xforms. Cameras and IMU sensors
# need to be created as USD prims before OmniGraph can reference them.
print("\nCreating sensor prims...")
import math

# ZED 2i cameras (left + right)
for cam_frame, baseline_y in [("zed_left_camera_frame", 0.0), ("zed_right_camera_frame", 0.0)]:
    cam_path = f"{ROBOT_PATH}/{cam_frame}/Camera"
    if not stage.GetPrimAtPath(cam_path).IsValid():
        cam = UsdGeom.Camera.Define(stage, cam_path)
        # ZED 2i: 110° HFOV at 1280px → focal length ≈ 2.05mm (at 36mm sensor)
        # In USD camera convention: focalLength in mm, horizontalAperture in mm
        hfov_rad = math.radians(110.0)
        h_aperture = 20.955  # mm (standard USD convention)
        focal_length = h_aperture / (2.0 * math.tan(hfov_rad / 2.0))
        cam.CreateFocalLengthAttr(focal_length)
        cam.CreateHorizontalApertureAttr(h_aperture)
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.3, 20.0))
        cam.CreateFStopAttr(0.0)  # no DoF
        print(f"  Camera: {cam_path} (focal={focal_length:.2f}mm, hFOV=110°)")

# IMU: IsaacReadIMU requires a prim with RigidBodyAPI. Since imu_link is not
# in the URDF (to avoid double TF in HIL), we attach the sensor to base_link
# which has physics. The offset to the real BMI088 position (camera mount at
# 0.70, 0, -0.055) is set on the sensor prim.
imu_parent = f"{ROBOT_PATH}/base_link"
imu_sensor_path = f"{imu_parent}/ImuSensor"
imu_sensor_prim = stage.GetPrimAtPath(imu_sensor_path)
if not imu_sensor_prim.IsValid():
    try:
        omni.kit.commands.execute(
            "IsaacSensorCreateImuSensor",
            path="/ImuSensor",
            parent=imu_parent,
            sensor_period=0,  # every physics step
            translation=Gf.Vec3d(0.70, 0.0, -0.055),
        )
        print(f"  IMU sensor created at {imu_sensor_path}")
    except Exception as e:
        print(f"  IMU sensor creation failed: {e}")

# --- Physics scene: 200 Hz timestep matches real BMI088 IMU rate ---
# Physics simulates deterministically at 200 Hz (5ms step). The brain's
# EKF runs at 50 Hz, so we have 4x headroom on sensor data. OmniGraphs
# driven by OnPhysicsStep fire at this exact rate.
from pxr import UsdPhysics, PhysxSchema
physics_scene_path = "/World/physicsScene"
physics_scene = UsdPhysics.Scene.Get(stage, physics_scene_path)
if not physics_scene:
    physics_scene = UsdPhysics.Scene.Define(stage, physics_scene_path)
physx_scene = PhysxSchema.PhysxSceneAPI.Apply(physics_scene.GetPrim())
physx_scene.CreateTimeStepsPerSecondAttr(200)
physx_scene.CreateEnableGPUDynamicsAttr(True)
physx_scene.CreateEnableCCDAttr(False)            # CCD off unless needed (heavy)
physx_scene.CreateSolverTypeAttr("TGS")           # Temporal Gauss-Seidel — more stable than PGS
physx_scene.CreateGpuMaxNumPartitionsAttr(8)
physx_scene.CreateGpuFoundLostPairsCapacityAttr(4096)
physx_scene.CreateGpuFoundLostAggregatePairsCapacityAttr(4096)
print(f"\nPhysics scene: {physics_scene_path} @ 200 Hz (5 ms step), TGS solver, GPU dynamics")

# Remove any stale OmniGraphs from previous runs so evaluator_name changes stick
for graph_path in (
    "/World/ClockGraph",
    f"{ROBOT_PATH}/DiffDriveController",
    f"{ROBOT_PATH}/OdometryGraph",
    f"{ROBOT_PATH}/JointStateGraph",
    f"{ROBOT_PATH}/IMUGraph",
    f"{ROBOT_PATH}/ZedLeftGraph",
    f"{ROBOT_PATH}/ZedRightGraph",
):
    existing = stage.GetPrimAtPath(graph_path)
    if existing.IsValid():
        stage.RemovePrim(graph_path)
        print(f"  Removed stale graph: {graph_path}")

# Save sensor prims before creating OmniGraphs
stage.GetRootLayer().Save()
for _ in range(5):
    app.get_app().update()

keys = og.Controller.Keys

# --- Clock Publisher (required for use_sim_time nodes) ---
# Fires every physics step (200 Hz) so /clock matches sim time exactly.
# IsaacReadSimulationTime provides the sim-time stamp to PublishClock.
print("\n0. Setting up simulation clock publisher...")
try:
    og.Controller.edit(
        {"graph_path": "/World/ClockGraph", "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND},
        {
            keys.CREATE_NODES: [
                ("OnPhysicsStep", "isaacsim.core.nodes.OnPhysicsStep"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            keys.CONNECT: [
                ("OnPhysicsStep.outputs:step", "PublishClock.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
            ],
        },
    )
    print("   OK (OnPhysicsStep → 200 Hz)")
except Exception as e:
    print(f"   FAILED: {e}")

# --- Diff Drive Controller ---
print("\n1. Setting up differential drive controller...")
try:
    og.Controller.edit(
        {"graph_path": f"{ROBOT_PATH}/DiffDriveController", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("BreakLinVel", "omni.graph.nodes.BreakVector3"),
                ("BreakAngVel", "omni.graph.nodes.BreakVector3"),
                ("DiffController", "isaacsim.robot.wheeled_robots.DifferentialController"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
            ],
            keys.SET_VALUES: [
                ("SubscribeTwist.inputs:topicName", TOPICS["cmd_vel"]),
                ("DiffController.inputs:wheelRadius", WHEEL_RADIUS),
                ("DiffController.inputs:wheelDistance", WHEEL_SEPARATION),
                ("ArticulationController.inputs:robotPath", ARTICULATION_PATH),
                ("ArticulationController.inputs:jointNames", ["left_wheel_joint", "right_wheel_joint"]),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
                ("Context.outputs:context", "SubscribeTwist.inputs:context"),
                ("SubscribeTwist.outputs:execOut", "DiffController.inputs:execIn"),
                # Break Vec3 → scalar: linear.x for forward, angular.z for rotation
                ("SubscribeTwist.outputs:linearVelocity", "BreakLinVel.inputs:tuple"),
                ("SubscribeTwist.outputs:angularVelocity", "BreakAngVel.inputs:tuple"),
                ("BreakLinVel.outputs:x", "DiffController.inputs:linearVelocity"),
                ("BreakAngVel.outputs:z", "DiffController.inputs:angularVelocity"),
                ("SubscribeTwist.outputs:execOut", "ArticulationController.inputs:execIn"),
                ("DiffController.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
            ],
        },
    )
    print("   OK")
except Exception as e:
    print(f"   FAILED: {e}")

# --- Odometry ---
# OnPhysicsStep fires at 200 Hz (physics rate). The real agv_odrive publishes
# at 50 Hz — the 4x higher sim rate is fine, brain's EKF consumes what it needs.
print("2. Setting up odometry publisher...")
try:
    og.Controller.edit(
        {"graph_path": f"{ROBOT_PATH}/OdometryGraph", "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND},
        {
            keys.CREATE_NODES: [
                ("OnPhysicsStep", "isaacsim.core.nodes.OnPhysicsStep"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ComputeOdom", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("PublishOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ],
            keys.SET_VALUES: [
                ("ComputeOdom.inputs:chassisPrim", ARTICULATION_PATH),
                # Was TOPICS["wheel_odom"] (= /agv/wheel_odom). Renamed
                # because IsaacComputeOdometry produces wrong-signed
                # position.x and zero covariance, which collapses the
                # brain ekf_local. The /agv/wheel_odom contract is now
                # owned by sim_wheel_odom_publisher.py (encoder-style
                # integration from /agv/joint_states). This OmniGraph
                # publisher stays alive for diagnostic comparisons.
                ("PublishOdom.inputs:topicName", "/agv/_sim_internal/isaac_compute_odom_raw"),
                ("PublishOdom.inputs:odomFrameId", "odom"),
                ("PublishOdom.inputs:chassisFrameId", "base_link"),
                ("PublishTF.inputs:targetPrims", ROBOT_PATH),
                ("PublishTF.inputs:topicName", "/tf"),
            ],
            keys.CONNECT: [
                ("OnPhysicsStep.outputs:step", "ComputeOdom.inputs:execIn"),
                ("ComputeOdom.outputs:execOut", "PublishOdom.inputs:execIn"),
                ("ComputeOdom.outputs:position", "PublishOdom.inputs:position"),
                ("ComputeOdom.outputs:orientation", "PublishOdom.inputs:orientation"),
                ("ComputeOdom.outputs:linearVelocity", "PublishOdom.inputs:linearVelocity"),
                ("ComputeOdom.outputs:angularVelocity", "PublishOdom.inputs:angularVelocity"),
                ("ReadSimTime.outputs:simulationTime", "PublishOdom.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
                ("Context.outputs:context", "PublishOdom.inputs:context"),
                ("Context.outputs:context", "PublishTF.inputs:context"),
                ("OnPhysicsStep.outputs:step", "PublishTF.inputs:execIn"),
            ],
        },
    )
    print("   OK (OnPhysicsStep → 200 Hz)")
except Exception as e:
    print(f"   FAILED: {e}")

# --- Joint States ---
print("3. Setting up joint state publisher...")
try:
    og.Controller.edit(
        {"graph_path": f"{ROBOT_PATH}/JointStateGraph", "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND},
        {
            keys.CREATE_NODES: [
                ("OnPhysicsStep", "isaacsim.core.nodes.OnPhysicsStep"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishJointStates", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ],
            keys.SET_VALUES: [
                ("PublishJointStates.inputs:targetPrim", ARTICULATION_PATH),
                ("PublishJointStates.inputs:topicName", TOPICS["joint_states"]),
            ],
            keys.CONNECT: [
                ("OnPhysicsStep.outputs:step", "PublishJointStates.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishJointStates.inputs:timeStamp"),
                ("Context.outputs:context", "PublishJointStates.inputs:context"),
            ],
        },
    )
    print("   OK (OnPhysicsStep → 200 Hz)")
except Exception as e:
    print(f"   FAILED: {e}")

# --- IMU ---
# Physics @ 200 Hz exactly matches the real BMI088 IMU rate.
print("4. Setting up IMU sensor...")
try:
    og.Controller.edit(
        {"graph_path": f"{ROBOT_PATH}/IMUGraph", "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND},
        {
            keys.CREATE_NODES: [
                ("OnPhysicsStep", "isaacsim.core.nodes.OnPhysicsStep"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("IsaacReadIMU", "isaacsim.sensors.physics.IsaacReadIMU"),
                ("PublishIMU", "isaacsim.ros2.bridge.ROS2PublishImu"),
            ],
            keys.SET_VALUES: [
                ("IsaacReadIMU.inputs:imuPrim", f"{ROBOT_PATH}/base_link/ImuSensor"),
                ("PublishIMU.inputs:topicName", TOPICS["imu"]),
                ("PublishIMU.inputs:frameId", "imu_link"),
            ],
            keys.CONNECT: [
                ("OnPhysicsStep.outputs:step", "IsaacReadIMU.inputs:execIn"),
                ("IsaacReadIMU.outputs:execOut", "PublishIMU.inputs:execIn"),
                ("IsaacReadIMU.outputs:angVel", "PublishIMU.inputs:angularVelocity"),
                ("IsaacReadIMU.outputs:linAcc", "PublishIMU.inputs:linearAcceleration"),
                ("IsaacReadIMU.outputs:orientation", "PublishIMU.inputs:orientation"),
                ("ReadSimTime.outputs:simulationTime", "PublishIMU.inputs:timeStamp"),
                ("Context.outputs:context", "PublishIMU.inputs:context"),
            ],
        },
    )
    print("   OK (OnPhysicsStep → 200 Hz, matches real BMI088)")
except Exception as e:
    print(f"   FAILED: {e}")

# --- Cameras ---
def setup_camera(cam_name, parent_link, frame_id, topics, enable_depth=False):
    node_list = [
        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
        ("Context", "isaacsim.ros2.bridge.ROS2Context"),
        ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
        ("PublishRGB", "isaacsim.ros2.bridge.ROS2CameraHelper"),
        ("PublishCameraInfo", "isaacsim.ros2.bridge.ROS2CameraHelper"),
    ]
    cam_prim = f"{ROBOT_PATH}/{parent_link}/Camera"
    set_values = [
        ("CreateRenderProduct.inputs:cameraPrim", cam_prim),
        ("CreateRenderProduct.inputs:width", CAMERA_WIDTH),
        ("CreateRenderProduct.inputs:height", CAMERA_HEIGHT),
        ("PublishRGB.inputs:topicName", topics["rgb"]),
        ("PublishRGB.inputs:frameId", frame_id),
        ("PublishRGB.inputs:type", "rgb"),
        ("PublishCameraInfo.inputs:topicName", topics["camera_info"]),
        ("PublishCameraInfo.inputs:frameId", frame_id),
        ("PublishCameraInfo.inputs:type", "camera_info"),
    ]
    # ROS2CameraHelper has no outputs — each node connects directly to tick
    connections = [
        ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
        ("CreateRenderProduct.outputs:execOut", "PublishRGB.inputs:execIn"),
        ("CreateRenderProduct.outputs:renderProductPath", "PublishRGB.inputs:renderProductPath"),
        ("Context.outputs:context", "PublishRGB.inputs:context"),
        ("CreateRenderProduct.outputs:execOut", "PublishCameraInfo.inputs:execIn"),
        ("CreateRenderProduct.outputs:renderProductPath", "PublishCameraInfo.inputs:renderProductPath"),
        ("Context.outputs:context", "PublishCameraInfo.inputs:context"),
    ]
    if enable_depth:
        node_list.append(("PublishDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"))
        set_values += [
            ("PublishDepth.inputs:topicName", topics["depth"]),
            ("PublishDepth.inputs:frameId", frame_id),
            ("PublishDepth.inputs:type", "depth"),
        ]
        connections += [
            ("CreateRenderProduct.outputs:execOut", "PublishDepth.inputs:execIn"),
            ("CreateRenderProduct.outputs:renderProductPath", "PublishDepth.inputs:renderProductPath"),
            ("Context.outputs:context", "PublishDepth.inputs:context"),
        ]
        if "pointcloud" in topics:
            node_list.append(("PublishPC", "isaacsim.ros2.bridge.ROS2CameraHelper"))
            set_values += [
                ("PublishPC.inputs:topicName", topics["pointcloud"]),
                ("PublishPC.inputs:frameId", frame_id),
                ("PublishPC.inputs:type", "depth_pcl"),
            ]
            connections += [
                ("CreateRenderProduct.outputs:execOut", "PublishPC.inputs:execIn"),
                ("CreateRenderProduct.outputs:renderProductPath", "PublishPC.inputs:renderProductPath"),
                ("Context.outputs:context", "PublishPC.inputs:context"),
            ]

    og.Controller.edit(
        {"graph_path": f"{ROBOT_PATH}/{cam_name}Graph", "evaluator_name": "execution"},
        {keys.CREATE_NODES: node_list, keys.SET_VALUES: set_values, keys.CONNECT: connections},
    )

print("5. Setting up left camera (depth+RGB+pointcloud)...")
try:
    setup_camera("ZedLeft", "zed_left_camera_frame", "zed_left_camera_frame_optical",
                 {"rgb": TOPICS["left_rgb"], "depth": TOPICS["left_depth"],
                  "camera_info": TOPICS["left_camera_info"], "pointcloud": TOPICS["left_pointcloud"]},
                 enable_depth=True)
    print("   OK")
except Exception as e:
    print(f"   FAILED: {e}")

print("6. Setting up right camera (RGB + camera_info — needed for cuVSLAM stereo)...")
# The ZED 2i's right sensor IS consumed by the brain in HIL:
#   - cuVSLAM (isaac_ros_visual_slam) on the Jetson runs against sim's stereo
#     pair + IMU. The Jetson cannot run the ZED SDK (no hardware), so the sim
#     must emulate both rectified outputs.
# An earlier note here said the right camera was not needed because we
# 'replaced cuVSLAM with sim_global_odom'. That sim-side relay was
# REMOVED 2026-04-15 (architecture cleanup) — the brain now owns
# /visual_slam/tracking/odometry, so it needs both camera streams again.
try:
    setup_camera("ZedRight", "zed_right_camera_frame", "zed_right_camera_frame_optical",
                 {"rgb": TOPICS["right_rgb"], "camera_info": TOPICS["right_camera_info"]},
                 enable_depth=False)
    print("   OK")
except Exception as e:
    print(f"   FAILED: {e}")

# Save
stage.GetRootLayer().Save()
print(f"\nComposed scene saved to: {OUTPUT_USD}")

simulation_app.close()
