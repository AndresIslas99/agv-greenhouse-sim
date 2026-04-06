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
from pxr import Usd, UsdGeom, Sdf

# Robot parameters
WHEEL_RADIUS = 0.0625
WHEEL_SEPARATION = 0.735
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
ROBOT_PATH = "/agv"

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
}

# Create a new composed stage: greenhouse world as sublayer + robot as reference
print(f"Creating composed stage at {OUTPUT_USD}...")
stage = Usd.Stage.CreateNew(OUTPUT_USD)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

# Add the greenhouse world as a sublayer (brings in /World and all its children)
stage.GetRootLayer().subLayerPaths.append(WORLD_USD)
print(f"  Sublayer: {WORLD_USD}")

# Add the robot as a reference under /agv
robot_prim = stage.DefinePrim(ROBOT_PATH, "Xform")
robot_prim.GetReferences().AddReference(ROBOT_USD)
print(f"  Robot ref: {ROBOT_USD} -> {ROBOT_PATH}")

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

keys = og.Controller.Keys

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
                ("DiffController", "isaacsim.robot.wheeled_robots.DifferentialController"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
            ],
            keys.SET_VALUES: [
                ("SubscribeTwist.inputs:topicName", TOPICS["cmd_vel"]),
                ("DiffController.inputs:wheelRadius", WHEEL_RADIUS),
                ("DiffController.inputs:wheelDistance", WHEEL_SEPARATION),
                ("ArticulationController.inputs:robotPath", ROBOT_PATH),
                ("ArticulationController.inputs:jointNames", ["left_wheel_joint", "right_wheel_joint"]),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
                ("Context.outputs:context", "SubscribeTwist.inputs:context"),
                ("SubscribeTwist.outputs:execOut", "DiffController.inputs:execIn"),
                ("SubscribeTwist.outputs:linearVelocity", "DiffController.inputs:linearVelocity"),
                ("SubscribeTwist.outputs:angularVelocity", "DiffController.inputs:angularVelocity"),
                ("SubscribeTwist.outputs:execOut", "ArticulationController.inputs:execIn"),
                ("DiffController.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
            ],
        },
    )
    print("   OK")
except Exception as e:
    print(f"   FAILED: {e}")

# --- Odometry ---
print("2. Setting up odometry publisher...")
try:
    og.Controller.edit(
        {"graph_path": f"{ROBOT_PATH}/OdometryGraph", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ComputeOdom", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("PublishOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ],
            keys.SET_VALUES: [
                ("ReadSimTime.inputs:resetOnStop", False),
                ("ComputeOdom.inputs:chassisPrim", ROBOT_PATH),
                ("PublishOdom.inputs:topicName", TOPICS["wheel_odom"]),
                ("PublishOdom.inputs:odomFrameId", "odom"),
                ("PublishOdom.inputs:chassisFrameId", "base_link"),
                ("PublishTF.inputs:targetPrims", ROBOT_PATH),
                ("PublishTF.inputs:topicName", "/tf"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "ComputeOdom.inputs:execIn"),
                ("ComputeOdom.outputs:execOut", "PublishOdom.inputs:execIn"),
                ("ComputeOdom.outputs:position", "PublishOdom.inputs:position"),
                ("ComputeOdom.outputs:orientation", "PublishOdom.inputs:orientation"),
                ("ComputeOdom.outputs:linearVelocity", "PublishOdom.inputs:linearVelocity"),
                ("ComputeOdom.outputs:angularVelocity", "PublishOdom.inputs:angularVelocity"),
                ("Context.outputs:context", "PublishOdom.inputs:context"),
                ("Context.outputs:context", "PublishTF.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishOdom.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
                ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
            ],
        },
    )
    print("   OK")
except Exception as e:
    print(f"   FAILED: {e}")

# --- Joint States ---
print("3. Setting up joint state publisher...")
try:
    og.Controller.edit(
        {"graph_path": f"{ROBOT_PATH}/JointStateGraph", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishJointStates", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ],
            keys.SET_VALUES: [
                ("ReadSimTime.inputs:resetOnStop", False),
                ("PublishJointStates.inputs:targetPrim", ROBOT_PATH),
                ("PublishJointStates.inputs:topicName", TOPICS["joint_states"]),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishJointStates.inputs:execIn"),
                ("Context.outputs:context", "PublishJointStates.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishJointStates.inputs:timeStamp"),
            ],
        },
    )
    print("   OK")
except Exception as e:
    print(f"   FAILED: {e}")

# --- IMU ---
print("4. Setting up IMU sensor...")
try:
    og.Controller.edit(
        {"graph_path": f"{ROBOT_PATH}/IMUGraph", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("IsaacReadIMU", "isaacsim.sensors.physics.IsaacReadIMU"),
                ("PublishIMU", "isaacsim.ros2.bridge.ROS2PublishImu"),
            ],
            keys.SET_VALUES: [
                ("IsaacReadIMU.inputs:imuPrim", f"{ROBOT_PATH}/imu_link"),
                ("PublishIMU.inputs:topicName", TOPICS["imu"]),
                ("PublishIMU.inputs:frameId", "imu_link"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "IsaacReadIMU.inputs:execIn"),
                ("IsaacReadIMU.outputs:execOut", "PublishIMU.inputs:execIn"),
                ("IsaacReadIMU.outputs:angVel", "PublishIMU.inputs:angularVelocity"),
                ("IsaacReadIMU.outputs:linAcc", "PublishIMU.inputs:linearAcceleration"),
                ("IsaacReadIMU.outputs:orientation", "PublishIMU.inputs:orientation"),
                ("Context.outputs:context", "PublishIMU.inputs:context"),
            ],
        },
    )
    print("   OK")
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

print("6. Setting up right camera (RGB)...")
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
