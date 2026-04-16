#!/usr/bin/env python3
"""Configure OmniGraph for the AGV robot in Isaac Sim.

Run inside the full Isaac Sim application (not standalone headless):
    isaacsim --exec setup_omnigraph.py

Sets up the brain-facing topic contract. Physical parameters match
agv_odrive/config/odrive_params.yaml (calibrated 2026-04-08).

Publishes:
- /agv/wheel_odom          50 Hz, frame odom → base_link
- /agv/joint_states        50 Hz
- /agv/imu/data            200 Hz, frame imu_link
- /agv/zed/left/image_rect_color, camera_info    15 Hz
- /agv/zed/right/image_rect_color, camera_info   15 Hz
- /agv/zed/depth/depth_registered                15 Hz
- /agv/zed/point_cloud/cloud_registered          15 Hz

Subscribes:
- /agv/shaped_cmd_vel      (output of sim_drive_shaping_node)
"""

import omni.graph.core as og
import omni.usd

# Robot parameters — calibrated 2026-04-08 (agv_odrive/config/odrive_params.yaml)
WHEEL_RADIUS = 0.0781
WHEEL_SEPARATION = 0.960
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
CAMERA_HFOV = 110.0

# Topic names. For the two sensors that need realism relays (IMU → bias drift,
# depth → distance-dependent noise), OmniGraph publishes to a `_clean` topic
# that the relay reads and republishes on the final brain-facing name. Cameras
# are already rectified in the ZED SDK real pipeline, so they go straight to
# the final topic name without any relay.
TOPICS = {
    "cmd_vel":           "/agv/shaped_cmd_vel",
    "wheel_odom":        "/agv/wheel_odom",
    "joint_states":      "/agv/joint_states",
    "imu":               "/agv/imu/data_clean",           # → isaac_ros_bridge_node → /agv/imu/data
    "left_depth":        "/agv/zed/depth/depth_clean",    # → sim_depth_noise → /agv/zed/depth/depth_registered
    "left_rgb":          "/agv/zed/left/image_rect_color",
    "left_camera_info":  "/agv/zed/left/camera_info",
    "left_pointcloud":   "/agv/zed/point_cloud/cloud_registered",
    "right_rgb":         "/agv/zed/right/image_rect_color",
    "right_camera_info": "/agv/zed/right/camera_info",
}

ROBOT_PATH = "/agv"


def setup_diff_drive_controller():
    """Differential drive: /agv/shaped_cmd_vel → wheel velocities."""
    print("Setting up differential drive controller...")
    keys = og.Controller.Keys

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
    print(f"  Subscribing to {TOPICS['cmd_vel']}")


def setup_odometry_publisher():
    """Publish wheel odometry on /agv/wheel_odom + TF."""
    print("Setting up odometry publisher...")
    keys = og.Controller.Keys

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
                ("OnPlaybackTick.outputs:tick", "ReadSimTime.inputs:execIn"),
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
    print(f"  Publishing to {TOPICS['wheel_odom']}")


def setup_joint_state_publisher():
    """Publish joint states on /agv/joint_states."""
    print("Setting up joint state publisher...")
    keys = og.Controller.Keys

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
                ("OnPlaybackTick.outputs:tick", "ReadSimTime.inputs:execIn"),
                ("Context.outputs:context", "PublishJointStates.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishJointStates.inputs:timeStamp"),
            ],
        },
    )
    print(f"  Publishing to {TOPICS['joint_states']}")


def setup_camera(cam_name, parent_link, frame_id, topics, enable_depth=False):
    """Set up camera with ROS 2 publishers."""
    print(f"Setting up camera: {cam_name}...")
    keys = og.Controller.Keys

    cam_prim_path = f"{ROBOT_PATH}/{parent_link}/Camera"

    node_list = [
        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
        ("Context", "isaacsim.ros2.bridge.ROS2Context"),
        ("IsaacCreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
        ("PublishRGB", "isaacsim.ros2.bridge.ROS2CameraHelper"),
        ("PublishCameraInfo", "isaacsim.ros2.bridge.ROS2CameraHelper"),
    ]
    set_values = [
        ("IsaacCreateRenderProduct.inputs:cameraPrim", cam_prim_path),
        ("IsaacCreateRenderProduct.inputs:width", CAMERA_WIDTH),
        ("IsaacCreateRenderProduct.inputs:height", CAMERA_HEIGHT),
        ("PublishRGB.inputs:topicName", topics["rgb"]),
        ("PublishRGB.inputs:frameId", frame_id),
        ("PublishRGB.inputs:type", "rgb"),
        ("PublishCameraInfo.inputs:topicName", topics["camera_info"]),
        ("PublishCameraInfo.inputs:frameId", frame_id),
        ("PublishCameraInfo.inputs:type", "camera_info"),
    ]
    connections = [
        ("OnPlaybackTick.outputs:tick", "IsaacCreateRenderProduct.inputs:execIn"),
        ("IsaacCreateRenderProduct.outputs:execOut", "PublishRGB.inputs:execIn"),
        ("IsaacCreateRenderProduct.outputs:renderProductPath", "PublishRGB.inputs:renderProductPath"),
        ("Context.outputs:context", "PublishRGB.inputs:context"),
        ("IsaacCreateRenderProduct.outputs:execOut", "PublishCameraInfo.inputs:execIn"),
        ("IsaacCreateRenderProduct.outputs:renderProductPath", "PublishCameraInfo.inputs:renderProductPath"),
        ("Context.outputs:context", "PublishCameraInfo.inputs:context"),
    ]

    if enable_depth:
        node_list.append(("PublishDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"))
        set_values.extend([
            ("PublishDepth.inputs:topicName", topics["depth"]),
            ("PublishDepth.inputs:frameId", frame_id),
            ("PublishDepth.inputs:type", "depth"),
        ])
        connections.extend([
            ("IsaacCreateRenderProduct.outputs:execOut", "PublishDepth.inputs:execIn"),
            ("IsaacCreateRenderProduct.outputs:renderProductPath", "PublishDepth.inputs:renderProductPath"),
            ("Context.outputs:context", "PublishDepth.inputs:context"),
        ])

        if "pointcloud" in topics:
            node_list.append(("PublishPC", "isaacsim.ros2.bridge.ROS2CameraHelper"))
            set_values.extend([
                ("PublishPC.inputs:topicName", topics["pointcloud"]),
                ("PublishPC.inputs:frameId", frame_id),
                ("PublishPC.inputs:type", "depth_pcl"),
            ])
            connections.extend([
                ("IsaacCreateRenderProduct.outputs:execOut", "PublishPC.inputs:execIn"),
                ("IsaacCreateRenderProduct.outputs:renderProductPath", "PublishPC.inputs:renderProductPath"),
                ("Context.outputs:context", "PublishPC.inputs:context"),
            ])

    og.Controller.edit(
        {"graph_path": f"{ROBOT_PATH}/{cam_name}Graph", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: node_list,
            keys.SET_VALUES: set_values,
            keys.CONNECT: connections,
        },
    )
    for t_name, t_value in topics.items():
        print(f"  {t_name}: {t_value}")


def setup_imu():
    """Set up IMU sensor with ROS 2 publisher."""
    print("Setting up IMU sensor...")
    keys = og.Controller.Keys

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
                ("IsaacReadIMU.inputs:imuPrim", f"{ROBOT_PATH}/base_link/ImuSensor"),
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
    print(f"  Publishing to {TOPICS['imu']}")


def main():
    print("Configuring OmniGraph for AGV robot...")

    setup_diff_drive_controller()
    setup_odometry_publisher()
    setup_joint_state_publisher()

    # Left camera (depth + RGB + point cloud)
    setup_camera(
        cam_name="ZedLeftCamera",
        parent_link="zed_left_camera_frame",
        frame_id="zed_left_camera_frame_optical",
        topics={
            "rgb": TOPICS["left_rgb"],
            "depth": TOPICS["left_depth"],
            "camera_info": TOPICS["left_camera_info"],
            "pointcloud": TOPICS["left_pointcloud"],
        },
        enable_depth=True,
    )

    # Right camera (RGB only)
    setup_camera(
        cam_name="ZedRightCamera",
        parent_link="zed_right_camera_frame",
        frame_id="zed_right_camera_frame_optical",
        topics={
            "rgb": TOPICS["right_rgb"],
            "camera_info": TOPICS["right_camera_info"],
        },
        enable_depth=False,
    )

    setup_imu()

    print("\nOmniGraph setup complete!")


if __name__ == "__main__":
    main()
