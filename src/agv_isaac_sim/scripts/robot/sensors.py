"""Camera and IMU sensor OmniGraph setup."""

import math
from pxr import UsdGeom, Gf

from .config import (TOPICS, CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_HFOV_DEG,
                     CAMERA_CLIP_NEAR, CAMERA_CLIP_FAR)
from .omnigraph_helpers import create_action_graph


def _create_camera_prim(stage, cam_prim_path):
    """Create and configure a USD Camera prim matching ZED 2i specs."""
    camera = UsdGeom.Camera.Define(stage, cam_prim_path)
    hfov_rad = math.radians(CAMERA_HFOV_DEG)
    focal_length = (CAMERA_WIDTH / 2.0) / math.tan(hfov_rad / 2.0)
    camera.CreateFocalLengthAttr(focal_length * 36.0 / CAMERA_WIDTH)
    camera.CreateHorizontalApertureAttr(36.0)
    camera.CreateClippingRangeAttr(Gf.Vec2f(CAMERA_CLIP_NEAR, CAMERA_CLIP_FAR))
    return camera


def setup_camera(stage, robot_path, cam_name, parent_link, frame_id,
                 topics, enable_depth=False):
    """Set up an Isaac Sim camera with ROS 2 publishers.

    Args:
        stage: USD stage
        robot_path: Robot prim path
        cam_name: Unique name for this camera graph
        parent_link: Link to attach camera to (e.g., 'zed_left_camera_frame')
        frame_id: ROS frame_id for published messages
        topics: Dict with keys 'rgb', 'camera_info', and optionally 'depth', 'pointcloud'
        enable_depth: If True, also publish depth and point cloud
    """
    print(f"Setting up camera: {cam_name}...")

    cam_prim_path = f"{robot_path}/{parent_link}/Camera"
    _create_camera_prim(stage, cam_prim_path)

    # Build OmniGraph nodes dynamically based on enabled outputs
    nodes = [
        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
        ("CreateRenderProduct", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
        ("PublishRGB", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
        ("PublishCameraInfo", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
    ]
    values = [
        ("CreateRenderProduct.inputs:cameraPrim", cam_prim_path),
        ("CreateRenderProduct.inputs:width", CAMERA_WIDTH),
        ("CreateRenderProduct.inputs:height", CAMERA_HEIGHT),
        ("PublishRGB.inputs:topicName", topics["rgb"]),
        ("PublishRGB.inputs:frameId", frame_id),
        ("PublishRGB.inputs:type", "rgb"),
        ("PublishCameraInfo.inputs:topicName", topics["camera_info"]),
        ("PublishCameraInfo.inputs:frameId", frame_id),
        ("PublishCameraInfo.inputs:type", "camera_info"),
    ]
    connections = [
        ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
        ("CreateRenderProduct.outputs:execOut", "PublishRGB.inputs:execIn"),
        ("CreateRenderProduct.outputs:renderProductPath", "PublishRGB.inputs:renderProductPath"),
        ("PublishRGB.outputs:execOut", "PublishCameraInfo.inputs:execIn"),
        ("CreateRenderProduct.outputs:renderProductPath",
         "PublishCameraInfo.inputs:renderProductPath"),
    ]

    prev_node = "PublishCameraInfo"

    if enable_depth and "depth" in topics:
        nodes.append(("PublishDepth", "omni.isaac.ros2_bridge.ROS2CameraHelper"))
        values.extend([
            ("PublishDepth.inputs:topicName", topics["depth"]),
            ("PublishDepth.inputs:frameId", frame_id),
            ("PublishDepth.inputs:type", "depth"),
        ])
        connections.extend([
            (f"{prev_node}.outputs:execOut", "PublishDepth.inputs:execIn"),
            ("CreateRenderProduct.outputs:renderProductPath",
             "PublishDepth.inputs:renderProductPath"),
        ])
        prev_node = "PublishDepth"

    if enable_depth and "pointcloud" in topics:
        nodes.append(("PublishPC", "omni.isaac.ros2_bridge.ROS2CameraHelper"))
        values.extend([
            ("PublishPC.inputs:topicName", topics["pointcloud"]),
            ("PublishPC.inputs:frameId", frame_id),
            ("PublishPC.inputs:type", "depth_pcl"),
        ])
        connections.extend([
            (f"{prev_node}.outputs:execOut", "PublishPC.inputs:execIn"),
            ("CreateRenderProduct.outputs:renderProductPath",
             "PublishPC.inputs:renderProductPath"),
        ])

    graph = create_action_graph(f"{robot_path}/{cam_name}Graph", nodes, values, connections)

    for key, val in topics.items():
        print(f"  {key}: {val}")
    return graph


def setup_imu(stage, robot_path):
    """Set up IMU sensor with ROS 2 publisher."""
    print("Setting up IMU sensor...")
    imu_prim_path = f"{robot_path}/imu_link"

    graph = create_action_graph(
        graph_path=f"{robot_path}/IMUGraph",
        nodes=[
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("IsaacReadIMU", "omni.isaac.sensor.IsaacReadIMU"),
            ("PublishIMU", "omni.isaac.ros2_bridge.ROS2PublishImu"),
        ],
        values=[
            ("IsaacReadIMU.inputs:imuPrim", imu_prim_path),
            ("PublishIMU.inputs:topicName", TOPICS["imu"]),
            ("PublishIMU.inputs:frameId", "imu_link"),
        ],
        connections=[
            ("OnPlaybackTick.outputs:tick", "IsaacReadIMU.inputs:execIn"),
            ("IsaacReadIMU.outputs:execOut", "PublishIMU.inputs:execIn"),
            ("IsaacReadIMU.outputs:angVel", "PublishIMU.inputs:angularVelocity"),
            ("IsaacReadIMU.outputs:linAcc", "PublishIMU.inputs:linearAcceleration"),
            ("IsaacReadIMU.outputs:orientation", "PublishIMU.inputs:orientation"),
        ],
    )
    print(f"  Publishing to {TOPICS['imu']}")
    return graph
