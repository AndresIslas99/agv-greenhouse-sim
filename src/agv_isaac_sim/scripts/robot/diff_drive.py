"""Differential drive controller and odometry publisher OmniGraph setup."""

from .config import TOPICS, WHEEL_RADIUS, WHEEL_SEPARATION
from .omnigraph_helpers import create_action_graph


def setup_diff_drive_controller(robot_path):
    """Create OmniGraph: subscribe Twist → DiffController → ArticulationController."""
    print("Setting up differential drive controller...")
    graph = create_action_graph(
        graph_path=f"{robot_path}/DiffDriveController",
        nodes=[
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("SubscribeTwist", "omni.isaac.ros2_bridge.ROS2SubscribeTwist"),
            ("DiffController", "omni.isaac.wheeled_robots.DifferentialController"),
            ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
        ],
        values=[
            ("SubscribeTwist.inputs:topicName", TOPICS["cmd_vel"]),
            ("DiffController.inputs:wheelRadius", WHEEL_RADIUS),
            ("DiffController.inputs:wheelDistance", WHEEL_SEPARATION),
            ("ArticulationController.inputs:robotPath", robot_path),
            ("ArticulationController.inputs:jointNames",
             ["left_wheel_joint", "right_wheel_joint"]),
        ],
        connections=[
            ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
            ("SubscribeTwist.outputs:execOut", "DiffController.inputs:execIn"),
            ("SubscribeTwist.outputs:linearVelocity", "DiffController.inputs:linearVelocity"),
            ("SubscribeTwist.outputs:angularVelocity", "DiffController.inputs:angularVelocity"),
            ("DiffController.outputs:execOut", "ArticulationController.inputs:execIn"),
            ("DiffController.outputs:velocityCommand",
             "ArticulationController.inputs:velocityCommand"),
        ],
    )
    print(f"  Subscribing to {TOPICS['cmd_vel']}")
    return graph


def setup_odometry_publisher(robot_path):
    """Create OmniGraph: compute odometry → publish to ROS 2 + TF."""
    print("Setting up odometry publisher...")
    graph = create_action_graph(
        graph_path=f"{robot_path}/OdometryPublisher",
        nodes=[
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ComputeOdom", "omni.isaac.core_nodes.IsaacComputeOdometry"),
            ("PublishOdom", "omni.isaac.ros2_bridge.ROS2PublishOdometry"),
            ("PublishTF", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
        ],
        values=[
            ("ComputeOdom.inputs:chassisPrim", robot_path),
            ("PublishOdom.inputs:topicName", TOPICS["wheel_odom"]),
            ("PublishOdom.inputs:odomFrameId", "odom"),
            ("PublishOdom.inputs:chassisFrameId", "base_link"),
            ("PublishTF.inputs:topicName", TOPICS["tf"]),
        ],
        connections=[
            ("OnPlaybackTick.outputs:tick", "ComputeOdom.inputs:execIn"),
            ("ComputeOdom.outputs:execOut", "PublishOdom.inputs:execIn"),
            ("ComputeOdom.outputs:position", "PublishOdom.inputs:position"),
            ("ComputeOdom.outputs:orientation", "PublishOdom.inputs:orientation"),
            ("ComputeOdom.outputs:linearVelocity", "PublishOdom.inputs:linearVelocity"),
            ("ComputeOdom.outputs:angularVelocity", "PublishOdom.inputs:angularVelocity"),
            ("PublishOdom.outputs:execOut", "PublishTF.inputs:execIn"),
        ],
    )
    print(f"  Publishing to {TOPICS['wheel_odom']}")
    return graph
