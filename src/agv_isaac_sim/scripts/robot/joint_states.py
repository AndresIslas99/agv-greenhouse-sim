"""Joint state publisher OmniGraph setup."""

from .config import TOPICS
from .omnigraph_helpers import create_action_graph


def setup_joint_state_publisher(robot_path):
    """Create OmniGraph: read joint states → publish to ROS 2."""
    print("Setting up joint state publisher...")
    graph = create_action_graph(
        graph_path=f"{robot_path}/JointStatePublisher",
        nodes=[
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ReadJoints", "omni.isaac.core_nodes.IsaacReadJointState"),
            ("PublishJointStates", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
        ],
        values=[
            ("ReadJoints.inputs:prim", robot_path),
            ("PublishJointStates.inputs:topicName", TOPICS["joint_states"]),
        ],
        connections=[
            ("OnPlaybackTick.outputs:tick", "ReadJoints.inputs:execIn"),
            ("ReadJoints.outputs:execOut", "PublishJointStates.inputs:execIn"),
            ("ReadJoints.outputs:jointNames", "PublishJointStates.inputs:jointNames"),
            ("ReadJoints.outputs:positionCommand", "PublishJointStates.inputs:position"),
            ("ReadJoints.outputs:velocityCommand", "PublishJointStates.inputs:velocity"),
        ],
    )
    print(f"  Publishing to {TOPICS['joint_states']}")
    return graph
