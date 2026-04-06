"""Helper for creating OmniGraph action graphs (DRY wrapper)."""

import omni.graph.core as og


def create_action_graph(graph_path, nodes, values, connections):
    """Create an OmniGraph action graph with the given nodes, values, and connections.

    Replaces the repeated og.Controller.edit() boilerplate across all setup functions.

    Args:
        graph_path: USD path for the graph (e.g., "/Robot/DiffDriveController")
        nodes: List of (name, type) tuples for CREATE_NODES
        values: List of (attr, value) tuples for SET_VALUES
        connections: List of (src, dst) tuples for CONNECT

    Returns:
        The created graph object.
    """
    keys = og.Controller.Keys
    (graph, _, _, _) = og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: nodes,
            keys.SET_VALUES: values,
            keys.CONNECT: connections,
        }
    )
    return graph
