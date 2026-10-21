"""Mesh connectivity helpers shared by analysis and scene export."""


def beam_mesh(cfg):
    """Return structural beam lines, their node IDs, and explicit node roles.

    Beam topology comes from beam-element connectivity and component typing.
    It does not depend on node names, node roles, or an ordering field.
    """
    nodes = {int(node["id"]): node for node in cfg.get("nodes", [])}
    components = cfg.get("components", {})
    lines = []
    beam_node_ids = set()
    for line in cfg.get("lines", []):
        component = components.get(line.get("component_id"), {})
        if (line.get("element_type") == "beam"
                and component.get("element_type") == "beam"
                and line.get("role") != "buoy_body"):
            lines.append(line)
            beam_node_ids.update(int(node_id) for node_id in line["nodes"])

    roles = {node_id: str(node.get("role", "structural")) for node_id, node in nodes.items()}
    return lines, sorted(beam_node_ids), roles
