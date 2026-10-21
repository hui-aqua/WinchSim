"""Shared result metadata and scene export for WinchSim solvers."""

from __future__ import annotations

from mesh_utils import beam_mesh

LOWER_SLING_RENDER_SEGMENTS = 24


def _vec(x, y, z):
    return [float(x), float(y), float(z)]


def _build_mesh_scene(cfg, rows):
    """Export the editable input mesh directly as the viewer scene graph."""
    first = rows[0]
    source_nodes = {int(node["id"]): node for node in cfg["nodes"]}
    _, beam_node_ids, inferred_roles = beam_mesh(cfg)
    scene_ids = {node_id: f"n-{node_id}" for node_id in source_nodes}
    role_map = {
        "beam_corner": "beam", "upper_joint": "joint", "lower_joint": "joint",
        "roller": "anchor", "winch": "anchor", "buoy_top": "buoy",
    }
    scene_nodes = []
    node_z_fields = {}
    for node_id, node in source_nodes.items():
        role = inferred_roles.get(node_id, node.get("role", "structural"))
        position = list(map(float, node["position_m"]))
        scene_role = "beam" if node_id in beam_node_ids and role == "structural" else role_map.get(role, role)
        if node_id in beam_node_ids:
            field = f"mesh_node_{node_id}_z_m"
            position[2] = float(first.get(field, first["beam_z_m"]))
            node_z_fields.setdefault(field, []).append(scene_ids[node_id])
        elif role == "upper_joint":
            position[2] = float(first["joint_z_m"])
            node_z_fields.setdefault("joint_z_m", []).append(scene_ids[node_id])
        elif role == "lower_joint":
            position[2] = float(first["lower_joint_z_m"])
            node_z_fields.setdefault("lower_joint_z_m", []).append(scene_ids[node_id])
        elif role == "buoy_top":
            position[2] = float(first.get("buoy_top_z_m", position[2]))
            node_z_fields.setdefault("buoy_top_z_m", []).append(scene_ids[node_id])
        scene_nodes.append({"id": scene_ids[node_id], "position": position,
                            "role": scene_role, "label": node.get("name", str(node_id))})

    elements, primitives = [], []
    node_position_fields = {}
    for line in cfg["lines"]:
        line_id = int(line["id"])
        role = line.get("role", "")
        component_id = line["component_id"]
        category = "beams" if line["element_type"] == "beam" else "trusses"
        endpoints = [scene_ids[int(value)] for value in line["nodes"]]

        if role == "buoy_body":
            bottom = source_nodes[int(line["nodes"][0])]
            top = source_nodes[int(line["nodes"][1])]
            if inferred_roles.get(int(line["nodes"][0])) == "buoy_top":
                bottom, top = top, bottom
            bottom_xyz, top_xyz = bottom["position_m"], top["position_m"]
            buoy = cfg["elements"].get("buoy", {})
            primitives.append({
                "id": component_id, "component_id": component_id, "type": "cylinder",
                "center": _vec(bottom_xyz[0], bottom_xyz[1], first["buoy_bottom_z_m"]),
                "radius": float(buoy["diameter_m"]) / 2,
                "height": abs(float(top_xyz[2]) - float(bottom_xyz[2])),
                "role": "buoy", "category": "buoys",
            })
            continue

        visual_role = "beam" if line["element_type"] == "beam" else "rope"
        if role == "lower_sling":
            subdivisions = int(line.get("subdivisions", 1))
            if subdivisions < 1:
                raise ValueError(f"line {line_id} subdivisions must be at least 1")
            internal_ids = []
            for j in range(1, subdivisions):
                node_id = f"line-{line_id}-p{j}"
                internal_ids.append(node_id)
                fields = {axis: f"lower_sling_{line_id}_profile_{j}_{axis}_m" for axis in ("x", "y", "z")}
                a, b = (source_nodes[int(v)]["position_m"] for v in line["nodes"])
                u = j / subdivisions
                position = [float(a[k]) * (1 - u) + float(b[k]) * u for k in range(3)]
                for axis_index, axis in enumerate(("x", "y", "z")):
                    value = first.get(fields[axis])
                    if value is not None:
                        position[axis_index] = float(value)
                scene_nodes.append({"id": node_id, "position": position, "role": "rope"})
                node_position_fields[node_id] = fields
            endpoint_roles = [inferred_roles[int(value)] for value in line["nodes"]]
            if endpoint_roles[0] == "lower_joint":
                ordered_endpoints = endpoints
            else:
                ordered_endpoints = list(reversed(endpoints))
            elements.append({"id": f"line-{line_id}", "name": line.get("name"),
                             "type": "polyline", "nodes": [ordered_endpoints[0], *internal_ids, ordered_endpoints[1]],
                             "role": visual_role, "category": category, "component_id": component_id})
        elif role == "main_rope" and {inferred_roles[int(value)] for value in line["nodes"]} == {"roller", "upper_joint"}:
            mid_id = f"line-{line_id}-slack-mid"
            a, b = (source_nodes[int(v)]["position_m"] for v in line["nodes"])
            mid = _vec((float(a[0]) + float(b[0])) / 2, (float(a[1]) + float(b[1])) / 2,
                       first.get("main_rope_slack_mid_z_m", (float(a[2]) + float(b[2])) / 2))
            scene_nodes.append({"id": mid_id, "position": mid, "role": "rope"})
            node_z_fields.setdefault("main_rope_slack_mid_z_m", []).append(mid_id)
            elements.append({"id": f"line-{line_id}", "name": line.get("name"),
                             "type": "polyline", "nodes": [endpoints[0], mid_id, endpoints[1]],
                             "role": visual_role, "category": category, "component_id": component_id})
        else:
            elements.append({"id": f"line-{line_id}", "name": line.get("name"),
                             "type": "segment", "nodes": endpoints, "role": visual_role,
                             "category": category, "component_id": component_id})

    components = []
    for component_id, component in cfg.get("components", {}).items():
        element_type = component.get("element_type", "truss")
        category = "beams" if element_type == "beam" else "trusses"
        if component_id in {p["component_id"] for p in primitives}:
            category = "buoys"
        components.append({"id": component_id, "label": component.get("name", component_id),
                           "category": category})
    return {
        "schema": "winchsim.scene.v1",
        "coordinate_system": {"handedness": "right", "up": "z",
                              "units": cfg.get("project", {}).get("units", {}).get("length", "m")},
        "waterline_z": float(cfg["environment"]["waterline_z_m"]),
        "nodes": scene_nodes, "elements": elements, "primitives": primitives,
        "components": components,
        "animation": {"node_z_fields": node_z_fields,
                      "node_position_fields": node_position_fields,
                      "primitive_z_fields": {"buoy_bottom_z_m": [p["id"] for p in primitives]}},
    }


def build_scene(cfg, rows):
    """Build an application-neutral scene graph from model configuration."""
    if "nodes" in cfg and "lines" in cfg:
        return _build_mesh_scene(cfg, rows)
    geom = cfg["geometry"]
    first = rows[0]
    side = float(geom.get("beam_side_m", 0))
    corners = geom.get("beam_corner_xy_m") or [
        [-side / 2, -side / 2], [side / 2, -side / 2],
        [side / 2, side / 2], [-side / 2, side / 2],
    ]
    beam_z = float(first["beam_z_m"])
    upper = _vec(0, 0, first.get("joint_z_m", beam_z))
    nodes = [{"id": "beam-" + str(i + 1), "position": _vec(p[0], p[1], beam_z), "role": "beam"}
             for i, p in enumerate(corners)]
    elements = [{"id": "beam-edge-" + str(i + 1), "type": "segment", "nodes": ["beam-" + str(a + 1), "beam-" + str(b + 1)], "role": "beam"}
                for i, (a, b) in enumerate([(0, 1), (1, 2), (2, 3), (3, 0)])]
    nodes.append({"id": "upper-joint", "position": upper, "role": "joint"})
    winch = geom.get("fixed_winch_xyz_m", [0, 0, 20])
    nodes.append({"id": "winch", "position": list(map(float, winch)), "role": "anchor"})
    roller = geom.get("roller_xyz_m")
    if roller:
        nodes.append({"id": "roller", "position": list(map(float, roller)), "role": "anchor"})
        nodes.append({"id": "main-rope-slack-mid", "position": upper.copy(), "role": "rope"})
        elements.extend([
            {"id": "main-rope-lead", "type": "segment", "nodes": ["winch", "roller"], "role": "rope"},
            {"id": "main-rope", "type": "polyline", "nodes": ["roller", "main-rope-slack-mid", "upper-joint"], "role": "rope"},
        ])
    else:
        elements.append({"id": "main-rope", "type": "segment", "nodes": ["winch", "upper-joint"], "role": "rope"})
    for i in range(len(corners)):
        elements.append({"id": "upper-sling-" + str(i + 1), "type": "segment", "nodes": ["upper-joint", "beam-" + str(i + 1)], "role": "rope"})

    node_position_fields = {}
    if "lower_joint_z_m" in first:
        nodes.append({"id": "lower-joint", "position": _vec(0, 0, first["lower_joint_z_m"]), "role": "joint"})
        buoy_xy = geom.get("buoy_center_xy_m", [0, 0])
        for i, corner in enumerate(corners):
            profile_ids = []
            for j in range(1, LOWER_SLING_RENDER_SEGMENTS):
                u = j / LOWER_SLING_RENDER_SEGMENTS
                node_id = f"lower-sling-{i + 1}-profile-{j}"
                profile_ids.append(node_id)
                fields = {axis: f"lower_sling_{i + 1}_profile_{j}_{axis}_m" for axis in ("x", "y", "z")}
                position = _vec(first.get(fields["x"], float(buoy_xy[0]) * (1-u) + float(corner[0]) * u),
                                first.get(fields["y"], float(buoy_xy[1]) * (1-u) + float(corner[1]) * u),
                                first.get(fields["z"], float(first["lower_joint_z_m"]) * (1-u) + beam_z*u))
                nodes.append({"id": node_id, "position": position, "role": "rope"})
                node_position_fields[node_id] = fields
            elements.append({"id": "lower-sling-" + str(i + 1), "type": "polyline",
                             "nodes": ["lower-joint", *profile_ids, "beam-" + str(i + 1)], "role": "rope"})
    primitives = []
    if "buoy_bottom_z_m" in first and all(k in geom for k in ("buoy_diameter_m", "buoy_height_m")):
        center = geom.get("buoy_center_xy_m", [0, 0])
        primitives.append({"id": "buoy", "type": "cylinder", "center": _vec(center[0], center[1], first["buoy_bottom_z_m"]),
                           "radius": float(geom["buoy_diameter_m"]) / 2, "height": float(geom["buoy_height_m"]), "role": "buoy"})
    anim_nodes = {"beam_z_m": ["beam-" + str(i + 1) for i in range(len(corners))], "joint_z_m": ["upper-joint"]}
    if "lower_joint_z_m" in first:
        anim_nodes["lower_joint_z_m"] = ["lower-joint"]
    if roller:
        anim_nodes["main_rope_slack_mid_z_m"] = ["main-rope-slack-mid"]
    components = []
    for element in elements:
        category = element.get("category") or ("beams" if element.get("role") == "beam" else "trusses")
        element["category"] = category
        components.append({"id": element["id"], "label": element["id"].replace("-", " ").capitalize(), "category": category})
    for primitive in primitives:
        primitive["category"] = "buoys"
        components.append({"id": primitive["id"], "label": primitive["id"].capitalize(), "category": "buoys"})
    return {
        "schema": "winchsim.scene.v1",
        "coordinate_system": {"handedness": "right", "up": "z", "units": cfg.get("project", {}).get("units", {}).get("length", "m")},
        "waterline_z": float(geom.get("waterline_z_m", 0)),
        "nodes": nodes, "elements": elements, "primitives": primitives,
        "components": components,
        "animation": {"node_z_fields": anim_nodes, "node_position_fields": node_position_fields,
                      "primitive_z_fields": {"buoy_bottom_z_m": ["buoy"]} if any(p["id"] == "buoy" for p in primitives) else {}},
    }


def metric_definitions(rows):
    """Describe numeric outputs; renderers can display these without field-specific code."""
    units = {"time_s": "s", "_n": "N", "_m": "m", "_kg": "kg", "_per_m": "kg/m", "_pa": "Pa", "_s": "s"}
    preferred = ["support_reaction_z_quasistatic_n", "beam_z_m", "joint_z_m", "main_rope_length_m", "rope_signed_line_load_equivalent_kg"]
    keys = list(rows[0]) if rows else []
    keys = [key for key in preferred if key in keys] + [key for key in keys if key not in preferred and key != "time_s"]
    definitions = []
    for key in keys:
        if "_profile_" in key or key.startswith("mesh_node_"):
            continue
        value = rows[0].get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            continue
        match = next(((suffix, unit) for suffix, unit in units.items() if key.endswith(suffix)), None)
        unit = match[1] if match else ""
        label = key[:-len(match[0])] if match else key
        definitions.append({"field": key, "label": label.replace("_", " ").capitalize(), "unit": unit})
    return definitions

