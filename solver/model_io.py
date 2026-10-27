"""Model input/output supporting AquaSim .amodel XML and legacy JSON formats.

This module converts between WinchSim configuration dictionaries and AquaSim-compliant
XML documents that can be read and viewed directly by AmodelViewer.
"""
from __future__ import annotations

import math
from pathlib import Path
import xml.etree.ElementTree as ET
from xml.dom import minidom
import json


DEFAULT_COLORS = {
    "beam": {"red": 0.72, "green": 0.87, "blue": 0.12},       # Yellow-green
    "main_rope": {"red": 0.68, "green": 0.56, "blue": 0.92},  # Purple
    "upper_sling": {"red": 0.55, "green": 0.65, "blue": 0.95}, # Medium purple
    "lower_sling": {"red": 0.25, "green": 0.78, "blue": 0.75}, # Cyan/teal
    "buoy": {"red": 0.95, "green": 0.60, "blue": 0.15},       # Orange
    "truss": {"red": 0.68, "green": 0.56, "blue": 0.92},
}


def _safe_float(val, default=0.0):
    if val is None:
        return default
    try:
        return float(val)
    except (ValueError, TypeError):
        return default


def _safe_int(val, default=0):
    if val is None:
        return default
    try:
        return int(val)
    except (ValueError, TypeError):
        return default


def model_to_xml(cfg: dict) -> str:
    """Convert a WinchSim model dictionary into an AquaSim-compatible XML string."""
    project = cfg.get("project", {})
    env = cfg.get("environment", {})
    units = project.get("units", {})

    root = ET.Element("model", {
        "version": "2.5",
        "name": str(project.get("name", "WinchSim Model")),
        "description": str(project.get("description", "AquaSim model file")),
        "model_type": str(project.get("model_type", "global_quasi_static_equilibrium")),
        "author": "hui cheng",
        "lastVersion": "2.22.1",
        "units_length": str(units.get("length", "m")),
        "units_time": str(units.get("time", "s")),
        "units_mass": str(units.get("mass", "kg")),
        "units_force": str(units.get("force", "N")),
    })

    # 1. Environment
    ET.SubElement(root, "environment", {
        "waterline_z_m": str(env.get("waterline_z_m", 0.0)),
        "gravity_m_per_s2": str(env.get("gravity_m_per_s2", 9.81)),
    })

    # 2. Nodes
    nodes_elem = ET.SubElement(root, "Nodes")
    for node in cfg.get("nodes", []):
        pos = node.get("position_m", [0.0, 0.0, 0.0])
        node_attrs = {
            "id": str(node["id"]),
            "x": f"{float(pos[0]):.4f}".rstrip("0").rstrip("."),
            "y": f"{float(pos[1]):.4f}".rstrip("0").rstrip("."),
            "z": f"{float(pos[2]):.4f}".rstrip("0").rstrip("."),
        }
        if "name" in node and node["name"]:
            node_attrs["name"] = str(node["name"])
        if "role" in node and node["role"]:
            node_attrs["role"] = str(node["role"])
        ET.SubElement(nodes_elem, "node", node_attrs)

    # 3. Components & Elements
    components_elem = ET.SubElement(root, "Components")
    components_dict = cfg.get("components", {})
    elements_dict = cfg.get("elements", {})
    lines_list = cfg.get("lines", [])

    # Group lines by component
    lines_by_comp: dict[str, list] = {comp_id: [] for comp_id in components_dict}
    for line in lines_list:
        comp_id = str(line.get("component_id", ""))
        lines_by_comp.setdefault(comp_id, []).append(line)

    for comp_index, (comp_id, comp) in enumerate(components_dict.items(), start=1):
        elem_type = comp.get("element_type", "beam")
        tag_name = "beam" if elem_type == "beam" else "truss"
        prop_set_name = comp.get("property_set", elem_type)
        prop_set = elements_dict.get(prop_set_name, {})

        comp_attrs = {
            "id": str(comp_id),
            "name": str(comp.get("name", comp_id)),
            "property_set": str(prop_set_name),
        }
        if "structural_model" in comp:
            comp_attrs["structural_model"] = str(comp["structural_model"])
        if "rotation_behavior" in comp:
            comp_attrs["rotation_behavior"] = str(comp["rotation_behavior"])
        if "allowed_rope_components" in comp:
            comp_attrs["allowed_rope_components"] = ",".join(map(str, comp["allowed_rope_components"]))

        comp_elem = ET.SubElement(components_elem, tag_name, comp_attrs)

        # Section & Materials according to element type
        if tag_name == "beam":
            od_m = float(prop_set.get("waterline_transition_diameter_m",
                         prop_set.get("diameter_m", 0.5)))
            od_mm = od_m * 1000.0
            young = float(prop_set.get("youngs_modulus_pa", 1.1e9))
            shear = float(prop_set.get("shear_modulus_pa", 3.84e8))
            area = float(prop_set.get("cross_section_area_m2", math.pi * (od_m / 2)**2))
            iy = float(prop_set.get("second_moment_area_y_m4", 0.001693))
            iz = float(prop_set.get("second_moment_area_z_m4", 0.001693))
            it = float(prop_set.get("torsional_constant_m4", 0.003386))
            air_mass = float(prop_set.get("effective_line_mass_in_air_kg_per_m",
                             prop_set.get("line_mass_in_air_kg_per_m", 283.98)))
            water_mass = float(prop_set.get("effective_line_mass_in_water_kg_per_m", 82.72))
            water_load = float(prop_set.get("net_line_load_in_water_kgf_per_m", 0.0))

            ET.SubElement(comp_elem, "wizard", {
                "type": "circular",
                "outerDiameter": f"{od_mm:.1f}",
                "eModulus": f"{young:.4E}",
                "gModulus": f"{shear:.4E}",
            })
            mat_attrs = {
                "young": f"{young:.4E}",
                "shear": f"{shear:.4E}",
                "crossectionareal": f"{area:.6E}",
                "iy": f"{iy:.6E}",
                "iz": f"{iz:.6E}",
                "it": f"{it:.6E}",
                "weightInAir": f"{air_mass:.4f}",
                "effectiveLineMassInWater": f"{water_mass:.4f}",
            }
            if water_load != 0.0:
                mat_attrs["netLineLoadInWater"] = f"{water_load:.4f}"
            ET.SubElement(comp_elem, "materials", mat_attrs)

            # Color
            color_key = "buoy" if "buoy" in comp_id.lower() or comp.get("role") == "buoy_body" else "beam"
            col = DEFAULT_COLORS.get(color_key, DEFAULT_COLORS["beam"])
            desc_elem = ET.SubElement(comp_elem, "description", {"active": "true", "visible": "true"})
            ET.SubElement(desc_elem, "color", {
                "red": f"{col['red']:.4f}", "green": f"{col['green']:.4f}", "blue": f"{col['blue']:.4f}"
            })

        else: # truss
            young = float(prop_set.get("youngs_modulus_pa", 6.0e10))
            area = float(prop_set.get("cross_section_area_m2", 7.85e-5))
            air_mass = float(prop_set.get("line_mass_in_air_kg_per_m", 0.07615))
            water_mass = float(prop_set.get("net_line_mass_in_water_kg_per_m", -0.00043125))

            ET.SubElement(comp_elem, "mooring", {
                "weightInAir": f"{air_mass:.6f}",
                "netLineMassInWater": f"{water_mass:.8f}",
                "areal": f"{area:.6E}",
                "young": f"{young:.4E}",
                "noCompressionForces": "true",
            })

            # Color selection based on component name / role
            comp_name_lower = comp.get("name", "").lower()
            if "main" in comp_name_lower:
                col = DEFAULT_COLORS["main_rope"]
            elif "lower" in comp_name_lower:
                col = DEFAULT_COLORS["lower_sling"]
            elif "upper" in comp_name_lower:
                col = DEFAULT_COLORS["upper_sling"]
            else:
                col = DEFAULT_COLORS["truss"]

            desc_elem = ET.SubElement(comp_elem, "description", {"active": "true", "visible": "true"})
            ET.SubElement(desc_elem, "color", {
                "red": f"{col['red']:.4f}", "green": f"{col['green']:.4f}", "blue": f"{col['blue']:.4f}"
            })

        # Elements under this component
        elements_sub = ET.SubElement(comp_elem, "elements")
        for line in lines_by_comp.get(comp_id, []):
            endpoints = line["nodes"]
            elem_attrs = {
                "id": str(line["id"]),
                "StartNode_ID": str(endpoints[0]),
                "EndNode_ID": str(endpoints[1]),
            }
            if "name" in line and line["name"]:
                elem_attrs["name"] = str(line["name"])
            if "role" in line and line["role"]:
                elem_attrs["role"] = str(line["role"])
            if "subdivisions" in line:
                elem_attrs["subdivisions"] = str(line["subdivisions"])
            if "rest_length_m" in line:
                elem_attrs["rest_length_m"] = str(line["rest_length_m"])

            line_elem = ET.SubElement(elements_sub, "element", elem_attrs)

            # If this is the main rope or connects to winch, embed winch element for AquaSim compatibility
            if line.get("role") == "main_rope" and "winch" in cfg:
                winch_cfg = cfg["winch"]
                first_command = (winch_cfg.get("schedule") or [{}])[0]
                speed = first_command.get(
                    "speed_m_per_s", winch_cfg.get("payout_or_retrieval_speed_m_per_s", 0.1)
                )
                direction = {
                    "in": "upward",
                    "out": "downward",
                }.get(first_command.get("direction"), winch_cfg.get("direction", "upward"))
                ET.SubElement(line_elem, "winch", {
                    "name": "Winch 1",
                    "mode": str(winch_cfg.get("mode", "prescribed_speed")),
                    "velocity": str(speed),
                    "direction": str(direction),
                    "active": "true",
                    "node": str(endpoints[0]),
                })

    # 4. Winch Configuration
    if "winch" in cfg:
        winch = cfg["winch"]
        first_command = (winch.get("schedule") or [{}])[0]
        speed = first_command.get(
            "speed_m_per_s", winch.get("payout_or_retrieval_speed_m_per_s", 0.1)
        )
        direction = {
            "in": "upward",
            "out": "downward",
        }.get(first_command.get("direction"), winch.get("direction", "upward"))
        winch_attrs = {
            "mode": str(winch.get("mode", "prescribed_speed")),
            "payout_or_retrieval_speed_m_per_s": str(speed),
            "direction": str(direction),
        }
        if "reversal_time_s" in winch and not winch.get("schedule"):
            winch_attrs["reversal_time_s"] = str(winch["reversal_time_s"])
        winch_elem = ET.SubElement(root, "winch", winch_attrs)
        if winch.get("schedule"):
            schedule_elem = ET.SubElement(winch_elem, "motion_schedule")
            for row in winch["schedule"]:
                ET.SubElement(schedule_elem, "point", {
                    "time_s": str(row["time_s"]),
                    "speed_m_per_s": str(row["speed_m_per_s"]),
                    "direction": str(row["direction"]),
                })

    # 5. Solver Configuration
    if "solver" in cfg:
        solver_cfg = cfg["solver"]
        solver_attrs = {}
        for k, v in solver_cfg.items():
            if k == "numerics":
                continue
            solver_attrs[k] = str(v).lower() if isinstance(v, bool) else str(v)
        solver_elem = ET.SubElement(root, "solver", solver_attrs)

        if "numerics" in solver_cfg and isinstance(solver_cfg["numerics"], dict):
            num_dict = solver_cfg["numerics"]
            num_attrs = {}
            for nk, nv in num_dict.items():
                if isinstance(nv, list):
                    num_attrs[nk] = ",".join(map(str, nv))
                else:
                    num_attrs[nk] = str(nv)
            ET.SubElement(solver_elem, "numerics", num_attrs)

    # Pretty print XML
    raw_xml = ET.tostring(root, encoding="utf-8")
    dom = minidom.parseString(raw_xml)
    return dom.toprettyxml(indent="  ", encoding="utf-8").decode("utf-8")


def xml_to_model(xml_content_or_path) -> dict:
    """Parse an AquaSim-compatible XML model into a WinchSim configuration dictionary."""
    if isinstance(xml_content_or_path, (str, Path)) and (
        Path(str(xml_content_or_path)).is_file() or str(xml_content_or_path).strip().endswith(".xml") or str(xml_content_or_path).strip().endswith(".amodel")
    ):
        file_path = Path(str(xml_content_or_path))
        if file_path.exists():
            xml_text = file_path.read_text(encoding="utf-8-sig")
        else:
            xml_text = str(xml_content_or_path)
    else:
        xml_text = str(xml_content_or_path)

    root = ET.fromstring(xml_text)
    if root.tag.lower() != "model":
        # Maybe wrapped inside another tag or document
        model_child = root.find(".//model")
        if model_child is not None:
            root = model_child

    # 1. Project
    project = {
        "name": root.attrib.get("name", "WinchSim Model"),
        "description": root.attrib.get("description", ""),
        "model_type": root.attrib.get("model_type", "global_quasi_static_equilibrium"),
        "units": {
            "length": root.attrib.get("units_length", "m"),
            "time": root.attrib.get("units_time", "s"),
            "mass": root.attrib.get("units_mass", "kg"),
            "force": root.attrib.get("units_force", "N"),
        }
    }

    # 2. Environment
    env_elem = root.find("environment")
    if env_elem is None:
        env_elem = root.find(".//environment")
    if env_elem is not None:
        environment = {
            "waterline_z_m": _safe_float(env_elem.attrib.get("waterline_z_m", 0.0)),
            "gravity_m_per_s2": _safe_float(env_elem.attrib.get("gravity_m_per_s2", 9.81)),
        }
    else:
        environment = {"waterline_z_m": 0.0, "gravity_m_per_s2": 9.81}

    # 3. Nodes
    nodes = []
    # Search for nodes in Nodes > node or model > Nodes > node
    for node_el in root.findall(".//Nodes/node") or root.findall(".//node"):
        if "id" not in node_el.attrib:
            continue
        node_id = int(node_el.attrib["id"])
        x = _safe_float(node_el.attrib.get("x", 0.0))
        y = _safe_float(node_el.attrib.get("y", 0.0))
        z = _safe_float(node_el.attrib.get("z", 0.0))
        node_dict = {
            "id": node_id,
            "position_m": [x, y, z]
        }
        if "name" in node_el.attrib:
            node_dict["name"] = node_el.attrib["name"]
        if "role" in node_el.attrib:
            node_dict["role"] = node_el.attrib["role"]
        dof_el = node_el.find("dof6")
        if dof_el is not None:
            node_dict["dof6"] = {
                name: value.lower() == "true"
                for name, value in dof_el.attrib.items()
            }
        point_loads = []
        for load_el in node_el.findall("pointLoad"):
            if load_el.attrib.get("active", "true").lower() != "true":
                continue
            force_el = load_el.find("Force")
            if force_el is not None:
                point_loads.append([
                    _safe_float(force_el.attrib.get(axis)) for axis in ("x", "y", "z")
                ])
        if point_loads:
            node_dict["point_load_n"] = [
                sum(force[axis] for force in point_loads) for axis in range(3)
            ]
        if node_el.find("roller") is not None:
            roller_el = node_el.find("roller")
            node_dict["roller_constraint"] = {
                "element_a": _safe_int(roller_el.attrib.get("elementA")),
                "element_b": _safe_int(roller_el.attrib.get("elementB")),
                "last_element_a": _safe_int(roller_el.attrib.get("lastelementA")),
                "last_element_b": _safe_int(roller_el.attrib.get("lastelementB")),
            }
        nodes.append(node_dict)


    # 4. Components & Lines & Elements
    components = {}
    elements = {}
    lines = []

    comp_elements = root.findall(".//Components/*") or root.findall(".//beam") + root.findall(".//truss")

    for comp_el in comp_elements:
        tag = comp_el.tag.lower()
        if tag not in ("beam", "truss"):
            continue

        comp_id = comp_el.attrib.get("id") or comp_el.attrib.get("name") or f"{tag}_{len(components)+1}"
        comp_name = comp_el.attrib.get("name", comp_id)
        prop_set_name = comp_el.attrib.get("property_set", comp_id)

        comp_data = {
            "name": comp_name,
            "element_type": tag,
            "property_set": prop_set_name,
        }
        if "structural_model" in comp_el.attrib:
            comp_data["structural_model"] = comp_el.attrib["structural_model"]
        if "rotation_behavior" in comp_el.attrib:
            comp_data["rotation_behavior"] = comp_el.attrib["rotation_behavior"]
        if "allowed_rope_components" in comp_el.attrib:
            comp_data["allowed_rope_components"] = [
                value.strip() for value in comp_el.attrib["allowed_rope_components"].split(",")
                if value.strip()
            ]
        components[comp_id] = comp_data

        # Parse property set for elements dict
        if tag == "beam":
            wizard_el = comp_el.find("wizard")
            mat_el = comp_el.find("materials")

            od_m = 0.5
            if wizard_el is not None and "outerDiameter" in wizard_el.attrib:
                od_m = _safe_float(wizard_el.attrib["outerDiameter"]) / 1000.0

            young = 1.1e9
            shear = 3.84e8
            area = math.pi * (od_m / 2)**2
            iy = 0.001693
            iz = 0.001693
            it = 0.003386
            air_mass = 283.98
            water_mass = 82.721721
            water_load = None

            if mat_el is not None:
                young = _safe_float(mat_el.attrib.get("young"), young)
                shear = _safe_float(mat_el.attrib.get("shear") or mat_el.attrib.get("torsionmoduleg"), shear)
                area = _safe_float(mat_el.attrib.get("crossectionareal"), area)
                iy = _safe_float(mat_el.attrib.get("iy"), iy)
                iz = _safe_float(mat_el.attrib.get("iz"), iz)
                it = _safe_float(mat_el.attrib.get("it"), it)
                air_mass = _safe_float(mat_el.attrib.get("weightInAir") or mat_el.attrib.get("lineMassInAir"), air_mass)
                water_mass = _safe_float(mat_el.attrib.get("effectiveLineMassInWater"), water_mass)
                if "effectiveLineMassInWater" not in mat_el.attrib and "weightInWater" in mat_el.attrib:
                    water_mass = _safe_float(mat_el.attrib["weightInWater"]) / 9.81
                if "netLineLoadInWater" in mat_el.attrib:
                    water_load = _safe_float(mat_el.attrib["netLineLoadInWater"])

            prop_data = {
                "element_type": "beam",
                "youngs_modulus_pa": young,
                "shear_modulus_pa": shear,
                "cross_section_area_m2": area,
                "second_moment_area_y_m4": iy,
                "second_moment_area_z_m4": iz,
                "torsional_constant_m4": it,
            }
            if water_load is not None:
                prop_data["diameter_m"] = od_m
                prop_data["line_mass_in_air_kg_per_m"] = air_mass
                prop_data["net_line_load_in_water_kgf_per_m"] = water_load
            else:
                prop_data["effective_line_mass_in_water_kg_per_m"] = water_mass
                prop_data["effective_line_mass_in_air_kg_per_m"] = air_mass
                prop_data["waterline_transition_diameter_m"] = od_m

            elements[prop_set_name] = prop_data

        else: # truss
            mooring_el = comp_el.find("mooring")
            young = 6.0e10
            area = 7.85e-5
            air_mass = 0.07615
            water_mass = -0.00043125
            tension_only = True

            if mooring_el is not None:
                young = _safe_float(mooring_el.attrib.get("young"), young)
                area = _safe_float(mooring_el.attrib.get("areal"), area)
                air_mass = _safe_float(mooring_el.attrib.get("weightInAir"), air_mass)
                water_mass = _safe_float(mooring_el.attrib.get("netLineMassInWater"), water_mass)
                if "netLineMassInWater" not in mooring_el.attrib and "weightInWater" in mooring_el.attrib:
                    water_mass = _safe_float(mooring_el.attrib["weightInWater"]) / 9.81
                if "noCompressionForces" in mooring_el.attrib:
                    tension_only = mooring_el.attrib["noCompressionForces"].lower() == "true"

            elements[prop_set_name] = {
                "element_type": "truss",
                "youngs_modulus_pa": young,
                "cross_section_area_m2": area,
                "line_mass_in_air_kg_per_m": air_mass,
                "net_line_mass_in_water_kg_per_m": water_mass,
                "tension_only": tension_only,
                "interpretation_note": "Negative underwater line mass represents net buoyancy.",
            }

        # Elements inside component
        for elem_el in comp_el.findall(".//element"):
            elem_id_str = elem_el.attrib.get("id")
            start_str = elem_el.attrib.get("StartNode_ID") or elem_el.attrib.get("startNodeId")
            end_str = elem_el.attrib.get("EndNode_ID") or elem_el.attrib.get("endNodeId")

            if elem_id_str and start_str and end_str:
                line_dict = {
                    "id": int(elem_id_str),
                    "nodes": [int(start_str), int(end_str)],
                    "element_type": tag,
                    "component_id": comp_id,
                }
                if "name" in elem_el.attrib:
                    line_dict["name"] = elem_el.attrib["name"]
                if "role" in elem_el.attrib:
                    line_dict["role"] = elem_el.attrib["role"]
                if "subdivisions" in elem_el.attrib:
                    line_dict["subdivisions"] = int(elem_el.attrib["subdivisions"])
                if "rest_length_m" in elem_el.attrib:
                    line_dict["rest_length_m"] = float(elem_el.attrib["rest_length_m"])
                if "breaking_strength_n" in elem_el.attrib:
                    line_dict["breaking_strength_n"] = float(elem_el.attrib["breaking_strength_n"])

                lines.append(line_dict)

    # 5. Winch
    winch = {}
    legacy_duration_s = None
    winch_el = root.find("winch")
    if winch_el is None:
        winch_el = root.find(".//winch")
    if winch_el is not None:
        winch = {
            "mode": winch_el.attrib.get("mode", "prescribed_speed"),
            "payout_or_retrieval_speed_m_per_s": _safe_float(
                winch_el.attrib.get("payout_or_retrieval_speed_m_per_s") or winch_el.attrib.get("velocity"), 0.1
            ),
            "direction": winch_el.attrib.get("direction", "upward"),
        }
        legacy_duration = winch_el.attrib.get("duration_s") or winch_el.attrib.get("duration")
        if legacy_duration is not None:
            legacy_duration_s = _safe_float(legacy_duration, 150.0)
        if "reversal_time_s" in winch_el.attrib:
            winch["reversal_time_s"] = _safe_float(winch_el.attrib["reversal_time_s"])
        schedule_el = winch_el.find("motion_schedule")
        if schedule_el is not None:
            winch["schedule"] = [
                {
                    "time_s": float(point.attrib["time_s"]),
                    "speed_m_per_s": float(point.attrib["speed_m_per_s"]),
                    "direction": point.attrib["direction"],
                }
                for point in schedule_el.findall("point")
            ]
    else:
        winch = {
            "mode": "prescribed_speed",
            "payout_or_retrieval_speed_m_per_s": 0.0,
            "direction": "upward",
        }

    # 6. Solver
    solver = {}
    solver_el = root.find("solver")
    if solver_el is not None:
        for k, v in solver_el.attrib.items():
            if v.lower() == "true":
                solver[k] = True
            elif v.lower() == "false":
                solver[k] = False
            else:
                try:
                    if "." in v or "e" in v.lower():
                        solver[k] = float(v)
                    else:
                        solver[k] = int(v)
                except ValueError:
                    solver[k] = v

        num_el = solver_el.find("numerics")
        if num_el is not None:
            numerics = {}
            for nk, nv in num_el.attrib.items():
                if "," in nv:
                    try:
                        numerics[nk] = [int(x) if x.isdigit() else float(x) for x in nv.split(",")]
                    except ValueError:
                        numerics[nk] = nv.split(",")
                else:
                    try:
                        if "." in nv or "e" in nv.lower():
                            numerics[nk] = float(nv)
                        else:
                            numerics[nk] = int(nv)
                    except ValueError:
                        numerics[nk] = nv
            solver["numerics"] = numerics
    else:
        solver = {
            "analysis_method": "global_quasi_static_equilibrium",
            "default_truss_subdivisions": 1,
            "analysis_substeps": 30,
            "output_bin": "results/results.bin",
            "output_csv": "results/results.csv"
        }
    # An AquaSim model often has a <solver> element containing only run
    # settings (for example duration_s). Keep the same global-solver default
    # in that case instead of falling through to solve.py's mesh-preview
    # default, which requires WinchSim-specific upper_joint/main_rope roles.
    solver.setdefault("analysis_method", project["model_type"])
    # With no winch schedule or legacy duration, an AquaSim file describes a
    # single static configuration.  A duration is only needed for a motion
    # history; the equilibrium solver's output time increment is independent.
    if legacy_duration_s is not None:
        solver.setdefault("duration_s", legacy_duration_s)
    elif solver.get("duration_s") is None:
        solver["duration_s"] = 0.0
    solver.setdefault("time_step_s", 1.0)

    return {
        "project": project,
        "environment": environment,
        "nodes": sorted(nodes, key=lambda n: n["id"]),
        "lines": sorted(lines, key=lambda l: l["id"]),
        "components": components,
        "elements": elements,
        "winch": winch,
        "solver": solver,
    }


def load_model(file_path_or_str) -> tuple[dict, Path]:
    """Load a model from an XML or JSON file path, returning (cfg, resolved_path)."""
    path = Path(file_path_or_str).resolve()
    if not path.is_file():
        raise FileNotFoundError(f"Model file not found: {path}")

    suffix = path.suffix.lower()
    if suffix in (".xml", ".amodel"):
        cfg = xml_to_model(path)
    elif suffix == ".json":
        cfg = json.loads(path.read_text(encoding="utf-8-sig"))
    else:
        # Try XML first, fallback to JSON
        text = path.read_text(encoding="utf-8-sig").strip()
        if text.startswith("<"):
            cfg = xml_to_model(text)
        else:
            cfg = json.loads(text)

    # Older JSON inputs kept duration under winch; normalize it to the
    # global solver settings just like legacy XML inputs.
    if isinstance(cfg, dict):
        solver = cfg.setdefault("solver", {})
        winch = cfg.setdefault("winch", {})
        legacy_duration = winch.pop("duration_s", None)
        solver.setdefault("duration_s", legacy_duration if legacy_duration is not None else 150.0)

    return cfg, path
