#!/usr/bin/env python3
"""WinchSim model runner for node/line JSON inputs.

Run ``python solver/solve.py [input-file.json]``. Ordinary mesh preview models
use the kinematic load estimator; models selecting lower-sling profile
analysis use the specialized discrete-truss path solver through this same
entry point.
"""
from pathlib import Path
import argparse
import json
import math
from result_schema import build_scene, metric_definitions
from winch_motion import prescribed_motion
from result_io import write_results
from hydrostatics import segment_air_fraction
from mesh_utils import beam_mesh

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_INPUT = ROOT / "input" / "model.json"


def clamp(value, low, high):
    return max(low, min(high, value))


def submerged_fraction(z1, z2, waterline):
    if abs(z2 - z1) < 1e-12:
        return 1.0 if z1 < waterline else 0.0
    if z1 < waterline and z2 < waterline:
        return 1.0
    if z1 >= waterline and z2 >= waterline:
        return 0.0
    crossing = (waterline - z1) / (z2 - z1)
    return clamp(crossing if z1 < waterline else 1.0 - crossing, 0.0, 1.0)


def validate_mesh(cfg):
    nodes = {int(node["id"]): node for node in cfg.get("nodes", [])}
    lines = cfg.get("lines", [])
    if not nodes or not lines:
        raise ValueError("Input must define numbered nodes and connected lines")
    if len(nodes) != len(cfg["nodes"]):
        raise ValueError("Node ids must be unique")
    components = cfg.get("components", {})
    elements = cfg.get("elements", {})
    line_ids = set()
    for node_id, node in nodes.items():
        position = node.get("position_m", [])
        if len(position) != 3 or any(not math.isfinite(float(value)) for value in position):
            raise ValueError(f"Node {node_id} position_m must contain three finite coordinates")
    for line in lines:
        line_id = int(line["id"])
        if line_id in line_ids:
            raise ValueError(f"Duplicate line id: {line_id}")
        line_ids.add(line_id)
        endpoints = [int(value) for value in line.get("nodes", [])]
        if len(endpoints) != 2 or any(node_id not in nodes for node_id in endpoints):
            raise ValueError(f"Line {line_id} must connect two existing node ids")
        component = components.get(line.get("component_id"))
        if component is None:
            raise ValueError(f"Line {line_id} references an unknown component")
        if line.get("element_type") != component.get("element_type"):
            raise ValueError(f"Line {line_id} element_type must match its component")
        property_set = component.get("property_set")
        if property_set not in elements:
            raise ValueError(f"Component {line['component_id']} references missing elements.{property_set}")
        if property_set == "truss":
            if float(elements[property_set]["line_mass_in_air_kg_per_m"]) < 0:
                raise ValueError("Truss line mass in air must be non-negative")
        line["id"], line["nodes"] = line_id, endpoints

    beam_lines, beam_node_ids, roles = beam_mesh(cfg)
    upper_joints = [node_id for node_id, role in roles.items() if role == "upper_joint"]
    main_lines = [line for line in lines if line.get("role") == "main_rope"]
    if len(beam_node_ids) < 2 or len(upper_joints) != 1 or not beam_lines or not main_lines:
        raise ValueError("Mesh needs connected beam elements, one upper_joint, and main_rope lines")
    if float(cfg["solver"]["time_step_s"]) <= 0:
        raise ValueError("solver.time_step_s must be positive")
    if float(cfg["winch"]["duration_s"]) < 0:
        raise ValueError("winch.duration_s must be non-negative")
    if float(cfg["environment"]["gravity_m_per_s2"]) <= 0:
        raise ValueError("environment.gravity_m_per_s2 must be positive")
    return {"nodes": nodes, "roles": roles, "beam_node_ids": beam_node_ids,
            "upper_joint_id": upper_joints[0], "beam_lines": beam_lines,
            "main_lines": main_lines,
            "upper_lines": [line for line in lines if line.get("role") == "upper_sling"],
            "truss_lines": [line for line in lines if line["element_type"] == "truss"]}


def calculate_row(time_s, cfg, model):
    nodes, roles = model["nodes"], model["roles"]
    positions = {node_id: list(map(float, node["position_m"])) for node_id, node in nodes.items()}
    displacement, _ = prescribed_motion(cfg["winch"], time_s)
    for node_id, role in roles.items():
        if node_id in model["beam_node_ids"] or role == "upper_joint":
            positions[node_id][2] += displacement
    beam_z = sum(positions[node_id][2] for node_id in model["beam_node_ids"]) / len(model["beam_node_ids"])
    joint_z = positions[model["upper_joint_id"]][2]
    waterline = float(cfg["environment"]["waterline_z_m"])
    gravity = float(cfg["environment"]["gravity_m_per_s2"])

    beam_length = 0.0
    beam_mass = 0.0
    beam_air_fraction = 0.0
    for line in model["beam_lines"]:
        component = cfg["components"][line["component_id"]]
        beam = cfg["elements"][component["property_set"]]
        length = math.dist(*(positions[node_id] for node_id in line["nodes"]))
        transition = float(beam.get("waterline_transition_diameter_m", 0.0))
        z1, z2 = (positions[node_id][2] for node_id in line["nodes"])
        air_fraction = segment_air_fraction(z1, z2, waterline, transition)
        line_mass = (float(beam["effective_line_mass_in_water_kg_per_m"])
                     + (float(beam["effective_line_mass_in_air_kg_per_m"])
                        - float(beam["effective_line_mass_in_water_kg_per_m"])) * air_fraction)
        beam_length += length
        beam_mass += length * line_mass
        beam_air_fraction += length * air_fraction
    average_beam_line_mass = beam_mass / beam_length if beam_length else 0.0
    average_beam_air_fraction = beam_air_fraction / beam_length if beam_length else 0.0

    lengths_by_role = {"main_rope": 0.0, "upper_sling": 0.0}
    wet_by_role = {"main_rope": 0.0, "upper_sling": 0.0}
    rope_air_length = 0.0
    rope_water_length = 0.0
    rope_signed_mass = 0.0
    horizontal_main = 0.0
    vertical_main = 0.0
    slack_mid_z = None
    for line in model["truss_lines"]:
        role = line.get("role")
        if role not in lengths_by_role:
            continue
        a_id, b_id = line["nodes"]
        a, b = positions[a_id], positions[b_id]
        length = math.dist(a, b)
        wet_length = length * submerged_fraction(a[2], b[2], waterline)
        air_length = length - wet_length
        component = cfg["components"][line["component_id"]]
        rope = cfg["elements"][component["property_set"]]
        air_mass = float(rope["line_mass_in_air_kg_per_m"])
        water_mass = float(rope["net_line_mass_in_water_kg_per_m"])
        rope_air_length += air_length
        rope_water_length += wet_length
        rope_signed_mass += air_length * air_mass + wet_length * water_mass
        lengths_by_role[role] += length
        wet_by_role[role] += wet_length
        endpoint_roles = {roles[a_id], roles[b_id]}
        if role == "main_rope":
            if endpoint_roles == {"winch", "roller"}:
                horizontal_main += length
            elif "roller" in endpoint_roles and "upper_joint" in endpoint_roles:
                vertical_main += length
                slack_mid_z = (a[2] + b[2]) / 2.0

    row = {
        "time_s": round(time_s, 6), "beam_z_m": beam_z, "joint_z_m": joint_z,
        "main_rope_length_m": lengths_by_role["main_rope"],
        "beam_air_fraction": average_beam_air_fraction,
        "beam_line_mass_equivalent_kg_per_m": average_beam_line_mass,
        "beam_mass_equivalent_kg": beam_mass,
        "main_rope_air_length_m": max(0.0, lengths_by_role["main_rope"] - wet_by_role["main_rope"]),
        "main_rope_water_length_m": wet_by_role["main_rope"],
        "all_slings_air_length_m": max(0.0, lengths_by_role["upper_sling"] - wet_by_role["upper_sling"]),
        "all_slings_water_length_m": wet_by_role["upper_sling"],
        "rope_air_length_total_m": rope_air_length,
        "rope_water_length_total_m": rope_water_length,
        "rope_signed_line_load_equivalent_kg": rope_signed_mass,
        "support_reaction_z_quasistatic_n": (beam_mass + rope_signed_mass) * gravity,
        "support_reaction_horizontal_n": 0.0,
    }
    if horizontal_main or vertical_main:
        row["main_rope_horizontal_length_m"] = horizontal_main
        row["main_rope_vertical_length_m"] = vertical_main
    if slack_mid_z is not None:
        row["main_rope_slack_mid_z_m"] = slack_mid_z
    row.update({f"mesh_node_{node_id}_z_m": positions[node_id][2]
                for node_id in model["beam_node_ids"]})
    row["support_reaction_magnitude_quasistatic_n"] = abs(row["support_reaction_z_quasistatic_n"])
    return row


def run_mesh_model(cfg, input_file, output_json=None, output_csv=None):
    solver_cfg = cfg.get("solver", {})
    if solver_cfg.get("dynamic_effects_enabled", False):
        raise NotImplementedError(
            "Dynamic analysis is not implemented. Set solver.dynamic_effects_enabled to false "
            "or select a supported quasi-static analysis method."
        )
    analysis_method = solver_cfg.get("analysis_method", "quasi_static_mesh_preview")
    if analysis_method == "quasi_static_lower_sling_profiles":
        from analysis.roller_buoy import run_model
        return run_model(cfg, input_file, output_json, output_csv)
    if analysis_method != "quasi_static_mesh_preview":
        raise ValueError(
            f"Unsupported solver.analysis_method {analysis_method!r}. "
            "Available methods: quasi_static_mesh_preview, quasi_static_lower_sling_profiles"
        )
    model = validate_mesh(cfg)
    dt = float(cfg["solver"]["time_step_s"])
    duration = float(cfg["winch"]["duration_s"])
    rows = [calculate_row(i * dt, cfg, model) for i in range(int(math.floor(duration / dt)) + 1)]
    if not rows or rows[-1]["time_s"] < duration - 1e-9:
        rows.append(calculate_row(duration, cfg, model))
    try:
        input_name = str(Path(input_file).resolve().relative_to(ROOT))
    except ValueError:
        input_name = str(Path(input_file).resolve())
    result = {
        "project": cfg["project"], "model_status": "quasi_static_mesh_preview",
        "input_file": input_name, "scene": build_scene(cfg, rows),
        "metrics": metric_definitions(rows),
        "mesh": {
            "nodes": [{"id": node_id, "name": node.get("name"), "role": node.get("role"),
                       "position_m": node["position_m"]} for node_id, node in model["nodes"].items()],
            "lines": cfg["lines"], "components": cfg["components"]
        },
        "results_count": len(rows), "fields": list(rows[0]), "results": rows
    }
    out_json, out_csv = write_results(rows, result, cfg, ROOT, output_json, output_csv)
    print(f"Wrote {len(rows)} time points to {out_json} and {out_csv}")
    print(f"Initial quasi-static support reaction: {rows[0]['support_reaction_z_quasistatic_n'] / 1000:.3f} kN")
    print(f"Final quasi-static support reaction: {rows[-1]['support_reaction_z_quasistatic_n'] / 1000:.3f} kN")
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description="Run a WinchSim node/line input model")
    parser.add_argument("input", nargs="?", default=str(DEFAULT_INPUT), help="Model JSON input file")
    parser.add_argument("--output-json", help="Override the JSON result path")
    parser.add_argument("--output-csv", help="Override the CSV result path")
    args = parser.parse_args(argv)
    input_file = Path(args.input).resolve()
    cfg = json.loads(input_file.read_text(encoding="utf-8-sig"))
    run_mesh_model(cfg, input_file, args.output_json, args.output_csv)


if __name__ == "__main__":
    main()
