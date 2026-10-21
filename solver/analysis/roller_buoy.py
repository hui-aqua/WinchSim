#!/usr/bin/env python3
"""Roller-buoy quasi-static lower-sling profile analysis.

The input mesh defines the route and material properties. Prescribed winch
motion drives a simplified body-motion rule; lower sling profiles are
minimized under signed submerged line loads with fixed link lengths. This is
not a global structural equilibrium or transient dynamics solver.
"""
from pathlib import Path
import math
from result_schema import build_scene, metric_definitions
from winch_motion import prescribed_motion
from result_io import write_results
from hydrostatics import segment_air_fraction
from mesh_utils import beam_mesh

ROOT = Path(__file__).resolve().parents[2]

DEFAULT_NUMERICS = {
    "path_gradient_iterations": 140,
    "path_initial_step_fraction": 0.03,
    "path_minimum_step_m": 1e-5,
    "path_maximum_step_fraction": 0.12,
    "path_step_growth": 1.04,
    "path_minimum_step_fraction": 1e-7,
    "bend_stiffness_ratio": 0.002,
    "link_relative_tolerance": 1e-4,
    "link_absolute_tolerance_m": 1e-8,
    "initialization_bisection_iterations": 80,
    "initial_projection_sweeps": 18,
    "trial_projection_sweeps": 5,
    "final_projection_sweeps": 400,
    "projection_retry_sweeps": [800, 1600, 3200, 6400],
    "floating_contact_tolerance_m": 0.01,
}


def numerical_options(cfg):
    return {**DEFAULT_NUMERICS, **cfg.get("solver", {}).get("numerics", {})}


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def submerged_fraction(z1, z2, waterline):
    if abs(z2 - z1) < 1e-12:
        return 1.0 if z1 < waterline else 0.0
    if z1 < waterline and z2 < waterline:
        return 1.0
    if z1 >= waterline and z2 >= waterline:
        return 0.0
    crossing = (waterline - z1) / (z2 - z1)
    return clamp(crossing if z1 < waterline else 1.0 - crossing, 0.0, 1.0)


def rope_load_at(z, waterline, air_line_mass, submerged_line_mass, transition_half_width):
    """Signed truss line load with a finite waterline transition band."""
    if transition_half_width <= 0:
        return submerged_line_mass if z < waterline else air_line_mass
    u = clamp((z - (waterline - transition_half_width)) / (2 * transition_half_width), 0.0, 1.0)
    return submerged_line_mass + (air_line_mass - submerged_line_mass) * u


def rope_load_potential(z, waterline, air_line_mass, submerged_line_mass, half_width):
    """Integral of signed line load over height, continuous at the waterline."""
    dz = z - waterline
    if half_width <= 0:
        return (submerged_line_mass if dz < 0 else air_line_mass) * dz
    if dz <= -half_width:
        return submerged_line_mass * dz
    if dz >= half_width:
        return air_line_mass * dz
    offset = dz + half_width
    return (-submerged_line_mass * half_width
            + submerged_line_mass * offset
            + (air_line_mass - submerged_line_mass) * offset * offset / (4 * half_width))


def equilibrate_truss_path(start, end, material_length, waterline, air_line_mass,
                           submerged_line_mass, gravity, transition_half_width, segments,
                           numerics=None):
    """Minimize gravitational potential for an inextensible discrete truss.

    Every link has fixed material length. The signed-load potential has a
    minimum at the waterline when the submerged load is buoyant and the air
    load is heavy, so floating contact emerges from equilibrium rather than a
    prescribed contact flag or guessed profile.
    """
    numerics = {**DEFAULT_NUMERICS, **(numerics or {})}
    rest = material_length / segments
    chord = math.dist(start, end)
    endpoint_tolerance = max(1e-8, material_length * 1e-10)
    if chord > material_length + endpoint_tolerance:
        raise ValueError(
            f"Endpoint chord ({chord:.6g} m) exceeds the truss material length "
            f"({material_length:.6g} m) by {chord - material_length:.6g} m"
        )
    slack = max(0.0, material_length - chord)
    if slack <= max(1e-10, material_length * 1e-10):
        # A taut, inextensible sling has exactly one feasible profile here:
        # its straight chord. Buoyancy cannot create length that is not present.
        return [[start[k] + (end[k] - start[k]) * j / segments for k in range(3)]
                for j in range(segments + 1)]
    if segments < 2:
        raise ValueError("A slack truss line requires at least two subdivisions")
    # Seed with a circular chain whose endpoints and every link length are
    # feasible by construction. Starting from a generic sinusoid can leave a
    # nearly straight chain far from the fixed-link constraint manifold.
    axis = [(end[k] - start[k]) / chord for k in range(3)]
    preferred = [0.0, 0.0, 1.0 if start[2] < waterline and submerged_line_mass < 0 else -1.0]
    axial = sum(preferred[k] * axis[k] for k in range(3))
    normal = [preferred[k] - axial * axis[k] for k in range(3)]
    normal_norm = math.sqrt(sum(v * v for v in normal))
    if normal_norm < 1e-10:
        fallback = [-(end[1] - start[1]), end[0] - start[0], 0.0]
        fallback_norm = math.sqrt(sum(v * v for v in fallback))
        normal = ([1.0, 0.0, 0.0] if fallback_norm < 1e-10
                  else [v / fallback_norm for v in fallback])
    else:
        normal = [v / normal_norm for v in normal]

    def chord_for_step_angle(angle):
        if abs(angle) < 1e-12:
            return material_length
        return rest * math.sin(segments * angle / 2.0) / math.sin(angle / 2.0)

    low_angle, high_angle = 0.0, 2.0 * math.pi / segments
    for _ in range(int(numerics["initialization_bisection_iterations"])):
        middle_angle = 0.5 * (low_angle + high_angle)
        if chord_for_step_angle(middle_angle) > chord:
            low_angle = middle_angle
        else:
            high_angle = middle_angle
    step_angle = 0.5 * (low_angle + high_angle)
    radius = rest / (2.0 * math.sin(step_angle / 2.0))
    half_sweep = segments * step_angle / 2.0
    midpoint = [(start[k] + end[k]) / 2.0 for k in range(3)]
    points = []
    for j in range(segments + 1):
        angle = (j - segments / 2.0) * step_angle
        points.append([
            midpoint[k]
            + axis[k] * radius * math.sin(angle)
            + normal[k] * radius * (math.cos(angle) - math.cos(half_sweep))
            for k in range(3)
        ])
    points[0], points[-1] = list(start), list(end)

    # Permit at most 0.01% link-length error, avoiding futile iterations below
    # the model's useful precision while keeping the discrete rope effectively
    # inextensible.
    link_tolerance = max(float(numerics["link_absolute_tolerance_m"]),
                         rest * float(numerics["link_relative_tolerance"]))

    def project_lengths(path, sweeps=6):
        """Project a chain with FABRIK, keeping the end nodes pinned."""
        path[0], path[-1] = list(start), list(end)
        for _ in range(sweeps):
            # Backward pass fixes the far endpoint and makes every link exact.
            path[-1] = list(end)
            for i in range(segments - 1, -1, -1):
                delta = [path[i][k] - path[i + 1][k] for k in range(3)]
                distance = math.sqrt(sum(v * v for v in delta))
                if distance < 1e-12:
                    delta, distance = [-axis[k] for k in range(3)], 1.0
                path[i] = [path[i + 1][k] + rest * delta[k] / distance for k in range(3)]

            # Forward pass fixes the near endpoint and restores exact links.
            path[0] = list(start)
            for i in range(segments):
                delta = [path[i + 1][k] - path[i][k] for k in range(3)]
                distance = math.sqrt(sum(v * v for v in delta))
                if distance < 1e-12:
                    delta, distance = [axis[k] for k in range(3)], 1.0
                path[i + 1] = [path[i][k] + rest * delta[k] / distance for k in range(3)]

            endpoint_error = math.dist(path[-1], end)
            path[-1] = list(end)
            if endpoint_error <= link_tolerance:
                break
        return max(abs(math.dist(a, b) - rest) for a, b in zip(path, path[1:]))

    project_lengths(points, sweeps=int(numerics["initial_projection_sweeps"]))
    bend_stiffness = (float(numerics["bend_stiffness_ratio"])
                      * max(abs(air_line_mass), abs(submerged_line_mass)) * gravity)

    def energy(path):
        total = 0.0
        for a, b in zip(path, path[1:]):
            z = 0.5 * (a[2] + b[2])
            total += rest * gravity * rope_load_potential(
                z, waterline, air_line_mass, submerged_line_mass, transition_half_width
            )
        for i in range(1, segments):
            curvature = [path[i - 1][k] - 2 * path[i][k] + path[i + 1][k] for k in range(3)]
            total += 0.5 * bend_stiffness * sum(v * v for v in curvature)
        return total

    step = max(rest * float(numerics["path_initial_step_fraction"]),
               float(numerics["path_minimum_step_m"]))
    current_energy = energy(points)
    for _ in range(int(numerics["path_gradient_iterations"])):
        gradient = [[0.0, 0.0, 0.0] for _ in points]
        for i, (a, b) in enumerate(zip(points, points[1:])):
            z = 0.5 * (a[2] + b[2])
            force = 0.5 * rest * gravity * rope_load_at(
                z, waterline, air_line_mass, submerged_line_mass, transition_half_width
            )
            gradient[i][2] += force
            gradient[i + 1][2] += force
        for i in range(1, segments):
            curvature = [points[i - 1][k] - 2 * points[i][k] + points[i + 1][k] for k in range(3)]
            for k in range(3):
                gradient[i - 1][k] += bend_stiffness * curvature[k]
                gradient[i][k] -= 2 * bend_stiffness * curvature[k]
                gradient[i + 1][k] += bend_stiffness * curvature[k]
        trial = [p.copy() for p in points]
        for i in range(1, segments):
            trial[i] = [trial[i][k] - step * gradient[i][k] for k in range(3)]
        project_lengths(trial, sweeps=int(numerics["trial_projection_sweeps"]))
        trial_energy = energy(trial)
        if trial_energy < current_energy - 1e-12:
            points, current_energy = trial, trial_energy
            step = min(step * float(numerics["path_step_growth"]),
                       rest * float(numerics["path_maximum_step_fraction"]))
        else:
            step *= 0.5
            if step < rest * float(numerics["path_minimum_step_fraction"]):
                break
    residual = project_lengths(points, sweeps=int(numerics["final_projection_sweeps"]))
    for extra_sweeps in numerics["projection_retry_sweeps"]:
        if residual <= link_tolerance:
            break
        residual = project_lengths(points, sweeps=extra_sweeps)
    if residual > link_tolerance:
        raise RuntimeError(
            f"Truss equilibrium failed link-length convergence: max error {residual:.6g} m; "
            f"allowed {link_tolerance:.6g} m, chord {chord:.6g} m, "
            f"material length {material_length:.6g} m, segments {segments}"
        )
    return points


def lower_sling_profiles(sling_lines, positions, waterline, air_line_mass,
                         submerged_line_mass, gravity, truss_area, time_s=None,
                         numerics=None):
    """Solve each connected lower-sling line using its own node IDs and mesh density."""
    fields, wet_lengths, records = {}, [], []
    truss_diameter = math.sqrt(4.0 * truss_area / math.pi) if truss_area > 0 else 0.0
    numerics = {**DEFAULT_NUMERICS, **(numerics or {})}
    contact_tolerance = max(float(numerics["floating_contact_tolerance_m"]), truss_diameter)
    floating_submergence = clamp(air_line_mass / (air_line_mass - submerged_line_mass), 0.0, 1.0)
    for line in sling_lines:
        line_id = str(line["id"])
        node_roles = [positions["roles"][node_id] for node_id in line["nodes"]]
        lower_index = node_roles.index("lower_joint")
        beam_index = next(index for index, node_id in enumerate(line["nodes"])
                          if node_id in positions["beam_node_ids"] and index != lower_index)
        start = positions["xyz"][line["nodes"][lower_index]]
        end = positions["xyz"][line["nodes"][beam_index]]
        rest_length = float(line["rest_length_m"])
        segments = int(line.get("subdivisions", 1))
        if segments < 1:
            raise ValueError(f"line {line_id} subdivisions must be at least 1")
        try:
            points = equilibrate_truss_path(
                start, end, rest_length, waterline, air_line_mass,
                submerged_line_mass, gravity, truss_diameter / 2.0, segments, numerics
            )
        except (RuntimeError, ValueError) as error:
            time_context = f" at t={time_s:.3f} s" if time_s is not None else ""
            raise RuntimeError(f"Lower sling line {line_id}{time_context}: {error}") from error
        wet = 0.0
        for a, b in zip(points, points[1:]):
            length = math.dist(a, b)
            if abs(a[2] - waterline) <= contact_tolerance and abs(b[2] - waterline) <= contact_tolerance:
                fraction = floating_submergence
            else:
                fraction = submerged_fraction(a[2], b[2], waterline)
            wet += length * fraction
        wet_lengths.append(wet)
        actual_length = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
        fields[f"lower_sling_{line_id}_profile_length_error_m"] = actual_length - rest_length
        fields[f"lower_sling_{line_id}_max_link_length_error_m"] = max(
            abs(math.dist(a, b) - rest_length / segments) for a, b in zip(points, points[1:])
        )
        surface_contact = any(abs(p[2] - waterline) <= contact_tolerance for p in points[1:-1])
        fields[f"lower_sling_{line_id}_surface_contact"] = surface_contact
        records.append({"id": line_id, "rest_length_m": rest_length,
                        "chord_length_m": math.dist(start, end),
                        "path_length_m": actual_length, "surface_contact": surface_contact,
                        "wet_length_m": wet})
        for j, point in enumerate(points[1:-1], start=1):
            for axis, coordinate in zip(("x", "y", "z"), point):
                fields[f"lower_sling_{line_id}_profile_{j}_{axis}_m"] = coordinate
    return fields, wet_lengths, records


def derive_mesh(cfg):
    """Resolve semantic model roles from the editable node/line mesh."""
    nodes = {int(node["id"]): node for node in cfg.get("nodes", [])}
    lines = cfg.get("lines", [])
    if not nodes or not lines:
        raise ValueError("Model must define numbered nodes and connected lines")
    xyz = {node_id: list(map(float, node["position_m"])) for node_id, node in nodes.items()}
    if any(len(p) != 3 for p in xyz.values()):
        raise ValueError("Every node.position_m must contain three coordinates")
    line_ids = set()
    for line in lines:
        line_id = int(line["id"])
        if line_id in line_ids:
            raise ValueError(f"Duplicate line id: {line_id}")
        line_ids.add(line_id)
        line["id"] = line_id
        endpoints = list(map(int, line.get("nodes", [])))
        if len(endpoints) != 2 or any(node_id not in nodes for node_id in endpoints):
            raise ValueError(f"Line {line_id} must connect two existing node ids")
        line["nodes"] = endpoints
        component_id = line.get("component_id")
        component = cfg.get("components", {}).get(component_id)
        if component is None:
            raise ValueError(f"Line {line_id} references missing component {component_id!r}")
        if line.get("element_type") != component.get("element_type"):
            raise ValueError(f"Line {line_id} element_type does not match component {component_id}")
        property_set = component.get("property_set")
        if property_set not in cfg.get("elements", {}):
            raise ValueError(f"Component {component_id} references missing elements.{property_set}")

    beam_lines, beam_node_ids, roles = beam_mesh(cfg)

    def unique_role(role):
        found = [node_id for node_id, value in roles.items() if value == role]
        if len(found) != 1:
            raise ValueError(f"Expected exactly one node with role '{role}', found {len(found)}")
        return found[0]

    if len(beam_node_ids) < 2:
        raise ValueError("Define at least two nodes connected by structural beam elements")
    main_lines = [line for line in lines if line.get("role") == "main_rope"]
    upper_lines = [line for line in lines if line.get("role") == "upper_sling"]
    lower_lines = [line for line in lines if line.get("role") == "lower_sling"]
    buoy_lines = [line for line in lines if line.get("role") == "buoy_body"]
    if not main_lines or not lower_lines or not beam_lines or len(buoy_lines) != 1:
        raise ValueError("Mesh needs main_rope and lower_sling lines, structural beam elements, and one buoy_body line")
    for line in lower_lines:
        line_node_ids = [int(node_id) for node_id in line["nodes"]]
        if sum(roles[node_id] == "lower_joint" for node_id in line_node_ids) != 1 or sum(
                node_id in beam_node_ids for node_id in line_node_ids) != 1:
            raise ValueError(f"Lower sling line {line['id']} must connect lower_joint to a structural beam node")
        initial_chord = math.dist(xyz[line["nodes"][0]], xyz[line["nodes"][1]])
        line["rest_length_m"] = float(line.get("rest_length_m", initial_chord))
        if line["rest_length_m"] <= 0:
            raise ValueError(f"Line {line['id']} rest_length_m must be positive")
        initial_tolerance = max(1e-8, initial_chord * 1e-10)
        if initial_chord > line["rest_length_m"] + initial_tolerance:
            raise ValueError(
                f"Lower sling line {line['id']} is infeasible in the initial mesh: "
                f"endpoint chord {initial_chord:.6g} m exceeds rest_length_m "
                f"{line['rest_length_m']:.6g} m. Increase rest_length_m or omit it "
                "to use the initial node spacing as the unstretched length."
            )
    lower_anchor_ids = sorted({
        node_id
        for line in lower_lines
        for node_id in line["nodes"]
        if node_id in beam_node_ids
    })
    if not lower_anchor_ids:
        raise ValueError("Lower sling lines must attach to at least one structural beam node")
    for role in ("upper_joint", "lower_joint", "roller", "winch", "buoy_top"):
        unique_role(role)
    for line in main_lines:
        line["rest_length_m"] = float(line.get("rest_length_m",
                                                math.dist(xyz[line["nodes"][0]], xyz[line["nodes"][1]])))
    for line in upper_lines:
        line["rest_length_m"] = float(line.get("rest_length_m",
                                                math.dist(xyz[line["nodes"][0]], xyz[line["nodes"][1]])))
    buoy_line = buoy_lines[0]
    buoy_line_roles = [roles[node_id] for node_id in buoy_line["nodes"]]
    if set(buoy_line_roles) != {"lower_joint", "buoy_top"}:
        raise ValueError("buoy_body line must connect lower_joint and buoy_top")
    buoy_height = abs(xyz[unique_role("buoy_top")][2] - xyz[unique_role("lower_joint")][2])
    return {
        "nodes": nodes, "roles": roles, "xyz": xyz, "beam_node_ids": beam_node_ids,
        "main_lines": main_lines, "upper_lines": upper_lines,
        "lower_lines": lower_lines, "beam_lines": beam_lines,
        "lower_anchor_ids": lower_anchor_ids,
        "upper_joint_id": unique_role("upper_joint"),
        "lower_joint_id": unique_role("lower_joint"),
        "roller_id": unique_role("roller"), "winch_id": unique_role("winch"),
        "buoy_top_id": unique_role("buoy_top"), "buoy_line": buoy_line,
        "buoy_height_m": buoy_height,
        "waterline_z_m": float(cfg["environment"]["waterline_z_m"]),
        "gravity_m_per_s2": float(cfg["environment"]["gravity_m_per_s2"]),
    }


def validate(cfg):
    if cfg.get("solver", {}).get("dynamic_effects_enabled", False):
        raise NotImplementedError(
            "Dynamic analysis is not implemented. Set solver.dynamic_effects_enabled to false."
        )
    model = derive_mesh(cfg)
    for element in ("truss", "beam", "buoy"):
        if element not in cfg.get("elements", {}):
            raise ValueError(f"Missing element properties: elements.{element}")
    for role, expected in (("truss", "truss"), ("beam", "beam"), ("buoy", "beam")):
        if cfg["elements"][role].get("element_type") != expected:
            raise ValueError(f"elements.{role}.element_type must be '{expected}'")
    if float(cfg["elements"]["truss"]["cross_section_area_m2"]) <= 0:
        raise ValueError("elements.truss.cross_section_area_m2 must be positive")
    if float(cfg["solver"]["time_step_s"]) <= 0:
        raise ValueError("solver.time_step_s must be positive")
    if float(cfg["solver"]["duration_s"]) < 0:
        raise ValueError("solver.duration_s must be non-negative")
    return model


def calculate_row(t, cfg, model=None):
    model = model or derive_mesh(cfg)
    winch, elements = cfg["winch"], cfg["elements"]
    beam, truss, buoy = elements["beam"], elements["truss"], elements["buoy"]
    displacement, signed_winch_speed = prescribed_motion(
        winch, t, cfg["solver"]["duration_s"]
    )
    positions = {"roles": model["roles"], "beam_node_ids": set(model["beam_node_ids"]),
                 "xyz": {i: p.copy() for i, p in model["xyz"].items()}}
    for node_id in model["beam_node_ids"]:
        positions["xyz"][node_id][2] += displacement
    upper_id, lower_id = model["upper_joint_id"], model["lower_joint_id"]
    positions["xyz"][upper_id][2] += displacement
    geom_beam_z = sum(positions["xyz"][i][2] for i in model["beam_node_ids"]) / len(model["beam_node_ids"])
    geom_upper_z = positions["xyz"][upper_id][2]
    lower_initial_z = model["xyz"][lower_id][2]
    initial_sling_anchor_z = sum(
        model["xyz"][node_id][2] for node_id in model["lower_anchor_ids"]
    ) / len(model["lower_anchor_ids"])
    sling_anchor_z = sum(
        positions["xyz"][node_id][2] for node_id in model["lower_anchor_ids"]
    ) / len(model["lower_anchor_ids"])
    lower_offset = lower_initial_z - initial_sling_anchor_z
    waterline = model["waterline_z_m"]
    gravity = model["gravity_m_per_s2"]
    buoy_height = model["buoy_height_m"]
    buoy_air_mass_per_m = float(buoy["line_mass_in_air_kg_per_m"])
    buoy_underwater_signed_load = float(buoy["net_line_load_in_water_kgf_per_m"])
    float_fraction = clamp(buoy_air_mass_per_m / (buoy_air_mass_per_m - buoy_underwater_signed_load), 0.0, 1.0)
    float_bottom = waterline - buoy_height * float_fraction
    lower_contact_start = float_bottom - lower_offset
    lower_contact_end = float_bottom + lower_offset
    if sling_anchor_z < lower_contact_start:
        z_lower = sling_anchor_z + lower_offset
        lower_state = "tracking_below_surface"
    elif sling_anchor_z <= lower_contact_end:
        z_lower = float_bottom
        lower_state = ("floating_slack_upstroke" if signed_winch_speed >= 0
                       else "floating_slack_downstroke")
    else:
        z_lower = sling_anchor_z - lower_offset
        lower_state = "caught_above_surface"
    positions["xyz"][lower_id][2] = z_lower
    positions["xyz"][model["buoy_top_id"]][2] = z_lower + buoy_height

    roller = positions["xyz"][model["roller_id"]]
    winch_xyz = positions["xyz"][model["winch_id"]]
    upper = positions["xyz"][upper_id]
    main_segments = [(line, math.dist(positions["xyz"][line["nodes"][0]],
                                     positions["xyz"][line["nodes"][1]]))
                     for line in model["main_lines"]]
    main_rope_taut_length = sum(length for _, length in main_segments)
    main_rope_initial_length = sum(float(line["rest_length_m"]) for line in model["main_lines"])
    main_rope_paid_length = main_rope_initial_length - displacement
    main_rope_slack_length = max(0.0, main_rope_paid_length - main_rope_taut_length)
    main_rope_extension_error = main_rope_taut_length - main_rope_paid_length
    horizontal_length = math.dist(winch_xyz[:2], roller[:2])
    main_vertical_length = math.dist(roller, upper)
    main_water = sum(length * submerged_fraction(
        positions["xyz"][line["nodes"][0]][2], positions["xyz"][line["nodes"][1]][2], waterline
    ) for line, length in main_segments)
    main_wet_fraction = main_water / main_rope_taut_length if main_rope_taut_length > 0 else 0.0
    main_rope_slack_water = main_rope_slack_length * main_wet_fraction

    lower_lines = model["lower_lines"]
    lower_profile_fields, lower_sling_wet_lengths, lower_records = lower_sling_profiles(
        lower_lines, positions, waterline, float(truss["line_mass_in_air_kg_per_m"]),
        float(truss["net_line_mass_in_water_kg_per_m"]), gravity,
        float(truss["cross_section_area_m2"]), t, numerical_options(cfg)
    )
    lower_rest_lengths = [record["rest_length_m"] for record in lower_records]
    lower_chords = [record["chord_length_m"] for record in lower_records]
    lower_slacks = [max(0.0, rest - chord) for rest, chord in zip(lower_rest_lengths, lower_chords)]
    lower_sling_rest_length = sum(lower_rest_lengths) / len(lower_rest_lengths)
    lower_sling_chord = sum(lower_chords) / len(lower_chords)
    lower_sling_slack = sum(lower_slacks) / len(lower_slacks)
    lower_sling_slack_total = sum(lower_slacks)
    lower_sling_water = sum(lower_sling_wet_lengths)
    lower_sling_surface_contact = any(record["surface_contact"] for record in lower_records)

    upper_lengths = [math.dist(positions["xyz"][line["nodes"][0]],
                              positions["xyz"][line["nodes"][1]]) for line in model["upper_lines"]]
    upper_water = sum(length * submerged_fraction(
        positions["xyz"][line["nodes"][0]][2], positions["xyz"][line["nodes"][1]][2], waterline
    ) for line, length in zip(model["upper_lines"], upper_lengths))
    upper_total = sum(upper_lengths)
    beam = elements["beam"]
    beam_length = 0.0
    beam_mass = 0.0
    beam_air_length = 0.0
    for line in model["beam_lines"]:
        component = cfg["components"][line["component_id"]]
        beam = cfg["elements"][component["property_set"]]
        a, b = (positions["xyz"][node_id] for node_id in line["nodes"])
        length = math.dist(a, b)
        transition = float(beam.get("waterline_transition_diameter_m", 0.0))
        air_fraction_line = segment_air_fraction(a[2], b[2], waterline, transition)
        air_mass = float(beam["effective_line_mass_in_air_kg_per_m"])
        water_mass = float(beam["effective_line_mass_in_water_kg_per_m"])
        beam_length += length
        beam_air_length += length * air_fraction_line
        beam_mass += length * (water_mass + (air_mass - water_mass) * air_fraction_line)
    air_fraction = beam_air_length / beam_length if beam_length else 0.0
    beam_line_mass = beam_mass / beam_length if beam_length else 0.0
    buoy_water_fraction = clamp((waterline - z_lower) / buoy_height, 0.0, 1.0)
    buoy_line_load = (buoy_water_fraction * buoy_underwater_signed_load
                      + (1.0 - buoy_water_fraction) * buoy_air_mass_per_m)
    buoy_load_equivalent = buoy_height * buoy_line_load
    rope_water_length = main_water + main_rope_slack_water + upper_water + lower_sling_water
    main_total = main_rope_taut_length + main_rope_slack_length
    rope_total = main_total + upper_total + sum(lower_rest_lengths)
    rope_air_length = max(0.0, rope_total - rope_water_length)
    rope_mass_equivalent = (rope_air_length * float(truss["line_mass_in_air_kg_per_m"])
                            + rope_water_length * float(truss["net_line_mass_in_water_kg_per_m"]))
    reaction = (beam_mass + buoy_load_equivalent + rope_mass_equivalent) * gravity
    lower_slings_taut = all(slack <= 1e-8 for slack in lower_slacks)
    vertical_direction_cosine = abs(lower_offset) / lower_sling_rest_length if lower_sling_rest_length else 1.0
    lower_sling_tension_quasistatic = (
        abs(buoy_load_equivalent * gravity) / (len(lower_lines) * vertical_direction_cosine)
        if lower_slings_taut and vertical_direction_cosine > 1e-12 else 0.0
    )
    return {
        "time_s": round(t, 6), "beam_z_m": geom_beam_z, "joint_z_m": geom_upper_z,
        "lower_joint_z_m": z_lower, "buoy_bottom_z_m": z_lower,
        "lower_sling_anchor_z_m": sling_anchor_z,
        "buoy_top_z_m": z_lower + buoy_height,
        "buoy_surface_float_bottom_z_m": float_bottom, "lower_sling_state": lower_state,
        "winch_speed_m_per_s": signed_winch_speed,
        "main_rope_length_m": main_rope_paid_length,
        "main_rope_taut_length_m": main_rope_taut_length,
        "main_rope_slack_length_m": main_rope_slack_length,
        "main_rope_extension_error_m": main_rope_extension_error,
        "main_rope_slack": main_rope_slack_length > 1e-9,
        "main_rope_slack_mid_z_m": (roller[2] + upper[2]) / 2.0 - min(1.0, main_rope_slack_length * 0.2),
        "main_rope_horizontal_length_m": horizontal_length,
        "main_rope_vertical_length_m": main_vertical_length,
        "lower_sling_rest_length_m": lower_sling_rest_length,
        "lower_sling_chord_length_m": lower_sling_chord,
        "lower_sling_slack_per_line_m": lower_sling_slack,
        "lower_sling_slack_total_m": lower_sling_slack_total,
        "lower_sling_surface_contact": lower_sling_surface_contact,
        "lower_sling_tension_quasistatic_n": lower_sling_tension_quasistatic,
        "main_rope_tension_quasistatic_n": max(0.0, reaction),
        **lower_profile_fields,
        "beam_air_fraction": air_fraction, "beam_line_mass_equivalent_kg_per_m": beam_line_mass,
        "beam_mass_equivalent_kg": beam_mass,
        "main_rope_air_length_m": main_rope_taut_length - main_water + main_rope_slack_length - main_rope_slack_water,
        "main_rope_water_length_m": main_water + main_rope_slack_water,
        "all_slings_air_length_m": (upper_total - upper_water) + (sum(lower_rest_lengths) - lower_sling_water),
        "all_slings_water_length_m": upper_water + lower_sling_water,
        "buoy_air_fraction": 1.0 - buoy_water_fraction,
        "buoy_net_load_equivalent_kg": buoy_load_equivalent,
        "rope_air_length_total_m": rope_air_length, "rope_water_length_total_m": rope_water_length,
        "rope_signed_line_load_equivalent_kg": rope_mass_equivalent,
        "support_reaction_z_quasistatic_n": reaction,
        "support_reaction_magnitude_quasistatic_n": abs(reaction),
        "support_reaction_horizontal_n": 0.0,
        **{f"mesh_node_{node_id}_z_m": positions["xyz"][node_id][2]
           for node_id in model["beam_node_ids"]},
    }
def _input_name(input_file):
    path = Path(input_file).resolve()
    try:
        return path.relative_to(ROOT).as_posix()
    except ValueError:
        return path.as_posix()


def run_model(cfg, input_file, output_json=None, output_csv=None, output_bin=None):
    model = validate(cfg)
    dt = float(cfg["solver"]["time_step_s"])
    duration = float(cfg["solver"]["duration_s"])
    rows = [calculate_row(i * dt, cfg, model) for i in range(int(math.floor(duration / dt)) + 1)]
    if rows[-1]["time_s"] < duration - 1e-9:
        rows.append(calculate_row(duration, cfg, model))
    result = {
        "project": cfg["project"],
        "model_status": "quasi_static_lower_sling_profiles",
        "input_file": _input_name(input_file),
        "scene": build_scene(cfg, rows),
        "metrics": metric_definitions(rows),
        "mesh": {
            "nodes": [{"id": node_id, "name": node.get("name"), "role": node.get("role"),
                       "position_m": model["xyz"][node_id]} for node_id, node in model["nodes"].items()],
            "lines": [{"id": line["id"], "name": line.get("name"), "nodes": line["nodes"],
                       "element_type": line["element_type"], "component_id": line["component_id"],
                       "role": line.get("role"), "subdivisions": line.get("subdivisions", 1)}
                      for line in cfg["lines"]],
            "components": cfg["components"]
        },
        "results_count": len(rows), "fields": list(rows[0].keys()), "results": rows
    }
    out_primary, out_csv = write_results(
        rows, result, cfg, ROOT, output_json, output_csv, output_bin,
        default_json=None,
        default_csv="results/roller_buoy_results.csv",
        default_bin="results/roller_buoy_results.bin",
    )
    print(f"Wrote {len(rows)} time points to {out_primary} and {out_csv}")
    print(f"Beam z at end: {rows[-1]['beam_z_m']:.2f} m")
    return result

