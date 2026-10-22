"""Global quasi-static equilibrium analysis for node/line meshes.

All line reference lengths are captured from the input mesh before the first
equilibrium solve. Rigid components use unilateral energy minimization;
elastic beam components use OpenSeesPy. Each requested winch position solves
the whole assembly under gravity and hydrostatic line loads.
"""

from __future__ import annotations

import math
import logging
from pathlib import Path
from time import perf_counter

from hydrostatics import segment_air_fraction
from analysis.rigid_equilibrium import force_acceptance_n
from result_io import write_results
from result_schema import build_scene, metric_definitions
from winch_motion import prescribed_motion

ROOT = Path(__file__).resolve().parents[2]


class MissingSolverDependencyError(RuntimeError):
    """Raised when the interpreter lacks the required global-solver backend."""


def _point_distance(a, b):
    return math.sqrt(sum((float(a[k]) - float(b[k])) ** 2 for k in range(3)))


def _xyz_distance(positions, a, b):
    return _point_distance(positions[int(a)], positions[int(b)])


def _is_rigid_model(cfg, model):
    fixed_nodes = model["fixed_nodes"]
    for component_id in model["component_nodes"]:
        component_lines = [line for line in model["beam_lines"]
                           if line["component_id"] == component_id]
        if cfg["components"][component_id].get("structural_model", "rigid") == "rigid":
            continue
        # A roller support beam with both endpoints fixed has no structural
        # degrees of freedom. Do not force otherwise-rigid assemblies through
        # the elastic OpenSees path just because this inactive beam is marked
        # elastic to provide roller metadata/section properties.
        if (component_lines
                and all(line.get("role") == "roller_body"
                        and set(line["nodes"]).issubset(fixed_nodes)
                        for line in component_lines)):
            continue
        return False
    return True


def _has_grounded_elastic_roller(cfg, model):
    fixed_nodes = model["fixed_nodes"]
    return any(
        line.get("role") == "roller_body"
        and set(line["nodes"]).issubset(fixed_nodes)
        and cfg["components"][line["component_id"]].get("structural_model") == "elastic"
        for line in model["beam_lines"]
    )


def _beam_properties(cfg, component_id):
    component = cfg["components"][component_id]
    props = cfg["elements"][component["property_set"]]
    required = (
        "youngs_modulus_pa", "shear_modulus_pa", "cross_section_area_m2",
        "second_moment_area_y_m4", "second_moment_area_z_m4",
        "torsional_constant_m4",
    )
    missing = [key for key in required if key not in props]
    if missing:
        raise ValueError(
            f"Elastic beam component {component_id!r} is missing section properties: "
            + ", ".join(missing)
        )
    values = [float(props[key]) for key in required]
    if any(value <= 0 or not math.isfinite(value) for value in values):
        raise ValueError(f"Elastic beam component {component_id!r} properties must be positive and finite")
    return props


def prepare_model(cfg):
    """Validate mesh and freeze material lengths from the input coordinates."""
    nodes = {int(node["id"]): node for node in cfg.get("nodes", [])}
    lines = cfg.get("lines", [])
    if not nodes or not lines:
        raise ValueError("Global equilibrium needs numbered nodes and connected lines")
    xyz = {node_id: list(map(float, node["position_m"])) for node_id, node in nodes.items()}
    if any(len(point) != 3 or any(not math.isfinite(v) for v in point) for point in xyz.values()):
        raise ValueError("Every node.position_m must contain three finite coordinates")

    components = cfg.get("components", {})
    elements = cfg.get("elements", {})
    line_ids, references = set(), {}
    truss_lines, beam_lines, component_nodes = [], [], {}
    structural_beam_node_ids = set()
    roles = {node_id: str(node.get("role", "structural")) for node_id, node in nodes.items()}
    for line in lines:
        line_id = int(line["id"])
        if line_id in line_ids:
            raise ValueError(f"Duplicate line id {line_id}")
        line_ids.add(line_id)
        endpoints = list(map(int, line.get("nodes", [])))
        if len(endpoints) != 2 or any(node_id not in nodes for node_id in endpoints):
            raise ValueError(f"Line {line_id} must connect two existing node ids")
        component_id = line.get("component_id")
        component = components.get(component_id)
        if component is None:
            raise ValueError(f"Line {line_id} references unknown component {component_id!r}")
        if line.get("element_type") != component.get("element_type"):
            raise ValueError(f"Line {line_id} element_type does not match component {component_id!r}")
        property_set = component.get("property_set")
        if property_set not in elements:
            raise ValueError(f"Component {component_id!r} references missing elements.{property_set}")
        line["id"], line["nodes"] = line_id, endpoints

        if line["element_type"] == "truss":
            props = elements[property_set]
            if float(props.get("youngs_modulus_pa", 0.0)) <= 0:
                raise ValueError(f"Truss line {line_id} needs positive youngs_modulus_pa")
            if float(props.get("cross_section_area_m2", 0.0)) <= 0:
                raise ValueError(f"Truss line {line_id} needs positive cross_section_area_m2")
            if "line_mass_in_air_kg_per_m" not in props:
                raise ValueError(f"Truss component {component_id!r} needs line_mass_in_air_kg_per_m")
            input_chord = _xyz_distance(xyz, *endpoints)
            reference = float(line.get("rest_length_m", input_chord))
            if reference <= 0 or not math.isfinite(reference):
                raise ValueError(f"Truss line {line_id} rest length must be positive and finite")
            tolerance = max(1e-8, input_chord * 1e-10)
            if input_chord > reference + tolerance:
                raise ValueError(
                    f"Truss line {line_id} is infeasible in the input geometry: its endpoint "
                    f"distance ({input_chord:.6g} m) exceeds rest_length_m ({reference:.6g} m)."
                )
            references[line_id] = reference
            truss_lines.append(line)
        elif line["element_type"] == "beam":
            behavior = str(component.get("structural_model", "rigid")).lower()
            if any(line.get("role") == "roller_body" and line["component_id"] == component_id
                   for line in beam_lines):
                behavior = "elastic"
            if behavior not in ("rigid", "elastic"):
                raise ValueError(
                    f"components.{component_id}.structural_model must be 'rigid' or 'elastic'"
                )
            component["structural_model"] = behavior
            if behavior == "elastic":
                _beam_properties(cfg, component_id)
            beam_lines.append(line)
            component_nodes.setdefault(component_id, set()).update(endpoints)
            if line.get("role") != "buoy_body":
                structural_beam_node_ids.update(endpoints)
        else:
            raise ValueError(f"Unsupported element_type {line['element_type']!r} on line {line_id}")

    for node_id, role in roles.items():
        if role == "upper_joint" and sum(node_id in line["nodes"] for line in lines
                                         if line.get("role") == "main_rope") == 0:
            raise ValueError("upper_joint must connect to a main_rope line")
    if not truss_lines:
        raise ValueError("Global equilibrium needs at least one truss line")

    seen_component_nodes = {}
    for component_id, member_nodes in component_nodes.items():
        for node_id in member_nodes:
            previous = seen_component_nodes.get(node_id)
            if previous is not None and previous != component_id:
                raise ValueError(
                    f"Beam node {node_id} belongs to both {previous!r} and {component_id!r}; "
                    "shared beam-component nodes are not supported by this analysis."
                )
            seen_component_nodes[node_id] = component_id

    solver_cfg = cfg.get("solver", {})
    default_segments = int(solver_cfg.get("default_truss_subdivisions", 8))
    if default_segments < 1:
        raise ValueError("solver.default_truss_subdivisions must be at least 1")
    internal_nodes, chains = {}, {}
    next_node_id = max(nodes) + 1
    for line in truss_lines:
        segments = int(line.get("subdivisions", default_segments))
        if segments < 1:
            raise ValueError(f"Truss line {line['id']} subdivisions must be at least 1")
        line["subdivisions"] = segments
        a_id, b_id = line["nodes"]
        a, b = xyz[a_id], xyz[b_id]
        chain = [a_id]
        for j in range(1, segments):
            node_id = next_node_id
            next_node_id += 1
            u = j / segments
            internal_nodes[node_id] = [a[k] * (1.0 - u) + b[k] * u for k in range(3)]
            chain.append(node_id)
        chain.append(b_id)
        chains[line["id"]] = chain

    roller_body_lines = [line for line in beam_lines if line.get("role") == "roller_body"]
    fixed_roles = {"roller", "winch", "fixed", "fixed_position", "anchor"}
    fixed_nodes = {node_id for node_id, role in roles.items() if role in fixed_roles}
    for line in roller_body_lines:
        ends = list(map(int, line["nodes"]))
        heads = [node_id for node_id in ends if roles.get(node_id) == "roller"]
        if len(heads) != 1:
            raise ValueError(f"roller_body beam {line['id']} must have exactly one endpoint with role 'roller'")
        head = heads[0]
        attached = [member for member in truss_lines if head in member["nodes"]]
        if len(attached) != 2:
            raise ValueError(f"roller_body beam {line['id']} head node {head} must connect to exactly two truss elements")
        beam_component = str(line["component_id"])
        beam_component_name = str(components[beam_component].get("name", beam_component))
        if any(str(components[str(member["component_id"])].get("name", member["component_id"]))
               != beam_component_name for member in attached):
            raise ValueError(
                f"Both truss elements at roller head {head} must have the same component name "
                f"as roller_body beam {line['id']} ({beam_component_name!r})"
            )
        roller_component = components[beam_component]
        allowed = set(map(str, roller_component.get("allowed_rope_components", [])))
        if not allowed:
            raise ValueError(
                f"Roller beam component {beam_component!r} must define allowed_rope_components"
            )
        known_rope_components = {str(member["component_id"]) for member in truss_lines}
        unknown = allowed - known_rope_components
        if unknown:
            raise ValueError(
                f"Roller beam component {beam_component!r} allows unknown truss components: "
                + ", ".join(sorted(unknown))
            )
        # Build truss connectivity by component. A pass-through rope is a
        # connected path that enters and leaves the roller head.
        for rope_component in allowed:
            members = [member for member in truss_lines
                       if str(member["component_id"]) == rope_component]
            degree = sum(head in member["nodes"] for member in members)
            if degree < 2:
                raise ValueError(
                    f"Allowed rope component {rope_component!r} must have at least two truss legs "
                    f"at roller head {head}"
                )
    for node_id, role in roles.items():
        if role == "roller" and not any(node_id in line["nodes"] for line in roller_body_lines):
            raise ValueError(f"roller node {node_id} must be an endpoint of a roller_body beam")
    for role in ("roller", "winch"):
        found = [node_id for node_id, value in roles.items() if value == role]
    return {
        "nodes": nodes, "xyz": xyz, "roles": roles, "truss_lines": truss_lines,
        "beam_lines": beam_lines, "component_nodes": component_nodes,
        "references": references, "chains": chains, "internal_nodes": internal_nodes,
        "beam_node_ids": sorted(structural_beam_node_ids),
        "fixed_nodes": fixed_nodes, "default_segments": default_segments,
    }


def _local_axes(a, b):
    dx = [b[k] - a[k] for k in range(3)]
    length = math.sqrt(sum(value * value for value in dx))
    if length <= 1e-10:
        raise ValueError("Beam element has zero length")
    x_axis = [value / length for value in dx]
    reference = [0.0, 0.0, 1.0]
    if abs(sum(x_axis[k] * reference[k] for k in range(3))) > 0.95:
        reference = [0.0, 1.0, 0.0]
    projection = sum(reference[k] * x_axis[k] for k in range(3))
    z_axis = [reference[k] - projection * x_axis[k] for k in range(3)]
    norm = math.sqrt(sum(value * value for value in z_axis))
    z_axis = [value / norm for value in z_axis]
    y_axis = [z_axis[1] * x_axis[2] - z_axis[2] * x_axis[1],
              z_axis[2] * x_axis[0] - z_axis[0] * x_axis[2],
              z_axis[0] * x_axis[1] - z_axis[1] * x_axis[0]]
    return x_axis, y_axis, z_axis


def _effective_line_load(props, line, endpoint_positions, waterline, gravity, buoy=False):
    a_id, b_id = line["nodes"]
    a, b = endpoint_positions[a_id], endpoint_positions[b_id]
    length = _point_distance(a, b)
    if buoy:
        air_mass = float(props["line_mass_in_air_kg_per_m"])
        water_mass = float(props["net_line_load_in_water_kgf_per_m"])
        # This component's axis is vertical; the cylinder diameter does not
        # smear its immersion boundary along the axis.
        transition = float(props.get("waterline_transition_diameter_m", 0.0))
    else:
        air_mass = float(props["effective_line_mass_in_air_kg_per_m"])
        water_mass = float(props["effective_line_mass_in_water_kg_per_m"])
        transition = float(props.get("waterline_transition_diameter_m", 0.0))
    air_fraction = segment_air_fraction(
        a[2], b[2], waterline, transition, circular_section=not buoy and transition > 0.0
    )
    line_mass = water_mass + (air_mass - water_mass) * air_fraction
    return length, line_mass * gravity, air_fraction


def _solver_once(cfg, model, time_s, load_positions):
    try:
        import openseespy.opensees as ops
    except ImportError as error:
        raise MissingSolverDependencyError(
            "Global equilibrium requires OpenSeesPy. Install solver dependencies with "
            "'.venv\\Scripts\\python.exe -m pip install -r requirements-solver.txt'."
        ) from error

    ops.wipe()
    ops.model("basic", "-ndm", 3, "-ndf", 6)
    input_xyz = model["xyz"]
    node_xyz = {node_id: list(load_positions.get(node_id, xyz))
                for node_id, xyz in input_xyz.items()}
    node_xyz.update({node_id: list(load_positions.get(node_id, xyz))
                     for node_id, xyz in model["internal_nodes"].items()})
    # Re-seed cable subdivisions on the current endpoint chord. Carrying old
    # internal coordinates after a payout or buoy release can fold the chain
    # and create artificial segment extensions before equilibrium starts.
    for line in model["truss_lines"]:
        chain = model["chains"][line["id"]]
        a, b = node_xyz[chain[0]], node_xyz[chain[-1]]
        for index, node_id in enumerate(chain[1:-1], start=1):
            fraction = index / (len(chain) - 1)
            node_xyz[node_id] = [a[k] * (1.0 - fraction) + b[k] * fraction for k in range(3)]
    for node_id, xyz in node_xyz.items():
        ops.node(int(node_id), *xyz)

    max_tag = max(node_xyz)
    master_tags, master_info = {}, {}
    for offset, (component_id, member_nodes) in enumerate(model["component_nodes"].items(), start=1):
        component = cfg["components"][component_id]
        if component["structural_model"] != "rigid":
            continue
        master = max_tag + offset
        center = [sum(node_xyz[node_id][k] for node_id in member_nodes) / len(member_nodes)
                  for k in range(3)]
        ops.node(master, *center)
        master_tags[component_id] = master
        master_info[component_id] = (center, sorted(member_nodes))
        for node_id in sorted(member_nodes):
            ops.rigidLink("beam", master, node_id)

    # Beam elements and rigid components use their component property sets.
    beam_element_tags, next_element = {}, 1
    transf_tag = 1
    for line in model["beam_lines"]:
        component_id = line["component_id"]
        component = cfg["components"][component_id]
        if component["structural_model"] == "rigid" and line.get("role") != "roller_body":
            continue
        props = _beam_properties(cfg, component_id)
        a_id, b_id = line["nodes"]
        x_axis, y_axis, z_axis = _local_axes(node_xyz[a_id], node_xyz[b_id])
        vecxz = z_axis
        ops.geomTransf("Linear", transf_tag, *vecxz)
        ops.element(
            "elasticBeamColumn", next_element, a_id, b_id,
            float(props["cross_section_area_m2"]), float(props["youngs_modulus_pa"]),
            float(props["shear_modulus_pa"]), float(props["torsional_constant_m4"]),
            float(props["second_moment_area_y_m4"]), float(props["second_moment_area_z_m4"]),
            transf_tag,
        )
        beam_element_tags[line["id"]] = next_element
        next_element += 1
        transf_tag += 1

    # Each rope keeps the material length captured from the original input
    # chord, while OpenSees starts the static solve from the supplied geometry.
    truss_element_tags, next_material_tag = {}, 1
    next_element = 1 + len(beam_element_tags)
    segment_load_lines = []
    waterline = float(cfg["environment"]["waterline_z_m"])
    gravity = float(cfg["environment"]["gravity_m_per_s2"])
    displacement, _ = prescribed_motion(cfg["winch"], time_s, cfg["solver"]["duration_s"])
    upper_joint_ids = {node_id for node_id, role in model["roles"].items() if role == "upper_joint"}
    main_lines = [line for line in model["truss_lines"] if line.get("role") == "main_rope"]
    lift_legs = [line for line in main_lines if upper_joint_ids.intersection(line["nodes"])]
    if not lift_legs and len(main_lines) == 1:
        lift_legs = main_lines
    if displacement and len(lift_legs) != 1:
        raise ValueError("Winch payout needs exactly one main_rope line connected to upper_joint")
    lift_leg_id = int(lift_legs[0]["id"]) if lift_legs else None

    for line_index, line in enumerate(model["truss_lines"]):
        component = cfg["components"][line["component_id"]]
        props = cfg["elements"][component["property_set"]]
        area = float(props["cross_section_area_m2"])
        young = float(props["youngs_modulus_pa"])
        rest_total = model["references"][line["id"]]
        if line["id"] == lift_leg_id:
            rest_total -= displacement
        if rest_total <= 0:
            raise ValueError(
                f"Winch motion makes main_rope line {line['id']} have non-positive material length"
            )
        chain = model["chains"][line["id"]]
        segments = len(chain) - 1
        rest_segment = rest_total / segments
        for segment_index, (a_id, b_id) in enumerate(zip(chain, chain[1:])):
            chord_segment = _point_distance(node_xyz[a_id], node_xyz[b_id])
            if chord_segment <= 1e-12:
                raise ValueError(f"Truss segment in line {line['id']} has zero current length")
            material_tag = next_material_tag
            compression_ratio = float(cfg.get("solver", {}).get(
                "truss_compression_regularization_ratio", 1e-6
            ))
            if not 0.0 < compression_ratio <= 1e-3:
                raise ValueError("truss_compression_regularization_ratio must be in (0, 1e-3]")
            compression_modulus = young * compression_ratio
            ops.uniaxialMaterial(
                "ElasticMultiLinear", material_tag, 0.0,
                "-strain", -1.0, 0.0, 1.0,
                "-stress", -compression_modulus, 0.0, young,
            )
            seed_strain = float(cfg.get("solver", {}).get("tangent_stabilization_strain", 1e-7))
            if not 0.0 <= seed_strain <= 1e-5:
                raise ValueError("tangent_stabilization_strain must be in [0, 1e-5]")
            initial_strain = 1.0 - rest_segment / chord_segment + seed_strain
            wrapped_tag = material_tag + 1
            ops.uniaxialMaterial("InitStrainMaterial", wrapped_tag, material_tag, initial_strain)
            truss_element_tags.setdefault(line["id"], []).append(next_element)
            ops.element("corotTruss", next_element, a_id, b_id, area, wrapped_tag)
            segment_load_lines.append((line, a_id, b_id))
            next_material_tag += 2
            next_element += 1

    # Translationally fixed anchors, free rope/joint translations, and
    # rotation releases for nodes that are not attached to beam components.
    for node_id in model["fixed_nodes"]:
        ops.fix(node_id, 1, 1, 1, 1, 1, 1)
    beam_member_nodes = set(model["beam_node_ids"])
    rigid_master_nodes = set(master_tags.values())
    buoy_component_ids = {line["component_id"] for line in model["beam_lines"]
                          if line.get("role") == "buoy_body"}
    for component_id in buoy_component_ids:
        component = cfg["components"][component_id]
        rotation_mode = component.get("rotation_behavior", "fixed")
        if rotation_mode not in ("fixed", "free"):
            raise ValueError(f"components.{component_id}.rotation_behavior must be 'fixed' or 'free'")
        if rotation_mode == "fixed" and component_id in master_tags:
            ops.fix(master_tags[component_id], 0, 0, 0, 1, 1, 1)
    for node_id in node_xyz:
        if node_id in model["fixed_nodes"] or node_id in beam_member_nodes:
            continue
        ops.fix(node_id, 0, 0, 0, 1, 1, 1)

    # Buoyancy/weight loads are integrated along each member and applied as
    # consistent uniform element loads for elastic beams and equivalent nodal
    # loads for rigid bodies.
    ops.timeSeries("Linear", 1)
    ops.pattern("Plain", 1, 1)
    for line in model["beam_lines"]:
        component = cfg["components"][line["component_id"]]
        props = cfg["elements"][component["property_set"]]
        is_buoy = line.get("role") == "buoy_body"
        length, weight_per_length, _ = _effective_line_load(
            props, line, load_positions, waterline, gravity, buoy=is_buoy
        )
        a_id, b_id = line["nodes"]
        if line["id"] in beam_element_tags:
            _, y_axis, z_axis = _local_axes(node_xyz[a_id], node_xyz[b_id])
            global_load = [0.0, 0.0, -weight_per_length]
            wy = sum(global_load[k] * y_axis[k] for k in range(3))
            wz = sum(global_load[k] * z_axis[k] for k in range(3))
            wx = sum(global_load[k] * _local_axes(node_xyz[a_id], node_xyz[b_id])[0][k]
                     for k in range(3))
            ops.eleLoad("-ele", beam_element_tags[line["id"]], "-type", "-beamUniform", wy, wz, wx)
        else:
            nodal_force = -weight_per_length * length / 2.0
            ops.load(a_id, 0.0, 0.0, nodal_force, 0.0, 0.0, 0.0)
            ops.load(b_id, 0.0, 0.0, nodal_force, 0.0, 0.0, 0.0)

    truss_props_by_line = {}
    for line, a_id, b_id in segment_load_lines:
        component = cfg["components"][line["component_id"]]
        props = cfg["elements"][component["property_set"]]
        a, b = node_xyz[a_id], node_xyz[b_id]
        length = _point_distance(a, b)
        air = float(props["line_mass_in_air_kg_per_m"])
        water = float(props.get("net_line_mass_in_water_kg_per_m", air))
        air_fraction = segment_air_fraction(a[2], b[2], waterline)
        mass_per_length = water + (air - water) * air_fraction
        nodal_force = -mass_per_length * gravity * length / 2.0
        ops.load(a_id, 0.0, 0.0, nodal_force, 0.0, 0.0, 0.0)
        ops.load(b_id, 0.0, 0.0, nodal_force, 0.0, 0.0, 0.0)
        truss_props_by_line[line["id"]] = props

    ops.constraints("Transformation")
    ops.numberer("RCM")
    ops.system("BandGeneral")
    tolerance = float(cfg.get("solver", {}).get("force_tolerance_n", 1e-4))
    max_iterations = int(cfg.get("solver", {}).get("max_newton_iterations", 100))
    substeps = int(cfg.get("solver", {}).get("analysis_substeps", 4))
    ops.test("NormUnbalance", tolerance, max_iterations, 0)
    ops.integrator("LoadControl", 1.0 / substeps)
    ops.analysis("Static")
    # Tangent stiffness changes abruptly as tension-only cable segments enter
    # or leave the taut set. Advance one load increment at a time so a failed
    # algorithm can be retried at the same load factor without accidentally
    # applying a second full set of increments.
    strategies = (
        ("Newton", ()),
        ("NewtonLineSearch", ("-type", "Bisection")),
        ("KrylovNewton", ()),
        ("ModifiedNewton", ("-initial",)),
        ("BFGS", ()),
    )
    status = 0
    failures = []
    for load_step in range(substeps):
        step_failures = []
        for algorithm, options in strategies:
            try:
                ops.algorithm(algorithm, *options)
                status = ops.analyze(1)
            except Exception as error:
                step_failures.append(f"{algorithm}: {error}")
                continue
            if status == 0:
                break
            step_failures.append(f"{algorithm}: status {status}")
        if status != 0:
            failures = step_failures
            break
    if status != 0:
        raise RuntimeError(
            f"Global static equilibrium failed at t={time_s:.3f} s (OpenSees status {status}). "
            "Tried " + "; ".join(failures) + ". Check supports, connectivity, component "
            "section properties, and cable lengths."
        )

    # OpenSees nodeCoord reports the undeformed coordinates; add the solved
    # translational displacement to obtain the actual equilibrium geometry.
    positions = {
        node_id: [float(node_xyz[node_id][axis]) + float(ops.nodeDisp(node_id)[axis])
                  for axis in range(3)]
        for node_id in node_xyz
    }
    beam_forces = {}
    for line_id, element_tag in beam_element_tags.items():
        response = ops.eleResponse(element_tag, "localForce")
        if response and len(response) >= 12:
            beam_forces[line_id] = [float(value) for value in response[:12]]
    reactions = {}
    ops.reactions()
    reaction = [0.0, 0.0, 0.0]
    for node_id in model["fixed_nodes"]:
        vector = ops.nodeReaction(node_id)
        for axis in range(3):
            reaction[axis] += float(vector[axis])
    tensions = {}
    for line_id, element_tags in truss_element_tags.items():
        force_values = []
        for element_tag in element_tags:
            response = ops.eleResponse(element_tag, "axialForce")
            if response:
                force_values.append(float(response[0]))
        slack_threshold = float(cfg.get("solver", {}).get("slack_tension_threshold_n", 1.0))
        tensions[line_id] = {
            "max_tension_n": max((force for force in force_values if force > 0.0), default=0.0),
            "slack_segment_fraction": (
                sum(force <= slack_threshold for force in force_values) / len(force_values)
                if force_values else 1.0
            ),
        }
    return positions, reaction, tensions, beam_forces


def solve_equilibrium(cfg, model, time_s, initial_load_positions=None, initial_time_s=None):
    if _is_rigid_model(cfg, model):
        from analysis.rigid_equilibrium import solve as solve_rigid
        guess = {**model["xyz"], **model["internal_nodes"]}
        if initial_load_positions:
            guess.update({node_id: list(point) for node_id, point in initial_load_positions.items()
                          if node_id in guess})
        if time_s > 0.0:
            now, _ = prescribed_motion(cfg["winch"], time_s, cfg["solver"]["duration_s"])
            before, _ = prescribed_motion(
                cfg["winch"], initial_time_s or 0.0, cfg["solver"]["duration_s"]
            )
            dz = now - before
            fixed_fixed = {node_id for line in model["truss_lines"]
                           if all(endpoint in model["fixed_nodes"] for endpoint in line["nodes"])
                           for node_id in model["chains"][line["id"]][1:-1]}
            for node_id in guess:
                if node_id not in model["fixed_nodes"] and node_id not in fixed_fixed:
                    guess[node_id][2] += dz
        # OpenSees supplies a useful initial geometry, but rebuilding its full
        # nonlinear model at every output time is redundant. Later states use
        # the previous converged equilibrium as their continuation start.
        if initial_load_positions is None and not _has_grounded_elastic_roller(cfg, model):
            predictor_start = perf_counter()
            try:
                guess, _, _ = _solver_once(cfg, model, time_s, guess)
                logging.info("OpenSees initial predictor completed in %.3f s",
                             perf_counter() - predictor_start)
            except MissingSolverDependencyError:
                # Do not disguise a wrong Python environment as a failed physical
                # equilibrium. That can otherwise fall through to the optimizer
                # with the raw input geometry and produce a misleading residual.
                raise
            except RuntimeError as error:
                # The unilateral energy solver is authoritative; OpenSees only
                # provides an initial geometry when it can find one.
                import warnings
                logging.warning("OpenSees initial predictor failed after %.3f s: %s",
                                perf_counter() - predictor_start, error)
                warnings.warn(
                    f"OpenSees predictor failed at t={time_s:.3f} s; trying the rigid "
                    f"energy solver from the current geometry. Predictor error: {error}",
                    RuntimeWarning,
                    stacklevel=2,
                )
        rigid_start = perf_counter()
        solution = solve_rigid(cfg, model, time_s, guess)
        logging.info("Rigid equilibrium solve at t=%.3f s completed in %.3f s "
                     "(iterations=%s, force residual=%.6g N)",
                     time_s, perf_counter() - rigid_start, solution[3], solution[4])
        # Rigid equilibrium returns (positions, reaction, tensions,
        # iterations, force_residual, chknorm). Normalize it to the shared
        # seven-field shape used by the OpenSees path, which also includes
        # beam end forces.
        return (solution[0], solution[1], solution[2], {},
                solution[3], solution[4], solution[5])
    load_positions = {**model["xyz"], **model["internal_nodes"]}
    if initial_load_positions:
        load_positions.update({node_id: list(position) for node_id, position in initial_load_positions.items()
                               if node_id in load_positions})
    if initial_time_s is not None:
        current_shift, _ = prescribed_motion(cfg["winch"], time_s, cfg["solver"]["duration_s"])
        previous_shift, _ = prescribed_motion(
            cfg["winch"], initial_time_s, cfg["solver"]["duration_s"]
        )
        dz = current_shift - previous_shift
        fixed_nodes = model["fixed_nodes"]
        fixed_fixed_lines = {int(line["id"]) for line in model["truss_lines"]
                             if all(node_id in fixed_nodes for node_id in line["nodes"])}
        fixed_fixed_internals = {node_id for line_id in fixed_fixed_lines
                                 for node_id in model["chains"][line_id][1:-1]}
        for node_id in load_positions:
            if node_id not in fixed_nodes and node_id not in fixed_fixed_internals:
                load_positions[node_id][2] += dz
    settings = cfg.get("solver", {})
    max_iterations = int(settings.get("hydrostatic_iterations", 6))
    tolerance = float(settings.get("hydrostatic_position_tolerance_m", 0.005))
    relaxation = float(settings.get("hydrostatic_relaxation", 0.7))
    previous_solution = None
    last_residual = math.inf
    for iteration in range(1, max_iterations + 1):
        positions = None
        reaction = tensions = beam_forces = None
        # The first state starts from the supplied geometry; subsequent states
        # warm-start from the preceding equilibrium while retaining input rope lengths.
        if initial_load_positions is None and iteration == 1:
            stage_ratios = [float(settings.get("truss_compression_regularization_ratio", 1e-3))]
            stage_positions = load_positions
            for ratio in stage_ratios:
                cfg["solver"]["truss_compression_regularization_ratio"] = ratio
                positions, reaction, tensions, beam_forces = _solver_once(cfg, model, time_s, stage_positions)
                stage_positions = positions
        else:
            positions, reaction, tensions, beam_forces = _solver_once(cfg, model, time_s, load_positions)
        if max_iterations == 1:
            return positions, reaction, tensions, beam_forces, 1, 0.0, math.inf
        if previous_solution is not None:
            last_residual = max(
                (_point_distance(positions[node_id], previous_solution[node_id])
                 for node_id in positions), default=0.0
            )
            if last_residual <= tolerance:
                return positions, reaction, tensions, beam_forces, iteration, last_residual, math.inf
        previous_solution = positions
        load_positions = {
            node_id: [load_positions[node_id][k] * (1.0 - relaxation)
                      + positions[node_id][k] * relaxation for k in range(3)]
            for node_id in positions
        }
    raise RuntimeError(
        f"Hydrostatic/global equilibrium did not converge at t={time_s:.3f} s after "
        f"{max_iterations} load updates; position residual {last_residual:.6g} m "
        f"(tolerance {tolerance:.6g} m)."
    )


def _solve_with_continuation_substeps(cfg, model, target_time_s, previous_positions,
                                      previous_time_s, minimum_step_s):
    """Retry a failed output step through smaller, unreported continuation steps."""
    import logging

    try:
        return solve_equilibrium(cfg, model, target_time_s, previous_positions, previous_time_s)
    except (RuntimeError, ArithmeticError) as direct_error:
        interval = target_time_s - previous_time_s if previous_time_s is not None else 0.0
        # A bisection below 2*minimum_step would create child steps smaller
        # than the configured floor. Stop here and report the true limiting
        # interval instead of silently attempting sub-minimum solves.
        if previous_positions is None or interval <= 2.0 * minimum_step_s * (1.0 + 1e-9):
            raise RuntimeError(
                f"Equilibrium failed at t={target_time_s:.3f} s with continuation interval "
                f"{interval:.6g} s (minimum {minimum_step_s:.6g} s): {direct_error}"
            ) from direct_error

        midpoint = (previous_time_s + target_time_s) / 2.0
        if midpoint <= previous_time_s or midpoint >= target_time_s:
            raise direct_error
        logging.warning(
            "Equilibrium at t=%.3f s failed; retrying continuation through t=%.6f s "
            "(interval %.6g s). Cause: %s",
            target_time_s, midpoint, interval, direct_error,
        )
        middle = _solve_with_continuation_substeps(
            cfg, model, midpoint, previous_positions, previous_time_s, minimum_step_s
        )
        rigid_model = _is_rigid_model(cfg, model)
        logging.info(
            "Continuation substep converged at t=%.6f s: iterations=%s, %s=%.6g %s, AquaSim Chknorm=%.6g",
            midpoint, middle[4], "force residual" if rigid_model else "position residual",
            middle[5], "N" if rigid_model else "m", middle[6],
        )
        return _solve_with_continuation_substeps(
            cfg, model, target_time_s, middle[0], midpoint, minimum_step_s
        )


def _row_from_state(time_s, cfg, model, positions, reaction, tensions, beam_forces,
                    hydro_iterations, hydro_residual, aquasim_chknorm):
    waterline = float(cfg["environment"]["waterline_z_m"])
    beam_zs = [positions[node_id][2] for node_id in model["beam_node_ids"]]
    upper_ids = [node_id for node_id, role in model["roles"].items() if role == "upper_joint"]
    lower_ids = [node_id for node_id, role in model["roles"].items() if role == "lower_joint"]
    buoy_top_ids = [node_id for node_id, role in model["roles"].items() if role == "buoy_top"]
    air_length = water_length = 0.0
    for line in model["truss_lines"]:
        chain = model["chains"][line["id"]]
        component = cfg["components"][line["component_id"]]
        props = cfg["elements"][component["property_set"]]
        for a_id, b_id in zip(chain, chain[1:]):
            a, b = positions[a_id], positions[b_id]
            length = _point_distance(a, b)
            air_fraction = segment_air_fraction(a[2], b[2], waterline)
            air_length += length * air_fraction
            water_length += length * (1.0 - air_fraction)
    rigid_model = _is_rigid_model(cfg, model)
    if rigid_model:
        residual_field = "equilibrium_force_residual_n"
        converged = hydro_residual <= force_acceptance_n(cfg, model)
    else:
        residual_field = "equilibrium_position_residual_m"
        converged = hydro_residual <= float(cfg.get("solver", {}).get("hydrostatic_position_tolerance_m", 0.005))
    row = {
        "time_s": round(float(time_s), 6),
        "beam_z_m": sum(beam_zs) / len(beam_zs) if beam_zs else 0.0,
        "joint_z_m": positions[upper_ids[0]][2] if upper_ids else 0.0,
        "lower_joint_z_m": positions[lower_ids[0]][2] if lower_ids else 0.0,
        "buoy_bottom_z_m": positions[lower_ids[0]][2] if lower_ids else 0.0,
        "buoy_top_z_m": positions[buoy_top_ids[0]][2] if buoy_top_ids else 0.0,
        "rope_air_length_total_m": air_length,
        "rope_water_length_total_m": water_length,
        "support_reaction_x_n": reaction[0],
        "support_reaction_y_n": reaction[1],
        "support_reaction_z_n": reaction[2],
        "support_reaction_magnitude_n": math.sqrt(sum(value * value for value in reaction)),
        "equilibrium_iterations": hydro_iterations,
        residual_field: hydro_residual,
        "global_equilibrium_converged": converged,
    }
    if rigid_model:
        row["aquasim_chknorm"] = aquasim_chknorm
    for node_id, point in positions.items():
        if node_id in model["nodes"]:
            prefix = f"mesh_node_{node_id}"
        else:
            prefix = f"mesh_internal_node_{node_id}"
        for axis, value in zip(("x", "y", "z"), point):
            row[f"{prefix}_{axis}_m"] = float(value)
    for line in model["truss_lines"]:
        chain = model["chains"][line["id"]]
        chord = _xyz_distance(positions, *line["nodes"])
        path_length = sum(_xyz_distance(positions, a_id, b_id) for a_id, b_id in zip(chain, chain[1:]))
        row[f"truss_line_{line['id']}_reference_length_m"] = model["references"][line["id"]]
        row[f"truss_line_{line['id']}_chord_length_m"] = chord
        row[f"truss_line_{line['id']}_path_length_m"] = path_length
        line_force = tensions.get(line["id"], {})
        row[f"truss_line_{line['id']}_max_tension_n"] = line_force.get("max_tension_n", 0.0)
        row[f"truss_line_{line['id']}_slack_segment_fraction"] = line_force.get("slack_segment_fraction", 0.0)
        for j, node_id in enumerate(chain[1:-1], start=1):
            for axis, value in zip(("x", "y", "z"), positions[node_id]):
                row[f"truss_line_{line['id']}_p{j}_{axis}_m"] = float(value)
    for line_id, forces in beam_forces.items():
        for end, offset in (("i", 0), ("j", 6)):
            row[f"beam_line_{line_id}_end_{end}_axial_n"] = forces[offset]
            row[f"beam_line_{line_id}_end_{end}_shear_y_n"] = forces[offset + 1]
            row[f"beam_line_{line_id}_end_{end}_shear_z_n"] = forces[offset + 2]
            row[f"beam_line_{line_id}_end_{end}_torsion_n_m"] = forces[offset + 3]
            row[f"beam_line_{line_id}_end_{end}_moment_y_n_m"] = forces[offset + 4]
            row[f"beam_line_{line_id}_end_{end}_moment_z_n_m"] = forces[offset + 5]
    return row


def run_model(cfg, input_file, output_json=None, output_csv=None, output_bin=None):
    model = prepare_model(cfg)
    dt = float(cfg["solver"]["time_step_s"])
    duration = float(cfg["solver"]["duration_s"])
    if dt <= 0 or duration < 0:
        raise ValueError("solver.time_step_s must be positive and solver.duration_s non-negative")

    times = [i * dt for i in range(int(math.floor(duration / dt)) + 1)]
    if not times or times[0] != 0.0:
        times.insert(0, 0.0)
    if times[-1] < duration - 1e-9:
        times.append(duration)
    rows, previous_positions, previous_time = [], None, None
    minimum_step = float(cfg.get("solver", {}).get("continuation_min_step_s", max(dt / 16.0, 0.01)))
    if minimum_step <= 0.0:
        raise ValueError("solver.continuation_min_step_s must be positive")
    initial_equilibrium = None
    result = None
    for time_s in times:
        logging.info("Solving equilibrium at t=%.3f s", time_s)
        positions, reaction, tensions, beam_forces, iterations, residual, aquasim_chknorm = _solve_with_continuation_substeps(
            cfg, model, time_s, previous_positions, previous_time, minimum_step
        )
        previous_positions = positions
        previous_time = time_s
        rows.append(_row_from_state(time_s, cfg, model, positions, reaction, tensions, beam_forces,
                                    iterations, residual, aquasim_chknorm))
        rigid_model = _is_rigid_model(cfg, model)
        logging.info("Equilibrium converged at t=%.3f s: iterations=%s, %s=%.6g %s, AquaSim Chknorm=%.6g",
                     time_s, iterations, "force residual" if rigid_model else "position residual",
                     residual, "N" if rigid_model else "m", aquasim_chknorm)
        if initial_equilibrium is None:
            acceptance = (force_acceptance_n(cfg, model) if rigid_model else float(
                cfg.get("solver", {}).get("hydrostatic_position_tolerance_m", 0.005)
            ))
            initial_equilibrium = {
                "converged": residual <= acceptance,
                "time_s": 0.0,
                "equilibrium_iterations": iterations,
                "force_residual_n" if rigid_model else "position_residual_m": residual,
                **({"aquasim_chknorm": aquasim_chknorm} if rigid_model else {}),
                "rope_reference_lengths_m": {
                    str(line_id): length for line_id, length in model["references"].items()
                },
            }
        if result is None:
            try:
                input_name = Path(input_file).resolve().relative_to(ROOT).as_posix()
            except ValueError:
                input_name = Path(input_file).resolve().as_posix()
            result = {
                "project": cfg["project"],
                "model_status": "global_quasi_static_equilibrium",
                "analysis": {
                    "method": "global total-potential equilibrium at each prescribed winch position",
                    "beam_component_models": {
                        component_id: component.get("structural_model", "rigid")
                        for component_id, component in cfg["components"].items()
                        if component.get("element_type") == "beam"
                    },
                    "initial_equilibrium": initial_equilibrium,
                    "rope_reference_lengths_source": "input node endpoint distances captured before equilibrium",
                },
                "input_file": input_name,
                "scene": build_scene(cfg, rows[:1]),
                "metrics": metric_definitions(rows[:1]),
                "mesh": {
                    "nodes": [{"id": node_id, "name": node.get("name"), "role": node.get("role"),
                               "position_m": model["xyz"][node_id]} for node_id, node in model["nodes"].items()],
                    "lines": [{"id": line["id"], "name": line.get("name"), "nodes": line["nodes"],
                               "element_type": line["element_type"], "component_id": line["component_id"],
                               "role": line.get("role"), "subdivisions": line.get("subdivisions", 1),
                               "reference_length_m": model["references"].get(line["id"])}
                              for line in cfg["lines"]],
                    "components": cfg["components"],
                },
                "fields": list(rows[0]),
            }
        result["results_count"] = len(rows)
        result["results_complete"] = False
        result["results"] = rows
        out_primary, out_csv = write_results(
            rows, result, cfg, ROOT, output_json, output_csv, output_bin,
            default_json=None,
            default_csv="results/global_equilibrium_results.csv",
            default_bin="results/global_equilibrium_results.bin",
            write_json=False,
        )
        logging.info("Checkpoint saved at t=%.3f s (%s output states): %s and %s",
                     time_s, len(rows), out_primary, out_csv)

    result["results_complete"] = True
    out_primary, out_csv = write_results(
        rows, result, cfg, ROOT, output_json, output_csv, output_bin,
        default_json=None,
        default_csv="results/global_equilibrium_results.csv",
        default_bin="results/global_equilibrium_results.bin",
    )
    logging.info("Wrote %s equilibrium states to %s and %s", len(rows), out_primary, out_csv)
    initial_reaction = math.sqrt(sum(rows[0][f"support_reaction_{axis}_n"] ** 2 for axis in "xyz"))
    logging.info("Initial equilibrium reaction: %.3f kN", initial_reaction / 1000)
    logging.info("Final beam mean height: %.3f m", rows[-1]["beam_z_m"])
    return result
