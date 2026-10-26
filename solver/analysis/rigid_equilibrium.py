"""Energy-minimization equilibrium for assemblies with rigid beam components.

Ropes are unilateral axial elements: they carry tension and become slack at
their material length. Beam and buoy members move as component-level rigid
bodies. The external potential integrates the configured air/water line mass.
"""
import math
import csv
import logging
from pathlib import Path


def force_reference_n(cfg, model):
    """Estimate a stable system-force scale from member self-weight/buoyancy."""
    gravity = float(cfg["environment"]["gravity_m_per_s2"])
    total = 0.0
    for line in model["truss_lines"] + model["beam_lines"]:
        props = cfg["elements"][cfg["components"][line["component_id"]]["property_set"]]
        if line.get("role") == "buoy_body":
            air = float(props["line_mass_in_air_kg_per_m"])
            water = float(props["net_line_load_in_water_kgf_per_m"])
        elif line["element_type"] == "beam":
            air = float(props["effective_line_mass_in_air_kg_per_m"])
            water = float(props["effective_line_mass_in_water_kg_per_m"])
        else:
            air = float(props["line_mass_in_air_kg_per_m"])
            water = float(props.get("net_line_mass_in_water_kg_per_m", air))
        length = (model["references"][line["id"]] if line["element_type"] == "truss"
                  else math.dist(model["xyz"][line["nodes"][0]], model["xyz"][line["nodes"][1]]))
        total += length * gravity * max(abs(air), abs(water))
    return max(total, 1.0)


def force_acceptance_n(cfg, model):
    settings = cfg.get("solver", {})
    absolute = float(settings.get("equilibrium_force_acceptance_n", 25.0))
    relative = float(settings.get("equilibrium_relative_force_tolerance", 0.005))
    if absolute < 0.0 or not math.isfinite(absolute):
        raise ValueError("solver.equilibrium_force_acceptance_n must be finite and non-negative")
    if relative <= 0.0 or not math.isfinite(relative):
        raise ValueError("solver.equilibrium_relative_force_tolerance must be positive and finite")
    # The system-wide relative threshold must not drown out the force scale of
    # a small, freely floating component.  The absolute value is a hard ceiling;
    # the relative value may make the target stricter for a small system.
    relative_acceptance = relative * force_reference_n(cfg, model)
    return relative_acceptance if absolute == 0.0 else min(absolute, relative_acceptance)


def _append_history(settings, row):
    target = settings.get("equilibrium_history_csv")
    if not target:
        return
    path = Path(target)
    path.parent.mkdir(parents=True, exist_ok=True)
    is_empty = not path.exists() or path.stat().st_size == 0
    fields = ("time_s", "load_factor", "load_stage", "iteration", "method",
              "potential_energy_j", "force_residual_n", "rms_update_m",
              "step_scale", "optimizer_status", "delta_norm", "norm", "chk_norm",
              "convergence_limit", "converged")
    with path.open("a", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        if is_empty:
            writer.writeheader()
        writer.writerow(row)


def solve(cfg, model, time_s, initial_positions=None):
    import numpy as np
    from scipy.optimize import minimize
    from scipy.spatial.transform import Rotation
    from hydrostatics import segment_air_fraction
    from winch_motion import prescribed_motion

    components = cfg["components"]
    gravity = float(cfg["environment"]["gravity_m_per_s2"])
    waterline = float(cfg["environment"]["waterline_z_m"])
    settings = cfg.get("solver", {})
    fixed = model["fixed_nodes"]
    start = {**model["xyz"], **model["internal_nodes"]}
    if initial_positions:
        start.update({i: list(p) for i, p in initial_positions.items() if i in start})

    # Rigid beam components that share a node are physically connected and
    # must move as one rigid assembly. Build connected groups so every mesh
    # node has one unambiguous position in the optimizer.
    component_groups = []
    for component_id, ids_set in model["component_nodes"].items():
        ids = set(ids_set)
        overlaps = [index for index, (_, group_nodes) in enumerate(component_groups)
                    if ids.intersection(group_nodes)]
        if not overlaps:
            component_groups.append(([component_id], ids))
            continue
        first = overlaps[0]
        group_components, group_nodes = component_groups[first]
        group_components.append(component_id)
        group_nodes.update(ids)
        for index in reversed(overlaps[1:]):
            other_components, other_nodes = component_groups.pop(index)
            group_components.extend(other_components)
            group_nodes.update(other_nodes)

    layout, x0, member_nodes = [], [], set()
    for component_ids, ids_set in component_groups:
        ids = sorted(ids_set)
        if set(ids).issubset(fixed):
            continue
        fixed_member_nodes = sorted(set(ids).intersection(fixed))
        if fixed_member_nodes:
            if len(fixed_member_nodes) >= 3:
                support_points = np.asarray(
                    [model["xyz"][node_id] for node_id in fixed_member_nodes], dtype=float
                )
                if np.linalg.matrix_rank(support_points - support_points.mean(axis=0)) >= 2:
                    # Three or more non-collinear fixed points prescribe all
                    # rigid-body motion. Keep this assembly at its input pose.
                    continue
            raise ValueError(
                "Rigid beam assembly has partially fixed support nodes; this analysis "
                "requires either no fixed member nodes or at least three non-collinear supports."
            )
        member_nodes.update(ids)
        reference_center = np.mean([model["xyz"][i] for i in ids], axis=0)
        local = np.array([model["xyz"][i] for i in ids]) - reference_center
        target = np.array([start[i] for i in ids])
        covariance = local.T @ (target - target.mean(axis=0))
        u, _, vt = np.linalg.svd(covariance)
        correction = np.eye(3)
        correction[2, 2] = np.linalg.det(u @ vt)
        rotation = Rotation.from_matrix(u @ correction @ vt)
        offset = len(x0)
        x0.extend(target.mean(axis=0).tolist())
        fixed_rotation = any(components[component_id].get("rotation_behavior") == "fixed"
                             for component_id in component_ids)
        if not fixed_rotation:
            x0.extend(rotation.as_rotvec().tolist())
        layout.append(("body", tuple(component_ids), ids, reference_center, local,
                       fixed_rotation, offset))
    for node_id in sorted(set(model["nodes"]) - member_nodes - fixed):
        offset = len(x0)
        x0.extend(start[node_id])
        layout.append(("point", node_id, offset))
    for node_id in sorted(model["internal_nodes"]):
        offset = len(x0)
        x0.extend(start[node_id])
        layout.append(("point", node_id, offset))
    x0 = np.asarray(x0, dtype=float)
    # Buoyant lower slings naturally settle at the waterline through the
    # air/water potential. Do not impose a one-sided geometric wall there:
    # tension can pull a cable segment above the surface.
    bounds = [(None, None)] * len(x0)

    # AquaSim expresses rotational increments as equivalent linear motion so
    # force-displacement terms are comparable across all generalized DOFs.
    force_scale = np.ones_like(x0)
    displacement_scale = np.ones_like(x0)
    for item in layout:
        if item[0] == "body" and not item[5]:
            radius = max(1.0, max(float(np.linalg.norm(q)) for q in item[4]))
            force_scale[item[6] + 3:item[6] + 6] = 1.0 / radius
            displacement_scale[item[6] + 3:item[6] + 6] = radius

    last_evaluation = {"x": None, "force": None}
    previous_iteration = {"x": None, "force": None}
    chknorm_state = {
        "value": math.inf, "norm": math.nan, "delta_norm": math.nan,
        "previous_norm": 0.0, "converged": False,
    }
    chknorm_tolerance = float(settings.get("aquasim_convergence_criteria", 1.0))
    if chknorm_tolerance <= 0.0 or not math.isfinite(chknorm_tolerance):
        raise ValueError("solver.aquasim_convergence_criteria must be positive and finite")
    convergence_criterion = str(
        settings.get("equilibrium_convergence_criterion", "force_residual")
    ).strip().lower()
    if convergence_criterion not in {"force_residual", "aquasim_chknorm"}:
        raise ValueError(
            "solver.equilibrium_convergence_criterion must be "
            "'force_residual' or 'aquasim_chknorm'"
        )
    force_tolerance = force_acceptance_n(cfg, model)

    def equilibrium_accepted(residual):
        if convergence_criterion == "aquasim_chknorm":
            return chknorm_state["value"] < chknorm_tolerance
        return math.isfinite(residual) and residual <= force_tolerance

    tangent_check_state = {"invalid": False}
    def update_aquasim_norm(current_x, current_force):
        """Record AquaSim Eq. 142 from consecutive force and displacement states."""
        if previous_iteration["x"] is None:
            delta_force = np.zeros_like(current_force)
            delta_displacement = np.zeros_like(current_x)
        else:
            delta_force = np.abs((current_force - previous_iteration["force"]) * force_scale)
            delta_displacement = np.abs(
                (current_x - previous_iteration["x"]) * displacement_scale
            )
        norm = float(np.sum(delta_force ** 1.5 * delta_displacement ** 0.5))
        delta_norm = norm - float(chknorm_state["previous_norm"])
        chk_norm = norm / max(1, len(current_x))
        chknorm_state.update(
            value=chk_norm, norm=norm, delta_norm=delta_norm,
            previous_norm=norm, converged=chk_norm < chknorm_tolerance,
        )
        previous_iteration["x"] = np.array(current_x, copy=True)
        previous_iteration["force"] = np.array(current_force, copy=True)
        return {
            "delta_norm": delta_norm, "norm": norm, "chk_norm": chk_norm,
            "convergence_limit": chknorm_tolerance,
            "converged": chknorm_state["converged"],
        }
    # Continue the external load from zero to its requested value. Starting a
    # large, slack cable assembly at full gravity can leave the tangent nearly
    # singular before the cable active set has settled. Keep the overall
    # iteration budget fixed; each load stage receives an equal share.
    load_steps = int(settings.get("equilibrium_load_steps", 10))
    if load_steps < 1:
        raise ValueError("solver.equilibrium_load_steps must be at least 1")

    displacement, _ = prescribed_motion(cfg["winch"], time_s, cfg["solver"]["duration_s"])
    upper = {i for i, role in model["roles"].items() if role == "upper_joint"}
    legs = [line for line in model["truss_lines"]
            if line.get("role") == "main_rope" and upper.intersection(line["nodes"])]
    if not legs and len([l for l in model["truss_lines"] if l.get("role") == "main_rope"]) == 1:
        legs = [l for l in model["truss_lines"] if l.get("role") == "main_rope"]
    if displacement and len(legs) != 1:
        raise ValueError("Winch payout requires one main_rope connected to upper_joint")
    moving_line = int(legs[0]["id"]) if legs else None
    gx, gw = np.polynomial.legendre.leggauss(8)
    gx, gw = (gx + 1.0) / 2.0, gw / 2.0

    def mass_potential(z, air, water, width):
        delta = air - water
        if width <= 0.0:
            mass = air if z >= waterline else water
            return mass, gravity * mass * (z - waterline)
        # Circular section immersion: integrate the actual circular-segment
        # area above the waterline instead of linearly blending mass over the
        # diameter. Its slope tends smoothly to zero at both edges.
        radius = width / 2.0
        h = z - waterline
        edge_offset = 2.0 / (3.0 * math.pi)
        if h <= -radius:
            primitive = water * h - delta * radius * edge_offset
            return water, gravity * primitive
        if h >= radius:
            primitive = (water * radius + delta * radius * (1.0 - edge_offset)
                         + air * (h - radius))
            return air, gravity * primitive
        u = h / radius
        root = math.sqrt(max(0.0, 1.0 - u * u))
        air_fraction = (math.asin(u) + math.pi / 2.0 + u * root) / math.pi
        primitive_shape = ((u * math.asin(u) + root + math.pi * u / 2.0
                            - root ** 3 / 3.0) / math.pi) - 2.0 / (3.0 * math.pi)
        mass = water + delta * air_fraction
        primitive = water * h + delta * radius * primitive_shape
        return mass, gravity * primitive

    active_load_factor = {"value": 1.0}

    def evaluate(x):
        p = {i: np.asarray(v, dtype=float) for i, v in model["xyz"].items()}
        p.update({i: np.asarray(v, dtype=float) for i, v in model["internal_nodes"].items()})
        for item in layout:
            if item[0] == "body":
                _, _, ids, center, local, fixed_rotation, offset = item
                translation = x[offset:offset + 3]
                rv = np.zeros(3) if fixed_rotation else x[offset + 3:offset + 6]
                rot = Rotation.from_rotvec(rv)
                for i, q in zip(ids, local):
                    p[i] = translation + rot.apply(q)
            else:
                _, i, offset = item
                p[i] = x[offset:offset + 3]
        grad = {i: np.zeros(3) for i in p}
        energy = 0.0

        # Integrate loads over the deformed mesh. A slack rope can take a very
        # different path from its endpoint chord; applying its full material
        # length to that chord gives the wrong submerged length and pulls the
        # buoy as if the rope were still straight.
        for line in model["truss_lines"] + model["beam_lines"]:
            props = cfg["elements"][components[line["component_id"]]["property_set"]]
            if line.get("role") == "buoy_body":
                air = float(props["line_mass_in_air_kg_per_m"])
                water = float(props["net_line_load_in_water_kgf_per_m"])
                # A vertical buoy cylinder has a sharp immersion boundary
                # along its axis. Its diameter does not smear the waterline.
                width = float(props.get("waterline_transition_diameter_m", 0.0))
            elif line["element_type"] == "beam":
                air = float(props["effective_line_mass_in_air_kg_per_m"])
                water = float(props["effective_line_mass_in_water_kg_per_m"])
                width = float(props.get("waterline_transition_diameter_m", 0.0))
            else:
                air = float(props["line_mass_in_air_kg_per_m"])
                water = float(props.get("net_line_mass_in_water_kg_per_m", air))
                default_diameter = 2.0 * math.sqrt(
                    float(props.get("cross_section_area_m2", 0.0)) / math.pi
                )
                width = float(props.get(
                    "waterline_transition_diameter_m",
                    props.get("diameter_m", default_diameter),
                ))
            if line["element_type"] == "truss":
                chain = model["chains"][line["id"]]
                total_length = model["references"][line["id"]]
                if line["id"] == moving_line:
                    total_length -= displacement
                segments = [(a_id, b_id, total_length / (len(chain) - 1))
                            for a_id, b_id in zip(chain, chain[1:])]
            else:
                a_id, b_id = line["nodes"]
                segments = [(a_id, b_id, math.dist(model["xyz"][a_id], model["xyz"][b_id]))]

            for a_id, b_id, length0 in segments:
                a, b = p[a_id], p[b_id]
                breaks = [0.0, 1.0]
                if width <= 0.0 and (a[2] - waterline) * (b[2] - waterline) < 0.0:
                    breaks.insert(1, (waterline - a[2]) / (b[2] - a[2]))
                for s0, s1 in zip(breaks, breaks[1:]):
                    for point, weight in zip(gx, gw):
                        s = s0 + (s1 - s0) * point
                        w = (s1 - s0) * weight
                        z = (1.0 - s) * a[2] + s * b[2]
                        mass, potential = mass_potential(z, air, water, width)
                        load_factor = active_load_factor["value"]
                        energy += load_factor * length0 * w * potential
                        f = load_factor * length0 * w * gravity * mass
                        grad[a_id][2] += f * (1.0 - s)
                        grad[b_id][2] += f * s

        # Exact tension-only axial strain energy; slack links have no force.
        for line in model["truss_lines"]:
            props = cfg["elements"][components[line["component_id"]]["property_set"]]
            ea = float(props["youngs_modulus_pa"]) * float(props["cross_section_area_m2"])
            total = model["references"][line["id"]]
            if line["id"] == moving_line:
                total -= displacement
            if total <= 0:
                raise ValueError(f"Winch payout makes line {line['id']} rest length non-positive")
            chain = model["chains"][line["id"]]
            rest = total / (len(chain) - 1)
            for a_id, b_id in zip(chain, chain[1:]):
                d = p[b_id] - p[a_id]
                length = float(np.linalg.norm(d))
                if length < 1e-12:
                    continue
                extension = length - rest
                if props.get("tension_only", True):
                    if extension < 0.0:
                        ratio = float(settings.get("truss_compression_regularization_ratio", 1e-3))
                        axial = ea * ratio / rest
                    else:
                        axial = ea / rest
                else:
                    axial = ea / rest
                tension = axial * extension
                energy += 0.5 * axial * extension * extension
                force = tension * d / length
                grad[a_id] -= force
                grad[b_id] += force

        # Point loads use the same potential convention as gravity: a force
        # contributes -F·x to energy and -F to its position gradient.
        for node_id, node in model["nodes"].items():
            force = node.get("point_load_n")
            if force is not None:
                applied = np.asarray(force, dtype=float)
                load_factor = active_load_factor["value"]
                energy -= load_factor * float(np.dot(applied, p[node_id]))
                grad[node_id] -= load_factor * applied

        g = np.zeros_like(x)
        for item in layout:
            if item[0] == "body":
                _, _, ids, _, local, fixed_rotation, offset = item
                rv = np.zeros(3) if fixed_rotation else x[offset + 3:offset + 6]
                rot = Rotation.from_rotvec(rv)
                g[offset:offset + 3] = sum((grad[i] for i in ids), np.zeros(3))
                if not fixed_rotation:
                    torque = sum((np.cross(rot.apply(q), grad[i]) for i, q in zip(ids, local)), np.zeros(3))
                    angle = np.linalg.norm(rv)
                    if angle < 1e-8:
                        skew = np.array([[0,-rv[2],rv[1]],[rv[2],0,-rv[0]],[-rv[1],rv[0],0]])
                        # Rotation.from_rotvec uses a spatial (left) angular
                        # perturbation: d(Rq) = (J_left dphi) x (Rq).
                        # The energy gradient is therefore J_left.T @ torque.
                        jac = np.eye(3) + 0.5 * skew
                    else:
                        a, b, c = rv
                        skew = np.array([[0,-c,b],[c,0,-a],[-b,a,0]])
                        jac = (np.eye(3) + (1-math.cos(angle))/angle**2*skew
                               + (angle-math.sin(angle))/angle**3*(skew@skew))
                    g[offset + 3:offset + 6] = jac.T @ torque
            else:
                _, i, offset = item
                g[offset:offset + 3] = grad[i]
        if not math.isfinite(float(energy)) or not np.all(np.isfinite(g)):
            raise FloatingPointError(
                f"Non-finite energy or force encountered at t={time_s:.6g} s"
            )
        last_evaluation["x"] = np.array(x, copy=True)
        last_evaluation["force"] = np.array(g, copy=True)
        return energy, g

    def tangent_stiffness_step(x, stage, iteration):
        """Take a Newton step and accept it only when equilibrium improves."""
        from scipy.sparse import coo_matrix, eye
        from scipy.sparse.linalg import spsolve

        energy, gradient = evaluate(x)
        scaled_residual = gradient * force_scale
        residual = float(np.linalg.norm(scaled_residual, ord=np.inf))
        if residual <= force_tolerance:
            norm_metrics = update_aquasim_norm(x, gradient)
            _append_history(settings, {
                "time_s": time_s, "load_factor": active_load_factor["value"],
                "load_stage": stage, "iteration": iteration, "method": "tangent_stiffness",
                "potential_energy_j": energy, "force_residual_n": residual,
                "rms_update_m": 0.0, "step_scale": 0.0, "optimizer_status": "converged",
                **norm_metrics,
            })
            if equilibrium_accepted(residual):
                return x, residual, True, "converged"

        positions = {i: np.asarray(v, dtype=float)
                     for i, v in {**model["xyz"], **model["internal_nodes"]}.items()}
        node_jacobians = {}
        for item in layout:
            if item[0] == "point":
                _, node_id, offset = item
                positions[node_id] = np.asarray(x[offset:offset + 3], dtype=float)
                node_jacobians[node_id] = (np.arange(offset, offset + 3), np.eye(3))
            else:
                _, _, node_ids, _, local, fixed_rotation, offset = item
                rv = np.zeros(3) if fixed_rotation else x[offset + 3:offset + 6]
                rotation = Rotation.from_rotvec(rv)
                angle = float(np.linalg.norm(rv))
                skew_rv = np.array([[0.0, -rv[2], rv[1]],
                                    [rv[2], 0.0, -rv[0]],
                                    [-rv[1], rv[0], 0.0]])
                if angle < 1e-8:
                    left_jacobian = np.eye(3) + 0.5 * skew_rv
                else:
                    left_jacobian = (
                        np.eye(3) + (1.0 - math.cos(angle)) / angle**2 * skew_rv
                        + (angle - math.sin(angle)) / angle**3 * (skew_rv @ skew_rv)
                    )
                for node_id, q in zip(node_ids, local):
                    arm = rotation.apply(q)
                    positions[node_id] = x[offset:offset + 3] + arm
                    indices = list(range(offset, offset + 3))
                    jacobian = np.eye(3)
                    if not fixed_rotation:
                        skew_arm = np.array([[0.0, -arm[2], arm[1]],
                                             [arm[2], 0.0, -arm[0]],
                                             [-arm[1], arm[0], 0.0]])
                        jacobian = np.column_stack((jacobian, -skew_arm @ left_jacobian))
                        indices.extend(range(offset + 3, offset + 6))
                    node_jacobians[node_id] = (np.asarray(indices), jacobian)

        rows, cols, values = [], [], []
        for line in model["truss_lines"]:
            props = cfg["elements"][components[line["component_id"]]["property_set"]]
            ea = float(props["youngs_modulus_pa"]) * float(props["cross_section_area_m2"])
            total = model["references"][line["id"]]
            if line["id"] == moving_line:
                total -= displacement
            chain = model["chains"][line["id"]]
            rest = total / (len(chain) - 1)
            for a_id, b_id in zip(chain, chain[1:]):
                d = positions[b_id] - positions[a_id]
                length = float(np.linalg.norm(d))
                if length <= 1e-12:
                    continue
                direction = d / length
                extension = length - rest
                if props.get("tension_only", True) and extension < 0.0:
                    ratio = float(settings.get("truss_compression_regularization_ratio", 1e-3))
                    axial = ea * ratio / rest
                    force = axial * extension
                else:
                    axial = ea / rest
                    force = axial * extension
                tangent = (axial * np.outer(direction, direction)
                           + max(force, 0.0) / length
                           * (np.eye(3) - np.outer(direction, direction)))
                a_map, b_map = node_jacobians.get(a_id), node_jacobians.get(b_id)
                if a_map is not None:
                    ai, aj = a_map
                    block = aj.T @ tangent @ aj
                    for i, row in enumerate(ai):
                        for j, col in enumerate(ai):
                            rows.append(int(row)); cols.append(int(col)); values.append(block[i, j])
                if b_map is not None:
                    bi, bj = b_map
                    block = bj.T @ tangent @ bj
                    for i, row in enumerate(bi):
                        for j, col in enumerate(bi):
                            rows.append(int(row)); cols.append(int(col)); values.append(block[i, j])
                if a_map is not None and b_map is not None:
                    ai, aj = a_map
                    bi, bj = b_map
                    block = -aj.T @ tangent @ bj
                    for i, row in enumerate(ai):
                        for j, col in enumerate(bi):
                            rows.append(int(row)); cols.append(int(col)); values.append(block[i, j])
                    block = -bj.T @ tangent @ aj
                    for i, row in enumerate(bi):
                        for j, col in enumerate(ai):
                            rows.append(int(row)); cols.append(int(col)); values.append(block[i, j])

        stiffness = coo_matrix((values, (rows, cols)), shape=(len(x), len(x))).tocsc()
        diagonal = np.asarray(stiffness.diagonal())
        typical = float(np.mean(np.abs(diagonal[diagonal != 0.0]))) if np.any(diagonal) else 1.0
        stiffness = stiffness + eye(len(x), format="csc") * max(typical * 1e-10, 1e-10)
        try:
            direction = np.asarray(spsolve(stiffness, -gradient), dtype=float)
        except Exception:
            direction = np.full_like(x, np.nan)
        if not np.all(np.isfinite(direction)) or float(np.dot(gradient, direction)) >= 0.0:
            direction = -gradient / max(typical, 1.0)

        max_translation = float(settings.get("tangent_max_translation_step_m", 0.05))
        max_rotation = float(settings.get("tangent_max_rotation_step_rad", 0.02))
        if max_translation <= 0.0 or max_rotation <= 0.0:
            raise ValueError("Tangent stiffness step limits must be positive")
        trust_scale = 1.0
        for item in layout:
            offset = item[-1]
            translation_step = float(np.linalg.norm(direction[offset:offset + 3]))
            if translation_step > max_translation:
                trust_scale = min(trust_scale, max_translation / translation_step)
            if item[0] == "body" and not item[5]:
                rotation_step = float(np.linalg.norm(direction[offset + 3:offset + 6]))
                if rotation_step > max_rotation:
                    trust_scale = min(trust_scale, max_rotation / rotation_step)
        direction *= trust_scale

        if (settings.get("tangent_finite_difference_check", True)
                and iteration == 1 and float(np.linalg.norm(direction)) > 0.0):
            # Directional finite difference of the full residual. This checks
            # the assembled tangent without constructing a dense FD matrix for
            # every DOF in a large cable model.
            unit_direction = direction / float(np.linalg.norm(direction))
            probe = 1e-5 * unit_direction
            _, probe_gradient = evaluate(x + probe)
            fd_tangent_action = (probe_gradient - gradient) / 1e-5
            analytical_action = np.asarray(stiffness @ unit_direction).reshape(-1)
            mismatch = float(np.linalg.norm(fd_tangent_action - analytical_action)
                             / max(np.linalg.norm(fd_tangent_action),
                                   np.linalg.norm(analytical_action), 1e-12))
            mismatch_limit = float(settings.get("tangent_finite_difference_relative_tolerance", 0.1))
            if mismatch_limit <= 0.0 or not math.isfinite(mismatch_limit):
                raise ValueError(
                    "solver.tangent_finite_difference_relative_tolerance must be positive and finite"
                )
            log_check = logging.warning if mismatch > mismatch_limit else logging.info
            log_check("Tangent directional finite-difference check at t=%.6g s: "
                      "relative mismatch %.6g", time_s, mismatch)
            evaluate(x)  # Restore the current iterate in the evaluation cache.
            if mismatch > mismatch_limit:
                tangent_check_state["invalid"] = True
                _append_history(settings, {
                    "time_s": time_s, "load_factor": active_load_factor["value"],
                    "load_stage": stage, "iteration": iteration, "method": "tangent_stiffness",
                    "potential_energy_j": energy, "force_residual_n": residual,
                    "rms_update_m": 0.0, "step_scale": 0.0,
                    "optimizer_status": "invalid_tangent_fallback",
                    "delta_norm": math.nan, "norm": math.nan,
                    "chk_norm": chknorm_state["value"],
                    "convergence_limit": chknorm_tolerance, "converged": False,
                })
                return x, residual, False, "invalid_tangent"

        # Accept a tangent step only when it lowers the scaled force residual.
        current_merit = 0.5 * float(np.dot(scaled_residual, scaled_residual))
        scale, accepted = 1.0, False
        next_gradient = None
        for _ in range(24):
            trial = x + scale * direction
            trial_energy, trial_gradient = evaluate(trial)
            trial_residual = trial_gradient * force_scale
            trial_merit = 0.5 * float(np.dot(trial_residual, trial_residual))
            if math.isfinite(trial_energy) and math.isfinite(trial_merit) and trial_merit < current_merit:
                accepted = True
                next_gradient = trial_gradient
                break
            scale *= 0.5
        if accepted:
            update = scale * direction
            x = x + update
            residual = float(np.linalg.norm(next_gradient * force_scale, ord=np.inf))
            rms_update = float(np.linalg.norm(update) / math.sqrt(max(1, len(update))))
            status = "accepted"
            norm_metrics = update_aquasim_norm(x, next_gradient)
        else:
            rms_update, scale, status = 0.0, 0.0, "line_search_failed"
            norm_metrics = {
                "delta_norm": math.nan, "norm": math.nan, "chk_norm": chknorm_state["value"],
                "convergence_limit": chknorm_tolerance, "converged": False,
            }
        _append_history(settings, {
            "time_s": time_s, "load_factor": active_load_factor["value"],
            "load_stage": stage, "iteration": iteration, "method": "tangent_stiffness",
            "potential_energy_j": energy, "force_residual_n": residual,
            "rms_update_m": rms_update, "step_scale": scale, "optimizer_status": status,
            **norm_metrics,
        })
        return x, residual, accepted, status

    max_iterations = int(settings.get("equilibrium_max_iterations", 2000))
    if max_iterations < 1:
        raise ValueError("solver.equilibrium_max_iterations must be positive")
    x = np.array(x0, copy=True)
    previous_iteration["x"] = np.array(x, copy=True)
    _, initial_force = evaluate(x)
    previous_iteration["force"] = np.array(initial_force, copy=True)
    result = None
    tangent_frame = bool(layout)
    iteration_number = 0
    for step in range(1, load_steps + 1):
        active_load_factor["value"] = step / load_steps
        chknorm_state.update(
            value=math.inf, norm=math.nan, delta_norm=math.nan,
            previous_norm=0.0, converged=False,
        )
        # Establish the previous-iteration force at this stage's load factor;
        # the load jump itself is not an iteration change.
        _, stage_force = evaluate(x)
        previous_iteration["x"] = np.array(x, copy=True)
        previous_iteration["force"] = np.array(stage_force, copy=True)
        stage_iterations = max(1, math.ceil(max_iterations / load_steps))
        tangent_converged = False
        tangent_iterations = 0
        tangent_iteration_limit = min(
            stage_iterations,
            max(1, int(settings.get("tangent_max_iterations_per_load_stage", 25))),
        )
        if tangent_frame:
            for local_iteration in range(1, tangent_iteration_limit + 1):
                iteration_number += 1
                tangent_iterations += 1
                x, tangent_residual, accepted_step, tangent_status = tangent_stiffness_step(
                    x, step, iteration_number
                )
                if tangent_status == "invalid_tangent":
                    logging.warning(
                        "Disabling tangent iterations at t=%.3f s, load stage %s/%s; "
                        "switching to energy minimization with the remaining iteration budget",
                        time_s, step, load_steps,
                    )
                    tangent_frame = False
                    break
                if equilibrium_accepted(tangent_residual):
                    tangent_converged = True
                    break
                if not accepted_step:
                    raise RuntimeError(
                        f"Tangent Newton line search could not reduce the force residual "
                        f"at t={time_s:.6g} s, load stage {step}/{load_steps}; "
                        f"residual {tangent_residual:.6g} N. The tangent or model "
                        "constraints need correction before equilibrium can be accepted."
                    )
        if not tangent_converged:
            # Retain the energy optimizer as a recovery path for mechanisms or
            # active-set changes that make the assembled tangent singular.
            remaining = max(1, stage_iterations - tangent_iterations)
            prior_x = {"value": np.array(x, copy=True)}

            def history_callback(xk):
                current = np.asarray(xk, dtype=float)
                energy, gradient = evaluate(current)
                delta = current - prior_x["value"]
                norm_metrics = update_aquasim_norm(current, gradient)
                _append_history(settings, {
                    "time_s": time_s, "load_factor": active_load_factor["value"],
                    "load_stage": step, "iteration": iteration_number + 1,
                    "method": "L-BFGS-B recovery", "potential_energy_j": energy,
                    "force_residual_n": float(np.linalg.norm(gradient * force_scale, ord=np.inf)),
                    "rms_update_m": float(np.linalg.norm(delta) / math.sqrt(max(1, len(delta)))),
                    "step_scale": 1.0, "optimizer_status": "iteration",
                    **norm_metrics,
                })
                prior_x["value"] = np.array(current, copy=True)
                current_residual = float(np.linalg.norm(gradient * force_scale, ord=np.inf))
                if equilibrium_accepted(current_residual):
                    raise StopIteration

            result = minimize(
                evaluate, x, jac=True, bounds=bounds,
                method=str(settings.get("equilibrium_optimizer", "L-BFGS-B")),
                callback=history_callback,
                options={"maxiter": remaining,
                         "ftol": float(settings.get("equilibrium_ftol", 1e-15)),
                         "gtol": float(settings.get("force_tolerance_n", 0.1)),
                         "maxls": 100, "maxcor": 30},
            )
            x = np.asarray(result.x, dtype=float)
            iteration_number += int(getattr(result, "nit", 0))
            _, recovered_gradient = evaluate(x)
            recovered_residual = float(
                np.linalg.norm(recovered_gradient * force_scale, ord=np.inf)
            )
            tangent_converged = equilibrium_accepted(recovered_residual)
        if not tangent_converged:
            final_stage_residual = float(
                np.linalg.norm(evaluate(x)[1] * force_scale, ord=np.inf)
            )
            if convergence_criterion == "aquasim_chknorm":
                raise RuntimeError(
                    f"Equilibrium step did not meet AquaSim ChkNorm acceptance at "
                    f"t={time_s:.6g} s, load stage {step}/{load_steps}: ChkNorm "
                    f"{chknorm_state['value']:.6g}, limit {chknorm_tolerance:.6g}; "
                    f"force residual diagnostic {final_stage_residual:.6g} N."
                )
            raise RuntimeError(
                f"Equilibrium step did not meet force acceptance at t={time_s:.6g} s, "
                f"load stage {step}/{load_steps}: force residual "
                f"{final_stage_residual:.6g} N, "
                f"limit {force_tolerance:.6g} N; AquaSim ChkNorm "
                f"{chknorm_state['value']:.6g} (diagnostic limit {chknorm_tolerance:.6g})."
            )
        previous_iteration["x"] = np.array(x, copy=True)
        _, force = evaluate(x)
        previous_iteration["force"] = np.array(force, copy=True)
        if tangent_converged and step == load_steps:
            result = type("TangentResult", (), {"message": "tangent stiffness converged"})()
    solution_x = x
    # At active unilateral bounds, the normal gradient is the contact
    # reaction. It is balanced by the surface constraint and must be removed
    # from the free-force residual. L-BFGS-B's projected gradient uses this
    # same KKT convention.
    final_energy, final_gradient = evaluate(solution_x)
    force_residual = final_gradient.copy()
    for index, (lower, upper_bound) in enumerate(bounds):
        coordinate = solution_x[index]
        if upper_bound is not None and coordinate >= upper_bound - 1e-7 and force_residual[index] < 0.0:
            force_residual[index] = 0.0
        if lower is not None and coordinate <= lower + 1e-7 and force_residual[index] > 0.0:
            force_residual[index] = 0.0
    for item in layout:
        if item[0] == "body" and not item[5]:
            radius = max(1.0, max(float(np.linalg.norm(q)) for q in item[4]))
            force_residual[item[6] + 3:item[6] + 6] /= radius
    residual = float(np.linalg.norm(force_residual, ord=np.inf))
    # ChkNorm remains a diagnostic, but a small per-DOF change can hide a large
    # unbalanced system. Accept equilibrium only when the projected force
    # residual also meets the configured force tolerance.
    chknorm_converged = chknorm_state["value"] < chknorm_tolerance
    if not equilibrium_accepted(residual):
        result_message = getattr(result, "message", "iteration limit reached")
        if convergence_criterion == "aquasim_chknorm":
            raise RuntimeError(
                f"Global equilibrium failed at t={time_s:.3f} s: {result_message}; "
                f"AquaSim ChkNorm {chknorm_state['value']:.6g} does not meet "
                f"criterion {chknorm_tolerance:.6g}; projected force residual "
                f"diagnostic {residual:.6g} N."
            )
        raise RuntimeError(f"Global equilibrium failed at t={time_s:.3f} s: {result_message}; "
                           f"projected force residual {residual:.6g} N exceeds "
                           f"acceptance {force_tolerance:.6g} N (AquaSim ChkNorm "
                           f"{chknorm_state['value']:.6g}, diagnostic criterion "
                           f"{chknorm_tolerance:.6g})")

    positions = {i: np.asarray(v, dtype=float) for i, v in model["xyz"].items()}
    positions.update({i: np.asarray(v, dtype=float) for i, v in model["internal_nodes"].items()})
    for item in layout:
        if item[0] == "body":
            _, _, ids, _, local, fixed_rotation, offset = item
            trans = solution_x[offset:offset + 3]
            rv = np.zeros(3) if fixed_rotation else solution_x[offset + 3:offset + 6]
            rot = Rotation.from_rotvec(rv)
            for i, q in zip(ids, local):
                positions[i] = trans + rot.apply(q)
        else:
            _, i, offset = item
            positions[i] = solution_x[offset:offset + 3]

    # The energy solve gives exactly zero axial force to a slack cable, so its
    # intermediate nodes otherwise have no length constraint and can collapse
    # into an arbitrary short path. Reconstruct slack lower-sling profiles at
    # their fixed material length under the configured air/water line loads.
    from analysis.roller_buoy import equilibrate_truss_path, numerical_options
    numerics = numerical_options(cfg)
    waterline = float(cfg["environment"]["waterline_z_m"])
    gravity = float(cfg["environment"]["gravity_m_per_s2"])
    for line in model["truss_lines"]:
        if line.get("role") != "lower_sling":
            continue
        chain = model["chains"][line["id"]]
        material_length = model["references"][line["id"]]
        chord = math.dist(positions[chain[0]], positions[chain[-1]])
        slack_tolerance = max(1e-8, material_length * 1e-10)
        if chord >= material_length - slack_tolerance:
            continue
        props = cfg["elements"][components[line["component_id"]]["property_set"]]
        area = float(props["cross_section_area_m2"])
        diameter = math.sqrt(4.0 * area / math.pi) if area > 0.0 else 0.0
        profile = equilibrate_truss_path(
            positions[chain[0]], positions[chain[-1]], material_length,
            waterline, float(props["line_mass_in_air_kg_per_m"]),
            float(props.get("net_line_mass_in_water_kg_per_m", 0.0)),
            gravity, diameter / 2.0, len(chain) - 1, numerics,
        )
        for node_id, point in zip(chain[1:-1], profile[1:-1]):
            positions[node_id] = np.asarray(point, dtype=float)

    tensions = {}
    for line in model["truss_lines"]:
        props = cfg["elements"][components[line["component_id"]]["property_set"]]
        ea = float(props["youngs_modulus_pa"]) * float(props["cross_section_area_m2"])
        total = model["references"][line["id"]] - (displacement if line["id"] == moving_line else 0.0)
        chain = model["chains"][line["id"]]
        rest = total / (len(chain) - 1)
        forces = [ea * max(0.0, math.dist(positions[a], positions[b]) - rest) / rest
                  for a, b in zip(chain, chain[1:])]
        threshold = float(settings.get("slack_tension_threshold_n", 1.0))
        tensions[line["id"]] = {"max_tension_n": max(forces, default=0.0),
                                "slack_segment_fraction": sum(f <= threshold for f in forces) / len(forces)}
    total_mass = 0.0
    for line in model["truss_lines"] + model["beam_lines"]:
        props = cfg["elements"][components[line["component_id"]]["property_set"]]
        if line.get("role") == "buoy_body":
            air = float(props["line_mass_in_air_kg_per_m"])
            water = float(props["net_line_load_in_water_kgf_per_m"])
            width = float(props.get("waterline_transition_diameter_m", 0.0))
        elif line["element_type"] == "beam":
            air = float(props["effective_line_mass_in_air_kg_per_m"])
            water = float(props["effective_line_mass_in_water_kg_per_m"])
            width = float(props.get("waterline_transition_diameter_m", 0.0))
        else:
            air = float(props["line_mass_in_air_kg_per_m"])
            water = float(props.get("net_line_mass_in_water_kg_per_m", air))
            width = float(props.get("waterline_transition_diameter_m", 0.0))
        if line["element_type"] == "truss":
            chain = model["chains"][line["id"]]
            length0 = (model["references"][line["id"]] -
                       (displacement if line["id"] == moving_line else 0.0)) / (len(chain) - 1)
            segments = [(positions[a_id], positions[b_id], length0)
                        for a_id, b_id in zip(chain, chain[1:])]
        else:
            a_id, b_id = line["nodes"]
            segments = [(positions[a_id], positions[b_id],
                         math.dist(model["xyz"][a_id], model["xyz"][b_id]))]
        for a, b, length0 in segments:
            fraction_air = segment_air_fraction(
                a[2], b[2], waterline, width,
                circular_section=line.get("role") != "buoy_body" and width > 0.0,
            )
            total_mass += length0 * (water + (air - water) * fraction_air)
    support = [0.0, 0.0, total_mass * gravity]
    # Summed support reaction follows vertical global force balance.
    return positions, support, tensions, iteration_number, residual, chknorm_state["value"]
