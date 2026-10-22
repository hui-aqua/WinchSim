"""Energy-minimization equilibrium for assemblies with rigid beam components.

Ropes are unilateral axial elements: they carry tension and become slack at
their material length. Beam and buoy members move as component-level rigid
bodies. The external potential integrates the configured air/water line mass.
"""
import math


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

    layout, x0, member_nodes = [], [], set()
    for component_id, ids_set in model["component_nodes"].items():
        ids = sorted(ids_set)
        # Components whose member nodes are all prescribed supports have no
        # equilibrium unknowns. Keep their input coordinates fixed instead of
        # introducing a free rigid-body motion for a grounded support beam.
        if set(ids).issubset(fixed):
            continue
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
        fixed_rotation = components[component_id].get("rotation_behavior") == "fixed"
        if not fixed_rotation:
            x0.extend(rotation.as_rotvec().tolist())
        layout.append(("body", component_id, ids, reference_center, local,
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
    chknorm_state = {"value": math.inf, "successive": 0}
    chknorm_tolerance = float(settings.get("aquasim_convergence_criteria", 1.0))
    if chknorm_tolerance <= 0.0 or not math.isfinite(chknorm_tolerance):
        raise ValueError("solver.aquasim_convergence_criteria must be positive and finite")
    acceptance = force_acceptance_n(cfg, model)

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
                        energy += length0 * w * potential
                        f = length0 * w * gravity * mass
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
                extension = max(0.0, length - rest)
                tension = ea * extension / rest
                energy += 0.5 * ea * extension * extension / rest
                force = tension * d / length
                grad[a_id] -= force
                grad[b_id] += force

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
                        jac = np.eye(3) + 0.5 * skew
                    else:
                        a, b, c = rv
                        skew = np.array([[0,-c,b],[c,0,-a],[-b,a,0]])
                        jac = np.eye(3) + (1-math.cos(angle))/angle**2*skew + (angle-math.sin(angle))/angle**3*(skew@skew)
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

    def aquasim_callback(xk):
        # L-BFGS-B evaluates the accepted iterate before calling the callback.
        # Reuse that gradient to calculate the AquaSim increment norm.
        current_x = np.asarray(xk, dtype=float)
        if last_evaluation["x"] is None or not np.array_equal(last_evaluation["x"], current_x):
            evaluate(current_x)
        current_force = last_evaluation["force"]
        if previous_iteration["x"] is not None:
            delta_force = np.abs((current_force - previous_iteration["force"]) * force_scale)
            delta_displacement = np.abs((current_x - previous_iteration["x"]) * displacement_scale)
            chknorm_state["value"] = float(np.mean(delta_force ** 1.5 * delta_displacement ** 0.5))
            if chknorm_state["value"] <= chknorm_tolerance:
                chknorm_state["successive"] += 1
            else:
                chknorm_state["successive"] = 0
            current_residual = float(np.linalg.norm(current_force * force_scale, ord=np.inf))
            if (chknorm_state["successive"] >= 2
                    and current_residual <= acceptance):
                raise StopIteration
        previous_iteration["x"] = np.array(current_x, copy=True)
        previous_iteration["force"] = np.array(current_force, copy=True)

    _, initial_force = evaluate(x0)
    previous_iteration["x"] = np.array(x0, copy=True)
    previous_iteration["force"] = np.array(initial_force, copy=True)
    result = minimize(evaluate, x0, jac=True, bounds=bounds,
                      method=str(settings.get("equilibrium_optimizer", "L-BFGS-B")),
                      callback=aquasim_callback,
                      options={"maxiter": int(settings.get("equilibrium_max_iterations", 2000)),
                               # A loose relative energy-change test can stop
                               # L-BFGS-B while the physical force residual is
                               # still above equilibrium_force_acceptance_n.
                               "ftol": float(settings.get("equilibrium_ftol", 1e-15)),
                               "gtol": float(settings.get("force_tolerance_n", 0.1)),
                               "maxls": 100, "maxcor": 30})
    # At active unilateral bounds, the normal gradient is the contact
    # reaction. It is balanced by the surface constraint and must be removed
    # from the free-force residual. L-BFGS-B's projected gradient uses this
    # same KKT convention.
    solution_x = result.x
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
    # L-BFGS-B can report success because the energy change became small even
    # while a force component remains large.  The force residual is the
    # equilibrium criterion; optimizer status alone is not sufficient.
    chknorm_converged = chknorm_state["value"] <= chknorm_tolerance
    if residual > acceptance:
        raise RuntimeError(f"Global equilibrium failed at t={time_s:.3f} s: {result.message}; "
                           f"generalized force residual {residual:.6g} N, "
                           f"AquaSim Chknorm {chknorm_state['value']:.6g} "
                           f"(criteria {chknorm_tolerance:.6g}), force acceptance "
                           f"{acceptance:.6g} N (AquaSim norm met: {chknorm_converged})")

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
    return positions, support, tensions, int(result.nit), residual, chknorm_state["value"]
