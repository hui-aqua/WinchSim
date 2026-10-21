"""Energy-minimization equilibrium for assemblies with rigid beam components.

Ropes are unilateral axial elements: they carry tension and become slack at
their material length. Beam and buoy members move as component-level rigid
bodies. The external potential integrates the configured air/water line mass.
"""
import math


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
    bounds = [(None, None)] * len(x0)
    surface_rope_nodes = {node_id for line in model["truss_lines"]
                          if line.get("role") == "lower_sling"
                          for node_id in model["chains"][line["id"]][1:-1]}
    for item in layout:
        if item[0] == "point" and item[1] in surface_rope_nodes:
            bounds[item[2] + 2] = (None, waterline)

    displacement, _ = prescribed_motion(cfg["winch"], time_s)
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
        bottom, top = waterline - width / 2.0, waterline + width / 2.0
        if z <= bottom:
            return water, gravity * water * (z - waterline)
        if z >= top:
            primitive = (water + air) * width / 2.0 + air * (z - top)
            return air, gravity * primitive
        q = z - bottom
        mass = water + delta * q / width
        primitive = water * (z - waterline) + delta * q * q / (2.0 * width)
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

        # Weight and buoyancy potential, integrated over each member.
        for line in model["truss_lines"] + model["beam_lines"]:
            props = cfg["elements"][components[line["component_id"]]["property_set"]]
            if line.get("role") == "buoy_body":
                air = float(props["line_mass_in_air_kg_per_m"])
                water = float(props["net_line_load_in_water_kgf_per_m"])
                width = float(props.get("waterline_transition_diameter_m", props.get("diameter_m", 0.0)))
            elif line["element_type"] == "beam":
                air = float(props["effective_line_mass_in_air_kg_per_m"])
                water = float(props["effective_line_mass_in_water_kg_per_m"])
                width = float(props.get("waterline_transition_diameter_m", 0.0))
            else:
                air = float(props["line_mass_in_air_kg_per_m"])
                water = float(props.get("net_line_mass_in_water_kg_per_m", air))
                width = float(props.get("waterline_transition_diameter_m", 0.0))
            a_id, b_id = line["nodes"]
            a, b = p[a_id], p[b_id]
            length0 = (model["references"][line["id"]] if line["element_type"] == "truss"
                       else math.dist(model["xyz"][a_id], model["xyz"][b_id]))
            for s, w in zip(gx, gw):
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
        return energy, g

    result = minimize(evaluate, x0, jac=True, bounds=bounds,
                      method=str(settings.get("equilibrium_optimizer", "L-BFGS-B")),
                      options={"maxiter": int(settings.get("equilibrium_max_iterations", 2000)),
                               "ftol": 1e-12, "gtol": float(settings.get("force_tolerance_n", 0.1)),
                               "maxls": 50, "maxcor": 30})
    # At active unilateral bounds, the normal gradient is the contact
    # reaction. It is balanced by the surface constraint and must be removed
    # from the free-force residual. L-BFGS-B's projected gradient uses this
    # same KKT convention.
    force_residual = result.jac.copy()
    for index, (lower, upper_bound) in enumerate(bounds):
        coordinate = result.x[index]
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
    acceptance = float(settings.get("equilibrium_force_acceptance_n", 10.0))
    if residual > acceptance:
        raise RuntimeError(f"Global equilibrium failed at t={time_s:.3f} s: {result.message}; "
                           f"generalized force residual {residual:.6g} N "
                           f"(acceptance {acceptance:.6g} N)")

    positions = {i: np.asarray(v, dtype=float) for i, v in model["xyz"].items()}
    positions.update({i: np.asarray(v, dtype=float) for i, v in model["internal_nodes"].items()})
    for item in layout:
        if item[0] == "body":
            _, _, ids, _, local, fixed_rotation, offset = item
            trans = result.x[offset:offset + 3]
            rv = np.zeros(3) if fixed_rotation else result.x[offset + 3:offset + 6]
            rot = Rotation.from_rotvec(rv)
            for i, q in zip(ids, local):
                positions[i] = trans + rot.apply(q)
        else:
            _, i, offset = item
            positions[i] = result.x[offset:offset + 3]
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
            width = float(props.get("waterline_transition_diameter_m", props.get("diameter_m", 0.0)))
        elif line["element_type"] == "beam":
            air = float(props["effective_line_mass_in_air_kg_per_m"])
            water = float(props["effective_line_mass_in_water_kg_per_m"])
            width = float(props.get("waterline_transition_diameter_m", 0.0))
        else:
            air = float(props["line_mass_in_air_kg_per_m"])
            water = float(props.get("net_line_mass_in_water_kg_per_m", air))
            width = float(props.get("waterline_transition_diameter_m", 0.0))
        a, b = positions[line["nodes"][0]], positions[line["nodes"][1]]
        length0 = (model["references"][line["id"]] -
                   (displacement if line["id"] == moving_line else 0.0)
                   if line["element_type"] == "truss" else
                   math.dist(model["xyz"][line["nodes"][0]], model["xyz"][line["nodes"][1]]))
        fraction_air = segment_air_fraction(a[2], b[2], waterline, width)
        total_mass += length0 * (water + (air - water) * fraction_air)
    support = [0.0, 0.0, total_mass * gravity]
    # Summed support reaction follows vertical global force balance.
    return positions, support, tensions, int(result.nit), residual
