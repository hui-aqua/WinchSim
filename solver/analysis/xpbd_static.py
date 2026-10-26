"""XPBD pseudo-dynamic relaxation for static WinchSim positions.

This is an opt-in alternative to the existing global equilibrium solver. Ropes
are compliant tension-only distance constraints; connected beam assemblies are
kept rigid by best-fit pose projection. Gravity, buoyancy, and point
loads drive a damped pseudo-time relaxation toward a static configuration.
"""

import logging
import math
from pathlib import Path
from time import perf_counter

import numpy as np

from analysis.global_equilibrium import (
    ROOT, _row_from_state, prepare_model,
)
from result_io import write_results
from result_schema import build_scene, metric_definitions
from winch_motion import prescribed_motion


def _beam_groups(model):
    groups = []
    for component_id, members in model["component_nodes"].items():
        nodes = set(members)
        overlaps = [i for i, group in enumerate(groups) if nodes.intersection(group)]
        if not overlaps:
            groups.append(nodes)
            continue
        first = overlaps[0]
        groups[first].update(nodes)
        for i in reversed(overlaps[1:]):
            groups[first].update(groups.pop(i))
    return groups


def _equilibrium_time_points(cfg):
    dt = float(cfg["solver"]["time_step_s"])
    duration = float(cfg["solver"]["duration_s"])
    if dt <= 0.0 or duration < 0.0:
        raise ValueError("solver.time_step_s must be positive and solver.duration_s non-negative")
    times = [i * dt for i in range(int(math.floor(duration / dt)) + 1)]
    if not times or times[0] != 0.0:
        times.insert(0, 0.0)
    if times[-1] < duration - 1e-9:
        times.append(duration)
    return times


def _solve_xpbd(cfg, model, time_s, initial_positions=None):
    solver = cfg.get("solver", {})
    dt = float(solver.get("xpbd_time_step_s", 0.1))
    iterations = int(solver.get("xpbd_max_iterations", 4000))
    projection_iterations = int(solver.get("xpbd_projection_iterations", 8))
    damping = float(solver.get("xpbd_damping", 0.98))
    relaxation = float(solver.get("xpbd_projection_relaxation", 0.7))
    tolerance = float(solver.get("xpbd_position_tolerance_m", 1e-4))
    stable_iterations = int(solver.get("xpbd_stable_iterations", 20))
    if dt <= 0 or iterations < 1 or projection_iterations < 1:
        raise ValueError("XPBD time step and iteration limits must be positive")
    if (not 0.0 <= damping < 1.0 or not 0.0 < relaxation <= 1.0
            or tolerance <= 0 or stable_iterations < 1):
        raise ValueError(
            "XPBD damping must be in [0, 1), projection relaxation in (0, 1], "
            "and tolerances and stable iterations positive"
        )

    all_positions = {**model["xyz"], **model["internal_nodes"]}
    node_ids = sorted(all_positions)
    node_index = {node_id: i for i, node_id in enumerate(node_ids)}
    x = np.asarray([all_positions[node_id] for node_id in node_ids], dtype=float)
    if initial_positions:
        for node_id, position in initial_positions.items():
            if node_id in node_index:
                x[node_index[node_id]] = np.asarray(position, dtype=float)
    original = x.copy()
    velocities = np.zeros_like(x)
    fixed = set(model["fixed_nodes"])
    rigid_bodies = []
    for group in _beam_groups(model):
        ids = sorted(group)
        indices = np.asarray([node_index[node_id] for node_id in ids], dtype=np.int64)
        fixed_members = [node_id for node_id in ids if node_id in fixed]
        if fixed_members:
            support = np.asarray([model["xyz"][node_id] for node_id in fixed_members], dtype=float)
            if (len(fixed_members) < 3
                    or np.linalg.matrix_rank(support - support.mean(axis=0)) < 2):
                raise ValueError(
                    "XPBD rigid beam assembly has partial supports; use either an "
                    "unsupported rigid assembly or at least three non-collinear fixed nodes"
                )
            fixed.update(ids)
            continue
        local = original[indices] - original[indices].mean(axis=0)
        rigid_bodies.append((indices, local))
    inverse_mass = np.zeros(len(node_ids), dtype=float)
    nodal_mass = np.zeros(len(node_ids), dtype=float)
    segment_data = []
    gravity = float(cfg["environment"]["gravity_m_per_s2"])
    waterline = float(cfg["environment"]["waterline_z_m"])

    for line in model["truss_lines"] + model["beam_lines"]:
        component = cfg["components"][line["component_id"]]
        props = cfg["elements"][component["property_set"]]
        if line.get("role") == "buoy_body":
            air = float(props["line_mass_in_air_kg_per_m"])
            water = float(props["net_line_load_in_water_kgf_per_m"])
            transition = float(props.get("waterline_transition_diameter_m", 0.0))
        elif line["element_type"] == "beam":
            if component.get("structural_model", "rigid") == "elastic":
                raise ValueError(
                    "XPBD static currently supports rigid beam assemblies; "
                    f"component {line['component_id']!r} is elastic"
                )
            air = float(props["effective_line_mass_in_air_kg_per_m"])
            water = float(props["effective_line_mass_in_water_kg_per_m"])
            transition = float(props.get("waterline_transition_diameter_m", 0.0))
        else:
            air = float(props["line_mass_in_air_kg_per_m"])
            water = float(props.get("net_line_mass_in_water_kg_per_m", air))
            transition = float(props.get("waterline_transition_diameter_m",
                                         props.get("diameter_m", 0.0)))

        if line["element_type"] == "truss":
            chain = model["chains"][line["id"]]
            base_length = model["references"][line["id"]] / (len(chain) - 1)
            pairs = list(zip(chain, chain[1:]))
        else:
            pairs = [tuple(line["nodes"])]
            base_length = math.dist(*(model["xyz"][i] for i in line["nodes"]))
        inertial_density = max(abs(air), abs(water), 1e-8)
        for a_id, b_id in pairs:
            ia, ib = node_index[a_id], node_index[b_id]
            nodal_mass[ia] += 0.5 * inertial_density * base_length
            nodal_mass[ib] += 0.5 * inertial_density * base_length
            segment_data.append((ia, ib, base_length, air, water, transition))

    segment_a = np.asarray([item[0] for item in segment_data], dtype=np.int64)
    segment_b = np.asarray([item[1] for item in segment_data], dtype=np.int64)
    segment_lengths = np.asarray([item[2] for item in segment_data], dtype=float)
    segment_air_mass = np.asarray([item[3] for item in segment_data], dtype=float)
    segment_water_mass = np.asarray([item[4] for item in segment_data], dtype=float)
    segment_transition = np.asarray([item[5] for item in segment_data], dtype=float)

    for node_id, node in model["nodes"].items():
        point_mass = node.get("mass_kg")
        if point_mass is not None:
            nodal_mass[node_index[node_id]] += max(float(point_mass), 0.0)
    # Point loads often represent a suspended payload whose mass is not
    # otherwise present in the mesh (for example the 12.3 kN load at node 1
    # in static_simple.xml). XPBD uses mass only to define its relaxation
    # path, not the final static force balance, so add the equivalent inertial
    # mass to prevent an unrealistically large pseudo-acceleration.
    point_loads = np.zeros_like(x)
    for node_id, node in model["nodes"].items():
        force = node.get("point_load_n")
        if force is not None:
            point_loads[node_index[node_id]] += np.asarray(force, dtype=float)
    point_load_mass_factor = float(solver.get("xpbd_point_load_mass_factor", 1.0))
    if not math.isfinite(point_load_mass_factor) or point_load_mass_factor < 0.0:
        raise ValueError("XPBD point-load mass factor must be finite and non-negative")
    nodal_mass += (np.linalg.norm(point_loads, axis=1)
                   / max(gravity, 1e-12)) * point_load_mass_factor
    nodal_mass = np.maximum(nodal_mass, 1e-8)
    for node_id, i in node_index.items():
        if node_id not in fixed:
            inverse_mass[i] = 1.0 / nodal_mass[i]

    # XPBD distance constraints: C = |x_b - x_a| - rest_length.
    con_a, con_b, rest_lengths, compliances, unilateral = [], [], [], [], []
    displacement, _ = prescribed_motion(cfg["winch"], time_s, solver["duration_s"])
    upper_nodes = {i for i, role in model["roles"].items() if role == "upper_joint"}
    main_lines = [line for line in model["truss_lines"] if line.get("role") == "main_rope"]
    moving_lines = [line for line in main_lines if upper_nodes.intersection(line["nodes"])]
    if displacement and len(moving_lines) != 1:
        raise ValueError("XPBD winch payout requires one main_rope connected to upper_joint")
    moving_line_id = int(moving_lines[0]["id"]) if moving_lines else None

    for line in model["truss_lines"]:
        component = cfg["components"][line["component_id"]]
        props = cfg["elements"][component["property_set"]]
        ea = float(props["youngs_modulus_pa"]) * float(props["cross_section_area_m2"])
        total = model["references"][line["id"]]
        if line["id"] == moving_line_id:
            total -= displacement
        chain = model["chains"][line["id"]]
        rest = total / (len(chain) - 1)
        if rest <= 0:
            raise ValueError(f"Winch payout makes line {line['id']} rest length non-positive")
        for a_id, b_id in zip(chain, chain[1:]):
            con_a.append(node_index[a_id]); con_b.append(node_index[b_id])
            rest_lengths.append(rest)
            compliances.append(rest / ea)
            unilateral.append(bool(props.get("tension_only", True)))

    con_a = np.asarray(con_a, dtype=np.int64)
    con_b = np.asarray(con_b, dtype=np.int64)
    rest_lengths = np.asarray(rest_lengths, dtype=float)
    compliances = np.asarray(compliances, dtype=float)
    unilateral = np.asarray(unilateral, dtype=bool)
    lambdas = np.zeros(len(con_a), dtype=float)
    fixed_indices = np.asarray([node_index[i] for i in fixed if i in node_index], dtype=int)

    def external_forces(current):
        forces = point_loads.copy()
        z_a, z_b = current[segment_a, 2], current[segment_b, 2]
        low, high = np.minimum(z_a, z_b), np.maximum(z_a, z_b)
        fraction_air = np.empty_like(low)
        sharp = segment_transition <= 0.0
        fully_submerged = sharp & (high <= waterline)
        fully_air = sharp & (low >= waterline) & ~fully_submerged
        crossing = sharp & ~fully_submerged & ~fully_air
        fraction_air[fully_submerged] = 0.0
        fraction_air[fully_air] = 1.0
        fraction_air[crossing] = (
            (high[crossing] - waterline)
            / np.maximum(high[crossing] - low[crossing], 1e-12)
        )
        if np.any(~sharp):
            midpoint = (low[~sharp] + high[~sharp]) / 2.0
            width = segment_transition[~sharp]
            fraction_air[~sharp] = np.clip(
                (midpoint - waterline + width / 2.0) / width, 0.0, 1.0
            )
        line_mass = (fraction_air * segment_air_mass
                     + (1.0 - fraction_air) * segment_water_mass)
        weights = line_mass * gravity * segment_lengths * 0.5
        np.add.at(forces[:, 2], segment_a, -weights)
        np.add.at(forces[:, 2], segment_b, -weights)
        return forces

    max_error = math.inf
    max_speed = math.inf
    stable = 0
    started = perf_counter()
    for iteration in range(1, iterations + 1):
        old = x.copy()
        forces = external_forces(x)
        velocities += forces * inverse_mass[:, None] * dt
        velocities *= damping
        x += velocities * dt
        if not np.all(np.isfinite(x)) or not np.all(np.isfinite(velocities)):
            raise RuntimeError(
                f"XPBD became numerically unstable during force integration at "
                f"iteration {iteration}; check point loads, effective masses, and time step"
            )
        if fixed_indices.size:
            x[fixed_indices] = original[fixed_indices]
            velocities[fixed_indices] = 0.0
        lambdas.fill(0.0)
        alpha = compliances / (dt * dt)
        for _ in range(projection_iterations):
            delta = x[con_b] - x[con_a]
            lengths = np.linalg.norm(delta, axis=1)
            valid = lengths > 1e-12
            direction = np.zeros_like(delta)
            direction[valid] = delta[valid] / lengths[valid, None]
            constraint = lengths - rest_lengths
            denom = inverse_mass[con_a] + inverse_mass[con_b] + alpha
            active = valid & (denom > 0.0)
            if np.any(unilateral):
                slack = unilateral & (constraint <= 0.0) & (lambdas == 0.0)
                active &= ~slack
            dlambda = np.zeros_like(lambdas)
            dlambda[active] = (-constraint[active] - alpha[active] * lambdas[active]) / denom[active]
            dlambda *= relaxation
            if np.any(unilateral):
                candidate = lambdas + dlambda
                candidate[unilateral] = np.minimum(candidate[unilateral], 0.0)
                dlambda = candidate - lambdas
            lambdas += dlambda
            correction_a = (-inverse_mass[con_a] * dlambda)[:, None] * direction
            correction_b = (inverse_mass[con_b] * dlambda)[:, None] * direction
            np.add.at(x, con_a, correction_a)
            np.add.at(x, con_b, correction_b)
            if not np.all(np.isfinite(x)):
                raise RuntimeError(
                    f"XPBD produced non-finite node positions during constraint "
                    f"projection at iteration {iteration}; check constraint geometry"
                )
            # Project each connected beam assembly back to its best-fit rigid
            # pose. This avoids the quadratic number of pairwise constraints.
            for indices, local in rigid_bodies:
                target = x[indices]
                center = target.mean(axis=0)
                covariance = local.T @ (target - center)
                try:
                    u, _, vt = np.linalg.svd(covariance)
                except np.linalg.LinAlgError as exc:
                    raise RuntimeError(
                        f"XPBD rigid-body pose projection failed at iteration {iteration}; "
                        "node positions diverged before the pose fit"
                    ) from exc
                correction = np.eye(3)
                correction[2, 2] = np.linalg.det(u @ vt)
                rotation = u @ correction @ vt
                x[indices] = center + local @ rotation.T
            if fixed_indices.size:
                x[fixed_indices] = original[fixed_indices]

        velocities = (x - old) / dt
        velocities *= damping
        max_error = float(np.max(np.abs(x - old)))
        max_speed = float(np.max(np.linalg.norm(velocities, axis=1)))
        if max_error <= tolerance and max_speed <= tolerance / dt:
            stable += 1
        else:
            stable = 0
        if stable >= stable_iterations:
            break

    if stable < stable_iterations:
        raise RuntimeError(
            f"XPBD did not reach a static position at t={time_s:.6g} s after "
            f"{iterations} iterations: max position change {max_error:.6g} m, "
            f"max speed {max_speed:.6g} m/s. Increase xpbd_max_iterations or "
            "review supports and cable rest lengths."
        )

    # Recover XPBD constraint forces (lambda / dt^2) to report a free-node
    # force residual. This is diagnostic only; XPBD convergence is based on
    # the static-motion tolerance above.
    residual_forces = external_forces(x)
    d = x[con_b] - x[con_a]
    lengths = np.maximum(np.linalg.norm(d, axis=1), 1e-12)
    directions = d / lengths[:, None]
    constraint_force_a = (-lambdas / (dt * dt))[:, None] * directions
    constraint_force_b = -constraint_force_a
    np.add.at(residual_forces, con_a, constraint_force_a)
    np.add.at(residual_forces, con_b, constraint_force_b)
    free = inverse_mass > 0.0
    force_residual = float(np.max(np.abs(residual_forces[free]))) if np.any(free) else 0.0
    positions = {node_id: x[i].tolist() for node_id, i in node_index.items()}
    tensions = {}
    for line in model["truss_lines"]:
        props = cfg["elements"][cfg["components"][line["component_id"]]["property_set"]]
        ea = float(props["youngs_modulus_pa"]) * float(props["cross_section_area_m2"])
        chain = model["chains"][line["id"]]
        rest = (model["references"][line["id"]]
                - (displacement if line["id"] == moving_line_id else 0.0)) / (len(chain) - 1)
        element_tensions = []
        for a_id, b_id in zip(chain, chain[1:]):
            extension = np.linalg.norm(x[node_index[b_id]] - x[node_index[a_id]]) - rest
            element_tensions.append(max(0.0, ea * extension / rest))
        threshold = float(solver.get("slack_tension_threshold_n", 1.0))
        tensions[line["id"]] = {
            "max_tension_n": max(element_tensions, default=0.0),
            "slack_segment_fraction": (
                sum(force <= threshold for force in element_tensions) / len(element_tensions)
                if element_tensions else 1.0
            ),
        }

    logging.info(
        "XPBD static solve at t=%.3f s: iterations=%s, max position change=%.6g m, "
        "max speed=%.6g m/s, force residual diagnostic=%.6g N, elapsed=%.3f s",
        time_s, iteration, max_error, max_speed, force_residual, perf_counter() - started,
    )
    total_vertical_load = float(np.sum(external_forces(x)[:, 2]))
    reaction = [0.0, 0.0, -total_vertical_load]
    return positions, reaction, tensions, {}, iteration, force_residual, max_error, max_speed


def run_model(cfg, input_file, output_json=None, output_csv=None, output_bin=None):
    model = prepare_model(cfg)
    if any(component.get("structural_model") == "elastic"
           for component in cfg["components"].values()
           if component.get("element_type") == "beam"):
        raise ValueError("XPBD static currently supports rigid beam assemblies only")
    times = _equilibrium_time_points(cfg)
    rows = []
    previous_positions = None
    initial_equilibrium = None
    solver = cfg.get("solver", {})
    for time_s in times:
        logging.info("Solving XPBD static position at t=%.3f s", time_s)
        (positions, reaction, tensions, beam_forces, iterations, force_residual,
         position_change, speed) = _solve_xpbd(cfg, model, time_s, previous_positions)
        previous_positions = positions
        row = _row_from_state(
            time_s, cfg, model, positions, reaction, tensions, beam_forces,
            iterations, force_residual, 0.0,
        )
        row.pop("aquasim_chknorm", None)
        row["equilibrium_convergence_criterion"] = "xpbd_static_tolerance"
        row["xpbd_max_position_change_m"] = position_change
        row["xpbd_max_speed_m_per_s"] = speed
        row["global_equilibrium_converged"] = True
        rows.append(row)
        if initial_equilibrium is None:
            initial_equilibrium = {
                "converged": True,
                "time_s": time_s,
                "equilibrium_iterations": iterations,
                "xpbd_force_residual_diagnostic_n": force_residual,
                "xpbd_max_position_change_m": position_change,
                "xpbd_max_speed_m_per_s": speed,
                "equilibrium_convergence_criterion": "xpbd_static_tolerance",
            }

    try:
        input_name = Path(input_file).resolve().relative_to(ROOT).as_posix()
    except ValueError:
        input_name = Path(input_file).resolve().as_posix()
    result = {
        "project": cfg["project"],
        "model_status": "xpbd_static_equilibrium",
        "analysis": {
            "method": "XPBD pseudo-dynamic relaxation to a static position",
            "equilibrium_convergence_criterion": "xpbd_static_tolerance",
            "initial_equilibrium": initial_equilibrium,
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
        "results_count": len(rows),
        "results_complete": True,
        "fields": list(rows[0]),
        "results": rows,
    }
    out_primary, out_csv = write_results(
        rows, result, cfg, ROOT, output_json, output_csv, output_bin,
        default_json=None,
        default_csv="results/xpbd_static_results.csv",
        default_bin="results/xpbd_static_results.bin",
    )
    logging.info("XPBD results written to %s and %s", out_primary, out_csv)
    return result
