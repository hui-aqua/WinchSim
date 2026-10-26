# Solver method invariants

These rules preserve the physical behavior used by the rigid-body quasi-static equilibrium solver. Changes to the nonlinear solve must keep these invariants.

## XPBD alternative

- `xpbd_static_equilibrium` is an opt-in method selected through `solver.analysis_method`; it does not replace or change `global_quasi_static_equilibrium`.
- XPBD relaxes under gravity, buoyancy, and nodal loads using compliant tension-only rope constraints. Connected beam assemblies are projected onto a best-fit rigid pose.
- XPBD static acceptance uses both the maximum per-step position change and speed for the configured number of consecutive steps. Report the XPBD motion tolerances and force residual diagnostic; do not label XPBD acceptance as AquaSim ChkNorm convergence.
- The current XPBD method supports rigid beam assemblies. Reject elastic beam models until bending and rotational beam constraints are implemented.

## Rope loads and slack profiles

- Integrate rope weight and buoyancy over the deformed chain segments, using the material length assigned to each segment. Do not apply the whole rope's submerged load along the endpoint chord; slack rope can follow a different path from that chord.
- Slack lower slings carry no axial compression. Reconstruct their output path with fixed material link lengths and the air/water potential so the visible sling does not collapse or acquire an arbitrary short shape.
- The waterline is a fluid boundary, not a mechanical stop. The buoy remains afloat only while its buoyancy and the attached sling geometry permit it. Use realistic `rest_length_m` values for the complete winch travel when a buoy should stay near the surface.

## Equilibrium convergence

- For the rigid-body solver, record AquaSim Eq. 142: `Norm = sum(|ΔF_i|^1.5 |Δu_i|^0.5)` and `ChkNorm = Norm / N`. Select acceptance with `equilibrium_convergence_criterion`: `force_residual` requires the projected force residual limit; `aquasim_chknorm` follows the AquaSim ChkNorm threshold. Always report the force residual as a diagnostic, including when ChkNorm is the acceptance criterion.
- Keep both ChkNorm and the projected global force residual in results and logs. For elastic-beam solves, the configured position residual criterion remains in use.
- The rigid-body energy solver ramps external loads from zero to full load using `solver.equilibrium_load_steps` (default `10`) and splits the configured iteration budget across these stages.
- Rigid-body tangent Newton steps are backtracked against the scaled force-residual norm. The first tangent solve also logs a directional finite-difference check of the assembled tangent (disable with `solver.tangent_finite_difference_check="false"`).

The supplied roller-buoy model uses 45 m lower slings because its 50 m retrieval takes the beam corners to a maximum float-to-anchor span of about 40.9 m. Its sling length is an input assumption, not a value the solver should silently invent for other models.
