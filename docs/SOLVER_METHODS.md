# Solver method invariants

These rules preserve the physical behavior used by the rigid-body quasi-static equilibrium solver. Changes to the nonlinear solve must keep these invariants.

## Rope loads and slack profiles

- Integrate rope weight and buoyancy over the deformed chain segments, using the material length assigned to each segment. Do not apply the whole rope's submerged load along the endpoint chord; slack rope can follow a different path from that chord.
- Slack lower slings carry no axial compression. Reconstruct their output path with fixed material link lengths and the air/water potential so the visible sling does not collapse or acquire an arbitrary short shape.
- The waterline is a fluid boundary, not a mechanical stop. The buoy remains afloat only while its buoyancy and the attached sling geometry permit it. Use realistic `rest_length_m` values for the complete winch travel when a buoy should stay near the surface.

## Equilibrium convergence

- Use the configured absolute force acceptance as a hard ceiling on the global force residual. A system-wide relative threshold may tighten that ceiling, but must never relax it; large beam loads must not hide imbalance on a lightly loaded buoy.
- Keep the physical force residual check in addition to the AquaSim iteration-change criterion. A small change between iterations alone does not establish static equilibrium.

The supplied roller-buoy model uses 45 m lower slings because its 50 m retrieval takes the beam corners to a maximum float-to-anchor span of about 40.9 m. Its sling length is an input assumption, not a value the solver should silently invent for other models.
