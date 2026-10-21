# Future work

Priorities are ordered by impact on user trust and engineering usefulness. Items are proposals; completion requires acceptance criteria below and review against trusted reference cases.

## P0 — Establish repeatable correctness checks

- [ ] Add a small automated test suite for XML/legacy input parsing, role and connectivity validation, winch schedule integration, waterline loads, binary/CSV result writing, and partial-run metadata.
- [ ] Save compact reference cases with expected positions, reactions, tensions, and convergence values.
- [ ] Compare representative cases with trusted AquaSim/reference results and record tolerances and known differences.
- [ ] Add a clear CI or local command that runs the checks without launching a long simulation.

**Acceptance:** a code change can be checked locally and produces a readable pass/fail report; reference comparisons state units and tolerances.

## P1 — Improve engineering review of results

- [ ] Add optional user-configured allowable tension/strain limits and warning/reporting first. Do not infer rope breaking strength from Young’s modulus or area alone.
- [ ] Show maximum component load, force residual, and incomplete/complete status in the viewer and CSV summary.
- [ ] Validate waterline behavior across rising, falling, and partially submerged cases against analytical buoyancy calculations.
- [ ] Report high extension ratios and suspicious slack/taut transitions without silently treating them as material failure.
- [ ] Document the exact force-residual scaling and convergence acceptance used for each analysis method.

**Acceptance:** each engineering warning traces to an explicit input value and a documented equation; no warning is presented as a certified safety result.

## P1 — Make long-run solving easier to diagnose

- [ ] Record the worst residual degree of freedom/component and the corresponding load or constraint in failure logs.
- [ ] Summarize continuation attempts, minimum interval reached, and the last saved output time at the end of a failed run.
- [ ] Evaluate adaptive output stepping based on convergence difficulty while keeping requested output times stable and reproducible.
- [ ] Profile the rigid equilibrium solve and optimize the common case before increasing default iteration limits.

**Acceptance:** a failed example can be diagnosed from its log and partial result without reconstructing a traceback by hand; adaptive runs can be compared at equivalent output times.

## P2 — Expand model capabilities

- [ ] Add dynamics only with a separately validated time integration method and clearly defined mass, damping, and boundary conditions.
- [ ] Add wave/current loading, pulley friction, and rope bending stiffness as explicit model features with independent validation cases.
- [ ] Support more AquaSim XML variants and report unsupported fields rather than silently ignoring them.
- [ ] Add a versioned input schema and a user-facing preflight report for units, roles, connectivity, material values, and motion schedule.

## P2 — Improve everyday workflow

- [ ] Add optional per-run output folders so logs, CSV, and binary files stay together without overwriting another run.
- [ ] Add a concise command-line summary for output paths, result completeness, final time, and warnings.
- [ ] Add viewer controls for partial-result status and direct access to the matching log.
- [ ] Document a clean setup on Windows and one supported non-Windows environment.

## Deferred by current project direction

- Breaking-load or rope-failure behavior is intentionally not implemented now. Revisit it only after component allowable-load inputs and reference validation are defined.
