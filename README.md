# WinchSim

WinchSim runs quasi-static lifting models from finite-element-style JSON input and exports JSON/CSV time histories for the interactive viewer.

## Run a model

Use the same solver entry point for all input files:

```powershell
python solver/solve.py input/model.json
python solver/solve.py input/model_roller.json
python solver/solve.py input/model_roller_buoy.json
```

The input file controls the output paths under `solver.output_json` and `solver.output_csv`. If no input path is given, `input/model.json` is used. You can override output paths with `--output-json` and `--output-csv`.

`solver.analysis_method` selects the analysis implementation. The current options are `quasi_static_mesh_preview` and `quasi_static_lower_sling_profiles`; both run through the same command-line entry point. Winch motion is shared through `winch.mode`, speed, direction, and optional `reversal_time_s`.

`solver/solve.py` is the only command-line solver entry point. Analysis implementations are internal modules selected by the input file.

## Input format

Each model defines numbered `nodes` with 3D `position_m` coordinates. Node `role` is optional: use it only where the analysis needs special behavior or an identifiable joint, such as a winch, roller, fixed support, or lifting joint. Ordinary structural nodes need no role or name. `lines` define the geometry by connecting node IDs and specify an `element_type` and `component_id`; beam and truss topology comes from those connections, so no node ordering field is needed. `components` group lines into editable parts such as `beam1`, `rope1`, and `rope2`; each component points to an element property set under `elements`.

For example, a line can be assigned to `rope3` by setting its `component_id` to `rope3`, and a component can point to a shared or separate truss property dictionary. Lower slings can specify `subdivisions` and optionally `rest_length_m`. If rest length is omitted, the solver uses the initial endpoint spacing as the unstretched length, so changing node coordinates does not leave a stale hard-coded length behind. Set `rest_length_m` explicitly when the physical sling length differs from the initial spacing; an initial chord longer than that length is rejected with a line-specific input error.

The three supplied inputs use the same node/line layout:

- `input/model.json`: direct winch-to-joint lift, 150-second cycle.
- `input/model_roller.json`: fixed-roller route with the winch 40 m away, 150-second cycle.
- `input/model_roller_buoy.json`: fixed roller, buoy, and lower slings, 300-second up/down cycle.

The supplied truss line loads are `0.07615 kg/m` in air and `-0.00043125 kg/m` in water. The negative underwater value represents net buoyancy. Beam and buoy properties are stored in their own element dictionaries.

The buoy model's numerical convergence settings are under `solver.numerics`. Sling mesh density remains independently controlled by each lower-sling line's `subdivisions` value.

## Results and viewer

The solver exports one JSON and one CSV file per input model. Open `Open-WinchSim.bat` to launch the viewer, then use **Browse** to load the corresponding JSON result. The result JSON includes a scene graph built from the same input nodes and lines, so the viewer follows the input connectivity and component names.

## Model scope

The direct and fixed-roller cases are kinematic, quasi-static load estimates based on prescribed winch motion; they do not solve structural force equilibrium. In the roller-buoy case, lower-sling profiles are minimized as discrete inextensible trusses, but the beam/joint and buoy motion are prescribed with a tracking/floating/caught rule. The main-rope slack centerline uses an empirical sag display formula. These are not full-system physical equilibrium results.

Beam nodes are inferred from connectivity to structural beam lines; any node-order field is ignored. Analysis-specific roles remain required for special control and connection points (for example `upper_joint`, `lower_joint`, `roller`, `winch`, and `buoy_top`) and for line behavior such as `main_rope` and `lower_sling`. The current analyses still require their stated connectivity. Pretension analysis, a transient dynamic solver, elastic rope behavior, and full beam finite elements are not implemented. Setting `dynamic_effects_enabled` to true now stops with an explicit error rather than silently returning quasi-static results.

Remaining assumptions users cannot currently override through the input include the main-rope slack display curve (`0.2 m` of sag per metre of slack, capped at `1 m`), the buoy solver's tracking/floating/caught body-motion rule, and several numerical safety thresholds (zero-length checks, strict feasibility tolerance, and energy acceptance tolerance). The legacy geometry-only scene exporter also defaults to 24 lower-sling render sections; node/line inputs use each line's `subdivisions` instead. The bend-stiffness ratio is configurable under `solver.numerics`, but it is a numerical regularizer, not a physical rope bending stiffness derived from material properties.
