# WinchSim

WinchSim runs quasi-static lifting models from finite-element-style JSON input and exports JSON/CSV time histories for the interactive viewer.

## Run a model

Use the same solver entry point for all input files:

```powershell
.\.venv\Scripts\python.exe solver/solve.py input/model.json
.\.venv\Scripts\python.exe solver/solve.py input/model_roller.json
.\.venv\Scripts\python.exe solver/solve.py input/model_roller_buoy.json
```

The workspace Python runtime is Python 3.12.12. To recreate the solver packages in a virtual environment, run `.python\cpython-3.12.12-windows-x86_64-none\python.exe -m venv .venv`, then `.venv\Scripts\python.exe -m pip install -r requirements-solver.txt`.

The input file controls the output paths under `solver.output_json` and `solver.output_csv`. If no input path is given, `input/model.json` is used. You can override output paths with `--output-json` and `--output-csv`.

`solver.analysis_method` selects the analysis implementation. The supplied models use `global_quasi_static_equilibrium`, which solves all connected components together. The earlier mesh preview and lower-sling profile methods remain available for older inputs. Winch motion is shared through `winch.mode`, speed, direction, and optional `reversal_time_s`.

`solver/solve.py` is the only command-line solver entry point. Analysis implementations are internal modules selected by the input file.

## Input format

Each model defines numbered `nodes` with 3D `position_m` coordinates. Node `role` is optional: use it only where the analysis needs special behavior or an identifiable joint, such as a winch, roller, fixed support, or lifting joint. Ordinary structural nodes need no role or name. `lines` define the geometry by connecting node IDs and specify an `element_type` and `component_id`; beam and truss topology comes from those connections, so no node ordering field is needed. `components` group lines into editable parts such as `beam1`, `rope1`, and `rope2`; each component points to an element property set under `elements`.

Beam components may set `structural_model` to `"rigid"` or `"elastic"`; the default is `"rigid"`. Elastic components require section properties in their assigned element dictionary: Young's modulus, shear modulus, area, both area moments, and torsional constant. A rigid buoy may set `rotation_behavior` to `"fixed"` to keep its axis upright while allowing translation.

For example, a line can be assigned to `rope3` by setting its `component_id` to `rope3`, and a component can point to a shared or separate truss property dictionary. Lower slings can specify `subdivisions` and optionally `rest_length_m`. If rest length is omitted, the solver uses the initial endpoint spacing as the unstretched length, so changing node coordinates does not leave a stale hard-coded length behind. Set `rest_length_m` explicitly when the physical sling length differs from the initial spacing; an initial chord longer than that length is rejected with a line-specific input error.

The three supplied inputs use the same node/line layout and select `global_quasi_static_equilibrium`:

- `input/model.json`: direct winch-to-joint lift, 150-second cycle.
- `input/model_roller.json`: fixed-roller route with the winch 40 m away, 150-second cycle.
- `input/model_roller_buoy.json`: fixed roller, buoy, and lower slings, 300-second up/down cycle.

The supplied truss line loads are `0.07615 kg/m` in air and `-0.00043125 kg/m` in water. The negative underwater value represents net buoyancy. Beam and buoy properties are stored in their own element dictionaries.

The buoy model's numerical convergence settings are under `solver.numerics`. Sling mesh density remains independently controlled by each lower-sling line's `subdivisions` value.

## Results and viewer

The solver exports one JSON and one CSV file per input model. Open `Open-WinchSim.bat` to launch the viewer, then use **Browse** to load the corresponding JSON result. The result JSON includes a scene graph built from the same input nodes and lines, so the viewer follows the input connectivity and component names.

## Model scope

The global analysis captures truss reference lengths from the input endpoint distances before equilibrium. For rigid beam components it minimizes the assembly potential energy with tension-only subdivided ropes, integrated gravity and hydrostatic line loads, and per-component rigid body motion. Elastic beam components use OpenSeesPy beam elements. The analysis is quasi-static: inertia, wave/current drag, rope bending stiffness, pulley friction, and dynamic impacts are not modeled. Install the solver packages with `.venv\Scripts\python.exe -m pip install -r requirements-solver.txt`.

The current roller-buoy input reaches an initial equilibrium, but the full lift cycle has not passed the solver force-balance acceptance check near buoy release. The solver stops on that state instead of writing a complete result file.

Beam nodes are inferred from connectivity to beam lines; any node-order field is ignored. Analysis-specific roles remain required for special control and connection points (for example `upper_joint`, `lower_joint`, `roller`, `winch`, and `buoy_top`) and line behavior such as `main_rope` and `lower_sling`. `dynamic_effects_enabled` remains unsupported and stops with an explicit error rather than silently returning quasi-static results.

Each truss line may specify `subdivisions`; otherwise `solver.default_truss_subdivisions` controls its equilibrium mesh density. Lower-sling display follows the solved finite-element centerline rather than a separately generated animation profile.
