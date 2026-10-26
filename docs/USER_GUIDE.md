# WinchSim user guide

This guide covers running a model, changing a winch schedule, opening results, and diagnosing a run that stops early.

## 1. Install and run

Use Python 3.12 on Windows. From PowerShell in the repository folder:

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements-solver.txt
.\.venv\Scripts\python.exe solver/solve.py input/model_roller_buoy.xml
```

Choose a different input path to run another model. The optional command-line overrides are:

```powershell
.\.venv\Scripts\python.exe solver/solve.py input/model.xml --output-bin results/direct.bin --output-csv results/direct.csv
```

`--output-json` is available for older workflows. The normal viewer workflow uses `.bin`.

## 2. Change simulation time and winch motion

The simulation duration and output interval belong to `<solver>`, not to the winch. For example:

```xml
<winch mode="prescribed_speed">
  <motion_schedule>
    <point time_s="0" speed_m_per_s="0.1" direction="in" />
    <point time_s="150" speed_m_per_s="0.1" direction="out" />
  </motion_schedule>
</winch>
<solver duration_s="300" time_step_s="1" ... />
```

Schedule rows are ordered start times. A row’s speed and direction apply until the next row. Use a non-negative speed and `direction="in"` to retrieve rope, or `direction="out"` to pay it out. The first row must start at zero, and row times must increase. The example above retrieves for 150 seconds and pays out for the next 150 seconds.

Useful global settings:

| Setting | Meaning |
| --- | --- |
| `duration_s` | Total simulated time. |
| `time_step_s` | Requested output interval. Failed intervals may be solved internally at smaller continuation steps. |
| `continuation_min_step_s` | Smallest interval allowed while retrying a failed equilibrium. |
| `output_bin` | Binary viewer result path. |
| `output_csv` | CSV result path. |
| `equilibrium_convergence_criterion` | Rigid-body acceptance rule: `force_residual` (default) or `aquasim_chknorm`. With `aquasim_chknorm`, WinchSim follows the AquaSim Eq. 142 threshold and reports force residual as a diagnostic. |
| `aquasim_convergence_criteria` | Threshold for AquaSim Eq. 142 `ChkNorm`, used when `equilibrium_convergence_criterion="aquasim_chknorm"`. |

Model coordinates use metres, time uses seconds, mass uses kilograms, and force uses newtons. `waterline_z_m` defines the still-water surface and `gravity_m_per_s2` defines gravity.

## 3. Input model layout

WinchSim reads node and line models in AquaSim-compatible `.xml` or `.amodel` form. The main XML sections are:

- `<Nodes>`: node IDs, positions, and optional names and roles.
- `<Components>`: beam and truss property sets and connected elements.
- `<winch>`: prescribed motion schedule.
- `<environment>`: waterline and gravity.
- `<solver>`: duration, output interval, solver settings, and output paths.

For an optional XPBD static-position solve, set `<solver analysis_method="xpbd_static_equilibrium">`. This selects a separate pseudo-dynamic relaxation path; the existing global equilibrium solver remains the default for current models. XPBD supports tension-only trusses and rigid beam assemblies, and reports its own position-change/speed convergence values.

Roles identify solver-specific connection points or line behavior. Current examples use roles such as `upper_joint`, `lower_joint`, `roller`, `winch`, `buoy_top`, `main_rope`, `upper_sling`, `lower_sling`, and `buoy_body`. Keep IDs unique and ensure every line endpoint references a defined node.

The examples in `input/` are a practical starting point. Preserve a copy before changing material values or connectivity. For a field-by-field reference and complete starter model, see the [input file guide](INPUT_FILE_GUIDE.md). AquaSim-compatible input can also be opened in [AmodelViewer](https://github.com/hui-aqua/AmodelViewer) for geometry inspection.

## 4. Results and playback

Every run creates a timestamped log in `results/`, for example `model_roller_buoy_20261021_223240.log`. It records the input path, selected analysis, step progress, convergence values, checkpoint paths, and exceptions.

The solver checkpoints `.bin` and `.csv` outputs after each requested output state. If a later state fails, the last saved files contain the preceding states and the binary metadata marks the result incomplete. A complete result has `results_complete: true` and ends at the configured duration.

To open playback, visit the [hosted WinchSim viewer](https://hui-aqua.github.io/WinchSim/), choose **Browse**, and select the `.bin` file from your generated `results/` folder. The viewer runs on GitHub Pages and reads the selected file in your browser; it does not upload the file. Use the timeline and playback controls to inspect the run, and the component controls to show or hide parts.

The CSV can be opened in Excel or loaded into Python with `pandas.read_csv`. It contains time, equilibrium/convergence fields, line force and geometry information, and solved mesh node coordinates. The `.bin` stores the same numeric history in packed Float32 form with scene metadata.

## 5. Read a solver log

Look for the last `INFO Solving equilibrium at t=...` and the matching outcome:

- `Equilibrium converged`: the requested rigid-body output state passed the configured criterion. The log and result CSV identify the criterion and report both AquaSim `ChkNorm` and projected force residual.
- `retrying continuation`: the solver is trying smaller internal time intervals to reach that output state.
- `Checkpoint saved`: the displayed `.bin` and `.csv` contain all output states through that time.
- `Solver run completed successfully`: all requested output states were written.
- `Solver run failed`: inspect the exception immediately below it and the last force residual, AquaSim `Norm`/`ChkNorm`, and convergence limit.

An OpenSees predictor warning can be followed by a successful rigid-energy fallback. A predictor warning alone does not mean the run failed; use the final completion line and `results_complete` metadata.

## 6. Model limits

The current global analysis is quasi-static. It does not include inertia, dynamic impact, wave/current drag, pulley friction, or rope bending stiffness. The equilibrium model uses supplied material and buoyancy properties and has no component failure or allowable-load check. Very high calculated tension should therefore be reviewed against real component data. See [the audit](AUDIT.md) and [future work](TODO.md) for open engineering and software work.
