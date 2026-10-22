# WinchSim

WinchSim calculates quasi-static lifting behavior for line and beam models. Models are stored as AquaSim-compatible XML and can also be opened in [AmodelViewer](https://github.com/hui-aqua/AmodelViewer). A run writes a CSV time history for analysis and a packed binary file for playback in the 3D viewer.

## Quick start

On Windows, open PowerShell in the project folder and install the solver dependencies once:

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements-solver.txt
```

Run one of the supplied models:

```powershell
.\.venv\Scripts\python.exe solver/solve.py input/model.xml
.\.venv\Scripts\python.exe solver/solve.py input/model_roller.xml
.\.venv\Scripts\python.exe solver/solve.py input/model_roller_buoy.xml
```

The solver prints progress and writes a timestamped log under `results/`. Results are checkpointed as the run proceeds, so a failed run retains the states completed before failure.

To view a result, open the [hosted viewer](https://hui-aqua.github.io/WinchSim/), select **Browse**, and open the corresponding `.bin` file from your generated `results/` folder. The viewer can also open an `.xml` or `.amodel` model for a static geometry preview.

The browser viewer is also available at [hui-aqua.github.io/WinchSim](https://hui-aqua.github.io/WinchSim/) after GitHub Pages deployment. It runs in the browser; choose **Browse** to load a local `.bin`, `.json`, `.xml`, or `.amodel` file. Files are not uploaded.

## Supplied examples

| Model | Description | Duration |
| --- | --- | ---: |
| `input/model.xml` | Direct lift | 150 s |
| `input/model_roller.xml` | Lift routed over a fixed roller | 150 s |
| `input/model_roller_buoy.xml` | Roller, lifting beam, buoy and lower slings | 600 s |

The current roller-buoy input retrieves rope at 0.1 m/s until 500 s, then pays out at 0.1 m/s through 600 s. See the [user guide](docs/USER_GUIDE.md), the [input file guide](docs/INPUT_FILE_GUIDE.md) for building a new case, and the [audit](docs/AUDIT.md) for current scope and known limitations.

## Results at a glance

- `results/*.csv`: rows by time step for Excel, Python and other analysis tools.
- `results/*.bin`: compact playback data for the viewer.
- `results/<model>_<timestamp>.log`: timestamped solver progress and errors.

The CSV and binary outputs follow the paths in the model’s `<solver>` element. The binary header contains the `WSIM` signature and metadata; the numerical time history is stored as little-endian Float32 values. Legacy JSON input/output support remains for compatibility.

## Model and solver scope

Input geometry and component properties are defined in XML. Simulation duration and time step are global solver settings; winch speed and direction are defined by a time-based schedule. The global quasi-static analysis includes tension-only ropes, gravity, waterline-dependent line loads, and rigid-body motion. It does not model inertia, waves, current drag, pulley friction, rope bending, or dynamic impacts. A completed solve means the configured equilibrium checks passed; it does not certify component strength or validate a model against a design standard.

See [docs/USER_GUIDE.md](docs/USER_GUIDE.md) for the workflow and input reference. See [docs/TODO.md](docs/TODO.md) for prioritized future work.
