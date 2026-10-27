# WinchSim AquaSim Model Editor

The viewer is hosted at [hui-aqua.github.io/WinchSim](https://hui-aqua.github.io/WinchSim/). It opens into a blank model and saves an editable draft in browser storage. The model tree and geometry are shared across six preparation tabs.

## Editor tabs

1. **Geometry** — create points by coordinates or relative offsets, lines between selected points, and triangle or quad surface connectivity.
2. **Elements** — create beam, truss, and membrane property sets, then assign selected lines to beam or truss sets. Sea floor support is reserved.
3. **Materials** — edit component EA, area, mass per metre, and beam section properties; edit breaking strength and unstretched length for a selected line. Density is derived from mass and area.
4. **Boundary** — set point constraints, point loads, and model winch settings. Line break events are reserved.
5. **Loads** — edit truss drag and added-mass coefficients. Lift coefficient and membrane solidity/load coefficients are reserved until their AquaSim XML mappings are confirmed.
6. **Solver** — edit solver method, duration, time step, environment waterline/gravity, waves, and current settings.
7. **Results** — open result files for playback and inspect statistics.

Tabs 1–6 edit one shared XML model. Changes are autosaved as a local browser draft; **Save XML** downloads the model for use with AquaSim. Use **New** to start another blank case or **Open model or results** in Results to import an existing file.

## Build and deploy

This directory contains the source used to build the GitHub Pages site. The deployment workflow rebuilds the site when files under `viewer/` change on `main` or `master`; it can also be started manually from the repository's Actions tab.

- `index.html` contains the page structure.
- `src/main.js` starts the editor and viewer.
- `src/xml-model.js` converts model XML to the viewer's scene format.
- `src/styles.css` contains the viewer styles.
