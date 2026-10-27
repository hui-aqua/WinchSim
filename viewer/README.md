# WinchSim AquaSim Model Editor

The viewer is hosted at [hui-aqua.github.io/WinchSim](https://hui-aqua.github.io/WinchSim/). It opens into a new, blank AquaSim-style model. No file is needed to begin editing.

## Create and save a model

1. Add nodes by XYZ coordinates or create a point relative to a selected node.
2. Choose a beam or truss property set and create a line by picking its two endpoint nodes in the viewport. Add more property sets with **Beam set** or **Truss set**.
3. Select nodes or lines in the viewport or Model tree to inspect and edit their properties.
4. Choose **Save XML** to download the current model. The XML keeps the environment, nodes, components, material properties, element connectivity, and supported WinchSim solver settings.

Use **Open model or results** to open an existing `.xml`, `.amodel`, JSON, or binary results file. Existing result playback and visualization tools remain available.

## Build and deploy

This directory contains the source used to build the GitHub Pages site. The deployment workflow rebuilds the site when files under `viewer/` change on `main` or `master`; it can also be started manually from the repository's Actions tab.

- `index.html` contains the page structure.
- `src/main.js` starts the editor and viewer.
- `src/xml-model.js` converts model XML to the viewer's scene format.
- `src/styles.css` contains the viewer styles.
