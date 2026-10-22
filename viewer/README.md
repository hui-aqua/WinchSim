# WinchSim Results Viewer

The viewer is hosted at [hui-aqua.github.io/WinchSim](https://hui-aqua.github.io/WinchSim/). Select **Browse** to load a simulation result or model file from your computer. Files are read by the browser and are not uploaded.

This directory contains the source used to build the GitHub Pages site. The deployment workflow rebuilds the site when files under `viewer/` change on `main` or `master`; it can also be started manually from the repository's Actions tab.

## Source layout

- `index.html` contains the page structure.
- `src/main.js` starts the viewer and renders simulation data.
- `src/xml-model.js` converts model XML to the viewer's scene format.
- `src/styles.css` contains the viewer styles.
