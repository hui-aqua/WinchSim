# WinchSim Results Viewer

The browser viewer is a standalone Vite app. It reads simulation result files locally in the browser; no result data is uploaded.

## Run locally

From this directory, run `npm ci` once, then `npm run dev`. Open the local URL printed by Vite (normally `http://127.0.0.1:8000/`).

## Build

Run `npm run build` to create the static site in `dist/`.

## Source layout

- `index.html` contains the page structure.
- `src/main.js` starts the viewer and renders simulation data.
- `src/xml-model.js` converts model XML to the viewer's scene format.
- `src/styles.css` contains the viewer styles.

The GitHub Pages workflow publishes this app when files under `viewer/` change on `main` or `master`. It can also be started manually from the repository's Actions tab.
