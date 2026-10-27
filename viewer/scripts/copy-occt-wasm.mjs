import { copyFile, mkdir } from 'node:fs/promises';
import { dirname, resolve } from 'node:path';
const targetDir = resolve('public');
await mkdir(targetDir, { recursive: true });
for (const file of ['occt-import-js.js', 'occt-import-js.wasm', 'LICENSE.md']) {
  const targetName = file === 'LICENSE.md' ? 'occt-import-js.LICENSE.md' : file;
  await copyFile(resolve('node_modules/occt-import-js', file === 'LICENSE.md' ? file : `dist/${file}`), resolve(targetDir, targetName));
}
