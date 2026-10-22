import { defineConfig } from 'vite';
export default defineConfig({
  root: '.',
  base: process.env.GITHUB_PAGES === 'true' ? '/WinchSim/' : '/',
  server: {
    port: 8000,
    strictPort: true,
    fs: { allow: ['..'] }
  },
  build: { outDir: 'dist', emptyOutDir: true }
});
