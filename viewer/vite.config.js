import { defineConfig } from 'vite';
export default defineConfig({
  root: '.',
  server: {
    port: 8000,
    strictPort: true,
    fs: { allow: ['..'] }
  },
  build: { outDir: 'dist', emptyOutDir: true }
});
