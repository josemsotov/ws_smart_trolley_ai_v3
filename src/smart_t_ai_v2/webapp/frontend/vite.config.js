import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

export default defineConfig({
  plugins: [react()],
  build: {
    outDir: 'build',
  },
  server: {
    host: true,
    port: 5173,
    proxy: {
      '/ws':          { target: 'ws://localhost:8000',  ws: true },
      '/api':         { target: 'http://localhost:8000', changeOrigin: true },
      '/video_feed':  { target: 'http://localhost:8000', changeOrigin: true },
    },
  },
})
