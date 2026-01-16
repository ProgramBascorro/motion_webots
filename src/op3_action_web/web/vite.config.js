import { resolve } from "path";
import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

export default defineConfig({
  plugins: [react()],
  resolve: {
    alias: [
      { find: /^three$/, replacement: resolve(__dirname, "src/three-compat.js") },
    ],
  },
  server: {
    host: true,
    port: 5173,
  },
});
