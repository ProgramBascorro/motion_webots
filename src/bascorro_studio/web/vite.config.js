import { resolve } from "path";
import { defineConfig, searchForWorkspaceRoot } from "vite";
import react from "@vitejs/plugin-react";
import { walkingVersionStorePlugin } from "./server/studioStorePlugin.js";

// Versi tersimpan tab Walking disimpan di DISK robot (lihat
// server/walkingVersionFile.js untuk alasannya), bukan di localStorage browser.
// BASCORRO_STUDIO_DATA_DIR cuma untuk uji: server uji terpisah dengan data
// sementara, supaya pengujian tidak pernah menyentuh versi milik operator.
const STUDIO_DATA_DIR = process.env.BASCORRO_STUDIO_DATA_DIR || resolve(__dirname, "../data");

export default defineConfig({
  plugins: [react(), walkingVersionStorePlugin({ dataDir: STUDIO_DATA_DIR })],
  resolve: {
    alias: [
      { find: /^three$/, replacement: resolve(__dirname, "src/three-compat.js") },
    ],
  },
  server: {
    host: true,
    port: 5173,
    // Acuan page 2 (data/walking_ready_reference.json) di-import langsung oleh
    // web/src/walkingPoseReference.js. Berkas itu di luar akar Vite, jadi
    // foldernya harus diizinkan -- tetap ../data, bukan STUDIO_DATA_DIR: acuan
    // cuma dibaca, dan server uji dengan data sementara tetap butuh acuan asli.
    fs: {
      allow: [searchForWorkspaceRoot(process.cwd()), resolve(__dirname, "../data")],
    },
  },
});
