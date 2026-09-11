// Plugin Vite: server yang SUDAH menyajikan Bascorro Studio (pnpm dev) sekaligus
// melayani penyimpanan versi walking. Tidak ada proses tambahan yang bisa lupa
// dinyalakan -- kalau halaman Studio terbuka, penyimpanannya juga hidup.
//
//   GET  /__bascorro/walking-versions   -> keadaan lengkap
//   POST /__bascorro/walking-versions   -> satu operasi (upsert/delete/activate/ui/import)
//
// POST wajib membawa header X-Bascorro-Studio. Header khusus memaksa browser
// melakukan preflight CORS yang tidak pernah dijawab server ini, jadi halaman
// web lain yang kebetulan terbuka tidak bisa menulis ke penyimpanan robot.
import { createWalkingVersionFile, StoreError } from "./walkingVersionFile.js";

export const WALKING_STORE_ROUTE = "/__bascorro/walking-versions";
const MAX_BODY_BYTES = 2 * 1024 * 1024;

function readBody(req) {
  return new Promise((resolve, reject) => {
    let size = 0;
    const chunks = [];
    req.on("data", (chunk) => {
      size += chunk.length;
      if (size > MAX_BODY_BYTES) {
        reject(new StoreError(413, "isi permintaan terlalu besar"));
        req.destroy();
        return;
      }
      chunks.push(chunk);
    });
    req.on("end", () => resolve(Buffer.concat(chunks).toString("utf8")));
    req.on("error", reject);
  });
}

function send(res, status, body) {
  res.statusCode = status;
  res.setHeader("Content-Type", "application/json; charset=utf-8");
  res.setHeader("Cache-Control", "no-store");
  res.end(JSON.stringify(body));
}

function describe(operation) {
  switch (operation?.op) {
    case "upsert": return `simpan "${operation.version?.name}"`;
    case "delete": return `hapus ${operation.id}`;
    case "activate": return `aktifkan ${operation.id}`;
    case "ui": return "tampilan";
    case "import": return `impor ${operation.versions?.length ?? 0} versi dari browser`;
    default: return String(operation?.op);
  }
}

export function walkingVersionStorePlugin({ dataDir }) {
  const store = createWalkingVersionFile({ dataDir });

  const middleware = (logger) => async (req, res) => {
    const route = (req.url || "/").split("?")[0];
    if (route !== "/" && route !== "") {
      send(res, 404, { error: "tidak ada" });
      return;
    }
    try {
      if (req.method === "GET") {
        send(res, 200, await store.load());
        return;
      }
      if (req.method === "POST") {
        if (req.headers["x-bascorro-studio"] !== "1") {
          throw new StoreError(403, "permintaan tanpa header X-Bascorro-Studio ditolak");
        }
        let operation;
        try {
          operation = JSON.parse(await readBody(req));
        } catch (error) {
          if (error instanceof StoreError) throw error;
          throw new StoreError(400, `JSON tidak sah: ${error.message}`);
        }
        const state = await store.mutate(operation);
        logger.info(`[walking-store] ${describe(operation)} -> revisi ${state.revision} (${store.file})`, { timestamp: true });
        send(res, 200, state);
        return;
      }
      res.setHeader("Allow", "GET, POST");
      throw new StoreError(405, `metode ${req.method} tidak didukung`);
    } catch (error) {
      const status = error instanceof StoreError ? error.status : 500;
      logger.error(`[walking-store] GAGAL (${status}): ${error.message}`, { timestamp: true });
      send(res, status, { error: error.message });
    }
  };

  return {
    name: "bascorro-walking-version-store",
    configureServer(server) {
      server.middlewares.use(WALKING_STORE_ROUTE, middleware(server.config.logger));
      server.config.logger.info(`[walking-store] versi walking disimpan di ${store.file}`);
    },
    configurePreviewServer(server) {
      server.middlewares.use(WALKING_STORE_ROUTE, middleware(server.config.logger));
    },
  };
}
