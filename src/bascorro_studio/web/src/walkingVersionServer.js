// Sisi browser dari penyimpanan versi walking di disk robot
// (server/walkingVersionFile.js, disajikan plugin Vite server/studioStorePlugin.js).
//
// Semua fungsi di sini mengembalikan KEADAAN LENGKAP dari server sesudah
// operasinya selesai, dan melempar galat yang jelas kalau gagal. Tidak ada yang
// ditelan: pemanggil wajib memperlihatkan kegagalan ke operator, karena
// "tersimpan" yang ternyata tidak tersimpan adalah persis bug yang diperbaiki.
export const WALKING_STORE_URL = "/__bascorro/walking-versions";
const TIMEOUT_MS = 8000;

async function request(method, operation, fetchImpl) {
  const controller = new AbortController();
  const timer = setTimeout(() => controller.abort(), TIMEOUT_MS);
  try {
    const response = await fetchImpl(WALKING_STORE_URL, {
      method,
      cache: "no-store",
      signal: controller.signal,
      headers: operation ? { "Content-Type": "application/json", "X-Bascorro-Studio": "1" } : undefined,
      body: operation ? JSON.stringify(operation) : undefined,
    });
    let data = null;
    try {
      data = await response.json();
    } catch {
      // Server tanpa plugin ini menjawab dengan index.html, bukan JSON.
    }
    if (!response.ok) throw new Error(data?.error || `server penyimpanan menjawab HTTP ${response.status}`);
    if (!data || !Array.isArray(data.versions)) {
      throw new Error("server tidak melayani penyimpanan versi walking (plugin Vite belum aktif?)");
    }
    return data;
  } catch (error) {
    if (error?.name === "AbortError") throw new Error(`server penyimpanan tidak menjawab dalam ${TIMEOUT_MS / 1000} detik`);
    throw error;
  } finally {
    clearTimeout(timer);
  }
}

export const fetchWalkingStore = (fetchImpl = globalThis.fetch) => request("GET", null, fetchImpl);
export const mutateWalkingStore = (operation, fetchImpl = globalThis.fetch) => request("POST", operation, fetchImpl);
