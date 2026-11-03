// Penyimpanan "Versi Tersimpan" tab Walking di DISK PC robot, bukan di browser.
//
// Kenapa tidak lagi di localStorage (terukur 2026-09-12, bukan dugaan):
//
// 1. Kuotanya PENUH. localStorage satu alamat dibatasi Firefox 5.242.880 unit,
//    dan localhost:5173 sudah terpakai 5.124.449 (97,7%) -- hampir seluruhnya
//    riwayat undo Action Editor (6 entri x ~638.000 unit, batas kodenya 120
//    entri, jadi ia terus mengisi sampai kuota habis). Begitu penuh,
//    localStorage.setItem melempar QuotaExceededError, dan penulis versi walking
//    menelannya diam-diam: layar bilang "updated", disk tetap berisi nilai
//    lama, dan setelah Studio dibuka ulang nilai lama itulah yang muncul.
// 2. localStorage terpisah PER ALAMAT dan PER BROWSER. localhost:5173,
//    10.137.162.166:5173, 10.104.107.241:5173 masing-masing punya daftar
//    sendiri; alamat baru = daftar kosong berisi preset bawaan saja, yang di
//    mata operator persis "parameter kembali ke default".
//
// Berkas di disk tidak berbagi kuota dengan apa pun, sama untuk semua alamat dan
// browser, dan bisa dibaca skrip (sync_preset_from_studio.py).
//
// Aturan yang dijaga di sini, dan diuji di test/walkingVersionFile.test.mjs:
// - Nilai disimpan APA ADANYA. Tidak ada pembulatan, konversi, clamp, atau
//   pengisian default: yang tidak sah DITOLAK, bukan "diperbaiki".
// - Tiap versi berdiri sendiri (disalin per field, tidak ada objek bersama).
// - Save tidak pernah mengganti nama versi, dan versi baru tidak boleh memakai
//   nama versi lain -- dua-duanya jalan yang dulu membuat satu versi diam-diam
//   menimpa atau menggantikan versi lain.
// - Tiap tulis: berkas sementara -> fsync -> salinan cadangan berkas lama ->
//   rename atomik -> dibaca ulang dan dibandingkan byte-per-byte. Listrik
//   padam di tengah jalan menyisakan berkas lama atau baru, tidak pernah
//   setengah.
import { promises as fs } from "node:fs";
import path from "node:path";
import { WALKING_KEYS, WALKING_BOOL_FIELDS, WALKING_INT_FIELDS } from "../src/walkingParams.js";

export const STORE_FORMAT = "bascorro-studio/walking-versions";
export const STORE_FORMAT_VERSION = 1;
export const STORE_FILE_NAME = "walking_versions.json";
export const STORE_BACKUP_DIR = "backup";
// Acuan page 2 (scripts/walking_ready_fk.py). Angka init pose tiap versi adalah
// nilai asli ROS TERHADAP acuan itu; berkas versi membawa id-nya di
// `poseReferenceId`, dan skrip itulah satu-satunya yang menggesernya.
export const REFERENCE_FILE_NAME = "walking_ready_reference.json";

export class StoreError extends Error {
  constructor(status, message) {
    super(message);
    this.status = status;
  }
}

const ID_PATTERN = /^[A-Za-z0-9_.:-]{1,200}$/;

export function checkWalkingVersion(version) {
  if (!version || typeof version !== "object" || Array.isArray(version)) {
    throw new StoreError(400, "versi harus berupa objek");
  }
  const { id, name, createdAt, updatedAt, params } = version;
  if (typeof id !== "string" || !ID_PATTERN.test(id)) {
    throw new StoreError(400, `id versi tidak sah: ${JSON.stringify(id)}`);
  }
  if (typeof name !== "string" || !name.trim() || name.length > 200) {
    throw new StoreError(400, `nama versi tidak sah: ${JSON.stringify(name)}`);
  }
  for (const [key, value] of [["createdAt", createdAt], ["updatedAt", updatedAt]]) {
    if (typeof value !== "number" || !Number.isFinite(value)) {
      throw new StoreError(400, `"${name}": ${key} harus berupa angka waktu`);
    }
  }
  if (!params || typeof params !== "object" || Array.isArray(params)) {
    throw new StoreError(400, `"${name}": params harus berupa objek`);
  }
  const unknown = Object.keys(params).filter((key) => !WALKING_KEYS.includes(key));
  if (unknown.length) {
    throw new StoreError(400, `"${name}": field tidak dikenal ${unknown.join(", ")}`);
  }
  for (const key of WALKING_KEYS) {
    const value = params[key];
    if (WALKING_BOOL_FIELDS.has(key)) {
      if (typeof value !== "boolean") throw new StoreError(400, `"${name}": ${key} harus true/false`);
      continue;
    }
    if (typeof value !== "number" || !Number.isFinite(value) || !Number.isFinite(Math.fround(value))) {
      throw new StoreError(400, `"${name}": ${key} harus angka berhingga (didapat ${JSON.stringify(value)})`);
    }
    if (WALKING_INT_FIELDS.has(key) && !Number.isInteger(value)) {
      throw new StoreError(400, `"${name}": ${key} harus bilangan bulat`);
    }
  }
}

// Salinan baru per versi: tidak ada objek params yang dipakai bersama dua versi.
function copyVersion(version) {
  return {
    id: version.id,
    name: version.name,
    createdAt: version.createdAt,
    updatedAt: version.updatedAt,
    params: { ...version.params },
  };
}

export function emptyStoreState() {
  return {
    format: STORE_FORMAT,
    formatVersion: STORE_FORMAT_VERSION,
    revision: 0,
    savedAt: null,
    activeVersionId: null,
    ui: {},
    versions: [],
  };
}

// Murni: keadaan lama + satu operasi -> keadaan baru. Tidak menyentuh disk.
export function applyStoreOperation(state, operation) {
  const versions = state.versions;
  switch (operation?.op) {
    case "upsert": {
      checkWalkingVersion(operation.version);
      const version = copyVersion(operation.version);
      const index = versions.findIndex((item) => item.id === version.id);
      if (index >= 0) {
        if (versions[index].name !== version.name) {
          throw new StoreError(409, `Versi "${versions[index].name}" tidak boleh berganti nama lewat Save `
            + `(diminta "${version.name}"). Simpan sebagai versi baru kalau perlu nama lain.`);
        }
        const next = versions.slice();
        next[index] = version;
        return { state: { ...state, versions: next, activeVersionId: version.id }, result: { mode: "update" } };
      }
      const clash = versions.find((item) => item.name === version.name);
      if (clash) {
        throw new StoreError(409, `Nama "${version.name}" sudah dipakai versi lain. `
          + "Buka versi itu dulu kalau memang mau menimpanya.");
      }
      return { state: { ...state, versions: [version, ...versions], activeVersionId: version.id }, result: { mode: "new" } };
    }
    case "delete": {
      if (!versions.some((item) => item.id === operation.id)) {
        throw new StoreError(404, `versi ${JSON.stringify(operation.id)} tidak ada`);
      }
      return {
        state: {
          ...state,
          versions: versions.filter((item) => item.id !== operation.id),
          activeVersionId: state.activeVersionId === operation.id ? null : state.activeVersionId,
        },
        result: {},
      };
    }
    case "activate": {
      const id = operation.id ?? null;
      if (id !== null && !versions.some((item) => item.id === id)) {
        throw new StoreError(404, `versi ${JSON.stringify(id)} tidak ada`);
      }
      return { state: { ...state, activeVersionId: id }, result: {} };
    }
    case "ui": {
      if (typeof operation.ui?.relativeDisplay !== "boolean") {
        throw new StoreError(400, "ui.relativeDisplay harus true/false");
      }
      return { state: { ...state, ui: { ...state.ui, relativeDisplay: operation.ui.relativeDisplay } }, result: {} };
    }
    case "import": {
      // Hanya MENAMBAH id yang belum ada. Tidak pernah menimpa, mengganti nama,
      // atau membuang yang sudah tersimpan, jadi aman diulang (StrictMode di
      // mode dev memang mengirimnya dua kali).
      if (!Array.isArray(operation.versions)) throw new StoreError(400, "import butuh daftar versi");
      const next = versions.slice();
      const rejected = [];
      let imported = 0;
      for (const candidate of operation.versions) {
        try {
          checkWalkingVersion(candidate);
        } catch (error) {
          rejected.push({ id: candidate?.id ?? null, name: candidate?.name ?? null, reason: error.message });
          continue;
        }
        if (next.some((item) => item.id === candidate.id)) continue;
        next.push(copyVersion(candidate));
        imported += 1;
      }
      let activeVersionId = state.activeVersionId;
      if (!activeVersionId && next.some((item) => item.id === operation.activeVersionId)) {
        activeVersionId = operation.activeVersionId;
      }
      return { state: { ...state, versions: next, activeVersionId }, result: { imported, rejected } };
    }
    default:
      throw new StoreError(400, `operasi tidak dikenal: ${JSON.stringify(operation?.op)}`);
  }
}

export function createWalkingVersionFile({ dataDir, keepBackups = 200, now = () => Date.now() }) {
  const file = path.join(dataDir, STORE_FILE_NAME);
  const backupDir = path.join(dataDir, STORE_BACKUP_DIR);

  // Satu penulis pada satu waktu: dua Save yang datang bersamaan tidak boleh
  // saling membaca keadaan lama lalu saling menimpa.
  let chain = Promise.resolve();
  const exclusive = (task) => {
    const run = chain.then(task);
    chain = run.catch(() => {});
    return run;
  };

  async function readStateFile() {
    let text;
    try {
      text = await fs.readFile(file, "utf8");
    } catch (error) {
      if (error.code === "ENOENT") return { exists: false, state: emptyStoreState() };
      throw error;
    }
    let parsed;
    try {
      parsed = JSON.parse(text);
    } catch (error) {
      // Sengaja TIDAK diganti kosong: menulis daftar kosong di sini akan
      // menghapus semua versi. Biarkan berkasnya, dan katakan dengan jelas.
      throw new StoreError(500, `${file} rusak (${error.message}). Berkas tidak disentuh; `
        + `salinan sebelumnya ada di ${backupDir}.`);
    }
    if (parsed?.format !== STORE_FORMAT || !Array.isArray(parsed.versions)) {
      throw new StoreError(500, `${file} bukan berkas versi walking yang dikenali; tidak disentuh.`);
    }
    return { exists: true, state: parsed };
  }

  // null kalau acuan belum dibuat (atau server uji dengan data sementara).
  async function readReferenceId() {
    try {
      const reference = JSON.parse(await fs.readFile(path.join(dataDir, REFERENCE_FILE_NAME), "utf8"));
      return typeof reference?.id === "string" ? reference.id : null;
    } catch {
      return null;
    }
  }

  async function backupCurrentFile(revision) {
    try {
      await fs.access(file);
    } catch {
      return;
    }
    await fs.mkdir(backupDir, { recursive: true });
    const stamp = new Date(now()).toISOString().replace(/[:.]/g, "-");
    await fs.copyFile(file, path.join(backupDir, `walking_versions.${stamp}.r${revision}.json`));
    const names = (await fs.readdir(backupDir))
      .filter((name) => name.startsWith("walking_versions.") && name.endsWith(".json"))
      .sort();
    for (const name of names.slice(0, Math.max(0, names.length - keepBackups))) {
      await fs.unlink(path.join(backupDir, name));
    }
  }

  async function writeStateFile(state, previousRevision) {
    await fs.mkdir(dataDir, { recursive: true });
    const text = `${JSON.stringify(state, null, 2)}\n`;
    const temporary = `${file}.tmp-${process.pid}-${now()}`;
    const handle = await fs.open(temporary, "w");
    try {
      await handle.writeFile(text, "utf8");
      await handle.sync();
    } finally {
      await handle.close();
    }
    await backupCurrentFile(previousRevision);
    await fs.rename(temporary, file);
    try {
      const directory = await fs.open(dataDir, "r");
      try {
        await directory.sync();
      } finally {
        await directory.close();
      }
    } catch {
      // Beberapa sistem berkas tidak mengizinkan fsync direktori; rename-nya
      // sendiri sudah atomik.
    }
    return text;
  }

  const view = (exists, state) => ({
    exists,
    poseReferenceId: state.poseReferenceId ?? null,
    revision: state.revision ?? 0,
    savedAt: state.savedAt ?? null,
    activeVersionId: state.activeVersionId ?? null,
    ui: state.ui ?? {},
    versions: state.versions,
  });

  return {
    file,
    backupDir,
    load: () => exclusive(async () => {
      const { exists, state } = await readStateFile();
      return view(exists, state);
    }),
    mutate: (operation) => exclusive(async () => {
      const { exists, state } = await readStateFile();
      const { state: changed, result } = applyStoreOperation(state, operation);
      const revision = (state.revision ?? 0) + 1;
      const next = {
        ...changed,
        format: STORE_FORMAT,
        formatVersion: STORE_FORMAT_VERSION,
        revision,
        savedAt: new Date(now()).toISOString(),
      };
      // Berkas yang LAHIR dari Save di Studio ini memakai acuan yang sedang
      // berlaku, jadi langsung ditandai. Impor dari browser sengaja tidak:
      // isinya bisa dari acuan mana pun, dan walking_ready_fk.py yang memutuskan.
      // Berkas yang sudah ada tidak pernah ditandai ulang di sini -- tandanya
      // ikut terbawa lewat ...changed.
      if (!exists && operation?.op === "upsert") {
        const referenceId = await readReferenceId();
        if (referenceId) next.poseReferenceId = referenceId;
      }
      const written = await writeStateFile(next, state.revision ?? 0);
      // BUKTI, bukan asumsi: yang ada di disk harus sama persis dengan yang
      // dimaksud sebelum operator diberi tahu "tersimpan".
      const reread = await fs.readFile(file, "utf8");
      if (reread !== written) {
        throw new StoreError(500, "verifikasi gagal: isi berkas di disk tidak sama dengan yang ditulis");
      }
      return { ...view(true, JSON.parse(reread)), result };
    }),
  };
}
