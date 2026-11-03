import { parseWalkingNumber, snapFloat32, WALKING_BIAS_FIELDS } from "./walkingParams.js";
// SATU sumber acuan, dibaca juga oleh demo D60 (auto_demo.cpp). Ditulis
// scripts/walking_ready_fk.py dari page 2 WALKING_READY di ALPHONSE.bin lewat IK
// walking_module yang di-port 1:1 -- jangan disunting tangan. Sesudah page 2
// di-teach ulang: tutup Studio, jalankan skrip itu dengan --write (ia juga
// menggeser semua versi tersimpan supaya angka relatifnya tetap), buka lagi.
import pageTwoReference from "../../data/walking_ready_reference.json" with { type: "json" };

// Urutan tampilan = urutan kotak isian grup Init Pose.
const REFERENCE_KEYS = [
  "init_x_offset", "init_y_offset", "init_z_offset", "init_roll_offset",
  "init_pitch_offset", "init_yaw_offset", "hip_pitch_offset",
];

function referenceOffset(key) {
  // y tidak pernah masuk acuan bias: computeNeutralLegAngle(..., false) di
  // op3_walking_module.cpp mengeluarkannya, jadi y selalu lebar TAMBAHAN di
  // atas page 2 dan nolnya memang 0.
  if (key === "init_y_offset") return 0;
  const value = pageTwoReference.offsets?.[key];
  // Berkas rusak harus gagal keras di sini: acuan yang diam-diam jadi 0 berarti
  // setiap preset berdiri belasan derajat dari page 2.
  if (typeof value !== "number" || !Number.isFinite(value)) {
    throw new Error(`walking_ready_reference.json: ${key} tidak ada atau bukan angka`);
  }
  return value;
}

export const WALKING_POSE_REFERENCE = Object.freeze({
  id: pageTwoReference.id,
  label: `Page ${pageTwoReference.source.page} · ${pageTwoReference.source.pageName}`,
  source: `FK dari ${pageTwoReference.source.actionFile} page ${pageTwoReference.source.page}, `
    + `${pageTwoReference.generatedAt.slice(0, 10)} (scripts/walking_ready_fk.py)`,
  offsets: Object.freeze(Object.fromEntries(REFERENCE_KEYS.map((key) => [key, referenceOffset(key)]))),
});

export function walkingReferenceOffset(key, relative = true) {
  return relative && Object.hasOwn(WALKING_POSE_REFERENCE.offsets, key)
    ? WALKING_POSE_REFERENCE.offsets[key] : null;
}

export function walkingOffsetToDisplay(value, referenceOffset = null) {
  if (referenceOffset === null || referenceOffset === 0) return value;
  const native = parseWalkingNumber(value, NaN);
  if (!Number.isFinite(native) || !Number.isFinite(Math.fround(native))) return value;
  if (Math.fround(native) === Math.fround(referenceOffset)) return 0;
  const difference = native - referenceOffset;
  for (let digits = 1; digits <= 17; digits += 1) {
    const candidate = Number(difference.toPrecision(digits));
    if (Math.fround(candidate + referenceOffset) === Math.fround(native)) return candidate;
  }
  return difference;
}

export function walkingOffsetFromDisplay(value, referenceOffset = null) {
  if (referenceOffset === null || referenceOffset === 0) return value;
  const relative = parseWalkingNumber(value, NaN);
  if (!Number.isFinite(relative)) return value;
  return snapFloat32(relative + referenceOffset);
}

// Init pose yang ditulis sebagai SELISIH dari page 2 -> nilai asli ROS, untuk
// preset yang tertanam di kode. Nilai aslinya ikut bergeser tiap kali acuan
// diturunkan ulang, selisihnya tidak -- sama seperti versi tersimpan yang
// digeser walking_ready_fk.py.
export function walkingPoseFromReference(relative = {}) {
  return Object.fromEntries(WALKING_BIAS_FIELDS.map((key) => [
    key, walkingOffsetFromDisplay(relative[key] ?? 0, WALKING_POSE_REFERENCE.offsets[key]),
  ]));
}
