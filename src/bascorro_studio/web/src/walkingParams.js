// Shared by the two places that edit op3_walking_module's WalkingParam: the Walking
// tab in App.jsx and the Walking Tuner card in TuningPage.jsx.
//
// Both used to convert their input fields with a bare Number(). That has two failure
// modes on a form the user is actively typing in:
//   Number("")    === 0    -- a field cleared to be retyped is applied as 0
//   Number("0,5") === NaN  -- the Indonesian decimal comma is not a number
// The first one is the damaging one. On the amplitudes it silently undoes an edit; on
// period_time it is worse, because updateTimeParam() divides by period_time and every
// gait angle downstream turns into NaN.
//
// `fallback` is what a blank or unparseable field falls back to -- the caller decides
// whether that is the default gait or the value last read back from the robot.
export function parseWalkingNumber(raw, fallback) {
  if (typeof raw === "number") return Number.isFinite(raw) ? raw : fallback;
  if (raw === null || raw === undefined) return fallback;
  const text = String(raw).trim().replace(",", ".");
  if (text === "" || text === "." || text === "-" || text === "-.") return fallback;
  const numeric = Number(text);
  return Number.isFinite(numeric) ? numeric : fallback;
}

export const WALKING_BOOL_FIELDS = new Set(["move_aim_on", "balance_enable"]);
export const WALKING_INT_FIELDS = new Set(["p_gain", "i_gain", "d_gain"]);
export const WALKING_KEYS = [
  "init_x_offset", "init_y_offset", "init_z_offset", "init_roll_offset",
  "init_pitch_offset", "init_yaw_offset", "period_time", "dsp_ratio",
  "step_fb_ratio", "soft_start_time", "x_move_amplitude", "y_move_amplitude",
  "z_move_amplitude", "angle_move_amplitude", "angle_move_amplitude_turn_in",
  "move_aim_on", "balance_enable", "balance_hip_roll_gain", "balance_knee_gain",
  "balance_ankle_roll_gain", "balance_ankle_pitch_gain", "y_swap_amplitude",
  "z_swap_amplitude", "arm_swing_gain", "pelvis_offset", "hip_pitch_offset",
  "p_gain", "i_gain", "d_gain",
];

// WalkingParam.msg memakai float32; JavaScript memakai float64. Jadi nilai yang
// DIKIRIM sebagai 0.028 KEMBALI sebagai 0.02800000086426735 -- bukan angka yang
// berbeda, cuma double terdekat dari float32 yang sama.
//
// Itu tidak berbahaya sampai angka tersebut ditulis kembali ke kotak isian, dan
// applyWalkingDraft() memang melakukannya (ia mengembalikan hasil BACA ULANG,
// bukan draft operator). Akibatnya tiap kali Apply ditekan, 16 dari 29 field
// berubah jadi angka 17 digit di layar dan terlihat seperti setelan yang
// berubah sendiri: -0.03 jadi -0.029999999329447746, 0.77 jadi 0.7699999809265137.
//
// Dibulatkan ke desimal TERPENDEK yang masih memetakan ke float32 yang sama.
// Tidak ada satu bit pun nilai robot yang berubah -- yang dirapikan hanya
// penulisannya kembali ke double.
export function snapFloat32(value) {
  const numeric = Number(value);
  if (!Number.isFinite(numeric)) return value;
  const target = Math.fround(numeric);
  for (let digits = 1; digits <= 9; digits += 1) {
    const candidate = Number(numeric.toPrecision(digits));
    if (Math.fround(candidate) === target) return candidate;
  }
  return numeric;
}

// Dipakai pada apa pun yang DIBACA dari walking_module, bukan pada draft
// operator: yang diketik operator sudah rapi, dan membulatkannya justru bisa
// mengubah maksudnya.
export function snapWalkingRuntime(params) {
  const snapped = { ...params };
  for (const key of WALKING_KEYS) {
    if (WALKING_BOOL_FIELDS.has(key) || WALKING_INT_FIELDS.has(key)) continue;
    snapped[key] = snapFloat32(snapped[key]);
  }
  return snapped;
}

export function parameterValuesEqual(left, right) {
  if (typeof left === "boolean" || typeof right === "boolean") return left === right;
  const leftNumber = parseWalkingNumber(left, NaN);
  const rightNumber = parseWalkingNumber(right, NaN);
  return Number.isFinite(leftNumber) && Number.isFinite(rightNumber)
    && Math.abs(leftNumber - rightNumber) <= 1e-7 * Math.max(1, Math.abs(leftNumber), Math.abs(rightNumber));
}

export function parametersEqual(left, right, keys) {
  return Boolean(left && right) && keys.every((key) => parameterValuesEqual(left[key], right[key]));
}

export function validateWalkingParams(params) {
  const payload = {};
  for (const key of WALKING_KEYS) {
    if (WALKING_BOOL_FIELDS.has(key)) {
      if (typeof params?.[key] !== "boolean") throw new Error(`${key}: nilai boolean belum terbaca dari robot`);
      payload[key] = params[key];
      continue;
    }
    const value = parseWalkingNumber(params?.[key], NaN);
    if (!Number.isFinite(value) || !Number.isFinite(Math.fround(value))) {
      throw new Error(`${key}: isi angka yang lengkap dan berhingga`);
    }
    if (WALKING_INT_FIELDS.has(key) && (!Number.isInteger(value) || value < 0 || value > 2147483647)) {
      throw new Error(`${key}: harus bilangan bulat nonnegatif dalam rentang int32`);
    }
    payload[key] = value;
  }
  if (!(payload.period_time > 0)) throw new Error("period_time: harus lebih besar dari 0 detik");
  if (payload.dsp_ratio < 0 || payload.dsp_ratio >= 1) throw new Error("dsp_ratio: harus 0 ≤ nilai < 1");
  if (payload.step_fb_ratio < 0 || payload.step_fb_ratio > 1) throw new Error("step_fb_ratio: harus antara 0 dan 1");
  if (payload.soft_start_time < 0) throw new Error("soft_start_time: tidak boleh negatif");
  if (payload.z_move_amplitude < 0) throw new Error("z_move_amplitude: tinggi langkah tidak boleh negatif");
  return payload;
}

// Apply mengirim SELURUH isi kotak isian, bukan cuma field yang berbeda dari
// baseline.
//
// Versi sebelumnya memulai payload dari nilai ROBOT lalu hanya menimpa field
// yang berbeda dari baseline. Niatnya baik -- "jangan sentuh yang tidak diedit,
// supaya penulis lain tidak terinjak" -- tapi salah begitu nilai di robot
// BERGESER SENDIRI:
//
//   walking_module menolkan x/y/angle_move_amplitude tiap kali Stop. Sesudah
//   itu draft (0,016) SAMA dengan baseline (0,016), jadi field itu dianggap
//   "tidak diedit" dan tidak ikut dikirim -- yang dikirim justru nilai robot
//   yang sudah 0. Di layar tertulis 0,016, robot berjalan dengan 0.
//
// Terukur di robot 2026-09-08: get_params melaporkan seluruh setelan JALAN
// benar KECUALI x_move_amplitude = 0, dan mengirim struct yang sama secara utuh
// lewat ros2 topic pub langsung membuatnya 0,016. Jadi rantai ke robot sehat;
// yang keliru cara paketnya disusun di sini.
//
// baseline dan current tetap divalidasi: pemanggil memakai keduanya untuk
// memutuskan perlu-tidaknya menerbitkan dan untuk memeriksa hasil bacaan balik.
export function mergeWalkingEdits(draft, baseline, current) {
  validateWalkingParams(baseline);
  validateWalkingParams(current);
  return validateWalkingParams(draft);
}

// Panjang tautan kaki OP3 (op3_kinematics_dynamics): paha + betis. Pergelangan
// TIDAK ikut -- ia di luar rantai dua-tautan yang dibatasi acos() di IK lutut.
export const LEG_REACH_M = 0.11015 + 0.110;   // 220,15 mm

// Sejauh apa pinggul harus menjangkau pergelangan pada init pose ini, sudah
// termasuk satu langkah penuh maju maupun mundur. Rumusnya sengaja sama persis
// dengan terjangkau() di scripts/sync_preset_from_studio.py.
export function legReachNeeded(params) {
  const drop = Math.abs(params.init_z_offset - LEG_REACH_M);
  let farthest = 0;
  for (const sx of [0, params.x_move_amplitude, -params.x_move_amplitude]) {
    for (const sy of [0, params.y_move_amplitude / 2, -params.y_move_amplitude / 2]) {
      farthest = Math.max(farthest, Math.hypot(params.init_x_offset + sx,
        Math.hypot(sy - params.init_y_offset / 2, drop)));
    }
  }
  return farthest;
}

// TERJADI DI ROBOT 2026-09-09. Versi tersimpan "JALAN MENUJU MEKKAH" berisi
// init_x_offset 0,3 m -- kaki diminta menjangkau ~300 mm padahal paha + betis
// cuma 220,2 mm. IK gagal SETIAP siklus, walking_module melewati seluruh siklus
// gait itu dan kaki menahan goal terakhir. Di layar Studio Apply & Start
// tampak sukses dan robot diam total tanpa satu pun petunjuk, sementara log
// manager membanjir "IK KAKI KANAN gagal".
//
// DIPERIKSA HANYA PADA DRAFT YANG AKAN DIKIRIM, bukan di validateWalkingParams.
// Itu penting: nilai yang DIBACA dari robot juga lewat validasi, dan saat robot
// sedang memegang setelan mustahil (persis keadaan tadi) penjaga di validator
// akan menolak pembacaan itu juga -- operator jadi tidak bisa Load, tidak bisa
// Apply, dan tidak punya jalan keluar sama sekali.
//
// Toleransi 0,1 mm supaya preset yang duduk PERSIS di batas tidak tertolak
// gara-gara pembulatan float.
export function checkLegReach(params) {
  const needed = legReachNeeded(params);
  if (needed <= LEG_REACH_M + 0.0001) return;
  const mm = (v) => (v * 1000).toFixed(1);
  throw new Error(
    `Kaki tidak sampai: init pose ini menuntut ${mm(needed)} mm, jangkauan paha+betis `
    + `cuma ${mm(LEG_REACH_M)} mm, jadi IK gagal tiap siklus dan robot akan diam total. `
    + `Periksa init_x_offset (${params.init_x_offset} m) dan init_z_offset `
    + `(${params.init_z_offset} m). Kesalahan paling sering: kelebihan satu angka nol, `
    + `mis. -0.18 padahal maksudnya -0.018.`);
}

export async function applyWalkingDraft({ draft, baseline, read, publish, check, onRead, wait }) {
  checkLegReach(validateWalkingParams(draft));
  validateWalkingParams(baseline);
  check();
  const current = await read();
  check();
  const payload = mergeWalkingEdits(draft, baseline, current);
  onRead(current);
  if (parametersEqual(payload, current, WALKING_KEYS)) return current;
  publish(payload);
  for (let attempt = 0; attempt < 4; attempt += 1) {
    await wait(200);
    check();
    const observed = await read();
    check();
    onRead(observed);
    if (parametersEqual(payload, observed, WALKING_KEYS)) return observed;
  }
  throw new Error("Apply belum terkonfirmasi. Runtime berbeda; periksa demo/teleop yang juga menulis walking.");
}

// Enam field yang MASUK perhitungan walking_bias_ (computeNeutralLegAngle di
// op3_walking_module.cpp). init_y_offset sengaja TIDAK ikut: penangkapan bias
// memanggilnya dengan with_y_offset = false, jadi y tidak pernah masuk acuan.
export const WALKING_BIAS_FIELDS = [
  "init_x_offset", "init_z_offset", "init_roll_offset",
  "init_pitch_offset", "init_yaw_offset", "hip_pitch_offset",
];

// Seberapa jauh offset boleh dari acuan page 2 sebelum operator diperingatkan.
// Longgar sengaja: ini penjaga supaya robot tidak tiba-tiba condong jauh, bukan
// batas tuning. 0,09 rad ~ 5 derajat.
const WALKING_POSE_WARN = {
  init_x_offset: 0.02, init_z_offset: 0.02, init_roll_offset: 0.09,
  init_pitch_offset: 0.09, init_yaw_offset: 0.09, hip_pitch_offset: 0.09,
};

// Offset mana saja yang jauh dari acuan page 2.
//
// Sebelum acuannya dipatok, angka-angka ini tidak pernah benar-benar terasa di
// awal sesi (bias ditangkap dengan nilai yang sama, jadi saling meniadakan),
// sehingga operator terus menaikkannya tiap sesi. Begitu acuannya tetap, angka
// yang sama menggeser pose SEPENUHNYA -- jadi nilai lama yang sudah terlanjur
// besar perlu diperingatkan sekali sebelum dikirim.
export function walkingPoseFarFromReference(params, reference) {
  const jauh = [];
  for (const key of WALKING_BIAS_FIELDS) {
    const acuan = reference?.[key];
    if (typeof acuan !== "number") continue;
    const nilai = parseWalkingNumber(params?.[key], NaN);
    if (!Number.isFinite(nilai)) continue;
    const selisih = nilai - acuan;
    if (Math.abs(selisih) > WALKING_POSE_WARN[key]) jauh.push({ key, selisih, nilai, acuan });
  }
  return jauh;
}
