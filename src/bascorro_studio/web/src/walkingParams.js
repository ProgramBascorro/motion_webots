// Parameter walking dipakai bersama oleh App.jsx (tab Walking) dan
// TuningPage.jsx (kartu Walking Tuner). Keduanya dulu punya parser sendiri:
// TuningPage memakai Number() mentah, dan Number("") === 0, jadi field yang
// dikosongkan untuk diketik ulang terkirim ke robot sebagai 0.
//
// Nilai default di bawah HARUS sama dengan
// src/ROBOTIS-OP3/op3_walking_module/config/param.yaml milik robot INI
// (CHRONUS) -- itu yang benar-benar dimuat robot saat boot. Kalau berbeda,
// studio menampilkan satu angka sementara robot memakai angka lain, dan Apply
// pertama diam-diam MENGUBAH gait robot. Ubah satu, ubah ketiganya:
// param.yaml, fallback WalkingModule::initialize(), dan berkas ini.
//
// Konversi satuan yaml -> pesan WalkingParam (lihat loadWalkingParam()):
//   period_time        ms      -> detik   (x 0.001)
//   *_offset           derajat -> radian  (x pi/180)
//   foot_height                -> z_move_amplitude
//   swing_right_left           -> y_swap_amplitude
//   swing_top_down             -> z_swap_amplitude
//   x/y/z_offset               -> init_x/y/z_offset
export const WALKING_DEFAULT_PARAMS = {
  init_x_offset: -0.015,        // param.yaml x_offset
  init_y_offset: 0.015,         // param.yaml y_offset
  init_z_offset: 0.075,         // param.yaml z_offset
  init_roll_offset: 0,          // param.yaml roll_offset 0 deg
  init_pitch_offset: 0.069813,  // param.yaml pitch_offset 4 deg
  init_yaw_offset: 0,           // param.yaml yaw_offset 0 deg
  period_time: 0.7,             // param.yaml period_time 700 ms
  dsp_ratio: 0.3,
  step_fb_ratio: 0.25,          // param.yaml step_forward_back_ratio
  x_move_amplitude: 0,          // tidak ada di param.yaml (perintah runtime)
  y_move_amplitude: 0,
  z_move_amplitude: 0.05,       // param.yaml foot_height
  angle_move_amplitude: 0,
  move_aim_on: false,
  balance_enable: true,         // tidak dibaca loadWalkingParam(), selalu true saat boot
  balance_hip_roll_gain: 0.35,
  balance_knee_gain: 0.4,
  balance_ankle_roll_gain: 0.7,
  balance_ankle_pitch_gain: 0.9,
  y_swap_amplitude: 0.002,      // param.yaml swing_right_left
  z_swap_amplitude: 0.006,      // param.yaml swing_top_down
  arm_swing_gain: 0.2,
  pelvis_offset: 0.008727,      // param.yaml pelvis_offset 0.5 deg
  hip_pitch_offset: 0.139626,   // param.yaml hip_pitch_offset 8 deg
  p_gain: 0,
  i_gain: 0,
  d_gain: 0,
};

export const WALKING_INT_FIELDS = new Set(["p_gain", "i_gain", "d_gain"]);
export const WALKING_BOOL_FIELDS = new Set(["move_aim_on", "balance_enable"]);

// Parse a walking field value robustly. Accepts both "0.014" and the
// Indonesian decimal form "0,014". Blank/partial input ("", " ", ".", "-")
// falls back to `fallback` instead of 0 -- Number("") === 0 was silently
// turning cleared or comma-typed fields into 0, which is why edited walking
// values reset to 0 after Save/Load/Apply.
export function parseWalkingNumber(raw, fallback) {
  if (typeof raw === "number") return Number.isFinite(raw) ? raw : fallback;
  if (raw === null || raw === undefined) return fallback;
  const text = String(raw).trim().replace(",", ".");
  if (text === "" || text === "." || text === "-" || text === "-.") return fallback;
  const numeric = Number(text);
  return Number.isFinite(numeric) ? numeric : fallback;
}

export function normalizeWalkingParams(params = {}) {
  const next = { ...WALKING_DEFAULT_PARAMS, ...params };
  Object.keys(WALKING_DEFAULT_PARAMS).forEach((key) => {
    if (WALKING_BOOL_FIELDS.has(key)) {
      next[key] = Boolean(next[key]);
      return;
    }
    const numeric = parseWalkingNumber(next[key], WALKING_DEFAULT_PARAMS[key]);
    next[key] = WALKING_INT_FIELDS.has(key) ? Math.round(numeric) : numeric;
  });
  return next;
}
