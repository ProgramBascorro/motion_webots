import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import ROSLIB from "roslib";
import {
  Activity,
  AlertCircle,
  Battery,
  Camera,
  Cpu,
  HardDrive,
  LayoutDashboard,
  Menu,
  Settings,
  SlidersHorizontal,
  StopCircle,
  Terminal,
  Video,
  Wifi,
  X,
  ScrollText,
  BicepsFlexed,
  Footprints,
  Gamepad2,
  Info,
  Play,
  RefreshCw,
  Save,
  Send,
  Square,
  Trash2
} from "lucide-react";
import ActionEditor from "./ActionEditor.jsx";
import GamepadVisualizer from "./GamepadVisualizer.jsx";
import ChartPage from "./ChartPage.jsx";
import TuningPage from "./TuningPage.jsx";
import WalkingSimPreview from "./WalkingSimPreview.jsx";
import {
  computeDefaultRosbridgeUrl,
  normalizeRosbridgeUrl,
  rewriteLoopbackToCurrentHost,
} from "./net/rosbridge.js";

const SHARED_ROS_URL_KEY = "bascorro.shared_ros_url.v1";
const LEGACY_ACTION_ROS_URL_KEY = "op3RosUrl";
const WALKING_VERSION_STORAGE_KEY = "bascorro.walking_versions.v1";
// Seeded once, then never again: if the version is deleted on purpose it must
// stay deleted, so the flag records that the seed already ran rather than
// checking whether the version is currently present.
// Bumped to v2 on 2026-08-11: the preset's init_* offsets were re-derived from
// action page 2, so a browser holding the v1 copy would keep serving the old
// stance. Bumping re-runs the seed once, which refreshes the preset in place.
const WALKING_SEED_FLAG_KEY = "bascorro.walking_versions.seeded.dari_claude.v2";
const TELEOP_COMMAND_TOPIC = "/op3_joy_teleop/command";
const TELEOP_STATUS_TOPIC = "/op3_joy_teleop/status";
const DEFAULT_OVERLAY_TOPIC =
  import.meta.env.VITE_OVERLAY_TOPIC || "/vision/yolo/debug";

// INIT_BARU is action page 2 (user's calibrated standing pose).
const INIT_BARU_PAGE_NUM = 2;
// Time for op3_manager to switch the controller to action_module before
// publishing the page command (the page_num subscriber needs the swap).
const ACTION_MODULE_SETTLE_MS = 600;
// Time for action page 2 (INIT_BARU) to finish playing on the hardware
// (~1.2 s on this robot; buffer for trajectory tail).
const INIT_BARU_PLAY_MS = 1800;
// Time for op3_manager to load the walking_module control_module before we
// publish "start"; otherwise the start command is dropped silently.
const WALKING_ENABLE_SETTLE_MS = 1200;
// robotis_controller merges sync_write_item per port per cycle — sending two
// items in the same ~8 ms tick silently drops the second. Stagger them.
const SYNC_ITEM_GAP_MS = 60;
const TORQUE_SPEED_PRESETS = [
  { key: "slow",   label: "Lambat", velocity: 15, accel: 6,  restoreMs: 5000 },
  { key: "medium", label: "Sedang", velocity: 35, accel: 12, restoreMs: 3000 },
  { key: "fast",   label: "Cepat",  velocity: 70, accel: 25, restoreMs: 2000 },
];
const DEFAULT_TORQUE_SPEED_KEY = "medium";

const PARAM_TYPES = {
  bool: 1,
  int: 2,
  double: 3,
  string: 4,
};

const YOLO_PARAM_KEYS = [
  "ball_confidence_threshold",
  "goalpost_confidence_threshold",
  "robot_confidence_threshold",
];

// Defaults match the FK-derived INIT_BARU geometry + the proven OP3 gait
// shape (op3_walking_module/config/param.yaml). init_x/y/z_offset come
// from forward kinematics on the calibrated INIT_BARU page 2 joint values
// (see scratchpad init_baru_fk.py): foot at (0.003, ±0.032, -0.181) m
// relative to hip → z_offset = leg_length(0.2195) + foot_z = 0.038.
const WALKING_DEFAULT_PARAMS = {
  // FK-derived from action page 2 (WALKING_READY) -- keep in step with
  // op3_walking_module/config/param.yaml. Regenerate both with
  // scripts/init_baru_fk.py --yaml <page 2 export> when the page is re-taught.
  init_x_offset: 0.0046,
  init_y_offset: 0.0205,
  init_z_offset: 0.0408,
  init_roll_offset: 0,
  init_pitch_offset: 0.0698,
  init_yaw_offset: 0,
  period_time: 0.75,
  dsp_ratio: 0.35,
  step_fb_ratio: 0.25,
  x_move_amplitude: 0,
  y_move_amplitude: 0,
  z_move_amplitude: 0.04,
  angle_move_amplitude: 0,
  move_aim_on: false,
  balance_enable: true,
  balance_hip_roll_gain: 0.35,
  balance_knee_gain: 0.4,
  balance_ankle_roll_gain: 0.7,
  balance_ankle_pitch_gain: 0.9,
  y_swap_amplitude: 0.020,
  z_swap_amplitude: 0.006,
  arm_swing_gain: 1.5,
  pelvis_offset: 0.0524,
  hip_pitch_offset: 0.1396,
  p_gain: 32,
  i_gain: 0,
  d_gain: 0,
};

const WALKING_PARAM_HELP = {
  init_x_offset: {
    artinya: "Offset posisi badan maju-mundur saat robot masuk posture walking.",
    fungsi: "Menggeser titik awal badan terhadap kaki supaya center of mass tidak terlalu maju atau mundur.",
    tuning: "Ubah kecil-kecil sekitar 0.001 m. Jika robot cenderung jatuh ke depan, coba lebih negatif; jika ke belakang, coba lebih positif.",
    risk: "Terlalu besar bisa bikin lutut/ankle bekerja keras dan robot langsung condong."
  },
  init_y_offset: {
    artinya: "Offset posisi badan kiri-kanan saat walking.",
    fungsi: "Memberi bias berat badan ke salah satu sisi untuk kompensasi mekanik atau offset servo.",
    tuning: "Pakai untuk koreksi robot yang selalu miring ke kiri/kanan. Naikkan atau turunkan 0.001 m per test.",
    risk: "Bias terlalu jauh bikin satu kaki lebih berat dan langkah jadi pincang."
  },
  init_z_offset: {
    artinya: "Tinggi badan dasar saat gait walking dihitung.",
    fungsi: "Menentukan seberapa jongkok/tinggi postur robot ketika berjalan.",
    tuning: "Lebih rendah biasanya lebih stabil tapi servo lebih berat. Lebih tinggi terasa ringan tapi mudah goyang.",
    risk: "Terlalu rendah bisa membebani lutut; terlalu tinggi bisa membuat kaki kehilangan clearance."
  },
  init_roll_offset: {
    artinya: "Offset sudut roll badan pada posture awal walking.",
    fungsi: "Memiringkan badan sedikit ke kiri/kanan untuk kompensasi mounting atau offset mekanik.",
    tuning: "Biarkan 0 kecuali robot selalu miring. Ubah sangat kecil, misalnya 0.001 rad.",
    risk: "Roll offset salah arah bisa memperparah jatuh samping."
  },
  init_pitch_offset: {
    artinya: "Offset sudut pitch badan pada posture awal walking.",
    fungsi: "Mengatur bias condong depan-belakang badan sebelum langkah berjalan.",
    tuning: "Jika jatuh ke depan, kurangi sedikit. Jika jatuh ke belakang, tambah sedikit.",
    risk: "Pitch sangat sensitif; perubahan besar bisa langsung membuat robot tersungkur."
  },
  init_yaw_offset: {
    artinya: "Offset sudut yaw badan pada posture awal walking.",
    fungsi: "Memberi bias putaran badan terhadap kaki, biasanya untuk kompensasi mekanik.",
    tuning: "Biasanya tetap 0. Ubah hanya jika robot punya twist tetap saat berdiri/jalan.",
    risk: "Yaw offset dapat membuat langkah tidak simetris."
  },
  period_time: {
    artinya: "Durasi satu siklus langkah dalam detik.",
    fungsi: "Mengatur cepat-lambat gait. Nilai besar berarti langkah lebih lambat dan biasanya lebih aman.",
    tuning: "Mulai dari 0.85 untuk test aman, lalu turun perlahan ke 0.78 atau 0.70 kalau sudah stabil.",
    risk: "Terlalu kecil membuat langkah agresif dan robot mudah jatuh."
  },
  dsp_ratio: {
    artinya: "Rasio double support phase, yaitu fase dua kaki sama-sama menyentuh lantai.",
    fungsi: "Menentukan berapa lama robot berada di fase paling stabil dalam setiap langkah.",
    tuning: "Naikkan untuk stabilitas, turunkan untuk langkah lebih dinamis. Range aman awal sekitar 0.30-0.35.",
    risk: "Terlalu kecil bisa kehilangan balance; terlalu besar membuat jalan kaku."
  },
  step_fb_ratio: {
    artinya: "Rasio timing gerakan kaki maju-mundur di dalam siklus langkah.",
    fungsi: "Mengatur pembagian fase ayunan kaki forward/backward.",
    tuning: "Biasanya jangan disentuh dulu. Pakai default 0.25 sampai gait dasar sudah stabil.",
    risk: "Timing yang salah bisa membuat kaki nyeret atau hentakan langkah."
  },
  x_move_amplitude: {
    artinya: "Besar langkah maju-mundur per siklus.",
    fungsi: "Ini command utama untuk robot bergerak maju atau mundur.",
    tuning: "Test awal gunakan 0.003 sampai 0.005 m. Naikkan pelan setelah robot stabil.",
    risk: "Nilai terlalu besar adalah penyebab paling umum robot jatuh saat mulai jalan."
  },
  y_move_amplitude: {
    artinya: "Besar langkah geser kiri-kanan.",
    fungsi: "Dipakai untuk strafing atau koreksi lateral.",
    tuning: "Untuk test awal biarkan 0. Pakai nilai kecil jika perlu geser samping.",
    risk: "Gerak lateral lebih sulit dari maju; jangan agresif di robot asli."
  },
  z_move_amplitude: {
    artinya: "Tinggi kaki diangkat saat melangkah, sering disebut foot height.",
    fungsi: "Membantu kaki tidak nyeret lantai atau rumput.",
    tuning: "Jika kaki nyeret, naikkan sedikit. Jika badan goyang, turunkan sedikit.",
    risk: "Kaki terlalu tinggi membuat robot memantul; terlalu rendah membuat ujung kaki tersangkut."
  },
  angle_move_amplitude: {
    artinya: "Besar putaran yaw per langkah.",
    fungsi: "Dipakai untuk robot belok kiri/kanan saat walking.",
    tuning: "Mulai kecil, misalnya 0.05 rad. Naikkan hanya kalau belok terlalu lambat.",
    risk: "Turn besar bisa membuat kaki silang dan balance hilang."
  },
  move_aim_on: {
    artinya: "Flag mode aim/move pada message walking.",
    fungsi: "Disediakan oleh WalkingParam untuk mode gerak tertentu, tapi pada setup ini biasanya tidak perlu aktif.",
    tuning: "Biarkan off kecuali kamu tahu module yang dipakai membutuhkan flag ini.",
    risk: "Mengaktifkan tanpa kebutuhan bisa membuat perilaku sulit dibaca saat tuning."
  },
  balance_enable: {
    artinya: "Mengaktifkan koreksi balance dari feedback gyro/IMU.",
    fungsi: "Walking module memakai gyro untuk koreksi hip, knee, dan ankle saat robot goyang.",
    tuning: "Untuk robot asli biasanya on. Untuk membandingkan efek gain, boleh off sebentar sambil robot dipegang.",
    risk: "Balance off di hardware bisa membuat robot lebih mudah jatuh."
  },
  balance_hip_roll_gain: {
    artinya: "Gain koreksi roll di joint hip.",
    fungsi: "Membantu mengoreksi miring kiri-kanan lewat panggul.",
    tuning: "Naikkan sedikit jika robot lambat melawan miring samping. Turunkan jika pinggul bergetar.",
    risk: "Gain terlalu besar bisa membuat osilasi kiri-kanan."
  },
  balance_knee_gain: {
    artinya: "Gain koreksi pitch lewat lutut.",
    fungsi: "Membantu robot menahan goyangan depan-belakang dengan bending knee.",
    tuning: "Naikkan sedikit jika koreksi depan-belakang kurang. Turunkan jika lutut terlihat pumping.",
    risk: "Terlalu besar bisa membuat lutut kerja keras dan jalan memantul."
  },
  balance_ankle_roll_gain: {
    artinya: "Gain koreksi roll di ankle.",
    fungsi: "Mengoreksi miring kiri-kanan langsung dari pergelangan kaki.",
    tuning: "Efektif untuk jatuh samping. Ubah 0.05-0.10 per test.",
    risk: "Gain besar bisa membuat ankle bergetar dan telapak tidak stabil."
  },
  balance_ankle_pitch_gain: {
    artinya: "Gain koreksi pitch di ankle.",
    fungsi: "Mengoreksi condong depan-belakang lewat pergelangan kaki.",
    tuning: "Jika robot jatuh pelan ke depan/belakang, ini parameter penting untuk dicoba.",
    risk: "Terlalu besar bisa membuat robot seperti menendang lantai saat koreksi."
  },
  y_swap_amplitude: {
    artinya: "Amplitudo sway badan kiri-kanan saat pindah berat badan.",
    fungsi: "Membantu robot memindahkan center of mass ke kaki tumpuan.",
    tuning: "Jika kaki sulit terangkat atau berat tidak pindah, naikkan sedikit.",
    risk: "Sway terlalu besar membuat robot bergoyang samping berlebihan."
  },
  z_swap_amplitude: {
    artinya: "Amplitudo naik-turun badan selama walking.",
    fungsi: "Memberi ritme vertikal agar langkah lebih natural dan kaki punya clearance.",
    tuning: "Turunkan jika robot memantul. Naikkan sedikit jika kaki kurang bebas.",
    risk: "Terlalu besar meningkatkan hentakan dan beban servo."
  },
  arm_swing_gain: {
    artinya: "Gain ayunan tangan saat berjalan.",
    fungsi: "Ayunan tangan membantu counterbalance terhadap gerakan kaki.",
    tuning: "Naikkan jika badan terlalu kaku. Turunkan jika ayunan tangan justru mengganggu balance.",
    risk: "Ayunan besar bisa mengganggu vision/kamera dan menambah goyangan."
  },
  pelvis_offset: {
    artinya: "Offset tetap pada orientasi pelvis saat walking.",
    fungsi: "Dipakai buat bias postur panggul supaya gait lebih cocok dengan mekanik robot.",
    tuning: "Ubah kecil-kecil karena efeknya terasa ke pinggul dan kaki. Default repo sekitar 0.5 derajat atau 0.0087 rad.",
    risk: "Pelvis offset terlalu besar bisa membuat langkah asimetris."
  },
  hip_pitch_offset: {
    artinya: "Offset sudut hip pitch saat walking.",
    fungsi: "Mengatur bias kaki/pinggul ke depan-belakang agar posture walking cocok dengan OP3.",
    tuning: "Sangat berpengaruh. Ubah sedikit, misalnya 0.005 rad per test.",
    risk: "Salah tuning bisa membuat robot terlalu membungkuk atau jatuh ke belakang."
  },
  p_gain: {
    artinya: "Field gain P integer yang ikut dibawa message WalkingParam.",
    fungsi: "Disiapkan untuk gain kontrol tambahan, tapi setup repo saat ini default 0.",
    tuning: "Biarkan 0 kecuali kamu sudah memastikan module memakai field ini.",
    risk: "Mengubah tanpa kebutuhan bisa bikin hasil test membingungkan."
  },
  i_gain: {
    artinya: "Field gain I integer yang ikut dibawa message WalkingParam.",
    fungsi: "Disiapkan untuk komponen integral kontrol tambahan, default repo 0.",
    tuning: "Biarkan 0 untuk tuning gait dasar.",
    risk: "Integral yang tidak tepat bisa menumpuk error dan memicu gerakan aneh."
  },
  d_gain: {
    artinya: "Field gain D integer yang ikut dibawa message WalkingParam.",
    fungsi: "Disiapkan untuk damping tambahan, default repo 0.",
    tuning: "Biarkan 0 kecuali ada alasan jelas untuk tuning kontrol level bawah.",
    risk: "D gain salah bisa membuat respons terlalu kasar atau noise-sensitive."
  },
};

const WALKING_PARAM_GROUPS = [
  {
    title: "Init Pose",
    description: "Body pose offsets used by the gait generator.",
    fields: [
      { key: "init_x_offset", label: "X offset", unit: "m", step: 0.001, ...WALKING_PARAM_HELP.init_x_offset },
      { key: "init_y_offset", label: "Y offset", unit: "m", step: 0.001, ...WALKING_PARAM_HELP.init_y_offset },
      { key: "init_z_offset", label: "Z offset", unit: "m", step: 0.001, ...WALKING_PARAM_HELP.init_z_offset },
      { key: "init_roll_offset", label: "Roll offset", unit: "rad", step: 0.001, ...WALKING_PARAM_HELP.init_roll_offset },
      { key: "init_pitch_offset", label: "Pitch offset", unit: "rad", step: 0.001, ...WALKING_PARAM_HELP.init_pitch_offset },
      { key: "init_yaw_offset", label: "Yaw offset", unit: "rad", step: 0.001, ...WALKING_PARAM_HELP.init_yaw_offset },
    ],
  },
  {
    title: "Timing",
    description: "Step cadence and support timing. Runtime units use seconds.",
    fields: [
      { key: "period_time", label: "Period time", unit: "s", step: 0.01, ...WALKING_PARAM_HELP.period_time },
      { key: "dsp_ratio", label: "DSP ratio", unit: "ratio", step: 0.01, ...WALKING_PARAM_HELP.dsp_ratio },
      { key: "step_fb_ratio", label: "Step FB ratio", unit: "ratio", step: 0.01, ...WALKING_PARAM_HELP.step_fb_ratio },
    ],
  },
  {
    title: "Movement",
    description: "Forward, lateral, lift, and turn amplitudes.",
    fields: [
      { key: "x_move_amplitude", label: "X move amplitude", unit: "m", step: 0.001, ...WALKING_PARAM_HELP.x_move_amplitude },
      { key: "y_move_amplitude", label: "Y move amplitude", unit: "m", step: 0.001, ...WALKING_PARAM_HELP.y_move_amplitude },
      { key: "z_move_amplitude", label: "Z move amplitude / foot height", unit: "m", step: 0.001, ...WALKING_PARAM_HELP.z_move_amplitude },
      { key: "angle_move_amplitude", label: "Angle move amplitude", unit: "rad", step: 0.005, ...WALKING_PARAM_HELP.angle_move_amplitude },
      { key: "move_aim_on", label: "Move aim on", type: "bool", ...WALKING_PARAM_HELP.move_aim_on },
    ],
  },
  {
    title: "Balance",
    description: "IMU feedback gains and body sway.",
    fields: [
      { key: "balance_enable", label: "Balance enable", type: "bool", ...WALKING_PARAM_HELP.balance_enable },
      { key: "balance_hip_roll_gain", label: "Hip roll gain", unit: "gain", step: 0.01, ...WALKING_PARAM_HELP.balance_hip_roll_gain },
      { key: "balance_knee_gain", label: "Knee gain", unit: "gain", step: 0.01, ...WALKING_PARAM_HELP.balance_knee_gain },
      { key: "balance_ankle_roll_gain", label: "Ankle roll gain", unit: "gain", step: 0.01, ...WALKING_PARAM_HELP.balance_ankle_roll_gain },
      { key: "balance_ankle_pitch_gain", label: "Ankle pitch gain", unit: "gain", step: 0.01, ...WALKING_PARAM_HELP.balance_ankle_pitch_gain },
      { key: "y_swap_amplitude", label: "Y swap amplitude", unit: "m", step: 0.001, ...WALKING_PARAM_HELP.y_swap_amplitude },
      { key: "z_swap_amplitude", label: "Z swap amplitude", unit: "m", step: 0.001, ...WALKING_PARAM_HELP.z_swap_amplitude },
      { key: "arm_swing_gain", label: "Arm swing gain", unit: "gain", step: 0.01, ...WALKING_PARAM_HELP.arm_swing_gain },
      { key: "pelvis_offset", label: "Pelvis offset", unit: "rad", step: 0.001, ...WALKING_PARAM_HELP.pelvis_offset },
      { key: "hip_pitch_offset", label: "Hip pitch offset", unit: "rad", step: 0.001, ...WALKING_PARAM_HELP.hip_pitch_offset },
    ],
  },
  {
    title: "Motor Gains",
    description: "Integer gain fields carried by WalkingParam.",
    fields: [
      { key: "p_gain", label: "P gain", type: "int", step: 1, ...WALKING_PARAM_HELP.p_gain },
      { key: "i_gain", label: "I gain", type: "int", step: 1, ...WALKING_PARAM_HELP.i_gain },
      { key: "d_gain", label: "D gain", type: "int", step: 1, ...WALKING_PARAM_HELP.d_gain },
    ],
  },
];

const WALKING_INT_FIELDS = new Set(["p_gain", "i_gain", "d_gain"]);
const WALKING_BOOL_FIELDS = new Set(["move_aim_on", "balance_enable"]);

function normalizeWalkingParams(params = {}) {
  const next = { ...WALKING_DEFAULT_PARAMS, ...params };
  Object.keys(WALKING_DEFAULT_PARAMS).forEach((key) => {
    if (WALKING_BOOL_FIELDS.has(key)) {
      next[key] = Boolean(next[key]);
      return;
    }
    const numeric = Number(next[key]);
    next[key] = Number.isFinite(numeric)
      ? (WALKING_INT_FIELDS.has(key) ? Math.round(numeric) : numeric)
      : WALKING_DEFAULT_PARAMS[key];
  });
  return next;
}

function formatWalkingValue(value, field) {
  if (field.type === "bool") return value ? "true" : "false";
  if (field.type === "int") return String(Math.round(Number(value) || 0));
  return formatNumber(value, 5);
}

function readWalkingVersions() {
  if (typeof window === "undefined") return [];
  try {
    const raw = localStorage.getItem(WALKING_VERSION_STORAGE_KEY);
    const parsed = JSON.parse(raw || "[]");
    if (!Array.isArray(parsed)) return [];
    return parsed
      .filter((item) => item?.id && item?.params)
      .map((item) => ({
        id: String(item.id),
        name: String(item.name || "Untitled"),
        createdAt: Number(item.createdAt || Date.now()),
        updatedAt: Number(item.updatedAt || item.createdAt || Date.now()),
        params: normalizeWalkingParams(item.params),
      }));
  } catch {
    return [];
  }
}

function makeWalkingVersionId() {
  if (typeof crypto !== "undefined" && typeof crypto.randomUUID === "function") {
    return crypto.randomUUID();
  }
  return `walking-${Date.now()}-${Math.random().toString(16).slice(2)}`;
}

// Conservative starting gait, built on the values that actually run on this
// robot (op3_walking_module/config/param.yaml) rather than on stock OP3 numbers.
// Five deliberate deltas from that baseline: x_move_amplitude 0 -> 0.02 (the
// robot marched in place without it), period_time 0.75 -> 0.80, dsp_ratio
// 0.35 -> 0.40, y_swap_amplitude 0.020 -> 0.025, arm_swing_gain 0 -> 1.0.
// The init_* offsets are left untouched: they are FK-tuned to INIT_BARU
// (action page 2) so the handoff into walking_module stays smooth.
// p/i/d are inert here -- op3_walking_module.cpp never writes them to the servos.
const CLAUDE_WALKING_VERSION_NAME = "dari claude";
const CLAUDE_WALKING_PARAMS = {
  init_x_offset: 0.0046,
  init_y_offset: 0.0205,
  init_z_offset: 0.0408,
  init_roll_offset: 0,
  init_pitch_offset: 0.0698,
  init_yaw_offset: 0,
  hip_pitch_offset: 0.1396,
  pelvis_offset: 0.0524,
  period_time: 0.8,
  dsp_ratio: 0.4,
  step_fb_ratio: 0.25,
  x_move_amplitude: 0.02,
  y_move_amplitude: 0,
  angle_move_amplitude: 0,
  z_move_amplitude: 0.04,
  move_aim_on: false,
  y_swap_amplitude: 0.025,
  z_swap_amplitude: 0.006,
  arm_swing_gain: 1.0,
  balance_enable: true,
  balance_hip_roll_gain: 0.35,
  balance_knee_gain: 0.4,
  balance_ankle_roll_gain: 0.7,
  balance_ankle_pitch_gain: 0.9,
  p_gain: 0,
  i_gain: 0,
  d_gain: 0,
};

// Adds the preset to the saved-version list the first time this browser loads
// the page. Existing versions are never touched -- the seed is only prepended.
//
// Kept pure on purpose: it reads the flag but never writes it, because
// StrictMode invokes useState initializers twice in dev. Writing the flag here
// meant the second call saw its own write and returned the list unseeded. The
// flag is set from an effect instead (see markWalkingSeed below), and the id is
// fixed so both invocations produce the same entry.
const CLAUDE_WALKING_VERSION_ID = "claude-walking-preset-v1";

function seedWalkingVersions(versions) {
  if (typeof window === "undefined") return versions;
  try {
    if (localStorage.getItem(WALKING_SEED_FLAG_KEY)) return versions;
  } catch {
    // Private/restricted storage: skip seeding rather than break the page.
    return versions;
  }
  const now = Date.now();
  const preset = normalizeWalkingParams(CLAUDE_WALKING_PARAMS);

  // An entry from an earlier seed is refreshed in place rather than duplicated.
  // Only this preset is touched; every other saved version is left alone.
  const existing = versions.findIndex((version) => version.name === CLAUDE_WALKING_VERSION_NAME);
  if (existing !== -1) {
    const refreshed = versions.slice();
    refreshed[existing] = { ...versions[existing], params: preset, updatedAt: now };
    return refreshed;
  }

  return [
    {
      id: CLAUDE_WALKING_VERSION_ID,
      name: CLAUDE_WALKING_VERSION_NAME,
      createdAt: now,
      updatedAt: now,
      params: preset,
    },
    ...versions,
  ];
}

// --- Utilities ---

function formatNumber(value, digits = 1) {
  if (value === null || value === undefined || Number.isNaN(value)) {
    return "-";
  }
  return Number(value).toFixed(digits);
}

function formatPercent(value) {
  if (value === null || value === undefined || Number.isNaN(value)) {
    return "-%";
  }
  return `${Math.round(value)}%`;
}

function decodeImage(msg) {
  if (!msg || !msg.data || !msg.width || !msg.height) {
    return null;
  }
  const { width, height, encoding } = msg;
  const bin = atob(msg.data);
  const raw = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i += 1) {
    raw[i] = bin.charCodeAt(i);
  }

  const pixels = new Uint8ClampedArray(width * height * 4);
  if (encoding === "rgb8" || encoding === "bgr8") {
    const isBgr = encoding === "bgr8";
    for (let i = 0; i < width * height; i += 1) {
      const offset = i * 3;
      const r = isBgr ? raw[offset + 2] : raw[offset];
      const g = raw[offset + 1];
      const b = isBgr ? raw[offset] : raw[offset + 2];
      const out = i * 4;
      pixels[out] = r;
      pixels[out + 1] = g;
      pixels[out + 2] = b;
      pixels[out + 3] = 255;
    }
    return new ImageData(pixels, width, height);
  }
  if (encoding === "bgra8") {
    for (let i = 0; i < width * height; i += 1) {
      const offset = i * 4;
      const out = i * 4;
      pixels[out] = raw[offset + 2];     // R
      pixels[out + 1] = raw[offset + 1]; // G
      pixels[out + 2] = raw[offset];     // B
      pixels[out + 3] = raw[offset + 3]; // A
    }
    return new ImageData(pixels, width, height);
  }
  if (encoding === "mono8") {
    for (let i = 0; i < width * height; i += 1) {
      const val = raw[i];
      const out = i * 4;
      pixels[out] = val;
      pixels[out + 1] = val;
      pixels[out + 2] = val;
      pixels[out + 3] = 255;
    }
    return new ImageData(pixels, width, height);
  }
  return null;
}

function makeParamValue(type, rawValue) {
  const value = String(rawValue ?? "").trim();
  if (type === "bool") {
    return { type: PARAM_TYPES.bool, bool_value: value === "true" };
  }
  if (type === "int") {
    return { type: PARAM_TYPES.int, integer_value: Number(value) };
  }
  if (type === "string") {
    return { type: PARAM_TYPES.string, string_value: value };
  }
  return { type: PARAM_TYPES.double, double_value: Number(value) };
}

function parseHealthMessage(message) {
  if (!message) return null;
  try {
    return JSON.parse(message);
  } catch (err) {
    console.error("Failed to parse health response", err);
    return { raw: message };
  }
}

// --- Components ---

const StatCard = ({ title, icon: Icon, value, subValue, status = "neutral" }) => {
  const statusStyles = {
    neutral: "border-gray-200 bg-white",
    danger: "border-red-200 bg-red-50",
    warning: "border-yellow-200 bg-yellow-50",
    success: "border-green-200 bg-green-50",
  };

  return (
    <div className={`p-5 rounded-2xl border shadow-sm transition-all hover:shadow-md ${statusStyles[status]}`}>
      <div className="flex justify-between items-center mb-2 text-gray-500">
        <span className="text-sm font-medium">{title}</span>
        {Icon && <Icon size={18} />}
      </div>
      <div>
        <span className="block text-2xl font-bold font-display text-gray-900">{value}</span>
        {subValue && <span className="text-xs text-gray-500 font-mono">{subValue}</span>}
      </div>
    </div>
  );
};

const SectionHeader = ({ title, children }) => (
  <div className="flex justify-between items-center mb-4">
    <h2 className="text-lg font-bold font-display text-gray-800">{title}</h2>
    <div className="flex gap-2">{children}</div>
  </div>
);

function resolveInitialRosUrl() {
  const envDefault = computeDefaultRosbridgeUrl(import.meta.env.VITE_ROSBRIDGE_URL);
  if (typeof window === "undefined") return envDefault;
  try {
    const shared = localStorage.getItem(SHARED_ROS_URL_KEY);
    if (shared) return rewriteLoopbackToCurrentHost(shared, window.location);
    const legacy = localStorage.getItem(LEGACY_ACTION_ROS_URL_KEY);
    if (legacy) return rewriteLoopbackToCurrentHost(legacy, window.location);
  } catch {
    return envDefault;
  }
  return envDefault;
}

export default function App() {
  const initialRosUrl = useMemo(() => resolveInitialRosUrl(), []);
  const [activeTab, setActiveTab] = useState("dashboard");
  const [sidebarOpen, setSidebarOpen] = useState(false);
  const [rosUrl, setRosUrl] = useState(initialRosUrl);
  const [rosUrlDraft, setRosUrlDraft] = useState(initialRosUrl);
  const [rosState, setRosState] = useState("disconnected");

  // Data State
  const [metrics, setMetrics] = useState(null);
  const [events, setEvents] = useState([]);
  const [studioStatus, setStudioStatus] = useState("");
  const [studioError, setStudioError] = useState(false);
  const [healthResult, setHealthResult] = useState(null);
  const [healthError, setHealthError] = useState("");
  const [healthRunning, setHealthRunning] = useState(false);
  const [healthCheckedAt, setHealthCheckedAt] = useState(null);
  const [demoMode, setDemoMode] = useState("");
  const [demoCommand, setDemoCommand] = useState("");
  const [demoCommandAt, setDemoCommandAt] = useState(null);
  const [walkingParams, setWalkingParams] = useState(() => normalizeWalkingParams());
  const [walkingCurrentParams, setWalkingCurrentParams] = useState(null);
  const [walkingLoaded, setWalkingLoaded] = useState(false);
  const [walkingDirty, setWalkingDirty] = useState(false);
  const [walkingLastAppliedAt, setWalkingLastAppliedAt] = useState(null);
  const [walkingError, setWalkingError] = useState("");
  const [torqueSpeedKey, setTorqueSpeedKey] = useState(DEFAULT_TORQUE_SPEED_KEY);
  const [walkingVersions, setWalkingVersions] = useState(() => seedWalkingVersions(readWalkingVersions()));
  const [walkingVersionName, setWalkingVersionName] = useState("");
  const [activeWalkingVersionId, setActiveWalkingVersionId] = useState("");
  const [teleopStatus, setTeleopStatus] = useState(null);
  const [teleopStatusAt, setTeleopStatusAt] = useState(null);
  
  // Vision
  const [overlayTopic, setOverlayTopic] = useState(DEFAULT_OVERLAY_TOPIC);
  const [showOverlay, setShowOverlay] = useState(true);
  const [overlayStats, setOverlayStats] = useState({ fps: 0, lastFrameMs: null, dropped: 0 });
  const [cameraTopics, setCameraTopics] = useState([]);
  const [cameraTopicsLoading, setCameraTopicsLoading] = useState(false);
  const [cameraTopicsError, setCameraTopicsError] = useState("");
  
  // Tuning - Vision Params
  const [yoloParams, setYoloParams] = useState({
    ball_confidence_threshold: 0.2,
    goalpost_confidence_threshold: 0.5,
    robot_confidence_threshold: 0.2,
  });
  
  // Input
  const [joyState, setJoyState] = useState(null);

  // Build terminal URL dynamically based on current window location
  // This allows accessing from phone/other devices on local network
  const terminalUrl = useMemo(() => {
    // Use current hostname/IP with terminal port
    const protocol = window.location.protocol;
    const hostname = window.location.hostname;
    const terminalPort = import.meta.env.VITE_TERMINAL_PORT || "7681";
    const url = `${protocol}//${hostname}:${terminalPort}`;
    console.log("Auto-detected terminal URL:", url, "from hostname:", hostname);
    return url;
  }, []);

  // Refs
  const rosRef = useRef(null);
  const overlayCanvasRef = useRef(null);
  const overlayLastRef = useRef({ stampMs: null, fps: 0, dropped: 0 });
  const showOverlayRef = useRef(true);
  
  // Topic/Service Refs
  const metricsSubRef = useRef(null);
  const eventsSubRef = useRef(null);
  const overlaySubRef = useRef(null);
  const joySubRef = useRef(null);
  const joyPubRef = useRef(null);
  const teleopCommandPubRef = useRef(null);
  const teleopStatusSubRef = useRef(null);
  const actionPagePubRef = useRef(null);
  const walkingCommandPubRef = useRef(null);
  const walkingParamPubRef = useRef(null);
  const walkingGetServiceRef = useRef(null);
  const torquePubRef = useRef(null);
  const headModulePubRef = useRef(null);
  const walkingModulePubRef = useRef(null);
  const walkingModuleEnabledRef = useRef(false);
  const yoloSetServiceRef = useRef(null);
  const yoloGetServiceRef = useRef(null);
  const snapshotServiceRef = useRef(null);
  const healthCheckServiceRef = useRef(null);
  const demoModePubRef = useRef(null);
  const demoCommandPubRef = useRef(null);

  // --- Effects ---

  const activeTabRef = useRef(activeTab);
  useEffect(() => { activeTabRef.current = activeTab; }, [activeTab]);

  useEffect(() => {
    setRosUrlDraft(rosUrl);
  }, [rosUrl]);

  useEffect(() => {
    try {
      localStorage.setItem(SHARED_ROS_URL_KEY, rosUrl);
      localStorage.removeItem(LEGACY_ACTION_ROS_URL_KEY);
    } catch {
      // Ignore storage failures in restrictive browser contexts.
    }
  }, [rosUrl]);

  useEffect(() => {
    showOverlayRef.current = showOverlay;
  }, [showOverlay]);

  useEffect(() => {
    try {
      localStorage.setItem(WALKING_VERSION_STORAGE_KEY, JSON.stringify(walkingVersions));
    } catch {
      // Browser storage can fail in private or restricted contexts.
    }
  }, [walkingVersions]);

  // Records that the "dari claude" preset has been offered to this browser, so
  // deleting it makes it stay deleted instead of returning on the next reload.
  useEffect(() => {
    try {
      localStorage.setItem(WALKING_SEED_FLAG_KEY, String(Date.now()));
    } catch {
      // Same restricted-storage case as above; the preset simply seeds again.
    }
  }, []);

  useEffect(() => {
    if (!rosUrl) return;

    const ros = new ROSLIB.Ros({ url: rosUrl });
    rosRef.current = ros;

    ros.on("connection", () => setRosState("connected"));
    ros.on("error", () => setRosState("error"));
    ros.on("close", () => setRosState("disconnected"));

    // Subscriptions
    metricsSubRef.current = new ROSLIB.Topic({
      ros,
      name: "/bascorro_studio/metrics",
      messageType: "std_msgs/String",
    });
    metricsSubRef.current.subscribe((msg) => {
      try {
        setMetrics(JSON.parse(msg.data));
      } catch (e) { console.error(e); }
    });

    eventsSubRef.current = new ROSLIB.Topic({
      ros,
      name: "/bascorro_studio/events",
      messageType: "std_msgs/String",
    });
    eventsSubRef.current.subscribe((msg) => {
      try {
        const payload = JSON.parse(msg.data);
        if (Array.isArray(payload.events)) setEvents(payload.events);
      } catch (e) { console.error(e); }
    });

    joySubRef.current = new ROSLIB.Topic({
      ros,
      name: "/joy",
      messageType: "sensor_msgs/Joy",
    });
    joySubRef.current.subscribe(setJoyState);

    teleopStatusSubRef.current = new ROSLIB.Topic({
      ros,
      name: TELEOP_STATUS_TOPIC,
      messageType: "std_msgs/String",
    });
    teleopStatusSubRef.current.subscribe((msg) => {
      try {
        setTeleopStatus(JSON.parse(msg.data));
        setTeleopStatusAt(Date.now());
      } catch (e) {
        setTeleopStatus({ raw: msg.data });
        setTeleopStatusAt(Date.now());
      }
    });

    // Publishers & Services
    joyPubRef.current = new ROSLIB.Topic({
      ros,
      name: "/joy",
      messageType: "sensor_msgs/Joy",
    });
    teleopCommandPubRef.current = new ROSLIB.Topic({
      ros,
      name: TELEOP_COMMAND_TOPIC,
      messageType: "std_msgs/String",
    });
    actionPagePubRef.current = new ROSLIB.Topic({
      ros,
      name: "/robotis/action/page_num",
      messageType: "std_msgs/Int32",
    });
    walkingCommandPubRef.current = new ROSLIB.Topic({
      ros,
      name: "/robotis/walking/command",
      messageType: "std_msgs/String",
    });
    walkingParamPubRef.current = new ROSLIB.Topic({
      ros,
      name: "/robotis/walking/set_params",
      messageType: "op3_walking_module_msgs/WalkingParam",
    });
    torquePubRef.current = new ROSLIB.Topic({
      ros,
      name: "/robotis/sync_write_item",
      messageType: "robotis_controller_msgs/SyncWriteItem",
    });
    headModulePubRef.current = new ROSLIB.Topic({
      ros,
      name: "/robotis/enable_ctrl_module",
      messageType: "std_msgs/String",
    });
    walkingModulePubRef.current = new ROSLIB.Topic({
      ros,
      name: "/robotis/enable_ctrl_module",
      messageType: "std_msgs/String",
    });

    yoloSetServiceRef.current = new ROSLIB.Service({
      ros,
      name: "/op3_yolo_vision/set_parameters",
      serviceType: "rcl_interfaces/srv/SetParameters",
    });
    yoloGetServiceRef.current = new ROSLIB.Service({
      ros,
      name: "/op3_yolo_vision/get_parameters",
      serviceType: "rcl_interfaces/srv/GetParameters",
    });
    snapshotServiceRef.current = new ROSLIB.Service({
      ros,
      name: "/bascorro_studio/snapshot",
      serviceType: "std_srvs/srv/Trigger",
    });
    healthCheckServiceRef.current = new ROSLIB.Service({
      ros,
      name: "/robotis/health_check",
      serviceType: "std_srvs/srv/Trigger",
    });
    walkingGetServiceRef.current = new ROSLIB.Service({
      ros,
      name: "/robotis/walking/get_params",
      serviceType: "op3_walking_module_msgs/srv/GetWalkingParam",
    });
    demoModePubRef.current = new ROSLIB.Topic({
      ros,
      name: "/robotis/mode_command",
      messageType: "std_msgs/String",
    });
    demoCommandPubRef.current = new ROSLIB.Topic({
      ros,
      name: "/robotis/demo_command",
      messageType: "std_msgs/String",
    });

    // Load initial params
    if (yoloGetServiceRef.current) {
        const request = new ROSLIB.ServiceRequest({ names: YOLO_PARAM_KEYS });
        yoloGetServiceRef.current.callService(request, (result) => {
            if (result && Array.isArray(result.values)) {
                const next = { ...yoloParams };
                result.values.forEach((val, i) => {
                    if (val && typeof val.double_value === 'number') {
                        next[YOLO_PARAM_KEYS[i]] = val.double_value;
                    }
                });
                setYoloParams(next);
            }
        });
    }

    return () => {
      metricsSubRef.current?.unsubscribe();
      eventsSubRef.current?.unsubscribe();
      joySubRef.current?.unsubscribe();
      teleopStatusSubRef.current?.unsubscribe();
      ros.close();
    };
  }, [rosUrl]);

  useEffect(() => {
    if (!rosRef.current || rosState === "disconnected") return;
    
    if (overlaySubRef.current) overlaySubRef.current.unsubscribe();
    
    overlaySubRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: overlayTopic,
      messageType: "sensor_msgs/Image",
    });

    overlaySubRef.current.subscribe((msg) => {
      if (!showOverlayRef.current) return;
      if (activeTabRef.current !== "vision") return;

      const imageData = decodeImage(msg);
      if (!imageData || !overlayCanvasRef.current) return;
      
      const canvas = overlayCanvasRef.current;
      if (canvas.width !== imageData.width || canvas.height !== imageData.height) {
        canvas.width = imageData.width;
        canvas.height = imageData.height;
      }
      const ctx = canvas.getContext("2d");
      ctx.putImageData(imageData, 0, 0);

      const now = performance.now();
      const last = overlayLastRef.current.stampMs;
      const fps = last ? 1000 / (now - last) : 0;
      overlayLastRef.current = { stampMs: now, fps, dropped: 0 };
      setOverlayStats(prev => ({ ...prev, fps, lastFrameMs: now }));
    });

    return () => overlaySubRef.current?.unsubscribe();
  }, [overlayTopic, rosState]);

  // Auto-enable head_control_module on the real robot so head torque comes
  // up as soon as the manager is reachable. Without this, INIT_BARU's head
  // channels are ignored because head joints have no owning module yet.
  // (Sim head torque-on is handled by the sim_domain_bridge — the Sim Start
  //  sequence publishes head_control_module via the bridged topic.)
  const headAutoEnabledRef = useRef({ real: false });
  useEffect(() => {
    if (rosState !== "connected") { headAutoEnabledRef.current.real = false; return; }
    if (headAutoEnabledRef.current.real) return;
    // Small delay so the manager has time to advertise the topic; if it
    // isn't up yet the publish is a harmless no-op.
    const t = window.setTimeout(() => {
      if (headModulePubRef.current) {
        headModulePubRef.current.publish(new ROSLIB.Message({ data: "head_control_module" }));
        headAutoEnabledRef.current.real = true;
      }
    }, 2000);
    return () => window.clearTimeout(t);
  }, [rosState]);

  // --- Handlers ---

  const sendStatus = (msg, isError = false) => {
    setStudioStatus(msg);
    setStudioError(isError);
    setTimeout(() => { if(studioStatus === msg) setStudioStatus(""); }, 3000);
  };

  const applyRosUrlDraft = () => {
    const next = rewriteLoopbackToCurrentHost(
      normalizeRosbridgeUrl(rosUrlDraft, rosUrl),
      window.location
    );
    setRosUrl(next);
    setRosUrlDraft(next);
    sendStatus(`ROS bridge: ${next}`);
  };

  const resetRosUrlAuto = () => {
    const next = computeDefaultRosbridgeUrl(import.meta.env.VITE_ROSBRIDGE_URL, window.location);
    setRosUrl(next);
    setRosUrlDraft(next);
    sendStatus(`ROS bridge reset: ${next}`);
  };

  const sendWalkingCommand = (command, label) => {
    if (!walkingCommandPubRef.current || rosState !== "connected") {
      sendStatus("Walking command unavailable", true);
      return;
    }
    walkingCommandPubRef.current.publish(new ROSLIB.Message({ data: command }));
    sendStatus(label || `Walking command: ${command}`);
  };

  const sendTeleopCommand = (command, label) => {
    if (!teleopCommandPubRef.current || rosState !== "connected") {
      sendStatus("Teleop command unavailable", true);
      return false;
    }
    teleopCommandPubRef.current.publish(new ROSLIB.Message({ data: command }));
    sendStatus(label || `Teleop command: ${command}`);
    return true;
  };

  const applyWalkingAndRefreshTeleop = () => {
    if (!applyWalkingParams()) return;
    window.setTimeout(() => {
      sendTeleopCommand("refresh_params", "Walking applied; teleop baseline refresh requested");
    }, 150);
  };

  const updateWalkingParam = (key, value) => {
    setWalkingParams((prev) => ({
      ...prev,
      [key]: WALKING_BOOL_FIELDS.has(key) ? Boolean(value) : value,
    }));
    setWalkingDirty(true);
    setWalkingError("");
  };

  const loadWalkingParams = () => {
    if (!walkingGetServiceRef.current || rosState !== "connected") {
      const message = "Walking params unavailable";
      setWalkingError(message);
      sendStatus(message, true);
      return;
    }

    setWalkingError("");
    walkingGetServiceRef.current.callService(
      new ROSLIB.ServiceRequest({ get_param: true }),
      (res) => {
        if (!res?.parameters) {
          const message = "No walking params returned";
          setWalkingError(message);
          sendStatus(message, true);
          return;
        }
        const normalized = normalizeWalkingParams(res.parameters);
        setWalkingParams(normalized);
        setWalkingCurrentParams(normalized);
        setWalkingLoaded(true);
        setWalkingDirty(false);
        sendStatus("Walking params loaded");
      },
      (err) => {
        const message = err?.message || "Failed to load walking params";
        setWalkingError(message);
        sendStatus(message, true);
      }
    );
  };

  const applyWalkingParams = () => {
    if (!walkingParamPubRef.current || rosState !== "connected") {
      const message = "Walking param publisher unavailable";
      setWalkingError(message);
      sendStatus(message, true);
      return false;
    }

    const payload = normalizeWalkingParams(walkingParams);
    walkingParamPubRef.current.publish(new ROSLIB.Message(payload));
    setWalkingParams(payload);
    setWalkingCurrentParams(payload);
    setWalkingLoaded(true);
    setWalkingDirty(false);
    setWalkingLastAppliedAt(Date.now());
    setWalkingError("");
    sendStatus("Walking params applied");
    return true;
  };

  const writeSyncItem = (itemName, value) => {
    if (!torquePubRef.current || !metrics?.joint_names?.length) return false;
    const values = metrics.joint_names.map(() => value);
    torquePubRef.current.publish(new ROSLIB.Message({
      item_name: itemName,
      joint_name: metrics.joint_names,
      value: values,
    }));
    return true;
  };

  const writeSyncItemsSequential = (items, onDone) => {
    let idx = 0;
    const next = () => {
      if (idx >= items.length) { onDone?.(); return; }
      const [name, val] = items[idx++];
      writeSyncItem(name, val);
      window.setTimeout(next, SYNC_ITEM_GAP_MS);
    };
    next();
  };

  const restoreFullSpeed = () => {
    writeSyncItemsSequential([
      ["profile_velocity", 0],
      ["profile_acceleration", 0],
    ]);
  };

  // Enable walking_module directly. Used to force a replay of action page 2
  // (INIT_BARU) first to "stamp" the walking-ready pose, but that added a
  // ~4 s animation + a jerky module handoff. walking_module's init_x/y/z/
  // pitch_offset are now FK-tuned (in op3_walking_module/config/param.yaml)
  // to match INIT_BARU page 2 geometry — its IK parks the legs at the
  // calibrated pose statically the moment it's enabled, no replay needed.
  const initThenEnableWalking = (onReady) => {
    restoreFullSpeed();
    walkingModuleEnabledRef.current = false;
    if (!walkingModulePubRef.current || rosState !== "connected") {
      sendStatus("Walking module command unavailable", true);
      return;
    }
    walkingModulePubRef.current.publish(new ROSLIB.Message({ data: "walking_module" }));
    walkingModuleEnabledRef.current = true;
    sendStatus("Walking module enabled (IK ready pose)…");
    window.setTimeout(() => { onReady?.(); }, WALKING_ENABLE_SETTLE_MS);
  };

  const applyWalkingAndStart = () => {
    if (!applyWalkingParams()) return;
    initThenEnableWalking(() => {
      sendWalkingCommand("start", "Walking dimulai dari INIT_BARU");
    });
  };

  // ===== SIM-ONLY walking pipeline =====
  // Publishes through the same rosbridge as the real-robot path, but on the
  // /bascorro_studio/sim/* namespace. The sim_domain_bridge process running
  // alongside manager_sim subscribes to those topics on the real domain and
  // republishes them on the sim ROS_DOMAIN_ID under their /robotis/* names,
  // so the commands physically cannot reach the real robot's stack.
  const publishOnSim = (name, messageType, data) => {
    const ros = rosRef.current;
    if (!ros) return false;
    const topic = new ROSLIB.Topic({ ros, name, messageType });
    topic.publish(new ROSLIB.Message(data));
    return true;
  };

  const applyWalkingAndStartOnSim = () => {
    if (rosState !== "connected") {
      sendStatus("ROS bridge belum siap", true);
      return;
    }
    const params = normalizeWalkingParams(walkingParams);
    if (!publishOnSim("/bascorro_studio/sim/walking/set_params", "op3_walking_module_msgs/WalkingParam", params)) return;
    sendStatus("[SIM] Parameter terkirim — enable walking_module…");

    // Skip the action_module → page 2 → walking_module relay (used to "stamp"
    // INIT_BARU). walking_module's IK now produces the same calibrated ready
    // pose statically from init_x/y/z_offset, so we just enable head + walking
    // and start. Mirror change in op3_demo + real-robot path above.
    publishOnSim("/bascorro_studio/sim/enable_ctrl_module", "std_msgs/String", { data: "head_control_module" });
    publishOnSim("/bascorro_studio/sim/enable_ctrl_module", "std_msgs/String", { data: "walking_module" });
    window.setTimeout(() => {
      publishOnSim("/bascorro_studio/sim/walking/command", "std_msgs/String", { data: "start" });
      sendStatus("[SIM] Walking dimulai di simulator (robot asli tidak ikut bergerak)");
    }, WALKING_ENABLE_SETTLE_MS);
  };

  const stopWalkingOnSim = () => {
    if (rosState !== "connected") {
      sendStatus("ROS bridge belum siap", true);
      return;
    }
    publishOnSim("/bascorro_studio/sim/walking/command", "std_msgs/String", { data: "stop" });
    sendStatus("[SIM] Stop dikirim ke manager_sim");
  };


  const enableWalkingModule = () => {
    initThenEnableWalking(() => sendStatus("Walking module enabled di INIT_BARU"));
  };

  const applyWalkingPreset = (preset) => {
    const presetParams = {
      tiny: {
        x_move_amplitude: 0.003,
        y_move_amplitude: 0,
        angle_move_amplitude: 0,
        period_time: 0.85,
        z_move_amplitude: 0.03,
        dsp_ratio: 0.35,
        balance_enable: true,
      },
      stop: {
        x_move_amplitude: 0,
        y_move_amplitude: 0,
        angle_move_amplitude: 0,
      },
      balance: {
        balance_enable: true,
        balance_hip_roll_gain: 0.35,
        balance_knee_gain: 0.4,
        balance_ankle_roll_gain: 0.7,
        balance_ankle_pitch_gain: 0.9,
      },
    }[preset];

    if (!presetParams) return;
    setWalkingParams((prev) => normalizeWalkingParams({ ...prev, ...presetParams }));
    setWalkingDirty(true);
    setWalkingError("");
  };

  const saveWalkingVersion = () => {
    const now = Date.now();
    const selected = walkingVersions.find((version) => version.id === activeWalkingVersionId);
    const name = walkingVersionName.trim() || selected?.name || `Walking ${new Date(now).toLocaleString()}`;
    const params = normalizeWalkingParams(walkingParams);

    if (selected) {
      setWalkingVersions((versions) =>
        versions.map((version) =>
          version.id === selected.id
            ? { ...version, name, params, updatedAt: now }
            : version
        )
      );
      setWalkingVersionName(name);
      sendStatus(`Walking version updated: ${name}`);
      return;
    }

    const next = {
      id: makeWalkingVersionId(),
      name,
      createdAt: now,
      updatedAt: now,
      params,
    };
    setWalkingVersions((versions) => [next, ...versions]);
    setActiveWalkingVersionId(next.id);
    setWalkingVersionName(name);
    sendStatus(`Walking version saved: ${name}`);
  };

  const loadWalkingVersion = () => {
    const selected = walkingVersions.find((version) => version.id === activeWalkingVersionId);
    if (!selected) {
      sendStatus("Select a walking version first", true);
      return;
    }
    setWalkingParams(normalizeWalkingParams(selected.params));
    setWalkingVersionName(selected.name);
    setWalkingDirty(true);
    setWalkingError("");
    sendStatus(`Walking version loaded: ${selected.name}`);
  };

  const deleteWalkingVersion = () => {
    const selected = walkingVersions.find((version) => version.id === activeWalkingVersionId);
    if (!selected) {
      sendStatus("Select a walking version first", true);
      return;
    }
    setWalkingVersions((versions) => versions.filter((version) => version.id !== selected.id));
    setActiveWalkingVersionId("");
    setWalkingVersionName("");
    sendStatus(`Walking version deleted: ${selected.name}`);
  };

  const refreshCameraTopics = () => {
    if (!rosRef.current || rosState !== "connected") {
      setCameraTopicsError("ROS not connected");
      return;
    }
    setCameraTopicsLoading(true);
    setCameraTopicsError("");
    const ros = rosRef.current;
    const handleTopics = (topics) => {
      setCameraTopics(Array.isArray(topics) ? topics : []);
      setCameraTopicsLoading(false);
    };
    const handleError = () => {
      setCameraTopicsError("Failed to query topics");
      setCameraTopicsLoading(false);
    };
    if (typeof ros.getTopicsForType === "function") {
      ros.getTopicsForType("sensor_msgs/Image", (res) => {
        const topics = Array.isArray(res) ? res : res?.topics;
        handleTopics(topics);
      }, handleError);
      return;
    }
    if (typeof ros.getTopics === "function") {
      ros.getTopics((res) => {
        const topics = res?.topics || [];
        const types = res?.types || [];
        const imageTopics = topics.filter((topic, idx) => types[idx] === "sensor_msgs/Image");
        handleTopics(imageTopics);
      }, handleError);
      return;
    }
    setCameraTopicsError("ROS API unavailable");
    setCameraTopicsLoading(false);
  };

  const handleInitPose = () => {
    // "Init Pose" button literally plays action page 2 (INIT_BARU — the
    // user's calibrated standing pose recorded in the action editor).
    // Walking-ready is a separate transition triggered by Apply & Start
    // (it switches to walking_module, whose FK-tuned IK pose matches
    // INIT_BARU geometry so the handoff is smooth).
    if (!walkingModulePubRef.current || !actionPagePubRef.current || rosState !== "connected") {
      sendStatus("Init Pose: action module unavailable", true);
      return;
    }
    walkingModuleEnabledRef.current = false;
    walkingModulePubRef.current.publish(new ROSLIB.Message({ data: "action_module" }));
    sendStatus(`Init Pose: switching to action_module → page ${INIT_BARU_PAGE_NUM}…`);
    window.setTimeout(() => {
      actionPagePubRef.current.publish(new ROSLIB.Message({ data: INIT_BARU_PAGE_NUM }));
      sendStatus(`Init Pose: playing page ${INIT_BARU_PAGE_NUM} (INIT_BARU)`);
    }, ACTION_MODULE_SETTLE_MS);
  };

  const handleSoftStop = () => {
    sendWalkingCommand("stop", "Soft Stop Sent");
  };

  const handleTorque = (enable) => {
    if (!torquePubRef.current || !metrics?.joint_names?.length) {
      sendStatus("Torque command unavailable", true);
      return;
    }
    if (!enable) {
      writeSyncItem("torque_enable", 0);
      sendStatus("Torque OFF");
      return;
    }
    const preset = TORQUE_SPEED_PRESETS.find(p => p.key === torqueSpeedKey) || TORQUE_SPEED_PRESETS[1];
    writeSyncItemsSequential(
      [
        ["profile_velocity", preset.velocity],
        ["profile_acceleration", preset.accel],
        ["torque_enable", 1],
      ],
      () => {
        sendStatus(`Torque ON (${preset.label})`);
        window.setTimeout(restoreFullSpeed, preset.restoreMs);
      }
    );
  };

  const handleEnableHeadModule = () => {
    if (headModulePubRef.current) {
      walkingModuleEnabledRef.current = false;
      headModulePubRef.current.publish(new ROSLIB.Message({ data: "head_control_module" }));
      sendStatus("Head Module Enabled");
    }
  };

  const pushDemoStatus = (label, mode = "") => {
    setDemoCommand(label);
    setDemoCommandAt(Date.now());
    if (mode) setDemoMode(mode);
  };

  const handleDemoMode = (mode) => {
    if (!demoModePubRef.current || rosState !== "connected") {
      sendStatus("Demo mode unavailable", true);
      return;
    }
    demoModePubRef.current.publish(new ROSLIB.Message({ data: mode }));
    pushDemoStatus(`mode ${mode}`, mode);
    sendStatus(`Demo mode: ${mode}`);
  };

  const handleDemoCommand = (command) => {
    if (!demoCommandPubRef.current || rosState !== "connected") {
      sendStatus("Demo command unavailable", true);
      return;
    }
    demoCommandPubRef.current.publish(new ROSLIB.Message({ data: command }));
    pushDemoStatus(`command ${command}`);
    sendStatus(`Demo ${command}`);
  };

  const handleHealthCheck = () => {
    if (!healthCheckServiceRef.current) {
      sendStatus("Health check unavailable", true);
      return;
    }
    setHealthRunning(true);
    setHealthError("");
    healthCheckServiceRef.current.callService(
      new ROSLIB.ServiceRequest({}),
      (res) => {
        setHealthRunning(false);
        if (res?.success) {
          setHealthResult(parseHealthMessage(res.message));
          setHealthCheckedAt(Date.now());
          sendStatus("Health check complete");
        } else {
          const message = res?.message || "Health check failed";
          setHealthError(message);
          sendStatus(message, true);
        }
      },
      (err) => {
        setHealthRunning(false);
        const message = err?.message || "Health check failed";
        setHealthError(message);
        sendStatus(message, true);
      }
    );
  };

  const publishJoy = useCallback((payload) => {
    if (joyPubRef.current && rosState === "connected") {
      const now = Date.now();
      joyPubRef.current.publish(new ROSLIB.Message({
        header: {
          stamp: { sec: Math.floor(now / 1000), nanosec: (now % 1000) * 1000000 },
          frame_id: "bascorro_studio"
        },
        axes: payload?.axes || [],
        buttons: payload?.buttons || []
      }));
      return true;
    }
    return false;
  }, [rosState]);

  const applyYoloParams = () => {
    if (yoloSetServiceRef.current) {
      const parameters = YOLO_PARAM_KEYS.map(name => ({
        name,
        value: makeParamValue("double", yoloParams[name])
      }));
      yoloSetServiceRef.current.callService(new ROSLIB.ServiceRequest({ parameters }), (res) => {
        if (res?.results) sendStatus("YOLO Params Applied");
        else sendStatus("YOLO Update Failed", true);
      });
    }
  };

  // --- Derived Data ---
  const battery = metrics?.battery || {};
  const system = metrics?.system || {};
  const network = metrics?.network || {};
  const imu = metrics?.imu || {};
  const torque = metrics?.torque || {};
  const healthOk = healthResult?.ok || [];
  const healthFailed = healthResult?.failed || [];
  const healthErrors = healthResult?.errors || {};
  const healthTotal = healthResult?.total ?? (healthOk.length + healthFailed.length);
  const healthDuration = healthResult?.duration_ms;
  const healthSkipped = healthResult?.skipped;
  const healthRaw = healthResult?.raw;
  // Read straight from the servos by the health service. A joint that answers a
  // ping but reports torque "off", or carries an error latch, is exactly the
  // case that used to be invisible: it looks alive over ROS while ignoring
  // every torque command.
  const healthTorque = healthResult?.torque || {};
  const healthHwErrors = healthResult?.hw_errors || {};
  const healthTorqueOff = Object.entries(healthTorque)
    .filter(([, state]) => state === "off")
    .map(([name]) => name);
  const healthHwErrorList = Object.entries(healthHwErrors);
  // Dynamixel trips its overheating latch at max_temperature_limit (80 C by
  // default). Listing anything from 50 C up shows the joints heading there
  // while there is still time to act, not just the ones that already gave up.
  const healthHotJoints = Object.entries(healthResult?.temps || {})
    .map(([name, value]) => [name, Number(value)])
    .filter(([, celsius]) => Number.isFinite(celsius) && celsius >= 50)
    .sort((a, b) => b[1] - a[1]);
  const torqueEntries = useMemo(() => {
    const e = Object.entries(torque?.joints || {});
    e.sort((a, b) => Math.abs(b[1]) - Math.abs(a[1]));
    return e.slice(0, 8);
  }, [torque]);

  // --- Render Views ---

  const renderSidebar = () => (
    <>
      {sidebarOpen && (
        <div 
          className="fixed inset-0 bg-black/50 z-30 lg:hidden backdrop-blur-sm"
          onClick={() => setSidebarOpen(false)}
        />
      )}
      <nav className={`
        fixed inset-y-0 left-0 z-40 w-64 bg-sidebar flex flex-col p-6 text-gray-400 transition-transform duration-300 lg:translate-x-0 lg:static lg:flex-shrink-0
        ${sidebarOpen ? "translate-x-0" : "-translate-x-full"}
      `}>
        <div className="flex items-center justify-between mb-8 px-2">
          <div className="flex items-center gap-3">
            <img src="/log.png" alt="Bascorro Logo" className="w-8 h-8 object-contain" />
            <span className="font-display font-bold text-lg text-white tracking-tight">Bascorro</span>
          </div>
          <button onClick={() => setSidebarOpen(false)} className="lg:hidden text-gray-400 hover:text-white p-1">
            <X size={20} />
          </button>
        </div>
        <div className="flex flex-col gap-2 flex-1">
          <button
            className={`flex items-center gap-3 px-4 py-3 rounded-lg font-medium transition-all ${activeTab === "dashboard" ? "bg-undip-blue text-white shadow-md border border-accent-yellow/20" : "hover:bg-white/5 hover:text-white"}`}
            onClick={() => { setActiveTab("dashboard"); setSidebarOpen(false); }}
          >
            <LayoutDashboard size={20} className={activeTab === "dashboard" ? "text-accent-yellow" : ""} />
            <span>Dashboard</span>
          </button>
          <button
            className={`flex items-center gap-3 px-4 py-3 rounded-lg font-medium transition-all ${activeTab === "charts" ? "bg-undip-blue text-white shadow-md border border-accent-yellow/20" : "hover:bg-white/5 hover:text-white"}`}
            onClick={() => { setActiveTab("charts"); setSidebarOpen(false); }}
          >
            <Activity size={20} className={activeTab === "charts" ? "text-accent-yellow" : ""} />
            <span>Charts</span>
          </button>
          <button
            className={`flex items-center gap-3 px-4 py-3 rounded-lg font-medium transition-all ${activeTab === "vision" ? "bg-undip-blue text-white shadow-md border border-accent-yellow/20" : "hover:bg-white/5 hover:text-white"}`}
            onClick={() => { setActiveTab("vision"); setSidebarOpen(false); }}
          >
            <Video size={20} className={activeTab === "vision" ? "text-accent-yellow" : ""} />
            <span>Vision</span>
          </button>
          <button
            className={`flex items-center gap-3 px-4 py-3 rounded-lg font-medium transition-all ${activeTab === "tuning" ? "bg-undip-blue text-white shadow-md border border-accent-yellow/20" : "hover:bg-white/5 hover:text-white"}`}
            onClick={() => { setActiveTab("tuning"); setSidebarOpen(false); }}
          >
            <Settings size={20} className={activeTab === "tuning" ? "text-accent-yellow" : ""} />
            <span>Tuning</span>
          </button>
          <button
            className={`flex items-center gap-3 px-4 py-3 rounded-lg font-medium transition-all ${activeTab === "walking" ? "bg-undip-blue text-white shadow-md border border-accent-yellow/20" : "hover:bg-white/5 hover:text-white"}`}
            onClick={() => { setActiveTab("walking"); setSidebarOpen(false); }}
          >
            <Footprints size={20} className={activeTab === "walking" ? "text-accent-yellow" : ""} />
            <span>Walking</span>
          </button>
          <button
            className={`flex items-center gap-3 px-4 py-3 rounded-lg font-medium transition-all ${activeTab === "teleop" ? "bg-undip-blue text-white shadow-md border border-accent-yellow/20" : "hover:bg-white/5 hover:text-white"}`}
            onClick={() => { setActiveTab("teleop"); setSidebarOpen(false); }}
          >
            <Gamepad2 size={20} className={activeTab === "teleop" ? "text-accent-yellow" : ""} />
            <span>Teleop</span>
          </button>
          <button
            className={`flex items-center gap-3 px-4 py-3 rounded-lg font-medium transition-all ${activeTab === "action" ? "bg-undip-blue text-white shadow-md border border-accent-yellow/20" : "hover:bg-white/5 hover:text-white"}`}
            onClick={() => { setActiveTab("action"); setSidebarOpen(false); }}
          >
            <BicepsFlexed size={20} className={activeTab === "action" ? "text-accent-yellow" : ""} />
            <span>Action</span>
          </button>
          <button
            className={`flex items-center gap-3 px-4 py-3 rounded-lg font-medium transition-all ${activeTab === "logs" ? "bg-undip-blue text-white shadow-md border border-accent-yellow/20" : "hover:bg-white/5 hover:text-white"}`}
            onClick={() => { setActiveTab("logs"); setSidebarOpen(false); }}
          >
            <ScrollText size={20} className={activeTab === "logs" ? "text-accent-yellow" : ""} />
            <span>Logs</span>
          </button>
          <button
            className={`flex items-center gap-3 px-4 py-3 rounded-lg font-medium transition-all ${activeTab === "terminal" ? "bg-undip-blue text-white shadow-md border border-accent-yellow/20" : "hover:bg-white/5 hover:text-white"}`}
            onClick={() => { setActiveTab("terminal"); setSidebarOpen(false); }}
          >
            <Terminal size={20} className={activeTab === "terminal" ? "text-accent-yellow" : ""} />
            <span>Terminal</span>
          </button>
        </div>
        <div className="mt-3 p-3 rounded-xl border border-white/10 bg-white/[0.03]">
          <label className="block text-[10px] font-bold uppercase tracking-wider text-gray-500 mb-2">
            ROS Bridge
          </label>
          <input
            type="text"
            value={rosUrlDraft}
            onChange={(e) => setRosUrlDraft(e.target.value)}
            onKeyDown={(e) => { if (e.key === "Enter") applyRosUrlDraft(); }}
            className="w-full px-2 py-1.5 rounded-md border border-white/10 bg-black/20 text-[11px] font-mono text-gray-200 focus:outline-none focus:border-accent-yellow/60"
            placeholder="ws://<robot-ip>:9090"
          />
          <div className="mt-2 flex gap-2">
            <button
              onClick={applyRosUrlDraft}
              className="flex-1 px-2 py-1.5 text-[10px] font-bold rounded-md bg-undip-blue text-white hover:bg-opacity-90"
            >
              Apply
            </button>
            <button
              onClick={resetRosUrlAuto}
              className="flex-1 px-2 py-1.5 text-[10px] font-bold rounded-md border border-white/10 text-gray-300 hover:bg-white/10"
            >
              Auto
            </button>
          </div>
        </div>
        <div className="pt-6 border-t border-white/10">
          <div className={`flex items-center gap-2 px-2 text-sm font-medium ${rosState === 'connected' ? 'text-green-400' : 'text-red-400'}`}>
            <div className={`w-2 h-2 rounded-full ${rosState === 'connected' ? 'bg-green-400 shadow-[0_0_8px_rgba(74,222,128,0.5)]' : 'bg-red-400'}`}></div>
            <span>{rosState === "connected" ? "System Online" : "Disconnected"}</span>
          </div>
        </div>
      </nav>
    </>
  );

  const renderWalking = () => {
    const rosConnected = rosState === "connected";
    const walkingButtonBase = "inline-flex items-center justify-center gap-2 px-3 py-2 rounded-lg text-xs font-bold transition-colors disabled:opacity-50 disabled:cursor-not-allowed";
    const currentValue = (field) => {
      if (!walkingCurrentParams) return "not loaded";
      return formatWalkingValue(walkingCurrentParams[field.key], field);
    };
    const isFieldDirty = (field) => {
      if (!walkingCurrentParams) return walkingLoaded;
      if (field.type === "bool") return Boolean(walkingParams[field.key]) !== Boolean(walkingCurrentParams[field.key]);
      return Number(walkingParams[field.key]) !== Number(walkingCurrentParams[field.key]);
    };
    const selectedWalkingVersion = walkingVersions.find((version) => version.id === activeWalkingVersionId);
    const renderWalkingHelp = (field) => (
      <div className="absolute left-4 right-4 top-[calc(100%-4px)] z-30 hidden rounded-xl border border-undip-blue/20 bg-white p-4 text-xs shadow-xl group-hover/param:block group-focus-within/param:block lg:left-5 lg:right-auto lg:w-[380px]">
        <div className="flex items-start justify-between gap-3">
          <div>
            <div className="font-bold text-gray-900">{field.label}</div>
            <div className="mt-1 font-mono text-[11px] text-gray-400">
              {field.key}{field.unit ? ` | ${field.unit}` : ""} | current {currentValue(field)} | edited {formatWalkingValue(walkingParams[field.key], field)}
            </div>
          </div>
          <Info size={16} className="mt-0.5 flex-shrink-0 text-undip-blue" />
        </div>
        <div className="mt-3 space-y-2 leading-relaxed text-gray-600">
          <p><span className="font-bold text-gray-800">Artinya:</span> {field.artinya}</p>
          <p><span className="font-bold text-gray-800">Fungsi:</span> {field.fungsi}</p>
          <p><span className="font-bold text-gray-800">Tuning:</span> {field.tuning}</p>
          <p className="text-yellow-700"><span className="font-bold">Risiko:</span> {field.risk}</p>
        </div>
      </div>
    );

    return (
      <div className="h-full p-4 md:p-8 overflow-y-auto">
        <div className="max-w-7xl mx-auto flex flex-col gap-5">
          {/* HEADER CARD — status, workflow hint, action buttons grouped by intent */}
          <div className="bg-white rounded-2xl border border-gray-200 shadow-sm overflow-hidden">
            {/* Title strip */}
            <div className="px-5 pt-5 pb-3 border-b border-gray-100">
              <div className="flex flex-wrap items-center gap-2">
                <h2 className="text-lg font-bold font-display text-gray-900">Walking</h2>
                <span className={`px-2 py-0.5 rounded-full text-[10px] font-bold uppercase ${walkingDirty ? "bg-yellow-100 text-yellow-700" : "bg-green-50 text-green-700"}`} title={walkingDirty ? "Ada perubahan yang belum di-Apply" : "UI sudah sama dengan runtime"}>
                  {walkingDirty ? "Belum di-Apply" : "Tersinkron"}
                </span>
                <span className={`px-2 py-0.5 rounded-full text-[10px] font-bold uppercase ${walkingLoaded ? "bg-blue-50 text-undip-blue" : "bg-gray-100 text-gray-500"}`} title={walkingLoaded ? "Nilai diambil dari robot via Load" : "Belum pernah Load — masih nilai default"}>
                  {walkingLoaded ? "Runtime" : "Default"}
                </span>
                {walkingLastAppliedAt && (
                  <span className="text-[10px] font-mono text-gray-400" title="Waktu terakhir Apply dipanggil">
                    Apply: {new Date(walkingLastAppliedAt).toLocaleTimeString()}
                  </span>
                )}
              </div>
              <p className="mt-1 text-xs text-gray-500">
                Workflow: <b>Enable Module</b> → ubah parameter → <b>Apply</b> → <b>Start</b>. Satuan: detik, meter, radian.
              </p>
              {walkingError && (
                <p className="mt-2 text-xs text-red-600 font-mono">{walkingError}</p>
              )}
            </div>

            {/* Apply group — primary actions (kirim parameter ke robot) */}
            <div className="px-5 py-3 bg-gray-50/60 border-b border-gray-100 flex flex-wrap items-center gap-2">
              <span className="text-[10px] font-bold text-gray-500 uppercase tracking-wider mr-1">Parameter</span>
              <button
                className={`${walkingButtonBase} bg-blue-50 text-undip-blue border border-blue-100 hover:bg-blue-100`}
                onClick={enableWalkingModule}
                disabled={!rosConnected}
                title="Aktifkan walking_module pada controller (wajib sebelum Apply/Start)"
              >
                <Cpu size={14} /> Enable Module
              </button>
              <button
                className={`${walkingButtonBase} bg-gray-100 text-gray-700 hover:bg-gray-200`}
                onClick={loadWalkingParams}
                disabled={!rosConnected}
                title="Tarik parameter walking yang sedang aktif dari robot"
              >
                <RefreshCw size={14} /> Load
              </button>
              <div className="h-5 w-px bg-gray-300 mx-1" />
              <button
                className={`${walkingButtonBase} bg-undip-blue text-white hover:bg-opacity-90`}
                onClick={applyWalkingParams}
                disabled={!rosConnected}
                title="Kirim parameter UI ke robot tanpa menjalankan walking"
              >
                <Send size={14} /> Apply
              </button>
              <button
                className={`${walkingButtonBase} bg-green-600 text-white hover:bg-green-700`}
                onClick={applyWalkingAndStart}
                disabled={!rosConnected}
                title="Apply parameter + langsung jalan (gabungan Apply + Start)"
              >
                <Play size={14} /> Apply &amp; Start
              </button>
              <details className="ml-auto">
                <summary className="text-xs font-semibold text-gray-500 hover:text-undip-blue cursor-pointer select-none">Advanced</summary>
                <div className="absolute right-5 mt-1 z-10 bg-white border border-gray-200 rounded-lg shadow-lg p-2 flex flex-col gap-1 min-w-[220px]">
                  <button
                    className={`${walkingButtonBase} bg-white text-gray-700 border border-gray-200 hover:bg-gray-50 justify-start`}
                    onClick={applyWalkingAndRefreshTeleop}
                    disabled={!rosConnected}
                    title="Apply + minta teleop publish baseline parameter baru"
                  >
                    <Gamepad2 size={14} /> Apply + Teleop Refresh
                  </button>
                </div>
              </details>
            </div>

            {/* Control group — runtime commands */}
            <div className="px-5 py-3 flex flex-wrap items-center gap-2">
              <span className="text-[10px] font-bold text-gray-500 uppercase tracking-wider mr-1">Kontrol</span>
              <button className={`${walkingButtonBase} bg-green-50 text-green-700 border border-green-100 hover:bg-green-100`} onClick={() => sendWalkingCommand("start")} disabled={!rosConnected} title="Mulai jalan dengan parameter yang sudah di-Apply">
                <Play size={14} /> Start
              </button>
              <button className={`${walkingButtonBase} bg-red-50 text-red-600 border border-red-100 hover:bg-red-100`} onClick={() => sendWalkingCommand("stop")} disabled={!rosConnected} title="Berhenti jalan (parameter tetap aktif)">
                <Square size={14} /> Stop
              </button>
              <div className="h-5 w-px bg-gray-300 mx-1" />
              <button className={`${walkingButtonBase} bg-gray-50 text-gray-700 border border-gray-200 hover:bg-gray-100`} onClick={() => sendWalkingCommand("balance on")} disabled={!rosConnected} title="Aktifkan balance IMU feedback">
                Balance On
              </button>
              <button className={`${walkingButtonBase} bg-gray-50 text-gray-700 border border-gray-200 hover:bg-gray-100`} onClick={() => sendWalkingCommand("balance off")} disabled={!rosConnected} title="Matikan balance IMU feedback">
                Balance Off
              </button>
              <div className="h-5 w-px bg-gray-300 mx-1" />
              <button className={`${walkingButtonBase} bg-yellow-50 text-yellow-700 border border-yellow-100 hover:bg-yellow-100`} onClick={() => sendWalkingCommand("save", "Walking params save command sent")} disabled={!rosConnected} title="Simpan parameter runtime ke config robot (persistent)">
                <Save size={14} /> Save to Robot
              </button>
            </div>
          </div>

          {/* QUICK PRESETS — single inline row */}
          <div className="bg-white rounded-2xl border border-gray-200 shadow-sm p-4">
            <div className="flex flex-wrap items-center gap-2 mb-3">
              <h3 className="text-sm font-bold text-gray-800">Preset Cepat</h3>
              <span className="text-[10px] text-gray-400">Klik untuk isi UI dengan nilai preset (belum di-Apply)</span>
            </div>
            <div className="grid grid-cols-1 sm:grid-cols-3 gap-2">
              <button
                className="text-left bg-gray-50 border border-gray-200 rounded-lg p-3 hover:border-undip-blue/40 hover:bg-white transition-all"
                onClick={() => applyWalkingPreset("tiny")}
              >
                <div className="text-xs font-bold text-gray-900">Tiny Test</div>
                <div className="mt-0.5 text-[11px] text-gray-500">Slow 0.85s, 3mm maju, balance on.</div>
              </button>
              <button
                className="text-left bg-gray-50 border border-gray-200 rounded-lg p-3 hover:border-undip-blue/40 hover:bg-white transition-all"
                onClick={() => applyWalkingPreset("stop")}
              >
                <div className="text-xs font-bold text-gray-900">Stop Motion</div>
                <div className="mt-0.5 text-[11px] text-gray-500">x/y/turn = 0, posture &amp; gain tetap.</div>
              </button>
              <button
                className="text-left bg-gray-50 border border-gray-200 rounded-lg p-3 hover:border-undip-blue/40 hover:bg-white transition-all"
                onClick={() => applyWalkingPreset("balance")}
              >
                <div className="text-xs font-bold text-gray-900">Balance Defaults</div>
                <div className="mt-0.5 text-[11px] text-gray-500">Hip 0.35, knee 0.4, ankle 0.7/0.9.</div>
              </button>
            </div>
          </div>

          {/* SAVED VERSIONS — single compact row */}
          <div className="bg-white rounded-2xl border border-gray-200 shadow-sm p-4">
            <div className="flex flex-wrap items-center gap-2 mb-3">
              <h3 className="text-sm font-bold text-gray-800">Versi Tersimpan</h3>
              <span className="rounded-full bg-gray-100 px-2 py-0.5 text-[10px] font-semibold text-gray-500">
                Tersimpan di browser
              </span>
              <span className="text-[10px] text-gray-400">Load Version mengisi UI — tetap perlu Apply untuk kirim ke robot</span>
            </div>
            <div className="grid grid-cols-1 sm:grid-cols-[1fr_1fr_auto_auto_auto] gap-2">
              <input
                type="text"
                value={walkingVersionName}
                onChange={(e) => setWalkingVersionName(e.target.value)}
                className="min-w-0 rounded-lg border border-gray-200 bg-gray-50 px-3 py-2 text-sm text-gray-900 focus:border-undip-blue focus:outline-none"
                placeholder="Nama versi…"
              />
              <select
                value={activeWalkingVersionId}
                onChange={(e) => {
                  const id = e.target.value;
                  const version = walkingVersions.find((item) => item.id === id);
                  setActiveWalkingVersionId(id);
                  setWalkingVersionName(version?.name || "");
                }}
                className="min-w-0 rounded-lg border border-gray-200 bg-gray-50 px-3 py-2 text-sm text-gray-900 focus:border-undip-blue focus:outline-none"
              >
                <option value="">— Pilih versi —</option>
                {walkingVersions.map((version) => (
                  <option key={version.id} value={version.id}>
                    {version.name} · {new Date(version.updatedAt).toLocaleString()}
                  </option>
                ))}
              </select>
              <button
                className={`${walkingButtonBase} bg-undip-blue text-white hover:bg-opacity-90`}
                onClick={saveWalkingVersion}
                title="Simpan nilai UI saat ini sebagai versi baru di browser"
              >
                <Save size={14} /> Save
              </button>
              <button
                className={`${walkingButtonBase} bg-gray-100 text-gray-700 hover:bg-gray-200`}
                onClick={loadWalkingVersion}
                disabled={!selectedWalkingVersion}
                title="Isi UI dengan parameter dari versi terpilih"
              >
                Load
              </button>
              <button
                className={`${walkingButtonBase} bg-red-50 text-red-600 border border-red-100 hover:bg-red-100`}
                onClick={deleteWalkingVersion}
                disabled={!selectedWalkingVersion}
                title="Hapus versi terpilih dari browser"
              >
                <Trash2 size={14} /> Delete
              </button>
            </div>
          </div>

          {/* WALKING SIMULATION PREVIEW — renders the standalone-sim joint
              stream (relayed onto /bascorro_studio/sim_preview/joint_states
              by sim_domain_bridge) in a 3D viewer, with sim Start/Stop
              wired against the cross-domain bridge so the real robot stays
              still. */}
          <div className="h-[420px]">
            <WalkingSimPreview
              rosRef={rosRef}
              rosState={rosState}
              isActive={activeTab === "walking"}
              onStartSimWalk={applyWalkingAndStartOnSim}
              onStopSimWalk={stopWalkingOnSim}
            />
          </div>

          <div className="grid grid-cols-1 xl:grid-cols-2 gap-6">
            {WALKING_PARAM_GROUPS.map((group) => (
              <div key={group.title} className="bg-white rounded-2xl border border-gray-200 shadow-sm overflow-visible">
                <div className="p-5 border-b border-gray-100 flex items-start justify-between gap-4">
                  <div>
                    <div className="flex items-center gap-2">
                      <SlidersHorizontal size={18} className="text-undip-blue" />
                      <h3 className="font-bold text-gray-900">{group.title}</h3>
                    </div>
                    <p className="mt-1 text-xs text-gray-500">{group.description}</p>
                  </div>
                </div>
                <div className="divide-y divide-gray-100">
                  {group.fields.map((field) => {
                    const dirty = isFieldDirty(field);
                    return (
                      <div key={field.key} className="group/param relative grid grid-cols-1 gap-3 p-4 md:grid-cols-[minmax(170px,1fr)_minmax(180px,220px)_minmax(150px,180px)] md:items-center">
                        <div className="min-w-0">
                          <div className="flex items-center gap-2">
                            <label className="text-sm font-bold text-gray-800">{field.label}</label>
                            <Info size={13} className="text-gray-300 transition-colors group-hover/param:text-undip-blue group-focus-within/param:text-undip-blue" />
                            {dirty && <span className="w-2 h-2 rounded-full bg-yellow-400" title="Changed" />}
                          </div>
                          <div className="mt-1 text-[11px] text-gray-400 font-mono break-all">{field.key}{field.unit ? ` (${field.unit})` : ""}</div>
                        </div>

                        {field.type === "bool" ? (
                          <button
                            type="button"
                            className={`w-full px-3 py-2 rounded-lg border text-sm font-bold transition-colors ${
                              walkingParams[field.key]
                                ? "bg-green-50 text-green-700 border-green-100 hover:bg-green-100"
                                : "bg-gray-50 text-gray-600 border-gray-200 hover:bg-gray-100"
                            }`}
                            onClick={() => updateWalkingParam(field.key, !walkingParams[field.key])}
                          >
                            {walkingParams[field.key] ? "Enabled" : "Disabled"}
                          </button>
                        ) : (
                          <input
                            type="number"
                            value={walkingParams[field.key]}
                            step={field.step || 0.001}
                            onChange={(e) => updateWalkingParam(field.key, e.target.value)}
                            className="w-full px-3 py-2 rounded-lg border border-gray-200 bg-gray-50 text-sm font-mono text-gray-900 focus:outline-none focus:ring-2 focus:ring-undip-blue/20 focus:border-undip-blue"
                          />
                        )}

                        <div className="rounded-lg bg-gray-50 border border-gray-100 px-3 py-2">
                          <div className="text-[10px] font-bold uppercase text-gray-400">Current</div>
                          <div className="text-xs font-mono text-gray-700 truncate">{currentValue(field)}</div>
                        </div>
                        {renderWalkingHelp(field)}
                      </div>
                    );
                  })}
                </div>
              </div>
            ))}
          </div>
        </div>
      </div>
    );
  };

  const renderTeleop = () => {
    const rosConnected = rosState === "connected";
    const now = Date.now();
    const statusAgeSec = teleopStatusAt ? Math.max(0, (now - teleopStatusAt) / 1000) : null;
    const statusFresh = statusAgeSec !== null && statusAgeSec < 3;
    const joyAge = teleopStatus?.last_joy_age_sec;
    const baselineAge = teleopStatus?.baseline_age_sec;
    const joyActive = Array.isArray(joyState?.buttons)
      ? joyState.buttons.some((button) => Number(button) > 0.5)
      : false;
    const axisActive = Array.isArray(joyState?.axes)
      ? joyState.axes.some((axis) => Math.abs(Number(axis) || 0) > 0.05)
      : false;

    const fmt = (value, digits = 3, suffix = "") => {
      if (typeof value !== "number" || Number.isNaN(value)) return "--";
      return `${value.toFixed(digits)}${suffix}`;
    };
    const fmtAge = (value) => {
      if (typeof value !== "number" || Number.isNaN(value)) return "--";
      if (value < 10) return `${value.toFixed(1)}s`;
      return `${Math.round(value)}s`;
    };
    const commandButton = (command, label, className = "") => (
      <button
        className={`flex items-center justify-center gap-2 rounded-lg px-3 py-2 text-sm font-bold transition-colors disabled:cursor-not-allowed disabled:opacity-50 ${className || "border border-gray-200 bg-white text-gray-700 hover:bg-gray-50"}`}
        onClick={() => sendTeleopCommand(command, `Teleop: ${label}`)}
        disabled={!rosConnected}
      >
        {label}
      </button>
    );

    return (
      <div className="h-full p-4 md:p-8 overflow-y-auto">
        <div className="mx-auto flex max-w-7xl flex-col gap-6">
          <div className="rounded-2xl border border-gray-200 bg-white p-5 shadow-sm">
            <div className="flex flex-col gap-4 xl:flex-row xl:items-center xl:justify-between">
              <div>
                <div className="flex flex-wrap items-center gap-2">
                  <h2 className="font-display text-lg font-bold text-gray-900">OP3 Joy Teleop</h2>
                  <span className={`rounded-full px-2 py-1 text-[10px] font-bold uppercase ${statusFresh ? "bg-green-50 text-green-700" : "bg-gray-100 text-gray-500"}`}>
                    {statusFresh ? "Status Live" : "No Teleop Status"}
                  </span>
                  <span className={`rounded-full px-2 py-1 text-[10px] font-bold uppercase ${teleopStatus?.baseline_loaded ? "bg-blue-50 text-undip-blue" : "bg-yellow-100 text-yellow-700"}`}>
                    {teleopStatus?.baseline_loaded ? `Baseline v${teleopStatus.baseline_version || 0}` : "Baseline Missing"}
                  </span>
                </div>
                <p className="mt-1 text-sm text-gray-500">
                  Tune Walking, apply it to runtime, then refresh teleop baseline so joystick commands inherit the same gait.
                </p>
                <p className="mt-1 text-xs font-mono text-gray-400">
                  command {TELEOP_COMMAND_TOPIC} | status {TELEOP_STATUS_TOPIC}
                </p>
                {teleopStatus?.baseline_error && (
                  <p className="mt-2 text-xs font-mono text-red-600">{teleopStatus.baseline_error}</p>
                )}
              </div>
              <div className="grid grid-cols-2 gap-2 md:flex md:flex-wrap">
                {commandButton("refresh_params", "Refresh Baseline", "bg-undip-blue text-white hover:bg-opacity-90")}
                <button
                  className="flex items-center justify-center gap-2 rounded-lg bg-white px-3 py-2 text-sm font-bold text-gray-700 border border-gray-200 transition-colors hover:bg-gray-50 disabled:cursor-not-allowed disabled:opacity-50"
                  onClick={applyWalkingAndRefreshTeleop}
                  disabled={!rosConnected}
                >
                  <Send size={14} /> Apply Walking + Refresh
                </button>
                {commandButton("start", "Enable + Start", "bg-green-600 text-white hover:bg-green-700")}
                {commandButton("stop", "Stop + Zero", "bg-red-50 text-red-600 border border-red-100 hover:bg-red-100")}
                {commandButton("zero_params", "Zero Motion")}
                {commandButton("status", "Poll Status")}
              </div>
            </div>
          </div>

          <div className="grid grid-cols-1 gap-6 xl:grid-cols-[1fr_1fr]">
            <div className="rounded-2xl border border-gray-200 bg-white shadow-sm">
              <div className="border-b border-gray-100 p-5">
                <div className="flex items-center gap-2">
                  <Gamepad2 size={18} className="text-undip-blue" />
                  <h3 className="font-bold text-gray-900">Runtime State</h3>
                </div>
              </div>
              <div className="grid grid-cols-2 gap-3 p-5 md:grid-cols-3">
                <div className="rounded-xl border border-gray-100 bg-gray-50 p-3">
                  <div className="text-[10px] font-bold uppercase text-gray-400">Deadman</div>
                  <div className={`mt-1 text-sm font-bold ${teleopStatus?.deadman_active ? "text-green-700" : "text-gray-700"}`}>
                    {teleopStatus?.deadman_active ? "Active" : "Released"}
                  </div>
                </div>
                <div className="rounded-xl border border-gray-100 bg-gray-50 p-3">
                  <div className="text-[10px] font-bold uppercase text-gray-400">Joy</div>
                  <div className={`mt-1 text-sm font-bold ${(joyActive || axisActive) ? "text-green-700" : "text-gray-700"}`}>
                    {(joyActive || axisActive) ? "Input Moving" : "Idle"}
                  </div>
                  <div className="mt-0.5 text-[11px] font-mono text-gray-400">{fmtAge(joyAge)}</div>
                </div>
                <div className="rounded-xl border border-gray-100 bg-gray-50 p-3">
                  <div className="text-[10px] font-bold uppercase text-gray-400">Gear</div>
                  <div className="mt-1 text-sm font-bold text-gray-800">
                    {teleopStatus?.gear || "--"} x{fmt(teleopStatus?.gear_scale, 2)}
                  </div>
                </div>
                <div className="rounded-xl border border-gray-100 bg-gray-50 p-3">
                  <div className="text-[10px] font-bold uppercase text-gray-400">Heading Hold</div>
                  <div className={`mt-1 text-sm font-bold ${teleopStatus?.heading_hold ? "text-undip-blue" : "text-gray-700"}`}>
                    {teleopStatus?.heading_hold ? "On" : "Off"}
                  </div>
                </div>
                <div className="rounded-xl border border-gray-100 bg-gray-50 p-3">
                  <div className="text-[10px] font-bold uppercase text-gray-400">Baseline Age</div>
                  <div className="mt-1 text-sm font-bold text-gray-800">{fmtAge(baselineAge)}</div>
                </div>
                <div className="rounded-xl border border-gray-100 bg-gray-50 p-3">
                  <div className="text-[10px] font-bold uppercase text-gray-400">Status Age</div>
                  <div className="mt-1 text-sm font-bold text-gray-800">{fmtAge(statusAgeSec)}</div>
                </div>
              </div>
            </div>

            <div className="rounded-2xl border border-gray-200 bg-white shadow-sm">
              <div className="border-b border-gray-100 p-5">
                <div className="flex items-center gap-2">
                  <SlidersHorizontal size={18} className="text-undip-blue" />
                  <h3 className="font-bold text-gray-900">Teleop Commands</h3>
                </div>
              </div>
              <div className="space-y-4 p-5">
                <div>
                  <div className="mb-2 text-[10px] font-bold uppercase text-gray-400">Gear</div>
                  <div className="grid grid-cols-2 gap-2 md:grid-cols-4">
                    {commandButton("gear_slow", "Slow")}
                    {commandButton("gear_normal", "Normal")}
                    {commandButton("gear_fast", "Fast")}
                    {commandButton("gear_next", "Next")}
                  </div>
                </div>
                <div>
                  <div className="mb-2 text-[10px] font-bold uppercase text-gray-400">Heading Hold</div>
                  <div className="grid grid-cols-1 gap-2 md:grid-cols-3">
                    {commandButton("heading_hold_on", "Hold On")}
                    {commandButton("heading_hold_off", "Hold Off")}
                    {commandButton("heading_hold_toggle", "Toggle")}
                  </div>
                </div>
                <div className="rounded-xl border border-blue-100 bg-blue-50 p-3 text-xs leading-relaxed text-blue-800">
                  Browser walking versions stay local. Use Walking: Load Version, Apply, then Teleop: Refresh Baseline so joystick publishes from that saved gait.
                </div>
              </div>
            </div>
          </div>

          <div className="grid grid-cols-1 gap-6 xl:grid-cols-[1fr_1fr]">
            <div className="rounded-2xl border border-gray-200 bg-white shadow-sm">
              <div className="border-b border-gray-100 p-5">
                <h3 className="font-bold text-gray-900">Current Command</h3>
              </div>
              <div className="grid grid-cols-3 gap-3 p-5">
                <div className="rounded-xl border border-gray-100 bg-gray-50 p-3">
                  <div className="text-[10px] font-bold uppercase text-gray-400">x</div>
                  <div className="mt-1 font-mono text-sm text-gray-800">{fmt(teleopStatus?.smoothed_x, 4)} m</div>
                  <div className="mt-0.5 text-[11px] text-gray-400">max {fmt(teleopStatus?.max_x, 4)}</div>
                </div>
                <div className="rounded-xl border border-gray-100 bg-gray-50 p-3">
                  <div className="text-[10px] font-bold uppercase text-gray-400">y</div>
                  <div className="mt-1 font-mono text-sm text-gray-800">{fmt(teleopStatus?.smoothed_y, 4)} m</div>
                  <div className="mt-0.5 text-[11px] text-gray-400">max {fmt(teleopStatus?.max_y, 4)}</div>
                </div>
                <div className="rounded-xl border border-gray-100 bg-gray-50 p-3">
                  <div className="text-[10px] font-bold uppercase text-gray-400">yaw</div>
                  <div className="mt-1 font-mono text-sm text-gray-800">{fmt(teleopStatus?.smoothed_yaw, 4)} rad</div>
                  <div className="mt-0.5 text-[11px] text-gray-400">target {fmt(teleopStatus?.yaw_target, 4)}</div>
                </div>
              </div>
            </div>

            <div className="rounded-2xl border border-gray-200 bg-white shadow-sm">
              <div className="border-b border-gray-100 p-5">
                <h3 className="font-bold text-gray-900">Joy Input</h3>
              </div>
              <div className="p-5">
                <GamepadVisualizer joy={joyState} rosConnected={rosConnected} publishJoy={publishJoy} />
              </div>
            </div>
          </div>
        </div>
      </div>
    );
  };

  const renderDashboard = () => (
    <div className="flex flex-col lg:grid lg:grid-cols-[3fr_1fr] gap-6 h-full p-4 md:p-8 overflow-y-auto">
      <div className="flex flex-col gap-6">
        {/* Safety & Quick Actions */}
        <div className="flex flex-wrap gap-4 items-center bg-white p-4 rounded-2xl border border-gray-200 shadow-sm">
          <button className="flex-1 sm:flex-none flex items-center justify-center gap-2 px-5 py-2.5 bg-red-50 text-red-600 border border-red-100 rounded-lg font-bold hover:bg-red-600 hover:text-white transition-all shadow-sm" onClick={handleInitPose}>
            <Settings size={18} /> Init Pose
          </button>
          <button className="flex-1 sm:flex-none flex items-center justify-center gap-2 px-5 py-2.5 bg-red-50 text-red-600 border border-red-100 rounded-lg font-bold hover:bg-red-600 hover:text-white transition-all shadow-sm" onClick={handleSoftStop}>
            <StopCircle size={18} /> Soft Stop
          </button>
          <div className="flex-1 hidden sm:block"></div>
          <div className="flex gap-2 w-full sm:w-auto items-center flex-wrap">
            <div className="flex bg-blue-50 border border-blue-200 rounded-lg p-1 text-xs font-medium" title="Kecepatan saat Torque ON (anti nyentak)">
              {TORQUE_SPEED_PRESETS.map(p => (
                <button
                  key={p.key}
                  className={`px-3 py-1 rounded-md transition-all font-semibold ${torqueSpeedKey === p.key ? "bg-undip-blue text-white shadow-sm" : "text-undip-blue hover:bg-blue-100"}`}
                  onClick={() => setTorqueSpeedKey(p.key)}
                >{p.label}</button>
              ))}
            </div>
            <button className="flex-1 sm:flex-none px-4 py-2 border border-gray-200 rounded-lg text-gray-600 font-medium hover:bg-gray-50 text-sm" onClick={() => handleTorque(false)}>Torque OFF</button>
            <button className="flex-1 sm:flex-none px-4 py-2 bg-undip-blue text-white rounded-lg font-bold hover:bg-opacity-90 shadow-sm transition-all text-sm" onClick={() => handleTorque(true)}>Torque ON</button>
          </div>
          <div className="flex gap-2 w-full sm:w-auto">
            <button className="flex-1 sm:flex-none px-4 py-2 bg-green-500 text-white rounded-lg font-bold hover:bg-green-600 shadow-sm transition-all text-sm" onClick={handleEnableHeadModule}>Enable Head</button>
          </div>
        </div>

        {/* Stats Grid */}
        <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-4 gap-6">
          <StatCard
            title="Battery"
            icon={Battery}
            value={`${formatNumber(battery.voltage, 2)} V`}
            subValue={formatPercent(battery.percent)}
            status={battery.voltage < battery.warn_voltage ? "danger" : "neutral"}
          />
          <StatCard
            title="CPU Load"
            icon={Cpu}
            value={formatPercent(system.cpu_percent)}
            subValue={`Load: ${formatNumber(system.load, 2)}`}
          />
          <StatCard
            title="Memory"
            icon={HardDrive}
            value={formatPercent(system.mem_percent)}
            subValue={`${formatNumber(system.mem_used_mb, 0)} MB Used`}
          />
          <StatCard
            title="Network"
            icon={Wifi}
            value={`${formatNumber(network.latency_ms, 0)} ms`}
            subValue="Latency"
          />
        </div>

        {/* Torque Graph */}
        <div className="bg-white p-6 rounded-2xl border border-gray-200 shadow-sm">
          <SectionHeader title="Joint Torque Load" />
          <div className="grid grid-cols-2 sm:grid-cols-4 gap-4 mt-4">
            {torqueEntries.length === 0 ? <p className="text-gray-400 text-sm col-span-2 sm:col-span-4 text-center py-8">No torque data available</p> :
              torqueEntries.map(([name, val]) => (
                <div key={name} className="flex flex-col gap-1">
                  <span className="text-xs font-medium text-gray-500 font-mono uppercase truncate">{name}</span>
                  <div className="h-2 bg-gray-100 rounded-full overflow-hidden">
                    <div
                      className={`h-full rounded-full transition-all duration-300 ${Math.abs(val) > 80 ? 'bg-red-500' : 'bg-undip-blue'}`}
                      style={{ width: `${Math.min(Math.abs(val) * 10, 100)}%` }}
                    ></div>
                  </div>
                  <span className="text-xs font-bold text-gray-700 text-right">{formatNumber(val, 2)}</span>
                </div>
              ))
            }
          </div>
        </div>
      </div>

      <div className="flex flex-col gap-6">
        {/* IMU Card */}
        <div className="bg-white p-6 rounded-2xl border border-gray-200 shadow-sm">
          <SectionHeader title="IMU Orientation" />
          <div className="flex flex-col gap-4">
            <div className="flex justify-between items-center py-2 border-b border-gray-50">
              <span className="text-sm text-gray-500">Roll</span>
              <strong className="font-mono text-gray-900">{formatNumber(imu.roll, 1)}°</strong>
            </div>
            <div className="flex justify-between items-center py-2 border-b border-gray-50">
              <span className="text-sm text-gray-500">Pitch</span>
              <strong className="font-mono text-gray-900">{formatNumber(imu.pitch, 1)}°</strong>
            </div>
            <div className="flex justify-between items-center py-2 border-b border-gray-50">
              <span className="text-sm text-gray-500">Yaw</span>
              <strong className="font-mono text-gray-900">{formatNumber(imu.yaw, 1)}°</strong>
            </div>
            <div className={`mt-4 text-center p-2 rounded-lg text-xs font-bold uppercase tracking-wider ${metrics?.fall?.state !== "upright" ? "bg-red-100 text-red-600" : "bg-green-50 text-green-600"}`}>
              State: {metrics?.fall?.state || "Unknown"}
            </div>
          </div>
        </div>

        {/* Demo Control */}
        <div className="bg-white p-6 rounded-2xl border border-gray-200 shadow-sm">
          <SectionHeader title="Demo Control" />
          <div className="flex flex-col gap-3">
            <div className="grid grid-cols-4 gap-2">
              {["ready", "soccer", "vision", "action"].map((mode) => (
                <button
                  key={mode}
                  className={`py-2 rounded-lg text-[10px] font-bold uppercase transition-all border disabled:opacity-50 disabled:cursor-not-allowed ${demoMode === mode ? "bg-undip-blue text-white border-undip-blue" : "bg-gray-50 text-gray-600 border-gray-200 hover:bg-gray-100"}`}
                  onClick={() => handleDemoMode(mode)}
                  disabled={rosState !== "connected"}
                >
                  {mode}
                </button>
              ))}
            </div>
            <div className="grid grid-cols-2 gap-2">
              <button
                className="py-2 rounded-lg text-xs font-bold bg-green-50 text-green-700 border border-green-100 hover:bg-green-100 transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
                onClick={() => handleDemoCommand("start")}
                disabled={rosState !== "connected"}
              >
                Start
              </button>
              <button
                className="py-2 rounded-lg text-xs font-bold bg-red-50 text-red-600 border border-red-100 hover:bg-red-100 transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
                onClick={() => handleDemoCommand("stop")}
                disabled={rosState !== "connected"}
              >
                Stop
              </button>
            </div>
            <div className="text-xs text-gray-500 font-mono">
              {demoCommandAt ? `${new Date(demoCommandAt).toLocaleTimeString()} | ${demoCommand}` : "No demo command sent"}
            </div>
            <div className="text-[11px] text-gray-400">
              Start/Stop affects soccer and action demos.
            </div>
          </div>
        </div>

        {/* Joint Health */}
        <div className="bg-white p-6 rounded-2xl border border-gray-200 shadow-sm">
          <SectionHeader title="Joint Health">
            <button
              className="flex items-center gap-2 px-3 py-1.5 rounded-lg text-xs font-bold bg-gray-100 text-gray-700 hover:bg-gray-200 transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
              onClick={handleHealthCheck}
              disabled={healthRunning || rosState !== "connected"}
            >
              <AlertCircle size={14} />
              {healthRunning ? "Checking..." : "Run Check"}
            </button>
          </SectionHeader>
          {healthCheckedAt && (
            <div className="text-xs text-gray-400 font-mono mb-3">
              Last check: {new Date(healthCheckedAt).toLocaleTimeString()}
            </div>
          )}
          {healthError && (
            <div className="mb-3 text-xs text-red-600 font-mono">{healthError}</div>
          )}
          {!healthResult && !healthError && (
            <div className="text-sm text-gray-400">No health check yet</div>
          )}
          {healthResult && !healthError && (
            <div className="flex flex-col gap-3">
              <div className={`px-3 py-2 rounded-lg text-xs font-bold uppercase tracking-wider ${healthSkipped ? "bg-gray-100 text-gray-500" : healthFailed.length > 0 ? "bg-red-50 text-red-600" : "bg-green-50 text-green-600"}`}>
                {healthSkipped ? "Skipped (simulation)" : healthFailed.length > 0 ? `${healthFailed.length} joints failed` : "All joints responded"}
              </div>
              <div className="flex justify-between text-xs text-gray-500 font-mono">
                <span>{healthTotal} joints</span>
                <span>{healthDuration !== undefined ? `${formatNumber(healthDuration, 1)} ms` : "-"}</span>
              </div>
              {healthHwErrorList.length > 0 && (
                <div className="rounded-lg border border-red-200 bg-red-50 px-3 py-2 space-y-1">
                  <div className="text-xs font-bold text-red-700">Servo error latch</div>
                  {healthHwErrorList.map(([name, why]) => (
                    <div key={name} className="flex justify-between text-xs">
                      <span className="font-mono text-gray-700">{name}</span>
                      <span className="text-red-600">{why}</span>
                    </div>
                  ))}
                  <div className="text-[11px] text-red-700">
                    Servo mematikan torque-nya sendiri. Torque ON tidak akan berpengaruh
                    sampai latch ini hilang — matikan daya servo lalu nyalakan lagi.
                  </div>
                </div>
              )}
              {healthHotJoints.length > 0 && (
                <div className="rounded-lg border border-orange-200 bg-orange-50 px-3 py-2 space-y-1">
                  <div className="text-xs font-bold text-orange-700">Suhu servo</div>
                  {healthHotJoints.map(([name, celsius]) => (
                    <div key={name} className="flex justify-between text-xs">
                      <span className="font-mono text-gray-700">{name}</span>
                      <span className={celsius >= 70 ? "text-red-600 font-bold" : "text-orange-700"}>
                        {celsius} °C
                      </span>
                    </div>
                  ))}
                  <div className="text-[11px] text-orange-700">
                    Latch overheating biasanya jatuh di 80 °C.
                  </div>
                </div>
              )}
              {/* Shown independently of the latch box: a joint can be torque-off
                  without a latch, and hiding this whenever any latch exists made
                  the torque state unreadable exactly when it mattered most. */}
              {healthTorqueOff.length > 0 && (
                <div className="rounded-lg border border-amber-200 bg-amber-50 px-3 py-2">
                  <div className="text-xs font-bold text-amber-700">Torque off</div>
                  <div className="font-mono text-xs text-gray-700">{healthTorqueOff.join(", ")}</div>
                </div>
              )}
              {healthFailed.length > 0 ? (
                <div className="max-h-[180px] overflow-y-auto pr-2 custom-scrollbar space-y-2">
                  {healthFailed.map((name) => (
                    <div key={name} className="flex justify-between text-xs">
                      <span className="font-mono text-gray-700">{name}</span>
                      <span className="text-red-600">{healthErrors[name] || "No response"}</span>
                    </div>
                  ))}
                </div>
              ) : (
                <div className="text-sm text-gray-500">No failures reported.</div>
              )}
              {healthRaw && (
                <div className="text-xs text-gray-400 font-mono break-words">{healthRaw}</div>
              )}
            </div>
          )}
        </div>

        {/* Logs Preview */}
        <div className="bg-white p-6 rounded-2xl border border-gray-200 shadow-sm flex-1">
          <SectionHeader title="System Events" />
          <div className="flex flex-col gap-3 mt-2 overflow-y-auto max-h-[300px] pr-2 custom-scrollbar">
            {events.slice().reverse().slice(0, 10).map((ev, i) => (
              <div key={i} className="flex gap-3 text-xs pb-3 border-b border-gray-50 last:border-0">
                <span className="font-mono text-gray-400 whitespace-nowrap">{new Date(ev.ts * 1000).toLocaleTimeString([], {hour:'2-digit', minute:'2-digit', second:'2-digit'})}</span>
                <span className="text-gray-700">{ev.message}</span>
              </div>
            ))}
            {events.length === 0 && <p className="text-gray-400 text-sm text-center py-8">No events logged</p>}
          </div>
        </div>
      </div>
    </div>
  );

  const renderVision = () => (
    <div className="flex flex-col lg:grid lg:grid-cols-[1fr_320px] gap-6 h-full p-4 md:p-8 overflow-y-auto lg:overflow-hidden">
      <div className="bg-black rounded-2xl overflow-hidden relative flex items-center justify-center border border-gray-800 shadow-lg min-h-[300px]">
        {showOverlay ? <canvas ref={overlayCanvasRef} className="max-w-full max-h-full object-contain" /> : <div className="text-gray-500 text-sm font-mono">Stream Paused</div>}
        <div className="absolute top-4 right-4 bg-black/70 text-green-400 px-3 py-1 rounded-full text-xs font-mono backdrop-blur-sm border border-white/10 flex items-center gap-2">
          <div className="w-2 h-2 rounded-full bg-green-400 animate-pulse"></div>
          {formatNumber(overlayStats.fps, 1)} FPS
        </div>
      </div>
      <div className="flex flex-col gap-6 lg:overflow-y-auto pr-2 custom-scrollbar">
        <div className="bg-white p-6 rounded-2xl border border-gray-200 shadow-sm">
          <SectionHeader title="Stream Control" />
          <div className="flex flex-col gap-4">
            <div>
              <label className="block text-xs font-bold text-gray-500 uppercase tracking-wider mb-2">Preset Topics</label>
              <div className="grid grid-cols-2 gap-2">
                <button 
                  onClick={() => setOverlayTopic("/vision/yolo/debug")} 
                  className={`py-2 rounded-lg text-[10px] font-bold transition-all border ${overlayTopic === "/vision/yolo/debug" ? 'bg-undip-blue text-white border-undip-blue' : 'bg-gray-50 text-gray-600 border-gray-200 hover:bg-gray-100'}`}
                >
                  YOLO Debug
                </button>
                <button 
                  onClick={() => setOverlayTopic("/robotis_op3/camera/image_raw")} 
                  className={`py-2 rounded-lg text-[10px] font-bold transition-all border ${overlayTopic === "/robotis_op3/camera/image_raw" ? 'bg-undip-blue text-white border-undip-blue' : 'bg-gray-50 text-gray-600 border-gray-200 hover:bg-gray-100'}`}
                >
                  Pure Camera
                </button>
              </div>
              <p className="mt-2 text-[11px] text-gray-400">
                Hardware camera (manager + usb_cam) publishes on <span className="font-mono">/robotis_op3/camera/image_raw</span>.
              </p>
            </div>
            <div>
              <label className="block text-xs font-bold text-gray-500 uppercase tracking-wider mb-2">Custom Topic</label>
              <input type="text" className="w-full px-3 py-2 bg-gray-50 border border-gray-200 rounded-lg text-sm font-mono focus:outline-none focus:ring-2 focus:ring-undip-blue/20 focus:border-undip-blue" value={overlayTopic} onChange={e => setOverlayTopic(e.target.value)} />
            </div>
            <div>
              <div className="flex items-center justify-between mb-2">
                <label className="block text-xs font-bold text-gray-500 uppercase tracking-wider">Discover Image Topics</label>
                <button
                  className="px-2 py-1 rounded-full text-[10px] font-bold bg-gray-100 text-gray-600 hover:bg-gray-200 transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
                  onClick={refreshCameraTopics}
                  disabled={rosState !== "connected" || cameraTopicsLoading}
                >
                  {cameraTopicsLoading ? "Scanning..." : "Scan"}
                </button>
              </div>
              {cameraTopicsError && (
                <div className="text-[11px] text-red-500 mb-2">{cameraTopicsError}</div>
              )}
              {cameraTopics.length ? (
                <div className="flex flex-wrap gap-2">
                  {cameraTopics.map((topic) => (
                    <button
                      key={topic}
                      className={`px-2 py-1 rounded-full text-[10px] font-bold border transition-colors ${
                        overlayTopic === topic
                          ? "bg-undip-blue text-white border-undip-blue"
                          : "bg-gray-50 text-gray-600 border-gray-200 hover:bg-gray-100"
                      }`}
                      onClick={() => setOverlayTopic(topic)}
                    >
                      {topic}
                    </button>
                  ))}
                </div>
              ) : (
                <div className="text-[11px] text-gray-400">No image topics found yet.</div>
              )}
            </div>
            <div className="flex gap-2">
              <button className="flex-1 py-2 bg-gray-100 hover:bg-gray-200 text-gray-700 rounded-lg text-sm font-bold transition-colors" onClick={() => setShowOverlay(!showOverlay)}>
                {showOverlay ? "Stop Stream" : "Start Stream"}
              </button>
              <button className="flex-1 py-2 bg-undip-blue text-white hover:bg-opacity-90 rounded-lg text-sm font-bold transition-colors flex items-center justify-center gap-2" onClick={() => snapshotServiceRef.current?.callService({}, () => sendStatus("Snapshot Saved"))}>
                <Camera size={14} /> Snap
              </button>
            </div>
          </div>
        </div>
        <div className="bg-white p-6 rounded-2xl border border-gray-200 shadow-sm">
          <SectionHeader title="YOLO Thresholds" />
          <div className="flex flex-col gap-6 mt-2">
            {YOLO_PARAM_KEYS.map(key => (
              <div key={key}>
                <div className="flex justify-between mb-2">
                  <label className="text-xs font-bold text-gray-500 uppercase tracking-wider">{key.replace(/_confidence_threshold|_/g, " ")}</label>
                  <span className="text-xs font-mono font-bold text-undip-blue">{yoloParams[key].toFixed(2)}</span>
                </div>
                <input
                  type="range" min="0" max="1" step="0.05"
                  className="w-full accent-undip-blue h-2 bg-gray-100 rounded-lg appearance-none cursor-pointer"
                  value={yoloParams[key]}
                  onChange={e => setYoloParams({ ...yoloParams, [key]: Number(e.target.value) })}
                />
              </div>
            ))}
            <button className="w-full py-3 bg-accent-yellow text-black hover:bg-yellow-400 rounded-lg text-sm font-bold transition-colors shadow-sm" onClick={applyYoloParams}>Apply Thresholds</button>
          </div>
        </div>
      </div>
    </div>
  );

  const renderLogs = () => (
    <div className="h-full p-4 md:p-8 overflow-hidden flex flex-col">
      <div className="bg-white rounded-2xl border border-gray-200 shadow-sm flex-1 flex flex-col overflow-hidden">
        <div className="p-6 border-b border-gray-100 flex justify-between items-center">
          <h2 className="text-lg font-bold font-display text-gray-800">System Logs</h2>
          <div className="flex gap-2">
             <span className="px-3 py-1 bg-gray-100 rounded-full text-xs font-medium text-gray-600">{events.length} Events</span>
          </div>
        </div>
        <div className="flex-1 overflow-y-auto p-0">
          <table className="w-full text-left border-collapse">
            <thead className="bg-gray-50 sticky top-0 z-10">
              <tr>
                <th className="py-3 px-6 text-xs font-bold text-gray-500 uppercase tracking-wider w-32 border-b border-gray-200">Time</th>
                <th className="py-3 px-6 text-xs font-bold text-gray-500 uppercase tracking-wider w-24 border-b border-gray-200">Type</th>
                <th className="py-3 px-6 text-xs font-bold text-gray-500 uppercase tracking-wider border-b border-gray-200">Message</th>
              </tr>
            </thead>
            <tbody className="divide-y divide-gray-100">
              {events.slice().reverse().map((ev, i) => (
                <tr key={i} className="hover:bg-gray-50 transition-colors group">
                  <td className="py-3 px-6 text-xs font-mono text-gray-500">{new Date(ev.ts * 1000).toLocaleTimeString([], {hour:'2-digit', minute:'2-digit', second:'2-digit'})}</td>
                  <td className="py-3 px-6">
                    <span className={`inline-block px-2 py-0.5 rounded text-[10px] font-bold uppercase ${ev.type === 'error' ? 'bg-red-100 text-red-600' : ev.type === 'warn' ? 'bg-yellow-100 text-yellow-700' : 'bg-blue-50 text-blue-600'}`}>
                      {ev.type || "INFO"}
                    </span>
                  </td>
                  <td className="py-3 px-6 text-sm text-gray-700 font-mono">{ev.message}</td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      </div>
    </div>
  );

  const renderTerminal = () => (
    <div className="h-full p-4 md:p-8 overflow-hidden flex flex-col">
      <div className="bg-black rounded-2xl border border-gray-800 shadow-lg flex-1 flex flex-col overflow-hidden">
        <div className="p-4 border-b border-gray-700 flex justify-between items-center bg-gray-900">
          <div className="flex items-center gap-3">
            <Terminal size={20} className="text-green-400" />
            <h2 className="text-lg font-bold font-display text-white">System Terminal</h2>
          </div>
          <div className="flex gap-2 items-center">
            <div className="w-2 h-2 rounded-full bg-green-400 animate-pulse"></div>
            <span className="text-xs text-gray-400 font-mono">Live Shell</span>
          </div>
        </div>
        <div className="flex-1 overflow-hidden">
          <iframe
            src={terminalUrl}
            className="w-full h-full border-0"
            title="Terminal"
            allow="fullscreen"
          />
        </div>
      </div>
    </div>
  );

  return (
    <div className="flex h-screen bg-gray-50 font-sans text-gray-900 overflow-hidden">
      {renderSidebar()}
      <main className="flex-1 flex flex-col min-w-0 bg-[#f8fafc]">
        <header className="h-16 bg-white border-b border-gray-200 flex items-center justify-between px-4 md:px-8 flex-shrink-0 z-20">
          <div className="flex items-center gap-4">
            <button 
              onClick={() => setSidebarOpen(true)}
              className="lg:hidden p-2 -ml-2 text-gray-600 hover:bg-gray-100 rounded-lg transition-colors"
            >
              <Menu size={24} />
            </button>
            <h1 className="text-xl font-bold font-display text-gray-900 capitalize tracking-tight">{activeTab}</h1>
          </div>
          <div className={`px-4 py-1.5 rounded-full text-sm font-medium transition-all ${studioError ? "bg-red-50 text-red-600 border border-red-100" : "bg-green-50 text-green-600 border border-green-100"} ${!studioStatus && "opacity-0"}`}>
            {studioStatus || "Ready"}
          </div>
        </header>
        <div className="flex-1 overflow-hidden relative">
          <div className={activeTab === "dashboard" ? "h-full" : "hidden"}>
            {renderDashboard()}
          </div>
          <div className={activeTab === "charts" ? "h-full" : "hidden"}>
            <ChartPage currentMetrics={metrics} />
          </div>
          <div className={activeTab === "vision" ? "h-full" : "hidden"}>
            {renderVision()}
          </div>
          <div className={activeTab === "tuning" ? "h-full" : "hidden"}>
            <TuningPage ros={rosRef.current} rosState={rosState} joyState={joyState} publishJoy={publishJoy} sendStatus={sendStatus} />
          </div>
          <div className={activeTab === "walking" ? "h-full" : "hidden"}>
            {renderWalking()}
          </div>
          <div className={activeTab === "teleop" ? "h-full" : "hidden"}>
            {renderTeleop()}
          </div>
          <div className={activeTab === "logs" ? "h-full" : "hidden"}>
            {renderLogs()}
          </div>
          <div className={activeTab === "terminal" ? "h-full" : "hidden"}>
            {renderTerminal()}
          </div>
          <div className={`absolute inset-0 p-4 ${activeTab === "action" ? "" : "hidden"}`}>
             <div className="bg-white rounded-2xl border border-gray-200 shadow-sm h-full overflow-hidden">
               <ActionEditor isActive={activeTab === "action"} rosUrl={rosUrl} />
             </div>
          </div>
        </div>
      </main>
    </div>
  );
}
