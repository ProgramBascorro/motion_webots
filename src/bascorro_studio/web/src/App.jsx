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
  Power,
  Download,
  RefreshCw,
  Save,
  Send,
  Square,
  Trash2,
  Brain,
  Target
} from "lucide-react";
import ActionEditor from "./ActionEditor.jsx";
import WalkingSim from "./WalkingSim.jsx";
import GamepadVisualizer from "./GamepadVisualizer.jsx";
import ChartPage from "./ChartPage.jsx";
import TuningPage from "./TuningPage.jsx";
import RlTrainingPage from "./RlTrainingPage.jsx";
import ActionTrainingPage from "./ActionTrainingPage.jsx";
import KalibrasiBatasGeser from "./KalibrasiBatasGeser.jsx";
import {
  computeDefaultRosbridgeUrl,
  normalizeRosbridgeUrl,
  readRosUrlPreference,
  rewriteLoopbackToCurrentHost,
  writeRosUrlPreference,
} from "./net/rosbridge.js";
import {
  WALKING_BOOL_FIELDS,
  normalizeWalkingParams,
  parseWalkingNumber,
  toWalkingParamMsg,
} from "./walkingParams.js";

const WALKING_VERSION_STORAGE_KEY = "bascorro.walking_versions.v1";
const TELEOP_COMMAND_TOPIC = "/op3_joy_teleop/command";
const TELEOP_STATUS_TOPIC = "/op3_joy_teleop/status";
// Tombol "Init Pose" memutar action page 2. Di robot ini halaman itu bernama
// WALKING_READY: pose berdiri hasil tera operator di action editor, dan acuan
// yang dipakai offset Init Pose milik walking module.
const WALKING_READY_PAGE_NUM = 2;
// Jeda supaya op3_manager sempat memindahkan controller ke action_module
// sebelum nomor halaman dikirim -- pelanggan page_num baru ada setelah pindah,
// jadi pesan yang dikirim terlalu dini hilang tanpa jejak.
const ACTION_MODULE_SETTLE_MS = 600;
const DEFAULT_OVERLAY_TOPIC =
  import.meta.env.VITE_OVERLAY_TOPIC || "/vision/yolo/debug";

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
    artinya: "Tinggi badan dasar saat gait walking dihitung. HARUS LEBIH BESAR DARI 0.",
    fungsi: "Menentukan seberapa jongkok/tinggi postur robot ketika berjalan.",
    tuning: "Diukur di robot ini (period 2,5 s, dsp 0,50, foot 25 mm): z 0,002 memberi ayun lutut 34,2 deg; z 0,010 memberi 45,8 deg; z -0,010 dan -0,025 membuat KAKI TIDAK BERGERAK SAMA SEKALI (ayun 0,1 deg). Mulai dari 0,002.",
    risk: "NILAI NOL ATAU NEGATIF MEMBEKUKAN KAKI. IK memakai ep[2] = z_offset - panjang_kaki, jadi z_offset negatif meminta telapak lebih jauh dari pinggul daripada panjang kaki itu sendiri -- tidak ada solusi, computeLegAngle() gagal, dan kaki menahan pose terakhir tanpa pesan galat yang terlihat. Gejalanya menyamar: gait masih 'jalan' kalau foot_height dibesarkan sampai telapak kembali terjangkau, dan hasilnya ayunan lutut raksasa yang terasa cepat dan kasar."
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
    fungsi: "Menggerbang sensoryFeedback(), yang menulis ke PERSIS 8 sendi kaki: r/l_hip_roll, r/l_knee, r/l_ank_pitch, r/l_ank_roll. Di luar itu tidak disentuh.",
    tuning: "Untuk robot asli biasanya on. Untuk membandingkan efek gain, boleh off sebentar sambil robot dipegang.",
    risk: "Balance off di hardware bisa membuat robot lebih mudah jatuh. Mematikannya TIDAK menghentikan ayunan tangan maupun sway badan -- itu bentuk gait, bukan balance."
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
    fungsi: "Ayunan tangan membantu counterbalance terhadap gerakan kaki. BUKAN parameter IMU: rumusnya di computeArmAngle() murni gait, tanpa gyro sama sekali.",
    tuning: "Naikkan jika badan terlalu kaku. Turunkan jika ayunan tangan justru mengganggu balance. Untuk benar-benar menghentikan ayunan tangan, set nilai ini 0 (atau x_move_amplitude 0) -- mematikan Balance tidak akan menghentikannya.",
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

WALKING_PARAM_HELP.soft_start_time = {
  artinya: "Lama (detik) x move, y move, dan tinggi angkat kaki naik dari nol ke nilai penuh setiap kali robot mulai berjalan dari diam.",
  fungsi: "Langkah pertama tidak langsung penuh: kaki mula-mula menyeret kecil lalu membesar mulus. Skalanya naik terus tiap siklus 8 ms, bukan bertahap per langkah, jadi tidak ada lonjakan kaki. Angle move (belok) tidak ikut.",
  tuning: "0 = mati (langsung penuh). 1-3 detik biasanya enak. Hanya berlaku saat MULAI dari diam -- Start ulang saat sudah berjalan tidak mengecilkan langkah lagi.",
  risk: "Terlalu lama membuat langkah awal lambat terasa 'ragu'. Begitu di-Apply, robot LANGSUNG menyimpannya ke param.yaml sendiri -- tidak perlu Save to Robot, dan tetap berlaku setelah robot mati-nyala. Ikut tersimpan juga di versi walking."
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
      { key: "soft_start_time", label: "Soft start", unit: "s", step: 0.1, ...WALKING_PARAM_HELP.soft_start_time },
    ],
  },
  // Balance dan bentuk gait sengaja DIPISAH. Checkbox "Balance enable" hanya
  // menggerbang sensoryFeedback(), dan fungsi itu menulis ke persis 8 sendi kaki
  // (r/l_hip_roll, r/l_knee, r/l_ank_pitch, r/l_ank_roll). Sebelumnya y_swap,
  // z_swap, arm_swing_gain, pelvis_offset, dan hip_pitch_offset ikut satu grup
  // bernama "Balance", jadi terbaca seolah checkbox itu mengatur semuanya juga.
  {
    title: "Balance (IMU)",
    description: "Hanya ini yang dimatikan checkbox Balance enable. Koreksi gyro ke 8 sendi kaki: hip roll, knee, ankle pitch, ankle roll (kiri+kanan). Cuma jalan saat gait berjalan.",
    fields: [
      { key: "balance_enable", label: "Balance enable", type: "bool", ...WALKING_PARAM_HELP.balance_enable },
      { key: "balance_hip_roll_gain", label: "Hip roll gain", unit: "gain", step: 0.01, ...WALKING_PARAM_HELP.balance_hip_roll_gain },
      { key: "balance_knee_gain", label: "Knee gain", unit: "gain", step: 0.01, ...WALKING_PARAM_HELP.balance_knee_gain },
      { key: "balance_ankle_roll_gain", label: "Ankle roll gain", unit: "gain", step: 0.01, ...WALKING_PARAM_HELP.balance_ankle_roll_gain },
      { key: "balance_ankle_pitch_gain", label: "Ankle pitch gain", unit: "gain", step: 0.01, ...WALKING_PARAM_HELP.balance_ankle_pitch_gain },
    ],
  },
  {
    title: "Bentuk Gait",
    description: "Bagian dari pola langkah itu sendiri, BUKAN IMU. Balance enable tidak mempengaruhi satu pun field di sini -- ayunan lengan misalnya datang dari computeArmAngle(), disetir x_move_amplitude.",
    fields: [
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

// Time to wait after enabling walking_module before sending a walking command,
// so the controller has switched the active module and settled the init pose.
const WALKING_ENABLE_SETTLE_MS = 1200;

// Soft torque-on: enabling torque makes each Dynamixel drive to its goal at full
// speed, so the robot snaps/jerks. To avoid that we first limit the XM430
// profile velocity/acceleration (RAM regs 112/108) so the joints ease into the
// goal, enable torque, then restore full speed (0 = unlimited) so walking/action
// run at normal speed. Velocity unit ~0.229 rev/min, accel unit ~214.6 rev/min^2.
// restoreMs = how long to keep the speed limit before unlocking full speed; sized
// so even a large bring-up move finishes ramping first (slower preset -> longer).
const TORQUE_SPEED_PRESETS = [
  { key: "slow", label: "Lambat", velocity: 15, accel: 6, restoreMs: 5000 },
  { key: "medium", label: "Sedang", velocity: 35, accel: 12, restoreMs: 3000 },
  { key: "fast", label: "Cepat", velocity: 70, accel: 25, restoreMs: 2000 },
];
// robotis_controller batches every /robotis/sync_write_item for a port into ONE
// GroupSyncWrite per ~8ms control cycle, keyed only by port (not the item address),
// so two different items sent in the same cycle collide and the second is dropped.
// Space successive items well past one cycle so each lands in its own batch.
const SYNC_ITEM_GAP_MS = 60;

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

const BUTTON_BASE =
  "inline-flex items-center justify-center gap-2 px-3 py-2 rounded-lg text-xs font-bold transition-colors disabled:opacity-50 disabled:cursor-not-allowed";

const SectionHeader = ({ title, children }) => (
  <div className="flex justify-between items-center mb-4">
    <h2 className="text-lg font-bold font-display text-gray-800">{title}</h2>
    <div className="flex gap-2">{children}</div>
  </div>
);

function resolveInitialRosUrl() {
  if (typeof window === "undefined") {
    return { url: computeDefaultRosbridgeUrl(import.meta.env.VITE_ROSBRIDGE_URL), manual: false };
  }
  return readRosUrlPreference(
    window.localStorage,
    window.location,
    import.meta.env.VITE_ROSBRIDGE_URL
  );
}

export default function App() {
  const initialRosUrl = useMemo(() => resolveInitialRosUrl(), []);
  const [activeTab, setActiveTab] = useState("dashboard");
  const [sidebarOpen, setSidebarOpen] = useState(false);
  const [rosUrl, setRosUrl] = useState(initialRosUrl.url);
  const [rosUrlManual, setRosUrlManual] = useState(initialRosUrl.manual);
  const [rosUrlDraft, setRosUrlDraft] = useState(initialRosUrl.url);
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
  const [torqueSpeedKey, setTorqueSpeedKey] = useState("medium");
  const [walkingParams, setWalkingParams] = useState(() => normalizeWalkingParams());
  const [walkingCurrentParams, setWalkingCurrentParams] = useState(null);
  const [walkingLoaded, setWalkingLoaded] = useState(false);
  const [walkingDirty, setWalkingDirty] = useState(false);
  const [walkingLastAppliedAt, setWalkingLastAppliedAt] = useState(null);
  const [walkingError, setWalkingError] = useState("");
  const [showWalkingAdvanced, setShowWalkingAdvanced] = useState(false);
  const [walkingVersions, setWalkingVersions] = useState(() => readWalkingVersions());
  const [walkingVersionName, setWalkingVersionName] = useState("");
  const [activeWalkingVersionId, setActiveWalkingVersionId] = useState("");
  const [teleopStatus, setTeleopStatus] = useState(null);
  const [teleopStatusAt, setTeleopStatusAt] = useState(null);
  // Kalibrasi yaw lima titik. Seluruh keadaan (rekaman, pemetaan, tabel)
  // datang dari apply_node di robot -- peramban tidak menyimpan apa pun sendiri,
  // supaya membuka Studio dari alamat/perangkat lain melihat rekaman yang sama.
  const [yawCal, setYawCal] = useState(null);
  const [yawCalBusy, setYawCalBusy] = useState(false);
  
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
  const imuCommandPubRef = useRef(null);
  const eventsSubRef = useRef(null);
  const overlaySubRef = useRef(null);
  const joySubRef = useRef(null);
  const joyPubRef = useRef(null);
  const teleopCommandPubRef = useRef(null);
  const teleopStatusSubRef = useRef(null);
  const walkingCommandPubRef = useRef(null);
  const walkingParamPubRef = useRef(null);
  const walkingGetServiceRef = useRef(null);
  // Layanan parameter ROS node /op3_walking_module (untuk soft_start_time).
  const walkingNodeSetParamRef = useRef(null);
  const walkingNodeGetParamRef = useRef(null);
  // Penyimpanan versi walking di ROBOT (apply_node), bukan di localStorage --
  // lihat komentar panjang di apply_node.walking_versions_path().
  const studioRequestPubRef = useRef(null);
  const studioResultSubRef = useRef(null);
  // Menahan satu siklus tulis-balik: daftar yang baru saja DIBACA dari robot
  // tidak perlu langsung dikirim balik ke robot.
  const skipVersionPushRef = useRef(true);
  // Gerbang anti-hapus. TANPA ini ada balapan yang MENGHAPUS daftar di robot:
  // effect penyimpan ikut bergantung pada rosState, jadi saat status berubah jadi
  // "connected" ia langsung mendorong isi walkingVersions -- dan pada peramban
  // yang localStorage-nya kosong isinya [], sehingga berkas di robot ditimpa
  // daftar kosong ~400 ms SEBELUM jawaban walking_versions_get sempat datang.
  // Sekarang: tidak ada dorongan sama sekali sampai satu jawaban get diterima.
  // Kalau apply_node mati, akibatnya cuma "tidak tersimpan di robot", bukan
  // "terhapus dari robot".
  const versionsSyncedRef = useRef(false);
  // Sekali per sambungan: isi tabel Walking dengan parameter yang BENAR-BENAR
  // dipakai robot, bukan WALKING_DEFAULT_PARAMS.
  const walkingAutoLoadedRef = useRef(false);
  // Tracks whether walking_module is the active control module. Walking commands
  // (start/stop/balance) are ignored by op3_walking_module unless it is enabled.
  const walkingModuleEnabledRef = useRef(false);
  const torquePubRef = useRef(null);
  const controlModulePubRef = useRef(null);
  const actionPagePubRef = useRef(null);
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
    writeRosUrlPreference(window.localStorage, rosUrl, rosUrlManual);
  }, [rosUrl, rosUrlManual]);

  useEffect(() => {
    showOverlayRef.current = showOverlay;
  }, [showOverlay]);

  // Setelah ROS di-restart lalu Studio dibuka lagi, tabel Walking dulu selalu
  // menampilkan WALKING_DEFAULT_PARAMS -- terbaca seperti "tuning saya hilang,
  // balik ke default", padahal robotnya menjalankan angka lain dan versi
  // tersimpannya masih utuh. Sekarang tabel diisi sekali dari robot begitu
  // tersambung, lewat jalur Load yang sama, jadi yang tampil adalah yang
  // benar-benar dipakai robot.
  useEffect(() => {
    if (rosState !== "connected") {
      walkingAutoLoadedRef.current = false;
      return;
    }
    if (walkingAutoLoadedRef.current) return;
    walkingAutoLoadedRef.current = true;
    const timer = window.setTimeout(() => loadWalkingParams(), 800);
    return () => window.clearTimeout(timer);
  }, [rosState]);

  useEffect(() => {
    try {
      // Tetap disimpan di peramban, tapi sekarang cuma singgahan supaya tabel
      // sudah terisi sebelum rosbridge tersambung. Yang berkuasa adalah robot.
      localStorage.setItem(WALKING_VERSION_STORAGE_KEY, JSON.stringify(walkingVersions));
    } catch {
      // Browser storage can fail in private or restricted contexts.
    }

    // Belum pernah membaca daftar dari robot -> jangan menulis apa pun ke sana.
    if (!versionsSyncedRef.current) return;
    // Jangan kirim balik daftar yang baru saja dibaca DARI robot.
    if (skipVersionPushRef.current) {
      skipVersionPushRef.current = false;
      return;
    }
    if (!studioRequestPubRef.current) return;
    studioRequestPubRef.current.publish(
      new ROSLIB.Message({
        data: JSON.stringify({ action: "walking_versions_put", versions: walkingVersions }),
      })
    );
    // rosState SENGAJA tidak masuk deps: perubahan status sambungan saja tidak
    // boleh memicu penulisan. Hanya perubahan daftar yang boleh.
  }, [walkingVersions]);

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
    controlModulePubRef.current = new ROSLIB.Topic({
      ros,
      name: "/robotis/enable_ctrl_module",
      messageType: "std_msgs/String",
    });
    actionPagePubRef.current = new ROSLIB.Topic({
      ros,
      name: "/robotis/action/page_num",
      messageType: "std_msgs/Int32",
    });
    imuCommandPubRef.current = new ROSLIB.Topic({
      ros,
      name: "/robotis/open_cr/imu_command",
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
    // Sambungan baru: lupakan status sinkron sambungan sebelumnya.
    versionsSyncedRef.current = false;
    studioRequestPubRef.current = new ROSLIB.Topic({
      ros,
      name: "/bascorro_studio/request",
      messageType: "std_msgs/String",
    });
    studioResultSubRef.current = new ROSLIB.Topic({
      ros,
      name: "/bascorro_studio/result",
      messageType: "std_msgs/String",
    });
    studioResultSubRef.current.subscribe((msg) => {
      let payload;
      try {
        payload = JSON.parse(msg.data);
      } catch {
        return;
      }
      if (typeof payload?.action === "string" && payload.action.startsWith("yaw_calib_")) {
        setYawCalBusy(false);
        // Balasan SELALU membawa keadaan lengkap, termasuk saat ok=false, jadi
        // tabel tetap tersegarkan walau satu langkah ditolak.
        if (Array.isArray(payload.table)) setYawCal(payload);
        if (payload.message) sendStatus(payload.message, payload.ok === false);
        else if (payload.ok === false) sendStatus("Kalibrasi yaw gagal", true);
        return;
      }
      if (payload?.action !== "walking_versions_get") return;
      if (payload.ok !== true || !Array.isArray(payload.versions)) return;
      const incoming = payload.versions
        .filter((item) => item?.id && item?.params)
        .map((item) => ({
          id: String(item.id),
          name: String(item.name || "Untitled"),
          createdAt: Number(item.createdAt || Date.now()),
          updatedAt: Number(item.updatedAt || item.createdAt || Date.now()),
          params: normalizeWalkingParams(item.params),
        }));
      // DIGABUNG, bukan diganti. Saat pertama kali dipakai, berkas di robot masih
      // kosong sementara versi lama pengguna ada di localStorage peramban ini.
      // Kalau daftar robot langsung menimpa, versi lama itu terhapus dari layar
      // DAN dari localStorage (effect di bawah ikut menulis) -- persis kehilangan
      // yang sedang diperbaiki. Jadi: yang ada di robot dipakai, yang cuma ada di
      // peramban ini dipertahankan lalu didorong naik ke robot.
      setWalkingVersions((prev) => {
        const known = new Set(incoming.map((item) => item.id));
        const localOnly = prev.filter((item) => !known.has(item.id));
        // Tidak ada yang baru dari sisi peramban -> tidak perlu tulis balik.
        skipVersionPushRef.current = localOnly.length === 0;
        return [...incoming, ...localOnly];
      });
      // Sejak titik ini isi robot sudah diketahui, jadi mendorong aman.
      versionsSyncedRef.current = true;
    });
    // Tarik daftar versi begitu tersambung.
    window.setTimeout(() => {
      studioRequestPubRef.current?.publish(
        new ROSLIB.Message({ data: JSON.stringify({ action: "walking_versions_get" }) })
      );
      studioRequestPubRef.current?.publish(
        new ROSLIB.Message({ data: JSON.stringify({ action: "yaw_calib_get" }) })
      );
    }, 400);

    walkingNodeSetParamRef.current = new ROSLIB.Service({
      ros,
      name: "/op3_walking_module/set_parameters",
      serviceType: "rcl_interfaces/srv/SetParameters",
    });
    walkingNodeGetParamRef.current = new ROSLIB.Service({
      ros,
      name: "/op3_walking_module/get_parameters",
      serviceType: "rcl_interfaces/srv/GetParameters",
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
      imuCommandPubRef.current = null;
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
    setRosUrlManual(true);
    setRosUrlDraft(next);
    sendStatus(`ROS bridge: ${next}`);
  };

  const resetRosUrlAuto = () => {
    const next = computeDefaultRosbridgeUrl(import.meta.env.VITE_ROSBRIDGE_URL, window.location);
    setRosUrl(next);
    setRosUrlManual(false);
    setRosUrlDraft(next);
    sendStatus(`ROS bridge reset: ${next}`);
  };

  const sendWalkingCommand = (command, label) => {
    if (!walkingCommandPubRef.current || rosState !== "connected") {
      sendStatus("Walking command unavailable", true);
      return false;
    }
    walkingCommandPubRef.current.publish(new ROSLIB.Message({ data: command }));
    sendStatus(label || `Walking command: ${command}`);
    return true;
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

  // soft_start_time bukan bagian GetWalkingParam, jadi dibaca terpisah dari
  // parameter node. Kalau node lama belum punya parameter ini, callback menerima
  // null dan pemanggil memakai nilai tabel yang ada.
  const readSoftStartTime = (onDone) => {
    if (!walkingNodeGetParamRef.current) { onDone(null); return; }
    walkingNodeGetParamRef.current.callService(
      new ROSLIB.ServiceRequest({ names: ["soft_start_time"] }),
      (res) => {
        const v = res?.values?.[0];
        const num = v?.type === PARAM_TYPES.double ? v.double_value
          : v?.type === PARAM_TYPES.int ? v.integer_value : null;
        onDone(Number.isFinite(num) ? num : null);
      },
      () => onDone(null)
    );
  };

  const writeSoftStartTime = (seconds) => {
    if (!walkingNodeSetParamRef.current) return;
    walkingNodeSetParamRef.current.callService(
      new ROSLIB.ServiceRequest({
        parameters: [{ name: "soft_start_time", value: { type: PARAM_TYPES.double, double_value: Number(seconds) } }],
      }),
      (res) => {
        const r = res?.results?.[0];
        if (r && r.successful === false) {
          setWalkingError(`Soft start ditolak robot: ${r.reason || "tanpa alasan"}`);
          sendStatus(`Soft start ditolak: ${r.reason || ""}`, true);
        }
      },
      (err) => sendStatus(`Soft start tidak terkirim: ${err?.message || err}`, true)
    );
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
        readSoftStartTime((softStart) => {
          const normalized = normalizeWalkingParams({
            ...res.parameters,
            soft_start_time: softStart ?? walkingParams.soft_start_time,
          });
          setWalkingParams(normalized);
          setWalkingCurrentParams(normalized);
          setWalkingLoaded(true);
          setWalkingDirty(false);
          sendStatus("Walking params loaded");
        });
      },
      (err) => {
        const message = err?.message || "Failed to load walking params";
        setWalkingError(message);
        sendStatus(message, true);
      }
    );
  };

  // Re-read the live params from the robot and refresh ONLY the "current"
  // column. It deliberately does not touch `walkingParams`, so it is safe to
  // call while the operator is typing in the grid. Used as proof-of-effect
  // after a balance on/off command.
  const refreshWalkingCurrent = (label) => {
    if (!walkingGetServiceRef.current || rosState !== "connected") return;
    walkingGetServiceRef.current.callService(
      new ROSLIB.ServiceRequest({ get_param: true }),
      (res) => {
        if (!res?.parameters) return;
        readSoftStartTime((softStart) => {
          const normalized = normalizeWalkingParams({
            ...res.parameters,
            soft_start_time: softStart ?? walkingParams.soft_start_time,
          });
          setWalkingCurrentParams(normalized);
          setWalkingLoaded(true);
          if (label) sendStatus(`${label} (robot: balance_enable=${normalized.balance_enable})`);
        });
      },
      () => {}
    );
  };

  // walkingParameterCallback() in op3_walking_module does `walking_param_ = *msg`,
  // overwriting the WHOLE struct. So a "balance off" command (which only flips one
  // bool on the robot) gets undone by the next Apply if the grid checkbox is still
  // checked. Keep the UI state in sync with the command, then read back from the
  // robot as proof it actually landed.
  const sendImuCommand = (command, label) => {
    if (!imuCommandPubRef.current || rosState !== "connected") {
      sendStatus("IMU command publisher unavailable", true);
      return;
    }
    imuCommandPubRef.current.publish(new ROSLIB.Message({ data: command }));
    sendStatus(label);
  };

  // Satu pintu ke apply_node untuk semua langkah kalibrasi yaw. Balasannya
  // ditangani di pelanggan /bascorro_studio/result di atas.
  const sendYawCalib = (action, extra = {}) => {
    if (!studioRequestPubRef.current || rosState !== "connected") {
      sendStatus("Robot belum tersambung", true);
      return;
    }
    setYawCalBusy(true);
    studioRequestPubRef.current.publish(
      new ROSLIB.Message({ data: JSON.stringify({ action, ...extra }) })
    );
    // Jangan menggantung selamanya kalau apply_node mati.
    window.setTimeout(() => setYawCalBusy(false), 4000);
  };

  const sendBalanceCommand = (enable) => {
    const ok = sendWalkingCommand(enable ? "balance on" : "balance off");
    if (!ok) return;
    setWalkingParams((prev) => ({ ...prev, balance_enable: enable }));
    window.setTimeout(() => {
      refreshWalkingCurrent(enable ? "Balance on" : "Balance off");
    }, 200);
  };

  const applyWalkingParams = () => {
    if (!walkingParamPubRef.current || rosState !== "connected") {
      const message = "Walking param publisher unavailable";
      setWalkingError(message);
      sendStatus(message, true);
      return false;
    }

    const payload = normalizeWalkingParams(walkingParams);
    // soft_start_time dibuang dari pesan (bukan field WalkingParam) dan dikirim
    // sebagai parameter node -- lihat WALKING_NODE_PARAM_FIELDS.
    walkingParamPubRef.current.publish(new ROSLIB.Message(toWalkingParamMsg(payload)));
    writeSoftStartTime(payload.soft_start_time);
    setWalkingParams(payload);
    setWalkingCurrentParams(payload);
    setWalkingLoaded(true);
    setWalkingDirty(false);
    setWalkingLastAppliedAt(Date.now());
    setWalkingError("");
    sendStatus("Walking params applied");
    return true;
  };

  // Apply yang benar-benar MEMINDAHKAN robot ke stance init-nya, tanpa Start.
  //
  // applyWalkingParams() sendiri cuma menerbitkan ke /robotis/walking/set_params.
  // walkingParameterCallback() memang selalu menerima (`walking_param_ = *msg`,
  // tanpa syarat, di thread spin modul sendiri), tapi yang menggerakkan kaki
  // adalah process() -- dan process() hanya dipanggil controller kalau
  // walking_module sedang memegang sendi. Kalau modulnya belum aktif, Apply
  // diam-diam tidak berefek apa-apa sampai ada yang menekan Start atau Enable
  // Module, dan itulah keluhannya: parameter Init Pose baru kelihatan setelah
  // Start.
  //
  // Urutannya PARAMETER DULU, BARU MODUL, bukan sebaliknya. onModuleEnable()
  // memasang capture_init_pose_, lalu siklus berikutnya menuju stance netral
  // dari offset yang sedang dipegang modul. Kalau modul diaktifkan lebih dulu,
  // robot bergerak dua kali: ke stance offset LAMA, lalu ke yang baru setelah
  // parameter menyusul. Dengan urutan ini modul sudah memegang angka baru
  // sebelum siklus pertamanya, jadi satu gerakan saja -- tetap dilandaikan
  // iniPoseTraGene (minimum-jerk, minimal 1,5 detik).
  const applyWalkingParamsNow = () => {
    if (!applyWalkingParams()) return false;
    if (walkingModuleEnabledRef.current) return true;

    // Beri jeda kecil supaya pesan parameter sudah sampai ke modul sebelum
    // controller menyerahkan sendi kepadanya.
    window.setTimeout(() => {
      ensureWalkingModuleEnabled();
      sendStatus("Walking module diaktifkan — robot bergerak ke posisi init");
    }, 200);
    return true;
  };

  // op3_walking_module ignores start/stop/balance commands unless walking_module
  // is the active control module ("walking module is not ready."). Switching to
  // Init Pose / Head module / Action deactivates it, so make sure it is enabled
  // before sending walking commands. Returns true if it was *just* enabled (caller
  // should wait for the module switch to settle before sending the command).
  const ensureWalkingModuleEnabled = () => {
    if (walkingModuleEnabledRef.current) return false;
    if (!controlModulePubRef.current || rosState !== "connected") {
      sendStatus("Walking module command unavailable", true);
      return false;
    }
    // make sure the soft torque-on speed limit is lifted before the gait runs
    restoreFullSpeed();
    controlModulePubRef.current.publish(new ROSLIB.Message({ data: "walking_module" }));
    walkingModuleEnabledRef.current = true;
    sendStatus("Walking module enabled");
    return true;
  };

  const startWalking = () => {
    const justEnabled = ensureWalkingModuleEnabled();
    window.setTimeout(() => {
      sendWalkingCommand("start");
    }, justEnabled ? WALKING_ENABLE_SETTLE_MS : 0);
  };

  const applyWalkingAndStart = () => {
    const justEnabled = ensureWalkingModuleEnabled();
    window.setTimeout(() => {
      if (applyWalkingParams()) {
        window.setTimeout(() => {
          sendWalkingCommand("start", "Walking params applied and started");
        }, 200);
      }
    }, justEnabled ? WALKING_ENABLE_SETTLE_MS : 0);
  };

  const enableWalkingModule = () => {
    if (!controlModulePubRef.current || rosState !== "connected") {
      sendStatus("Walking module command unavailable", true);
      return;
    }
    controlModulePubRef.current.publish(new ROSLIB.Message({ data: "walking_module" }));
    walkingModuleEnabledRef.current = true;
    sendStatus("Walking module enabled");
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
    // Tombol ini memutar action page 2 (WALKING_READY) apa adanya. Perpindahan
    // ke walking module adalah hal lain, dipicu Apply & Start -- stance neutral
    // yang dihitung walking module dari offset Init Pose sengaja ditera supaya
    // sama dengan geometri WALKING_READY, jadi serah-terimanya mulus.
    if (!controlModulePubRef.current || !actionPagePubRef.current || rosState !== "connected") {
      sendStatus("Init Pose: action module tidak tersedia", true);
      return;
    }
    walkingModuleEnabledRef.current = false;
    controlModulePubRef.current.publish(new ROSLIB.Message({ data: "action_module" }));
    sendStatus(`Init Pose: pindah ke action_module -> page ${WALKING_READY_PAGE_NUM}...`);
    window.setTimeout(() => {
      actionPagePubRef.current?.publish(new ROSLIB.Message({ data: WALKING_READY_PAGE_NUM }));
      sendStatus(`Init Pose: memutar page ${WALKING_READY_PAGE_NUM} (WALKING_READY)`);
    }, ACTION_MODULE_SETTLE_MS);
  };

  const handleSoftStop = () => {
    sendWalkingCommand("stop", "Soft Stop Sent");
  };

  // Write a single control-table item (same value) to every joint via
  // /robotis/sync_write_item. Returns false if no joints are known yet.
  const writeSyncItem = (itemName, value) => {
    if (!torquePubRef.current || !metrics?.joint_names?.length) return false;
    torquePubRef.current.publish(new ROSLIB.Message({
      item_name: itemName,
      joint_name: metrics.joint_names,
      value: metrics.joint_names.map(() => value),
    }));
    return true;
  };

  // Send a list of [item, value] writes one per control cycle. The controller drops
  // multiple different items sent in the same cycle (see SYNC_ITEM_GAP_MS), so the
  // profile-velocity / acceleration / torque writes must NOT be sent back-to-back.
  const writeSyncItemsSequential = (items, onDone) => {
    let i = 0;
    const step = () => {
      if (i >= items.length) {
        if (onDone) onDone();
        return;
      }
      const [name, value] = items[i++];
      writeSyncItem(name, value);
      window.setTimeout(step, SYNC_ITEM_GAP_MS);
    };
    step();
  };

  // Lift the soft torque-on speed limit (0 = unlimited) so module-driven motion
  // (walking, action) is not slowed down.
  const restoreFullSpeed = () => {
    writeSyncItemsSequential([
      ["profile_velocity", 0],
      ["profile_acceleration", 0],
    ]);
  };

  const handleTorque = (enable) => {
    if (!torquePubRef.current || !metrics?.joint_names?.length) return;

    if (!enable) {
      writeSyncItem("torque_enable", 0);
      sendStatus("Torque OFF");
      return;
    }

    // Soft bring-up, each item in its own control cycle:
    //   profile_velocity -> profile_acceleration -> torque_enable
    // so the joints ease into their goal instead of snapping. Then restore full
    // speed so walking/action are not slowed.
    const preset = TORQUE_SPEED_PRESETS.find((p) => p.key === torqueSpeedKey) || TORQUE_SPEED_PRESETS[1];
    writeSyncItemsSequential(
      [
        ["profile_velocity", preset.velocity],
        ["profile_acceleration", preset.accel],
        ["torque_enable", 1],
      ],
      () => {
        sendStatus(`Torque ON (${preset.label})`);
        window.setTimeout(restoreFullSpeed, preset.restoreMs);
      },
    );
  };

  const handleEnableHeadModule = () => {
    if (controlModulePubRef.current) {
      walkingModuleEnabledRef.current = false;
      controlModulePubRef.current.publish(new ROSLIB.Message({ data: "head_control_module" }));
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
  const mag = metrics?.mag || null;

  // Warna kartu magnetometer mengikuti status firmware, bukan sekadar ada/tidak
  // ada data: "settling" itu normal selama 3 detik pertama setelah OpenCR menyala
  // dan tidak boleh terbaca sebagai kerusakan.
  const magState = mag?.stale ? "stale" : mag?.state || "no_data";
  const MAG_STATE_STYLE = {
    ok: { box: "bg-green-50 text-green-600", note: "Yaw kompas valid" },
    settling: { box: "bg-yellow-50 text-yellow-700", note: "Mengunci referensi 0 derajat, tunggu ~3 detik" },
    calibrating: { box: "bg-blue-50 text-blue-700", note: "Putar robot pelan 1 putaran penuh (20 detik)" },
    no_sensor: { box: "bg-red-100 text-red-600", note: "Papan tidak mengirim data magnetometer" },
    stale: { box: "bg-red-100 text-red-600", note: "Topik mag_status berhenti mengalir" },
    no_data: { box: "bg-gray-100 text-gray-500", note: "Menunggu data dari open_cr_module" },
  };
  const magStyle = MAG_STATE_STYLE[magState] || MAG_STATE_STYLE.no_data;
  const torque = metrics?.torque || {};
  const healthOk = healthResult?.ok || [];
  const healthFailed = healthResult?.failed || [];
  const healthErrors = healthResult?.errors || {};
  const healthTotal = healthResult?.total ?? (healthOk.length + healthFailed.length);
  const healthDuration = healthResult?.duration_ms;
  const healthSkipped = healthResult?.skipped;
  const healthRaw = healthResult?.raw;
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
            className={`flex items-center gap-3 px-4 py-3 rounded-lg font-medium transition-all ${activeTab === "rl" ? "bg-undip-blue text-white shadow-md border border-accent-yellow/20" : "hover:bg-white/5 hover:text-white"}`}
            onClick={() => { setActiveTab("rl"); setSidebarOpen(false); }}
          >
            <Brain size={20} className={activeTab === "rl" ? "text-accent-yellow" : ""} />
            <span>RL Training</span>
          </button>
          <button
            className={`flex items-center gap-3 px-4 py-3 rounded-lg font-medium transition-all ${activeTab === "actrain" ? "bg-undip-blue text-white shadow-md border border-accent-yellow/20" : "hover:bg-white/5 hover:text-white"}`}
            onClick={() => { setActiveTab("actrain"); setSidebarOpen(false); }}
          >
            <Target size={20} className={activeTab === "actrain" ? "text-accent-yellow" : ""} />
            <span>Action Training</span>
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
    const walkingButtonBase = BUTTON_BASE;
    const currentValue = (field) => {
      if (!walkingCurrentParams) return "not loaded";
      return formatWalkingValue(walkingCurrentParams[field.key], field);
    };
    const isFieldDirty = (field) => {
      if (!walkingCurrentParams) return walkingLoaded;
      if (field.type === "bool") return Boolean(walkingParams[field.key]) !== Boolean(walkingCurrentParams[field.key]);
      return parseWalkingNumber(walkingParams[field.key], NaN) !== parseWalkingNumber(walkingCurrentParams[field.key], NaN);
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
        <div className="max-w-7xl mx-auto flex flex-col gap-6">
          <div className="bg-white rounded-2xl border border-gray-200 shadow-sm p-5">
            {/* Badge dibuat sama seperti ALPHONSE: kata Indonesia + tooltip, dan
                waktu Apply terakhir ikut jadi badge di baris yang sama daripada
                baris teks tersendiri. "Dirty/Synced" tidak pernah menjelaskan
                dirty terhadap apa. */}
            <div className="flex flex-wrap items-center gap-2">
              <h2 className="text-lg font-bold font-display text-gray-900">Walking</h2>
              <span
                className={`px-2 py-0.5 rounded-full text-[10px] font-bold uppercase ${walkingDirty ? "bg-yellow-100 text-yellow-700" : "bg-green-50 text-green-700"}`}
                title={walkingDirty ? "Ada perubahan di tabel yang belum di-Apply ke robot" : "Isi tabel sudah sama dengan yang dipakai robot"}
              >
                {walkingDirty ? "Belum di-Apply" : "Tersinkron"}
              </span>
              <span
                className={`px-2 py-0.5 rounded-full text-[10px] font-bold uppercase ${walkingLoaded ? "bg-blue-50 text-undip-blue" : "bg-gray-100 text-gray-500"}`}
                title={walkingLoaded ? "Angka diambil dari robot lewat Load/Apply" : "Belum pernah Load \u2014 masih nilai default dari walkingParams.js"}
              >
                {walkingLoaded ? "Runtime" : "Default"}
              </span>
              {walkingLastAppliedAt && (
                <span className="px-2 py-0.5 rounded-full bg-gray-100 text-[10px] font-mono text-gray-500" title="Waktu Apply terakhir dari browser ini">
                  Apply: {new Date(walkingLastAppliedAt).toLocaleTimeString()}
                </span>
              )}
            </div>
            <p className="mt-1 text-sm text-gray-500">
              Alur: atur parameter → <b>Apply</b> (robot langsung ke posisi init, modul diaktifkan otomatis) → <b>Start</b> kalau mau berjalan. Satuan runtime: detik, meter, radian.
            </p>
            {walkingError && (
              <p className="mt-2 text-xs text-red-600 font-mono">{walkingError}</p>
            )}

            {/* PARAMETER row */}
            <div className="mt-4 flex flex-wrap items-center gap-2">
              <span className="w-20 shrink-0 text-[10px] font-bold uppercase tracking-wide text-gray-400">Parameter</span>
              <button title="Aktifkan walking_module dulu, supaya parameter & perintah jalan diterima robot." className={`${walkingButtonBase} bg-undip-blue text-white hover:bg-opacity-90`} onClick={enableWalkingModule} disabled={!rosConnected}>
                <Power size={14} /> Enable Module
              </button>
              <button title="Baca parameter yang sedang aktif di robot ke tabel." className={`${walkingButtonBase} bg-gray-100 text-gray-700 hover:bg-gray-200`} onClick={loadWalkingParams} disabled={!rosConnected}>
                <Download size={14} /> Load
              </button>
              <button title="Kirim semua parameter ke robot DAN langsung bawa robot ke posisi init-nya. Walking module diaktifkan otomatis kalau belum, jadi tidak perlu Start dulu untuk melihat offset Init Pose." className={`${walkingButtonBase} bg-blue-50 text-undip-blue border border-blue-100 hover:bg-blue-100`} onClick={applyWalkingParamsNow} disabled={!rosConnected}>
                <Send size={14} /> Apply
              </button>
              <button title="Kirim parameter lalu langsung mulai berjalan." className={`${walkingButtonBase} bg-green-600 text-white hover:bg-green-700`} onClick={applyWalkingAndStart} disabled={!rosConnected}>
                <Play size={14} /> Apply &amp; Start
              </button>
              {showWalkingAdvanced && (
                <button title="Kirim parameter lalu segarkan baseline teleop joystick agar ikut gaya jalan yang sama." className={`${walkingButtonBase} bg-white text-gray-700 border border-gray-200 hover:bg-gray-100`} onClick={applyWalkingAndRefreshTeleop} disabled={!rosConnected}>
                  <RefreshCw size={14} /> + Teleop
                </button>
              )}
              <button onClick={() => setShowWalkingAdvanced((v) => !v)} className="ml-auto shrink-0 text-xs font-bold text-undip-blue hover:underline">
                {showWalkingAdvanced ? "− Advanced" : "+ Advanced"}
              </button>
            </div>

            {/* CONTROL row */}
            <div className="mt-2 flex flex-wrap items-center gap-2">
              <span className="w-20 shrink-0 text-[10px] font-bold uppercase tracking-wide text-gray-400">Control</span>
              <button title="Mulai gait berjalan (pastikan modul aktif & parameter sudah dikirim)." className={`${walkingButtonBase} bg-green-50 text-green-700 border border-green-100 hover:bg-green-100`} onClick={startWalking} disabled={!rosConnected}>
                <Play size={14} /> Start
              </button>
              <button title="Hentikan jalan dengan halus." className={`${walkingButtonBase} bg-red-50 text-red-600 border border-red-100 hover:bg-red-100`} onClick={() => sendWalkingCommand("stop")} disabled={!rosConnected}>
                <Square size={14} /> Stop
              </button>
              <button title="Aktifkan balance control (koreksi pakai IMU saat berjalan)." className={`${walkingButtonBase} bg-gray-50 text-gray-700 border border-gray-200 hover:bg-gray-100`} onClick={() => sendBalanceCommand(true)} disabled={!rosConnected}>
                Balance On
              </button>
              <button title="Matikan balance control." className={`${walkingButtonBase} bg-gray-50 text-gray-700 border border-gray-200 hover:bg-gray-100`} onClick={() => sendBalanceCommand(false)} disabled={!rosConnected}>
                Balance Off
              </button>
              <div className="h-5 w-px bg-gray-300 mx-1" />
              <button title="Simpan parameter ke file di robot supaya tetap dipakai setelah restart." className={`${walkingButtonBase} bg-yellow-50 text-yellow-700 border border-yellow-100 hover:bg-yellow-100`} onClick={() => sendWalkingCommand("save", "Walking params save command sent")} disabled={!rosConnected}>
                <Save size={14} /> Save to Robot
              </button>
            </div>
          </div>

          {/* Kalibrasi batas geser kiper D58 (2026-11-12): perekamnya
              scripts/calib_batas_geser.py layan; panel ini tidak menggerakkan robot. */}
          <KalibrasiBatasGeser ros={rosRef.current} rosState={rosState} />

          {/* PRESET CEPAT -- dirapatkan seperti ALPHONSE: satu kartu berisi tiga
              tombol kecil, bukan tiga kartu besar selebar layar. Urutannya juga
              dibalik supaya preset (yang mengisi tabel) datang sebelum Versi
              Tersimpan (yang juga mengisi tabel), bukan sesudahnya. */}
          <div className="bg-white rounded-2xl border border-gray-200 shadow-sm p-4">
            <div className="flex flex-wrap items-center gap-2 mb-3">
              <h3 className="text-sm font-bold text-gray-800">Preset Cepat</h3>
              <span className="text-[10px] text-gray-400">Klik untuk mengisi tabel dengan nilai preset — belum dikirim ke robot, tetap perlu Apply</span>
            </div>
            <div className="grid grid-cols-1 sm:grid-cols-3 gap-2">
              <button
                className="text-left bg-gray-50 border border-gray-200 rounded-lg p-3 hover:border-undip-blue/40 hover:bg-white transition-all"
                onClick={() => applyWalkingPreset("tiny")}
                title="Langkah uji paling aman: pelan dan sangat pendek"
              >
                <div className="text-xs font-bold text-gray-900">Tiny Test</div>
                <div className="mt-0.5 text-[11px] text-gray-500">Period 0.85s, maju 3mm, balance on.</div>
              </button>
              <button
                className="text-left bg-gray-50 border border-gray-200 rounded-lg p-3 hover:border-undip-blue/40 hover:bg-white transition-all"
                onClick={() => applyWalkingPreset("stop")}
                title="Nolkan amplitudo gerak tanpa mengubah postur dan gain"
              >
                <div className="text-xs font-bold text-gray-900">Stop Motion</div>
                <div className="mt-0.5 text-[11px] text-gray-500">x/y/turn = 0, postur &amp; gain tetap.</div>
              </button>
              <button
                className="text-left bg-gray-50 border border-gray-200 rounded-lg p-3 hover:border-undip-blue/40 hover:bg-white transition-all"
                onClick={() => applyWalkingPreset("balance")}
                title="Kembalikan gain balance ke nilai repo"
              >
                <div className="text-xs font-bold text-gray-900">Balance Defaults</div>
                <div className="mt-0.5 text-[11px] text-gray-500">Hip 0.35, knee 0.4, ankle 0.7/0.9.</div>
              </button>
            </div>
          </div>

          {/* VERSI TERSIMPAN -- satu baris ringkas, bukan kartu tinggi. */}
          <div className="bg-white rounded-2xl border border-gray-200 shadow-sm p-4">
            <div className="flex flex-wrap items-center gap-2 mb-3">
              <h3 className="text-sm font-bold text-gray-800">Versi Tersimpan</h3>
              <span
                className="rounded-full bg-gray-100 px-2 py-0.5 text-[10px] font-semibold text-gray-500"
                title="Disimpan di robot (walking_versions.json di samping berkas .bin action), bukan di peramban. Jadi versi yang sama muncul dari alamat, perangkat, atau peramban mana pun."
              >
                Tersimpan di robot
              </span>
              <span className="text-[10px] text-gray-400">Load hanya mengisi tabel — tetap perlu Apply untuk kirim ke robot</span>
            </div>
            <div className="grid grid-cols-1 sm:grid-cols-[1fr_1fr_auto_auto_auto] gap-2">
              <input
                type="text"
                value={walkingVersionName}
                onChange={(e) => setWalkingVersionName(e.target.value)}
                className="min-w-0 rounded-lg border border-gray-200 bg-gray-50 px-3 py-2 text-sm text-gray-900 focus:border-undip-blue focus:outline-none focus:ring-2 focus:ring-undip-blue/20"
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
                className="min-w-0 rounded-lg border border-gray-200 bg-gray-50 px-3 py-2 text-sm text-gray-900 focus:border-undip-blue focus:outline-none focus:ring-2 focus:ring-undip-blue/20"
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
                title="Simpan isi tabel saat ini sebagai versi baru di browser"
              >
                <Save size={14} /> Save
              </button>
              <button
                className={`${walkingButtonBase} bg-gray-100 text-gray-700 hover:bg-gray-200`}
                onClick={loadWalkingVersion}
                disabled={!selectedWalkingVersion}
                title="Isi tabel dengan parameter dari versi terpilih"
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

          <WalkingSim rosUrl={rosUrl} active={activeTab === "walking"} />

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
                          {field.key === "init_z_offset" &&
                            parseWalkingNumber(walkingParams.init_z_offset, 1) <= 0 && (
                            // Diam-diam mematikan gait, jadi harus terlihat di layar --
                            // bukan cuma di tooltip yang perlu di-hover.
                            <div className="mt-1 rounded bg-red-50 px-2 py-1 text-[11px] font-bold text-red-700">
                              z_offset ≤ 0 membekukan kaki: IK tidak punya solusi. Pakai 0,002 atau lebih.
                            </div>
                          )}
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
                            type="text"
                            inputMode="decimal"
                            value={walkingParams[field.key]}
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
          <div className="flex items-center gap-1.5 w-full sm:w-auto" title="Kecepatan saat Torque ON (mencegah robot menyentak)">
            <span className="text-xs text-gray-400 hidden sm:inline">Speed</span>
            {TORQUE_SPEED_PRESETS.map((p) => (
              <button
                key={p.key}
                className={`flex-1 sm:flex-none px-2.5 py-2 rounded-lg text-xs font-medium border transition-all ${torqueSpeedKey === p.key ? "bg-undip-blue text-white border-undip-blue shadow-sm" : "border-gray-200 text-gray-500 hover:bg-gray-50"}`}
                onClick={() => setTorqueSpeedKey(p.key)}
              >
                {p.label}
              </button>
            ))}
          </div>
          <div className="flex gap-2 w-full sm:w-auto">
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

        {/* Magnetometer Card */}
        <div className="bg-white p-6 rounded-2xl border border-gray-200 shadow-sm">
          <SectionHeader title="Magnetometer / Yaw" />
          <div className="flex flex-col gap-4">
            <div className="flex justify-between items-center py-2 border-b border-gray-50">
              <span className="text-sm text-gray-500" title="Arah kompas absolut, 0-360 derajat. Tidak bergantung arah robot saat dinyalakan.">Heading</span>
              <strong className="font-mono text-gray-900">{formatNumber(mag?.heading_deg, 1)}°</strong>
            </div>
            <div className="flex justify-between items-center py-2 border-b border-gray-50">
              <span className="text-sm text-gray-500" title="Yaw dari magnetometer, relatif arah robot saat OpenCR dinyalakan. Inilah yaw yang dipakai /robotis/open_cr/imu.">Yaw (mag)</span>
              <strong className="font-mono text-gray-900">{formatNumber(mag?.yaw_mag_deg, 1)}°</strong>
            </div>
            <div className="flex justify-between items-center py-2 border-b border-gray-50">
              <span className="text-sm text-gray-500" title="Yaw lama hasil integrasi gyro. Disimpan firmware sebagai pembanding.">Yaw (gyro)</span>
              <strong className="font-mono text-gray-900">{formatNumber(mag?.yaw_gyro_deg, 1)}°</strong>
            </div>
            <div className="flex justify-between items-center py-2 border-b border-gray-50">
              <span className="text-sm text-gray-500" title="Selisih yaw kompas terhadap yaw gyro. Angka yang terus membesar = drift gyro, dan itu yang dikoreksi magnetometer.">Drift gyro</span>
              <strong className="font-mono text-gray-900">{formatNumber(mag?.yaw_drift_deg, 1)}°</strong>
            </div>
            <div className="flex justify-between items-center py-2 border-b border-gray-50">
              <span className="text-sm text-gray-500" title="Kuat medan magnet total. Nilai wajar di dalam ruangan sekitar 25-65 uT. Jauh di luar itu = ada besi/magnet dekat sensor, kalibrasi tidak akan menolong.">Field</span>
              <strong className="font-mono text-gray-900">{formatNumber(mag?.norm_ut, 1)} µT</strong>
            </div>
            <div className="grid grid-cols-3 gap-2 text-center">
              {["x", "y", "z"].map((axis) => (
                <div key={axis} className="bg-gray-50 rounded-lg py-2">
                  <div className="text-[10px] uppercase tracking-wider text-gray-400">{axis}</div>
                  <div className="font-mono text-xs text-gray-800">{formatNumber(mag?.[`${axis}_ut`], 1)}</div>
                </div>
              ))}
            </div>

            <div className={`text-center p-2 rounded-lg text-xs font-bold uppercase tracking-wider ${magStyle.box}`}>
              {magState}
            </div>
            <p className="text-[11px] text-gray-500 text-center -mt-2">{magStyle.note}</p>

            <div className="grid grid-cols-2 gap-2">
              <button
                title="Darurat: nol-kan yaw di arah robot saat ini. Normalnya tidak perlu -- yaw sudah nol menghadap ke arah robot saat dinyalakan. Menekan ini membuang geser tetap yang mengoreksi medan servo, jadi pakai hanya kalau robot terlanjur diputar sebelum servo hidup."
                className={`${BUTTON_BASE} col-span-2 bg-gray-50 text-gray-700 border border-gray-200 hover:bg-gray-100`}
                onClick={() => sendImuCommand("zero_yaw", "Yaw di-nol-kan di arah sekarang")}
                disabled={rosState !== "connected"}
              >
                Zero Yaw
              </button>
              <button
                title="Kalibrasi bias gyro. Robot harus benar-benar diam selama proses."
                className={`${BUTTON_BASE} col-span-2 bg-gray-50 text-gray-700 border border-gray-200 hover:bg-gray-100`}
                onClick={() => sendImuCommand("calibrate_gyro", "Kalibrasi gyro: jangan gerakkan robot")}
                disabled={rosState !== "connected"}
              >
                Calibrate Gyro
              </button>
            </div>
          </div>
        </div>

        {/* Kalibrasi Yaw -- lima titik, direkam operator */}
        <div className="bg-white p-6 rounded-2xl border border-gray-200 shadow-sm">
          <SectionHeader title="Kalibrasi Yaw" />
          {(() => {
            const cal = yawCal;
            const rows = cal?.table || [];
            const recorded = new Set(cal?.recorded || []);
            const complete = (cal?.missing || [1]).length === 0;
            const src = cal?.source || "gyro";
            const gap = cal?.back_gap_deg || {};
            const fmt = (v, d = 1) =>
              typeof v === "number" && Number.isFinite(v) ? v.toFixed(d) : "—";
            const conn = rosState === "connected";
            return (
              <div className="flex flex-col gap-3">
                <p className="text-[11px] leading-relaxed text-gray-500">
                  Putar robot ke tiap posisi fisik, tahan diam, lalu tekan <b>Rekam</b>.
                  Nilai sensor disimpan apa adanya. Pemetaan baru dihitung setelah
                  kelima titik lengkap dan kamu menekan Terapkan.
                  <span className="font-bold text-gray-700"> Kiri = positif, kanan = negatif.</span>
                </p>

                {/* Pilih sumber sinyal. Keduanya direkam bersamaan, jadi berpindah
                    sumber tidak perlu merekam ulang. */}
                <div className="flex items-center gap-2">
                  <span className="text-[10px] font-bold uppercase tracking-wide text-gray-400">Sumber</span>
                  {["gyro", "compass"].map((opt) => (
                    <button
                      key={opt}
                      title={opt === "gyro"
                        ? "Yaw integrasi gyro. Mulus, melayang ~0,4 derajat/menit. Disarankan di robot ini."
                        : "Heading magnetometer. Absolut, tapi di robot ini terukur melayang +33 derajat per 300 detik saat diam dan melompat saat servo aktif."}
                      className={`${BUTTON_BASE} flex-1 ${src === opt
                        ? "bg-undip-blue text-white"
                        : "bg-gray-50 text-gray-600 border border-gray-200 hover:bg-gray-100"}`}
                      onClick={() => sendYawCalib("yaw_calib_apply", { source: opt })}
                      disabled={!conn || yawCalBusy || !complete}
                    >
                      {opt === "gyro" ? "Gyro" : "Kompas"}
                      {Number.isFinite(gap[opt]) && (
                        <span className="ml-1 font-mono text-[10px] opacity-70">
                          ±{fmt(gap[opt])}°
                        </span>
                      )}
                    </button>
                  ))}
                </div>
                {(Number.isFinite(gap.gyro) || Number.isFinite(gap.compass)) && (
                  <p className="-mt-1 text-[10px] text-gray-400">
                    Angka di tombol = selisih antara rekaman kiri 180 dan kanan 180.
                    Keduanya arah fisik yang sama, jadi makin kecil makin bisa dipercaya.
                  </p>
                )}

                {/* Tabel: target, sensor terekam, hasil koreksi, galat */}
                <div className="overflow-x-auto">
                  <table className="w-full text-[11px]">
                    <thead>
                      <tr className="text-left text-gray-400">
                        <th className="py-1 pr-2 font-bold uppercase">Target</th>
                        <th className="py-1 pr-2 font-bold uppercase">Sensor</th>
                        <th className="py-1 pr-2 font-bold uppercase">Koreksi</th>
                        <th className="py-1 pr-2 font-bold uppercase">Galat</th>
                        <th className="py-1 font-bold uppercase">Rekam</th>
                      </tr>
                    </thead>
                    <tbody>
                      {(rows.length ? rows : [0, 90, 180, -90, -180].map((t) => ({ target: t })))
                        .map((row) => {
                        const t = row.target;
                        const has = recorded.has(t);
                        const err = row.err_new;
                        return (
                          <tr key={t} className="border-t border-gray-50">
                            <td className="py-1.5 pr-2 font-bold text-gray-800">
                              {t > 0 ? `+${t}` : t}°
                              <span className="ml-1 font-normal text-gray-400">
                                {t === 0 ? "awal" : t > 0 ? "kiri" : "kanan"}
                              </span>
                            </td>
                            <td className="py-1.5 pr-2 font-mono text-gray-600">
                              {has ? `${fmt(row.raw)}°` : "—"}
                            </td>
                            <td className="py-1.5 pr-2 font-mono font-bold text-gray-900">
                              {typeof row.yaw_new === "number" ? `${fmt(row.yaw_new)}°` : "—"}
                            </td>
                            <td className={`py-1.5 pr-2 font-mono ${
                              typeof err !== "number" ? "text-gray-400"
                                : Math.abs(err) < 3 ? "text-green-600" : "text-yellow-700"}`}>
                              {typeof err === "number" ? `${err >= 0 ? "+" : ""}${fmt(err)}°` : "—"}
                            </td>
                            <td className="py-1.5">
                              <div className="flex gap-1">
                                <button
                                  className={`${BUTTON_BASE} flex-1 ${has
                                    ? "bg-green-50 text-green-700 border border-green-100 hover:bg-green-100"
                                    : "bg-undip-blue text-white hover:bg-opacity-90"}`}
                                  onClick={() => sendYawCalib("yaw_calib_record", { target: t })}
                                  disabled={!conn || yawCalBusy}
                                  title={has ? "Rekam ulang titik ini" : "Rekam nilai sensor di posisi ini"}
                                >
                                  {has ? "Ulang" : "Rekam"}
                                </button>
                                {cal?.applied && (
                                  // Verifikasi merekam yaw SESUDAH koreksi di posisi
                                  // ini. Itulah galat sebenarnya -- beda dengan kolom
                                  // koreksi yang dihitung dari rekaman itu sendiri.
                                  <button
                                    className={`${BUTTON_BASE} px-2 ${row.basis === "verifikasi"
                                      ? "bg-green-600 text-white"
                                      : "bg-white text-gray-500 border border-gray-200 hover:bg-gray-100"}`}
                                    onClick={() => sendYawCalib("yaw_calib_record", { target: t, verify: true })}
                                    disabled={!conn || yawCalBusy}
                                    title="Putar robot ke posisi ini, lalu tekan untuk mengukur galat sebenarnya"
                                  >
                                    ✓
                                  </button>
                                )}
                              </div>
                            </td>
                          </tr>
                        );
                      })}
                    </tbody>
                  </table>
                </div>

                {rows.some((r) => r.basis === "rekaman") && (
                  // Galat nol di sini TIDAK membuktikan apa-apa: pemetaan dipasang
                  // pada rekamannya sendiri. Yang membuktikan adalah verifikasi.
                  <p className="rounded bg-blue-50 px-2 py-1.5 text-[10px] leading-relaxed text-blue-800">
                    Kolom koreksi masih dihitung dari rekaman yang sama, jadi galatnya
                    nol menurut konstruksi. Untuk galat yang sebenarnya: terapkan dulu,
                    putar robot ke tiap titik lagi, lalu tekan tombol <b>✓</b> di baris itu.
                  </p>
                )}

                {cal?.mapping_error?.[src] && (
                  <p className="rounded bg-yellow-50 px-2 py-1.5 text-[10px] leading-relaxed text-yellow-800">
                    {cal.mapping_error[src]}
                  </p>
                )}

                <div className="grid grid-cols-2 gap-2">
                  <button
                    className={`${BUTTON_BASE} bg-undip-blue text-white hover:bg-opacity-90`}
                    onClick={() => sendYawCalib("yaw_calib_apply")}
                    disabled={!conn || yawCalBusy || !complete}
                    title={complete
                      ? "Hitung pemetaan dari kelima titik, tulis ke robot, dan pakai sekarang"
                      : "Rekam kelima titik dulu"}
                  >
                    Terapkan
                  </button>
                  <button
                    className={`${BUTTON_BASE} bg-gray-50 text-gray-700 border border-gray-200 hover:bg-gray-100`}
                    onClick={() => sendYawCalib("yaw_calib_get")}
                    disabled={!conn || yawCalBusy}
                    title="Baca ulang rekaman dan status dari robot"
                  >
                    Segarkan
                  </button>
                  <button
                    className={`${BUTTON_BASE} bg-gray-50 text-gray-700 border border-gray-200 hover:bg-gray-100`}
                    onClick={() => sendYawCalib("yaw_calib_disable")}
                    disabled={!conn || yawCalBusy || !cal?.applied}
                    title="Matikan pemetaan sementara untuk membandingkan dengan perilaku lama. Rekaman dan berkas tidak dihapus."
                  >
                    Matikan
                  </button>
                  <button
                    className={`${BUTTON_BASE} bg-red-50 text-red-600 border border-red-100 hover:bg-red-100`}
                    onClick={() => sendYawCalib("yaw_calib_clear")}
                    disabled={!conn || yawCalBusy || recorded.size === 0}
                    title="Hapus semua titik yang sudah direkam dan mulai dari awal"
                  >
                    Hapus Rekaman
                  </button>
                </div>

                <div className={`rounded-lg px-2 py-1.5 text-center text-[10px] font-bold uppercase tracking-wider ${
                  cal?.applied ? "bg-green-50 text-green-700"
                    : complete ? "bg-yellow-50 text-yellow-700"
                    : "bg-gray-100 text-gray-500"}`}>
                  {cal?.applied
                    ? `Dipakai — sumber ${src === "gyro" ? "gyro" : "kompas"}`
                    : complete ? "Lengkap, belum diterapkan"
                    : `${recorded.size} dari 5 titik direkam`}
                </div>
              </div>
            );
          })()}
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
          <div className={activeTab === "rl" ? "h-full" : "hidden"}>
            <RlTrainingPage
              ros={rosRef.current}
              rosState={rosState}
              sendStatus={sendStatus}
              studioParams={walkingParams}
              studioVersionName={walkingVersions.find((v) => v.id === activeWalkingVersionId)?.name || ""}
              walkingVersions={walkingVersions}
              onBeforeWalking={restoreFullSpeed}
            />
          </div>
          <div className={activeTab === "actrain" ? "h-full" : "hidden"}>
            <ActionTrainingPage
              ros={rosRef.current}
              rosState={rosState}
              sendStatus={sendStatus}
              onBeforeAction={() => {
                // Sama dengan Init Pose: kaki pindah ke action_module, jadi Start
                // walking berikutnya harus mengaktifkan walking_module lagi; batas
                // kecepatan Torque ON dilepas supaya page diputar dengan laju aslinya.
                walkingModuleEnabledRef.current = false;
                restoreFullSpeed();
              }}
            />
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
