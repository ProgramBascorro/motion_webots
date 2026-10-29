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
  Brain,
  Footprints,
  Gamepad2,
  Crosshair,
  Info,
  Play,
  PlayCircle,
  RefreshCw,
  Ruler,
  Save,
  Send,
  Square,
  Trash2
} from "lucide-react";
import ActionEditor from "./ActionEditor.jsx";
import GamepadVisualizer from "./GamepadVisualizer.jsx";
import ChartPage from "./ChartPage.jsx";
import TuningPage from "./TuningPage.jsx";
import RlTrainingPage from "./RlTrainingPage.jsx";
import WalkingSimPreview from "./WalkingSimPreview.jsx";
import WalkingOffsetInput from "./WalkingOffsetInput.jsx";
import {
  WALKING_POSE_REFERENCE, walkingReferenceOffset, walkingOffsetToDisplay,
} from "./walkingPoseReference.js";
import {
  seedWalkingVersion, parseStoredWalkingVersions, findRestorableVersion, planWalkingSave,
  walkingDraftUnsaved, sameWalkingParams,
} from "./walkingVersionStore.js";
import { fetchWalkingStore, mutateWalkingStore } from "./walkingVersionServer.js";
import RobotBlueprint from "./RobotBlueprint.jsx";
import {
  parseWalkingNumber, WALKING_KEYS, WALKING_INT_FIELDS, WALKING_BOOL_FIELDS,
  parameterValuesEqual, parametersEqual, validateWalkingParams, applyWalkingDraft,
  snapWalkingRuntime, WALKING_BIAS_FIELDS, walkingPoseFarFromReference,
} from "./walkingParams.js";
import {
  KICK_DEFAULT_PARAMS, KICK_PARAM_KEYS, validateKickParams,
  decodeKickParameters, makeKickParameters,
} from "./kickParams.js";
import { measureBall, MEASURE_WINDOW_DEFAULTS } from "./kickMeasure.js";
import { callRosService, waitForRobot } from "./net/services.js";
import {
  computeDefaultRosbridgeUrl,
  normalizeRosbridgeUrl,
  rewriteLoopbackToCurrentHost,
} from "./net/rosbridge.js";

// Status magnetometer OpenCR (register 61). Menentukan apakah nilai Yaw layak dipercaya:
// pada status selain OK/kalibrasi, open_cr_module sengaja memaksa yaw ke 0.
const MAG_STATUS = {
  0: { label: "no mag", cls: "bg-red-100 text-red-600" },
  1: { label: "settling", cls: "bg-amber-100 text-amber-600" },
  2: { label: "ok", cls: "bg-green-100 text-green-600" },
  3: { label: "calib", cls: "bg-blue-100 text-blue-600" },
};

const SHARED_ROS_URL_KEY = "bascorro.shared_ros_url.v1";
const LEGACY_ACTION_ROS_URL_KEY = "op3RosUrl";
const WALKING_VERSION_STORAGE_KEY = "bascorro.walking_versions.v1";
// Versi aktif di browser: HANYA dipakai untuk impor pertama ke disk robot dan
// untuk mode baca-saja saat server penyimpanan tidak terjangkau.
const WALKING_ACTIVE_VERSION_KEY = "bascorro.walking_versions.active.v1";
// Seeded once, then never again: if the version is deleted on purpose it must
// stay deleted, so the flag records that the seed already ran rather than
// checking whether the version is currently present.
// Bumped to v2 on 2026-08-11: the preset's init_* offsets were re-derived from
// action page 2, so a browser holding the v1 copy would keep serving the old
// stance. Bumping re-runs the seed once, which refreshes the preset in place.
const WALKING_SEED_FLAG_KEY = "bascorro.walking_versions.seeded.dari_claude.v2";
const TELEOP_COMMAND_TOPIC = "/op3_joy_teleop/command";
const TELEOP_STATUS_TOPIC = "/op3_joy_teleop/status";
// Gambar beranotasi dari detektor bola yang SAMA dengan yang dipakai demo.
//
// demo.launch.xml meng-include op3_ball_detector/yolo_ball_detector_openvino.launch.py,
// dan berkas itu menjalankan usb_cam (namespace usb_cam_node) + detektornya
// (namespace ball_detector_node). Jadi topik yang benar-benar ada saat demo
// hidup adalah /ball_detector_node/image_out, bukan /vision/yolo/debug --
// yang terakhir itu milik op3_yolo_vision, pipeline lain yang tidak dipakai
// demo.launch.xml. Peluncur tmux kini menjalankan berkas yang sama.
const DEFAULT_OVERLAY_TOPIC =
  import.meta.env.VITE_OVERLAY_TOPIC || "/ball_detector_node/image_out";

// WALKING_READY is action page 2 (user's calibrated standing pose).
const WALKING_READY_PAGE_NUM = 2;
// Time for op3_manager to switch the controller to action_module before
// publishing the page command (the page_num subscriber needs the swap).
const ACTION_MODULE_SETTLE_MS = 600;
// Time for action page 2 (WALKING_READY) to finish playing on the hardware
// (~1.2 s on this robot; buffer for trajectory tail).
const WALKING_READY_PLAY_MS = 1800;
// Time for op3_manager to load the walking_module control_module before we
// publish "start"; otherwise the start command is dropped silently.
const WALKING_ENABLE_SETTLE_MS = 1200;
// Torque ON: melepas modul menunggu modul yang sedang berjalan BERHENTI dulu
// (setCtrlModuleThread: while isRunning -> usleep), jadi lamanya tidak tetap --
// sampai sekitar satu langkah kalau robot sedang berjalan. Ditunggu sampai
// terkonfirmasi lewat layanan, bukan ditebak dengan jeda.
const MODULE_RELEASE_TIMEOUT_MS = 5000;
const MODULE_RELEASE_POLL_MS = 150;
// Sesudah goal disamakan: beberapa siklus kontrol 8 ms supaya nilai barunya
// sudah terkirim ke servo sebelum torsi dinyalakan.
const GOAL_SYNC_SETTLE_MS = 80;
const WALKING_LEG_JOINTS = [
  "r_hip_yaw", "l_hip_yaw", "r_hip_roll", "l_hip_roll", "r_hip_pitch", "l_hip_pitch",
  "r_knee", "l_knee", "r_ank_pitch", "l_ank_pitch", "r_ank_roll", "l_ank_roll",
];
// robotis_controller merges sync_write_item per port per cycle — sending two
// items in the same ~8 ms tick silently drops the second. Stagger them.
const SYNC_ITEM_GAP_MS = 60;
// Batas kecepatan servo yang dipasang sebelum torsi dinyalakan.
//
// APA YANG DIBATASI -- DIUKUR ULANG 2026-09-09, KOREKSI DARI CATATAN LAMA.
// Catatan sebelumnya di sini menyatakan "Init Pose dan gait TIDAK ikut
// terkekang" karena modulnya menulis goal kecil-kecil tiap 8 ms. ITU SALAH.
// profile_velocity adalah pembangkit lintasan DI DALAM servo dan berlaku untuk
// SETIAP penulisan goal_position, termasuk yang 8 ms itu. Terukur di robot,
// preset JALAN MENUJU MEKKAH, parameter operator tanpa diubah sedikit pun:
//
//   batas masih terpasang : perintah ayun pinggul 23,21 deg -> nyata  5,0 deg (22%)
//                           lutut          36,30 deg -> nyata  4,0 deg (11%)
//                           telat ~350 ms = hampir setengah periode gait
//   profile_velocity = 0  : pinggul 23,21 -> 17,50 deg (75%), lutut 36,30 -> 21,88 (60%)
//
// Jadi batas yang tertinggal membuat robot berjalan pelan dengan langkah kecil
// PADAHAL parameternya sama persis -- persis keluhan "nilainya jadi kecil
// sendiri". Berdiri diam tidak terpengaruh (galat cuma 0,2-0,4 deg), jadi
// gejalanya cuma muncul saat berjalan.
//
// Lamanya Init Pose sendiri tetap ditentukan halaman di ALPHONSE.bin.
//
// Satuan XM430-W350: profile_velocity 0,229 rpm per hitungan, jadi 8 kira-kira
// 11 deg/detik di keluaran. 0 berarti TANPA BATAS.
//
// Angka "Lambat" diturunkan 15 -> 8 pada 2026-09-06 atas laporan operator
// ("sudah dipilih lambat masih juga kencang").
const TORQUE_SPEED_PRESETS = [
  { key: "slow",   label: "Lambat", velocity: 8,  accel: 4,  approxDegPerSec: 11 },
  { key: "medium", label: "Sedang", velocity: 20, accel: 8,  approxDegPerSec: 27 },
  { key: "fast",   label: "Cepat",  velocity: 45, accel: 20, approxDegPerSec: 62 },
];
const DEFAULT_TORQUE_SPEED_KEY = "medium";

const PARAM_TYPES = {
  bool: 1,
  int: 2,
  double: 3,
  string: 4,
};

// Parameter DETEKTOR yang benar-benar jalan, yaitu node
// /ball_detector_node/yolo_ball_detector dari paket op3_ball_detector -- yang
// sama dengan yang di-include demo.launch.xml.
//
// SEBELUM 2026-09-08 daftar ini berisi ball_/goalpost_/robot_confidence_threshold
// dan ditembakkan ke /op3_yolo_vision/*. Tiga-tiganya salah sekaligus:
//   1. nama nodenya bukan "op3_yolo_vision" melainkan "yolo_detector";
//   2. pipeline itu memang tidak pernah dijalankan peluncur mana pun;
//   3. detektor yang JALAN tidak punya satu pun nama parameter itu.
// Jadi slider di halaman Vision tidak pernah berpengaruh sama sekali.
//
// Yang dibuka hanya dua ambang yang memang dibaca ulang saat berjalan (lihat
// _on_set_parameters di yolo_ball_detector.py). model_path, imgsz, device dan
// detection_rate SENGAJA tidak dibuka: ketiganya cuma dipakai saat memuat model
// atau membuat timer, jadi menyetelnya lewat parameter tidak berpengaruh.
const YOLO_PARAM_KEYS = ["confidence_threshold", "iou_threshold"];

const YOLO_PARAM_INFO = {
  confidence_threshold: {
    label: "Ambang keyakinan",
    help: "Di bawah angka ini deteksi dibuang. Terikat pada BOBOT yang dipakai: " +
      "v8n (Bola-baru) memberi keyakinan sekitar 0,45 sehingga 0,50 memblokir semuanya, " +
      "sedangkan ball_yolov8n int8 sekitar 0,85. Turunkan kalau bola tidak pernah terdeteksi.",
  },
  iou_threshold: {
    label: "Ambang IoU",
    help: "Seberapa banyak dua kotak boleh bertindih sebelum salah satunya dibuang. " +
      "Jarang perlu diubah; naikkan kalau satu bola terdeteksi jadi beberapa kotak.",
  },
};

// What the grid shows before the first Load, so it has to be the same gait the robot
// actually boots with: every value here mirrors op3_walking_module/config/param.yaml,
// which is the file walking_module loads at startup (and rewrites on "save").
// Two unit conversions to watch when comparing the two by eye:
//   param.yaml period_time is in ms, here it is in seconds (500 -> 0.5)
//   param.yaml roll/pitch/yaw/hip_pitch/pelvis offsets are in degrees, here in radians
// param.yaml foot_height maps to z_move_amplitude, swing_right_left to
// y_swap_amplitude, swing_top_down to z_swap_amplitude.
// init_x/z_offset come from forward kinematics on the calibrated WALKING_READY
// (action page 2) joint values -- regenerate with scripts/walking_ready_fk.py --yaml
// <page 2 export> when the page is re-taught, and update param.yaml together with
// this block.
// CERMIN op3_walking_module/config/param.yaml: preset "dari ketua"
// (tersimpan 2026-08-18), yang sejak 2026-08-22 juga jadi isi param.yaml.
//
// Inilah yang mengisi kotak isian SEBELUM operator menekan Load, jadi harus
// tetap seirama dengan param.yaml. Dikembalikan ke isi 3 September pada
// 2026-09-06 bersama param.yaml-nya.
const WALKING_DEFAULT_PARAMS = {
  // init_y_offset is the one value here that is NOT FK-derived: the walking module
  // leaves the lateral offset out of the stance it derives walking_bias_ from, so this
  // is extra foot separation on top of page 2 -- applied both standing and walking.
  init_x_offset: -0.02,
  init_y_offset: 0.028,
  init_z_offset: 0.0408,
  init_roll_offset: 0,
  init_pitch_offset: -0.07,
  init_yaw_offset: 0,
  period_time: 0.8,       // param.yaml period_time: 800 (ms)
  dsp_ratio: 0.58,
  step_fb_ratio: 0.2,
  soft_start_time: 2.0,   // param.yaml soft_start_time: 2000 (ms)
  // x_move_amplitude di sini cuma nilai awal kotak isian. Saat demo, besarnya
  // ditentukan ball_follower dan dibatasi MAX_FB_STEP = 0.0181 m -- lihat
  // op3_demo/src/soccer/ball_follower.cpp.
  x_move_amplitude: 0,
  y_move_amplitude: 0,
  z_move_amplitude: 0.037,
  angle_move_amplitude: 0,
  angle_move_amplitude_turn_in: 0,
  move_aim_on: false,
  balance_enable: false,
  balance_hip_roll_gain: 0.35,
  balance_knee_gain: 0.4,
  balance_ankle_roll_gain: 0.7,
  balance_ankle_pitch_gain: 0.9,
  y_swap_amplitude: -0.002,
  z_swap_amplitude: 0.006,
  arm_swing_gain: 0,
  pelvis_offset: 0.0524,
  hip_pitch_offset: 0.1396,
  p_gain: 36,
  i_gain: 0,
  d_gain: 0,
};

const WALKING_PARAM_HELP = {
  init_x_offset: {
    artinya: "Offset posisi badan maju-mundur saat robot masuk posture walking.",
    fungsi: "Menggeser titik awal badan terhadap kaki supaya center of mass tidak terlalu maju atau mundur.",
    tuning: "Ubah kecil-kecil sekitar 0.001 m. Jika robot cenderung jatuh ke depan, coba lebih negatif; jika ke belakang, coba lebih positif. Apply langsung terlihat saat robot berdiri (walking module aktif, belum start), bergerak bertahap maksimal 25 deg/detik.",
    risk: "Terlalu besar bisa bikin lutut/ankle bekerja keras dan robot langsung condong. Karena sekarang bergerak saat berdiri juga, pegang robot waktu mencoba nilai baru."
  },
  init_y_offset: {
    artinya: "Tambahan jarak antar telapak kaki. Bukan bias kiri-kanan — kedua kaki melebar simetris.",
    fungsi: "Melebarkan pijakan di atas stance page 2, dan nilainya dipakai sama persis saat berdiri maupun saat gait berjalan. Inilah satu-satunya offset yang mengubah lebar kaki; kalau 0, lebar kaki murni ikut page 2.",
    tuning: "Naikkan 0.005 m per percobaan sampai kedua telapak tidak lagi merapat/bersenggolan waktu melangkah. Tambahan jarak kira-kira y_offset + 6 mm. Nilai 0.010 adalah titik awal, bukan hasil FK — silakan disesuaikan.",
    risk: "Sejak 2026-08-17 nilai ini ikut bergerak saat berdiri (dilandaikan maks 25 deg/detik), jadi pegang robot waktu mencoba nilai baru. Terlalu lebar bikin langkah terhuyung dan boros tenaga."
  },
  init_z_offset: {
    artinya: "Tinggi badan dasar saat gait walking dihitung.",
    fungsi: "Menentukan seberapa jongkok/tinggi postur robot ketika berjalan.",
    tuning: "Lebih rendah biasanya lebih stabil tapi servo lebih berat. Lebih tinggi terasa ringan tapi mudah goyang. Apply langsung terlihat saat berdiri, sama seperti X offset.",
    risk: "Terlalu rendah bisa membebani lutut; terlalu tinggi bisa membuat kaki kehilangan clearance. Lompatan besar sekali ketik dilandaikan sekitar sedetik, bukan dihantam sekaligus."
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
    tuning: "Baca nilai runtime dengan Load; param.yaml dan preset demo dapat berbeda. Ubah bertahap dengan robot digantung atau dipegang, bukan berdasarkan angka referensi browser.",
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
  soft_start_time: {
    artinya: "Lama (detik) robot menaikkan parameter grup Movement dan tempo langkah dari kecil ke nilai penuh, dihitung sejak tombol start ditekan.",
    fungsi: "Membuat nilai di halaman ini jadi BATAS ATAS, bukan nilai langkah pertama. X/Y/Z/Angle move amplitude dan period time naik bertahap selama waktu ini, supaya arus servo tidak melonjak dan robot tidak oleng saat mulai jalan. Menekan Apply di tengah jalan juga lewat jalur yang sama: nilai baru dikejar bertahap, bukan dilompati.",
    tuning: "2.0 detik kira-kira empat langkah pada period time 0.5 s. Naikkan kalau langkah pertama masih tersentak; turunkan kalau robot terasa lamban menuruti perintah. Isi 0 untuk mematikan (langsung penuh, perilaku sebelum 2026-08-18).",
    risk: "Terlalu besar membuat robot lama mencapai kecepatan yang diminta, dan itu merepotkan kalau butuh reaksi cepat. Hanya grup Timing dan Movement yang terpengaruh; Init pose, Gait shape, Balance dan Gain tidak."
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
  angle_move_amplitude_turn_in: {
    artinya: "Besar putaran yaw \"putar dalam\", sumbu TERPISAH dari Angle move amplitude di atasnya.",
    fungsi: "Bedanya dua: (1) kedua hip yaw -- servo 7 dan servo 8 -- memutar ke arah yang SAMA, sedangkan sumbu di atas memutar keduanya berlawanan; (2) satu kaki yaw-nya DIAM selama telapaknya di udara, dan seluruh putarannya dipindah ke saat telapak itu sudah menapak, jadi putaran badan datang dari gesekan telapak dan bukan terbuang di udara.",
    tuning: "Tanda menentukan arah putaran badan sekaligus kaki mana yang berputar saat menapak: POSITIF = searah jarum jam, kaki KIRI (servo 8) yang berputar saat menapak; NEGATIF = berlawanan arah jarum jam, kaki KANAN (servo 7), dan sifat \"putar ke dalam\"-nya tetap sama karena kedua sisi benar-benar bercermin. 0 mematikan sumbu ini sepenuhnya dan gait kembali persis aslinya. Nilai ini DIJUMLAHKAN di atas Angle move amplitude, jadi keduanya bisa dipakai bersamaan; untuk mengenal efeknya, biarkan Angle move amplitude 0 dulu.",
    risk: "Telapak yang menapak menggosok lantai lebih lama, jadi beban servo 7/8 dan keausan telapak naik. Nilai besar bisa membuat kaki silang dan balance hilang, sama seperti turn biasa."
  },
  move_aim_on: {
    artinya: "Flag mode aim/move pada message walking.",
    fungsi: "Disediakan oleh WalkingParam untuk mode gerak tertentu, tapi pada setup ini biasanya tidak perlu aktif.",
    tuning: "Biarkan off kecuali kamu tahu module yang dipakai membutuhkan flag ini.",
    risk: "Mengaktifkan tanpa kebutuhan bisa membuat perilaku sulit dibaca saat tuning."
  },
  balance_enable: {
    artinya: "Mengaktifkan koreksi balance dari feedback gyro/IMU.",
    fungsi: "Persisnya: hanya menyalakan/mematikan sensoryFeedback(), yang menulis ke 8 sendi kaki saja — hip_roll, knee, ank_pitch, ank_roll kiri dan kanan. Empat gain di grup ini adalah bobotnya. Di luar 8 sendi itu tidak ada yang berubah: ayunan tangan, sway badan, pelvis, dan hip pitch semuanya bagian dari gait dan tetap jalan walau ini dimatikan. Saat robot cuma berdiri (belum Start) koreksi ini juga memang tidak dipakai.",
    tuning: "Untuk robot asli biasanya on. Untuk membandingkan efek gain, boleh off sebentar sambil robot dipegang. Tombol Balance on/off dan checkbox ini selalu sinkron, dan setelah ditekan studio langsung membaca ulang nilai di robot — lihat kolom Current untuk buktinya.",
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
    artinya: "Gain ayunan tangan saat berjalan. TIDAK ada hubungannya dengan IMU.",
    fungsi: "Ayunan tangan mengimbangi gerakan kaki. Rumusnya murni gait: ayunan = x_move_amplitude x arm_swing_gain, tanpa membaca gyro sama sekali. Karena itu mematikan Balance enable tidak akan menghentikan ayunan tangan.",
    tuning: "Mau tangan benar-benar diam? Set arm_swing_gain ke 0, atau set x_move_amplitude ke 0 (tangan otomatis diam kalau robot tidak maju/mundur). Naikkan jika badan terlalu kaku, turunkan jika ayunan mengganggu.",
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
      { key: "soft_start_time", label: "Soft start", unit: "s", step: 0.1, ...WALKING_PARAM_HELP.soft_start_time },
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
      { key: "angle_move_amplitude_turn_in", label: "Angle move amplitude turn in (putar dalam)", unit: "rad", step: 0.005, ...WALKING_PARAM_HELP.angle_move_amplitude_turn_in },
      { key: "move_aim_on", label: "Move aim on", type: "bool", ...WALKING_PARAM_HELP.move_aim_on },
    ],
  },
  // The five gait-shape fields below used to sit in this same "Balance" group, which
  // read as if "Balance enable" governed them. It does not: balance_enable gates
  // sensoryFeedback(), and sensoryFeedback() writes to exactly eight leg joints
  // (hip_roll, knee, ank_pitch, ank_roll, kiri+kanan) and nothing else. Arm swing in
  // particular comes from computeArmAngle(), driven by x_move_amplitude -- turning the
  // IMU off has never stopped it and never will. Splitting the group so the checkbox
  // only covers what it actually controls.
  {
    title: "Balance (IMU)",
    description: "Hanya ini yang dimatikan oleh Balance enable. Koreksi gyro ke 8 sendi kaki: hip roll, knee, ankle pitch, ankle roll (kiri+kanan). Cuma jalan saat gait berjalan.",
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
    description: "Bagian dari pola langkah itu sendiri, BUKAN IMU. Balance enable tidak mempengaruhi satu pun field di sini.",
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

// Blank/partial input falls back to the default rather than to 0 -- see
// walkingParams.js for why Number() alone was wrong here.
function normalizeWalkingParams(params = {}) {
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

function formatWalkingValue(value, field) {
  if (field.type === "bool") return value ? "true" : "false";
  const numeric = parseWalkingNumber(value, NaN);
  if (!Number.isFinite(numeric)) return "—";
  if (field.type === "int") return String(numeric);
  return formatNumber(numeric, 5);
}

// Daftar versi ditulis ULANG ke localStorage setiap kali ia berubah, termasuk
// beberapa milidetik sesudah halaman dimuat. Jadi apa pun yang gagal dibaca di
// sini akan tertimpa dan HILANG SELAMANYA -- satu JSON.parse yang gagal, atau
// satu entri yang bentuknya tidak dikenali, cukup untuk menghapus seluruh versi
// tersimpan operator tanpa jejak. Teks mentahnya disalin dulu ke kunci
// bertanggal supaya selalu ada yang bisa dipulihkan.
function rescueWalkingVersions(raw) {
  try {
    localStorage.setItem(`${WALKING_VERSION_STORAGE_KEY}.rusak.${Date.now()}`, raw);
  } catch {
    // Penyimpanan penuh atau dibatasi -- tidak ada lagi yang bisa dilakukan.
  }
}

function readWalkingVersions() {
  if (typeof window === "undefined") return [];
  let raw = null;
  try {
    raw = localStorage.getItem(WALKING_VERSION_STORAGE_KEY);
  } catch {
    // Penyimpanan dibatasi (mode privat): jalan tanpa versi tersimpan.
    return [];
  }
  const { versions, rusak } = parseStoredWalkingVersions(raw);
  if (rusak) rescueWalkingVersions(rusak);
  return versions.map((item) => ({
    id: String(item.id),
    name: String(item.name || "Untitled"),
    createdAt: Number(item.createdAt || Date.now()),
    updatedAt: Number(item.updatedAt || item.createdAt || Date.now()),
    params: normalizeWalkingParams(item.params),
  }));
}

function readLocalActiveVersionId() {
  try {
    return localStorage.getItem(WALKING_ACTIVE_VERSION_KEY) || null;
  } catch {
    return null;
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
// Five deliberate deltas from that baseline: x_move_amplitude 0 -> 0.02 (the robot
// marched in place without it), period_time 0.50 -> 0.80 and dsp_ratio 0.35 -> 0.40
// (slower cycle, longer double support), y_swap_amplitude 0.020 -> 0.025,
// arm_swing_gain 1.5 -> 1.0. Everything else, p/i/d included, is the baseline.
// The init_* offsets are left untouched: they are FK-tuned to WALKING_READY
// (action page 2) so the handoff into walking_module stays smooth.
// p/i/d are inert here -- op3_walking_module.cpp never writes them to the servos.
// Editing this only reaches browsers that have not seeded it yet; seedWalkingVersions
// leaves an already-seeded list alone.
const CLAUDE_WALKING_VERSION_NAME = "dari claude";
const CLAUDE_WALKING_PARAMS = {
  init_x_offset: 0.0046,
  init_y_offset: 0.010,
  init_z_offset: 0.0408,
  init_roll_offset: 0,
  init_pitch_offset: 0.0698,
  init_yaw_offset: 0,
  hip_pitch_offset: 0.1396,
  pelvis_offset: 0.0524,
  period_time: 0.8,
  dsp_ratio: 0.4,
  step_fb_ratio: 0.25,
  soft_start_time: 2.0,
  x_move_amplitude: 0.02,
  y_move_amplitude: 0,
  angle_move_amplitude: 0,
  angle_move_amplitude_turn_in: 0,
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
  p_gain: 32,
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

// Semaian TIDAK PERNAH menimpa slot yang sudah terisi -- alasan lengkapnya di
// walkingVersionStore.js. Yang tersisa di sini hanya penanda localStorage-nya,
// yang membuat preset yang sudah dihapus operator tetap hilang.
function seedWalkingVersions(versions) {
  if (typeof window === "undefined") return versions;
  try {
    if (localStorage.getItem(WALKING_SEED_FLAG_KEY)) return versions;
  } catch {
    // Private/restricted storage: skip seeding rather than break the page.
    return versions;
  }
  return seedWalkingVersion(versions, {
    id: CLAUDE_WALKING_VERSION_ID,
    name: CLAUDE_WALKING_VERSION_NAME,
    params: normalizeWalkingParams(CLAUDE_WALKING_PARAMS),
  });
}

// Versi tersimpan "JALAN KE SURGA" di Studio.
//
// Angka aslinya, yang disetel operator 2026-09-02 dan harus sama persis dengan
// bagian `jalan` di op3_demo/config/preset_walking.yaml -- itulah yang
// benar-benar dipakai robot saat demo, jadi keduanya wajib seirama.
//
// Sempat diisi preset lama "dari ketua" pada 2026-09-05 (penanda .v1), lalu
// dikembalikan ke angka ini di hari yang sama (penanda .v2). Arsip "dari ketua"
// ada di komentar preset_walking.yaml dan di WALKING_DEFAULT_PARAMS di atas.
//
// x_move_amplitude 0,016 sengaja DITULIS di sini (WALKING_DEFAULT_PARAMS
// menaruh 0 karena di sana perannya cuma nilai awal kotak isian). Di slot
// tersimpan angkanya harus asli: ball_follower membacanya sebagai max_fb_step_.
const JALAN_WALKING_VERSION_NAME = "JALAN KE SURGA";
const JALAN_WALKING_VERSION_ID = "jalan-ke-surga-v1";
// Penanda seed diturunkan dari REVISI ISI di bawah, bukan nomor yang dinaikkan
// manual. Menaikkannya membuat browser yang slotnya MASIH KOSONG mendapat angka
// baru di muat berikutnya.
//
// Dulu kenaikan revisi juga MENIMPA slot yang sudah terisi, supaya "angkanya
// sudah diganti tapi Studio diam" tidak terjadi. Itu dicabut: harganya adalah
// menghapus hasil Simpan operator tanpa peringatan, dan itu jauh lebih mahal
// daripada Studio yang memperlihatkan angka lama. Lihat walkingVersionSlotTaken.
// Kalau sebuah preset memang harus disebarkan ulang ke browser yang sudah
// terisi, beri nama BARU -- jangan memakai nama yang sama.
const JALAN_WALKING_SEED_REV = "2026-09-02";
const JALAN_WALKING_SEED_FLAG_KEY =
  `bascorro.walking_versions.seeded.jalan_ke_surga.${JALAN_WALKING_SEED_REV}`;
const JALAN_WALKING_PARAMS = {
  init_x_offset: -0.03,
  init_y_offset: 0.028,
  init_z_offset: 0.0408,
  init_roll_offset: 0,
  init_pitch_offset: -0.12,
  init_yaw_offset: 0,
  period_time: 0.77,
  dsp_ratio: 0.4,
  step_fb_ratio: 0.25,
  soft_start_time: 0.05,
  x_move_amplitude: 0.016,
  y_move_amplitude: 0,
  z_move_amplitude: 0.037,
  angle_move_amplitude: 0,
  angle_move_amplitude_turn_in: 0,
  move_aim_on: false,
  balance_enable: false,
  balance_hip_roll_gain: 0.35,
  balance_knee_gain: 0.4,
  balance_ankle_roll_gain: 0.7,
  balance_ankle_pitch_gain: 0.9,
  y_swap_amplitude: 0.0021,
  z_swap_amplitude: 0.0,
  arm_swing_gain: 0,
  pelvis_offset: 0.05,
  hip_pitch_offset: 0.14,
  p_gain: 16,
  i_gain: 0,
  d_gain: 0,
};

// Menawarkan versi "JALAN KE SURGA" sekali per browser -- hanya kalau slotnya
// memang masih kosong.
//
// Bentuknya sengaja sama dengan seedWalkingVersions(): MURNI (cuma membaca
// penanda, tidak pernah menulisnya) karena StrictMode memanggil initializer
// useState dua kali di mode dev, dan id-nya tetap supaya kedua panggilan
// menghasilkan entri yang sama. Penandanya ditulis dari useEffect di bawah.
//
// Kalau slotnya sudah ada -- dari semaian lama maupun dari Simpan operator --
// ia TIDAK DISENTUH. Kalau belum ada, ditambahkan di depan.
function seedJalanWalkingVersion(versions) {
  if (typeof window === "undefined") return versions;
  try {
    if (localStorage.getItem(JALAN_WALKING_SEED_FLAG_KEY)) return versions;
  } catch {
    // Penyimpanan browser dibatasi (mode privat): lewati saja, jangan
    // merusak halaman.
    return versions;
  }
  return seedWalkingVersion(versions, {
    id: JALAN_WALKING_VERSION_ID,
    name: JALAN_WALKING_VERSION_NAME,
    params: normalizeWalkingParams(JALAN_WALKING_PARAMS),
  });
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

// Tiga mode demo yang dikenal demoModeCommandCallback() di
// op3_demo/src/demo_node.cpp. "ready" sengaja TIDAK di sini: ia bukan mode
// yang bisa dijalankan, melainkan perintah keluar dari demo yang sedang
// berjalan, jadi tempatnya di tombol berhenti.
//
// YANG PALING PENTING dan sempat salah di halaman ini: MEMILIH MODE SUDAH
// MEMULAI DEMONYA. Di robot, setDemoEnable() memanggil startSoccerMode()
// (soccer) atau startProcess() (action) langsung. Menekan Start sesudahnya
// justru MENGHENTIKAN demo, karena "start" itu toggle:
//
//     if (msg->data == "start") {
//       if (soccer_requested_ == true) stopSoccerMode();
//       else                           startSoccerMode();
//     }
//
// Urutan "pilih mode lalu tekan Start" -- yang dulu tertulis di halaman ini --
// karena itu berakhir dengan robot yang diam saja.
const DEMO_MODES = [
  {
    key: "soccer",
    label: "Soccer",
    description:
      "Cari bola, dekati, luruskan yaw, lalu tendang. Inilah mode yang memakai semua parameter di halaman Jarak Tendangan.",
  },
  {
    key: "action",
    label: "Action",
    description: "Memainkan daftar halaman gerakan dari berkas aksi. Tidak menyentuh walking module.",
  },
  {
    key: "vision",
    label: "Vision",
    description: "Peragaan pengolahan citra saja; kendali Jeda/Lanjut tidak dipakai di mode ini.",
  },
];

// ---- Halaman TENDANGAN: semua penentu jarak tendangan --------------------
//
// Yang diatur di sini BUKAN isi WalkingParam, melainkan parameter ROS milik
// node demo_node. Bawaannya cermin op3_demo/launch/demo.launch.xml; kalau
// launch-nya diubah, ubah juga di sini supaya kotak isian sebelum tombol
// "Muat dari robot" tidak menyesatkan.
//
// SEMUANYA dikirim sebagai DOUBLE, saklar on/off sekalipun. Di launch pun
// saklar ditulis bertipe float (0.0 / 1.0), dan set_parameters MENOLAK nilai
// yang tipenya berbeda dari yang sudah dideklarasikan node -- mengirim boolean
// untuk kick_radius_ready_enable akan gagal, bukan sekadar diabaikan.
const KICK_PARAM_GROUPS = [
  {
    title: "Jarak tendang",
    description:
      "Inti halaman ini. Jarak yang dipakai robot untuk memutuskan bola sudah cukup dekat untuk ditendang.",
    fields: [
      {
        key: "kick_distance",
        label: "Jarak tendang",
        unit: "m",
        step: 0.005,
        artinya: "Ambang estimasi jarak horizontal kamera ke bola untuk mendekat dan memicu tendangan, bukan jarak laju bola sesudah ditendang.",
        fungsi:
          "Dipakai dua kali sekaligus: (1) pemicu tendangan, dan (2) jarak BERHENTI saat mendekat. calcFootstep() diberi (jarak bola - jarak tendang), jadi begitu bola sampai di angka ini langkah majunya dijepit ke nol dengan sendirinya.",
        tuning:
          "Acuan kanan 7 September 2026: 0,15 m dari proyeksi kamera, bukan dari ujung kaki. Ubah paling banyak 0,005 m per uji terawasi. Kalau berhenti terlalu jauh, kecilkan; kalau menabrak bola, besarkan.",
        risk:
          "Terlalu kecil berarti robot menginjak bola sebelum sempat menendang; terlalu besar berarti kaki mengayun di udara.",
      },
      {
        key: "kick_pan_max_distance",
        label: "Jarak maksimum jendela pan",
        unit: "m",
        step: 0.01,
        artinya:
          "Batas jarak yang masih membolehkan jendela pan (pengecekan bola ada di depan kaki mana) menyala.",
        fungsi:
          "Memberi ruang di atas jarak tendang supaya jendela sempat menyala sesaat sebelum robot benar-benar berhenti.",
        tuning:
          "Acuan kanan: 0,20 m, yaitu 0,05 m di atas jarak tendang 0,15 m. Ini batas mulai penyelarasan, bukan izin langsung menendang.",
        risk:
          "Kalau disamakan persis dengan jarak tendang, jendelanya baru menyala tepat di ambang dan tendangan sering terlewat.",
      },
    ],
  },
  {
    title: "Geometri pengukur jarak",
    description:
      "Jarak bola TIDAK diukur langsung; dihitung dari sudut tunduk kepala dan ukuran kamera. Angka-angka inilah rumusnya.",
    fields: [
      {
        key: "kick_camera_height",
        label: "Tinggi kamera",
        unit: "m",
        step: 0.005,
        artinya: "Tinggi lensa kamera dari lantai.",
        fungsi:
          "Jarak = (tinggi kamera - jari-jari bola) / tan(sudut tunduk). Salah di sini membuat SEMUA bacaan jarak salah dengan pengali yang sama.",
        tuning:
          "Tinggi vertikal terukur 7 September 2026: 0,52 m. Ukur ulang jika pose atau ketinggian robot berubah; jangan memakai nilai lantai saat robot digantung.",
        risk:
          "Ini penyebab paling sering 'robot tidak pernah menendang' padahal bola sudah di kaki: ambang radius bola ikut dihitung dari sini.",
      },
      {
        key: "kick_camera_pitch_offset_deg",
        label: "Koreksi sudut kamera",
        unit: "deg",
        step: 0.1,
        artinya: "Koreksi sudut efektif pada model kamera; positif berarti sinar kamera lebih menunduk.",
        fungsi: "Ditambahkan saat memutar sinar gambar ke badan, sehingga jarak dan arah bola terkoreksi bersama. Tidak menggerakkan kepala atau mengubah parameter walking.",
        tuning: "Acuan kanan 7 September 2026: 30,84°, hasil pencocokan rekaman dengan meteran pada satu pose. Perlu diperiksa lagi pada pose walking yang sudah berhenti dan beberapa jarak.",
        risk: "Jangan menyalin dari sudut sendi atau mengubah untuk sekadar meloloskan tendangan. Koreksi yang salah mengubah perkiraan jarak sekaligus pilihan kaki.",
      },
      {
        key: "kick_ball_real_radius_m",
        label: "Jari-jari bola sebenarnya",
        unit: "m",
        step: 0.005,
        artinya: "Jari-jari bola yang dipakai, dalam meter.",
        fungsi:
          "Kepala mengunci PUSAT bola, bukan titik singgungnya di lantai, jadi beda tinggi yang benar adalah (tinggi kamera - jari-jari bola). Angka ini juga dipakai menghitung ambang radius di gambar.",
        tuning: "Bola terukur berdiameter 13,4 cm: isi jari-jari 0,067 m, bukan diameter 0,134 m.",
        risk: "Salah 1 cm menggeser jarak terukur beberapa sentimeter di jarak dekat.",
      },
      {
        key: "kick_fov_width_deg",
        label: "Setengah bukaan kamera",
        unit: "deg",
        step: 0.1,
        artinya:
          "Sudut yang dipakai menghitung panjang fokus: fokus_px = (lebar gambar / 2) / tan(sudut ini).",
        fungsi: "Menentukan berapa piksel jari-jari bola pada jarak tertentu.",
        tuning:
          "Hanya diubah kalau lensa atau kameranya diganti. 35,2 untuk kamera yang terpasang sekarang.",
        risk: "Salah di sini merusak ambang radius bola, bukan jarak dari sudut kepala.",
      },
      {
        key: "kick_image_width_px",
        label: "Lebar gambar",
        unit: "px",
        step: 1,
        artinya: "Lebar gambar yang benar-benar keluar dari detektor.",
        fungsi: "Pasangan wajib sudut bukaan di rumus panjang fokus.",
        tuning:
          "HARUS sama dengan lebar gambar di berkas parameter detektor. ALPHONSE 640. Robot dengan kamera 1280 harus diisi 1280.",
        risk: "Beda dari lebar gambar sungguhan membuat ambang radius meleset dua kali lipat.",
      },
      {
        key: "kick_head_tilt_sign",
        label: "Tanda sudut tunduk",
        unit: "+1 / -1",
        step: 1,
        artinya:
          "Konvensi tanda head_tilt. +1 berarti head_tilt NEGATIF artinya menunduk; -1 kebalikannya.",
        fungsi: "Menentukan arah sudut tunduk yang masuk ke rumus jarak.",
        tuning:
          "ALPHONSE +1, sudah DIUKUR 2026-09-03 dari 77 sampel goal_joint_states. Jangan ditebak dari komentar yaml, kesan gerak, atau foto lintas run -- tanda ini sudah dua kali salah ditebak.",
        risk:
          "Kalau terbalik, jarak bolanya negatif atau raksasa dan robot TIDAK PERNAH menendang sama sekali.",
      },
    ],
  },
  {
    title: "Sumber dan mutu bacaan jarak",
    description:
      "Dari mana angka jaraknya diambil, dan bacaan seperti apa yang dibuang sebelum dipakai memutuskan.",
    fields: [
      {
        key: "kick_distance_from_radius",
        label: "Ukur jarak dari radius bola",
        type: "switch",
        artinya:
          "Mati = jarak dihitung dari SUDUT SENDI KEPALA + geometri. Aktif = jarak dihitung dari besar bola di gambar (cara lama).",
        fungsi: "Memilih salah satu dari dua pengukur jarak yang tersedia.",
        tuning:
          "Biarkan MATI. Radius dari YOLO menyusut justru saat bola sudah dekat kaki, karena badan robot menutupi sebagian bola.",
        risk:
          "Diaktifkan berarti jarak ikut goyang tiap kali kotak deteksi berubah ukuran, dan robot berhenti di tempat yang tidak konsisten.",
      },
      {
        key: "kick_min_depression_deg",
        label: "Sudut tunduk minimum",
        unit: "deg",
        step: 0.5,
        artinya: "Bacaan dengan sudut tunduk di bawah angka ini DIBUANG.",
        fungsi:
          "Kepala yang hampir mendatar memberi tan() mendekati nol, jadi jaraknya meledak jadi puluhan meter. Ambang ini membuang bacaan seperti itu dan sekalian menahan pemicu tendangan.",
        tuning:
          "3 derajat sudah aman. Naikkan kalau di log terlihat jarak melonjak besar saat kepala sedang menyapu ke atas.",
        risk: "Terlalu besar berarti bola jauh tidak pernah terbaca sama sekali.",
      },
      {
        key: "kick_distance_filter_n",
        label: "Panjang penapis median",
        unit: "sampel",
        step: 1,
        artinya: "Berapa bacaan jarak terakhir yang dimedian sebelum dipakai.",
        fungsi:
          "Membuang satu-dua bacaan liar tanpa memperlambat tanggapan seperti rata-rata biasa.",
        tuning: "5 adalah titik aman. Isi 1 untuk MEMATIKAN penapis. Dibulatkan ke bawah oleh robot.",
        risk: "Terlalu panjang membuat jarak tertinggal saat robot mendekat cepat.",
      },
      {
        key: "kick_ready_count",
        label: "Siklus siap sebelum menendang",
        unit: "siklus",
        step: 1,
        artinya: "Berapa siklus berturut-turut syarat tendangan harus terpenuhi.",
        fungsi: "Mencegah satu frame kebetulan langsung memicu tendangan.",
        tuning: "4 adalah bawaan. Turunkan ke 2 kalau tendangan terasa lambat menyusul. Dibulatkan ke bawah oleh robot.",
        risk: "Terlalu kecil berarti derau satu frame sudah cukup memicu tendangan.",
      },
    ],
  },
  {
    title: "Ambang radius bola",
    description:
      "Syarat tambahan berbasis besar bola di gambar. Dihitung SENDIRI dari jarak tendang + tinggi kamera selama kolom piksel diisi 0.",
    fields: [
      {
        key: "kick_ball_radius_px",
        label: "Ambang radius (manual)",
        unit: "px",
        step: 1,
        artinya: "Ambang besar bola di gambar. 0 berarti HITUNG SENDIRI.",
        fungsi:
          "Kalau diisi angka > 0, angka itulah yang dipakai dan hasil hitungan diabaikan. Kalau 0, robot menghitungnya dari jarak tendang, tinggi kamera, jari-jari bola, dan panjang fokus.",
        tuning:
          "Biarkan 0. Ambang otomatis ikut benar sendiri saat robot naik/turun gantungan; angka tetap tidak.",
        risk:
          "Angka tetap yang disalin dari kondisi berdiri akan terlalu menuntut saat robot digantung -- bolanya harus diangkat dulu supaya cukup besar.",
      },
      {
        key: "kick_ball_radius_margin",
        label: "Potongan ambang radius",
        unit: "pecahan",
        step: 0.05,
        artinya: "Berapa bagian ambang ideal yang dipotong. 0,10 = ambangnya 90 % dari hitungan geometri.",
        fungsi:
          "Radius TERUKUR sistematis lebih kecil daripada hitungan geometri justru saat bola sudah dekat kaki: badan robot menutupi sebagian bola dan kotak YOLO ikut mengecil.",
        tuning:
          "Terukur 2026-08-28 dari 261 sampel: dengan ambang ideal utuh robot TIDAK PERNAH menendang. Dijepit robot ke rentang 0 sampai 0,9.",
        risk: "Terlalu besar berarti bola yang masih jauh pun sudah dianggap dekat.",
      },
      {
        key: "kick_radius_ready_enable",
        label: "Radius ikut memutuskan",
        type: "switch",
        artinya:
          "Mati = ambang radius tetap dihitung dan dicetak di telemetri tapi TIDAK ikut memutuskan apa pun.",
        fungsi: "Menyalakan pemicu radius yang berdiri sendiri (OR), bukan pagar tambahan yang memperketat syarat jarak.",
        tuning:
          "Biarkan MATI sampai radius detektor benar-benar stabil. Nyalakan hanya untuk menambah ketelitian setelah jarak sudah rapi.",
        risk:
          "Radius yang melonjak dapat memicu tendangan terlalu dini walaupun syarat jarak belum terpenuhi.",
      },
    ],
  },
];

// Cermin persis rumus di BallFollower::loadTunableParameters(). Dipakai supaya
// operator melihat AKIBAT angka yang baru diketik sebelum menekan kirim --
// tanpa ini, satu-satunya cara mengetahui ambang radius yang baru adalah
// membaca log robot sesudah demo diluncurkan ulang.
function computeKickGeometry(params) {
  const num = (key) => parseWalkingNumber(params[key], NaN);
  const distance = num("kick_distance");
  const cameraHeight = num("kick_camera_height");
  const ballRadius = num("kick_ball_real_radius_m");
  const fovDeg = num("kick_fov_width_deg");
  const imageWidth = num("kick_image_width_px");
  const manualRadiusPx = num("kick_ball_radius_px");
  const minDepressionDeg = num("kick_min_depression_deg");

  let margin = num("kick_ball_radius_margin");
  if (margin < 0) margin = 0;
  if (margin > 0.9) margin = 0.9;

  const tanFov = Math.tan((fovDeg * Math.PI) / 180);
  const focalPx = tanFov !== 0 ? (imageWidth * 0.5) / tanFov : 0;
  // Kamera mengunci PUSAT bola, jadi beda tingginya dikurangi jari-jari bola.
  const drop = cameraHeight - ballRadius;
  const slant = Math.sqrt(distance * distance + drop * drop);
  const idealPx = slant > 0 ? (focalPx * ballRadius) / slant : 0;
  const autoThresholdPx = idealPx * (1 - margin);
  const usingManual = manualRadiusPx > 0;

  const tanMin = Math.tan((minDepressionDeg * Math.PI) / 180);
  return {
    focalPx,
    drop,
    slant,
    idealPx,
    margin,
    usingManual,
    thresholdPx: usingManual ? manualRadiusPx : autoThresholdPx,
    // Sudut tunduk kepala yang seharusnya terbaca saat bola tepat di jarak tendang.
    depressionDeg: (Math.atan2(drop, distance) * 180) / Math.PI,
    // Di luar jarak ini sudut tunduknya sudah di bawah ambang dan bacaannya dibuang.
    maxReadableM: tanMin > 0 ? drop / tanMin : NaN,
  };
}

function normalizeKickParams(params = {}) {
  const next = { ...KICK_DEFAULT_PARAMS, ...params };
  KICK_PARAM_KEYS.forEach((key) => {
    next[key] = parseWalkingNumber(next[key], KICK_DEFAULT_PARAMS[key]);
  });
  return next;
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
    <div className={`studio-stat-card p-5 rounded-2xl border shadow-sm transition-all hover:shadow-md ${statusStyles[status]}`}>
      <div className="flex justify-between items-center mb-2 text-gray-500">
        <span className="text-sm font-medium">{title}</span>
        {Icon && <span className="studio-stat-icon"><Icon size={18} /></span>}
      </div>
      <div>
        <span className="block text-2xl font-bold font-display text-gray-900">{value}</span>
        {subValue && <span className="text-xs text-gray-500 font-mono">{subValue}</span>}
      </div>
    </div>
  );
};

const SectionHeader = ({ title, children }) => (
  <div className="studio-section-header flex justify-between items-center mb-4">
    <h2 className="text-lg font-bold font-display text-gray-800">{title}</h2>
    <div className="flex gap-2">{children}</div>
  </div>
);

const STUDIO_PAGE_DETAILS = {
  dashboard: { title: "Dashboard", description: "Kondisi robot dan kontrol utama dalam satu pandangan.", group: "Monitor" },
  charts: { title: "Charts", description: "Amati perubahan data dan bandingkan telemetri robot.", group: "Monitor" },
  vision: { title: "Vision", description: "Pantau kamera dan hasil deteksi objek.", group: "Monitor" },
  tuning: { title: "Tuning", description: "Sesuaikan offset sendi dan kontrol robot.", group: "Kontrol robot" },
  walking: { title: "Walking", description: "Kelola preset, periksa parameter, dan atur langkah robot.", group: "Kontrol robot" },
  rl: { title: "RL Training", description: "Latih parameter walking di robot asli dengan penilaian operator.", group: "Kontrol robot" },
  tendangan: { title: "Tendangan", description: "Periksa posisi bola dan parameter kesiapan tendangan.", group: "Kontrol robot" },
  demo: { title: "Demo", description: "Pilih mode dan kendalikan rangkaian demo robot.", group: "Kontrol robot" },
  teleop: { title: "Teleop", description: "Kendali gerakan dan pemantauan input joystick.", group: "Kontrol robot" },
  action: { title: "Action Editor", description: "Susun halaman gerakan, langkah, dan pratinjau pose.", group: "Kontrol robot" },
  logs: { title: "Logs", description: "Telusuri peristiwa dan pesan dari sistem.", group: "Sistem" },
  terminal: { title: "Terminal", description: "Akses terminal sistem di dalam workspace Studio.", group: "Sistem" },
};

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
  const [walkingRelativeDisplay, setWalkingRelativeDisplay] = useState(true);
  const [walkingCurrentParams, setWalkingCurrentParams] = useState(null);
  const [walkingLoaded, setWalkingLoaded] = useState(false);
  const walkingDirty = !parametersEqual(walkingParams, walkingCurrentParams || WALKING_DEFAULT_PARAMS, WALKING_KEYS);
  const [walkingBusy, setWalkingBusy] = useState(false);
  const [walkingLastAppliedAt, setWalkingLastAppliedAt] = useState(null);
  const [showWalkingAdvanced, setShowWalkingAdvanced] = useState(false);
  const [walkingError, setWalkingError] = useState("");
  const [torqueSpeedKey, setTorqueSpeedKey] = useState(DEFAULT_TORQUE_SPEED_KEY);
  // Batas kecepatan servo TIDAK lagi dilepas sendiri oleh pewaktu.
  //
  // Dulu ia dikembalikan ke kecepatan penuh 5 detik sesudah Torque ON. Dalam
  // pemakaian nyata, gerakan yang ingin diperlambat operator justru terjadi
  // SESUDAH itu (Init Pose ditekan beberapa detik kemudian), jadi batasnya
  // sudah kedaluwarsa sebelum sempat berguna -- persis keluhan "sudah dipilih
  // lambat masih juga kencang". Sekarang batasnya bertahan sampai dilepas
  // dengan sengaja, atau sampai sesuatu yang memang butuh kecepatan penuh
  // dimulai (jalan atau demo).
  const [torqueLimitActive, setTorqueLimitActive] = useState(false);
  const [walkingVersions, setWalkingVersions] = useState(() =>
    seedJalanWalkingVersion(seedWalkingVersions(readWalkingVersions()))
  );
  const [walkingVersionName, setWalkingVersionName] = useState("");
  const [activeWalkingVersionId, setActiveWalkingVersionId] = useState("");
  // Versi tersimpan tinggal di DISK robot (server/walkingVersionFile.js), bukan
  // di localStorage. Sampai penyimpanan itu selesai dibaca, Save dan Hapus
  // dimatikan: menulis sebelum membaca adalah persis pola "default menimpa
  // yang tersimpan".
  const [walkingStoreStatus, setWalkingStoreStatus] = useState("loading");
  const [walkingStoreError, setWalkingStoreError] = useState("");
  const [walkingSaving, setWalkingSaving] = useState(false);
  const initialLocalWalkingVersionsRef = useRef(walkingVersions);
  const initialWalkingDraftRef = useRef(walkingParams);
  const activeWalkingVersion = walkingVersions.find((version) => version.id === activeWalkingVersionId) || null;
  const walkingUnsaved = walkingDraftUnsaved({
    draft: walkingParams, activeVersion: activeWalkingVersion, defaults: WALKING_DEFAULT_PARAMS,
  });
  // Tendangan (parameter ROS milik demo_node, bukan WalkingParam)
  const [kickParams, setKickParams] = useState(() => normalizeKickParams());
  const [kickCurrentParams, setKickCurrentParams] = useState(null);
  const [kickLoaded, setKickLoaded] = useState(false);
  const kickDirty = !parametersEqual(kickParams, kickCurrentParams || KICK_DEFAULT_PARAMS, KICK_PARAM_KEYS);
  const [kickBusy, setKickBusy] = useState(false);
  const [kickError, setKickError] = useState("");
  const [kickLastAppliedAt, setKickLastAppliedAt] = useState(null);
  // Dua langkah, sengaja: tombol ini menulis berkas sumber yang dilacak git,
  // beda kelas dengan tombol lain di halaman yang cuma mengubah RAM robot.
  const [kickSaveArmed, setKickSaveArmed] = useState(false);
  // Kamera di halaman Tendangan. Mematikannya benar-benar MEMUTUS langganan,
  // bukan sekadar berhenti menggambar: saat menera jarak, gambar 640x480 mentah
  // lewat rosbridge itu beban yang tidak perlu kalau memang tidak dilihat.
  const [kickCameraOn, setKickCameraOn] = useState(true);
  // PENGUKUR JARAK: pengamatan MENTAH dari robot (lingkaran bola + sudut leher).
  // Sengaja disimpan mentah, bukan sudah jadi jarak: jaraknya dihitung ulang
  // tiap render dengan parameter yang SEDANG DIKETIK, supaya operator melihat
  // akibat draftnya sebelum mengirim apa pun.
  const [kickObservation, setKickObservation] = useState(null);
  const [kickHipPitch, setKickHipPitch] = useState(null);
  const [kickSavedAt, setKickSavedAt] = useState(null);
  const [kickSaveMessage, setKickSaveMessage] = useState("");
  // Apakah node yang dibutuhkan halaman Tendangan dan Demo benar-benar hidup.
  // null = belum pernah diperiksa. Tanpa ini, "tombolnya ditekan tapi robot
  // diam saja" tidak bisa dibedakan dari "demonya memang belum diluncurkan".
  const [rosNodePresence, setRosNodePresence] = useState(null);
  const [rosNodeCheckedAt, setRosNodeCheckedAt] = useState(null);
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
  // Bawaan mengikuti config/yolo_ball_detector_openvino_params.yaml, supaya
  // tampilan sebelum tombol Muat tidak menyesatkan.
  const [yoloParams, setYoloParams] = useState({
    confidence_threshold: 0.3,
    iou_threshold: 0.45,
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
  const kickCanvasRef = useRef(null);
  const kickCameraOnRef = useRef(true);
  // Sudut leher terbaru; dipasangkan ke lingkaran bola begitu deteksi tiba.
  const kickHeadRef = useRef(null);
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
  // Daftar sendi CADANGAN, langsung dari robot.
  //
  // Torque ON/OFF dan batas kecepatannya dulu hanya memakai metrics.joint_names
  // dari studio_agent. Begitu agent itu mati -- dan ia memang mati tiap boot
  // karena NameError di studio_agent.py -- metrics jadi null, writeSyncItem()
  // berhenti sebelum menerbitkan apa pun, dan tombol Torque cuma menjawab
  // "Torque command unavailable". Batas kecepatan yang membuat torsi menyala
  // dengan santai ikut tidak pernah terkirim. /robotis/present_joint_states
  // datang dari robotis_controller sendiri, jadi selama robotnya hidup daftar
  // ini ada.
  const robotJointNamesRef = useRef(null);
  const jointNameSubRef = useRef(null);
  const setJointStatesPubRef = useRef(null);
  const headModulePubRef = useRef(null);
  const walkingModulePubRef = useRef(null);
  const walkingModuleEnabledRef = useRef(false);
  const walkingOperationRef = useRef(null);
  const kickOperationRef = useRef(null);
  const motionSequenceRef = useRef(0);
  const simStartTimerRef = useRef(null);
  const walkingModuleGetRef = useRef(null);
  const yoloSetServiceRef = useRef(null);
  const yoloGetServiceRef = useRef(null);
  const snapshotServiceRef = useRef(null);
  const healthCheckServiceRef = useRef(null);
  const demoModePubRef = useRef(null);
  const demoCommandPubRef = useRef(null);
  const kickGetServiceRef = useRef(null);
  const kickSetServiceRef = useRef(null);
  const kickSaveServiceRef = useRef(null);
  const kickLaunchServiceRef = useRef(null);

  const runRobotOperation = async (kind, action) => {
    const operationRef = kind === "walking" ? walkingOperationRef : kickOperationRef;
    const setBusy = kind === "walking" ? setWalkingBusy : setKickBusy;
    const setError = kind === "walking" ? setWalkingError : setKickError;
    if (operationRef.current) return false;
    const controller = new AbortController();
    const ros = rosRef.current;
    operationRef.current = controller;
    setBusy(true);
    setError("");
    const check = () => {
      if (controller.signal.aborted || ros !== rosRef.current || !ros?.isConnected) {
        throw new Error("Operasi dibatalkan atau koneksi ROS terputus");
      }
    };
    const context = {
      check,
      wait: (duration) => waitForRobot(duration, controller.signal),
      call: async (service, args, options) => {
        check();
        if (service?.ros !== ros) throw new Error("Service berasal dari koneksi ROS yang berbeda");
        const result = await callRosService(service, args, { ...options, signal: controller.signal });
        check();
        return result;
      },
    };
    try {
      check();
      await action(context);
      return true;
    } catch (error) {
      if (!controller.signal.aborted) {
        setError(error.message);
        sendStatus(error.message, true);
      }
      return false;
    } finally {
      if (operationRef.current === controller) {
        operationRef.current = null;
        setBusy(false);
      }
    }
  };

  const cancelPendingMotion = () => {
    motionSequenceRef.current += 1;
    walkingOperationRef.current?.abort();
    walkingOperationRef.current = null;
    setWalkingBusy(false);
    clearTimeout(simStartTimerRef.current);
    simStartTimerRef.current = null;
  };

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
    kickCameraOnRef.current = kickCameraOn;
  }, [kickCameraOn]);

  // Salinan cadangan di localStorage, HANYA untuk dibaca saat server penyimpanan
  // tidak terjangkau. Tidak pernah dipakai untuk menimpa disk robot, dan gagal
  // menulisnya (kuota penuh) bukan kegagalan Save.
  const mirrorWalkingVersionsLocally = (state) => {
    try {
      localStorage.setItem(WALKING_VERSION_STORAGE_KEY, JSON.stringify(state.versions));
      localStorage.setItem(WALKING_ACTIVE_VERSION_KEY, state.activeVersionId || "");
    } catch (error) {
      console.warn("[SAVE] salinan cadangan di browser gagal ditulis; disk robot tetap yang utama:", error?.name || error);
    }
  };

  // Daftar versi selalu diambil dari JAWABAN server, bukan dari yang kita kira
  // sudah tersimpan. Nilainya dipakai apa adanya, tanpa normalisasi.
  const takeWalkingStoreState = (state) => {
    setWalkingVersions(state.versions);
    mirrorWalkingVersionsLocally(state);
  };

  // Mengisi EDITING STATE dari snapshot tersimpan saat Studio dibuka -- satu
  // satunya penulis isian yang bukan tindakan operator langsung.
  //
  // Hanya mengganti isian yang BELUM disentuh operator: kalau pembacaan
  // penyimpanan lambat dan operator sudah sempat mengetik, ketikannya menang.
  const restoreWalkingDraft = (params) => {
    const draft = normalizeWalkingParams(params);
    setWalkingParams((current) => (current === initialWalkingDraftRef.current ? draft : current));
  };

  // URUTAN STARTUP: baca penyimpanan -> periksa -> pulihkan versi aktif -> UI.
  //
  // Dulu isian Walking dibuat dari WALKING_DEFAULT_PARAMS dan versi yang
  // terakhir dipakai tidak pernah diingat, jadi setiap kali Studio dibuka isian
  // menampilkan default walaupun versi tersimpannya utuh. Default sekarang HANYA
  // dipakai kalau memang tidak ada versi aktif yang tersimpan. Tidak ada yang
  // dikirim ke robot di sini: Apply tetap tombol operator.
  useEffect(() => {
    let cancelled = false;
    (async () => {
      console.info("[LOAD] loading persisted walking parameters");
      try {
        let state = await fetchWalkingStore();
        if (!state.exists) {
          const local = initialLocalWalkingVersionsRef.current;
          console.info(`[LOAD] penyimpanan robot belum ada -> memindahkan ${local.length} versi dari browser ini`);
          state = await mutateWalkingStore({ op: "import", versions: local, activeVersionId: readLocalActiveVersionId() });
          if (state.result?.rejected?.length) console.warn("[LOAD] versi browser yang ditolak:", state.result.rejected);
        }
        if (cancelled) return;
        takeWalkingStoreState(state);
        if (typeof state.ui?.relativeDisplay === "boolean") setWalkingRelativeDisplay(state.ui.relativeDisplay);
        const restored = findRestorableVersion(state);
        if (restored) {
          restoreWalkingDraft(restored.params);
          setActiveWalkingVersionId(restored.id);
          setWalkingVersionName(restored.name);
          console.info(`[LOAD] action = ${restored.name}`);
          console.info(`[LOAD] parameters = ${JSON.stringify(restored.params)}`);
          sendStatus(`Versi terakhir dipulihkan: ${restored.name} (disimpan ${new Date(restored.updatedAt).toLocaleString()})`);
        } else {
          console.info("[LOAD] no persisted data found -> using default");
        }
        setWalkingStoreStatus("ready");
      } catch (error) {
        if (cancelled) return;
        console.error("[LOAD] penyimpanan versi di robot tidak terbaca:", error);
        setWalkingStoreError(error.message);
        setWalkingStoreStatus("offline");
      }
    })();
    return () => {
      cancelled = true;
    };
  }, []);

  // Menutup Studio dengan isian yang belum di-Save: browser bertanya dulu,
  // supaya "kukira sudah tersimpan" tidak bisa terjadi diam-diam.
  useEffect(() => {
    if (!walkingUnsaved || walkingStoreStatus !== "ready") return undefined;
    const warn = (event) => {
      event.preventDefault();
      event.returnValue = "";
    };
    window.addEventListener("beforeunload", warn);
    return () => window.removeEventListener("beforeunload", warn);
  }, [walkingUnsaved, walkingStoreStatus]);

  // Records that the "dari claude" preset has been offered to this browser, so
  // deleting it makes it stay deleted instead of returning on the next reload.
  useEffect(() => {
    try {
      localStorage.setItem(WALKING_SEED_FLAG_KEY, String(Date.now()));
    } catch {
      // Same restricted-storage case as above; the preset simply seeds again.
    }
  }, []);

  // Sama untuk penyegaran "JALAN KE SURGA": begitu penandanya tertulis, seed
  // tidak menyentuh slot itu lagi, jadi Simpan berikutnya dari operator tetap
  // menang dan tidak akan tertimpa saat halaman dimuat ulang.
  useEffect(() => {
    try {
      localStorage.setItem(JALAN_WALKING_SEED_FLAG_KEY, String(Date.now()));
    } catch {
      // Penyimpanan dibatasi; seed akan jalan lagi di muat berikutnya.
    }
  }, []);

  useEffect(() => {
    if (!rosUrl) return;

    const ros = new ROSLIB.Ros({ url: rosUrl });
    rosRef.current = ros;

    const resetRuntime = (state) => {
      cancelPendingMotion();
      kickOperationRef.current?.abort();
      kickOperationRef.current = null;
      setKickBusy(false);
      setWalkingLoaded(false);
      setWalkingCurrentParams(null);
      setKickLoaded(false);
      setKickCurrentParams(null);
      setKickSaveArmed(false);
      setKickSaveMessage("");
      walkingModuleEnabledRef.current = false;
      setRosState(state);
    };
    ros.on("connection", () => resetRuntime("connected"));
    ros.on("error", () => resetRuntime("error"));
    ros.on("close", () => resetRuntime("disconnected"));

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

    // Cuma untuk mengambil DAFTAR NAMA sendi, bukan sudutnya: 2 detik sekali
    // sudah lebih dari cukup, dan rosbridge yang membatasi di sisi server.
    jointNameSubRef.current = new ROSLIB.Topic({
      ros,
      name: "/robotis/present_joint_states",
      messageType: "sensor_msgs/JointState",
      throttle_rate: 2000,
      queue_length: 1,
    });
    jointNameSubRef.current.subscribe((msg) => {
      if (Array.isArray(msg.name) && msg.name.length) robotJointNamesRef.current = msg.name;
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

    // Menyetel goal_position_ controller langsung, dalam RADIAN -- controller
    // yang mengubahnya ke satuan servo berikut offset per sendi. Hanya
    // berpengaruh pada sendi yang modulnya "none" (robotis_controller.cpp:2082),
    // dan itu memang keadaan yang dipakai Torque ON di bawah.
    setJointStatesPubRef.current = new ROSLIB.Topic({
      ros,
      name: "/robotis/set_joint_states",
      messageType: "sensor_msgs/JointState",
    });

    yoloSetServiceRef.current = new ROSLIB.Service({
      ros,
      name: "/ball_detector_node/yolo_ball_detector/set_parameters",
      serviceType: "rcl_interfaces/srv/SetParameters",
    });
    yoloGetServiceRef.current = new ROSLIB.Service({
      ros,
      name: "/ball_detector_node/yolo_ball_detector/get_parameters",
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
    walkingModuleGetRef.current = new ROSLIB.Service({
      ros,
      name: "/robotis/get_present_joint_ctrl_modules",
      serviceType: "robotis_controller_msgs/srv/GetJointModule",
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

    // Parameter tendangan hidup di node demo, jadi jalurnya service parameter
    // ROS biasa -- bukan topik seperti WalkingParam.
    //
    // Nama NODE-nya "demo_node"; "op_demo_node" itu nama executable-nya dan
    // launch tidak menimpanya dengan atribut name. Salah satu huruf saja di
    // sini membuat service-nya tidak pernah ketemu dan halaman ini diam-diam
    // tidak berfungsi. Diverifikasi di robot: ros2 service list menampilkan
    // /demo_node/get_parameters dan /demo_node/set_parameters.
    //
    // Node-nya baru ada sesudah demo diluncurkan; sebelum itu panggilannya
    // gagal dan halaman Tendangan mengatakannya apa adanya.
    kickGetServiceRef.current = new ROSLIB.Service({
      ros,
      name: "/demo_node/get_parameters",
      serviceType: "rcl_interfaces/srv/GetParameters",
    });
    kickSetServiceRef.current = new ROSLIB.Service({
      ros,
      name: "/demo_node/set_parameters_atomically",
      serviceType: "rcl_interfaces/srv/SetParametersAtomically",
    });

    // Menyimpan permanen ke demo.launch.xml. Dilayani studio_agent, bukan
    // demo_node: yang menulis berkas harus proses yang punya akses berkas, dan
    // agent membaca sendiri angka yang hidup di demo_node supaya yang tersimpan
    // dijamin sama dengan yang barusan dicoba.
    kickSaveServiceRef.current = new ROSLIB.Service({
      ros,
      name: "/bascorro_studio/save_kick_to_launch",
      serviceType: "std_srvs/srv/Trigger",
    });
    kickLaunchServiceRef.current = new ROSLIB.Service({
      ros,
      name: "/bascorro_studio/get_kick_launch",
      serviceType: "std_srvs/srv/Trigger",
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
      cancelPendingMotion();
      kickOperationRef.current?.abort();
      metricsSubRef.current?.unsubscribe();
      eventsSubRef.current?.unsubscribe();
      joySubRef.current?.unsubscribe();
      teleopStatusSubRef.current?.unsubscribe();
      ros.close();
    };
  }, [rosUrl]);

  // Satu langganan gambar dipakai DUA halaman: Vision dan Tendangan.
  //
  // Dulu langganannya selalu hidup dan yang dilakukan saklar cuma berhenti
  // menggambar -- gambarnya tetap mengalir lewat rosbridge. Sekarang saklarnya
  // menentukan langganannya sendiri, jadi "matikan kamera" benar-benar
  // menghentikan aliran datanya. Sengaja tetap SATU langganan, bukan satu per
  // halaman: keduanya tidak pernah aktif bersamaan (halaman aktif cuma satu),
  // dan dua langganan ke topik yang sama akan menggandakan lalu lintasnya.
  const overlayWanted =
    (activeTab === "vision" && showOverlay) ||
    (activeTab === "tendangan" && kickCameraOn);

  useEffect(() => {
    if (!rosRef.current || rosState === "disconnected") return;

    if (overlaySubRef.current) overlaySubRef.current.unsubscribe();
    if (!overlayWanted) return undefined;

    // Dibersihkan supaya "belum ada gambar sama sekali" tidak tertutup oleh
    // cap waktu frame terakhir dari langganan sebelumnya.
    overlayLastRef.current = { stampMs: null, fps: 0, dropped: 0 };

    overlaySubRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: overlayTopic,
      messageType: "sensor_msgs/Image",
    });

    overlaySubRef.current.subscribe((msg) => {
      // Kanvasnya beda per halaman; yang dipakai adalah milik halaman yang
      // sedang terbuka, dan hanya kalau saklar halaman itu menyala.
      const tab = activeTabRef.current;
      let canvas = null;
      if (tab === "vision" && showOverlayRef.current) canvas = overlayCanvasRef.current;
      else if (tab === "tendangan" && kickCameraOnRef.current) canvas = kickCanvasRef.current;
      if (!canvas) return;

      const imageData = decodeImage(msg);
      if (!imageData) return;

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
  }, [overlayTopic, rosState, overlayWanted]);

  // Saat gambar BERHENTI datang, tidak ada lagi yang memicu render -- tanpa
  // pencacah ini panel kamera akan diam hitam selamanya tanpa keterangan, yang
  // persis kejadian saat topiknya memang tidak ada yang menerbitkan.
  const [cameraTick, setCameraTick] = useState(0);
  useEffect(() => {
    if (!overlayWanted) return undefined;
    const id = window.setInterval(() => setCameraTick((n) => n + 1), 1000);
    return () => window.clearInterval(id);
  }, [overlayWanted]);

  // ---- PENGUKUR JARAK TENDANGAN -----------------------------------------
  //
  // Dua langganan RINGAN, hidup hanya selama halaman Tendangan terbuka:
  // sudut leher dan lingkaran bola. Keduanya dibatasi 200 ms lewat
  // throttle_rate rosbridge -- /robotis/present_joint_states terbit 125 Hz
  // untuk 20 sendi, dan mengalirkan itu utuh ke browser tidak ada gunanya
  // untuk mengukur bola yang diam.
  //
  // Yang disimpan pengamatan MENTAH. Jaraknya dihitung di render.
  useEffect(() => {
    if (!rosRef.current || rosState !== "connected" || activeTab !== "tendangan") {
      return undefined;
    }
    const ros = rosRef.current;

    const jointSub = new ROSLIB.Topic({
      ros, name: "/robotis/present_joint_states", messageType: "sensor_msgs/JointState",
      throttle_rate: 200, queue_length: 1,
    });
    jointSub.subscribe((msg) => {
      const pan = msg.name.indexOf("head_pan");
      const tilt = msg.name.indexOf("head_tilt");
      if (pan < 0 || tilt < 0) return;
      kickHeadRef.current = { pan: msg.position[pan], tilt: msg.position[tilt] };
    });

    const circleSub = new ROSLIB.Topic({
      ros, name: "/ball_detector_node/circle_set",
      messageType: "op3_ball_detector_msgs/CircleSetStamped",
      throttle_rate: 200, queue_length: 1,
    });
    circleSub.subscribe((msg) => {
      const head = kickHeadRef.current;
      // Deteksi tanpa sudut leher tidak bisa dipakai: seluruh geometrinya
      // bergantung ke mana kepala menghadap.
      if (!head) return;
      if (!msg.circles || msg.circles.length === 0) {
        setKickObservation((prev) => (prev ? { ...prev, circle: null, at: Date.now() } : prev));
        return;
      }
      // Detektor sudah mengurutkan bola terbesar di depan.
      const c = msg.circles[0];
      setKickObservation({
        circle: { x: c.x, y: c.y, radiusPx: c.z },
        head, at: Date.now(),
      });
    });

    return () => {
      jointSub.unsubscribe();
      circleSub.unsubscribe();
      kickHeadRef.current = null;
    };
  }, [rosState, activeTab]);

  // hip_pitch_offset ikut menentukan arah pandang kamera, dan ia milik
  // walking_module -- bukan parameter tendangan. Dibaca sekali saat halaman
  // dibuka; kalau gagal (manager mati), pengukurannya sengaja tidak ditampilkan
  // daripada memakai tebakan.
  useEffect(() => {
    if (rosState !== "connected" || activeTab !== "tendangan") return undefined;
    let batal = false;
    (async () => {
      try {
        const result = await callRosService(walkingGetServiceRef.current,
          { get_param: true }, { timeoutMs: 4000 });
        if (!batal) setKickHipPitch(Number(result?.parameters?.hip_pitch_offset));
      } catch {
        if (!batal) setKickHipPitch(null);
      }
    })();
    return () => { batal = true; };
  }, [rosState, activeTab]);

  // Auto-enable head_control_module on the real robot so head torque comes
  // up as soon as the manager is reachable. Without this, WALKING_READY's head
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
    if (command === "stop") cancelPendingMotion();
    if (!walkingCommandPubRef.current || !rosRef.current?.isConnected) {
      sendStatus("ROS terputus: perintah walking tidak dikirim", true);
      return false;
    }
    walkingCommandPubRef.current.publish(new ROSLIB.Message({ data: command }));
    sendStatus(label || "Perintah walking terkirim: " + command);
    return true;
  };

  const readWalkingRuntime = async (context) => {
    const result = await context.call(walkingGetServiceRef.current, { get_param: true });
    // Dirapikan di SATU tempat: semua jalur baca walking lewat sini, dan hasilnya
    // masuk ke kotak isian lewat applyWalkingDraft(). Tanpa ini tiap Apply
    // menuliskan angka float32 yang sudah berubah jadi double panjang ke layar.
    const params = snapWalkingRuntime(validateWalkingParams(result?.parameters));
    setWalkingCurrentParams(params);
    setWalkingLoaded(true);
    return params;
  };

  const walkingOwnsLegs = async (context) => {
    const result = await context.call(walkingModuleGetRef.current, { joint_name: WALKING_LEG_JOINTS });
    const modules = new Map(result?.joint_name?.map((name, index) => [name, result.module_name?.[index]]) || []);
    const enabled = WALKING_LEG_JOINTS.every((name) => modules.get(name) === "walking_module");
    walkingModuleEnabledRef.current = enabled;
    return enabled;
  };

  const sendBalanceCommand = (enable) => runRobotOperation("walking", async (context) => {
    if (!await walkingOwnsLegs(context)) throw new Error("Balance belum dikirim: aktifkan walking_module terlebih dahulu, tanpa menekan Start");
    context.check();
    sendWalkingCommand(enable ? "balance on" : "balance off");
    await context.wait(200);
    const current = await readWalkingRuntime(context);
    if (current.balance_enable !== enable) throw new Error("Balance belum terkonfirmasi; periksa penulis parameter lain");
    setWalkingParams((previous) => ({ ...previous, balance_enable: current.balance_enable }));
    sendStatus("Balance IMU terkonfirmasi: " + (enable ? "ON" : "OFF"));
  });

  const saveWalkingRuntime = () => runRobotOperation("walking", async (context) => {
    if (!walkingLoaded || walkingDirty) throw new Error("Load dan Apply perubahan terlebih dahulu; Save menyimpan runtime, bukan draft");
    const current = await readWalkingRuntime(context);
    if (!parametersEqual(current, walkingParams, WALKING_KEYS)) throw new Error("Runtime berubah sejak Load. Periksa nilai robot sebelum menyimpan");
    if (!await walkingOwnsLegs(context)) throw new Error("Save belum dikirim: walking_module belum aktif");
    context.check();
    sendWalkingCommand("save", "Perintah simpan runtime dikirim; periksa log walking untuk hasil penulisan file");
  });

  const sendTeleopCommand = (command, label) => {
    if (!teleopCommandPubRef.current || rosState !== "connected") {
      sendStatus("Teleop command unavailable", true);
      return false;
    }
    teleopCommandPubRef.current.publish(new ROSLIB.Message({ data: command }));
    sendStatus(label || `Teleop command: ${command}`);
    return true;
  };

  const applyWalkingAndRefreshTeleop = async () => {
    if (await applyWalkingParams()) sendTeleopCommand("refresh_params", "Apply terkonfirmasi; refresh baseline teleop diminta");
  };

  const updateWalkingParam = (key, value) => {
    setWalkingParams((previous) => ({ ...previous, [key]: value }));
    setWalkingError("");
  };

  const loadWalkingParams = async () => {
    let params;
    const successful = await runRobotOperation("walking", async (context) => {
      params = await readWalkingRuntime(context);
      setWalkingParams(params);
      sendStatus("Parameter walking dimuat dari robot");
    });
    return successful ? params : null;
  };

  const applyWalkingWithContext = async (context, draft = walkingParams, baseline = walkingCurrentParams) => {
    // TIDAK perlu Load dulu.
    //
    // Dulu di sini ada `if (!baseline) throw "Load parameter dari robot terlebih
    // dahulu"`. Syarat itu sudah tidak ada gunanya sejak mergeWalkingEdits()
    // mengirim SELURUH draft: baseline tidak lagi menentukan sedikit pun isi
    // yang diterbitkan, ia cuma ikut divalidasi. Dan applyWalkingDraft() memang
    // membaca robot sendiri sebelum menerbitkan, lalu membaca lagi untuk
    // memverifikasi -- jadi pembacaan yang dulu dipaksa lewat tombol Load tetap
    // terjadi, cuma tidak lagi menghalangi.
    //
    // Akibatnya operator bisa memilih versi tersimpan lalu langsung Apply &
    // Start, tanpa singgah ke tombol Load di baris PARAMETER.
    const params = await applyWalkingDraft({
      draft,
      baseline: baseline ?? draft,
      read: () => readWalkingRuntime(context),
      publish: (payload) => {
        context.check();
        walkingParamPubRef.current.publish(new ROSLIB.Message(payload));
      },
      check: context.check,
      onRead: setWalkingCurrentParams,
      wait: context.wait,
    });
    // APPLY TIDAK MENYENTUH ISIAN OPERATOR.
    //
    // Dulu di sini ada setWalkingParams(params) -- hasil BACA ULANG dari robot
    // ditulis balik ke isian. Itu mencampur dua hal yang harus terpisah:
    //
    //   EDITING STATE  = isian operator, calon isi Save
    //   RUNTIME STATE  = apa yang sedang dipegang walking_module (kolom Current)
    //
    // Akibatnya runtime bisa menimpa nilai tersimpan tanpa operator mengetik
    // apa pun. Jalur yang terbukti: Walking Tuner di tab Tuning memakai
    // applyWalkingTuner() -> fungsi ini, dengan draft MILIKNYA SENDIRI. Begitu
    // Tuner menekan Apply, isian tab Walking ikut berubah jadi nilai Tuner,
    // dan Save berikutnya menuliskannya ke versi yang sedang terbuka.
    // walking_param_ di robot juga bisa digeser penulis lain (perintah
    // "balance on/off", demo, teleop), dan semuanya ikut mendarat di isian.
    //
    // Sekarang hasil pembacaan hanya masuk walkingCurrentParams. Isian tetap
    // milik operator, dan penanda "belum di-Apply"/"belum disimpan" jadi jujur:
    // kalau robot mengembalikan angka lain, itu TERLIHAT, bukan diam-diam
    // menggantikan yang diketik. Nilai yang diterbitkan sendiri sudah dijamin
    // sama dengan draft oleh applyWalkingDraft() -- ia mengulang pembacaan
    // sampai cocok, dan melempar kalau tidak pernah cocok.
    setWalkingCurrentParams(params);
    setWalkingLastAppliedAt(Date.now());
    console.info(`[APPLY] runtime confirmed = ${JSON.stringify(params)}`);
    sendStatus("Apply walking terkonfirmasi melalui pembacaan runtime");
    return params;
  };

  const applyWalkingParams = () => runRobotOperation("walking", (context) => applyWalkingWithContext(context));

  const applyWalkingTuner = async (draft, baseline, start) => {
    let params;
    const successful = await runRobotOperation("walking", async (context) => {
      if (start) await initThenEnableWalking(context, draft);
      params = await applyWalkingWithContext(context, draft, baseline);
      if (start) {
        context.check();
        sendWalkingCommand("start");
      }
    });
    return successful ? params : null;
  };

  // Nama sendi dari studio_agent kalau ada, kalau tidak dari robot langsung.
  const walkingJointNames = () => {
    if (metrics?.joint_names?.length) return metrics.joint_names;
    return robotJointNamesRef.current?.length ? robotJointNamesRef.current : null;
  };

  const writeSyncItem = (itemName, value) => {
    const names = walkingJointNames();
    if (!rosRef.current?.isConnected || !torquePubRef.current || !names) return false;
    torquePubRef.current.publish(new ROSLIB.Message({
      item_name: itemName,
      joint_name: names,
      value: names.map(() => value),
    }));
    return true;
  };

  // onDone SELALU dipanggil, juga saat urutannya dibatalkan atau ROS putus --
  // argumennya menyatakan apakah semua item sempat terkirim. Kalau pemanggil
  // menunggunya sebagai Promise (restoreFullSpeed), keluar diam-diam di sini
  // berarti Promise itu tidak pernah selesai dan Apply & Start menggantung.
  const writeSyncItemsSequential = (items, onDone) => {
    const sequence = motionSequenceRef.current;
    let idx = 0;
    const next = () => {
      if (sequence !== motionSequenceRef.current || !rosRef.current?.isConnected) {
        onDone?.(false);
        return;
      }
      if (idx >= items.length) { onDone?.(true); return; }
      const [name, val] = items[idx++];
      writeSyncItem(name, val);
      window.setTimeout(next, SYNC_ITEM_GAP_MS);
    };
    next();
  };

  // 0 = tanpa batas di tabel kontrol Dynamixel.
  //
  // Mengembalikan Promise: writeSyncItemsSequential menyela tiap item dengan
  // setTimeout, jadi tanpa menunggu, "start" bisa terkirim sebelum tulisannya
  // mendarat dan langkah pertama tetap terkekang.
  const restoreFullSpeed = (note) => {
    const selesai = new Promise((resolve) => {
      writeSyncItemsSequential([
        ["profile_velocity", 0],
        ["profile_acceleration", 0],
      ], resolve);
    });
    setTorqueLimitActive(false);
    if (note) sendStatus(note);
    return selesai;
  };

  // ACUAN POSE DIPATOK SEBELUM MODUL DINYALAKAN.
  //
  // walking_bias_ direkam SEKALI, pada tick pertama sesudah walking_module
  // diaktifkan (op3_walking_module.cpp:608-620):
  //
  //   walking_bias_ = pose_page_2 - neutral(nilai yang hidup saat itu)
  //   pose robot    = walking_bias_ + neutral(nilai sekarang)
  //                 = page 2 + neutral(sekarang) - neutral(saat direkam)
  //
  // Studio dulu mengirim parameter DULU baru menyalakan modul, jadi "saat
  // direkam" == "sekarang" dan seluruh offset init saling meniadakan: sesi
  // baru selalu berdiri persis di page 2, berapa pun angkanya. Di dalam satu
  // sesi modulnya sudah menyala, jadi perubahan berikutnya terasa penuh.
  // Akibatnya operator melihat "sesi 1 benar, sesi 2 bentuknya balik lagi",
  // dan angkanya terus dinaikkan tiap sesi tanpa pernah menetap.
  //
  // Sekarang acuannya dipatok ke offset page 2 yang tetap (sama dengan yang
  // dipakai tampilan Delta, dan sama dengan yang diterbitkan demo lewat
  // publishPoseReference()). Dengan begitu neutral(saat direkam) selalu sama,
  // jadi angka yang sama SELALU memberi bentuk yang sama -- di sesi mana pun.
  const publishWalkingPoseReference = async (context, draft) => {
    const payload = validateWalkingParams(draft);
    for (const key of WALKING_BIAS_FIELDS) payload[key] = WALKING_POSE_REFERENCE.offsets[key];
    for (let percobaan = 0; percobaan < 4; percobaan += 1) {
      context.check();
      walkingParamPubRef.current.publish(new ROSLIB.Message(payload));
      await context.wait(120);
      context.check();
      const runtime = await readWalkingRuntime(context);
      if (WALKING_BIAS_FIELDS.every((key) => parameterValuesEqual(runtime[key], payload[key]))) {
        console.info(`[APPLY] acuan pose dipatok sebelum modul menyala = ${JSON.stringify(
          Object.fromEntries(WALKING_BIAS_FIELDS.map((key) => [key, payload[key]])))}`);
        return;
      }
    }
    throw new Error("Acuan pose belum mendarat di walking_module. Start dibatalkan supaya "
      + "bentuk gerakannya tidak jadi acak seperti sebelum perbaikan acuan.");
  };

  // Nilai lama operator disetel saat offset init tidak pernah benar-benar
  // terasa, jadi banyak yang sudah terlanjur jauh. Sekarang angka itu menggeser
  // pose sepenuhnya -- tanyakan sekali sebelum robot bergerak.
  const confirmWalkingPoseOffset = (draft) => {
    const jauh = walkingPoseFarFromReference(draft, WALKING_POSE_REFERENCE.offsets);
    if (!jauh.length) return true;
    const daftar = jauh.map(({ key, selisih }) => `  ${key}: ${selisih > 0 ? "+" : ""}${Number(selisih.toPrecision(3))} dari page 2`).join("\n");
    return window.confirm(
      "Offset ini jauh dari page 2:\n" + daftar
      + "\n\nSejak acuannya dipatok, angka ini benar-benar menggeser pose robot "
      + "(sebelumnya tidak terasa di awal sesi). Robot bisa condong jauh.\n\n"
      + "Pegang robot dulu. Lanjutkan?");
  };

  const initThenEnableWalking = async (context, draft = walkingParams) => {
    context.check();
    // TANPA SYARAT. torqueLimitActive cuma state React: hilang begitu halaman
    // dimuat ulang, sementara batasnya hidup di RAM servo sampai daya servo
    // dimatikan. Jadi urutan "Torque ON -> reload Studio -> Apply & Start"
    // dulu berjalan dengan batas yang masih terpasang dan tidak ada satu pun
    // yang melepasnya. Menulis 0 saat tidak ada batas sama sekali tidak
    // berbahaya (0 = nilai pabrik), sedangkan melewatkannya melumpuhkan gait.
    await restoreFullSpeed();
    context.check();
    if (!await walkingOwnsLegs(context)) {
      context.check();
      // Harus mendarat SEBELUM modul menyala: biasnya direkam di tick pertama.
      await publishWalkingPoseReference(context, draft);
      context.check();
      walkingModulePubRef.current.publish(new ROSLIB.Message({ data: "walking_module" }));
      await context.wait(WALKING_ENABLE_SETTLE_MS);
      if (!await walkingOwnsLegs(context)) throw new Error("Walking module belum menguasai semua sendi kaki; Start dibatalkan");
    }
    context.check();
  };

  // Urutannya dibalik: modul dinyalakan dengan ACUAN yang hidup, baru nilai
  // operator dikirim. Kalau dibalik lagi, biasnya kembali menangkap nilai
  // operator sendiri dan offsetnya meniadakan diri seperti dulu.
  const applyWalkingAndStart = () => {
    if (!confirmWalkingPoseOffset(walkingParams)) return undefined;
    return runRobotOperation("walking", async (context) => {
      await initThenEnableWalking(context);
      await applyWalkingWithContext(context);
      context.check();
      sendWalkingCommand("start", "Start dikirim setelah acuan dipatok, Apply, dan modul walking terkonfirmasi");
    });
  };

  const startWalkingNow = (expected = walkingParams) => {
    if (!confirmWalkingPoseOffset(expected)) return undefined;
    return runRobotOperation("walking", async (context) => {
      const current = await readWalkingRuntime(context);
      if (!parametersEqual(current, expected, WALKING_KEYS)) {
        throw new Error("Robot belum memakai nilai ini. Tekan Apply (atau Apply & Start) dulu.");
      }
      // Menyalakan modul menerbitkan ACUAN dulu, jadi runtime sempat berisi
      // acuan itu -- nilai operator harus dikirim lagi sesudahnya.
      await initThenEnableWalking(context, expected);
      await applyWalkingWithContext(context, expected);
      context.check();
      sendWalkingCommand("start");
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
    if (!ros?.isConnected) return false;
    const topic = new ROSLIB.Topic({ ros, name, messageType });
    topic.publish(new ROSLIB.Message(data));
    return true;
  };

  const applyWalkingAndStartOnSim = () => {
    if (rosState !== "connected") {
      sendStatus("ROS bridge belum siap", true);
      return;
    }
    let params;
    try {
      params = validateWalkingParams(walkingParams);
    } catch (error) {
      setWalkingError(error.message);
      return;
    }
    clearTimeout(simStartTimerRef.current);
    if (!publishOnSim("/bascorro_studio/sim/walking/set_params", "op3_walking_module_msgs/WalkingParam", params)) return;
    sendStatus("[SIM] Parameter terkirim — enable walking_module…");

    // Skip the action_module → page 2 → walking_module relay (used to "stamp"
    // WALKING_READY). walking_module's IK now produces the same calibrated ready
    // pose statically from init_x/y/z_offset, so we just enable head + walking
    // and start. Mirror change in op3_demo + real-robot path above.
    publishOnSim("/bascorro_studio/sim/enable_ctrl_module", "std_msgs/String", { data: "head_control_module" });
    publishOnSim("/bascorro_studio/sim/enable_ctrl_module", "std_msgs/String", { data: "walking_module" });
    simStartTimerRef.current = window.setTimeout(() => {
      publishOnSim("/bascorro_studio/sim/walking/command", "std_msgs/String", { data: "start" });
      sendStatus("[SIM] Walking dimulai di simulator (robot asli tidak ikut bergerak)");
    }, WALKING_ENABLE_SETTLE_MS);
  };

  const stopWalkingOnSim = () => {
    clearTimeout(simStartTimerRef.current);
    simStartTimerRef.current = null;
    if (rosState !== "connected") {
      sendStatus("ROS bridge belum siap", true);
      return;
    }
    publishOnSim("/bascorro_studio/sim/walking/command", "std_msgs/String", { data: "stop" });
    sendStatus("[SIM] Stop dikirim ke manager_sim");
  };


  const enableWalkingModule = () => runRobotOperation("walking", async (context) => {
    await initThenEnableWalking(context);
    sendStatus("Walking module terkonfirmasi aktif; Start belum dikirim");
  });

  // Tab RL Training: urutan yang sama dengan Apply & Start (page 2 -> acuan
  // dipatok -> modul), dengan stance percobaan sebagai draft. runRobotOperation
  // menolak diam-diam kalau operasi walking lain masih jalan, jadi dikatakan di sini.
  const enableWalkingForRl = (stance) => {
    if (walkingOperationRef.current) {
      sendStatus("RL: operasi walking lain masih berjalan; tunggu selesai", true);
      return Promise.resolve(false);
    }
    return runRobotOperation("walking", (context) => initThenEnableWalking(context, stance));
  };

  const applyWalkingPreset = (preset) => {
    if (walkingOperationRef.current) return;
    const presetParams = {
      tiny: {
        x_move_amplitude: 0.003,
        y_move_amplitude: 0,
        angle_move_amplitude: 0,
        angle_move_amplitude_turn_in: 0,
        period_time: 0.85,
        z_move_amplitude: 0.03,
        dsp_ratio: 0.35,
        balance_enable: true,
      },
      stop: {
        x_move_amplitude: 0,
        y_move_amplitude: 0,
        angle_move_amplitude: 0,
        angle_move_amplitude_turn_in: 0,
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
    setWalkingParams((prev) => ({ ...prev, ...presetParams }));
    setWalkingError("");
  };

  // SAVE: isian -> versi yang sedang TERBUKA (atau versi baru kalau namanya
  // lain) -> disk robot -> jawaban server diperiksa KETAT -> baru "tersimpan".
  // Kalau satu langkah gagal, operator melihat galatnya dan isian tetap
  // bertanda "belum disimpan"; tidak ada lagi sukses palsu.
  const saveWalkingVersion = async () => {
    if (walkingOperationRef.current || walkingSaving) return;
    if (walkingStoreStatus !== "ready") {
      setWalkingError(walkingStoreStatus === "loading"
        ? "Penyimpanan versi masih dibaca; tunggu sebentar lalu Save lagi."
        : `Save dimatikan: penyimpanan versi di robot tidak terbaca (${walkingStoreError}).`);
      return;
    }
    let plan;
    try {
      plan = planWalkingSave({
        versions: walkingVersions,
        activeVersionId: activeWalkingVersionId,
        name: walkingVersionName,
        params: validateWalkingParams(walkingParams),
        now: Date.now(),
        makeId: makeWalkingVersionId,
      });
    } catch (error) {
      setWalkingError(error.message);
      return;
    }
    const { version } = plan;
    console.info(`[SAVE] action = ${version.name}`);
    console.info(`[SAVE] walking parameters = ${JSON.stringify(version.params)}`);
    setWalkingSaving(true);
    try {
      const state = await mutateWalkingStore({ op: "upsert", version });
      const stored = state.versions.find((item) => item.id === version.id);
      if (!stored || !sameWalkingParams(stored.params, version.params)) {
        throw new Error("server mengembalikan nilai yang berbeda dari yang dikirim");
      }
      takeWalkingStoreState(state);
      setActiveWalkingVersionId(stored.id);
      setWalkingVersionName(stored.name);
      setWalkingParams(normalizeWalkingParams(stored.params));
      setWalkingError("");
      console.info(`[SAVE] persisted successfully (revisi ${state.revision}, ${new Date(state.savedAt).toLocaleString()})`);
      sendStatus(`${plan.mode === "new" ? "Versi baru tersimpan" : "Tersimpan"} di disk robot: ${stored.name}`);
    } catch (error) {
      console.error(`[SAVE] GAGAL -- "${version.name}" BELUM tersimpan:`, error);
      setWalkingError(`Save GAGAL, "${version.name}" BELUM tersimpan: ${error.message}`);
      sendStatus(`Save gagal: ${error.message}`, true);
    } finally {
      setWalkingSaving(false);
    }
  };

  // Memuat versi tersimpan: versinya SELALU masuk kotak isian dulu, pembacaan
  // robot menyusul sebagai bonus.
  //
  // Halaman ini punya DUA tombol bernama "Load": yang di baris PARAMETER
  // membaca robot, yang di baris Versi Tersimpan cuma mengisi kotak isian.
  // Operator menekan yang kedua lalu Apply, dan Apply menolak dengan "Load
  // parameter dari robot terlebih dahulu" -- pesan yang menunjuk tombol lain
  // tanpa mengatakannya. Karena itu tombol ini sekalian membaca robot: itu yang
  // memasang angka pembanding yang dibutuhkan Apply.
  //
  // URUTANNYA: isi draft DULU, baca robot belakangan. Versi pertama fungsi ini
  // melakukan sebaliknya, dan itu SALAH -- kalau op3_manager tidak menjawab
  // (mati, atau baru dinyalakan), pembacaannya melempar dan baris pengisi draft
  // TIDAK PERNAH dijalankan, sehingga versi yang dipilih tidak jadi dimuat sama
  // sekali. Operator melihatnya sebagai "gerakan yang tersimpan hilang".
  //
  // Membalik urutannya aman: readWalkingRuntime() hanya menulis
  // walkingCurrentParams (kolom CURRENT) dan TIDAK menyentuh walkingParams,
  // jadi ia tidak bisa membuang draft yang baru saja diisi. Komentar lama di
  // sini yang mengatakan sebaliknya keliru.
  const loadWalkingVersion = () => {
    if (walkingOperationRef.current) return;
    const selected = walkingVersions.find((version) => version.id === activeWalkingVersionId);
    if (!selected) {
      sendStatus("Pilih versi walking terlebih dahulu", true);
      return;
    }
    if (walkingUnsaved && !window.confirm(`Isian sekarang belum disimpan ke "${selected.name}". `
      + "Buang perubahan itu dan muat ulang nilai yang tersimpan?")) return;

    setWalkingParams(normalizeWalkingParams(selected.params));
    setWalkingVersionName(selected.name);
    setWalkingError("");
    console.info(`[LOAD] action = ${selected.name}`);
    console.info(`[LOAD] parameters = ${JSON.stringify(selected.params)}`);

    if (rosState !== "connected") {
      sendStatus(`Versi "${selected.name}" dimuat ke kotak isian. ROS terputus, jadi belum ada `
        + "pembacaan robot: tekan Load di baris PARAMETER sebelum Apply.", true);
      return;
    }
    runRobotOperation("walking", async (context) => {
      await readWalkingRuntime(context);
      sendStatus(`Versi "${selected.name}" dimuat, dan parameter robot ikut dibaca. Tinggal Apply.`);
    });
  };

  const deleteWalkingVersion = async () => {
    const selected = walkingVersions.find((version) => version.id === activeWalkingVersionId);
    if (!selected) {
      sendStatus("Pilih versi walking terlebih dahulu", true);
      return;
    }
    if (walkingStoreStatus !== "ready") {
      setWalkingError("Hapus dimatikan: penyimpanan versi di robot belum terbaca.");
      return;
    }
    if (!window.confirm(`Hapus versi "${selected.name}" dari robot? `
      + "Salinan sebelum dihapus tetap ada di src/bascorro_studio/data/backup/.")) return;
    try {
      const state = await mutateWalkingStore({ op: "delete", id: selected.id });
      takeWalkingStoreState(state);
      setActiveWalkingVersionId("");
      setWalkingVersionName("");
      console.info(`[SAVE] versi dihapus: ${selected.name} (revisi ${state.revision})`);
      sendStatus(`Versi dihapus: ${selected.name}`);
    } catch (error) {
      setWalkingError(`Hapus gagal: ${error.message}`);
      sendStatus(`Hapus gagal: ${error.message}`, true);
    }
  };

  // Memilih versi di daftar = MEMUATNYA ke isian.
  //
  // Dulu pilihan dan isian terpisah: memilih versi hanya mengganti SASARAN,
  // isian tetap berisi apa pun sebelumnya (default sesudah Studio dibuka, atau
  // nilai versi lain), dan Save menulis isian itu ke versi yang dipilih. Jadi
  // "pilih JALAN MENUJU MEKKAH lalu Save" menimpa MEKKAH dengan nilai yang
  // bukan miliknya, dan "pilih lalu Apply & Start" mengirim nilai itu ke robot.
  // Pilihan terakhir juga dicatat di disk robot, supaya dibuka lagi saat
  // Studio dimulai ulang.
  const selectWalkingVersion = (id) => {
    if (id === activeWalkingVersionId) return;
    const target = walkingVersions.find((version) => version.id === id) || null;
    if (target && walkingUnsaved && !window.confirm(
      `Isian sekarang belum disimpan${activeWalkingVersion ? ` ke "${activeWalkingVersion.name}"` : ""}. `
      + `Buang perubahan itu dan buka "${target.name}"?`)) return;
    setActiveWalkingVersionId(target?.id || "");
    setWalkingVersionName(target?.name || "");
    setWalkingError("");
    if (target) {
      setWalkingParams(normalizeWalkingParams(target.params));
      console.info(`[LOAD] action = ${target.name}`);
      console.info(`[LOAD] parameters = ${JSON.stringify(target.params)}`);
    }
    if (walkingStoreStatus === "ready") {
      mutateWalkingStore({ op: "activate", id: target?.id ?? null })
        .catch((error) => sendStatus(`Versi aktif belum tercatat di robot: ${error.message}`, true));
    }
  };

  // Pilihan tampilan (relatif page 2 / nilai asli ROS) ikut disimpan. Tanpa
  // itu, angka yang diketik dalam "Nilai asli ROS" terlihat LAIN sesudah
  // Studio dibuka ulang (tampilan kembali ke relatif), padahal nilainya sama.
  const setWalkingDisplayMode = (relative) => {
    setWalkingRelativeDisplay(relative);
    if (walkingStoreStatus !== "ready") return;
    mutateWalkingStore({ op: "ui", ui: { relativeDisplay: relative } })
      .catch((error) => sendStatus(`Pilihan tampilan belum tersimpan: ${error.message}`, true));
  };

  // Node mana yang benar-benar hidup di robot.
  //
  // Halaman Tendangan butuh demo_node (pemilik parameternya) dan
  // bascorro_studio_agent (yang menulis ke demo.launch.xml); halaman Demo butuh
  // demo_node. Tanpa penanda ini, satu-satunya gejala "demonya belum
  // diluncurkan" adalah tombol yang ditekan tapi robot diam saja -- persis
  // keluhan yang sudah terjadi.
  const refreshRosNodes = useCallback(() => {
    const ros = rosRef.current;
    if (!ros || rosState !== "connected" || typeof ros.getNodes !== "function") {
      setRosNodePresence(null);
      return;
    }
    ros.getNodes(
      (nodes) => {
        const list = Array.isArray(nodes) ? nodes : [];
        setRosNodePresence({
          demo: list.includes("/demo_node"),
          agent: list.includes("/bascorro_studio_agent"),
          // Dua bagian rantai kamera, diperiksa TERPISAH: gejalanya sama
          // (panel hitam) tapi obatnya berbeda sama sekali.
          camera: list.includes("/usb_cam_node/usb_cam"),
          detector: list.includes("/ball_detector_node/yolo_ball_detector"),
        });
        setRosNodeCheckedAt(Date.now());
      },
      () => {
        setRosNodePresence(null);
        setRosNodeCheckedAt(Date.now());
      }
    );
  }, [rosState]);

  // Diperiksa saat masuk halaman yang memerlukannya, bukan terus-menerus:
  // rosapi memindai seluruh graf ROS dan itu tidak gratis.
  useEffect(() => {
    if (activeTab !== "demo" && activeTab !== "tendangan") return;
    refreshRosNodes();
  }, [activeTab, rosState, refreshRosNodes]);

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
    cancelPendingMotion();
    const sequence = motionSequenceRef.current;
    // "Init Pose" button literally plays action page 2 (WALKING_READY — the
    // user's calibrated standing pose recorded in the action editor).
    // Walking-ready is a separate transition triggered by Apply & Start
    // (it switches to walking_module, whose FK-tuned IK pose matches
    // WALKING_READY geometry so the handoff is smooth).
    if (!walkingModulePubRef.current || !actionPagePubRef.current || rosState !== "connected") {
      sendStatus("Init Pose: action module unavailable", true);
      return;
    }
    walkingModuleEnabledRef.current = false;
    // Batas kecepatan dipasang ulang di sini supaya perpindahan ke
    // action_module tidak menyentak. Yang TIDAK bisa diperlambat dari sini
    // adalah laju pemutaran halamannya sendiri: action_module menghitung
    // lintasannya sendiri dan menulis goal tiap 8 ms dengan langkah kecil,
    // jadi lamanya ditentukan halaman di ALPHONSE.bin.
    reapplyTorqueLimit();
    walkingModulePubRef.current.publish(new ROSLIB.Message({ data: "action_module" }));
    sendStatus(`Init Pose: switching to action_module → page ${WALKING_READY_PAGE_NUM}…`);
    window.setTimeout(() => {
      if (sequence !== motionSequenceRef.current || !rosRef.current?.isConnected) return;
      actionPagePubRef.current.publish(new ROSLIB.Message({ data: WALKING_READY_PAGE_NUM }));
      sendStatus(`Init Pose: playing page ${WALKING_READY_PAGE_NUM} (WALKING_READY)`);
    }, ACTION_MODULE_SETTLE_MS);
  };

  const handleSoftStop = () => {
    sendWalkingCommand("stop", "Soft Stop Sent");
  };

  // SATU pembacaan SEGAR posisi sendi.
  //
  // Langganan daftar nama dibatasi 2 detik, dan snapshot seumur itu tidak boleh
  // dipakai di sini: kalau operator baru saja menggeser robot dengan tangan,
  // goal akan diisi pose dua detik lalu dan robot menyentak ke sana -- persis
  // yang sedang dihindari.
  const readPresentJointsOnce = (timeoutMs = 2000) => new Promise((resolve) => {
    const ros = rosRef.current;
    if (!ros?.isConnected) {
      resolve(null);
      return;
    }
    const topic = new ROSLIB.Topic({
      ros, name: "/robotis/present_joint_states",
      messageType: "sensor_msgs/JointState", queue_length: 1,
    });
    let selesai = false;
    const tutup = (nilai) => {
      if (selesai) return;
      selesai = true;
      try { topic.unsubscribe(); } catch { /* sudah lepas */ }
      resolve(nilai);
    };
    topic.subscribe((msg) => {
      if (Array.isArray(msg.name) && msg.name.length === msg.position?.length) {
        tutup({ name: msg.name, position: msg.position });
      }
    });
    window.setTimeout(() => tutup(null), timeoutMs);
  });

  const writeSyncItemsAwaited = (items) => new Promise((resolve) => {
    writeSyncItemsSequential(items, resolve);
  });

  // TORQUE ON = MENAHAN DI TEMPAT, bukan kembali ke pose lama.
  //
  // Selagi torsi mati, goal_position_ berhenti di pose terakhir yang
  // diperintahkan modul sementara robot melorot / digeser tangan. Apa pun yang
  // menutup jarak itu sesudah torsi menyala pasti bermasalah:
  //   - dibiarkan cepat  -> robot menyentak (keluhan awal operator);
  //   - diperlambat      -> servo menahan beban penuh jauh LEBIH LAMA saat kaki
  //                         mentok lantai, dan overload shutdown justru dipicu
  //                         oleh torsi tinggi yang BERKEPANJANGAN -- servo mati.
  //
  // Jadi jaraknya tidak ditutup dengan menggerakkan robot, melainkan dengan
  // memindahkan GOAL-nya ke robot:
  //   1. lepas kepemilikan modul ("none") supaya tidak ada yang menimpa goal;
  //   2. goal := posisi NYATA yang baru dibaca (controller yang mengonversi);
  //   3. pasang batas kecepatan untuk gerakan BERIKUTNYA;
  //   4. baru torsi dinyalakan.
  // Torsi menyala sambil menahan tempatnya sendiri: nol sentakan, nol beban
  // berkepanjangan. Untuk berdiri, operator menekan Walking Ready seperti biasa
  // -- itu memainkan trayektori page 2 yang memang halus.
  // Menunggu sampai SEMUA sendi benar-benar dilepas ke "none". Tanpa ini
  // set_joint_states diabaikan untuk sendi yang masih dipegang modul
  // (robotis_controller.cpp:2082 hanya menerima sendi "none"), dan sendi itu
  // tetap membawa goal lama saat torsi menyala.
  const waitForJointsReleased = async (names, dibatalkan) => {
    const batas = Date.now() + MODULE_RELEASE_TIMEOUT_MS;
    while (Date.now() < batas) {
      if (dibatalkan()) return false;
      try {
        const hasil = await callRosService(walkingModuleGetRef.current, { joint_name: names }, { timeoutMs: 2000 });
        const modul = hasil?.module_name || [];
        if (modul.length === names.length && modul.every((nama) => nama === "none")) return true;
      } catch {
        // Layanan belum menjawab; coba lagi sampai batas waktu.
      }
      await new Promise((r) => window.setTimeout(r, MODULE_RELEASE_POLL_MS));
    }
    return false;
  };

  // TORQUE ON = MENAHAN DI TEMPAT, bukan kembali ke pose lama.
  //
  // Selagi torsi mati, goal_position_ controller berhenti di pose terakhir
  // modul, sementara robot melorot atau digeser tangan. Apa pun yang menutup
  // jarak itu dengan MENGGERAKKAN robot sesudah torsi menyala pasti buruk:
  //   - dibiarkan cepat -> robot menyentak ke "suatu nilai tertentu";
  //   - diperlambat     -> servo menahan beban penuh jauh lebih lama sambil
  //                        mengangkat badan, dan justru torsi tinggi yang
  //                        BERKEPANJANGAN itu yang memicu overload shutdown.
  // Jalur lama juga membawa modul yang masih memegang sendi mengirim pose
  // lamanya tiap 8 ms, jadi servo langsung ditarik ke sana.
  //
  // Jaraknya ditutup dengan memindahkan GOAL ke robot, bukan robot ke goal:
  //   1. batas kecepatan dipasang dulu (RAM, aman saat torsi mati) untuk
  //      gerakan BERIKUTNYA;
  //   2. semua modul dilepas ke "none" dan DITUNGGU sampai terkonfirmasi,
  //      supaya tidak ada yang menimpa goal lagi;
  //   3. goal := posisi NYATA yang baru dibaca (bukan snapshot 2 detik);
  //   4. baru torsi dinyalakan.
  // Robot mengeras di tempatnya sendiri: nol sentakan, nol beban berkepanjangan.
  // Karena dxl_state_->goal_position_ ikut jadi posisi nyata, Init Pose
  // sesudahnya juga mulai dari sana (action_module mengambil titik awalnya dari
  // situ, action_module.cpp:97/274), bukan dari pose lama.
  const handleTorque = async (enable) => {
    cancelPendingMotion();
    const names = walkingJointNames();
    if (!torquePubRef.current || !names) {
      sendStatus("Torque belum bisa dikirim: daftar sendi robot belum terbaca", true);
      return;
    }
    if (!enable) {
      writeSyncItem("torque_enable", 0);
      sendStatus("Torque OFF");
      return;
    }
    if (!walkingModulePubRef.current || !setJointStatesPubRef.current || !walkingModuleGetRef.current) {
      sendStatus("Torque ON dibatalkan: kanal modul/joint state belum siap", true);
      return;
    }
    const preset = TORQUE_SPEED_PRESETS.find(p => p.key === torqueSpeedKey) || TORQUE_SPEED_PRESETS[1];
    const urutan = motionSequenceRef.current;
    const dibatalkan = () => urutan !== motionSequenceRef.current || !rosRef.current?.isConnected;

    sendStatus("Torque ON: memasang batas kecepatan dan menyamakan posisi…");
    const batasTerpasang = await writeSyncItemsAwaited([
      ["profile_velocity", preset.velocity],
      ["profile_acceleration", preset.accel],
    ]);
    if (!batasTerpasang || dibatalkan()) return;

    walkingModuleEnabledRef.current = false;
    walkingModulePubRef.current.publish(new ROSLIB.Message({ data: "none" }));
    if (!await waitForJointsReleased(names, dibatalkan)) {
      if (!dibatalkan()) {
        sendStatus("Torque ON dibatalkan: modul belum melepas semua sendi, torsi TIDAK dinyalakan supaya robot tidak menyentak", true);
      }
      return;
    }

    const posisi = await readPresentJointsOnce();
    if (dibatalkan()) return;
    if (!posisi) {
      sendStatus("Torque ON dibatalkan: posisi sendi tidak terbaca, torsi TIDAK dinyalakan supaya robot tidak menyentak", true);
      return;
    }
    setJointStatesPubRef.current.publish(new ROSLIB.Message({
      name: posisi.name, position: posisi.position, velocity: [], effort: [],
    }));
    await new Promise((r) => window.setTimeout(r, GOAL_SYNC_SETTLE_MS));
    if (dibatalkan()) return;

    writeSyncItem("torque_enable", 1);
    setTorqueLimitActive(true);
    sendStatus(`Torque ON — robot menahan posisinya sendiri, tanpa menyentak. `
      + `Batas ${preset.label} (~${preset.approxDegPerSec} deg/s) terpasang untuk gerakan berikutnya. `
      + `Tekan Init Pose untuk berdiri.`);
  };

  // Dipasang ULANG tepat sebelum gerakan yang dimulai dari Studio, bukan
  // sekadar saat tombol torsi ditekan: itulah gerakan yang benar-benar dilihat
  // operator. Kembali false kalau tidak ada batas yang sedang aktif.
  const reapplyTorqueLimit = (onDone) => {
    if (!torqueLimitActive) return false;
    const preset = TORQUE_SPEED_PRESETS.find(p => p.key === torqueSpeedKey) || TORQUE_SPEED_PRESETS[1];
    writeSyncItemsSequential(
      [
        ["profile_velocity", preset.velocity],
        ["profile_acceleration", preset.accel],
      ],
      onDone
    );
    return true;
  };

  const handleEnableHeadModule = () => {
    cancelPendingMotion();
    if (rosRef.current?.isConnected && headModulePubRef.current) {
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
    cancelPendingMotion();
    if (!demoModePubRef.current || rosState !== "connected") {
      sendStatus("Demo mode unavailable", true);
      return;
    }
    // Memilih mode SUDAH memulai demonya (lihat DEMO_MODES), dan demo berjalan
    // dengan gait -- batas kecepatan servo yang masih terpasang akan
    // melumpuhkannya. Dilepas dulu, kecuali saat keluar ke posisi siap.
    // Tanpa syarat, alasannya sama seperti di initThenEnableWalking.
    if (mode !== "ready") {
      restoreFullSpeed("Batas kecepatan servo dinolkan — demo butuh kecepatan penuh");
    }
    demoModePubRef.current.publish(new ROSLIB.Message({ data: mode }));
    pushDemoStatus(`mode ${mode}`, mode);
    sendStatus(`Demo mode: ${mode}`);
  };

  const handleDemoCommand = (command) => {
    cancelPendingMotion();
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

  const updateKickParam = (key, value) => {
    setKickParams((previous) => ({ ...previous, [key]: value }));
    setKickSaveArmed(false);
    setKickSaveMessage("");
    setKickError("");
  };

  const readKickRuntime = async (context) => {
    const result = await context.call(kickGetServiceRef.current, { names: KICK_PARAM_KEYS });
    const decoded = decodeKickParameters(result);
    setKickCurrentParams(decoded.params);
    setKickLoaded(true);
    return decoded;
  };

  const loadKickParams = () => runRobotOperation("kick", async (context) => {
    setKickSaveArmed(false);
    setKickSaveMessage("");
    const { params } = await readKickRuntime(context);
    setKickParams(params);
    sendStatus("Parameter tendangan dimuat lengkap dari robot");
  });

  const applyKickParams = () => runRobotOperation("kick", async (context) => {
    setKickSaveArmed(false);
    setKickSaveMessage("");
    if (!kickLoaded || !kickCurrentParams) throw new Error("Muat parameter dari robot terlebih dahulu sebelum mengirim");
    const draft = validateKickParams(kickParams);
    const { params: current, types } = await readKickRuntime(context);
    const payload = { ...current };
    for (const key of KICK_PARAM_KEYS) {
      if (!parameterValuesEqual(draft[key], kickCurrentParams[key])) payload[key] = draft[key];
    }
    validateKickParams(payload);
    const parameters = makeKickParameters(payload, types);
    if (!parametersEqual(payload, current, KICK_PARAM_KEYS)) {
      const response = await context.call(kickSetServiceRef.current, { parameters });
      if (response?.result?.successful !== true) throw new Error(response?.result?.reason || "Perubahan parameter ditolak secara atomik");
      await context.wait(300);
    }
    const { params: observed } = await readKickRuntime(context);
    if (!parametersEqual(payload, observed, KICK_PARAM_KEYS)) throw new Error("Hasil kirim belum cocok dengan pembacaan robot; periksa perubahan dari klien lain");
    setKickParams(observed);
    setKickLastAppliedAt(Date.now());
    sendStatus("Parameter tendangan diterima atomik dan terbaca balik dari robot");
  });

  const saveKickParamsToLaunch = () => runRobotOperation("kick", async (context) => {
    setKickSaveArmed(false);
    setKickSaveMessage("");
    if (!kickLoaded || kickDirty) throw new Error("Muat dan kirim perubahan sebelum menyimpan ke launch");
    const { params } = await readKickRuntime(context);
    if (!parametersEqual(params, kickParams, KICK_PARAM_KEYS)) throw new Error("Runtime berubah sejak pembacaan terakhir. Periksa nilai robot sebelum menyimpan");
    const response = await context.call(kickSaveServiceRef.current, {}, { timeoutMs: 12000 });
    if (response?.success !== true) throw new Error(response?.message || "Penyimpanan launch belum terkonfirmasi");
    setKickSavedAt(Date.now());
    setKickSaveMessage(response.message);
    sendStatus("Setelan runtime tendangan tersimpan ke demo.launch.xml");
  });

  const resetKickParamsToLaunch = () => runRobotOperation("kick", async (context) => {
    setKickSaveArmed(false);
    setKickSaveMessage("");
    const response = await context.call(kickLaunchServiceRef.current);
    if (response?.success !== true) throw new Error(response?.message || "Bawaan launch tidak terbaca; restart studio_agent jika service belum tersedia");
    const result = JSON.parse(response.message);
    setKickParams(validateKickParams(result.parameters));
    sendStatus("Draft dimuat dari berkas " + result.path + "; belum dikirim ke robot");
  });

  const revertKickEdits = () => {
    if (!kickCurrentParams) return;
    setKickParams({ ...kickCurrentParams });
    setKickSaveArmed(false);
    setKickSaveMessage("");
    setKickError("");
    sendStatus("Draft kembali ke hasil pembacaan robot terakhir");
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
      <nav aria-label="Navigasi utama" className={`
        studio-sidebar fixed inset-y-0 left-0 z-40 w-64 bg-sidebar flex flex-col p-6 text-gray-400 transition-transform duration-300 lg:translate-x-0 lg:static lg:flex-shrink-0
        ${sidebarOpen ? "translate-x-0" : "-translate-x-full"}
      `}>
        <div className="studio-brand flex items-center justify-between mb-8 px-2">
          <div className="studio-brand-identity">
            <img src="/bascorro-logo.png" alt="EWS BASCORRO" width="211" height="75" className="studio-brand-logo" />
            <span className="studio-brand-caption">ROBOTICS STUDIO</span>
          </div>
          <button aria-label="Tutup navigasi" onClick={() => setSidebarOpen(false)} className="lg:hidden text-gray-400 hover:text-white p-1">
            <X size={20} />
          </button>
        </div>
        <div className="studio-nav-links flex flex-col gap-2 flex-1">
          <div className="studio-nav-label">Monitor <span>01</span></div>
          <button
            className={`flex items-center gap-3 px-4 py-3 rounded-lg font-medium transition-all ${activeTab === "dashboard" ? "bg-undip-blue text-white shadow-md border border-accent-yellow/20" : "hover:bg-white/5 hover:text-white"}`}
            onClick={() => { setActiveTab("dashboard"); setSidebarOpen(false); }}
            aria-current={activeTab === "dashboard" ? "page" : undefined}
          >
            <LayoutDashboard size={20} className={activeTab === "dashboard" ? "text-accent-yellow" : ""} />
            <span>Dashboard</span>
          </button>
          <button
            className={`flex items-center gap-3 px-4 py-3 rounded-lg font-medium transition-all ${activeTab === "charts" ? "bg-undip-blue text-white shadow-md border border-accent-yellow/20" : "hover:bg-white/5 hover:text-white"}`}
            onClick={() => { setActiveTab("charts"); setSidebarOpen(false); }}
            aria-current={activeTab === "charts" ? "page" : undefined}
          >
            <Activity size={20} className={activeTab === "charts" ? "text-accent-yellow" : ""} />
            <span>Charts</span>
          </button>
          <button
            className={`flex items-center gap-3 px-4 py-3 rounded-lg font-medium transition-all ${activeTab === "vision" ? "bg-undip-blue text-white shadow-md border border-accent-yellow/20" : "hover:bg-white/5 hover:text-white"}`}
            onClick={() => { setActiveTab("vision"); setSidebarOpen(false); }}
            aria-current={activeTab === "vision" ? "page" : undefined}
          >
            <Video size={20} className={activeTab === "vision" ? "text-accent-yellow" : ""} />
            <span>Vision</span>
          </button>
          <div className="studio-nav-label">Kontrol robot <span>02</span></div>
          <button
            className={`flex items-center gap-3 px-4 py-3 rounded-lg font-medium transition-all ${activeTab === "tuning" ? "bg-undip-blue text-white shadow-md border border-accent-yellow/20" : "hover:bg-white/5 hover:text-white"}`}
            onClick={() => { setActiveTab("tuning"); setSidebarOpen(false); }}
            aria-current={activeTab === "tuning" ? "page" : undefined}
          >
            <Settings size={20} className={activeTab === "tuning" ? "text-accent-yellow" : ""} />
            <span>Tuning</span>
          </button>
          <button
            className={`flex items-center gap-3 px-4 py-3 rounded-lg font-medium transition-all ${activeTab === "walking" ? "bg-undip-blue text-white shadow-md border border-accent-yellow/20" : "hover:bg-white/5 hover:text-white"}`}
            onClick={() => { setActiveTab("walking"); setSidebarOpen(false); }}
            aria-current={activeTab === "walking" ? "page" : undefined}
          >
            <Footprints size={20} className={activeTab === "walking" ? "text-accent-yellow" : ""} />
            <span>Walking</span>
          </button>
          <button
            className={`flex items-center gap-3 px-4 py-3 rounded-lg font-medium transition-all ${activeTab === "rl" ? "bg-undip-blue text-white shadow-md border border-accent-yellow/20" : "hover:bg-white/5 hover:text-white"}`}
            onClick={() => { setActiveTab("rl"); setSidebarOpen(false); }}
            aria-current={activeTab === "rl" ? "page" : undefined}
          >
            <Brain size={20} className={activeTab === "rl" ? "text-accent-yellow" : ""} />
            <span>RL Training</span>
          </button>
          <button
            className={`flex items-center gap-3 px-4 py-3 rounded-lg font-medium transition-all ${activeTab === "tendangan" ? "bg-undip-blue text-white shadow-md border border-accent-yellow/20" : "hover:bg-white/5 hover:text-white"}`}
            onClick={() => { setActiveTab("tendangan"); setSidebarOpen(false); }}
            aria-current={activeTab === "tendangan" ? "page" : undefined}
          >
            <Crosshair size={20} className={activeTab === "tendangan" ? "text-accent-yellow" : ""} />
            <span>Tendangan</span>
          </button>
          <button
            className={`flex items-center gap-3 px-4 py-3 rounded-lg font-medium transition-all ${activeTab === "demo" ? "bg-undip-blue text-white shadow-md border border-accent-yellow/20" : "hover:bg-white/5 hover:text-white"}`}
            onClick={() => { setActiveTab("demo"); setSidebarOpen(false); }}
            aria-current={activeTab === "demo" ? "page" : undefined}
          >
            <PlayCircle size={20} className={activeTab === "demo" ? "text-accent-yellow" : ""} />
            <span>Demo</span>
          </button>
          <button
            className={`flex items-center gap-3 px-4 py-3 rounded-lg font-medium transition-all ${activeTab === "teleop" ? "bg-undip-blue text-white shadow-md border border-accent-yellow/20" : "hover:bg-white/5 hover:text-white"}`}
            onClick={() => { setActiveTab("teleop"); setSidebarOpen(false); }}
            aria-current={activeTab === "teleop" ? "page" : undefined}
          >
            <Gamepad2 size={20} className={activeTab === "teleop" ? "text-accent-yellow" : ""} />
            <span>Teleop</span>
          </button>
          <button
            className={`flex items-center gap-3 px-4 py-3 rounded-lg font-medium transition-all ${activeTab === "action" ? "bg-undip-blue text-white shadow-md border border-accent-yellow/20" : "hover:bg-white/5 hover:text-white"}`}
            onClick={() => { setActiveTab("action"); setSidebarOpen(false); }}
            aria-current={activeTab === "action" ? "page" : undefined}
          >
            <BicepsFlexed size={20} className={activeTab === "action" ? "text-accent-yellow" : ""} />
            <span>Action</span>
          </button>
          <div className="studio-nav-label">Sistem <span>03</span></div>
          <button
            className={`flex items-center gap-3 px-4 py-3 rounded-lg font-medium transition-all ${activeTab === "logs" ? "bg-undip-blue text-white shadow-md border border-accent-yellow/20" : "hover:bg-white/5 hover:text-white"}`}
            onClick={() => { setActiveTab("logs"); setSidebarOpen(false); }}
            aria-current={activeTab === "logs" ? "page" : undefined}
          >
            <ScrollText size={20} className={activeTab === "logs" ? "text-accent-yellow" : ""} />
            <span>Logs</span>
          </button>
          <button
            className={`flex items-center gap-3 px-4 py-3 rounded-lg font-medium transition-all ${activeTab === "terminal" ? "bg-undip-blue text-white shadow-md border border-accent-yellow/20" : "hover:bg-white/5 hover:text-white"}`}
            onClick={() => { setActiveTab("terminal"); setSidebarOpen(false); }}
            aria-current={activeTab === "terminal" ? "page" : undefined}
          >
            <Terminal size={20} className={activeTab === "terminal" ? "text-accent-yellow" : ""} />
            <span>Terminal</span>
          </button>
        </div>
        <div className="studio-connection mt-3 p-3 rounded-xl border border-white/10 bg-white/[0.03]">
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
        <div className="studio-sidebar-status pt-6 border-t border-white/10">
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
    const displayWalkingValue = (value, field) => formatWalkingValue(
      walkingOffsetToDisplay(value, walkingReferenceOffset(field.key, walkingRelativeDisplay)), field,
    );
    const currentValue = (field) => {
      if (!walkingCurrentParams) return "not loaded";
      return displayWalkingValue(walkingCurrentParams[field.key], field);
    };
    const isFieldDirty = (field) => {
      if (!walkingCurrentParams) return false;
      if (field.type === "bool") return walkingParams[field.key] !== walkingCurrentParams[field.key];
      // NaN !== NaN, so comparing with Number() marked a field as changed forever the
      // moment it held anything Number() could not read (a half-typed value, a comma).
      return !parameterValuesEqual(walkingParams[field.key], walkingCurrentParams[field.key]);
    };
    const selectedWalkingVersion = walkingVersions.find((version) => version.id === activeWalkingVersionId);
    const renderWalkingHelp = (field) => (
      <div className="absolute left-4 right-4 top-[calc(100%-4px)] z-30 hidden rounded-xl border border-undip-blue/20 bg-white p-4 text-xs shadow-xl group-hover/param:block group-focus-within/param:block lg:left-5 lg:right-auto lg:w-[380px]">
        <div className="flex items-start justify-between gap-3">
          <div>
            <div className="font-bold text-gray-900">{field.label}</div>
            <div className="mt-1 font-mono text-[11px] text-gray-400">
              {field.key}{field.unit ? ` | ${field.unit}` : ""} | current {currentValue(field)} | edited {displayWalkingValue(walkingParams[field.key], field)}
            </div>
          </div>
          <Info size={16} className="mt-0.5 flex-shrink-0 text-undip-blue" />
        </div>
        <div className="mt-3 space-y-2 leading-relaxed text-gray-600">
          {walkingReferenceOffset(field.key, walkingRelativeDisplay) !== null && (
            <p className="rounded-lg bg-blue-50 p-2 text-undip-blue">
              <strong>Δ page 2:</strong> angka tampilan = nilai asli ROS − {WALKING_POSE_REFERENCE.offsets[field.key]} {field.unit}.
              Nilai asli draft: <span className="font-mono">{formatWalkingValue(walkingParams[field.key], field)}</span>;
              nilai asli robot: <span className="font-mono">{walkingCurrentParams ? formatWalkingValue(walkingCurrentParams[field.key], field) : "belum dibaca"}</span>.
              Nol di sini bukan perintah memainkan page 2 atau pembacaan posisi sendi/IMU.
            </p>
          )}
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
          <div className="studio-control-panel bg-white rounded-2xl border border-gray-200 shadow-sm overflow-hidden">
            {/* Title strip */}
            <div className="px-5 pt-5 pb-3 border-b border-gray-100">
              <div className="flex flex-wrap items-center gap-2">
                <h2 className="studio-control-title text-lg font-bold font-display text-gray-900"><Footprints size={20} aria-hidden="true" /> Walking</h2>
                <span className={`px-2 py-0.5 rounded-full text-[10px] font-bold uppercase ${walkingDirty ? "bg-yellow-100 text-yellow-700" : "bg-green-50 text-green-700"}`} title={walkingDirty ? "Ada perubahan yang belum di-Apply" : "Cocok dengan pembacaan terakhir, bukan pemantauan runtime terus-menerus"}>
                  {!walkingLoaded ? "Belum dibaca" : walkingDirty ? "Belum di-Apply" : "Cocok saat dibaca"}
                </span>
                  <span className={`px-2 py-0.5 rounded-full text-[10px] font-bold uppercase ${walkingLoaded ? "bg-blue-50 text-undip-blue" : "bg-gray-100 text-gray-500"}`} title={walkingLoaded ? "Nilai diambil dari robot via Load" : "Belum pernah Load — masih nilai referensi UI"}>
                  {walkingLoaded ? "Runtime" : "Default"}
                </span>
                {walkingLastAppliedAt && (
                  <span className="text-[10px] font-mono text-gray-400" title="Waktu terakhir Apply dipanggil">
                    Apply: {new Date(walkingLastAppliedAt).toLocaleTimeString()}
                  </span>
                )}
              </div>
              <p className="mt-1 text-xs text-gray-500">
                Workflow: <b>Load</b> → ubah draft → <b>Apply</b> → <b>Start</b>. Start memeriksa modul terlebih dahulu. Satuan: detik, meter, radian.
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
                disabled={!rosConnected || walkingBusy}
                title="Aktifkan walking_module; dapat mengubah postur meskipun belum Start"
              >
                <Cpu size={14} /> Enable Module
              </button>
              <button
                className={`${walkingButtonBase} bg-gray-100 text-gray-700 hover:bg-gray-200`}
                onClick={loadWalkingParams}
                disabled={!rosConnected || walkingBusy}
                title="Tarik parameter walking yang sedang aktif dari robot"
              >
                <RefreshCw size={14} /> Load
              </button>
              <div className="h-5 w-px bg-gray-300 mx-1" />
              <button
                className={`${walkingButtonBase} bg-undip-blue text-white hover:bg-opacity-90`}
                onClick={applyWalkingParams}
                disabled={!rosConnected || walkingBusy}
                title="Kirim perubahan draft dan baca balik hasilnya. Dapat mengubah postur/gait jika modul aktif"
              >
                <Send size={14} /> Apply
              </button>
              <button
                className={`${walkingButtonBase} bg-green-600 text-white hover:bg-green-700`}
                onClick={applyWalkingAndStart}
                disabled={!rosConnected || walkingBusy}
                title="Apply parameter + langsung jalan (gabungan Apply + Start)"
              >
                <Play size={14} /> Apply &amp; Start
              </button>
              {/* Inline toggle, like ORION_NEW. The previous version was a <details>
                  whose panel was position:absolute with no positioned ancestor in this
                  flex row, so the popup landed wherever the nearest positioned block
                  happened to be. An extra button in the same row cannot mis-place. */}
              {showWalkingAdvanced && (
                <button
                  className={`${walkingButtonBase} bg-white text-gray-700 border border-gray-200 hover:bg-gray-100`}
                  onClick={applyWalkingAndRefreshTeleop}
                  disabled={!rosConnected || walkingBusy}
                  title="Apply + minta teleop publish baseline parameter baru"
                >
                  <Gamepad2 size={14} /> + Teleop
                </button>
              )}
              <button
                onClick={() => setShowWalkingAdvanced((v) => !v)}
                className="ml-auto shrink-0 text-xs font-bold text-undip-blue hover:underline"
              >
                {showWalkingAdvanced ? "− Advanced" : "+ Advanced"}
              </button>
            </div>

            {/* Control group — runtime commands */}
            <div className="px-5 py-3 flex flex-wrap items-center gap-2">
              <span className="text-[10px] font-bold text-gray-500 uppercase tracking-wider mr-1">Kontrol</span>
              <button className={`${walkingButtonBase} bg-green-50 text-green-700 border border-green-100 hover:bg-green-100`} onClick={() => startWalkingNow()} disabled={!rosConnected || walkingBusy} title="Mulai jalan dengan parameter yang sudah di-Apply">
                <Play size={14} /> Start
              </button>
              <button className={`${walkingButtonBase} bg-red-50 text-red-600 border border-red-100 hover:bg-red-100`} onClick={() => sendWalkingCommand("stop")} disabled={!rosConnected} title="Berhenti jalan (parameter tetap aktif)">
                <Square size={14} /> Stop
              </button>
              <div className="h-5 w-px bg-gray-300 mx-1" />
              <button className={`${walkingButtonBase} bg-gray-50 text-gray-700 border border-gray-200 hover:bg-gray-100`} onClick={() => sendBalanceCommand(true)} disabled={!rosConnected || walkingBusy} title="Aktifkan balance IMU feedback (checkbox balance_enable ikut menyala)">
                Balance On
              </button>
              <button className={`${walkingButtonBase} bg-gray-50 text-gray-700 border border-gray-200 hover:bg-gray-100`} onClick={() => sendBalanceCommand(false)} disabled={!rosConnected || walkingBusy} title="Matikan balance IMU feedback (checkbox balance_enable ikut padam, jadi Apply berikutnya tidak menyalakannya lagi)">
                Balance Off
              </button>
              <div className="h-5 w-px bg-gray-300 mx-1" />
              <button className={`${walkingButtonBase} bg-yellow-50 text-yellow-700 border border-yellow-100 hover:bg-yellow-100`} onClick={saveWalkingRuntime} disabled={!rosConnected || walkingBusy || !walkingLoaded || walkingDirty} title="Simpan parameter runtime ke config robot (persistent)">
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
              <span
                className={`rounded-full px-2 py-0.5 text-[10px] font-semibold ${walkingStoreStatus === "ready" ? "bg-green-50 text-green-700" : walkingStoreStatus === "loading" ? "bg-gray-100 text-gray-500" : "bg-red-50 text-red-700"}`}
                title="src/bascorro_studio/data/walking_versions.json di PC robot — sama untuk semua browser dan alamat (localhost maupun IP). Salinan sebelum tiap Save: data/backup/."
              >
                {walkingStoreStatus === "ready" ? "Tersimpan di disk robot" : walkingStoreStatus === "loading" ? "Membaca penyimpanan…" : "Penyimpanan robot TIDAK terbaca"}
              </span>
              {walkingStoreStatus === "ready" && (
                <span
                  data-testid="walking-save-state"
                  className={`rounded-full px-2 py-0.5 text-[10px] font-bold ${walkingUnsaved ? "bg-yellow-100 text-yellow-800" : "bg-blue-50 text-undip-blue"}`}
                >
                  {activeWalkingVersion
                    ? (walkingUnsaved ? `● Belum disimpan ke "${activeWalkingVersion.name}"` : `✓ Sama dengan "${activeWalkingVersion.name}" yang tersimpan`)
                    : (walkingUnsaved ? "● Versi baru — belum disimpan" : "Belum ada versi yang dibuka (nilai default)")}
                </span>
              )}
              <span className="text-[10px] text-gray-400">Memilih versi langsung memuatnya ke isian. Save menulis ke versi yang terbuka; nama lain = versi baru. Tetap perlu Apply untuk kirim ke robot.</span>
            </div>
            {walkingStoreStatus === "offline" && (
              <div className="mb-3 rounded-lg border border-red-200 bg-red-50 p-3 text-xs text-red-700">
                <b>Penyimpanan versi di robot tidak terbaca:</b> {walkingStoreError}. Daftar di bawah hanya salinan lama dari browser ini
                (baca saja); Save dan Delete dimatikan supaya tidak ada simpanan yang hilang. Jalankan Studio lewat
                <code className="mx-1">scripts/bascorro_studio.sh</code> lalu muat ulang halaman.
              </div>
            )}
            <div className="grid grid-cols-1 sm:grid-cols-[1fr_1fr_auto_auto_auto] gap-2">
              <input
                type="text"
                aria-label="Nama versi"
                value={walkingVersionName}
                onChange={(e) => setWalkingVersionName(e.target.value)}
                className="min-w-0 rounded-lg border border-gray-200 bg-gray-50 px-3 py-2 text-sm text-gray-900 focus:border-undip-blue focus:outline-none"
                placeholder="Nama versi…"
              />
              <select
                aria-label="Versi tersimpan"
                value={activeWalkingVersionId}
                onChange={(e) => selectWalkingVersion(e.target.value)}
                disabled={walkingStoreStatus === "loading"}
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
                disabled={walkingStoreStatus !== "ready" || walkingSaving}
                aria-label="Save versi"
                title="Simpan isian ke versi yang sedang terbuka (nama lain = versi baru), langsung ke disk robot"
              >
                <Save size={14} /> {walkingSaving ? "Menyimpan…" : "Save"}
              </button>
              <button
                className={`${walkingButtonBase} bg-gray-100 text-gray-700 hover:bg-gray-200`}
                onClick={loadWalkingVersion}
                disabled={!selectedWalkingVersion || walkingBusy}
                title="Muat ulang versi terpilih dari penyimpanan (membuang perubahan yang belum disimpan), lalu baca robot"
              >
                <RefreshCw size={14} /> Muat versi
              </button>
              <button
                className={`${walkingButtonBase} bg-red-50 text-red-600 border border-red-100 hover:bg-red-100`}
                onClick={deleteWalkingVersion}
                disabled={!selectedWalkingVersion || walkingStoreStatus !== "ready"}
                title="Hapus versi terpilih dari disk robot (salinan sebelum dihapus tetap ada di data/backup/)"
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

          <section className="bg-white rounded-2xl border border-gray-200 shadow-sm p-5" aria-label="Acuan angka walking">
            <div className="flex flex-wrap items-center justify-between gap-3">
              <div>
                <h3 className="font-bold text-gray-900">Acuan angka: {WALKING_POSE_REFERENCE.label}</h3>
                <p className="mt-1 text-xs text-gray-500">Hanya cara membaca dan mengisi offset. Tidak mengubah posisi robot atau isi preset.</p>
              </div>
              <div className="flex flex-wrap gap-2" role="group" aria-label="Tampilan nilai walking">
                <button type="button" aria-pressed={walkingRelativeDisplay} onClick={() => setWalkingDisplayMode(true)}
                  className={`${walkingButtonBase} ${walkingRelativeDisplay ? "bg-undip-blue text-white" : "bg-gray-100 text-gray-700"}`}>
                  Relatif page 2
                </button>
                <button type="button" aria-pressed={!walkingRelativeDisplay} onClick={() => setWalkingDisplayMode(false)}
                  className={`${walkingButtonBase} ${!walkingRelativeDisplay ? "bg-undip-blue text-white" : "bg-gray-100 text-gray-700"}`}>
                  Nilai asli ROS
                </button>
              </div>
            </div>
            <div className="mt-3 space-y-2 text-xs leading-relaxed text-gray-600">
              <p>Init Pose dan Hip pitch ditampilkan sebagai selisih dari acuan tetap page 2. Kolom Current memakai acuan yang sama.
                Y tetap tambahan lebar kaki: nilai 0 mengikuti lebar acuan, bukan mengurangi 0.028 m yang tersimpan.
                Timing, Movement, Balance, dan gain tetap memakai nilai aslinya.</p>
              <p>Load membaca nilai asli robot; Apply dan Save tetap memakai format asli ROS. Memilih tampilan tidak mengirim perintah.
                Acuan ini bukan kalibrasi langsung: modul walking masih menangkap pose saat diaktifkan, sehingga nol di layar bukan jaminan pose fisik sama dengan page 2.</p>
              <p className="font-semibold text-undip-blue">Untuk menyalin angka ke YAML, skrip, atau terminal ROS, pilih Nilai asli ROS terlebih dahulu.</p>
            </div>
            <details className="mt-3 text-xs text-gray-600">
              <summary className="font-semibold text-undip-blue">Lihat nilai nol dalam format ROS</summary>
              <p className="mt-2 text-gray-500">{WALKING_POSE_REFERENCE.source}. Acuan tidak ikut bergeser saat Load, Apply, atau ganti preset.</p>
              <div className="mt-2 grid grid-cols-1 sm:grid-cols-2 gap-2 font-mono">
                {Object.entries(WALKING_POSE_REFERENCE.offsets).map(([key, value]) => (
                  <div key={key} className="flex flex-wrap justify-between gap-2 rounded-lg bg-gray-50 px-3 py-2">
                    <span>{key}</span><span>{value} {key.includes("_x_") || key.includes("_y_") || key.includes("_z_") ? "m" : "rad"}</span>
                  </div>
                ))}
              </div>
            </details>
          </section>

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
                    const referenceOffset = walkingReferenceOffset(field.key, walkingRelativeDisplay);
                    return (
                      <div key={field.key} className="group/param relative grid grid-cols-1 gap-3 p-4 md:grid-cols-[minmax(170px,1fr)_minmax(180px,220px)_minmax(150px,180px)] md:items-center">
                        <div className="min-w-0">
                          <div className="flex items-center gap-2">
                            <label className="text-sm font-bold text-gray-800">{field.label}</label>
                            <Info size={13} className="text-gray-300 transition-colors group-hover/param:text-undip-blue group-focus-within/param:text-undip-blue" />
                            {dirty && <span className="w-2 h-2 rounded-full bg-yellow-400" title="Changed" />}
                          </div>
                          <div className="mt-1 text-[11px] text-gray-400 font-mono break-all">{field.key}{field.unit ? ` (${field.unit})` : ""}</div>
                          {referenceOffset !== null && <div className="mt-1 text-[10px] font-semibold text-undip-blue">Δ page 2 · ROS: {formatWalkingValue(walkingParams[field.key], field)}</div>}
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
                            disabled={walkingBusy}
                            aria-label={field.key}
                          >
                            {walkingParams[field.key] ? "Enabled" : "Disabled"}
                          </button>
                        ) : (
                          // type="text": a number input hands back "" for anything the
                          // browser considers a partial value (mid-typing "0.", a
                          // comma, a paste), and that empty string used to be applied
                          // to the robot as 0. inputMode keeps the numeric keypad on
                          // touch; normalizeWalkingParams does the parsing.
                          <WalkingOffsetInput
                            value={walkingParams[field.key]}
                            referenceOffset={referenceOffset}
                            onChange={(value) => updateWalkingParam(field.key, value)}
                            disabled={walkingBusy}
                            aria-label={field.key}
                            className="w-full px-3 py-2 rounded-lg border border-gray-200 bg-gray-50 text-sm font-mono text-gray-900 focus:outline-none focus:ring-2 focus:ring-undip-blue/20 focus:border-undip-blue"
                          />
                        )}

                        <div className="rounded-lg bg-gray-50 border border-gray-100 px-3 py-2">
                          <div className="text-[10px] font-bold uppercase text-gray-400">{referenceOffset !== null ? "Current Δ page 2" : "Current"}</div>
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

  const renderKick = () => {
    const rosConnected = rosState === "connected";
    const kickButtonBase = "inline-flex items-center justify-center gap-2 px-3 py-2 rounded-lg text-xs font-bold transition-colors disabled:opacity-50 disabled:cursor-not-allowed";

    const fmt = (value, digits = 2) =>
      Number.isFinite(value) ? Number(value.toFixed(digits)).toString() : "--";

    const showValue = (raw, field) => {
      const value = parseWalkingNumber(raw, NaN);
      if (Number.isNaN(value)) return String(raw ?? "");
      if (field?.type === "switch") return value > 0.5 ? "aktif" : "mati";
      return fmt(value, 3);
    };
    const currentValue = (field) => {
      if (!kickCurrentParams) return "belum dimuat";
      return showValue(kickCurrentParams[field.key], field);
    };
    const isFieldDirty = (field) => {
      if (!kickCurrentParams) return kickDirty;
      return (
        !parameterValuesEqual(kickParams[field.key], kickCurrentParams[field.key])
      );
    };

    // cameraTick sengaja dibaca supaya perhitungan ini ikut disegarkan tiap
    // detik walaupun tidak ada frame yang datang.
    void cameraTick;
    const lastFrameMs = overlayLastRef.current.stampMs;
    const frameAgeSec = lastFrameMs === null ? null : (performance.now() - lastFrameMs) / 1000;
    const cameraSilent = kickCameraOn && (frameAgeSec === null || frameAgeSec > 3);

    // Rantai kameranya dua bagian, dan kalau salah satunya hilang gejalanya
    // SAMA PERSIS: panel hitam. Menyebut bagian mana yang hilang menghemat satu
    // putaran tanya-jawab -- sudah terjadi dua kali.
    let cameraDiagnosis =
      "Coba tombol Scan di halaman Vision untuk melihat topik gambar yang benar-benar ada.";
    if (rosNodePresence) {
      if (!rosNodePresence.detector && !rosNodePresence.camera) {
        cameraDiagnosis =
          "Kamera dan detektor dua-duanya tidak jalan. Pilih YOLO Vision di peluncur — ia menjalankan " +
          "yolo_ball_detector_openvino.launch.py, berkas yang sama dengan yang dipakai demo, lengkap dengan usb_cam-nya.";
      } else if (!rosNodePresence.camera) {
        cameraDiagnosis =
          "Detektornya JALAN, tapi usb_cam tidak ada — jadi detektor itu tidak punya gambar untuk dilihat. " +
          "Jalankan ulang tanpa use_usb_cam:=false.";
      } else if (!rosNodePresence.detector) {
        cameraDiagnosis =
          "usb_cam JALAN, tapi detektornya tidak ada. Periksa log pane YOLO Vision — kalau node-nya mati saat " +
          "start, alasannya tercetak di situ.";
      } else {
        cameraDiagnosis =
          "Kamera dan detektor dua-duanya jalan, tapi tidak ada gambar yang sampai ke sini. Periksa nama topiknya " +
          "dengan tombol Scan di halaman Vision.";
      }
    }

    // PENGUKURAN LANGSUNG. Dihitung ulang tiap render supaya ia memakai
    // parameter yang SEDANG DIKETIK, bukan yang tersimpan di robot.
    const measureParams = { ...MEASURE_WINDOW_DEFAULTS, ...kickParams };
    const obsAgeSec = kickObservation ? (Date.now() - kickObservation.at) / 1000 : null;
    const obsFresh = obsAgeSec !== null && obsAgeSec < 2;
    const measurement =
      obsFresh && kickObservation.circle && Number.isFinite(kickHipPitch)
        ? measureBall(kickObservation.circle, kickObservation.head, kickHipPitch, measureParams)
        : null;
    const draftKickDistance = parseWalkingNumber(kickParams.kick_distance, NaN);
    const draftPanMax = parseWalkingNumber(kickParams.kick_pan_max_distance, NaN);
    // Kenapa belum ada angka. Diurut dari yang paling sering jadi sebabnya.
    const measureBlocker = !rosConnected
      ? "ROS bridge belum tersambung."
      : !Number.isFinite(kickHipPitch)
        ? "hip_pitch_offset belum terbaca dari walking_module -- op3_manager belum jalan."
        : kickObservation === null
          ? "Belum ada deteksi sama sekali di /ball_detector_node/circle_set -- jalankan usb_cam + detektor."
          : !obsFresh
            ? `Deteksi terakhir sudah ${fmt(obsAgeSec, 1)} detik lalu.`
            : !kickObservation.circle
              ? "Detektor jalan tapi tidak melihat bola. Dekatkan bola atau turunkan confidence_threshold di halaman Vision."
              : "Sudut tunduk di luar rentang sahih; kepala terlalu datar untuk mengukur.";

    const geometry = computeKickGeometry(kickParams);
    const liveGeometry = kickCurrentParams ? computeKickGeometry(kickCurrentParams) : null;
    const geometryRows = [
      {
        label: "Ambang radius bola (hitung/manual)",
        value: `${fmt(geometry.thresholdPx, 1)} px`,
        note: geometry.usingManual
          ? "diisi manual; hanya memicu jika Radius ikut memutuskan aktif"
          : `hitung sendiri: ideal ${fmt(geometry.idealPx, 1)} px dipotong ${fmt(geometry.margin * 100, 0)} %`,
        highlight: true,
      },
      {
        label: "Sudut tunduk kepala saat bola tepat di jarak tendang",
        value: `${fmt(geometry.depressionDeg, 1)} deg`,
        note: "sudut sinar ke bola setelah koreksi posisi gambar dan kepala; bukan head_tilt mentah",
      },
      {
        label: "Jarak terjauh yang masih terbaca",
        value: Number.isFinite(geometry.maxReadableM) ? `${fmt(geometry.maxReadableM, 2)} m` : "—",
        note: "di luar ini sudut tunduknya di bawah ambang minimum dan bacaannya dibuang",
      },
      {
        label: "Beda tinggi efektif kamera ke pusat bola",
        value: `${fmt(geometry.drop, 3)} m`,
        note: "tinggi kamera dikurangi jari-jari bola",
      },
      {
        label: "Jarak miring kamera ke bola",
        value: `${fmt(geometry.slant, 3)} m`,
        note: `panjang fokus ${fmt(geometry.focalPx, 0)} px`,
      },
    ];

    const renderKickHelp = (field) => (
      <div className="absolute left-4 right-4 top-[calc(100%-4px)] z-30 hidden rounded-xl border border-undip-blue/20 bg-white p-4 text-xs shadow-xl group-hover/param:block group-focus-within/param:block lg:left-5 lg:right-auto lg:w-[380px]">
        <div className="flex items-start justify-between gap-3">
          <div>
            <div className="font-bold text-gray-900">{field.label}</div>
            <div className="mt-1 font-mono text-[11px] text-gray-400">
              {field.key}{field.unit ? ` | ${field.unit}` : ""} | di robot {currentValue(field)} | diketik {showValue(kickParams[field.key], field)}
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
          <div className="bg-white rounded-2xl border border-gray-200 shadow-sm overflow-hidden">
            <div className="px-5 pt-5 pb-3 border-b border-gray-100">
              <div className="flex flex-wrap items-center gap-2">
                <h2 className="studio-control-title text-lg font-bold font-display text-gray-900"><Crosshair size={20} aria-hidden="true" /> Jarak Tendangan</h2>
                <span
                  className={`px-2 py-0.5 rounded-full text-[10px] font-bold uppercase ${kickDirty ? "bg-yellow-100 text-yellow-700" : "bg-green-50 text-green-700"}`}
                  title={kickDirty ? "Ada perubahan yang belum dikirim ke robot" : "Cocok dengan pembacaan terakhir, bukan pemantauan terus-menerus"}
                >
                  {!kickLoaded ? "Belum dibaca" : kickDirty ? "Belum dikirim" : "Cocok saat dibaca"}
                </span>
                <span
                  className={`px-2 py-0.5 rounded-full text-[10px] font-bold uppercase ${kickLoaded ? "bg-blue-50 text-undip-blue" : "bg-gray-100 text-gray-500"}`}
                  title={kickLoaded ? "Angka diambil dari robot" : "Belum dibaca — angka referensi UI, bukan isi launch aktual"}
                >
                  {kickLoaded ? "Dari robot" : "Referensi UI"}
                </span>
                <span
                  className={`px-2 py-0.5 rounded-full text-[10px] font-bold uppercase ${
                    rosNodePresence === null
                      ? "bg-gray-100 text-gray-500"
                      : rosNodePresence.demo
                        ? "bg-green-50 text-green-700"
                        : "bg-red-50 text-red-600"
                  }`}
                  title="Pemilik parameter tendangan. Tanpa node ini, Muat dan Kirim tidak ada yang menjawab."
                >
                  demo_node {rosNodePresence === null ? "?" : rosNodePresence.demo ? "hidup" : "tidak ada"}
                </span>
                <span
                  className={`px-2 py-0.5 rounded-full text-[10px] font-bold uppercase ${
                    rosNodePresence === null
                      ? "bg-gray-100 text-gray-500"
                      : rosNodePresence.agent
                        ? "bg-green-50 text-green-700"
                        : "bg-red-50 text-red-600"
                  }`}
                  title="Yang menulis ke demo.launch.xml. Tanpa node ini, Simpan ke launch tidak bisa dipakai."
                >
                  studio_agent {rosNodePresence === null ? "?" : rosNodePresence.agent ? "hidup" : "tidak ada"}
                </span>
                {kickLastAppliedAt && (
                  <span className="text-[10px] font-mono text-gray-400" title="Waktu terakhir dikirim ke robot">
                    Kirim: {new Date(kickLastAppliedAt).toLocaleTimeString()}
                  </span>
                )}
              </div>
              <p className="mt-2 text-xs text-gray-500 leading-relaxed">
                Angka di halaman ini adalah parameter ROS milik node <span className="font-mono">demo_node</span>, bukan
                isi WalkingParam. Robot membacanya ulang sendiri 0,2 detik setelah dikirim, jadi tidak perlu
                meluncurkan ulang demo. Perubahannya hidup di RAM: sesudah demo dimatikan semuanya kembali ke
                bawaan <span className="font-mono">demo.launch.xml</span> — pakai <span className="font-bold">Simpan ke launch</span> kalau
                setelannya sudah pas dan mau dipermanenkan.
              </p>
            </div>

            <div className="px-5 py-4 flex flex-wrap items-center gap-2">
              <button
                className={`${kickButtonBase} bg-gray-100 text-gray-700 hover:bg-gray-200`}
                onClick={loadKickParams}
                disabled={!rosConnected || kickBusy}
                title="Baca angka yang benar-benar hidup di robot"
              >
                <RefreshCw size={14} /> Muat dari robot
              </button>
              <button
                className={`${kickButtonBase} bg-accent-yellow text-black hover:bg-yellow-400 shadow-sm`}
                onClick={applyKickParams}
                disabled={!rosConnected || kickBusy || !kickLoaded}
                title="Kirim semua angka di halaman ini ke robot sekarang juga"
              >
                <Send size={14} /> Kirim ke robot
              </button>
              <button
                className={`${kickButtonBase} bg-gray-100 text-gray-700 hover:bg-gray-200`}
                onClick={revertKickEdits}
                disabled={kickBusy || !kickLoaded}
                title="Kembalikan kotak isian ke angka terakhir yang dibaca dari robot"
              >
                Batalkan perubahan
              </button>
              <button
                className={`${kickButtonBase} bg-gray-100 text-gray-700 hover:bg-gray-200`}
                onClick={resetKickParamsToLaunch}
                disabled={!rosConnected || kickBusy}
                title="Baca default langsung dari berkas demo.launch.xml melalui studio_agent; hanya mengisi draft"
              >
                Bawaan launch
              </button>
              <button
                className={`${kickButtonBase} ${
                  kickSaveArmed
                    ? "bg-red-600 text-white hover:bg-red-700"
                    : "bg-white text-gray-700 border border-gray-300 hover:bg-gray-50"
                }`}
                onClick={() => {
                  if (kickSaveArmed) saveKickParamsToLaunch();
                  else setKickSaveArmed(true);
                }}
                disabled={!rosConnected || kickBusy || kickDirty || !kickLoaded}
                title={
                  kickDirty
                    ? "Kirim ke robot dulu — yang disimpan adalah angka yang hidup di robot, bukan isi kotak ini"
                    : "Tulis permanen ke demo.launch.xml supaya bertahan sesudah demo dimatikan"
                }
              >
                <Save size={14} /> {kickSaveArmed ? "Yakin? klik lagi" : "Simpan ke launch"}
              </button>
              <div className="flex-1" />
              <button
                className={`${kickButtonBase} bg-undip-blue text-white hover:bg-opacity-90`}
                onClick={() => setActiveTab("demo")}
                title="Buka halaman Demo untuk langsung mencoba hasil tuning"
              >
                <PlayCircle size={14} /> Coba di Demo
              </button>
            </div>

            {kickDirty && (
              <div className="mx-5 mb-3 rounded-xl border border-gray-200 bg-gray-50 px-4 py-3 text-xs text-gray-600">
                <span className="font-bold">Simpan ke launch dikunci.</span> Yang ditulis ke berkas adalah angka
                yang benar-benar hidup di robot, bukan isi kotak di bawah — jadi kirim dulu ke robot, coba di
                demo, baru simpan.
              </div>
            )}
            {kickSaveMessage && (
              <div className="mx-5 mb-3 rounded-xl border border-green-100 bg-green-50 px-4 py-3 text-xs text-green-700">
                <div className="font-bold">
                  Tersimpan ke demo.launch.xml{kickSavedAt ? ` — ${new Date(kickSavedAt).toLocaleTimeString()}` : ""}
                </div>
                <div className="mt-1 font-mono break-all leading-relaxed">{kickSaveMessage}</div>
              </div>
            )}
            {kickError && (
              <div className="mx-5 mb-5 rounded-xl border border-red-100 bg-red-50 px-4 py-3 text-xs text-red-600">
                {kickError}
              </div>
            )}
            {!rosConnected && (
              <div className="mx-5 mb-5 rounded-xl border border-yellow-100 bg-yellow-50 px-4 py-3 text-xs text-yellow-700">
                Rosbridge belum tersambung. Kotak isian tetap bisa diubah, tapi tombol kirim dimatikan.
              </div>
            )}
          </div>

          <div className="grid grid-cols-1 xl:grid-cols-2 gap-6">
            {/* Kamera ditaruh BERSEBELAHAN dengan panel hitungan, bukan di halaman
                lain: yang dinilai operator saat menera jarak adalah lingkaran bola
                di gambar versus ambang radius yang tercetak di sebelahnya. */}
            <div className="bg-white rounded-2xl border border-gray-200 shadow-sm overflow-hidden flex flex-col">
              <div className="p-5 border-b border-gray-100 flex items-start justify-between gap-3">
                <div className="min-w-0">
                  <div className="flex items-center gap-2">
                    <Video size={18} className="text-undip-blue" />
                    <h3 className="font-bold text-gray-900">Kamera</h3>
                    {kickCameraOn && (
                      <span className="px-2 py-0.5 rounded-full text-[10px] font-bold uppercase bg-green-50 text-green-700">
                        {formatNumber(overlayStats.fps, 1)} FPS
                      </span>
                    )}
                    {rosNodePresence && (
                      <>
                        <span
                          className={`px-2 py-0.5 rounded-full text-[10px] font-bold uppercase ${rosNodePresence.camera ? "bg-green-50 text-green-700" : "bg-red-50 text-red-600"}`}
                          title="/usb_cam_node/usb_cam"
                        >
                          usb_cam {rosNodePresence.camera ? "hidup" : "tidak ada"}
                        </span>
                        <span
                          className={`px-2 py-0.5 rounded-full text-[10px] font-bold uppercase ${rosNodePresence.detector ? "bg-green-50 text-green-700" : "bg-red-50 text-red-600"}`}
                          title="/ball_detector_node/yolo_ball_detector"
                        >
                          detektor {rosNodePresence.detector ? "hidup" : "tidak ada"}
                        </span>
                      </>
                    )}
                  </div>
                  <p className="mt-1 text-xs text-gray-500">
                    Tampilan YOLO memperlihatkan lingkaran bola yang jari-jarinya dibandingkan dengan ambang di panel
                    sebelah.
                  </p>
                </div>
                <button
                  type="button"
                  className={`flex-shrink-0 inline-flex items-center gap-2 px-3 py-2 rounded-lg text-xs font-bold transition-colors ${
                    kickCameraOn
                      ? "bg-red-50 text-red-600 border border-red-100 hover:bg-red-100"
                      : "bg-green-50 text-green-700 border border-green-100 hover:bg-green-100"
                  }`}
                  onClick={() => setKickCameraOn((prev) => !prev)}
                  title={
                    kickCameraOn
                      ? "Memutus langganan gambar — aliran datanya benar-benar berhenti, bukan cuma berhenti digambar"
                      : "Berlangganan lagi ke topik gambar"
                  }
                >
                  {kickCameraOn ? <><Square size={14} /> Matikan kamera</> : <><Play size={14} /> Nyalakan kamera</>}
                </button>
              </div>

              <div className="bg-black flex items-center justify-center min-h-[260px] flex-1 relative">
                {kickCameraOn ? (
                  <canvas ref={kickCanvasRef} className="max-w-full max-h-[420px] object-contain" />
                ) : (
                  <div className="text-gray-500 text-sm font-mono py-16">Kamera dimatikan</div>
                )}
                {cameraSilent && (
                  <div className="absolute inset-0 flex items-center justify-center p-6">
                    <div className="max-w-sm rounded-xl border border-amber-300/40 bg-black/85 px-4 py-3 text-xs text-amber-200 leading-relaxed">
                      <div className="font-bold">Belum ada gambar yang masuk.</div>
                      <div className="mt-1 font-mono break-all text-amber-100/80">{overlayTopic}</div>
                      <div className="mt-2 text-amber-100/70">{cameraDiagnosis}</div>
                    </div>
                  </div>
                )}
              </div>

              <div className="p-5 border-t border-gray-100">
                <label className="block text-[10px] font-bold uppercase tracking-wider text-gray-400 mb-2">
                  Sumber gambar
                </label>
                <div className="grid grid-cols-2 gap-2">
                  <button
                    onClick={() => setOverlayTopic("/ball_detector_node/image_out")}
                    className={`py-2 rounded-lg text-[10px] font-bold transition-all border ${overlayTopic === "/ball_detector_node/image_out" ? "bg-undip-blue text-white border-undip-blue" : "bg-gray-50 text-gray-600 border-gray-200 hover:bg-gray-100"}`}
                  >
                    YOLO Debug
                  </button>
                  <button
                    onClick={() => setOverlayTopic("/usb_cam_node/image_raw")}
                    className={`py-2 rounded-lg text-[10px] font-bold transition-all border ${overlayTopic === "/usb_cam_node/image_raw" ? "bg-undip-blue text-white border-undip-blue" : "bg-gray-50 text-gray-600 border-gray-200 hover:bg-gray-100"}`}
                  >
                    Kamera murni
                  </button>
                </div>
                <div className="mt-2 text-[11px] font-mono text-gray-400 break-all">{overlayTopic}</div>
                <div className="mt-1 text-[11px] text-gray-400">
                  Sumbernya sama dengan halaman Vision — mengubahnya di sini ikut mengubah di sana.
                </div>
              </div>
            </div>

            {/* PENGUKUR JARAK: taruh bola di tempat yang diinginkan, baca
                angkanya, lalu pakai. Ini menjawab "berapa kick_distance yang
                pas" tanpa meteran dan tanpa menebak. */}
            <div className="bg-white rounded-2xl border border-gray-200 shadow-sm overflow-hidden">
              <div className="p-5 border-b border-gray-100">
                <div className="flex flex-wrap items-center gap-2">
                  <Ruler size={18} className="text-undip-blue" />
                  <h3 className="font-bold text-gray-900">Ukur jarak bola sekarang</h3>
                  <span className={`px-2 py-0.5 rounded-full text-[10px] font-bold uppercase ${measurement ? "bg-green-50 text-green-700" : "bg-gray-100 text-gray-500"}`}>
                    {measurement ? "terukur" : "menunggu bola"}
                  </span>
                </div>
                <p className="mt-1 text-xs text-gray-500">
                  Letakkan bola tepat di tempat Abang ingin robot BERHENTI, arahkan kepala ke bola,
                  lalu tekan <span className="font-bold">Pakai jarak ini</span>. Dihitung dengan geometri
                  yang sama persis dengan yang dipakai robot saat demo.
                </p>
              </div>

              {measurement && measurement.distanceM !== null ? (
                <>
                  <div className="px-5 py-4 flex flex-wrap items-end gap-x-8 gap-y-3">
                    <div>
                      <div className="text-[10px] font-bold uppercase text-gray-400">Jarak bola</div>
                      <div className="font-mono text-3xl font-bold text-undip-blue">
                        {fmt(measurement.distanceM, 3)} <span className="text-lg">m</span>
                      </div>
                    </div>
                    <div>
                      <div className="text-[10px] font-bold uppercase text-gray-400">Jarak tendang (draft)</div>
                      <div className="font-mono text-xl text-gray-700">{fmt(draftKickDistance, 3)} m</div>
                    </div>
                    <div>
                      <div className="text-[10px] font-bold uppercase text-gray-400">Selisih</div>
                      <div className={`font-mono text-xl font-bold ${Math.abs(measurement.distanceM - draftKickDistance) < 0.02 ? "text-green-600" : "text-amber-600"}`}>
                        {measurement.distanceM > draftKickDistance ? "+" : ""}
                        {fmt((measurement.distanceM - draftKickDistance) * 100, 1)} cm
                      </div>
                    </div>
                  </div>

                  <div className="grid grid-cols-2 sm:grid-cols-4 gap-px bg-gray-100 border-y border-gray-100">
                    {[
                      { l: "Radius bola", v: `${fmt(measurement.radiusPx, 1)} px`,
                        n: `ambang ${fmt(geometry.thresholdPx, 1)} px` },
                      { l: "Sudut tunduk", v: `${fmt(measurement.depressionDeg, 1)} deg`, n: "ke pusat bola" },
                      { l: "Arah bola", v: `${fmt(measurement.bearingDeg, 1)} deg`, n: "+ = kiri" },
                      { l: "Jendela kaki", v: measurement.footWindow === "right" ? "KANAN"
                          : measurement.footWindow === "left" ? "KIRI" : "di luar",
                        n: `servo ${fmt(measurement.servoWindowDeg, 1)} deg` },
                    ].map((k) => (
                      <div key={k.l} className="bg-white px-4 py-3">
                        <div className="text-[10px] font-bold uppercase text-gray-400">{k.l}</div>
                        <div className="font-mono text-sm font-bold text-gray-800">{k.v}</div>
                        <div className="text-[10px] text-gray-400">{k.n}</div>
                      </div>
                    ))}
                  </div>

                  <div className="px-5 py-4 flex flex-wrap items-center gap-3">
                    <button
                      className={`${kickButtonBase} bg-undip-blue text-white hover:bg-undip-blue/90`}
                      onClick={() => {
                        updateKickParam("kick_distance", measurement.distanceM.toFixed(3));
                        sendStatus(`Jarak tendang draft diisi ${measurement.distanceM.toFixed(3)} m dari pengukuran`);
                      }}
                      title="Isi kotak Jarak tendang dengan jarak bola yang barusan diukur"
                    >
                      <Crosshair size={14} /> Pakai jarak ini
                    </button>
                    <span className="text-[11px] text-gray-500">
                      Baru mengisi kotak isian; masih perlu tombol kirim untuk sampai ke robot.
                    </span>
                  </div>

                  {Number.isFinite(draftPanMax) && Number.isFinite(draftKickDistance)
                    && draftPanMax <= draftKickDistance && (
                    <div className="px-5 pb-4 text-[11px] text-yellow-800 bg-yellow-50 border-t border-yellow-100 pt-3">
                      <span className="font-bold">Perhatian:</span> batas mulai penyelarasan
                      ({fmt(draftPanMax, 3)} m) tidak boleh lebih kecil daripada jarak tendang
                      ({fmt(draftKickDistance, 3)} m) -- robot tidak akan pernah mulai meluruskan yaw.
                      Isi sekitar 5 cm lebih besar.
                    </div>
                  )}
                </>
              ) : (
                <div className="px-5 py-6 text-sm text-gray-500">
                  <div className="font-bold text-gray-700">Belum ada angka.</div>
                  <div className="mt-1">{measureBlocker}</div>
                  {measurement && measurement.distanceM === null && (
                    <div className="mt-2 text-[11px]">
                      Arah bola terbaca ({fmt(measurement.bearingDeg, 1)} deg) tapi jaraknya tidak:
                      sudut tunduk {fmt(measurement.depressionDeg, 1)} deg di luar rentang sahih
                      ({fmt(parseWalkingNumber(kickParams.kick_min_depression_deg, NaN), 1)} sampai 89 deg).
                    </div>
                  )}
                </div>
              )}
            </div>

          </div>

          {/* Akibat dari angka yang sedang diketik — dihitung dengan rumus yang
              sama persis dengan yang dipakai robot.

              SELEBAR HALAMAN, dan isinya ubin, bukan daftar menurun. Dulu ia
              kartu ketiga di grid dua kolom di atas, jadi ia sendirian di baris
              kedua dan separuh kanan halaman kosong sepanjang kartunya. */}
          <div className="bg-white rounded-2xl border border-gray-200 shadow-sm overflow-hidden">
              <div className="p-5 border-b border-gray-100">
                <div className="flex items-center gap-2">
                  <Crosshair size={18} className="text-undip-blue" />
                  <h3 className="font-bold text-gray-900">Akibat angka yang sedang diketik</h3>
                </div>
                <p className="mt-1 text-xs text-gray-500">
                  Dihitung dengan rumus yang sama persis dengan <span className="font-mono">BallFollower::loadTunableParameters()</span>.
                  Belum tentu sama dengan yang di robot sampai tombol kirim ditekan.
                </p>
              </div>
              <div>
                <div className="grid grid-cols-1 sm:grid-cols-2 xl:grid-cols-3 gap-3 p-5">
                  {geometryRows.map((row) => (
                    <div key={row.label} className={`rounded-xl border px-4 py-3 ${row.highlight ? "border-undip-blue/20 bg-blue-50/40" : "border-gray-100 bg-gray-50"}`}>
                      <div className={`text-xs ${row.highlight ? "font-bold text-gray-900" : "text-gray-600"}`}>{row.label}</div>
                      <div className={`mt-1 font-mono ${row.highlight ? "text-xl font-bold text-undip-blue" : "text-base text-gray-800"}`}>
                        {row.value}
                      </div>
                      <div className="mt-1 text-[11px] leading-snug text-gray-400">{row.note}</div>
                    </div>
                  ))}
                </div>
                {liveGeometry && (
                  <div className="px-5 pb-4 -mt-1 text-[11px] text-gray-500">
                    Pada pembacaan robot terakhir, ambang radius hitung/manual <span className="font-mono font-bold">{fmt(liveGeometry.thresholdPx, 1)} px</span> pada
                    jarak tendang <span className="font-mono font-bold">{fmt(parseWalkingNumber(kickCurrentParams.kick_distance, NaN), 3)} m</span>.
                  </div>
                )}
              </div>
          </div>

          {/* KOLOM CSS, bukan grid. Keempat grup ini isinya 2, 6, 4 dan 3 field;
              di grid dua kolom tinggi tiap BARIS mengikuti kartu tertinggi, jadi
              "Jarak tendang" (2 field) meninggalkan lubang setinggi 4 field di
              sebelah "Geometri pengukur jarak" (6 field). Kolom CSS menata
              kartunya berurutan dan menyeimbangkan tinggi kedua kolom sendiri.
              Jarak antar kartu pakai mb-6: row-gap tidak berlaku di multikolom. */}
          <div className="columns-1 xl:columns-2 gap-6">
            {KICK_PARAM_GROUPS.map((group) => (
              <div key={group.title} className="mb-6 break-inside-avoid bg-white rounded-2xl border border-gray-200 shadow-sm overflow-visible">
                <div className="p-5 border-b border-gray-100">
                  <div className="flex items-center gap-2">
                    <SlidersHorizontal size={18} className="text-undip-blue" />
                    <h3 className="font-bold text-gray-900">{group.title}</h3>
                  </div>
                  <p className="mt-1 text-xs text-gray-500">{group.description}</p>
                </div>
                <div className="divide-y divide-gray-100">
                  {group.fields.map((field) => {
                    const dirty = isFieldDirty(field);
                    const switchOn = parseWalkingNumber(kickParams[field.key], 0) > 0.5;
                    return (
                      <div key={field.key} className="group/param relative grid grid-cols-1 gap-3 p-4 md:grid-cols-[minmax(170px,1fr)_minmax(180px,220px)_minmax(150px,180px)] md:items-center">
                        <div className="min-w-0">
                          <div className="flex items-center gap-2">
                            <label className="text-sm font-bold text-gray-800">{field.label}</label>
                            <Info size={13} className="text-gray-300 transition-colors group-hover/param:text-undip-blue group-focus-within/param:text-undip-blue" />
                            {dirty && <span className="w-2 h-2 rounded-full bg-yellow-400" title="Diubah, belum dikirim" />}
                          </div>
                          <div className="mt-1 text-[11px] text-gray-400 font-mono break-all">{field.key}{field.unit ? ` (${field.unit})` : ""}</div>
                        </div>

                        {field.type === "switch" ? (
                          // Tetap angka 0/1, bukan boolean: robot mendeklarasikannya
                          // bertipe float dan set_parameters menolak tipe lain.
                          <button
                            type="button"
                            className={`w-full px-3 py-2 rounded-lg border text-sm font-bold transition-colors ${
                              switchOn
                                ? "bg-green-50 text-green-700 border-green-100 hover:bg-green-100"
                                : "bg-gray-50 text-gray-600 border-gray-200 hover:bg-gray-100"
                            }`}
                            onClick={() => updateKickParam(field.key, switchOn ? 0 : 1)}
                            disabled={kickBusy}
                            aria-label={field.key}
                          >
                            {switchOn ? "Aktif (1.0)" : "Mati (0.0)"}
                          </button>
                        ) : (
                          <input
                            type="text"
                            inputMode="decimal"
                            value={kickParams[field.key]}
                            onChange={(e) => updateKickParam(field.key, e.target.value)}
                            disabled={kickBusy}
                            aria-label={field.key}
                            className="w-full px-3 py-2 rounded-lg border border-gray-200 bg-gray-50 text-sm font-mono text-gray-900 focus:outline-none focus:ring-2 focus:ring-undip-blue/20 focus:border-undip-blue"
                          />
                        )}

                        <div className="rounded-lg bg-gray-50 border border-gray-100 px-3 py-2">
                          <div className="text-[10px] font-bold uppercase text-gray-400">Di robot</div>
                          <div className="text-xs font-mono text-gray-700 truncate">{currentValue(field)}</div>
                        </div>
                        {renderKickHelp(field)}
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

  const renderDemo = () => {
    const rosConnected = rosState === "connected";
    const demoButtonBase = "inline-flex items-center justify-center gap-2 px-3 py-2 rounded-lg text-xs font-bold transition-colors disabled:opacity-50 disabled:cursor-not-allowed";
    const demoNodeUp = rosNodePresence?.demo === true;
    const demoNodeKnown = rosNodePresence !== null;
    const controlsDisabled = !rosConnected || (demoNodeKnown && !demoNodeUp);

    return (
      <div className="h-full p-4 md:p-8 overflow-y-auto">
        <div className="max-w-7xl mx-auto flex flex-col gap-5">
          <div className="bg-white rounded-2xl border border-gray-200 shadow-sm overflow-hidden">
            <div className="px-5 pt-5 pb-3 border-b border-gray-100">
              <div className="flex flex-wrap items-center gap-2">
                <h2 className="text-lg font-bold font-display text-gray-900">Demo</h2>
                <span className={`px-2 py-0.5 rounded-full text-[10px] font-bold uppercase ${rosConnected ? "bg-green-50 text-green-700" : "bg-red-50 text-red-600"}`}>
                  {rosConnected ? "Tersambung" : "Terputus"}
                </span>
                <span
                  className={`px-2 py-0.5 rounded-full text-[10px] font-bold uppercase ${
                    !demoNodeKnown
                      ? "bg-gray-100 text-gray-500"
                      : demoNodeUp
                        ? "bg-green-50 text-green-700"
                        : "bg-red-50 text-red-600"
                  }`}
                  title="Node yang menerima semua perintah di halaman ini"
                >
                  demo_node {!demoNodeKnown ? "?" : demoNodeUp ? "hidup" : "tidak ada"}
                </span>
                {demoMode && (
                  <span className="px-2 py-0.5 rounded-full text-[10px] font-bold uppercase bg-blue-50 text-undip-blue">
                    terakhir dikirim: {demoMode}
                  </span>
                )}
                <button
                  className="px-2 py-0.5 rounded-full text-[10px] font-bold uppercase bg-gray-100 text-gray-600 hover:bg-gray-200 disabled:opacity-50"
                  onClick={refreshRosNodes}
                  disabled={!rosConnected}
                >
                  Periksa lagi
                </button>
                {rosNodeCheckedAt && (
                  <span className="text-[10px] font-mono text-gray-400">
                    {new Date(rosNodeCheckedAt).toLocaleTimeString()}
                  </span>
                )}
              </div>
            </div>

            {demoNodeKnown && !demoNodeUp && (
              <div className="mx-5 mt-4 rounded-xl border border-red-100 bg-red-50 px-4 py-3 text-xs text-red-700">
                <div className="font-bold">demo_node tidak ditemukan di robot.</div>
                <div className="mt-1 leading-relaxed">
                  Selama node ini belum ada, semua tombol di halaman ini terkirim ke ruang hampa — robot tidak akan
                  bereaksi sama sekali. Luncurkan demonya dulu di robot:
                  <span className="font-mono block mt-1">ros2 launch op3_demo demo.launch.xml</span>
                </div>
              </div>
            )}

            {/* Perilaku sebenarnya di robot, bukan tebakan: memilih mode SUDAH
                memulai demonya, dan "start" itu toggle. */}
            <div className="mx-5 my-4 rounded-xl border border-amber-200 bg-amber-50 px-4 py-3 text-xs text-amber-800 leading-relaxed">
              <span className="font-bold">Memilih mode sudah memulai demonya.</span> Tidak perlu menekan Lanjut
              sesudahnya — di robot, memilih mode langsung memanggil rutin start-nya. Menekan Lanjut saat demo
              sedang berjalan justru <span className="font-bold">menghentikannya</span>, karena perintah start itu
              toggle. Kalau ragu, tekan Jeda dulu.
            </div>
          </div>

          <div className="bg-white rounded-2xl border border-gray-200 shadow-sm overflow-hidden">
            <div className="p-5 border-b border-gray-100">
              <div className="flex items-center gap-2">
                <PlayCircle size={18} className="text-undip-blue" />
                <h3 className="font-bold text-gray-900">Mulai demo</h3>
              </div>
              <p className="mt-1 text-xs text-gray-500">
                Satu klik = kirim mode ke /robotis/mode_command, dan robot langsung menjalankannya. Mode hanya bisa
                dipilih dari keadaan siap; kalau demo lain masih berjalan, tekan Berhenti dulu.
              </p>
            </div>
            <div className="divide-y divide-gray-100">
              {DEMO_MODES.map((mode) => (
                <button
                  key={mode.key}
                  type="button"
                  className={`w-full text-left px-5 py-4 transition-colors disabled:opacity-50 disabled:cursor-not-allowed ${
                    demoMode === mode.key ? "bg-blue-50/60" : "hover:bg-gray-50"
                  }`}
                  onClick={() => handleDemoMode(mode.key)}
                  disabled={controlsDisabled}
                >
                  <div className="flex items-center gap-2">
                    <Play size={15} className={demoMode === mode.key ? "text-undip-blue" : "text-gray-400"} />
                    <span className={`text-sm font-bold ${demoMode === mode.key ? "text-undip-blue" : "text-gray-800"}`}>
                      Mulai {mode.label}
                    </span>
                    <span className="font-mono text-[11px] text-gray-400">{mode.key}</span>
                    {demoMode === mode.key && (
                      <span className="px-1.5 py-0.5 rounded text-[10px] font-bold uppercase bg-undip-blue text-white">terkirim</span>
                    )}
                  </div>
                  <p className="mt-1 text-[11px] text-gray-500 leading-relaxed">{mode.description}</p>
                </button>
              ))}
            </div>
          </div>

          <div className="grid grid-cols-1 xl:grid-cols-2 gap-6">
            <div className="bg-white rounded-2xl border border-gray-200 shadow-sm overflow-hidden">
              <div className="p-5 border-b border-gray-100">
                <div className="flex items-center gap-2">
                  <StopCircle size={18} className="text-undip-blue" />
                  <h3 className="font-bold text-gray-900">Saat demo berjalan</h3>
                </div>
                <p className="mt-1 text-xs text-gray-500">Berlaku untuk Soccer dan Action; mode Vision tidak memakainya.</p>
              </div>
              <div className="p-5 grid grid-cols-2 gap-3">
                <button
                  className="py-4 rounded-xl text-sm font-bold bg-red-50 text-red-600 border border-red-100 hover:bg-red-100 transition-colors disabled:opacity-50 disabled:cursor-not-allowed inline-flex items-center justify-center gap-2"
                  onClick={() => handleDemoCommand("stop")}
                  disabled={controlsDisabled}
                  title="Selalu menghentikan, aman ditekan walaupun demo sudah berhenti"
                >
                  <Square size={16} /> Jeda
                </button>
                <button
                  className="py-4 rounded-xl text-sm font-bold bg-green-50 text-green-700 border border-green-100 hover:bg-green-100 transition-colors disabled:opacity-50 disabled:cursor-not-allowed inline-flex items-center justify-center gap-2"
                  onClick={() => handleDemoCommand("start")}
                  disabled={controlsDisabled}
                  title="Toggle di robot: melanjutkan kalau sedang berhenti, MENGHENTIKAN kalau sedang berjalan"
                >
                  <Play size={16} /> Lanjut
                </button>
                <button
                  className="col-span-2 py-3 rounded-xl text-sm font-bold bg-gray-100 text-gray-700 border border-gray-200 hover:bg-gray-200 transition-colors disabled:opacity-50 disabled:cursor-not-allowed inline-flex items-center justify-center gap-2"
                  onClick={() => handleDemoMode("ready")}
                  disabled={controlsDisabled}
                  title="Keluar dari demo dan kembali ke posisi siap"
                >
                  <StopCircle size={16} /> Berhenti &amp; kembali siap
                </button>
              </div>
              <div className="px-5 pb-5 text-[11px] text-gray-500">
                {demoCommandAt
                  ? `Terakhir: ${new Date(demoCommandAt).toLocaleTimeString()} — ${demoCommand}`
                  : "Belum ada perintah demo yang dikirim dari halaman ini."}
                <div className="mt-1 text-gray-400">
                  Penanda di atas mencatat apa yang DIKIRIM halaman ini, bukan bacaan balik dari robot. Kalau tombol
                  fisik di punggung robot ikut dipakai, keduanya bisa berbeda.
                </div>
              </div>
            </div>

            <div className="bg-white rounded-2xl border border-gray-200 shadow-sm overflow-hidden">
              <div className="p-5 border-b border-gray-100">
                <div className="flex items-center gap-2">
                  <SlidersHorizontal size={18} className="text-undip-blue" />
                  <h3 className="font-bold text-gray-900">Persiapan cepat</h3>
                </div>
                <p className="mt-1 text-xs text-gray-500">Torsi dan posisi siap sebelum demo dimulai, tanpa pindah halaman.</p>
              </div>
              <div className="p-5 flex flex-wrap gap-2">
                <button
                  className={`${demoButtonBase} bg-gray-100 text-gray-700 hover:bg-gray-200`}
                  onClick={() => handleTorque(true)}
                  disabled={!rosConnected}
                >
                  Torsi ON
                </button>
                <button
                  className={`${demoButtonBase} bg-gray-100 text-gray-700 hover:bg-gray-200`}
                  onClick={handleInitPose}
                  disabled={!rosConnected}
                >
                  Init Pose (page {WALKING_READY_PAGE_NUM})
                </button>
                <button
                  className={`${demoButtonBase} bg-gray-100 text-gray-700 hover:bg-gray-200`}
                  onClick={handleSoftStop}
                  disabled={!rosConnected}
                >
                  <StopCircle size={14} /> Stop jalan
                </button>
                <button
                  className={`${demoButtonBase} bg-red-50 text-red-600 hover:bg-red-100`}
                  onClick={() => handleTorque(false)}
                  disabled={!rosConnected}
                >
                  Torsi OFF
                </button>
                {torqueLimitActive && (
                  <button
                    className={`${demoButtonBase} bg-amber-100 text-amber-800 border border-amber-200 hover:bg-amber-200`}
                    onClick={() => restoreFullSpeed("Kecepatan penuh dikembalikan")}
                  >
                    Batas kecepatan aktif — lepas
                  </button>
                )}
                <div className="w-full" />
                <button
                  className={`${demoButtonBase} bg-undip-blue text-white hover:bg-opacity-90`}
                  onClick={() => setActiveTab("tendangan")}
                >
                  <Crosshair size={14} /> Setel jarak tendangan
                </button>
              </div>
              <div className="px-5 pb-5 text-[11px] text-gray-400 leading-relaxed">
                Torsi OFF melepas seluruh servo — pegang robotnya dulu.
                <div className="mt-1">
                  Pilihan Lambat/Sedang/Cepat di Dashboard mengekang sentakan SERVO saat torsi menyala. Laju Init
                  Pose ditentukan halaman di ALPHONSE.bin dan laju jalan oleh walking_module, jadi keduanya tidak
                  ikut melambat. Memulai demo otomatis melepas batasnya — gait butuh kecepatan penuh.
                </div>
              </div>
            </div>
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
    <div className="studio-dashboard flex flex-col lg:grid lg:grid-cols-[3fr_1fr] gap-6 h-full p-4 md:p-8 overflow-y-auto">
      <div className="studio-dashboard-primary flex flex-col gap-6">
        <section className="studio-overview" aria-label="Bascorro control room">
          <div className="studio-overview-copy">
            <span className="studio-eyebrow"><span /> BASCORRO CONTROL ROOM</span>
            <h2>Seluruh kendali.<br /><span>Satu workspace.</span></h2>
            <p>Pantau kondisi, atur gerakan, dan kenali setiap langkah robot Anda.</p>
            <div className="studio-overview-tags"><span><Cpu size={13} /> ROBOTIS OP3</span><span><Activity size={13} /> Motion &amp; vision</span></div>
          </div>
          <RobotBlueprint />
        </section>
        {/* Safety & Quick Actions */}
        <div className="studio-quick-actions flex flex-wrap gap-4 items-center bg-white p-4 rounded-2xl border border-gray-200 shadow-sm">
          <div className="studio-toolbar-caption"><span>Kontrol utama</span><span>Pose, gerakan &amp; torque</span></div>
          <button className="flex-1 sm:flex-none flex items-center justify-center gap-2 px-5 py-2.5 bg-red-50 text-red-600 border border-red-100 rounded-lg font-bold hover:bg-red-600 hover:text-white transition-all shadow-sm" onClick={handleInitPose}>
            <Settings size={18} /> Init Pose
          </button>
          <button className="flex-1 sm:flex-none flex items-center justify-center gap-2 px-5 py-2.5 bg-red-50 text-red-600 border border-red-100 rounded-lg font-bold hover:bg-red-600 hover:text-white transition-all shadow-sm" onClick={handleSoftStop}>
            <StopCircle size={18} /> Soft Stop
          </button>
          <div className="flex-1 hidden sm:block"></div>
          <div className="flex gap-2 w-full sm:w-auto items-center flex-wrap">
            <div
              className="flex bg-blue-50 border border-blue-200 rounded-lg p-1 text-xs font-medium"
              title="Batas kecepatan servo saat Torque ON. Mengekang sentakan servo, BUKAN laju Init Pose atau gait."
            >
              {TORQUE_SPEED_PRESETS.map(p => (
                <button
                  key={p.key}
                  className={`px-3 py-1 rounded-md transition-all font-semibold ${torqueSpeedKey === p.key ? "bg-undip-blue text-white shadow-sm" : "text-undip-blue hover:bg-blue-100"}`}
                  onClick={() => setTorqueSpeedKey(p.key)}
                  title={`~${p.approxDegPerSec} deg/detik (profile_velocity ${p.velocity})`}
                >{p.label}</button>
              ))}
            </div>
            {torqueLimitActive && (
              <button
                className="flex-1 sm:flex-none px-3 py-2 rounded-lg text-xs font-bold bg-amber-100 text-amber-800 border border-amber-200 hover:bg-amber-200 transition-all"
                onClick={() => restoreFullSpeed("Kecepatan penuh dikembalikan")}
                title="Batas kecepatan sedang terpasang. Klik untuk melepasnya."
              >
                Batas aktif — lepas
              </button>
            )}
            <button className="flex-1 sm:flex-none px-4 py-2 border border-gray-200 rounded-lg text-gray-600 font-medium hover:bg-gray-50 text-sm" onClick={() => handleTorque(false)}>Torque OFF</button>
            <button className="flex-1 sm:flex-none px-4 py-2 bg-undip-blue text-white rounded-lg font-bold hover:bg-opacity-90 shadow-sm transition-all text-sm" onClick={() => handleTorque(true)}>Torque ON</button>
          </div>
          <div className="flex gap-2 w-full sm:w-auto">
            <button className="flex-1 sm:flex-none px-4 py-2 bg-green-500 text-white rounded-lg font-bold hover:bg-green-600 shadow-sm transition-all text-sm" onClick={handleEnableHeadModule}>Enable Head</button>
          </div>
        </div>

        {/* Stats Grid */}
        <div className="studio-stat-grid grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-4 gap-6">
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

      <div className="studio-dashboard-secondary flex flex-col gap-6">
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
              <span className="flex items-center gap-2 text-sm text-gray-500">
                Yaw
                {/* Yaw berasal dari magnetometer OpenCR. Tanpa penanda ini, yaw yang
                    dipaksa 0 (sensor belum siap) tidak bisa dibedakan dari robot yang
                    memang sedang menghadap arah awalnya. */}
                <span className={`px-1.5 py-0.5 rounded text-[10px] font-bold uppercase tracking-wide ${MAG_STATUS[imu.mag_status]?.cls || "bg-gray-100 text-gray-400"}`}>
                  {MAG_STATUS[imu.mag_status]?.label || "n/a"}
                </span>
              </span>
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
                  onClick={() => setOverlayTopic("/ball_detector_node/image_out")} 
                  className={`py-2 rounded-lg text-[10px] font-bold transition-all border ${overlayTopic === "/ball_detector_node/image_out" ? 'bg-undip-blue text-white border-undip-blue' : 'bg-gray-50 text-gray-600 border-gray-200 hover:bg-gray-100'}`}
                >
                  YOLO Debug
                </button>
                <button 
                  onClick={() => setOverlayTopic("/usb_cam_node/image_raw")} 
                  className={`py-2 rounded-lg text-[10px] font-bold transition-all border ${overlayTopic === "/usb_cam_node/image_raw" ? 'bg-undip-blue text-white border-undip-blue' : 'bg-gray-50 text-gray-600 border-gray-200 hover:bg-gray-100'}`}
                >
                  Pure Camera
                </button>
              </div>
              <p className="mt-2 text-[11px] text-gray-400">
                usb_cam menerbitkan <span className="font-mono">/usb_cam_node/image_raw</span>, dan detektor bola
                menerbitkan gambar beranotasi di <span className="font-mono">/ball_detector_node/image_out</span>.
                Keduanya berasal dari berkas launch yang sama dengan yang dipakai demo, dan dijalankan oleh
                pilihan <span className="font-bold">YOLO Vision</span> di peluncur.
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
                <div className="flex justify-between mb-1">
                  <label className="text-xs font-bold text-gray-500 uppercase tracking-wider">
                    {YOLO_PARAM_INFO[key]?.label || key}
                  </label>
                  <span className="text-xs font-mono font-bold text-undip-blue">{yoloParams[key].toFixed(2)}</span>
                </div>
                <input
                  type="range" min="0.05" max="0.95" step="0.01"
                  className="w-full accent-undip-blue h-2 bg-gray-100 rounded-lg appearance-none cursor-pointer"
                  value={yoloParams[key]}
                  onChange={e => setYoloParams({ ...yoloParams, [key]: Number(e.target.value) })}
                />
                <p className="mt-1 text-[11px] leading-relaxed text-gray-500">{YOLO_PARAM_INFO[key]?.help}</p>
              </div>
            ))}
            <p className="text-[11px] text-gray-400">
              Ditulis ke <span className="font-mono">/ball_detector_node/yolo_ball_detector</span>, langsung
              berlaku tanpa memuat ulang model. Tidak permanen: kembali ke isi
              <span className="font-mono"> yolo_ball_detector_openvino_params.yaml</span> saat detektor dijalankan ulang.
            </p>
            <button className="w-full py-3 bg-accent-yellow text-black hover:bg-yellow-400 rounded-lg text-sm font-bold transition-colors shadow-sm" onClick={applyYoloParams}>Kirim ke detektor</button>
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
    <div className="studio-shell flex h-screen bg-gray-50 font-sans text-gray-900 overflow-hidden">
      {renderSidebar()}
      <main className="studio-workspace flex-1 flex flex-col min-w-0 bg-[#f8fafc]">
        <header className="studio-topbar bg-white border-b border-gray-200 flex items-center justify-between px-4 md:px-8 flex-shrink-0 z-20">
          <div className="studio-page-heading flex items-center gap-4">
            <button 
              onClick={() => setSidebarOpen(true)}
              aria-label="Buka navigasi"
              className="lg:hidden p-2 -ml-2 text-gray-600 hover:bg-gray-100 rounded-lg transition-colors"
            >
              <Menu size={24} />
            </button>
            <div className="min-w-0">
              <div className="studio-breadcrumb">Studio <span>/</span> {STUDIO_PAGE_DETAILS[activeTab]?.group}</div>
              <h1 className="text-xl font-bold font-display text-gray-900 tracking-tight">{STUDIO_PAGE_DETAILS[activeTab]?.title || activeTab}</h1>
              <p className="studio-page-description">{STUDIO_PAGE_DETAILS[activeTab]?.description}</p>
            </div>
          </div>
          <div className="studio-topbar-status">
            <div className={`studio-connection-badge ${rosState === "connected" ? "is-connected" : "is-disconnected"}`}>
              <span /><Wifi size={14} /><span>{rosState === "connected" ? "Robot terhubung" : "Belum terhubung"}</span>
            </div>
            <div role="status" aria-live="polite" className={`studio-notice px-4 py-1.5 rounded-full text-sm font-medium transition-all ${studioError ? "bg-red-50 text-red-600 border border-red-100" : "bg-green-50 text-green-600 border border-green-100"} ${!studioStatus && "opacity-0"}`}>
              {studioStatus || "Ready"}
            </div>
          </div>
        </header>
        <div className="studio-content flex-1 overflow-hidden relative" data-studio-page={activeTab}>
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
            <TuningPage
              ros={rosRef.current} rosState={rosState} joyState={joyState} publishJoy={publishJoy} sendStatus={sendStatus}
              walkingControls={{
                busy: walkingBusy, load: loadWalkingParams, apply: applyWalkingTuner, enable: enableWalkingModule,
                start: startWalkingNow, stop: () => sendWalkingCommand("stop"),
                balance: sendBalanceCommand, save: saveWalkingRuntime,
              }}
            />
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
              normalizeParams={normalizeWalkingParams}
              enableWalking={enableWalkingForRl}
              cancelWalking={cancelPendingMotion}
            />
          </div>
          <div className={activeTab === "tendangan" ? "h-full" : "hidden"}>
            {renderKick()}
          </div>
          <div className={activeTab === "demo" ? "h-full" : "hidden"}>
            {renderDemo()}
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
