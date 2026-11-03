# Panduan Demo Otomatis D60 (ALPHONSE, 17 Sep 2026)

Rangkuman kondisi AKHIR demo D60 di ALPHONSE, untuk dibuat ulang di robot lain.
Bagian A = daftar gerakan, page, dan parameter. Bagian B = prompt siap tempel.

Tidak termasuk: geser singkat pra-tendang sesudah PUTAR KANAN NEW, dan kalibrasi IMU 5 titik.

---

# BAGIAN A — Daftar

## A1. Gerakan (save-an Bascorro Studio, nama PERSIS)

| Nama save-an | Dipakai untuk |
|---|---|
| `JALAN MENUJU MEKKAH` | INIT (tanpa start, pose berdiri dari page 2), APPROACH (mendekati bola), DISTANCE RECHECK (maju sekali lagi) |
| `MENCARI CELAH` | mencari bola saat tidak terdeteksi, dan SearchAlign (yaw asli, atau yaw dibalik saat runtime) |
| `Putar Kiri New` | koreksi yaw saat yaw > +6° |
| `Putar Kanan New` | koreksi yaw saat yaw < −0,5° |
| `KIRI MIN NEW` | geser tarik < 45 %, geser tengah 50–52 % |
| `KANAN MIN NEW` | geser tarik > 55 %, geser tengah 48–50 % |

## A2. Action Page (Action Editor)

| Page | Fungsi |
|---|---|
| **2** | WALKING_READY: pose init saat boot, sekaligus acuan pose INIT JALAN MENUJU MEKKAH |
| **104** | tendang kiri |
| **105** | tendang kanan |
| **85** | bangun dari jatuh depan |
| **86** | bangun dari jatuh belakang |

## A3. Parameter yang bisa diubah (nilai sekarang)

### Tombol dan firmware
| Parameter | Nilai | Keterangan |
|---|---|---|
| `d60_button_bit` (open_cr_module) | 4 | bit register Button untuk tombol D60 (pin 60) |
| `auto_demo_button` | d60 | nama pesan tombol yang memicu D60 |

### Nama preset dan page
| Parameter | Nilai |
|---|---|
| `auto_demo_preset_init` | JALAN MENUJU MEKKAH |
| `auto_demo_preset_approach` (jalan: APPROACH + DISTANCE RECHECK) | JALAN BARU |
| `auto_demo_preset_search` | MENCARI CELAH |
| `auto_demo_preset_turn_left` / `auto_demo_preset_turn_right` | Putar Kiri New / Putar Kanan New |
| `auto_demo_preset_shift_left` / `auto_demo_preset_shift_right` | KIRI MIN NEW / KANAN MIN NEW |
| `auto_demo_page_kick_left` / `auto_demo_page_kick_right` | 104 / 105 |
| `auto_demo_page_getup_front` / `auto_demo_page_getup_back` | 85 / 86 |
| `auto_demo_pose_reference_path` (6 offset acuan page 2) | kosong = `walking_ready_reference.json` di samping `walking_versions.json` — berkas yang SAMA dengan Bascorro Studio, ditulis `scripts/walking_ready_fk.py` (FK page 2 `ALPHONSE.bin`). Sesi DITOLAK kalau berkas versi bertanda acuan lain (`poseReferenceId`) |
| `auto_demo_studio_versions_path` | kosong = `src/bascorro_studio/data/walking_versions.json` |

### Waktu
| Parameter | Nilai |
|---|---|
| `auto_demo_transition_sec` | 0,4 s (dihitung sesudah soft start gait terakhir selesai dan badan diam) |
| `auto_demo_start_search_sec` | 7,0 s (diam mencari sesudah INIT, KHUSUS tepat sesudah D60 ditekan) |
| `auto_demo_initial_search_sec` | 3,0 s (sesudah batas MENCARI CELAH 60 s) |
| `auto_demo_post_action_search_sec` | 1,5 s |
| `auto_demo_lost_search_sec` | 3,0 s |
| `auto_demo_ball_lost_sec` | 1,0 s |
| `auto_demo_ball_valid_gap_sec` | 5/30 s |
| `auto_demo_search_to_walk_delay_sec` | 0 s (tambahan; jeda umum 0,4 s sudah berlaku) |
| `auto_demo_approach_timeout_sec` | 30 s |
| `auto_demo_shift_timeout_sec` | 8 s |
| `auto_demo_search_timeout_sec` | 60 s |
| `auto_demo_yaw_timeout_sec` | 20 s |

### Servo 19 (persen horizontal) dan servo 20 (jarak)
| Parameter | Nilai |
|---|---|
| `auto_demo_servo_center_deg` / `auto_demo_servo_dir` | 180 / 1 |
| `auto_demo_head_pan_left_limit_deg` / `auto_demo_head_pan_right_limit_deg` | +85 / −85 (0 % / 100 %) |
| `auto_demo_tilt_limit_deg` | 200 (servo 20 > 200 = dekat) |
| `auto_demo_pan_kick_left` / `auto_demo_pan_center` / `auto_demo_pan_kick_right` | 48 / 50 / 52 |
| `auto_demo_pan_outer_left` / `auto_demo_pan_outer_left_stop` | 45 / 46 |
| `auto_demo_pan_outer_right` / `auto_demo_pan_outer_right_stop` | 55 / 54 |
| `auto_demo_search_align_left_max` / `auto_demo_search_align_right_min` | 30 / 70 |
| `auto_demo_search_align_stop_left` / `auto_demo_search_align_stop_right` | 40 / 60 |

### Yaw
| Parameter | Nilai |
|---|---|
| `auto_demo_yaw_target_deg` | 0 |
| `auto_demo_yaw_tolerance_deg` | 6 |
| `auto_demo_yaw_stop_deg` | 4 |
| `auto_demo_yaw_tolerance_neg_deg` | 0,5 (putar kanan mulai di yaw < −0,5) |
| `auto_demo_yaw_stop_neg_deg` | 0 (putar kanan berhenti di yaw >= 0) |

### Setir mengikuti kepala saat mendekat
| Parameter | Nilai |
|---|---|
| `auto_demo_steer_gain` | 0,2 |
| `auto_demo_steer_ramp_deg` | 0,5 °/tick |
| `auto_demo_steer_min_deg` / `auto_demo_steer_max_deg` | 5 / 15 |
| `auto_demo_steer_deadband_deg` | 5 |
| `auto_demo_steer_sign` | 1 |
| `auto_demo_steer_publish_period_sec` | 0,1 s |

### Parkir kepala sebelum Action Page
| Parameter | Nilai |
|---|---|
| `auto_demo_head_park_pan_deg` / `auto_demo_head_park_tilt_servo_deg` | 0° / 200 |
| `auto_demo_head_park_wait_sec` / `auto_demo_head_park_tol_deg` | 1,5 s / 3° |
| `head_park_before_action_sec` (D56) | 1,1 s |
| `auto_kick_head_park_wait_sec` (D58) | 1,2 s |

### Sapuan kepala (`op3_ball_localization/config/head_tracking.yaml`)
| Parameter | Nilai |
|---|---|
| `scan_mode` | legacy (D60 mengubahnya lewat perintah) |
| `scan_pan_left_limit_deg` / `scan_pan_right_limit_deg` | +85 / −85 |
| `scan_start_pan_percent` / `scan_prestart_tilt_servo_deg` / `scan_start_tilt_servo_deg` | 50 / 180 / 210 |
| `scan_stages_percent_servo` | (70,207) (100,180) (100,135) (50,135) (50,207) (30,207) (0,180) (0,135) (50,135) (50,207) |
| `scan_walk_percent_servo` | (0,169) (100,169) |
| `scan_speed_deg_s` / `scan_min_stage_sec` | 30 / 0,2 |
| `park_pan_deg` / `park_tilt_servo_deg` | 0 / 200 |
| tracking (tidak diubah) | `p_gain` 0,30 · `d_gain` 0,045 · `center_deadzone` 0,10 · `lost_timeout` 0,8 · `lock_hold_sec` 2,0 |

### Batas kepala (`op3_manager.launch.py` / `head_control_module`)
| Parameter | Nilai |
|---|---|
| `angle_unit` | 35 °/s |
| `head_tilt_min_deg` / `head_tilt_max_deg` | −45 / +75 |
| `head_tilt_envelope_enable` | true |
| `head_tilt_envelope_center_deg` | 30 (servo 20 210 di 30–70 %; diset di op3_manager.launch.py) |
| `head_tilt_envelope_edge_deg` | 0 (servo 20 180 di 0 % / 100 %) |
| `head_tilt_envelope_flat_percent` | 30 |

---

# BAGIAN B — PROMPT

````
Buat autonomous demo D60 pada stack ROBOTIS OP3 ROS 2 robot ini. Pelajari dulu implementasi existing: demo_node
(D56 soccer, D58 auto_kick), demo.launch.xml, detektor "Bola-Baru" (/ball_detector_node/circle_set),
head_tracking_node (/ball_tracker/command, /ball_tracker/status), head_control_module, action_module
(page_num, is_running, movement_done, /robotis/status), walking_module (set_params, get_params, command),
fall_detector D56, dan walking_versions.json Bascorro Studio. Jangan menebak nama fungsi/topik.

LARANGAN: jangan buat detector, tracker, walking engine, motion, atau trajectory baru. Semua gerakan dari
save-an Studio dan Action Page yang ada. Jangan ubah data Studio, Action Editor, atau timing page. Jangan
rusak D56/D58. D60 ditulis sebagai kelas terpisah (mis. AutoDemo). Semua logika non-blocking (tick ±30 Hz,
service async). Publisher selalu anggota kelas. RCLCPP_*_THROTTLE pakai clock anggota kelas. Semua angka
dijadikan parameter launch auto_demo_* (lihat daftar parameter).

1. TOMBOL
- Firmware OpenCR: tambah S5 = pin 60 -> bit 4 register Button (INPUT_PULLUP). D59 JANGAN diganti (pemutus
  power DXL). open_cr_module menerbitkan "d60" dari parameter d60_button_bit (bawaan 4), dan WARN untuk bit
  menyala yang tidak dipetakan.
- D60 MANDIRI (status demo sendiri): dari Ready langsung mulai; dari demo lain, demo itu setDemoDisable dulu.
  Toggle: OFF->ON mulai dari AWAL (semua flag/timer reset, bukan resume), ON->OFF berhenti aman. Ditekan saat
  STOPPING = antre mulai dari awal. mode_long saat D60 hidup = D60 OFF (D56/D58 dinonaktifkan 19 Sep). Sesudah STOPPED status Ready
  tanpa page 2. Uji tanpa tombol: /robotis/demo_command "auto_toggle" / "auto_stop".
- Perbaiki op3_manager dxlTorqueCheckCallback: saat loop kontrol jalan (read1Byte = COMM_PORT_BUSY) jangan
  restart loop; kirim torque_enable=1 lewat syncWriteItemCallback. Kalau tidak, manager mati saat demo
  dinyalakan ulang.

2. YAW NOL
- open_cr_module: topik /robotis/open_cr/reset_yaw -> arah robot saat itu = yaw 0 (offset diterapkan pada yaw
  yang diterbitkan di /robotis/open_cr/imu, filter tidak diubah).
- Reset saat D60 MULAI (bukan saat mati; sesudah mati yaw jalan terus). Tombol D56/D58 tidak lagi mereset (dinonaktifkan 19 Sep).

3. GERAKAN
- Preset dicari dengan nama PERSIS dan diterbitkan UTUH (init pose, soft start, dan semua field milik save-an
  itu), dibaca-balik via get_params sebelum "start".
- INIT = preset JALAN MENUJU MEKKAH dengan 6 offset bias diganti acuan page 2 (seperti Studio) -> kaki ke
  walking_module -> preset JMM TANPA start -> tunggu kaki diam.
- Setelah Action Page selesai, dan saat diam: kembali ke INIT JMM.
- Ukur arah nyata preset geser/putar di robot tujuan (di ALPHONSE namanya terbalik dari arah geraknya).

4. KONVENSI SERVO
- Servo 19 persen = (85 - pan_deg)/170*100: 0 % kiri, 50 % tengah, 100 % kanan. Hanya untuk posisi horizontal.
- Servo 20 = 180 + tilt_deg (ALPHONSE: positif = menunduk). Hanya untuk jarak: > 200 dekat, <= 200 jauh.
- HEAD TRACK = status tracker "TRACKING" DAN deteksi segar (<= 5/30 s). Nilai servo hanya sah saat TRACK.
  Bola hilang -> ballTrackingValid=false, data lama tidak dipakai.

5. STATE MACHINE (transisi normal 0,4 s, dihitung sesudah soft start gait terakhir selesai dan badan diam; flag sekali per percobaan: yawCorrectionAttempted,
   distanceCorrectionAttempted, shiftAttempted, outerShiftAttempted, kickExecuted, approachCompleted,
   yawStageCompleted, distanceStageCompleted)
a) INIT -> STAND_SEARCH: diam, kepala menyapu, maks 3 s. Ketemu -> cek SEARCH_ALIGN, lalu lanjut sesuai flag.
   Tidak ketemu -> SEARCH_BALL.
b) SEARCH_BALL: MENCARI CELAH + sapuan kepala mode walk. Bola ketemu -> cek SEARCH_ALIGN; kalau tidak dipicu
   -> stop -> gait reda -> jeda 0,4 s -> gerakan berikutnya. Batas 60 s -> INIT.
c) SEARCH_ALIGN (bola pertama terlihat saat mencari):
   - servo 19 <= 30 % -> MENCARI CELAH dengan angle_move_amplitude DIBALIK tandanya (runtime saja, field lain
     identik) sampai > 40 %;
   - >= 70 % -> MENCARI CELAH asli sampai < 60 %;
   - di antaranya tidak dipicu.
   Simpan arah pemicu, jangan dipicu ulang, kepala tetap search/track, keputusan badan lain ditahan. Kalau
   gait MENCARI CELAH sudah jalan, ganti parameter live tanpa stop/start. Selesai -> gait reda -> jeda
   0,4 s -> lanjut.
d) APPROACH (JMM): badan menyetir mengikuti kepala: kalau |pan| > 5°, angle = sign*clamp(|pan|*0,2; 5..15°),
   berubah maks 0,5°/tick, preset JMM diterbitkan ulang tiap 0,1 s hanya dengan angle_move_amplitude diubah.
   Servo 20 > 200 -> stop -> YAW. Batas 30 s.
e) YAW (IMU, sekali): > +6 -> Putar Kiri New, < -0,5 -> Putar Kanan New, dalam -0,5..+6 -> lewati. Berhenti begitu
   yaw di dalam 0..+4 walaupun hanya sesaat (dijepit di callback IMU ±125 Hz). Kelebihan putar -> tetap lanjut.
   mag_status 0/1 -> lewati. Batas 20 s.
f) DISTANCE_RECHECK: servo 20 <= 200 -> maju JMM SEKALI sampai > 200. Masih jauh sesudahnya -> percobaan baru
   dari INIT + cari 1,5 s.
g) BALL_ALIGNMENT (servo 19 %, saat TRACK):
   - < 45 -> KIRI MIN NEW sekali (tarik) sampai >= 46, lalu tabel dibaca lagi;
   - > 55 -> KANAN MIN NEW sekali (tarik) sampai <= 54, lalu tabel dibaca lagi;
   - < 48 -> tendang kiri page 104;
   - 48-50 -> KANAN MIN NEW sekali; begitu < 48 TERSENTUH (dijepit di callback servo) -> stop -> langsung 104;
   - 50-52 -> KIRI MIN NEW sekali; begitu > 52 TERSENTUH -> stop -> langsung 105;
   - > 52 -> tendang kanan page 105.
   Batas geser 8 s. Batas tidak tersentuh dan masih 48-52 -> percobaan baru dari INIT + 1,5 s.
h) KICK: syarat (dinilai ulang tiap tick) = D60 aktif, bola terdeteksi, ballTrackingValid, head TRACK,
   approachCompleted, yawStageCompleted, distanceStageCompleted, tidak jatuh, action player bebas, belum
   menendang, walking tidak diperintah, servo 19 sesuai kaki. Tracker "stop" -> kepala parkir (0°, servo 20
   200) -> tunggu sampai atau 1,5 s -> semua sendi ke action_module -> page. Selesai -> INIT -> cari 1,5 s,
   flag direset.
i) Bola hilang > 1 s saat bergerak -> INIT -> cari 3 s, flag TIDAK direset. Tidak ketemu -> SEARCH_BALL.
j) JATUH (prioritas tertinggi, fall_detector D56 pitch EMA α 0,4 ±60): walking stop, tracker stop, action
   jalan -> page_num -2, parkir kepala, page 85 (depan) / 86 (belakang) -> INIT -> cari 1,5 s, flag direset.
k) D60 OFF: walking stop, tracker stop + "scan_legacy", action jalan -> page_num -1 (diselesaikan), parkir ke
   pose INIT JMM -> STOPPED. Kepala diparkir pan 0 (50 %) / servo 20 135 (`auto_demo_stop_head_tilt_servo_deg`),
   jadi D60 ON berikutnya mulai dari posisi itu. Diam pertama (7 s sesudah INIT) kepala DITAHAN di (50 %, 135)
   (sapuan "scan_hold"); bola terlihat -> langsung dikejar, tidak -> JALAN BARU 20 s (`auto_demo_start_forward_sec`)
   sambil kepala turun ke 169 menyapu; bola terlihat -> dikejar, tidak -> MENCARI CELAH. Berlaku tiap D60 ditekan (sesi baru).
l) Kaki direbut base_module/none -> D60 berhenti.

6. KEPALA
- head_tracking_node punya 3 mode sapuan lewat /ball_tracker/command:
  - "scan_legacy": pola lama D56/D58;
  - "scan_stages" (D60 umum): pra-awal (50 %, 180) -> awal (50 %, 210) -> (70,207) (100,180) (100,135) (50,135)
    (50,207) (30,207) (0,180) (0,135) (50,135) (50,207) -> ulang;
  - "scan_walk" (D60 hanya di SEARCH_BALL): servo 20 tetap 169, servo 19 bolak-balik 0 <-> 100 %, mulai dari
    ujung terdekat.
  Semua mode: interpolasi linier kedua servo bersamaan, 30 °/s, tanpa jeda, dikirim sebagai offset 20 Hz
  (setpoint - goal dari /robotis/goal_joint_states). Bola terlihat -> tracking dari posisi saat itu (PID
  tracking tidak diubah). D60 kirim "scan_stages" sebelum start, "scan_walk" saat masuk SEARCH_BALL,
  "scan_stages" saat keluar SEARCH_BALL, "scan_legacy" saat D60 mati.
- head_control_module (semua demo):
  - head_tilt_min_deg -45 (servo 20 135);
  - amplop batas menunduk: maks 210 di 30-70 %, turun linier ke 180 di 0/100 %, dijepit pada TARGET semua
    perintah (sapuan dan tracking);
  - onModuleDisable dilindungi tra_lock_.
- Parkir kepala (pan 0°, servo 20 200) sebelum Action Page di D56, D58, dan D60.

7. CEK SEBELUM UJI
op3_ping semua servo menjawab (cek kabel ID19->20 dan rantai kaki); hanya SATU launch; ukur tanda head_tilt,
arah 4 preset geser/putar, batas pan, ambang servo 20, arah yaw; pastikan kepala bisa mendongak ke 135 tanpa
membentur dan kabel leher longgar di pan ±85.
````
