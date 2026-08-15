# OP3 (CHRONUS) — Handoff Lengkap untuk Claude di Robot Berikutnya

Dokumen ini merangkum SEMUA perubahan, perbaikan, gotcha, dan prosedur dari robot PERTAMA
(workspace ROS 2 Humble, OP3, Dockerized). Tujuan: di-replikasi ke robot OP3 berikutnya.

**Konteks robot 1:** ROBOTIS OP3 (20 servo XM430-W350 + OPEN-CR). Dijalankan via `./script.sh`
(tmux launcher) di dalam container (`/ros2_ws`, bind-mount ke host workspace). Dikontrol lewat web
**BASCORRO Studio** (Vite dev server :5173, roslib via rosbridge :9090). Servo via
`op3_manager` + `robotis_controller` di `/dev/ttyUSB0` @ 2 Mbps. INIT_BARU = action **page 2**.

---

## 📌 INSTRUKSI UNTUK CLAUDE BERIKUTNYA (baca paling awal)

Tugasmu: **terapkan logika/kode dari dokumen ini ke robot ini**, TAPI **JANGAN anggap semuanya sama
dengan robot 1**. Untuk tiap hal yang spesifik per-robot, **VERIFIKASI & SESUAIKAN dengan kondisi
robot ini dulu** — periksa nilai aslinya, jangan copy mentah dari dokumen.

**Yang WAJIB kamu sesuaikan/verifikasi (bukan copy):**
1. **Root workspace & path file** — temukan dengan `find` (lihat "BACA DULU"); path di sini relatif.
2. **Alamat rosbridge** `ws://IP:9090` di web (panel ROS BRIDGE) → sesuaikan IP robot ini.
3. **Port serial** — cek `ls -l /dev/ttyUSB*`; kalau bukan `ttyUSB0`, sesuaikan/symlink.
4. **`OP3_ACTION_FILE`** — cek `~/.config/op3-stack/prefs.sh`, pastikan menunjuk bin robot ini.
5. **`offset.yaml`** — JANGAN copy dari robot 1 (offset mekanik beda). Pakai/kalibrasi punya robot ini.
6. **INIT_BARU (page 2)** — **sudah ada di robot ini dengan nilai sendiri** → pakai apa adanya,
   JANGAN ditimpa. Kode capture bekerja dengan nilai apa pun. Cukup verifikasi `INIT_BARU_PAGE_NUM=2`.
7. **Tuning walking (§10)** — itu rekomendasi/titik-awal, bukan nilai final. Setel & uji di robot ini.

**Yang boleh di-copy/re-apply apa adanya:** logika kode C++ & JS (file #1–#7 di MANIFEST), prosedur
build, gotcha, mapping ID servo, dimensi OP3.

**⚠️ CEK DULU sebelum apply (kode robot 2 mirip tapi belum tentu identik / bisa jadi sudah ada
sebagian):** untuk tiap perubahan, grep penanda dulu — kalau **sudah ada**, SKIP; kalau **belum/
sebagian**, baru apply dan sesuaikan dengan struktur kode robot ini (jangan paksa tempel mentah).
```bash
grep -rn "capture_init_pose_\|walking_bias_\|computeNeutralLegAngle" src/   # §1 walking INIT_BARU
grep -rn "knee_cos\|alpha_sin"                                          src/   # §3 IK clamp
grep -rn "round(mov_time / smp_time + 1)"                              src/   # §2 fix SIGABRT
grep -rn "TORQUE_SPEED_PRESETS\|writeSyncItemsSequential\|SYNC_ITEM_GAP_MS" src/   # §5 soft Torque ON
grep -rn "ensureWalkingModuleEnabled\|WALKING_ENABLE_SETTLE_MS"        src/   # §4 auto-enable walking
grep -rn "showUploadConfirm\|binIsNewerThanDraft"                      src/   # §6 guard upload (ActionEditor)
grep -rn "def file_info\|action == \"stat\""                          src/   # §6 backend apply_node
```

**Mekanisme parameter walking & tab Walking di studio → `PANDUAN_PARAMETER_WALKING.md`.**
Dokumen itu memakai pola yang sama dengan halaman ini (grep penanda dulu, terapkan
yang belum ada) dan punya blok cek siap jalan di bagian 0. Per 2026-08-15 branch ini
sudah punya 6 dari 10; yang **belum** ada empat:

```bash
test -f src/bascorro_studio/web/src/walkingParams.js                      # parser dipakai bersama App+TuningPage
grep -rn "refreshWalkingCurrent\|sendBalanceCommand"  src/bascorro_studio/  # balance IMU sinkron + dibaca ulang
grep -rn "payload.period_time > 0"                    src/bascorro_studio/  # period_time 0 -> NaN, kaki beku
grep -rn "Bentuk Gait"                                src/bascorro_studio/  # grup Balance dipecah dari gait
```

Yang keempat penting untuk dipahami, bukan sekadar ditempel: `balance_enable` hanya
menggerbang `sensoryFeedback()`, yang menulis ke **8 sendi kaki saja**. `arm_swing_gain`
bukan parameter IMU — ayunan tangan datang dari `computeArmAngle()` dan tidak akan
berhenti walau balance dimatikan. Dulu semuanya satu grup bernama "Balance", dan itu
menyesatkan operator.

**Alur ringkas:** verifikasi path → **cek penanda (grep) untuk tiap fitur** → apply hanya yang belum
ada (sesuaikan dgn kode robot 2) → sesuaikan item per-robot 2–7 di atas → build di container + cek
timestamp → restart → uji (robot digantung dulu).

---

## ⚠️ BACA DULU — Path & perbedaan antar-robot (PENTING)

Dokumen ini dari robot pertama. Robot berikutnya **beda mesin**, jadi:

- **Path host berbeda** (home dir / nama workspace beda). Semua path di dokumen ini ditulis
  **relatif terhadap root workspace ROS 2** (folder berisi `src/` + `build/` + `install/`).
  Contoh `src/ROBOTIS-OP3/op3_walking_module/...` → gabungkan dengan root workspace robot itu.

- **Langkah pertama Claude berikutnya — temukan dulu lokasi nyata sebelum mengedit:**
  ```bash
  pwd; ls                       # apakah ini root workspace? (ada src/ build/ install/)
  find . -path '*op3_walking_module/src/op3_walking_module.cpp'
  find . -path '*op3_kinematics_dynamics/src/op3_kinematics_dynamics.cpp'
  find . -path '*bascorro_studio/web/src/App.jsx'
  find . -path '*bascorro_studio/bascorro_studio/apply_node.py'
  cat ~/.config/op3-stack/prefs.sh 2>/dev/null   # OP3_ACTION_FILE yang dipakai stack
  ls -l /dev/ttyUSB*                              # port serial aktual
  ```

- **Spesifik per-robot — JANGAN copy buta, RE-DISCOVER / KALIBRASI ulang:**
  - Alamat rosbridge `ws://IP:9090` (panel ROS BRIDGE di web).
  - `OP3_ACTION_FILE` & lokasi file `.bin`.
  - Port serial (`/dev/ttyUSB0` atau lain).
  - `offset.yaml` (offset mekanik tiap robot BEDA — ini kalibrasi, bukan kode).
  - Nilai servo pose INIT_BARU di page 2 (kalibrasi fisik tiap robot beda — robot lain **sudah punya
    INIT_BARU di page 2 dengan nilai sendiri**; JANGAN ditimpa, kode capture pakai apa adanya).
  - `INIT_BARU_PAGE_NUM` di App.jsx = **2** (konvensi, biasanya sama di semua robot — cukup verifikasi).

- **SAMA & bisa di-copy apa adanya:** logika/kode C++ & JS, struktur paket ROS (`src/ROBOTIS-OP3/...`),
  prosedur build, gotcha, mapping ID servo, dimensi OP3, rekomendasi tuning (titik awal).

---

## MANIFEST — 8 file yang diubah di robot 1 (path relatif ke `<ws_root>`)

| # | File | Perubahan | Detail di |
|---|---|---|---|
| 1 | `src/ROBOTIS-OP3/op3_walking_module/src/op3_walking_module.cpp` | capture/bias INIT_BARU; `round()` fix; finite-guard IK | §1, §2, §3 |
| 2 | `src/ROBOTIS-OP3/op3_walking_module/include/op3_walking_module/op3_walking_module.h` | anggota `captured_init_pose_`, `walking_bias_`, `capture_init_pose_` + method `computeNeutralLegAngle()` | §1 |
| 3 | `src/ROBOTIS-OP3/op3_kinematics_dynamics/src/op3_kinematics_dynamics.cpp` | clamp `acos`/`asin` (anti NaN / kaki-beku) | §3 |
| 4 | `src/bascorro_studio/web/src/App.jsx` | auto-enable walking; soft Torque ON (stagger) + preset kecepatan | §4, §5 |
| 5 | `src/bascorro_studio/web/src/ActionEditor.jsx` | dialog konfirmasi upload + info/staleness bin | §6 |
| 6 | `src/bascorro_studio/bascorro_studio/apply_node.py` | `file_info()` + action `stat` (python — restart node) | §6 |
| 7 | `src/ROBOTIS-OP3-Tools/op3_action_editor/scripts/executor.py` | default action file = salinan src | §7 |
| 8 | `src/ROBOTIS-OP3/op3_action_module/data/motion_4095_CHRONUS.bin` | **DATA, bukan kode** — berisi INIT_BARU (page 2) dll. **Spesifik tiap robot, JANGAN copy bin robot 1.** Robot lain biasanya **sudah punya INIT_BARU di page 2 dengan nilai sendiri** → pakai apa adanya, JANGAN ditimpa. Kode capture bekerja dengan nilai page 2 berapa pun. | §6, §7 |

> File #1–#7 = kode/logika (di-copy / re-apply). File #8 = data kalibrasi (buat sendiri per-robot).

---

## 0. ATURAN EMAS (paling sering bikin masalah)

1. **Build HARUS di dalam container** (`cd <ws_root> && colcon build …`). Host `install/`+`build/`
   biasanya root-owned; `sudo` host minta password.
2. **Setelah ubah C++, WAJIB rebuild + verifikasi timestamp `.so` lebih baru dari source, lalu
   restart stack.** User sering lupa → terlihat seperti "perubahan tidak ngefek".
3. **Ubah layout class `WalkingModule` (header) → `op3_manager` WAJIB ikut rebuild** (Singleton
   `getInstance()` pakai inline `new T`).
4. **Action editor (`executor_py`) & `op3_manager` tidak boleh jalan bersamaan** — rebutan
   `/dev/ttyUSB0`. Pakai SATU saja.
5. Saat Torque OFF robot lemas; perintah web (Init Pose/walking) butuh `op3_manager` HIDUP.

---

## 1. FITUR: Berjalan dalam pose INIT_BARU (kunci stance)

**Mekanik:** saat walking module di-*enable*, capture pose robot SAAT ITU sebagai stance netral;
gait berayun di sekitarnya. **Dinamis — bekerja dengan nilai INIT_BARU APA PUN** (tidak hard-coded),
jadi robot lain cukup pakai page 2 yang sudah ada, tanpa mengubah kode.
- idle → `goal = captured_init_pose_` (tahan INIT_BARU, tanpa lompat)
- walk → `goal = walking_bias_ + IK_angle + balance`, `walking_bias_ = captured − neutral_IK`

**File:** `src/ROBOTIS-OP3/op3_walking_module/src/op3_walking_module.cpp` (+`.h`)

Header (private, setelah `init_position_`):
```cpp
Eigen::MatrixXd captured_init_pose_;
Eigen::MatrixXd walking_bias_;
bool capture_init_pose_;
bool computeNeutralLegAngle(double *neutral);   // dekat deklarasi iniPoseTraGene
```
Constructor: init Zero / false. `onModuleEnable()`: `capture_init_pose_ = true;`

`process()` — blok capture (setelah loop baca dxl `goal_position_`, sebelum `bool get_angle`):
```cpp
if (capture_init_pose_ == true) {
  captured_init_pose_ = goal_position_;
  walking_bias_ = goal_position_;
  double neutral_leg[12];
  if (computeNeutralLegAngle(neutral_leg) == true)
    for (int i=0;i<12;i++) walking_bias_.coeffRef(0,i) = goal_position_.coeff(0,i) - neutral_leg[i];
  capture_init_pose_ = false;
}
```
`process()` — goal loop:
```cpp
if (walking_idle == true)              goal_position = captured_init_pose_.coeff(0, idx);
else if (get_angle == false && idx<12) goal_position = goal_position_.coeff(0, idx);
else                                   goal_position = walking_bias_.coeff(0,idx) + angle[idx] + balance_angle[idx];
```
Method `computeNeutralLegAngle` (sebelum `iniPoseTraGene`): IK pada zero swap/move dgn offset;
`ep[0..5]`=kanan, `ep[6..11]`=kiri, `ep[2]=ep[8]=z_offset_-leg_length`; lalu kurangi
`hit_pitch_offset_` pada r/l_hip_pitch (samakan dengan computeLegAngle).

**Trade-off (DITERIMA user):** slider Init Pose offset (x/y/z/roll/pitch/yaw) **tidak** menggeser
postur berdiri (stance selalu INIT_BARU). Ubah postur dasar lewat edit INIT_BARU di action editor.
JANGAN buat offset menggeser stance — sudah dicoba & ditolak.

**Urutan pakai (capture saat enable):** Torque ON → **Init Pose** (tunggu sampai di INIT_BARU) →
**Enable Walking Module** (detik ini capture) → **Start**. Kalau enable saat bukan di INIT_BARU,
yang terkunci pose itu.

---

## 2. FIX: op3_manager SIGABRT saat enable walking (Eigen resize)

`iniPoseTraGene()` di `op3_walking_module.cpp`:
```cpp
int all_time_steps = round(mov_time / smp_time + 1);   // BUKAN int(mov_time/smp_time)+1
```
Agar jumlah baris cocok dengan `calcMinimumJerkTra()`; kalau tidak → Eigen assertion → op3_manager
mati saat walking enable.

---

## 3. FIX: Kaki beku saat jalan (IK NaN), lengan tetap ayun

Gejala: set `x_move_amplitude`, **tangan ayun tapi kaki diam, tanpa error**. Sebab:
`calcInverseKinematicsForLeg` di `src/ROBOTIS-OP3/op3_kinematics_dynamics/src/op3_kinematics_dynamics.cpp`
tidak clamp `acos`/`asin` → target di luar jangkauan = NaN → goal NaN → controller buang → kaki beku.

```cpp
double p36_norm = p36.norm(); if (p36_norm < 1e-6) p36_norm = 1e-6;
double knee_cos = (thigh_length_m_*thigh_length_m_ + calf_length_m_*calf_length_m_ - p36_norm*p36_norm)
                  / (2*thigh_length_m_*calf_length_m_);
if (knee_cos>1.0) knee_cos=1.0; else if (knee_cos<-1.0) knee_cos=-1.0;
*(out+3) = -acos(knee_cos) + EIGEN_PI;
double alpha_sin = thigh_length_m_*sin(EIGEN_PI - *(out+3))/p36_norm;
if (alpha_sin>1.0) alpha_sin=1.0; else if (alpha_sin<-1.0) alpha_sin=-1.0;
double alpha = asin(alpha_sin);
```
Plus jaring pengaman di `computeLegAngle` (op3_walking_module): setelah IK, cek `std::isfinite`
pada leg_angle[0..11]; kalau NaN → `RCLCPP_ERROR_THROTTLE(...)` + `return false`.

Dimensi OP3: paha 0.110 m, betis 0.110 m, ankle 0.0305 m → total ~0.2505; jangkauan lutut ~0.220;
jarak antar pinggul ~0.07.

---

## 4. WEB (App.jsx): walking tak respon Start → auto-enable module

`op3_walking_module` mengabaikan start/stop/balance kecuali ia control module aktif. Di
`src/bascorro_studio/web/src/App.jsx`:
- `ensureWalkingModuleEnabled()` publish `walking_module` ke `/robotis/enable_ctrl_module`.
- `WALKING_ENABLE_SETTLE_MS = 1200` sebelum kirim "start". `walkingModuleEnabledRef`.
- `startWalking()`, `applyWalkingAndStart()`. Reset flag saat Init Pose / Enable Head.

---

## 5. WEB+CONTROLLER: Soft Torque ON (anti nyentak) + BUG sync_write_item

**BUG `robotis_controller`** (`syncWriteItemCallback`): semua item satu port digabung ke 1
`GroupSyncWrite` per siklus (~8ms), dicocokkan **hanya per port** (bukan alamat register) → 2 item
berbeda dalam 1 siklus → item ke-2 **dibuang diam-diam**.

**Solusi (App.jsx, web-only):** kirim tiap item **satu per siklus** (di-stagger).
```js
const SYNC_ITEM_GAP_MS = 60;
const TORQUE_SPEED_PRESETS = [
  { key:"slow",   label:"Lambat", velocity:15, accel:6,  restoreMs:5000 },
  { key:"medium", label:"Sedang", velocity:35, accel:12, restoreMs:3000 },
  { key:"fast",   label:"Cepat",  velocity:70, accel:25, restoreMs:2000 },
];
```
- `writeSyncItem(item,value)`, `writeSyncItemsSequential(items,onDone)`, `restoreFullSpeed()`.
- `handleTorque(enable)`: `profile_velocity → profile_acceleration → torque_enable` (staggered),
  lalu restore `profile_velocity=0` setelah `restoreMs`.
- XM430 reg: `profile_velocity`(112), `profile_acceleration`(108), unit ~0.229 rev/min, 0=tak terbatas,
  operating_mode=3 (Position), drive mode default velocity-based.
- `restoreFullSpeed()` juga dipanggil di `ensureWalkingModuleEnabled()` (biar gait tak melambat).
- UI: pemilih kecepatan (Lambat/Sedang/Cepat) di Dashboard, state `torqueSpeedKey`.

> Bug controller bisa diperbaiki permanen (key per port+alamat) tapi butuh rebuild framework.
> Workaround stagger sudah cukup.

---

## 6. WEB+BACKEND: Action editor "upload" menimpa bin (DATA LOSS) + guard

**Bahaya:** tombol upload (↑) di YAML panel Action editor menulis ulang `.bin` dari **draft browser
(localStorage)** yang bisa BASI (lihat `generated_at`) → INIT_BARU & banyak page rusak.

**Recovery:** stack baca **src** (`OP3_ACTION_FILE`), upload hanya sentuh src → salinan **install**
(`install/op3_action_module/share/op3_action_module/data/<bin>`) jadi snapshot pemulihan. Decode bin:
256 page × 512 B; page N @ `N*512`; nama=`bytes[0:14]`; step0 pos = `struct.unpack_from('<31H', d, 64)`
(idx = joint id 1..20). Splice per-page (512 B) atau full copy. Contoh pulih 1 page:
```python
inst=open(INSTALL,'rb').read(); b=bytearray(open(SRC,'rb').read())
n=2; b[n*512:(n+1)*512]=inst[n*512:(n+1)*512]; open(SRC,'wb').write(b)
```

**Guard yang sudah dibuat (2 file):**
- `src/bascorro_studio/bascorro_studio/apply_node.py`: `file_info()` (path+mtime+mtime_iso)
  dilampirkan ke **setiap** `publish_result`; action baru `"stat"`.
- `src/bascorro_studio/web/src/ActionEditor.jsx`: header tampilkan target bin + waktu diubah + waktu
  draft; badge **"⚠ Draft mungkin basi"**; tombol upload → **dialog konfirmasi** (target, waktu,
  peringatan kuning/hijau/abu, tombol Batal/Muat ulang/Timpa). State: `binInfo`, `showUploadConfirm`,
  `lastSyncMs`; kirim `{action:"stat"}` saat mount; `handleApply`→buka modal, `confirmApply`,
  `handleReloadFromBin`; derived `binIsNewerThanDraft`.

**Hapus draft basi di browser:** tekan tombol **Download/"Muat ulang dari bin" (↓)** sekali (refresh
saja TIDAK cukup — refresh memuat ulang draft basi).

---

## 7. PATH bin (editor & stack baca bin sama)

- `src/ROBOTIS-OP3-Tools/op3_action_editor/scripts/executor.py` di-patch:
  `resolve_action_file_default()` memilih salinan **src** `…/op3_action_module/data/<bin>`.
- Stack baca `OP3_ACTION_FILE` (di-set `script.sh`, persisted di `~/.config/op3-stack/prefs.sh`).
- INIT_BARU = **action page 2**; web "Init Pose" memutar page 2.
- `op3_action_module` memuat bin **sekali saat launch** → restart stack untuk reload bin yang diubah.

---

## 8. GOTCHA: Port serial /dev/ttyUSB0

`op3_manager` membuka `/dev/ttyUSB0` (hardcoded di `src/ROBOTIS-OP3/op3_manager/config/OP3.robot`,
semua baris dynamixel + OPEN-CR). Kalau U2D2 re-enumerate jadi `ttyUSB1` → "PORT SETUP ERROR / Error
opening serial port" → op3_manager mati (exit 255).
- Fix: cabut-colok USB (biar balik ke ttyUSB0) atau `ln -sf /dev/ttyUSB1 /dev/ttyUSB0`.
- Bersihkan sisa proses: `pkill -f op3_manager; pkill -f op3_action_editor; pkill -f executor;
  pkill -f op3_offset_tuner; pkill -f bridge_webots`.

---

## 9. PERILAKU NORMAL (bukan bug) — sendi saat jalan

Mapping ID (OP3.robot): 1 r_sho_pitch · 2 l_sho_pitch · 3 r_sho_roll · 4 l_sho_roll · 5 r_el ·
6 l_el · 7 r_hip_yaw · 8 l_hip_yaw · **9 r_hip_roll · 10 l_hip_roll** · **11 r_hip_pitch · 12 l_hip_pitch** ·
13 r_knee · 14 l_knee · 15 r_ank_pitch · 16 l_ank_pitch · 17 r_ank_roll · 18 l_ank_roll · 19 head_pan · 20 head_tilt.

- **hip_roll (9,10) goyang kiri-kanan saat jalan = NORMAL** (transfer berat lateral; kalau diam
  jatuh samping). Diatur `y_swap_amplitude`/`pelvis_offset`/`balance_hip_roll_gain`.
- **hip_pitch (11,12) ayun = sendi langkah utama**, tak bisa "diam".
- Jangan nilai goyangan saat robot **digantung** (terlihat sia-sia, padahal di lantai itu menjaga keseimbangan).

---

## 10. TUNING walking (satuan runtime: detik/meter/radian)

> Sebelum menyetel: baca `PANDUAN_PARAMETER_WALKING.md`. Di situ ada peta empat
> tempat parameter walking hidup (termasuk **tiga salinan angka** yang diam-diam
> bisa berbeda: `param.yaml`, fallback `initialize()`, dan `WALKING_DEFAULT_PARAMS`),
> peta nama/satuan yaml ↔ pesan ↔ UI, dan delapan jebakan mekanisme yang membuat
> nilai yang diketik tidak sampai ke robot. Menyetel angka sebelum mekanismenya
> benar hanya menghasilkan hasil yang tidak bisa diulang.

OP3 standar → param.yaml resmi sudah baik. Paling berdampak: **perbesar y_swap**.

| Grup | Param | Rekomendasi | Default repo |
|---|---|---|---|
| Timing | period_time | 0.78 (uji 0.82) | 0.78 |
| | dsp_ratio | 0.30 (0.34 ekstra stabil) | 0.30 |
| | step_fb_ratio | 0.25 | 0.25 |
| Movement | z_move (foot height) | 0.033 (0.040 rumput) | 0.033 |
| | x_move_amplitude | ramp 0.010→0.025–0.030 | 0 |
| | y_move / angle_move | 0 | 0 |
| Balance | balance_enable | ON di hardware | — |
| | hip_roll/knee/ank_roll/ank_pitch gain | 0.35/0.40/0.70/0.90 | sama |
| | **y_swap_amplitude** | **0.018–0.022** (paling penting) | 0.002 |
| | z_swap_amplitude | 0.010–0.015 | 0.006 |
| | pelvis_offset | ~0.045 rad (2.5°) | 0.0087 |
| | arm_swing_gain | 0.3–0.5 | 0.2 |

Urutan tuning: samakan z_offset/pitch/hip_pitch dgn INIT_BARU → balance ON → jalan di tempat (setel
y_swap & z_swap sampai transfer berat mulus, kaki tak nyeret) → naikkan x_move bertahap → finishing.
Gejala→fix: jatuh depan=kurangi x_move/pitch; nyeret=naikkan y_swap/foot_height; memantul=turunkan
z_swap/knee_gain; pinggul getar=turunkan hip_roll_gain.

---

## 11. PROSEDUR BUILD & TEST (template, DI CONTAINER)

```bash
cd <ws_root>      # root workspace (ada src/ build/ install/) — di robot 1 = /ros2_ws
colcon build --packages-select op3_kinematics_dynamics op3_walking_module op3_manager
# perubahan python web → tambah: bascorro_studio
ls -la --time-style=+%H:%M:%S \
  build/op3_walking_module/libop3_walking_module.so \
  build/op3_manager/op3_manager \
  build/op3_kinematics_dynamics/libop3_kinematics_dynamics.so
# pastikan timestamp == sekarang, lalu RESTART stack (./script.sh)
```
Web (App.jsx / ActionEditor.jsx) = Vite HMR (refresh halaman). `apply_node.py` (python) = restart
node / rebuild `bascorro_studio`.

**Test walking aman:** robot digantung → Torque ON (Lambat) → Init Pose → Enable Walking → Start
`x_move=0` (march di tempat dalam postur INIT_BARU) → naikkan x_move bertahap.

---

## 12. CHECKLIST untuk ROBOT BERIKUTNYA

- [ ] Temukan root workspace + file aktual (lihat bagian "BACA DULU" di atas).
- [ ] Re-apply 6 file (path relatif terhadap `src/`):
  1. `ROBOTIS-OP3/op3_walking_module/src/op3_walking_module.cpp` (+`.h`): capture/bias INIT_BARU, round() fix, finite-guard.
  2. `ROBOTIS-OP3/op3_kinematics_dynamics/src/op3_kinematics_dynamics.cpp`: clamp acos/asin.
  3. `bascorro_studio/web/src/App.jsx`: auto-enable walking, soft Torque ON (stagger), speed preset.
  4. `bascorro_studio/web/src/ActionEditor.jsx`: dialog konfirmasi upload + info bin.
  5. `bascorro_studio/bascorro_studio/apply_node.py`: file_info + action "stat".
  6. `ROBOTIS-OP3-Tools/op3_action_editor/scripts/executor.py`: default action file = src copy.
- [ ] INIT_BARU **sudah ada di page 2** (nilai sudah disesuaikan robot ini) → **JANGAN buat/timpa.**
      Pakai apa adanya; mekanik capture bekerja dengan nilai apa pun. Cukup pastikan `INIT_BARU_PAGE_NUM = 2` di App.jsx.
- [ ] Sinkronkan bin src↔install + simpan 1 backup bersih sebagai snapshot recovery.
- [ ] `OP3.robot` & `OP3_ACTION_FILE` & port serial benar untuk robot ini.
- [ ] **Kalibrasi `offset.yaml` & nilai servo INIT_BARU sendiri** — spesifik per-robot, JANGAN copy
      buta dari robot 1.
- [ ] Set alamat rosbridge `ws://IP:9090` di web (panel ROS BRIDGE) sesuai robot ini.
- [ ] Build (container) + verifikasi timestamp + restart.
- [ ] Tuning walking per §10 (utamakan y_swap), uji digantung dulu.

> RINGKAS: kode/logika & prosedur = SAMA (copy). Angka kalibrasi (offset, INIT_BARU, IP, port,
> path) = SPESIFIK per-robot (re-discover / set ulang).
