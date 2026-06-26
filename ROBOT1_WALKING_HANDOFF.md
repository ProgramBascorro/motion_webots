# Handoff: Walking & Demo Refactor → Robot 1

**Tujuan**: Replikasi perubahan walking + soccer demo dari Robot 0 ke Robot 1.

**Untuk Claude / engineer di Robot 1**:
- Baca dulu seluruh dokumen ini.
- Setiap section punya **VERIFY** step — cek dulu apakah perubahan sudah ada di Robot 1 (mungkin via git merge). Kalau sudah, skip section itu.
- **JANGAN** langsung implement tanpa cek. User wants idempotent application.

## ⚠️ PATH ADJUSTMENT (BACA DULU)

Path di dokumen ini pakai **Robot 0** convention:
- Host: `/home/alphonse/motion_webots/`
- Container: `/ros2_ws/` (mounted dari host)
- User: `alphonse`

**Robot 1 mungkin BEDA**:
- Username bisa beda (`root`, `op3`, `pi`, dll)
- Repo mungkin di-clone ke path lain (`/home/op3/op3_workspace/`, `/opt/ros2_ws/src/`, dll)
- Container path mapping bisa beda (atau bahkan ga pakai container)

**Action**: Sebelum jalanin VERIFY/grep command, cari dulu repo root Robot 1 dengan:
```bash
find / -name "op3_walking_module" -type d 2>/dev/null | head -3
# Atau cari berdasar package.xml:
find / -name "package.xml" -path "*op3_walking_module*" 2>/dev/null | head -1
```

Lalu **substitute** path-nya di semua command. Contoh transformasi:
- `/home/alphonse/motion_webots/` → `<REPO_ROOT_ROBOT1>/`
- `/ros2_ws/` → `<WORKSPACE_ROOT_ROBOT1>/` (kalau ada container) atau sama dengan `<REPO_ROOT_ROBOT1>` (kalau direct)

Atau set environment variable di awal:
```bash
export ROBOT1_REPO="/path/to/robot1/repo"
# Lalu pakai: $ROBOT1_REPO/src/ROBOTIS-OP3/...
```

---

## A. KOMPONEN CUSTOM YANG MUNGKIN BELUM ADA DI ROBOT 1

Ini bukan dari upstream ROBOTIS — Robot 0 ada custom node/package yang harus dicek dulu apakah Robot 1 juga punya. Kalau **tidak ada**, harus ditambahkan supaya soccer demo berfungsi penuh (terutama head SCAN waktu nungguin bola + YOLO bridge).

### VERIFY semua custom components:
```bash
# Subtitusi $ROOT dengan repo root Robot 1
ROOT="/path/to/robot1/repo"   # ← ganti ini

ls $ROOT/src/op3_ball_localization/ 2>&1
ls $ROOT/src/op3_yolo_vision/ 2>&1
ls $ROOT/src/bascorro_studio/ 2>&1
```

Kalau ketiga directory ini exist → semua custom komponen ada. Skip section ini.

Kalau **tidak ada**, lakukan section sub di bawah untuk yang missing.

---

### A.1. `head_tracking_node.py` (di package `op3_ball_localization`)

**Fungsi**: Robot SCAN kepala kiri-kanan-bawah saat bola ga ke-detect. Track bola pakai PID saat bola ke-detect. Aktif/non-aktif via `/ball_tracker/command` ("start"/"stop").

**File**: `<ROOT>/src/op3_ball_localization/op3_ball_localization/head_tracking_node.py`

**Verify exist**:
```bash
ls $ROOT/src/op3_ball_localization/op3_ball_localization/head_tracking_node.py
```

**Kalau ga ada**: copy file dari Robot 0 (369 lines). Pakai `scp` dari Robot 0 atau git pull.

**Topic yang dipake**:
- Subscribe: `/vision/yolo/ball_center` (Point) — ball position dari YOLO
- Subscribe: `/ball_tracker/command` (String) — "start"/"stop" command dari soccer_demo
- Publish: `/robotis/head_control/set_joint_states` (JointState) — target head_pan + head_tilt

**Config**: `<ROOT>/src/op3_ball_localization/config/head_tracking.yaml`
```yaml
head_tracking_node:
  ros__parameters:
    scan_enabled: True
    scan_period_sec: 2.5
    scan_pan_rad: 0.7        # 40° kiri-kanan
    scan_tilt_forward_rad: -0.10
    scan_tilt_down_rad: -0.35
    pan_p_gain: 0.3
    pan_i_gain: 0.0
    pan_d_gain: 0.045
    tilt_p_gain: 0.3
    tilt_i_gain: 0.0
    tilt_d_gain: 0.045
```

**Behavior summary** (penting buat next Claude paham logika):
- `auto_start: false` (default param) → node start dalam mode "paused", ga ngirim command apapun sampai dapet "start" command
- Saat "start" diterima dari `/ball_tracker/command`:
  - Kalau bola ke-detect: PID track ke center frame
  - Kalau bola hilang > 1 second: scan mode aktif, kepala goyang antar 5 preset position (per scan_period_sec)
- Saat "stop" diterima: deactivate, kepala statis

---

### A.2. `yolo_to_demo_bridge.py` (di package `op3_ball_localization`)

**Fungsi**: Convert YOLO detection format (`soccer_msgs/BoundingBoxes`) ke format yang dipake `op3_demo`'s `BallTracker` (`op3_ball_detector_msgs/CircleSetStamped`). Tanpa bridge ini, soccer_demo ga dapat ball position.

**File**: `<ROOT>/src/op3_ball_localization/op3_ball_localization/yolo_to_demo_bridge.py`

**Verify exist**:
```bash
ls $ROOT/src/op3_ball_localization/op3_ball_localization/yolo_to_demo_bridge.py
```

**Topic yang dipake**:
- Subscribe: `/vision/yolo/detections` (BoundingBoxes)
- Publish: `/ball_detector_node/circle_set` (CircleSetStamped) — format lama yang soccer_demo expect
- Publish: `/vision/yolo/ball_center` (Point) — buat head_tracking_node

**Kalau ga ada**: copy file dari Robot 0 (~130 lines). Pastikan msg dependencies (`soccer_msgs`, `op3_ball_detector_msgs`) juga di-install.

---

### A.3. `op3_yolo_vision` package

**Fungsi**: YOLO detector — load model `.onnx`, deteksi bola dari `/usb_cam/image_raw`, publish ke `/vision/yolo/detections`.

**Verify exist**:
```bash
ls $ROOT/src/op3_yolo_vision/
ls $ROOT/src/op3_yolo_vision/launch/yolo.launch.py
ls $ROOT/src/op3_yolo_vision/models/*.onnx
```

**Kalau ga ada**:
1. Copy entire `op3_yolo_vision/` directory dari Robot 0
2. Copy model `models/yolo.onnx` (binary file, ~10MB)
3. Install OpenVINO runtime kalau belum:
   ```bash
   pip install openvino openvino-dev
   ```
4. Build:
   ```bash
   cd <workspace_root>
   colcon build --packages-select op3_yolo_vision --symlink-install
   ```

**Topic yang dipake**:
- Subscribe: `/usb_cam_node/image_raw` (Image) — input camera
- Publish: `/vision/yolo/detections` (BoundingBoxes)

**Dependencies**:
- `soccer_msgs` package (BoundingBoxes msg type)
- USB cam node aktif: `usb_cam/usb_cam_node` publishing `/usb_cam_node/image_raw`

---

### A.4. `op3_ball_detector_msgs` package

**Fungsi**: Custom msg types (`CircleSet`, `CircleSetStamped`) untuk format ball detection lama yang soccer_demo expect.

**Verify exist**:
```bash
find $ROOT/src -name "circle_set*.msg" 2>&1
```

**Kalau ga ada**: copy `op3_ball_detector_msgs/` package dari Robot 0.

---

### A.5. `bascorro_studio` package (UI web + apply_node)

**Fungsi**: Web UI buat tune walking params, action editor, sim preview. Plus Python backend `apply_node` yang spawn `op3_manager_sim` untuk simulasi.

**Verify exist**:
```bash
ls $ROOT/src/bascorro_studio/
ls $ROOT/src/bascorro_studio/web/src/App.jsx
ls $ROOT/src/bascorro_studio/bascorro_studio/apply_node.py
```

**Kalau ga ada**: copy entire `bascorro_studio/` directory. Plus:
1. Install Node deps untuk web UI:
   ```bash
   cd $ROOT/src/bascorro_studio/web
   npm install
   ```
2. Build webapp:
   ```bash
   npm run build
   ```
3. Untuk dev (live reload):
   ```bash
   npm run dev
   # → http://localhost:5173
   ```

**Kalau Robot 1 ga punya display/browser** (headless): Bascorro UI bisa diakses dari laptop client. ROS bridge (`rosbridge_websocket`) running di Robot 1 di port 9090, browser di laptop konek ke `ws://<robot1_ip>:9090`.

---

### A.6. `op3_camera_setting_tool` (mungkin sudah ada upstream)

**Fungsi**: Tune camera params (brightness, exposure, white balance) via v4l2-ctl. Launch di `demo_yolo.launch.xml`.

**Verify exist**:
```bash
ls $ROOT/src/op3_camera_setting_tool/
```

**Kalau ga ada**: bukan bagian upstream ROBOTIS — copy dari Robot 0.

---

### A.7. `usb_cam` package (mungkin sudah dari apt)

**Verify**:
```bash
ros2 pkg list | grep usb_cam
```

**Kalau ga ada**:
```bash
apt install ros-humble-usb-cam
```

---

### A.8. Patched `DynamixelSDK` (rxpacket buffer fix)

**Fungsi**: User memperbaiki bug rxpacket buffer overflow di Protocol1/2 handler.

**Verify**:
```bash
grep "RXPACKET_MAX_LEN" $ROOT/src/DynamixelSDK/dynamixel_sdk/src/dynamixel_sdk/protocol2_packet_handler.cpp | head -3
```

Kalau hasil ada minimal 3 baris dengan `RXPACKET_MAX_LEN` di `reboot`, `clearMultiTurn`, `factoryReset`, `writeTxRx`, `regWriteTxRx` → fix sudah ada.

**Kalau ga ada**: copy file `protocol1_packet_handler.cpp` + `protocol2_packet_handler.cpp` dari Robot 0. Atau apply diff:
```diff
-  uint8_t rxpacket[11]        = {0};
+  uint8_t rxpacket[RXPACKET_MAX_LEN] = {0};
```
di setiap function yang punya `rxpacket[11]`.

---

### A.9. Patched `op3_kinematics_dynamics.cpp` (acos/asin clamp)

**Fungsi**: Clamp acos/asin args ke [-1, 1] di leg IK supaya ga produce NaN saat target di luar reach.

**Verify**:
```bash
grep "knee_cos\|alpha_sin\|clamp" $ROOT/src/ROBOTIS-OP3/op3_kinematics_dynamics/src/op3_kinematics_dynamics.cpp | head -5
```

Kalau ada line dengan `knee_cos` atau `alpha_sin` → fix sudah ada.

**Kalau ga ada**: lihat patch di Robot 0 sekitar line 1002-1024. Wrap acos/asin di leg IK dengan clamp.

---

## 0. KESALAHAN YANG SUDAH TERJADI (jangan ulangi)

### ❌ Memaksa INIT_BARU sebelum walking_module enable
**Konteks dulu**: Bascorro `initThenEnableWalking` dan demo path memaksa robot ke action_module + page 2 (INIT_BARU) sebelum switch ke walking_module.

**Kenapa salah**:
- 4 detik animasi action page 2 + jerky module handoff
- Walking_module IK sebenarnya bisa langsung produce calibrated standing pose dari `init_x/y/z_offset` (kalau di-tune)
- Double init bikin user experience lambat & ga smooth

**Aturan baru**:
- **Boot**: action_module + page 2 (INIT_BARU) — robot ke calibrated standing
- **Tombol START**: walking_module DIRECT (no page 2 replay)
- `init_x/y/z_offset` di walking_module **harus FK-tuned** match INIT_BARU geometry — supaya walking-ready pose mirip INIT_BARU → smooth transition

### ❌ Setting walking params ke nol di Bascorro UI lalu Save
**Gejala**: kaki seret / lompat bareng / robot ga maju (cuma goyang hip_roll ID 9/10)

**Akarnya**: `Save to Robot` di Bascorro nulis nilai UI ke yaml. Kalau UI punya `foot_height=0`, yaml jadi `0`, walking_module patuh, gait engine ga produce langkah.

**Pencegahan**: Sebelum Save, verify CURRENT column di Bascorro UI matches nilai bagus yang udah di-set di table di section 6 dokumen ini.

### ❌ Menambah publisher-creation code di op3_manager.cpp auto-init
**Gejala**: `*** stack smashing detected ***` saat boot — exit code -6

**Akarnya**: Ada code yang bikin publishers + sleep + publish di tengah inisialisasi op3_manager. Stack canary corruption.

**Aturan**: JANGAN tambah init logic kompleks di `op3_manager.cpp`. Keep init di `demo_node` (proses terpisah).

---

## 1. WALKING PARAM (op3_walking_module/config/param.yaml)

### VERIFY:
```bash
cat /home/alphonse/motion_webots/src/ROBOTIS-OP3/op3_walking_module/config/param.yaml
```

### EXPECTED CONTENT:
```yaml
x_offset: 0.003
y_offset: 0.032
z_offset: 0.038
roll_offset: 0
pitch_offset: 4.0
yaw_offset: 0
hip_pitch_offset: 8.0
period_time: 750
dsp_ratio: 0.35
step_forward_back_ratio: 0.25
foot_height: 0.040
swing_right_left: 0.020
swing_top_down: 0.006
pelvis_offset: 3.0
arm_swing_gain: 1.5
balance_hip_roll_gain: 0.35
balance_knee_gain: 0.4
balance_ankle_roll_gain: 0.7
balance_ankle_pitch_gain: 0.9
p_gain: 32
i_gain: 0
d_gain: 0
```

### KENAPA NILAI INI:
- `init_x/y/z_offset` = FK-derived dari INIT_BARU page 2 Robot 0. Diturunkan via `scripts/init_baru_fk.py` (lihat section 2).
- `pitch_offset = 4°` — proven value, body lean forward kompensasi berat kepala+kamera.
- `hip_pitch_offset = 8°` — juga dipake `ball_follower` untuk kalkulasi jarak bola.
- `period_time = 750` (ms) — comfortable cadence.
- `dsp_ratio = 0.35` — 35% double-support biar stabil.
- `foot_height = 0.04` — angkat kaki 4cm, clear lantai.
- `swing_right_left = 0.02` — body sway 2cm, weight transfer.
- `arm_swing_gain = 1.5` — counter-balance.
- `pelvis_offset = 3°` — tilt pelvis.
- `balance_*_gain` — IMU feedback gains (low-to-mid).
- `p_gain = 32` — XM430 standard PID position.

### CATATAN ROBOT 1:
Kalau Robot 1 punya INIT_BARU yang **beda** dari Robot 0 (joint values di page 2 bin file beda), maka `init_x/y/z_offset` perlu di-derive ulang. Lihat section 2.

---

## 2. FK DERIVATION — Perhitungan x/y/z offset dari INIT_BARU page 2

### VERIFY:
```bash
ls /home/alphonse/motion_webots/scripts/init_baru_fk.py
```

### A. INPUT — INIT_BARU page 2 joint values (Robot 0)

Diekstrak dari `op3_action_module/data/action_25_febuari_jam11Malem.yaml` page index 2 step 0 (= INIT_BARU pose yang user record di action editor):

| Joint | Raw dxl | Sudut (°) | Joint | Raw dxl | Sudut (°) |
|-------|---------|-----------|-------|---------|-----------|
| r_hip_yaw   | 2045 | -0.26  | l_hip_yaw   | 2047 | -0.09 |
| r_hip_roll  | 2045 | -0.26  | l_hip_roll  | 2010 | -3.34 |
| r_hip_pitch | 2471 | +37.18 | l_hip_pitch | 1612 | -38.32 |
| r_knee      | 1371 | -59.50 | l_knee      | 2724 | +59.41 |
| r_ank_pitch | 1727 | -28.21 | l_ank_pitch | 2377 | +28.92 |
| r_ank_roll  | 2061 | +1.14  | l_ank_roll  | 2070 | +1.93 |

Konversi raw→rad: `angle = (raw - 2048) * 2π/4096`. Catatan: sisi kanan-kiri punya tanda mirror (OP3 convention).

### B. PARAMETER GEOMETRI OP3 (spec sheet)

| Konstanta | Nilai (m) | Apa |
|-----------|-----------|-----|
| `THIGH`   | 0.093     | Hip pitch → knee axis distance |
| `CALF`    | 0.093     | Knee → ankle pitch axis distance |
| `ANKLE`   | 0.0335    | Ankle roll axis → foot sole |
| `HIP_Y`   | 0.037     | Half hip width (right/left hip axis ke center body) |
| `LEG_LENGTH` | **0.2195** | THIGH + CALF + ANKLE (full extension) |

### C. FK CHAIN (right leg, mirror buat left)

Transformasi homogen dari pusat hip ke telapak kaki:

```
T_foot = T_hip_origin × Rz(hip_yaw) × Rx(hip_roll) × Ry(hip_pitch)
       × Tz(-THIGH) × Ry(knee)
       × Tz(-CALF) × Ry(ank_pitch) × Rx(ank_roll)
       × Tz(-ANKLE)
```

Posisi foot relatif hip = origin transformasi T_foot.

Untuk right leg, OP3 punya inversi tanda pada pitch joints (mirror dari left):
```python
if side == -1:  # right leg
    hip_pitch = -hip_pitch
    knee = -knee
    ank_pitch = -ank_pitch
```

### D. OUTPUT FK untuk Robot 0 (jalankan `init_baru_fk.py`)

```
Right foot pos (m):  x = -0.0050   y = -0.0371   z = -0.1814
Left foot pos  (m):  x = -0.0015   y = +0.0276   z = -0.1810
```

Average foot position (untuk derive offset body):
```
avg_x = (-0.0050 + -0.0015) / 2 = -0.0032
avg_y ≈ 0  (right & left simetris di +Y dan -Y)
avg_z = (-0.1814 + -0.1810) / 2 = -0.1812
```

### E. CONVERT FK RESULT → walking_module init offsets

Walking_module pakai semantic ini (lihat `op3_walking_module.cpp:1220`):
```cpp
ep[2] = z_offset_ - leg_length;   // foot_z target relatif hip
```

Jadi:
```
init_z_offset = LEG_LENGTH + foot_z_from_hip
              = 0.2195 + (-0.1812)
              = +0.0383 m  ≈ 0.038
```

Untuk x dan y:
```
init_x_offset = -avg_x = +0.0032  ≈ 0.003   (body geser depan biar CoM di tengah kaki)
init_y_offset = |right_foot_y - left_foot_y| / 2
              = |-0.0371 - (+0.0276)| / 2
              = 0.0324  ≈ 0.032   (half feet spread)
```

### F. FINAL VALUES untuk param.yaml + Bascorro

```yaml
x_offset: 0.003    # body x shift (positive = body forward dari hip)
y_offset: 0.032    # half feet spread (32mm dari center, atau 64mm total spread)
z_offset: 0.038    # standing height — foot 181mm di bawah hip
```

### G. INTERPRETASI FISIK

- `z_offset = 0.038` artinya foot 181mm di bawah hip (= LEG_LENGTH - 0.038 = 0.2195 - 0.038 = 0.1815). Robot **mostly extended**, knee bend ringan (~60° per joint angle tapi geometrically kaki hampir lurus karena chain summation).
- `y_offset = 0.032` = 64mm feet spread total. Cukup buat stability tapi ga terlalu lebar.
- `x_offset = 0.003` = body 3mm di depan center hip → kompensasi minor untuk CoM.

### H. CARA PAKAI UNTUK ROBOT 1 (kalau INIT_BARU page 2 beda)

1. Export Robot 1's motion bin → yaml (action editor → Save as YAML), atau decode bin file langsung.
2. Buka `scripts/init_baru_fk.py`, update `PAGE2` dict dengan joint values Robot 1.
3. Run:
   ```bash
   python3 /home/alphonse/motion_webots/scripts/init_baru_fk.py
   ```
4. Copy 3 line output (`x_offset:`, `y_offset:`, `z_offset:`) ke `param.yaml`.
5. Update juga `WALKING_DEFAULT_PARAMS` di `App.jsx` (section 7) dengan nilai yang sama (`init_x_offset`, `init_y_offset`, `init_z_offset`).

### CATATAN PENTING:

- Script ga derive `pitch_offset` — joint sign convention OP3 ga sepenuhnya verified empirically. Pakai 4° = 0.0698 rad sebagai default; adjust ±1° kalau robot lean miring (terlalu nunduk = naikkan, terlalu mundur = turunkan).
- `init_z_offset` = `LEG_LENGTH + foot_z_from_hip`, BUKAN langsung jarak hip→foot. Salah interpretasi ini gampang.
- Robot 0 dan Robot 1 mungkin punya **calibration offset** berbeda → walaupun page 2 visually sama, raw dxl values bisa beda 10-30 step. Selalu re-derive untuk hardware baru.
- Kalau hasil `init_z_offset` keluar > 0.10 atau < 0.02, kemungkinan FK ada bug atau sign convention salah — verify dengan ngeplot kaki posisi vs raw values.

---

## 3. WALKING MODULE IN-CODE DEFAULTS (op3_walking_module.cpp:111-136)

### VERIFY:
```bash
grep -A 30 "walking_param_.init_x_offset" /home/alphonse/motion_webots/src/ROBOTIS-OP3/op3_walking_module/src/op3_walking_module.cpp | head -30
```

### EXPECTED:
```cpp
walking_param_.init_x_offset = 0.003;
walking_param_.init_y_offset = 0.032;
walking_param_.init_z_offset = 0.038;
walking_param_.init_roll_offset = 0.0;
walking_param_.init_pitch_offset = 4.0 * DEGREE2RADIAN;
walking_param_.init_yaw_offset = 0.0 * DEGREE2RADIAN;
walking_param_.hip_pitch_offset = 8.0 * DEGREE2RADIAN;
walking_param_.period_time = 750 * 0.001;
walking_param_.dsp_ratio = 0.35;
walking_param_.step_fb_ratio = 0.25;
walking_param_.z_move_amplitude = 0.040;
walking_param_.balance_enable = true;
walking_param_.balance_hip_roll_gain = 0.35;
walking_param_.balance_knee_gain = 0.4;
walking_param_.balance_ankle_roll_gain = 0.7;
walking_param_.balance_ankle_pitch_gain = 0.9;
walking_param_.y_swap_amplitude = 0.020;
walking_param_.z_swap_amplitude = 0.006;
walking_param_.pelvis_offset = 3.0 * DEGREE2RADIAN;
walking_param_.arm_swing_gain = 1.5;
```

### KENAPA UPDATE IN-CODE DEFAULTS:
Fallback kalau yaml fail load. Walking_module pakai defaults ini jadi nilai-nya HARUS match yaml.

---

## 4. EIGEN CRASH FIX (off-by-one trajectory size)

### VERIFY:
```bash
grep -E "all_time_steps.*int\(.*\)\s*\+\s*1" \
  /home/alphonse/motion_webots/src/ROBOTIS-OP3/op3_tuning_module/src/tuning_module.cpp \
  /home/alphonse/motion_webots/src/ROBOTIS-OP3/op3_base_module/src/base_module.cpp
```

Kalau ada hasil = belum di-fix.

### MASALAH:
`calcMinimumJerkTra` (di `robotis_math/robotis_trajectory_calculator.cpp:66`) return matrix dengan `round(mov_time/smp_time + 1)` baris. Tapi `tuning_module` + `base_module` resize matrix pakai `int(mov_time/smp_time) + 1`. Off-by-one kalau fractional ≥ 0.5 → Eigen `Block.h:146` assertion fail → SIGABRT.

`op3_walking_module.cpp:1254` udah ada fix-nya (`round(... + 1)`). Tinggal samain tuning + base.

### FIX:
Di **tuning_module.cpp** (line 215, 317, 434, 484) dan **base_module.cpp** (line 150, 244, 291) — replace:
```cpp
int(mov_time / smp_time) + 1
```
dengan:
```cpp
round(mov_time / smp_time + 1)
```

---

## 5. head_control_module SPAM FIX

### VERIFY:
```bash
grep -A 3 "wait for receiving current position" /home/alphonse/motion_webots/src/ROBOTIS-OP3/op3_head_control_module/src/head_control_module.cpp
```

Kalau hasilnya ada `while(has_goal_position_ == false)` = belum di-fix.

### MASALAH:
`head_control_module.cpp:123-127` blocking spin saat `has_goal_position_` belum di-set → spam stdout "wait for receiving current position" sampai 6 detik (selama module-enable handshake).

### FIX:
Replace while-loop dengan early return + WARN throttle:
```cpp
if (has_goal_position_ == false)
{
  RCLCPP_WARN_THROTTLE(this->get_logger(), *rclcpp::Clock::make_shared(), 2000,
                       "head_control_module: goal_position not initialized yet, dropping command");
  return;
}
```

head_tracking_node re-publish SCAN tiap 2.5s, jadi drop satu command ga apa-apa.

---

## 6. DEMO FLOW (op3_demo/src/demo_node.cpp + soccer_demo.cpp)

### VERIFY 6.1 — Boot path di demo_node.cpp:
```bash
grep -A 2 "controller running" /home/alphonse/motion_webots/src/ROBOTIS-OP3-Demo/op3_demo/src/demo_node.cpp
```

Harus ada `"controller running — playing INIT_BARU (action page 2)"`.

### MASALAH KALAU BELUM ADA:
- Tanpa wait `/robotis/present_joint_states`, page 2 publish bisa drop (controller belum running)
- Robot ga ke INIT_BARU di boot

### EXPECTED CODE di demo_node.cpp main() (setelah checkManagerRunning):
```cpp
// Boot-time INIT pose. Per user spec: robot lands at INIT_BARU (action
// page 2 — calibrated standing pose) when demo launches. Walking-ready
// is a separate transition that happens on START button press.
{
  bool modules_constructed = false;
  for (int i = 0; i < 150; ++i) {
    if (action_page_pub->get_subscription_count() > 0) {
      modules_constructed = true;
      break;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  bool controller_running = false;
  auto js_sub = node->create_subscription<sensor_msgs::msg::JointState>(
      "/robotis/present_joint_states", 1,
      [&controller_running](const sensor_msgs::msg::JointState::SharedPtr) {
        controller_running = true;
      });
  for (int i = 0; i < 100 && !controller_running; ++i) {
    rclcpp::spin_some(node);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  js_sub.reset();

  if (modules_constructed && controller_running) {
    RCLCPP_WARN(node->get_logger(),
                "controller running — playing INIT_BARU (action page 2)");
    goInitPose();
  } else {
    RCLCPP_WARN(node->get_logger(),
                "boot-init skipped: modules_constructed=%d controller_running=%d.",
                modules_constructed, controller_running);
  }
}
```

Plus include `#include <sensor_msgs/msg/joint_state.hpp>` di top.

### VERIFY 6.2 — goInitPose() pakai action_module + page 2:
```bash
grep -A 5 "void goInitPose" /home/alphonse/motion_webots/src/ROBOTIS-OP3-Demo/op3_demo/src/demo_node.cpp
```

Harus publish `"action_module"` ke `/robotis/enable_ctrl_module`, lalu publish `2` ke `/robotis/action/page_num`.

### VERIFY 6.3 — soccer_demo.startSoccerMode() skip action page 2:
```bash
grep -B 1 -A 10 "void SoccerDemo::startSoccerMode" /home/alphonse/motion_webots/src/ROBOTIS-OP3-Demo/op3_demo/src/soccer/soccer_demo.cpp
```

Harus langsung `setBodyModuleToDemo("walking_module")` tanpa replay page 2. Kalau ada `playMotion(WalkingReady)` atau setupan page 2 — REMOVE.

### VERIFY 6.4 — stopSoccerMode dengan debounce + head_tracker stop publish:
```bash
grep -A 20 "void SoccerDemo::stopSoccerMode" /home/alphonse/motion_webots/src/ROBOTIS-OP3-Demo/op3_demo/src/soccer/soccer_demo.cpp
```

Harus:
- Publish `"stop"` ke `head_tracker_cmd_pub_` 
- Call `ball_tracker_.stopTracking()` + `ball_follower_.stopFollowing()` LANGSUNG (bukan via flag)
- Set `stop_debounce_until_ = now + 1200ms`

### VERIFY 6.5 — buttonHandlerCallback reject toggle dalam debounce window:
```bash
grep -A 15 "void SoccerDemo::buttonHandlerCallback" /home/alphonse/motion_webots/src/ROBOTIS-OP3-Demo/op3_demo/src/soccer/soccer_demo.cpp
```

Harus check `std::chrono::steady_clock::now() < stop_debounce_until_` sebelum panggil `startSoccerMode()`.

### VERIFY 6.6 — soccer_demo.h tambahkan member:
```bash
grep "stop_debounce_until_\|chrono" /home/alphonse/motion_webots/src/ROBOTIS-OP3-Demo/op3_demo/include/op3_demo/soccer_demo.h
```

Harus ada `#include <chrono>` dan member `std::chrono::steady_clock::time_point stop_debounce_until_;`.

---

## 7. BASCORRO STUDIO (bascorro_studio/web/src/App.jsx)

### VERIFY 7.1 — Walking defaults:
```bash
grep -A 5 "init_x_offset:" /home/alphonse/motion_webots/src/bascorro_studio/web/src/App.jsx | head -10
```

Harus ada nilai FK-derived (`init_x_offset: 0.003`, `init_y_offset: 0.032`, `init_z_offset: 0.038`).

### EXPECTED WALKING_DEFAULT_PARAMS:
```js
const WALKING_DEFAULT_PARAMS = {
  init_x_offset: 0.003,
  init_y_offset: 0.032,
  init_z_offset: 0.038,
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
```

### VERIFY 7.2 — `initThenEnableWalking` skip action page 2:
```bash
grep -B 1 -A 20 "const initThenEnableWalking" /home/alphonse/motion_webots/src/bascorro_studio/web/src/App.jsx
```

Harus enable `walking_module` LANGSUNG, JANGAN ada `playInitBaru()` di sini.

### EXPECTED:
```js
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
```

### VERIFY 7.3 — `handleInitPose` PAKAI action page 2 (kebalikan dari initThenEnableWalking):
```bash
grep -B 1 -A 18 "const handleInitPose" /home/alphonse/motion_webots/src/bascorro_studio/web/src/App.jsx
```

Tombol "Init Pose" di UI HARUS literally ke INIT pose (action page 2), bukan walking_module.

### EXPECTED:
```js
const handleInitPose = () => {
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
```

### VERIFY 7.4 — Constants masih ada:
```bash
grep "INIT_BARU_PAGE_NUM\|ACTION_MODULE_SETTLE_MS\|INIT_BARU_PLAY_MS\|WALKING_ENABLE_SETTLE_MS\|actionPagePubRef" /home/alphonse/motion_webots/src/bascorro_studio/web/src/App.jsx | head -8
```

Harus ada deklarasi semua constant + `actionPagePubRef = useRef(null)`.

### VERIFY 7.5 — sim path `startWalkingOnSim` skip page 2:
```bash
grep -A 15 "startWalkingOnSim" /home/alphonse/motion_webots/src/bascorro_studio/web/src/App.jsx | head -20
```

Harus enable `walking_module` direct di sim namespace, ga ada action page 2 replay.

---

## 8. WALKING SIM PREVIEW SIMPLIFIKASI

User minta: "kalau untuk simulasi pada walking tidak perlu dibuat lengkap karena masih gagal juga, jadi hanya tampilkan saja deh model robotis op3 nya".

### Action: Simplifikasi `WalkingSimPreview` component

**Pendekatan**:
- Hapus subscription ke joint state simulator yang aktif (yang dulu konsumsi `/bascorro_studio/sim_preview/joint_states`)
- Buat 3D viewer load OP3 URDF static (T-pose / INIT_BARU static)
- TIDAK perlu sim_domain_bridge active walking
- TIDAK perlu manager_sim spawn lewat apply_node untuk walking

### VERIFY:
```bash
grep -l "sim_preview/joint_states\|sim_domain_bridge\|WalkingSimPreview" /home/alphonse/motion_webots/src/bascorro_studio/web/src/*.jsx 2>&1
```

### LANGKAH SIMPLIFIKASI (kalau perlu):
1. Di `WalkingSimPreview.jsx`: hapus prop/ref `simRos*`, hapus subscription `/bascorro_studio/sim_preview/joint_states`
2. Set joint state ke zero (T-pose) atau ke nilai INIT_BARU hardcoded
3. Hapus pemanggilan ke `sim_domain_bridge` dari Bascorro start scripts
4. Disable button "Apply & Start" di sim mode atau tampilkan note "sim walking dinonaktifkan"

---

## 9. LAUNCH FILE CLEANUP (demo_yolo.launch.xml)

### VERIFY:
```bash
cat /home/alphonse/motion_webots/src/ROBOTIS-OP3-Demo/op3_demo/launch/demo_yolo.launch.xml
```

### EXPECTED CONTENT:
```xml
<?xml version="1.0"?>
<launch>
  <arg name="with_manager" default="true"/>
  <include if="$(var with_manager)" file="$(find-pkg-share op3_manager)/launch/op3_manager.launch.py"/>
  <include file="$(find-pkg-share op3_yolo_vision)/launch/yolo.launch.py"/>
  <node pkg="op3_ball_localization" exec="yolo_to_demo_bridge" output="screen"/>
  <node pkg="op3_ball_localization" exec="head_tracking_node" output="screen">
    <param from="$(find-pkg-share op3_ball_localization)/config/head_tracking.yaml"/>
    <param name="auto_start" value="false"/>
  </node>
  <include file="$(find-pkg-share op3_camera_setting_tool)/launch/op3_camera_setting_tool.launch.xml"/>
  <node pkg="op3_demo" exec="op_demo_node" output="screen">
    <param name="grass_demo" value="False"/>
    <param name="p_gain" value="0.45"/>
    <param name="d_gain" value="0.045"/>
    <param name="use_head_control" value="false"/>
  </node>
</launch>
```

### REMOVED (kalau Robot 1 masih ada):
- `face_detection_op3.launch.xml` — print OpenCV build info, flood terminal, ga dipake soccer
- `web_setting_server.launch.xml` — web UI buat tune kamera, ga perlu untuk soccer demo

---

## 10. op3_manager.cpp — JANGAN tambah init code

### VERIFY:
```bash
grep -A 5 "Boot init pose" /home/alphonse/motion_webots/src/ROBOTIS-OP3/op3_manager/src/op3_manager.cpp
```

### EXPECTED:
```cpp
// Boot-time INIT pose is handled by demo_node (after waiting for the
// motion modules to load + the controller's first bulkread). It enables
// action_module and plays page 2 (INIT_BARU — the user's calibrated
// standing pose). Walking-ready is a separate transition triggered by
// the START button.
RCLCPP_INFO(node->get_logger(),
            "Boot init pose: deferred to demo_node (will play action page 2)");
```

**JANGAN** ada publisher creation (`node->create_publisher<...>`) + `std::this_thread::sleep_for` + `publish()` di area ini — itu yang dulu menyebabkan stack smash di hardware real.

---

## 11. BUILD + TEST SEQUENCE

```bash
cd /ros2_ws

# Clean build kalau migrasi pertama (delete build artifacts lama)
# rm -rf build/op3_manager build/op3_demo build/op3_walking_module build/op3_tuning_module build/op3_base_module build/op3_head_control_module
# rm -rf install/op3_manager install/op3_demo install/op3_walking_module install/op3_tuning_module install/op3_base_module install/op3_head_control_module

# Build all dependencies
colcon build --packages-up-to op3_manager op3_demo op3_walking_module --symlink-install
source install/setup.bash

# Verify build pick up fixes:
strings install/op3_demo/lib/op3_demo/op_demo_node | grep "controller running" | head -1
# Expected: "controller running — playing INIT_BARU (action page 2)"

strings install/op3_demo/lib/op3_demo/op_demo_node | grep "dropping command" | head -1
# Expected: "head_control_module: goal_position not initialized yet, dropping command"

# Kill semua
pkill -f "op3_manager/op3_manager"; pkill -f "op_demo_node"; pkill -f "head_tracking_node"; pkill -f "yolo_detector"; sleep 2

# Launch
ros2 launch op3_demo demo_yolo.launch.xml
```

### Expected log:
```
[op3_manager] Boot init pose: deferred to demo_node (will play action page 2)
[op_demo_node] controller running — playing INIT_BARU (action page 2)
... robot animates to INIT_BARU pose ...
[op_demo_node] Demo node loop start
```

Lalu pencet MODE → START:
- Walking pake init offsets dari yaml (foot_height 0.04, swing 0.02, etc.)
- Kaki gantian forward (hip_pitch ID 11/12)
- Body sway moderate (hip_roll ID 9/10)
- Tangan ayun counter-balance

### VALIDATION STATUS pada Robot 0 (per 2026-06-26):

| Item | Status |
|------|--------|
| Boot → robot ke INIT_BARU (page 2) otomatis | ✓ **CONFIRMED** |
| START → walking jalan, kaki gantian (ga lompat) | ✓ **CONFIRMED** |
| STOP button responsiveness | ⚠️ **BELUM OPTIMAL** — masih perlu iteration |

### STOP button optimization — root cause + fix (validated)

**3 issue yang dilaporkan user** dan rootcausenya:

| Issue | Root cause |
|-------|------------|
| 1. Robot lambat berhenti (>1s walking masih jalan) | `ball_follower.stopFollowing()` cuma publish `"stop"`, walking_module finish current step (~750ms = period_time). Foot still swings forward for ~750ms after STOP. |
| 2. Kepala tetap SCAN saat kaki berhenti | `head_tracking_node` "stop" handler set `_active=False`, tapi servo MASIH executing last commanded SCAN position. Head looks "still scanning" karena servo nyelesein perintah terakhir. |
| 3. Perlu 2-3x pencet baru beneran berhenti | Press 1 = stopSoccerMode (visual still moving). Press 2 dalam debounce 1200ms = REJECTED. Press 3 setelah debounce expired = RESTART (toggle). User interpret as "ga mati". |

**Fix yang udah di-apply (3 files)**:

1. **`op3_ball_localization/.../head_tracking_node.py`** — `_tracker_command_callback` "stop" branch:
   - Set `_scan_active = False`, reset `_last_scan_cmd_time`
   - **Publish neutral pose to head**: `JointState(name=[head_pan, head_tilt], position=[0.0, scan_tilt_forward_rad])`
   - Head ga lagi nyelesein last SCAN target — langsung snap ke neutral

2. **`op3_demo/src/soccer/ball_follower.cpp`** — `stopFollowing()`:
   - **Set walking amplitudes ke 0 BEFORE publish "stop"**: `setWalkingParam(0.0, 0.0, 0.0)`
   - Walking_module's next cycle baca x/y/angle = 0 → decelerate ke in-place march → smoother + visible stop sooner
   - Lalu publish `"stop"` command (existing behavior)

3. **`op3_demo/src/soccer/soccer_demo.cpp`** — `stopSoccerMode()`:
   - Re-order: `ball_follower.stopFollowing()` first (walking amplitudes → 0), THEN head publish "stop"
   - **Debounce 1200ms → 800ms**: period_time (750ms) + 50ms safety. Cukup buat walking stop tapi short enough biar press restart feel responsive.

**Expected behavior setelah fix**:
- Press STOP → kaki immediately decelerate (foot stops swinging forward ~50-150ms), head snap ke neutral position ~100ms, body settle ~750ms total
- Press 2 dalam 800ms = REJECTED dengan log `"Ignoring START press: still stopping (debounce)"`
- Press 2 setelah 800ms = walking restart smoothly

**⚠️ STATUS: WIP — STOP fix belum di-validate pada Robot 0**

Perubahan ada di 3 file (di-apply per 2026-06-26):
- `head_tracking_node.py` _tracker_command_callback "stop" branch
- `ball_follower.cpp` stopFollowing()
- `soccer_demo.cpp` stopSoccerMode()

**Untuk next Claude di Robot 1**:
- File-file ini AKAN ada di hand-over (kalau Robot 0 push terbaru), tapi belum proven kerja
- Test dulu di Robot 1: pencet START → jalan → pencet STOP. Lihat apakah responsive (foot ~150ms decelerate, head snap neutral, 1x pencet cukup)
- Kalau ada issue, kasih feedback ke user — issue ini akan di-iterate. Backup plan: rollback ke versi sebelumnya (debounce 1200ms, head ga snap neutral, ball_follower stop without amplitude zero) kalau perilaku baru lebih buruk

Issue list yang harus di-evaluasi:
- [ ] STOP responsive (foot decelerate cepet)?
- [ ] Head snap ke neutral (ga continue SCAN)?
- [ ] 1x pencet STOP cukup (ga perlu 2-3x)?
- [ ] Restart setelah debounce 800ms smooth?

---

## 12. JOINT ID REFERENCE (OP3)

| ID | Joint | Fungsi walking |
|----|-------|----------------|
| 1, 2 | r/l_sho_pitch | Arm swing (counter-balance) |
| 3, 4 | r/l_sho_roll | Tidak aktif di walking |
| 5, 6 | r/l_el | Tidak aktif di walking |
| 7, 8 | r/l_hip_yaw | Rotate leg (turning) |
| **9, 10** | **r/l_hip_roll** | **Body sway samping** (controlled by `swing_right_left`, `pelvis_offset`, `balance_hip_roll_gain`) |
| **11, 12** | **r/l_hip_pitch** | **Leg swing forward** (controlled by `x_move_amplitude` from ball_follower) |
| 13, 14 | r/l_knee | Knee bend during swing |
| 15, 16 | r/l_ank_pitch | Foot push-off + landing |
| 17, 18 | r/l_ank_roll | Foot side stability (`balance_ankle_roll_gain`) |
| 19 | head_pan | Head left-right (tracking ball) |
| 20 | head_tilt | Head up-down (tracking ball) |

---

## 13. TUNING NOTES (kalau Robot 1 berbeda)

### Robot terlalu liar / goyang besar:
- `y_swap_amplitude`: 0.020 → 0.012
- `arm_swing_gain`: 1.5 → 0.8
- `period_time`: 0.75 → 0.9 (lebih lambat)
- `balance_hip_roll_gain`: 0.35 → 0.5

### Robot kaki seret (foot scuff):
- `foot_height`: 0.04 → 0.045 (max 0.05)
- `z_swap_amplitude`: 0.006 (jangan turun ke 0)

### Robot oleng / jatuh:
- Verify `balance_enable: true`
- `dsp_ratio`: 0.35 → 0.40 (lebih banyak double-support)
- Cek `pitch_offset` = 4° (bukan 0)

### Robot pelan ga bisa belok:
- `balance_ankle_roll_gain`: 0.7 → 0.85
- Cek `hip_pitch_offset` = 8° (mempengaruhi ball distance calc juga)

### Kaki 9 & 10 (hip_roll) bergerak liar:
- `y_swap_amplitude`: turunin (paling efektif)
- `balance_hip_roll_gain`: naikkin (kompensasi IMU)

---

## 14. UNDERSTANDING ball_follower

ball_follower (file: `op3_demo/src/soccer/ball_follower.cpp`) **mengkontrol arah jalan**, tapi **TIDAK overwrite gait shape**.

### Yang di-OVERRIDE ball_follower (dari Bascorro UI x/y/angle move amp DIABAIKAN):
- `x_move_amplitude` — set berdasar jarak bola
- `y_move_amplitude` — selalu 0
- `angle_move_amplitude` — set berdasar pan bola
- `balance_enable` — force true saat START

### Yang DI-CACHE ball_follower dari Bascorro:
- `hip_pitch_offset` — buat hitung jarak bola (line 212): `distance = CAMERA_HEIGHT * tan(π/2 + tilt - hip_pitch_offset - ball_size)`
- `period_time` — throttle update walking command

### Yang DIPRESERVE (passthrough ke walking_module):
- Semua field lainnya: foot_height, swing, pelvis, balance, gains, init_offsets, dll

### Default in-place march constants di ball_follower.cpp:
```cpp
NOT_FOUND_THRESHOLD = 50      // tick (~1.67s) sebelum dianggap bola hilang
IN_PLACE_FB_STEP = -0.003     // step in-place pas nungguin bola
SPOT_FB_OFFSET = 0.0
SPOT_RL_OFFSET = 0.0
SPOT_ANGLE_OFFSET = 0.0
```

### Behavior:
1. Press START → ball_follower set `x_move=-0.003` (in-place march)
2. YOLO detect bola → ball_follower hitung x/angle dari ball position → robot maju ke bola
3. Bola hilang > 25 tick → `setWalkingParam(0,0,0)` → robot berhenti

---

## 15. CHECKLIST FINAL

**Pre-requisite custom komponen (section A)**:
- [ ] Repo root Robot 1 ditemukan + `$ROOT` di-set untuk semua command
- [ ] `op3_ball_localization/head_tracking_node.py` ada (head SCAN + tracking)
- [ ] `op3_ball_localization/yolo_to_demo_bridge.py` ada (YOLO → CircleSet bridge)
- [ ] `op3_yolo_vision` package + `models/yolo.onnx` ada
- [ ] `op3_ball_detector_msgs` package ada (CircleSet msg type)
- [ ] `bascorro_studio` package + npm deps installed
- [ ] `usb_cam` package ada (apt install kalau perlu)
- [ ] DynamixelSDK patched (RXPACKET_MAX_LEN)
- [ ] op3_kinematics_dynamics patched (acos/asin clamp)

**Walking + demo fixes (section 1-10)**:
- [ ] `param.yaml` matches section 1
- [ ] `walking_module.cpp` defaults matches section 3
- [ ] `tuning_module.cpp` + `base_module.cpp` pakai `round(... + 1)` (section 4)
- [ ] `head_control_module.cpp` pakai early return (section 5)
- [ ] `demo_node.cpp` punya `controller running` wait + goInitPose pakai action page 2 (section 6.1-6.2)
- [ ] `soccer_demo.cpp` startSoccerMode skip page 2 (section 6.3)
- [ ] `soccer_demo.cpp` stopSoccerMode + debounce (section 6.4-6.5)
- [ ] `soccer_demo.h` ada `stop_debounce_until_` member (section 6.6)
- [ ] `App.jsx` walking defaults FK-derived (section 7.1)
- [ ] `App.jsx` initThenEnableWalking skip page 2 (section 7.2)
- [ ] `App.jsx` handleInitPose PAKAI page 2 (section 7.3)
- [ ] `demo_yolo.launch.xml` removed face_detection + web_setting (section 9)
- [ ] `op3_manager.cpp` log-only deferral, no publisher init code (section 10)
- [ ] Build sukses + log "controller running — playing INIT_BARU" muncul (section 11)
- [ ] Robot animasi ke INIT_BARU di boot
- [ ] START button → walking dengan kaki gantian (ID 11/12), bukan goyang samping (ID 9/10)
- [ ] STOP button responsif (1x pencet, ga restart bug)

---

## CONTACT POINTS DI KODEBASE

Kalau ada bug atau perubahan baru:
- Walking gait logic: `op3_walking_module/src/op3_walking_module.cpp`
- Ball follow strategy: `op3_demo/src/soccer/ball_follower.cpp`
- Demo FSM: `op3_demo/src/demo_node.cpp` + `soccer_demo.cpp`
- Bascorro UI: `bascorro_studio/web/src/App.jsx`
- Bascorro backend: `bascorro_studio/bascorro_studio/apply_node.py`
- Motion bin format: `op3_action_module/data/motion_4095_ros1_lama.bin` (page 2 = INIT_BARU)

Dokumen ini terbaru per 2026-06-26. Untuk perubahan selanjutnya, update section yang relevan + checklist.
