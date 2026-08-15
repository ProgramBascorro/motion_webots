# Panduan: mekanisme parameter walking + layout Bascorro Studio

Catatan penerapan untuk robot berikutnya. Isinya **mekanisme**, bukan angka.
Nilai `x/y/z_offset` dan kawan-kawannya wajib diturunkan ulang per robot —
itu urusan `ROBOT1_WALKING_HANDOFF.md` dan `scripts/walking_ready_fk.py`
(di `CHRONUS_NEW` skrip itu masih bernama `scripts/init_baru_fk.py` — isinya sama).

Berkas ini sengaja dijaga **identik di semua branch robot**. Kalau memperbaikinya,
perbaiki di semua branch, jangan bikin salinan yang bercabang.

Semua yang di bawah ini boleh disalin mentah ke robot lain.

---

## 0. Cek cepat: perbaikan mana yang sudah ada di repo ini?

Jalankan dari akar repo. Setiap baris yang **tidak** menghasilkan keluaran berarti
perbaikannya belum terpasang — buka bagian yang disebut.

```bash
# A  parser angka bersama                              -> bagian 2A
test -f src/bascorro_studio/web/src/walkingParams.js && echo "A ok"
# A  input teks, bukan number                          -> bagian 2A
grep -q 'inputMode="decimal"' src/bascorro_studio/web/src/App.jsx && echo "A-input ok"
# B  perbandingan dirty tahan NaN                      -> bagian 2B
grep -q 'parseWalkingNumber(walkingParams\[field.key\], NaN)' src/bascorro_studio/web/src/App.jsx && echo "B ok"
# C  Start mengaktifkan modul dulu                     -> bagian 2C
grep -q 'startWalkingNow' src/bascorro_studio/web/src/App.jsx && echo "C ok"
# D  Balance sinkron + baca ulang dari robot           -> bagian 2D
grep -q 'refreshWalkingCurrent' src/bascorro_studio/web/src/App.jsx && echo "D ok"
# E  gerbang idle di walking module                    -> bagian 2E
grep -q 'ctrl_running_ == false && real_running_ == false' \
  src/ROBOTIS-OP3/op3_walking_module/src/op3_walking_module.cpp && echo "E ok"
# F  durasi trajektori double + lantai 1.5 s           -> bagian 2F
grep -q 'mov_time < 1.5 ? 1.5 : mov_time' \
  src/ROBOTIS-OP3/op3_walking_module/src/op3_walking_module.cpp && echo "F ok"
# F  prasyarat: round() di iniPoseTraGene              -> bagian 2F
grep -q 'round(mov_time / smp_time + 1)' \
  src/ROBOTIS-OP3/op3_walking_module/src/op3_walking_module.cpp && echo "F-round ok"
# H  TuningPage menolak period_time <= 0               -> bagian 2H
grep -q 'payload.period_time > 0' src/bascorro_studio/web/src/TuningPage.jsx && echo "H ok"
# I  grup Balance sudah dipecah                        -> bagian 3
grep -q 'Bentuk Gait' src/bascorro_studio/web/src/App.jsx && echo "I ok"
```

Untuk **G** (tiga salinan angka) tidak ada grep tunggal — bandingkan bertiga:

```bash
grep -E '^(period_time|dsp_ratio|foot_height|arm_swing_gain|p_gain):' \
  src/ROBOTIS-OP3/op3_walking_module/config/param.yaml
grep -nA3 'walking_param_.period_time =' \
  src/ROBOTIS-OP3/op3_walking_module/src/op3_walking_module.cpp
sed -n '/^const WALKING_DEFAULT_PARAMS/,/^};/p' src/bascorro_studio/web/src/App.jsx
```

Ingat konversi satuan di tabel bagian 1 sebelum menyimpulkan ada yang beda.

---

## 1. Peta: parameter walking hidup di empat tempat

```
op3_walking_module/config/param.yaml      ← yang benar-benar dimuat robot saat boot
        │  loadWalkingParam()  (ms→s, deg→rad)
        ▼
WalkingModule::walking_param_             ← struct hidup di RAM
        ▲                    │
        │ set_params         │ get_params
        │                    ▼
Bascorro Studio: App.jsx (tab Walking) + TuningPage.jsx (kartu Walking Tuner)
```

Ditambah **fallback** di `WalkingModule::initialize()`, dipakai hanya kalau
`param.yaml` hilang/rusak.

Jadi ada **tiga salinan angka yang sama**: `param.yaml`, fallback di
`initialize()`, dan `WALKING_DEFAULT_PARAMS` di `App.jsx`. Ketiganya diam-diam
bisa berbeda. Pernah kejadian di ALPHONSE: `param.yaml` diubah ke
`period_time: 500`, tapi dua yang lain tetap `750` selama berbulan-bulan — studio
menampilkan 0,75 s sementara robot berjalan 0,5 s, dan Apply pertama justru
**mengubah** gait robot. **Ubah satu, ubah ketiganya.**

### Peta nama & satuan (sering tertukar)

| `param.yaml` | pesan `WalkingParam` / UI | konversi |
|---|---|---|
| `period_time` (ms) | `period_time` (detik) | `× 0.001` |
| `roll/pitch/yaw_offset`, `hip_pitch_offset`, `pelvis_offset` (derajat) | sama (radian) | `× π/180` |
| `foot_height` | `z_move_amplitude` | nama beda |
| `swing_right_left` | `y_swap_amplitude` | nama beda |
| `swing_top_down` | `z_swap_amplitude` | nama beda |
| `x/y/z_offset` | `init_x/y/z_offset` | nama beda |

`balance_enable` **tidak ada** di `param.yaml` — `loadWalkingParam()` sengaja
tidak membacanya, jadi nilainya selalu `true` saat boot dari `initialize()`.

---

## 2. Delapan perbaikan mekanisme

| # | Gejala di lapangan | Sebab | Berkas |
|---|---|---|---|
| A | Nilai yang diketik balik jadi 0 setelah Apply/Load/Save | `Number("") === 0` | `walkingParams.js`, `App.jsx`, `TuningPage.jsx` |
| B | Titik kuning "berubah" nyangkut selamanya | `NaN !== NaN` | `App.jsx` |
| C | Start / Balance / Stop ditekan, robot diam saja | modul tidak pegang joint | `App.jsx` |
| D | IMU dimatikan, hidup lagi sendiri | `walking_param_ = *msg` | `App.jsx` |
| E | Amplitudo & CPU terbuang saat robot cuma berdiri | gait pipeline tidak digerbang | `op3_walking_module.cpp` |
| F | Robot menyentak sesudah Apply | `int mov_time` memotong | `op3_walking_module.cpp` |
| G | Studio dan robot beda angka sebelum Load | tiga salinan tidak sinkron | ketiganya |
| H | Kaki membeku setelah Apply dari Tuning | `period_time = 0` → bagi nol → NaN | `TuningPage.jsx` |

### A. Field kosong terkirim sebagai 0

`Number("")` bernilai **0**, dan `Number("0,5")` bernilai **NaN**. Input
`<input type="number">` juga mengembalikan `""` untuk apa pun yang dianggap
browser setengah jadi — sedang mengetik `"0."`, hasil paste, koma desimal.
Gabungannya: field yang dihapus untuk diketik ulang **terkirim ke robot sebagai 0**.

Perbaikannya dua bagian, keduanya wajib:

```js
// src/bascorro_studio/web/src/walkingParams.js  (dipakai App.jsx dan TuningPage.jsx)
export function parseWalkingNumber(raw, fallback) {
  if (typeof raw === "number") return Number.isFinite(raw) ? raw : fallback;
  if (raw === null || raw === undefined) return fallback;
  const text = String(raw).trim().replace(",", ".");
  if (text === "" || text === "." || text === "-" || text === "-.") return fallback;
  const numeric = Number(text);
  return Number.isFinite(numeric) ? numeric : fallback;
}
```

```jsx
<input type="text" inputMode="decimal" ... />   {/* BUKAN type="number" */}
```

`fallback` dipilih per pemanggil: `App.jsx` jatuh ke nilai default, `TuningPage.jsx`
jatuh ke nilai hasil Load terakhir.

### B. Penanda "berubah" macet

`Number(x) !== Number(y)` selalu `true` kalau salah satunya NaN. Pakai
`parseWalkingNumber(x, NaN) !== parseWalkingNumber(y, NaN)`.

### C. Perintah walking dibuang diam-diam

`walkingCommandCallback()` diawali:

```cpp
if (enable_ == false) { RCLCPP_WARN("walking module is not ready."); return; }
```

Init Pose / Head module / Action semuanya mengambil alih joint. Sesudah salah
satunya, `start` `stop` `balance on` `balance off` `save` **dibuang** — rosbridge
tetap melaporkan publish sukses, jadi UI-nya mengaku berhasil. Satu-satunya jejak
adalah WARN di log op3_manager.

Dua penanganan:

- Tombol **Start** mengaktifkan `walking_module` dulu kalau perlu, baru kirim
  perintah sesudah `WALKING_ENABLE_SETTLE_MS`.
- `sendWalkingCommand()` memberi peringatan **bersyarat** kalau modul kelihatan
  belum aktif. Jangan dijadikan klaim gagal: penanda itu hanya mencatat apa yang
  dilakukan tab browser ini, jadi sesudah halaman di-refresh nilainya `false`
  walaupun robot sedang berjalan.

### D. Balance menyala lagi sendiri

`walkingParameterCallback()` isinya cuma `walking_param_ = *msg;` — **seluruh**
struct ditimpa. Jadi `balance off` (yang cuma membalik satu bool) langsung
dibatalkan oleh Apply berikutnya kalau checkbox `balance_enable` di grid masih
menyala.

Tombol Balance karena itu harus ikut mengubah state UI, lalu **membaca ulang**
dari robot sebagai bukti:

```js
const sendBalanceCommand = (enable) => {
  const ok = sendWalkingCommand(enable ? "balance on" : "balance off", ...);
  if (!ok) return;
  setWalkingParams((prev) => ({ ...prev, balance_enable: enable }));
  window.setTimeout(() => refreshWalkingCurrent(...), 200);
};
```

`refreshWalkingCurrent()` memanggil `get_params` dan **hanya** memperbarui kolom
Current — tidak menyentuh field yang sedang diketik, jadi aman dipanggil di tengah
tuning. Statusnya menampilkan `balance_enable=false` langsung dari robot, sehingga
"sudah dimatikan tapi masih jalan" bisa dibedakan dari "perintahnya tidak sampai".

Aturan turunannya berlaku juga untuk `TuningPage.jsx`: kartu yang cuma
menampilkan sebagian field **wajib** mengirim `{...walkingFull, ...yangDiedit}`.
Mengirim struct sebagian akan menolkan sisa gait.

### E. Gerbang idle di walking module

`processPhase()` + IK kaki + `computeArmAngle()` + `sensoryFeedback()` dulu
dipanggil tiap 8 ms walau robot cuma berdiri, dan hasilnya dibuang. Digerbang jadi:

```cpp
const bool walking_idle = (ctrl_running_ == false && real_running_ == false);
if (walking_idle == false) { processPhase(...); ... sensoryFeedback(...); }
```

`ctrl_running_` **harus** ikut diperiksa: `stop` hanya menurunkan `ctrl_running_`,
dan yang menurunkan `real_running_` di batas fase berikutnya adalah `processPhase()`
sendiri. Kalau digerbang hanya dengan `real_running_`, proses berhenti tidak akan
pernah tuntas.

### F. Transisi pose sesudah Apply

Kalau pose berubah lebih dari 5°, modul masuk `WalkingInitPose` dan menjalankan
trajektori minimum-jerk. Versi lama memakai integer:

```cpp
int mov_time = err_max / 30;              // 29° dan 59° sama-sama jadi 1
iniPoseTraGene(mov_time < 1 ? 1 : mov_time);
```

Semua koreksi di bawah 60° dipadatkan ke ramp 1 detik yang sama → menyentak.

```cpp
double mov_time = err_max / 30.0;
iniPoseTraGene(mov_time < 1.5 ? 1.5 : mov_time);
```

**Prasyarat:** `iniPoseTraGene()` harus memakai `round()` untuk jumlah baris:

```cpp
int all_time_steps = round(mov_time / smp_time + 1);
```

Kalau masih `int(...) + 1`, `mov_time` pecahan menghasilkan baris kurang satu
dibanding `calcMinimumJerkTra()`, memicu assertion Eigen, dan **op3_manager mati
(SIGABRT)** persis saat walking module diaktifkan.

### G/H. Sisanya

G: samakan `period_time` di `param.yaml`, `initialize()`, dan
`WALKING_DEFAULT_PARAMS`. H: tolak `period_time <= 0` sebelum publish —
`updateTimeParam()` membaginya, dan nol membuat semua sudut gait jadi NaN.

---

## 3. Layout Bascorro Studio yang diubah

### Grup "Balance" dipecah dua

Dulu satu grup berjudul **Balance** memuat `balance_enable`, empat gain balance,
**dan** `y_swap_amplitude`, `z_swap_amplitude`, `arm_swing_gain`, `pelvis_offset`,
`hip_pitch_offset`. Terbaca seolah checkbox itu mengatur semuanya.

Kenyataannya `balance_enable` hanya menggerbang `sensoryFeedback()`, dan fungsi
itu menulis ke **persis 8 sendi kaki**:

```
r/l_hip_roll · r/l_knee · r/l_ank_pitch · r/l_ank_roll
```

`arm_swing_gain` sama sekali bukan parameter IMU. Rumusnya di `computeArmAngle()`
murni gait, tanpa gyro:

```cpp
arm_angle[0] = wSin(time_, period_time_, M_PI*1.5, -x_move_amplitude_ * arm_swing_gain_ * 1000, 0) * ...
```

Jadi mematikan balance tidak akan pernah menghentikan ayunan tangan. Untuk itu
set `arm_swing_gain = 0`, atau `x_move_amplitude = 0`.

Sekarang jadi dua grup:

| Grup | Isi |
|---|---|
| **Balance (IMU)** | `balance_enable` + 4 gain balance — hanya ini yang dimatikan checkbox |
| **Bentuk Gait** | `y_swap`, `z_swap`, `arm_swing_gain`, `pelvis_offset`, `hip_pitch_offset` |

> Catatan penting untuk penerapan: **ORION_NEW punya pengelompokan keliru yang
> sama persis**, baris per baris. Jangan menyalin grup dari sana.

### Tombol Advanced

Dulu `<details>` dengan panel `position:absolute` di dalam baris flex yang tidak
punya ancestor ber-`position` — popup-nya melayang ke tempat acak. Diganti toggle
inline `+ Advanced` / `− Advanced` yang memunculkan tombol di baris yang sama.

### Teks bantuan

`WALKING_PARAM_HELP` untuk `arm_swing_gain` dan `balance_enable` sekarang menyebut
batas kerjanya secara eksplisit. Ini yang mencegah salah paham terulang, jadi
ikut disalin.

---

## 4. Checklist penerapan ke robot berikutnya

Salin apa adanya:

- [ ] `src/bascorro_studio/web/src/walkingParams.js`
- [ ] `App.jsx`: import `parseWalkingNumber`, `normalizeWalkingParams`,
      perbandingan dirty, `<input type="text" inputMode="decimal">`,
      `sendWalkingCommand` + `sendBalanceCommand` + `refreshWalkingCurrent`,
      `startWalkingNow`, pemecahan `WALKING_PARAM_GROUPS`, toggle Advanced
- [ ] `TuningPage.jsx`: `parseWalkingNumber` di apply/tampilan/tombol ±,
      payload `{...walkingFull}`, penolakan `period_time <= 0`
- [ ] `op3_walking_module.cpp`: gerbang `walking_idle`, `iniPoseTraGene` double,
      `round()` di `iniPoseTraGene`

Turunkan ulang per robot — **jangan disalin**:

- [ ] `param.yaml` `x/y/z_offset`, `pitch_offset`, `hip_pitch_offset`
      → `scripts/walking_ready_fk.py --yaml <export page 2>`
      (`scripts/init_baru_fk.py` di `CHRONUS_NEW`)
- [ ] Fallback di `initialize()` disamakan dengan `param.yaml` hasil di atas
- [ ] `WALKING_DEFAULT_PARAMS` di `App.jsx` disamakan juga (ingat konversi satuan)

Build:

```bash
# di DALAM container
colcon build --symlink-install --packages-above op3_walking_module
# web
cd src/bascorro_studio/web && ./node_modules/.bin/vite build
```

`--packages-above`, bukan `--packages-select` — kalau header ikut berubah,
pemakainya jadi basi (lihat `SETUP_ROBOT_BARU.md` bagian 4).

Sesudah itu **restart op3_manager** dan **hard refresh** browser.

---

## 5. Urutan verifikasi di robot

Robot **digantung atau dipegang** sampai langkah 5 lewat.

1. `scripts/op3_ping.py` — pastikan tidak ada servo dengan `err=128`.
2. Enable walking_module, **jangan** Start. Ubah `init_z_offset` sedikit → Apply.
   Kaki harus melandai (dibatasi 25 °/detik), bukan menyentak.
3. Balance Off → checkbox `balance_enable` ikut padam → status menampilkan
   `balance_enable=false` dari robot → Apply → tetap padam.
4. Hapus isi sebuah field sampai kosong, ketik ulang → Apply → Load.
   Nilainya harus kembali persis, bukan 0.
5. Start dengan `x_move_amplitude` kecil.

Kalau langkah 3 tetap melaporkan `balance_enable=true`, perintahnya memang tidak
sampai — periksa apakah `walking_module` yang sedang pegang joint (bagian 2C).

---

## 6. Jebakan yang sudah terbukti

- **Jangan menyalin nilai gait dari robot lain.** `swing_right_left: 0.002`,
  `arm_swing_gain: 0.2`, `pelvis_offset: 0.5`, `p_gain: 0` adalah kombinasi yang
  di ALPHONSE terbukti menghasilkan gejala "kaki menyeret, tidak terangkat".
- **`save` menimpa `param.yaml` dan membuang seluruh komentarnya.** Semua
  penjelasan di berkas itu hilang sekali tekan.
- **Node yatim menahan port.** op3_manager yang gagal start sering tidak mati
  dengan Ctrl-C maupun SIGTERM (status `Sl`, PPID 1) dan tetap memegang port
  Dynamixel. Periksa dulu sebelum menyalakan ulang:
  ```bash
  docker exec op3 ps -eo pid,stat,comm | grep op3_manager   # SIGKILL kalau ada
  ```
- **`init_position_` di walking module tidak pernah dibaca** setelah
  `initialize()`. Jangan buang waktu menyetelnya.
