# RL Training & Action Training (Bascorro Studio, CHRONUS)

Dua tab Studio untuk melatih gerakan **di robot asli dengan penilaian operator**.
Diambil selektif dari branch `ALPHONSE_NEW_V3` (hanya bagian Studio; menu Training
terminal tidak) dan disesuaikan dengan CHRONUS. **Tidak ada data** dari ALPHONSE
yang ikut: percobaan, versi walking, page, dan model mulai kosong di robot ini.

| Tab | Yang dilatih | Disimpan di robot |
|---|---|---|
| **RL Training** | parameter walking (data awal = versi walking Studio) | `rl_runs/studio/trials.json` |
| **Action Training** | nilai servo satu page `.bin` (tendangan, bangun, pose) | `rl_runs/action/trials.json`, cadangan `.bin` di `rl_runs/action/backup/` |

## Cara kerja (sama untuk kedua tab)

1. **Kandidat** dipilih `web/src/training/learn.js`: mode *Belajar* = cross-entropy
   sederhana (elit = percobaan baik, pusat berbobot `2^(skor − terbaik)`, sebaran
   menyempit tiap hasil kokoh, melebar kalau 3 terakhir gagal), *Acak penuh*, atau
   (Action) *Page asli* sebagai pembanding.
2. **Robot menjalankan** kandidat; IMU `/robotis/open_cr/imu` direkam.
3. **Operator menilai** kestabilan (normal +2 / hampir jatuh 0 / jatuh −4) dan hasil
   (+3 / +1 / −1). Skor = jumlahnya. Metrik IMU tampil di samping sebagai pembanding
   objektif dan pemecah seri — penilaian operator tetap kebenaran akhirnya.
4. Percobaan dibandingkan hanya dengan yang **sebanding**: sesi yang sama, target/page
   yang sama, dan data awal / isi page (sha) yang sama.

### RL Training — satu percobaan
Walking module diaktifkan hanya kalau belum memegang kaki → stance data awal
(amplitudo nol) dikirim dan **dibaca balik** dari `/robotis/walking/get_params` →
`soft_start_time` dipasang (harus diterima node) → tunggu stance 2,5 s → parameter
percobaan dikirim + dibaca balik → `start` → `soft_stop`. Gagal sebelum `start` =
tidak ada yang dinilai.

Diacak bawaan: x/y/angle move, period. Bisa dicentang: dsp, angkat kaki (maks
**0,025 m**, servo 17), goyang kiri-kanan, **hip pitch / pitch offset, soft start**.
Target: jalan maju, mutar kiri/kanan, geser kanan/kiri; data awal otomatis per target
(`geser kanan RL` → `geser kanan`, `geser kiri RL` → `geser kiri`, `mutar kiri/kanan`).

### Action Training — satu percobaan (`web/src/training/actionTrial.js`)
Kandidat ditulis ke **page kerja 254** (page asli tidak disentuh) → batas kecepatan
servo dilepas → `action_module` memegang kaki (dicek lewat layanan) → page 2 →
rekam diam 1 s → page 254 sampai selesai (rem otomatis kalau miring > 50° atau
melewati batas waktu) → rekam 1,5 s → metrik dihitung `apply_node`.

| Jenis page (dari **nama page**) | Usulan variabel | Penilaian hasil | Mulai dari |
|---|---|---|---|
| tendangan (`TEND_KANAN` 7, `TEND_KIRI` 8, …) | A kaki tumpu ±4°, B kaki tendang ±6° / yaw ±4° / waktu ×0,8–1,2, C waktu kembali ×0,8–1,0 | kena lurus / melenceng kiri / kanan / tidak kena (+ jarak, maks +1,5) | page 2, digantung |
| bangun (`BANGBEL` 12, …) | waktu seluruh page ×0,85–1,15 | berdiri sendiri / dibantu / gagal | **berbaring** (dicek IMU ≥ 45°) |
| pose / lainnya (`menahan bola` 14, …) | tidak ada — tambah manual | sesuai / mendekati / tidak | page 2, digantung |

Variabel `<sendi>@a-b` = derajat **arah raw** (angka Action Editor, 11,38 raw/°),
`time_scale@a-b` = pengali time+pause. Batas keras ±15° dan ×0,6–1,4. Step pertama
dan terakhir tidak diusulkan (sambungan ke/dari page 2).

**Simpan hasil**: percobaan terbaik → page kosong (bawaan) atau timpa page asli
(centang). Cadangan `.bin` utuh dibuat dulu; page ditulis **di tempat** dan dibaca
balik — `action_module` membuka `.bin` sekali dan membaca ulang page tiap diputar,
jadi isi baru langsung terpakai tanpa restart. Tab Action: muat ulang dari bin (↓)
sebelum menyunting di sana.

## Beda dengan ALPHONSE_NEW_V3 (disengaja)

* Jenis page dari **nama page**, bukan `Motion_Index`: `op_demo.h` CHRONUS tidak
  sejalan dengan `.bin` (WalkingReady = 8 padahal page 8 = TEND_KIRI; RightKick 218,
  LeftKick 104, GetUp 85/86 kosong — ditampilkan sebagai catatan di tab).
* Page bangun dan pose kiper ikut bisa dilatih, dengan penilaian sendiri.
* Rentang usulan tendangan lebih sempit (ALPHONSE ±10°, ×0,7–1,3): servo 17 pernah
  reset daya dan lutut melatch OVERLOAD saat diperintah melompat jauh.
* Sebelum kandidat ditulis, dicek lewat `/proc/<pid>/fd` bahwa `op3_manager` membuka
  **inode** `.bin` yang sama (env tidak cukup: `action_module` diam-diam pindah ke
  berkas cadangan kalau `OP3_ACTION_FILE` gagal dibuka).
* RL: tetap `soft_stop`, batas angkat kaki 0,025, stance acuan tetap `REF_*`
  (offset menggeser relatif page 2 secara deterministik — trik "patok acuan" ALPHONSE
  tidak diperlukan). Batas hip pitch / pitch diambil dari versi walking CHRONUS.
* Kunci robot bersama (`training/robotLock.js`): dua tab tidak bisa menjalankan
  percobaan bersamaan.

## Berkas

```
bascorro_studio/training/        (Python, murni, dipanggil apply_node)
  action_bin.py      format .bin, tulis di tempat, validasi, cadangan
  action_knobs.py    variabel -> page, fase page dari data servo
  imu_metrics.py     kemiringan tertapis, detektor jatuh, metrik satu percobaan
  action_trainer.py  aksi act_pages/act_page/act_load/act_restore/act_metrics/act_deploy
  trial_store.py     rl_* dan act_get/act_add_trial/act_delete_trial
web/src/training/                (JS)
  learn.js  imuMetrics.js  actionTrial.js   (murni, diuji)
  imuRecorder.js  actionRobot.js  studioClient.js  robotLock.js   (ROS / React)
web/src/RlTrainingPage.jsx  web/src/ActionTrainingPage.jsx
```

Dependensi: tidak ada yang baru. Python `numpy` (sudah ada di container, dimuat saat
tab Action Training pertama dipakai); web memakai `roslib` dan `lucide-react` yang
sudah ada. Layanan ROS: `/robotis/action/is_running`, `/robotis/get_present_joint_ctrl_modules`,
`/robotis/walking/get_params`, `/op3_walking_module/set_parameters`.

## Menerapkan dan menguji

* `apply_node` cukup **di-restart** (paket symlink ke `src/`). Web: Vite HMR / refresh.
  Tanpa robot nyata jangan dicoba di robot: percobaan pertama **robot digantung**.
* Jangan jalankan node demo, `rl_geser_kanan.py`, atau Action Editor terminal selama
  training — tab menolak kalau node demo terdeteksi.
* Uji tanpa robot:

```
cd src/bascorro_studio && python3 -m pytest test/ -q            # 21 uji, salinan .bin di folder sementara
cd src/bascorro_studio/web && node --test test/*.test.mjs        # 15 uji
```

## Belum dikerjakan

* Menu Training terminal ALPHONSE (CMA-ES `autotune_walking/action`, feedback teks →
  perubahan, riwayat/ekspor hasil) — tidak dibawa; bergantung pada alat ALPHONSE
  (`walking_ready_fk.py`, `sync_ini_pose_from_page2.py`, D60).
* Hitung template dari banyak percobaan (`rl_hitung_template.py` ALPHONSE) — di CHRONUS
  masih dihitung manual seperti sesi 27–29/10.
* Belum diuji di robot sungguhan.
