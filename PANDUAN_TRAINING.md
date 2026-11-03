# Panduan Training

Menu **Training** di `./script.sh` melatih gerakan robot langsung di robot: memilih
sumber, memilih gerakan, menjalankan percobaan, melihat hasil terbaik, menulis
**feedback** dengan kalimat sendiri -- yang **langsung diterjemahkan menjadi
perubahan dan dicoba di percobaan berikutnya** (bagian 7) -- lalu **menyimpan**
hasilnya dan/atau **mengirimnya ke robot**. Ada dua sumber:

| Sumber | Yang dilatih | Pelatih | Hasil dikirim ke robot sebagai |
|---|---|---|---|
| **Bascorro Studio** | preset/versi walking (29 parameter jalan) | `scripts/autotune_walking.py` | preset baru / preset yang ditimpa di `walking_versions.json` (lewat server Studio) |
| **Action Editor** | satu page `.bin` (tendangan, bangun, pose, dll.) | `scripts/autotune_action.py` | page di `.bin` yang dipakai action_module |

Keduanya memakai cara yang sama: **CMA-ES** (pencarian acak terarah) di sekitar
gerakan asli, dimulai dengan beberapa **baseline** (gerakan asli apa adanya),
lalu kandidat demi kandidat. Tiap percobaan diukur dari IMU, dinilai, dan
**operator selalu boleh mengoreksi** (jatuh/hampir/normal).

---

## 1. Persiapan dan dependensi

1. Container `op3` hidup dan sehat: `bash scripts/op3_docker.sh doctor`.
2. **op3_manager hidup** (Launch picker → stack robot seperti biasa). Domain
   ROS diambil otomatis dari proses op3_manager -- tidak perlu `export
   ROS_DOMAIN_ID` lagi.
3. Workspace sudah ter-build (pesan `op3_walking_module_msgs`,
   `op3_action_module_msgs`, `robotis_controller_msgs`). Python: numpy dan
   PyYAML (sudah ada di container). `gum` untuk menu.
4. **Jangan** pakai tombol walking Studio, D60, atau Action Editor selama
   training -- mereka menulis ke modul/berkas yang sama.
5. Baterai di atas 11,1 V (alat berhenti dan meminta ganti baterai).

Keselamatan (robot ini sungguhan):

* Walking / tendangan / pose / gerakan umum: robot **digantung tali kendur**
  (menangkap jatuh, tidak menahan berat), ada orang di dekatnya, lintasan
  kosong ≥ 1 m.
* Bangun (getup): robot **di matras, berbaring** (tengkurap/telentang sesuai
  page), tali kendur atau dipegang longgar. Percobaan hanya mulai kalau IMU
  membaca robot memang berbaring.
* Detektor jatuh: roll/pitch tertapis > 45° = langkah/page dihentikan seketika.
* Ada jeda pendinginan 15 s antar percobaan; suhu servo tidak terbaca dari
  bus, jadi raba servo lutut/engkel sesekali.

## 2. Menjalankan

```
./script.sh            → 🏋️ Training
./script.sh --training (langsung)
```

Dari host, menu masuk ke container sendiri (`scripts/op3_docker.sh training`,
zona waktu host ikut supaya jam di daftar sama dengan Studio).

```
Training
  1) 🧪 Train dari Bascorro Studio
  2) 🎬 Train dari Action Editor
  3) 🏆 Riwayat hasil (feedback, simpan, kirim ke robot)
  4) 💾 Hasil tersimpan (kirim ke robot / ekspor)
  5) 📖 Panduan Training
  6) ↩ Kembali ke menu utama
```

Sesudah training berhenti (selesai, **q**, atau Ctrl+C) menu langsung membuka
**hasil terbaik** sesi itu, dengan tindakan yang sama seperti di riwayat
(bagian 6): tambah feedback, simpan dulu, simpan langsung ke robot, uji, dan
seterusnya.

Di setiap daftar ada **↻ Perbarui daftar** dan **↩ Kembali**. Daftar dibaca ulang
dari berkasnya setiap kali dibuka -- preset/page yang ditambah, diganti nama,
diubah, atau dihapus langsung terlihat tanpa mengubah kode.

### 2a. Train dari Bascorro Studio

1. Pilih preset. Daftarnya = `src/bascorro_studio/data/walking_versions.json`
   (berkas yang sama dengan dropdown Studio, urutan sama), dengan waktu
   pembaruan dan jenis gerak.
2. Layar info menampilkan: id, waktu pembaruan, revisi berkas, status acuan
   page 2, **jenis gerak** (dibaca dari angka, bukan nama -- preset "KANAN MIN
   NEW" ber-y +0,01 sebenarnya geser KIRI), parameter yang dilatih/dikunci,
   kriteria skor, dan sesi training yang sudah ada untuk preset itu.
3. **▶ Mulai sesi training baru**: nama sesi (disarankan otomatis), anggaran
   percobaan, lama jalan, jenis gerak (otomatis atau pilih sendiri).
   Atau **⏯ Lanjutkan** sesi yang sudah ada.

Jenis gerak menentukan apa yang dikunci dan apa sasaran beloknya:

| Jenis | Dikunci | Sasaran belok | Lama jalan bawaan |
|---|---|---|---|
| maju / mundur / di_tempat | x_move, y_move | lurus (0°) | 12 s |
| geser | x_move, y_move | lurus (0°) | 8 s |
| putar | x_move, y_move, angle_move | belok preset itu sendiri (diukur di baseline) | 8 s |

Kalau acuan page 2 berubah sejak versi Studio terakhir dimigrasi, preset
ditolak dengan perintah perbaikannya (`python3 scripts/walking_ready_fk.py --write`).

### 2b. Train dari Action Editor

1. Pilih page. Daftarnya = semua page berisi gerakan di `.bin` yang **sedang
   dipakai action_module** (env `OP3_ACTION_FILE` milik op3_manager yang hidup;
   jatuh ke `op3_action_module/data/ALPHONSE.bin`). Page yang punya nama di
   `enum Motion_Index` (`op3_demo/include/op3_demo/op_demo.h`) tampil paling
   atas dengan namanya: InitPose 1, WalkingReady 2, Ceremony 17, ForGrass 21,
   GetUpFront 85, GetUpBack 86, LeftKick 104, RightKick 105.
   Atau **⌨ Ketik nomor page...** untuk page mana saja.
2. Layar info menampilkan isi page (step, time/pause per step dan durasinya,
   speed, accel, sha isi), rantai `next` (mis. 105 → page 2), profil yang
   ditebak, kenop yang dilatih, kriteria, dan persiapan robot.
3. Validasi: page ditolak (dengan alasan) kalau nomornya di luar 1..255, page
   kerja (250 Studio / 254 training), kosong (stepnum 0), repeat 0 (ditolak
   action_module), checksum rusak, tanpa posisi kaki yang sah, atau rantai
   `next`-nya berputar tanpa akhir.
4. **▶ Mulai sesi training baru**: nama sesi, profil (otomatis atau pilih),
   anggaran percobaan.

Cara kerjanya: page **tidak** ditulis ulang bebas sendi per sendi. Yang dicari
adalah beberapa **kenop** yang dikenakan ke semua step page itu (rincian lengkap,
termasuk nilai servo per step: bagian 2c):

| Kenop | Rentang | Fungsi |
|---|---|---|
| `time_scale` | 0,85 .. 1,25 | pengali time + pause tiap step (1 = asli) |
| `pitch_hip_deg` | ±3° | condong lewat pinggul, dua kaki searah (negatif = badan atas ke DEPAN) |
| `pitch_ankle_deg` | ±3° | condong lewat engkel, dua kaki searah (negatif = seluruh badan ke DEPAN) |
| `roll_hip_deg` | ±2° | geser badan ke samping lewat pinggul |
| `roll_ankle_deg` | ±2° | condong ke samping lewat engkel (negatif = badan ke KANAN) |

"Searah" mengikuti arah sendi IK walking_module (pitch kanan/kiri berlawanan
tanda di raw = cermin, roll searah). Sendi bertanda INVALID/TORQUE_OFF di page
dibiarkan. Profil menentukan kenop dan kriteria:

| Profil | Ditebak dari | Kenop | Persiapan |
|---|---|---|---|
| `kick` tendangan | kick/tend/nendang/umpan | 4 kenop sudut; **kecepatan dikunci** (lebih pelan = lebih lemah) | digantung, mulai page 2 |
| `getup` bangun | getup/bang/bangun/tangi | time_scale, pitch hip, pitch ankle | berbaring di matras |
| `pose` pose diam | init/ready/pose, atau 1 step | pitch hip, pitch ankle, roll ankle | digantung, mulai page 2 |
| `umum` lainnya | selain itu | time_scale, pitch hip, pitch ankle | digantung, mulai page 2 |

Kandidat diputar lewat **page kerja 254**; page asli tidak ditulis selama
training, dan isi 254 dikembalikan persis saat alat selesai. Page lanjutan
(`next`, mis. page 2 sesudah tendangan) ikut diputar tapi tidak ikut dilatih.
Kalau page aslinya diubah di Action Editor di tengah sesi, training berhenti
(kandidat tidak boleh dibandingkan dengan page yang lain); lanjutkan lewat menu
(ia menawarkan "isi page yang baru") atau `--follow-page`.

Page 2 = acuan Init Pose walking (Studio + D60). Kalau hasil training page 2
dipakai, jalankan `walking_ready_fk.py --write` dan
`sync_ini_pose_from_page2.py --write` sesudahnya.

### 2c. Apa yang sebenarnya dioptimasi training page (Action Editor)

Menu: page → **📐 Apa yang dilatih di page ini**, atau hasil → **📐 Apa yang
dilatih** (`python3 scripts/training.py action-params 104` /
`action-params --session SESI --trial N`) -- menampilkan semua di bawah ini untuk
page itu, dengan angka servonya sendiri.

**1. Variabel pembentuk gerakan = kenop.** Yang dicari CMA-ES **bukan** posisi
tiap servo per step (31 sendi × 7 step = 217 angka yang bisa membanting robot),
melainkan 4-5 angka di atas. Tempatnya: `scripts/autotune_action_core.py`
(`KNOB_JOINTS` = sendi tiap kenop, `BOUNDS` = batas, `NEUTRAL` = nilai asli,
`PROFILES` = kenop & bobot per jenis gerak, `apply_knobs` = rumusnya). Per sesi
di `state.json`: `params` (kenop yang dicari), `lo`/`hi` (batasnya), `cma`
(titik tengah pencarian), `fixed` (kenop yang ditetapkan feedback), dan
`params` tiap percobaan (nilai kenop percobaan itu).

**Rumus ke nilai servo** -- per step, per sendi (raw Dynamixel 0..4095,
4096/360 = **11,38 raw per derajat**):

```
raw_baru  = raw_asli + round(arah_sendi × kenop_derajat × 4096/360)   (dijepit 0..4095)
time_baru = round(time × time_scale),  pause_baru = round(pause × time_scale)
```

`arah_sendi` = arah IK walking_module (`DIR_R`/`DIR_L` di `walking_ready_fk.py`):

| Kenop | Sendi kanan | Sendi kiri | Arah badan |
|---|---|---|---|
| `pitch_hip_deg` +1° | r_hip_pitch −11 | l_hip_pitch +11 | badan atas ke BELAKANG (negatif = ke depan) |
| `pitch_ankle_deg` +1° | r_ank_pitch +11 | l_ank_pitch −11 | seluruh badan ke BELAKANG (negatif = ke depan) |
| `roll_hip_deg` +1° | r_hip_roll −11 | l_hip_roll −11 | pinggul ke samping |
| `roll_ankle_deg` +1° | r_ank_roll +11 | l_ank_roll +11 | badan ke KIRI (negatif = ke kanan) |

Arah badan itu **diturunkan dari IK** (diuji di `test_training.py`), dan cocok
dengan page asli: TEND_KIRI step 2 memindahkan berat ke kaki kanan dengan
`ank_roll` raw negatif.

**Kenop fase** (dari feedback, bagian 7): kunci `kenop@a-b`, mis.
`roll_ankle_deg@2-5 = -0,5`, = kenop yang sama tapi **hanya di step 2..5**,
ditambahkan di atas kenop global (time_scale dikalikan). Step lain tidak berubah;
robot berpindah mulus ke/dari offset itu karena action_module menginterpolasi
antar step.

**Bagian gerakan** dibaca dari **data servo**, bukan nama page: step yang kaki
kiri-kanannya bergerak tidak simetris (> 8°, dilihat dalam arah IK) = satu kaki
terangkat. Contoh page 104 TEND_KIRI: step 1 persiapan, step 2 memindahkan berat
(ikut fase satu kaki), step 3-5 kaki kiri terangkat & menendang (kaki kiri
bergerak sampai 67°, kanan 18°), step 6-7 kembali ke posisi siap. Kaki tendang
dibandingkan dengan nama page (kiri/kanan/left/right) dan bedanya disebut.

**Tidak dioptimasi**: posisi servo per step secara bebas, jumlah step, speed/accel
page, p-gain, offset sendi (`offset.yaml`), page lanjutan (`next`).

**2. Setelan algoritma** (bukan gerakan): `--sigma` (langkah awal), `--popsize`,
`--trials` (anggaran), `--baseline-runs`, `--seed`, `--cooldown-sec`,
`--min-voltage`. Feedback **tidak** mengubahnya -- kecuali titik tengah CMA yang
dipindah ke titik feedback.

**3. Bobot penilaian** (sasaran evaluasi): metrik IMU dan bobotnya per profil
(bagian 6). Feedback **bisa** mengubahnya (dijaga 0,5..2× bobot bawaan).

**Dari hasil ke page**: isi page saat sesi dibuat (`state.json` `page_hex`) +
kenop percobaan (termasuk kenop fase) → `apply_knobs` → page 512 byte (checksum
dihitung ulang) → disimpan YAML format `action_yaml.py` + JSON `page_hex` → dikirim
ke page tujuan dengan cadangan `.bin` dan dibaca balik (bagian 6). Saat training,
page yang sama persis diputar lewat page kerja 254.

### 2d. Tab "Action Training" di Bascorro Studio (web)

Jalur kedua untuk page Action Editor, khusus **nilai servo per step** dengan
**penilaian operator** (2026-10-31; tendangan di `op_demo.h`: kanan 105/32/218/220/6,
kiri 104/219/221/7, page lain juga bisa dipilih). Tidak memakai CMA-ES; kandidat
dipilih mode Belajar yang sama dengan tab RL Training (`web/src/trainingLearn.js`).

- **Variabel**: `<sendi>@a-b` = derajat ditambahkan ke servo itu di step a..b, **arah
  raw** (angka sama dengan Action Editor, 11,38 raw/derajat); `time_scale@a-b` = pengali
  time + pause. Satu rumus dengan kenop di atas (`apply_knobs`).
- **Usulan otomatis** dari data servo (`page_phases`), bertahap: **A** tidak jatuh
  (engkel roll, pinggul roll, engkel pitch kaki tumpu, ±4°), **B** kena/lurus/jauh
  (kaki tendang di step menendang = ayunan pinggul terbesar, ±10°, yaw ±5°, waktu step
  itu ×0,7..1,3), **C** cepat kembali (waktu step sesudahnya). Step pertama dan
  terakhir tidak diusulkan. Batas keras ±15° dan waktu ×0,6..1,4 dari page asli.
- **Satu percobaan**: kandidat ke page kerja 254 → page 2 → rekam IMU → page 254
  diputar → metrik IMU (rumus bagian 6) → operator menilai stabil (normal/hampir/jatuh),
  bola (kena & lurus / melenceng kiri / melenceng kanan / tidak kena), jarak (cm, boleh
  kosong). Skor = stabil + bola + jarak (maks +1,5); "goyang sesudah" pemecah seri.
- **Simpan**: percobaan terbaik → page kosong dulu (bawaan) atau page asli (centang
  timpa), cadangan `.bin` utuh dibuat sebelum menulis.
- **Berkas**: `scripts/training_action_web.py` (dipanggil `apply_node`, aksi `act_*`),
  percobaan di `rl_runs/action/trials.json`. Uji: `test_training.ActionWebTest`.

## 3. Selama training

Tiap percobaan:

```
=== percobaan 12 (cma gen 1) ===
  period_time=0.66, dsp_ratio=0.21, ...
  Robot digantung di garis start? Enter = mulai, s = lewati, q = selesai:
  ... robot bergerak ...
  Menurut kamu? Enter = normal (deteksi), n = normal, j = jatuh/tertahan tali, h = hampir jatuh, dipegang/diangkat, c = tulis catatan:
  skor 0.812 | arah LURUS (-2 deg) | gyro 31.2 deg/s | ...
```

* **Enter** = mulai; **s** = lewati kandidat ini (tidak dihitung); **q** = selesai.
* Label sesudah percobaan: **Enter** = setuju dengan detektor, **n** = normal
  (bangun: berhasil), **j** = jatuh/tertahan tali (bangun: gagal), **h** = hampir
  jatuh/dipegang (bangun: dibantu). **Label operator mengalahkan detektor** --
  tali sering menahan badan sebelum IMU melihat jatuh.
* **c** = tulis **catatan bebas** untuk percobaan itu dulu (beberapa baris, lihat
  bagian 5). Catatan itu langsung **dipahami dan diterjemahkan** ke perubahan, buktinya
  ditampilkan, lalu: **Enter** = pakai di percobaan BERIKUTNYA, **t** = simpan
  sebagai catatan saja (bagian 7). Sesudah itu label ditanya lagi.
* Percobaan berjenis **feedback** = percobaan yang memakai perubahan dari catatanmu.
  Sesudahnya tampil perbandingan dengan percobaan yang dinilai, lalu **Enter** =
  pertahankan (CMA lanjut dari titik itu) atau **b** = batalkan.
* Label yang terlewat bisa diberikan belakangan: menu sesi → 🏷 Beri label
  (format `4=j,7=h`).
* **Ctrl+C kapan saja** = robot dihentikan (walking `stop` / page `brake`), sesi
  tersimpan.

Memantau dari terminal lain (di container):

```
python3 scripts/training.py results          # semua sesi, status, terbaik
python3 scripts/training.py result-info jb1  # rincian satu sesi
tail -f src/bascorro_studio/data/autotune/jb1/trials.csv
```

Status sesi: `BERJALAN (pid ..)` selama ada pelatih yang memegangnya
(`running.json`), `terhenti di x/y` (bisa dilanjutkan), `selesai`.

## 4. Menghentikan dan melanjutkan

* Hentikan: **q** saat ditanya, atau **Ctrl+C**. Semua percobaan yang selesai
  sudah tersimpan; percobaan yang terpotong tidak dihitung.
* Lanjutkan: pilih preset/page yang sama → **⏯ Lanjutkan**, atau
  🏆 Riwayat hasil → sesi → ⏯. Menu menangani sendiri:
  * **anggaran habis** → menanyakan total baru (bawaan +2 generasi);
  * **rumus skor berubah** → menawarkan hitung ulang dari rekaman mentah (rescore, ada cadangan);
  * **page 2 di-teach ulang** (Studio) → lanjut dengan offset pose digeser ke acuan baru;
  * **page asli diubah** (Action) → lanjut dengan kenop dikenakan ke isi baru.
* **🔁 Ulangi kandidat terbaik** (2-3 kali) sebelum dipakai di demo: skor kandidat
  jadi rata-rata semua run-nya, jadi kebetulan bagus sekali tidak menang.

Perintah langsung (setara menu):

```
python3 scripts/autotune_walking.py --session NAMA --start "JALAN BARU" [--trials 48] [--motion putar] [--walk-sec 8]
python3 scripts/autotune_walking.py --session NAMA [--trials 64] [--rerun-best 2 [--trial N]] [--label 4=j] [--rescore] [--report]
python3 scripts/autotune_action.py  --session NAMA --page 105 [--profile kick] [--trials 32]
python3 scripts/autotune_action.py  --session NAMA [--trials 48] [--rerun-best 2 [--trial N]] [--label 4=j] [--follow-page] [--report]
```

Parameter yang bisa diatur:

| Opsi | Bawaan | Fungsi |
|---|---|---|
| `--trials` | 48 (Studio) / 32 (Action) | anggaran TOTAL percobaan CMA; pada sesi lama menaikkannya |
| `--popsize` | 8 (Studio) / otomatis | kandidat per generasi |
| `--sigma` | 0,15 / 0,2 | langkah awal, pecahan rentang tiap parameter |
| `--baseline-runs` | 2 | berapa kali gerakan asli diukur sebagai pembanding |
| `--cooldown-sec` | 15 | jeda pendinginan servo antar percobaan |
| `--min-voltage` | 11,1 | di bawah ini alat meminta ganti baterai |
| `--walk-sec` (Studio) | 12 / 8 | lama jalan per percobaan |
| `--motion` (Studio) | dari angka | maju, mundur, geser, putar, di_tempat |
| `--params` | menurut jenis/profil | daftar parameter/kenop yang dicari (dipisah koma) |
| `--measure-distance` (Studio) | mati | operator mengetik jarak tempuh (cm) tiap percobaan |
| `--profile` (Action) | ditebak | kick, getup, pose, umum |
| `--trial N` | terbaik | dengan `--rerun-best`: ulangi percobaan N (menu "Uji hasil ini di robot") |
| `--warm-start SESI[:N]` | – | sesi BARU mulai dari terbaik (atau percobaan N) sesi lain |
| `--weights k=v,...` | bobot bawaan | sesi BARU: bobot skor lain |
| `--feedback-run` | – | jalankan iterasi feedback yang MENUNGGU (1 percobaan per langkah), lalu berhenti |

## 5. Feedback manual

Selain label n/j/h, kamu bisa menulis **evaluasi dengan kalimat sendiri**:

* **saat training**: tekan **c** di pertanyaan label (catatan untuk percobaan itu);
* **sesudah training / kapan saja**: 🏆 Riwayat hasil → sesi → hasil → **✍ Tambah
  feedback manual** (juga dari menu yang muncul sendiri sesudah training).

Sesudah disimpan, catatan langsung masuk ke alur **feedback → perubahan →
iterasi berikutnya** (bagian 7).

```
  Diuji lewat apa? Enter = 'page 105 (RightKick) di ALPHONSE.bin', atau ketik sendiri (mis. 'D60 di lapangan'):
  Ketik feedback. Boleh beberapa baris/paragraf, boleh ditempel (paste).
    Enter            = baris baru
    baris berisi  .  = SELESAI menulis (lalu dikonfirmasi)
    !batal  atau Ctrl+C = BATAL, tidak ada yang disimpan
  > Tendangan kiri sudah berhasil, tetapi kaki tumpuan masih goyah.
  > Setelah menendang, robot kesulitan kembali ke posisi siap.
  > .
  Kesimpulan (boleh kosong): b = baik, p = perlu perbaikan, g = GAGAL/jangan dipakai, Enter = tanpa kesimpulan:
  ---- feedback untuk kandidat #31 (2 baris, 118 karakter) ----
  | ...
  Enter = SIMPAN, u = tulis ulang dari awal, b = BATAL:
```

Yang membantu ditulis: kondisi dan perilaku robot, bagian yang sudah berhasil,
masalah yang muncul **dan kapan**, serta perbaikan yang diharapkan. Supaya bisa
diterjemahkan ke perubahan, sebut **arah** (ke depan/belakang/kiri/kanan),
**bagian** (kaki kiri/kanan/tumpuan, badan) dan **kapan** (awal/mulai, setelah
beberapa langkah, saat menendang, kembali ke posisi siap, saat berhenti).

* Satu hasil boleh punya **banyak catatan** (uji berulang); semuanya tampil utuh di
  rincian hasil, berurutan waktu, dengan kesimpulan dan "diuji lewat apa".
* Catatan menempel ke **kandidat** yang dinilai (nomor percobaannya). Catatan saat
  uji ulang (`rerun`) ikut ke kandidat aslinya, dengan nomor run-nya.
* Disimpan di `data/autotune/<sesi>/feedback.jsonl` (satu catatan per baris, hanya
  ditambah, dibaca balik sebelum dilaporkan "tersimpan"). Berkasnya terpisah dari
  `state.json`, jadi menulis feedback saat pelatih berjalan di terminal lain aman.
* Feedback boleh ditulis sebelum atau sesudah hasil disimpan/dikirim ke robot.

## 6. Hasil, menyimpan, dan mengirim ke robot

Semua sesi (dua sumber) ada di `src/bascorro_studio/data/autotune/<sesi>/`:

| Berkas | Isi |
|---|---|
| `state.json` | seluruh sesi: sumber, target, parameter, keadaan CMA, semua percobaan + metrik + label |
| `trials.csv` | satu baris per percobaan (dibuka di spreadsheet) |
| `feedback.jsonl` | semua feedback manual sesi ini |
| `hasil.json` | daftar hasil yang disimpan + riwayat pengiriman ke robot (waktu, target, status, cadangan) |
| `export/<sesi>-nNNN.*` | berkas hasil yang disimpan (format sumbernya, lihat 6a) |
| `state.json` `feedback_steps` | riwayat iterasi feedback: catatan → pemahaman → perubahan → percobaan → hasil (bagian 7) |
| `raw/trial_NNN.json.gz` | rekaman IMU mentah (untuk rescore; tidak di-commit) |
| `running.json` | ada selama pelatih memegang sesi |

**🏆 Riwayat hasil** → pilih sesi → ringkasan sesi (sumber, preset/page, waktu,
status, lokasi, kriteria, terbaik + alasan, hasil tersimpan) → pilih **satu
hasil**. Daftar hasil menandai 🏆 terbaik, 💾 tersimpan, 🤖 sedang di robot,
✍ jumlah feedback, ⚠ disimpulkan GAGAL. Rincian satu hasil memuat: identitas
(`<sesi>-nNNN`), sumber, percobaan & waktu, skor (rata-rata semua ulangan),
status terbaik + alasan, perubahan dari aslinya, metrik, kriteria, berkas,
riwayat pengiriman, dan **seluruh feedback**.

Tindakan untuk satu hasil:

```
✍  Tambah feedback manual -> terapkan & jalankan iterasi berikutnya
🔁 Terapkan feedback yang sudah ada & jalankan 1 iterasi
💾 Simpan hasil terlebih dahulu (robot belum disentuh)
🤖 Simpan langsung ke robot
▶  Uji hasil ini di robot (MENJALANKAN gerakan)
🧾 Riwayat iterasi feedback sesi ini
📐 Apa yang dilatih: kenop <-> nilai servo page ini      (Action Editor)
📤 Ekspor salinan berkas + feedback
↶  Kembalikan target ke isi sebelum penerapan terakhir
```

Hasil iterasi feedback (percobaan berjenis **feedback**) adalah hasil biasa: bisa
jadi terbaik, disimpan, dikirim ke robot, diuji ulang, dan diberi feedback lagi.

**4) 💾 Hasil tersimpan** = daftar semua hasil yang sudah disimpan (semua sesi,
dengan status kirimnya) untuk dikirim ke robot atau diekspor kapan saja.

Kriteria skor (makin kecil makin baik; gerakan asli ≈ 1,0 atau sedikit di
bawahnya, karena metrik yang di bawah batas bawahnya menyumbang kurang dari
bobotnya):

* **Studio (walking)**: 35% goyang gyro, 25% sebaran kemiringan, 30% simpangan
  belok dari sasaran, 10% condong. Jatuh 5..10, hampir 5, pose condong sebelum
  melangkah 10, kaki di luar jangkauan IK 15 (tidak dijalankan).
* **kick**: 30% goyang gyro selama gerakan, 25% miring terbesar, 20% selisih
  kemiringan akhir, 10% goyang sesudahnya, 15% badan berputar.
* **getup**: **wajib berhasil tegak** (|roll|,|pitch| < 25° di akhir), lalu 40%
  waktu sampai tegak, 30% goyang gyro, 30% goyang sesudah selesai. Gagal 5..10
  (makin jauh terangkat makin baik).
* **pose**: 40% goyang gyro, 40% sebaran kemiringan, 20% goyang sesudahnya.
* **umum**: 35% goyang gyro, 30% miring terbesar, 25% selisih kemiringan akhir,
  10% goyang sesudahnya.

(Feedback bisa mengubah bobot sesi; rincian sesi dan laporan menyebutnya. Semua
skor sesi dihitung ulang dengan bobot yang sama, jadi perbandingan tetap adil.)

**Hasil terbaik** = kandidat SAH (tidak jatuh/gagal/hampir menurut detektor
**dan** label operator) dengan skor rata-rata terendah, **kecuali** kandidat yang
kesimpulan feedback terakhirnya **GAGAL** -- ia dilewati dan alasannya disebut.
Alasan yang ditampilkan: skor dan jumlah kandidat sah, berapa persen lebih baik
dari rata-rata baseline, apakah sudah diulang. Kalau yang terbaik adalah
baseline sendiri, menu mengatakannya.

### 6a. Simpan hasil terlebih dahulu

Menulis hasil ke penyimpanan proyek **tanpa menyentuh robot**:

* **Studio** → `export/<sesi>-nNNN.json`: 29 parameter walking lengkap (format
  tab Walking Studio), perubahan dari preset asal, metrik, kriteria, acuan page 2.
* **Action** → `export/<sesi>-nNNN.yaml` (format `action_yaml.py`, bisa diimpor
  alat yang sudah ada; perintah impornya di baris ke-2 berkas) + `.json` berisi
  page utuh (`page_hex`, sha), kenop, dan perubahan raw per sendi.

Hasilnya terdaftar di `hasil.json` dan muncul di **Hasil tersimpan**. Saat
dikirim ke robot nanti, yang dipakai adalah **isi berkas itu** (dibaca ulang dan
diperiksa sha/formatnya), bukan dihitung ulang -- jadi yang dikirim = yang
disimpan.

### 6b. Simpan langsung ke robot

Pilih target, lalu menu menampilkan **rencana** sebelum menulis apa pun:
identitas hasil, robot (`ALPHONSE (komputer …)`), penyimpanan, target dan isinya
sekarang, yang akan berubah, hasil pemeriksaan (✔/✘), cara pencadangan, dan apa
yang terjadi sesudahnya. Hasil otomatis disimpan dulu (6a), jadi kalau pengiriman
gagal salinannya tetap ada dan bisa dicoba lagi.

**Bascorro Studio** (lewat server Studio -- penulis tunggal `walking_versions.json`):

| Target | Konfirmasi |
|---|---|
| 🆕 Preset BARU (nama sendiri) | ya/tidak; nama yang sudah dipakai ditolak |
| ♻ Timpa preset asal | ketik **YA** (id & nama preset tetap, isi lama dicatat) |
| 📋 Timpa preset lain | ketik **YA** |

Pemeriksaan: server Studio menjawab, acuan page 2 hasil = versi Studio =
sekarang, 29 parameter dengan nama & tipe yang sama dengan versi Studio,
jangkauan kaki IK. Cadangan: server Studio menyalin `walking_versions.json` ke
`data/backup/` setiap tulis, dan isi lama preset dicatat di `hasil.json`.
Verifikasi: preset dibaca balik dari server, 29 parameter harus sama persis.
Pilihan preset aktif di Studio tidak diubah.

Status sesudahnya: **TERSIMPAN di robot, BELUM aktif di walking_module**
(penerapan tidak menjalankan robot). Memakainya: Studio → **muat ulang halaman
(F5)** → pilih preset → Load → Apply & Start. Studio yang sedang terbuka masih
memegang angka lama: muat ulang dulu sebelum menekan Save di sana. Kalau nama
preset dipakai mode D60, menu menyebut perannya (berlaku saat D60 dinyalakan
berikutnya, tanpa build). Peran mendekat D60 memakai salinan beku
`op3_demo/config/d60_jalan_baru_r95.json`, jadi menimpa "JALAN BARU" tidak
mengubah D60 -- menu mengatakannya.

**Action Editor** (`.bin` yang dipakai action_module, env op3_manager):

| Target | Konfirmasi |
|---|---|
| 📥 Page KOSONG (disarankan dicoba dulu) | ya/tidak; nama page boleh diganti (maks 13 karakter) |
| ♻ Timpa page asal | ketik **YA** |
| ⌨ Page lain | ketik **YA** kalau page itu berisi |

Pemeriksaan: `.bin` terbaca & bisa ditulis, nomor page sah (bukan page kerja
250/254), page hasil lolos validasi (step, repeat, checksum, posisi kaki), tidak
ada sesi yang sedang melatih page itu. Cadangan: salinan utuh
`<bin>.bak.<waktu>` sebelum menulis (+ isi lama page dicatat). Verifikasi: page
dibaca balik dari berkas, harus sama byte demi byte.

Status sesudahnya: **AKTIF LANGSUNG** kalau op3_manager hidup dan memakai berkas
itu (action_module membuka ulang `.bin` setiap page diputar, tanpa restart);
"tersimpan, belum aktif" kalau manager mati atau memakai berkas lain. Action
Editor terminal: buka ulang page-nya; tab Action Studio: Reload. Page yang
dipakai demo (Motion_Index) disebut namanya. Page 2: jalankan
`walking_ready_fk.py --write` dan `sync_ini_pose_from_page2.py --write` sesudahnya.
Kalau page asal ditimpa hasil, **jangan** lanjutkan sesi itu dengan "isi page
yang baru" (kenop terkena dua kali) -- mulai sesi baru dari page itu.

**↶ Kembalikan** = target penerapan berhasil terakhir dikembalikan ke isi
sebelumnya (preset baru dihapus lagi, preset/page yang ditimpa diisi ulang isi
lamanya), hanya kalau target masih berisi persis hasil ini -- perubahan orang lain
sesudahnya tidak ditimpa.

Setiap pengiriman (berhasil, gagal, sama, dikembalikan) tercatat di
`hasil.json` dan tampil di rincian hasil dengan waktu, target, pesan
verifikasi, dan berkas cadangannya.

### 6c. Menguji hasil di robot (tindakan terpisah)

Menyimpan/mengirim **tidak pernah** menggerakkan robot. **▶ Uji hasil ini di robot**
menjalankan hasil itu lewat pelatihnya (`--rerun-best N --trial <nomor>`):
walking = page 2 lalu jalan dengan parameter hasil; page = page 2 (atau
berbaring) lalu isi page hasil lewat page kerja 254. Persiapan keselamatan sama
dengan training. Skornya ikut dirata-rata ke kandidat itu (bukti bukan
kebetulan), dan sesudah tiap run kamu bisa memberi label atau **c** catatan.
Hasil page tetap persis walau page aslinya sudah diubah sesudah training.

### 6d. Ekspor salinan

**📤 Ekspor salinan berkas + feedback** → folder (bawaan `training_export/`, tidak
di-commit) `<sesi>-nNNN/` berisi berkas hasil, `hasil.json` (identitas + riwayat
kirim + feedback), `feedback.md` (semua catatan, mudah dibaca), dan `BACA.txt`
(cara memakainya, termasuk perintah impor `action_yaml.py` untuk page).

## 7. Feedback → perubahan → iterasi berikutnya

Feedback **bukan hanya catatan**: tiap catatan diterjemahkan menjadi perubahan
yang **dipakai percobaan berikutnya di sesi yang sama**, lalu hasilnya
dibandingkan. Alurnya sama untuk Bascorro Studio dan Action Editor:

1. **Pemahaman** -- per kalimat: perilaku yang dikeluhkan (dengan frasa
   pemicunya), fase (awal / setelah beberapa langkah / saat menendang / kembali /
   berhenti), sisi & kaki, arah yang diinginkan, hal yang sudah baik, dan potongan
   yang **tidak dipahami**.
2. **Perubahan** -- pemahaman + konteks (preset/page, nilai kandidat yang dinilai,
   metrik IMU-nya, langkah feedback sebelumnya) → variabel gerakan dengan arah dan
   besar langkah tertentu, bobot penilaian, atau "tidak bisa" + alasannya.
3. **Validasi** -- batas nilai (tabel batas yang sama dengan pencarian), satuan,
   jangkauan kaki IK (walking) / page hasil tanpa sendi terjepit (page).
4. **Bukti sebelum dijalankan** -- layar menampilkan: feedback yang diterima,
   pemahamannya, tiap perubahan **sebelum → sesudah** dengan alasan, pemicu, dan
   batas, bobot sebelum → sesudah, data robot yang mendukung/menyangkal (mis.
   "IMU: condong −5,8° ke belakang -- cocok"), nilai servo yang berubah (page),
   preset/page yang terpengaruh, dan **identitas iterasi** (`if-N`, nomor
   percobaan berikutnya).
5. **Disimpan** di `state.json` sesi (`feedback_steps`, status MENUNGGU).
6. **Satu percobaan otomatis** memakai perubahan itu (jenis `feedback`), sebelum
   kandidat CMA mana pun. Robot tetap menunggu Enter "Robot digantung...?".
7. **Hasil**: dibuktikan bahwa nilai yang dijalankan = nilai langkah itu
   (parameter walking **dibaca balik** dari walking_module / page kerja 254
   **dibaca balik** dari `.bin`), lalu dibandingkan dengan percobaan yang dinilai:
   status (jatuh/hampir/sah), skor, dan tiap metrik, dengan kesimpulan **LEBIH
   BAIK / SETARA / LEBIH BURUK / BELUM BERHASIL**.
8. **Pertahankan** (Enter) → titik tengah CMA pindah ke titik itu (generasi yang
   belum selesai dibuang), variabel di luar ruang CMA ditetapkan (`fixed`) untuk
   SEMUA percobaan berikutnya; **batalkan** (b) → bobot kembali, CMA tidak
   berubah. Catatan baru pada percobaan feedback itu langsung menjadi iterasi
   berikutnya yang dibangun di atasnya.

**Status "DITERAPKAN" hanya berarti nilai itu sudah benar-benar dijalankan
robot.** Status lain: MENUNGGU (belum dijalankan), dibatalkan, tidak diterapkan
(catatan saja), tidak bisa diterapkan, diganti (usulan lebih baru).

Dari mana:

* **saat training** -- `c` di pertanyaan label → bukti di layar → Enter =
  percobaan berikutnya memakainya (lanjut CMA sesudahnya);
* **dari menu** -- hasil → **✍ Tambah feedback manual** (atau **🔁 Terapkan
  feedback yang sudah ada**) → bukti → **▶ Terapkan & jalankan 1 iterasi** →
  pelatih menjalankan satu percobaan → hasil barunya terbuka dengan
  perbandingannya. Memilih **📝 Simpan sebagai catatan saja** = tidak diterapkan
  (tetap tercatat). **⏯ Lanjutkan training** juga menjalankan iterasi yang masih
  MENUNGGU lebih dulu.

Catatan pada percobaan dengan nilai **persis sama** (mis. semua baseline) dibaca
bersama -- semuanya bicara tentang konfigurasi yang sama. Catatan yang sudah
dipakai iterasi yang dijalankan tidak dihitung lagi. Kalau preset/page aslinya
terus jatuh di baseline, iterasi feedback yang dipertahankan mengakhiri baseline
(CMA mulai dari titik feedback; skala skor memakai batas minimum).

### Tabel terjemahan

Besar satu langkah sengaja kecil. "MASIH ..." pada hasil sebuah iterasi = langkah
searah dibesarkan ×1,5 (paling besar 2×); arah berbalik = setengah langkah;
"sedikit" = setengah langkah.

| Feedback (contoh kata) | Bascorro Studio (walking) | Action Editor (page) |
|---|---|---|
| condong/berat/jatuh ke **belakang**, mendangak, kurang ke depan | `hip_pitch_offset` **+1°** (badan atas ke depan), bobot condong ×1,5; dicek dengan condong IMU | `pitch_ankle_deg` **−0,5°** di fase yang disebut (bawaan: satu kaki) |
| condong/jatuh ke **depan**, nyungsep, kurang ke belakang | `hip_pitch_offset` **−1°** | `pitch_ankle_deg` **+0,5°** |
| + telapak tidak menapak rata (bersama condong) | `init_pitch_offset` **∓1°** (seluruh badan diputar di engkel) | perlu info: telapak mana, bagian mana |
| condong ke kiri/kanan, kurang miring ke X | tidak ada parameter condong samping yang simetris → hanya bobot sebaran miring | `roll_ankle_deg` **0,5° ke arah yang diminta** di fase satu kaki |
| kaki **mentok**/menyeret/tersandung/kurang terangkat | `z_move_amplitude` **+5 mm** (kedua kaki -- tidak ada setelan per kaki) | badan **0,5° ke kaki tumpuan** (`roll_ankle_deg@fase satu kaki`); kaki dari teks atau dari data page |
| goyah, oleng, kurang stabil, perpindahan berat | `dsp_ratio` **+0,03** (dua kaki menapak lebih lama), bobot gyro ×1,4 & sebaran ×1,3 | bobot gyro/miring naik; **perlu arah** untuk menggeser badan |
| terlalu kuat / menyentak / kurang halus **di awal / saat berhenti** | `soft_start_time` **+0,25 s** | – |
| sulit kembali ke posisi siap | tidak diukur (walking dinilai saat berjalan) | `time_scale@fase kembali` **+10%** (step kembali lebih pelan; tendangan tidak berubah), bobot kemiringan akhir ×1,5 |
| terlalu cepat / kasar saat menendang | – | **tidak bisa**: kecepatan tendangan dikunci |
| langkah terlalu cepat / lambat | `period_time` **±0,03 s** | kick: dikunci; profil lain `time_scale ±0,05` |
| belok / melenceng (ke X, atau dari IMU) | `angle_move_amplitude` **0,01 rad ke arah sebaliknya** (preset putar: dikunci), bobot belok ×1,5 | bobot badan berputar ×1,5 |
| jatuh (arah tidak disebut) | bobot gyro & sebaran naik + **perlu arah** | sama |
| tendangan lemah | – | **tidak bisa** (kekuatan tidak diukur, kecepatan dikunci) |

Contoh nyata (catatan sesi `page104-1026`, 26 Okt): *"kaki kirinya itu mentok ke
tanah, menurut ku sih karena kaki kanan nya kurang miring ke arah kanan"* + *"masih
sama seperti yang sebelumnya, kaki kirinya mentok yah"* → kaki kiri mentok + badan
diinginkan lebih ke kanan → `roll_ankle_deg@2-5` 0 → **−0,5°** (step 2-5 saja;
`r_ank_roll` step 2 1916 → 1910, dst.). Catatan sesi `jalan-baru-1026-2` ("terlalu
mendangak ... jatuh ke belakang", "masih terlalu kuat untuk awalannya", "masih
berat kebalakang ... ga menapak semuanya") → `hip_pitch_offset` 8,02° → 9,02°,
`soft_start_time` 0,65 → 0,90 s, `init_pitch_offset` 0,79° → −0,21°; IMU percobaan
itu condong −5,8° (ke belakang) -- cocok.

**Kemampuan alat ini apa adanya**: pemahaman = **pola kata** bahasa Indonesia
(`BEHAVIORS` di `scripts/training_feedback.py`, termasuk salah ketik yang pernah
muncul), **bukan model bahasa**. Kalimat di luar pola dilaporkan "tidak
dipahami" beserta informasi yang dibutuhkan -- tidak ditebak. Arah perubahan
mengikuti pedoman tetap (`scripts/training_iteration.py`), besarnya kecil dan
dijepit batas; berhasil-tidaknya ditentukan percobaan di robot dan labelmu. Tinggi
kaki dan kekuatan tendangan tidak terukur IMU: untuk itu yang menilai hanya label
dan feedback-mu.

Kesimpulan **GAGAL** di feedback = kandidat itu tidak lagi dipilih sebagai
terbaik (bagian 6). Feedback **tidak** mengubah label n/j/h yang sudah tercatat.

**🧾 Riwayat iterasi feedback** (hasil/sesi) menghubungkan semuanya: catatan →
pemahaman → perubahan → percobaan → hasil → dipertahankan/dibatalkan.

Perintah langsung (setara menu, di container dari `/ros2_ws`):

```
python3 scripts/training.py trial-info SESI N|best      # semua tentang satu hasil (+ hasil iterasinya)
python3 scripts/training.py feedback-add SESI N         # tulis feedback
python3 scripts/training.py feedback-list SESI [N]
python3 scripts/training.py feedback-plan SESI N        # pemahaman + perubahan (tidak mengubah apa pun)
python3 scripts/training.py feedback-apply SESI N       # simpan sebagai MENUNGGU (--decline = catatan saja)
python3 scripts/autotune_walking.py --session SESI --feedback-run   # atau autotune_action.py: jalankan 1 iterasi
python3 scripts/training.py feedback-steps SESI [--last] # riwayat / hasil iterasi terakhir
python3 scripts/training.py action-params PAGE|--session SESI [--trial N]
python3 scripts/training.py save SESI N                 # simpan hasil terlebih dahulu
python3 scripts/training.py deploy SESI N --to page --target 201 [--name P] [--check] [--overwrite]
python3 scripts/training.py deploy SESI N --to studio-new --target "NAMA BARU" [--check]
python3 scripts/training.py deploy SESI N --to studio-preset --target "JALAN BARU" --overwrite
python3 scripts/training.py undo SESI N
python3 scripts/training.py bundle SESI N [--dir FOLDER]
```

## 8. Masalah umum

| Gejala | Sebab / jalan keluar |
|---|---|
| `service ... tidak ada -- op3_manager hidup?` | manager belum hidup, atau dijalankan di host. Jalankan lewat menu (masuk container sendiri). |
| `ROS/workspace tidak termuat` | menu dijalankan di luar container; pakai `./script.sh --training`. |
| `TIDAK BISA DILATIH: versi Studio belum dimigrasi` | `python3 scripts/walking_ready_fk.py --write` (Studio ditutup). |
| `page ... sudah diubah sejak sesi dimulai` | page di Action Editor berubah; lanjutkan dengan isi baru (menu menawarkan) atau buat sesi baru. |
| `robot belum berbaring` (getup) | baringkan robot; percobaan diulang, tidak dihitung. |
| `action_module tidak memutar page 254` | page kerja ditolak (lihat `/robotis/status`); pastikan tidak ada page lain yang sedang diputar. |
| `sesi ... sedang dipakai proses lain` | ada terminal lain yang sedang melatih sesi itu. |
| jam di daftar beda dengan Studio | jalankan lewat `./script.sh` (TZ host ikut); atau `export TZ=Asia/Jakarta`. |
| `koneksi: server Studio tidak menjawab` | Bascorro Studio belum hidup. Nyalakan, lalu kirim lagi dari Hasil tersimpan (salinan hasil tetap ada). |
| `acuan page 2: ... -- angka init pose ... berarti acuan lain` | page 2 di-teach ulang sesudah training; latih ulang dari preset yang sudah dimigrasi. |
| `target SUDAH BERISI` | menimpa butuh **YA**; untuk mencoba, pakai page kosong / preset baru. |
| `... sudah diubah lagi sesudah penerapan -- tidak dikembalikan otomatis` | target diubah orang lain; kembalikan manual dari cadangan yang disebut. |
| `--warm-start ...: isi page ... tidak berlaku` | page asal berubah sejak sesi lama; mulai sesi baru dari page sekarang (tanpa warm start). |
| feedback "BELUM bisa diterapkan" | tidak ada perubahan yang sah: baca baris "? perlu" / "tidak dipahami", tulis feedback lagi dengan arah, bagian, dan kapan. |
| `sudah di batas ... tidak bisa lebih jauh` | variabel itu sudah di batas aman; masalahnya perlu perubahan lain (page: ubah step di Action Editor). |
| `sesi ... sedang berjalan ... tulis catatannya di terminal training` | pelatih sesi itu masih hidup di terminal lain: pakai **c** di sana. |
| `Menu ini versi lama` | menu Training dibuka sebelum pembaruan: keluar lalu buka lagi. |
