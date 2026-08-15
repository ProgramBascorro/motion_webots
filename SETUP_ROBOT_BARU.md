# Memakai repo ini di robot lain (mis. CHRONUS)

Repo ini dipakai di robot **ALPHONSE**. Sebagian besar isinya umum dan langsung
jalan di robot OP3 lain, tapi ada beberapa yang **khusus ALPHONSE** dan akan
salah kalau dibawa mentah-mentah. Dokumen ini memisahkan keduanya.

Baca bagian 1-3 saja sudah cukup untuk menjalankan. Sisanya untuk kalau ada yang
aneh.

Khusus untuk parameter walking dan tab Walking di Bascorro Studio, ada dokumen
terpisah: **`PANDUAN_PARAMETER_WALKING.md`** — mekanisme yang boleh disalin apa
adanya, nilai mana yang wajib diturunkan ulang per robot, dan urutan verifikasi
di robot.

---

## 1. Langkah cepat

```bash
git clone -b ALPHONSE_NEW https://github.com/ProgramBascorro/motion_webots.git
cd motion_webots

# 1. Sesuaikan alamat serial dengan perangkat yang tercolok di robot ini.
#    Tanpa --apply dia cuma memperlihatkan, tidak mengubah apa pun.
scripts/op3_serial_setup.py
scripts/op3_serial_setup.py --apply

# 2. Nyalakan container (image perlu dibangun sekali: docker build -t op3-webots-ros2:humble .)
scripts/op3_docker.sh up

# 3. Build di DALAM container -- wajib, lihat bagian 5
scripts/op3_docker.sh shell
colcon build --symlink-install

# 4. Periksa
scripts/op3_docker.sh doctor
```

`doctor` memandu, bukan sekadar melapor: kalau ada yang salah dia menyebut
perintah perbaikannya.

---

## 2. Yang khusus ALPHONSE — jangan dibawa mentah

| Hal | Kenapa khusus | Yang harus dilakukan |
|---|---|---|
| **Alamat serial** (U2D2 + OpenCR) | Setiap adaptor punya serial sendiri | `op3_serial_setup.py --apply` |
| **OpenCR di port USB terpisah** | Transceiver bus TTL board ALPHONSE **mati** | Lihat bagian 3 — kemungkinan besar CHRONUS tidak butuh ini |
| **Model servo di `OP3.robot`** | Tertulis `XM430-W350` untuk semua, padahal aslinya beda | Lihat bagian 6 |
| `op3_walking_module/config/param.yaml` | `x/y/z_offset` diturunkan lewat FK dari page 2 milik ALPHONSE | Turunkan ulang: `scripts/walking_ready_fk.py --yaml <export>` |
| `op3_action_module/data/ALPHONSE.bin` | Gerakan hasil kalibrasi ALPHONSE | Rekam sendiri jadi `CHRONUS.bin` — lihat bawah |
| `offset.yaml` | Offset mekanis per robot | Tuning ulang dengan offset tuner |

Yang **umum** dan aman dipakai apa adanya: `scripts/op3_docker.sh`,
`docker-compose.yml`, `docker-entrypoint.sh`, `script.sh`, seluruh `op3_tmux`,
dan semua modul C++.

### Berkas gerakan dinamai menurut robotnya

Dulu berkas ini bernama `motion_4095_ros1_lama.bin`, bersebelahan dengan
`motion_4095.bin` dan `motion_4095_ros1.bin` yang **tidak** dipakai — mudah
sekali salah pilih. Sekarang namanya nama robot, jadi jelas milik siapa.

Untuk CHRONUS, salin lalu ganti nama defaultnya sekali jalan:

```bash
cp src/ROBOTIS-OP3/op3_action_module/data/{ALPHONSE,CHRONUS}.bin
grep -rl 'ALPHONSE\.bin' script.sh scripts/ src/ --exclude-dir=.git \
  | xargs sed -i 's/ALPHONSE\.bin/CHRONUS.bin/g'
colcon build --packages-select op3_action_module --symlink-install
```

Sesudah itu **rekam ulang** page-page-nya lewat action editor — isi salinan itu
masih gerakan ALPHONSE, cuma namanya yang berganti. Untuk coba-coba tanpa
mengubah berkas, cukup `export OP3_ACTION_FILE=/path/ke/lain.bin`.

---

## 3. Keputusan yang harus diambil manusia: OpenCR di mana?

OP3 normal menaruh OpenCR (ID 200) di **bus TTL yang sama dengan servo**. Repo
ini defaultnya **tidak** begitu — OpenCR diberi port USB sendiri — karena
transceiver bus di board ALPHONSE mati.

**Buktikan dulu, jangan menebak.** Colokkan robot, lalu:

```bash
scripts/op3_docker.sh shell
python3 /ros2_ws/scripts/op3_ping.py
```

Alat ini tanpa dependensi (protokol 2.0 mentah), jadi tetap jalan meski
lingkungannya sedang bermasalah. Baca hasilnya:

| Hasil di port U2D2 | Artinya | Perintah |
|---|---|---|
| ID 200 **menjawab** bersama servo | Board sehat, susunan OP3 normal | `scripts/op3_serial_setup.py --apply --opencr=bus` |
| Servo menjawab, ID 200 **diam** | Transceiver TTL mati (kasus ALPHONSE) | `scripts/op3_serial_setup.py --apply --opencr=usb` |
| Semua diam | Kabel/daya/baud, bukan soal konfigurasi | Periksa fisik dulu |

Contoh keluaran di ALPHONSE — perhatikan ID 200 diam di bus servo tapi menjawab
di port USB-nya sendiri:

```
/dev/serial/by-id/usb-FTDI_..._FT3WKHM6-if00-port0  @ 2000000 baud
  ok  ID   1  model 0x0137  err=128
  ...
  ok  ID  20  model 0x001E  err=0
  diam: 200                                    <- transceiver bus mati

/dev/serial/by-id/usb-ROBOTIS_OpenCR_..._FFFFFFFEFFFF-if00  @ 2000000 baud
  ok  ID 200  model 0x7400  err=0  <- OpenCR di bus ini
```

> `err=128` berarti bit **hardware error alert** menyala di servo itu — biasanya
> overheating/overload yang ter-latch. Bukan masalah komunikasi. Bersihkan
> penyebabnya (beban/suhu), lalu reboot servo tersebut.

---

## 4. Jebakan yang paling mahal: build basi (ABI)

Kalau kamu mengubah **header** yang dipakai banyak paket — terutama
`robotis_controller.h` — lalu hanya membangun ulang paket itu sendiri, paket lain
tetap memakai **layout class lama**. Bukan error link, bukan crash: memori dibaca
di alamat yang salah.

Gejalanya sama sekali tidak menunjuk penyebabnya. `op3_action_editor` jalan,
mencetak daftar port dan 20 servo, berhenti tepat setelah
`[INFO] [robotis_controller]: Load offsets...`, lalu berputar **100% CPU
selamanya tanpa satu pun pesan error**. Terlihat seperti masalah serial atau
terminal — bukan.

**Selalu bangun berikut pemakainya:**

```bash
# SALAH -- pemakainya jadi basi
colcon build --packages-select robotis_controller

# BENAR -- paket itu DAN semua yang bergantung padanya
colcon build --packages-above robotis_controller --symlink-install
```

`scripts/op3_docker.sh doctor` bagian 6 mendeteksi ini otomatis, dan `up`
menjalankan `doctor` sendiri — jadi kamu akan diberi tahu, bukan menebak.

---

## 5. Kenapa harus build di dalam container

Seluruh `install/` dibangun dengan `--symlink-install` **di dalam** container,
tempat repo di-mount sebagai `/ros2_ws`. Di host semua symlink itu menggantung.

Jebakannya diam-diam: `source install/setup.bash` di host **berhasil, exit 0** —
colcon hanya melewati paket yang berkasnya tidak ketemu. Hasilnya `ros2 run`
menjawab `Package 'op3_action_editor' not found`, seolah paketnya lenyap.

**"Package not found" hampir selalu berarti kamu berada di host, bukan di dalam
container.** Jangan mulai membongkar `CMakeLists.txt`.

`build/` dan `install/` tidak pernah ikut ke git, jadi di robot baru `colcon
build` itu wajib, bukan opsional.

---

## 6. Model servo di `OP3.robot`

`OP3.robot` menulis `XM430-W350` untuk kedua puluh sendi. Di ALPHONSE itu
**tidak** sesuai kenyataan (lihat kolom model dari `op3_ping.py`):

| ID | Bagian | Model asli | Kode |
|---|---|---|---|
| 1-6 | lengan | MX-64 (protokol 2.0) | `0x0137` |
| 7-18 | kaki | XM540-W270 | `0x0460` |
| 19-20 | kepala | MX-28 (protokol 2.0) | `0x001E` |

Dibiarkan begitu **dengan sengaja**: MX seri 2.0 memakai tabel kontrol yang sama
dengan seri X, sedangkan berkas `MX-*.device` di repo ini protokol 1.0 dan akan
salah. Jadi jangan "membetulkan" nama model tanpa memeriksa berkas `.device`-nya.

Cek dulu model di robotmu dengan `op3_ping.py` sebelum menyalin apa pun.

---

## 7. Kalau ada yang aneh: baca `doctor`

`scripts/op3_docker.sh doctor` punya enam bagian:

| # | Memeriksa | Kalau merah |
|---|---|---|
| 1 | Perangkat tercolok di host | Colokan/kabel |
| 2 | Kestabilan USB 30 menit terakhir | Sering disconnect = masalah fisik |
| 3 | by-id di `OP3.robot` vs yang tercolok | `op3_serial_setup.py --apply` |
| 4 | Port bisa dibuka dari dalam container | Container dijalankan cara lama |
| 5 | `ros2` + paket op3 dari shell **baru** | Masuk lewat `op3_docker.sh shell` |
| 6 | Kesegaran build (ABI) | `colcon build --packages-above ...` |

### Gejala → penyebab

| Gejala | Kemungkinan besar |
|---|---|
| `Package '...' not found` | Kamu di host, bukan di container (bagian 5) |
| `ros2: command not found` | Shell tanpa ROS — pakai `op3_docker.sh shell` |
| Berhenti setelah `Load offsets...`, 100% CPU, tanpa pesan | Build basi / ABI (bagian 4) |
| `Error opening serial port` / `Error Set port` | Serial salah atau port dipakai proses lain |
| Manager jalan, gait berputar, **robot diam**, `present_joint_states` beku | fd serial mati diam-diam — container tidak pakai `-v /dev:/dev` |
| Editor/tuner nyangkut walau sudah Ctrl+C | Proses lama masih hidup memegang port: `pkill -x op3_action_edit` (nama dipotong 15 karakter!) |

---

## 8. Satu container, satu definisi

`scripts/op3_docker.sh` adalah **satu-satunya** tempat container didefinisikan.
`./script.sh` (menu Container) dan `docker-compose.yml` memakai container yang
sama (`op3`) dan hanya mendelegasikan ke situ.

Jangan membuat `docker run` sendiri. Yang wajib ada dan gampang terlupa:
`-v /dev:/dev` (bukan `--device=`, yang membekukan major:minor saat U2D2
re-enumerate), mount **seluruh** workspace (bukan cuma `src`, karena `build/` dan
`install/` harus ikut), `--restart unless-stopped`, `--init`, dan menjalankan
container detached — bukan `-it --rm bash`, yang membuat PID 1 adalah shell-mu
sehingga menutup terminal mematikan sekaligus menghapus container.
