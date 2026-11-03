# Runbook: Fix Serial OpenCR OP3 — "Error Set port" & "Failed to turn on Power" (untuk Claude Code)

> **Untuk agen Claude Code / chat berikutnya.** Ini SATU dokumen gabungan (menggantikan
> `TROUBLESHOOT_OPENCR_POWER.md` yang lama). Isinya: diagnosis + fix lengkap untuk error
> serial OpenCR di robot OP3. Sudah terbukti di robot ini (branch `ALPHONSE_NEW_V3`) dan
> kembarannya. Kerjakan berurutan.
>
> **Aturan #1 — jangan hardcode.** Semua nilai konkret (nama container, nomor `ttyUSB`,
> serial FTDI, isi by-id) **BEDA di tiap robot**. Deteksi di FASE 0, jangan menyalin angka
> dari mana pun.
>
> **Aturan #2 — jangan sentuh `op3_manager.cpp` (C++).** Menambah publisher/init di situ
> pernah memicu *stack smashing* di hardware. Fix ini cukup di **config + script**
> (`OP3.robot`, `executor.py`, `profiles.sh`, `ensure_opencr_port.py`, `main.cpp` default).

---

## 1. Ringkasan masalah

Tool ROS 2 (`ros2 run op3_action_editor executor.py`, `op3_manager`, offset tuner, dll.)
gagal bicara ke OpenCR. Muncul salah satu / beberapa error ini:

```
[PortHandlerLinux::SetupPort] Error opening serial port!
[ERROR] ... Error Set port                          <- tahap buka port / power-on
[ERROR] ... PORT [...] SETUP ERROR!                 <- tahap robotis_controller
[ERROR] ... Failed to turn on the Power of DXLs!    <- tahap power-on sub-controller
[ERROR] ... Torque on DXLs! [[RxPacketError] ...]
[ERROR] ... Fail to control LED [[RxPacketError] ...]
```

**Fakta kunci: board-nya SEHAT.** Lewat **Dynamixel Wizard** di host, RX/TX ke servo lancar.
Jadi ini **bukan** firmware rusak, **bukan** board mati, **bukan** baudrate, **bukan** port
kepakai program lain. (Lihat §9 kalau Wizard pun gagal — itu baru masalah lain.)

### Dua string error itu artinya beda — penting dibedakan
| Error | Tahap | Artinya |
|---|---|---|
| `Error opening serial port` / `Error Set port` / `SETUP ERROR` | **buka** port | Node `/dev/ttyUSBx` yang dibuka **salah/basi** → `open()` gagal |
| `Failed to turn on Power` / `Torque on DXLs! [RxPacketError]` / `Fail to control LED` | **tulis** ke sub-controller (ID 200) | Port kebuka tapi write gagal — di kasus kita karena yang dibuka node **basi** (by-id board lain), bukan `/dev/ttyOP3` |

Keduanya **satu akar**: aplikasi membuka node serial yang salah. Sekali semua tool memakai
alias stabil `/dev/ttyOP3`, dua-duanya hilang.

---

## 2. Akar masalah (2 lapis)

1. **Node serial di container basi.** Container di-run dengan `--device=/dev/ttyUSBx`
   (lihat `scripts/op3_tmux/lib/actions.sh` → `action_docker_run`, ~baris 253). Docker
   mengunci node itu ke **major:minor saat start**. OpenCR = FTDI FT232H (`0403:6014`);
   tiap re-flash / cabut-colok / **ganti board** (di robot ini serial `FT3WKHM6` &
   `FT8J0QK9`), ia dapat **minor baru** di host, tapi node di container beku di minor lama
   → `open()` gagal. Bind-mount `/dev/serial/by-id` juga ikut stale/kosong.
2. **Port di-hardcode ke satu board.** `OP3.robot`, `profiles.sh`, dan
   `op3_manager.launch.py` dulu menaruh path by-id satu board
   (`usb-FTDI_..._FT8J0QK9-if00-port0`) → cuma cocok untuk board itu.

---

## 3. Strategi fix — satu alias stabil `/dev/ttyOP3`

`/dev/ttyOP3` **bukan port fisik** — ia **symlink/alias** yang tiap launch diarahkan
otomatis ke `/dev/ttyUSB*` mana pun yang benar-benar hidup.

> Analogi: seperti **kontak HP**. Nomor asli OpenCR (`ttyUSB0/1/2…`) berubah-ubah; kamu
> simpan sebagai kontak `"OP3"`. Tiap boot, provisioner update kontak `"OP3"` ke nomor yang
> benar. Semua software cukup telepon `"OP3"`, tak peduli nomor aslinya.

**Provisioner** (`ensure_opencr_port`): jalan sebagai root di container privileged →
scan `/dev/ttyUSB0..15`, bikin ulang node yang hilang (`mknod`, major 188), buka tiap node
`O_RDWR` untuk cari yang live (minor mati balas `ENXIO`), lalu arahkan symlink `/dev/ttyOP3`
ke situ. Idempotent, aman tiap launch, no-op diam kalau bukan di hardware.

Hasil: ganti board mana pun / minor berapa pun → cukup restart launcher, **tanpa langkah
manual, tanpa restart container, tanpa sudo**.

---

## 4. Peta file (end-state yang dituju)

| File | Peran | Target |
|---|---|---|
| `src/ROBOTIS-OP3/op3_manager/config/OP3.robot` | dipakai controller (manager + editor) | semua baris port = `/dev/ttyOP3` |
| `scripts/op3_tmux/lib/ensure_opencr_port.py` | **provisioner mandiri** (stdlib) | scan 0..15 → symlink `/dev/ttyOP3` |
| `scripts/op3_tmux/lib/profiles.sh` (profil `real_robot`) | launcher Bascorro Studio | `MANAGER_CMD` panggil provisioner dulu, lalu `device_name:=/dev/ttyOP3` |
| `src/ROBOTIS-OP3/op3_manager/launch/op3_manager.launch.py` | entry-point `ros2 launch` | `device_name_default = '/dev/ttyOP3'` |
| `src/ROBOTIS-OP3-Tools/op3_action_editor/scripts/executor.py` | wrapper action editor | panggil `ensure_opencr_port()` + `device_name_default = '/dev/ttyOP3'` |
| `src/ROBOTIS-OP3-Tools/op3_action_editor/src/main.cpp` | node editor C++ | `SUB_CONTROLLER_DEVICE = "/dev/ttyOP3"`; retry power-on 5×; **tanpa** hack `fopen` |

---

## 5. FASE 0 — Deteksi environment (WAJIB dulu, jangan lewati)

Jalankan, CATAT hasilnya, tentukan environment.

```bash
# 0.1 Docker atau native?
docker ps 2>/dev/null && echo "--> ada docker" || echo "--> tidak ada docker (native?)"

# 0.2 (jika Docker) nama container yang jalan
CONTAINER=$(docker ps --format '{{.Names}}' | head -1); echo "CONTAINER=$CONTAINER"
#     kalau >1, pilih yang benar & set CONTAINER=<nama> manual.

# 0.3 Device serial di HOST + identitas
ls -l /dev/ttyUSB* /dev/ttyACM* 2>&1
for d in /dev/ttyUSB* /dev/ttyACM*; do [ -e "$d" ] && \
  echo "$d: $(udevadm info -q property -n "$d" 2>/dev/null | grep -E 'ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT' | tr '\n' ' ')"; done

# 0.4 (jika Docker) apa yang dilihat container
[ -n "$CONTAINER" ] && docker exec "$CONTAINER" bash -c 'ls -l /dev/ttyUSB* /dev/ttyACM* 2>&1'

# 0.5 Port yang saat ini di-hardcode
grep -nE "by-id|/dev/tty" src/ROBOTIS-OP3/op3_manager/config/OP3.robot | head
grep -nE "device_name:=|by-id" scripts/op3_tmux/lib/profiles.sh | head
grep -nE "device_name_default|by-id" src/ROBOTIS-OP3/op3_manager/launch/op3_manager.launch.py | head
```

**GATE — lanjut hanya jika:**
- **Docker:** host punya OpenCR (FTDI `0403:6014`) di `ttyUSBX`, TAPI container kosong /
  cuma punya node minor lain, DAN config masih nunjuk by-id/port board tertentu →
  kerjakan FASE 1–6.
- **Native (tanpa Docker):** tak ada lapis "node container basi". Cukup config pakai nama
  stabil + symlink stabil via udev → kerjakan FASE 1, lalu FASE 6 (udev), lewati yang
  khusus Docker.
- Kalau board **tak terdeteksi di host sama sekali**, atau Wizard pun gagal → itu masalah
  lain (kabel/firmware/board). **STOP**, jangan paksa runbook ini; lihat §9.

---

## 6. FASE 1 — `OP3.robot` pakai `/dev/ttyOP3`

```bash
F=src/ROBOTIS-OP3/op3_manager/config/OP3.robot
OLDPORT=$(grep -oE '/dev/serial/by-id/[^ ]+|/dev/ttyUSB[0-9]+' "$F" | sort -u | grep -v '/dev/ttyOP3' | head -1)
echo "OLDPORT=$OLDPORT"
[ -n "$OLDPORT" ] && sed -i "s#${OLDPORT}#/dev/ttyOP3#g" "$F"
grep -c "/dev/ttyOP3" "$F"          # = jumlah baris port (OP3 standar ~22)
grep -cE "by-id|/dev/ttyUSB" "$F"   # harus 0
```

---

## 7. FASE 2 — Provisioner bersama + action_editor

### (a) Buat provisioner mandiri `scripts/op3_tmux/lib/ensure_opencr_port.py`
Stdlib saja (tak butuh rclpy) supaya bisa dipanggil dari mana pun. Isi = fungsi
`ensure_opencr_port()` di bawah + blok `if __name__ == '__main__':` yang memanggilnya
(`link = sys.argv[1] or '/dev/ttyOP3'`, `max_minor = sys.argv[2] or 15`, selalu `exit 0`).

```python
import os, stat, sys
TTY_MAJOR = 188

def ensure_opencr_port(link_path='/dev/ttyOP3', max_minor=15):
    for minor in range(max_minor + 1):                      # bikin node yang hilang
        dev = f'/dev/ttyUSB{minor}'
        if not os.path.exists(dev):
            try: os.mknod(dev, 0o666 | stat.S_IFCHR, os.makedev(TTY_MAJOR, minor))
            except OSError: pass
    live = None                                             # cari node yang benar2 kebuka
    for minor in range(max_minor + 1):
        dev = f'/dev/ttyUSB{minor}'
        try:
            fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError:
            continue
        os.close(fd); live = dev; break
    if live is None:
        print(f'ensure_opencr_port: no live /dev/ttyUSB* found; leaving {link_path} as-is', file=sys.stderr)
        return False
    try:
        if os.path.realpath(link_path) == live: return True
        if os.path.islink(link_path) or os.path.exists(link_path): os.remove(link_path)
        os.symlink(live, link_path)
        print(f'ensure_opencr_port: {link_path} -> {live}', file=sys.stderr)
        return True
    except OSError as e:
        print(f'ensure_opencr_port: could not set {link_path}: {e}', file=sys.stderr)
        return False
```

### (b) `executor.py` (action editor)
- Pastikan `import stat` ada, dan fungsi `ensure_opencr_port()` versi Python-nya ada
  (boleh copy yang di atas, atau tetap versi lokal — dua-duanya identik).
- Panggil di awal `main()` sebelum blok `params = [...]` (skip saat gazebo):
  ```python
  if not gazebo_default:
      ensure_opencr_port()
  ```
- Ubah default device: cari `device_name_default = '/dev/ttyUSB0'` → **`'/dev/ttyOP3'`**.
  (Baris ini juga dioper ke node C++ lewat `-p device_name:=...`.)

### (c) `main.cpp` (action editor C++)
- `const std::string SUB_CONTROLLER_DEVICE = "/dev/ttyUSB0";` → **`"/dev/ttyOP3";`**
- **HAPUS** hack auto-detect lama `findOpenCRDevice()` + `#include <cstdio>` + blok
  pemanggilnya di `main()` (digantikan `/dev/ttyOP3`; lihat §10 kenapa).
- **PERTAHANKAN** retry power-on 5× di `turnOnDynamixelPower()` (robustness; sama seperti
  `op3_manager`). Pola: `while (torque_on_count < 5) { write1ByteTxRx(...); if(ok) return true; else count++; }`.

---

## 8. FASE 3 — `op3_manager` ikut `/dev/ttyOP3` (INI yang dulu sering kelupaan)

`op3_manager` juga baca `OP3.robot` **dan** punya blok power-on/LED sendiri di awal
`main()` yang pakai parameter `device_name`. Kalau `device_name` masih by-id → blok itu
gagal (`Error Set port`, `Torque on DXLs! [RxPacketError]`, `Fail to control LED`) walau
controller-nya jalan. `op3_manager` **tidak** bikin `/dev/ttyOP3` sendiri, jadi harus
diprovisioning **sebelum** ia start.

### (a) `scripts/op3_tmux/lib/profiles.sh` — profil `real_robot`, baris `MANAGER_CMD`
Sisipkan provisioner di depan (dirangkai `;`, karena `wrap_cmd` jalankan via `bash -lc`),
dan set device ke `/dev/ttyOP3`:
```
MANAGER_CMD="python3 $WS/scripts/op3_tmux/lib/ensure_opencr_port.py; ros2 run op3_manager op3_manager --ros-args ... -p device_name:=/dev/ttyOP3 -p baud_rate:=2000000 -p simulation:=false"
```
> Jangan pakai `${OP3_SERIAL_DEVICE:-...}` untuk `device_name` — env itu di-set ke by-id
> oleh `docker run`, jadi akan menimpa balik ke by-id. Hardcode `/dev/ttyOP3` saja.

### (b) `op3_manager/launch/op3_manager.launch.py`
`device_name_default = '/dev/serial/by-id/...'` → **`'/dev/ttyOP3'`** (konsistensi entry
`ros2 launch`). Perubahan launch.py baru aktif setelah rebuild `op3_manager`.

---

## 9. FASE 4 — Rebuild atau tidak?

```bash
ls -l install/op3_manager/share/op3_manager/config/OP3.robot 2>/dev/null
ls -l install/op3_action_editor/lib/op3_action_editor/executor.py 2>/dev/null
```
- **`OP3.robot`, `executor.py`, `profiles.sh`, `ensure_opencr_port.py`** → script/config.
  Kalau install `--symlink-install` (symlink ke `src/…`) → **langsung berlaku, tanpa
  rebuild**. `profiles.sh` selalu langsung (host-mounted).
- **`main.cpp` (C++) & `op3_manager.launch.py`** → **wajib rebuild di dalam container**:
  ```bash
  cd /ros2_ws
  colcon build --packages-select op3_action_editor op3_manager
  source install/setup.bash
  ```
  > Jangan build di HOST: folder `build/`/`log/` sering dimiliki `root` (sisa build dari
  > dalam container) → `Permission denied`. Build DI DALAM container.

---

## 10. FASE 5 — Verifikasi

**Action editor:**
```bash
ros2 run op3_action_editor executor.py
```
Sukses bila: `ensure_opencr_port: /dev/ttyOP3 -> /dev/ttyUSBn` → `Power on DXLs!` →
`(/dev/ttyOP3) [ID: N] XM430-W350 added.` … `[ID:200] OPEN-CR added.`, **tanpa** `SETUP
ERROR` / `Error Set port`.

**op3_manager / Bascorro Studio (profil `real_robot`):** restart Studio. Pane manager
harus tampil `ensure_opencr_port: /dev/ttyOP3 -> ...` lalu `Torque on DXLs!` (bukan
`Error Set port` / `Fail to control LED`), LED board jadi hijau.

**Tes swap (bukti fix):** cabut board, colok board satunya (atau sama), jalankan lagi
**tanpa langkah manual** → tetap kebaca. Kalau iya → selesai.

> `Segmentation fault` saat "Terminate Action Editor" (waktu KELUAR) = bug exit-time
> terpisah, TIDAK terkait port. Abaikan.

---

## 11. FASE 6 — (Opsional) permanen global: udev + `-v /dev:/dev`

Perlu bila: (a) robot native tanpa Docker, atau (b) mau semua tool beres tanpa menaruh
provisioner di tiap launcher. Butuh `sudo` di host.

```bash
# udev: OpenCR -> /dev/ttyOP3 (host). Batasi per-serial kalau ada FTDI lain.
sudo tee /etc/udev/rules.d/99-op3-opencr.rules >/dev/null <<'EOF'
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", SYMLINK+="ttyOP3", MODE="0666"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/ttyOP3   # harus -> ttyUSBn
```
Lalu (Docker) tambah `-v /dev:/dev` ke array `cmd` di `action_docker_run`
(`scripts/op3_tmux/lib/actions.sh`; container sudah `--privileged`), restart container
sekali. Setelah ini `/dev/ttyOP3` global & FASE 2/3-provisioner tak wajib lagi.

---

## 12. Lapisan robustness sekunder: retry power-on (BUKAN akar fix)

`turnOnDynamixelPower` (editor) & blok power-on `op3_manager` mencoba tulis power 5×
sebelum menyerah. Ini membantu komunikasi yang sesekali *flaky*, **tapi bukan** penyembuh
akar masalah — kalau port yang dibuka salah, 5× pun tetap gagal. Akar fix = `/dev/ttyOP3`.
Simpan retry sebagai bonus, jangan andalkan sebagai solusi utama.

---

## 13. Checklist debugging cepat (kalau masih error)

- [ ] **Device ada di host?** `ls -l /dev/ttyUSB*` — dan identitas FTDI `0403:6014`?
- [ ] **Container melihatnya?** `docker exec $CONTAINER ls -l /dev/ttyUSB*` (sering ini yang basi)
- [ ] **Symlink kebentuk?** di container: `ls -l /dev/ttyOP3` → harus `-> /dev/ttyUSBn`
- [ ] **Provisioner jalan sebagai root?** (butuh privileged untuk `mknod`) — cek log `ensure_opencr_port: ...`
- [ ] **Akses user (native/host)?** `id | grep dialout`
- [ ] **Port tak dipakai proses lain?** `lsof /dev/ttyUSBn`
- [ ] **Board sehat?** test Dynamixel Wizard / Arduino IDE — kalau jalan, hardware OK
- [ ] **C++ sudah di-rebuild di container?** (`main.cpp`/`launch.py` butuh rebuild)
- [ ] **Config bukan simulation?** pastikan `simulation:=false` untuk real robot

Bypass cepat untuk isolasi (paksa satu device): 
`ros2 run op3_action_editor executor.py --ros-args -p device_name:=/dev/ttyUSB1`

---

## 14. ⛔ Yang SUDAH USANG — jangan diulang (dan alasannya)

Pendekatan lama (dari `TROUBLESHOOT_OPENCR_POWER.md`) yang **diganti**:

1. **"Board lama rusak / firmware incompatible → discard board."** ❌ SALAH.
   Board terbukti sehat di Dynamixel Wizard. Gejalanya murni node serial basi di
   container, bukan hardware/firmware. Jangan buang board atas dasar ini.
2. **Auto-detect via `fopen("/dev/ttyUSBi","r")` di `main.cpp` (`findOpenCRDevice`).**
   ❌ DIGANTI `/dev/ttyOP3`. Alasan: `fopen("r")` bisa "berhasil" pada node yang
   sebenarnya tak bisa I/O serial, tidak membuat node yang hilang, dan tiap tool jalan
   sendiri-sendiri (tak seragam). `ensure_opencr_port` (mknod + probe `O_RDWR` + symlink
   satu nama) lebih benar & konsisten dipakai semua tool.
3. **Ubah baudrate (2000000 → lebih rendah) untuk "memperbaiki" `Error Set port`.**
   ❌ Tidak relevan — `Error Set port` = gagal `open()` port, bukan negosiasi baud.

---

## 15. Rollback

```bash
git checkout -- src/ROBOTIS-OP3/op3_manager/config/OP3.robot \
                src/ROBOTIS-OP3-Tools/op3_action_editor/scripts/executor.py \
                src/ROBOTIS-OP3-Tools/op3_action_editor/src/main.cpp \
                src/ROBOTIS-OP3/op3_manager/launch/op3_manager.launch.py \
                scripts/op3_tmux/lib/profiles.sh
rm -f scripts/op3_tmux/lib/ensure_opencr_port.py
# kalau sempat bikin udev rule:
sudo rm -f /etc/udev/rules.d/99-op3-opencr.rules && sudo udevadm control --reload-rules
```

---

## 16. Ceklist selesai
- [ ] FASE 0 dijalankan, GATE cocok, `CONTAINER`/`OLDPORT` terisi dari robot INI.
- [ ] `OP3.robot`: 0 sisa by-id/ttyUSB, semua `/dev/ttyOP3`.
- [ ] `ensure_opencr_port.py` ada & lolos `python3 -m py_compile`.
- [ ] `executor.py`: `ensure_opencr_port()` dipanggil + `device_name_default='/dev/ttyOP3'`.
- [ ] `main.cpp`: default `/dev/ttyOP3`, hack `fopen` dibuang, retry 5× tetap ada.
- [ ] `profiles.sh`: MANAGER_CMD provisioning + `device_name:=/dev/ttyOP3`.
- [ ] `op3_manager.launch.py`: default `/dev/ttyOP3`.
- [ ] Rebuild C++ (`op3_action_editor` + `op3_manager`) DI DALAM container.
- [ ] `executor.py` & manager lolos tanpa `SETUP ERROR`/`Error Set port`.
- [ ] Tes swap board tanpa langkah manual → tetap kebaca.

---

## 17. Referensi
- Kode: `op3_manager/src/op3_manager.cpp` (blok power-on + retry ~baris 258–274 = pola
  referensi retry; **jangan diedit**), `op3_action_editor/src/main.cpp`,
  `.../executor.py`, `scripts/op3_tmux/lib/{profiles.sh,actions.sh,commands.sh}`.
- Memory notes: `reference_container_serial_stale_node.md` (akar + fix),
  `feedback_avoid_op3_manager_init_code.md` (kenapa jangan edit op3_manager.cpp),
  `reference_container_paths.md` (mount host↔container), `project_walking_demo_architecture.md`.
```
