#!/usr/bin/env bash
#
# Satu-satunya cara yang benar untuk menjalankan container OP3 di robot ini.
#
# Kenapa script ini ada (2026-08-11): perintah lama di README memakai
# `--device=/dev/ttyUSB0`. Docker menyalin major:minor perangkat itu SEKALI saat
# container start lalu membekukannya. U2D2 (FTDI FT232H) di robot ini
# re-enumerate terus -- 63 kali dalam satu sore -- dan setiap kali dia pindah
# antara ttyUSB0/ttyUSB1, node di dalam container jadi basi. Gejalanya menipu:
#
#   * "Error opening serial port" / "PORT [...] SETUP ERROR!"  -> node hilang
#   * ATAU tidak ada error sama sekali: manager jalan normal, gait berputar,
#     goal_joint_states berayun, tapi robot diam dan present_joint_states beku
#     bit-for-bit -- fd-nya mati diam-diam (ENODEV) setelah perangkat pindah.
#
# `-v /dev:/dev` membuat /dev container mengikuti host secara live, jadi seluruh
# kelas masalah itu hilang. `--privileged` diperlukan supaya cgroup device tidak
# memblokir akses (sudah diuji: --privileged saja TIDAK cukup, /dev/ttyUSB* dan
# /dev/serial tidak muncul tanpa bind mount-nya).
#
# Container juga dijalankan detached dengan `sleep infinity`, bukan `-it bash`.
# Dengan `-it bash` PID 1 adalah shell-mu: menutup terminal mengirim SIGHUP,
# container mati (exit 129), dan `--rm` menghapusnya berikut semua yang jalan di
# dalamnya. Itu terjadi dua kali dalam satu sore. Sekarang umur container tidak
# lagi terikat pada satu jendela terminal.
#
# Pemakaian:
#   scripts/op3_docker.sh up       # nyalakan container (idempoten)
#   scripts/op3_docker.sh shell    # buka shell baru di dalamnya (boleh berkali-kali)
#   scripts/op3_docker.sh doctor   # periksa colokan + node + kecocokan OP3.robot
#   scripts/op3_docker.sh match-mode on|off|status
#                                  # kurung desktop ke inti 3,7 supaya YOLO dapat
#                                  # inti 0-2 bersih (lihat MATCH_MODE_* di bawah)
#   scripts/op3_docker.sh cpu-thieves
#                                  # SELAMA demo jalan: siapa yang duduk di inti
#                                  # sibling milik detektor dan memperlambatnya
#   scripts/op3_docker.sh down     # matikan dan hapus container

set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NAME="${OP3_CONTAINER_NAME:-op3}"
IMAGE="${OP3_IMAGE:-op3-webots-ros2:humble}"
ROBOT_FILE="$WS/src/ROBOTIS-OP3/op3_manager/config/OP3.robot"

# Isolasi jaringan ROS 2. Alasan lengkapnya ada di docker-entrypoint.sh: di LAN
# lab ada robot lain di domain 0, dan graph-nya menyatu dengan punya kita --
# perintah modul mereka menyapu kepala robot ini di tengah demo. Ubah lewat
# OP3_ROS_DOMAIN_ID / OP3_ROS_LOCALHOST_ONLY kalau memang perlu.
ROS_DOMAIN="${OP3_ROS_DOMAIN_ID:-42}"
ROS_LOCALHOST="${OP3_ROS_LOCALHOST_ONLY:-1}"

BOLD=$'\033[1m'; RED=$'\033[31m'; GREEN=$'\033[32m'; YELLOW=$'\033[33m'; RST=$'\033[0m'
ok()   { echo "${GREEN}  ok${RST}   $*"; }
warn() { echo "${YELLOW}  warn${RST} $*"; }
bad()  { echo "${RED}  gagal${RST} $*"; }

# Pakai sudo hanya kalau user tidak punya akses ke docker socket.
docker_cmd() {
  if docker info >/dev/null 2>&1; then echo "docker"; else echo "sudo docker"; fi
}
DOCKER="$(docker_cmd)"

container_running() {
  [ -n "$($DOCKER ps -q --filter "name=^${NAME}$" 2>/dev/null)" ]
}

container_exists() {
  [ -n "$($DOCKER ps -aq --filter "name=^${NAME}$" 2>/dev/null)" ]
}

# by-id yang dipakai OP3.robot -- inilah yang harus benar-benar ada, bukan
# /dev/ttyUSBx, karena nomor minor-nya berubah tiap re-enumerate.
robot_file_ports() {
  [ -f "$ROBOT_FILE" ] || return 0
  grep -o '/dev/serial/by-id/[^ |]*' "$ROBOT_FILE" | sort -u
}

# Latency timer FTDI. INI BUKAN detail sepele -- ini yang menentukan gerakan
# robot mulus atau patah-patah.
#
# U2D2 memakai chip FTDI, dan driver ftdi_sio menahan byte yang masuk sampai
# buffer penuh ATAU latency timer habis. Bawaan Linux: 16 ms. Sementara
# OP3.robot menyetel control_cycle = 8 ms.
#
# Terukur di ALPHONSE (sync-read present_position 20 servo, 120 sampel):
#     latency_timer 16 -> median 16,00 ms   (100% siklus MELEBIHI 8 ms)
#     latency_timer  1 -> median  3,00 ms   (0% melebihi)
#
# Efeknya di robotis_controller: timerThread menambah 8 ms ke jadwal, memanggil
# process(), lalu kalau ternyata sudah lewat ia MENYETEL ULANG jadwal ke
# "sekarang + 3 ms" (robotis_controller.cpp:733-735). Jadi siklus kendali
# menjadi tidak rata -- kadang 8 ms, kadang overrun lalu 3 ms. Trajektori kepala
# dan kaki dihitung dengan asumsi langkah 8 ms yang TETAP, sehingga langkahnya
# maju dengan laju jam dinding yang tidak rata. Itulah yang terlihat sebagai
# gerakan patah-patah.
#
# Nilainya hilang tiap kali U2D2 dicabut/dicolok, jadi disetel ulang di sini
# tiap `up` dan `doctor`. Untuk permanen, pasang aturan udev di host:
#   echo 'ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"' \
#     | sudo tee /etc/udev/rules.d/99-dynamixelsdk-usb.rules
#   sudo udevadm control --reload-rules && sudo udevadm trigger --action=add
FTDI_LATENCY_TARGET=1

# Tulis $2 ke berkas sysfs $1. Coba langsung dulu (kalau dijalankan sebagai
# root), lalu lewat container yang privileged. Menggemakan "" kalau dua-duanya
# gagal, supaya pemanggil bisa menyarankan sudo.
write_sysfs() {
  local path="$1" val="$2"
  if [ -w "$path" ] && echo "$val" > "$path" 2>/dev/null; then
    echo host; return 0
  fi
  if container_running && $DOCKER exec "$NAME" sh -c "echo $val > $path" 2>/dev/null; then
    echo container; return 0
  fi
  echo ""
}

# Pastikan tiap port FTDI yang dipakai OP3.robot memakai latency timer 1 ms.
# quiet=1 -> hanya lapor kalau ada yang perlu diperhatikan.
ensure_ftdi_latency() {
  local quiet="${1:-0}" tty path cur via any=0
  for tty in $(ls /sys/bus/usb-serial/devices/ 2>/dev/null); do
    path="/sys/bus/usb-serial/devices/$tty/latency_timer"
    [ -r "$path" ] || continue
    any=1
    cur="$(cat "$path" 2>/dev/null || echo '?')"
    if [ "$cur" = "$FTDI_LATENCY_TARGET" ]; then
      [ "$quiet" = "1" ] || ok "$tty latency_timer = ${cur} ms"
      continue
    fi
    via="$(write_sysfs "$path" "$FTDI_LATENCY_TARGET")"
    if [ -n "$via" ]; then
      ok "$tty latency_timer ${cur} -> ${FTDI_LATENCY_TARGET} ms (lewat $via)"
    else
      bad "$tty latency_timer = ${cur} ms, seharusnya ${FTDI_LATENCY_TARGET}."
      bad "control_cycle 8 ms TIDAK mungkin dipenuhi -- gerakan akan patah-patah."
      bad "perbaiki: echo 1 | sudo tee $path"
    fi
  done
  [ "$any" = "1" ] || { [ "$quiet" = "1" ] || warn "tidak ada perangkat usb-serial FTDI terlihat"; }
}

# --- match mode: kurung desktop ke satu inti fisik ---------------------------
#
# i5-10210U ini 4 inti FISIK / 8 utas, pasangan Hyper-Threading 0<->4, 1<->5,
# 2<->6, 3<->7. Dua utas dalam satu pasangan BERBAGI unit eksekusi, jadi proses
# apa pun yang mendarat di 4/5/6 langsung memperlambat detektor bola yang
# dipatok di 0-2. Terukur pada model ball_yolov8n @640:
#
#   inferensi, inti sibling sepi ....  40,3 ms
#   inferensi, inti sibling sibuk ...  84,2 ms    <- 2,1x lebih lambat
#
# Pesaing terbesar BUKAN node ROS, melainkan desktop: saat diukur, Firefox
# memakai 39% + 19% CPU dan VS Code sekitar 35%. `taskset` di berkas launch
# hanya mengatur proses ROS; desktop tetap bebas berkeliaran ke mana saja.
# Perintah ini mengurung SELURUH sesi pengguna (Firefox, VS Code, GNOME, Xorg)
# ke inti fisik P3 lewat cgroup v2, menyisakan P0/P1/P2 utuh untuk YOLO.
#
# Sengaja perintah TERPISAH, bukan otomatis di `up`: kalau selalu menyala,
# desktop terasa berat saat sedang ngoding. Nyalakan menjelang bertanding.
#
# `--runtime` berarti setelan ini HILANG SENDIRI saat reboot -- environment
# permanen tidak diubah sama sekali.
MATCH_MODE_SLICE="user.slice"
MATCH_MODE_CPUS="3,7"      # inti untuk desktop; kebalikan dari cpu_affinity YOLO
MATCH_MODE_ALL="0-7"

match_mode_current() {
  cat "/sys/fs/cgroup/${MATCH_MODE_SLICE}/cpuset.cpus.effective" 2>/dev/null || echo '?'
}

# Menerapkan AllowedCPUs. Butuh root: coba langsung, lalu lewat sudo non-interaktif.
match_mode_set() {
  local cpus="$1"
  if systemctl set-property --runtime "$MATCH_MODE_SLICE" "AllowedCPUs=$cpus" 2>/dev/null; then
    return 0
  fi
  if sudo -n systemctl set-property --runtime "$MATCH_MODE_SLICE" "AllowedCPUs=$cpus" 2>/dev/null; then
    return 0
  fi
  return 1
}

cmd_match_mode() {
  local action="${1:-status}"
  case "$action" in
    status)
      local cur; cur="$(match_mode_current)"
      if [ "$cur" = "$MATCH_MODE_CPUS" ]; then
        ok "match mode NYALA - desktop terkurung di inti $cur, YOLO dapat 0-2 bersih"
      else
        warn "match mode MATI - desktop boleh memakai inti $cur"
        warn "inti 4,5,6 adalah sibling HT milik YOLO. Nyalakan dengan salah satu:"
        warn "  ./script.sh --match-mode on      (atau lewat menu: Match mode)"
        warn "  scripts/op3_docker.sh match-mode on"
      fi
      ;;
    on)
      if match_mode_set "$MATCH_MODE_CPUS"; then
        ok "desktop dikurung ke inti $MATCH_MODE_CPUS (sekarang: $(match_mode_current))"
        echo "     inti 0-2 kini bersih untuk detektor bola; hilang sendiri saat reboot."
      else
        bad "gagal menerapkan (butuh root). Jalankan:"
        bad "  sudo systemctl set-property --runtime $MATCH_MODE_SLICE AllowedCPUs=$MATCH_MODE_CPUS"
      fi
      ;;
    off)
      if match_mode_set "$MATCH_MODE_ALL"; then
        ok "desktop dilepas kembali ke inti $(match_mode_current)"
      else
        bad "gagal menerapkan (butuh root). Jalankan:"
        bad "  sudo systemctl set-property --runtime $MATCH_MODE_SLICE AllowedCPUs=$MATCH_MODE_ALL"
      fi
      ;;
    *)
      echo "Pemakaian: $0 match-mode {on|off|status}" >&2
      return 1
      ;;
  esac
}

# --- laporan kamera: pajan membatasi FPS ------------------------------------
#
# Gejala "FPS turun" nyaris selalu ditebak sebagai masalah CPU. Sering BUKAN.
# Kamera UVC boleh MEMPERLAMBAT DIRI supaya sensornya sempat memajan lebih lama
# di tempat gelap, dan tidak ada optimasi komputasi yang bisa menambah frame
# yang memang tidak pernah dikirim:
#
#     FPS maksimum = 1 / waktu_pajan
#
# Terukur di robot ini saat FPS turun ke 14-15: pajan 66,6 ms -> tepat 15,0 FPS.
#
# Bagian ini HANYA MELAPOR, tidak menyetel. Penyetelnya satu dan ada di
# launch/yolo_ball_detector_openvino.launch.py (v4l2-ctl
# exposure_dynamic_framerate=0), supaya tidak ada dua tempat yang bisa berbeda
# pendapat soal keadaan kamera.
report_camera_exposure() {
  local dev="${1:-/dev/video0}"
  if [ ! -e "$dev" ]; then
    warn "$dev tidak ada -- kamera tidak terpasang?"
    return 0
  fi
  if ! command -v v4l2-ctl >/dev/null 2>&1; then
    warn "v4l2-ctl tidak ada di host; lewati pemeriksaan pajan kamera"
    return 0
  fi
  local dyn exp gain
  dyn="$(v4l2-ctl -d "$dev" -C exposure_dynamic_framerate 2>/dev/null | awk '{print $2}')"
  exp="$(v4l2-ctl -d "$dev" -C exposure_time_absolute 2>/dev/null | awk '{print $2}')"
  gain="$(v4l2-ctl -d "$dev" -C gain 2>/dev/null | awk '{print $2}')"

  if [ -n "$exp" ] && [ "$exp" -gt 0 ] 2>/dev/null; then
    # exposure_time_absolute satuannya 100 us.
    local cap
    cap="$(awk -v e="$exp" 'BEGIN{printf "%.1f", 10000.0/e}')"
    echo "     pajan ${exp} (=$(awk -v e="$exp" 'BEGIN{printf "%.1f", e/10.0}') ms) -> batas FPS ${cap}"
  fi

  case "$dyn" in
    0) ok "exposure_dynamic_framerate = 0 (kamera tidak boleh memperlambat diri)" ;;
    1) bad "exposure_dynamic_framerate = 1 -- kamera BOLEH menurunkan laju di tempat gelap."
       bad "itu membatasi FPS di sumbernya, bukan di CPU. Jalankan detektor lewat"
       bad "launch-nya (yang menyetel ini ke 0), atau manual:"
       bad "  v4l2-ctl -d $dev -c exposure_dynamic_framerate=0" ;;
    *) warn "exposure_dynamic_framerate tidak terbaca (kamera mungkin tidak punya kontrol ini)" ;;
  esac

  if [ "$gain" = "255" ]; then
    warn "gain = 255 (MENTOK) -- sensor tak punya cadangan cahaya lagi;"
    warn "memperpendek pajan pasti menggelapkan gambar. Tambah penerangan lapangan."
  fi
}

cmd_doctor() {
  echo "${BOLD}1. Perangkat di host${RST}"
  local ftdi
  ftdi="$(ls /dev/serial/by-id/ 2>/dev/null | grep -i ftdi || true)"
  if [ -n "$ftdi" ]; then
    ok "U2D2 terpasang: $ftdi -> $(readlink -f "/dev/serial/by-id/$ftdi")"
  else
    bad "U2D2 TIDAK ada di /dev/serial/by-id -- colokannya lepas."
  fi
  if ls /dev/serial/by-id/ 2>/dev/null | grep -qi opencr; then
    ok "OpenCR terpasang"
  else
    warn "OpenCR tidak terlihat di by-id"
  fi
  # Lihat komentar di atas ensure_ftdi_latency(): 16 ms bawaan vs control_cycle
  # 8 ms adalah selisih yang membuat gerakan patah-patah.
  ensure_ftdi_latency
  # Desktop yang berkeliaran ke inti 4,5,6 memperlambat YOLO 2,1x (lihat komentar
  # di atas cmd_match_mode). Cuma dilaporkan di sini, tidak dipaksa.
  cmd_match_mode status
  # Pajan kamera membatasi FPS di SUMBERNYA -- lihat komentar di atas
  # report_camera_exposure. Cuma dilaporkan, tidak dipaksa.
  report_camera_exposure /dev/video0

  echo
  echo "${BOLD}2. Kestabilan colokan (kernel, 30 menit terakhir)${RST}"
  local drops
  drops="$(journalctl -k --since '-30min' --no-pager 2>/dev/null | grep -c 'USB disconnect' || true)"
  if [ "${drops:-0}" -eq 0 ]; then
    ok "tidak ada USB disconnect"
  elif [ "${drops:-0}" -le 2 ]; then
    warn "$drops disconnect -- perhatikan"
  else
    bad "$drops disconnect dalam 30 menit. Kabel/konektor U2D2 bermasalah;"
    bad "ini akan memutus bus di tengah walking, berapa kali pun software diperbaiki."
  fi

  echo
  echo "${BOLD}3. Port yang diminta OP3.robot${RST}"
  local found=0
  while read -r port; do
    [ -n "$port" ] || continue
    found=1
    if [ -e "$port" ]; then
      ok "$(basename "$port") -> $(readlink -f "$port")"
    else
      bad "$(basename "$port") TIDAK ada di host."
      bad "  cek 'ls /dev/serial/by-id/' lalu samakan OP3.robot dengan yang benar-benar tercolok."
    fi
  done < <(robot_file_ports)
  [ "$found" -eq 1 ] || warn "tidak ada path by-id di $ROBOT_FILE"

  echo
  echo "${BOLD}4. Di dalam container${RST}"
  if ! container_running; then
    warn "container '$NAME' tidak jalan -- jalankan: scripts/op3_docker.sh up"
    return 0
  fi
  if $DOCKER inspect "$NAME" --format '{{range .Mounts}}{{.Destination}}{{"\n"}}{{end}}' 2>/dev/null | grep -qx '/dev'; then
    ok "/dev di-bind dari host (node mengikuti host secara live)"
  else
    bad "/dev TIDAK di-bind. Container ini dijalankan dengan perintah lama;"
    bad "  node perangkatnya akan basi begitu U2D2 pindah. Jalankan ulang:"
    bad "  scripts/op3_docker.sh down && scripts/op3_docker.sh up"
  fi
  while read -r port; do
    [ -n "$port" ] || continue
    if $DOCKER exec "$NAME" test -e "$port" 2>/dev/null; then
      if $DOCKER exec "$NAME" python3 -c "
import os, sys
try:
    fd = os.open(sys.argv[1], os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK); os.close(fd)
except OSError as exc:
    print(exc); sys.exit(1)
" "$port" >/dev/null 2>&1; then
        ok "$(basename "$port") bisa dibuka dari dalam container"
      else
        bad "$(basename "$port") ada tapi GAGAL dibuka (node basi / perangkat lepas)"
      fi
    else
      bad "$(basename "$port") tidak terlihat di dalam container"
    fi
  done < <(robot_file_ports)

  # "Bisa dibuka" BUKAN berarti bebas: SDK membuka serial tanpa TIOCEXCL, jadi
  # proses kedua tetap sukses open() lalu berebut byte dengan proses lama --
  # errornya kemudian menuduh perangkat ("Failed to turn on the Power of DXLs!",
  # "first bulk read fail!!") padahal perangkatnya sehat. Pemegangnya juga sulit
  # diburu sendiri: comm dipotong 15 karakter jadi 'op3_action_edit', sehingga
  # 'pgrep -x op3_action_editor' selalu kosong.
  local guard=/ros2_ws/scripts/op3_port_users.py holders_out
  if ! $DOCKER exec "$NAME" test -f "$guard" 2>/dev/null; then
    warn "op3_port_users.py tidak ada di container -- lewati cek pemegang port"
  elif holders_out="$($DOCKER exec "$NAME" python3 "$guard" --brief 2>/dev/null)"; then
    ok "tidak ada proses lain yang memegang port robot"
  else
    bad "port robot sedang dipegang proses lain:"
    printf '%s\n' "$holders_out" | sed 's/^/  /'
    bad "  matikan: scripts/op3_docker.sh shell -> scripts/op3_port_users.py --kill"
  fi

  echo
  echo "${BOLD}5. Workspace di dalam container${RST}"
  # Regresi yang pernah menipu berjam-jam: `docker exec` melewati ENTRYPOINT,
  # shell lahir tanpa ROS, dan tiap `ros2 run ...` cuma menjawab "command not
  # found" atau "Package not found" -- seolah paketnya hilang. Diuji dari shell
  # BARU, bukan dari env PID 1, karena itu yang benar-benar kamu pakai.
  if $DOCKER exec "$NAME" /docker-entrypoint.sh bash -c 'command -v ros2' >/dev/null 2>&1; then
    ok "ros2 ada di PATH lewat entrypoint (dipakai 'op3_docker.sh shell')"
  else
    bad "ros2 TIDAK ketemu lewat entrypoint -- periksa docker-entrypoint.sh"
  fi
  if $DOCKER exec "$NAME" bash -i -c 'command -v ros2' >/dev/null 2>&1; then
    ok "ros2 ada di PATH untuk 'docker exec -it $NAME bash' biasa"
  else
    warn "shell polos belum punya ROS -- jalankan 'scripts/op3_docker.sh up' sekali lagi"
  fi
  local pkg missing=0
  for pkg in op3_action_editor op3_manager op3_action_module; do
    if ! $DOCKER exec "$NAME" /docker-entrypoint.sh bash -c \
         "ros2 pkg prefix $pkg" >/dev/null 2>&1; then
      bad "paket '$pkg' tidak ketemu -- workspace belum di-build di dalam container"
      missing=1
    fi
  done
  [ "$missing" -eq 0 ] && ok "paket op3 inti terbaca (action_editor, manager, action_module)"

  echo
  echo "${BOLD}6. Isolasi jaringan ROS 2${RST}"
  # Terukur 2026-08-26: dengan op3_manager MATI, topik /robotis/enable_ctrl_module
  # dan /robotis/present_joint_ctrl_modules MASIH menerima pesan -- itu robot lain
  # di LAN yang juga memakai domain 0. Graph-nya menyatu: modul kepala kita disapu
  # ke base_module/none tiap 1,5 detik dan parameter jalan tertimpa punya mereka.
  local dom lhost
  dom="$($DOCKER exec "$NAME" bash -lc 'echo "${ROS_DOMAIN_ID:-0}"' 2>/dev/null | tr -d "\r")"
  lhost="$($DOCKER exec "$NAME" bash -lc 'echo "${ROS_LOCALHOST_ONLY:-0}"' 2>/dev/null | tr -d "\r")"
  if [ "$lhost" = "1" ]; then
    ok "ROS_LOCALHOST_ONLY=1 (DDS terkunci ke loopback) -- domain $dom"
  elif [ "$dom" != "0" ] && [ -n "$dom" ]; then
    ok "ROS_DOMAIN_ID=$dom (bukan 0) -- terpisah dari robot lain, tapi masih lewat LAN"
  else
    bad "shell container lahir di ROS_DOMAIN_ID=${dom:-0} tanpa ROS_LOCALHOST_ONLY."
    bad "  Robot lain di LAN yang juga domain 0 akan mengambil alih modul sendi robot ini."
    bad "  Perbaiki: scripts/op3_docker.sh up   (memasang env-nya di .bashrc container)"
  fi

  echo
  echo "${BOLD}7. Kesegaran build (ABI)${RST}"
  # Jebakan paling mahal yang pernah kena di repo ini (2026-08-12).
  #
  # robotis_controller.h diubah (menambah anggota data ke class RobotisController),
  # lalu HANYA paket robotis_controller yang di-build ulang. Paket lain yang
  # meng-#include header itu masih memakai layout class LAMA: offset anggota dan
  # ukuran object-nya beda dengan .so yang baru. Hasilnya bukan error link dan
  # bukan crash, tapi memori dibaca di alamat yang salah.
  #
  # Gejalanya sama sekali tidak menunjuk ke penyebabnya: op3_action_editor jalan,
  # mencetak daftar port dan 20 servo, berhenti tepat setelah "Load offsets...",
  # lalu berputar 100% CPU selamanya tanpa satu pun pesan error. Butuh gdb untuk
  # menemukannya. Karena itu diperiksa otomatis di sini.
  local hdr="$WS/src/ROBOTIS-Framework/robotis_controller/include/robotis_controller/robotis_controller.h"
  if [ ! -f "$hdr" ]; then
    warn "robotis_controller.h tidak ditemukan -- lewati pemeriksaan"
    return 0
  fi
  local abi_stale=0 src_file pkg pkg_dir artifact
  while IFS= read -r src_file; do
    pkg_dir="$(dirname "$src_file")"
    while [ "$pkg_dir" != "/" ] && [ "$pkg_dir" != "$WS" ]; do
      [ -f "$pkg_dir/package.xml" ] && break
      pkg_dir="$(dirname "$pkg_dir")"
    done
    [ -f "$pkg_dir/package.xml" ] || continue
    pkg="$(basename "$pkg_dir")"
    artifact="$(find "$WS/build/$pkg" -maxdepth 1 \( -name '*.so' -o -type f -perm -u+x \) \
                 ! -name '*.cmake' ! -name '*.txt' ! -name '*.py' 2>/dev/null | head -1)"
    [ -n "$artifact" ] || continue
    if [ "$artifact" -ot "$hdr" ]; then
      bad "'$pkg' dibangun SEBELUM robotis_controller.h berubah -- layout class beda."
      bad "  Perbaiki: colcon build --packages-select $pkg --symlink-install"
      abi_stale=1
    fi
  done < <(grep -rl 'robotis_controller/robotis_controller.h' \
             --include='*.h' --include='*.hpp' --include='*.cpp' "$WS/src" 2>/dev/null)
  [ "$abi_stale" -eq 0 ] && ok "semua pemakai robotis_controller.h dibangun setelah header terakhir berubah"
}

# `docker exec ... bash` melewati ENTRYPOINT, jadi shell lewat jalur itu lahir
# tanpa ROS sama sekali. `shell` di script ini sudah dibungkus entrypoint, tapi
# siapa pun masih bisa mengetik `docker exec -it op3 bash` dari kebiasaan lama.
# Tanam sourcing-nya di .bashrc container supaya SEMUA jalan masuk beres.
# Idempoten: aman dipanggil berkali-kali.
harden_container_shell() {
  # `docker exec ... bash -lc` MELEWATI entrypoint, jadi domain ROS harus
  # dipasang di sini juga -- kalau tidak, shell semacam itu lahir di domain 0
  # dan kembali menyatu dengan robot sebelah. Penanda diberi versi supaya
  # container yang sudah terlanjur punya blok lama ikut diperbarui.
  $DOCKER exec -e "OP3_DOMAIN=$ROS_DOMAIN" -e "OP3_LOCALHOST=$ROS_LOCALHOST" "$NAME" bash -c '
marker="# >>> op3: auto-source ROS v2 (dipasang op3_docker.sh) >>>"
grep -qF "$marker" /root/.bashrc 2>/dev/null && exit 0
cat >> /root/.bashrc <<EOF
$marker
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID="\${ROS_DOMAIN_ID:-$OP3_DOMAIN}"
export ROS_LOCALHOST_ONLY="\${ROS_LOCALHOST_ONLY:-$OP3_LOCALHOST}"
# <<< op3: auto-source ROS v2 <<<
EOF
' >/dev/null 2>&1 || warn "gagal memasang auto-source ROS di .bashrc container"

  # .bashrc saja tidak cukup: `docker exec op3 bash -lc '...'` itu shell LOGIN,
  # dan shell login membaca /etc/profile (-> /etc/profile.d/*.sh), bukan
  # .bashrc. Tanpa berkas ini perintah semacam itu lahir di domain 0 lagi.
  $DOCKER exec -e "OP3_DOMAIN=$ROS_DOMAIN" -e "OP3_LOCALHOST=$ROS_LOCALHOST" "$NAME" bash -c '
cat > /etc/profile.d/op3-ros.sh <<EOF
# Dipasang op3_docker.sh -- jangan diubah manual, akan ditimpa.
export ROS_DOMAIN_ID="\${ROS_DOMAIN_ID:-$OP3_DOMAIN}"
export ROS_LOCALHOST_ONLY="\${ROS_LOCALHOST_ONLY:-$OP3_LOCALHOST}"
EOF
' >/dev/null 2>&1 || warn "gagal memasang /etc/profile.d/op3-ros.sh di container"
}

cmd_up() {
  if container_running; then
    harden_container_shell
    ensure_ftdi_latency 1
    echo "Container '$NAME' sudah jalan."
    echo "Buka shell: scripts/op3_docker.sh shell"
    return 0
  fi
  if container_exists; then
    echo "Menghapus sisa container '$NAME' yang mati..."
    $DOCKER rm -f "$NAME" >/dev/null
  fi
  if ! $DOCKER image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "${RED}Image $IMAGE tidak ada.${RST} Build dulu: docker build -t $IMAGE $WS" >&2
    exit 1
  fi

  local input_gid dialout_gid
  input_gid="$(getent group input | cut -d: -f3)"
  dialout_gid="$(getent group dialout | cut -d: -f3)"

  # X11 diteruskan kalau ada, supaya GUI (rqt_image_view, Webots) tetap bisa
  # tampil. Jalur `--docker-run` lama membawa ini; tanpa disalin ke sini,
  # menyatukan semua ke satu script justru mematikan GUI.
  local -a x11=()
  if [ -n "${DISPLAY:-}" ]; then
    x11+=(-e "DISPLAY=$DISPLAY")
    [ -d /tmp/.X11-unix ] && x11+=(-v /tmp/.X11-unix:/tmp/.X11-unix:rw)
    local xauth="${XAUTHORITY:-$HOME/.Xauthority}"
    [ -f "$xauth" ] && x11+=(-e XAUTHORITY=/tmp/.Xauthority -v "$xauth":/tmp/.Xauthority:rw)
  fi

  # docker-entrypoint.sh di-bind dari source: image masih menyimpan versi lama
  # yang kehilangan argumen CMD (setupvars.sh milik OpenVINO menelan "$@", jadi
  # `exec "$@"` jalan tanpa argumen dan container langsung exit 0). Bind mount
  # ini membuat perbaikannya berlaku tanpa rebuild image.
  # `--restart unless-stopped`: setelah PC reboot container ikut hidup lagi
  # sendiri. Tanpa ini, tiap kali PC dinyalakan ulang container hilang dan semua
  # `ros2 run ...` menjawab "Package not found" -- workspace ini dibangun dengan
  # --symlink-install DI DALAM container, jadi seluruh install/ menunjuk ke
  # /ros2_ws yang cuma ada di sana. Kalau tidak mau auto-start, pakai
  # `scripts/op3_docker.sh down` (itu dihitung "stopped", jadi tidak bangkit).
  # `--init`: PID 1 di container ini `sleep infinity`, dan sleep tidak pernah
  # me-reap anak yatim. Proses ROS yang mati saat terminalnya ditutup jadi
  # menumpuk sebagai <defunct> dan mengotori `pgrep`/`ps` -- sempat bikin salah
  # baca PID waktu mendiagnosis. tini sebagai PID 1 membereskannya sendiri.
  $DOCKER run -d --name "$NAME" \
    --init \
    --restart unless-stopped \
    --net=host --ipc=host \
    --privileged \
    -v /dev:/dev \
    -v "$WS":/ros2_ws \
    -v "$WS/docker-entrypoint.sh":/docker-entrypoint.sh:ro \
    -w /ros2_ws \
    -e "ROS_DOMAIN_ID=$ROS_DOMAIN" \
    -e "ROS_LOCALHOST_ONLY=$ROS_LOCALHOST" \
    ${input_gid:+--group-add "$input_gid"} \
    ${dialout_gid:+--group-add "$dialout_gid"} \
    "${x11[@]}" \
    --ulimit rtprio=99 --ulimit memlock=-1 \
    --cap-add SYS_NICE --cap-add SYS_RESOURCE \
    "$IMAGE" \
    sleep infinity >/dev/null

  sleep 1
  if ! container_running; then
    echo "${RED}Container gagal start.${RST} Lihat: $DOCKER logs $NAME" >&2
    exit 1
  fi
  harden_container_shell
  echo "Container '$NAME' jalan."
  echo
  cmd_doctor
  echo
  echo "Buka shell: ${BOLD}scripts/op3_docker.sh shell${RST}"
}

cmd_shell() {
  container_running || { echo "Container '$NAME' tidak jalan. Jalankan: scripts/op3_docker.sh up" >&2; exit 1; }
  # Shell HARUS lewat /docker-entrypoint.sh. `docker exec` melewati ENTRYPOINT
  # container, jadi shell polos lahir tanpa ROS sama sekali -- `ros2` tidak ada
  # di PATH dan setiap `ros2 run ...` cuma menjawab "command not found". PID 1
  # memang punya env-nya (dia lewat entrypoint waktu start), tapi env itu tidak
  # diwariskan ke proses baru hasil exec. Entrypoint yang men-source
  # /opt/ros/humble + /ros2_ws/install, jadi panggil dia sebagai pembungkus.
  exec $DOCKER exec -it "$NAME" /docker-entrypoint.sh bash
}

# Bebaskan port serial: matikan proses apa pun yang masih memegangnya.
#
# Kenapa harus lewat container dan bukan langsung dari host: stack ini hidup DI
# DALAM container, dan /proc/<pid>/fd milik proses root di sana tidak terbaca
# dari host, jadi jawaban "bersih" versi host menyesatkan. Membuka perangkatnya
# juga bukan bukti apa-apa: SDK membuka serial tanpa TIOCEXCL, sehingga dua
# proses memang boleh memegangnya bersamaan -- justru itu masalahnya.
#
# Pemindainya satu: scripts/op3_port_users.py, yang juga dipakai bagian 4 doctor,
# executor.py, dan health_check launcher tmux.
cmd_free_port() {
  container_running || {
    echo "Container '$NAME' tidak jalan -- tidak ada proses yang perlu dibereskan."
    return 0
  }
  $DOCKER exec "$NAME" test -f /ros2_ws/scripts/op3_port_users.py 2>/dev/null || {
    echo "scripts/op3_port_users.py tidak ada di dalam container." >&2
    return 1
  }
  $DOCKER exec "$NAME" python3 /ros2_ws/scripts/op3_port_users.py --kill
}

# Siapa yang mencuri CPU dari detektor bola selama demo berjalan.
#
# Sengaja dijalankan di HOST, bukan di dalam container: /proc dibagi, jadi dari
# host semua proses container tetap terlihat SEKALIGUS proses desktop -- dan
# desktop justru sering jadi pencuri terbesar. Dijalankan dari dalam container,
# desktop tidak akan terlihat sama sekali.
#
# Pemindainya satu: scripts/op3_cpu_thieves.py. Jangan bikin versi tandingan.
cmd_cpu_thieves() {
  local scanner="$WS/scripts/op3_cpu_thieves.py"
  [ -f "$scanner" ] || { bad "Tidak ditemukan: $scanner"; return 1; }
  if [ -e /.dockerenv ]; then
    warn "Dijalankan DI DALAM container: proses desktop host tidak akan terlihat."
    warn "Untuk gambaran penuh, jalankan dari host: ./script.sh --cpu-thieves"
  fi
  python3 "$scanner" "$@"
}

cmd_down() {
  container_exists || { echo "Container '$NAME' tidak ada."; return 0; }
  $DOCKER rm -f "$NAME" >/dev/null
  echo "Container '$NAME' dihapus."
}

case "${1:-up}" in
  up)        cmd_up ;;
  shell)     cmd_shell ;;
  doctor)    cmd_doctor ;;
  free-port) cmd_free_port ;;
  match-mode) shift; cmd_match_mode "${1:-status}" ;;
  cpu-thieves) shift; cmd_cpu_thieves "$@" ;;
  down)      cmd_down ;;
  *)
    echo "Pemakaian: $0 {up|shell|doctor|free-port|match-mode|cpu-thieves|down}" >&2
    exit 1
    ;;
esac
