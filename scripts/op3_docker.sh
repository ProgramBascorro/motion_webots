#!/usr/bin/env bash
#
# Satu-satunya cara yang benar untuk menjalankan container OP3 di robot ini.
#
# Diambil dari CHRONUS_NEW (7da8234) dan disesuaikan dengan robot ini. Dulu ada
# tiga definisi container yang saling bertentangan -- menu tmux, docker-compose.yml,
# dan perintah `docker run` manual di README. Sekarang cuma di sini; dua yang lain
# tinggal mendelegasikan.
#
# Dua kebiasaan lama yang harus mati:
#
#   * `--device=/dev/ttyUSB0`  Docker menyalin major:minor perangkat SEKALI saat
#     container start lalu membekukannya. FTDI FT232H re-enumerate tiap kali
#     dicabut/di-reset dan bisa pindah antara ttyUSB0/ttyUSB1, sehingga node di
#     dalam container jadi basi. Gejalanya menipu: kadang "PORT [...] SETUP
#     ERROR!", kadang tanpa error sama sekali -- manager jalan, gait berputar,
#     goal_joint_states berayun, tapi robot diam dan present_joint_states beku
#     bit-for-bit karena fd-nya mati diam-diam (ENODEV).
#     `-v /dev:/dev` membuat /dev container mengikuti host secara live, jadi
#     seluruh kelas masalah itu hilang. `--privileged` tetap perlu supaya cgroup
#     device tidak memblokir akses, tapi `--privileged` SAJA tidak cukup: tanpa
#     bind mount, container dapat devtmpfs sendiri yang tidak diisi udev --
#     terbukti di robot ini, /dev/ttyUSB0 muncul tapi symlink /dev/ttyOP3 tidak.
#
#   * `-it --rm ... bash`      PID 1 adalah shell-mu. Menutup terminal mengirim
#     SIGHUP: container mati (exit 129) DAN terhapus, berikut manager, rosbridge,
#     dan studio yang sedang jalan di dalamnya. Di sini container dijalankan
#     detached dengan `sleep infinity`, jadi umurnya tidak terikat satu jendela.
#
# Susunan serial robot ini -- beda dari CHRONUS_NEW, jangan disamakan:
#   Adapter FTDI FT232H (0403:6014) membawa servo ID 1-20 DAN sub controller
#   ID 200 di satu bus TTL yang sama (OP3.robot menaruh keduanya di satu port).
#   Nomor ttyUSB-nya TIDAK dipegang mati: op3_manager.launch.py dan
#   op3_action_editor/scripts/executor.py memindai /dev/ttyUSB0-9, memilih yang
#   benar-benar bisa dibuka, lalu menulis salinan OP3.robot dengan port itu.
#   Jadi yang perlu diperiksa bukan "apakah ttyUSB0 ada", melainkan "apakah ada
#   satu ttyUSB yang bisa dibuka, dan tidak sedang dipegang proses lain".
#   Symlink udev /dev/ttyOP3 ada di host (99-ftdi-op3.rules) tapi stack ini tidak
#   bergantung padanya. Paksa satu port tertentu dengan env OP3_DEVICE.
#
# Pemakaian:
#   scripts/op3_docker.sh up       # nyalakan container (idempoten)
#   scripts/op3_docker.sh shell    # buka shell baru di dalamnya (boleh berkali-kali)
#   scripts/op3_docker.sh doctor   # periksa colokan + container + kesegaran build
#   scripts/op3_docker.sh down     # matikan dan hapus container

set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NAME="${OP3_CONTAINER_NAME:-op3}"
ROBOT_FILE="$WS/src/ROBOTIS-OP3/op3_manager/config/OP3.robot"
CONTAINER_BASHRC="$WS/scripts/container_bashrc"

# Image, urut prioritas. Di robot ini ADA DUA image yang mirip dan gampang
# tertukar; yang lebih tua justru yang namanya lebih rapi:
#   motion_webots_new-op3_ros2:latest  dibangun 2026-07-04 dari Dockerfile yang
#     sudah diadaptasi arm64 -- punya torch(cpu), ultralytics, dan wheel openvino
#     yang dipakai op3_ball_detector (YOLO dari ALPHONSE) dan op3_yolo_vision.
#   op3-webots-ros2:humble             dibangun 2026-06-30, sebelum blok pip YOLO
#     masuk Dockerfile: `import torch` di dalamnya gagal.
# Dipakai yang pertama ketemu. Paksa satu tertentu dengan env OP3_IMAGE.
default_image() {
  local candidate
  for candidate in motion_webots_new-op3_ros2:latest op3-webots-ros2:humble; do
    if $DOCKER image inspect "$candidate" >/dev/null 2>&1; then
      echo "$candidate"
      return 0
    fi
  done
  echo motion_webots_new-op3_ros2:latest
}

BOLD=$'\033[1m'; RED=$'\033[31m'; GREEN=$'\033[32m'; YELLOW=$'\033[33m'; RST=$'\033[0m'
ok()   { echo "${GREEN}  ok${RST}   $*"; }
warn() { echo "${YELLOW}  warn${RST} $*"; }
bad()  { echo "${RED}  gagal${RST} $*"; }

# Pakai sudo hanya kalau user tidak punya akses ke docker socket.
docker_cmd() {
  if docker info >/dev/null 2>&1; then echo "docker"; else echo "sudo docker"; fi
}
DOCKER="$(docker_cmd)"
IMAGE="${OP3_IMAGE:-$(default_image)}"

container_running() {
  [ -n "$($DOCKER ps -q --filter "name=^${NAME}$" 2>/dev/null)" ]
}

container_exists() {
  [ -n "$($DOCKER ps -aq --filter "name=^${NAME}$" 2>/dev/null)" ]
}

# Port yang benar-benar diminta OP3.robot. Komentar dibuang dulu supaya baris
# penjelasan tidak ikut terbaca sebagai port wajib.
robot_file_ports() {
  [ -f "$ROBOT_FILE" ] || return 0
  sed 's/#.*//' "$ROBOT_FILE" | grep -oE '/dev/[^ |]+' | sort -u
}

# Kandidat port yang dipindai stack ini, urut sama seperti
# detect_dynamixel_device() di op3_manager.launch.py: OP3_DEVICE menang, lalu
# /dev/ttyUSB0-9, lalu /dev/ttyOP3.
#
# Disaring dengan realpath: /dev/ttyOP3 hampir selalu symlink ke salah satu
# ttyUSB yang sudah disebut di atas, dan melaporkannya dua kali membuat seolah
# ada dua bus DXL.
dxl_candidates() {
  if [ -n "${OP3_DEVICE:-}" ]; then
    echo "$OP3_DEVICE"
    return 0
  fi
  local dev real
  local seen=" "
  for dev in /dev/ttyUSB{0,1,2,3,4,5,6,7,8,9} /dev/ttyOP3; do
    [ -e "$dev" ] || continue
    real="$(readlink -f "$dev")"
    case "$seen" in *" $real "*) continue ;; esac
    seen="$seen$real "
    echo "$dev"
  done
  return 0
}

# Kamera V4L2. ALPHONSE_NEW menuliskan /dev/video0 dan /dev/video1 satu per satu
# di docker-compose.yml (`devices:`) dan di menu tmux (`--device=/dev/video0`).
# Di sini keduanya sengaja TIDAK ditiru, dua alasan:
#   * `devices:` menolak start kalau perangkatnya belum ada. Robot ini sering
#     hidup tanpa kamera tercolok (waktu kalibrasi servo, action editor, webots),
#     dan container yang gagal start membawa serta manager dan studio.
#   * nomor /dev/videoN geser sama seperti ttyUSB. Dengan `-v /dev:/dev`, kamera
#     yang dicolok setelah container jalan langsung terlihat di dalamnya --
#     dengan `devices:` harus buat ulang container.
# Yang tersisa cuma perlu dilaporkan: op3_ball_detector dan op3_yolo_vision diam
# saja kalau tidak ada kamera, jadi bagian 1 dan 5 menyebutkannya.
camera_devices() {
  local dev
  for dev in /dev/video*; do
    [ -e "$dev" ] || continue
    echo "$dev"
  done
  return 0
}

# Buka-tutup port sekali. Dipakai di host maupun (lewat exec) di dalam container.
PORT_OPEN_PY='
import os, sys
try:
    fd = os.open(sys.argv[1], os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK); os.close(fd)
except OSError as exc:
    print(exc); sys.exit(1)
'

# Siapa yang sedang memegang port. Bus DXL bermaster tunggal: dua proses yang
# sama-sama membuka port itu saling merusak paket, dan gejalanya baru muncul
# jauh kemudian sebagai "first bulk read fail!!". Penyebab tersering: satu
# op3_action_editor yatim yang terminalnya sudah ditutup.
PORT_USER_PY='
import os, sys
target = os.path.realpath(sys.argv[1])
for pid in os.listdir("/proc"):
    if not pid.isdigit():
        continue
    fd_dir = "/proc/%s/fd" % pid
    try:
        for fd in os.listdir(fd_dir):
            try:
                if os.path.realpath(os.path.join(fd_dir, fd)) == target:
                    with open("/proc/%s/comm" % pid) as handle:
                        print("%s(pid %s)" % (handle.read().strip(), pid))
                    sys.exit(0)
            except OSError:
                continue
    except OSError:
        continue
'

cmd_doctor() {
  echo "${BOLD}1. Perangkat di host${RST}"
  if lsusb 2>/dev/null | grep -qi '0403:6014'; then
    ok "adapter DXL (FTDI FT232H 0403:6014) terpasang"
  else
    bad "adapter FTDI TIDAK terlihat di lsusb -- colokannya lepas atau adapternya mati."
    bad "  Tanpa dia tidak ada bus DXL sama sekali: semua 20 joint akan"
    bad "  'does NOT respond!!'. Itu bukan servo rusak."
  fi
  if lsusb 2>/dev/null | grep -qi '0483:5740'; then
    ok "OpenCR micro-USB terpasang (konsol debug; ID 200 tetap lewat bus TTL)"
  else
    warn "OpenCR micro-USB tidak terlihat -- tidak apa-apa untuk jalan normal,"
    warn "  cuma tidak bisa flash / buka serial monitor lewat USB."
  fi
  local cams
  cams="$(camera_devices | tr '\n' ' ')"
  if [ -n "${cams// /}" ]; then
    ok "kamera V4L2: $cams"
  else
    warn "tidak ada /dev/video* -- op3_ball_detector dan op3_yolo_vision tidak akan"
    warn "  mengeluarkan deteksi apa pun (node-nya jalan, topiknya sepi). Tidak"
    warn "  masalah kalau memang sedang kalibrasi servo / action editor / webots."
  fi

  echo
  echo "${BOLD}2. Port DXL di host${RST}"
  # Robot ini tidak memegang mati satu nomor ttyUSB: launch file dan executor
  # memindai ttyUSB0-9 dan memakai yang bisa dibuka. Jadi yang diperiksa di sini
  # adalah kandidat yang sama dengan yang mereka pindai, bukan cuma ttyUSB0.
  local candidate usable="" holder
  local found_any=0
  while read -r candidate; do
    [ -n "$candidate" ] || continue
    found_any=1
    holder="$(python3 -c "$PORT_USER_PY" "$candidate" 2>/dev/null || true)"
    if python3 -c "$PORT_OPEN_PY" "$candidate" >/dev/null 2>&1; then
      [ -n "$usable" ] || usable="$candidate"
      if [ -n "$holder" ]; then
        warn "$candidate bisa dibuka, tapi sedang dipegang $holder"
      else
        ok "$candidate bisa dibuka"
      fi
    else
      if [ -n "$holder" ]; then
        bad "$candidate TIDAK bisa dibuka -- dipegang $holder"
      else
        bad "$candidate ada tapi tidak bisa dibuka (izin? node basi?)"
      fi
    fi
  done < <(dxl_candidates)
  if [ "$found_any" -eq 0 ]; then
    bad "tidak ada /dev/ttyUSB0-9 sama sekali. Lihat bagian 1 dan colokan USB-nya."
  elif [ -n "$usable" ]; then
    ok "auto-detect akan memakai $usable"
    echo "  ${BOLD}catatan${RST} paksa port lain dengan: OP3_DEVICE=/dev/ttyUSBx"
    echo "          pemegang port hanya terbaca untuk proses milikmu sendiri di host;"
    echo "          proses di dalam container dilaporkan di bagian 5."
  else
    bad "ada port tapi tak satu pun bisa dibuka -- hentikan dulu yang memegangnya."
  fi
  # Symlink udev cuma informasi di sini: OP3.robot memakai /dev/ttyUSBx dan
  # udev tidak jalan di dalam container, jadi ttyOP3 tidak wajib ada.
  if [ -L /dev/ttyOP3 ]; then
    ok "/dev/ttyOP3 -> $(readlink -f /dev/ttyOP3) (symlink udev host; tidak wajib untuk stack ini)"
  else
    warn "/dev/ttyOP3 tidak ada -- tidak masalah, stack ini memakai /dev/ttyUSBx."
  fi

  echo
  echo "${BOLD}3. Kestabilan colokan (kernel, 30 menit terakhir)${RST}"
  local drops
  drops="$(journalctl -k --since '-30min' --no-pager 2>/dev/null | grep -c 'USB disconnect' || true)"
  if [ "${drops:-0}" -eq 0 ]; then
    ok "tidak ada USB disconnect"
  elif [ "${drops:-0}" -le 2 ]; then
    warn "$drops disconnect -- perhatikan"
  else
    bad "$drops disconnect dalam 30 menit. Kabel/konektor adapter DXL bermasalah;"
    bad "ini akan memutus bus di tengah walking, berapa kali pun software diperbaiki."
  fi

  echo
  echo "${BOLD}4. Port yang tertulis di OP3.robot${RST}"
  local port found=0
  while read -r port; do
    [ -n "$port" ] || continue
    found=1
    if [ -e "$port" ]; then
      ok "$(basename "$port") ada di host"
    else
      # Bukan kegagalan: launch file menambal salinan OP3.robot dengan port hasil
      # auto-detect sebelum manager dijalankan. Yang fatal justru kalau bagian 2
      # juga kosong.
      warn "$(basename "$port") tidak ada -- auto-detect akan menambalnya"
      [ -n "$usable" ] && warn "  (ke $usable, lihat bagian 2)"
    fi
  done < <(robot_file_ports)
  [ "$found" -eq 1 ] || warn "tidak ada path /dev di $ROBOT_FILE"

  echo
  # Kalau container mati, bagian 5 dan 6 memang tidak bisa diperiksa -- tapi
  # bagian 7 murni soal timestamp berkas di host, jadi jangan ikut dilewati:
  # justru itu yang biasanya perlu dibaca sebelum menyalakan apa pun.
  if container_running; then
    echo "${BOLD}5. Port di dalam container${RST}"
    doctor_container_ports
    echo
    echo "${BOLD}6. Workspace di dalam container${RST}"
    doctor_container_workspace
  else
    echo "${BOLD}5-6. Di dalam container${RST}"
    warn "container '$NAME' tidak jalan -- jalankan: scripts/op3_docker.sh up"
    warn "  (dilewati; bagian 7 di bawah tetap berlaku)"
    # Kalau bangkainya masih ada, sebab matinya jauh lebih berguna daripada
    # sekadar "tidak jalan". Pernah terjadi di robot ini: container mati sendiri
    # dengan exit 127 dan `--restart unless-stopped` TIDAK membangunkannya
    # (docker tidak me-restart container yang dihentikan atas permintaan), jadi
    # seisi stack ikut hilang tanpa jejak sampai ada yang menjalankan doctor.
    if container_exists; then
      local state
      state="$($DOCKER inspect "$NAME" \
        --format 'exit={{.State.ExitCode}} berhenti={{.State.FinishedAt}} restart={{.RestartCount}}' \
        2>/dev/null || true)"
      [ -n "$state" ] && warn "  bangkai container terakhir: $state"
      warn "  log terakhirnya: $DOCKER logs --tail 30 $NAME"
    fi
  fi
  # Diperiksa di kedua keadaan. Justru waktu '$NAME' jalan pun container lama
  # yang masih hidup berbahaya: dua-duanya bisa membuka port DXL yang sama, dan
  # bus ini bermaster tunggal.
  doctor_foreign_container

  echo
  echo "${BOLD}7. Kesegaran build (ABI)${RST}"
  doctor_abi
}

# Container lain yang me-mount workspace ini akan menahan port DXL dan bikin
# bingung ("kok manager-nya jalan padahal container-ku mati?"). Sering terjadi
# sesudah `docker run` manual: namanya acak, jadi tidak ketahuan kalau tidak
# dicari.
doctor_foreign_container() {
  local other
  other="$($DOCKER ps --format '{{.Names}}' 2>/dev/null | while read -r c; do
    [ "$c" = "$NAME" ] && continue
    if $DOCKER inspect "$c" --format '{{range .Mounts}}{{.Destination}} {{end}}' 2>/dev/null | grep -q '/ros2_ws'; then
      echo "$c"
    fi
  done)"
  if [ -n "$other" ]; then
    warn "container LAIN memakai workspace ini: $(echo "$other" | tr '\n' ' ')"
    warn "  Itu container gaya lama. Pindah kalau sudah selesai memakainya:"
    warn "  docker rm -f $(echo "$other" | head -1) && scripts/op3_docker.sh up"
  fi
}

doctor_container_ports() {
  if $DOCKER inspect "$NAME" --format '{{range .Mounts}}{{.Destination}}{{"\n"}}{{end}}' 2>/dev/null | grep -qx '/dev'; then
    ok "/dev di-bind dari host (node dan symlink udev mengikuti host secara live)"
  else
    bad "/dev TIDAK di-bind. Container ini dijalankan dengan perintah lama;"
    bad "  node perangkatnya akan basi begitu adapter DXL pindah nomor. Jalankan ulang:"
    bad "  scripts/op3_docker.sh down && scripts/op3_docker.sh up"
  fi
  local candidate seen=0 holder
  while read -r candidate; do
    [ -n "$candidate" ] || continue
    if ! $DOCKER exec "$NAME" test -e "$candidate" 2>/dev/null; then
      bad "$(basename "$candidate") ada di host tapi TIDAK terlihat di dalam container"
      continue
    fi
    seen=1
    holder="$($DOCKER exec "$NAME" python3 -c "$PORT_USER_PY" "$candidate" 2>/dev/null || true)"
    if $DOCKER exec "$NAME" python3 -c "$PORT_OPEN_PY" "$candidate" >/dev/null 2>&1; then
      if [ -n "$holder" ]; then
        warn "$(basename "$candidate") terbuka, sedang dipegang $holder di dalam container"
      else
        ok "$(basename "$candidate") bisa dibuka dari dalam container"
      fi
    else
      bad "$(basename "$candidate") ada tapi GAGAL dibuka (node basi / perangkat lepas)"
    fi
  done < <(dxl_candidates)
  [ "$seen" -eq 1 ] || bad "tidak ada kandidat port yang terlihat di dalam container"
  # Kamera lewat bind yang sama. Yang dicari bukan "ada kamera" (itu urusan
  # bagian 1) melainkan "kamera host tembus ke container" -- kalau container
  # dijalankan cara lama dengan `devices:`, kamera yang dicolok belakangan tidak
  # akan pernah muncul di sini sampai container dibuat ulang.
  local cam cam_seen=0 cam_any=0
  while read -r cam; do
    [ -n "$cam" ] || continue
    cam_any=1
    if $DOCKER exec "$NAME" test -e "$cam" 2>/dev/null; then
      cam_seen=1
    else
      bad "$(basename "$cam") ada di host tapi TIDAK terlihat di dalam container"
    fi
  done < <(camera_devices)
  if [ "$cam_any" -eq 0 ]; then
    warn "tidak ada kamera untuk diperiksa (lihat bagian 1)"
  elif [ "$cam_seen" -eq 1 ]; then
    ok "kamera host terlihat di dalam container"
  fi
  echo "  ${BOLD}catatan${RST} port terbuka belum berarti ada yang menjawab. Kalau semua di"
  echo "          atas hijau tapi joint tetap diam, ping bus-nya:"
  echo "          scripts/op3_docker.sh shell -c 'python3 scripts/op3_ping.py'"
}

doctor_container_workspace() {
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
    warn "shell polos belum punya ROS -- scripts/container_bashrc belum ter-mount"
    warn "  ke /root/.bashrc. Jalankan ulang: scripts/op3_docker.sh down && ... up"
  fi
  local pkg missing=0
  for pkg in op3_action_editor op3_manager op3_action_module; do
    if ! $DOCKER exec "$NAME" /docker-entrypoint.sh bash -c \
         "ros2 pkg prefix $pkg" >/dev/null 2>&1; then
      bad "paket '$pkg' tidak ketemu -- workspace belum di-build di dalam container"
      missing=1
    fi
  done
  if [ "$missing" -eq 0 ]; then
    ok "paket op3 inti terbaca (action_editor, manager, action_module)"
  fi

  # Khusus robot ini: op3_ball_detector memakai YOLO Python dari ALPHONSE, dan
  # image lama (dibangun sebelum blok pip di Dockerfile) tidak punya torch dsb.
  # Node-nya baru mengeluh saat launch, jauh setelah semua yang lain hijau.
  local pymod pymissing=""
  for pymod in torch ultralytics openvino cv2; do
    $DOCKER exec "$NAME" python3 -c "import $pymod" >/dev/null 2>&1 || pymissing="$pymissing $pymod"
  done
  if [ -z "$pymissing" ]; then
    ok "modul python YOLO lengkap (torch, ultralytics, openvino, cv2)"
  else
    warn "modul python belum ada di image '$IMAGE':$pymissing"
    warn "  yolo_ball_detector.py akan gagal import. Kemungkinan besar image-nya yang"
    warn "  tertukar -- lihat daftar di default_image() di script ini, atau paksa:"
    warn "  OP3_IMAGE=motion_webots_new-op3_ros2:latest scripts/op3_docker.sh down && ... up"
    warn "  Kalau memang image-nya yang kurang: build ulang (Dockerfile sudah memasangnya),"
    warn "  atau sementara di dalam container (hilang saat container dibuat ulang):"
    warn "  pip install -r src/ROBOTIS-OP3-Demo/op3_ball_detector/requirements_yolo.txt"
  fi
}

doctor_abi() {
  # Jebakan paling mahal yang pernah kena di repo ini (2026-08-12, ALPHONSE).
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
      bad "  Perbaiki: colcon build --symlink-install --merge-install --packages-select $pkg"
      abi_stale=1
    fi
  done < <(grep -rl 'robotis_controller/robotis_controller.h' \
             --include='*.h' --include='*.hpp' --include='*.cpp' "$WS/src" 2>/dev/null)
  # if/fi, bukan `[ ... ] && ok ...`: bentuk kedua membuat status keluar fungsi
  # ini jadi 1 waktu ada yang basi, dan `set -e` akan mematikan cmd_up tepat
  # sebelum ia sempat mencetak cara membuka shell.
  if [ "$abi_stale" -eq 0 ]; then
    ok "semua pemakai robotis_controller.h dibangun setelah header terakhir berubah"
  fi
}

cmd_up() {
  if container_running; then
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

  # X11 diteruskan kalau ada, supaya GUI (rqt_image_view, action editor Qt) tetap
  # bisa tampil. Jalur `--docker-run` lama membawa ini; tanpa disalin ke sini,
  # menyatukan semua ke satu script justru mematikan GUI.
  local -a x11=()
  if [ -n "${DISPLAY:-}" ]; then
    x11+=(-e "DISPLAY=$DISPLAY" -e QT_X11_NO_MITSHM=1)
    [ -d /tmp/.X11-unix ] && x11+=(-v /tmp/.X11-unix:/tmp/.X11-unix:rw)
    local xauth="${XAUTHORITY:-$HOME/.Xauthority}"
    [ -f "$xauth" ] && x11+=(-e XAUTHORITY=/tmp/.Xauthority -v "$xauth":/tmp/.Xauthority:rw)
  fi

  # ROS_DOMAIN_ID diteruskan apa adanya kalau di-set di host. Jangan dipatok di
  # sini: profil real_robot di scripts/op3_tmux memakai 1 dan meng-export-nya
  # sendiri ke tiap pane, sedangkan profil webots memakai 0.
  local -a ros_env=()
  [ -n "${ROS_DOMAIN_ID:-}" ] && ros_env+=(-e "ROS_DOMAIN_ID=$ROS_DOMAIN_ID")

  # docker-entrypoint.sh dan container_bashrc di-bind dari source, bukan dari
  # image: image yang sekarang terpasang masih menyimpan entrypoint lama yang
  # kehilangan argumen CMD (setupvars.sh milik OpenVINO menelan "$@", jadi
  # `exec "$@"` jalan tanpa argumen). Bind mount membuat perbaikannya berlaku
  # tanpa build ulang image -- di Jetson ini build image makan puluhan menit.
  # container_bashrc mengurus `docker exec -it op3 bash` polos, yang melewati
  # ENTRYPOINT dan kalau tidak lahir tanpa ROS sama sekali.
  #
  # `--restart unless-stopped`: setelah PC reboot container ikut hidup lagi
  # sendiri. Tanpa ini, tiap kali PC dinyalakan ulang container hilang dan semua
  # `ros2 run ...` menjawab "Package not found" -- workspace ini dibangun dengan
  # --symlink-install DI DALAM container, jadi seluruh install/ menunjuk ke
  # /ros2_ws yang cuma ada di sana. Kalau tidak mau auto-start, pakai
  # `scripts/op3_docker.sh down` (itu dihitung "stopped", jadi tidak bangkit).
  # `--init`: PID 1 di container ini `sleep infinity`, dan sleep tidak pernah
  # me-reap anak yatim. Proses ROS yang mati saat terminalnya ditutup jadi
  # menumpuk sebagai <defunct> dan mengotori `pgrep`/`ps`.
  $DOCKER run -d --name "$NAME" \
    --init \
    --restart unless-stopped \
    --net=host --ipc=host \
    --privileged \
    -v /dev:/dev \
    -v "$WS":/ros2_ws \
    -v "$WS/docker-entrypoint.sh":/docker-entrypoint.sh:ro \
    -v "$CONTAINER_BASHRC":/root/.bashrc:ro \
    -w /ros2_ws \
    ${input_gid:+--group-add "$input_gid"} \
    ${dialout_gid:+--group-add "$dialout_gid"} \
    "${x11[@]}" \
    "${ros_env[@]}" \
    --ulimit rtprio=99 --ulimit memlock=-1 \
    --cap-add SYS_NICE --cap-add SYS_RESOURCE \
    "$IMAGE" \
    sleep infinity >/dev/null

  sleep 1
  if ! container_running; then
    echo "${RED}Container gagal start.${RST} Lihat: $DOCKER logs $NAME" >&2
    exit 1
  fi
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
  #
  # `shell -c '...'` menjalankan satu perintah lalu keluar; tanpa argumen dapat
  # shell interaktif.
  #
  # `-t` hanya kalau stdin memang terminal. Dipasang tanpa syarat, pemakaian dari
  # dalam script atau lewat pipe langsung ditolak docker dengan "cannot attach
  # stdin to a TTY-enabled container because stdin is not a terminal".
  local -a flags=(-i)
  [ -t 0 ] && flags=(-i -t)
  exec $DOCKER exec "${flags[@]}" "$NAME" /docker-entrypoint.sh bash ${1+"$@"}
}

cmd_down() {
  container_exists || { echo "Container '$NAME' tidak ada."; return 0; }
  $DOCKER rm -f "$NAME" >/dev/null
  echo "Container '$NAME' dihapus."
}

sub="${1:-up}"
shift || true
case "$sub" in
  up)     cmd_up ;;
  shell)  cmd_shell "$@" ;;
  doctor) cmd_doctor ;;
  down)   cmd_down ;;
  *)
    echo "Pemakaian: $0 {up|shell [-c 'perintah']|doctor|down}" >&2
    exit 1
    ;;
esac
