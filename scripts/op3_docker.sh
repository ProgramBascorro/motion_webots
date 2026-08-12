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
#   scripts/op3_docker.sh down     # matikan dan hapus container

set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NAME="${OP3_CONTAINER_NAME:-op3}"
IMAGE="${OP3_IMAGE:-op3-webots-ros2:humble}"
ROBOT_FILE="$WS/src/ROBOTIS-OP3/op3_manager/config/OP3.robot"

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
  echo "${BOLD}6. Kesegaran build (ABI)${RST}"
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
  $DOCKER exec "$NAME" bash -c '
marker="# >>> op3: auto-source ROS (dipasang op3_docker.sh) >>>"
grep -qF "$marker" /root/.bashrc 2>/dev/null && exit 0
cat >> /root/.bashrc <<EOF
$marker
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash
# <<< op3: auto-source ROS <<<
EOF
' >/dev/null 2>&1 || warn "gagal memasang auto-source ROS di .bashrc container"
}

cmd_up() {
  if container_running; then
    harden_container_shell
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

cmd_down() {
  container_exists || { echo "Container '$NAME' tidak ada."; return 0; }
  $DOCKER rm -f "$NAME" >/dev/null
  echo "Container '$NAME' dihapus."
}

case "${1:-up}" in
  up)     cmd_up ;;
  shell)  cmd_shell ;;
  doctor) cmd_doctor ;;
  down)   cmd_down ;;
  *)
    echo "Pemakaian: $0 {up|shell|doctor|down}" >&2
    exit 1
    ;;
esac
