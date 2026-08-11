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

  # docker-entrypoint.sh di-bind dari source: image masih menyimpan versi lama
  # yang kehilangan argumen CMD (setupvars.sh milik OpenVINO menelan "$@", jadi
  # `exec "$@"` jalan tanpa argumen dan container langsung exit 0). Bind mount
  # ini membuat perbaikannya berlaku tanpa rebuild image.
  $DOCKER run -d --name "$NAME" \
    --net=host --ipc=host \
    --privileged \
    -v /dev:/dev \
    -v "$WS":/ros2_ws \
    -v "$WS/docker-entrypoint.sh":/docker-entrypoint.sh:ro \
    -w /ros2_ws \
    ${input_gid:+--group-add "$input_gid"} \
    ${dialout_gid:+--group-add "$dialout_gid"} \
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
  exec $DOCKER exec -it "$NAME" bash
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
