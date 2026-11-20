#!/usr/bin/env bash
#
# Ukur FPS detektor secara berulang-ulang dan SEBANDING, tanpa menjalankan demo.
#
# Kenapa alat ini ada: PANDUAN_OPTIMASI_FPS_DETEKSI.md menuntut semua pengukuran
# dilakukan "saat demo penuh berjalan", karena angka detektor sendirian jauh
# lebih bagus daripada angka di lapangan. Robot ini TIDAK bisa menjalankan demo.
# Jadi bebannya dibangkitkan di sini: kamera nyata + detektor nyata + sejumlah
# utas sibuk yang menggantikan pesaing CPU milik demo (usb_cam, op3_manager,
# op_demo_node, head_tracking). Angka jadi bisa dibandingkan sebelum/sesudah
# tiap langkah optimasi -- itu yang penting, bukan kemiripannya dengan demo.
#
# op3_manager sengaja TIDAK ikut dijalankan: ia menggerakkan servo ke pose init.
#
# Dijalankan dari HOST. Detektornya sendiri jalan di dalam container `op3`.
#
#   scripts/fps_bench.sh                          # bola, 3 pesaing, 45 detik
#   scripts/fps_bench.sh --both --seconds 60
#   scripts/fps_bench.sh --landmark --load 0      # tanpa pesaing
#   scripts/fps_bench.sh --both --label "sesudah-int8"
#
# ROS_DOMAIN_ID dipisah (bawaan 77) supaya tidak mengganggu node lain di LAN --
# ada detektor landmark milik mesin lain yang memakai nama topik yang sama.

set -u

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER="${OP3_CONTAINER:-op3}"
THROTTLE=/sys/devices/system/cpu/cpu0/thermal_throttle/package_throttle_total_time_ms

WHICH=ball
SECONDS_RUN=45
LOAD=3
LOAD_CPUS=""
DOMAIN=77
LABEL=""
EXTRA_BALL=""
EXTRA_LANDMARK=""

usage() {
  sed -n '2,30p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
  cat <<'USAGE'

Opsi:
  --ball | --landmark | --both   detektor mana yang diukur (bawaan --ball)
  --seconds N                    lama pengukuran (bawaan 45)
  --load N                       jumlah utas sibuk pengganti beban demo (bawaan 3)
  --load-cpus LIST               kurung utas sibuk ke inti ini (bawaan: bebas)
  --domain N                     ROS_DOMAIN_ID (bawaan 77)
  --label TEKS                   label yang dicetak di judul laporan
  --ball-args "..."              argumen tambahan untuk ball_detector.launch.py
  --landmark-args "..."          argumen tambahan untuk landmark_detector.launch.py
USAGE
}

while [ $# -gt 0 ]; do
  case "$1" in
    --ball) WHICH=ball ;;
    --landmark) WHICH=landmark ;;
    --both) WHICH=both ;;
    --seconds) SECONDS_RUN="$2"; shift ;;
    --load) LOAD="$2"; shift ;;
    --load-cpus) LOAD_CPUS="$2"; shift ;;
    --domain) DOMAIN="$2"; shift ;;
    --label) LABEL="$2"; shift ;;
    --ball-args) EXTRA_BALL="$2"; shift ;;
    --landmark-args) EXTRA_LANDMARK="$2"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Opsi tidak dikenal: $1" >&2; usage; exit 2 ;;
  esac
  shift
done

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
  echo "Container '$CONTAINER' tidak berjalan. Jalankan: scripts/op3_docker.sh up" >&2
  exit 1
fi
if [ ! -e /dev/video0 ]; then
  echo "PERINGATAN: /dev/video0 tidak ada -- kamera tidak terpasang." >&2
  echo "Detektor akan melaporkan 'FPS: 0.0 - no frame processed'." >&2
fi

LOGDIR="$(mktemp -d /tmp/fps_bench.XXXXXX)"
LOAD_PIDS=()
DET_PIDS=()

cleanup() {
  for p in "${LOAD_PIDS[@]:-}"; do kill "$p" 2>/dev/null; done
  docker exec "$CONTAINER" pkill -INT -f 'ball_detector.py|landmark_detector.py|usb_cam_node_exe' 2>/dev/null
  sleep 1
  docker exec "$CONTAINER" pkill -KILL -f 'ball_detector.py|landmark_detector.py|usb_cam_node_exe' 2>/dev/null
}
trap cleanup EXIT INT TERM

# --- pesaing CPU pengganti demo ---------------------------------------
# Utas sibuk murni: tidak meniru pola beban demo dengan setia, tapi ia
# MENGISI unit eksekusi -- dan itulah yang bikin inferensi melambat.
start_load() {
  [ "$LOAD" -le 0 ] && return
  local i
  for i in $(seq 1 "$LOAD"); do
    # Penanda "op3_fps_load" ada supaya utas yang tertinggal bisa dibersihkan
    # dengan `pkill -f op3_fps_load` tanpa ikut menembak proses lain.
    if [ -n "$LOAD_CPUS" ]; then
      taskset -c "$LOAD_CPUS" bash -c ': op3_fps_load; while :; do :; done' &
    else
      bash -c ': op3_fps_load; while :; do :; done' &
    fi
    LOAD_PIDS+=($!)
  done
}

run_detector() {   # $1 = ball|landmark, $2 = argumen tambahan
  local name="$1" extra="$2" launch camera
  if [ "$name" = ball ]; then launch=ball_detector.launch.py; else launch=landmark_detector.launch.py; fi
  # Hanya SATU node yang boleh membuka kamera.
  if [ "$WHICH" = both ] && [ "$name" = landmark ]; then camera="use_usb_cam:=false"; else camera=""; fi
  # ROS di-source EKSPLISIT: auto-source container ada di ~/.bashrc, dan
  # `bash -lc` membaca /etc/profile + ~/.profile -- bukan .bashrc. Tanpa ini
  # `ros2` tidak ketemu sama sekali.
  docker exec -e ROS_DOMAIN_ID="$DOMAIN" "$CONTAINER" bash -lc \
    "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && \
     timeout -s INT ${SECONDS_RUN}s ros2 launch op3_ball_detector $launch $camera $extra" \
    >"$LOGDIR/$name.log" 2>&1 &
  DET_PIDS+=($!)
}

# --- laporan ----------------------------------------------------------
report_one() {   # $1 = ball|landmark
  local log="$LOGDIR/$1.log"
  [ -f "$log" ] || return
  echo
  echo "--- $1 ---"
  # Baris hidupnya: "FPS: 12.3  |  infer 20.1 ms (...)  |  camera 30.0 FPS  |  ..."
  awk '
    match($0, /FPS: [0-9.]+/) {
      f = substr($0, RSTART+5, RLENGTH-5) + 0
      if (f > 0) { n++; s += f; if (mn=="" || f<mn) mn=f; if (f>mx) mx=f }
    }
    match($0, /infer [0-9.]+ ms/) {
      i = substr($0, RSTART+6, RLENGTH-9) + 0
      if (i > 0) { ni++; si += i }
    }
    match($0, /camera [0-9.]+ FPS/) {
      c = substr($0, RSTART+7, RLENGTH-11) + 0
      if (c > 0) { nc++; sc += c }
    }
    END {
      if (n == 0) { print "  tidak ada satu pun baris FPS -- lihat log lengkap"; exit }
      printf "  FPS   : min %.1f   avg %.1f   max %.1f   (%d jendela)\n", mn, s/n, mx, n
      if (ni) printf "  infer : %.1f ms  -> %.1f FPS model-only\n", si/ni, 1000/(si/ni)
      if (nc) printf "  kamera: %.1f FPS\n", sc/nc
    }' "$log"
  grep -E 'frames processed|pipeline FPS|inference *:' "$log" | sed 's/^/  /' | tail -4
  if grep -q 'no frame processed' "$log"; then
    echo "  PERINGATAN: ada jendela tanpa frame sama sekali (kamera?)"
  fi
}

# --- jalan ------------------------------------------------------------
THR_BEFORE=$(cat "$THROTTLE" 2>/dev/null || echo 0)
echo "==============================================================="
echo " fps_bench ${LABEL:+[$LABEL] }-- $WHICH, ${SECONDS_RUN}s, ${LOAD} pesaing CPU${LOAD_CPUS:+ di $LOAD_CPUS}"
echo " domain $DOMAIN | container $CONTAINER"
echo "==============================================================="

start_load
case "$WHICH" in
  ball)     run_detector ball "$EXTRA_BALL" ;;
  landmark) run_detector landmark "$EXTRA_LANDMARK" ;;
  both)     run_detector ball "$EXTRA_BALL"
            sleep 4                 # beri waktu usb_cam membuka perangkat
            run_detector landmark "$EXTRA_LANDMARK" ;;
esac

echo "mengukur ${SECONDS_RUN} detik..."
# Tunggu HANYA detektornya. `wait` tanpa argumen ikut menunggu utas sibuk
# pengganti beban demo, dan utas itu tidak pernah selesai -> menggantung.
for p in "${DET_PIDS[@]}"; do wait "$p"; done
for p in "${LOAD_PIDS[@]:-}"; do kill "$p" 2>/dev/null; done
LOAD_PIDS=()
THR_AFTER=$(cat "$THROTTLE" 2>/dev/null || echo 0)

report_one ball
report_one landmark

echo
echo "--- panas ---"
# Nilainya KUMULATIF sejak boot; hanya SELISIHNYA yang berarti.
DELTA=$(( THR_AFTER - THR_BEFORE ))
# Tanda kurung wajib: tanpa itu awk membaca "?:" sebagai bagian dari daftar
# argumen printf dan mengeluarkan string kosong.
PCT=$(awk -v d="$DELTA" -v s="$SECONDS_RUN" 'BEGIN{printf "%.1f", (s>0 ? d/(s*10) : 0)}')
echo "  throttling paket: +${DELTA} ms selama ${SECONDS_RUN} s  (${PCT}% dari durasi)"
if [ "$DELTA" -gt $(( SECONDS_RUN * 200 )) ]; then
  echo "  -> DI ATAS 20%: panas ikut bermain, lihat Langkah 5 panduan."
fi

echo
echo "log lengkap: $LOGDIR"
