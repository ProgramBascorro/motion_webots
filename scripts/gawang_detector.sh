#!/usr/bin/env bash
# Satu perintah: detektor GAWANG + rqt_image_view.
#
# Bisa dari HOST maupun dari DALAM container op3 (workspace di sana /ros2_ws):
#   host      : bash ~/motion_webots/scripts/gawang_detector.sh
#   container : bash /ros2_ws/scripts/gawang_detector.sh
#
#   ... gawang_detector.sh rqt:=false                   # tanpa jendela
#   ... gawang_detector.sh confidence_threshold:=0.45   # argumen launch lain
#
# Tutup jendela rqt ATAU Ctrl-C = detektor ikut berhenti.
#
# Kenapa skrip, bukan ketik `ros2 launch` langsung: detektor harus memakai
# ROS_DOMAIN_ID DAN ROS_LOCALHOST_ONLY yang sama dengan kamera yang sedang
# hidup, dan keduanya tergantung cara stack dinyalakan (30 Okt: 1/1, sedangkan
# `docker exec bash -lc` memberi 42). Satu saja beda = tidak ada gambar dan rqt
# hitam, tanpa galat yang jelas. Skrip ini membacanya dari proses kamera.
# Lihat src/ROBOTIS-OP3-Demo/op3_ball_detector/README_GAWANG.md.
set -euo pipefail

CONTAINER="${OP3_CONTAINER:-op3}"

# Di dalam container tidak ada perintah docker; semuanya dijalankan langsung.
if [ -f /.dockerenv ] && ! command -v docker >/dev/null 2>&1; then
  inside=1
else
  inside=0
  if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    echo "Container '$CONTAINER' tidak hidup. Nyalakan: bash scripts/op3_docker.sh up" >&2
    exit 1
  fi
fi

# Jalankan satu perintah shell di tempat yang benar (container op3).
in_op3() {
  if [ "$inside" -eq 1 ]; then
    bash -c "$1"
  else
    docker exec "$CONTAINER" bash -c "$1"
  fi
}

# --- 1. env ROS milik kamera yang sedang hidup -------------------------------
# Nama proses terpotong 15 huruf, maka usb_cam_node_ex (pgrep usb_cam_node_exe
# selalu kosong). Variabel yang TIDAK ada di environ kamera berarti nilai bawaan
# ROS (domain 0, localhost-only 0) -- jadi diisi eksplisit, bukan dibiarkan
# jatuh ke nilai shell/profil yang belum tentu sama.
cam_env="$(in_op3 \
  'p=$(pgrep -o -x usb_cam_node_ex) || exit 0
   echo "CAM_PID=$p"
   echo "CAM_STAT=$(ps -o stat= -p $p)"
   echo "CAM_PGID=$(ps -o pgid= -p $p | tr -d " ")"
   tr "\0" "\n" < /proc/$p/environ | grep -E "^ROS_(DOMAIN_ID|LOCALHOST_ONLY)=" || true')"
cam_pid="$(sed -n 's/^CAM_PID=//p' <<<"$cam_env")"
cam_stat="$(sed -n 's/^CAM_STAT=//p' <<<"$cam_env")"
cam_pgid="$(sed -n 's/^CAM_PGID=//p' <<<"$cam_env")"
# Kamera yang DITANGGUHKAN (Ctrl-Z -> stat "T") masih terlihat hidup tapi tidak
# mengirim satu frame pun. Terjadi 30 Okt: rqt hitam, detektor "tidak jalan".
# Bukan proses milik skrip ini, jadi tidak dilanjutkan diam-diam. Perintahnya
# dicetak dengan PGID yang SUDAH terisi: `ps -o pgid=` mengeluarkan spasi di
# depan angka, dan `-$(ps ...)` jadi "-" + angka -> kill menolak (terjadi 30 Okt).
if [ -n "$cam_pid" ] && [[ "$cam_stat" == T* ]]; then
  echo "[gawang] KAMERA (PID $cam_pid) sedang DITANGGUHKAN (stat $cam_stat, biasanya karena Ctrl-Z)." >&2
  echo "         Tidak ada frame sama sekali. Lanjutkan dengan (di dalam container):" >&2
  echo "             kill -CONT -- -$cam_pgid" >&2
  echo "         lalu jalankan skrip ini lagi. 'fg' hanya berlaku di terminal/pane yang" >&2
  echo "         menjalankannya. Untuk MENGHENTIKAN proses pakai Ctrl-C, bukan Ctrl-Z." >&2
  exit 1
fi
if [ -n "$cam_pid" ]; then
  domain="$(sed -n 's/^ROS_DOMAIN_ID=//p' <<<"$cam_env")"
  localhost="$(sed -n 's/^ROS_LOCALHOST_ONLY=//p' <<<"$cam_env")"
  domain="${domain:-0}"
  localhost="${localhost:-0}"
  ros_env="export ROS_DOMAIN_ID=$domain ROS_LOCALHOST_ONLY=$localhost;"
  echo "[gawang] kamera hidup (PID $cam_pid): ROS_DOMAIN_ID=$domain ROS_LOCALHOST_ONLY=$localhost -- dipakai juga di sini"
else
  # Tidak ada kamera: launch (use_usb_cam:=auto) menyalakannya sendiri di env
  # bawaan container, jadi tidak ada yang perlu disamakan.
  ros_env=""
  echo "[gawang] belum ada kamera hidup -- launch akan menyalakan usb_cam sendiri"
fi

# --- 2. hentikan detektor gawang lama -----------------------------------------
# Dua instance = dua node bernama sama yang berebut topik. Dicari dengan comm
# "ros2" + nama berkas launch; saringan comm membuat awk/grep tidak bisa
# mencocokkan dirinya sendiri (jebakan pgrep -f).
old="$(in_op3 'ps -eo pid=,comm=,args=' \
  | awk '$2 == "ros2" && /yolo_gawang_detector\.launch\.py/ {print $1}')"
gawang_alive() { in_op3 'ps -eo args=' | grep -q '[y]olo_gawang_detector\.py'; }
if [ -n "$old" ]; then
  echo "[gawang] menghentikan detektor gawang lama (launch PID $(echo $old))"
  # Ke seluruh GRUP proses (launch + detektor + rqt), persis seperti Ctrl-C.
  # SIGCONT sesudahnya wajib: proses yang ditangguhkan Ctrl-Z (stat "T") tidak
  # memproses SIGINT sampai dilanjutkan -- 30 Okt skrip menunggu sia-sia 6x.
  for pid in $old; do
    in_op3 "pg=\$(ps -o pgid= -p $pid | tr -d ' '); [ -n \"\$pg\" ] && \
            { kill -INT -- -\$pg; kill -CONT -- -\$pg; }" || true
  done
  for _ in $(seq 1 30); do
    gawang_alive || break
    sleep 0.5
  done
  if gawang_alive; then
    echo "[gawang] belum mati sesudah 15 s -- dipaksa (SIGKILL)"
    for pid in $old; do
      in_op3 "pg=\$(ps -o pgid= -p $pid | tr -d ' '); [ -n \"\$pg\" ] && kill -KILL -- -\$pg" || true
    done
    in_op3 "pkill -KILL -f '[y]olo_gawang_detector\\.py'" || true
    sleep 1
  fi
  if gawang_alive; then
    echo "[gawang] detektor lama tetap hidup; cek: ps -eo pid,stat,args | grep yolo_gawang" >&2
    exit 1
  fi
fi

# --- 3. jalankan --------------------------------------------------------------
launch_args=()
want_rqt=1
for a in "$@"; do
  [[ "$a" == rqt:=* ]] && want_rqt=0
  launch_args+=("$(printf '%q' "$a")")
done
[ "$want_rqt" -eq 1 ] && launch_args+=("rqt:=true")

cmd="$ros_env source /ros2_ws/install/setup.bash && \
  exec ros2 launch op3_ball_detector yolo_gawang_detector.launch.py ${launch_args[*]}"

echo "[gawang] menjalankan detektor -- tutup jendela rqt atau Ctrl-C untuk berhenti"
if [ "$inside" -eq 1 ]; then
  exec bash -lc "$cmd"
fi
tty_flags=(-i)
[ -t 0 ] && [ -t 1 ] && tty_flags=(-it)
exec docker exec "${tty_flags[@]}" "$CONTAINER" bash -lc "$cmd"
