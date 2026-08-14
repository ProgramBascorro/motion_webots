# -------------------- Pohon proses: matikan sampai ke akar-akarnya --------------------
#
# Dipakai bersama oleh scripts/op3_tmux/lib/reap.sh (saat stack dihentikan) dan
# scripts/bascorro_studio.sh (saat studio ditutup). Keduanya menghadapi masalah
# yang sama persis:
#
#   `kill $pid` pada sebuah `ros2 run ...` / `ros2 launch ...` cuma membunuh
#   pembungkus python-nya. Node yang sebenarnya adalah CUCU-nya, dan dia selamat:
#   dipungut init jadi PPID 1, tetap memegang port TCP (rosbridge 9090, asset
#   server 8001, vite 5173) atau bus serial (op3_action_editor -> /dev/ttyUSBx).
#   Sisa itu lalu menghalangi start berikutnya, dan gejalanya jauh dari sebabnya:
#   tombol Walking di studio mati semua karena rosbridge tidak pernah naik.
#
# Berkas ini sengaja tidak bergantung apa pun: tanpa tmux, tanpa ROS, tanpa
# colors.sh (variabel warna dipakai lewat ${VAR:-} jadi aman kalau belum ada).

# Jeda SIGTERM sebelum naik ke SIGKILL.
PROC_TREE_GRACE_SEC="${PROC_TREE_GRACE_SEC:-2}"

proc_comm_of() {
  local comm
  comm="$(cat "/proc/$1/comm" 2>/dev/null || true)"
  printf '%s' "${comm:-?}"
}

proc_ppid_of() {
  local stat fields
  stat="$(cat "/proc/$1/stat" 2>/dev/null || true)"
  if [[ -z "$stat" ]]; then
    return 0
  fi
  # Buang "<pid> (<comm>) " lebih dulu: comm boleh berisi spasi dan kurung,
  # sisanya cuma huruf state dan angka, jadi aman dipecah spasi.
  stat="${stat##*) }"
  read -r -a fields <<< "$stat"
  printf '%s' "${fields[1]:-}"
}

# Diri sendiri dan seluruh leluhurnya. Tanpa daftar ini sebuah sapuan bisa bunuh
# diri di tengah jalan -- script pemanggilnya sering justru keturunan dari pid
# yang sedang dibereskan.
proc_protected_pids() {
  local pid="${BASHPID:-$$}"
  while [[ -n "$pid" && "$pid" != "0" && "$pid" != "1" ]]; do
    printf '%s\n' "$pid"
    pid="$(proc_ppid_of "$pid")"
  done
}

proc_is_protected() {
  local pid="$1"
  shift
  local guard
  for guard in "$@"; do
    if [[ "$pid" == "$guard" ]]; then
      return 0
    fi
  done
  return 1
}

# Semua keturunan sebuah pid (anak, cucu, ...), yang terdalam lebih dulu supaya
# induk tidak sempat memunculkan anak baru saat kita membunuh dari atas.
proc_descendants() {
  local pid="$1"
  local child
  while read -r child; do
    if [[ -z "$child" ]]; then
      continue
    fi
    proc_descendants "$child"
    printf '%s\n' "$child"
  done < <(pgrep -P "$pid" 2>/dev/null || true)
}

# SIGTERM dulu supaya node ROS sempat menutup port/berkasnya baik-baik, SIGKILL
# untuk yang bandel. Yang biasanya sampai tahap SIGKILL adalah op3_action_editor:
# handler SIGINT/SIGTERM-nya (op3_action_editor/src/main.cpp:103-106) memanggil
# system("clear") dan menggantung begitu pty-nya hilang.
proc_kill_pids() {
  local -a pids=("$@")
  if [[ "${#pids[@]}" -eq 0 ]]; then
    return 0
  fi

  local pid
  for pid in "${pids[@]}"; do
    echo "  ${DIM:-}TERM${RST:-} $(proc_comm_of "$pid")(pid $pid)"
    kill -TERM "$pid" 2>/dev/null || true
  done

  local -a alive=()
  local ticks=$(( PROC_TREE_GRACE_SEC * 5 ))
  local tick=0
  while :; do
    alive=()
    for pid in "${pids[@]}"; do
      if kill -0 "$pid" 2>/dev/null; then
        alive+=("$pid")
      fi
    done
    if [[ "${#alive[@]}" -eq 0 ]]; then
      return 0
    fi
    if [[ "$tick" -ge "$ticks" ]]; then
      break
    fi
    sleep 0.2
    tick=$(( tick + 1 ))
  done

  for pid in "${alive[@]}"; do
    echo "  ${YLW:-}KILL${RST:-} $(proc_comm_of "$pid")(pid $pid) ${DIM:-}-- tidak mati oleh SIGTERM${RST:-}"
    kill -KILL "$pid" 2>/dev/null || true
  done
  sleep 0.3
  return 0
}

# Bunuh sebuah pid BERIKUT seluruh keturunannya. Ini bentuk yang hampir selalu
# kamu mau untuk `ros2 run`/`ros2 launch`.
proc_kill_tree() {
  local -a protected=() targets=()
  mapfile -t protected < <(proc_protected_pids)

  local root child
  for root in "$@"; do
    if [[ -z "$root" ]] || ! kill -0 "$root" 2>/dev/null; then
      continue
    fi
    while read -r child; do
      if [[ -z "$child" ]] || proc_is_protected "$child" "${protected[@]}"; then
        continue
      fi
      targets+=("$child")
    done < <(proc_descendants "$root")
    if ! proc_is_protected "$root" "${protected[@]}"; then
      targets+=("$root")
    fi
  done

  proc_kill_pids "${targets[@]}"
}
