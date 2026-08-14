# -------------------- Reaper: pastikan port DXL benar-benar lepas --------------------
#
# `tmux kill-session` mengirim SIGHUP ke shell tiap pane lalu menutup pty-nya.
# Untuk proses biasa itu cukup -- SIGHUP menjalar ke process group pane dan
# semuanya mati. Untuk stack ini TIDAK cukup, dua sebab:
#
#   * op3_action_editor memasang handler sendiri untuk SIGINT/SIGTERM/SIGQUIT
#     (op3_action_editor/src/main.cpp:103-106). Handler itu memanggil tcsetattr()
#     ke STDIN lalu system("clear") lalu exit(0). Kalau terminalnya sudah hilang,
#     ia menggantung di situ: prosesnya tetap hidup dan tetap memegang
#     /dev/ttyUSBx. Cuma SIGKILL yang menembusnya.
#   * proses yang sudah pindah process group tidak terjangkau SIGHUP pane mana
#     pun. Contoh yang paling sering: `ros2 run op3_action_editor executor.py`
#     dijalankan tangan dari `op3_docker.sh shell`, lalu di-Ctrl-C. executor.py
#     mati, tapi `ros2 run` yang di-Popen-nya dan node C++ di bawahnya dipungut
#     init container jadi PPID 1 dan terus jalan.
#
# Sisa seperti itu tidak kelihatan di tmux sama sekali, tapi bus DXL bermaster
# tunggal: op3_manager berikutnya menunggu 30 detik lalu menyerah
# ("/dev/ttyUSB0 is in use by op3_action_edit(pid ...) - giving up"), dan action
# editor berikutnya menolak start. Karena itu "berhenti" di sini berarti dua
# langkah, bukan satu:
#
#   1. bunuh pohon proses tiap pane secara eksplisit (presisi, lewat PPID --
#      tidak ada tebak-tebakan nama proses), baru kill-session;
#   2. sapu siapa pun yang MASIH memegang port DXL, dari mana pun asalnya.
#
# Langkah 2 juga dipakai sendirian lewat `./script.sh --free-port` untuk
# membereskan sisa run manual tanpa mematikan stack yang sedang jalan.

# Helper pohon proses (proc_descendants / proc_kill_pids / proc_protected_pids)
# dipakai bersama scripts/bascorro_studio.sh -- lihat scripts/lib/proc_tree.sh.
# shellcheck source=../../lib/proc_tree.sh
source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../../lib" && pwd)/proc_tree.sh"

# Kandidat port DXL: sama persis dengan yang dipindai op3_manager.launch.py dan
# executor.py, karena nomor ttyUSB-nya memang tidak dipegang mati.
#
# Sengaja TIDAK disaring dengan `-e`. Justru perangkat yang sudah tidak ada yang
# paling penting di sini: adapter FTDI re-enumerate (ttyUSB0 -> ttyUSB2), node
# lamanya terhapus, dan proses yang menggantung tetap memegang fd ke inode lama
# itu. Menyaring dengan `-e` membuat nama lamanya tidak pernah ikut dicari, jadi
# persis proses yang paling perlu dibersihkan yang lolos. Pencocokannya lewat
# nama, dan nama yang tidak ada tidak akan cocok dengan apa pun.
reap_dxl_devices() {
  local dev
  if [[ -n "${OP3_DEVICE:-}" ]]; then
    printf '%s\n' "$OP3_DEVICE"
  fi
  for dev in /dev/ttyUSB{0,1,2,3,4,5,6,7,8,9} /dev/ttyOP3; do
    printf '%s\n' "$dev"
  done
  return 0
}

# Sumber kebenarannya sama dengan RobotisController::findPortUser() dan bagian
# 2/5 op3_docker.sh: /proc/<pid>/fd. Dua bedanya:
#
#   * dikembalikan SEMUA pemegang, bukan yang pertama saja, karena semuanya
#     harus dibereskan;
#   * fd yang menunjuk inode terhapus ikut dihitung. Kernel menuliskannya sebagai
#     "/dev/ttyUSB0 (deleted)", dan itu BUKAN kasus langka di robot ini: adapter
#     FTDI re-enumerate tiap kali dicabut/di-reset, jadi node lamanya hilang
#     sementara proses yang memegangnya masih hidup. realpath() atas nama seperti
#     itu tidak akan pernah sama dengan "/dev/ttyUSB0", jadi tanpa penanganan ini
#     proses yang menggantung justru yang paling gampang lolos dari sapuan.
REAP_PORT_HOLDERS_PY='
import os, sys

DELETED = " (deleted)"

targets = set()
for dev in sys.argv[1:]:
    targets.add(os.path.realpath(dev))

for pid in os.listdir("/proc"):
    if not pid.isdigit():
        continue
    fd_dir = "/proc/%s/fd" % pid
    try:
        entries = os.listdir(fd_dir)
    except OSError:
        continue
    for fd in entries:
        try:
            link = os.readlink(os.path.join(fd_dir, fd))
        except OSError:
            continue
        if link.endswith(DELETED):
            link = link[:-len(DELETED)]
        if os.path.realpath(link) not in targets:
            continue
        try:
            with open("/proc/%s/comm" % pid) as handle:
                name = handle.read().strip()
        except OSError:
            name = "?"
        print("%s %s" % (pid, name or "?"))
        break
'

# stdout: satu baris "pid comm" per pemegang port.
reap_port_holders() {
  local -a devices=()
  mapfile -t devices < <(reap_dxl_devices)
  if [[ "${#devices[@]}" -eq 0 ]]; then
    return 0
  fi
  if ! command -v python3 >/dev/null 2>&1; then
    echo "${YLW}Warning:${RST} python3 tidak ada; tidak bisa memeriksa pemegang port." >&2
    return 0
  fi
  python3 -c "$REAP_PORT_HOLDERS_PY" "${devices[@]}" 2>/dev/null || true
}

# Apakah sapuan ini dijalankan di host sementara container-nya jalan?
#
# Kalau ya, hasil "bersih" tidak bisa dipercaya penuh: stack ini hidup DI DALAM
# container, dan /proc/<pid>/fd milik proses root di sana tidak terbaca oleh user
# biasa di host. Membuka perangkatnya juga bukan bukti apa-apa -- /dev/ttyUSBx
# bukan device eksklusif, dua proses bisa memegangnya bersamaan (justru itu
# masalahnya: paketnya saling merusak). Jadi satu-satunya yang jujur adalah
# menyebutkan keterbatasannya.
reap_running_outside_container() {
  if [[ -e /.dockerenv ]]; then
    return 1
  fi
  if ! command -v docker >/dev/null 2>&1; then
    return 1
  fi
  docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "${OP3_CONTAINER_NAME:-op3}"
}

# Bunuh isi tiap pane. Dipanggil SEBELUM kill-session, selagi daftar pane masih
# ada. Yang dibunuh cuma keturunan pane, bukan shell pane-nya sendiri -- itu
# urusan kill-session.
reap_session_panes() {
  if ! session_exists; then
    return 0
  fi

  local -a protected=()
  mapfile -t protected < <(proc_protected_pids)

  local -a targets=()
  local pane_pid child
  while read -r pane_pid; do
    if [[ -z "$pane_pid" ]]; then
      continue
    fi
    while read -r child; do
      if [[ -z "$child" ]]; then
        continue
      fi
      if proc_is_protected "$child" "${protected[@]}"; then
        continue
      fi
      targets+=("$child")
    done < <(proc_descendants "$pane_pid")
  done < <(tmux list-panes -s -t "$SESSION" -F '#{pane_pid}' 2>/dev/null || true)

  if [[ "${#targets[@]}" -eq 0 ]]; then
    return 0
  fi
  echo "${DIM}Menghentikan ${#targets[@]} proses milik pane...${RST}"
  proc_kill_pids "${targets[@]}"
}

# Sapu sisa pemegang port DXL dari mana pun asalnya (pane yang sudah hilang, run
# manual di shell container, sesi tmux sebelumnya).
reap_dxl_port_holders() {
  local -a protected=()
  mapfile -t protected < <(proc_protected_pids)

  local -a targets=()
  local -a labels=()
  local pid comm
  while read -r pid comm; do
    if [[ -z "$pid" ]]; then
      continue
    fi
    if proc_is_protected "$pid" "${protected[@]}"; then
      continue
    fi
    targets+=("$pid")
    labels+=("$comm(pid $pid)")
  done < <(reap_port_holders)

  if [[ "${#targets[@]}" -eq 0 ]]; then
    if reap_running_outside_container; then
      echo "${GRN}OK:${RST} tidak ada pemegang port DXL yang terlihat ${BOLD}dari host${RST}."
      echo "${DIM}  Stack ini jalan di dalam container dan fd proses root di sana tidak"
      echo "  terbaca dari sini. Kalau port masih terasa terkunci, ulangi dari dalam:${RST}"
      echo "    scripts/op3_docker.sh shell -c './script.sh --free-port'"
      return 0
    fi
    echo "${GRN}OK:${RST} tidak ada proses yang memegang port DXL."
    return 0
  fi

  echo "${YLW}Masih memegang port DXL:${RST} ${labels[*]}"
  proc_kill_pids "${targets[@]}"

  local -a leftover=()
  mapfile -t leftover < <(reap_port_holders)
  if [[ "${#leftover[@]}" -eq 0 ]]; then
    echo "${GRN}OK:${RST} port DXL sudah bebas -- action editor bisa langsung dijalankan."
    return 0
  fi
  echo "${RED}ERROR:${RST} masih ada yang memegang port DXL: ${leftover[*]}"
  echo "  Proses itu di luar jangkauan (beda user atau beda PID namespace)."
  echo "  Kalau ./script.sh dijalankan dari host, ulangi dari dalam container:"
  echo "    scripts/op3_docker.sh shell -c './script.sh --free-port'"
  return 1
}
