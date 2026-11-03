#!/usr/bin/env bash
# Menu Training (./script.sh -> Training).
#
# Berkas ini HANYA navigasi (gum). Data dibaca scripts/training.py setiap kali
# layar dibuka -- tidak ada daftar preset/page di sini -- dan training
# dijalankan scripts/autotune_walking.py (Bascorro Studio) atau
# scripts/autotune_action.py (Action Editor). Feedback: training_feedback.py (teks),
# training_iteration.py (feedback -> perubahan -> iterasi berikutnya); simpan & kirim
# ke robot: training_deploy.py. Panduan: PANDUAN_TRAINING.md.
#
# Harus jalan di DALAM kontainer op3 (ROS + op3_manager di sana); dari host,
# ./script.sh -> Training masuk lewat scripts/op3_docker.sh training.

set -uo pipefail

TRAINING_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TRAINING_WS="$(dirname "$TRAINING_DIR")"
TRAINING_PY=(python3 "$TRAINING_DIR/training.py")
TRAINING_GUIDE="$TRAINING_WS/PANDUAN_TRAINING.md"
TRAINING_SESSIONS="${BASCORRO_AUTOTUNE_DIR:-$TRAINING_WS/src/bascorro_studio/data/autotune}"

readonly OPT_REFRESH="↻ Perbarui daftar"
readonly OPT_BACK="↩ Kembali"
readonly OPT_MANUAL="⌨ Ketik nomor page..."
readonly OPT_NEW="▶ Mulai sesi training baru"
readonly OPT_OTHER_TRIAL="⌨ Percobaan lain (ketik nomor)..."
readonly OPT_PARAMS="📐 Apa yang dilatih di page ini (kenop, nilai servo, fase)"

# Kontainer berjalan di UTC, sedangkan dropdown Studio menampilkan waktu lokal
# (WIB): tanpa ini "diperbarui 16:04" di sini = "23:04" di Studio.
if [[ -z "${TZ:-}" && -e /.dockerenv ]]; then
  export TZ="${BASCORRO_TZ:-Asia/Jakarta}"
fi

# Pelatih butuh pesan ROS dari workspace ini. Shell `docker exec` biasa belum
# men-source apa pun; setup.bash ROS tidak tahan `set -u`.
ensure_ros() {
  python3 -c "import rclpy, op3_walking_module_msgs" >/dev/null 2>&1 && return 0
  set +u
  [[ -f /opt/ros/humble/setup.bash ]] && source /opt/ros/humble/setup.bash
  [[ -f "$TRAINING_WS/install/setup.bash" ]] && source "$TRAINING_WS/install/setup.bash"
  set -u
  python3 -c "import rclpy, op3_walking_module_msgs" >/dev/null 2>&1 && return 0
  echo "ROS/workspace tidak termuat -- jalankan menu ini di DALAM kontainer op3 (./script.sh -> Training)." >&2
  return 1
}

pause() {
  echo
  read -r -p "Tekan Enter untuk kembali..." _ || true
}

show_text() {  # show_text CMD... : keluaran panjang lewat pager
  "$@" 2>&1 | gum pager --soft-wrap 2>/dev/null || "$@"
}

# Baris "KUNCI<TAB>LABEL" dari training.py -> ROW_KEYS / ROW_LABELS
load_rows() {
  ROW_KEYS=()
  ROW_LABELS=()
  local line
  while IFS= read -r line; do
    [[ "$line" == *$'\t'* ]] || continue
    ROW_KEYS+=("${line%%$'\t'*}")
    ROW_LABELS+=("${line#*$'\t'}")
  done < <("$@")
}

# choose_row HEADER PILIHAN_TAMBAHAN... -> PICKED = kunci baris, atau teks pilihan tambahan
choose_row() {
  local header="$1"
  shift
  local -a labels=("${ROW_LABELS[@]}" "$@")
  local picked i
  picked="$(gum choose --height 22 --header "$header" "${labels[@]}")" || return 1
  for i in "${!ROW_LABELS[@]}"; do
    if [[ "${ROW_LABELS[$i]}" == "$picked" ]]; then
      PICKED="${ROW_KEYS[$i]}"
      return 0
    fi
  done
  PICKED="$picked"
}

ask_number() {  # ask_number PROMPT DEFAULT -> angka >= 1 di stdout
  local value
  value="$(gum input --prompt "$1" --value "$2")" || return 1
  [[ "$value" =~ ^[0-9]+$ && "$value" -ge 1 ]] || { echo "Bukan angka: $value" >&2; return 1; }
  echo "$value"
}

ask_session_name() {  # ask_session_name SARAN -> nama sesi baru yang sah
  local name
  name="$(gum input --prompt "Nama sesi: " --value "$1")" || return 1
  if [[ ! "$name" =~ ^[A-Za-z0-9_.-]+$ ]]; then
    echo "Nama sesi hanya huruf, angka, titik, garis bawah, atau strip." >&2
    return 1
  fi
  if [[ -e "$TRAINING_SESSIONS/$name" ]]; then
    echo "Sesi '$name' sudah ada -- pilih dari daftar untuk melanjutkannya." >&2
    return 1
  fi
  echo "$name"
}

run_trainer() {
  ensure_ros || { pause; return 1; }
  echo
  echo "Menjalankan: $*"
  echo "(Ctrl+C kapan saja = robot dihentikan, sesi tersimpan)"
  echo
  "$@"
  local rc=$?
  [[ $rc -eq 0 ]] || echo "(berhenti dengan kode $rc)"
  pause
}

trainer_for() {
  if [[ "$1" == "studio" ]]; then echo "$TRAINING_DIR/autotune_walking.py"; else echo "$TRAINING_DIR/autotune_action.py"; fi
}

# load_flags SESI: KIND DONE BUDGET ... milik sesi itu ke variabel `local` PEMANGGIL
# (bash memakai cakupan dinamis, jadi eval di sini mengisi local pemanggil).
load_flags() {
  local flags
  flags="$("${TRAINING_PY[@]}" session-flags "$1")" || return 1
  eval "$flags"
}
readonly FLAG_VARS="KIND DONE BUDGET POPSIZE STALE RUNNING HAS_BEST BEST PAGE START START_ID PENDING LAST_FB_TRIAL"

# prepare_session SESI [rerun] -> EXTRA=(opsi pelatih); 1 = batal / tidak bisa sekarang.
# rerun: menguji ulang satu hasil. Hasil page tetap persis walau page aslinya sudah
# diubah (yang diputar isi saat sesi dibuat), jadi tidak perlu --follow-page.
prepare_session() {
  local name="$1" purpose="${2:-train}"
  local $FLAG_VARS
  load_flags "$name" || { pause; return 1; }
  EXTRA=()
  if (( RUNNING )); then
    echo "Sesi $name sedang berjalan di terminal lain (pid $RUNNING). Tutup dulu di sana."
    pause
    return 1
  fi
  case "$STALE" in
    rescore)
      gum confirm "Rumus skor sesi $name sudah berubah. Hitung ulang dari rekaman mentah dulu (cadangan otomatis)?" || return 1
      python3 "$(trainer_for "$KIND")" --session "$name" --rescore | tail -n 12
      pause
      ;;
    follow)
      if [[ "$KIND" == "studio" ]]; then
        gum confirm "Page 2 sudah di-teach ulang sejak sesi ini dimulai. Lanjutkan dengan offset pose digeser ke acuan baru?" || return 1
        EXTRA+=(--follow-reference)
      elif [[ "$purpose" != "rerun" ]]; then
        gum confirm "Page aslinya sudah diubah sejak sesi ini dimulai. Lanjutkan dengan kenop dikenakan ke isi page yang baru?" || return 1
        EXTRA+=(--follow-page)
      fi
      ;;
  esac
  return 0
}

# ---- lanjutkan / ulangi / label sesi yang sudah ada ----------------------------------

session_menu() {
  local name="$1"
  local $FLAG_VARS
  load_flags "$name" || { pause; return; }
  prepare_session "$name" || return
  local script
  script="$(trainer_for "$KIND")"
  local choice
  local -a extra=()
  [[ -n "$PENDING" ]] && extra=("⏭ Jalankan iterasi feedback $PENDING yang menunggu (1 percobaan)")
  choice="$(gum choose --header "Sesi $name: $DONE/$BUDGET percobaan CMA${PENDING:+ · iterasi feedback $PENDING MENUNGGU (dijalankan sebelum kandidat lain)}" \
    "▶ Lanjutkan training" \
    "${extra[@]}" \
    "🔁 Ulangi kandidat terbaik (bukti bukan kebetulan)" \
    "🏷  Beri label percobaan (n/j/h)" \
    "📋 Laporan lengkap percobaan" \
    "$OPT_BACK")" || return
  case "$choice" in
    "⏭"*)
      run_trainer python3 "$script" --session "$name" --feedback-run "${EXTRA[@]}"
      after_feedback_run "$name"
      ;;
    "▶ Lanjutkan training")
      if (( DONE >= BUDGET )); then
        local total
        total="$(ask_number "Anggaran habis ($DONE/$BUDGET). Total percobaan CMA baru: " $((DONE + 2 * POPSIZE)))" || { pause; return; }
        EXTRA+=(--trials "$total")
      fi
      run_trainer python3 "$script" --session "$name" "${EXTRA[@]}"
      after_training "$name"
      ;;
    "🔁"*)
      if (( ! HAS_BEST )); then echo "Belum ada kandidat yang sah untuk diulang."; pause; return; fi
      local times
      times="$(ask_number "Ulangi kandidat terbaik #$BEST berapa kali? " 2)" || { pause; return; }
      run_trainer python3 "$script" --session "$name" --rerun-best "$times" --trial "$BEST" "${EXTRA[@]}"
      after_training "$name"
      ;;
    "🏷"*)
      local spec
      spec="$(gum input --prompt "Label (mis. 4=j,7=h; n normal, j jatuh/gagal, h hampir/dibantu): ")" || return
      [[ -n "$spec" ]] && python3 "$script" --session "$name" --label "$spec" | tail -n 12
      pause
      ;;
    "📋"*)
      show_text python3 "$script" --session "$name" --report
      ;;
  esac
}

# Sesudah pelatih keluar (selesai, q, atau Ctrl+C): langsung ke hasil terbaiknya, supaya
# feedback / simpan / kirim ke robot bisa dipilih tanpa mencari sesinya lagi.
after_training() {
  local name="$1"
  local $FLAG_VARS
  [[ -f "$TRAINING_SESSIONS/$name/state.json" ]] || return
  load_flags "$name" || return
  [[ -n "$PENDING" ]] && echo "Iterasi feedback $PENDING MENUNGGU: dijalankan pertama kali sesi ini dilanjutkan (Lanjutkan training / ⏭)."
  if (( HAS_BEST )); then
    result_menu "$name" "$BEST" "Training sesi $name berhenti ($DONE/$BUDGET percobaan CMA). Hasil terbaik sekarang: #$BEST." \
      "sesudah training"
  else
    echo "Sesi $name belum punya hasil yang sah (belum ada percobaan, atau semuanya jatuh/gagal)."
    echo "Feedback tetap bisa ditulis untuk percobaan mana pun lewat Riwayat hasil."
    pause
  fi
}

# ---- Train dari Bascorro Studio --------------------------------------------------

new_studio_session() {
  local id="$1" name trials walk motion
  name="$(ask_session_name "$("${TRAINING_PY[@]}" suggest-name studio "$id")")" || { pause; return; }
  trials="$(ask_number "Anggaran percobaan CMA (8 = satu generasi): " 48)" || { pause; return; }
  walk="$(gum input --prompt "Lama jalan per percobaan, detik (kosong = bawaan jenis gerak): ")" || return
  motion="$(gum choose --header "Jenis gerak (menentukan yang dikunci & sasaran belok)" \
    "otomatis (dari angka preset)" maju mundur geser putar di_tempat)" || return
  local -a args=(--session "$name" --start "$id" --trials "$trials")
  [[ -n "$walk" ]] && args+=(--walk-sec "$walk")
  [[ "$motion" != otomatis* ]] && args+=(--motion "$motion")
  gum confirm "Mulai sesi '$name' ($trials percobaan)? Robot harus DIGANTUNG." || return
  run_trainer python3 "$(trainer_for studio)" "${args[@]}"
  after_training "$name"
}

studio_target() {
  local id="$1"
  while true; do
    clear 2>/dev/null || true
    "${TRAINING_PY[@]}" studio-info "$id"
    local rc=$?
    echo
    if (( rc != 0 )); then pause; return; fi
    load_rows "${TRAINING_PY[@]}" sessions studio "$id"
    ROW_LABELS=("${ROW_LABELS[@]/#/⏯ Lanjutkan }")
    choose_row "Preset ini:" "$OPT_NEW" "$OPT_BACK" || return
    case "$PICKED" in
      "$OPT_NEW") new_studio_session "$id" ;;
      "$OPT_BACK") return ;;
      *) session_menu "$PICKED" ;;
    esac
  done
}

studio_flow() {
  while true; do
    load_rows "${TRAINING_PY[@]}" studio-list
    if (( ${#ROW_KEYS[@]} == 0 )); then
      echo "Tidak ada preset di walking_versions.json Bascorro Studio."
      pause
      return
    fi
    choose_row "Bascorro Studio: pilih preset (${#ROW_KEYS[@]} versi, dibaca $(date +%H:%M:%S))" \
      "$OPT_REFRESH" "$OPT_BACK" || return
    case "$PICKED" in
      "$OPT_REFRESH") continue ;;
      "$OPT_BACK") return ;;
      *) studio_target "$PICKED" ;;
    esac
  done
}

# ---- Train dari Action Editor -----------------------------------------------------

new_action_session() {
  local page="$1" name trials profile
  name="$(ask_session_name "$("${TRAINING_PY[@]}" suggest-name action "$page")")" || { pause; return; }
  profile="$(gum choose --header "Jenis gerakan (menentukan kenop & kriteria skor)" \
    "otomatis (sesuai info di atas)" kick getup pose umum)" || return
  trials="$(ask_number "Anggaran percobaan CMA: " 32)" || { pause; return; }
  local -a args=(--session "$name" --page "$page" --trials "$trials")
  [[ "$profile" != otomatis* ]] && args+=(--profile "$profile")
  gum confirm "Mulai sesi '$name' untuk page $page ($trials percobaan)?" || return
  run_trainer python3 "$(trainer_for action)" "${args[@]}"
  after_training "$name"
}

action_target() {
  local page="$1"
  while true; do
    clear 2>/dev/null || true
    "${TRAINING_PY[@]}" action-info "$page"
    local rc=$?
    echo
    if (( rc != 0 )); then pause; return; fi
    load_rows "${TRAINING_PY[@]}" sessions action "$page"
    ROW_LABELS=("${ROW_LABELS[@]/#/⏯ Lanjutkan }")
    choose_row "Page $page:" "$OPT_NEW" "$OPT_PARAMS" "$OPT_BACK" || return
    case "$PICKED" in
      "$OPT_NEW") new_action_session "$page" ;;
      "$OPT_PARAMS") show_text "${TRAINING_PY[@]}" action-params "$page" ;;
      "$OPT_BACK") return ;;
      *) session_menu "$PICKED" ;;
    esac
  done
}

action_flow() {
  while true; do
    load_rows "${TRAINING_PY[@]}" action-list
    choose_row "Action Editor: pilih page (${#ROW_KEYS[@]} page berisi gerakan, dibaca $(date +%H:%M:%S))" \
      "$OPT_MANUAL" "$OPT_REFRESH" "$OPT_BACK" || return
    case "$PICKED" in
      "$OPT_REFRESH") continue ;;
      "$OPT_BACK") return ;;
      "$OPT_MANUAL")
        PICKED="$(gum input --prompt "Nomor page (1-255): ")" || continue
        [[ -n "$PICKED" ]] && action_target "$PICKED"
        ;;
      *) action_target "$PICKED" ;;
    esac
  done
}

# ---- satu hasil: feedback, simpan, kirim ke robot, uji ---------------------------------

deploy_menu() {  # deploy_menu SESI N -- "Simpan langsung ke robot": target -> periksa -> konfirmasi -> tulis
  local name="$1" trial="$2"
  local $FLAG_VARS
  load_flags "$name" || { pause; return; }
  local choice to target pname=""
  if [[ "$KIND" == "studio" ]]; then
    choice="$(gum choose --header "Simpan hasil #$trial ke penyimpanan versi Bascorro Studio di robot sebagai:" \
      "🆕 Preset BARU (nama sendiri)" \
      "♻  Timpa preset asal '$START'" \
      "📋 Timpa preset lain..." \
      "$OPT_BACK")" || return
    case "$choice" in
      "🆕"*)
        to=studio-new
        target="$(gum input --prompt "Nama preset baru: " --value "$START #$trial $name")" || return
        [[ -n "$target" ]] || return
        ;;
      "♻"*) to=studio-preset; target="$START_ID" ;;
      "📋"*)
        load_rows "${TRAINING_PY[@]}" studio-list
        choose_row "Preset yang ditimpa (dibaca $(date +%H:%M:%S)):" "$OPT_BACK" || return
        [[ "$PICKED" == "$OPT_BACK" ]] && return
        to=studio-preset
        target="$PICKED"
        ;;
      *) return ;;
    esac
  else
    choice="$(gum choose --header "Simpan hasil #$trial ke .bin action_module di robot, page:" \
      "📥 Page KOSONG (disarankan untuk dicoba dulu)" \
      "♻  Timpa page asal $PAGE" \
      "⌨  Page lain (ketik nomor)" \
      "$OPT_BACK")" || return
    case "$choice" in
      "📥"*)
        to=page
        target="$(ask_number "Page tujuan (kosong): " "$("${TRAINING_PY[@]}" free-page)")" || { pause; return; }
        pname="$(gum input --prompt "Nama page (maks 13 karakter, kosong = nama asli): ")" || return
        ;;
      "♻"*) to=page; target="$PAGE" ;;
      "⌨"*)
        to=page
        target="$(ask_number "Nomor page tujuan (1-255): " "")" || { pause; return; }
        ;;
      *) return ;;
    esac
  fi
  local -a args=(deploy "$name" "$trial" --to "$to" --target "$target")
  [[ -n "$pname" ]] && args+=(--name "$pname")
  clear 2>/dev/null || true
  "${TRAINING_PY[@]}" "${args[@]}" --check
  local rc=$?
  echo
  case $rc in
    0)
      gum confirm "Tulis ke robot sekarang? (tidak menggerakkan robot)" || { echo "Dibatalkan, tidak ada yang ditulis."; pause; return; }
      ;;
    5)
      local answer
      answer="$(gum input --prompt "Target SUDAH BERISI. Ketik YA untuk menimpanya (cadangan dibuat dulu): ")" || return
      if [[ "$answer" != "YA" ]]; then
        echo "Dibatalkan, tidak ada yang ditulis."
        pause
        return
      fi
      args+=(--overwrite)
      ;;
    6) pause; return ;;
    4)
      # Dicatat sebagai pengiriman GAGAL di riwayat (tidak ada yang ditulis); salinan hasil tetap ada.
      "${TRAINING_PY[@]}" "${args[@]}" >/dev/null 2>&1
      echo "Tidak diterapkan -- tercatat di riwayat hasil. Salinan hasil tetap tersimpan, bisa dikirim lagi nanti."
      pause
      return
      ;;
    *) pause; return ;;
  esac
  echo
  "${TRAINING_PY[@]}" "${args[@]}"
  pause
}

test_result() {  # test_result SESI N -- MENJALANKAN gerakan hasil itu di robot (tindakan tersendiri)
  local name="$1" trial="$2"
  local $FLAG_VARS
  load_flags "$name" || { pause; return; }
  prepare_session "$name" rerun || return
  local times
  times="$(ask_number "Uji hasil #$trial berapa kali? " 1)" || { pause; return; }
  echo "Robot akan BERGERAK: $([[ "$KIND" == "studio" ]] && echo "page 2, lalu jalan dengan parameter hasil #$trial" \
    || echo "page 2/berbaring, lalu page hasil #$trial lewat page kerja 254"). Skornya ikut dirata-rata ke kandidat ini."
  echo "Sesudah tiap run: label n/j/h, atau c = tulis catatan."
  run_trainer python3 "$(trainer_for "$KIND")" --session "$name" --rerun-best "$times" --trial "$trial" "${EXTRA[@]}"
}

# Feedback -> pemahaman & perubahan (bukti lengkap) -> operator memilih -> SATU percobaan di
# robot memakai perubahan itu -> perbandingan -> hasil barunya. Perubahan disimpan di sesi
# yang SAMA (state.json "feedback_steps"), jadi "Lanjutkan training" melanjutkan dari situ.
feedback_iteration() {  # feedback_iteration SESI N
  local name="$1" trial="$2"
  local $FLAG_VARS
  load_flags "$name" || { pause; return; }
  if (( RUNNING )); then
    echo "Sesi $name sedang berjalan di terminal lain (pid $RUNNING): tulis catatannya di sana dengan 'c' --"
    echo "perubahannya langsung ditawarkan untuk percobaan berikutnya."
    pause
    return
  fi
  clear 2>/dev/null || true
  "${TRAINING_PY[@]}" feedback-plan "$name" "$trial"
  local rc=$?
  echo
  if (( rc == 7 )); then
    "${TRAINING_PY[@]}" feedback-apply "$name" "$trial" >/dev/null 2>&1   # pemahamannya tetap tercatat di riwayat
    echo "Feedback ini BELUM bisa diterapkan (lihat 'perlu' di atas). Tambahkan feedback yang menyebut arah,"
    echo "bagian badan/kaki, dan kapan terjadinya, lalu pilih lagi."
    pause
    return
  fi
  (( rc == 0 || rc == 8 )) || { pause; return; }
  local run="▶ Terapkan & jalankan 1 iterasi training sekarang (robot BERGERAK)"
  (( rc == 8 )) && run="✔ Terapkan ke penilaian (bobot saja, gerakan tidak berubah)"
  local choice
  choice="$(gum choose --header "Feedback hasil #$trial sesi $name -- rincian di atas:" \
    "$run" "📝 Simpan sebagai catatan saja (tidak diterapkan)" "$OPT_BACK")" || return
  case "$choice" in
    "📝"*) "${TRAINING_PY[@]}" feedback-apply "$name" "$trial" --decline; pause; return ;;
    "$OPT_BACK") return ;;
  esac
  "${TRAINING_PY[@]}" feedback-apply "$name" "$trial"
  rc=$?
  if (( rc == 8 )); then
    echo "Berlaku untuk penilaian semua percobaan dan pencarian CMA berikutnya (▶ Lanjutkan training)."
    pause
    return
  fi
  (( rc == 0 )) || { pause; return; }
  prepare_session "$name" || return
  echo "Robot akan BERGERAK: $([[ "$KIND" == "studio" ]] && echo "page 2, lalu jalan dengan nilai yang sudah diubah" \
    || echo "page 2/berbaring, lalu page dengan kenop yang sudah diubah lewat page kerja 254")."
  echo "Sesudah percobaan: label n/j/h, atau c = feedback lagi (langsung jadi iterasi berikutnya)."
  run_trainer python3 "$(trainer_for "$KIND")" --session "$name" --feedback-run "${EXTRA[@]}"
  after_feedback_run "$name" "$trial"
}

after_feedback_run() {  # after_feedback_run SESI [N_SUMBER] -- buka hasil iterasi feedback terakhir
  local name="$1" source="${2:-}"
  local $FLAG_VARS
  load_flags "$name" || return
  (( LAST_FB_TRIAL )) || return
  result_menu "$name" "$LAST_FB_TRIAL" "Iterasi feedback: hasil #$LAST_FB_TRIAL${source:+ (dibanding #$source)} -- perbandingan & bukti di bawah." \
    "sesudah iterasi feedback"
}

result_menu() {  # result_menu SESI N [JUDUL] [KONTEKS_FEEDBACK]
  local name="$1" trial="$2" header="${3:-}" where="${4:-riwayat hasil}"
  while true; do
    clear 2>/dev/null || true
    [[ -n "$header" ]] && printf '%s\n\n' "$header"
    "${TRAINING_PY[@]}" trial-info "$name" "$trial" || { pause; return; }
    echo
    local choice
    local $FLAG_VARS
    load_flags "$name" || { pause; return; }
    local -a params=()
    [[ "$KIND" == "action" ]] && params=("📐 Apa yang dilatih: kenop <-> nilai servo page ini")
    choice="$(gum choose --height 16 --header "Hasil #$trial sesi $name:" \
      "✍  Tambah feedback manual -> terapkan & jalankan iterasi berikutnya" \
      "🔁 Terapkan feedback yang sudah ada & jalankan 1 iterasi" \
      "💾 Simpan hasil terlebih dahulu (robot belum disentuh)" \
      "🤖 Simpan langsung ke robot" \
      "▶  Uji hasil ini di robot (MENJALANKAN gerakan)" \
      "🧾 Riwayat iterasi feedback sesi ini" \
      "${params[@]}" \
      "📤 Ekspor salinan berkas + feedback" \
      "↶  Kembalikan target ke isi sebelum penerapan terakhir" \
      "📖 Baca rincian lengkap (bisa digulir)" \
      "$OPT_BACK")" || return
    case "$choice" in
      "✍"*)
        clear 2>/dev/null || true
        if "${TRAINING_PY[@]}" feedback-add "$name" "$trial" --where "$where"; then
          feedback_iteration "$name" "$trial"
        else
          pause
        fi
        ;;
      "🧾"*) show_text "${TRAINING_PY[@]}" feedback-steps "$name" ;;
      "📐"*) show_text "${TRAINING_PY[@]}" action-params --session "$name" --trial "$trial" ;;
      "💾"*) "${TRAINING_PY[@]}" save "$name" "$trial"; pause ;;
      "🤖"*) deploy_menu "$name" "$trial" ;;
      "▶"*) test_result "$name" "$trial" ;;
      "🔁"*) feedback_iteration "$name" "$trial" ;;
      "📤"*)
        local dir
        dir="$(gum input --prompt "Folder tujuan: " --value "$TRAINING_WS/training_export")" || continue
        [[ -n "$dir" ]] && "${TRAINING_PY[@]}" bundle "$name" "$trial" --dir "$dir"
        pause
        ;;
      "↶"*)
        gum confirm "Kembalikan target penerapan terakhir hasil #$trial ke isi sebelumnya?" || continue
        "${TRAINING_PY[@]}" undo "$name" "$trial"
        pause
        ;;
      "📖"*) show_text "${TRAINING_PY[@]}" trial-info "$name" "$trial" ;;
      *) return ;;
    esac
    header=""
  done
}

session_results() {  # session_results SESI -- ringkasan sesi lalu pilih satu hasil
  local name="$1"
  while true; do
    clear 2>/dev/null || true
    "${TRAINING_PY[@]}" result-info "$name" || { pause; return; }
    echo
    load_rows "${TRAINING_PY[@]}" trial-list "$name"
    choose_row "Sesi $name: pilih hasil (🏆 terbaik · 💾 tersimpan · 🤖 di robot · ✍ feedback · 🧪 iterasi feedback)" \
      "$OPT_OTHER_TRIAL" "⏯ Lanjutkan / ulangi terbaik / label sesi" "✍ Baca semua feedback sesi ini" \
      "🧾 Riwayat iterasi feedback (catatan -> perubahan -> hasil)" "📋 Laporan lengkap percobaan" "$OPT_BACK" || return
    case "$PICKED" in
      "$OPT_BACK") return ;;
      "$OPT_OTHER_TRIAL")
        local n
        n="$(ask_number "Nomor percobaan: " "")" || { pause; continue; }
        result_menu "$name" "$n"
        ;;
      "⏯"*) session_menu "$name" ;;
      "✍"*) show_text "${TRAINING_PY[@]}" feedback-list "$name" ;;
      "🧾"*) show_text "${TRAINING_PY[@]}" feedback-steps "$name" ;;
      "📋"*)
        local $FLAG_VARS
        load_flags "$name" || { pause; continue; }
        show_text python3 "$(trainer_for "$KIND")" --session "$name" --report
        ;;
      *) result_menu "$name" "$PICKED" ;;
    esac
  done
}

results_flow() {
  while true; do
    load_rows "${TRAINING_PY[@]}" results-list
    if (( ${#ROW_KEYS[@]} == 0 )); then
      echo "Belum ada hasil training. Mulai dari 'Train dari Bascorro Studio' atau 'Train dari Action Editor'."
      pause
      return
    fi
    choose_row "Riwayat hasil training (${#ROW_KEYS[@]} sesi, terbaru dulu) -- pilih sesi" \
      "📋 Ringkasan semua sesi + hasil terbaik" "$OPT_REFRESH" "$OPT_BACK" || return
    case "$PICKED" in
      "$OPT_REFRESH") continue ;;
      "$OPT_BACK") return ;;
      "📋"*) show_text "${TRAINING_PY[@]}" results ;;
      *) session_results "$PICKED" ;;
    esac
  done
}

saved_flow() {
  while true; do
    load_rows "${TRAINING_PY[@]}" saved-list
    if (( ${#ROW_KEYS[@]} == 0 )); then
      echo "Belum ada hasil tersimpan. Simpan dari Riwayat hasil -> sesi -> hasil -> 💾 Simpan hasil terlebih dahulu."
      pause
      return
    fi
    choose_row "Hasil tersimpan (${#ROW_KEYS[@]}, kejadian terbaru dulu) -- pilih untuk dikirim ke robot / diekspor" \
      "$OPT_REFRESH" "$OPT_BACK" || return
    case "$PICKED" in
      "$OPT_REFRESH") continue ;;
      "$OPT_BACK") return ;;
      *) result_menu "${PICKED%%:*}" "${PICKED##*:}" ;;
    esac
  done
}

show_guide() {
  if [[ -f "$TRAINING_GUIDE" ]]; then
    gum pager --soft-wrap < "$TRAINING_GUIDE" 2>/dev/null || less "$TRAINING_GUIDE"
  else
    echo "Panduan tidak ditemukan: $TRAINING_GUIDE"
    pause
  fi
}

training_menu() {
  while true; do
    local choice
    choice="$(gum choose --header "Training" \
      "1) 🧪 Train dari Bascorro Studio" \
      "2) 🎬 Train dari Action Editor" \
      "3) 🏆 Riwayat hasil (feedback, simpan, kirim ke robot)" \
      "4) 💾 Hasil tersimpan (kirim ke robot / ekspor)" \
      "5) 📖 Panduan Training" \
      "6) ↩ Kembali ke menu utama")" || return 0
    case "$choice" in
      1*) studio_flow ;;
      2*) action_flow ;;
      3*) results_flow ;;
      4*) saved_flow ;;
      5*) show_guide ;;
      *) return 0 ;;
    esac
  done
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  command -v gum >/dev/null 2>&1 || { echo "gum tidak terpasang (dibutuhkan menu)." >&2; exit 1; }
  training_menu
fi
