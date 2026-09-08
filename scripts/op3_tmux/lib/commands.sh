run_in_shell() {
  local cmd="$1"
  $SHELL_RUNNER "$cmd"
}

# Build a command that sources setup, exports ROS_DOMAIN_ID + ROS_LOCALHOST_ONLY,
# then runs the cmd.
#
# ROS_LOCALHOST_ONLY WAJIB ikut diekspor di sini, bukan diserahkan ke profil.
#
# Pane tmux dijalankan lewat shell NON-login non-interaktif, jadi
# /etc/profile.d/op3-ros.sh dan /root/.bashrc -- dua tempat yang menyetel
# ROS_LOCALHOST_ONLY=1 -- sama sekali tidak terbaca. Dulu baris ini hanya
# mengekspor ROS_DOMAIN_ID, sehingga tiap node yang lahir dari tmux berjalan
# dengan domain 42 tapi ROS_LOCALHOST_ONLY=0, sementara Bascorro Studio (yang
# dijalankan dari shell login) berjalan dengan =1.
#
# Akibatnya stack terbelah dua dan TIDAK saling melihat, tanpa satu pun pesan
# kesalahan. Terukur 2026-08-29: op3_manager memegang kedua port serial dan
# hidup sehat, tapi /robotis/present_joint_states punya "Publisher count: 0"
# dari sisi Studio, /robotis/sync_write_item tidak punya pelanggan (tombol
# torque diam), dan /robotis/walking/get_params tidak pernah menjawab (parameter
# walking terlihat "hilang" padahal param.yaml di disk utuh).
#
# `scripts/op3_docker.sh doctor` TIDAK menangkap ini: ia memeriksa lewat
# `bash -lc`, yaitu shell login yang justru membaca profil itu, jadi ia
# melaporkan "ok" untuk lingkungan yang tidak dipakai pane tmux.
wrap_cmd() {
  local cmd="$1"
  local env_script="${OP3_ENV_SCRIPT:-$WS/scripts/op3_env.sh}"
  # NILAINYA DIPAKSA, bukan dijadikan bawaan "kalau belum diset".
  #
  # Versi sebelumnya memakai \${ROS_LOCALHOST_ONLY:-1}, dan itu tidak pernah
  # bekerja: pane tmux mewarisi env dari SERVER tmux, dan server itu lahir dari
  # shell pemanggil `tmux new-session` (start.sh). Kalau shell itu sudah punya
  # ROS_LOCALHOST_ONLY=0, variabelnya SUDAH TERSET saat pane lahir, sehingga
  # penjaga :- tidak pernah aktif dan angka 0 menang.
  #
  # Terukur 2026-08-30, persis kambuhnya gejala yang komentar di atas jelaskan:
  #   PID 6697 tmux new-session  ROS_LOCALHOST_ONLY=0
  #   PID 6807 op3_manager       ROS_LOCALHOST_ONLY=0
  #   PID 1039 rosbridge         ROS_LOCALHOST_ONLY=1
  # Manager sendirian di partisi DDS lain, jadi Bascorro Studio tidak melihat
  # satu pun topik robot dan SEMUA fungsinya diam tanpa pesan kesalahan.
  #
  # Override yang disengaja tetap mungkin lewat OP3_ROS_LOCALHOST_ONLY /
  # OP3_ROS_DOMAIN_ID, mengikuti pola yang sudah dipakai op3_docker.sh.
  local lh_forced="${OP3_ROS_LOCALHOST_ONLY:-1}"
  local dom_forced="${OP3_ROS_DOMAIN_ID:-$ROS_DOMAIN_ID_DEFAULT}"
  local ros_env="export ROS_DOMAIN_ID=$dom_forced; export ROS_LOCALHOST_ONLY=$lh_forced;"
  # shellcheck disable=SC2016
  if [[ -f "$env_script" ]]; then
    echo "source '$env_script'; source '$SETUP'; $ros_env export OP3_ACTION_FILE='${ACTION_FILE_PATH}'; $cmd"
  else
    echo "source '$SETUP'; $ros_env export OP3_ACTION_FILE='${ACTION_FILE_PATH}'; $cmd"
  fi
}
