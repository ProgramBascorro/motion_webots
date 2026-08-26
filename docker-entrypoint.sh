#!/usr/bin/env bash
set -e

# A sourced script inherits the caller's positional parameters, and OpenVINO's
# setupvars.sh parses + shifts them away. That left "$@" empty here, so the
# final `exec "$@"` ran nothing and the container exited 0 the instant it
# started -- with `docker run -it ... bash` never reaching a shell. Stash the
# command Docker handed us and restore it right before exec.
entrypoint_cmd=("$@")

if [ -n "${OPENVINO_ROOT:-}" ] && [ -f "${OPENVINO_ROOT}/setupvars.sh" ]; then
  # setupvars.sh exports the runtime library paths and CMake hints expected by OpenVINO.
  # shellcheck disable=SC1090
  if ! source "${OPENVINO_ROOT}/setupvars.sh"; then
    echo "[docker-entrypoint] Warning: OpenVINO setupvars.sh returned non-zero, continuing anyway" >&2
  fi
fi

if ! source /opt/ros/humble/setup.bash; then
  echo "[docker-entrypoint] Error: failed to source /opt/ros/humble/setup.bash" >&2
  exit 1
fi

if [ -f /ros2_ws/install/setup.bash ]; then
  if ! source /ros2_ws/install/setup.bash; then
    echo "[docker-entrypoint] Warning: /ros2_ws/install/setup.bash returned non-zero, continuing anyway" >&2
  fi
fi

# Domain ROS 2 milik robot ini SENDIRI.
#
# Latar belakangnya nyata dan terukur (2026-08-26): tanpa ini semua node lahir
# di ROS_DOMAIN_ID 0 dengan DDS multicast ke seluruh LAN, jadi robot LAIN yang
# juga memakai domain 0 masuk ke graph yang sama. Yang terekam waktu itu: dengan
# op3_manager di sini MATI TOTAL, topik /robotis/enable_ctrl_module dan
# /robotis/present_joint_ctrl_modules masih menerima pesan -- perintah modul
# robot sebelah. Akibatnya di demo: kepala terlihat "dicuri" ke base_module/none
# setiap 1,5 detik, pelacak merebutnya kembali, dan parameter jalan/kepala
# tertimpa punya robot lain di tengah demo ("tiba-tiba parameternya lain").
# Perintah kita pun ikut mendarat di robot mereka.
#
# ROS_LOCALHOST_ONLY=1 mengunci DDS ke loopback: seluruh stack ini (manager,
# demo, kamera, studio agent) hidup di mesin yang sama, jadi tidak ada yang
# hilang. Kalau suatu saat perlu node dari laptop lain, matikan dengan
# `docker exec -e ROS_LOCALHOST_ONLY=0 ...` dan pastikan domain-nya unik.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

# Webots env (redundant with Dockerfile ENV but harmless)
export WEBOTS_HOME="${WEBOTS_HOME:-/usr/local/webots}"
export LD_LIBRARY_PATH="$WEBOTS_HOME/lib:$WEBOTS_HOME/lib/controller:${LD_LIBRARY_PATH:-}"
export USER="${USER:-root}"

set -- "${entrypoint_cmd[@]}"

echo "[docker-entrypoint] launching: $*" >&2
exec "$@"
