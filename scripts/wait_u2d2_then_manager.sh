#!/usr/bin/env bash
# Wait for the U2D2 to be present AND stable, then launch op3_manager.
#
# The FT232H on this robot drops off the USB bus repeatedly (6 disconnects in 30
# minutes on 2026-08-11), so the by-id node keeps vanishing and reappearing on a
# different ttyUSB minor. Starting the manager by hand means racing that window;
# this waits for it instead. It does NOT fix the cable -- if the link drops again
# mid-walk, the manager still loses the bus and has to be restarted.
#
# Run it in the pane where you normally start the manager, then seat the plug.

set -u

WS="${OP3_WS:-/ros2_ws}"
DEV="${OP3_SERIAL_DEVICE:-/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WKHM6-if00-port0}"
STABLE_SECS="${OP3_STABLE_SECS:-3}"

# The by-id path must match the port line in OP3.robot -- the manager reads the
# servo bus port from there, not from device_name.
ROBOT_FILE="$WS/src/ROBOTIS-OP3/op3_manager/config/OP3.robot"
if [[ -f "$ROBOT_FILE" ]] && ! grep -q "$(basename "$DEV")" "$ROBOT_FILE"; then
  echo "[wait-u2d2] WARNING: $DEV is not the port in OP3.robot -- servos will not be found." >&2
  echo "[wait-u2d2]          check 'ls /dev/serial/by-id/' and fix OP3.robot first." >&2
fi

port_opens() {
  python3 - "$1" <<'PY' 2>/dev/null
import os, sys
try:
    fd = os.open(sys.argv[1], os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    os.close(fd)
except OSError:
    sys.exit(1)
PY
}

echo "[wait-u2d2] waiting for $DEV ..."
while true; do
  if [[ -e "$DEV" ]] && port_opens "$DEV"; then
    # Require the link to hold for STABLE_SECS -- a device that enumerates and
    # drops a second later would otherwise hand the manager a dying port.
    ok=1
    for ((i = 0; i < STABLE_SECS; i++)); do
      sleep 1
      if [[ ! -e "$DEV" ]] || ! port_opens "$DEV"; then
        echo "[wait-u2d2] port vanished after ${i}s -- still flapping, waiting again."
        ok=0
        break
      fi
    done
    if [[ $ok -eq 1 ]]; then
      break
    fi
  fi
  sleep 0.3
done

echo "[wait-u2d2] port stable for ${STABLE_SECS}s -> $(readlink -f "$DEV")"
echo "[wait-u2d2] starting op3_manager (robot will torque on and move to init pose)"

exec ros2 run op3_manager op3_manager --ros-args \
  -p offset_file_path:="$WS/src/ROBOTIS-OP3/op3_manager/config/offset.yaml" \
  -p robot_file_path:="$ROBOT_FILE" \
  -p init_file_path:="$WS/src/ROBOTIS-OP3/op3_manager/config/dxl_init_OP3.yaml" \
  -p device_name:="$DEV" \
  -p baud_rate:=2000000 \
  -p simulation:=false
