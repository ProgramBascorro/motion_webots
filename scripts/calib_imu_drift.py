#!/usr/bin/env python3
"""Kalibrasi DRIFT (bias) gyro IMU -- robot harus DIAM selama proses.

Urutannya:
  1. pastikan robot benar-benar diam (rentang gyro_z tiap 1 dtk kecil),
  2. minta firmware mengkalibrasi gyro (imu_control bit 3) -- ini menghapus bias
     di sumbernya; firmware melakukannya sendiri tiap OpenCR menyala, tapi kalau
     saat itu robot sedang dipegang, hasilnya salah seumur nyala,
  3. ukur sisa hanyutan yaw gyro selama --seconds detik,
  4. tulis sisanya ke op3_manager/config/imu_drift.yaml (gyro_bias_dps), yang
     dimuat open_cr_module tiap op3_manager start.

Tidak menyentuh titik nol yaw: reset yaw (D58 / reset_yaw / zero_yaw) tetap
bekerja seperti biasa, hanya saja IMU-nya kini tidak menghanyut.

    python3 /ros2_ws/scripts/calib_imu_drift.py              # di dalam container
    docker exec -it op3 /docker-entrypoint.sh python3 /ros2_ws/scripts/calib_imu_drift.py

Kode keluar: 0 tersimpan, 1 gagal (robot bergerak / data tidak masuk).
"""
import argparse
import math
import os
import shutil
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_OUTPUT = os.path.join(REPO, "src", "ROBOTIS-OP3", "op3_manager", "config", "imu_drift.yaml")

# DIAM DINILAI DARI GYRO, sama dengan open_cr_module: rentang gyro_z tiap 1 dtk.
# Kompas tidak dipakai sebagai patokan karena bisa berkeliaran 8 deg dalam 10 dtk
# walau robot diam (medan terganggu, 2026-09-19). Diukur: diam maks 0,19 deg/s,
# berjalan p5 2,02 deg/s.
STILL_GZ_RANGE_DPS = 0.5
STILL_WINDOW_SEC = 1.0
MIN_SAMPLES = 100
SETTLE_SEC = 3.0                # jeda sesudah kalibrasi gyro firmware
EXIT_SAVED, EXIT_FAIL = 0, 1


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def sync_ros_env():
    """Samakan ROS_DOMAIN_ID/ROS_LOCALHOST_ONLY dengan op3_manager yang hidup."""
    if os.environ.get("CALIB_DRIFT_ENV_SYNCED") == "1":
        return
    pid = None
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        try:
            with open(f"/proc/{entry}/cmdline", "rb") as f:
                argv = f.read().split(b"\0")
        except OSError:
            continue
        if argv and argv[0].endswith(b"lib/op3_manager/op3_manager"):
            pid = entry
            break
    if pid is None:
        os.environ.setdefault("ROS_DOMAIN_ID", "1")
        return
    want = {}
    try:
        with open(f"/proc/{pid}/environ", "rb") as f:
            for item in f.read().split(b"\0"):
                k, _, v = item.decode(errors="replace").partition("=")
                if k in ("ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY"):
                    want[k] = v
    except OSError:
        return
    if all(os.environ.get(k) == v for k, v in want.items()):
        return
    env = dict(os.environ)
    env.update(want)
    env["CALIB_DRIFT_ENV_SYNCED"] = "1"
    print("env ROS disamakan dengan op3_manager: "
          + ", ".join(f"{k}={v}" for k, v in want.items()), flush=True)
    os.execve(sys.executable, [sys.executable] + sys.argv, env)


class Sensor:
    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Imu
        from std_msgs.msg import Float64MultiArray, String

        self.rclpy = rclpy
        rclpy.init()
        self.node = Node("calib_imu_drift")
        self.status = None
        self.gz = 0.0
        self.node.create_subscription(Float64MultiArray, "/robotis/open_cr/mag_status",
                                      self._status, 50)
        self.node.create_subscription(Imu, "/robotis/open_cr/imu", self._imu, 50)
        self.cmd_pub = self.node.create_publisher(String, "/robotis/open_cr/imu_command", 10)
        self.String = String

    def _status(self, msg):
        if len(msg.data) >= 4:
            self.status = list(msg.data)

    def _imu(self, msg):
        self.gz = msg.angular_velocity.z * 180.0 / math.pi

    def spin(self, sec=0.02):
        self.rclpy.spin_once(self.node, timeout_sec=sec)

    def wait_data(self, timeout=30.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            self.spin(0.05)
            if self.status is not None:
                return True
        return False

    def send(self, command):
        t0 = time.monotonic()
        while self.cmd_pub.get_subscription_count() == 0 and time.monotonic() - t0 < 5.0:
            self.spin(0.1)
        self.cmd_pub.publish(self.String(data=command))
        self.spin(0.2)

    def record(self, seconds, label):
        """(t, heading, yaw_gyro_raw, gz) tiap sampel selama `seconds` detik."""
        rows = []
        t0 = time.monotonic()
        shown = -1
        while time.monotonic() - t0 < seconds:
            self.spin(0.01)
            if self.status is None:
                continue
            d = self.status
            rows.append((time.monotonic() - t0, d[1], d[3], self.gz))
            s = int(time.monotonic() - t0)
            if s != shown:
                shown = s
                print(f"\r  {label} {s:3d}/{int(seconds)} dtk   gyro_z {self.gz:+.3f} deg/s   "
                      f"heading {d[1]:6.1f}", end="", flush=True)
        print()
        return rows

    def close(self):
        try:
            self.node.destroy_node()
            self.rclpy.shutdown()
        except Exception:
            pass


def still_check(rows):
    """(diam?, rentang gyro_z terbesar per 1 dtk) -- gyro yang menilai, bukan kompas."""
    if len(rows) < MIN_SAMPLES:
        return False, 0.0
    worst = 0.0
    start = 0
    for end in range(len(rows)):
        while rows[end][0] - rows[start][0] > STILL_WINDOW_SEC:
            start += 1
        window = [r[3] for r in rows[start:end + 1]]
        worst = max(worst, max(window) - min(window))
    return worst < STILL_GZ_RANGE_DPS, worst


def drift_rate(rows):
    """Laju hanyut yaw gyro MENTAH (deg/s) -- regresi linear, tahan kuantisasi 0,1."""
    t = [r[0] for r in rows]
    y = []
    acc = rows[0][2]
    prev = rows[0][2]
    for r in rows:
        acc += wrap180(r[2] - prev)
        prev = r[2]
        y.append(acc)
    n = len(t)
    tm = sum(t) / n
    ym = sum(y) / n
    den = sum((x - tm) ** 2 for x in t)
    if den <= 0:
        return 0.0
    return sum((t[i] - tm) * (y[i] - ym) for i in range(n)) / den


def write_yaml(path, bias, info):
    cfg_dir = os.path.dirname(os.path.abspath(path))
    st = os.stat(cfg_dir)
    hist = os.path.join(cfg_dir, "imu_drift_history")
    if os.path.exists(path):
        os.makedirs(hist, exist_ok=True)
        backup = os.path.join(hist, "imu_drift_" + time.strftime("%Y%m%d_%H%M%S") + ".yaml")
        shutil.copy2(path, backup)
        for p in (hist, backup):
            try:
                os.chown(p, st.st_uid, st.st_gid)
            except OSError:
                pass
        print(f"berkas lama dicadangkan: {backup}")

    text = (
        "# Kalibrasi drift (bias) gyro IMU -- ditulis scripts/calib_imu_drift.py\n"
        f"# Tanggal: {time.strftime('%Y-%m-%d %H:%M:%S')}\n"
        f"# {info}\n"
        "# Dipakai open_cr_module sebagai bias awal kompensasi hanyutan.\n"
        "# Berlaku sesudah op3_manager start ulang. Jangan disunting tangan.\n"
        "/**:\n"
        "  ros__parameters:\n"
        f"    gyro_bias_dps: {bias:.6f}\n"
    )
    tmp = f"{path}.tmp{os.getpid()}"
    with open(tmp, "w") as f:
        f.write(text)
        f.flush()
        os.fsync(f.fileno())
    try:
        os.chown(tmp, st.st_uid, st.st_gid)
        os.chmod(tmp, 0o664)
    except OSError:
        pass
    os.replace(tmp, path)
    dfd = os.open(cfg_dir, os.O_RDONLY)
    try:
        os.fsync(dfd)
    finally:
        os.close(dfd)
    print(f"tersimpan: {path}")


def main():
    ap = argparse.ArgumentParser(description="Kalibrasi drift (bias) gyro IMU")
    ap.add_argument("--seconds", type=float, default=60.0, help="lama pengukuran sisa hanyutan")
    ap.add_argument("--output", default=DEFAULT_OUTPUT)
    ap.add_argument("--no-gyro-cal", action="store_true",
                    help="jangan minta firmware mengkalibrasi gyro, ukur apa adanya")
    args = ap.parse_args()

    sync_ros_env()
    sensor = Sensor()
    try:
        if sensor.wait_data() is False:
            print("GAGAL: /robotis/open_cr/mag_status tidak mengalir -- op3_manager hidup? env ROS sama?")
            return EXIT_FAIL
        print("Robot harus DIAM sampai selesai. Jangan disentuh.\n")

        before = sensor.record(5.0, "cek diam ")
        ok, span = still_check(before)
        if ok is False:
            print(f"GAGAL: robot bergerak (rentang gyro_z {span:.2f} deg/s per 1 dtk > "
                  f"{STILL_GZ_RANGE_DPS}) -- diamkan robot lalu ulangi")
            return EXIT_FAIL
        gz_before = sum(r[3] for r in before) / len(before)
        drift_before = drift_rate(before)
        print(f"sebelum kalibrasi: gyro_z {gz_before:+.4f} deg/s, hanyut yaw {drift_before * 60:+.2f} deg/menit")

        if args.no_gyro_cal is False:
            sensor.send("calibrate_gyro")
            print("firmware diminta mengkalibrasi gyro -- robot tetap diam ...")
            sensor.record(SETTLE_SEC, "menunggu ")

        rows = sensor.record(args.seconds, "mengukur")
        ok, span = still_check(rows)
        if ok is False:
            print(f"GAGAL: robot bergerak saat pengukuran (rentang gyro_z {span:.2f} deg/s) -- ulangi")
            return EXIT_FAIL
        bias = drift_rate(rows)
        gz_after = sum(r[3] for r in rows) / len(rows)
        print(f"\nsesudah kalibrasi : gyro_z {gz_after:+.4f} deg/s")
        print(f"sisa hanyutan yaw : {bias:+.4f} deg/s = {bias * 60:+.2f} deg/menit "
              f"({len(rows)} sampel, rentang gyro_z maks {span:.2f} deg/s per 1 dtk)")

        info = (f"gyro_z sebelum {gz_before:+.4f} -> sesudah {gz_after:+.4f} deg/s; "
                f"sisa hanyutan {bias * 60:+.2f} deg/menit selama {args.seconds:.0f} dtk")
        write_yaml(args.output, bias, info)
        print("Berlaku sesudah op3_manager start ulang (log: 'Kalibrasi drift gyro dipakai').")
        print("Titik nol yaw tidak tersentuh: reset yaw D58 tetap seperti biasa.")
        return EXIT_SAVED
    except KeyboardInterrupt:
        print("\nDIBATALKAN -- tidak ada yang disimpan.")
        return EXIT_FAIL
    finally:
        sensor.close()


if __name__ == "__main__":
    sys.exit(main())
