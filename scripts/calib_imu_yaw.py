#!/usr/bin/env python3
"""Kalibrasi YAW IMU (magnetometer OpenCR) 5 titik.

Merekam mag_heading MENTAH di 5 arah (putar KIRI: 0, 90, 180, 270, kembali ke
0) lalu menulis tabel ke op3_manager/config/yaw_calibration.yaml. open_cr_module
memuat tabel itu saat op3_manager start (op3_manager.launch.py), jadi hasilnya
berlaku SESUDAH op3_manager start ulang.

Jalankan di DALAM container (env ROS sama dengan stack):
    python3 /ros2_ws/scripts/calib_imu_yaw.py
Dari host:
    docker exec -it op3 /docker-entrypoint.sh python3 /ros2_ws/scripts/calib_imu_yaw.py

Kalau op3_manager belum hidup, alat menyalakannya SENDIRI (op3_manager
op3_manager.launch.py) dan menunggu sampai robot di pose INIT (page 2), lalu
mematikannya lagi di akhir -- jadi tidak perlu membuka menu manager dulu. Kalau
op3_manager sudah hidup, dipakai apa adanya dan TIDAK dimatikan.

Kode keluar: 0 tersimpan, 1 gagal, 2 dibatalkan, 3 tabel ditolak.
"""
import argparse
import math
import os
import re
import select
import shutil
import signal
import subprocess
import sys
import termios
import time
import tty

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_OUTPUT = os.path.join(REPO, "src", "ROBOTIS-OP3", "op3_manager", "config",
                              "yaw_calibration.yaml")
DEFAULT_ACTION_FILE = os.path.join(REPO, "src", "ROBOTIS-OP3", "op3_action_module", "data",
                                   "motion_4095_CHRONUS.bin")
STACK_LOG = "/tmp/imu_calib_stack.log"
# op3_manager SAJA, bukan demo.launch.xml. op3_manager sudah mengirim sendiri
# action page 2 (WALKING_READY) saat boot -- lihat goToInitActionPage() di
# op3_manager.cpp -- jadi robot tetap sampai ke pose INIT. Yang hilang cuma
# kamera, detektor bola dan head tracking, yang tidak dipakai kalibrasi ini dan
# justru menyita CPU selama perekaman "robot harus diam". demo.launch.xml pun
# hanya meng-include op3_manager.launch.py tanpa argumen, jadi manager-nya
# identik -- termasuk yaw_calibration.yaml yang dimuat open_cr_module.
LAUNCH_CMD = "ros2 launch op3_manager op3_manager.launch.py"

# Titik rekam: (sudut nyata, petunjuk). Putar KIRI = berlawanan jarum jam dilihat dari atas.
POINTS = [
    (0, "0 deg: hadapkan robot ke arah ACUAN (mis. gawang lawan), TANDAI posisi kaki di lantai"),
    (90, "90 deg KIRI dari acuan"),
    (180, "180 deg: membelakangi acuan"),
    (270, "270 deg KIRI (= 90 deg KANAN dari acuan)"),
    (360, "360 deg: kembali TEPAT ke tanda kaki titik 1"),
]
TRUE_DEG = [0.0, 90.0, 180.0, 270.0]

STILL_GZ_RAD_S = 0.05     # |gyro z| maksimum selama rekam
STILL_SHIFT_DEG = 2.0     # geser rata-rata paruh 1 vs paruh 2 maksimum
MIN_SAMPLES = 20
JITTER_WARN_DEG = 8.0
CLOSE_REJECT_DEG = 15.0   # |titik5 - titik1|
CLOSE_WARN_DEG = 5.0
MIN_SPAN_DEG = 5.0

EXIT_SAVED, EXIT_FAIL, EXIT_CANCEL, EXIT_REJECT = 0, 1, 2, 3

MAG_STATUS_TEXT = {0: "tanpa magnetometer", 1: "settling", 2: "OK", 3: "kalibrasi hard-iron"}


class Cancelled(Exception):
    pass


# --------------------------------------------------------------------------
# Matematika sudut
# --------------------------------------------------------------------------
def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def circ_mean(values):
    s = sum(math.sin(math.radians(v)) for v in values)
    c = sum(math.cos(math.radians(v)) for v in values)
    return math.degrees(math.atan2(s, c)) % 360.0


def forward_span(a, b):
    """Bentang maju a -> b (mod 360), sama dengan open_cr_module."""
    return math.fmod(b - a + 720.0, 360.0)


def build_table(headings):
    """5 heading mentah -> (tabel, catatan, galat). galat None = sah."""
    p1, p2, p3, p4, p5 = headings
    notes = []
    close = wrap180(p5 - p1)
    if abs(close) > CLOSE_REJECT_DEG:
        return None, notes, (f"titik 5 berbeda {close:+.1f} deg dari titik 1 (batas "
                             f"{CLOSE_REJECT_DEG:.0f}): robot tidak kembali ke tanda, atau kompas "
                             "bergeser selama kalibrasi")
    if abs(close) > CLOSE_WARN_DEG:
        notes.append(f"PERINGATAN: titik 5 berbeda {close:+.1f} deg dari titik 1 (> {CLOSE_WARN_DEG:.0f})")
    p0 = circ_mean([p1, p5])
    table = [round(p0, 2), round(p2, 2), round(p3, 2), round(p4, 2)]
    spans = [forward_span(table[i], table[(i + 1) % 4]) for i in range(4)]
    total = sum(spans)
    if abs(total - 360.0) > 1.0:
        return None, notes, (f"total bentang {total:.1f} deg, bukan 360: robot diputar ke KANAN atau "
                             "urutan titik salah. (Kalau urutannya benar, heading firmware tidak "
                             "berputar penuh -- ulangi dulu kalibrasi hard-iron firmware, "
                             "imu_command calibrate_mag, robot diputar 20 detik.)")
    small = [i for i, s in enumerate(spans) if s < MIN_SPAN_DEG]
    if small:
        return None, notes, f"bentang titik {small[0] + 1}->{(small[0] + 1) % 4 + 1} hanya {spans[small[0]]:.1f} deg"
    return {"table": table, "spans": spans, "close": close, "raw": list(headings)}, notes, None


# --------------------------------------------------------------------------
# Env ROS sama dengan stack
# --------------------------------------------------------------------------
def find_manager_pid():
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                argv = f.read().split(b"\0")
        except OSError:
            continue
        if argv and argv[0].endswith(b"lib/op3_manager/op3_manager"):
            return int(pid)
    return None


def manager_ros_env(pid):
    env = {}
    try:
        with open(f"/proc/{pid}/environ", "rb") as f:
            for item in f.read().split(b"\0"):
                k, _, v = item.decode(errors="replace").partition("=")
                if k in ("ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY"):
                    env[k] = v
    except OSError:
        pass
    return env


def sync_ros_env(args):
    """Samakan ROS_DOMAIN_ID/ROS_LOCALHOST_ONLY dengan op3_manager yang hidup.
    Menjalankan ulang proses ini (sebelum rclpy di-import) kalau berbeda."""
    if args.keep_env or os.environ.get("CALIB_IMU_ENV_SYNCED") == "1":
        return
    pid = find_manager_pid()
    if pid is None:
        os.environ.setdefault("ROS_DOMAIN_ID", "1")   # profil real_robot
        return
    want = manager_ros_env(pid)
    diff = {k: v for k, v in want.items() if os.environ.get(k) != v}
    gone = [k for k in ("ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY")
            if k not in want and k in os.environ]
    if not diff and not gone:
        return
    env = dict(os.environ)
    env.update(want)
    for k in gone:
        env.pop(k, None)
    env["CALIB_IMU_ENV_SYNCED"] = "1"
    print(f"env ROS disamakan dengan op3_manager (pid {pid}): "
          + ", ".join(f"{k}={v}" for k, v in want.items()), flush=True)
    os.execve(sys.executable, [sys.executable] + sys.argv, env)


# --------------------------------------------------------------------------
# Stack (hanya kalau op3_manager belum hidup)
# --------------------------------------------------------------------------
class Stack:
    def __init__(self):
        self.proc = None

    def start(self, timeout):
        setup = os.path.join(REPO, "install", "setup.bash")
        env_sh = os.path.join(REPO, "scripts", "op3_env.sh")
        action = os.environ.get("OP3_ACTION_FILE") or DEFAULT_ACTION_FILE
        cmd = (f"[ -f '{env_sh}' ] && source '{env_sh}'; source '{setup}'; "
               f"export ROS_DOMAIN_ID=${{ROS_DOMAIN_ID:-1}}; export OP3_ACTION_FILE='{action}'; "
               f"exec {LAUNCH_CMD}")
        log = open(STACK_LOG, "w")
        print(f"op3_manager belum hidup -> menyalakan sendiri: '{LAUNCH_CMD}' (log: {STACK_LOG})",
              flush=True)
        self.proc = subprocess.Popen(["bash", "-c", cmd], stdout=log, stderr=subprocess.STDOUT,
                                     stdin=subprocess.DEVNULL, start_new_session=True)
        died = re.compile(r"\[op3_manager[^\]]*\]: process has died|op3_manager-\d+\]: process has died")
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            time.sleep(0.5)
            try:
                text = open(STACK_LOG, errors="replace").read()
            except OSError:
                text = ""
            if "Succeed to start page 2" in text:
                print(f"robot di pose INIT (page 2) sesudah {time.monotonic() - t0:.0f} dtk", flush=True)
                return True
            if died.search(text) or self.proc.poll() is not None:
                print("GAGAL: op3_manager mati saat start -- lihat " + STACK_LOG, flush=True)
                return False
            print(f"\r  menunggu pose INIT ... {time.monotonic() - t0:4.0f} dtk", end="", flush=True)
        print(f"\nGAGAL: pose INIT tidak tercapai dalam {timeout:.0f} dtk -- lihat {STACK_LOG}", flush=True)
        return False

    def stop(self):
        if self.proc is None or self.proc.poll() is not None:
            return
        print("mematikan stack yang diluncurkan alat ini ...", flush=True)
        pgid = os.getpgid(self.proc.pid)
        for sig, wait in ((signal.SIGINT, 12.0), (signal.SIGTERM, 5.0), (signal.SIGKILL, 3.0)):
            try:
                os.killpg(pgid, sig)
            except ProcessLookupError:
                return
            try:
                self.proc.wait(timeout=wait)
                return
            except subprocess.TimeoutExpired:
                continue


# --------------------------------------------------------------------------
# ROS
# --------------------------------------------------------------------------
class Sensor:
    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Imu
        from std_msgs.msg import Float32, Float64MultiArray

        self.rclpy = rclpy
        rclpy.init()
        self.node = Node("calib_imu_yaw")
        self.heading = None
        self.heading_time = 0.0
        self.status = None
        self.gz = 0.0
        self.recording = None
        self.node.create_subscription(Float32, "/robotis/open_cr/mag_heading", self._heading, 50)
        # Di robot ini mag_status adalah Float64MultiArray (dipakai Studio); status = data[0].
        self.node.create_subscription(Float64MultiArray, "/robotis/open_cr/mag_status", self._status, 10)
        self.node.create_subscription(Imu, "/robotis/open_cr/imu", self._imu, 50)

    def _heading(self, msg):
        self.heading = float(msg.data)
        self.heading_time = time.monotonic()
        if self.recording is not None:
            self.recording.append((time.monotonic(), self.heading, abs(self.gz)))

    def _status(self, msg):
        if len(msg.data) > 0:
            self.status = int(msg.data[0])

    def _imu(self, msg):
        self.gz = msg.angular_velocity.z

    def spin(self, sec=0.02):
        self.rclpy.spin_once(self.node, timeout_sec=sec)

    def close(self):
        try:
            self.node.destroy_node()
            self.rclpy.shutdown()
        except Exception:
            pass


class Keys:
    """Tombol tanpa Enter (termios cbreak), dikembalikan di finally."""

    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)

    def flush(self):
        termios.tcflush(self.fd, termios.TCIFLUSH)

    def get(self):
        r, _, _ = select.select([self.fd], [], [], 0)
        if not r:
            return None
        return os.read(self.fd, 1).decode(errors="replace")

    def restore(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)


def wait_heading(sensor, timeout=30.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        sensor.spin(0.05)
        if sensor.heading is not None and time.monotonic() - sensor.heading_time < 1.0:
            return True
        print(f"\r  menunggu /robotis/open_cr/mag_heading ... {time.monotonic() - t0:4.0f} dtk "
              f"(mag_status {sensor.status})", end="", flush=True)
    print()
    return False


def record_point(sensor, keys, idx, prompt, sample_sec, prev):
    """Kembalikan (heading, jitter) atau lempar Cancelled."""
    while True:
        print(f"\n[{idx}/5] {prompt}")
        print("      SPASI = rekam, q/Esc = batal")
        keys.flush()
        while True:
            sensor.spin(0.02)
            key = keys.get()
            if key in ("q", "Q", "\x1b"):
                raise Cancelled()
            h = sensor.heading
            st = sensor.status
            live = f"{h:6.1f} deg" if h is not None else "   --   "
            print(f"\r      heading mentah {live}   mag_status {st} ({MAG_STATUS_TEXT.get(st, '?')})"
                  f"   gyro z {sensor.gz:+.3f} rad/s   ", end="", flush=True)
            if key == " ":
                break
        print(f"\n      merekam {sample_sec:.1f} dtk, jangan sentuh robot ...", flush=True)
        sensor.recording = []
        t0 = time.monotonic()
        while time.monotonic() - t0 < sample_sec:
            sensor.spin(0.01)
            key = keys.get()
            if key in ("q", "Q", "\x1b"):
                sensor.recording = None
                raise Cancelled()
        rec, sensor.recording = sensor.recording, None
        if len(rec) < MIN_SAMPLES:
            print(f"      ULANG: hanya {len(rec)} sampel (< {MIN_SAMPLES}) -- mag_heading tersendat?")
            continue
        headings = [r[1] for r in rec]
        gz_max = max(r[2] for r in rec)
        half = len(headings) // 2
        shift = abs(wrap180(circ_mean(headings[:half]) - circ_mean(headings[half:])))
        # Diam dinilai dari GYRO, bukan sebaran heading: di beberapa arah kompas
        # bergetar sendiri +-4..7 deg walau robot diam.
        if gz_max > STILL_GZ_RAD_S or shift > STILL_SHIFT_DEG:
            print(f"      ULANG: robot bergerak (|gyro z| maks {gz_max:.3f} rad/s, batas "
                  f"{STILL_GZ_RAD_S}; geser paruh {shift:.1f} deg, batas {STILL_SHIFT_DEG})")
            continue
        value = circ_mean(headings)
        jitter = math.sqrt(sum(wrap180(h - value) ** 2 for h in headings) / len(headings))
        line = f"      titik {idx}: heading {value:.2f} deg ({len(rec)} sampel, getaran RMS {jitter:.1f} deg"
        if prev is not None:
            line += f", berubah {wrap180(value - prev):+.1f} deg dari titik sebelumnya"
        print(line + ")")
        if jitter > JITTER_WARN_DEG:
            print(f"      PERINGATAN: getaran > {JITTER_WARN_DEG:.0f} deg -- jauhkan logam/kabel/laptop dari robot")
        return value, jitter


# --------------------------------------------------------------------------
# Simpan
# --------------------------------------------------------------------------
def save_yaml(path, result):
    cfg_dir = os.path.dirname(os.path.abspath(path))
    st = os.stat(cfg_dir)
    uid, gid = st.st_uid, st.st_gid
    hist = os.path.join(cfg_dir, "yaw_calibration_history")
    stamp = time.strftime("%Y%m%d_%H%M%S")
    if os.path.exists(path):
        os.makedirs(hist, exist_ok=True)
        backup = os.path.join(hist, f"yaw_calibration_{stamp}.yaml")
        shutil.copy2(path, backup)
        for p in (hist, backup):
            try:
                os.chown(p, uid, gid)
            except OSError:
                pass
        print(f"berkas lama dicadangkan: {backup}")

    raw = result["raw"]
    spans = result["spans"]
    table = result["table"]
    text = (
        "# Kalibrasi yaw IMU (magnetometer OpenCR) 5 titik -- ditulis scripts/calib_imu_yaw.py\n"
        f"# Tanggal: {time.strftime('%Y-%m-%d %H:%M:%S')}\n"
        "# Heading mentah (putar kiri): "
        + ", ".join(f"titik{i + 1} {h:.2f}" for i, h in enumerate(raw)) + "\n"
        f"# Selisih titik 5 vs titik 1: {result['close']:+.2f} deg\n"
        "# Bentang per 90 deg (0->90, 90->180, 180->270, 270->360): "
        + ", ".join(f"{s:.2f}" for s in spans) + "\n"
        "# Berlaku sesudah op3_manager start ulang. Jangan disunting tangan.\n"
        "/**:\n"
        "  ros__parameters:\n"
        "    yaw_calib_enable: true\n"
        "    yaw_calib_measured_heading_deg: [" + ", ".join(f"{v:.2f}" for v in table) + "]\n"
        "    yaw_calib_true_deg: [" + ", ".join(f"{v:.1f}" for v in TRUE_DEG) + "]\n"
    )
    tmp = f"{path}.tmp{os.getpid()}"
    with open(tmp, "w") as f:
        f.write(text)
        f.flush()
        os.fsync(f.fileno())
    try:
        os.chown(tmp, uid, gid)
        os.chmod(tmp, 0o664)
    except OSError:
        pass
    os.replace(tmp, path)
    dfd = os.open(cfg_dir, os.O_RDONLY)
    try:
        os.fsync(dfd)
    finally:
        os.close(dfd)
    print(f"tabel ditulis: {path}")


# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="Kalibrasi yaw IMU (magnetometer OpenCR) 5 titik")
    ap.add_argument("--no-launch", action="store_true", help="jangan meluncurkan stack walau manager mati")
    ap.add_argument("--output", default=DEFAULT_OUTPUT, help="berkas yaml tujuan")
    ap.add_argument("--sample-sec", type=float, default=3.0, help="lama rekam per titik (detik)")
    ap.add_argument("--init-timeout", type=float, default=90.0, help="batas tunggu pose INIT (detik)")
    ap.add_argument("--keep-env", action="store_true",
                    help="jangan samakan ROS_DOMAIN_ID/LOCALHOST_ONLY dengan op3_manager (untuk uji)")
    args = ap.parse_args()

    if not (sys.stdin.isatty() and sys.stdout.isatty()):
        print("GAGAL: alat ini butuh terminal (jalankan dengan docker exec -it / di tmux).")
        return EXIT_FAIL

    sync_ros_env(args)

    stack = Stack()
    manager_was_running = find_manager_pid() is not None
    sensor = None
    keys = None
    try:
        if not manager_was_running and not args.no_launch:
            if not stack.start(args.init_timeout):
                return EXIT_FAIL
        sensor = Sensor()
        print(f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '')} "
              f"ROS_LOCALHOST_ONLY={os.environ.get('ROS_LOCALHOST_ONLY', '')}")
        if not wait_heading(sensor):
            print("GAGAL: /robotis/open_cr/mag_heading tidak mengalir dalam 30 dtk. Kemungkinan: "
                  "op3_manager lama (dibuild sebelum topik ini ada), mag_status belum 2, atau env "
                  "ROS berbeda dengan stack.")
            return EXIT_FAIL
        print("\nKALIBRASI YAW 5 TITIK -- putar robot ke KIRI (berlawanan jarum jam dilihat dari atas).")
        print("Robot tegak dan diam di tiap titik. Jauhkan laptop/logam/kabel dari robot.")
        keys = Keys()
        headings = []
        prev = None
        for idx, (_deg, prompt) in enumerate(POINTS, start=1):
            value, _ = record_point(sensor, keys, idx, prompt, args.sample_sec, prev)
            headings.append(value)
            prev = value
        keys.restore()
        keys = None

        result, notes, err = build_table(headings)
        for n in notes:
            print(n)
        if err is not None:
            print(f"\nTABEL DITOLAK: {err}")
            print("Tidak ada yang disimpan; tabel lama tidak disentuh.")
            return EXIT_REJECT
        print("\nTabel: " + ", ".join(f"{m:.2f}->{t:.0f}" for m, t in zip(result["table"], TRUE_DEG)))
        print("Bentang per 90 deg: " + ", ".join(f"{s:.1f}" for s in result["spans"]))
        save_yaml(args.output, result)
        if manager_was_running:
            print("op3_manager sudah hidup sebelum alat ini: tabel baru berlaku SESUDAH op3_manager "
                  "start ulang (log: 'KALIBRASI YAW AKTIF').")
        else:
            print("Tabel berlaku saat op3_manager start berikutnya (log: 'KALIBRASI YAW AKTIF').")
        return EXIT_SAVED
    except (Cancelled, KeyboardInterrupt):
        print("\nDIBATALKAN -- tidak ada yang disimpan.")
        return EXIT_CANCEL
    finally:
        if keys is not None:
            keys.restore()
        if sensor is not None:
            sensor.close()
        stack.stop()


if __name__ == "__main__":
    sys.exit(main())
