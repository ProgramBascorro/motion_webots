#!/usr/bin/env python3
"""Kalibrasi yaw IMU OpenCR 5 titik -- interaktif, hasilnya disimpan PERMANEN.

Dipanggil dari menu ./script.sh ("Calibration") atau
langsung di dalam container:  python3 /ros2_ws/scripts/calib_imu_yaw.py

Alur:
  1. Kalau stack belum hidup, alat ini meluncurkan launch yang SAMA PERSIS
     dengan perintah demo (ros2 launch op3_demo demo.launch.xml). Robot menyala
     ke pose INIT yang sama dengan sebelum D60 ditekan -- tidak ada
     salinan pose di sini, jadi kalau init demo diubah, init kalibrasi ikut.
  2. Operator memutar robot dengan tangan ke 5 posisi: 0, 90 kiri, 180, 270,
     360 (kembali ke 0). Di tiap posisi, lepaskan robot lalu tekan SPASI.
     Alat membaca mag_heading MENTAH (/robotis/open_cr/mag_heading) selama
     robot diam dan menolak bacaan kalau robot masih bergerak. "Diam" dinilai
     dari GYRO, bukan dari heading: di beberapa arah kompas bergetar sendiri
     +-4..7 deg walau robot diam (18 Sep, arah 270), jadi getaran itu dirata-rata.
  3. Tabel diperiksa (arah putar, total 360, selisih titik 1 vs 5), lalu
     ditulis ke src/ROBOTIS-OP3/op3_manager/config/yaw_calibration.yaml.
     Berkas lama dicadangkan ke config/yaw_calibration_history/.
  4. Stack yang diluncurkan alat ini dimatikan lagi. Launch demo berikutnya
     memakai kalibrasi baru (open_cr_module membacanya saat start).

Yaw 0 saat demo TETAP diatur reset (tombol D60 / /robotis/open_cr/reset_yaw):
tabel ini hanya meluruskan skala sudut magnetometer, titik nolnya tidak.
"""

import argparse
import datetime
import math
import os
import select
import shutil
import signal
import subprocess
import sys
import termios
import time
import tty

WS = "/ros2_ws" if os.path.isdir("/ros2_ws") else os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG = os.path.join(WS, "src/ROBOTIS-OP3/op3_manager/config/yaw_calibration.yaml")
STACK_LOG = "/tmp/imu_calib_stack.log"

POINTS = [
    (0.0, "TITIK 1/5 -- 0 derajat",
     "Hadapkan robot ke ARAH ACUAN (mis. lurus ke gawang lawan). Tandai posisi kakinya di lantai."),
    (90.0, "TITIK 2/5 -- 90 derajat ke KIRI",
     "Putar robot 90 derajat ke KIRI (berlawanan jarum jam dilihat dari atas) dari arah acuan."),
    (180.0, "TITIK 3/5 -- 180 derajat",
     "Putar lagi 90 derajat ke KIRI: robot kini MEMBELAKANGI arah acuan."),
    (270.0, "TITIK 4/5 -- 270 derajat",
     "Putar lagi 90 derajat ke KIRI (= 90 derajat ke KANAN dari arah acuan)."),
    (360.0, "TITIK 5/5 -- 360 derajat (kembali ke 0)",
     "Putar lagi 90 derajat ke KIRI sampai TEPAT kembali ke tanda titik 1."),
]

# Syarat bacaan diam dan tabel sah. Batas tabel sama dengan pemeriksa di
# open_cr_module (total bentang 360 +-1), supaya yang disimpan pasti diterima.
SAMPLE_SEC = 3.0
MIN_SAMPLES = 20
# Robot dianggap diam kalau gyro z (rad/s, /robotis/open_cr/imu) tetap di bawah
# ini selama jendela bacaan; saat diam firmware melapor tepat 0. Heading boleh
# bergetar, asal rata-rata paruh pertama dan kedua jendela tidak bergeser.
STILL_GYRO_RAD_S = 0.05
STILL_DRIFT_DEG = 2.0
NOISY_JITTER_DEG = 8.0
MIN_SPAN_DEG = 5.0
CLOSURE_WARN_DEG = 5.0
CLOSURE_REJECT_DEG = 15.0


# ---------------------------------------------------------------- matematika

def fwd(a, b):
    """Bentang maju a -> b dalam derajat, 0..360."""
    return (b - a) % 360.0


def signed(a, b):
    """Selisih bertanda b - a, (-180, 180]."""
    d = (b - a) % 360.0
    return d - 360.0 if d > 180.0 else d


def circ_mean(values):
    s = sum(math.sin(math.radians(v)) for v in values)
    c = sum(math.cos(math.radians(v)) for v in values)
    return math.degrees(math.atan2(s, c)) % 360.0


def circ_rms(values):
    """Getaran: akar rata-rata kuadrat simpangan dari rata-rata melingkar (deg)."""
    m = circ_mean(values)
    return math.sqrt(sum(signed(m, v) ** 2 for v in values) / len(values)) if values else float("inf")


def judge_still(values, gyro):
    """(ok, alasan, rata-rata, getaran). values = heading urut waktu, gyro = |z| rad/s."""
    if gyro and max(gyro) > STILL_GYRO_RAD_S:
        return False, "robot masih bergerak (gyro %.2f rad/s)" % max(gyro), None, None
    half = len(values) // 2
    drift = signed(circ_mean(values[:half]), circ_mean(values[half:]))
    if abs(drift) > STILL_DRIFT_DEG:
        return False, "heading masih bergeser %+.1f deg dalam jendela bacaan" % drift, None, None
    return True, "", circ_mean(values), circ_rms(values)


def build_table(measured):
    """measured = heading di titik 0, 90, 180, 270, 360.

    Kembalian: (heading_tabel[4], true[4], info, masalah[]). Titik 0 tabel =
    rata-rata melingkar titik 1 dan 5, sama dengan kalibrasi 17 Sep.
    """
    m0, m90, m180, m270, m360 = measured
    closure = signed(m0, m360)
    p0 = circ_mean([m0, m360])
    table = [p0, m90, m180, m270]
    true = [0.0, 90.0, 180.0, 270.0]
    spans = [fwd(table[i], table[(i + 1) % 4]) for i in range(4)]
    total = sum(spans)
    problems = []
    if abs(total - 360.0) > 1.0:
        # Total 720/1080 = heading bergerak ke arah sebaliknya di sebagian atau
        # semua langkah: robot diputar ke KANAN, atau urutan titik tertukar.
        problems.append("bentang tidak berjumlah 360 (total %.1f): robot diputar ke arah KANAN, atau urutan "
                        "titik tertukar. Semua putaran harus ke KIRI." % total)
    for i, span in enumerate(spans):
        if span < MIN_SPAN_DEG:
            problems.append("bentang %d->%d derajat hanya %.1f (heading hampir tidak berubah): robot tidak "
                            "berputar, atau magnetometer terganggu logam di dekatnya." % (true[i], (true[i] + 90) % 360, span))
    if abs(closure) > CLOSURE_REJECT_DEG:
        problems.append("titik 5 berbeda %.1f derajat dari titik 1 (batas %.0f): robot tidak kembali ke tanda "
                        "awal, atau medan magnet berubah selama kalibrasi." % (closure, CLOSURE_REJECT_DEG))
    info = {"closure": closure, "spans": spans, "total": total, "p0": p0}
    return table, true, info, problems


def keep_hard_iron(path):
    """Baris kalibrasi HARD-IRON milik berkas lama, supaya tidak ikut terhapus.

    Berkas ini menampung dua kalibrasi yang berbeda umur: tabel yaw 5 titik (di
    sini) dan offset hard-iron magnetometer (direkam terpisah, dipakai
    open_cr_module untuk menghitung heading di PC). Menulis ulang berkas tanpa
    membawa serta baris hard-iron akan MENGHAPUS kalibrasi itu diam-diam.
    """
    kept = []
    try:
        with open(path, encoding="utf-8") as f:
            lines = f.readlines()
        for line in lines:
            stripped = line.strip()
            if stripped.startswith("mag_hard_iron_"):
                kept.append("    " + stripped)
    except OSError:
        pass
    return kept


def render_yaml(measured, table, true, info, stamp, hard_iron=()):
    spans = " / ".join("%.1f" % s for s in info["spans"])
    return f"""# KALIBRASI YAW 5 TITIK untuk open_cr_module (satu sumber untuk semua cara
# menyalakan op3_manager: op3_manager.launch.py -- dipakai demo.launch.xml --,
# op3_tmux profil real_robot, scripts/wait_u2d2_then_manager.sh).
#
# DIBUAT OTOMATIS oleh scripts/calib_imu_yaw.py (menu ./script.sh), {stamp}.
# Robot diputar manual ke KIRI 0/90/180/270/360 deg dalam pose INIT demo,
# heading dibaca MENTAH dari /robotis/open_cr/mag_heading saat robot diam.
#   heading mentah 5 titik : {measured[0]:.2f} / {measured[1]:.2f} / {measured[2]:.2f} / {measured[3]:.2f} / {measured[4]:.2f}
#   titik 5 vs titik 1     : {info['closure']:+.2f} deg (titik 0 tabel = rata-rata keduanya)
#   bentang per 90 deg nyata: {spans} deg
#
# Ulangi kalibrasi kalau OpenCR, baterai, atau kabel dipindah, atau di lapangan
# lain. yaw_calib_enable: false = yaw firmware mentah seperti dulu.
# Berkas sebelumnya dicadangkan di config/yaw_calibration_history/.
#
# Baris mag_hard_iron_* (kalau ada) DIBAWA dari berkas sebelumnya: itu kalibrasi
# hard-iron magnetometer yang direkam terpisah dan dipakai open_cr_module untuk
# menghitung heading di mini PC. Jangan dihapus saat mengedit tangan.
/**:
  ros__parameters:
{{hard_iron_block}}    yaw_calib_enable: true
    yaw_calib_measured_heading_deg: [{', '.join('%.2f' % v for v in table)}]
    yaw_calib_true_deg: [{', '.join('%.1f' % v for v in true)}]
""".replace("{hard_iron_block}", "".join(line + "\n" for line in hard_iron))


def save(path, text):
    """Cadangkan berkas lama, tulis atomik, kembalikan pemilik ke pemilik folder."""
    folder = os.path.dirname(os.path.abspath(path))
    history = os.path.join(folder, "yaw_calibration_history")
    backup = None
    if os.path.exists(path):
        os.makedirs(history, exist_ok=True)
        backup = os.path.join(history, "yaw_calibration_%s.yaml" % datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
        shutil.copy2(path, backup)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(text)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)
    # Ditulis root dari dalam container ke repo milik operator di host.
    st = os.stat(folder)
    for p in [path] + ([backup, history] if backup else []):
        try:
            os.chown(p, st.st_uid, st.st_gid)
        except OSError:
            pass
    return backup


# ------------------------------------------------------------ stack & ROS

def stack_running():
    out = subprocess.run(["pgrep", "-f", "[o]p3_manager/lib/op3_manager/op3_manager"],
                         capture_output=True, text=True).stdout.strip()
    return bool(out)


class Stack:
    """Launch demo milik alat ini (kalau belum ada yang hidup)."""

    def __init__(self):
        self.proc = None

    def start(self):
        log = open(STACK_LOG, "w")
        self.proc = subprocess.Popen(["ros2", "launch", "op3_demo", "demo.launch.xml"], stdout=log,
                                     stderr=subprocess.STDOUT, start_new_session=True)

    def wait_init(self, timeout):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if self.proc.poll() is not None:
                return False, "launch berhenti (kode %s)" % self.proc.returncode
            with open(STACK_LOG, errors="replace") as f:
                text = f.read()
            if "process has died" in text and "op3_manager" in text.split("process has died")[-1][:200]:
                return False, "op3_manager mati saat start (lihat %s)" % STACK_LOG
            if "Succeed to start page 2" in text:
                return True, "pose INIT (page 2) selesai %.1f s" % (time.monotonic() - t0)
            time.sleep(0.5)
        return False, "pose INIT tidak selesai dalam %.0f s (lihat %s)" % (timeout, STACK_LOG)

    def stop(self):
        if self.proc is None or self.proc.poll() is not None:
            return
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


class Keys:
    """Satu tombol tanpa Enter (cbreak); dikembalikan seperti semula saat keluar."""

    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        termios.tcflush(self.fd, termios.TCIFLUSH)

    def get(self, timeout):
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        return os.read(self.fd, 1).decode(errors="ignore") if r else None

    def flush(self):
        termios.tcflush(self.fd, termios.TCIFLUSH)

    def restore(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)


def run(args):
    # Domain ROS diambil dari op3_manager yang hidup: peluncur tmux profil
    # real_robot memakai ROS_DOMAIN_ID 1, shell entrypoint container 42. Tanpa
    # ini alat berkata "tidak ada data" padahal stack sehat. WAJIB sebelum
    # rclpy.init(), karena nilainya dibaca saat context dibuat.
    pid = subprocess.run(["pgrep", "-f", "[o]p3_manager/lib/op3_manager/op3_manager"],
                         capture_output=True, text=True).stdout.split()
    if pid:
        try:
            with open("/proc/%s/environ" % pid[0], "rb") as f:
                for entry in f.read().decode("utf-8", "replace").split("\0"):
                    name, _, value = entry.partition("=")
                    if name in ("ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "RMW_IMPLEMENTATION"):
                        os.environ[name] = value
        except OSError:
            pass
        print("memakai env ROS op3_manager (pid %s): domain %s, localhost_only %s" %
              (pid[0], os.environ.get("ROS_DOMAIN_ID"), os.environ.get("ROS_LOCALHOST_ONLY")))

    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32, UInt8
    from sensor_msgs.msg import Imu

    rclpy.init()
    node = Node("calib_imu_yaw")
    state = {"heading": None, "t": 0.0, "status": None, "buf": [], "gyro": []}

    def on_heading(msg):
        state["heading"] = float(msg.data)
        state["t"] = time.monotonic()
        state["buf"].append((state["t"], state["heading"]))
        if len(state["buf"]) > 2000:
            del state["buf"][:1000]

    def on_imu(msg):
        state["gyro"].append((time.monotonic(), abs(msg.angular_velocity.z)))
        if len(state["gyro"]) > 2000:
            del state["gyro"][:1000]

    def on_status(msg):
        state["status"] = int(msg.data)

    node.create_subscription(Float32, "/robotis/open_cr/mag_heading", on_heading, 50)
    node.create_subscription(UInt8, "/robotis/open_cr/mag_status", on_status, 10)
    node.create_subscription(Imu, "/robotis/open_cr/imu", on_imu, 50)

    def spin(sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=min(0.02, max(0.0, end - time.monotonic())))

    # Heading mentah harus mengalir sebelum mulai.
    t0 = time.monotonic()
    while time.monotonic() - t0 < 30.0 and (state["heading"] is None or time.monotonic() - state["t"] > 0.5):
        spin(0.1)
    if state["heading"] is None:
        print("\nGAGAL: tidak ada data di /robotis/open_cr/mag_heading dalam 30 s.")
        print("  - op3_manager lama (sebelum 18 Sep) belum menerbitkan topik ini: matikan stack lalu jalankan lagi;")
        print("  - atau magnetometer belum siap (mag_status %s, harus 2)." % state["status"])
        return 1

    keys = Keys()
    measured = []
    try:
        for true_deg, title, hint in POINTS:
            print("\n\033[1m%s\033[0m\n  %s\n  Lepaskan robot sampai diam, lalu tekan SPASI.  (q = batal, tidak ada yang disimpan)"
                  % (title, hint))
            keys.flush()
            while True:
                key = None
                while key is None:
                    spin(0.2)
                    age = time.monotonic() - state["t"]
                    live = "%6.1f deg" % state["heading"] if age < 0.5 else "  (data berhenti!)"
                    sys.stdout.write("\r  heading mentah %s  mag_status %s   " % (live, state["status"]))
                    sys.stdout.flush()
                    key = keys.get(0.0)
                if key in ("q", "Q", "\x1b"):
                    print("\n\nDIBATALKAN. Kalibrasi lama tidak diubah.")
                    return 2
                if key != " ":
                    continue
                sys.stdout.write("\r  membaca %.1f s, jangan sentuh robot...                  " % args.sample_sec)
                sys.stdout.flush()
                start = time.monotonic()
                spin(args.sample_sec)
                values = [h for (t, h) in state["buf"] if t >= start]
                if len(values) < MIN_SAMPLES:
                    print("\n  Data terlalu sedikit (%d sampel). Tekan SPASI lagi." % len(values))
                    continue
                gyro = [g for (t, g) in state["gyro"] if t >= start]
                ok, why, value, jitter = judge_still(values, gyro)
                if not ok:
                    print("\n  Belum diam: %s. Lepaskan robot, tunggu sebentar, tekan SPASI lagi." % why)
                    continue
                msg = "\n  TERCATAT: heading %.2f deg (%d sampel, getaran kompas %.1f deg)" % (value, len(values), jitter)
                if jitter > NOISY_JITTER_DEG:
                    msg += "\n  PERINGATAN: kompas bergetar kuat di arah ini -- jauhkan logam/kabel/laptop dari robot."
                if measured:
                    msg += ", berubah %+.1f deg dari titik sebelumnya" % signed(measured[-1], value)
                print(msg)
                measured.append(value)
                break
    finally:
        keys.restore()

    table, true, info, problems = build_table(measured)
    print("\n\033[1mHASIL\033[0m")
    for (true_deg, title, _), m in zip(POINTS, measured):
        print("  %-42s heading %7.2f" % (title, m))
    print("  titik 5 vs titik 1        : %+.2f deg%s" % (
        info["closure"], "  (PERINGATAN: > %.0f deg)" % CLOSURE_WARN_DEG if abs(info["closure"]) > CLOSURE_WARN_DEG else ""))
    print("  bentang per 90 deg nyata  : %s  (total %.1f)" % (" / ".join("%.1f" % s for s in info["spans"]), info["total"]))
    if problems:
        print("\n\033[31mTIDAK DISIMPAN\033[0m -- tabel ini akan ditolak open_cr_module atau salah:")
        for p in problems:
            print("  - " + p)
        print("Kalibrasi lama tidak diubah. Ulangi dari menu.")
        return 3

    hard_iron = keep_hard_iron(args.output)
    text = render_yaml(measured, table, true, info, datetime.datetime.now().strftime("%Y-%m-%d %H:%M"), hard_iron)
    backup = save(args.output, text)
    print("\n\033[32mTERSIMPAN PERMANEN\033[0m: %s" % args.output)
    if hard_iron:
        print("  kalibrasi hard-iron magnetometer ikut dipertahankan: %s" % "; ".join(l.strip() for l in hard_iron))
    if backup:
        print("  kalibrasi lama dicadangkan: %s" % backup)
    print("  Dipakai otomatis setiap op3_manager start (demo, D60). Reset yaw D60 tetap")
    print("  membuat arah saat itu = 0, dengan sudut yang sudah dikoreksi tabel ini.")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-launch", action="store_true",
                    help="jangan meluncurkan stack; pakai op3_manager yang sudah hidup")
    ap.add_argument("--output", default=CONFIG, help="berkas hasil (bawaan: yaw_calibration.yaml op3_manager)")
    ap.add_argument("--sample-sec", type=float, default=SAMPLE_SEC, help="lama membaca tiap titik (detik)")
    ap.add_argument("--init-timeout", type=float, default=90.0)
    args = ap.parse_args()

    if not sys.stdin.isatty():
        print("Alat ini interaktif (butuh terminal untuk tombol SPASI).")
        return 1

    stack = Stack()
    owned = False
    if stack_running():
        print("op3_manager sudah hidup: dipakai apa adanya (tidak diluncurkan ulang, tidak dimatikan).")
        print("Pastikan robot sudah di pose INIT dan TIDAK ada demo D60 yang berjalan.")
    elif args.no_launch:
        print("--no-launch: stack tidak diluncurkan; hanya membaca topik yang sudah ada.")
    else:
        print("Menyalakan robot ke pose INIT lewat launch demo (sama dengan perintah demo)...")
        print("  log: %s" % STACK_LOG)
        stack.start()
        owned = True

    rc = 1
    try:
        if owned:
            ok, why = stack.wait_init(args.init_timeout)
            print("  " + why)
            if not ok:
                return 1
            print("  Robot di pose INIT. JANGAN tekan D60 selama kalibrasi.")
        rc = run(args)
    except KeyboardInterrupt:
        print("\n\nDIBATALKAN (Ctrl+C). Kalibrasi lama tidak diubah.")
        rc = 2
    finally:
        if owned:
            print("\nMematikan stack kalibrasi...")
            stack.stop()
            print("  selesai. Jalankan demo seperti biasa; kalibrasi terbaca saat op3_manager start.")
        else:
            print("\nStack yang sudah hidup TIDAK dimatikan. Kalibrasi baru baru berlaku sesudah op3_manager"
                  " dijalankan ulang.")
        try:
            import rclpy
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
    return rc


if __name__ == "__main__":
    sys.exit(main())
