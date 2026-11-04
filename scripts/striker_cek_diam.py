#!/usr/bin/env python3
"""Uji DIAM demo_striker -- robot duduk / menggantung, TIDAK berdiri.

Stack yang dipakai (Terminal 1, di container lewat `bash scripts/op3_docker.sh shell`):

    ros2 launch op3_demo demo_striker.launch.xml with_striker_node:=false

Itu menjalankan manager (servo menahan pose sekarang, TANPA page 2), kamera +
detektor bola, head tracker (diam), dan 2 detektor kiper/gawang (mati), tapi
TIDAK menjalankan op_striker_node -- jadi tidak ada sendi yang digerakkan.

Lalu di Terminal 2 (container juga):

    python3 /ros2_ws/scripts/striker_cek_diam.py gyro      # yaw gyro: tanda, skala, drift
    python3 /ros2_ws/scripts/striker_cek_diam.py deteksi   # kiper (robot) & tiang (goalpost)
    python3 /ros2_ws/scripts/striker_cek_diam.py gerbang   # bola: servo 19/20 + putusan B/A/C/D
                                                           # (HANYA kepala yang bergerak)

Rumus di sini SAMA dengan StrikerDemo / striker_strategy.cpp, dan semua angka
dibaca dari op3_demo/config/striker_params.yaml (+ ambang servo dari
demo_striker.launch.xml), jadi yang diuji adalah setelan yang akan dipakai.
Magnetometer hanya ditampilkan sebagai PEMBANDING di uji gyro; striker sendiri
tidak memakainya.
"""

import argparse
import math
import os
import sys
import threading
import time
import xml.etree.ElementTree as ET

import yaml

WS = "/ros2_ws" if os.path.isdir("/ros2_ws") else os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEMO = os.path.join(WS, "src/ROBOTIS-OP3-Demo/op3_demo")
PARAMS = os.path.join(DEMO, "config/striker_params.yaml")
LAUNCH = os.path.join(DEMO, "launch/demo_striker.launch.xml")
NAMA_BOLA = ["B", "A", "C", "D"]

# Bawaan StrikerDemo::declareParameters() untuk nilai yang TIDAK ada di launch.
SERVO_CENTER_DEG = 180.0
PAN_LEFT_LIMIT_DEG = 85.0
PAN_RIGHT_LIMIT_DEG = -85.0


def muat_setelan():
    with open(PARAMS, encoding="utf-8") as f:
        p = yaml.safe_load(f)["/**"]["ros__parameters"]
    tilt_limit = 200.0
    try:
        for arg in ET.parse(LAUNCH).getroot().iter("arg"):
            if arg.get("name") == "auto_demo_tilt_limit_deg" and arg.get("default"):
                tilt_limit = float(arg.get("default"))
    except (ET.ParseError, OSError, ValueError):
        pass
    p["_tilt_limit_deg"] = tilt_limit
    return p


def normal_deg(d):
    d = math.fmod(d, 360.0)
    if d > 180.0:
        d -= 360.0
    elif d <= -180.0:
        d += 360.0
    return d


def stamp_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def arah_piksel_deg(x_px, pan_deg, p):
    """Arah kolom piksel relatif badan, + = kiri (= striker::arahPikselDeg)."""
    x_norm = 2.0 * x_px / float(p["striker_image_width_px"]) - 1.0
    return pan_deg - math.degrees(math.atan(x_norm * math.tan(math.radians(p["striker_half_hfov_deg"]))))


# ---------------------------------------------------------------------------
#  ROS
# ---------------------------------------------------------------------------

def buat_node(nama):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    rclpy.init()
    node = rclpy.create_node(nama)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    node.cek_executor = executor
    node.cek_thread = threading.Thread(target=executor.spin, daemon=True)
    node.cek_thread.start()
    return node


def tutup(node):
    import rclpy
    try:
        # Utas spin HARUS selesai sebelum interpreter keluar; kalau tidak, utas
        # C++ yang masih menunggu membuat proses "terminate" + dump core.
        node.cek_executor.shutdown(timeout_sec=2.0)
        node.cek_thread.join(timeout=2.0)
        node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def node_ada(node, nama, tunggu=3.0):
    akhir = time.monotonic() + tunggu
    while time.monotonic() < akhir:
        if nama in node.get_node_names():
            return True
        time.sleep(0.2)
    return False


def cek_tidak_ada_striker(node):
    if node_ada(node, "striker_node", tunggu=1.5):
        print("\n[STOP] op_striker_node sedang jalan. Uji diam ini butuh launch:\n"
              "  ros2 launch op3_demo demo_striker.launch.xml with_striker_node:=false\n")
        return False
    return True


def enter(pesan):
    print()
    print(pesan)
    try:
        input("   ...lalu tekan ENTER ")
    except EOFError:
        pass


# ---------------------------------------------------------------------------
#  1. GYRO
# ---------------------------------------------------------------------------

class PencatatGyro:
    """Integrasi gyro z persis striker::GyroYaw: bias mentah, dt <= gyro_max_dt_sec."""

    def __init__(self, max_dt):
        self.lock = threading.Lock()
        self.max_dt = max_dt
        self.last = None
        self.n = 0
        self.kalibrasi = True
        self.sampel_bias = []
        self.bias = 0.0
        self.yaw_mentah = 0.0          # tanda +1 (angular_velocity.z apa adanya)
        self.laju_baru = []            # (waktu, laju mentah) 0,6 s terakhir
        self.mag = None
        self.mag_berubah = False

    def callback(self, msg):
        t = stamp_sec(msg.header.stamp)
        laju = math.degrees(msg.angular_velocity.z)
        q = msg.orientation
        mag = math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        with self.lock:
            self.n += 1
            if self.mag is not None and abs(mag - self.mag) > 1e-6:
                self.mag_berubah = True
            self.mag = mag
            self.laju_baru.append((t, laju))
            while self.laju_baru and t - self.laju_baru[0][0] > 0.6:
                self.laju_baru.pop(0)
            if self.last is not None:
                dt = t - self.last
                if dt > 0.0:
                    dt = min(dt, self.max_dt)
                    if self.kalibrasi:
                        self.sampel_bias.append(laju)
                    else:
                        self.yaw_mentah = normal_deg(self.yaw_mentah + (laju - self.bias) * dt)
            self.last = t

    def bergerak(self):
        with self.lock:
            if not self.laju_baru:
                return True
            return max(abs(l - self.bias) for _, l in self.laju_baru) > 3.0

    def baca(self):
        with self.lock:
            return self.yaw_mentah, self.mag


def jalankan_gyro(args, p):
    from sensor_msgs.msg import Imu
    sign = -1.0 if float(p["striker_gyro_z_sign"]) < 0.0 else 1.0
    node = buat_node("striker_cek_gyro")
    g = PencatatGyro(float(p["striker_gyro_max_dt_sec"]))
    node.create_subscription(Imu, "/robotis/open_cr/imu", g.callback, 50)
    try:
        if not cek_tidak_ada_striker(node):
            return 1
        print("Menunggu /robotis/open_cr/imu ...")
        akhir = time.monotonic() + 15.0
        while g.n == 0 and time.monotonic() < akhir:
            time.sleep(0.1)
        if g.n == 0:
            print("[GAGAL] IMU tidak terbit. Launch with_striker_node:=false sudah jalan dan manager siap?")
            return 1
        n0 = g.n
        time.sleep(1.0)
        print("IMU %.0f Hz. Tanda yang dipakai striker sekarang: striker_gyro_z_sign = %+.0f" % (g.n - n0, sign))

        # ---- langkah 1: bias -------------------------------------------------
        while True:
            enter("LANGKAH 1/6 -- Robot di TANDA 0 derajat. LEPASKAN robot, JANGAN DISENTUH.\n"
                  "   Bias gyro diukur %.0f detik sesudah ENTER." % args.bias_detik)
            with g.lock:
                g.kalibrasi = True
                g.sampel_bias = []
            for sisa in range(int(args.bias_detik), 0, -1):
                print("   diam... %d" % sisa)
                time.sleep(1.0)
            with g.lock:
                sampel = list(g.sampel_bias)
                if len(sampel) < 10:
                    print("   [ULANG] sampel IMU terlalu sedikit (%d)" % len(sampel))
                    continue
                rata = sum(sampel) / len(sampel)
                simpang = max(abs(s - rata) for s in sampel)
                if simpang > 2.0:
                    print("   [ULANG] robot BERGERAK saat diukur (simpangan %.1f deg/s). Jangan disentuh." % simpang)
                    continue
                g.bias = rata
                g.yaw_mentah = 0.0
                g.kalibrasi = False
                mag0 = g.mag
            print("   bias gyro %+.3f deg/s (simpangan %.2f). Yaw = 0 sekarang." % (rata, simpang))
            break

        def ambil(pesan):
            while True:
                enter(pesan)
                if g.bergerak():
                    print("   [TAHAN] robot masih bergerak -- tahan DIAM di tanda, lalu ENTER lagi.")
                    continue
                yaw, mag = g.baca()
                dmag = normal_deg(mag - mag0) if (g.mag_berubah and mag is not None and mag0 is not None) else None
                print("   yaw gyro (tanda yaml %+.0f) = %+6.1f deg | mentah %+6.1f%s"
                      % (sign, sign * yaw, yaw, "" if dmag is None else " | magnetometer (pembanding) %+6.1f" % dmag))
                return yaw, dmag

        kiri, mag_kiri = ambil("LANGKAH 2/6 -- PUTAR robot ke KIRI sampai tanda 90 derajat\n"
                               "   (BERLAWANAN jarum jam dilihat dari ATAS). Putar pelan, lalu TAHAN DIAM.")
        nol1, _ = ambil("LANGKAH 3/6 -- KEMBALIKAN robot ke tanda 0 derajat, lalu TAHAN DIAM.")
        kanan, mag_kanan = ambil("LANGKAH 4/6 -- PUTAR robot ke KANAN sampai tanda 90 derajat\n"
                                 "   (SEARAH jarum jam dilihat dari ATAS), lalu TAHAN DIAM.")
        nol2, _ = ambil("LANGKAH 5/6 -- KEMBALIKAN robot ke tanda 0 derajat, lalu TAHAN DIAM.")

        enter("LANGKAH 6/6 -- LEPASKAN robot. JANGAN DISENTUH selama %.0f detik (drift)." % args.drift_detik)
        awal, _ = g.baca()
        t0 = time.monotonic()
        while time.monotonic() - t0 < args.drift_detik:
            time.sleep(min(10.0, args.drift_detik - (time.monotonic() - t0)))
            sekarang, _ = g.baca()
            print("   %3.0f s: yaw %+6.2f deg (berubah %+5.2f)"
                  % (time.monotonic() - t0, sign * sekarang, sign * normal_deg(sekarang - awal)))
        akhir_yaw, _ = g.baca()
        drift = abs(normal_deg(akhir_yaw - awal)) * 60.0 / max(1.0, args.drift_detik)

        # ---- ringkasan ---------------------------------------------------------
        tanda_benar = 1.0 if kiri > 0.0 else -1.0
        print("\n================ RINGKASAN UJI GYRO ================")
        print("Putar KIRI terbaca mentah %+.1f -> tanda yang BENAR: striker_gyro_z_sign: %.1f %s"
              % (kiri, tanda_benar, "(SAMA dengan yaml, OK)" if tanda_benar == sign
                 else "(BEDA dengan yaml %+.0f -- UBAH di config/striker_params.yaml)" % sign))
        print("Konsisten: kanan mentah %+.1f %s" % (kanan, "(berlawanan tanda dgn kiri, OK)"
                                                     if kanan * kiri < 0 else "(TANDA SAMA dgn kiri -- ANEH, ulangi)"))
        for nama, nilai, mag in (("kiri", kiri, mag_kiri), ("kanan", kanan, mag_kanan)):
            s = "Skala %-5s: |%.1f| / 90 = %.1f %%" % (nama, nilai, abs(nilai) / 90.0 * 100.0)
            if mag is not None and abs(mag) > 5.0:
                s += "  | vs magnetometer %.1f %%" % (abs(nilai) / abs(mag) * 100.0)
            print(s + ("  OK" if abs(abs(nilai) - 90.0) <= 4.5 else "  (> 5 % -- cek tanda lakban/ulangi)"))
        print("Kembali ke 0: %+.1f dan %+.1f deg %s" % (nol1, nol2,
              "OK" if max(abs(nol1), abs(nol2)) <= 3.0 else "(> 3 deg)"))
        print("Drift diam  : %.2f deg/menit %s" % (drift, "OK" if drift <= 3.0 else "(> 3 deg/menit)"))
        print("====================================================")
        return 0
    except KeyboardInterrupt:
        print("\n(dihentikan)")
        return 130
    finally:
        tutup(node)


# ---------------------------------------------------------------------------
#  2. DETEKSI kiper (robot) & tiang (goalpost)
# ---------------------------------------------------------------------------

def jalankan_deteksi(args, p):
    from op3_ball_detector_msgs.msg import CircleSetStamped
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool
    from vision_msgs.msg import Detection2DArray

    node = buat_node("striker_cek_deteksi")
    lock = threading.Lock()
    data = {"kiper": None, "tiang": None, "bola": None, "pan": None}

    def simpan(kunci):
        def cb(msg):
            with lock:
                data[kunci] = (time.monotonic(), msg)
        return cb

    def cb_sendi(msg):
        for nama, pos in zip(msg.name, msg.position):
            if nama == "head_pan":
                with lock:
                    data["pan"] = math.degrees(pos)

    node.create_subscription(Detection2DArray, "/kiper_detector_node/detections", simpan("kiper"), 10)
    node.create_subscription(Detection2DArray, "/gawang_detector_node/detections", simpan("tiang"), 10)
    node.create_subscription(CircleSetStamped, "/ball_detector_node/circle_set", simpan("bola"), 10)
    node.create_subscription(JointState, "/robotis/present_joint_states", cb_sendi, 10)
    pub_kiper = node.create_publisher(Bool, "/kiper_detector_node/enable", 10)
    pub_tiang = node.create_publisher(Bool, "/gawang_detector_node/enable", 10)

    def nyalakan(on):
        for pub in (pub_kiper, pub_tiang):
            pub.publish(Bool(data=on))

    try:
        if not cek_tidak_ada_striker(node):
            return 1
        akhir = time.monotonic() + 8.0
        while (pub_kiper.get_subscription_count() == 0 or pub_tiang.get_subscription_count() == 0) \
                and time.monotonic() < akhir:
            time.sleep(0.2)
        if pub_kiper.get_subscription_count() == 0 or pub_tiang.get_subscription_count() == 0:
            print("[GAGAL] detektor kiper/gawang tidak jalan (kiper %d, gawang %d pelanggan enable).\n"
                  "Jalankan: ros2 launch op3_demo demo_striker.launch.xml with_striker_node:=false"
                  % (pub_kiper.get_subscription_count(), pub_tiang.get_subscription_count()))
            return 1
        enter("DETEKSI -- Taruh robot lain (kiper) dan/atau tiang gawang 1-2 m DI DEPAN kamera.\n"
              "   Selama %.0f detik geser objek ke KIRI lalu ke KANAN; arah harus + di kiri, - di kanan.\n"
              "   Ambang striker: robot >= %.2f, goalpost >= %.2f (tiang butuh 2 untuk tengah gawang)."
              % (args.detik, p["striker_keeper_min_conf"], p["striker_goalpost_min_conf"]))
        nyalakan(True)
        t0 = time.monotonic()
        nyala_lagi = t0
        hitung = {"kiper": 0, "tiang2": 0, "baris": 0}
        while time.monotonic() - t0 < args.detik:
            time.sleep(0.5)
            now = time.monotonic()
            if now - nyala_lagi > 1.0:
                nyalakan(True)   # jaga-jaga kalau pesan pertama hilang
                nyala_lagi = now
            with lock:
                kiper, tiang, bola, pan = data["kiper"], data["tiang"], data["bola"], data["pan"]
            pan = pan if pan is not None else 0.0
            baris = "t=%4.1f pan %+5.1f |" % (now - t0, pan)

            if kiper and now - kiper[0] < 1.0:
                kotak = [(d.results[0].hypothesis.score if d.results else 0.0, d) for d in kiper[1].detections]
                sah = [k for k in kotak if k[0] >= p["striker_keeper_min_conf"]]
                if sah:
                    conf, d = max(sah, key=lambda k: k[0])
                    hitung["kiper"] += 1
                    baris += " KIPER conf %.2f cx %3.0f w %3.0f arah %+5.1f |" % (
                        conf, d.bbox.center.position.x, d.bbox.size_x, arah_piksel_deg(d.bbox.center.position.x, pan, p))
                else:
                    baris += " kiper - (%d kotak di bawah ambang) |" % len(kotak)
            else:
                baris += " kiper (detektor diam) |"

            if tiang and now - tiang[0] < 1.0:
                cx = [d.bbox.center.position.x for d in tiang[1].detections
                      if (d.results[0].hypothesis.score if d.results else 0.0) >= p["striker_goalpost_min_conf"]]
                if len(cx) >= 2:
                    tengah = 0.5 * (min(cx) + max(cx))
                    hitung["tiang2"] += 1
                    baris += " GAWANG %d tiang tengah %3.0f arah %+5.1f |" % (len(cx), tengah, arah_piksel_deg(tengah, pan, p))
                else:
                    baris += " gawang %d tiang (perlu 2) |" % len(cx)
            else:
                baris += " gawang (detektor diam) |"

            if bola and now - bola[0] < 1.0 and bola[1].circles:
                c = bola[1].circles[0]
                baris += " bola x %+.2f y %+.2f r %.0f" % (c.x, c.y, c.z)
            else:
                baris += " bola -"
            hitung["baris"] += 1
            print(baris)
        print("\nRingkasan: kiper sah di %d/%d baris, gawang (>=2 tiang) di %d/%d baris."
              % (hitung["kiper"], hitung["baris"], hitung["tiang2"], hitung["baris"]))
        return 0
    except KeyboardInterrupt:
        print("\n(dihentikan)")
        return 130
    finally:
        for _ in range(3):
            nyalakan(False)
            time.sleep(0.1)
        print("detektor kiper/gawang DIMATIKAN lagi.")
        tutup(node)


# ---------------------------------------------------------------------------
#  3. GERBANG bola (kepala saja bergerak)
# ---------------------------------------------------------------------------

def jalankan_gerbang(args, p):
    from op3_ball_detector_msgs.msg import CircleSetStamped
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    node = buat_node("striker_cek_gerbang")
    lock = threading.Lock()
    data = {"bola": None, "status": None, "pan": None, "tilt": None}

    def cb_bola(msg):
        with lock:
            data["bola"] = (time.monotonic(), msg)

    def cb_status(msg):
        with lock:
            data["status"] = (time.monotonic(), msg.data)

    def cb_sendi(msg):
        with lock:
            for nama, pos in zip(msg.name, msg.position):
                if nama == "head_pan":
                    data["pan"] = pos
                elif nama == "head_tilt":
                    data["tilt"] = pos

    node.create_subscription(CircleSetStamped, "/ball_detector_node/circle_set", cb_bola, 10)
    node.create_subscription(String, "/ball_tracker/status", cb_status, 10)
    node.create_subscription(JointState, "/robotis/present_joint_states", cb_sendi, 10)
    pub = node.create_publisher(String, "/ball_tracker/command", 10)
    sektor = list(p["striker_sector_pct_per_step"])
    gate = float(p["striker_sector_gate_pct"])
    hh = math.radians(p["striker_half_hfov_deg"])
    hv = math.radians(p["striker_half_vfov_deg"])
    span = PAN_LEFT_LIMIT_DEG - PAN_RIGHT_LIMIT_DEG

    try:
        if not cek_tidak_ada_striker(node):
            return 1
        akhir = time.monotonic() + 8.0
        while pub.get_subscription_count() == 0 and time.monotonic() < akhir:
            time.sleep(0.2)
        if pub.get_subscription_count() == 0:
            print("[GAGAL] head_tracking_node tidak jalan. Jalankan launch with_striker_node:=false dulu.")
            return 1
        enter("GERBANG -- KEPALA akan bergerak (menyapu / melacak bola). Badan tetap diam.\n"
              "   Taruh SATU bola di depan robot, lalu geser ke kiri / kanan / dekat / jauh selama %.0f detik.\n"
              "   Gerbang striker: bola DITOLAK kalau |servo 19 - sektor| > %.0f %% (sebelum approach)."
              % (args.detik, gate))
        pub.publish(String(data="start"))
        t0 = time.monotonic()
        while time.monotonic() - t0 < args.detik:
            time.sleep(0.5)
            now = time.monotonic()
            with lock:
                bola, status, pan, tilt = data["bola"], data["status"], data["pan"], data["tilt"]
            st = status[1] if status and now - status[0] < 0.5 else "?"
            baris = "t=%4.1f %-9s |" % (now - t0, st)
            if bola and now - bola[0] < 1.0 and bola[1].circles and pan is not None and tilt is not None:
                c = bola[1].circles[0]
                pan_eq = math.degrees(pan - math.atan(c.x * math.tan(hh)))
                pct = (PAN_LEFT_LIMIT_DEG - pan_eq) / span * 100.0
                s20 = SERVO_CENTER_DEG + math.degrees(tilt + math.atan(c.y * math.tan(hv)))
                jarak = "DEKAT" if s20 > p["_tilt_limit_deg"] else "jauh"
                putusan = " ".join("%s:%s" % (NAMA_BOLA[i], "ok" if abs(pct - sektor[i]) <= gate else "TOLAK")
                                   for i in range(4))
                baris += " servo19 %5.1f %% servo20 %5.1f (%s) | %s%s" % (
                    pct, s20, jarak, putusan, "" if st == "TRACKING" else "   (dinilai striker hanya saat TRACKING)")
            else:
                baris += " bola -"
            print(baris)
        return 0
    except KeyboardInterrupt:
        print("\n(dihentikan)")
        return 130
    finally:
        pub.publish(String(data="stop"))
        time.sleep(0.3)
        print("pelacak DIHENTIKAN (kepala diparkir).")
        tutup(node)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="uji", required=True)
    g = sub.add_parser("gyro", help="yaw gyro: tanda, skala 90 derajat, drift")
    g.add_argument("--bias-detik", type=float, default=5.0)
    g.add_argument("--drift-detik", type=float, default=60.0)
    d = sub.add_parser("deteksi", help="detektor kiper (robot) dan tiang gawang (goalpost)")
    d.add_argument("--detik", type=float, default=20.0)
    b = sub.add_parser("gerbang", help="bola: servo 19/20 dan putusan gerbang B/A/C/D (kepala bergerak)")
    b.add_argument("--detik", type=float, default=60.0)
    args = ap.parse_args()

    p = muat_setelan()
    if args.uji == "gyro":
        return jalankan_gyro(args, p)
    if args.uji == "deteksi":
        return jalankan_deteksi(args, p)
    return jalankan_gerbang(args, p)


if __name__ == "__main__":
    sys.exit(main())
