#!/usr/bin/env python3
"""Ukur geometri penentuan TENDANG langsung dari robot yang sedang hidup.

CARA PAKAI
----------
Robot berdiri di lantai, demo/manager jalan, bola DIAM di depan robot dan
terlihat kamera. Lalu:

    scripts/op3_docker.sh shell
    python3 /ros2_ws/scripts/calib_kick_geometry.py

Skrip ini HANYA menggerakkan sendi LEHER. Kaki, lengan, dan badan tidak
disentuh sama sekali.

APA YANG DIUKURNYA
------------------
1. KESEHATAN SENDI KEPALA. Diperintahkan pan, lalu tilt, lalu diperiksa apakah
   GAMBARNYA benar-benar berubah. Kalau tilt tidak mengubah gambar sama sekali,
   servo head_tilt (ID 20) tidak menurut dan tidak ada pengukuran jarak yang
   bisa dipercaya sampai itu dibetulkan.

2. SUDUT TUNDUK KAMERA. Kepala disapu pan -55..+55 derajat sementara bola diam.
   Karena bolanya diam, arah bola terhadap badan HARUS tetap; sudut tunduk yang
   membuatnya paling tetap itulah tunduk kamera yang sebenarnya.

3. ARAH BOLA terhadap badan, yaitu angka yang dipakai memilih kaki kanan/kiri.
   Dibandingkan antara rumus LAMA (head_pan + sudut bola di gambar) dan rumus
   BARU (geometri 3 dimensi, yang dipakai ball_follower sejak 2026-09-04).

CATATAN PENTING
---------------
* /robotis/present_joint_states BISA BEKU di robot ini (bulk read gagal, semua
  20 sendi jadi satu nilai). Karena itu sudut kepala diambil dari
  /robotis/goal_joint_states, sama seperti yang dipakai ball_follower.
* FOV kamera TIDAK ditebak: Logitech C920 pada 16:9 punya bukaan 70,4 x 43,3
  derajat (setengahnya 35,2 x 21,65). Itu yang dipakai sebagai busur derajat.
"""

import argparse
import json
import math
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import String

from op3_ball_detector_msgs.msg import CircleSetStamped

try:
    import cv2
except ImportError:  # gambar cuma dipakai untuk uji "berubah / tidak"
    cv2 = None

# Setengah bukaan Logitech C920 pada 640x360 (16:9). SPESIFIKASI, bukan tebakan.
HALF_FOV_H = math.radians(35.2)
HALF_FOV_V = math.radians(21.65)

REL = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                 history=HistoryPolicy.KEEP_LAST)
BEST = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT,
                  history=HistoryPolicy.KEEP_LAST)


def ball_direction(pan_rad, depression_rad, x_angle, y_angle):
    """Salinan persis BallFollower::ballDirection() dalam Python.

    Kerangka kamera: X ke depan, Y ke KIRI, Z ke ATAS.
    Mengembalikan (azimut, tunduk) bola terhadap BADAN, radian.
    """
    dx, dy, dz = 1.0, math.tan(x_angle), math.tan(y_angle)
    cp, sp = math.cos(depression_rad), math.sin(depression_rad)
    px = dx * cp + dz * sp
    py = dy
    pz = -dx * sp + dz * cp
    cy, sy = math.cos(pan_rad), math.sin(pan_rad)
    bx = px * cy - py * sy
    by = px * sy + py * cy
    return math.atan2(by, bx), -math.atan2(pz, math.hypot(bx, by))


class KickGeometry(Node):
    def __init__(self):
        super().__init__("calib_kick_geometry")
        self.goal = {}
        self.ball = None
        self.ball_t = 0.0
        self.img = None
        self.n_img = 0

        self.create_subscription(JointState, "/robotis/goal_joint_states",
                                 self._on_joint, REL)
        self.create_subscription(CircleSetStamped,
                                 "/ball_detector_node/circle_set", self._on_ball, REL)
        self.create_subscription(CompressedImage,
                                 "/usb_cam_node/image_raw/compressed", self._on_img, BEST)
        self.head_pub = self.create_publisher(
            JointState, "/robotis/head_control/set_joint_states", REL)
        self.tracker_pub = self.create_publisher(String, "/ball_tracker/command", REL)
        self.module_pub = self.create_publisher(String, "/robotis/enable_ctrl_module", REL)

    def _on_joint(self, msg):
        d = dict(zip(msg.name, msg.position))
        if "head_pan" in d:
            self.goal = {"pan": d["head_pan"], "tilt": d["head_tilt"]}

    def _on_ball(self, msg):
        if msg.circles:
            c = msg.circles[0]
            self.ball = (c.x, c.y, c.z)
            self.ball_t = time.time()

    def _on_img(self, msg):
        self.n_img += 1
        if cv2 is not None:
            self.img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)

    # -- dasar -------------------------------------------------------------
    def spin(self, seconds):
        end = time.time() + seconds
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def say(self, pub, text):
        msg = String()
        msg.data = text
        pub.publish(msg)
        self.spin(0.4)

    def goto(self, pan_deg, tilt_deg, settle=3.5):
        js = JointState()
        js.name = ["head_pan", "head_tilt"]
        js.position = [math.radians(pan_deg), math.radians(tilt_deg)]
        self.head_pub.publish(js)
        self.spin(settle)
        return None if self.img is None else self.img.copy()

    def sample(self, n=12):
        """Median beberapa bacaan bola + sudut leher saat itu."""
        obs = []
        for _ in range(n):
            self.spin(0.12)
            if self.ball is not None and (time.time() - self.ball_t) < 0.8 and self.goal:
                obs.append((self.ball[0], self.ball[1], self.ball[2],
                            self.goal["pan"], self.goal["tilt"]))
        if not obs:
            return None
        med = lambda i: sorted(o[i] for o in obs)[len(obs) // 2]
        return {"px": med(0), "py": med(1), "radius_px": med(2),
                "goal_pan": med(3), "goal_tilt": med(4), "n": len(obs)}


def image_diff(a, b):
    if a is None or b is None:
        return None
    return float(np.abs(a.astype(np.int16) - b.astype(np.int16)).mean())


def check_head_joints(node, tilt_deg):
    """Uji 1: apakah pan dan tilt benar-benar menggerakkan gambar?"""
    print("\n[1] KESEHATAN SENDI KEPALA")
    if cv2 is None:
        print("    dilewati: cv2 tidak ada di lingkungan ini")
        return None

    base = node.goto(0.0, tilt_deg)
    base2 = node.goto(0.0, tilt_deg)          # perintah SAMA -> derau kamera
    noise = image_diff(base, base2)

    tilt_a = node.goto(0.0, tilt_deg + 30.0)
    tilt_b = node.goto(0.0, tilt_deg - 30.0)
    d_tilt = max(image_diff(base2, tilt_a) or 0.0, image_diff(tilt_a, tilt_b) or 0.0)

    pan_a = node.goto(25.0, tilt_deg)
    pan_b = node.goto(-25.0, tilt_deg)
    d_pan = max(image_diff(tilt_b, pan_a) or 0.0, image_diff(pan_a, pan_b) or 0.0)

    node.goto(0.0, tilt_deg)
    print("    derau kamera (perintah sama 2x) : %.2f" % noise)
    print("    perubahan gambar oleh TILT +-30 : %.2f" % d_tilt)
    print("    perubahan gambar oleh PAN  +-25 : %.2f" % d_pan)

    healthy = {"pan": d_pan > noise * 3.0, "tilt": d_tilt > noise * 3.0}
    print("    head_pan  : %s" % ("BERGERAK" if healthy["pan"] else "TIDAK BERGERAK"))
    print("    head_tilt : %s" % ("BERGERAK" if healthy["tilt"] else "TIDAK BERGERAK"))
    if not healthy["tilt"]:
        print("    >> head_tilt (servo ID 20) tidak menurut. Jarak bola dihitung dari")
        print("       sudut ini, jadi penentuan tendang TIDAK BISA dipercaya sampai")
        print("       servo/kabelnya dibetulkan.")
    return healthy


def sweep_pan(node, tilt_deg, angles):
    """Uji 2+3: sapu pan, catat letak bola di gambar."""
    print("\n[2] SAPUAN PAN (bola harus DIAM selama ini)")
    rows = []
    for deg in angles:
        node.goto(deg, tilt_deg)
        s = node.sample()
        if s is None:
            print("    pan %+6.1f deg : bola tidak terlihat" % deg)
            continue
        s["cmd_deg"] = deg
        rows.append(s)
        print("    pan %+6.1f deg : px %+7.4f  py %+7.4f  radius %5.1f px"
              % (deg, s["px"], s["py"], s["radius_px"]))
    node.goto(0.0, tilt_deg)
    return rows


def fit_depression(rows):
    """Sudut tunduk yang membuat arah bola paling TETAP sepanjang sapuan."""
    if len(rows) < 4:
        return None

    def spread(depression_deg):
        vals = []
        for r in rows:
            xa = -math.atan(r["px"] * math.tan(HALF_FOV_H))
            ya = -math.atan(r["py"] * math.tan(HALF_FOV_V))
            az, _ = ball_direction(r["goal_pan"], math.radians(depression_deg), xa, ya)
            vals.append(math.degrees(az))
        mean = sum(vals) / len(vals)
        return math.sqrt(sum((v - mean) ** 2 for v in vals) / len(vals)), mean, vals

    best = None
    for d in [x * 0.25 for x in range(0, 340)]:
        sd, mean, _ = spread(d)
        if best is None or sd < best[0]:
            best = (sd, d, mean)
    sd, depression_deg, azimuth_deg = best

    old = [math.degrees(r["goal_pan"] - math.atan(r["px"] * math.tan(HALF_FOV_H)))
           for r in rows]
    old_mean = sum(old) / len(old)
    old_sd = math.sqrt(sum((v - old_mean) ** 2 for v in old) / len(old))
    _, _, new = spread(depression_deg)

    print("\n[3] HASIL")
    print("    sudut tunduk kamera        : %.1f deg" % depression_deg)
    print("    arah bola thd badan        : %+.2f deg  (+ = bola di KIRI)" % azimuth_deg)
    print("    bola TIDAK bergerak, jadi jawaban yang benar harus TETAP:")
    print("      rumus LAMA (pan + gambar): %+.1f .. %+.1f deg   sebar %.2f deg"
          % (min(old), max(old), old_sd))
    print("      rumus BARU (geometri 3D) : %+.1f .. %+.1f deg   sebar %.2f deg"
          % (min(new), max(new), sd))
    if old_sd > 0:
        print("    perbaikan                  : %.1f kali lebih tetap" % (old_sd / max(sd, 1e-6)))
    return {"depression_deg": depression_deg, "azimuth_deg": azimuth_deg,
            "spread_new_deg": sd, "spread_old_deg": old_sd}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tilt-deg", type=float, default=None,
                    help="sudut tilt yang dipakai selama pengukuran "
                         "(bawaan: sudut tilt yang sedang berlaku)")
    ap.add_argument("--out", default="/tmp/calib_kick_geometry.json")
    ap.add_argument("--skip-health", action="store_true")
    args = ap.parse_args()

    rclpy.init()
    node = KickGeometry()
    node.spin(2.5)

    if not node.goal:
        print("Tidak ada /robotis/goal_joint_states -- op3_manager jalan?")
        rclpy.shutdown()
        return
    if node.ball is None:
        print("Bola tidak terdeteksi -- pastikan bola terlihat kamera sebelum mulai.")
        rclpy.shutdown()
        return

    tilt_deg = args.tilt_deg
    if tilt_deg is None:
        tilt_deg = math.degrees(node.goal["tilt"])
    print("mulai: goal pan %+.2f deg, tilt dipakai %+.2f deg, bola px %+.4f py %+.4f"
          % (math.degrees(node.goal["pan"]), tilt_deg, node.ball[0], node.ball[1]))

    # Ambil alih kepala dari head_tracking_node.
    node.say(node.tracker_pub, "stop")
    node.say(node.module_pub, "head_control_module")
    node.spin(1.0)

    result = {"tilt_deg": tilt_deg}
    try:
        if not args.skip_health:
            result["head"] = check_head_joints(node, tilt_deg)
        rows = sweep_pan(node, tilt_deg,
                         (0, 15, 30, 45, 55, 0, -15, -30, -45, -55, 0))
        result["rows"] = rows
        fit = fit_depression(rows)
        if fit:
            result.update(fit)
    finally:
        node.goto(0.0, tilt_deg, settle=2.5)
        node.say(node.tracker_pub, "start")
        os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
        with open(args.out, "w") as f:
            json.dump(result, f, indent=2)
        print("\ntersimpan: %s" % args.out)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
