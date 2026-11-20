#!/usr/bin/env python3
"""Kalibrasi jarak kamera CHRONUS: model prediksi jarak bola dari posisi servo 20.

Cara kerja (operator 2026-11-08):
  1. Operator menaruh SATU bola lurus di depan robot pada jarak yang diukur dari
     garis ujung jari kaki robot ke tanda di lantai tepat di bawah pusat bola.
  2. `sweep --jarak <m>` menggerakkan kepala dari atas ke bawah (servo 20), di
     tiap posisi menunggu kepala diam lalu merekam posisi bola di gambar.
  3. Sesudah beberapa jarak, `fit` membuat model jarak = f(servo 20 setara):

         s20_setara = servo20 + atan((cy - 180) / fy) * 180/pi

     = posisi servo 20 SEANDAINYA kepala tepat mengarah ke bola. Dengan ini
     jarak bisa dihitung untuk bola mana pun di gambar, juga saat kepala sedang
     mengikuti benda lain (mode kiper D58 mengikuti striker).

Stack yang dibutuhkan untuk `sweep` (TANPA demo -- demo_node dan
head_tracking_node ikut menggerakkan kepala):
    ros2 launch op3_manager op3_manager.launch.py
    ros2 launch op3_ball_detector ball_detector.launch.py
dengan ROS_DOMAIN_ID=1. Badan tetap di page 2 (op3_manager memainkannya saat
boot); skrip ini hanya mengambil alih sendi head_pan dan head_tilt.

Subperintah: sweep, fit, status, hapus, uji-sintetis, latensi, uji-latensi.
Lihat --help masing-masing.

`latensi` (2026-11-12) mengukur jeda gambar -> deteksi yang SEBENARNYA dengan
mengayun kepala di depan satu bola diam; hasilnya diisi ke param demo
intercept_gk_cam_latency_sec (demo.launch.xml).
"""
import argparse
import bisect
import json
import math
import os
import statistics
import sys
import time
from datetime import datetime

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA_DIR = os.path.join(REPO, "src", "ROBOTIS-OP3-Demo", "op3_demo", "data")
SAMPLE_FILE = os.path.join(DATA_DIR, "kalibrasi_jarak_kamera_sampel.jsonl")
RESULT_FILE = os.path.join(DATA_DIR, "kalibrasi_jarak_kamera.yaml")

# Gambar yang diterima detektor (op3_ball_detector/config/camera_param.yaml).
IMG_W = 640.0
IMG_H = 360.0
CY0 = IMG_H / 2.0
# Fokus awal dari FOV yang dipakai BallTracker (setengah FOV horizontal 35,2 deg).
FY_AWAL = (IMG_W / 2.0) / math.tan(math.radians(35.2))

# Batas head_control_module (head_control_module.cpp): -75 .. +58,03 deg sendi,
# positif = menunduk di robot ini. servo 20 (deg) = 180 + sudut sendi.
TILT_MIN_DEG = -75.0
TILT_MAX_DEG = 58.03

# Terukur 2026-11-08: head_tilt berhenti ~0,6 deg sebelum target (gravitasi + gain P),
# jadi "sampai" = dalam 1 deg ATAU sudah tidak bergerak (< TOL_DIAM_DEG per 0,25 dtk).
TOL_SAMPAI_DEG = 1.0
TOL_DIAM_DEG = 0.1
TEPI_PX = 3.0             # kotak bola yang menyentuh tepi gambar = terpotong, dibuang di fit
BATAS_SAMPAI_DTK = 3.0
DIAM_DTK = 0.3            # kepala diam dulu sebelum merekam
REKAM_DTK = 0.6
TILT_PULANG_DEG = 15.0    # kepala dikembalikan ke (0, +15) sesudah sapuan


def joint_rad_ke_servo_deg(rad):
    """Sama dengan InterceptDemo::jointRadToServoDeg (XM430: raw 2048 = 0 rad)."""
    return 180.0 + rad * (2047.0 * 360.0) / (4096.0 * math.pi)


def servo_deg_ke_joint_rad(servo_deg):
    return (servo_deg - 180.0) * (4096.0 * math.pi) / (2047.0 * 360.0)


# == berkas sampel ===========================================================

def baca_sampel(path):
    rows = []
    if not os.path.exists(path):
        return rows
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def ikuti_pemilik_folder(path):
    """Skrip berjalan sebagai root di container; kembalikan pemilik ke pemilik folder."""
    try:
        st = os.stat(os.path.dirname(path))
        os.chown(path, st.st_uid, st.st_gid)
    except OSError:
        pass


def tulis_sampel(path, rows):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        for r in rows:
            f.write(json.dumps(r, ensure_ascii=False) + "\n")
    os.replace(tmp, path)
    ikuti_pemilik_folder(path)


def sama_jarak(a, b):
    return abs(float(a) - float(b)) < 1e-6


def di_tepi(row):
    """Kotak bola menyentuh tepi gambar = bola terpotong, cy/w-nya bergeser."""
    if row.get("cx") is None or row.get("h") is None:
        return False
    return (row["cx"] - row["w"] / 2.0 <= TEPI_PX or row["cx"] + row["w"] / 2.0 >= IMG_W - TEPI_PX or
            row["cy"] - row["h"] / 2.0 <= TEPI_PX or row["cy"] + row["h"] / 2.0 >= IMG_H - TEPI_PX)


# == sweep ===================================================================

def cmd_sweep(args):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState, Imu
    from std_msgs.msg import String
    from vision_msgs.msg import Detection2DArray
    from robotis_controller_msgs.msg import JointCtrlModule

    if not (TILT_MIN_DEG <= args.dari <= TILT_MAX_DEG and TILT_MIN_DEG <= args.sampai <= TILT_MAX_DEG):
        sys.exit("--dari/--sampai harus di dalam %.0f..%.2f deg" % (TILT_MIN_DEG, TILT_MAX_DEG))
    if args.langkah <= 0:
        sys.exit("--langkah harus > 0")

    rclpy.init()
    node = Node("kalibrasi_jarak_kamera")
    st = {"pan": None, "tilt": None, "pitch": None, "roll": None, "det": []}

    def on_joint(msg):
        for i, name in enumerate(msg.name):
            if i >= len(msg.position):
                break
            if name == "head_pan":
                st["pan"] = msg.position[i]
            elif name == "head_tilt":
                st["tilt"] = msg.position[i]

    def on_imu(msg):
        q = msg.orientation
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        st["pitch"] = math.degrees(math.asin(max(-1.0, min(1.0, sinp))))
        st["roll"] = math.degrees(math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                                             1.0 - 2.0 * (q.x * q.x + q.y * q.y)))

    def on_det(msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        balls = []
        for d in msg.detections:
            if d.id != "ball":
                continue
            score = d.results[0].hypothesis.score if d.results else 1.0
            balls.append((d.bbox.center.position.x, d.bbox.center.position.y,
                          d.bbox.size_x, d.bbox.size_y, score))
        st["det"].append((stamp, balls))

    node.create_subscription(JointState, "/robotis/present_joint_states", on_joint, 10)
    node.create_subscription(Imu, "/robotis/open_cr/imu", on_imu, 10)
    node.create_subscription(Detection2DArray, "/ball_detector_node/detections", on_det, 10)
    head_pub = node.create_publisher(JointState, "/robotis/head_control/set_joint_states", 10)
    enable_pub = node.create_publisher(String, "/robotis/enable_ctrl_module", 10)
    ctrl_pub = node.create_publisher(JointCtrlModule, "/robotis/set_joint_ctrl_modules", 10)

    def spin_for(sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)

    def now_sec():
        return node.get_clock().now().nanoseconds * 1e-9

    def kirim_kepala(pan_deg, tilt_deg):
        m = JointState()
        m.header.stamp = node.get_clock().now().to_msg()
        m.name = ["head_pan", "head_tilt"]
        m.position = [math.radians(pan_deg), math.radians(tilt_deg)]
        head_pub.publish(m)

    def selesai(kode=0, pesan=None):
        node.destroy_node()
        rclpy.shutdown()
        if pesan:
            print(pesan)
        sys.exit(kode)

    # -- prasyarat -----------------------------------------------------------
    print("Menunggu op3_manager (/robotis/present_joint_states) dan detektor ...", flush=True)
    t_end = time.monotonic() + 6.0
    while time.monotonic() < t_end and (st["tilt"] is None or not st["det"]):
        rclpy.spin_once(node, timeout_sec=0.1)
    if st["tilt"] is None:
        selesai(2, "GAGAL: /robotis/present_joint_states tidak terdengar -- op3_manager belum jalan "
                   "(atau ROS_DOMAIN_ID bukan 1).")
    if not st["det"]:
        selesai(2, "GAGAL: /ball_detector_node/detections tidak terdengar -- jalankan "
                   "ros2 launch op3_ball_detector ball_detector.launch.py")
    nama = set(node.get_node_names())
    pengganggu = sorted(nama & {"demo_node", "head_tracking_node"})
    if pengganggu and not args.paksa:
        selesai(2, "GAGAL: node %s sedang jalan dan ikut menggerakkan kepala. Matikan demo dulu "
                   "(atau --paksa)." % ", ".join(pengganggu))

    # -- ambil alih kepala saja ---------------------------------------------
    enable_pub.publish(String(data="head_control_module"))
    cm = JointCtrlModule()
    cm.joint_name = ["head_pan", "head_tilt"]
    cm.module_name = ["head_control_module", "head_control_module"]
    ctrl_pub.publish(cm)
    spin_for(0.5)

    tilts = []
    t = args.dari
    arah = 1.0 if args.sampai >= args.dari else -1.0
    while (t - args.sampai) * arah <= 1e-9:
        tilts.append(round(t, 3))
        t += arah * args.langkah

    print("Sapuan jarak %.2f m: %d posisi, tilt %+.1f -> %+.1f deg (servo 20 %.1f -> %.1f)"
          % (args.jarak, len(tilts), tilts[0], tilts[-1], 180 + tilts[0], 180 + tilts[-1]), flush=True)
    print("%6s %7s %7s %5s %6s %6s %6s %5s %s" % ("tilt", "servo20", "aktual", "frame", "cy", "w",
                                                   "conf", "bola", "catatan"), flush=True)

    rows = []
    t_mulai = time.monotonic()
    n_frame_total = 0
    try:
        for tilt in tilts:
            kirim_kepala(0.0, tilt)
            target = math.radians(tilt)
            t0 = time.monotonic()
            sampai = False
            ref_tilt, ref_t = None, time.monotonic()
            # Posisi pertama menempuh perjalanan jauh (mis. +58 -> -10 deg, terukur > 3 dtk)
            # dan trajektori kepala mulai pelan: jangan dianggap "berhenti" terlalu cepat.
            pertama = (tilt == tilts[0])
            batas = BATAS_SAMPAI_DTK * (3.0 if pertama else 1.0)
            min_gerak = 1.5 if pertama else 0.4
            while time.monotonic() - t0 < batas:
                rclpy.spin_once(node, timeout_sec=0.02)
                if st["tilt"] is None:
                    continue
                if abs(st["tilt"] - target) <= math.radians(TOL_SAMPAI_DEG):
                    sampai = True
                    break
                # Sudah berhenti bergerak (tertahan sebelum target) -> lanjut, tilt aktual yang direkam.
                if time.monotonic() - ref_t >= 0.25:
                    if ref_tilt is not None and time.monotonic() - t0 >= min_gerak and \
                            abs(st["tilt"] - ref_tilt) < math.radians(TOL_DIAM_DEG):
                        break
                    ref_tilt, ref_t = st["tilt"], time.monotonic()
            spin_for(DIAM_DTK)
            t_diam = now_sec()
            st["det"] = []
            tilt_buf, pan_buf = [], []
            t1 = time.monotonic()
            while time.monotonic() - t1 < REKAM_DTK:
                rclpy.spin_once(node, timeout_sec=0.02)
                if st["tilt"] is not None:
                    tilt_buf.append(st["tilt"])
                if st["pan"] is not None:
                    pan_buf.append(st["pan"])
            frames = [b for (s, b) in st["det"] if s >= t_diam]
            n_frame_total += len(frames)
            pilih = []
            multi = False
            for balls in frames:
                if not balls:
                    continue
                if len(balls) > 1:
                    multi = True
                pilih.append(min(balls, key=lambda b: abs(b[0] - IMG_W / 2.0)))
            terlihat = len(pilih) >= max(2, 0.5 * len(frames)) if frames else False
            tilt_akt = statistics.mean(tilt_buf) if tilt_buf else st["tilt"]
            row = {
                "jarak": float(args.jarak),
                "waktu": datetime.now().isoformat(timespec="seconds"),
                "tilt_target_deg": tilt,
                "tilt_deg": math.degrees(tilt_akt),
                "servo20": joint_rad_ke_servo_deg(tilt_akt),
                "pan_deg": math.degrees(statistics.mean(pan_buf)) if pan_buf else None,
                "imu_pitch_deg": st["pitch"],
                "imu_roll_deg": st["roll"],
                "sampai": sampai,
                "frame": len(frames),
                "frame_bola": len(pilih),
                "bola_lebih_dari_satu": multi,
                "terlihat": terlihat,
            }
            if pilih:
                for k, idx in (("cx", 0), ("cy", 1), ("w", 2), ("h", 3), ("conf", 4)):
                    row[k] = statistics.median(b[idx] for b in pilih)
            rows.append(row)
            catatan = []
            selisih = row["tilt_deg"] - tilt
            if abs(selisih) > TOL_SAMPAI_DEG:
                catatan.append("kepala tertahan %+.1f deg" % selisih)
            if di_tepi(row):
                catatan.append("tepi (terpotong, tak dipakai fit)")
            if multi:
                catatan.append(">1 bola")
            print("%+6.1f %7.1f %7.1f %5d %6s %6s %6s %5s %s" % (
                tilt, 180 + tilt, row["servo20"], len(frames),
                "%.0f" % row["cy"] if "cy" in row else "-",
                "%.0f" % row["w"] if "w" in row else "-",
                "%.2f" % row["conf"] if "conf" in row else "-",
                "YA" if terlihat else "-", ", ".join(catatan)), flush=True)
    except KeyboardInterrupt:
        print("\nDibatalkan -- data sapuan ini TIDAK disimpan.")
        kirim_kepala(0.0, TILT_PULANG_DEG)
        spin_for(0.3)
        selesai(1)

    kirim_kepala(0.0, TILT_PULANG_DEG)
    spin_for(0.3)
    durasi = time.monotonic() - t_mulai
    fps = n_frame_total / max(1e-6, len(tilts) * REKAM_DTK)

    vis = [r for r in rows if r["terlihat"]]
    print()
    if vis:
        print("Bola terlihat di servo 20 %.1f .. %.1f (%d dari %d posisi)."
              % (min(r["servo20"] for r in vis), max(r["servo20"] for r in vis), len(vis), len(rows)))
    else:
        print("PERINGATAN: bola TIDAK terlihat di posisi mana pun.")
    print("Durasi %.0f dtk, FPS detektor selama rekam ~%.1f%s" % (
        durasi, fps, "  <-- RENDAH (< 20): cek baterai/CPU sebelum lanjut" if fps < 20 else ""))
    if any(r["bola_lebih_dari_satu"] for r in rows):
        print("PERINGATAN: ada frame dengan >1 bola -- singkirkan bola lain dari pandangan.")

    if args.tidak_simpan:
        print("(--tidak-simpan: data tidak ditulis)")
    else:
        lama = [r for r in baca_sampel(args.file) if not sama_jarak(r["jarak"], args.jarak)]
        tulis_sampel(args.file, lama + rows)
        print("Tersimpan %d posisi untuk jarak %.2f m -> %s" % (len(rows), args.jarak, args.file))
    selesai(0)


# == fit =====================================================================

def lm_fit(f, x0, xs, ys, iters=300):
    """Levenberg-Marquardt kecil (numpy saja). f(x, params) -> prediksi."""
    import numpy as np
    p = np.array(x0, dtype=float)
    xs = np.asarray(xs, dtype=float)
    ys = np.asarray(ys, dtype=float)

    def res(pp):
        return f(xs, pp) - ys

    lam = 1e-2
    r = res(p)
    cost = float(r @ r)
    for _ in range(iters):
        J = np.zeros((len(xs), len(p)))
        for j in range(len(p)):
            dp = np.zeros_like(p)
            dp[j] = 1e-6 * max(1.0, abs(p[j]))
            J[:, j] = (res(p + dp) - r) / dp[j]
        A = J.T @ J
        g = J.T @ r
        step = np.linalg.solve(A + lam * np.diag(np.diag(A) + 1e-12), -g)
        p_new = p + step
        r_new = res(p_new)
        c_new = float(r_new @ r_new)
        if np.isfinite(c_new) and c_new < cost:
            p, r, cost = p_new, r_new, c_new
            lam = max(lam / 3.0, 1e-9)
            if np.linalg.norm(step) < 1e-10 * (1.0 + np.linalg.norm(p)):
                break
        else:
            lam *= 4.0
            if lam > 1e9:
                break
    return p, cost


def model_geometri(s, pr):
    import numpy as np
    H, theta0, c = pr
    ang = np.radians(np.asarray(s, dtype=float) - 180.0 + theta0)
    out = H / np.tan(ang) - c
    return np.where(ang > math.radians(0.5), out, 1e3)


def model_poli(s, coef):
    import numpy as np
    return np.polyval(coef, np.asarray(s, dtype=float))


def fit_geometri(s, d):
    best = None
    for H0 in (0.3, 0.45, 0.6):
        for th0 in (-10.0, 0.0, 10.0):
            p, cost = lm_fit(model_geometri, [H0, th0, 0.05], s, d)
            if best is None or cost < best[1]:
                best = (p, cost)
    return best[0]


def fit_poli(s, d):
    import numpy as np
    return np.polyfit(np.asarray(s, dtype=float), np.asarray(d, dtype=float), 2)


def s20_setara(row, fy):
    return row["servo20"] + math.degrees(math.atan((row["cy"] - CY0) / fy))


def cari_fy(groups):
    """fy yang membuat s20_setara paling konstan di dalam tiap jarak."""
    import numpy as np

    def biaya(fy):
        tot = 0.0
        for rows in groups.values():
            v = np.array([s20_setara(r, fy) for r in rows])
            tot += float(((v - v.mean()) ** 2).sum())
        return tot

    grid = np.arange(200.0, 1200.0, 2.0)
    costs = [biaya(f) for f in grid]
    lo = grid[max(0, int(np.argmin(costs)) - 2)]
    hi = grid[min(len(grid) - 1, int(np.argmin(costs)) + 2)]
    g = (math.sqrt(5) - 1) / 2
    a, b = lo, hi
    for _ in range(60):
        c1, c2 = b - g * (b - a), a + g * (b - a)
        if biaya(c1) < biaya(c2):
            b = c2
        else:
            a = c1
    return (a + b) / 2.0


def silang_tengah(rows):
    """Servo 20 saat cy melintasi tengah gambar (interpolasi linear), atau None."""
    rs = sorted(rows, key=lambda r: r["servo20"])
    for a, b in zip(rs, rs[1:]):
        ea, eb = a["cy"] - CY0, b["cy"] - CY0
        if ea == 0:
            return a["servo20"]
        if ea * eb < 0:
            return a["servo20"] + (b["servo20"] - a["servo20"]) * ea / (ea - eb)
    return None


def invers(f, d, lo=150.0, hi=TILT_MAX_DEG + 180.0):
    """Servo 20 (tengah) yang memberi jarak d; model turun terhadap servo 20."""
    flo, fhi = f(lo) - d, f(hi) - d
    if flo * fhi > 0:
        return None
    for _ in range(80):
        mid = (lo + hi) / 2.0
        fm = f(mid) - d
        if flo * fm <= 0:
            hi, fhi = mid, fm
        else:
            lo, flo = mid, fm
    return (lo + hi) / 2.0


def hitung_fit(rows, cetak=print):
    """Mengembalikan dict hasil; juga mencetak laporan."""
    import numpy as np

    vis_semua = [r for r in rows if r.get("terlihat") and r.get("cy") is not None]
    vis = [r for r in vis_semua if not di_tepi(r)]
    if len(vis) < len(vis_semua):
        cetak("(%d posisi dibuang: kotak bola menyentuh tepi gambar)" % (len(vis_semua) - len(vis)))
    groups = {}
    for r in vis:
        groups.setdefault(round(float(r["jarak"]), 4), []).append(r)
    # Bola lain di pandangan (terukur 2026-11-08: bola w 68 px ikut tercatat di sapuan
    # 0,2 m yang bolanya w ~150 px). Bola yang SAMA lebarnya hampir tetap sepanjang
    # sapuan, jadi sampel yang lebarnya > 30 % dari median jaraknya dibuang.
    for k in list(groups):
        med_w = statistics.median(r["w"] for r in groups[k])
        sah = [r for r in groups[k] if abs(r["w"] - med_w) <= 0.3 * med_w]
        if len(sah) < len(groups[k]):
            cetak("(jarak %.2f m: %d posisi dibuang -- lebar bola menyimpang > 30%% dari median %.0f px, "
                  "kemungkinan bola lain)" % (k, len(groups[k]) - len(sah), med_w))
        groups[k] = sah
    groups = {k: v for k, v in groups.items() if len(v) >= 3}
    if len(groups) < 3:
        sys.exit("Butuh minimal 3 jarak dengan >= 3 posisi bola terlihat (ada %d). "
                 "Rekam jarak lagi dengan `sweep`." % len(groups))

    fy = cari_fy(groups)
    # Benda lain yang lolos filter lebar (2026-11-08: "bola" w 36 px di servo 225 pada
    # sapuan 2,5 m, menarik fy 491 -> 525). Bola yang sama memberi s20_setara yang sama
    # sepanjang sapuan, jadi sampel > 2 deg dari median jaraknya dibuang, lalu fy diulang.
    for _ in range(2):
        dibuang = 0
        for k in list(groups):
            med = statistics.median(s20_setara(r, fy) for r in groups[k])
            sah = [r for r in groups[k] if abs(s20_setara(r, fy) - med) <= 2.0]
            dibuang += len(groups[k]) - len(sah)
            if len(sah) < len(groups[k]):
                cetak("(jarak %.2f m: %d posisi dibuang -- servo 20 setara > 2 deg dari median, benda lain)"
                      % (k, len(groups[k]) - len(sah)))
            groups[k] = sah
        groups = {k: v for k, v in groups.items() if len(v) >= 3}
        if dibuang == 0:
            break
        fy = cari_fy(groups)
    jarak = sorted(groups)
    tabel = []
    for d in jarak:
        g = groups[d]
        eq = [s20_setara(r, fy) for r in g]
        tabel.append({
            "jarak_m": d,
            "servo20_tengah": float(np.median(eq)),
            "sebaran_deg": float(np.std(eq)),
            "servo20_silang": silang_tengah(g),
            "w_px": float(np.median([r["w"] for r in g])),
            "n": len(g),
            "servo20_terlihat": [min(r["servo20"] for r in g), max(r["servo20"] for r in g)],
        })
    s = np.array([t["servo20_tengah"] for t in tabel])
    d = np.array([t["jarak_m"] for t in tabel])
    w = np.array([t["w_px"] for t in tabel])

    pg = fit_geometri(s, d)
    pp = fit_poli(s, d)
    # model lebar: d = K/w - c  (linear di 1/w)
    A = np.vstack([1.0 / w, -np.ones_like(w)]).T
    K, cw = np.linalg.lstsq(A, d, rcond=None)[0]

    def loo(fitter, model):
        if len(d) < 4:
            return None
        err = []
        for i in range(len(d)):
            m = np.ones(len(d), bool)
            m[i] = False
            pr = fitter(s[m], d[m])
            err.append(float(model(np.array([s[i]]), pr)[0] - d[i]))
        return float(np.sqrt(np.mean(np.square(err))))

    pred_g = model_geometri(s, pg)
    pred_p = model_poli(s, pp)
    pred_w = K / w - cw
    rmse = lambda e: float(np.sqrt(np.mean(np.square(e))))
    loo_g = loo(fit_geometri, model_geometri)
    loo_p = loo(fit_poli, model_poli)
    if loo_g is None or loo_p is None:
        pilih = "geometri"
    else:
        # Polinom berbahaya di luar rentang data (2026-11-08: LOO seri 7,3 cm, polinom
        # menang tipis lalu rekomendasi observingnya ngawur). Geometri dipertahankan
        # kecuali polinom JELAS lebih baik.
        pilih = "polinomial" if loo_p < 0.8 * loo_g else "geometri"

    cetak("== KALIBRASI JARAK KAMERA (%d jarak, %d posisi terlihat) ==" % (len(d), sum(len(g) for g in groups.values())))
    cetak("fy (fokus vertikal) = %.1f px  (dari FOV BallTracker: %.1f px, selisih %+.1f%%)"
          % (fy, FY_AWAL, 100.0 * (fy - FY_AWAL) / FY_AWAL))
    cetak("s20_setara = servo20 + atan((cy - %.0f) / %.1f) * 180/pi" % (CY0, fy))
    cetak("")
    cetak("%7s %10s %8s %9s %6s %4s   %8s %8s %8s" % ("jarak", "s20_tengah", "sebaran", "s20_silang",
                                                     "w_px", "n", "geometri", "polinom", "lebar"))
    for i, t in enumerate(tabel):
        cetak("%6.2fm %10.2f %7.2f° %9s %6.1f %4d   %+7.1fcm %+7.1fcm %+7.1fcm" % (
            t["jarak_m"], t["servo20_tengah"], t["sebaran_deg"],
            "%.2f" % t["servo20_silang"] if t["servo20_silang"] is not None else "-",
            t["w_px"], t["n"],
            100 * (pred_g[i] - d[i]), 100 * (pred_p[i] - d[i]), 100 * (pred_w[i] - d[i])))
    cetak("")
    cetak("GEOMETRI  jarak = %.4f / tan((s20 - 180 %+.3f) deg) - %.4f   RMSE %.1f cm, LOO %s"
          % (pg[0], pg[1], pg[2], 100 * rmse(pred_g - d),
             "%.1f cm" % (100 * loo_g) if loo_g is not None else "n/a (<4 jarak)"))
    cetak("POLINOM   jarak = %.6g*s20^2 %+.6g*s20 %+.6g   RMSE %.1f cm, LOO %s"
          % (pp[0], pp[1], pp[2], 100 * rmse(pred_p - d),
             "%.1f cm" % (100 * loo_p) if loo_p is not None else "n/a (<4 jarak)"))
    cetak("LEBAR    jarak = %.4f / w_px - %.4f   RMSE %.1f cm (cadangan)" % (K, cw, 100 * rmse(pred_w - d)))
    cetak("MODEL TERPILIH: %s%s" % (pilih, "" if len(d) >= 4 else "  (rekam >= 4 jarak supaya pilihan diuji)"))

    # -- peta keterlihatan & rekomendasi observing --------------------------
    f_pilih = (lambda x: float(model_geometri(np.array([x]), pg)[0])) if pilih == "geometri" \
        else (lambda x: float(model_poli(np.array([x]), pp)[0]))
    half_v = math.degrees(math.atan(CY0 / fy))
    margin = 3.0
    obs = None
    s_jauh, s_dekat = invers(f_pilih, 3.0), invers(f_pilih, 0.5)
    cetak("")
    cetak("Setengah FOV vertikal %.1f deg (margin tepi %.0f deg)." % (half_v, margin))
    if s_jauh is not None and s_dekat is not None:
        span = s_dekat - s_jauh
        if span <= 2 * (half_v - margin):
            obs = (s_dekat + s_jauh) / 2.0
            cetak("Observing: servo 20 %.1f -> bola 0,5 m (s20 %.1f) s.d. 3,0 m (s20 %.1f) masuk gambar sekaligus."
                  % (obs, s_dekat, s_jauh))
        else:
            obs = s_jauh + (half_v - margin)
            dekat = f_pilih(obs + (half_v - margin))
            cetak("Observing: 0,5–3,0 m tak muat satu gambar (rentang %.1f deg). Servo 20 %.1f memuat 3,0 m "
                  "s.d. ~%.2f m." % (span, obs, dekat))
    else:
        cetak("Observing: model tak bisa dibalik untuk 0,5/3,0 m (di luar rentang) -- rekam jarak lebih lebar.")
    if min(d) > 0.5 or max(d) < 3.0:
        cetak("CATATAN: data hanya %.2f–%.2f m; angka di luar rentang itu ekstrapolasi." % (min(d), max(d)))

    return {
        "fy": fy, "tabel": tabel, "pilih": pilih,
        "geometri": {"H_m": float(pg[0]), "theta0_deg": float(pg[1]), "c_m": float(pg[2]),
                     "rmse_cm": 100 * rmse(pred_g - d), "loo_rmse_cm": None if loo_g is None else 100 * loo_g},
        "polinomial": {"koef": [float(x) for x in pp], "rmse_cm": 100 * rmse(pred_p - d),
                       "loo_rmse_cm": None if loo_p is None else 100 * loo_p},
        "lebar": {"K_px_m": float(K), "c_m": float(cw), "rmse_cm": 100 * rmse(pred_w - d)},
        "observing_servo20": obs, "half_fov_v_deg": half_v,
        "rentang_data_m": [float(min(d)), float(max(d))],
    }


def tulis_yaml(path, h):
    def num(x, fmt="%.6g"):
        return "null" if x is None else fmt % x
    L = []
    L.append("# Kalibrasi jarak kamera CHRONUS -- dibuat scripts/calib_jarak_kamera.py fit")
    L.append("# JANGAN diedit tangan; ulangi sapuan + fit.")
    L.append("tanggal: \"%s\"" % datetime.now().isoformat(timespec="seconds"))
    L.append("gambar: {lebar: %d, tinggi: %d, cy0: %.1f}" % (IMG_W, IMG_H, CY0))
    L.append("fy_px: %s" % num(h["fy"], "%.3f"))
    L.append("# s20_setara = servo20 + atan((cy - cy0) / fy_px) * 180/pi")
    L.append("model_terpilih: %s" % h["pilih"])
    g = h["geometri"]
    L.append("geometri:   # jarak = H_m / tan((s20_setara - 180 + theta0_deg) deg) - c_m")
    L.append("  H_m: %s" % num(g["H_m"]))
    L.append("  theta0_deg: %s" % num(g["theta0_deg"]))
    L.append("  c_m: %s" % num(g["c_m"]))
    L.append("  rmse_cm: %s" % num(g["rmse_cm"], "%.2f"))
    L.append("  loo_rmse_cm: %s" % num(g["loo_rmse_cm"], "%.2f"))
    p = h["polinomial"]
    L.append("polinomial:   # jarak = koef[0]*s^2 + koef[1]*s + koef[2], s = s20_setara")
    L.append("  koef: [%s]" % ", ".join("%.9g" % c for c in p["koef"]))
    L.append("  rmse_cm: %s" % num(p["rmse_cm"], "%.2f"))
    L.append("  loo_rmse_cm: %s" % num(p["loo_rmse_cm"], "%.2f"))
    w = h["lebar"]
    L.append("lebar_bbox:   # jarak = K_px_m / w_px - c_m (cadangan)")
    L.append("  K_px_m: %s" % num(w["K_px_m"]))
    L.append("  c_m: %s" % num(w["c_m"]))
    L.append("  rmse_cm: %s" % num(w["rmse_cm"], "%.2f"))
    L.append("rentang_data_m: [%.3f, %.3f]" % tuple(h["rentang_data_m"]))
    L.append("observing:")
    L.append("  servo20_rekomendasi: %s" % num(h["observing_servo20"], "%.2f"))
    L.append("  setengah_fov_vertikal_deg: %.2f" % h["half_fov_v_deg"])
    L.append("tabel:")
    for t in h["tabel"]:
        L.append("  - {jarak_m: %.3f, servo20_tengah: %.3f, sebaran_deg: %.3f, servo20_silang: %s, "
                 "w_px: %.2f, n: %d, servo20_terlihat: [%.2f, %.2f]}" % (
                     t["jarak_m"], t["servo20_tengah"], t["sebaran_deg"],
                     num(t["servo20_silang"], "%.3f"), t["w_px"], t["n"],
                     t["servo20_terlihat"][0], t["servo20_terlihat"][1]))
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        f.write("\n".join(L) + "\n")
    os.replace(tmp, path)
    ikuti_pemilik_folder(path)


def cmd_fit(args):
    rows = baca_sampel(args.file)
    if not rows:
        sys.exit("Belum ada sampel di %s -- jalankan `sweep --jarak <m>` dulu." % args.file)
    h = hitung_fit(rows)
    if args.tidak_simpan:
        print("(--tidak-simpan: hasil tidak ditulis)")
    else:
        tulis_yaml(args.hasil, h)
        print("Hasil ditulis ke %s" % args.hasil)


def cmd_status(args):
    rows = baca_sampel(args.file)
    if not rows:
        print("Belum ada sampel (%s)." % args.file)
        return
    per = {}
    for r in rows:
        per.setdefault(float(r["jarak"]), []).append(r)
    print("%7s %6s %9s %18s  %s" % ("jarak", "posisi", "terlihat", "servo20 terlihat", "direkam"))
    for d in sorted(per):
        g = per[d]
        v = [r for r in g if r.get("terlihat")]
        rng = "%.1f..%.1f" % (min(r["servo20"] for r in v), max(r["servo20"] for r in v)) if v else "-"
        print("%6.2fm %6d %9d %18s  %s" % (d, len(g), len(v), rng, g[0].get("waktu", "?")))


def cmd_hapus(args):
    rows = baca_sampel(args.file)
    sisa = [r for r in rows if not sama_jarak(r["jarak"], args.jarak)]
    if len(sisa) == len(rows):
        print("Tidak ada data untuk jarak %.2f m." % args.jarak)
        return
    tulis_sampel(args.file, sisa)
    print("Dihapus %d posisi untuk jarak %.2f m." % (len(rows) - len(sisa), args.jarak))


# == uji sintetis ============================================================

def cmd_uji_sintetis(args):
    """Data buatan dari geometri yang diketahui (kamera 4 cm di depan sumbu tilt,
    yang TIDAK ada di model) + derau, lalu fit dan bandingkan jarak hasil prediksi."""
    import random
    random.seed(args.seed)
    H_pivot, r_bola, L_cam, theta0, c_true, fy_true = 0.50, 0.067, 0.04, 3.0, 0.06, 470.0
    jarak = [0.3, 0.5, 0.75, 1.0, 1.5, 2.0, 2.5, 3.0]
    rows = []
    for d in jarak:
        for tilt in [x * 2.0 for x in range(-5, 30)]:
            p = math.radians(tilt + theta0)
            xc, zc = L_cam * math.cos(p), H_pivot - L_cam * math.sin(p)
            horiz = d + c_true - xc
            delta = math.atan2(zc - r_bola, horiz)
            phi = delta - p
            cy = CY0 + fy_true * math.tan(phi) + random.gauss(0, 2.0)
            if not (8 <= cy <= IMG_H - 8):
                continue
            jarak_miring = math.hypot(horiz, zc - r_bola)
            w = fy_true * 2 * r_bola / jarak_miring + random.gauss(0, 1.0)
            s20 = 180.0 + tilt + random.gauss(0, 0.2)
            rows.append({"jarak": d, "servo20": s20, "cy": cy, "w": w, "terlihat": True})
    h = hitung_fit(rows)
    import numpy as np
    pg = h["geometri"]

    def prediksi(s):
        if h["pilih"] == "geometri":
            return float(model_geometri(np.array([s]), [pg["H_m"], pg["theta0_deg"], pg["c_m"]])[0])
        return float(model_poli(np.array([s]), h["polinomial"]["koef"])[0])

    print("")
    # fy hanya angka bantu: ia ikut menyerap paralaks kamera yang berada di depan
    # sumbu tilt (tidak ada di model), jadi tidak dijadikan syarat lulus.
    print("UJI SINTETIS: fy sebenarnya %.1f -> hasil %.1f (%+.1f%%, informasi)"
          % (fy_true, h["fy"], 100 * (h["fy"] - fy_true) / fy_true))
    # (1) per jarak: servo 20 saat bola di tengah gambar.
    worst = max(abs(prediksi(t["servo20_tengah"]) - t["jarak_m"]) / t["jarak_m"] for t in h["tabel"])
    # (2) per SAMPEL: bola di mana pun di gambar -- inilah yang dipakai saat demo.
    err = sorted(abs(prediksi(s20_setara(r, h["fy"])) - r["jarak"]) / r["jarak"] for r in rows)
    p90 = err[int(0.9 * (len(err) - 1))]
    lolos = worst <= 0.03 and p90 <= 0.06
    print("Galat jarak: per jarak terburuk %.1f%% (syarat <= 3%%), per sampel p90 %.1f%% / maks %.1f%% "
          "(syarat p90 <= 6%%) -> %s" % (100 * worst, 100 * p90, 100 * err[-1], "LULUS" if lolos else "GAGAL"))
    if args.keluar:
        tulis_sampel(args.keluar, rows)
        print("Data sintetis ditulis ke %s" % args.keluar)
    sys.exit(0 if lolos else 1)

# == latensi =================================================================
# Stempel gambar usb_cam 0.8.1 TIDAK bisa dipakai: get_epoch_time_shift_us()
# (/opt/ros/humble/include/usb_cam/utils.hpp) membagi tv_usec dengan 1000, jadi
# stempel bergeser konstan acak 0..1 dtk tiap usb_cam start -- itulah "latensi
# ~900 ms" yang dulu terukur. Latensi SEBENARNYA diukur dari gerak kepala: arah
# bola DIAM = sudut kepala saat gambar ditangkap + sudut bola di gambar, harus
# konstan. tau yang membuatnya paling konstan = waktu terima deteksi - waktu
# tangkap, dengan jam yang sama dengan demo (waktu terima present_joint_states).
LATENSI_FILE = os.path.join(DATA_DIR, "kalibrasi_latensi_kamera_rekaman.json")
TAU_MAKS = 0.6
TAU_LANGKAH = 0.002


def baca_kalibrasi(path=RESULT_FILE):
    cal = {"fy": FY_AWAL, "cy0": CY0, "H": None, "theta0": None, "c": None}
    try:
        import yaml
        with open(path) as f:
            y = yaml.safe_load(f)
        cal["fy"] = float(y.get("fy_px", cal["fy"]))
        cal["cy0"] = float((y.get("gambar") or {}).get("cy0", CY0))
        g = y.get("geometri") or {}
        if g:
            cal["H"], cal["theta0"], cal["c"] = float(g["H_m"]), float(g["theta0_deg"]), float(g["c_m"])
    except (OSError, ImportError, KeyError, TypeError, ValueError, AttributeError) as e:
        print("PERINGATAN: kalibrasi jarak tak terbaca (%s) -- fy awal %.1f, tanpa jarak." % (e, FY_AWAL))
    return cal


def interp(ts, vs, t):
    """Interpolasi linear vs(t) pada ts terurut; None di luar rentang."""
    i = bisect.bisect_left(ts, t)
    if i <= 0 or i >= len(ts):
        return None
    t0, t1 = ts[i - 1], ts[i]
    u = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
    return vs[i - 1] + u * (vs[i] - vs[i - 1])


def jarak_geometri(cal, s20):
    if cal["H"] is None:
        return None
    ang = math.radians(s20 - 180.0 + cal["theta0"])
    if ang < math.radians(1.0):
        return None
    return cal["H"] / math.tan(ang) - cal["c"]


def analisis_latensi(rec, cal, cetak=print):
    """tau per fase (pan/tilt) + geseran stempel usb_cam. Kembalikan dict hasil."""
    joint = sorted(rec["joint"])
    ts = [j[0] for j in joint]
    pans = [j[1] for j in joint]
    tilts = [j[2] for j in joint]
    hasil = {"tau": {}}

    for fase in ("pan", "tilt"):
        det = [d for d in rec["det"] if d["fase"] == fase and d["bola"] is not None]
        if len(det) < 30:
            cetak("Fase %s: deteksi satu-bola terlalu sedikit (%d) -- dilewati." % (fase, len(det)))
            continue

        def nilai(tau):
            vals = []
            for d in det:
                if fase == "pan":
                    p = interp(ts, pans, d["t"] - tau)
                    if p is not None:
                        vals.append(math.degrees(math.atan((d["bola"][0] - IMG_W / 2.0) / cal["fy"]) - p))
                else:
                    k = interp(ts, tilts, d["t"] - tau)
                    if k is not None:
                        vals.append(joint_rad_ke_servo_deg(k) +
                                    math.degrees(math.atan((d["bola"][1] - cal["cy0"]) / cal["fy"])))
            return vals

        skor = []
        n = int(round(TAU_MAKS / TAU_LANGKAH))
        for i in range(n + 1):
            tau = i * TAU_LANGKAH
            v = nilai(tau)
            if len(v) >= 20:
                skor.append((statistics.pstdev(v), tau))
        if not skor:
            cetak("Fase %s: data kepala tak menutupi deteksi -- dilewati." % fase)
            continue
        sd_best, tau_best = min(skor)
        sd0 = next(sd for sd, t in skor if t == 0.0) if skor[0][1] == 0.0 else float("nan")
        gerak = [math.degrees(v) for v, t in zip(pans if fase == "pan" else tilts, ts)
                 if det[0]["t"] <= t <= det[-1]["t"]]
        ayun = (max(gerak) - min(gerak)) if gerak else 0.0
        hasil["tau"][fase] = tau_best
        cetak("Fase %-4s: %4d deteksi, kepala berayun %.1f deg -> tau %.3f dtk "
              "(sebaran arah bola %.2f deg; tanpa koreksi %.2f deg)" % (fase, len(det), ayun, tau_best, sd_best, sd0))
        if ayun < 5.0:
            cetak("  PERINGATAN: kepala hampir tak bergerak -- hasil fase ini tak bisa dipercaya.")
        if tau_best <= TAU_LANGKAH or tau_best >= TAU_MAKS - TAU_LANGKAH:
            cetak("  PERINGATAN: tau di tepi rentang 0..%.1f dtk -- periksa rekaman." % TAU_MAKS)
        if fase == "tilt" and cal["H"] is not None:
            for label, tau in (("tanpa koreksi", 0.0), ("dengan tau", tau_best)):
                dd = [jarak_geometri(cal, v) for v in nilai(tau)]
                dd = [x for x in dd if x is not None]
                if len(dd) >= 20:
                    cetak("  jarak bola diam (%s): median %.2f m, sebaran +-%.3f m, rentang %.2f m"
                          % (label, statistics.median(dd), statistics.pstdev(dd), max(dd) - min(dd)))

    if hasil["tau"]:
        taus = list(hasil["tau"].values())
        hasil["rekomendasi"] = round(statistics.mean(taus), 3)
        if len(taus) == 2 and abs(taus[0] - taus[1]) > 0.03:
            cetak("PERINGATAN: tau pan %.3f vs tilt %.3f berbeda > 30 ms -- ulangi pengukuran."
                  % (hasil["tau"]["pan"], hasil["tau"]["tilt"]))
        geser = [d["t_wall"] - d["stamp"] for d in rec["det"] if d.get("stamp")]
        if geser:
            hasil["umur_stempel"] = statistics.median(geser)
            cetak("Umur stempel gambar saat diterima: median %.3f dtk = latensi %.3f + geseran stempel usb_cam "
                  "%.3f dtk (bug usb_cam, berubah tiap usb_cam start)."
                  % (hasil["umur_stempel"], hasil["rekomendasi"], hasil["umur_stempel"] - hasil["rekomendasi"]))
        cetak("")
        cetak("ISI di demo.launch.xml:  <param name=\"intercept_gk_cam_latency_sec\" value=\"%.3f\" />"
              % hasil["rekomendasi"])
    return hasil


def cmd_latensi(args):
    cal = baca_kalibrasi()
    if args.dari_rekaman:
        with open(args.dari_rekaman) as f:
            rec = json.load(f)
        analisis_latensi(rec, cal)
        return

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from vision_msgs.msg import Detection2DArray
    from robotis_controller_msgs.msg import JointCtrlModule

    rclpy.init()
    node = Node("kalibrasi_latensi_kamera")
    st = {"pan": None, "tilt": None, "fase": "siap"}
    rec = {"meta": {"waktu": datetime.now().isoformat(timespec="seconds"), "amplitudo_pan_deg": args.pan,
                    "amplitudo_tilt_deg": args.tilt, "periode_dtk": args.periode, "siklus": args.siklus},
           "joint": [], "det": []}
    multi = [0]

    def on_joint(msg):
        pan = tilt = None
        for i, name in enumerate(msg.name):
            if i >= len(msg.position):
                break
            if name == "head_pan":
                pan = msg.position[i]
            elif name == "head_tilt":
                tilt = msg.position[i]
        if pan is None or tilt is None:
            return
        st["pan"], st["tilt"] = pan, tilt
        if st["fase"] in ("pan", "tilt"):
            rec["joint"].append([time.monotonic(), pan, tilt])

    def on_det(msg):
        t = time.monotonic()
        balls = [(d.bbox.center.position.x, d.bbox.center.position.y, d.bbox.size_x)
                 for d in msg.detections if d.id == "ball"]
        st["bola_terakhir"] = balls
        if st["fase"] not in ("pan", "tilt"):
            return
        if len(balls) > 1:
            multi[0] += 1
        rec["det"].append({"t": t, "t_wall": time.time(),
                           "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                           "fase": st["fase"], "n": len(balls),
                           "bola": list(balls[0]) if len(balls) == 1 else None})

    node.create_subscription(JointState, "/robotis/present_joint_states", on_joint, 50)
    node.create_subscription(Detection2DArray, "/ball_detector_node/detections", on_det, 10)
    head_pub = node.create_publisher(JointState, "/robotis/head_control/set_joint_states", 10)
    enable_pub = node.create_publisher(String, "/robotis/enable_ctrl_module", 10)
    ctrl_pub = node.create_publisher(JointCtrlModule, "/robotis/set_joint_ctrl_modules", 10)

    def spin_for(sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.005)

    def kirim_kepala(pan_deg, tilt_deg):
        m = JointState()
        m.header.stamp = node.get_clock().now().to_msg()
        m.name = ["head_pan", "head_tilt"]
        m.position = [math.radians(pan_deg), math.radians(tilt_deg)]
        head_pub.publish(m)

    def selesai(kode=0, pesan=None):
        node.destroy_node()
        rclpy.shutdown()
        if pesan:
            print(pesan)
        sys.exit(kode)

    print("Menunggu op3_manager (/robotis/present_joint_states) dan detektor ...", flush=True)
    t_end = time.monotonic() + 6.0
    while time.monotonic() < t_end and (st["tilt"] is None or "bola_terakhir" not in st):
        rclpy.spin_once(node, timeout_sec=0.1)
    if st["tilt"] is None:
        selesai(2, "GAGAL: /robotis/present_joint_states tidak terdengar -- op3_manager belum jalan "
                   "(atau ROS_DOMAIN_ID / ROS_LOCALHOST_ONLY beda dengan stack).")
    if "bola_terakhir" not in st:
        selesai(2, "GAGAL: /ball_detector_node/detections tidak terdengar -- detektor belum jalan.")
    nama = set(node.get_node_names())
    if "head_tracking_node" in nama and not args.paksa:
        selesai(2, "GAGAL: head_tracking_node jalan dan bisa ikut menggerakkan kepala. Pastikan D58 MATI, "
                   "lalu ulangi dengan --paksa.")

    enable_pub.publish(String(data="head_control_module"))
    cm = JointCtrlModule()
    cm.joint_name = ["head_pan", "head_tilt"]
    cm.module_name = ["head_control_module", "head_control_module"]
    ctrl_pub.publish(cm)
    spin_for(0.5)

    # Arahkan kepala ke bola (bola di tengah gambar), dari posisi observing.
    pan_c, tilt_c = 0.0, args.tilt_awal
    kirim_kepala(pan_c, tilt_c)
    spin_for(2.0)
    for _ in range(2):
        b = st.get("bola_terakhir") or []
        if len(b) != 1:
            kirim_kepala(0.0, TILT_PULANG_DEG)
            spin_for(0.5)
            selesai(2, "GAGAL: perlu TEPAT satu bola di gambar (terlihat %d). Taruh satu bola diam ~1 m "
                       "di depan robot, singkirkan bola lain." % len(b))
        cx, cy = b[0][0], b[0][1]
        pan_c -= math.degrees(math.atan((cx - IMG_W / 2.0) / cal["fy"]))
        tilt_c += math.degrees(math.atan((cy - cal["cy0"]) / cal["fy"]))
        tilt_c = max(TILT_MIN_DEG + args.tilt, min(TILT_MAX_DEG - args.tilt, tilt_c))
        kirim_kepala(pan_c, tilt_c)
        spin_for(1.5)
    print("Bola di tengah: pan %+.1f deg, tilt %+.1f deg (servo 20 %.1f)." % (pan_c, tilt_c, 180.0 + tilt_c))

    try:
        for fase, amp in (("pan", args.pan), ("tilt", args.tilt)):
            if amp <= 0.0:
                continue
            print("Fase %s: +-%.0f deg, %d bolak-balik @ %.2f dtk ..." % (fase, amp, args.siklus, args.periode),
                  flush=True)
            st["fase"] = fase
            for i in range(2 * args.siklus):
                arah = 1.0 if i % 2 == 0 else -1.0
                if fase == "pan":
                    kirim_kepala(pan_c + arah * amp, tilt_c)
                else:
                    kirim_kepala(pan_c, tilt_c + arah * amp)
                spin_for(args.periode)
            st["fase"] = "jeda"
            kirim_kepala(pan_c, tilt_c)
            spin_for(1.5)
    except KeyboardInterrupt:
        print("\nDibatalkan.")
        kirim_kepala(0.0, TILT_PULANG_DEG)
        spin_for(0.3)
        selesai(1)

    kirim_kepala(0.0, TILT_PULANG_DEG)
    spin_for(0.3)
    if multi[0] > 0:
        print("PERINGATAN: %d frame berisi > 1 bola (frame itu tak dipakai)." % multi[0])
    if not args.tidak_simpan:
        with open(LATENSI_FILE, "w") as f:
            json.dump(rec, f)
        ikuti_pemilik_folder(LATENSI_FILE)
        print("Rekaman -> %s (analisis ulang: latensi --dari-rekaman <berkas>)" % LATENSI_FILE)
    print()
    analisis_latensi(rec, cal)
    selesai(0)


def cmd_uji_latensi(args):
    """Rekaman buatan dengan tau diketahui -> analisis harus menemukannya kembali."""
    import random
    rnd = random.Random(args.seed)
    cal = baca_kalibrasi()
    if cal["H"] is None:
        cal.update({"H": 0.4797, "theta0": 12.52, "c": 0.0706})
    arah_bola = math.radians(4.0)    # bola sedikit di kanan
    s20_bola = 191.2                 # bola ~1 m
    geser_stempel = rnd.uniform(0.0, 1.0)
    rec = {"joint": [], "det": []}
    rentang = {}
    t = 0.0
    for fase, amp, base_tilt in (("pan", 12.0, 0.0), ("tilt", 6.0, s20_bola - 180.0)):
        t0 = t
        while t < t0 + 2 * 8 * 1.25:
            k = int((t - t0) / 1.25)
            u = min(1.0, ((t - t0) % 1.25) / 1.0)
            mj = 10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5           # min-jerk 1 dtk
            a1 = amp if k % 2 == 0 else -amp
            a0 = 0.0 if k == 0 else -a1
            sudut = math.radians(a0 + (a1 - a0) * mj)
            pan = sudut if fase == "pan" else 0.0
            tilt = math.radians(base_tilt) + (sudut if fase == "tilt" else math.radians(s20_bola - 180.0))
            rec["joint"].append([t, pan, tilt])
            t += 0.008
        rentang[fase] = (t0, t)
        t += 1.0
    ts = [j[0] for j in rec["joint"]]
    pans = [j[1] for j in rec["joint"]]
    tilts = [j[2] for j in rec["joint"]]
    for fase, (t0, t1) in rentang.items():
        tt = t0 + 0.05
        while tt < t1 - 0.05:
            pan = interp(ts, pans, tt)
            tilt = interp(ts, tilts, tt)
            cx = IMG_W / 2.0 + cal["fy"] * math.tan(arah_bola + pan) + rnd.gauss(0, 1.5)
            cy = cal["cy0"] + cal["fy"] * math.tan(math.radians(s20_bola - joint_rad_ke_servo_deg(tilt))) \
                + rnd.gauss(0, 1.5)
            t_terima = tt + args.tau + rnd.uniform(0.0, 0.012)
            rec["det"].append({"t": t_terima, "t_wall": 1.0e9 + t_terima, "stamp": 1.0e9 + tt - geser_stempel,
                               "fase": fase, "n": 1, "bola": [cx, cy, 63.0]})
            tt += 1.0 / 28.0
    print("Uji sintetis: tau sebenarnya %.3f dtk (+ jitter 0-12 ms), geseran stempel %.3f dtk"
          % (args.tau, geser_stempel))
    h = analisis_latensi(rec, cal)
    ok = len(h["tau"]) == 2 and all(abs(v - args.tau - 0.006) <= 0.01 for v in h["tau"].values())
    print("LULUS" if ok else "GAGAL")
    sys.exit(0 if ok else 1)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--file", default=SAMPLE_FILE, help="berkas sampel (jsonl)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("sweep", help="sapuan kepala atas->bawah pada satu jarak")
    s.add_argument("--jarak", type=float, required=True,
                   help="m, dari garis ujung jari kaki robot ke tanda di bawah pusat bola")
    s.add_argument("--dari", type=float, default=-10.0, help="tilt awal (deg sendi, + = menunduk)")
    s.add_argument("--sampai", type=float, default=58.0, help="tilt akhir (deg sendi)")
    s.add_argument("--langkah", type=float, default=2.0, help="deg per langkah")
    s.add_argument("--paksa", action="store_true", help="jalan walau demo_node/head_tracking_node hidup")
    s.add_argument("--tidak-simpan", action="store_true", help="uji saja, data tidak ditulis")
    s.set_defaults(func=cmd_sweep)

    f = sub.add_parser("fit", help="cocokkan model jarak dari semua sapuan")
    f.add_argument("--hasil", default=RESULT_FILE, help="berkas YAML hasil")
    f.add_argument("--tidak-simpan", action="store_true")
    f.set_defaults(func=cmd_fit)

    st = sub.add_parser("status", help="jarak yang sudah direkam")
    st.set_defaults(func=cmd_status)

    h = sub.add_parser("hapus", help="buang data satu jarak")
    h.add_argument("--jarak", type=float, required=True)
    h.set_defaults(func=cmd_hapus)

    u = sub.add_parser("uji-sintetis", help="uji fit dengan data buatan (tanpa robot)")
    u.add_argument("--seed", type=int, default=1)
    u.add_argument("--keluar", default=None, help="tulis data sintetis ke berkas ini (opsional)")
    u.set_defaults(func=cmd_uji_sintetis)

    la = sub.add_parser("latensi", help="ukur latensi kamera->deteksi (satu bola diam ~1 m, D58 MATI)")
    la.add_argument("--pan", type=float, default=12.0, help="amplitudo ayunan pan (deg), 0 = lewati")
    la.add_argument("--tilt", type=float, default=6.0, help="amplitudo ayunan tilt (deg), 0 = lewati")
    la.add_argument("--periode", type=float, default=1.25,
                    help="dtk per gerakan (head_control_module: 1 dtk per perintah)")
    la.add_argument("--siklus", type=int, default=8, help="bolak-balik per fase")
    la.add_argument("--tilt-awal", type=float, default=11.0, help="tilt awal (deg sendi, ~bola 1 m)")
    la.add_argument("--paksa", action="store_true", help="jalan walau head_tracking_node hidup (D58 MATI)")
    la.add_argument("--tidak-simpan", action="store_true", help="rekaman tidak ditulis")
    la.add_argument("--dari-rekaman", default=None, help="analisis ulang rekaman json tanpa robot")
    la.set_defaults(func=cmd_latensi)

    ul = sub.add_parser("uji-latensi", help="uji analisis latensi dengan rekaman buatan (tanpa robot)")
    ul.add_argument("--tau", type=float, default=0.12)
    ul.add_argument("--seed", type=int, default=1)
    ul.set_defaults(func=cmd_uji_latensi)

    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
