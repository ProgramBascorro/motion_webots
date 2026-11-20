#!/usr/bin/env python3
"""RL geser kanan dengan penilaian operator, langsung di robot asli (CHRONUS).

Robot mencoba satu setelan "geser kanan", operator menilainya di terminal,
lalu setelan berikutnya digeser ke arah yang dinilai bagus. Algoritmenya PGPE
(policy gradient dengan eksplorasi di ruang parameter): tiap putaran robot
mencoba PASANGAN setelan simetris -- mu + sigma*eps dan mu - sigma*eps -- dan
selisih skor keduanya menentukan ke mana mu bergerak. Pasangan simetris dipakai
karena skor dari manusia kasar dan berisik; yang dibandingkan cuma "mana yang
lebih bagus dari dua ini", bukan angka mutlak.

Penilaian per percobaan:
  kestabilan  : normal / hampir jatuh / jatuh
  kecepatan   : terlalu cepat / cepat / normal / lambat   (yang dicari: cepat)
  kualitas    : bagus / jelek
  kesesuaian  : sesuai (benar-benar ke kanan) / tidak sesuai (berputar/maju)

Percobaannya meniru demo D58 (InterceptDemo::startPreset): init pose dari versi
"mutar kanan" dengan amplitudo nol, soft_start_time dipasang dulu, parameter
dikirim, "start", lalu "soft_stop" sesudah durasi percobaan.

Yang dipelajari hanya PARAM_SPACE di bawah. Field lain (offset Init Pose,
balance, x_move, angle_move, ...) disalin apa adanya dari versi "geser kanan".
Hasilnya disimpan sebagai versi BARU "geser kanan RL" di walking_versions.json;
versi "geser kanan" sendiri tidak pernah ditimpa.

Jalankan DI DALAM container, dengan op3_manager hidup dan demo TIDAK jalan:
    source /ros2_ws/install/setup.bash; export ROS_DOMAIN_ID=1
    python3 /ros2_ws/scripts/rl_geser_kanan.py
Uji logika tanpa robot (penilaian acak, tanpa ROS):
    python3 scripts/rl_geser_kanan.py --tanpa-robot
"""
import argparse
import copy
import csv
import datetime
import json
import math
import os
import random
import select
import shutil
import sys
import threading
import time
import uuid

import numpy as np

WS = os.environ.get("OP3_WS", "/ros2_ws")
VERSIONS_PATH = os.path.join(WS, "src/ROBOTIS-OP3/op3_action_module/data/walking_versions.json")
RUN_DIR = os.environ.get("OP3_RL_DIR", os.path.join(WS, "rl_runs/geser_kanan"))

BASE_PRESET = "geser kanan"
INIT_PRESET = "mutar kanan"        # sama dengan preset_init_ InterceptDemo
RESULT_PRESET = "geser kanan RL"

# (nama field di versi walking, batas bawah, batas atas). Batasnya sengaja
# sempit di sekitar setelan geser kanan yang sudah terbukti jalan -- RL di robot
# asli tidak boleh menjelajah sampai setelan yang membakar lutut (lihat latch
# OVERLOAD servo 13). Satuan sama dengan walking_versions.json (detik, meter).
PARAM_SPACE = [
    ("period_time",      0.55,   1.00),
    ("dsp_ratio",        0.05,   0.30),
    ("y_move_amplitude", -0.040, -0.010),   # negatif = ke kanan
    ("z_move_amplitude", 0.020,  0.045),    # tinggi angkat kaki
    ("y_swap_amplitude", 0.010,  0.030),    # goyang badan kiri-kanan
    ("soft_start_time",  0.20,   1.00),     # demo menolak geser tanpa soft start > 0
]

# Skor per penilaian. "jatuh" sengaja paling berat: satu jatuh harus mengalahkan
# semua kelebihan lain. Kecepatan yang dicari operator untuk kiper adalah "cepat".
RATINGS = [
    ("kestabilan", [("normal", 2.0), ("hampir jatuh", 0.0), ("jatuh", -4.0)]),
    ("kecepatan",  [("terlalu cepat", -0.5), ("cepat", 1.0), ("normal", 0.5), ("lambat", -0.5)]),
    ("kualitas",   [("bagus", 1.0), ("jelek", -1.0)]),
    ("kesesuaian", [("sesuai", 1.0), ("tidak sesuai", -1.0)]),
]
R_MAX = sum(max(v for _, v in opts) for _, opts in RATINGS)
R_MIN = sum(min(v for _, v in opts) for _, opts in RATINGS)

SIGMA_INIT = 0.15      # dalam ruang ternormalisasi [0, 1] per parameter
SIGMA_MIN = 0.03
SIGMA_MAX = 0.30
ALPHA_MU = 1.0
ALPHA_SIGMA = 0.5
PAIRS_PER_UPDATE = 2   # 2 pasangan = 4 percobaan per pembaruan

WALKING_ENABLE_SETTLE = 1.2   # = WALKING_ENABLE_SETTLE_MS Studio
ACTION_MODULE_SETTLE = 0.6    # = ACTION_MODULE_SETTLE_MS Studio
INIT_SETTLE = 1.0
STANCE_SETTLE = 2.5           # ramp iniPoseTraGene minimal 1,5 dtk + pembatas 25 deg/s
INIT_PAGE = 2                 # WALKING_READY


# ---------------------------------------------------------------- versi walking
def load_versions():
    with open(VERSIONS_PATH, encoding="utf-8") as fh:
        return json.load(fh)


def newest_version(versions, name):
    """Entri bernama `name` dengan updatedAt terbaru -- aturan yang sama dengan
    InterceptDemo::loadWalkingVersions(), karena nama boleh kembar."""
    best = None
    for v in versions:
        if v.get("name", "").lower() != name.lower() or "params" not in v:
            continue
        if best is None or v.get("updatedAt", 0) > best.get("updatedAt", 0):
            best = v
    return best


def save_result_version(params, note):
    """Tulis/ganti versi RESULT_PRESET. Berkas lama dicadangkan dulu."""
    versions = load_versions()
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    os.makedirs(RUN_DIR, exist_ok=True)
    backup = os.path.join(RUN_DIR, f"walking_versions.json.bak-{stamp}")
    shutil.copy2(VERSIONS_PATH, backup)

    now_ms = int(time.time() * 1000)
    entry = newest_version(versions, RESULT_PRESET)
    if entry is None:
        entry = {"id": str(uuid.uuid4()), "name": RESULT_PRESET, "createdAt": now_ms}
        versions.insert(0, entry)
    entry["updatedAt"] = now_ms
    entry["params"] = params
    entry["rl_note"] = note

    st = os.stat(VERSIONS_PATH)
    tmp = VERSIONS_PATH + ".tmp"
    with open(tmp, "w", encoding="utf-8") as fh:
        json.dump(versions, fh, indent=2)
    os.replace(tmp, VERSIONS_PATH)
    try:
        os.chown(VERSIONS_PATH, st.st_uid, st.st_gid)
        os.chmod(VERSIONS_PATH, st.st_mode & 0o777)
    except OSError:
        pass
    return backup


# ---------------------------------------------------------------- PGPE
def to_physical(x):
    return {name: lo + float(np.clip(v, 0.0, 1.0)) * (hi - lo)
            for (name, lo, hi), v in zip(PARAM_SPACE, x)}


def to_normalized(params):
    return np.array([np.clip((params[name] - lo) / (hi - lo), 0.0, 1.0)
                     for name, lo, hi in PARAM_SPACE])


def norm_reward(r):
    return (r - R_MIN) / (R_MAX - R_MIN)


class Learner:
    """Keadaan PGPE + log, disimpan ke state.json sesudah tiap penilaian."""

    def __init__(self, base_params, state_path):
        self.state_path = state_path
        self.base_params = base_params
        if os.path.exists(state_path):
            with open(state_path, encoding="utf-8") as fh:
                s = json.load(fh)
            self.mu = np.array(s["mu"])
            self.sigma = np.array(s["sigma"])
            self.baseline = s["baseline"]
            self.trial_no = s["trial_no"]
            self.update_no = s["update_no"]
            self.batch = s["batch"]          # pasangan yang sudah lengkap dinilai
            self.pending = s["pending"]      # pasangan yang sedang berjalan
            self.best = s["best"]
            self.resumed = True
        else:
            self.mu = to_normalized(base_params)
            self.sigma = np.full(len(PARAM_SPACE), SIGMA_INIT)
            self.baseline = None
            self.trial_no = 0
            self.update_no = 0
            self.batch = []
            self.pending = None
            self.best = None
            self.resumed = False

    def save(self):
        s = {
            "mu": self.mu.tolist(), "sigma": self.sigma.tolist(), "baseline": self.baseline,
            "trial_no": self.trial_no, "update_no": self.update_no,
            "batch": self.batch, "pending": self.pending, "best": self.best,
            "param_space": PARAM_SPACE,
        }
        tmp = self.state_path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as fh:
            json.dump(s, fh, indent=2)
        os.replace(tmp, self.state_path)

    def next_candidate(self):
        """(label, vektor ternormalisasi) untuk percobaan berikutnya."""
        if self.pending is None:
            eps = np.random.randn(len(PARAM_SPACE))
            self.pending = {"eps": eps.tolist(), "r_plus": None, "r_minus": None}
            self.save()
        eps = np.array(self.pending["eps"])
        if self.pending["r_plus"] is None:
            return "+", np.clip(self.mu + self.sigma * eps, 0.0, 1.0)
        return "-", np.clip(self.mu - self.sigma * eps, 0.0, 1.0)

    def record(self, label, x, reward):
        self.trial_no += 1
        params = to_physical(x)
        if self.best is None or reward >= self.best["reward"]:
            self.best = {"reward": reward, "params": params, "trial_no": self.trial_no}
        self.pending["r_plus" if label == "+" else "r_minus"] = reward
        updated = False
        if self.pending["r_plus"] is not None and self.pending["r_minus"] is not None:
            self.batch.append(self.pending)
            self.pending = None
            if len(self.batch) >= PAIRS_PER_UPDATE:
                self._update()
                updated = True
        self.save()
        return updated

    def _update(self):
        d_mu = np.zeros(len(PARAM_SPACE))
        d_sigma = np.zeros(len(PARAM_SPACE))
        mean_r = np.mean([(norm_reward(p["r_plus"]) + norm_reward(p["r_minus"])) / 2 for p in self.batch])
        b = mean_r if self.baseline is None else self.baseline
        for p in self.batch:
            eps = np.array(p["eps"])
            rp, rm = norm_reward(p["r_plus"]), norm_reward(p["r_minus"])
            d_mu += (rp - rm) / 2.0 * self.sigma * eps
            d_sigma += ((rp + rm) / 2.0 - b) * (eps ** 2 - 1.0) * self.sigma
        n = len(self.batch)
        self.mu = np.clip(self.mu + ALPHA_MU * d_mu / n, 0.0, 1.0)
        self.sigma = np.clip(self.sigma + ALPHA_SIGMA * d_sigma / n, SIGMA_MIN, SIGMA_MAX)
        self.baseline = mean_r if self.baseline is None else 0.7 * self.baseline + 0.3 * mean_r
        self.batch = []
        self.update_no += 1

    def full_params(self, physical):
        p = copy.deepcopy(self.base_params)
        p.update(physical)
        return p


# ---------------------------------------------------------------- robot
class Robot:
    """Perintah ke op3_manager, sama dengan yang dikirim Studio dan InterceptDemo."""

    def __init__(self):
        import rclpy
        import rclpy.executors
        from rclpy.node import Node
        from std_msgs.msg import Int32, String
        from op3_walking_module_msgs.msg import WalkingParam
        from rcl_interfaces.srv import GetParameters, SetParameters
        from robotis_controller_msgs.srv import GetJointModule

        self.rclpy = rclpy
        self.String, self.Int32, self.WalkingParam = String, Int32, WalkingParam
        self.SetParameters, self.GetJointModule = SetParameters, GetJointModule
        self.GetParameters = GetParameters
        # Walking module MENULIS soft_start_time ke param.yaml tiap kali diubah
        # (persistSoftStartTime), jadi nilai aslinya dicatat dan dikembalikan saat
        # keluar -- kalau tidak, percobaan terakhir RL jadi bawaan boot berikutnya.
        self.orig_soft_start = None
        self.soft_start_touched = False
        rclpy.init()
        self.node = Node("rl_geser_kanan")
        self.pub_module = self.node.create_publisher(String, "/robotis/enable_ctrl_module", 10)
        self.pub_cmd = self.node.create_publisher(String, "/robotis/walking/command", 10)
        self.pub_param = self.node.create_publisher(WalkingParam, "/robotis/walking/set_params", 10)
        self.pub_page = self.node.create_publisher(Int32, "/robotis/action/page_num", 10)
        self.cli_param = self.node.create_client(SetParameters, "/op3_walking_module/set_parameters")
        self.cli_get_param = self.node.create_client(GetParameters, "/op3_walking_module/get_parameters")
        self.cli_module = self.node.create_client(GetJointModule, "/robotis/get_present_joint_ctrl_modules")
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

    def check(self):
        """Pesan masalah, atau None kalau siap. Tidak menggerakkan robot."""
        if not self.cli_param.wait_for_service(timeout_sec=3.0):
            return ("/op3_walking_module/set_parameters tidak ada -- op3_manager mati, atau ROS_DOMAIN_ID / "
                    "ROS_LOCALHOST_ONLY terminal ini (sekarang %s / %s) beda dengan op3_manager"
                    % (os.environ.get("ROS_DOMAIN_ID", "-"), os.environ.get("ROS_LOCALHOST_ONLY", "-")))
        # Pesan yang dikirim sebelum op3_manager berlangganan hilang tanpa jejak.
        deadline = time.time() + 3.0
        while time.time() < deadline:
            if all(p.get_subscription_count() > 0 for p in (self.pub_module, self.pub_cmd, self.pub_param)):
                break
            time.sleep(0.1)
        else:
            return "op3_manager belum berlangganan topik walking"
        # Nama node baru lengkap setelah discovery, jadi dicek sesudah menunggu di atas.
        if any("demo" in n for n in self.node.get_node_names()):
            return "node demo sedang jalan -- matikan dulu, dua pemberi perintah walking akan saling timpa"
        self.orig_soft_start = self._get_soft_start()
        return None

    def _get_soft_start(self):
        if not self.cli_get_param.wait_for_service(timeout_sec=1.0):
            return None
        req = self.GetParameters.Request()
        req.names = ["soft_start_time"]
        fut = self.cli_get_param.call_async(req)
        deadline = time.time() + 1.0
        while not fut.done() and time.time() < deadline:
            time.sleep(0.02)
        if not fut.done() or not fut.result().values:
            return None
        return fut.result().values[0].double_value

    def _walking_active(self):
        if not self.cli_module.wait_for_service(timeout_sec=1.0):
            return False
        req = self.GetJointModule.Request()
        req.joint_name = ["r_knee", "l_knee"]
        fut = self.cli_module.call_async(req)
        deadline = time.time() + 1.0
        while not fut.done() and time.time() < deadline:
            time.sleep(0.02)
        return fut.done() and all(m == "walking_module" for m in fut.result().module_name)

    def ensure_walking(self):
        if self._walking_active():
            return
        self.pub_module.publish(self.String(data="walking_module"))
        time.sleep(WALKING_ENABLE_SETTLE)

    def set_soft_start(self, seconds):
        from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
        req = self.SetParameters.Request()
        req.parameters = [Parameter(name="soft_start_time",
                                    value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                                         double_value=float(seconds)))]
        self.soft_start_touched = True
        fut = self.cli_param.call_async(req)
        deadline = time.time() + 1.0
        while not fut.done() and time.time() < deadline:
            time.sleep(0.02)
        return fut.done() and fut.result().results and fut.result().results[0].successful

    def send_params(self, p, moving=True):
        m = self.WalkingParam()
        f = lambda k, d=0.0: float(p.get(k, d))
        m.init_x_offset, m.init_y_offset, m.init_z_offset = f("init_x_offset"), f("init_y_offset"), f("init_z_offset")
        m.init_roll_offset, m.init_pitch_offset, m.init_yaw_offset = (
            f("init_roll_offset"), f("init_pitch_offset"), f("init_yaw_offset"))
        m.period_time, m.dsp_ratio, m.step_fb_ratio = f("period_time", 0.6), f("dsp_ratio", 0.4), f("step_fb_ratio", 0.28)
        m.x_move_amplitude = f("x_move_amplitude") if moving else 0.0
        m.y_move_amplitude = f("y_move_amplitude") if moving else 0.0
        m.z_move_amplitude = f("z_move_amplitude")
        m.angle_move_amplitude = f("angle_move_amplitude") if moving else 0.0
        m.move_aim_on = bool(p.get("move_aim_on", False))
        m.balance_enable = bool(p.get("balance_enable", True))
        m.balance_hip_roll_gain, m.balance_knee_gain = f("balance_hip_roll_gain"), f("balance_knee_gain")
        m.balance_ankle_roll_gain, m.balance_ankle_pitch_gain = f("balance_ankle_roll_gain"), f("balance_ankle_pitch_gain")
        m.y_swap_amplitude, m.z_swap_amplitude = f("y_swap_amplitude"), f("z_swap_amplitude")
        m.arm_swing_gain, m.pelvis_offset, m.hip_pitch_offset = f("arm_swing_gain"), f("pelvis_offset"), f("hip_pitch_offset")
        m.p_gain, m.i_gain, m.d_gain = int(f("p_gain", 32)), int(f("i_gain")), int(f("d_gain"))
        self.pub_param.publish(m)

    def command(self, text):
        self.pub_cmd.publish(self.String(data=text))

    def init_page(self):
        self.pub_module.publish(self.String(data="action_module"))
        time.sleep(ACTION_MODULE_SETTLE)
        self.pub_page.publish(self.Int32(data=INIT_PAGE))

    def shutdown(self, send_stop=True):
        try:
            if send_stop:
                self.command("stop")
                time.sleep(0.1)
            if self.soft_start_touched and self.orig_soft_start is not None:
                if self.set_soft_start(self.orig_soft_start):
                    print(f"soft_start_time dikembalikan ke {self.orig_soft_start:.2f} dtk")
            # Executor dihentikan dan thread-nya ditunggu SEBELUM rclpy dimatikan;
            # kalau tidak, thread spin masih hidup saat proses keluar -> abort.
            self.executor.shutdown()
            self.spin_thread.join(timeout=2.0)
            self.node.destroy_node()
            self.rclpy.try_shutdown()
        except Exception:
            pass


class FakeRobot:
    def check(self): return None
    def ensure_walking(self): pass
    def set_soft_start(self, s): return True
    def send_params(self, p, moving=True): pass
    def command(self, text): print(f"    [tanpa robot] walking/command = {text}")
    def init_page(self): print("    [tanpa robot] page 2")
    def shutdown(self, send_stop=True): pass


def run_trial(robot, init_params, params, duration):
    """Satu percobaan seperti startPreset() D58. True kalau dihentikan operator."""
    robot.ensure_walking()
    robot.send_params(init_params, moving=False)
    time.sleep(INIT_SETTLE)
    if not robot.set_soft_start(params["soft_start_time"]):
        print("  ! soft_start_time gagal dipasang -- percobaan dibatalkan")
        return True
    # Pindah ke stance VERSI INI dulu, selagi diam. Walking module hanya
    # melandaikan perubahan offset Init Pose saat diam; begitu gait jalan, offset
    # baru dipakai seketika. Offset "geser kanan" berbeda dari "mutar kanan"
    # (hip_pitch/knee +-12 deg), dan menyentaknya dalam satu siklus 8 ms membuat
    # arus melonjak -> tegangan anjlok -> SEMUA servo mati (2026-10-26).
    robot.send_params(params, moving=False)
    time.sleep(STANCE_SETTLE)
    robot.send_params(params, moving=True)
    time.sleep(0.2)
    robot.command("start")
    print(f"  JALAN {duration:.1f} dtk  (tekan Enter untuk BERHENTI SEKARANG)")
    stopped = wait_or_enter(duration)
    if stopped:
        robot.command("stop")
        print("  DIHENTIKAN operator")
    else:
        robot.command("soft_stop")
    time.sleep(params["soft_start_time"] + params["period_time"] + 0.3)
    robot.send_params(init_params, moving=False)
    return stopped


def wait_or_enter(seconds):
    if not sys.stdin.isatty():
        time.sleep(seconds)
        return False
    end = time.time() + seconds
    while time.time() < end:
        r, _, _ = select.select([sys.stdin], [], [], max(0.0, end - time.time()))
        if r:
            sys.stdin.readline()
            return True
    return False


# ---------------------------------------------------------------- penilaian
def ask_ratings(auto=False):
    """dict penilaian, atau None kalau percobaan diulang (tidak dihitung)."""
    if auto:
        return {name: random.choice(opts)[0] for name, opts in RATINGS}
    print("  Nilai (boleh sekaligus 4 angka, mis. 1211; u = ulangi, tidak dihitung):")
    for name, opts in RATINGS:
        print("    %-11s %s" % (name, "  ".join(f"{i + 1}={o}" for i, (o, _) in enumerate(opts))))
    while True:
        raw = input("  > ").strip().lower()
        if raw == "u":
            return None
        digits = [c for c in raw if c.isdigit()]
        if len(digits) == len(RATINGS):
            try:
                return {name: opts[int(d) - 1][0] for (name, opts), d in zip(RATINGS, digits)}
            except IndexError:
                pass
        print("  ! ketik %d angka sesuai pilihan, mis. 1211" % len(RATINGS))


def score(ratings):
    total = 0.0
    for name, opts in RATINGS:
        total += dict(opts)[ratings[name]]
    return total


def fmt(params):
    return "  ".join(f"{k}={params[k]:.4f}" for k, _, _ in PARAM_SPACE)


# ---------------------------------------------------------------- utama
def match_manager_ros_env():
    """Samakan ROS_DOMAIN_ID / ROS_LOCALHOST_ONLY dengan op3_manager yang sedang
    jalan, lalu jalankan ulang skrip ini. Nilai beda = kedua sisi tidak saling
    melihat, dan stack di robot ini kadang menyala dengan LOCALHOST_ONLY 0,
    kadang 1. Lingkungannya dibaca dari /proc (satu container dengan manager)."""
    keys = ("ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY")
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as fh:
                argv0 = fh.read().split(b"\0")[0]
            if not argv0.endswith(b"lib/op3_manager/op3_manager"):
                continue
            with open(f"/proc/{pid}/environ", "rb") as fh:
                env = dict(kv.split(b"=", 1) for kv in fh.read().split(b"\0") if b"=" in kv)
        except OSError:
            continue
        wanted = {k: env.get(k.encode(), b"").decode() for k in keys}
        if all(os.environ.get(k, "") == v for k, v in wanted.items()):
            return
        new_env = dict(os.environ)
        for k, v in wanted.items():
            if v:
                new_env[k] = v
            else:
                new_env.pop(k, None)
        print("Menyamakan dengan op3_manager: " + ", ".join(f"{k}={v or '(kosong)'}" for k, v in wanted.items()), flush=True)
        os.execve(sys.executable, [sys.executable] + sys.argv, new_env)
    # Manager tidak ketemu: biarkan check() yang melapor.


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--durasi", type=float, default=3.0, help="lama geser per percobaan (detik), bawaan 3")
    ap.add_argument("--tanpa-robot", action="store_true", help="uji logika: tanpa ROS, penilaian acak")
    ap.add_argument("--percobaan", type=int, default=0, help="--tanpa-robot: jumlah percobaan otomatis")
    ap.add_argument("--baru", action="store_true", help="mulai dari awal (state lama disimpan dengan nama lain)")
    ap.add_argument("--cek", action="store_true", help="hanya periksa sambungan ke op3_manager, robot tidak digerakkan")
    args = ap.parse_args()
    if not args.tanpa_robot:
        match_manager_ros_env()

    os.makedirs(RUN_DIR, exist_ok=True)
    state_path = os.path.join(RUN_DIR, "state.json")
    log_path = os.path.join(RUN_DIR, "log.csv")
    if args.baru and os.path.exists(state_path):
        os.rename(state_path, state_path + "." + datetime.datetime.now().strftime("%Y%m%d-%H%M%S"))

    versions = load_versions()
    base = newest_version(versions, BASE_PRESET)
    init = newest_version(versions, INIT_PRESET)
    if base is None or init is None:
        sys.exit(f"Versi '{BASE_PRESET}' atau '{INIT_PRESET}' tidak ada di {VERSIONS_PATH}")
    base_params = base["params"]
    if not base_params.get("soft_start_time"):
        base_params["soft_start_time"] = 0.5

    learner = Learner(base_params, state_path)
    robot = FakeRobot() if args.tanpa_robot else Robot()
    problem = robot.check()
    if problem:
        robot.shutdown(send_stop=False)
        sys.exit("Tidak bisa mulai: " + problem)
    if args.cek:
        robot.shutdown(send_stop=False)
        print("Siap: op3_manager menjawab, tidak ada node demo, versi '%s' dan '%s' ditemukan."
              % (BASE_PRESET, INIT_PRESET))
        return

    print(f"RL geser kanan -- {'LANJUTAN' if learner.resumed else 'BARU'}, "
          f"percobaan ke-{learner.trial_no + 1}, pembaruan ke-{learner.update_no}")
    print(f"Dasar: versi '{BASE_PRESET}' (updatedAt {base.get('updatedAt')}), init dari '{INIT_PRESET}'")
    print(f"Data: {RUN_DIR}")
    print("ROBOT HARUS DIJAGA/DIPEGANG. Mulai dari page 2 (WALKING_READY).")

    new_log = not os.path.exists(log_path)
    log_fh = open(log_path, "a", newline="", encoding="utf-8")
    log = csv.writer(log_fh)
    if new_log:
        log.writerow(["waktu", "percobaan", "pembaruan", "sisi"] + [k for k, _, _ in PARAM_SPACE]
                     + [n for n, _ in RATINGS] + ["skor"])

    auto_left = args.percobaan
    try:
        while True:
            if args.tanpa_robot and auto_left <= 0:
                break
            if not args.tanpa_robot:
                print("\n[Enter] percobaan  [b] jalankan terbaik  [i] init pose page 2  "
                      "[s] simpan terbaik  [r] ringkasan  [q] keluar")
                cmd = input("> ").strip().lower()
            else:
                cmd = ""
                auto_left -= 1

            if cmd == "q":
                break
            if cmd == "i":
                robot.init_page()
                print("  page 2 diputar -- tunggu robot berdiri")
                continue
            if cmd == "r":
                print(f"  percobaan {learner.trial_no}, pembaruan {learner.update_no}")
                print("  mu   : " + fmt(to_physical(learner.mu)))
                print("  sigma: " + "  ".join(f"{s:.3f}" for s in learner.sigma))
                if learner.best:
                    print(f"  terbaik (percobaan {learner.best['trial_no']}, skor {learner.best['reward']:+.1f}): "
                          + fmt(learner.best["params"]))
                continue
            if cmd == "s":
                if not learner.best:
                    print("  belum ada percobaan")
                    continue
                note = f"RL percobaan {learner.best['trial_no']} skor {learner.best['reward']:+.1f}"
                bak = save_result_version(learner.full_params(learner.best["params"]), note)
                print(f"  disimpan sebagai '{RESULT_PRESET}' ({note}); cadangan: {bak}")
                continue
            if cmd == "b":
                if not learner.best:
                    print("  belum ada percobaan")
                    continue
                run_trial(robot, init["params"], learner.full_params(learner.best["params"]), args.durasi)
                continue
            if cmd != "":
                continue

            label, x = learner.next_candidate()
            physical = to_physical(x)
            print(f"\nPercobaan {learner.trial_no + 1} (pasangan {label}): " + fmt(physical))
            stopped = run_trial(robot, init["params"], learner.full_params(physical), args.durasi)
            ratings = ask_ratings(auto=args.tanpa_robot)
            if ratings is None:
                print("  diulang -- percobaan ini tidak dihitung")
                continue
            r = score(ratings)
            updated = learner.record(label, x, r)
            log.writerow([datetime.datetime.now().isoformat(timespec="seconds"), learner.trial_no,
                          learner.update_no, label] + [f"{physical[k]:.5f}" for k, _, _ in PARAM_SPACE]
                         + [ratings[n] for n, _ in RATINGS] + [f"{r:.2f}"])
            log_fh.flush()
            print(f"  skor {r:+.1f}" + ("  (dihentikan operator)" if stopped else ""))
            if updated:
                print(f"  >> pembaruan {learner.update_no}: setelan tengah sekarang " + fmt(to_physical(learner.mu)))
            if ratings["kestabilan"] == "jatuh" and not args.tanpa_robot:
                print("  JATUH: berdirikan robot, lalu tekan [i] untuk page 2 sebelum percobaan berikutnya")
    except KeyboardInterrupt:
        print("\nCtrl-C: walking dihentikan")
    finally:
        robot.shutdown()
        log_fh.close()
        learner.save()


if __name__ == "__main__":
    main()
