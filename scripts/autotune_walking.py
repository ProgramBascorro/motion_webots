#!/usr/bin/env python3
"""Auto-tune walking FASE 1: jalan lurus maju, mulai dari versi Studio "JALAN BARU".

APA YANG DILAKUKAN
------------------
Robot mencoba satu set parameter walking, diukur dari IMU, diberi skor, lalu
CMA-ES (keluarga RL "policy search" per episode) memilih set berikutnya.
Diulang sampai anggaran percobaan habis. walking_module TIDAK diganti -- yang
dicari hanya angka-angka di tab Walking Studio.

Tiap percobaan (Enter dari operator dulu):
  1. batas kecepatan servo dilepas (Torque ON Studio bisa meninggalkannya)
  2. page 2 WALKING_READY diputar, ditunggu sampai action_module selesai
  3. acuan page 2 dipatok, walking_module dinyalakan (pose terekam = page 2)
  4. parameter percobaan dikirim dan DIBACA BALIK sampai sama
  5. berdiri 1 s (bias gyro), "start", jalan --walk-sec detik, "stop"
  6. skor dari IMU: goyangan gyro, variasi kemiringan, condong badan, dan
     belokan (integral gyro_z) -- jatuh = skor buruk, makin cepat jatuh makin buruk

Langkah maju (x_move_amplitude) DIKUNCI di nilai JALAN BARU: skornya cuma
kestabilan, jadi tanpa kunci itu "jalan di tempat" yang menang. Jarak tempuh
bisa ikut dinilai dengan --measure-distance (operator mengetik cm tiap percobaan).

KESELAMATAN -- WAJIB
--------------------
* Robot DIGANTUNG tali kendur (menangkap jatuh, tidak menahan berat) dan ada
  orang di dekatnya. Lintasan lurus kosong >= 1 m.
* Tiap percobaan menunggu Enter. Ctrl+C kapan saja = "stop" dikirim, sesi disimpan.
* Berhenti sendiri kalau roll/pitch tertapis > 45 deg. Suhu servo TIDAK terbaca
  dari bus, jadi ada jeda --cooldown-sec antar percobaan; raba servo lutut/engkel.
* Jangan pakai tombol walking di Studio dan jangan nyalakan D60 selama tuning.

PAKAI (di DALAM kontainer, op3_manager hidup, terminal interaktif -it)
-----
  docker exec -it op3 bash -lc 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash \\
      && python3 /ros2_ws/scripts/autotune_walking.py --session jb1'

  --session NAMA        sesi baru, atau dilanjutkan kalau sudah ada (ganti baterai, istirahat)
  --trials N            anggaran TOTAL percobaan CMA sesi (bawaan 48 = 6 generasi x 8).
                        Sesi yang anggarannya habis dilanjutkan dengan angka lebih besar,
                        mis. --session jb1 --trials 64 = 2 generasi lagi
  --report              tabel semua percobaan sesi itu
  --rerun-best N        ulangi kandidat terbaik N kali (cek bukan kebetulan)
  --trial N             dengan --rerun-best: ulangi percobaan N, bukan yang terbaik
                        (menu Training -> "Uji hasil ini di robot")
  --save-best "NAMA"    simpan terbaik sebagai versi Studio BARU (Studio harus hidup);
                        tidak pernah menimpa versi yang sudah ada. Sama dengan menu Training ->
                        Riwayat -> hasil -> "Simpan langsung ke robot" (tercatat di hasil.json)
  --label "2=h,4=j"     penilaian operator untuk percobaan yang sudah tercatat
                        (n = normal, j = jatuh/tertahan tali, h = hampir jatuh/dipegang)
  --rescore             hitung ulang metrik sesi dari rekaman mentah dengan rumus sekarang
  --export [CSV]        semua percobaan SEMUA sesi jadi satu dataset (bawaan
                        data/autotune/dataset.csv) -- bekal tuning/model berikutnya
  --warm-start SESI     sesi BARU mulai dari kandidat terbaik sesi lain (acuan page 2 sama);
                        SESI:N = mulai dari percobaan N sesi itu
  --weights k=v,...     sesi BARU: bobot skor lain (kunci = metrik skor, dinormalkan ke 1)
  --feedback-run        jalankan iterasi feedback yang MENUNGGU (scripts/training.py feedback-apply,
                        menu: hasil -> feedback -> "Terapkan & jalankan"), lalu berhenti
  --follow-reference    lanjutkan sesi sesudah page 2 di-teach ulang: offset pose digeser
                        ke acuan baru (sama seperti versi Studio), pencarian berlanjut

Sesudah tiap percobaan operator ditanya hasilnya (Enter = setuju dengan detektor).
Dengan robot digantung, tali menahan badan, jadi label operator mengalahkan detektor.
"c" di pertanyaan itu = tulis catatan bebas (beberapa baris) untuk percobaan tersebut;
tersimpan di <sesi>/feedback.jsonl. Catatan itu LANGSUNG dipahami dan diterjemahkan
ke perubahan (training_iteration.py); kalau operator menyetujuinya, percobaan
BERIKUTNYA menjalankan nilai yang sudah diubah (jenis "feedback"), hasilnya
dibandingkan dengan percobaan yang dinilai, dan CMA melanjutkan dari titik itu.

Data sesi: src/bascorro_studio/data/autotune/<sesi>/ -- state.json + trials.csv ikut
repo (bekal), raw/ (rekaman IMU per percobaan, untuk --rescore) di-gitignore.
"""

import argparse
import csv
import datetime
import gzip
import json
import os
import re
import sys
import time
import urllib.error
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import autotune_walking_core as core  # noqa: E402
import training_feedback as feedback  # noqa: E402
import training_iteration as iteration  # noqa: E402
from walking_ready_fk import HERE, REFERENCE_JSON, STUDIO_VERSIONS  # noqa: E402

# Env hanya untuk uji (robot tiruan + penyimpanan sementara), supaya pengujian
# tidak pernah menyentuh sesi atau versi tersimpan milik operator.
SESSIONS_DIR = os.environ.get("BASCORRO_AUTOTUNE_DIR", os.path.join(HERE, "src/bascorro_studio/data/autotune"))
STUDIO_URL = os.environ.get("BASCORRO_STUDIO_STORE_URL", "http://localhost:5173/__bascorro/walking-versions")
STATE_FORMAT = "bascorro-studio/walking-autotune"
DEFAULT_BUDGET = 48  # 6 generasi x popsize 8
WALKING_READY_PAGE = 2
LEG_JOINTS = ["r_hip_yaw", "r_hip_roll", "r_hip_pitch", "r_knee", "r_ank_pitch", "r_ank_roll",
              "l_hip_yaw", "l_hip_roll", "l_hip_pitch", "l_knee", "l_ank_pitch", "l_ank_roll"]
ALL_JOINTS = LEG_JOINTS + ["r_sho_pitch", "l_sho_pitch", "r_sho_roll", "l_sho_roll", "r_el", "l_el",
                           "head_pan", "head_tilt"]
INT_FIELDS = {"p_gain", "i_gain", "d_gain"}
BOOL_FIELDS = {"move_aim_on", "balance_enable"}


class StanceUnstable(Exception):
    """Pose berdiri kandidat sudah condong sebelum melangkah -- salah parameternya, bukan sistem."""


def now_iso():
    return datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")


# ---- sesi ---------------------------------------------------------------------

class Session:
    """state.json + trials.csv + raw/ satu sesi. Subkelas (training page Action
    Editor) mengganti FORMAT dan write_csv; sisanya sama."""

    FORMAT = STATE_FORMAT

    def __init__(self, name):
        self.name = name
        self.dir = os.path.join(SESSIONS_DIR, name)
        self.path = os.path.join(self.dir, "state.json")
        self.lock_path = os.path.join(self.dir, "running.json")
        self.state = None

    def exists(self):
        return os.path.exists(self.path)

    def load(self):
        with open(self.path, encoding="utf-8") as handle:
            self.state = json.load(handle)
        if self.state.get("format") != self.FORMAT:
            raise SystemExit("%s bukan berkas sesi %s" % (self.path, self.FORMAT))
        return self.state

    def save(self):
        os.makedirs(os.path.join(self.dir, "raw"), exist_ok=True)
        temporary = self.path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as handle:
            json.dump(self.state, handle, indent=1)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, self.path)
        with open(os.path.join(self.dir, "trials.csv"), "w", newline="", encoding="utf-8") as handle:
            self.write_csv(csv.writer(handle))

    def write_csv(self, writer):
        names = self.state["params"] + iteration.extra_keys(self.state)
        writer.writerow(["n", "jenis", "generasi", "skor", "jatuh", "arah", "jarak_cm", "volt"] + names + METRIC_KEYS)
        for t in self.state["trials"]:
            m = t.get("metrics") or {}
            full = iteration.full_params(self.state, t)
            writer.writerow([t["n"], t["kind"], t.get("generation", ""), "%.4f" % t["cost"], int(bool(m.get("fell"))),
                             core.turn_direction(m.get("yaw_drift_deg")),
                             "" if t.get("distance_cm") is None else t["distance_cm"], t.get("voltage") or ""]
                            + [full.get(k, "") for k in names]
                            + ["" if m.get(k) is None else "%.3f" % m[k] for k in METRIC_KEYS])

    # -- "sedang berjalan": dibaca menu Training (Hasil) dan mencegah dua proses
    #    menulis sesi yang sama.
    def running_pid(self):
        try:
            with open(self.lock_path, encoding="utf-8") as handle:
                pid = int(json.load(handle)["pid"])
            os.kill(pid, 0)
            return pid
        except (OSError, ValueError, KeyError):
            return None

    def lock(self):
        other = self.running_pid()
        if other and other != os.getpid():
            raise SystemExit("sesi %r sedang dipakai proses lain (pid %d) -- tutup dulu terminal itu" % (self.name, other))
        os.makedirs(self.dir, exist_ok=True)
        with open(self.lock_path, "w", encoding="utf-8") as handle:
            json.dump({"pid": os.getpid(), "since": now_iso()}, handle)

    def unlock(self):
        if self.running_pid() == os.getpid():
            os.remove(self.lock_path)

    def save_raw(self, n, imu, joints, marks):
        os.makedirs(os.path.join(self.dir, "raw"), exist_ok=True)
        with gzip.open(os.path.join(self.dir, "raw", "trial_%03d.json.gz" % n), "wt", encoding="utf-8") as handle:
            json.dump({"imu": imu, "joints": joints, "marks": marks}, handle)


def load_start(key):
    """Preset Studio menurut nama PERSIS atau id."""
    with open(STUDIO_VERSIONS, encoding="utf-8") as handle:
        store = json.load(handle)
    with open(REFERENCE_JSON, encoding="utf-8") as handle:
        reference = json.load(handle)
    if store.get("poseReferenceId") != reference["id"]:
        raise SystemExit("versi Studio bertanda acuan %r, acuan sekarang %r -- jalankan dulu "
                         "scripts/walking_ready_fk.py --write" % (store.get("poseReferenceId"), reference["id"]))
    found = [v for v in store["versions"] if key in (v["name"], v.get("id"))]
    if not found:
        raise SystemExit("versi %r tidak ada di %s" % (key, STUDIO_VERSIONS))
    return found[0], dict(found[0]["params"]), reference


# Preset putar/geser menempuh lebih jauh dan memelintir tali gantungan: lebih pendek.
WALK_SEC_BY_MOTION = {"maju": 12.0, "mundur": 12.0, "di_tempat": 12.0, "geser": 8.0, "putar": 8.0}


def new_state(args):
    version, start, reference = load_start(args.start)
    motion, motion_text = core.walking_motion(start)
    if args.motion:
        motion, motion_text = args.motion, "%s (dipilih operator; dari angka: %s)" % (args.motion, motion_text)
    names = args.params.split(",") if args.params else core.motion_params(motion)
    space = core.Space(names, start, reference["offsets"])
    x0, warm = (warm_start_point(args.warm_start.split(","), space, start, reference["id"]) if args.warm_start
                else (space.to_unit(start), None))
    es = core.CMAES(x0, args.sigma, popsize=args.popsize, seed=args.seed)
    return {
        "format": STATE_FORMAT, "formatVersion": 1, "metricsVersion": core.METRICS_VERSION, "createdAt": now_iso(),
        "source": "studio", "motion": motion, "motion_text": motion_text,
        "start_name": version["name"], "start_id": version.get("id"), "start_params": start,
        "reference_id": reference["id"], "reference_offsets": reference["offsets"],
        "params": names, "lo": space.lo.tolist(), "hi": space.hi.tolist(),
        "sigma0": args.sigma, "popsize": es.lam,
        "walk_sec": args.walk_sec if args.walk_sec is not None else WALK_SEC_BY_MOTION[motion],
        "measure_distance": args.measure_distance,
        "budget": args.trials if args.trials is not None else DEFAULT_BUDGET, "baseline_runs": args.baseline_runs, "warm_start": warm,
        "weights": parse_weights(args.weights, core.WEIGHTS) if args.weights else None,
        "cma": es.state(), "pending": [], "evaluated": [],
        "trials": [], "feedback_steps": [], "fixed": {},
    }


def parse_weights(text, allowed):
    """"gyro_rms_dps=0.4,lean_deg=0.2" -> bobot lengkap (yang tidak disebut tetap), jumlah 1."""
    weights = dict(allowed)
    for part in text.split(","):
        key, _, value = part.strip().partition("=")
        if key not in allowed:
            raise SystemExit("--weights: %r bukan metrik skor (pilih dari %s)" % (key, ", ".join(allowed)))
        try:
            weights[key] = float(value)
        except ValueError:
            raise SystemExit("--weights: nilai %r untuk %s bukan angka" % (value, key))
        if weights[key] < 0:
            raise SystemExit("--weights: bobot %s negatif" % key)
    total = sum(weights.values())
    if total <= 0:
        raise SystemExit("--weights: semua bobot nol")
    return {k: v / total for k, v in weights.items()}


def plan_lines(state, metric_text=None):
    """Bobot sesi, variabel tetap, dan iterasi feedback untuk laporan (kosong kalau sesi biasa)."""
    lines = []
    if state.get("weights"):
        lines.append("bobot skor sesi ini (bukan bawaan): %s" % ", ".join(
            "%s %.0f%%" % (k, 100 * w) for k, w in state["weights"].items()))
    plan = state.get("feedback_plan")       # sesi lama (sebelum 27 Okt): rencana feedback saat sesi dibuat
    if plan:
        source = plan.get("source", {})
        lines.append("dimulai dari rencana feedback lama: sesi %s percobaan %s, catatan %s" % (
            source.get("session", "-"), source.get("trial", "-"), ", ".join(plan.get("notes", [])) or "-"))
    if state.get("fixed"):
        lines.append("ditetapkan feedback untuk semua percobaan berikutnya: %s" % ", ".join(
            "%s=%g" % kv for kv in state["fixed"].items()))
    if iteration.steps(state):
        lines.append("iterasi feedback:")
        lines += ["  " + line for line in iteration.history_text(state).splitlines()]
    return lines


def space_of(state):
    space = core.Space(state["params"], state["start_params"], state["reference_offsets"])
    space.lo, space.hi = core.np.array(state["lo"]), core.np.array(state["hi"])
    return space


LABEL_KEYS = {"n": "normal", "j": "jatuh", "h": "hampir"}


def usable(trial, weights=None):
    """Percobaan yang benar-benar berjalan normal menurut detektor DAN operator.
    weights = metrik yang wajib ada (bawaan: skor walking)."""
    m = trial.get("metrics") or {}
    label = trial.get("label")
    if m.get("stance_unstable") or label in ("jatuh", "hampir") or (m.get("fell") and label != "normal"):
        return False
    return all(m.get(k) is not None for k in (weights or core.WEIGHTS))


def baseline_trials(state, is_usable=usable):
    return [t for t in state["trials"] if t["kind"] == "baseline" and is_usable(t)]


def yaw_target(state, base=None):
    """Belok yang diminta preset: 0, kecuali preset putar = rata-rata belok baseline-nya."""
    if not core.motion_keeps_turn(state.get("motion", "maju")):
        return 0.0
    turns = [t["metrics"]["yaw_drift_deg"] for t in (baseline_trials(state) if base is None else base)]
    return sum(turns) / len(turns) if turns else 0.0


def score(state, trial):
    base = baseline_trials(state)
    target = yaw_target(state, base)
    # Tanpa baseline sah (preset asli selalu jatuh): skala minimum, supaya percobaan yang
    # berhasil tetap jauh di bawah skor jatuh (5..10).
    scales = (core.scales_from_baseline([t["metrics"] for t in base], target) if base else dict(core.FLOORS))
    dists = [t["distance_cm"] for t in base if t.get("distance_cm") is not None]
    base_d = sum(dists) / len(dists) if dists else None
    if trial.get("metrics") is None:
        return core.COST_UNREACHABLE
    return core.cost(trial["metrics"], scales, trial.get("distance_cm"), base_d, trial.get("label"), target,
                     state.get("weights"))


def recost_all(state):
    """Skala baseline bisa berubah (baseline baru, label, rescore): semua skor dihitung ulang."""
    for trial in state["trials"]:
        trial["cost"] = score(state, trial)


def candidate_cost(state, trial):
    """Skor kandidat = rata-rata percobaan aslinya + semua ulangannya (--rerun-best).
    Ulangan yang jatuh ikut dihitung: kandidat yang kadang jatuh bukan juara."""
    runs = [trial["cost"]] + [t["cost"] for t in state["trials"] if t.get("rerun_of") == trial["n"]]
    return sum(runs) / len(runs), len(runs) - 1


CANDIDATE_KINDS = ("baseline", "cma", "feedback")


def best_trial(state, is_usable=usable):
    """Kandidat terbaik dari baseline + CMA + iterasi feedback. Ulangan TIDAK bersaing
    sendiri; ia hanya mengoreksi skor kandidat yang diulangnya."""
    ok = [t for t in state["trials"] if t["kind"] in CANDIDATE_KINDS and is_usable(t)]
    return min(ok, key=lambda t: candidate_cost(state, t)[0]) if ok else None


def report(state):
    names = state["params"] + iteration.extra_keys(state)
    print("sesi mulai %s dari %r, acuan %s, %d percobaan" % (state["createdAt"], state["start_name"],
                                                          state["reference_id"], len(state["trials"])))
    motion = state.get("motion", "maju")
    print("jenis gerak: %s | sasaran belok %s" % (
        state.get("motion_text", motion),
        "%s (belok preset sendiri)" % core.turn_text(yaw_target(state)) if core.motion_keeps_turn(motion) else "lurus (0 deg)"))
    for line in plan_lines(state, core.METRIC_TEXT):
        print(line)
    print("%4s %-8s %4s %7s %6s %-15s %s" % ("n", "jenis", "gen", "skor", "jatuh", "arah", "  ".join(names)))
    for t in state["trials"]:
        m = t.get("metrics") or {}
        gen = "" if t.get("generation") is None else t["generation"]
        flag = {"jatuh": "YA", "hampir": "hampir", "normal": ""}.get(t.get("label"), "YA" if m.get("fell") else "")
        flag = "pose" if m.get("stance_unstable") else flag
        full = iteration.full_params(state, t)
        print("%4d %-8s %4s %7.3f %6s %-15s %s" % (t["n"], t["kind"], gen, t["cost"], flag,
                                                core.turn_text(m.get("yaw_drift_deg")),
                                                "  ".join("%g" % full[k] for k in names)))
    turns = [t["metrics"]["yaw_drift_deg"] for t in state["trials"]
             if (t.get("metrics") or {}).get("yaw_drift_deg") is not None]
    if turns:
        recent = turns[-8:]
        count = lambda xs, d: sum(1 for y in xs if core.turn_direction(y) == d)
        print("\narah belok (|belok| < %.0f deg = lurus): semua %d percobaan -> kanan %d, kiri %d, lurus %d | "
              "8 terakhir -> kanan %d, kiri %d, lurus %d, rata-rata %s"
              % (core.TURN_STRAIGHT_DEG, len(turns), count(turns, "kanan"), count(turns, "kiri"), count(turns, "lurus"),
                 count(recent, "kanan"), count(recent, "kiri"), count(recent, "lurus"),
                 core.turn_text(sum(recent) / len(recent))))
    best = best_trial(state)
    if best:
        mean, reruns = candidate_cost(state, best)
        base = [t["cost"] for t in state["trials"] if t["kind"] == "baseline" and not t["metrics"].get("fell")]
        print("\nterbaik: percobaan %d, skor %.3f%s | baseline %r: %s" % (
            best["n"], mean, " (rata-rata dengan %d ulangan)" % reruns if reruns else " (belum diulang)",
            state["start_name"], " ".join("%.3f" % c for c in base) or "-"))
        full = iteration.full_params(state, best)
        for k in names:
            print("  %-22s %10g  (awal %g)" % (k, full[k], state["start_params"][k]))


# ---- ROS ----------------------------------------------------------------------

def adopt_manager_ros_env():
    """Domain ROS diambil dari op3_manager yang hidup (sama dengan calib_imu_yaw.py):
    peluncur tmux real_robot memakai ROS_DOMAIN_ID 1, shell container 42, dan tanpa ini
    alat berkata "service tidak ada" padahal stack sehat. WAJIB sebelum rclpy.init().
    BASCORRO_TRAINING_ROS_ENV=keep (uji dengan robot tiruan) = env shell dipakai apa adanya."""
    if os.environ.get("BASCORRO_TRAINING_ROS_ENV") == "keep":
        return
    import subprocess
    pids = subprocess.run(["pgrep", "-f", "[o]p3_manager/lib/op3_manager/op3_manager"],
                          capture_output=True, text=True).stdout.split()
    if not pids:
        print("op3_manager tidak terlihat -- memakai env shell (domain %s)" % os.environ.get("ROS_DOMAIN_ID", "0"))
        return
    try:
        with open("/proc/%s/environ" % pids[0], "rb") as handle:
            for entry in handle.read().decode("utf-8", "replace").split("\0"):
                name, _, value = entry.partition("=")
                if name in ("ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "RMW_IMPLEMENTATION"):
                    os.environ[name] = value
    except OSError:
        return
    print("memakai env ROS op3_manager (pid %s): domain %s, localhost_only %s"
          % (pids[0], os.environ.get("ROS_DOMAIN_ID"), os.environ.get("ROS_LOCALHOST_ONLY")))


class Robot:
    def __init__(self, node_name="walking_autotune"):
        adopt_manager_ros_env()
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Int32, String
        from sensor_msgs.msg import Imu, JointState
        from op3_walking_module_msgs.msg import WalkingParam
        from op3_walking_module_msgs.srv import GetWalkingParam
        from op3_action_module_msgs.srv import IsRunning
        from robotis_controller_msgs.msg import StatusMsg, SyncWriteItem
        from robotis_controller_msgs.srv import GetJointModule
        self.rclpy, self.String, self.Int32, self.WalkingParam = rclpy, String, Int32, WalkingParam
        self.SyncWriteItem, self.GetJointModule = SyncWriteItem, GetJointModule
        self.GetWalkingParam, self.IsRunning = GetWalkingParam, IsRunning
        rclpy.init()
        self.node = Node(node_name)
        # Penerbit ANGGOTA (penemuan DDS asinkron; penerbit sekali-pakai kehilangan pesan).
        self.module_pub = self.node.create_publisher(String, "/robotis/enable_ctrl_module", 10)
        self.page_pub = self.node.create_publisher(Int32, "/robotis/action/page_num", 10)
        self.param_pub = self.node.create_publisher(WalkingParam, "/robotis/walking/set_params", 10)
        self.command_pub = self.node.create_publisher(String, "/robotis/walking/command", 10)
        self.sync_pub = self.node.create_publisher(SyncWriteItem, "/robotis/sync_write_item", 10)
        self.modules_cli = self.node.create_client(GetJointModule, "/robotis/get_present_joint_ctrl_modules")
        self.params_cli = self.node.create_client(GetWalkingParam, "/robotis/walking/get_params")
        self.action_cli = self.node.create_client(IsRunning, "/robotis/action/is_running")
        self.node.create_subscription(Imu, "/robotis/open_cr/imu", self._imu, 50)
        self.node.create_subscription(JointState, "/robotis/present_joint_states", self._joints, 20)
        self.node.create_subscription(StatusMsg, "/robotis/status", self._status, 50)
        self.recording = False
        self.imu, self.joints = [], []
        self.tilt = core.TiltFilter(core.FALL_TAU_S)
        self.fall = None
        self.fell_at = None
        self.voltage = None
        self.walk_started_at = None

    # -- callback
    def _imu(self, msg):
        if not self.recording:
            return
        q = msg.orientation
        roll, pitch, yaw = core.quat_to_rpy_deg(q.x, q.y, q.z, q.w)
        t = time.monotonic()
        g = msg.angular_velocity
        self.imu.append((t, g.x, g.y, g.z, roll, pitch, yaw))
        self.tilt.update(t, roll, pitch)
        if self.fall is not None and self.fell_at is None and self.fall.update(t, roll, pitch):
            self.fell_at = t

    def _joints(self, msg):
        if self.recording and "r_hip_pitch" in msg.name:
            self.joints.append((time.monotonic(), msg.position[msg.name.index("r_hip_pitch")]))

    def _status(self, msg):
        match = re.search(r"Present Volt\s*:\s*([0-9.]+)", msg.status_msg)
        if match:
            self.voltage = float(match.group(1))
        if msg.status_msg == "Start walking":
            self.walk_started_at = time.monotonic()

    # -- utilitas
    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.005)

    def call(self, client, request, timeout=3.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("service %s tidak ada -- op3_manager hidup?" % client.srv_name)
        future = client.call_async(request)
        self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError("service %s tidak menjawab" % client.srv_name)
        return future.result()

    def legs_owned_by(self, module):
        response = self.call(self.modules_cli, self.GetJointModule.Request(joint_name=LEG_JOINTS))
        owners = dict(zip(response.joint_name, response.module_name))
        return all(owners.get(j) == module for j in LEG_JOINTS)

    def read_params(self):
        response = self.call(self.params_cli, self.GetWalkingParam.Request(get_param=True))
        return {k: getattr(response.parameters, k) for k in self._fields()}

    def _fields(self):
        return [f for f in self.WalkingParam.get_fields_and_field_types()]

    def publish_params(self, params):
        msg = self.WalkingParam()
        for key in self._fields():
            value = params[key]
            setattr(msg, key, bool(value) if key in BOOL_FIELDS else int(value) if key in INT_FIELDS else float(value))
        self.param_pub.publish(msg)

    def apply_params(self, params, label):
        for _ in range(5):
            self.publish_params(params)
            self.spin_for(0.15)
            back = self.read_params()
            if all(_same(back[k], params[k]) for k in self._fields()):
                return
        raise RuntimeError("%s tidak mendarat di walking_module (ada penulis lain? Studio/D60)" % label)

    def clear_speed_limit(self):
        for item in ("profile_velocity", "profile_acceleration"):
            self.sync_pub.publish(self.SyncWriteItem(item_name=item, joint_name=ALL_JOINTS, value=[0] * len(ALL_JOINTS)))
            self.spin_for(0.3)  # satu item per siklus sync write, kalau tidak bertabrakan

    def action_running(self):
        return self.call(self.action_cli, self.IsRunning.Request()).is_running

    def take_action_module(self):
        self.module_pub.publish(self.String(data="action_module"))
        self.spin_for(0.6)
        if not self.legs_owned_by("action_module"):
            raise RuntimeError("action_module belum memegang kaki")

    def play_page(self, page, timeout=5.0, settle=1.8):
        self.take_action_module()
        self.page_pub.publish(self.Int32(data=page))
        self.spin_for(settle)
        deadline = time.monotonic() + timeout
        while self.action_running():
            if time.monotonic() > deadline:
                raise RuntimeError("page %d tidak selesai diputar" % page)
            self.spin_for(0.1)

    def play_page2(self):
        self.play_page(WALKING_READY_PAGE)

    def begin_recording(self):
        self.imu, self.joints, self.fell_at, self.fall = [], [], None, None
        self.tilt = core.TiltFilter(core.FALL_TAU_S)
        self.recording = True

    def enable_walking(self, reference):
        # Acuan dulu, baru modul menyala: bias terekam terhadap acuan page 2
        # (sama dengan Studio dan D60), jadi angka percobaan berarti "page 2 + selisih".
        self.apply_params(reference, "acuan page 2")
        self.module_pub.publish(self.String(data="walking_module"))
        self.spin_for(1.2)
        if not self.legs_owned_by("walking_module"):
            raise RuntimeError("walking_module belum memegang kaki")

    def command(self, text):
        self.command_pub.publish(self.String(data=text))

    def shutdown(self):
        try:
            self.command("stop")
            self.spin_for(0.2)
        finally:
            self.node.destroy_node()
            self.rclpy.shutdown()


def _same(a, b):
    if isinstance(a, bool) or isinstance(b, bool):
        return bool(a) == bool(b)
    return abs(float(a) - float(b)) <= 1e-6 * max(1.0, abs(float(a)), abs(float(b)))


def watch_stance(robot, page2_tilt, seconds, reference):
    """Awasi pose berdiri baru. Condong > STANCE_LIMIT_DEG dari page 2 = kembali ke
    page 2 SEBELUM melangkah. Sesi coba 23 Okt: dua kandidat terguling di sini."""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        robot.spin_for(0.02)
        roll, pitch = robot.tilt.roll, robot.tilt.pitch
        if roll is None:
            continue
        deviation = max(abs(roll - page2_tilt[0]), abs(pitch - page2_tilt[1]))
        if deviation > core.STANCE_LIMIT_DEG:
            robot.apply_params(reference, "acuan page 2 (pembatalan)")
            raise StanceUnstable("pose berdiri condong %.1f deg dari page 2" % deviation)


def run_trial(robot, state, params):
    """Satu percobaan di robot. Mengembalikan (imu, joints, marks); RuntimeError =
    gagal sistem, StanceUnstable = pose kandidat tidak sanggup berdiri."""
    reference = dict(params)
    reference.update(state["reference_offsets"])
    robot.clear_speed_limit()
    robot.play_page2()
    robot.enable_walking(reference)
    robot.begin_recording()
    robot.spin_for(0.6)
    if robot.tilt.roll is None:
        raise RuntimeError("IMU tidak terbit (/robotis/open_cr/imu)")
    page2_tilt = (robot.tilt.roll, robot.tilt.pitch)
    marks = {"page2": time.monotonic(), "page2_tilt": page2_tilt}
    robot.apply_params(params, "parameter percobaan")
    watch_stance(robot, page2_tilt, 1.5, reference)
    marks["stand0"] = time.monotonic()
    watch_stance(robot, page2_tilt, 1.0, reference)
    robot.fall = core.FallWatch(seed=(robot.tilt.t, robot.tilt.roll, robot.tilt.pitch))
    robot.walk_started_at = None
    marks["start"] = time.monotonic()
    robot.command("start")
    end = marks["planned_stop"] = marks["start"] + state["walk_sec"]
    while time.monotonic() < end and robot.fell_at is None:
        robot.spin_for(0.02)
        if robot.walk_started_at is None and time.monotonic() - marks["start"] > 1.5:
            robot.command("stop")
            raise RuntimeError("walking_module tidak melapor 'Start walking'")
    robot.command("stop")
    marks["stop"] = time.monotonic()
    marks["fell_at"] = robot.fell_at
    # Soft stop menurunkan amplitudo selama soft_start_time, lalu gait selesai di
    # batas fase berikutnya; rekaman diteruskan sampai robot berdiri diam.
    robot.spin_for(params["soft_start_time"] + 2 * params["period_time"] + 0.8)
    marks["end"] = time.monotonic()
    robot.recording = False
    return robot.imu, robot.joints, marks


def ask(prompt, allowed=None):
    while True:
        try:
            answer = input(prompt).strip().lower()
        except EOFError:  # terminal ditutup = selesai, bukan crash di tengah percobaan
            raise KeyboardInterrupt

        if allowed is None or answer in allowed:
            return answer


def ask_label(session, n, candidate, prompt, context=None):
    """Label operator (Enter/n/j/h). "c" = tulis catatan bebas dulu untuk percobaan ini
    (boleh berkali-kali), lalu label ditanya lagi. context (percobaan yang sedang
    dilabeli: kind, params, metrics) = catatan itu langsung ditawarkan sebagai perubahan
    untuk percobaan BERIKUTNYA (offer_feedback)."""
    while True:
        answer = ask(prompt + "c = tulis catatan: ", {"", "n", "j", "h", "c"})
        if answer != "c":
            return answer
        note = feedback.prompt_note(session.dir, session.name, candidate or n, "saat training (percobaan %d)" % n,
                                    run=n, ask_verdict=False)
        if note is not None and context is not None:
            offer_feedback(session, dict(context, n=n, candidate=candidate), note)


def offer_feedback(session, context, note):
    """Catatan baru -> usulan perubahan (bukti lengkap di layar) -> operator memilih
    memakainya di percobaan BERIKUTNYA atau menyimpannya sebagai catatan saja."""
    state = session.state
    if context.get("candidate"):
        source = next(t for t in state["trials"] if t["n"] == context["candidate"])
    else:
        source = {"n": context["n"], "kind": context["kind"], "label": None, "metrics": context["metrics"],
                  "params": iteration.recorded(state, context["params"])}
    previous = iteration.find_step(state, context["step"]) if context.get("step") else None
    step = iteration.propose(state, session.dir, source, "saat training (percobaan %d)" % context["n"],
                             metrics=context["metrics"], extra_notes=[note], previous=previous)
    print()
    print(iteration.step_text(state, step, session.name))
    _, recost = iteration._trainer(state)
    if step["status"] == "tidak_berlaku":
        iteration.record(state, step, "tidak_berlaku")
        print("  -> catatan tersimpan; belum ada perubahan yang bisa diterapkan (lihat 'perlu' di atas).")
    elif ask("  Terapkan di percobaan BERIKUTNYA? Enter = ya, t = tidak (simpan sebagai catatan saja): ",
             {"", "t"}) == "t":
        iteration.record(state, step, "ditolak")
        print("  -> tidak diterapkan; catatan tetap tersimpan dan bisa diterapkan nanti dari menu hasil.")
    else:
        iteration.apply(state, step, recost)
        print("  -> %s" % ("MENUNGGU: percobaan berikutnya (#%d) memakai nilai ini" % step["next_trial"]
                           if step["status"] == "menunggu" else step.get("used", "")))
    session.save()
    print()


def evaluate(robot, session, kind, params, args, generation=None, candidate=None, step=None):
    """Satu percobaan lengkap + catatan. None = dilewati/gagal sistem (tidak dihitung).
    candidate = kandidat yang diulang (rerun), untuk menautkan catatan operator;
    step = id langkah feedback yang sedang dijalankan (kind "feedback")."""
    state = session.state
    n = len(state["trials"]) + 1
    shown = ", ".join("%s=%g" % kv for kv in iteration.recorded(state, params).items())
    print("\n=== percobaan %d (%s%s) ===\n  %s" % (n, kind, "" if generation is None else " gen %d" % generation, shown))
    if robot.voltage is not None and robot.voltage < args.min_voltage:
        ask("  !! baterai %.1f V < %.1f V. Ganti baterai, lalu Enter. " % (robot.voltage, args.min_voltage))
    answer = ask("  Robot digantung di garis start? Enter = mulai, s = lewati, q = selesai: ", {"", "s", "q"})
    if answer == "q":
        raise KeyboardInterrupt
    if answer == "s":
        return None
    try:
        imu, joints, marks = run_trial(robot, state, params)
    except StanceUnstable as unstable:
        robot.recording = False
        session.save_raw(n, robot.imu, robot.joints, {"stance_unstable": str(unstable)})
        return {"n": n, "kind": kind, "generation": generation, "time": now_iso(), "voltage": robot.voltage,
                "params": iteration.recorded(state, params), "distance_cm": None,
                "metrics": {"stance_unstable": True, "fell": False, "note": str(unstable)},
                "reference_id": state["reference_id"]}
    except RuntimeError as error:
        robot.recording = False
        robot.command("stop")
        print("  GAGAL SISTEM (bukan salah parameter, tidak dihitung): %s" % error)
        return None
    marks["page2_tilt"] = list(marks["page2_tilt"])
    metrics = core.compute_metrics(imu, joints, marks, params["period_time"], params["soft_start_time"])
    if not metrics.get("fell") and not metrics.get("valid"):
        print("  rekaman tidak sah (ayun pinggul %.2f deg, %d sampel) -- gait tidak benar-benar jalan? "
              "Tidak dihitung." % (metrics.get("hip_swing_deg", 0), metrics.get("walk_samples", 0)))
        return None
    distance = None
    if state["measure_distance"]:
        text = ask("  Jarak maju (cm), x = tidak terukur: ")
        distance = None if text == "x" else float(text.replace(",", "."))
    # Penilaian OPERATOR = kebenaran akhir: dengan robot digantung, tali menahan badan
    # sehingga detektor tidak selalu melihat jatuh (sesi coba: percobaan 4), dan
    # "hampir jatuh lalu diangkat" sama sekali tidak terlihat dari IMU.
    detected = "jatuh" if metrics.get("fell") else "normal"
    answer = ask_label(session, n, candidate, "  Menurut kamu? Enter = %s (deteksi), n = normal, j = jatuh/tertahan "
                       "tali, h = hampir jatuh, dipegang/diangkat, " % detected,
                       {"kind": kind, "params": params, "metrics": metrics, "step": step})
    label = LABEL_KEYS.get(answer, detected)
    session.save_raw(n, imu, joints, marks)
    trial = {"n": n, "kind": kind, "generation": generation, "time": now_iso(), "voltage": robot.voltage,
             "params": iteration.recorded(state, params), "metrics": metrics, "distance_cm": distance,
             "label": label, "reference_id": state["reference_id"]}
    if kind == "feedback":
        trial["loaded"] = ("walking_module membaca balik %d parameter sama dengan yang dikirim (apply_params) "
                           "sebelum melangkah" % len(params))
    return trial


def print_result(trial):
    m = trial["metrics"]
    if trial.get("label") == "hampir":
        print("  HAMPIR JATUH (label operator) -> skor %.2f" % trial["cost"])
    elif trial.get("label") == "jatuh" and not m.get("fell"):
        print("  JATUH (label operator, detektor tidak melihat) -> skor %.2f" % trial["cost"])
    elif m.get("stance_unstable"):
        print("  POSE BERDIRI TIDAK STABIL (%s) -> dikembalikan ke page 2 tanpa melangkah, skor %.2f"
              % (m.get("note"), trial["cost"]))
    elif m.get("fell") and trial.get("label") != "normal":
        print("  JATUH sesudah %.0f%% waktu jalan -> skor %.2f. Dirikan dan gantung lagi robotnya."
              % (100 * m["survived_frac"], trial["cost"]))
    else:
        print("  skor %.3f | arah %s | gyro %.1f deg/s | goyang %.2f deg | condong %+.1f deg | maks miring %.1f deg"
              % (trial["cost"], core.turn_text(m["yaw_drift_deg"]).upper(), m["gyro_rms_dps"], m["tilt_std_deg"],
                 m["lean_deg"], m["tilt_max_deg"]))


def baseline_loop(session, run_baseline, recost, is_usable, print_trial, failure_note, feedback_steps=None):
    """Jalankan preset/page ASLI sampai ada state["baseline_runs"] baseline yang sah --
    atau sampai operator mengambil alih titik awal lewat feedback (langkah yang
    dipertahankan): preset yang terus jatuh tidak diulang tanpa akhir."""
    state = session.state
    while len(baseline_trials(state, is_usable)) < state["baseline_runs"]:
        if feedback_steps and feedback_steps():
            if iteration.anchored(state):
                print("\n  baseline tidak diulang lagi: titik awal sudah diambil alih feedback -- CMA mulai dari sana "
                      "(skala skor: %s)" % ("baseline sah" if baseline_trials(state, is_usable) else
                                            "skala minimum, belum ada baseline sah"))
                return
            continue
        trial = run_baseline()
        if trial is None:
            continue
        state["trials"].append(trial)
        recost(state)
        session.save()
        print_trial(trial)
        if not is_usable(trial):
            print("  " + failure_note)


def cma_loop(robot, session, args, space, run_candidate, best, feedback_steps=None):
    """Generasi CMA-ES sampai anggaran habis. run_candidate(params, generation) -> trial
    yang sudah berskor, atau None (gagal sistem: kandidat yang sama dicoba lagi).
    feedback_steps() menjalankan iterasi feedback yang menunggu SEBELUM kandidat
    berikutnya; sesudahnya CMA dibangun ulang dari state (titik tengahnya bisa pindah)."""
    state = session.state
    es = None
    while sum(1 for t in state["trials"] if t["kind"] == "cma") < state["budget"]:
        if feedback_steps and feedback_steps():
            es = None
        if es is None:
            es = core.CMAES(space.to_unit(state["start_params"]), state["sigma0"], popsize=state["popsize"],
                            state=state["cma"])
        start = iteration.effective_start(state)
        if not state["pending"]:
            state["pending"] = [x.tolist() for x in es.ask()]
            state["evaluated"] = []
            session.save()
        unit = state["pending"][len(state["evaluated"])]
        trial = run_candidate(core.candidate_params(start, space, unit), es.generation)
        if trial is None:
            continue
        state["trials"].append(trial)
        state["evaluated"].append(core.bounded_cost(unit, trial["cost"]))
        if len(state["evaluated"]) == len(state["pending"]):
            es.tell([core.np.array(x) for x in state["pending"]], state["evaluated"])
            state["cma"], state["pending"], state["evaluated"] = es.state(), [], []
            top = best(state)
            print("\n--- generasi %d selesai; terbaik sejauh ini: percobaan %d skor %.3f ---"
                  % (es.generation, top["n"], top["cost"]) if top else "")
        session.save()
        if args.cooldown_sec > 0:
            print("  jeda pendinginan servo %.0f s ..." % args.cooldown_sec)
            robot.spin_for(args.cooldown_sec)
    if feedback_steps:
        feedback_steps()           # catatan di percobaan terakhir tetap dijalankan
    print("\nAnggaran %d percobaan habis." % state["budget"])


def run_feedback_steps(session, run_feedback, recost):
    """Iterasi feedback yang MENUNGGU = percobaan berikutnya (jenis "feedback"), satu
    percobaan per langkah: bukti nilai, perbandingan, lalu operator mempertahankan atau
    membatalkan. Mengembalikan True kalau ada yang dijalankan."""
    state = session.state
    ran = False
    while True:
        step = iteration.pending(state)
        if step is None:
            return ran
        print("\n>>> iterasi feedback %s dari percobaan #%s: %s" % (
            step["id"], step["source"]["trial"], "; ".join(
                "%s %+g" % (c["key"], c["delta"]) for c in step["changes"])))
        trial = run_feedback(dict(step["after"]), step["id"])
        if trial is None:
            continue           # dilewati / gagal sistem: langkah tetap menunggu
        trial["feedback_step"] = step["id"]
        state["trials"].append(trial)
        recost(state)
        iteration.finish(state, step, trial)
        session.save()
        print()
        print(iteration.result_text(state, step))
        newer = iteration.pending(state)
        if newer is not None:
            keep = True
            print("  (dipertahankan: iterasi %s berikutnya dibangun di atas percobaan ini)" % newer["id"])
        else:
            keep = ask("  Pertahankan perubahan ini (CMA lanjut dari titik ini)? Enter = ya, b = batalkan: ",
                       {"", "b"}) != "b"
        iteration.settle(state, step, keep, recost)
        session.save()
        print("  -> %s\n" % step["used"])
        ran = True


def tune(robot, session, args):
    state = session.state

    def run_candidate(params, generation):
        if not core.leg_reachable(params):
            trial = {"n": len(state["trials"]) + 1, "kind": "cma", "generation": generation, "time": now_iso(),
                     "params": {k: params[k] for k in state["params"]}, "metrics": None,
                     "cost": core.COST_UNREACHABLE, "note": "kaki tidak sampai (IK), tidak dijalankan"}
            print("\n=== percobaan %d: kaki tidak sampai (IK) -- dilewati tanpa menggerakkan robot ===" % trial["n"])
            return trial
        trial = evaluate(robot, session, "cma", params, args, generation)
        if trial is not None:
            trial["cost"] = score(state, trial)
            print_result(trial)
        return trial

    steps = feedback_runner(robot, session, args)
    baseline_loop(session, lambda: evaluate(robot, session, "baseline", state["start_params"], args),
                  recost_all, usable, print_result,
                  "%s sendiri jatuh/hampir jatuh/condong -- tidak dipakai sebagai baseline. Periksa gantungan, "
                  "baterai, dan page 2 dulu -- atau tulis 'c' (mis. 'jatuh ke belakang') supaya percobaan "
                  "berikutnya memakai nilai yang diperbaiki." % state["start_name"], steps)
    cma_loop(robot, session, args, space_of(state), run_candidate, best_trial, steps)
    report(state)


def feedback_runner(robot, session, args):
    """feedback_steps() untuk loop pelatih walking."""
    state = session.state

    def run(params, step_id):
        trial = evaluate(robot, session, "feedback", params, args, step=step_id)
        if trial is not None:
            trial["cost"] = score(state, trial)
            print_result(trial)
        return trial
    return lambda: run_feedback_steps(session, run, recost_all)


def rerun_target(state, which, best):
    """Kandidat yang diulang: percobaan `which` (baseline/cma), atau yang terbaik."""
    if which is None:
        target = best(state)
        if not target:
            raise SystemExit("belum ada percobaan yang sah")
        return target
    target = next((t for t in state["trials"] if t["n"] == which), None)
    if target is None or target["kind"] not in CANDIDATE_KINDS or target.get("metrics") is None:
        raise SystemExit("percobaan %s bukan kandidat yang bisa diulang (harus baseline/cma/feedback yang pernah "
                         "dijalankan)" % which)
    return target


def rerun_best(robot, session, args):
    state = session.state
    best = rerun_target(state, args.trial, best_trial)
    print("mengulang kandidat percobaan %d (%s) %d kali" % (best["n"], best["kind"], args.rerun_best))
    params = dict(state["start_params"], **best["params"])
    steps, waiting = feedback_runner(robot, session, args), iteration.pending(state)
    for _ in range(args.rerun_best):
        trial = evaluate(robot, session, "rerun", params, args, candidate=best["n"])
        if trial is None:
            continue
        trial["cost"] = score(state, trial)
        trial["rerun_of"] = best["n"]
        state["trials"].append(trial)
        session.save()
        print_result(trial)
        if iteration.pending(state) not in (None, waiting):
            steps()            # catatan "c" di ulangan ini: perubahannya langsung dicoba


def studio_request(method, body=None):
    """Server penyimpanan versi Studio (plugin Vite, server/studioStorePlugin.js) -- satu-
    satunya penulis walking_versions.json; tiap tulis dicadangkan dan dibaca ulang di sana."""
    req = urllib.request.Request(STUDIO_URL, method=method, data=None if body is None else json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json", "X-Bascorro-Studio": "1"})
    try:
        with urllib.request.urlopen(req, timeout=8) as response:
            return json.loads(response.read())
    except urllib.error.HTTPError as error:
        try:
            message = json.loads(error.read()).get("error")
        except ValueError:
            message = None
        raise StudioRefused(message or "HTTP %d" % error.code)


class StudioRefused(Exception):
    """Server Studio hidup tapi menolak operasinya (nama bentrok, field tidak sah, ...)."""


METRIC_KEYS = ["gyro_rms_dps", "tilt_std_deg", "tilt_max_deg", "lean_deg", "yaw_drift_deg", "hip_swing_deg"]


def follow_reference(state, reference):
    """Pindahkan sesi ke acuan page 2 baru TANPA kehilangan apa pun.

    Sama dengan migrasi versi Studio (walking_ready_fk.py --write): offset pose
    digeser sebesar perubahan acuan, jadi tiap kandidat tetap "page 2 + selisih
    yang sama". Batas ruang pencarian ikut bergeser, sehingga koordinat CMA
    (0..1) tidak berubah dan pencarian berlanjut dari titik yang sama.
    Percobaan lama tetap dicatat dengan acuan tempat ia benar-benar dijalankan."""
    old_id, old = state["reference_id"], state["reference_offsets"]
    delta = {k: reference["offsets"][k] - old[k] for k in core.BIAS_FIELDS}

    def shift(params):
        for key, d in delta.items():
            if key in params and d:
                params[key] = round(params[key] + d, 10)

    shift(state["start_params"])
    for step in iteration.steps(state):
        if step["status"] in ("menunggu", "usulan"):
            shift(step["base"])
            shift(step["after"])
    for i, name in enumerate(state["params"]):
        if name in delta:
            state["lo"][i] += delta[name]
            state["hi"][i] += delta[name]
    for trial in state["trials"]:
        trial.setdefault("reference_id", old_id)
        shift(trial["params"])
    state.setdefault("reference_history", []).append(
        {"from": old_id, "to": reference["id"], "at": now_iso(), "delta": delta})
    state["reference_id"], state["reference_offsets"] = reference["id"], reference["offsets"]
    print("sesi dipindah ke acuan %s (dari %s): %s" % (reference["id"], old_id, ", ".join(
        "%s %+.4f" % (k, d) for k, d in delta.items() if d)))
    print("  catatan: baseline & percobaan lama dijalankan dengan page 2 LAMA; skala skornya tetap dipakai.")


def apply_labels(state, spec, recost=None):
    """--label "2=h,4=j": penilaian operator untuk percobaan yang sudah tercatat."""
    for part in spec.split(","):
        n, _, key = part.strip().partition("=")
        key = key.strip().lower()
        label = LABEL_KEYS.get(key, key if key in core.LABELS else None)
        trial = next((t for t in state["trials"] if str(t["n"]) == n.strip()), None)
        if label is None or trial is None:
            raise SystemExit("label %r tidak sah (pakai N=n|j|h, mis. 2=h,4=j)" % part)
        trial["label"] = label
    (recost or recost_all)(state)


def rebound_session(state):
    """Batas parameter di BOUNDS berubah -> petakan ulang keadaan CMA ke batas baru.

    Koordinat CMA 0..1 relatif terhadap batas, jadi kalau batas melebar titik yang
    sama punya koordinat lain. Pemetaan afin u' = r*u + b per parameter membuat
    rata-rata, kovarians, jalur evolusi, dan kandidat yang belum dijalankan tetap
    di NILAI ASLI yang sama -- pencarian berlanjut, bukan mulai dari nol.
    ps (ruang terputihkan) dan sigma tidak berubah oleh pemetaan afin."""
    np = core.np
    space = core.Space(state["params"], state["start_params"], state["reference_offsets"])
    old_lo, old_hi = np.array(state["lo"]), np.array(state["hi"])
    if np.allclose(old_lo, space.lo) and np.allclose(old_hi, space.hi):
        return False
    r = (old_hi - old_lo) / (space.hi - space.lo)
    b = (old_lo - space.lo) / (space.hi - space.lo)
    cma = state["cma"]
    cma["mean"] = (np.array(cma["mean"]) * r + b).tolist()
    cma["C"] = (np.array(cma["C"]) * np.outer(r, r)).tolist()
    cma["pc"] = (np.array(cma["pc"]) * r).tolist()
    state["pending"] = [(np.array(u) * r + b).tolist() for u in state["pending"]]
    for i, name in enumerate(state["params"]):
        if not (np.isclose(old_lo[i], space.lo[i]) and np.isclose(old_hi[i], space.hi[i])):
            print("  batas %s: %g..%g -> %g..%g" % (name, old_lo[i], old_hi[i], space.lo[i], space.hi[i]))
    state["lo"], state["hi"] = space.lo.tolist(), space.hi.tolist()
    return True


def rescore(session):
    """Hitung ulang metrik SEMUA percobaan dari rekaman mentah dengan rumus sekarang,
    termasuk detektor jatuh. Sesi lama tidak dibuang hanya karena rumusnya membaik."""
    state = session.state
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    backup = os.path.join(session.dir, "state.sebelum-rescore-%s.json" % stamp)
    with open(backup, "w", encoding="utf-8") as handle:
        json.dump(state, handle, indent=1)
    print("cadangan sesi: %s" % os.path.relpath(backup, HERE))
    rebound_session(state)
    for trial in state["trials"]:
        path = os.path.join(session.dir, "raw", "trial_%03d.json.gz" % trial["n"])
        if trial.get("metrics") is None or not os.path.exists(path):
            continue
        with gzip.open(path, "rt", encoding="utf-8") as handle:
            raw = json.load(handle)
        marks = raw["marks"]
        if "stance_unstable" in marks:
            continue
        marks.setdefault("planned_stop", marks["start"] + state["walk_sec"])
        marks["fell_at"] = core.detect_fall(raw["imu"], marks)
        params = dict(state["start_params"], **trial["params"])
        trial["metrics"] = core.compute_metrics(raw["imu"], raw["joints"], marks,
                                                params["period_time"], params["soft_start_time"])
    state["metricsVersion"] = core.METRICS_VERSION
    recost_all(state)


def export_dataset(path):
    """Semua percobaan dari semua sesi -> satu CSV: 29 parameter lengkap, metrik,
    label operator, skor. Bekal untuk tuning berikutnya (dan model/RL nanti)."""
    rows, param_keys = [], None
    for name in sorted(os.listdir(SESSIONS_DIR)):
        state_path = os.path.join(SESSIONS_DIR, name, "state.json")
        if not os.path.isfile(state_path):
            continue
        with open(state_path, encoding="utf-8") as handle:
            state = json.load(handle)
        param_keys = param_keys or list(state["start_params"])
        for t in state["trials"]:
            m = t.get("metrics") or {}
            full = dict(state["start_params"], **t["params"])
            row = {"sesi": name, "n": t["n"], "jenis": t["kind"], "generasi": t.get("generation"),
                   "waktu": t.get("time"), "acuan": t.get("reference_id", state["reference_id"]),
                   "versi_metrik": state.get("metricsVersion", 1),
                   "label": t.get("label", ""), "skor": round(t["cost"], 4), "terdeteksi_jatuh": m.get("fell"),
                   "pose_tak_stabil": m.get("stance_unstable"), "bertahan": m.get("survived_frac"),
                   "volt": t.get("voltage"), "jarak_cm": t.get("distance_cm"),
                   "arah": core.turn_direction(m.get("yaw_drift_deg")), "dituning": " ".join(state["params"])}
            row.update({k: full.get(k) for k in param_keys})
            row.update({k: m.get(k) for k in METRIC_KEYS})
            rows.append(row)
    if not rows:
        raise SystemExit("belum ada sesi di %s" % SESSIONS_DIR)
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print("%d percobaan dari %d sesi -> %s" % (len(rows), len({r["sesi"] for r in rows}), path))


def warm_start_point(names, space, start, reference_id):
    """Titik awal CMA = kandidat terbaik sesi sebelumnya (acuan page 2 sama, sudah
    di-rescore), asal memang lebih baik dari baseline sesi itu sendiri."""
    best = None
    for name in names:
        name, _, which = name.strip().partition(":")
        previous = Session(name)
        if not previous.exists():
            raise SystemExit("sesi --warm-start %r tidak ada" % name)
        state = previous.load()
        if which:
            # Dipilih operator (menu: "Training lanjutan dari hasil ini"): dipakai apa adanya,
            # asal acuan page 2-nya sama -- angka init pose berarti acuan itu.
            candidate = next((t for t in state["trials"] if str(t["n"]) == which), None)
            if candidate is None:
                raise SystemExit("--warm-start %s:%s: percobaan itu tidak ada" % (name, which))
            if state["reference_id"] != reference_id:
                raise SystemExit("--warm-start %s:%s: sesi itu memakai acuan page 2 %s, sekarang %s -- angkanya "
                                 "tidak berlaku lagi" % (name, which, state["reference_id"], reference_id))
            cost, _ = candidate_cost(state, candidate)
            print("  --warm-start: mulai dari sesi %s percobaan %d (skor %.3f, dipilih operator)"
                  % (name, candidate["n"], cost))
            unit = core.np.clip(space.to_unit(dict(start, **candidate["params"])), 0.0, 1.0)
            return unit, {"session": name, "n": candidate["n"], "cost": cost, "chosen": True}
        if state["reference_id"] != reference_id:
            print("  --warm-start %s dilewati: acuan page 2 sudah berubah sejak sesi itu" % name)
            continue
        if state.get("metricsVersion") != core.METRICS_VERSION:
            raise SystemExit("sesi %r masih rumus versi %s -- jalankan dulu --session %s --rescore"
                             % (name, state.get("metricsVersion", 1), name))
        candidate = best_trial(state)
        base = [t["cost"] for t in baseline_trials(state)]
        if candidate is None or not base:
            continue
        cost, _ = candidate_cost(state, candidate)
        if cost < sum(base) / len(base) and (best is None or cost < best[0]):
            best = (cost, name, candidate)
    if best is None:
        print("  --warm-start: tidak ada kandidat yang lebih baik dari baseline; mulai dari %r" % "preset awal")
        return space.to_unit(start), None
    cost, name, candidate = best
    print("  --warm-start: mulai dari sesi %s percobaan %d (skor %.3f)" % (name, candidate["n"], cost))
    unit = core.np.clip(space.to_unit(dict(start, **candidate["params"])), 0.0, 1.0)
    return unit, {"session": name, "n": candidate["n"], "cost": cost}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--session")
    ap.add_argument("--start", default="JALAN BARU", help="versi Studio awal (nama PERSIS atau id)")
    ap.add_argument("--motion", choices=core.MOTIONS,
                    help="jenis gerak preset (bawaan: dibaca dari x/y/angle_move). Menentukan parameter yang "
                    "dikunci dan sasaran belok")
    ap.add_argument("--params", help="daftar parameter dipisah koma (bawaan: %s)" % ",".join(core.DEFAULT_PARAMS))
    ap.add_argument("--trials", type=int, help="anggaran TOTAL percobaan CMA (bawaan %d); pada sesi lama: "
                    "menaikkan anggarannya" % DEFAULT_BUDGET)
    ap.add_argument("--popsize", type=int, default=8)
    ap.add_argument("--sigma", type=float, default=0.15, help="langkah awal, pecahan rentang tiap parameter")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--walk-sec", type=float, help="lama jalan per percobaan (bawaan 12 s; geser/putar 8 s)")
    ap.add_argument("--baseline-runs", type=int, default=2)
    ap.add_argument("--cooldown-sec", type=float, default=15.0)
    ap.add_argument("--min-voltage", type=float, default=11.1)
    ap.add_argument("--measure-distance", action="store_true")
    ap.add_argument("--report", action="store_true")
    ap.add_argument("--rerun-best", type=int, default=0, metavar="N")
    ap.add_argument("--trial", type=int, metavar="N", help="dengan --rerun-best: ulangi percobaan N, bukan terbaik")
    ap.add_argument("--save-best", metavar="NAMA")
    ap.add_argument("--label", metavar="N=n|j|h,...", help="penilaian operator untuk percobaan tercatat")
    ap.add_argument("--rescore", action="store_true", help="hitung ulang metrik sesi dari rekaman mentah")
    ap.add_argument("--export", nargs="?", const=os.path.join(SESSIONS_DIR, "dataset.csv"), metavar="CSV",
                    help="semua percobaan semua sesi jadi satu CSV (bawaan data/autotune/dataset.csv)")
    ap.add_argument("--warm-start", metavar="SESI[:N][,SESI]", help="sesi BARU mulai dari terbaik sesi lain "
                    "(SESI:N = percobaan N sesi itu)")
    ap.add_argument("--weights", metavar="k=v,...", help="sesi BARU: bobot skor (metrik: %s)" % ",".join(core.WEIGHTS))
    ap.add_argument("--feedback-run", action="store_true",
                    help="jalankan iterasi feedback yang menunggu (training.py feedback-apply), lalu berhenti")
    ap.add_argument("--follow-reference", action="store_true",
                    help="lanjutkan sesi walau page 2 sudah di-teach ulang (offset pose digeser ke acuan baru)")
    args = ap.parse_args()

    if args.export:
        export_dataset(args.export)
        return 0
    if not args.session:
        ap.error("--session wajib (kecuali --export)")
    if args.trial is not None and not args.rerun_best:
        ap.error("--trial hanya bersama --rerun-best N")

    session = Session(args.session)
    if session.exists() and (args.weights or args.warm_start):
        ap.error("--weights/--warm-start hanya untuk sesi BARU (sesi %r sudah ada)" % args.session)
    if session.exists():
        state = session.load()
        with open(REFERENCE_JSON, encoding="utf-8") as handle:
            current = json.load(handle)["id"]
        if args.label or args.rescore:
            if args.rescore:
                rescore(session)
            if args.label:
                apply_labels(session.state, args.label)
            session.save()
            report(session.state)
            return 0
        if state.get("metricsVersion") != core.METRICS_VERSION and not args.report:
            raise SystemExit("sesi %r memakai rumus skor versi %s, alat ini versi %d -- skornya tidak "
                             "sebanding. Hitung ulang dulu: --session %s --rescore"
                             % (args.session, state.get("metricsVersion", 1), core.METRICS_VERSION, args.session))
        if current != state["reference_id"] and args.follow_reference:
            with open(REFERENCE_JSON, encoding="utf-8") as handle:
                follow_reference(state, json.load(handle))
            session.save()
        elif current != state["reference_id"] and not args.report:
            raise SystemExit("page 2 / acuan berubah sejak sesi %r dimulai (%s -> %s). Mulai sesi BARU, atau "
                             "lanjutkan sesi ini dengan --follow-reference (offset pose digeser ke acuan baru)"
                             % (args.session, state["reference_id"], current))
        print("melanjutkan sesi %r: %d percobaan tercatat" % (args.session, len(state["trials"])))
        if args.trials is not None and args.trials != state["budget"] and not (args.report or args.save_best):
            print("anggaran percobaan CMA %d -> %d" % (state["budget"], args.trials))
            state["budget"] = args.trials
            session.save()
    elif args.report or args.save_best or args.rerun_best or args.label or args.rescore or args.feedback_run:
        raise SystemExit("sesi %r belum ada" % args.session)
    else:
        session.state = new_state(args)
        session.save()
        print("sesi baru %r dari %r (%s): %s" % (args.session, session.state["start_name"],
                                                 session.state["motion_text"], ", ".join(session.state["params"])))

    if args.report:
        report(session.state)
        return 0
    if args.save_best:
        # Jalur yang sama dengan menu Training -> "Simpan langsung ke robot" -> preset baru:
        # diperiksa, dibaca balik, dan tercatat di <sesi>/hasil.json.
        import training_deploy as deploy  # noqa: E402  (modul itu memakai modul ini)
        import training_results as results  # noqa: E402
        opened = results.open_session(args.session)
        plan, entry = deploy.deploy(opened, results.find_trial(opened, "best"), "studio-new", args.save_best)
        print(deploy.plan_text(plan))
        print(deploy.entry_text(entry))
        return 0 if entry["status"] == "berhasil" else 1

    if args.feedback_run and iteration.pending(session.state) is None:
        raise SystemExit("sesi %r tidak punya iterasi feedback yang menunggu (scripts/training.py feedback-apply)"
                         % args.session)
    done = sum(1 for t in session.state["trials"] if t["kind"] == "cma")
    if not (args.rerun_best or args.feedback_run) and done >= session.state["budget"]:
        more = done + 2 * session.state["popsize"]
        raise SystemExit("anggaran sesi %r sudah habis (%d/%d percobaan CMA). Lanjutkan dengan anggaran lebih "
                         "besar, mis. --session %s --trials %d (= 2 generasi lagi)"
                         % (args.session, done, session.state["budget"], args.session, more))

    print(__doc__.split("KESELAMATAN -- WAJIB")[1].split("PAKAI")[0])
    # Iterasi feedback dari menu: operator baru saja menyetujui perubahannya di menu, dan
    # tiap percobaan tetap menunggu Enter "Robot digantung di garis start?".
    if not args.feedback_run and ask("Ketik YA kalau robot sudah digantung dan D60/Studio walking tidak "
                                     "dipakai: ").upper() != "YA":
        return 1
    session.lock()
    robot = None
    try:
        robot = Robot()
        robot.spin_for(1.5)  # penemuan DDS
        if args.feedback_run:
            feedback_runner(robot, session, args)()
        elif args.rerun_best:
            rerun_best(robot, session, args)
        else:
            tune(robot, session, args)
    except KeyboardInterrupt:
        print("\ndihentikan -- 'stop' dikirim, sesi tersimpan. Lanjutkan dengan --session %s." % args.session)
    finally:
        session.save()
        session.unlock()
        if robot is not None:
            robot.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
