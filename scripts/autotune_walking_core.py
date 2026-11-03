"""Bagian MURNI auto-tune walking (tanpa ROS): ruang parameter, CMA-ES, metrik, skor.

Dipakai scripts/autotune_walking.py; diuji scripts/test_autotune_walking_core.py
tanpa robot. Lihat docstring autotune_walking.py untuk alur dan keselamatan.
"""

import math

import numpy as np

from walking_ready_fk import BIAS_FIELDS, reachable

# ---- ruang parameter ---------------------------------------------------------
# Fase 1 = JALAN LURUS MAJU. Langkah maju (x_move_amplitude) SENGAJA tidak ikut:
# skornya cuma kestabilan, jadi kalau langkah boleh dikecilkan, "jalan di tempat"
# otomatis jadi pemenang. Yang dicari adalah cara berjalan yang stabil DENGAN
# langkah JALAN BARU.
DEFAULT_PARAMS = [
    "period_time", "dsp_ratio", "z_move_amplitude", "y_swap_amplitude",
    "hip_pitch_offset", "init_x_offset", "init_pitch_offset", "init_y_offset",
    "angle_move_amplitude",
]

# (bawah, atas) MUTLAK; "rel" = terhadap acuan page 2 (offset bias), "start" =
# terhadap nilai preset awal. Batas sengaja sempit: ini penyetelan di sekitar
# JALAN BARU, bukan pencarian bebas yang bisa membanting robot.
BOUNDS = {
    "period_time": ("start", -0.15, +0.20),         # s
    "dsp_ratio": ("abs", 0.05, 0.45),
    "step_fb_ratio": ("abs", 0.10, 0.40),
    "z_move_amplitude": ("abs", 0.025, 0.07),       # m, tinggi angkat kaki
    "y_swap_amplitude": ("abs", 0.0, 0.035),        # m, ayun badan kiri-kanan
    "z_swap_amplitude": ("abs", 0.0, 0.015),        # m
    "pelvis_offset": ("abs", 0.0, 0.15),            # rad
    "arm_swing_gain": ("abs", 0.0, 1.5),
    # Tiga offset pose ini menggeser pose BERDIRI dari page 2. Sesi "coba" 23 Okt
    # 2026: kandidat dengan init_pitch -5,7 deg / hip_pitch +3,5 deg dari page 2
    # terguling sebelum sempat melangkah, jadi rentangnya dipersempit.
    "hip_pitch_offset": ("rel", -0.06, +0.06),      # rad dari acuan page 2
    "init_x_offset": ("rel", -0.012, +0.012),       # m dari acuan page 2
    "init_pitch_offset": ("rel", -0.05, +0.05),     # rad dari acuan page 2
    "init_y_offset": ("abs", 0.0, 0.04),            # m, lebar kaki tambahan
    # Koreksi belok. +-0,02 tidak cukup: sesi jb1 sesudah servo 12 diperbaiki belok
    # kanan ~36 deg per 8 s. +-0,05 (2,9 deg) masih jauh di bawah preset putar operator
    # (0,055-0,2). Tanda: negatif = belok KANAN (REP-103, terlihat di jb1).
    "angle_move_amplitude": ("abs", -0.05, 0.05),   # rad
    # Amplitudo naik/turun bertahap saat mulai & berhenti (feedback "awalan terlalu kuat").
    # Di bawah ~0,29 s tidak ada yang bertahap sama sekali (updateMovementParam hanya di batas fase).
    "soft_start_time": ("abs", 0.3, 2.0),          # s
}


class Space:
    """Peta [0,1]^n <-> nilai parameter asli. bounds = tabel batas (bawaan BOUNDS walking;
    training page Action Editor memakai tabelnya sendiri dengan bentuk yang sama)."""

    def __init__(self, names, start, reference, overrides=None, bounds=None):
        bounds = BOUNDS if bounds is None else bounds
        unknown = [n for n in names if n not in bounds]
        if unknown:
            raise ValueError("parameter tanpa batas: " + ", ".join(unknown))
        self.names = list(names)
        self.lo, self.hi = [], []
        for name in self.names:
            kind, a, b = bounds[name]
            if overrides and name in overrides:
                kind, a, b = ("abs",) + tuple(overrides[name])
            base = {"abs": 0.0, "rel": reference.get(name, 0.0), "start": start[name]}[kind]
            lo, hi = base + a, base + b
            # Titik awal harus selalu terjangkau optimizer.
            lo, hi = min(lo, start[name]), max(hi, start[name])
            if not hi > lo:
                raise ValueError("batas %s kosong" % name)
            self.lo.append(lo)
            self.hi.append(hi)
        self.lo, self.hi = np.array(self.lo), np.array(self.hi)

    def to_params(self, unit):
        value = self.lo + np.clip(np.asarray(unit, dtype=float), 0.0, 1.0) * (self.hi - self.lo)
        return {n: round(float(v), 5) for n, v in zip(self.names, value)}

    def to_unit(self, params):
        return (np.array([params[n] for n in self.names], dtype=float) - self.lo) / (self.hi - self.lo)


# ---- jenis gerak preset ---------------------------------------------------------
# Training bisa dimulai dari preset Studio APA SAJA, dan namanya tidak bisa dipercaya
# (keempat preset menyamping operator bertanda kebalikan dari namanya), jadi jenis
# geraknya dibaca dari parameter. Konvensi REP-103: y_move dan angle_move positif = KIRI.
TURN_COMMAND_RAD = 0.05    # |angle_move| di atas rentang koreksi belok = preset memang memutar
SIDE_COMMAND_M = 0.005     # |y_move| di atas ini = preset menyamping
STEP_COMMAND_M = 0.002     # |x_move| di atas ini = preset melangkah maju/mundur
MOTIONS = ("maju", "mundur", "geser", "putar", "di_tempat")


def walking_motion(params):
    """Jenis gerak dari angka preset: (jenis, keterangan)."""
    x, y, a = params["x_move_amplitude"], params["y_move_amplitude"], params["angle_move_amplitude"]
    if abs(a) > TURN_COMMAND_RAD:
        return "putar", "putar %s (angle_move %+.3f rad)" % ("kiri" if a > 0 else "kanan", a)
    if abs(y) >= SIDE_COMMAND_M:
        return "geser", "geser %s (y_move %+.3f m)" % ("kiri" if y > 0 else "kanan", y)
    if abs(x) >= STEP_COMMAND_M:
        return ("maju" if x > 0 else "mundur"), "%s (x_move %+.4f m)" % ("maju" if x > 0 else "mundur", x)
    return "di_tempat", "jalan di tempat (x, y, angle ~0)"


def motion_params(motion):
    """Parameter yang dicari. Perintah gerak (x/y_move, dan angle_move untuk preset
    putar) DIKUNCI: skornya kestabilan, jadi kalau perintah boleh dikecilkan, "diam"
    yang menang. Untuk preset lain angle_move tetap ikut sebagai koreksi belok."""
    if motion == "putar":
        return [p for p in DEFAULT_PARAMS if p != "angle_move_amplitude"]
    return list(DEFAULT_PARAMS)


def motion_keeps_turn(motion):
    """Preset putar: sasaran belok = belok preset itu sendiri (diukur di baseline), bukan 0."""
    return motion == "putar"


def candidate_params(start, space, unit):
    """Preset lengkap 29 field: preset awal + nilai percobaan."""
    params = dict(start)
    params.update(space.to_params(unit))
    return params


def leg_reachable(params):
    trial = {k: params[k] for k in BIAS_FIELDS}
    trial.update(init_y_offset=params["init_y_offset"], x_move_amplitude=params["x_move_amplitude"])
    return reachable(trial)


# ---- CMA-ES -------------------------------------------------------------------
# Versi ringkas purecma (N. Hansen), cukup untuk n <= ~15 dan anggaran ratusan
# evaluasi. Keadaan lengkapnya bisa disimpan ke JSON supaya sesi bisa dilanjut
# sesudah ganti baterai atau servo didinginkan.

class CMAES:
    def __init__(self, x0, sigma0, popsize=None, seed=1, state=None):
        self.n = n = len(x0)
        self.lam = popsize or 4 + int(3 * math.log(n))
        self.mu = self.lam // 2
        w = np.log(self.mu + 0.5) - np.log(np.arange(1, self.mu + 1))
        self.weights = w / w.sum()
        self.mueff = 1.0 / np.sum(self.weights ** 2)
        self.cc = (4 + self.mueff / n) / (n + 4 + 2 * self.mueff / n)
        self.cs = (self.mueff + 2) / (n + self.mueff + 5)
        self.c1 = 2 / ((n + 1.3) ** 2 + self.mueff)
        self.cmu = min(1 - self.c1, 2 * (self.mueff - 2 + 1 / self.mueff) / ((n + 2) ** 2 + self.mueff))
        self.damps = 1 + 2 * max(0, math.sqrt((self.mueff - 1) / (n + 1)) - 1) + self.cs
        self.chin = math.sqrt(n) * (1 - 1 / (4 * n) + 1 / (21 * n * n))
        self.rng = np.random.default_rng(seed)
        if state:
            self.mean = np.array(state["mean"])
            self.sigma = float(state["sigma"])
            self.C = np.array(state["C"])
            self.pc = np.array(state["pc"])
            self.ps = np.array(state["ps"])
            self.generation = int(state["generation"])
            self.rng.bit_generator.state = state["rng"]
        else:
            self.mean = np.array(x0, dtype=float)
            self.sigma = float(sigma0)
            self.C = np.eye(n)
            self.pc = np.zeros(n)
            self.ps = np.zeros(n)
            self.generation = 0

    def ask(self):
        values, vectors = np.linalg.eigh((self.C + self.C.T) / 2)
        D = np.sqrt(np.maximum(values, 1e-20))
        z = self.rng.standard_normal((self.lam, self.n))
        return [self.mean + self.sigma * (vectors @ (D * zi)) for zi in z]

    def tell(self, xs, fs):
        order = np.argsort(fs)
        xs = np.array(xs)[order]
        old = self.mean
        self.mean = self.weights @ xs[: self.mu]
        values, vectors = np.linalg.eigh((self.C + self.C.T) / 2)
        inv_sqrt = vectors @ np.diag(1 / np.sqrt(np.maximum(values, 1e-20))) @ vectors.T
        step = (self.mean - old) / self.sigma
        self.ps = (1 - self.cs) * self.ps + math.sqrt(self.cs * (2 - self.cs) * self.mueff) * (inv_sqrt @ step)
        self.generation += 1
        hsig = (np.linalg.norm(self.ps) / math.sqrt(1 - (1 - self.cs) ** (2 * self.generation))
                / self.chin) < 1.4 + 2 / (self.n + 1)
        self.pc = (1 - self.cc) * self.pc + hsig * math.sqrt(self.cc * (2 - self.cc) * self.mueff) * step
        artmp = (xs[: self.mu] - old) / self.sigma
        self.C = ((1 - self.c1 - self.cmu) * self.C
                  + self.c1 * (np.outer(self.pc, self.pc) + (1 - hsig) * self.cc * (2 - self.cc) * self.C)
                  + self.cmu * (artmp.T @ np.diag(self.weights) @ artmp))
        self.sigma *= math.exp((self.cs / self.damps) * (np.linalg.norm(self.ps) / self.chin - 1))

    def state(self):
        return {"mean": self.mean.tolist(), "sigma": self.sigma, "C": self.C.tolist(),
                "pc": self.pc.tolist(), "ps": self.ps.tolist(), "generation": self.generation,
                "rng": self.rng.bit_generator.state}


def bounded_cost(unit, cost):
    """Sampel CMA di luar [0,1] dievaluasi di titik terjepit + penalti jaraknya."""
    unit = np.asarray(unit, dtype=float)
    return cost + 10.0 * float(np.sum((unit - np.clip(unit, 0.0, 1.0)) ** 2))


# ---- metrik dari rekaman satu percobaan ---------------------------------------

# Roll/pitch dari OpenCR dihitung dari AKSELEROMETER saja (open_cr_module::publishIMU),
# jadi hentakan kaki ikut terbaca sebagai "miring" (roll mentah jalan normal std 16-18
# deg, puncak 48 deg). Tetap saja penapis TIPIS yang benar untuk jatuh, dan itu
# DIUKUR, bukan ditebak: sesi coba 23 Okt dilabeli operator (4, 5, 6 jatuh; 2 hampir
# jatuh lalu diangkat; 1, 3 normal). Robot DIGANTUNG, jadi tali menahan badan sebelum
# rebah: percobaan 4 (jatuh sungguhan) cuma mencapai 18,6 deg pada tapisan 0,25 s
# dan TERLEWAT, sedangkan tapisan ~16 ms (= EMA alpha 0,4 per sampel pada 125 Hz,
# fall_detector.h) + 45 deg menangkap ketiganya tanpa satu pun alarm palsu saat
# berjalan. Label operator tetap kebenaran akhirnya (lihat cost(label=...)).
FALL_LIMIT_DEG = 45.0
FALL_TAU_S = 0.016
METRIC_TAU_S = 0.15       # kemiringan untuk skor: buang hentakan, sisakan ayunan langkah
# Pose berdiri kandidat condong sejauh ini dari page 2 = dibatalkan sebelum melangkah.
# Sesi coba: 5 (jatuh) 18 deg, 6 (jatuh) 8 deg, 3 (normal) 2,7 deg.
STANCE_LIMIT_DEG = 7.0


def quat_to_rpy_deg(x, y, z, w):
    """robotis_framework::convertQuaternionToRPY (via matriks rotasi), derajat."""
    r21 = 2 * (y * z + w * x)
    r22 = 1 - 2 * (x * x + y * y)
    r20 = 2 * (x * z - w * y)
    r10 = 2 * (x * y + w * z)
    r00 = 1 - 2 * (y * y + z * z)
    roll = math.atan2(r21, r22)
    pitch = math.atan2(-r20, math.sqrt(r21 * r21 + r22 * r22))
    yaw = math.atan2(r10, r00)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class TiltFilter:
    """EMA roll/pitch dengan konstanta waktu tetap, tidak bergantung laju IMU."""

    def __init__(self, tau_s, seed=None):
        """seed = (t, roll, pitch) keadaan awal; tanpa itu sampel pertama dipakai apa adanya."""
        self.tau = tau_s
        self.t = self.roll = self.pitch = None
        if seed is not None and seed[0] is not None:
            self.t, self.roll, self.pitch = seed

    def update(self, t, roll_deg, pitch_deg):
        if self.t is None:
            self.roll, self.pitch = roll_deg, pitch_deg
        else:
            a = 1.0 - math.exp(-max(t - self.t, 0.0) / self.tau)
            self.roll += a * (roll_deg - self.roll)
            self.pitch += a * (pitch_deg - self.pitch)
        self.t = t
        return self.roll, self.pitch


class FallWatch:
    """Jatuh = roll/pitch tertapis FALL_TAU_S melewati batas MUTLAK."""

    def __init__(self, limit_deg=FALL_LIMIT_DEG, tau_s=FALL_TAU_S, seed=None):
        # seed WAJIB di robot: tanpa itu, kalau sampel pertama sesudah "start"
        # kebetulan hentakan 55 deg, penapis langsung mulai dari 55 = jatuh palsu.
        self.limit = limit_deg
        self.filter = TiltFilter(tau_s, seed)

    def update(self, t, roll_deg, pitch_deg):
        roll, pitch = self.filter.update(t, roll_deg, pitch_deg)
        return abs(roll) > self.limit or abs(pitch) > self.limit


def filtered_tilt(samples, tau_s=METRIC_TAU_S):
    """(roll, pitch) tertapis untuk deretan sampel IMU (t, gx, gy, gz, roll, pitch, yaw)."""
    tilt = TiltFilter(tau_s)
    return np.array([tilt.update(s[0], s[4], s[5]) for s in samples])


def detect_fall(imu, marks):
    """Putar ulang detektor jatuh atas rekaman (sama dengan yang jalan di robot):
    disemai dari jendela berdiri, aktif dari start sampai stop."""
    seed = TiltFilter(FALL_TAU_S)
    for s in imu:
        if marks["stand0"] <= s[0] <= marks["start"]:
            seed.update(s[0], s[4], s[5])
    watch = FallWatch(seed=(seed.t, seed.roll, seed.pitch))
    stop = marks.get("planned_stop", marks["stop"])
    for s in imu:
        if marks["start"] < s[0] <= stop and watch.update(s[0], s[4], s[5]):
            return s[0]
    return None


def _window(samples, t0, t1):
    return [s for s in samples if t0 <= s[0] <= t1]


METRICS_VERSION = 4   # naik tiap kali rumus metrik/skor berubah; sesi lama di-rescore, tidak dicampur


def compute_metrics(imu, joints, marks, period_time, soft_start_time):
    """imu: (t, gx, gy, gz [rad/s], roll, pitch, yaw [deg]); joints: (t, r_hip_pitch [rad]).
    marks: stand0, start, stop, end (waktu monotonic), opsional fell_at dan planned_stop
    (akhir jalan yang DIRENCANAKAN -- saat jatuh, stop terjadi seketika itu juga)."""
    skip = max(soft_start_time, 2 * period_time)
    stand = _window(imu, marks["stand0"], marks["start"])
    walk = _window(imu, marks["start"] + skip, marks["stop"])
    out = {"fell": marks.get("fell_at") is not None, "walk_samples": len(walk)}
    duration = marks.get("planned_stop", marks["stop"]) - marks["start"]
    out["survived_frac"] = (min(1.0, (marks["fell_at"] - marks["start"]) / duration)
                            if out["fell"] and duration > 0 else 1.0)
    if len(stand) < 10 or len(walk) < 20:
        out["valid"] = False
        return out
    arr = np.array(walk)
    st = np.array(stand)
    # Penapis dijalankan dari awal berdiri supaya sudah mapan saat jendela jalan dimulai.
    whole = [s for s in imu if marks["stand0"] <= s[0] <= marks["stop"]]
    tilt = filtered_tilt(whole)
    times = np.array([s[0] for s in whole])
    walk_tilt = tilt[times >= marks["start"] + skip]
    stand_tilt = tilt[times <= marks["start"]]
    out["gyro_rms_dps"] = float(np.degrees(np.sqrt(np.mean(arr[:, 1] ** 2 + arr[:, 2] ** 2))))
    out["tilt_std_deg"] = float(math.sqrt(np.var(walk_tilt[:, 0]) + np.var(walk_tilt[:, 1])))
    out["tilt_max_deg"] = float(np.max(np.abs(walk_tilt)))
    out["lean_deg"] = float(np.mean(walk_tilt[:, 1]) - np.mean(stand_tilt[:, 1]))
    # Arah hadap: integral gyro_z dikurangi bias saat berdiri, dari berdiri-sebelum
    # sampai berdiri-sesudah. Ayunan badan tiap langkah saling meniadakan, dan
    # tidak bergantung magnetometer.
    bias = float(np.mean(st[:, 3]))
    seg = np.array(_window(imu, marks["start"], marks["end"]))
    if len(seg) > 1:
        rate = seg[:, 3] - bias
        # Trapesium manual: np.trapz hilang di NumPy 2, kontainer terpatok NumPy 1.
        out["yaw_drift_deg"] = float(np.degrees(np.sum((rate[1:] + rate[:-1]) / 2 * np.diff(seg[:, 0]))))
    else:
        out["yaw_drift_deg"] = 0.0
    hips = [j[1] for j in _window(joints, marks["start"] + skip, marks["stop"])]
    # Bukti gait benar-benar jalan: tanpa ini "robot diam" terbaca paling stabil.
    out["hip_swing_deg"] = float(np.degrees(np.std(hips))) if len(hips) > 5 else 0.0
    out["valid"] = out["hip_swing_deg"] >= 1.0
    return out


# ---- arah belok -------------------------------------------------------------------
# yaw_drift_deg mengikuti REP-103: positif = berputar berlawanan jarum jam = KIRI,
# negatif = KANAN. Diverifikasi di robot 24 Okt: sesi jb1 percobaan 33-41 bernilai
# -35..-50 deg dan operator melihat robot belok kanan; magnetometer sepakat tandanya.
TURN_STRAIGHT_DEG = 5.0   # di bawah ini dianggap lurus (= lantai skala belok)


def turn_direction(yaw_drift_deg, straight_deg=TURN_STRAIGHT_DEG):
    if yaw_drift_deg is None:
        return "-"
    if abs(yaw_drift_deg) < straight_deg:
        return "lurus"
    return "kiri" if yaw_drift_deg > 0 else "kanan"


def turn_text(yaw_drift_deg):
    """'kanan 36 deg' / 'kiri 12 deg' / 'lurus (+2 deg)' / '-'."""
    direction = turn_direction(yaw_drift_deg)
    if direction == "-":
        return "-"
    if direction == "lurus":
        return "lurus (%+.0f deg)" % yaw_drift_deg
    return "%s %.0f deg" % (direction, abs(yaw_drift_deg))


# ---- skor (makin kecil makin baik) ----------------------------------------------

WEIGHTS = {"gyro_rms_dps": 0.35, "tilt_std_deg": 0.25, "yaw_drift_deg": 0.30, "lean_deg": 0.10}
# Skala minimum: baseline JALAN BARU yang kebetulan nyaris 0 di satu metrik tidak
# boleh membuat metrik itu menenggelamkan yang lain.
FLOORS = {"gyro_rms_dps": 10.0, "tilt_std_deg": 1.0, "yaw_drift_deg": 5.0, "lean_deg": 2.0}
CLIP = 2.0
CLIP_YAW = 5.0
WEIGHT_DISTANCE = 0.4
COST_UNREACHABLE = 15.0


METRIC_TEXT = {
    "gyro_rms_dps": "goyang gyro roll/pitch saat jalan (deg/s, RMS)",
    "tilt_std_deg": "sebaran kemiringan saat jalan (deg)",
    "yaw_drift_deg": "simpangan belok dari sasaran (deg)",
    "lean_deg": "condong saat jalan dibanding berdiri (deg)",
}


def criteria_text(motion="maju", weights=None):
    """weights = bobot sesi (rencana dari feedback operator), bawaan WEIGHTS."""
    goal = ("belok sebesar belok preset sendiri (diukur di baseline), goyang paling sedikit"
            if motion_keeps_turn(motion) else "jalan selurus dan setenang mungkin")
    return ("%s; perintah gerak dikunci. Skor = jumlah berbobot metrik dibagi rata-rata preset asli "
            "(baseline ~1,0), tiap suku dijepit %.0fx (belok %.0fx): %s. Jatuh 5..10, hampir 5, "
            "pose condong sebelum melangkah 10, kaki tak terjangkau %.0f."
            % (goal, CLIP, CLIP_YAW,
               "; ".join("%.0f%% %s" % (100 * w, METRIC_TEXT[k]) for k, w in (weights or WEIGHTS).items()),
               COST_UNREACHABLE))


def scales_from_baseline(baselines, yaw_target_deg=0.0):
    """Skala tiap metrik = rata-rata baseline; belok diukur sebagai SIMPANGAN dari sasaran."""
    def size(b, k):
        return abs(b[k] - yaw_target_deg) if k == "yaw_drift_deg" else abs(b[k])
    return {k: max(FLOORS[k], float(np.mean([size(b, k) for b in baselines]))) for k in WEIGHTS}


LABELS = ("normal", "jatuh", "hampir")


def cost(metrics, scales, distance_cm=None, baseline_distance_cm=None, label=None, yaw_target_deg=0.0,
         weights=None):
    """Baseline JALAN BARU ~ 1.0. Jatuh: 5..10 (jatuh lebih lambat = lebih baik).
    Pose berdiri yang sudah condong sebelum melangkah = 10 (tidak sempat jalan).

    label = penilaian OPERATOR, mengalahkan detektor: dengan robot digantung, tali
    menahan badan, jadi operator-lah yang tahu pasti. "jatuh" yang tidak tertangkap
    detektor dihitung bertahan separuh waktu; "hampir" (dipegang/diangkat) = 5,
    seburuk jatuh paling lambat -- kandidatnya tetap harus dihindari.

    yaw_target_deg = belok yang DIMINTA preset (0 untuk maju/mundur/geser; untuk preset
    putar = belok baseline-nya), jadi yang dihukum simpangannya, bukan beloknya.

    weights = bobot sesi (bawaan WEIGHTS); kuncinya selalu sama dengan WEIGHTS."""
    weights = weights or WEIGHTS
    if metrics.get("stance_unstable"):
        return 10.0
    detected = bool(metrics.get("fell")) and label != "normal"   # "normal" = operator membatalkan alarm
    if label == "jatuh" or detected:
        return 10.0 - 5.0 * (metrics.get("survived_frac", 0.5) if metrics.get("fell") else 0.5)
    if label == "hampir":
        return 5.0
    if any(metrics.get(k) is None for k in WEIGHTS):
        return 5.0   # rekaman terpotong alarm yang ternyata palsu: tidak bisa dinilai
    # Tiap suku dijepit supaya satu kejadian aneh tidak menenggelamkan metrik lain.
    # Belok dijepit jauh lebih longgar (CLIP_YAW): jepit 2x membuat belok 30 dan 50
    # deg sama buruknya, jadi jb1 terus belok kanan ~36 deg tanpa dorongan untuk
    # meluruskan. Kasus asal jepit itu (robot diangkat tangan, tercatat belok 40 deg)
    # kini ditangani label "hampir".
    def size(k):
        return abs(metrics[k] - yaw_target_deg) if k == "yaw_drift_deg" else abs(metrics[k])
    total = sum(w * min(CLIP_YAW if k == "yaw_drift_deg" else CLIP, size(k) / scales[k])
                for k, w in weights.items())
    if distance_cm is not None and baseline_distance_cm:
        total += WEIGHT_DISTANCE * baseline_distance_cm / max(distance_cm, 0.1 * baseline_distance_cm)
    return float(total)
