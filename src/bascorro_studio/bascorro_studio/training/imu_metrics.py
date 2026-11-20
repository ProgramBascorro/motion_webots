"""Metrik IMU satu percobaan tab Action Training -- MURNI (numpy, tanpa ROS).

Rekaman datang dari browser: sampel /robotis/open_cr/imu = [t, gx, gy, gz (rad/s),
qx, qy, qz, qw], t = cap waktu header (detik). Di CHRONUS quaternion itu dibangun
dari roll/pitch AKSELEROMETER + yaw kompas (OpenCRModule::publishIMU), jadi roll/
pitch yang dibaca balik di sini = kemiringan akselerometer; putaran badan diambil
dari gyro z (bukan kompas) dikurangi bias saat diam.

Rumus dan konstanta sama dengan ALPHONSE_NEW_V3 (autotune_walking_core +
autotune_action_core), supaya angka kedua robot sebanding:
  * kemiringan untuk metrik: EMA tau 0,15 s (buang hentakan, sisakan ayunan);
  * detektor jatuh: EMA tau 16 ms, |roll| atau |pitch| > 45 deg, disemai dari
    jendela berdiri (tanpa semai, hentakan pertama bisa terbaca jatuh).
Penilaian operator tetap kebenaran akhirnya; metrik ini pelengkap dan pemecah seri.
"""

import math

import numpy as np

FALL_LIMIT_DEG = 45.0
FALL_TAU_S = 0.016
METRIC_TAU_S = 0.15
UPRIGHT_DEG = 25.0     # bangun: tegak = |roll| dan |pitch| di bawah ini
SETTLE_S = 1.5         # rekaman diteruskan sesudah gerakan selesai (dipakai browser)
FINAL_WINDOW_S = 0.8   # "akhir" = sekian detik terakhir rekaman
MIN_SAMPLES = 10
PROFILES = ("kick", "getup", "pose", "umum")


def quat_to_rpy_deg(x, y, z, w):
    """robotis_framework::convertQuaternionToRPY (lewat matriks rotasi), derajat."""
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
    """EMA roll/pitch dengan konstanta waktu tetap (tidak bergantung laju IMU)."""

    def __init__(self, tau_s, seed=None):
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


def parse_samples(samples):
    """[t, gx, gy, gz, qx, qy, qz, qw] -> (t, gx, gy, gz, roll, pitch, yaw) terurut waktu."""
    out = []
    for s in samples:
        if len(s) < 8:
            raise ValueError("sampel IMU harus 8 angka [t, gx, gy, gz, qx, qy, qz, qw]")
        values = [float(v) for v in s[:8]]
        out.append((values[0], values[1], values[2], values[3], *quat_to_rpy_deg(*values[4:8])))
    return sorted(out)


def filtered_tilt(imu, tau_s=METRIC_TAU_S):
    tilt = TiltFilter(tau_s)
    return np.array([tilt.update(s[0], s[4], s[5]) for s in imu])


def detect_fall(imu, marks):
    """Waktu jatuh pertama antara start dan done, atau None."""
    seed = TiltFilter(FALL_TAU_S)
    for s in imu:
        if marks["stand0"] <= s[0] <= marks["start"]:
            seed.update(s[0], s[4], s[5])
    watch = TiltFilter(FALL_TAU_S, (seed.t, seed.roll, seed.pitch))
    for s in imu:
        if marks["start"] < s[0] <= marks["done"]:
            roll, pitch = watch.update(s[0], s[4], s[5])
            if abs(roll) > FALL_LIMIT_DEG or abs(pitch) > FALL_LIMIT_DEG:
                return s[0]
    return None


def _window(imu, t0, t1):
    return [s for s in imu if t0 <= s[0] <= t1]


def _gyro_rms_dps(window):
    if not window:
        return None
    arr = np.array(window)
    return float(np.degrees(np.sqrt(np.mean(arr[:, 1] ** 2 + arr[:, 2] ** 2))))


def compute_metrics(imu, marks, profile):
    """imu: hasil parse_samples; marks: stand0, start (page diperintah), done (action_module
    selesai), end (akhir rekaman), planned_s. Kunci yang tidak terukur = None, valid=False
    kalau rekaman berdiri atau rekaman gerakan kurang dari MIN_SAMPLES."""
    if profile not in PROFILES:
        raise ValueError("profil %r bukan salah satu %s" % (profile, ", ".join(PROFILES)))
    lying = profile == "getup"
    fell_at = None if lying else detect_fall(imu, marks)
    out = {"fell": fell_at is not None}
    planned = max(marks.get("planned_s") or 1.0, 0.1)
    out["survived_frac"] = min(1.0, (fell_at - marks["start"]) / planned) if fell_at is not None else 1.0
    pre = _window(imu, marks["stand0"], marks["start"])
    play = _window(imu, marks["start"], marks["done"])
    after = _window(imu, marks["done"], marks["end"])
    out["duration_s"] = marks["done"] - marks["start"]
    if len(pre) < MIN_SAMPLES or len(play) < MIN_SAMPLES:
        out["valid"] = False
        return out
    whole = _window(imu, marks["stand0"], marks["end"])
    tilt = filtered_tilt(whole)
    times = np.array([s[0] for s in whole])
    ref = tilt[times <= marks["start"]].mean(axis=0)
    dev = np.hypot(tilt[:, 0] - ref[0], tilt[:, 1] - ref[1])
    moving = (times >= marks["start"]) & (times <= marks["done"])
    final = times >= marks["end"] - FINAL_WINDOW_S
    span = tilt[times >= marks["start"]]
    out["gyro_rms_dps"] = _gyro_rms_dps(play)
    out["tilt_max_deg"] = float(dev[moving].max())
    out["tilt_std_deg"] = float(math.sqrt(np.var(span[:, 0]) + np.var(span[:, 1])))
    out["settle_dev_deg"] = float(dev[final].mean()) if final.any() else None
    out["settle_gyro_dps"] = _gyro_rms_dps(after)
    # Putaran: integral gyro z dikurangi bias saat diam (tidak memakai kompas).
    bias = float(np.mean([s[3] for s in pre]))
    seg = np.array(_window(imu, marks["start"], marks["end"]))
    rate = seg[:, 3] - bias
    out["yaw_abs_deg"] = float(abs(np.degrees(np.sum((rate[1:] + rate[:-1]) / 2 * np.diff(seg[:, 0])))))
    if lying:
        upright = np.maximum(np.abs(tilt[:, 0]), np.abs(tilt[:, 1]))
        out["initial_tilt_deg"] = float(upright[times <= marks["start"]].mean())
        out["final_tilt_deg"] = float(upright[final].mean()) if final.any() else None
        out["success"] = out["final_tilt_deg"] is not None and out["final_tilt_deg"] < UPRIGHT_DEG
        out["time_to_upright_s"] = None
        if out["success"]:
            # sejak sampel terakhir yang belum tegak: "tegak dan TETAP tegak"
            later = times >= marks["start"]
            bad = np.where(later & (upright >= UPRIGHT_DEG))[0]
            first = (bad[-1] + 1) if len(bad) else int(np.argmax(later))
            out["time_to_upright_s"] = float(times[first] - marks["start"])
    out["valid"] = True
    return out


def trial_metrics(samples, marks, profile="kick"):
    """Pintu masuk apply_node (aksi act_metrics)."""
    keys = ("stand0", "start", "done", "end", "planned_s")
    missing = [k for k in keys if k not in (marks or {})]
    if missing:
        raise ValueError("marks kurang: %s" % ", ".join(missing))
    clean = {k: float(marks[k]) for k in keys}
    metrics = compute_metrics(parse_samples(samples or []), clean, profile)
    metrics["samples"] = len(samples or [])
    return metrics
