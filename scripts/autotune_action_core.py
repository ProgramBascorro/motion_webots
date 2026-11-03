"""Bagian MURNI training page Action Editor (tanpa ROS): kenop, profil, metrik, skor.

Dipakai scripts/autotune_action.py; diuji scripts/test_training.py.
Pencarian (CMA-ES), penapis kemiringan, dan detektor jatuh SAMA dengan training
walking (autotune_walking_core) -- yang berbeda cuma APA yang diubah dan APA
yang dinilai.

APA YANG DIUBAH: page TIDAK ditulis ulang bebas sendi per sendi (31 x 7 angka di
robot sungguhan = membanting robot). Yang dicari beberapa KENOP yang dikenakan
ke SEMUA step page itu:

  time_scale       pengali time + pause tiap step (1,0 = asli)
  pitch_hip_deg    condong pinggul, dua kaki searah
  pitch_ankle_deg  condong engkel, dua kaki searah
  roll_hip_deg     geser badan ke samping lewat pinggul, dua kaki searah
  roll_ankle_deg   geser badan ke samping lewat engkel, dua kaki searah

"Searah" = arah IK walking_module: sudut sendi = arah_sendi x sudut, dengan
arah_sendi = DIR_R/DIR_L (walking_ready_fk.py, dari joint_axis op3_kinematics).
Jadi pitch kanan dan kiri berlawanan tanda di raw (cermin), roll searah.
Sendi yang di page bertanda INVALID/TORQUE_OFF dibiarkan.

Rumusnya, per step dan per sendi (raw Dynamixel 0..4095, 4096/360 = 11,38 raw/deg):

  raw_baru = raw_asli + round(arah_sendi x kenop_deg x 4096/360)    (dijepit 0..4095)
  time_baru = round(time x time_scale), pause_baru = round(pause x time_scale)

Arah BADAN (diturunkan dari IK, diuji di test_training.py, bukan ditulis tangan):
roll_ankle_deg NEGATIF = seluruh badan condong ke KANAN di atas engkel;
pitch_ankle_deg NEGATIF = seluruh badan condong ke DEPAN di atas engkel;
pitch_hip_deg NEGATIF = badan atas menekuk ke DEPAN di pinggul.

KENOP FASE (dari feedback operator, training_iteration.py): kunci "kenop@a-b"
= kenop yang sama tapi HANYA di step a..b (1 = step pertama), di atas kenop
global: sudut DITAMBAHKAN, time_scale DIKALIKAN. Contoh roll_ankle_deg@2-5 = -0,5
= badan 0,5 deg lebih ke kanan hanya selama step 2-5 (saat bertumpu satu kaki);
step lain tidak berubah. Fase page dibaca dari DATA servo (page_phases), bukan
dari nama page.
"""

import math

import numpy as np

import autotune_walking_core as wcore
from training_sources import JOINT_IDS, position_valid
from walking_ready_fk import DIR_L, DIR_R

RAW_PER_DEG = 4096.0 / 360.0

# kenop -> [(sendi, arah)], diturunkan dari arah sendi IK, bukan ditulis tangan
_LEG = ["hip_yaw", "hip_roll", "hip_pitch", "knee", "ank_pitch", "ank_roll"]


def _pair(joint):
    i = _LEG.index(joint)
    return (("r_" + joint, DIR_R[i]), ("l_" + joint, DIR_L[i]))


KNOB_JOINTS = {
    "pitch_hip_deg": _pair("hip_pitch"),
    "pitch_ankle_deg": _pair("ank_pitch"),
    "roll_hip_deg": _pair("hip_roll"),
    "roll_ankle_deg": _pair("ank_roll"),
}
NEUTRAL = {"time_scale": 1.0, "pitch_hip_deg": 0.0, "pitch_ankle_deg": 0.0, "roll_hip_deg": 0.0, "roll_ankle_deg": 0.0}

# Bentuk sama dengan autotune_walking_core.BOUNDS ("abs" = mutlak). Sempit dengan
# sengaja: ini penyetelan di sekitar page asli, bukan merancang gerakan baru.
BOUNDS = {
    "time_scale": ("abs", 0.85, 1.25),
    "pitch_hip_deg": ("abs", -3.0, 3.0),
    "pitch_ankle_deg": ("abs", -3.0, 3.0),
    "roll_hip_deg": ("abs", -2.0, 2.0),
    "roll_ankle_deg": ("abs", -2.0, 2.0),
}

# ---- profil = jenis gerakan --------------------------------------------------------
# params  : kenop yang dicari (time_scale TIDAK ikut di tendangan: lebih lambat =
#           lebih stabil tapi lebih lemah, jadi "pelan" otomatis menang)
# weights : metrik skor (lihat compute_metrics), jumlahnya 1
# lying   : percobaan dimulai dari robot BERBARING (bangun), bukan dari page 2
PROFILES = {
    "kick": {
        "label": "tendangan", "lying": False,
        "params": ["pitch_hip_deg", "pitch_ankle_deg", "roll_hip_deg", "roll_ankle_deg"],
        "weights": {"gyro_rms_dps": 0.30, "tilt_max_deg": 0.25, "settle_dev_deg": 0.20,
                    "settle_gyro_dps": 0.10, "yaw_abs_deg": 0.15},
        "goal": "badan paling tenang selama & sesudah menendang, kembali tegak, tidak berputar; "
                "kecepatan tendangan dikunci",
    },
    "getup": {
        "label": "bangun", "lying": True,
        "params": ["time_scale", "pitch_hip_deg", "pitch_ankle_deg"],
        "weights": {"time_to_upright_s": 0.40, "gyro_rms_dps": 0.30, "settle_gyro_dps": 0.30},
        "goal": "WAJIB berhasil berdiri; di antara yang berhasil: paling cepat tegak, paling halus, "
                "paling sedikit terhuyung sesudah gerakan selesai",
    },
    "pose": {
        "label": "pose diam", "lying": False,
        "params": ["pitch_hip_deg", "pitch_ankle_deg", "roll_ankle_deg"],
        "weights": {"gyro_rms_dps": 0.40, "tilt_std_deg": 0.40, "settle_gyro_dps": 0.20},
        "goal": "paling sedikit bergoyang saat berpindah ke pose dan saat menahannya",
    },
    "umum": {
        "label": "gerakan umum", "lying": False,
        "params": ["time_scale", "pitch_hip_deg", "pitch_ankle_deg"],
        "weights": {"gyro_rms_dps": 0.35, "tilt_max_deg": 0.30, "settle_dev_deg": 0.25, "settle_gyro_dps": 0.10},
        "goal": "badan paling tenang selama gerakan dan kembali tegak sesudahnya",
    },
}

METRIC_TEXT = {
    "gyro_rms_dps": "goyang gyro roll/pitch selama gerakan (deg/s, RMS)",
    "tilt_max_deg": "miring terbesar dari pose awal selama gerakan (deg)",
    "tilt_std_deg": "sebaran kemiringan selama gerakan + tahan (deg)",
    "settle_dev_deg": "selisih kemiringan akhir dari pose awal (deg)",
    "settle_gyro_dps": "goyang gyro sesudah gerakan selesai (deg/s)",
    "yaw_abs_deg": "badan berputar (yaw) selama gerakan (deg)",
    "time_to_upright_s": "waktu sampai tegak dan tetap tegak (s)",
}
FLOORS = {"gyro_rms_dps": 10.0, "tilt_max_deg": 2.0, "tilt_std_deg": 1.0, "settle_dev_deg": 1.0,
          "settle_gyro_dps": 3.0, "yaw_abs_deg": 5.0, "time_to_upright_s": 1.0}
CLIP = 2.0
UPRIGHT_DEG = 25.0     # bangun: tegak = |roll| dan |pitch| akselerometer di bawah ini
LYING_DEG = 45.0       # bangun: percobaan hanya dimulai kalau robot memang berbaring
SETTLE_S = 1.5         # rekaman diteruskan sesudah gerakan selesai
FINAL_WINDOW_S = 0.8   # "akhir" = sekian detik terakhir rekaman
METRICS_VERSION = 1


# Arah badan per kenop pergelangan (lihat docstring): dipakai feedback "badan ke kanan/depan".
BODY_RIGHT = {"roll_ankle_deg": -1.0}
BODY_FORWARD = {"pitch_ankle_deg": -1.0}
ANGLE_KNOBS = ("pitch_hip_deg", "pitch_ankle_deg", "roll_hip_deg", "roll_ankle_deg")


# ---- kenop -> page ----------------------------------------------------------------

def split_key(key):
    """"roll_ankle_deg@2-5" -> ("roll_ankle_deg", (2, 5)); kenop global -> (kenop, None)."""
    knob, _, steps = key.partition("@")
    if not steps:
        return knob, None
    first, _, last = steps.partition("-")
    return knob, (int(first), int(last or first))


def phase_key(knob, steps):
    return knob if steps is None else "%s@%d-%d" % (knob, steps[0], steps[1])


def step_knobs(knobs, number):
    """Kenop yang berlaku di step `number` (1 = pertama): global + kenop fase yang mencakupnya."""
    here = {k: v for k, v in knobs.items() if "@" not in k}
    for key, value in knobs.items():
        knob, steps = split_key(key)
        if steps and steps[0] <= number <= steps[1]:
            if knob == "time_scale":
                here["time_scale"] = here.get("time_scale", 1.0) * value
            else:
                here[knob] = here.get(knob, 0.0) + value
    return here


def joint_deltas(knobs):
    """{id sendi: perubahan raw} dari kenop sudut (time_scale tidak termasuk).
    Kunci yang berupa nama sendi (tab Action Training: satu servo) = derajat dalam arah RAW,
    angka yang sama dengan Action Editor -- bukan arah IK seperti kenop."""
    deltas = {}
    for knob, pairs in KNOB_JOINTS.items():
        value = knobs.get(knob, 0.0)
        for joint, direction in pairs:
            deltas[JOINT_IDS[joint]] = deltas.get(JOINT_IDS[joint], 0.0) + direction * value * RAW_PER_DEG
    for joint, value in knobs.items():
        if joint in JOINT_IDS:
            deltas[JOINT_IDS[joint]] = deltas.get(JOINT_IDS[joint], 0.0) + value * RAW_PER_DEG
    return {i: int(round(d)) for i, d in deltas.items() if int(round(d))}


def apply_knobs(page, knobs):
    """Page baru (dict training_sources) = page asli + kenop (termasuk kenop fase "kenop@a-b").
    Page asli tidak diubah."""
    steps = []
    for number, step in enumerate(page["steps"], 1):
        here = step_knobs(knobs, number)
        scale = here.get("time_scale", 1.0)
        deltas = joint_deltas(here)
        positions = list(step["positions"])
        for joint_id, delta in deltas.items():
            if position_valid(positions[joint_id]):
                positions[joint_id] = min(4095, max(0, positions[joint_id] + delta))
        time_value = step["time"] if step["time"] == 0 else min(255, max(1, int(round(step["time"] * scale))))
        steps.append({"positions": positions, "pause": min(255, max(0, int(round(step["pause"] * scale)))),
                      "time": time_value})
    return dict(page, steps=steps)


def clamped_joints(page, knobs):
    """[(step, sendi)] yang hasilnya akan terjepit di 0/4095 -- kenop itu tidak bisa dipakai utuh."""
    out = []
    for number, step in enumerate(page["steps"], 1):
        for joint_id, delta in joint_deltas(step_knobs(knobs, number)).items():
            value = step["positions"][joint_id]
            if position_valid(value) and not 0 <= value + delta <= 4095:
                out.append((number, next(j for j, i in JOINT_IDS.items() if i == joint_id)))
    return out


# ---- fase page dari data servo -------------------------------------------------------
# Satu kaki terangkat = kaki kiri dan kanan bergerak TIDAK sama (dilihat dalam arah IK,
# jadi cermin kiri-kanan dihitung sama). Step pertama = acuan berdiri.
ASYM_DEG = 8.0
_PITCH = ("hip_pitch", "knee", "ank_pitch")


def page_phases(page, name_text=""):
    """Bagian page menurut data servo: {rows, kick ('l'/'r'/None), support, single (a,b),
    phases {awal, satu_kaki, kembali: (a,b)}, name_side, evidence}. None kalau < 2 step."""
    steps = page["steps"]
    if len(steps) < 2:
        return None
    first = steps[0]["positions"]
    rows = []
    for number, step in enumerate(steps, 1):
        pos = step["positions"]
        asym, move = 0.0, {"r": 0.0, "l": 0.0}
        for joint in _PITCH:
            i = _LEG.index(joint)
            r_id, l_id = JOINT_IDS["r_" + joint], JOINT_IDS["l_" + joint]
            if not all(position_valid(v) for v in (pos[r_id], pos[l_id], first[r_id], first[l_id])):
                continue
            ik_r = DIR_R[i] * (pos[r_id] - first[r_id]) / RAW_PER_DEG
            ik_l = DIR_L[i] * (pos[l_id] - first[l_id]) / RAW_PER_DEG
            asym = max(asym, abs(ik_r - ik_l))
            move["r"], move["l"] = max(move["r"], abs(ik_r)), max(move["l"], abs(ik_l))
        rows.append({"step": number, "asym_deg": round(asym, 1), "move": {k: round(v, 1) for k, v in move.items()}})
    lowered = name_text.lower()
    name_side = ("l" if any(w in lowered for w in ("kiri", "left")) else
                 "r" if any(w in lowered for w in ("kanan", "right")) else None)
    out = {"rows": rows, "kick": None, "support": None, "single": None, "phases": {}, "name_side": name_side}
    single = [r["step"] for r in rows if r["asym_deg"] >= ASYM_DEG]
    if not single:
        return out
    a, b = single[0], single[-1]
    span = [r for r in rows if a <= r["step"] <= b]
    moved = {side: max(r["move"][side] for r in span) for side in ("l", "r")}
    kick = "l" if moved["l"] >= moved["r"] else "r"
    # step sebelum satu kaki (memindahkan berat) ikut fase satu kaki: badan sudah di atas
    # kaki tumpuan sebelum kaki lain terangkat
    start = max(1, a - 1)
    phases = {"satu_kaki": (start, b)}
    if start > 1:
        phases["awal"] = (1, start - 1)
    if b < len(steps):
        phases["kembali"] = (b + 1, len(steps))
    out.update(kick=kick, support="r" if kick == "l" else "l", single=(a, b), phases=phases,
               evidence="kaki %s bergerak sampai %.0f deg dari step 1, kaki %s %.0f deg (step %d-%d tidak simetris)" % (
                   SIDE_TEXT[kick], moved[kick], SIDE_TEXT["r" if kick == "l" else "l"],
                   moved["r" if kick == "l" else "l"], a, b))
    return out


SIDE_TEXT = {"l": "kiri", "r": "kanan"}
PHASE_NAME = {"awal": "persiapan", "satu_kaki": "bertumpu satu kaki (menendang)", "kembali": "kembali ke posisi siap"}


def changed_joints(original, tuned):
    """Ringkasan perubahan per sendi (raw) untuk laporan/ekspor."""
    out = {}
    for joint, joint_id in JOINT_IDS.items():
        diffs = {b["positions"][joint_id] - a["positions"][joint_id]
                 for a, b in zip(original["steps"], tuned["steps"])
                 if position_valid(a["positions"][joint_id])}
        diffs.discard(0)
        if diffs:
            out[joint] = sorted(diffs)
    return out


# ---- metrik dari rekaman satu percobaan ---------------------------------------

def _window(samples, t0, t1):
    return [s for s in samples if t0 <= s[0] <= t1]


def _gyro_rms(samples):
    if not samples:
        return None
    arr = np.array(samples)
    return float(np.degrees(np.sqrt(np.mean(arr[:, 1] ** 2 + arr[:, 2] ** 2))))


def compute_metrics(imu, marks, profile):
    """imu: (t, gx, gy, gz [rad/s], roll, pitch, yaw [deg]); marks (waktu monotonic):
    stand0, start (page diperintah), done (action_module selesai), end, opsional fell_at,
    planned_s (perkiraan durasi page). Kemiringan = akselerometer tertapis METRIC_TAU_S."""
    out = {"fell": marks.get("fell_at") is not None}
    planned = max(marks.get("planned_s") or 1.0, 0.1)
    out["survived_frac"] = (min(1.0, (marks["fell_at"] - marks["start"]) / planned) if out["fell"] else 1.0)
    pre = _window(imu, marks["stand0"], marks["start"])
    play = _window(imu, marks["start"], marks["done"])
    after = _window(imu, marks["done"], marks["end"])
    out["duration_s"] = marks["done"] - marks["start"]
    out["timed_out"] = bool(marks.get("timed_out"))
    if len(pre) < 10 or len(play) < 10:
        out["valid"] = False
        return out
    whole = _window(imu, marks["stand0"], marks["end"])
    tilt = wcore.filtered_tilt(whole)
    times = np.array([s[0] for s in whole])
    ref = tilt[times <= marks["start"]].mean(axis=0)
    dev = np.hypot(tilt[:, 0] - ref[0], tilt[:, 1] - ref[1])
    moving = (times >= marks["start"]) & (times <= marks["done"])
    final = times >= marks["end"] - FINAL_WINDOW_S
    out["gyro_rms_dps"] = _gyro_rms(play)
    out["tilt_max_deg"] = float(dev[moving].max())
    span = tilt[times >= marks["start"]]
    out["tilt_std_deg"] = float(math.sqrt(np.var(span[:, 0]) + np.var(span[:, 1])))
    out["settle_dev_deg"] = float(dev[final].mean()) if final.any() else None
    out["settle_gyro_dps"] = _gyro_rms(after)
    # Yaw: integral gyro z dikurangi bias saat diam (sama dengan training walking).
    bias = float(np.mean([s[3] for s in pre]))
    seg = np.array(_window(imu, marks["start"], marks["end"]))
    rate = seg[:, 3] - bias
    out["yaw_abs_deg"] = float(abs(np.degrees(np.sum((rate[1:] + rate[:-1]) / 2 * np.diff(seg[:, 0])))))
    if profile == "getup":
        upright = np.maximum(np.abs(tilt[:, 0]), np.abs(tilt[:, 1]))
        out["initial_tilt_deg"] = float(upright[times <= marks["start"]].mean())
        out["final_tilt_deg"] = float(upright[final].mean()) if final.any() else None
        out["success"] = out["final_tilt_deg"] is not None and out["final_tilt_deg"] < UPRIGHT_DEG
        out["progress"] = float(np.clip(1.0 - (out["final_tilt_deg"] or out["initial_tilt_deg"])
                                        / max(out["initial_tilt_deg"], 1.0), 0.0, 1.0))
        out["time_to_upright_s"] = None
        if out["success"]:
            # sejak sampel terakhir yang belum tegak: "tegak dan TETAP tegak"
            later = times >= marks["start"]
            bad = np.where(later & (upright >= UPRIGHT_DEG))[0]
            first = (bad[-1] + 1) if len(bad) else int(np.argmax(later))
            out["time_to_upright_s"] = float(times[first] - marks["start"])
        out["fell"] = False   # dimulai berbaring: detektor jatuh tidak berlaku, yang dinilai "berhasil tegak"
    out["valid"] = True
    return out


# ---- skor (makin kecil makin baik) ----------------------------------------------

def scales_from_baseline(baselines, weights):
    return {k: max(FLOORS[k], float(np.mean([abs(b[k]) for b in baselines]))) for k in weights}


def failed(metrics, label, profile):
    """Percobaan gagal menurut operator atau detektor."""
    if label in ("jatuh", "hampir"):
        return True
    if label == "normal":
        return False
    if profile == "getup":
        return not metrics.get("success")
    return bool(metrics.get("fell"))


def cost(metrics, scales, weights, profile, label=None):
    """Baseline (page asli) ~ 1,0. Gagal: 5..10.

    label = penilaian OPERATOR, mengalahkan detektor (sama dengan training walking):
    "jatuh" (bangun: gagal tegak), "hampir" (dipegang/diangkat) = 5, "normal" =
    membatalkan alarm detektor."""
    if label == "hampir":
        return 5.0
    if failed(metrics, label, profile):
        if profile == "getup":
            return 10.0 - 5.0 * metrics.get("progress", 0.0)
        return 10.0 - 5.0 * (metrics.get("survived_frac", 0.5) if metrics.get("fell") else 0.5)
    if any(metrics.get(k) is None for k in weights):
        return 5.0   # rekaman tidak lengkap (mis. operator membatalkan "gagal" bangun): tidak bisa dinilai
    return float(sum(w * min(CLIP, abs(metrics[k]) / scales[k]) for k, w in weights.items()))


def criteria_text(profile, weights=None):
    """weights = bobot sesi (rencana dari feedback operator), bawaan bobot profil."""
    spec = PROFILES[profile]
    return "%s. Skor = jumlah berbobot metrik dibagi rata-rata page asli (baseline ~1,0), " \
           "tiap suku dijepit %.0fx: %s. Gagal/jatuh = 5..10." % (
               spec["goal"], CLIP, "; ".join("%.0f%% %s" % (100 * w, METRIC_TEXT[k])
                                             for k, w in (weights or spec["weights"]).items()))
