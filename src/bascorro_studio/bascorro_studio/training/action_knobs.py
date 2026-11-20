"""Variabel tab Action Training -> page baru, dan fase page dari data servo -- MURNI.

Satu variabel = satu kunci:
  "<sendi>@a-b"     derajat DITAMBAHKAN ke nilai servo sendi itu di step a..b (1 = step
                    pertama), arah RAW -- angka yang sama dengan Action Editor
  "time_scale@a-b"  pengali time + pause step a..b

Rumus (raw Dynamixel 0..4095, 4096/360 = 11,38 raw per derajat; sama untuk MX dan XM):
  raw_baru  = raw_asli + round(derajat x 4096/360)          (dijepit 0..4095)
  time_baru = round(time x skala), pause_baru = round(pause x skala)
Variabel yang mencakup step yang sama dijumlahkan (sudut) / dikalikan (skala).
Posisi INVALID / TORQUE_OFF tidak disentuh.

Rumus ini sama dengan apply_knobs milik ALPHONSE_NEW_V3 (autotune_action_core), jadi
hasil percobaan kedua robot bisa dibandingkan; kenop IK global milik menu Training
terminal (pitch_hip_deg dll.) tidak dibawa karena tidak dipakai Studio.
"""

import re

from .action_bin import JOINT_IDS, position_valid

RAW_PER_DEG = 4096.0 / 360.0

# Arah sendi IK walking_module CHRONUS: joint_axis_direction_ di
# op3_walking_module.cpp (= sumbu op3_kinematics_dynamics tautan 7..18).
# Dipakai untuk membaca fase page: kaki kiri dan kanan yang bergerak "sama"
# dalam arah IK berarti dua kaki menapak; yang berbeda berarti satu kaki terangkat.
LEG = ["hip_yaw", "hip_roll", "hip_pitch", "knee", "ank_pitch", "ank_roll"]
DIR_R = [-1, -1, -1, -1, 1, 1]
DIR_L = [-1, -1, 1, 1, -1, 1]
SIDE_TEXT = {"l": "kiri", "r": "kanan"}

ASYM_DEG = 8.0                      # beda kiri-kanan (arah IK) yang dianggap satu kaki terangkat
_PITCH = ("hip_pitch", "knee", "ank_pitch")


def split_key(key):
    """"r_knee@2-5" -> ("r_knee", (2, 5)); "time_scale@3" -> ("time_scale", (3, 3)); tanpa @ -> (nama, None)."""
    name, _, steps = str(key).partition("@")
    if not steps:
        return name, None
    first, _, last = steps.partition("-")
    return name, (int(first), int(last or first))


def phase_key(name, steps):
    return name if steps is None else "%s@%d-%d" % (name, steps[0], steps[1])


def step_knobs(knobs, number):
    """{nama: nilai} yang berlaku di step `number`: sudut dijumlah, time_scale dikalikan."""
    here = {}
    for key, value in knobs.items():
        name, steps = split_key(key)
        if steps is not None and not steps[0] <= number <= steps[1]:
            continue
        if name == "time_scale":
            here["time_scale"] = here.get("time_scale", 1.0) * float(value)
        else:
            here[name] = here.get(name, 0.0) + float(value)
    return here


def joint_deltas(here):
    """{id servo: perubahan raw} dari derajat per nama sendi (arah raw)."""
    deltas = {}
    for name, value in here.items():
        if name in JOINT_IDS:
            delta = int(round(value * RAW_PER_DEG))
            if delta:
                deltas[JOINT_IDS[name]] = deltas.get(JOINT_IDS[name], 0) + delta
    return deltas


def apply_knobs(page, knobs):
    """Page baru = page asli + variabel. Page asli tidak diubah."""
    steps = []
    for number, step in enumerate(page["steps"], 1):
        here = step_knobs(knobs, number)
        scale = here.get("time_scale", 1.0)
        positions = list(step["positions"])
        for joint_id, delta in joint_deltas(here).items():
            if position_valid(positions[joint_id]):
                positions[joint_id] = min(4095, max(0, positions[joint_id] + delta))
        time_value = step["time"] if step["time"] == 0 else min(255, max(1, int(round(step["time"] * scale))))
        steps.append({"positions": positions, "pause": min(255, max(0, int(round(step["pause"] * scale)))),
                      "time": time_value})
    return dict(page, steps=steps)


def clamped_joints(page, knobs):
    """[(step, sendi)] yang hasilnya terjepit di 0/4095 -- variabel itu tidak terpakai utuh."""
    names = {joint_id: name for name, joint_id in JOINT_IDS.items()}
    out = []
    for number, step in enumerate(page["steps"], 1):
        for joint_id, delta in joint_deltas(step_knobs(knobs, number)).items():
            value = step["positions"][joint_id]
            if position_valid(value) and not 0 <= value + delta <= 4095:
                out.append((number, names[joint_id]))
    return out


def page_diff(original, tuned):
    """Daftar nilai yang berubah: {step, joint, before, after}; joint "time" = [time, pause]."""
    names = {joint_id: name for name, joint_id in JOINT_IDS.items()}
    diff = []
    for number, (before, after) in enumerate(zip(original["steps"], tuned["steps"]), 1):
        for joint_id in sorted(names):
            if before["positions"][joint_id] != after["positions"][joint_id]:
                diff.append({"step": number, "joint": names[joint_id], "before": before["positions"][joint_id],
                             "after": after["positions"][joint_id]})
        if (before["time"], before["pause"]) != (after["time"], after["pause"]):
            diff.append({"step": number, "joint": "time", "before": [before["time"], before["pause"]],
                         "after": [after["time"], after["pause"]]})
    return diff


def page_phases(page, name_text=""):
    """Bagian page menurut DATA servo, bukan nama: {rows, kick, support, single, phases,
    name_side, evidence}. kick = kaki yang terangkat ('l'/'r'), None kalau tidak ada fase
    satu kaki. None kalau page < 2 step. Step pertama = acuan berdiri."""
    steps = page["steps"]
    if len(steps) < 2:
        return None
    first = steps[0]["positions"]
    rows = []
    for number, step in enumerate(steps, 1):
        pos = step["positions"]
        asym, move = 0.0, {"r": 0.0, "l": 0.0}
        for joint in _PITCH:
            i = LEG.index(joint)
            r_id, l_id = JOINT_IDS["r_" + joint], JOINT_IDS["l_" + joint]
            if not all(position_valid(v) for v in (pos[r_id], pos[l_id], first[r_id], first[l_id])):
                continue
            ik_r = DIR_R[i] * (pos[r_id] - first[r_id]) / RAW_PER_DEG
            ik_l = DIR_L[i] * (pos[l_id] - first[l_id]) / RAW_PER_DEG
            asym = max(asym, abs(ik_r - ik_l))
            move["r"], move["l"] = max(move["r"], abs(ik_r)), max(move["l"], abs(ik_l))
        rows.append({"step": number, "asym_deg": round(asym, 1), "move": {k: round(v, 1) for k, v in move.items()}})
    lowered = (name_text or "").lower()
    name_side = ("l" if re.search(r"kiri|left|(^|[^a-z])l_", lowered) else
                 "r" if re.search(r"kanan|right|(^|[^a-z])r_", lowered) else None)
    out = {"rows": rows, "kick": None, "support": None, "single": None, "phases": {}, "name_side": name_side}
    single = [r["step"] for r in rows if r["asym_deg"] >= ASYM_DEG]
    if not single:
        return out
    a, b = single[0], single[-1]
    span = [r for r in rows if a <= r["step"] <= b]
    moved = {side: max(r["move"][side] for r in span) for side in ("l", "r")}
    kick = "l" if moved["l"] >= moved["r"] else "r"
    support = "r" if kick == "l" else "l"
    # Step sebelum satu kaki (memindahkan berat) ikut fase satu kaki: badan sudah
    # di atas kaki tumpu sebelum kaki lain terangkat.
    start = max(1, a - 1)
    phases = {"satu_kaki": (start, b)}
    if start > 1:
        phases["awal"] = (1, start - 1)
    if b < len(steps):
        phases["kembali"] = (b + 1, len(steps))
    out.update(kick=kick, support=support, single=(a, b), phases=phases,
               evidence="kaki %s bergerak sampai %.0f deg dari step 1, kaki %s %.0f deg (step %d-%d tidak simetris)" % (
                   SIDE_TEXT[kick], moved[kick], SIDE_TEXT[support], moved[support], a, b))
    return out
