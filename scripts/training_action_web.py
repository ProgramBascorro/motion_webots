#!/usr/bin/env python3
"""Bagian berkas tab "Action Training" Bascorro Studio (dipanggil apply_node).

Robot dijalankan dari browser, sama dengan tab RL Training. Di sini hanya yang
menyentuh .bin atau butuh numpy: daftar dan isi page, usulan variabel, kandidat
(page asli + perubahan nilai servo) di page kerja 254, metrik IMU satu percobaan,
dan menyimpan hasil ke page tujuan dengan cadangan .bin.

Variabel = kunci kenop autotune_action_core, jadi rumusnya satu dengan menu Training:
  "<sendi>@a-b"     derajat ditambahkan ke nilai servo sendi itu di step a..b, arah RAW
                    (angka yang sama dengan Action Editor; 4096/360 = 11,38 raw per derajat)
  "time_scale@a-b"  pengali time + pause step a..b

Batas = nilai page asli +- margin (keputusan operator 2026-10-31), dijepit lagi ke 0..4095.
Servo ALPHONSE dibaca 2026-10-31: semua mode 3 (posisi), batas 0..4095, homing offset 0;
ID 1-4 (bahu) drive mode terbalik -- lengan tidak pernah diusulkan.

Page kerja 254 dipakai bergantian dengan menu Training (nama "train_tmp" yang sama), jadi
sisa salah satunya dikenali yang lain. Page 254 yang berisi gerakan lain tidak ditimpa.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import autotune_action as act  # noqa: E402  (SCRATCH_NAME, empty_page: sama dengan menu Training)
import autotune_action_core as core  # noqa: E402
import autotune_walking_core as wcore  # noqa: E402
import training_deploy as deploy  # noqa: E402
import training_sources as src  # noqa: E402

# Tendangan di op_demo.h: Motion_Index RightKick 105 / LeftKick 104 + riwayat nomornya
# di komentar enum (kanan 32/218/220/6, kiri 104/219/221/7). Nama 219-221 tidak memuat
# kata "tend", jadi src.guess_profile tidak mengenalinya.
KICK_PAGES = (105, 32, 218, 220, 6, 104, 219, 221, 7)
HARD_DEG = 15.0            # perubahan nilai servo terbesar dari page asli, derajat
HARD_TIME = (0.6, 1.4)     # pengali time + pause terkecil / terbesar

_PART = {"hip_yaw": "pinggul yaw", "hip_roll": "pinggul roll", "hip_pitch": "pinggul pitch", "knee": "lutut",
         "ank_pitch": "engkel pitch", "ank_roll": "engkel roll", "sho_pitch": "bahu pitch", "sho_roll": "bahu roll",
         "el": "siku"}
JOINT_LABELS = {joint: ("kepala " + joint[5:] if joint.startswith("head_") else
                        "%s %s" % (_PART[joint[2:]], core.SIDE_TEXT[joint[0]]))
                for joint in src.JOINT_IDS}


def _steps_text(steps):
    return "step %d" % steps[0] if steps[0] == steps[1] else "step %d-%d" % steps


def _variable(joint, steps, group, why, spread):
    key = core.phase_key(joint, steps)
    if joint == "time_scale":
        return {"key": key, "label": "waktu " + _steps_text(steps), "group": group, "why": why, "unit": "x",
                "neutral": 1.0, "step": 0.05, "lo": spread[0], "hi": spread[1],
                "hard_lo": HARD_TIME[0], "hard_hi": HARD_TIME[1]}
    return {"key": key, "label": "%s · %s" % (JOINT_LABELS[joint], _steps_text(steps)), "group": group, "why": why,
            "unit": "deg", "neutral": 0.0, "step": 0.5, "lo": -spread, "hi": spread,
            "hard_lo": -HARD_DEG, "hard_hi": HARD_DEG}


def _valid_in(page, joint, steps):
    joint_id = src.JOINT_IDS[joint]
    return all(src.position_valid(page["steps"][n - 1]["positions"][joint_id]) for n in range(steps[0], steps[1] + 1))


def propose_variables(page):
    """(variabel usulan, fase) dari data servo page itu (core.page_phases), bertahap:
    A = tidak jatuh: kaki TUMPU selama satu kaki;
    B = kena, lurus, jauh: kaki TENDANG di step menendang (ayunan pinggul terbesar) dan sebelumnya;
    C = cepat kembali: waktu step sesudah satu kaki.
    Step pertama dan terakhir tidak pernah diubah posisinya: sambungan dari dan ke page 2.
    ([], fase) kalau page tidak terbaca sebagai gerakan satu kaki."""
    phases = core.page_phases(page, page["name"])
    n = len(page["steps"])
    if not phases or not phases["kick"] or n < 3:
        return [], phases
    kick, support = phases["kick"], phases["support"]
    first, last = phases["phases"]["satu_kaki"]
    span = (max(first, 2), min(last, n - 1))
    out = []
    if span[0] <= span[1]:
        for part in ("ank_roll", "hip_roll", "ank_pitch"):
            joint = "%s_%s" % (support, part)
            if _valid_in(page, joint, span):
                out.append(_variable(joint, span, "A", "kaki tumpu menahan badan selama satu kaki", 4.0))
    hip = src.JOINT_IDS[kick + "_hip_pitch"]

    def swing(number):
        a, b = page["steps"][number - 2]["positions"][hip], page["steps"][number - 1]["positions"][hip]
        return abs(b - a) if src.position_valid(a) and src.position_valid(b) else 0

    candidates = range(span[0], span[1] + 1)
    strike = max(candidates, key=swing) if span[0] <= span[1] else None
    if strike is not None and swing(strike):
        back = (strike - 1, strike - 1)
        if back[0] >= 2 and _valid_in(page, kick + "_hip_pitch", back):
            out.append(_variable(kick + "_hip_pitch", back, "B", "ayunan ke belakang sebelum menendang", 10.0))
        hit = (strike, strike)
        for part, spread, why in (("hip_pitch", 10.0, "ayunan ke depan saat menendang"),
                                  ("knee", 10.0, "lutut saat kaki mengenai bola"),
                                  ("ank_pitch", 10.0, "sudut telapak saat mengenai bola"),
                                  ("hip_yaw", 5.0, "arah telapak (bola lurus atau melenceng)")):
            joint = "%s_%s" % (kick, part)
            if _valid_in(page, joint, hit):
                out.append(_variable(joint, hit, "B", why, spread))
        if page["steps"][strike - 1]["time"]:
            out.append(_variable("time_scale", hit, "B", "kecepatan tendangan (lebih kecil = lebih cepat)",
                                 (0.7, 1.3)))
    if last < n:
        out.append(_variable("time_scale", (last + 1, n), "C", "waktu kembali ke posisi siap", (0.7, 1.0)))
    return out, phases


def profile_of(index, page, names):
    """Profil metrik autotune_action_core (kick/getup/pose/umum)."""
    return "kick" if index in KICK_PAGES else src.guess_profile(index, page, names)[0]


def _load():
    path, how = src.active_action_bin()
    try:
        return path, how, src.read_bin(path)
    except SystemExit as exc:      # read_bin melapor lewat SystemExit (alat terminal)
        raise ValueError(str(exc))


def _scratch_state(data):
    page = src.decode_page(data, src.TRAINING_SCRATCH_PAGE)
    if page["name"] == act.SCRATCH_NAME:
        return "kandidat"
    return "berisi gerakan %r" % page["name"] if page["stepnum"] else "kosong"


def page_list():
    """Page berisi gerakan di .bin yang dipakai action_module, tendangan duluan."""
    path, how, data = _load()
    names = src.motion_index()
    pages, free = [], None
    for index in range(1, src.MAX_PAGE):
        if index in (src.STUDIO_SCRATCH_PAGE, src.TRAINING_SCRATCH_PAGE):
            continue
        page = src.decode_page(data, index)
        if not src.page_used(page):
            free = free or index
            continue
        if not page["stepnum"]:
            continue
        pages.append({"index": index, "name": page["name"], "steps": page["stepnum"], "motion": names.get(index),
                      "kick": index in KICK_PAGES, "seconds": round(src.page_seconds(page), 2)})
    pages.sort(key=lambda p: (not p["kick"], KICK_PAGES.index(p["index"]) if p["kick"] else p["index"]))
    return {"bin": path, "source": how, "pages": pages, "free": free, "scratch": _scratch_state(data)}


def _position(value):
    if value & src.INVALID_BIT:
        return "invalid"
    return "torque_off" if value & src.TORQUE_OFF_BIT else value


def page_detail(index):
    path, how, data = _load()
    index = int(index)
    if not 1 <= index < src.MAX_PAGE:
        raise ValueError("nomor page %d di luar 1..%d" % (index, src.MAX_PAGE - 1))
    names = src.motion_index()
    errors, warnings = src.check_page(data, index)
    page = src.decode_page(data, index)
    variables, phases = propose_variables(page) if not errors else ([], None)
    chain, _ = src.page_chain(data, index)
    return {
        "bin": path, "source": how, "index": index, "name": page["name"], "sha": src.page_sha(page),
        "motion": names.get(index), "profile": profile_of(index, page, names), "kick": index in KICK_PAGES,
        "repeat": page["repeat"], "speed": page["speed"], "accel": page["accel"], "next": page["next"],
        "chain": chain, "planned_s": round(src.chain_seconds(data, chain), 2),
        "steps": [{"time": s["time"], "pause": s["pause"], "seconds": round(src.step_seconds(page, s), 2),
                   "positions": {j: _position(s["positions"][i]) for j, i in src.JOINT_IDS.items()}}
                  for s in page["steps"]],
        "phases": phases, "variables": variables, "errors": errors, "warnings": warnings,
        "joint_labels": JOINT_LABELS, "hard": {"deg": HARD_DEG, "time": list(HARD_TIME)},
        "scratch": _scratch_state(data),
    }


def check_values(page, values):
    """Galat kunci/nilai variabel (daftar kosong = sah)."""
    errors = []
    count = len(page["steps"])
    for key, value in values.items():
        try:
            name, steps = core.split_key(key)
            value = float(value)
        except (TypeError, ValueError):
            errors.append("variabel %r tidak sah" % (key,))
            continue
        if steps is None or not 1 <= steps[0] <= steps[1] <= count:
            errors.append("%s: step harus 1..%d" % (key, count))
        elif name == "time_scale":
            if not HARD_TIME[0] <= value <= HARD_TIME[1]:
                errors.append("%s = %g di luar %g..%g" % (key, value, HARD_TIME[0], HARD_TIME[1]))
        elif name in src.JOINT_IDS:
            if abs(value) > HARD_DEG:
                errors.append("%s = %+g deg melewati batas +-%g deg dari page asli" % (key, value, HARD_DEG))
        else:
            errors.append("%s: bukan nama sendi atau time_scale" % key)
    return errors


def candidate(data, index, sha, values):
    """(page asli, page kandidat, beda per servo, sendi terjepit). Page yang berubah sejak dibuka ditolak:
    kandidat harus dibandingkan dengan page yang sama."""
    page = src.decode_page(data, int(index))
    if src.page_sha(page) != sha:
        raise ValueError("page %d sudah berubah sejak dibuka (sha %s, sekarang %s) -- buka ulang page itu"
                         % (int(index), sha, src.page_sha(page)))
    errors = check_values(page, values)
    if errors:
        raise ValueError("; ".join(errors))
    knobs = {key: float(value) for key, value in values.items()}
    tuned = core.apply_knobs(page, knobs)
    diff = []
    for number, (before, after) in enumerate(zip(page["steps"], tuned["steps"]), 1):
        for joint, joint_id in src.JOINT_IDS.items():
            if before["positions"][joint_id] != after["positions"][joint_id]:
                diff.append({"step": number, "joint": joint, "before": before["positions"][joint_id],
                             "after": after["positions"][joint_id]})
        if (before["time"], before["pause"]) != (after["time"], after["pause"]):
            diff.append({"step": number, "joint": "time", "before": [before["time"], before["pause"]],
                         "after": [after["time"], after["pause"]]})
    clamped = ["step %d %s" % hit for hit in core.clamped_joints(page, knobs)]
    return page, tuned, diff, clamped


def load(index, sha, values):
    """Kandidat ke page kerja 254 (dibaca balik oleh write_page). Page asli tidak disentuh."""
    path, _, data = _load()
    page, tuned, diff, clamped = candidate(data, index, sha, values)
    scratch = src.decode_page(data, src.TRAINING_SCRATCH_PAGE)
    if scratch["stepnum"] and scratch["name"] != act.SCRATCH_NAME:
        raise ValueError("page %d berisi gerakan %r, bukan page kerja -- tidak ditimpa"
                         % (src.TRAINING_SCRATCH_PAGE, scratch["name"]))
    if tuned["next"] == page["index"]:      # page yang mengulang dirinya: ulangi kandidatnya
        tuned["next"] = src.TRAINING_SCRATCH_PAGE
    src.write_page(path, src.TRAINING_SCRATCH_PAGE, src.encode_page(tuned, name=act.SCRATCH_NAME))
    chain, _ = src.page_chain(data, page["index"])
    planned = src.page_seconds(tuned) + src.chain_seconds(data, chain[1:])
    return {"scratch": src.TRAINING_SCRATCH_PAGE, "diff": diff, "clamped": clamped, "planned_s": round(planned, 2)}


def restore():
    """Page kerja 254 kembali kosong -- hanya kalau isinya kandidat (nama train_tmp)."""
    path, _, data = _load()
    if src.decode_page(data, src.TRAINING_SCRATCH_PAGE)["name"] != act.SCRATCH_NAME:
        return {"restored": False}
    try:
        empty = act.empty_page(data)
    except SystemExit as exc:
        raise ValueError(str(exc))
    src.write_page(path, src.TRAINING_SCRATCH_PAGE, empty)
    return {"restored": True}


def trial_metrics(samples, marks, profile="kick"):
    """samples: [t, gx, gy, gz (rad/s), qx, qy, qz, qw] dari /robotis/open_cr/imu (stamp header);
    marks: stand0, start, done, end (detik, jam yang sama), planned_s. Jatuh dideteksi ulang
    dengan detektor yang sama dengan menu Training."""
    imu = sorted((float(s[0]), float(s[1]), float(s[2]), float(s[3]),
                  *wcore.quat_to_rpy_deg(float(s[4]), float(s[5]), float(s[6]), float(s[7])))
                 for s in samples)
    marks = {key: float(marks[key]) for key in ("stand0", "start", "done", "end", "planned_s")}
    marks["fell_at"] = wcore.detect_fall(imu, dict(marks, stop=marks["done"]))
    metrics = core.compute_metrics(imu, marks, profile)
    metrics["samples"] = len(imu)
    return {"metrics": metrics}


def deploy_result(index, sha, values, target, name=None, overwrite=False):
    """Hasil ke page tujuan: cadangan utuh .bin dulu (pola training_deploy), lalu tulis + baca balik.
    Page tujuan yang berisi gerakan, termasuk page asli itu sendiri, hanya ditimpa kalau overwrite."""
    path, _, data = _load()
    page, tuned, diff, _ = candidate(data, index, sha, values)
    target = int(target)
    if not 1 <= target < src.MAX_PAGE or target in (src.STUDIO_SCRATCH_PAGE, src.TRAINING_SCRATCH_PAGE):
        raise ValueError("page tujuan %d tidak boleh (1..255, bukan page kerja %d/%d)"
                         % (target, src.STUDIO_SCRATCH_PAGE, src.TRAINING_SCRATCH_PAGE))
    existing = src.decode_page(data, target)
    if src.page_used(existing) and not overwrite:
        raise ValueError("page %d berisi %r -- centang 'timpa' untuk menimpanya" % (target, existing["name"]))
    if name is not None and not (1 <= len(name) <= 13 and all(" " <= c <= "~" for c in name)):
        raise ValueError("nama page %r tidak sah (1..13 karakter ASCII)" % name)
    if tuned["next"] == page["index"]:
        tuned["next"] = target
    page_bytes = src.encode_page(tuned, name=name)
    modified = bytearray(data)
    modified[target * src.PAGE_SIZE:(target + 1) * src.PAGE_SIZE] = page_bytes
    errors, _ = src.check_page(bytes(modified), target)
    if errors:
        raise ValueError("hasil tidak sah: " + "; ".join(errors))
    backup = deploy._backup(path)
    src.write_page(path, target, page_bytes)
    return {"target": target, "name": src.decode_page(page_bytes, 0)["name"], "backup": backup,
            "changed": len(diff)}
