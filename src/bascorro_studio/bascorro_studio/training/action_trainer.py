"""Layanan tab "Action Training" Bascorro Studio (dipanggil apply_node, aksi act_*).

Robot dijalankan dari BROWSER (web/src/training/actionTrial.js). Di sini hanya yang
menyentuh .bin atau butuh numpy: daftar dan isi page, usulan variabel, kandidat di
page kerja 254, metrik IMU satu percobaan, dan menyimpan hasil ke page tujuan
dengan cadangan .bin utuh. Page asli TIDAK pernah ditulis selama training.

Diadaptasi dari ALPHONSE_NEW_V3 scripts/training_action_web.py. Bedanya di CHRONUS:
  * Jenis gerakan dari NAMA PAGE, bukan Motion_Index: enum op_demo.h di sini tidak
    sejalan dengan .bin (WalkingReady = 8 padahal page 8 = TEND_KIRI; RightKick 218
    dan LeftKick 104 kosong). Daftar tendangan tidak ditulis di kode.
  * Selain tendangan, page bangun (BANGBEL 12 dipakai D58) dan pose (menahan bola 14)
    juga bisa dilatih; usulan variabel mengikuti jenisnya.
  * Rentang usulan bawaan lebih sempit (lihat SPREAD). Batas keras sama.
  * Sebelum kandidat ditulis, dicek bahwa op3_manager yang hidup memang membuka
    .bin yang sama (lewat /proc/<pid>/fd, bukan env: action_module diam-diam
    pindah ke berkas cadangan kalau OP3_ACTION_FILE gagal dibuka).
"""

import os

from . import action_bin as ab
from . import imu_metrics
from .action_knobs import RAW_PER_DEG, SIDE_TEXT, apply_knobs, clamped_joints, page_diff, page_phases, \
    phase_key, split_key

HARD_DEG = 15.0            # perubahan sudut terbesar dari page asli, derajat
HARD_TIME = (0.6, 1.4)     # pengali time + pause terkecil / terbesar

# Rentang usulan bawaan (operator bisa melebarkannya sampai batas keras).
# ALPHONSE memakai B +-10 deg dan waktu 0,7..1,3. CHRONUS lebih hati-hati: servo 17
# reset daya saat lutut diperintah melompat 20-30 deg per siklus (z_move 0,035,
# 2026-10-27), dan lutut pernah melatch OVERLOAD lalu mematikan torsinya sendiri.
# Ayunan tendangan yang lebih cepat dan lebih jauh menarik arus yang sama.
SPREAD = {
    "support_deg": 4.0,          # A: kaki tumpu selama satu kaki
    "kick_deg": 6.0,             # B: pinggul/lutut/engkel kaki tendang
    "yaw_deg": 4.0,              # B: arah telapak
    "strike_time": (0.8, 1.2),   # B: waktu step menendang
    "return_time": (0.8, 1.0),   # C: waktu kembali ke posisi siap (hanya dipercepat)
    "getup_time": (0.85, 1.15),  # bangun: kecepatan seluruh page
}

_PART = {"hip_yaw": "pinggul yaw", "hip_roll": "pinggul roll", "hip_pitch": "pinggul pitch", "knee": "lutut",
         "ank_pitch": "engkel pitch", "ank_roll": "engkel roll", "sho_pitch": "bahu pitch", "sho_roll": "bahu roll",
         "el": "siku"}
JOINT_LABELS = {joint: ("kepala " + joint[5:] if joint.startswith("head_") else
                        "%s %s" % (_PART[joint[2:]], SIDE_TEXT[joint[0]]))
                for joint in ab.JOINT_IDS}


# ---- .bin yang dibuka op3_manager --------------------------------------------------

def manager_bin_status(bin_path, proc_root="/proc", process_name="op3_manager"):
    """Apakah op3_manager yang hidup membuka berkas yang sama dengan bin_path?
    -> {"state": "sama" | "beda" | "tidak_diketahui", "open": [path...], "reason": teks}.
    Dibandingkan per inode, jadi path container/host yang berbeda tetap dikenali, dan
    .bin yang sudah diganti lewat rename (op3_manager masih memegang inode lama) = "beda"."""
    try:
        target = os.stat(bin_path)
    except OSError as exc:
        return {"state": "tidak_diketahui", "open": [], "reason": "bin tidak terbaca: %s" % exc}
    try:
        pids = [p for p in os.listdir(proc_root) if p.isdigit()]
    except OSError:
        return {"state": "tidak_diketahui", "open": [], "reason": "%s tidak terbaca" % proc_root}
    seen, found = False, []
    for pid in pids:
        base = os.path.join(proc_root, pid)
        try:
            with open(os.path.join(base, "comm"), encoding="utf-8") as handle:
                if handle.read().strip() != process_name:
                    continue
            fds = os.listdir(os.path.join(base, "fd"))
        except OSError:
            continue
        seen = True
        for fd in fds:
            path = os.path.join(base, "fd", fd)
            try:
                link = os.readlink(path)
                if ".bin" not in link:
                    continue
                stat = os.stat(path)
            except OSError:
                continue
            found.append((link, (stat.st_dev, stat.st_ino) == (target.st_dev, target.st_ino)))
    if not found:
        reason = "op3_manager tidak membuka .bin yang terlihat" if seen else "op3_manager tidak ditemukan"
        return {"state": "tidak_diketahui", "open": [], "reason": reason}
    same = any(match for _, match in found)
    return {"state": "sama" if same else "beda", "open": [link for link, _ in found],
            "reason": "" if same else "op3_manager membuka berkas lain (atau .bin sudah diganti lewat rename)"}


# ---- usulan variabel ----------------------------------------------------------------

def _steps_text(steps):
    return "step %d" % steps[0] if steps[0] == steps[1] else "step %d-%d" % steps


def _angle_var(joint, steps, group, why, spread):
    return {"key": phase_key(joint, steps), "label": "%s · %s" % (JOINT_LABELS[joint], _steps_text(steps)),
            "group": group, "why": why, "unit": "deg", "neutral": 0.0, "step": 0.5, "lo": -spread, "hi": spread,
            "hard_lo": -HARD_DEG, "hard_hi": HARD_DEG}


def _time_var(steps, group, why, spread):
    return {"key": phase_key("time_scale", steps), "label": "waktu " + _steps_text(steps), "group": group,
            "why": why, "unit": "x", "neutral": 1.0, "step": 0.05, "lo": spread[0], "hi": spread[1],
            "hard_lo": HARD_TIME[0], "hard_hi": HARD_TIME[1]}


def _valid_in(page, joint, steps):
    joint_id = ab.JOINT_IDS[joint]
    return all(ab.position_valid(page["steps"][n - 1]["positions"][joint_id])
               for n in range(steps[0], steps[1] + 1))


def _single_leg_variables(page, phases):
    """Tendangan (dan gerakan satu kaki lain): A tidak jatuh, B kena/lurus/jauh, C cepat
    kembali. Step pertama dan terakhir tidak diubah posisinya: sambungan ke/dari page 2."""
    n = len(page["steps"])
    kick, support = phases["kick"], phases["support"]
    first, last = phases["phases"]["satu_kaki"]
    span = (max(first, 2), min(last, n - 1))
    out = []
    if span[0] <= span[1]:
        for part in ("ank_roll", "hip_roll", "ank_pitch"):
            joint = "%s_%s" % (support, part)
            if _valid_in(page, joint, span):
                out.append(_angle_var(joint, span, "A", "kaki tumpu menahan badan selama satu kaki",
                                      SPREAD["support_deg"]))
    hip = ab.JOINT_IDS[kick + "_hip_pitch"]

    def swing(number):
        a, b = page["steps"][number - 2]["positions"][hip], page["steps"][number - 1]["positions"][hip]
        return abs(b - a) if ab.position_valid(a) and ab.position_valid(b) else 0

    strike = max(range(span[0], span[1] + 1), key=swing) if span[0] <= span[1] else None
    if strike is not None and swing(strike):
        back = (strike - 1, strike - 1)
        if back[0] >= 2 and _valid_in(page, kick + "_hip_pitch", back):
            out.append(_angle_var(kick + "_hip_pitch", back, "B", "ayunan ke belakang sebelum menendang",
                                  SPREAD["kick_deg"]))
        hit = (strike, strike)
        for part, spread, why in (("hip_pitch", SPREAD["kick_deg"], "ayunan ke depan saat menendang"),
                                  ("knee", SPREAD["kick_deg"], "lutut saat kaki mengenai bola"),
                                  ("ank_pitch", SPREAD["kick_deg"], "sudut telapak saat mengenai bola"),
                                  ("hip_yaw", SPREAD["yaw_deg"], "arah telapak (bola lurus atau melenceng)")):
            joint = "%s_%s" % (kick, part)
            if _valid_in(page, joint, hit):
                out.append(_angle_var(joint, hit, "B", why, spread))
        if page["steps"][strike - 1]["time"]:
            out.append(_time_var(hit, "B", "kecepatan tendangan (lebih kecil = lebih cepat)", SPREAD["strike_time"]))
    if last < n:
        out.append(_time_var((last + 1, n), "C", "waktu kembali ke posisi siap", SPREAD["return_time"]))
    return out


def propose_variables(page, profile):
    """(variabel usulan, fase page). Daftar kosong = operator menambah variabel sendiri."""
    phases = page_phases(page, page["name"])
    n = len(page["steps"])
    if profile == "getup":
        if n >= 2 and any(s["time"] for s in page["steps"]):
            return [_time_var((1, n), "A", "kecepatan seluruh gerakan bangun", SPREAD["getup_time"])], phases
        return [], phases
    if profile in ("kick", "umum") and phases and phases["kick"] and n >= 3:
        return _single_leg_variables(page, phases), phases
    return [], phases


# ---- layanan ------------------------------------------------------------------------

def _position(value):
    if value & ab.INVALID_BIT:
        return "invalid"
    return "torque_off" if value & ab.TORQUE_OFF_BIT else value


class ActionTrainer:
    def __init__(self, bin_path, op_demo_h=None, backup_dir=None, proc_root="/proc"):
        self.bin_path = bin_path
        self.op_demo_h = op_demo_h
        self.backup_dir = backup_dir or os.path.join(os.path.dirname(bin_path), "backup")
        self.proc_root = proc_root

    def _data(self):
        return ab.read_bin(self.bin_path)

    def _profile(self, page, names):
        return ab.guess_profile(page, names.get(page["index"], ""))

    def _scratch_state(self, data):
        page = ab.decode_page(data, ab.TRAINING_SCRATCH_PAGE)
        if page["name"] == ab.SCRATCH_NAME:
            return "kandidat"
        return "berisi gerakan %r" % page["name"] if page["stepnum"] else "kosong"

    def page_list(self):
        """Page berisi gerakan, tendangan duluan, plus catatan enum Motion_Index yang kosong."""
        data = self._data()
        names = ab.motion_index(self.op_demo_h)
        pages, free = [], None
        for index in range(1, ab.MAX_PAGE):
            if index in (ab.STUDIO_SCRATCH_PAGE, ab.TRAINING_SCRATCH_PAGE):
                continue
            page = ab.decode_page(data, index)
            if not ab.page_used(page):
                free = free or index
                continue
            if not page["stepnum"]:
                continue
            profile, _ = self._profile(page, names)
            pages.append({"index": index, "name": page["name"], "steps": page["stepnum"], "motion": names.get(index),
                          "profile": profile, "seconds": round(ab.page_seconds(page), 2)})
        pages.sort(key=lambda p: (p["profile"] != "kick", p["index"]))
        notes = ["Motion_Index %s = page %d kosong di .bin ini" % (name, index)
                 for index, name in sorted(names.items()) if not ab.decode_page(data, index)["stepnum"]]
        return {"bin": self.bin_path, "pages": pages, "free": free, "scratch": self._scratch_state(data),
                "manager": manager_bin_status(self.bin_path, self.proc_root), "notes": notes}

    def page_detail(self, index):
        data = self._data()
        index = int(index)
        if not 1 <= index < ab.MAX_PAGE:
            raise ValueError("nomor page %d di luar 1..%d" % (index, ab.MAX_PAGE - 1))
        names = ab.motion_index(self.op_demo_h)
        errors, warnings = ab.check_page(data, index)
        page = ab.decode_page(data, index)
        profile, profile_reason = self._profile(page, names)
        variables, phases = propose_variables(page, profile) if not errors else ([], None)
        chain, _ = ab.page_chain(data, index)
        return {
            "bin": self.bin_path, "index": index, "name": page["name"], "sha": ab.page_sha(page),
            "motion": names.get(index), "profile": profile, "profile_reason": profile_reason,
            "repeat": page["repeat"], "speed": page["speed"], "accel": page["accel"], "next": page["next"],
            "chain": chain, "planned_s": round(ab.chain_seconds(data, chain), 2),
            "steps": [{"time": s["time"], "pause": s["pause"], "seconds": round(ab.step_seconds(page, s), 2),
                       "positions": {j: _position(s["positions"][i]) for j, i in ab.JOINT_IDS.items()}}
                      for s in page["steps"]],
            "phases": phases, "variables": variables, "errors": errors, "warnings": warnings,
            "joint_labels": JOINT_LABELS, "hard": {"deg": HARD_DEG, "time": list(HARD_TIME)},
            "scratch": self._scratch_state(data),
        }

    @staticmethod
    def check_values(page, values):
        """Galat kunci/nilai variabel (daftar kosong = sah)."""
        errors = []
        count = len(page["steps"])
        for key, value in values.items():
            try:
                name, steps = split_key(key)
                value = float(value)
            except (TypeError, ValueError):
                errors.append("variabel %r tidak sah" % (key,))
                continue
            if steps is None or not 1 <= steps[0] <= steps[1] <= count:
                errors.append("%s: step harus 1..%d" % (key, count))
            elif name == "time_scale":
                if not HARD_TIME[0] <= value <= HARD_TIME[1]:
                    errors.append("%s = %g di luar %g..%g" % (key, value, HARD_TIME[0], HARD_TIME[1]))
            elif name in ab.JOINT_IDS:
                if abs(value) > HARD_DEG:
                    errors.append("%s = %+g deg melewati batas +-%g deg dari page asli" % (key, value, HARD_DEG))
            else:
                errors.append("%s: bukan nama sendi atau time_scale" % key)
        return errors

    def candidate(self, data, index, sha, values):
        """(page asli, page kandidat, beda, sendi terjepit). Page yang berubah sejak dibuka
        ditolak: kandidat hanya sebanding dengan isi page yang sama."""
        index = int(index)
        errors, _ = ab.check_page(data, index)
        if errors:
            raise ValueError("page %d tidak bisa dilatih: %s" % (index, "; ".join(errors)))
        page = ab.decode_page(data, index)
        if ab.page_sha(page) != sha:
            raise ValueError("page %d sudah berubah sejak dibuka (sha %s, sekarang %s) -- buka ulang page itu"
                             % (index, sha, ab.page_sha(page)))
        errors = self.check_values(page, values)
        if errors:
            raise ValueError("; ".join(errors))
        knobs = {key: float(value) for key, value in values.items()}
        tuned = apply_knobs(page, knobs)
        clamped = ["step %d %s" % hit for hit in clamped_joints(page, knobs)]
        return page, tuned, page_diff(page, tuned), clamped

    def load(self, index, sha, values):
        """Kandidat ke page kerja 254 (dibaca balik). Page asli tidak disentuh."""
        manager = manager_bin_status(self.bin_path, self.proc_root)
        if manager["state"] == "beda":
            raise ValueError("op3_manager memutar %s, bukan %s -- kandidat tidak akan terlihat. %s"
                             % (", ".join(manager["open"]), self.bin_path, manager["reason"]))
        data = self._data()
        page, tuned, diff, clamped = self.candidate(data, index, sha, values)
        scratch = ab.decode_page(data, ab.TRAINING_SCRATCH_PAGE)
        if scratch["stepnum"] and scratch["name"] != ab.SCRATCH_NAME:
            raise ValueError("page %d berisi gerakan %r, bukan page kerja -- tidak ditimpa"
                             % (ab.TRAINING_SCRATCH_PAGE, scratch["name"]))
        ab.write_page(self.bin_path, ab.TRAINING_SCRATCH_PAGE, ab.encode_page(tuned, name=ab.SCRATCH_NAME))
        chain, _ = ab.page_chain(data, page["index"])
        planned = ab.page_seconds(tuned) + ab.chain_seconds(data, chain[1:])
        return {"scratch": ab.TRAINING_SCRATCH_PAGE, "diff": diff, "clamped": clamped,
                "planned_s": round(planned, 2), "manager": manager}

    def restore(self):
        """Page kerja 254 kembali kosong -- hanya kalau isinya kandidat (nama train_tmp)."""
        data = self._data()
        if ab.decode_page(data, ab.TRAINING_SCRATCH_PAGE)["name"] != ab.SCRATCH_NAME:
            return {"restored": False}
        ab.write_page(self.bin_path, ab.TRAINING_SCRATCH_PAGE, ab.empty_page(data))
        return {"restored": True}

    @staticmethod
    def trial_metrics(samples, marks, profile):
        return {"metrics": imu_metrics.trial_metrics(samples, marks, profile)}

    def deploy(self, index, sha, values, target, name=None, overwrite=False):
        """Hasil ke page tujuan: cadangan utuh .bin dulu, lalu tulis + baca balik. Page tujuan
        yang berisi gerakan (termasuk page asli sendiri) hanya ditimpa kalau overwrite."""
        data = self._data()
        page, tuned, diff, _ = self.candidate(data, index, sha, values)
        target = int(target)
        if not 1 <= target < ab.MAX_PAGE or target in (ab.STUDIO_SCRATCH_PAGE, ab.TRAINING_SCRATCH_PAGE):
            raise ValueError("page tujuan %d tidak boleh (1..%d, bukan page kerja %d/%d)"
                             % (target, ab.MAX_PAGE - 1, ab.STUDIO_SCRATCH_PAGE, ab.TRAINING_SCRATCH_PAGE))
        existing = ab.decode_page(data, target)
        if ab.page_used(existing) and not overwrite:
            raise ValueError("page %d berisi %r -- centang 'timpa' untuk menimpanya" % (target, existing["name"]))
        if name is not None and not (1 <= len(name) <= 13 and all(" " <= c <= "~" for c in name)):
            raise ValueError("nama page %r tidak sah (1..13 karakter ASCII)" % name)
        page_bytes = ab.encode_page(tuned, name=name)
        modified = bytearray(data)
        modified[target * ab.PAGE_SIZE:(target + 1) * ab.PAGE_SIZE] = page_bytes
        errors, _ = ab.check_page(bytes(modified), target)
        if errors:
            raise ValueError("hasil tidak sah: " + "; ".join(errors))
        backup = ab.backup_bin(self.bin_path, self.backup_dir)
        ab.write_page(self.bin_path, target, page_bytes)
        notes = []
        if target == ab.WALKING_READY_PAGE:
            notes.append("page 2 = pose tangkapan walking_module: aktifkan ulang walking (Init Pose dulu) "
                         "supaya stance ikut isi baru")
        return {"target": target, "name": ab.decode_page(page_bytes, 0)["name"], "backup": backup,
                "changed": len(diff), "notes": notes, "raw_per_deg": RAW_PER_DEG}
