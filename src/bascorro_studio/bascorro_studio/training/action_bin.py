"""Format berkas action OP3 (.bin) -- MURNI, tanpa ROS.

Tata letak sama dengan op3_action_module (action_file_define.h):
  256 page x 512 byte. Header 64 byte: nama [0:14], repeat [15], schedule [16],
  stepnum [20], speed [22], accel [24], next [25], exit [26], checksum [31],
  pgain [32:63]. Step mulai byte 64, 7 step x 64 byte = 31 posisi uint16
  (indeks = ID servo) + pause + time. Posisi bertanda INVALID / TORQUE_OFF bukan
  sudut dan tidak pernah diubah.

PENTING (CHRONUS): action_module membuka .bin SEKALI (fopen "r+b") lalu
fseek+fread setiap page diputar. Page harus ditulis DI TEMPAT (inode sama):
berkas yang diganti lewat rename tetap tidak terlihat oleh op3_manager yang
sedang hidup -- ia terus membaca inode lama. write_page() menulis di tempat.
"""

import collections
import datetime
import hashlib
import os
import re
import shutil
import struct

PAGE_SIZE = 512
MAX_PAGE = 256
MAX_STEP = 7
MAX_JOINTS = 31
STEP_OFFSET = 64
STEP_SIZE = 64
INVALID_BIT = 0x4000
TORQUE_OFF_BIT = 0x2000
TIME_BASE_SCHEDULE = 0x0A
CONTROL_CYCLE_S = 0.008          # satu unit waktu action_module = satu siklus kontrol

# Page kerja: Studio memutar "Run step/range" lewat 250 (ActionEditor.jsx,
# "scratch_range" di .bin CHRONUS); training memutar kandidat lewat 254. Nama
# kandidat sama dengan ALPHONSE, supaya sisa salah satunya dikenali yang lain.
STUDIO_SCRATCH_PAGE = 250
TRAINING_SCRATCH_PAGE = 254
SCRATCH_NAME = "train_tmp"
WALKING_READY_PAGE = 2

JOINT_NAMES = {
    1: "r_sho_pitch", 2: "l_sho_pitch", 3: "r_sho_roll", 4: "l_sho_roll",
    5: "r_el", 6: "l_el", 7: "r_hip_yaw", 8: "l_hip_yaw",
    9: "r_hip_roll", 10: "l_hip_roll", 11: "r_hip_pitch", 12: "l_hip_pitch",
    13: "r_knee", 14: "l_knee", 15: "r_ank_pitch", 16: "l_ank_pitch",
    17: "r_ank_roll", 18: "l_ank_roll", 19: "head_pan", 20: "head_tilt",
}
JOINT_IDS = {name: joint_id for joint_id, name in JOINT_NAMES.items()}
LEG_JOINTS = [
    "r_hip_yaw", "r_hip_roll", "r_hip_pitch", "r_knee", "r_ank_pitch", "r_ank_roll",
    "l_hip_yaw", "l_hip_roll", "l_hip_pitch", "l_knee", "l_ank_pitch", "l_ank_roll",
]

# Enum Motion_Index yang BUKAN nomor page: ForGrass ditambahkan ke nomor
# tendangan/bangun saat lapangan rumput (SoccerDemo::playMotion).
MOTION_INDEX_OFFSETS = ("ForGrass",)


def read_bin(path):
    with open(path, "rb") as handle:
        data = handle.read()
    if len(data) != MAX_PAGE * PAGE_SIZE:
        raise ValueError("%s bukan berkas action OP3 (%d byte, seharusnya %d)"
                         % (path, len(data), MAX_PAGE * PAGE_SIZE))
    return data


def decode_page(data, index):
    base = index * PAGE_SIZE
    raw = bytes(data[base:base + PAGE_SIZE])
    page = {
        "index": index,
        "name": raw[:14].split(b"\0")[0].decode("ascii", "replace").strip(),
        "repeat": raw[15], "schedule": raw[16], "stepnum": raw[20], "speed": raw[22], "accel": raw[24],
        "next": raw[25], "exit": raw[26],
        "checksum_ok": (sum(raw) & 0xFF) == 0xFF,
        "raw": raw,
        "steps": [],
    }
    for step in range(min(page["stepnum"], MAX_STEP)):
        values = struct.unpack_from("<31HBB", raw, STEP_OFFSET + step * STEP_SIZE)
        page["steps"].append({"positions": list(values[:MAX_JOINTS]), "pause": values[MAX_JOINTS],
                              "time": values[MAX_JOINTS + 1]})
    return page


def encode_page(page, name=None):
    """Page dict -> 512 byte. Byte lain (dan nama, kecuali `name` diberikan) dari
    page["raw"]; checksum dihitung ulang seperti ActionModule::setChecksum."""
    out = bytearray(page["raw"])
    if name is not None:
        encoded = name.encode("ascii", "ignore")[:13]
        out[0:14] = encoded + b"\0" * (14 - len(encoded))
    for offset, key in ((15, "repeat"), (16, "schedule"), (20, "stepnum"), (22, "speed"), (24, "accel"),
                        (25, "next"), (26, "exit")):
        out[offset] = int(page[key]) & 0xFF
    for step, content in enumerate(page["steps"]):
        struct.pack_into("<31HBB", out, STEP_OFFSET + step * STEP_SIZE, *content["positions"],
                         int(content["pause"]) & 0xFF, int(content["time"]) & 0xFF)
    out[31] = 0
    out[31] = (0xFF - (sum(out) & 0xFF)) & 0xFF
    return bytes(out)


def page_sha(page):
    return hashlib.sha1(page["raw"]).hexdigest()[:12]


def position_valid(value):
    return not value & (INVALID_BIT | TORQUE_OFF_BIT)


def page_used(page):
    return bool(page["name"]) or page["stepnum"] > 0


def step_seconds(page, step):
    """Durasi satu step menurut jadwal waktu action_module: gerak time*speed/32
    siklus, jeda pause*32/speed siklus, 1 siklus = 8 ms."""
    speed = max(1, page["speed"])
    move = max(1, (step["time"] * speed) >> 5)
    pause = (step["pause"] << 5) // speed
    return (move + pause) * CONTROL_CYCLE_S


def page_seconds(page):
    return max(1, page["repeat"]) * sum(step_seconds(page, s) for s in page["steps"])


def page_chain(data, index, limit=16):
    """Urutan page yang diputar (mengikuti `next`) -> (daftar, berputar?)."""
    chain = [index]
    while True:
        following = decode_page(data, chain[-1])["next"]
        if following == 0:
            return chain, False
        if following in chain or len(chain) >= limit:
            return chain + [following], True
        chain.append(following)


def chain_seconds(data, chain):
    return sum(page_seconds(decode_page(data, i)) for i in chain)


def empty_page(data):
    """Page kosong persis seperti di .bin ini: yang paling sering muncul di antara
    page tanpa nama, tanpa step, dengan checksum sah (di CHRONUS 152 page identik)."""
    counts = collections.Counter()
    for index in range(1, MAX_PAGE):
        page = decode_page(data, index)
        if not page["name"] and page["stepnum"] == 0 and page["checksum_ok"]:
            counts[page["raw"]] += 1
    if not counts:
        raise ValueError("tidak ada page kosong di .bin sebagai contoh")
    return counts.most_common(1)[0][0]


def check_page(data, index):
    """(galat, peringatan) untuk dilatih. Galat = page tidak bisa dipakai sama sekali."""
    errors, warnings = [], []
    if not 1 <= index < MAX_PAGE:
        return ["nomor page harus 1..%d" % (MAX_PAGE - 1)], warnings
    if index in (STUDIO_SCRATCH_PAGE, TRAINING_SCRATCH_PAGE):
        return ["page %d adalah page kerja (Studio %d, training %d), bukan gerakan"
                % (index, STUDIO_SCRATCH_PAGE, TRAINING_SCRATCH_PAGE)], warnings
    page = decode_page(data, index)
    if not page["checksum_ok"]:
        errors.append("checksum page rusak -- simpan ulang page ini di Action Editor")
    if not 1 <= page["stepnum"] <= MAX_STEP:
        errors.append("page kosong (stepnum %d) -- tidak ada gerakan" % page["stepnum"])
    if page["repeat"] == 0:
        errors.append("repeat 0: action_module menolaknya ('has no action')")
    legs = [JOINT_IDS[j] for j in LEG_JOINTS]
    if page["steps"] and not any(position_valid(s["positions"][i]) for s in page["steps"] for i in legs):
        errors.append("tidak ada posisi sendi kaki yang sah di page ini")
    if page["schedule"] != TIME_BASE_SCHEDULE:
        warnings.append("jadwal speed-base: perkiraan durasi kasar")
    if any(s["time"] == 0 for s in page["steps"]):
        warnings.append("ada step dengan time 0 (gerak secepat mungkin)")
    chain, loops = page_chain(data, index)
    if loops:
        errors.append("rantai next berputar (%s) -- gerakan tidak pernah selesai sendiri"
                      % " -> ".join(str(i) for i in chain))
    for following in chain[1:]:
        nxt = decode_page(data, following)
        if not nxt["checksum_ok"] or nxt["stepnum"] == 0:
            warnings.append("page lanjutan %d kosong/rusak: rantai berhenti di situ" % following)
    return errors, warnings


def write_page(path, index, page_bytes):
    """Tulis SATU page di tempat (inode sama, lihat docstring modul), lalu baca balik."""
    if len(page_bytes) != PAGE_SIZE or not 1 <= index < MAX_PAGE:
        raise ValueError("page %d / %d byte tidak sah" % (index, len(page_bytes)))
    with open(path, "r+b") as handle:
        handle.seek(index * PAGE_SIZE)
        handle.write(page_bytes)
        handle.flush()
        os.fsync(handle.fileno())
    with open(path, "rb") as handle:
        handle.seek(index * PAGE_SIZE)
        if handle.read(PAGE_SIZE) != page_bytes:
            raise OSError("page %d tidak terbaca balik sama di %s" % (index, path))


def backup_bin(path, backup_dir):
    """Salinan utuh .bin sebelum menulis hasil: <backup_dir>/<nama>.<waktu>.bin."""
    os.makedirs(backup_dir, exist_ok=True)
    stem = os.path.splitext(os.path.basename(path))[0]
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    target = os.path.join(backup_dir, "%s.%s.bin" % (stem, stamp))
    counter = 1
    while os.path.exists(target):
        target = os.path.join(backup_dir, "%s.%s-%d.bin" % (stem, stamp, counter))
        counter += 1
    shutil.copy2(path, target)
    return target


def motion_index(op_demo_h):
    """{page: nama} dari enum Motion_Index (op3_demo/op_demo.h), tanpa offset ForGrass.

    CHRONUS: enum ini TIDAK sejalan dengan .bin (WalkingReady = 8 padahal page 8 =
    TEND_KIRI; RightKick 218 / LeftKick 104 kosong). Jadi nama ini hanya keterangan
    "dipakai demo sebagai ...", bukan dasar menebak jenis gerakan."""
    try:
        with open(op_demo_h, encoding="utf-8") as handle:
            text = handle.read()
    except (OSError, TypeError):
        return {}
    body = re.search(r"enum\s+Motion_Index\s*\{(.*?)\}", text, re.S)
    if not body:
        return {}
    code = re.sub(r"//[^\n]*", "", body.group(1))
    return {int(number): name for name, number in re.findall(r"(\w+)\s*=\s*(\d+)", code)
            if name not in MOTION_INDEX_OFFSETS}


_PROFILE_PATTERNS = (
    ("kick", r"kick|tend|nendang|umpan|shoot"),
    ("getup", r"get_?up|bang|bangun|tangi"),
    ("pose", r"init|ready|pose|tahan|nahan|block"),     # "menahan bola" (page 14, D58)
)


def guess_profile(page, motion_name=""):
    """(profil, alasan) untuk metrik dan penilaian: kick / getup / pose / umum.
    NAMA PAGE dulu (isi .bin yang sebenarnya), Motion_Index hanya cadangan."""
    for text, source in ((page["name"], "nama page %r" % page["name"]),
                         (motion_name, "nama Motion_Index %r" % motion_name)):
        lowered = (text or "").lower()
        if not lowered:
            continue
        for profile, pattern in _PROFILE_PATTERNS:
            if re.search(pattern, lowered):
                return profile, source
    if page["stepnum"] == 1:
        return "pose", "satu step (pose diam)"
    return "umum", "tidak dikenali dari nama"
