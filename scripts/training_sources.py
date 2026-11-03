"""Sumber data menu Training: versi Bascorro Studio dan page Action Editor (.bin).

MURNI (tanpa ROS), dipakai scripts/training.py (menu), autotune_action.py, dan
training_results.py. Semua daftar dibaca ULANG dari berkasnya setiap kali
dipanggil -- tidak ada daftar preset/page yang ditulis di kode:

  * Studio : src/bascorro_studio/data/walking_versions.json (berkas yang sama
             yang dibaca/ditulis server Studio, urutannya sama dengan dropdown).
  * Action : .bin yang dipakai action_module yang SEDANG hidup (env
             OP3_ACTION_FILE milik op3_manager), jatuh ke ALPHONSE.bin.
             Nama gerakan (WalkingReady = 2, RightKick = 105, ...) dari
             enum Motion_Index di op3_demo/op_demo.h -- sumber utamanya sendiri.
"""

import datetime
import hashlib
import json
import os
import re
import socket
import struct
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sync_ini_pose_from_page2 import (  # noqa: E402  (tata letak .bin yang sama)
    DEFAULT_BIN, HERE, INVALID_BIT, JOINT_NAMES, PAGE_SIZE, STEP_OFFSET, STEP_SIZE, TORQUE_OFF_BIT,
)
from walking_ready_fk import LEG_JOINTS, REFERENCE_JSON, STUDIO_VERSIONS  # noqa: E402

OP_DEMO_H = os.path.join(HERE, "src/ROBOTIS-OP3-Demo/op3_demo/include/op3_demo/op_demo.h")
DEMO_LAUNCH = os.path.join(HERE, "src/ROBOTIS-OP3-Demo/op3_demo/launch/demo.launch.xml")
MAX_PAGE = 256
MAX_STEP = 7
MAX_JOINTS = 31
TIME_BASE_SCHEDULE = 0x0A
CONTROL_CYCLE_S = 0.008          # satu "unit waktu" action_module = satu siklus kontrol
# Page kerja: Studio memutar "Run step/range" lewat page 250 (ActionEditor.jsx,
# scratchPageIndex), training memutar kandidat lewat 254. Keduanya bukan gerakan.
STUDIO_SCRATCH_PAGE = 250
TRAINING_SCRATCH_PAGE = 254
JOINT_IDS = {name: joint_id for joint_id, name in JOINT_NAMES.items()}


def local_time(ms_or_iso):
    """Waktu untuk ditampilkan: epoch ms (Studio) atau ISO UTC (sesi) -> waktu lokal."""
    if ms_or_iso in (None, ""):
        return "-"
    if isinstance(ms_or_iso, (int, float)):
        moment = datetime.datetime.fromtimestamp(ms_or_iso / 1000.0)
    else:
        moment = datetime.datetime.fromisoformat(str(ms_or_iso).replace("Z", "+00:00")).astimezone()
    return moment.strftime("%Y-%m-%d %H:%M:%S")


# ---- Bascorro Studio -------------------------------------------------------------

def load_studio_store(path=STUDIO_VERSIONS):
    with open(path, encoding="utf-8") as handle:
        store = json.load(handle)
    if store.get("format") != "bascorro-studio/walking-versions":
        raise SystemExit("%s bukan berkas versi Studio" % path)
    return store


def current_reference_id():
    with open(REFERENCE_JSON, encoding="utf-8") as handle:
        return json.load(handle)["id"]


def studio_versions(store):
    """Versi Studio dalam urutan dropdown Studio."""
    active = store.get("activeVersionId")
    return [{"id": v.get("id", ""), "name": v["name"], "updated_at": v.get("updatedAt") or v.get("createdAt"),
             "created_at": v.get("createdAt"), "active": v.get("id") == active, "params": v["params"]}
            for v in store.get("versions", [])]


def find_studio_version(store, key):
    for version in studio_versions(store):
        if key in (version["id"], version["name"]):
            return version
    return None


# ---- Action Editor (.bin) --------------------------------------------------------

def manager_pid():
    """pid op3_manager yang hidup (action_module & walking_module ada di proses itu), atau None."""
    try:
        pids = subprocess.run(["pgrep", "-f", "[o]p3_manager/lib/op3_manager/op3_manager"],
                              capture_output=True, text=True).stdout.split()
    except OSError:
        return None
    return int(pids[0]) if pids else None


def robot_name():
    """Robot tujuan untuk ditampilkan: BASCORRO_ROBOT_NAME, atau nama .bin bawaan (ALPHONSE.bin -> ALPHONSE)."""
    name = os.environ.get("BASCORRO_ROBOT_NAME") or os.path.splitext(os.path.basename(DEFAULT_BIN))[0]
    return "%s (komputer %s)" % (name, socket.gethostname())


def d60_presets(path=DEMO_LAUNCH):
    """({nama preset: [peran D60]}, berkas preset mendekat terpisah atau "") dari arg bawaan
    demo.launch.xml -- sumber yang sama dengan yang dibaca mode D60 saat sesi dimulai."""
    try:
        with open(path, encoding="utf-8") as handle:
            text = handle.read()
    except OSError:
        return {}, ""
    roles = {}
    for role, name in re.findall(r'<arg\s+name="auto_demo_preset_(\w+)"\s+default="([^"]*)"', text):
        if role != "approach_file":
            roles.setdefault(name, []).append(role)
    approach = re.search(r'<arg\s+name="auto_demo_preset_approach_file"\s+default="([^"]*)"', text)
    return roles, approach.group(1) if approach else ""


def _manager_env(name):
    """Nilai env milik op3_manager yang hidup (action_module ada di proses itu)."""
    try:
        pid = manager_pid()
        if not pid:
            return None
        with open("/proc/%s/environ" % pid, "rb") as handle:
            for entry in handle.read().decode("utf-8", "replace").split("\0"):
                key, _, value = entry.partition("=")
                if key == name:
                    return value
    except OSError:
        pass
    return None


def _local_path(path):
    """Path host (/home/.../motion_webots/src/...) dipetakan ke repo ini kalau perlu."""
    if not path or os.path.exists(path):
        return path
    marker = "/src/ROBOTIS-OP3/"
    if marker in path:
        return os.path.join(HERE, "src/ROBOTIS-OP3/" + path.split(marker, 1)[1])
    return path


def active_action_bin():
    """(.bin yang dipakai action_module sekarang, dari mana diketahui).
    BASCORRO_TRAINING_BIN menang atas semuanya -- dipakai uji dengan robot tiruan
    supaya page kerja tidak pernah ditulis ke .bin robot sungguhan."""
    for value, source in ((os.environ.get("BASCORRO_TRAINING_BIN"), "env BASCORRO_TRAINING_BIN (uji)"),
                          (_manager_env("OP3_ACTION_FILE"), "env op3_manager yang hidup"),
                          (os.environ.get("OP3_ACTION_FILE"), "env OP3_ACTION_FILE"),
                          (DEFAULT_BIN, "bawaan")):
        path = _local_path((value or "").strip())
        if path and os.path.isfile(path):
            return path, source
    return DEFAULT_BIN, "bawaan (berkas tidak ditemukan)"


def read_bin(path):
    with open(path, "rb") as handle:
        data = handle.read()
    if len(data) != MAX_PAGE * PAGE_SIZE:
        raise SystemExit("%s bukan berkas action OP3 (%d byte, seharusnya %d)" % (path, len(data), MAX_PAGE * PAGE_SIZE))
    return data


def motion_index(path=OP_DEMO_H):
    """{page: nama} dari enum Motion_Index (op_demo.h) -- nama yang dipakai demo."""
    try:
        with open(path, encoding="utf-8") as handle:
            text = handle.read()
    except OSError:
        return {}
    body = re.search(r"enum\s+Motion_Index\s*\{(.*?)\}", text, re.S)
    if not body:
        return {}
    code = re.sub(r"//[^\n]*", "", body.group(1))
    return {int(number): name for name, number in re.findall(r"(\w+)\s*=\s*(\d+)", code)}


def decode_page(data, index):
    base = index * PAGE_SIZE
    raw = bytes(data[base:base + PAGE_SIZE])
    page = {
        "index": index,
        "name": raw[:14].split(b"\0")[0].decode("ascii", "replace").strip(),
        # persis seperti action_yaml.py export (nama Action Editor sering berakhiran '\n')
        "name_raw": raw[:14].split(b"\0")[0].decode("ascii", "ignore"),
        "repeat": raw[15], "schedule": raw[16], "stepnum": raw[20], "speed": raw[22], "accel": raw[24],
        "next": raw[25], "exit": raw[26], "pgain": list(raw[32:63]),
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
    """Page dict -> 512 byte. Byte cadangan (dan nama, kecuali name diberikan) diambil
    dari page["raw"] -- nama asli Action Editor sering berakhiran '\\n'; checksum dihitung ulang."""
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


def step_seconds(page, step):
    """Durasi satu step persis seperti action_module::actionPlayProcess (jadwal waktu):
    gerak = time*speed/32 siklus, jeda = pause*32/speed siklus, 1 siklus = 8 ms."""
    speed = max(1, page["speed"])
    move = max(1, (step["time"] * speed) >> 5)
    pause = (step["pause"] << 5) // speed
    return (move + pause) * CONTROL_CYCLE_S


def page_seconds(page):
    return max(1, page["repeat"]) * sum(step_seconds(page, s) for s in page["steps"])


def page_chain(data, index, limit=16):
    """Urutan page yang diputar (ikut 'next'). Mengembalikan (daftar, berputar?)."""
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


def page_used(page):
    return bool(page["name"]) or page["stepnum"] > 0


def guess_profile(index, page, names):
    """Jenis gerakan untuk kriteria skor: (profil, alasan). Nama Motion_Index dulu,
    lalu nama page; operator selalu bisa memilih profil lain di menu."""
    label = names.get(index, "")
    for text, source in ((label, "nama Motion_Index %r" % label), (page["name"], "nama page %r" % page["name"])):
        lowered = text.lower()
        if not lowered:
            continue
        if re.search(r"kick|tend|nendang|umpan|shoot", lowered):
            return "kick", source
        if re.search(r"get_?up|bang|bangun|tangi", lowered):
            return "getup", source
        if re.search(r"init|ready|pose", lowered):
            return "pose", source
    if page["stepnum"] == 1:
        return "pose", "satu step (pose diam)"
    return "umum", "tidak dikenali dari nama"


def check_page(data, index):
    """(galat, peringatan) untuk dilatih. Galat = page tidak bisa dipakai sama sekali."""
    errors, warnings = [], []
    if not 1 <= index < MAX_PAGE:
        return ["nomor page harus 1..%d" % (MAX_PAGE - 1)], warnings
    if index in (STUDIO_SCRATCH_PAGE, TRAINING_SCRATCH_PAGE):
        return ["page %d adalah page kerja sementara (Studio %d, training %d), bukan gerakan"
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


def page_summary(data, index, names):
    page = decode_page(data, index)
    chain, loops = page_chain(data, index)
    profile, reason = guess_profile(index, page, names)
    return {"index": index, "name": page["name"], "motion_name": names.get(index, ""), "stepnum": page["stepnum"],
            "seconds": page_seconds(page), "chain": chain, "loops": loops,
            "chain_seconds": None if loops else chain_seconds(data, chain),
            "profile": profile, "profile_reason": reason, "sha": page_sha(page)}


def list_pages(data, names):
    """Page yang berisi gerakan: yang bernama di Motion_Index (dipakai demo) dulu."""
    used = [i for i in range(1, MAX_PAGE)
            if i not in (STUDIO_SCRATCH_PAGE, TRAINING_SCRATCH_PAGE) and decode_page(data, i)["stepnum"] > 0]
    used.sort(key=lambda i: (i not in names, i))
    return [page_summary(data, i, names) for i in used]


def page_to_yaml_entry(page, index=None, name=None):
    """Satu page dalam format YAML op3_action_editor/scripts/action_yaml.py
    (export/import), supaya hasil training bisa diimpor dengan alat yang sudah ada."""
    def key(joint_id):
        return JOINT_NAMES.get(joint_id, "id_%d" % joint_id)

    def position(value):
        if value & INVALID_BIT:
            return None
        if value & TORQUE_OFF_BIT:
            return "torque_off"
        return int(value)

    return {
        "index": page["index"] if index is None else index,
        "name": page["name_raw"] if name is None else name,
        "header": {
            "repeat": page["repeat"], "schedule": "time" if page["schedule"] == TIME_BASE_SCHEDULE else "speed",
            "stepnum": page["stepnum"], "speed": page["speed"], "accel": page["accel"],
            "next": page["next"], "exit": page["exit"],
            "pgain": {key(i): page["pgain"][i] for i in range(1, MAX_JOINTS)},
        },
        "steps": [{"index": n, "pause": s["pause"], "time": s["time"],
                   "positions": {key(i): position(s["positions"][i]) for i in range(1, MAX_JOINTS)}}
                  for n, s in enumerate(page["steps"])],
    }


def write_page(path, index, page_bytes):
    """Tulis SATU page (512 byte) di tempat, tanpa menyentuh page lain. action_module
    membuka ulang berkasnya setiap kali memutar page, jadi langsung terpakai."""
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
