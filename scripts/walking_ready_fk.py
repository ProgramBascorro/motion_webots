#!/usr/bin/env python3
"""Acuan Init Pose walking, diturunkan dari page 2 WALKING_READY di ALPHONSE.bin.

SATU SUMBER
-----------
Berkas yang ditulis skrip ini, src/bascorro_studio/data/walking_ready_reference.json,
dibaca langsung oleh:

  * Bascorro Studio  -- titik nol tampilan "Relatif page 2", nilai awal grup
                        Init Pose, dan acuan yang dipatok sebelum walking_module
                        menyala (web/src/walkingPoseReference.js)
  * demo D60         -- acuan yang sama sebelum walking_module menyala
                        (op3_demo/src/auto_demo/auto_demo.cpp)

dan disalin ke op3_walking_module/config/param.yaml (nilai boot walking_module),
karena yaml itu tidak bisa membaca berkas lain.

APA YANG DIHITUNG
-----------------
walking_module merekam bias saat dinyalakan:

    walking_bias_ = pose_terekam - neutral(acuan)
    pose robot    = page 2 + [ neutral(preset) - neutral(acuan) ]

neutral() = computeNeutralLegAngle(), IK kaki dengan offset init. Skrip ini
mencari acuan yang neutral()-nya SAMA dengan sendi kaki page 2, memakai IK
controller yang di-port 1:1 (op3_kinematics_dynamics.cpp
calcInverseKinematicsForLeg + arah sendi + koreksi hip_pitch_offset). Jadi
angka acuan = geometri page 2 yang sebenarnya, dan bias yang tersisa hanya
asimetri page 2 yang memang tidak bisa dinyatakan offset simetris.

  dicocokkan : init_x_offset, init_z_offset, init_pitch_offset
  dipegang   : init_roll_offset, init_yaw_offset, hip_pitch_offset
               (hip_pitch_offset dan pitch saling menggantikan -- ada satu
               keluarga solusi yang memberi sendi persis sama -- jadi salah satu
               harus dipatok; roll/yaw page 2 cuma asimetri kiri-kanan)
  init_y_offset tidak ikut: penangkapan bias memanggil neutral() tanpa y,
               jadi y selalu lebar TAMBAHAN di atas page 2, acuannya 0.

MIGRASI PRESET
--------------
Angka tersimpan (versi Studio, salinan preset demo) adalah nilai asli ROS.
Kalau acuan bergeser sebesar d dan preset tidak ikut, SEMUA preset berdiri
sejauh d dari page 2 -- pada pergantian 23 Okt 2026 itu 16 derajat. Jadi --write
menggeser keenam offset bias tiap versi sebesar d yang sama: angka "Relatif
page 2" di Studio tetap, pose berdiri tetap (selisih sisa karena IK tidak
linear ditampilkan per versi). Tiap berkas versi ditandai `poseReferenceId`;
berkas yang tandanya tidak cocok dengan acuan lama DITOLAK, supaya tidak ada
yang tergeser dua kali.

PAKAI
-----
    python3 scripts/walking_ready_fk.py            # lihat (dry run)
    python3 scripts/walking_ready_fk.py --check    # exit 1 kalau ada yang basi
    python3 scripts/walking_ready_fk.py --write    # tulis acuan + geser preset

Jalankan --write sesudah page 2 di-teach ulang, bersama
scripts/sync_ini_pose_from_page2.py --write. Bascorro Studio (pnpm dev) harus
DITUTUP dulu: tab yang terbuka masih memegang angka lama, dan Save berikutnya
akan menulis balik angka itu. Sesudahnya: jalankan ulang op3_manager
(param.yaml dibaca sekali saat boot) dan buka lagi Studio. Tidak perlu
colcon build.
"""

import argparse
import datetime
import glob
import hashlib
import json
import math
import os
import re
import shutil
import socket
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sync_ini_pose_from_page2 import (  # noqa: E402  (pembaca .bin yang sama)
    DEFAULT_BIN, DEFAULT_YAML as INI_POSE_YAML, HERE, INVALID_BIT, JOINT_NAMES, TORQUE_OFF_BIT,
    build_tar_pose_block, parse_existing_tar_pose, read_page_step0,
)

REFERENCE_JSON = os.path.join(HERE, "src/bascorro_studio/data/walking_ready_reference.json")
STUDIO_VERSIONS = os.path.join(HERE, "src/bascorro_studio/data/walking_versions.json")
DEMO_CONFIG_DIR = os.path.join(HERE, "src/ROBOTIS-OP3-Demo/op3_demo/config")
PARAM_YAML = os.path.join(HERE, "src/ROBOTIS-OP3/op3_walking_module/config/param.yaml")
# Bukan data/backup/: folder itu milik server Studio (sering jalan sebagai root
# di kontainer), jadi skrip yang dijalankan di host tidak bisa menulis ke sana.
BACKUP_DIR = os.path.join(HERE, "src/bascorro_studio/data/backup_walking_ready")
STUDIO_PORT = 5173

REFERENCE_FORMAT = "bascorro-studio/walking-ready-reference"
VERSIONS_FORMAT = "bascorro-studio/walking-versions"

LEG_JOINTS = [
    "r_hip_yaw", "r_hip_roll", "r_hip_pitch", "r_knee", "r_ank_pitch", "r_ank_roll",
    "l_hip_yaw", "l_hip_roll", "l_hip_pitch", "l_knee", "l_ank_pitch", "l_ank_roll",
]
# Urutan dan isi sama dengan WALKING_BIAS_FIELDS (web/src/walkingParams.js) dan
# kPoseReferenceFields (auto_demo.cpp).
BIAS_FIELDS = [
    "init_x_offset", "init_z_offset", "init_roll_offset",
    "init_pitch_offset", "init_yaw_offset", "hip_pitch_offset",
]
FITTED = ["init_x_offset", "init_z_offset", "init_pitch_offset"]
HELD = [k for k in BIAS_FIELDS if k not in FITTED]
# 0,1 mm dan 0,0001 rad (0,006 deg): jauh di bawah yang bisa dirasakan robot,
# dan angkanya tetap terbaca di kotak isian Studio.
DECIMALS = 4

# Acuan yang DI-HARDCODE di tiga tempat sampai 23 Okt 2026
# (walkingPoseReference.js, auto_demo.cpp kPoseReferenceDefaults, param.yaml).
# Bukan hasil FK -- itu isi param.yaml 3 September. Hanya dipakai --from-legacy,
# untuk migrasi pertama dari berkas versi yang belum bertanda.
LEGACY_REFERENCE = {
    "init_x_offset": -0.02, "init_z_offset": 0.0408, "init_roll_offset": 0.0,
    "init_pitch_offset": -0.07, "init_yaw_offset": 0.0, "hip_pitch_offset": 0.1396,
}
LEGACY_REFERENCE_ID = "legacy-param-yaml-2026-09-03"

# ---- IK walking_module, port 1:1 ---------------------------------------------
# op3_kinematics_dynamics.cpp:515-521 dan data tautan [13], [15], [31].
THIGH = math.hypot(0.0001, 0.11015)
CALF = 0.11
ANKLE = 0.0305
HIP_OFFSET_ANGLE = math.atan2(0.0001, 0.11015)
HIP_PITCH_OFFSET_M = 0.0001
LEG_LENGTH = THIGH + CALF + ANKLE  # computeNeutralLegAngle: thigh + calf + ankle
# getJointDirection() = jumlah komponen joint_axis_ tautan 7..18.
DIR_R = [-1, -1, -1, -1, 1, 1]
DIR_L = [-1, -1, 1, 1, -1, 1]


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return [[1, 0, 0], [0, c, -s], [0, s, c]]


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, 0, s], [0, 1, 0], [-s, 0, c]]


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]


def _mm(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _mv(a, v):
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def _t(a):
    return [[a[j][i] for j in range(3)] for i in range(3)]


def ik_leg(x, y, z, roll, pitch, yaw):
    """calcInverseKinematicsForLeg(); None = di luar jangkauan (penjaga NaN-nya)."""
    out = [0.0] * 6
    r06 = _mm(_mm(_rz(yaw), _ry(pitch)), _rx(roll))  # convertRPYToRotation
    p06 = [x + ANKLE * r06[0][2], y + ANKLE * r06[1][2], z + ANKLE * r06[2][2]]
    p60 = [-v for v in _mv(_t(r06), p06)]
    out[5] = math.atan2(p60[1], p60[2])
    r05 = _mm(r06, _rx(-out[5]))
    out[0] = math.atan2(-r05[0][1], r05[1][1])
    p03 = _mv(_rz(out[0]), [HIP_PITCH_OFFSET_M, 0.0, 0.0])
    p36 = [p06[i] - p03[i] for i in range(3)]
    norm = math.sqrt(sum(v * v for v in p36))
    knee_cos = (THIGH * THIGH + CALF * CALF - norm * norm) / (2 * THIGH * CALF)
    if not -1.0 <= knee_cos <= 1.0:
        return None
    out[3] = -math.acos(knee_cos) + math.pi
    alpha_sin = THIGH * math.sin(math.pi - out[3]) / norm
    if not -1.0 <= alpha_sin <= 1.0:
        return None
    alpha = math.asin(alpha_sin)
    p63 = [-v for v in _mv(_t(r06), p36)]
    sign = -1.0 if p63[2] < 0 else 1.0
    out[4] = -math.atan2(p63[0], sign * math.hypot(p63[1], p63[2])) - alpha
    r13 = _mm(_mm(_rz(-out[0]), r05), _ry(-(out[4] + out[3])))
    out[1] = math.atan2(r13[2][1], r13[1][1])
    out[2] = math.atan2(r13[0][2], r13[0][0])
    out[2] += HIP_OFFSET_ANGLE
    out[3] -= HIP_OFFSET_ANGLE
    return out


def neutral(p, with_y=False):
    """computeNeutralLegAngle(); 12 sudut [rad] urutan LEG_JOINTS, None = IK gagal."""
    y_ep = p.get("init_y_offset", 0.0) / 2 if with_y else 0.0
    z = p["init_z_offset"] - LEG_LENGTH
    right = ik_leg(p["init_x_offset"], -y_ep, z, -p["init_roll_offset"] / 2,
                   p["init_pitch_offset"], -p["init_yaw_offset"] / 2)
    left = ik_leg(p["init_x_offset"], y_ep, z, p["init_roll_offset"] / 2,
                  p["init_pitch_offset"], p["init_yaw_offset"] / 2)
    if right is None or left is None:
        return None
    q = [v * d for v, d in zip(right, DIR_R)] + [v * d for v, d in zip(left, DIR_L)]
    q[2] -= DIR_R[2] * p["hip_pitch_offset"]
    q[8] -= DIR_L[2] * p["hip_pitch_offset"]
    return q


def raw_to_rad(raw):
    """ActionModule::convertw4095ToRad -- nilai yang direkam walking_module."""
    return (raw - 2048) * math.pi / 2048.0


# ---- pencocokan -------------------------------------------------------------

def _solve3(a, b):
    """Eliminasi Gauss 3x3 dengan pivot."""
    m = [row[:] + [b[i]] for i, row in enumerate(a)]
    for c in range(3):
        p = max(range(c, 3), key=lambda r: abs(m[r][c]))
        m[c], m[p] = m[p], m[c]
        if abs(m[c][c]) < 1e-15:
            raise SystemExit("pencocokan FK gagal: matriks singular")
        for r in range(3):
            if r != c:
                f = m[r][c] / m[c][c]
                m[r] = [m[r][k] - f * m[c][k] for k in range(4)]
    return [m[i][3] / m[i][i] for i in range(3)]


def fit_reference(target, held, start):
    """Gauss-Newton atas FITTED supaya neutral() = sendi kaki page 2."""
    params = dict(held)
    values = [start[k] for k in FITTED]

    def residual(v):
        trial = dict(params, **dict(zip(FITTED, v)))
        q = neutral(trial)
        if q is None:
            return None
        return [q[i] - target[i] for i in range(12)]

    for _ in range(50):
        r = residual(values)
        if r is None:
            raise SystemExit("pencocokan FK gagal: IK di luar jangkauan dari titik awal")
        jac = []
        for j in range(3):
            h = 1e-6
            up = list(values)
            dn = list(values)
            up[j] += h
            dn[j] -= h
            ru, rd = residual(up), residual(dn)
            jac.append([(ru[i] - rd[i]) / (2 * h) for i in range(12)])
        jtj = [[sum(jac[a][i] * jac[b][i] for i in range(12)) for b in range(3)] for a in range(3)]
        jtr = [sum(jac[a][i] * r[i] for i in range(12)) for a in range(3)]
        step = _solve3(jtj, [-v for v in jtr])
        values = [values[i] + step[i] for i in range(3)]
        if max(abs(s) for s in step) < 1e-12:
            break
    return dict(params, **dict(zip(FITTED, values)))


def tidy(value):
    """Angka bersih seperti yang ditulis Studio: 0.0331, bukan 0.033100000000000004."""
    value = round(value, 10)
    return int(value) if value == int(value) else value


def page2_legs(bin_path, page):
    name, _, checksum_ok, positions = read_page_step0(bin_path, page)
    if not checksum_ok:
        raise SystemExit("checksum halaman %d RUSAK di %s -- perbaiki dulu berkasnya" % (page, bin_path))
    index = {n: i for i, n in JOINT_NAMES.items()}
    raw = {}
    for joint in LEG_JOINTS:
        value = positions[index[joint]]
        if value & (INVALID_BIT | TORQUE_OFF_BIT):
            raise SystemExit("halaman %d tidak memerintah %s (raw 0x%04X)" % (page, joint, value))
        raw[joint] = value
    return name, raw


def reference_id(raw, offsets):
    blob = json.dumps({"raw": raw, "offsets": offsets}, sort_keys=True).encode()
    return "page2-" + hashlib.sha1(blob).hexdigest()[:12]


def build_reference(bin_path, page, held):
    name, raw = page2_legs(bin_path, page)
    target = [raw_to_rad(raw[j]) for j in LEG_JOINTS]
    fitted = fit_reference(target, held, dict(LEGACY_REFERENCE, **held))
    offsets = {k: tidy(round(fitted[k], DECIMALS)) for k in BIAS_FIELDS}
    q = neutral(offsets)
    residual = {j: round(math.degrees(q[i] - target[i]), 2) for i, j in enumerate(LEG_JOINTS)}
    return {
        "format": REFERENCE_FORMAT,
        "formatVersion": 1,
        "id": reference_id(raw, offsets),
        "generatedAt": datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="milliseconds")
                       .replace("+00:00", "Z"),
        "generator": "scripts/walking_ready_fk.py",
        "source": {
            "actionFile": os.path.relpath(bin_path, HERE),
            "page": page,
            "pageName": name,
            "legRaw": raw,
        },
        "fitted": FITTED,
        "held": HELD,
        "offsets": offsets,
        "residualDeg": residual,
    }


# ---- berkas -----------------------------------------------------------------

def read_json(path):
    with open(path, encoding="utf-8") as handle:
        return json.load(handle)


def write_atomic(path, text):
    temporary = "%s.tmp-%d" % (path, os.getpid())
    with open(temporary, "w", encoding="utf-8") as handle:
        handle.write(text)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, path)


def backup(path, stamp):
    if not os.path.exists(path):
        return None
    base, ext = os.path.splitext(os.path.basename(path))
    target = os.path.join(BACKUP_DIR, "%s.%s%s" % (base, stamp, ext))
    shutil.copyfile(path, target)
    return target


def preflight(paths):
    """Semua target harus bisa ditulis SEBELUM satu pun disentuh -- berkas acuan
    yang sudah baru sementara preset masih lama adalah keadaan paling berbahaya
    yang bisa ditinggalkan skrip ini."""
    os.makedirs(BACKUP_DIR, exist_ok=True)
    for directory in sorted({BACKUP_DIR} | {os.path.dirname(p) for p in paths}):
        if not os.access(directory, os.W_OK | os.X_OK):
            raise SystemExit("tidak bisa menulis ke %s; tidak ada yang diubah" % directory)


def json_text(doc):
    # Sama dengan JSON.stringify(doc, null, 2) + "\n" di walkingVersionFile.js.
    return json.dumps(doc, indent=2, ensure_ascii=False) + "\n"


def versions_files():
    files = [STUDIO_VERSIONS]
    for path in sorted(glob.glob(os.path.join(DEMO_CONFIG_DIR, "*.json"))):
        try:
            if read_json(path).get("format") == VERSIONS_FORMAT:
                files.append(path)
        except (OSError, ValueError):
            continue
    return files


PARAM_KEYS = {  # kunci param.yaml -> field WalkingParam, satuan yaml
    "x_offset": ("init_x_offset", 1.0),
    "z_offset": ("init_z_offset", 1.0),
    "roll_offset": ("init_roll_offset", 180.0 / math.pi),
    "pitch_offset": ("init_pitch_offset", 180.0 / math.pi),
    "yaw_offset": ("init_yaw_offset", 180.0 / math.pi),
    "hip_pitch_offset": ("hip_pitch_offset", 180.0 / math.pi),
}


def param_yaml_values(text):
    out = {}
    for key in PARAM_KEYS:
        m = re.search(r"^%s:\s*([-+0-9.eE]+)\s*$" % key, text, re.M)
        if m:
            out[key] = float(m.group(1))
    return out


def _same(a, b):
    # Toleransi float32, sama dengan parameterValuesEqual() di walkingParams.js:
    # param.yaml yang ditulis "save" walking_module berisi 0.1396 sebagai
    # float32 (7.99849046622 deg), dan itu bukan angka yang berbeda.
    return abs(a - b) <= 1e-7 * max(1.0, abs(a), abs(b))


def param_yaml_with_reference(text, offsets):
    current = param_yaml_values(text)
    for key, (field, scale) in PARAM_KEYS.items():
        if key in current and _same(current[key] / scale, offsets[field]):
            continue  # dibiarkan bit-per-bit
        value = offsets[field] * scale
        literal = repr(tidy(value)) if scale == 1.0 else repr(value if value != 0 else 0)
        text, count = re.subn(r"^(%s:\s*)[-+0-9.eE]+\s*$" % key, lambda m: m.group(1) + literal, text, flags=re.M)
        if count != 1:
            raise SystemExit("param.yaml: kunci %s ditemukan %d kali, harus tepat sekali" % (key, count))
    return text


def param_yaml_matches(text, offsets):
    values = param_yaml_values(text)
    return all(key in values and _same(values[key] / scale, offsets[field])
               for key, (field, scale) in PARAM_KEYS.items())


def studio_running():
    try:
        with socket.create_connection(("127.0.0.1", STUDIO_PORT), timeout=0.3):
            return True
    except OSError:
        return False


# ---- laporan ----------------------------------------------------------------

def stance_shift_deg(params, old_ref, new_ref, delta):
    """Seberapa jauh pose berdiri satu versi bergeser oleh migrasi [deg, maks sendi]."""
    p_old = dict((k, params[k]) for k in BIAS_FIELDS)
    p_new = dict((k, params[k] + delta[k]) for k in BIAS_FIELDS)
    a, b, c, d = neutral(p_old), neutral(old_ref), neutral(p_new), neutral(new_ref)
    if None in (a, b, c, d):
        return None
    return max(abs((c[i] - d[i]) - (a[i] - b[i])) for i in range(12)) * 180 / math.pi


def reachable(params):
    """IK kaki sanggup di init pose ini, berdiri dan pada ujung satu langkah."""
    for sx in (0.0, params["x_move_amplitude"], -params["x_move_amplitude"]):
        trial = dict(params, init_x_offset=params["init_x_offset"] + sx)
        if neutral(trial, with_y=True) is None:
            return False
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bin", default=DEFAULT_BIN)
    ap.add_argument("--page", type=int, default=2)
    ap.add_argument("--write", action="store_true", help="tulis acuan, geser preset, segarkan param.yaml")
    ap.add_argument("--check", action="store_true", help="exit 1 kalau acuan/preset/param.yaml basi")
    ap.add_argument("--from-legacy", action="store_true",
                    help="migrasi pertama: berkas versi tanpa tanda dianggap memakai acuan lama 11 Sep")
    ap.add_argument("--hip-pitch-offset", type=float, help="ganti hip_pitch_offset yang dipegang [rad]")
    ap.add_argument("--allow-running-studio", action="store_true", help=argparse.SUPPRESS)
    args = ap.parse_args()

    current = read_json(REFERENCE_JSON) if os.path.exists(REFERENCE_JSON) else None
    if current is not None and current.get("format") != REFERENCE_FORMAT:
        raise SystemExit("%s bukan berkas acuan yang dikenali" % REFERENCE_JSON)
    if current is not None:
        old_ref, old_id = current["offsets"], current["id"]
    elif args.from_legacy or args.check:
        old_ref, old_id = LEGACY_REFERENCE, LEGACY_REFERENCE_ID
    else:
        old_ref, old_id = None, None

    held = {k: (old_ref or LEGACY_REFERENCE)[k] for k in HELD}
    if args.hip_pitch_offset is not None:
        held["hip_pitch_offset"] = args.hip_pitch_offset
    new = build_reference(args.bin, args.page, held)
    new_ref = new["offsets"]
    if current is not None and current["id"] == new["id"]:
        new["generatedAt"] = current["generatedAt"]  # tidak ada yang berubah, jangan bikin diff

    print("page %d %r dari %s" % (args.page, new["source"]["pageName"], new["source"]["actionFile"]))
    print("\nacuan (dicocokkan: %s; dipegang: %s)" % (", ".join(FITTED), ", ".join(HELD)))
    print("  %-20s %12s %12s %12s" % ("field", "sekarang", "page 2", "geser"))
    for key in BIAS_FIELDS:
        old = old_ref[key] if old_ref else float("nan")
        print("  %-20s %12.4f %12.4f %+12.4f" % (key, old, new_ref[key], new_ref[key] - old))
    print("  id: %s -> %s" % (old_id or "(belum ada)", new["id"]))
    print("\nsisa neutral(acuan) - page 2 [deg] (asimetri page 2, diserap bias):")
    print("  " + "  ".join("%s %+.2f" % (j, v) for j, v in new["residualDeg"].items()))

    delta = {k: new_ref[k] - old_ref[k] for k in BIAS_FIELDS} if old_ref else None
    stale = []

    # Berkas versi: tanda harus = acuan lama, lalu digeser sebesar delta.
    plans = []
    for path in versions_files():
        doc = read_json(path)
        tag = doc.get("poseReferenceId")
        rel = os.path.relpath(path, HERE)
        if tag == new["id"]:
            print("\n%s: sudah memakai acuan ini" % rel)
            continue
        stale.append(rel)
        if delta is None:
            print("\n%s: belum bertanda dan belum ada acuan -- pakai --from-legacy untuk migrasi pertama" % rel)
            continue
        if tag != (old_id if current is not None else None) and not (tag is None and old_id == LEGACY_REFERENCE_ID):
            raise SystemExit("%s bertanda %r, padahal acuan lama %r. Tidak digeser supaya tidak tergeser dua "
                             "kali; pulihkan dari backup atau periksa manual." % (rel, tag, old_id))
        print("\n%s (%d versi), tanda %s:" % (rel, len(doc.get("versions", [])), tag or "(tidak ada)"))
        print("  %-32s %12s %12s %12s %8s" % ("versi", "x", "z", "pitch", "pose"))
        for version in doc.get("versions", []):
            params = version["params"]
            shift = stance_shift_deg(params, old_ref, new_ref, delta)
            # Field yang tidak bergeser dibiarkan bit-per-bit; tidy() hanya untuk yang dihitung.
            moved = dict(params, **{k: tidy(params[k] + delta[k]) for k in BIAS_FIELDS if delta[k] != 0})
            ok = reachable(moved)
            print("  %-32s %12s %12s %12s %7s%s" % (
                version["name"][:32],
                "%g>%g" % (params["init_x_offset"], moved["init_x_offset"]),
                "%g>%g" % (params["init_z_offset"], moved["init_z_offset"]),
                "%g>%g" % (params["init_pitch_offset"], moved["init_pitch_offset"]),
                "-" if shift is None else "%.2f" % shift, "" if ok else "  !! IK GAGAL"))
            if not ok:
                raise SystemExit("versi %r di luar jangkauan kaki sesudah digeser; tidak ada yang ditulis"
                                 % version["name"])
            version["params"] = moved
        doc["poseReferenceId"] = new["id"]
        plans.append((path, doc))
    if plans:
        print("\n  'pose' = selisih pose berdiri sebelum vs sesudah migrasi, maks sendi [deg].")
        print("  Angka 'Relatif page 2' di Studio tidak berubah.")

    with open(PARAM_YAML, encoding="utf-8") as handle:
        param_text = handle.read()
    param_ok = param_yaml_matches(param_text, new_ref)
    if not param_ok:
        stale.append(os.path.relpath(PARAM_YAML, HERE))
    print("\nparam.yaml: %s" % ("sama dengan acuan" if param_ok else "BEDA, ditulis ulang oleh --write"))

    with open(INI_POSE_YAML, encoding="utf-8") as handle:
        ini_text = handle.read()
    positions = read_page_step0(args.bin, args.page)[3]
    _, ini_changes = build_tar_pose_block(positions, parse_existing_tar_pose(ini_text), args.bin, args.page)
    print("ini_pose.yaml: %s" % ("sama dengan page 2" if not ini_changes
                                 else "BEDA -- jalankan scripts/sync_ini_pose_from_page2.py --write"))

    reference_changed = current is None or current["id"] != new["id"]
    if reference_changed:
        stale.insert(0, os.path.relpath(REFERENCE_JSON, HERE))

    if args.check:
        if ini_changes:
            stale.append(os.path.relpath(INI_POSE_YAML, HERE))
        print("\nBASI: " + ", ".join(stale) if stale else "\nsemua selaras dengan page 2")
        return 1 if stale else 0

    if not args.write:
        print("\n(dry run) tambahkan --write untuk menulis")
        return 0
    if delta is None and stale:
        raise SystemExit("belum ada acuan lama untuk menggeser preset; jalankan dengan --from-legacy")
    if studio_running() and not args.allow_running_studio:
        raise SystemExit("Bascorro Studio masih hidup di port %d. Tutup dulu (Ctrl+C di pnpm dev): tab yang "
                         "terbuka masih memegang angka lama, dan Save berikutnya menulis balik angka itu."
                         % STUDIO_PORT)

    # Semua isi disiapkan dan semua cadangan dibuat dulu; baru sesudahnya ditulis.
    # Acuan ditulis TERAKHIR: kalau ada yang gagal di tengah, yang tertinggal
    # adalah preset bertanda baru dengan acuan lama -- dan itu ditolak keras oleh
    # demo dan --check, bukan diam-diam membuat robot condong.
    outputs = []
    for path, doc in plans:
        if path == STUDIO_VERSIONS:
            # Sama seperti satu Save di Studio: revisi naik, waktunya dicatat.
            doc["revision"] = int(doc.get("revision") or 0) + 1
            doc["savedAt"] = datetime.datetime.now(datetime.timezone.utc).isoformat(
                timespec="milliseconds").replace("+00:00", "Z")
        outputs.append((path, json_text(doc)))
    if not param_ok:
        outputs.append((PARAM_YAML, param_yaml_with_reference(param_text, new_ref)))
    if reference_changed:
        outputs.append((REFERENCE_JSON, json_text(new)))
    if not outputs:
        print("\ntidak ada yang perlu ditulis")
        return 0

    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    preflight([path for path, _ in outputs])
    for path, _ in outputs:
        backup(path, stamp)
    written = []
    for path, text in outputs:
        write_atomic(path, text)
        written.append(path)
    print("\nditulis:")
    for path in written:
        print("  " + os.path.relpath(path, HERE))
    print("cadangan: %s (*.%s.*)" % (os.path.relpath(BACKUP_DIR, HERE), stamp))
    print("Sesudah ini: jalankan ulang op3_manager (param.yaml), lalu buka lagi Bascorro Studio.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
