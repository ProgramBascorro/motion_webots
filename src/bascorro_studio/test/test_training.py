"""Uji modul training Bascorro Studio tanpa robot.

Jalankan dari src/bascorro_studio:  python3 -m pytest test/test_training.py -q
Memakai SALINAN .bin CHRONUS di folder sementara -- berkas asli tidak pernah ditulis.
"""

import math
import os
import shutil
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from bascorro_studio.training import action_bin as ab  # noqa: E402
from bascorro_studio.training import imu_metrics  # noqa: E402
from bascorro_studio.training.action_knobs import RAW_PER_DEG, apply_knobs, clamped_joints, page_phases, \
    split_key, step_knobs  # noqa: E402
from bascorro_studio.training.action_trainer import ActionTrainer, manager_bin_status  # noqa: E402
from bascorro_studio.training.trial_store import TrialStore  # noqa: E402

WS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
CHRONUS_BIN = os.path.join(WS, "src/ROBOTIS-OP3/op3_action_module/data/motion_4095_CHRONUS.bin")
OP_DEMO_H = os.path.join(WS, "src/ROBOTIS-OP3-Demo/op3_demo/include/op3_demo/op_demo.h")
KICK_RIGHT, KICK_LEFT = 7, 8          # TEND_KANAN / TEND_KIRI di .bin CHRONUS


@pytest.fixture
def bin_copy(tmp_path):
    if not os.path.isfile(CHRONUS_BIN):
        pytest.skip("bin CHRONUS tidak ada")
    path = tmp_path / "chronus.bin"
    shutil.copyfile(CHRONUS_BIN, path)
    return str(path)


@pytest.fixture
def trainer(bin_copy, tmp_path):
    # proc palsu yang kosong: status op3_manager "tidak_diketahui", load tetap boleh
    (tmp_path / "proc").mkdir()
    return ActionTrainer(bin_copy, OP_DEMO_H, str(tmp_path / "backup"), proc_root=str(tmp_path / "proc"))


# ---- format .bin -----------------------------------------------------------------

def test_decode_encode_roundtrip_seluruh_bin(bin_copy):
    data = ab.read_bin(bin_copy)
    for index in range(ab.MAX_PAGE):
        page = ab.decode_page(data, index)
        if page["checksum_ok"]:
            assert ab.encode_page(page) == page["raw"], "page %d berubah tanpa disunting" % index


def test_write_page_di_tempat_inode_sama(bin_copy):
    before = os.stat(bin_copy).st_ino
    data = ab.read_bin(bin_copy)
    page = ab.decode_page(data, KICK_RIGHT)
    ab.write_page(bin_copy, ab.TRAINING_SCRATCH_PAGE, ab.encode_page(page, name=ab.SCRATCH_NAME))
    assert os.stat(bin_copy).st_ino == before          # action_module memegang inode ini
    again = ab.decode_page(ab.read_bin(bin_copy), ab.TRAINING_SCRATCH_PAGE)
    assert again["name"] == ab.SCRATCH_NAME and again["checksum_ok"]


def test_profil_dari_nama_page_bukan_motion_index(bin_copy):
    data = ab.read_bin(bin_copy)
    names = ab.motion_index(OP_DEMO_H)
    assert names.get(8) == "WalkingReady"              # enum CHRONUS memang menyebut 8 begitu
    assert ab.guess_profile(ab.decode_page(data, 8), names[8])[0] == "kick"
    assert ab.guess_profile(ab.decode_page(data, 12), "")[0] == "getup"
    assert ab.guess_profile(ab.decode_page(data, 14), "")[0] == "pose"
    assert "ForGrass" not in names.values()            # offset, bukan page


# ---- variabel -> page --------------------------------------------------------------

def test_split_key_dan_step_knobs():
    assert split_key("r_knee@2-5") == ("r_knee", (2, 5))
    assert split_key("time_scale@4") == ("time_scale", (4, 4))
    here = step_knobs({"r_knee@2-5": 1.0, "r_knee@4-4": 0.5, "time_scale@3-4": 0.8, "time_scale@4-6": 1.25}, 4)
    assert here["r_knee"] == pytest.approx(1.5)
    assert here["time_scale"] == pytest.approx(1.0)


def test_apply_knobs_arah_raw_dan_waktu(bin_copy):
    page = ab.decode_page(ab.read_bin(bin_copy), KICK_RIGHT)
    tuned = apply_knobs(page, {"r_knee@4-4": 2.0, "time_scale@4-4": 0.8})
    knee = ab.JOINT_IDS["r_knee"]
    delta = tuned["steps"][3]["positions"][knee] - page["steps"][3]["positions"][knee]
    assert delta == round(2.0 * RAW_PER_DEG) == 23
    for number in (1, 2, 3, 5, 6):                     # step lain tidak tersentuh
        assert tuned["steps"][number - 1] == page["steps"][number - 1]
    assert tuned["steps"][3]["time"] == round(page["steps"][3]["time"] * 0.8)
    assert page["steps"][3]["positions"][knee] != tuned["steps"][3]["positions"][knee]   # asli tidak diubah


def test_posisi_invalid_dan_jepit():
    raw = bytearray(ab.PAGE_SIZE)
    raw[15], raw[16], raw[20], raw[22] = 1, ab.TIME_BASE_SCHEDULE, 2, 32
    page = ab.decode_page(bytes(raw), 0)
    page["steps"] = [{"positions": [2048] * ab.MAX_JOINTS, "pause": 0, "time": 50} for _ in range(2)]
    page["steps"][0]["positions"][ab.JOINT_IDS["r_knee"]] = ab.INVALID_BIT
    page["steps"][1]["positions"][ab.JOINT_IDS["r_knee"]] = 4090
    tuned = apply_knobs(page, {"r_knee@1-2": 5.0})
    assert tuned["steps"][0]["positions"][ab.JOINT_IDS["r_knee"]] == ab.INVALID_BIT
    assert tuned["steps"][1]["positions"][ab.JOINT_IDS["r_knee"]] == 4095
    assert clamped_joints(page, {"r_knee@1-2": 5.0}) == [(2, "r_knee")]


def test_fase_tendangan_dibaca_dari_data_servo(bin_copy):
    data = ab.read_bin(bin_copy)
    right = page_phases(ab.decode_page(data, KICK_RIGHT), "TEND_KANAN")
    left = page_phases(ab.decode_page(data, KICK_LEFT), "TEND_KIRI")
    assert (right["kick"], right["support"]) == ("r", "l")
    assert (left["kick"], left["support"]) == ("l", "r")
    assert right["name_side"] == "r" and left["name_side"] == "l"


# ---- layanan Action Training -----------------------------------------------------------

def test_page_list_dan_usulan_chronus(trainer):
    listing = trainer.page_list()
    kicks = [p["index"] for p in listing["pages"] if p["profile"] == "kick"]
    assert kicks[:2] == [KICK_RIGHT, KICK_LEFT]        # tendangan duluan
    assert any("RightKick = page 218 kosong" in n for n in listing["notes"])
    detail = trainer.page_detail(KICK_RIGHT)
    groups = {v["group"] for v in detail["variables"]}
    assert groups == {"A", "B", "C"}
    assert all(v["key"].startswith("l_") for v in detail["variables"] if v["group"] == "A")   # kaki tumpu kiri
    n = len(detail["steps"])
    for v in detail["variables"]:                      # step pertama & terakhir tidak diubah posisinya
        name, (a, b) = split_key(v["key"])
        if name != "time_scale":
            assert 1 < a and b < n
    getup = trainer.page_detail(12)
    assert [v["key"] for v in getup["variables"]] == ["time_scale@1-5"]


def test_load_menulis_page_kerja_saja(trainer, bin_copy):
    original = ab.read_bin(bin_copy)
    detail = trainer.page_detail(KICK_RIGHT)
    result = trainer.load(KICK_RIGHT, detail["sha"], {"l_ank_roll@2-5": -1.0, "time_scale@6-6": 0.9})
    after = ab.read_bin(bin_copy)
    changed = [i for i in range(ab.MAX_PAGE)
               if original[i * ab.PAGE_SIZE:(i + 1) * ab.PAGE_SIZE] != after[i * ab.PAGE_SIZE:(i + 1) * ab.PAGE_SIZE]]
    assert changed == [ab.TRAINING_SCRATCH_PAGE]
    assert result["diff"] and result["planned_s"] > 0
    assert trainer.page_list()["scratch"] == "kandidat"
    assert trainer.restore() == {"restored": True}
    assert ab.read_bin(bin_copy) == original           # 254 kembali persis seperti semula


def test_load_menolak_yang_tidak_sah(trainer, bin_copy):
    sha = trainer.page_detail(KICK_RIGHT)["sha"]
    with pytest.raises(ValueError, match="sudah berubah"):
        trainer.load(KICK_RIGHT, "000000000000", {})
    with pytest.raises(ValueError, match="melewati batas"):
        trainer.load(KICK_RIGHT, sha, {"r_knee@4-4": 20})
    with pytest.raises(ValueError, match="step harus"):
        trainer.load(KICK_RIGHT, sha, {"r_knee@4-9": 1})
    with pytest.raises(ValueError, match="page kerja"):
        trainer.load(ab.TRAINING_SCRATCH_PAGE, sha, {})
    # page 254 berisi gerakan lain -> tidak ditimpa
    data = ab.read_bin(bin_copy)
    ab.write_page(bin_copy, ab.TRAINING_SCRATCH_PAGE, ab.encode_page(ab.decode_page(data, 14), name="punya_orang"))
    with pytest.raises(ValueError, match="bukan page kerja"):
        trainer.load(KICK_RIGHT, sha, {})
    assert trainer.restore() == {"restored": False}


def test_deploy_cadangan_dan_timpa(trainer, bin_copy, tmp_path):
    detail = trainer.page_detail(KICK_RIGHT)
    values = {"r_knee@4-4": 1.5}
    free = trainer.page_list()["free"]
    result = trainer.deploy(KICK_RIGHT, detail["sha"], values, free, name="TEND_KANAN_RL")
    assert os.path.isfile(result["backup"]) and result["changed"] == 1
    saved = ab.decode_page(ab.read_bin(bin_copy), free)
    assert saved["name"] == "TEND_KANAN_RL" and saved["checksum_ok"]
    with pytest.raises(ValueError, match="centang 'timpa'"):
        trainer.deploy(KICK_RIGHT, detail["sha"], values, KICK_LEFT)
    with pytest.raises(ValueError, match="tidak boleh"):
        trainer.deploy(KICK_RIGHT, detail["sha"], values, ab.STUDIO_SCRATCH_PAGE, overwrite=True)
    with pytest.raises(ValueError, match="nama page"):
        trainer.deploy(KICK_RIGHT, detail["sha"], values, free + 1, name="nama-terlalu-panjang")


def _fake_proc(root, bin_path, comm="op3_manager"):
    fd = root / "4242" / "fd"
    fd.mkdir(parents=True)
    (root / "4242" / "comm").write_text(comm + "\n")
    os.symlink(bin_path, str(fd / "7"))


def test_status_bin_op3_manager(tmp_path, bin_copy):
    same, other = tmp_path / "same", tmp_path / "other"
    _fake_proc(same, bin_copy)
    assert manager_bin_status(bin_copy, str(same))["state"] == "sama"
    other_bin = tmp_path / "lain.bin"
    shutil.copyfile(bin_copy, other_bin)
    _fake_proc(other, str(other_bin))
    status = manager_bin_status(bin_copy, str(other))
    assert status["state"] == "beda" and status["open"] == [str(other_bin)]
    trainer = ActionTrainer(bin_copy, OP_DEMO_H, str(tmp_path / "b"), proc_root=str(other))
    with pytest.raises(ValueError, match="kandidat tidak akan terlihat"):
        trainer.load(KICK_RIGHT, trainer.page_detail(KICK_RIGHT)["sha"], {})


# ---- metrik IMU -----------------------------------------------------------------------

def _quat(roll_deg, pitch_deg):
    r, p = math.radians(roll_deg) / 2, math.radians(pitch_deg) / 2
    # ZYX dengan yaw 0 (convertRPYToQuaternion)
    return (math.sin(r) * math.cos(p), math.cos(r) * math.sin(p), -math.sin(r) * math.sin(p),
            math.cos(r) * math.cos(p))


def _recording(pitch_of_t, gyro_x_of_t=lambda t: 0.0, seconds=5.0, rate=125):
    return [[100 + i / rate, gyro_x_of_t(i / rate), 0.0, 0.002, *_quat(0.0, pitch_of_t(i / rate))]
            for i in range(int(seconds * rate) + 1)]


MARKS = {"stand0": 100.0, "start": 101.0, "done": 103.0, "end": 104.5, "planned_s": 2.0}


def test_quat_ke_rpy():
    roll, pitch, _ = imu_metrics.quat_to_rpy_deg(*_quat(5.0, -12.0))
    assert roll == pytest.approx(5.0, abs=1e-6) and pitch == pytest.approx(-12.0, abs=1e-6)


def test_metrik_tendangan_tenang():
    samples = _recording(lambda t: 4.0 * math.sin(math.pi * (t - 1) / 2) if 1 <= t <= 3 else 0.0,
                         lambda t: 0.3 * math.sin(6 * t) if 1 <= t <= 3 else 0.0)
    m = imu_metrics.trial_metrics(samples, MARKS, "kick")
    assert m["valid"] and not m["fell"]
    assert 2.0 < m["tilt_max_deg"] < 4.5
    assert m["gyro_rms_dps"] > 5
    assert m["yaw_abs_deg"] < 0.5                       # gyro z konstan = bias, dikurangi


def test_metrik_jatuh_terdeteksi():
    m = imu_metrics.trial_metrics(_recording(lambda t: min(80.0, max(0.0, (t - 1.5) * 120))), MARKS, "kick")
    assert m["fell"] and m["survived_frac"] < 0.5


def test_metrik_bangun_berhasil():
    samples = _recording(lambda t: 85.0 if t < 1.2 else max(2.0, 85.0 - (t - 1.2) * 60))
    m = imu_metrics.trial_metrics(samples, MARKS, "getup")
    assert m["valid"] and m["success"] and not m["fell"]
    assert 0.5 < m["time_to_upright_s"] < 2.0


def test_metrik_rekaman_pendek_tidak_sah():
    assert imu_metrics.trial_metrics(_recording(lambda t: 0.0, seconds=0.5), MARKS, "kick")["valid"] is False
    with pytest.raises(ValueError, match="marks kurang"):
        imu_metrics.trial_metrics([], {"start": 1}, "kick")


# ---- penyimpanan percobaan ---------------------------------------------------------------

def test_trial_store_nomor_per_sesi(tmp_path):
    store = TrialStore(str(tmp_path / "action" / "trials.json"), "values")
    a, _ = store.add({"id": "a", "session": "s1", "values": {}})
    b, _ = store.add({"id": "b", "session": "s1", "values": {}})
    c, trials = store.add({"id": "c", "session": "s2", "values": {}})
    assert (a["no"], b["no"], c["no"]) == (1, 2, 1) and len(trials) == 3
    with pytest.raises(ValueError, match="values"):
        store.add({"id": "d", "params": {}})
    assert [t["id"] for t in store.delete("a")] == ["b", "c"]
    with pytest.raises(LookupError):
        store.delete("tidak-ada")
