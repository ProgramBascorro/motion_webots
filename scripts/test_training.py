"""Uji menu Training tanpa robot: sumber data, kenop page, metrik/skor, hasil, feedback,
feedback -> iterasi berikutnya, simpan & penerapan ke robot.

    python3 -m unittest scripts/test_training.py

Semua .bin di sini SINTETIS (berkas sementara); .bin robot tidak pernah disentuh.
Penerapan ke Studio diuji ke server Studio TIRUAN (StudioStub) berisi SALINAN
walking_versions.json -- berkas Studio asli hanya dibaca.
"""
import copy
import http.server
import json
import math
import os
import shutil
import struct
import sys
import tempfile
import threading
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import autotune_action as act  # noqa: E402
import autotune_action_core as acore  # noqa: E402
import autotune_walking as walk  # noqa: E402
import autotune_walking_core as wcore  # noqa: E402
import training_deploy as deploy  # noqa: E402
import builtins
import training_feedback as feedback  # noqa: E402
import training_iteration as iteration  # noqa: E402
import training_results as results  # noqa: E402
from walking_ready_fk import ik_leg  # noqa: E402
import training_sources as src  # noqa: E402
import training_action_web as web  # noqa: E402

EMPTY_HEADER = {15: 1, 16: 0x0A, 22: 32, 24: 32}


def make_page(name="", steps=(), repeat=1, speed=32, accel=32, next_page=0, schedule=0x0A):
    """512 byte page seperti buatan Action Editor (pgain 0x55, checksum benar)."""
    raw = bytearray(512)
    encoded = name.encode("ascii")[:13]
    raw[:len(encoded)] = encoded
    raw[15], raw[16], raw[20], raw[22], raw[24], raw[25] = repeat, schedule, len(steps), speed, accel, next_page
    raw[32:63] = b"\x55" * 31
    for n, (positions, pause, time_value) in enumerate(steps):
        struct.pack_into("<31HBB", raw, 64 + 64 * n, *positions, pause, time_value)
    raw[31] = (0xFF - (sum(raw) & 0xFF)) & 0xFF
    return bytes(raw)


def stand(**overrides):
    """Pose berdiri: semua sendi 2048 kecuali yang ditimpa (id: raw)."""
    positions = [0] + [2048] * 30
    for joint_id, value in overrides.items():
        positions[int(joint_id)] = value
    return positions


def make_bin(path, pages):
    data = bytearray(make_page() * 256)
    for index, page in pages.items():
        data[index * 512:(index + 1) * 512] = page
    with open(path, "wb") as handle:
        handle.write(data)
    return bytes(data)


KICK = make_page("TEND_KANAN\n", [(stand(), 30, 80), (stand(**{"15": 2200}), 40, 100), (stand(), 10, 30)],
                 next_page=2)


def ids(**raw):
    return {str(src.JOINT_IDS[joint]): value for joint, value in raw.items()}


# Tendangan KIRI sintetis: step 2 geser berat ke kanan, step 3-4 kaki kiri terangkat & menendang, step 5 kembali.
SHIFT = ids(r_ank_roll=1900, l_ank_roll=1900)
KICK_LEFT = make_page("TEND_KIRI", [
    (stand(), 30, 80), (stand(**SHIFT), 30, 100),
    (stand(**dict(SHIFT, **ids(l_knee=2648, l_hip_pitch=1748))), 30, 25),
    (stand(**dict(SHIFT, **ids(l_knee=2300, l_hip_pitch=1348))), 20, 20),
    (stand(), 10, 80)])
POSE = make_page("WALKING_READY", [(stand(), 100, 100)])


class SourcesTest(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.bin = os.path.join(self.dir, "TEST.bin")
        self.data = make_bin(self.bin, {2: POSE, 105: KICK, 106: make_page("loop", [(stand(), 0, 10)], next_page=107),
                                        107: make_page("loop2", [(stand(), 0, 10)], next_page=106),
                                        74: make_page("babay", [(stand(), 0, 10)], repeat=0),
                                        60: make_page("rusak", [(stand(), 0, 10)])[:31] + b"\x00" + KICK[32:]})

    def tearDown(self):
        shutil.rmtree(self.dir)

    def test_pages_round_trip_byte_exact_including_names_with_newline(self):
        for index in (2, 105):
            page = src.decode_page(self.data, index)
            self.assertTrue(page["checksum_ok"])
            self.assertEqual(src.encode_page(page), page["raw"])
        self.assertEqual(src.decode_page(self.data, 105)["name"], "TEND_KANAN")

    def test_motion_index_comes_from_op_demo_h(self):
        names = src.motion_index()
        self.assertEqual(names.get(2), "WalkingReady")
        self.assertIn("RightKick", names.values())

    def test_step_time_matches_action_module(self):
        page = src.decode_page(self.data, 105)
        self.assertAlmostEqual(src.step_seconds(page, page["steps"][0]), (80 + 30) * 0.008)
        slow = dict(page, speed=26)
        move, pause = (80 * 26) >> 5, (30 << 5) // 26
        self.assertAlmostEqual(src.step_seconds(slow, page["steps"][0]), (move + pause) * 0.008)
        chain, loops = src.page_chain(self.data, 105)
        self.assertEqual((chain, loops), ([105, 2], False))

    def test_validation_names_every_reason(self):
        self.assertEqual(src.check_page(self.data, 105)[0], [])
        self.assertIn("page kosong", src.check_page(self.data, 200)[0][0])
        self.assertIn("repeat 0", src.check_page(self.data, 74)[0][0])
        self.assertTrue(any("berputar" in e for e in src.check_page(self.data, 106)[0]))
        self.assertTrue(any("checksum" in e for e in src.check_page(self.data, 60)[0]))
        self.assertIn("page kerja", src.check_page(self.data, src.TRAINING_SCRATCH_PAGE)[0][0])
        self.assertIn("1..255", src.check_page(self.data, 0)[0][0])

    def test_list_puts_demo_pages_first_and_guesses_the_profile(self):
        listed = src.list_pages(self.data, {105: "RightKick", 2: "WalkingReady"})
        self.assertEqual([p["index"] for p in listed[:2]], [2, 105])
        self.assertEqual(listed[1]["profile"], "kick")
        self.assertEqual(listed[0]["profile"], "pose")
        self.assertEqual(src.guess_profile(85, src.decode_page(self.data, 105), {85: "GetUpFront"})[0], "getup")

    def test_bin_override_wins_over_everything(self):
        os.environ["BASCORRO_TRAINING_BIN"] = self.bin
        try:
            self.assertEqual(src.active_action_bin()[0], self.bin)
        finally:
            del os.environ["BASCORRO_TRAINING_BIN"]

    def test_write_page_touches_exactly_one_page(self):
        before = src.read_bin(self.bin)
        src.write_page(self.bin, 201, KICK)
        after = src.read_bin(self.bin)
        self.assertEqual(after[201 * 512:202 * 512], KICK)
        self.assertEqual(after[:201 * 512] + after[202 * 512:], before[:201 * 512] + before[202 * 512:])

    def test_yaml_entry_uses_action_yaml_conventions(self):
        page = src.decode_page(self.data, 105)
        page["steps"][0]["positions"][19] = src.INVALID_BIT
        page["steps"][0]["positions"][20] = src.TORQUE_OFF_BIT | 2048
        entry = src.page_to_yaml_entry(page, index=201)
        self.assertEqual(entry["index"], 201)
        self.assertEqual(entry["header"]["schedule"], "time")
        positions = entry["steps"][0]["positions"]
        self.assertIsNone(positions["head_pan"])
        self.assertEqual(positions["head_tilt"], "torque_off")
        self.assertEqual(positions["r_ank_pitch"], 2048)
        self.assertEqual(len(positions), 30)


class KnobTest(unittest.TestCase):
    def test_knobs_follow_ik_joint_directions(self):
        one = round(acore.RAW_PER_DEG)
        deltas = acore.joint_deltas({"pitch_hip_deg": 1.0})
        ids = src.JOINT_IDS
        # pitch: kanan dan kiri berlawanan tanda di raw (cermin 4096 - x) = condong searah
        self.assertEqual(deltas[ids["r_hip_pitch"]], -deltas[ids["l_hip_pitch"]])
        self.assertEqual(abs(deltas[ids["r_hip_pitch"]]), one)
        # roll: searah di raw = badan bergeser ke samping
        deltas = acore.joint_deltas({"roll_ankle_deg": 1.0})
        self.assertEqual(deltas[ids["r_ank_roll"]], deltas[ids["l_ank_roll"]])

    def test_body_directions_come_from_the_ik(self):
        # kaki 2 cm di KIRI panggul = panggul (badan) di KANAN kaki; kaki 2 cm di depan = badan di belakang
        z = -0.20
        level, right, back = ik_leg(0, 0, z, 0, 0, 0), ik_leg(0, 0.02, z, 0, 0, 0), ik_leg(0.02, 0, z, 0, 0, 0)
        self.assertEqual(acore.BODY_RIGHT["roll_ankle_deg"], math.copysign(1.0, right[5] - level[5]))
        self.assertEqual(acore.BODY_FORWARD["pitch_ankle_deg"], -math.copysign(1.0, back[4] - level[4]))

    def test_phase_knob_changes_only_its_steps(self):
        page = src.decode_page(KICK_LEFT, 0)
        out = acore.apply_knobs(page, dict(acore.NEUTRAL, **{"roll_ankle_deg@2-4": -0.5, "time_scale@5-5": 1.1}))
        roll = src.JOINT_IDS["r_ank_roll"]
        self.assertEqual([s["positions"][roll] - o["positions"][roll] for s, o in zip(out["steps"], page["steps"])],
                         [0, -6, -6, -6, 0])
        self.assertEqual([s["time"] for s in out["steps"]], [80, 100, 25, 20, 88])
        # kenop fase netral = page persis sama
        self.assertEqual(src.encode_page(acore.apply_knobs(page, dict(acore.NEUTRAL, **{"roll_ankle_deg@2-4": 0.0}))),
                         KICK_LEFT)

    def test_phases_are_read_from_the_servo_data(self):
        ph = acore.page_phases(src.decode_page(KICK_LEFT, 0), "TEND_KIRI")
        self.assertEqual((ph["kick"], ph["support"], ph["single"], ph["name_side"]), ("l", "r", (3, 4), "l"))
        self.assertEqual(ph["phases"], {"awal": (1, 1), "satu_kaki": (2, 4), "kembali": (5, 5)})
        self.assertIsNone(acore.page_phases(src.decode_page(POSE, 0)))

    def test_neutral_knobs_leave_the_page_byte_identical(self):
        page = src.decode_page(KICK * 1, 0)
        self.assertEqual(src.encode_page(acore.apply_knobs(page, acore.NEUTRAL)), KICK)

    def test_flagged_joints_untouched_and_time_scaled(self):
        positions = stand()
        positions[src.JOINT_IDS["r_ank_pitch"]] = src.INVALID_BIT
        positions[src.JOINT_IDS["l_ank_pitch"]] = src.TORQUE_OFF_BIT
        page = src.decode_page(make_page("x", [(positions, 100, 200), (stand(), 0, 0)]), 0)
        out = acore.apply_knobs(page, dict(acore.NEUTRAL, pitch_ankle_deg=2.0, time_scale=1.25))
        self.assertEqual(out["steps"][0]["positions"][src.JOINT_IDS["r_ank_pitch"]], src.INVALID_BIT)
        self.assertEqual(out["steps"][0]["positions"][src.JOINT_IDS["l_ank_pitch"]], src.TORQUE_OFF_BIT)
        self.assertEqual((out["steps"][0]["pause"], out["steps"][0]["time"]), (125, 250))
        self.assertEqual(out["steps"][1]["time"], 0)          # time 0 = "secepatnya", tetap 0
        self.assertEqual(page["steps"][0]["time"], 200)       # page asli tidak berubah


def action_recording(sway_deg=4.0, settle_tilt=0.0, yaw_dps=0.0, fall_at=None, lying=False, rises=True,
                     rise_dps=30.0, hz=125.0):
    """1 s diam, 4 s gerakan, 1,5 s sesudahnya."""
    imu = []
    marks = {"stand0": 0.0, "start": 1.0, "done": 5.0, "end": 6.5, "fell_at": fall_at, "planned_s": 4.0}
    for i in range(int(6.5 * hz)):
        t = i / hz
        moving = 1.0 <= t <= 5.0
        roll = sway_deg * math.sin(2 * math.pi * t) if moving else 0.0
        gyro = math.radians(sway_deg) * 2 * math.pi * math.cos(2 * math.pi * t) if moving else 0.0
        pitch = settle_tilt if t > 5.0 else 0.0
        if lying:
            pitch = 80.0 if t < 1.0 else (max(3.0, 80.0 - rise_dps * (t - 1.0)) if rises else 60.0)
        if fall_at is not None and t >= fall_at:
            pitch = 70.0
        imu.append((t, gyro, 0.0, math.radians(yaw_dps) if moving else 0.0, roll, pitch, 0.0))
    return imu, marks


class ActionScoreTest(unittest.TestCase):
    def score(self, profile, base_rec, rec, label=None):
        weights = acore.PROFILES[profile]["weights"]
        base = acore.compute_metrics(*base_rec, profile)
        scales = acore.scales_from_baseline([base], weights)
        return acore.cost(acore.compute_metrics(*rec, profile), scales, weights, profile, label)

    def test_calmer_kick_scores_better_than_the_original(self):
        # Baseline <= 1: metrik yang di bawah lantainya (mis. yaw ~0) menyumbang < bobotnya.
        base = action_recording(sway_deg=4.0)
        original = self.score("kick", base, base)
        self.assertLessEqual(original, 1.0)
        self.assertLess(self.score("kick", base, action_recording(sway_deg=2.0)), original)
        self.assertGreater(self.score("kick", base, action_recording(sway_deg=4.0, settle_tilt=6.0)), original)
        self.assertGreater(self.score("kick", base, action_recording(sway_deg=4.0, yaw_dps=10.0)), original)

    def test_fall_is_worse_than_any_wobble_and_operator_has_the_last_word(self):
        base = action_recording()
        fell = action_recording(fall_at=3.0)
        self.assertGreaterEqual(self.score("kick", base, fell), 5.0)
        self.assertGreater(self.score("kick", base, fell), self.score("kick", base, action_recording(sway_deg=12.0)))
        self.assertEqual(self.score("kick", base, base, label="hampir"), 5.0)
        self.assertGreaterEqual(self.score("kick", base, base, label="jatuh"), 5.0)

    def test_getup_must_stand_up_first(self):
        base = action_recording(lying=True)
        metrics = acore.compute_metrics(*base, "getup")
        self.assertTrue(metrics["success"])
        self.assertGreater(metrics["time_to_upright_s"], 1.0)
        original = self.score("getup", base, base)
        self.assertLessEqual(original, 1.0)
        self.assertGreater(self.score("getup", base, action_recording(lying=True, rise_dps=20.0)), original)
        failed = self.score("getup", base, action_recording(lying=True, rises=False))
        self.assertGreaterEqual(failed, 5.0)
        self.assertLess(failed, 10.0)                       # sebagian bangun lebih baik dari tidak sama sekali
        # operator bilang berhasil padahal IMU tidak: tidak bisa dinilai, bukan juara
        self.assertEqual(self.score("getup", base, action_recording(lying=True, rises=False), label="normal"), 5.0)

    def test_every_profile_metric_has_a_floor_and_a_description(self):
        for spec in acore.PROFILES.values():
            self.assertAlmostEqual(sum(spec["weights"].values()), 1.0)
            for key in spec["weights"]:
                self.assertIn(key, acore.FLOORS)
                self.assertIn(key, acore.METRIC_TEXT)
            for knob in spec["params"]:
                self.assertIn(knob, acore.BOUNDS)


class ResultsTest(unittest.TestCase):
    """Sesi sintetis di folder sementara (SESSIONS_DIR dialihkan)."""

    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.saved = walk.SESSIONS_DIR
        walk.SESSIONS_DIR = os.path.join(self.dir, "autotune")
        self.bin = os.path.join(self.dir, "TEST.bin")
        make_bin(self.bin, {2: POSE, 105: KICK, 201: make_page("dipakai", [(stand(), 0, 10)])})
        os.environ["BASCORRO_TRAINING_BIN"] = self.bin

    def tearDown(self):
        walk.SESSIONS_DIR = self.saved
        del os.environ["BASCORRO_TRAINING_BIN"]
        shutil.rmtree(self.dir)

    def action_session(self):
        class Args:
            page, profile, params, sigma, popsize, seed, trials, baseline_runs = 105, None, None, 0.2, None, 1, 8, 2
            warm_start, weights = None, None
        state = act.new_state(Args)
        state["bin_path"] = self.bin
        base_metrics = acore.compute_metrics(*action_recording(sway_deg=4.0), "kick")
        good = acore.compute_metrics(*action_recording(sway_deg=2.0), "kick")
        fell = acore.compute_metrics(*action_recording(fall_at=3.0), "kick")
        knobs = dict(acore.NEUTRAL, pitch_ankle_deg=1.5, roll_ankle_deg=-0.5)
        trial = lambda n, kind, m, params, label="normal": {
            "n": n, "kind": kind, "metrics": m, "label": label, "time": "2026-10-24T10:00:0%dZ" % n,
            "params": {k: params[k] for k in state["params"]}}
        state["trials"] = [trial(1, "baseline", base_metrics, acore.NEUTRAL),
                           trial(2, "baseline", base_metrics, acore.NEUTRAL),
                           trial(3, "cma", good, knobs),
                           trial(4, "cma", fell, dict(acore.NEUTRAL, pitch_hip_deg=-3.0), label="jatuh")]
        act.recost_all(state)
        session = act.ActionSession("tk-test")
        session.state = state
        session.save()
        return session

    def test_best_is_the_lowest_usable_candidate_with_a_reason(self):
        self.action_session()
        listed = results.sessions()
        self.assertEqual([s.name for s in listed], ["tk-test"])
        session = listed[0]
        self.assertEqual(session.kind, "action")
        top = results.best(session)
        self.assertEqual(top["trial"]["n"], 3)
        self.assertIn("lebih baik dari rata-rata baseline", top["reason"])
        self.assertIn("BELUM diulang", top["reason"])
        self.assertEqual([t["n"] for _, _, t in results.ranked(session)], [3, 1, 2])
        self.assertIn("terhenti di 2/8", results.status(session))
        self.assertIsNone(results.stale_reason(session))

    def test_save_deploy_overwrite_and_undo_page(self):
        session = results.open_session(self.action_session().name)
        trial = results.find_trial(session, "best")
        with self.assertRaises(SystemExit):
            results.find_trial(session, "4")                      # jatuh: tidak boleh disimpan/diterapkan
        # "Simpan hasil terlebih dahulu": berkas + daftar, .bin belum disentuh
        before = src.read_bin(self.bin)
        record, created = deploy.save_result(session, trial)
        self.assertTrue(created)
        self.assertEqual(src.read_bin(self.bin), before)
        self.assertEqual(deploy.save_result(session, trial)[1], False)     # kedua kali: dipakai lagi
        data = deploy.load_data(deploy.saved_record(session, 3))           # DIMUAT KEMBALI dari berkasnya
        self.assertEqual(data["knobs"]["pitch_ankle_deg"], 1.5)
        self.assertIn("r_ank_pitch", data["joint_raw_changes"])
        self.assertTrue(any(f.endswith(".yaml") for f in record["files"]))
        self.assertEqual(bytes.fromhex(data["page_hex"]), results.tuned_page(session, trial)[1])
        # page berisi tanpa izin menimpa: ditolak, tidak ditulis, tidak dicatat sebagai kiriman
        with self.assertRaises(SystemExit):
            deploy.deploy(session, trial, "page", 201)
        self.assertEqual(src.read_bin(self.bin), before)
        self.assertEqual(deploy.saved_record(session, 3)["deployments"], [])
        # page kosong: ditulis, dibaca balik, cadangan, tercatat
        plan, entry = deploy.deploy(session, trial, "page", 202, name="kick_tuned")
        self.assertEqual(entry["status"], "berhasil", entry)
        self.assertTrue(entry["verified"])
        self.assertTrue(os.path.isfile(deploy._abs(entry["backup"])))
        written = src.decode_page(src.read_bin(self.bin), 202)
        self.assertEqual((written["name"], written["next"], written["checksum_ok"]), ("kick_tuned", 2, True))
        self.assertEqual(src.decode_page(src.read_bin(self.bin), 105)["raw"], KICK)    # page asli tidak tersentuh
        # sama persis -> tidak ditulis ulang
        self.assertEqual(deploy.deploy(session, trial, "page", 202, name="kick_tuned")[1]["status"], "sama")
        # timpa page asli dengan izin
        plan, entry = deploy.deploy(session, trial, "page", 105, overwrite=True)
        self.assertEqual(entry["status"], "berhasil")
        self.assertNotEqual(src.decode_page(src.read_bin(self.bin), 105)["raw"], KICK)
        self.assertIn("page 105", deploy.delivery_text(deploy.saved_record(session, 3)))
        # kembalikan: page 105 = isi asli lagi; cadangan sebelum menimpa TIDAK tertimpa cadangan berikutnya
        overwrite_backup = deploy._abs(entry["backup"])
        undone = deploy.undo(session, trial)
        self.assertEqual(undone["status"], "berhasil")
        self.assertNotEqual(deploy._abs(undone["backup"]), overwrite_backup)
        self.assertEqual(src.decode_page(src.read_bin(overwrite_backup), 105)["raw"], KICK)
        self.assertEqual(src.decode_page(src.read_bin(self.bin), 105)["raw"], KICK)
        # yang sebelumnya (202) sekarang penerapan aktif terakhir; diubah orang lain -> tidak ditimpa
        src.write_page(self.bin, 202, make_page("orang_lain", [(stand(), 0, 10)]))
        with self.assertRaises(SystemExit):
            deploy.undo(session, trial)
        self.assertEqual(src.decode_page(src.read_bin(self.bin), 202)["name"], "orang_lain")

    def test_saved_result_is_rejected_when_its_file_is_tampered(self):
        session = results.open_session(self.action_session().name)
        record, _ = deploy.save_result(session, results.find_trial(session, "best"))
        path = deploy._abs(record["data_file"])
        with open(path, encoding="utf-8") as handle:
            data = json.load(handle)
        data["page_hex"] = data["page_hex"][:200] + ("00" if data["page_hex"][200:202] != "00" else "01") + \
            data["page_hex"][202:]
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(data, handle)
        with self.assertRaises(SystemExit):
            deploy.load_data(record)

    def test_bundle_copies_files_and_feedback(self):
        session = results.open_session(self.action_session().name)
        trial = results.find_trial(session, "best")
        feedback.add_note(session.dir, session.name, 3, "baris satu\nbaris dua", "perbaikan")
        folder, written = deploy.bundle(session, trial, os.path.join(self.dir, "out"))
        self.assertTrue(all(os.path.isfile(p) for p in written))
        with open(os.path.join(folder, "feedback.md"), encoding="utf-8") as handle:
            self.assertIn("baris satu\nbaris dua", handle.read())

    def test_walking_session_reports_through_the_same_layer(self):
        jb1 = os.path.join(self.saved, "jb1", "state.json")
        if not os.path.isfile(jb1):
            self.skipTest("sesi jb1 tidak ada di mesin ini")
        os.makedirs(os.path.join(walk.SESSIONS_DIR, "jb1"))
        shutil.copy(jb1, os.path.join(walk.SESSIONS_DIR, "jb1", "state.json"))
        session = results.open_session("jb1")
        self.assertEqual(session.kind, "studio")
        top = results.best(session)
        self.assertIsNotNone(top)
        record, _ = deploy.save_result(session, top["trial"])
        meta = deploy.load_data(record)
        self.assertEqual(len(meta["params"]), 29)
        self.assertEqual(meta["params"]["x_move_amplitude"], session.state["start_params"]["x_move_amplitude"])

    def test_verdict_gagal_moves_best_to_the_next_candidate_with_the_reason(self):
        session = results.open_session(self.action_session().name)
        self.assertEqual(results.best(session)["trial"]["n"], 3)
        note = feedback.add_note(session.dir, session.name, 3, "tendangan bagus di uji, tapi di lapangan jatuh",
                                 "gagal")
        top = results.best(session)
        self.assertEqual(top["trial"]["n"], 1)
        self.assertIn(note["id"], top["reason"])
        feedback.add_note(session.dir, session.name, 3, "diuji lagi, aman", "baik")   # kesimpulan TERAKHIR berlaku
        self.assertEqual(results.best(session)["trial"]["n"], 3)


class FeedbackTest(unittest.TestCase):
    LONG = ("Saat berjalan ke kanan, robot stabil pada tiga langkah pertama. Setelah itu, badan mulai miring ke "
            "belakang dan kaki kiri kurang terangkat.\n\n  Saya ingin perpindahan berat badan lebih halus — "
            "dan robot “lebih stabil” saat melangkah.\n" + "detail " * 800)

    def setUp(self):
        self.dir = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.dir)

    def reader(self, lines):
        feed = iter(lines)

        def read(_prompt=""):
            value = next(feed)
            if isinstance(value, BaseException):
                raise value
            return value
        return read

    def test_long_multiline_text_is_stored_whole_and_linked_to_the_candidate(self):
        note = feedback.add_note(self.dir, "s1", 7, self.LONG, "perbaikan", "sesudah training", run=9,
                                 tested_on="page 105 di ALPHONSE.bin", result_id="s1-n007")
        feedback.add_note(self.dir, "s1", 7, "catatan kedua")
        feedback.add_note(self.dir, "s1", 3, "kandidat lain")
        mine = feedback.notes(self.dir, 7)
        self.assertEqual([n["text"] for n in mine], [self.LONG.strip("\n"), "catatan kedua"])
        self.assertEqual((mine[0]["candidate"], mine[0]["trial"], mine[0]["result_id"]), (7, 9, "s1-n007"))
        self.assertEqual(mine[0]["id"], note["id"])
        self.assertEqual(len(feedback.notes(self.dir)), 3)
        self.assertEqual(feedback.latest_verdicts(self.dir)[7][0], "perbaikan")

    def test_a_broken_line_does_not_hide_the_other_notes(self):
        feedback.add_note(self.dir, "s1", 1, "satu")
        with open(feedback.feedback_path(self.dir), "a", encoding="utf-8") as handle:
            handle.write('{"potongan baris yang rusak\n')
        feedback.add_note(self.dir, "s1", 1, "dua")
        self.assertEqual([n["text"] for n in feedback.notes(self.dir)], ["satu", "dua"])

    def test_terminal_input_save_cancel_and_rewrite(self):
        quiet = lambda *_: None
        self.assertEqual(feedback.read_text(self.reader(["baris 1", "", "baris 3", "."]), quiet), "baris 1\n\nbaris 3")
        self.assertIsNone(feedback.read_text(self.reader(["abc", "!batal"]), quiet))
        self.assertIsNone(feedback.read_text(self.reader(["abc", KeyboardInterrupt()]), quiet))
        self.assertEqual(feedback.read_text(self.reader(["abc", EOFError()]), quiet), "abc")
        self.assertIsNone(feedback.read_text(self.reader(["   ", "."]), quiet))
        # tulis -> kesimpulan -> "u" tulis ulang -> tulis lagi -> simpan
        note = feedback.prompt_note(self.dir, "s1", 4, "riwayat hasil", read=self.reader(
            ["salah ketik", ".", "", "u", "goyah saat", "menendang", ".", "g", ""]), write=quiet)
        self.assertEqual((note["text"], note["verdict"]), ("goyah saat\nmenendang", "gagal"))
        self.assertIsNone(feedback.prompt_note(self.dir, "s1", 4, "x", read=self.reader(["a", ".", "", "b"]),
                                               write=quiet))
        self.assertEqual(len(feedback.notes(self.dir, 4)), 1)

    def test_understanding_of_the_spec_and_operator_sentences(self):
        def behaviors(text):
            return [(b["id"], b["want"], b["foot"], tuple(b["phases"]))
                    for s in feedback.understand(text)["sentences"] for b in s["behaviors"]]
        walk_text = ("Robot mulai miring ke belakang setelah beberapa langkah. Kaki kiri juga kurang terangkat. "
                     "Tolong perbaiki kestabilan badan dan gerakan kaki kiri.")
        self.assertEqual(behaviors(walk_text), [("condong_belakang", "depan", None, ("tengah",)),
                                                ("kaki_mentok", None, "kiri", ()), ("goyah", None, None, ())])
        unclear = [u for s in feedback.understand(walk_text)["sentences"] for u in s["unclear"]]
        self.assertEqual(unclear, ["gerakan kaki kiri"])
        kick = "Tendangan kiri sudah mengenai sasaran, tetapi kaki tumpuan masih goyah dan robot sulit kembali ke posisi siap."
        result = feedback.understand(kick)["sentences"][0]
        self.assertEqual([(b["id"], b["foot"]) for b in result["behaviors"]], [("goyah", "tumpuan"), ("sulit_kembali", None)])
        self.assertEqual(result["positives"], ["tendangan kiri sudah mengenai sasaran"])
        # catatan operator asli 26 Okt (salah ketik ikut dikenali)
        self.assertEqual(behaviors("kaki kirinya itu mentok ke tanah, menurut ku sih karena kaki kanan nya kurang miring "
                                   "ke arah kanan"),
                         [("condong_samping", "kanan", "kanan", ()), ("kaki_mentok", None, "kiri", ())])
        self.assertEqual(behaviors("masih kurang kedepan sih posisi badannya, itu dia masih berat kebalakang"),
                         [("condong_belakang", "depan", None, ())])
        self.assertEqual(behaviors("yang buat jatuh itu karena posisi nya terlalu mendangak sehingga ketika pertama "
                                   "mulai itu dia jatuh ke belakang."), [("condong_belakang", "depan", None, ("awal",))])
        self.assertEqual(behaviors("masih terlalu kuat untuk awalannya."), [("kasar", None, None, ("awal",))])
        self.assertEqual(behaviors("awalan nya sih bagus, tapi ketika agak jauh malah jatuh."),
                         [("jatuh", None, None, ("tengah",))])
        # ingkar = hal yang sudah baik, bukan keluhan
        calm = feedback.understand("Sudah tidak goyah, tidak miring ke depan. Robot stabil.")["sentences"]
        self.assertEqual([b for s in calm for b in s["behaviors"]], [])
        self.assertEqual(len([p for s in calm for p in s["positives"]]), 3)


class TrainerOptionTest(unittest.TestCase):
    def test_weights_override_changes_the_walking_score_only_by_weight(self):
        metrics = {"gyro_rms_dps": 20.0, "tilt_std_deg": 2.0, "yaw_drift_deg": 10.0, "lean_deg": 2.0}
        scales = {k: 10.0 for k in wcore.WEIGHTS}
        base = wcore.cost(metrics, scales)
        heavy = walk.parse_weights("gyro_rms_dps=1", wcore.WEIGHTS)
        self.assertAlmostEqual(sum(heavy.values()), 1.0)
        self.assertGreater(wcore.cost(metrics, scales, weights=heavy), base)
        with self.assertRaises(SystemExit):
            walk.parse_weights("bukan_metrik=1", wcore.WEIGHTS)

    def test_rerun_target_is_the_chosen_candidate(self):
        state = {"trials": [{"n": 1, "kind": "baseline", "metrics": {}}, {"n": 2, "kind": "cma", "metrics": {}},
                            {"n": 3, "kind": "rerun", "metrics": {}, "rerun_of": 2}]}
        self.assertEqual(walk.rerun_target(state, 2, lambda s: s["trials"][0])["n"], 2)
        self.assertEqual(walk.rerun_target(state, None, lambda s: s["trials"][0])["n"], 1)
        with self.assertRaises(SystemExit):
            walk.rerun_target(state, 3, None)                    # ulangan bukan kandidat


def walking_metrics(lean=-6.0, gyro=30.0, tilt=3.0, yaw=2.0):
    return {"gyro_rms_dps": gyro, "tilt_std_deg": tilt, "tilt_max_deg": 9.0, "lean_deg": lean,
            "yaw_drift_deg": yaw, "hip_swing_deg": 8.0, "valid": True, "fell": False}


class IterationTest(unittest.TestCase):
    """Feedback -> perubahan -> percobaan berikutnya -> bukti -> CMA lanjut, tanpa robot:
    percobaan 'feedback' dibuat persis seperti pelatih membuatnya (iteration.recorded)."""

    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.saved = walk.SESSIONS_DIR
        walk.SESSIONS_DIR = os.path.join(self.dir, "autotune")
        self.bin = os.path.join(self.dir, "TEST.bin")
        make_bin(self.bin, {2: POSE, 104: KICK_LEFT})
        os.environ["BASCORRO_TRAINING_BIN"] = self.bin

    def tearDown(self):
        walk.SESSIONS_DIR = self.saved
        del os.environ["BASCORRO_TRAINING_BIN"]
        shutil.rmtree(self.dir)

    def kick_session(self):
        class Args:
            page, profile, params, sigma, popsize, seed, trials, baseline_runs = 104, "kick", None, 0.2, None, 1, 8, 2
            warm_start, weights = None, None
        state = act.new_state(Args)
        state["bin_path"] = self.bin
        base = acore.compute_metrics(*action_recording(sway_deg=4.0), "kick")
        state["trials"] = [{"n": n, "kind": "baseline", "metrics": base, "label": "normal", "time": "2026-10-27T01:00:0%dZ" % n,
                            "params": iteration.recorded(state, acore.NEUTRAL)} for n in (1, 2)]
        act.recost_all(state)
        session = act.ActionSession("kick-fb")
        session.state = state
        session.save()
        return session

    def run_feedback_trial(self, session, step, metrics, label="normal", score=None):
        """Yang dilakukan walk.run_feedback_steps sesudah robot memutar step["after"]."""
        state = session.state
        trial = {"n": len(state["trials"]) + 1, "kind": "feedback", "time": feedback.now_iso(), "label": label,
                 "params": iteration.recorded(state, dict(step["after"])), "metrics": metrics,
                 "feedback_step": step["id"], "loaded": "dibaca balik (uji)"}
        state["trials"].append(trial)
        (score or act.recost_all)(state)
        iteration.finish(state, step, trial)
        return trial

    def test_kick_feedback_becomes_a_phase_knob_used_by_the_next_trial_and_the_saved_page(self):
        session = self.kick_session()
        state = session.state
        feedback.add_note(session.dir, session.name, 2, "kaki kiri mentok saat menendang, kaki kanan kurang miring ke kanan")
        step = iteration.propose(state, session.dir, state["trials"][1], "uji")
        self.assertEqual(step["source"]["configs"], [1, 2])
        self.assertEqual([(c["key"], c["before"], c["after"]) for c in step["changes"]],
                         [("roll_ankle_deg@2-4", 0.0, -0.5)])                 # badan ke kanan, hanya fase satu kaki
        self.assertTrue(all(ok for ok, _ in step["checks"]))
        text = iteration.step_text(state, step, session.name)
        for expected in ("mentok", "step 2 r_ank_roll", "1900 -> 1894", "DIPAKAI: percobaan berikutnya", "#3"):
            self.assertIn(expected, text)
        iteration.apply(state, step, act.recost_all)
        self.assertEqual(iteration.pending(state)["id"], step["id"])
        self.assertEqual(state["weights"], step["weights_after"])           # bobot dipakai skor SEKARANG
        better = acore.compute_metrics(*action_recording(sway_deg=2.0), "kick")
        trial = self.run_feedback_trial(session, step, better)
        self.assertEqual(trial["params"]["roll_ankle_deg@2-4"], -0.5)
        self.assertTrue(step["verified"]["same"])
        self.assertEqual(step["result"]["verdict"], "LEBIH BAIK")
        iteration.settle(state, step, True, act.recost_all)
        session.save()
        self.assertEqual(step["status"], "dipakai")
        self.assertEqual(state["fixed"], {"roll_ankle_deg@2-4": -0.5})
        self.assertTrue(iteration.anchored(state))
        # kandidat CMA berikutnya membawa kenop fase itu; page kerja yang diputar berisi offsetnya
        space = wcore.Space(state["params"], state["start_params"], {}, bounds=acore.BOUNDS)
        candidate = wcore.candidate_params(iteration.effective_start(state), space, [0.5] * len(state["params"]))
        played = src.decode_page(act.candidate_page(state, candidate), 0)
        roll = src.JOINT_IDS["r_ank_roll"]
        self.assertEqual([s["positions"][roll] for s in played["steps"]], [2048, 1894, 1894, 1894, 2048])
        # hasil feedback = kandidat biasa: terbaik, disimpan dengan kenop fase dan page yang sama dengan yang diputar
        session = results.open_session(session.name)
        self.assertEqual(results.best(session)["trial"]["n"], 3)
        record, _ = deploy.save_result(session, results.find_trial(session, 3))
        data = deploy.load_data(record)
        self.assertEqual(data["knobs"]["roll_ankle_deg@2-4"], -0.5)
        self.assertEqual(src.page_sha({"raw": bytes.fromhex(data["page_hex"])}),
                         src.page_sha({"raw": results.tuned_page(session, trial)[1]}))
        self.assertIn("roll_ankle_deg@2-4 0 -> -0.5", results.changes_text(session, trial))
        # "masih" pada hasil feedback itu: langkah searah dibesarkan (x1.5), dasarnya hasil itu
        feedback.add_note(session.dir, session.name, 3, "masih mentok kaki kirinya")
        again = iteration.propose(session.state, session.dir, trial, "uji")
        self.assertEqual([(c["key"], c["before"], c["after"], c["factor"]) for c in again["changes"]],
                         [("roll_ankle_deg@2-4", -0.5, -1.25, 1.5)])
        self.assertIn("MASIH", " ".join(again["changes"][0]["notes"]))

    def test_kick_speed_stays_locked_but_the_return_phase_may_slow_down(self):
        session = self.kick_session()
        feedback.add_note(session.dir, session.name, 1, "tendangannya terlalu cepat. robot sulit kembali ke posisi siap.")
        step = iteration.propose(session.state, session.dir, session.state["trials"][0], "uji")
        self.assertEqual([(c["key"], c["after"]) for c in step["changes"]], [("time_scale@5-5", 1.1)])
        self.assertTrue(any("DIKUNCI" in item["text"] for item in step["limits"]))

    def test_unclear_feedback_is_not_applied_and_says_what_is_missing(self):
        session = self.kick_session()
        feedback.add_note(session.dir, session.name, 1, "gerakannya masih aneh menurutku")
        step = iteration.propose(session.state, session.dir, session.state["trials"][0], "uji")
        self.assertEqual((step["status"], step["changes"]), ("tidak_berlaku", []))
        self.assertTrue(any("gerakannya masih aneh" in item["text"] for item in step["needs"]))
        self.assertIn("BELUM bisa diterapkan", iteration.step_text(session.state, step))
        iteration.record(session.state, step, "tidak_berlaku")
        iteration.record(session.state, iteration.propose(session.state, session.dir, session.state["trials"][0], "uji"),
                         "tidak_berlaku")
        self.assertEqual(len(iteration.steps(session.state)), 1)             # dibuka ulang: tidak dobel

    def test_weights_only_feedback_rescores_and_cancel_restores(self):
        session = self.kick_session()
        state = session.state
        feedback.add_note(session.dir, session.name, 1, "kaki tumpuan goyah")
        step = iteration.propose(state, session.dir, state["trials"][0], "uji")
        self.assertEqual(step["changes"], [])
        self.assertGreater(step["weights_after"]["gyro_rms_dps"], step["weights_before"]["gyro_rms_dps"])
        iteration.apply(state, step, act.recost_all)
        self.assertEqual((step["status"], state["weights"]), ("dipakai", step["weights_after"]))
        # langkah gerak yang dibatalkan sesudah diuji: bobot kembali, CMA tidak pindah
        feedback.add_note(session.dir, session.name, 2, "kaki kiri mentok")
        mean = list(state["cma"]["mean"])
        move = iteration.propose(state, session.dir, state["trials"][1], "uji")
        iteration.apply(state, move, act.recost_all)
        self.run_feedback_trial(session, move, acore.compute_metrics(*action_recording(fall_at=2.0), "kick"), "jatuh")
        self.assertEqual(move["result"]["verdict"], "LEBIH BURUK")
        iteration.settle(state, move, False, act.recost_all)
        self.assertEqual((move["status"], state["cma"]["mean"], state.get("fixed")), ("dibatalkan", mean, {}))
        self.assertEqual(state["weights"], move["weights_before"])

    def walking_session(self):
        with open(src.STUDIO_VERSIONS, encoding="utf-8") as handle:       # hanya DIBACA
            store = json.load(handle)
        if store.get("poseReferenceId") != src.current_reference_id():
            self.skipTest("versi Studio di mesin ini belum dimigrasi ke acuan page 2 sekarang")

        class Args:
            start, motion, params, warm_start, sigma, popsize, seed = store["versions"][0]["id"], "maju", None, None, \
                0.15, 8, 1
            walk_sec, measure_distance, trials, baseline_runs, weights = None, False, 8, 2, None
        state = walk.new_state(Args)
        state["trials"] = [{"n": n, "kind": "baseline", "metrics": walking_metrics(), "label": "hampir",
                            "time": "2026-10-27T01:00:0%dZ" % n, "reference_id": state["reference_id"],
                            "params": iteration.recorded(state, state["start_params"])} for n in (1, 2)]
        walk.recost_all(state)
        session = walk.Session("walk-fb")
        session.state = state
        session.save()
        return session

    def test_walking_feedback_moves_the_cma_centre_and_fixes_soft_start(self):
        session = self.walking_session()
        state = session.state
        feedback.add_note(session.dir, session.name, 1, "terlalu mendangak, saat mulai jatuh ke belakang")
        feedback.add_note(session.dir, session.name, 2, "masih terlalu kuat untuk awalannya")
        step = iteration.propose(state, session.dir, state["trials"][1], "uji")
        start = state["start_params"]
        self.assertEqual({c["key"]: round(c["after"] - c["before"], 5) for c in step["changes"]},
                         {"hip_pitch_offset": round(math.radians(1.0), 5), "soft_start_time": 0.25})
        self.assertIn("cocok dengan feedback", step["evidence"][0]["text"])        # IMU: condong -6 deg
        iteration.apply(state, step, walk.recost_all)
        trial = self.run_feedback_trial(session, step, walking_metrics(lean=-2.0, gyro=25.0), score=walk.recost_all)
        self.assertEqual(trial["params"]["soft_start_time"], start["soft_start_time"] + 0.25)
        self.assertEqual(step["result"]["verdict"], "LEBIH BAIK")                 # hampir -> sah
        iteration.settle(state, step, True, walk.recost_all)
        i = state["params"].index("hip_pitch_offset")
        lo, hi = state["lo"][i], state["hi"][i]
        self.assertAlmostEqual(state["cma"]["mean"][i] * (hi - lo) + lo, step["after"]["hip_pitch_offset"], places=6)
        self.assertEqual(state["fixed"], {"soft_start_time": start["soft_start_time"] + 0.25})
        nxt = wcore.candidate_params(iteration.effective_start(state), walk.space_of(state), state["cma"]["mean"])
        self.assertAlmostEqual(nxt["hip_pitch_offset"], step["after"]["hip_pitch_offset"], places=4)
        self.assertEqual(nxt["soft_start_time"], start["soft_start_time"] + 0.25)
        self.assertIn("soft_start_time", iteration.history_text(state))

    def test_loops_run_pending_feedback_first_and_stop_repeating_a_falling_baseline(self):
        session = self.walking_session()
        state = session.state
        state["trials"] = []                     # preset asli selalu "hampir jatuh"
        calls = {"baseline": 0}

        def baseline():
            calls["baseline"] += 1
            n = len(state["trials"]) + 1
            if n == 1:
                feedback.add_note(session.dir, session.name, 1, "jatuh ke belakang")
                iteration.apply(state, iteration.propose(state, session.dir, {
                    "n": 1, "kind": "baseline", "params": iteration.recorded(state, state["start_params"]),
                    "metrics": walking_metrics()}, "uji"), walk.recost_all)
            return {"n": n, "kind": "baseline", "metrics": walking_metrics(), "label": "hampir", "cost": 5.0,
                    "params": iteration.recorded(state, state["start_params"])}

        def steps():
            step = iteration.pending(state)
            if step is None:
                return False
            self.run_feedback_trial(session, step, walking_metrics(lean=-1.0), score=walk.recost_all)
            iteration.settle(state, step, True, walk.recost_all)
            return True
        walk.baseline_loop(session, baseline, walk.recost_all, walk.usable, lambda t: None, "x", steps)
        self.assertEqual(calls["baseline"], 1)                              # tidak diulang tanpa akhir
        self.assertEqual([t["kind"] for t in state["trials"]], ["baseline", "feedback"])
        # CMA dibangun ulang dari titik tengah baru: generasi berikutnya di sekitar titik feedback
        state["cma"]["mean"] = [0.95] * len(state["params"])
        state["pending"], state["budget"] = [], 2

        class Robot:
            def spin_for(self, _):
                pass

        class Args:
            cooldown_sec = 0
        seen = []

        def run(params, generation):
            seen.append(walk.space_of(state).to_unit(params))
            return {"n": len(state["trials"]) + 1, "kind": "cma", "cost": 1.0, "metrics": walking_metrics(),
                    "params": iteration.recorded(state, params)}
        walk.cma_loop(Robot(), session, Args, walk.space_of(state), run, walk.best_trial, lambda: False)
        self.assertGreater(float(sum(u.mean() for u in seen)) / len(seen), 0.75)

    def test_label_prompt_note_is_offered_for_the_next_trial(self):
        session = self.walking_session()
        state = session.state
        answers = iter(["c", "", "h"])          # c = catatan; Enter = terapkan; lalu label h
        saved_input, saved_prompt = builtins.input, feedback.prompt_note
        builtins.input = lambda _prompt="": next(answers)
        feedback.prompt_note = lambda directory, name, candidate, where, run=None, ask_verdict=True: feedback.add_note(
            directory, name, candidate, "badan berat ke belakang", where=where, run=run)
        try:
            label = walk.ask_label(session, 3, None, "label? ", {"kind": "cma", "params": state["start_params"],
                                                                 "metrics": walking_metrics(), "step": None})
        finally:
            builtins.input, feedback.prompt_note = saved_input, saved_prompt
        self.assertEqual(label, "h")
        step = iteration.pending(walk.Session("walk-fb").load())                  # TERSIMPAN di state.json
        self.assertEqual((step["source"]["trial"], step["next_trial"]), (3, 4))
        self.assertEqual([c["key"] for c in step["changes"]], ["hip_pitch_offset"])


class StudioStub(http.server.BaseHTTPRequestHandler):
    """Server penyimpanan Studio TIRUAN: aturan upsert/activate/delete sama dengan
    web/server/walkingVersionFile.js (nama tidak berganti lewat upsert, nama bentrok ditolak)."""
    store = None

    def log_message(self, *_):
        pass

    def reply(self, status, body):
        data = json.dumps(body).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        self.reply(200, StudioStub.store)

    def do_POST(self):
        op = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
        store, versions = StudioStub.store, StudioStub.store["versions"]
        if op["op"] == "upsert":
            version = op["version"]
            index = next((i for i, v in enumerate(versions) if v["id"] == version["id"]), None)
            if index is not None and versions[index]["name"] != version["name"]:
                return self.reply(409, {"error": "tidak boleh berganti nama"})
            if index is None and any(v["name"] == version["name"] for v in versions):
                return self.reply(409, {"error": "nama sudah dipakai"})
            if index is None:
                versions.insert(0, version)
            else:
                versions[index] = version
            store["activeVersionId"] = version["id"]
        elif op["op"] == "activate":
            store["activeVersionId"] = op["id"]
        elif op["op"] == "delete":
            store["versions"] = [v for v in versions if v["id"] != op["id"]]
        store["revision"] += 1
        self.reply(200, store)


class StudioDeployTest(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.saved_dir, self.saved_url = walk.SESSIONS_DIR, walk.STUDIO_URL
        walk.SESSIONS_DIR = os.path.join(self.dir, "autotune")
        with open(src.STUDIO_VERSIONS, encoding="utf-8") as handle:     # hanya DIBACA, disalin ke server tiruan
            StudioStub.store = json.load(handle)
        if StudioStub.store.get("poseReferenceId") != src.current_reference_id():
            self.skipTest("versi Studio di mesin ini belum dimigrasi ke acuan page 2 sekarang")
        self.server = http.server.HTTPServer(("127.0.0.1", 0), StudioStub)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        walk.STUDIO_URL = "http://127.0.0.1:%d/" % self.server.server_port

    def tearDown(self):
        self.server.shutdown()
        walk.SESSIONS_DIR, walk.STUDIO_URL = self.saved_dir, self.saved_url
        shutil.rmtree(self.dir)

    def session(self):
        first = StudioStub.store["versions"][0]

        class Args:
            start, motion, params, warm_start, sigma, popsize, seed = first["id"], None, None, None, 0.15, 8, 1
            walk_sec, measure_distance, trials, baseline_runs, weights = None, False, 8, 2, None
        state = walk.new_state(Args)
        metrics = {"gyro_rms_dps": 30.0, "tilt_std_deg": 3.0, "tilt_max_deg": 9.0, "lean_deg": 1.0,
                   "yaw_drift_deg": 2.0, "hip_swing_deg": 8.0, "valid": True}
        better = dict(metrics, gyro_rms_dps=20.0)
        tuned = dict(state["start_params"], dsp_ratio=round(state["start_params"]["dsp_ratio"] + 0.02, 6))
        trial = lambda n, kind, m, params: {"n": n, "kind": kind, "metrics": m, "label": "normal",
                                            "time": "2026-10-26T10:00:0%dZ" % n, "reference_id": state["reference_id"],
                                            "params": {k: params[k] for k in state["params"]}}
        state["trials"] = [trial(1, "baseline", metrics, state["start_params"]),
                           trial(2, "baseline", metrics, state["start_params"]), trial(3, "cma", better, tuned)]
        walk.recost_all(state)
        session = walk.Session("st-test")
        session.state = state
        session.save()
        return results.open_session("st-test"), first

    def test_new_preset_overwrite_and_undo_through_the_studio_server(self):
        session, start = self.session()
        trial = results.find_trial(session, "best")
        self.assertEqual(trial["n"], 3)
        active = StudioStub.store["activeVersionId"]
        plan, entry = deploy.deploy(session, trial, "studio-new", "uji training baru")
        self.assertEqual(entry["status"], "berhasil", entry)
        created = next(v for v in StudioStub.store["versions"] if v["name"] == "uji training baru")
        self.assertEqual(created["params"], deploy.load_data(plan["record"])["params"])
        self.assertEqual(len(created["params"]), 29)
        self.assertEqual(StudioStub.store["activeVersionId"], active)      # pilihan di Studio tidak berganti
        # nama yang sudah dipakai: ditolak sebelum menulis, tercatat gagal
        plan, entry = deploy.deploy(session, trial, "studio-new", start["name"])
        self.assertEqual(entry["status"], "gagal")
        self.assertIn("SUDAH dipakai", entry["message"])
        # timpa preset asal (perlu izin), lalu kembalikan
        with self.assertRaises(SystemExit):
            deploy.deploy(session, trial, "studio-preset", start["id"])
        old_params = copy.deepcopy(start["params"])
        plan, entry = deploy.deploy(session, trial, "studio-preset", start["id"], overwrite=True)
        self.assertEqual(entry["status"], "berhasil", entry)
        current = next(v for v in StudioStub.store["versions"] if v["id"] == start["id"])
        self.assertEqual((current["name"], current["params"]["dsp_ratio"]),
                         (start["name"], round(old_params["dsp_ratio"] + 0.02, 6)))
        self.assertEqual(deploy.undo(session, trial)["status"], "berhasil")
        current = next(v for v in StudioStub.store["versions"] if v["id"] == start["id"])
        self.assertEqual(current["params"], old_params)

    def test_studio_down_is_reported_and_the_copy_is_kept(self):
        session, _ = self.session()
        trial = results.find_trial(session, "best")
        walk.STUDIO_URL = "http://127.0.0.1:9/"            # tidak ada yang mendengar
        plan, entry = deploy.deploy(session, trial, "studio-new", "tidak sampai")
        self.assertEqual(entry["status"], "gagal")
        self.assertIn("koneksi", entry["message"])
        record = deploy.saved_record(session, trial["n"])
        self.assertTrue(os.path.isfile(deploy._abs(record["data_file"])))
        self.assertIn("GAGAL", deploy.delivery_text(record))


def quaternion(roll_deg, pitch_deg, yaw_deg=0.0):
    """Kebalikan wcore.quat_to_rpy_deg (urutan ZYX), untuk sampel IMU tiruan."""
    r, p, y = (math.radians(v) / 2 for v in (roll_deg, pitch_deg, yaw_deg))
    return (math.sin(r) * math.cos(p) * math.cos(y) - math.cos(r) * math.sin(p) * math.sin(y),
            math.cos(r) * math.sin(p) * math.cos(y) + math.sin(r) * math.cos(p) * math.sin(y),
            math.cos(r) * math.cos(p) * math.sin(y) - math.sin(r) * math.sin(p) * math.cos(y),
            math.cos(r) * math.cos(p) * math.cos(y) + math.sin(r) * math.sin(p) * math.sin(y))


class ActionWebTest(unittest.TestCase):
    """Tab Action Training (training_action_web) pada .bin sintetis; .bin robot tidak disentuh."""

    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.bin = os.path.join(self.dir, "TEST.bin")
        self.data = make_bin(self.bin, {2: POSE, 104: KICK_LEFT, 105: KICK, 201: make_page("dipakai", [(stand(), 0, 10)])})
        os.environ["BASCORRO_TRAINING_BIN"] = self.bin
        self.sha = src.page_sha(src.decode_page(self.data, 104))

    def tearDown(self):
        del os.environ["BASCORRO_TRAINING_BIN"]
        shutil.rmtree(self.dir)

    def page(self, index):
        return src.decode_page(src.read_bin(self.bin), index)

    def test_joint_key_moves_one_servo_in_raw_direction(self):
        deltas = acore.joint_deltas({"l_hip_pitch": 2.0})
        self.assertEqual(deltas, {src.JOINT_IDS["l_hip_pitch"]: round(2.0 * acore.RAW_PER_DEG)})
        page = src.decode_page(KICK_LEFT, 0)
        out = acore.apply_knobs(page, {"l_hip_pitch@3-4": -1.0})
        hip = src.JOINT_IDS["l_hip_pitch"]
        self.assertEqual([s["positions"][hip] - o["positions"][hip] for s, o in zip(out["steps"], page["steps"])],
                         [0, 0, -11, -11, 0])

    def test_proposal_follows_the_phases_and_keeps_first_and_last_step(self):
        variables, phases = web.propose_variables(src.decode_page(KICK_LEFT, 0))
        self.assertEqual((phases["kick"], phases["support"]), ("l", "r"))
        keys = {v["group"]: [] for v in variables}
        for v in variables:
            keys[v["group"]].append(v["key"])
            _, steps = acore.split_key(v["key"])
            if not v["key"].startswith("time_scale"):
                self.assertTrue(2 <= steps[0] <= steps[1] <= 4, v["key"])
        self.assertEqual(keys["A"], ["r_ank_roll@2-4", "r_hip_roll@2-4", "r_ank_pitch@2-4"])
        # ayunan pinggul kiri terbesar: step 4 (1748 -> 1348), jadi ayunan belakang = step 3
        self.assertEqual(keys["B"], ["l_hip_pitch@3-3", "l_hip_pitch@4-4", "l_knee@4-4", "l_ank_pitch@4-4",
                                     "l_hip_yaw@4-4", "time_scale@4-4"])
        self.assertEqual(keys["C"], ["time_scale@5-5"])
        self.assertEqual(web.propose_variables(src.decode_page(POSE, 0))[0], [])

    def test_load_writes_only_the_scratch_page(self):
        result = web.load(104, self.sha, {"l_knee@3-3": 2.5, "time_scale@4-4": 1.2})
        after = src.read_bin(self.bin)
        knee = src.JOINT_IDS["l_knee"]
        scratch = src.decode_page(after, src.TRAINING_SCRATCH_PAGE)
        self.assertEqual(scratch["name"], act.SCRATCH_NAME)
        self.assertEqual(scratch["steps"][2]["positions"][knee], 2648 + round(2.5 * acore.RAW_PER_DEG))
        self.assertEqual(scratch["steps"][3]["time"], 24)
        self.assertEqual(result["diff"], [
            {"step": 3, "joint": "l_knee", "before": 2648, "after": 2648 + round(2.5 * acore.RAW_PER_DEG)},
            {"step": 4, "joint": "time", "before": [20, 20], "after": [24, 24]}])
        base = src.TRAINING_SCRATCH_PAGE * 512
        self.assertEqual(after[:base] + after[base + 512:], self.data[:base] + self.data[base + 512:])
        self.assertEqual(web.restore(), {"restored": True})
        self.assertEqual(src.read_bin(self.bin), self.data)

    def test_load_refuses_stale_page_bad_values_and_foreign_scratch(self):
        with self.assertRaisesRegex(ValueError, "sudah berubah"):
            web.load(104, "000000000000", {})
        with self.assertRaisesRegex(ValueError, "melewati batas"):
            web.load(104, self.sha, {"l_knee@3-3": web.HARD_DEG + 1})
        with self.assertRaisesRegex(ValueError, "step harus"):
            web.load(104, self.sha, {"l_knee@3-9": 1.0})
        with self.assertRaisesRegex(ValueError, "bukan nama sendi"):
            web.load(104, self.sha, {"lutut@3-3": 1.0})
        src.write_page(self.bin, src.TRAINING_SCRATCH_PAGE, make_page("milik orang", [(stand(), 0, 10)]))
        with self.assertRaisesRegex(ValueError, "tidak ditimpa"):
            web.load(104, self.sha, {})
        self.assertEqual(web.restore(), {"restored": False})

    def test_deploy_backs_up_and_refuses_used_pages(self):
        with self.assertRaisesRegex(ValueError, "centang 'timpa'"):
            web.deploy_result(104, self.sha, {"r_ank_roll@2-4": -1.0}, 201)
        with self.assertRaisesRegex(ValueError, "centang 'timpa'"):
            web.deploy_result(104, self.sha, {"r_ank_roll@2-4": -1.0}, 104)
        result = web.deploy_result(104, self.sha, {"r_ank_roll@2-4": -1.0}, 150, name="TEND_KIRI2")
        self.assertEqual(src.read_bin(result["backup"]), self.data)
        written = self.page(150)
        roll = src.JOINT_IDS["r_ank_roll"]
        self.assertEqual((written["name"], written["checksum_ok"], written["steps"][1]["positions"][roll]),
                         ("TEND_KIRI2", True, 1900 - round(acore.RAW_PER_DEG)))
        self.assertEqual(self.page(104)["raw"], KICK_LEFT)
        web.deploy_result(104, self.sha, {"r_ank_roll@2-4": -1.0}, 104, overwrite=True)
        self.assertEqual(self.page(104)["name"], "TEND_KIRI")
        self.assertEqual(self.page(104)["steps"][1]["positions"][roll], 1900 - round(acore.RAW_PER_DEG))

    def test_metrics_from_quaternion_samples(self):
        imu, marks = action_recording(sway_deg=4.0)
        samples = [[t, gx, gy, gz, *quaternion(roll, pitch)] for t, gx, gy, gz, roll, pitch, _ in imu]
        out = web.trial_metrics(samples, dict(marks, fell_at=None))["metrics"]
        expected = acore.compute_metrics(imu, marks, "kick")
        self.assertTrue(out["valid"])
        self.assertFalse(out["fell"])
        for key in ("gyro_rms_dps", "tilt_max_deg", "settle_dev_deg"):
            self.assertAlmostEqual(out[key], expected[key], places=3)
        imu, marks = action_recording(fall_at=3.0)
        samples = [[t, gx, gy, gz, *quaternion(roll, pitch)] for t, gx, gy, gz, roll, pitch, _ in imu]
        self.assertTrue(web.trial_metrics(samples, marks)["metrics"]["fell"])


if __name__ == "__main__":
    unittest.main()
