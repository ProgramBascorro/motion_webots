"""Uji bagian murni auto-tune walking, tanpa robot:  python3 -m unittest scripts/test_autotune_walking_core.py"""
import json
import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import autotune_walking_core as core  # noqa: E402
from walking_ready_fk import REFERENCE_JSON, STUDIO_VERSIONS  # noqa: E402


def jalan_baru():
    with open(STUDIO_VERSIONS, encoding="utf-8") as handle:
        store = json.load(handle)
    with open(REFERENCE_JSON, encoding="utf-8") as handle:
        reference = json.load(handle)["offsets"]
    return next(v["params"] for v in store["versions"] if v["name"] == "JALAN BARU"), reference


def fake_trial(rock_deg=3.0, yaw_rate_dps=0.0, hip_swing=True, fall_at=None, hz=125.0):
    """Rekaman sintetis: 1 s berdiri, 10 s jalan, 1 s berhenti."""
    imu, joints = [], []
    marks = {"stand0": 0.0, "start": 1.0, "stop": 11.0, "end": 12.0, "fell_at": fall_at}
    for i in range(int(12 * hz)):
        t = i / hz
        walking = 1.0 <= t <= 11.0
        phase = 2 * math.pi * t / 0.65
        roll = rock_deg * math.sin(phase) if walking else 0.0
        gyro = math.radians(rock_deg) * (2 * math.pi / 0.65) * math.cos(phase) if walking else 0.0
        gz = math.radians(yaw_rate_dps) if walking else 0.0
        imu.append((t, gyro, 0.0, gz + 0.001, roll, 2.0, 0.0))  # bias gyro_z 0,001 rad/s
        joints.append((t, 0.8 + (0.1 * math.sin(phase) if walking and hip_swing else 0.0)))
    return imu, joints, marks


class SpaceTest(unittest.TestCase):
    def test_start_preset_is_inside_and_round_trips(self):
        start, reference = jalan_baru()
        space = core.Space(core.DEFAULT_PARAMS, start, reference)
        unit = space.to_unit(start)
        self.assertTrue(np.all(unit >= 0) and np.all(unit <= 1))
        back = space.to_params(unit)
        for name in core.DEFAULT_PARAMS:
            self.assertAlmostEqual(back[name], start[name], places=5)

    def test_forward_step_is_never_tuned(self):
        # Skornya cuma kestabilan: langkah maju yang boleh mengecil = jalan di tempat menang.
        self.assertNotIn("x_move_amplitude", core.DEFAULT_PARAMS)
        self.assertNotIn("x_move_amplitude", core.BOUNDS)

    def test_bias_bounds_are_relative_to_page2(self):
        start, reference = jalan_baru()
        space = core.Space(["init_x_offset"], start, reference)
        _, lo, hi = core.BOUNDS["init_x_offset"]
        self.assertLessEqual(space.lo[0], reference["init_x_offset"] + lo + 1e-12)
        self.assertGreaterEqual(space.hi[0], reference["init_x_offset"] + hi - 1e-12)

    def test_candidates_keep_every_other_field(self):
        start, reference = jalan_baru()
        space = core.Space(core.DEFAULT_PARAMS, start, reference)
        params = core.candidate_params(start, space, np.full(len(core.DEFAULT_PARAMS), 0.5))
        self.assertEqual(set(params), set(start))
        self.assertEqual(params["x_move_amplitude"], start["x_move_amplitude"])
        self.assertTrue(core.leg_reachable(params))


class CmaTest(unittest.TestCase):
    def test_converges_on_a_shifted_sphere_and_resumes_exactly(self):
        target = np.array([0.3, 0.7, 0.55, 0.2])
        es = core.CMAES(np.full(4, 0.5), 0.2, popsize=8, seed=3)
        for _ in range(40):
            xs = es.ask()
            es.tell(xs, [float(np.sum((x - target) ** 2)) for x in xs])
        self.assertLess(np.linalg.norm(es.mean - target), 0.02)
        # Lanjut dari JSON = sampel berikutnya identik (sesi bisa diputus di mana saja).
        restored = core.CMAES(np.zeros(4), 0.5, popsize=8, state=json.loads(json.dumps(es.state())))
        np.testing.assert_allclose(np.array(restored.ask()), np.array(es.ask()))

    def test_out_of_range_samples_are_penalised(self):
        self.assertEqual(core.bounded_cost([0.2, 0.9], 1.0), 1.0)
        self.assertGreater(core.bounded_cost([1.3, -0.2], 1.0), 1.0)


class MetricsTest(unittest.TestCase):
    def test_rocking_and_turning_are_measured(self):
        imu, joints, marks = fake_trial(rock_deg=3.0, yaw_rate_dps=1.0)
        m = core.compute_metrics(imu, joints, marks, 0.65, 0.65)
        self.assertTrue(m["valid"])
        # Kemiringan skor ditapis METRIC_TAU_S: ayunan langkah teredam 1/sqrt(1+(w*tau)^2).
        w = 2 * math.pi / 0.65
        expected = 3.0 / math.sqrt(2) / math.sqrt(1 + (w * core.METRIC_TAU_S) ** 2)
        self.assertAlmostEqual(m["tilt_std_deg"], expected, delta=0.1)
        # 1 deg/s selama 10 s; bias gyro berdiri dikurangi.
        self.assertAlmostEqual(m["yaw_drift_deg"], 10.0, delta=0.3)
        self.assertGreater(m["hip_swing_deg"], 1.0)

    def test_robot_that_did_not_walk_is_not_a_valid_stable_result(self):
        imu, joints, marks = fake_trial(rock_deg=0.0, hip_swing=False)
        self.assertFalse(core.compute_metrics(imu, joints, marks, 0.65, 0.65)["valid"])

    def test_steadier_gait_scores_better_and_falls_score_worst(self):
        base = core.compute_metrics(*fake_trial(rock_deg=4.0), 0.65, 0.65)
        scales = core.scales_from_baseline([base])
        calm = core.compute_metrics(*fake_trial(rock_deg=2.0), 0.65, 0.65)
        self.assertLess(core.cost(calm, scales), core.cost(base, scales))
        early = dict(base, fell=True, survived_frac=0.2)
        late = dict(base, fell=True, survived_frac=0.9)
        self.assertGreater(core.cost(early, scales), core.cost(late, scales))
        self.assertGreater(core.cost(late, scales), 3 * core.cost(base, scales))

    def test_fall_fraction_is_against_the_planned_walk(self):
        # Jatuh menghentikan percobaan seketika, jadi stop == saat jatuh; pecahan
        # bertahan harus dihitung terhadap lama jalan yang direncanakan.
        imu, joints, marks = fake_trial()
        marks.update(fell_at=3.0, stop=3.0, planned_stop=11.0)
        m = core.compute_metrics(imu, joints, marks, 0.65, 0.65)
        self.assertTrue(m["fell"])
        self.assertAlmostEqual(m["survived_frac"], 0.2)

    def test_impact_spikes_are_not_falls_but_real_tilts_are(self):
        # Sesi coba 23 Okt: roll mentah 55 deg sesaat saat jalan normal (hentakan kaki
        # di akselerometer) memicu "jatuh" palsu dengan EMA per-sampel yang lama.
        watch = core.FallWatch(seed=(-0.008, 3.0, 5.0))   # disemai dari pose berdiri
        t = 0.0
        for i in range(250):                       # 2 s jalan, lonjakan 55 deg tiap 0,3 s
            t = i / 125
            spike = 55.0 if i % 38 == 0 else 3.0
            self.assertFalse(watch.update(t, spike, 5.0), "lonjakan sesaat dianggap jatuh")
        fell_after = None
        for i in range(1, 250):                    # lalu badan benar-benar rebah ke depan
            if watch.update(t + i / 125, 2.0, 70.0):
                fell_after = i / 125
                break
        self.assertIsNotNone(fell_after)
        self.assertLess(fell_after, 0.6)           # tertangkap dalam waktu wajar

    def test_tilt_filter_time_constant_is_independent_of_rate(self):
        for hz in (50, 125, 250):
            tilt = core.TiltFilter(0.25)
            tilt.update(0.0, 0.0, 0.0)
            for i in range(1, int(0.25 * hz) + 1):
                roll, _ = tilt.update(i / hz, 10.0, 0.0)
            self.assertAlmostEqual(roll, 10.0 * (1 - math.exp(-1)), delta=0.3)

    def test_operator_label_is_the_final_word(self):
        base = core.compute_metrics(*fake_trial(rock_deg=4.0), 0.65, 0.65)
        scales = core.scales_from_baseline([base])
        normal = core.cost(base, scales)
        # Tertahan tali: detektor tidak melihat jatuh, operator melihat.
        self.assertGreaterEqual(core.cost(base, scales, label="jatuh"), 5.0)
        # Hampir jatuh lalu dipegang/diangkat: harus dihindari, walau IMU tampak biasa.
        self.assertEqual(core.cost(base, scales, label="hampir"), 5.0)
        # Alarm palsu dibatalkan operator: dinilai seperti jalan normal.
        alarm = dict(base, fell=True, survived_frac=0.9)
        self.assertAlmostEqual(core.cost(alarm, scales, label="normal"), normal)
        self.assertGreater(core.cost(alarm, scales), 5.0)

    def test_fall_replay_matches_marks(self):
        imu, joints, marks = fake_trial()
        self.assertIsNone(core.detect_fall(imu, marks))
        tipped = [s if s[0] < 6.0 else (s[0], s[1], s[2], s[3], s[4], 70.0, s[6]) for s in imu]
        when = core.detect_fall(tipped, marks)
        self.assertIsNotNone(when)
        self.assertGreaterEqual(when, 6.0)
        self.assertLess(when, 6.2)

    def test_unstable_stance_scores_like_the_worst_fall(self):
        self.assertEqual(core.cost({"stance_unstable": True, "fell": False}, {}), 10.0)

    def test_one_noisy_metric_cannot_dominate(self):
        scales = {k: 1.0 for k in core.WEIGHTS}
        calm = {"gyro_rms_dps": 1.0, "tilt_std_deg": 1.0, "yaw_drift_deg": 1.0, "lean_deg": 1.0}
        wild_yaw = dict(calm, yaw_drift_deg=40.0)
        self.assertLessEqual(core.cost(wild_yaw, scales) - core.cost(calm, scales),
                             core.WEIGHTS["yaw_drift_deg"] * (core.CLIP_YAW - 1.0) + 1e-9)
        wild_gyro = dict(calm, gyro_rms_dps=40.0)
        self.assertLessEqual(core.cost(wild_gyro, scales) - core.cost(calm, scales),
                             core.WEIGHTS["gyro_rms_dps"] * (core.CLIP - 1.0) + 1e-9)

    def test_bigger_turns_keep_costing_more(self):
        # jb1: belok kanan 30 dan 50 deg dulu dihukum sama (jepit 2x skala 13,7 deg).
        scales = {"gyro_rms_dps": 40.0, "tilt_std_deg": 2.0, "yaw_drift_deg": 13.7, "lean_deg": 2.0}
        walk = {"gyro_rms_dps": 40.0, "tilt_std_deg": 2.0, "lean_deg": 0.0}
        costs = [core.cost(dict(walk, yaw_drift_deg=y), scales) for y in (0.0, -20.0, -36.0, -50.0)]
        self.assertEqual(costs, sorted(costs))
        self.assertGreater(costs[3] - costs[2], 0.2)

    def test_turn_direction_follows_rep103_as_seen_on_the_robot(self):
        # jb1 percobaan 33-41: -35..-50 deg, operator melihat robot belok KANAN.
        self.assertEqual(core.turn_direction(-36.0), "kanan")
        self.assertEqual(core.turn_direction(+12.0), "kiri")
        self.assertEqual(core.turn_direction(-3.0), "lurus")
        self.assertEqual(core.turn_direction(None), "-")
        self.assertEqual(core.turn_text(-36.4), "kanan 36 deg")

    def test_quaternion_matches_controller_convention(self):
        half = math.radians(20) / 2
        roll, pitch, yaw = core.quat_to_rpy_deg(0.0, math.sin(half), 0.0, math.cos(half))
        self.assertAlmostEqual(pitch, 20.0, places=6)
        self.assertAlmostEqual(roll, 0.0, places=6)


class MotionTest(unittest.TestCase):
    def preset(self, x=0.0, y=0.0, a=0.0):
        return {"x_move_amplitude": x, "y_move_amplitude": y, "angle_move_amplitude": a}

    def test_motion_is_read_from_the_numbers_not_the_name(self):
        # Preset operator "KANAN MIN NEW" ber-y +0,01 = ke KIRI (REP-103), "MENCARI CELAH" a -0,2.
        self.assertEqual(core.walking_motion(self.preset(x=0.0145))[0], "maju")
        self.assertEqual(core.walking_motion(self.preset(x=-0.01))[0], "mundur")
        kind, text = core.walking_motion(self.preset(y=0.01))
        self.assertEqual((kind, text.split(" (")[0]), ("geser", "geser kiri"))
        kind, text = core.walking_motion(self.preset(y=0.025, a=-0.2))
        self.assertEqual((kind, text.split(" (")[0]), ("putar", "putar kanan"))
        # Koreksi belok kecil (rentang pencarian +-0,05) bukan preset putar.
        self.assertEqual(core.walking_motion(self.preset(x=0.0145, a=-0.03))[0], "maju")
        self.assertEqual(core.walking_motion(self.preset())[0], "di_tempat")

    def test_turning_preset_keeps_its_turn_and_locks_it(self):
        self.assertNotIn("angle_move_amplitude", core.motion_params("putar"))
        self.assertIn("angle_move_amplitude", core.motion_params("geser"))
        scales = core.scales_from_baseline([{"gyro_rms_dps": 30.0, "tilt_std_deg": 2.0, "lean_deg": 1.0,
                                             "yaw_drift_deg": 88.0}], yaw_target_deg=88.0)
        self.assertEqual(scales["yaw_drift_deg"], core.FLOORS["yaw_drift_deg"])
        walk = {"gyro_rms_dps": 30.0, "tilt_std_deg": 2.0, "lean_deg": 1.0}
        on_target = core.cost(dict(walk, yaw_drift_deg=88.0), scales, yaw_target_deg=88.0)
        straight = core.cost(dict(walk, yaw_drift_deg=0.0), scales, yaw_target_deg=88.0)
        self.assertLess(on_target, straight)
        # Sasaran 0 (bawaan) = perilaku lama persis.
        self.assertEqual(core.cost(dict(walk, yaw_drift_deg=-20.0), scales),
                         core.cost(dict(walk, yaw_drift_deg=-20.0), scales, yaw_target_deg=0.0))


if __name__ == "__main__":
    unittest.main()


class ReboundTest(unittest.TestCase):
    def test_widened_bounds_keep_the_search_at_the_same_real_values(self):
        import autotune_walking as runner
        start, reference = jalan_baru()
        names = ["period_time", "angle_move_amplitude"]
        space = core.Space(names, start, reference)
        i = names.index("angle_move_amplitude")
        old_lo, old_hi = space.lo.copy(), space.hi.copy()
        old_lo[i], old_hi[i] = -0.02, 0.02          # batas lama
        es = core.CMAES(np.array([0.5, 0.36]), 0.15, popsize=4, seed=2)
        pending = [x.tolist() for x in es.ask()]
        state = {"params": names, "start_params": start, "reference_offsets": reference,
                 "lo": old_lo.tolist(), "hi": old_hi.tolist(), "cma": es.state(), "pending": pending}
        native = lambda u, lo, hi: np.array(lo) + np.array(u) * (np.array(hi) - np.array(lo))
        before_mean = native(state["cma"]["mean"], state["lo"], state["hi"])
        before_pending = [native(u, state["lo"], state["hi"]) for u in pending]
        self.assertTrue(runner.rebound_session(state))
        self.assertAlmostEqual(state["lo"][i], -0.05)
        np.testing.assert_allclose(native(state["cma"]["mean"], state["lo"], state["hi"]), before_mean)
        for u, b in zip(state["pending"], before_pending):
            np.testing.assert_allclose(native(u, state["lo"], state["hi"]), b)
        self.assertFalse(runner.rebound_session(state))   # kedua kali: tidak ada yang berubah
