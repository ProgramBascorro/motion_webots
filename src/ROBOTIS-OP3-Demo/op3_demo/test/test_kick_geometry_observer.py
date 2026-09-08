import importlib.util
import copy
import math
from pathlib import Path
import unittest


SCRIPT = Path(__file__).resolve().parents[4] / "scripts" / "observe_kick_geometry.py"
SPEC = importlib.util.spec_from_file_location("observe_kick_geometry", SCRIPT)
OBSERVER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(OBSERVER)


class KickGeometryObserverTest(unittest.TestCase):
    def setUp(self):
        self.parameters = {
            "kick_camera_height": 0.56,
            "kick_ball_real_radius_m": 0.11,
            "kick_head_tilt_sign": 1.0,
            "kick_min_depression_deg": 3.0,
            "kick_pan_window_from_bearing": 1.0,
            "head_pan_servo_center_deg": 180.0,
            "head_pan_servo_dir": 1.0,
            "kick_pan_right_min_deg": 152.0,
            "kick_pan_right_max_deg": 178.0,
            "kick_pan_left_min_deg": 193.0,
            "kick_pan_left_max_deg": 208.0,
        }
        self.head = {"pan": math.radians(15.0), "tilt": 0.14 - math.pi / 4.0}

    def test_center_ray_uses_height_above_ball_center(self):
        result = OBSERVER.ball_geometry([0.0, 0.0, 80.0], self.head, 0.14, self.parameters)
        self.assertAlmostEqual(0.45, result["distance_m"])
        self.assertAlmostEqual(45.0, result["depression_deg"])
        self.assertAlmostEqual(15.0, result["bearing_deg"])
        self.assertAlmostEqual(195.0, result["servo_window_deg"])
        self.assertEqual("left", result["foot_window"])

    def test_image_offset_changes_range_and_foot(self):
        result = OBSERVER.ball_geometry([0.65, 0.5, 80.0], self.head, 0.14, self.parameters)
        self.assertLess(result["distance_m"], 0.45)
        self.assertLess(result["bearing_deg"], 0.0)
        self.assertEqual("right", result["foot_window"])
        farther_side = OBSERVER.ball_geometry([0.8, 0.5, 80.0], self.head, 0.14, self.parameters)
        self.assertEqual("outside", farther_side["foot_window"])

    def test_height_override_does_not_change_runtime_parameters(self):
        result = OBSERVER.ball_geometry([0.0, 0.0, 80.0], self.head, 0.14,
                                        self.parameters, camera_height=0.52)
        self.assertAlmostEqual(0.41, result["distance_m"])
        self.assertEqual(0.56, self.parameters["kick_camera_height"])

    def test_horizon_and_impossible_height_have_no_distance(self):
        result = OBSERVER.ball_geometry([0.0, 0.0, 80.0], {"pan": 0.0, "tilt": 0.14},
                                        0.14, self.parameters)
        self.assertIsNone(result["distance_m"])
        result = OBSERVER.ball_geometry([0.0, 0.0, 80.0], self.head, 0.14,
                                        self.parameters, camera_height=0.1)
        self.assertIsNone(result["distance_m"])

    def test_invalid_detection_is_rejected(self):
        self.assertIsNone(OBSERVER.ball_geometry([0.0, 0.0, 0.0], self.head, 0.14, self.parameters))
        self.assertIsNone(OBSERVER.ball_geometry([math.nan, 0.0, 80.0], self.head, 0.14, self.parameters))

    def test_matches_head_at_image_time_instead_of_reception(self):
        heads = [{"stamp": 1.0}, {"stamp": 1.05}, {"stamp": 1.8}]
        stamps = [head["stamp"] for head in heads]
        self.assertEqual(heads[1], OBSERVER.nearest_head(heads, stamps, 1.04))
        self.assertIsNone(OBSERVER.nearest_head(heads, stamps, 1.4))
        self.assertIsNone(OBSERVER.nearest_head([], [], 1.04))

    def test_statistics_distinguish_missing_measurements(self):
        self.assertEqual({"count": 0}, OBSERVER.describe([None, math.nan, math.inf]))
        result = OBSERVER.describe([None, 0.5, 0.7, 0.3])
        self.assertEqual(3, result["count"])
        self.assertEqual(0.5, result["median"])
        self.assertEqual(0.3, result["min"])

    def test_missing_head_samples_are_not_reported_as_valid_distances(self):
        recording = {"parameters": self.parameters, "walking": {"hip_pitch_offset": 0.14},
                     "frames": [{"stamp": 1.0, "received": 1.8, "circles": [[0.0, 0.0, 80.0]]}],
                     "goal": [], "present": [], "imu": []}
        result = OBSERVER.summarize(recording)
        self.assertEqual(1, result["frames_with_ball"])
        self.assertEqual(0, result["goal"]["distance_m"]["count"])
        self.assertEqual(0, result["present"]["distance_m"]["count"])
        self.assertAlmostEqual(800.0, result["detector_age_ms"]["median"])

    def recorded_contact(self):
        head = {"stamp": 1.0, "pan": -0.01227184630308513, "tilt": -0.521553467881118}
        frame = {"stamp": 1.0, "received": 1.85,
                 "circles": [[0.1358221054077149, 0.190411885579427, 81.51219177246094]]}
        return {"parameters": dict(self.parameters), "walking": {"hip_pitch_offset": 0.1395999938249588},
                "frames": [{**frame, "stamp": 1.0 + index * 0.01} for index in range(3)],
                "goal": [head], "present": [head], "imu": []}

    def test_confirmed_manual_measurements_fit_recorded_right_contact(self):
        recording = self.recorded_contact()
        original = copy.deepcopy(recording)
        result = OBSERVER.calibrate_recording(recording, 0.52, 0.134, 0.54)
        self.assertEqual(recording, original)
        self.assertAlmostEqual(0.1456021978, result["reference"]["horizontal_distance_m"])
        self.assertAlmostEqual(0.4088245475, result["reference"]["predicted_camera_to_nearest_surface_m"])
        self.assertEqual(0.067, result["geometry_parameters"]["kick_ball_real_radius_m"])
        self.assertAlmostEqual(30.84, result["geometry_parameters"]["kick_camera_pitch_offset_deg"], delta=0.02)
        self.assertAlmostEqual(0.1456, result["replay_summary"]["goal"]["distance_m"]["median"], delta=0.001)
        self.assertAlmostEqual(161.19, result["replay_summary"]["goal"]["servo_window_deg"]["median"], delta=0.1)
        self.assertEqual(3, result["replay_summary"]["goal"]["foot_windows"]["right"])

    def test_calibration_rejects_impossible_or_missing_measurements(self):
        for measurements in [(0.52, 0.134, 0.51), (0.52, 0.134, 0.52), (0.05, 0.134, 0.54),
                             (math.nan, 0.134, 0.54), (0.52, 0.0, 0.54), (0.52, 0.134, math.inf)]:
            with self.assertRaises(ValueError):
                OBSERVER.calibrate_recording(self.recorded_contact(), *measurements)
        recording = self.recorded_contact()
        recording["goal"] = []
        with self.assertRaises(ValueError):
            OBSERVER.calibrate_recording(recording, 0.52, 0.134, 0.54)

    def test_invalid_pitch_or_impossible_ray_is_not_a_measurement(self):
        for offset in [-60.01, 60.01, math.nan, math.inf]:
            parameters = {**self.parameters, "kick_camera_pitch_offset_deg": offset}
            self.assertIsNone(OBSERVER.ball_geometry([0.0, 0.0, 80.0], self.head, 0.14, parameters))
        self.assertIsNone(OBSERVER.fit_camera_pitch([1.0, 0.0, 80.0], self.head, 0.14,
                                                    self.parameters, 0.01))

    def test_fit_is_absolute_not_added_twice_to_an_existing_offset(self):
        recording = self.recorded_contact()
        initial = OBSERVER.calibrate_recording(recording, 0.52, 0.134, 0.54)
        recording["parameters"].update(initial["geometry_parameters"])
        repeated = OBSERVER.calibrate_recording(recording, 0.52, 0.134, 0.54)
        self.assertEqual(initial["geometry_parameters"], repeated["geometry_parameters"])


if __name__ == "__main__":
    unittest.main()
