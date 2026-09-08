import math
from pathlib import Path
import unittest

from bascorro_studio.kick_launch import (
    KICK_LAUNCH_KEYS, format_launch_float, read_kick_launch_defaults,
    rewrite_kick_launch_defaults,
)


LAUNCH_PATH = Path(__file__).resolve().parents[2] / "ROBOTIS-OP3-Demo/op3_demo/launch/demo.launch.xml"


class KickLaunchTest(unittest.TestCase):
    def setUp(self):
        self.original = LAUNCH_PATH.read_text(encoding="utf-8")
        self.values = read_kick_launch_defaults(self.original)

    def test_reads_all_defaults(self):
        self.assertEqual(set(self.values), set(KICK_LAUNCH_KEYS))

    def test_unchanged_values_preserve_every_byte(self):
        updated, changed, error = rewrite_kick_launch_defaults(self.original, self.values)
        self.assertEqual(updated, self.original)
        self.assertEqual(changed, [])
        self.assertEqual(error, "")

    def test_only_selected_default_changes_and_comments_survive(self):
        values = {**self.values, "kick_distance": 0.31}
        updated, changed, error = rewrite_kick_launch_defaults(self.original, values)
        self.assertFalse(error)
        self.assertEqual(len(changed), 1)
        self.assertEqual(read_kick_launch_defaults(updated)["kick_distance"], 0.31)
        old_token = '<arg name="kick_distance" default="0.15"/>'
        self.assertEqual(updated, self.original.replace(old_token, '<arg name="kick_distance" default="0.31"/>'))

    def test_invalid_values_never_produce_a_partial_update(self):
        for invalid in [math.nan, math.inf, -1, "0.3", True]:
            values = {**self.values, "kick_camera_height": invalid, "kick_distance": 0.31}
            updated, changed, error = rewrite_kick_launch_defaults(self.original, values)
            self.assertTrue(error)
            self.assertEqual(updated, self.original)
            self.assertEqual(changed, [])

    def test_missing_or_duplicate_arguments_are_rejected(self):
        broken = self.original.replace('name="kick_distance"', 'name="not_kick_distance"')
        duplicate = self.original.replace("</launch>", '<arg name="kick_distance" default="0.3"/></launch>')
        for text in [broken, duplicate, "<launch>"]:
            with self.assertRaises(Exception):
                read_kick_launch_defaults(text)

    def test_camera_pitch_round_trip_and_invalid_values(self):
        for offset in [-60.0, 0.0, 30.84, 60.0]:
            updated, _, error = rewrite_kick_launch_defaults(
                self.original, {**self.values, "kick_camera_pitch_offset_deg": offset})
            self.assertFalse(error)
            self.assertEqual(offset, read_kick_launch_defaults(updated)["kick_camera_pitch_offset_deg"])
        for offset in [-60.01, 60.01, math.nan, math.inf, True]:
            updated, changed, error = rewrite_kick_launch_defaults(
                self.original, {**self.values, "kick_camera_pitch_offset_deg": offset})
            self.assertTrue(error)
            self.assertEqual(updated, self.original)
            self.assertEqual(changed, [])

    def test_launch_float_is_finite_and_keeps_a_decimal_mantissa(self):
        for value in [4, 0.3, 1e-12, 1e10]:
            rendered = format_launch_float(value)
            self.assertIn(".", rendered.partition("e")[0])
            self.assertEqual(float(rendered), value)
        with self.assertRaises(ValueError):
            format_launch_float(math.nan)


if __name__ == "__main__":
    unittest.main()
