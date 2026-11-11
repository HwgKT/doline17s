"""Validation tests for the editable per-map eight-stage profile."""
import unittest
from pathlib import Path

from robot_debug import validate_map1_speed_profile


ROOT = Path(__file__).resolve().parents[1]


class Map1SpeedProfileTests(unittest.TestCase):
    def test_accepts_valid_seven_thresholds_and_eight_base_values(self):
        ticks, bases = validate_map1_speed_profile(
            [3800, 7800, 8350, 10300, 11400, 12800, 15000],
            [140, 140, 140, 240, 140, 140, 210, 255],
        )
        self.assertEqual(ticks[0], 3800)
        self.assertEqual(ticks[-1], 15000)
        self.assertEqual(len(bases), 8)

    def test_rejects_wrong_number_of_fields(self):
        with self.assertRaises(ValueError):
            validate_map1_speed_profile([100, 200], [140] * 8)
        with self.assertRaises(ValueError):
            validate_map1_speed_profile([100] * 7, [140] * 7)

    def test_rejects_nonascending_or_duplicate_ticks(self):
        for ticks in (
            [100, 200, 300, 400, 500, 600, 600],
            [100, 200, 300, 250, 500, 600, 700],
            [0, 200, 300, 400, 500, 600, 700],
            [100, 200, 300, 400, 500, 600, 10_000_001],
        ):
            with self.subTest(ticks=ticks), self.assertRaises(ValueError):
                validate_map1_speed_profile(ticks, [140] * 8)

    def test_rejects_base_outside_firmware_range(self):
        for bad_base in (44, 256):
            bases = [140] * 8
            bases[3] = bad_base
            with self.subTest(base=bad_base), self.assertRaises(ValueError):
                validate_map1_speed_profile(
                    [100, 200, 300, 400, 500, 600, 700], bases)

    def test_firmware_exposes_independent_profiles_for_all_maps(self):
        pid = (ROOT / "src" / "pid_controller.cpp").read_text(encoding="utf-8")
        header = (ROOT / "include" / "pid_controller.h").read_text(encoding="utf-8")
        main = (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        self.assertIn("int32_t mapSegmentTicks[3][7]", pid)
        self.assertIn("uint8_t mapSegmentPwm[3][8]", pid)
        self.assertIn("pid_set_map_speed_profile", pid)
        self.assertIn("pid_set_map_speed_profile", header)
        self.assertIn("MAP_SPEED_PROFILE:", main)
        self.assertIn("MAP_SPEED_PROFILE MAP=", main)
        self.assertIn("selectedMap >= 1 && selectedMap <= 3", pid)

    def test_firmware_keeps_separate_default_profiles_per_map(self):
        pid = (ROOT / "src" / "pid_controller.cpp").read_text(encoding="utf-8")
        self.assertIn("{140, 140, 140, 240, 140, 140, 210, 255}", pid)
        self.assertIn("{160, 160, 160, 160, 160, 160, 160, 160}", pid)
        self.assertIn("{150, 150, 150, 150, 150, 150, 150, 150}", pid)
        self.assertIn("mapSegmentTicks[mapIndex][i] = xTicks[i]", pid)
        self.assertIn("mapSegmentPwm[mapIndex][i] = basePwm[i]", pid)

    def test_gui_sends_tick_and_base_values_in_one_command(self):
        app = (ROOT / "robot_debug.py").read_text(encoding="utf-8")
        self.assertIn('command = "MAP_SPEED_PROFILE:"', app)
        self.assertIn("validate_map1_speed_profile(ticks, bases)", app)
        self.assertIn('self.read_map_speed_profile(initial_map)', app)
        self.assertIn("self.map_speed_profiles = {", app)


if __name__ == "__main__":
    unittest.main()
