"""Source contracts for per-map independent sensor masks and speed profiles."""
import unittest
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]

class IndependentMapSettingsTest(unittest.TestCase):
    def test_independent_mask_arrays_and_defaults(self):
        c=(ROOT/"src/pid_controller.cpp").read_text(encoding="utf-8")
        self.assertIn("mapSensorThresholds[3][2]",c)
        self.assertIn("mapSensorMasks[3][3]",c)
        self.assertIn("{0xFFC0U,0x03FFU,0xFFFFU}",c)
        self.assertEqual(c.count("{0xFFFFU,0xFFFFU,0xFFFFU}"),2)
        self.assertIn("mapSensorMasks[i][0]=m1",c)
        self.assertIn("selectedMap-1U",c)
        self.assertIn("const uint16_t pidSensorMask = activePidSensorMask();",c)
        self.assertIn("mapSegmentPwm[3][8]",c)
    def test_protocol_and_legacy(self):
        c=(ROOT/"src/main.cpp").read_text(encoding="utf-8")
        self.assertIn('s.startsWith("MAP_SENSOR_PROFILE:")',c)
        self.assertIn('s.substring(19)',c)
        self.assertIn('pid_set_map_sensor_profile',c)
        self.assertIn('if(s=="SENSOR_PROFILE")',c)
    def test_gui_and_map_specific_cache(self):
        c=(ROOT/"robot_debug.py").read_text(encoding="utf-8")
        self.assertIn("self.sensor_profiles = {",c)
        self.assertIn("self.sensor_profile_pending = (target, a, b, *values)",c)
        self.assertIn('f"MAP_SENSOR_PROFILE:{target},{a},{b},"',c)
        self.assertIn("self.read_map_sensor_profile(target)",c)
    def test_header_exposes_map_specific_api(self):
        c=(ROOT/"include/pid_controller.h").read_text(encoding="utf-8")
        self.assertIn("pid_set_map_sensor_profile",c)
        self.assertIn("pid_get_map_sensor_profile",c)

if __name__=="__main__":
    unittest.main()
