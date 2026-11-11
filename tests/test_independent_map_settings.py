"""Source contracts for independent variable-stage MAP configuration."""
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class IndependentMapSettingsTest(unittest.TestCase):
    def test_sensor_profiles_are_variable_and_map_specific(self):
        c = (ROOT / "src/pid_controller.cpp").read_text(encoding="utf-8")
        self.assertIn("mapSensorEndTicks[3][MAP_PROFILE_MAX_STAGES - 1]", c)
        self.assertIn("mapSensorMasks[3][MAP_PROFILE_MAX_STAGES]", c)
        self.assertIn("mapSensorStageCount[3] = {3, 3, 3}", c)
        self.assertIn("{0xFFC0U, 0x03FFU, 0xFFFFU", c)
        self.assertIn("pid_set_map_sensor_profile_v2", c)
        self.assertIn("mapSensorMasks[index][i]=masks[i]", c)
        self.assertIn("activePidSensorMask()", c)

    def test_speed_profiles_are_variable_and_map_specific(self):
        c = (ROOT / "src/pid_controller.cpp").read_text(encoding="utf-8")
        self.assertIn("mapSegmentTicks[3][MAP_PROFILE_MAX_STAGES - 1]", c)
        self.assertIn("mapSegmentPwm[3][MAP_PROFILE_MAX_STAGES]", c)
        self.assertIn("mapSegmentStageCount[3] = {8, 8, 8}", c)
        self.assertIn("pid_set_map_speed_profile_v2", c)
        self.assertIn("mapSegmentStageCount[index]=stageCount", c)
        self.assertIn("while (segment + 1U < stageCount", c)

    def test_protocol_supports_dynamic_sensor_and_speed_stage_counts(self):
        c = (ROOT / "src/main.cpp").read_text(encoding="utf-8")
        self.assertIn('s.startsWith("MAP_SENSOR_PROFILE_V2:")', c)
        self.assertIn('s.startsWith("MAP_SPEED_PROFILE_V2:")', c)
        self.assertIn("PID_MAP_PROFILE_MAX_STAGES", c)
        self.assertIn("LAST_STAGE_MUST_FINISH", c)
        self.assertIn("parseMapProfileCsv", c)
        self.assertIn('if(s.startsWith("MAP_SENSOR_PROFILE:"))', c)
        self.assertIn('if(s.startsWith("MAP_SPEED_PROFILE:"))', c)

    def test_sensor_editor_has_stage_count_and_sixteen_eye_controls(self):
        c = (ROOT / "robot_debug.py").read_text(encoding="utf-8")
        self.assertIn('text="Số giai đoạn:"', c)
        self.assertIn('for bit, state in enumerate(states)', c)
        self.assertIn('text=f"S{bit}"', c)
        self.assertIn('MAP_SENSOR_PROFILE_V2:{target},{count}', c)
        self.assertIn("self.sensor_profiles[target]", c)

    def test_base_editor_has_stage_count_and_common_base(self):
        c = (ROOT / "robot_debug.py").read_text(encoding="utf-8")
        self.assertIn('text="Số giai đoạn:"', c)
        self.assertIn("BASE chung (45–255)", c)
        self.assertIn("def apply_common_base():", c)
        self.assertIn("for var in base_vars:", c)
        self.assertIn('MAP_SPEED_PROFILE_V2:', c)
        self.assertIn("end_tick, base = int(fields[2+2*i]), int(fields[3+2*i])", c)

    def test_start_requires_active_map_profiles_to_be_confirmed(self):
        c = (ROOT / "robot_debug.py").read_text(encoding="utf-8")
        self.assertIn("self.sensor_profile_confirmed.get(self.selected_map, False)", c)
        self.assertIn("self.map_profile_confirmed.get(self.selected_map, False)", c)
        self.assertIn("self.read_map_speed_profile(map_id)", c)

    def test_header_exposes_v2_apis_and_legacy_apis(self):
        c = (ROOT / "include/pid_controller.h").read_text(encoding="utf-8")
        self.assertIn("pid_set_map_sensor_profile_v2", c)
        self.assertIn("pid_get_map_sensor_profile_v2", c)
        self.assertIn("pid_set_map_speed_profile_v2", c)
        self.assertIn("pid_get_map_speed_profile_v2", c)
        self.assertIn("pid_set_map1_base_profile", c)


if __name__ == "__main__":
    unittest.main()
