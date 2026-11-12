"""Map 1 physical GPIO34/GPIO35 button workflow source-level regression checks.

Run with:
python -m unittest discover -s tests -p 'test_gpio34_three_press_source.py'
Requires separate ESP32 build and hardware test.
"""
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAIN = (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
SENSOR = (ROOT / "src" / "sensor.cpp").read_text(encoding="utf-8")
PINOUT = (ROOT / "include" / "pinout.h").read_text(encoding="utf-8")
PID = (ROOT / "src" / "pid_controller.cpp").read_text(encoding="utf-8")
PID_HEADER = (ROOT / "include" / "pid_controller.h").read_text(encoding="utf-8")


class PhysicalButtonsMap1Tests(unittest.TestCase):
    def test_gpio_pins_external_pullups_and_debounce(self):
        self.assertIn("#define PIN_CAL_BUTTON    34", PINOUT)
        self.assertIn("#define PIN_SELECT_BUTTON 35", PINOUT)
        self.assertIn("pinMode(PIN_CAL_BUTTON, INPUT);", MAIN)
        self.assertIn("pinMode(PIN_SELECT_BUTTON, INPUT);", MAIN)
        self.assertIn("BUTTON_DEBOUNCE_MS=50", MAIN)
        self.assertIn("BUTTON_HOLD_MS=1000", MAIN)
        self.assertIn("current==LOW", MAIN)
        self.assertNotIn("PIN_START_BUTTON", MAIN + PINOUT + SENSOR)

    def test_calibration_is_sequential_and_required(self):
        self.assertIn("PHYS_NEED_WHITE", MAIN)
        self.assertIn("PHYS_NEED_BLACK", MAIN)
        self.assertIn('command("CAL_WHITE")', MAIN)
        self.assertIn('command("CAL_BLACK")', MAIN)
        self.assertIn("case CAL_WHITE_DONE:", MAIN)
        self.assertIn("physicalPhase=PHYS_NEED_BLACK;", MAIN)
        self.assertIn("case CAL_SAVED:", MAIN)
        self.assertIn("physicalPhase=PHYS_SELECT_SIDE;", MAIN)
        self.assertIn("case CAL_ERROR:", MAIN)
        self.assertIn("physicalPhase=PHYS_NEED_WHITE;", MAIN)

    def test_confirmation_cancellation_and_start_modes(self):
        self.assertIn("physicalSide==SIDE_LEFT?SIDE_RIGHT:SIDE_LEFT", MAIN)
        self.assertIn("if(cal.longPress && physicalSide!=SIDE_NONE)", MAIN)
        self.assertIn("if(side.longPress)", MAIN)
        self.assertIn('reply("BUTTON_SIDE_CLEARED")', MAIN)
        self.assertIn("else if(cal.shortPress) startPhysicalMap1(false);", MAIN)
        self.assertIn("else if(side.shortPress) startPhysicalMap1(true);", MAIN)
        self.assertIn('command("AUTO:1")', MAIN)
        # STOP must clear the chosen side and publish the cleared state to SPP.
        self.assertRegex(MAIN, r"if\\(physicalRunInProgress\\)\\s*\\{\\s*resetPhysicalSelection\\(\\);\\s*reportPhysicalSide\\(\\);")
        self.assertIn("physicalPhase=PHYS_RUNNING;", MAIN)
        self.assertIn("if(physicalPhase!=PHYS_READY", MAIN)
        self.assertIn("if(pid_is_auto_mode() || manualActive", MAIN)

    def test_masks_are_run_scoped_and_gui_profile_unchanged(self):
        self.assertIn("pid_set_map1_button_mode(bool left, bool fullEyes)", PID_HEADER)
        self.assertIn("if (!enabled) map1ButtonOverride = false;", PID)
        self.assertIn("if (map1ButtonFullEyes) return 0xFFFFU;", PID)
        self.assertIn("MAP1_BUTTON_TICK1 = 650", PID)
        self.assertIn("MAP1_BUTTON_TICK2 = 1500", PID)
        self.assertIn("map1ButtonLeft ? 0x03FFU : 0xFFC0U", PID)
        self.assertIn("map1ButtonLeft ? 0xFFC0U : 0x03FFU", PID)
        self.assertIn("map1SensorStageMasks[0] = mask1;", PID)
        self.assertIn("map1SensorStageMasks[1] = mask2;", PID)
        self.assertIn("map1SensorStageMasks[2] = mask3;", PID)

    def test_led_is_nonblocking_and_has_all_patterns(self):
        self.assertIn("ledPulseHalfMs=rhythmMs/2U;", MAIN)
        for pattern in ("announcePhysical(1,500)", "announcePhysical(2,500)",
                        "announcePhysical(3,500)", "announcePhysical(3,500)",
                        "announcePhysical(physicalSide==SIDE_LEFT?1:2,1000)"):
            self.assertIn(pattern, MAIN)
        self.assertIn("void updateLedIndicator(uint32_t now)", MAIN)
        self.assertIn("ledPulseSteps", MAIN)
        self.assertIn("state==CAL_WHITE_RUNNING", MAIN)
        self.assertIn("state==CAL_BLACK_RUNNING", MAIN)

    def test_spp_and_finish_not_removed(self):
        for cmd in ('s=="CAL_WHITE"', 's=="CAL_BLACK"', 's=="AUTO"',
                    's=="STOP"', 's=="ESTOP"', 's=="RESET_ESTOP"'):
            self.assertIn(cmd, MAIN)
        self.assertIn("sensor_update_calibration_loop()", MAIN)
        self.assertIn("sensor_read_all()", MAIN)
        self.assertIn("FINISH_CONFIRM_FRAMES = 3", PID)


if __name__ == "__main__":
    unittest.main()
