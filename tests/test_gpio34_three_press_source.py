"""Regression checks: physical GPIO34 button removal must preserve SPP control.

These source-level invariants do not replace an ESP32 build.
Run: python -m unittest discover -s tests -p 'test_gpio34_three_press_source.py'
"""
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAIN = (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
SENSOR = (ROOT / "src" / "sensor.cpp").read_text(encoding="utf-8")
HEADER = (ROOT / "include" / "sensor.h").read_text(encoding="utf-8")
PINOUT = (ROOT / "include" / "pinout.h").read_text(encoding="utf-8")
PID = (ROOT / "src" / "pid_controller.cpp").read_text(encoding="utf-8")


class NoPhysicalButtonSourceTests(unittest.TestCase):
    def test_no_gpio_button_input_or_button_state_machine(self):
        for content in (MAIN, SENSOR, HEADER, PINOUT):
            self.assertNotIn("PIN_START_BUTTON", content)
            self.assertNotIn("BUTTON34_", content)
            self.assertNotIn("sensor_start_button_pressed", content)
        self.assertNotIn("button34Phase", MAIN)
        self.assertNotIn("lastButtonRead", SENSOR)
        self.assertNotIn("BUTTON_DEBOUNCE_MS", HEADER)
        self.assertNotIn("pinMode(34", MAIN + SENSOR)
        self.assertNotIn("pinMode(35", MAIN + SENSOR)

    def test_spp_calibration_and_auto_remain_available(self):
        for command in ('s=="CAL_WHITE"', 's=="CAL_BLACK"', 's=="AUTO"',
                        's=="STOP"', 's=="ESTOP"', 's=="RESET_ESTOP"'):
            self.assertIn(command, MAIN)
        for event in ("CAL_WHITE_DONE", "CAL_OK_SAVED", "CAL_FAILED"):
            self.assertIn(event, MAIN)
        self.assertIn("sensor_update_calibration_loop()", MAIN)
        self.assertIn("sensor_read_all()", MAIN)
        self.assertIn("pid_set_auto_mode(true);", MAIN)

    def test_led_and_finish_detection_are_preserved(self):
        self.assertIn("PIN_DEBUG_LED", MAIN)
        self.assertIn("void updateLedIndicator(uint32_t now)", MAIN)
        self.assertIn("CAL_WHITE_LED_BLINK_MS=40", MAIN)
        self.assertIn("CAL_BLACK_LED_BLINK_MS=90", MAIN)
        self.assertIn("state==CAL_WHITE_RUNNING", MAIN)
        self.assertIn("state==CAL_BLACK_RUNNING", MAIN)
        self.assertIn("updateLedIndicator(now);", MAIN)
        self.assertIn("START_PAD_EXIT_CONFIRM_MS = 30", PID)
        self.assertIn("finishDetectionArmed = false;", PID)
        self.assertIn("FINISH_CONFIRM_FRAMES = 3", PID)


if __name__ == "__main__":
    unittest.main()
