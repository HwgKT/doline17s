"""Source-level regression tests for the Map 1 two-button state machine.

Run locally:
    python -m unittest discover -s tests -p test_gpio34_three_press_source.py

This checks the contract; it does not replace an ESP32 firmware build or a
physical GPIO/LED/Bluetooth acceptance test.
"""
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAIN = (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
PID = (ROOT / "src" / "pid_controller.cpp").read_text(encoding="utf-8")
PID_H = (ROOT / "include" / "pid_controller.h").read_text(encoding="utf-8")
APP = (ROOT / "robot_debug.py").read_text(encoding="utf-8")
PINOUT = (ROOT / "include" / "pinout.h").read_text(encoding="utf-8")


class Map1TwoButtonTests(unittest.TestCase):
    def test_no_unresolved_merge_markers(self):
        for text in (MAIN, PID, PID_H, APP, PINOUT):
            self.assertNotRegex(text, r"(?m)^(?:<<<<<<< |=======\s*$|>>>>>>> )")

    def test_physical_pins_and_nonblocking_debounce(self):
        self.assertRegex(PINOUT, r"#define PIN_START_BUTTON\s+34")
        self.assertRegex(PINOUT, r"#define PIN_UNUSED_BUTTON\s+35")
        self.assertIn("#define PIN_CAL_BUTTON PIN_START_BUTTON", PINOUT)
        self.assertIn("#define PIN_SELECT_BUTTON PIN_UNUSED_BUTTON", PINOUT)
        self.assertIn("PHYSICAL_SIDE_BUTTON_DEBOUNCE_MS = 50", MAIN)
        self.assertIn("BUTTON_HOLD_MS = 1000", MAIN)
        self.assertIn("pollPhysicalButton(calButton,now)", MAIN)
        self.assertIn("pollPhysicalButton(selectButton,now)", MAIN)

    def test_fresh_white_then_black_calibration_is_required(self):
        self.assertIn("button34Phase=BUTTON34_WHITE_NEXT", MAIN)
        self.assertIn('if(cal.shortPress) command("CAL_WHITE");', MAIN)
        self.assertIn('if(cal.shortPress) command("CAL_BLACK");', MAIN)
        self.assertIn("bool physicalCalibrationReady()", MAIN)
        self.assertIn("sensor_calibration_state()==CAL_READY", MAIN)
        self.assertIn("if(sensor_is_calibrating()||!physicalCalibrationReady())", MAIN)
        self.assertIn("!physicalCalibrationReady() || physicalSide==SIDE_NONE", MAIN)
        self.assertIn('reply("ERR: CALIB_REQUIRED_OR_PENDING")', MAIN)
        self.assertIn("case CAL_ERROR:", MAIN)
        self.assertIn("setButton34Phase(BUTTON34_WHITE_NEXT)", MAIN)
        self.assertIn("case CAL_SAVED:", MAIN)
        self.assertIn("setButton34Phase(BUTTON34_START_NEXT)", MAIN)

    def test_led_acknowledgements_match_v2(self):
        self.assertIn("CAL_WHITE_LED_BLINK_MS=40", MAIN)
        self.assertIn("CAL_BLACK_LED_BLINK_MS=90", MAIN)
        self.assertIn("announcePhysical(1,500)", MAIN)  # WHITE success
        self.assertIn("announcePhysical(2,500)", MAIN)  # BLACK success
        self.assertIn("announcePhysical(physicalSide==SIDE_LEFT?1:2,1000)", MAIN)
        self.assertIn("announcePhysical(3,500)", MAIN)
        self.assertIn("ledPulseSteps=(uint8_t)(pulses*2U)", MAIN)
        self.assertIn("ledPulseHalfMs=rhythmMs/2U", MAIN)
        self.assertIn("void updateLedIndicator(uint32_t now)", MAIN)
        self.assertIn("const bool runLed=pid_is_auto_mode()", MAIN)
        # No blocking delay() in the LED updater.
        updater = MAIN.split("void updateLedIndicator(uint32_t now)", 1)[1].split(
            "void sendFrame()", 1)[0]
        self.assertNotIn("delay(", updater)

    def test_left_right_confirmation_and_clear(self):
        self.assertIn("physicalSide=physicalSide==SIDE_LEFT?SIDE_RIGHT:SIDE_LEFT", MAIN)
        self.assertIn("if(cal.longPress && physicalSide!=SIDE_NONE)", MAIN)
        self.assertIn("if(side.longPress && physicalSide!=SIDE_NONE)", MAIN)
        self.assertIn("physicalSideConfirmed=true", MAIN)
        for token in ("SIDE:LEFT", "SIDE:RIGHT", "SIDE:CLEAR", "SIDE:STATUS",
                      "BUTTON_STATE phase=", "BUTTON_SIDE_CONFIRMED:",
                      "BUTTON_SIDE_CLEARED", "BUTTON_PROFILE side="):
            self.assertIn(token, MAIN)

    def test_short_start_buttons_have_independent_eye_modes(self):
        self.assertIn("else if(cal.shortPress) startPhysicalMap1(false)", MAIN)
        self.assertIn("else if(side.shortPress) startPhysicalMap1(true)", MAIN)
        self.assertIn('command("AUTO:1")', MAIN)
        self.assertIn('fullEyes?"FULL16":"TICK_MASK"', MAIN)
        self.assertIn("pid_set_map1_button_mode(physicalSide==SIDE_LEFT,fullEyes)", MAIN)
        self.assertIn("pid_clear_map1_button_mode();", MAIN)
        self.assertIn("void pid_clear_map1_button_mode(void)", PID_H)
        self.assertIn("if (map1ButtonFullEyes) return 0xFFFFU", PID)
        self.assertIn("map1ButtonLeft ? 0x03FFU : 0xFF80U", PID)
        self.assertIn("map1ButtonLeft ? 0xFF80U : 0x03FFU", PID)
        self.assertIn("map1SensorMaxTicks < 650", PID)
        self.assertIn("map1SensorMaxTicks < 1500", PID)

    def test_map1_defaults_and_other_maps_preserved(self):
        self.assertIn("pid_init(1.6f,0.0f,0.8f,140)", MAIN)
        self.assertIn("MAP_DEFAULT_BASE_PWM[3] = {140, 160, 150}", PID)
        self.assertIn("MAP_DEFAULT_HS_PEAK_PWM = 180", PID)
        self.assertIn("MAP1_LATE_BOOST_START_TICKS = 8000", PID)
        self.assertIn("mapId == 1 ? MAP_DEFAULT_HS_PEAK_PWM : 240", PID)
        self.assertIn("MAP2_CHECKPOINT_ENABLED = false", PID)

    def test_app_uses_esp32_confirmation_not_nvs_alone(self):
        self.assertIn("self.physical_calib_ready = False", APP)
        self.assertIn('self.send_command("SIDE:STATUS")', APP)
        self.assertIn('self.physical_calib_ready = phase in ("SELECT", "READY", "RUNNING")', APP)
        self.assertIn("and self.calibration_ok and self.physical_calib_ready", APP)
        self.assertIn("self.map1_side_confirmed", APP)
        self.assertIn('self.send_command("SIDE:" + side)', APP)
        self.assertIn('SIDE_START:TICK', APP)
        self.assertIn('SIDE_START:FULL', APP)
        self.assertIn('("Kd", "0.8"), ("Base", "140")', APP)
        self.assertIn('self.pwm_var = tk.DoubleVar(value=140)', APP)


if __name__ == "__main__":
    unittest.main()
