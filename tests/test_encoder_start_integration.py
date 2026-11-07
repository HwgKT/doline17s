"""Static integration regressions for encoder reset and black-pad startup."""
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
GUI = (ROOT / "robot_debug.py").read_text(encoding="utf-8")
PID = (ROOT / "src" / "pid_controller.cpp").read_text(encoding="utf-8")
MAIN = (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")

class EncoderStartIntegrationTests(unittest.TestCase):
    def test_encoder_button_above_scope(self):
        self.assertIn('self.reset_encoder_btn = ttk.Button(', GUI)
        self.assertIn('command=self.reset_encoder', GUI)
        self.assertLess(GUI.index("self.reset_encoder_btn = ttk.Button("),
                        GUI.index("self.scope = tk.Canvas("))
        self.assertNotIn('command=lambda: self.send_command("ENC_RESET")', GUI)

    def test_encoder_reset_guard_and_ack(self):
        self.assertIn('ERR: ENC_RESET_REQUIRES_STOPPED', MAIN)
        self.assertRegex(MAIN, r'if\(pid_is_auto_mode\(\)\|\|manualActive\|\|')
        self.assertIn('elif message == "ENC_RESET: OK":', GUI)
        self.assertIn('self.prev = None', GUI)
        self.assertIn('self.send_command("ENC")', GUI)

    def test_start_always_requires_all_black(self):
        self.assertIn('if(sensor_get_line_bits()!=0xFFFFU)', MAIN)
        self.assertIn('ERR: START_REQUIRES_ALL_BLACK_16', MAIN)

    def test_start_coasts_without_pid_until_center_lock(self):
        self.assertIn('START_PAD_EXIT_CONFIRM_MS = 30', PID)
        self.assertIn('return blackCount >= 11 ? LINE_CASE_CROSS_LINE', PID)
        self.assertIn(': LINE_CASE_STARTUP_COAST;', PID)
        self.assertLess(PID.index('if (!finishDetectionArmed) {'),
                        PID.index('if (blackCount > 0) {', PID.index('if (!finishDetectionArmed) {')))
        self.assertIn('lineLocked = true;', PID)
        self.assertIn('STARTUP_LOSS_GRACE_MS', PID)

if __name__ == "__main__":
    unittest.main()
