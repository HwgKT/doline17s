"""Web and SPP client consistency checks for mainHighSpeedv6.

Run: python -m unittest discover -s tests -p 'test_web_v6_protocol_source.py'
The webpage still needs a compatible SPP COM transport; BLE Nordic UART
cannot connect to the Bluetooth Classic-only firmware.
"""
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WEB = (ROOT / "index.html").read_text(encoding="utf-8")
APP = (ROOT / "robot_debug.py").read_text(encoding="utf-8")
MAIN = (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")


class WebV6ProtocolTests(unittest.TestCase):
    def test_web_uses_confirmation_state_from_esp32(self):
        for token in ("BUTTON_STATE phase=", "BUTTON_PROFILE side=",
                      "BUTTON_RUN map=1", "handleV6Response",
                      "webV6State.confirmed", "webV6State.calReady"):
            self.assertIn(token, WEB)
        self.assertIn("webV6State.confirmed = webV6State.calReady", WEB)
        self.assertIn("const startable = editable && webV6State.confirmed", WEB)
        self.assertIn("self.map1_side_confirmed", APP)
        self.assertIn("self.physical_calib_ready = phase in", APP)

    def test_calibration_and_two_start_modes(self):
        for command in ("CAL_WHITE", "CAL_BLACK",
                        "SIDE:LEFT", "SIDE:RIGHT",
                        "SIDE_START:TICK", "SIDE_START:FULL"):
            # SIDE commands are dynamically formed in JS but appear in firmware.
            self.assertIn(command, MAIN)
        self.assertIn("requestV6Calibration('WHITE')", WEB)
        self.assertIn("requestV6Calibration('BLACK')", WEB)
        self.assertIn("requestMap1Side('LEFT')", WEB)
        self.assertIn("requestMap1Side('RIGHT')", WEB)
        self.assertIn("fullEyes ? 'SIDE_START:FULL' : 'SIDE_START:TICK'", WEB)
        self.assertIn("btnMap1StartTick", WEB)
        self.assertIn("btnMap1StartFull", WEB)

    def test_web_default_pid_is_map1_default(self):
        self.assertRegex(WEB, r'id="inputKp"[^>]+value="1\.6"')
        self.assertRegex(WEB, r'id="inputKd"[^>]+value="0\.8"')
        self.assertRegex(WEB, r'id="inputBase"[^>]+value="140"')
        self.assertIn('id="speedSlider" min="0" max="255" value="140"', WEB)
        self.assertIn('("Kd", "0.8"), ("Base", "140")', APP)

    def test_legacy_bluetooth_features_are_not_claimed_as_supported(self):
        self.assertIn("Bluetooth Classic SPP", WEB)
        self.assertIn("Web Bluetooth BLE Nordic UART không tương thích", WEB)
        self.assertIn("STREAM:0", WEB)
        self.assertNotIn("sendCommand('BAM')", WEB)
        self.assertNotIn("sendCommand('STAGE2')", WEB)
        self.assertNotIn("BẮT ĐẦU LIA CALIB (4 Giây)", WEB)
        self.assertIn("const accepted = clean.search", WEB)

    def test_disconnect_fails_closed_and_stop_resyncs(self):
        self.assertIn("resetWebV6State();", WEB)
        self.assertIn("if(isConnected) sendCommand('SIDE:STATUS')", WEB)
        self.assertIn("if(webV6State.side === 'NONE') webV6State.eyeMode = 'TICK_MASK';", WEB)


if __name__ == "__main__":
    unittest.main()
