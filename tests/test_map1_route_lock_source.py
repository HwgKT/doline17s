"""Static checks for Map 01 route-lock integration."""
import unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
PID = (ROOT/"src"/"pid_controller.cpp").read_text(encoding="utf-8")
MAIN = (ROOT/"src"/"main.cpp").read_text(encoding="utf-8")
APP = (ROOT/"robot_debug.py").read_text(encoding="utf-8")
ROUTE = (ROOT/"include"/"map1_route_lock.h").read_text(encoding="utf-8")

class Map1RouteSourceTests(unittest.TestCase):
    def test_first2k_windows_and_sequence(self):
        for name in ("STRAIGHT_1", "SEMICIRCLE", "RIGHT_TURN", "STRAIGHT_2"):
            self.assertIn(name, ROUTE)
        for marker in ("ticks >= 250", "ticks >= 900", "ticks >= 1220",
                       "ticks >= 1540", "ticks > 1680", "ticks > 1920"):
            self.assertIn(marker, ROUTE)
        self.assertIn("sawLeftApex", ROUTE)
        self.assertIn("chooseCluster(", ROUTE)

    def test_search_is_not_encoder_triggered_without_white_and_search(self):
        self.assertIn("map1Route.firstRightSearchWindow(", PID)
        self.assertIn("sensor_get_line_bits() == 0U && isSearchTurn()", PID)
        self.assertIn("turnState = SEARCH_RIGHT;", PID)
        self.assertIn("map1Route.fail(map1route::FIRST_SEARCH_TIMEOUT)", PID)

    def test_lock_defaults_and_map_scope(self):
        self.assertIn("bool map1RouteLockEnabled = true;", PID)
        self.assertIn("autoMode && selectedMap == 1 && map1RouteLockEnabled", PID)
        self.assertIn("if (autoMode) return false;", PID)
        self.assertIn("if (map1Route.active)", PID)
        self.assertIn("if (map1BoostActive) target = (float)highSpeedPeakPwm;", PID)

    def test_firmware_ui_contract(self):
        for msg in ("ROUTE_LOCK: ENABLED=", "ROUTE_STAGE", "ROUTE_FAULT",
                    "AUTO_STOPPED: ROUTE_FAULT"):
            self.assertIn(msg, MAIN)
        for msg in ("ROUTE_LOCK: ENABLED=", "ROUTE_STAGE ",
                    "ROUTE_FAULT ", "AUTO_STOPPED: ROUTE_FAULT"):
            self.assertIn(msg, APP)
        self.assertIn("self.route_status = tk.StringVar", APP)
        self.assertIn('self.send_command("ROUTE_LOCK")', APP)

if __name__ == "__main__":
    unittest.main()
