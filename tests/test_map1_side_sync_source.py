"""Static checks for Map 1 app and physical button side synchronization."""
import unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
MAIN=(ROOT/"src/main.cpp").read_text(encoding="utf-8")
APP=(ROOT/"robot_debug.py").read_text(encoding="utf-8")
PID=(ROOT/"src/pid_controller.cpp").read_text(encoding="utf-8")
class SideSyncTests(unittest.TestCase):
    def test_commands_and_acks(self):
        for s in ("SIDE:STATUS","SIDE:LEFT","SIDE:RIGHT","SIDE:CLEAR",
                  "SIDE_START:TICK","SIDE_START:FULL","BUTTON_STATE phase=",
                  "BUTTON_SIDE_CONFIRMED:", "BUTTON_SIDE_CLEARED"):
            self.assertIn(s, MAIN)
    def test_app_updates_only_on_ack(self):
        for s in ('self.send_command("SIDE:" + side)', 'self.send_command("SIDE:CLEAR")',
                  'self.send_command("SIDE:STATUS")','message.startswith("BUTTON_SIDE_CONFIRMED: ")',
                  'message.startswith("BUTTON_SIDE: ")','message=="BUTTON_SIDE_CLEARED"',
                  'self.selected_map != 1 or self.map1_side_confirmed',
                  'SIDE_START:FULL','SIDE_START:TICK'):
            self.assertIn(s,APP)
    def test_same_start_and_mask_source(self):
        self.assertIn('startPhysicalMap1(s=="SIDE_START:FULL")',MAIN)
        self.assertIn('else if(cal.shortPress) startPhysicalMap1(false)',MAIN)
        self.assertIn('else if(side.shortPress) startPhysicalMap1(true)',MAIN)
        self.assertIn('command("AUTO:1")',MAIN)
        self.assertIn('if(physicalRunInProgress)',MAIN)
        self.assertIn('0xFFC0U',PID)
        self.assertIn('MAP1_BUTTON_TICK2 = 1500',PID)
    def test_other_maps_unchanged(self):
        self.assertIn('else f"AUTO:{self.selected_map}"',APP)
if __name__=="__main__": unittest.main()
