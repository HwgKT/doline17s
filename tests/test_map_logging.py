"""Run with: python -m unittest discover -s tests -p 'test_map_logging.py' -v

Headless tests: routing, CSV schema, markers, and sequential map sessions.
No Bluetooth radio, display or ESP32 required.
"""
import csv
import queue
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import spp_csv_recorder
from run_csv_lifecycle import RunCsvLifecycle


HEADER = (["host_iso", "t_ms"] + [f"adc{i}" for i in range(16)] +
          [f"cal{i}" for i in range(16)] + [f"logic{i}" for i in range(16)] +
          ["encL", "encR", "pwmL", "pwmR", "err", "p", "i", "d", "out",
           "mode", "logic16", "event_type", "event_id", "segment_id",
           "event_status"])
THRESHOLDS = [900] * 16
CALIB = ([400] * 16, [1650] * 16)
SAMPLE = (12345, *([1100] * 16), 101, 202, 110, 120, 0.0,
          0.0, 0.0, 0.0, 0.0, 1)


class FakeReader:
    def __init__(self):
        self.recorder = None


class MapLoggingTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path_patch = patch.object(
            spp_csv_recorder, "__file__",
            str(Path(self.temp.name) / "spp_csv_recorder.py"))
        self.path_patch.start()
        self.addCleanup(self.path_patch.stop)

    def test_invalid_map_is_rejected_without_creating_logs(self):
        for value in (None, 0, 4, "1", True):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    spp_csv_recorder.map_log_directory(value)
        self.assertFalse((Path(self.temp.name) / "logs").exists())

    def test_three_map_runs_remain_in_isolated_folders(self):
        lifecycle = RunCsvLifecycle(queue.SimpleQueue())
        reader = FakeReader()
        paths = []
        for map_id in (1, 2, 3):
            recorder = lifecycle.begin(
                reader, HEADER, CALIB, THRESHOLDS, map_id)
            self.assertEqual(reader.recorder, recorder)
            self.assertEqual(recorder.map_id, map_id)
            self.assertEqual(recorder.path.parent.name, f"map_{map_id}")
            self.assertIn(f"map{map_id}_", recorder.path.name)
            recorder.submit(SAMPLE)
            self.assertTrue(recorder.submit_marker(
                "START", 1, 12345, 101, 202, 1, "LOCAL"))
            self.assertIs(lifecycle.finish(reader, "STOP"), recorder)
            self.assertIsNone(reader.recorder)
            paths.append(recorder.path)
        lifecycle.close_all()
        self.assertEqual(len(set(paths)), 3)
        for index, path in enumerate(paths, 1):
            with self.subTest(map_id=index):
                with path.open(newline="", encoding="utf-8-sig") as stream:
                    rows = list(csv.reader(stream))
                self.assertEqual(rows[0], HEADER)
                self.assertEqual(rows[1][0], "CAL_THRESHOLDS")
                self.assertEqual(rows[2][1], "12345")
                self.assertEqual(rows[2][50:52], ["101", "202"])
                self.assertEqual(rows[3][-4:], ["START", "1", "1", "LOCAL"])
                self.assertEqual(len(rows[2]), len(HEADER))

    def test_same_map_gets_a_new_file_on_each_run(self):
        lifecycle = RunCsvLifecycle(queue.SimpleQueue())
        reader = FakeReader()
        paths = []
        for _ in range(2):
            recorder = lifecycle.begin(reader, HEADER, CALIB, THRESHOLDS, 2)
            paths.append(recorder.path)
            lifecycle.finish(reader, "STOP")
        lifecycle.close_all()
        self.assertNotEqual(paths[0], paths[1])
        self.assertTrue(all(path.parent.name == "map_2" for path in paths))


if __name__ == "__main__":
    unittest.main()
