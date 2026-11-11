"""Headless STOP/auto-finish recovery regression tests for mainHighSpeed.

Run: python -m unittest discover -s tests -p 'test_stop_recovery.py'
"""
import queue
import struct
import unittest
from types import SimpleNamespace
from unittest.mock import patch
from robot_debug import App, Reader, FMT, HDR, xor_sum


class Var:
    def __init__(self, value=None):
        self.value = value
    def set(self, value):
        self.value = value
    def get(self):
        return self.value


class FakeStopApp:
    stop_line = App.stop_line
    handle_stop_timeout = App.handle_stop_timeout
    confirm_stopped = App.confirm_stopped

    def __init__(self):
        self.reader = SimpleNamespace(is_alive=lambda: True)
        self.run_csv = SimpleNamespace(active=object())
        self.stop_ack_deadline = None
        self.stop_verify_pending = False
        self.stop_verify_deadline = None
        self.stop_verify_retries = 0
        self.start_pending = False
        self.start_confirmed = True
        self.auto_running = True
        self.run_elapsed_seconds = 0.0
        self.run_started_at = 10.0
        self.run_finished_latched = False
        self.estop_latched = False
        self.estop_pending = False
        self.control_state = Var()
        self.control_state_label = SimpleNamespace(configure=lambda **kw: None)
        self.adc_status = Var()
        self.messages = []
        self.reasons = []
        self.logs = []
        self.button_updates = 0

    def send_command(self, command):
        self.messages.append(command)
        return True

    def finish_run_csv(self, reason):
        self.reasons.append(reason)
        self.run_csv.active = None
        self.stop_ack_deadline = None
        self.start_confirmed = False

    set_auto_running = App.set_auto_running

    def update_control_buttons(self):
        self.button_updates += 1

    def update_run_timer_display(self, now=None):
        pass  # Test double; display formatting is verified separately.

    def log(self, message):
        self.logs.append(message)


class StopRecoveryTests(unittest.TestCase):
    def test_missing_stop_ack_recovers_without_reopening_app(self):
        ui = FakeStopApp()
        ui.stop_line()
        self.assertEqual(ui.messages, ["STOP"])
        self.assertIsNotNone(ui.stop_ack_deadline)
        ui.handle_stop_timeout()
        self.assertTrue(ui.stop_verify_pending)
        self.assertEqual(ui.messages[-1], "STATUS")
        self.assertFalse(ui.start_confirmed)
        self.assertIn("CHƯA XÁC NHẬN", ui.reasons[-1])
        App.on_text(
            ui,
            "STATUS: bt=1 auto=0 estop=0 pwmL=0 pwmR=0 adc=IDF_GPIO_1 adc_ok=1",
        )
        self.assertFalse(ui.stop_verify_pending)
        self.assertIsNone(ui.stop_ack_deadline)
        self.assertFalse(ui.auto_running)
        self.assertIn("STATUS auto=0 pwmL=0 pwmR=0", ui.reasons[-1])

    def test_status_auto_or_pwm_nonzero_does_not_unlock(self):
        for msg in (
            "STATUS: bt=1 auto=1 estop=0 pwmL=0 pwmR=0",
            "STATUS: bt=1 auto=0 estop=0 pwmL=40 pwmR=0",
        ):
            with self.subTest(msg=msg):
                ui = FakeStopApp()
                ui.stop_verify_pending = True
                App.on_text(ui, msg)
                self.assertTrue(ui.stop_verify_pending)
                self.assertEqual(ui.reasons, [])

    def test_automatic_full_black_finish_unlocks(self):
        ui = FakeStopApp()
        App.on_text(
            ui, "AUTO_FINISHED: FULL_BLACK_3 map=2 runEnc=13350 t_ms=123456"
        )
        self.assertFalse(ui.auto_running)
        self.assertFalse(ui.stop_verify_pending)
        self.assertIsNone(ui.run_csv.active)
        self.assertIn("FULL_BLACK_3", ui.reasons[-1])

    def test_unexpected_pid_stop_is_reported(self):
        ui = FakeStopApp()
        App.on_text(ui, "AUTO_STOPPED: PID_STOPPED map=1 runEnc=5400 t_ms=22222")
        self.assertFalse(ui.auto_running)
        self.assertIn("runEnc=5400", ui.reasons[-1])

    def test_v2_clock_freezes_on_finish_independent_of_firmware_timestamp(self):
        ui = FakeStopApp()
        with patch("robot_debug.time.monotonic", return_value=18.765):
            App.on_text(
                ui, "AUTO_FINISHED: FULL_BLACK_3 map=1 runEnc=15500 t_ms=999999"
            )
        self.assertAlmostEqual(ui.run_elapsed_seconds, 8.765, places=3)
        self.assertFalse(ui.auto_running)
        self.assertTrue(ui.run_finished_latched)
        self.assertIsNone(ui.run_started_at)

    def test_v2_clock_ignores_late_auto_status_after_finish(self):
        ui = FakeStopApp()
        with patch("robot_debug.time.monotonic", return_value=18.765):
            App.on_text(ui, "AUTO_FINISHED: FULL_BLACK_3 map=1 t_ms=999999")
        frozen = ui.run_elapsed_seconds
        with patch("robot_debug.time.monotonic", return_value=25.0):
            App.on_text(ui, "STATUS: bt=1 auto=1 estop=0 pwmL=200 pwmR=200")
        self.assertFalse(ui.auto_running)
        self.assertEqual(ui.run_elapsed_seconds, frozen)

    def test_stopwatch_continues_through_line_loss(self):
        ui = FakeStopApp()
        with patch("robot_debug.time.monotonic", return_value=10.0):
            ui.auto_running = False
            ui.set_auto_running(True)
        # No line-sensor state is read by the timer; it must advance even if
        # every sensor is white and the SPP telemetry stream is silent.
        with patch("robot_debug.time.monotonic", return_value=13.25):
            ui.set_auto_running(False)
        self.assertAlmostEqual(ui.run_elapsed_seconds, 3.25)
        self.assertFalse(ui.auto_running)

    def test_reader_detects_stop_without_finish_text_reply(self):
        reader = Reader.__new__(Reader)
        reader.buf = bytearray()
        reader.text_buf = bytearray()
        reader.control_queue = queue.SimpleQueue()
        reader.out_queue = queue.Queue()
        reader.recorder = None
        reader.bad = reader.frames_rx = reader.frames_dropped = 0
        reader.saw_auto_frame = False
        reader.stopped_frame_count = 0

        def frame(t_ms, mode):
            values = (t_ms, *([0] * 16), 0, 0, 0, 0,
                      *([0.0] * 5), mode)
            payload = struct.pack(FMT, *values)
            return HDR + payload + bytes([xor_sum(payload)])

        reader.buf.extend(frame(1000, 1))
        reader._parse()
        for t in (1010, 1020, 1030):
            reader.buf.extend(frame(t, 0))
            reader._parse()
        self.assertEqual(reader.control_queue.get_nowait(),
                         ("stopped_sample", 1030))
        self.assertTrue(reader.control_queue.empty())
        # Idle frames after the race must not repeatedly close the log.
        reader.buf.extend(frame(1040, 0))
        reader._parse()
        self.assertTrue(reader.control_queue.empty())

    def test_zero_pwm_during_auto_does_not_mean_finished(self):
        reader = Reader.__new__(Reader)
        reader.buf = bytearray()
        reader.text_buf = bytearray()
        reader.control_queue = queue.SimpleQueue()
        reader.out_queue = queue.Queue()
        reader.recorder = None
        reader.bad = reader.frames_rx = reader.frames_dropped = 0
        reader.saw_auto_frame = False
        reader.stopped_frame_count = 0
        payload = struct.pack(FMT, 1000, *([0] * 16), 0, 0, 0, 0,
                              *([0.0] * 5), 1)
        for _ in range(4):
            reader.buf.extend(HDR + payload + bytes([xor_sum(payload)]))
            reader._parse()
        self.assertTrue(reader.control_queue.empty())

    def test_explicit_motor_stop_remains_supported(self):
        ui = FakeStopApp()
        App.on_text(ui, "MOTOR: STOP")
        self.assertFalse(ui.auto_running)
        self.assertIn("MOTOR: STOP", ui.reasons[-1])


if __name__ == "__main__":
    unittest.main()
