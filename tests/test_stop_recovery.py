"""Headless STOP/auto-finish recovery regression tests for mainHighSpeed.

Run: python -m unittest discover -s tests -p 'test_stop_recovery.py'
"""
import unittest
from types import SimpleNamespace
from robot_debug import App


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
    _live_run_elapsed = App._live_run_elapsed

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
        self.run_origin_esp_ms = 10000
        self.run_finish_esp_ms = None
        self.last_esp_elapsed_seconds = None
        self.last_esp_sample_received_at = None
        self.run_live_elapsed_floor = 0.0
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

    def set_auto_running(self, running):
        self.auto_running = bool(running)

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

    def test_firmware_finish_time_overrides_host_lag(self):
        ui = FakeStopApp()
        App.on_text(
            ui, "AUTO_FINISHED: FULL_BLACK_3 map=1 runEnc=15500 t_ms=18765"
        )
        self.assertAlmostEqual(ui.run_elapsed_seconds, 8.765, places=3)
        self.assertFalse(ui.auto_running)
        self.assertTrue(ui.run_finished_latched)

    def test_telemetry_gap_does_not_freeze_after_250ms(self):
        ui = FakeStopApp()
        ui.last_esp_elapsed_seconds = 3.0
        ui.last_esp_sample_received_at = 20.0
        self.assertAlmostEqual(App._live_run_elapsed(ui, 21.0), 4.0, places=3)
        # Older telemetry must not move the display backward.
        ui.last_esp_elapsed_seconds = 2.9
        ui.last_esp_sample_received_at = 21.0
        self.assertAlmostEqual(App._live_run_elapsed(ui, 21.0), 4.0, places=3)

    def test_explicit_motor_stop_remains_supported(self):
        ui = FakeStopApp()
        App.on_text(ui, "MOTOR: STOP")
        self.assertFalse(ui.auto_running)
        self.assertIn("MOTOR: STOP", ui.reasons[-1])


if __name__ == "__main__":
    unittest.main()
