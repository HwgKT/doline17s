#!/usr/bin/env python3
"""ESP32 Line Robot Windows dashboard over Bluetooth Classic SPP.

Install: pip install pyserial
Run:     python robot_debug.py

First pair the ESP32 in Windows and select its SPP COM port.
mainv4 firmware now starts Classic SPP as LineRobot and sends 72-byte frames.
Use its outgoing Windows Bluetooth COM port, separate from the USB serial COM port.

The 69-byte SPP payload includes raw ADC. The GUI requests exact 16-channel
white/black references from the ESP32 NVS profile through CAL_DUMP, then
normalizes telemetry to 0..1000 using the same reference levels.
"""
import collections
import csv
import datetime
import math
import operator
import functools
import queue
import re
import shutil
import tempfile
import struct
import threading
import time
import tkinter as tk
from tkinter import filedialog, ttk
from pathlib import Path

import serial
from serial.tools import list_ports
from run_csv_lifecycle import RunCsvLifecycle

HDR = b"\xAA\x55"
FMT = "<I16H2i2h5fB"
PAYLOAD_LEN = struct.calcsize(FMT)  # 69
FRAME_LEN = len(HDR) + PAYLOAD_LEN + 1  # 72
ADC_MAX, PWM_MAX, HIST_LEN = 4095, 255, 300
MIN_CALIB_DELTA = 150
BLACK_THRESHOLD_CAL = 400  # 40% of each eye's calibrated W->B contrast (not RAW)
def eye_raw_threshold(white, black):
    """Exact RAW cutoff for firmware's normalized CAL>=400 rule."""
    return white + ((black - white) * BLACK_THRESHOLD_CAL + 999) // 1000
CSV_HEADER = (["host_iso", "t_ms"] +
              [f"adc{i}" for i in range(16)] +
              [f"cal{i}" for i in range(16)] +
              [f"logic{i}" for i in range(16)] +
              ["encL", "encR", "pwmL", "pwmR", "err", "p", "i", "d", "out", "mode", "logic16"])
BG, PANEL, FG, MUTED = "#0f1420", "#161d31", "#e2e8f0", "#94a3b8"
GREEN, GREY, AMBER, CYAN, ROSE = "#10b981", "#475569", "#f59e0b", "#22d3ee", "#f43f5e"


def xor_sum(data):
    return functools.reduce(operator.xor, data, 0)


def normalize_adc(raw, low, high, black_is_high=True):
    """Calibrate raw ADC to 0..1000; calibrated 1000 always means black."""
    if high - low < MIN_CALIB_DELTA:
        return 0
    raw = max(low, min(high, raw))
    if black_is_high:
        return (raw - low) * 1000 // (high - low)
    return (high - raw) * 1000 // (high - low)


class Reader(threading.Thread):
    """Read a mixed SPP stream of framed binary samples and text replies."""

    def __init__(self, port, out_queue, recorder=None, control_queue=None):
        super().__init__(daemon=True)
        # Never let a stalled Windows SPP COM write freeze the Tk UI.
        self.ser = serial.Serial(port, 115200, timeout=0.05, write_timeout=0.5)
        self.out_queue = out_queue
        self.recorder = recorder
        self.control_queue = control_queue if control_queue is not None else out_queue
        self.running = True
        self.buf = bytearray()
        self.text_buf = bytearray()
        self.bad = 0
        self.frames_rx = 0
        self.frames_dropped = 0

    def run(self):
        while self.running:
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
            except (serial.SerialException, OSError):
                self.control_queue.put(("error", "Bluetooth/COM đã ngắt kết nối"))
                break
            if chunk:
                self.buf.extend(chunk)
                self._parse()
        try:
            self.ser.close()
        except Exception:
            pass

    def _text_bytes(self, chunk):
        for b in chunk:
            if b in (10, 13):
                if self.text_buf:
                    line = self.text_buf.decode("utf-8", "replace").strip()
                    if line:
                        self.control_queue.put(("text", line))
                    self.text_buf.clear()
            elif 32 <= b <= 126 or b >= 128:
                if len(self.text_buf) < 512:
                    self.text_buf.append(b)
                else:
                    self.text_buf.clear()

    def _parse(self):
        while self.buf:
            offset = self.buf.find(HDR)
            if offset < 0:
                # A leading 0xAA may be split across read() calls.
                keep = 1 if self.buf[-1] == 0xAA else 0
                self._text_bytes(self.buf[:-1] if keep else self.buf)
                self.buf = bytearray(b"\xAA") if keep else bytearray()
                return
            if offset:
                self._text_bytes(self.buf[:offset])
                del self.buf[:offset]
            if len(self.buf) < FRAME_LEN:
                return
            data = bytes(self.buf[2:2 + PAYLOAD_LEN])
            if xor_sum(data) == self.buf[2 + PAYLOAD_LEN]:
                self.frames_rx += 1
                sample = struct.unpack(FMT, data)
                # Logging precedes GUI delivery; Tk slowdown cannot throttle disk writes.
                if self.recorder is not None:
                    self.recorder.submit(sample)
                try:
                    self.out_queue.put_nowait(("data", sample))
                except queue.Full:
                    self.frames_dropped += 1
                del self.buf[:FRAME_LEN]
            else:
                self.bad += 1
                del self.buf[:1]

    def send(self, text):
        data = (text + "\n").encode("utf-8")
        if self.ser.write(data) != len(data):
            raise serial.SerialTimeoutException("SPP command was only partially written")


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ESP32 Line Robot — Bluetooth Classic SPP")
        self.geometry("1440x900")
        self.minsize(1080, 720)
        self.configure(bg=BG)
        self.q = queue.Queue(maxsize=4096)  # bounded GUI data queue
        self.controls_q = queue.SimpleQueue()  # priority text replies / E-STOP ACK
        self.reader = None
        self.run_csv = RunCsvLifecycle(self.controls_q)
        self.stop_ack_deadline = None
        self.stop_verify_pending = False
        self.stop_verify_deadline = None
        self.stop_verify_retries = 0
        self.start_confirmed = False
        self.csv_status = tk.StringVar(value="CSV: chờ START")
        self.selected_map = None  # Require explicit choice to avoid mislabeled experiments.
        self.map_status = tk.StringVar(value="Chưa chọn")
        self.map_buttons = {}
        self.run_txt_file = None
        self.run_txt_path = None
        self.session_log_file = tempfile.TemporaryFile(mode="w+t", encoding="utf-8")
        self.hist = collections.deque(maxlen=HIST_LEN)
        self.recent_rows = collections.deque(maxlen=15000)  # UI cache only, ~150 sec at 100 Hz
        self.log_lines = collections.deque(maxlen=30000)  # display cache; TXT export uses disk
        self.log_window = None
        self.log_mirror = None
        self.sensor_window = None
        self.sensor_table = None
        self.preview_pid_log = tk.BooleanVar(value=True)
        self.last_preview_ms = None
        self.stream_rate = tk.StringVar(value="100")
        self.prev = None
        self.wheel_speed = [0.0, 0.0]
        self.encoder_reset_pending = False
        self.encoder_reset_pending_at = None
        self.frames = 0
        self.last_rx_total = 0
        self.last_csv_total = 0
        self.rate_time = time.monotonic()
        # Report actual firmware loop/PID rates using periodic SPP DIAG.
        self.diag_interval_s = 2.0
        self.next_diag_poll = None
        self.last_diag_rx = None
        self.loop_debug = tk.StringVar(
            value="LOOP: -- Hz    PID: -- Hz    SPP TX: -- Hz    (chưa có DIAG)")
        self.high_speed_peak = 220  # Expected startup default; read back from firmware.
        self.hs_brake_start = 1800  # Readback from legacy first-straight mode.
        self.hs_late_start = 10500
        self.hs_late_mode = False
        self.hs_peak_confirmed = None
        self.hs_peak_pending = None
        self.hs_peak_pending_at = None
        self.hs_peak_dirty = False
        self.hs_peak_var = tk.DoubleVar(value=220)
        self.hs_peak_value_text = tk.StringVar(value="220 (chưa xác nhận)")
        self.hs_peak_feedback = tk.StringVar(value="Đọc HS từ ESP32 trước khi START")
        self.high_speed_status = tk.StringVar(
            value="HS: MAP 1 từ 10500 tick | chờ ESP32 xác nhận")
        self.route_status = tk.StringVar(value="ROUTE: chờ ESP32 xác nhận")
        # Three-stage sensor selection, stored in ESP32 RAM until power-off.
        self.sensor_profile = (350, 1500, 0x01FF, 0xFF80, 0xFFFF)
        self.sensor_profile_confirmed = False
        self.sensor_profile_status = tk.StringVar(
            value="PID mắt: chờ ESP32 (mặc định 350 / 1500 ticks)")
        self.sensor_profile_window = None
        self.map2_sensor_profile = ([1000, 2000, 3000], [0xFFFF] * 4)
        self.map2_sensor_confirmed = False
        self.map2_sensor_pending = None
        self.map2_sensor_editor_refresh = None
        self.map2_sensor_status = tk.StringVar(value="MAP 2 mắt PID: chờ ESP32")
        self.map2_sensor_window = None
        # Map 3 preset: right-side eyes, all eyes, left-side eyes, then all eyes.
        self.map3_sensor_profile = (
            730, 1960, 2350, 0xFF00, 0xFFFF, 0x01FF, 0xFFFF)
        self.map3_sensor_confirmed = False
        self.map3_sensor_status = tk.StringVar(value="MAP 3 mắt PID: chờ ESP32")
        self.map3_sensor_window = None
        # Independent eight-stage profiles. MAP 2/3 preserve their old constant
        # BASE until the user deliberately changes their per-stage values.
        default_ticks = [3800, 7800, 8350, 10300, 11400, 12800, 15000]
        self.map_speed_profiles = {
            1: {"ticks": list(default_ticks),
                "bases": [140, 140, 140, 240, 140, 140, 210, 255]},
            2: {"ticks": list(default_ticks), "bases": [160] * 8},
            3: {"ticks": list(default_ticks), "bases": [150] * 8},
        }
        self.map_profile_confirmed = {1: False, 2: False, 3: False}
        self.map_profile_pending = None
        self.map_profile_window = None
        self.map_profile_map_id = None
        self.map_profile_editor_refresh = None
        self.map_profile_tick_vars = []
        self.map_profile_base_vars = []
        self.map_profile_stage_labels = []
        self.map_profile_dirty = False
        self.map_profile_status = tk.StringVar(
            value="MAP BASE: chọn MAP và đọc cấu hình từ ESP32")
        self.values = {}
        self.last_adc = [0] * 16
        # Display only firmware calibration. Keep last valid NVS profile on failures.
        self.calibration_ok = False
        self.black_high = True
        self.cal_min = [0] * 16
        self.cal_max = [4095] * 16
        self.cal_thresholds = [None] * 16
        self.cal_data = {}
        self.estop_latched = None  # Unknown until firmware STATUS.
        self.estop_pending = False
        self.start_pending = False
        self.auto_running = False
        self.run_elapsed_seconds = 0.0
        self.run_started_at = None
        self.run_timer_var = tk.StringVar(value="00:00:00")
        self._build()
        self.update_control_buttons()  # Includes HS widgets created after START/STOP.
        self.refresh_ports()
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.bind_all("<Escape>", lambda _event: self.stop_line())
        self.after(30, self.tick)

    def label(self, parent, text="", fg=FG, bg=None, **kw):
        return tk.Label(parent, text=text, fg=fg, bg=BG if bg is None else bg, **kw)

    def _build(self):
        top = tk.Frame(self, bg=BG)
        top.pack(fill="x", padx=8, pady=(4, 2))
        self.label(top, "Bluetooth SPP", font=("Segoe UI", 10, "bold")).pack(side="left", padx=(0, 6))
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=11)
        self.port_combo.pack(side="left", padx=3)
        ttk.Button(top, text="Quét COM", command=self.refresh_ports).pack(side="left", padx=3)
        self.connect_btn = ttk.Button(top, text="Kết nối", command=self.toggle_connect)
        self.connect_btn.pack(side="left", padx=3)
        self.label(top, "CSV: tự động START/STOP", fg=GREEN,
                   font=("Segoe UI", 9)).pack(side="left", padx=4)
        self.label(top, "SPP Hz", fg=MUTED).pack(side="left", padx=(4, 2))
        self.rate_combo = ttk.Combobox(top, textvariable=self.stream_rate,
                                       values=("300", "200", "100", "50", "20", "0"),
                                       width=5, state="readonly")
        self.rate_combo.pack(side="left")
        self.rate_combo.bind("<<ComboboxSelected>>",
                             lambda _e: self.send_command("STREAM:" + self.stream_rate.get()))
        self.status_label = self.label(top, "Chưa kết nối", fg=ROSE)
        self.status_label.pack(side="left", padx=5)
        ttk.Button(top, text="Đo TX/Loop", command=self.request_diagnostics).pack(side="left", padx=3)
        self.rate_label = self.label(top, "RX 0 Hz | APP 0 Hz | CRC 0", fg=MUTED)
        self.rate_label.pack(side="right")

        diag_bar = tk.Frame(self, bg=PANEL)
        diag_bar.pack(fill="x", padx=8, pady=(0, 2))
        map_bar = tk.Frame(diag_bar, bg=PANEL)
        map_bar.pack(side="right", padx=6, pady=1)
        self.label(map_bar, "Chọn MAP:", fg=MUTED, bg=PANEL,
                   font=("Segoe UI", 9, "bold")).pack(side="left", padx=(0, 3))
        for map_id in (1, 2, 3):
            button = tk.Button(map_bar, text=f"MAP {map_id}", width=6,
                               command=lambda selected=map_id: self.select_map(selected),
                               bg=GREY, fg=FG, relief="flat", height=1,
                               font=("Segoe UI", 9, "bold"))
            button.pack(side="left", padx=2)
            self.map_buttons[map_id] = button
        self.label(map_bar, textvariable=self.map_status, fg=AMBER, bg=PANEL,
                   font=("Consolas", 8)).pack(side="left", padx=(5, 0))
        self.label(diag_bar, textvariable=self.loop_debug, fg=CYAN, bg=PANEL,
                   font=("Consolas", 9, "bold")).pack(anchor="w", padx=8, pady=(1, 0))
        self.label(diag_bar, textvariable=self.route_status, fg=CYAN, bg=PANEL,
                   font=("Consolas", 8, "bold")).pack(anchor="w", padx=8, pady=(0, 1))
        self.label(diag_bar, textvariable=self.csv_status, fg=GREEN, bg=PANEL,
                   font=("Consolas", 8)).pack(anchor="w", padx=8, pady=(0, 1))

        sensor_profile_bar = tk.Frame(self, bg=PANEL, padx=8, pady=3)
        sensor_profile_bar.pack(fill="x", padx=8, pady=(0, 2))
        self.sensor_profile_btn = ttk.Button(
            sensor_profile_bar, text="Cấu hình mắt PID / ticks",
            command=self.open_sensor_profile_editor)
        self.sensor_profile_btn.pack(side="left", padx=(0, 6))
        self.label(sensor_profile_bar, textvariable=self.sensor_profile_status,
                   bg=PANEL, fg=CYAN, font=("Consolas", 9)).pack(
                       side="left", fill="x", expand=True)
        self.sensor_profile_read_btn = ttk.Button(
            sensor_profile_bar, text="Đọc từ ESP32",
            command=lambda: self.send_command("SENSOR_PROFILE"))
        self.sensor_profile_read_btn.pack(side="right")

        map2_sensor_bar = tk.Frame(self, bg=PANEL, padx=8, pady=3)
        map2_sensor_bar.pack(fill="x", padx=8, pady=(0, 2))
        self.map2_sensor_btn = ttk.Button(
            map2_sensor_bar, text="MAP 2 — 1–8 giai đoạn mắt PID / tick",
            command=self.open_map2_sensor_editor)
        self.map2_sensor_btn.pack(side="left", padx=(0, 6))
        self.label(map2_sensor_bar, textvariable=self.map2_sensor_status,
                   bg=PANEL, fg=CYAN, font=("Consolas", 9)).pack(
                       side="left", fill="x", expand=True)
        self.map2_sensor_read_btn = ttk.Button(
            map2_sensor_bar, text="Đọc từ ESP32",
            command=lambda: self.send_command("MAP_SENSOR_PROFILE_V2:2"))
        self.map2_sensor_read_btn.pack(side="right")

        map3_sensor_bar = tk.Frame(self, bg=PANEL, padx=8, pady=3)
        map3_sensor_bar.pack(fill="x", padx=8, pady=(0, 2))
        self.map3_sensor_btn = ttk.Button(
            map3_sensor_bar, text="MAP 3 — 4 giai đoạn mắt PID / tick",
            command=self.open_map3_sensor_editor)
        self.map3_sensor_btn.pack(side="left", padx=(0, 6))
        self.label(map3_sensor_bar, textvariable=self.map3_sensor_status,
                   bg=PANEL, fg=CYAN, font=("Consolas", 9)).pack(
                       side="left", fill="x", expand=True)
        self.map3_sensor_read_btn = ttk.Button(
            map3_sensor_bar, text="Đọc từ ESP32",
            command=lambda: self.send_command("MAP3_SENSOR_PROFILE"))
        self.map3_sensor_read_btn.pack(side="right")

        # Calibration is a prerequisite for START. Keep its controls visible at
        # minimum window size rather than below the control panel.
        cal_bar = tk.Frame(self, bg=PANEL, padx=8, pady=3)
        cal_bar.pack(fill="x", padx=8, pady=(0, 2))
        self.label(cal_bar, "Calibration — 16 mắt / NVS", bg=PANEL,
                   font=("Segoe UI", 9, "bold")).pack(side="left", padx=(0, 8))
        self.cal_white_btn = ttk.Button(
            cal_bar, text="1. Lấy TRẮNG", command=self.start_calibration)
        self.cal_white_btn.pack(side="left", padx=(0, 4))
        self.cal_black_btn = ttk.Button(
            cal_bar, text="2. Lấy ĐEN", command=self.stop_calibration)
        self.cal_black_btn.pack(side="left", padx=(0, 8))
        ttk.Button(cal_bar, text="CAL Status",
                   command=lambda: self.send_command("CAL_STATUS")).pack(
                       side="left", padx=(0, 4))
        ttk.Button(cal_bar, text="Đọc NVS 16 mắt",
                   command=lambda: self.send_command("CAL_DUMP")).pack(
                       side="left", padx=(0, 8))
        self.cal_state = tk.StringVar(
            value="Chưa tải profile calibration từ ESP32")
        self.label(cal_bar, textvariable=self.cal_state, bg=PANEL, fg=AMBER,
                   font=("Segoe UI", 8), justify="left", anchor="w",
                   wraplength=290).pack(side="left", fill="x", expand=True)

        # Balanced 50/50 columns; bottom-right quarter reserved for SPP console.
        body = tk.Frame(self, bg=BG)
        body.pack(fill="both", expand=True, padx=8, pady=(0, 5))
        body.grid_columnconfigure(0, weight=1, uniform="dashboard", minsize=490)
        body.grid_columnconfigure(1, weight=1, uniform="dashboard", minsize=490)
        body.grid_rowconfigure(0, weight=1)
        left = tk.Frame(body, bg=BG)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 5))
        right = tk.Frame(body, bg=PANEL)
        right.grid(row=0, column=1, sticky="nsew", padx=(5, 0))
        right.grid_columnconfigure(0, weight=1)
        # Give compact controls enough space, without clipping at 720p/HiDPI.
        right.grid_rowconfigure(0, weight=56, minsize=275)
        right.grid_rowconfigure(1, weight=44, minsize=170)

        # Upper-right quarter: compact fixed controls, no scrolling.
        # Lower-right quarter remains a full-height SPP terminal.
        panel = tk.Frame(right, bg=PANEL, padx=6, pady=3)
        panel.grid(row=0, column=0, sticky="nsew")

        terminal_panel = tk.Frame(right, bg=PANEL, padx=8, pady=5)
        terminal_panel.grid(row=1, column=0, sticky="nsew")

        control = tk.Frame(panel, bg=PANEL)
        control.pack(fill="x", pady=(0, 2))
        self.start_btn = tk.Button(control, text="BẮT ĐẦU DÒ LINE",
                                   command=self.start_line, font=("Segoe UI", 9, "bold"),
                                   bg=GREEN, fg="#ffffff", activebackground="#059669",
                                   relief="flat", height=1, pady=0)
        self.start_btn.pack(side="left", fill="x", expand=True, padx=(0, 5))
        self.stop_btn = tk.Button(control, text="DỪNG",
                                  command=self.stop_line, font=("Segoe UI", 9, "bold"),
                                  bg=ROSE, fg="#ffffff", activebackground="#be123c",
                                  relief="flat", height=1, pady=1)
        self.stop_btn.pack(side="left", fill="x", expand=True)
        run_timer = tk.Frame(panel, bg=PANEL)
        run_timer.pack(fill="x", pady=(1, 2))
        self.label(run_timer, "Thời gian xe chạy", bg=PANEL, fg=MUTED,
                   font=("Segoe UI", 9)).pack(side="left")
        self.label(run_timer, textvariable=self.run_timer_var, bg=PANEL, fg=CYAN,
                   font=("Consolas", 10, "bold")).pack(side="left", padx=5)
        ttk.Button(run_timer, text="Reset giờ", width=9,
                   command=self.reset_run_timer).pack(side="right")
        self.control_state = tk.StringVar(value="Chưa kết nối / chưa xác định trạng thái")
        self.control_state_label = self.label(panel, textvariable=self.control_state,
                                              bg=PANEL, fg=AMBER, wraplength=430,
                                              font=("Segoe UI", 9),
                                              anchor="w", justify="left")
        self.control_state_label.pack(fill="x", pady=0)
        self.adc_status = tk.StringVar(value="ADC: chờ STATUS")
        self.label(panel, textvariable=self.adc_status, bg=PANEL, fg=CYAN,
                   font=("Consolas", 8)).pack(anchor="w", pady=(0, 1))
        self.update_control_buttons()

        sensor_head = tk.Frame(left, bg=BG)
        sensor_head.pack(fill="x")
        self.label(sensor_head, "16 mắt: RAW / T_i / CAL / 0-1", font=("Segoe UI", 11, "bold")).pack(side="left")
        ttk.Button(sensor_head, text="Bảng 16 mắt", command=self.open_sensor_table).pack(side="right", padx=4)
        self.label(sensor_head, "Mỗi mắt có ngưỡng riêng", fg=AMBER).pack(side="right", padx=6)
        self.bars = tk.Canvas(left, bg=PANEL, width=570, height=282, highlightthickness=0)
        self.bars.pack(fill="x", pady=5)
        self.logic_var = tk.StringVar(value="0 = TRẮNG | 1 = ĐEN | Chưa có NVS: ----------------")
        self.label(left, textvariable=self.logic_var, fg=CYAN,
                   font=("Consolas", 10, "bold")).pack(anchor="w", pady=(0, 8))

        # Encoder reset must be visible WITHOUT scrolling below the Scope.
        encoder_header = tk.Frame(left, bg=BG)
        encoder_header.pack(fill="x", pady=(3, 0))
        self.label(encoder_header, "Encoder / tốc độ / PID", fg=MUTED).pack(
            side="left", padx=(2, 0))
        self.reset_encoder_btn = ttk.Button(
            encoder_header, text="Reset ENC", width=11,
            command=self.reset_encoder)
        self.reset_encoder_btn.pack(side="right", padx=(0, 4))
        metrics = tk.Frame(left, bg=PANEL)
        metrics.pack(fill="x", pady=4)
        names = [("encL", "Encoder L"), ("encR", "Encoder R"),
                 ("spdL", "Tốc độ L (tick/s)"), ("spdR", "Tốc độ R (tick/s)"),
                 ("pwmL", "PWM L"), ("pwmR", "PWM R"),
                 ("err", "Error"), ("out", "PID out"),
                 ("p", "P"), ("i", "I"), ("d", "D"), ("mode", "Chế độ")]
        for n, (key, name) in enumerate(names):
            r, c = divmod(n, 3)
            self.label(metrics, name, fg=MUTED, bg=PANEL, width=12,
                       anchor="e").grid(row=r, column=c * 2, padx=(2, 1), pady=4)
            self.values[key] = tk.StringVar(value="—")
            self.label(metrics, textvariable=self.values[key], bg=PANEL, fg=CYAN,
                       font=("Consolas", 9, "bold"), width=8,
                       anchor="w").grid(row=r, column=c * 2 + 1, padx=(1, 2))

        self.label(left, "Scope: PWM trái / PWM phải / PID out", fg=MUTED).pack(anchor="w", pady=(11, 3))
        self.scope = tk.Canvas(left, bg=PANEL, width=570, height=165, highlightthickness=0)
        self.scope.pack(fill="x")
        self.label(left, "RAW: 0–4095  |  T_i=W_i+40%×(B_i−W_i)  |  CAL: 0–1000  |  0=TRẮNG, 1=ĐEN",
                   fg=MUTED).pack(anchor="w")

        # Compact one-line PWM and PID controls keep HS buttons visible.
        pwm_row = tk.Frame(panel, bg=PANEL)
        pwm_row.pack(fill="x", pady=(1, 0))
        self.label(pwm_row, "PWM", bg=PANEL,
                   font=("Segoe UI", 9, "bold")).pack(side="left", padx=(0, 5))
        self.pwm_var = tk.DoubleVar(value=120)
        self.pwm_text = tk.StringVar(value="120 / 255")
        self.label(pwm_row, textvariable=self.pwm_text, bg=PANEL, fg=CYAN,
                   font=("Consolas", 9), width=9, anchor="e").pack(side="right")
        self.pwm_slider = ttk.Scale(
            pwm_row, from_=0, to=255, variable=self.pwm_var,
            command=self.preview_pwm)
        self.pwm_slider.pack(side="left", fill="x", expand=True, padx=(4, 6))
        self.pwm_slider.bind("<ButtonRelease-1>", lambda _e: self.apply_pwm())
        self.pwm_slider.bind("<KeyRelease>", lambda _e: self.apply_pwm())
        speed_buttons = tk.Frame(panel, bg=PANEL)
        speed_buttons.pack(fill="x", pady=(0, 1))
        self.label(speed_buttons, "Mức PWM:", bg=PANEL, fg=MUTED,
                   font=("Segoe UI", 8)).pack(side="left", padx=(0, 4))
        for val in (60, 120, 180, 255):
            ttk.Button(speed_buttons, text=str(val), width=4,
                       command=lambda v=val: self.preset_pwm(v)).pack(
                           side="left", expand=True, padx=1)

        pid_row = tk.Frame(panel, bg=PANEL)
        pid_row.pack(fill="x", pady=(1, 2))
        self.label(pid_row, "PID", bg=PANEL,
                   font=("Segoe UI", 9, "bold")).pack(side="left", padx=(0, 4))
        self.pid_vars = {}
        ttk.Button(pid_row, text="Đọc", width=5,
                   command=lambda: self.send_command("PID")).pack(
                       side="right", padx=(2, 0))
        ttk.Button(pid_row, text="Áp dụng", width=7,
                   command=self.apply_pid).pack(side="right", padx=(2, 0))
        for key, initial in (("Kp", "1.6"), ("Ki", "0"),
                             ("Kd", "0.8"), ("Base", "120")):
            field = tk.Frame(pid_row, bg=PANEL)
            field.pack(side="left", padx=(0, 4))
            self.label(field, key, bg=PANEL, fg=MUTED,
                       font=("Segoe UI", 8)).pack(side="left", padx=(0, 1))
            var = tk.StringVar(value=initial)
            self.pid_vars[key] = var
            ttk.Entry(field, textvariable=var, width=5).pack(side="left")

        map_base_row = tk.Frame(panel, bg=PANEL)
        map_base_row.pack(fill="x", pady=(3, 2))
        ttk.Button(map_base_row, text="BASE từng đoạn MAP 1/2/3",
                   command=self.open_map_speed_editor).pack(side="left", padx=(0, 7))
        self.label(map_base_row, textvariable=self.map_profile_status,
                   bg=PANEL, fg=CYAN, font=("Segoe UI", 8),
                   anchor="w").pack(side="left", fill="x", expand=True)

        terminal_heading = tk.Frame(terminal_panel, bg=PANEL)
        terminal_heading.pack(fill="x", pady=(6, 3))
        self.label(terminal_heading, "Terminal TX / RX", bg=PANEL,
                   font=("Segoe UI", 11, "bold")).pack(side="left")
        ttk.Button(terminal_heading, text="Xóa", command=self.clear_terminal).pack(side="right", padx=2)
        ttk.Button(terminal_heading, text="Mở rộng", command=self.open_log_window).pack(side="right", padx=2)
        ttk.Button(terminal_heading, text="Xuất TXT", command=self.export_txt).pack(side="right", padx=2)
        ttk.Button(terminal_heading, text="Xuất CSV", command=self.export_csv).pack(side="right", padx=2)
        ttk.Checkbutton(terminal_panel, text="PID preview 5 Hz (CSV đầy đủ)",
                        variable=self.preview_pid_log).pack(anchor="w", pady=(1, 2))
        text_area = tk.Frame(terminal_panel, bg=PANEL)
        text_area.pack(fill="both", expand=True)
        self.console = tk.Text(text_area, bg=BG, fg=FG, insertbackground=FG,
                               font=("Consolas", 10), height=8, wrap="none",
                               state="disabled", relief="flat")
        scrollbar = ttk.Scrollbar(text_area, orient="vertical", command=self.console.yview)
        self.console.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        self.console.pack(side="left", fill="both", expand=True)
        hscroll = ttk.Scrollbar(terminal_panel, orient="horizontal", command=self.console.xview)
        self.console.configure(xscrollcommand=hscroll.set)
        hscroll.pack(fill="x")
        cmd_row = tk.Frame(terminal_panel, bg=PANEL)
        cmd_row.pack(fill="x", pady=(5, 0))
        self.cmd_var = tk.StringVar()
        command_entry = ttk.Entry(cmd_row, textvariable=self.cmd_var)
        command_entry.pack(side="left", fill="x", expand=True, padx=(0, 4))
        command_entry.bind("<Return>", lambda _e: self.send_terminal())
        ttk.Button(cmd_row, text="Gửi", command=self.send_terminal).pack(side="right")
        self.log("Chọn COM Bluetooth SPP (outgoing). Chưa có xác nhận firmware.")

    def log(self, message):
        line = f"[{datetime.datetime.now().astimezone().isoformat(timespec='milliseconds')}] {message}\n"
        self.log_lines.append(line)
        self.session_log_file.write(line)
        if self.run_txt_file is not None:
            try:
                self.run_txt_file.write(line)
            except OSError as exc:
                self.csv_status.set(f"TXT WRITE ERROR: {exc}")
                try:
                    self.run_txt_file.close()
                except OSError:
                    pass
                self.run_txt_file = None
        self.console.configure(state="normal")
        self.console.insert("end", line)
        if int(self.console.index("end-1c").split(".")[0]) > 1200:
            self.console.delete("1.0", "201.0")
        self.console.see("end")
        self.console.configure(state="disabled")
        if self.log_mirror is not None and self.log_mirror.winfo_exists():
            self.log_mirror.configure(state="normal")
            self.log_mirror.insert("end", line)
            self.log_mirror.see("end")
            self.log_mirror.configure(state="disabled")

    def clear_terminal(self):
        self.log_lines.clear()
        self.session_log_file.seek(0)
        self.session_log_file.truncate()
        for widget in (self.console, self.log_mirror):
            if widget is not None and widget.winfo_exists():
                widget.configure(state="normal")
                widget.delete("1.0", "end")
                widget.configure(state="disabled")

    def open_log_window(self):
        if self.log_window is not None and self.log_window.winfo_exists():
            self.log_window.lift()
            return
        win = tk.Toplevel(self)
        win.title("LineRobot — Terminal TX/RX và PID preview")
        win.geometry("1100x640")
        txt = tk.Text(win, bg=BG, fg=FG, font=("Consolas", 10), wrap="none")
        sb = ttk.Scrollbar(win, orient="vertical", command=txt.yview)
        txt.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        txt.pack(fill="both", expand=True)
        txt.insert("end", "".join(self.log_lines))
        txt.see("end")
        txt.configure(state="disabled")
        self.log_window, self.log_mirror = win, txt
        def on_close():
            self.log_mirror = None
            self.log_window = None
            win.destroy()
        win.protocol("WM_DELETE_WINDOW", on_close)

    def open_sensor_table(self):
        if self.sensor_window is not None and self.sensor_window.winfo_exists():
            self.sensor_window.lift()
            return
        win = tk.Toplevel(self)
        win.title("LineRobot — 16 ngưỡng riêng từ NVS")
        win.geometry("780x560")
        cols = ("eye", "raw", "white", "black", "threshold", "cal", "logic")
        table = ttk.Treeview(win, columns=cols, show="headings", height=16)
        for col, label in zip(cols, ("Mắt", "ADC RAW", "Trắng W", "Đen B", "Ngưỡng T_i",
                                      "CAL 0–1000", "0 trắng / 1 đen")):
            table.heading(col, text=label)
            table.column(col, width=95, anchor="center")
        table.pack(fill="both", expand=True)
        for i in range(16):
            table.insert("", "end", iid=str(i), values=(i, "—", "—", "—", "—", "—", "—"))
        self.sensor_window, self.sensor_table = win, table
        self.update_sensor_table()
        def on_close():
            self.sensor_window, self.sensor_table = None, None
            win.destroy()
        win.protocol("WM_DELETE_WINDOW", on_close)

    def update_sensor_table(self):
        if self.sensor_table is None or not self.sensor_table.winfo_exists():
            return
        for i, raw in enumerate(self.last_adc):
            if self.calibration_ok:
                white, black = self.cal_min[i], self.cal_max[i]
                cal = normalize_adc(raw, white, black)
                self.sensor_table.item(str(i), values=(i, raw, white, black,
                                     eye_raw_threshold(white, black), cal, int(cal >= BLACK_THRESHOLD_CAL)))
            else:
                self.sensor_table.item(str(i), values=(i, raw, "—", "—", "—", "—", "—"))

    def export_txt(self):
        recorder = self.run_csv.last
        if recorder is None:
            self.log("Chưa có lượt chạy nào để xuất TXT.")
            return
        source_path = recorder.path.with_suffix(".txt")
        if not source_path.is_file():
            self.log(f"Không tìm thấy TXT của lượt chạy: {source_path}")
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".txt", initialdir=str(source_path.parent),
            initialfile=source_path.name, filetypes=[("Text log", "*.txt")])
        if not path:
            return
        if Path(path).resolve() == source_path.resolve():
            self.log(f"TXT đã được lưu sẵn tại: {source_path}")
            return
        try:
            if self.run_txt_file is not None and self.run_txt_path == source_path:
                self.run_txt_file.flush()
            with source_path.open("rb") as source, open(path, "wb") as target:
                shutil.copyfileobj(source, target)
            self.log(f"Đã xuất TXT lượt chạy: {path}")
        except OSError as exc:
            self.log(f"Không thể xuất TXT: {exc}")

    def export_csv(self):
        selected = self.run_csv.last
        if selected is None:
            self.log("Chưa có lượt chạy CSV nào để xuất.")
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".csv", initialdir=str(selected.path.parent),
            initialfile=selected.path.name, filetypes=[("CSV", "*.csv")])
        if not path:
            return

        def export_job():
            try:
                count = selected.export_snapshot(path)
                message = f"Đã xuất CSV ({count} mẫu): {path}"
            except (OSError, ValueError) as exc:
                message = f"Lỗi xuất CSV: {exc}"
            self.controls_q.put(("csv_export", message))

        threading.Thread(target=export_job, daemon=True).start()

    def refresh_ports(self):
        ports = [p.device for p in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def toggle_connect(self):
        if self.reader:
            self.disconnect()
            return
        port = self.port_var.get().strip()
        if not port:
            self.log("Chưa chọn COM")
            return
        try:
            # Merely connecting Bluetooth never opens a CSV file.
            self.reader = Reader(port, self.q, None, self.controls_q)
        except (serial.SerialException, OSError, ValueError) as exc:
            self.status_label.config(text=f"Lỗi COM: {exc}", fg=ROSE)
            self.log(str(exc))
            return
        self.reader.start()
        self.encoder_reset_pending = False
        self.encoder_reset_pending_at = None
        self.hs_brake_start = 1800
        self.hs_peak_confirmed = None
        self.hs_peak_pending = None
        self.hs_peak_pending_at = None
        self.hs_peak_dirty = False
        self.hs_peak_value_text.set("— / 255")
        self.hs_peak_feedback.set("Đang đọc HS_PEAK (BASE sau 10k) từ ESP32...")
        self.estop_latched = None
        self.estop_pending = False
        self.start_pending = False
        self.set_auto_running(False)
        self.control_state.set("Đang kết nối — chờ STATUS từ firmware.")
        self.adc_status.set("ADC: chờ STATUS")
        self.control_state_label.configure(fg=AMBER)
        self.update_control_buttons()
        self.prev = None
        self.last_rx_total = self.reader.frames_rx
        self.last_csv_total = 0
        self.frames = 0
        self.rate_time = time.monotonic()
        self.last_diag_rx = None
        self.stop_ack_deadline = None
        self.stop_verify_pending = False
        self.stop_verify_deadline = None
        self.stop_verify_retries = 0
        self.next_diag_poll = self.rate_time + self.diag_interval_s
        self.loop_debug.set("LOOP: đang đo...    PID: -- Hz    SPP TX: -- Hz")
        self.connect_btn.config(text="Ngắt")
        self.status_label.config(text=f"Đã mở {port}", fg=GREEN)
        self.log(f"Đã mở {port}; chờ frame SPP từ ESP32")
        self.log("CSV: BẮT ĐẦU tạo file mới, DỪNG chốt file")
        self.csv_status.set("CSV: SẴN SÀNG — chưa ghi")
        self.calibration_ok = False
        self.cal_thresholds = [None] * 16
        self.cal_data = {}
        self.update_control_buttons()
        self.send_command("STATUS")
        self.send_command("CAL_STATUS")
        self.send_command("CAL_DUMP")
        self.send_command("ROUTE_LOCK")
        self.sensor_profile_confirmed = False
        self.send_command("SENSOR_PROFILE")
        self.map2_sensor_confirmed = False
        self.map2_sensor_pending = None
        self.send_command("MAP_SENSOR_PROFILE_V2:2")
        for map_id in (1, 2, 3):
            self.read_map_speed_profile(map_id)
        self.map3_sensor_confirmed = False
        self.send_command("MAP3_SENSOR_PROFILE")
        self.send_command("PID")
        self.send_command("STREAM:" + self.stream_rate.get())

    def request_diagnostics(self):
        if self.send_command("DIAG"):
            self.next_diag_poll = time.monotonic() + self.diag_interval_s
        self.send_command("PERF")

    def disconnect(self):
        if self.reader:
            self.reader.running = False
            self.reader.join(timeout=1)
            if self.run_csv.active is not None:
                self.finish_run_csv("MẤT KẾT NỐI; CHƯA XÁC NHẬN DỪNG")
            self.reader = None
        self.hs_peak_confirmed = None
        self.hs_peak_pending = None
        self.hs_peak_pending_at = None
        self.encoder_reset_pending = False
        self.encoder_reset_pending_at = None
        self.hs_peak_feedback.set("Chưa kết nối ESP32 — HS chưa xác nhận")
        self.route_status.set("ROUTE: mất kết nối ESP32")
        self.sensor_profile_confirmed = False
        self.sensor_profile_status.set("PID mắt: mất kết nối ESP32")
        self.map2_sensor_confirmed = False
        self.map2_sensor_pending = None
        self.map2_sensor_status.set("MAP 2 mắt PID: mất kết nối ESP32")
        self.map3_sensor_confirmed = False
        self.map3_sensor_status.set("MAP 3 mắt PID: mất kết nối ESP32")
        self.map_profile_pending = None
        for map_id in (1, 2, 3):
            self.map_profile_confirmed[map_id] = False
        self.map_profile_status.set("MAP BASE: mất kết nối ESP32")
        self.connect_btn.config(text="Kết nối")
        self.status_label.config(text="Chưa kết nối", fg=ROSE)
        self.estop_latched = None
        self.estop_pending = False
        self.start_pending = False
        self.stop_ack_deadline = None
        self.stop_verify_pending = False
        self.stop_verify_deadline = None
        self.stop_verify_retries = 0
        self.set_auto_running(False)
        self.control_state.set("Đã ngắt SPP — không thể xác nhận tình trạng động cơ.")
        self.adc_status.set("ADC: chưa kết nối")
        self.next_diag_poll = None
        self.last_diag_rx = None
        self.loop_debug.set("LOOP: -- Hz    PID: -- Hz    SPP TX: -- Hz    (mất kết nối)")
        self.control_state_label.configure(fg=ROSE)
        self.update_control_buttons()
        self.log("Đã đóng COM. Firmware phải dừng động cơ khi SPP mất kết nối.")

    def send_command(self, command):
        command = command.strip()
        if not command:
            return False
        if self.reader is None or not self.reader.is_alive():
            self.log(f"Chưa kết nối — không gửi: {command}")
            return False
        try:
            self.reader.send(command)
            self.log(f"TX > {command}")
            return True
        except (serial.SerialException, OSError) as exc:
            self.log(f"Lỗi gửi: {exc}")
            self.disconnect()
            return False

    def _can_reset_encoder(self):
        return (self.reader is not None and self.reader.is_alive()
                and not self.encoder_reset_pending
                and not self.auto_running and not self.start_pending
                and self.stop_ack_deadline is None
                and not self.stop_verify_pending
                and self.run_csv.active is None)

    def reset_encoder(self):
        if not self._can_reset_encoder():
            self.log("Reset ENC chỉ thực hiện khi robot đã dừng, không START/STOP đang chờ.")
            return False
        self.encoder_reset_pending = True
        self.encoder_reset_pending_at = time.monotonic()
        self.update_control_buttons()
        if not self.send_command("ENC_RESET"):
            self.encoder_reset_pending = False
            self.encoder_reset_pending_at = None
            self.update_control_buttons()
            return False
        return True

    def send_terminal(self):
        cmd = self.cmd_var.get().strip()
        if self.send_command(cmd):
            self.cmd_var.set("")

    def open_map_speed_editor(self):
        if self.map_profile_window is not None:
            try:
                if self.map_profile_window.winfo_exists():
                    self.map_profile_window.lift()
                    self.read_map_speed_profile(self.map_profile_map_id)
                    return
            except tk.TclError:
                self.map_profile_window = None

        win = tk.Toplevel(self)
        self.map_profile_window = win
        win.title("BASE PWM theo giai đoạn — MAP 1/2/3")
        win.configure(bg=PANEL)
        win.geometry("760x650")
        win.minsize(720, 560)
        win.transient(self)
        initial_map = self.selected_map if self.selected_map in (1, 2, 3) else 1
        self.map_profile_map_id = initial_map

        self.label(win,
                   "Mỗi MAP có số giai đoạn và profile Tick/BASE riêng. "
                   "Giai đoạn cuối chạy đến FINISH. Giới hạn 1–8 giai đoạn.",
                   bg=PANEL, fg=MUTED, wraplength=730, justify="left").pack(
                       anchor="w", padx=12, pady=(10, 6))

        top = tk.Frame(win, bg=PANEL, padx=12)
        top.pack(fill="x", pady=3)
        self.label(top, "MAP:", bg=PANEL, fg=CYAN).pack(side="left")
        map_var = tk.StringVar(value=str(initial_map))
        map_combo = ttk.Combobox(top, textvariable=map_var, values=("1", "2", "3"),
                                 state="readonly", width=5)
        map_combo.pack(side="left", padx=(5, 18))
        self.label(top, "Số giai đoạn:", bg=PANEL).pack(side="left")
        count_var = tk.StringVar(value=str(len(self.map_speed_profiles[initial_map]["bases"])))
        ttk.Entry(top, textvariable=count_var, width=5).pack(side="left", padx=5)
        row_widgets = tk.Frame(win, bg=PANEL, padx=10)
        row_widgets.pack(fill="both", expand=True, padx=4, pady=4)

        common = tk.Frame(win, bg=PANEL, padx=12, pady=5)
        common.pack(fill="x")
        self.label(common, "BASE chung (45–255):", bg=PANEL, fg=CYAN).pack(side="left")
        common_var = tk.StringVar(value=str(self.map_speed_profiles[initial_map]["bases"][0]))
        ttk.Entry(common, textvariable=common_var, width=8, justify="center").pack(side="left", padx=7)
        feedback = tk.StringVar(value="Đọc cấu hình từ ESP32 trước khi chỉnh sửa.")
        self.label(win, textvariable=feedback, bg=PANEL, fg=AMBER,
                   wraplength=730).pack(anchor="w", padx=12, pady=5)

        tick_vars = []
        base_vars = []
        start_labels = []
        loading = [False]

        def snapshot():
            try:
                ticks = [int(v.get().strip()) for v in tick_vars]
                bases = [int(v.get().strip()) for v in base_vars]
                return ticks, bases
            except (ValueError, tk.TclError):
                profile = self.map_speed_profiles[self.map_profile_map_id]
                return list(profile["ticks"]), list(profile["bases"])

        def mark_dirty(*_):
            if not loading[0]:
                self.map_profile_dirty = True

        def refresh_ranges(*_):
            try:
                values = [int(v.get().strip()) for v in tick_vars]
            except (ValueError, tk.TclError):
                values = []
            for i, label in enumerate(start_labels):
                start = 0 if i == 0 else (values[i - 1] if i - 1 < len(values) else 0)
                label.configure(text=f"{start:,}")

        def build_rows(ticks, bases):
            nonlocal tick_vars, base_vars, start_labels
            loading[0] = True
            for child in row_widgets.winfo_children():
                child.destroy()
            tick_vars, base_vars, start_labels = [], [], []
            count = len(bases)
            headers = tk.Frame(row_widgets, bg=PANEL)
            headers.pack(fill="x", pady=(0, 4))
            for title, width in (("Giai đoạn", 13), ("Từ tick", 14),
                                 ("Đến tick", 14), ("BASE PWM", 12)):
                self.label(headers, title, bg=PANEL, fg=CYAN,
                           width=width, anchor="w").pack(side="left", padx=4)
            for i in range(count):
                row = tk.Frame(row_widgets, bg=PANEL)
                row.pack(fill="x", pady=3)
                self.label(row, f"Giai đoạn {i+1}", bg=PANEL,
                           width=13, anchor="w").pack(side="left", padx=4)
                start_label = self.label(row, "0", bg=PANEL, fg=MUTED,
                                         width=14, anchor="w")
                start_label.pack(side="left", padx=4)
                start_labels.append(start_label)
                if i < count - 1:
                    var = tk.StringVar(value=str(ticks[i]))
                    var.trace_add("write", mark_dirty)
                    var.trace_add("write", refresh_ranges)
                    tick_vars.append(var)
                    ttk.Entry(row, textvariable=var, width=14,
                              justify="center").pack(side="left", padx=4)
                else:
                    self.label(row, "FINISH", bg=PANEL, fg=MUTED,
                               width=14, anchor="w").pack(side="left", padx=4)
                base_var = tk.StringVar(value=str(bases[i]))
                base_var.trace_add("write", mark_dirty)
                base_vars.append(base_var)
                ttk.Entry(row, textvariable=base_var, width=12,
                          justify="center").pack(side="left", padx=4)
            count_var.set(str(count))
            common_var.set(str(bases[0]))
            refresh_ranges()
            loading[0] = False

        def rebuild():
            try:
                count = int(count_var.get().strip())
            except ValueError:
                feedback.set("Số giai đoạn phải là số nguyên từ 1 đến 8.")
                return
            if not 1 <= count <= 8:
                feedback.set("Chỉ hỗ trợ từ 1 đến 8 giai đoạn.")
                return
            old_ticks, old_bases = snapshot()
            ticks = old_ticks[:max(0, count - 1)]
            while len(ticks) < count - 1:
                previous = ticks[-1] if ticks else 0
                ticks.append(previous + 1000)
            bases = old_bases[:count]
            while len(bases) < count:
                bases.append(bases[-1] if bases else 160)
            build_rows(ticks, bases)
            self.map_profile_dirty = True
            feedback.set(f"Đã tạo {count} giai đoạn cho MAP {self.map_profile_map_id}; nhấn Áp dụng cấu hình.")

        def populate(map_id):
            profile = self.map_speed_profiles[map_id]
            build_rows(list(profile["ticks"]), list(profile["bases"]))
            self.map_profile_dirty = False

        def read_profile():
            if self.map_profile_pending is not None:
                feedback.set("Đang chờ ESP32 xác nhận lần ghi trước.")
                return
            self.map_profile_dirty = False
            if self.read_map_speed_profile(self.map_profile_map_id):
                feedback.set(f"Đang đọc profile BASE của MAP {self.map_profile_map_id}...")
            else:
                feedback.set("Chưa kết nối Bluetooth SPP.")

        def change_map(_event=None):
            try:
                target = int(map_var.get())
            except ValueError:
                return
            if self.map_profile_dirty or self.map_profile_pending is not None:
                map_var.set(str(self.map_profile_map_id))
                feedback.set("Áp dụng hoặc Đọc lại trước khi chuyển MAP.")
                return
            if not self.select_map(target):
                map_var.set(str(self.map_profile_map_id))
                return
            self.map_profile_map_id = target
            populate(target)
            read_profile()

        def apply_common_base():
            try:
                value = int(common_var.get().strip())
            except ValueError:
                feedback.set("BASE chung phải là số nguyên.")
                return
            if not 45 <= value <= 255:
                feedback.set("BASE chung phải nằm trong khoảng 45–255.")
                return
            loading[0] = True
            for var in base_vars:
                var.set(str(value))
            loading[0] = False
            self.map_profile_dirty = True
            feedback.set(f"Đã đặt BASE={value} cho toàn bộ {len(base_vars)} giai đoạn. Nhấn Áp dụng cấu hình để gửi.")

        def apply_profile():
            map_id = self.map_profile_map_id
            if (map_id not in (1, 2, 3) or self.selected_map != map_id
                    or self.reader is None or not self.reader.is_alive()
                    or self.auto_running or self.start_pending
                    or self.stop_ack_deadline is not None
                    or self.stop_verify_pending or self.run_csv.active is not None
                    or self.map_profile_pending is not None):
                feedback.set("Cần chọn đúng MAP, kết nối SPP, robot STOP và không có ACK đang chờ.")
                return
            try:
                ticks = [int(v.get().strip()) for v in tick_vars]
                bases = [int(v.get().strip()) for v in base_vars]
                count = int(count_var.get().strip())
                if count != len(bases) or count < 1 or count > 8 or len(ticks) != count - 1:
                    raise ValueError("Số giai đoạn không khớp số dòng.")
                previous = 0
                for tick in ticks:
                    if tick <= previous or tick > 10000000:
                        raise ValueError("Mốc Tick phải tăng nghiêm ngặt và không vượt 10.000.000.")
                    previous = tick
                if any(value < 45 or value > 255 for value in bases):
                    raise ValueError("BASE PWM phải nằm trong khoảng 45–255.")
            except ValueError as exc:
                feedback.set(f"Cấu hình không hợp lệ: {exc}")
                return
            pairs = []
            for i, base in enumerate(bases):
                pairs.extend((ticks[i] if i < len(ticks) else 0, base))
            command = "MAP_SPEED_PROFILE_V2:" + ",".join(
                str(v) for v in (map_id, count, *pairs))
            if self.send_command(command):
                self.map_profile_pending = (map_id, ticks, bases)
                self.map_profile_confirmed[map_id] = False
                self.map_profile_dirty = False
                self.map_profile_status.set(f"MAP {map_id} BASE: đang chờ ESP32 xác nhận")
                feedback.set(f"Đã gửi {count} giai đoạn BASE cho MAP {map_id}; chờ ESP32 đọc lại.")

        def defaults():
            default = {
                1: ([3800, 7800, 8350, 10300, 11400, 12800, 15000],
                    [140, 140, 140, 240, 140, 140, 210, 255]),
                2: ([3800, 7800, 8350, 10300, 11400, 12800, 15000], [160] * 8),
                3: ([3800, 7800, 8350, 10300, 11400, 12800, 15000], [150] * 8),
            }[self.map_profile_map_id]
            build_rows(list(default[0]), list(default[1]))
            self.map_profile_dirty = True
            feedback.set("Đã nạp mặc định của MAP; nhấn Áp dụng cấu hình để gửi.")

        actions = tk.Frame(win, bg=PANEL, padx=12, pady=8)
        actions.pack(fill="x")
        ttk.Button(actions, text="Tạo số giai đoạn",
                   command=rebuild).pack(side="left", padx=(0, 6))
        ttk.Button(actions, text="BASE chung → tất cả",
                   command=apply_common_base).pack(side="left", padx=6)
        ttk.Button(actions, text="Đọc từ ESP32",
                   command=read_profile).pack(side="left", padx=6)
        ttk.Button(actions, text="Áp dụng cấu hình",
                   command=apply_profile).pack(side="left", padx=6)
        ttk.Button(actions, text="Mặc định",
                   command=defaults).pack(side="left", padx=6)
        ttk.Button(actions, text="Đóng",
                   command=lambda: self._close_map_profile_window(win)).pack(side="right")

        self.map_profile_editor_refresh = lambda map_id: (
            populate(map_id) if map_id == self.map_profile_map_id
            and win.winfo_exists() and not self.map_profile_dirty else None)
        map_combo.bind("<<ComboboxSelected>>", change_map)
        count_var.trace_add("write", mark_dirty)
        common_var.trace_add("write", mark_dirty)
        build_rows(list(self.map_speed_profiles[initial_map]["ticks"]),
                   list(self.map_speed_profiles[initial_map]["bases"]))
        if self.selected_map != initial_map and not self.select_map(initial_map):
            feedback.set("Không thể đổi MAP khi robot chưa xác nhận STOP.")
        read_profile()
        win.protocol("WM_DELETE_WINDOW", lambda: self._close_map_profile_window(win))

    def _close_map_profile_window(self, win):
        self.map_profile_window = None
        self.map_profile_editor_refresh = None
        self.map_profile_tick_vars = []
        self.map_profile_base_vars = []
        self.map_profile_stage_labels = []
        win.destroy()

    def read_map_speed_profile(self, map_id):
        if map_id not in (1, 2, 3):
            return False
        if self.reader is None or not self.reader.is_alive():
            return False
        if map_id == self.selected_map:
            self.map_profile_confirmed[map_id] = False
            self.update_control_buttons()
        return self.send_command(f"MAP_SPEED_PROFILE_V2:{map_id}")

    def open_map2_sensor_editor(self):
        """Edit 1–8 Map 2 PID eye groups and tick boundaries while stopped."""
        if self.map2_sensor_window is not None:
            try:
                if self.map2_sensor_window.winfo_exists():
                    self.map2_sensor_window.lift()
                    return
            except tk.TclError:
                pass

        win = tk.Toplevel(self)
        self.map2_sensor_window = win
        win.title("MAP 2 — Mắt PID / Tick (1–8 giai đoạn)")
        win.configure(bg=PANEL)
        win.geometry("1300x730")
        win.minsize(1110, 580)
        win.transient(self)

        top = tk.Frame(win, bg=PANEL, padx=12, pady=8)
        top.pack(fill="x")
        self.label(top, "Số giai đoạn (1–8):", bg=PANEL,
                   fg=CYAN).pack(side="left")
        count_var = tk.StringVar(value=str(len(self.map2_sensor_profile[1])))
        ttk.Entry(top, textvariable=count_var, width=5).pack(
            side="left", padx=6)
        self.label(top, "Chọn số giai đoạn rồi nhấn Tạo số giai đoạn.",
                   bg=PANEL, fg=MUTED).pack(side="left", padx=8)
        self.label(
            win,
            "Mỗi giai đoạn dùng một mask S0–S15 và một mốc tick kết thúc; "
            "giai đoạn cuối chạy đến FINISH. Chỉ thay mắt tính PID, "
            "không thay đổi 16 ADC vật lý, START/FINISH hoặc phát hiện cua.",
            bg=PANEL, fg=MUTED, wraplength=1250, justify="left"
        ).pack(anchor="w", padx=12, pady=(0, 4))
        self.label(
            win, "MAP 2: tại mỗi mốc tick, căn line rồi giữ ổn định 10 ms "
            "trước khi chuyển sang giai đoạn tiếp theo.",
            bg=PANEL, fg=AMBER, wraplength=1250, justify="left"
        ).pack(anchor="w", padx=12, pady=(0, 4))
        stages_frame = tk.Frame(win, bg=PANEL, padx=8)
        stages_frame.pack(fill="both", expand=True, padx=6, pady=4)
        feedback = tk.StringVar(value="Chỉ thay đổi khi đã chọn MAP 2 và AUTO đã dừng.")
        self.label(win, textvariable=feedback, bg=PANEL, fg=CYAN,
                   wraplength=1250).pack(anchor="w", padx=12, pady=5)

        tick_vars = []
        mask_vars = []
        start_labels = []
        loading = [False]
        dirty = [False]

        def mark_dirty(*_):
            if not loading[0]:
                dirty[0] = True

        def refresh_ranges(*_):
            try:
                values = [int(var.get().strip()) for var in tick_vars]
            except (ValueError, tk.TclError):
                values = []
            for i, label in enumerate(start_labels):
                start = 0 if i == 0 else (
                    values[i - 1] if i - 1 < len(values) else 0)
                label.configure(text=f"{start:,}")

        def snapshot():
            try:
                ticks = [int(var.get().strip()) for var in tick_vars]
                masks = [sum(1 << bit for bit, state in enumerate(row)
                             if state.get()) for row in mask_vars]
                return ticks, masks
            except (ValueError, tk.TclError):
                ticks, masks = self.map2_sensor_profile
                return list(ticks), list(masks)

        def build_rows(ticks, masks):
            nonlocal tick_vars, mask_vars, start_labels
            loading[0] = True
            for widget in stages_frame.winfo_children():
                widget.destroy()
            tick_vars, mask_vars, start_labels = [], [], []
            count = len(masks)
            header = tk.Frame(stages_frame, bg=PANEL)
            header.pack(fill="x", pady=(0, 3))
            for title, width in (("Giai đoạn", 13), ("Từ tick", 14),
                                 ("Đến tick", 14)):
                self.label(header, title, bg=PANEL, fg=CYAN,
                           width=width, anchor="w").pack(side="left", padx=4)
            self.label(header, "16 mắt PID (S0–S15)", bg=PANEL,
                       fg=CYAN).pack(side="left", padx=8)
            for i, mask in enumerate(masks):
                row = tk.Frame(stages_frame, bg=PANEL, padx=3, pady=3,
                               highlightthickness=1, highlightbackground=GREY)
                row.pack(fill="x", pady=2)
                self.label(row, f"Giai đoạn {i + 1}", bg=PANEL,
                           width=13, anchor="w").pack(side="left", padx=4)
                label = self.label(row, "0", bg=PANEL, fg=MUTED,
                                   width=14, anchor="w")
                label.pack(side="left", padx=4)
                start_labels.append(label)
                if i < count - 1:
                    var = tk.StringVar(value=str(ticks[i]))
                    var.trace_add("write", mark_dirty)
                    var.trace_add("write", refresh_ranges)
                    tick_vars.append(var)
                    ttk.Entry(row, textvariable=var, width=13,
                              justify="center").pack(side="left", padx=4)
                else:
                    self.label(row, "FINISH", bg=PANEL, fg=MUTED,
                               width=13, anchor="w").pack(side="left", padx=4)
                states = [tk.BooleanVar(value=bool(mask & (1 << bit)))
                          for bit in range(16)]
                mask_vars.append(states)
                for bit, state in enumerate(states):
                    ttk.Checkbutton(row, text=f"S{bit}", variable=state,
                                    command=mark_dirty).pack(
                                        side="left", padx=1)
            count_var.set(str(count))
            refresh_ranges()
            loading[0] = False

        def rebuild():
            try:
                count = int(count_var.get().strip())
            except ValueError:
                feedback.set("Số giai đoạn phải là số nguyên từ 1 đến 8.")
                return
            if not 1 <= count <= 8:
                feedback.set("Chỉ hỗ trợ từ 1 đến 8 giai đoạn.")
                return
            ticks, masks = snapshot()
            ticks = ticks[:count - 1]
            while len(ticks) < count - 1:
                ticks.append((ticks[-1] if ticks else 0) + 1000)
            masks = masks[:count]
            while len(masks) < count:
                masks.append(0xFFFF)
            build_rows(ticks, masks)
            dirty[0] = True
            feedback.set(f"Đã tạo {count} giai đoạn; nhấn Áp dụng để gửi xuống ESP32.")

        def populate():
            ticks, masks = self.map2_sensor_profile
            build_rows(list(ticks), list(masks))
            dirty[0] = False

        def read_profile():
            if self.map2_sensor_pending is not None:
                feedback.set("Đang chờ ESP32 xác nhận cấu hình vừa gửi.")
            elif self.send_command("MAP_SENSOR_PROFILE_V2:2"):
                feedback.set("Đang đọc cấu hình MAP 2 từ ESP32...")
            else:
                feedback.set("Chưa kết nối Bluetooth SPP.")

        def apply_profile():
            connected = self.reader is not None and self.reader.is_alive()
            locked = (self.auto_running or self.start_pending or
                      self.stop_ack_deadline is not None or
                      self.stop_verify_pending or
                      self.run_csv.active is not None)
            if (self.selected_map != 2 or not connected or locked or
                    self.map2_sensor_pending is not None):
                feedback.set("Chọn MAP 2, kết nối SPP, STOP AUTO và chờ ACK trước khi áp dụng.")
                return
            try:
                count = int(count_var.get().strip())
                ticks = [int(var.get().strip()) for var in tick_vars]
                masks = [sum(1 << bit for bit, state in enumerate(row)
                             if state.get()) for row in mask_vars]
                if not (1 <= count <= 8) or count != len(masks) or len(ticks) != count - 1:
                    raise ValueError("Số giai đoạn không khớp số dòng.")
                previous = 0
                for tick in ticks:
                    if tick <= previous or tick > 10000000:
                        raise ValueError("Các mốc tick phải tăng dần và không vượt 10.000.000.")
                    previous = tick
                if any(mask == 0 for mask in masks):
                    raise ValueError("Mỗi giai đoạn phải bật ít nhất một mắt PID.")
            except ValueError as exc:
                feedback.set(f"Cấu hình không hợp lệ: {exc}")
                return
            pairs = []
            for i, mask in enumerate(masks):
                pairs.extend((ticks[i] if i < len(ticks) else 0, f"{mask:04X}"))
            command = f"MAP_SENSOR_PROFILE_V2:2,{count}," + ",".join(
                str(part) for part in pairs)
            if self.send_command(command):
                self.map2_sensor_pending = (list(ticks), list(masks))
                self.map2_sensor_confirmed = False
                dirty[0] = False
                self.map2_sensor_status.set("MAP 2 mắt PID: chờ ESP32 đọc lại cấu hình...")
                feedback.set("Đã gửi; đang chờ ESP32 trả về cấu hình để đối chiếu.")

        def defaults():
            build_rows([1000, 2000, 3000], [0xFFFF] * 4)
            dirty[0] = True
            feedback.set("Đã nạp mặc định 4 giai đoạn; nhấn Áp dụng để gửi.")

        def close():
            self.map2_sensor_window = None
            self.map2_sensor_editor_refresh = None
            win.destroy()

        def refresh():
            if win.winfo_exists() and not dirty[0]:
                populate()
                feedback.set("Đã nhận cấu hình MAP 2 từ ESP32.")

        self.map2_sensor_editor_refresh = refresh
        actions = tk.Frame(win, bg=PANEL, padx=12, pady=8)
        actions.pack(fill="x")
        ttk.Button(actions, text="Tạo số giai đoạn",
                   command=rebuild).pack(side="left", padx=5)
        ttk.Button(actions, text="Đọc từ ESP32",
                   command=read_profile).pack(side="left", padx=5)
        ttk.Button(actions, text="Áp dụng cấu hình",
                   command=apply_profile).pack(side="left", padx=5)
        ttk.Button(actions, text="Mặc định",
                   command=defaults).pack(side="left", padx=5)
        ttk.Button(actions, text="Đóng",
                   command=close).pack(side="right")
        count_var.trace_add("write", mark_dirty)
        populate()
        win.protocol("WM_DELETE_WINDOW", close)

    def open_map3_sensor_editor(self):
        """Map 3 only: four encoder-selected PID masks, continuous motion."""
        if self.map3_sensor_window is not None:
            try:
                if self.map3_sensor_window.winfo_exists():
                    self.map3_sensor_window.lift()
                    return
            except tk.TclError:
                pass
        win = tk.Toplevel(self)
        self.map3_sensor_window = win
        win.title("MAP 3 — 4 giai đoạn chọn mắt PID")
        win.configure(bg=PANEL)
        win.geometry("850x505")
        win.minsize(780, 475)
        win.transient(self)

        def close():
            self.map3_sensor_window = None
            win.destroy()

        win.protocol("WM_DELETE_WINDOW", close)
        t1, t2, t3, *initial_masks = self.map3_sensor_profile
        tickvars = [tk.StringVar(value=str(x)) for x in (t1, t2, t3)]
        tickrow = tk.Frame(win, bg=PANEL, padx=10, pady=9)
        tickrow.pack(fill="x")
        for i, var in enumerate(tickvars, 1):
            self.label(tickrow, f"Mốc {i} (tick):", bg=PANEL).pack(side="left")
            ttk.Entry(tickrow, textvariable=var, width=9).pack(
                side="left", padx=(4, 13))
        self.label(
            win,
            "Chỉ chọn mắt dùng để tính PID; vẫn đọc đủ 16 ADC cho mất line, cua và FINISH.",
            bg=PANEL, fg=MUTED).pack(anchor="w", padx=12)
        self.label(
            win,
            "Theo tick encoder từ lúc AUTO bắt đầu. Chuyển giai đoạn liên tục, KHÔNG dừng xe.",
            bg=PANEL, fg=AMBER).pack(anchor="w", padx=12, pady=(3, 5))

        groups = []
        titles = (
            "Giai đoạn 1: 0 → mốc 1",
            "Giai đoạn 2: mốc 1 → mốc 2",
            "Giai đoạn 3: mốc 2 → mốc 3",
            "Giai đoạn 4: từ mốc 3")
        for title, mask in zip(titles, initial_masks):
            group = tk.LabelFrame(
                win, text=title, bg=PANEL, fg=FG, padx=7, pady=7)
            group.pack(fill="x", padx=10, pady=4)
            bits = [tk.BooleanVar(value=bool(mask & (1 << i)))
                    for i in range(16)]
            groups.append(bits)
            for i, var in enumerate(bits):
                ttk.Checkbutton(group, text=f"S{i}", variable=var).grid(
                    row=0, column=i, padx=2)

        feedback = tk.StringVar(
            value="Cấu hình chỉ áp dụng khi robot đã dừng.")
        self.label(win, textvariable=feedback, bg=PANEL, fg=CYAN).pack(
            anchor="w", padx=12, pady=(7, 2))
        row = tk.Frame(win, bg=PANEL, padx=12, pady=4)
        row.pack(fill="x")

        def apply():
            connected = self.reader is not None and self.reader.is_alive()
            locked = (
                self.auto_running or self.start_pending or
                self.stop_ack_deadline is not None or
                self.stop_verify_pending or self.run_csv.active is not None)
            if not connected or locked:
                feedback.set("Chỉ chỉnh khi đã kết nối SPP và AUTO dừng.")
                return
            try:
                ticks = [int(v.get().strip()) for v in tickvars]
            except ValueError:
                feedback.set("Mốc tick phải là số nguyên.")
                return
            masks = [
                sum((1 << i) for i, v in enumerate(group) if v.get())
                for group in groups]
            if not (0 < ticks[0] < ticks[1] < ticks[2] <= 10000000) or not all(masks):
                feedback.set(
                    "Cần 0 < T1 < T2 < T3, mỗi giai đoạn bật ít nhất 1 mắt.")
                return
            command = (
                "MAP3_SENSOR_PROFILE:{},{},{},{:04X},{:04X},{:04X},{:04X}"
                .format(*ticks, *masks))
            if self.send_command(command):
                self.map3_sensor_confirmed = False
                self.map3_sensor_status.set(
                    "MAP 3 mắt PID: đang chờ ESP32 xác nhận...")
                feedback.set("Đã gửi, chờ ESP32 xác nhận cấu hình.")

        ttk.Button(row, text="Áp dụng xuống ESP32", command=apply).pack(
            side="left", padx=(0, 8))
        ttk.Button(
            row, text="Đọc cấu hình từ ESP32",
            command=lambda: self.send_command("MAP3_SENSOR_PROFILE")).pack(
                side="left")
        ttk.Button(row, text="Đóng", command=close).pack(side="right")

    def open_sensor_profile_editor(self):
        """Edit per-stage masks and switch ticks; changes require robot stopped."""
        if self.sensor_profile_window is not None:
            try:
                self.sensor_profile_window.lift()
                return
            except tk.TclError:
                self.sensor_profile_window = None
        win = tk.Toplevel(self)
        self.sensor_profile_window = win
        win.title("MAP 1 — chọn mắt PID theo encoder")
        win.configure(bg=PANEL)
        win.geometry("780x390")
        win.minsize(730, 360)
        win.transient(self)
        t1, t2, m1, m2, m3 = self.sensor_profile
        threshold1 = tk.StringVar(value=str(t1))
        threshold2 = tk.StringVar(value=str(t2))
        row = tk.Frame(win, bg=PANEL, padx=12, pady=10)
        row.pack(fill="x")
        self.label(row, "Mốc 1 (ticks):", bg=PANEL).pack(side="left")
        ttk.Entry(row, textvariable=threshold1, width=9).pack(side="left", padx=(5, 20))
        self.label(row, "Mốc 2 (ticks):", bg=PANEL).pack(side="left")
        ttk.Entry(row, textvariable=threshold2, width=9).pack(side="left", padx=5)
        self.label(win, "Tích chọn mắt được DÙNG CHO PID. ADC / mất line / FINISH vẫn đọc đủ 16 mắt.",
                   bg=PANEL, fg=MUTED).pack(anchor="w", padx=12)
        values = []
        for index, (label, mask) in enumerate(
                (("GĐ 1: 0 → mốc 1", m1),
                 ("GĐ 2: mốc 1 → mốc 2", m2),
                 ("GĐ 3: từ mốc 2", m3))):
            group = tk.LabelFrame(win, text=label, bg=PANEL, fg=FG,
                                  padx=6, pady=7)
            group.pack(fill="x", padx=10, pady=5)
            bits = [tk.BooleanVar(value=bool(mask & (1 << i)))
                    for i in range(16)]
            values.append(bits)
            for i, state in enumerate(bits):
                ttk.Checkbutton(group, text=f"S{i}", variable=state).grid(
                    row=0, column=i, padx=2)
        feedback = tk.StringVar(value="Áp dụng khi robot đã STOP; không thay đổi PID gains.")
        self.label(win, textvariable=feedback, bg=PANEL, fg=AMBER).pack(
            anchor="w", padx=12, pady=(7, 2))
        actions = tk.Frame(win, bg=PANEL, padx=12, pady=4)
        actions.pack(fill="x")

        def apply():
            if (self.reader is None or not self.reader.is_alive()
                    or self.auto_running or self.start_pending
                    or self.stop_ack_deadline is not None
                    or self.stop_verify_pending or self.run_csv.active is not None):
                feedback.set("Robot phải dừng và đã kết nối SPP trước khi áp dụng.")
                return
            try:
                a = int(threshold1.get().strip())
                b = int(threshold2.get().strip())
            except ValueError:
                feedback.set("Mốc encoder phải là số nguyên.")
                return
            masks = [sum(1 << i for i, selected in enumerate(bits)
                         if selected.get()) for bits in values]
            if not (0 < a < b <= 10000000) or any(mask == 0 for mask in masks):
                feedback.set("Cần 0 < mốc 1 < mốc 2; mỗi giai đoạn phải bật ít nhất 1 mắt.")
                return
            command = "SENSOR_PROFILE:{},{},{:04X},{:04X},{:04X}".format(
                a, b, *masks)
            if self.send_command(command):
                self.sensor_profile_confirmed = False
                self.sensor_profile_status.set("PID mắt: đang chờ ESP32 xác nhận...")
                feedback.set("Đã gửi cấu hình. Chờ dòng SENSOR_PROFILE từ ESP32.")

        ttk.Button(actions, text="Áp dụng xuống ESP32",
                   command=apply).pack(side="left", padx=(0, 8))
        ttk.Button(actions, text="Đọc cấu hình đang dùng",
                   command=lambda: self.send_command("SENSOR_PROFILE")).pack(side="left")
        ttk.Button(actions, text="Đóng",
                   command=win.destroy).pack(side="right")
        win.protocol("WM_DELETE_WINDOW", win.destroy)

    def preview_pwm(self, *_args):
        self.pwm_text.set(f"{round(self.pwm_var.get())} / 255")

    def _hs_base_limit(self):
        """Use the app's requested BASE for early validation; ESP32 rechecks."""
        try:
            return int(self.pid_vars["Base"].get().strip())
        except (ValueError, AttributeError):
            return 0

    def preview_hs_peak(self, *_args):
        value = max(100, min(255, round(self.hs_peak_var.get())))
        self.hs_peak_dirty = True
        self.hs_peak_value_text.set(f"{value} (chưa áp dụng)")
        self.hs_peak_feedback.set("Nhấn Áp dụng HS để gửi ESP32, hoặc Đọc HS để hủy chọn.")
        if hasattr(self, "hs_peak_widgets"):
            self.update_control_buttons()

    def preset_hs_peak(self, value):
        if not self._can_edit_hs_peak() or self.hs_peak_pending is not None:
            return
        self.hs_peak_var.set(value)
        self.preview_hs_peak()

    def _can_edit_hs_peak(self):
        return (self.reader is not None and self.reader.is_alive()
                and self.estop_latched is False and not self.estop_pending
                and not self.auto_running and not self.start_pending
                and self.stop_ack_deadline is None
                and not self.stop_verify_pending
                and self.run_csv.active is None)

    def read_hs_peak(self):
        if self.hs_peak_pending is not None:
            self.log("Chờ ESP32 xác nhận HS_PEAK trước khi đọc lại.")
            return False
        if self.reader is None or not self.reader.is_alive():
            self.log("Chưa kết nối ESP32 để đọc HS_PEAK.")
            return False
        self.hs_peak_dirty = False
        self.hs_peak_confirmed = None
        self.hs_peak_feedback.set("Đang đọc lại HS_PEAK từ firmware...")
        self.update_control_buttons()
        return self.send_command("HS_PEAK")

    def apply_hs_peak(self):
        if not self._can_edit_hs_peak() or self.hs_peak_pending is not None:
            self.log("HS_PEAK chỉ chỉnh được khi đã kết nối và xe đã dừng.")
            return False
        peak = max(100, min(255, round(self.hs_peak_var.get())))
        base = self._hs_base_limit()
        if peak < base:
            self.hs_peak_feedback.set(f"Không hợp lệ: HS_PEAK {peak} < BASE {base}")
            self.log(f"Không áp dụng HS_PEAK={peak}: thấp hơn BASE={base}.")
            return False
        self.hs_peak_pending = peak
        self.hs_peak_pending_at = time.monotonic()
        self.hs_peak_feedback.set(f"Đang chờ ESP32 xác nhận PWM {peak}...")
        self.update_control_buttons()
        if not self.send_command(f"HS_PEAK:{peak}"):
            self.hs_peak_pending = None
            self.hs_peak_pending_at = None
            self.update_control_buttons()
            return False
        return True

    def apply_pwm(self):
        pwm = max(0, min(255, round(self.pwm_var.get())))
        if self.send_command(f"V:{pwm}"):
            self.pid_vars["Base"].set(str(pwm))

    def preset_pwm(self, value):
        self.pwm_var.set(value)
        self.preview_pwm()
        self.apply_pwm()

    def apply_pid(self):
        try:
            vals = [float(self.pid_vars[key].get()) for key in ("Kp", "Ki", "Kd")]
            base_s = self.pid_vars["Base"].get().strip()
            base = int(base_s)
            if not all(math.isfinite(value) and 0 <= value <= 50 for value in vals):
                raise ValueError("Kp, Ki, Kd phải là số hữu hạn từ 0 đến 50")
            if str(base) != base_s or not 0 <= base <= 255:
                raise ValueError("Base phải là số nguyên từ 0 đến 255")
        except ValueError as exc:
            self.log(f"Thông số PID không hợp lệ: {exc}")
            return False
        for command in (f"KP:{vals[0]:.2f}", f"KI:{vals[1]:.2f}",
                        f"KD:{vals[2]:.2f}", f"BASE:{base}"):
            if not self.send_command(command):
                return False
        self.pwm_var.set(base)
        self.preview_pwm()
        self.log("Đã GỬI các lệnh PID, đang chờ firmware xác nhận.")
        return True

    def select_map(self, map_id):
        """Select the destination for future runs, never relabel a live run."""
        if map_id not in (1, 2, 3):
            raise ValueError("Map must be 1, 2 or 3")
        if (self.run_csv.active is not None or self.auto_running
                or self.start_pending or self.stop_ack_deadline is not None
                or self.stop_verify_pending):
            self.log("Không thể đổi MAP khi xe đang chạy hoặc chưa xác nhận STOP.")
            return False
        if self.map_profile_dirty or self.map_profile_pending is not None:
            self.log("Hãy áp dụng/đọc lại cấu hình BASE trước khi đổi MAP.")
            return False
        # A physical GPIO34 START uses the firmware's stored MAP.
        # Update the ESP32 as soon as the user selects a map, not only
        # when the Python START button sends AUTO:map_id.
        if self.reader is not None and self.reader.is_alive():
            if not self.send_command(f"MAP:{map_id}"):
                self.log("Chưa đồng bộ MAP với ESP32; kiểm tra kết nối.")
                return False
        self.selected_map = map_id
        self.map_status.set(f"Đã chọn {map_id}")
        for number, button in self.map_buttons.items():
            chosen = number == map_id
            button.configure(bg=CYAN if chosen else GREY,
                             fg=BG if chosen else FG)
        self.log(f"Đã chọn MAP {map_id}; các lượt START kế tiếp lưu trong logs/map_{map_id:02d}/")
        if self.reader is not None and self.reader.is_alive():
            self.read_map_speed_profile(map_id)
        self.update_control_buttons()
        return True

    def update_control_buttons(self):
        connected = self.reader is not None and self.reader.is_alive()
        can_start = (connected and self.estop_latched is False
                     and not self.encoder_reset_pending
                     and not self.estop_pending and not self.start_pending
                     and not self.auto_running and self.run_csv.active is None
                     and self.stop_ack_deadline is None
                     and not self.stop_verify_pending
                     and self.calibration_ok and self.selected_map is not None
                     and self.map_profile_pending is None
                     and self.map_profile_confirmed.get(self.selected_map, False)
                     and all(v is not None for v in self.cal_thresholds))
        self.start_btn.configure(state="normal" if can_start else "disabled")
        self.stop_btn.configure(state="normal" if connected else "disabled")
        locked = (self.run_csv.active is not None or self.auto_running
                  or self.start_pending or self.stop_ack_deadline is not None
                  or self.stop_verify_pending)
        for button in self.map_buttons.values():
            button.configure(state="disabled" if locked else "normal")
        if hasattr(self,"map2_sensor_btn"):
            self.map2_sensor_btn.configure(
                state="normal" if self.selected_map == 2 else "disabled")
            self.map2_sensor_read_btn.configure(
                state="normal" if connected else "disabled")
        if hasattr(self, "sensor_profile_btn"):
            self.sensor_profile_btn.configure(state="normal")
            self.sensor_profile_read_btn.configure(
                state="normal" if connected else "disabled")
        if hasattr(self, "reset_encoder_btn"):
            self.reset_encoder_btn.configure(
                state="normal" if self._can_reset_encoder() else "disabled")
    def set_auto_running(self, running):
        now = time.monotonic()
        if running and not self.auto_running:
            self.run_started_at = now
        elif not running and self.auto_running:
            if self.run_started_at is not None:
                self.run_elapsed_seconds += now - self.run_started_at
            self.run_started_at = None
        self.auto_running = running
        self.update_run_timer_display(now)

    def reset_run_timer(self):
        self.run_elapsed_seconds = 0.0
        self.run_started_at = time.monotonic() if self.auto_running else None
        self.update_run_timer_display()

    def update_run_timer_display(self, now=None):
        elapsed = self.run_elapsed_seconds
        if self.auto_running and self.run_started_at is not None:
            elapsed += (time.monotonic() if now is None else now) - self.run_started_at
        total_ticks = max(0, int(elapsed * 100))
        total_seconds, ticks = divmod(total_ticks, 100)
        hours, remainder = divmod(total_seconds, 3600)
        minutes, seconds = divmod(remainder, 60)
        self.run_timer_var.set(
            f"{hours:02d}:{minutes:02d}:{seconds:02d}.{ticks:02d}")

    def finish_run_csv(self, reason):
        recorder = self.run_csv.finish(self.reader, reason)
        self.stop_ack_deadline = None
        self.start_confirmed = False
        if recorder is None:
            return
        self.csv_status.set(f"CSV ĐANG ĐÓNG: {recorder.path.name}")
        self.log(f"CSV đang chốt ({reason}): {recorder.path}")
        if self.run_txt_file is not None:
            try:
                self.run_txt_file.close()
            except OSError as exc:
                self.csv_status.set(f"TXT CLOSE ERROR: {exc}")
            self.run_txt_file = None

    def start_line(self):
        if self.encoder_reset_pending:
            self.log("Đang chờ ACK reset encoder; chưa gửi START.")
            return
        if self.selected_map is None:
            self.log("Hãy chọn MAP 1, MAP 2 hoặc MAP 3 trước khi START.")
            return
        if (not self.map_profile_confirmed.get(self.selected_map, False)
                or self.map_profile_pending is not None):
            self.log("Chưa xác nhận cấu hình BASE của MAP đang chọn.")
            self.read_map_speed_profile(self.selected_map)
            return
        if (self.estop_latched is not False or self.auto_running or self.start_pending
                or self.stop_ack_deadline is not None or self.stop_verify_pending):
            self.log("Chưa thể START: AUTO đang chạy hoặc chưa xác nhận STOP.")
            return
        if self.reader is None or not self.reader.is_alive():
            self.log("Chưa kết nối Bluetooth SPP.")
            return
        if not self.calibration_ok or any(v is None for v in self.cal_thresholds):
            self.log("Chưa nhận đủ 16 ngưỡng TH từ ESP32; gửi lại CAL_DUMP.")
            self.send_command("CAL_DUMP")
            return
        try:
            calib = (self.cal_min, self.cal_max)
            recorder = self.run_csv.begin(
                self.reader, CSV_HEADER, calib, list(self.cal_thresholds),
                self.selected_map)
        except (OSError, RuntimeError, ValueError) as exc:
            self.log(f"Không tạo được CSV, chưa gửi AUTO: {exc}")
            return
        try:
            self.run_txt_path = recorder.path.with_suffix(".txt")
            self.run_txt_file = self.run_txt_path.open(
                "w", encoding="utf-8", buffering=1)
        except OSError as exc:
            self.run_txt_file = None
            self.run_csv.finish(self.reader, "KHÔNG TẠO ĐƯỢC TXT")
            self.log(f"Không tạo được TXT, chưa gửi AUTO: {exc}")
            self.update_control_buttons()
            return
        self.last_csv_total = 0
        self.stop_ack_deadline = None
        self.start_confirmed = False
        self.csv_status.set(f"CSV GHI: {recorder.path.name}")
        self.log(f"CSV BẮT ĐẦU: {recorder.path}")
        # Firmware receives the same MAP ID as the CSV recorder in one START command.
        if self.send_command(f"AUTO:{self.selected_map}"):
            self.start_pending = True
            self.control_state.set("Đã gửi AUTO — chờ ESP32 xác nhận.")
            self.update_control_buttons()
        else:
            self.finish_run_csv("KHÔNG GỬI ĐƯỢC AUTO")

    def begin_external_run(self, origin_message):
        """Start MAP-specific CSV/TXT when GPIO34 starts AUTO on the ESP32.

        GUI START already opens its recorder before sending AUTO; in that
        case the existing run owns the log and must not be replaced.
        """
        if self.run_csv.active is not None:
            return
        match = re.search(r"\bmap=([123])\b", origin_message)
        if match is None:
            self.log("GPIO34 START: RUN_ORIGIN không có MAP hợp lệ, không tạo CSV.")
            return
        map_id = int(match.group(1))
        self.selected_map = map_id  # Actual firmware MAP always wins for labels.
        self.map_status.set(f"Đã chọn {map_id} (GPIO34)")
        for number, button in self.map_buttons.items():
            button.configure(bg=CYAN if number == map_id else GREY,
                             fg=BG if number == map_id else FG)
        if self.reader is None or not self.reader.is_alive():
            self.log("GPIO34 START: không có kết nối SPP, không thể ghi CSV.")
            return
        if not self.calibration_ok or any(v is None for v in self.cal_thresholds):
            self.log("GPIO34 START: chưa tải CAL_DUMP, không thể tạo CSV đủ ngưỡng.")
            self.send_command("CAL_DUMP")
            return
        try:
            recorder = self.run_csv.begin(
                self.reader, CSV_HEADER, (self.cal_min, self.cal_max),
                list(self.cal_thresholds), map_id)
        except (OSError, RuntimeError, ValueError) as exc:
            self.log(f"GPIO34 START: không thể mở CSV: {exc}")
            return
        try:
            self.run_txt_path = recorder.path.with_suffix(".txt")
            self.run_txt_file = self.run_txt_path.open(
                "w", encoding="utf-8", buffering=1)
        except OSError as exc:
            self.run_txt_file = None
            self.run_csv.finish(self.reader, "GPIO34 START: TXT OPEN FAILED")
            self.log(f"GPIO34 START: lỗi mở TXT: {exc}")
            return
        self.last_csv_total = 0
        self.stop_ack_deadline = None
        self.start_confirmed = True
        self.csv_status.set(f"CSV GHI: {recorder.path.name}")
        self.log(f"GPIO34 START: lưu dữ liệu MAP {map_id} tại {recorder.path}")
        self.update_control_buttons()

    def stop_line(self):
        # Require an ESP32 ACK even if the current CSV has already closed.
        if self.send_command("STOP"):
            self.stop_ack_deadline = time.monotonic() + 2.0
            self.stop_verify_pending = False
            self.stop_verify_deadline = None
            self.stop_verify_retries = 0
            self.start_pending = False
            self.control_state.set("Đã gửi STOP — chờ ESP32 xác nhận.")
            self.update_control_buttons()
        else:
            self.finish_run_csv("STOP KHÔNG GỬI ĐƯỢC / CHƯA XÁC NHẬN DỪNG")

    def confirm_stopped(self, reason):
        # Only explicit ESP32 stop/finish ACK or STATUS with zero PWM
        # is allowed to release START after a missing STOP acknowledgment.
        self.stop_ack_deadline = None
        self.stop_verify_pending = False
        self.stop_verify_deadline = None
        self.stop_verify_retries = 0
        self.start_pending = False
        self.set_auto_running(False)
        self.finish_run_csv("ESP32 ĐÃ XÁC NHẬN DỪNG: " + reason)
        self.start_confirmed = False
        if self.estop_latched is False:
            self.control_state.set("ESP32 đã xác nhận dừng — có thể START lượt mới.")
            self.control_state_label.configure(fg=GREEN)
        self.update_control_buttons()

    def handle_stop_timeout(self):
        # Freeze log, but keep START interlocked until ESP32 confirms stopped.
        self.stop_ack_deadline = None
        self.stop_verify_pending = True
        self.stop_verify_deadline = time.monotonic() + 1.5
        self.stop_verify_retries = 0
        self.finish_run_csv("STOP ACK QUÁ HẠN — CHƯA XÁC NHẬN ĐỘNG CƠ")
        self.control_state.set("Mất ACK STOP; đang hỏi STATUS để xác nhận động cơ.")
        self.control_state_label.configure(fg=AMBER)
        self.log("STOP timeout: đang hỏi STATUS, không cần đóng app.")
        self.send_command("STATUS")
        self.update_control_buttons()

    def start_calibration(self):
        if self.send_command("CAL_WHITE"):
            self.cal_state.set("Đang lấy TRẮNG trên ESP32; giữ 16 mắt cố định.")

    def stop_calibration(self):
        if self.send_command("CAL_BLACK"):
            self.cal_state.set("Đã gửi Lấy ĐEN; chờ ESP32 xác nhận.")

    def sample_csv_row(self, s):
        adc = list(s[1:17])
        if self.calibration_ok:
            cal = [normalize_adc(adc[i], self.cal_min[i], self.cal_max[i]) for i in range(16)]
            bits = [int(value >= BLACK_THRESHOLD_CAL) for value in cal]
            word = "".join(str(bit) for bit in bits)
        else:
            cal, bits, word = [""] * 16, [""] * 16, ""
        host_time = datetime.datetime.now().astimezone().isoformat(timespec="milliseconds")
        return [host_time, s[0], *adc, *cal, *bits, *s[17:27], word]

    def on_sample(self, s):
        self.hist.append(s)
        self.frames += 1
        row = self.sample_csv_row(s)
        self.recent_rows.append(row)
        # Independent CSV worker handles all received frames, even if UI drops.
        self.last_adc = list(s[1:17])
        t, left, right = s[0], s[17], s[18]
        if self.preview_pid_log.get() and s[26] == 1:
            if self.last_preview_ms is None or (t - self.last_preview_ms) % (2**32) >= 200:
                self.last_preview_ms = t
                self.log(f"PID t={t}ms adc={','.join(map(str, s[1:17]))}"
                         f" cal={','.join(str(v) for v in row[18:34])}"
                         f" logic={row[-1] or '?'} pwm=({s[19]},{s[20]})"
                         f" enc=({s[17]},{s[18]}) err={s[21]:.3f}"
                         f" p={s[22]:.3f} i={s[23]:.3f} d={s[24]:.3f}"
                         f" out={s[25]:.3f}")
        if self.prev and t > self.prev[0]:
            dt = (t - self.prev[0]) / 1000.0
            v_left = (left - self.prev[1]) / dt
            v_right = (right - self.prev[2]) / dt
            self.wheel_speed[0] = 0.8 * self.wheel_speed[0] + 0.2 * v_left
            self.wheel_speed[1] = 0.8 * self.wheel_speed[1] + 0.2 * v_right
        self.prev = (t, left, right)

    def on_text(self, message):
        # Hardware START has no GUI callback, so RUN_ORIGIN opens its log.
        # Do this before mirroring RUN_ORIGIN to TXT.
        if message.startswith("RUN_ORIGIN "):
            self.begin_external_run(message)
        self.log(f"RX < {message}")
        if message.startswith("MAP_SPEED_PROFILE_V2:"):
            try:
                fields = message.split(":", 1)[1].split(",")
                map_id, count = int(fields[0]), int(fields[1])
                if map_id not in (1, 2, 3) or not 1 <= count <= 8 or len(fields) != 2 + 2*count:
                    raise ValueError("profile length or map/count invalid")
                ticks, bases = [], []
                for i in range(count):
                    end_tick, base = int(fields[2+2*i]), int(fields[3+2*i])
                    if i == count-1:
                        if end_tick != 0:
                            raise ValueError("last stage must end at FINISH")
                    else:
                        ticks.append(end_tick)
                    if base < 45 or base > 255:
                        raise ValueError("BASE outside 45..255")
                    bases.append(base)
                if any(t <= 0 or t > 10000000 for t in ticks) or any(
                        ticks[i] >= ticks[i+1] for i in range(len(ticks)-1)):
                    raise ValueError("ticks are not strictly increasing")
                actual = (map_id, ticks, bases)
                pending = self.map_profile_pending
                ok = pending is None or pending[0] != map_id or pending == actual
                self.map_speed_profiles[map_id] = {"ticks": list(ticks), "bases": list(bases)}
                self.map_profile_confirmed[map_id] = ok
                if pending is not None and pending[0] == map_id:
                    self.map_profile_pending = None
                self.map_profile_status.set(
                    f"MAP {map_id} BASE: " + ("ESP32 OK" if ok else "ACK không khớp"))
                if self.map_profile_editor_refresh is not None:
                    self.map_profile_editor_refresh(map_id)
            except (ValueError, IndexError, TypeError, tk.TclError):
                self.map_profile_status.set("MAP BASE: phản hồi ESP32 V2 không hợp lệ")
        map_profile_match = re.fullmatch(
            r"MAP_SPEED_PROFILE MAP=(\d+) X1=(\d+) X2=(\d+) X3=(\d+) "
            r"X4=(\d+) X5=(\d+) X6=(\d+) X7=(\d+) "
            r"B1=(\d+) B2=(\d+) B3=(\d+) B4=(\d+) "
            r"B5=(\d+) B6=(\d+) B7=(\d+) B8=(\d+)", message)
        if map_profile_match:
            map_id = None
            try:
                values = [int(value) for value in map_profile_match.groups()]
                map_id = values[0]
                if map_id not in (1, 2, 3):
                    raise ValueError("MAP ID ngoài phạm vi")
                ticks, bases = validate_map1_speed_profile(values[1:8], values[8:16])
                actual = (map_id, ticks, bases)
                pending = self.map_profile_pending
                if pending is not None and pending[0] == map_id:
                    self.map_profile_pending = None
                    if actual != pending:
                        self.map_profile_confirmed[map_id] = False
                        self.map_profile_status.set(
                            f"MAP {map_id} BASE: ESP32 trả về khác cấu hình vừa gửi")
                    else:
                        self.map_profile_confirmed[map_id] = True
                        self.map_profile_status.set(
                            f"MAP {map_id} BASE: ESP32 OK | " +
                            " / ".join(str(value) for value in bases))
                else:
                    # An unrelated map readback must not clear another map's pending write.
                    self.map_profile_confirmed[map_id] = True
                    self.map_profile_status.set(
                        f"MAP {map_id} BASE: ESP32 OK | " +
                        " / ".join(str(value) for value in bases))
                self.map_speed_profiles[map_id] = {
                    "ticks": list(ticks), "bases": list(bases)}
                if (self.map_profile_window is not None
                        and self.map_profile_window.winfo_exists()
                        and self.map_profile_map_id == map_id):
                    if not self.map_profile_dirty:
                        for var, value in zip(self.map_profile_tick_vars, ticks):
                            var.set(str(value))
                        for var, value in zip(self.map_profile_base_vars, bases):
                            var.set(str(value))
                        self.map_profile_dirty = False
                    for i, label in enumerate(self.map_profile_stage_labels):
                        start_tick = 0 if i == 0 else int(
                            self.map_profile_tick_vars[i - 1].get().strip())
                        label.configure(text=f"{start_tick:,}")
            except (ValueError, TypeError, tk.TclError, IndexError):
                if map_id in (1, 2, 3):
                    self.map_profile_confirmed[map_id] = False
                self.map_profile_status.set("MAP BASE: phản hồi ESP32 không hợp lệ")
        elif message.startswith("ERR: MAP_SPEED_PROFILE_"):
            self.map_profile_pending = None
            self.map_profile_status.set("MAP BASE: ESP32 từ chối cấu hình — xem Terminal")
        elif (message.startswith("MAP1_SPEED_PROFILE ")
              or message.startswith("MAP1_BASE_PROFILE ")):
            try:
                ticks = [
                    int(re.search(rf"\bX{i}=(\d+)\b", message).group(1))
                    for i in range(1, 8)
                ]
                bases = [
                    int(re.search(rf"\bB{i}=(\d+)\b", message).group(1))
                    for i in range(1, 9)
                ]
                ticks, bases = validate_map1_speed_profile(ticks, bases)
                self.map_speed_profiles[1] = {
                    "ticks": list(ticks), "bases": list(bases)}
                self.map_profile_confirmed[1] = True
                self.map_profile_status.set(
                    "MAP 1 BASE (legacy): ESP32 OK | " +
                    " / ".join(str(value) for value in bases))
            except (AttributeError, ValueError, TypeError):
                self.map_profile_confirmed[1] = False
                self.map_profile_status.set("MAP 1 BASE: phản hồi ESP32 không hợp lệ")
        elif (message.startswith("ERR: MAP1_SPEED_PROFILE_")
              or message.startswith("ERR: MAP1_BASE_PROFILE_")):
            self.map_profile_pending = None
            self.map_profile_confirmed[1] = False
            self.map_profile_status.set("MAP 1 BASE: ESP32 từ chối — xem Terminal")
        if (message.startswith("MAP_SPEED_PROFILE_V2:") or
                message.startswith("MAP_SPEED_PROFILE ") or
                message.startswith("ERR: MAP_SPEED_PROFILE_")):
            self.update_control_buttons()
        if message.startswith("MAP_SENSOR_PROFILE_V2:"):
            try:
                parts = message.split(":", 1)[1].split(",")
                target, count = int(parts[0]), int(parts[1])
                if target == 2:
                    if not (1 <= count <= 8) or len(parts) != 2 + 2 * count:
                        raise ValueError("Invalid stage count or token count")
                    ends = [int(parts[2 + 2 * i]) for i in range(count)]
                    masks = [int(parts[3 + 2 * i], 16) for i in range(count)]
                    if ends[-1] != 0 or any(mask == 0 for mask in masks):
                        raise ValueError("Invalid finish/mask")
                    ticks = ends[:-1]
                    if any(t <= 0 or t > 10000000 or
                           (i > 0 and t <= ticks[i - 1])
                           for i, t in enumerate(ticks)):
                        raise ValueError("Invalid tick thresholds")
                    self.map2_sensor_profile = (list(ticks), list(masks))
                    pending = self.map2_sensor_pending
                    matches = (pending is None or
                               (pending[0] == ticks and pending[1] == masks))
                    self.map2_sensor_pending = None
                    self.map2_sensor_confirmed = matches
                    if matches:
                        self.map2_sensor_status.set(
                            f"MAP 2 mắt PID: {count} GĐ | "
                            f"Tick={ticks} | ESP32 OK")
                    else:
                        self.map2_sensor_status.set(
                            "MAP 2 mắt PID: phản hồi ESP32 KHÁC cấu hình gửi")
                    if self.map2_sensor_editor_refresh is not None:
                        try:
                            self.map2_sensor_editor_refresh()
                        except tk.TclError:
                            self.map2_sensor_editor_refresh = None
            except (ValueError, IndexError) as exc:
                self.map2_sensor_confirmed = False
                self.map2_sensor_pending = None
                self.map2_sensor_status.set(
                    "MAP 2 mắt PID: dữ liệu trả về không hợp lệ")
                self.log(f"MAP 2 sensor profile readback error: {exc}")
        elif message.startswith("ERR: MAP_SENSOR_PROFILE_V2_"):
            self.map2_sensor_confirmed = False
            self.map2_sensor_pending = None
            self.map2_sensor_status.set(
                "MAP 2 mắt PID: ESP32 từ chối — xem Terminal")
        m2 = re.fullmatch(
            r"MAP2_SENSOR_PROFILE T1=(\d+) T2=(\d+) T3=(\d+) "
            r"M1=([0-9A-Fa-f]{4}) M2=([0-9A-Fa-f]{4}) "
            r"M3=([0-9A-Fa-f]{4}) M4=([0-9A-Fa-f]{4}) STAGE=(\d+)",
            message)
        if m2:
            ticks = tuple(int(m2.group(i)) for i in (1,2,3))
            masks = tuple(int(m2.group(i),16) for i in (4,5,6,7))
            self.map2_sensor_profile = (list(ticks), list(masks))
            self.map2_sensor_confirmed = True
            self.map2_sensor_status.set(
                f"MAP 2 mắt PID: T={ticks} | stage={m2.group(8)} | "
                + "/".join(f"{m:04X}" for m in masks)+" (ESP32 OK)")
            if self.map2_sensor_window is not None:
                try:
                    self.map2_sensor_window.title("MAP 2 — ESP32 đã xác nhận; mở lại để cập nhật")
                except tk.TclError:
                    self.map2_sensor_window = None
        elif message.startswith("ERR: MAP2_SENSOR_PROFILE_"):
            self.map2_sensor_confirmed=False
            self.map2_sensor_status.set("MAP 2 mắt PID: ESP32 từ chối — xem Terminal")
        m3 = re.fullmatch(
            r"MAP3_SENSOR_PROFILE T1=(\d+) T2=(\d+) T3=(\d+) "
            r"M1=([0-9A-Fa-f]{4}) M2=([0-9A-Fa-f]{4}) "
            r"M3=([0-9A-Fa-f]{4}) M4=([0-9A-Fa-f]{4}) STAGE=(\d+)",
            message)
        if m3:
            ticks = tuple(int(m3.group(i)) for i in (1, 2, 3))
            masks = tuple(int(m3.group(i), 16) for i in (4, 5, 6, 7))
            self.map3_sensor_profile = ticks + masks
            self.map3_sensor_confirmed = True
            self.map3_sensor_status.set(
                f"MAP 3 mắt PID: T={ticks} | stage={m3.group(8)} | " +
                "/".join(f"{m:04X}" for m in masks) + " (ESP32 OK)")
            if self.map3_sensor_window is not None:
                try:
                    self.map3_sensor_window.title(
                        "MAP 3 — ESP32 đã xác nhận; mở lại để cập nhật")
                except tk.TclError:
                    self.map3_sensor_window = None
        elif message.startswith("ERR: MAP3_SENSOR_PROFILE_"):
            self.map3_sensor_confirmed = False
            self.map3_sensor_status.set(
                "MAP 3 mắt PID: ESP32 từ chối — xem Terminal")
        if message.startswith("MAP3_STAGE "):
            match = re.search(r"stage=(\d+) runEnc=(-?\d+)", message)
            if match:
                stage, tick = map(int, match.groups())
                self.map3_sensor_status.set(
                    f"MAP 3: GĐ {stage}/4 | ENC={tick}")
        if message.startswith("MAP2_STAGE "):
            match = re.search(r"stage=(\d+) hold=(\d+) runEnc=(-?\d+)", message)
            if match:
                stage, hold, tick = map(int, match.groups())
                names = {0: "RUN", 1: "CĂN LINE",
                         2: "GIỮ LINE 10ms", 3: "LỖI CHECKPOINT"}
                self.map2_sensor_status.set(
                    f"MAP 2: GĐ {stage}/{len(self.map2_sensor_profile[1])} | "
                    f"{names.get(hold, 'UNKNOWN')} "
                    f"| ENC={tick}")
        sensor_profile_match = re.fullmatch(
            r"SENSOR_PROFILE T1=(\d+) T2=(\d+) M1=([0-9A-Fa-f]{4}) "
            r"M2=([0-9A-Fa-f]{4}) M3=([0-9A-Fa-f]{4})", message)
        if sensor_profile_match:
            a, b = (int(sensor_profile_match.group(i)) for i in (1, 2))
            masks = tuple(int(sensor_profile_match.group(i), 16)
                          for i in (3, 4, 5))
            self.sensor_profile = (a, b, *masks)
            self.sensor_profile_confirmed = True
            self.sensor_profile_status.set(
                f"PID mắt: {a}/{b} tick | "
                + " / ".join(f"{m:04X}" for m in masks) + " (ESP32 OK)")
            if self.sensor_profile_window is not None:
                try:
                    self.sensor_profile_window.title(
                        "MAP 1 — đã nhận cấu hình từ ESP32 (đóng/mở lại để tải)")
                except tk.TclError:
                    self.sensor_profile_window = None
        elif message == "ERR: SENSOR_PROFILE_INVALID" or message == "ERR: SENSOR_PROFILE_REQUIRES_STOPPED":
            self.sensor_profile_status.set("PID mắt: ESP32 từ chối thay đổi — xem Terminal")
        if message == "ESTOP: LATCHED" or message == "ERR: ESTOP_LATCHED":
            self.stop_ack_deadline = None
            self.stop_verify_pending = False
            self.stop_verify_deadline = None
            self.stop_verify_retries = 0
            self.estop_latched = True
            self.estop_pending = False
            self.start_pending = False
            self.set_auto_running(False)
            self.control_state.set("Firmware báo E-STOP đã chốt; có thể RESET_ESTOP qua terminal.")
            self.control_state_label.configure(fg=ROSE)
            self.finish_run_csv("ESTOP ĐÃ XÁC NHẬN")
            self.update_control_buttons()
        elif message == "ESTOP: CLEARED (MOTORS OFF)":
            self.estop_latched = False
            self.estop_pending = False
            self.start_pending = False
            self.set_auto_running(False)
            self.control_state.set("Đã mở khóa E-STOP — động cơ vẫn dừng.")
            self.control_state_label.configure(fg=GREEN)
            self.update_control_buttons()
        elif message == "ENC_RESET: OK":
            # Only the ESP32 acknowledgement confirms that counts were reset.
            self.encoder_reset_pending = False
            self.encoder_reset_pending_at = None
            self.prev = None
            self.wheel_speed = [0.0, 0.0]
            for key in ("encL", "encR", "spdL", "spdR"):
                if key in self.values:
                    self.values[key].set("0")
            self.update_control_buttons()
            self.log("Đã reset encoder trên ESP32 (ACK).")
            self.send_command("ENC")  # Verify actual L/R counters after ACK.
        elif message == "ERR: ENC_RESET_REQUIRES_STOPPED":
            self.encoder_reset_pending = False
            self.encoder_reset_pending_at = None
            self.update_control_buttons()
            self.log("ESP32 từ chối reset encoder: xe vẫn đang chạy.")
        elif message == "MODE: AUTO PID":
            self.start_pending = False
            self.start_confirmed = True
            self.set_auto_running(True)
            self.control_state.set("AUTO ĐANG CHẠY — đi thẳng tìm line rồi chuyển PID.")
            self.control_state_label.configure(fg=GREEN)
            self.update_control_buttons()
        elif message in ("ERR: MAP_NOT_SELECTED", "ERR: MAP_USE_1_2_3",
                              "ERR: MAP_SELECT_FAILED", "ERR: AUTO_ALREADY_RUNNING"):
            if self.start_pending and not self.start_confirmed:
                self.start_pending = False
                self.set_auto_running(False)
                self.control_state.set("Firmware từ chối START/MAP: " + message)
                self.control_state_label.configure(fg=AMBER)
                self.finish_run_csv("AUTO BỊ TỪ CHỐI: " + message)
                self.update_control_buttons()
        elif message.startswith("ERR: CALIB_REQUIRED_OR_PENDING"):
            self.start_pending = False
            self.set_auto_running(False)
            self.control_state.set("Không thể bắt đầu: cần hoàn tất calibration TRẮNG/ĐEN.")
            self.control_state_label.configure(fg=AMBER)
            self.finish_run_csv("AUTO BỊ TỪ CHỐI: CALIBRATION")
            self.update_control_buttons()
        elif message in ("ERR: ADC_NOT_READY", "ERR: ADC_BENCH_ACTIVE"):
            if self.start_pending and not self.start_confirmed:
                self.start_pending = False
                self.set_auto_running(False)
                self.finish_run_csv("AUTO BỊ TỪ CHỐI: " + message)
                self.update_control_buttons()
        elif message.startswith("ROUTE_LOCK: ENABLED="):
            enabled = re.search(r"\bENABLED=(\d)\b", message)
            route_map = re.search(r"\bMAP=(\d)\b", message)
            if enabled:
                map_label = route_map.group(1) if route_map else "?"
                status = "BẬT" if enabled.group(1) == "1" else "TẮT"
                self.route_status.set(
                    f"ROUTE: khóa={status} | Map hiện tại={map_label} | "
                    "thẳng → nửa vòng → phải → thẳng")
        elif message.startswith("ROUTE_STAGE "):
            stage = re.search(r"ROUTE_STAGE (\w+)", message)
            tick = re.search(r"\brunEnc=(-?\d+)\b", message)
            if stage:
                self.route_status.set(
                    f"ROUTE: {stage.group(1)} | ENC="
                    f"{tick.group(1) if tick else '--'} | MAP 01")
        elif message.startswith("ROUTE_FAULT "):
            code = re.search(r"\bcode=(\d+)\b", message)
            reasons = {
                1: "SEARCH cua nửa vòng vượt 250 ms",
                2: "Thiếu tín hiệu xác nhận cung nửa vòng",
                3: "Chưa xác nhận được đoạn thẳng sau rẽ phải",
            }
            fault_code = int(code.group(1)) if code else -1
            info = reasons.get(fault_code, "Lỗi lộ trình không xác định")
            self.route_status.set(f"ROUTE: FAULT #{fault_code} — {info}")
            self.control_state.set(f"ROUTE FAULT — {info}")
            self.control_state_label.configure(fg=ROSE)
        elif message.startswith("ERR: ROUTE_LOCK_"):
            self.route_status.set(f"ROUTE: ESP32 từ chối — {message}")
        elif message.startswith("AUTO_STOPPED: ROUTE_FAULT"):
            self.confirm_stopped(message)
            self.control_state.set("ROUTE FAULT — xe dừng an toàn; xem ROUTE ở trên.")
            self.control_state_label.configure(fg=ROSE)
        elif message.startswith("HS_PEAK: PWM="):
            peak_match = re.search(r"\bPWM=(\d+)\b", message)
            if peak_match:
                confirmed = int(peak_match.group(1))
                if 100 <= confirmed <= 255:
                    self.high_speed_peak = confirmed
                    # MAP 1 uses START_ENC; other maps use legacy BRAKE_START.
                    late_match = re.search(r"\bSTART_ENC=(\d+)\b", message)
                    self.hs_late_mode = late_match is not None
                    if late_match:
                        self.hs_late_start = int(late_match.group(1))
                    start_match = re.search(r"\bBRAKE_START=(\d+)\b", message)
                    if start_match:
                        self.hs_brake_start = int(start_match.group(1))
                    self.hs_peak_confirmed = confirmed
                    expected = self.hs_peak_pending
                    if expected is None or expected == confirmed:
                        self.hs_peak_pending = None
                        self.hs_peak_pending_at = None
                        # Explicit firmware readbacks supersede local edits
                        # only when the user clicked Read or Apply.
                        if expected is not None or not self.hs_peak_dirty:
                            self.hs_peak_var.set(confirmed)
                            self.hs_peak_dirty = False
                            self.hs_peak_value_text.set(f"{confirmed} / 255")
                        self.hs_peak_feedback.set(
                            f"ESP32 xác nhận HS_PEAK = {confirmed} PWM")
                    else:
                        self.hs_peak_feedback.set(
                            f"ESP32 báo {confirmed}; chờ xác nhận PWM {expected}")
                    if self.hs_late_mode:
                        self.high_speed_status.set(
                            f"HS: READY | MAP 1 ENC≥{self.hs_late_start} | BASE đích={confirmed}")
                    else:
                        self.high_speed_status.set(
                            f"HS: READY | PEAK={confirmed} | BRAKE={self.hs_brake_start}-2300 | HOLD=110")
                    self.update_control_buttons()
        elif message.startswith("ERR: HS_PEAK_"):
            if self.hs_peak_pending is not None:
                self.hs_peak_pending = None
                self.hs_peak_pending_at = None
                self.hs_peak_feedback.set(f"ESP32 từ chối: {message}")
                self.update_control_buttons()
                self.send_command("HS_PEAK")
        elif message == "ERR: BASE_GT_HS_PEAK":
            self.hs_peak_feedback.set(
                "BASE vượt HS_PEAK; tăng HS_PEAK trước khi đổi BASE")
            self.send_command("PID")
        elif message.startswith("HS_STAGE "):
            peak = re.search(r"\bpeak=(\d+)\b", message)
            if peak:
                self.high_speed_peak = int(peak.group(1))
            start = re.search(r"\bbrake=(\d+)\b", message)
            if start:
                self.hs_brake_start = int(start.group(1))
            late_start = re.search(r"\bstart=(\d+)\b", message)
            if late_start:
                self.hs_late_start = int(late_start.group(1))
                self.hs_late_mode = True
            stage = re.search(r"HS_STAGE (\w+)", message)
            tick = re.search(r"\brunEnc=(-?\d+)", message)
            if stage:
                if late_start or stage.group(1) in ("WAIT_LINE", "BOOST"):
                    self.high_speed_status.set(
                        f"HS: {stage.group(1)} | ENC={tick.group(1) if tick else '--'} "
                        f"| tăng BASE sau {self.hs_late_start} | PEAK={self.high_speed_peak}"
                    )
                else:
                    self.high_speed_status.set(
                        f"HS: {stage.group(1)} | ENC={tick.group(1) if tick else '--'} "
                        f"| PEAK={self.high_speed_peak} | BRAKE={self.hs_brake_start}-2300 | HOLD=110"
                    )
        elif message.startswith("DIAG:"):
            route_stage = re.search(r"\broute=(\d+)\b", message)
            route_fault = re.search(r"\brf=(\d+)\b", message)
            route_tick = re.search(r"\brunEnc=(-?\d+)\b", message)
            if route_stage and int(route_stage.group(1)) in (1, 2, 3, 4):
                names = ("OFF", "STRAIGHT_1", "SEMICIRCLE",
                         "RIGHT_TURN", "STRAIGHT_2", "FAULT")
                self.route_status.set(
                    f"ROUTE: {names[int(route_stage.group(1))]} | "
                    f"ENC={route_tick.group(1) if route_tick else '--'}")
            if route_fault and int(route_fault.group(1)) != 0:
                self.route_status.set(
                    f"ROUTE: FAULT #{route_fault.group(1)} | xem log TXT")
            peak = re.search(r"\bpeak=(\d+)\b", message)
            if peak:
                value = int(peak.group(1))
                if 100 <= value <= 255:
                    self.high_speed_peak = value
                    if not getattr(self, "hs_late_mode", False):
                        self.hs_brake_start = 1400 if value >= 240 else 1600 if value >= 230 else 1800
                    # DIAG is telemetry, not an ACK for a pending HS write.
                    if self.hs_peak_pending is None:
                        self.hs_peak_confirmed = value
                        if not self.hs_peak_dirty:
                            self.hs_peak_var.set(value)
                            self.hs_peak_value_text.set(f"{value} / 255")
            hs = re.search(r"\bhs=(\d+)\b", message)
            tick = re.search(r"\brunEnc=(-?\d+)\b", message)
            if hs:
                names = ("OFF", "RAMP", "CRUISE", "BRAKE", "COMPLETE",
                         "ABORTED", "HOLD", "WAIT_LINE", "BOOST")
                state = int(hs.group(1))
                title = names[state] if 0 <= state < len(names) else "UNKNOWN"
                if state in (7, 8) or (
                        getattr(self, "hs_late_mode", False)
                        and getattr(self, "selected_map", None) == 1):
                    self.high_speed_status.set(
                        f"HS: {title} | ENC={tick.group(1) if tick else '--'} "
                        f"| 10k+ BASE→{self.high_speed_peak}"
                    )
                else:
                    self.high_speed_status.set(
                        f"HS: {title} | ENC={tick.group(1) if tick else '--'} "
                        f"| PEAK={self.high_speed_peak} | BRAKE={self.hs_brake_start}-2300 | HOLD=110"
                    )
            loop = re.search(r"\bloop=(\d+(?:\.\d+)?)Hz\b", message)
            pid = re.search(r"\bpid=(\d+(?:\.\d+)?)Hz\b", message)
            sent = re.search(r"\bsent=(\d+(?:\.\d+)?)Hz\b", message)
            dropped = re.search(r"\bdrop=(\d+)\b", message)
            if loop and pid and sent:
                self.last_diag_rx = time.monotonic()
                warning = f"    TX drop: {dropped.group(1)}" if dropped else ""
                self.loop_debug.set(
                    f"LOOP: {float(loop.group(1)):.1f} Hz    "
                    f"PID: {float(pid.group(1)):.1f} Hz    "
                    f"SPP TX: {float(sent.group(1)):.1f} Hz{warning}")
        elif message.startswith("STATUS:"):
            estop = re.search(r"\bestop=(\d)\b", message)
            auto = re.search(r"\bauto=(\d)\b", message)
            pwm_l = re.search(r"\bpwmL=(-?\d+)\b", message)
            pwm_r = re.search(r"\bpwmR=(-?\d+)\b", message)
            is_stopped = (auto is not None and auto.group(1) == "0"
                          and pwm_l is not None and pwm_r is not None
                          and int(pwm_l.group(1)) == 0
                          and int(pwm_r.group(1)) == 0)
            if is_stopped and (self.stop_ack_deadline is not None
                               or self.stop_verify_pending
                               or self.start_confirmed or self.auto_running):
                self.confirm_stopped("STATUS auto=0 pwmL=0 pwmR=0")
            adc_mode = re.search(r"\badc=([A-Z0-9_]+)\b", message)
            adc_ok = re.search(r"\badc_ok=(\d)\b", message)
            if adc_mode:
                ready = bool(adc_ok and adc_ok.group(1) == "1")
                self.adc_status.set(f"ADC: {adc_mode.group(1)} | {'READY' if ready else 'FAULT'}")
            if estop:
                self.estop_latched = estop.group(1) == "1"
                self.estop_pending = False
                self.start_pending = False
                self.set_auto_running(bool(auto and auto.group(1) == "1"))
                if self.estop_latched:
                    self.set_auto_running(False)
                    self.control_state.set("E-STOP đang khóa — gửi RESET_ESTOP trong terminal để mở khóa.")
                    self.control_state_label.configure(fg=ROSE)
                    self.finish_run_csv("STATUS: ESTOP ĐANG KHÓA")
                elif self.start_confirmed and not self.auto_running:
                    self.finish_run_csv("STATUS: AUTO ĐÃ DỪNG")
                elif self.auto_running:
                    self.control_state.set("ĐANG DÒ LINE — PID AUTO đang chạy.")
                    self.control_state_label.configure(fg=GREEN)
                else:
                    self.control_state.set("SẴN SÀNG — động cơ đang dừng.")
                    self.control_state_label.configure(fg=GREEN)
                self.update_control_buttons()
        elif (message in ("MOTOR: STOP", "MODE: MANUAL",
                              "[SAFE] MANUAL DEADMAN: MOTOR STOP",
                              "ERR: ADC_READ_FAULT_MOTORS_STOPPED")
              or message.startswith("AUTO_FINISHED:")
              or message.startswith("AUTO_STOPPED:")):
            self.confirm_stopped(message)
        pid = re.search(r"PID:\s*Kp=([\d.]+)\s+Ki=([\d.]+)\s+Kd=([\d.]+)\s+Base=(\d+)",
                        message, flags=re.IGNORECASE)
        if pid:
            for key, value in zip(("Kp", "Ki", "Kd", "Base"), pid.groups()):
                self.pid_vars[key].set(value)
            self.pwm_var.set(int(pid.group(4)))
            self.preview_pwm()
        if message == "CAL_DUMP_BEGIN":
            self.cal_data = {}
        elif message.startswith("CAL_DATA "):
            parts = message.split()
            if len(parts) == 5:
                try:
                    eye, white, black, threshold = map(int, parts[1:])
                except ValueError:
                    return
                if (0 <= eye < 16 and 0 <= white <= 4095 and
                        0 <= black <= 4095 and black - white >= MIN_CALIB_DELTA and
                        threshold == eye_raw_threshold(white, black)):
                    self.cal_data[eye] = (white, black, threshold)
        elif message == "CAL_DUMP_DONE":
            if len(self.cal_data) == 16:
                self.cal_min = [self.cal_data[i][0] for i in range(16)]
                self.cal_max = [self.cal_data[i][1] for i in range(16)]
                self.cal_thresholds = [self.cal_data[i][2] for i in range(16)]
                self.calibration_ok = True
                self.update_control_buttons()
                # Update the current START/STOP CSV with newly downloaded NVS data.
                if self.run_csv.active is not None:
                    self.run_csv.active.set_calibration(self.cal_min, self.cal_max)
                self.cal_state.set("Đã tải NVS 16/16 mắt; dữ liệu CAL 0–1000 theo ESP32.")
                self.draw_bars(self.last_adc)
            else:
                self.cal_state.set(f"Chưa nhận đủ NVS: {len(self.cal_data)}/16 mắt.")
        elif message == "CAL_WHITE_DONE":
            self.cal_state.set("Đã lấy TRẮNG. Đặt nền ĐEN rồi bấm 2. Lấy ĐEN.")
        elif message == "CAL_WHITE_STARTED":
            self.cal_state.set("Đang thu 128 khung TRẮNG.")
        elif message == "CAL_BLACK_STARTED":
            self.cal_state.set("Đang thu 128 khung ĐEN.")
        elif message == "CAL_OK_SAVED":
            self.cal_state.set("ESP32 đã lưu NVS; đang đọc lại profile 16 mắt.")
            self.calibration_ok = False
            self.cal_thresholds = [None] * 16
            self.cal_data = {}
            self.update_control_buttons()
            self.send_command("CAL_DUMP")
        elif message == "CAL_FAILED":
            self.cal_state.set("Calib mới thất bại; giữ profile cũ nếu có.")
        elif message.startswith("CAL_STATUS"):
            self.cal_state.set(message)
        elif message == "CAL_NEED_WHITE":
            self.cal_state.set("Chưa lấy TRẮNG; bấm 1. Lấy TRẮNG trước.")
        elif message == "CAL_NO_PROFILE":
            self.cal_state.set("ESP32 chưa có profile NVS hợp lệ.")
            self.calibration_ok = False
            self.cal_thresholds = [None] * 16
            self.update_control_buttons()
        elif message.startswith("STREAM Hz="):
            hz = re.search(r"STREAM Hz=(\d+)", message)
            if hz:
                self.stream_rate.set(hz.group(1))

    def tick(self):
        last = None
        # Process safety-critical Bluetooth replies before bulk telemetry.
        try:
            for _ in range(64):
                kind, payload = self.controls_q.get_nowait()
                if kind == "text":
                    self.on_text(payload)
                elif kind == "error":
                    self.disconnect()
                    self.status_label.config(text=payload, fg=ROSE)
                elif kind == "csv_export":
                    self.log(payload)
                elif kind == "csv_closed":
                    self.csv_status.set(payload)
                    self.log(payload)
        except queue.Empty:
            pass
        try:
            for _ in range(512):
                kind, payload = self.q.get_nowait()
                if kind == "data":
                    self.on_sample(payload)
                    last = payload
                elif kind == "csv_export":
                    self.log(payload)
        except queue.Empty:
            pass
        if last is not None:
            self.update_view(last)
        now = time.monotonic()
        self.update_run_timer_display(now)
        if (self.hs_peak_pending is not None and self.hs_peak_pending_at is not None
                and now - self.hs_peak_pending_at >= 3.0):
            expected = self.hs_peak_pending
            self.hs_peak_pending = None
            self.hs_peak_pending_at = None
            self.hs_peak_confirmed = None
            self.hs_peak_feedback.set(
                f"Chưa nhận ACK cho HS_PEAK={expected}; đang đọc lại ESP32.")
            self.log(f"HS_PEAK={expected}: timeout xác nhận, chưa coi là đã áp dụng.")
            self.update_control_buttons()
            self.send_command("HS_PEAK")
        if (self.encoder_reset_pending and self.encoder_reset_pending_at is not None
                and now - self.encoder_reset_pending_at >= 3.0):
            self.encoder_reset_pending = False
            self.encoder_reset_pending_at = None
            self.log("Reset ENC: chưa nhận ACK từ ESP32, kiểm tra lại bằng lệnh ENC.")
            self.update_control_buttons()
            self.send_command("ENC")
        if now - self.rate_time >= 1:
            interval = now - self.rate_time
            total = self.reader.frames_rx if self.reader else self.last_rx_total
            rx_hz = (total - self.last_rx_total) / interval
            app_hz = self.frames / interval
            errors = self.reader.bad if self.reader else 0
            drops = self.reader.frames_dropped if self.reader else 0
            self.rate_label.config(
                text=f"RX {rx_hz:.0f} Hz | APP {app_hz:.0f} Hz | "
                     f"q={self.q.qsize()} dropUI={drops} CRC={errors}")
            if self.run_csv.active is not None:
                rec = self.run_csv.active
                csv_hz = (rec.written - self.last_csv_total) / interval
                self.csv_status.set(
                    f"CSV GHI: {csv_hz:.0f} Hz | tổng={rec.written} "
                    f"đợi={rec.queue.qsize()} dropCSV={rec.dropped}"
                    + (f" LỖI={rec.error}" if rec.error else "")
                    + f" | MAP {rec.map_id} | {rec.path.name}")
                self.last_csv_total = rec.written
            self.last_rx_total = total
            self.frames = 0
            self.rate_time = now
        # One small SPP text request per 2 seconds; no USB traffic.
        if self.next_diag_poll is not None and now >= self.next_diag_poll:
            if self.reader is not None and self.reader.is_alive():
                self.send_command("DIAG")
                self.next_diag_poll = now + self.diag_interval_s
            else:
                self.next_diag_poll = None
        if self.stop_ack_deadline is not None and now >= self.stop_ack_deadline:
            self.handle_stop_timeout()
        if (self.stop_verify_pending and self.stop_verify_deadline is not None
                and now >= self.stop_verify_deadline):
            if self.stop_verify_retries < 3:
                self.stop_verify_retries += 1
                self.stop_verify_deadline = now + 1.5
                self.send_command("STATUS")
            else:
                # No ACK: preserve the interlock; STOP/STATUS are still usable.
                self.stop_verify_deadline = None
                self.control_state.set(
                    "Chưa xác nhận STOP; gửi lại STOP hoặc STATUS, kiểm tra bánh xe.")
                self.control_state_label.configure(fg=ROSE)
        if self.last_diag_rx is not None and now - self.last_diag_rx > 6.0:
            self.loop_debug.set("LOOP: -- Hz    PID: -- Hz    SPP TX: -- Hz    (DIAG quá hạn)")
            self.last_diag_rx = None
        self.after(30, self.tick)

    def update_view(self, s):
        values = {"encL": str(s[17]), "encR": str(s[18]),
                  "spdL": f"{self.wheel_speed[0]:.0f}", "spdR": f"{self.wheel_speed[1]:.0f}",
                  "pwmL": str(s[19]), "pwmR": str(s[20]),
                  "err": f"{s[21]:.2f}", "p": f"{s[22]:.2f}",
                  "i": f"{s[23]:.2f}", "d": f"{s[24]:.2f}",
                  "out": f"{s[25]:.2f}", "mode": {0: "MANUAL", 1: "AUTO PID"}.get(s[26], str(s[26]))}
        for key, value in values.items():
            self.values[key].set(value)
        self.draw_bars(s[1:17])
        self.update_sensor_table()
        self.draw_scope()

    def draw_bars(self, adc_values):
        c = self.bars
        c.delete("all")
        width = max(420, c.winfo_width())
        top, height = 25, 154
        step = width / 16
        bits = []
        for i, adc in enumerate(adc_values):
            low, high = self.cal_min[i], self.cal_max[i]
            cal = normalize_adc(adc, low, high) if self.calibration_ok else None
            black = cal >= BLACK_THRESHOLD_CAL if cal is not None else None
            digit = ("1" if black else "0") if black is not None else "—"
            bits.append(digit)
            x0, x1 = i * step + 4, (i + 1) * step - 4
            bar_height = height * adc / ADC_MAX
            color = GREEN if black is True else GREY if black is False else MUTED
            c.create_rectangle(x0, top + height - bar_height, x1,
                               top + height, fill=color, outline="")
            if self.calibration_ok:
                threshold = eye_raw_threshold(low, high)
                yt = top + height * (1 - threshold / ADC_MAX)
                c.create_line(x0, yt, x1, yt, fill=AMBER, width=2)
            else:
                threshold = None
            c.create_text((x0 + x1) / 2, 12, text=str(adc), fill=FG,
                          font=("Consolas", 8))
            c.create_text((x0 + x1) / 2, 197,
                          text="—" if threshold is None else str(threshold),
                          fill=AMBER, font=("Consolas", 8))
            c.create_text((x0 + x1) / 2, 220,
                          text="—" if cal is None else str(cal),
                          fill=CYAN, font=("Consolas", 8))
            if digit != "—":
                c.create_rectangle(x0+4, 235, x1-4, 255, fill=GREEN if black else "#475569",
                                   outline="")
            c.create_text((x0 + x1) / 2, 245, text=digit, fill=FG,
                          font=("Consolas", 10, "bold"))
            c.create_text((x0 + x1) / 2, 273, text=str(i), fill=MUTED,
                          font=("Consolas", 9))
        if self.calibration_ok:
            self.logic_var.set("0 = TRẮNG | 1 = ĐEN   [" +
                               "".join(bits[:8]) + " | " + "".join(bits[8:]) +
                               "]   (ngưỡng riêng từng mắt, NVS)")
        else:
            self.logic_var.set("Chưa tải được profile NVS: logic chưa xác định (—)")

    def draw_scope(self):
        c = self.scope
        c.delete("all")
        width, height = max(420, c.winfo_width()), int(c["height"])
        mid = height / 2
        c.create_line(0, mid, width, mid, fill="#334155")
        for i, (name, color) in enumerate((("pwmL", CYAN), ("pwmR", AMBER), ("PID out", ROSE))):
            c.create_text(45 + i * 95, 10, text=name, fill=color, font=("Consolas", 9))
        if len(self.hist) < 2:
            return
        for idx, color in ((19, CYAN), (20, AMBER), (25, ROSE)):
            pts = []
            for k, row in enumerate(self.hist):
                x = k * width / (len(self.hist) - 1)
                y = mid - row[idx] / PWM_MAX * (mid - 6)
                pts.extend((x, max(2, min(height - 2, y))))
            c.create_line(*pts, fill=color)

    def on_close(self):
        self.disconnect()
        self.run_csv.close_all()
        if self.run_txt_file is not None:
            self.run_txt_file.close()
            self.run_txt_file = None
        self.session_log_file.close()
        self.destroy()


if __name__ == "__main__":
    App().mainloop()