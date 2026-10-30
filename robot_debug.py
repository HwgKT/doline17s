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
RUN_STOP_DEBOUNCE_SECONDS = 0.25


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
        self.ser = serial.Serial(port, 115200, timeout=0.05)
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
        self.ser.write((text + "\n").encode("utf-8"))


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
        self.start_confirmed = False
        self.csv_status = tk.StringVar(value="CSV: chờ START")
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
        self.run_timer_running = False
        self.run_timer_stationary_since = None
        self.run_timer_stop_latched = False
        self.run_timer_var = tk.StringVar(value="00:00:00")
        self._build()
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
        self.label(diag_bar, textvariable=self.loop_debug, fg=CYAN, bg=PANEL,
                   font=("Consolas", 9, "bold")).pack(anchor="w", padx=8, pady=(1, 0))
        self.label(diag_bar, textvariable=self.csv_status, fg=GREEN, bg=PANEL,
                   font=("Consolas", 8)).pack(anchor="w", padx=8, pady=(0, 1))

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
        right.grid_rowconfigure(0, weight=1, uniform="right_half", minsize=220)
        right.grid_rowconfigure(1, weight=1, uniform="right_half", minsize=220)

        # Upper-right quarter: compact fixed controls, no scrolling.
        # Lower-right quarter remains a full-height SPP terminal.
        panel = tk.Frame(right, bg=PANEL, padx=8, pady=5)
        panel.grid(row=0, column=0, sticky="nsew")

        terminal_panel = tk.Frame(right, bg=PANEL, padx=8, pady=5)
        terminal_panel.grid(row=1, column=0, sticky="nsew")

        control = tk.Frame(panel, bg=PANEL)
        control.pack(fill="x", pady=(0, 2))
        self.start_btn = tk.Button(control, text="BẮT ĐẦU DÒ LINE",
                                   command=self.start_line, font=("Segoe UI", 9, "bold"),
                                   bg=GREEN, fg="#ffffff", activebackground="#059669",
                                   relief="flat", height=1, pady=1)
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
                   font=("Consolas", 12, "bold")).pack(side="left", padx=8)
        ttk.Button(run_timer, text="Reset thời gian",
                   command=self.reset_run_timer).pack(side="right")
        self.control_state = tk.StringVar(value="Chưa kết nối / chưa xác định trạng thái")
        self.control_state_label = self.label(panel, textvariable=self.control_state,
                                              bg=PANEL, fg=AMBER, wraplength=430,
                                              font=("Segoe UI", 9),
                                              anchor="w", justify="left")
        self.control_state_label.pack(fill="x", pady=(0, 1))
        self.adc_status = tk.StringVar(value="ADC: chờ STATUS")
        self.label(panel, textvariable=self.adc_status, bg=PANEL, fg=CYAN,
                   font=("Consolas", 8)).pack(anchor="w", pady=(0, 2))
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
        encoder_actions = tk.Frame(left, bg=BG)
        encoder_actions.pack(fill="x", pady=5)
        ttk.Button(encoder_actions, text="Reset ENC",
                   command=lambda: self.send_command("ENC_RESET")).pack(side="left", padx=4)
        self.label(left, "RAW: 0–4095  |  T_i=W_i+40%×(B_i−W_i)  |  CAL: 0–1000  |  0=TRẮNG, 1=ĐEN",
                   fg=MUTED).pack(anchor="w")

        # Compact PWM controls: one header with numeric feedback and short rows.
        pwm_title = tk.Frame(panel, bg=PANEL)
        pwm_title.pack(fill="x", pady=(2, 0))
        self.label(pwm_title, "PWM (0–255)", bg=PANEL,
                   font=("Segoe UI", 9, "bold")).pack(side="left")
        self.pwm_var = tk.DoubleVar(value=120)
        self.pwm_text = tk.StringVar(value="120 / 255")
        self.label(pwm_title, textvariable=self.pwm_text, bg=PANEL, fg=CYAN,
                   font=("Consolas", 9)).pack(side="right")
        self.pwm_slider = ttk.Scale(panel, from_=0, to=255, variable=self.pwm_var,
                                    command=self.preview_pwm)
        self.pwm_slider.pack(fill="x", pady=(0, 1))
        self.pwm_slider.bind("<ButtonRelease-1>", lambda _e: self.apply_pwm())
        self.pwm_slider.bind("<KeyRelease>", lambda _e: self.apply_pwm())
        speed_buttons = tk.Frame(panel, bg=PANEL)
        speed_buttons.pack(fill="x", pady=(0, 2))
        for val in (60, 120, 180, 255):
            ttk.Button(speed_buttons, text=str(val), width=5,
                       command=lambda v=val: self.preset_pwm(v)).pack(
                           side="left", expand=True, padx=1)

        self.label(panel, "Kp / Ki / Kd / Base", bg=PANEL,
                   font=("Segoe UI", 9, "bold")).pack(anchor="w", pady=(1, 1))
        pid = tk.Frame(panel, bg=PANEL)
        pid.pack(fill="x")
        self.pid_vars = {}
        for idx, (key, initial) in enumerate((("Kp", "2.2"), ("Ki", "0"),
                                             ("Kd", "0"), ("Base", "120"))):
            self.label(pid, key, bg=PANEL, fg=MUTED,
                       font=("Segoe UI", 8)).grid(row=0, column=idx, sticky="w")
            var = tk.StringVar(value=initial)
            self.pid_vars[key] = var
            ttk.Entry(pid, textvariable=var, width=6).grid(
                row=1, column=idx, padx=(0, 2), sticky="ew")
            pid.grid_columnconfigure(idx, weight=1)
        pid_actions = tk.Frame(panel, bg=PANEL)
        pid_actions.pack(fill="x", pady=(2, 1))
        ttk.Button(pid_actions, text="Áp dụng PID", command=self.apply_pid).pack(
            side="left", fill="x", expand=True, padx=(0, 2))
        ttk.Button(pid_actions, text="Đọc PID", command=lambda: self.send_command("PID")).pack(
            side="left", fill="x", expand=True)

        self.label(panel, "Calibration — 16 mắt / NVS", bg=PANEL,
                   font=("Segoe UI", 9, "bold")).pack(anchor="w", pady=(2, 1))
        cal_buttons = tk.Frame(panel, bg=PANEL)
        cal_buttons.pack(fill="x")
        ttk.Button(cal_buttons, text="Lấy TRẮNG", command=self.start_calibration).pack(
            side="left", fill="x", expand=True, padx=(0, 2))
        ttk.Button(cal_buttons, text="Lấy ĐEN", command=self.stop_calibration).pack(
            side="left", fill="x", expand=True)
        more = tk.Frame(panel, bg=PANEL)
        more.pack(fill="x", pady=(2, 0))
        ttk.Button(more, text="CAL Status", command=lambda: self.send_command("CAL_STATUS")).pack(
            side="left", fill="x", expand=True, padx=(0, 2))
        ttk.Button(more, text="Đọc NVS 16 mắt", command=lambda: self.send_command("CAL_DUMP")).pack(
            side="left", fill="x", expand=True)
        self.cal_state = tk.StringVar(value="Chưa tải profile calibration từ ESP32")
        self.label(panel, textvariable=self.cal_state, bg=PANEL, fg=AMBER,
                   font=("Segoe UI", 8), justify="left", anchor="w",
                   wraplength=430).pack(fill="x", pady=(1, 0))

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
        path = filedialog.asksaveasfilename(defaultextension=".txt",
            initialfile=time.strftime("linerobot_terminal_%Y%m%d_%H%M%S.txt"),
            filetypes=[("Text log", "*.txt")])
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8") as f:
                self.session_log_file.flush()
                self.session_log_file.seek(0)
                try:
                    shutil.copyfileobj(self.session_log_file, f)
                finally:
                    self.session_log_file.seek(0, 2)
            self.log(f"Đã xuất terminal TXT: {path}")
        except OSError as exc:
            self.log(f"Không thể xuất TXT: {exc}")

    def export_csv(self):
        path = filedialog.asksaveasfilename(defaultextension=".csv",
            initialfile=time.strftime("linerobot_telemetry_%Y%m%d_%H%M%S.csv"),
            filetypes=[("CSV", "*.csv")])
        if not path:
            return
        try:
            if self.run_csv.last is None:
                self.log("Chưa có lượt chạy CSV nào để xuất")
                return
            selected = self.run_csv.last

            def export_job():
                try:
                    count = selected.export_snapshot(path)
                    message = f"Đã xuất CSV ({count} mẫu): {path}"
                except (OSError, ValueError) as exc:
                    message = f"Lỗi xuất CSV: {exc}"
                self.controls_q.put(("csv_export", message))

            threading.Thread(target=export_job, daemon=True).start()
        except OSError as exc:
            self.log(f"Không thể xuất CSV: {exc}")

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
        self.connect_btn.config(text="Kết nối")
        self.status_label.config(text="Chưa kết nối", fg=ROSE)
        self.estop_latched = None
        self.estop_pending = False
        self.start_pending = False
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
            if command.upper() in ("ENC_RESET", "RESET_ENCODER"):
                # Discard the old tick/time baseline before computing speed.
                self.prev = None
                self.wheel_speed = [0.0, 0.0]
            return True
        except (serial.SerialException, OSError) as exc:
            self.log(f"Lỗi gửi: {exc}")
            self.disconnect()
            return False

    def send_terminal(self):
        cmd = self.cmd_var.get().strip()
        if self.send_command(cmd):
            self.cmd_var.set("")

    def preview_pwm(self, *_args):
        self.pwm_text.set(f"{round(self.pwm_var.get())} / 255")

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

    def update_control_buttons(self):
        connected = self.reader is not None and self.reader.is_alive()
        can_start = (connected and self.estop_latched is False
                     and not self.estop_pending and not self.start_pending
                     and not self.auto_running and self.run_csv.active is None
                     and self.calibration_ok
                     and all(v is not None for v in self.cal_thresholds))
        self.start_btn.configure(state="normal" if can_start else "disabled")
        self.stop_btn.configure(state="normal" if connected else "disabled")

    def set_auto_running(self, running):
        now = time.monotonic()
        if running and not self.auto_running:
            self.run_timer_stationary_since = None
            self.run_timer_stop_latched = False
        elif not running and self.auto_running:
            stop_at = self.run_timer_stationary_since
            self.stop_run_timer(now if stop_at is None else stop_at)
        self.auto_running = running
        self.update_run_timer_display(now)

    def update_run_timer_from_telemetry(self, sample, now=None):
        if not self.auto_running or sample[26] != 1 or self.run_timer_stop_latched:
            return
        now = time.monotonic() if now is None else now
        if sample[19] != 0 or sample[20] != 0:
            self.run_timer_stationary_since = None
            if not self.run_timer_running:
                self.run_started_at = now
                self.run_timer_running = True
            return
        if not self.run_timer_running:
            return
        if self.run_timer_stationary_since is None:
            self.run_timer_stationary_since = now
        elif now - self.run_timer_stationary_since >= RUN_STOP_DEBOUNCE_SECONDS:
            self.stop_run_timer(self.run_timer_stationary_since)
            self.run_timer_stop_latched = True

    def stop_run_timer(self, now=None):
        if self.run_timer_running and self.run_started_at is not None:
            stop_at = time.monotonic() if now is None else now
            self.run_elapsed_seconds += max(0.0, stop_at - self.run_started_at)
        self.run_started_at = None
        self.run_timer_running = False
        self.run_timer_stationary_since = None

    def reset_run_timer(self):
        self.run_elapsed_seconds = 0.0
        self.run_started_at = time.monotonic() if self.run_timer_running else None
        self.run_timer_stationary_since = None
        self.update_run_timer_display()

    def update_run_timer_display(self, now=None):
        elapsed = self.run_elapsed_seconds
        if self.run_timer_running and self.run_started_at is not None:
            elapsed += (time.monotonic() if now is None else now) - self.run_started_at
        total_ticks = max(0, int(elapsed * 100))
        total_seconds, ticks = divmod(total_ticks, 100)
        hours, remainder = divmod(total_seconds, 3600)
        minutes, seconds = divmod(remainder, 60)
        self.run_timer_var.set(
            f"{hours:02d}:{minutes:02d}:{seconds:02d}.{ticks:02d}")

    def finish_run_csv(self, reason):
        recorder = self.run_csv.finish(self.reader, reason)
        if recorder is None:
            return
        self.stop_ack_deadline = None
        self.start_confirmed = False
        self.csv_status.set(f"CSV ĐANG ĐÓNG: {recorder.path.name}")
        self.log(f"CSV đang chốt ({reason}): {recorder.path}")

    def start_line(self):
        if self.estop_latched is not False or self.auto_running or self.start_pending:
            self.log("Chưa thể START: AUTO đang chạy hoặc firmware bị khóa.")
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
                self.reader, CSV_HEADER, calib, list(self.cal_thresholds))
        except (OSError, RuntimeError, ValueError) as exc:
            self.log(f"Không tạo được CSV, chưa gửi AUTO: {exc}")
            return
        self.last_csv_total = 0
        self.stop_ack_deadline = None
        self.start_confirmed = False
        self.csv_status.set(f"CSV GHI: {recorder.path.name}")
        self.log(f"CSV BẮT ĐẦU: {recorder.path}")
        if self.send_command("AUTO"):
            self.start_pending = True
            self.control_state.set("Đã gửi AUTO — chờ ESP32 xác nhận.")
            self.update_control_buttons()
        else:
            self.finish_run_csv("KHÔNG GỬI ĐƯỢC AUTO")

    def stop_line(self):
        # STOP is the single visible stop action. CSV closes after firmware ACK.
        if self.send_command("STOP"):
            if self.run_csv.active is not None:
                self.stop_ack_deadline = time.monotonic() + 2.0
            self.start_pending = False
            self.control_state.set("Đã gửi STOP — chờ ESP32 xác nhận.")
            self.update_control_buttons()
        else:
            self.finish_run_csv("STOP KHÔNG GỬI ĐƯỢC / CHƯA XÁC NHẬN DỪNG")

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
        self.update_run_timer_from_telemetry(s)
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
        self.log(f"RX < {message}")
        if message == "ESTOP: LATCHED" or message == "ERR: ESTOP_LATCHED":
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
        elif message == "MODE: AUTO PID":
            self.start_pending = False
            self.start_confirmed = True
            self.set_auto_running(True)
            self.control_state.set("ĐANG DÒ LINE — PID AUTO đang chạy.")
            self.control_state_label.configure(fg=GREEN)
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
        elif message.startswith("DIAG:"):
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
        elif message in ("MOTOR: STOP", "MODE: MANUAL",
                              "[SAFE] MANUAL DEADMAN: MOTOR STOP",
                              "ERR: ADC_READ_FAULT_MOTORS_STOPPED"):
            self.set_auto_running(False)
            self.start_pending = False
            self.finish_run_csv("ESP32 ĐÃ XÁC NHẬN DỪNG: " + message)
            if self.estop_latched is False:
                self.control_state.set("Đã dừng — sẵn sàng START lượt mới.")
            self.update_control_buttons()
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
                    + f" | {rec.path.name}")
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
            self.finish_run_csv("STOP: HẾT HẠN XÁC NHẬN — CHƯA RÕ ĐỘNG CƠ ĐÃ DỪNG")
            self.control_state.set("STOP chưa được ESP32 xác nhận. Kiểm tra động cơ!")
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
        self.session_log_file.close()
        self.destroy()


if __name__ == "__main__":
    App().mainloop()