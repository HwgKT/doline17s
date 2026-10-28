#!/usr/bin/env python3
"""ESP32 Line Robot Windows dashboard over Bluetooth Classic SPP.

Install: pip install pyserial
Run:     python robot_debug.py

First pair the ESP32 in Windows and select its SPP COM port.
mainv4 firmware now starts Classic SPP as LineRobot and sends 72-byte frames.
Use its outgoing Windows Bluetooth COM port, separate from the USB serial COM port.

The 69-byte SPP payload includes raw ADC, not firmware-calibrated ADC. Thus,
values 0..1000 and black/white bits below come from independent local PC
Min/Max calibration. The GUI does not claim to read firmware calibration data.
"""
import collections
import csv
import math
import operator
import functools
import queue
import re
import struct
import threading
import time
import tkinter as tk
from tkinter import ttk

import serial
from serial.tools import list_ports

HDR = b"\xAA\x55"
FMT = "<I16H2i2h5fB"
PAYLOAD_LEN = struct.calcsize(FMT)  # 69
FRAME_LEN = len(HDR) + PAYLOAD_LEN + 1  # 72
ADC_MAX, PWM_MAX, HIST_LEN = 4095, 255, 300
MIN_CALIB_DELTA = 150
# Time-normalized derivative: keep Kd=0 until hardware re-tuning.
PROFILES = {
    "Siêu bám": (1.45, 0.0, 0.00, 105),
    "Pin 3S": (1.55, 0.0, 0.00, 115),
    "Pin 2S": (1.55, 0.0, 0.00, 145),
    "Race": (1.65, 0.0, 0.00, 135),
}
CSV_HEADER = (["t_ms"] + [f"adc{i}" for i in range(16)] +
              ["encL", "encR", "pwmL", "pwmR", "err", "p", "i", "d", "out", "mode"])
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

    def __init__(self, port, out_queue):
        super().__init__(daemon=True)
        self.ser = serial.Serial(port, 115200, timeout=0.05)
        self.out_queue = out_queue
        self.running = True
        self.buf = bytearray()
        self.text_buf = bytearray()
        self.bad = 0

    def run(self):
        while self.running:
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
            except (serial.SerialException, OSError):
                self.out_queue.put(("error", "Bluetooth/COM đã ngắt kết nối"))
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
                        self.out_queue.put(("text", line))
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
                self.out_queue.put(("data", struct.unpack(FMT, data)))
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
        self.geometry("1200x810")
        self.minsize(1050, 760)
        self.configure(bg=BG)
        self.q = queue.Queue()
        self.reader = None
        self.csv_file = None
        self.csv_writer = None
        self.hist = collections.deque(maxlen=HIST_LEN)
        self.prev = None
        self.wheel_speed = [0.0, 0.0]
        self.frames = 0
        self.rate_time = time.monotonic()
        self.values = {}
        self.last_adc = [0] * 16
        # PC-only calibration state. Firmware calibration values are unavailable in SPP.
        self.calibration_ok = False
        self.calibrating = False
        self.black_high = True
        self.cal_min = [0] * 16
        self.cal_max = [4095] * 16
        self.scan_min = [4095] * 16
        self.scan_max = [0] * 16
        self.scan_samples = 0
        self.cal_timer = None
        self._build()
        self.refresh_ports()
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(30, self.tick)

    def label(self, parent, text="", fg=FG, bg=None, **kw):
        return tk.Label(parent, text=text, fg=fg, bg=BG if bg is None else bg, **kw)

    def _build(self):
        top = tk.Frame(self, bg=BG)
        top.pack(fill="x", padx=12, pady=10)
        self.label(top, "Bluetooth Classic SPP", font=("Segoe UI", 12, "bold")).pack(side="left", padx=(0, 10))
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=14)
        self.port_combo.pack(side="left", padx=3)
        ttk.Button(top, text="Quét COM", command=self.refresh_ports).pack(side="left", padx=3)
        self.connect_btn = ttk.Button(top, text="Kết nối", command=self.toggle_connect)
        self.connect_btn.pack(side="left", padx=3)
        self.csv_btn = ttk.Button(top, text="Ghi CSV", command=self.toggle_csv)
        self.csv_btn.pack(side="left", padx=8)
        self.status_label = self.label(top, "Chưa kết nối", fg=ROSE)
        self.status_label.pack(side="left", padx=5)
        self.rate_label = self.label(top, "0 Hz | 0 lỗi gói", fg=MUTED)
        self.rate_label.pack(side="right")

        body = tk.Frame(self, bg=BG)
        body.pack(fill="both", expand=True, padx=12, pady=3)
        left = tk.Frame(body, bg=BG)
        left.pack(side="left", fill="both", expand=True, padx=(0, 12))
        right = tk.Frame(body, bg=PANEL, width=390)
        right.pack(side="right", fill="y")
        right.pack_propagate(False)
        panel = tk.Frame(right, bg=PANEL)
        panel.pack(fill="both", expand=True, padx=12, pady=12)

        sensor_head = tk.Frame(left, bg=BG)
        sensor_head.pack(fill="x")
        self.label(sensor_head, "16 mắt: RAW / CAL / LOGIC", font=("Segoe UI", 11, "bold")).pack(side="left")
        self.label(sensor_head, "Ngưỡng raw", fg=MUTED).pack(side="left", padx=(15, 4))
        self.threshold = tk.DoubleVar(value=2000)
        ttk.Scale(sensor_head, from_=0, to=4095, variable=self.threshold,
                  length=120).pack(side="left")
        self.threshold_label = self.label(sensor_head, "2000", fg=AMBER)
        self.threshold_label.pack(side="left", padx=5)
        self.bars = tk.Canvas(left, bg=PANEL, width=715, height=250, highlightthickness=0)
        self.bars.pack(fill="x", pady=5)
        self.logic_var = tk.StringVar(value="Logic: ---------------- (chưa calib PC)")
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
            self.label(metrics, name, fg=MUTED, bg=PANEL, width=18,
                       anchor="e").grid(row=r, column=c * 2, padx=(5, 1), pady=5)
            self.values[key] = tk.StringVar(value="—")
            self.label(metrics, textvariable=self.values[key], bg=PANEL, fg=CYAN,
                       font=("Consolas", 10, "bold"), width=10,
                       anchor="w").grid(row=r, column=c * 2 + 1, padx=(1, 4))

        self.label(left, "Scope: PWM trái / PWM phải / PID out", fg=MUTED).pack(anchor="w", pady=(11, 3))
        self.scope = tk.Canvas(left, bg=PANEL, width=715, height=190, highlightthickness=0)
        self.scope.pack(fill="x")
        modes = tk.Frame(left, bg=BG)
        modes.pack(fill="x", pady=12)
        self.label(modes, "Điều khiển", fg=MUTED).pack(side="left", padx=(0, 9))
        for title, cmd in (("Auto", "A"), ("Manual", "M"), ("DỪNG", "S")):
            ttk.Button(modes, text=title, width=12,
                       command=lambda text=cmd: self.send_command(text)).pack(side="left", padx=4)
        self.label(left, "Chỉ số CAL/0-1 được tính từ Min/Max đo ở PC, không đọc từ ESP32.",
                   fg=MUTED).pack(anchor="w")

        self.label(panel, "Tốc độ PWM (0–255)", bg=PANEL,
                   font=("Segoe UI", 11, "bold")).pack(anchor="w")
        self.pwm_var = tk.DoubleVar(value=120)
        self.pwm_text = tk.StringVar(value="120 / 255")
        self.label(panel, textvariable=self.pwm_text, bg=PANEL, fg=CYAN).pack(anchor="w")
        self.pwm_slider = ttk.Scale(panel, from_=0, to=255, variable=self.pwm_var,
                                    command=self.preview_pwm)
        self.pwm_slider.pack(fill="x", pady=4)
        self.pwm_slider.bind("<ButtonRelease-1>", lambda _e: self.apply_pwm())
        self.pwm_slider.bind("<KeyRelease>", lambda _e: self.apply_pwm())
        speed_buttons = tk.Frame(panel, bg=PANEL)
        speed_buttons.pack(fill="x", pady=(0, 10))
        for val in (60, 120, 180, 255):
            ttk.Button(speed_buttons, text=str(val), width=6,
                       command=lambda v=val: self.preset_pwm(v)).pack(side="left", expand=True, padx=2)

        self.label(panel, "Kp / Ki / Kd / Base", bg=PANEL,
                   font=("Segoe UI", 11, "bold")).pack(anchor="w", pady=(3, 4))
        pid = tk.Frame(panel, bg=PANEL)
        pid.pack(fill="x")
        self.pid_vars = {}
        for idx, (key, initial) in enumerate((("Kp", "2.2"), ("Ki", "0"),
                                             ("Kd", "0"), ("Base", "120"))):
            self.label(pid, key, bg=PANEL, fg=MUTED).grid(row=0, column=idx, sticky="w")
            var = tk.StringVar(value=initial)
            self.pid_vars[key] = var
            ttk.Entry(pid, textvariable=var, width=9).grid(row=1, column=idx, padx=(0, 6), sticky="ew")
            pid.grid_columnconfigure(idx, weight=1)
        pid_actions = tk.Frame(panel, bg=PANEL)
        pid_actions.pack(fill="x", pady=(6, 3))
        ttk.Button(pid_actions, text="Áp dụng PID", command=self.apply_pid).pack(side="left", fill="x", expand=True, padx=(0, 3))
        ttk.Button(pid_actions, text="Đọc lại PID", command=lambda: self.send_command("PID")).pack(side="left", fill="x", expand=True)

        self.label(panel, "Presets (không tự bật Auto)", bg=PANEL, fg=MUTED).pack(anchor="w", pady=(9, 4))
        profiles = tk.Frame(panel, bg=PANEL)
        profiles.pack(fill="x")
        for index, name in enumerate(PROFILES):
            ttk.Button(profiles, text=name, command=lambda p=name: self.apply_profile(p)).grid(
                row=index // 2, column=index % 2, sticky="ew", padx=2, pady=2)
        profiles.grid_columnconfigure(0, weight=1)
        profiles.grid_columnconfigure(1, weight=1)

        self.label(panel, "Calibration 16 mắt (PC + lệnh ESP32)", bg=PANEL,
                   font=("Segoe UI", 11, "bold")).pack(anchor="w", pady=(12, 4))
        cal_buttons = tk.Frame(panel, bg=PANEL)
        cal_buttons.pack(fill="x")
        ttk.Button(cal_buttons, text="Lia Calib 4 s", command=self.start_calibration).pack(side="left", fill="x", expand=True, padx=(0, 3))
        ttk.Button(cal_buttons, text="Chốt sớm", command=self.stop_calibration).pack(side="left", fill="x", expand=True)
        self.cal_state = tk.StringVar(value="Chưa hiệu chuẩn trên PC")
        self.label(panel, textvariable=self.cal_state, bg=PANEL, fg=AMBER,
                   justify="left", anchor="w", wraplength=350).pack(fill="x", pady=5)
        ttk.Button(panel, text="Đảo cực (INV)", command=self.invert).pack(fill="x")
        self.polarity_state = tk.StringVar(value="ADC cao = đen (1000)")
        self.label(panel, textvariable=self.polarity_state, bg=PANEL, fg=MUTED).pack(anchor="w", pady=4)

        terminal_heading = tk.Frame(panel, bg=PANEL)
        terminal_heading.pack(fill="x", pady=(6, 3))
        self.label(terminal_heading, "Terminal TX / RX", bg=PANEL,
                   font=("Segoe UI", 11, "bold")).pack(side="left")
        ttk.Button(terminal_heading, text="Xóa log", command=self.clear_terminal).pack(side="right")
        text_area = tk.Frame(panel, bg=PANEL)
        text_area.pack(fill="both", expand=True)
        self.console = tk.Text(text_area, bg=BG, fg=FG, insertbackground=FG,
                               font=("Consolas", 9), height=8, wrap="word",
                               state="disabled", relief="flat")
        scrollbar = ttk.Scrollbar(text_area, orient="vertical", command=self.console.yview)
        self.console.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        self.console.pack(side="left", fill="both", expand=True)
        cmd_row = tk.Frame(panel, bg=PANEL)
        cmd_row.pack(fill="x", pady=(5, 0))
        self.cmd_var = tk.StringVar()
        command_entry = ttk.Entry(cmd_row, textvariable=self.cmd_var)
        command_entry.pack(side="left", fill="x", expand=True, padx=(0, 4))
        command_entry.bind("<Return>", lambda _e: self.send_terminal())
        ttk.Button(cmd_row, text="Gửi", command=self.send_terminal).pack(side="right")
        self.log("Chọn COM Bluetooth SPP (outgoing). Chưa có xác nhận firmware.")

    def log(self, message):
        self.console.configure(state="normal")
        self.console.insert("end", f"[{time.strftime('%H:%M:%S')}] {message}\n")
        if int(self.console.index("end-1c").split(".")[0]) > 550:
            self.console.delete("1.0", "101.0")
        self.console.see("end")
        self.console.configure(state="disabled")

    def clear_terminal(self):
        self.console.configure(state="normal")
        self.console.delete("1.0", "end")
        self.console.configure(state="disabled")

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
            self.reader = Reader(port, self.q)
        except (serial.SerialException, OSError, ValueError) as exc:
            self.status_label.config(text=f"Lỗi COM: {exc}", fg=ROSE)
            self.log(str(exc))
            return
        self.reader.start()
        self.prev = None
        self.connect_btn.config(text="Ngắt")
        self.status_label.config(text=f"Đã mở {port}", fg=GREEN)
        self.log(f"Đã mở {port}; chờ frame SPP từ ESP32")

    def disconnect(self):
        if self.reader:
            self.reader.running = False
            self.reader.join(timeout=1)
            self.reader = None
        self.connect_btn.config(text="Kết nối")
        self.status_label.config(text="Chưa kết nối", fg=ROSE)
        self.log("Đã đóng COM. Dừng động cơ cần được bảo đảm trên firmware.")

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

    def apply_profile(self, name):
        kp, ki, kd, base = PROFILES[name]
        for key, value in (("Kp", kp), ("Ki", ki), ("Kd", kd), ("Base", base)):
            self.pid_vars[key].set(str(value))
        self.log(f"Preset {name}: Kp={kp}, Ki={ki}, Kd={kd}, Base={base} (không tự Auto)")
        self.apply_pid()

    def start_calibration(self):
        if self.calibrating:
            self.log("Đang hiệu chuẩn")
            return
        if not self.send_command("C"):
            return
        self.scan_min = [4095] * 16
        self.scan_max = [0] * 16
        self.scan_samples = 0
        self.calibrating = True
        self.cal_state.set("Đang lia qua nền/vạch, thu Min/Max trên PC (4 s)…")
        self.cal_timer = self.after(4000, lambda: self.finish_calibration(auto=True))
        self.log("Bắt đầu quét PC 4 s; firmware phải tự kết thúc sau 4 s")

    def stop_calibration(self):
        self.finish_calibration(auto=False)

    def finish_calibration(self, auto=False):
        if not self.calibrating:
            self.log("Không có lượt hiệu chuẩn đang chạy")
            return
        self.calibrating = False
        if self.cal_timer is not None:
            self.after_cancel(self.cal_timer)
            self.cal_timer = None
        if not auto:
            self.send_command("K")
        failed = [i for i in range(16)
                  if self.scan_max[i] - self.scan_min[i] < MIN_CALIB_DELTA]
        if self.scan_samples == 0 or failed:
            self.cal_state.set(f"Calib PC chưa đạt: mắt {failed}. Giữ Min/Max PC cũ nếu có.")
            self.log(f"Calib PC thất bại: {self.scan_samples} mẫu, thiếu delta mắt {failed}")
            return
        self.cal_min = self.scan_min[:]
        self.cal_max = self.scan_max[:]
        self.calibration_ok = True
        self.cal_state.set(f"Đã hiệu chuẩn PC: 16/16 mắt, {self.scan_samples} frame")
        self.log("Calib PC thành công: 0–1000, đen khi >=500; không xác nhận calib ESP32")
        self.draw_bars(self.last_adc)

    def invert(self):
        if self.send_command("INV"):
            self.black_high = not self.black_high
            self.polarity_state.set("ADC cao = đen (1000)" if self.black_high
                                    else "ADC thấp = đen (1000)")
            self.log("Đã đảo cực tính hiển thị PC; chưa được firmware xác nhận")
            self.draw_bars(self.last_adc)

    def toggle_csv(self):
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = self.csv_writer = None
            self.csv_btn.config(text="Ghi CSV")
            self.log("Đã đóng CSV")
            return
        filename = time.strftime("robot_log_%Y%m%d_%H%M%S.csv")
        try:
            self.csv_file = open(filename, "w", newline="", encoding="utf-8")
        except OSError as exc:
            self.log(f"Không mở được CSV: {exc}")
            return
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(CSV_HEADER)
        self.csv_btn.config(text="Dừng CSV")
        self.log(f"Bắt đầu ghi {filename}")

    def on_sample(self, s):
        self.hist.append(s)
        self.frames += 1
        if self.csv_writer:
            self.csv_writer.writerow(s)
        self.last_adc = list(s[1:17])
        if self.calibrating:
            self.scan_samples += 1
            for i, adc in enumerate(self.last_adc):
                self.scan_min[i] = min(self.scan_min[i], adc)
                self.scan_max[i] = max(self.scan_max[i], adc)
        t, left, right = s[0], s[17], s[18]
        if self.prev and t > self.prev[0]:
            dt = (t - self.prev[0]) / 1000.0
            v_left = (left - self.prev[1]) / dt
            v_right = (right - self.prev[2]) / dt
            self.wheel_speed[0] = 0.8 * self.wheel_speed[0] + 0.2 * v_left
            self.wheel_speed[1] = 0.8 * self.wheel_speed[1] + 0.2 * v_right
        self.prev = (t, left, right)

    def on_text(self, message):
        self.log(f"RX < {message}")
        pid = re.search(r"PID:\s*Kp=([\d.]+)\s+Ki=([\d.]+)\s+Kd=([\d.]+)\s+Base=(\d+)",
                        message, flags=re.IGNORECASE)
        if pid:
            for key, value in zip(("Kp", "Ki", "Kd", "Base"), pid.groups()):
                self.pid_vars[key].set(value)
            self.pwm_var.set(int(pid.group(4)))
            self.preview_pwm()
        if "CALIB THANH CONG" in message:
            self.log("ESP32 báo calib thành công; dữ liệu CAL vẫn dùng Min/Max PC")
        elif "CALIB" in message and ("CHUA DAT" in message or "THAT BAI" in message):
            self.log("ESP32 báo hiệu chuẩn chưa đạt")

    def tick(self):
        last = None
        try:
            while True:
                kind, payload = self.q.get_nowait()
                if kind == "data":
                    self.on_sample(payload)
                    last = payload
                elif kind == "text":
                    self.on_text(payload)
                elif kind == "error":
                    self.disconnect()
                    self.status_label.config(text=payload, fg=ROSE)
        except queue.Empty:
            pass
        if last is not None:
            self.update_view(last)
        now = time.monotonic()
        if now - self.rate_time >= 1:
            errors = self.reader.bad if self.reader else 0
            self.rate_label.config(text=f"{self.frames / (now - self.rate_time):.0f} Hz | {errors} lỗi gói")
            self.frames = 0
            self.rate_time = now
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
        self.threshold_label.config(text=str(round(self.threshold.get())))
        self.draw_bars(s[1:17])
        self.draw_scope()

    def draw_bars(self, adc_values):
        c = self.bars
        c.delete("all")
        width = max(715, c.winfo_width())
        chart_top, chart_height = 22, 155
        step = width / 16
        threshold = self.threshold.get()
        bits = []
        for i, adc in enumerate(adc_values):
            cal = normalize_adc(adc, self.cal_min[i], self.cal_max[i], self.black_high) if self.calibration_ok else None
            black = (cal >= 500) if cal is not None else (adc >= threshold)
            bits.append("1" if cal is not None and black else "0" if cal is not None else "-")
            x0, x1 = i * step + 4, (i + 1) * step - 4
            bar_height = chart_height * adc / ADC_MAX
            c.create_rectangle(x0, chart_top + chart_height - bar_height, x1,
                               chart_top + chart_height, fill=GREEN if black else GREY, outline="")
            c.create_text((x0 + x1) / 2, 12, text=str(adc), fill=FG, font=("Consolas", 8))
            c.create_text((x0 + x1) / 2, 191, text="—" if cal is None else str(cal),
                          fill=CYAN, font=("Consolas", 8))
            c.create_text((x0 + x1) / 2, 212, text=bits[-1],
                          fill=GREEN if bits[-1] == "1" else MUTED)
            c.create_text((x0 + x1) / 2, 238, text=str(i), fill=MUTED)
        threshold_y = chart_top + chart_height * (1 - threshold / ADC_MAX)
        c.create_line(0, threshold_y, width, threshold_y, fill=AMBER, dash=(4, 3))
        self.logic_var.set("Logic: " + "".join(bits) + ("  (CAL PC)" if self.calibration_ok else "  (chưa calib PC)"))

    def draw_scope(self):
        c = self.scope
        c.delete("all")
        width, height = max(715, c.winfo_width()), int(c["height"])
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
        if self.cal_timer is not None:
            self.after_cancel(self.cal_timer)
        self.disconnect()
        if self.csv_file:
            self.csv_file.close()
        self.destroy()


if __name__ == "__main__":
    App().mainloop()