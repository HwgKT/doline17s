#!/usr/bin/env python3
"""ESP32 Line Robot - Debug monitor qua Bluetooth Classic (SPP).

Cài đặt:  pip install pyserial
Chạy:     python robot_debug.py

Ghép đôi "LineRobot" trong cài đặt Bluetooth của hệ điều hành trước,
sau đó chọn cổng COM (Windows) / /dev/rfcomm0 (Linux) / /dev/cu.LineRobot (macOS).
"""
import collections
import csv
import functools
import operator
import queue
import struct
import threading
import time
import tkinter as tk
from tkinter import ttk

import serial
from serial.tools import list_ports

HDR = b"\xAA\x55"
FMT = "<I16H2i2h5fB"            # t_ms, adc[16], encL, encR, pwmL, pwmR, err, p, i, d, out, mode
PAYLOAD_LEN = struct.calcsize(FMT)   # 69
FRAME_LEN = len(HDR) + PAYLOAD_LEN + 1

ADC_MAX = 4095
PWM_MAX = 255
HIST_LEN = 300
MODES = {0: "MANUAL", 1: "AUTO PID"}
CSV_HEADER = (["t_ms"] + [f"adc{i}" for i in range(16)]
              + ["encL", "encR", "pwmL", "pwmR", "err", "p", "i", "d", "out", "mode"])

BG, PANEL, FG, MUTED = "#0f1420", "#161d31", "#e2e8f0", "#94a3b8"
GREEN, GREY, AMBER, CYAN, ROSE = "#10b981", "#475569", "#f59e0b", "#22d3ee", "#f43f5e"


def xor_sum(data):
    return functools.reduce(operator.xor, data, 0)


class Reader(threading.Thread):
    """Đọc cổng serial, tách gói theo header + checksum, đẩy vào queue."""

    def __init__(self, port, q):
        super().__init__(daemon=True)
        self.ser = serial.Serial(port, 115200, timeout=0.05)  # baud không ảnh hưởng với SPP
        self.q = q
        self.running = True
        self.buf = bytearray()
        self.bad = 0
        self.textbuf = bytearray()

    def run(self):
        while self.running:
            try:
                data = self.ser.read(self.ser.in_waiting or 1)
            except (serial.SerialException, OSError):
                self.q.put(("error", "Mất kết nối"))
                break
            if data:
                self.buf += data
                self._parse()
        try:
            self.ser.close()
        except Exception:
            pass

    def _consume_text(self, data):
        self.textbuf.extend(data)
        while b"\n" in self.textbuf:
            line, _, rest = self.textbuf.partition(b"\n")
            self.textbuf = bytearray(rest)
            msg = line.decode("utf-8", errors="replace").strip()
            if msg:
                self.q.put(("message", msg))
        if len(self.textbuf) > 512:
            self.textbuf = self.textbuf[-128:]

    def _parse(self):
        while True:
            i = self.buf.find(HDR)
            if i < 0:
                if len(self.buf) > 1:
                    self._consume_text(bytes(self.buf[:-1]))
                    del self.buf[:-1]
                return
            if i > 0:
                self._consume_text(bytes(self.buf[:i]))
                del self.buf[:i]
            if len(self.buf) < FRAME_LEN:
                return
            payload = bytes(self.buf[2:2 + PAYLOAD_LEN])
            if xor_sum(payload) == self.buf[2 + PAYLOAD_LEN]:
                self.q.put(("data", struct.unpack(FMT, payload)))
                del self.buf[:FRAME_LEN]
            else:
                self.bad += 1
                del self.buf[:1]           # lệch sync, tìm header kế tiếp

    def send(self, text):
        self.ser.write((text + "\n").encode())


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ESP32 Line Robot - Debug (Bluetooth Classic)")
        self.configure(bg=BG)
        self.q = queue.Queue()
        self.reader = None
        self.csv_file = None
        self.csv_writer = None
        self.hist = collections.deque(maxlen=HIST_LEN)
        self.prev = None
        self.speed = (0.0, 0.0)
        self.frames = 0
        self.rate_t = time.time()
        self.vars = {}
        self._build()
        self.refresh_ports()
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(30, self.tick)

    # ---------- UI ----------
    def _build(self):
        top = tk.Frame(self, bg=BG)
        top.pack(fill="x", padx=10, pady=8)
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(top, textvariable=self.port_var, width=20)
        self.port_box.pack(side="left")
        ttk.Button(top, text="⟳", width=3, command=self.refresh_ports).pack(side="left", padx=4)
        self.btn_conn = ttk.Button(top, text="Kết nối", command=self.toggle_connect)
        self.btn_conn.pack(side="left", padx=4)
        self.btn_csv = ttk.Button(top, text="● Ghi CSV", command=self.toggle_csv)
        self.btn_csv.pack(side="left", padx=4)
        self.status = tk.Label(top, text="Chưa kết nối", bg=BG, fg=ROSE)
        self.status.pack(side="left", padx=10)
        self.rate_lbl = tk.Label(top, text="0 Hz", bg=BG, fg=MUTED)
        self.rate_lbl.pack(side="right")

        # 16 mắt cảm biến
        sf = tk.Frame(self, bg=BG)
        sf.pack(fill="x", padx=10)
        tk.Label(sf, text="ADC 16 mắt (ngưỡng tô xanh):", bg=BG, fg=MUTED).pack(side="left")
        self.th_var = tk.IntVar(value=2000)
        ttk.Scale(sf, from_=0, to=ADC_MAX, variable=self.th_var, length=240).pack(side="left", padx=8)
        self.th_lbl = tk.Label(sf, text="2000", bg=BG, fg=AMBER, width=5)
        self.th_lbl.pack(side="left")
        self.bars = tk.Canvas(self, width=760, height=210, bg=PANEL, highlightthickness=0)
        self.bars.pack(padx=10, pady=4)
        cf = tk.Frame(self, bg=BG)
        cf.pack(fill="x", padx=10, pady=5)
        for title, command in (("1. Lấy TRẮNG", "CAL_WHITE"),
                               ("2. Lấy ĐEN", "CAL_BLACK"),
                               ("Trạng thái", "CAL_STATUS"),
                               ("Đọc tham số", "CAL_DUMP")):
            ttk.Button(cf, text=title, command=lambda c=command: self.send_cmd(c)).pack(side="left", padx=4)
        self.cal_status = tk.Label(self, text="Calibration: chưa truy vấn", bg=BG, fg=AMBER)
        self.cal_status.pack(fill="x", padx=10)

        # Thông số số học
        grid = tk.Frame(self, bg=PANEL)
        grid.pack(fill="x", padx=10, pady=4)
        items = [("encL", "Encoder L"), ("encR", "Encoder R"), ("spdL", "Tốc độ L (tick/s)"),
                 ("spdR", "Tốc độ R (tick/s)"), ("pwmL", "PWM L"), ("pwmR", "PWM R"),
                 ("err", "Error"), ("out", "PID out"), ("p", "P"), ("i", "I"), ("d", "D"),
                 ("mode", "Chế độ")]
        for n, (key, name) in enumerate(items):
            r, c = divmod(n, 4)
            tk.Label(grid, text=name, bg=PANEL, fg=MUTED, anchor="e", width=16).grid(
                row=r, column=c * 2, sticky="e", padx=(8, 2), pady=3)
            self.vars[key] = tk.StringVar(value="-")
            tk.Label(grid, textvariable=self.vars[key], bg=PANEL, fg=CYAN,
                     font=("Consolas", 11, "bold"), anchor="w", width=10).grid(
                row=r, column=c * 2 + 1, sticky="w")

        # Scope
        self.scope = tk.Canvas(self, width=760, height=170, bg=PANEL, highlightthickness=0)
        self.scope.pack(padx=10, pady=4)

        # Gửi lệnh
        cmd = tk.Frame(self, bg=BG)
        cmd.pack(fill="x", padx=10, pady=8)
        for label in ("A", "M", "S"):
            ttk.Button(cmd, text=label, width=4,
                       command=lambda t=label: self.send_cmd(t)).pack(side="left", padx=2)
        self.cmd_var = tk.StringVar()
        ent = ttk.Entry(cmd, textvariable=self.cmd_var, width=40)
        ent.pack(side="left", padx=8)
        ent.bind("<Return>", lambda e: self.send_cmd(self.cmd_var.get()))
        ttk.Button(cmd, text="Gửi", command=lambda: self.send_cmd(self.cmd_var.get())).pack(side="left")

    # ---------- Kết nối ----------
    def refresh_ports(self):
        ports = [p.device for p in list_ports.comports()]
        self.port_box["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def toggle_connect(self):
        if self.reader:
            self.disconnect()
            return
        port = self.port_var.get().strip()
        try:
            self.reader = Reader(port, self.q)
        except (serial.SerialException, ValueError, OSError) as e:
            self.status.config(text=f"Lỗi: {e}", fg=ROSE)
            return
        self.reader.start()
        self.prev = None
        self.btn_conn.config(text="Ngắt")
        self.status.config(text=f"Đã kết nối {port}", fg=GREEN)

    def disconnect(self):
        if self.reader:
            self.reader.running = False
            self.reader.join(timeout=1)
            self.reader = None
        self.btn_conn.config(text="Kết nối")
        self.status.config(text="Chưa kết nối", fg=ROSE)

    def send_cmd(self, text):
        text = text.strip()
        if self.reader and text:
            try:
                self.reader.send(text)
            except (serial.SerialException, OSError):
                self.disconnect()

    def toggle_csv(self):
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = self.csv_writer = None
            self.btn_csv.config(text="● Ghi CSV")
            return
        name = time.strftime("robot_log_%Y%m%d_%H%M%S.csv")
        self.csv_file = open(name, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(CSV_HEADER)
        self.btn_csv.config(text=f"■ Dừng ({name})")

    # ---------- Xử lý dữ liệu ----------
    def on_sample(self, s):
        self.hist.append(s)
        self.frames += 1
        if self.csv_writer:
            self.csv_writer.writerow(s)
        t, eL, eR = s[0], s[17], s[18]
        if self.prev and t > self.prev[0]:
            dt = (t - self.prev[0]) / 1000.0
            vL, vR = (eL - self.prev[1]) / dt, (eR - self.prev[2]) / dt
            self.speed = (0.8 * self.speed[0] + 0.2 * vL, 0.8 * self.speed[1] + 0.2 * vR)  # làm mượt
        self.prev = (t, eL, eR)

    def tick(self):
        last = None
        try:
            while True:
                kind, payload = self.q.get_nowait()
                if kind == "data":
                    self.on_sample(payload)
                    last = payload
                elif kind == "message":
                    if payload.startswith("CAL_"):
                        self.cal_status.config(text=payload)
                    else:
                        self.status.config(text=payload[:90])
                elif kind == "error":
                    self.disconnect()
                    self.status.config(text=payload, fg=ROSE)
        except queue.Empty:
            pass
        if last:
            self.update_view(last)
        now = time.time()
        if now - self.rate_t >= 1.0:
            bad = self.reader.bad if self.reader else 0
            self.rate_lbl.config(text=f"{self.frames / (now - self.rate_t):.0f} Hz | lỗi gói: {bad}")
            self.frames = 0
            self.rate_t = now
        self.after(30, self.tick)

    def update_view(self, s):
        v = self.vars
        v["encL"].set(str(s[17]))
        v["encR"].set(str(s[18]))
        v["spdL"].set(f"{self.speed[0]:.0f}")
        v["spdR"].set(f"{self.speed[1]:.0f}")
        v["pwmL"].set(str(s[19]))
        v["pwmR"].set(str(s[20]))
        v["err"].set(f"{s[21]:.2f}")
        v["p"].set(f"{s[22]:.2f}")
        v["i"].set(f"{s[23]:.2f}")
        v["d"].set(f"{s[24]:.2f}")
        v["out"].set(f"{s[25]:.2f}")
        v["mode"].set(MODES.get(s[26], str(s[26])))
        self.th_lbl.config(text=str(int(self.th_var.get())))
        self.draw_bars(s[1:17])
        self.draw_scope()

    def draw_bars(self, adc):
        c = self.bars
        c.delete("all")
        W, H = int(c["width"]), int(c["height"])
        top, bottom, pad = 20, 18, 4
        ch = H - top - bottom
        bw = W / 16
        th = self.th_var.get()
        for i, val in enumerate(adc):
            x0, x1 = i * bw + pad, (i + 1) * bw - pad
            h = ch * val / ADC_MAX
            c.create_rectangle(x0, top + ch - h, x1, top + ch,
                               fill=GREEN if val >= th else GREY, outline="")
            c.create_text((x0 + x1) / 2, top + ch - h - 8, text=str(val),
                          fill=FG, font=("Consolas", 8))
            c.create_text((x0 + x1) / 2, H - 8, text=str(i), fill=MUTED)
        ty = top + ch - ch * th / ADC_MAX
        c.create_line(0, ty, W, ty, fill=AMBER, dash=(4, 3))

    def draw_scope(self):
        c = self.scope
        c.delete("all")
        W, H = int(c["width"]), int(c["height"])
        mid = H / 2
        c.create_line(0, mid, W, mid, fill="#334155")
        for n, (name, color) in enumerate((("pwmL", CYAN), ("pwmR", AMBER), ("PID out", ROSE))):
            c.create_text(40 + n * 70, 10, text=name, fill=color, font=("Consolas", 9))
        if len(self.hist) < 2:
            return
        for idx, color in ((19, CYAN), (20, AMBER), (25, ROSE)):
            pts = []
            for k, row in enumerate(self.hist):
                x = k * W / (HIST_LEN - 1)
                y = mid - (row[idx] / PWM_MAX) * (mid - 6)
                pts += [x, max(2, min(H - 2, y))]
            c.create_line(*pts, fill=color)

    def on_close(self):
        self.disconnect()
        if self.csv_file:
            self.csv_file.close()
        self.destroy()


if __name__ == "__main__":
    App().mainloop()
