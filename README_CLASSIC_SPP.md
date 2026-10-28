# Bluetooth Classic SPP (mainv4)
Target: original ESP32 DevKit V1, PlatformIO esp32dev and Bluetooth Classic SPP.

Windows: pair device "LineRobot" and open its outgoing SPP COM port in Python app. USB debug 115200 8N1 and newline-terminated ASCII commands.

LED onboard GPIO2: blinking each 400 ms when disconnected; solid when SPP connected. If inverted, change DEBUG_LED_ACTIVE_HIGH in include/pinout.h.

Bluetooth telemetry: header AA 55 + packed 69-byte payload + 1 XOR checksum = 72 bytes. Python struct <I16H2i2h5fB, targeted every 10 ms (100 Hz by default, configurable via STREAM:0/20/50/100). ADC readings are RAW (0..4095), not calibrated 0..1000. PC-calibrated displays are independent of ESP calibration.

Both USB and SPP commands: PING, HELP, STATUS, PID, PWM, ENC, ENC_RESET, P, AUTO/START, STOP, ESTOP, RESET_ESTOP, M, F, B, L, R, KP:x, KI:x, KD:x, BASE:n, V:n, CAL_WHITE, CAL_BLACK, CAL_STATUS, CAL_DUMP. Old presets were removed. A binary packet is never sent to USB serial; diagnostic text is printed once per second.

IMPORTANT PID D-term: Based on d3529b7, the D formula is still Kd*(e[k]-e[k-1])/measuredDt where measuredDt is in seconds from micros(). Invalid dt and the first derivative sample after mode/case/Kd changes are skipped. Kd defaults to ZERO. All built-in presets force Kd=0. Old Kd=3.4/3.9/10 must NOT be reused without re-tuning. Only debug P/I/D getters were added; D formula is unchanged.

Calibration: firmware sweeps 4 seconds; failure restores the previous valid Min/Max. Encoder counting is imported from mainv2: interrupt-based quadrature x4 on GPIO 21/22 (left) and GPIO 32/33 (right), with signed cumulative ticks. mainv2 sign convention is left +1, right -1; verify the signs on the physical robot. Counts are transmitted at the configured stream rate (100 Hz default) in both manual and auto PID modes, displayed as signed ticks and smoothed tick/s in the Windows App, and saved with CSV. ENC and ENC_RESET commands work over SPP and USB. This does not introduce a new wheel-speed PID. mainv4 still uses Bluetooth Classic SPP, not Web BLE. See README_ENCODER_MAINV4.md. Manual movement expires after 750 ms with no repeat command, for safety. SPP disconnect stops Auto and motors.

Smoke test:
1. python -m platformio run -e esp32dev
2. python -m platformio run -e esp32dev -t upload
3. python -m platformio device monitor -p COM_USB -b 115200
4. Send PING with newline; expect PONG.
5. Pair LineRobot on Windows; LED solid and USB log [BT] CONNECTED.
6. Observe [DIAG], telemetry up to ~100 Hz (actual rate depends on loop timing and SPP link), and PID: Kd=0.00 before running any motor.
7. `loop()` has no `delay(5)` (only `yield()`); USB `[DIAG]`, `[PERF]`, and `[PERF_LOOP]` report loop frequency and stage timing. The SPP scheduler skips missed slots, never bursts stale frames. Start with wheels off the ground. Build and hardware behavior have not yet been verified.

## Runtime profiling
- `[PERF]`: `adc_us`, `pid_us`, `comm_us`, `tx_us`, `busy_us` are **average/maximum microseconds**. `other_us` is the remaining loop work (buttons, log output, safety). During calibration, `adc_us` also includes statistics and occasionally NVS writes.
- `[PERF_LOOP]`: `period_us=min/mean/max` measures start-to-start intervals (including scheduler interruptions, yield, and USB log spikes); `jitter_pp_us=max-min`; `peak_observed_hz=1e6/minPeriod`. `ceiling_busy_hz=1e6/averageBodyTime` and `ceiling_adc_hz=1e6/averageAdcTime` are **optimistic calculated ceilings**, not demonstrated sustained PID frequencies. `pid_samples=0` means AUTO was off.
- USB and SPP command `TASKS` requests FreeRTOS runtime counters **only if compiled into the framework**; otherwise it prints an explicit unavailable message. Stage timings measure elapsed wall time, not per-task CPU percentages.
- Record 10–20 samples in three conditions: Bluetooth disconnected idle; Bluetooth connected `STREAM:100`; and AUTO enabled with wheels raised. Compare mean/maximum times and jitter before changing ADC averaging or PID frequency.


## Line follow and latched E-STOP

The Windows app has **BẮT ĐẦU DÒ LINE** (`AUTO`) and **DỪNG KHẨN CẤP** (`ESTOP`), plus an explicit **Mở khóa E-STOP** (`RESET_ESTOP`). E-STOP stops PWM and AUTO immediately when the command is processed and latches motion inhibition. AUTO and manual F/B/L/R respond `ERR: ESTOP_LATCHED` until reset. `RESET_ESTOP` does not start either motor. `STATUS` includes `estop=0/1`; confirmation is `ESTOP: LATCHED`. The ESP32 still stops on SPP disconnection. This software stop is not a hardware power cut and cannot guarantee emergency protection if the radio link, firmware or driver fails. 72-byte binary telemetry remains unchanged.
