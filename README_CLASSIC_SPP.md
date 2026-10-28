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

## ADC/MUX read benchmarking (safe, non-blocking)
- On USB 115200 terminal (or Bluetooth command terminal), send `ADC_BENCH`. It always stops motor output and AUTO. While it runs, firmware denies AUTO/manual movement/calibration, and still parses ESTOP between complete scans. Send `ADC_BENCH_STOP` to cancel or `ADC_BENCH_STATUS` to see progress. The 72-byte SPP protocol stays unchanged; telemetry is paused only during the benchmark.
- It sweeps **9 configurations**: `ARDUINO` (digitalWrite + analogRead), `IDF` (digitalWrite + adc1_get_raw on ADC1_CH0/GPIO36), and `IDF_GPIO` (direct GPIO output registers + adc1_get_raw), each for `samples=4,2,1`. The first read after each MUX switch is always discarded, 6 us MUX settle and 3 us after each retained sample are preserved.
- It measures 64 full scans of all 16 eyes per configuration, interleaved by pass. `ARDUINO/4` is the paired reference on each pass. **Keep the sensor bar perfectly stationary** in three separate runs on white, black and a black/white edge. Avoid changing illumination or touching the sensor bar during a run.
- Nine `[ADC_BENCH]` lines on **USB Serial** report full scan `scan_us=avg/max`, breakdown `mux_us, settle_us, discard_us, read_us, wait_us` in microseconds per scan, `noise_adc=mean/max per-eye SD` across repeated scans, `mae_adc/maxerr_adc` vs paired reference, `mismatch=count/1024` vs the existing NVS per-eye thresholds, `near100` reference values within 100 raw ADC counts of a threshold, `discard_delta` first discarded ADC vs kept average (an **imperfect proxy**, not proof, of MUX settling), and `invalid` number of ADC1 read failures.
- The benchmark does **not** change production `sensorRaw`, calibration in NVS or production scan algorithm. `ADC_BENCH` does configure ADC1 width to 12 bits and GPIO36 attenuation to 11dB to match Arduino's usual default; compare white/black references to rule out offset changes.
- `mismatch=0` on uniform white/black alone is insufficient to certify the PID; test an edge where some eyes sit close to their thresholds. The most promising mode is one that reduces scan time **and** keeps noise, threshold mismatches and carryover proxy acceptable. Do not select merely the smallest `scan_us`.

## Production 16-channel ADC mode (selected fastest tested configuration)
- Production sensor reads now use **IDF_GPIO/1**: ADC1_CH0 at GPIO36, 12 bits, attenuation 11 dB, direct GPIO writes to select MUX channels on GPIO17/18/19/23, 6-us settle, one discarded sample, one retained `adc1_get_raw()` and 3-us post-sample wait. This matches the fastest of nine bench modes.
- In the stationary hardware benchmark, ARDUINO/4 took ~12,232 us per 16 eyes versus IDF_GPIO/1 ~1,615 us, a 7.57x reduction in measured scan duration. These are **bench scan-only**, not proven sustained PID/BT loop Hz; edge-of-line detection at speed still requires validation.
- The ADC driver initializes explicitly at startup. USB prints `[ADC] mode=IDF_GPIO_1 ready=1 ...`; `STATUS` includes `adc=IDF_GPIO_1 adc_ok=1 adc_err=0`, also shown on the Python app. Initialization or read fault disables auto/manual movement and stops active motors. Reset the ESP32 to recover from a latched ADC fault.
- Previous NVS white/black levels, per-sensor 40%-contrast thresholds, PID algorithms, ESP32 Classic SPP telemetry frame (72 bytes), app CSV and E-STOP are unchanged. Verify calibration levels in the live 16-eye view after reflash; recalibrate if any eye has shifted enough to cross its threshold.
- `ADC_BENCH` is still available unchanged for further comparison and continues using its own ARDUINO/4 baseline. Previously the benchmark printed 'firmware keeps production ADC=ARDUINO/4'; that final message now refers to the old mode and will be updated to the selected production mode.
- Measure USB `[PERF] adc_us` and `[DIAG] loop=...Hz` with BT disconnected, then with SPP at 100 Hz and AUTO (wheels raised). The theoretical ADC scan ceiling (~619 Hz) is not a demonstrated control-loop rate.


## SPP-only 300 Hz telemetry and full-session export (October 2026)
- USB serial initialization, USB command polling and scheduled diagnostic prints are disabled. Use Bluetooth SPP commands through the Python app. Firmware still uploads through the USB connector.
- Telemetry defaults to STREAM:300; supported STREAM:0/20/50/100/200/300. `micros()` scheduler targets 3333 us between frames; each binary frame remains 72 bytes. 300 frames/sec means 21,600 bytes/sec payload+framing (~172.8 kbps) before link overhead. The actual receive rate depends on SPP throughput and may be lower.
- Main loop only enqueues telemetry (12-frame bounded FreeRTOS queue). A Core-0 background task owns `BluetoothSerial.write`, which can block under congestion. `DIAG` reports loop/pid Hz, queued/sent rates and dropped/pending frames; `PERF` reports ADC/PID/COMM/TX timing and jitter on demand, never periodically. Click **Đo TX/Loop** in the app (prefer twice 5 seconds apart).
- Python app defaults to 300 Hz and shows **RX Hz** (actual frame decode in reader thread), **APP Hz** (samples processed in GUI), `q` (pending GUI queue), `dropUI`, and `CRC` errors. GUI screen refresh remains ~33 Hz; it is not the telemetry rate.
- **Xuất CSV** now exports the **entire app session**, not just the last 15,000 frames. **Xuất TXT** exports the entire terminal log since app startup or the last explicit **Xóa**. Both use auto-created temporary disk-backed session files; these are deleted when the app closes. **Ghi CSV** still writes an explicit persistent CSV file continuously for measurements that must survive crashes/app shutdown. Large exports take disk space and time.
- App history deques and the visible terminal remain bounded for UI responsiveness, but no longer limit session export. `ADC_BENCH` and calibration debug text are routed through SPP. No hardware measurement at 300 Hz has yet been reported; test with wheels raised.
