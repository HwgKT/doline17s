# mainHighSpeed — early braking and corner HOLD on the first straight (experimental)

Base branch: `linedutdoan`. The stable `mainv10` and the `linedutdoan` branches are not modified by this experiment.

## 1. Profile, relative encoder counts, safety

An experimental **PWM command envelope**, not a physical speed controller. The shared profile for all three maps:

| Encoder from AUTO (signed mean ticks) | Forward PWM target |
| --- | --- |
| 0 | current `BASE` |
| 0–300 | linear ramp `BASE → HS_PEAK` |
| 300–BRAKE_START | hold `HS_PEAK` |
| **1600–2300 when HS_PEAK=230–255** | linear early brake `HS_PEAK → min(110, HS_PEAK)` |
| **1800–2300 when HS_PEAK=100–229** | standard brake `HS_PEAK → min(110, HS_PEAK)` |
| >=2300 | `HOLD`: maintain forward target `min(110, HS_PEAK)` through first right turn |
| current encoder >=3100 **and** valid line position within ±15 of center | `COMPLETE`: return to original PID speed governor with `BASE` |

Defaults: `BASE=100` after boot; `HS_PEAK=220` (safer default based on the first-corner overshoot logs). Example with `BASE:120` and `HS_PEAK:220`: `120 → 220 → 110 (HOLD) → 120 when encoder and centered-line conditions are met`. Start experiments at `HS_PEAK:180`, then 220.

The HOLD forward target is **110 PWM when HS_PEAK >=110**, regardless of the configured BASE. At peaks 100–109, it is capped at HS_PEAK to prevent the BRAKE phase from accelerating. The physical PWM on each wheel can still be reduced or differentiated by tracking and steering controllers.

Default gains on firmware and in the app: `Kp=1.6`, `Ki=0.0`, `Kd=0.8`. The PID's error-dependent speed governor and outer-eye speed reduction still apply, therefore actual wheel PWM may differ from the forward target; sharp-turn/SEARCH motor commands retain their original higher turning authority. `HOLD` releases immediately when current encoder progress is >=3100 and a valid line measurement is within ±15 of center. A valid line sample requires at least one black sensor (`blackCount>0`); this prevents a no-line sample, whose remembered position can be near zero, from falsely releasing HOLD. No corner-detection flag, `lineLocked`, turn-state, outer-sensor, or consecutive-frame condition is added. A failed turn/search leaves the robot in HOLD or the existing STOP/search safety logic rather than automatically returning to BASE. In `GAP_BRIDGE`, the motor holds the **last governed forward PWM** equally on both wheels for at most 100ms, rather than using a new speed target while line is missing. This avoids introducing an acceleration step when there is no line feedback.

`runEnc = ((encL-encL0)+(encR-encR0))/2`, with origin taken at `AUTO`. No global encoder reset. An internal monotonic progress high-water mark prevents the acceleration ramp from restarting if wheels temporarily reverse. After an unexpected SEARCH before the brake end, the high-speed profile is marked `ABORTED` for the remainder of that AUTO run.

**The 1600 (for peaks >=230), 1800 (for peaks <230), 2300, and 3100 encoder ticks and HOLD PWM 110 are experimental settings.** Their braking performance has NOT been validated on hardware. Test with wheels lifted and physical clearance; use `HS_PEAK:180` first, then 220, then 255 only after successful runs. The finish line still requires all 16 sensors black for three consecutive controller samples.

## 2. Three track-specific dashed-line modes

| MAP | GAP encoder entry window | Max continuous loss |
| --- | --- | --- |
| 1 | 1500–2100 | 100ms |
| 2 | 1050–1750 | 100ms |
| 3 | 1000–1700 | 100ms |

All three MAP modes share the same peak-dependent brake: 1600–2300 for HS_PEAK >=230 and 1800–2300 for HS_PEAK <230; both hold at PWM 110 after 2300. MAP only changes the GAP detection window and log destination. Any timeout at 100ms still white enters existing `SEARCH` at ±85 PWM. Outside GAP windows, original 30ms DASHED_LINE / startup-coast / SEARCH logic applies.

## 3. Python app — selecting map, testing and saving logs

Environment: Windows; ESP32 Classic SPP connected through the outgoing COM port, Python packages from project requirements. Connect the ESP32 running the `mainHighSpeed` firmware (it must be flashed first).

1. From a checkout of `mainHighSpeed`, start `python robot_debug.py`.
2. Scan/select Bluetooth Classic SPP's **outgoing COM** port, press **Kết nối**; verify status and CAL profile (`CAL_DUMP` contains 16 valid eye thresholds). Calibrate WHITE then BLACK if required.
3. Before START, set Base PWM to 120 (optional) in the PID controls and press **Áp dụng PID**, or send `BASE:120` in Terminal and then `PID` to read back settings. Keep Kp/Ki/Kd unchanged unless conducting a separate controlled experiment.
4. Use the new **HS_PEAK — PWM đỉnh (100–255)** slider next to the PID controls. Select the desired value directly or click preset **180**, **220**, **255**; then click **Áp dụng HS**. The app sends `HS_PEAK:n` once and waits for the firmware readback, e.g. `HS_PEAK: PWM=180 BASE=120 RAMP_END=300 BRAKE_START=1800 BRAKE_END=2300 HOLD_PWM=110 HOLD_MIN_END=3100`. **Đọc HS** sends `HS_PEAK` and refreshes the slider from the firmware. The app auto-reads this value when connecting; the START button stays disabled until HS has been read and any unsaved slider changes have been applied or discarded with **Đọc HS**. Changing HS is disabled while AUTO is running and while a STOP acknowledgement is pending. Terminal commands `HS_PEAK:180`, `HS_PEAK:220`, `HS_PEAK:255` remain available for diagnostic use. The firmware reports the actual `BRAKE_START` for every readback and `brake=` in every HS_STAGE event, rather than showing a fixed 1800. It rejects a HS peak lower than the current BASE and rejects BASE settings above HS peak (`ERR: BASE_GT_HS_PEAK`).
5. Click exactly one of the three **MAP 1**, **MAP 2**, **MAP 3** buttons. MAP changes are disabled during a run; START requires MAP selection.
6. Put the robot in the same START position for each attempt. Click **BẮT ĐẦU DÒ LINE**. The app creates a CSV+TXT in the selected map folder **before** transmitting `AUTO:1`, `AUTO:2`, or `AUTO:3` to ESP32.
7. Observe `HS: RAMP/CRUISE/BRAKE/HOLD/COMPLETE/ABORTED`, the relative encoder from periodic DIAG, wheel PWM, black/white line signals and SPP receive frequency. Stop manually via **DỪNG** if needed. Do not treat absence of STOP acknowledgment as proof the motors are off; use E-STOP / physical safety procedure.
8. Wait for the text `ESP32 ĐÃ XÁC NHẬN DỪNG` and `CSV DA DONG` before the next run. Files are automatically written at:

```text
logs/map_01/linerobot_map01_YYYYMMDD_HHMMSS_xxxxxx.csv
logs/map_01/linerobot_map01_YYYYMMDD_HHMMSS_xxxxxx.txt
logs/map_02/...
logs/map_03/...
```

No need to use **Xuất CSV/TXT** for the automatic file; those controls are optional copies.

- CSV: existing 16 ADC readings, calibration/logic, absolute encL/encR, PWM L/R, error/P/I/D and mode. No changes to the 72-byte binary telemetry format.
- TXT: exact `RUN_ORIGIN map=... encL0=... encR0=...` for encoder-zero alignment, plus `HS_STAGE RAMP/CRUISE/BRAKE/HOLD/COMPLETE/ABORTED` and `GAP_START/GAP_RECOVERED/GAP_TIMEOUT_SEARCH`.
- DIAG: contains `map`, `runEnc`, `hs`, `peak`, and loop/PID/SPP rates. The app has separate HS peak **requested/confirmed** displays, plus `RAMP/CRUISE/BRAKE/HOLD/COMPLETE` tracking. Firmware responds to `HS_PEAK` automatically at Bluetooth connection; writes are never assumed successful before readback.
- For exact CSV-aligned encoder use the values in TXT: `E=((encL-encL0)+(encR-encR0))/2` rather than treating the first logged CSV row as necessarily zero.

## 4. Validation commands

Native test focused on GAP and the trapezoid:
```sh
g++ -std=c++17 -Wall -Wextra -Itests/native -Iinclude src/pid_controller.cpp tests/native/gap_bridge_smoke.cpp -o gap_bridge_smoke
./gap_bridge_smoke
```

Headless Python command/response tests (requires pyserial):
```sh
python -m unittest discover -s tests -p test_hs_peak_app.py
```

ESP32 build:
```sh
pio run -e esp32dev
```

These commands are provided for independent validation; no successful build or hardware execution is claimed here.

## 5. Test protocol / comparison

Keep the same MAP, surface, battery state, calibration and PID parameters across tests. Collect at least three runs per peak setting. Start with `HS_PEAK:180`, then 220, then 255.

Review encoder speed near runEnc=2300/2550/2600, time to `HS_STAGE HOLD` and `COMPLETE`, duration spent in GAP, confirmed right-corner re-acquisition, unexpected SEARCH, negative PWM/reverse steering, maximum line error, and physical 90-degree corner success. Reject any peak that repeatedly overshoots the first turn; `HS_STAGE HOLD` is not physical proof of deceleration. The earliest large-error region in historical logs is around 2530–2580 ticks; a SEARCH pivot usually begins around 2600–2640 ticks.

**MAP 1 caveat:** its GAP window (1500–2100) overlaps the early BRAKE. While `GAP_BRIDGE` is active, the firmware holds the last forward PWM; the requested linear ramp resumes after line reacquisition. Starting BRAKE earlier does not guarantee immediate mechanical deceleration during white gaps.


## GPIO34 physical START (ported from mainv10 button sampling)

On this branch GPIO34 is the sole physical START button (not CALIB). GPIO35 is unused. ESP32 GPIO34 is input-only and **has no internal pull-up or pull-down**: use a 10-kΩ external pulldown from GPIO34 to GND and a momentary button from 3.3 V to GPIO34 (active HIGH). Never apply 5 V. Debounce is 50 ms with one rising-edge event per press; holding the button or bouncing it does not retrigger AUTO. A button already held during power-up must be released and pressed again.

The button calls the same firmware AUTO handler as SPP, so calibration, E-STOP, ADC, and running-state guards remain in force. The firmware defaults to MAP 1 after boot for standalone operation. For MAP 2/3, connect the Python app and select MAP 2/3 **before** pressing GPIO34; selection sends MAP:2 / MAP:3 to the ESP32. The physical button does **not** recalibrate the sensors. Calibration remains available through CAL_WHITE and CAL_BLACK commands in the app.

While connected, the app sees the button's MODE: AUTO PID and RUN_ORIGIN map=N responses and opens map-specific CSV/TXT automatically if no run log was already created by the app. A physical START without SPP can still run with NVS calibration but cannot produce PC-side CSV/TXT; a connected app should first finish reading CAL_DUMP so it has all 16 calibrated thresholds for the log. A run started from the physical button cannot be started twice while AUTO is active; STOP remains an independent app/SPP command.
