# mainHighSpeed — trapezoidal PWM on the first straight (experimental)

Base branch: `linedutdoan`. The stable `mainv10` and the `linedutdoan` branches are not modified by this experiment.

## 1. Profile, relative encoder counts, safety

An experimental **PWM command envelope**, not a physical speed controller. The shared profile for all three maps:

| Encoder from AUTO (signed mean ticks) | Forward PWM target |
| --- | --- |
| 0 | current `BASE` |
| 0–300 | linear ramp `BASE → HS_PEAK` |
| 300–2050 | hold `HS_PEAK` |
| 2050–2450 | linear ramp `HS_PEAK → BASE` |
| >=2450 | return to original PID speed governor with `BASE` |

Defaults: `BASE=100` after boot; `HS_PEAK=255`. To reproduce `120 → 255 → 120`, first set `BASE:120` on the app, then set `HS_PEAK:255`.

The PID's error-dependent speed governor and outer-eye speed reduction still apply, therefore actual motor PWM may be below the target. The existing steering correction, sharp-turn controller and SEARCH algorithm remain unchanged. In `GAP_BRIDGE`, the motor holds the **last governed forward PWM** equally on both wheels for at most 100ms, rather than using a new speed target while line is missing. This avoids introducing an acceleration step when there is no line feedback.

`runEnc = ((encL-encL0)+(encR-encR0))/2`, with origin taken at `AUTO`. No global encoder reset. An internal monotonic progress high-water mark prevents the acceleration ramp from restarting if wheels temporarily reverse. After an unexpected SEARCH before the brake end, the high-speed profile is marked `ABORTED` for the remainder of that AUTO run.

**The 2050 and 2450 ticks are trial values.** Their braking distance has NOT been validated at 255 PWM. Test with wheels lifted and physical clearance; use `HS_PEAK:180` first, then 220, then 255 after successful runs. The finish line still requires all 16 sensors black for three consecutive controller samples.

## 2. Three track-specific dashed-line modes

| MAP | GAP encoder entry window | Max continuous loss |
| --- | --- | --- |
| 1 | 1500–2100 | 100ms |
| 2 | 1050–1750 | 100ms |
| 3 | 1000–1700 | 100ms |

All three MAP modes share the same trapezoid and brake start at 2050. MAP only changes the GAP detection window and log destination. Any timeout at 100ms still white enters existing `SEARCH` at ±85 PWM. Outside GAP windows, original 30ms DASHED_LINE / startup-coast / SEARCH logic applies.

## 3. Python app — selecting map, testing and saving logs

Environment: Windows; ESP32 Classic SPP connected through the outgoing COM port, Python packages from project requirements. Connect the ESP32 running the `mainHighSpeed` firmware (it must be flashed first).

1. From a checkout of `mainHighSpeed`, start `python robot_debug.py`.
2. Scan/select Bluetooth Classic SPP's **outgoing COM** port, press **Kết nối**; verify status and CAL profile (`CAL_DUMP` contains 16 valid eye thresholds). Calibrate WHITE then BLACK if required.
3. Before START, set Base PWM to 120 (optional) in the PID controls and press **Áp dụng PID**, or send `BASE:120` in Terminal and then `PID` to read back settings. Keep Kp/Ki/Kd unchanged unless conducting a separate controlled experiment.
4. For first tests, use Terminal to send `HS_PEAK:180`; the device responds `HS_PEAK: PWM=180 RAMP_END=300 BRAKE_START=2050 BRAKE_END=2450`. Repeat with `HS_PEAK:220` and eventually `HS_PEAK:255` only while STOPPED. `HS_PEAK` reads current config. The app's live HS bar shows the confirmed peak.
5. Click exactly one of the three **MAP 1**, **MAP 2**, **MAP 3** buttons. MAP changes are disabled during a run; START requires MAP selection.
6. Put the robot in the same START position for each attempt. Click **BẮT ĐẦU DÒ LINE**. The app creates a CSV+TXT in the selected map folder **before** transmitting `AUTO:1`, `AUTO:2`, or `AUTO:3` to ESP32.
7. Observe `HS: RAMP/CRUISE/BRAKE/COMPLETE/ABORTED`, the relative encoder from periodic DIAG, wheel PWM, black/white line signals and SPP receive frequency. Stop manually via **DỪNG** if needed. Do not treat absence of STOP acknowledgment as proof the motors are off; use E-STOP / physical safety procedure.
8. Wait for the text `ESP32 ĐÃ XÁC NHẬN DỪNG` and `CSV DA DONG` before the next run. Files are automatically written at:

```text
logs/map_01/linerobot_map01_YYYYMMDD_HHMMSS_xxxxxx.csv
logs/map_01/linerobot_map01_YYYYMMDD_HHMMSS_xxxxxx.txt
logs/map_02/...
logs/map_03/...
```

No need to use **Xuất CSV/TXT** for the automatic file; those controls are optional copies.

- CSV: existing 16 ADC readings, calibration/logic, absolute encL/encR, PWM L/R, error/P/I/D and mode. No changes to the 72-byte binary telemetry format.
- TXT: exact `RUN_ORIGIN map=... encL0=... encR0=...` for encoder-zero alignment, plus `HS_STAGE RAMP/CRUISE/BRAKE/COMPLETE/ABORTED` and `GAP_START/GAP_RECOVERED/GAP_TIMEOUT_SEARCH`.
- DIAG: contains `map`, `runEnc`, `hs`, `peak`, and loop/PID/SPP rates.
- For exact CSV-aligned encoder use the values in TXT: `E=((encL-encL0)+(encR-encR0))/2` rather than treating the first logged CSV row as necessarily zero.

## 4. Validation commands

Native test focused on GAP and the trapezoid:
```sh
g++ -std=c++17 -Wall -Wextra -Itests/native -Iinclude src/pid_controller.cpp tests/native/gap_bridge_smoke.cpp -o gap_bridge_smoke
./gap_bridge_smoke
```

ESP32 build:
```sh
pio run -e esp32dev
```

These commands are provided for independent validation; no successful build or hardware execution is claimed here.

## 5. Test protocol / comparison

Keep the same MAP, surface, battery state, calibration and PID parameters across tests. Collect at least three runs per peak setting. Start with `HS_PEAK:180`, then 220, then 255.

Review time to reach `runEnc=2450`, duration spent in GAP, successful recoveries, unexpected SEARCH, negative PWM/reverse steering, maximum absolute line error, measured encoder rate (ticks/s), and whether the robot really negotiates the first corner. Reject any PWM setting that cannot brake to normal PID before the first corner. The earliest large-error region in historical logs is around 2530–2580 ticks; a SEARCH pivot usually begins around 2600–2640 ticks.
