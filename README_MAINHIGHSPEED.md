# mainHighSpeedv2 — PID, Map 01 Route Lock and HS_PEAK

> Map 01 now has an aligned-START route-lock FSM for the first ~2,000
> encoder ticks. See [Map 01 Route Lock](README_MAP1_ROUTE_LOCK.md)
> for the state sequence, fault codes and SPP controls.
> Map 02/03, the 10k late-BASE boost and full-black FINISH are preserved.

> **mainHighSpeedv2 – MAP 01 late-BASE profile (2026-11-07):**
> The old first-straight RAMP/CRUISE/BRAKE/HOLD and GAP_BRIDGE windows remain
> disabled by default. MAP 01 now has a separate BASE scheduler, while normal
> PID steering, the error/outer-eye speed governor, SEARCH and all-black
> START/FINISH logic remain unchanged.
>
> - 4,800–6,900 relative encoder ticks: cap BASE at 115 PWM and each
>   NORMAL_PID wheel command to 185 PWM; 5,600–6,200 caps BASE at 105
>   and each NORMAL_PID wheel command at 170 PWM. After SEARCH in that
>   cluster, cap forward PWM at 105 for 80 ms. SEARCH remains ±85 PWM.
> - From 10,000 ticks, the Python HS_PEAK slider sets the **target BASE**.
>   Activate only after a confirmed line remains centered (abs(error) <= 20)
>   for 40 ms and at least 80 ms have passed since SEARCH/turn. A valid
>   line means >=2 adjacent black sensors, or exactly one center eye with
>   abs(error) <= 15. Ramp +250 PWM/s to the target; if line is lost, the
>   sensor pattern is invalid, or abs(error) >= 40, cancel BOOST and
>   ramp down at -500 PWM/s.
> - **No fixed speed reduction near the finish**: BOOST may continue
>   past 16,000 ticks until the original full-black FINISH detector stops
>   the motors. This changes neither the START/FINISH detectors nor SEARCH.
> - MAP 2/3 remain in PID-only mode by default.
>
> Firmware readback: `HS_PEAK: PWM=... START_ENC=10000 MODE=MAP1_LATE_BASE`.
> Live transitions: `HS_STAGE WAIT_LINE` and `HS_STAGE BOOST`.
> The Python app preserves the same `HS_PEAK:n` command/ACK protocol.
> 
> **Restore later:** Add `-DDOLINE_ENABLE_ENCODER_SPEED_PROFILE=1` to
> `build_flags` of the esp32dev environment in `platformio.ini` and rebuild.
> Without this flag (the default), encoder-based changes stay off.
> The original `mainHighSpeed` branch is not modified.
>
> Old high-speed/GAP native tests can be run with the flag enabled, e.g.:
> `g++ -std=c++17 -DDOLINE_ENABLE_ENCODER_SPEED_PROFILE=1 -Itests/native -Iinclude src/pid_controller.cpp tests/native/gap_bridge_smoke.cpp -o gap_bridge_smoke`


Base branch: `linedutdoan`. The stable `mainv10` and the `linedutdoan` branches are not modified by this experiment.

## 1. Profile, relative encoder counts, safety

An experimental **PWM command envelope**, not a physical speed controller. The shared profile for all three maps:

| Encoder from AUTO (signed mean ticks) | Forward PWM target |
| --- | --- |
| 0 | current `BASE` |
| 0–300 | linear ramp `BASE → HS_PEAK` |
| 300–BRAKE_START | hold `HS_PEAK` |
| **1400–2300 when HS_PEAK=240–255** | linear very-early brake `HS_PEAK → min(110, HS_PEAK)` |
| **1600–2300 when HS_PEAK=230–239** | linear early brake `HS_PEAK → min(110, HS_PEAK)` |
| **1800–2300 when HS_PEAK=100–229** | standard brake `HS_PEAK → min(110, HS_PEAK)` |
| >=2300 | `HOLD`: maintain forward target `min(110, HS_PEAK)` through first right turn |
| current encoder >=3100 **and** valid line position within ±15 of center | `COMPLETE`: return to original PID speed governor with `BASE` |

**Built-in MAP defaults (no Python app required):** MAP 1: `BASE=130`, `HS_PEAK=240`; MAP 2: `BASE=160`, `HS_PEAK=240`; MAP 3: `BASE=150`, `HS_PEAK=240`. Firmware boots into MAP 1. The first switch to MAP 2 or 3 loads that map's defaults; selecting the same map again preserves deliberate app overrides. All maps use `Kp=1.6`, `Ki=0.0`, `Kd=0.8`. HOLD remains 110 PWM.

The HOLD forward target is **110 PWM when HS_PEAK >=110**, regardless of the configured BASE. At peaks 100–109, it is capped at HS_PEAK to prevent the BRAKE phase from accelerating. The physical PWM on each wheel can still be reduced or differentiated by tracking and steering controllers.

Default gains on firmware and in the app: `Kp=1.6`, `Ki=0.0`, `Kd=0.8`. The PID's error-dependent speed governor and outer-eye speed reduction still apply, therefore actual wheel PWM may differ from the forward target; sharp-turn/SEARCH motor commands retain their original higher turning authority. `HOLD` releases immediately when current encoder progress is >=3100 and a valid line measurement is within ±15 of center. A valid line sample requires at least one black sensor (`blackCount>0`); this prevents a no-line sample, whose remembered position can be near zero, from falsely releasing HOLD. No corner-detection flag, `lineLocked`, turn-state, outer-sensor, or consecutive-frame condition is added. A failed turn/search leaves the robot in HOLD or the existing STOP/search safety logic rather than automatically returning to BASE. In `GAP_BRIDGE`, the motor holds the **last governed forward PWM** equally on both wheels for at most 100ms, rather than using a new speed target while line is missing. This avoids introducing an acceleration step when there is no line feedback.

`runEnc = ((encL-encL0)+(encR-encR0))/2`, with origin taken at `AUTO`. No global encoder reset. An internal monotonic progress high-water mark prevents the acceleration ramp from restarting if wheels temporarily reverse. After an unexpected SEARCH before the brake end, the high-speed profile is marked `ABORTED` for the remainder of that AUTO run.

**The 1400 (peaks 240–255), 1600 (peaks 230–239), 1800 (peaks 100–229), 2300, and 3100 encoder ticks and HOLD PWM 110 are experimental settings.** Their braking performance has NOT been validated on hardware. Test with wheels lifted and physical clearance; use `HS_PEAK:180` first, then 220, then 255 only after successful runs. The finish line still requires all 16 sensors black for three consecutive controller samples.

## 2. Three track-specific dashed-line modes

| MAP | GAP encoder entry window | Max continuous loss |
| --- | --- | --- |
| 1 | 1500–2100 | 100ms |
| 2 | 1050–1750 | 100ms |
| 3 | 1000–1700 | 100ms |

All three MAP modes share the same peak-dependent brake: 1400–2300 for HS_PEAK 240–255, 1600–2300 for 230–239, and 1800–2300 for 100–229; all hold at PWM 110 after 2300. MAP only changes the GAP detection window and log destination. Any timeout at 100ms still white enters existing `SEARCH` at ±85 PWM. Outside GAP windows, original 30ms DASHED_LINE / startup-coast / SEARCH logic applies.

## 3. Python app — selecting map, testing and saving logs

Environment: Windows; ESP32 Classic SPP connected through the outgoing COM port, Python packages from project requirements. Connect the ESP32 running the `mainHighSpeed` firmware (it must be flashed first).

1. From a checkout of `mainHighSpeed`, start `python robot_debug.py`.
2. Scan/select Bluetooth Classic SPP's **outgoing COM** port, press **Kết nối**; verify status and CAL profile (`CAL_DUMP` contains 16 valid eye thresholds). Calibrate WHITE then BLACK if required.
3. Before START, set Base PWM to 120 (optional) in the PID controls and press **Áp dụng PID**, or send `BASE:120` in Terminal and then `PID` to read back settings. Keep Kp/Ki/Kd unchanged unless conducting a separate controlled experiment.
4. Use the new **HS_PEAK — PWM đỉnh (100–255)** slider next to the PID controls. Select the desired value directly or click preset **180**, **220**, **255**; then click **Áp dụng HS**. The app sends `HS_PEAK:n` once and waits for the firmware readback, e.g. `HS_PEAK: PWM=180 BASE=120 RAMP_END=300 BRAKE_START=1800 BRAKE_END=2300 HOLD_PWM=110 HOLD_MIN_END=3100`. **Đọc HS** sends `HS_PEAK` and refreshes the slider from the firmware. The app auto-reads this value when connecting; the START button stays disabled until HS has been read and any unsaved slider changes have been applied or discarded with **Đọc HS**. Changing HS is disabled while AUTO is running and while a STOP acknowledgement is pending. Terminal commands `HS_PEAK:180`, `HS_PEAK:220`, `HS_PEAK:255` remain available for diagnostic use. The firmware reports the actual `BRAKE_START` for every readback and `brake=` in every HS_STAGE event, rather than showing a fixed 1800. It rejects a HS peak lower than the current BASE and rejects BASE settings above HS peak (`ERR: BASE_GT_HS_PEAK`).
5. Click exactly one of the three **MAP 1**, **MAP 2**, **MAP 3** buttons. MAP changes are disabled during a run; START requires MAP selection.
6. Put the robot in the same START position for each attempt. Click **BẮT ĐẦU DÒ LINE**. The app creates a CSV+TXT in the selected map folder **before** transmitting `AUTO:1`, `AUTO:2`, or `AUTO:3` to ESP32.
### MAP 1/2/3 — independent eight-stage encoder/BASE profiles

Open **BASE từng đoạn MAP 1/2/3**, choose a map, and edit its seven encoder
end-tick thresholds (X1..X7) and eight BASE PWM targets (B1..B8). Each map
has its own independent profile in ESP32 RAM. Stage starts are derived from
the previous end boundary; stage 8 runs from X7 to FINISH. Tick boundaries
must be strictly increasing in 1..10,000,000; BASE values must be 45..255.

SPP read: `MAP_SPEED_PROFILE:<mapId>`.
SPP write: `MAP_SPEED_PROFILE:<mapId>,X1,...,X7,B1,...,B8`.
Writes are accepted only when the robot is stopped and the requested map is
selected. The GUI reads the selected map back from ESP32 before editing and
waits for a matching readback after applying a profile.

Defaults preserve current behavior: MAP 1 retains its existing eight BASE
values; MAP 2 starts with BASE 160 in all eight stages; MAP 3 starts with BASE
150 in all eight stages. MAP 2/3 tick boundaries are provisional common
defaults and should be calibrated from encoder logs before assigning different
segment speeds. Profiles are runtime RAM settings and reset to firmware
defaults after ESP32 reboot. Legacy Map 1 SPP commands remain supported.

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


## Start from any surface -> find line -> all-black FINISH (mainHighSpeedv2)

The robot now **accepts START on WHITE, a narrow BLACK line, or a full
16/16-BLACK pad**, from Python (`AUTO:1/2/3`), terminal (`START`) or
the GPIO34 physical button. The old `ERR: START_REQUIRES_ALL_BLACK_16`
restriction has been removed. Calibration, ADC-readiness, E-STOP and
other AUTO interlocks are unchanged.

FINISH is initially **disarmed**. If starting on 16/16-BLACK, the robot
drives straight at the selected BASE; broad patterns with 11..15 black
sensors use the existing straight CROSS_LINE speed. On WHITE or
an unconfirmed narrow/off-center line, it coasts straight at up to 95 PWM
without PID steering or SEARCH. A narrow centered line must persist for
30 ms (1..10 black eyes, >=1 central eye S6..S9, position within +/-22);
only then does normal PID steering begin and FINISH arm. The transition
resets derivative history. If no centered line is confirmed within
5 seconds after first seeing a non-full-black pattern, AUTO safely stops.
It logs `FINISH_ARMED: LINE_CONFIRMED` over SPP. Thereafter **16/16 BLACK**
for 3 consecutive controller samples stops the robot with
`AUTO_FINISHED: FULL_BLACK_3`. No encoder distance is used to arm FINISH.

**Physical caution:** The controller cuts PWM upon detecting the destination;
it does not actively brake to a precise location, so the robot can coast
further due to inertia. Any intermediate all-black square after FINISH has
armed can also be mistaken for the destination; use a distinct marker or
additional conditions if the track has such squares.

Native behavior test:
```sh
g++ -std=c++17 -Wall -Wextra -Itests/native -Iinclude src/pid_controller.cpp tests/native/black_pad_start_finish_smoke.cpp -o black_pad_start_finish_smoke
./black_pad_start_finish_smoke
```

**Reset encoder in the GUI:** The `Reset ENC` button is now in the encoder
metrics header above the Scope, so it remains visible on smaller screens.
It sends `ENC_RESET` and waits for the `ENC_RESET: OK` acknowledgement
before resetting the app's speed baseline. The firmware rejects resets
while AUTO or motor movement is active with
`ERR: ENC_RESET_REQUIRES_STOPPED`, protecting `runEnc` and the 10k BOOST
threshold. After a successful reset the app requests `ENC` for readback.

## GPIO34 three-press CAL / START workflow

The **single physical button on GPIO34** follows a deterministic sequence after every ESP32 boot, even when a valid NVS calibration profile already exists:

1. **First press — CAL_WHITE:** Place all 16 eyes on uniform WHITE before pressing. The firmware stops the motors, captures 128 ADC frames, and blinks the onboard GPIO2 LED **rapidly (40 ms per toggle) during the 128-frame acquisition only**. At `CAL_WHITE_DONE`, GPIO2 turns **OFF** and stays off until the second press.
2. **Second press — CAL_BLACK:** Place all 16 eyes on uniform BLACK, then press. The firmware captures 128 ADC frames and blinks GPIO2 **more slowly (90 ms per toggle) during the 128-frame acquisition only**. At `CAL_OK_SAVED` (NVS profile stored), GPIO2 turns **OFF** and stays off until START.
3. **Third press — START:** The robot may start on WHITE, a narrow LINE, or a full-black START square. AUTO still checks ADC, calibration, E-STOP and running-state interlocks. On successful START, GPIO2 resumes its normal Bluetooth indication. FINISH remains disarmed until the robot has confirmed a narrow centered line for 30 ms; if no line appears within 5 seconds after non-full-black is seen, AUTO stops.

Pressing again after a completed run starts another run without needing to recalibrate. To restart calibration manually, send `CAL_WHITE` through the app; a successful `CAL_WHITE` resets the physical button sequence to BLACK next, and `CAL_BLACK` advances to START only after `CAL_OK_SAVED`. If black calibration fails, the physical button sequence returns to WHITE first. A premature press during the 128-frame acquisition is refused and does not advance the stage.

GPIO34 is input-only with **no internal pull-up/pull-down**. Use a **10-kΩ external pullup to 3.3 V** and a button connecting GPIO34 to GND (active LOW); never apply 5 V. The button is debounced for 50 ms and only one press event is produced per press. GPIO35 remains unused.

The LED is shared with Bluetooth status: while waiting for each calibration press and after each completed calibration, GPIO2 is forced **OFF even if SPP is connected**. It blinks only during WHITE/BLACK sample acquisition. After START, the normal Bluetooth LED status returns (solid with SPP connection, normal blink otherwise). Since calibration takes roughly 0.2 s at 650 Hz, the previous 120/450 ms toggle intervals were too slow for useful capture feedback; the faster 40/90 ms toggle intervals are LED-only settings and **do not alter ADC sample timing**.

The firmware defaults to MAP 1 at boot. To use MAP 2 or MAP 3, connect the Python app and select MAP 2/3 before physical START; it sends `MAP:2` / `MAP:3`. Physical START still sends `BUTTON34: START_REQUEST`, `MODE: AUTO PID`, and `RUN_ORIGIN map=N` through SPP. When the app is connected and has loaded `CAL_DUMP`, it automatically saves CSV/TXT into the matching `logs/map_NN` folder. Without SPP, the robot can start independently, but no PC log is recorded.

The existing STOP rule (16 black sensors for three consecutive controller samples), HS_PEAK BRAKE windows (1400 / 1600 / 1800 to 2300), HOLD PWM 110 and default PID gains remain unchanged.


### Independent PID sensor configuration per MAP

Each MAP has independent T1/T2 thresholds and three 16-bit PID-only masks.
MAP 1 defaults to T1=650, T2=1350; masks FFC0/03FF/FFFF.
MAP 2/3 default to T1=650, T2=1350; masks FFFF/FFFF/FFFF.
The speed Tick/BASE profiles also remain independently stored per MAP.

Read: `MAP_SENSOR_PROFILE:<mapId>`.
Write: `MAP_SENSOR_PROFILE:<mapId>,T1,T2,M1,M2,M3` (hex masks).
Writes require a stopped robot and the specified MAP selected.
The legacy `SENSOR_PROFILE` command still addresses MAP 1.
Only PID sensor position uses masks; ADC, corner detection, SEARCH, START
and FINISH continue using unmasked sensor data.
Profiles reside in ESP32 RAM and revert to defaults after reboot.


## Variable-stage profiles per MAP (SPP V2)

Each of MAP 1, MAP 2 and MAP 3 stores its own sensor-mask profile and its own
BASE speed profile. Both editors expose a stage-count input from 1 to 8. The
last stage always runs to FINISH; therefore only the first N-1 rows need an
editable end-tick boundary. Tick boundaries must be strictly increasing and
within 1..10,000,000.

### PID sensor configuration

Each stage contains one encoder end-tick boundary (except the final FINISH
stage) and 16 independent sensor checkboxes S0..S15. The mask only filters the
position measurement supplied to NORMAL PID. ADC acquisition, physical line
classification, sharp-turn/SEARCH, START and FINISH continue using the full
sensor set. MAP 1 boots with its previous three-stage profile (650, 1350 ticks;
FFC0, 03FF, FFFF masks). MAP 2/3 boot with three all-sensor stages.

Read: `MAP_SENSOR_PROFILE_V2:<mapId>`.
Write: `MAP_SENSOR_PROFILE_V2:<mapId>,<count>,<endTick1>,<mask1Hex>,...,0,<maskNHex>`.
Masks are four-digit hexadecimal values; the last stage end-tick is the
literal zero sentinel meaning FINISH.

### BASE speed configuration

Each stage displays From Tick, To Tick and BASE PWM. The final row's To Tick
is FINISH. The **BASE chung** input plus **BASE chung → tất cả** copies one
validated PWM value (45..255) into every stage; the normal Apply action then
sends the whole profile to the selected MAP.

Read: `MAP_SPEED_PROFILE_V2:<mapId>`.
Write: `MAP_SPEED_PROFILE_V2:<mapId>,<count>,<endTick1>,<base1>,...,0,<baseN>`.
The final end-tick is the literal zero sentinel meaning FINISH.

Both V2 writes require AUTO stopped and the requested MAP selected. The GUI
reads profiles back from the ESP32 and requires the active MAP's sensor and
BASE profiles to be confirmed before GUI START. Profiles are runtime RAM
settings and revert to firmware defaults after ESP32 reboot. The older fixed
three-stage sensor and fixed eight-stage BASE SPP commands remain for legacy
clients; the GUI uses V2.
