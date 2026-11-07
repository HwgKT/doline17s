# Map 01 Route Lock — first 2,000 relative encoder ticks

Implementation: branch `mainHighSpeedv2`, from ten short MAP 01 runs dated 2026-11-07.
Assumption: **every run starts at the same physical START position and heading**.
The logs reach only ~1,700–2,000 ticks; they do not prove the complete map
route, and encoder drift while one wheel reverses is real. This is an
experimental first-segment supervisor; test with the robot lifted first.

## State machine

| Stage | Approx. ticks | Conditions to advance | Route action |
| --- | ---: | --- | --- |
| STRAIGHT_1 | 0–250 | >=250 ticks and confirmed narrow line | Normal PID, BASE cap 130 after 200 |
| SEMICIRCLE | 250–1,300 | Saw left apex: position <=-30 in 900–1,280; then rightward line >=+15 after 1,220 | PID follows one contiguous IR cluster, BASE cap 115, 100 at 500–800, 110 after 800 |
| RIGHT_TURN | ~1,300–1,600 | >=1,540 ticks; abs(position)<=12 continuously >=40ms with valid PID line | PID right turn, BASE cap 110 |
| STRAIGHT_2 | after ~1,600 | End of route-locked segment | Release early speed caps, normal PID through finish; HS_PEAK boost at 10k unchanged |

The phase 600–700 from logs was a *rightward SEARCH* during the semicircle,
not a separate mandatory intersection turn. Only while the physical sensor
mask is all WHITE and SEARCH is already active in [540,800] does route lock
force SEARCH_RIGHT with PWM (85,-85); there is **no blind pivot by encoder**.
A search beginning in that window and lasting >=250ms stops AUTO (code 1),
even when encoder drift carries progress past 800 or a single noisy sensor
frame occurs during SEARCH.

Other safety stop codes:
- code 2: cannot confirm arc / subsequent rightward cue by tick 1,680
- code 3: cannot confirm centered straight after right by tick 1,920

At split line patterns (two or more separate clusters), `pid_calculate_position`
chooses only one contiguous group using continuity with prior line position
and a phase-dependent bias. The raw sensor mask remains available for START /
FINISH detection; the algorithm does NOT change calibration or polarity.

## Bluetooth Classic SPP

- `ROUTE_LOCK` — read current setting, map, stage and fault code
- `ROUTE_LOCK:1` — enable Map 01 lock for a fixed start pose (default)
- `ROUTE_LOCK:0` — disable lock for general tests or starts elsewhere
- Settings may change **only when stopped**
- `ROUTE_STAGE STRAIGHT_1/SEMICIRCLE/RIGHT_TURN/STRAIGHT_2 map=1 runEnc=...`
- `ROUTE_FAULT code=... map=1 runEnc=...`
- `AUTO_STOPPED: ROUTE_FAULT map=1 runEnc=... route_fault=...`
- `DIAG` includes `route=<0..5> rf=<0..3> clu=<hex>`

The Python app shows a dedicated ROUTE line in the diagnostics area, logs
each transition in the MAP 01 TXT, and explains route faults. The old
`HS_PEAK (10k+)` and `Reset ENC` controls are unchanged. MAP 2/3 use
their previous PID-only mode.

## Verification

Pure FSM/cluster native regression (no Arduino required):

```bash
g++ -std=c++17 -Wall -Wextra -Werror -Iinclude \
  tests/native/map1_route_lock_logic_smoke.cpp -o /tmp/map1_route_lock_logic_smoke
/tmp/map1_route_lock_logic_smoke
```

Existing firmware-dependent native regressions:

```bash
g++ -std=c++17 -Itests/native -Iinclude \
  src/pid_controller.cpp tests/native/black_pad_start_finish_smoke.cpp -o /tmp/black_start
/tmp/black_start
g++ -std=c++17 -Itests/native -Iinclude \
  src/pid_controller.cpp tests/native/map01_late_base_smoke.cpp -o /tmp/late_base
/tmp/late_base
```

Firmware (ESP32):

```bash
pio run -e esp32dev
pio run -e esp32dev -t upload
python robot_debug.py
```

Recommended first test: BASE=100–115; HS_PEAK<=180; repeat ten short
controlled starts from the exact same mark. Compare the `ROUTE_STAGE`
transitions, encoder positions and duration of first SEARCH before raising
speed. Route lock must not be assumed safe at a different initial pose.
