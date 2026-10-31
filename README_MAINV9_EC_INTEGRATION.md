# mainv9 — ECcodedoline logic adapted to doline17s

## Source and integration boundary

- Parent branch: `mainv7` (commit `a06a7333bed7552a2cf3ed7d9cdfcfb39903e5cb`).
- Sample: https://github.com/clb-robot-line-2026/ECcodedoline (`src/pid_controller.c`, `src/sensor.c`).
- Alternative sample sketch `Ultimate_line/` uses different motor drivers, pin mappings and blocking `delay()` loops. **Do not flash that sketch onto this hardware.**
- Only `src/pid_controller.cpp` and `tests/native/pid_stability_smoke.cpp` changed by the algorithm port. The existing SPP, CSV telemetry, NVS PID configuration, sensors/thresholds, MUX pinout, button handling, motor driver, encoder and Python application remain the mainv7 implementation.

## Behavior brought over from ECcodedoline

1. **Nonlinear P**: `P = Kp * (1 + 0.012 * abs(error)) * error`. Compared with mainv7's 0.018 slope, this is less aggressive near large offsets.
2. **Dynamic corner drag**: sample's `base - 0.55 * abs(error)`; the target is additionally bounded by a 45% guard that ramps between abs(error)=10 and 35. The normal PID `2*base` correction limit and motor peak limiter remain intact.
3. **Straight boost**: after 140 ms of continuous centered, contiguous line (1–4 eyes, abs(error)<=6), increase nominal Base by up to 20 PWM over 120 ms, capped at 215 unless configured Base already exceeds 215. Loss, junction, search or turn cancels the boost. This deliberately avoids immediate high-speed acceleration after reacquisition.
4. **D damping**: retains mainv7's time-normalized Kd (and therefore SPP `KD:x` semantics); instead of copying the sample's discrete unfiltered D, IIR becomes `0.2 * rawD + 0.8 * prevD` with abs(D) capped at min(120, 0.55 * configured Base). Raw-data ADC outliers cannot directly command a large D impulse.
5. **Qualified 90-degree turn**: if >=3 sensors on one outside edge are black and the opposite half is clear, the existing visible-line PID tracks until a real loss. After 40ms confirmed loss, the EC three-phase nonblocking profile applies: 22ms initial (left: -110/+35; right: +35/-110), then a bounded pivot (left: -105/+115; right: +115/-105), then **only after** 40ms stable centered reacquisition a short 18ms opposite counter-brake (+45/-45 or inverse). The existing 150ms stable settle and 2.5s recovery watchdog remain. Generic search remains +/-85 without active counter-braking.
6. **T-junction classification**: require >=2 center eyes, >=2 outside branch eyes, a nearly clear opposing half, stable 20ms. T-left outputs -0.4B/+B and T-right outputs +B/-0.4B, with 120ms cooldown. It is an automatic routing policy from the sample; verify it suits the course before driving at speed.
7. **Cross/turn cooldown**: 120ms after a wide cross-line and 80ms after a qualified sharp exit to avoid spurious edge reactivation.

## Deliberately not copied

- The sample's sweep-calibration, GPIO mapping, BLE and motor modules. Hardware is different and the mainv7 per-channel stationary white/black NVS calibration is already optimized.
- Sample's unfiltered derivative and unlimited reversal: these caused potentially dangerous PWM spikes in past logs.
- Sample's busy-wait loops and blocking delays.
- Sample's `Ultimate_line/Maze.ino` route-specific sequence. It is not a generic junction policy and depends on unrelated hardware.

## Verify and flash

```powershell
git fetch origin
git switch mainv9
git pull --ff-only origin mainv9
python -m platformio run -e esp32dev
python -m platformio run -e esp32dev -t upload
```

A native smoke test can be run on a host machine with g++:

```bash
g++ -std=c++17 -Wall -Wextra -Itests/native -Iinclude src/pid_controller.cpp tests/native/pid_stability_smoke.cpp -o /tmp/mainv9_pid_smoke
/tmp/mainv9_pid_smoke
```

Branch-specific CI: `.github/workflows/mainv9-native.yml`.

## Before evaluating speed

- On first run, keep the wheels raised and check motor direction during left/right sharp-turn phases.
- Send `PID` to read actual Kp/Ki/Kd/Base: the NVS profile can override compile-time defaults. The sample's Kd value must NOT be pasted blindly because the units differ.
- Compare the same course and calibrated threshold set with mainv7. Watch line-loss count, `abs(D)`, PWM sign flips, overshoot and corner completion rate.
- Treat 215 PWM straight cap and T-branch automatic turn as experiments, **not prevalidated optimum parameters**.

No app or telemetry protocol schema was changed.
