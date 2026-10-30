# Physical buttons and feedback LED — mainv4

## Pin mapping
- GPIO34: CALIB (external pull-down, pressed = HIGH).
- GPIO35: START/STOP (external pull-down, pressed = HIGH).
- GPIO16: feedback LED (active-high, assumed).
- GPIO2: Bluetooth SPP status LED, unchanged.

GPIO34 and GPIO35 are input-only and do not support internal pull-ups or pull-downs. Confirm actual schematic and wire external resistors before flashing. GPIO16 must not already be occupied by another peripheral.

## Sequence
1. Boot: restore NVS profile if valid. Button LED initially OFF.
2. GPIO34 first click: stop motors, capture WHITE (128 frames). After CAL_WHITE_DONE, LED blinks once.
3. GPIO34 second click: capture BLACK, validate/store NVS. After CAL_OK_SAVED, LED blinks three times. If capture fails, no success blink.
4. GPIO35 click: if ADC + calibration valid and no E-STOP, enable AUTO PID, blink LED three times.
5. GPIO35 next click: stop PID and motor, blink LED twice.
6. Repeat start/stop. To recalibrate, press GPIO34 while stopped; a completed WHITE must precede BLACK.

The button driver uses 35-ms debounce, events on stable press only, and LED 100-ms ON/OFF pulses via millis() with no blocking delay(). It is independent of the CSV local START/END markers in robot_debug.py (keyboard 1/0). Button START itself does not automatically open the Python CSV; only app START begins a CSV recording session.

Bluetooth disconnection stops manual timed driving, but does not interrupt an AUTO run started physically. Use GPIO35 to stop autonomous motion if SPP is absent. The Bluetooth ESTOP latch prevents physical START until RESET_ESTOP.

## Install
On Windows in project directory:

```powershell
git switch mainv4
git pull --ff-only origin mainv4
python -m platformio run -e esp32dev -t upload
```

Verify the LED polarity and button levels before enabling drive motors. First test with motors disconnected or wheels raised. Firmware compile and hardware validation are not yet confirmed.
