# Physical buttons and feedback LED — mainv4

## Pin mapping
- GPIO34: CALIB (external 10 kOhm pull-up, pressed = LOW).
- GPIO35: START/STOP (external 10 kOhm pull-up, pressed = LOW).
- GPIO16: feedback LED (active-high, assumed).
- GPIO2: Bluetooth SPP status LED, unchanged.

GPIO34 and GPIO35 are input-only and have no internal pull resistors. The supplied schematic shows 10 kOhm to 3.3 V and a 100 nF capacitor to GND for both input signals; the switch shorts the signal to GND when pressed. Therefore release = HIGH and press = LOW. GPIO16 LED polarity remains assumed active-HIGH pending its schematic.

## Sequence
1. Boot: restore NVS profile if valid. GPIO16 LED executes two short test blinks automatically (without Bluetooth), then remains OFF.
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

## Diagnosing button and LED

On boot, GPIO16 flashes twice independently of SPP. If this never happens after updating the firmware, inspect GPIO16 LED wiring, polarity, current limiting, and whether the board was flashed successfully. For the two switches, measure about 3.3 V when released and 0 V when pressed. The LED patterns for CALIB are emitted on successful calibration *completion*, not immediately on button press. Test with wheels lifted and motors disconnected as practical.
