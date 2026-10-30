# Standalone button hardware check (GPIO34 + GPIO35)

This PlatformIO project does NOT build or modify the robot firmware. It only reads GPIO34 and GPIO35 and prints to USB Serial (115200).

Button schematic: 10 kOhm external pull-up to 3.3V, 100 nF to GND, switch connects signal to GND. Release = HIGH, press = LOW. Uses 35 ms debounce and prints press/release edges, a press counter and 1 Hz status.

From the main repository root in PowerShell:

```powershell
git switch mainv4
git pull --ff-only origin mainv4
pio run -d tests/button_hw -e esp32dev -t upload
pio device monitor -b 115200
```

Select or pass the correct USB COM port using `-p COM4` if necessary. Close the monitor before trying another upload.

Expected: GPIO34 and GPIO35 show HIGH while released; press GPIO34 to see CALIB PRESSED, and GPIO35 to see START PRESSED; each must return to HIGH on release.

IMPORTANT: Flashing this hardware test replaces the running firmware in ESP32 flash. To restore the full robot program afterward, upload the main project: `pio run -e esp32dev -t upload`.

This test does not require Bluetooth or the damaged LED. Lift the wheels or disconnect the motor power when diagnosing electrical hardware.
