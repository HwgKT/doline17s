# GPIO34 one-button WHITE -> BLACK -> START/STOP (mainv4)

## Hardware
- **GPIO34**: the only physical control button, active LOW (external 10-kOhm pull-up to 3.3 V). Release=HIGH, press=LOW; 35-ms software debounce.
- GPIO35: no longer used for control (existing test in `tests/button_hw` may still read it).
- GPIO16: damaged external LED; firmware does not use it for feedback.
- GPIO2: remains the Bluetooth SPP status LED.
- USB Serial Monitor at 115200 displays sparse physical-button and calibration events. Bluetooth is NOT required to operate GPIO34.

## Operating sequence
All steps must be performed **after each boot**, even if calibration data has previously been restored from NVS.

1. Place all **16 sensors on white** and click GPIO34 once. Allow 128 ADC frames to finish, until the USB log prints `CAL_WHITE_DONE: NEXT_PRESS_BLACK`.
2. Place all **16 sensors on black** and click GPIO34 a second time. Allow the BLACK capture/validation and NVS commit to finish; look for `CAL_OK_SAVED: NEXT_PRESS_START`.
3. Position the robot in the **all-black starting box** facing the intended direction. Click GPIO34 a third time. `BUTTON34: AUTO_STARTED` enables the line follower; no Bluetooth connection is required.
4. Click GPIO34 while the robot is running to **STOP immediately**. Subsequent clicks restart AUTO without recalibration. Restarting ESP32 resets the physical sequence to WHITE.

Failed calibration prints `CAL_FAILED: RETRY_WHITE`; the full WHITE-then-BLACK capture must be repeated. While calibration is running, extra clicks are ignored, not counted as a stage transition.

For safety, an SPP E-STOP blocks physical START until `RESET_ESTOP`. A physical stop does NOT clear a latched E-STOP.

## Escape from the all-black starting box
The old PID stopped after just 3 consecutive full-black frames. That behavior is suppressed **only if the FIRST post-START sensor frame has at least 14/16 black channels** (tolerates two noisy eyes):

- Start moving straight with equal, limited PWM to both motors. Startup PWM cap is **90/255**, also capped by the configured base speed.
- Continue through the initial black area and a potential short all-white gap without interpreting either as a new line.
- Switch to normal PID when **2–8 adjacent black sensor channels** are detected for **3 consecutive frames**.
- If no suitable line is acquired within **2500 ms** after start, stop both motors and disable AUTO; USB reports `AUTO: START_BOX_TIMEOUT_MOTORS_STOPPED`.
- Once the start box has been exited, normal black-crossing and all-black finish detection remain in effect (the latter stops after 3 full-black frames).

These parameters are initial engineering estimates, not validated measurements. Tune the PWM/time cap for the real track only after testing with the wheels elevated and a clear emergency stop route. A black square alone does not indicate the direction of the outgoing line; **align the vehicle toward the exit before pressing START**.

## Install / monitor on Windows
Open PowerShell in the repository root. If `pio` is not on PATH, use the full installed executable:

```powershell
git switch mainv4
git pull --ff-only origin mainv4
& "C:\Users\KIEUTUANHUNG\.platformio\penv\Scripts\platformio.exe" run -e esp32dev -t upload
& "C:\Users\KIEUTUANHUNG\.platformio\penv\Scripts\platformio.exe" device monitor -p COM4 -b 115200
```

Replace COM4 with the correct USB serial port; close the monitor before re-uploading. Uploading the normal firmware is essential if the standalone `tests/button_hw` sketch is currently flashed.

The app's CSV recorder still opens when starting from the Python dashboard, **not automatically** on physical GPIO34 START. Local segment markers in the Python app are unchanged.

**Verification**: This change has been code-reviewed but full PlatformIO build and hardware testing remain to be performed by the user.
