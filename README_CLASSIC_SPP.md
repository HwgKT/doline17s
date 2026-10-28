# Bluetooth Classic SPP (mainv4)
Target: original ESP32 DevKit V1, PlatformIO esp32dev and Bluetooth Classic SPP.

Windows: pair device "LineRobot" and open its outgoing SPP COM port in Python app. USB debug 115200 8N1 and newline-terminated ASCII commands.

LED onboard GPIO2: blinking each 400 ms when disconnected; solid when SPP connected. If inverted, change DEBUG_LED_ACTIVE_HIGH in include/pinout.h.

Bluetooth telemetry: header AA 55 + packed 69-byte payload + 1 XOR checksum = 72 bytes. Python struct <I16H2i2h5fB, sent every 50 ms (20 Hz). ADC readings are RAW (0..4095), not calibrated 0..1000. PC-calibrated displays are independent of ESP calibration.

Both USB and SPP commands: PING, HELP, STATUS, PID, PWM, P, A, M, S, F, B, L, R, C, K, INV, KP:x, KI:x, KD:x, BASE:n, V:n, STAGE1, STAGE2, 3S, 2S, RACE. A binary packet is never sent to USB serial; diagnostic text is printed once per second.

IMPORTANT PID D-term: Based on d3529b7, the D formula is still Kd*(e[k]-e[k-1])/measuredDt where measuredDt is in seconds from micros(). Invalid dt and the first derivative sample after mode/case/Kd changes are skipped. Kd defaults to ZERO. All built-in presets force Kd=0. Old Kd=3.4/3.9/10 must NOT be reused without re-tuning. Only debug P/I/D getters were added; D formula is unchanged.

Calibration: firmware sweeps 4 seconds; failure restores the previous valid Min/Max. Encoder counters read only one edge per wheel and cannot infer direction. Manual movement expires after 750 ms with no repeat command, for safety. SPP disconnect stops Auto and motors.

Smoke test:
1. python -m platformio run -e esp32dev
2. python -m platformio run -e esp32dev -t upload
3. python -m platformio device monitor -p COM_USB -b 115200
4. Send PING with newline; expect PONG.
5. Pair LineRobot on Windows; LED solid and USB log [BT] CONNECTED.
6. Observe [DIAG], telemetry ~20 Hz, and PID: Kd=0.00 before running any motor.
7. Start with wheels off the ground. Build and hardware behavior have not yet been verified.
