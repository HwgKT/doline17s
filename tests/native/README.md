# Native PID stability smoke test

Compile the **real** firmware PID controller using a host Arduino clock stub,
and mock only the hardware-facing ADC/motor I/O:

~~~bash
g++ -std=c++17 -Wall -Wextra -Itests/native -Iinclude \
  src/pid_controller.cpp tests/native/pid_stability_smoke.cpp \
  -o pid_stability_smoke
./pid_stability_smoke
~~~

Requires a desktop C++17 compiler (GCC/Clang); no board or PlatformIO required.

Test coverage:
- Raw calibration threshold TH_i=1400 is respected, CAL=350 is not a black line.
- No startup movement without 10 ms stable black detection.
- Brief no-black interval commands (65,65), then a confirmed 25-ms loss stops.
- Reacquire requires another 10 ms of stable black, counted in diagnostics.
- Extreme left/right and wide black intersection never command negative PWM.
- Stopping AUTO stops both motors immediately.

This **does not validate actual ESP32 pin wiring, motor polarity, timing under
RTOS load, Bluetooth SPP delivery or the compiled full firmware**. Perform the
real hardware acceptance test with raised wheels and then a straight black line.
