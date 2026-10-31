#include <Arduino.h>
#include <stdint.h>

// Board schematic: 10k pull-ups to 3.3V and buttons short to GND.
// Released -> HIGH; pressed -> LOW. GPIO34/35 have no internal pulls.
constexpr uint8_t PIN_CALIB = 34;
constexpr uint8_t PIN_START = 35;
constexpr uint32_t DEBOUNCE_MS = 35;
constexpr uint32_t STATUS_MS = 1000;

struct TestButton {
  const char *name;
  uint8_t pin;
  bool rawPressed;
  bool stablePressed;
  uint32_t changedAtMs;
  uint32_t pressCount;

  TestButton(const char *buttonName, uint8_t gpio)
      : name(buttonName), pin(gpio), rawPressed(false),
        stablePressed(false), changedAtMs(0), pressCount(0) {}
};

TestButton calib("CALIB", PIN_CALIB);
TestButton startButton("START", PIN_START);
uint32_t lastStatusMs = 0;

void initializeButton(TestButton &button) {
  pinMode(button.pin, INPUT); // External 10k pull-up on PCB.
  const bool pressed = digitalRead(button.pin) == LOW;
  button.rawPressed = pressed;
  button.stablePressed = pressed;
  button.changedAtMs = millis();
  Serial.printf("[INIT] %s GPIO%u: %s\r\n",
                button.name, button.pin, pressed ? "PRESSED" : "RELEASED");
}

void updateButton(TestButton &button, uint32_t nowMs) {
  const bool pressed = digitalRead(button.pin) == LOW;

  if (pressed != button.rawPressed) {
    button.rawPressed = pressed;
    button.changedAtMs = nowMs;
  }

  if (pressed != button.stablePressed &&
      (uint32_t)(nowMs - button.changedAtMs) >= DEBOUNCE_MS) {
    button.stablePressed = pressed;

    if (pressed) {
      ++button.pressCount;
      Serial.printf("[%lu ms] GPIO%u %s: PRESSED  (#%lu)\r\n",
                    (unsigned long)nowMs, button.pin, button.name,
                    (unsigned long)button.pressCount);
    } else {
      Serial.printf("[%lu ms] GPIO%u %s: RELEASED\r\n",
                    (unsigned long)nowMs, button.pin, button.name);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println();
  Serial.println("=== ESP32 GPIO34/GPIO35 BUTTON HARDWARE TEST ===");
  Serial.println("10k pull-up: RELEASED=HIGH, PRESSED=LOW");
  Serial.println("No Bluetooth, motor, PID, or LED required.");
  initializeButton(calib);
  initializeButton(startButton);
  lastStatusMs = millis();
}

void loop() {
  const uint32_t nowMs = millis();
  updateButton(calib, nowMs);
  updateButton(startButton, nowMs);

  if ((uint32_t)(nowMs - lastStatusMs) >= STATUS_MS) {
    lastStatusMs = nowMs;
    Serial.printf(
        "[STATUS] GPIO34=%s (%s), count=%lu | "
        "GPIO35=%s (%s), count=%lu\r\n",
        digitalRead(PIN_CALIB) == LOW ? "LOW" : "HIGH",
        calib.stablePressed ? "PRESSED" : "RELEASED",
        (unsigned long)calib.pressCount,
        digitalRead(PIN_START) == LOW ? "LOW" : "HIGH",
        startButton.stablePressed ? "PRESSED" : "RELEASED",
        (unsigned long)startButton.pressCount);
  }

  delay(1);
}
