#include <Arduino.h>
#include "button.h"
#include "pinout.h"

namespace {
constexpr uint32_t DEBOUNCE_MS = 35;
// External 10k pull-up to 3.3 V; button shorts GPIO34 to GND.
bool stablePressed = false;
bool lastRawPressed = false;
uint32_t changedAtMs = 0;
}

void button_init() {
  pinMode(PIN_CALIB_BUTTON, INPUT); // GPIO34 cannot use internal pull-up.
  stablePressed = lastRawPressed = (digitalRead(PIN_CALIB_BUTTON) == LOW);
  changedAtMs = millis();
}

bool button_pressed(uint32_t now_ms) {
  const bool rawPressed = (digitalRead(PIN_CALIB_BUTTON) == LOW);
  if (rawPressed != lastRawPressed) {
    lastRawPressed = rawPressed;
    changedAtMs = now_ms;
  }
  if (rawPressed != stablePressed &&
      (uint32_t)(now_ms - changedAtMs) >= DEBOUNCE_MS) {
    stablePressed = rawPressed;
    return stablePressed; // Only one event per full press/release cycle.
  }
  return false;
}
