#include "button.h"

#include <Arduino.h>
#include "adc_bench.h"
#include "input_logic.h"
#include "pinout.h"
#include "spp_debug.h"

namespace {
ButtonPressDebouncer calibButton;
AckLedBlink successLed;
bool ledOutput = false;
bool physicalCalibrationPending = false;
bool physicalCalibrationBlack = false;
enum PhysicalButtonStep {
  BUTTON_STEP_WHITE,
  BUTTON_STEP_BLACK,
  BUTTON_STEP_START
};
PhysicalButtonStep physicalButtonStep = BUTTON_STEP_WHITE;

void writeLed(bool on) {
  if (ledOutput == on) return;
  ledOutput = on;
  digitalWrite(PIN_ACTION_LED, on == (ACTION_LED_ACTIVE_HIGH != 0) ? HIGH : LOW);
}

void acknowledgeSuccess() {
  successLed.request(millis());
  writeLed(successLed.isOn());
}
}  // namespace

void button_init() {
  // GPIO34 is input-only and requires an external 10k pull-up.
  pinMode(PIN_ACTION_LED, OUTPUT);
  digitalWrite(PIN_ACTION_LED, ACTION_LED_ACTIVE_HIGH ? LOW : HIGH);
  ledOutput = false;
  pinMode(PIN_CALIB_BUTTON, INPUT);
  const uint32_t now = millis();
  calibButton.begin(digitalRead(PIN_CALIB_BUTTON) == HIGH, now);
  physicalCalibrationPending = false;
  physicalCalibrationBlack = false;
  physicalButtonStep = BUTTON_STEP_WHITE;
}

void button_update_led(uint32_t now) {
  successLed.update(now);
  writeLed(successLed.isOn());
}

void button_poll(uint32_t now, const ButtonCallbacks &callbacks) {
  const bool calibPressed =
      calibButton.update(digitalRead(PIN_CALIB_BUTTON) == HIGH, now);
  if (!calibPressed) return;

  sppLogLine("[BUTTON] GPIO34 PRESSED");
  if (physicalButtonStep == BUTTON_STEP_START) {
    if (!callbacks.startAuto) {
      sppLogLine("[AUTO] START REJECTED: no handler");
      return;
    }
    const char *error = callbacks.startAuto();
    if (error) {
      sppLogPrintf("[AUTO] START REJECTED: %s\n", error);
      return;
    }
    physicalButtonStep = BUTTON_STEP_WHITE;
    acknowledgeSuccess();
    sppLogLine("[AUTO] START SUCCESS");
    sppLogLine("[LED] ACK 3 BLINKS");
    return;
  }

  if (adcBenchActive() || sensor_is_calibrating()) {
    sppLogLine("[CALIB] REJECTED: busy");
    return;
  }

  if (callbacks.safeStop) callbacks.safeStop();
  const bool captureStarted = physicalButtonStep == BUTTON_STEP_WHITE
      ? sensor_capture_white()
      : sensor_capture_black();
  if (!captureStarted) {
    sppLogLine("[CALIB] REJECTED: ADC unavailable or WHITE step required");
    return;
  }

  physicalCalibrationPending = true;
  physicalCalibrationBlack = physicalButtonStep == BUTTON_STEP_BLACK;
  sppLogLine(physicalCalibrationBlack ?
      "[CALIB] BLACK START" : "[CALIB] WHITE START");
}

void button_on_calibration_event(CalEvent event) {
  if (!physicalCalibrationPending) return;
  if (event == CAL_WHITE_DONE && !physicalCalibrationBlack) {
    physicalCalibrationPending = false;
    physicalButtonStep = BUTTON_STEP_BLACK;
    acknowledgeSuccess();
    sppLogLine("[CALIB] WHITE SUCCESS; LED ACK 3 BLINKS");
  } else if (event == CAL_SAVED && physicalCalibrationBlack) {
    physicalCalibrationPending = false;
    physicalButtonStep = BUTTON_STEP_START;
    acknowledgeSuccess();
    sppLogLine("[CALIB] BLACK SAVED; LED ACK 3 BLINKS");
  } else if (event == CAL_ERROR) {
    physicalCalibrationPending = false;
    sppLogLine(physicalCalibrationBlack ?
        "[CALIB] BLACK FAILED; RETRY BLACK" :
        "[CALIB] WHITE FAILED; RETRY WHITE");
  }
}
