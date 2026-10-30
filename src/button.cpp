#include "button.h"

#include <Arduino.h>
#include "adc_bench.h"
#include "input_logic.h"
#include "pinout.h"
#include "spp_debug.h"

namespace {
ButtonPressDebouncer calibButton;
ButtonPressDebouncer startButton;
AckLedBlink successLed;
bool ledOutput = false;
bool physicalCalibrationPending = false;
bool physicalCalibrationBlack = false;

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
  // GPIO34 and GPIO35 are input-only and require external 10k pull-ups.
  pinMode(PIN_ACTION_LED, OUTPUT);
  digitalWrite(PIN_ACTION_LED, ACTION_LED_ACTIVE_HIGH ? LOW : HIGH);
  ledOutput = false;
  pinMode(PIN_CALIB_BUTTON, INPUT);
  pinMode(PIN_START_BUTTON, INPUT);
  const uint32_t now = millis();
  calibButton.begin(digitalRead(PIN_CALIB_BUTTON) == HIGH, now);
  startButton.begin(digitalRead(PIN_START_BUTTON) == HIGH, now);
  physicalCalibrationPending = false;
  physicalCalibrationBlack = false;
}

void button_update_led(uint32_t now) {
  successLed.update(now);
  writeLed(successLed.isOn());
}

void button_poll(uint32_t now, const ButtonCallbacks &callbacks) {
  const bool calibPressed =
      calibButton.update(digitalRead(PIN_CALIB_BUTTON) == HIGH, now);
  const bool startPressed =
      startButton.update(digitalRead(PIN_START_BUTTON) == HIGH, now);

  if (calibPressed) {
    sppLogLine("[BUTTON] GPIO34 PRESSED");
    if (adcBenchActive() || sensor_is_calibrating()) {
      sppLogLine("[CALIB] REJECTED: busy");
    } else {
      if (callbacks.safeStop) callbacks.safeStop();
      // Reuses the existing WHITE -> BLACK calibration state machine.
      sensor_handle_calibration_step();
      if (sensor_is_calibrating()) {
        physicalCalibrationPending = true;
        physicalCalibrationBlack =
            sensor_calibration_state() == CAL_BLACK_RUNNING;
        sppLogLine(physicalCalibrationBlack ?
            "[CALIB] BLACK START" : "[CALIB] WHITE START");
      } else {
        sppLogLine("[CALIB] REJECTED: ADC unavailable");
      }
    }
  }

  if (startPressed) {
    sppLogLine("[BUTTON] GPIO35 PRESSED");
    if (!callbacks.startAuto) {
      sppLogLine("[AUTO] START REJECTED: no handler");
    } else {
      const char *error = callbacks.startAuto();
      if (!error) {
        acknowledgeSuccess();
        sppLogLine("[AUTO] START SUCCESS");
        sppLogLine("[LED] ACK 3 BLINKS");
      } else {
        sppLogPrintf("[AUTO] START REJECTED: %s\n", error);
      }
    }
  }
}

void button_on_calibration_event(CalEvent event) {
  if (!physicalCalibrationPending) return;
  if (event == CAL_WHITE_DONE && !physicalCalibrationBlack) {
    physicalCalibrationPending = false;
    acknowledgeSuccess();
    sppLogLine("[CALIB] WHITE SUCCESS; LED ACK 3 BLINKS");
  } else if (event == CAL_SAVED && physicalCalibrationBlack) {
    physicalCalibrationPending = false;
    acknowledgeSuccess();
    sppLogLine("[CALIB] BLACK SAVED; LED ACK 3 BLINKS");
  } else if (event == CAL_ERROR) {
    physicalCalibrationPending = false;
    sppLogLine(physicalCalibrationBlack ?
        "[CALIB] BLACK FAILED; RETRY BLACK" :
        "[CALIB] WHITE FAILED; RETRY WHITE");
  }
}
