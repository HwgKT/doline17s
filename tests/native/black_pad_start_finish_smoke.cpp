// Native regression for a 16/16-black START pad and 16/16-black FINISH pad.
// g++ -std=c++17 -Wall -Wextra -Itests/native -Iinclude \
//     src/pid_controller.cpp tests/native/black_pad_start_finish_smoke.cpp \
//     -o black_pad_start_finish_smoke
// ./black_pad_start_finish_smoke
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "encoder.h"
#include "sensor.h"
#include "motor.h"
#include "pid_controller.h"

static uint32_t clockUs = 0;
uint32_t millis(void) { return clockUs / 1000U; }
uint32_t micros(void) { return clockUs; }

extern "C" {
uint16_t sensorRaw[N_SENSORS] = {};
uint16_t minValues[N_SENSORS] = {};
uint16_t maxValues[N_SENSORS] = {};
uint16_t thresholds[N_SENSORS] = {};
bool calibrationReady = true;
bool isCalibrating = false;
static int16_t leftPwm = 0;
static int16_t rightPwm = 0;

bool sensor_adc_ready(void) { return true; }
bool sensor_is_black(uint8_t i) {
    return calibrationReady && i < N_SENSORS && sensorRaw[i] >= thresholds[i];
}
uint16_t sensor_get_calibrated(uint8_t i) {
    if (!calibrationReady || i >= N_SENSORS) return 0;
    const uint16_t raw = sensorRaw[i], white = minValues[i], black = maxValues[i];
    if (raw <= white) return 0;
    if (raw >= black) return 1000;
    return uint16_t(uint32_t(raw - white) * 1000U / (black - white));
}
uint16_t sensor_get_line_bits(void) {
    uint16_t bits = 0;
    for (uint8_t i = 0; i < N_SENSORS; ++i)
        if (sensor_is_black(i)) bits |= uint16_t(1U << i);
    return bits;
}
void encoder_get_counts(int32_t* left, int32_t* right) {
    if (left) *left = 0;
    if (right) *right = 0;
}
void motor_set_speeds(int16_t left, int16_t right) {
    assert(left >= -255 && left <= 255);
    assert(right >= -255 && right <= 255);
    leftPwm = left;
    rightPwm = right;
}
void motor_stop(void) { leftPwm = rightPwm = 0; }
}

static void setMask(uint16_t mask) {
    for (uint8_t i = 0; i < N_SENSORS; ++i)
        sensorRaw[i] = (mask & uint16_t(1U << i)) ? 2000 : 1000;
}
static void stepMs(unsigned duration) {
    assert((duration % 2U) == 0U);
    for (unsigned i = 0; i < duration; i += 2) {
        clockUs += 2000U;
        pid_line_follow_step();
    }
}
int main() {
    for (uint8_t i = 0; i < N_SENSORS; ++i) {
        minValues[i] = 1000;
        maxValues[i] = 2000;
        thresholds[i] = 1400;
    }

    // Start on the same full-width black square pattern as the FINISH.
    setMask(0xFFFFU);
    pid_init(1.6f, 0.0f, 0.8f, 130);
    assert(pid_select_map(1));
    pid_set_auto_mode(true);
    stepMs(100);
    assert(pid_get_current_case() == LINE_CASE_CROSS_LINE);
    assert(leftPwm == 130 && rightPwm == 130);
    assert(!pid_finish_line_latched());
    assert(!pid_finish_detection_armed());

    // A nearly all-black segment must not arm the finish.
    setMask(0xFFFEU); // 15/16 black
    stepMs(60);
    assert(!pid_finish_detection_armed());
    assert(pid_get_current_case() == LINE_CASE_CROSS_LINE);

    // A white gap at START must coast STRAIGHT, not spin in SEARCH.
    setMask(0x0000U);
    stepMs(40);
    assert(!pid_finish_detection_armed());
    assert(pid_get_current_case() == LINE_CASE_STARTUP_COAST);
    assert(leftPwm == rightPwm && leftPwm > 0);

    // An off-center partial line must also coast STRAIGHT; no premature PID.
    setMask(0x0003U);
    stepMs(30);
    assert(!pid_finish_detection_armed());
    assert(pid_get_current_case() == LINE_CASE_STARTUP_COAST);
    assert(leftPwm == rightPwm && leftPwm > 0);

    // A short narrow-line glitch is insufficient to arm the finish.
    setMask(0x0180U); // both central sensors see black
    stepMs(10);
    assert(!pid_finish_detection_armed());
    assert(pid_get_current_case() == LINE_CASE_STARTUP_COAST);
    setMask(0xFFFFU);
    stepMs(6);
    assert(!pid_finish_detection_armed());
    assert(!pid_finish_line_latched());

    // A stable narrow centered track for >= 30 ms arms the finish.
    setMask(0x0180U);
    stepMs(28);
    assert(!pid_finish_detection_armed());
    stepMs(6);
    assert(pid_finish_detection_armed());
    assert(pid_get_current_case() == LINE_CASE_NORMAL_PID);

    // One full-black sample, then 15/16, must not trigger STOP.
    setMask(0xFFFFU);
    stepMs(2);
    assert(pid_get_current_case() == LINE_CASE_CROSS_LINE);
    setMask(0x7FFFU);
    stepMs(2);
    assert(pid_get_current_case() == LINE_CASE_CROSS_LINE);

    // Only three consecutive full-black controller samples stop the robot.
    setMask(0xFFFFU);
    stepMs(4);
    assert(pid_get_current_case() == LINE_CASE_CROSS_LINE);
    stepMs(2);
    assert(pid_get_current_case() == LINE_CASE_STOPPED);
    assert(pid_finish_line_latched());
    assert(leftPwm == 0 && rightPwm == 0);
    setMask(0x0180U);
    stepMs(20);
    assert(pid_get_current_case() == LINE_CASE_STOPPED);
    assert(leftPwm == 0 && rightPwm == 0);

    // A new AUTO run must reset the finish arming state.
    pid_set_auto_mode(false);
    assert(!pid_finish_detection_armed());
    setMask(0xFFFFU);
    pid_set_auto_mode(true);
    stepMs(60);
    assert(!pid_finish_detection_armed());
    assert(!pid_finish_line_latched());
    assert(pid_get_current_case() == LINE_CASE_CROSS_LINE);

    // Missing the line after leaving the pad must eventually STOP safely.
    setMask(0x0000U);
    stepMs(5010);
    assert(pid_get_current_case() == LINE_CASE_STOPPED);
    assert(leftPwm == 0 && rightPwm == 0);

    // START directly on WHITE must move forward rather than be rejected.
    pid_set_auto_mode(false);
    setMask(0x0000U);
    pid_set_auto_mode(true);
    stepMs(100);
    assert(pid_get_current_case() == LINE_CASE_STARTUP_COAST);
    assert(leftPwm == rightPwm && leftPwm > 0);
    assert(!pid_finish_detection_armed());

    // When the centered track arrives, go straight during confirmation
    // and then hand off to normal PID (without another START request).
    setMask(0x0180U);
    stepMs(28);
    assert(!pid_finish_detection_armed());
    assert(pid_get_current_case() == LINE_CASE_STARTUP_COAST);
    stepMs(6);
    assert(pid_finish_detection_armed());
    assert(pid_get_current_case() == LINE_CASE_NORMAL_PID);
    setMask(0xFFFFU);
    stepMs(6);
    assert(pid_finish_line_latched());
    assert(leftPwm == 0 && rightPwm == 0);

    // START directly on the centered LINE must also enter PID after 30ms.
    pid_set_auto_mode(false);
    setMask(0x0180U);
    pid_set_auto_mode(true);
    stepMs(36);
    assert(pid_finish_detection_armed());
    assert(pid_get_current_case() == LINE_CASE_NORMAL_PID);

    // START on white without any line must time out and STOP safely.
    pid_set_auto_mode(false);
    setMask(0x0000U);
    pid_set_auto_mode(true);
    stepMs(5100);
    assert(pid_get_current_case() == LINE_CASE_STOPPED);
    assert(leftPwm == 0 && rightPwm == 0);
    assert(!pid_finish_detection_armed());

    puts("PASS: free START, straight search, centered line PID, black FINISH, timeout STOP");
    return 0;
}
