// Native regression for MAP 1 late-BASE scheduling (default legacy HS OFF).
// Build: g++ -std=c++17 -Wall -Wextra -Itests/native -Iinclude \
//        src/pid_controller.cpp tests/native/map01_late_base_smoke.cpp \
//        -o map01_late_base_smoke
// Run: ./map01_late_base_smoke
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "encoder.h"
#include "sensor.h"
#include "motor.h"
#include "pid_controller.h"

static uint32_t fakeTimeUs = 0;
uint32_t millis(void) { return fakeTimeUs / 1000U; }
uint32_t micros(void) { return fakeTimeUs; }
static int32_t leftTicks = 0, rightTicks = 0;

extern "C" {
uint16_t sensorRaw[N_SENSORS] = {};
uint16_t minValues[N_SENSORS] = {};
uint16_t maxValues[N_SENSORS] = {};
uint16_t thresholds[N_SENSORS] = {};
bool calibrationReady = true;
bool isCalibrating = false;
static int16_t pwmLeft = 0, pwmRight = 0;
bool sensor_adc_ready(void) { return true; }
bool sensor_is_black(uint8_t i) {
    return calibrationReady && i < N_SENSORS && sensorRaw[i] >= thresholds[i];
}
uint16_t sensor_get_calibrated(uint8_t i) {
    if (!calibrationReady || i >= N_SENSORS) return 0;
    if (sensorRaw[i] <= minValues[i]) return 0;
    if (sensorRaw[i] >= maxValues[i]) return 1000;
    return (uint16_t)((uint32_t)(sensorRaw[i] - minValues[i]) * 1000U /
                      (uint32_t)(maxValues[i] - minValues[i]));
}
uint16_t sensor_get_line_bits(void) {
    uint16_t mask = 0;
    for (uint8_t i = 0; i < N_SENSORS; ++i)
        if (sensor_is_black(i)) mask |= uint16_t(1U << i);
    return mask;
}
void encoder_get_counts(int32_t* left, int32_t* right) {
    if (left) *left = leftTicks;
    if (right) *right = rightTicks;
}
void motor_set_speeds(int16_t left, int16_t right) {
    assert(left >= -255 && left <= 255);
    assert(right >= -255 && right <= 255);
    pwmLeft = left; pwmRight = right;
}
void motor_stop(void) { pwmLeft = pwmRight = 0; }
}
static void stepMs(unsigned totalMs) {
    for (unsigned elapsed = 0; elapsed < totalMs; elapsed += 2U) {
        fakeTimeUs += 2000U;
        pid_line_follow_step();
    }
}
static void setEyes(int sensorA, int sensorB = -1) {
    for (int i = 0; i < N_SENSORS; ++i) sensorRaw[i] = 1000;
    if (sensorA >= 0) sensorRaw[sensorA] = 2000;
    if (sensorB >= 0) sensorRaw[sensorB] = 2000;
}
static void setProgress(int32_t ticks) {
    leftTicks = ticks;
    rightTicks = ticks;
    assert(pid_get_run_encoder_progress() == ticks);
}
int main(void) {
    for (int i = 0; i < N_SENSORS; ++i) {
        minValues[i] = 1000;
        maxValues[i] = 2000;
        thresholds[i] = 1400;
    }
    pid_init(1.60f, 0.0f, 0.80f, 140);
    assert(pid_select_map(1));
    pid_set_base_speed(140);
    assert(pid_set_high_speed_peak(220));
    assert(pid_get_map1_late_boost_start() == 10000);
    setEyes(7, 8);  // centered, adjacent black eyes
    pid_set_auto_mode(true);
    stepMs(140);
    assert(pid_get_current_case() == LINE_CASE_NORMAL_PID);
    assert(pid_finish_detection_armed());
    setProgress(9990);
    stepMs(50);
    assert(pid_get_high_speed_stage() == 0); // no early first-straight HS
    assert(pwmLeft == 140 && pwmRight == 140);
    setProgress(10020);
    stepMs(26);
    assert(pid_get_high_speed_stage() == 7); // WAIT_LINE confirmation
    stepMs(60);
    assert(pid_get_high_speed_stage() == 8); // BOOST after stable line
    assert(pwmLeft > 140 && pwmLeft < 220);
    setProgress(16300);
    stepMs(500);
    // Regression: no automatic 125 PWM cap beyond 16k.
    assert(pid_get_high_speed_stage() == 8);
    assert(pwmLeft == 220 && pwmRight == 220);

    // Deviation beyond 40 stops boosting and forces a deceleration ramp.
    setEyes(3, 4); // far to one side of the array
    stepMs(30);
    assert(pid_get_high_speed_stage() != 8);
    assert(pwmLeft != 220 || pwmRight != 220);
    setEyes(7, 8);
    stepMs(180);
    assert(pid_get_high_speed_stage() == 8);

    // Full-black finish must always stop motors, even with BOOST active.
    setEyes(-1);
    for (int i = 0; i < N_SENSORS; ++i) sensorRaw[i] = 2000;
    stepMs(12);
    assert(pid_finish_line_latched());
    assert(pid_get_current_case() == LINE_CASE_STOPPED);
    assert(pwmLeft == 0 && pwmRight == 0);

    // Reboot scenario: a single central sensor is sufficient for BOOST.
    pid_set_auto_mode(false);
    setEyes(7);
    pid_set_auto_mode(true);
    stepMs(140);
    setProgress(10020);
    stepMs(100);
    assert(pid_get_high_speed_stage() == 8);
    puts("MAP01_LATE_BASE_SMOKE_OK");
    return 0;
}
