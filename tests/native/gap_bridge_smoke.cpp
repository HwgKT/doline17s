// Build: g++ -std=c++17 -Wall -Wextra -Itests/native -Iinclude \
//        src/pid_controller.cpp tests/native/gap_bridge_smoke.cpp -o gap_bridge_smoke
// Run: ./gap_bridge_smoke
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "encoder.h"
#include "sensor.h"
#include "motor.h"
#include "pid_controller.h"

static uint32_t ticksUs = 0;
uint32_t millis(void) { return ticksUs / 1000; }
uint32_t micros(void) { return ticksUs; }
static int32_t simLeft = 0, simRight = 0;

extern "C" {
uint16_t sensorRaw[N_SENSORS] = {};
uint16_t minValues[N_SENSORS] = {};
uint16_t maxValues[N_SENSORS] = {};
uint16_t thresholds[N_SENSORS] = {};
bool calibrationReady = true;
bool isCalibrating = false;
static int16_t pwmL = 0, pwmR = 0;

bool sensor_adc_ready(void) { return true; }
bool sensor_is_black(uint8_t i) {
    return calibrationReady && i < N_SENSORS && sensorRaw[i] >= thresholds[i];
}
uint16_t sensor_get_calibrated(uint8_t i) {
    if (!calibrationReady || i >= N_SENSORS) return 0;
    const uint16_t v = sensorRaw[i];
    if (v <= minValues[i]) return 0;
    if (v >= maxValues[i]) return 1000;
    return uint16_t(uint32_t(v - minValues[i]) * 1000U /
                    uint32_t(maxValues[i] - minValues[i]));
}
uint16_t sensor_get_line_bits(void) {
    uint16_t mask = 0;
    for (uint8_t i = 0; i < N_SENSORS; ++i)
        if (sensor_is_black(i)) mask |= uint16_t(1U << i);
    return mask;
}
void encoder_get_counts(int32_t* left, int32_t* right) {
    if (left) *left = simLeft;
    if (right) *right = simRight;
}
void motor_set_speeds(int16_t left, int16_t right) {
    assert(left >= -255 && left <= 255);
    assert(right >= -255 && right <= 255);
    pwmL = left; pwmR = right;
}
void motor_stop(void) { pwmL = pwmR = 0; }
}

static void stepMs(unsigned duration) {
    for (unsigned i = 0; i < duration; i += 2) {
        ticksUs += 2000U;
        pid_line_follow_step();
    }
}
static void setLine(void) {
    for (int i = 0; i < N_SENSORS; ++i) sensorRaw[i] = 1000;
    sensorRaw[7] = sensorRaw[8] = 2000;
}
static void setWhite(void) {
    for (int i = 0; i < N_SENSORS; ++i) sensorRaw[i] = 1000;
}
static void beginMap(uint8_t map, int32_t origin) {
    pid_set_auto_mode(false);
    pid_init(2.0f, 0.0f, 0.0f, 120);
    simLeft = origin;
    simRight = origin + 700;
    assert(pid_select_map(map));
    setLine();
    pid_set_auto_mode(true);
    assert(pid_get_run_encoder_progress() == 0);
    stepMs(14);
    assert(pid_get_current_case() == LINE_CASE_NORMAL_PID);
    assert(pwmL == 120 && pwmR == 120);
}
static void advanceTo(int32_t relativeTicks, int32_t origin) {
    simLeft = origin + relativeTicks;
    simRight = origin + 700 + relativeTicks;
    assert(pid_get_run_encoder_progress() == relativeTicks);
}
int main(void) {
    for (int i = 0; i < N_SENSORS; ++i) {
        minValues[i] = 1000;
        maxValues[i] = 2000;
        thresholds[i] = 1400;
    }
    pid_init(2.0f, 0.0f, 0.0f, 120);
    assert(!pid_select_map(0));
    assert(!pid_select_map(4));

    beginMap(1, 50000);
    int32_t from = 0, to = 0;
    pid_get_gap_window(&from, &to);
    assert(from == 1500 && to == 2100);
    advanceTo(1650, 50000);
    stepMs(2);
    assert(pid_get_high_speed_stage() == 2);
    assert(pwmL == 255 && pwmR == 255);
    setWhite();
    stepMs(60);
    assert(pid_get_current_case() == LINE_CASE_GAP_BRIDGE);
    assert(pwmL == 255 && pwmR == 255);  // hold last forward PWM, not 95
    assert(!pid_select_map(2));          // no MAP change mid-run
    setLine();
    stepMs(2);
    assert(pid_get_current_case() == LINE_CASE_NORMAL_PID);
    advanceTo(1750, 50000);
    setWhite();
    stepMs(92);
    assert(pid_get_current_case() == LINE_CASE_GAP_BRIDGE);
    stepMs(12); // 100ms measured from first white frame, not from 30ms loss
    assert(pid_get_current_case() == LINE_CASE_SHARP_RIGHT);
    assert(pwmL == 85 && pwmR == -85);
    stepMs(20);
    assert(pid_get_current_case() != LINE_CASE_GAP_BRIDGE); // cannot restart

    // A new run resets relative origin and timer; one window per MAP.
    beginMap(2, 70000);
    pid_get_gap_window(&from, &to);
    assert(from == 1050 && to == 1750);
    advanceTo(1130, 70000);
    setWhite();
    stepMs(50);
    assert(pid_get_current_case() == LINE_CASE_GAP_BRIDGE);
    setLine(); stepMs(2);
    assert(pid_get_current_case() == LINE_CASE_NORMAL_PID);
    advanceTo(1640, 70000);
    setWhite(); stepMs(50);
    assert(pid_get_current_case() == LINE_CASE_GAP_BRIDGE);

    beginMap(3, 90000);
    pid_get_gap_window(&from, &to);
    assert(from == 1000 && to == 1700);
    advanceTo(1090, 90000);
    setWhite(); stepMs(30);
    assert(pid_get_current_case() == LINE_CASE_GAP_BRIDGE);
    setLine(); stepMs(2);
    assert(pid_get_current_case() == LINE_CASE_NORMAL_PID);
    advanceTo(1690, 90000);
    setWhite(); stepMs(20);
    advanceTo(1750, 90000); // crossing the upper boundary must not abort
    stepMs(50);
    assert(pid_get_current_case() == LINE_CASE_GAP_BRIDGE);
    setLine(); stepMs(2);
    assert(pid_get_current_case() == LINE_CASE_NORMAL_PID);

    // First-straight trapezoidal PWM envelope, based on encoder since AUTO.
    // Baseline remains configurable; here the app would set BASE:120.
    beginMap(1, 210000);
    assert(pid_get_high_speed_stage() == 1);
    advanceTo(100, 210000);
    stepMs(2);
    assert(pwmL >= 164 && pwmL <= 166 && pwmL == pwmR);
    advanceTo(300, 210000);
    stepMs(2);
    assert(pid_get_high_speed_stage() == 2);
    assert(pwmL == 255 && pwmR == 255);
    advanceTo(2050, 210000);
    stepMs(2);
    assert(pid_get_high_speed_stage() == 3);
    assert(pwmL == 255 && pwmR == 255);
    advanceTo(2250, 210000);
    stepMs(2);
    assert(pwmL >= 187 && pwmL <= 188 && pwmL == pwmR);
    advanceTo(2450, 210000);
    stepMs(2);
    assert(pid_get_high_speed_stage() == 4);
    assert(pwmL == 120 && pwmR == 120);
    advanceTo(2500, 210000);
    stepMs(2);
    assert(pwmL == 120 && pwmR == 120);
    advanceTo(1900, 210000); // encoder reversal must not re-enter high speed
    stepMs(2);
    assert(pid_get_high_speed_stage() == 4);
    assert(pwmL == 120 && pwmR == 120);

    // Track window selection does not change the common brake-onset target.
    beginMap(2, 250000);
    advanceTo(2050, 250000); stepMs(2);
    assert(pid_get_high_speed_stage() == 3);
    beginMap(3, 280000);
    advanceTo(2050, 280000); stepMs(2);
    assert(pid_get_high_speed_stage() == 3);

    // Peak is adjustable only while stopped, so low-risk 180/220 trials
    // can be run before ever enabling the full experimental 255 command.
    pid_set_auto_mode(false);
    assert(pid_set_high_speed_peak(180));
    assert(pid_get_high_speed_peak() == 180);
    assert(!pid_set_high_speed_peak(99));
    assert(!pid_set_high_speed_peak(256));
    assert(pid_select_map(1));
    simLeft=300000; simRight=300700;
    setLine(); pid_set_auto_mode(true);
    stepMs(14);
    simLeft+=300; simRight+=300;
    stepMs(2);
    assert(pwmL==180 && pwmR==180);
    assert(!pid_set_high_speed_peak(220)); // frozen during AUTO
    simLeft+=1750; simRight+=1750;
    stepMs(2);
    assert(pid_get_high_speed_stage()==3);
    assert(pwmL==180 && pwmR==180);
    simLeft+=400; simRight+=400;
    stepMs(2);
    assert(pwmL==120 && pwmR==120);

    // Out of the MAP 1 window: preserve the existing 30ms + startup coast.
    beginMap(1, 150000);
    advanceTo(1200, 150000);
    setWhite(); stepMs(14);
    assert(pid_get_current_case() == LINE_CASE_DASHED_LINE);
    stepMs(32);
    assert(pid_get_current_case() == LINE_CASE_STARTUP_COAST);
    assert(pwmL == 95 && pwmR == 95);
    pid_set_auto_mode(false);
    assert(pwmL == 0 && pwmR == 0);
    puts("PASS: 3 MAP GAP windows, 100ms timeout, 300/2050/2450 PWM trapezoid, adjustable peak, legacy outside");
    return 0;
}
