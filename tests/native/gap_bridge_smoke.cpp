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
    assert(pwmL == 220 && pwmR == 220);
    setWhite();
    stepMs(60);
    assert(pid_get_current_case() == LINE_CASE_GAP_BRIDGE);
    assert(pwmL == 220 && pwmR == 220);  // hold last forward PWM, not 95
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

    // All MAPs share first-straight brake 1800->2300, then HOLD PWM 90.
    // Finishing the ramp must NOT return to BASE at encoder 2300.
    for (uint8_t map = 1; map <= 3; ++map) {
        const int32_t origin = 210000 + int32_t(map) * 10000;
        beginMap(map, origin);
        assert(pid_get_high_speed_stage() == 1);
        advanceTo(100, origin);
        stepMs(2);
        assert(pwmL >= 153 && pwmL <= 154 && pwmL == pwmR);
        advanceTo(300, origin);
        stepMs(2);
        assert(pid_get_high_speed_stage() == 2);
        assert(pwmL == 220 && pwmR == 220);
        advanceTo(1800, origin);
        stepMs(2);
        assert(pid_get_high_speed_stage() == 3);
        assert(pwmL == 220 && pwmR == 220);
        advanceTo(2050, origin);
        stepMs(2);
        assert(pwmL == 155 && pwmL == pwmR);
        advanceTo(2250, origin);
        stepMs(2);
        assert(pwmL == 103 && pwmL == pwmR);
        advanceTo(2300, origin);
        stepMs(2);
        assert(pid_get_high_speed_stage() == 6); // HOLD
        assert(pwmL == 90 && pwmR == 90);
        advanceTo(2500, origin);
        stepMs(2);
        assert(pid_get_high_speed_stage() == 6);
        assert(pwmL == 90 && pwmR == 90);
        advanceTo(3100, origin);
        stepMs(100);
        assert(pid_get_high_speed_stage() == 6); // never release without corner
        assert(pwmL == 90 && pwmR == 90);
        // Encoder high-water mark must prevent a ramp restart.
        advanceTo(1900, origin);
        stepMs(2);
        assert(pid_get_high_speed_stage() == 6);
        assert(pwmL == 90 && pwmR == 90);

        // Real right-hand edge, then stable centered line after the turn.
        advanceTo(2600, origin);
        setWhite();
        sensorRaw[15] = 2000;
        stepMs(2);
        assert(pid_get_high_speed_stage() == 6);
        // The moment the sensor recenters, TURN_RIGHT still persists for
        // two controller frames. Forward target must remain at 90, not BASE.
        setLine();
        stepMs(2);
        assert(pid_get_high_speed_stage() == 6);
        assert(pwmL == 90 && pwmR == 90);
        stepMs(100); // centered too early at 2600: HOLD must remain
        assert(pid_get_high_speed_stage() == 6);
        advanceTo(3100, origin);
        stepMs(20);
        assert(pid_get_high_speed_stage() == 6);
        stepMs(60);
        assert(pid_get_high_speed_stage() == 4); // COMPLETE after 30 frames
        assert(pwmL == 120 && pwmR == 120);
        advanceTo(1900, origin);
        stepMs(2);
        assert(pid_get_high_speed_stage() == 4);
        assert(pwmL == 120 && pwmR == 120);
    }

    // MAP 1 has a later dashed segment (~1985-2058) inside BRAKE.
    // Its existing 100ms GAP_BRIDGE must still work while decelerating.
    beginMap(1, 400000);
    advanceTo(1985, 400000);
    stepMs(2);
    assert(pid_get_high_speed_stage() == 3); // BRAKE
    const int16_t gapBrakePwm = pwmL;
    assert(gapBrakePwm > 90 && gapBrakePwm < 220);
    setWhite();
    stepMs(60);
    assert(pid_get_current_case() == LINE_CASE_GAP_BRIDGE);
    assert(pwmL == gapBrakePwm && pwmR == gapBrakePwm);
    advanceTo(2058, 400000);
    setLine();
    stepMs(2);
    assert(pid_get_current_case() == LINE_CASE_NORMAL_PID);
    assert(pid_get_high_speed_stage() == 3);

    // Peak is configurable only while stopped. Low peaks still brake to 90.
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
    simLeft+=1500; simRight+=1500; // 1800
    stepMs(2);
    assert(pid_get_high_speed_stage()==3);
    assert(pwmL==180 && pwmR==180);
    simLeft+=250; simRight+=250; // 2050
    stepMs(2);
    assert(pwmL==135 && pwmR==135);
    simLeft+=250; simRight+=250; // 2300
    stepMs(2);
    assert(pid_get_high_speed_stage()==6);
    assert(pwmL==90 && pwmR==90);

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
    puts("PASS: 3 MAP GAP windows, common 300/1800/2300 PWM brake to 90, stable right-corner HOLD release, adjustable peak");
    return 0;
}
