// Compile on a PC without any ESP32 board:
// g++ -std=c++17 -Wall -Wextra -Itests/native -Iinclude src/pid_controller.cpp tests/native/pid_stability_smoke.cpp -o pid_stability_smoke
// ./pid_stability_smoke
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "sensor.h"
#include "motor.h"
#include "pid_controller.h"

static uint32_t clockUs=0;
uint32_t millis(void) { return clockUs/1000; }
uint32_t micros(void) { return clockUs; }

extern "C" {
uint16_t sensorRaw[N_SENSORS]={};
uint16_t minValues[N_SENSORS]={};
uint16_t maxValues[N_SENSORS]={};
uint16_t thresholds[N_SENSORS]={};
bool calibrationReady=true;
bool isCalibrating=false;

bool sensor_adc_ready(void) {return true;}
bool sensor_is_black(uint8_t i) {
    return calibrationReady && i<N_SENSORS && sensorRaw[i]>=thresholds[i];
}
uint16_t sensor_get_calibrated(uint8_t i) {
    if (!calibrationReady || i>=N_SENSORS) return 0;
    const uint16_t x=sensorRaw[i], w=minValues[i], b=maxValues[i];
    if (x<=w) return 0;
    if (x>=b) return 1000;
    return (uint32_t)(x-w)*1000U/(b-w);
}
uint16_t sensor_get_line_bits(void) {
    uint16_t mask=0;
    for(uint8_t i=0;i<N_SENSORS;i++)
        if(sensor_is_black(i))mask|=(uint16_t)(1U<<i);
    return mask;
}
static int16_t leftPwm=0,rightPwm=0;
void motor_set_speeds(int16_t left,int16_t right) {
    // Safety invariant of ALL automatic PID paths.
    assert(left>=0 && right>=0);
    assert(left<=220 && right<=220);
    leftPwm=left;rightPwm=right;
}
void motor_stop(void) {leftPwm=rightPwm=0;}
}

static void stepMs(unsigned ms) {
    for(unsigned elapsed=0;elapsed<ms;elapsed+=2) {
        clockUs+=2000;
        pid_line_follow_step();
    }
}
static void allWhite(void) {
    for(int i=0;i<N_SENSORS;i++)sensorRaw[i]=1000;
}
static void onlyEye(unsigned eye, uint16_t raw) {
    allWhite();
    sensorRaw[eye]=raw;
}
int main(void) {
    for(int i=0;i<N_SENSORS;i++){
        minValues[i]=1000;maxValues[i]=2000;thresholds[i]=1400;
    }
    pid_init(2.2f,0,0,120);
    pid_set_auto_mode(true);
    pid_line_follow_step();
    assert(leftPwm==0 && rightPwm==0);

    // 1) Analog CAL=350, logic=0: must NOT start or count as a line.
    onlyEye(8,1350);
    stepMs(20);
    assert(!pid_line_is_confirmed());
    assert(leftPwm==0 && rightPwm==0);

    // 2) Valid black must persist for >=10ms before PID drives.
    onlyEye(8,1700);
    stepMs(8);
    assert(leftPwm==0 && rightPwm==0);
    stepMs(6);
    assert(pid_line_is_confirmed());
    assert(leftPwm>0 && rightPwm>0);

    // 3) A brief missed sensor run drives only FORWARD at reduced speed.
    allWhite();
    stepMs(10);
    assert(leftPwm==65 && rightPwm==65);
    stepMs(20);
    assert(!pid_line_is_confirmed());
    assert(leftPwm==0 && rightPwm==0);
    assert(pid_get_confirmed_line_loss_count()==1);

    // 4) Require stable reacquisition, not a single noisy black frame.
    onlyEye(8,1600);
    stepMs(6);
    assert(leftPwm==0 && rightPwm==0);
    stepMs(8);
    assert(pid_line_is_confirmed());
    assert(pid_get_line_reacquisition_count()==1);
    assert(leftPwm>0 && rightPwm>0);

    // 5) Extreme edge detection cannot cause reverse motor commands.
    onlyEye(15,2000);
    stepMs(14);
    assert(leftPwm>=0 && rightPwm>=0);
    // Even an 11+eye intersection only drives both motors forward.
    for(int i=0;i<N_SENSORS;i++)sensorRaw[i]=2000;
    stepMs(14);
    assert(pid_get_current_case()==LINE_CASE_CROSS_LINE);
    assert(leftPwm==120 && rightPwm==120);

    // 6) Disabling AUTO immediately stops both motors.
    pid_set_auto_mode(false);
    assert(leftPwm==0 && rightPwm==0);
    puts("PASS: per-eye thresholds, 10ms acquire, 25ms loss, relock, forward-only and STOP");
    return 0;
}
