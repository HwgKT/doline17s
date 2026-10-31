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
    assert(left>=-220 && right>=-220);
    assert(left<=220 && right<=220);
    if(left<0 || right<0) {
        const LineCase_t current=pid_get_current_case();
        assert(current==LINE_CASE_SHARP_LEFT || current==LINE_CASE_SHARP_RIGHT ||
               current==LINE_CASE_DASHED_LINE || current==LINE_CASE_NORMAL_PID ||
               current==LINE_CASE_REACQUIRE_SETTLE ||
               current==LINE_CASE_T_LEFT || current==LINE_CASE_T_RIGHT);
        if(current==LINE_CASE_SHARP_LEFT) assert(left<0 && right>0);
        if(current==LINE_CASE_SHARP_RIGHT) assert(left>0 && right<0);
    }
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
static void setPattern(const char *bits) {
    for(unsigned i=0;i<N_SENSORS;i++)
        sensorRaw[i]=bits[i]=='1'?2000:1000;
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
    assert(leftPwm>0 && rightPwm>=0);

    // Startup loss stays straight at a bounded speed, then recovers on line.
    allWhite();
    stepMs(60);
    assert(pid_get_current_case()==LINE_CASE_STARTUP_COAST);
    assert(leftPwm==65 && rightPwm==65);
    onlyEye(8,1700);
    stepMs(12);
    assert(pid_get_current_case()==LINE_CASE_NORMAL_PID);
    assert(pid_line_is_confirmed());

    // Outside the startup grace window, a brief missed sensor run coasts
    // with the damped PID correction before the existing pivot search.
    stepMs(5000);
    allWhite();
    stepMs(10);
    assert(pid_get_current_case()==LINE_CASE_DASHED_LINE);
    assert(leftPwm>0 && rightPwm>0);
    stepMs(32);
    assert(!pid_line_is_confirmed());
    assert(pid_get_current_case()==LINE_CASE_SHARP_RIGHT);
    assert(leftPwm==85 && rightPwm==-85);
    assert(pid_get_confirmed_line_loss_count()==1);

    // 4) Search confirms a centered/narrow line for 40ms. During the
    // candidate window both motors stop; afterwards a 150ms settle runs.
    pid_set_tunings(2.2f,0.0f,2.0f);
    onlyEye(8,1600);
    stepMs(20);
    assert(pid_get_current_case()==LINE_CASE_SHARP_RIGHT);
    assert(leftPwm==0 && rightPwm==0);
    stepMs(26);
    assert(pid_get_current_case()==LINE_CASE_REACQUIRE_SETTLE);
    assert(pid_line_is_confirmed());
    assert(pid_get_line_reacquisition_count()==1);
    assert(leftPwm>0 && rightPwm>0);
    assert(pid_get_term_d()==0.0f);
    stepMs(80);
    assert(pid_get_current_case()==LINE_CASE_REACQUIRE_SETTLE);
    stepMs(75);
    assert(pid_get_current_case()==LINE_CASE_NORMAL_PID);
    setPattern("0000000000110000");
    stepMs(6);
    assert(pid_get_current_case()==LINE_CASE_NORMAL_PID);
    pid_set_tunings(2.2f,0.0f,0.0f);

    // Visible corners stay under PID control instead of using a fixed arc.
    setPattern("0000001000001100");
    stepMs(2);
    assert(pid_get_current_case()==LINE_CASE_NORMAL_PID);
    assert(leftPwm>rightPwm);
    onlyEye(8,1600);
    stepMs(4);
    assert(pid_get_current_case()==LINE_CASE_NORMAL_PID);

    // Outermost-sensor errors allow full PID steering authority up to maxSpeed.
    onlyEye(15,2000);
    stepMs(14);
    assert(pid_get_current_case()==LINE_CASE_NORMAL_PID);
    assert(leftPwm>0 && rightPwm<0);
    // 2B cap: governed base is ~66, so the unmagnified 3B/-B output
    // is ~198/-66 (no need to hit the hardware peak of 220).
    assert(leftPwm>=190 && leftPwm<=205);
    assert(rightPwm<=-55 && rightPwm>=-75);
    assert(leftPwm-rightPwm>=245);

    // Keep the arc through a brief line gap; then pivot to search.
    allWhite();
    stepMs(20);
    assert(pid_get_current_case()==LINE_CASE_SHARP_RIGHT);
    assert(leftPwm>rightPwm && rightPwm>0);
    assert(leftPwm>66 && leftPwm<=185);
    stepMs(620);
    assert(pid_get_current_case()==LINE_CASE_SHARP_RIGHT);
    assert(leftPwm==85 && rightPwm==-85);
    onlyEye(15,2000);
    stepMs(2);
    assert(pid_get_current_case()==LINE_CASE_SHARP_RIGHT);
    assert(leftPwm==85 && rightPwm==-85);
    // Edge-only black sensors do not authorize leaving SEARCH.
    setPattern("0000000000000011");
    stepMs(50);
    assert(pid_get_current_case()==LINE_CASE_SHARP_RIGHT);
    assert(leftPwm==85 && rightPwm==-85);
    // A genuinely centered narrow line is confirmed over real time.
    setPattern("0000000110000000");
    stepMs(20);
    assert(pid_get_current_case()==LINE_CASE_SHARP_RIGHT);
    assert(leftPwm==0 && rightPwm==0);
    stepMs(26);
    assert(pid_get_current_case()==LINE_CASE_REACQUIRE_SETTLE);
    assert(leftPwm>0 && rightPwm>0);
    stepMs(154);
    assert(pid_get_current_case()==LINE_CASE_NORMAL_PID);

    // A wide junction can be seen while PID tracks a sharp turn.
    onlyEye(0,2000);
    stepMs(14);
    assert(pid_get_current_case()==LINE_CASE_NORMAL_PID);
    assert(rightPwm>leftPwm && leftPwm<0);
    allWhite();
    stepMs(350);
    assert(pid_get_current_case()==LINE_CASE_SHARP_LEFT);
    assert(leftPwm==-85 && rightPwm==85);
    // Wide/ambiguous junctions are not valid SEARCH recovery evidence.
    setPattern("1101111111110000");
    stepMs(20);
    assert(pid_get_current_case()==LINE_CASE_SHARP_LEFT);
    assert(leftPwm==-85 && rightPwm==85);
    setPattern("0111111111111100");
    stepMs(50);
    assert(pid_get_current_case()==LINE_CASE_SHARP_LEFT);
    assert(leftPwm==-85 && rightPwm==85);
    // Two separated black clusters with a centered centroid are not a line.
    setPattern("1000000110000001");
    stepMs(50);
    assert(pid_get_current_case()==LINE_CASE_SHARP_LEFT);
    assert(leftPwm==-85 && rightPwm==85);
    setPattern("0000000110000000");
    stepMs(46);
    assert(pid_get_current_case()==LINE_CASE_REACQUIRE_SETTLE);
    stepMs(154);
    assert(pid_get_current_case()==LINE_CASE_NORMAL_PID);

    // A pivot search that finds no line stops safely after two seconds.
    onlyEye(0,2000);
    stepMs(2);
    assert(pid_get_current_case()==LINE_CASE_NORMAL_PID);
    assert(rightPwm>leftPwm && leftPwm<0);
    allWhite();
    stepMs(44);
    assert(pid_get_current_case()==LINE_CASE_SHARP_LEFT);
    assert(leftPwm==-85 && rightPwm==85);
    stepMs(2010);
    assert(pid_get_current_case()==LINE_CASE_LOST_LINE);
    assert(leftPwm==0 && rightPwm==0);
    // Timeout is latched, rather than acquiring again on a brief black patch.
    onlyEye(8,1700);
    stepMs(50);
    assert(pid_get_current_case()==LINE_CASE_LOST_LINE);
    assert(leftPwm==0 && rightPwm==0);

    // The speed governor follows line error directly, without a timed
    // post-reacquisition acceleration ramp.
    pid_set_auto_mode(false);
    setPattern("0000000110000000");
    pid_set_auto_mode(true);
    stepMs(12);
    assert(pid_get_current_case()==LINE_CASE_NORMAL_PID);
    const int16_t centeredPwm = leftPwm > rightPwm ? leftPwm : rightPwm;
    assert(centeredPwm==120);

    onlyEye(10,1700);
    stepMs(20);
    const int16_t cornerPwm = leftPwm > rightPwm ? leftPwm : rightPwm;
    // EC corner-drag profile allows a slightly faster outside wheel than mainv7.\n    assert(cornerPwm < centeredPwm && cornerPwm >= 95 && cornerPwm <= 115);

    setPattern("0000000110000000");
    stepMs(10);
    const int16_t recoveringPwm = leftPwm > rightPwm ? leftPwm : rightPwm;
    assert(recoveringPwm==centeredPwm);

    // Stop only after all 16 eyes stay black for 50 ms.
    for(int i=0;i<N_SENSORS;i++)sensorRaw[i]=2000;
    stepMs(2);
    assert(pid_get_current_case()==LINE_CASE_CROSS_LINE);
    assert(leftPwm==84 && rightPwm==84);
    stepMs(6);
    assert(pid_get_current_case()==LINE_CASE_STOPPED);
    assert(leftPwm==0 && rightPwm==0);
    onlyEye(8,1700);
    stepMs(20);
    assert(pid_get_current_case()==LINE_CASE_STOPPED);
    assert(leftPwm==0 && rightPwm==0);

    // Outer sensors reduce forward speed before centroid error reaches the
    // normal governor limit.
    pid_set_auto_mode(false);
    pid_init(0.0f,0.0f,0.0f,120);
    setPattern("0000000110000000");
    pid_set_auto_mode(true);
    stepMs(12);
    assert(leftPwm==120 && rightPwm==120);
    setPattern("0000000110000001");
    stepMs(2);
    assert(pid_get_current_case()==LINE_CASE_NORMAL_PID);
    assert(leftPwm==72 && rightPwm==72);

    // 6) Repeated false reacquisitions cannot reset the global watchdog.
    pid_set_auto_mode(false);
    pid_init(2.2f,0.0f,0.0f,120);
    setPattern("0000000110000000");
    pid_set_auto_mode(true);
    stepMs(12);
    stepMs(5000); // startup grace expires
    allWhite();
    stepMs(44);
    assert(pid_get_current_case()==LINE_CASE_SHARP_RIGHT);
    for (unsigned attempt=0;attempt<5;attempt++) {
        onlyEye(8,1700);
        stepMs(46); // confirmed search candidate
        assert(pid_get_current_case()==LINE_CASE_REACQUIRE_SETTLE);
        allWhite();
        stepMs(44); // settle invalid for >=40ms, back to search
        assert(pid_get_current_case()==LINE_CASE_SHARP_RIGHT);
        stepMs(380);
        assert(pid_get_current_case()==LINE_CASE_SHARP_RIGHT);
    }
    stepMs(200);
    assert(pid_get_current_case()==LINE_CASE_LOST_LINE);
    assert(leftPwm==0 && rightPwm==0);

    // 6) Disabling AUTO immediately stops both motors.
    pid_set_auto_mode(false);
    assert(leftPwm==0 && rightPwm==0);
    // mainv9 EC integration: stable centered tracking builds a gradual
    // +20 PWM straight boost, but moving off-center cancels it immediately.
    pid_set_auto_mode(false);
    pid_init(0.0f, 0.0f, 0.0f, 120);
    setPattern("0000000110000000");
    pid_set_auto_mode(true);
    stepMs(12);
    assert(pid_get_current_case()==LINE_CASE_NORMAL_PID);
    assert(leftPwm==120 && rightPwm==120);
    stepMs(280);
    assert(leftPwm==140 && rightPwm==140);
    onlyEye(10,1700);
    stepMs(4);
    assert(leftPwm<140 && rightPwm<140);

    // EC sample: a T mask must be recognized ahead of the overlapping
    // sharp-corner mask, only after a stable 20ms signature.
    pid_set_auto_mode(false);
    pid_init(0.0f, 0.0f, 0.0f, 120);
    setPattern("1100000110000000");
    pid_set_auto_mode(true);
    stepMs(12);
    assert(pid_get_current_case()==LINE_CASE_NORMAL_PID);
    stepMs(24);
    assert(pid_get_current_case()==LINE_CASE_T_LEFT);
    assert(leftPwm<0 && rightPwm>0);

    // EC sample: a truly outer-edge-qualified 90-degree corner enters
    // a 22ms brake, then a bounded pivot; generic SEARCH remains 85/-85.
    pid_set_auto_mode(false);
    pid_init(2.2f, 0.0f, 0.0f, 120);
    setPattern("1110000000000000");
    pid_set_auto_mode(true);
    stepMs(12);
    assert(pid_get_current_case()==LINE_CASE_NORMAL_PID);
    allWhite();
    stepMs(44);
    assert(pid_get_current_case()==LINE_CASE_SHARP_LEFT);
    assert(leftPwm==-110 && rightPwm==35);
    stepMs(24);
    assert(leftPwm==-105 && rightPwm==115);
    setPattern("0000000110000000");
    stepMs(46);
    assert(pid_get_current_case()==LINE_CASE_REACQUIRE_SETTLE);
    assert(leftPwm==45 && rightPwm==-45);
    stepMs(22);
    assert(pid_get_current_case()==LINE_CASE_REACQUIRE_SETTLE);
    assert(leftPwm>0 && rightPwm>0);

    pid_set_auto_mode(false);
    puts("PASS: mainv7 recovery and mainv9 EC boost, T-junction and 3-phase corner");
    return 0;
}
