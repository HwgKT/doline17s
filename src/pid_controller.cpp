#include "pid_controller.h"
#include "sensor.h"
#include "motor.h"
#include <Arduino.h>

// Stability test: 16 physical eyes, independently calibrated per-eye TH_i.
// Analog weights are retained for PID; only confirmed black eyes contribute.
static const float SENSOR_WEIGHTS[N_SENSORS] = {
    -75.0f, -65.0f, -55.0f, -45.0f, -35.0f, -25.0f, -15.0f, -5.0f, // 8 mắt bên trái (0..7)
      5.0f,  15.0f,  25.0f,  35.0f,  45.0f,  55.0f,  65.0f,  75.0f  // 8 mắt bên phải (8..15)
};


// Reverse-free, 10-ms line acquire, 25-ms line loss confirmation.
// The old sharp 90-degree and T-junction reverse/pivot branches are disabled
// during straight-line stability testing (preserve CASE enumeration for GUI).
static PID_Config_t pidConfig = {2.2f,0.0f,0.0f,120,220,0};
static bool autoMode=false;
static LineCase_t currentCase=LINE_CASE_STOPPED;
static float lastError=0, integralError=0, lastCorrection=0;
static float lastPTerm=0, lastITerm=0, lastDTerm=0;
static float lastKnownPosition=0;
static uint32_t lastComputeTimeUs=0;
static bool derivativeHistoryValid=false;

static constexpr uint32_t LINE_ACQUIRE_MS=10;
static constexpr uint32_t LINE_LOSS_MS=25;
static constexpr int16_t DROPOUT_FORWARD_PWM=65;
static bool lineLocked=false, previouslyLocked=false;
static bool acquiring=false, missing=false;
static uint32_t acquireSinceMs=0, missingSinceMs=0;
static uint32_t confirmedLosses=0, reacquisitions=0;

static int16_t forwardPwm(float value) {
    if (value <= 0) return 0;
    if (value >= pidConfig.maxSpeed) return pidConfig.maxSpeed;
    return (int16_t)value;
}
static void clearPidState() {
    lastError=integralError=lastCorrection=0;
    lastPTerm=lastITerm=lastDTerm=0;
    derivativeHistoryValid=false;
}
static void resetLineTracking() {
    lineLocked=previouslyLocked=acquiring=missing=false;
    acquireSinceMs=missingSinceMs=0;
    lastKnownPosition=0;
}

void pid_init(float kp,float ki,float kd,int16_t baseSpeed) {
    pidConfig.Kp=kp;pidConfig.Ki=ki;pidConfig.Kd=kd;
    pidConfig.baseSpeed=(baseSpeed<0)?0:(baseSpeed>255?255:baseSpeed);
    pidConfig.minSpeed=0;
    autoMode=false;
    currentCase=LINE_CASE_STOPPED;
    clearPidState();
    resetLineTracking();
    confirmedLosses=reacquisitions=0;
    lastComputeTimeUs=micros();
}
void pid_set_tunings(float kp,float ki,float kd) {
    if(pidConfig.Kd!=kd)derivativeHistoryValid=false;
    pidConfig.Kp=kp;pidConfig.Ki=ki;pidConfig.Kd=kd;
}
void pid_set_base_speed(int16_t speed) {
    pidConfig.baseSpeed=speed<0?0:(speed>255?255:speed);
}
void pid_set_auto_mode(bool enabled) {
    autoMode=enabled;
    currentCase=LINE_CASE_STOPPED;
    resetLineTracking();
    clearPidState();
    lastComputeTimeUs=micros();
    motor_stop(); // Never start blind; wait for 10 ms of stable black detection.
}
bool pid_is_auto_mode(void) { return autoMode; }

bool pid_calculate_position(float *outPosition,uint8_t *outBlackCount) {
    if(!outPosition||!outBlackCount)return false;
    float weighted=0;
    uint32_t strength=0;
    uint8_t blackCount=0;
    if(calibrationReady){
        for(int i=0;i<N_SENSORS;i++){
            // Logic 1 iff RAW ADC_i >= the calibrated NVS threshold TH_i.
            // The same criterion now governs detection AND analog centroid.
            if(!sensor_is_black(i))continue;
            const uint16_t cal=sensor_get_calibrated(i); // 0..1000
            weighted+=(float)cal*SENSOR_WEIGHTS[i];
            strength+=cal;
            ++blackCount;
        }
    }
    *outBlackCount=blackCount;
    if(blackCount>0 && strength>0){
        *outPosition=weighted/(float)strength;
        lastKnownPosition=*outPosition;
        return true;
    }
    *outPosition=lastKnownPosition; // Memory is not line evidence.
    return false;
}

LineCase_t pid_detect_case(float position,uint8_t blackCount){
    (void)position;
    if(!calibrationReady||!sensor_adc_ready())return LINE_CASE_STOPPED;
    const uint32_t now=millis();
    if(blackCount>0){
        missing=false;
        if(!lineLocked){
            if(!acquiring){acquiring=true;acquireSinceMs=now;}
            if(uint32_t(now-acquireSinceMs)<LINE_ACQUIRE_MS)
                return LINE_CASE_LOST_LINE; // Keep both motors stopped.
            lineLocked=true;acquiring=false;
            if(previouslyLocked)++reacquisitions;
            previouslyLocked=true;
        }
        const uint16_t mask=sensor_get_line_bits();
        if(__builtin_popcount((unsigned)mask)>=11)return LINE_CASE_CROSS_LINE;
        return LINE_CASE_NORMAL_PID; // No 90-degree or T turns during test.
    }
    acquiring=false;
    if(!lineLocked)return LINE_CASE_LOST_LINE;
    if(!missing){missing=true;missingSinceMs=now;}
    if(uint32_t(now-missingSinceMs)<LINE_LOSS_MS)
        return LINE_CASE_DASHED_LINE; // At most 25 ms gentle FORWARD.
    lineLocked=false;missing=false;
    ++confirmedLosses;
    return LINE_CASE_LOST_LINE; // STOP, not reverse.
}

void pid_line_follow_step(void) {
    if(!autoMode)return;
    float pos=0;
    uint8_t blackCount=0;
    const bool good=pid_calculate_position(&pos,&blackCount);
    const LineCase_t prev=currentCase;
    currentCase=pid_detect_case(pos,blackCount);
    const uint32_t nowUs=micros();
    const float measuredDt=(float)(nowUs-lastComputeTimeUs)*1e-6f;
    const bool dtValid=measuredDt>0 && measuredDt<=0.1f;
    const float dt=dtValid?measuredDt:0.01f;
    lastComputeTimeUs=nowUs;
    if(prev!=LINE_CASE_NORMAL_PID || currentCase!=LINE_CASE_NORMAL_PID)
        derivativeHistoryValid=false;

    if(currentCase==LINE_CASE_NORMAL_PID && good){
        const float err=pos, absErr=fabsf(err);
        const float p=pidConfig.Kp*(1.0f+0.012f*absErr)*err;
        integralError+=err*dt;
        if(integralError>150.0f)integralError=150.0f;
        if(integralError< -150.0f)integralError= -150.0f;
        const float it=pidConfig.Ki*integralError;
        float d=0;
        if(pidConfig.Kd!=0 && derivativeHistoryValid && dtValid)
            d=pidConfig.Kd*(err-lastError)/measuredDt;
        lastError=err;
        derivativeHistoryValid=true;
        lastPTerm=p;lastITerm=it;lastDTerm=d;
        lastCorrection=p+it+d;
        float base=(float)pidConfig.baseSpeed-absErr*0.55f;
        if(base<50.0f)base=50.0f;
        if(base>pidConfig.maxSpeed)base=pidConfig.maxSpeed;
        float correction=lastCorrection;
        if(correction>512.0f)correction=512.0f;
        if(correction< -512.0f)correction= -512.0f;
        // Both motors are ALWAYS forward or stopped in AUTO.
        motor_set_speeds(forwardPwm(base+correction),
                         forwardPwm(base-correction));
    }else if(currentCase==LINE_CASE_CROSS_LINE){
        clearPidState();
        const int16_t speed=forwardPwm((float)pidConfig.baseSpeed);
        motor_set_speeds(speed,speed);
    }else if(currentCase==LINE_CASE_DASHED_LINE){
        clearPidState();
        const int16_t speed=forwardPwm((float)(pidConfig.baseSpeed<DROPOUT_FORWARD_PWM?
                              pidConfig.baseSpeed:DROPOUT_FORWARD_PWM));
        motor_set_speeds(speed,speed);
    }else{
        clearPidState();
        motor_stop(); // Confirmed line loss, no line yet, or fault.
    }
}
LineCase_t pid_get_current_case(void) {
    return currentCase;
}
bool pid_line_is_confirmed(void) {return lineLocked;}
uint32_t pid_get_confirmed_line_loss_count(void) {return confirmedLosses;}
uint32_t pid_get_line_reacquisition_count(void) {return reacquisitions;}

const char* pid_get_case_name(LineCase_t c) {
    switch (c) {
        case LINE_CASE_NORMAL_PID: return "CASE 1: Bam Line PID";
        case LINE_CASE_SHARP_LEFT: return "CASE 2: Cua Gat Trai (90 deg)";
        case LINE_CASE_SHARP_RIGHT: return "CASE 3: Cua Gat Phai (90 deg)";
        case LINE_CASE_LOST_LINE: return "CASE 4: Mat Line - STOP";
        case LINE_CASE_CROSS_LINE: return "CASE 5: Vach Ngang / Nga Tu";
        case LINE_CASE_T_LEFT: return "CASE 6: Nga 3 Trai";
        case LINE_CASE_T_RIGHT: return "CASE 7: Nga 3 Phai";
        case LINE_CASE_STOPPED: return "CASE 8: Dung Xe";
        case LINE_CASE_DASHED_LINE: return "CASE 9: Pending loss - forward only";
        default: return "Khong Xac Dinh";
    }
}

float pid_get_last_error(void) {
    return lastError;
}

float pid_get_correction(void) {
    return lastCorrection;
}
float pid_get_term_p(void) { return currentCase == LINE_CASE_NORMAL_PID ? lastPTerm : 0.0f; }
float pid_get_term_i(void) { return currentCase == LINE_CASE_NORMAL_PID ? lastITerm : 0.0f; }
float pid_get_term_d(void) { return currentCase == LINE_CASE_NORMAL_PID ? lastDTerm : 0.0f; }

float pid_get_kp(void) {
    return pidConfig.Kp;
}

float pid_get_ki(void) {
    return pidConfig.Ki;
}

float pid_get_kd(void) {
    return pidConfig.Kd;
}

int16_t pid_get_base_speed(void) {
    return pidConfig.baseSpeed;
}
