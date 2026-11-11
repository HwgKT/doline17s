#include "motor.h"
#include <Arduino.h>
#include "encoder.h"

static int16_t currentLeftSpeed = 0;
static int16_t currentRightSpeed = 0;

// Instrumentation only; does not modify motor command or output duty.
namespace {
MotorReverseEvent reverseEvents[2] = {};
bool reversePending[2] = {false, false};
int8_t previousDirection[2] = {0, 0};
int16_t previousMeaningfulPwm[2] = {0, 0};
uint32_t previousDirectionUs[2] = {0, 0};
constexpr int16_t kReverseMinPwm = 60;
void captureReversal(uint8_t wheel, int16_t pwm) {
    const int8_t direction = pwm >= kReverseMinPwm ? 1 :
                             pwm <= -kReverseMinPwm ? -1 : 0;
    if (!direction) return;
    const uint32_t now = micros();
    if (previousDirection[wheel] && previousDirection[wheel] != direction &&
        (uint32_t)(now - previousDirectionUs[wheel]) <= 300000U) {
        int32_t encL = 0, encR = 0;
        encoder_get_counts(&encL, &encR);
        reverseEvents[wheel] = {now, wheel == 0 ? encL : encR,
                                previousMeaningfulPwm[wheel], pwm};
        reversePending[wheel] = true;
    }
    previousDirection[wheel] = direction;
    previousMeaningfulPwm[wheel] = pwm;
    previousDirectionUs[wheel] = now;
}
}

bool motor_take_reverse_event(uint8_t wheel, MotorReverseEvent *event) {
    if (wheel > 1 || !event || !reversePending[wheel]) return false;
    *event = reverseEvents[wheel];
    reversePending[wheel] = false;
    return true;
}


void motor_init(void) {
    // Cài đặt tần số và độ phân giải cho 4 kênh PWM
    ledcSetup(CH_L1, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
    ledcSetup(CH_L2, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
    ledcSetup(CH_R1, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
    ledcSetup(CH_R2, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);

    // Gán chân điều khiển vào các kênh PWM tương ứng
    ledcAttachPin(PIN_MOTOR_L1, CH_L1);
    ledcAttachPin(PIN_MOTOR_L2, CH_L2);
    ledcAttachPin(PIN_MOTOR_R1, CH_R1);
    ledcAttachPin(PIN_MOTOR_R2, CH_R2);

    // Mặc định dừng cả 2 động cơ khi khởi động
    motor_stop();
}

void motor_stop(void) {
    currentLeftSpeed = 0;
    currentRightSpeed = 0;
    ledcWrite(CH_L1, 0);
    ledcWrite(CH_L2, 0);
    ledcWrite(CH_R1, 0);
    ledcWrite(CH_R2, 0);
}

void motor_set_left(int16_t speed) {
    // Giới hạn tốc độ trong khoảng [-255, 255]
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;
    captureReversal(0, speed);
    currentLeftSpeed = speed;

    if (speed > 0) {
        // Động cơ trái TIẾN
        ledcWrite(CH_L1, (uint32_t)speed);
        ledcWrite(CH_L2, 0);
    } else if (speed < 0) {
        // Động cơ trái LÙI
        ledcWrite(CH_L1, 0);
        ledcWrite(CH_L2, (uint32_t)(-speed));
    } else {
        // Động cơ trái DỪNG
        ledcWrite(CH_L1, 0);
        ledcWrite(CH_L2, 0);
    }
}

void motor_set_right(int16_t speed) {
    // Giới hạn tốc độ trong khoảng [-255, 255]
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;
    captureReversal(1, speed);
    currentRightSpeed = speed;

    if (speed > 0) {
        // Động cơ phải TIẾN
        ledcWrite(CH_R1, (uint32_t)speed);
        ledcWrite(CH_R2, 0);
    } else if (speed < 0) {
        // Động cơ phải LÙI
        ledcWrite(CH_R1, 0);
        ledcWrite(CH_R2, (uint32_t)(-speed));
    } else {
        // Động cơ phải DỪNG
        ledcWrite(CH_R1, 0);
        ledcWrite(CH_R2, 0);
    }
}

void motor_set_speeds(int16_t speed_left, int16_t speed_right) {
    motor_set_left(speed_left);
    motor_set_right(speed_right);
}

int16_t motor_get_left_speed(void) {
    return currentLeftSpeed;
}

int16_t motor_get_right_speed(void) {
    return currentRightSpeed;
}

void motor_forward(uint8_t speed) {
    motor_set_speeds(speed, speed);
}

void motor_backward(uint8_t speed) {
    motor_set_speeds(-(int16_t)speed, -(int16_t)speed);
}
