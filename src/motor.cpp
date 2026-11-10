#include "motor.h"
#include "encoder.h"
#include <Arduino.h>
#include <math.h>

static int16_t currentLeftSpeed = 0;
static int16_t currentRightSpeed = 0;

// Normal line following uses closed-loop speed; special turns and manual
// commands retain their original direct PWM behavior.
namespace {
constexpr uint32_t WHEEL_SAMPLE_US = 20000U; // 50 Hz
// Initial nominal mapping (ticks/s per legacy PWM command); CALIBRATE on robot.
constexpr float TICKS_PER_SECOND_PER_CMD_L = 7.0f;
constexpr float TICKS_PER_SECOND_PER_CMD_R = 7.0f;
constexpr float WHEEL_KP = 0.025f;  // PWM / (tick/s)
constexpr float WHEEL_KI = 0.35f;   // PWM / tick
constexpr float VELOCITY_EMA_ALPHA = 0.40f;
constexpr float MAX_PI_CORRECTION_PWM = 45.0f;
constexpr float MAX_INTEGRAL_PWM = 35.0f;
bool wheelMode = false;
int16_t targetCommandL = 0, targetCommandR = 0;
int32_t previousCountL = 0, previousCountR = 0;
uint32_t previousSampleUs = 0;
float velocityL = 0.0f, velocityR = 0.0f;
float integralL = 0.0f, integralR = 0.0f;
bool speedSampleValid = false;

int16_t clampDuty(float duty) {
    if (duty > 255.0f) duty = 255.0f;
    if (duty < -255.0f) duty = -255.0f;
    return (int16_t)lroundf(duty);
}
void resetWheelState() {
    encoder_get_counts(&previousCountL, &previousCountR);
    previousSampleUs = micros();
    velocityL = velocityR = 0.0f;
    integralL = integralR = 0.0f;
    speedSampleValid = false;
}
float calculateWheelPwm(int16_t command, float velocity, float dt,
                        float ticksPerCommand, float& integral) {
    if (command == 0) { integral = 0.0f; return 0.0f; }
    const float error = (float)command * ticksPerCommand - velocity;
    // Feedforward: approximate PWM for the desired encoder velocity.
    const float p = WHEEL_KP * error;
    const float candidateI = fmaxf(-MAX_INTEGRAL_PWM,
                                  fminf(MAX_INTEGRAL_PWM, integral + WHEEL_KI * error * dt));
    const float trialCorrection = p + candidateI;
    const float boundedTrial = fmaxf(-MAX_PI_CORRECTION_PWM,
                                    fminf(MAX_PI_CORRECTION_PWM, trialCorrection));
    const float trialOutput = (float)command + boundedTrial;
    // Freeze integration if additional integral effort would deepen saturation.
    if (!((trialOutput >= 255.0f && error > 0.0f) ||
          (trialOutput <= -255.0f && error < 0.0f) ||
          (fabsf(trialCorrection) >= MAX_PI_CORRECTION_PWM &&
           trialCorrection * error > 0.0f))) {
        integral = candidateI;
    }
    const float correction = fmaxf(-MAX_PI_CORRECTION_PWM,
                                  fminf(MAX_PI_CORRECTION_PWM, p + integral));
    float output = (float)command + correction;
    if (command > 0) output = fmaxf(0.0f, output);
    else output = fminf(0.0f, output);
    return output;
}
} // namespace

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
    targetCommandL = targetCommandR = 0;
    integralL = integralR = 0.0f;
    speedSampleValid = false;
    ledcWrite(CH_L1, 0);
    ledcWrite(CH_L2, 0);
    ledcWrite(CH_R1, 0);
    ledcWrite(CH_R2, 0);
}

void motor_set_left(int16_t speed) {
    // Giới hạn tốc độ trong khoảng [-255, 255]
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;
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
    if (!wheelMode) {
        motor_set_left(speed_left);
        motor_set_right(speed_right);
        return;
    }
    targetCommandL = speed_left > 255 ? 255 : (speed_left < -255 ? -255 : speed_left);
    targetCommandR = speed_right > 255 ? 255 : (speed_right < -255 ? -255 : speed_right);
    if (targetCommandL == 0 && targetCommandR == 0) motor_stop();
}

void motor_set_wheel_speed_mode(bool enabled) {
    if (wheelMode == enabled) return;
    wheelMode = enabled;
    targetCommandL = targetCommandR = 0;
    resetWheelState();
}

void motor_update_wheel_pi(void) {
    if (!wheelMode) return;
    const uint32_t now = micros();
    const uint32_t elapsed = now - previousSampleUs;
    if (elapsed < WHEEL_SAMPLE_US) return;
    if (elapsed > 100000U) { resetWheelState(); return; }
    int32_t countL = 0, countR = 0;
    encoder_get_counts(&countL, &countR);
    const float dt = (float)elapsed * 1e-6f;
    const float rawL = (float)((int64_t)countL - previousCountL) / dt;
    const float rawR = (float)((int64_t)countR - previousCountR) / dt;
    velocityL = speedSampleValid
        ? VELOCITY_EMA_ALPHA * rawL + (1.0f - VELOCITY_EMA_ALPHA) * velocityL : rawL;
    velocityR = speedSampleValid
        ? VELOCITY_EMA_ALPHA * rawR + (1.0f - VELOCITY_EMA_ALPHA) * velocityR : rawR;
    speedSampleValid = true;
    previousCountL = countL; previousCountR = countR;
    previousSampleUs = now;
    const float newL = calculateWheelPwm(targetCommandL, velocityL, dt,
                                         TICKS_PER_SECOND_PER_CMD_L, integralL);
    const float newR = calculateWheelPwm(targetCommandR, velocityR, dt,
                                         TICKS_PER_SECOND_PER_CMD_R, integralR);
    motor_set_left(clampDuty(newL));
    motor_set_right(clampDuty(newR));
}

void motor_get_wheel_velocity(float* target_left, float* actual_left,
                              float* target_right, float* actual_right) {
    if (target_left) *target_left = wheelMode
        ? (float)targetCommandL * TICKS_PER_SECOND_PER_CMD_L : 0.0f;
    if (actual_left) *actual_left = velocityL;
    if (target_right) *target_right = wheelMode
        ? (float)targetCommandR * TICKS_PER_SECOND_PER_CMD_R : 0.0f;
    if (actual_right) *actual_right = velocityR;
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
