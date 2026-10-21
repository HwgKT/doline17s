#include "pid_controller.h"
#include "sensor.h"
#include "motor.h"
#include <Arduino.h>

// Cấu hình dải mắt cảm biến sử dụng (Bỏ mắt 0 và 15 bị hỏng)
#define SENSOR_START_IDX   1      // Bắt đầu từ mắt 1
#define SENSOR_END_IDX     14     // Kết thúc ở mắt 14 (Tổng cộng 14 mắt hoạt động)

// Trọng số vị trí đối xứng cho 16 mắt (Mắt 0 và 15 gán 0 vì không dùng)
static const float SENSOR_WEIGHTS[N_SENSORS] = {
      0.0f,                       // Mắt 0: Bỏ qua (không dùng)
    -65.0f, -55.0f, -45.0f, -35.0f, -25.0f, -15.0f, -5.0f, // 7 mắt bên trái
      5.0f,  15.0f,  25.0f,  35.0f,  45.0f,  55.0f,  65.0f, // 7 mắt bên phải
      0.0f                        // Mắt 15: Bỏ qua (không dùng)
};

// Cấu hình PID AN TOÀN TỐC ĐỘ CAO (Nấc 1: Mục tiêu 12-13s, tỷ lệ bám 95%):
// Kp=1.55, Ki=0.0, Kd=3.4, baseSpeed=175, maxSpeed=240, minSpeed=-110
static PID_Config_t pidConfig = {
    .Kp = 1.55f,
    .Ki = 0.0f,
    .Kd = 3.4f,
    .baseSpeed = 175,
    .maxSpeed = 240,
    .minSpeed = -110
};

static bool autoMode = false;
static LineCase_t currentCase = LINE_CASE_STOPPED;
static float lastError = 0.0f;
static float lastFilteredD = 0.0f; // Bộ lọc khử gai vi phân chống rung lắc
static float integralError = 0.0f;
static float lastCorrection = 0.0f;
static unsigned long lastComputeTime = 0;
static uint16_t straightStreak = 0; // Đếm số chu kỳ bám thẳng liên tục để kích hoạt Turbo Boost

#define DASHED_LINE_TIMEOUT_MS  55    // Rút ngắn để tránh phóng mù ra ngoài khi văng góc cua
#define SHARP_TURN_TIMEOUT_MS   380   // Thời gian tối đa giữ cua 90 độ trước khi tự thoát an toàn
#define SHARP_TURN_COOLDOWN_MS  65    // Thời gian miễn nhiễm sau khi thoát cua (tránh quay vòng tròn lặp lại)
#define CROSS_LINE_COOLDOWN_MS  95    // Thời gian miễn nhiễm sau ngã tư (tránh mép vạch ngang kích hoạt nhầm cua 90 độ)
static unsigned long lostLineStartTime = 0;
static bool isLineCurrentlyLost = false;
static int8_t lastKnownDirection = 0; // -1: Trái, +1: Phải

// Biến chốt trạng thái rẽ 90 độ
static bool isTurningSharpLeft = false;
static bool isTurningSharpRight = false;
static bool hasClearedOldLine = false;
static float lastKnownPosition = 0.0f; // Bộ nhớ vị trí vạch cuối cùng (Line Memory)
static unsigned long sharpTurnStartTime = 0;
static unsigned long lastSharpTurnExitTime = 0;
static unsigned long lastCrossLineExitTime = 0;

static inline int16_t clamp_val(int16_t v, int16_t min_v, int16_t max_v) {
    if (v < min_v) return min_v;
    if (v > max_v) return max_v;
    return v;
}

void pid_init(float kp, float ki, float kd, int16_t baseSpeed) {
    pidConfig.Kp = kp;
    pidConfig.Ki = ki;
    pidConfig.Kd = kd;
    pidConfig.baseSpeed = baseSpeed;
    lastError = 0.0f;
    integralError = 0.0f;
    lastComputeTime = millis();
    autoMode = false;
    currentCase = LINE_CASE_STOPPED;
    isLineCurrentlyLost = false;
    isTurningSharpLeft = false;
    isTurningSharpRight = false;
    hasClearedOldLine = false;
    lastKnownPosition = 0.0f;
    lastSharpTurnExitTime = 0;
    lastCrossLineExitTime = 0;
    lastFilteredD = 0.0f;
    straightStreak = 0;
}

void pid_set_tunings(float kp, float ki, float kd) {
    pidConfig.Kp = kp;
    pidConfig.Ki = ki;
    pidConfig.Kd = kd;
}

void pid_set_base_speed(int16_t speed) {
    pidConfig.baseSpeed = clamp_val(speed, 0, 255);
}

void pid_set_auto_mode(bool enable) {
    autoMode = enable;
    isLineCurrentlyLost = false;
    isTurningSharpLeft = false;
    isTurningSharpRight = false;
    hasClearedOldLine = false;
    lastKnownPosition = 0.0f;
    lastSharpTurnExitTime = 0;
    lastCrossLineExitTime = 0;
    lastFilteredD = 0.0f;
    straightStreak = 0;
    if (!autoMode) {
        currentCase = LINE_CASE_STOPPED;
        motor_stop();
    } else {
        integralError = 0.0f;
        lastError = 0.0f;
        lastComputeTime = millis();
    }
}

bool pid_is_auto_mode(void) {
    return autoMode;
}

bool pid_calculate_position(float *outPosition, uint8_t *outBlackCount) {
    float sumWeights = 0.0f;
    uint32_t sumCalibrated = 0;
    uint8_t count = 0;

    // Quét toàn bộ mắt từ 1 đến 14 bằng giá trị chuẩn hóa Analog 0 - 1000
    for (int i = SENSOR_START_IDX; i <= SENSOR_END_IDX; i++) {
        uint16_t cal = sensor_get_calibrated(i); // Giá trị 0 - 1000 chuẩn hóa

        // Lọc nhiễu nền: chỉ lấy tín hiệu thực sự có vạch đen (cal >= 150)
        if (cal >= 150) {
            sumCalibrated += cal;
            sumWeights += (float)cal * SENSOR_WEIGHTS[i];
            if (cal >= 500) {
                count++;
            }
        }
    }

    *outBlackCount = count;

    // Nếu tổng độ đen trên thanh cảm biến >= 300 (tức là có vạch line)
    if (sumCalibrated >= 300) {
        float pos = sumWeights / (float)sumCalibrated; // Trọng tâm liên tục (Smooth Centroid)
        *outPosition = pos;

        // Ghi nhớ vị trí và hướng vạch cuối cùng (Line Memory)
        lastKnownPosition = pos;
        if (pos < -15.0f) {
            lastKnownDirection = -1; // Vạch ở bên trái
        } else if (pos > 15.0f) {
            lastKnownDirection = 1;  // Vạch ở bên phải
        } else {
            lastKnownDirection = 0;  // Vạch đang ở chính giữa tâm
        }
        return true;
    }

    // Mất vạch hoàn toàn (cả 14 mắt đều trắng tinh) -> Giữ vị trí cuối cùng
    *outPosition = lastKnownPosition;
    return false;
}

LineCase_t pid_detect_case(float position, uint8_t blackCount) {
    if (!calibrationReady) {
        return LINE_CASE_STOPPED;
    }

    uint16_t mask = sensor_get_line_bits();
    uint8_t totalBlack = __builtin_popcount(mask & 0b0111111111111110);

    // =========================================================================
    // 1. KIỂM TRA TRẠNG THÁI ĐANG GIỮ CUA 90 ĐỘ (LATCHED TURN)
    // =========================================================================
    if (isTurningSharpLeft) {
        unsigned long turnElapsed = millis() - sharpTurnStartTime;

        // Sau 30ms: Khi bất kỳ mắt tâm nào (6, 7, 8, 9) bắt lại line mới
        if (turnElapsed >= 30) {
            if ((mask & 0b0000001111000000) != 0 || (turnElapsed >= 70 && (mask & 0b0000011111100000) != 0)) {
                isTurningSharpLeft = false;
                lastKnownDirection = 0;
                lastError = 0.0f;
                lastSharpTurnExitTime = millis();
                return LINE_CASE_NORMAL_PID;
            }
        }

        if (turnElapsed > SHARP_TURN_TIMEOUT_MS) {
            isTurningSharpLeft = false;
            lastSharpTurnExitTime = millis();
        } else {
            return LINE_CASE_SHARP_LEFT;
        }
    }

    if (isTurningSharpRight) {
        unsigned long turnElapsed = millis() - sharpTurnStartTime;

        if (turnElapsed >= 30) {
            if ((mask & 0b0000001111000000) != 0 || (turnElapsed >= 70 && (mask & 0b0000011111100000) != 0)) {
                isTurningSharpRight = false;
                lastKnownDirection = 0;
                lastError = 0.0f;
                lastSharpTurnExitTime = millis();
                return LINE_CASE_NORMAL_PID;
            }
        }

        if (turnElapsed > SHARP_TURN_TIMEOUT_MS) {
            isTurningSharpRight = false;
            lastSharpTurnExitTime = millis();
        } else {
            return LINE_CASE_SHARP_RIGHT;
        }
    }

    // =========================================================================
    // 2. VẠCH NGANG / NGÃ TƯ / VẠCH ĐÍCH (Từ 9 mắt đen trở lên cùng lúc)
    // =========================================================================
    if (totalBlack >= 9) {
        lastCrossLineExitTime = millis();
        return LINE_CASE_CROSS_LINE;
    }

    bool canTriggerSharp = (millis() - lastSharpTurnExitTime >= SHARP_TURN_COOLDOWN_MS) &&
                           (millis() - lastCrossLineExitTime >= CROSS_LINE_COOLDOWN_MS);

    // =========================================================================
    // 3. MẤT LINE HOÀN TOÀN (Cả 14 mắt đều trắng tinh: blackCount == 0)
    // =========================================================================
    if (blackCount == 0) {
        if (!isLineCurrentlyLost) {
            isLineCurrentlyLost = true;
            lostLineStartTime = millis();
        }

        unsigned long lostElapsed = millis() - lostLineStartTime;

        if (canTriggerSharp && lastKnownPosition < -28.0f) {
            isTurningSharpLeft = true;
            isTurningSharpRight = false;
            sharpTurnStartTime = millis();
            lastKnownDirection = -1;
            return LINE_CASE_SHARP_LEFT;
        }

        if (canTriggerSharp && lastKnownPosition > 28.0f) {
            isTurningSharpRight = true;
            isTurningSharpLeft = false;
            sharpTurnStartTime = millis();
            lastKnownDirection = 1;
            return LINE_CASE_SHARP_RIGHT;
        }

        if (lastKnownDirection == 0 && lostElapsed <= DASHED_LINE_TIMEOUT_MS) {
            return LINE_CASE_DASHED_LINE;
        }

        return LINE_CASE_LOST_LINE;
    }

    isLineCurrentlyLost = false;

    // =========================================================================
    // 4. PHÁT HIỆN SỚM CUA 90 ĐỘ KHI VẠCH CẮT NGANG MÉP NGOÀI
    // =========================================================================
    uint8_t leftBits  = __builtin_popcount(mask & 0b0000000011111110);
    uint8_t rightBits = __builtin_popcount(mask & 0b0111111100000000);

    if (canTriggerSharp && (mask & 0b0000000000001110) != 0 && leftBits >= 3 && rightBits == 0) {
        isTurningSharpLeft = true;
        isTurningSharpRight = false;
        sharpTurnStartTime = millis();
        lastKnownDirection = -1;
        return LINE_CASE_SHARP_LEFT;
    }

    if (canTriggerSharp && (mask & 0b0111000000000000) != 0 && rightBits >= 3 && leftBits == 0) {
        isTurningSharpRight = true;
        isTurningSharpLeft = false;
        sharpTurnStartTime = millis();
        lastKnownDirection = 1;
        return LINE_CASE_SHARP_RIGHT;
    }

    // =========================================================================
    // 5. PHÁT HIỆN NGÃ 3 CHỮ T
    // =========================================================================
    if ((mask & 0b0000001111000000) != 0 && (mask & 0b0000000000001110) != 0 && rightBits <= 1) {
        return LINE_CASE_T_LEFT;
    }

    if ((mask & 0b0000001111000000) != 0 && (mask & 0b0111000000000000) != 0 && leftBits <= 1) {
        return LINE_CASE_T_RIGHT;
    }

    return LINE_CASE_NORMAL_PID;
}

void pid_line_follow_step(void) {
    if (!autoMode) {
        return;
    }

    float position = 0.0f;
    uint8_t blackCount = 0;
    pid_calculate_position(&position, &blackCount);

    currentCase = pid_detect_case(position, blackCount);

    unsigned long now = millis();
    float dt = (float)(now - lastComputeTime) / 1000.0f;
    if (dt <= 0.0f || dt > 0.05f) dt = 0.0025f;
    lastComputeTime = now;

    switch (currentCase) {

        // ---------------------------------------------------------------------
        // CASE 1: BÁM LINE AN TOÀN TỐC ĐỘ CAO (Nấc 1: 12-13s, Max Thẳng 215, Cua 95-105)
        // ---------------------------------------------------------------------
        case LINE_CASE_NORMAL_PID: {
            float error = position;
            float absErr = fabsf(error);

            int16_t currentBase = pidConfig.baseSpeed;
            if (absErr < 4.5f) {
                if (straightStreak < 500) straightStreak++;
                // Bứt ga đường thẳng: +35 PWM (đạt ~210 - 215 PWM)
                int16_t boost = (straightStreak > 18) ? 35 : (int16_t)(straightStreak * 2.0f);
                currentBase += boost;
                if (currentBase > pidConfig.maxSpeed) currentBase = pidConfig.maxSpeed;
            } else {
                straightStreak = 0;
                // Hạ tốc độ ôm cua đầm chắc, triệt tiêu quán tính
                if (absErr <= 16.0f) {
                    currentBase -= (int16_t)((absErr - 4.5f) * 2.4f);
                } else {
                    currentBase -= (int16_t)(28 + (absErr - 16.0f) * 2.5f);
                }
                if (currentBase < 95) currentBase = 95;
            }

            if (absErr < 0.8f) {
                error *= 0.5f;
            }

            // Kp phi tuyến: êm dịu khi thẳng, đanh chắc khi cua
            float effectiveKp = pidConfig.Kp;
            if (absErr > 11.0f) {
                effectiveKp += (absErr - 11.0f) * 0.016f;
                if (effectiveKp > pidConfig.Kp * 1.45f) effectiveKp = pidConfig.Kp * 1.45f;
            }
            float pTerm = effectiveKp * error;

            integralError += error * dt;
            if (integralError > 100.0f) integralError = 100.0f;
            if (integralError < -100.0f) integralError = -100.0f;
            float iTerm = pidConfig.Ki * integralError;

            // Khâu D lọc thông thấp chống rung lắc
            float deltaErr = error - lastError;
            lastError = error;
            lastFilteredD = 0.60f * lastFilteredD + 0.40f * deltaErr;
            float dTerm = pidConfig.Kd * lastFilteredD;

            lastCorrection = pTerm + iTerm + dTerm;

            int16_t leftSpeed  = currentBase + (int16_t)lastCorrection;
            int16_t rightSpeed = currentBase - (int16_t)lastCorrection;

            leftSpeed  = clamp_val(leftSpeed, pidConfig.minSpeed, pidConfig.maxSpeed);
            rightSpeed = clamp_val(rightSpeed, pidConfig.minSpeed, pidConfig.maxSpeed);

            motor_set_speeds(leftSpeed, rightSpeed);
            break;
        }

        // ---------------------------------------------------------------------
        // CASE 2: CUA GẮT TRÁI 90 ĐỘ (Phanh khựng dứt khoát + Xoay tâm ổn định)
        // ---------------------------------------------------------------------
        case LINE_CASE_SHARP_LEFT: {
            straightStreak = 0;
            unsigned long turnElapsed = millis() - sharpTurnStartTime;
            if (turnElapsed < 36) {
                motor_set_speeds(-120, -50);
            } else {
                motor_set_speeds(-100, 95);
            }
            integralError = 0.0f;
            lastError = -15.0f;
            break;
        }

        // ---------------------------------------------------------------------
        // CASE 3: CUA GẮT PHẢI 90 ĐỘ (Phanh khựng dứt khoát + Xoay tâm ổn định)
        // ---------------------------------------------------------------------
        case LINE_CASE_SHARP_RIGHT: {
            straightStreak = 0;
            unsigned long turnElapsed = millis() - sharpTurnStartTime;
            if (turnElapsed < 36) {
                motor_set_speeds(-50, -120);
            } else {
                motor_set_speeds(95, -100);
            }
            integralError = 0.0f;
            lastError = 15.0f;
            break;
        }

        // ---------------------------------------------------------------------
        // CASE 4: MẤT LINE
        // ---------------------------------------------------------------------
        case LINE_CASE_LOST_LINE: {
            straightStreak = 0;
            unsigned long lostElapsed = millis() - lostLineStartTime;
            if (lostElapsed > 1000) {
                motor_stop();
                break;
            }
            if (lastKnownDirection < 0) {
                motor_set_speeds(-88, 82);
            } else if (lastKnownDirection > 0) {
                motor_set_speeds(82, -88);
            } else {
                motor_set_speeds(-75, -75);
            }
            break;
        }

        // ---------------------------------------------------------------------
        // CASE 5: VẠCH NGANG / NGÃ TƯ / VẠCH XUẤT PHÁT
        // ---------------------------------------------------------------------
        case LINE_CASE_CROSS_LINE: {
            motor_set_speeds(pidConfig.baseSpeed, pidConfig.baseSpeed);
            break;
        }

        // ---------------------------------------------------------------------
        // CASE 6: NGÃ 3 CHỮ T BÊN TRÁI
        // ---------------------------------------------------------------------
        case LINE_CASE_T_LEFT: {
            straightStreak = 0;
            int16_t slowSpd = -(int16_t)(pidConfig.baseSpeed * 0.40f);
            motor_set_speeds(slowSpd, pidConfig.baseSpeed);
            break;
        }

        // ---------------------------------------------------------------------
        // CASE 7: NGÃ 3 CHỮ T BÊN PHẢI
        // ---------------------------------------------------------------------
        case LINE_CASE_T_RIGHT: {
            straightStreak = 0;
            int16_t slowSpd = -(int16_t)(pidConfig.baseSpeed * 0.40f);
            motor_set_speeds(pidConfig.baseSpeed, slowSpd);
            break;
        }

        // ---------------------------------------------------------------------
        // CASE 9: ĐOẠN NÉT ĐỨT (DASHED LINE)
        // ---------------------------------------------------------------------
        case LINE_CASE_DASHED_LINE: {
            motor_set_speeds(pidConfig.baseSpeed, pidConfig.baseSpeed);
            break;
        }

        // ---------------------------------------------------------------------
        // CASE 8: DỪNG XE HOẶC CHƯA HIỆU CHUẨN
        // ---------------------------------------------------------------------
        case LINE_CASE_STOPPED:
        default: {
            straightStreak = 0;
            motor_stop();
            break;
        }
    }
}

LineCase_t pid_get_current_case(void) {
    return currentCase;
}

const char* pid_get_case_name(LineCase_t c) {
    switch (c) {
        case LINE_CASE_NORMAL_PID: return "CASE 1: Bam Line PID";
        case LINE_CASE_SHARP_LEFT: return "CASE 2: Cua Gat Trai (90 deg)";
        case LINE_CASE_SHARP_RIGHT: return "CASE 3: Cua Gat Phai (90 deg)";
        case LINE_CASE_LOST_LINE: return "CASE 4: Mat Line (Tim Lai)";
        case LINE_CASE_CROSS_LINE: return "CASE 5: Vach Ngang / Nga Tu";
        case LINE_CASE_T_LEFT: return "CASE 6: Nga 3 Trai";
        case LINE_CASE_T_RIGHT: return "CASE 7: Nga 3 Phai";
        case LINE_CASE_STOPPED: return "CASE 8: Dung Xe";
        case LINE_CASE_DASHED_LINE: return "CASE 9: Doan Net Dut (Dashed Line)";
        default: return "Khong Xac Dinh";
    }
}

float pid_get_last_error(void) {
    return lastError;
}

float pid_get_correction(void) {
    return lastCorrection;
}

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
