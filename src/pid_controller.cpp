#include "pid_controller.h"
#include "sensor.h"
#include "motor.h"
#include <Arduino.h>

// Cấu hình dải mắt cảm biến sử dụng (Đầy đủ 16 mắt: từ mắt 0 đến 15)
#define SENSOR_START_IDX   0      // Bắt đầu từ mắt 0
#define SENSOR_END_IDX     15     // Kết thúc ở mắt 15 (Tổng cộng 16 mắt hoạt động)

// Trọng số vị trí đối xứng cho 16 mắt (Tâm vạch ở giữa mắt 7 và 8)
static const float SENSOR_WEIGHTS[N_SENSORS] = {
    -75.0f, -65.0f, -55.0f, -45.0f, -35.0f, -25.0f, -15.0f, -5.0f, // 8 mắt bên trái (0..7)
      5.0f,  15.0f,  25.0f,  35.0f,  45.0f,  55.0f,  65.0f,  75.0f  // 8 mắt bên phải (8..15)
};

// Cấu hình PID mặc định
static PID_Config_t pidConfig = {
    .Kp = 2.2f,
    .Ki = 0.0f,
    .Kd = 0.0f,
    .baseSpeed = 120,
    .maxSpeed = 220,
    .minSpeed = -95
};

static bool autoMode = false;
static LineCase_t currentCase = LINE_CASE_STOPPED;
static float lastError = 0.0f;
static float integralError = 0.0f;
static float lastCorrection = 0.0f;
static float lastPTerm = 0.0f, lastITerm = 0.0f, lastDTerm = 0.0f;
// Use a microsecond timebase for the derivative (dt is measured in seconds).
static uint32_t lastComputeTimeUs = 0;
static bool derivativeHistoryValid = false;
#define DASHED_LINE_TIMEOUT_MS  60    // Rút ngắn xuống 60ms để tránh phóng mù ra ngoài khi văng góc cua
#define SHARP_TURN_TIMEOUT_MS   450   // Thời gian tối đa giữ cua 90 độ trước khi tự thoát an toàn
#define SHARP_TURN_COOLDOWN_MS  80    // Thời gian miễn nhiễm sau khi thoát cua (tránh quay vòng tròn lặp lại)
#define CROSS_LINE_COOLDOWN_MS  120   // Thời gian miễn nhiễm sau ngã tư (tránh mép vạch ngang kích hoạt nhầm cua 90 độ)
#define SHARP_BRAKE_MS          35    // Thời gian phanh giật ngược (Active Brake) khử trớn quán tính khi bắt đầu cua
static unsigned long lostLineStartTime = 0;
static bool isLineCurrentlyLost = false;
static int8_t lastKnownDirection = 0; // -1: Trái, +1: Phải

// Biến chốt trạng thái rẽ 90 độ
static bool isTurningSharpLeft = false;
static bool isTurningSharpRight = false;
static bool hasClearedOldLine = false;
static bool isSharpCounterBraking = false;
static unsigned long sharpCounterBrakeStartTime = 0;
static int8_t sharpTurnSide = 0; // -1: Trái, +1: Phải
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
    lastComputeTimeUs = micros();
    derivativeHistoryValid = false;
    autoMode = false;
    currentCase = LINE_CASE_STOPPED;
    isLineCurrentlyLost = false;
    isTurningSharpLeft = false;
    isTurningSharpRight = false;
    hasClearedOldLine = false;
    isSharpCounterBraking = false;
    sharpTurnSide = 0;
    lastKnownPosition = 0.0f;
    lastSharpTurnExitTime = 0;
    lastCrossLineExitTime = 0;
}

void pid_set_tunings(float kp, float ki, float kd) {
    // Reinitialize the derivative history if Kd is changed at runtime.
    if (pidConfig.Kd != kd) derivativeHistoryValid = false;
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
    isSharpCounterBraking = false;
    sharpTurnSide = 0;
    lastKnownPosition = 0.0f;
    lastSharpTurnExitTime = 0;
    lastCrossLineExitTime = 0;
    if (!autoMode) {
        currentCase = LINE_CASE_STOPPED;
        motor_stop();
    } else {
        integralError = 0.0f;
        lastError = 0.0f;
        lastComputeTimeUs = micros();
        derivativeHistoryValid = false;
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
            // Match the exact per-eye black/white logic used by the bitmask.
            if (sensor_is_black(i)) {
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
    uint8_t totalBlack = __builtin_popcount(mask & 0b1111111111111111);

    // =========================================================================
    // 1. KIỂM TRA TRẠNG THÁI ĐANG GIỮ CUA 90 ĐỘ (LATCHED TURN)
    // =========================================================================
    // Kiểm tra pha 3: Phanh dập quán tính quay (Counter-Brake 18ms) để khóa cứng tâm vạch mới
    if (isSharpCounterBraking) {
        if (millis() - sharpCounterBrakeStartTime >= 18) {
            isSharpCounterBraking = false;
            lastSharpTurnExitTime = millis();
            lastKnownDirection = 0;
            lastError = 0.0f;
            integralError = 0.0f;
            return LINE_CASE_NORMAL_PID; // Khóa vào line mới siêu ổn định!
        }
        return (sharpTurnSide < 0) ? LINE_CASE_SHARP_LEFT : LINE_CASE_SHARP_RIGHT;
    }

    if (isTurningSharpLeft) {
        unsigned long turnElapsed = millis() - sharpTurnStartTime;

        // Bước 1: Xác nhận thanh cảm biến đã quét hoàn toàn ra khỏi vạch cũ (tránh thoát cua non)
        if (!hasClearedOldLine) {
            if ((mask & 0b0000001111000000) == 0 && turnElapsed >= 28) {
                hasClearedOldLine = true;
            }
        }

        // Bước 2: Khi đã rời vạch cũ (hoặc sau 42ms an toàn), đón bắt vạch line mới
        if (hasClearedOldLine || turnElapsed >= 42) {
            if ((mask & 0b0000001111000000) != 0 || (turnElapsed >= 75 && (mask & 0b0000011111100000) != 0)) {
                isTurningSharpLeft = false;
                // Kích hoạt Pha 3: Xung phanh ngược dập tắt 100% quán tính xoay
                isSharpCounterBraking = true;
                sharpCounterBrakeStartTime = millis();
                sharpTurnSide = -1;
                return LINE_CASE_SHARP_LEFT;
            }
        }

        if (turnElapsed > SHARP_TURN_TIMEOUT_MS) {
            isTurningSharpLeft = false;
            lastSharpTurnExitTime = millis();
            return LINE_CASE_NORMAL_PID;
        } else {
            return LINE_CASE_SHARP_LEFT;
        }
    }

    if (isTurningSharpRight) {
        unsigned long turnElapsed = millis() - sharpTurnStartTime;

        // Bước 1: Xác nhận thanh cảm biến đã quét hoàn toàn ra khỏi vạch cũ (tránh thoát cua non)
        if (!hasClearedOldLine) {
            if ((mask & 0b0000001111000000) == 0 && turnElapsed >= 28) {
                hasClearedOldLine = true;
            }
        }

        // Bước 2: Khi đã rời vạch cũ (hoặc sau 42ms an toàn), đón bắt vạch line mới
        if (hasClearedOldLine || turnElapsed >= 42) {
            if ((mask & 0b0000001111000000) != 0 || (turnElapsed >= 75 && (mask & 0b0000011111100000) != 0)) {
                isTurningSharpRight = false;
                // Kích hoạt Pha 3: Xung phanh ngược dập tắt 100% quán tính xoay
                isSharpCounterBraking = true;
                sharpCounterBrakeStartTime = millis();
                sharpTurnSide = 1;
                return LINE_CASE_SHARP_RIGHT;
            }
        }

        if (turnElapsed > SHARP_TURN_TIMEOUT_MS) {
            isTurningSharpRight = false;
            lastSharpTurnExitTime = millis();
            return LINE_CASE_NORMAL_PID;
        } else {
            return LINE_CASE_SHARP_RIGHT;
        }
    }

    // =========================================================================
    // 2. VẠCH NGANG / NGÃ TƯ / VẠCH ĐÍCH (Từ 11 mắt đen trở lên cùng lúc)
    // =========================================================================
    if (totalBlack >= 11) {
        lastCrossLineExitTime = millis(); // Ghi nhận để tránh mép vạch ngang kích hoạt nhầm cua 90 độ
        return LINE_CASE_CROSS_LINE;
    }

    // Cờ kiểm tra xem có được phép kích hoạt cua 90 độ không (chống kích hoạt lặp & chống nhầm sau ngã tư)
    bool canTriggerSharp = (millis() - lastSharpTurnExitTime >= SHARP_TURN_COOLDOWN_MS) &&
                           (millis() - lastCrossLineExitTime >= CROSS_LINE_COOLDOWN_MS);

    // =========================================================================
    // 3. MẤT LINE HOÀN TOÀN (Cả 14 mắt đều trắng tinh: blackCount == 0)
    // SỬ DỤNG BỘ NHỚ VỊ TRÍ CUỐI (LINE MEMORY) - BÍ KÍP ĐỘI VÔ ĐỊCH!
    // =========================================================================
    if (blackCount == 0) {
        if (!isLineCurrentlyLost) {
            isLineCurrentlyLost = true;
            lostLineStartTime = millis();
        }

        unsigned long lostElapsed = millis() - lostLineStartTime;

        // Nếu vừa mất vạch ở cực bên trái (lastKnownPosition < -35.0f)
        // -> Đây là CUA VUÔNG 90 ĐỘ TRÁI! Kích hoạt xoay đón vạch ngay!
        if (canTriggerSharp && lastKnownPosition < -35.0f) {
            isTurningSharpLeft = true;
            isTurningSharpRight = false;
            hasClearedOldLine = false;
            isSharpCounterBraking = false;
            sharpTurnStartTime = millis();
            lastKnownDirection = -1;
            return LINE_CASE_SHARP_LEFT;
        }

        // Nếu vừa mất vạch ở cực bên phải (lastKnownPosition > 35.0f)
        // -> Đây là CUA VUÔNG 90 ĐỘ PHẢI! Kích hoạt xoay đón vạch ngay!
        if (canTriggerSharp && lastKnownPosition > 35.0f) {
            isTurningSharpRight = true;
            isTurningSharpLeft = false;
            hasClearedOldLine = false;
            isSharpCounterBraking = false;
            sharpTurnStartTime = millis();
            lastKnownDirection = 1;
            return LINE_CASE_SHARP_RIGHT;
        }

        // Nếu trước đó đang bám thẳng ở giữa tâm (lastKnownDirection == 0) trong thời gian ngắn (<= 60ms)
        // -> Đây là ĐOẠN NÉT ĐỨT (Dashed line) -> Tiếp tục lướt thẳng qua
        if (lastKnownDirection == 0 && lostElapsed <= DASHED_LINE_TIMEOUT_MS) {
            return LINE_CASE_DASHED_LINE;
        }

        // Mất line bất thường -> Kích hoạt tìm lại line
        return LINE_CASE_LOST_LINE;
    }

    // Đã nhìn thấy vạch đen -> Reset cờ mất line
    isLineCurrentlyLost = false;

    // =========================================================================
    // 4. PHÁT HIỆN SỚM CUA 90 ĐỘ KHI VẠCH CẮT NGANG MÉP NGOÀI
    // =========================================================================
    uint8_t leftBits  = __builtin_popcount(mask & 0b0000000011111111); // 8 mắt trái (0..7)
    uint8_t rightBits = __builtin_popcount(mask & 0b1111111100000000); // 8 mắt phải (8..15)

    // Cua 90° Trái: Mép ngoài cùng (0..3) đen VÀ nửa phải trắng tinh
    if (canTriggerSharp && (mask & 0b0000000000001111) != 0 && leftBits >= 3 && rightBits == 0) {
        isTurningSharpLeft = true;
        isTurningSharpRight = false;
        hasClearedOldLine = false;
        isSharpCounterBraking = false;
        sharpTurnStartTime = millis();
        lastKnownDirection = -1;
        return LINE_CASE_SHARP_LEFT;
    }

    // Cua 90° Phải: Mép ngoài cùng (12..15) đen VÀ nửa trái trắng tinh
    if (canTriggerSharp && (mask & 0b1111000000000000) != 0 && rightBits >= 3 && leftBits == 0) {
        isTurningSharpRight = true;
        isTurningSharpLeft = false;
        hasClearedOldLine = false;
        isSharpCounterBraking = false;
        sharpTurnStartTime = millis();
        lastKnownDirection = 1;
        return LINE_CASE_SHARP_RIGHT;
    }

    // =========================================================================
    // 5. PHÁT HIỆN NGÃ 3 CHỮ T (CASE 6 & CASE 7)
    // =========================================================================
    // Ngã 3 chữ T Trái: Tâm có vạch thẳng VÀ mép trái (0..3) có vạch rẽ ngang VÀ nửa phải trắng
    if ((mask & 0b0000001111000000) != 0 && (mask & 0b0000000000001111) != 0 && rightBits <= 1) {
        return LINE_CASE_T_LEFT;
    }

    // Ngã 3 chữ T Phải: Tâm có vạch thẳng VÀ mép phải (12..15) có vạch rẽ ngang VÀ nửa trái trắng
    if ((mask & 0b0000001111000000) != 0 && (mask & 0b1111000000000000) != 0 && leftBits <= 1) {
        return LINE_CASE_T_RIGHT;
    }

    // =========================================================================
    // 6. TOÀN BỘ CÁC TRƯỜNG HỢP CÒN LẠI (Đường thẳng, cua nhẹ, cua chữ S, cua gấp):
    // -> ĐỀU ĐƯỢC ĐIỀU KHIỂN BẰNG THUẬT TOÁN PID TRỌNG TÂM LIÊN TỤC!
    // =========================================================================
    return LINE_CASE_NORMAL_PID;
}

void pid_line_follow_step(void) {
    if (!autoMode) {
        return;
    }

    float position = 0.0f;
    uint8_t blackCount = 0;
    pid_calculate_position(&position, &blackCount);

    // Nhan dien Case hien tai, ghi nho Case cu de tranh D kick.
    LineCase_t previousCase = currentCase;
    currentCase = pid_detect_case(position, blackCount);

    const uint32_t nowUs = micros();
    const float measuredDt = (float)(nowUs - lastComputeTimeUs) * 1e-6f;
    const bool dtValid = measuredDt > 0.0f && measuredDt <= 0.1f;
    // Retain the original fallback for I; do not differentiate invalid dt.
    const float dt = dtValid ? measuredDt : 0.01f;
    lastComputeTimeUs = nowUs;

    // Do not differentiate stale/error-memory values across line cases.
    if (previousCase != LINE_CASE_NORMAL_PID ||
        currentCase != LINE_CASE_NORMAL_PID) {
        derivativeHistoryValid = false;
    }

    // =========================================================================
    // XỬ LÝ TỪNG CASE TRƯỜNG HỢP CỤ THỂ
    // =========================================================================
    switch (currentCase) {

        // ---------------------------------------------------------------------
        // CASE 1: BÁM LINE CHUẨN PID (Đường thẳng và khúc cua vừa phải)
        // ---------------------------------------------------------------------
        case LINE_CASE_NORMAL_PID: {
            float error = position; // Sai so vi tri cua 16 mat (-75.0 den +75.0)
            float absErr = fabsf(error);

            // 1. Khâu tỷ lệ P phi tuyến (Non-linear P):
            // Ở giữa tâm (|error| <= 8): lái êm ru, không rung lắc
            // Càng lệch ra xa (|error| > 8): tự động tăng phản xạ bẻ lái để ép đầu xe kéo về tâm
            float pGain = pidConfig.Kp * (1.0f + 0.012f * absErr);
            float pTerm = pGain * error;

            // 2. Khâu tích phân I (Anti-windup: kẹp trong khoảng [-150, 150])
            integralError += error * dt;
            if (integralError > 150.0f) integralError = 150.0f;
            if (integralError < -150.0f) integralError = -150.0f;
            float iTerm = pidConfig.Ki * integralError;

            // 3. Khau D: Kd * (e[k] - e[k-1]) / dt, dt tinh bang giay.
            // Kd mac dinh = 0. Bo qua D o mau dau sau khi doi mode/case/Kd.
            float dTerm = 0.0f;
            if (pidConfig.Kd != 0.0f && derivativeHistoryValid && dtValid) {
                dTerm = pidConfig.Kd * (error - lastError) / measuredDt;
            }
            lastError = error;
            derivativeHistoryValid = true;

            // Tổng bù lái
            lastPTerm = pTerm; lastITerm = iTerm; lastDTerm = dTerm;
            lastCorrection = pTerm + iTerm + dTerm;

            // 4. Giảm tốc ôm cua chủ động (Dynamic Corner Drag):
            // Giảm nhẹ ga khi vào cua để bánh xe bám chặt mặt sàn, thu hẹp bán kính quay vòng
            int16_t currentBase = pidConfig.baseSpeed - (int16_t)(absErr * 0.55f);
            if (currentBase < 50) currentBase = 50;

            // Avoid float-to-int16 overflow with a large retuned D term.
            float driveCorrection = lastCorrection;
            if (driveCorrection > 512.0f) driveCorrection = 512.0f;
            if (driveCorrection < -512.0f) driveCorrection = -512.0f;
            int16_t leftSpeed  = currentBase + (int16_t)driveCorrection;
            int16_t rightSpeed = currentBase - (int16_t)driveCorrection;

            // Giới hạn tốc độ trong dải an toàn
            leftSpeed  = clamp_val(leftSpeed, pidConfig.minSpeed, pidConfig.maxSpeed);
            rightSpeed = clamp_val(rightSpeed, pidConfig.minSpeed, pidConfig.maxSpeed);

            motor_set_speeds(leftSpeed, rightSpeed);
            break;
        }

        // ---------------------------------------------------------------------
        // CASE 2: CUA GẮT TRÁI 90 ĐỘ (3 Pha: Hãm trớn -> Ngoặt nhanh -> Phanh dập quán tính)
        // ---------------------------------------------------------------------
        case LINE_CASE_SHARP_LEFT: {
            if (isSharpCounterBraking) {
                // Pha 3 (18ms): Xung phanh ngược ghìm chết đà quay, khóa cứng tâm vạch mới
                motor_set_speeds(45, -45);
            } else {
                unsigned long turnElapsed = millis() - sharpTurnStartTime;
                if (turnElapsed < 22) {
                    // Pha 1 (0-22ms): Phanh giật bánh trong khử trớn lao thẳng, mớm góc quay
                    motor_set_speeds(-110, 35);
                } else {
                    // Pha 2 (>22ms): Xoay tâm nhanh dứt khoát nhưng bám sàn, không trượt lốp
                    motor_set_speeds(-105, 115);
                }
            }
            integralError = 0.0f;
            lastError = -10.0f;
            break;
        }

        // ---------------------------------------------------------------------
        // CASE 3: CUA GẮT PHẢI 90 ĐỘ (3 Pha: Hãm trớn -> Ngoặt nhanh -> Phanh dập quán tính)
        // ---------------------------------------------------------------------
        case LINE_CASE_SHARP_RIGHT: {
            if (isSharpCounterBraking) {
                // Pha 3 (18ms): Xung phanh ngược ghìm chết đà quay, khóa cứng tâm vạch mới
                motor_set_speeds(-45, 45);
            } else {
                unsigned long turnElapsed = millis() - sharpTurnStartTime;
                if (turnElapsed < 22) {
                    // Pha 1 (0-22ms): Phanh giật bánh trong khử trớn lao thẳng, mớm góc quay
                    motor_set_speeds(35, -110);
                } else {
                    // Pha 2 (>22ms): Xoay tâm nhanh dứt khoát nhưng bám sàn, không trượt lốp
                    motor_set_speeds(115, -105);
                }
            }
            integralError = 0.0f;
            lastError = 10.0f;
            break;
        }

        // ---------------------------------------------------------------------
        // CASE 4: MẤT LINE (16 mắt đều trắng)
        // ---------------------------------------------------------------------
        case LINE_CASE_LOST_LINE: {
            unsigned long lostElapsed = millis() - lostLineStartTime;

            // Nếu mất line quá 1.2 giây mà không thấy -> Dừng xe để an toàn
            if (lostElapsed > 1200) {
                motor_stop();
                break;
            }

            // Xoay êm ái tại chỗ tìm lại line với tốc độ vừa phải
            if (lastKnownDirection < 0) {
                motor_set_speeds(-75, 70);
            } else if (lastKnownDirection > 0) {
                motor_set_speeds(70, -75);
            } else {
                motor_set_speeds(-65, -65);
            }
            break;
        }

        // ---------------------------------------------------------------------
        // CASE 5: VẠCH NGANG / NGÃ TƯ / VẠCH XUẤT PHÁT
        // ---------------------------------------------------------------------
        case LINE_CASE_CROSS_LINE: {
            // Giữ hướng chạy thẳng vượt qua giao lộ với tốc độ cơ bản
            motor_set_speeds(pidConfig.baseSpeed, pidConfig.baseSpeed);
            break;
        }

        // ---------------------------------------------------------------------
        // CASE 6: NGÃ 3 CHỮ T BÊN TRÁI
        // ---------------------------------------------------------------------
        case LINE_CASE_T_LEFT: {
            // Ưu tiên rẽ trái ôm cua
            int16_t slowSpd = -(int16_t)(pidConfig.baseSpeed * 0.4f);
            motor_set_speeds(slowSpd, pidConfig.baseSpeed);
            break;
        }

        // ---------------------------------------------------------------------
        // CASE 7: NGÃ 3 CHỮ T BÊN PHẢI
        // ---------------------------------------------------------------------
        case LINE_CASE_T_RIGHT: {
            // Ưu tiên rẽ phải ôm cua
            int16_t slowSpd = -(int16_t)(pidConfig.baseSpeed * 0.4f);
            motor_set_speeds(pidConfig.baseSpeed, slowSpd);
            break;
        }

        // ---------------------------------------------------------------------
        // CASE 9: ĐOẠN NÉT ĐỨT (DASHED LINE)
        // ---------------------------------------------------------------------
        case LINE_CASE_DASHED_LINE: {
            // Xe duy trì chạy thẳng với tốc độ baseSpeed để lướt qua khoảng trắng đón đoạn vạch tiếp theo
            motor_set_speeds(pidConfig.baseSpeed, pidConfig.baseSpeed);
            break;
        }

        // ---------------------------------------------------------------------
        // CASE 8: DỪNG XE HOẶC CHƯA HIỆU CHUẨN
        // ---------------------------------------------------------------------
        case LINE_CASE_STOPPED:
        default: {
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
