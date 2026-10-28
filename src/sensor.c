#include "sensor.h"
#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>

// Định nghĩa các biến toàn cục
uint16_t sensorRaw[N_SENSORS];
uint16_t minValues[N_SENSORS];
uint16_t maxValues[N_SENSORS];
uint16_t thresholds[N_SENSORS];
bool blackIsHigher[N_SENSORS];
bool isCalibrating = false;
bool calibrationReady = false;

// Biến quản lý thời gian quét hiệu chuẩn
static unsigned long calibStartTime = 0;
static uint32_t calibDurationMs = 4000; // Mặc định lia trong 4 giây

// Biến nội bộ phục vụ chống rung nút nhấn
static int lastCalibReading = LOW;
static int calibButtonState = LOW;
static unsigned long lastCalibChangeTime = 0;

void sensor_init(void) {
    pinMode(PIN_MUX_S0, OUTPUT);
    pinMode(PIN_MUX_S1, OUTPUT);
    pinMode(PIN_MUX_S2, OUTPUT);
    pinMode(PIN_MUX_S3, OUTPUT);
    pinMode(PIN_CALIB_BUTTON, INPUT);

    // GPIO36 trên ESP32 là chân input-only
    analogReadResolution(12); // Độ phân giải 12-bit (0 - 4095)

    // Khởi tạo giá trị ban đầu cho các mắt
    for (int i = 0; i < N_SENSORS; i++) {
        minValues[i] = 4095;
        maxValues[i] = 0;
        thresholds[i] = 2048;
        blackIsHigher[i] = true;
    }
}

void sensor_mux_select(uint8_t channel) {
    digitalWrite(PIN_MUX_S0, channel & 0x01);
    digitalWrite(PIN_MUX_S1, (channel >> 1) & 0x01);
    digitalWrite(PIN_MUX_S2, (channel >> 2) & 0x01);
    digitalWrite(PIN_MUX_S3, (channel >> 3) & 0x01);
}

uint16_t sensor_read_mux(uint8_t channel) {
    sensor_mux_select(channel);
    delayMicroseconds(6); // Đợi MUX chốt kênh ổn định (tối ưu từ 20us -> 6us)

    analogRead(PIN_LINE_SIG); // Bỏ mẫu đầu tiên để xả điện dung ký sinh

    uint32_t sum = 0;
    for (int i = 0; i < ADC_SAMPLES; i++) {
        sum += analogRead(PIN_LINE_SIG);
        delayMicroseconds(3); // Tối ưu từ 10us -> 3us
    }

    return (uint16_t)(sum / ADC_SAMPLES);
}

void sensor_read_all(void) {
    // Chỉ đọc các mắt cần dùng (từ mắt 0 đến 15)
    for (int i = SENSOR_START_IDX; i <= SENSOR_END_IDX; i++) {
        sensorRaw[i] = sensor_read_mux(i);
    }
}

// Bắt đầu chế độ lia cảm biến qua lại
void sensor_start_sweep_calibration(uint32_t duration_ms) {
    isCalibrating = true;
    calibrationReady = false;
    calibDurationMs = duration_ms;
    calibStartTime = millis();

    // Reset lại min/max của các mắt 0..15
    for (int i = SENSOR_START_IDX; i <= SENSOR_END_IDX; i++) {
        minValues[i] = 4095;
        maxValues[i] = 0;
    }

    printf("\n======================================================\n");
    printf("[CALIB] BAT DAU LIA CAM BIEN QUA LAI TREN VACH LINE!\n");
    printf(">> Hay lia thanh cam bien qua lai trong %u giay...\n", (unsigned int)(duration_ms / 1000));
    printf("======================================================\n");
}

// Cập nhật Min/Max liên tục khi đang lia (gọi trong loop)
void sensor_update_calibration_loop(void) {
    if (!isCalibrating) return;

    // Đọc toàn bộ mắt
    sensor_read_all();

    // Cập nhật giá trị cực tiểu (Min) và cực đại (Max) cho từng mắt
    for (int i = SENSOR_START_IDX; i <= SENSOR_END_IDX; i++) {
        uint16_t val = sensorRaw[i];
        if (val < minValues[i]) minValues[i] = val;
        if (val > maxValues[i]) maxValues[i] = val;
    }

    // Kiểm tra nếu hết thời gian lia
    if (millis() - calibStartTime >= calibDurationMs) {
        sensor_finish_sweep_calibration();
    }
}

// Chốt kết thúc quá trình lia và tính ngưỡng trung bình (Min + Max) / 2
// Trả về true nếu hiệu chuẩn thành công, false nếu thất bại hoặc chưa bắt đầu.
bool sensor_finish_sweep_calibration(void) {
    // Bảo vệ: Chỉ cho phép kết thúc khi đang thực sự hiệu chuẩn
    if (!isCalibrating) {
        printf("[CALIB] CANH BAO: Khong co luot hieu chuan nao dang hoat dong. Lenh bi tu choi.\n");
        return false;
    }

    isCalibrating = false;

    // Dùng bộ đệm tạm để không ghi đè dữ liệu cũ hợp lệ khi lượt mới thất bại
    uint16_t tempThresholds[N_SENSORS];
    bool tempBlackIsHigher[N_SENSORS];
    bool valid = true;

    printf("\n=== KET QUA HIEU CHUAN (LIA CAM BIEN QUA LAI) ===\n");
    printf("Mat  |  Min ADC  |  Max ADC  | Delta ADC | Nguong TB\n");
    printf("--------------------------------------------------\n");

    for (int i = SENSOR_START_IDX; i <= SENSOR_END_IDX; i++) {
        // Kiểm tra thứ tự logic: max phải lớn hơn min trước khi trừ (tránh underflow uint16_t)
        if (maxValues[i] <= minValues[i]) {
            printf(" %2d  |   %4u    |   %4u    |    N/A    | CHUA QUA VACH! (max<=min)\n",
                   i, minValues[i], maxValues[i]);
            valid = false;
            continue;
        }

        // Tính delta bằng int32_t để đảm bảo an toàn kiểu dữ liệu, không underflow
        int32_t delta = (int32_t)maxValues[i] - (int32_t)minValues[i];

        // Kiểm tra phạm vi ADC hợp lệ (0..4095)
        if (minValues[i] > 4095 || maxValues[i] > 4095) {
            printf(" %2d  |   %4u    |   %4u    |   %4ld    | GIA TRI NGOAI PHAM VI ADC!\n",
                   i, minValues[i], maxValues[i], (long)delta);
            valid = false;
            continue;
        }

        // Kiểm tra độ chênh lệch tối thiểu
        if (delta < (int32_t)MIN_CALIBRATION_DELTA) {
            printf(" %2d  |   %4u    |   %4u    |   %4ld    | DELTA QUA NHO! (< %d)\n",
                   i, minValues[i], maxValues[i], (long)delta, MIN_CALIBRATION_DELTA);
            valid = false;
            continue;
        }

        // Lưu vào bộ đệm tạm, chưa ghi vào thresholds thật
        tempThresholds[i] = (uint16_t)(((uint32_t)minValues[i] + (uint32_t)maxValues[i]) / 2u);
        tempBlackIsHigher[i] = true; // Giá trị cao hơn ngưỡng là vạch đen

        printf(" %2d  |   %4u    |   %4u    |   %4ld    |   %4u (OK)\n",
               i, minValues[i], maxValues[i], (long)delta, tempThresholds[i]);
    }
    printf("--------------------------------------------------\n");
    
    if (valid) {
        // Chỉ commit dữ liệu mới khi TẤT CẢ các mắt đều hợp lệ
        for (int i = SENSOR_START_IDX; i <= SENSOR_END_IDX; i++) {
            thresholds[i] = tempThresholds[i];
            blackIsHigher[i] = tempBlackIsHigher[i];
        }
        calibrationReady = true;
        printf("[CALIB THANH CONG] Da tinh xong nguong trung binh tung mat! (0 = Trang, 1 = Den)\n\n");
        return true;
    } else {
        // Không cập nhật thresholds[] – dữ liệu cũ hợp lệ (nếu có) được giữ nguyên
        // calibrationReady KHÔNG được thay đổi để bảo toàn trạng thái dữ liệu cũ
        printf("[CALIB THAT BAI] Mot so mat khong du delta. Du lieu cu (neu co) duoc giu nguyen.\n");
        printf("[CALIB] Hay lia lai toan bo mat qua vach den va gui lenh ket thuc.\n\n");
        return false;
    }
}

bool sensor_is_calibrating(void) {
    return isCalibrating;
}

bool sensor_is_button_pressed(void) {
    int reading = digitalRead(PIN_CALIB_BUTTON);
    if (reading != lastCalibReading) {
        lastCalibChangeTime = millis();
    }

    bool pressed = false;
    if (millis() - lastCalibChangeTime > BUTTON_DEBOUNCE_MS &&
        reading != calibButtonState) {
        calibButtonState = reading;
        pressed = (calibButtonState == HIGH);
    }
    lastCalibReading = reading;
    return pressed;
}

void sensor_handle_calibration_step(void) {
    if (!isCalibrating) {
        // Bắt đầu lia trong 4 giây
        sensor_start_sweep_calibration(4000);
    } else {
        // Nếu đang lia mà bấm nút lần nữa -> Chốt kết thúc sớm
        sensor_finish_sweep_calibration();
    }
}

uint16_t sensor_get_calibrated(uint8_t index) {
    if (index < SENSOR_START_IDX || index > SENSOR_END_IDX) return 0;
    uint16_t minVal = minValues[index];
    uint16_t maxVal = maxValues[index];
    if (maxVal <= minVal + MIN_CALIBRATION_DELTA) return 0;

    uint16_t raw = sensorRaw[index];
    if (raw <= minVal) raw = minVal;
    if (raw >= maxVal) raw = maxVal;

    // Hỗ trợ cực tính: Nếu blackIsHigher là true thì ADC cao là đen (0 -> 1000)
    // Nếu false thì ADC thấp là đen (đảo ngược lại để 1000 luôn luôn đại diện cho vạch đen)
    if (blackIsHigher[index]) {
        return (uint16_t)(((uint32_t)(raw - minVal) * 1000UL) / (maxVal - minVal));
    } else {
        return (uint16_t)(((uint32_t)(maxVal - raw) * 1000UL) / (maxVal - minVal));
    }
}

void sensor_set_polarity(bool black_is_higher) {
    for (int i = 0; i < N_SENSORS; i++) {
        blackIsHigher[i] = black_is_higher;
    }
    printf("[SENSOR] Da doi cuc tinh: %s\n", black_is_higher ? "DEN_ADC_CAO" : "DEN_ADC_THAP");
}

bool sensor_get_polarity(void) {
    return blackIsHigher[SENSOR_START_IDX];
}

bool sensor_is_black(uint8_t index) {
    // Bỏ qua mắt ngoài phạm vi
    if (index < SENSOR_START_IDX || index > SENSOR_END_IDX) return false;

    // Ngưỡng 500/1000 tương đương với giá trị trung bình chính xác (min + max) / 2
    return (sensor_get_calibrated(index) >= 500);
}

uint16_t sensor_get_line_bits(void) {
    uint16_t bits = 0;
    for (int i = SENSOR_START_IDX; i <= SENSOR_END_IDX; i++) {
        if (sensor_is_black(i)) {
            bits |= (1 << i);
        }
    }
    return bits;
}

void sensor_print_states(void) {
    for (int i = 0; i < N_SENSORS; i++) {
        if (i < SENSOR_START_IDX || i > SENSOR_END_IDX) {
            printf("- "); // Mắt không dùng
        } else {
            printf("%c ", sensor_is_black(i) ? '1' : '0');
        }
    }
    printf("\n");
}
