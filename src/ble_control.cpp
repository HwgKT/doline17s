#include "ble_control.h"
#include "motor.h"
#include "sensor.h"
#include "pid_controller.h"
#include "bt_telemetry.h"
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <string>
#include <string.h>

// Chuẩn dịch vụ Nordic UART (tương thích app Serial Bluetooth Terminal, nRF Connect, v.v.)
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

static BLEServer* pServer = nullptr;
static BLECharacteristic* pTxCharacteristic = nullptr;
static bool deviceConnected = false;
static bool oldDeviceConnected = false;

// Tốc độ điều khiển mặc định
static uint8_t currentSpeed = 120;

void ble_send_string(const char* message) {
    btSendText(message);
    if (deviceConnected && pTxCharacteristic != nullptr) {
        pTxCharacteristic->setValue((uint8_t*)message, strlen(message));
        pTxCharacteristic->notify();
    }
}

void ble_send_sensor_states(void) {
    char buffer[64];
    char stateStr[N_SENSORS + 1];

    for (int i = 0; i < N_SENSORS; i++) {
        stateStr[i] = sensor_is_black(i) ? '1' : '0';
    }
    stateStr[N_SENSORS] = '\0';

    snprintf(buffer, sizeof(buffer), "SENSORS: %s\n", stateStr);
    ble_send_string(buffer);
}

void ble_send_pwm_telemetry(int16_t left, int16_t right, float error, const char* caseName) {
    if (!deviceConnected || pTxCharacteristic == nullptr) return;
    char buffer[80];
    snprintf(buffer, sizeof(buffer), "PWM: L=%d R=%d | Err=%.1f | Case=%s\n",
             left, right, error, caseName ? caseName : "Unknown");
    pTxCharacteristic->setValue((uint8_t*)buffer, strlen(buffer));
    pTxCharacteristic->notify();
}

namespace {
constexpr uint8_t kDebugMagic = 0xA7;
constexpr uint8_t kDebugPayloadLength = 56;
constexpr uint8_t kDebugFragmentPayloadLength = 16;
constexpr uint8_t kDebugFragmentCount = 4;
uint8_t debugTelemetrySequence = 0;

void writeU16LE(uint8_t* destination, uint16_t value) {
    destination[0] = static_cast<uint8_t>(value & 0xFF);
    destination[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

void writeU32LE(uint8_t* destination, uint32_t value) {
    destination[0] = static_cast<uint8_t>(value & 0xFF);
    destination[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    destination[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
    destination[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

void packUnsignedBits(uint8_t* destination, size_t startBit,
                      uint16_t value, uint8_t bitWidth) {
    for (uint8_t bit = 0; bit < bitWidth; ++bit) {
        if (value & static_cast<uint16_t>(1U << bit)) {
            const size_t outputBit = startBit + bit;
            destination[outputBit >> 3] |=
                static_cast<uint8_t>(1U << (outputBit & 7));
        }
    }
}
}  // namespace

void ble_send_debug_telemetry(int16_t left, int16_t right,
                              uint32_t encoder_left, uint32_t encoder_right) {
    if (!deviceConnected || pTxCharacteristic == nullptr) return;

    // Payload: signed PWM L/R (4 B), encoder counters (8 B),
    // 16 raw ADC values packed at 12 bits (24 B), calibrated at 10 bits (20 B).
    uint8_t payload[kDebugPayloadLength] = {};
    writeU16LE(payload, static_cast<uint16_t>(left));
    writeU16LE(payload + 2, static_cast<uint16_t>(right));
    writeU32LE(payload + 4, encoder_left);
    writeU32LE(payload + 8, encoder_right);

    for (uint8_t i = 0; i < N_SENSORS; ++i) {
        const uint16_t raw = sensorRaw[i] > 4095 ? 4095 : sensorRaw[i];
        const uint16_t calibrated = sensor_get_calibrated(i);
        packUnsignedBits(payload, 12U * 8U + i * 12U, raw, 12);
        packUnsignedBits(payload, 36U * 8U + i * 10U,
                         calibrated > 1000 ? 1000 : calibrated, 10);
    }

    const uint8_t sequence = debugTelemetrySequence++;
    for (uint8_t fragment = 0; fragment < kDebugFragmentCount; ++fragment) {
        const uint8_t offset = fragment * kDebugFragmentPayloadLength;
        const uint8_t remaining = kDebugPayloadLength - offset;
        const uint8_t chunkLength = remaining > kDebugFragmentPayloadLength
            ? kDebugFragmentPayloadLength : remaining;
        uint8_t frame[3 + kDebugFragmentPayloadLength] = {};
        frame[0] = kDebugMagic;
        frame[1] = sequence;
        frame[2] = fragment;
        memcpy(frame + 3, payload + offset, chunkLength);
        pTxCharacteristic->setValue(frame, chunkLength + 3);
        pTxCharacteristic->notify();
        delay(1); // Let the BLE task submit each notification before the next one.
    }
}

uint8_t ble_get_current_speed(void) {
    return currentSpeed;
}

void ble_set_current_speed(uint8_t speed) {
    currentSpeed = speed;
}

// Xử lý các lệnh gửi từ điện thoại qua BLE
static void processBleCommand(const std::string& rxValue) {
    if (rxValue.empty()) return;

    // Chuẩn hóa: loại bỏ ký tự xuống dòng / khoảng trắng thừa
    std::string cmd = rxValue;
    while (!cmd.empty() && (cmd.back() == '\r' || cmd.back() == '\n' || cmd.back() == ' ')) {
        cmd.pop_back();
    }
    while (!cmd.empty() && cmd.front() == ' ') {
        cmd.erase(cmd.begin());
    }
    if (cmd.empty()) return;

    // Chuyển ký tự đầu thành chữ in hoa để dễ so khớp
    char firstChar = toupper(cmd[0]);

    // Fixed polarity: BLACK ADC always exceeds WHITE ADC.
    // CAL_WHITE and CAL_BLACK each capture 128 frames of all 16 channels.
    if (cmd == "CAL_WHITE" || cmd == "C" || cmd == "CALIB") {
        if (sensor_is_calibrating()) { ble_send_string("CAL_BUSY\n"); return; }
        pid_set_auto_mode(false);
        motor_stop();
        ble_send_string(sensor_capture_white() ? "CAL_WHITE_ACCEPTED\n" : "CAL_BUSY\n");
        return;
    }
    if (cmd == "CAL_BLACK") {
        if (sensor_calibration_state() != CAL_WHITE_READY) {
            ble_send_string("CAL_NEED_WHITE\n"); return;
        }
        pid_set_auto_mode(false);
        motor_stop();
        ble_send_string(sensor_capture_black() ? "CAL_BLACK_ACCEPTED\n" : "CAL_BUSY\n");
        return;
    }
    if (cmd == "CAL_STATUS") {
        char msg[80];
        snprintf(msg, sizeof(msg), "CAL_STATUS READY=%d STATE=%d\n",
                 calibrationReady ? 1 : 0, (int)sensor_calibration_state());
        ble_send_string(msg);
        return;
    }
    if (cmd == "CAL_DUMP") {
        sensor_print_calibration();
        if (!calibrationReady) ble_send_string("CAL_NO_PROFILE\n");
        else for (int i = 0; i < N_SENSORS; ++i) {
            char msg[64];
            snprintf(msg, sizeof(msg), "CAL_DATA %d %u %u %u\n", i,
                     minValues[i], maxValues[i], thresholds[i]);
            ble_send_string(msg);
            delay(4);
        }
        return;
    }
    if (cmd == "INV" || cmd == "INVERT") {
        ble_send_string("CAL_FIXED_POLARITY BLACK_ADC_HIGH\n");
        return;
    }
    if (cmd == "K" || cmd == "STOP_CALIB") {
        ble_send_string("CAL_USE_WHITE_THEN_BLACK\n");
        return;
    }
    if (cmd == "P" || cmd == "SENSORS" || cmd == "STATUS") {
        ble_send_sensor_states();
        return;
    }

    // ========================================================
    // 2. CHẾ ĐỘ TỰ ĐỘNG DÒ LINE (AUTO / MANUAL)
    // ========================================================

    // Lệnh BẬT tự động dò line: 'A' hoặc "AUTO"
    if (firstChar == 'A' || cmd == "AUTO") {
        if (!calibrationReady || sensor_is_calibrating() ||
            sensor_calibration_state() == CAL_WHITE_READY) {
            ble_send_string("CAL_NOT_READY_FOR_AUTO\n");
            return;
        }
        pid_set_auto_mode(true);
        ble_send_string("MODE: BAT CHAY TU DONG (PID LINE FOLLOWING)\n");
        return;
    }

    // Lệnh TẮT tự động dò line: 'M' hoặc "MANUAL"
    if (firstChar == 'M' || cmd == "MANUAL") {
        pid_set_auto_mode(false);
        ble_send_string("MODE: CHUYEN SANG LAI TAY (MANUAL)\n");
        return;
    }

    // ========================================================
    // 3. NHÓM LỆNH ĐIỀU KHIỂN ĐỘNG CƠ (MOTOR CONTROL THỦ CÔNG)
    // ========================================================
    // Lệnh DỪNG: "S", "STOP", "DUNG" (Kiểm tra chính xác để không xung đột với "SPEED")
    if (cmd == "S" || cmd == "s" || cmd == "DUNG" || cmd == "STOP") {
        pid_set_auto_mode(false);
        motor_stop();
        ble_send_string("MOTOR: STOP\n");
        return;
    }

    // Lệnh CÙNG TIẾN: "F", "FORWARD", "TIEN"
    if (cmd == "F" || cmd == "f" || cmd == "TIEN" || cmd == "FORWARD") {
        pid_set_auto_mode(false);
        motor_forward(currentSpeed);
        char msg[48];
        snprintf(msg, sizeof(msg), "MOTOR: TIEN [Speed: %d]\n", currentSpeed);
        ble_send_string(msg);
        return;
    }

    // Lệnh CÙNG LÙI: "B", "BACKWARD", "LUI"
    if (cmd == "B" || cmd == "b" || cmd == "LUI" || cmd == "BACKWARD") {
        pid_set_auto_mode(false);
        motor_backward(currentSpeed);
        char msg[48];
        snprintf(msg, sizeof(msg), "MOTOR: LUI [Speed: %d]\n", currentSpeed);
        ble_send_string(msg);
        return;
    }

    // Lệnh RẼ TRÁI: "L", "LEFT", "TRAI"
    if (cmd == "L" || cmd == "l" || cmd == "TRAI" || cmd == "LEFT") {
        pid_set_auto_mode(false);
        motor_set_speeds(-((int16_t)currentSpeed * 2 / 3), currentSpeed);
        char msg[48];
        snprintf(msg, sizeof(msg), "MOTOR: RE TRAI [Speed: %d]\n", currentSpeed);
        ble_send_string(msg);
        return;
    }

    // Lệnh RẼ PHẢI: "R", "RIGHT", "PHAI"
    if (cmd == "R" || cmd == "r" || cmd == "PHAI" || cmd == "RIGHT") {
        pid_set_auto_mode(false);
        motor_set_speeds(currentSpeed, -((int16_t)currentSpeed * 2 / 3));
        char msg[48];
        snprintf(msg, sizeof(msg), "MOTOR: RE PHAI [Speed: %d]\n", currentSpeed);
        ble_send_string(msg);
        return;
    }

    // ========================================================
    // 4. CÀI ĐẶT TỐC ĐỘ (SPEED) & ĐẢO CỰC CẢM BIẾN
    // ========================================================
    // Cú pháp: "V:200" hoặc "SPEED:150"
    if (cmd.rfind("V:", 0) == 0 || cmd.rfind("v:", 0) == 0) {
        int val = atoi(cmd.c_str() + 2);
        if (val >= 0 && val <= 255) {
            currentSpeed = (uint8_t)val;
            pid_set_base_speed((int16_t)val);
            char msg[48];
            snprintf(msg, sizeof(msg), "SPEED SET: %d\n", currentSpeed);
            ble_send_string(msg);
        }
        return;
    }
    if (cmd.rfind("SPEED:", 0) == 0 || cmd.rfind("speed:", 0) == 0) {
        int val = atoi(cmd.c_str() + 6);
        if (val >= 0 && val <= 255) {
            currentSpeed = (uint8_t)val;
            pid_set_base_speed((int16_t)val);
            char msg[48];
            snprintf(msg, sizeof(msg), "SPEED SET: %d\n", currentSpeed);
            ble_send_string(msg);
        }
        return;
    }

    // Tốc độ phím nhanh 0..9 (từ các ứng dụng RC Car)
    if (cmd.length() == 1 && cmd[0] >= '0' && cmd[0] <= '9') {
        int level = cmd[0] - '0';
        currentSpeed = (uint8_t)map(level, 0, 9, 0, 255);
        char msg[48];
        snprintf(msg, sizeof(msg), "SPEED LEVEL %d -> %d\n", level, currentSpeed);
        ble_send_string(msg);
        return;
    }

    // ========================================================
    // 5. XUẤT PWM VÀ TINH CHỈNH THÔNG SỐ PID TRỰC TIẾP
    // ========================================================
    // Lệnh xuất PWM tức thì: "PWM"
    if (cmd == "PWM" || cmd == "pwm") {
        char msg[64];
        snprintf(msg, sizeof(msg), "PWM: L=%d R=%d | Err=%.1f\n",
                 motor_get_left_speed(), motor_get_right_speed(), pid_get_last_error());
        ble_send_string(msg);
        return;
    }

    // Xem thông số PID hiện tại: "PID"
    if (cmd == "PID" || cmd == "pid") {
        char msg[80];
        snprintf(msg, sizeof(msg), "PID: Kp=%.2f Ki=%.2f Kd=%.2f Base=%d\n",
                 pid_get_kp(), pid_get_ki(), pid_get_kd(), pid_get_base_speed());
        ble_send_string(msg);
        return;
    }

    // Chỉnh Kp (Cú pháp: "KP:2.2")
    if (cmd.rfind("KP:", 0) == 0 || cmd.rfind("kp:", 0) == 0) {
        float val = atof(cmd.c_str() + 3);
        if (val >= 0.0f && val <= 50.0f) {
            pid_set_tunings(val, pid_get_ki(), pid_get_kd());
            char msg[64];
            snprintf(msg, sizeof(msg), "PID SET: Kp=%.2f (Kd=%.2f)\n", pid_get_kp(), pid_get_kd());
            ble_send_string(msg);
        }
        return;
    }

    // Chỉnh Ki (Cú pháp: "KI:0.0")
    if (cmd.rfind("KI:", 0) == 0 || cmd.rfind("ki:", 0) == 0) {
        float val = atof(cmd.c_str() + 3);
        if (val >= 0.0f && val <= 50.0f) {
            pid_set_tunings(pid_get_kp(), val, pid_get_kd());
            char msg[64];
            snprintf(msg, sizeof(msg), "PID SET: Ki=%.2f\n", pid_get_ki());
            ble_send_string(msg);
        }
        return;
    }

    // Chinh Kd (cu phap: "KD:0.05"; mac dinh KD:0; D dung dao ham / giay)
    if (cmd.rfind("KD:", 0) == 0 || cmd.rfind("kd:", 0) == 0) {
        float val = atof(cmd.c_str() + 3);
        if (val >= 0.0f && val <= 50.0f) {
            pid_set_tunings(pid_get_kp(), pid_get_ki(), val);
            char msg[64];
            snprintf(msg, sizeof(msg), "PID SET: Kd=%.2f (Kp=%.2f)\n", pid_get_kd(), pid_get_kp());
            ble_send_string(msg);
        }
        return;
    }

    // Chỉnh baseSpeed (Cú pháp: "BASE:120")
    if (cmd.rfind("BASE:", 0) == 0 || cmd.rfind("base:", 0) == 0) {
        int val = atoi(cmd.c_str() + 5);
        if (val >= 0 && val <= 255) {
            pid_set_base_speed(val);
            currentSpeed = (uint8_t)val;
            char msg[64];
            snprintf(msg, sizeof(msg), "PID SET: baseSpeed=%d\n", pid_get_base_speed());
            ble_send_string(msg);
        }
        return;
    }

    // Lệnh không nhận diện được
    ble_send_string("LENH KHONG HOP LE! Gui ? hoac HELP de xem tro giup.\n");
}

// Same command interface for BLE NUS and Bluetooth Classic SPP.
void ble_control_process_command(const char* command) {
    if (command) processBleCommand(std::string(command));
}

// Callback quản lý kết nối BLE
class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* server) override {
        deviceConnected = true;
        Serial.println("[BLE] Thiet bi da ket noi!");
    }

    void onDisconnect(BLEServer* server) override {
        deviceConnected = false;
        Serial.println("[BLE] Thiet bi da ngat ket noi -> Dung dong co & tat che do Auto.");
        pid_set_auto_mode(false); // An toàn tối đa: Tắt ngay chế độ tự động dò line
        motor_stop();             // Dừng ngay động cơ
    }
};

// Callback nhận dữ liệu ghi vào characteristic RX
class RxCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) override {
        std::string rxValue = pCharacteristic->getValue();
        if (!rxValue.empty()) {
            Serial.printf("[BLE RX] %s\n", rxValue.c_str());
            processBleCommand(rxValue);
        }
    }
};

void ble_control_init(const char* deviceName) {
    // Khởi tạo thiết bị BLE
    BLEDevice::init(deviceName);

    // Tạo BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    // Tạo Service theo chuẩn Nordic UART
    BLEService* pService = pServer->createService(SERVICE_UUID);

    // Characteristic TX (ESP32 gửi thông báo lên điện thoại)
    pTxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_TX,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pTxCharacteristic->addDescriptor(new BLE2902());

    // Characteristic RX (Nhận lệnh điều khiển từ điện thoại)
    BLECharacteristic* pRxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_RX,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
    );
    pRxCharacteristic->setCallbacks(new RxCallbacks());

    // Bắt đầu chạy Service
    pService->start();

    // Bắt đầu quảng bá BLE để điện thoại có thể quét thấy
    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();

    Serial.printf("[BLE] Da bat BLE quang ba voi ten: %s\n", deviceName);
}

void ble_control_update(void) {
    // Tự động quảng bá lại khi điện thoại ngắt kết nối
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // Đợi Bluetooth stack ổn định
        pServer->startAdvertising();
        Serial.println("[BLE] Bat dau quang ba lai...");
        oldDeviceConnected = deviceConnected;
    }

    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
        ble_send_string("KET NOI BLE THANH CONG!\n=== HUONG DAN ===\nA: Auto | M: Lai tay | S: Dung\nPWM: Xem PWM | PID: Xem PID\nKP:2.2 | KI:0 | KD:0 | BASE:120: Chinh PID\n");
    }
}

bool ble_is_connected(void) {
    return deviceConnected;
}
