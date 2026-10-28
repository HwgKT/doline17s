#include <Arduino.h>
#include "pinout.h"
#include "sensor.h"
#include "motor.h"
#include "ble_control.h"
#include "pid_controller.h"

namespace {
volatile uint32_t encoderLeftPulses = 0;
volatile uint32_t encoderRightPulses = 0;
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onEncoderLeftPulse() {
  portENTER_CRITICAL_ISR(&encoderMux);
  ++encoderLeftPulses;
  portEXIT_CRITICAL_ISR(&encoderMux);
}

void IRAM_ATTR onEncoderRightPulse() {
  portENTER_CRITICAL_ISR(&encoderMux);
  ++encoderRightPulses;
  portEXIT_CRITICAL_ISR(&encoderMux);
}
}  // namespace

unsigned long lastPrintTime = 0;

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n==============================================");
  Serial.println("  ESP32 XE DO LINE 16 MAT - BLE + MOTOR + PID");
  Serial.println("==============================================");

  // 1. Khởi tạo cảm biến dò line 16 mắt (CD74HC4067)
  sensor_init();

  // 2. Khởi tạo điều khiển 2 động cơ PWM
  motor_init();

  // One pulse input per wheel. Use 3.3 V-compatible encoder outputs.
  // Current speed counter uses the first output channel from each encoder.
  pinMode(PIN_ENCODER_L1, INPUT_PULLUP);
  pinMode(PIN_ENCODER_R1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_L1), onEncoderLeftPulse, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_R1), onEncoderRightPulse, RISING);

  // 3. Khởi tạo PID với cấu hình mặc định
  pid_init(2.2f, 0.0f, 0.0f, 120);

  // 4. Khởi tạo BLE Server điều khiển không dây
  ble_control_init("ESP32_LineRobot");

  Serial.println("\nSan sang!");
  Serial.println("- Dung nut CALIB tren mach HOAC Web/BLE de lia calib Min/Max.");
  Serial.println("- Lenh BLE/Web:");
  Serial.println("  + 'C' / 'W': Bat dau lia calib 4s | 'K': Chot som");
  Serial.println("  + 'INV': Dao cuc tinh cam bien (Den <-> Trang)");
  Serial.println("  + 'A': BAT chay tu dong PID | 'M': TAT ve lai tay");
  Serial.println("  + 'PWM': Xem toc do PWM 2 banh | 'PID': Xem thong so PID");
  Serial.println("  + 'KP:2.2' | 'KI:0' | 'KD:0' | 'BASE:120': Chinh thong so PID");
  Serial.println("  + 'F'/'B'/'L'/'R'/'S': Dieu khien tay tien/lui/trai/phai/dung");
}

void loop() {
  // 1. Duy trì kết nối và thông báo qua BLE
  ble_control_update();

  // 2. Kiểm tra nút nhấn vật lý (hoạt động song song với lệnh BLE/Web)
  if (sensor_is_button_pressed()) {
    sensor_handle_calibration_step();
  }

  // 3. Xử lý quét Min/Max nếu đang trong chế độ lia calib (trong hàm này đã có sensor_read_all)
  sensor_update_calibration_loop();

  // 4. Quét các mắt cảm biến khi ở chế độ bình thường (tránh quét 2 lần liên tiếp khi calib)
  if (!sensor_is_calibrating()) {
    sensor_read_all();
  }

  // 5. Nếu đang ở chế độ Tự động -> Chạy thuật toán PID xử lý từng Case
  if (pid_is_auto_mode()) {
    pid_line_follow_step();
  }

  // 6. Định kỳ in thông tin trạng thái ra Serial & xuất PWM qua BLE (150ms/lần)
  if (millis() - lastPrintTime >= 150) {
    lastPrintTime = millis();
    int16_t leftPwm = motor_get_left_speed();
    int16_t rightPwm = motor_get_right_speed();
    float err = pid_get_last_error();
    const char* caseName = pid_get_case_name(pid_get_current_case());

    if (pid_is_auto_mode()) {
      Serial.printf("[%s] Err: %.1f | PWM L:%d R:%d | Corr: %.1f\n",
                    caseName,
                    err,
                    leftPwm,
                    rightPwm,
                    pid_get_correction());
    } else if (calibrationReady) {
      sensor_print_states();
    }

    // Send one compact debug snapshot: PWM, pulse counters and both ADC arrays.
    if (ble_is_connected()) {
      uint32_t leftEncoderSnapshot;
      uint32_t rightEncoderSnapshot;
      portENTER_CRITICAL(&encoderMux);
      leftEncoderSnapshot = encoderLeftPulses;
      rightEncoderSnapshot = encoderRightPulses;
      portEXIT_CRITICAL(&encoderMux);

      ble_send_debug_telemetry(
          leftPwm, rightPwm, leftEncoderSnapshot, rightEncoderSnapshot);
    }
  }

  delay(5); // Chu kỳ lấy mẫu và điều khiển 5ms chuẩn ổn định
}
