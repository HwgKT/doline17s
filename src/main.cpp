#include <Arduino.h>
#include "pinout.h"
#include "sensor.h"
#include "motor.h"
#include "encoder.h"
#include "ble_control.h"
#include "pid_controller.h"

static unsigned long lastPrintTime = 0;

void setup() {
  Serial.begin(115200);
  delay(500); // Chỉ chờ Serial lúc khởi động, không nằm trong vòng điều khiển.

  Serial.println("\n==============================================");
  Serial.println("  ESP32 XE DO LINE 16 MAT - BLE + MOTOR + PID");
  Serial.println("==============================================");

  sensor_init();
  motor_init();
  encoder_init();

  pid_init(2.2f, 0.0f, 10.0f, 120);
  ble_control_init("ESP32_LineRobot");

  Serial.println("\nSan sang!");
  Serial.println("- Dung nut CALIB tren mach HOAC Web/BLE de lia calib Min/Max.");
  Serial.println("- Lenh BLE/Web:");
  Serial.println("  + 'C' / 'W': Bat dau lia calib 4s | 'K': Chot som");
  Serial.println("  + 'INV': Dao cuc tinh cam bien (Den <-> Trang)");
  Serial.println("  + 'A': BAT chay tu dong PID | 'M': TAT ve lai tay");
  Serial.println("  + 'PWM': Xem PWM | 'PID': Xem PID | 'ENC_RESET': Dat encoder ve 0");
  Serial.println("  + 'KP:2.2' | 'KI:0' | 'KD:10' | 'BASE:120': Chinh thong so PID");
  Serial.println("  + 'F'/'B'/'L'/'R'/'S': Dieu khien tay tien/lui/trai/phai/dung");
}

void loop() {
  // Duy tri BLE; khong them delay co dinh vao vong lap.
  ble_control_update();

  if (sensor_is_button_pressed()) {
    sensor_handle_calibration_step();
  }

  // Trong calib, ham nay tu quet sensor va cap nhat Min/Max.
  sensor_update_calibration_loop();

  // Khi khong calib, quet lien tuc 16 mat cho PID.
  if (!sensor_is_calibrating()) {
    sensor_read_all();
  }

  if (pid_is_auto_mode()) {
    pid_line_follow_step();
  }

  // Telemetry gioi han 150 ms/lần de tranh Serial/BLE lam cham vong dieu khien.
  if (millis() - lastPrintTime >= 150) {
    lastPrintTime = millis();

    int16_t leftPwm = motor_get_left_speed();
    int16_t rightPwm = motor_get_right_speed();
    float err = pid_get_last_error();
    const char* caseName = pid_get_case_name(pid_get_current_case());

    int32_t leftEncoderCount = 0;
    int32_t rightEncoderCount = 0;
    encoder_get_counts(&leftEncoderCount, &rightEncoderCount);

    if (pid_is_auto_mode()) {
      Serial.printf("[%s] Err: %.1f | PWM L:%d R:%d | Corr: %.1f\n",
                    caseName, err, leftPwm, rightPwm, pid_get_correction());
    } else if (calibrationReady) {
      sensor_print_states();
    }

    Serial.printf("[ENC] L:%ld R:%ld\n",
                  static_cast<long>(leftEncoderCount),
                  static_cast<long>(rightEncoderCount));

    if (ble_is_connected()) {
      // Snapshot gom PWM, encoder, ADC raw va gia tri calibrated trong mot goi debug.
      ble_send_debug_telemetry(
          leftPwm, rightPwm, leftEncoderCount, rightEncoderCount);
    }
  }

  // Khong dung delay(5): vong lap chay lien tuc theo thoi gian quet ADC,
  // PID va cong viec BLE/telemetry.
}
