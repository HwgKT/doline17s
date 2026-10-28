#pragma once
// Module telemetry qua Bluetooth Classic (SPP) cho ESP32 gốc.
// Gói tin: [0xAA 0x55][payload 69 byte little-endian][XOR checksum của payload]
// Khớp với struct format "<I16H2i2h5fB" trong robot_debug.py.
//
// Cách dùng trong src/main.cpp:
//
//   #include "bt_telemetry.h"
//
//   static void onCommand(const String &cmd) {
//     if (cmd.length() == 1) { /* handleCharCommand(cmd[0]); */ }
//     else if (cmd.startsWith("PID")) {
//       float kp, ki, kd; int base;
//       if (sscanf(cmd.c_str(), "PID %f %f %f %d", &kp, &ki, &kd, &base) == 4) { /* gán PID */ }
//     }
//   }
//
//   void setup() { btBegin("LineRobot"); /* ... */ }
//
//   void loop() {
//     /* ... đọc 16 mắt, PID, xuất PWM ... */
//     static uint32_t lastTx = 0;
//     if (millis() - lastTx >= 10) {            // 100 Hz
//       lastTx = millis();
//       Telemetry t;
//       for (int k = 0; k < 16; k++) t.adc[k] = sensorAdc[k];
//       t.encL = encLeft;  t.encR = encRight;
//       t.pwmL = pwmLeft;  t.pwmR = pwmRight;
//       t.err = pidErr; t.p = pidP; t.i = pidI; t.d = pidD; t.out = pidOut;
//       t.mode = autoMode ? 1 : 0;
//       btSendTelemetry(t);
//     }
//     btPollCommands(onCommand);
//   }

#include <Arduino.h>

#pragma pack(push, 1)
struct Telemetry {
  uint32_t t_ms;     // được điền tự động trong btSendTelemetry()
  uint16_t adc[16];  // ADC 12-bit của 16 mắt
  int32_t encL, encR;
  int16_t pwmL, pwmR;
  float err, p, i, d, out;
  uint8_t mode;      // 0 = manual, 1 = auto PID
};
#pragma pack(pop)

static_assert(sizeof(Telemetry) == 69, "Telemetry phai dung 69 byte");

typedef void (*BtCommandHandler)(const String &cmd);

void btBegin(const char *deviceName = "LineRobot");
bool btConnected();
void btSendTelemetry(const Telemetry &t);       // bỏ qua nếu chưa có PC kết nối
void btPollCommands(BtCommandHandler handler);  // lệnh kết thúc bằng '\n'
