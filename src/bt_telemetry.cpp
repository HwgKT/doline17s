#include "bt_telemetry.h"

#include <BluetoothSerial.h>  // có sẵn trong framework arduino-espressif32, không cần lib_deps

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error "Board/chip nay khong bat Bluetooth Classic. Can ESP32 goc (board = esp32dev), khong phai S3/C3/C6."
#endif
#if !defined(CONFIG_BT_SPP_ENABLED)
#error "Can bat SPP (Serial Port Profile)."
#endif

static BluetoothSerial SerialBT;

void btBegin(const char *deviceName) {
  SerialBT.begin(deviceName);
}

bool btConnected() {
  return SerialBT.hasClient();
}

void btSendTelemetry(const Telemetry &t) {
  if (!SerialBT.hasClient()) return;  // chưa có PC kết nối thì bỏ qua, tránh block vòng PID

  Telemetry out = t;
  out.t_ms = millis();

  uint8_t frame[2 + sizeof(Telemetry) + 1];
  frame[0] = 0xAA;
  frame[1] = 0x55;
  memcpy(frame + 2, &out, sizeof(out));

  uint8_t cs = 0;
  for (size_t k = 0; k < sizeof(out); k++) cs ^= frame[2 + k];
  frame[sizeof(frame) - 1] = cs;

  SerialBT.write(frame, sizeof(frame));  // gửi 1 lần cả gói
}

void btPollCommands(BtCommandHandler handler) {
  static String line;
  while (SerialBT.available()) {
    char c = (char)SerialBT.read();
    if (c == '\n' || c == '\r') {
      if (line.length() && handler) handler(line);
      line = "";
    } else if (line.length() < 64) {
      line += c;
    }
  }
}
