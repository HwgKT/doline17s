#include "bt_telemetry.h"
#include <BluetoothSerial.h>
#include <cstring>
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED) || !defined(CONFIG_BT_SPP_ENABLED)
#error "Bluetooth Classic SPP requires original ESP32 and Bluedroid"
#endif
static BluetoothSerial SerialBT;
bool btBegin(const char *name) { return SerialBT.begin(name); }
bool btConnected() { return SerialBT.hasClient(); }
void btSendLine(const String &line) {
  if (!btConnected()) return;
  SerialBT.print(line); SerialBT.print('\n');
}
void btSendTelemetry(const Telemetry &sample) {
  if (!btConnected()) return;
  Telemetry t=sample;  // preserve snapshot timestamp assigned in main.cpp
  uint8_t frame[2+sizeof(t)+1]; frame[0]=0xAA;frame[1]=0x55;
  memcpy(frame+2,&t,sizeof(t));
  uint8_t xorByte=0;
  for(size_t i=0;i<sizeof(t);++i)xorByte^=frame[2+i];
  frame[sizeof(frame)-1]=xorByte;
  SerialBT.write(frame,sizeof(frame));
}
void btPollCommands(BtCommandHandler handler) {
  static String line;
  uint16_t processed=0;
  while(SerialBT.available() && processed++<128) {
    char c=(char)SerialBT.read();
    if(c=='\n'||c=='\r') {
      if(line.length() && handler)handler(line);
      line="";
    } else if(c>=32 && c<=126 && line.length()<80)line+=c;
    else if(line.length()>=80)line="";
  }
}
