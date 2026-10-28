#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include "pinout.h"
#include "sensor.h"
#include "motor.h"
#include "encoder.h"
#include "pid_controller.h"
#include "bt_telemetry.h"

namespace {
constexpr uint32_t TX_PERIOD_MS=50;       // 20 Hz telemetry for bring-up
constexpr uint32_t DIAG_PERIOD_MS=1000;   // 1 Hz USB diagnostic
constexpr uint32_t BLINK_MS=400;
constexpr uint32_t MANUAL_TIMEOUT_MS=750; // prevents uncontrolled manual driving
bool sppStarted=false, btPrevious=false, manualActive=false, ledOn=false;
uint8_t manualSpeed=120;
uint32_t lastTx=0, lastDiag=0, lastBlink=0, lastManual=0;
uint32_t loopCount=0, pidCount=0, txCount=0;
String serialLine;

void ledWrite(bool on) {
  ledOn=on;
  digitalWrite(PIN_DEBUG_LED,on==(DEBUG_LED_ACTIVE_HIGH!=0)?HIGH:LOW);
}
void reply(const String &line) {
  Serial.println(line);
  btSendLine(line);
}
void safeStop() {
  pid_set_auto_mode(false); motor_stop(); manualActive=false;
}
bool parseGain(const String &s,float &val) {
  if(!s.length())return false;
  char *end=nullptr;
  float v=strtof(s.c_str(),&end);
  if(end==s.c_str()||*end||!isfinite(v)||v<0||v>50)return false;
  val=v;return true;
}
bool parseSpeed(const String &s,int &val) {
  if(!s.length())return false;
  char *end=nullptr;
  long v=strtol(s.c_str(),&end,10);
  if(end==s.c_str()||*end||v<0||v>255)return false;
  val=(int)v;return true;
}
void showPid() {
  char b[100];
  snprintf(b,sizeof(b),"PID: Kp=%.2f Ki=%.2f Kd=%.2f Base=%d",
           pid_get_kp(),pid_get_ki(),pid_get_kd(),pid_get_base_speed());
  reply(b);
}
void showCalib() {
  reply(String("CAL_STATUS READY=")+(calibrationReady?"1":"0")+
        " STATE="+String((int)sensor_calibration_state()));
}
void setPreset(float kp,int base,const char *name) {
  // Old Kd=3.4/3.9/10 presets do not apply after the /dt derivative fix.
  pid_set_tunings(kp,0.0f,0.0f); pid_set_base_speed(base);
  manualSpeed=(uint8_t)base;
  reply(String("PRESET: ")+name+" (Kd=0, tune D again)");
  showPid();
}
void command(const String &raw) {
  String s=raw;s.trim();s.toUpperCase();
  if(!s.length())return;
  Serial.printf("[CMD] %s\n",s.c_str());
  if(s=="PING"){reply("PONG");return;}
  if(s=="?"||s=="HELP"){
    reply("HELP: PING PID PWM ENC ENC_RESET STATUS P A M S F B L R V:120 KP:2.2 KI:0 KD:0 BASE:120 CAL_WHITE CAL_BLACK CAL_STATUS CAL_DUMP 3S 2S RACE");
    return;
  }
  if(s=="PID"){showPid();return;}
  if(s=="PWM"){
    char b[110];snprintf(b,sizeof(b),"PWM: L=%d R=%d Err=%.2f Out=%.2f",
      motor_get_left_speed(),motor_get_right_speed(),
      pid_get_last_error(),pid_get_correction());
    reply(b);return;
  }
  if(s=="ENC"||s=="ENCODER"){
    int32_t l=0,r=0;
    encoder_get_counts(&l,&r);
    char b[92];
    snprintf(b,sizeof(b),"ENC: L=%ld R=%ld (signed x4)",(long)l,(long)r);
    reply(b);return;
  }
  if(s=="ENC_RESET"||s=="RESET_ENCODER"){
    encoder_reset_counts();
    reply("ENC_RESET: OK");return;
  }
  if(s=="STATUS"){
    char b[150];snprintf(b,sizeof(b),"STATUS: bt=%d auto=%d calib=%d calibrating=%d case=%d pwmL=%d pwmR=%d",
      btConnected()?1:0,pid_is_auto_mode()?1:0,calibrationReady?1:0,
      sensor_is_calibrating()?1:0,(int)pid_get_current_case(),
      motor_get_left_speed(),motor_get_right_speed());
    reply(b);return;
  }
  if(s=="P"||s=="SENSORS"){
    String b="SENSORS: ";
    for(int i=0;i<N_SENSORS;++i)b+=(sensor_is_black(i)?'1':'0');
    reply(b);return;
  }
  if(s=="S"||s=="STOP"||s=="M"||s=="MANUAL"){
    safeStop();reply(s=="M"||s=="MANUAL"?"MODE: MANUAL":"MOTOR: STOP");return;
  }
  if(s=="A"||s=="AUTO"){
    if(sensor_is_calibrating()||!calibrationReady||sensor_calibration_state()==CAL_WHITE_READY){reply("ERR: CALIB_REQUIRED_OR_PENDING");return;}
    manualActive=false;pid_set_auto_mode(true);reply("MODE: AUTO PID");return;
  }
  // Fixed-polarity 16-channel calibration: WHITE, then BLACK.
  if(s=="C"||s=="W"||s=="CALIB"||s=="CAL_WHITE"){
    if(sensor_is_calibrating()){reply("CAL_BUSY");return;}
    safeStop();
    reply(sensor_capture_white()?"CAL_WHITE_ACCEPTED":"CAL_BUSY");return;
  }
  if(s=="CAL_BLACK"){
    if(sensor_calibration_state()!=CAL_WHITE_READY){reply("CAL_NEED_WHITE");return;}
    safeStop();
    reply(sensor_capture_black()?"CAL_BLACK_ACCEPTED":"CAL_BUSY");return;
  }
  if(s=="CAL_STATUS"){showCalib();return;}
  if(s=="CAL_DUMP"){
    showCalib();
    if(!calibrationReady){reply("CAL_NO_PROFILE");return;}
    reply("CAL_DUMP_BEGIN");
    for(int i=0;i<N_SENSORS;++i){
      char line[80];
      snprintf(line,sizeof(line),"CAL_DATA %d %u %u %u",
          i,minValues[i],maxValues[i],thresholds[i]);
      reply(line);
    }
    reply("CAL_DUMP_DONE");return;
  }
  if(s=="K"||s=="STOP_CALIB"||s=="SWEEP"){reply("CAL_USE_WHITE_THEN_BLACK");return;}
  if(s=="INV"||s=="INVERT"){reply("CAL_FIXED_POLARITY BLACK_ADC_HIGH");return;}
  if(s.startsWith("KP:")||s.startsWith("KI:")||s.startsWith("KD:")){
    float v;
    if(!parseGain(s.substring(3),v)){reply("ERR: PID_VALUE_0_50");return;}
    float kp=pid_get_kp(),ki=pid_get_ki(),kd=pid_get_kd();
    if(s.startsWith("KP:"))kp=v;
    if(s.startsWith("KI:"))ki=v;
    if(s.startsWith("KD:"))kd=v;
    pid_set_tunings(kp,ki,kd);showPid();return;
  }
  if(s.startsWith("V:")||s.startsWith("BASE:")||s.startsWith("SPEED:")){
    int v;int sep=s.indexOf(':');
    if(sep<0||!parseSpeed(s.substring(sep+1),v)){reply("ERR: SPEED_0_255");return;}
    pid_set_base_speed(v);manualSpeed=(uint8_t)v;
    reply(String("SPEED SET: ")+v);return;
  }
  if(s=="BAM"||s=="GRIP"){setPreset(1.45f,105,"GRIP");return;}
  if(s=="3S"||s=="STAGE1"||s=="SAFE12"){setPreset(1.55f,115,"3S");return;}
  if(s=="2S"){setPreset(1.55f,145,"2S");return;}
  if(s=="RACE"||s=="RACE10"||s=="STAGE2"){setPreset(1.65f,135,"RACE");return;}
  if(s=="F"||s=="B"||s=="L"||s=="R"){
    safeStop();
    if(s=="F")motor_forward(manualSpeed);
    if(s=="B")motor_backward(manualSpeed);
    if(s=="L")motor_set_speeds(-(int16_t)manualSpeed*2/3,manualSpeed);
    if(s=="R")motor_set_speeds(manualSpeed,-(int16_t)manualSpeed*2/3);
    manualActive=true;lastManual=millis();
    reply(String("MOTOR: ")+s+" (deadman 750ms)");return;
  }
  reply(String("ERR: UNKNOWN_COMMAND ")+s);
}
void pollUSB() {
  uint16_t n=0;
  while(Serial.available()&&n++<128){
    char c=(char)Serial.read();
    if(c=='\n'||c=='\r'){
      if(serialLine.length())command(serialLine);
      serialLine="";
    }else if(c>=32&&c<=126&&serialLine.length()<80)serialLine+=c;
    else if(serialLine.length()>=80)serialLine="";
  }
}
void connectionState(uint32_t now) {
  bool connected=sppStarted&&btConnected();
  if(connected!=btPrevious){
    if(connected){
      ledWrite(true);reply("[BT] CONNECTED: Windows SPP");
      reply("READY: LineRobot SPP v1 (72-byte frame)");showPid();
    }else{
      safeStop();ledWrite(false);lastBlink=now;
      Serial.println("[BT] DISCONNECTED -> AUTO OFF, MOTOR STOP");
    }
    btPrevious=connected;
  }
  if(connected){if(!ledOn)ledWrite(true);}
  else if(now-lastBlink>=BLINK_MS){lastBlink=now;ledWrite(!ledOn);}
}
void sendFrame(){
  if(!sppStarted||!btConnected())return;
  Telemetry t{};
  for(int i=0;i<N_SENSORS;++i)t.adc[i]=sensorRaw[i];
  int32_t l=0,r=0;
  encoder_get_counts(&l,&r);
  t.encL=l;t.encR=r;
  t.pwmL=motor_get_left_speed();t.pwmR=motor_get_right_speed();
  t.err=pid_is_auto_mode()?pid_get_last_error():0.0f;
  t.p=pid_get_term_p();t.i=pid_get_term_i();t.d=pid_get_term_d();
  t.out=pid_is_auto_mode()?pid_get_correction():0.0f;
  t.mode=pid_is_auto_mode()?1:0;
  btSendTelemetry(t);++txCount;
}
} // namespace

void setup() {
  Serial.begin(115200);delay(300);
  pinMode(PIN_DEBUG_LED,OUTPUT);ledWrite(false);
  Serial.println("\n=== ESP32 mainv4 Bluetooth Classic SPP / USB Debug ===");
  Serial.printf("[LED] GPIO%d active_%s blink=waiting solid=connected\n",
    PIN_DEBUG_LED,DEBUG_LED_ACTIVE_HIGH?"HIGH":"LOW");
  sensor_init();motor_init();
  encoder_init(); // mainv2 signed quadrature x4 on A and B channels
  pid_init(2.2f,0.0f,0.0f,120); // preserve seconds-based derivative default Kd=0
  sppStarted=btBegin("LineRobot");
  Serial.printf("[BT] %s | device=LineRobot\n",sppStarted?"SPP STARTED":"SPP INIT FAILED");
  Serial.println("[READY] USB 115200 newline; PING/PID/STATUS/CAL_WHITE/CAL_BLACK/A/M/S");
  lastTx=lastDiag=lastBlink=millis();
}
void loop() {
  uint32_t now=millis();++loopCount;
  connectionState(now);pollUSB();
  if(sppStarted)btPollCommands(command);
  if(sensor_is_button_pressed() && !sensor_is_calibrating()){
    safeStop();
    sensor_handle_calibration_step(); // press 1: WHITE, press 2: BLACK
  }
  const bool wasCalibrating=sensor_is_calibrating();
  if(wasCalibrating)sensor_update_calibration_loop();
  else sensor_read_all();
  switch(sensor_take_calibration_event()){
    case CAL_WHITE_STARTED: reply("CAL_WHITE_STARTED"); break;
    case CAL_WHITE_DONE:    reply("CAL_WHITE_DONE"); break;
    case CAL_BLACK_STARTED: reply("CAL_BLACK_STARTED"); break;
    case CAL_SAVED:         reply("CAL_OK_SAVED");showCalib(); break;
    case CAL_ERROR:         reply("CAL_FAILED");showCalib(); break;
    default: break;
  }
  if(pid_is_auto_mode()&&!sensor_is_calibrating()){pid_line_follow_step();++pidCount;}
  if(manualActive&&now-lastManual>=MANUAL_TIMEOUT_MS){
    motor_stop();manualActive=false;reply("[SAFE] MANUAL DEADMAN: MOTOR STOP");
  }
  if(now-lastTx>=TX_PERIOD_MS){lastTx=now;sendFrame();}
  if(now-lastDiag>=DIAG_PERIOD_MS){
    float sec=(now-lastDiag)/1000.0f;
    int32_t encL=0,encR=0;
    encoder_get_counts(&encL,&encR);
    Serial.printf("[DIAG] bt=%d calib=%d auto=%d loop=%.1fHz pid=%.1fHz tx=%.1fHz encL=%ld encR=%ld\n",
      btConnected()?1:0,calibrationReady?1:0,pid_is_auto_mode()?1:0,
      loopCount/sec,pidCount/sec,txCount/sec,(long)encL,(long)encR);
    loopCount=pidCount=txCount=0;lastDiag=now;
  }
  delay(5);
}
