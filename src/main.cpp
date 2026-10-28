#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include "pinout.h"
#include "sensor.h"
#include "motor.h"
#include "pid_controller.h"
#include "bt_telemetry.h"

namespace {
constexpr uint32_t TX_PERIOD_MS=50;       // 20 Hz telemetry for bring-up
constexpr uint32_t DIAG_PERIOD_MS=1000;   // 1 Hz USB diagnostic
constexpr uint32_t BLINK_MS=400;
constexpr uint32_t MANUAL_TIMEOUT_MS=750; // prevents uncontrolled manual driving
volatile uint32_t leftPulses=0, rightPulses=0;
portMUX_TYPE encoderLock=portMUX_INITIALIZER_UNLOCKED;
bool sppStarted=false, btPrevious=false, manualActive=false, ledOn=false;
uint8_t manualSpeed=120;
uint32_t lastTx=0, lastDiag=0, lastBlink=0, lastManual=0;
uint32_t loopCount=0, pidCount=0, txCount=0;
String serialLine;

void IRAM_ATTR countLeft() {
  portENTER_CRITICAL_ISR(&encoderLock); ++leftPulses; portEXIT_CRITICAL_ISR(&encoderLock);
}
void IRAM_ATTR countRight() {
  portENTER_CRITICAL_ISR(&encoderLock); ++rightPulses; portEXIT_CRITICAL_ISR(&encoderLock);
}
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
  reply(sensor_get_last_calibration_result()?"CALIB THANH CONG":"CALIB CHUA DAT");
  reply(String("CALIB_STATUS: ready=")+(calibrationReady?"1":"0"));
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
    reply("HELP: PING PID PWM STATUS P A M S F B L R V:120 KP:2.2 KI:0 KD:0 BASE:120 C K INV 3S 2S RACE");
    return;
  }
  if(s=="PID"){showPid();return;}
  if(s=="PWM"){
    char b[110];snprintf(b,sizeof(b),"PWM: L=%d R=%d Err=%.2f Out=%.2f",
      motor_get_left_speed(),motor_get_right_speed(),
      pid_get_last_error(),pid_get_correction());
    reply(b);return;
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
    if(sensor_is_calibrating()||!calibrationReady){reply("ERR: CALIB_REQUIRED");return;}
    manualActive=false;pid_set_auto_mode(true);reply("MODE: AUTO PID");return;
  }
  if(s=="C"||s=="W"||s=="CALIB"||s=="SWEEP"){
    safeStop();sensor_start_sweep_calibration(4000);
    reply("CALIB: BAT DAU LIA (4s)");return;
  }
  if(s=="K"||s=="STOP_CALIB"){
    if(!sensor_is_calibrating()){reply("ERR: NO_ACTIVE_CALIB");return;}
    sensor_finish_sweep_calibration();showCalib();return;
  }
  if(s=="INV"||s=="INVERT"){
    sensor_set_polarity(!sensor_get_polarity());
    reply(String("POLARITY: ")+(sensor_get_polarity()?"DEN_ADC_CAO":"DEN_ADC_THAP"));return;
  }
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
  uint32_t l,r;
  portENTER_CRITICAL(&encoderLock);l=leftPulses;r=rightPulses;portEXIT_CRITICAL(&encoderLock);
  t.encL=(int32_t)l;t.encR=(int32_t)r;
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
  pinMode(PIN_ENCODER_L1,INPUT_PULLUP);pinMode(PIN_ENCODER_R1,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_L1),countLeft,RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_R1),countRight,RISING);
  pid_init(2.2f,0.0f,0.0f,120); // preserve seconds-based derivative default Kd=0
  sppStarted=btBegin("LineRobot");
  Serial.printf("[BT] %s | device=LineRobot\n",sppStarted?"SPP STARTED":"SPP INIT FAILED");
  Serial.println("[READY] USB 115200 newline; PING/PID/PWM/STATUS/C/K/A/M/S");
  lastTx=lastDiag=lastBlink=millis();
}
void loop() {
  uint32_t now=millis();++loopCount;
  connectionState(now);pollUSB();
  if(sppStarted)btPollCommands(command);
  if(sensor_is_button_pressed()){
    if(sensor_is_calibrating()){sensor_finish_sweep_calibration();showCalib();}
    else{safeStop();sensor_start_sweep_calibration(4000);reply("CALIB: BAT DAU LIA (button)");}
  }
  bool wasCalibrating=sensor_is_calibrating();
  if(wasCalibrating)sensor_update_calibration_loop();
  else sensor_read_all();
  if(wasCalibrating&&!sensor_is_calibrating())showCalib();
  if(pid_is_auto_mode()&&!sensor_is_calibrating()){pid_line_follow_step();++pidCount;}
  if(manualActive&&now-lastManual>=MANUAL_TIMEOUT_MS){
    motor_stop();manualActive=false;reply("[SAFE] MANUAL DEADMAN: MOTOR STOP");
  }
  if(now-lastTx>=TX_PERIOD_MS){lastTx=now;sendFrame();}
  if(now-lastDiag>=DIAG_PERIOD_MS){
    float sec=(now-lastDiag)/1000.0f;
    Serial.printf("[DIAG] bt=%d calib=%d auto=%d loop=%.1fHz pid=%.1fHz tx=%.1fHz\n",
      btConnected()?1:0,calibrationReady?1:0,pid_is_auto_mode()?1:0,
      loopCount/sec,pidCount/sec,txCount/sec);
    loopCount=pidCount=txCount=0;lastDiag=now;
  }
  delay(5);
}
