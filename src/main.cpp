#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include "pinout.h"
#include "sensor.h"
#include "motor.h"
#include "encoder.h"
#include "pid_controller.h"
#include "bt_telemetry.h"
#include "perf_monitor.h"
#include "adc_bench.h"
#include "spp_debug.h"
#include "input_logic.h"

namespace {
// 300 Hz * 72 B = 21.6 kB/s. USB debug fully disabled.
constexpr uint32_t BLINK_MS=400;
constexpr uint32_t MANUAL_TIMEOUT_MS=750; // prevents uncontrolled manual driving
bool sppStarted=false, btPrevious=false, manualActive=false, ledOn=false;
bool actionLedOutput=false;
bool physicalCalibPending=false, physicalCalibBlackPending=false;
ButtonPressDebouncer calibButton,startButton;
AckLedBlink actionLed;
bool estopLatched=false; // Latching software E-STOP; only RESET_ESTOP can clear.
uint8_t manualSpeed=120;
uint16_t streamHz=300;
uint32_t streamPeriodUs=1000000UL/300;
uint32_t lastTxUs=0,lastDiagUs=0,lastBlink=0,lastManual=0;
uint32_t loopCount=0,pidCount=0,txCount=0,skippedSlots=0;
BtTxStats lastTxStats{};

void ledWrite(bool on) {
  ledOn=on;
  digitalWrite(PIN_DEBUG_LED,on==(DEBUG_LED_ACTIVE_HIGH!=0)?HIGH:LOW);
}
void actionLedWrite(bool on) {
  if(actionLedOutput==on)return;
  actionLedOutput=on;
  digitalWrite(PIN_ACTION_LED,on==(ACTION_LED_ACTIVE_HIGH!=0)?HIGH:LOW);
}
void actionLedBlinkSuccess() {
  actionLed.request(millis());
  actionLedWrite(actionLed.isOn());
}
void actionLedUpdate(uint32_t now) {
  actionLed.update(now);
  actionLedWrite(actionLed.isOn());
}
const char *startAuto() {
  if(pid_is_auto_mode())return "ALREADY_RUNNING";
  if(adcBenchActive())return "ADC_BENCH_ACTIVE";
  if(estopLatched)return "ESTOP_LATCHED";
  if(!sensor_adc_ready())return "ADC_NOT_READY";
  if(sensor_is_calibrating()||!calibrationReady||
     sensor_calibration_state()==CAL_WHITE_READY)return "CALIB_REQUIRED_OR_PENDING";
  manualActive=false;
  pid_set_auto_mode(true);
  return nullptr;
}
void reply(const String &line) { btSendLine(line); }
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
void command(const String &raw) {
  String s=raw;s.trim();s.toUpperCase();
  if(!s.length())return;
  // Highest-priority commands: E-STOP cuts PWM and prevents all new motion.
  if(s=="ESTOP"||s=="E_STOP"||s=="EMERGENCY_STOP"){
    estopLatched=true; safeStop();adcBenchCancel();
    reply("ESTOP: LATCHED");return;
  }
  if(s=="RESET_ESTOP"){
    safeStop();estopLatched=false; // Unlock only: motors remain stopped.
    reply("ESTOP: CLEARED (MOTORS OFF)");return;
  }
  // USB command echo intentionally disabled.
  if(s=="PING"){reply("PONG");return;}
  if(s=="?"||s=="HELP"){
    reply("HELP: PING STATUS DIAG PERF PID TASKS ADC_BENCH ADC_BENCH_STOP ADC_BENCH_STATUS AUTO START STOP ESTOP RESET_ESTOP PWM ENC ENC_RESET P CAL_WHITE CAL_BLACK CAL_STATUS CAL_DUMP KP:x KI:x KD:x BASE:n STREAM:0/20/50/100/200/300 F B L R");
    return;
  }
  if(s=="STREAM"){
    reply(String("STREAM Hz=")+streamHz+" FRAME_BYTES=72");return;
  }
  if(s.startsWith("STREAM:")){
    const String textHz=s.substring(7);
    const int hz=textHz.toInt();
    const bool valid=(textHz==String(hz))&&
       (hz==0||hz==20||hz==50||hz==100||hz==200||hz==300);
    if(!valid){reply("ERR: STREAM_RATE_USE_0_20_50_100_200_300");return;}
    streamHz=(uint16_t)hz;
    streamPeriodUs=streamHz?1000000UL/streamHz:0;
    lastTxUs=micros();
    btDropPendingTelemetry();
    reply(String("STREAM Hz=")+streamHz+" FRAME_BYTES=72");return;
  }
  if(s=="DIAG"){
    const uint32_t t=micros(),dt=t-lastDiagUs;
    const float sec=dt?dt*1e-6f:1.0f;
    const BtTxStats tx=btGetTxStats();
    int32_t encL=0,encR=0;encoder_get_counts(&encL,&encR);
    char b[290];
    snprintf(b,sizeof(b),
      "DIAG: bt=%d auto=%d target=%u loop=%.1fHz pid=%.1fHz queued=%.1fHz sent=%.1fHz "
      "drop=%lu failed=%lu missed=%lu pending=%lu encL=%ld encR=%ld "
      "case=%u line=%u lost=%lu reacq=%lu",
      btConnected()?1:0,pid_is_auto_mode()?1:0,(unsigned)streamHz,
      loopCount/sec,pidCount/sec,
      (tx.queued-lastTxStats.queued)/sec,(tx.sent-lastTxStats.sent)/sec,
      (unsigned long)(tx.dropped-lastTxStats.dropped),
      (unsigned long)(tx.failed-lastTxStats.failed),
      (unsigned long)skippedSlots,(unsigned long)tx.pending,
      (long)encL,(long)encR,
      (unsigned)pid_get_current_case(),pid_line_is_confirmed()?1U:0U,
      (unsigned long)pid_get_confirmed_line_loss_count(),
      (unsigned long)pid_get_line_reacquisition_count());
    lastDiagUs=t;loopCount=pidCount=txCount=0;
    lastTxStats=tx;skippedSlots=0;
    reply(b);return;
  }
  if(s=="PERF"){perfReport();return;}
  if(s=="TASKS"){perfTaskReport();return;}
  if(s=="ADC_BENCH"){
    if(sensor_is_calibrating()){reply("ERR: ADC_BENCH_CALIB_ACTIVE");return;}
    safeStop();
    reply(adcBenchStart()?"ADC_BENCH: STARTED (SPP RESULTS)":
                          "ERR: ADC_BENCH_BUSY_OR_INIT_FAILED");
    return;
  }
  if(s=="ADC_BENCH_STOP"){adcBenchCancel();reply("ADC_BENCH: STOPPED");return;}
  if(s=="ADC_BENCH_STATUS"){adcBenchStatus();return;}
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
    char b[220];snprintf(b,sizeof(b),"STATUS: bt=%d auto=%d estop=%d calib=%d calibrating=%d case=%d pwmL=%d pwmR=%d adc=%s adc_ok=%d adc_err=%lu",
      btConnected()?1:0,pid_is_auto_mode()?1:0,estopLatched?1:0,
      calibrationReady?1:0,sensor_is_calibrating()?1:0,(int)pid_get_current_case(),
      motor_get_left_speed(),motor_get_right_speed(),sensor_adc_mode(),
      sensor_adc_ready()?1:0,(unsigned long)sensor_adc_error_count());
    reply(b);return;
  }
  if(s=="P"||s=="SENSORS"){
    String b="SENSORS: ";
    for(int i=0;i<N_SENSORS;++i)b+=(sensor_is_black(i)?'1':'0');
    reply(b);return;
  }
  if(s=="S"||s=="STOP"||s=="M"||s=="MANUAL"){
    safeStop();adcBenchCancel();reply(s=="M"||s=="MANUAL"?"MODE: MANUAL":"MOTOR: STOP");return;
  }
  if(s=="A"||s=="AUTO"||s=="START"){
    const char *error=startAuto();
    if(error){
      if(String(error)=="ALREADY_RUNNING")reply("MODE: AUTO PID");
      else reply(String("ERR: ")+error);
      return;
    }
    reply("MODE: AUTO PID");return;
  }
  // Fixed-polarity 16-channel calibration: WHITE, then BLACK.
  if(s=="C"||s=="W"||s=="CALIB"||s=="CAL_WHITE"){
    if(adcBenchActive()){reply("ERR: ADC_BENCH_ACTIVE");return;}
    if(sensor_is_calibrating()){reply("CAL_BUSY");return;}
    safeStop();
    reply(sensor_capture_white()?"CAL_WHITE_ACCEPTED":"CAL_BUSY");return;
  }
  if(s=="CAL_BLACK"){
    if(adcBenchActive()){reply("ERR: ADC_BENCH_ACTIVE");return;}
    if(sensor_is_calibrating()){reply("CAL_BUSY");return;}
    safeStop();
    reply(sensor_capture_black()?"CAL_BLACK_ACCEPTED":"CAL_NEED_WHITE");return;
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
  if(s=="F"||s=="B"||s=="L"||s=="R"){
    if(adcBenchActive()){reply("ERR: ADC_BENCH_ACTIVE");return;}
    if(estopLatched){reply("ERR: ESTOP_LATCHED");return;}
    if(!sensor_adc_ready()){reply("ERR: ADC_NOT_READY");return;}
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
void connectionState(uint32_t now) {
  bool connected=sppStarted&&btConnected();
  if(connected!=btPrevious){
    if(connected){
      ledWrite(true);reply("[BT] CONNECTED: Windows SPP");
      reply("READY: LineRobot SPP v1 (72-byte frame)");showPid();
    }else{
      safeStop();adcBenchCancel();ledWrite(false);lastBlink=now;
      btDropPendingTelemetry(); // no replay after reconnect
    }
    btPrevious=connected;
  }
  if(connected){if(!ledOn)ledWrite(true);}
  else if(now-lastBlink>=BLINK_MS){lastBlink=now;ledWrite(!ledOn);}
}
void sendFrame(){
  if(!sppStarted||!btConnected())return;
  Telemetry t{};
  // Timestamp belongs to the single post-PID snapshot, not the Bluetooth write.
  t.t_ms=millis();
  for(int i=0;i<N_SENSORS;++i)t.adc[i]=sensorRaw[i];
  int32_t l=0,r=0;
  encoder_get_counts(&l,&r);
  t.encL=l;t.encR=r;
  t.pwmL=motor_get_left_speed();t.pwmR=motor_get_right_speed();
  t.err=pid_is_auto_mode()?pid_get_last_error():0.0f;
  t.p=pid_get_term_p();t.i=pid_get_term_i();t.d=pid_get_term_d();
  t.out=pid_is_auto_mode()?pid_get_correction():0.0f;
  t.mode=pid_is_auto_mode()?1:0;
  if(btSendTelemetry(t))++txCount;
}
} // namespace

void setup() {
  // USB Serial disabled. Debug/commands exclusively via Bluetooth SPP.
  pinMode(PIN_DEBUG_LED,OUTPUT);ledWrite(false);
  pinMode(PIN_ACTION_LED,OUTPUT);digitalWrite(PIN_ACTION_LED,LOW);actionLedOutput=false;
  pinMode(PIN_START_BUTTON,INPUT);
  sensor_init();motor_init();
  const uint32_t buttonInitTime=millis();
  calibButton.begin(digitalRead(PIN_CALIB_BUTTON)==HIGH,buttonInitTime);
  startButton.begin(digitalRead(PIN_START_BUTTON)==HIGH,buttonInitTime);
  encoder_init(); // mainv2 signed quadrature x4 on A and B channels
  pid_init(2.0f,0.0f,0.0f,100); // conservative baseline while validating sharp-turn behavior
  sppStarted=btBegin("LineRobot");
  lastTxUs=lastDiagUs=micros();
  lastTxStats=btGetTxStats();
  lastBlink=millis();
}
void loop() {
  const uint32_t loopStartUs=micros();
  perfLoopBegin(loopStartUs);
  uint32_t now=millis();++loopCount;
  actionLedUpdate(now);
  uint32_t stageStartUs=micros();
  connectionState(now);
  if(sppStarted)btPollCommands(command);
  perfRecord(PERF_COMM, micros()-stageStartUs);
  const bool calibPressed=calibButton.update(digitalRead(PIN_CALIB_BUTTON)==HIGH,now);
  const bool startPressed=startButton.update(digitalRead(PIN_START_BUTTON)==HIGH,now);
  if(calibPressed) sppLogLine("[BUTTON] GPIO34 PRESSED");
  if(startPressed) sppLogLine("[BUTTON] GPIO35 PRESSED");
  if(!adcBenchActive() && calibPressed && !sensor_is_calibrating()){
    safeStop();
    sensor_handle_calibration_step(); // press 1: WHITE, press 2: BLACK
    if(sensor_is_calibrating()){
      physicalCalibPending=true;
      physicalCalibBlackPending=sensor_calibration_state()==CAL_BLACK_RUNNING;
      sppLogLine(!physicalCalibBlackPending?
                 "[CALIB] WHITE START":"[CALIB] BLACK START");
    } else {
      sppLogLine("[CALIB] REJECTED: ADC unavailable or calibration state invalid");
    }
  }
  if(startPressed){
    const char *error=startAuto();
    if(!error){
      actionLedBlinkSuccess();
      sppLogLine("[AUTO] START SUCCESS");
      sppLogLine("[LED] ACK 3 BLINKS");
      reply("MODE: AUTO PID");
    } else {
      sppLogPrintf("[AUTO] START REJECTED: %s\n",error);
    }
  }
  const bool wasCalibrating=sensor_is_calibrating();
  stageStartUs=micros();
  if(adcBenchActive())adcBenchStep();
  else if(wasCalibrating)sensor_update_calibration_loop();
  else sensor_read_all();
  perfRecord(PERF_ADC, micros()-stageStartUs);
  if(!sensor_adc_ready() && (pid_is_auto_mode() || manualActive)){
    safeStop();
    reply("ERR: ADC_READ_FAULT_MOTORS_STOPPED");
  }
  switch(sensor_take_calibration_event()){
    case CAL_WHITE_STARTED: reply("CAL_WHITE_STARTED"); break;
    case CAL_WHITE_DONE:
      reply("CAL_WHITE_DONE");
      if(physicalCalibPending){
        physicalCalibPending=false;
        sppLogLine("[CALIB] WHITE SUCCESS");
        actionLedBlinkSuccess();sppLogLine("[LED] ACK 3 BLINKS");
      }
      break;
    case CAL_BLACK_STARTED: reply("CAL_BLACK_STARTED"); break;
    case CAL_SAVED:
      reply("CAL_OK_SAVED");showCalib();
      if(physicalCalibPending){
        physicalCalibPending=false;
        sppLogLine("[CALIB] BLACK SUCCESS");
        sppLogLine("[CALIB] SAVED TO NVS");
        actionLedBlinkSuccess();sppLogLine("[LED] ACK 3 BLINKS");
      }
      break;
    case CAL_ERROR:
      reply("CAL_FAILED");showCalib();
      if(physicalCalibPending){physicalCalibPending=false;
        sppLogLine(physicalCalibBlackPending?
                   "[CALIB] BLACK FAILED; RETRY BLACK":
                   "[CALIB] WHITE FAILED; RETRY WHITE");}
      break;
    default: break;
  }
  if(pid_is_auto_mode()&&!sensor_is_calibrating()){
    stageStartUs=micros();
    pid_line_follow_step();++pidCount;
    perfRecord(PERF_PID, micros()-stageStartUs);
  }
  if(manualActive&&now-lastManual>=MANUAL_TIMEOUT_MS){
    motor_stop();manualActive=false;reply("[SAFE] MANUAL DEADMAN: MOTOR STOP");
  }
  // No stale bursts; bounded nonblocking enqueue at the selected stream rate.
  const uint32_t txNow=micros();
  if(streamPeriodUs&&!adcBenchActive()){
    const uint32_t elapsed=txNow-lastTxUs;
    if(elapsed>=streamPeriodUs){
      const uint32_t slots=elapsed/streamPeriodUs;
      lastTxUs+=slots*streamPeriodUs;
      if(slots>1)skippedSlots+=slots-1;
      if(sppStarted&&btConnected()){
        stageStartUs=micros();
        sendFrame();
        perfRecord(PERF_TX,micros()-stageStartUs);
      }
    }
  }
  perfLoopEnd(micros()-loopStartUs);
  // Loop start-to-start time includes yield and background scheduler work.
  yield();
}
