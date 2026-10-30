#include <Arduino.h>
#include "button.h"
#include "pinout.h"

namespace {
constexpr uint32_t DEBOUNCE_MS=35;
constexpr uint32_t LED_STEP_MS=100;
// Original GPIO34 implementation detected HIGH as a press.
// GPIO35 uses the same external pull-down / active-HIGH wiring assumption.
constexpr int PRESSED_LEVEL=HIGH;
struct Key {
  uint8_t pin;
  bool stable=false;
  bool lastRaw=false;
  uint32_t changedAt=0;
};
Key keys[2]={{PIN_CALIB_BUTTON},{PIN_START_BUTTON}};
uint8_t ledTransitions=0;
bool ledOn=false;
uint32_t ledAt=0;
void ledWrite(bool state) {
  ledOn=state;
  digitalWrite(PIN_BUTTON_LED, state == (BUTTON_LED_ACTIVE_HIGH!=0) ? HIGH : LOW);
}
bool pressed(Key &key,uint32_t now) {
  const bool raw=digitalRead(key.pin)==PRESSED_LEVEL;
  if(raw!=key.lastRaw){key.lastRaw=raw;key.changedAt=now;}
  if(raw!=key.stable && (uint32_t)(now-key.changedAt)>=DEBOUNCE_MS){
    key.stable=raw;
    return raw;
  }
  return false;
}
}
void button_init(){
  const uint32_t now=millis();
  for(auto &key:keys){
    pinMode(key.pin,INPUT); // GPIO34 and GPIO35 have no internal pulls.
    key.lastRaw=key.stable=(digitalRead(key.pin)==PRESSED_LEVEL);
    key.changedAt=now;
  }
  pinMode(PIN_BUTTON_LED,OUTPUT);
  ledWrite(false);
}
ButtonEvent button_poll(uint32_t now){
  const bool calib=pressed(keys[0],now);
  const bool start=pressed(keys[1],now);
  if(calib)return BUTTON_CALIB;
  if(start)return BUTTON_START_STOP;
  return BUTTON_NONE;
}
void button_led_blink(uint8_t count){
  if(count==0){ledTransitions=0;ledWrite(false);return;}
  ledTransitions=(uint8_t)(2*count-1);
  ledAt=millis();
  ledWrite(true);
}
void button_led_update(uint32_t now){
  if(!ledTransitions || (uint32_t)(now-ledAt)<LED_STEP_MS)return;
  ledAt=now;
  ledWrite(!ledOn);
  --ledTransitions;
}
