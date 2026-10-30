#pragma once
#include <stdint.h>
enum ButtonEvent : uint8_t { BUTTON_NONE=0, BUTTON_CALIB=1, BUTTON_START_STOP=2 };
void button_init();
ButtonEvent button_poll(uint32_t now_ms);
void button_led_blink(uint8_t count);
void button_led_update(uint32_t now_ms);
