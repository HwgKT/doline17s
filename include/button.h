#pragma once
#include <stdint.h>

// One active-low physical button on GPIO34 for WHITE -> BLACK -> START/STOP.
void button_init();
bool button_pressed(uint32_t now_ms);
