#ifndef BUTTON_H
#define BUTTON_H

#include <stdint.h>
#include "sensor.h"

// Return nullptr when AUTO was successfully entered; otherwise a reason string.
typedef const char *(*ButtonStartAutoFn)();
typedef void (*ButtonStopFn)();

struct ButtonCallbacks {
  ButtonStartAutoFn startAuto;
  ButtonStopFn safeStop;
};

// Physical buttons and success LED work independently of Bluetooth SPP.
void button_init();
void button_poll(uint32_t now, const ButtonCallbacks &callbacks);
void button_on_calibration_event(CalEvent event);
void button_update_led(uint32_t now);

#endif // BUTTON_H
