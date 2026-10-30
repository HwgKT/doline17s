#include "input_logic.h"
#include <assert.h>

int main() {
  ButtonPressDebouncer calib, start;
  calib.begin(true, 0);
  start.begin(true, 0);

  // Bounce and a held press produce exactly one event.
  assert(!calib.update(false, 10));
  assert(!calib.update(true, 15));
  assert(!calib.update(false, 20));
  assert(!calib.update(false, 49));
  assert(calib.update(false, 50));
  assert(!calib.update(false, 500));

  // The second button is independent and requires release before another press.
  assert(!start.update(false, 40));
  assert(start.update(false, 70));
  assert(!start.update(false, 100));
  assert(!start.update(true, 110));
  assert(!start.update(true, 140));
  assert(!start.update(false, 150));
  assert(start.update(false, 180));

  AckLedBlink led;
  assert(!led.isOn());
  led.request(100);
  assert(led.isOn());
  led.update(299); assert(led.isOn());
  led.update(300); assert(!led.isOn());
  led.update(500); assert(led.isOn());
  led.update(700); assert(!led.isOn());
  led.update(900); assert(led.isOn());
  led.update(1100); assert(!led.isOn());
  led.update(2000); assert(!led.isOn());

  // A request during a pattern restarts the one-slot acknowledgement.
  led.request(2100); led.update(2300); assert(!led.isOn());
  return 0;
}
