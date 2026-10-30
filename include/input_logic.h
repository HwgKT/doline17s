#ifndef INPUT_LOGIC_H
#define INPUT_LOGIC_H

#include <stdint.h>

class ButtonPressDebouncer {
 public:
  void begin(bool rawHigh, uint32_t now) {
    candidateHigh_ = stableHigh_ = rawHigh;
    changedAt_ = now;
    initialized_ = true;
  }

  // Returns once per debounced HIGH-to-LOW press edge.
  bool update(bool rawHigh, uint32_t now) {
    if (!initialized_) begin(rawHigh, now);
    if (rawHigh != candidateHigh_) {
      candidateHigh_ = rawHigh;
      changedAt_ = now;
    }
    if (now - changedAt_ < kDebounceMs || candidateHigh_ == stableHigh_)
      return false;
    stableHigh_ = candidateHigh_;
    return !stableHigh_;
  }

  static const uint32_t kDebounceMs = 30;

 private:
  bool candidateHigh_ = true;
  bool stableHigh_ = true;
  bool initialized_ = false;
  uint32_t changedAt_ = 0;
};

class AckLedBlink {
 public:
  // One active pattern; a new request restarts it instead of queuing.
  void request(uint32_t now) {
    on_ = true;
    transitions_ = 5;  // ON, OFF, ON, OFF, ON, OFF = three flashes
    deadline_ = now + kStepMs;
  }

  void update(uint32_t now) {
    if (!transitions_ || static_cast<int32_t>(now - deadline_) < 0) return;
    on_ = !on_;
    --transitions_;
    if (transitions_) deadline_ = now + kStepMs;
    else on_ = false;
  }

  bool isOn() const { return on_; }
  static const uint32_t kStepMs = 200;

 private:
  bool on_ = false;
  uint8_t transitions_ = 0;
  uint32_t deadline_ = 0;
};

#endif
