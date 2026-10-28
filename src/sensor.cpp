#include "sensor.h"
#include <Arduino.h>
#include <Preferences.h>
#include <stddef.h>
#include <string.h>

uint16_t sensorRaw[N_SENSORS] = {};
uint16_t minValues[N_SENSORS] = {};
uint16_t maxValues[N_SENSORS] = {};
uint16_t thresholds[N_SENSORS] = {};
bool calibrationReady = false;
bool isCalibrating = false;
static bool lastCalibrationOK = false;

namespace {
constexpr uint32_t MAGIC = 0x4C313643UL;
constexpr uint16_t VERSION = 1;
struct Profile {
  uint32_t magic;
  uint16_t version, count;
  uint16_t white[N_SENSORS], black[N_SENSORS];
  uint32_t crc;
};
static_assert(sizeof(Profile) == 76, "NVS profile size changed");
CalState state = CAL_EMPTY;
CalEvent pending = CAL_NONE;
uint32_t sums[N_SENSORS] = {};
uint64_t squares[N_SENSORS] = {};
uint16_t whiteStage[N_SENSORS] = {}, whiteNoise[N_SENSORS] = {};
uint32_t frames = 0;
int lastButtonRead = LOW, buttonState = LOW;
unsigned long lastButtonChange = 0;

uint32_t crc32(const uint8_t *p, size_t size) {
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t i = 0; i < size; ++i) {
    crc ^= p[i];
    for (uint8_t j = 0; j < 8; ++j)
      crc = (crc >> 1) ^ ((crc & 1U) ? 0xEDB88320UL : 0U);
  }
  return ~crc;
}
uint32_t signature(const Profile &p) {
  return crc32(reinterpret_cast<const uint8_t *>(&p), offsetof(Profile, crc));
}
bool valid(const Profile &p) {
  if (p.magic != MAGIC || p.version != VERSION || p.count != N_SENSORS ||
      signature(p) != p.crc) return false;
  for (int i = 0; i < N_SENSORS; ++i)
    if (p.white[i] > 4095 || p.black[i] > 4095 ||
        p.black[i] <= p.white[i] ||
        p.black[i] - p.white[i] < MIN_CALIBRATION_DELTA) return false;
  return true;
}
void apply(const Profile &p) {
  for (int i = 0; i < N_SENSORS; ++i) {
    minValues[i] = p.white[i];
    maxValues[i] = p.black[i];
    // Per-eye raw threshold = W_i + ceil((B_i - W_i)*400/1000).
    // Integer rounding makes raw>=TH_i equivalent to CAL_i>=400.
    const uint32_t contrast = uint32_t(p.black[i]) - p.white[i];
    thresholds[i] = uint16_t(uint32_t(p.white[i]) +
      (contrast * SENSOR_BLACK_THRESHOLD_CAL + 999U) / 1000U);
  }
  calibrationReady = true;
}
bool load() {
  Preferences pref;
  if (!pref.begin("line16cal", true)) return false;
  Profile p = {};
  const bool ok = pref.getBytesLength("profile") == sizeof(p) &&
    pref.getBytes("profile", &p, sizeof(p)) == sizeof(p);
  pref.end();
  if (!ok || !valid(p)) return false;
  apply(p);
  return true;
}
bool persist(const Profile &p) {
  Preferences pref;
  if (!pref.begin("line16cal", false)) return false;
  const bool ok = pref.putBytes("profile", &p, sizeof(p)) == sizeof(p);
  pref.end();
  return ok;
}
void resetAccum() {
  memset(sums, 0, sizeof(sums));
  memset(squares, 0, sizeof(squares));
  frames = 0;
}
void stats(uint16_t *mean, uint16_t *stddev) {
  const uint64_t n = frames;
  for (int i = 0; i < N_SENSORS; ++i) {
    mean[i] = (sums[i] + n / 2) / n;
    const uint64_t numerator = squares[i] * n - uint64_t(sums[i]) * sums[i];
    const uint64_t variance = numerator / (n * n);
    uint16_t sd = 0;
    while (uint32_t(sd) * sd < variance && sd < 4095) ++sd;
    stddev[i] = sd;
  }
}
void finish() {
  uint16_t mean[N_SENSORS], sd[N_SENSORS];
  stats(mean, sd);
  isCalibrating = false;
  if (state == CAL_WHITE_RUNNING) {
    memcpy(whiteStage, mean, sizeof(whiteStage));
    memcpy(whiteNoise, sd, sizeof(whiteNoise));
    state = CAL_WHITE_READY;
    pending = CAL_WHITE_DONE;
    Serial.println("[CAL] WHITE captured: 16 channels.");
    return;
  }
  Profile p = {};
  p.magic = MAGIC; p.version = VERSION; p.count = N_SENSORS;
  bool ok = true;
  Serial.println("[CAL] eye white black delta sigmaWhite sigmaBlack valid");
  for (int i = 0; i < N_SENSORS; ++i) {
    p.white[i] = whiteStage[i]; p.black[i] = mean[i];
    const int delta = int(mean[i]) - int(whiteStage[i]);
    const bool eyeOk = delta >= MIN_CALIBRATION_DELTA &&
        whiteNoise[i] <= CAL_MAX_STDDEV && sd[i] <= CAL_MAX_STDDEV;
    if (!eyeOk) ok = false;
    Serial.printf("[CAL] %02d %u %u %d %u %u %d\n",
      i, p.white[i], p.black[i], delta, whiteNoise[i], sd[i], eyeOk ? 1 : 0);
  }
  p.crc = signature(p);
  if (ok && persist(p)) {
    apply(p);
    lastCalibrationOK = true;
    state = CAL_READY; pending = CAL_SAVED;
    Serial.println("[CAL] CAL_OK_SAVED: NVS committed");
  } else {
    lastCalibrationOK = false;
    state = CAL_FAILED; pending = CAL_ERROR;
    Serial.println("[CAL] CAL_FAILED: previous RAM calibration retained");
  }
}
} // namespace

void sensor_init(void) {
  pinMode(PIN_MUX_S0, OUTPUT); pinMode(PIN_MUX_S1, OUTPUT);
  pinMode(PIN_MUX_S2, OUTPUT); pinMode(PIN_MUX_S3, OUTPUT);
  pinMode(PIN_CALIB_BUTTON, INPUT);
  analogReadResolution(12);
  calibrationReady = false;
  state = load() ? CAL_READY : CAL_EMPTY;
  Serial.printf("[CAL] boot: %s\n", calibrationReady ? "NVS_RESTORED" : "NO_VALID_PROFILE");
}
void sensor_mux_select(uint8_t ch) {
  digitalWrite(PIN_MUX_S0, ch & 1U);
  digitalWrite(PIN_MUX_S1, (ch >> 1) & 1U);
  digitalWrite(PIN_MUX_S2, (ch >> 2) & 1U);
  digitalWrite(PIN_MUX_S3, (ch >> 3) & 1U);
}
uint16_t sensor_read_mux(uint8_t ch) {
  sensor_mux_select(ch);
  delayMicroseconds(6);
  (void)analogRead(PIN_LINE_SIG);
  uint32_t sum = 0;
  for (int n = 0; n < ADC_SAMPLES; ++n) {
    sum += analogRead(PIN_LINE_SIG);
    delayMicroseconds(3);
  }
  return sum / ADC_SAMPLES;
}
void sensor_read_all(void) {
  for (int i = 0; i < N_SENSORS; ++i) sensorRaw[i] = sensor_read_mux(i);
}
bool sensor_capture_white(void) {
  if (isCalibrating) return false;
  lastCalibrationOK = false;
  resetAccum(); isCalibrating = true; state = CAL_WHITE_RUNNING; pending = CAL_WHITE_STARTED;
  Serial.println("[CAL] WHITE: hold all 16 sensors stationary on uniform WHITE");
  return true;
}
bool sensor_capture_black(void) {
  if (isCalibrating || state != CAL_WHITE_READY) return false;
  resetAccum(); isCalibrating = true; state = CAL_BLACK_RUNNING; pending = CAL_BLACK_STARTED;
  Serial.println("[CAL] BLACK: hold all 16 sensors stationary on uniform BLACK");
  return true;
}
void sensor_update_calibration_loop(void) {
  if (!isCalibrating) return;
  sensor_read_all();
  for (int i = 0; i < N_SENSORS; ++i) {
    uint32_t v = sensorRaw[i];
    sums[i] += v;
    squares[i] += uint64_t(v) * v;
  }
  if (++frames >= CAL_FRAMES) finish();
}
bool sensor_is_calibrating(void) { return isCalibrating; }
bool sensor_get_last_calibration_result(void) { return lastCalibrationOK; }
CalState sensor_calibration_state(void) { return state; }
CalEvent sensor_take_calibration_event(void) {
  CalEvent e = pending; pending = CAL_NONE; return e;
}
void sensor_print_calibration(void) {
  Serial.printf("[CAL] READY=%d STATE=%d\n", calibrationReady ? 1 : 0, int(state));
  if (!calibrationReady) return;
  for (int i = 0; i < N_SENSORS; ++i)
    Serial.printf("[CAL] eye=%02d W=%u B=%u TH=%u\n",
      i, minValues[i], maxValues[i], thresholds[i]);
}
bool sensor_is_button_pressed(void) {
  const int read = digitalRead(PIN_CALIB_BUTTON);
  if (read != lastButtonRead) lastButtonChange = millis();
  bool pressed = false;
  if (millis() - lastButtonChange > BUTTON_DEBOUNCE_MS && read != buttonState) {
    buttonState = read; pressed = buttonState == HIGH;
  }
  lastButtonRead = read;
  return pressed;
}
void sensor_handle_calibration_step(void) {
  if (isCalibrating) return;
  if (state == CAL_WHITE_READY) sensor_capture_black();
  else sensor_capture_white();
}
uint16_t sensor_get_calibrated(uint8_t index) {
  if (!calibrationReady || index >= N_SENSORS) return 0;
  uint16_t white = minValues[index], black = maxValues[index];
  if (black <= white || black - white < MIN_CALIBRATION_DELTA) return 0;
  uint16_t x = sensorRaw[index];
  if (x <= white) return 0;
  if (x >= black) return 1000;
  return (uint32_t(x - white) * 1000UL) / (black - white);
}
bool sensor_is_black(uint8_t i) {
  // Use this sensor's independently calibrated ADC threshold, not a global cutoff.
  return calibrationReady && i < N_SENSORS && sensorRaw[i] >= thresholds[i];
}
uint16_t sensor_get_line_bits(void) {
  uint16_t bits = 0;
  for (int i = 0; i < N_SENSORS; ++i)
    if (sensor_is_black(i)) bits |= uint16_t(1U << i);
  return bits;
}
void sensor_print_states(void) {
  for (int i = 0; i < N_SENSORS; ++i)
    Serial.printf("%d%s", sensor_is_black(i) ? 1 : 0, i == N_SENSORS-1 ? "\n" : " ");
}
