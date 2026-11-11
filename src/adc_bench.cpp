#include "adc_bench.h"
#include "spp_debug.h"
#include "pinout.h"
#include "sensor.h"
#include <Arduino.h>
#include <driver/adc.h>
#include <soc/gpio_struct.h>
#include <math.h>
#include <string.h>

namespace {
constexpr uint8_t kModes = 10; // 9 original modes + IDF_GPIO with no discarded conversion
constexpr uint8_t kNoDiscardMode = 9;
constexpr uint8_t kProductionMode = 8;
constexpr uint16_t kNearThresholdAdc = 100;
constexpr uint8_t kScansPerMode = 64;
constexpr uint8_t kSamples[3] = {4, 2, 1};
constexpr uint32_t kMuxMask = (1UL << PIN_MUX_S0) |
                              (1UL << PIN_MUX_S1) |
                              (1UL << PIN_MUX_S2) |
                              (1UL << PIN_MUX_S3);
static_assert(PIN_LINE_SIG == 36, "ADC benchmark assumes GPIO36 = ADC1_CHANNEL_0");
static_assert(PIN_MUX_S0 < 32 && PIN_MUX_S1 < 32 &&
              PIN_MUX_S2 < 32 && PIN_MUX_S3 < 32,
              "Direct GPIO mode requires output pins below 32");

struct Stat {
  uint64_t scanSum = 0, selectUs = 0, settleUs = 0;
  uint64_t discardUs = 0, readUs = 0, sampleDelayUs = 0;
  uint64_t discardDifference = 0;
  // On the original modes: compare the discarded first conversion with
  // the very next conversion of the SAME channel, not the averaged reading.
  uint64_t firstSecondAbs = 0;
  uint32_t firstSecondMax = 0, firstSecondMismatch = 0;
  uint32_t firstSecondNear = 0, firstSecondNearMismatch = 0;
  uint32_t firstSecondPairs = 0;
  // Mode 9 is a genuinely single-conversion scan; compare it separately
  // with the immediately preceding production (mode 8) scan.
  uint64_t noDiscardVsKeptAbs = 0;
  uint32_t noDiscardVsKeptMax = 0, noDiscardVsKeptMismatch = 0;
  uint32_t noDiscardVsKeptNear = 0, noDiscardVsKeptNearMismatch = 0;
  uint64_t sum[N_SENSORS] = {}, squares[N_SENSORS] = {};
  uint64_t absToRef = 0;
  uint32_t scanMax = 0, absMax = 0;
  uint32_t mismatch = 0, nearThreshold = 0, invalid = 0;
};

Stat stats[kModes];
uint16_t reference[N_SENSORS] = {};
uint16_t productionReference[N_SENSORS] = {};
bool running = false;
bool printing = false;
uint8_t modeIndex = 0, passIndex = 0, reportIndex = 0;

inline void directSelect(uint8_t channel) {
  const uint32_t set = ((channel & 1U) ? (1UL << PIN_MUX_S0) : 0) |
                       ((channel & 2U) ? (1UL << PIN_MUX_S1) : 0) |
                       ((channel & 4U) ? (1UL << PIN_MUX_S2) : 0) |
                       ((channel & 8U) ? (1UL << PIN_MUX_S3) : 0);
  // Clear all, then set selected bits. A 6-us settling delay follows.
  GPIO.out_w1tc = kMuxMask;
  GPIO.out_w1ts = set;
}

inline uint16_t readOnce(uint8_t backend, Stat &s) {
  if (backend == 0) return analogRead(PIN_LINE_SIG);
  const int raw = adc1_get_raw(ADC1_CHANNEL_0);
  if (raw < 0 || raw > 4095) {
    ++s.invalid;
    return 0;
  }
  return (uint16_t)raw;
}

void scan(uint8_t mode, uint16_t *values) {
  Stat &s = stats[mode];
  const uint8_t backend = mode == kNoDiscardMode ? 2 : mode / 3;
  const uint8_t n = mode == kNoDiscardMode ? 1 : kSamples[mode % 3];
  const bool noDiscard = mode == kNoDiscardMode;
  const uint32_t started = micros();
  for (uint8_t ch = 0; ch < N_SENSORS; ++ch) {
    uint32_t t = micros();
    if (backend == 2) directSelect(ch);
    else sensor_mux_select(ch);
    s.selectUs += (uint32_t)(micros() - t);

    t = micros();
    delayMicroseconds(6); // Preserve the existing MUX settling requirement.
    s.settleUs += (uint32_t)(micros() - t);

    uint16_t discard = 0;
    if (!noDiscard) {
      t = micros();
      discard = readOnce(backend, s);
      s.discardUs += (uint32_t)(micros() - t);
    }

    uint32_t total = 0;
    for (uint8_t j = 0; j < n; ++j) {
      t = micros();
      const uint16_t sample = readOnce(backend, s);
      s.readUs += (uint32_t)(micros() - t);
      total += sample;
      if (j == 0 && !noDiscard) {
        const uint32_t d = sample > discard ? sample - discard : discard - sample;
        s.firstSecondAbs += d;
        if (d > s.firstSecondMax) s.firstSecondMax = d;
        ++s.firstSecondPairs;
        if (calibrationReady) {
          const uint16_t threshold = thresholds[ch];
          const bool mismatch = (sample >= threshold) != (discard >= threshold);
          s.firstSecondMismatch += mismatch;
          const uint32_t dd = discard > threshold ? discard - threshold : threshold - discard;
          const uint32_t ds = sample > threshold ? sample - threshold : threshold - sample;
          if (dd < kNearThresholdAdc || ds < kNearThresholdAdc) {
            ++s.firstSecondNear;
            s.firstSecondNearMismatch += mismatch;
          }
        }
      }
      t = micros();
      delayMicroseconds(3); // Preserve the existing post-sample delay.
      s.sampleDelayUs += (uint32_t)(micros() - t);
    }
    const uint16_t value = (uint16_t)(total / n);
    values[ch] = value;
    if (!noDiscard)
      s.discardDifference += discard > value ? discard - value : value - discard;
    s.sum[ch] += value;
    s.squares[ch] += (uint64_t)value * value;
  }
  const uint32_t timeUs = micros() - started;
  s.scanSum += timeUs;
  if (timeUs > s.scanMax) s.scanMax = timeUs;
}

void compare(uint8_t mode, const uint16_t *values) {
  Stat &s = stats[mode];
  for (uint8_t ch = 0; ch < N_SENSORS; ++ch) {
    const uint16_t v = values[ch], ref = reference[ch];
    if (mode == kNoDiscardMode) {
      // Sequential, not simultaneous. Interpret only with stationary sensors.
      const uint16_t kept = productionReference[ch];
      const uint32_t df = v > kept ? v - kept : kept - v;
      s.noDiscardVsKeptAbs += df;
      if (df > s.noDiscardVsKeptMax) s.noDiscardVsKeptMax = df;
      if (calibrationReady) {
        const uint16_t th = thresholds[ch];
        const bool mismatch = (v >= th) != (kept >= th);
        s.noDiscardVsKeptMismatch += mismatch;
        const uint32_t dv = v > th ? v - th : th - v;
        const uint32_t dk = kept > th ? kept - th : th - kept;
        if (dv < kNearThresholdAdc || dk < kNearThresholdAdc) {
          ++s.noDiscardVsKeptNear;
          s.noDiscardVsKeptNearMismatch += mismatch;
        }
      }
    }
    const uint32_t d = v > ref ? v - ref : ref - v;
    s.absToRef += d;
    if (d > s.absMax) s.absMax = d;
    if (calibrationReady) {
      const uint16_t threshold = thresholds[ch];
      s.mismatch += ((v >= threshold) != (ref >= threshold));
      const uint32_t distance = ref > threshold ? ref - threshold : threshold - ref;
      s.nearThreshold += distance < 100;
    }
  }
}

void printRow(uint8_t mode) {
  const Stat &s = stats[mode];
  const uint8_t backend = mode == kNoDiscardMode ? 2 : mode / 3;
  const char *name = mode == kNoDiscardMode ? "IDF_GPIO_NO_DISCARD" :
                     backend == 0 ? "ARDUINO" : backend == 1 ? "IDF" : "IDF_GPIO";
  const uint8_t samples = mode == kNoDiscardMode ? 1 : kSamples[mode % 3];
  const double scans = kScansPerMode;
  const double observations = (double)kScansPerMode * N_SENSORS;
  double noiseSum = 0.0, maxSd = 0.0;
  for (uint8_t ch = 0; ch < N_SENSORS; ++ch) {
    const double mean = (double)s.sum[ch] / scans;
    const double var = (double)s.squares[ch] / scans - mean * mean;
    const double sigma = sqrt(var > 0 ? var : 0);
    noiseSum += sigma;
    if (sigma > maxSd) maxSd = sigma;
  }
  // Total field is measured from scan entry to exit; phase sums do not
  // include timestamp overhead, result accumulation or loop scheduling.
  sppLogPrintf(
    "[ADC_BENCH] method=%s samples=%u scans=%u scan_us=%.0f/%.0f "
    "mux_us=%.1f settle_us=%.1f discard_us=%.1f read_us=%.1f "
    "wait_us=%.1f noise_adc=%.2f/%.2f mae_adc=%.2f maxerr_adc=%lu "
    "mismatch=%lu/%lu near100=%lu discard_delta=%.1f invalid=%lu\n",
    name, (unsigned)samples, (unsigned)kScansPerMode,
    (double)s.scanSum / scans, (double)s.scanMax,
    (double)s.selectUs / scans, (double)s.settleUs / scans,
    (double)s.discardUs / scans, (double)s.readUs / scans,
    (double)s.sampleDelayUs / scans, noiseSum / N_SENSORS, maxSd,
    (double)s.absToRef / observations, (unsigned long)s.absMax,
    (unsigned long)s.mismatch, (unsigned long)(kScansPerMode * N_SENSORS),
    (unsigned long)s.nearThreshold,
    (double)s.discardDifference / observations,
    (unsigned long)s.invalid);
  if (mode == kNoDiscardMode) {
    sppLogPrintf(
      "[ADC_BENCH_NODISCARD] scan_no_discard_us=%.0f/%.0f "
      "vs_kept_mae=%.2f vs_kept_max=%lu vs_kept_mismatch=%lu/%lu "
      "near_threshold_mismatch=%lu/%lu (mode8_ref; stationary_only)\n",
      (double)s.scanSum / scans, (double)s.scanMax,
      (double)s.noDiscardVsKeptAbs / observations,
      (unsigned long)s.noDiscardVsKeptMax,
      (unsigned long)s.noDiscardVsKeptMismatch, (unsigned long)(kScansPerMode * N_SENSORS),
      (unsigned long)s.noDiscardVsKeptNearMismatch, (unsigned long)s.noDiscardVsKeptNear);
  } else {
    sppLogPrintf(
      "[ADC_BENCH_PAIR] method=%s samples=%u first_vs_second_mae=%.2f "
      "first_vs_second_max=%lu first_vs_second_mismatch=%lu/%lu "
      "near_threshold_mismatch=%lu/%lu\n",
      name, (unsigned)samples,
      s.firstSecondPairs ? (double)s.firstSecondAbs / s.firstSecondPairs : 0.0,
      (unsigned long)s.firstSecondMax,
      (unsigned long)s.firstSecondMismatch, (unsigned long)s.firstSecondPairs,
      (unsigned long)s.firstSecondNearMismatch, (unsigned long)s.firstSecondNear);
  }
}
}

bool adcBenchStart() {
  if (running || printing || sensor_is_calibrating()) return false;
  // Match the 12-bit, 11-dB Arduino ESP32 ADC setting.
  // This mode uses only ADC1 channel 0 (GPIO36): no ADC2/Wi-Fi contention.
  if (adc1_config_width(ADC_WIDTH_BIT_12) != ESP_OK ||
      adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11) != ESP_OK)
    return false;
  memset(stats, 0, sizeof(stats));
  memset(reference, 0, sizeof(reference));
  memset(productionReference, 0, sizeof(productionReference));
  modeIndex = 0;
  passIndex = 0;
  reportIndex = 0;
  running = true;
  sppLogLine("[ADC_BENCH] START modes=10 scans_per_mode=64 pins=GPIO36/17/18/19/23");
  sppLogLine("[ADC_BENCH] Fixed scene required: WHITE, BLACK or EDGE; motors OFF");
  sppLogLine("[ADC_BENCH] Mode 0 ARDUINO/4 is the per-pass reference; no NVS or PID changes");
  sppLogLine("[ADC_BENCH] Mode 9 IDF_GPIO_NO_DISCARD is benchmark-only (production unchanged)");
  return true;
}

bool adcBenchActive() { return running || printing; }

void adcBenchCancel() {
  if (!adcBenchActive()) return;
  running = false;
  printing = false;
  sppLogLine("[ADC_BENCH] CANCELLED");
}

void adcBenchStatus() {
  sppLogPrintf("[ADC_BENCH] state=%s pass=%u/%u mode=%u/%u\n",
      running ? "RUNNING" : printing ? "REPORT" : "IDLE",
      (unsigned)passIndex, (unsigned)kScansPerMode,
      (unsigned)modeIndex, (unsigned)kModes);
}

void adcBenchStep() {
  if (running) {
    uint16_t values[N_SENSORS];
    scan(modeIndex, values);
    if (modeIndex == 0) memcpy(reference, values, sizeof(reference));
    if (modeIndex == kProductionMode)
      memcpy(productionReference, values, sizeof(productionReference));
    compare(modeIndex, values);
    if (++modeIndex == kModes) {
      modeIndex = 0;
      ++passIndex;
      if (passIndex == kScansPerMode) {
        running = false;
        printing = true;
        reportIndex = 0;
        sppLogLine("[ADC_BENCH] RESULT reference=ARDUINO/4; stationary scene");
        sppLogLine("[ADC_BENCH] noise=stddev across scans; mismatch=changed black/white"
                       " classification vs paired ARDUINO/4; near100=ref within 100 ADC of threshold");
        sppLogLine("[ADC_BENCH] discard_delta=discarded first read vs averaged kept reads"
                       " (settling proxy, NOT proof of crosstalk)");
        sppLogLine("[ADC_BENCH_PAIR] first vs next conversion of same channel;"
                   " near threshold means either within 100 ADC of TH");
        sppLogLine("[ADC_BENCH_NODISCARD] mode9 vs immediately preceding mode8;"
                   " valid only when sensors are stationary");
      }
    }
  } else if (printing) {
    // Print just one row per loop to remain responsive to ESTOP.
    printRow(reportIndex);
    if (++reportIndex >= kModes) {
      printing = false;
      sppLogLine("[ADC_BENCH] DONE; production sensor mode=IDF_GPIO_1");
    }
  }
}
