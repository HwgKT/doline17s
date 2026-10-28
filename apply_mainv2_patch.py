#!/usr/bin/env python3
'''Apply compact BLE telemetry changes to a local HwgKT/doline17s checkout.

Safety rules:
- Only edits when the checked-out branch is exactly `mainv2`.
- Validates every replacement anchor before writing any file.
- Never performs Git checkout, commit, push, or changes GitHub branches.
'''
from pathlib import Path
import subprocess
import sys

ROOT = Path.cwd()


def require_mainv2():
    try:
        branch = subprocess.check_output(
            ["git", "branch", "--show-current"], cwd=ROOT,
            text=True, stderr=subprocess.STDOUT
        ).strip()
    except Exception as exc:
        raise RuntimeError("Khong doc duoc nhanh Git hien tai: " + str(exc))
    if branch != "mainv2":
        raise RuntimeError(
            f"Dang o nhanh '{branch or 'detached HEAD'}'; tu choi sua. "
            "Hay checkout mainv2 truoc."
        )


def replace_once(text, old, new, label):
    count = text.count(old)
    if count != 1:
        raise RuntimeError(
            f"{label}: can tim dung 1 anchor, nhung tim thay {count}. "
            "Khong co file nao duoc ghi; hay kiem tra dung ban repo main."
        )
    return text.replace(old, new, 1)


def main():
    require_mainv2()
    paths = ["index.html", "src/main.cpp", "src/ble_control.cpp", "include/ble_control.h"]
    original = {}
    for rel in paths:
        path = ROOT / rel
        if not path.is_file():
            raise RuntimeError(f"Thieu file can sua: {rel}")
        original[rel] = path.read_text(encoding="utf-8")
    changed = dict(original)

    # index.html: per-sensor raw/calibrated numbers and motor/encoder cards.
    rel = "index.html"
    s = changed[rel]
    s = replace_once(s,
'''    .sensor-visualizer {
      background: rgba(0, 0, 0, 0.3);
      padding: 20px;
      border-radius: 16px;
      border: 1px solid rgba(255, 255, 255, 0.05);
    }

    .sensor-bar {
      display: grid;
      grid-template-columns: repeat(16, 1fr);
      gap: 6px;
      margin: 16px 0;
    }''',
'''    .sensor-visualizer {
      background: rgba(0, 0, 0, 0.3);
      padding: 20px;
      border-radius: 16px;
      border: 1px solid rgba(255, 255, 255, 0.05);
      overflow-x: auto;
    }

    .sensor-bar {
      display: grid;
      grid-template-columns: repeat(16, minmax(30px, 1fr));
      gap: 6px;
      margin: 16px 0;
      min-width: 680px;
    }''', "CSS sensor visualizer")
    s = replace_once(s,
'''    .sensor-idx {
      font-size: 10px;
      font-family: 'JetBrains Mono', monospace;
      color: var(--text-muted);
    }

    .line-indicator-preview {''',
'''    .sensor-idx {
      font-size: 10px;
      font-family: 'JetBrains Mono', monospace;
      color: var(--text-muted);
    }

    .sensor-adc {
      width: 100%;
      display: flex;
      flex-direction: column;
      gap: 2px;
      font-family: 'JetBrains Mono', monospace;
      font-size: 9px;
      line-height: 1.25;
      color: var(--text-muted);
    }

    .sensor-adc span {
      display: flex;
      justify-content: space-between;
      gap: 2px;
      white-space: nowrap;
    }

    .sensor-adc b {
      color: #e2e8f0;
      font-weight: 600;
    }

    .debug-metrics {
      min-width: 680px;
      display: grid;
      grid-template-columns: repeat(4, minmax(0, 1fr));
      gap: 8px;
      margin: 12px 0 16px;
    }

    .debug-metric {
      min-width: 0;
      padding: 9px 10px;
      border: 1px solid rgba(255, 255, 255, 0.07);
      border-radius: 10px;
      background: rgba(255, 255, 255, 0.035);
      display: flex;
      flex-direction: column;
      gap: 3px;
    }

    .debug-metric span {
      font-size: 10px;
      line-height: 1.3;
      color: var(--text-muted);
    }

    .debug-metric strong {
      overflow-wrap: anywhere;
      font-family: 'JetBrains Mono', monospace;
      font-size: 14px;
      color: var(--accent-cyan);
    }

    .debug-metric small {
      font-size: 9px;
      color: var(--text-muted);
    }

    .line-indicator-preview {''', "CSS ADC/debug fields")
    s = replace_once(s,
'''        <div class="sensor-bar" id="sensorBar">
          <!-- 16 Mắt tự sinh bằng JavaScript -->
        </div>

        <div class="line-indicator-preview">''',
'''        <div class="sensor-bar" id="sensorBar">
          <!-- 16 Mắt tự sinh bằng JavaScript -->
        </div>

        <div class="debug-metrics" aria-label="Thong so debug truc tiep">
          <div class="debug-metric">
            <span>PWM dong co trai</span>
            <strong id="debugPwmLeft">—</strong>
            <small>−255 đến 255</small>
          </div>
          <div class="debug-metric">
            <span>PWM dong co phai</span>
            <strong id="debugPwmRight">—</strong>
            <small>−255 đến 255</small>
          </div>
          <div class="debug-metric">
            <span>Xung encoder trai</span>
            <strong id="debugEncoderLeft">—</strong>
            <small>Dem tich luy</small>
          </div>
          <div class="debug-metric">
            <span>Xung encoder phai</span>
            <strong id="debugEncoderRight">—</strong>
            <small>Dem tich luy</small>
          </div>
        </div>

        <div class="line-indicator-preview">''', "HTML debug cards")
    s = replace_once(s,
'''      node.innerHTML = `
        <div class="sensor-led" id="led_${i}">0</div>
        <span class="sensor-idx">${i}</span>
      `;''',
'''      node.innerHTML = `
        <div class="sensor-led" id="led_${i}">0</div>
        <span class="sensor-idx">${i}</span>
        <div class="sensor-adc">
          <span title="ADC tho">R:<b id="adcRaw_${i}">—</b></span>
          <span title="ADC sau hieu chuan">C:<b id="adcCal_${i}">—</b></span>
        </div>
      `;''', "Sensor card content")
    s = replace_once(s,
'''        txCharacteristic.addEventListener('characteristicvaluechanged', (e) => {
          const value = new TextDecoder().decode(e.target.value);
          handleIncomingData(value);
        });''',
'''        txCharacteristic.addEventListener('characteristicvaluechanged', (e) => {
          handleBleNotification(e.target.value);
        });''', "BLE notification handler")
    s = replace_once(s,
'''    function handleIncomingData(raw) {
      log(`RX <- ${raw}`, 'rx');''',
'''    // Binary debug notification: [0xA7, sequence, fragmentIndex, payload...].
    // The ESP32 sends 4 fragments; update the UI only after a complete snapshot.
    let debugAssembly = null;

    function handleBleNotification(value) {
      if (value && value.byteLength > 0 && value.getUint8(0) === 0xA7) {
        handleDebugFragment(value);
        return;
      }
      handleIncomingData(new TextDecoder().decode(value));
    }

    function handleDebugFragment(frame) {
      if (frame.byteLength < 4) return;
      const sequence = frame.getUint8(1);
      const fragmentIndex = frame.getUint8(2);
      if (fragmentIndex >= 4) return;
      const expectedPayloadLength = fragmentIndex === 3 ? 8 : 16;
      if (frame.byteLength !== expectedPayloadLength + 3) return;

      if (!debugAssembly || debugAssembly.sequence !== sequence) {
        debugAssembly = { sequence, fragments: new Array(4).fill(null) };
      }
      const fragment = new Uint8Array(expectedPayloadLength);
      for (let i = 0; i < expectedPayloadLength; i++) fragment[i] = frame.getUint8(i + 3);
      debugAssembly.fragments[fragmentIndex] = fragment;
      if (debugAssembly.fragments.some(part => part === null)) return;

      const payload = new Uint8Array(56);
      for (let i = 0; i < 4; i++) payload.set(debugAssembly.fragments[i], i * 16);
      debugAssembly = null;
      renderDebugTelemetry(payload);
    }

    // Read LSB-first packed unsigned fields.
    function readPackedUnsigned(bytes, startBit, bitCount) {
      let value = 0;
      for (let bit = 0; bit < bitCount; bit++) {
        const absoluteBit = startBit + bit;
        value |= ((bytes[absoluteBit >>> 3] >>> (absoluteBit & 7)) & 1) << bit;
      }
      return value >>> 0;
    }

    function renderDebugTelemetry(payload) {
      if (payload.length !== 56) return;
      const view = new DataView(payload.buffer, payload.byteOffset, payload.byteLength);
      const leftPwm = view.getInt16(0, true);
      const rightPwm = view.getInt16(2, true);
      const leftEncoder = view.getUint32(4, true);
      const rightEncoder = view.getUint32(8, true);
      const rawValues = [];
      const calibratedValues = [];

      for (let i = 0; i < 16; i++) {
        rawValues.push(readPackedUnsigned(payload, 12 * 8 + i * 12, 12));
        calibratedValues.push(readPackedUnsigned(payload, 36 * 8 + i * 10, 10));
      }

      updateSensorLeds(calibratedValues.map(value => value >= 500 ? '1' : '0').join(''));
      for (let i = 0; i < 16; i++) {
        const rawElement = document.getElementById(`adcRaw_${i}`);
        const calibratedElement = document.getElementById(`adcCal_${i}`);
        if (rawElement) rawElement.textContent = String(rawValues[i]);
        if (calibratedElement) calibratedElement.textContent = String(calibratedValues[i]);
      }

      document.getElementById('debugPwmLeft').textContent = String(leftPwm);
      document.getElementById('debugPwmRight').textContent = String(rightPwm);
      document.getElementById('debugEncoderLeft').textContent = leftEncoder.toLocaleString();
      document.getElementById('debugEncoderRight').textContent = rightEncoder.toLocaleString();
    }

    function handleIncomingData(raw) {
      log(`RX <- ${raw}`, 'rx');''', "BLE binary telemetry decoder")
    s = replace_once(s,
'''    function startAutoPoll() {
      if (!autoPollActive) return;
      stopAutoPoll();
      autoPollInterval = setInterval(() => {
        if (isConnected) {
          sendCommand('P'); // Gửi lệnh P để lấy trạng thái 16 mắt
        }
      }, 200);
    }''',
'''    function startAutoPoll() {
      stopAutoPoll();
      const button = document.getElementById('btnAutoPoll');

      // BLE telemetry is pushed by ESP32; polling with P adds redundant notifications.
      if (connectionType === 'ble') {
        if (button) {
          button.textContent = '⚡ Debug BLE: ESP đẩy';
          button.style.color = 'var(--accent-cyan)';
          button.disabled = true;
        }
        return;
      }

      if (button) {
        button.disabled = false;
        button.textContent = `⚡ Tự động quét: ${autoPollActive ? 'BẬT' : 'TẮT'}`;
        button.style.color = autoPollActive ? 'var(--accent-cyan)' : 'var(--text-muted)';
      }
      if (!autoPollActive || !isConnected || connectionType !== 'serial') return;
      autoPollInterval = setInterval(() => {
        if (isConnected && connectionType === 'serial') sendCommand('P');
      }, 500);
    }''', "Auto-poll function")
    s = replace_once(s,
'''    function toggleAutoPoll() {
      autoPollActive = !autoPollActive;
      const btn = document.getElementById('btnAutoPoll');
      if (autoPollActive) {
        btn.textContent = '⚡ Tự động quét: BẬT';
        btn.style.color = 'var(--accent-cyan)';
        startAutoPoll();
      } else {
        btn.textContent = '⚡ Tự động quét: TẮT';
        btn.style.color = 'var(--text-muted)';
        stopAutoPoll();
      }
    }''',
'''    function toggleAutoPoll() {
      if (connectionType === 'ble') return; // BLE debug data is push-only.
      autoPollActive = !autoPollActive;
      startAutoPoll();
    }''', "Auto-poll toggle")
    s = replace_once(s,
'''        btnSerial.innerHTML = '<span>🔌 Cáp USB Serial</span>';
        connectionType = null;
        stopAutoPoll();''',
'''        btnSerial.innerHTML = '<span>🔌 Cáp USB Serial</span>';
        connectionType = null;
        stopAutoPoll();
        const autoPollButton = document.getElementById('btnAutoPoll');
        if (autoPollButton) {
          autoPollButton.disabled = false;
          autoPollButton.textContent = `⚡ Tự động quét: ${autoPollActive ? 'BẬT' : 'TẮT'}`;
          autoPollButton.style.color = autoPollActive ? 'var(--accent-cyan)' : 'var(--text-muted)';
        }''', "Re-enable auto-poll after disconnect")
    changed[rel] = s

    # include/ble_control.h: public API.
    rel = "include/ble_control.h"
    s = changed[rel]
    s = replace_once(s,
'''void ble_send_pwm_telemetry(int16_t left, int16_t right, float error, const char* caseName);''',
'''void ble_send_pwm_telemetry(int16_t left, int16_t right, float error, const char* caseName);

/**
 * Send compact binary debug snapshot over BLE: PWM L/R, encoder pulse counts,
 * 16 raw ADC readings and 16 calibrated readings. Fragment size stays below
 * the default MTU notification payload limit.
 */
void ble_send_debug_telemetry(int16_t left, int16_t right,
                              uint32_t encoder_left, uint32_t encoder_right);''',
        "BLE header API")
    changed[rel] = s

    # src/ble_control.cpp: compact payload (56 bytes) split into 4 <=19-byte frames.
    rel = "src/ble_control.cpp"
    s = changed[rel]
    s = replace_once(s,
'''#include <BLE2902.h>
#include <string>''',
'''#include <BLE2902.h>
#include <string>
#include <string.h>''', "BLE includes")
    s = replace_once(s,
'''void ble_send_pwm_telemetry(int16_t left, int16_t right, float error, const char* caseName) {
    if (!deviceConnected || pTxCharacteristic == nullptr) return;
    char buffer[80];
    snprintf(buffer, sizeof(buffer), "PWM: L=%d R=%d | Err=%.1f | Case=%s\\n",
             left, right, error, caseName ? caseName : "Unknown");
    pTxCharacteristic->setValue((uint8_t*)buffer, strlen(buffer));
    pTxCharacteristic->notify();
}''',
'''void ble_send_pwm_telemetry(int16_t left, int16_t right, float error, const char* caseName) {
    if (!deviceConnected || pTxCharacteristic == nullptr) return;
    char buffer[80];
    snprintf(buffer, sizeof(buffer), "PWM: L=%d R=%d | Err=%.1f | Case=%s\\n",
             left, right, error, caseName ? caseName : "Unknown");
    pTxCharacteristic->setValue((uint8_t*)buffer, strlen(buffer));
    pTxCharacteristic->notify();
}

namespace {
constexpr uint8_t kDebugMagic = 0xA7;
constexpr uint8_t kDebugPayloadLength = 56;
constexpr uint8_t kDebugFragmentPayloadLength = 16;
constexpr uint8_t kDebugFragmentCount = 4;
uint8_t debugTelemetrySequence = 0;

void writeU16LE(uint8_t* destination, uint16_t value) {
    destination[0] = static_cast<uint8_t>(value & 0xFF);
    destination[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

void writeU32LE(uint8_t* destination, uint32_t value) {
    destination[0] = static_cast<uint8_t>(value & 0xFF);
    destination[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    destination[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
    destination[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

void packUnsignedBits(uint8_t* destination, size_t startBit,
                      uint16_t value, uint8_t bitWidth) {
    for (uint8_t bit = 0; bit < bitWidth; ++bit) {
        if (value & static_cast<uint16_t>(1U << bit)) {
            const size_t outputBit = startBit + bit;
            destination[outputBit >> 3] |=
                static_cast<uint8_t>(1U << (outputBit & 7));
        }
    }
}
}  // namespace

void ble_send_debug_telemetry(int16_t left, int16_t right,
                              uint32_t encoder_left, uint32_t encoder_right) {
    if (!deviceConnected || pTxCharacteristic == nullptr) return;

    // Payload: signed PWM L/R (4 B), encoder counters (8 B),
    // 16 raw ADC values packed at 12 bits (24 B), calibrated at 10 bits (20 B).
    uint8_t payload[kDebugPayloadLength] = {};
    writeU16LE(payload, static_cast<uint16_t>(left));
    writeU16LE(payload + 2, static_cast<uint16_t>(right));
    writeU32LE(payload + 4, encoder_left);
    writeU32LE(payload + 8, encoder_right);

    for (uint8_t i = 0; i < N_SENSORS; ++i) {
        const uint16_t raw = sensorRaw[i] > 4095 ? 4095 : sensorRaw[i];
        const uint16_t calibrated = sensor_get_calibrated(i);
        packUnsignedBits(payload, 12U * 8U + i * 12U, raw, 12);
        packUnsignedBits(payload, 36U * 8U + i * 10U,
                         calibrated > 1000 ? 1000 : calibrated, 10);
    }

    const uint8_t sequence = debugTelemetrySequence++;
    for (uint8_t fragment = 0; fragment < kDebugFragmentCount; ++fragment) {
        const uint8_t offset = fragment * kDebugFragmentPayloadLength;
        const uint8_t remaining = kDebugPayloadLength - offset;
        const uint8_t chunkLength = remaining > kDebugFragmentPayloadLength
            ? kDebugFragmentPayloadLength : remaining;
        uint8_t frame[3 + kDebugFragmentPayloadLength] = {};
        frame[0] = kDebugMagic;
        frame[1] = sequence;
        frame[2] = fragment;
        memcpy(frame + 3, payload + offset, chunkLength);
        pTxCharacteristic->setValue(frame, chunkLength + 3);
        pTxCharacteristic->notify();
        delay(1); // Let the BLE task submit each notification before the next one.
    }
}''', "Add compact debug sender")
    changed[rel] = s

    # src/main.cpp: one rising-edge input per wheel, independent counters.
    rel = "src/main.cpp"
    s = changed[rel]
    s = replace_once(s,
'''unsigned long lastPrintTime = 0;

void setup() {''',
'''namespace {
constexpr uint8_t PIN_ENCODER_LEFT = 32;
constexpr uint8_t PIN_ENCODER_RIGHT = 33;
volatile uint32_t encoderLeftPulses = 0;
volatile uint32_t encoderRightPulses = 0;
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onEncoderLeftPulse() {
  portENTER_CRITICAL_ISR(&encoderMux);
  ++encoderLeftPulses;
  portEXIT_CRITICAL_ISR(&encoderMux);
}

void IRAM_ATTR onEncoderRightPulse() {
  portENTER_CRITICAL_ISR(&encoderMux);
  ++encoderRightPulses;
  portEXIT_CRITICAL_ISR(&encoderMux);
}
}  // namespace

unsigned long lastPrintTime = 0;

void setup() {''', "Encoder counters and ISR")
    s = replace_once(s,
'''  // 2. Khởi tạo điều khiển 2 động cơ PWM
  motor_init();

  // 3. Khởi tạo PID với cấu hình mặc định''',
'''  // 2. Khởi tạo điều khiển 2 động cơ PWM
  motor_init();

  // One pulse input per wheel. Use 3.3 V-compatible encoder outputs.
  pinMode(PIN_ENCODER_LEFT, INPUT_PULLUP);
  pinMode(PIN_ENCODER_RIGHT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_LEFT), onEncoderLeftPulse, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_RIGHT), onEncoderRightPulse, RISING);

  // 3. Khởi tạo PID với cấu hình mặc định''', "Initialize encoder pins")
    s = replace_once(s,
'''    // Xuất giá trị PWM 2 động cơ, sai số và tên Case lên BLE
    if (ble_is_connected()) {
      ble_send_pwm_telemetry(leftPwm, rightPwm, err, caseName);
    }''',
'''    // Send one compact debug snapshot: PWM, pulse counters and both ADC arrays.
    if (ble_is_connected()) {
      uint32_t leftEncoderSnapshot;
      uint32_t rightEncoderSnapshot;
      portENTER_CRITICAL(&encoderMux);
      leftEncoderSnapshot = encoderLeftPulses;
      rightEncoderSnapshot = encoderRightPulses;
      portEXIT_CRITICAL(&encoderMux);

      ble_send_debug_telemetry(
          leftPwm, rightPwm, leftEncoderSnapshot, rightEncoderSnapshot);
    }''', "Replace old verbose telemetry call")
    changed[rel] = s

    # All transformations succeeded before writing any files.
    for rel in paths:
        (ROOT / rel).write_text(changed[rel], encoding="utf-8", newline="\n")
    print("Da ap dung patch tren nhanh mainv2:")
    for rel in paths:
        print("  - " + rel)
    print("Ke tiep: pio run")
    print("Sau khi build thanh cong: git diff --check && git diff --stat")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print("LOI: " + str(exc), file=sys.stderr)
        sys.exit(1)
