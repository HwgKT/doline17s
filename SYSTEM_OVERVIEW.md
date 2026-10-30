# BÁO CÁO TOÀN DIỆN VỀ HỆ THỐNG XE DÒ LINE DOLINE17S (ESP32 LINE ROBOT)
> **Tài liệu tham chiếu chuẩn cho AI Agent & Kỹ sư phát triển**  
> *Mục đích: Cung cấp đầy đủ cấu hình phần cứng, chỉ số kỹ thuật, thuật toán và bản đồ phân chia trách nhiệm của từng file trong codebase. Các agent tiếp theo chỉ cần đọc file này là nắm toàn bộ kiến trúc mà không cần đọc lại từng file mã nguồn.*

---

## MỤC LỤC
1. [Tổng quan Phần cứng & Thông số Kỹ thuật Toàn xe](#1-tổng-quan-phần-cứng--thông-số-kỹ-thuật-toàn-xe)
2. [Sơ đồ Chân cắm Chi tiết (Pinout Mapping)](#2-sơ-đồ-chân-cắm-chi-tiết-pinout-mapping)
3. [Phân tích Chi tiết các Phân hệ & Chỉ số Hoạt động](#3-phân-tích-chi-tiết-các-phân-hệ--chỉ-số-hoạt-động)
   - 3.1. Phân hệ Cảm biến Line 16 mắt & ADC MUX
   - 3.2. Phân hệ Hiệu chuẩn (Calibration) & Lưu trữ NVS
   - 3.3. Phân hệ Động cơ & Driver LEDC PWM
   - 3.4. Phân hệ Encoder Đọc xung x4 Quadrature
   - 3.5. Thuật toán Điều khiển Dò Line PID & State Machine 10 Cases
   - 3.6. Phân hệ Truyền thông Bluetooth Classic SPP & Định dạng Telemetry
   - 3.7. Bộ Đo lường Hiệu năng Thời gian thực (Perf Monitor & ADC Benchmark)
   - 3.8. Ứng dụng Giám sát & Ghi Log trên Máy tính (Python Dashboard & CSV Lifecycle)
4. [Bản đồ Phân loại Trách nhiệm Từng File (File Directory & Responsibility Map)](#4-bản-đồ-phân-loại-trách-nhiệm-từng-file)
   - Nhóm 1: Cấu hình Build & Pinout Phần cứng
   - Nhóm 2: Cảm biến & Quét ADC
   - Nhóm 3: Động cơ & Truyền động
   - Nhóm 4: Đo xung Encoder
   - Nhóm 5: Thuật toán PID & Xử lý Tình huống Đường đua
   - Nhóm 6: Truyền thông Bluetooth Classic & Telemetry
   - Nhóm 7: Giám sát Hiệu năng & Đo đạc Tốc độ Thực thi
   - Nhóm 8: Khởi tạo & Điều phối Vòng lặp Chính (Main Core)
   - Nhóm 9: Giao diện Cũ / BLE UART Dự phòng
   - Nhóm 10: Ứng dụng Điều khiển & Thu thập Dữ liệu Máy tính (Python)
   - Nhóm 11: Unit Test & Kiểm thử Tĩnh Native C++
   - Nhóm 12: Tài liệu Hướng dẫn & Nhật ký Phiên bản
5. [Cẩm nang Hướng dẫn cho AI Agent: Cần sửa file nào khi thực hiện nhiệm vụ?](#5-cẩm-nang-hướng-dẫn-cho-ai-agent-khi-nào-cần-sửa-file-nào)

---

## 1. TỔNG QUAN PHẦN CỨNG & THÔNG SỐ KỸ THUẬT TOÀN XE

| Hạng mục | Thông số kỹ thuật | Ghi chú & Giá trị trong Code |
| :--- | :--- | :--- |
| **Vi điều khiển (MCU)** | ESP32-WROOM-32 / ESP32 DevKit V1 | 2 nhân Xtensa LX6 @ 240 MHz, 320 KB SRAM, 4 MB Flash |
| **Platform / Framework** | PlatformIO (`espressif32@6.12.0`), Arduino framework | `[env:esp32dev]`, build filter loại trừ `ble_control.cpp` |
| **Bản đồ Phân vùng (Partition)**| `huge_app.csv` (~3 MB App, không OTA) | Dành dung lượng cho Bluetooth Classic SPP stack |
| **Tần số Vòng lặp Điều khiển (Loop)** | ~300 Hz – 500 Hz (thực tế đo ~300 Hz – 450 Hz) | Non-blocking loop với `yield()`, không có `delay()` |
| **Tốc độ Quét ADC 16 mắt** | ~1,615 µs / chu kỳ 16 mắt (Trần lý thuyết ~619 Hz) | Chế độ siêu tốc `IDF_GPIO_1`: direct GPIO mask + ADC1 API |
| **Tần số Telemetry Bluetooth** | 300 Hz mặc định (Tùy chỉnh: 0, 20, 50, 100, 200, 300 Hz) | 21.6 kB/s, kích thước mỗi frame nhị phân là 72 bytes |
| **Tần số PWM Động cơ** | 10,000 Hz (10 kHz) | Độ phân giải 8-bit (0 – 255), 4 kênh LEDC độc lập |
| **Bộ đếm Encoder** | 2 kênh Quadrature x4 (A/B), đếm ngắt ISR | Signed 32-bit (`int32_t`), Left Sign: `+1`, Right Sign: `-1` |
| **Cảm biến Line** | 16 mắt quang trở hồng ngoại qua IC MUX CD74HC4067 | Kênh đọc 0 – 15, tín hiệu Analog 12-bit (0 – 4095) |
| **Cơ chế An toàn (Safety)** | Phần mềm E-STOP chốt chặn, Watchdog Manual 750 ms, ngắt khi mất ADC / mất kết nối BT, dừng đích khi đủ 16 mắt đen liên tục 50 ms | Dừng tức thời PWM và Auto PID |

---

## 2. SƠ ĐỒ CHÂN CẮM CHI TIẾT (PINOUT MAPPING)

Tất cả chân cắm được khai báo tập trung tại [include/pinout.h](file:///d:/CAR_CHALLENGE_2026/doline17s/include/pinout.h).

```
                            +-------------------+
                            |    ESP32 DEVKIT   |
      (ADC1_CH0) GPIO 36 <--| PIN_LINE_SIG      |
                 GPIO 17 -->| PIN_MUX_S0        |--> CD74HC4067 S0 (Bit 0)
                 GPIO 18 -->| PIN_MUX_S1        |--> CD74HC4067 S1 (Bit 1)
                 GPIO 19 -->| PIN_MUX_S2        |--> CD74HC4067 S2 (Bit 2)
                 GPIO 23 -->| PIN_MUX_S3        |--> CD74HC4067 S3 (Bit 3)
                            |                   |
                 GPIO 34 <--| PIN_CALIB_BUTTON  |<-- Nút Calib (Active LOW, pull-up ngoài 10 kΩ)
                 GPIO 35 <--| PIN_START_BUTTON  |<-- Nút Start (Active LOW, pull-up ngoài 10 kΩ)
                            |                   |
                 GPIO 21 <--| PIN_ENCODER_L1    |<-- Encoder Trái Kênh A
                 GPIO 22 <--| PIN_ENCODER_L2    |<-- Encoder Trái Kênh B
                 GPIO 32 <--| PIN_ENCODER_R1    |<-- Encoder Phải Kênh A
                 GPIO 33 <--| PIN_ENCODER_R2    |<-- Encoder Phải Kênh B
                            |                   |
                 GPIO 25 -->| PIN_MOTOR_L1      |--> Driver Động cơ Trái (Tiến)
                 GPIO 26 -->| PIN_MOTOR_L2      |--> Driver Động cơ Trái (Lùi)
                 GPIO 27 -->| PIN_MOTOR_R1      |--> Driver Động cơ Phải (Tiến)
                 GPIO 13 -->| PIN_MOTOR_R2      |--> Driver Động cơ Phải (Lùi)
                            |                   |
                 GPIO 2  -->| PIN_DEBUG_LED     |--> LED Onboard (Nháy: ngắt BT / Sáng: nối BT)
                 GPIO 16 -->| PIN_ACTION_LED    |--> LED RX2 (HIGH sáng; tắt mặc định; ACK 3 lần)
                            +-------------------+
```

---

## 3. PHÂN TÍCH CHI TIẾT CÁC PHÂN HỆ & CHỈ SỐ HOẠT ĐỘNG

### 3.1. Phân hệ Cảm biến Line 16 mắt & ADC MUX ([sensor.cpp](file:///d:/CAR_CHALLENGE_2026/doline17s/src/sensor.cpp) & [sensor.h](file:///d:/CAR_CHALLENGE_2026/doline17s/include/sensor.h))
- **Số lượng cảm biến:** 16 mắt (Index 0 đến 15).
- **Mạch giải mã kênh (Multiplexer):** CD74HC4067, điều khiển 4 chân nhị phân $S_0, S_1, S_2, S_3$.
- **Cơ chế đọc nhanh `IDF_GPIO_1`:**
  - Ghi thanh ghi phần cứng trực tiếp `GPIO.out_w1tc` và `GPIO.out_w1ts` qua mặt nạ `kMuxMask` (nhanh gấp nhiều lần so với `digitalWrite`).
  - Delay chờ điện áp ổn định sau khi chuyển kênh: `6 µs`.
  - Đọc xả mẫu đầu tiên (discard reading) qua `adc1_get_raw(ADC1_CHANNEL_0)` để chống hiện tượng tích tụ điện dung ký sinh trên đường dẫn MUX.
  - Đọc mẫu chính thức lấy kết quả (1 sample).
  - Delay sau đọc: `3 µs`.
  - Tổng thời gian đọc 16 kênh: ~1,615 µs (giảm 7.57 lần so với Arduino analogRead chuẩn 12,232 µs).
- **Phát hiện lỗi phần cứng:** Nếu mẫu ADC đọc về $<0$ hoặc $>4095$, cờ chốt `fastAdcReady = false`, tăng biến lỗi `fastAdcErrors`, ngắt động cơ ngay lập tức để bảo vệ robot khỏi lao tự do.

### 3.2. Phân hệ Hiệu chuẩn (Calibration) & Lưu trữ NVS
- **Cơ chế lấy mẫu 2 bước độc lập (2-surface static calibration):**
  - **Bước 1 (Lấy Trắng - White):** Đặt cả 16 mắt trên bề mặt trắng, bấm nút hoặc gửi lệnh `CAL_WHITE`. Hệ thống lấy mẫu tích lũy $128$ khung hình (`CAL_FRAMES = 128`), tính giá trị trung bình (`mean`) và độ lệch chuẩn (`stddev`).
  - **Bước 2 (Lấy Đen - Black):** Chuyển xe sang bề mặt đen, bấm nút hoặc gửi lệnh `CAL_BLACK`. Lấy tiếp $128$ khung hình.
- **Tiêu chuẩn nghiệm thu hiệu chuẩn:**
  - Độ tương phản tối thiểu: $\Delta = (Black_i - White_i) \ge 150$ (`MIN_CALIBRATION_DELTA = 150`).
  - Giới hạn nhiễu tối đa: Độ lệch chuẩn $\sigma_{White} \le 90$ và $\sigma_{Black} \le 90$ (`CAL_MAX_STDDEV = 90`).
- **Ngưỡng đen/trắng riêng từng mắt (Per-eye Raw Threshold):**
  $$TH_i = W_i + \left\lceil \frac{(B_i - W_i) \times 400}{1000} \right\rceil$$
  *(Vạch đen được xác nhận khi tín hiệu vượt 40% biên độ tương phản từ trắng lên đen).*
- **Phán đoán Logic vạch đen (`sensor_is_black(i)`):**
  - Trả về `true` khi và chỉ khi `sensorRaw[i] >= thresholds[i]`.
- **Chuẩn hóa Analog (`sensor_get_calibrated(i)`):**
  - Tín hiệu được co giãn tuyến tính về thang $0 - 1000$ ($0$ là hoàn toàn trắng, $1000$ là hoàn toàn đen).
- **Lưu trữ NVS (Non-Volatile Storage):**
  - Cấu trúc `Profile` 76 bytes, lưu trong NVS namespace `"line16cal"` với Magic number `0x4C313643`, Version `1` và kiểm tra toàn vẹn bằng mã CRC32. Khởi động lại MCU tự nạp cấu hình nếu CRC hợp lệ.

### 3.2.1. Nút Calib, nút Start và LED phản hồi
- **Phần cứng nút:** GPIO34 (CALIB) và GPIO35 (START) active LOW. Mỗi chân cần điện trở kéo lên ngoài 10 kΩ tới 3.3 V; nút nối chân xuống GND. GPIO34/35 không có điện trở kéo nội.
- **Debounce:** Mỗi nút được lấy mẫu độc lập, debounce 30 ms; chỉ phát sự kiện ở cạnh nhấn HIGH → LOW. Giữ nút không lặp sự kiện; cần thả rồi nhấn lại.
- **CALIB:** bắt đầu ở WAIT_WHITE. Nhấn lần một gọi calibration WHITE hiện có, tích lũy 128 khung không chặn và kiểm tra noise từng mắt; chỉ khi có sự kiện `CAL_WHITE_DONE` mới xác nhận LED và chuyển logic sang WAIT_BLACK. Nhấn lần hai gọi calibration BLACK, kiểm tra contrast/noise và lưu NVS; chỉ `CAL_SAVED` mới xác nhận thành công, rồi quay về WAIT_WHITE. Nếu WHITE lỗi thì thử lại WHITE; nếu BLACK lỗi thì giữ mẫu WHITE trong RAM để thử lại BLACK. Profile NVS cũ vẫn được giữ.
- **START:** dùng cùng hàm chuyển AUTO với lệnh SPP `AUTO`/`START`. Từ chối khi đang AUTO, E-STOP, ADC bench/lỗi ADC, đang calib, đang chờ BLACK hoặc calibration chưa hợp lệ. Không reset encoder và dùng bộ PID hiện hành.
- **LED:** GPIO16 (RX2), active HIGH, mặc định LOW/tắt. Sau mỗi hành động nút thành công, chạy đúng ba chu kỳ HIGH 200 ms / LOW 200 ms. State machine `millis()` không dùng `delay()`; yêu cầu mới khi đang nháy khởi động lại một mẫu duy nhất, không xếp hàng.
- **Luồng:** `GPIO → debounce → sự kiện nút → handler calibration/AUTO hiện có → kết quả thành công → LED ACK`. Các log chẩn đoán đi qua `sppLogLine` và không thay đổi frame telemetry 72 byte. Logic debounce/LED thuần C++ nằm trong `include/input_logic.h`, có smoke test host tại `tests/native/input_logic_smoke.cpp`.

### 3.2.2. Chạy độc lập Bluetooth và lưu cấu hình PID
- Bluetooth Classic SPP là tùy chọn. `loop()` luôn quét ADC và chạy PID khi AUTO được bật vật lý, bất kể `btBegin()` thành công hoặc đang có client SPP hay không.
- Khi SPP mất kết nối, firmware chỉ dừng chuyển động manual đang điều khiển qua Bluetooth (`manualActive`). AUTO vật lý không bị dừng; E-STOP, ADC fault và các điều kiện an toàn khác vẫn có hiệu lực.
- Calibration cảm biến nằm độc lập trong NVS namespace `line16cal`. Cấu hình PID nằm trong namespace `line16pid`, key `config`, có magic/version/size/CRC và kiểm tra Kp/Ki/Kd trong 0–50, Base PWM trong 0–255. Boot khôi phục profile hợp lệ; nếu không có profile hợp lệ thì dùng mặc định hiện hành `Kp=2.0, Ki=0.0, Kd=0.0, Base=100`.
- Thay đổi PID qua `KP:`, `KI:`, `KD:` hoặc `BASE:` chỉ cập nhật RAM. Gửi `PID_SAVE` khi motor đã dừng để ghi NVS một lần có chủ đích; không ghi NVS trong vòng PID. `maxSpeed=255` là giới hạn firmware không có API tuning nên không nằm trong profile người dùng.

### 3.3. Phân hệ Động cơ & Driver LEDC PWM ([motor.cpp](file:///d:/CAR_CHALLENGE_2026/doline17s/src/motor.cpp) & [motor.h](file:///d:/CAR_CHALLENGE_2026/doline17s/include/motor.h))
- **Cấu hình phần cứng:**
  - Tần số PWM: `10,000 Hz` (10 kHz) - Giúp động cơ chạy êm, lực kéo tốt, không có tiếng rít tần số thấp.
  - Độ phân giải: `8-bit` (Dải giá trị tốc độ từ $-255$ đến $+255$).
  - Phân bổ kênh ESP32 LEDC:
    - Kênh 0 (`CH_L1` - GPIO 25): Bánh trái tiến.
    - Kênh 1 (`CH_L2` - GPIO 26): Bánh trái lùi.
    - Kênh 2 (`CH_R1` - GPIO 27): Bánh phải tiến.
    - Kênh 3 (`CH_R2` - GPIO 13): Bánh phải lùi.
- **Cơ chế điều khiển:**
  - `motor_set_left(speed)` / `motor_set_right(speed)`: Nhận dải $-255 \dots +255$. Số dương phát xung chân tiến (chân lùi = 0), số âm phát xung chân lùi (chân tiến = 0), số 0 ngắt cả hai chân (phanh tự do).
  - Cho phép điều khiển tốc độ có dấu (Signed Wheel Commands) cho phép đảo chiều bánh phía trong để quay tâm gắt khi vào cua.

### 3.4. Phân hệ Encoder Đọc xung x4 Quadrature ([encoder.cpp](file:///d:/CAR_CHALLENGE_2026/doline17s/src/encoder.cpp) & [encoder.h](file:///d:/CAR_CHALLENGE_2026/doline17s/include/encoder.h))
- **Phương pháp giải mã:** Giải mã 4 trạng thái (x4 Quadrature Decoding) trên cả 2 cạnh lên và xuống của cả 2 kênh A & B thông qua ngắt phần cứng `attachInterrupt(..., CHANGE)`.
- **Chống nhiễu:** Thuật toán bỏ qua các bước nhảy 2-bit đồng thời (coi là nhiễu xung).
- **Quy ước chiều quay:**
  - Trái (`PIN_ENCODER_L1`: 21, `PIN_ENCODER_L2`: 22): Hệ số `ENCODER_LEFT_SIGN = 1`.
  - Phải (`PIN_ENCODER_R1`: 32, `PIN_ENCODER_R2`: 33): Hệ số `ENCODER_RIGHT_SIGN = -1`.
  - Cả hai số đếm đều tăng dương khi xe chuyển động thẳng tiến về phía trước.
- **An toàn luồng (Thread-safety):** Dùng `portENTER_CRITICAL_ISR` / `portENTER_CRITICAL` bảo vệ biến đếm `leftCount` và `rightCount` khỏi xung đột giữa ngắt và vòng lặp chính.

### 3.5. Thuật toán Điều khiển Dò Line PID & State Machine 10 Cases ([pid_controller.cpp](file:///d:/CAR_CHALLENGE_2026/doline17s/src/pid_controller.cpp) & [pid_controller.h](file:///d:/CAR_CHALLENGE_2026/doline17s/include/pid_controller.h))

#### A. Trọng số Cảm biến & Tính toán Vị trí Trọng tâm (Centroid Calculation)
- **Mảng trọng số 16 mắt (`SENSOR_WEIGHTS`):**
  $$\{-75.0, -65.0, -55.0, -45.0, -35.0, -25.0, -15.0, -5.0, +5.0, +15.0, +25.0, +35.0, +45.0, +55.0, +65.0, +75.0\}$$
- **Nguyên lý tính toán:**
  - Chỉ các mắt nào thỏa mãn điều kiện logic $Raw_i \ge TH_i$ mới được đưa vào tính trọng tâm.
  - Công thức: $Position = \frac{\sum (Calibrated_i \times Weight_i)}{\sum Calibrated_i}$.
  - Sai số vị trí ($Error$) dao động từ $-75.0$ (lệch trái tối đa) đến $+75.0$ (lệch phải tối đa), $0.0$ là line nằm chính giữa xe.

#### B. Thuật toán PID Phi tuyến Nâng cao
1. **Khâu Tỉ lệ Phi tuyến (Non-linear Proportional Term):**
   $$P = K_p \times (1.0 + 0.018 \times |Error|) \times Error$$
   *(Gần tâm xe phản ứng êm dịu, nhưng khi lệch ra xa thì hệ số khuếch đại tự động tăng vọt theo hàm bậc hai).*
2. **Khâu Tích phân Chống bão hòa (Anti-windup & Fast-decay Integral):**
   - Giới hạn tích lũy chặt: Clamped trong khoảng $[-80.0, +80.0]$.
   - Xả nhanh khi đổi dấu: Nếu sai số chuyển dấu qua tâm ($Error \times Integral < 0$), giá trị $Integral$ bị giảm $50\%$ ngay lập tức để chống hiện tượng vọt lố (overshoot).
3. **Khâu Vi phân Lọc thông thấp (Filtered Derivative Term):**
   $$rawD = K_d \times \frac{Error - LastError}{\Delta t}$$
   $$D = 0.6 \times rawD + 0.4 \times LastDTerm$$
   *(Bộ lọc IIR làm mượt các gai nhiễu tín hiệu tần số cao).*

#### C. Bộ Điều tốc Động (Dynamic Speed Governor) & Kiểm soát Góc bẻ lái
- **Giảm tốc theo sai số tâm:**
  $$v_{target} = BaseSpeed \times \left(1.0 - 0.45 \times \min\left(\frac{|Error|}{30.0}, 1.0\right)\right)$$
  *(Khi sai số đạt 30.0, tốc độ cơ sở giảm 45%; tốc độ đáy tối thiểu không bao giờ dưới 45 PWM).*
- **Phát hiện sớm mép ngoài (Outer Sensors Severity):**
  - Nếu 4 mắt ngoài cùng (0, 1, 2, 3 hoặc 12, 13, 14, 15) thấy line, tốc độ tiếp tục bị ghì giảm thêm tối đa $40\%$.
- **Mở rộng hạn mức công suất bẻ lái (Steering Peak Authority):**
  - Khi xe đi thẳng, giới hạn công suất bánh xe bị hãm ở tốc độ $BaseSpeed$. Khi sai số tăng từ 20 đến 60, hạn mức trần mở rộng dần lên tới $MaxSpeed$ (255) cho phép bánh ngoài bốc ga mạnh để kéo xe về lại tâm line.

#### D. Bảng Trạng thái 10 Cases Dò Line (`LineCase_t`)
| Case ID | Tên Case Enum | Điều kiện Kích hoạt | Hành vi Điều khiển Động cơ |
| :---: | :--- | :--- | :--- |
| **Case 1** | `LINE_CASE_NORMAL_PID` | Thấy line ổn định, sai số trong tầm kiểm soát | Chạy thuật toán PID phi tuyến + Điều tốc động |
| **Case 2** | `LINE_CASE_SHARP_LEFT` | Vị trí $\le -40.0$ hoặc $\le -28.0$ kèm $\ge 2$ mắt mép trái | Giảm tốc bánh trong, tăng bánh ngoài theo profile cua gắt; nếu mất line thì quay tâm trái |
| **Case 3** | `LINE_CASE_SHARP_RIGHT` | Vị trí $\ge +40.0$ hoặc $\ge +28.0$ kèm $\ge 2$ mắt mép phải | Giảm tốc bánh trong, tăng bánh ngoài theo profile cua gắt; nếu mất line thì quay tâm phải |
| **Case 4** | `LINE_CASE_LOST_LINE` | Không thấy vạch đen và hết thời gian cho phép tìm kiếm | Ngắt toàn bộ động cơ lập tức (`motor_stop()`) |
| **Case 5** | `LINE_CASE_CROSS_LINE` | $\ge 11$ mắt cảm biến đều nhận diện vạch đen | Vạch ngang/ngã tư: Đi thẳng với $70\%$ BaseSpeed để vượt qua vạch |
| **Case 6** | `LINE_CASE_T_LEFT` | Phân loại rẽ ngã 3 trái | Cơ chế rẽ trái |
| **Case 7** | `LINE_CASE_T_RIGHT` | Phân loại rẽ ngã 3 phải | Cơ chế rẽ phải |
| **Case 8** | `LINE_CASE_STOPPED` | Đang tắt AUTO, hoặc chốt đích khi cả 16 mắt đen liên tục $\ge 50$ ms | Dừng cả hai động cơ (`PWM = 0`); vạch ngang có 11–15 mắt đen chỉ chạy qua |
| **Case 9** | `LINE_CASE_DASHED_LINE` | Mất vạch đen $< 40$ ms | Duy trì quán tính góc lái cũ (coast với $70\%$ lực bẻ lái trước đó) |
| **Case 10** | `LINE_CASE_STARTUP_COAST`| Mất line trong 5 giây đầu sau khi bật AUTO | Chạy thẳng với PWM tối đa $65$ để tìm lại line |

- **Cơ chế Pivot Search khi mất line sau cua:**
  - Nếu mất vạch quá $40$ ms sau khi vừa vào cua: Xe quay tại chỗ (Pivot) với $PWM = \pm 85$ (tạo chênh lệch $170$ PWM giữa 2 bánh) về hướng cuối cùng nhìn thấy line.
  - Bắt lại line: Yêu cầu $3$ khung hình liên tiếp tin cậy (bắt bằng mắt giữa hoặc cặp mắt liền kề). Khâu vi phân $D$ được giữ bằng $0$ trong $4$ chu kỳ đầu và tốc độ dốc dần từ PWM 65 lên để chống giật xe.
  - Timeout an toàn: Nếu quay tìm line quá $2.0$ giây (`LINE_SEARCH_TIMEOUT_MS = 2000`) mà không thấy line $\rightarrow$ Chuyển về `LINE_CASE_LOST_LINE` và dừng xe hoàn toàn.
- **Chốt đích toàn đen:** Chỉ mask cảm biến `0xFFFF` mới bắt đầu xác nhận đích. Bất kỳ mắt nào đọc trắng đều xóa bộ đếm thời gian; đủ 50 ms liên tục thì chốt `finishLineLatched`, dừng động cơ và kết thúc AUTO. Vạch giao cắt rộng với 11–15 mắt đen vẫn được xử lý như `LINE_CASE_CROSS_LINE`.

### 3.6. Phân hệ Truyền thông Bluetooth Classic SPP & Định dạng Telemetry ([bt_telemetry.cpp](file:///d:/CAR_CHALLENGE_2026/doline17s/src/bt_telemetry.cpp) & [bt_telemetry.h](file:///d:/CAR_CHALLENGE_2026/doline17s/include/bt_telemetry.h))
- **Giao thức:** Bluetooth Serial Port Profile (SPP) tiêu chuẩn, tên thiết bị phát sóng `"LineRobot"`.
- **Cấu trúc Khung truyền nhị phân (Binary Frame) - Tổng 72 bytes:**
  - Header: `0xAA 0x55` (2 bytes).
  - Payload struct `Telemetry`: 69 bytes (định dạng Python struct `<I16H2i2h5fB`).
  - Checksum: `XOR` của toàn bộ 69 bytes payload (1 byte).
- **Chi tiết trường dữ liệu trong Payload 69 bytes:**
  ```c
  struct Telemetry {
    uint32_t t_ms;      // 4B: Timestamp từ millis()
    uint16_t adc[16];   // 32B: 16 giá trị ADC thô (0 - 4095)
    int32_t  encL;      // 4B: Số xung encoder tích lũy bánh trái
    int32_t  encR;      // 4B: Số xung encoder tích lũy bánh phải
    int16_t  pwmL;      // 2B: PWM điều khiển bánh trái (-255 đến 255)
    int16_t  pwmR;      // 2B: PWM điều khiển bánh phải (-255 đến 255)
    float    err;       // 4B: Sai số lệch tâm vị trí line (-75.0 đến 75.0)
    float    p;         // 4B: Thành phần điều khiển tỉ lệ P
    float    i;         // 4B: Thành phần điều khiển tích phân I
    float    d;         // 4B: Thành phần điều khiển vi phân D
    float    out;       // 4B: Tổng lực điều khiển PID (p + i + d)
    uint8_t  mode;      // 1B: 0 = Manual / Lái tay, 1 = Auto PID
  };
  ```
- **Kiến trúc luồng truyền FreeRTOS (Core 0 Background Worker):**
  - Dữ liệu từ vòng lặp chính (`Core 1`) được đẩy vào hàng đợi non-blocking `txQueue` (dung lượng $12$ frames).
  - Tác vụ `txWorker` chạy trên `Core 0` lấy dữ liệu ra và truyền qua SPP. Khi mạng Bluetooth bị trễ hoặc tắc nghẽn, khung truyền bị thả bỏ (`dropped++`) chứ tuyệt đối không làm chậm vòng lặp điều khiển PID của xe.
  - Sử dụng Mutex `txLock` để đồng bộ các chuỗi phản hồi dạng text (ASCII replies) với dữ liệu nhị phân, đảm bảo không bị đan xen làm hỏng cấu trúc khung 72 bytes.

### 3.7. Bộ Đo lường Hiệu năng Thời gian thực ([perf_monitor.cpp](file:///d:/CAR_CHALLENGE_2026/doline17s/src/perf_monitor.cpp) & [adc_bench.cpp](file:///d:/CAR_CHALLENGE_2026/doline17s/src/adc_bench.cpp))
- **Các giai đoạn đo lường thời gian (`PerfStage`):**
  - `PERF_COMM`: Thời gian kiểm tra và xử lý lệnh kết nối Bluetooth.
  - `PERF_ADC`: Thời gian quét 16 mắt cảm biến và lọc số liệu.
  - `PERF_PID`: Thời gian tính toán thuật toán PID và ra quyết định bánh xe.
  - `PERF_TX`: Thời gian đóng gói và nạp khung telemetry vào hàng đợi.
- **Lệnh `PERF`:** Báo cáo thời gian trung bình/cực đại ($\mu s$) của từng giai đoạn, chu kỳ vòng lặp, độ giật tín hiệu thời gian ($jitter$), trần tần số lý thuyết.
- **Lệnh `ADC_BENCH`:** Chương trình đo kiểm 9 chế độ đọc ADC khác nhau, so sánh độ nhiễu, độ chính xác và tốc độ giữa Arduino API và ESP-IDF Direct Registers.

### 3.8. Ứng dụng Giám sát & Ghi Log trên Máy tính ([robot_debug.py](file:///d:/CAR_CHALLENGE_2026/doline17s/robot_debug.py) & [spp_csv_recorder.py](file:///d:/CAR_CHALLENGE_2026/doline17s/spp_csv_recorder.py))
- **Giao thức kết nối:** Python kết nối tới cổng COM ảo do Windows ánh xạ với SPP của ESP32.
- **Tách luồng xử lý cao tốc:**
  - Luồng 1 (`Reader` thread): Nhận dữ liệu nhị phân từ Serial, kiểm tra mã kiểm tra XOR, bóc tách gói tin.
  - Luồng 2 (`TelemetryCsvRecorder` thread): Hàng đợi 8192 frames, ghi liên tục xuống đĩa file `logs/linerobot_YYYYMMDD_HHMMSS_ffffff.csv` với đầy đủ 61 cột dữ liệu (Raw ADC, Calibrated ADC, Logic bits, Encoders, PWM, PID terms).
  - Hàng đầu tiên sau tiêu đề chứa dòng metadata `CAL_THRESHOLDS` ghi nhận 16 ngưỡng thô $TH_0 \dots TH_{15}$.
  - Luồng 3 (`Tkinter GUI`): Cập nhật màn hình ở tần số mượt mà ~33 Hz, không gây nghẽn luồng đọc dữ liệu 300 Hz.
- **Vòng đời CSV (`run_csv_lifecycle.py`):** Mỗi lần bấm **BẮT ĐẦU DÒ LINE (AUTO)** ứng dụng tự động mở 1 file CSV mới; khi bấm **DỪNG (STOP)** file sẽ tự động đóng và đồng bộ xuống đĩa hoàn tất.
- **Đồng hồ phiên chạy:** Dashboard bắt đầu tính thời gian khi telemetry AUTO cho thấy ít nhất một bánh có PWM khác 0. Nếu cả hai PWM bằng 0 liên tục 250 ms, đồng hồ chốt tại thời điểm bắt đầu dừng (không tính thời gian chờ xác nhận); STOP/AUTO tắt cũng chốt đồng hồ. Việc chốt đồng hồ không tự đóng CSV.

---

## 4. BẢN ĐỒ PHÂN LOẠI TRÁCH NHIỆM TỪNG FILE

Bảng dưới đây chia toàn bộ các file trong workspace thành các nhóm chức năng rõ ràng:

### Nhóm 1: Cấu hình Build & Pinout Phần cứng
| File | Đường dẫn | Trách nhiệm chính & Nội dung cốt lõi |
| :--- | :--- | :--- |
| **`platformio.ini`** | [platformio.ini](file:///d:/CAR_CHALLENGE_2026/doline17s/platformio.ini) | Cấu hình PlatformIO cho môi trường `esp32dev`, baud rate upload (921600), monitor (115200), phân vùng `huge_app.csv`, filter loại bỏ `ble_control.cpp`. |
| **`include/pinout.h`** | [pinout.h](file:///d:/CAR_CHALLENGE_2026/doline17s/include/pinout.h) | Nơi duy nhất định nghĩa toàn bộ chân GPIO phần cứng: MUX ($S_0-S_3$, SIG), nút bấm, 4 chân Encoder, 4 chân LEDC Motor và LED chỉ thị. |

### Nhóm 2: Cảm biến & Quét ADC
| File | Đường dẫn | Trách nhiệm chính & Nội dung cốt lõi |
| :--- | :--- | :--- |
| **`include/sensor.h`** | [sensor.h](file:///d:/CAR_CHALLENGE_2026/doline17s/include/sensor.h) | Khai báo hằng số cảm biến ($N=16$, $Threshold=400$, mẫu Calib=128), biến toàn cục (`sensorRaw`, `thresholds`, `minValues`, `maxValues`), và prototype hàm đọc cảm biến/calib. |
| **`src/sensor.cpp`** | [sensor.cpp](file:///d:/CAR_CHALLENGE_2026/doline17s/src/sensor.cpp) | Triển khai quét nhanh ADC bằng direct GPIO mask + ADC1, giải thuật lọc mẫu, quy trình lấy mẫu tĩnh 2 bước Trắng/Đen, tính ngưỡng $TH_i$, chuẩn hóa $0-1000$ và lưu trữ NVS Flash có CRC32. |

### Nhóm 3: Động cơ & Truyền động
| File | Đường dẫn | Trách nhiệm chính & Nội dung cốt lõi |
| :--- | :--- | :--- |
| **`include/motor.h`** | [motor.h](file:///d:/CAR_CHALLENGE_2026/doline17s/include/motor.h) | Khai báo cấu hình PWM LEDC: Tần số `10000 Hz`, độ phân giải 8-bit, 4 kênh LEDC 0..3, và prototype các hàm điều khiển tốc độ từng bánh (`-255` đến `+255`). |
| **`src/motor.cpp`** | [motor.cpp](file:///d:/CAR_CHALLENGE_2026/doline17s/src/motor.cpp) | Triển khai khởi tạo kênh LEDC, gán chân GPIO, điều khiển hướng quay tiến/lùi độc lập cho từng động cơ, phanh dừng khẩn cấp. |

### Nhóm 4: Đo xung Encoder
| File | Đường dẫn | Trách nhiệm chính & Nội dung cốt lõi |
| :--- | :--- | :--- |
| **`include/encoder.h`** | [encoder.h](file:///d:/CAR_CHALLENGE_2026/doline17s/include/encoder.h) | Khai báo hàm khởi tạo encoder, đọc số xung tích lũy atomic (`encoder_get_counts`) và reset số xung (`encoder_reset_counts`). |
| **`src/encoder.cpp`** | [encoder.cpp](file:///d:/CAR_CHALLENGE_2026/doline17s/src/encoder.cpp) | Triển khai ngắt ISR giải mã Quadrature x4 hai kênh độc lập, lọc nhiễu nhảy 2-bit, bảo vệ critical section bằng mutex spinlock. |

### Nhóm 5: Thuật toán PID & Xử lý Tình huống Đường đua
| File | Đường dẫn | Trách nhiệm chính & Nội dung cốt lõi |
| :--- | :--- | :--- |
| **`include/pid_controller.h`** | [pid_controller.h](file:///d:/CAR_CHALLENGE_2026/doline17s/include/pid_controller.h) | Khai báo enum 10 Cases đường line (`LINE_CASE_NORMAL_PID`, `SHARP_LEFT`, `LOST_LINE`...), cấu trúc tham số PID (`PID_Config_t`), và các hàm tính toán vị trí/chạy step PID. |
| **`src/pid_controller.cpp`** | [pid_controller.cpp](file:///d:/CAR_CHALLENGE_2026/doline17s/src/pid_controller.cpp) | Trọng tâm thuật toán điều khiển của xe: Mảng trọng số 16 mắt, tính trọng tâm analog, khâu P phi tuyến, khâu I anti-windup, khâu D IIR filter, bộ điều tốc Dynamic Speed Governor, cơ chế Pivot search, và chốt đích khi đủ 16 mắt đen liên tục 50 ms. |

### Nhóm 6: Truyền thông Bluetooth Classic & Telemetry
| File | Đường dẫn | Trách nhiệm chính & Nội dung cốt lõi |
| :--- | :--- | :--- |
| **`include/bt_telemetry.h`** | [bt_telemetry.h](file:///d:/CAR_CHALLENGE_2026/doline17s/include/bt_telemetry.h) | Định nghĩa cấu trúc `Telemetry` chuẩn đóng gói nhị phân 69 bytes (`#pragma pack(push, 1)`), cấu trúc thống kê truyền dẫn `BtTxStats`. |
| **`src/bt_telemetry.cpp`** | [bt_telemetry.cpp](file:///d:/CAR_CHALLENGE_2026/doline17s/src/bt_telemetry.cpp) | Triển khai BluetoothSerial SPP, FreeRTOS background worker truyền telemetry trên Core 0, hàng đợi 12 frame, mutex bảo vệ truyền chuỗi ASCII, cơ chế giải mã lệnh nhận từ PC. |
| **`include/spp_debug.h`** | [spp_debug.h](file:///d:/CAR_CHALLENGE_2026/doline17s/include/spp_debug.h) | Cung cấp các hàm tiện ích log chẩn đoán qua Bluetooth SPP (`sppLogLine`, `sppLogPrintf`), tự động kiểm tra trạng thái kết nối trước khi gửi. |

### Nhóm 7: Giám sát Hiệu năng & Đo đạc Tốc độ Thực thi
| File | Đường dẫn | Trách nhiệm chính & Nội dung cốt lõi |
| :--- | :--- | :--- |
| **`include/perf_monitor.h`** | [perf_monitor.h](file:///d:/CAR_CHALLENGE_2026/doline17s/include/perf_monitor.h) | Khai báo các enum giai đoạn vòng lặp (`PERF_COMM`, `PERF_ADC`, `PERF_PID`, `PERF_TX`) và các hàm ghi nhận microsecond. |
| **`src/perf_monitor.cpp`** | [perf_monitor.cpp](file:///d:/CAR_CHALLENGE_2026/doline17s/src/perf_monitor.cpp) | Thu thập thống kê min/max/mean của từng giai đoạn trong vòng lặp, tính chu kỳ vòng lặp, jitter và in báo cáo khi nhận lệnh `PERF` hoặc `TASKS`. |
| **`include/adc_bench.h`** | [adc_bench.h](file:///d:/CAR_CHALLENGE_2026/doline17s/include/adc_bench.h) | Khai báo module kiểm thử ADC 9 chế độ độc lập. |
| **`src/adc_bench.cpp`** | [adc_bench.cpp](file:///d:/CAR_CHALLENGE_2026/doline17s/src/adc_bench.cpp) | Triển khai đo kiểm so sánh 9 chế độ đọc ADC, tính thời gian MUX settle, sai số so với chế độ mẫu, không làm ảnh hưởng tới dữ liệu hiệu chuẩn NVS. |

### Nhóm 8: Khởi tạo & Điều phối Vòng lặp Chính (Main Core)
| File | Đường dẫn | Trách nhiệm chính & Nội dung cốt lõi |
| :--- | :--- | :--- |
| **`src/main.cpp`** | [main.cpp](file:///d:/CAR_CHALLENGE_2026/doline17s/src/main.cpp) | Trái tim điều phối firmware: xử lý SPP, nút CALIB/START đã debounce, state machine LED ACK, ADC, PID và telemetry. AUTO vật lý/SPP dùng chung handler; khi PID chốt đích, tắt AUTO, dừng xe và báo `[FINISH]`. |
| **`include/input_logic.h`** | [input_logic.h](file:///d:/CAR_CHALLENGE_2026/doline17s/include/input_logic.h) | Logic độc lập phần cứng cho debounce cạnh nhấn active-low 30 ms và state machine LED ACK 3 nhịp 200 ms, dùng chung bởi firmware và native smoke test. |

### Nhóm 9: Giao diện Cũ / BLE UART Dự phòng
| File | Đường dẫn | Trách nhiệm chính & Nội dung cốt lõi |
| :--- | :--- | :--- |
| **`include/ble_control.h`** | [ble_control.h](file:///d:/CAR_CHALLENGE_2026/doline17s/include/ble_control.h) | Khai báo dịch vụ BLE Nordic UART Service (NUS) dự phòng. |
| **`src/ble_control.cpp`** | [ble_control.cpp](file:///d:/CAR_CHALLENGE_2026/doline17s/src/ble_control.cpp) | Triển khai BLE Server GATT (tạm thời bị loại trừ khỏi bản build bởi `platformio.ini` để ưu tiên bộ nhớ cho Bluetooth Classic SPP). |
| **`index.html`** | [index.html](file:///d:/CAR_CHALLENGE_2026/doline17s/index.html) | Dashboard giao diện web HTML5/JS hỗ trợ Web Bluetooth API và Web Serial API (chạy trực tiếp trên trình duyệt Chrome/Edge). |

### Nhóm 10: Ứng dụng Điều khiển & Thu thập Dữ liệu Máy tính (Python)
| File | Đường dẫn | Trách nhiệm chính & Nội dung cốt lõi |
| :--- | :--- | :--- |
| **`robot_debug.py`** | [robot_debug.py](file:///d:/CAR_CHALLENGE_2026/doline17s/robot_debug.py) | Ứng dụng GUI máy tính (Tkinter) kết nối SPP: Hiển thị 16 mắt cảm biến, PID, tốc độ và thời gian phiên chạy; đồng hồ bắt đầu theo PWM chuyển động, chốt khi hai PWM cùng dừng ổn định hoặc AUTO dừng; quản lý CSV theo START/STOP. |
| **`spp_csv_recorder.py`** | [spp_csv_recorder.py](file:///d:/CAR_CHALLENGE_2026/doline17s/spp_csv_recorder.py) | Module đa luồng chuyên dụng ghi nhận dữ liệu 300 Hz vào file CSV, chèn dòng metadata `CAL_THRESHOLDS`, tự động flush đĩa định kỳ 1 giây. |
| **`run_csv_lifecycle.py`** | [run_csv_lifecycle.py](file:///d:/CAR_CHALLENGE_2026/doline17s/run_csv_lifecycle.py) | Quản lý vòng đời file CSV tương ứng với từng phiên chạy START và STOP. |

### Nhóm 11: Unit Test & Kiểm thử Tĩnh Native C++
| File | Đường dẫn | Trách nhiệm chính & Nội dung cốt lõi |
| :--- | :--- | :--- |
| **`tests/native/Arduino.h`** | [Arduino.h](file:///d:/CAR_CHALLENGE_2026/doline17s/tests/native/Arduino.h) | Mock header giả lập các hàm của Arduino (`millis`, `micros`) để biên dịch trên môi trường máy tính bằng g++/clang mà không cần vi điều khiển thật. |
| **`tests/native/pid_stability_smoke.cpp`** | [pid_stability_smoke.cpp](file:///d:/CAR_CHALLENGE_2026/doline17s/tests/native/pid_stability_smoke.cpp) | Kịch bản kiểm thử tự động toàn diện thuật toán PID: Giả lập vạch đen, vạch trắng, cua gắt 90 độ, mất line, vạch nét đứt, kiểm tra ngưỡng $TH_i$, kiểm tra cơ chế chống trôi xe khi khởi động. |
| **`tests/native/README.md`** | [README.md (native)](file:///d:/CAR_CHALLENGE_2026/doline17s/tests/native/README.md) | Hướng dẫn lệnh chạy biên dịch smoke test native bằng lệnh g++ đơn giản. |

### Nhóm 12: Tài liệu Hướng dẫn & Nhật ký Phiên bản
| File | Đường dẫn | Trách nhiệm chính & Nội dung cốt lõi |
| :--- | :--- | :--- |
| **`README.md`** | [README.md](file:///d:/CAR_CHALLENGE_2026/doline17s/README.md) | Tài liệu giới thiệu tổng quan dự án (Lưu ý: Một số sơ đồ chân trong file này là từ phiên bản cũ, cần đối chiếu với `pinout.h`). |
| **`README_CLASSIC_SPP.md`**| [README_CLASSIC_SPP.md](file:///d:/CAR_CHALLENGE_2026/doline17s/README_CLASSIC_SPP.md) | Tài liệu kỹ thuật chi tiết về giao thức Bluetooth Classic SPP 300 Hz, cấu trúc gói tin 72 bytes, các bản vá ổn định PID ngày 29/10/2026. |
| **`README_ENCODER_MAINV4.md`**| [README_ENCODER_MAINV4.md](file:///d:/CAR_CHALLENGE_2026/doline17s/README_ENCODER_MAINV4.md) | Tài liệu kỹ thuật tích hợp encoder x4 từ nhánh `mainv2` sang `mainv4`. |
| **`README_MAINV2_PATCH.md`**| [README_MAINV2_PATCH.md](file:///d:/CAR_CHALLENGE_2026/doline17s/README_MAINV2_PATCH.md) | Ghi chép lịch sử bản vá telemetry BLE trên nhánh cũ. |

---

## 5. CẨM NANG HƯỚNG DẪN CHO AI AGENT: KHI NÀO CẦN SỬA FILE NÀO?

Để tránh lãng phí token và thời gian đọc lại toàn bộ code, khi nhận một tác vụ cụ thể, agent chỉ cần tra cứu bảng phân công công việc dưới đây:

| Tình huống / Nhiệm vụ cần thực hiện | File cần mở và chỉnh sửa | Hướng dẫn thao tác nhanh |
| :--- | :--- | :--- |
| **Thay đổi chân cắm phần cứng (đổi GPIO cảm biến, motor, encoder, nút bấm, LED)** | [include/pinout.h](file:///d:/CAR_CHALLENGE_2026/doline17s/include/pinout.h) | Sửa các macro `PIN_*`. Lưu ý: 4 chân MUX $S_0-S_3$ phải là các chân $<32$ để hỗ trợ ghi thanh ghi direct mask trong `sensor.cpp`. |
| **Bánh xe quay ngược chiều hoặc đếm xung encoder bị lùi khi xe tiến** | [include/pinout.h](file:///d:/CAR_CHALLENGE_2026/doline17s/include/pinout.h) | Đổi dấu macro `ENCODER_LEFT_SIGN` hoặc `ENCODER_RIGHT_SIGN` (từ `1` thành `-1` hoặc ngược lại). |
| **Thay đổi tần số PWM hoặc độ phân giải điều khiển động cơ** | [include/motor.h](file:///d:/CAR_CHALLENGE_2026/doline17s/include/motor.h) | Thay đổi `MOTOR_PWM_FREQ` (hiện tại `10000`) hoặc `MOTOR_PWM_RESOLUTION` (hiện tại `8`). |
| **Chỉnh sửa công thức PID, độ nhạy khi vào cua, hoặc xử lý vạch đích full đen** | [src/pid_controller.cpp](file:///d:/CAR_CHALLENGE_2026/doline17s/src/pid_controller.cpp)<br>[include/pid_controller.h](file:///d:/CAR_CHALLENGE_2026/doline17s/include/pid_controller.h)<br>[src/main.cpp](file:///d:/CAR_CHALLENGE_2026/doline17s/src/main.cpp)<br>[robot_debug.py](file:///d:/CAR_CHALLENGE_2026/doline17s/robot_debug.py) | - Chỉnh thuật toán PID và ngưỡng cua trong `pid_controller.cpp`.<br>- Vạch đích chỉ được chốt khi đủ 16 mắt đen liên tục 50 ms; main dừng xe/báo `[FINISH]`, dashboard chốt đồng hồ và CSV.<br>- Cập nhật test native tại `tests/native/pid_stability_smoke.cpp`. |
| **Thay đổi thuật toán hiệu chuẩn cảm biến (Calibration) hoặc ngưỡng nhận diện vạch đen** | [src/sensor.cpp](file:///d:/CAR_CHALLENGE_2026/doline17s/src/sensor.cpp)<br>[include/sensor.h](file:///d:/CAR_CHALLENGE_2026/doline17s/include/sensor.h) | - Đổi số khung hình tích lũy `CAL_FRAMES` (128).<br>- Đổi tỉ lệ tương phản nhận diện vạch đen `SENSOR_BLACK_THRESHOLD_CAL` (400 = 40%).<br>- Đổi điều kiện kiểm tra độ lệch chuẩn tối đa `CAL_MAX_STDDEV` (90). |
| **Thêm / bớt trường dữ liệu truyền về máy tính qua Bluetooth Telemetry** | [include/bt_telemetry.h](file:///d:/CAR_CHALLENGE_2026/doline17s/include/bt_telemetry.h)<br>[src/bt_telemetry.cpp](file:///d:/CAR_CHALLENGE_2026/doline17s/src/bt_telemetry.cpp)<br>[src/main.cpp](file:///d:/CAR_CHALLENGE_2026/doline17s/src/main.cpp)<br>[robot_debug.py](file:///d:/CAR_CHALLENGE_2026/doline17s/robot_debug.py) | - **Bước 1:** Cập nhật struct `Telemetry` trong `bt_telemetry.h` (nhớ kiểm tra `static_assert(sizeof(Telemetry)==...)`).<br>- **Bước 2:** Cập nhật hàm `sendFrame()` trong `main.cpp` để gán dữ liệu.<br>- **Bước 3:** Cập nhật chuỗi unpack `FMT` và `CSV_HEADER` trong `robot_debug.py`. |
| **Thêm tập lệnh điều khiển mới qua Bluetooth SPP (Command parser)** | [src/main.cpp](file:///d:/CAR_CHALLENGE_2026/doline17s/src/main.cpp) | Tìm hàm `command(const String &raw)` trong namespace nội bộ, thêm nhánh `if(s == "TEN_LENH")` và xử lý phản hồi bằng `reply(...)`. |
| **Thay đổi tần số gửi Telemetry mặc định** | [src/main.cpp](file:///d:/CAR_CHALLENGE_2026/doline17s/src/main.cpp) | Sửa biến `streamHz = 300` và tính toán lại `streamPeriodUs`. Có thể chỉnh động qua lệnh `STREAM:100` từ app. |
| **Chỉnh sửa giao diện điều khiển máy tính, đồ thị hoặc xuất file CSV** | [robot_debug.py](file:///d:/CAR_CHALLENGE_2026/doline17s/robot_debug.py)<br>[spp_csv_recorder.py](file:///d:/CAR_CHALLENGE_2026/doline17s/spp_csv_recorder.py) | Sửa giao diện Tkinter, đồ thị Canvas trong `robot_debug.py`. Chỉnh sửa định dạng cột hoặc xử lý ghi file trong `spp_csv_recorder.py`. |
| **Kiểm thử logic PID nhanh trên máy tính không cần nạp vi điều khiển** | [tests/native/pid_stability_smoke.cpp](file:///d:/CAR_CHALLENGE_2026/doline17s/tests/native/pid_stability_smoke.cpp) | Viết thêm test case kịch bản cảm biến trong file này, sau đó chạy biên dịch bằng `g++ -std=c++17` theo hướng dẫn tại `tests/native/README.md`. |
| **Khắc phục lỗi biên dịch hoặc chuyển đổi giữa Bluetooth Classic và BLE** | [platformio.ini](file:///d:/CAR_CHALLENGE_2026/doline17s/platformio.ini) | Sửa dòng `build_src_filter`. Nếu bật BLE thì bỏ `bt_telemetry.cpp` và thêm `ble_control.cpp`. |

---
*Tài liệu được khởi tạo và kiểm tra tính nhất quán trực tiếp từ mã nguồn thực tế của dự án `doline17s` (Tháng 10/2026).*
