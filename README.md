# 🏎️ ESP32 16-Sensor High-Speed Line Follower Robot (PID + BLE Dashboard)

Hệ thống điều khiển xe dò line tốc độ cao sử dụng vi điều khiển **ESP32**, thanh cảm biến quang trở hồng ngoại **16 mắt qua IC giải mã CD74HC4067**, thuật toán **PID phi tuyến kết hợp điều tốc động (Dynamic Speed Profiling)**, và giao diện giám sát/điều khiển không dây qua **Web Bluetooth (BLE) / Web Serial Dashboard**.

---

## 📌 Mục lục
1. [Tính năng nổi bật](#-tính-năng-nổi-bật)
2. [Cấu hình phần cứng & Sơ đồ chân (Pinout)](#-cấu-hình-phần-cứng--sơ-đồ-chân-pinout)
3. [Kiến trúc Phần mềm & Cấu trúc Thư mục](#-kiến-trúc-phần-mềm--cấu-trúc-thư-mục)
4. [Thuật toán Dò Line & Các Chế độ Đua](#-thuật-toán-dò-line--các-chế-độ-đua)
5. [Tập lệnh điều khiển qua BLE / Serial Terminal](#-tập-lệnh-điều-khiển-qua-ble--serial-terminal)
6. [Giao diện Web Dashboard (index.html)](#-giao-diện-web-dashboard-indexhtml)
7. [Hướng dẫn cài đặt & Nạp code (PlatformIO)](#-hướng-dẫn-cài-đặt--nạp-code-platformio)

---

## 🌟 Tính năng nổi bật

- ⚡ **Chu kỳ lấy mẫu siêu tốc (~400Hz - 500Hz):** Phản xạ chỉ trong 2ms - 2.5ms, loại bỏ hoàn toàn hiện tượng trễ pha quán tính khi chạy tốc độ cao.
- 🎯 **Thanh cảm biến 16 mắt chuẩn hóa Analog (0 - 1000):** Tính toán trọng tâm liên tục (*Smooth Centroid*) với độ mịn cao từ `-65.0` đến `+65.0`.
- 🔄 **Cơ chế Hiệu chuẩn Quét (Sweep Calibration):** Chỉ cần lia xe qua vạch line trong 4 giây, hệ thống tự động lưu ngưỡng Min/Max và tính toán giá trị trung bình cho từng mắt riêng biệt.
- 🚀 **Bứt tốc đường thẳng & Tự hạ ga ôm cua (Dynamic Speed Curve):** Tự động bốc ga kịch trần (lên tới 215–255 PWM) khi đi thẳng và tự ghì ga xuống 95–115 PWM khi vào cua để bánh xe bám chặt mặt đường.
- 🛡️ **Khâu vi phân $K_d$ có lọc thông thấp (Anti-Oscillation IIR):** Triệt tiêu hoàn toàn hiện tượng rung lắc li ti hoặc lắc đuôi cá ở tốc độ cao.
- 📐 **Xử lý chuyên sâu Cua vuông 90° & Vạch đứt nét:** Phanh giật ngược chủ động (*Active Counter-Brake*) kết hợp quay tâm xe đón line tức thời.
- 📱 **Điều khiển & Tinh chỉnh không dây qua Web Bluetooth:** Giám sát trạng thái 16 mắt cảm biến, đo sai số, hiển thị PWM động cơ thời gian thực và chỉnh thông số PID trực tiếp từ trình duyệt điện thoại/máy tính.

---

## 🔌 Cấu hình phần cứng & Sơ đồ chân (Pinout)

### 1. Vi điều khiển
* **ESP32 DevKit V1** (Dual-core 240MHz, 320KB RAM, 4MB Flash)

### 2. Cảm biến 16 mắt (CD74HC4067 MUX)
* Mắt hoạt động: **Mắt 1 đến Mắt 14** (Mắt 0 và 15 bỏ qua để tối ưu khoảng cách).

| Chân CD74HC4067 | Chân ESP32 | Chức năng |
| :--- | :--- | :--- |
| **S0 (A)** | `GPIO 18` | Chọn kênh Bit 0 |
| **S1 (B)** | `GPIO 19` | Chọn kênh Bit 1 |
| **S2 (C)** | `GPIO 21` | Chọn kênh Bit 2 |
| **S3 (D)** | `GPIO 23` | Chọn kênh Bit 3 |
| **SIG (Signal Out)** | `GPIO 36 (ADC1_CH0)` | Tín hiệu Analog đọc về ESP32 |

### 3. Nút nhấn.

| **Nút Calib** | `GPIO 34` | Nút nhấn vật lý kích hoạt lia Calib |
| **Nút Start** | `GPIO35`  | Nút nhấn vật lý kích hoạt start     |

### 4. Động cơ & Driver (LEDC PWM 8-bit, 1000Hz)
| Chân Driver | Chân ESP32 | Kênh PWM LEDC | Chiều quay |
| :--- | :--- | :--- | :--- |
| **IN_L1** | `GPIO 25` | Kênh 0 (`CH_L1`) | Động cơ Trái - Tiến |
| **IN_L2** | `GPIO 26` | Kênh 1 (`CH_L2`) | Động cơ Trái - Lùi |
| **IN_R1** | `GPIO 27` | Kênh 2 (`CH_R1`) | Động cơ Phải - Tiến |
| **IN_R2** | `GPIO 13` | Kênh 3 (`CH_R2`) | Động cơ Phải - Lùi |

---

## 📁 Kiến trúc Phần mềm & Cấu trúc Thư mục

```text
d:/sensor/
├── include/
│   ├── ble_control.h      # Khai báo dịch vụ BLE Nordic UART & giao tiếp telemetry
│   ├── motor.h            # Khai báo điều khiển LEDC PWM 4 kênh động cơ
│   ├── pid_controller.h   # Khai báo thuật toán PID và phân loại các Case dò line
│   └── sensor.h           # Khai báo quét MUX 16 kênh, chuẩn hóa ADC và Calib
├── src/
│   ├── ble_control.cpp    # Triển khai BLE Server, xử lý lệnh và gửi trạng thái
│   ├── main.cpp           # Khởi tạo hệ thống, vòng lặp điều khiển thời gian thực
│   ├── motor.c            # Điều khiển động cơ tiến/lùi/phanh độc lập
│   ├── pid_controller.c   # Thuật toán PID phi tuyến, bộ lọc IIR, xử lý cua 90°
│   └── sensor.c           # Quét ADC qua MUX 74HC4067, tự tính ngưỡng tự động
├── index.html             # Dashboard Web Bluetooth / Web Serial điều khiển xe
├── platformio.ini         # Cấu hình biên dịch PlatformIO cho ESP32 DevKit V1
└── README.md              # Tài liệu hướng dẫn hệ thống
```

---



## 📡 Tập lệnh điều khiển qua BLE / Serial Terminal

Bạn có thể gửi các lệnh sau từ máy tính (qua cáp USB 115200 baud) hoặc từ điện thoại (qua Bluetooth BLE UART):

| Lệnh | Ý nghĩa & Chức năng |
| :--- | :--- |
| **`A`** hoặc **`AUTO`** | **BẬT** chế độ tự động dò line theo PID |
| **`M`** hoặc **`MANUAL`** | **TẮT** tự động, chuyển sang lái tay |
| **`STAGE1`** hoặc **`SAFE12`** | Đổi ngay sang **Nấc 1 An Toàn (12–13s)**: `Base=175, Kp=1.55, Kd=3.4` |
| **`STAGE2`** hoặc **`RACE10`** | Đổi ngay sang **Nấc 2 Đua 10s**: `Base=205, Kp=1.75, Kd=4.2` |
| **`C`** hoặc **`W`** | Bắt đầu chu trình **Lia Calib 4s** (lia qua vạch đen/trắng) |
| **`K`** | Chốt kết thúc sớm chu trình Calib |
| **`INV`** | Đảo cực tính cảm biến (Vạch đen nền trắng $\leftrightarrow$ Vạch trắng nền đen) |
| **`BASE:xxx`** | Chỉnh tốc độ cơ sở (mặc định `BASE:120`) |
| **`KP:x.x`** | Chỉnh hệ số tỉ lệ Kp (mặc định `KP:2.2`) |
| **`KI:x.x`** | Chỉnh hệ số tích phân Ki (mặc định `KI:0`) |
| **`KD:x.x`** | Chỉnh hệ số vi phân Kd (mặc định `KD:10`) |
| **`V:xxx`** | Chỉnh tốc độ tay và tốc độ cơ sở PID (0–255) |
| **`PWM`** | Xem giá trị PWM 2 bánh và sai số tức thời |
| **`PID`** | Xem các thông số PID và Base Speed hiện tại |
| **`F` / `B` / `L` / `R` / `S`** | Lái tay: Tiến / Lùi / Trái / Phải / Dừng xe |

---

## 💻 Giao diện Web Dashboard (`index.html`)

Dự án tích hợp sẵn file [index.html](index.html) là một Dashboard điều khiển trực quan bằng công nghệ **Web Bluetooth API** và **Web Serial API** (chạy trực tiếp trên Google Chrome / Microsoft Edge mà không cần cài thêm app):

1. **Thanh 16 LED cảm biến:** Hiển thị thời gian thực trạng thái nhận diện đen/trắng của từng mắt từ 0 đến 15.
2. **Telemetry động cơ:** Hiển thị trực quan tốc độ PWM bánh trái/phải, sai số lệch tâm `Err`, và trạng thái `Case`.
3. **Bàn phím ảo D-pad & Thanh trượt:** Lái tay, điều chỉnh tốc độ, chuyển đổi Auto/Manual nhanh chóng.
4. **Terminal tích hợp:** Gửi và nhận lệnh gỡ lỗi trực tiếp qua Bluetooth.

---

## 🛠️ Hướng dẫn cài đặt & Nạp code (PlatformIO)

### 1. Chuẩn bị môi trường
- Cài đặt [VS Code](https://code.visualstudio.com/)
- Cài đặt extension **PlatformIO IDE** trong VS Code.

### 2. Biên dịch & Nạp code
Mở Terminal tại thư mục dự án và chạy:

```bash
# 1. Biên dịch kiểm tra mã nguồn
pio run

# 2. Cắm cáp nạp firmware vào ESP32
pio run -t upload

# 3. Mở Serial Monitor theo dõi
pio run -t monitor
```

---

## 🏆 Bản quyền & Tác giả
- **Tác giả:** [congdoan23082006-svg](https://github.com/congdoan23082006-svg)
- **Organization:** [clb-robot-line-2026](https://github.com/clb-robot-line-2026)
- **Repository:** [ECcodedoline](https://github.com/clb-robot-line-2026/ECcodedoline.git)
- Giấy phép: MIT License.
