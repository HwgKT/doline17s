# ESP32 16-Sensor High-Speed Line Follower Robot (PID + Bluetooth Classic Debug)

Hệ thống điều khiển xe dò line tốc độ cao sử dụng vi điều khiển **ESP32**, thanh cảm biến quang trở hồng ngoại **16 mắt qua IC giải mã CD74HC4067**, thuật toán **PID phi tuyến kết hợp điều tốc động (Dynamic Speed Profiling)**, và giao diện giám sát/điều khiển qua **Bluetooth Classic SPP với robot_debug.py** trên nhánh `mainv4`.

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
- 🔄 **Hiệu chuẩn 2 bề mặt:** Lấy mẫu đồng thời 16 mắt lần lượt trên nền trắng và đen (128 khung mỗi nền), kiểm tra nhiễu/tương phản rồi lưu NVS với CRC32. Không dùng lia Min/Max nữa.
- 🚀 **Bứt tốc đường thẳng & Tự hạ ga ôm cua (Dynamic Speed Curve):** Tự động bốc ga kịch trần (lên tới 215–255 PWM) khi đi thẳng và tự ghì ga xuống 95–115 PWM khi vào cua để bánh xe bám chặt mặt đường.
- 🛡️ **Khâu vi phân $K_d$ có lọc thông thấp (Anti-Oscillation IIR):** Triệt tiêu hoàn toàn hiện tượng rung lắc li ti hoặc lắc đuôi cá ở tốc độ cao.
- 📐 **Xử lý chuyên sâu Cua vuông 90° & Vạch đứt nét:** Phanh giật ngược chủ động (*Active Counter-Brake*) kết hợp quay tâm xe đón line tức thời.
- **Giám sát bằng app Python Bluetooth Classic:** 16 ADC, 16 giá trị chuẩn hóa, ngưỡng White/Black riêng từng mắt, logic 0/1, PID, PWM và encoder; xuất TXT/CSV.

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

| **Nút Calib** | `GPIO 34` | Bấm lần 1 để lấy trắng; chờ hoàn tất, bấm lần 2 để lấy đen |
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
| **`STAGE1`** hoặc **`SAFE12`** | Đổi ngay sang **Nấc 1 An Toàn (12–13s)**: thông số cũ chưa phù hợp công thức D mới, cần tinh chỉnh lại |
| **`STAGE2`** hoặc **`RACE10`** | Đổi ngay sang **Nấc 2 Đua 10s**: thông số cũ chưa phù hợp công thức D mới, cần tinh chỉnh lại |
| **`CAL_WHITE`** / **`C`** | Bắt đầu lấy mẫu 16 mắt trên nền **trắng** |
| **`CAL_BLACK`** | Sau `CAL_WHITE_DONE`, bắt đầu lấy mẫu trên nền **đen** và lưu NVS nếu đạt |
| **`CAL_STATUS`** | Xem trạng thái hiệu chuẩn trên ESP32 |
| **`CAL_DUMP`** | Đọc 16 cặp trắng/đen/threshold từ ESP32 qua SPP hoặc USB |
| **`BASE:xxx`** | Chỉnh tốc độ cơ sở (mặc định `BASE:120`) |
| **`KP:x.x`** | Chỉnh hệ số tỉ lệ Kp (mặc định `KP:2.2`) |
| **`KI:x.x`** | Chỉnh hệ số tích phân Ki (mặc định `KI:0`) |
| **`KD:x.x`** | Chỉnh hệ số vi phân Kd (mặc định `KD:0`; D = Kd × (e[k] − e[k−1]) / dt, với dt đo bằng giây) |
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


## Hiệu chuẩn 16 mắt — mainv4 Bluetooth Classic

1. Đặt đủ 16 mắt lên cùng một mặt **trắng** đồng nhất, giữ nguyên chiều cao, gửi `CAL_WHITE` (hoặc bấm nút CALIB một lần), chờ `CAL_WHITE_DONE`.
2. Chuyển đủ 16 mắt sang cùng một mặt **đen** đồng nhất, gửi `CAL_BLACK` (hoặc bấm nút CALIB lần hai sau bước trắng), chờ `CAL_OK_SAVED`.
3. `CAL_FAILED` có nghĩa là độ tương phản của ít nhất một mắt < 150 ADC, nhiễu quá lớn, hoặc không ghi được NVS; dữ liệu hợp lệ cũ được giữ nguyên.
4. Gửi `CAL_DUMP` để đọc 16 mức white/black được lưu và ngưỡng trung bình; app Python `robot_debug.py` tự truy vấn NVS sau khi kết nối.
5. Nếu ESP32 mất nguồn, firmware tự tải lại profile NVS hợp lệ. Cực tính phần cứng cố định: **ADC đen > ADC trắng**; `INV` không còn đảo cực.

SPP device: **`LineRobot`**; BLE Web **không được khởi tạo trong `mainv4`**. Protocol telemetry 72 byte và các chức năng PID/encoder giữ nguyên.

## Debug SPP và CSV trong app Python (mainv4)

- **Không có ngưỡng RAW chung** cho 16 mắt. Mắt `i` có `T_i = ceil((white_i + black_i) / 2)` tải từ NVS bằng `CAL_DUMP`. `CAL_i = clamp(1000*(ADC_i-white_i)/(black_i-white_i),0,1000)`; `logic_i = (CAL_i >= 500)`. `0=TRẮNG`, `1=ĐEN`. Trước khi đọc đủ NVS, app hiện `—`, không đoán logic theo ngưỡng giả.
- **App:** `robot_debug.py` có nút `Bảng 16 mắt` hiển thị RAW, W, B, T_i, CAL, logic từng mắt; terminal `Mở rộng`, `Xuất TXT`, `Xuất CSV`. Tùy chọn `Xem mẫu PID` mặc định bật, chỉ in 5 Hz để không tràn terminal; `Ghi CSV` lưu **toàn bộ** telemetry ở tần số nhận được, không giảm mẫu.
- **SPP telemetry:** giữ nguyên 72 byte/frame, header `AA 55`, payload 69 byte (`<I16H2i2h5fB`), byte XOR; để tương thích app cũ. Payload chứa `t_ms` tại snapshot sau PID, `adc0..15` raw 12-bit, `encL/encR` signed quadrature x4, `pwmL/pwmR` sau điều khiển (có dấu), `err,p,i,d,out,mode`. Dữ liệu đã chuẩn hóa và 16 bit logic do app **tính từ profile NVS thực**; không gửi lặp 16 giá trị CAL/logic qua Bluetooth.
- **Tần số gửi:** mặc định 50 Hz (72*50=3600 bytes/s), chỉnh `STREAM:20`, `STREAM:50`, `STREAM:100`, `STREAM:0` (tạm ngừng). `STREAM` truy vấn trạng thái. Không gửi các dòng TXT cho từng mẫu trên ESP32. 100 Hz là chế độ thử nghiệm, kiểm tra Hz thực tế, số lỗi gói và vòng PID; nếu chậm, về 20 hoặc 50 Hz.
- **CSV:** cột `host_iso` là thời gian PC nhận/xử lý, `t_ms` là đồng hồ ESP32, `adc0..15`, `cal0..15`, `logic0..15`, encoder, PWM, PID và chuỗi `logic16`. Các cột CAL/logic để trống trước khi tải đủ profile NVS. Xuất CSV lưu tối đa 15.000 mẫu gần nhất; `Ghi CSV` ghi liên tục để thu thập thí nghiệm dài hơn. Xuất TXT chỉ là log lệnh/trạng thái/PID preview, không thay thế CSV toàn tốc độ.
- **Kết nối:** tên Classic SPP là `LineRobot`; COM Windows chọn `Outgoing`. `COM4` kiểu CP210x chỉ là cổng USB trên máy người dùng ví dụ, không phải SPP.

