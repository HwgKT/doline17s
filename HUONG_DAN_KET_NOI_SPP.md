# HƯỚNG DẪN KẾT NỐI BLUETOOTH CLASSIC SPP CHO XE DÒ LINE DOLINE17S

Tài liệu này hướng dẫn chi tiết từng bước từ cài đặt, ghép đôi Bluetooth trên Windows, xác định đúng cổng COM Bluetooth, đến vận hành xe qua ứng dụng Python Dashboard (`robot_debug.py`).

---

## 📌 MỤC LỤC
1. [Chuẩn bị Phần cứng & Nạp Firmware](#1-chuẩn-bị-phần-cứng--nạp-firmware)
2. [Cài đặt Môi trường Python trên Máy tính](#2-cài-đặt-môi-trường-python-trên-máy-tính)
3. [Ghép đôi (Pair) Bluetooth trên Windows 10/11](#3-ghép-đôi-pair-bluetooth-trên-windows-1011)
4. [Xác định Cổng COM Outgoing của Bluetooth](#4-xác-định-cổng-com-outgoing-của-bluetooth)
5. [Khởi chạy Ứng dụng & Kết nối Giám sát](#5-khởi-chạy-ứng-dụng--kết-nối-giám-sát)
6. [Quy trình Vận hành Chuẩn trên Đường đua](#6-quy-trình-vận-hành-chuẩn-trên-đường-đua)
7. [Tập lệnh Điều khiển Nhanh qua SPP Terminal](#7-tập-lệnh-điều-khiển-nhanh-qua-spp-terminal)
8. [Xử lý Sự cố Thường gặp (Troubleshooting)](#8-xử-lý-sự-cố-thường-gặp-troubleshooting)

---

## 1. CHUẨN BỊ PHẦN CỨNG & NẠP FIRMWARE

### Bước 1.1: Nạp Firmware lên ESP32
Cắm cáp USB kết nối ESP32 với máy tính, mở terminal tại thư mục dự án và chạy:
```powershell
cd d:\CAR_CHALLENGE_2026\doline17s
pio run -e esp32dev -t upload
```

### Bước 1.2: Quan sát Đèn LED Báo trạng thái (GPIO 2)
Sau khi nạp code và cấp nguồn cho xe (bằng pin hoặc cáp USB):
- **LED nhấp nháy đều đặn (mỗi 400ms):** ESP32 đã khởi động xong Bluetooth SPP với tên **`LineRobot`**, đang ở chế độ chờ máy tính kết nối.
- **LED sáng đứng liên tục:** Đã có thiết bị kết nối SPP thành công.
- **LED tắt hoàn toàn:** Chưa bật Bluetooth hoặc mất kết nối.

> ⚠️ **LƯU Ý AN TOÀN:** Trước khi bật chế độ tự động chạy line (`AUTO`), luôn **kê cao bánh xe khỏi mặt đất** để kiểm tra chiều quay động cơ và cảm biến!

---

## 2. CÀI ĐẶT MÔI TRƯỜNG PYTHON TRÊN MÁY TÍNH

Ứng dụng giám sát [robot_debug.py](file:///d:/CAR_CHALLENGE_2026/doline17s/robot_debug.py) yêu cầu Python 3.8+ và thư viện `pyserial`.

Mở PowerShell hoặc Command Prompt trên Windows và chạy lệnh:
```powershell
pip install pyserial
```

*(Nếu máy tính chưa có Python, hãy tải từ [python.org](https://www.python.org/) và nhớ tick chọn **"Add Python to PATH"** khi cài đặt).*

---

## 3. GHÉP ĐÔI (PAIR) BLUETOOTH TRÊN WINDOWS 10/11

1. Bật công tắc nguồn của xe dò line (hoặc cắm nguồn cho ESP32). Đảm bảo LED xanh trên ESP32 đang nhấp nháy.
2. Trên máy tính Windows:
   - Nhấn phím `Windows + I` để mở **Settings**.
   - Chọn **Bluetooth & devices** (hoặc *Devices* trên Windows 10).
   - Đảm bảo Bluetooth của máy tính đang **ON**.
   - Bấm nút **Add device** $\rightarrow$ chọn **Bluetooth**.
3. Danh sách thiết bị quét được sẽ hiện ra:
   - Tìm và chọn thiết bị có tên **`LineRobot`**.
   - Nếu Windows yêu cầu mã PIN, nhập `1234` hoặc `0000` (thông thường ESP32 sẽ tự động pair không cần mã).
   - Chờ vài giây cho đến khi Windows báo **"Your device is ready to go!"** hoặc **"Paired"**.

---

## 4. XÁC ĐỊNH CỔNG COM OUTGOING CỦA BLUETOOTH

Khi ghép đôi thiết bị Bluetooth SPP, Windows sẽ tự động tạo ra **2 cổng COM ảo** (1 cổng Outgoing và 1 cổng Incoming). Bạn bắt buộc phải chọn đúng cổng **Outgoing** thì app mới kết nối được.

### Cách kiểm tra:
1. Nhấn tổ hợp phím `Windows + R`, gõ `devmgmt.msc` rồi nhấn **Enter** để mở **Device Manager**.
2. Tìm và mở rộng mục **Ports (COM & LPT)**:
   - Bạn sẽ thấy cổng nạp code USB (ví dụ: *Silicon Labs CP210x... (COM3)* hoặc *CH340 (COM3)*).
   - Đồng thời xuất hiện thêm 2 cổng ghi là **Standard Serial over Bluetooth link (COMx)** và **(COMy)**.
3. Để biết chính xác cổng nào là Outgoing:
   - Mở cửa sổ Run (`Windows + R`), gõ: `control bthprops.cpl` rồi nhấn **Enter**.
   - Chuyển sang thẻ **COM Ports**.
   - Xem dòng có ghi tên thiết bị **LineRobot** với hướng **Outgoing**: Hãy ghi nhớ số cổng COM này (Ví dụ: `COM8`).

---

## 5. KHỞI CHẠY ỨNG DỤNG & KẾT NỐI GIÁM SÁT

### Bước 5.1: Mở Dashboard Python
Tại cửa sổ Terminal, chạy lệnh:
```powershell
python d:\CAR_CHALLENGE_2026\doline17s\robot_debug.py
```

### Bước 5.2: Thực hiện Kết nối
Giao diện **ESP32 Line Robot - Bluetooth SPP Dashboard** xuất hiện:
1. Nhìn lên góc trên bên trái tại ô **Cổng COM**:
   - Bấm vào menu thả xuống để chọn cổng **COM Outgoing** của `LineRobot` vừa xác định ở Mục 4 (ví dụ: `COM8`).
   - Tốc độ baud để mặc định `115200`.
2. Bấm nút **Kết nối**:
   - Khi kết nối thành công, nút chuyển thành màu đỏ với chữ **Ngắt kết nối**.
   - Đèn LED trên ESP32 sẽ **chuyển từ nháy sang sáng đứng liên tục**.
   - Dải trạng thái dưới đáy sẽ hiển thị `RX Hz: ~300`, `PID Hz`, `LOOP Hz` nhảy số liên tục.
   - Bảng 16 cột cảm biến bên trái bắt đầu hiển thị giá trị Analog và đồ thị dạng sóng.

---

## 6. QUY TRÌNH VẬN HÀNH CHUẨN TRÊN ĐƯỜNG ĐUA

Thực hiện tuần tự 4 bước sau để đảm bảo xe chạy chính xác và lưu log dữ liệu:

```
+------------------+     +------------------+     +------------------+     +------------------+
| 1. HIỆU CHUẨN    |     | 2. ĐẶT THAM SỐ   |     | 3. BẮT ĐẦU CHẠY  |     | 4. DỪNG & LƯU LOG|
| TRẮNG -> ĐEN     | --> | BASE SPEED & PID | --> | BẤM "BẮT ĐẦU"    | --> | BẤM "DỪNG" (ESC) |
| (CAL_WHITE/BLACK)|     | (Kp, Ki, Kd)     |     | (AUTO)           |     | (Tự đóng CSV)    |
+------------------+     +------------------+     +------------------+     +------------------+
```

### Bước 1: Hiệu chuẩn cảm biến (Calibration)
*(Chỉ cần làm lần đầu tiên hoặc khi thay đổi bề mặt sân thi đấu; giá trị sẽ được lưu vĩnh viễn vào bộ nhớ flash NVS của ESP32)*:
1. Đặt toàn bộ 16 mắt cảm biến của xe lên **bề mặt nền trắng hoàn toàn**.
2. Trên app bấm nút **Lấy mẫu Trắng (White)** (hoặc bấm nút vật lý `GPIO 34` trên xe 1 lần). Chờ 1 giây để app báo `CAL_WHITE_DONE`.
3. Nhấc xe sang đặt cả 16 mắt lên **bề mặt vạch đen hoàn toàn**.
4. Trên app bấm nút **Lấy mẫu Đen (Black)** (hoặc bấm nút vật lý lần 2).
5. Khi màn hình hiện `CAL_OK_SAVED`, hệ thống đã tính toán xong 16 ngưỡng $TH_0 \dots TH_{15}$ và lưu an toàn vào NVS.
6. Nếu thao tác trực tiếp bằng nút vật lý, bấm GPIO34 lần 3 để chạy AUTO PID; cấu hình PID đã lưu trong NVS được dùng (nếu chưa có profile hợp lệ thì dùng mặc định).

### Bước 2: Thiết lập Tốc độ Cơ sở & PID
- **Tốc độ cơ bản (`Base Speed`):** Đặt từ `100` đến `160` (tối đa 255).
- **Bộ thông số gợi ý ban đầu:**
  - $K_p = 3.8$
  - $K_i = 0.015$
  - $K_d = 2.0$
- Nhập giá trị vào ô và bấm **Gửi** tương ứng.

### Bước 3: Cho xe chạy dò line (`AUTO`)
1. Đặt xe vào vạch xuất phát, đảm bảo các mắt giữa nằm trên vạch đen.
2. Bấm nút màu xanh **BẮT ĐẦU DÒ LINE (AUTO)**:
   - App sẽ tự động tạo một file log độc lập: `logs/linerobot_YYYYMMDD_HHMMSS.csv`.
   - ESP32 nhận lệnh và bắt đầu bám line tự động ở tần số ~300 Hz.
   - Đồng hồ bấm giờ trên dashboard bắt đầu đếm thời gian thực.

### Bước 4: Dừng xe và xuất file Log
- Khi xe hoàn thành vòng đua, bấm nút màu đỏ **DỪNG (STOP)** (hoặc nhấn phím **Escape** trên bàn phím).
- Xe sẽ lập tức dừng động cơ.
- File CSV tương ứng cho lần chạy đó sẽ tự động đóng lại, xả hết đệm ghi xuống ổ cứng và thông báo đường dẫn file hoàn tất tại cửa sổ terminal.

---

## 7. TẬP LỆNH ĐIỀU KHIỂN NHANH QUA SPP TERMINAL

Bạn có thể gõ các lệnh sau vào ô nhập lệnh ở góc dưới bên phải giao diện rồi nhấn **Enter**:

| Lệnh | Ý nghĩa | Ví dụ |
| :--- | :--- | :--- |
| `AUTO` hoặc `START` | Bắt đầu chạy tự động dò line | `AUTO` |
| `STOP` hoặc `M` | Dừng động cơ, chuyển về lái tay | `STOP` |
| `ESTOP` | Dừng khẩn cấp phần mềm (khóa mọi lệnh chạy) | `ESTOP` |
| `RESET_ESTOP` | Mở khóa dừng khẩn cấp (động cơ vẫn dừng) | `RESET_ESTOP` |
| `BASE:xxx` | Cài đặt tốc độ cơ sở (0 – 255) | `BASE:130` |
| `KP:x.x` | Cài đặt hệ số Kp | `KP:4.2` |
| `KI:x.x` | Cài đặt hệ số Ki | `KI:0.02` |
| `KD:x.x` | Cài đặt hệ số Kd | `KD:2.5` |
| `STREAM:xxx` | Đổi tần số gửi Telemetry (0, 20, 50, 100, 200, 300) | `STREAM:300` |
| `ENC_RESET` | Reset số xung cả hai bánh về 0 | `ENC_RESET` |
| `STATUS` | Xem trạng thái kết nối, pinout, chế độ ADC, lỗi | `STATUS` |
| `PERF` | Báo cáo chi tiết thời gian đo ($\mu s$) các giai đoạn vòng lặp | `PERF` |
| `F`, `B`, `L`, `R` | Lái tay: Tiến, Lùi, Rẽ Trái, Rẽ Phải (chạy 750ms tự ngắt) | `F` |

---

## 8. XỬ LÝ SỰ CỐ THƯỜNG GẶP (TROUBLESHOOTING)

### ❌ Sự cố 1: Máy tính quét không thấy Bluetooth "LineRobot"
- **Nguyên nhân:** ESP32 chưa có điện hoặc đang chạy firmware cũ chưa kích hoạt Bluetooth Classic.
- **Khắc phục:** 
  1. Kiểm tra đèn nguồn trên ESP32.
  2. Kiểm tra xem đèn LED onboard GPIO 2 có đang nháy mỗi 400ms không.
  3. Mở [platformio.ini](file:///d:/CAR_CHALLENGE_2026/doline17s/platformio.ini), đảm bảo dòng `build_src_filter = +<*> -<ble_control.cpp>` và nạp lại code bằng lệnh `pio run -e esp32dev -t upload`.

### ❌ Sự cố 2: App báo lỗi "PermissionError" hoặc "Access Denied" khi bấm Kết nối
- **Nguyên nhân:** Cổng COM đó đang bị một chương trình khác chiếm dụng (ví dụ: Serial Monitor của PlatformIO, Arduino IDE, PuTTY, hoặc một tab debug khác).
- **Khắc phục:** Đóng tất cả các phần mềm mở cổng Serial khác rồi bấm Kết nối lại.

### ❌ Sự cố 3: Đã kết nối thành công nhưng RX Hz = 0, đồ thị không nhảy
- **Nguyên nhân:** Bạn đã chọn nhầm vào cổng **Incoming** thay vì cổng **Outgoing**, hoặc chọn nhầm vào cổng nạp USB.
- **Khắc phục:** Ngắt kết nối, chọn cổng COM Bluetooth còn lại trong danh sách rồi kết nối lại (xem lại Mục 4).

### ❌ Sự cố 4: Bấm "BẮT ĐẦU DÒ LINE" nhưng xe không chạy, báo `ERR: CALIB_REQUIRED`
- **Nguyên nhân:** Xe chưa được hiệu chuẩn trắng/đen nên hệ thống bảo vệ từ chối chạy để tránh đâm xe.
- **Khắc phục:** Thực hiện lấy mẫu Trắng $\rightarrow$ Đen theo đúng Bước 1 tại Mục 6.

### ❌ Sự cố 5: Xe bị rung lắc liên tục (Fish-tailing) khi chạy thẳng
- **Nguyên nhân:** Hệ số $K_p$ quá cao hoặc $K_d$ chưa đủ để giảm chấn.
- **Khắc phục:** Giảm $K_p$ xuống (ví dụ từ `3.8` về `2.5` - `3.0`), hoặc tăng nhẹ $K_d$ (từ `2.0` lên `2.5`).

---
*Tài liệu hướng dẫn kết nối Bluetooth Classic SPP - Dự án Line Robot Doline17s.*
