# DOLINE17S — PID-only baseline

Nhánh **`mainPID`** dùng để kiểm tra bộ điều khiển PID dò line với tốc độ tiến cơ sở cố định. Nhánh được tách từ `mainHighSpeedv3`; các nhánh HighSpeed gốc không bị thay đổi.

## 1. Luật điều khiển

Vị trí line được tính từ trọng tâm có trọng số của 16 cảm biến, trong khoảng xấp xỉ `-75..+75`. Sai số (e) là độ lệch của line so với tâm cảm biến.

[
P=K_p e,qquad
I=K_iint e(t),dt,qquad
D=K_drac{de(t)}{dt}
]

[
u=P+I+D
]

[
PWM_L=operatorname{sat}(BASE+u),qquad
PWM_R=operatorname{sat}(BASE-u)
]

Trong đó `BASE` được giữ cố định ở các chu kỳ PID bình thường; PID chỉ tạo chênh lệch tốc độ giữa hai bánh. Lệnh PWM từng bánh được giới hạn trong khoảng `[-255,255]`. Khi một bánh bão hòa, tổng PWM hai bánh có thể không còn bằng (2\,BASE).

P đã được chuyển về dạng tuyến tính `Kp * error`; I vẫn có giới hạn tích phân và suy giảm khi sai số đổi dấu; D vẫn dùng lọc thông thấp để hạn chế nhiễu.

## 2. Các cơ chế tốc độ đã tắt

- Không còn tăng tốc từ BASE lên HS_PEAK theo encoder.
- Không còn giữ CRUISE rồi phanh theo encoder hoặc giữ CORNER HOLD.
- Không còn Speed Governor giảm tốc theo độ lớn sai số.
- Không còn giảm tốc theo cảm biến ngoài cùng.
- Không còn tăng tốc theo thời gian sau khi bắt lại line.
- Không còn hệ số P phi tuyến hoặc ramp tốc độ rẽ theo thời gian.

Các API `HS_PEAK` được giữ lại để tương thích giao thức SPP cũ. Firmware trả `PID_ONLY=1`; giá trị HS_PEAK chỉ còn là metadata và **không tác động đến PWM động cơ**.

## 3. Những xử lý vẫn được giữ

Để không làm thay đổi toàn bộ logic chạy trên sa bàn, nhánh này vẫn giữ:

- Hiệu chuẩn trắng/đen, đọc 16 cảm biến và tính trọng tâm line.
- Debounce khi bắt đầu nhận line và phát hiện mất line.
- MAP 1/2/3 cùng cửa sổ nhận diện đoạn line đứt. Encoder chỉ được dùng để nhận diện cửa sổ GAP, không còn dùng để điều chỉnh tốc độ HighSpeed.
- GAP_BRIDGE giữ PWM tiến gần nhất trong tối đa 100 ms.
- DASHED_LINE và STARTUP_COAST có giới hạn an toàn `min(BASE, 95)`; tìm lại line dùng PWM cố định 85.
- Dừng khi cả 16 cảm biến nhận vạch đích trong 3 chu kỳ liên tiếp.
- E-STOP, calibration, Bluetooth Classic SPP, telemetry và ghi encoder.

Vì các trạng thái mất line/GAP là cơ chế phục hồi riêng, khi đánh giá PID nên phân biệt dữ liệu ở trạng thái `NORMAL_PID` với dữ liệu ở các trạng thái phục hồi.

## 4. Giao diện web

File `index.html` được giữ nguyên so với `mainHighSpeedv3`. Các nút AUTO/MANUAL, thanh BASE, chỉnh Kp/Ki/Kd, telemetry, terminal và kết nối Web Bluetooth/Web Serial không bị thay đổi.

Có thể dùng các lệnh SPP hiện tại:

| Lệnh | Chức năng |
|---|---|
| `PID` | Đọc Kp, Ki, Kd và BASE |
| `KP:x`, `KI:x`, `KD:x` | Chỉnh hệ số PID |
| `BASE:n` hoặc `V:n` | Đặt tốc độ tiến cơ sở |
| `MAP:1/2/3` | Chọn bản đồ thử nghiệm |
| `AUTO:1/2/3` | Chọn MAP và bắt đầu dò line |
| `STOP` | Dừng AUTO |
| `ESTOP` | Dừng khẩn cấp và khóa AUTO |
| `DIAG`, `ENC`, `STATUS` | Đọc dữ liệu chẩn đoán |
| `HS_PEAK` | Chỉ đọc metadata tương thích; không bật HighSpeed |

## 5. Quy trình thử nghiệm đề xuất

1. Nạp firmware, hiệu chuẩn WHITE rồi BLACK và kiểm tra đủ 16 ngưỡng.
2. Đặt MAP cố định, đặt BASE và đọc lại bằng lệnh `PID`.
3. Giữ nguyên BASE, Kp, Ki, Kd trong toàn bộ một nhóm lượt thử; chỉ thay một tham số giữa các nhóm.
4. Ghi CSV/telemetry và so sánh sai số line, PWM hai bánh, tỷ lệ bão hòa PWM, số lần mất line, số lần bắt lại line và tỷ lệ hoàn thành.
5. Thực hiện nhiều lượt lặp trên cùng MAP và cùng vị trí xuất phát trước khi kết luận PID đã ổn định.

## 6. Biên dịch và kiểm tra

Biên dịch firmware bằng PlatformIO:

```bash
pio run
pio run -t upload
```

Chạy native smoke test trên máy tính có g++:

```bash
g++ -std=c++17 -Wall -Wextra -Itests/native -Iinclude src/pid_controller.cpp tests/native/pid_stability_smoke.cpp -o pid_stability_smoke
./pid_stability_smoke
```

`README_MAINHIGHSPEED.md` là tài liệu tham khảo cho cấu hình HighSpeed cũ, không mô tả chế độ đang chạy trên nhánh `mainPID`.
