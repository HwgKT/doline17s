# Đánh dấu phân đoạn khó trong CSV (mainv4)

Dashboard hiện tại là `robot_debug.py` qua Bluetooth Classic SPP, không phải `index.html` Web Bluetooth cũ.

## Thao tác
1. Kết nối cổng COM SPP trên laptop; đảm bảo đã CAL_WHITE/CAL_BLACK và có 16 ngưỡng.
2. Nhấn **BẮT ĐẦU DÒ LINE** để mở CSV và chạy robot.
3. Ở đầu đoạn cua/đoạn khó: nhấn **MỐC START [1]** hoặc phím **1** khi cửa sổ dashboard đang focus.
4. Ở cuối đoạn: nhấn **MỐC END [0]** hoặc phím **0**.
5. Có thể lặp lại nhiều cặp START/END trong cùng một lượt chạy. Kết thúc lượt bằng nút **DỪNG** (khác MỐC END).

Phím tắt không kích hoạt khi đang nhập trong ô text hoặc combobox. Không cần chuột nếu cửa sổ dashboard đang nhận focus. Sự kiện không tác động vào PID hoặc tốc độ động cơ.

## Xác nhận từ ESP32
Laptop gửi `MARK_START:<id>` hoặc `MARK_END:<id>`. ESP32 chụp `millis()`, signed encoder L/R và trả về `MARK_ACK:<START|END>:<id>:<t_ms>:<encL>:<encR>`.

Chỉ sau khi ACK đúng ID, dashboard mới ghi dòng sự kiện vào file CSV đang thu thập. LED onboard **GPIO2** thường sáng liên tục khi SPP kết nối và **tắt 150 ms rồi sáng trở lại** để xác nhận lệnh (nháy tối một lần). Màu LED thực tế tùy board, không nhất thiết xanh lục. ACK thể hiện firmware đã xử lý lệnh; nó không chứng minh đèn thực sự sáng hoặc dữ liệu đã được flush ra đĩa.

## Định dạng CSV
Thêm hai cột cuối: `event_type`, `event_id`. Dòng telemetry bình thường để trống. Dòng mốc có `host_iso`, `t_ms` của ESP32, `encL`, `encR`, `event_type` START/END và `event_id`; các cột cảm biến/PID khác để trống vì mốc không phải một mẫu telemetry.

Để phân tích, tìm các dòng START/END theo thứ tự `t_ms`, dùng `t_ms` của ESP32 để lọc mẫu telemetry trong đoạn, không dùng vị trí dòng CSV vì phản hồi SPP có độ trễ và hai hàng đợi độc lập. Mốc lưu thời gian và quãng xung encoder, **không lưu tọa độ XY của map**. Muốn hiện mốc trực tiếp lên bản đồ cần thêm map/odometry và phép đồng bộ tọa độ. 

## Chưa nghiệm thu trên phần cứng
Thay đổi đã commit nhưng chưa build ESP32 hay test Bluetooth và LED thực tế. Trước chạy thử kiểm tra `python -m py_compile robot_debug.py spp_csv_recorder.py run_csv_lifecycle.py` và `pio run -e esp32dev` (nếu môi trường PlatformIO tương ứng). Kiểm tra số dòng marker, timestamp, encoder và phản hồi LED.

## Nhiều phân đoạn trong một lượt
Có thể đánh dấu START(đoạn 1), END(đoạn 1), START(đoạn 2), END(đoạn 2), ... không giới hạn số cặp theo cấu trúc file. Mỗi cặp được gán `segment_id` từ 1, còn `event_id` tăng cho từng lần bấm và không reset khi mở lượt CSV mới trong cùng phiên app.

CSV thêm `segment_id`, `event_status` sau hai cột `event_type`, `event_id`. Giá trị `event_status=ACK` xác nhận ESP32 đã phản hồi; `TX_FAILED`, `ACK_TIMEOUT`, `RUN_CLOSED` thể hiện mốc không được xác nhận, không được dùng làm điểm cắt chính xác khi đánh giá PID. Dòng telemetry để trống cả bốn cột event.

Lưu ý: ESP32 và ứng dụng phải dùng firmware/app mới cùng lúc. Nếu không có ACK trong 2 giây, app ghi mốc lỗi vào CSV và cho phép thử lại; việc retry tạo ID mới. Mốc lỗi không có timestamp ESP32/encoder. Sau khi dừng lượt, mọi ACK còn treo được ghi `RUN_CLOSED` trước khi đóng CSV.
