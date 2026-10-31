# Ghi log theo 3 map — Python SPP debug (mainv4)

## Mục tiêu

Một robot chạy trên ba bố cục đường line khác nhau. Mỗi lần START/STOP tạo một CSV riêng, được phân loại theo map đã chọn trên giao diện `robot_debug.py`:

```text
doline17s/
├── robot_debug.py
├── spp_csv_recorder.py
├── run_csv_lifecycle.py
└── logs/
    ├── map_1/
    │   ├── linerobot_map1_20261031_103000_123456.csv
    │   └── ...
    ├── map_2/
    │   └── linerobot_map2_....csv
    └── map_3/
        └── linerobot_map3_....csv
```

Các thư mục được tạo khi chọn map / bắt đầu ghi; dữ liệu cũ trong `logs/` **không được tự phân loại** vì không đủ bằng chứng để gán map.

## Vận hành

1. Chạy `python robot_debug.py`; trên thanh ngang **CHỌN MAP LƯU LOG** bấm **MAP 1**, **MAP 2** hoặc **MAP 3**. Nhãn cyan hiển thị map hiện tại và thư mục đích.
2. Kết nối COM Bluetooth Classic SPP và đợi nhận đủ 16 giá trị calib NVS từ `CAL_DUMP`.
3. Bấm **BẮT ĐẦU DÒ LINE** trên app, hoặc bấm nút GPIO34 trên ESP32 sau khi hoàn thành bước calib. App bắt đầu ghi CSV khi nhận `BUTTON34: AUTO_STARTED` hoặc xác nhận AUTO trên SPP. Với nút START trên app, CSV được mở **trước khi** gửi lệnh AUTO.
4. Nhấn **DỪNG** / GPIO34 STOP. Mỗi lượt tạo một CSV mới, kể cả các lượt chạy liên tiếp trên cùng map.
5. Chỉ được chuyển map khi AUTO đã dừng và CSV đã ngừng ghi. Ba nút map bị khóa trong khi chạy. Lần chạy mới dùng map vừa chọn.

**Lưu ý:** Để ghi đầy đủ từ lúc xuất phát bằng nút cứng, phải chọn map và tải xong `CAL_DUMP` *trước* khi bấm GPIO34. Nếu app nhận thông báo AUTO muộn, một số frame đầu có thể không có trong CSV.

## Quy tắc lưu / xuất file

- **CSV tự động:** luôn lưu dưới `logs/map_1/`, `logs/map_2/` hoặc `logs/map_3/`. Tên bao gồm `mapN` và timestamp microsecond, không ghi đè lần chạy cũ.
- **CSV manual export:** hộp thoại lưu mở sẵn tại thư mục thuộc **map của bản CSV cuối cùng**, kể cả khi đã chọn map khác. Mặc định không trộn dữ liệu của nhiều map vào một file. Người dùng có thể tự chọn vị trí khác qua hộp thoại.
- **TXT manual export:** xuất lịch sử terminal riêng của map **đang chọn**, mặc định vào thư mục tương ứng. Chỉ các sự kiện được nhận / ghi khi map đó được chọn mới nằm trong TXT đó.
- **Định dạng CSV:** giữ nguyên `CSV_HEADER`, hàng `CAL_THRESHOLDS`, 16 RAW, 16 CAL, 16 logic, encoder hai bánh, PWM, PID, và mốc START/END. Không thay đổi ESP32 firmware hay giao thức frame 72 byte.
- **Không có map mặc định**: phải bấm một trong ba nút trước START để tránh gán sai một lượt chạy sang map khác.

## Kiểm thử

```powershell
python -m unittest discover -s tests -p "test_map_logging.py" -v
```

Kiểm thử không cần Bluetooth/ESP32: kiểm tra ID map hợp lệ, đường dẫn tách biệt, nhiều lượt chạy, cấu trúc cột CSV, ngưỡng calibration và mốc START.

Chạy ứng dụng:

```powershell
python -m pip install pyserial
python robot_debug.py
```
