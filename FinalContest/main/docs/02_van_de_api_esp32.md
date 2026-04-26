# Báo cáo vấn đề về API ESP32

## Vấn đề API ESP32 Version 3.0

Dự án hiện đang sử dụng các API của ESP32 không tương thích với phiên bản 3.0, điều này có thể dẫn đến lỗi biên dịch và thực thi.

### Sự không nhất quán trong việc sử dụng delay()

```cpp
// Trong main.ino - hàm setup()
delay(50); // Chờ Serial ổn định

// Trong ReadIR.cpp - hàm CalibrateSensor()
// Không sử dụng delay() mà dùng phương pháp đo thời gian
currentTime = millis();
```

**Mô tả vấn đề:** Dự án đang có sự không nhất quán trong cách xử lý thời gian. Một số nơi sử dụng `delay()` (hàm chặn) trong khi mục tiêu là xây dựng hệ thống non-blocking.

**Giải pháp đề xuất:**
- Loại bỏ tất cả các lệnh `delay()` và thay thế bằng cơ chế đo thời gian sử dụng `millis()`
- Chuẩn hóa phương pháp xử lý thời gian trong toàn bộ dự án

## Các vấn đề khác về API

### Sử dụng API không phù hợp cho màn hình debug

```cpp
// Trong hàm sendDebugInfo() (main.ino)
Serial.print("E:");
Serial.print(rawError);
// ... và các lệnh debug khác
```

**Mô tả vấn đề:** Việc sử dụng Serial để debug có thể gây ra độ trễ trong hệ thống thời gian thực, đặc biệt khi kết nối Serial không ổn định hoặc khi có nhiều thông tin cần in ra.

**Giải pháp đề xuất:**
- Sử dụng BLEdebug một cách nhất quán thay vì Serial khi có thể
- Giảm tần suất gửi thông tin debug trong các vòng lặp quan trọng về thời gian
- Xem xét sử dụng bộ đệm cho debug để giảm thời gian chờ I/O
