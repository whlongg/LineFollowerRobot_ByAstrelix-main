# Báo cáo vấn đề về cấu trúc chương trình

## Vấn đề quản lý thời gian và scheduling

### Quản lý thời gian không nhất quán

```cpp
// Trong main.ino - hàm loop()
void loop() {
  unsigned long currentTime = millis();
  loopTimeMicros = micros();
  
  // Cập nhật thông tin màu sắc nếu đã đến thời điểm
  if (enableColorReading && (currentTime - previousColorTime >= COLOR_SAMPLE_TIME)) {
    updateColor();
    previousColorTime = currentTime;
  }
  
  // Xử lý PID định kỳ
  if (currentTime - previousPIDTime >= SAMPLE_TIME) {
    if (isRunning) {
      processPID();
    } else {
      motor.stop();
    }
    previousPIDTime = currentTime;
  }
  
  // Kiểm tra nếu có lệnh từ Serial
  if (currentTime - previousSerialCheckTime >= 20) {
    checkSerial();
    previousSerialCheckTime = currentTime;
  }
  
  // Gửi thông tin debug nếu được bật
  if (enableDebug && (currentTime - previousDebugTime >= DEBUG_INTERVAL)) {
    sendDebugInfo();
    previousDebugTime = currentTime;
  }
  
  loopTimeMicros = micros() - loopTimeMicros;
}
```

**Mô tả vấn đề:**
1. Sử dụng nhiều biến thời gian khác nhau để quản lý các hoạt động
2. Thiếu một cơ chế quản lý tác vụ thống nhất (scheduler)
3. Khoảng thời gian kiểm tra Serial cứng cố định (20ms) thay vì sử dụng một hằng số có thể cấu hình
4. Biến `loopTimeMicros` được tính toán không chính xác - trong một số trường hợp, nó có thể không phản ánh đúng thời gian của toàn bộ vòng lặp

**Giải pháp đề xuất:**
- Triển khai một scheduler đơn giản để quản lý tất cả các tác vụ định kỳ
- Tạo các hằng số có thể cấu hình cho tất cả các khoảng thời gian
- Tách biệt phép đo thời gian vòng lặp khỏi xử lý chính

### Chưa tối ưu hóa thứ tự thực hiện các tác vụ

```cpp
// Trích đoạn từ loop()
if (enableColorReading && (currentTime - previousColorTime >= COLOR_SAMPLE_TIME)) {
  updateColor();
  previousColorTime = currentTime;
}

// Xử lý PID định kỳ
if (currentTime - previousPIDTime >= SAMPLE_TIME) {
  if (isRunning) {
    processPID();
  } else {
    motor.stop();
  }
  previousPIDTime = currentTime;
}
```

**Mô tả vấn đề:**
1. Thứ tự thực hiện các tác vụ chưa được tối ưu hóa theo mức độ ưu tiên
2. Không có cơ chế ưu tiên cho các tác vụ quan trọng
3. Các tác vụ có thể ảnh hưởng lẫn nhau - ví dụ, một tác vụ kéo dài có thể làm trễ việc xử lý PID

**Giải pháp đề xuất:**
- Sắp xếp lại thứ tự các tác vụ theo mức độ ưu tiên: điều khiển -> cảm biến -> tương tác -> debug
- Đảm bảo các tác vụ quan trọng không bị ảnh hưởng bởi các tác vụ thứ yếu
- Cân nhắc hạn chế thời gian thực hiện tối đa cho mỗi tác vụ

## Vấn đề xử lý lỗi và logging

### Thiếu cơ chế xử lý lỗi mạnh mẽ

```cpp
// Ví dụ từ main.ino - hàm processColor()
void processColor() {
  uint8_t color;
  bool highSpeed;
  unsigned long timestamp;
  
  if (colorTracker.getResult(&color, &highSpeed, &timestamp)) {
    // ... xử lý màu sắc
  }
}
```

**Mô tả vấn đề:**
1. Thiếu kiểm tra lỗi trong nhiều phần của chương trình
2. Không có cơ chế khôi phục từ lỗi cảm biến
3. Thiếu logging cho các sự kiện quan trọng hoặc lỗi

**Giải pháp đề xuất:**
- Thêm kiểm tra lỗi cho tất cả các hàm quan trọng
- Triển khai cơ chế khôi phục tự động khi phát hiện lỗi
- Xây dựng hệ thống logging có thể cấu hình để theo dõi trạng thái hệ thống

### Thiếu cơ chế chẩn đoán

```cpp
// Trích đoạn từ sendDebugInfo()
void sendDebugInfo() {
  if (enableDebug) {
    // Gửi thông tin qua Serial
    Serial.print("E:");
    Serial.print(rawError);
    // ... gửi thêm thông tin
  }
}
```

**Mô tả vấn đề:**
1. Chỉ sử dụng Serial đơn giản cho debug, khó khăn trong việc phân tích các vấn đề phức tạp
2. Không có khả năng lọc mức độ thông tin debug
3. Không có cơ chế lưu trữ thông tin debug để phân tích sau

**Giải pháp đề xuất:**
- Tạo một hệ thống logging có nhiều cấp độ (INFO, DEBUG, WARNING, ERROR)
- Sử dụng BLE để gửi thông tin debug đến ứng dụng di động chuyên dụng
- Cân nhắc lưu trữ thông tin debug trong bộ nhớ không bay hơi cho phân tích sau
