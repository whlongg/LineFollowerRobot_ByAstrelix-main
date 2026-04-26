# Báo cáo vấn đề về cảm biến

## Vấn đề với cảm biến hồng ngoại (IR)

### Hiệu chuẩn cảm biến IR không hiệu quả

```cpp
// Trong ReadIR.cpp - hàm CalibrateSensor()
void ReadIR::CalibrateSensor() {
    unsigned long startTime = millis(), currentTime = startTime;
    float sensorValues[SENSOR_COUNT];
    uint16_t rawValues[SENSOR_COUNT];
    MotorControl motor;

    while (currentTime - startTime < duration) {
        readSensors(sensorValues);
        qtr.calibrate();
        for (int i = 0; i < SENSOR_COUNT; i++) {
            rawValues[i] = (uint16_t)sensorValues[i];
        }
        if (qtr.readLineBlack(rawValues) > 1500)
            motor.turnRight(SPEED_CALIBRATE);
        else
            motor.turnLeft(SPEED_CALIBRATE);
        currentTime = millis();
    }
    
    // Dừng động cơ sau khi hoàn tất hiệu chỉnh
    motor.stop();
}
```

**Mô tả vấn đề:**
1. Phương pháp hiệu chuẩn không thông minh, robot chỉ quay trái/phải dựa vào giá trị cảm biến
2. Không lưu trữ kết quả hiệu chuẩn
3. Tạo đối tượng `MotorControl motor` cục bộ trong hàm thay vì sử dụng đối tượng toàn cục
4. Không có cơ chế để hiệu chuẩn khi có thay đổi đột ngột về điều kiện ánh sáng

**Giải pháp đề xuất:**
- Cải thiện thuật toán hiệu chuẩn để quét toàn bộ phạm vi giá trị cảm biến
- Lưu trữ và tải giá trị hiệu chuẩn từ bộ nhớ không bay hơi (EEPROM/Flash)
- Sử dụng đối tượng `motor` toàn cục đã được khai báo
- Cân nhắc chế độ hiệu chuẩn tự động trong quá trình chạy

### Vấn đề lọc và nhiễu cảm biến

```cpp
// Trong ReadIR.cpp - hàm filterSensorValues()
void ReadIR::filterSensorValues(float rawValues[][FILTER_SIZE], float *filteredValues) {
    // Áp dụng bộ lọc trung bình trượt
    for (int i = 0; i < SENSOR_COUNT; i++) {
        float sum = 0;
        for (int j = 0; j < FILTER_SIZE; j++)
            sum += rawValues[i][j];
        filteredValues[i] = sum / FILTER_SIZE;
    }
}
```

**Mô tả vấn đề:**
1. Sử dụng bộ lọc trung bình đơn giản, dễ bị ảnh hưởng bởi nhiễu bất thường
2. Kích thước mảng bộ đệm (FILTER_SIZE = 5) có thể gây độ trễ quá lớn khi tốc độ cao
3. Các giá trị bất thường không được xử lý đặc biệt

**Giải pháp đề xuất:**
- Cân nhắc sử dụng bộ lọc trung vị hoặc bộ lọc Kalman đơn giản để loại bỏ nhiễu tốt hơn
- Điều chỉnh FILTER_SIZE tự động dựa trên tốc độ robot
- Thêm xử lý giá trị bất thường (outlier rejection)

## Vấn đề với cảm biến màu sắc

### Xử lý lỗi khi không có cảm biến màu

```cpp
// Trong main.ino - hàm setup()
if (!colorTracker.begin()) {
    Serial.println("CẢNH BÁO: Không thể khởi tạo cảm biến màu!");
    digitalWrite(LED_PIN, HIGH); // Bật LED để báo lỗi
    
    // Nhấp nháy LED để báo lỗi, nhưng không treo hệ thống
    for (int i = 0; i < 10; i++) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(200);
      digitalWrite(LED_PIN, LOW);
      delay(200);
    }
    
    // Tiếp tục với các cảm biến khác đã hoạt động
    Serial.println("Tiếp tục mà không có cảm biến màu");
}
```

**Mô tả vấn đề:**
1. Sử dụng `delay()` trong vòng lặp báo lỗi, điều này chặn hệ thống trong 4 giây
2. Biến `enableColorReading` không được cập nhật khi cảm biến không khởi tạo được
3. Hậu quả: Mã vẫn cố gắng đọc cảm biến màu trong quá trình chạy dù không khởi tạo thành công

**Giải pháp đề xuất:**
- Xóa bỏ `delay()` trong vòng báo lỗi, sử dụng LED nhấp nháy không chặn
- Cập nhật biến `enableColorReading = false` khi cảm biến màu không khởi tạo được
- Kiểm tra trạng thái cảm biến trước khi đọc giá trị trong hàm `processColor()`

### Vấn đề phân loại màu

```cpp
// Trong ColorTracker.cpp (giả định dựa trên interface trong ColorTracker.h)
void ColorTracker::classifyColor() {
    // ... code phân loại màu
}
```

**Mô tả vấn đề:**
1. Bộ phân loại màu sắc hiện tại phụ thuộc vào ngưỡng cố định (`minWhiteThreshold`, `maxBlackThreshold`, v.v.)
2. Không thích ứng với sự thay đổi điều kiện ánh sáng môi trường
3. Không sử dụng bộ lọc để giảm nhiễu trong các phép đo màu sắc

**Giải pháp đề xuất:**
- Xây dựng cơ chế hiệu chuẩn tự động cho các ngưỡng màu sắc
- Thêm bộ lọc trung vị hoặc trung bình trượt cho các phép đo màu sắc
- Cân nhắc sử dụng phương pháp nhận dạng màu sắc tiên tiến hơn (ví dụ: mô hình màu HSV)
- Lưu trữ mẫu màu sắc để cải thiện độ chính xác nhận dạng theo thời gian
