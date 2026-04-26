# Báo cáo vấn đề về điện năng và hiệu suất

## Vấn đề quản lý điện năng

### Thiếu cơ chế theo dõi pin

Trong toàn bộ mã nguồn, không có cơ chế nào để theo dõi tình trạng pin (điện áp, mức sạc), điều này có thể dẫn đến hành vi không ổn định khi pin yếu.

**Mô tả vấn đề:**
1. Không có cảm biến điện áp pin
2. Không có cơ chế cảnh báo pin yếu
3. Không điều chỉnh hiệu suất dựa trên mức pin

**Giải pháp đề xuất:**
- Thêm mạch đo điện áp pin kết nối với ADC
- Triển khai cơ chế theo dõi và cảnh báo pin yếu
- Tự động giảm tốc độ khi pin yếu để kéo dài thời gian hoạt động

### Tối ưu hóa điện năng cho các cảm biến

```cpp
// Trong ReadIR.cpp - hàm begin()
void ReadIR::begin()
{
    // Cấu hình các chân LED
    pinMode(W_LED_ON, OUTPUT);
    pinMode(IR_LED_ON, OUTPUT);

    // Luôn bật WLED và IRLED
    digitalWrite(W_LED_ON, HIGH);
    digitalWrite(IR_LED_ON, HIGH);
    
    // ... khởi tạo khác
}
```

**Mô tả vấn đề:**
1. LED hồng ngoại (IR LED) luôn bật, tiêu thụ điện năng không cần thiết
2. Cảm biến màu TCS34725 luôn hoạt động ngay cả khi không cần thiết
3. Không có chế độ tiết kiệm điện khi tạm dừng robot

**Giải pháp đề xuất:**
- Chỉ bật IR LED khi đọc cảm biến, sử dụng PWM hoặc bật/tắt thông minh
- Đặt cảm biến màu vào chế độ sleep khi không sử dụng
- Triển khai chế độ tiết kiệm điện cho toàn hệ thống

## Vấn đề hiệu suất và độ trễ

### Độ trễ trong xử lý dữ liệu cảm biến

```cpp
// Trong ReadIR.cpp - hàm readSensors() và filterSensorValues()
void ReadIR::readSensors(float *sensorValues)
{
    // Đọc giá trị analog từ cảm biến
    uint16_t rawValues[SENSOR_COUNT];
    qtr.read(rawValues);

    // Chuyển đổi sang float và lưu vào mảng
    for (int i = 0; i < SENSOR_COUNT; i++)
        ir_values[i][filter_index] = (float)rawValues[i];

    // Lọc giá trị cảm biến
    filterSensorValues(ir_values, sensorValues);

    // Cập nhật chỉ số bộ lọc
    filter_index = (filter_index + 1) % FILTER_SIZE;
}

void ReadIR::filterSensorValues(float rawValues[][FILTER_SIZE], float *filteredValues)
{
    // Áp dụng bộ lọc trung bình trượt
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        float sum = 0;
        for (int j = 0; j < FILTER_SIZE; j++)
            sum += rawValues[i][j];
        filteredValues[i] = sum / FILTER_SIZE;
    }
}
```

**Mô tả vấn đề:**
1. Chuyển đổi không cần thiết từ uint16_t sang float và ngược lại
2. Bộ lọc trung bình trượt tạo độ trễ 5 chu kỳ trong việc phản ứng với thay đổi
3. Mỗi lần đọc cảm biến đều tính toán lại giá trị trung bình

**Giải pháp đề xuất:**
- Sử dụng bộ lọc hiệu quả hơn như bộ lọc alpha đệ quy (one-pole filter)
- Sử dụng một kiểu dữ liệu nhất quán trong toàn bộ đường dẫn xử lý
- Tối ưu hóa việc tính toán bằng cách chỉ cập nhật phần thay đổi của bộ lọc

### Tối ưu hóa chu kỳ điều khiển

```cpp
// Trong main.ino - các định nghĩa chu kỳ
#define SAMPLE_TIME 10
#define COLOR_SAMPLE_TIME 25
// ... và trong loop()
if (currentTime - previousPIDTime >= SAMPLE_TIME) {
  // ... xử lý PID
}
```

**Mô tả vấn đề:**
1. Chu kỳ điều khiển PID cố định (10ms) không thích ứng với tốc độ hoặc điều kiện đường đi
2. Chu kỳ lấy mẫu màu sắc (25ms) có thể quá chậm ở tốc độ cao
3. Không cân bằng giữa độ trễ phản hồi và tải xử lý

**Giải pháp đề xuất:**
- Điều chỉnh chu kỳ lấy mẫu PID dựa trên tốc độ di chuyển
- Tăng tần suất lấy mẫu màu sắc khi di chuyển nhanh
- Tối ưu hóa chu kỳ để cân bằng giữa tải xử lý và độ trễ phản hồi

## Vấn đề tối ưu hóa bộ nhớ

### Sử dụng bộ nhớ không hiệu quả

Trong dự án hiện tại, có một số điểm sử dụng bộ nhớ không hiệu quả:

1. Khai báo nhiều mảng và buffer tạm thời trong các hàm
2. Tạo nhiều đối tượng cục bộ không cần thiết (như trong hàm `ReadIR::CalibrateSensor()`)
3. Sử dụng biến kiểu float (4 byte) trong nhiều trường hợp chỉ cần độ chính xác của uint16_t (2 byte)

**Giải pháp đề xuất:**
- Sử dụng mảng tĩnh toàn cục cho các buffer thường xuyên sử dụng
- Giảm phạm vi của các biến xuống mức tối thiểu cần thiết
- Xem xét sử dụng kiểu dữ liệu nhỏ hơn khi có thể (int8_t, uint16_t thay cho float)
