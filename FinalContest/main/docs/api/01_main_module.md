# Tài liệu mô đun main.ino

## Tổng quan

File `main.ino` là tệp chính điều khiển toàn bộ luồng hoạt động của robot dò đường. Mô đun này phối hợp các thành phần khác nhau của hệ thống, bao gồm điều khiển động cơ, đọc cảm biến hồng ngoại, phát hiện màu sắc, xử lý rẽ, và giao tiếp BLE.

## Thiết lập cấu hình (Configuration)

### Tham số điều khiển cơ bản
```cpp
#define BASE_SPEED 212           // Tốc độ cơ bản
#define HIGH_SPEED 250           // Tốc độ cao (dùng khi phát hiện checkpoint xanh lá)
#define TURN_SPEED 150           // Tốc độ khi rẽ
#define MODE_CONTROL CONTROLLER_PID // Chế độ điều khiển (PID, Fuzzy, Hybrid)
```

### Tham số Fuzzy
```cpp
#define FUZZY_INPUT_MIN -100.0   // Phạm vi đầu vào tối thiểu
#define FUZZY_INPUT_MAX 100.0    // Phạm vi đầu vào tối đa
#define FUZZY_OUTPUT_MIN -100.0  // Phạm vi đầu ra tối thiểu
#define FUZZY_OUTPUT_MAX 100.0   // Phạm vi đầu ra tối đa
```

### Tham số PID
```cpp
#define KP 0.38     // Hệ số tỉ lệ
#define KI 0.0      // Hệ số tích phân
#define KD 0.0      // Hệ số vi phân
#define PIDW 1.0    // Trọng số PID trong điều khiển hybrid
#define FUZW 0.0    // Trọng số Fuzzy trong điều khiển hybrid
```

### Thời gian và chu kỳ
```cpp
#define SAMPLE_TIME 5          // Chu kỳ lấy mẫu PID (ms)
#define COLOR_SAMPLE_TIME 25   // Chu kỳ đọc màu (ms)
#define DEBUG_INTERVAL 500     // Thời gian giữa các lần gửi debug (ms)
```

### Bộ lọc và các thiết lập khác
```cpp
#define OUTPUT_FILTER_ALPHA 0.6  // Hệ số lọc cho đầu ra điều khiển (0-1)
#define ENABLE_COLOR_DEFAULT 1   // Mặc định bật đọc màu
#define ENABLE_DEBUG true        // Mặc định bật debug
```

## Đối tượng toàn cục

```cpp
RobotController controller;    // Bộ điều khiển chính (PID, Fuzzy, Hybrid)
ReadIR irSensor;               // Đọc cảm biến hồng ngoại
ColorTracker colorTracker;     // Đọc và phân tích màu sắc
MotorControl motor;            // Điều khiển động cơ
TurnLogic turnLogic;           // Logic dự đoán hướng rẽ
TurnHandler turnHandler;       // Xử lý hành động rẽ
```

## Biến toàn cục chính

```cpp
bool isRunning;                // Trạng thái chạy hay dừng của robot
bool highSpeedMode;            // Chế độ tốc độ cao (kích hoạt bởi checkpoint xanh lá)
bool enableColorReading;       // Bật/tắt đọc màu
float currentKp, currentKi, currentKd; // Tham số PID hiện tại
float filteredError;           // Lỗi đã qua bộ lọc
float filteredOutput;          // Đầu ra điều khiển đã qua bộ lọc
```

## Luồng thực thi chính

### setup()

Chức năng: Khởi tạo hệ thống, cấu hình các thành phần, và chuẩn bị robot để chạy.

Các bước chính:
1. Khởi tạo Serial để debug
2. Thiết lập TurnHandler (tốc độ rẽ, tốc độ cơ bản, hướng rẽ mặc định)
3. Khởi tạo BLE
4. Khởi tạo LED và nút nhấn
5. Khởi tạo động cơ
6. Khởi tạo cảm biến hồng ngoại
7. Khởi tạo và hiệu chuẩn cảm biến màu (nếu cần)
8. Thiết lập tham số điều khiển (PID, Fuzzy, hoặc Hybrid)
9. Hiệu chỉnh cảm biến hồng ngoại
10. Đợi nút nhấn để bắt đầu chạy
11. Khởi tạo các biến thời gian và trạng thái khác

Chi tiết hiệu chuẩn cảm biến màu:
- Kiểm tra xem đã có dữ liệu hiệu chuẩn trong EEPROM
- Nếu chưa, hướng dẫn người dùng đặt mẫu màu xanh dương để hiệu chuẩn
- Tiếp theo hiệu chuẩn mẫu màu xanh lá
- Lưu thông số hiệu chuẩn vào EEPROM

### loop()

Chức năng: Vòng lặp chính để xử lý các tác vụ định kỳ theo tần suất khác nhau.

Các tác vụ chính:
1. Cập nhật BLE
2. Kiểm tra lệnh Serial mỗi 100ms
3. Cập nhật thông tin màu sắc mỗi COLOR_SAMPLE_TIME ms
4. Xử lý PID và điều khiển động cơ mỗi SAMPLE_TIME ms
5. Gửi thông tin debug mỗi DEBUG_INTERVAL ms
6. Cập nhật máy trạng thái xử lý rẽ
7. Nhường cho các tác vụ khác thông qua yield()

## Các hàm chức năng chính

### updateColor()

Chức năng: Cập nhật thông tin từ cảm biến màu và xử lý thông qua hàm processColor().

```cpp
void updateColor() {
  if (enableColorReading && colorTracker.update()) {
    processColor();
  }
}
```

### processColor()

Chức năng: Xử lý thông tin màu sắc đọc được từ cảm biến và điều chỉnh hành vi của robot dựa trên màu.

Hành vi:
- Khi phát hiện màu xanh dương: Chuyển sang chế độ tốc độ thường
- Khi phát hiện màu xanh lá: Kích hoạt chế độ tốc độ cao
- Gửi thông tin debug chi tiết về màu sắc phát hiện được

### processPID()

Chức năng: Đọc giá trị cảm biến IR, tính toán điều khiển PID, và điều khiển động cơ.

Các bước chính:
1. Ghi lại thời gian bắt đầu
2. Đọc giá trị cảm biến và tính toán error
3. Lọc error để giảm nhiễu
4. Tính toán đầu ra điều khiển theo thuật toán đã chọn
5. Lọc đầu ra để làm mượt chuyển động
6. Chuyển đổi đầu ra thành tốc độ động cơ trái/phải
7. Điều chỉnh tốc độ theo chế độ (thường hoặc cao)
8. Kiểm tra điều kiện rẽ và xử lý nếu cần
9. Điều khiển động cơ

Lưu ý quan trọng về xử lý rẽ:
```cpp
// Kiểm tra điều kiện rẽ nếu không đang trong quá trình rẽ
if (!turnHandler.isTurning()) {
  TurnDirection nextTurn = turnLogic.getNextTurn();
  if (nextTurn != TURN_UNKNOWN) {
    turnHandler.startTurn(nextTurn);
    turnLogic.reset(); // Reset sau khi đã bắt đầu xử lý
  }
}
```

### checkSerialCommand() và processBLECommand()

Chức năng: Xử lý các lệnh điều khiển nhận được từ Serial hoặc BLE.

Các lệnh được hỗ trợ:
- Tạm dừng/tiếp tục: `p` hoặc `pause` / `r` hoặc `resume`
- Chuyển đổi chế độ tốc độ: `fast` / `normal`
- Bật/tắt phát hiện màu: `rgb on` / `rgb off`
- Thay đổi tham số PID: `pid kp ki kd`
- Thay đổi phạm vi Fuzzy: `fuzzy_range inMin inMax outMin outMax`
- Thay đổi hướng rẽ mặc định: `TL` / `TR`
- Bật/tắt chế độ checkpoint: `checkpoint on` / `checkpoint off`
- Thay đổi tốc độ: `base_speed value` / `turn_speed value`
- Rẽ thủ công: `turn=left` / `turn=right`
- Bật/tắt debug: `debug on` / `debug off`
- Thay đổi bộ lọc: `filter value`

### sendDebugInfo()

Chức năng: Gửi thông tin debug chi tiết thông qua Serial và BLE.

Thông tin được gửi:
- Error (thô và đã lọc)
- Đầu ra điều khiển
- Tốc độ động cơ trái/phải
- Thời gian chu kỳ thực thi
- Trạng thái tốc độ cao
- Tham số PID hiện tại
- Thông tin về checkpoint và hướng rẽ

## Lưu ý quan trọng

1. **Non-blocking Design**: Mã nguồn đã được thiết kế để không chặn, sử dụng kiểm tra thời gian thay vì delay() để đảm bảo phản ứng thời gian thực.

2. **Máy trạng thái xử lý rẽ**: Hệ thống xử lý rẽ hoạt động không chặn thông qua máy trạng thái để robot vẫn có thể phản ứng với môi trường trong khi đang rẽ.

3. **Xử lý song song**: Các tác vụ được chia thành chu kỳ xử lý khác nhau (PID, đọc màu, debug) để tối ưu hiệu suất.

4. **Tối ưu hóa bộ lọc**: Sử dụng các bộ lọc cho error và đầu ra điều khiển để giảm nhiễu và cải thiện ổn định.

5. **Động lực học thích ứng**: Hệ thống thay đổi động lực học (tốc độ) dựa trên màu sắc phát hiện được, tạo mềm dẻo cho việc thích ứng với đường đua.

## Khuyến nghị sử dụng và tùy chỉnh

1. **Điều chỉnh PID**: Thông số KP, KI, KD có ảnh hưởng lớn đến hiệu suất. Điều chỉnh chúng phù hợp với đường đua cụ thể.

2. **Tối ưu hóa tốc độ**: BASE_SPEED và HIGH_SPEED nên được điều chỉnh dựa trên khả năng cơ khí của robot và độ phức tạp của đường đua.

3. **Chu kỳ lấy mẫu**: Giá trị SAMPLE_TIME ảnh hưởng đến tần suất cập nhật PID. Giá trị nhỏ hơn (2-5ms) cho đáp ứng nhanh, giá trị lớn hơn (10-20ms) cho ổn định.

4. **Hệ số lọc**: OUTPUT_FILTER_ALPHA điều chỉnh mức độ làm mịn đầu ra. Giá trị gần 1.0 có ít lọc, gần 0.0 có nhiều lọc hơn.

5. **Thời gian rẽ**: Cân nhắc điều chỉnh thời gian rẽ qua hàm turnHandler.setTurnTiming() nếu robot rẽ quá nhanh hoặc quá chậm.
