# Hướng dẫn sử dụng mã nguồn Robot Dò Đường

## Tổng quan về cấu trúc mã nguồn

Mã nguồn robot dò đường được tổ chức theo mô hình hướng đối tượng với các module sau:

1. **RobotController**: Bộ điều khiển chính kết hợp các phương pháp PID và Fuzzy
2. **PIDController**: Bộ điều khiển PID cổ điển 
3. **FuzzyController**: Bộ điều khiển mờ
4. **MotorControl**: Điều khiển động cơ
5. **ReadIR**: Đọc và xử lý tín hiệu từ cảm biến hồng ngoại
6. **ColorTracker**: Theo dõi và nhận diện màu sắc
7. **TurnLogic**: Logic dự đoán và quyết định hướng rẽ
8. **TurnHandler**: Xử lý hành động rẽ
9. **BLEdebug**: Module debug qua Bluetooth

## Luồng thực thi chính (Main Flow)

1. **Thiết lập ban đầu (setup)**:
   - Khởi tạo các module: động cơ, cảm biến hồng ngoại, cảm biến màu
   - Hiệu chỉnh cảm biến màu nếu chưa được hiệu chỉnh
   - Đợi nút được nhấn để bắt đầu

2. **Vòng lặp chính (loop)**:
   - Đọc và xử lý dữ liệu từ cảm biến hồng ngoại
   - Cập nhật trạng thái theo dõi màu sắc
   - Tính toán điều khiển PID
   - Xử lý các lệnh từ Serial và BLE
   - Gửi thông tin debug

## Chi tiết từng module

### 1. RobotController (RobotController.h/cpp)

**Chức năng**: Kết hợp cả bộ điều khiển PID và Fuzzy để tạo ra bộ điều khiển hybrid linh hoạt.

**Các phương thức chính**:

- `RobotController(ControllerType type, float kp, float ki, float kd, float inputMin, float inputMax, float outputMin, float outputMax)`: Khởi tạo bộ điều khiển với các tham số cơ bản.
- `compute(float error, float dt)`: Tính toán đầu ra điều khiển dựa trên error và thời gian delta.
- `setControllerType(ControllerType type)`: Chọn loại bộ điều khiển (PID, Fuzzy, hoặc Hybrid).
- `setPIDTunings(float kp, float ki, float kd)`: Thiết lập các tham số PID.
- `setOutputLimits(float min, float max)`: Giới hạn đầu ra.
- `setFuzzyInputRange(float min, float max)`: Thiết lập phạm vi đầu vào cho Fuzzy.
- `reset()`: Khởi tạo lại bộ điều khiển.
- `setHybridWeights(float pidWeight, float fuzzyWeight)`: Thiết lập trọng số cho mỗi bộ điều khiển trong chế độ hybrid.

**Cách sử dụng**:
```cpp
// Khởi tạo bộ điều khiển với PID
RobotController controller(CONTROLLER_PID, 0.38, 0.0, 0.0, -100, 100, -100, 100);

// Tính toán điều khiển
float error = irSensor.calculateError();
int output = controller.compute(error, 0.005); // dt = 5ms
```

### 2. PIDController (PIDController.h/cpp)

**Chức năng**: Bộ điều khiển PID cổ điển với các tính năng lọc và giới hạn.

**Các phương thức chính**:

- `PIDController(float kp, float ki, float kd)`: Khởi tạo với các hệ số PID.
- `reset()`: Đặt lại trạng thái.
- `compute(float error, float dt)`: Tính toán đầu ra PID từ error.
- `setTunings(float kp, float ki, float kd)`: Thiết lập các hệ số PID.
- `setOutputLimits(float min, float max)`: Thiết lập giới hạn đầu ra.
- `setIntegralLimits(float min, float max)`: Thiết lập giới hạn cho phần tích phân.

**Cách sử dụng**:
```cpp
// Khởi tạo
PIDController pid(0.38, 0.0, 3.8);
pid.setOutputLimits(-100, 100);

// Sử dụng
float error = sensors.getError();
float output = pid.compute(error, 0.01); // dt = 10ms
```

### 3. FuzzyController (FuzzyController.h/cpp)

**Chức năng**: Bộ điều khiển mờ với 7 tập mờ đầu vào và 7 đầu ra.

**Các phương thức chính**:

- `FuzzyController()`: Khởi tạo bộ điều khiển mờ.
- `computeOutput(float error, float deltaError)`: Tính toán đầu ra dựa trên error và thay đổi error.
- `setInputRange(float min, float max)`: Thiết lập phạm vi đầu vào.
- `setOutputRange(float min, float max)`: Thiết lập phạm vi đầu ra.

**Cách sử dụng**:
```cpp
// Khởi tạo
FuzzyController fuzzy;
fuzzy.setInputRange(-100, 100);
fuzzy.setOutputRange(-100, 100);

// Sử dụng
float error = sensors.getError();
float deltaError = error - lastError;
int output = fuzzy.computeOutput(error, deltaError);
```

### 4. MotorControl (MotorControl.h/cpp)

**Chức năng**: Điều khiển động cơ với các chức năng di chuyển cơ bản.

**Các hằng số và biến**:
- `PWM_FREQ`: Tần số PWM (20kHz)
- `PWM_RES`: Độ phân giải PWM (8 bit)
- `SCALE_RIGHT_MOTOR`: Hệ số hiệu chỉnh cho động cơ phải (1.002)

**Các phương thức chính**:

- `setupMotors()`: Khởi tạo các chân PWM cho động cơ.
- `setMotorSpeeds(int left, int right)`: Thiết lập tốc độ cho hai động cơ.
- `turnRight(int speed)`: Rẽ phải với tốc độ cụ thể.
- `turnLeft(int speed)`: Rẽ trái với tốc độ cụ thể.
- `Forward(int speed)`: Di chuyển tiến với tốc độ cụ thể.
- `Back(int speed)`: Di chuyển lùi với tốc độ cụ thể.
- `stop()`: Dừng động cơ.

**Cách sử dụng**:
```cpp
// Khởi tạo
setupMotors();
MotorControl motor;

// Điều khiển
motor.Forward(200); // Tiến với tốc độ 200
motor.turnLeft(150); // Rẽ trái với tốc độ 150
motor.stop(); // Dừng lại
```

### 5. ReadIR (ReadIR.h/cpp)

**Chức năng**: Đọc và xử lý tín hiệu từ cảm biến hồng ngoại để xác định vị trí đường line.

**Các phương thức chính**:

- `begin()`: Khởi tạo cảm biến.
- `readSensors(float *sensorValues)`: Đọc giá trị từ cảm biến và áp dụng bộ lọc.
- `calculateError()`: Tính toán sai số vị trí dựa trên giá trị cảm biến.
- `CalibrateSensor()`: Hiệu chuẩn cảm biến.

**Cách sử dụng**:
```cpp
// Khởi tạo
ReadIR irSensor;
irSensor.begin();

// Đọc giá trị cảm biến
float sensorValues[SENSOR_COUNT];
irSensor.readSensors(sensorValues);

// Tính toán error
float error = irSensor.calculateError();
```

### 6. ColorTracker (ColorTracker.h/cpp)

**Chức năng**: Theo dõi và nhận diện màu sắc từ cảm biến màu.

**Các mã màu**:
- `COLOR_WHITE`: 0
- `COLOR_BLACK`: 1
- `COLOR_BLUE`: 2
- `COLOR_GREEN`: 3

**Các phương thức chính**:

- `begin()`: Khởi tạo cảm biến màu.
- `update()`: Cập nhật giá trị từ cảm biến nếu đã đến thời điểm đọc.
- `getColor()`: Lấy mã màu hiện tại.
- `isHighSpeedMode()`: Kiểm tra xem có phải chế độ tốc độ cao (màu xanh lá).
- `calibrateBlueColor()`: Hiệu chuẩn màu xanh dương.
- `calibrateGreenColor()`: Hiệu chuẩn màu xanh lá.
- `isCalibrated()`: Kiểm tra xem đã hiệu chuẩn chưa.
- `saveCalibrationToEEPROM()`: Lưu thông số hiệu chuẩn vào EEPROM.
- `loadCalibrationFromEEPROM()`: Đọc thông số hiệu chuẩn từ EEPROM.

**Cách sử dụng**:
```cpp
// Khởi tạo
ColorTracker colorTracker(25); // Đọc mỗi 25ms
colorTracker.begin();

// Kiểm tra đã hiệu chuẩn chưa
if (!colorTracker.isCalibrated()) {
    // Hiệu chuẩn
    colorTracker.calibrateBlueColor();
    colorTracker.calibrateGreenColor();
    colorTracker.saveCalibrationToEEPROM();
}

// Cập nhật và đọc màu
if (colorTracker.update()) {
    uint8_t color = colorTracker.getColor();
    bool highSpeed = colorTracker.isHighSpeedMode();
}
```

### 7. TurnLogic (TurnLogic.h/cpp)

**Chức năng**: Logic dự đoán và quyết định hướng rẽ.

**Các phương thức chính**:

- `update(int blue, int green, float error)`: Cập nhật thông tin và xác định hướng rẽ.
- `getNextTurn()`: Lấy hướng rẽ tiếp theo dự đoán.
- `getLastTurn()`: Lấy hướng rẽ vừa xác định.
- `reset()`: Đặt lại trạng thái.

**Cách sử dụng**:
```cpp
// Khởi tạo
TurnLogic turnLogic;

// Cập nhật
bool isBlue = (colorTracker.getColor() == COLOR_BLUE);
bool isGreen = (colorTracker.getColor() == COLOR_GREEN);
float error = irSensor.calculateError();
turnLogic.update(isBlue, isGreen, error);

// Lấy hướng rẽ
TurnDirection direction = turnLogic.getNextTurn();
```

### 8. TurnHandler (TurnHandler.h/cpp)

**Chức năng**: Xử lý hành động rẽ.

**Các phương thức chính**:

- `setTurnSpeed(int speed)`: Thiết lập tốc độ khi rẽ.
- `setBaseSpeed(int speed)`: Thiết lập tốc độ cơ bản.
- `enableCheckpointMode(bool enable)`: Bật/tắt chế độ checkpoint.
- `setDefaultDirection(TurnDirection direction)`: Thiết lập hướng rẽ mặc định.
- `handleTurn(TurnDirection direction)`: Xử lý rẽ theo hướng chỉ định.
- `processCommand(const String& command)`: Xử lý lệnh rẽ từ command.

**Cách sử dụng**:
```cpp
// Khởi tạo
TurnHandler turnHandler(&motor, &turnLogic);
turnHandler.setTurnSpeed(150);
turnHandler.setBaseSpeed(200);
turnHandler.setDefaultDirection(TURN_RIGHT);

// Xử lý rẽ
TurnDirection direction = turnLogic.getNextTurn();
if (direction != TURN_UNKNOWN) {
    turnHandler.handleTurn(direction);
}
```

### 9. BLEdebug (BLEdebug.h/cpp)

**Chức năng**: Module debug qua Bluetooth.

**Các phương thức chính**:

- `begin(const char* deviceName)`: Khởi tạo BLE với tên thiết bị.
- `update()`: Cập nhật trạng thái kết nối.
- `isConnected()`: Kiểm tra xem có thiết bị đang kết nối không.
- `sendData(const String& data)`: Gửi thông tin debug qua BLE.
- `getCommand()`: Kiểm tra và trả về lệnh nhận được qua BLE.

**Cách sử dụng**:
```cpp
// Khởi tạo
bleDebug.begin("LineFollowerRobot");

// Kiểm tra kết nối và cập nhật
bleDebug.update();
if (bleDebug.isConnected()) {
    // Gửi dữ liệu
    bleDebug.sendData("Error: " + String(error));
    
    // Nhận lệnh
    String command = bleDebug.getCommand();
    if (command.length() > 0) {
        processBLECommand(command);
    }
}
```

## Flow chính (Main Flow)

### 1. setup()
- Khởi tạo cổng Serial với tốc độ 115200 baud
- Thiết lập thông số cho TurnHandler (tốc độ rẽ, tốc độ cơ bản, hướng rẽ mặc định)
- Khởi tạo BLE với tên thiết bị
- Thiết lập LED và các chân GPIO
- Khởi tạo động cơ qua hàm setupMotors()
- Khởi tạo cảm biến hồng ngoại qua irSensor.begin()
- Khởi tạo cảm biến màu qua colorTracker.begin()
- Nếu chưa hiệu chuẩn cảm biến màu, tiến hành hiệu chuẩn cho màu xanh dương và xanh lá
- Nếu tất cả khởi tạo thành công, chờ người dùng nhấn nút để bắt đầu chạy

### 2. loop()
- Ghi thời gian bắt đầu chu kỳ
- Xử lý lệnh Serial và BLE
- Nếu chương trình đang chạy (isRunning):
  - Cập nhật thông tin về màu sắc theo chu kỳ COLOR_SAMPLE_TIME
  - Xử lý logic PID theo chu kỳ SAMPLE_TIME
  - Gửi thông tin debug nếu đã đến thời điểm DEBUG_INTERVAL
- Tính toán thời gian xử lý chu kỳ

### 3. updateColor()
- Cập nhật thông tin màu từ cảm biến
- Xử lý màu thông qua hàm processColor()

### 4. processColor()
- Xử lý màu đọc được từ cảm biến
- Nếu phát hiện màu xanh dương (checkpoint) và chưa xử lý trước đó:
  - Bật chế độ xử lý checkpoint trong TurnHandler
  - Hỗ trợ xác định hướng rẽ
- Nếu phát hiện màu xanh lá và chưa xử lý trước đó:
  - Chuyển đổi chế độ tốc độ cao/thấp
- Cập nhật trạng thái cho các lần xử lý tiếp theo

### 5. processPID()
- Đọc giá trị cảm biến và tính toán error
- Lọc error để giảm nhiễu
- Tính toán đầu ra điều khiển từ error và deltaic time
- Áp dụng bộ lọc cho đầu ra để làm mượt chuyển động
- Chuyển đổi đầu ra thành tốc độ động cơ trái/phải
- Áp dụng tốc độ cho động cơ qua setMotorSpeeds()

### 6. checkSerialCommand() và processBLECommand()
- Xử lý các lệnh từ Serial hoặc BLE
- Điều chỉnh tham số như: tốc độ, hệ số PID, tốc độ rẽ, v.v.
- Điều khiển trạng thái chạy/dừng robot
- Thay đổi chế độ điều khiển (PID, Fuzzy, Hybrid)

## Lưu ý quan trọng

1. **Không sử dụng delay()**:
   - Chương trình sử dụng kiểm tra thời gian để thực hiện các tác vụ theo chu kỳ nhất định
   - Không có hoạt động chặn để đảm bảo thời gian phản hồi thấp

2. **Tối ưu hóa bộ nhớ**:
   - Các bộ lọc được sử dụng để làm mượt đầu vào và đầu ra
   - Các cấu trúc dữ liệu được thiết kế để tiết kiệm bộ nhớ

3. **Khả năng mở rộng**:
   - Cấu trúc module hóa cho phép dễ dàng thay đổi thuật toán điều khiển
   - Tham số có thể được điều chỉnh trong thời gian thực qua Serial hoặc BLE
   
4. **Hệ thống sao lưu**:
   - Dữ liệu hiệu chuẩn màu sắc được lưu trong EEPROM
   - Các thiết lập có thể được khôi phục sau khi khởi động lại

5. **Tinh chỉnh hiệu suất**:
   - Tham số PID (KP, KI, KD) cần được điều chỉnh theo đặc tính cụ thể của robot
   - Tốc độ cơ bản và tốc độ rẽ cần được tối ưu cho từng đường đua

## Các thành phần cần chú ý

1. **Điều khiển hybrid (kết hợp PID và Fuzzy)**:
   - Cho phép kết hợp ưu điểm của cả hai phương pháp
   - Trọng số có thể điều chỉnh (PIDW và FUZW)

2. **Nhận diện màu sắc**:
   - Sử dụng không gian màu HSV và Lab để nhận diện chính xác
   - Hỗ trợ hiệu chuẩn cho màu xanh dương (checkpoint) và xanh lá (tốc độ cao)

3. **Xử lý rẽ thông minh**:
   - Dự đoán hướng rẽ dựa trên error và màu sắc
   - Chế độ checkpoint tự động xử lý các giao lộ

4. **Debug qua BLE**:
   - Cho phép giám sát và điều khiển từ xa qua thiết bị Bluetooth
   - Gửi thông tin về error, đầu ra điều khiển, tốc độ động cơ

## Hướng cải tiến

1. **Cải thiện thuật toán dò đường**:
   - Thêm tính năng học tập và tự điều chỉnh tham số
   - Sử dụng mã hóa động cơ để kiểm soát tốc độ chính xác hơn

2. **Mở rộng nhận diện màu**:
   - Thêm nhận diện màu đỏ, vàng để xử lý loại đường đua phức tạp
   - Áp dụng thuật toán thị giác máy tính nâng cao

3. **Cải thiện giao tiếp**:
   - Phát triển giao diện đồ họa trên thiết bị BLE
   - Hỗ trợ điều khiển và giám sát qua Wi-Fi

4. **Tối ưu hóa năng lượng**:
   - Thêm chế độ tiết kiệm năng lượng
   - Theo dõi và báo cáo mức pin
