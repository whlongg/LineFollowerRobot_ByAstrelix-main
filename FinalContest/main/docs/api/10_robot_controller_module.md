# Tài liệu mô đun RobotController

## Tổng quan

Mô đun `RobotController` là thành phần trung tâm của hệ thống điều khiển robot, quản lý tất cả các hệ thống con và điều phối các hoạt động của robot. Module này kết hợp dữ liệu từ tất cả các cảm biến, xử lý logic điều khiển và ra quyết định về chuyển động của robot. RobotController hoạt động như "bộ não" của robot, tích hợp và điều phối các module khác để tạo ra một hệ thống điều khiển hoàn chỉnh, ổn định và hiệu quả.

## Tệp liên quan
- **RobotController.h**: Định nghĩa giao diện của lớp RobotController
- **RobotController.cpp**: Triển khai các phương thức điều khiển và quản lý

## Kiến trúc tổng thể

`RobotController` kết hợp các module sau:
- **ReadIR**: Đọc và xử lý dữ liệu từ cảm biến hồng ngoại
- **ColorTracker**: Phát hiện và phân tích màu sắc
- **PIDController**: Điều khiển PID cơ bản
- **FuzzyController**: Điều khiển fuzzy nâng cao
- **MotorControl**: Điều khiển động cơ
- **TurnLogic**: Phát hiện giao lộ và quyết định hướng rẽ
- **TurnHandler**: Thực hiện các thao tác rẽ
- **BLEdebug**: Giao tiếp và debug qua Bluetooth

## Lớp RobotController

### Constructor

```cpp
RobotController()
```

**Mô tả**: Khởi tạo đối tượng RobotController với các giá trị mặc định.

**Chi tiết triển khai**:
- Khởi tạo các biến trạng thái
- Thiết lập các giá trị mặc định cho tốc độ, thông số, và chế độ

**Ví dụ**:
```cpp
RobotController robot;
```

### begin

```cpp
void begin()
```

**Mô tả**: Khởi tạo tất cả các thành phần con và chuẩn bị robot cho hoạt động.

**Chi tiết triển khai**:
- Khởi tạo các module con theo thứ tự phù hợp
- Thiết lập các kết nối giữa các module
- Cấu hình các thông số ban đầu

**Ví dụ**:
```cpp
void setup() {
  Serial.begin(115200);
  robot.begin();
  
  // Cấu hình thêm nếu cần
  robot.setOperatingMode(MODE_NORMAL);
  robot.setBaseSpeed(180);
}
```

### update

```cpp
void update()
```

**Mô tả**: Cập nhật trạng thái của robot và xử lý điều khiển theo thời gian thực.

**Chi tiết triển khai**:
- Đọc và xử lý dữ liệu từ tất cả các cảm biến
- Cập nhật các trạng thái nội bộ
- Thực hiện tính toán điều khiển
- Cập nhật các đầu ra động cơ

**Ví dụ**:
```cpp
void loop() {
  // Cập nhật và điều khiển robot
  robot.update();
  
  // Kiểm tra trạng thái và hiển thị nếu cần
  if (robot.isDebugEnabled()) {
    robot.printStatus();
  }
  
  // Xử lý các sự kiện bên ngoài
  processSerialCommands();
}
```

### setOperatingMode

```cpp
void setOperatingMode(RobotMode mode)
```

**Mô tả**: Thiết lập chế độ hoạt động cho robot.

**Tham số**:
- `mode`: Chế độ hoạt động (MODE_NORMAL, MODE_CALIBRATION, MODE_DEBUG, MODE_HIGH_SPEED)

**Chi tiết triển khai**:
- Cập nhật biến trạng thái chế độ
- Điều chỉnh các thông số của tất cả các module liên quan để phù hợp với chế độ mới
- Thông báo thay đổi chế độ qua debug

**Ví dụ**:
```cpp
// Chuyển sang chế độ hiệu chuẩn
robot.setOperatingMode(MODE_CALIBRATION);

// Thực hiện hiệu chuẩn
performCalibration();

// Trở lại chế độ bình thường
robot.setOperatingMode(MODE_NORMAL);
```

### setBaseSpeed

```cpp
void setBaseSpeed(int speed)
```

**Mô tả**: Thiết lập tốc độ cơ bản cho robot.

**Tham số**:
- `speed`: Giá trị tốc độ (0-255)

**Chi tiết triển khai**:
- Cập nhật biến tốc độ cơ bản
- Thông báo thay đổi qua debug

**Ví dụ**:
```cpp
// Thiết lập tốc độ thấp cho đường khó
robot.setBaseSpeed(150);

// Sau khi vượt qua đoạn khó, tăng tốc
robot.setBaseSpeed(200);
```

### setControlType

```cpp
void setControlType(ControlType type)
```

**Mô tả**: Chọn loại điều khiển cho robot (PID hoặc Fuzzy).

**Tham số**:
- `type`: Loại điều khiển (CONTROL_PID, CONTROL_FUZZY, CONTROL_HYBRID)

**Chi tiết triển khai**:
- Cập nhật biến trạng thái điều khiển
- Điều chỉnh các thông số liên quan đến điều khiển

**Ví dụ**:
```cpp
// Sử dụng điều khiển PID cho đường đơn giản
robot.setControlType(CONTROL_PID);

// Chuyển sang Fuzzy cho đường phức tạp
robot.setControlType(CONTROL_FUZZY);

// Hoặc chế độ lai kết hợp cả hai
robot.setControlType(CONTROL_HYBRID);
```

### calibrateSensors

```cpp
void calibrateSensors()
```

**Mô tả**: Thực hiện hiệu chuẩn tất cả các cảm biến.

**Chi tiết triển khai**:
- Chuyển sang chế độ hiệu chuẩn
- Hiệu chuẩn cảm biến IR
- Hiệu chuẩn cảm biến màu
- Lưu trữ thông số hiệu chuẩn
- Trở lại chế độ trước đó

**Ví dụ**:
```cpp
// Bắt đầu hiệu chuẩn
robot.calibrateSensors();

// Đợi hoàn thành
while (robot.isCalibrating()) {
  delay(100);
}

Serial.println("Hiệu chuẩn hoàn tất!");
```

### processCommand

```cpp
bool processCommand(const String &command)
```

**Mô tả**: Xử lý lệnh từ giao diện người dùng.

**Tham số**:
- `command`: Chuỗi lệnh cần xử lý

**Giá trị trả về**: `true` nếu lệnh được xử lý thành công, `false` nếu không

**Chi tiết triển khai**:
- Phân tích cú pháp lệnh
- Chuyển lệnh đến module thích hợp hoặc xử lý trực tiếp
- Phản hồi kết quả

**Ví dụ**:
```cpp
// Xử lý lệnh từ Serial
void processSerialCommand() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (robot.processCommand(command)) {
      Serial.println("OK");
    } else {
      Serial.println("Lệnh không hợp lệ");
    }
  }
}
```

### getStatus

```cpp
RobotStatus getStatus()
```

**Mô tả**: Lấy thông tin trạng thái hiện tại của robot.

**Giá trị trả về**: Cấu trúc RobotStatus chứa thông tin trạng thái

**Chi tiết triển khai**:
- Thu thập thông tin từ tất cả các module
- Đóng gói vào cấu trúc RobotStatus

**Ví dụ**:
```cpp
// Lấy và hiển thị trạng thái
RobotStatus status = robot.getStatus();

Serial.print("Tốc độ: ");
Serial.print(status.currentSpeed);
Serial.print(", Error: ");
Serial.print(status.currentError);
Serial.print(", Chế độ: ");
Serial.println(status.currentMode);
```

### enableDebug

```cpp
void enableDebug(bool enable)
```

**Mô tả**: Bật/tắt chế độ debug.

**Tham số**:
- `enable`: `true` để bật debug, `false` để tắt

**Chi tiết triển khai**:
- Cập nhật biến trạng thái debug
- Bật/tắt debug cho các module con

**Ví dụ**:
```cpp
// Bật chế độ debug
robot.enableDebug(true);

// Thực hiện một số thao tác cần debug
performOperations();

// Tắt debug khi không cần
robot.enableDebug(false);
```

## Chi tiết triển khai

### Kiến trúc máy trạng thái

RobotController được triển khai như một máy trạng thái để quản lý các chế độ hoạt động khác nhau của robot:

```cpp
enum RobotState {
  STATE_INIT,           // Khởi tạo
  STATE_CALIBRATION,    // Hiệu chuẩn
  STATE_FOLLOWING_LINE, // Đang theo đường
  STATE_TURNING,        // Đang rẽ
  STATE_STOPPED,        // Đã dừng
  STATE_ERROR           // Lỗi
};

// Cập nhật máy trạng thái
void RobotController::updateStateMachine() {
  switch (currentState) {
    case STATE_INIT:
      // Xử lý khởi tạo
      if (initComplete) {
        currentState = STATE_FOLLOWING_LINE;
      }
      break;
      
    case STATE_CALIBRATION:
      // Xử lý hiệu chuẩn
      if (calibrationComplete) {
        currentState = STATE_FOLLOWING_LINE;
      }
      break;
      
    case STATE_FOLLOWING_LINE:
      // Kiểm tra các điều kiện rẽ
      if (turnLogic->isIntersectionDetected()) {
        currentState = STATE_TURNING;
        TurnDirection dir = turnLogic->getNextTurn();
        turnHandler->startTurn(dir);
      }
      break;
      
    case STATE_TURNING:
      // Kiểm tra hoàn thành rẽ
      if (!turnHandler->isTurning()) {
        currentState = STATE_FOLLOWING_LINE;
      }
      break;
      
    // Xử lý các trạng thái khác...
  }
}
```

### Xử lý điều khiển

Phần xử lý điều khiển chính xác định loại điều khiển (PID hoặc Fuzzy) và tính toán đầu ra tương ứng:

```cpp
// Cập nhật điều khiển
void RobotController::updateControl() {
  // Chỉ xử lý khi đang theo đường
  if (currentState != STATE_FOLLOWING_LINE) return;
  
  // Đọc giá trị cảm biến
  float sensorValues[SENSOR_COUNT];
  irSensor->readSensors(sensorValues);
  
  // Tính toán lỗi
  float error = irSensor->calculateError();
  
  // Thời gian kể từ lần cập nhật cuối
  unsigned long currentTime = millis();
  float dt = (currentTime - lastControlUpdateTime) / 1000.0f;
  lastControlUpdateTime = currentTime;
  
  // Tính toán đầu ra điều khiển
  float output = 0;
  
  switch (controlType) {
    case CONTROL_PID:
      output = pidController->compute(error, dt);
      break;
      
    case CONTROL_FUZZY:
      output = fuzzyController->compute(error, dt);
      break;
      
    case CONTROL_HYBRID:
      // Kết hợp cả PID và Fuzzy
      float pidOutput = pidController->compute(error, dt);
      float fuzzyOutput = fuzzyController->compute(error, dt);
      
      // Tính trọng số dựa trên độ lớn của lỗi
      float errorAbs = abs(error);
      float fuzzyWeight = min(1.0f, errorAbs / 50.0f);  // Tăng trọng số Fuzzy khi lỗi lớn
      
      output = (1.0f - fuzzyWeight) * pidOutput + fuzzyWeight * fuzzyOutput;
      break;
  }
  
  // Điều chỉnh tốc độ động cơ
  int leftSpeed = baseSpeed - output;
  int rightSpeed = baseSpeed + output;
  
  // Kiểm tra giới hạn tốc độ
  const int MIN_SPEED = 0;
  const int MAX_SPEED = 255;
  
  leftSpeed = constrain(leftSpeed, MIN_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);
  
  // Điều khiển động cơ
  motorControl->setSpeed(leftSpeed, rightSpeed);
  
  // Cập nhật trạng thái
  currentLeftSpeed = leftSpeed;
  currentRightSpeed = rightSpeed;
  currentError = error;
  currentOutput = output;
}
```
