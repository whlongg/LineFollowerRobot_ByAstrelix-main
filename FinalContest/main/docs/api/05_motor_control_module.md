# Tài liệu mô đun MotorControl

## Tổng quan

Mô đun `MotorControl` cung cấp các chức năng điều khiển động cơ DC của robot dò đường. Mô đun này quản lý việc điều khiển tốc độ và hướng của hai động cơ thông qua các tín hiệu PWM (Pulse Width Modulation).

## Tệp liên quan
- **MotorControl.h**: Định nghĩa giao diện của lớp MotorControl và các hằng số
- **MotorControl.cpp**: Triển khai các phương thức điều khiển động cơ

## Cấu hình PWM

Mô đun này sử dụng PWM để điều khiển tốc độ động cơ với các thông số cấu hình sau:

```cpp
#define PWM_FREQ 20000     // Tần số PWM (20kHz)
#define PWM_RES 8          // Độ phân giải PWM (8 bit = 0-255)
#define SCALE_RIGHT_MOTOR 1.002  // Hệ số hiệu chỉnh cho động cơ phải
```

## Chân GPIO và kênh PWM

Mô đun định nghĩa các chân GPIO và kênh PWM cho điều khiển động cơ:

```cpp
// Chân PWM
extern const int PWM_PIN_L_A;  // Chân A động cơ trái
extern const int PWM_PIN_L_B;  // Chân B động cơ trái
extern const int PWM_PIN_R_A;  // Chân A động cơ phải
extern const int PWM_PIN_R_B;  // Chân B động cơ phải

// Kênh PWM
extern const int left_motor_channel_a;   // Kênh A động cơ trái
extern const int left_motor_channel_b;   // Kênh B động cơ trái
extern const int right_motor_channel_a;  // Kênh A động cơ phải
extern const int right_motor_channel_b;  // Kênh B động cơ phải
```

## Hàm thiết lập động cơ

```cpp
void setupMotors()
```

**Mô tả**: Thiết lập cấu hình PWM cho các chân điều khiển động cơ.

**Chi tiết triển khai**:
- Khởi tạo tần số PWM và độ phân giải
- Cấu hình các chân GPIO để điều khiển động cơ
- Gán các kênh PWM cho các chân

**Ví dụ**:
```cpp
// Trong hàm setup()
setupMotors();
```

## Hàm điều khiển tốc độ

```cpp
void setMotorSpeeds(int left, int right)
```

**Mô tả**: Điều khiển tốc độ và hướng của cả hai động cơ.

**Tham số**:
- `left`: Tốc độ động cơ trái (-255 đến 255)
- `right`: Tốc độ động cơ phải (-255 đến 255)

**Chi tiết triển khai**:
- Giá trị dương: Động cơ quay tiến
- Giá trị âm: Động cơ quay lùi
- Giá trị 0: Động cơ dừng
- Áp dụng hệ số hiệu chỉnh SCALE_RIGHT_MOTOR cho động cơ phải

**Ví dụ**:
```cpp
// Điều khiển robot đi thẳng với tốc độ 200
setMotorSpeeds(200, 200);

// Điều khiển robot quay trái tại chỗ
setMotorSpeeds(-150, 150);
```

## Lớp MotorControl

Lớp `MotorControl` cung cấp các phương thức với mức trừu tượng cao hơn để điều khiển robot.

### Phương thức turnRight

```cpp
void turnRight(int speed)
```

**Mô tả**: Điều khiển robot rẽ phải.

**Tham số**:
- `speed`: Tốc độ rẽ (0-255)

**Chi tiết triển khai**:
- Điều chỉnh tốc độ tương đối giữa hai động cơ để tạo chuyển động rẽ phải
- Động cơ trái quay nhanh hơn, động cơ phải quay chậm hơn hoặc ngược hướng

**Ví dụ**:
```cpp
MotorControl motor;
motor.turnRight(150);
```

### Phương thức turnLeft

```cpp
void turnLeft(int speed)
```

**Mô tả**: Điều khiển robot rẽ trái.

**Tham số**:
- `speed`: Tốc độ rẽ (0-255)

**Chi tiết triển khai**:
- Điều chỉnh tốc độ tương đối giữa hai động cơ để tạo chuyển động rẽ trái
- Động cơ phải quay nhanh hơn, động cơ trái quay chậm hơn hoặc ngược hướng

**Ví dụ**:
```cpp
MotorControl motor;
motor.turnLeft(150);
```

### Phương thức Forward

```cpp
void Forward(int speed)
```

**Mô tả**: Điều khiển robot di chuyển tiến.

**Tham số**:
- `speed`: Tốc độ tiến (0-255)

**Chi tiết triển khai**:
- Cả hai động cơ đều quay tiến với cùng tốc độ
- Áp dụng hệ số hiệu chỉnh cho động cơ phải

**Ví dụ**:
```cpp
MotorControl motor;
motor.Forward(200);
```

### Phương thức Back

```cpp
void Back(int speed)
```

**Mô tả**: Điều khiển robot di chuyển lùi.

**Tham số**:
- `speed`: Tốc độ lùi (0-255)

**Chi tiết triển khai**:
- Cả hai động cơ đều quay lùi với cùng tốc độ
- Áp dụng hệ số hiệu chỉnh cho động cơ phải

**Ví dụ**:
```cpp
MotorControl motor;
motor.Back(150);
```

### Phương thức stop

```cpp
void stop()
```

**Mô tả**: Dừng cả hai động cơ.

**Chi tiết triển khai**:
- Đặt tốc độ cả hai động cơ về 0

**Ví dụ**:
```cpp
MotorControl motor;
motor.stop();
```

## Cách triển khai chi tiết trong MotorControl.cpp

```cpp
// Điều khiển động cơ trái
void controlLeftMotor(int speed) {
  if (speed >= 0) {
    // Quay tiến
    ledcWrite(left_motor_channel_a, speed);
    ledcWrite(left_motor_channel_b, 0);
  } else {
    // Quay lùi
    ledcWrite(left_motor_channel_a, 0);
    ledcWrite(left_motor_channel_b, -speed);
  }
}

// Điều khiển động cơ phải
void controlRightMotor(int speed) {
  // Áp dụng hệ số hiệu chỉnh
  speed = round(speed * SCALE_RIGHT_MOTOR);
  
  if (speed >= 0) {
    // Quay tiến
    ledcWrite(right_motor_channel_a, speed);
    ledcWrite(right_motor_channel_b, 0);
  } else {
    // Quay lùi
    ledcWrite(right_motor_channel_a, 0);
    ledcWrite(right_motor_channel_b, -speed);
  }
}

// Điều khiển cả hai động cơ
void setMotorSpeeds(int left, int right) {
  // Giới hạn tốc độ trong khoảng -255 đến 255
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);
  
  // Điều khiển từng động cơ
  controlLeftMotor(left);
  controlRightMotor(right);
}
```

## Cách sử dụng trong dự án

### Khởi tạo

```cpp
// Trong hàm setup()
setupMotors();
MotorControl motor;
```

### Điều khiển cơ bản

```cpp
// Điều khiển robot di chuyển tiến
motor.Forward(BASE_SPEED);

// Điều khiển robot rẽ trái
motor.turnLeft(TURN_SPEED);

// Dừng robot
motor.stop();
```

### Điều khiển từ đầu ra PID

```cpp
// Trong hàm processPID()
float output = controller.compute(error, dt);
int leftSpeed = baseSpeed + output;
int rightSpeed = baseSpeed - output;
setMotorSpeeds(leftSpeed, rightSpeed);
```

## Lưu ý khi sử dụng

1. **Hiệu chỉnh động cơ**:
   - Hệ số SCALE_RIGHT_MOTOR giúp cân bằng sự khác biệt giữa hai động cơ
   - Có thể cần điều chỉnh hệ số này dựa trên động cơ cụ thể của robot

2. **Giới hạn tốc độ**:
   - Luôn giới hạn tốc độ trong khoảng -255 đến 255
   - Nên tránh thay đổi tốc độ đột ngột để giảm áp lực lên động cơ và pin

3. **Ổn định nguồn điện**:
   - Điều khiển PWM ở tần số cao (20kHz) giúp giảm tiếng ồn và tăng hiệu suất
   - Cần đảm bảo nguồn điện ổn định để động cơ hoạt động đều đặn

4. **Điều khiển mượt mà**:
   - Khi tốc độ robot cao, nên áp dụng thêm bộ lọc đầu ra để tránh thay đổi đột ngột
   - Ví dụ: `filteredOutput = alpha * newOutput + (1-alpha) * previousOutput`

## Nâng cấp khả năng

1. **Bổ sung mã hóa xung (encoder)**:
   - Thêm mã hóa xung cho các động cơ để đo tốc độ chính xác
   - Triển khai điều khiển tốc độ vòng kín thay vì điều khiển PWM vòng hở

2. **Điều khiển PID cho động cơ**:
   - Triển khai bộ điều khiển PID riêng cho từng động cơ
   - Giúp duy trì tốc độ ổn định ngay cả khi điều kiện thay đổi (pin yếu, ma sát khác nhau)

3. **Chế độ tiết kiệm năng lượng**:
   - Thêm chế độ tiết kiệm năng lượng khi robot không di chuyển
   - Giảm tín hiệu PWM hoặc tắt động cơ khi không sử dụng

4. **Chức năng phanh điện động**:
   - Thêm chức năng phanh điện động để dừng nhanh
   - Điều này có thể thực hiện bằng cách đặt cả hai đầu ra của động cơ ở mức HIGH

## Ví dụ triển khai nâng cao

### Điều khiển với bộ lọc mượt mà

```cpp
float filterAlpha = 0.7; // Hệ số lọc (0-1)
float filteredLeftSpeed = 0;
float filteredRightSpeed = 0;

void setSmoothMotorSpeeds(int targetLeft, int targetRight) {
  // Áp dụng bộ lọc
  filteredLeftSpeed = filterAlpha * targetLeft + (1-filterAlpha) * filteredLeftSpeed;
  filteredRightSpeed = filterAlpha * targetRight + (1-filterAlpha) * filteredRightSpeed;
  
  // Điều khiển động cơ với tốc độ đã lọc
  setMotorSpeeds(round(filteredLeftSpeed), round(filteredRightSpeed));
}
```

### Quay với góc nhất định

```cpp
void turnWithAngle(int speed, float angle) {
  // Tính thời gian quay dựa trên góc và tốc độ
  // (cần hiệu chỉnh dựa trên robot cụ thể)
  float turnRate = 0.5; // đơn vị: độ/ms ở tốc độ chuẩn
  float turnTime = abs(angle) / (turnRate * speed / 100.0);
  
  // Lưu thời điểm bắt đầu
  unsigned long startTime = millis();
  
  // Quay theo hướng phù hợp
  if (angle > 0) {
    // Quay phải
    motor.turnRight(speed);
  } else {
    // Quay trái
    motor.turnLeft(speed);
  }
  
  // Không sử dụng delay(), thay vào đó kiểm tra thời gian
  while (millis() - startTime < turnTime) {
    // Có thể thực hiện các tác vụ khác ở đây
    yield();
  }
  
  // Dừng quay
  motor.stop();
}
```
