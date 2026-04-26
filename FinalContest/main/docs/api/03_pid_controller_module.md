# Tài liệu mô đun PIDController

## Tổng quan

Mô đun `PIDController` cung cấp bộ điều khiển PID (Proportional-Integral-Derivative) cổ điển, được sử dụng để điều khiển robot dò đường. Bộ điều khiển PID là thuật toán điều khiển phản hồi phổ biến nhất trong các hệ thống tự động và robot.

## Tệp liên quan
- **PIDController.h**: Định nghĩa giao diện (interface) của lớp PIDController
- **PIDController.cpp**: Triển khai (implementation) các phương thức

## Nguyên lý hoạt động

Bộ điều khiển PID tính toán đầu ra điều khiển dựa trên ba thành phần:

1. **P (Proportional - Tỉ lệ)**: Phản ứng tỉ lệ với error hiện tại
2. **I (Integral - Tích phân)**: Phản ứng dựa trên tích lũy error theo thời gian
3. **D (Derivative - Vi phân)**: Phản ứng dựa trên tốc độ thay đổi của error

Công thức cơ bản:
```
output = Kp * error + Ki * ∫error dt + Kd * d(error)/dt
```

## Thuộc tính (Properties)

| Thuộc tính | Kiểu | Mô tả |
|------------|------|-------|
| `kp` | float | Hệ số tỉ lệ |
| `ki` | float | Hệ số tích phân |
| `kd` | float | Hệ số vi phân |
| `lastError` | float | Giá trị error lần trước |
| `integral` | float | Giá trị tích phân tích lũy |
| `lastDerivative` | float | Giá trị đạo hàm trước đó |
| `outputMin` | float | Giới hạn đầu ra nhỏ nhất |
| `outputMax` | float | Giới hạn đầu ra lớn nhất |
| `integralMin` | float | Giới hạn tích phân nhỏ nhất |
| `integralMax` | float | Giới hạn tích phân lớn nhất |
| `isFirstCompute` | bool | Đánh dấu lần tính toán đầu tiên |
| `alpha` | const float | Hệ số lọc cho đạo hàm (0.7) |

## Phương thức (Methods)

### Constructor

```cpp
PIDController(float kp = 1.0f, float ki = 0.0f, float kd = 0.0f)
```

**Mô tả**: Khởi tạo bộ điều khiển PID với các hệ số được chỉ định.

**Tham số**:
- `kp`: Hệ số tỉ lệ (mặc định: 1.0)
- `ki`: Hệ số tích phân (mặc định: 0.0)
- `kd`: Hệ số vi phân (mặc định: 0.0)

**Ví dụ**:
```cpp
// Tạo bộ điều khiển PID với Kp=0.38, Ki=0.0, Kd=3.8
PIDController pid(0.38, 0.0, 3.8);
```

### reset

```cpp
void reset()
```

**Mô tả**: Đặt lại trạng thái của bộ điều khiển, xóa giá trị tích phân và lịch sử error.

**Ví dụ**:
```cpp
// Reset bộ điều khiển khi phát hiện thay đổi lớn trong điều kiện
if (abs(error) > 50) {
  pid.reset();
}
```

### compute

```cpp
float compute(float error, float dt = 1.0f)
```

**Mô tả**: Tính toán đầu ra PID dựa trên error hiện tại và delta time.

**Tham số**:
- `error`: Giá trị error hiện tại (sai số giữa setpoint và giá trị thực)
- `dt`: Khoảng thời gian kể từ lần gọi compute trước (tính bằng giây, mặc định: 1.0)

**Giá trị trả về**: Giá trị đầu ra điều khiển đã qua xử lý

**Ví dụ**:
```cpp
float error = targetPosition - currentPosition;
float dt = 0.005; // 5ms
float output = pid.compute(error, dt);
```

### setTunings

```cpp
void setTunings(float kp, float ki, float kd)
```

**Mô tả**: Thiết lập các hệ số PID.

**Tham số**:
- `kp`: Hệ số tỉ lệ mới
- `ki`: Hệ số tích phân mới
- `kd`: Hệ số vi phân mới

**Ví dụ**:
```cpp
// Điều chỉnh thông số PID trong thời gian chạy
pid.setTunings(0.45, 0.01, 2.5);
```

### setOutputLimits

```cpp
void setOutputLimits(float min, float max)
```

**Mô tả**: Thiết lập giới hạn đầu ra của bộ điều khiển.

**Tham số**:
- `min`: Giá trị đầu ra tối thiểu
- `max`: Giá trị đầu ra tối đa

**Ví dụ**:
```cpp
// Giới hạn đầu ra từ -100 đến +100
pid.setOutputLimits(-100, 100);
```

### setIntegralLimits

```cpp
void setIntegralLimits(float min, float max)
```

**Mô tả**: Thiết lập giới hạn cho giá trị tích phân để tránh tích lũy quá mức (integral windup).

**Tham số**:
- `min`: Giá trị tích phân tối thiểu
- `max`: Giá trị tích phân tối đa

**Ví dụ**:
```cpp
// Giới hạn tích phân trong khoảng hợp lý
pid.setIntegralLimits(-50, 50);
```

### Các getter

```cpp
float getKp() const
float getKi() const
float getKd() const
float getLastError() const
float getIntegral() const
```

**Mô tả**: Các phương thức để lấy thông tin về trạng thái hiện tại của bộ điều khiển.

**Giá trị trả về**: Giá trị tương ứng

**Ví dụ**:
```cpp
// Hiển thị thông tin PID
Serial.print("Kp: ");
Serial.println(pid.getKp());
Serial.print("Error: ");
Serial.println(pid.getLastError());
Serial.print("Integral: ");
Serial.println(pid.getIntegral());
```

## Chi tiết triển khai

### Tính toán tỉ lệ (P)
```cpp
float pTerm = kp * error;
```
Phần P tỉ lệ trực tiếp với error. Tăng `kp` làm tăng phản ứng với error.

### Tính toán tích phân (I)
```cpp
integral += ki * error * dt;
integral = constrain(integral, integralMin, integralMax);
```
Phần I tích lũy error theo thời gian, giúp loại bỏ error tĩnh. Giá trị tích phân được giới hạn để tránh tích lũy quá mức.

### Tính toán vi phân (D)
```cpp
float derivative = (error - lastError) / dt;
// Lọc đạo hàm để giảm nhiễu
derivative = alpha * derivative + (1 - alpha) * lastDerivative;
float dTerm = kd * derivative;
```
Phần D phản ứng với tốc độ thay đổi của error, giúp đoán trước và giảm dao động. Đạo hàm được lọc để giảm nhiễu.

### Kết hợp đầu ra
```cpp
float output = pTerm + integral + dTerm;
return constrain(output, outputMin, outputMax);
```
Đầu ra là tổng của ba thành phần, được giới hạn trong phạm vi cho phép.

## Cách sử dụng

### Khởi tạo và thiết lập

```cpp
// Khởi tạo với các tham số phù hợp
PIDController pid(0.38, 0.0, 3.8);

// Thiết lập giới hạn
pid.setOutputLimits(-100, 100);
pid.setIntegralLimits(-50, 50);
```

### Sử dụng trong vòng lặp điều khiển

```cpp
void loop() {
  // Đọc cảm biến và tính error
  float sensorValue = readSensor();
  float setpoint = 0; // Vị trí trung tâm
  float error = setpoint - sensorValue;
  
  // Tính delta time (giây)
  static unsigned long lastTime = 0;
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0f;
  lastTime = now;
  
  // Tính đầu ra PID
  float output = pid.compute(error, dt);
  
  // Sử dụng đầu ra để điều khiển robot
  setMotorSpeeds(baseSpeed + output, baseSpeed - output);
}
```

## Lưu ý điều chỉnh PID

### Hệ số tỉ lệ (Kp)
- **Tăng Kp**: Phản ứng nhanh hơn với error, nhưng có thể gây dao động
- **Giảm Kp**: Phản ứng chậm hơn, ổn định hơn, nhưng error tĩnh lớn hơn
- **Hiệu chỉnh**: Bắt đầu với giá trị thấp và tăng dần cho đến khi robot bắt đầu dao động, sau đó giảm xuống

### Hệ số tích phân (Ki)
- **Tăng Ki**: Loại bỏ error tĩnh tốt hơn, nhưng có thể gây dao động chậm
- **Giảm Ki**: Ít tích lũy error, ổn định hơn, nhưng khó loại bỏ error tĩnh
- **Hiệu chỉnh**: Bắt đầu với giá trị 0, tăng dần để loại bỏ error tĩnh

### Hệ số vi phân (Kd)
- **Tăng Kd**: Giảm dao động, ổn định hơn, nhưng có thể nhạy với nhiễu
- **Giảm Kd**: Ít nhạy với nhiễu, nhưng ít khả năng chống dao động
- **Hiệu chỉnh**: Sau khi điều chỉnh Kp, tăng Kd cho đến khi robot ổn định

## Mẹo hiệu chỉnh PID

1. **Phương pháp Ziegler-Nichols**:
   - Đặt Ki và Kd về 0
   - Tăng Kp cho đến khi robot dao động liên tục (Kp_crit)
   - Đo chu kỳ dao động (T_crit)
   - Thiết lập:
     - Kp = 0.6 * Kp_crit
     - Ki = 1.2 * Kp_crit / T_crit
     - Kd = 0.075 * Kp_crit * T_crit

2. **Điều chỉnh thủ công**:
   - Bắt đầu với Ki = 0, Kd = 0
   - Tăng Kp cho đến khi phản ứng nhanh nhưng không dao động nhiều
   - Tăng Kd để giảm dao động
   - Tăng Ki để loại bỏ error tĩnh

3. **Lưu ý về tốc độ lấy mẫu**:
   - Đối với robot dò đường, tốc độ lấy mẫu (dt) nên nhỏ (2-10ms)
   - Kd rất nhạy với dt, cần điều chỉnh lại nếu thay đổi chu kỳ lấy mẫu

4. **Giám sát hiệu suất**:
   - Theo dõi giá trị error theo thời gian
   - Kiểm tra xem robot có dao động quá nhiều không
   - Điều chỉnh từng hệ số một để xác định ảnh hưởng

## Trường hợp cụ thể cho robot dò đường

Trong dự án này, các tham số đã được thiết lập:
```cpp
#define KP 0.38    // Hệ số tỉ lệ
#define KI 0.0     // Hệ số tích phân
#define KD 0.0     // Hệ số vi phân
```

Các giá trị này cho thấy:
- Sử dụng điều khiển P đơn giản (chỉ có thành phần tỉ lệ)
- Không sử dụng tích phân (Ki = 0) để tránh tích lũy error
- Không sử dụng vi phân (Kd = 0) có thể để tránh nhiễu từ cảm biến

Đây là thiết lập đơn giản nhưng hiệu quả cho robot dò đường ở tốc độ trung bình. Để cải thiện hiệu suất ở tốc độ cao, nên cân nhắc:
- Tăng Kd để giảm dao động
- Sử dụng Ki nhỏ (0.01-0.05) để loại bỏ error tĩnh
