# Phân tích hệ thống điều khiển PID

## Tổng quan về hệ thống PID trong dự án

Hệ thống điều khiển PID của robot dò line được triển khai thông qua lớp `PIDController` và được tích hợp vào hệ thống thông qua lớp `RobotController`. Dưới đây là phân tích chi tiết về cách triển khai và các vấn đề tiềm ẩn.

## Phân tích cấu trúc PID

### Cấu hình PID hiện tại
```cpp
// Trong main.ino
#define KP 0.38     // Tăng hệ số tỉ lệ để phản ứng nhanh hơn với lỗi
#define KI 0.0    // Giảm hệ số tích phân để tránh dao động
#define KD 0.0    // Tăng hệ số vi phân để ổn định khi đường cong
```

Cấu hình này chỉ sử dụng thành phần P (tỉ lệ) và bỏ qua các thành phần I (tích phân) và D (vi phân). Điều này không tối ưu vì:

1. **Thiếu thành phần D**: Thành phần vi phân (KD) có vai trò quan trọng trong việc dự đoán và phản ứng trước với những thay đổi error, đặc biệt khi robot đi vào đường cong hoặc góc cua.

2. **Thiếu thành phần I**: Mặc dù KI = 0 có thể tránh dao động, nhưng nó cũng làm mất khả năng khắc phục sai số tĩnh (steady-state error) của hệ thống.

3. **Mâu thuẫn trong mã nguồn**: 
   - Trong RobotController.h, các giá trị mặc định là: `kp = 0.38f, ki = 0.0f, kd = 3.8f`
   - Nhưng trong main.ino, các giá trị được đặt là: `KP = 0.38, KI = 0.0, KD = 0.0`

## Phân tích triển khai PIDController

```cpp
// Trong PIDController.cpp
float PIDController::compute(float error, float dt) {
    // Xác định đạo hàm của error
    float derivative = 0.0f;
    
    if (!isFirstCompute) {
        // Nếu không phải lần đầu, tính đạo hàm của error
        derivative = (error - lastError) / dt;
    } else {
        // Nếu là lần đầu, bỏ qua thành phần đạo hàm
        isFirstCompute = false;
    }
    
    // Cập nhật giá trị integral
    integral += error * dt;
    
    // Giới hạn giá trị integral để tránh tích lũy quá mức
    integral = std::max(integralMin, std::min(integralMax, integral));
    
    // Lưu error hiện tại để sử dụng cho lần tính toán tiếp theo
    lastError = error;
    
    // Tính toán đầu ra PID
    float output = kp * error + ki * integral + kd * derivative;
    
    // Giới hạn đầu ra
    output = std::max(outputMin, std::min(outputMax, output));
    
    return output;
}
```

### Vấn đề trong triển khai PID

1. **Cách tính derivative đơn giản**: Thuật toán sử dụng phương pháp khác biệt hữu hạn đơn giản (naive finite difference) để tính toán đạo hàm. Phương pháp này rất nhạy cảm với nhiễu, đặc biệt khi dt nhỏ.

2. **Thiếu bộ lọc derivative**: Không có bộ lọc cho thành phần đạo hàm, khiến nó dễ bị ảnh hưởng bởi nhiễu từ cảm biến.

3. **Phương pháp tích phân đơn giản**: Phương pháp Euler được sử dụng để tính tích phân có thể gây ra sai số tích lũy. Ngoài ra, không có cơ chế anti-windup tiên tiến cho thành phần tích phân.

4. **Tần số cập nhật cố định**: Giá trị dt được cố định trong nhiều lần gọi, không xét đến sự thay đổi về chu kỳ thực tế giữa các lần tính toán.

## Phân tích cách sử dụng PID trong processPID()

```cpp
// Trong main.ino - hàm processPID()
void processPID() {
  // Đọc giá trị từ cảm biến IR và tính error
  rawError = irSensor.calculateError();
  
  // Áp dụng bộ lọc cho error để làm mịn
  filteredError = errorFilterAlpha * rawError + (1 - errorFilterAlpha) * filteredError;
  
  // Tính toán điều khiển từ bộ điều khiển
  rawControlOutput = controller.compute(filteredError, SAMPLE_TIME / 1000.0f);
  
  // Áp dụng bộ lọc cho đầu ra để làm mịn
  filteredOutput = outputFilterAlpha * rawControlOutput + (1 - outputFilterAlpha) * filteredOutput;
  
  // Gọi hàm chung để xử lý đầu ra
  processOutput(filteredOutput);
  
  // Cập nhật giá trị error cuối cùng
  lastError = filteredError;
}
```

### Vấn đề trong cách sử dụng PID

1. **Bộ lọc error kép**: Error đã được lọc trong ReadIR::readSensors() và lại tiếp tục được lọc trong processPID(), có thể gây ra độ trễ quá lớn.

2. **Bộ lọc đầu ra**: Đầu ra điều khiển bị lọc thêm một lần nữa, có thể làm giảm hiệu quả của thành phần vi phân.

3. **Không cân nhắc thời gian thực tế**: dt được cố định là SAMPLE_TIME/1000.0f, không tính đến thời gian thực tế giữa các lần gọi, có thể dẫn đến sai lệch trong tính toán.

4. **Thiếu cơ chế feed-forward**: Không có thành phần feed-forward để dự đoán và đáp ứng trước với những thay đổi đã biết.

## Vấn đề với đầu vào PID (error calculation)

```cpp
// Trong ReadIR.cpp - hàm calculateError()
float ReadIR::calculateError() {
    float sensorValues[SENSOR_COUNT];
    readSensors(sensorValues);

    // Convert float values to uint16_t for QTR line position calculation
    uint16_t rawValues[SENSOR_COUNT];
    for (int i = 0; i < SENSOR_COUNT; i++) {
        rawValues[i] = (uint16_t)sensorValues[i];
    }

    // Get line position from QTRSensors (returns value from 0 to 3000 for 4 sensors)
    uint16_t position = qtr.readLineBlack(rawValues);

    float error = (position - 1500.0f) / 15.0f; // [-100, 100]

    return error;
}
```

### Vấn đề trong tính toán error

1. **Tối ưu hóa không hiệu quả**: Chuyển đổi giữa float và uint16_t không cần thiết và có thể ảnh hưởng đến hiệu suất.

2. **Thiếu xử lý trường hợp đặc biệt**: Không có xử lý cho trường hợp tất cả cảm biến mất dấu đường (robot đi ra ngoài line).

3. **Tỷ lệ error cố định**: Hệ số chia 15.0f để chuyển đổi thành phạm vi [-100, 100] có thể không phù hợp cho mọi tình huống, đặc biệt là khi có sự thay đổi về khoảng cách giữa các cảm biến hoặc độ rộng của đường.

## Đề xuất cải tiến hệ thống PID

### 1. Cấu hình PID hợp lý hơn
```cpp
// Đề xuất các giá trị khởi đầu cho tinh chỉnh
#define KP 0.35    // Thành phần tỉ lệ vừa phải
#define KI 0.01    // Thêm thành phần tích phân nhỏ để khắc phục sai số tĩnh
#define KD 0.2     // Thêm thành phần vi phân để phản ứng tốt hơn với đường cong
```

### 2. Cải thiện tính toán thành phần vi phân
```cpp
// Đề xuất cải thiện trong PIDController.cpp
float PIDController::compute(float error, float dt) {
    // ... code hiện tại
    
    // Thêm bộ lọc cho thành phần vi phân
    float derivativeFiltered = derivative;
    if (!isFirstCompute) {
        // Áp dụng bộ lọc low-pass cho thành phần vi phân
        const float alpha = 0.7f; // Hệ số lọc (có thể điều chỉnh)
        derivativeFiltered = alpha * derivative + (1 - alpha) * lastDerivative;
        lastDerivative = derivativeFiltered;
    }
    
    // Tính toán đầu ra PID với thành phần vi phân đã lọc
    float output = kp * error + ki * integral + kd * derivativeFiltered;
    
    // ... code hiện tại
}
```

### 3. Cải thiện thành phần tích phân với anti-windup
```cpp
// Đề xuất cải thiện trong PIDController.cpp
float PIDController::compute(float error, float dt) {
    // ... code hiện tại
    
    // Cập nhật giá trị integral với cơ chế anti-windup
    float preIntegral = integral;
    integral += error * dt;
    
    // Tính toán đầu ra PID tạm thời
    float rawOutput = kp * error + ki * integral + kd * derivative;
    
    // Kiểm tra nếu đầu ra bị bão hòa
    if (rawOutput > outputMax || rawOutput < outputMin) {
        // Khôi phục lại giá trị tích phân trước đó để tránh windup
        integral = preIntegral;
    }
    
    // Giới hạn tích phân
    integral = std::max(integralMin, std::min(integralMax, integral));
    
    // Tính toán đầu ra PID cuối cùng
    float output = kp * error + ki * integral + kd * derivative;
    
    // ... code hiện tại
}
```

### 4. Cập nhật dt dựa trên thời gian thực

```cpp
// Đề xuất cải thiện trong main.ino - hàm processPID
void processPID() {
  // Tính dt thực tế
  unsigned long currentTime = millis();
  float dt = (currentTime - previousPIDTime) / 1000.0f;
  dt = std::max(0.001f, std::min(0.1f, dt)); // Giới hạn dt trong phạm vi hợp lý
  
  // ... code hiện tại
  
  // Tính toán điều khiển với dt thực tế
  rawControlOutput = controller.compute(filteredError, dt);
  
  // ... code hiện tại
  
  previousPIDTime = currentTime;
}
```

## Kết luận

Hệ thống PID trong dự án robot dò line có tiềm năng tốt nhưng chưa được tối ưu hóa. Với một số cải tiến về cấu hình tham số và thuật toán, hệ thống có thể đạt được hiệu suất cao hơn nhiều, đặc biệt là trong việc theo dõi đường cong, góc cua và thích ứng với các điều kiện khác nhau.

Tuy nhiên, để đạt được hiệu quả cao nhất, việc tinh chỉnh PID nên được thực hiện thông qua thử nghiệm thực tế, bắt đầu với các giá trị đề xuất và tinh chỉnh dần dựa trên phản hồi của robot trong các tình huống cụ thể. Đồng thời, cải thiện thuật toán tính toán derivative và anti-windup sẽ giúp tăng độ ổn định và hiệu suất của hệ thống điều khiển.
