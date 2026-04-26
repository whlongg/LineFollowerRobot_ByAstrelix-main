# Tài liệu mô đun ReadIR

## Tổng quan

Mô đun `ReadIR` quản lý việc đọc và xử lý dữ liệu từ các cảm biến hồng ngoại (IR) được sử dụng để phát hiện đường line. Module này sử dụng thư viện QTRSensors để tương tác với cảm biến và thực hiện lọc nhiễu để đảm bảo đọc dữ liệu chính xác.

## Tệp liên quan
- **ReadIR.h**: Định nghĩa giao diện của lớp ReadIR
- **ReadIR.cpp**: Triển khai các phương thức xử lý cảm biến IR

## Cấu hình cảm biến

Mô đun này sử dụng 4 cảm biến IR được cấu hình với các thông số sau:

```cpp
#define SENSOR_COUNT 4        // Số lượng cảm biến IR
#define FILTER_SIZE 5         // Kích thước bộ lọc cho dữ liệu cảm biến
```

Các chân kết nối cảm biến được định nghĩa trong code:
```cpp
const uint8_t IR_PINS[SENSOR_COUNT] = {4, 3, 1, 0};
```

## Lớp ReadIR

Lớp `ReadIR` cung cấp các phương thức để đọc, xử lý và hiệu chuẩn cảm biến hồng ngoại.

### Constructor

```cpp
ReadIR()
```

**Mô tả**: Khởi tạo đối tượng ReadIR với các giá trị mặc định.

**Chi tiết triển khai**:
- Thiết lập các giá trị ban đầu cho các mảng lưu trữ giá trị cảm biến
- Không thiết lập hardware (việc này được thực hiện trong phương thức begin())

**Ví dụ**:
```cpp
ReadIR irSensor;
```

### begin

```cpp
void begin()
```

**Mô tả**: Khởi tạo và cấu hình cảm biến hồng ngoại.

**Chi tiết triển khai**:
- Thiết lập chân LED cho cảm biến IR
- Cấu hình cảm biến QTR với các chân đã định nghĩa
- Thiết lập thời gian và ngưỡng đọc cảm biến

**Ví dụ**:
```cpp
irSensor.begin();
```

### readSensors

```cpp
void readSensors(float *sensorValues)
```

**Mô tả**: Đọc giá trị từ cảm biến IR và lọc nhiễu.

**Tham số**:
- `sensorValues`: Con trỏ đến mảng lưu trữ giá trị đọc được (kích thước SENSOR_COUNT)

**Chi tiết triển khai**:
- Đọc giá trị thô từ cảm biến QTR
- Lưu trữ giá trị vào bộ đệm lọc
- Áp dụng bộ lọc để giảm nhiễu
- Trả về giá trị đã lọc thông qua tham số sensorValues

**Ví dụ**:
```cpp
float sensorValues[SENSOR_COUNT];
irSensor.readSensors(sensorValues);

// In ra giá trị
for (int i = 0; i < SENSOR_COUNT; i++) {
  Serial.print(sensorValues[i]);
  Serial.print(" ");
}
Serial.println();
```

### calculateError

```cpp
float calculateError()
```

**Mô tả**: Tính toán giá trị error dựa trên vị trí của đường line.

**Giá trị trả về**: Giá trị error (-100 đến +100)
- Giá trị 0: Đường line ở chính giữa
- Giá trị âm: Đường line lệch sang trái
- Giá trị dương: Đường line lệch sang phải

**Chi tiết triển khai**:
- Đọc giá trị từ cảm biến
- Tính toán vị trí line dựa trên tương quan giữa các giá trị cảm biến
- Chuẩn hóa kết quả thành error trong khoảng -100 đến +100

**Ví dụ**:
```cpp
float error = irSensor.calculateError();
Serial.print("Line Error: ");
Serial.println(error);
```

### CalibrateSensor

```cpp
void CalibrateSensor()
```

**Mô tả**: Thực hiện hiệu chuẩn cảm biến IR bằng cách quét qua đường line nhiều lần.

**Chi tiết triển khai**:
- Điều khiển robot di chuyển để quét cảm biến qua đường line
- Thu thập giá trị min và max cho mỗi cảm biến
- Lưu trữ các giá trị hiệu chuẩn để sử dụng trong tính toán sau này

**Ví dụ**:
```cpp
// Thực hiện hiệu chuẩn (thường được gọi trong setup)
irSensor.CalibrateSensor();
```

### filterSensorValues (private)

```cpp
void filterSensorValues(float rawValues[][FILTER_SIZE], float *filteredValues)
```

**Mô tả**: Lọc giá trị cảm biến để giảm nhiễu.

**Tham số**:
- `rawValues`: Mảng 2 chiều chứa giá trị thô từ cảm biến
- `filteredValues`: Con trỏ đến mảng lưu trữ giá trị đã lọc

**Chi tiết triển khai**:
- Sử dụng bộ lọc trung vị hoặc bộ lọc trung bình để loại bỏ nhiễu
- Tính toán giá trị trung bình cho mỗi cảm biến từ các giá trị được lưu trong bộ đệm

## Chi tiết triển khai

### Đọc cảm biến và lọc nhiễu

Quá trình đọc cảm biến được thực hiện qua hai bước:

1. **Đọc giá trị thô**:
```cpp
// Đọc giá trị thô từ cảm biến
uint16_t rawValues[SENSOR_COUNT];
qtr.read(rawValues);

// Chuẩn hóa và lưu vào bộ đệm lọc
for (int i = 0; i < SENSOR_COUNT; i++) {
  ir_values[i][filter_index] = map(rawValues[i], 0, 1023, 0, 100);
}
filter_index = (filter_index + 1) % FILTER_SIZE;
```

2. **Lọc nhiễu**:
```cpp
// Lọc nhiễu bằng cách tính trung bình
void filterSensorValues(float rawValues[][FILTER_SIZE], float *filteredValues) {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    float sum = 0;
    for (int j = 0; j < FILTER_SIZE; j++) {
      sum += rawValues[i][j];
    }
    filteredValues[i] = sum / FILTER_SIZE;
  }
}
```

### Tính toán error

Quá trình tính toán error dựa trên vị trí của đường line:

```cpp
float calculateError() {
  // Đọc giá trị cảm biến
  float sensorValues[SENSOR_COUNT];
  readSensors(sensorValues);
  
  // Tính tổng giá trị và vị trí có trọng số
  float totalValue = 0;
  float weightedPosition = 0;
  for (int i = 0; i < SENSOR_COUNT; i++) {
    totalValue += sensorValues[i];
    weightedPosition += sensorValues[i] * (i - (SENSOR_COUNT - 1) / 2.0);
  }
  
  // Tránh chia cho 0
  if (totalValue < 0.001) return 0;
  
  // Tính error và chuẩn hóa về khoảng -100 đến 100
  float normalizedPosition = weightedPosition / totalValue;
  return normalizedPosition * 100.0;
}
```

### Hiệu chuẩn cảm biến

Quá trình hiệu chuẩn giúp robot thích nghi với các điều kiện ánh sáng và bề mặt khác nhau:

```cpp
void CalibrateSensor() {
  // Thiết lập LED báo hiệu đang hiệu chuẩn
  digitalWrite(W_LED_ON, HIGH);
  digitalWrite(IR_LED_ON, HIGH);
  
  // Hiệu chuẩn bằng cách di chuyển qua đường line
  for (uint16_t i = 0; i < 120; i++) {
    // Điều khiển robot di chuyển để quét cảm biến
    if (i > 30 && i <= 90) {
      // Di chuyển sang phải
      setMotorSpeeds(SPEED_CALIBRATE, -SPEED_CALIBRATE);
    } else {
      // Di chuyển sang trái
      setMotorSpeeds(-SPEED_CALIBRATE, SPEED_CALIBRATE);
    }
    
    // Hiệu chuẩn cảm biến
    qtr.calibrate();
    delay(20);
  }
  
  // Dừng động cơ và tắt LED
  setMotorSpeeds(0, 0);
  digitalWrite(W_LED_ON, LOW);
  digitalWrite(IR_LED_ON, LOW);
}
```

## Cách sử dụng

### Khởi tạo

```cpp
// Trong hàm setup()
ReadIR irSensor;
irSensor.begin();

// Hiệu chuẩn nếu cần
irSensor.CalibrateSensor();
```

### Đọc giá trị cảm biến và tính error

```cpp
// Trong vòng lặp điều khiển
float error = irSensor.calculateError();

// Sử dụng error cho điều khiển PID
float output = pid.compute(error, dt);

// Điều khiển động cơ dựa trên output
int leftSpeed = baseSpeed + output;
int rightSpeed = baseSpeed - output;
setMotorSpeeds(leftSpeed, rightSpeed);
```

### Đọc trực tiếp giá trị cảm biến

```cpp
float sensorValues[SENSOR_COUNT];
irSensor.readSensors(sensorValues);

// Xử lý giá trị trực tiếp
for (int i = 0; i < SENSOR_COUNT; i++) {
  // Xử lý từng cảm biến
  if (sensorValues[i] > threshold) {
    // Phát hiện đường line
  }
}
```

## Lưu ý khi sử dụng

1. **Hiệu chuẩn cảm biến**:
   - Nên thực hiện hiệu chuẩn mỗi khi robot được đặt trên đường đua mới
   - Hiệu chuẩn giúp robot thích nghi với các điều kiện ánh sáng khác nhau

2. **Vị trí cảm biến**:
   - Khoảng cách giữa các cảm biến IR ảnh hưởng đến khả năng phát hiện đường line
   - Các cảm biến nên được đặt đối xứng qua trục giữa của robot

3. **Bộ lọc nhiễu**:
   - FILTER_SIZE ảnh hưởng đến độ nhạy và độ trễ của cảm biến
   - Giá trị lớn hơn giảm nhiễu nhưng tăng độ trễ

4. **Đèn LED IR**:
   - Cường độ của đèn LED IR ảnh hưởng đến phạm vi và độ nhạy của cảm biến
   - Có thể cần điều chỉnh cường độ LED dựa trên điều kiện ánh sáng môi trường

## Nâng cấp khả năng

1. **Thêm cảm biến**:
   - Tăng số lượng cảm biến (ví dụ: 6 hoặc 8) để tăng độ chính xác
   - Cập nhật SENSOR_COUNT và IR_PINS tương ứng

2. **Thuật toán phát hiện nâng cao**:
   - Triển khai thuật toán phát hiện ngã ba, ngã tư
   - Thêm khả năng phát hiện mất line hoàn toàn

3. **Lọc nhiễu thích ứng**:
   - Điều chỉnh mức lọc dựa trên tốc độ (lọc ít hơn ở tốc độ cao)
   - Sử dụng bộ lọc Kalman thay vì bộ lọc trung bình đơn giản

4. **Lưu trữ hiệu chuẩn**:
   - Lưu giá trị hiệu chuẩn vào EEPROM để sử dụng sau khi khởi động lại
   - Thêm tùy chọn hiệu chuẩn nhanh và hiệu chuẩn đầy đủ

## Ví dụ triển khai nâng cao

### Phát hiện đường giao nhau

```cpp
bool detectIntersection() {
  float sensorValues[SENSOR_COUNT];
  readSensors(sensorValues);
  
  // Tính số lượng cảm biến phát hiện đường line
  int lineDetected = 0;
  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (sensorValues[i] > 70) {  // Ngưỡng phát hiện line
      lineDetected++;
    }
  }
  
  // Nếu nhiều cảm biến cùng phát hiện line, có thể là giao lộ
  return (lineDetected >= SENSOR_COUNT - 1);
}
```

### Phát hiện mất line

```cpp
bool detectLineLost() {
  float sensorValues[SENSOR_COUNT];
  readSensors(sensorValues);
  
  // Tính tổng giá trị cảm biến
  float totalValue = 0;
  for (int i = 0; i < SENSOR_COUNT; i++) {
    totalValue += sensorValues[i];
  }
  
  // Nếu tổng giá trị quá thấp, line có thể đã bị mất
  return (totalValue < 10 * SENSOR_COUNT);
}
```

### Sử dụng bộ lọc Kalman

```cpp
// Thêm biến cho bộ lọc Kalman
float errorEstimate = 0;
float errorCovariance = 1;
float measurementNoise = 0.1;
float processNoise = 0.01;

float kalmanFilter(float measurement) {
  // Dự đoán
  float predictedCovariance = errorCovariance + processNoise;
  
  // Cập nhật
  float kalmanGain = predictedCovariance / (predictedCovariance + measurementNoise);
  float currentEstimate = errorEstimate + kalmanGain * (measurement - errorEstimate);
  errorCovariance = (1 - kalmanGain) * predictedCovariance;
  errorEstimate = currentEstimate;
  
  return currentEstimate;
}

// Sử dụng trong calculateError
float calculateError() {
  // ... code tính error ...
  
  // Áp dụng bộ lọc Kalman
  float filteredError = kalmanFilter(error);
  return filteredError;
}
```
