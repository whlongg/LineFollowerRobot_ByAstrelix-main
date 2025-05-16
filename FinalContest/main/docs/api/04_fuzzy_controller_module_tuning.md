# Kỹ thuật điều chỉnh và tối ưu hóa FuzzyController

## Các tham số cần điều chỉnh

FuzzyController có nhiều tham số có thể điều chỉnh để tối ưu hiệu suất. Dưới đây là các tham số quan trọng:

1. **Phạm vi Error (ErrorRange)**: Xác định giới hạn tối thiểu và tối đa của lỗi vị trí.
2. **Phạm vi ErrorChange (DErrorRange)**: Xác định giới hạn tối thiểu và tối đa của tốc độ thay đổi lỗi.
3. **Phạm vi đầu ra (OutputRange)**: Xác định giới hạn đầu ra của bộ điều khiển.
4. **Hệ số khuếch đại đầu ra (OutputGain)**: Điều chỉnh cường độ của đầu ra.
5. **Ma trận luật mờ (RuleMatrix)**: Xác định mối quan hệ giữa Error, ErrorChange và Output.

## Phương pháp điều chỉnh có hệ thống

### Bước 1: Xác định phạm vi Error và DError

Bắt đầu bằng việc xác định phạm vi lỗi thực tế từ robot:

```cpp
// Đoạn mã để thu thập dữ liệu lỗi
float maxError = -999, minError = 999;
float maxDError = -999, minDError = 999;
float prevError = 0;

// Thực hiện trong vòng lặp điều khiển một thời gian
for (int i = 0; i < 1000; i++) {
  // Đọc giá trị cảm biến
  float sensorValues[SENSOR_COUNT];
  irSensor.readSensors(sensorValues);
  
  // Tính toán lỗi
  float error = irSensor.calculateError();
  float dError = (error - prevError) / 0.01; // dt = 10ms
  prevError = error;
  
  // Cập nhật min/max
  maxError = max(maxError, error);
  minError = min(minError, error);
  maxDError = max(maxDError, dError);
  minDError = min(minDError, dError);
  
  delay(10);
}

// In ra phạm vi
Serial.print("Error Range: "); Serial.print(minError); Serial.print(" to "); Serial.println(maxError);
Serial.print("DError Range: "); Serial.print(minDError); Serial.print(" to "); Serial.println(maxDError);
```

Sau khi có phạm vi, thiết lập các giá trị cho bộ điều khiển, thêm một lượng đệm nhỏ:

```cpp
fuzzyController.setErrorRange(minError * 1.2, maxError * 1.2);
fuzzyController.setDErrorRange(minDError * 1.2, maxDError * 1.2);
```

### Bước 2: Điều chỉnh OutputRange và OutputGain

1. Bắt đầu với OutputRange phù hợp với phạm vi điều khiển động cơ của bạn, và OutputGain = 1.0:

```cpp
fuzzyController.setOutputRange(-100, 100);
fuzzyController.setOutputGain(1.0);
```

2. Chạy robot và quan sát phản ứng:
   - Nếu robot phản ứng quá chậm: Tăng OutputGain
   - Nếu robot dao động: Giảm OutputGain
   - Nếu đầu ra bị giới hạn (robot không thể rẽ đủ nhanh): Tăng phạm vi OutputRange

3. Tinh chỉnh dần dần cho đến khi đạt hiệu suất tốt:

```cpp
// Ví dụ sau vài lần điều chỉnh
fuzzyController.setOutputRange(-150, 150);
fuzzyController.setOutputGain(1.2);
```

### Bước 3: Điều chỉnh và tối ưu Ma trận luật

Ma trận luật mờ quyết định "trí thông minh" của bộ điều khiển. Dưới đây là một số nguyên tắc điều chỉnh:

1. **Khảo sát ma trận ban đầu**:

```cpp
// Ma trận luật mặc định
int8_t ruleMatrix[7][5] = {
  // DERROR_NB  DERROR_NS  DERROR_ZE  DERROR_PS  DERROR_PB
    {OUTPUT_NB, OUTPUT_NB, OUTPUT_NB, OUTPUT_NM, OUTPUT_NS}, // ERROR_NB
    {OUTPUT_NB, OUTPUT_NM, OUTPUT_NM, OUTPUT_NS, OUTPUT_ZE}, // ERROR_NM
    {OUTPUT_NM, OUTPUT_NS, OUTPUT_NS, OUTPUT_ZE, OUTPUT_PS}, // ERROR_NS
    {OUTPUT_NS, OUTPUT_NS, OUTPUT_ZE, OUTPUT_PS, OUTPUT_PS}, // ERROR_ZE
    {OUTPUT_NS, OUTPUT_ZE, OUTPUT_PS, OUTPUT_PS, OUTPUT_PM}, // ERROR_PS
    {OUTPUT_ZE, OUTPUT_PS, OUTPUT_PM, OUTPUT_PM, OUTPUT_PB}, // ERROR_PM
    {OUTPUT_PS, OUTPUT_PM, OUTPUT_PB, OUTPUT_PB, OUTPUT_PB}  // ERROR_PB
};
```

2. **Các nguyên tắc điều chỉnh**:

   a. **Phát hiện vấn đề**:
   - Robot quá mạnh khi rẽ: Giảm các giá trị Output_PB/NB trong ma trận
   - Robot quá yếu khi rẽ: Tăng các giá trị Output_PB/NB trong ma trận
   - Dao động quanh đường line: Điều chỉnh các giá trị zero-crossing (khu vực ERROR_ZE)

   b. **Tối ưu cho các tình huống cụ thể**:
   - Đường cong sắc: Tăng giá trị đầu ra ở khu vực error lớn
   - Đường thẳng tốc độ cao: Làm mượt đầu ra ở khu vực error nhỏ

3. **Ví dụ điều chỉnh ma trận**:

```cpp
// Ma trận được tối ưu hóa cho đường cong sắc
int8_t ruleMatrix[7][5] = {
  // DERROR_NB  DERROR_NS  DERROR_ZE  DERROR_PS  DERROR_PB
    {OUTPUT_NB, OUTPUT_NB, OUTPUT_NB, OUTPUT_NB, OUTPUT_NM}, // ERROR_NB - Tăng cường
    {OUTPUT_NB, OUTPUT_NM, OUTPUT_NM, OUTPUT_NS, OUTPUT_ZE}, // ERROR_NM
    {OUTPUT_NM, OUTPUT_NS, OUTPUT_NS, OUTPUT_ZE, OUTPUT_PS}, // ERROR_NS
    {OUTPUT_NS, OUTPUT_ZE, OUTPUT_ZE, OUTPUT_ZE, OUTPUT_PS}, // ERROR_ZE - Làm mượt
    {OUTPUT_NS, OUTPUT_PS, OUTPUT_PS, OUTPUT_PS, OUTPUT_PM}, // ERROR_PS
    {OUTPUT_ZE, OUTPUT_PS, OUTPUT_PM, OUTPUT_PM, OUTPUT_PB}, // ERROR_PM
    {OUTPUT_NM, OUTPUT_PB, OUTPUT_PB, OUTPUT_PB, OUTPUT_PB}  // ERROR_PB - Tăng cường
};
```

### Bước 4: Tạo phương thức để dễ dàng tùy chỉnh ma trận

```cpp
// Thêm phương thức này vào lớp FuzzyController
void FuzzyController::setRuleMatrix(int8_t row, int8_t col, int8_t value) {
  if (row >= 0 && row < 7 && col >= 0 && col < 5) {
    ruleMatrix[row][col] = value;
  }
}

// Cách sử dụng
void tuneRuleMatrixForSharpTurns() {
  // Điều chỉnh cho ERROR_NB + DERROR_PS
  fuzzyController.setRuleMatrix(0, 3, OUTPUT_NB); // Tăng cường phản ứng
  
  // Điều chỉnh cho ERROR_PB + DERROR_NS
  fuzzyController.setRuleMatrix(6, 1, OUTPUT_PB); // Tăng cường phản ứng
}
```

## Phân tích hiệu suất và sửa lỗi

### Ghi lại và phân tích dữ liệu

Việc điều chỉnh hiệu quả đòi hỏi có dữ liệu thực tế. Tạo một chức năng ghi lại và phân tích:

```cpp
// Cấu trúc dữ liệu để ghi lại
struct FuzzyData {
  float error;
  float dError;
  float output;
  unsigned long timestamp;
};

// Mảng lưu trữ
FuzzyData dataLog[1000];
int dataIndex = 0;

// Ghi lại dữ liệu trong vòng lặp điều khiển
void recordFuzzyData(float error, float dError, float output) {
  if (dataIndex < 1000) {
    dataLog[dataIndex].error = error;
    dataLog[dataIndex].dError = dError;
    dataLog[dataIndex].output = output;
    dataLog[dataIndex].timestamp = millis();
    dataIndex++;
  }
}

// In ra dữ liệu để phân tích
void printFuzzyDataLog() {
  Serial.println("timestamp,error,dError,output");
  for (int i = 0; i < dataIndex; i++) {
    Serial.print(dataLog[i].timestamp);
    Serial.print(",");
    Serial.print(dataLog[i].error);
    Serial.print(",");
    Serial.print(dataLog[i].dError);
    Serial.print(",");
    Serial.println(dataLog[i].output);
  }
}
```

### Phương thức trực quan hóa hàm thành viên

Để hiểu rõ hơn về việc fuzzification, tạo một phương thức để trực quan hóa hàm thành viên:

```cpp
// Thêm phương thức này vào lớp FuzzyController
void FuzzyController::visualizeMembershipFunctions() {
  Serial.println("Error Membership Functions:");
  Serial.println("Value,NB,NM,NS,ZE,PS,PM,PB");
  
  // In ra 20 điểm trên phạm vi error
  float step = (errorMax - errorMin) / 20.0f;
  for (float x = errorMin; x <= errorMax; x += step) {
    float membership[7] = {0};
    calculateErrorMembership(x, membership);
    
    Serial.print(x);
    for (int i = 0; i < 7; i++) {
      Serial.print(",");
      Serial.print(membership[i]);
    }
    Serial.println();
  }
  
  Serial.println("\nDError Membership Functions:");
  Serial.println("Value,NB,NS,ZE,PS,PB");
  
  // In ra 20 điểm trên phạm vi dError
  step = (dErrorMax - dErrorMin) / 20.0f;
  for (float x = dErrorMin; x <= dErrorMax; x += step) {
    float membership[5] = {0};
    calculateDErrorMembership(x, membership);
    
    Serial.print(x);
    for (int i = 0; i < 5; i++) {
      Serial.print(",");
      Serial.print(membership[i]);
    }
    Serial.println();
  }
}
```

### Phương thức mô phỏng đầu ra

Để hiểu cách bộ điều khiển sẽ phản ứng với các giá trị đầu vào khác nhau:

```cpp
// Thêm phương thức này vào lớp FuzzyController
void FuzzyController::simulateResponses() {
  Serial.println("Simulating Fuzzy Controller Responses:");
  Serial.println("Error,DError,Output");
  
  // Mô phỏng nhiều tổ hợp error/dError khác nhau
  float errorValues[] = {-100, -75, -50, -25, 0, 25, 50, 75, 100};
  float dErrorValues[] = {-50, -25, 0, 25, 50};
  
  for (int i = 0; i < 9; i++) {
    for (int j = 0; j < 5; j++) {
      float error = errorValues[i];
      float dError = dErrorValues[j];
      
      // Tính đầu ra mà không ảnh hưởng đến trạng thái nội bộ
      float errorMembership[7] = {0};
      float dErrorMembership[5] = {0};
      
      calculateErrorMembership(error, errorMembership);
      calculateDErrorMembership(dError, dErrorMembership);
      float output = inferenceAndDefuzzify(errorMembership, dErrorMembership) * outputGain;
      
      Serial.print(error);
      Serial.print(",");
      Serial.print(dError);
      Serial.print(",");
      Serial.println(output);
    }
  }
}
```

## Các chiến lược điều chỉnh cho tình huống cụ thể

### Đường cong sắc

```cpp
void tuneFuzzyForSharpTurns() {
  // Mở rộng phạm vi error
  fuzzyController.setErrorRange(-120, 120);
  
  // Tăng gain đầu ra
  fuzzyController.setOutputGain(1.5);
  
  // Điều chỉnh ma trận luật để phản ứng mạnh hơn với lỗi lớn
  fuzzyController.setRuleMatrix(0, 2, OUTPUT_NB); // ERROR_NB, DERROR_ZE
  fuzzyController.setRuleMatrix(6, 2, OUTPUT_PB); // ERROR_PB, DERROR_ZE
  
  // Chắc chắn đầu ra đủ lớn
  fuzzyController.setOutputRange(-200, 200);
}
```

### Đường thẳng tốc độ cao

```cpp
void tuneFuzzyForHighSpeedStraight() {
  // Thu hẹp phạm vi error để nhạy hơn với lỗi nhỏ
  fuzzyController.setErrorRange(-80, 80);
  
  // Giảm gain đầu ra để giảm dao động
  fuzzyController.setOutputGain(0.8);
  
  // Tinh chỉnh ma trận luật cho error nhỏ
  fuzzyController.setRuleMatrix(2, 2, OUTPUT_ZE); // ERROR_NS, DERROR_ZE
  fuzzyController.setRuleMatrix(3, 2, OUTPUT_ZE); // ERROR_ZE, DERROR_ZE
  fuzzyController.setRuleMatrix(4, 2, OUTPUT_ZE); // ERROR_PS, DERROR_ZE
  
  // Khuyến khích sự ổn định khi gần line
  fuzzyController.setRuleMatrix(3, 1, OUTPUT_ZE); // ERROR_ZE, DERROR_NS
  fuzzyController.setRuleMatrix(3, 3, OUTPUT_ZE); // ERROR_ZE, DERROR_PS
}
```

### Chế độ dò line có nhiễu

```cpp
void tuneFuzzyForNoisyTrack() {
  // Giảm nhạy cảm với sự thay đổi error
  fuzzyController.setDErrorRange(-30, 30);
  
  // Giảm gain đầu ra
  fuzzyController.setOutputGain(0.7);
  
  // Tập trung nhiều hơn vào error hiện tại, ít vào sự thay đổi
  for (int i = 0; i < 7; i++) {
    for (int j = 0; j < 5; j++) {
      // Lấy giá trị đầu ra hiện tại cho error này
      int8_t currentOutput = fuzzyController.getRuleOutput(i, 2); // DERROR_ZE
      
      // Điều chỉnh các cột khác để gần với cột DERROR_ZE hơn
      fuzzyController.setRuleMatrix(i, j, (currentOutput + fuzzyController.getRuleOutput(i, j)) / 2);
    }
  }
}
```

## Kỹ thuật chuyển đổi thích ứng

Một cải tiến mạnh mẽ là khả năng tự động chuyển đổi giữa các bộ thông số khác nhau dựa trên tình huống:

```cpp
// Tạo một lớp quản lý thích ứng
class AdaptiveFuzzyManager {
private:
  FuzzyController* fuzzy;
  
  // Lưu trữ các bộ thông số
  float sharpTurnGain;
  float straightGain;
  float noisyGain;
  
  int8_t ruleMatrixBackup[7][5];
  
public:
  AdaptiveFuzzyManager(FuzzyController* controller) : fuzzy(controller) {
    // Lưu trữ thông số ban đầu
    backupCurrentSettings();
    
    // Thiết lập các giá trị gain đặc biệt
    sharpTurnGain = 1.5;
    straightGain = 0.8;
    noisyGain = 0.7;
  }
  
  void backupCurrentSettings() {
    // Lưu ma trận luật hiện tại
    for (int i = 0; i < 7; i++) {
      for (int j = 0; j < 5; j++) {
        ruleMatrixBackup[i][j] = fuzzy->getRuleOutput(i, j);
      }
    }
  }
  
  void restoreBackup() {
    // Khôi phục ma trận luật
    for (int i = 0; i < 7; i++) {
      for (int j = 0; j < 5; j++) {
        fuzzy->setRuleMatrix(i, j, ruleMatrixBackup[i][j]);
      }
    }
    fuzzy->setOutputGain(1.0);
  }
  
  void switchToSharpTurnMode() {
    restoreBackup();
    fuzzy->setOutputGain(sharpTurnGain);
    // Điều chỉnh các luật mờ...
  }
  
  void switchToStraightMode() {
    restoreBackup();
    fuzzy->setOutputGain(straightGain);
    // Điều chỉnh các luật mờ...
  }
  
  void switchToNoisyMode() {
    restoreBackup();
    fuzzy->setOutputGain(noisyGain);
    // Điều chỉnh các luật mờ...
  }
  
  void adaptToConditions(float errorMagnitude, float dErrorMagnitude) {
    // Tự động chọn chế độ phù hợp
    if (errorMagnitude > 70) {
      switchToSharpTurnMode();
    } else if (errorMagnitude < 20 && dErrorMagnitude < 15) {
      switchToStraightMode();
    } else if (dErrorMagnitude > 40) {
      switchToNoisyMode();
    } else {
      restoreBackup();
    }
  }
};

// Sử dụng trong vòng lặp chính
void loop() {
  // Đọc cảm biến và tính error
  float error = irSensor.calculateError();
  float dError = (error - prevError) / dt;
  prevError = error;
  
  // Thích ứng với điều kiện
  adaptiveFuzzyManager.adaptToConditions(abs(error), abs(dError));
  
  // Tính toán đầu ra điều khiển
  float output = fuzzyController.compute(error, dt);
  
  // Điều khiển động cơ
  motorController.setMotorSpeeds(baseSpeed - output, baseSpeed + output);
}
```
