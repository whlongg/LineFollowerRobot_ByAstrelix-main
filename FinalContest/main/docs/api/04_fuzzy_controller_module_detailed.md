# Tài liệu chi tiết mô đun FuzzyController

## Tổng quan

Mô đun `FuzzyController` là một hệ thống điều khiển tiên tiến dựa trên logic mờ (fuzzy logic), cho phép robot điều khiển chính xác và mượt mà hơn khi theo dõi đường line so với các phương pháp điều khiển truyền thống như PID. Module này cung cấp khả năng xử lý phi tuyến, thích ứng tốt hơn với các đường cong phức tạp, giao lộ, và các tình huống đặc biệt khác trên đường đua.

## Lý thuyết nền tảng về Fuzzy Logic

Trước khi đi vào chi tiết triển khai, chúng ta cần hiểu một số khái niệm cơ bản về logic mờ:

### Các khái niệm cơ bản

1. **Tập mờ (Fuzzy Set)**: Khác với tập hợp thông thường (một phần tử hoặc thuộc hoặc không thuộc tập hợp), trong tập mờ, một phần tử có thể thuộc về tập với một mức độ thành viên từ 0 đến 1.

2. **Hàm thành viên (Membership Function)**: Hàm xác định mức độ thành viên của một giá trị trong một tập mờ.

3. **Biến ngôn ngữ (Linguistic Variable)**: Biến có giá trị là các từ hoặc cụm từ thay vì số. Ví dụ: "Error" có thể có các giá trị như "Negative Large", "Zero", "Positive Small", v.v.

4. **Luật mờ (Fuzzy Rule)**: Quy tắc dạng IF-THEN sử dụng các biến ngôn ngữ. Ví dụ: "IF Error is Negative Large AND ErrorChange is Zero THEN Output is Positive Large".

5. **Fuzzification**: Quá trình chuyển đổi giá trị đầu vào thành mức độ thành viên trong các tập mờ.

6. **Inference**: Quá trình áp dụng các luật mờ để xác định kết quả mờ.

7. **Defuzzification**: Quá trình chuyển đổi kết quả mờ thành giá trị đầu ra rõ ràng.

### Sơ đồ hoạt động của hệ thống Fuzzy Logic

```
Đầu vào số → Fuzzification → Inference Engine → Defuzzification → Đầu ra số
```

## Cấu trúc mô đun FuzzyController

Mô đun `FuzzyController` được triển khai với các thành phần sau:

1. **Tập mờ và hàm thành viên**: Định nghĩa các tập mờ cho Error, ErrorChange, và Output.

2. **Bộ luật mờ**: Các luật IF-THEN xác định hành vi của hệ thống.

3. **Engine xử lý mờ**: Thực hiện quá trình fuzzification, inference, và defuzzification.

4. **Giao diện điều chỉnh**: Các phương thức để tùy chỉnh và tinh chỉnh hệ thống.

## Các tập mờ và hàm thành viên

### Tập mờ cho Error

Mô đun sử dụng 7 tập mờ cho Error:

```cpp
// Các nhãn cho tập mờ Error
#define ERROR_NB -3  // Negative Big (Âm Lớn)
#define ERROR_NM -2  // Negative Medium (Âm Vừa)
#define ERROR_NS -1  // Negative Small (Âm Nhỏ)
#define ERROR_ZE 0   // Zero (Không)
#define ERROR_PS 1   // Positive Small (Dương Nhỏ)
#define ERROR_PM 2   // Positive Medium (Dương Vừa)
#define ERROR_PB 3   // Positive Big (Dương Lớn)
```

### Tập mờ cho ErrorChange

Sử dụng 5 tập mờ cho ErrorChange (sự thay đổi của Error):

```cpp
// Các nhãn cho tập mờ ErrorChange
#define DERROR_NB -2  // Negative Big (Âm Lớn)
#define DERROR_NS -1  // Negative Small (Âm Nhỏ)
#define DERROR_ZE 0   // Zero (Không)
#define DERROR_PS 1   // Positive Small (Dương Nhỏ)
#define DERROR_PB 2   // Positive Big (Dương Lớn)
```

### Tập mờ cho Output

Sử dụng 7 tập mờ cho Output:

```cpp
// Các nhãn cho tập mờ Output
#define OUTPUT_NB -3  // Negative Big (Âm Lớn)
#define OUTPUT_NM -2  // Negative Medium (Âm Vừa)
#define OUTPUT_NS -1  // Negative Small (Âm Nhỏ)
#define OUTPUT_ZE 0   // Zero (Không)
#define OUTPUT_PS 1   // Positive Small (Dương Nhỏ)
#define OUTPUT_PM 2   // Positive Medium (Dương Vừa)
#define OUTPUT_PB 3   // Positive Big (Dương Lớn)
```

### Hàm thành viên tam giác (Triangle Membership Function)

```cpp
float FuzzyController::triangleMF(float x, float a, float b, float c) {
  if (x <= a || x >= c) return 0.0f;
  if (x == b) return 1.0f;
  if (x < b) return (x - a) / (b - a);
  return (c - x) / (c - b);
}
```

### Hàm thành viên hình thang (Trapezoidal Membership Function)

```cpp
float FuzzyController::trapezoidMF(float x, float a, float b, float c, float d) {
  if (x <= a || x >= d) return 0.0f;
  if (x >= b && x <= c) return 1.0f;
  if (x < b) return (x - a) / (b - a);
  return (d - x) / (d - c);
}
```

## Bộ luật mờ

Bộ luật mờ được định nghĩa trong một ma trận 7x5, mỗi ô đại diện cho một luật:

```cpp
// Ma trận luật mờ: Hàng = Error, Cột = ErrorChange
int8_t FuzzyController::ruleMatrix[7][5] = {
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

## Lớp FuzzyController

### Constructor

```cpp
FuzzyController::FuzzyController() {
  // Thiết lập các tham số mặc định
  setErrorRange(-100.0f, 100.0f);
  setDErrorRange(-50.0f, 50.0f);
  setOutputRange(-100.0f, 100.0f);
  
  prevError = 0.0f;
  outputGain = 1.0f;
}
```

### compute

```cpp
float FuzzyController::compute(float error, float dt) {
  // Giới hạn giá trị error
  error = constrain(error, errorMin, errorMax);
  
  // Tính toán errorChange
  float errorChange = (error - prevError) / dt;
  prevError = error;
  
  // Giới hạn giá trị errorChange
  errorChange = constrain(errorChange, dErrorMin, dErrorMax);
  
  // Quá trình Fuzzification
  float errorMembership[7] = {0};
  float dErrorMembership[5] = {0};
  
  calculateErrorMembership(error, errorMembership);
  calculateDErrorMembership(errorChange, dErrorMembership);
  
  // Quá trình Inference và Defuzzification
  float output = inferenceAndDefuzzify(errorMembership, dErrorMembership);
  
  // Áp dụng hệ số khuếch đại
  return output * outputGain;
}
```

### Phương thức setErrorRange

```cpp
void FuzzyController::setErrorRange(float min, float max) {
  errorMin = min;
  errorMax = max;
  
  // Tính toán các điểm phân chia cho các tập mờ Error
  errorStep = (max - min) / 6.0f;
  
  errorRanges[0] = min;
  errorRanges[1] = min + errorStep;
  errorRanges[2] = min + 2 * errorStep;
  errorRanges[3] = min + 3 * errorStep;  // Điểm giữa (0)
  errorRanges[4] = min + 4 * errorStep;
  errorRanges[5] = min + 5 * errorStep;
  errorRanges[6] = max;
}
```

### Phương thức setDErrorRange

```cpp
void FuzzyController::setDErrorRange(float min, float max) {
  dErrorMin = min;
  dErrorMax = max;
  
  // Tính toán các điểm phân chia cho các tập mờ DError
  dErrorStep = (max - min) / 4.0f;
  
  dErrorRanges[0] = min;
  dErrorRanges[1] = min + dErrorStep;
  dErrorRanges[2] = min + 2 * dErrorStep;  // Điểm giữa (0)
  dErrorRanges[3] = min + 3 * dErrorStep;
  dErrorRanges[4] = max;
}
```

### Phương thức setOutputRange

```cpp
void FuzzyController::setOutputRange(float min, float max) {
  outputMin = min;
  outputMax = max;
  
  // Tính toán các điểm phân chia cho các tập mờ Output
  outputStep = (max - min) / 6.0f;
  
  outputValues[0] = min;                 // OUTPUT_NB
  outputValues[1] = min + outputStep;    // OUTPUT_NM
  outputValues[2] = min + 2 * outputStep; // OUTPUT_NS
  outputValues[3] = min + 3 * outputStep; // OUTPUT_ZE
  outputValues[4] = min + 4 * outputStep; // OUTPUT_PS
  outputValues[5] = min + 5 * outputStep; // OUTPUT_PM
  outputValues[6] = max;                 // OUTPUT_PB
}
```

### Tính toán mức độ thành viên cho Error

```cpp
void FuzzyController::calculateErrorMembership(float error, float* membership) {
  // ERROR_NB (Negative Big)
  if (error <= errorRanges[0]) {
    membership[0] = 1.0f;
  } else if (error < errorRanges[1]) {
    membership[0] = (errorRanges[1] - error) / (errorRanges[1] - errorRanges[0]);
  } else {
    membership[0] = 0.0f;
  }
  
  // ERROR_NM (Negative Medium)
  membership[1] = triangleMF(error, errorRanges[0], errorRanges[1], errorRanges[2]);
  
  // ERROR_NS (Negative Small)
  membership[2] = triangleMF(error, errorRanges[1], errorRanges[2], errorRanges[3]);
  
  // ERROR_ZE (Zero)
  membership[3] = triangleMF(error, errorRanges[2], errorRanges[3], errorRanges[4]);
  
  // ERROR_PS (Positive Small)
  membership[4] = triangleMF(error, errorRanges[3], errorRanges[4], errorRanges[5]);
  
  // ERROR_PM (Positive Medium)
  membership[5] = triangleMF(error, errorRanges[4], errorRanges[5], errorRanges[6]);
  
  // ERROR_PB (Positive Big)
  if (error >= errorRanges[6]) {
    membership[6] = 1.0f;
  } else if (error > errorRanges[5]) {
    membership[6] = (error - errorRanges[5]) / (errorRanges[6] - errorRanges[5]);
  } else {
    membership[6] = 0.0f;
  }
}
```

### Tính toán mức độ thành viên cho DError

```cpp
void FuzzyController::calculateDErrorMembership(float dError, float* membership) {
  // DERROR_NB (Negative Big)
  if (dError <= dErrorRanges[0]) {
    membership[0] = 1.0f;
  } else if (dError < dErrorRanges[1]) {
    membership[0] = (dErrorRanges[1] - dError) / (dErrorRanges[1] - dErrorRanges[0]);
  } else {
    membership[0] = 0.0f;
  }
  
  // DERROR_NS (Negative Small)
  membership[1] = triangleMF(dError, dErrorRanges[0], dErrorRanges[1], dErrorRanges[2]);
  
  // DERROR_ZE (Zero)
  membership[2] = triangleMF(dError, dErrorRanges[1], dErrorRanges[2], dErrorRanges[3]);
  
  // DERROR_PS (Positive Small)
  membership[3] = triangleMF(dError, dErrorRanges[2], dErrorRanges[3], dErrorRanges[4]);
  
  // DERROR_PB (Positive Big)
  if (dError >= dErrorRanges[4]) {
    membership[4] = 1.0f;
  } else if (dError > dErrorRanges[3]) {
    membership[4] = (dError - dErrorRanges[3]) / (dErrorRanges[4] - dErrorRanges[3]);
  } else {
    membership[4] = 0.0f;
  }
}
```

### Phương thức Inference và Defuzzification

```cpp
float FuzzyController::inferenceAndDefuzzify(float* errorMembership, float* dErrorMembership) {
  float outputStrengths[7] = {0}; // Lưu trữ độ mạnh của mỗi đầu ra
  float totalStrength = 0;        // Tổng độ mạnh
  
  // Áp dụng các luật và tính toán độ mạnh của mỗi đầu ra
  for (int i = 0; i < 7; i++) {         // Lặp qua các tập Error
    for (int j = 0; j < 5; j++) {       // Lặp qua các tập DError
      // Lấy độ mạnh của luật = min(độ thành viên Error, độ thành viên DError)
      float ruleStrength = min(errorMembership[i], dErrorMembership[j]);
      
      if (ruleStrength > 0) {
        // Lấy chỉ số đầu ra từ ma trận luật
        int8_t outputIndex = ruleMatrix[i][j] + 3; // +3 vì OUTPUT_NB = -3
        
        // Cập nhật độ mạnh đầu ra (lấy max nếu có nhiều luật cho cùng đầu ra)
        outputStrengths[outputIndex] = max(outputStrengths[outputIndex], ruleStrength);
      }
    }
  }
  
  // Tính tổng trọng số
  float weightedSum = 0;
  totalStrength = 0;
  
  for (int i = 0; i < 7; i++) {
    weightedSum += outputValues[i] * outputStrengths[i];
    totalStrength += outputStrengths[i];
  }
  
  // Defuzzification bằng phương pháp trọng tâm
  // Nếu không có luật nào được kích hoạt, trả về 0
  if (totalStrength < 0.001f) return 0.0f;
  
  return weightedSum / totalStrength;
}
```

### Phương thức debug

```cpp
void FuzzyController::debug(float error, float dError) {
  Serial.println("===== Fuzzy Controller Debug =====");
  Serial.print("Error: "); Serial.println(error);
  Serial.print("DError: "); Serial.println(dError);
  
  // Tính toán và hiển thị thành viên cho Error
  float errorMembership[7] = {0};
  calculateErrorMembership(error, errorMembership);
  
  Serial.println("Error Membership:");
  Serial.print("NB: "); Serial.println(errorMembership[0]);
  Serial.print("NM: "); Serial.println(errorMembership[1]);
  Serial.print("NS: "); Serial.println(errorMembership[2]);
  Serial.print("ZE: "); Serial.println(errorMembership[3]);
  Serial.print("PS: "); Serial.println(errorMembership[4]);
  Serial.print("PM: "); Serial.println(errorMembership[5]);
  Serial.print("PB: "); Serial.println(errorMembership[6]);
  
  // Tính toán và hiển thị thành viên cho DError
  float dErrorMembership[5] = {0};
  calculateDErrorMembership(dError, dErrorMembership);
  
  Serial.println("DError Membership:");
  Serial.print("NB: "); Serial.println(dErrorMembership[0]);
  Serial.print("NS: "); Serial.println(dErrorMembership[1]);
  Serial.print("ZE: "); Serial.println(dErrorMembership[2]);
  Serial.print("PS: "); Serial.println(dErrorMembership[3]);
  Serial.print("PB: "); Serial.println(dErrorMembership[4]);
  
  // Tính toán đầu ra và hiển thị
  float output = inferenceAndDefuzzify(errorMembership, dErrorMembership);
  Serial.print("Final Output: "); Serial.println(output);
  Serial.print("Scaled Output: "); Serial.println(output * outputGain);
  Serial.println("================================");
}
```
