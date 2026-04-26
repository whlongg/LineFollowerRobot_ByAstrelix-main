# Tài liệu mô đun FuzzyController

## Tổng quan

Mô đun `FuzzyController` triển khai bộ điều khiển mờ (Fuzzy Logic Controller) cho robot dò đường. Điều khiển mờ là một phương pháp điều khiển dựa trên logic mờ, cho phép robot có phản ứng mượt mà hơn và thích nghi với các tình huống phức tạp hơn so với PID truyền thống.

## Tệp liên quan
- **FuzzyController.h**: Định nghĩa giao diện của lớp FuzzyController
- **FuzzyController.cpp**: Triển khai các phương thức

## Nguyên lý hoạt động

Bộ điều khiển mờ hoạt động thông qua ba bước chính:

1. **Mờ hóa (Fuzzification)**: Chuyển đổi giá trị đầu vào (error, deltaError) thành mức độ thành viên trong các tập mờ.
2. **Suy luận mờ (Fuzzy Inference)**: Áp dụng các luật suy luận để xác định mức độ kích hoạt của các tập mờ đầu ra.
3. **Giải mờ (Defuzzification)**: Chuyển đổi các tập mờ đầu ra thành giá trị điều khiển cụ thể.

## Các định nghĩa và tập mờ

### Tập mờ đầu vào
```cpp
enum FuzzySet { NL, NM, NS, Z, PS, PM, PL };
```
- **NL**: Negative Large (Âm lớn)
- **NM**: Negative Medium (Âm trung bình)
- **NS**: Negative Small (Âm nhỏ)
- **Z**: Zero (Không)
- **PS**: Positive Small (Dương nhỏ)
- **PM**: Positive Medium (Dương trung bình)
- **PL**: Positive Large (Dương lớn)

### Tập mờ đầu ra
```cpp
enum FuzzyOutput { HARD_LEFT, LEFT, SLIGHT_LEFT, STRAIGHT, SLIGHT_RIGHT, RIGHT, HARD_RIGHT };
```
- **HARD_LEFT**: Rẽ trái mạnh
- **LEFT**: Rẽ trái
- **SLIGHT_LEFT**: Rẽ trái nhẹ
- **STRAIGHT**: Đi thẳng
- **SLIGHT_RIGHT**: Rẽ phải nhẹ
- **RIGHT**: Rẽ phải
- **HARD_RIGHT**: Rẽ phải mạnh

### Mức độ thành viên
```cpp
struct MembershipDegree {
    float NL, NM, NS, Z, PS, PM, PL;
};
```
Cấu trúc này lưu trữ mức độ thành viên của một giá trị đầu vào trong từng tập mờ.

## Thuộc tính (Properties)

| Thuộc tính | Kiểu | Mô tả |
|------------|------|-------|
| `ruleTable` | FuzzyOutput[7][7] | Bảng luật suy luận 2 chiều (error × deltaError) |
| `inputSets` | struct | Thông số các tập mờ đầu vào (center và width của mỗi tập) |
| `outputCenters` | float[7] | Giá trị trung tâm của các tập mờ đầu ra |
| `inputMin`, `inputMax` | float | Phạm vi đầu vào |
| `outputMin`, `outputMax` | float | Phạm vi đầu ra |

## Phương thức (Methods)

### Constructor

```cpp
FuzzyController()
```

**Mô tả**: Khởi tạo bộ điều khiển mờ với các tham số mặc định.

**Ví dụ**:
```cpp
FuzzyController fuzzy;
```

### computeOutput

```cpp
int computeOutput(float error, float deltaError)
```

**Mô tả**: Tính toán đầu ra điều khiển dựa trên error và thay đổi error.

**Tham số**:
- `error`: Giá trị error hiện tại
- `deltaError`: Thay đổi của error (error hiện tại - error trước đó)

**Giá trị trả về**: Giá trị đầu ra điều khiển (số nguyên)

**Ví dụ**:
```cpp
float error = currentPosition - targetPosition;
float deltaError = error - lastError;
int output = fuzzy.computeOutput(error, deltaError);
```

### setInputRange

```cpp
void setInputRange(float min, float max)
```

**Mô tả**: Thiết lập phạm vi giá trị đầu vào.

**Tham số**:
- `min`: Giá trị đầu vào tối thiểu
- `max`: Giá trị đầu vào tối đa

**Ví dụ**:
```cpp
fuzzy.setInputRange(-100.0, 100.0);
```

### setOutputRange

```cpp
void setOutputRange(float min, float max)
```

**Mô tả**: Thiết lập phạm vi giá trị đầu ra.

**Tham số**:
- `min`: Giá trị đầu ra tối thiểu
- `max`: Giá trị đầu ra tối đa

**Ví dụ**:
```cpp
fuzzy.setOutputRange(-100.0, 100.0);
```

## Các phương thức private

### fuzzify

```cpp
MembershipDegree fuzzify(float val)
```

**Mô tả**: Chuyển đổi giá trị đầu vào thành mức độ thành viên trong các tập mờ.

### defuzzify

```cpp
int defuzzify(float outputStrength[7])
```

**Mô tả**: Chuyển đổi sức mạnh của các tập mờ đầu ra thành giá trị đầu ra cụ thể, sử dụng phương pháp trọng tâm (Center of Gravity).

### triangularMF

```cpp
float triangularMF(float x, float center, float width)
```

**Mô tả**: Tính toán hàm thành viên dạng tam giác.

## Chi tiết triển khai

### Bảng luật suy luận
Bộ điều khiển mờ sử dụng bảng luật 7×7 để ánh xạ cặp (error, deltaError) vào đầu ra. Bảng này được khai báo và khởi tạo trong constructor.

Ví dụ về một số luật:
- Nếu error là NL (Âm lớn) và deltaError là NL (Âm lớn) → HARD_RIGHT (Rẽ phải mạnh)
- Nếu error là Z (Không) và deltaError là Z (Không) → STRAIGHT (Đi thẳng)
- Nếu error là PL (Dương lớn) và deltaError là PL (Dương lớn) → HARD_LEFT (Rẽ trái mạnh)

### Mờ hóa (Fuzzification)
Quá trình mờ hóa chuyển đổi error và deltaError thành mức độ thành viên trong các tập mờ NL, NM, NS, Z, PS, PM, PL sử dụng hàm thành viên dạng tam giác:

```cpp
MembershipDegree fuzzify(float val) {
    MembershipDegree md;
    md.NL = triangularMF(val, inputSets.NL_center, inputSets.NL_width);
    md.NM = triangularMF(val, inputSets.NM_center, inputSets.NM_width);
    // ... tương tự cho các tập còn lại
    return md;
}
```

### Suy luận mờ (Fuzzy Inference)
Quá trình suy luận áp dụng bảng luật để xác định mức độ kích hoạt của các tập mờ đầu ra. Giá trị này dựa trên quy tắc MIN-MAX:

```cpp
float outputStrength[7] = {0};
for (int e = 0; e < 7; e++) {
    for (int de = 0; de < 7; de++) {
        // Tính mức độ kích hoạt của luật (e,de)
        float ruleStrength = min(errorMD.values[e], deltaErrorMD.values[de]);
        // Cập nhật mức độ kích hoạt của tập mờ đầu ra
        int outputIndex = ruleTable[e][de];
        outputStrength[outputIndex] = max(outputStrength[outputIndex], ruleStrength);
    }
}
```

### Giải mờ (Defuzzification)
Quá trình giải mờ sử dụng phương pháp trọng tâm (Center of Gravity) để chuyển đổi mức độ kích hoạt của các tập mờ đầu ra thành giá trị đầu ra cụ thể:

```cpp
int defuzzify(float outputStrength[7]) {
    float sumProduct = 0;
    float sumStrength = 0;
    
    // Tính tổng có trọng số
    for (int i = 0; i < 7; i++) {
        sumProduct += outputStrength[i] * outputCenters[i];
        sumStrength += outputStrength[i];
    }
    
    // Tránh chia cho 0
    if (sumStrength < 0.001f) return 0;
    
    // Tính giá trị trọng tâm
    float centroid = sumProduct / sumStrength;
    
    // Chuyển đổi về phạm vi đầu ra
    return map(centroid, -1.0f, 1.0f, outputMin, outputMax);
}
```

## Cách sử dụng

### Khởi tạo và thiết lập

```cpp
// Khởi tạo
FuzzyController fuzzy;

// Thiết lập phạm vi
fuzzy.setInputRange(-100.0, 100.0);
fuzzy.setOutputRange(-100.0, 100.0);
```

### Sử dụng trong vòng lặp điều khiển

```cpp
void loop() {
    // Đọc cảm biến và tính error
    float error = targetPosition - currentPosition;
    
    // Tính deltaError
    static float lastError = 0;
    float deltaError = error - lastError;
    lastError = error;
    
    // Tính đầu ra Fuzzy
    int output = fuzzy.computeOutput(error, deltaError);
    
    // Sử dụng đầu ra để điều khiển động cơ
    int leftSpeed = baseSpeed + output;
    int rightSpeed = baseSpeed - output;
    setMotorSpeeds(leftSpeed, rightSpeed);
}
```

### Kết hợp với PID trong điều khiển hybrid

```cpp
// Trong RobotController
int compute(float error, float dt) {
    // Tính đầu ra PID
    float pidOutput = pid.compute(error, dt);
    
    // Tính đầu ra Fuzzy
    float deltaError = (error - lastError) / dt;
    float fuzzyOutput = fuzzy.computeOutput(error, deltaError);
    
    // Kết hợp đầu ra theo tỷ lệ
    float combinedOutput = (pidWeight * pidOutput + fuzzyWeight * fuzzyOutput) / 
                          (pidWeight + fuzzyWeight);
    
    lastError = error;
    return round(combinedOutput);
}
```

## Ưu điểm của điều khiển mờ

1. **Mượt mà hơn**: Chuyển đổi giữa các hành động điều khiển mượt mà hơn so với PID.
2. **Dễ hiệu chỉnh trực quan**: Có thể điều chỉnh các luật và tập mờ một cách trực quan mà không cần hiểu biết sâu về lý thuyết điều khiển.
3. **Thích ứng tốt với phi tuyến**: Xử lý tốt các hệ thống phi tuyến và thay đổi đặc tính.
4. **Bền vững (robust)**: Ít nhạy cảm với nhiễu và có khả năng thích ứng tốt với các điều kiện khác nhau.

## Hiệu chỉnh bộ điều khiển mờ

### Điều chỉnh các tập mờ
- **Tập mờ đầu vào**: Điều chỉnh vị trí trung tâm và độ rộng của các tập để phản ánh phạm vi error và deltaError.
- **Tập mờ đầu ra**: Điều chỉnh giá trị trung tâm để thay đổi cường độ của hành động điều khiển.

### Điều chỉnh bảng luật
- Có thể thay đổi bảng luật để thay đổi hành vi của robot trong các tình huống khác nhau.
- Ví dụ, nếu robot phản ứng quá mạnh với error nhỏ, có thể điều chỉnh các luật liên quan đến NS (Negative Small) và PS (Positive Small).

### Điều chỉnh phạm vi
- **Phạm vi đầu vào**: Nên phản ánh phạm vi thực tế của error và deltaError.
- **Phạm vi đầu ra**: Nên tương ứng với phạm vi điều khiển động cơ.

## Lưu ý khi sử dụng

1. **Chi phí tính toán**: Bộ điều khiển mờ yêu cầu nhiều tính toán hơn PID, có thể ảnh hưởng đến hiệu suất trên các vi điều khiển có tài nguyên hạn chế.

2. **Cân bằng giữa PID và Fuzzy**: Trong điều khiển hybrid, cần cân bằng giữa tính chính xác của PID và tính mượt mà của Fuzzy thông qua các trọng số pidWeight và fuzzyWeight.

3. **Tinh chỉnh theo thực tế**: Cần điều chỉnh các tham số dựa trên thực tế đường đua và hành vi mong muốn của robot.

## Trường hợp cụ thể cho robot dò đường

Trong dự án này, các tham số đã được thiết lập:
```cpp
#define FUZZY_INPUT_MIN -100.0  // Phạm vi đầu vào tối thiểu
#define FUZZY_INPUT_MAX 100.0   // Phạm vi đầu vào tối đa
#define FUZZY_OUTPUT_MIN -100.0 // Phạm vi đầu ra tối thiểu
#define FUZZY_OUTPUT_MAX 100.0  // Phạm vi đầu ra tối đa
```

Trong chế độ hybrid, trọng số được thiết lập:
```cpp
#define PIDW 1.0  // Trọng số PID
#define FUZW 0.0  // Trọng số Fuzzy
```

Với FUZW = 0.0, điều này cho thấy mặc định chỉ sử dụng PID (không sử dụng Fuzzy). Để kích hoạt điều khiển hybrid, cần thiết lập FUZW > 0, ví dụ:
```cpp
#define PIDW 0.7  // 70% PID
#define FUZW 0.3  // 30% Fuzzy
```

Điều này sẽ tạo ra sự cân bằng giữa tính chính xác của PID và tính mượt mà của Fuzzy.
