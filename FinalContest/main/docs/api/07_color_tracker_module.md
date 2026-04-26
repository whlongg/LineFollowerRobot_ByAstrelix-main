# Tài liệu mô đun ColorTracker

## Tổng quan

Mô đun `ColorTracker` quản lý việc đọc và xử lý dữ liệu từ cảm biến màu TCS34725, cho phép robot phát hiện các màu sắc khác nhau trên đường đua. Module này hỗ trợ phát hiện màu đen, trắng, xanh dương (checkpoint), và xanh lá (chế độ tốc độ cao), với khả năng hiệu chuẩn để cải thiện độ chính xác.

## Tệp liên quan
- **ColorTracker.h**: Định nghĩa giao diện của lớp ColorTracker
- **ColorTracker.cpp**: Triển khai các phương thức xử lý màu sắc

## Các mã màu

Module định nghĩa 4 loại màu cơ bản:

```cpp
#define COLOR_WHITE 0  // Màu trắng (nền đường đua)
#define COLOR_BLACK 1  // Màu đen (đường line)
#define COLOR_BLUE  2  // Màu xanh dương (checkpoint)
#define COLOR_GREEN 3  // Màu xanh lá (chế độ tốc độ cao)
```

## Lớp ColorTracker

Lớp `ColorTracker` cung cấp các phương thức để đọc, phân tích và hiệu chuẩn cảm biến màu.

### Constructor

```cpp
ColorTracker(uint16_t readInterval = 25)
```

**Mô tả**: Khởi tạo đối tượng ColorTracker với chu kỳ đọc tùy chỉnh.

**Tham số**:
- `readInterval`: Khoảng thời gian giữa các lần đọc cảm biến (ms, mặc định: 25ms)

**Chi tiết triển khai**:
- Cấu hình cảm biến TCS34725 với thời gian tích hợp 24ms và độ khuếch đại 16x
- Thiết lập chu kỳ đọc cảm biến
- Khởi tạo các biến lưu trữ trạng thái

**Ví dụ**:
```cpp
// Tạo đối tượng với chu kỳ đọc mặc định (25ms)
ColorTracker colorTracker;

// Hoặc tùy chỉnh chu kỳ đọc (50ms)
ColorTracker colorTracker(50);
```

### begin

```cpp
bool begin()
```

**Mô tả**: Khởi tạo cảm biến màu.

**Giá trị trả về**: `true` nếu khởi tạo thành công, `false` nếu thất bại

**Chi tiết triển khai**:
- Khởi tạo cảm biến TCS34725
- Thiết lập các ngưỡng mặc định cho việc phát hiện màu
- Thử tải thông số hiệu chuẩn từ EEPROM nếu có

**Ví dụ**:
```cpp
if (!colorTracker.begin()) {
  Serial.println("CẢNH BÁO: Không thể khởi tạo cảm biến màu!");
  // Xử lý lỗi...
}
```

### update

```cpp
bool update()
```

**Mô tả**: Cập nhật giá trị từ cảm biến nếu đã đến thời điểm đọc.

**Giá trị trả về**: `true` nếu giá trị được cập nhật, `false` nếu chưa đến thời điểm đọc

**Chi tiết triển khai**:
- Kiểm tra xem đã đến thời điểm đọc cảm biến chưa
- Đọc giá trị RGB và Clear từ cảm biến
- Phân loại màu dựa trên giá trị đọc được
- Cập nhật thông tin về màu đã phát hiện và trạng thái tốc độ cao

**Ví dụ**:
```cpp
if (colorTracker.update()) {
  // Dữ liệu mới đã được cập nhật
  uint8_t color = colorTracker.getColor();
  // Xử lý màu...
}
```

### getColor

```cpp
uint8_t getColor()
```

**Mô tả**: Lấy giá trị màu hiện tại.

**Giá trị trả về**: Mã màu (0: trắng, 1: đen, 2: xanh dương, 3: xanh lá)

**Ví dụ**:
```cpp
uint8_t color = colorTracker.getColor();
switch (color) {
  case COLOR_WHITE: 
    Serial.println("Đã phát hiện màu TRẮNG");
    break;
  case COLOR_BLACK: 
    Serial.println("Đã phát hiện màu ĐEN");
    break;
  case COLOR_BLUE: 
    Serial.println("Đã phát hiện màu XANH DƯƠNG");
    break;
  case COLOR_GREEN: 
    Serial.println("Đã phát hiện màu XANH LÁ");
    break;
}
```

### isHighSpeedMode

```cpp
bool isHighSpeedMode()
```

**Mô tả**: Kiểm tra xem có đang ở chế độ tốc độ cao hay không.

**Giá trị trả về**: `true` nếu là chế độ tốc độ cao (đã phát hiện màu xanh lá), `false` nếu không

**Ví dụ**:
```cpp
if (colorTracker.isHighSpeedMode()) {
  // Chuyển sang chế độ tốc độ cao
  baseSpeed = HIGH_SPEED;
} else {
  // Sử dụng tốc độ thông thường
  baseSpeed = NORMAL_SPEED;
}
```

### getResult

```cpp
bool getResult(uint8_t *color, bool *highSpeedMode, unsigned long *timestamp)
```

**Mô tả**: Lấy kết quả hiện tại bao gồm màu, chế độ tốc độ và thời gian.

**Tham số**:
- `color`: Con trỏ đến biến lưu mã màu
- `highSpeedMode`: Con trỏ đến biến lưu trạng thái tốc độ cao
- `timestamp`: Con trỏ đến biến lưu thời gian

**Giá trị trả về**: `true` nếu giá trị là mới nhất, `false` nếu chưa cập nhật

**Ví dụ**:
```cpp
uint8_t colorValue;
bool speedMode;
unsigned long timestamp;

if (colorTracker.getResult(&colorValue, &speedMode, &timestamp)) {
  // Dữ liệu mới, xử lý...
  if (speedMode) {
    Serial.println("Đang ở chế độ tốc độ cao");
  }
}
```

### setThresholds

```cpp
void setThresholds(uint16_t minWhite = 4000, 
                  uint16_t maxBlack = 800,
                  float blueThreshold = 1.2,
                  float greenThreshold = 1.2)
```

**Mô tả**: Thiết lập các ngưỡng cho việc phát hiện màu.

**Tham số**:
- `minWhite`: Ngưỡng tối thiểu để xác định màu trắng (giá trị clear)
- `maxBlack`: Ngưỡng tối đa để xác định màu đen (giá trị clear)
- `blueThreshold`: Ngưỡng tỉ lệ để xác định màu xanh dương
- `greenThreshold`: Ngưỡng tỉ lệ để xác định màu xanh lá

**Ví dụ**:
```cpp
// Điều chỉnh ngưỡng phát hiện
colorTracker.setThresholds(3500, 600, 1.3, 1.3);
```

### getRawValues

```cpp
void getRawValues(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
```

**Mô tả**: Lấy các giá trị RGB và Clear hiện tại.

**Tham số**:
- `r`: Con trỏ đến biến lưu giá trị đỏ
- `g`: Con trỏ đến biến lưu giá trị xanh lá
- `b`: Con trỏ đến biến lưu giá trị xanh dương
- `c`: Con trỏ đến biến lưu giá trị clear

**Ví dụ**:
```cpp
uint16_t r, g, b, c;
colorTracker.getRawValues(&r, &g, &b, &c);
Serial.print("R:");
Serial.print(r);
Serial.print(" G:");
Serial.print(g);
Serial.print(" B:");
Serial.print(b);
Serial.print(" C:");
Serial.println(c);
```

### getNormalizedValues

```cpp
void getNormalizedValues(uint8_t *r, uint8_t *g, uint8_t *b)
```

**Mô tả**: Lấy các giá trị RGB chuẩn hóa (0-255).

**Tham số**:
- `r`: Con trỏ đến biến lưu giá trị đỏ chuẩn hóa
- `g`: Con trỏ đến biến lưu giá trị xanh lá chuẩn hóa
- `b`: Con trỏ đến biến lưu giá trị xanh dương chuẩn hóa

**Ví dụ**:
```cpp
uint8_t r, g, b;
colorTracker.getNormalizedValues(&r, &g, &b);
Serial.print("R(normalized):");
Serial.print(r);
Serial.print(" G:");
Serial.print(g);
Serial.print(" B:");
Serial.println(b);
```

### Hiệu chuẩn màu sắc

```cpp
void calibrateBlueColor()
void calibrateGreenColor()
```

**Mô tả**: Hiệu chuẩn màu xanh dương/xanh lá bằng mẫu hiện tại.

**Chi tiết triển khai**:
- Đọc giá trị RGB từ cảm biến
- Chuyển đổi sang không gian màu HSV và Lab
- Lưu trữ thông số màu để sử dụng trong phân loại

**Ví dụ**:
```cpp
// Hiệu chuẩn màu xanh dương
Serial.println("Đặt mẫu màu XANH DƯƠNG vào cảm biến");
// Đợi người dùng đặt mẫu
delay(3000);
colorTracker.calibrateBlueColor();
Serial.println("Đã hiệu chuẩn màu XANH DƯƠNG");

// Tiếp tục với màu xanh lá
Serial.println("Đặt mẫu màu XANH LÁ vào cảm biến");
delay(3000);
colorTracker.calibrateGreenColor();
Serial.println("Đã hiệu chuẩn màu XANH LÁ");
```

### Lưu và đọc hiệu chuẩn

```cpp
void saveCalibrationToEEPROM()
bool loadCalibrationFromEEPROM()
bool isCalibrated() const
```

**Mô tả**: Các phương thức để lưu, đọc và kiểm tra trạng thái hiệu chuẩn.

**Ví dụ**:
```cpp
// Kiểm tra và thực hiện hiệu chuẩn
if (!colorTracker.isCalibrated()) {
  // Thực hiện hiệu chuẩn
  colorTracker.calibrateBlueColor();
  colorTracker.calibrateGreenColor();
  
  // Lưu thông số hiệu chuẩn
  colorTracker.saveCalibrationToEEPROM();
} else {
  Serial.println("Đã tải thông số hiệu chuẩn từ EEPROM");
}
```

## Chi tiết triển khai

### Phân loại màu

Quá trình phân loại màu được thực hiện trong phương thức `classifyColor()`:

```cpp
void ColorTracker::classifyColor() {
  // Mặc định là màu TRẮNG
  detectedColor = COLOR_WHITE;
  highSpeedMode = false;
  
  // Nhận diện màu ĐEN (cường độ ánh sáng thấp)
  if (c < maxBlackThreshold) {
    detectedColor = COLOR_BLACK;
    return;
  }
  
  // Nhận diện màu TRẮNG (cường độ ánh sáng cao)
  if (c > minWhiteThreshold) {
    detectedColor = COLOR_WHITE;
    return;
  }
  
  // Chuyển đổi sang HSV và Lab để phân tích
  float r_norm = (float)r / c;
  float g_norm = (float)g / c;
  float b_norm = (float)b / c;
  
  float h, s, v;
  rgbToHsv(r_norm, g_norm, b_norm, &h, &s, &v);
  
  float l, a, b_lab;
  rgbToLab(r_norm, g_norm, b_norm, &l, &a, &b_lab);
  
  // Kiểm tra màu XANH DƯƠNG dựa trên hiệu chuẩn
  float blueColorDistance = colorDistance(
    l, a, b_lab,
    colorParams.blueL, colorParams.blueA, colorParams.blueB
  );
  
  // Kiểm tra màu XANH LÁ dựa trên hiệu chuẩn
  float greenColorDistance = colorDistance(
    l, a, b_lab,
    colorParams.greenL, colorParams.greenA, colorParams.greenB
  );
  
  // Xác định màu dựa trên khoảng cách màu gần nhất
  if (blueColorDistance < colorParams.labTolerance && 
      blueColorDistance < greenColorDistance) {
    detectedColor = COLOR_BLUE;
  } else if (greenColorDistance < colorParams.labTolerance) {
    detectedColor = COLOR_GREEN;
    highSpeedMode = true;
  }
}
```

### Chuyển đổi không gian màu

Module hỗ trợ chuyển đổi giữa các không gian màu để cải thiện độ chính xác:

1. **RGB sang HSV**:
```cpp
void ColorTracker::rgbToHsv(float r, float g, float b, float *h, float *s, float *v) {
  float max_val = max(r, max(g, b));
  float min_val = min(r, min(g, b));
  float delta = max_val - min_val;
  
  *v = max_val;
  *s = (max_val > 0.0f) ? (delta / max_val) : 0.0f;
  
  if (delta < 0.001f) {
    *h = 0.0f; // Không có sắc độ với giá trị monochrome
  } else if (max_val == r) {
    *h = 60.0f * fmod(((g - b) / delta), 6.0f);
  } else if (max_val == g) {
    *h = 60.0f * (((b - r) / delta) + 2.0f);
  } else { // max_val == b
    *h = 60.0f * (((r - g) / delta) + 4.0f);
  }
  
  if (*h < 0.0f) *h += 360.0f;
}
```

2. **RGB sang Lab**:
```cpp
void ColorTracker::rgbToLab(float r, float g, float b, float *l, float *a, float *b_out) {
  float x, y, z;
  rgbToXyz(r, g, b, &x, &y, &z);
  xyzToLab(x, y, z, l, a, b_out);
}
```

### Tính khoảng cách màu

```cpp
float ColorTracker::colorDistance(float l1, float a1, float b1, float l2, float a2, float b2) {
  // Tính khoảng cách Euclidean trong không gian Lab
  return sqrt(pow(l1 - l2, 2) + pow(a1 - a2, 2) + pow(b1 - b2, 2));
}
```

## Cách sử dụng

### Khởi tạo và hiệu chuẩn

```cpp
// Trong hàm setup()
ColorTracker colorTracker(25); // Đọc mỗi 25ms

if (!colorTracker.begin()) {
  Serial.println("Lỗi: Không thể khởi tạo cảm biến màu!");
  // Xử lý lỗi...
} else {
  // Kiểm tra và thực hiện hiệu chuẩn nếu cần
  if (!colorTracker.isCalibrated()) {
    // Hướng dẫn hiệu chuẩn màu xanh dương
    Serial.println("Đặt MẪU MÀU XANH DƯƠNG vào cảm biến");
    waitForButtonPress();
    colorTracker.calibrateBlueColor();
    
    // Hướng dẫn hiệu chuẩn màu xanh lá
    Serial.println("Đặt MẪU MÀU XANH LÁ vào cảm biến");
    waitForButtonPress();
    colorTracker.calibrateGreenColor();
    
    // Lưu thông số hiệu chuẩn
    colorTracker.saveCalibrationToEEPROM();
  }
}
```

### Cập nhật và xử lý màu sắc

```cpp
// Trong vòng lặp chính
if (colorTracker.update()) {
  uint8_t colorValue = colorTracker.getColor();
  bool isHighSpeed = colorTracker.isHighSpeedMode();
  
  // Xử lý màu đã phát hiện
  switch (colorValue) {
    case COLOR_BLUE:
      // Xử lý checkpoint xanh dương
      turnHandler.enableCheckpointMode(true);
      break;
      
    case COLOR_GREEN:
      // Xử lý chế độ tốc độ cao
      baseSpeed = (isHighSpeed) ? HIGH_SPEED : BASE_SPEED;
      break;
  }
}
```

### Đọc giá trị màu chi tiết

```cpp
// Đọc giá trị thô
uint16_t r, g, b, c;
colorTracker.getRawValues(&r, &g, &b, &c);

// Tính tỷ lệ chuẩn hóa
float r_norm = 0, g_norm = 0, b_norm = 0;
if (c > 0) {
  r_norm = (float)r / c;
  g_norm = (float)g / c;
  b_norm = (float)b / c;
}

// Hiển thị thông tin
Serial.print("R:");
Serial.print(r);
Serial.print("(");
Serial.print(r_norm, 3);
Serial.print(") G:");
Serial.print(g);
Serial.print("(");
Serial.print(g_norm, 3);
Serial.print(") B:");
Serial.print(b);
Serial.print("(");
Serial.print(b_norm, 3);
Serial.print(") C:");
Serial.println(c);
```

## Lưu ý khi sử dụng

1. **Hiệu chuẩn màu sắc**:
   - Hiệu chuẩn là bước quan trọng để nhận diện chính xác các màu
   - Nên thực hiện hiệu chuẩn trong điều kiện ánh sáng tương tự với điều kiện đường đua
   - Lưu thông số hiệu chuẩn vào EEPROM để tránh cần hiệu chuẩn lại mỗi khi khởi động

2. **Thời gian đọc cảm biến**:
   - Tham số `readInterval` ảnh hưởng đến tần suất đọc cảm biến
   - Giá trị nhỏ hơn (10-15ms) cho phản ứng nhanh hơn nhưng tăng tải cho CPU
   - Giá trị lớn hơn (30-50ms) giảm tải CPU nhưng có thể bỏ lỡ checkpoint ngắn

3. **Cấu hình cảm biến**:
   - Thời gian tích hợp (integration time) và độ khuếch đại (gain) ảnh hưởng đến độ nhạy
   - Giá trị mặc định (24ms, 16x) phù hợp cho hầu hết các tình huống
   - Có thể cần điều chỉnh dựa trên điều kiện ánh sáng cụ thể

4. **Vị trí cảm biến**:
   - Cảm biến nên được đặt gần bề mặt đường đua (~5-10mm) để có kết quả tốt nhất
   - Tránh ánh sáng trực tiếp từ bên ngoài để giảm nhiễu

## Nâng cấp khả năng

1. **Thêm màu sắc**:
   - Mở rộng phát hiện thêm màu đỏ, vàng để xử lý các loại đường đua phức tạp hơn
   - Thêm các hằng số màu và phương thức hiệu chuẩn tương ứng

2. **Cải thiện thuật toán nhận diện**:
   - Áp dụng machine learning để phân loại màu chính xác hơn
   - Sử dụng k-nearest neighbors với mẫu đã hiệu chuẩn

3. **Theo dõi lịch sử màu**:
   - Lưu trữ lịch sử phát hiện màu để tránh các phát hiện sai do nhiễu
   - Sử dụng bộ lọc trung vị để ổn định kết quả

4. **Động thái thích ứng**:
   - Tự động điều chỉnh ngưỡng dựa trên điều kiện ánh sáng
   - Cập nhật thời gian tích hợp và độ khuếch đại dựa trên giá trị clear

## Ví dụ triển khai nâng cao

### Nhận diện đường line không đồng nhất

```cpp
struct LineColorProfile {
  uint16_t clearMin;
  uint16_t clearMax;
  float rNorm, gNorm, bNorm;
  float tolerance;
};

LineColorProfile lineProfiles[3]; // Lưu trữ 3 profile màu line khác nhau

// Thêm phương thức hiệu chuẩn profile line
void calibrateLineProfile(int profileIndex) {
  // Đọc giá trị màu hiện tại
  uint16_t r, g, b, c;
  getRawValues(&r, &g, &b, &c);
  
  // Lưu trữ profile
  lineProfiles[profileIndex].clearMin = c * 0.85;
  lineProfiles[profileIndex].clearMax = c * 1.15;
  lineProfiles[profileIndex].rNorm = (float)r / c;
  lineProfiles[profileIndex].gNorm = (float)g / c;
  lineProfiles[profileIndex].bNorm = (float)b / c;
  lineProfiles[profileIndex].tolerance = 0.15;
}

// Trong phương thức classifyColor()
bool isLine = false;
for (int i = 0; i < 3; i++) {
  LineColorProfile &profile = lineProfiles[i];
  if (c >= profile.clearMin && c <= profile.clearMax) {
    float r_norm = (float)r / c;
    float g_norm = (float)g / c;
    float b_norm = (float)b / c;
    
    float colorDist = sqrt(
      pow(r_norm - profile.rNorm, 2) +
      pow(g_norm - profile.gNorm, 2) +
      pow(b_norm - profile.bNorm, 2)
    );
    
    if (colorDist < profile.tolerance) {
      isLine = true;
      break;
    }
  }
}

if (isLine) {
  detectedColor = COLOR_BLACK;
}
```

### Phát hiện giao lộ dựa trên màu

```cpp
bool detectColorIntersection() {
  // Lưu trữ lịch sử phát hiện màu xanh dương
  static bool blueHistory[5] = {false, false, false, false, false};
  static int historyIndex = 0;
  
  // Cập nhật lịch sử
  blueHistory[historyIndex] = (getColor() == COLOR_BLUE);
  historyIndex = (historyIndex + 1) % 5;
  
  // Đếm số lần phát hiện màu xanh dương trong lịch sử
  int blueCount = 0;
  for (int i = 0; i < 5; i++) {
    if (blueHistory[i]) blueCount++;
  }
  
  // Nếu phát hiện màu xanh dương ít nhất 3/5 lần gần đây, có thể là giao lộ
  return (blueCount >= 3);
}
```
