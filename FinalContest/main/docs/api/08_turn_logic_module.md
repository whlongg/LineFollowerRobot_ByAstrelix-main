# Tài liệu mô đun TurnLogic

## Tổng quan

Mô đun `TurnLogic` phụ trách việc phát hiện các điều kiện giao lộ và quyết định hướng rẽ tiếp theo cho robot. Module này phân tích dữ liệu từ các cảm biến IR và dữ liệu màu sắc để xác định khi nào robot đến giao lộ và robot nên rẽ theo hướng nào dựa trên các quy tắc đã được lập trình.

## Tệp liên quan
- **TurnLogic.h**: Định nghĩa giao diện của lớp TurnLogic
- **TurnLogic.cpp**: Triển khai các phương thức phát hiện và xử lý giao lộ

## Các hướng rẽ

Module định nghĩa 6 loại hướng rẽ:

```cpp
enum TurnDirection {
  STRAIGHT = 0,  // Đi thẳng
  LEFT90   = 1,  // Rẽ trái 90 độ
  RIGHT90  = 2,  // Rẽ phải 90 độ
  LEFT45   = 3,  // Rẽ trái 45 độ
  RIGHT45  = 4,  // Rẽ phải 45 độ
  UTURN    = 5   // Quay đầu 180 độ
};
```

## Lớp TurnLogic

Lớp `TurnLogic` cung cấp các phương thức để phát hiện giao lộ và xác định hướng rẽ.

### Constructor

```cpp
TurnLogic()
```

**Mô tả**: Khởi tạo đối tượng TurnLogic.

**Chi tiết triển khai**:
- Thiết lập các biến trạng thái ban đầu
- Thiết lập các ngưỡng mặc định cho việc phát hiện giao lộ

**Ví dụ**:
```cpp
TurnLogic turnLogic;
```

### begin

```cpp
void begin(ReadIR *irSensor, ColorTracker *colorSensor)
```

**Mô tả**: Khởi tạo module TurnLogic với tham chiếu đến các cảm biến cần thiết.

**Tham số**:
- `irSensor`: Con trỏ đến đối tượng ReadIR để đọc dữ liệu từ cảm biến IR
- `colorSensor`: Con trỏ đến đối tượng ColorTracker để đọc dữ liệu màu sắc

**Chi tiết triển khai**:
- Lưu trữ tham chiếu đến các cảm biến
- Thiết lập các tham số phát hiện giao lộ

**Ví dụ**:
```cpp
ReadIR irSensor;
ColorTracker colorTracker;
TurnLogic turnLogic;

void setup() {
  irSensor.begin();
  colorTracker.begin();
  turnLogic.begin(&irSensor, &colorTracker);
}
```

### update

```cpp
bool update(int currentTime, float *sensorValues)
```

**Mô tả**: Cập nhật trạng thái giao lộ dựa trên dữ liệu cảm biến hiện tại.

**Tham số**:
- `currentTime`: Thời gian hiện tại (thường là giá trị từ millis())
- `sensorValues`: Mảng giá trị cảm biến IR

**Giá trị trả về**: `true` nếu phát hiện giao lộ, `false` nếu không

**Chi tiết triển khai**:
- Phân tích giá trị cảm biến để xác định mẫu giao lộ
- Cập nhật máy trạng thái nội bộ
- Xác định hướng rẽ dựa trên quy tắc đã thiết lập

**Ví dụ**:
```cpp
float sensorValues[SENSOR_COUNT];
irSensor.readSensors(sensorValues);

if (turnLogic.update(millis(), sensorValues)) {
  // Đã phát hiện giao lộ
  TurnDirection nextTurn = turnLogic.getNextTurn();
  turnHandler.startTurn(nextTurn);
}
```

### getNextTurn

```cpp
TurnDirection getNextTurn()
```

**Mô tả**: Lấy hướng rẽ tiếp theo được xác định.

**Giá trị trả về**: Hướng rẽ (TurnDirection) cho lần rẽ tiếp theo

**Ví dụ**:
```cpp
TurnDirection direction = turnLogic.getNextTurn();
switch (direction) {
  case LEFT90:
    Serial.println("Rẽ trái 90 độ");
    break;
  case RIGHT90:
    Serial.println("Rẽ phải 90 độ");
    break;
  case STRAIGHT:
    Serial.println("Đi thẳng");
    break;
  // Xử lý các hướng khác...
}
```

### resetIntersectionState

```cpp
void resetIntersectionState()
```

**Mô tả**: Đặt lại trạng thái phát hiện giao lộ.

**Chi tiết triển khai**:
- Xóa các biến trạng thái
- Đặt lại bộ đếm thời gian
- Chuẩn bị cho lần phát hiện giao lộ tiếp theo

**Ví dụ**:
```cpp
// Sau khi hoàn thành quá trình rẽ
turnLogic.resetIntersectionState();
```

### setTurnRule

```cpp
void setTurnRule(TurnRule rule)
```

**Mô tả**: Thiết lập quy tắc rẽ được sử dụng khi phát hiện giao lộ.

**Tham số**:
- `rule`: Quy tắc rẽ (TurnRule) để áp dụng

**Ví dụ**:
```cpp
// Thiết lập quy tắc luôn rẽ trái
turnLogic.setTurnRule(ALWAYS_LEFT);

// Hoặc thiết lập quy tắc theo hướng ưu tiên
turnLogic.setTurnRule(PRIORITY_STRAIGHT_LEFT_RIGHT);
```

### isIntersectionDetected

```cpp
bool isIntersectionDetected()
```

**Mô tả**: Kiểm tra xem giao lộ có đang được phát hiện không.

**Giá trị trả về**: `true` nếu đang phát hiện giao lộ, `false` nếu không

**Ví dụ**:
```cpp
if (turnLogic.isIntersectionDetected()) {
  // Đang ở giao lộ, tránh phát hiện lặp lại
  // ...
}
```

### setThresholds

```cpp
void setThresholds(float intersectionThreshold, int debounceTime)
```

**Mô tả**: Thiết lập các ngưỡng cho việc phát hiện giao lộ.

**Tham số**:
- `intersectionThreshold`: Ngưỡng để xác định giao lộ (từ 0.0 đến 1.0)
- `debounceTime`: Thời gian chờ giữa các lần phát hiện giao lộ (ms)

**Ví dụ**:
```cpp
// Điều chỉnh các ngưỡng phát hiện
turnLogic.setThresholds(0.7, 500);
```

### enableDebug

```cpp
void enableDebug(bool enable)
```

**Mô tả**: Bật/tắt chế độ debug cho module TurnLogic.

**Tham số**:
- `enable`: `true` để bật chế độ debug, `false` để tắt

**Ví dụ**:
```cpp
// Bật chế độ debug để theo dõi quá trình phát hiện
turnLogic.enableDebug(true);
```

## Chi tiết triển khai

### Phát hiện giao lộ

Quá trình phát hiện giao lộ sử dụng kết hợp các thông tin sau:

1. **Mẫu cảm biến IR**: 
```cpp
bool TurnLogic::detectIntersectionPattern(float *sensorValues) {
  // Giao lộ dạng T hoặc X: Nhiều cảm biến phát hiện đường line
  int lineDetected = 0;
  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (sensorValues[i] > lineThreshold) {
      lineDetected++;
    }
  }
  
  // Nếu 3 hoặc 4 cảm biến đều phát hiện đường line, có thể là giao lộ
  if (lineDetected >= SENSOR_COUNT - 1) {
    return true;
  }
  
  // Kiểm tra mẫu giao lộ chữ Y
  bool leftLines = (sensorValues[0] > lineThreshold && sensorValues[1] > lineThreshold);
  bool rightLines = (sensorValues[2] > lineThreshold && sensorValues[3] > lineThreshold);
  if (leftLines && rightLines) {
    return true;
  }
  
  return false;
}
```

2. **Dấu hiệu màu sắc**:
```cpp
bool TurnLogic::detectColorMarker() {
  if (colorSensor == nullptr) return false;
  
  uint8_t color = colorSensor->getColor();
  
  // Phát hiện màu xanh dương như một dấu hiệu của giao lộ
  return (color == COLOR_BLUE);
}
```

3. **Kết hợp và chống nhiễu**:
```cpp
bool TurnLogic::update(int currentTime, float *sensorValues) {
  // Kiểm tra chống dội (debounce)
  if (currentTime - lastIntersectionTime < debounceTimeMs) {
    return false;
  }
  
  // Kiểm tra mẫu cảm biến cho giao lộ
  bool irIndicatesIntersection = detectIntersectionPattern(sensorValues);
  
  // Kiểm tra dấu hiệu màu sắc
  bool colorIndicatesIntersection = detectColorMarker();
  
  // Phát hiện giao lộ nếu có đủ dấu hiệu
  bool intersectionDetected = irIndicatesIntersection || colorIndicatesIntersection;
  
  if (intersectionDetected) {
    if (!currentlyAtIntersection) {
      // Mới phát hiện giao lộ
      lastIntersectionTime = currentTime;
      currentlyAtIntersection = true;
      determineNextTurn(sensorValues);
      return true;
    }
  } else {
    // Không còn ở giao lộ
    currentlyAtIntersection = false;
  }
  
  return false;
}
```

### Xác định hướng rẽ

Quá trình xác định hướng rẽ dựa trên quy tắc được thiết lập:

```cpp
void TurnLogic::determineNextTurn(float *sensorValues) {
  // Mặc định là đi thẳng
  nextTurn = STRAIGHT;
  
  // Phân tích các tùy chọn có sẵn tại giao lộ
  bool canGoLeft = (sensorValues[0] > lineThreshold);
  bool canGoStraight = (sensorValues[1] > lineThreshold && sensorValues[2] > lineThreshold);
  bool canGoRight = (sensorValues[3] > lineThreshold);
  
  // Áp dụng quy tắc rẽ
  switch (currentRule) {
    case ALWAYS_LEFT:
      if (canGoLeft) nextTurn = LEFT90;
      else if (canGoStraight) nextTurn = STRAIGHT;
      else if (canGoRight) nextTurn = RIGHT90;
      else nextTurn = UTURN;
      break;
      
    case ALWAYS_RIGHT:
      if (canGoRight) nextTurn = RIGHT90;
      else if (canGoStraight) nextTurn = STRAIGHT;
      else if (canGoLeft) nextTurn = LEFT90;
      else nextTurn = UTURN;
      break;
      
    case PRIORITY_STRAIGHT_LEFT_RIGHT:
      if (canGoStraight) nextTurn = STRAIGHT;
      else if (canGoLeft) nextTurn = LEFT90;
      else if (canGoRight) nextTurn = RIGHT90;
      else nextTurn = UTURN;
      break;
      
    case RANDOM_CHOICE:
      // Chọn ngẫu nhiên một trong các hướng có thể đi
      int options = 0;
      if (canGoLeft) options++;
      if (canGoStraight) options++;
      if (canGoRight) options++;
      
      if (options > 0) {
        int choice = random(options);
        int current = 0;
        
        if (canGoLeft && current++ == choice) nextTurn = LEFT90;
        else if (canGoStraight && current++ == choice) nextTurn = STRAIGHT;
        else if (canGoRight) nextTurn = RIGHT90;
      } else {
        nextTurn = UTURN;
      }
      break;
  }
  
  if (debugEnabled) {
    Serial.print("Đã phát hiện giao lộ. Hướng rẽ: ");
    Serial.println(getTurnName(nextTurn));
  }
}
```

## Cách sử dụng

### Khởi tạo và cấu hình

```cpp
// Trong hàm setup()
ReadIR irSensor;
ColorTracker colorTracker;
TurnLogic turnLogic;

void setup() {
  // Khởi tạo các cảm biến
  irSensor.begin();
  colorTracker.begin();
  
  // Khởi tạo TurnLogic với tham chiếu đến các cảm biến
  turnLogic.begin(&irSensor, &colorTracker);
  
  // Cấu hình quy tắc rẽ
  turnLogic.setTurnRule(ALWAYS_LEFT);
  
  // Tùy chỉnh ngưỡng nếu cần
  turnLogic.setThresholds(0.7, 500);
}
```

### Phát hiện và xử lý giao lộ

```cpp
// Trong vòng lặp chính
void loop() {
  // Đọc giá trị cảm biến
  float sensorValues[SENSOR_COUNT];
  irSensor.readSensors(sensorValues);
  
  // Kiểm tra giao lộ
  if (turnLogic.update(millis(), sensorValues)) {
    // Đã phát hiện giao lộ, lấy hướng rẽ
    TurnDirection nextTurn = turnLogic.getNextTurn();
    
    // Thực hiện rẽ
    turnHandler.startTurn(nextTurn);
    
    // Có thể cần lưu trữ thông tin rẽ cho bản đồ
    updateMap(nextTurn);
  }
  
  // Tiếp tục điều khiển...
}
```

### Sử dụng với giải thuật giải mê cung

```cpp
// Khai báo struct để lưu trữ thông tin nút trong bản đồ
struct MapNode {
  bool visited;
  bool hasLeft;
  bool hasStraight;
  bool hasRight;
};

// Mảng lưu trữ bản đồ (giả sử tối đa 50 nút)
MapNode maze[50];
int currentNode = 0;
int nodeCount = 1;  // Bắt đầu với nút gốc

// Hàm cập nhật bản đồ khi phát hiện giao lộ
void updateMap(TurnDirection turn) {
  // Lấy thông tin về các hướng có thể đi từ giao lộ hiện tại
  bool canGoLeft = /* ... */;
  bool canGoStraight = /* ... */;
  bool canGoRight = /* ... */;
  
  // Cập nhật thông tin nút hiện tại
  maze[currentNode].visited = true;
  maze[currentNode].hasLeft = canGoLeft;
  maze[currentNode].hasStraight = canGoStraight;
  maze[currentNode].hasRight = canGoRight;
  
  // Dựa vào hướng rẽ, cập nhật nút hiện tại
  int nextNode;
  switch (turn) {
    case LEFT90:
      nextNode = findOrCreateNode(currentNode, "left");
      break;
    case STRAIGHT:
      nextNode = findOrCreateNode(currentNode, "straight");
      break;
    case RIGHT90:
      nextNode = findOrCreateNode(currentNode, "right");
      break;
    default:
      // Xử lý các trường hợp khác
      break;
  }
  
  currentNode = nextNode;
}

// Giải thuật chọn hướng rẽ tiếp theo dựa trên bản đồ
TurnDirection decideTurnBasedOnMap() {
  // Chọn hướng chưa đi trước
  if (maze[currentNode].hasLeft && !nodeVisited(currentNode, "left")) {
    return LEFT90;
  }
  if (maze[currentNode].hasStraight && !nodeVisited(currentNode, "straight")) {
    return STRAIGHT;
  }
  if (maze[currentNode].hasRight && !nodeVisited(currentNode, "right")) {
    return RIGHT90;
  }
  
  // Nếu tất cả đã đi qua, tìm đường ngắn nhất đến nút chưa khám phá
  // ...
  
  // Mặc định quay đầu nếu không tìm thấy đường nào
  return UTURN;
}
```

## Lưu ý khi sử dụng

1. **Phát hiện giao lộ**:
   - Thời gian chống dội (debounce) ảnh hưởng đến khả năng phát hiện giao lộ liên tiếp
   - Ngưỡng phát hiện cần được điều chỉnh dựa trên đặc điểm đường đua cụ thể

2. **Xác định hướng rẽ**:
   - Quy tắc rẽ mặc định có thể không phù hợp cho mọi loại đường đua
   - Nên cân nhắc kết hợp với thuật toán giải mê cung cho đường đua phức tạp

3. **Tương tác với TurnHandler**:
   - Đảm bảo rằng TurnLogic và TurnHandler hoạt động đồng bộ
   - Sau khi hoàn thành rẽ, cần đặt lại trạng thái giao lộ để chuẩn bị cho lần phát hiện tiếp theo

4. **Vị trí cảm biến**:
   - Vị trí và khoảng cách giữa các cảm biến IR ảnh hưởng đến khả năng phát hiện giao lộ
   - Một bố trí tối ưu sẽ giúp phát hiện các loại giao lộ khác nhau

## Nâng cấp khả năng

1. **Phát hiện giao lộ nâng cao**:
   - Hỗ trợ phát hiện nhiều loại giao lộ hơn (T, Y, X, +)
   - Thêm khả năng phân biệt giữa giao lộ thực sự và đường cong

2. **Chiến lược giải mê cung**:
   - Triển khai các thuật toán như Left-Hand Rule, Right-Hand Rule
   - Thêm khả năng xây dựng và lưu trữ bản đồ đường đua

3. **Học tập và thích nghi**:
   - Thêm khả năng ghi nhớ và học từ các lần chạy trước
   - Tối ưu hóa lộ trình dựa trên dữ liệu đã thu thập

4. **Xử lý tình huống đặc biệt**:
   - Thêm khả năng phát hiện và xử lý đường cụt
   - Xử lý trường hợp mất đường line hoàn toàn

## Ví dụ triển khai nâng cao

### Phát hiện giao lộ dựa trên mẫu cụ thể

```cpp
bool TurnLogic::detectSpecificIntersection(float *sensorValues) {
  // Mẫu cảm biến cho giao lộ hình chữ T
  bool isT_Junction = (!sensorValues[0] && sensorValues[1] && sensorValues[2] && !sensorValues[3]) || 
                      (!sensorValues[0] && !sensorValues[1] && sensorValues[2] && sensorValues[3]) || 
                      (sensorValues[0] && sensorValues[1] && !sensorValues[2] && !sensorValues[3]);
  
  // Mẫu cảm biến cho giao lộ hình X
  bool isX_Junction = (sensorValues[0] && sensorValues[1] && sensorValues[2] && sensorValues[3]);
  
  // Mẫu cảm biến cho giao lộ hình +
  bool isCross_Junction = (!sensorValues[0] && sensorValues[1] && sensorValues[2] && !sensorValues[3]);
  
  // Phát hiện và phân loại giao lộ
  if (isT_Junction) {
    intersectionType = T_JUNCTION;
    return true;
  } else if (isX_Junction) {
    intersectionType = X_JUNCTION;
    return true;
  } else if (isCross_Junction) {
    intersectionType = CROSS_JUNCTION;
    return true;
  }
  
  return false;
}
```

### Thuật toán giải mê cung tối ưu (Flood Fill)

```cpp
// Điều chỉnh hướng rẽ dựa trên thuật toán Flood Fill
TurnDirection optimizePath() {
  // Mảng lưu giá trị khoảng cách đến đích
  static int distanceMap[10][10];
  static bool initialized = false;
  
  if (!initialized) {
    // Khởi tạo bản đồ khoảng cách với giá trị tối đa
    for (int i = 0; i < 10; i++) {
      for (int j = 0; j < 10; j++) {
        distanceMap[i][j] = 255;
      }
    }
    
    // Đặt vị trí đích có khoảng cách 0
    distanceMap[5][5] = 0;
    initialized = true;
  }
  
  // Lấy vị trí hiện tại (để đơn giản, giả sử chúng ta có thể xác định vị trí)
  int currentX = /* ... */;
  int currentY = /* ... */;
  
  // Tìm ô hàng xóm có giá trị nhỏ nhất
  int minDist = 255;
  TurnDirection bestDirection = STRAIGHT;
  
  // Kiểm tra các hướng có thể đi
  if (canGoForward) {
    int forwardX = currentX + /* ... */;
    int forwardY = currentY + /* ... */;
    if (distanceMap[forwardX][forwardY] < minDist) {
      minDist = distanceMap[forwardX][forwardY];
      bestDirection = STRAIGHT;
    }
  }
  
  if (canGoLeft) {
    int leftX = currentX + /* ... */;
    int leftY = currentY + /* ... */;
    if (distanceMap[leftX][leftY] < minDist) {
      minDist = distanceMap[leftX][leftY];
      bestDirection = LEFT90;
    }
  }
  
  if (canGoRight) {
    int rightX = currentX + /* ... */;
    int rightY = currentY + /* ... */;
    if (distanceMap[rightX][rightY] < minDist) {
      minDist = distanceMap[rightX][rightY];
      bestDirection = RIGHT90;
    }
  }
  
  return bestDirection;
}
```
