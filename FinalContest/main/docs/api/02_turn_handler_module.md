# Tài liệu mô đun TurnHandler

## Tổng quan

Mô đun `TurnHandler` quản lý việc thực hiện rẽ của robot theo các hướng khác nhau. Mô đun này sử dụng máy trạng thái (state machine) không chặn để đảm bảo robot vẫn phản ứng với môi trường trong quá trình rẽ.

## Tệp liên quan
- **TurnHandler.h**: Định nghĩa giao diện (interface) của lớp TurnHandler
- **TurnHandler.cpp**: Triển khai (implementation) các phương thức

## Phụ thuộc
- **TurnLogic**: Cung cấp logic dự đoán hướng rẽ
- **MotorControl**: Điều khiển động cơ

## Trạng thái của máy trạng thái

Máy trạng thái xử lý quá trình rẽ qua 4 trạng thái:

```cpp
enum TurnState {
  TURN_IDLE,            // Không rẽ
  TURN_PREPARE,         // Chuẩn bị rẽ (giảm tốc)
  TURN_EXECUTING,       // Đang thực hiện rẽ
  TURN_FINISHING        // Hoàn thành rẽ
}
```

## Thuộc tính (Properties)

| Thuộc tính | Kiểu | Mô tả |
|------------|------|-------|
| `motor` | MotorControl* | Con trỏ đến đối tượng điều khiển động cơ |
| `turnLogic` | TurnLogic* | Con trỏ đến đối tượng logic rẽ |
| `checkpointMode` | bool | Trạng thái chế độ checkpoint (true: luôn rẽ theo hướng mặc định) |
| `defaultDirection` | TurnDirection | Hướng rẽ mặc định khi gặp giao lộ |
| `turnSpeed` | int | Tốc độ khi rẽ |
| `baseSpeed` | int | Tốc độ cơ bản |
| `currentState` | TurnState | Trạng thái hiện tại của máy trạng thái |
| `activeDirection` | TurnDirection | Hướng rẽ đang thực hiện |
| `stateStartTime` | unsigned long | Thời điểm bắt đầu trạng thái hiện tại |
| `prepareTime` | unsigned long | Thời gian cho giai đoạn chuẩn bị rẽ (ms) |
| `executionTime` | unsigned long | Thời gian cho giai đoạn thực hiện rẽ (ms) |

## Phương thức (Methods)

### Constructor

```cpp
TurnHandler(MotorControl* motorControl, TurnLogic* logic)
```

**Mô tả**: Khởi tạo đối tượng TurnHandler với các giá trị mặc định.

**Tham số**:
- `motorControl`: Con trỏ đến đối tượng điều khiển động cơ
- `logic`: Con trỏ đến đối tượng logic rẽ

**Ví dụ**:
```cpp
MotorControl motor;
TurnLogic turnLogic;
TurnHandler turnHandler(&motor, &turnLogic);
```

### setTurnSpeed

```cpp
void setTurnSpeed(int speed)
```

**Mô tả**: Thiết lập tốc độ khi rẽ.

**Tham số**:
- `speed`: Tốc độ khi rẽ (0-255)

**Ví dụ**:
```cpp
turnHandler.setTurnSpeed(150);
```

### setBaseSpeed

```cpp
void setBaseSpeed(int speed)
```

**Mô tả**: Thiết lập tốc độ cơ bản (tốc độ sau khi rẽ).

**Tham số**:
- `speed`: Tốc độ cơ bản (0-255)

**Ví dụ**:
```cpp
turnHandler.setBaseSpeed(200);
```

### setTurnTiming

```cpp
void setTurnTiming(unsigned long prepare, unsigned long execute)
```

**Mô tả**: Thiết lập thời gian cho giai đoạn chuẩn bị và thực hiện rẽ.

**Tham số**:
- `prepare`: Thời gian cho giai đoạn chuẩn bị (ms)
- `execute`: Thời gian cho giai đoạn thực hiện (ms)

**Ví dụ**:
```cpp
turnHandler.setTurnTiming(200, 500);
```

### enableCheckpointMode

```cpp
void enableCheckpointMode(bool enable)
```

**Mô tả**: Bật hoặc tắt chế độ checkpoint. Khi bật, robot sẽ luôn rẽ theo hướng mặc định.

**Tham số**:
- `enable`: true để bật, false để tắt

**Ví dụ**:
```cpp
turnHandler.enableCheckpointMode(true);
```

### isCheckpointModeEnabled

```cpp
bool isCheckpointModeEnabled()
```

**Mô tả**: Kiểm tra xem chế độ checkpoint có được bật không.

**Giá trị trả về**: true nếu đang bật, false nếu đang tắt

**Ví dụ**:
```cpp
if (turnHandler.isCheckpointModeEnabled()) {
  // Xử lý chế độ checkpoint
}
```

### setDefaultDirection

```cpp
void setDefaultDirection(TurnDirection direction)
```

**Mô tả**: Thiết lập hướng rẽ mặc định cho chế độ checkpoint.

**Tham số**:
- `direction`: Hướng rẽ mặc định (TURN_LEFT, TURN_RIGHT, TURN_UNKNOWN)

**Ví dụ**:
```cpp
turnHandler.setDefaultDirection(TURN_RIGHT);
```

### getDefaultDirection

```cpp
TurnDirection getDefaultDirection()
```

**Mô tả**: Lấy hướng rẽ mặc định hiện tại.

**Giá trị trả về**: Hướng rẽ mặc định

**Ví dụ**:
```cpp
TurnDirection dir = turnHandler.getDefaultDirection();
```

### startTurn

```cpp
void startTurn(TurnDirection direction)
```

**Mô tả**: Bắt đầu quá trình rẽ không chặn. Phương thức này chỉ khởi tạo quá trình rẽ và không chặn luồng thực thi.

**Tham số**:
- `direction`: Hướng rẽ (TURN_LEFT, TURN_RIGHT, TURN_UNKNOWN)

**Ví dụ**:
```cpp
turnHandler.startTurn(TURN_LEFT);
```

### update

```cpp
void update()
```

**Mô tả**: Cập nhật máy trạng thái xử lý rẽ. Cần gọi phương thức này trong mỗi vòng lặp để tiến hành quá trình rẽ.

**Ví dụ**:
```cpp
// Trong vòng lặp chính
turnHandler.update();
```

### isTurning

```cpp
bool isTurning()
```

**Mô tả**: Kiểm tra xem robot có đang trong quá trình rẽ không.

**Giá trị trả về**: true nếu đang rẽ, false nếu không

**Ví dụ**:
```cpp
if (!turnHandler.isTurning()) {
  // Có thể bắt đầu rẽ mới
}
```

### processCommand

```cpp
bool processCommand(const String& command)
```

**Mô tả**: Xử lý lệnh điều khiển liên quan đến rẽ.

**Tham số**:
- `command`: Lệnh cần xử lý

**Giá trị trả về**: true nếu lệnh được xử lý, false nếu không

**Ví dụ**:
```cpp
if (turnHandler.processCommand("TL")) {
  // Lệnh đã được xử lý
}
```

## Cách sử dụng

### Khởi tạo và thiết lập

```cpp
// Khởi tạo
MotorControl motor;
TurnLogic turnLogic;
TurnHandler turnHandler(&motor, &turnLogic);

// Thiết lập
turnHandler.setTurnSpeed(150);    // Tốc độ khi rẽ
turnHandler.setBaseSpeed(200);    // Tốc độ cơ bản sau khi rẽ
turnHandler.setTurnTiming(200, 500); // 200ms chuẩn bị, 500ms thực hiện
turnHandler.setDefaultDirection(TURN_RIGHT); // Mặc định rẽ phải
```

### Sử dụng trong vòng lặp chính

```cpp
void loop() {
  // Cập nhật máy trạng thái xử lý rẽ
  turnHandler.update();
  
  // Kiểm tra xem có cần rẽ không
  if (!turnHandler.isTurning()) {
    TurnDirection nextTurn = turnLogic.getNextTurn();
    if (nextTurn != TURN_UNKNOWN) {
      turnHandler.startTurn(nextTurn);
    }
  }
  
  // Các công việc khác...
}
```

### Xử lý lệnh

```cpp
void handleCommand(String command) {
  // Thử xử lý lệnh bằng TurnHandler
  if (turnHandler.processCommand(command)) {
    // Lệnh đã được xử lý bởi TurnHandler
    return;
  }
  
  // Xử lý các lệnh khác...
}
```

## Lưu ý quan trọng

1. **Không chặn (Non-blocking)**: TurnHandler sử dụng máy trạng thái thay vì `delay()` để đảm bảo robot vẫn phản ứng được với môi trường trong quá trình rẽ.

2. **Gọi update() trong mỗi vòng lặp**: Phương thức update() phải được gọi trong mỗi vòng lặp để quá trình rẽ diễn ra đúng.

3. **Kiểm tra isTurning()**: Sử dụng phương thức này để tránh bắt đầu rẽ mới khi đang trong quá trình rẽ.

4. **Tùy chỉnh thời gian rẽ**: Sử dụng phương thức setTurnTiming() để điều chỉnh thời gian rẽ phù hợp với đặc tính cơ khí của robot và độ phức tạp của đường đua.

5. **Chế độ checkpoint**: Chức năng này cho phép robot luôn rẽ theo một hướng cố định (trái hoặc phải) khi gặp giao lộ, bất kể logic rẽ dự đoán.

## Bản vá lỗi quan trọng

**Vấn đề đã được khắc phục**: Phiên bản cũ sử dụng `delay()` trong quá trình xử lý rẽ, điều này vi phạm nguyên tắc cốt lõi của hệ thống nhúng thời gian thực và có thể gây ra những vấn đề nghiêm trọng:

- Robot bị "đóng băng" trong suốt quá trình rẽ
- Không thể phản ứng với cảm biến hoặc lệnh điều khiển khi đang rẽ
- Có khả năng mất kiểm soát khi rẽ ở tốc độ cao

**Giải pháp**: 
- Thay thế hàm `handleTurn()` bằng một cặp hàm `startTurn()` và `update()` sử dụng máy trạng thái
- Xóa bỏ hoàn toàn việc sử dụng `delay()` trong module
- Cho phép robot tiếp tục xử lý các tác vụ khác trong suốt quá trình rẽ
