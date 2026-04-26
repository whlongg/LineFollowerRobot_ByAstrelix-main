# Tài liệu mô đun BLEdebug

## Tổng quan

Mô đun `BLEdebug` cung cấp khả năng giao tiếp không dây qua Bluetooth Low Energy (BLE) để điều khiển, giám sát và cấu hình robot từ xa. Module này cho phép điều chỉnh các thông số điều khiển, gửi lệnh đến robot, và nhận dữ liệu debug theo thời gian thực, mang lại sự linh hoạt và thuận tiện trong quá trình phát triển và hiệu chỉnh hệ thống.

## Tệp liên quan
- **BLEdebug.h**: Định nghĩa giao diện của lớp BLEdebug
- **BLEdebug.cpp**: Triển khai các phương thức giao tiếp và xử lý BLE

## Các đặc điểm BLE

Module BLEdebug sử dụng cấu trúc sau:

- **Tên thiết bị**: "LineFollower" + ID duy nhất
- **Service UUID**: Dịch vụ chính để tương tác với robot
- **Characteristic UUIDs**: Các đặc tính riêng biệt để đọc/ghi dữ liệu cụ thể

## Lớp BLEdebug

Lớp `BLEdebug` cung cấp các phương thức để thiết lập kết nối BLE, xử lý lệnh nhận được và gửi dữ liệu debug.

### Constructor

```cpp
BLEdebug()
```

**Mô tả**: Khởi tạo đối tượng BLEdebug với các giá trị mặc định.

**Chi tiết triển khai**:
- Khởi tạo các biến trạng thái BLE
- Thiết lập các con trỏ callback mặc định là nullptr

**Ví dụ**:
```cpp
BLEdebug bleDebug;
```

### begin

```cpp
bool begin(const char* deviceName = nullptr)
```

**Mô tả**: Khởi tạo và cấu hình giao tiếp BLE.

**Tham số**:
- `deviceName`: Tên thiết bị BLE tùy chọn (nếu nullptr, sẽ sử dụng tên mặc định)

**Giá trị trả về**: `true` nếu khởi tạo thành công, `false` nếu thất bại

**Chi tiết triển khai**:
- Khởi tạo stack BLE
- Tạo dịch vụ và các đặc tính
- Thiết lập các callback cho sự kiện kết nối và nhận dữ liệu
- Bắt đầu quảng cáo dịch vụ BLE

**Ví dụ**:
```cpp
if (!bleDebug.begin("LineFollower_001")) {
  Serial.println("Không thể khởi tạo BLE!");
  // Xử lý lỗi...
}
```

### update

```cpp
void update()
```

**Mô tả**: Cập nhật trạng thái BLE, xử lý sự kiện và dữ liệu nhận được.

**Chi tiết triển khai**:
- Kiểm tra kết nối BLE
- Xử lý các lệnh đã nhận
- Cập nhật dữ liệu thông báo nếu cần

**Ví dụ**:
```cpp
// Trong vòng lặp chính
void loop() {
  // Cập nhật BLE
  bleDebug.update();
  
  // Xử lý các tác vụ khác...
}
```

### isConnected

```cpp
bool isConnected()
```

**Mô tả**: Kiểm tra xem kết nối BLE có đang hoạt động không.

**Giá trị trả về**: `true` nếu có kết nối, `false` nếu không

**Ví dụ**:
```cpp
if (bleDebug.isConnected()) {
  // Thực hiện các tác vụ chỉ khi có kết nối BLE
  bleDebug.sendDebugData("Connected");
}
```

### sendDebugData

```cpp
void sendDebugData(const char* data)
void sendDebugData(String data)
```

**Mô tả**: Gửi dữ liệu debug qua BLE.

**Tham số**:
- `data`: Dữ liệu cần gửi (chuỗi ký tự hoặc đối tượng String)

**Chi tiết triển khai**:
- Kiểm tra kết nối BLE
- Gửi dữ liệu qua đặc tính thông báo
- Xử lý giới hạn kích thước nếu cần

**Ví dụ**:
```cpp
// Gửi dữ liệu đơn giản
bleDebug.sendDebugData("PID: Kp=2.5, Ki=0.1, Kd=10.0");

// Hoặc sử dụng String
String statusMsg = "Error: " + String(currentError) + ", Speed: " + String(currentSpeed);
bleDebug.sendDebugData(statusMsg);
```

### sendSensorData

```cpp
void sendSensorData(float *sensorValues, int count)
```

**Mô tả**: Gửi dữ liệu từ cảm biến qua BLE.

**Tham số**:
- `sensorValues`: Mảng giá trị cảm biến
- `count`: Số lượng giá trị trong mảng

**Chi tiết triển khai**:
- Định dạng dữ liệu cảm biến thành chuỗi JSON
- Gửi dữ liệu qua đặc tính thông báo

**Ví dụ**:
```cpp
// Đọc và gửi dữ liệu cảm biến IR
float irValues[4];
irSensor.readSensors(irValues);
bleDebug.sendSensorData(irValues, 4);
```

### sendPIDData

```cpp
void sendPIDData(float error, float output, float kp, float ki, float kd)
```

**Mô tả**: Gửi dữ liệu PID để giám sát và điều chỉnh.

**Tham số**:
- `error`: Giá trị lỗi hiện tại
- `output`: Giá trị đầu ra của PID
- `kp`: Hệ số tỷ lệ
- `ki`: Hệ số tích phân 
- `kd`: Hệ số vi phân

**Chi tiết triển khai**:
- Định dạng dữ liệu PID thành chuỗi JSON
- Gửi dữ liệu qua đặc tính thông báo

**Ví dụ**:
```cpp
// Trong quá trình tính toán PID
float error = irSensor.calculateError();
float output = pidController.compute(error, dt);

// Gửi dữ liệu PID để giám sát
bleDebug.sendPIDData(
  error, 
  output, 
  pidController.getKp(), 
  pidController.getKi(), 
  pidController.getKd()
);
```

### registerCommandCallback

```cpp
void registerCommandCallback(void (*callback)(const char*, int))
```

**Mô tả**: Đăng ký hàm callback để xử lý lệnh nhận được qua BLE.

**Tham số**:
- `callback`: Con trỏ đến hàm callback có dạng `void function(const char* command, int length)`

**Chi tiết triển khai**:
- Lưu con trỏ callback để gọi khi nhận được dữ liệu

**Ví dụ**:
```cpp
// Hàm xử lý lệnh BLE
void handleBLECommand(const char* command, int length) {
  String cmd = String(command);
  
  if (cmd.startsWith("SPEED:")) {
    int newSpeed = cmd.substring(6).toInt();
    setBaseSpeed(newSpeed);
  } else if (cmd.startsWith("PID:")) {
    // Phân tích và cập nhật thông số PID
    // ...
  }
}

// Đăng ký callback
void setup() {
  // Khởi tạo BLE
  bleDebug.begin();
  
  // Đăng ký callback
  bleDebug.registerCommandCallback(handleBLECommand);
}
```

### registerConnectionCallback

```cpp
void registerConnectionCallback(void (*callback)(bool))
```

**Mô tả**: Đăng ký hàm callback để xử lý sự kiện kết nối/ngắt kết nối BLE.

**Tham số**:
- `callback`: Con trỏ đến hàm callback có dạng `void function(bool connected)`

**Chi tiết triển khai**:
- Lưu con trỏ callback để gọi khi trạng thái kết nối thay đổi

**Ví dụ**:
```cpp
// Hàm xử lý sự kiện kết nối
void handleBLEConnection(bool connected) {
  if (connected) {
    Serial.println("BLE đã kết nối");
    digitalWrite(LED_BUILTIN, HIGH);  // Bật LED khi có kết nối
  } else {
    Serial.println("BLE đã ngắt kết nối");
    digitalWrite(LED_BUILTIN, LOW);   // Tắt LED khi mất kết nối
  }
}

// Đăng ký callback
void setup() {
  // Khởi tạo BLE
  bleDebug.begin();
  
  // Đăng ký callback
  bleDebug.registerConnectionCallback(handleBLEConnection);
}
```

### setDeviceName

```cpp
void setDeviceName(const char* name)
```

**Mô tả**: Đặt tên cho thiết bị BLE.

**Tham số**:
- `name`: Tên thiết bị mới

**Ví dụ**:
```cpp
// Thiết lập tên riêng dựa trên ID phần cứng
uint8_t id = getHardwareID();
String deviceName = "LineBot_" + String(id, HEX);
bleDebug.setDeviceName(deviceName.c_str());
```

## Chi tiết triển khai

### Cấu trúc BLE Service

```cpp
// Khởi tạo server BLE và các đặc tính
void BLEdebug::setupBLEServices() {
  // Tạo server BLE
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks(this));
  
  // Tạo dịch vụ
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  // Tạo đặc tính lệnh (cho phép ghi từ client)
  commandCharacteristic = pService->createCharacteristic(
    COMMAND_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  commandCharacteristic->setCallbacks(new CommandCallbacks(this));
  
  // Tạo đặc tính thông báo (để gửi dữ liệu đến client)
  notifyCharacteristic = pService->createCharacteristic(
    NOTIFY_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  notifyCharacteristic->addDescriptor(new BLE2902());
  
  // Tạo đặc tính cấu hình PID (cho phép đọc/ghi)
  pidConfigCharacteristic = pService->createCharacteristic(
    PID_CONFIG_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
  );
  
  // Khởi động dịch vụ
  pService->start();
}
```

### Xử lý lệnh BLE

```cpp
// Lớp xử lý các lệnh nhận được
class CommandCallbacks: public BLECharacteristicCallbacks {
private:
  BLEdebug* bleDebug;
  
public:
  CommandCallbacks(BLEdebug* debug) : bleDebug(debug) {}
  
  void onWrite(BLECharacteristic *characteristic) {
    std::string value = characteristic->getValue();
    if (value.length() > 0) {
      // Gọi callback do người dùng đăng ký
      if (bleDebug->commandCallback) {
        bleDebug->commandCallback(value.c_str(), value.length());
      }
    }
  }
};
```

### Định dạng dữ liệu JSON

```cpp
// Gửi dữ liệu cảm biến dưới dạng JSON
void BLEdebug::sendSensorData(float *sensorValues, int count) {
  if (!isConnected() || count <= 0) return;
  
  // Tạo đối tượng JSON
  DynamicJsonDocument doc(256);
  JsonArray array = doc.createNestedArray("sensors");
  
  // Thêm giá trị cảm biến
  for (int i = 0; i < count; i++) {
    array.add(sensorValues[i]);
  }
  
  // Thêm thời gian
  doc["timestamp"] = millis();
  
  // Chuyển đổi thành chuỗi
  String jsonString;
  serializeJson(doc, jsonString);
  
  // Gửi dữ liệu
  sendDebugData(jsonString);
}
```

## Cách sử dụng

### Khởi tạo BLE

```cpp
// Trong hàm setup()
BLEdebug bleDebug;

void setup() {
  // Khởi tạo Serial
  Serial.begin(115200);
  
  // Khởi tạo BLE với tên tùy chỉnh
  if (!bleDebug.begin("LineFollower_Robot")) {
    Serial.println("Lỗi: Không thể khởi tạo BLE!");
  } else {
    Serial.println("BLE đã khởi tạo, đang chờ kết nối...");
  }
  
  // Đăng ký các hàm callback
  bleDebug.registerCommandCallback(handleBLECommand);
  bleDebug.registerConnectionCallback(handleBLEConnection);
}
```

### Cập nhật và gửi dữ liệu

```cpp
// Trong vòng lặp chính
void loop() {
  // Đọc cảm biến và tính toán điều khiển
  float sensorValues[SENSOR_COUNT];
  irSensor.readSensors(sensorValues);
  
  float error = irSensor.calculateError();
  float output = pidController.compute(error, dt);
  
  // Gửi dữ liệu debug qua BLE
  if (bleDebug.isConnected()) {
    // Gửi dữ liệu cảm biến mỗi 50ms
    static unsigned long lastSensorUpdate = 0;
    if (millis() - lastSensorUpdate > 50) {
      lastSensorUpdate = millis();
      bleDebug.sendSensorData(sensorValues, SENSOR_COUNT);
    }
    
    // Gửi dữ liệu PID mỗi 100ms
    static unsigned long lastPIDUpdate = 0;
    if (millis() - lastPIDUpdate > 100) {
      lastPIDUpdate = millis();
      bleDebug.sendPIDData(
        error, 
        output, 
        pidController.getKp(), 
        pidController.getKi(), 
        pidController.getKd()
      );
    }
  }
  
  // Cập nhật BLE (xử lý sự kiện, lệnh...)
  bleDebug.update();
  
  // Xử lý các tác vụ khác...
}
```

### Xử lý lệnh BLE

```cpp
// Hàm xử lý lệnh nhận được qua BLE
void handleBLECommand(const char* command, int length) {
  String cmd = String(command);
  Serial.print("Đã nhận lệnh BLE: ");
  Serial.println(cmd);
  
  // Xử lý lệnh điều khiển tốc độ
  if (cmd.startsWith("SPEED:")) {
    int speed = cmd.substring(6).toInt();
    baseSpeed = constrain(speed, 0, 255);
    Serial.print("Đã đặt tốc độ mới: ");
    Serial.println(baseSpeed);
  }
  // Xử lý lệnh điều chỉnh PID
  else if (cmd.startsWith("PID:")) {
    // Phân tích chuỗi để lấy các tham số PID
    // Ví dụ: "PID:2.5,0.1,10.0" -> Kp=2.5, Ki=0.1, Kd=10.0
    String params = cmd.substring(4);
    int commaIndex1 = params.indexOf(',');
    int commaIndex2 = params.indexOf(',', commaIndex1 + 1);
    
    if (commaIndex1 > 0 && commaIndex2 > 0) {
      float kp = params.substring(0, commaIndex1).toFloat();
      float ki = params.substring(commaIndex1 + 1, commaIndex2).toFloat();
      float kd = params.substring(commaIndex2 + 1).toFloat();
      
      // Cập nhật tham số PID
      pidController.setTunings(kp, ki, kd);
      
      Serial.println("Đã cập nhật tham số PID:");
      Serial.print("Kp="); Serial.print(kp);
      Serial.print(", Ki="); Serial.print(ki);
      Serial.print(", Kd="); Serial.println(kd);
    }
  }
  // Xử lý lệnh điều khiển trực tiếp
  else if (cmd.startsWith("TURN:")) {
    String direction = cmd.substring(5);
    if (direction == "LEFT") {
      turnHandler.startTurn(LEFT90);
    } else if (direction == "RIGHT") {
      turnHandler.startTurn(RIGHT90);
    } else if (direction == "UTURN") {
      turnHandler.startTurn(UTURN);
    }
  }
  // Xử lý lệnh chế độ
  else if (cmd.startsWith("MODE:")) {
    String mode = cmd.substring(5);
    if (mode == "FAST") {
      // Chuyển sang chế độ tốc độ cao
      baseSpeed = HIGH_SPEED;
    } else if (mode == "NORMAL") {
      // Chuyển sang chế độ bình thường
      baseSpeed = NORMAL_SPEED;
    } else if (mode == "CALIBRATE") {
      // Bắt đầu hiệu chuẩn cảm biến
      performCalibration();
    }
  }
}
```

### Xử lý sự kiện kết nối

```cpp
// Hàm xử lý sự kiện kết nối/ngắt kết nối
void handleBLEConnection(bool connected) {
  if (connected) {
    Serial.println("BLE đã kết nối!");
    
    // Gửi thông tin hiện tại đến ứng dụng
    String info = "Robot Info: Speed=" + String(baseSpeed) + 
                  ", PID=" + String(pidController.getKp()) + 
                  "," + String(pidController.getKi()) + 
                  "," + String(pidController.getKd());
    bleDebug.sendDebugData(info);
    
    // Bật LED báo hiệu có kết nối
    digitalWrite(STATUS_LED, HIGH);
  } else {
    Serial.println("BLE đã ngắt kết nối!");
    
    // Tắt LED
    digitalWrite(STATUS_LED, LOW);
  }
}
```

## Lưu ý khi sử dụng

1. **Tần suất gửi dữ liệu**:
   - Việc gửi dữ liệu quá thường xuyên có thể làm giảm hiệu năng hệ thống
   - Khuyến nghị: Gửi dữ liệu cảm biến mỗi 50-100ms, dữ liệu PID mỗi 100-200ms

2. **Kích thước dữ liệu**:
   - Thông số BLE có giới hạn về kích thước gói tin (thường là 20 bytes)
   - Cần chia dữ liệu lớn thành nhiều gói nhỏ hơn hoặc sử dụng nén dữ liệu

3. **Xử lý lệnh**:
   - Nên kiểm tra kỹ lệnh nhận được trước khi thực thi
   - Giới hạn các giá trị trong phạm vi hợp lý để tránh hành vi không mong muốn

4. **Tiêu thụ năng lượng**:
   - BLE tiêu thụ pin, đặc biệt khi quảng cáo và truyền dữ liệu liên tục
   - Cân nhắc tắt BLE khi không cần thiết hoặc giảm tần suất gửi dữ liệu

## Nâng cấp khả năng

1. **Nâng cao giao diện lệnh**:
   - Cải thiện cú pháp lệnh và hỗ trợ nhiều lệnh phức tạp hơn
   - Thêm kiểm tra lỗi và xác nhận lệnh

2. **Giao thức dữ liệu hiệu quả hơn**:
   - Sử dụng định dạng nhị phân thay vì JSON để giảm kích thước dữ liệu
   - Nén dữ liệu để giảm lưu lượng truyền

3. **Bảo mật**:
   - Thêm xác thực để ngăn kết nối trái phép
   - Mã hóa dữ liệu truyền để tăng cường bảo mật

4. **Ứng dụng điều khiển nâng cao**:
   - Phát triển ứng dụng di động tùy chỉnh để điều khiển robot
   - Bổ sung giao diện trực quan để theo dõi và điều chỉnh thông số

## Ví dụ triển khai nâng cao

### Gửi dữ liệu nhị phân

```cpp
// Gửi dữ liệu cảm biến dưới dạng nhị phân thay vì JSON
void BLEdebug::sendBinarySensorData(float *sensorValues, int count) {
  if (!isConnected() || count <= 0) return;
  
  // Tính kích thước gói: 1 byte loại gói + 4 bytes mỗi giá trị
  int packetSize = 1 + (count * sizeof(float));
  uint8_t *packet = new uint8_t[packetSize];
  
  // Loại gói: 0x01 = dữ liệu cảm biến
  packet[0] = 0x01;
  
  // Đóng gói giá trị cảm biến
  for (int i = 0; i < count; i++) {
    memcpy(&packet[1 + (i * sizeof(float))], &sensorValues[i], sizeof(float));
  }
  
  // Gửi gói
  notifyCharacteristic->setValue(packet, packetSize);
  notifyCharacteristic->notify();
  
  // Giải phóng bộ nhớ
  delete[] packet;
}
```

### Mã hóa lệnh và dữ liệu

```cpp
// Lớp mã hóa đơn giản
class SimpleEncryption {
private:
  uint8_t key[16];
  
public:
  SimpleEncryption(const char* passphrase) {
    // Tạo khóa từ cụm mật khẩu
    for (int i = 0; i < 16; i++) {
      key[i] = (i < strlen(passphrase)) ? passphrase[i] : 0;
    }
  }
  
  // Mã hóa/giải mã dữ liệu (XOR với khóa)
  void processData(uint8_t* data, size_t length) {
    for (size_t i = 0; i < length; i++) {
      data[i] = data[i] ^ key[i % 16];
    }
  }
};

// Sử dụng mã hóa trong BLEdebug
void BLEdebug::sendEncryptedData(const char* data) {
  if (!isConnected()) return;
  
  // Tạo bản sao của dữ liệu
  size_t length = strlen(data);
  uint8_t* buffer = new uint8_t[length];
  memcpy(buffer, data, length);
  
  // Mã hóa dữ liệu
  encryption.processData(buffer, length);
  
  // Gửi dữ liệu đã mã hóa
  notifyCharacteristic->setValue(buffer, length);
  notifyCharacteristic->notify();
  
  // Giải phóng bộ nhớ
  delete[] buffer;
}
```

### Giao thức lệnh cấu trúc

```cpp
// Định nghĩa cấu trúc lệnh
struct BLECommand {
  uint8_t commandId;
  uint8_t dataLength;
  uint8_t data[20];
};

// Hàm xử lý lệnh cấu trúc
void processStructuredCommand(BLECommand* cmd) {
  switch (cmd->commandId) {
    case 0x01:  // Lệnh điều khiển tốc độ
      if (cmd->dataLength >= 1) {
        uint8_t speed = cmd->data[0];
        setBaseSpeed(speed);
      }
      break;
      
    case 0x02:  // Lệnh PID
      if (cmd->dataLength >= 12) {
        float kp, ki, kd;
        memcpy(&kp, &cmd->data[0], 4);
        memcpy(&ki, &cmd->data[4], 4);
        memcpy(&kd, &cmd->data[8], 4);
        pidController.setTunings(kp, ki, kd);
      }
      break;
      
    case 0x03:  // Lệnh rẽ
      if (cmd->dataLength >= 1) {
        uint8_t direction = cmd->data[0];
        turnHandler.startTurn((TurnDirection)direction);
      }
      break;
      
    // Xử lý các lệnh khác...
  }
}
```
