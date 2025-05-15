/*
 * Line Follower Robot - Phiên bản cuối cùng
 * Sử dụng điều khiển HYBRID (kết hợp PID và Fuzzy)
 */

#include <Arduino.h>
#include "ColorTracker.h"
#include "RobotController.h"
#include "ReadIR.h"
#include "MotorControl.h"
#include "TurnLogic.h"

// Định nghĩa chân đèn LED
#define LED_PIN 8
#define BUTTON_PIN 7

// Các hằng số điều khiển
#define BASE_SPEED 200          // Tốc độ cơ bản
#define HIGH_SPEED 250          // Tốc độ cao
#define TURN_SPEED 150          // Tốc độ khi rẽ
#define MAX_ADJUST_SPEED 100    // Điều chỉnh tốc độ tối đa
#define MODE_CONTROL CONTROLLER_FUZZY // Chế độ điều khiển (PID, Fuzzy, Hybrid)

// Tham số Fuzzy cho dễ điều chỉnh
#define FUZZY_INPUT_MIN -100.0  // Phạm vi đầu vào tối thiểu
#define FUZZY_INPUT_MAX 100.0   // Phạm vi đầu vào tối đa
#define FUZZY_OUTPUT_MIN -100.0 // Phạm vi đầu ra tối thiểu
#define FUZZY_OUTPUT_MAX 100.0  // Phạm vi đầu ra tối đa

// Các tham số PID
#define KP 0.25
#define KI 0.1
#define KD 0
#define PIDW 0.0
#define FUZW 1.0
// Thời gian lấy mẫu và chu kỳ xử lý (ms)
#define SAMPLE_TIME 2
#define COLOR_SAMPLE_TIME 50


// Khai báo các đối tượng toàn cục  CONTROLLER_HYBRID
RobotController controller(MODE_CONTROL, KP, KI, KD, -100, 100, -100, 100);
ReadIR irSensor;
ColorTracker colorTracker(COLOR_SAMPLE_TIME);
MotorControl motor;
TurnLogic turnLogic;

// Biến toàn cục
unsigned long lastUpdateTime = 0;
unsigned long lastStatusTime = 0;
unsigned long lastTurnCheck = 0;
float lastError = 0;
bool highSpeedMode = false;
bool lastColorIsBlue = false;  // Biến để theo dõi checkpoint màu xanh dương
bool lastColorIsGreen = false; // Biến để theo dõi checkpoint màu xanh lá
bool isRunning = false;

// Khai báo hàm
void setupLED();
void processColor();
void handleTurn(TurnDirection direction);
void checkSerialCommand();

void setup() {
  // Khởi tạo Serial để debug
  Serial.begin(115200);
  delay(50); // Chờ Serial ổn định
  Serial.println("Line Follower Robot - Phiên bản cuối cùng");
  
  // Khởi tạo LED
  setupLED();
  
  // Khởi tạo các module
  setupMotors();
  Serial.println("Đã khởi tạo động cơ");
  
  // Khởi tạo cảm biến IR
  irSensor.begin();
  Serial.println("Đã khởi tạo cảm biến IR");
  
  // Khởi tạo cảm biến màu
  if (!colorTracker.begin()) {
    Serial.println("CẢNH BÁO: Không thể khởi tạo cảm biến màu!");
    digitalWrite(LED_PIN, HIGH); // Bật LED để báo lỗi
    
    // Nhấp nháy LED để báo lỗi, nhưng không treo hệ thống
    for (int i = 0; i < 10; i++) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(200);
      digitalWrite(LED_PIN, LOW);
      delay(200);
    }
    
    // Tiếp tục với các cảm biến khác đã hoạt động
    Serial.println("Tiếp tục mà không có cảm biến màu");
  } else {
    Serial.println("Đã khởi tạo cảm biến màu");
  }
  if (MODE_CONTROL == CONTROLLER_HYBRID) {
    // Thiết lập trọng số cho điều khiển hybrid
    controller.setHybridWeights(PIDW, FUZW);
  }
  
  // Cấu hình phạm vi đầu vào/ra cho Fuzzy Controller
  controller.setFuzzyInputRange(FUZZY_INPUT_MIN, FUZZY_INPUT_MAX);
  controller.setOutputLimits(FUZZY_OUTPUT_MIN, FUZZY_OUTPUT_MAX);
  
  Serial.println("Đã cấu hình bộ điều khiển");
  Serial.print("Chế độ điều khiển: ");
  
  switch (MODE_CONTROL) {
    case CONTROLLER_PID:
      Serial.println("PID");
      break;
    case CONTROLLER_FUZZY:
      Serial.println("FUZZY");
      break;
    case CONTROLLER_HYBRID:
      Serial.print("HYBRID (PID: ");
      Serial.print(PIDW);
      Serial.print(", FUZZY: ");
      Serial.print(FUZW);
      Serial.println(")");
      break;
  }

  // Hiệu chỉnh cảm biến IR
  Serial.println("Bắt đầu hiệu chỉnh cảm biến IR...");
  
  // Nhấp nháy LED để thông báo bắt đầu hiệu chỉnh
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
    yield();
  }
  
  // Thực hiện hiệu chỉnh
  irSensor.CalibrateSensor();
  Serial.println("Hiệu chỉnh hoàn tất!");
  
  // Đợi người dùng nhấn nút để bắt đầu
  Serial.println("Nhấn nút để bắt đầu hoặc gửi ký tự 's' qua Serial");
  
  pinMode(BUTTON_PIN, INPUT_PULLUP); // Cấu hình nút nhấn với pull-up

  // Chờ nút nhấn để bắt đầu
  while (digitalRead(BUTTON_PIN) == HIGH) {
    // Nhấp nháy LED trong khi chờ
    digitalWrite(LED_PIN, (millis() / 500) % 2);
    // Cho phép xử lý các tác vụ khác
    yield();
  }
  
  // Đèn LED sáng liên tục khi bắt đầu chạy
  digitalWrite(LED_PIN, HIGH);
  
  // Đặt thời gian ban đầu
  lastUpdateTime = millis();
  lastStatusTime = millis();
  lastTurnCheck = millis();
}

void loop() {
  unsigned long currentTime = millis();
  
  // Kiểm tra xem có lệnh điều chỉnh từ Serial không
  checkSerialCommand();
  
  // Cập nhật giá trị cảm biến màu
  colorTracker.update();
  
  // Xử lý thông tin màu
  processColor();
  
  // Tính toán error từ cảm biến hồng ngoại
  float error = irSensor.calculateError();
  
  // Tính toán delta time
  float dt = (currentTime - lastUpdateTime) / 1000.0f; // Chuyển đổi sang giây
  
  // Làm mới thời gian
  lastUpdateTime = currentTime;
  
  // Tính toán đầu ra điều khiển
  int controlOutput = controller.compute(error, dt);
  
  // Giới hạn đầu ra điều khiển để tránh thay đổi đột ngột
  controlOutput = constrain(controlOutput, -MAX_ADJUST_SPEED, MAX_ADJUST_SPEED);
  
  // Xác định tốc độ cơ bản
  int baseSpeed = highSpeedMode ? HIGH_SPEED : BASE_SPEED;
  
  // Tính toán tốc độ cho mỗi động cơ
  int leftSpeed = baseSpeed - controlOutput;
  int rightSpeed = baseSpeed + controlOutput;
  
  // Điều khiển động cơ
  setMotorSpeeds(leftSpeed, rightSpeed);
  
  // In thông tin trạng thái mỗi 500ms
  if (currentTime - lastStatusTime >= 500) {
    lastStatusTime = currentTime;
    Serial.print("Error: ");
    Serial.print(error);
    Serial.print(" | Control: ");
    Serial.print(controlOutput);
    Serial.print(" | Tốc độ (T,P): (");
    Serial.print(leftSpeed);
    Serial.print(",");
    Serial.print(rightSpeed);
    Serial.print(") | Chế độ: ");
    Serial.println(highSpeedMode ? "NHANH" : "THƯỜNG");
  }
  
  // Cập nhật logic rẽ nếu cần
  uint8_t colorValue;
  bool speedMode;
  unsigned long timestamp;
  
  if (colorTracker.getResult(&colorValue, &speedMode, &timestamp)) {
    // Cập nhật cho logic rẽ - Checkpoint không ảnh hưởng đến tốc độ tại đây
    int blue = (colorValue == COLOR_BLUE) ? 1 : 0;
    int green = (colorValue == COLOR_GREEN) ? (highSpeedMode ? 2 : 1) : 0;
    turnLogic.update(blue, green, error);
  }
  
  // Kiểm tra điều kiện rẽ mỗi 100ms
  if (currentTime - lastTurnCheck >= 100) {
    lastTurnCheck = currentTime;
    TurnDirection nextTurn = turnLogic.getNextTurn();
    
    // Xử lý rẽ nếu gặp giao lộ mới
    if (nextTurn != TURN_UNKNOWN) {
      handleTurn(nextTurn);
      turnLogic.reset(); // Reset sau khi đã xử lý
    }
  }
  
  // Đảm bảo tần suất lấy mẫu ổn định
  int remainingTime = SAMPLE_TIME - (millis() - currentTime);
  if (remainingTime > 0) {
    delay(remainingTime);
  }
}

// Thiết lập LED
void setupLED() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}

// Xử lý thông tin màu
void processColor() {
  uint8_t colorValue;
  bool speedMode;
  unsigned long timestamp;
  
  if (colorTracker.getResult(&colorValue, &speedMode, &timestamp)) {
    // Cập nhật trạng thái màu
    bool isBlueColor = (colorValue == COLOR_BLUE);
    bool isGreenColor = (colorValue == COLOR_GREEN);
    
    // Xử lý chuyển đổi tốc độ dựa trên checkpoint
    if (isBlueColor && !lastColorIsBlue) {
      highSpeedMode = false;
      // Khi mới phát hiện điểm xanh dương, giữ nguyên tốc độ hiện tại
      Serial.println("Checkpoint XANH DƯƠNG: Giữ tốc độ hiện tại");
    }
    
    if (isGreenColor && !lastColorIsGreen) {
      // Khi mới phát hiện điểm xanh lá, kích hoạt chế độ tốc độ cao
      highSpeedMode = true;
      Serial.println("Checkpoint XANH LÁ: Kích hoạt tốc độ cao");
    }
    
    // Cập nhật trạng thái màu trước đó
    lastColorIsBlue = isBlueColor;
    lastColorIsGreen = isGreenColor;
    
    // In thông tin màu nếu khác trắng/đen
    if (colorValue != COLOR_BLACK && colorValue != COLOR_WHITE) {
      String colorName = "UNKNOWN";
      switch (colorValue) {
        case COLOR_BLACK: colorName = "ĐEN"; break;
        case COLOR_WHITE: colorName = "TRẮNG"; break;
        case COLOR_BLUE: colorName = "XANH DƯƠNG"; break;
        case COLOR_GREEN: colorName = "XANH LÁ"; break;
      }
      
      // In thông tin cảm biến màu chi tiết (để điều chỉnh)
      uint16_t r, g, b, c;
      colorTracker.getRawValues(&r, &g, &b, &c);
      Serial.print("Màu phát hiện: ");
      Serial.print(colorName);
      Serial.print(" | R:");
      Serial.print(r);
      Serial.print(" G:");
      Serial.print(g);
      Serial.print(" B:");
      Serial.print(b);
      Serial.print(" C:");
      Serial.println(c);
    }
  }
}

// Xử lý rẽ
void handleTurn(TurnDirection direction) {
  Serial.print("Thực hiện rẽ: ");
  
  // Giảm tốc độ trước khi rẽ
  motor.Forward(TURN_SPEED);
  delay(200);
  
  switch (direction) {
    case TURN_LEFT:
      Serial.println("TRÁI");
      motor.turnLeft(TURN_SPEED);
      delay(500); // Điều chỉnh thời gian cho phù hợp với góc rẽ
      break;
      
    case TURN_RIGHT:
      Serial.println("PHẢI");
      motor.turnRight(TURN_SPEED);
      delay(500); // Điều chỉnh thời gian cho phù hợp với góc rẽ
      break;
      
    default:
      Serial.println("KHÔNG XÁC ĐỊNH");
      break;
  }
  
  // Tiếp tục di chuyển
  motor.Forward(BASE_SPEED);
  
  // Reset controller để tránh tích lũy sai số
  controller.reset();
}

// Hàm kiểm tra và xử lý lệnh từ Serial (để điều chỉnh tham số)
void checkSerialCommand() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    
    // Bỏ các ký tự whitespace
    command.trim();
    
    // Tạm dừng hoặc tiếp tục
    if (command == "p" || command == "pause") {
      Serial.println("Tạm dừng robot");
      setMotorSpeeds(0, 0);
      while (Serial.read() != 'r'); // Đợi lệnh tiếp tục ('r')
      Serial.println("Tiếp tục chạy");
      return;
    }
    
    // Lệnh thay đổi chế độ tốc độ
    if (command == "fast") {
      highSpeedMode = true;
      Serial.println("Đã chuyển sang chế độ tốc độ cao");
      return;
    }
    
    if (command == "normal") {
      highSpeedMode = false;
      Serial.println("Đã chuyển sang chế độ tốc độ thường");
      return;
    }
    
    // Lệnh thay đổi phạm vi đầu vào/ra Fuzzy
    if (command.startsWith("fuzzy_range")) {
      int idx1 = command.indexOf(' ');
      int idx2 = command.indexOf(' ', idx1 + 1);
      int idx3 = command.indexOf(' ', idx2 + 1);
      int idx4 = command.indexOf(' ', idx3 + 1);
      
      if (idx1 > 0 && idx2 > idx1 && idx3 > idx2 && idx4 > idx3) {
        float inMin = command.substring(idx1, idx2).toFloat();
        float inMax = command.substring(idx2, idx3).toFloat();
        float outMin = command.substring(idx3, idx4).toFloat();
        float outMax = command.substring(idx4).toFloat();
        
        controller.setFuzzyInputRange(inMin, inMax);
        controller.setOutputLimits(outMin, outMax);
        
        Serial.print("Đã cập nhật phạm vi Fuzzy: Input [");
        Serial.print(inMin);
        Serial.print(", ");
        Serial.print(inMax);
        Serial.print("], Output [");
        Serial.print(outMin);
        Serial.print(", ");
        Serial.print(outMax);
        Serial.println("]");
      }
      return;
    }
    
    // Lệnh hiển thị trợ giúp
    if (command == "help") {
      Serial.println("\n=== Lệnh điều khiển robot ===");
      Serial.println("p/pause: Tạm dừng robot");
      Serial.println("r: Tiếp tục sau khi tạm dừng");
      Serial.println("fast: Chuyển sang chế độ tốc độ cao");
      Serial.println("normal: Chuyển sang chế độ tốc độ thường");
      Serial.println("fuzzy_range [inMin] [inMax] [outMin] [outMax]: Điều chỉnh phạm vi Fuzzy");
      Serial.println("help: Hiển thị trợ giúp");
      return;
    }
  }
}
