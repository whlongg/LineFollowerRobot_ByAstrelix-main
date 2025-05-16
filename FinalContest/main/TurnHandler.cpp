#include "TurnHandler.h"

TurnHandler::TurnHandler(MotorControl* motorControl, TurnLogic* logic) {
  motor = motorControl;
  turnLogic = logic;
  checkpointMode = false;
  defaultDirection = TURN_RIGHT; // Mặc định rẽ phải khi gặp giao lộ
  turnSpeed = 150; // Tốc độ mặc định khi rẽ
  baseSpeed = 200; // Tốc độ cơ bản mặc định
  
  // Khởi tạo máy trạng thái
  currentState = TURN_IDLE;
  prepareTime = 200;    // 200ms cho giai đoạn chuẩn bị
  executionTime = 500;  // 500ms cho giai đoạn thực hiện rẽ
}

void TurnHandler::setTurnSpeed(int speed) {
  turnSpeed = speed;
}

void TurnHandler::setBaseSpeed(int speed) {
  baseSpeed = speed;
}

void TurnHandler::setTurnTiming(unsigned long prepare, unsigned long execute) {
  prepareTime = prepare;
  executionTime = execute;
}

void TurnHandler::enableCheckpointMode(bool enable) {
  checkpointMode = enable;
  if (enable) {
    Serial.println("Đã bật chế độ checkpoint (rẽ theo hướng mặc định)");
  } else {
    Serial.println("Đã tắt chế độ checkpoint (rẽ theo TurnLogic)");
  }
}

bool TurnHandler::isCheckpointModeEnabled() {
  return checkpointMode;
}

void TurnHandler::setDefaultDirection(TurnDirection direction) {
  defaultDirection = direction;
  Serial.print("Đã thiết lập hướng rẽ mặc định: ");
  
  if (direction == TURN_LEFT) {
    Serial.println("TRÁI");
  } else if (direction == TURN_RIGHT) {
    Serial.println("PHẢI");
  } else {
    Serial.println("KHÔNG XÁC ĐỊNH");
  }
}

TurnDirection TurnHandler::getDefaultDirection() {
  return defaultDirection;
}

void TurnHandler::startTurn(TurnDirection direction) {
  // Không bắt đầu rẽ nếu đang trong quá trình rẽ
  if (currentState != TURN_IDLE) {
    return;
  }
  
  // Nếu chế độ checkpoint được bật, sử dụng hướng mặc định thay vì hướng được chỉ định
  if (checkpointMode) {
    activeDirection = defaultDirection;
    Serial.print("Chế độ checkpoint: Rẽ theo hướng mặc định: ");
  } else {
    activeDirection = direction;
    Serial.print("Bắt đầu rẽ: ");
  }
  
  // In ra hướng rẽ
  if (activeDirection == TURN_LEFT) {
    Serial.println("TRÁI");
  } else if (activeDirection == TURN_RIGHT) {
    Serial.println("PHẢI");
  } else {
    Serial.println("KHÔNG XÁC ĐỊNH");
    // Không rẽ nếu hướng không xác định
    return;
  }
  
  // Khởi tạo máy trạng thái
  currentState = TURN_PREPARE;
  stateStartTime = millis();
}

void TurnHandler::update() {
  // Không làm gì nếu không trong quá trình rẽ
  if (currentState == TURN_IDLE) {
    return;
  }
  
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - stateStartTime;
  
  switch (currentState) {
    case TURN_PREPARE:
      // Giai đoạn chuẩn bị: giảm tốc độ
      motor->Forward(turnSpeed);
      
      // Kiểm tra xem đã hoàn thành giai đoạn chuẩn bị chưa
      if (elapsedTime >= prepareTime) {
        // Chuyển sang giai đoạn thực hiện rẽ
        currentState = TURN_EXECUTING;
        stateStartTime = currentTime;
        
        // Thực hiện rẽ theo hướng đã xác định
        if (activeDirection == TURN_LEFT) {
          motor->turnLeft(turnSpeed);
        } else if (activeDirection == TURN_RIGHT) {
          motor->turnRight(turnSpeed);
        }
      }
      break;
      
    case TURN_EXECUTING:
      // Kiểm tra xem đã hoàn thành giai đoạn thực hiện chưa
      if (elapsedTime >= executionTime) {
        // Chuyển sang giai đoạn hoàn thành
        currentState = TURN_FINISHING;
        stateStartTime = currentTime;
        
        // Tiếp tục di chuyển
        motor->Forward(baseSpeed);
      }
      break;
      
    case TURN_FINISHING:
      // Giai đoạn hoàn thành: đảm bảo robot đã ổn định
      // Sau khoảng 100ms, kết thúc quá trình rẽ
      if (elapsedTime >= 100) {
        currentState = TURN_IDLE;
        Serial.println("Hoàn thành rẽ");
      }
      break;
      
    default:
      // Trạng thái không hợp lệ, đặt lại về IDLE
      currentState = TURN_IDLE;
      break;
  }
}

bool TurnHandler::isTurning() {
  return (currentState != TURN_IDLE);
}

bool TurnHandler::processCommand(const String& command) {
  // Xử lý lệnh TL (Turn Left)
  if (command == "TL" || command == "tl") {
    setDefaultDirection(TURN_LEFT);
    return true;
  }
  
  // Xử lý lệnh TR (Turn Right)
  if (command == "TR" || command == "tr") {
    setDefaultDirection(TURN_RIGHT);
    return true;
  }
  
  // Xử lý lệnh bật checkpoint
  if (command == "checkpoint on" || command == "cp on") {
    enableCheckpointMode(true);
    return true;
  }
  
  // Xử lý lệnh tắt checkpoint
  if (command == "checkpoint off" || command == "cp off") {
    enableCheckpointMode(false);
    return true;
  }
  
  // Lệnh không được xử lý bởi TurnHandler
  return false;
}
