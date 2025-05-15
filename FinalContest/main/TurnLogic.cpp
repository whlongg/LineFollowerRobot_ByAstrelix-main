#include "TurnLogic.h"

TurnLogic::TurnLogic() {
  reset();
}

void TurnLogic::reset() {
  isCollecting = false;
  totalError = 0.0f;
  sampleCount = 0;
  lastTurn = TURN_UNKNOWN;
  nextTurn = TURN_UNKNOWN;
}

void TurnLogic::update(int blue, int green, float error) {
  // Bắt đầu thu thập error khi phát hiện giao lộ (blue = 1 và green = 1)
  if (blue == 1 && green == 1 && !isCollecting) {
    isCollecting = true;
    totalError = 0.0f;
    sampleCount = 0;
    Serial.println("Bắt đầu thu thập thông tin rẽ...");
  }
  
  // Tiếp tục thu thập error
  if (isCollecting) {
    totalError += error;
    sampleCount++;
    
    // Kết thúc thu thập khi blue = 1 và green = 2
    if (blue == 1 && green == 2) {
      isCollecting = false;
      
      // Phân tích hướng rẽ dựa vào tổng error
      if (sampleCount > 0) {
        float averageError = totalError / sampleCount;
        
        // Xác định hướng rẽ dựa vào dấu của error trung bình
        // Error dương: robot lệch phải => rẽ trái
        // Error âm: robot lệch trái => rẽ phải
        if (averageError >= 0.0) { // Ngưỡng dương
          lastTurn = TURN_LEFT;
          nextTurn = TURN_RIGHT; // Hướng tiếp theo ngược lại
          Serial.println("Đã phát hiện rẽ TRÁI, lần sau sẽ rẽ PHẢI");
        }
        else if (averageError < 0.0) { // Ngưỡng âm
          lastTurn = TURN_RIGHT;
          nextTurn = TURN_LEFT; // Hướng tiếp theo ngược lại
          Serial.println("Đã phát hiện rẽ PHẢI, lần sau sẽ rẽ TRÁI");
        } 
        else {
          // Không xác định được hướng rẽ rõ ràng
          lastTurn = TURN_UNKNOWN;
          nextTurn = TURN_UNKNOWN;
          Serial.println("Không xác định được hướng rẽ");
        }
        
        // In thông tin debug
        Serial.print("Tổng error: ");
        Serial.print(totalError);
        Serial.print(", Số mẫu: ");
        Serial.print(sampleCount);
        Serial.print(", Error trung bình: ");
        Serial.println(averageError);
      }
    }
  }
}

TurnDirection TurnLogic::getNextTurn() {
  return nextTurn;
}

TurnDirection TurnLogic::getLastTurn() {
  return lastTurn;
}
