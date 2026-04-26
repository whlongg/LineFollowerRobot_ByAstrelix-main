#ifndef TURN_LOGIC_H
#define TURN_LOGIC_H

#include <Arduino.h>

enum TurnDirection {
  TURN_LEFT,
  TURN_RIGHT,
  TURN_UNKNOWN
};

class TurnLogic {
private:
  bool isCollecting;       // Trạng thái thu thập dữ liệu error
  float totalError;        // Tổng error thu thập được trong quá trình rẽ
  int sampleCount;         // Số lượng mẫu đã thu thập
  TurnDirection lastTurn;  // Hướng rẽ gần nhất
  TurnDirection nextTurn;  // Hướng rẽ tiếp theo dự đoán
  
public:
  // Constructor
  TurnLogic();
  
  // Cập nhật thông tin và xác định hướng rẽ
  void update(int blue, int green, float error);
  
  // Lấy hướng rẽ tiếp theo
  TurnDirection getNextTurn();
  
  // Lấy hướng rẽ vừa xác định
  TurnDirection getLastTurn();
  
  // Reset trạng thái
  void reset();
};

#endif
