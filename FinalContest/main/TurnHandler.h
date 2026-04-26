#ifndef TURN_HANDLER_H
#define TURN_HANDLER_H

#include <Arduino.h>
#include "TurnLogic.h"
#include "MotorControl.h"

// Định nghĩa các trạng thái cho máy trạng thái xử lý rẽ
enum TurnState {
  TURN_IDLE,            // Không rẽ
  TURN_PREPARE,         // Chuẩn bị rẽ (giảm tốc)
  TURN_EXECUTING,       // Đang thực hiện rẽ
  TURN_FINISHING        // Hoàn thành rẽ
};

// Lớp xử lý rẽ
class TurnHandler {
private:
  MotorControl* motor;
  TurnLogic* turnLogic;
  bool checkpointMode;      // Chế độ checkpoint - khi true sẽ rẽ theo hướng mặc định
  TurnDirection defaultDirection; // Hướng rẽ mặc định khi gặp giao lộ
  int turnSpeed;           // Tốc độ khi rẽ
  int baseSpeed;           // Tốc độ cơ bản
  
  // Biến cho máy trạng thái
  TurnState currentState;   // Trạng thái hiện tại
  TurnDirection activeDirection; // Hướng rẽ đang thực hiện
  unsigned long stateStartTime; // Thời điểm bắt đầu trạng thái
  unsigned long prepareTime;    // Thời gian cho giai đoạn chuẩn bị (ms)
  unsigned long executionTime;  // Thời gian cho giai đoạn thực hiện (ms)

public:
  // Constructor
  TurnHandler(MotorControl* motorControl, TurnLogic* logic);
  
  // Thiết lập các thông số
  void setTurnSpeed(int speed);
  void setBaseSpeed(int speed);
  void setTurnTiming(unsigned long prepare, unsigned long execute);
  
  // Bật/tắt chế độ checkpoint
  void enableCheckpointMode(bool enable);
  bool isCheckpointModeEnabled();
  
  // Thiết lập hướng rẽ mặc định
  void setDefaultDirection(TurnDirection direction);
  TurnDirection getDefaultDirection();
  
  // Bắt đầu quá trình rẽ (không chặn)
  void startTurn(TurnDirection direction);
  
  // Cập nhật trạng thái máy trạng thái (gọi trong mỗi vòng lặp)
  void update();
  
  // Kiểm tra xem có đang trong quá trình rẽ không
  bool isTurning();
  
  // Xử lý lệnh rẽ từ command
  bool processCommand(const String& command);
};

#endif
