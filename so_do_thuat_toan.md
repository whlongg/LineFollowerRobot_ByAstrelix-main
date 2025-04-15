#         ROBOT LINE FOLLOWER - SƠ ĐỒ THUẬT TOÁN(PD)

- Nhớ mở full màn hình để không bị lỗi khi xem.

```
// PID constants (tune as needed)
float Kp = 0.18;
float Ki = 0.0;
float Kd = 0.11;
int weights[4] = {-3, -1, 1, 3};

// --- CONFIGURABLE CONSTANTS ---
#define PWM_FREQ 20000
#define PWM_RES 8 // 8-bit (0-255)
#define MAX_PWM 255

#define BASE_SPEED 180   // Base speed for both motors (0-255)
#define SENSOR_THRESHOLD 1200 // Adjust based on your sensor and surface

#define IR1_PIN 4
#define IR2_PIN 3
#define IR3_PIN 1
#define IR4_PIN 0

#define PWM_PIN_L_A 2
#define PWM_PIN_L_B 10
#define PWM_PIN_R_A 6
#define PWM_PIN_R_B 5

#define left_motor_channel_a 0
#define left_motor_channel_b 1
#define right_motor_channel_a 2
#define right_motor_channel_b 3

#define W_LED_ON 20
#define IR_LED_ON 21

// --- GLOBALS ---
int ir_pins[4] = {IR1_PIN, IR2_PIN, IR3_PIN, IR4_PIN};
int ir_values[4] = {0, 0, 0, 0};
float last_error = 0;
float integral = 0;
```

## Sơ đồ khối

![Sơ đồ khối](https://www.mermaidchart.com/raw/29f7851d-f85f-4f28-8475-0a265501aec3?theme=light&version=v0.1&format=svg)

```
+------------------------+      +-----------------------+      +--------------------------+
|    [ SensorReader ]    |<-----|       Môi Trường      |<-----|       [ Động Cơ ]        |
|  (Đọc 4 IR Analog)     |      | (Line, Màu, Giao lộ)  |      |      (PWM, DIR)          |
+------------------------+      +-----------------------+      +--------------------------+
          |                                                           ^
          | Raw Analog Values                                         | Final PWM & DIR
          V                                                           |
+------------------------+                                          +--------------------------+
|  [ SensorProcessor ]   |                                          |   [ MotorController ]    |
|  (Digital hóa ->       |                                          |   (Tính PWM cuối cùng,   |
|   [0,1,1,0] dựa trên   |                                          |    Giới hạn tốc độ,      |
|   Threshold)           |                                          |    Set DIR)              |
+------------------------+                                          +--------------------------+
          |                                                                   ^
          | Digital Values [0,1,1,0]                                          | Motor Command
          V                                                                   | (Use PD / Use Maneuver)
+-----------------------------+       +--------------------------+            |
| [ LinePositionCalculator ]  |------>|   [ PatternDetector ]    |------------+
| (Tính 'error',              |       |   (Phát hiện Giao lộ,    |            |
|  lineStatus [ALL_BLACK,..]) |       |    Mất line lâu, Bypass) |            |
+-----------------------------+       +--------------------------+            |
          |                                    |                              |
          | error                              | DetectedEvent                |
          V                                    V                              |
+-----------------------------+      +--------------------------+             |
| [ Controller (PD Recom.)]   |----->|     [ StateManager ]     |<------------+
| (Tính: Kp*error + Kd*deriv) |      |     (Quyết định State:   |
| Output: pdCorrection        |      |      FOLLOWING_PD,       |
+-----------------------------+      |      CROSSING, LOST,     |
          |                          |      BYPASS...)          |
          | pdCorrection             |     Output: activeMode   |
          |                          +--------------------------+
          |                                    |
          |                                    | Trigger Maneuver
          |                                    V
          +---------------------------------->+--------------------------+
                                              |   [ ManeuverLogic ]      |
                                              |   (Logic cho Bypass,     |
                                              |    Tìm kiếm, Đi thẳng)   |
                                              | Output: maneuverCmd      |
                                              +--------------------------+
                                                         |
                                                         | maneuverCmd
                                                         V
                                         [ MotorController ] (Nhận từ đây nếu state đặc biệt)


---------------------------------------------------------------------------------
                      Luồng Gỡ Lỗi (Tùy chọn)
---------------------------------------------------------------------------------
[LinePosCalc] --error,status--> [DebugLogger]
[StateMgr] ----currentState----> [DebugLogger] (Serial Print, LED)
[MotorCtrl] ---finalPWMs------> [DebugLogger]
[SensorProc] --digitalVals-----> [DebugLogger]
```

