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

void setup() {
    // Setup PWM cho động cơ
    ledcSetup(left_motor_channel_a, PWM_FREQ, PWM_RES);
    ledcSetup(left_motor_channel_b, PWM_FREQ, PWM_RES);
    ledcSetup(right_motor_channel_a, PWM_FREQ, PWM_RES);
    ledcSetup(right_motor_channel_b, PWM_FREQ, PWM_RES);
  
    ledcAttachPin(PWM_PIN_L_A, left_motor_channel_a);
    ledcAttachPin(PWM_PIN_L_B, left_motor_channel_b);
    ledcAttachPin(PWM_PIN_R_A, right_motor_channel_a);
    ledcAttachPin(PWM_PIN_R_B, right_motor_channel_b);
  
    // IR LED sáng
    pinMode(IR_LED_ON, OUTPUT);
    digitalWrite(IR_LED_ON, HIGH);
  
    // Cảm biến IR input
    for (int i = 0; i < 4; i++) {
      pinMode(ir_pins[i], INPUT);
    }
  }
  
  void loop() {
    readIR();
    float error = calcError();
    float correction = computePID(error);
  
    int left_speed = constrain(BASE_SPEED - correction, 0, MAX_PWM);
    int right_speed = constrain(BASE_SPEED + correction, 0, MAX_PWM);
  
    setMotorSpeed(left_speed, right_speed);
    delay(5); // delay nhỏ cho vòng lặp ổn định
  }
  
  // ------------------------ PID ------------------------
  
  float computePID(float error) {
    integral += error;
    float derivative = error - last_error;
    last_error = error;
  
    return Kp * error + Ki * integral + Kd * derivative;
  }
  
  // ------------------------ IR ------------------------
  
  void readIR() {
    for (int i = 0; i < 4; i++) {
      ir_values[i] = analogRead(ir_pins[i]);
    }
  }
  
  float calcError() {
    int sum = 0;
    int active_count = 0;
  
    for (int i = 0; i < 4; i++) {
      if (ir_values[i] > SENSOR_THRESHOLD) {
        sum += weights[i];
        active_count++;
      }
    }
  
    if (active_count == 0) return last_error; // không thấy line, giữ lỗi cũ
  
    return (float)sum / active_count;
  }
  
  // ------------------------ Motor ------------------------
  
  void setMotorSpeed(int left, int right) {
    // Left motor forward
    ledcWrite(left_motor_channel_a, left);
    ledcWrite(left_motor_channel_b, 0);
  
    // Right motor forward
    ledcWrite(right_motor_channel_a, right);
    ledcWrite(right_motor_channel_b, 0);
  }
  