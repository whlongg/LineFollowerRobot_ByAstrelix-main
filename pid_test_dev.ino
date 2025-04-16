#include <Wire.h>
#include <Adafruit_TCS34725.h>

// --- CONFIGURABLE CONSTANTS ---
#define DEBUG 1

#define LOG_INFO 0
#define LOG_ERROR 1
#define LOG_PID 2
#define LOG_CALIB 3

#define RGB_CHECKING 0                  // 1: kiểm tra màu sắc, 0: không kiểm tra
#define ENABLE_BYPASS_INTERSECTION 1    // Đặt 0 nếu muốn tắt tính năng này
#define ENABLE_HIGH_SPEED_ON_STRAIGHT 1 // 0: luôn chạy tốc độ CORNERING_SPEED

const float SCALE_RIGHT_MOTOR = 1.001;
const float Kp_default = 0.235;
const float Ki_default = 0.0;
const float Kd_default = 3;
const float ERROR_DEADBAND = 0.5; // Khoảng chết, error nhỏ hơn giá trị này sẽ coi như 0
const float INTEGRAL_MAX = 50.0f;

const int BASE_SPEED = 230;
const int MAX_STRAIGHT_SPEED = 255;         // Tốc độ tối đa mong muốn trên đường thẳng (để lại chút headroom)
const int CORNERING_SPEED = 230;            // Tốc độ cơ bản an toàn khi vào cua hoặc hiệu chỉnh mạnh
const float MAX_ERROR_FOR_HIGH_SPEED = 0.5; // Ngưỡng lỗi tối đa để còn chạy tốc độ cao (cần tune)
const float MIN_ERROR_FOR_LOW_SPEED = 1.5;  // Ngưỡng lỗi tối thiểu để chắc chắn chạy tốc độ thấp (cần tune)

// --- CONFIGURABLE THRESHOLDS ---
#define USE_DYNAMIC_THRESHOLD 1          // 1: dùng ngưỡng động từ calibration, 0: dùng ngưỡng mặc định
const int DEFAULT_BLACK_THRESHOLD = 200; // Giá trị mặc định nếu không dùng calibration
const int DEFAULT_WHITE_THRESHOLD = 400; // Giá trị mặc định nếu không dùng calibration

const int PWM_FREQ = 20000;
const int PWM_RES = 8;
const int MAX_PWM = 255;
const int CORRECTION_SCALE = 80; // Giảm để tránh giật cục (có thể thử 40-80)
const float IR_WEIGHT = 0.22;
const float RGB_FILTER_ALPHA = 0.3f;

const int SENSOR_COUNT = 4;
const int SENSOR_CALIB_SAMPLES = 32;
const int SENSOR_NORM_MAX = 1000;

// --- PIN DEFINITIONS ---
const int IR_PINS[SENSOR_COUNT] = {4, 3, 1, 0};
const int PWM_PIN_L_A = 2;
const int PWM_PIN_L_B = 10;
const int PWM_PIN_R_A = 6;
const int PWM_PIN_R_B = 5;
const int left_motor_channel_a = 0;
const int left_motor_channel_b = 1;
const int right_motor_channel_a = 2;
const int right_motor_channel_b = 3;
const int W_LED_ON = 20;
const int IR_LED_ON = 21;
const int LED_CHECK = 8;
const int BUTTON_PIN = 7;

// --- RGB Sensor Setup ---
Adafruit_TCS34725 rgbSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
uint16_t rgb_min = 65535, rgb_max = 0; // For auto-calibration

// --- IR Calibration ---
uint16_t ir_min[SENSOR_COUNT] = {65535, 65535, 65535, 65535};
uint16_t ir_max[SENSOR_COUNT] = {0, 0, 0, 0};

// --- GLOBALS ---
float Kp = Kp_default, Ki = Ki_default, Kd = Kd_default;
float last_error = 0, integral = 0;
uint16_t c_filtered = 0;
int ir_raw[SENSOR_COUNT] = {0, 0, 0, 0};
int ir_norm[SENSOR_COUNT] = {0, 0, 0, 0}; // 0-1000
float smoothed_base_speed = BASE_SPEED;
const float SPEED_SMOOTH_ALPHA = 0.22; // 0.1~0.3, càng nhỏ càng mượt

// --- LOOP TIMING ---
unsigned long lastLoop = 0;
const unsigned long LOOP_INTERVAL = 2; // ms

// --- FUNCTION DECLARATIONS ---
void setupMotors();
void setMotorSpeeds(int left, int right);
void readIRSensors();
void calibrateSensors();
float computeError();
float computePID(float error);
float computePID_dynamic(float error, float anticipation, float Kp_dyn, float Kd_dyn);
void applyMotorSpeed(float error, float correction);
void tunePID();
void debugLog(uint8_t level, String msg);
bool detectNonWhiteLine();
bool isAllIRBlack();
bool anticipationRight();
bool anticipationLeft();
bool isRightIntersection();
void bypassIntersection();
void bypassRightIntersection();
float getDynamicKp(float error, bool sharp_turn);
float getDynamicKd(float error, bool sharp_turn);
int getBlackThreshold(int sensor_idx);
int getWhiteThreshold(int sensor_idx);

// --- SETUP ---
void setup()
{
  Serial.begin(115200);
  setupMotors();
  pinMode(W_LED_ON, OUTPUT);
  pinMode(IR_LED_ON, OUTPUT);
  pinMode(LED_CHECK, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(W_LED_ON, 1);
  digitalWrite(IR_LED_ON, 1);

  if (!rgbSensor.begin())
  {
    debugLog(LOG_ERROR, "RGB sensor not found!");
    while (1)
      ;
  }
  calibrateSensors();
  debugLog(LOG_INFO, "Setup complete.");
#if DEBUG
  Serial.println("time_us,C,R,G,B,c_norm,IR1,IR2,IR3,IR4,IR1n,IR2n,IR3n,IR4n,error,P,I,D,OUT,left,right");
#endif
  // setMotorSpeeds(0, MAX_PWM); //turn right
  // delay(1000);
  // setMotorSpeeds(MAX_PWM, 0); //turn left
  // delay(1000);
  // setMotorSpeeds(0, 0);
}

// --- MAIN LOOP ---
unsigned long bypass_start_time = 0;
bool bypassing = false;

void bypassRightIntersection()
{
  // Tạo bản sao mảng ir_norm và ép IR1, IR2 thành trắng
  int ir_bypass[SENSOR_COUNT];
  for (int i = 0; i < SENSOR_COUNT; i++)
    ir_bypass[i] = ir_norm[i];
  ir_bypass[0] = SENSOR_NORM_MAX;
  ir_bypass[1] = SENSOR_NORM_MAX;

  // Tính error chỉ dựa vào IR3, IR4
  int weights[SENSOR_COUNT] = {3, 1, -1, -3}; // hoặc đúng chiều bạn đang dùng
  int sum = 0, total = 0;
  for (int i = 0; i < SENSOR_COUNT; i++)
  {
    sum += ir_bypass[i] * weights[i];
    total += ir_bypass[i];
  }
  float error = 0;
  if (total > 0)
    error = (float)sum / total;

  // PID động khi bypass (có thể tăng Kp, giảm Kd nếu muốn rẽ mạnh hơn)
  float Kp_dyn = Kp * 1.2;
  float Kd_dyn = Kd * 0.7;
  float anticipation = 1.2; // boost phải
  float correction = computePID_dynamic(error, anticipation, Kp_dyn, Kd_dyn);

  applyMotorSpeed(error, correction);
}

void loop()
{
  unsigned long t0 = micros();
  unsigned long now = millis();

  if (bypassing)
  {
    bypassRightIntersection();
    // Thoát bypass khi IR3 hoặc IR4 không còn nhận line hoặc timeout
    if (!(ir_norm[2] < getBlackThreshold(2) && ir_norm[3] < getBlackThreshold(3)) ||
        (now - bypass_start_time > 600))
    {
      bypassing = false;
    }
    return;
  }

  if (now - lastLoop >= LOOP_INTERVAL)
  {
    lastLoop = now;

    float error = computeError();
    bool sharp_turn = (ir_norm[0] < getBlackThreshold(0) && ir_norm[1] < getWhiteThreshold(1)) ||
                      (ir_norm[2] < getWhiteThreshold(2) && ir_norm[3] < getBlackThreshold(3));

    float Kp_dyn = getDynamicKp(error, sharp_turn);
    float Kd_dyn = getDynamicKd(error, sharp_turn);

    float anticipation_boost = 0;
    if (anticipationRight())
      anticipation_boost = 1.2;
    if (anticipationLeft())
      anticipation_boost = -1.2;

    // Khi phát hiện giao lộ phải, bật bypass
    if (isRightIntersection())
    {
      bypassing = true;
      bypass_start_time = now;
      return;
    }

    float correction = computePID_dynamic(error, anticipation_boost, Kp_dyn, Kd_dyn);
    applyMotorSpeed(error, correction);
    tunePID();

#if DEBUG
    unsigned long t1 = micros();
    Serial.print("looptime_us=");
    Serial.println(t1 - t0);
#endif
  }
}

// --- Unified Debug Logging ---
void debugLog(uint8_t level, String msg)
{
#if DEBUG
  switch (level)
  {
  case LOG_INFO:
    Serial.print("[INFO] ");
    break;
  case LOG_ERROR:
    Serial.print("[ERROR] ");
    break;
  case LOG_PID:
    Serial.print("[PID] ");
    break;
  case LOG_CALIB:
    Serial.print("[CALIB] ");
    break;
  default:
    Serial.print("[LOG] ");
    break;
  }
  Serial.println(msg);
#endif
}

// --- Calibration for RGB and IR Sensors ---
void calibrateSensors()
{
  debugLog(LOG_CALIB, "Starting calibration. Place robot on WHITE, then BLACK, press button each time...");
  // --- RGB Calibration ---
  for (int i = 0; i < 2; i++)
  {
    debugLog(LOG_CALIB, i == 0 ? "Calibrating WHITE..." : "Calibrating BLACK...");
    digitalWrite(LED_CHECK, HIGH);
    while (digitalRead(BUTTON_PIN) == HIGH)
      ; // Wait for button press
    delay(200);

    uint32_t c_accum = 0;
    uint16_t c, r, g, b;
    for (int s = 0; s < SENSOR_CALIB_SAMPLES; s++)
    {
      rgbSensor.getRawData(&r, &g, &b, &c);
      c_accum += c;
      delay(5);
    }
    uint16_t c_avg = c_accum / SENSOR_CALIB_SAMPLES;
    if (i == 0)
      rgb_max = c_avg;
    else
      rgb_min = c_avg;
    debugLog(LOG_CALIB, String("RGB.C avg=") + c_avg);

    // --- IR Calibration ---
    for (int s = 0; s < SENSOR_COUNT; s++)
    {
      uint32_t ir_accum = 0;
      for (int j = 0; j < SENSOR_CALIB_SAMPLES; j++)
      {
        ir_accum += analogRead(IR_PINS[s]);
        delay(2);
      }
      uint16_t ir_avg = ir_accum / SENSOR_CALIB_SAMPLES;
      if (i == 0)
        ir_max[s] = ir_avg;
      else
        ir_min[s] = ir_avg;
      debugLog(LOG_CALIB, String("IR") + (s + 1) + (i == 0 ? " max=" : " min=") + ir_avg);
    }

    while (digitalRead(BUTTON_PIN) == LOW)
      ; // Wait for release
    delay(200);
    digitalWrite(LED_CHECK, LOW);
  }
  // Prevent div by zero
  if (rgb_max == rgb_min)
    rgb_max = rgb_min + 1;
  for (int s = 0; s < SENSOR_COUNT; s++)
  {
    if (ir_max[s] == ir_min[s])
      ir_max[s] = ir_min[s] + 1;
  }
  debugLog(LOG_CALIB, "Calibration done.");
}

// --- Read and Normalize IR Sensors ---
void readIRSensors()
{
  for (int i = 0; i < SENSOR_COUNT; i++)
  {
    ir_raw[i] = analogRead(IR_PINS[i]);
    // Normalize to 0 (black) ... 1000 (white)
    ir_norm[i] = map(ir_raw[i], ir_min[i], ir_max[i], 0, SENSOR_NORM_MAX);
    ir_norm[i] = constrain(ir_norm[i], 0, SENSOR_NORM_MAX);
  }
}

// --- Error Calculation (Weighted Center, IR only) ---
float computeError()
{
  // 1. Read and normalize IR sensors
  readIRSensors();

  // 2. Weighted error calculation (centered at 0)
  // Sensor positions: 3, 1, -1, -3 (left to right)
  int weights[SENSOR_COUNT] = {3, 1, -1, -3};
  int sum = 0, total = 0;
  for (int i = 0; i < SENSOR_COUNT; i++)
  {
    sum += ir_norm[i] * weights[i];
    total += ir_norm[i];
  }
  float error = 0;
  if (total > 0)
    error = (float)sum / total;

#if DEBUG
  for (int i = 0; i < SENSOR_COUNT; i++)
  {
    Serial.print(ir_raw[i]);
    Serial.print(",");
  }
  for (int i = 0; i < SENSOR_COUNT; i++)
  {
    Serial.print(ir_norm[i]);
    Serial.print(",");
  }
  Serial.print(error, 4);
  Serial.print(",");
#endif

  return error;
}

// --- PID Controller ---
float computePID(float error)
{
  // Deadband: Nếu error nhỏ, coi như 0 để tránh robot lắc nhẹ liên tục
  if (abs(error) < ERROR_DEADBAND)
    error = 0;

  // Chỉ tích luỹ integral khi robot ở gần line (|error| nhỏ)
  if (abs(error) < 1.0)
  {
    integral += error;
    // Giới hạn integral để tránh windup
    integral = constrain(integral, -INTEGRAL_MAX, INTEGRAL_MAX);
  }

  // Đặt lại integral nếu error đổi dấu (qua line) hoặc error rất lớn (mất line)
  if ((error * last_error < 0) || (abs(error) > 2.5))
  {
    integral = 0;
  }

  bool sharp_turn = (ir_norm[0] < 200 && ir_norm[1] < 400) || (ir_norm[2] < 400 && ir_norm[3] < 200);
  float dynamic_Kp = getDynamicKp(error, sharp_turn);
  float dynamic_Kd = getDynamicKd(error, sharp_turn);

  float derivative = error - last_error;
  float output = dynamic_Kp * error + Ki * integral + dynamic_Kd * derivative;
  last_error = error;
#if DEBUG
  Serial.print(dynamic_Kp * error, 4);
  Serial.print(",");
  Serial.print(Ki * integral, 4);
  Serial.print(",");
  Serial.print(dynamic_Kd * derivative, 4);
  Serial.print(",");
  Serial.print(output, 4);
  Serial.print(",");
#endif
  return output;
}

// PID động với anticipation
float computePID_dynamic(float error, float anticipation, float Kp_dyn, float Kd_dyn)
{
  static float last_error = 0, integral = 0;
  if (abs(error) < ERROR_DEADBAND)
    error = 0;
  if (abs(error) < 1.0)
  {
    integral += error;
    integral = constrain(integral, -INTEGRAL_MAX, INTEGRAL_MAX);
  }
  if ((error * last_error < 0) || (abs(error) > 2.5))
    integral = 0;
  float derivative = error - last_error;
  float output = Kp_dyn * error + Ki * integral + Kd_dyn * derivative + anticipation;
  last_error = error;
  return output;
}

// --- Motor Control (PID-based, smooth, DYNAMIC SPEED, auto slow on sharp turn) ---
void applyMotorSpeed(float error, float correction)
{
  float abs_error = abs(error);
  int target_base_speed;

  // Pattern giảm tốc chỉ khi thực sự cần thiết (ví dụ: 2 cảm biến ngoài cùng đều nhận line)
  bool sharp_turn = (ir_norm[0] < 200 && ir_norm[1] < 400) || (ir_norm[2] < 400 && ir_norm[3] < 200);

#if ENABLE_HIGH_SPEED_ON_STRAIGHT
  if (sharp_turn)
  {
    target_base_speed = CORNERING_SPEED - 30;
  }
  else if (abs_error <= MAX_ERROR_FOR_HIGH_SPEED)
  {
    target_base_speed = MAX_STRAIGHT_SPEED;
  }
  else if (abs_error >= MIN_ERROR_FOR_LOW_SPEED)
  {
    target_base_speed = CORNERING_SPEED;
  }
  else
  {
    target_base_speed = map(abs_error * 100,
                            MAX_ERROR_FOR_HIGH_SPEED * 100,
                            MIN_ERROR_FOR_LOW_SPEED * 100,
                            MAX_STRAIGHT_SPEED,
                            CORNERING_SPEED);
    target_base_speed = constrain(target_base_speed, CORNERING_SPEED, MAX_STRAIGHT_SPEED);
  }
#else
  target_base_speed = CORNERING_SPEED;
#endif

  // Exponential smoothing cho tốc độ cơ sở
  smoothed_base_speed = SPEED_SMOOTH_ALPHA * target_base_speed + (1.0 - SPEED_SMOOTH_ALPHA) * smoothed_base_speed;

  // Correction tối ưu cho cua gắt: tăng hệ số khi cua mạnh để bám sát mép line
  float dynamic_correction_scale = CORRECTION_SCALE;
  if (sharp_turn)
    dynamic_correction_scale *= 1.25; // Tăng correction khi cua gắt

  correction = constrain(correction, -2.5, 2.5);

  int left_speed = smoothed_base_speed + correction * dynamic_correction_scale;
  int right_speed = smoothed_base_speed - correction * dynamic_correction_scale;
  left_speed = constrain(left_speed, 0, MAX_PWM);
  right_speed = constrain(right_speed, 0, MAX_PWM);

  setMotorSpeeds(left_speed, right_speed);

#if DEBUG
  Serial.print(left_speed);
  Serial.print(",");
  Serial.println(right_speed);
#endif
}

// --- PID Tuning via Serial/Bluetooth ---
void tunePID()
{
  // Có thể dùng Serial Bluetooth (HC-05/06 hoặc BLE Serial trên ESP32)
  if (Serial.available())
  {
    String cmd = Serial.readStringUntil('\n');
    if (cmd.startsWith("kp="))
      Kp = cmd.substring(3).toFloat();
    else if (cmd.startsWith("ki="))
      Ki = cmd.substring(3).toFloat();
    else if (cmd.startsWith("kd="))
      Kd = cmd.substring(3).toFloat();
    debugLog(LOG_PID, String("Kp: ") + Kp + " Ki: " + Ki + " Kd: " + Kd);
  }
}

// --- Motor Setup ---
void setupMotors()
{
  ledcAttachChannel(PWM_PIN_L_A, PWM_FREQ, PWM_RES, left_motor_channel_a);
  ledcAttachChannel(PWM_PIN_L_B, PWM_FREQ, PWM_RES, left_motor_channel_b);
  ledcAttachChannel(PWM_PIN_R_A, PWM_FREQ, PWM_RES, right_motor_channel_a);
  ledcAttachChannel(PWM_PIN_R_B, PWM_FREQ, PWM_RES, right_motor_channel_b);
}

// --- Motor Speed Control ---
void setMotorSpeeds(int left, int right)
{
  right = right * SCALE_RIGHT_MOTOR; // Apply scaling factor to right motor
  ledcWriteChannel(left_motor_channel_a, left);
  ledcWriteChannel(left_motor_channel_b, 0);
  ledcWriteChannel(right_motor_channel_a, right);
  ledcWriteChannel(right_motor_channel_b, 0);
}

// --- Color Detection (use in loop if needed) ---
bool detectNonWhiteLine()
{
  uint16_t c, r, g, b;
  rgbSensor.getRawData(&r, &g, &b, &c);
  // Exponential filter nếu muốn
  c_filtered = (uint16_t)(RGB_FILTER_ALPHA * c + (1.0f - RGB_FILTER_ALPHA) * c_filtered);
  int c_norm = map(c_filtered, rgb_min, rgb_max, 0, SENSOR_NORM_MAX);
  c_norm = constrain(c_norm, 0, SENSOR_NORM_MAX);
  // Nếu c_norm < 700 (ví dụ), tức là không phải nền trắng
  return (c_norm < 700);
}

// --- Intersection Bypass: hard right (góc vuông) ---
bool isAllIRBlack()
{
  for (int i = 0; i < SENSOR_COUNT; i++)
  {
    if (ir_norm[i] > getBlackThreshold(i))
      return false;
  }
  return true;
}

void bypassIntersection()
{
  // Dừng trái, phải chạy tốc độ cao, delay lâu hơn
  setMotorSpeeds(0, MAX_PWM);
  delay(600); // Có thể tăng lên 400ms nếu cần
}

// --- Dynamic PID ---
float getDynamicKp(float error, bool sharp_turn)
{
  if (sharp_turn)
    return Kp * 1.2; // tăng Kp khi cua gắt
  if (abs(error) < 0.3)
    return Kp * 0.7; // giảm Kp khi đi thẳng
  return Kp;
}

float getDynamicKd(float error, bool sharp_turn)
{
  if (sharp_turn)
    return Kd * 0.7; // giảm Kd khi cua gắt để robot linh hoạt hơn
  if (abs(error) < 0.3)
    return Kd * 1.3; // tăng Kd khi đi thẳng để robot ổn định, không lắc
  return Kd;
}

// --- Anticipation and Intersection Detection ---
bool anticipationRight()
{
  // IR3 (phải trong) nhận line, IR4 (phải ngoài) chưa nhận line
  return (ir_norm[2] < getBlackThreshold(2) && ir_norm[3] > getWhiteThreshold(3));
}

bool anticipationLeft()
{
  // IR2 (trái trong) nhận line, IR1 (trái ngoài) chưa nhận line
  return (ir_norm[1] < getBlackThreshold(1) && ir_norm[0] > getWhiteThreshold(0));
}

bool isRightIntersection()
{
  return (ir_norm[2] < getBlackThreshold(2) && ir_norm[3] < getBlackThreshold(3));
}

// --- Threshold Calculation ---
int getBlackThreshold(int sensor_idx)
{
#if USE_DYNAMIC_THRESHOLD
  // Ngưỡng black = trung bình giữa min và min+20% khoảng min-max
  return ir_min[sensor_idx] + (ir_max[sensor_idx] - ir_min[sensor_idx]) * 0.2;
#else
  return DEFAULT_BLACK_THRESHOLD;
#endif
}

int getWhiteThreshold(int sensor_idx)
{
#if USE_DYNAMIC_THRESHOLD
  // Ngưỡng white = min + 40% khoảng min-max
  return ir_min[sensor_idx] + (ir_max[sensor_idx] - ir_min[sensor_idx]) * 0.4;
#else
  return DEFAULT_WHITE_THRESHOLD;
#endif
}