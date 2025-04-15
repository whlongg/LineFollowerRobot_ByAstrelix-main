#include <Wire.h>
#include <Adafruit_TCS34725.h>

// --- CONFIGURABLE CONSTANTS ---
#define DEBUG 1

#define LOG_INFO   0
#define LOG_ERROR  1
#define LOG_PID    2
#define LOG_CALIB  3

const float SCALE_RIGHT_MOTOR = 0.999;
const float Kp_default = 0.4;
const float Ki_default = 0.0;
const float Kd_default = 0.11;
const float INTEGRAL_MAX = 50.0f;

const int PWM_FREQ = 20000;
const int PWM_RES = 8;
const int MAX_PWM = 255;
const int BASE_SPEED = 180;
const int CORRECTION_SCALE = 100;
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
uint16_t ir_min[SENSOR_COUNT] = {65535,65535,65535,65535};
uint16_t ir_max[SENSOR_COUNT] = {0,0,0,0};

// --- GLOBALS ---
float Kp = Kp_default, Ki = Ki_default, Kd = Kd_default;
float last_error = 0, integral = 0;
uint16_t c_filtered = 0;
int ir_raw[SENSOR_COUNT] = {0, 0, 0, 0};
int ir_norm[SENSOR_COUNT] = {0, 0, 0, 0}; // 0-1000

// --- LOOP TIMING ---
unsigned long lastLoop = 0;
const unsigned long LOOP_INTERVAL = 10; // ms

// --- FUNCTION DECLARATIONS ---
void setupMotors();
void setMotorSpeeds(int left, int right);
void readIRSensors();
void calibrateSensors();
float computeError();
float computePID(float error);
void applyMotorSpeed(float correction);
void tunePID();
void debugLog(uint8_t level, String msg);
bool detectNonWhiteLine();

// --- SETUP ---
void setup() {
  Serial.begin(115200);
  setupMotors();
  pinMode(W_LED_ON, OUTPUT);
  pinMode(IR_LED_ON, OUTPUT);
  pinMode(LED_CHECK, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(W_LED_ON, 1);
  digitalWrite(IR_LED_ON, 1);

  if (!rgbSensor.begin()) {
    debugLog(LOG_ERROR, "RGB sensor not found!");
    while (1);
  }
  calibrateSensors();
  debugLog(LOG_INFO, "Setup complete.");
  #if DEBUG
    Serial.println("time_us,C,R,G,B,c_norm,IR1,IR2,IR3,IR4,IR1n,IR2n,IR3n,IR4n,error,P,I,D,OUT,left,right");
  #endif
}

// --- MAIN LOOP ---
void loop() {
  unsigned long now = millis();
  if (now - lastLoop >= LOOP_INTERVAL) {
    lastLoop = now;

    float error = computeError();
    float correction = computePID(error);

    // Nếu phát hiện line màu khác trắng (ví dụ vạch dừng)
    if (detectNonWhiteLine()) {
      debugLog(LOG_INFO, "Non-white line detected!");
      // Xử lý đặc biệt, ví dụ: stop(); return;
    }

    applyMotorSpeed(correction);
    tunePID();
  }
}

// --- Unified Debug Logging ---
void debugLog(uint8_t level, String msg) {
#if DEBUG
  switch(level) {
    case LOG_INFO:   Serial.print("[INFO] "); break;
    case LOG_ERROR:  Serial.print("[ERROR] "); break;
    case LOG_PID:    Serial.print("[PID] "); break;
    case LOG_CALIB:  Serial.print("[CALIB] "); break;
    default:         Serial.print("[LOG] "); break;
  }
  Serial.println(msg);
#endif
}

// --- Calibration for RGB and IR Sensors ---
void calibrateSensors() {
  debugLog(LOG_CALIB, "Starting calibration. Place robot on WHITE, then BLACK, press button each time...");
  // --- RGB Calibration ---
  for (int i = 0; i < 2; i++) {
    debugLog(LOG_CALIB, i == 0 ? "Calibrating WHITE..." : "Calibrating BLACK...");
    digitalWrite(LED_CHECK, HIGH);
    while (digitalRead(BUTTON_PIN) == HIGH); // Wait for button press
    delay(200);

    uint32_t c_accum = 0;
    uint16_t c, r, g, b;
    for (int s = 0; s < SENSOR_CALIB_SAMPLES; s++) {
      rgbSensor.getRawData(&r, &g, &b, &c);
      c_accum += c;
      delay(5);
    }
    uint16_t c_avg = c_accum / SENSOR_CALIB_SAMPLES;
    if (i == 0) rgb_max = c_avg;
    else rgb_min = c_avg;
    debugLog(LOG_CALIB, String("RGB.C avg=") + c_avg);

    // --- IR Calibration ---
    for (int s = 0; s < SENSOR_COUNT; s++) {
      uint32_t ir_accum = 0;
      for (int j = 0; j < SENSOR_CALIB_SAMPLES; j++) {
        ir_accum += analogRead(IR_PINS[s]);
        delay(2);
      }
      uint16_t ir_avg = ir_accum / SENSOR_CALIB_SAMPLES;
      if (i == 0) ir_max[s] = ir_avg;
      else ir_min[s] = ir_avg;
      debugLog(LOG_CALIB, String("IR") + (s+1) + (i==0?" max=":" min=") + ir_avg);
    }

    while (digitalRead(BUTTON_PIN) == LOW); // Wait for release
    delay(200);
    digitalWrite(LED_CHECK, LOW);
  }
  // Prevent div by zero
  if (rgb_max == rgb_min) rgb_max = rgb_min + 1;
  for (int s = 0; s < SENSOR_COUNT; s++) {
    if (ir_max[s] == ir_min[s]) ir_max[s] = ir_min[s] + 1;
  }
  debugLog(LOG_CALIB, "Calibration done.");
}

// --- Read and Normalize IR Sensors ---
void readIRSensors() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    ir_raw[i] = analogRead(IR_PINS[i]);
    // Normalize to 0 (black) ... 1000 (white)
    ir_norm[i] = map(ir_raw[i], ir_min[i], ir_max[i], 0, SENSOR_NORM_MAX);
    ir_norm[i] = constrain(ir_norm[i], 0, SENSOR_NORM_MAX);
  }
}

// --- Error Calculation (Weighted Center, IR only) ---
float computeError() {
  // 1. Read and normalize IR sensors
  readIRSensors();

  // 2. Weighted error calculation (centered at 0)
  // Sensor positions: 3, 1, -1, -3 (left to right)
  int weights[SENSOR_COUNT] = {3, 1, -1, -3};
  int sum = 0, total = 0;
  for (int i = 0; i < SENSOR_COUNT; i++) {
    sum += ir_norm[i] * weights[i];
    total += ir_norm[i];
  }
  float error = 0;
  if (total > 0) error = (float)sum / total;

  #if DEBUG
    for (int i = 0; i < SENSOR_COUNT; i++) {
      Serial.print(ir_raw[i]); Serial.print(",");
    }
    for (int i = 0; i < SENSOR_COUNT; i++) {
      Serial.print(ir_norm[i]); Serial.print(",");
    }
    Serial.print(error, 4); Serial.print(",");
  #endif

  return error;
}

// --- PID Controller ---
float computePID(float error) {
  integral += error;
  integral = constrain(integral, -INTEGRAL_MAX, INTEGRAL_MAX);
  float derivative = error - last_error;
  float output = Kp * error + Ki * integral + Kd * derivative;
  last_error = error;
  #if DEBUG
    Serial.print(Kp * error, 4); Serial.print(",");
    Serial.print(Ki * integral, 4); Serial.print(",");
    Serial.print(Kd * derivative, 4); Serial.print(",");
    Serial.print(output, 4); Serial.print(",");
  #endif
  return output;
}

// --- Motor Control (PID-based, smooth) ---
void applyMotorSpeed(float correction) {
  // Correction is in range ~[-3,3], scale accordingly
  int left_speed = BASE_SPEED + correction * CORRECTION_SCALE;
  int right_speed = BASE_SPEED - correction * CORRECTION_SCALE;
  left_speed = constrain(left_speed, 0, MAX_PWM);
  right_speed = constrain(right_speed, 0, MAX_PWM);
  setMotorSpeeds(left_speed, right_speed);
  #if DEBUG
    Serial.print(left_speed); Serial.print(",");
    Serial.println(right_speed);
  #endif
}

// --- PID Tuning via Serial ---
void tunePID() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    if (cmd.startsWith("kp=")) Kp = cmd.substring(3).toFloat();
    else if (cmd.startsWith("ki=")) Ki = cmd.substring(3).toFloat();
    else if (cmd.startsWith("kd=")) Kd = cmd.substring(3).toFloat();
    debugLog(LOG_PID, String("Kp: ") + Kp + " Ki: " + Ki + " Kd: " + Kd);
  }
}

// --- Motor Setup ---
void setupMotors() {
  ledcAttachChannel(PWM_PIN_L_A, PWM_FREQ, PWM_RES, left_motor_channel_a);
  ledcAttachChannel(PWM_PIN_L_B, PWM_FREQ, PWM_RES, left_motor_channel_b);
  ledcAttachChannel(PWM_PIN_R_A, PWM_FREQ, PWM_RES, right_motor_channel_a);
  ledcAttachChannel(PWM_PIN_R_B, PWM_FREQ, PWM_RES, right_motor_channel_b);
}

// --- Motor Speed Control ---
void setMotorSpeeds(int left, int right) {
    right = right * SCALE_RIGHT_MOTOR; // Apply scaling factor to right motor
    ledcWriteChannel(left_motor_channel_a, left);
    ledcWriteChannel(left_motor_channel_b, 0);
    ledcWriteChannel(right_motor_channel_a, right);
    ledcWriteChannel(right_motor_channel_b, 0);
}

// --- Color Detection (use in loop if needed) ---
bool detectNonWhiteLine() {
  uint16_t c, r, g, b;
  rgbSensor.getRawData(&r, &g, &b, &c);
  // Exponential filter nếu muốn
  c_filtered = (uint16_t)(RGB_FILTER_ALPHA * c + (1.0f - RGB_FILTER_ALPHA) * c_filtered);
  int c_norm = map(c_filtered, rgb_min, rgb_max, 0, SENSOR_NORM_MAX);
  c_norm = constrain(c_norm, 0, SENSOR_NORM_MAX);
  // Nếu c_norm < 700 (ví dụ), tức là không phải nền trắng
  return (c_norm < 700);
}