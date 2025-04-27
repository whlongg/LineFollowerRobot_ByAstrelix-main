#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// --- CONFIGURABLE CONSTANTS ---
#define DEBUG 0

#define LOG_INFO 0
#define LOG_ERROR 1
#define LOG_PID 2
#define LOG_CALIB 3

#define RGB_CHECKING 0                  // 1: kiểm tra màu sắc
#define ENABLE_BYPASS_INTERSECTION 1    // Đặt 0 nếu tắt bypass
#define ENABLE_HIGH_SPEED_ON_STRAIGHT 1 // 0: Không kích hoạt HIGH_SPEED
#define THREADSOLD_BLACK 300

const float SCALE_RIGHT_MOTOR = 1.002;
const float Kp_default = 3.66;
const float Ki_default = 0;
const float Kd_default = 39;
const float INTEGRAL_MAX = 70.0f;

// --- Tunable Parameters (Defaults) ---
int max_straight_speed = 250;             // Max speed on straights (leave headroom)
int cornering_speed = 210;                // Base speed for corners or large corrections
float max_error_for_high_speed = 2.5;     // Error must be BELOW this to use max_straight_speed. Lower value = stricter condition for high speed.
float min_error_for_low_speed = 3.0;      // Error must be ABOVE this to force cornering_speed. Higher value = more tolerant before slowing down.
int sharp_turn_speed_reduction = 60;      // Amount to reduce speed further during detected sharp turns.
int correction_scale = 40;                // Scales PID output to motor speed difference (Tune: 40-80)

// Intersection state timing & speeds
unsigned long state_start_time = 0; // Timer for state durations
const unsigned long BRAKING_DURATION_MS = 30;
const unsigned long MIN_TURN_DURATION_MS = 200; // Turn at least this long before searching
const int INTERSECTION_TURN_SPEED = 255;
const int SEARCHING_TURN_SPEED_LEFT = 200;  // Adjust as needed
const int SEARCHING_TURN_SPEED_RIGHT = 80; // Slower search turn
const int LINE_DETECT_THRESHOLD = 300; // IR threshold to detect line

// --- TIMING ---
unsigned long lastLoop = 0;
const unsigned long LOOP_INTERVAL = 2; // loop timing ms
const unsigned long BLE_LOG_INTERVAL_MS = 5; //BLE Log timing ms

// --- Fixed Constants ---
const int PWM_FREQ = 20000;
const int PWM_RES = 8;
const int MAX_PWM = 255;
const float IR_WEIGHT = 0.22;
const float RGB_FILTER_ALPHA = 0.3f;
const float D_FILTER_ALPHA = 0.5; // <<----- THÊM MỚI (Hệ số lọc cho Derivative, thử nghiệm 0.3 - 0.7)

// --- BLE Definitions ---
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_RX "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID_TX "beb5483e-36e1-4688-b7f5-ea07361b26a9" // For sending back PID values
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristicRX = NULL;
BLECharacteristic *pCharacteristicTX = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

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

// --- Robot States ---
enum RobotState
{
  STATE_LINE_FOLLOWING,
  STATE_INTERSECTION_DETECTED, // Optional pre-state
  STATE_INTERSECTION_BRAKING,
  STATE_INTERSECTION_TURNING,
  STATE_INTERSECTION_SEARCHING_LINE
};

// --- GLOBALS ---
float Kp = Kp_default, Ki = Ki_default, Kd = Kd_default;
float last_error = 0, integral = 0;
float filtered_derivative = 0; //gia tri vi phan da duoc loc
uint16_t c_filtered = 0;
RobotState current_state = STATE_LINE_FOLLOWING;

// --- BLE Logging Globals ---
float currentP = 0, currentI = 0, currentD = 0;
int currentLeftSpeed = 0, currentRightSpeed = 0;
unsigned long lastBleLogTime = 0; // Timer for throttling BLE logs

int ir_raw[SENSOR_COUNT] = {0, 0, 0, 0};
int ir_norm[SENSOR_COUNT] = {0, 0, 0, 0}; // 0-1000

// --- FUNCTION DECLARATIONS ---
void setupMotors();
void setMotorSpeeds(int left, int right);
void readIRSensors();
void calibrateSensors();
float computeError();
float computePID(float error);
void applyMotorSpeed(float error, float correction);
void handleBLECommands(String value); // Changed parameter type to Arduino String
void setupBLE();
void notifyTuningValues(); // Renamed: Function to send ALL tuning values over BLE
void debugLog(uint8_t level, String msg);
bool detectNonWhiteLine();
bool isAllIRBlack();

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
  setupBLE(); // Initialize BLE
  debugLog(LOG_INFO, "Setup complete. BLE Initialized.");
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
void loop()
{
  unsigned long t0 = micros();
  unsigned long now = millis();

  // Enforce loop timing
  if (now - lastLoop < LOOP_INTERVAL) {
    return;
  }
  lastLoop = now;

  // Read sensors
  readIRSensors();

  // State machine
  switch (current_state) {
    case STATE_LINE_FOLLOWING:
    {
      float error = computeError();
      float correction = computePID(error);
      applyMotorSpeed(error, correction);

      // Detect intersection and start braking
      if (ENABLE_BYPASS_INTERSECTION && isAllIRBlack()) {
        debugLog(LOG_INFO, "Intersection detected, starting brake.");
        setMotorSpeeds(-MAX_PWM, -MAX_PWM);
        current_state = STATE_INTERSECTION_BRAKING;
        state_start_time = millis();
        integral = 0;
        last_error = 0;
        filtered_derivative = 0;
      }
      break;
    }

    case STATE_INTERSECTION_BRAKING:
    {
      if (millis() - state_start_time >= BRAKING_DURATION_MS) {
        debugLog(LOG_INFO, "Braking complete, starting turn.");
        setMotorSpeeds(INTERSECTION_TURN_SPEED, 0);
        current_state = STATE_INTERSECTION_TURNING;
        state_start_time = millis();
      }
      break;
    }

    case STATE_INTERSECTION_TURNING:
    {
      if (millis() - state_start_time >= MIN_TURN_DURATION_MS) {
        debugLog(LOG_INFO, "Min turn duration met, searching for line.");
        current_state = STATE_INTERSECTION_SEARCHING_LINE;
        state_start_time = millis();
      }
      break;
    }

    case STATE_INTERSECTION_SEARCHING_LINE:
    {
      setMotorSpeeds(SEARCHING_TURN_SPEED_LEFT, SEARCHING_TURN_SPEED_RIGHT);

      if (ir_norm[1] > LINE_DETECT_THRESHOLD || ir_norm[2] > LINE_DETECT_THRESHOLD) {
        debugLog(LOG_INFO, "Line found! Resuming line following.");
        current_state = STATE_LINE_FOLLOWING;
      }
      break;
    }
  }

  // BLE connection management
  if (deviceConnected && !oldDeviceConnected) {
    // Connected
    oldDeviceConnected = deviceConnected;
    debugLog(LOG_INFO, "BLE device connected");
  }
  if (!deviceConnected && oldDeviceConnected) {
    // Disconnected
    oldDeviceConnected = deviceConnected;
    pServer->startAdvertising(); // restart advertising
    debugLog(LOG_INFO, "BLE device disconnected, start advertising");
  }

  // --- BLE Real-time Data Logging (CSV Format) - Throttled ---
  if (deviceConnected && pCharacteristicTX != NULL && (now - lastBleLogTime >= BLE_LOG_INTERVAL_MS))
  {
    lastBleLogTime = now; // Update the last log time

    // New Format: IR_norm(ir0,ir1,ir2,ir3),Pid(kp,ki,kd),Error,Speed(left,right)\n
    String logData = String(millis()) + "," +
                String(ir_norm[0]) + "," +
                String(ir_norm[1]) + "," +
                String(ir_norm[2]) + "," +
                String(ir_norm[3]) + "," +
                String(currentP, 4) + "," +
                String(currentI, 4) + "," +
                String(currentD, 4) + "," +
                String(last_error, 4) + "," +
                String(currentLeftSpeed) + "," +
                String(currentRightSpeed) + "\n";

    // Send data via BLE Notification
    // Note: Ensure MTU is large enough or handle potential fragmentation if needed.
    // For typical terminal apps, sending line by line should be okay.
    pCharacteristicTX->setValue(logData.c_str());
    pCharacteristicTX->notify();
  }

#if DEBUG
  unsigned long t1 = micros();
  Serial.print("looptime_us=");
  Serial.println(t1 - t0);
#endif
}

// --- Unified Debug Logging ---
void debugLog(uint8_t level, String msg)
{
#if DEBUG
  String prefix;
  switch (level)
  {
  case LOG_INFO:
    prefix = "[INFO] ";
    break;
  case LOG_ERROR:
    prefix = "[ERROR] ";
    break;
  case LOG_PID:
    prefix = "[PID] ";
    break;
  case LOG_CALIB:
    prefix = "[CALIB] ";
    break;
  default:
    prefix = "[LOG] ";
    break;
  }
  String fullMsg = prefix + msg;
  Serial.println(fullMsg);

  // --- REMOVED BLE LOGGING FROM HERE ---
  // BLE logging will now be done separately in the main loop with CSV data
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
  readIRSensors();
  int weights[SENSOR_COUNT] = {2, 1, -1, -2};
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
  // Chỉ tích luỹ integral khi robot ở gần line (|error| nhỏ)
  if (abs(error) < 1.0)
  {
    integral += error;
    // Giới hạn integral để tránh windup
    integral = constrain(integral, -INTEGRAL_MAX, INTEGRAL_MAX);
  }
  if ((error * last_error < 0) || (abs(error) > 2.5))
    integral = 0;

  float raw_derivative = error - last_error;
  filtered_derivative = D_FILTER_ALPHA * raw_derivative + (1.0 - D_FILTER_ALPHA) * filtered_derivative; //filter D term

  // Store components for logging
  currentP = Kp * error;
  currentI = Ki * integral;
  currentD = Kd * filtered_derivative;

  float output = currentP + currentI + currentD;
  last_error = error;

#if DEBUG
  Serial.print(currentP, 4);
  Serial.print(",");
  Serial.print(currentI, 4);
  Serial.print(",");
  Serial.print(currentD, 4);
  Serial.print(",");
  Serial.print(output, 4);
  Serial.print(",");
#endif
  return output;
}

// --- Motor Control (PID-based, smooth, DYNAMIC SPEED, auto slow on sharp turn) ---
void applyMotorSpeed(float error, float correction)
{
  float abs_error = abs(error);
  int current_base_speed;

  // Phát hiện góc cua gắt: 2 cảm biến ngoài cùng đều nhận line (giá trị thấp)
  bool sharp_turn = (ir_norm[0] < 200 && ir_norm[1] < 400) || (ir_norm[2] < 400 && ir_norm[3] < 200);

#if ENABLE_HIGH_SPEED_ON_STRAIGHT
  if (sharp_turn)
    current_base_speed = cornering_speed - sharp_turn_speed_reduction;   
  else if (abs_error <= max_error_for_high_speed)   
    current_base_speed = max_straight_speed;   
  else if (abs_error >= min_error_for_low_speed)   
    current_base_speed = cornering_speed;   
  else
  {
    current_base_speed = map(abs_error * 100,
                             max_error_for_high_speed * 100,                                   
                             min_error_for_low_speed * 100,                                    
                             max_straight_speed,                                               
                             cornering_speed);                                                 
    current_base_speed = constrain(current_base_speed, cornering_speed, max_straight_speed);   
  }
#else
  current_base_speed = cornering_speed;
#endif
  correction = constrain(correction, -2.5, 2.5);

  currentLeftSpeed = current_base_speed + correction * correction_scale;
  currentRightSpeed = current_base_speed - correction * correction_scale;
  currentLeftSpeed = constrain(currentLeftSpeed, 0, MAX_PWM);
  currentRightSpeed = constrain(currentRightSpeed, 0, MAX_PWM);

  setMotorSpeeds(currentLeftSpeed, currentRightSpeed);

#if DEBUG
  Serial.print(currentLeftSpeed);
  Serial.print(",");
  Serial.println(currentRightSpeed);
#endif
}


//=============================BLE FUNCTION================================
// --- BLE Characteristic Callbacks ---
class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
  }
};

class MyCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    String rxValue = pCharacteristic->getValue();
    if (rxValue.length() > 0)  handleBLECommands(rxValue);
  }
};

// --- Setup BLE ---
void setupBLE()
{
  BLEDevice::init("LineFollowerPID");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristicRX = pService->createCharacteristic(
      CHARACTERISTIC_UUID_RX,
      BLECharacteristic::PROPERTY_WRITE);
  pCharacteristicRX->setCallbacks(new MyCallbacks());
  pCharacteristicTX = pService->createCharacteristic(
      CHARACTERISTIC_UUID_TX,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristicTX->addDescriptor(new BLE2902());
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  debugLog(LOG_INFO, "Waiting a client connection to notify...");
}

// --- Handle Commands Received via BLE ---
void handleBLECommands(String cmd) // Changed parameter type to Arduino String
{
  cmd.trim();
  debugLog(LOG_INFO, "BLE RX: " + cmd);
  bool valueChanged = false;
  if (cmd.startsWith("kp="))
  {
    Kp = cmd.substring(3).toFloat();
    valueChanged = true;
  }
  else if (cmd.startsWith("ki="))
  {
    Ki = cmd.substring(3).toFloat();
    valueChanged = true;
  }
  else if (cmd.startsWith("kd="))
  {
    Kd = cmd.substring(3).toFloat();
    valueChanged = true;
  }
  else if (cmd.startsWith("ms=")) 
  {
    max_straight_speed = cmd.substring(3).toInt();
    valueChanged = true;
  }
  else if (cmd.startsWith("cs=")) 
  {
    cornering_speed = cmd.substring(3).toInt();
    valueChanged = true;
  }
  else if (cmd.startsWith("maxerr=")) 
  {
    max_error_for_high_speed = cmd.substring(7).toFloat();
    valueChanged = true;
  }
  else if (cmd.startsWith("minerr=")) 
  {
    min_error_for_low_speed = cmd.substring(7).toFloat();
    valueChanged = true;
  }
  else if (cmd.startsWith("sharpred=")) 
  {
    sharp_turn_speed_reduction = cmd.substring(9).toInt();
    valueChanged = true;
  }
  else if (cmd.startsWith("corrscale=")) 
  {
    correction_scale = cmd.substring(10).toInt();
    valueChanged = true;
  }
  else if (cmd.equalsIgnoreCase("getpid"))
    valueChanged = true;
  else if (cmd.equalsIgnoreCase("getall")) 
    valueChanged = true; 

  if (valueChanged)
  {
    // Log all current values for confirmation
    String logMsg = "Values: Kp=" + String(Kp) + " Ki=" + String(Ki) + " Kd=" + String(Kd) +
                    " MS=" + String(max_straight_speed) + " CS=" + String(cornering_speed) +
                    " MaxErr=" + String(max_error_for_high_speed) + " MinErr=" + String(min_error_for_low_speed) +
                    " SharpRed=" + String(sharp_turn_speed_reduction) + " CorrScl=" + String(correction_scale);
    debugLog(LOG_PID, logMsg);
    notifyTuningValues();
  }
}

// --- Send PID Values via BLE Notification ---
void notifyPIDValues()
{
  if (deviceConnected && pCharacteristicTX != NULL)
  {
    String pidValues = "Kp:" + String(Kp, 4) + ",Ki:" + String(Ki, 4) + ",Kd:" + String(Kd, 4);
    pCharacteristicTX->setValue(pidValues.c_str());
    pCharacteristicTX->notify();
    debugLog(LOG_PID, "Notified PID: " + pidValues);
  }
}

// --- Send ALL Tuning Values via BLE Notification ---
void notifyTuningValues()
{
  if (deviceConnected && pCharacteristicTX != NULL)
  {
    // Construct a longer string - might need multiple packets if too long for BLE MTU
    String allValues = "Kp:" + String(Kp, 4) + ",Ki:" + String(Ki, 4) + ",Kd:" + String(Kd, 4) +
                       ",MS:" + String(max_straight_speed) + ",CS:" + String(cornering_speed) +
                       ",MaxErr:" + String(max_error_for_high_speed, 2) + ",MinErr:" + String(min_error_for_low_speed, 2) +
                       ",SharpRed:" + String(sharp_turn_speed_reduction) + ",CorrScl:" + String(correction_scale);

    // Check length - BLE characteristics have limits (default ~20 bytes, can be negotiated higher)
    if (allValues.length() > 100)
    { // Example check, adjust based on expected MTU
      debugLog(LOG_ERROR, "Notification string too long!");
      // Consider sending multiple notifications or a shorter summary
      pCharacteristicTX->setValue("Error: Data too long");
    }
    else
    {
      pCharacteristicTX->setValue(allValues.c_str());
    }
    pCharacteristicTX->notify();
    debugLog(LOG_PID, "Notified All Values: " + allValues.substring(0, 100) + (allValues.length() > 100 ? "..." : "")); // Log truncated if too long
  }
}
//=========================================================================


// --- Motor Setup ---
void setupMotors()
{
  ledcAttachChannel(PWM_PIN_L_A, PWM_FREQ, PWM_RES, left_motor_channel_a);
  ledcAttachChannel(PWM_PIN_L_B, PWM_FREQ, PWM_RES, left_motor_channel_b);
  ledcAttachChannel(PWM_PIN_R_A, PWM_FREQ, PWM_RES, right_motor_channel_a);
  ledcAttachChannel(PWM_PIN_R_B, PWM_FREQ, PWM_RES, right_motor_channel_b);
}

// --- Motor Speed Control (backward support)---
void setMotorSpeeds(int left, int right) 
{
  if (right > 0)  right = right * SCALE_RIGHT_MOTOR; 
  // --- Điều khiển Motor Trái ---
  if (left > 0) {
    ledcWriteChannel(left_motor_channel_a, left);
    ledcWriteChannel(left_motor_channel_b, 0);
  } else if (left < 0) {
    ledcWriteChannel(left_motor_channel_a, 0);
    ledcWriteChannel(left_motor_channel_b, abs(left));
  } else {
    ledcWriteChannel(left_motor_channel_a, 0);
    ledcWriteChannel(left_motor_channel_b, 0);
  }

  // --- Điều khiển Motor Phải ---
  if (right > 0) {
    ledcWriteChannel(right_motor_channel_a, right);
    ledcWriteChannel(right_motor_channel_b, 0);
  } else if (right < 0) {
    ledcWriteChannel(right_motor_channel_a, 0);
    ledcWriteChannel(right_motor_channel_b, abs(right));
  } else {
    ledcWriteChannel(right_motor_channel_a, 0);
    ledcWriteChannel(right_motor_channel_b, 0);
  }
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

// --- Intersection Detection ---
bool isAllIRBlack()
{
  // Check sensors first (ensure readIRSensors was called recently)
  return (ir_norm[0] < THREADSOLD_BLACK && ir_norm[1] < THREADSOLD_BLACK && ir_norm[2] < THREADSOLD_BLACK && ir_norm[3] < THREADSOLD_BLACK);

}