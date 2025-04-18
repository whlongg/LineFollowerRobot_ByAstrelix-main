#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// --- CONFIGURABLE CONSTANTS ---
#define DEBUG 1

#define LOG_INFO 0
#define LOG_ERROR 1
#define LOG_PID 2
#define LOG_CALIB 3

#define RGB_CHECKING 0                  // 1: kiểm tra màu sắc
#define ENABLE_BYPASS_INTERSECTION 0    // Đặt 0 nếu tắt bypass
#define ENABLE_HIGH_SPEED_ON_STRAIGHT 0 // 0: Không kích hoạt HIGH_SPEED
#define THREADSOLD_BLACK 300

const float SCALE_RIGHT_MOTOR = 1.002;
const float Kp_default = 2.1; // 2.91
const float Ki_default = 0.001;
const float Kd_default = 9;
const float INTEGRAL_MAX = 70.0f;

// --- Tunable Parameters (Defaults) ---
int max_straight_speed = 240;             // Max speed on straights (leave headroom)
int cornering_speed = 212;                // Base speed for corners or large corrections
float max_error_for_high_speed = 2.5;     // Error must be BELOW this to use max_straight_speed. Lower value = stricter condition for high speed.
float min_error_for_low_speed = 3.5;      // Error must be ABOVE this to force cornering_speed. Higher value = more tolerant before slowing down.
int sharp_turn_speed_reduction = 30;      // Amount to reduce speed further during detected sharp turns.
int correction_scale = 45;                // Scales PID output to motor speed difference (Tune: 40-80)
unsigned long bypass_turn_duration = 520; // ms - Duration for the bypass turn
int threadshold_black = 300;              // IR threshold to consider 'black' for intersection detection

// --- TIMING ---
unsigned long lastLoop = 0;
const unsigned long LOOP_INTERVAL = 1; // loop timing ms
const unsigned long BLE_LOG_INTERVAL_MS = 10; //BLE Log timing ms

// --- Fixed Constants ---
const int PWM_FREQ = 20000;
const int PWM_RES = 8;
const int MAX_PWM = 255;
const float IR_WEIGHT = 0.22;
const float RGB_FILTER_ALPHA = 0.3f;

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
  STATE_BYPASSING,
  STATE_TUNING // Added state for PID auto-tuning
};

// --- GLOBALS ---
float Kp = Kp_default, Ki = Ki_default, Kd = Kd_default;
float last_error = 0, integral = 0;
uint16_t c_filtered = 0;
RobotState current_state = STATE_LINE_FOLLOWING;
unsigned long bypass_start_time = 0;

// --- PID Auto-Tuning Globals ---
bool isAutoTuning = false; // Flag to indicate tuning process active
unsigned long tuningStartTime = 0;
float Ku = 0; // Ultimate Gain
float Tu = 0; // Oscillation Period (ms)
float originalKp, originalKi, originalKd; // To restore if tuning fails/times out
const unsigned long TUNING_TIMEOUT = 2000; // Max tuning time in ms
const float KP_INCREMENT = 0.1; // How much to increase Kp each step
const unsigned long KP_INCREMENT_INTERVAL = 100; // Increase Kp every 100ms
unsigned long lastKpIncreaseTime = 0;
// Oscillation Detection Helpers
int lastErrorSign = 0;
int zeroCrossings = 0;
unsigned long lastCrossingTime = 0;
unsigned long firstCrossingTime = 0; // To calculate period
const int REQUIRED_CROSSINGS = 6; // Need 3 full cycles for stable period measurement
float peakError = 0;
float troughError = -0; // Initialize with opposite sign for first comparison
unsigned long periodSum = 0;

// --- BLE Logging Globals ---
float currentP = 0, currentI = 0, currentD = 0;
int currentLeftSpeed = 0, currentRightSpeed = 0;
unsigned long lastBleLogTime = 0; // Timer for throttling BLE logs

// const unsigned long BYPASS_TURN_DURATION = 520; // Now a tunable variable above

int ir_raw[SENSOR_COUNT] = {0, 0, 0, 0};
int ir_norm[SENSOR_COUNT] = {0, 0, 0, 0}; // 0-1000

// --- FUNCTION DECLARATIONS ---
void startBypassManeuver(); // Renamed from bypassIntersection
void checkBypassCompletion();
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
void startAutoTuning(); // Added
void processAutoTuning(); // Added
// void bypassIntersection(); // Replaced by startBypassManeuver and checkBypassCompletion

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

  if (now - lastLoop >= LOOP_INTERVAL)
  {
    lastLoop = now;

    // --- State Machine Logic ---
    switch (current_state)
    {
    case STATE_LINE_FOLLOWING:
    {
      float error = computeError();
      float correction = computePID(error);

#if ENABLE_BYPASS_INTERSECTION
      if (isAllIRBlack())
      {
        debugLog(LOG_INFO, "Intersection detected, starting bypass.");
        startBypassManeuver();
        // Don't apply motor speed based on PID this cycle, bypass maneuver takes over
      }
      else
      {
        // Only apply PID motor speed if not starting a bypass
        applyMotorSpeed(error, correction);
      }
#else
      // If bypass is disabled, always apply PID motor speed
      applyMotorSpeed(error, correction);
#endif // ENABLE_BYPASS_INTERSECTION

#if RGB_CHECKING // RGB_CHECKING
      if (detectNonWhiteLine())
      {
        debugLog(LOG_INFO, "Non-white line detected!");
        // stop(); // Example action
      }
#endif
    }
    break; // End STATE_LINE_FOLLOWING

    case STATE_BYPASSING:
      checkBypassCompletion();
      // Skip normal line following while bypassing
      break; // End STATE_BYPASSING

    case STATE_TUNING:
      processAutoTuning(); // Handle the tuning logic
      break; // End STATE_TUNING

    } // End State Machine Switch

    // --- BLE Real-time Data Logging (CSV Format) - Throttled ---
    if (deviceConnected && pCharacteristicTX != NULL && (now - lastBleLogTime >= BLE_LOG_INTERVAL_MS))
    {
      lastBleLogTime = now; // Update the last log time

      // New Format: IR_norm(ir0,ir1,ir2,ir3),Pid(kp,ki,kd),Error,Speed(left,right)\n
      String logData = "IR(" + String(ir_norm[0]) + "," + String(ir_norm[1]) + "," + String(ir_norm[2]) + "," + String(ir_norm[3]) + ")," +
                       "Pid(" + String(Kp, 2) + "," + String(Ki, 4) + "," + String(Kd, 2) + ")," +
                       "Err:" + String(last_error, 4) + "," + // Use last_error as it corresponds to the PID calculation done
                       "Spd(" + String(currentLeftSpeed) + "," + String(currentRightSpeed) + ")\n"; // Add newline for terminal apps


      // Send data via BLE Notification
      // Note: Ensure MTU is large enough or handle potential fragmentation if needed.
      // For typical terminal apps, sending line by line should be okay.
      pCharacteristicTX->setValue(logData.c_str());
      pCharacteristicTX->notify();
    }

    // Handle BLE connection status changes (can happen in any state)
    if (deviceConnected && !oldDeviceConnected)
    {
      oldDeviceConnected = deviceConnected;
      debugLog(LOG_INFO, "BLE Device Connected");
    }
    if (!deviceConnected && oldDeviceConnected)
    {
      oldDeviceConnected = deviceConnected;
      debugLog(LOG_INFO, "BLE Device Disconnected");
      // Optional: Start advertising again if needed, depending on library behavior
      // BLEDevice::startAdvertising();
    }

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

  float derivative = error - last_error;

  // Store components for logging
  currentP = Kp * error;
  currentI = Ki * integral;
  currentD = Kd * derivative;

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
  {
    current_base_speed = cornering_speed - sharp_turn_speed_reduction; // Use variable names
  }
  else if (abs_error <= max_error_for_high_speed) // Use variable name
  {
    current_base_speed = max_straight_speed; // Use variable name
  }
  else if (abs_error >= min_error_for_low_speed) // Use variable name
  {
    current_base_speed = cornering_speed; // Use variable name
  }
  else
  {
    current_base_speed = map(abs_error * 100,
                             max_error_for_high_speed * 100,                                 // Use variable name
                             min_error_for_low_speed * 100,                                  // Use variable name
                             max_straight_speed,                                             // Use variable name
                             cornering_speed);                                               // Use variable name
    current_base_speed = constrain(current_base_speed, cornering_speed, max_straight_speed); // Use variable names
  }
#else
  current_base_speed = cornering_speed; // Use variable name
#endif

  // Giới hạn correction để tránh oversteer
  correction = constrain(correction, -2.5, 2.5);

  currentLeftSpeed = current_base_speed + correction * correction_scale;  // Use variable name
  currentRightSpeed = current_base_speed - correction * correction_scale; // Use variable name
  currentLeftSpeed = constrain(currentLeftSpeed, 0, MAX_PWM);
  currentRightSpeed = constrain(currentRightSpeed, 0, MAX_PWM);

  setMotorSpeeds(currentLeftSpeed, currentRightSpeed);

#if DEBUG
  Serial.print(currentLeftSpeed);
  Serial.print(",");
  Serial.println(currentRightSpeed);
#endif
}

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
    String rxValue = pCharacteristic->getValue(); // Changed type to Arduino String
    if (rxValue.length() > 0)
    {
      handleBLECommands(rxValue); // Pass Arduino String
    }
  }
};

// --- Setup BLE ---
void setupBLE()
{
  // Create the BLE Device
  BLEDevice::init("LineFollowerPID"); // Give your BLE device a name

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic for receiving commands (RX)
  pCharacteristicRX = pService->createCharacteristic(
      CHARACTERISTIC_UUID_RX,
      BLECharacteristic::PROPERTY_WRITE);
  pCharacteristicRX->setCallbacks(new MyCallbacks());

  // Create a BLE Characteristic for sending PID values (TX)
  pCharacteristicTX = pService->createCharacteristic(
      CHARACTERISTIC_UUID_TX,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristicTX->addDescriptor(new BLE2902()); // Needed for notifications

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06); // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  debugLog(LOG_INFO, "Waiting a client connection to notify...");
}

// --- Handle Commands Received via BLE ---
void handleBLECommands(String cmd) // Changed parameter type to Arduino String
{
  // String cmd = String(value.c_str()); // No longer needed, already Arduino String
  cmd.trim(); // Remove potential whitespace/newlines
  debugLog(LOG_INFO, "BLE RX: " + cmd);

  bool valueChanged = false; // Flag to check if any value was updated
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
  else if (cmd.startsWith("ms=")) // MAX_STRAIGHT_SPEED
  {
    max_straight_speed = cmd.substring(3).toInt();
    valueChanged = true;
  }
  else if (cmd.startsWith("cs=")) // CORNERING_SPEED
  {
    cornering_speed = cmd.substring(3).toInt();
    valueChanged = true;
  }
  else if (cmd.startsWith("maxerr=")) // MAX_ERROR_FOR_HIGH_SPEED
  {
    max_error_for_high_speed = cmd.substring(7).toFloat();
    valueChanged = true;
  }
  else if (cmd.startsWith("minerr=")) // MIN_ERROR_FOR_LOW_SPEED
  {
    min_error_for_low_speed = cmd.substring(7).toFloat();
    valueChanged = true;
  }
  else if (cmd.startsWith("sharpred=")) // SHARP_TURN_SPEED_REDUCTION
  {
    sharp_turn_speed_reduction = cmd.substring(9).toInt();
    valueChanged = true;
  }
  else if (cmd.startsWith("corrscale=")) // CORRECTION_SCALE
  {
    correction_scale = cmd.substring(10).toInt();
    valueChanged = true;
  }
  else if (cmd.startsWith("bypdur=")) // BYPASS_TURN_DURATION
  {
    bypass_turn_duration = cmd.substring(7).toInt(); // Use toInt() for unsigned long
    valueChanged = true;
  }
  else if (cmd.startsWith("threshblk=")) // THREADSOLD_BLACK
  {
    threadshold_black = cmd.substring(10).toInt();
    valueChanged = true;
  }
  else if (cmd.equalsIgnoreCase("getpid")) // Keep this for compatibility or specific PID request
  {
    valueChanged = true; // Trigger notification without changing values
  }
  else if (cmd.equalsIgnoreCase("getall")) // New command to get all values
  {
    valueChanged = true; // Trigger notification without changing values
  }
  else if (cmd.equalsIgnoreCase("TUNE")) // Command to start auto-tuning
  {
    startAutoTuning();
    // valueChanged = false; // Don't notify all values immediately, tuning will do it
  }


  if (valueChanged)
  {
    // Log all current values for confirmation
    String logMsg = "Values: Kp=" + String(Kp) + " Ki=" + String(Ki) + " Kd=" + String(Kd) +
                    " MS=" + String(max_straight_speed) + " CS=" + String(cornering_speed) +
                    " MaxErr=" + String(max_error_for_high_speed) + " MinErr=" + String(min_error_for_low_speed) +
                    " SharpRed=" + String(sharp_turn_speed_reduction) + " CorrScl=" + String(correction_scale) +
                    " BypDur=" + String(bypass_turn_duration) + " ThreshBlk=" + String(threadshold_black);
    debugLog(LOG_PID, logMsg);
    notifyTuningValues(); // Send updated values back via BLE
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
                       ",SharpRed:" + String(sharp_turn_speed_reduction) + ",CorrScl:" + String(correction_scale) +
                       ",BypDur:" + String(bypass_turn_duration) + ",ThreshBlk:" + String(threadshold_black);

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

// --- Intersection Detection ---
bool isAllIRBlack()
{
  // Check sensors first (ensure readIRSensors was called recently)
  return (ir_norm[0] < threadshold_black && ir_norm[1] < threadshold_black && ir_norm[2] < threadshold_black && ir_norm[3] < threadshold_black);
  // for (int i = 0; i < SENSOR_COUNT; i++)
  // {
  //   if (ir_norm[i] > 200)
  //     return false; // 200 là ngưỡng, có thể chỉnh
  // }
  // return true;
}

// --- Start Intersection Bypass Maneuver (Non-Blocking) ---
void startBypassManeuver()
{
  current_state = STATE_BYPASSING;
  bypass_start_time = millis();
  // Dừng trái, phải chạy tốc độ cao
  setMotorSpeeds(MAX_PWM, 0);
  debugLog(LOG_INFO, "Bypass state entered. Turning right.");
  // Reset PID terms to avoid jump when resuming
  integral = 0;
  last_error = 0;
}

// --- Check if Bypass Maneuver is Complete ---
void checkBypassCompletion()
{
  if (millis() - bypass_start_time >= bypass_turn_duration)
  {
    current_state = STATE_LINE_FOLLOWING;
    // Optionally stop motors briefly or let PID take over immediately
    // setMotorSpeeds(0, 0); // Optional brief stop
    debugLog(LOG_INFO, "Bypass complete. Resuming line following.");
  }
  // Keep turning while bypassing
  // setMotorSpeeds(MAX_PWM, 0); // Ensure motors stay on during bypass check - already set in startBypassManeuver
}


// --- PID Auto-Tuning Functions ---

void startAutoTuning() {
  if (current_state == STATE_TUNING) {
    debugLog(LOG_INFO, "Tuning already in progress.");
    return;
  }

  debugLog(LOG_INFO, "Starting PID Auto-Tuning...");

  // Store original values
  originalKp = Kp;
  originalKi = Ki;
  originalKd = Kd;

  // Initialize ZN tuning parameters
  Kp = 0.5; // Start with a small Kp value
  Ki = 0;
  Kd = 0;
  integral = 0; // Reset PID state
  last_error = 0;

  // Reset tuning state variables
  tuningStartTime = millis();
  lastKpIncreaseTime = tuningStartTime;
  Ku = 0;
  Tu = 0;
  zeroCrossings = 0;
  lastCrossingTime = 0;
  firstCrossingTime = 0;
  periodSum = 0;
  lastErrorSign = 0;
  peakError = 0;
  troughError = 0; // Reset trough error as well

  current_state = STATE_TUNING;
  isAutoTuning = true; // Use this flag if needed elsewhere

  debugLog(LOG_PID, "Tuning started. Initial Kp=" + String(Kp) + ", Ki=0, Kd=0");
}

void processAutoTuning() {
  unsigned long now = millis();

  // 1. Check for Timeout
  if (now - tuningStartTime > TUNING_TIMEOUT) {
    debugLog(LOG_ERROR, "Auto-Tuning TIMEOUT!");
    // Restore original values
    Kp = originalKp;
    Ki = originalKi;
    Kd = originalKd;
    setMotorSpeeds(0, 0); // Stop robot
    current_state = STATE_LINE_FOLLOWING;
    isAutoTuning = false;
    notifyTuningValues(); // Notify restored values
    return;
  }

  // 2. Calculate Error & Apply P-Control
  float error = computeError();
  // Use computePID which now uses global Kp, Ki=0, Kd=0
  // Note: computePID also updates currentP, currentI, currentD for logging
  float correction = computePID(error);
  // Use standard applyMotorSpeed, it will use cornering_speed as base during tuning
  applyMotorSpeed(error, correction);

  // 3. Oscillation Detection Logic
  int currentSign = (error > 0) ? 1 : ((error < 0) ? -1 : 0);

  // Update peak/trough within the current half-cycle
  if (currentSign != 0) { // Ignore if error is exactly zero for peak/trough
      peakError = max(peakError, error);
      troughError = min(troughError, error);
  }


  // Detect zero crossing (sign change)
  if (lastErrorSign != 0 && currentSign != 0 && currentSign != lastErrorSign) {
    unsigned long crossingTime = now;
    zeroCrossings++;

    if (zeroCrossings == 1) {
        firstCrossingTime = crossingTime; // Record time of the very first crossing
        debugLog(LOG_PID, "Tuning: First zero crossing detected.");
    } else {
        // Calculate time since *last* crossing for period estimation
        unsigned long timeSinceLastCrossing = crossingTime - lastCrossingTime;
        // Add the duration of this half-period to the sum (we average later)
        periodSum += timeSinceLastCrossing;
        debugLog(LOG_PID, "Tuning: Crossing " + String(zeroCrossings) + ", Half-Period(ms): " + String(timeSinceLastCrossing));
    }

    lastCrossingTime = crossingTime;
    lastErrorSign = currentSign;

    // Reset peak/trough for the next half-cycle measurement *after* checking stability
    // peakError = 0; // Resetting here might miss the true peak if crossing happens right after
    // troughError = 0; // Keep peak/trough over the whole Kp increment interval for stability check? Let's reset per crossing for now.
    peakError = (currentSign > 0) ? error : 0; // Start new peak/trough from current error after crossing
    troughError = (currentSign < 0) ? error : 0;


    // 4. Check for Stable Oscillation & Calculate PID
    if (zeroCrossings >= REQUIRED_CROSSINGS) {
      // Calculate average period (Tu) based on time between first and last crossing
      // We have (zeroCrossings - 1) measured half-periods.
      // Number of full cycles = (zeroCrossings - 1) / 2
      float numFullCycles = (float)(zeroCrossings - 1) / 2.0;
      if (numFullCycles >= 1) { // Need at least one full cycle measurement
          Tu = (float)(crossingTime - firstCrossingTime) / numFullCycles; // Average period in ms

          debugLog(LOG_PID, "Tuning: Potential Oscillation Found. Crossings=" + String(zeroCrossings) +
                            ", Total Time(ms)=" + String(crossingTime - firstCrossingTime) +
                            ", Cycles=" + String(numFullCycles, 2) +
                            ", Avg Period Tu(ms)=" + String(Tu, 2));

          // Basic stability check: Period reasonable? Amplitude significant?
          // Add more checks if needed (e.g., consistency of last few periods)
          float amplitude = peakError - troughError; // Use peak/trough from the *last full cycle*? Needs more complex tracking. Using overall peak/trough for now.
          debugLog(LOG_PID, "Tuning: Amplitude=" + String(amplitude, 2) + " (Peak: " + String(peakError, 2) + ", Trough: " + String(troughError, 2) + ")");

          // Define thresholds for "stable" oscillation (adjust these based on testing)
          const float MIN_AMPLITUDE = 0.5; // Minimum peak-to-trough error difference
          const float MIN_PERIOD_MS = 50;  // Minimum realistic oscillation period
          const float MAX_PERIOD_MS = 1000; // Maximum realistic oscillation period

          if (Tu > MIN_PERIOD_MS && Tu < MAX_PERIOD_MS && amplitude > MIN_AMPLITUDE) {
            Ku = Kp; // Found the ultimate gain!
            debugLog(LOG_PID, "Auto-Tuning SUCCESS!");
            debugLog(LOG_PID, "Ku = " + String(Ku, 4));
            debugLog(LOG_PID, "Tu (ms) = " + String(Tu, 4));

            // Apply Ziegler-Nichols Classic PID formulas
            float Tu_sec = Tu / 1000.0; // Convert period to seconds for formula
            Kp = 0.6 * Ku;
            Ki = (Tu_sec > 0.001) ? (1.2 * Ku / Tu_sec) : 0; // Avoid division by zero/tiny Tu
            Kd = 0.075 * Ku * Tu_sec;

            debugLog(LOG_PID, "Calculated New PID: Kp=" + String(Kp, 4) + ", Ki=" + String(Ki, 4) + ", Kd=" + String(Kd, 4));

            setMotorSpeeds(0, 0); // Stop robot
            current_state = STATE_LINE_FOLLOWING;
            isAutoTuning = false;
            notifyTuningValues(); // Send newly calculated values
            return; // Exit tuning process
          } else {
             debugLog(LOG_PID, "Tuning: Oscillation detected but not stable/significant enough yet.");
             // Reset crossing count to wait for more stable cycles at this Kp? Or just continue increasing Kp?
             // Let's continue increasing Kp for now. If Kp gets too high, timeout will trigger.
             // Resetting counters here might prevent finding Ku if oscillation starts right at the end of an interval.
          }
      }
    } // End stability check
  } // End zero crossing detected


  // 5. Increase Kp if needed and oscillation not yet stable
  if (now - lastKpIncreaseTime > KP_INCREMENT_INTERVAL && Ku == 0) { // Only increase if Ku not found yet
    Kp += KP_INCREMENT;
    lastKpIncreaseTime = now;
    // Reset detection helpers as dynamics change with new Kp
    zeroCrossings = 0;
    lastCrossingTime = 0;
    firstCrossingTime = 0;
    periodSum = 0;
    lastErrorSign = 0; // Reset sign to allow immediate crossing detection
    peakError = 0;
    troughError = 0;
    integral = 0; // Keep integral zero during tuning
    debugLog(LOG_PID, "Tuning: Increasing Kp to " + String(Kp, 4));
  }
}
