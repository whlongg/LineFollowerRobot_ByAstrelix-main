#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

/*****************************************************************************************
 * LINE FOLLOWER ROBOT – OPTIMISED FOR HIGH‑SPEED & ROBUSTNESS (ESP32‑C3)
 *----------------------------------------------------------------------------------------
 *  - PID line following with EMA‑filtered IR sensors
 *  - Dynamic speed control & sharp‑turn protection
 *  - Intersection bypass using sensor‑based re‑acquire logic
 *  - Ziegler‑Nichols auto‑tuning option
 *  - BLE telemetry & run‑time parameter update
 *  - Calibration routine preserved (press BUTTON)
 *  - Uses ledcWriteChannel() for 200 RPM gear motors (≈20 kHz PWM)
 *****************************************************************************************/

//--------------------------------------------------
//                        CONFIG
//--------------------------------------------------

// -------- Debug & Logging --------
#define DEBUG 1                // 0 = no serial, 1 = enable
#define LOG_INFO       0
#define LOG_ERROR      1
#define LOG_PID        2
#define LOG_CALIB      3
#define BLE_LOG_PERIOD_MS 10    // Telemetry rate

// -------- Feature Toggles --------
#define RGB_CHECKING                0   // Keep for future colour tasks
#define ENABLE_BYPASS_INTERSECTION  1   // 0 = disable bypass logic
#define ENABLE_HIGH_SPEED_ON_STRAIGHT 1 // 0 = fixed speed always

// -------- Constants / Pins --------
const uint8_t IR_PINS[]          = {4, 3, 1, 0};      // Left‑>Right order (outer‑left .. outer‑right)
const uint8_t SENSOR_COUNT       = sizeof(IR_PINS);
const uint8_t PWM_PIN_L_A        = 2;
const uint8_t PWM_PIN_L_B        = 10;
const uint8_t PWM_PIN_R_A        = 6;
const uint8_t PWM_PIN_R_B        = 5;
const uint8_t left_motor_channel_a = 0;
const uint8_t left_motor_channel_b = 1;
const uint8_t right_motor_channel_a = 2;
const uint8_t right_motor_channel_b = 3;
const uint8_t W_LED_PIN          = 20;  // White LED (optional)
const uint8_t IR_LED_PIN         = 21;  // IR LED power (optional)
const uint8_t LED_CHECK_PIN      = 8;   // Small status LED
const uint8_t BUTTON_PIN         = 7;   // Push‑button for calibration

// -------- PWM --------
const uint16_t PWM_FREQ          = 20000; // 20 kHz – silent for gear‑motors
const uint16_t PWM_RES           = 8;
const uint16_t MAX_PWM           = 255;
const float SCALE_RIGHT_MOTOR    = 1.002;

// -------- IR Normalisation --------
const uint16_t IR_CALIB_SAMPLES  = 32;
const uint16_t IR_NORM_MAX       = 1000; // After mapping min‑max
const float     IR_FILTER_ALPHA  = 0.4f; // EMA filter constant (0<α≤1)
const uint16_t THRESHOLD_BLACK   = 300;  // Filtered value ≤ → sensor sees black

// -------- PID Default Gains --------
const float KP_DEFAULT           = 1.8f;
const float KI_DEFAULT           = 0.001f;
const float KD_DEFAULT           = 11.0f;
const float INTEGRAL_MAX         = 70.0f;

// -------- Speed Parameters --------
int   maxStraightSpeed           = 240; // top speed on clear straight
int   corneringSpeed             = 212; // base speed on curves
float maxErrForHighSpeed         = 2.5f;
float minErrForLowSpeed          = 3.5f;
int   sharpTurnSpeedReduction    = 30;  // additional reduction for very sharp turn
int   correctionScale            = 50;  // scale PID output before mixing
int   minBaseSpeed               = 160; // never go below this

// -------- Bypass / Intersection --------
const uint16_t FIXED_TURN_DURATION_MS  = 520;   // Duration to pivot 90° (tune)
const uint16_t SEARCH_TIMEOUT_MS       = 1000;  // Max time to re‑acquire line

// -------- Timing --------
const uint32_t LOOP_INTERVAL_US        = 1000;  // 1 ms loop target

//--------------------------------------------------
//                     GLOBALS
//--------------------------------------------------

// IR sampling buffers
uint16_t irRaw[SENSOR_COUNT]     = {0};
uint16_t irMin[SENSOR_COUNT]     = {65535};
uint16_t irMax[SENSOR_COUNT]     = {0};
float    irNorm[SENSOR_COUNT]    = {0};    // 0‑1000
float    irFilt[SENSOR_COUNT]    = {0};    // EMA filtered 0‑1000

// Error / PID
float lastError   = 0;
float integral    = 0;

// Motor speed cache (for telemetry)
int   leftSpeedCmd  = 0;
int   rightSpeedCmd = 0;

//--------------------------------------------------
//               STATE MACHINE
//--------------------------------------------------

enum class RobotState : uint8_t {
  LINE_FOLLOWING,
  BYPASS_TURN,
  BYPASS_SEARCH,
  AUTO_TUNE
};
RobotState state = RobotState::LINE_FOLLOWING;

// --- Intersection bypass helpers
uint32_t bypassStartMs   = 0;

// --- Auto Tuning (Z‑N) helpers
bool   autoTuning             = false;
float  Ku                     = 0;    // ultimate gain
float  Tu                     = 0;    // oscillation period (ms)
uint32_t tuningStartMs        = 0;
uint32_t lastKpStepMs         = 0;
const uint32_t TUNING_TIMEOUT = 3000; // 3 s overall‑timeout
const float    KP_STEP        = 0.1f;
const uint32_t KP_STEP_PERIOD = 100;  // every 100 ms
float  Kp = KP_DEFAULT, Ki = KI_DEFAULT, Kd = KD_DEFAULT;

//--------------------------------------------------
//               RGB SENSOR (not critical)
//--------------------------------------------------
Adafruit_TCS34725 rgbSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS,
                                                TCS34725_GAIN_4X);
uint16_t rgbMin = 65535, rgbMax = 0; // for possible use

//--------------------------------------------------
//               BLE DEFINITIONS
//--------------------------------------------------
#define SERVICE_UUID           "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_UUID_TX           "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHAR_UUID_RX           "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

BLEServer*        pServer   = nullptr;
BLECharacteristic* pCharTx  = nullptr;
BLECharacteristic* pCharRx  = nullptr;
BLE2902* tx2902 = nullptr;
bool deviceConnected = false;

//--------------------------------------------------
//               BLE CALLBACKS
//--------------------------------------------------
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    deviceConnected = true;
    Serial.println(F("[BLE] Device Connected"));
  }
  void onDisconnect(BLEServer* pServer) override {
    deviceConnected = false;
    Serial.println(F("[BLE] Device Disconnected"));
    pServer->getAdvertising()->start();
  }
};

class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    String rx = pChar->getValue().c_str();
    Serial.print(F("[BLE] RX: "));
    Serial.println(rx);
  }
};

void setupBLE() {
  BLEDevice::init("LineFollower");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService* pService = pServer->createService(SERVICE_UUID);

  pCharTx = pService->createCharacteristic(CHAR_UUID_TX,
                    BLECharacteristic::PROPERTY_NOTIFY);
  tx2902 = new BLE2902();
  pCharTx->addDescriptor(tx2902);

  pCharRx = pService->createCharacteristic(CHAR_UUID_RX,
                    BLECharacteristic::PROPERTY_WRITE);
  pCharRx->setCallbacks(new RxCallbacks());

  pService->start();
  pServer->getAdvertising()->start();
  Serial.println(F("[BLE] Ready and advertising"));
}

//--------------------------------------------------
//                 DEBUG / LOGGING
//--------------------------------------------------
void debugLog(uint8_t level, const String& msg) {
  #if DEBUG
    Serial.print("["); Serial.print(level); Serial.print("] "); Serial.println(msg);
  #endif
  // Only send BLE notify if client subscribed
  if (deviceConnected && tx2902 && tx2902->getNotifications()) {
    pCharTx->setValue(msg.c_str());
    pCharTx->notify();
    #if DEBUG
      Serial.println(F("[BLE] Sent notify (debugLog)"));
    #endif
  }
}

//--------------------------------------------------
//               UTILITY MACROS
//--------------------------------------------------
#if DEBUG
  #define DPRINT(level, x)  debugLog(level, x)
#else
  #define DPRINT(level, x)
#endif

//--------------------------------------------------
//               FUNCTION PROTOTYPES
//--------------------------------------------------
void  setupMotors();
void  setMotorSpeeds(int left, int right);
void  readIRSensors();
float computeError();
float computePID(float error);
void  applyMotorSpeed(float error, float correction);
void  startBypassTurn();
void  handleBypass();
void  startAutoTuning();
void  processAutoTuning();
void  handleBLECommands(const String& cmd);
void  debugLog(uint8_t level, const String& msg);
void  calibrateSensors();

//--------------------------------------------------
//                       SETUP
//--------------------------------------------------
void setup() {
  #if DEBUG
    Serial.begin(115200);
    while (!Serial);
  #endif

  pinMode(LED_CHECK_PIN, OUTPUT);
  pinMode(BUTTON_PIN,    INPUT_PULLUP);
  pinMode(IR_LED_PIN,    OUTPUT);
  digitalWrite(IR_LED_PIN, HIGH);     // turn IR LEDs ON
  calibrateSensors();
  
  setupMotors();
  setupBLE();

  // Optional RGB sensor init (kept for compatibility)
  if (rgbSensor.begin()) {
    DPRINT(LOG_INFO, F("RGB sensor initialised"));
  } else {
    DPRINT(LOG_ERROR, F("RGB sensor NOT found"));
  }

  

  lastError = 0;
  integral  = 0;
}

//--------------------------------------------------
//                        LOOP
//--------------------------------------------------
void loop() {
  static uint32_t lastLoopUs = micros();
  uint32_t nowUs = micros();
  if (nowUs - lastLoopUs < LOOP_INTERVAL_US) return; // simple time‑slice
  lastLoopUs = nowUs;

  // --- BLE handling (non‑blocking) ---
  if (deviceConnected && pCharRx && pCharRx->getLength() > 0) {
    String cmd = pCharRx->getValue().c_str();
    pCharRx->setValue(""); // clear buffer
    Serial.print(F("[BLE] Handle command: "));
    Serial.println(cmd);
    handleBLECommands(cmd);
  }

  //--------------------------------------------------
  //                STATE MACHINE
  //--------------------------------------------------
  switch (state) {
    case RobotState::LINE_FOLLOWING: {
        readIRSensors();
        float error      = computeError();
        float correction = computePID(error);
        applyMotorSpeed(error, correction);
        break;
    }
    case RobotState::BYPASS_TURN:
    case RobotState::BYPASS_SEARCH:
        handleBypass();
        break;
    case RobotState::AUTO_TUNE:
        processAutoTuning();
        break;
  }

  //--------------------------------------------------
  //          Periodic telemetry via BLE
  //--------------------------------------------------
  static uint32_t lastBleMs = 0;
  uint32_t nowMs = millis();
  if (deviceConnected && tx2902 && tx2902->getNotifications() && pCharTx && (nowMs - lastBleMs) >= BLE_LOG_PERIOD_MS) {
    lastBleMs = nowMs;
    // Log Format: ms,ir0,ir1,ir2,ir3,Kp,Ki,Kd,err,left,right\n
    String logData = String(nowMs) + "," +
                     String(irNorm[0], 1) + "," +
                     String(irNorm[1], 1) + "," +
                     String(irNorm[2], 1) + "," +
                     String(irNorm[3], 1) + "," +
                     String(Kp, 2) + "," +
                     String(Ki, 4) + "," +
                     String(Kd, 2) + "," +
                     String(lastError, 4) + "," +
                     String(leftSpeedCmd) + "," +
                     String(rightSpeedCmd) + "\n";
    pCharTx->setValue(logData.c_str());
    pCharTx->notify();
    #if DEBUG
      Serial.println(F("[BLE] Sent notify (logData)"));
    #endif
  }
}

//--------------------------------------------------
//            SENSOR READING & FILTERING
//--------------------------------------------------
void readIRSensors() {
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    uint16_t raw = analogRead(IR_PINS[i]);
    irRaw[i] = raw;
    // ---- map to 0‑1000 using calibration ----
    uint16_t clamped = constrain(raw, irMin[i], irMax[i]);
    float norm = ((float)(clamped - irMin[i]) / (irMax[i] - irMin[i])) * IR_NORM_MAX;
    irNorm[i] = norm;
    // ---- EMA filter ----
    irFilt[i] = IR_FILTER_ALPHA * norm + (1 - IR_FILTER_ALPHA) * irFilt[i];
  }
}

//--------------------------------------------------
//                ERROR CALCULATION
//--------------------------------------------------
float computeError() {
  // Weights: left most = ‑3, inner‑left = ‑1, inner‑right = +1, rightmost = +3
  const int8_t weights[] = {-3, -1, 1, 3};
  float weightedSum = 0;
  float total = 0;
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    float val = irFilt[i];             // 0 (black) … 1000 (white)
    float darkness = IR_NORM_MAX - val;// convert to darkness (line)
    weightedSum += darkness * weights[i];
    total       += darkness;
  }
  if (total < 10) {
    // Line lost – use last error direction to drive search
    return (lastError >= 0) ? 4.0f : -4.0f; // saturate to edge weight
  }
  return weightedSum / total;
}

//--------------------------------------------------
//                PID CONTROLLER
//--------------------------------------------------
float computePID(float error) {
  // simple discrete PID assuming constant dt ≈ LOOP_INTERVAL_US
  const float dt = LOOP_INTERVAL_US / 1000000.0f; // seconds

  float P = error;
  integral += error * dt;
  integral = constrain(integral, -INTEGRAL_MAX, INTEGRAL_MAX);
  float D = (error - lastError) / dt;

  float output = Kp * P + Ki * integral + Kd * D;
  lastError = error;
  return output;
}

//--------------------------------------------------
//             MOTOR SPEED APPLICATION
//--------------------------------------------------
void applyMotorSpeed(float error, float correction) {
  // Determine base speed
  int base = corneringSpeed;
  if (ENABLE_HIGH_SPEED_ON_STRAIGHT && fabs(error) < maxErrForHighSpeed) {
      base = maxStraightSpeed;
  } else if (fabs(error) > minErrForLowSpeed) {
      base = corneringSpeed;
  }
  // Additional reduction for extremely high error (sharp turn)
  if (fabs(error) > 2.5f) {
      base = max(base - sharpTurnSpeedReduction, minBaseSpeed);
  }

  // Scale correction and saturate so we never reverse wheel direction
  int delta = (int)(correction / correctionScale);
  delta = constrain(delta, -base, base);

  int left  = base + delta;
  int right = base - delta;

  left  = constrain(left, 0, MAX_PWM);
  right = constrain(right, 0, MAX_PWM);

  setMotorSpeeds(left, right);
  leftSpeedCmd  = left;
  rightSpeedCmd = right;
}

//--------------------------------------------------
//                MOTOR HELPERS
//--------------------------------------------------
void setupMotors() {
  ledcAttachChannel(PWM_PIN_L_A, PWM_FREQ, PWM_RES, left_motor_channel_a);
  ledcAttachChannel(PWM_PIN_L_B, PWM_FREQ, PWM_RES, left_motor_channel_b);
  ledcAttachChannel(PWM_PIN_R_A, PWM_FREQ, PWM_RES, right_motor_channel_a);
  ledcAttachChannel(PWM_PIN_R_B, PWM_FREQ, PWM_RES, right_motor_channel_b);
  setMotorSpeeds(0,0);
}

void setMotorSpeeds(int left, int right)
{
  right = right * SCALE_RIGHT_MOTOR; // Apply scaling factor to right motor
  ledcWriteChannel(left_motor_channel_a, left);
  ledcWriteChannel(left_motor_channel_b, 0);
  ledcWriteChannel(right_motor_channel_a, right);
  ledcWriteChannel(right_motor_channel_b, 0);
}

//--------------------------------------------------
//             INTERSECTION BYPASS LOGIC
//--------------------------------------------------
void startBypassTurn() {
  bypassStartMs = millis();
  state = RobotState::BYPASS_TURN;
  // Reset PID dynamics
  integral  = 0;
  lastError = 0;
  // Pivot in‑place (example: left turn)
  setMotorSpeeds(corneringSpeed, 0);
}

void handleBypass() {
  uint32_t now = millis();

  if (state == RobotState::BYPASS_TURN) {
    if (now - bypassStartMs >= FIXED_TURN_DURATION_MS) {
      // Move to search state
      state = RobotState::BYPASS_SEARCH;
      bypassStartMs = now; // reuse timer
      setMotorSpeeds(minBaseSpeed, minBaseSpeed); // forward slow
    }
    return;
  }

  // --- BYPASS_SEARCH ---
  readIRSensors();
  bool lineSeen = (irFilt[1] < THRESHOLD_BLACK) || (irFilt[2] < THRESHOLD_BLACK);
  if (lineSeen) {
    state = RobotState::LINE_FOLLOWING;
    return;
  }
  if (now - bypassStartMs > SEARCH_TIMEOUT_MS) {
    DPRINT(LOG_ERROR, F("Bypass search TIMEOUT – returning to LINE_FOLLOWING"));
    state = RobotState::LINE_FOLLOWING;
  }
}

//--------------------------------------------------
//                AUTO TUNING (Z‑N)
//--------------------------------------------------
void startAutoTuning() {
  if (autoTuning) return;
  autoTuning      = true;
  state           = RobotState::AUTO_TUNE;
  tuningStartMs   = millis();
  lastKpStepMs    = tuningStartMs;
  Kp              = 0.5f; // start small
  Ki = 0; Kd = 0;         // PI off during tuning
  Ku = Tu = 0;
  integral = lastError = 0;
  DPRINT(LOG_PID, F("Auto‑tuning START"));
}

void processAutoTuning() {
  // Simple P‑only oscillation → detect zero cross
  readIRSensors();
  float error = computeError();
  float output = Kp * error; // P only
  int   delta  = (int)(output / correctionScale);
  int   base   = corneringSpeed;
  int left = constrain(base + delta, 0, MAX_PWM);
  int right= constrain(base - delta, 0, MAX_PWM);
  setMotorSpeeds(left, right);

  // Zero‑cross detection
  static int8_t lastSign = 0;
  int8_t sign = (error > 0) - (error < 0);
  static uint8_t zeroCrossCnt = 0;
  static uint32_t firstCrossMs = 0;
  if (sign != 0 && sign != lastSign) {
    if (zeroCrossCnt == 0) firstCrossMs = millis();
    zeroCrossCnt++;
    lastSign = sign;
  }
  // Enough crossings?
  if (zeroCrossCnt >= 6) { // 3 full cycles
    uint32_t periodMs = (millis() - firstCrossMs) / 3;
    Ku = Kp;
    Tu = periodMs;
    // Compute PID via Z‑N classic
    Kp = 0.6f * Ku;
    Ki = 1.2f * Ku / (Tu / 1000.0f);
    Kd = 0.075f * Ku * (Tu / 1000.0f);

    DPRINT(LOG_PID, String("Tuning DONE – Kp=") + Kp + ", Ki=" + Ki + ", Kd=" + Kd);
    autoTuning = false;
    state = RobotState::LINE_FOLLOWING;
    return;
  }

  // Increase Kp periodically until oscillation observed or timeout
  if (millis() - lastKpStepMs >= KP_STEP_PERIOD) {
    Kp += KP_STEP;
    lastKpStepMs = millis();
  }
  if (millis() - tuningStartMs > TUNING_TIMEOUT) {
    DPRINT(LOG_ERROR, F("Tuning TIMEOUT – restore defaults"));
    Kp = KP_DEFAULT; Ki = KI_DEFAULT; Kd = KD_DEFAULT;
    autoTuning = false;
    state = RobotState::LINE_FOLLOWING;
  }
}

//--------------------------------------------------
//                      BLE
//--------------------------------------------------

void handleBLECommands(const String& cmd) {
  if (cmd == "STOP") {
    setMotorSpeeds(0,0);
    debugLog(LOG_INFO, F("Received STOP"));
    return;
  }
  if (cmd == "TUNE") {
    startAutoTuning();
    debugLog(LOG_INFO, F("Received TUNE"));
    return;
  }
  // Example: "KP=2.3" etc.
  if (cmd.startsWith("KP=")) {
    Kp = cmd.substring(3).toFloat();
    debugLog(LOG_INFO, String("KP updated to ") + Kp);
  } else if (cmd.startsWith("KI=")) {
    Ki = cmd.substring(3).toFloat();
    debugLog(LOG_INFO, String("KI updated to ") + Ki);
  } else if (cmd.startsWith("KD=")) {
    Kd = cmd.substring(3).toFloat();
    debugLog(LOG_INFO, String("KD updated to ") + Kd);
  } else {
    debugLog(LOG_INFO, String("Unknown BLE command: ") + cmd);
  }
}

//--------------------------------------------------
//                 DEBUG / LOGGING
//--------------------------------------------------
void debugLog(uint8_t level, const String& msg) {
  #if DEBUG
    Serial.print("["); Serial.print(level); Serial.print("] "); Serial.println(msg);
  #endif
  // Only send BLE notify if client subscribed
  if (deviceConnected && tx2902 && tx2902->getNotifications()) {
    pCharTx->setValue(msg.c_str());
    pCharTx->notify();
    #if DEBUG
      Serial.println(F("[BLE] Sent notify (debugLog)"));
    #endif
  }
}

//--------------------------------------------------
//                   CALIBRATION
//--------------------------------------------------
void calibrateSensors() {
  #if DEBUG
    Serial.println(F("*** Hold BUTTON to calibrate IR sensors…"));
  #endif
  DPRINT(LOG_CALIB, F("*** Hold BUTTON to calibrate IR sensors…"));
  // Wait for user to press button
  while (digitalRead(BUTTON_PIN) == HIGH) {
    delay(10);
  }
  #if DEBUG
    Serial.println(F("Calibrating... keep holding the button!"));
  #endif
  DPRINT(LOG_CALIB, F("Calibrating... keep holding the button!"));
  // Calibrate while button held
  while (digitalRead(BUTTON_PIN) == LOW) {
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
      uint16_t v = analogRead(IR_PINS[i]);
      irMin[i] = min(irMin[i], v);
      irMax[i] = max(irMax[i], v);
    }
    delay(2);
  }
  // Wait for user to release button
  while (digitalRead(BUTTON_PIN) == LOW) {
    delay(10);
  }
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    if (irMax[i] - irMin[i] < 50) {
      irMax[i] = irMin[i] + 50; // avoid divide by zero later
    }
  }
  #if DEBUG
    Serial.println(F("Calibration done!"));
  #endif
  DPRINT(LOG_CALIB, F("Calibration done!"));
}
