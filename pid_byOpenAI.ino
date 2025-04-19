#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ============================================================================
// CONFIGURATION
// ============================================================================

#define DEBUG_SERIAL 1      // 1 => enable Serial debug, 0 => disable
#define BLE_LOG_THROTTLE 10 // ms between log packets

// -------------------- CONSTANTS (tunable) ----------------------------------
static const uint8_t  SENSOR_COUNT                 = 4;
static const uint16_t SENSOR_NORM_MAX              = 1000;
static const uint16_t THRESHOLD_BLACK              = 200;   // IR value considered "black"
static const float    IR_FILTER_ALPHA              = 0.3f;  // EMA coefficient [0..1]

static const uint8_t  MAX_PWM                      = 255;
static const uint16_t LOOP_INTERVAL_MS             = 1;     // main loop period
static const uint16_t FIXED_TURN_DURATION_MS       = 400;   // time for 90° turn during bypass
static const uint16_t SEARCH_TIMEOUT_MS            = 800;   // max time spent searching after turn

// Speed profile
static const uint8_t  MAX_STRAIGHT_SPEED           = 240;
static const uint8_t  CORNERING_SPEED              = 212;
static const uint8_t  SHARP_TURN_SPEED_REDUCTION   = 30;
static const float    MAX_ERROR_FOR_HIGH_SPEED     = 1.5f;
static const float    MIN_ERROR_FOR_LOW_SPEED      = 2.0f;
static const uint8_t  CORRECTION_SCALE             = 50;    // scale PID output ➜ speed diff
static const uint8_t  MIN_BASE_SPEED               = 150;   // never drive slower than this

// PID parameters (defaults – can be updated via BLE)
static float Kp_default = 3.6f;
static float Ki_default = 0.001f;
static float Kd_default = 39.0f;
static const float INTEGRAL_MAX = 70.0f; // anti‑windup clamp

// ============================================================================
// ENUMS & STRUCTS
// ============================================================================

enum class RobotState : uint8_t {
  LineFollowing,
  BypassTurn,
  BypassSearch,
  Tuning
};

// ============================================================================
// PINOUT (ESP32‑DEVKIT V1 ‑ adapt as required)
// ============================================================================

static const uint8_t IR_PINS[SENSOR_COUNT] = {4, 3, 1, 0};

static const uint8_t PWM_PIN_L_A = 2;
static const uint8_t PWM_PIN_L_B = 10;
static const uint8_t PWM_PIN_R_A = 6;
static const uint8_t PWM_PIN_R_B = 5;

static const uint8_t LED_CHECK   = 8;
static const uint8_t BUTTON_PIN  = 7; // not used yet

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

static Adafruit_TCS34725 rgbSensor(TCS34725_INTEGRATIONTIME_50MS,
                                   TCS34725_GAIN_4X);

static uint16_t ir_raw[SENSOR_COUNT]     = {0};
static uint16_t ir_norm[SENSOR_COUNT]    = {0};
static float    ir_filtered[SENSOR_COUNT]= {0};

static uint16_t ir_min[SENSOR_COUNT] = {65535, 65535, 65535, 65535};
static uint16_t ir_max[SENSOR_COUNT] = {0};

static RobotState currentState = RobotState::LineFollowing;
static unsigned long stateStartMs      = 0;  // millis when state entered

// PID globals
static float Kp = Kp_default;
static float Ki = Ki_default;
static float Kd = Kd_default;

static float lastError  = 0;
static float integral   = 0;

// Logging helpers
static unsigned long lastBleLogMs = 0;

// BLE globals
static BLECharacteristic *pChrRX = nullptr;
static BLECharacteristic *pChrTX = nullptr;
static bool deviceConnected = false;

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

#if DEBUG_SERIAL
#define DBG(...) Serial.print(__VA_ARGS__)
#define DBGLN(...) Serial.println(__VA_ARGS__)
#else
#define DBG(...)
#define DBGLN(...)
#endif

static inline uint32_t nowMs() { return millis(); }

// ============================================================================
// HARDWARE SETUP
// ============================================================================

void setupMotors(){
  pinMode(PWM_PIN_L_A, OUTPUT);
  pinMode(PWM_PIN_L_B, OUTPUT);
  pinMode(PWM_PIN_R_A, OUTPUT);
  pinMode(PWM_PIN_R_B, OUTPUT);
}

void setMotorPwm(int left, int right){
  left  = constrain(left, -MAX_PWM, MAX_PWM);
  right = constrain(right, -MAX_PWM, MAX_PWM);

  // Left motor
  if(left >= 0){
    analogWrite(PWM_PIN_L_A, left);
    analogWrite(PWM_PIN_L_B, 0);
  }else{
    analogWrite(PWM_PIN_L_A, 0);
    analogWrite(PWM_PIN_L_B, -left);
  }
  // Right motor
  if(right >= 0){
    analogWrite(PWM_PIN_R_A, right);
    analogWrite(PWM_PIN_R_B, 0);
  }else{
    analogWrite(PWM_PIN_R_A, 0);
    analogWrite(PWM_PIN_R_B, -right);
  }
}

// ============================================================================
// SENSOR HANDLING
// ============================================================================

void readIrSensors(){
  for(uint8_t i=0;i<SENSOR_COUNT;++i){
    uint16_t v = analogRead(IR_PINS[i]);
    ir_raw[i] = v;
    // Track min/max for auto‑normalization
    if(v < ir_min[i]) ir_min[i] = v;
    if(v > ir_max[i]) ir_max[i] = v;

    // Normalise to 0..SENSOR_NORM_MAX (white -> high, black -> low)
    uint16_t range = ir_max[i] - ir_min[i];
    if(range < 10) range = 10; // avoid div0 early on
    uint16_t norm = (uint32_t)(v - ir_min[i]) * SENSOR_NORM_MAX / range;
    ir_norm[i] = constrain(norm, (uint16_t)0, SENSOR_NORM_MAX);

    // EMA FILTER
    ir_filtered[i] = IR_FILTER_ALPHA * ir_norm[i] + (1.0f - IR_FILTER_ALPHA) * ir_filtered[i];
  }
}

bool lineReacquired(){
  // Use inner sensors (index 1,2) < threshold to declare line found
  return (ir_filtered[1] < THRESHOLD_BLACK) || (ir_filtered[2] < THRESHOLD_BLACK);
}

bool detectIntersection(){
  // All sensors black => intersection / dead end
  for(uint8_t i=0;i<SENSOR_COUNT;++i)
    if(ir_filtered[i] > THRESHOLD_BLACK) return false;
  return true;
}

// ============================================================================
// ERROR & PID
// ============================================================================

float computeError(){
  static const int8_t WEIGHTS[SENSOR_COUNT] = { 3, 1, -1, -3 }; // L ➜ R
  int32_t sum=0, total=0;
  for(uint8_t i=0;i<SENSOR_COUNT;++i){
    sum   += (int32_t)ir_filtered[i] * WEIGHTS[i];
    total += ir_filtered[i];
  }
  if(total == 0) return 0; // avoid div0; shouldn't happen due to normalization
  return (float)sum / (float)total; // ~‑3 .. +3
}

float computePid(float error){
  if(fabs(error) < 1.0f){
    integral += error;
    integral = constrain(integral, -INTEGRAL_MAX, INTEGRAL_MAX);
  }else if((error * lastError < 0) || fabs(error) > 2.5f){
    integral = 0; // reset on line swap / lost line
  }
  float derivative = error - lastError;
  lastError = error;
  return Kp*error + Ki*integral + Kd*derivative;
}

// ============================================================================
// SPEED CONTROL
// ============================================================================

void applyMotorSpeed(float error, float pidOut){
  // Base speed selection
  static uint8_t baseSpeed = CORNERING_SPEED;
  bool sharpTurn = (ir_filtered[0] < THRESHOLD_BLACK && ir_filtered[3] < THRESHOLD_BLACK);

  if(sharpTurn){
    baseSpeed = constrain(baseSpeed - SHARP_TURN_SPEED_REDUCTION, MIN_BASE_SPEED, MAX_STRAIGHT_SPEED);
  } else if(fabs(error) < MAX_ERROR_FOR_HIGH_SPEED){
    baseSpeed = MAX_STRAIGHT_SPEED;
  } else if(fabs(error) > MIN_ERROR_FOR_LOW_SPEED){
    baseSpeed = CORNERING_SPEED;
  }

  // PID correction → speed diff
  int16_t correction = (int16_t)constrain(pidOut * CORRECTION_SCALE, -baseSpeed, baseSpeed);
  int16_t left  = baseSpeed + correction;
  int16_t right = baseSpeed - correction;

  setMotorPwm(left, right);
}

// ============================================================================
// BLE (minimal – RX to update PID, TX for logs)
// ============================================================================

class ChrCallbacks : public BLECharacteristicCallbacks{
  void onWrite(BLECharacteristic *chr) override {
    String v = chr->getValue();
    if(v.length() == 0) return;
    if(v[0]=='P'){ // "P=<value>" etc.
      Kp = atof(v.c_str()+2);
    }else if(v[0]=='I'){
      Ki = atof(v.c_str()+2);
    }else if(v[0]=='D'){
      Kd = atof(v.c_str()+2);
    }else if(v == "TUNE"){
      currentState = RobotState::Tuning;
      stateStartMs = nowMs();
    }
  }
};

void setupBle(){
  BLEDevice::init("LineFollower");
  BLEServer *server = BLEDevice::createServer();
  server->getAdvertising()->addServiceUUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
  server->startAdvertising();

  BLEService *svc = server->createService("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
  pChrRX = svc->createCharacteristic("beb5483e-36e1-4688-b7f5-ea07361b26a8",
                                     BLECharacteristic::PROPERTY_WRITE);
  pChrTX = svc->createCharacteristic("beb5483e-36e1-4688-b7f5-ea07361b26a9",
                                     BLECharacteristic::PROPERTY_NOTIFY);
  pChrTX->addDescriptor(new BLE2902());
  pChrRX->setCallbacks(new ChrCallbacks());
  svc->start();
}

void bleLogSensors(float error, float pid){
  if(!deviceConnected) return;
  if(nowMs() - lastBleLogMs < BLE_LOG_THROTTLE) return;
  lastBleLogMs = nowMs();
  char buf[64];
  snprintf(buf, sizeof(buf), "%.0f,%.0f,%.0f,%.0f,%.2f,%.2f", ir_filtered[0], ir_filtered[1], ir_filtered[2], ir_filtered[3], error, pid);
  pChrTX->setValue((uint8_t*)buf, strlen(buf));
  pChrTX->notify();
}

// ============================================================================
// BYPASS (intersection) LOGIC
// ============================================================================

void startBypass(){
  currentState = RobotState::BypassTurn;
  stateStartMs = nowMs();
  integral = 0; lastError = 0; // reset PID memories
}

// ============================================================================
// SETUP & LOOP
// ============================================================================

void setup(){
  Serial.begin(115200);
  setupMotors();
  setupBle();

  pinMode(LED_CHECK, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // initialise IR filter to a sane value
  for(uint8_t i=0;i<SENSOR_COUNT;++i) ir_filtered[i] = 500; // mid‑range
}

void loop(){
  static unsigned long lastLoopMs = 0;
  if(nowMs() - lastLoopMs < LOOP_INTERVAL_MS) return; // small fixed period
  lastLoopMs = nowMs();

  readIrSensors();

  switch(currentState){
    case RobotState::LineFollowing: {
      if(detectIntersection()){
        startBypass();
        break;
      }
      float error = computeError();
      float pid   = computePid(error);
      applyMotorSpeed(error, pid);
      bleLogSensors(error, pid);
      }
      break;

    case RobotState::BypassTurn: {
      // in‑place turn (left) – you may adjust direction
      setMotorPwm(-CORNERING_SPEED, CORNERING_SPEED);
      if(nowMs() - stateStartMs >= FIXED_TURN_DURATION_MS){
        currentState = RobotState::BypassSearch;
        stateStartMs = nowMs();
      }
      }
      break;

    case RobotState::BypassSearch: {
      setMotorPwm(MIN_BASE_SPEED, MIN_BASE_SPEED); // crawl forward
      if(lineReacquired()){
        currentState = RobotState::LineFollowing;
        stateStartMs = nowMs();
      }else if(nowMs() - stateStartMs >= SEARCH_TIMEOUT_MS){
        // timeout – give up and resume
        currentState = RobotState::LineFollowing;
        stateStartMs = nowMs();
      }
      }
      break;

    case RobotState::Tuning:
      // TODO: implement auto‑tune using filtered error (omitted here for brevity)
      currentState = RobotState::LineFollowing;
      break;
  }
}
