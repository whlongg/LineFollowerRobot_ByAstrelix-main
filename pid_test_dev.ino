#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <string.h>
#include <stdio.h>
#include <math.h> // For abs() with floats

// --- CORE DEBUG & LOGGING ---
#define DEBUG 1
#define LOG_INFO 0
#define LOG_ERROR 1
#define LOG_PID 2
#define LOG_CALIB 3
#define LOG_STATE 4

// --- ROBOT CONFIGURATION ---
#define ENABLE_BYPASS_INTERSECTION 1
#define ENABLE_HIGH_SPEED_ON_STRAIGHT 1

// --- TIMING ---
const unsigned long LOOP_INTERVAL_MS = 2;
const unsigned long BLE_LOG_INTERVAL_MS = 50;

// --- SENSOR CONFIGURATION ---
const int SENSOR_COUNT = 4;
const int IR_PINS[SENSOR_COUNT] = {4, 3, 1, 0};
const int SENSOR_CALIB_SAMPLES = 32;
const int SENSOR_NORM_MAX = 1000;              // Giá trị chuẩn hóa tối đa (cao = trắng)
// --- Ngưỡng dựa trên ir_norm (0=Đen, 1000=Trắng) ---
const int INTERSECTION_DETECT_THRESHOLD = 250; // Giá trị ir_norm phải DƯỚI ngưỡng này để coi là đen (cho giao lộ)
const int LINE_SEARCH_DETECT_THRESHOLD = 300;  // Giá trị ir_norm phải DƯỚI ngưỡng này để tìm thấy line đen khi tìm kiếm
const int SHARP_TURN_OUTER_THRESHOLD = 200;    // Ngưỡng ir_norm cho cảm biến ngoài khi cua gấp (phải < ngưỡng này = đủ đen)
const int SHARP_TURN_INNER_THRESHOLD = 400;    // Ngưỡng ir_norm cho cảm biến trong khi cua gấp (phải < ngưỡng này = khá đen)


// --- MOTOR CONFIGURATION ---
const int PWM_PIN_L_A = 2;
const int PWM_PIN_L_B = 10;
const int PWM_PIN_R_A = 6;
const int PWM_PIN_R_B = 5;
const int LEFT_MOTOR_CHANNEL_A = 0;
const int LEFT_MOTOR_CHANNEL_B = 1;
const int RIGHT_MOTOR_CHANNEL_A = 2;
const int RIGHT_MOTOR_CHANNEL_B = 3;
const int PWM_FREQ = 20000;
const int PWM_RES = 8;
const int MAX_PWM = (1 << PWM_RES) - 1;
const float SCALE_RIGHT_MOTOR = 1.002; // Cần hiệu chuẩn tốt hơn

// --- PID & CONTROL PARAMETERS (Tunable via BLE) ---
float Kp = 3.66;
float Ki = 0.01;
float Kd = 39.0;
float INTEGRAL_MAX = 100.0f;
float D_FILTER_ALPHA = 0.5f;
float MAX_PID_OUTPUT = 150.0f; // Giới hạn chênh lệch tốc độ từ PID

// --- SPEED PARAMETERS (Tunable via BLE) ---
int max_straight_speed = 250;
int cornering_speed = 210;
float max_error_for_high_speed = 2.5; // Sai số (từ computeError) phải DƯỚI ngưỡng này để chạy tốc độ cao
float min_error_for_low_speed = 3.0;  // Sai số (từ computeError) phải TRÊN ngưỡng này để buộc chạy tốc độ thấp
int sharp_turn_speed_reduction = 60;

// --- INTERSECTION PARAMETERS ---
unsigned long state_start_time = 0;
const unsigned long BRAKING_DURATION_MS = 30;
const unsigned long MIN_TURN_DURATION_MS = 210;
const int INTERSECTION_BRAKE_SPEED = -MAX_PWM;
const int INTERSECTION_TURN_SPEED = 255;
const int SEARCHING_TURN_SPEED_LEFT = 210;
const int SEARCHING_TURN_SPEED_RIGHT = 80;

// --- HARDWARE PINS ---
const int IR_LED_ON = 21;
const int LED_CHECK = 8;
const int BUTTON_PIN = 7;

// --- BLE Definitions ---
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_RX "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID_TX "beb5483e-36e1-4688-b7f5-ea07361b26a9"
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristicRX = NULL;
BLECharacteristic *pCharacteristicTX = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
unsigned long lastBleLogTime = 0;

// --- GLOBAL VARIABLES ---
// Sensor Readings
uint16_t ir_min[SENSOR_COUNT]; // Giá trị ADC thô thấp nhất (khi ở trên màu đen)
uint16_t ir_max[SENSOR_COUNT]; // Giá trị ADC thô cao nhất (khi ở trên màu trắng)
int ir_raw[SENSOR_COUNT];
int ir_norm[SENSOR_COUNT]; // Giá trị chuẩn hóa: 0 (Đen) - SENSOR_NORM_MAX (Trắng)

// PID State
float last_error = 0.0f;
float integral = 0.0f;
float filtered_derivative = 0.0f;

// State Machine
enum RobotState {
    STATE_IDLE,
    STATE_LINE_FOLLOWING,
    STATE_INTERSECTION_DETECTED,
    STATE_INTERSECTION_BRAKING,
    STATE_INTERSECTION_TURNING,
    STATE_INTERSECTION_SEARCHING_LINE
};
RobotState current_state = STATE_IDLE;

// Logging Variables
float currentP = 0, currentI = 0, currentD = 0;
int currentLeftSpeed = 0, currentRightSpeed = 0;
float currentError = 0;

// Timing
unsigned long lastLoopTime = 0;

// --- FUNCTION DECLARATIONS ---
void setupMotors();
void setupBLE();
void calibrateSensors();
void loop();
void readIRSensors();
float computeError(const int* norm_values);
float computePID(float error);
void applyMotorSpeed(float error, float correction, const int* norm_values);
void setMotorSpeeds(int left, int right);
bool checkIntersection(const int* norm_values);
bool checkLineFound(const int* norm_values);
void handleBLECommands(const std::string& cmd);
void notifyTuningValues();
class MyServerCallbacks : public BLEServerCallbacks;
class MyCallbacks : public BLECharacteristicCallbacks;
void debugLog(uint8_t level, const char* msg);
void waitForButtonPress();
void blinkLed(int pin, int times, int duration_ms);

// --- SETUP ---
void setup() {
    Serial.begin(115200);
    debugLog(LOG_INFO, "--- System Booting ---");
    debugLog(LOG_INFO, "Raw ADC: 0=Black, 4095=White");
    debugLog(LOG_INFO, "Normalized IR: 0=Black, 1000=White");

    pinMode(IR_LED_ON, OUTPUT);
    pinMode(LED_CHECK, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    digitalWrite(IR_LED_ON, HIGH);
    digitalWrite(LED_CHECK, LOW);

    setupMotors();
    debugLog(LOG_INFO, "Motors Initialized.");

    // Default calibration values (Raw ADC: 0=Black, 4095=White)
    for (int i = 0; i < SENSOR_COUNT; ++i) {
        ir_min[i] = 50;    // Expect black to be low raw value
        ir_max[i] = 4000; // Expect white to be high raw value
    }
    debugLog(LOG_INFO, "Sensors Initialized (Default Calibration).");

    debugLog(LOG_INFO, "Press button to start calibration...");
    waitForButtonPress();
    calibrateSensors();

    setupBLE();
    debugLog(LOG_INFO, "BLE Initialized. Waiting for connection...");

    lastLoopTime = millis();
    last_error = 0.0f; integral = 0.0f; filtered_derivative = 0.0f;
    current_state = STATE_IDLE;

    debugLog(LOG_INFO, "Setup Complete. Robot is IDLE. Press button to start.");
    blinkLed(LED_CHECK, 3, 100);
}

// --- MAIN LOOP ---
void loop() {
    unsigned long currentTime = millis();

    if (currentTime - lastLoopTime < LOOP_INTERVAL_MS) {
        return;
    }
    unsigned long loopStartTime = micros();
    lastLoopTime = currentTime;

    readIRSensors(); // Updates ir_norm[] (0=Black, 1000=White)

    RobotState previous_state = current_state;

    switch (current_state) {
        case STATE_IDLE:
            setMotorSpeeds(0, 0);
            if (digitalRead(BUTTON_PIN) == LOW) {
                debugLog(LOG_STATE, "Button pressed. Starting Line Following.");
                delay(50);
                while(digitalRead(BUTTON_PIN) == LOW); // Wait for release
                delay(50);
                current_state = STATE_LINE_FOLLOWING;
                last_error = 0.0f; integral = 0.0f; filtered_derivative = 0.0f;
                blinkLed(LED_CHECK, 2, 50);
            }
            break;

        case STATE_LINE_FOLLOWING:
            currentError = computeError(ir_norm);
            float correction = computePID(currentError);
            applyMotorSpeed(currentError, correction, ir_norm);

            if (ENABLE_BYPASS_INTERSECTION && checkIntersection(ir_norm)) {
                debugLog(LOG_STATE, "Intersection detected!");
                current_state = STATE_INTERSECTION_DETECTED;
            }
            break;

        case STATE_INTERSECTION_DETECTED:
            debugLog(LOG_STATE, "Braking for intersection...");
            setMotorSpeeds(INTERSECTION_BRAKE_SPEED, INTERSECTION_BRAKE_SPEED);
            current_state = STATE_INTERSECTION_BRAKING;
            state_start_time = currentTime;
            integral = 0; last_error = 0; filtered_derivative = 0;
            break;

        case STATE_INTERSECTION_BRAKING:
            setMotorSpeeds(INTERSECTION_BRAKE_SPEED, INTERSECTION_BRAKE_SPEED);
            if (currentTime - state_start_time >= BRAKING_DURATION_MS) {
                debugLog(LOG_STATE, "Braking complete. Starting turn.");
                setMotorSpeeds(INTERSECTION_TURN_SPEED, 0); // Turn right
                current_state = STATE_INTERSECTION_TURNING;
                state_start_time = currentTime;
            }
            break;

        case STATE_INTERSECTION_TURNING:
            setMotorSpeeds(INTERSECTION_TURN_SPEED, 0);
            if (currentTime - state_start_time >= MIN_TURN_DURATION_MS) {
                 // Optimization: Check for line during turn after min duration? Could save time.
                 if (checkLineFound(ir_norm)) {
                     debugLog(LOG_STATE, "Line found during turn! Resuming.");
                     current_state = STATE_LINE_FOLLOWING;
                 } else {
                     // If not found during turn, start dedicated search phase
                    debugLog(LOG_STATE, "Min turn duration met. Searching for line.");
                    setMotorSpeeds(SEARCHING_TURN_SPEED_LEFT, SEARCHING_TURN_SPEED_RIGHT);
                    current_state = STATE_INTERSECTION_SEARCHING_LINE;
                    state_start_time = currentTime;
                 }
            }
            break;

        case STATE_INTERSECTION_SEARCHING_LINE:
            setMotorSpeeds(SEARCHING_TURN_SPEED_LEFT, SEARCHING_TURN_SPEED_RIGHT);
            if (checkLineFound(ir_norm)) {
                debugLog(LOG_STATE, "Line found! Resuming line following.");
                current_state = STATE_LINE_FOLLOWING;
            }
            // Add search timeout?
            // if (currentTime - state_start_time > MAX_SEARCH_TIME_MS) { ... }
            break;
    }

    if (current_state != previous_state) {
        char stateMsg[50];
        snprintf(stateMsg, sizeof(stateMsg), "State changed from %d to %d", previous_state, current_state);
        debugLog(LOG_STATE, stateMsg);
    }

    // --- BLE Connection & Logging ---
    // (BLE connection management code remains the same)
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = true;
        debugLog(LOG_INFO, "BLE device connected");
    }
    if (!deviceConnected && oldDeviceConnected) {
        oldDeviceConnected = false;
        debugLog(LOG_INFO, "BLE device disconnected, restarting advertising");
        pServer->startAdvertising();
    }

    if (deviceConnected && pCharacteristicTX != NULL && (currentTime - lastBleLogTime >= BLE_LOG_INTERVAL_MS)) {
        lastBleLogTime = currentTime;
        char logBuffer[200];
        // Log ir_norm (0=Black, 1000=White)
        snprintf(logBuffer, sizeof(logBuffer), "%lu,%d,%d,%d,%d,%.3f,%.3f,%.3f,%.3f,%d,%d,%d\n",
                 currentTime,
                 ir_norm[0], ir_norm[1], ir_norm[2], ir_norm[3],
                 currentP, currentI, currentD,
                 currentError,
                 currentLeftSpeed, currentRightSpeed,
                 current_state
        );

        if (strlen(logBuffer) >= sizeof(logBuffer) - 1) {
             debugLog(LOG_ERROR, "BLE log buffer overflow!");
        } else {
            pCharacteristicTX->setValue((uint8_t*)logBuffer, strlen(logBuffer));
            pCharacteristicTX->notify();
        }
    }

    // --- Loop Performance Logging ---
#if DEBUG
    unsigned long loopEndTime = micros();
    if(loopEndTime - loopStartTime > (LOOP_INTERVAL_MS * 1000 + 500)) {
       char perfLog[50];
       snprintf(perfLog, sizeof(perfLog), "Loop time warning: %lu us", loopEndTime - loopStartTime);
       debugLog(LOG_ERROR, perfLog);
    }
#endif
} // --- END OF loop() ---


// --- Sensor Calibration ---
// Calibrates based on Raw ADC: 0=Black, 4095=White
void calibrateSensors() {
    debugLog(LOG_CALIB, "Calibration Started.");
    digitalWrite(LED_CHECK, HIGH);

    // 1. Calibrate BLACK (Min raw readings)
    debugLog(LOG_CALIB, "Place sensors on BLACK line and press button.");
    waitForButtonPress();
    blinkLed(LED_CHECK, 1, 50);

    uint32_t ir_sum_min[SENSOR_COUNT] = {0};
    for (int sample = 0; sample < SENSOR_CALIB_SAMPLES; ++sample) {
        for (int i = 0; i < SENSOR_COUNT; ++i) {
            ir_sum_min[i] += analogRead(IR_PINS[i]);
        }
        delay(5);
    }
    for (int i = 0; i < SENSOR_COUNT; ++i) {
        ir_min[i] = ir_sum_min[i] / SENSOR_CALIB_SAMPLES; // Store average raw min
        char calibMsg[50];
        snprintf(calibMsg, sizeof(calibMsg), "IR %d MIN (Black Raw): %d", i, ir_min[i]);
        debugLog(LOG_CALIB, calibMsg);
    }

    // 2. Calibrate WHITE (Max raw readings)
    debugLog(LOG_CALIB, "Place sensors on WHITE surface and press button.");
    waitForButtonPress();
    blinkLed(LED_CHECK, 1, 50);

    uint32_t ir_sum_max[SENSOR_COUNT] = {0};
    for (int sample = 0; sample < SENSOR_CALIB_SAMPLES; ++sample) {
        for (int i = 0; i < SENSOR_COUNT; ++i) {
            ir_sum_max[i] += analogRead(IR_PINS[i]);
        }
        delay(5);
    }
    for (int i = 0; i < SENSOR_COUNT; ++i) {
        ir_max[i] = ir_sum_max[i] / SENSOR_CALIB_SAMPLES; // Store average raw max
        // Sanity checks
        if (ir_max[i] <= ir_min[i]) {
            debugLog(LOG_ERROR, "Calibration Error: Max <= Min. Adjusting.");
            ir_max[i] = ir_min[i] + 100; // Ensure max is greater than min
             if (ir_min[i] == 0) ir_min[i] = 1; // Avoid min being 0 if max was also 0 initially
        }
         if (ir_max[i] == ir_min[i]) { // Prevent division by zero in map
             ir_max[i] = ir_min[i] + 1;
         }

        char calibMsg[50];
        snprintf(calibMsg, sizeof(calibMsg), "IR %d MAX (White Raw): %d", i, ir_max[i]);
        debugLog(LOG_CALIB, calibMsg);
    }

    digitalWrite(LED_CHECK, LOW);
    debugLog(LOG_CALIB, "Calibration Finished.");
    blinkLed(LED_CHECK, 3, 50);
}

// --- Read and Normalize IR Sensors ---
// Reads raw ADC (0=Black, 4095=White) and calculates normalized values (0=Black, 1000=White)
void readIRSensors() {
    for (int i = 0; i < SENSOR_COUNT; i++) {
        ir_raw[i] = analogRead(IR_PINS[i]);
        // Normalize: map raw value from [min_raw_black, max_raw_white] to [0, NORM_MAX]
        long mapped_value = map(ir_raw[i], ir_min[i], ir_max[i], 0, SENSOR_NORM_MAX);
        // Constrain ensures value stays within bounds [0, 1000]
        ir_norm[i] = constrain(mapped_value, 0, SENSOR_NORM_MAX);
        // Result: ir_norm[i] is ~0 for Black, ~1000 for White
    }
}


// --- Error Calculation (Weighted Center) ---
// Calculates weighted error based on normalized values (0=Black, 1000=White)
// Positive error = line is to the right, Negative error = line is to the left
float computeError(const int* norm_values) {
    // Weights: Positive for right sensors, negative for left sensors.
    // Since 0=Black, 1000=White, a sensor seeing WHITE contributes MORE to the sum.
    // If line is to the RIGHT: Left sensors see WHITE (high norm_values), Right sensors see BLACK (low norm_values).
    // To get POSITIVE error, weights should be NEGATIVE on the LEFT, POSITIVE on the RIGHT.
    float weights[SENSOR_COUNT] = {-3.0f, -1.0f, 1.0f, 3.0f};

    long weighted_sum = 0;
    long sensor_sum = 0; // Sum of (SENSOR_NORM_MAX - norm_value) to weight by "blackness"
    bool on_line = false;

    for (int i = 0; i < SENSOR_COUNT; i++) {
        int value = norm_values[i]; // 0=Black, 1000=White
        int inverted_value = SENSOR_NORM_MAX - value; // 0=White, 1000=Black (represents blackness)

        if (inverted_value > (SENSOR_NORM_MAX / 10)) { // Check if blackness > 100
            on_line = true;
        }
        // Weighted sum uses the original weights but applies them to the direct norm_value
        // Example: Line right -> Left sees white (high value * neg weight), Right sees black (low value * pos weight) -> Negative sum? -> Need to invert the final error?
        // Let's rethink: Calculate weighted average of sensor *positions* weighted by *blackness*.
        // Sensor positions: e.g., -1.5, -0.5, 0.5, 1.5 (or indices -2, -1, 1, 2)
        // Weights = blackness = inverted_value
        // weighted_sum = pos[0]*inv[0] + pos[1]*inv[1] + pos[2]*inv[2] + pos[3]*inv[3]
        // total_weight = inv[0] + inv[1] + inv[2] + inv[3]
        // error = weighted_sum / total_weight

        // Simpler approach: Use the original weights {-3, -1, 1, 3} applied to the 'blackness' value.
        // Line right -> Left sees white (low inverted_value), Right sees black (high inverted_value)
        // weighted_sum = (low * -3) + (low * -1) + (high * 1) + (high * 3) -> POSITIVE. This seems correct.
        weighted_sum += (long)inverted_value * weights[i];
        sensor_sum += inverted_value; // Sum of blackness
    }

    float error = 0.0f;
    if (on_line && sensor_sum > 0) {
        error = (float)weighted_sum / sensor_sum;
    } else if (!on_line) {
        // Lost line (all sensors see white -> all inverted_values are low/zero)
        float max_off_line_error = 3.0; // Corresponds to magnitude of outer weights
        error = (last_error >= 0) ? max_off_line_error : -max_off_line_error;
    } else {
        // On line but sensor_sum is zero? (Shouldn't happen if on_line is true)
        error = last_error;
    }

    // Optional: Clamp error?
    // error = constrain(error, -4.0, 4.0);

    currentError = error;
    return error;
}


// --- PID Controller (Remains the same as previous corrected version) ---
float computePID(float error) {
    unsigned long currentTime = micros();
    static unsigned long lastPIDTime = 0;
    float dt = (currentTime - lastPIDTime) / 1000000.0f;
    lastPIDTime = currentTime;

    if (dt <= 0.00001f) {
        dt = LOOP_INTERVAL_MS / 1000.0f;
    }

    currentP = Kp * error;

    // Integral with clamping
    integral += error * dt;
    integral = constrain(integral, -INTEGRAL_MAX, INTEGRAL_MAX);
    currentI = Ki * integral;

    // Derivative with filter
    float derivative = (error - last_error) / dt;
    filtered_derivative = D_FILTER_ALPHA * derivative + (1.0f - D_FILTER_ALPHA) * filtered_derivative;
    currentD = Kd * filtered_derivative;

    float pid_output = currentP + currentI + currentD;
    pid_output = constrain(pid_output, -MAX_PID_OUTPUT, MAX_PID_OUTPUT);

    last_error = error;
    return pid_output;
}


// --- Apply Motor Speed ---
// Calculates and sets motor speeds based on PID correction and dynamic speed logic
void applyMotorSpeed(float error, float correction, const int* norm_values) {
    float abs_error = abs(error);
    int base_speed;

#if ENABLE_HIGH_SPEED_ON_STRAIGHT
    // Sharp turn detection using norm_values (0=Black, 1000=White)
    // Condition: Outer sensor is very black (< OuterThresh) AND Inner sensor is quite black (< InnerThresh)
    bool sharp_turn = (norm_values[0] < SHARP_TURN_OUTER_THRESHOLD && norm_values[1] < SHARP_TURN_INNER_THRESHOLD) ||
                      (norm_values[3] < SHARP_TURN_OUTER_THRESHOLD && norm_values[2] < SHARP_TURN_INNER_THRESHOLD);

    if (sharp_turn) {
        base_speed = max(0, cornering_speed - sharp_turn_speed_reduction);
        // debugLog(LOG_PID, "Sharp turn detected, reducing speed.");
    } else if (abs_error <= max_error_for_high_speed) {
        base_speed = max_straight_speed;
    } else if (abs_error >= min_error_for_low_speed) {
        base_speed = cornering_speed;
    } else {
        base_speed = map(abs_error * 100,
                         max_error_for_high_speed * 100,
                         min_error_for_low_speed * 100,
                         max_straight_speed,
                         cornering_speed);
        base_speed = constrain(base_speed, cornering_speed, max_straight_speed);
    }
#else
    base_speed = cornering_speed;
#endif

    int left_speed = base_speed + correction;
    int right_speed = base_speed - correction;

    left_speed = constrain(left_speed, 0, MAX_PWM);
    right_speed = constrain(right_speed, 0, MAX_PWM);

    currentLeftSpeed = left_speed;
    currentRightSpeed = right_speed;

    setMotorSpeeds(currentLeftSpeed, currentRightSpeed);
}


// --- Motor Driver Control (Remains the same) ---
void setMotorSpeeds(int left, int right) {
    if (right > 0) {
        right = (int)((float)right * SCALE_RIGHT_MOTOR);
        right = min(right, MAX_PWM);
    }
    left = constrain(left, -MAX_PWM, MAX_PWM);
    right = constrain(right, -MAX_PWM, MAX_PWM);

    if (left > 0) { ledcWrite(LEFT_MOTOR_CHANNEL_A, left); ledcWrite(LEFT_MOTOR_CHANNEL_B, 0); }
    else if (left < 0) { ledcWrite(LEFT_MOTOR_CHANNEL_A, 0); ledcWrite(LEFT_MOTOR_CHANNEL_B, abs(left)); }
    else { ledcWrite(LEFT_MOTOR_CHANNEL_A, 0); ledcWrite(LEFT_MOTOR_CHANNEL_B, 0); }

    if (right > 0) { ledcWrite(RIGHT_MOTOR_CHANNEL_A, right); ledcWrite(RIGHT_MOTOR_CHANNEL_B, 0); }
    else if (right < 0) { ledcWrite(RIGHT_MOTOR_CHANNEL_A, 0); ledcWrite(RIGHT_MOTOR_CHANNEL_B, abs(right)); }
    else { ledcWrite(RIGHT_MOTOR_CHANNEL_A, 0); ledcWrite(RIGHT_MOTOR_CHANNEL_B, 0); }
}

// --- Setup Motors (Remains the same) ---
void setupMotors() {
    ledcAttachChannel(PWM_PIN_L_A, PWM_FREQ, PWM_RES, LEFT_MOTOR_CHANNEL_A);
    ledcAttachChannel(PWM_PIN_L_B, PWM_FREQ, PWM_RES, LEFT_MOTOR_CHANNEL_B);
    ledcAttachChannel(PWM_PIN_R_A, PWM_FREQ, PWM_RES, RIGHT_MOTOR_CHANNEL_A);
    ledcAttachChannel(PWM_PIN_R_B, PWM_FREQ, PWM_RES, RIGHT_MOTOR_CHANNEL_B);
    setMotorSpeeds(0, 0);
}

// --- State Machine Helper Functions ---
// Checks if all IR sensors detect black (norm_value < threshold)
bool checkIntersection(const int* norm_values) {
    // norm_values: 0=Black, 1000=White
    for (int i = 0; i < SENSOR_COUNT; ++i) {
        if (norm_values[i] >= INTERSECTION_DETECT_THRESHOLD) { // If any sensor is NOT black enough (value too high)
            return false;
        }
    }
    // If loop completes, all sensors are below the threshold (black enough)
    return true;
}

// Checks if the inner sensors detect the line again (norm_value < threshold)
bool checkLineFound(const int* norm_values) {
     // norm_values: 0=Black, 1000=White
    // Line found if either middle sensor is black enough
    return (norm_values[1] < LINE_SEARCH_DETECT_THRESHOLD || norm_values[2] < LINE_SEARCH_DETECT_THRESHOLD);
}


// --- BLE Setup, Callbacks, Command Handling, Notify (Remains the same) ---
// ... (Copy the BLE functions from the previous corrected code block) ...
// --- BLE Setup ---
void setupBLE() {
    BLEDevice::init("ESP32_LineFollower"); // Set device name
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks()); // Set server callbacks

    BLEService *pService = pServer->createService(SERVICE_UUID);

    // RX Characteristic (Client writes to this)
    pCharacteristicRX = pService->createCharacteristic(
        CHARACTERISTIC_UUID_RX,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR // Allow Write Without Response
    );
    pCharacteristicRX->setCallbacks(new MyCallbacks()); // Set characteristic callbacks

    // TX Characteristic (Server notifies this)
    pCharacteristicTX = pService->createCharacteristic(
        CHARACTERISTIC_UUID_TX,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    pCharacteristicTX->addDescriptor(new BLE2902()); // Add descriptor for notifications

    pService->start();

    // Setup Advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    // pAdvertising->setMinPreferred(0x06); // Helps with faster connections on some devices
    // pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
}

// --- BLE Server Callbacks ---
class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        // Optional: Stop advertising once connected?
        // BLEDevice::getAdvertising()->stop();
    }

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        // Advertising is restarted in the main loop's connection management section
    }
};

// --- BLE Characteristic Callbacks ---
class MyCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue(); // Get value as std::string
        if (rxValue.length() > 0) {
            // debugLog(LOG_INFO, "BLE RX Raw: "); // Log raw received data
            // Serial.println(rxValue.c_str());
            handleBLECommands(rxValue); // Process the command
        }
    }
};

// --- Handle Commands Received via BLE ---
// Parses commands like "kp=1.2", "ms=200", "getall"
void handleBLECommands(const std::string& cmd_str) {
    const char* cmd = cmd_str.c_str(); // Work with C-style string for parsing
    char logBuffer[200]; // Buffer for logging received command and responses
    snprintf(logBuffer, sizeof(logBuffer), "BLE RX: %s", cmd);
    debugLog(LOG_INFO, logBuffer);

    bool valueChanged = false;

    // Use sscanf for robust parsing of "key=value" format
    float float_val;
    int int_val;

    if (sscanf(cmd, "kp=%f", &float_val) == 1) { Kp = float_val; valueChanged = true; }
    else if (sscanf(cmd, "ki=%f", &float_val) == 1) { Ki = float_val; valueChanged = true; }
    else if (sscanf(cmd, "kd=%f", &float_val) == 1) { Kd = float_val; valueChanged = true; }
    else if (sscanf(cmd, "ms=%d", &int_val) == 1) { max_straight_speed = constrain(int_val, 0, MAX_PWM); valueChanged = true; }
    else if (sscanf(cmd, "cs=%d", &int_val) == 1) { cornering_speed = constrain(int_val, 0, MAX_PWM); valueChanged = true; }
    else if (sscanf(cmd, "maxerr=%f", &float_val) == 1) { max_error_for_high_speed = float_val; valueChanged = true; }
    else if (sscanf(cmd, "minerr=%f", &float_val) == 1) { min_error_for_low_speed = float_val; valueChanged = true; }
    else if (sscanf(cmd, "sharpred=%d", &int_val) == 1) { sharp_turn_speed_reduction = int_val; valueChanged = true; }
    // Add more parameters here if needed
    else if (strcmp(cmd, "getall") == 0) { valueChanged = true; } // Trigger sending current values
    else if (strcmp(cmd, "start") == 0) { // Command to start the robot if in IDLE
        if (current_state == STATE_IDLE) {
             debugLog(LOG_STATE, "BLE Start command received.");
             current_state = STATE_LINE_FOLLOWING;
             last_error = 0.0f; integral = 0.0f; filtered_derivative = 0.0f; // Reset PID
        }
    } else if (strcmp(cmd, "stop") == 0) { // Command to stop the robot
        debugLog(LOG_STATE, "BLE Stop command received.");
        current_state = STATE_IDLE;
        setMotorSpeeds(0, 0);
    }


    if (valueChanged) {
        // Log and notify updated values
        snprintf(logBuffer, sizeof(logBuffer),
                 "Values Updated: Kp=%.2f Ki=%.2f Kd=%.2f MS=%d CS=%d MaxErr=%.2f MinErr=%.2f SharpRed=%d",
                 Kp, Ki, Kd, max_straight_speed, cornering_speed,
                 max_error_for_high_speed, min_error_for_low_speed, sharp_turn_speed_reduction);
        debugLog(LOG_PID, logBuffer);
        notifyTuningValues(); // Send all current values back to the client
    }
}

// --- Send ALL Tuning Values via BLE Notification ---
void notifyTuningValues() {
    if (deviceConnected && pCharacteristicTX != NULL) {
        char buffer[250]; // Ensure buffer is large enough
        snprintf(buffer, sizeof(buffer),
                 "Kp:%.3f,Ki:%.3f,Kd:%.3f,MS:%d,CS:%d,MaxE:%.2f,MinE:%.2f,SRed:%d",
                 Kp, Ki, Kd, max_straight_speed, cornering_speed, max_error_for_high_speed,
                 min_error_for_low_speed, sharp_turn_speed_reduction);

        if (strlen(buffer) >= sizeof(buffer) - 1) {
            debugLog(LOG_ERROR, "Notify buffer overflow!");
             pCharacteristicTX->setValue("Error: Data too long");
        } else {
             pCharacteristicTX->setValue((uint8_t*)buffer, strlen(buffer));
        }
        pCharacteristicTX->notify();
        // debugLog(LOG_PID, "Notified All Values."); // Keep log concise
    }
}


// --- Utility Functions (Remains the same) ---
void debugLog(uint8_t level, const char* msg) {
#if DEBUG
    const char* prefix;
    switch (level) {
        case LOG_INFO: prefix = "[INFO] "; break;
        case LOG_ERROR: prefix = "[ERROR] "; break;
        case LOG_PID: prefix = "[PID] "; break;
        case LOG_CALIB: prefix = "[CALIB] "; break;
        case LOG_STATE: prefix = "[STATE] "; break;
        default: prefix = "[LOG] "; break;
    }
    Serial.print(prefix);
    Serial.println(msg);
#endif
}

void waitForButtonPress() {
    while (digitalRead(BUTTON_PIN) == HIGH) { delay(10); }
    delay(50);
    while (digitalRead(BUTTON_PIN) == LOW) { delay(10); }
    delay(50);
}

void blinkLed(int pin, int times, int duration_ms) {
    for (int i = 0; i < times; ++i) {
        digitalWrite(pin, HIGH); delay(duration_ms / 2);
        digitalWrite(pin, LOW); delay(duration_ms / 2);
    }
}