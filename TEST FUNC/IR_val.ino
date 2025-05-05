#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

// --- CORE DEBUG & LOGGING ---
#define DEBUG 1 // Bật log Serial để theo dõi trạng thái
#define LOG_INFO 0
#define LOG_ERROR 1
#define LOG_CALIB 3
#define LOG_STATE 4

// --- TIMING ---
// Giảm LOOP_INTERVAL để đọc cảm biến thường xuyên hơn một chút
const unsigned long LOOP_INTERVAL_MS = 1;
// Tăng tần suất gửi log BLE để bắt chi tiết khi di chuyển tay
const unsigned long BLE_LOG_INTERVAL_MS = 20; // Gửi dữ liệu mỗi 20ms

// --- SENSOR CONFIGURATION ---
const int SENSOR_COUNT = 4;
const int IR_PINS[SENSOR_COUNT] = {4, 3, 1, 0};
const int SENSOR_CALIB_SAMPLES = 32;
const int SENSOR_NORM_MAX = 1000; // Giá trị chuẩn hóa tối đa (cao = trắng)

// --- MOTOR CONFIGURATION (Giữ lại để setup nhưng không dùng) ---
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

// --- HARDWARE PINS ---
const int IR_LED_ON = 21;
const int LED_CHECK = 8;
const int BUTTON_PIN = 7;

// --- BLE Definitions ---
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_RX "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID_TX "beb5483e-36e1-4688-b7f5-ea07361b26a9" // Characteristic để gửi dữ liệu cảm biến
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristicRX = NULL; // Vẫn giữ để có thể nhận lệnh (ví dụ: dừng log sau này)
BLECharacteristic *pCharacteristicTX = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
unsigned long lastBleLogTime = 0;

// --- GLOBAL VARIABLES ---
// Sensor Readings
uint16_t ir_min[SENSOR_COUNT]; // Giá trị ADC thô thấp nhất (khi ở trên màu đen)
uint16_t ir_max[SENSOR_COUNT]; // Giá trị ADC thô cao nhất (khi ở trên màu trắng)
int ir_raw[SENSOR_COUNT];      // Mảng lưu giá trị ADC thô mới nhất
int ir_norm[SENSOR_COUNT];     // Mảng lưu giá trị chuẩn hóa mới nhất (0=Black, 1000=White)

// State Machine
enum RobotState {
    STATE_CALIBRATING,
    STATE_SENSOR_SCAN // Trạng thái chính: chỉ đọc và gửi cảm biến
};
RobotState current_state = STATE_CALIBRATING;

// Timing
unsigned long lastLoopTime = 0;

// --- FUNCTION DECLARATIONS ---
void setupMotors();
void setupBLE();
void calibrateSensors();
void loop();
void readIRSensors();
void handleBLECommands(const std::string& cmd); // Giữ lại để tương thích
void notifySensorData(); // Hàm mới để gửi dữ liệu cảm biến
class MyServerCallbacks : public BLEServerCallbacks;
class MyCallbacks : public BLECharacteristicCallbacks;
void debugLog(uint8_t level, const char* msg);
void waitForButtonPress();
void blinkLed(int pin, int times, int duration_ms);
void setMotorSpeeds(int left, int right); // Giữ lại để dừng motor

// --- SETUP ---
void setup() {
    Serial.begin(115200);
    debugLog(LOG_INFO, "--- Sensor Scan Test ---");
    debugLog(LOG_INFO, "Raw ADC: 0=Black, 4095=White");
    debugLog(LOG_INFO, "Normalized IR: 0=Black, 1000=White");

    pinMode(IR_LED_ON, OUTPUT);
    pinMode(LED_CHECK, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    digitalWrite(IR_LED_ON, HIGH); // Bật LED hồng ngoại
    digitalWrite(LED_CHECK, LOW);

    setupMotors(); // Khởi tạo motor
    setMotorSpeeds(0, 0); // Đảm bảo motor dừng hẳn
    debugLog(LOG_INFO, "Motors Initialized & Stopped.");

    // Default calibration values
    for (int i = 0; i < SENSOR_COUNT; ++i) {
        ir_min[i] = 50;
        ir_max[i] = 4000;
    }

    current_state = STATE_CALIBRATING;
    debugLog(LOG_STATE, "State: CALIBRATING. Press button to start.");
    waitForButtonPress();
    calibrateSensors();

    // Sau khi hiệu chuẩn xong, chuyển sang trạng thái quét cảm biến
    current_state = STATE_SENSOR_SCAN;
    debugLog(LOG_STATE, "State: SENSOR_SCAN. Waiting for BLE connection...");

    setupBLE(); // Khởi tạo BLE sau khi hiệu chuẩn
    debugLog(LOG_INFO, "BLE Initialized.");

    lastLoopTime = millis();
    blinkLed(LED_CHECK, 3, 100); // Báo hiệu sẵn sàng quét
}

// --- MAIN LOOP ---
void loop() {
    unsigned long currentTime = millis();

    // --- Loop Timing Control ---
    if (currentTime - lastLoopTime < LOOP_INTERVAL_MS) {
        return;
    }
    lastLoopTime = currentTime;

    // --- Read Sensors ---
    // Luôn đọc cảm biến ở mỗi vòng lặp trong trạng thái quét
    if (current_state == STATE_SENSOR_SCAN) {
        readIRSensors(); // Cập nhật ir_raw[] và ir_norm[]
    }

    // --- State Machine (rất đơn giản cho test này) ---
    switch (current_state) {
        case STATE_CALIBRATING:
            // Đã xử lý trong setup()
            break;

        case STATE_SENSOR_SCAN:
            // Logic chính: Gửi dữ liệu qua BLE nếu được kết nối và đủ thời gian
            if (deviceConnected && pCharacteristicTX != NULL && (currentTime - lastBleLogTime >= BLE_LOG_INTERVAL_MS)) {
                notifySensorData(); // Gửi gói dữ liệu cảm biến hiện tại
                lastBleLogTime = currentTime;
            }
            // Đảm bảo motor luôn dừng
            setMotorSpeeds(0, 0);
            break;
    }

    // --- BLE Connection Management ---
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = true;
        debugLog(LOG_INFO, "BLE device connected. Starting sensor data stream.");
        // Reset timer log khi mới kết nối để gửi dữ liệu ngay
        lastBleLogTime = millis() - BLE_LOG_INTERVAL_MS;
    }
    if (!deviceConnected && oldDeviceConnected) {
        oldDeviceConnected = false;
        debugLog(LOG_INFO, "BLE device disconnected. Stopping sensor data stream.");
        debugLog(LOG_INFO, "Restarting advertising...");
        pServer->startAdvertising();
    }

} // --- END OF loop() ---


// --- Read and Normalize IR Sensors ---
// Reads raw ADC (0=Black, 4095=White) and calculates normalized values (0=Black, 1000=White)
void readIRSensors() {
    for (int i = 0; i < SENSOR_COUNT; i++) {
        ir_raw[i] = analogRead(IR_PINS[i]);
        long mapped_value = map(ir_raw[i], ir_min[i], ir_max[i], 0, SENSOR_NORM_MAX);
        ir_norm[i] = constrain(mapped_value, 0, SENSOR_NORM_MAX);
    }
}

// --- Send Sensor Data via BLE Notification ---
void notifySensorData() {
    char buffer[150]; // Buffer đủ lớn cho timestamp + 8 giá trị int + dấu phẩy + ký tự kết thúc
    snprintf(buffer, sizeof(buffer), "%lu,%d,%d,%d,%d,%d,%d,%d,%d\n",
             millis(),
             ir_raw[0], ir_raw[1], ir_raw[2], ir_raw[3],
             ir_norm[0], ir_norm[1], ir_norm[2], ir_norm[3]);

    if (strlen(buffer) >= sizeof(buffer) - 1) {
        debugLog(LOG_ERROR, "Sensor data log buffer overflow!");
    } else {
        pCharacteristicTX->setValue((uint8_t*)buffer, strlen(buffer));
        pCharacteristicTX->notify();
    }
}


// --- Sensor Calibration (Giữ nguyên logic) ---
void calibrateSensors() {
    debugLog(LOG_CALIB, "Calibration Started.");
    digitalWrite(LED_CHECK, HIGH);

    // 1. Calibrate BLACK (Min raw readings)
    debugLog(LOG_CALIB, "Place sensors on BLACK line and press button.");
    waitForButtonPress();
    blinkLed(LED_CHECK, 1, 50);

    uint32_t ir_sum_min[SENSOR_COUNT] = {0};
    for (int sample = 0; sample < SENSOR_CALIB_SAMPLES; ++sample) {
        for (int i = 0; i < SENSOR_COUNT; ++i) { ir_sum_min[i] += analogRead(IR_PINS[i]); }
        delay(5);
    }
    for (int i = 0; i < SENSOR_COUNT; ++i) {
        ir_min[i] = ir_sum_min[i] / SENSOR_CALIB_SAMPLES;
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
        for (int i = 0; i < SENSOR_COUNT; ++i) { ir_sum_max[i] += analogRead(IR_PINS[i]); }
        delay(5);
    }
    for (int i = 0; i < SENSOR_COUNT; ++i) {
        ir_max[i] = ir_sum_max[i] / SENSOR_CALIB_SAMPLES;
        if (ir_max[i] <= ir_min[i]) {
            debugLog(LOG_ERROR, "Calibration Error: Max <= Min. Adjusting.");
            ir_max[i] = ir_min[i] + 100;
            if (ir_min[i] == 0) ir_min[i] = 1;
        }
        if (ir_max[i] == ir_min[i]) { ir_max[i] = ir_min[i] + 1; }
        char calibMsg[50];
        snprintf(calibMsg, sizeof(calibMsg), "IR %d MAX (White Raw): %d", i, ir_max[i]);
        debugLog(LOG_CALIB, calibMsg);
    }

    digitalWrite(LED_CHECK, LOW);
    debugLog(LOG_CALIB, "Calibration Finished.");
}


// --- Motor Driver Control (Chỉ dùng để dừng) ---
void setMotorSpeeds(int left, int right) {
    // Đảm bảo giá trị là 0 để dừng
    left = 0;
    right = 0;

    ledcWrite(LEFT_MOTOR_CHANNEL_A, 0);
    ledcWrite(LEFT_MOTOR_CHANNEL_B, 0);
    ledcWrite(RIGHT_MOTOR_CHANNEL_A, 0);
    ledcWrite(RIGHT_MOTOR_CHANNEL_B, 0);
}

// --- Setup Motors (Giữ nguyên) ---
void setupMotors() {
    ledcAttachChannel(PWM_PIN_L_A, PWM_FREQ, PWM_RES, LEFT_MOTOR_CHANNEL_A);
    ledcAttachChannel(PWM_PIN_L_B, PWM_FREQ, PWM_RES, LEFT_MOTOR_CHANNEL_B);
    ledcAttachChannel(PWM_PIN_R_A, PWM_FREQ, PWM_RES, RIGHT_MOTOR_CHANNEL_A);
    ledcAttachChannel(PWM_PIN_R_B, PWM_FREQ, PWM_RES, RIGHT_MOTOR_CHANNEL_B);
}

// --- BLE Setup (Giữ nguyên) ---
void setupBLE() {
    BLEDevice::init("ESP32_SensorScan"); // Đổi tên để phân biệt
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pService = pServer->createService(SERVICE_UUID);
    pCharacteristicRX = pService->createCharacteristic(
        CHARACTERISTIC_UUID_RX,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
    );
    pCharacteristicRX->setCallbacks(new MyCallbacks());
    pCharacteristicTX = pService->createCharacteristic(
        CHARACTERISTIC_UUID_TX,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    pCharacteristicTX->addDescriptor(new BLE2902());
    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    BLEDevice::startAdvertising();
}

// --- BLE Server Callbacks (Giữ nguyên) ---
class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { deviceConnected = true; }
    void onDisconnect(BLEServer* pServer) { deviceConnected = false; }
};

// --- BLE Characteristic Callbacks ---
// Có thể thêm lệnh để dừng/bắt đầu log ở đây nếu muốn
class MyCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue();
        if (rxValue.length() > 0) {
            // Có thể xử lý lệnh đơn giản ở đây, ví dụ "stoplog", "startlog"
            // handleBLECommands(rxValue); // Hiện tại chưa làm gì
             char logBuffer[50];
             snprintf(logBuffer, sizeof(logBuffer), "BLE RX (ignored): %s", rxValue.c_str());
             debugLog(LOG_INFO, logBuffer);
        }
    }
};

// --- Handle Commands Received via BLE (Hiện tại không làm gì) ---
void handleBLECommands(const std::string& cmd_str) {

}


// --- Utility Functions (Giữ nguyên) ---
void debugLog(uint8_t level, const char* msg) {
#if DEBUG
    const char* prefix;
    switch (level) {
        case LOG_INFO: prefix = "[INFO] "; break;
        case LOG_ERROR: prefix = "[ERROR] "; break;
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