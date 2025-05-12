#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <pidController.h>
#include <Motor.h>

// --- CONFIGURABLE CONSTANTS ---
#define DEBUG 0

#define RGB_CHECKING 0                  // 1: kiểm tra màu sắc
#define ENABLE_BYPASS_INTERSECTION 1    // Đặt 0 nếu tắt bypass
#define ENABLE_HIGH_SPEED_ON_STRAIGHT 1 // 0: Không kích hoạt HIGH_SPEED
#define THREADSOLD_BLACK 300

const float SCALE_RIGHT_MOTOR = 1.002;
const float KP = 0;
const float KI = 0;
const float KD = 0;
const float INTEGRAL_MAX = 70.0f;

int MAX_SPEED = 250;
int BASE_SPEED = 210;

// --- TIMING ---
const unsigned long LOOP_INTERVAL = 2; // loop timing ms
const unsigned long BLE_LOG_INTERVAL_MS = 5; //BLE Log timing ms

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

// --- RGB Sensor Setup ---
Adafruit_TCS34725 rgbSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// --- PIN DEFINITIONS ---
const int IR_PINS[SENSOR_COUNT] = {4, 3, 1, 0};
const int W_LED_ON = 20;
const int IR_LED_ON = 21;
const int LED_CHECK = 8;
const int BUTTON_PIN = 7;

