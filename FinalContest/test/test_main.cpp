#include <Arduino.h>
#include <unity.h>
#include "../main/MotorControl.h"
#include "../main/ReadIR.h"
#include "../main/utils.h"

// Define IR sensor pins for testing
const uint8_t TEST_IR_PINS[4] = {4, 3, 1, 0};

MotorControl motorControl;
ReadIR irSensor;
Utils utils;

// Mock functions for hardware
// Mock QTR sensor readings
void mockQTRSensorReadings(int values[4]) {
    // Set mock analog readings for IR sensors
    for(int i = 0; i < 4; i++) {
        analogWrite(TEST_IR_PINS[i], values[i]);
    }
}

void mockSetup() {
    // Mock Arduino pins
    for(int i = 0; i < 50; i++) {
        pinMode(i, OUTPUT);
    }
    
    // Initialize mock sensor values
    int initialValues[] = {500, 500, 500, 500};
    mockQTRSensorReadings(initialValues);
}

// Test cases for MotorControl
void test_motor_forward() {
    motorControl.Forward(100);
    // Test that correct PWM values are set
    TEST_ASSERT_EQUAL(100, analogRead(PWM_PIN_L_A));
    TEST_ASSERT_EQUAL(0, analogRead(PWM_PIN_L_B));
    TEST_ASSERT_EQUAL(100 * SCALE_RIGHT_MOTOR, analogRead(PWM_PIN_R_A));
    TEST_ASSERT_EQUAL(0, analogRead(PWM_PIN_R_B));
}

void test_motor_backward() {
    motorControl.Back(100);
    // Test that correct PWM values are set
    TEST_ASSERT_EQUAL(0, analogRead(PWM_PIN_L_A));
    TEST_ASSERT_EQUAL(100, analogRead(PWM_PIN_L_B));
    TEST_ASSERT_EQUAL(0, analogRead(PWM_PIN_R_A));
    TEST_ASSERT_EQUAL(100 * SCALE_RIGHT_MOTOR, analogRead(PWM_PIN_R_B));
}

void test_motor_turn_right() {
    motorControl.turnRight(100);
    // Test that only left motor runs
    TEST_ASSERT_EQUAL(100, analogRead(PWM_PIN_L_A));
    TEST_ASSERT_EQUAL(0, analogRead(PWM_PIN_L_B));
    TEST_ASSERT_EQUAL(0, analogRead(PWM_PIN_R_A));
    TEST_ASSERT_EQUAL(0, analogRead(PWM_PIN_R_B));
}

void test_motor_turn_left() {
    motorControl.turnLeft(100);
    // Test that only right motor runs
    TEST_ASSERT_EQUAL(0, analogRead(PWM_PIN_L_A));
    TEST_ASSERT_EQUAL(0, analogRead(PWM_PIN_L_B));
    TEST_ASSERT_EQUAL(100 * SCALE_RIGHT_MOTOR, analogRead(PWM_PIN_R_A));
    TEST_ASSERT_EQUAL(0, analogRead(PWM_PIN_R_B));
}

void test_motor_stop() {
    motorControl.stop();
    // Test that all motors are stopped
    TEST_ASSERT_EQUAL(0, analogRead(PWM_PIN_L_A));
    TEST_ASSERT_EQUAL(0, analogRead(PWM_PIN_L_B));
    TEST_ASSERT_EQUAL(0, analogRead(PWM_PIN_R_A));
    TEST_ASSERT_EQUAL(0, analogRead(PWM_PIN_R_B));
}

// Test cases for ReadIR
void test_ir_sensor_initialization() {
    irSensor.begin();
    // Test that LED pins are initialized correctly
    TEST_ASSERT_EQUAL(HIGH, digitalRead(20)); // W_LED_ON
    TEST_ASSERT_EQUAL(HIGH, digitalRead(21)); // IR_LED_ON
}

void test_ir_sensor_read() {
    float sensorValues[4];
    
    // Test case 1: All sensors reading mid-range
    int midValues[] = {500, 500, 500, 500};
    mockQTRSensorReadings(midValues);
    irSensor.readSensors(sensorValues);
    for(int i = 0; i < 4; i++) {
        TEST_ASSERT_FLOAT_WITHIN(1.0, 500.0, sensorValues[i]);
    }
    
    // Test case 2: Line on the left
    int leftValues[] = {900, 700, 300, 100};
    mockQTRSensorReadings(leftValues);
    irSensor.readSensors(sensorValues);
    TEST_ASSERT_FLOAT_WITHIN(1.0, 900.0, sensorValues[0]);
    TEST_ASSERT_FLOAT_WITHIN(1.0, 100.0, sensorValues[3]);
    
    // Test case 3: Line on the right
    int rightValues[] = {100, 300, 700, 900};
    mockQTRSensorReadings(rightValues);
    irSensor.readSensors(sensorValues);
    TEST_ASSERT_FLOAT_WITHIN(1.0, 100.0, sensorValues[0]);
    TEST_ASSERT_FLOAT_WITHIN(1.0, 900.0, sensorValues[3]);
}

void test_calculate_error() {
    // Test case 1: Line in the middle (should give near-zero error)
    int middleValues[] = {500, 900, 900, 500};
    mockQTRSensorReadings(middleValues);
    float error = irSensor.calculateError();
    TEST_ASSERT_FLOAT_WITHIN(10.0, 0.0, error);
    
    // Test case 2: Line on the far left (should give negative error)
    int farLeftValues[] = {900, 300, 100, 100};
    mockQTRSensorReadings(farLeftValues);
    error = irSensor.calculateError();
    TEST_ASSERT_TRUE(error < -50);
    
    // Test case 3: Line on the far right (should give positive error)
    int farRightValues[] = {100, 100, 300, 900};
    mockQTRSensorReadings(farRightValues);
    error = irSensor.calculateError();
    TEST_ASSERT_TRUE(error > 50);
}

// Test cases for Utils
void test_is_all_black() {
    // Mock sensor values to simulate all black condition
    // Test the isAllBlack() function
    bool result = utils.isAllBlack();
    TEST_ASSERT_TRUE(result);
}

void test_is_all_white() {
    // Mock sensor values to simulate all white condition
    // Test the isAllWhite() function
    bool result = utils.isAllWhite();
    TEST_ASSERT_TRUE(result);
}

void setup() {
    delay(2000);
    UNITY_BEGIN();
    
    mockSetup();
    
    // Run MotorControl tests
    RUN_TEST(test_motor_forward);
    RUN_TEST(test_motor_backward);
    RUN_TEST(test_motor_turn_right);
    RUN_TEST(test_motor_turn_left);
    RUN_TEST(test_motor_stop);
    
    // Run ReadIR tests
    RUN_TEST(test_ir_sensor_initialization);
    RUN_TEST(test_ir_sensor_read);
    RUN_TEST(test_calculate_error);
    
    // Run Utils tests
    RUN_TEST(test_is_all_black);
    RUN_TEST(test_is_all_white);
    
    UNITY_END();
}

void loop() {
    // Empty loop for Arduino framework
}
