#include "MotorControl.h"

const int PWM_PIN_L_A = 2;
const int PWM_PIN_L_B = 10;
const int PWM_PIN_R_A = 6;
const int PWM_PIN_R_B = 5;
const int left_motor_channel_a = 0;
const int left_motor_channel_b = 1;
const int right_motor_channel_a = 2;
const int right_motor_channel_b = 3;

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

void MotorControl::turnRight(int speed)
{
  setMotorSpeeds(speed, 0);
}

void MotorControl::turnLeft(int speed)
{
  setMotorSpeeds(0, speed);
}

void MotorControl::Back(int speed)
{
  setMotorSpeeds(-speed, -speed);
}

void MotorControl::Forward(int speed)
{
  setMotorSpeeds(speed, speed);
}

void MotorControl::stop()
{
  setMotorSpeeds(0, 0);
}
