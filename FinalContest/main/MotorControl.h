#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

// Define các hằng số và channel ở đây (có thể tùy chỉnh theo project của bạn)
#define PWM_FREQ 20000
#define PWM_RES 8
#define SCALE_RIGHT_MOTOR 1.002

// Chân PWM và kênh PWM
extern const int PWM_PIN_L_A;
extern const int PWM_PIN_L_B;
extern const int PWM_PIN_R_A;
extern const int PWM_PIN_R_B;
extern const int left_motor_channel_a;
extern const int left_motor_channel_b;
extern const int right_motor_channel_a;
extern const int right_motor_channel_b;

// Hàm setup và control motor
void setupMotors();
void setMotorSpeeds(int left, int right);

class MotorControl
{
    public:
        void turnRight(int speed);
        void turnLeft(int speed);
        void Back(int speed);
        void Forward(int speed);
        void stop();
};

#endif
