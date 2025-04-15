// --- PIN DEFINITIONS ---
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
const int PWM_FREQ = 20000;
const int PWM_RES = 8;

#define SCALE_RIGHT_MOTOR 1.05

void setup(){
    Serial.begin(115200);
    setupMotors();
}

void loop(){
    setMotorSpeeds(180,180);
}

// --- Motor Setup ---
void setupMotors() {
    ledcAttachChannel(PWM_PIN_L_A, PWM_FREQ, PWM_RES, left_motor_channel_a);
    ledcAttachChannel(PWM_PIN_L_B, PWM_FREQ, PWM_RES, left_motor_channel_b);
    ledcAttachChannel(PWM_PIN_R_A, PWM_FREQ, PWM_RES, right_motor_channel_a);
    ledcAttachChannel(PWM_PIN_R_B, PWM_FREQ, PWM_RES, right_motor_channel_b);
  }

// --- Motor Speed Control ---
void setMotorSpeeds(int left, int right) {
    right = right * SCALE_RIGHT_MOTOR; // Apply scaling factor to right motor
    ledcWriteChannel(left_motor_channel_a, left);
    ledcWriteChannel(left_motor_channel_b, 0);
    ledcWriteChannel(right_motor_channel_a, right);
    ledcWriteChannel(right_motor_channel_b, 0);
}