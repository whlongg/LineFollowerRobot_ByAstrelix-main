//#include <Wire.h>
//#include "Adafruit_TCS34725.h"

//Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

#define PWM_PIN_L_A 2
#define PWM_PIN_L_B 10
#define PWM_PIN_R_A 6
#define PWM_PIN_R_B 5

#define left_motor_channel_a 0
#define left_motor_channel_b 1
#define right_motor_channel_a 2
#define right_motor_channel_b 3

#define SENSOR_1_PIN 4
#define SENSOR_2_PIN 3
#define SENSOR_3_PIN 1
#define SENSOR_4_PIN 0

#define W_LED_ON 20
#define IR_LED_ON 21

#define threshold 2000

uint8_t vspeed = 100; //50    
uint8_t tspeed = 100; //70

uint8_t vspeed_l = vspeed;
uint8_t tspeed_l = tspeed;
uint8_t vspeed_r = vspeed * 1.1;
uint8_t tspeed_r = tspeed * 1.1;

void stop()
{
  ledcWriteChannel(left_motor_channel_a, 0);
  ledcWriteChannel(left_motor_channel_b, 0);
  ledcWriteChannel(right_motor_channel_a, 0);
  ledcWriteChannel(right_motor_channel_b, 0); 
  Serial.println("stop");
}

void forward()
{
  ledcWriteChannel(left_motor_channel_a, vspeed_l);
  ledcWriteChannel(left_motor_channel_b, 0);
  ledcWriteChannel(right_motor_channel_a, vspeed_r);
  ledcWriteChannel(right_motor_channel_b, 0);    
  Serial.println("forward");
} 

void backward()
{
  ledcWriteChannel(left_motor_channel_a, 0);
  ledcWriteChannel(left_motor_channel_b, vspeed_l);
  ledcWriteChannel(right_motor_channel_a, 0);
  ledcWriteChannel(right_motor_channel_b, vspeed_r);
  Serial.println("backward");
} 

void left()
{
  ledcWriteChannel(left_motor_channel_a, 0);
  ledcWriteChannel(left_motor_channel_b, tspeed_l);
  ledcWriteChannel(right_motor_channel_a, tspeed_r);
  ledcWriteChannel(right_motor_channel_b, 0);      
  Serial.println("left");
}

void right()
{
  ledcWriteChannel(left_motor_channel_a, tspeed_l);
  ledcWriteChannel(left_motor_channel_b, 0);
  ledcWriteChannel(right_motor_channel_a, 0);
  ledcWriteChannel(right_motor_channel_b, tspeed_r);  
  Serial.println("right");
}

void setup() {
  Serial.begin(115200);

  // attach the channel to the GPIO to be controlled

  // for esp32 arduino core 2.0.15
  // ledcSetup(0, 5000, 8);
  // ledcSetup(1, 5000, 8);
  // ledcSetup(2, 5000, 8);
  // ledcSetup(3, 5000, 8);
  // ledcAttachPin(PWM_PIN_L_A, left_motor_channel_a);
  // ledcAttachPin(PWM_PIN_L_B, left_motor_channel_b);
  // ledcAttachPin(PWM_PIN_R_A, right_motor_channel_a);
  // ledcAttachPin(PWM_PIN_R_B, right_motor_channel_b);

  // for esp32 arduino core 3.2.0
  ledcAttachChannel(PWM_PIN_L_A, 5000, 8, left_motor_channel_a);
  ledcAttachChannel(PWM_PIN_L_B, 5000, 8, left_motor_channel_b);
  ledcAttachChannel(PWM_PIN_R_A, 5000, 8, right_motor_channel_a);
  ledcAttachChannel(PWM_PIN_R_B, 5000, 8, right_motor_channel_b);

  pinMode(W_LED_ON,OUTPUT);
  pinMode(IR_LED_ON,OUTPUT);
  digitalWrite(W_LED_ON, 0);
  digitalWrite(IR_LED_ON, 1);

  // if (tcs.begin()) {
  //   Serial.println("Found sensor");
  // } else {
  //   Serial.println("No TCS34725 found ... check your connections");
  // }
}

void loop() {
  int sensor_1_state = analogRead(SENSOR_1_PIN);
  int sensor_2_state = analogRead(SENSOR_2_PIN);
  int sensor_3_state = analogRead(SENSOR_3_PIN);
  int sensor_4_state = analogRead(SENSOR_4_PIN);

  Serial.print("ir = \t");
  Serial.print(sensor_1_state);
  Serial.print("\t");
  Serial.print(sensor_2_state);
  Serial.print("\t");
  Serial.print(sensor_3_state);
  Serial.print("\t");
  Serial.println(sensor_4_state);

  // uint16_t r, g, b, c;
  // tcs.getRawData(&r, &g, &b, &c);

  // Serial.print("color = \t");
  // Serial.print(r);
  // Serial.print("\t");
  // Serial.print(g);
  // Serial.print("\t");
  // Serial.print(b);
  // Serial.print("\t");
  // Serial.println(c);

  uint8_t sensor_array = 0;
  
  sensor_array += (sensor_1_state <= threshold);
  sensor_array<<=1;
  sensor_array += (sensor_2_state <= threshold);
  sensor_array<<=1;
  sensor_array += (sensor_3_state <= threshold);
  sensor_array<<=1;
  sensor_array += (sensor_4_state <= threshold);

  switch(sensor_array) 
  {
    case 0b1000:
    case 0b1100:
      left();
      break;

    case 0b0001:
    case 0b0011:
      right();
      break;

    case 0b0110:
    case 0b0100:
    case 0b0010:
    case 0b1110:
    case 0b0111:
    case 0b1111:
      forward();
      break;

    // case 0b0000:
    //   backward();
    //   break;

    default:
      forward();
      break;
  }

  Serial.println(sensor_array, BIN);
  delay(10);
}