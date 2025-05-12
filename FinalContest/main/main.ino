#include <ReadIR.h>

ReadIR irSensor;

void setup()
{
  Serial.begin(115200);
  irSensor.begin();
}

void loop()
{
  // float sensorValues[4];
  // irSensor.readSensors(sensorValues);

  // // In giá trị cảm biến ra Serial Monitor
  // for (int i = 0; i < 4; i++)
  // {
  //   Serial.print(sensorValues[i]);
  //   Serial.print(" ");
  // }
  // Serial.println();

  Serial.println(irSensor.calculateError());
}

// #include "MotorControl.h"

// void setup() {
//   setupMotors();
// }

// void loop() {
//   setMotorSpeeds(150, 150);  // chạy thẳng
//   delay(1000);
//   setMotorSpeeds(-150, -150); // lùi
//   delay(1000);
// }
