#ifndef READ_IR_H
#define READ_IR_H

#include <QTRSensors.h>

#define SENSOR_COUNT 4
#define FILTER_SIZE 5

class ReadIR
{
public:
    ReadIR();
    void begin();
    void readSensors(float *sensorValues);
    float calculateError();
    void CalibrateSensor();

private:
    QTRSensors qtr;
    const int duration = 5000; // 3 seconds
    const int SPEED_CALIBRATE = 100;
    const uint8_t IR_PINS[SENSOR_COUNT] = {4, 3, 1, 0};
    const int W_LED_ON = 20;
    const int IR_LED_ON = 21;
    float ir_values[SENSOR_COUNT][FILTER_SIZE] = {0};
    int filter_index = 0;

    void filterSensorValues(float rawValues[][FILTER_SIZE], float *filteredValues);
};

#endif