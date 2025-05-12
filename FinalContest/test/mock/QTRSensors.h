#ifndef QTR_SENSORS_H
#define QTR_SENSORS_H

#include <Arduino.h>

class QTRSensors {
public:
    void setTypeAnalog() {}
    void setSensorPins(const uint8_t* pins, uint8_t sensorCount) {
        for(uint8_t i = 0; i < sensorCount; i++) {
            sensorPins[i] = pins[i];
        }
        numSensors = sensorCount;
    }
    
    void read(uint16_t* sensorValues) {
        for(uint8_t i = 0; i < numSensors; i++) {
            sensorValues[i] = analogRead(sensorPins[i]);
        }
    }
    
    void calibrate() {}
    
    uint16_t readLineBlack(uint16_t* sensorValues) {
        read(sensorValues);
        
        // Calculate weighted average
        uint32_t avg = 0;
        uint32_t sum = 0;
        for(uint8_t i = 0; i < numSensors; i++) {
            avg += (uint32_t)sensorValues[i] * (i * 1000);
            sum += sensorValues[i];
        }
        
        if(sum == 0) {
            return 1500; // Center position
        }
        
        return (avg / sum);
    }

private:
    uint8_t sensorPins[16];
    uint8_t numSensors;
};

#endif
