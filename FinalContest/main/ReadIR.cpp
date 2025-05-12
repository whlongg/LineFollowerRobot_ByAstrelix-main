#include "ReadIR.h"
#include <Arduino.h>

ReadIR::ReadIR()
{
    // Constructor không cần khởi tạo gì thêm
}

void ReadIR::begin()
{
    // Cấu hình các chân LED
    pinMode(W_LED_ON, OUTPUT);
    pinMode(IR_LED_ON, OUTPUT);

    // Luôn bật WLED và IRLED
    digitalWrite(W_LED_ON, HIGH);
    digitalWrite(IR_LED_ON, HIGH);

    // Cấu hình QTRSensors
    qtr.setTypeAnalog();
    qtr.setSensorPins(IR_PINS, SENSOR_COUNT);

    // Khởi tạo mảng giá trị cảm biến
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        for (int j = 0; j < FILTER_SIZE; j++)
        {
            ir_values[i][j] = 0;
        }
    }
}

void ReadIR::readSensors(float *sensorValues)
{
    // Đọc giá trị analog từ cảm biến
    uint16_t rawValues[SENSOR_COUNT];
    qtr.read(rawValues);

    // Chuyển đổi sang float và lưu vào mảng
    for (int i = 0; i < SENSOR_COUNT; i++)
        ir_values[i][filter_index] = (float)rawValues[i];

    // Lọc giá trị cảm biến (sửa kiểu truyền tham số)
    filterSensorValues(ir_values, sensorValues);

    // Cập nhật chỉ số bộ lọc
    filter_index = (filter_index + 1) % FILTER_SIZE;
}


void ReadIR::filterSensorValues(float rawValues[][FILTER_SIZE], float *filteredValues)
{
    // Áp dụng bộ lọc trung bình trượt
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        float sum = 0;
        for (int j = 0; j < FILTER_SIZE; j++)
            sum += rawValues[i][j];
        filteredValues[i] = sum / FILTER_SIZE;
    }
}

float ReadIR::calculateError()
{
    float sensorValues[SENSOR_COUNT];
    readSensors(sensorValues);

    // Convert float values to uint16_t for QTR line position calculation
    uint16_t rawValues[SENSOR_COUNT];
    for (int i = 0; i < SENSOR_COUNT; i++) {
        rawValues[i] = (uint16_t)sensorValues[i];
    }

    // Get line position from QTRSensors (returns value from 0 to 3000 for 4 sensors)
    uint16_t position = qtr.readLineBlack(rawValues);
    
    // Convert position to normalized error (-1 to 1)
    // 1500 is the center position (3000/2)
    float error = (position - 1500.0f) / 1500.0f;
    
    return error;
}

