#include "ReadIR.h"
#include <Arduino.h>
#include "MotorControl.h"
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
        for (int j = 0; j < FILTER_SIZE; j++)
            ir_values[i][j] = 0;
}

void ReadIR::readSensors(float *sensorValues)
{
    // Đọc giá trị đã chuẩn hóa từ cảm biến
    uint16_t calibratedValues[SENSOR_COUNT];
    qtr.readCalibrated(calibratedValues);

    // Chuyển đổi sang float và lưu vào mảng
    for (int i = 0; i < SENSOR_COUNT; i++)
        ir_values[i][filter_index] = (float)calibratedValues[i];

    // Lọc giá trị cảm biến
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
    // Định nghĩa vị trí vật lý của các cảm biến (mm)
    const float sensorPositions[SENSOR_COUNT] = {0.0, 7.5, 25.5, 33.0};
    // Tâm của dãy cảm biến
    const float center = 16.5;
    // Ngưỡng nhiễu cho giá trị đã chuẩn hóa (thường là 0-1000)
    // Giá trị này thấp hơn vì giá trị chuẩn hóa đã xử lý tốt hơn
    const float noiseThreshold = 100.0;
    // Hệ số chuẩn hóa để chuyển error từ khoảng [-16.5, 16.5] sang [-100, 100]
    const float normalizationFactor = 6.06; // = 100 / 16.5
    
    // Đọc giá trị từ các cảm biến
    float sensorValues[SENSOR_COUNT];
    readSensors(sensorValues);
    
    // Kiểm tra xem line có hiện diện trên ít nhất 1 cảm biến hay không
    bool lineDetected = false;
    for (int i = 0; i < SENSOR_COUNT; i++) {
        if (sensorValues[i] > noiseThreshold) {
            lineDetected = true;
            break;
        }
    }
    
    // Nếu không phát hiện line trên bất kỳ cảm biến nào
    if (!lineDetected) {
        // Kiểm tra xem line đã biến mất bên trái hay bên phải
        // (dựa vào error trước đó)
        static float lastError = 0;
        // Trả về cùng dấu với error trước nhưng với giá trị lớn hơn
        return lastError > 0 ? 100.0 : -100.0;
    }
    
    // Tính toán vị trí line sử dụng nội suy tuyến tính
    float numerator = 0;
    float denominator = 0;
    
    // Tối ưu: Tính trước offset để tránh tính lại trong vòng lặp
    float offsets[SENSOR_COUNT];
    for (int i = 0; i < SENSOR_COUNT; i++) {
        offsets[i] = sensorPositions[i] - center;
    }
    
    // Dùng thêm biến lastMax để tăng độ tin cậy khi chỉ một phần line nằm trên cảm biến
    int maxIndex = 0;
    float maxValue = 0;
    
    for (int i = 0; i < SENSOR_COUNT; i++) {
        if (sensorValues[i] > noiseThreshold) {
            // Áp dụng trọng số dựa trên vị trí cảm biến
            numerator += sensorValues[i] * offsets[i];
            denominator += sensorValues[i];
            
            // Theo dõi cảm biến có giá trị lớn nhất
            if (sensorValues[i] > maxValue) {
                maxValue = sensorValues[i];
                maxIndex = i;
            }
        }
    }
    
    // Nếu mẫu số quá nhỏ, có thể gây ra kết quả không ổn định
    if (denominator < noiseThreshold * 2) {
        // Đường line nằm ở rìa, sử dụng vị trí cảm biến có giá trị lớn nhất
        float error = offsets[maxIndex] * normalizationFactor;
        // Giới hạn trong khoảng [-100, 100]
        if (error > 100.0) error = 100.0;
        if (error < -100.0) error = -100.0;
        return error;
    }
    
    // Tính toán error và chuẩn hóa sang khoảng [-100, 100]
    float error = (numerator / denominator) * normalizationFactor;
    
    // Giới hạn trong khoảng [-100, 100]
    if (error > 100.0) error = 100.0;
    if (error < -100.0) error = -100.0;
    
    // Lưu lại error hiện tại để sử dụng trong trường hợp mất line
    static float lastError = error;
    lastError = error;
    
    return error;
}
// "Lệch trái" là đường đen nằm lệch sang phía bên tay trái của robot -> rẽ phải

void ReadIR::CalibrateSensor()
{
    unsigned long startTime = millis(), currentTime = startTime;
    float sensorValues[SENSOR_COUNT];
    uint16_t calibratedValues[SENSOR_COUNT];
    MotorControl motor;

    motor.turnRight(SPEED_CALIBRATE);
    while (currentTime - startTime < duration)
    {
        readSensors(sensorValues);
        qtr.calibrate();
        for (int i = 0; i < SENSOR_COUNT; i++) {
            calibratedValues[i] = (uint16_t)sensorValues[i];
        }
        currentTime = millis();
    }
    
    // Dừng động cơ sau khi hoàn tất hiệu chỉnh
    motor.stop();
}