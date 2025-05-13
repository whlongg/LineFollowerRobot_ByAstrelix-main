#include "ColorTracker.h"

ColorTracker::ColorTracker(uint16_t readInterval, 
                         tcs34725IntegrationTime_t integrationTime,
                         tcs34725Gain_t gain)
    : tcs(integrationTime, gain),
      readInterval(readInterval),
      lastReadTime(0),
      sensorReady(false),
      dataUpdated(false),
      detectedColor(COLOR_BLACK),
      highSpeedMode(false) {
          
    // Khởi tạo giá trị mặc định cho các ngưỡng
    setThresholds();
}

bool ColorTracker::begin() {
    sensorReady = tcs.begin();
    
    if (sensorReady) {
        // Khởi tạo thành công, trì hoãn đọc đầu tiên
        delay(10);
        
        // Đọc lần đầu
        readRawData();
        classifyColor();
        dataUpdated = true;
        timestamp = millis();
    }
    
    return sensorReady;
}

bool ColorTracker::update() {
    if (!sensorReady) return false;
    
    unsigned long currentTime = millis();
    
    if (currentTime - lastReadTime >= readInterval) {
        // Đã đến thời điểm đọc mới
        readRawData();
        classifyColor();
        
        lastReadTime = currentTime;
        timestamp = currentTime;
        dataUpdated = true;
        
        return true;
    }
    
    return false;
}

void ColorTracker::readRawData() {
    // Đọc giá trị trực tiếp từ thanh ghi không gây chặn
    c = tcs.read16(TCS34725_CDATAL);
    r = tcs.read16(TCS34725_RDATAL);
    g = tcs.read16(TCS34725_GDATAL);
    b = tcs.read16(TCS34725_BDATAL);
}

void ColorTracker::classifyColor() {
    // Xử lý logic phân loại màu
    
    // Kiểm tra màu đen (độ sáng thấp)
    if (c <= maxBlackThreshold) {
        detectedColor = COLOR_BLACK;
        highSpeedMode = false;
        return;
    }
    
    // Kiểm tra màu trắng (độ sáng cao)
    if (c >= minWhiteThreshold) {
        detectedColor = COLOR_WHITE;
        highSpeedMode = false;
        return;
    }
    
    // Chuẩn hóa giá trị để so sánh tương quan giữa các kênh màu
    float r_norm = (float)r / c;
    float g_norm = (float)g / c;
    float b_norm = (float)b / c;
    
    // Màu xanh lá: kênh G cao hơn đáng kể so với R và B
    if ((g_norm > r_norm * greenRatioThreshold) && (g_norm > b_norm * greenRatioThreshold)) {
        detectedColor = COLOR_GREEN;
        highSpeedMode = true;
        return;
    }
    
    // Màu xanh dương: kênh B cao hơn đáng kể so với R và G
    if ((b_norm > r_norm * blueRatioThreshold) && (b_norm > g_norm * blueRatioThreshold)) {
        detectedColor = COLOR_BLUE;
        highSpeedMode = false;
        return;
    }
    
    // Mặc định: coi như đen nếu không khớp với màu nào
    detectedColor = COLOR_BLACK;
    highSpeedMode = false;
}

uint8_t ColorTracker::getColor() {
    return detectedColor;
}

bool ColorTracker::isHighSpeedMode() {
    return highSpeedMode;
}

bool ColorTracker::getResult(uint8_t *color, bool *highSpeedMode, unsigned long *timestampValue) {
    if (!dataUpdated) return false;
    
    *color = detectedColor;
    *highSpeedMode = this->highSpeedMode;
    *timestampValue = timestamp;
    
    // Đánh dấu rằng dữ liệu đã được đọc
    dataUpdated = false;
    
    return true;
}

void ColorTracker::setThresholds(uint16_t minWhite, uint16_t maxBlack, 
                               float blueThreshold, float greenThreshold) {
    minWhiteThreshold = minWhite;
    maxBlackThreshold = maxBlack;
    blueRatioThreshold = blueThreshold;
    greenRatioThreshold = greenThreshold;
}

void ColorTracker::getRawValues(uint16_t *r_val, uint16_t *g_val, uint16_t *b_val, uint16_t *c_val) {
    *r_val = r;
    *g_val = g;
    *b_val = b;
    *c_val = c;
}

void ColorTracker::getNormalizedValues(uint8_t *r_val, uint8_t *g_val, uint8_t *b_val) {
    // Chuẩn hóa giá trị về thang 0-255
    // Tránh chia cho 0
    if (c > 0) {
        float sum = r + g + b;
        *r_val = (uint8_t)(255.0f * r / sum);
        *g_val = (uint8_t)(255.0f * g / sum);
        *b_val = (uint8_t)(255.0f * b / sum);
    } else {
        *r_val = *g_val = *b_val = 0;
    }
}
