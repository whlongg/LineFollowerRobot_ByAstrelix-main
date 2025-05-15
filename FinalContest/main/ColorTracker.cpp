#include "ColorTracker.h"

ColorTracker::ColorTracker(uint16_t readInterval)
    : readInterval(readInterval),
      lastReadTime(0),
      sensorReady(false),
      dataUpdated(false),
      detectedColor(COLOR_BLACK),
      highSpeedMode(false)
{
    // Khởi tạo giá trị mặc định cho các ngưỡng
    setThresholds();
}

bool ColorTracker::begin()
{
    // Thử khởi tạo với tham số tích hợp thấp hơn và gain cao hơn
    // TCS34725_INTEGRATIONTIME_24MS và GAIN_16X để phản ứng nhanh hơn
    sensorReady = tcs.begin();
    
    if (sensorReady)
    {
        // Đọc lần đầu 
        readRawData();
        classifyColor();
        dataUpdated = true;
        timestamp = millis();
    }
    
    return sensorReady;
}

bool ColorTracker::update()
{
    if (!sensorReady) 
        return false;

    unsigned long currentTime = millis();

    // Đảm bảo khoảng cách giữa các lần đọc không nhỏ hơn 10ms
    // để tránh đọc quá nhanh vượt khả năng cảm biến
    if (currentTime - lastReadTime >= readInterval)
    {
        // Đọc giá trị trực tiếp từ thanh ghi không gây chặn
        readRawData();
        
        // Phân loại màu
        classifyColor();
        
        lastReadTime = currentTime;
        timestamp = currentTime;
        dataUpdated = true;
        
        return true;
    }

    return false;
}

void ColorTracker::readRawData()
{
    // Đọc giá trị trực tiếp từ thanh ghi không gây chặn
    c = tcs.read16(TCS34725_CDATAL);
    r = tcs.read16(TCS34725_RDATAL);
    g = tcs.read16(TCS34725_GDATAL);
    b = tcs.read16(TCS34725_BDATAL);
}

void ColorTracker::classifyColor()
{
    // Xử lý nhanh hơn bằng cách kiểm tra trường hợp đơn giản trước
    
    // Kiểm tra nhanh cho màu đen (độ sáng thấp)
    if (c <= maxBlackThreshold)
    {
        detectedColor = COLOR_BLACK;
        highSpeedMode = false;
        return;
    }

    // Kiểm tra nhanh cho màu trắng (độ sáng cao)
    // if (c >= minWhiteThreshold)
    // {
    //     detectedColor = COLOR_WHITE;
    //     highSpeedMode = false;
    //     return;
    // }

    // Cache giá trị để tránh tính toán nhiều lần
    float inv_c = 0;
    if (c > 0) inv_c = 1.0f / c;
    
    // Tính giá trị chuẩn hóa
    float r_norm = r * inv_c;
    float g_norm = g * inv_c;
    float b_norm = b * inv_c;

    // Sử dụng cả tỷ lệ và giá trị tuyệt đối để xác định màu xanh lá và xanh dương
    // Màu xanh lá: G cao hơn R và B
    bool greenDominant = (g_norm > r_norm * greenRatioThreshold) && (g_norm > b_norm * greenRatioThreshold);
    bool greenIntensity = (g > 500) && (g > r * 1.2) && (g > b * 1.2);
    
    if (greenDominant || greenIntensity)
    {
        detectedColor = COLOR_GREEN;
        highSpeedMode = true;
        return;
    }

    // Màu xanh dương: B cao hơn R và G
    bool blueDominant = (b_norm > r_norm * blueRatioThreshold) && (b_norm > g_norm * blueRatioThreshold);
    bool blueIntensity = (b > 500) && (b > r * 1.2) && (b > g * 1.2);
    
    if (blueDominant || blueIntensity)
    {
        detectedColor = COLOR_BLUE;
        highSpeedMode = false;
        return;
    }

    // Mặc định: màu đen
    detectedColor = COLOR_BLACK;
    highSpeedMode = false;
}

uint8_t ColorTracker::getColor()
{
    return detectedColor;
}

bool ColorTracker::isHighSpeedMode()
{
    return highSpeedMode;
}

bool ColorTracker::getResult(uint8_t *color, bool *highSpeedMode, unsigned long *timestampValue)
{
    if (!dataUpdated)
        return false;

    *color = detectedColor;
    *highSpeedMode = this->highSpeedMode;
    *timestampValue = timestamp;

    // Đánh dấu rằng dữ liệu đã được đọc
    dataUpdated = false;

    return true;
}

void ColorTracker::setThresholds(uint16_t minWhite, uint16_t maxBlack,
                                 float blueThreshold, float greenThreshold)
{
    minWhiteThreshold = minWhite;
    maxBlackThreshold = maxBlack;
    blueRatioThreshold = blueThreshold;
    greenRatioThreshold = greenThreshold;
}

void ColorTracker::getRawValues(uint16_t *r_val, uint16_t *g_val, uint16_t *b_val, uint16_t *c_val)
{
    *r_val = r;
    *g_val = g;
    *b_val = b;
    *c_val = c;
}

void ColorTracker::getNormalizedValues(uint8_t *r_val, uint8_t *g_val, uint8_t *b_val)
{
    // Chuẩn hóa giá trị về thang 0-255
    // Tránh chia cho 0
    if (c > 0)
    {
        float sum = r + g + b;
        *r_val = (uint8_t)(255.0f * r / sum);
        *g_val = (uint8_t)(255.0f * g / sum);
        *b_val = (uint8_t)(255.0f * b / sum);
    }
    else
    {
        *r_val = *g_val = *b_val = 0;
    }
}
