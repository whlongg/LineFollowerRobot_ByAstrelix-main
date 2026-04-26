#include "ColorTracker.h"

ColorTracker::ColorTracker(uint16_t readInterval)
    : readInterval(readInterval),
      lastReadTime(0),
      sensorReady(false),
      dataUpdated(false),
      detectedColor(COLOR_BLACK),
      highSpeedMode(false),
      m_isCalibrated(false)
{
    // Khởi tạo giá trị mặc định cho các ngưỡng
    setThresholds();
    
    // Thiết lập giá trị mặc định cho các thông số màu sắc
    colorParams.blueH = 240.0f;
    colorParams.blueS = 0.8f;
    colorParams.blueV = 0.5f;
    
    colorParams.greenH = 120.0f;
    colorParams.greenS = 0.8f;
    colorParams.greenV = 0.5f;
    
    colorParams.blueL = 50.0f;
    colorParams.blueA = 0.0f;
    colorParams.blueB = -50.0f;
    
    colorParams.greenL = 60.0f;
    colorParams.greenA = -50.0f;
    colorParams.greenB = 30.0f;
    
    colorParams.hueTolerance = 20.0f;
    colorParams.satTolerance = 0.2f;
    colorParams.valTolerance = 0.2f;
    colorParams.labTolerance = 25.0f;
    
    // Thử đọc cấu hình hiệu chuẩn từ EEPROM
    loadCalibrationFromEEPROM();
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

    // Cache giá trị để tránh tính toán nhiều lần
    float inv_c = 0;
    if (c > 0) inv_c = 1.0f / c;
    
    // Tính giá trị chuẩn hóa
    float r_norm = r * inv_c;
    float g_norm = g * inv_c;
    float b_norm = b * inv_c;
    
    // Chuyển đổi sang không gian HSV và Lab để phân loại màu chính xác hơn
    float h, s, v;
    rgbToHsv(r_norm, g_norm, b_norm, &h, &s, &v);
    
    float lab_l, lab_a, lab_b;
    rgbToLab(r_norm, g_norm, b_norm, &lab_l, &lab_a, &lab_b);
    
    // Tính điểm số cho mỗi màu dựa trên khoảng cách trong không gian HSV và Lab
    float blue_hsv_score = 0;
    float green_hsv_score = 0;
    
    // Tính điểm HSV - khoảng cách màu dựa trên H, S, V
    if (s > 0.2 && v > 0.1) {  // Bỏ qua màu quá tối hoặc không bão hòa
        // Tính điểm cho màu xanh dương dựa trên HSV
        float h_dist_blue = std::min(std::abs(h - colorParams.blueH), 
                                    std::abs(h - colorParams.blueH + 360.0f));
        h_dist_blue = std::min(h_dist_blue, std::abs(h - colorParams.blueH - 360.0f));
        
        float s_dist_blue = std::abs(s - colorParams.blueS);
        float v_dist_blue = std::abs(v - colorParams.blueV);
        
        blue_hsv_score = (h_dist_blue <= colorParams.hueTolerance && 
                          s_dist_blue <= colorParams.satTolerance && 
                          v_dist_blue <= colorParams.valTolerance) ? 1.0f : 0.0f;
        
        // Tính điểm cho màu xanh lá dựa trên HSV
        float h_dist_green = std::min(std::abs(h - colorParams.greenH), 
                                     std::abs(h - colorParams.greenH + 360.0f));
        h_dist_green = std::min(h_dist_green, std::abs(h - colorParams.greenH - 360.0f));
        
        float s_dist_green = std::abs(s - colorParams.greenS);
        float v_dist_green = std::abs(v - colorParams.greenV);
        
        green_hsv_score = (h_dist_green <= colorParams.hueTolerance && 
                           s_dist_green <= colorParams.satTolerance && 
                           v_dist_green <= colorParams.valTolerance) ? 1.0f : 0.0f;
    }
    
    // Tính điểm Lab - khoảng cách màu trong không gian Lab
    float blue_lab_distance = colorDistance(lab_l, lab_a, lab_b,
                                          colorParams.blueL, colorParams.blueA, colorParams.blueB);
    float green_lab_distance = colorDistance(lab_l, lab_a, lab_b, 
                                           colorParams.greenL, colorParams.greenA, colorParams.greenB);
    
    float blue_lab_score = (blue_lab_distance <= colorParams.labTolerance) ? 
                          (1.0f - blue_lab_distance / colorParams.labTolerance) : 0.0f;
    float green_lab_score = (green_lab_distance <= colorParams.labTolerance) ? 
                           (1.0f - green_lab_distance / colorParams.labTolerance) : 0.0f;
    
    // Kết hợp điểm số từ HSV và Lab (trọng số bằng nhau)
    float blue_score = (blue_hsv_score + blue_lab_score) / 2.0f;
    float green_score = (green_hsv_score + green_lab_score) / 2.0f;
    
    // Kiểm tra khả năng phát hiện ở không gian RGB thông thường như trước đó
    bool greenDominant = (g_norm > r_norm * greenRatioThreshold) && (g_norm > b_norm * greenRatioThreshold);
    bool greenIntensity = (g > 300) && (g > r * 1.1) && (g > b * 1.1);
    bool greenliness = greenDominant || greenIntensity;
    
    bool blueDominant = (b_norm > r_norm * blueRatioThreshold) && (b_norm > g_norm * blueRatioThreshold);
    bool blueIntensity = (b > 300) && (b > r * 1.1) && (b > g * 1.1);
    bool blueliness = blueDominant || blueIntensity;
    
    // Tăng cường điểm số từ phương pháp truyền thống
    if (greenliness) green_score += 0.5f;
    if (blueliness) blue_score += 0.5f;
    
    // Phân loại dựa trên điểm số cuối cùng
    if (blue_score > 0.6f && blue_score > green_score) {
        detectedColor = COLOR_BLUE;
        highSpeedMode = false;
        return;
    }
    
    if (green_score > 0.6f && green_score > blue_score) {
        detectedColor = COLOR_GREEN;
        highSpeedMode = true;
        return;
    }
    
    // Mặc định: màu đen hoặc trắng
    if (c >= minWhiteThreshold) {
        detectedColor = COLOR_WHITE;
    } else {
        detectedColor = COLOR_BLACK;
    }
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

// Chuyển từ RGB (0.0-1.0) sang HSV
void ColorTracker::rgbToHsv(float r, float g, float b, float *h, float *s, float *v)
{
    float max_val = std::max(std::max(r, g), b);
    float min_val = std::min(std::min(r, g), b);
    float delta = max_val - min_val;
    
    *v = max_val;
    
    if (max_val == 0.0f || delta == 0.0f) {
        *s = 0.0f;
        *h = 0.0f;  // Không xác định, nhưng trả về 0
        return;
    }
    
    *s = delta / max_val;
    
    if (r == max_val) {
        *h = (g - b) / delta;  // Giữa vàng và đỏ tím
    } else if (g == max_val) {
        *h = 2.0f + (b - r) / delta;  // Giữa lục lam và vàng
    } else {
        *h = 4.0f + (r - g) / delta;  // Giữa đỏ tím và lục lam
    }
    
    *h *= 60.0f;  // Chuyển sang độ
    if (*h < 0.0f) *h += 360.0f;
}

// Chuyển từ RGB sang XYZ (không gian màu trung gian)
void ColorTracker::rgbToXyz(float r, float g, float b, float *x, float *y, float *z)
{
    // Giả sử đầu vào RGB là tuyến tính (đã sRGB gamma-corrected)
    // Trong thực tế, có thể cần chuyển từ sRGB sang RGB tuyến tính
    float r_linear = (r > 0.04045f) ? pow((r + 0.055f) / 1.055f, 2.4f) : r / 12.92f;
    float g_linear = (g > 0.04045f) ? pow((g + 0.055f) / 1.055f, 2.4f) : g / 12.92f;
    float b_linear = (b > 0.04045f) ? pow((b + 0.055f) / 1.055f, 2.4f) : b / 12.92f;
    
    // Chuyển đổi từ RGB sang XYZ sử dụng ma trận chuyển đổi sRGB D65
    *x = r_linear * 0.4124f + g_linear * 0.3576f + b_linear * 0.1805f;
    *y = r_linear * 0.2126f + g_linear * 0.7152f + b_linear * 0.0722f;
    *z = r_linear * 0.0193f + g_linear * 0.1192f + b_linear * 0.9505f;
}

// Chuyển từ XYZ sang Lab
void ColorTracker::xyzToLab(float x, float y, float z, float *l, float *a, float *b)
{
    // Điểm trắng tham chiếu D65
    const float xn = 0.95047f;
    const float yn = 1.0f;
    const float zn = 1.08883f;
    
    float x_rel = x / xn;
    float y_rel = y / yn;
    float z_rel = z / zn;
    
    x_rel = (x_rel > 0.008856f) ? pow(x_rel, 1.0f/3.0f) : (7.787f * x_rel + 16.0f/116.0f);
    y_rel = (y_rel > 0.008856f) ? pow(y_rel, 1.0f/3.0f) : (7.787f * y_rel + 16.0f/116.0f);
    z_rel = (z_rel > 0.008856f) ? pow(z_rel, 1.0f/3.0f) : (7.787f * z_rel + 16.0f/116.0f);
    
    *l = (116.0f * y_rel) - 16.0f;
    *a = 500.0f * (x_rel - y_rel);
    *b = 200.0f * (y_rel - z_rel);
}

// Hàm trung gian để chuyển từ RGB sang Lab
void ColorTracker::rgbToLab(float r, float g, float b, float *l, float *a, float *b_out)
{
    float x, y, z;
    rgbToXyz(r, g, b, &x, &y, &z);
    xyzToLab(x, y, z, l, a, b_out);
}

// Tính khoảng cách Euclidean giữa hai màu trong không gian Lab
float ColorTracker::colorDistance(float l1, float a1, float b1, float l2, float a2, float b2)
{
    return sqrt(pow(l1 - l2, 2) + pow(a1 - a2, 2) + pow(b1 - b2, 2));
}

// Lưu cấu hình hiệu chuẩn vào EEPROM
void ColorTracker::saveCalibrationToEEPROM()
{
    int addr = EEPROM_CALIBRATION_ADDR;
    
    // Đánh dấu rằng dữ liệu đã được hiệu chuẩn
    EEPROM.write(addr++, EEPROM_CALIBRATION_FLAG);
    
    // Lưu thông số HSV của màu xanh dương
    EEPROM.put(addr, colorParams.blueH);
    addr += sizeof(float);
    EEPROM.put(addr, colorParams.blueS);
    addr += sizeof(float);
    EEPROM.put(addr, colorParams.blueV);
    addr += sizeof(float);
    
    // Lưu thông số HSV của màu xanh lá
    EEPROM.put(addr, colorParams.greenH);
    addr += sizeof(float);
    EEPROM.put(addr, colorParams.greenS);
    addr += sizeof(float);
    EEPROM.put(addr, colorParams.greenV);
    addr += sizeof(float);
    
    // Lưu thông số Lab của màu xanh dương
    EEPROM.put(addr, colorParams.blueL);
    addr += sizeof(float);
    EEPROM.put(addr, colorParams.blueA);
    addr += sizeof(float);
    EEPROM.put(addr, colorParams.blueB);
    addr += sizeof(float);
    
    // Lưu thông số Lab của màu xanh lá
    EEPROM.put(addr, colorParams.greenL);
    addr += sizeof(float);
    EEPROM.put(addr, colorParams.greenA);
    addr += sizeof(float);
    EEPROM.put(addr, colorParams.greenB);
    addr += sizeof(float);
    
    // Lưu dung sai
    EEPROM.put(addr, colorParams.hueTolerance);
    addr += sizeof(float);
    EEPROM.put(addr, colorParams.satTolerance);
    addr += sizeof(float);
    EEPROM.put(addr, colorParams.valTolerance);
    addr += sizeof(float);
    EEPROM.put(addr, colorParams.labTolerance);
    
    m_isCalibrated = true;
}

// Đọc cấu hình hiệu chuẩn từ EEPROM
bool ColorTracker::loadCalibrationFromEEPROM()
{
    int addr = EEPROM_CALIBRATION_ADDR;
    
    // Kiểm tra đánh dấu hiệu chuẩn
    if (EEPROM.read(addr++) != EEPROM_CALIBRATION_FLAG) {
        m_isCalibrated = false;
        return false;
    }
    
    // Đọc thông số HSV của màu xanh dương
    EEPROM.get(addr, colorParams.blueH);
    addr += sizeof(float);
    EEPROM.get(addr, colorParams.blueS);
    addr += sizeof(float);
    EEPROM.get(addr, colorParams.blueV);
    addr += sizeof(float);
    
    // Đọc thông số HSV của màu xanh lá
    EEPROM.get(addr, colorParams.greenH);
    addr += sizeof(float);
    EEPROM.get(addr, colorParams.greenS);
    addr += sizeof(float);
    EEPROM.get(addr, colorParams.greenV);
    addr += sizeof(float);
    
    // Đọc thông số Lab của màu xanh dương
    EEPROM.get(addr, colorParams.blueL);
    addr += sizeof(float);
    EEPROM.get(addr, colorParams.blueA);
    addr += sizeof(float);
    EEPROM.get(addr, colorParams.blueB);
    addr += sizeof(float);
    
    // Đọc thông số Lab của màu xanh lá
    EEPROM.get(addr, colorParams.greenL);
    addr += sizeof(float);
    EEPROM.get(addr, colorParams.greenA);
    addr += sizeof(float);
    EEPROM.get(addr, colorParams.greenB);
    addr += sizeof(float);
    
    // Đọc dung sai
    EEPROM.get(addr, colorParams.hueTolerance);
    addr += sizeof(float);
    EEPROM.get(addr, colorParams.satTolerance);
    addr += sizeof(float);
    EEPROM.get(addr, colorParams.valTolerance);
    addr += sizeof(float);
    EEPROM.get(addr, colorParams.labTolerance);
    
    m_isCalibrated = true;
    return true;
}

// Hàm hiệu chuẩn màu xanh dương
void ColorTracker::calibrateBlueColor()
{
    // Đọc giá trị RGB hiện tại từ cảm biến
    readRawData();
    
    // Chuyển đổi sang dạng chuẩn hóa
    float inv_c = 0;
    if (c > 0) inv_c = 1.0f / c;
    
    float r_norm = r * inv_c;
    float g_norm = g * inv_c;
    float b_norm = b * inv_c;
    
    // Lưu thông số HSV
    rgbToHsv(r_norm, g_norm, b_norm, &colorParams.blueH, &colorParams.blueS, &colorParams.blueV);
    
    // Lưu thông số Lab
    rgbToLab(r_norm, g_norm, b_norm, &colorParams.blueL, &colorParams.blueA, &colorParams.blueB);
}

// Hàm hiệu chuẩn màu xanh lá
void ColorTracker::calibrateGreenColor()
{
    // Đọc giá trị RGB hiện tại từ cảm biến
    readRawData();
    
    // Chuyển đổi sang dạng chuẩn hóa
    float inv_c = 0;
    if (c > 0) inv_c = 1.0f / c;
    
    float r_norm = r * inv_c;
    float g_norm = g * inv_c;
    float b_norm = b * inv_c;
    
    // Lưu thông số HSV
    rgbToHsv(r_norm, g_norm, b_norm, &colorParams.greenH, &colorParams.greenS, &colorParams.greenV);
    
    // Lưu thông số Lab
    rgbToLab(r_norm, g_norm, b_norm, &colorParams.greenL, &colorParams.greenA, &colorParams.greenB);
}
