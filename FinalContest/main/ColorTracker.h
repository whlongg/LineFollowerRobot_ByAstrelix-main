#ifndef COLOR_TRACKER_H
#define COLOR_TRACKER_H

#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <EEPROM.h>

// Định nghĩa các mã màu
#define COLOR_WHITE 0
#define COLOR_BLACK 1
#define COLOR_BLUE  2
#define COLOR_GREEN 3

class ColorTracker {
public:
    /**
     * @brief Khởi tạo ColorTracker với thời gian đọc cảm biến tùy chỉnh
     * @param readInterval Khoảng thời gian giữa các lần đọc cảm biến (ms)
     * @param integrationTime Thời gian tích hợp của cảm biến (từ 2.4 đến 614.4 ms)
     */
    ColorTracker(uint16_t readInterval = 25);
    
    /**
     * @brief Khởi tạo cảm biến
     * @return true nếu khởi tạo thành công, false nếu thất bại
     */
    bool begin();
    
    /**
     * @brief Cập nhật giá trị từ cảm biến nếu đã đến thời điểm
     * @return true nếu giá trị được cập nhật, false nếu chưa đến thời điểm
     */
    bool update();
    
    /**
     * @brief Lấy giá trị màu hiện tại
     * @return Mã màu (0: trắng, 1: đen, 2: xanh dương, 3: xanh lá)
     */
    uint8_t getColor();
    
    /**
     * @brief Kiểm tra xem có phải chế độ tốc độ cao
     * @return true nếu là chế độ tốc độ cao (xanh lá), false nếu không
     */
    bool isHighSpeedMode();
    
    /**
     * @brief Lấy kết quả hiện tại
     * @param color Con trỏ đến biến chứa mã màu
     * @param highSpeedMode Con trỏ đến biến chứa trạng thái tốc độ cao
     * @param timestamp Con trỏ đến biến chứa thời gian
     * @return true nếu giá trị là mới nhất, false nếu chưa cập nhật
     */
    bool getResult(uint8_t *color, bool *highSpeedMode, unsigned long *timestamp);
    
    /**
     * @brief Đặt ngưỡng cho việc phát hiện màu
     * @param minWhite Ngưỡng tối thiểu để xác định màu trắng (giá trị clear)
     * @param maxBlack Ngưỡng tối đa để xác định màu đen (giá trị clear)
     * @param blueThreshold Ngưỡng tỉ lệ để xác định màu xanh dương
     * @param greenThreshold Ngưỡng tỉ lệ để xác định màu xanh lá
     */
    void setThresholds(uint16_t minWhite = 4000, 
                       uint16_t maxBlack = 800,
                       float blueThreshold = 1.2,
                       float greenThreshold = 1.2);
    
    /**
     * @brief Lấy các giá trị RGB và Clear hiện tại
     */
    void getRawValues(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c);
    
    /**
     * @brief Lấy các giá trị RGB chuẩn hóa (0-255)
     */
    void getNormalizedValues(uint8_t *r, uint8_t *g, uint8_t *b);

private:
    //Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);  
    Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_16X);
    unsigned long lastReadTime;
    uint16_t readInterval;
    bool sensorReady;
    bool dataUpdated;
    
    uint16_t r, g, b, c;
    uint8_t detectedColor;
    bool highSpeedMode;
    unsigned long timestamp;
    
    // Ngưỡng phát hiện màu
    uint16_t minWhiteThreshold;
    uint16_t maxBlackThreshold;
    float blueRatioThreshold;
    float greenRatioThreshold;
    
    // Thông số màu sắc HSV và Lab
    struct ColorParams {
        // Thông số HSV
        float blueH, blueS, blueV;
        float greenH, greenS, greenV;
        
        // Thông số Lab
        float blueL, blueA, blueB;
        float greenL, greenA, greenB;
        
        // Dung sai
        float hueTolerance;
        float satTolerance;
        float valTolerance;
        float labTolerance;
    };
    
    ColorParams colorParams;
    bool m_isCalibrated;
    
    // Địa chỉ EEPROM để lưu thông số
    const int EEPROM_CALIBRATION_ADDR = 0;
    const int EEPROM_CALIBRATION_FLAG = 100;  // Giá trị kiểm tra
    
    /**
     * @brief Đọc giá trị RGB và Clear từ cảm biến không chặn
     */
    void readRawData();
    
    /**
     * @brief Phân loại màu từ giá trị RGB hiện tại
     */
    void classifyColor();
    
    /**
     * @brief Chuyển từ không gian RGB sang HSV
     * @param r Giá trị kênh đỏ (0.0 - 1.0)
     * @param g Giá trị kênh xanh lá (0.0 - 1.0)
     * @param b Giá trị kênh xanh dương (0.0 - 1.0)
     * @param h Con trỏ đến H (0-360 độ)
     * @param s Con trỏ đến S (0.0-1.0)
     * @param v Con trỏ đến V (0.0-1.0)
     */
    void rgbToHsv(float r, float g, float b, float *h, float *s, float *v);
    
    /**
     * @brief Chuyển từ không gian RGB sang XYZ
     * @param r Giá trị kênh đỏ (0.0 - 1.0)
     * @param g Giá trị kênh xanh lá (0.0 - 1.0)
     * @param b Giá trị kênh xanh dương (0.0 - 1.0)
     * @param x Con trỏ đến X
     * @param y Con trỏ đến Y
     * @param z Con trỏ đến Z
     */
    void rgbToXyz(float r, float g, float b, float *x, float *y, float *z);
    
    /**
     * @brief Chuyển từ XYZ sang Lab
     * @param x Giá trị X
     * @param y Giá trị Y
     * @param z Giá trị Z
     * @param l Con trỏ đến L (0-100)
     * @param a Con trỏ đến a (-128 đến 127)
     * @param b Con trỏ đến b (-128 đến 127)
     */
    void xyzToLab(float x, float y, float z, float *l, float *a, float *b);
    
    /**
     * @brief Chuyển từ không gian RGB sang Lab
     * @param r Giá trị kênh đỏ (0.0 - 1.0)
     * @param g Giá trị kênh xanh lá (0.0 - 1.0)
     * @param b Giá trị kênh xanh dương (0.0 - 1.0)
     * @param l Con trỏ đến L (0.0-100.0)
     * @param a Con trỏ đến a (-128.0 to 127.0)
     * @param b_out Con trỏ đến b (-128.0 to 127.0)
     */
    void rgbToLab(float r, float g, float b, float *l, float *a, float *b_out);
    
    /**
     * @brief Tính khoảng cách trong không gian màu Lab
     * @return Khoảng cách Euclidean giữa hai màu
     */
    float colorDistance(float l1, float a1, float b1, float l2, float a2, float b2);
    
public:
    /**
     * @brief Lưu thông số calibrate vào EEPROM
     */
    void saveCalibrationToEEPROM();
    
    /**
     * @brief Đọc thông số calibrate từ EEPROM
     * @return true nếu đọc thành công, false nếu chưa có dữ liệu
     */
    bool loadCalibrationFromEEPROM();

public:
    /**
     * @brief Hiệu chuẩn màu xanh dương bằng mẫu hiện tại
     */
    void calibrateBlueColor();
    
public:
    /**
     * @brief Hiệu chuẩn màu xanh lá bằng mẫu hiện tại
     */
    void calibrateGreenColor();
    
    /**
     * @brief Kiểm tra xem đã hiệu chuẩn chưa
     * @return true nếu đã hiệu chuẩn, false nếu chưa
     */
    bool isCalibrated() const { return m_isCalibrated; }
};

#endif // COLOR_TRACKER_H
