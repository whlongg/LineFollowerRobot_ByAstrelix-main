#ifndef COLOR_TRACKER_H
#define COLOR_TRACKER_H

#include <Wire.h>
#include <Adafruit_TCS34725.h>

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
     * @param gain Độ khuếch đại (1x, 4x, 16x, 60x)
     */
    ColorTracker(uint16_t readInterval = 50, 
                 tcs34725IntegrationTime_t integrationTime = TCS34725_INTEGRATIONTIME_50MS,
                 tcs34725Gain_t gain = TCS34725_GAIN_4X);
    
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
    Adafruit_TCS34725 tcs;
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
    
    /**
     * @brief Đọc giá trị RGB và Clear từ cảm biến không chặn
     */
    void readRawData();
    
    /**
     * @brief Phân loại màu từ giá trị RGB hiện tại
     */
    void classifyColor();
};

#endif // COLOR_TRACKER_H
