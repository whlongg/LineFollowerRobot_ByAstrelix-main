#include "ColorTracker.h"

// Tạo đối tượng ColorTracker với thời gian đọc là 20ms
ColorTracker colorTracker(51);

void setup() {
  Serial.begin(115200);
  Serial.println("Color Tracking Example");
  
  // Khởi tạo cảm biến
  if (!colorTracker.begin()) {
    Serial.println("Không thể kết nối với cảm biến TCS34725! Vui lòng kiểm tra lại kết nối.");
    while (1);
  }
  
  // Tùy chỉnh ngưỡng màu nếu cần
  // colorTracker.setThresholds(4000, 800, 1.2, 1.2);
  
  Serial.println("Color Tracker đã sẵn sàng");
  Serial.println("Thời gian, Màu, High Speed, R, G, B, C");
}

void loop() {
  // Cập nhật giá trị từ cảm biến
  colorTracker.update();
  
  // Các biến để lưu kết quả
  uint8_t color;
  bool highSpeedMode;
  unsigned long timestamp;
  
  // Nếu có dữ liệu mới từ cảm biến
  if (colorTracker.getResult(&color, &highSpeedMode, &timestamp)) {
    // Lấy giá trị RGB raw để debug
    uint16_t r, g, b, c;
    colorTracker.getRawValues(&r, &g, &b, &c);
    
    // Hiển thị kết quả
    Serial.print(timestamp);
    Serial.print(", ");
    
    // Hiển thị tên màu
    switch (color) {
      case COLOR_WHITE:
        Serial.print("Trắng");
        break;
      case COLOR_BLACK:
        Serial.print("Đen");
        break;
      case COLOR_BLUE:
        Serial.print("Xanh Dương");
        break;
      case COLOR_GREEN:
        Serial.print("Xanh Lá");
        break;
      default:
        Serial.print("Unknown");
        break;
    }
    
    Serial.print(", ");
    Serial.print(highSpeedMode ? "HIGH" : "BASE");
    Serial.print(", ");
    
    // In giá trị RGB và Clear để debug
    Serial.print(r);
    Serial.print(", ");
    Serial.print(g);
    Serial.print(", ");
    Serial.print(b);
    Serial.print(", ");
    Serial.println(c);
  }
  
  // Mã chương trình chính khác ở đây
  // ...
  
  // Không cần delay() vì ColorTracker đã xử lý thời gian tự động
}
