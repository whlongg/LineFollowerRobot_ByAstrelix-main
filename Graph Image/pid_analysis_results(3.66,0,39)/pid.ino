#include <Wire.h> // Dùng để giao tiếp I2C giữa Arduino và cảm biến
#include "Adafruit_TCS34725.h"  // Thư viện điều khiển cảm biến màu TCS34725

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X); // Thời gian tích hợp tín hiệu (50ms - đọc nhanh),  Mức khuếch đại tín hiệu (4X - trung bình)
void setup() {
  Serial.begin(115200); // Khởi tạo giao tiếp Serial tốc độ 115200 baud
  if (tcs.begin()) {  // Thử khởi động cảm biến
    Serial.println("Đã tìm thấy cảm biến màu TCS34725."); // kết nối Thành công
  } else {
    Serial.println("Không tìm thấy TCS34725 ... hãy kiểm tra kết nối của bạn.");// Kết nối Thất bại
    while (1); // Dừng nếu không tìm thấy cảm biến
  }
}

void loop() {

  uint16_t r, g, b, c;   // Khai báo 4 biến để lưu giá trị màu
  tcs.getRawData(&r, &g, &b, &c);  // Đọc dữ liệu màu (RGB + Clear/độ sáng)

   Serial.print("R: "); Serial.print(r);   // In giá trị màu đỏ
  Serial.print("\tG: "); Serial.print(g);  // In giá trị màu xanh lá
  Serial.print("\tB: "); Serial.print(b);  // In giá trị màu xanh dương
  Serial.print("\tC: "); Serial.println(c);  // In giá trị độ sáng (clear)

  // Phân biệt màu đơn giản
  String color = "";  // Chuỗi để lưu kết quả nhận diện màu

  if (c < 150 ){
    color = "Màu Đen";  // Nếu độ sáng thấp → giả định là màu đen
  } else {
    if (r > g && r > b) {
      color = "Màu Đỏ ";   // Đỏ lớn nhất → màu đỏ
      
    } else if (g > r && g > b) {
      color = "Xanh Lá";   //Xanh lá lớn nhất → xanh lá
    } else if (b > r && b > g) {
      color = "Xanh Dương";  //Xanh dương lớn nhất → xanh dương
    } else {
      color = "Không Xác Định";   // Trường hợp không rõ ràng → không xác định
    }
  }

  Serial.print("Phát Hiện: ");
  Serial.println(color);  // In ra màu đã phát hiện
  Serial.println("");    // Xuống dòng
  delay(700); // Dừng 0,7 s rồi tiếp tục đo lần sau.
}