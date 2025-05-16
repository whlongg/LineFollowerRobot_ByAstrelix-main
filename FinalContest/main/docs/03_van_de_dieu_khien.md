# Báo cáo vấn đề về bộ điều khiển

## Vấn đề điều khiển Hybrid

Mã nguồn cho thấy robot hỗ trợ 3 kiểu điều khiển (PID, Fuzzy, Hybrid), nhưng cấu hình hiện tại không tận dụng được khả năng hybrid.

```cpp
// Trong main.ino
#define MODE_CONTROL CONTROLLER_PID // Chế độ điều khiển (PID, Fuzzy, Hybrid)
#define PIDW 1.0
#define FUZW 0.0
```

**Mô tả vấn đề:** 
- Robot được thiết lập để chạy ở chế độ PID thuần túy (MODE_CONTROL = CONTROLLER_PID)
- Ngay cả khi chuyển sang chế độ CONTROLLER_HYBRID, cấu hình PIDW=1.0 và FUZW=0.0 sẽ vẫn tương đương với chỉ sử dụng PID
- Không tận dụng được sức mạnh của hệ điều khiển kết hợp
- Trong RobotController.cpp, hàm setHybridWeights() chuẩn hóa tổng trọng số về 1.0, nhưng giá trị mặc định được thiết lập không hợp lý

**Giải pháp đề xuất:**
- Khi sử dụng MODE_CONTROL = CONTROLLER_HYBRID, nên điều chỉnh trọng số PIDW và FUZW để tận dụng cả hai bộ điều khiển
- Thử nghiệm với các giá trị trọng số khác nhau như PIDW=0.7, FUZW=0.3 để tìm cấu hình tối ưu
- Cân nhắc việc tự động điều chỉnh trọng số dựa trên tình huống (tốc độ cao, góc cua gấp, v.v.)

## Vấn đề tham số PID

```cpp
// Trong main.ino
#define KP 0.38     // Tăng hệ số tỉ lệ để phản ứng nhanh hơn với lỗi
#define KI 0.0    // Giảm hệ số tích phân để tránh dao động
#define KD 0.0    // Tăng hệ số vi phân để ổn định khi đường cong
```

**Mô tả vấn đề:** 
- KI và KD đều được đặt bằng 0, có nghĩa là bộ điều khiển chỉ là "P" thuần túy, không phải "PID"
- Comment và cài đặt không khớp nhau: comment đề cập đến việc tăng KD nhưng thực tế KD = 0
- Trong điều kiện đường cong hoặc góc cua, thiếu thành phần vi phân có thể dẫn đến phản ứng không đủ nhanh
- Có sự mâu thuẫn giữa:
  * Khai báo mặc định trong RobotController.h: `kp = 0.38f, ki = 0.0f, kd = 3.8f`
  * Cấu hình trong main.ino: `KP = 0.38, KI = 0.0, KD = 0.0`

**Giải pháp đề xuất:**
- Thử nghiệm với các giá trị KD > 0 để cải thiện phản ứng khi có sự thay đổi đột ngột về error
- Cân nhắc sử dụng một giá trị KI nhỏ để giảm sai số tĩnh
- Đồng nhất các giá trị mặc định trong các file khác nhau

## Vấn đề trong FuzzyController

```cpp
// Trong FuzzyController.cpp
ruleTable[NL][NL] = HARD_LEFT;   ruleTable[NL][NM] = LEFT;        ruleTable[NL][NS] = SLIGHT_LEFT;
// ... còn nhiều cấu hình rule khác
```

**Mô tả vấn đề:**
- Bảng luật Fuzzy rất phức tạp và khó điều chỉnh
- Không có cơ chế tự động điều chỉnh các tham số Fuzzy trong quá trình chạy
- Bộ phân loại màu sắc trong ColorTracker.cpp có thể tạo ra các quyết định sai khi điều kiện ánh sáng thay đổi

**Giải pháp đề xuất:**
- Xem xét đơn giản hóa bảng luật Fuzzy
- Thêm chức năng tự hiệu chỉnh các tham số Fuzzy dựa trên hiệu suất
- Kết hợp thông tin từ cảm biến màu sắc để điều chỉnh bộ điều khiển Fuzzy theo từng loại đường đi
