# Báo cáo phân tích tổng quan

## Cấu trúc dự án

Dự án Line Following Robot được xây dựng với kiến trúc module hóa, bao gồm các thành phần chính:

1. **ReadIR**: Module đọc và xử lý tín hiệu từ cảm biến hồng ngoại
2. **MotorControl**: Điều khiển động cơ 
3. **PIDController**: Bộ điều khiển PID
4. **FuzzyController**: Bộ điều khiển dựa trên logic mờ
5. **RobotController**: Quản lý tổng thể các chiến lược điều khiển (PID, Fuzzy, Hybrid)
6. **ColorTracker**: Xử lý thông tin màu sắc từ cảm biến
7. **TurnLogic**: Xử lý logic rẽ của robot
8. **BLEdebug**: Module hỗ trợ debug qua Bluetooth Low Energy

Kiến trúc thiết kế này giúp phân tách rõ ràng các chức năng, dễ bảo trì và mở rộng. Tuy nhiên, vẫn tồn tại một số vấn đề trong thiết kế và triển khai có thể ảnh hưởng đến hiệu suất và độ ổn định của hệ thống.

## Phương pháp điều khiển

Robot sử dụng ba phương pháp điều khiển khác nhau có thể được lựa chọn:

1. **Điều khiển PID thuần túy**: Phản hồi tỷ lệ-tích phân-vi phân với tham số (KP=0.38, KI=0.0, KD=0.0)
2. **Điều khiển Fuzzy Logic**: Sử dụng lý thuyết tập mờ với 7 tập mờ đầu vào và 7 tập mờ đầu ra
3. **Điều khiển Hybrid**: Kết hợp cả PID và Fuzzy với trọng số có thể điều chỉnh (PIDW=1.0, FUZW=0.0)

Hiện tại, mã nguồn đang thiết lập mặc định là chế độ PID (MODE_CONTROL = CONTROLLER_PID), mặc dù trọng số được thiết lập không tận dụng được khả năng hybrid.

## Chú ý chung

Qua phân tích ban đầu, dự án đã được thiết kế với nhiều tính năng nâng cao. Tuy nhiên, một số điểm cần lưu ý:

1. Dự án sử dụng ESP32 Version 3.0 API, nhưng có một số API đã bị loại bỏ hoặc thay đổi cần được cập nhật
2. Hệ thống đã được thiết kế để tránh sử dụng delay() trong luồng chính, nhưng vẫn còn một số vị trí sử dụng delay()
3. Có nhiều tính năng thông minh như tự động điều chỉnh tốc độ dựa trên màu sắc đường đi
4. Cấu trúc chương trình đã được thiết kế theo hướng non-blocking và event-driven
