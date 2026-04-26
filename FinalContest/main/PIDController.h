#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>
#include <algorithm>

class PIDController {
public:
    // Khởi tạo bộ điều khiển PID với các tham số kp, ki, kd
    PIDController(float kp = 1.0f, float ki = 0.0f, float kd = 0.0f);
    
    // Reset lại bộ điều khiển
    void reset();
    
    // Tính toán đầu ra PID từ error
    float compute(float error, float dt = 1.0f);
    
    // Thiết lập các tham số PID
    void setTunings(float kp, float ki, float kd);
    
    // Thiết lập giới hạn đầu ra
    void setOutputLimits(float min, float max);
    
    // Thiết lập giới hạn tích phân
    void setIntegralLimits(float min, float max);
    
    // Lấy các giá trị hiện tại
    float getKp() const { return kp; }
    float getKi() const { return ki; }
    float getKd() const { return kd; }
    float getLastError() const { return lastError; }
    float getIntegral() const { return integral; }
    
private:
    // Các tham số PID
    float kp;   // Hệ số tỉ lệ
    float ki;   // Hệ số tích phân
    float kd;   // Hệ số vi phân
    
    // Trạng thái của bộ điều khiển
    float lastError;  // Sai số trước đó
    float integral;   // Giá trị tích phân
    float lastDerivative; // Giá trị đạo hàm trước đó
    
    // Giới hạn
    float outputMin;   // Giới hạn đầu ra nhỏ nhất
    float outputMax;   // Giới hạn đầu ra lớn nhất
    float integralMin; // Giới hạn tích phân nhỏ nhất
    float integralMax; // Giới hạn tích phân lớn nhất
    bool isFirstCompute; // Kiểm tra có phải lần gọi compute đầu tiên không
    //filter
    const float alpha = 0.7f; // Hệ số lọc (có thể điều chỉnh)
};

#endif // PID_CONTROLLER_H
