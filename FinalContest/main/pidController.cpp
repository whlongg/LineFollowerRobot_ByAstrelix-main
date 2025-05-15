#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd) 
    : kp(kp), ki(ki), kd(kd), 
      lastError(0.0f), integral(0.0f), 
      outputMin(-100.0f), outputMax(100.0f),
      integralMin(-100.0f), integralMax(100.0f),
      isFirstCompute(true) {
}

void PIDController::reset() {
    lastError = 0.0f;
    integral = 0.0f;
    isFirstCompute = true;
}

float PIDController::compute(float error, float dt) {
    // Xác định đạo hàm của error
    float derivative = 0.0f;
    
    if (!isFirstCompute) {
        // Nếu không phải lần đầu, tính đạo hàm của error
        derivative = (error - lastError) / dt;
    } else {
        // Nếu là lần đầu, bỏ qua thành phần đạo hàm
        isFirstCompute = false;
    }
    
    // Cập nhật giá trị integral
    integral += error * dt;
    
    // Giới hạn giá trị integral để tránh tích lũy quá mức
    integral = std::max(integralMin, std::min(integralMax, integral));
    
    // Lưu error hiện tại để sử dụng cho lần tính toán tiếp theo
    lastError = error;
    
    // Tính toán đầu ra PID
    float output = kp * error + ki * integral + kd * derivative;
    
    // Giới hạn đầu ra
    output = std::max(outputMin, std::min(outputMax, output));
    
    return output;
}

void PIDController::setTunings(float kp, float ki, float kd) {
    // Đảm bảo các hệ số không âm
    this->kp = kp >= 0 ? kp : 0;
    this->ki = ki >= 0 ? ki : 0;
    this->kd = kd >= 0 ? kd : 0;
}

void PIDController::setOutputLimits(float min, float max) {
    if (min >= max) return; // Tham số không hợp lệ
    
    outputMin = min;
    outputMax = max;
}

void PIDController::setIntegralLimits(float min, float max) {
    if (min >= max) return; // Tham số không hợp lệ
    
    integralMin = min;
    integralMax = max;
    
    // Cập nhật lại integral nếu nó vượt ra ngoài giới hạn mới
    integral = std::max(integralMin, std::min(integralMax, integral));
}
