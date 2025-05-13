#include "RobotController.h"
#include <algorithm>

RobotController::RobotController(ControllerType type, float kp, float ki, float kd, 
                               float inputMin, float inputMax, float outputMin, float outputMax)
    : controllerType(type), 
      lastError(0.0f),
      pidWeight(0.5f), 
      fuzzyWeight(0.5f) {
          
    // Thiết lập các tham số PID
    pid.setTunings(kp, ki, kd);
    pid.setOutputLimits(outputMin, outputMax);
    
    // Thiết lập phạm vi cho Fuzzy
    fuzzy.setInputRange(inputMin, inputMax);
    fuzzy.setOutputRange(outputMin, outputMax);
}

int RobotController::compute(float error, float dt) {
    // Tính deltaError cho Fuzzy Controller
    float deltaError = (error - lastError) / dt;
    
    // Lưu error hiện tại cho lần tính toán tiếp theo
    lastError = error;
    
    // Tính toán đầu ra dựa trên loại điều khiển được chọn
    switch (controllerType) {
        case CONTROLLER_PID:
            return static_cast<int>(pid.compute(error, dt));
            
        case CONTROLLER_FUZZY:
            return fuzzy.computeOutput(error, deltaError);
            
        case CONTROLLER_HYBRID: {
            // Kết hợp cả hai đầu ra với trọng số tương ứng
            float pidOutput = pid.compute(error, dt);
            int fuzzyOutput = fuzzy.computeOutput(error, deltaError);
            
            // Kết hợp có trọng số
            return static_cast<int>(pidWeight * pidOutput + fuzzyWeight * fuzzyOutput);
        }
            
        default:
            return static_cast<int>(pid.compute(error, dt));
    }
}

void RobotController::setControllerType(ControllerType type) {
    // Nếu loại điều khiển không thay đổi, không cần reset
    if (type == controllerType) {
        return;
    }
    
    controllerType = type;
    reset();
}

void RobotController::setPIDTunings(float kp, float ki, float kd) {
    pid.setTunings(kp, ki, kd);
}

void RobotController::setOutputLimits(float min, float max) {
    // Kiểm tra tham số đầu vào
    if (min >= max) {
        return; // Bỏ qua thiết lập không hợp lệ
    }
    
    // Thiết lập giới hạn đầu ra cho cả hai bộ điều khiển
    pid.setOutputLimits(min, max);
    fuzzy.setOutputRange(min, max);
}

void RobotController::setFuzzyInputRange(float min, float max) {
    // Kiểm tra tham số đầu vào
    if (min >= max) {
        return; // Bỏ qua thiết lập không hợp lệ
    }
    
    fuzzy.setInputRange(min, max);
}

void RobotController::reset() {
    // Đặt lại trạng thái của bộ điều khiển
    pid.reset();
    lastError = 0.0f;
}

void RobotController::setHybridWeights(float pidWeight, float fuzzyWeight) {
    // Kiểm tra giá trị trọng số
    if (pidWeight < 0.0f || fuzzyWeight < 0.0f) {
        return; // Bỏ qua thiết lập không hợp lệ
    }
    
    // Chuẩn hóa trọng số để tổng bằng 1.0
    float total = pidWeight + fuzzyWeight;
    
    if (total <= 0.0f) {
        this->pidWeight = 0.5f;
        this->fuzzyWeight = 0.5f;
    } else {
        this->pidWeight = pidWeight / total;
        this->fuzzyWeight = fuzzyWeight / total;
    }
}
