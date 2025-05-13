#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "FuzzyController.h"
#include "PIDController.h"

/**
 * @brief Loại bộ điều khiển robot
 */
enum ControllerType {
    CONTROLLER_PID,     ///< Chỉ sử dụng bộ điều khiển PID
    CONTROLLER_FUZZY,   ///< Chỉ sử dụng bộ điều khiển Fuzzy
    CONTROLLER_HYBRID   ///< Kết hợp cả hai bộ điều khiển
};

/**
 * @brief Bộ điều khiển robot tích hợp cả PID và Fuzzy
 */
class RobotController {
public:
    /**
     * @brief Khởi tạo bộ điều khiển robot
     * 
     * @param type Loại bộ điều khiển
     * @param kp Hệ số tỉ lệ PID
     * @param ki Hệ số tích phân PID
     * @param kd Hệ số vi phân PID
     * @param inputMin Giá trị đầu vào tối thiểu cho Fuzzy
     * @param inputMax Giá trị đầu vào tối đa cho Fuzzy
     * @param outputMin Giá trị đầu ra tối thiểu
     * @param outputMax Giá trị đầu ra tối đa
     */
    explicit RobotController(
        ControllerType type = CONTROLLER_PID,
        float kp = 1.0f, float ki = 0.0f, float kd = 0.0f,
        float inputMin = -100.0f, float inputMax = 100.0f,
        float outputMin = -100.0f, float outputMax = 100.0f
    );
    
    /**
     * @brief Tính toán đầu ra điều khiển dựa trên error
     * 
     * @param error Sai số giữa giá trị mong muốn và thực tế
     * @param dt Delta time giữa các lần gọi (tính theo giây)
     * @return int Giá trị điều khiển đầu ra
     */
    int compute(float error, float dt);
    
    /**
     * @brief Thiết lập loại bộ điều khiển
     * 
     * @param type Loại bộ điều khiển cần thiết lập
     */
    void setControllerType(ControllerType type);
    
    /**
     * @brief Thiết lập các tham số PID
     * 
     * @param kp Hệ số tỉ lệ
     * @param ki Hệ số tích phân
     * @param kd Hệ số vi phân
     */
    void setPIDTunings(float kp, float ki, float kd);
    
    /**
     * @brief Thiết lập giới hạn đầu ra cho cả hai bộ điều khiển
     * 
     * @param min Giá trị tối thiểu
     * @param max Giá trị tối đa
     */
    void setOutputLimits(float min, float max);
    
    /**
     * @brief Thiết lập phạm vi đầu vào cho bộ điều khiển Fuzzy
     * 
     * @param min Giá trị đầu vào tối thiểu
     * @param max Giá trị đầu vào tối đa
     */
    void setFuzzyInputRange(float min, float max);
    
    /**
     * @brief Đặt lại trạng thái của bộ điều khiển
     */
    void reset();
    
    /**
     * @brief Thiết lập trọng số cho điều khiển hybrid
     * 
     * @param pidWeight Trọng số cho bộ điều khiển PID
     * @param fuzzyWeight Trọng số cho bộ điều khiển Fuzzy
     */
    void setHybridWeights(float pidWeight, float fuzzyWeight);
    
    /**
     * @brief Lấy loại bộ điều khiển hiện tại
     * 
     * @return ControllerType Loại bộ điều khiển
     */
    ControllerType getControllerType() const { return controllerType; }
    
private:
    PIDController pid;            ///< Bộ điều khiển PID
    FuzzyController fuzzy;        ///< Bộ điều khiển Fuzzy
    ControllerType controllerType; ///< Loại bộ điều khiển hiện tại
    float lastError;              ///< Sai số lần trước đó
    float pidWeight;              ///< Trọng số cho PID trong chế độ hybrid
    float fuzzyWeight;            ///< Trọng số cho Fuzzy trong chế độ hybrid
};

#endif // ROBOT_CONTROLLER_H
