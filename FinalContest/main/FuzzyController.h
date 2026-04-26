#ifndef FUZZY_CONTROLLER_H
#define FUZZY_CONTROLLER_H

#include <stdint.h>
#include <math.h>
#include <algorithm> // Thêm include cho std::min, std::max

// Định nghĩa các tập mờ đầu vào
enum FuzzySet { NL, NM, NS, Z, PS, PM, PL };
// Định nghĩa các đầu ra mờ
enum FuzzyOutput { HARD_LEFT, LEFT, SLIGHT_LEFT, STRAIGHT, SLIGHT_RIGHT, RIGHT, HARD_RIGHT };

// Cấu trúc lưu trữ mức độ thành viên
struct MembershipDegree {
    float NL, NM, NS, Z, PS, PM, PL;
};

class FuzzyController {
public:
    FuzzyController();
    
    // Phương thức chính để tính toán đầu ra từ error và deltaError
    int computeOutput(float error, float deltaError);
    
    // Thiết lập tùy chỉnh cho phạm vi đầu vào
    void setInputRange(float min, float max);
    
    // Thiết lập tùy chỉnh cho phạm vi đầu ra
    void setOutputRange(float min, float max);

private:
    // Hàm tính toán mức độ thành viên của giá trị trong các tập mờ
    MembershipDegree fuzzify(float val);
    
    // Giải mờ hóa bằng phương pháp Center of Gravity
    int defuzzify(float outputStrength[7]);
    
    // Bảng luật suy luận
    FuzzyOutput ruleTable[7][7];
    
    // Các giá trị cho các tập mờ đầu vào
    struct {
        float NL_center, NL_width;
        float NM_center, NM_width;
        float NS_center, NS_width;
        float Z_center, Z_width;
        float PS_center, PS_width;
        float PM_center, PM_width;
        float PL_center, PL_width;
    } inputSets;
    
    // Giá trị trung tâm của các tập mờ đầu ra
    float outputCenters[7];
    
    // Phạm vi đầu vào và đầu ra
    float inputMin, inputMax;
    float outputMin, outputMax;
    
    // Các hàm thành viên (tam giác)
    float triangularMF(float x, float center, float width);
};

#endif // FUZZY_CONTROLLER_H
