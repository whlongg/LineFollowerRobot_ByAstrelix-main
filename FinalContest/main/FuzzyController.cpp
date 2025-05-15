#include "FuzzyController.h"

FuzzyController::FuzzyController() {
    // Thiết lập phạm vi mặc định
    inputMin = -100.0f;
    inputMax = 100.0f;
    outputMin = -100.0f;
    outputMax = 100.0f;
    
    // Khởi tạo các tập mờ đầu vào
    float range = inputMax - inputMin;
    float step = range / 6.0f; // Chia đều thành 7 tập mờ
    
    inputSets.NL_center = inputMin + step * 0;
    inputSets.NM_center = inputMin + step * 1;
    inputSets.NS_center = inputMin + step * 2; 
    inputSets.Z_center = inputMin + step * 3;  // Giữa khoảng
    inputSets.PS_center = inputMin + step * 4;
    inputSets.PM_center = inputMin + step * 5;
    inputSets.PL_center = inputMin + step * 6;
    
    // Độ rộng của mỗi hàm thành viên
    inputSets.NL_width = step * 1.5f;
    inputSets.NM_width = step;
    inputSets.NS_width = step;
    inputSets.Z_width = step;
    inputSets.PS_width = step;
    inputSets.PM_width = step;
    inputSets.PL_width = step * 1.5f;
    
    // Giá trị trung tâm đầu ra
    outputCenters[HARD_LEFT] = -100;
    outputCenters[LEFT] = -70;
    outputCenters[SLIGHT_LEFT] = -40;
    outputCenters[STRAIGHT] = 0;
    outputCenters[SLIGHT_RIGHT] = 40;
    outputCenters[RIGHT] = 70;
    outputCenters[HARD_RIGHT] = 100;
    
    // Khởi tạo bảng luật chính xác theo đúng bảng luật ban đầu
    // Hàng NL (Error)
    ruleTable[NL][NL] = HARD_LEFT;   ruleTable[NL][NM] = LEFT;        ruleTable[NL][NS] = SLIGHT_LEFT;
    ruleTable[NL][Z]  = SLIGHT_LEFT; ruleTable[NL][PS] = SLIGHT_LEFT; ruleTable[NL][PM] = STRAIGHT;    ruleTable[NL][PL] = STRAIGHT;

    // Hàng NM (Error) 
    ruleTable[NM][NL] = LEFT;        ruleTable[NM][NM] = SLIGHT_LEFT; ruleTable[NM][NS] = SLIGHT_LEFT;
    ruleTable[NM][Z]  = SLIGHT_LEFT; ruleTable[NM][PS] = STRAIGHT;    ruleTable[NM][PM] = STRAIGHT;    ruleTable[NM][PL] = SLIGHT_RIGHT;

    // Hàng NS (Error)
    ruleTable[NS][NL] = SLIGHT_LEFT; ruleTable[NS][NM] = SLIGHT_LEFT; ruleTable[NS][NS] = STRAIGHT;
    ruleTable[NS][Z]  = STRAIGHT;    ruleTable[NS][PS] = STRAIGHT;    ruleTable[NS][PM] = SLIGHT_RIGHT;ruleTable[NS][PL] = RIGHT;

    // Hàng Z (Error) 
    ruleTable[Z][NL]  = SLIGHT_LEFT; ruleTable[Z][NM]  = STRAIGHT;    ruleTable[Z][NS]  = STRAIGHT;
    ruleTable[Z][Z]   = STRAIGHT;    ruleTable[Z][PS]  = STRAIGHT;    ruleTable[Z][PM]  = STRAIGHT;    ruleTable[Z][PL]  = RIGHT;

    // Hàng PS (Error)
    ruleTable[PS][NL] = STRAIGHT;    ruleTable[PS][NM] = STRAIGHT;    ruleTable[PS][NS] = STRAIGHT;
    ruleTable[PS][Z]  = STRAIGHT;    ruleTable[PS][PS] = SLIGHT_RIGHT;ruleTable[PS][PM] = SLIGHT_RIGHT;ruleTable[PS][PL] = RIGHT;

    // Hàng PM (Error)
    ruleTable[PM][NL] = STRAIGHT;    ruleTable[PM][NM] = SLIGHT_RIGHT;ruleTable[PM][NS] = SLIGHT_RIGHT;
    ruleTable[PM][Z]  = SLIGHT_RIGHT;ruleTable[PM][PS] = SLIGHT_RIGHT;ruleTable[PM][PM] = RIGHT;       ruleTable[PM][PL] = RIGHT;

    // Hàng PL (Error)
    ruleTable[PL][NL] = STRAIGHT;    ruleTable[PL][NM] = STRAIGHT;    ruleTable[PL][NS] = SLIGHT_RIGHT;
    ruleTable[PL][Z]  = SLIGHT_RIGHT;ruleTable[PL][PS] = RIGHT;       ruleTable[PL][PM] = HARD_RIGHT;  ruleTable[PL][PL] = HARD_RIGHT;
}

// Thiết lập tùy chỉnh phạm vi đầu vào
void FuzzyController::setInputRange(float min, float max) {
    if (min >= max) return; // Tham số không hợp lệ
    
    inputMin = min;
    inputMax = max;
    
    // Tính lại các tập mờ đầu vào
    float range = inputMax - inputMin;
    float step = range / 6.0f;
    
    inputSets.NL_center = inputMin + step * 0;
    inputSets.NM_center = inputMin + step * 1;
    inputSets.NS_center = inputMin + step * 2;
    inputSets.Z_center = inputMin + step * 3;
    inputSets.PS_center = inputMin + step * 4;
    inputSets.PM_center = inputMin + step * 5;
    inputSets.PL_center = inputMin + step * 6;
    
    inputSets.NL_width = step * 1.5f;
    inputSets.NM_width = step;
    inputSets.NS_width = step;
    inputSets.Z_width = step;
    inputSets.PS_width = step;
    inputSets.PM_width = step;
    inputSets.PL_width = step * 1.5f;
}

// Thiết lập tùy chỉnh phạm vi đầu ra
void FuzzyController::setOutputRange(float min, float max) {
    if (min >= max) return; // Tham số không hợp lệ
    
    outputMin = min;
    outputMax = max;
    
    // Tính lại các giá trị trung tâm đầu ra theo tỷ lệ như trước
    float range = max - min;
    outputCenters[HARD_LEFT] = min;
    outputCenters[LEFT] = min + range * 0.15f;
    outputCenters[SLIGHT_LEFT] = min + range * 0.3f;
    outputCenters[STRAIGHT] = min + range * 0.5f;
    outputCenters[SLIGHT_RIGHT] = min + range * 0.7f;
    outputCenters[RIGHT] = min + range * 0.85f;
    outputCenters[HARD_RIGHT] = max;
}

// Hàm thành viên tam giác
float FuzzyController::triangularMF(float x, float center, float width) {
    float a = center - width;
    float b = center;
    float c = center + width;
    
    if(x <= a || x >= c) return 0.0f;
    else if(x < b) return (x - a) / (b - a);
    else return (c - x) / (c - b);
}

// Tính mức độ thành viên
MembershipDegree FuzzyController::fuzzify(float val) {
    MembershipDegree degree = {0}; // Khởi tạo tất cả giá trị bằng 0
    
    // Giới hạn giá trị trong phạm vi đầu vào
    val = std::max(inputMin, std::min(inputMax, val));
    
    // Tính mức độ thành viên cho từng tập mờ
    degree.NL = triangularMF(val, inputSets.NL_center, inputSets.NL_width);
    degree.NM = triangularMF(val, inputSets.NM_center, inputSets.NM_width);
    degree.NS = triangularMF(val, inputSets.NS_center, inputSets.NS_width);
    degree.Z  = triangularMF(val, inputSets.Z_center,  inputSets.Z_width);
    degree.PS = triangularMF(val, inputSets.PS_center, inputSets.PS_width);
    degree.PM = triangularMF(val, inputSets.PM_center, inputSets.PM_width);
    degree.PL = triangularMF(val, inputSets.PL_center, inputSets.PL_width);
    
    return degree;
}

// Giải mờ hóa (Defuzzification) bằng phương pháp Center of Gravity
int FuzzyController::defuzzify(float outputStrength[7]) {
    float numerator = 0.0f;
    float denominator = 0.0f;
    
    for (int i = 0; i < 7; i++) {
        numerator += outputStrength[i] * outputCenters[i];
        denominator += outputStrength[i];
    }
    
    // Xử lý trường hợp không có luật nào được kích hoạt
    if (denominator < 0.001f) return (int)outputCenters[STRAIGHT];
    
    float result = numerator / denominator;
    // Giới hạn và chuyển đổi kết quả sang số nguyên
    return (int)std::max(outputMin, std::min(outputMax, result));
}

// Phương thức chính để tính toán đầu ra từ error và deltaError
int FuzzyController::computeOutput(float error, float deltaError) {
    // Bước 1: Mờ hóa đầu vào
    MembershipDegree errorDegree = fuzzify(error);
    MembershipDegree deltaErrorDegree = fuzzify(deltaError);
    
    // Bước 2: Đánh giá luật và tổng hợp kết quả
    float outputStrength[7] = {0}; // Mức độ kích hoạt cho mỗi đầu ra mờ
    
    // Lấy mức độ thành viên của từng tập mờ đầu vào
    float eMembers[7] = {
        errorDegree.NL, errorDegree.NM, errorDegree.NS, errorDegree.Z,
        errorDegree.PS, errorDegree.PM, errorDegree.PL
    };
    
    float deMembers[7] = {
        deltaErrorDegree.NL, deltaErrorDegree.NM, deltaErrorDegree.NS, deltaErrorDegree.Z,
        deltaErrorDegree.PS, deltaErrorDegree.PM, deltaErrorDegree.PL
    };
    
    // Debug - in thông tin mức độ thành viên
    /*
    Serial.println("Error membership:");
    Serial.print("NL: "); Serial.print(eMembers[0]);
    Serial.print(", NM: "); Serial.print(eMembers[1]);
    Serial.print(", NS: "); Serial.print(eMembers[2]);
    Serial.print(", Z: "); Serial.print(eMembers[3]);
    Serial.print(", PS: "); Serial.print(eMembers[4]);
    Serial.print(", PM: "); Serial.print(eMembers[5]);
    Serial.print(", PL: "); Serial.println(eMembers[6]);
    */
    
    // Đánh giá từng luật và áp dụng phương pháp MAX-MIN
    for (int e = 0; e < 7; e++) {
        for (int de = 0; de < 7; de++) {
            if (eMembers[e] > 0 && deMembers[de] > 0) {
                FuzzyOutput output = ruleTable[e][de];
                // Áp dụng toán tử MIN cho từng cặp luật
                float ruleFiringStrength = std::min(eMembers[e], deMembers[de]);
                // Áp dụng toán tử MAX cho mỗi đầu ra
                outputStrength[output] = std::max(outputStrength[output], ruleFiringStrength);
            }
        }
    }
    
    // Bước 3: Giải mờ hóa
    int result = defuzzify(outputStrength);
    
    // Debug - in kết quả đầu ra
    //Serial.print("Fuzzy Output: ");
    //Serial.println(result);
    
    return result;
}
