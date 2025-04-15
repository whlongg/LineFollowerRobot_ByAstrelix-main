// --- Motor Control (PID-based, smooth, DYNAMIC SPEED) ---
void applyMotorSpeed(float error, float correction) { // Thêm error vào tham số hàm

    // --- Dynamic Base Speed Calculation ---
    float abs_error = abs(error);
    int current_base_speed;
  
    // Map the absolute error to the base speed range
    // If error is small (<= MAX_ERROR_FOR_HIGH_SPEED), use MAX_STRAIGHT_SPEED
    // If error is large (>= MIN_ERROR_FOR_LOW_SPEED), use CORNERING_SPEED
    // Linearly interpolate between the two speeds in the transition zone
    current_base_speed = map(abs_error * 100, // Nhân 100 để dùng map với số nguyên
                             MAX_ERROR_FOR_HIGH_SPEED * 100,
                             MIN_ERROR_FOR_LOW_SPEED * 100,
                             MAX_STRAIGHT_SPEED,
                             CORNERING_SPEED);
  
    // Constrain the calculated speed to the defined limits
    current_base_speed = constrain(current_base_speed, CORNERING_SPEED, MAX_STRAIGHT_SPEED);
  
    // --- Apply Correction ---
    // Correction is scaled by CORRECTION_SCALE
    int left_speed = current_base_speed + correction * CORRECTION_SCALE;
    int right_speed = current_base_speed - correction * CORRECTION_SCALE;
  
    // Constrain final speeds to PWM limits
    left_speed = constrain(left_speed, 0, MAX_PWM);
    right_speed = constrain(right_speed, 0, MAX_PWM);
  
    setMotorSpeeds(left_speed, right_speed);
  
    #if DEBUG
      // Thêm current_base_speed vào log nếu muốn theo dõi
      // Ví dụ: Serial.print(current_base_speed); Serial.print(",");
      Serial.print(left_speed); Serial.print(",");
      Serial.println(right_speed);
    #endif
  }