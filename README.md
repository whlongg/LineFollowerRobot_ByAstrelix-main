# High-Speed Adaptive Line-Following Control System (ESP32)

An advanced embedded control system featuring dual-stage Exponential Moving Average (EMA) signal filtering, Adaptive Base-Speed PID regulation, and high-frequency real-time telemetry over Bluetooth Low Energy (BLE).

## System Architecture & Key Engineering Highlights

Unlike standard hobbyist line-followers that rely on basic `if/else` logic or static-speed PID on legacy microcontrollers, this production-ready firmware leverages the power of the **ESP32 dual-core architecture** to implement highly-optimized control theory and signal processing techniques.

### 1. Signal Processing: Dual-Stage EMA Low-Pass Filtering
High-frequency switching noise from ambient sa bàn lighting and sensor cross-talk can cause massive derivative kicks in PID systems. This project solves this by implementing a hardware-efficient **Exponential Moving Average (EMA) Filter** ($\alpha = 0.4$) applied directly to both raw and normalized analog infrared array inputs, ensuring smooth error computation without introducing significant phase lag.

### 2. Control Theory: Adaptive Base-Speed Regulation
To achieve maximum velocity on straights while preventing overshoot and spin-outs on sharp corners, the system computes a dynamic baseline velocity rather than using a static reference:
- **Straight-Line Aggression**: Voids cross-track error ($|e| \le 2.5$) and ramps up to maximum terminal velocity (`max_straight_speed = 250`).
- **Dynamic Cornering Damping**: Continuously maps intermediate errors using non-linear interpolation to adjust the base speed down to `cornering_speed (210)`.
- **Sharp-Turn Guard**: Instantly applies a speed reduction penalty (`sharp_turn_speed_reduction = 60`) upon detecting line presence on outer extreme sensors.

### 3. Industrial-Grade PID Anti-Windup Logic
To counteract the cumulative growth of the integral term during long curves (Integral Windup), the controller implements strict optimization boundaries:
- **Integral Windowing**: Accumulation only occurs within a tight error margin ($|e| < 1.0$).
- **Instantaneous Zero-Crossing Reset**: The integral accumulator is instantly wiped out (`integral = 0`) if the error changes signs or spikes past a critical threshold, eliminating oscillation-induced crashes.

### 4. Real-Time BLE Telemetry (Data Science Integration)
The system leverages the ESP32 NimBLE/BLE stack to stream a high-frequency CSV telemetry pipe (`LOOP_INTERVAL = 2ms`, `BLE_LOG_INTERVAL_MS = 5ms`) including timestamp, normalized IR array values, raw $P, I, D$ components, computed error, and dual-motor PWM outputs. This data is fed into a custom Python-Matplotlib pipeline for real-time control loop analysis and parameter tuning.

---

## Core Repository Structure


```

├── pid_MAIN_TARGET.ino          # Primary production firmware (Monolithic/High-Speed Execution)
├── banket.cpp                   # Semi-final tournament iteration code
├── graph.py                     # Python telemetry parser and Matplotlib analytical pipeline
├── robot_log.csv                # High-frequency time-series system execution logs
├── Graph Image/                 # Post-run analytical visualization reports
│   ├── error_and_speed.png      # Velocity vs. Cross-track error analysis
│   └── pid_components.png       # Real-time P, I, D compensation profiles

```

## Hardware & Pin Configuration (ESP32-Native)

The firmware bypasses standard Arduino overhead by utilizing native ESP32 hardware peripherals for execution efficiency:
- **Actuators**: Coreless High-RPM DC Motors regulated via the hardware-native **ESP32 LEDC Component** running at a high-frequency PWM of `20KHz` with an 8-bit resolution to completely eradicate audible coil whine and step jitters.
- **Sensors**: 4-Channel Analog IR Reflective Array + Adafruit TCS34725 RGB Sensor via $I^2C$ for cross-track localization and boundary checking.

| Peripheral / Signal | ESP32 GPIO Pin Pinout |
|---------------------|-----------------------|
| IR Sensor Array     | GPIO 4, 3, 1, 0       |
| Left Motor PWM (A/B)| GPIO 2, 10            |
| Right Motor PWM (A/B)| GPIO 6, 5            |
| TCS34725 SDA / SCL  | Standard $I^2C$ Pins  |
| Debug Trigger Button| GPIO 7                |

---

## Embedded Control Algorithms Insight

### High-Frequency Control Loop Execution
```cpp
void loop() {
  unsigned long now = millis();
  if (now - lastLoop < LOOP_INTERVAL) return; // Enforces deterministic 500Hz loop execution
  lastLoop = now;

  readIRSensors(); // Dual-stage EMA Filtering
  
  // High-Speed Autonomous Finite State Machine (FSM)
  switch (current_state) {
    case STATE_LINE_FOLLOWING:
      float error = computeError();
      float correction = computePID(error);
      applyMotorSpeed(error, correction); // Adaptive Base-Speed Execution
      break;
    ...
  }
}

```

### Telemetry Processing & Analytical Insights

By converting real-time BLE notifications into `robot_log.csv`, the mathematical tuning curves can be observed via `graph.py`.

* **Proportional Component**: Tracks the physical layout of the path.
* **Derivative Component**: Dampens rapid shifts, utilizing a filtered derivative alpha (`D_FILTER_ALPHA = 0.5`) to eliminate noisy high-frequency spikes.

---

## 👥 Authors & Development Status

* **Maintainer**: WHLong (formerly AstrelixDev)
* **Role**: Embedded Firmware & Control Systems Developer
* **Status**: Completed Deployment (Finalist Competition Architecture)
