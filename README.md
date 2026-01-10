A sophisticated line-following robot project built using Arduino with PID control optimization.

## Project Overview

This repository contains the complete source code and documentation for an autonomous line-following robot developed by the Astrelix team. The robot uses infrared sensors to detect and follow black lines on light surfaces, employing advanced PID (Proportional-Integral-Derivative) control algorithms for precise movement control.

## Key Features

- **PID Control Algorithm**: Optimized motor control using PID feedback for smooth and accurate line following
- **Multiple Implementation Versions**: Various iterations and optimizations of the core algorithm
- **Motor Control Module**: Dedicated module for motor operation and speed management
- **Data Logging**: Comprehensive logging system to record robot performance metrics
- **Visualization Tools**: Python scripts for plotting and analyzing robot performance data

## Repository Structure

```
LineFollowerRobot_ByAstrelix-main/
├── pid_MAIN_TARGET.ino          # Main PID implementation (primary code)
├── line_follower_optimized.ino  # Optimized version of line follower logic
├── pid_backup.ino               # Backup PID implementation
├── pid_byOpenAI.ino             # PID implementation with AI assistance
├── pid_test_dev.ino             # Development and testing version
├── motorControlModule.ino       # Motor control module
├── test.ino                     # General testing code
├── examples.ino                 # Example usage and reference code
├── test_motor.ino               # Motor testing utilities
├── graph.py                     # Data visualization script
├── merge_images.py              # Image processing utility
├── robot_log.csv                # Performance log data
├── epoch.txt                    # Training/calibration data
├── so_do_thuat_toan.md          # Algorithm flowchart documentation
├── updateFBLE.md                # Bluetooth Low Energy (BLE) updates
├── luudothuattoan.txt           # Algorithm notes (Vietnamese)
├── logcode.txt                  # Logging code documentation
├── FinalContest/                # Final competition code
├── TEST FUNC/                   # Test functions directory
└── Graph Image/                 # Performance visualization images
```

## Main Files Description

### Firmware Files (.ino)

| File | Purpose |
|------|---------|
| `pid_MAIN_TARGET.ino` | Primary implementation with main PID control loop |
| `line_follower_optimized.ino` | Optimized version with performance improvements |
| `pid_test_dev.ino` | Development version with extensive debugging features |
| `motorControlModule.ino` | Motor PWM control and speed adjustment |
| `examples.ino` | Example usage and configuration reference |

### Support Files

| File | Purpose |
|------|---------|
| `graph.py` | Matplotlib visualization of performance data |
| `merge_images.py` | Image concatenation for comparison views |
| `robot_log.csv` | Time-series data of sensor and motor values |

## Getting Started

### Hardware Requirements

- Arduino microcontroller (Uno/Mega recommended)
- IR line sensors (typically 5+ sensors for line detection)
- DC motors with PWM control capability
- Motor driver module (L298N or similar)
- Power supply (battery pack)
- Wheels and chassis

### Software Requirements

- Arduino IDE
- Python 3.x (for data visualization)
- Matplotlib library (for graphing)

### Installation

1. **Clone the repository**
   ```bash
   git clone https://github.com/whlongg/LineFollowerRobot_ByAstrelix-main.git
   cd LineFollowerRobot_ByAstrelix-main
   ```

2. **Upload firmware to Arduino**
   - Open `pid_MAIN_TARGET.ino` in Arduino IDE
   - Select appropriate board and COM port
   - Click Upload

3. **Install Python dependencies (optional)**
   ```bash
   pip install matplotlib numpy
   ```

## How It Works

### PID Control System

The robot uses a PID controller to maintain position on the line:

- **Proportional (P)**: Responsive to current error from the line
- **Integral (I)**: Corrects accumulated errors over time
- **Derivative (D)**: Predicts future error and prevents overshoot

The IR sensors detect the line position, and the PID algorithm adjusts motor speeds accordingly to keep the robot centered on the line.

### Algorithm Flow

```
Read IR Sensors
    ↓
Calculate Error (Line Position)
    ↓
PID Calculation (P + I + D terms)
    ↓
Adjust Motor PWM Values
    ↓
Drive Motors
    ↓
Log Data (optional)
```

## Configuration & Tuning

### Adjusting PID Parameters

Modify these constants in the main sketch:

```cpp
float Kp = 1.0;  // Proportional gain
float Ki = 0.1;  // Integral gain
float Kd = 0.5;  // Derivative gain
```

**Tuning Tips:**
- Start with Kp only, increase until oscillation occurs
- Add Ki to eliminate steady-state error
- Add Kd to reduce overshoot

### Sensor Calibration

The robot includes calibration routines to adapt to different lighting conditions and track colors. Refer to the comments in `pid_MAIN_TARGET.ino` for calibration procedures.

## Performance Analysis

### Generating Performance Graphs

1. **Collect data**: The robot logs sensor and motor values to serial output
2. **Process data**: Save the serial output to `robot_log.csv`
3. **Visualize**: Run the Python script
   ```bash
   python graph.py
   ```

This generates graphs showing:
- Line position error over time
- Motor speed adjustments
- PID term contributions
- Overall robot performance metrics

## File Guide

### Version Comparison

- **pid_MAIN_TARGET.ino**: Latest stable version (recommended)
- **pid_test_dev.ino**: Extensive features, includes debugging and data logging
- **line_follower_optimized.ino**: Focused on speed and efficiency
- **pid_byOpenAI.ino**: AI-assisted implementation

## Troubleshooting

### Robot Doesn't Follow the Line

1. Check sensor calibration
2. Verify motor connections and polarity
3. Adjust PID parameters (start with Kp)
4. Ensure adequate lighting conditions

### Oscillation/Jerky Movement

- Reduce Kp value
- Increase Kd value to add damping
- Check for mechanical friction

### Slow Response

- Increase Kp value
- Check motor power supply voltage

## Contributing

Contributions are welcome! Please feel free to:
- Report issues
- Suggest improvements
- Submit pull requests with enhancements

## License

This project is open source. See repository for license details.

## References

- Algorithm documentation: See `so_do_thuat_toan.md`
- BLE/Connectivity updates: See `updateFBLE.md`
- Contest version: See `FinalContest/` directory

## Contact

For questions or collaboration, reach out to the project maintainer at https://github.com/whlongg

---

**Last Updated**: January 2026

**Status**: Active Development

**Team**: Whlong