# Autonomous Micromouse Robot (Version 1.0)

An advanced, fully autonomous maze-solving robot designed for competitive 16x16 cell micromouse mazes. The system is engineered around a custom dual-core ESP32-S3 architecture, featuring real-time sensor fusion, PID motor control, and a stochastically robust flood-fill pathfinding algorithm.

## 🚀 Key Features

- **Dual-Core Processing (FreeRTOS)**
  - **Core 0:** Manages a real-time asynchronous Web Server and a live UI dashboard (zero lag on motors).
  - **Core 1:** Dedicated to 1kHz hardware interrupts, PID control loops, Gyroscope reading, and motor PWM updates.
- **Advanced Pathfinding & Mapping**
  - Uses the classic **Flood-Fill Algorithm** to map unknown 16x16 mazes and dynamically calculate the mathematically shortest path to the center `(7,7)`.
  - Non-volatile memory (`EEPROM`) map saving and loading mechanism for multi-run efficiency.
  - State Machine Architecture: `EXPLORE`, `RETURN`, and `FAST RUN` phases.
- **Precision Closed-Loop Control**
  - **PID Controllers** for Gyroscope-assisted yaw correction, Encoder velocity matching, and Wall-distance tracking.
  - **Gyro Slalom (Curve) Turns:** Specialized smooth trajectory calculations for the high-speed "Fast Run" phase without complete stops.
- **Hardware Integration**
  - **ESP32-S3 Microcontroller** (240MHz)
  - **4Ã— VL53L0X ToF Sensors** (Front, Left, Right arrays with XSHUT addressing)
  - **MPU6050 6-Axis IMU** for absolute heading retention.
  - **TB6612FNG Motor Driver** with high-resolution magnetic wheel encoders.
- **Live Web Dashboard**
  - Embedded HTML/JS server hosting a live telemetry dashboard, complete with emergency stops, state triggers, and an HTML5 Canvas actively drawing the discovered maze layout in real-time over Wi-Fi.

## 📂 Project Structure

The project was developed modularly before the final integration. Each module isolates and tests a specific sub-system:
- `module01_hardware/`: Basic I/O testing (Motors, Encoders, XSHUT).
- `module03_motion_control/`: PID, straight-line tracking, and IMU calibration.
- `module04_turning_system_v2/`: Slalom turning mathematics and 90/180 degree point turns.
- `module05_maze_exploration/`: Wall thresholding and logic.
- `module06_maze_mapping/`: Flood-fill algorithm implementations and matrix operations.
- `module_final/`: **The Final Integrated Dual-Core Firmware** (`module_final.ino`).

## 🛠️ Setup & Flashing

1. **Prerequisites:**
   - [Arduino IDE](https://www.arduino.cc/en/software) or VS Code (PlatformIO).
   - Install ESP32 Board Definitions in the Boards Manager.
   - Required Libraries: `Adafruit_VL53L0X`, `Adafruit_MPU6050`, `Adafruit_Sensor`, `WebServer`.

2. **Compilation:**
   - Open `module_final/module_final.ino`.
   - Select **ESP32-S3 Dev Module** as your target board.
   - Ensure you enable dual-core features if configuring via PlatformIO.
   - Update the `ssid` and `password` variables to match your local network for the telemetry server.

3. **Operation:**
   - Place the robot at the starting cell `(0,0)`.
   - Once powered on, the robot will host a web server on its allocated IP address.
   - Trigger the `STATE_EXPLORE` phase via the dashboard or onboard button.
   - The robot will autonomously map the maze, reach the center, calculate the shortest path, and optionally execute a `FAST RUN` back to the center.

## ⚠️ Safety Mechanisms
- **Software Stall Detection:** Disengages motors if encoder ticks halt unexpectedly while PWM is active, preventing motor burnout.

---
*Developed for embedded systems research, robotics competitions, and high-performance C++ control architecture.*
