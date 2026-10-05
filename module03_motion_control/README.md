# Module 03 - Motion Control (Unified)

This folder contains the complete, unified motion control logic for the MicroMouse robot. It brings together hardware setup, encoder tracking, PID motor control, and sensor fusion into a single, cohesive `.ino` file.

## Features Implemented

*   **Single Unified Architecture:** All logic (sensors, encoders, motors, PID, networking) is consolidated into `module03_motion_control.ino`.
*   **PID Straight Drive:** Uses encoder differences (`leftPulse - rightPulse`) and fuses MPU6050 Yaw data to drive the robot in a perfectly straight line.
*   **PID Wall Following / Cell Centering:** Uses Left and Right ToF sensors to automatically center the robot in the maze cell. If the robot drifts too close to one wall, it automatically corrects its trajectory.
*   **Web Dashboard Interface:** Hosts an internal Web Server that serves a modern, dark-themed UI.
*   **Live Sensor Feed:** The dashboard live-streams ToF distances and Gyro Yaw angle.
*   **Remote Commands:** You can issue "MOVE 1 CELL" or "EMERGENCY STOP" commands directly from your phone/PC browser.
*   **Station Mode WiFi:** The ESP32 connects to an existing WiFi Hotspot (e.g., your phone) instead of broadcasting its own, preventing IP conflicts and making it easier to manage.

## Pin Mapping

*   **Motors:**
    *   Left Motor: `PWMA=4`, `AIN1=5`, `AIN2=6`
    *   Right Motor: `PWMB=18`, `BIN1=15`, `BIN2=14`
    *   Standby: `STBY=17`
*   **Encoders (A channels):** `LEFT_ENC_A=9`, `RIGHT_ENC_A=36`
*   **I2C Bus:** `SDA=40`, `SCL=2`
*   **VL53L0X ToF Sensors (XSHUT):**
    *   Front1: `10` *(Temporarily disabled in code if unplugged)*
    *   Front2: `12`
    *   Left: `11`
    *   Right: `13`
*   **I2C Addresses:** 
    *   Front1: `0x30`, Front2: `0x31`, Left: `0x32`, Right: `0x33`

## Calibration Values

*   **Pulses Per Cell:** `450` (Encoder pulses required to move one 18cm cell)
*   **Target Wall Distance:** `90.0 mm`

## How to Use

1.  **Turn on your Hotspot:** Ensure your phone or router's hotspot is named **`Tecno Spark`** with the password **`20240869kp`**. (Change these in the code if needed).
2.  **Upload the Code:** Upload `module03_motion_control.ino` to the ESP32-S3.
3.  **Open Serial Monitor:** Keep the Serial Monitor open (Baud: 115200). The ESP32 requires the Serial connection to open before it starts executing the setup process (due to Native USB).
4.  **Find the IP:** Watch the Serial Monitor. Once it connects to your WiFi, it will print an IP address (e.g., `192.168.43.153`).
5.  **Open Dashboard:** Ensure your laptop/phone is connected to the same `Tecno Spark` network. Open a web browser and enter the printed IP address.
6.  **Control the Robot:** Use the dashboard buttons to trigger cell movements or emergency stops!
