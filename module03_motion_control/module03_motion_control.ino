/*
====================================================
MODULE 03 - MOTION CONTROL (Unified Single File)
WITH WEB DASHBOARD
====================================================
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <WebServer.h>

//==================================================
// PIN DEFINITIONS & CALIBRATION (Extracted from Mod 02)
//==================================================
#define PWMA 4
#define AIN1 5
#define AIN2 6
#define PWMB 18
#define BIN1 15
#define BIN2 14
#define STBY 17

#define LEFT_ENC_A 9
#define RIGHT_ENC_A 36

#define I2C_SDA 40
#define I2C_SCL 2

#define XSHUT_FRONT1 10
#define XSHUT_FRONT2 12
#define XSHUT_LEFT   11
#define XSHUT_RIGHT  13

#define ADDR_FRONT1 0x30
#define ADDR_FRONT2 0x31
#define ADDR_LEFT   0x32
#define ADDR_RIGHT  0x33

const int PULSES_PER_CELL = 388;
const float TARGET_WALL_DIST = 90.0; // Target distance from walls in mm

//==================================================
// GLOBAL VARIABLES
//==================================================
volatile long leftPulse = 0;
volatile long rightPulse = 0;

int distFront1 = -1;
int distFront2 = -1;
int distLeft = -1;
int distRight = -1;

float yawAngle = 0.0;
float gyroZoffset = 0.0;
unsigned long lastMPUTime = 0;

const int BASE_PWM = 130; // Base speed for motors

// PID Settings - TUNED FOR SMOOTHER MOVEMENT
float Kp_straight = 1.0;
float Ki_straight = 0.0;
float Kd_straight = 0.5;

// ToF sensors are very noisy. High Kd amplifies this noise, causing violent shaking.
float Kp_wall = 0.5;  // Reduced Kp
float Ki_wall = 0.0;
float Kd_wall = 0.0;  // Set Kd to 0 to prevent derivative kick from sensor noise

// PID Variables
float prevError_straight = 0;
float integral_straight = 0;
unsigned long lastTime_straight = 0;

float prevError_wall = 0;
float integral_wall = 0;
unsigned long lastTime_wall = 0;

// Web Server Commands
bool commandMove = false;
bool commandStop = false;

//==================================================
// OBJECTS & WIFI
//==================================================
Adafruit_VL53L0X tofFront1 = Adafruit_VL53L0X();
Adafruit_VL53L0X tofFront2 = Adafruit_VL53L0X();
Adafruit_VL53L0X tofLeft   = Adafruit_VL53L0X();
Adafruit_VL53L0X tofRight  = Adafruit_VL53L0X();
Adafruit_MPU6050 mpu;

WebServer server(80);
const char* ssid = "Tecno Spark";
const char* password = "20240869kp";

//==================================================
// WEB DASHBOARD HTML
//==================================================
const char htmlPage[] PROGMEM = R"=====(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>MicroMouse Motion Control</title>
  <style>
    @import url('https://fonts.googleapis.com/css2?family=Inter:wght@400;600;800&display=swap');
    body {
      margin: 0;
      padding: 0;
      background: linear-gradient(135deg, #0f172a 0%, #1e293b 100%);
      color: #f8fafc;
      font-family: 'Inter', sans-serif;
      display: flex;
      flex-direction: column;
      align-items: center;
      min-height: 100vh;
    }
    h1 {
      margin-top: 20px;
      font-weight: 800;
      font-size: 2rem;
      text-align: center;
      background: -webkit-linear-gradient(45deg, #38bdf8, #818cf8);
      -webkit-background-clip: text;
      -webkit-text-fill-color: transparent;
    }
    .grid {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 15px;
      width: 90%;
      max-width: 400px;
      margin-top: 20px;
    }
    .card {
      background: rgba(255, 255, 255, 0.05);
      border: 1px solid rgba(255, 255, 255, 0.1);
      border-radius: 12px;
      padding: 15px;
      text-align: center;
    }
    .card h2 { margin: 0 0 5px 0; font-size: 1rem; color: #94a3b8; }
    .card .value { font-size: 1.8rem; font-weight: 800; }
    .yaw-card { grid-column: span 2; background: linear-gradient(135deg, rgba(56, 189, 248, 0.1), rgba(129, 140, 248, 0.1)); }
    
    .button-container {
      display: flex;
      flex-direction: column;
      gap: 15px;
      margin-top: 30px;
      width: 90%;
      max-width: 400px;
    }
    button {
      padding: 15px;
      font-size: 1.2rem;
      font-weight: 800;
      border: none;
      border-radius: 12px;
      cursor: pointer;
    }
    .btn-move { background: linear-gradient(135deg, #10b981 0%, #059669 100%); color: white; }
    .btn-stop { background: linear-gradient(135deg, #ef4444 0%, #dc2626 100%); color: white; }
    .status { margin-top: 20px; margin-bottom: 30px; font-size: 1rem; color: #94a3b8; }
  </style>
</head>
<body>
  <h1>MicroMouse Dashboard</h1>
  
  <div class="grid">
    <div class="card"><h2>Left (mm)</h2><div class="value" id="l">--</div></div>
    <div class="card"><h2>Right (mm)</h2><div class="value" id="r">--</div></div>
    <div class="card"><h2>Front1 (mm)</h2><div class="value" id="f1">--</div></div>
    <div class="card"><h2>Front2 (mm)</h2><div class="value" id="f2">--</div></div>
    <div class="card yaw-card"><h2>Yaw Angle</h2><div class="value" id="y">-- &deg;</div></div>
  </div>

  <div class="button-container">
    <button class="btn-move" onclick="sendCommand('move')">MOVE 1 CELL</button>
    <button class="btn-stop" onclick="sendCommand('stop')">EMERGENCY STOP</button>
  </div>

  <div class="status" id="status-text">Status: Ready</div>

  <script>
    async function sendCommand(cmd) {
      document.getElementById('status-text').innerText = "Status: Sending...";
      try {
        const response = await fetch('/' + cmd);
        const text = await response.text();
        document.getElementById('status-text').innerText = "Status: " + text;
      } catch (error) {
        document.getElementById('status-text').innerText = "Status: Error";
      }
    }

    async function fetchData() {
      try {
        const response = await fetch('/data');
        const data = await response.json();
        document.getElementById('f1').innerText = data.f1;
        document.getElementById('f2').innerText = data.f2;
        document.getElementById('l').innerText = data.l;
        document.getElementById('r').innerText = data.r;
        document.getElementById('y').innerHTML = parseFloat(data.y).toFixed(2) + ' &deg;';
      } catch (error) {}
    }
    setInterval(fetchData, 300);
  </script>
</body>
</html>
)=====";

//==================================================
// WEB SERVER HANDLERS
//==================================================
void handleRoot() {
    server.send(200, "text/html", htmlPage);
}

void handleMove() {
    commandMove = true; 
    server.send(200, "text/plain", "Moving 1 Cell...");
}

void handleStop() {
    commandStop = true;
    server.send(200, "text/plain", "Stopped.");
}

void handleData() {
    String json = "{";
    json += "\"f1\":" + String(distFront1) + ",";
    json += "\"f2\":" + String(distFront2) + ",";
    json += "\"l\":" + String(distLeft) + ",";
    json += "\"r\":" + String(distRight) + ",";
    json += "\"y\":" + String(yawAngle, 2);
    json += "}";
    server.send(200, "application/json", json);
}

//==================================================
// ENCODER & MOTOR LOGIC
//==================================================
void IRAM_ATTR leftEncoderISR() { leftPulse++; }
void IRAM_ATTR rightEncoderISR() { rightPulse++; }

void setupMotors() {
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(STBY, OUTPUT);
    digitalWrite(STBY, HIGH);
    
    ledcAttach(PWMA, 1000, 8);
    ledcAttach(PWMB, 1000, 8);
}

void setupEncoders() {
    pinMode(LEFT_ENC_A, INPUT_PULLUP);
    pinMode(RIGHT_ENC_A, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);
}

void motorsStop() {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    ledcWrite(PWMA, 0);
    ledcWrite(PWMB, 0);
}

void leftMotor(int pwm) {
    pwm = constrain(pwm, -255, 255);
    if (pwm > 0) {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
        ledcWrite(PWMA, pwm);
    } else if (pwm < 0) {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        ledcWrite(PWMA, -pwm);
    } else {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, LOW);
        ledcWrite(PWMA, 0);
    }
}

void rightMotor(int pwm) {
    pwm = constrain(pwm, -255, 255);
    if (pwm > 0) {
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
        ledcWrite(PWMB, pwm);
    } else if (pwm < 0) {
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
        ledcWrite(PWMB, -pwm);
    } else {
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, LOW);
        ledcWrite(PWMB, 0);
    }
}

//==================================================
// SENSOR LOGIC (ToF)
//==================================================
void setupToF() {
    pinMode(XSHUT_FRONT1, OUTPUT);
    pinMode(XSHUT_FRONT2, OUTPUT);
    pinMode(XSHUT_LEFT, OUTPUT);
    pinMode(XSHUT_RIGHT, OUTPUT);

    digitalWrite(XSHUT_FRONT1, LOW);
    digitalWrite(XSHUT_FRONT2, LOW);
    digitalWrite(XSHUT_LEFT, LOW);
    digitalWrite(XSHUT_RIGHT, LOW);
    delay(10);

    // Front 1 is unplugged, disabled to prevent hanging
    // digitalWrite(XSHUT_FRONT1, HIGH); delay(10);
    // if (!tofFront1.begin(ADDR_FRONT1, false, &Wire)) Serial.println("Error F1");

    digitalWrite(XSHUT_FRONT2, HIGH); delay(10);
    if (!tofFront2.begin(ADDR_FRONT2, false, &Wire)) Serial.println("Error F2");

    digitalWrite(XSHUT_LEFT, HIGH); delay(10);
    if (!tofLeft.begin(ADDR_LEFT, false, &Wire)) Serial.println("Error L");

    digitalWrite(XSHUT_RIGHT, HIGH); delay(10);
    if (!tofRight.begin(ADDR_RIGHT, false, &Wire)) Serial.println("Error R");
}

void readToF() {
    VL53L0X_RangingMeasurementData_t m;
    
    // Front 1 is unplugged
    // tofFront1.rangingTest(&m, false);
    // distFront1 = (m.RangeStatus != 4) ? m.RangeMilliMeter : -1;

    tofFront2.rangingTest(&m, false);
    distFront2 = (m.RangeStatus != 4) ? m.RangeMilliMeter : -1;

    tofLeft.rangingTest(&m, false);
    distLeft = (m.RangeStatus != 4) ? m.RangeMilliMeter : -1;

    tofRight.rangingTest(&m, false);
    distRight = (m.RangeStatus != 4) ? m.RangeMilliMeter : -1;
}

//==================================================
// SENSOR LOGIC (MPU6050)
//==================================================
void calibrateMPU() {
    Serial.println("Calibrating IMU... Keep still.");
    delay(1000);
    float zSum = 0;
    int samples = 200;
    sensors_event_t a, g, t;
    for (int i = 0; i < samples; i++) {
        mpu.getEvent(&a, &g, &t);
        zSum += g.gyro.z;
        delay(5);
    }
    gyroZoffset = zSum / samples;
    lastMPUTime = millis();
    yawAngle = 0.0;
}

void setupMPU() {
    if (mpu.begin()) {
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
        calibrateMPU();
    }
}

void updateYaw() {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    
    unsigned long now = millis();
    float dt = (now - lastMPUTime) / 1000.0;
    lastMPUTime = now;
    
    float zRate = g.gyro.z - gyroZoffset;
    if (abs(zRate) < 0.03) zRate = 0.0;
    
    yawAngle += (zRate * 57.2958) * dt; 
}

//==================================================
// MOTION CONTROL ALGORITHMS
//==================================================
void driveStraight() {
    // Standard: error > 0 means Steer RIGHT. 
    // If rightPulse > leftPulse, robot is curving LEFT -> We need to steer RIGHT. So error = rightPulse - leftPulse
    float error = rightPulse - leftPulse;
    
    updateYaw();
    // If yawAngle > 0, robot turned LEFT -> We need to steer RIGHT -> error needs to increase.
    error += yawAngle * 10; 
    
    unsigned long now = millis();
    float dt = (now - lastTime_straight) / 1000.0;
    if (dt <= 0.001) dt = 0.05; // Fallback
    lastTime_straight = now;
    
    integral_straight += error * dt;
    float derivative = (error - prevError_straight) / dt;
    
    // Limit integral
    integral_straight = constrain(integral_straight, -100, 100);
    
    float correction = (Kp_straight * error) + (Ki_straight * integral_straight) + (Kd_straight * derivative);
    
    // Prevent massive corrections that cause spinning
    correction = constrain(correction, -60, 60);
    
    prevError_straight = error;

    // +correction means Steer RIGHT (Left motor speeds up, Right motor slows down)
    int currentLeftPWM = constrain(BASE_PWM + correction, 0, 255);
    int currentRightPWM = constrain(BASE_PWM - correction, 0, 255);

    leftMotor(currentLeftPWM);
    rightMotor(currentRightPWM);
}

void followWalls() {
    readToF();
    
    bool leftWall = (distLeft > 0 && distLeft < 150);
    bool rightWall = (distRight > 0 && distRight < 150);
    
    float error = 0;
    
    if (leftWall && rightWall) {
        // If distRight < distLeft, robot is closer to RIGHT wall.
        // We must steer LEFT. 
        // If error > 0, the code below steers RIGHT (Left wheel gets +, Right wheel gets -).
        // Therefore, if closer to RIGHT, error must be < 0.
        // error = distRight - distLeft will be negative when closer to right wall.
        error = distRight - distLeft; 
    } else if (leftWall) {
        // Closer to left wall -> distLeft < TARGET -> error > 0 -> steers RIGHT
        error = TARGET_WALL_DIST - distLeft;
    } else if (rightWall) {
        // Closer to right wall -> distRight < TARGET -> error < 0 -> steers LEFT
        error = distRight - TARGET_WALL_DIST; 
    } else {
        driveStraight();
        return;
    }
    
    unsigned long now = millis();
    float dt = (now - lastTime_wall) / 1000.0;
    if (dt <= 0.001) dt = 0.05; // Fallback
    lastTime_wall = now;
    
    integral_wall += error * dt;
    float derivative = (error - prevError_wall) / dt;
    
    // Limit integral to prevent windup
    integral_wall = constrain(integral_wall, -100, 100);
    
    float correction = (Kp_wall * error) + (Ki_wall * integral_wall) + (Kd_wall * derivative);
    
    // Prevent massive corrections that cause spinning
    correction = constrain(correction, -60, 60);
    
    prevError_wall = error;

    // BASE_PWM + correction for Left, BASE_PWM - correction for Right
    // error > 0 => correction > 0 => Left speeds up, Right slows down => Steers RIGHT
    int currentLeftPWM = constrain(BASE_PWM + correction, 0, 255);
    int currentRightPWM = constrain(BASE_PWM - correction, 0, 255);

    leftMotor(currentLeftPWM);
    rightMotor(currentRightPWM);
}

void moveOneCell() {
    noInterrupts();
    leftPulse = 0;
    rightPulse = 0;
    interrupts();
    
    yawAngle = 0.0; 
    
    while (true) {
        // If an emergency stop command comes in while moving, break out
        server.handleClient();
        if (commandStop) {
            commandStop = false;
            break;
        }

        long avgPulse = (leftPulse + rightPulse) / 2;
        if (avgPulse >= PULSES_PER_CELL) {
            break;
        }
        
        followWalls(); 
        // Removed delay(50) here! The sensor reading in followWalls() 
        // naturally provides the loop delay (~60-90ms).
        // Removing the extra delay makes the robot react MUCH faster.
    }
    
    motorsStop();
}

//==================================================
// SETUP
//==================================================
void setup() {
    Serial.begin(115200);
    
    // Wait for Serial Monitor to connect (Native USB on ESP32-S3)
    // We wait up to 5 seconds so it doesn't block forever if not connected
    unsigned long startWait = millis();
    while (!Serial && millis() - startWait < 5000) {
        delay(10);
    }
    delay(1000); 

    Serial.println("\n\n=========================================");
    Serial.println("--- STARTING MODULE 03 SETUP ---");
    Serial.println("=========================================\n");
    
    Serial.println("[1/7] Initializing I2C...");
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000); // 400kHz
    
    // Small delay to let I2C stabilize
    delay(100);
    Serial.println("      I2C Initialized.");

    Serial.println("[2/7] Setting up motors...");
    setupMotors();
    motorsStop();
    Serial.println("      Motors setup complete.");

    Serial.println("[3/7] Setting up encoders...");
    setupEncoders();
    Serial.println("      Encoders setup complete.");

    Serial.println("[4/7] Setting up ToF sensors...");
    setupToF();
    Serial.println("      ToF sensors setup complete.");

    Serial.println("[5/7] Setting up MPU6050...");
    setupMPU();
    Serial.println("      MPU6050 setup complete.");

    Serial.println("[6/7] Connecting to Wi-Fi...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    
    Serial.println();
    Serial.print("      Connected! Dashboard IP: ");
    Serial.println(WiFi.localIP());

    Serial.println("[7/7] Starting Web Server...");
    server.on("/", handleRoot);
    server.on("/move", handleMove);
    server.on("/stop", handleStop);
    server.on("/data", handleData);
    server.begin();
    Serial.println("      Web Server running.");

    Serial.println("\n--- MODULE 03 READY ---");
    Serial.println("Go to the Dashboard IP to control the robot.");
}

//==================================================
// LOOP
//==================================================
void loop() {
    // 1. Listen for Web Server clients
    server.handleClient();
    
    // 2. Read Sensors Periodically for the Dashboard
    static unsigned long lastToF = 0;
    if (millis() - lastToF > 200) {
        readToF();
        lastToF = millis();
    }

    // 2. Execute pending web commands safely outside of the interrupt/handler context
    if (commandMove) {
        commandMove = false;
        Serial.println("Moving 1 Cell (Web Command)...");
        moveOneCell();
        Serial.println("Done.");
    }

    if (commandStop) {
        commandStop = false;
        motorsStop();
        Serial.println("Emergency Stop Executed.");
    }

    // 3. Fallback Serial Commands
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'm') {
            Serial.println("Moving 1 Cell (Serial Command)...");
            moveOneCell();
            Serial.println("Done.");
        } else if (c == 's') {
            motorsStop();
            Serial.println("Stopped (Serial Command).");
        }
    }
    
    updateYaw();
    delay(10);
}
