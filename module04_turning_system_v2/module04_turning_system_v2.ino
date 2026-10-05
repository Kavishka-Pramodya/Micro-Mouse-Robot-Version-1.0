/*
====================================================
MODULE 04 - TURNING SYSTEM (V2)
====================================================
Features:
- Web Dashboard
- MPU6050 precise Gyro turning (Pivot & Smooth)
- ToF sensor integration
- Pin Mapping exactly as per Module 03 README
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
// PIN DEFINITIONS (From Module 03 README)
//==================================================
// Motors
#define PWMA 4
#define AIN1 5
#define AIN2 6
#define PWMB 18
#define BIN1 15
#define BIN2 14
#define STBY 17

// Encoders
#define LEFT_ENC_A 9
#define RIGHT_ENC_A 36

// I2C
#define I2C_SDA 40
#define I2C_SCL 2

// ToF Sensors (XSHUT)
#define XSHUT_FRONT1 10
#define XSHUT_FRONT2 12
#define XSHUT_LEFT   11
#define XSHUT_RIGHT  13

// I2C Addresses
#define ADDR_FRONT1 0x30
#define ADDR_FRONT2 0x31
#define ADDR_LEFT   0x32
#define ADDR_RIGHT  0x33

//==================================================
// GLOBAL VARIABLES
//==================================================
float yawAngle = 0.0;
float gyroZoffset = 0.0;
unsigned long lastMPUTime = 0;

int distFront1 = -1;
int distFront2 = -1;
int distLeft = -1;
int distRight = -1;

String currentStatus = "Ready";

// Web Server Commands
bool cmdTurn90L_Pivot = false;
bool cmdTurn90R_Pivot = false;
bool cmdTurn180 = false;
bool cmdSmoothTurnL = false;
bool cmdSmoothTurnR = false;
bool cmdStop = false;

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
  <title>Module 04 - Turning System</title>
  <style>
    @import url('https://fonts.googleapis.com/css2?family=Inter:wght@400;600;800&display=swap');
    body { margin: 0; padding: 20px; background: #0f172a; color: #f8fafc; font-family: 'Inter', sans-serif; text-align: center; }
    h1 { color: #38bdf8; }
    .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; max-width: 400px; margin: 0 auto; }
    .card { background: rgba(255,255,255,0.05); padding: 15px; border-radius: 8px; border: 1px solid rgba(255,255,255,0.1); }
    .value { font-size: 1.5rem; font-weight: bold; color: #fbbf24; }
    .btn { padding: 15px; margin: 5px; border: none; border-radius: 8px; font-weight: bold; font-size: 1rem; cursor: pointer; color: white; width: 100%; }
    .btn-pivot { background: #6366f1; }
    .btn-smooth { background: #10b981; }
    .btn-stop { background: #ef4444; grid-column: span 2; }
    .controls { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; max-width: 400px; margin: 20px auto; }
  </style>
</head>
<body>
  <h1>Module 04 - Turning System</h1>
  <div class="grid">
    <div class="card">Left Wall<br><span class="value" id="l">--</span> mm</div>
    <div class="card">Right Wall<br><span class="value" id="r">--</span> mm</div>
    <div class="card">Front Wall<br><span class="value" id="f2">--</span> mm</div>
    <div class="card">Yaw Angle<br><span class="value" id="y">--</span> &deg;</div>
  </div>
  <h3 id="status" style="color: #a7f3d0;">Status: Ready</h3>
  
  <div class="controls">
    <button class="btn btn-pivot" onclick="sendCmd('turn90L')">Pivot 90&deg; L</button>
    <button class="btn btn-pivot" onclick="sendCmd('turn90R')">Pivot 90&deg; R</button>
    <button class="btn btn-smooth" onclick="sendCmd('smoothL')">Smooth 90&deg; L</button>
    <button class="btn btn-smooth" onclick="sendCmd('smoothR')">Smooth 90&deg; R</button>
    <button class="btn btn-pivot" style="grid-column: span 2;" onclick="sendCmd('turn180')">U-Turn 180&deg;</button>
    <button class="btn btn-stop" onclick="sendCmd('stop')">STOP</button>
  </div>

  <script>
    async function sendCmd(cmd) { fetch('/' + cmd); }
    async function fetchData() {
      try {
        const res = await fetch('/data');
        const data = await res.json();
        document.getElementById('l').innerText = data.l;
        document.getElementById('r').innerText = data.r;
        document.getElementById('f2').innerText = data.f2;
        document.getElementById('y').innerText = parseFloat(data.y).toFixed(1);
        document.getElementById('status').innerText = "Status: " + data.s;
      } catch (e) {}
    }
    setInterval(fetchData, 300);
  </script>
</body>
</html>
)=====";

//==================================================
// MOTORS
//==================================================
void setupMotors() {
    pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
    pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);
    ledcAttach(PWMA, 1000, 8); ledcAttach(PWMB, 1000, 8);
}
void motorsStop() {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
    ledcWrite(PWMA, 0); ledcWrite(PWMB, 0);
}
void leftMotor(int pwm) {
    pwm = constrain(pwm, -255, 255);
    if (pwm > 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); ledcWrite(PWMA, pwm); }
    else if (pwm < 0) { digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); ledcWrite(PWMA, -pwm); }
    else { digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW); ledcWrite(PWMA, 0); }
}
void rightMotor(int pwm) {
    pwm = constrain(pwm, -255, 255);
    if (pwm > 0) { digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); ledcWrite(PWMB, pwm); }
    else if (pwm < 0) { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); ledcWrite(PWMB, -pwm); }
    else { digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW); ledcWrite(PWMB, 0); }
}

//==================================================
// SENSORS
//==================================================
void setupToF() {
    pinMode(XSHUT_FRONT1, OUTPUT); pinMode(XSHUT_FRONT2, OUTPUT);
    pinMode(XSHUT_LEFT, OUTPUT); pinMode(XSHUT_RIGHT, OUTPUT);
    digitalWrite(XSHUT_FRONT1, LOW); digitalWrite(XSHUT_FRONT2, LOW);
    digitalWrite(XSHUT_LEFT, LOW); digitalWrite(XSHUT_RIGHT, LOW);
    delay(10);

    digitalWrite(XSHUT_FRONT2, HIGH); delay(10);
    tofFront2.begin(ADDR_FRONT2, false, &Wire);
    digitalWrite(XSHUT_LEFT, HIGH); delay(10);
    tofLeft.begin(ADDR_LEFT, false, &Wire);
    digitalWrite(XSHUT_RIGHT, HIGH); delay(10);
    tofRight.begin(ADDR_RIGHT, false, &Wire);
}

void readToF() {
    VL53L0X_RangingMeasurementData_t m;
    
    memset(&m, 0, sizeof(m));
    tofFront2.rangingTest(&m, false);
    distFront2 = (m.RangeStatus != 4 && m.RangeMilliMeter != 0) ? m.RangeMilliMeter : -1;

    memset(&m, 0, sizeof(m));
    tofLeft.rangingTest(&m, false);
    distLeft = (m.RangeStatus != 4 && m.RangeMilliMeter != 0) ? m.RangeMilliMeter : -1;

    memset(&m, 0, sizeof(m));
    tofRight.rangingTest(&m, false);
    distRight = (m.RangeStatus != 4 && m.RangeMilliMeter != 0) ? m.RangeMilliMeter : -1;
}

void setupMPU() {
    if (mpu.begin()) {
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
        
        Serial.println("Calibrating IMU...");
        delay(1000);
        float zSum = 0; int samples = 200;
        sensors_event_t a, g, t;
        for (int i = 0; i < samples; i++) { mpu.getEvent(&a, &g, &t); zSum += g.gyro.z; delay(5); }
        gyroZoffset = zSum / samples;
        lastMPUTime = millis(); yawAngle = 0.0;
    }
}

void updateYaw() {
    sensors_event_t a, g, t; mpu.getEvent(&a, &g, &t);
    unsigned long now = millis(); float dt = (now - lastMPUTime) / 1000.0; lastMPUTime = now;
    float zRate = g.gyro.z - gyroZoffset;
    if (abs(zRate) < 0.03) zRate = 0.0;
    yawAngle += (zRate * 57.2958) * dt; 
}

//==================================================
// TURNING LOGIC
//==================================================
void pivotTurn(float targetAngle) {
    currentStatus = "Pivot Turn " + String(targetAngle) + " deg";
    yawAngle = 0.0;
    float Kp = 3.0;
    
    while (true) {
        server.handleClient();
        if (cmdStop) { cmdStop = false; break; }
        
        updateYaw();
        float error = targetAngle - yawAngle;
        if (abs(error) < 1.5) break;
        
        float pOut = Kp * error;
        pOut = constrain(pOut, -180, 180);
        if (pOut > 0 && pOut < 55) pOut = 55;
        if (pOut < 0 && pOut > -55) pOut = -55;
        
        leftMotor(-pOut);
        rightMotor(pOut);
        delay(5);
    }
    motorsStop();
    currentStatus = "Ready";
}

void smoothTurn(float targetAngle) {
    currentStatus = "Smooth Turn " + String(targetAngle) + " deg";
    yawAngle = 0.0;
    float Kp = 2.5;
    
    while (true) {
        server.handleClient();
        if (cmdStop) { cmdStop = false; break; }
        
        updateYaw();
        float error = targetAngle - yawAngle;
        if (abs(error) < 2.0) break;
        
        float turnSpeed = Kp * error;
        turnSpeed = constrain(turnSpeed, -120, 120);
        
        int baseSpeed = 100; // Keep moving forward while turning
        leftMotor(baseSpeed - turnSpeed);
        rightMotor(baseSpeed + turnSpeed);
        delay(5);
    }
    motorsStop();
    currentStatus = "Ready";
}

//==================================================
// SETUP & LOOP
//==================================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Wire.begin(I2C_SDA, I2C_SCL); Wire.setClock(400000);
    setupMotors();
    setupToF();
    setupMPU();

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    Serial.print("\nConnected IP: "); Serial.println(WiFi.localIP());

    server.on("/", [](){ server.send(200, "text/html", htmlPage); });
    server.on("/turn90L", [](){ cmdTurn90L_Pivot = true; server.send(200, "text/plain", "OK"); });
    server.on("/turn90R", [](){ cmdTurn90R_Pivot = true; server.send(200, "text/plain", "OK"); });
    server.on("/turn180", [](){ cmdTurn180 = true; server.send(200, "text/plain", "OK"); });
    server.on("/smoothL", [](){ cmdSmoothTurnL = true; server.send(200, "text/plain", "OK"); });
    server.on("/smoothR", [](){ cmdSmoothTurnR = true; server.send(200, "text/plain", "OK"); });
    server.on("/stop", [](){ cmdStop = true; server.send(200, "text/plain", "OK"); });
    server.on("/data", [](){
        String json = "{\"f2\":" + String(distFront2) + ",\"l\":" + String(distLeft) + ",\"r\":" + String(distRight) + ",\"y\":" + String(yawAngle, 2) + ",\"s\":\"" + currentStatus + "\"}";
        server.send(200, "application/json", json);
    });
    server.begin();
}

void loop() {
    server.handleClient();
    
    static unsigned long lastToF = 0;
    if (millis() - lastToF > 200) { readToF(); lastToF = millis(); }

    if (cmdTurn90L_Pivot) { cmdTurn90L_Pivot = false; pivotTurn(90.0); }
    if (cmdTurn90R_Pivot) { cmdTurn90R_Pivot = false; pivotTurn(-90.0); }
    if (cmdTurn180) { cmdTurn180 = false; pivotTurn(180.0); }
    if (cmdSmoothTurnL) { cmdSmoothTurnL = false; smoothTurn(90.0); }
    if (cmdSmoothTurnR) { cmdSmoothTurnR = false; smoothTurn(-90.0); }
    if (cmdStop) { cmdStop = false; motorsStop(); }

    updateYaw();
    delay(10);
}
