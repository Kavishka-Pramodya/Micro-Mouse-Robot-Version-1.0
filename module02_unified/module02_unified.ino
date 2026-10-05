/*
====================================================
MODULE 02 UNIFIED (Testing Phase)
Combined Sensor System, Calibration & Web Dashboard

Pin Mapping:
Motors:
- PWMA: 4
- AIN1: 5
- AIN2: 6
- PWMB: 7
- BIN1: 15
- BIN2: 14
- STBY: 17

Encoders:
- Left_ENC_A: 9
- Right_ENC_A: 36

I2C Bus:
- SDA: 41
- SCL: 40

ToF (VL53L0X) XSHUT:
- Front1: 10
- Front2: 11
- Left: 12
- Right: 13
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
// PIN DEFINITIONS
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

//==================================================
// GLOBAL VARIABLES
//==================================================
volatile long leftPulse = 0;
volatile long rightPulse = 0;
int leftPWM = 180;
int rightPWM = 180;
int PULSES_PER_CELL = 388;

int distFront1 = 0;
int distFront2 = 0;
int distLeft = 0;
int distRight = 0;

float yawAngle = 0.0;
float gyroZoffset = 0.0;
unsigned long lastMPUTime = 0;

//==================================================
// OBJECTS
//==================================================
Adafruit_VL53L0X tofFront1 = Adafruit_VL53L0X();
Adafruit_VL53L0X tofFront2 = Adafruit_VL53L0X();
Adafruit_VL53L0X tofLeft   = Adafruit_VL53L0X();
Adafruit_VL53L0X tofRight  = Adafruit_VL53L0X();

Adafruit_MPU6050 mpu;
WebServer server(80);

const char* ssid = "Tecno Spark";
const char* password = "20240869kp"; // Open AP

//==================================================
// WEB DASHBOARD HTML
//==================================================
const char htmlPage[] PROGMEM = R"=====(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>MicroMouse Dashboard</title>
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
      margin-top: 40px;
      font-weight: 800;
      font-size: 2.5rem;
      background: -webkit-linear-gradient(45deg, #38bdf8, #818cf8);
      -webkit-background-clip: text;
      -webkit-text-fill-color: transparent;
    }
    .grid {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 20px;
      margin-top: 30px;
      width: 90%;
      max-width: 600px;
    }
    .card {
      background: rgba(255, 255, 255, 0.05);
      backdrop-filter: blur(10px);
      border: 1px solid rgba(255, 255, 255, 0.1);
      border-radius: 16px;
      padding: 30px 20px;
      text-align: center;
      transition: transform 0.3s ease, box-shadow 0.3s ease;
    }
    .card:hover {
      transform: translateY(-5px);
      box-shadow: 0 10px 20px rgba(0,0,0,0.3);
    }
    .card h2 {
      margin: 0 0 10px 0;
      font-size: 1.2rem;
      color: #94a3b8;
      font-weight: 600;
    }
    .card .value {
      font-size: 2.5rem;
      font-weight: 800;
      color: #f8fafc;
    }
    .yaw-card {
      grid-column: span 2;
      background: linear-gradient(135deg, rgba(56, 189, 248, 0.1), rgba(129, 140, 248, 0.1));
    }
  </style>
</head>
<body>
  <h1>Live Diagnostics</h1>
  <div class="grid">
    <div class="card"><h2>Front 1</h2><div class="value" id="f1">-- mm</div></div>
    <div class="card"><h2>Front 2</h2><div class="value" id="f2">-- mm</div></div>
    <div class="card"><h2>Left</h2><div class="value" id="l">-- mm</div></div>
    <div class="card"><h2>Right</h2><div class="value" id="r">-- mm</div></div>
    <div class="card yaw-card"><h2>Yaw Angle</h2><div class="value" id="y">-- &deg;</div></div>
  </div>

  <script>
    async function fetchData() {
      try {
        const response = await fetch('/data');
        const data = await response.json();
        document.getElementById('f1').innerText = data.f1 + ' mm';
        document.getElementById('f2').innerText = data.f2 + ' mm';
        document.getElementById('l').innerText = data.l + ' mm';
        document.getElementById('r').innerText = data.r + ' mm';
        document.getElementById('y').innerHTML = parseFloat(data.y).toFixed(2) + ' &deg;';
      } catch (error) {
        console.error("Fetch error:", error);
      }
    }
    setInterval(fetchData, 200); // Poll every 200ms
  </script>
</body>
</html>
)=====";

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

void motorStop() {
  ledcWrite(PWMA, 0);
  ledcWrite(PWMB, 0);
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

  digitalWrite(XSHUT_FRONT1, HIGH); delay(10);
  tofFront1.begin(ADDR_FRONT1, false, &Wire);

  digitalWrite(XSHUT_FRONT2, HIGH); delay(10);
  tofFront2.begin(ADDR_FRONT2, false, &Wire);

  digitalWrite(XSHUT_LEFT, HIGH); delay(10);
  tofLeft.begin(ADDR_LEFT, false, &Wire);

  digitalWrite(XSHUT_RIGHT, HIGH); delay(10);
  tofRight.begin(ADDR_RIGHT, false, &Wire);
}

void readToF() {
  VL53L0X_RangingMeasurementData_t m;
  
  tofFront1.rangingTest(&m, false);
  distFront1 = (m.RangeStatus != 4) ? m.RangeMilliMeter : -1;

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
// WEB SERVER HANDLERS
//==================================================
void handleRoot() {
  server.send(200, "text/html", htmlPage);
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
// SETUP
//==================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  // 1. I2C Init
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  // 2. Hardware Init
  setupMotors();
  setupEncoders();
  motorStop();
  setupToF();
  setupMPU();

  // 3. WiFi Access Point Init
  Serial.println("Starting Wi-Fi AP...");
  WiFi.softAP(ssid, password);
  Serial.print("Dashboard IP: ");
  Serial.println(WiFi.softAPIP());

  // 4. Web Server Init
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();
  
  Serial.println("\n--- UNIFIED MODULE 02 READY ---");
}

//==================================================
// LOOP
//==================================================
void loop() {
  // 1. Non-blocking Web Server Handler
  server.handleClient();

  // 2. Continuous IMU Yaw Integration
  updateYaw();

  // 3. Non-blocking ToF Polling & Serial Print (every 500ms)
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    readToF();
    
    Serial.print("Front1 = "); Serial.print(distFront1); Serial.println(" mm");
    Serial.print("Front2 = "); Serial.print(distFront2); Serial.println(" mm");
    Serial.print("Left = ");   Serial.print(distLeft);   Serial.println(" mm");
    Serial.print("Right = ");  Serial.print(distRight);  Serial.println(" mm");
    Serial.print("Yaw = ");    Serial.print(yawAngle, 2); Serial.println("°");
    Serial.println("--------------------");

    lastPrint = millis();
  }
}
