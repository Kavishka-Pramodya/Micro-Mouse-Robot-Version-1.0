/*
====================================================
MODULE 05 - MAZE EXPLORATION (MAP BUILDING)
====================================================
Features:
- Wall, Cell, Junction, Dead End Detection
- Live Maze Map Building
- Pivot Turning ONLY (as requested)
- Web Dashboard for Map View
- Pin Mapping EXACTLY from Module 03
====================================================
*/

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_VL53L0X.h>
#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>

//==================================================
// PIN DEFINITIONS (From Module 03 README)
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
#define XSHUT_LEFT 11
#define XSHUT_RIGHT 13

#define ADDR_FRONT1 0x30
#define ADDR_FRONT2 0x31
#define ADDR_LEFT 0x32
#define ADDR_RIGHT 0x33

const int PULSES_PER_CELL = 388;
int WALL_THRESHOLD = 130; // mm (dynamic)
const float TARGET_WALL_DIST = 65.0; // Perfect center derived from manual telemetry
const int BASE_PWM = 130;
float targetSideDist = 55.0; // Dynamic target side distance

// PID Settings
float Kp_straight = 1.0;
float Ki_straight = 0.0;
float Kd_straight = 0.5;

float Kp_wall = 0.5;
float Ki_wall = 0.0;
float Kd_wall = 0.0;

// PID Variables
float prevError_straight = 0;
float integral_straight = 0;
unsigned long lastTime_straight = 0;

float prevError_wall = 0;
float integral_wall = 0;
unsigned long lastTime_wall = 0;

//==================================================
// GLOBAL VARIABLES
//==================================================
volatile long leftPulse = 0;
volatile long rightPulse = 0;

int distFront1 = -1; // Using Front1
int distFront = -1;  // Using Front2
int distLeft = -1;
int distRight = -1;

float yawAngle = 0.0;
float gyroZoffset = 0.0;
unsigned long lastMPUTime = 0;

// Robot Position & Map
int rx = 0;
int ry = 0;
int rDir = 0; // 0=N, 1=E, 2=S, 3=W
const char *cellStatus = "Unknown";
const char *currentStatus = "Ready";

// Command flags
bool cmdExplore = false;
bool cmdStop = false;

Adafruit_VL53L0X tofFront1 = Adafruit_VL53L0X();
Adafruit_VL53L0X tofFront = Adafruit_VL53L0X();
Adafruit_VL53L0X tofLeft = Adafruit_VL53L0X();
Adafruit_VL53L0X tofRight = Adafruit_VL53L0X();
Adafruit_MPU6050 mpu;
WebServer server(80);

const char *ssid = "Tecno Spark";
const char *password = "20240869kp";

//==================================================
// WEB DASHBOARD HTML
//==================================================
const char htmlPage[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <title>Module 05 - Maze Explorer</title>
  <style>
    body { background: #0f172a; color: white; font-family: sans-serif; text-align: center; }
    h1 { color: #38bdf8; }
    .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; max-width: 400px; margin: 0 auto; }
    .card { background: rgba(255,255,255,0.1); padding: 10px; border-radius: 8px; }
    .value { font-size: 1.2rem; font-weight: bold; color: #fbbf24; }
    .btn { padding: 15px; border: none; border-radius: 8px; font-weight: bold; cursor: pointer; color: white; width: 90%; max-width: 400px; margin: 5px auto; display: block; }
    .btn-auto { background: #6366f1; }
    .btn-stop { background: #ef4444; }
    .map-box { background: black; padding: 20px; border: 2px solid #38bdf8; max-width: 400px; margin: 20px auto; font-family: monospace; font-size: 1.2rem;}
  </style>
</head>
<body>
  <h1>Maze Exploration</h1>
  
  <div class="grid">
    <div class="card">X, Y Position<br><span class="value" id="pos">0, 0</span></div>
    <div class="card">Direction<br><span class="value" id="dir">North</span></div>
    <div class="card" style="grid-column: span 2;">Detection<br><span class="value" id="cell">--</span></div>
  </div>

  <div class="map-box">
    Robot State:<br><br>
    Yaw Angle: <span id="yaw" style="color: #fbbf24;">0.00</span>&deg;<br><br>
    Front 1 (L): <span id="f1w">--</span> <span style="font-size:0.9rem; color:#9ca3af;">(<span id="f1_dist">--</span> mm)</span><br>
    Front 2 (R): <span id="f2w">--</span> <span style="font-size:0.9rem; color:#9ca3af;">(<span id="f2_dist">--</span> mm)</span><br>
    Left:  <span id="lw">--</span> <span style="font-size:0.9rem; color:#9ca3af;">(<span id="l_dist">--</span> mm)</span><br>
    Right: <span id="rw">--</span> <span style="font-size:0.9rem; color:#9ca3af;">(<span id="r_dist">--</span> mm)</span>
  </div>
  
  <h3 id="status" style="color: #a7f3d0;">Status: Ready</h3>
  
  <script>
    async function fetchData() {
      try {
        const res = await fetch('/data');
        const data = await res.json();
        document.getElementById('pos').innerText = data.x + ", " + data.y;
        document.getElementById('dir').innerText = data.d;
        document.getElementById('cell').innerText = data.c;
        document.getElementById('f1w').innerText = data.fw1 ? "WALL" : "OPEN";
        document.getElementById('f2w').innerText = data.fw2 ? "WALL" : "OPEN";
        document.getElementById('lw').innerText = data.lw ? "WALL" : "OPEN";
        document.getElementById('rw').innerText = data.rw ? "WALL" : "OPEN";
        document.getElementById('f1_dist').innerText = data.f1_dist;
        document.getElementById('f2_dist').innerText = data.f2_dist;
        document.getElementById('l_dist').innerText = data.l_dist;
        document.getElementById('r_dist').innerText = data.r_dist;
        document.getElementById('yaw').innerText = data.yaw.toFixed(2);
        document.getElementById('status').innerText = "Status: " + data.s;
      } catch (e) {}
    }
    setInterval(fetchData, 300);
  </script>
</body>
</html>
)=====";

//==================================================
// MOTORS & ENCODERS
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
// SENSORS
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

  digitalWrite(XSHUT_FRONT1, HIGH);
  delay(10);
  tofFront1.begin(ADDR_FRONT1, false, &Wire);

  digitalWrite(XSHUT_FRONT2, HIGH);
  delay(10);
  tofFront.begin(ADDR_FRONT2, false, &Wire);

  digitalWrite(XSHUT_LEFT, HIGH);
  delay(10);
  tofLeft.begin(ADDR_LEFT, false, &Wire);

  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(10);
  tofRight.begin(ADDR_RIGHT, false, &Wire);
}

void readToF() {
  VL53L0X_RangingMeasurementData_t m;

  memset(&m, 0, sizeof(m));
  tofFront1.rangingTest(&m, false);
  distFront1 =
      (m.RangeStatus != 4 && m.RangeMilliMeter != 0) ? m.RangeMilliMeter : -1;

  memset(&m, 0, sizeof(m));
  tofFront.rangingTest(&m, false);
  distFront =
      (m.RangeStatus != 4 && m.RangeMilliMeter != 0) ? m.RangeMilliMeter : -1;

  memset(&m, 0, sizeof(m));
  tofLeft.rangingTest(&m, false);
  distLeft =
      (m.RangeStatus != 4 && m.RangeMilliMeter != 0) ? m.RangeMilliMeter : -1;

  memset(&m, 0, sizeof(m));
  tofRight.rangingTest(&m, false);
  distRight =
      (m.RangeStatus != 4 && m.RangeMilliMeter != 0) ? m.RangeMilliMeter : -1;
}

void setupMPU() {
  if (mpu.begin()) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
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
}

void updateYaw() {
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  unsigned long now = millis();
  float dt = (now - lastMPUTime) / 1000.0;
  lastMPUTime = now;
  float zRate = g.gyro.z - gyroZoffset;
  if (abs(zRate) < 0.03)
    zRate = 0.0;
  yawAngle += (zRate * 57.2958) * dt;
}

//==================================================
// TURNING LOGIC (PIVOT ONLY AS REQUESTED)
//==================================================
void pivotTurn(float targetAngle) {
  currentStatus = "Turning...";
  yawAngle = 0.0;
  float Kp = 3.0;
  unsigned long turnStart = millis();

  while (millis() - turnStart < 2500) {
    server.handleClient();
    if (cmdStop)
      break;

    static unsigned long lastToFTurn = 0;
    if (millis() - lastToFTurn > 33) {
      readToF();
      lastToFTurn = millis();
    }

    updateYaw();
    float error = targetAngle - yawAngle;
    if (abs(error) < 1.5)
      break;

    float pOut = Kp * error;
    pOut = constrain(pOut, -180, 180);
    if (pOut > 0 && pOut < 55)
      pOut = 55;
    if (pOut < 0 && pOut > -55)
      pOut = -55;

    leftMotor(-pOut);
    rightMotor(pOut);
    delay(5);
  }
  motorsStop();
  delay(200);
}

// Advanced Technique 1: Squaring (Front Sensor Alignment)
void alignWithFrontWall() {
  currentStatus = "Aligning...";
  float Kp_align = 1.5;
  unsigned long alignStart = millis();

  while (millis() - alignStart < 1000) {
    server.handleClient();
    if (cmdStop)
      break;

    VL53L0X_RangingMeasurementData_t m1, m2;
    tofFront1.rangingTest(&m1, false);
    tofFront.rangingTest(&m2, false);

    int d1 = (m1.RangeStatus != 4 && m1.RangeMilliMeter != 0)
                 ? m1.RangeMilliMeter
                 : -1;
    int d2 = (m2.RangeStatus != 4 && m2.RangeMilliMeter != 0)
                 ? m2.RangeMilliMeter
                 : -1;

    if (d1 > 0 && d2 > 0) {
      int error =
          d1 - d2; // Difference between left-front and right-front sensors
      if (abs(error) <= 2)
        break; // Aligned perfectly!

      int pOut = Kp_align * error;

      // Add minimum PWM to overcome friction
      if (pOut > 0 && pOut < 50)
        pOut = 50;
      if (pOut < 0 && pOut > -50)
        pOut = -50;

      pOut = constrain(pOut, -60, 60); // Gentle rotation to square up

      leftMotor(-pOut);
      rightMotor(pOut);
    } else {
      break; // Sensor read failed, abort squaring
    }
    delay(5);
  }
  motorsStop();
  delay(100);

  motorsStop();
  delay(100);
  delay(100);

  yawAngle =
      0.0; // Reset Gyro because we are perfectly aligned and distanced now
}

void moveForwardCell() {
  currentStatus = "Moving 1 Cell";
  noInterrupts();
  leftPulse = 0;
  rightPulse = 0;
  interrupts();
  yawAngle = 0.0;

  // Fast PID tuning for straight line
  float Kp_gyro = 6.0;
  float Kp_enc = 0.5;
  // Use global Kp_wall for wall following

  while (true) {
    server.handleClient();
    if (cmdStop)
      break;

    long avgPulse = (leftPulse + rightPulse) / 2;
    if (avgPulse >= PULSES_PER_CELL)
      break;

    // Read ToF sensors periodically inside the fast loop without blocking too
    // much
    static unsigned long lastToFRead = 0;
    if (millis() - lastToFRead > 20) {
      readToF();
      lastToFRead = millis();
    }

    int currentBaseSpeed = BASE_PWM;

    // Proportional Braking for Front Wall
    if (distFront > 0) {
      if (distFront <= TARGET_WALL_DIST) {
        motorsStop();         // Reached target distance, stop instantly
        // alignWithFrontWall(); // REMOVED: Sensor shoots over the low wooden planks causing infinite violent spinning!
        break;
      } else if (distFront < 180) {
        // Smoothly reduce speed as it gets closer to the wall (from 180mm down
        // to TARGET_WALL_DIST) Minimum speed set to 50 so it has enough power
        // to reach the target mark without stalling
        currentBaseSpeed = map(distFront, TARGET_WALL_DIST, 180, 50, BASE_PWM);
        currentBaseSpeed = constrain(currentBaseSpeed, 50, BASE_PWM);
      }
    }

    // VERY FAST LOOP: Only read Gyro and Encoders
    updateYaw();

    // Calculate Errors
    float gyroError = 0.0 - yawAngle;        // We want yaw to stay at 0
    float encError = rightPulse - leftPulse; // We want wheels to spin equally

    float wallError = 0.0;
    // Only follow walls that are actually side walls (close enough), not perpendicular walls in intersections!
    bool hasLeftWall = (distLeft > 0 && distLeft < (targetSideDist + 35));
    bool hasRightWall = (distRight > 0 && distRight < (targetSideDist + 35));

    if (hasLeftWall && hasRightWall) {
      wallError = distLeft - distRight; // Center between both walls
    } else if (hasLeftWall) {
      wallError = distLeft - targetSideDist; // Maintain distance from left wall
    } else if (hasRightWall) {
      wallError = targetSideDist - distRight; // Maintain distance from right wall
    }

    // Combine Gyro, Encoder, and Wall Following for 100% accurate PID
    float correction =
        (Kp_gyro * gyroError) + (Kp_enc * encError) + (Kp_wall * wallError);
    correction = constrain(correction, -60, 60);

    // Apply correction using the dynamic currentBaseSpeed for Proportional
    // Braking
    leftMotor(currentBaseSpeed - correction);
    rightMotor(currentBaseSpeed + correction);

    delay(5); // 5ms delay = 200Hz loop! Extremely fast and smooth.
  }

  // Update Virtual Map Position
  if (rDir == 0)
    ry++;
  else if (rDir == 1)
    rx++;
  else if (rDir == 2)
    ry--;
  else if (rDir == 3)
    rx--;
}

//==================================================
// MAZE EXPLORATION LOGIC
//==================================================
// Auto Calibration at Start
void autoCalibrateWalls() {
  currentStatus = "Calibrating...";
  for (int i = 0; i < 5; i++) {
    readToF();
    server.handleClient();
    delay(50);
  }

  if (distLeft > 0 && distRight > 0 && distLeft < 150 && distRight < 150) {
    targetSideDist = (distLeft + distRight) / 2.0;
    WALL_THRESHOLD = targetSideDist + 80.0; // Increased tolerance!
  } else if (distLeft > 0 && distLeft < 150) {
    targetSideDist = distLeft;
    WALL_THRESHOLD = distLeft + 80.0;
  } else if (distRight > 0 && distRight < 150) {
    targetSideDist = distRight;
    WALL_THRESHOLD = distRight + 80.0;
  }
}

void exploreStep() {
  readToF();

  bool wallF = (distFront > 0 && distFront < WALL_THRESHOLD);
  bool wallL = (distLeft > 0 && distLeft < WALL_THRESHOLD);
  bool wallR = (distRight > 0 && distRight < WALL_THRESHOLD);

  // Detect Junction Type
  if (wallF && wallL && wallR)
    cellStatus = "Dead End";
  else if (!wallF && !wallL && !wallR)
    cellStatus = "Cross Junction";
  else if (!wallF && wallL && wallR)
    cellStatus = "Straight Path";
  else
    cellStatus = "Corner / T-Junct";

  currentStatus = "Analyzing...";
  // Removed delay(300) to allow continuous movement without stopping

  // Left-Hand Rule Maze Solving
  if (!wallL) {
    pivotTurn(90.0);       // Turn Left
    rDir = (rDir + 3) % 4; // Update Virtual Direction
    moveForwardCell();
  } else if (!wallF) {
    // Go Straight
    moveForwardCell();
  } else if (!wallR) {
    pivotTurn(-90.0); // Turn Right
    rDir = (rDir + 1) % 4;
    moveForwardCell();
  } else {
    pivotTurn(180.0); // U-Turn (Dead End)
    rDir = (rDir + 2) % 4;
    moveForwardCell();
  }
}

//==================================================
// SETUP & MAIN LOOP
//==================================================
void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  setupMotors();
  setupToF();
  setupMPU();

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected!");
  Serial.print("Dashboard IP: ");
  Serial.println(WiFi.localIP());

  server.on("/", []() { server.send(200, "text/html", htmlPage); });
  server.on("/auto", []() {
    cmdExplore = true;
    cmdStop = false;
    server.send(200, "text/plain", "OK");
  });
  server.on("/stop", []() {
    cmdStop = true;
    cmdExplore = false;
    motorsStop();
    server.send(200, "text/plain", "OK");
  });

  server.on("/data", []() {
    bool wf1 = (distFront1 > 0 && distFront1 < WALL_THRESHOLD);
    bool wf2 = (distFront > 0 && distFront < WALL_THRESHOLD);
    bool wl = (distLeft > 0 && distLeft < WALL_THRESHOLD);
    bool wr = (distRight > 0 && distRight < WALL_THRESHOLD);

    String dStr = (rDir == 0)   ? "North"
                  : (rDir == 1) ? "East"
                  : (rDir == 2) ? "South"
                                : "West";

    String json =
        "{\"x\":" + String(rx) + ",\"y\":" + String(ry) + ",\"d\":\"" + dStr +
        "\",\"c\":\"" + String(cellStatus) +
        "\",\"fw1\":" + (wf1 ? "true" : "false") +
        ",\"fw2\":" + (wf2 ? "true" : "false") +
        ",\"lw\":" + (wl ? "true" : "false") +
        ",\"rw\":" + (wr ? "true" : "false") + ",\"s\":\"" +
        String(currentStatus) + "\",\"f1_dist\":" + String(distFront1) +
        ",\"f2_dist\":" + String(distFront) +
        ",\"l_dist\":" + String(distLeft) + ",\"r_dist\":" + String(distRight) +
        ",\"yaw\":" + String(yawAngle, 2) + "}";
    server.send(200, "application/json", json);
  });

  server.begin();

  Serial.println(
      "Waiting 3 seconds for perfect Gyro stabilization before Auto Start...");
  delay(3000);
  autoCalibrateWalls(); // Dynamically set thresholds based on starting position
  cmdExplore = true;    // Auto start the exploration!
}

void loop() {
  server.handleClient();

  static unsigned long lastToF = 0;
  if (millis() - lastToF > 200) {
    readToF();
    lastToF = millis();
  }

  if (cmdExplore) {
    exploreStep();
  }

  updateYaw();
  delay(10);
}
