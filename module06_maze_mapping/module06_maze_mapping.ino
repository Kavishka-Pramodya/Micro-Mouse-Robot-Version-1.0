/*
====================================================
MODULE 06 - MAZE MAPPING
====================================================
Features:
- Wall, Cell, Junction, Dead End Detection
- Live 16x16 Maze Map Building in RAM
- Real-time HTML5 Canvas Map Rendering on Dashboard
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
int WALL_THRESHOLD = 130; 
const float TARGET_WALL_DIST = 55.0; // Perfect center derived from manual telemetry
const int BASE_PWM = 130;
float targetSideDist = 55.0; 

// PID Settings
float Kp_straight = 1.0;
float Ki_straight = 0.0;
float Kd_straight = 0.5;

float Kp_wall = 0.5;
float Ki_wall = 0.0;
float Kd_wall = 0.0;

//==================================================
// GLOBAL VARIABLES
//==================================================
volatile long leftPulse = 0;
volatile long rightPulse = 0;

int distFront1 = -1; 
int distFront = -1;  
int distLeft = -1;
int distRight = -1;

float yawAngle = 0.0;
float gyroZoffset = 0.0;
unsigned long lastMPUTime = 0;

// Robot Position
int rx = 0;
int ry = 0;
int rDir = 0; // 0=N, 1=E, 2=S, 3=W
const char *cellStatus = "Unknown";
const char *currentStatus = "Ready";

bool cmdExplore = false;
bool cmdStop = false;

//==================================================
// MAZE MAPPING (16x16 ARRAY)
//==================================================
#define WALL_NORTH 0b00000001
#define WALL_EAST  0b00000010
#define WALL_SOUTH 0b00000100
#define WALL_WEST  0b00001000
#define VISITED    0b00010000

uint8_t maze[16][16];

void setWall(int x, int y, uint8_t wall) {
  if (x < 0 || x > 15 || y < 0 || y > 15) return;
  maze[x][y] |= wall;
  // Update adjacent cell walls for consistency
  if (wall == WALL_NORTH && y < 15) maze[x][y+1] |= WALL_SOUTH;
  if (wall == WALL_SOUTH && y > 0) maze[x][y-1] |= WALL_NORTH;
  if (wall == WALL_EAST && x < 15) maze[x+1][y] |= WALL_WEST;
  if (wall == WALL_WEST && x > 0) maze[x-1][y] |= WALL_EAST;
}

void updateMap(bool wF, bool wL, bool wR) {
  if (rx < 0 || rx > 15 || ry < 0 || ry > 15) return;
  maze[rx][ry] |= VISITED;
  
  if (rDir == 0) { // North
    if (wF) setWall(rx, ry, WALL_NORTH);
    if (wL) setWall(rx, ry, WALL_WEST);
    if (wR) setWall(rx, ry, WALL_EAST);
  } else if (rDir == 1) { // East
    if (wF) setWall(rx, ry, WALL_EAST);
    if (wL) setWall(rx, ry, WALL_NORTH);
    if (wR) setWall(rx, ry, WALL_SOUTH);
  } else if (rDir == 2) { // South
    if (wF) setWall(rx, ry, WALL_SOUTH);
    if (wL) setWall(rx, ry, WALL_EAST);
    if (wR) setWall(rx, ry, WALL_WEST);
  } else if (rDir == 3) { // West
    if (wF) setWall(rx, ry, WALL_WEST);
    if (wL) setWall(rx, ry, WALL_SOUTH);
    if (wR) setWall(rx, ry, WALL_NORTH);
  }
}

//==================================================
// HARDWARE INSTANCES
//==================================================
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
  <title>Module 06 - Live Maze Mapping</title>
  <style>
    body { background: #0f172a; color: white; font-family: sans-serif; text-align: center; }
    h1 { color: #38bdf8; margin-bottom: 5px;}
    .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; max-width: 400px; margin: 10px auto; }
    .card { background: rgba(255,255,255,0.1); padding: 10px; border-radius: 8px; }
    .value { font-size: 1.2rem; font-weight: bold; color: #fbbf24; }
    .canvas-container { background: #1e293b; padding: 10px; border: 2px solid #38bdf8; border-radius: 12px; display: inline-block; margin-top: 10px;}
    canvas { background: #000; border-radius: 4px; }
  </style>
</head>
<body>
  <h1>Live Maze Map</h1>
  <h3 id="status" style="color: #a7f3d0; margin-top:0;">Status: Ready</h3>
  
  <div class="canvas-container">
    <canvas id="mazeCanvas" width="320" height="320"></canvas>
  </div>

  <div class="grid">
    <div class="card">X, Y Position<br><span class="value" id="pos">0, 0</span></div>
    <div class="card">Direction<br><span class="value" id="dir">North</span></div>
    <div class="card" style="grid-column: span 2;">Detection<br><span class="value" id="cell">--</span></div>
  </div>
  
  <script>
    const canvas = document.getElementById('mazeCanvas');
    const ctx = canvas.getContext('2d');
    const cellSize = 20;

    async function fetchData() {
      try {
        const res = await fetch('/data');
        const data = await res.json();
        
        document.getElementById('pos').innerText = data.x + ", " + data.y;
        document.getElementById('dir').innerText = data.d;
        document.getElementById('cell').innerText = data.c;
        document.getElementById('status').innerText = "Status: " + data.s;

        // Render Maze
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        
        for(let x = 0; x < 16; x++){
            for(let y = 0; y < 16; y++){
                let cell = data.maze[x * 16 + y];
                let px = x * cellSize;
                let py = (15 - y) * cellSize; // Origin bottom-left
                
                if (cell & 16) { // VISITED
                    ctx.fillStyle = '#334155';
                    ctx.fillRect(px, py, cellSize, cellSize);
                }
                
                ctx.strokeStyle = '#38bdf8';
                ctx.lineWidth = 2;
                if (cell & 1) { // North
                    ctx.beginPath(); ctx.moveTo(px, py); ctx.lineTo(px+cellSize, py); ctx.stroke();
                }
                if (cell & 2) { // East
                    ctx.beginPath(); ctx.moveTo(px+cellSize, py); ctx.lineTo(px+cellSize, py+cellSize); ctx.stroke();
                }
                if (cell & 4) { // South
                    ctx.beginPath(); ctx.moveTo(px, py+cellSize); ctx.lineTo(px+cellSize, py+cellSize); ctx.stroke();
                }
                if (cell & 8) { // West
                    ctx.beginPath(); ctx.moveTo(px, py); ctx.lineTo(px, py+cellSize); ctx.stroke();
                }
            }
        }
        
        // Draw Robot
        let rx = data.x * cellSize;
        let ry = (15 - data.y) * cellSize;
        
        ctx.fillStyle = '#ef4444';
        ctx.beginPath();
        ctx.arc(rx + cellSize/2, ry + cellSize/2, cellSize/3, 0, 2*Math.PI);
        ctx.fill();
        
        // Draw Heading Indicator
        ctx.strokeStyle = '#fff';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(rx + cellSize/2, ry + cellSize/2);
        if (data.d === "North") ctx.lineTo(rx + cellSize/2, ry + cellSize/6);
        else if (data.d === "South") ctx.lineTo(rx + cellSize/2, ry + cellSize - cellSize/6);
        else if (data.d === "East") ctx.lineTo(rx + cellSize - cellSize/6, ry + cellSize/2);
        else if (data.d === "West") ctx.lineTo(rx + cellSize/6, ry + cellSize/2);
        ctx.stroke();

      } catch (e) {}
    }
    setInterval(fetchData, 400);
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
// TURNING LOGIC
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

void moveForwardCell() {
  currentStatus = "Moving 1 Cell";
  noInterrupts();
  leftPulse = 0;
  rightPulse = 0;
  interrupts();
  yawAngle = 0.0;

  float Kp_gyro = 6.0;
  float Kp_enc = 0.5;

  while (true) {
    server.handleClient();
    if (cmdStop) break;

    long avgPulse = (leftPulse + rightPulse) / 2;
    if (avgPulse >= PULSES_PER_CELL) break;

    static unsigned long lastToFRead = 0;
    if (millis() - lastToFRead > 20) {
      readToF();
      lastToFRead = millis();
    }

    int currentBaseSpeed = BASE_PWM;
    
    // Use F2 (distFront) for perfect stopping, as fixed in Module 05
    int dFront = 8191;
    if (distFront > 0) dFront = distFront;
    else if (distFront1 > 0) dFront = distFront1;

    if (dFront > 0) {
      if (dFront <= TARGET_WALL_DIST) {
        motorsStop();         
        break;
      } else if (dFront < 180) {
        currentBaseSpeed = map(dFront, TARGET_WALL_DIST, 180, 50, BASE_PWM);
        currentBaseSpeed = constrain(currentBaseSpeed, 50, BASE_PWM);
      }
    }

    updateYaw();

    float gyroError = 0.0 - yawAngle;        
    float encError = rightPulse - leftPulse; 
    float wallError = 0.0;
    
    bool hasLeftWall = (distLeft > 0 && distLeft < (targetSideDist + 35));
    bool hasRightWall = (distRight > 0 && distRight < (targetSideDist + 35));

    if (hasLeftWall && hasRightWall) {
      wallError = distLeft - distRight; 
    } else if (hasLeftWall) {
      wallError = distLeft - targetSideDist; 
    } else if (hasRightWall) {
      wallError = targetSideDist - distRight; 
    }

    float correction = (Kp_gyro * gyroError) + (Kp_enc * encError) + (Kp_wall * wallError);
    correction = constrain(correction, -60, 60);

    leftMotor(currentBaseSpeed - correction);
    rightMotor(currentBaseSpeed + correction);
    delay(5); 
  }

  // Update Position
  if (rDir == 0) ry++;
  else if (rDir == 1) rx++;
  else if (rDir == 2) ry--;
  else if (rDir == 3) rx--;
}

//==================================================
// MAZE EXPLORATION LOGIC
//==================================================
void autoCalibrateWalls() {
  currentStatus = "Calibrating...";
  for (int i = 0; i < 5; i++) {
    readToF();
    server.handleClient();
    delay(50);
  }

  if (distLeft > 0 && distRight > 0 && distLeft < 150 && distRight < 150) {
    targetSideDist = (distLeft + distRight) / 2.0;
    WALL_THRESHOLD = targetSideDist + 80.0; 
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

  // === NEW: MAP BUILDING ===
  updateMap(wallF, wallL, wallR);

  // Detect Junction Type
  if (wallF && wallL && wallR) cellStatus = "Dead End";
  else if (!wallF && !wallL && !wallR) cellStatus = "Cross Junction";
  else if (!wallF && wallL && wallR) cellStatus = "Straight Path";
  else cellStatus = "Corner / T-Junct";

  currentStatus = "Analyzing...";

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

  // Initialize Map Array to 0
  for(int i=0; i<16; i++) {
    for(int j=0; j<16; j++) {
      maze[i][j] = 0;
    }
  }

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
  
  server.on("/data", []() {
    String dStr = (rDir == 0) ? "North" : (rDir == 1) ? "East" : (rDir == 2) ? "South" : "West";

    // Build JSON containing the whole 16x16 map
    String json = "{\"x\":" + String(rx) + ",\"y\":" + String(ry) + 
                  ",\"d\":\"" + dStr + "\",\"c\":\"" + String(cellStatus) +
                  "\",\"s\":\"" + String(currentStatus) + "\",\"maze\":[";
    
    for(int i = 0; i < 16; i++){
        for(int j = 0; j < 16; j++){
            json += String(maze[i][j]);
            if(i != 15 || j != 15) json += ",";
        }
    }
    json += "]}";
    
    server.send(200, "application/json", json);
  });

  server.begin();

  Serial.println("Waiting 3 seconds for perfect Gyro stabilization before Auto Start...");
  delay(3000);
  
  // Starting at (0,0) facing North
  rx = 0;
  ry = 0;
  rDir = 0;
  
  // We assume we have a South wall behind us at the starting point
  setWall(0, 0, WALL_SOUTH); 
  
  autoCalibrateWalls(); 
  cmdExplore = true;    
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
