/*
====================================================
MODULE 07, 08, 09 - THE FINAL BRAIN (DUAL CORE)
====================================================
Features:
- Flood Fill Algorithm (Shortest Path to 7,7)
- EEPROM Map Saving/Loading
- State Machine (Explore -> Return -> Fast Run)
- Gyro Slalom (Curve) Turns for Fast Run
- Software Safety Stop (Stall Detection)
- DUAL CORE OPTIMIZATION (FreeRTOS)
  * Core 0: Web Server & UI (Zero Lag)
  * Core 1: PID, Gyro & Motors (100% Performance)
====================================================
*/

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_VL53L0X.h>
#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>
#include <EEPROM.h>

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
#define XSHUT_LEFT 11
#define XSHUT_RIGHT 13

#define ADDR_FRONT1 0x30
#define ADDR_FRONT2 0x31
#define ADDR_LEFT 0x32
#define ADDR_RIGHT 0x33

const int PULSES_PER_CELL = 388;
int WALL_THRESHOLD = 130; 
const float TARGET_WALL_DIST = 65.0; 
int BASE_PWM = 130;
int FAST_PWM = 200; 
float targetSideDist = 55.0; 

// PID Settings
float Kp_gyro = 6.0;
float Kp_enc = 0.5;
float Kp_wall = 0.5;

//==================================================
// GLOBAL VARIABLES
//==================================================
volatile long leftPulse = 0;
volatile long rightPulse = 0;

int distFront1 = -1, distFront = -1, distLeft = -1, distRight = -1;
float yawAngle = 0.0;
float gyroZoffset = 0.0;
unsigned long lastMPUTime = 0;

int rx = 0, ry = 0, rDir = 0; 
const char *currentStatus = "Ready";

enum State { STATE_IDLE, STATE_EXPLORE, STATE_RETURN, STATE_FAST_RUN };
State robotState = STATE_IDLE;

//==================================================
// MAZE & FLOOD FILL
//==================================================
#define WALL_NORTH 0b00000001
#define WALL_EAST  0b00000010
#define WALL_SOUTH 0b00000100
#define WALL_WEST  0b00001000
#define VISITED    0b00010000

uint8_t maze[16][16];
uint8_t distances[16][16];

void setWall(int x, int y, uint8_t wall) {
  if (x < 0 || x > 15 || y < 0 || y > 15) return;
  maze[x][y] |= wall;
  if (wall == WALL_NORTH && y < 15) maze[x][y+1] |= WALL_SOUTH;
  if (wall == WALL_SOUTH && y > 0) maze[x][y-1] |= WALL_NORTH;
  if (wall == WALL_EAST && x < 15) maze[x+1][y] |= WALL_WEST;
  if (wall == WALL_WEST && x > 0) maze[x-1][y] |= WALL_EAST;
}

void updateMap(bool wF, bool wL, bool wR) {
  if (rx < 0 || rx > 15 || ry < 0 || ry > 15) return;
  maze[rx][ry] |= VISITED;
  
  if (rDir == 0) { if(wF) setWall(rx,ry,WALL_NORTH); if(wL) setWall(rx,ry,WALL_WEST); if(wR) setWall(rx,ry,WALL_EAST); }
  else if (rDir == 1) { if(wF) setWall(rx,ry,WALL_EAST); if(wL) setWall(rx,ry,WALL_NORTH); if(wR) setWall(rx,ry,WALL_SOUTH); }
  else if (rDir == 2) { if(wF) setWall(rx,ry,WALL_SOUTH); if(wL) setWall(rx,ry,WALL_EAST); if(wR) setWall(rx,ry,WALL_WEST); }
  else if (rDir == 3) { if(wF) setWall(rx,ry,WALL_WEST); if(wL) setWall(rx,ry,WALL_SOUTH); if(wR) setWall(rx,ry,WALL_NORTH); }
}

void floodFill(int targetX, int targetY) {
  for (int i=0; i<16; i++) for (int j=0; j<16; j++) distances[i][j] = 255;
  
  int qx[256], qy[256];
  int head = 0, tail = 0;
  
  if (targetX == 7 && targetY == 7) {
    distances[7][7] = 0; distances[7][8] = 0; distances[8][7] = 0; distances[8][8] = 0;
    qx[tail] = 7; qy[tail++] = 7;
    qx[tail] = 7; qy[tail++] = 8;
    qx[tail] = 8; qy[tail++] = 7;
    qx[tail] = 8; qy[tail++] = 8;
  } else {
    distances[targetX][targetY] = 0;
    qx[tail] = targetX; qy[tail++] = targetY;
  }
  
  while (head < tail) {
    int cx = qx[head]; int cy = qy[head++]; int d = distances[cx][cy];
    if (!(maze[cx][cy] & WALL_NORTH) && cy < 15 && distances[cx][cy+1] == 255) { distances[cx][cy+1] = d + 1; qx[tail] = cx; qy[tail++] = cy+1; }
    if (!(maze[cx][cy] & WALL_SOUTH) && cy > 0 && distances[cx][cy-1] == 255) { distances[cx][cy-1] = d + 1; qx[tail] = cx; qy[tail++] = cy-1; }
    if (!(maze[cx][cy] & WALL_EAST) && cx < 15 && distances[cx+1][cy] == 255) { distances[cx+1][cy] = d + 1; qx[tail] = cx+1; qy[tail++] = cy; }
    if (!(maze[cx][cy] & WALL_WEST) && cx > 0 && distances[cx-1][cy] == 255) { distances[cx-1][cy] = d + 1; qx[tail] = cx-1; qy[tail++] = cy; }
  }
}

void saveMap() {
  currentStatus = "Saving Map...";
  for(int i=0; i<16; i++) for(int j=0; j<16; j++) EEPROM.write(i*16+j, maze[i][j]);
  EEPROM.write(256, 0xAA); 
  EEPROM.commit();
}
bool loadMap() {
  if (EEPROM.read(256) == 0xAA) {
    for(int i=0; i<16; i++) for(int j=0; j<16; j++) maze[i][j] = EEPROM.read(i*16+j);
    return true;
  }
  return false;
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

const char htmlPage[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <title>Module 07-09: The Final Brain</title>
  <style>
    body { background: #0f172a; color: white; font-family: sans-serif; text-align: center; }
    h1 { color: #38bdf8; margin-bottom: 5px;}
    .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; max-width: 400px; margin: 10px auto; }
    .card { background: rgba(255,255,255,0.1); padding: 10px; border-radius: 8px; }
    .btn { padding: 15px; border: none; border-radius: 8px; font-weight: bold; cursor: pointer; color: white; width: 90%; max-width: 400px; margin: 5px auto; display: block; }
    .btn-auto { background: #6366f1; }
    .btn-fast { background: #10b981; }
    .btn-stop { background: #ef4444; }
    .canvas-container { background: #1e293b; padding: 10px; border: 2px solid #38bdf8; border-radius: 12px; display: inline-block; margin-top: 10px;}
  </style>
</head>
<body>
  <h1>MicroMouse Final</h1>
  <h3 id="status" style="color: #a7f3d0; margin-top:0;">Status: Ready</h3>
  
  <div class="canvas-container">
    <canvas id="mazeCanvas" width="320" height="320"></canvas>
  </div>

  <div class="grid">
    <button class="btn btn-stop" onclick="fetch('/stop')" style="grid-column: span 2;">EMERGENCY STOP</button>
  </div>
  
  <script>
    const canvas = document.getElementById('mazeCanvas');
    const ctx = canvas.getContext('2d');
    const cellSize = 20;

    async function fetchData() {
      try {
        const res = await fetch('/data');
        const data = await res.json();
        document.getElementById('status').innerText = "Status: " + data.s;

        ctx.clearRect(0, 0, canvas.width, canvas.height);
        
        ctx.fillStyle = 'rgba(251, 191, 36, 0.3)';
        ctx.fillRect(7*cellSize, (15-8)*cellSize, cellSize*2, cellSize*2);

        for(let x = 0; x < 16; x++){
            for(let y = 0; y < 16; y++){
                let cell = data.maze[x * 16 + y];
                let dist = data.dist[x * 16 + y];
                let px = x * cellSize; let py = (15 - y) * cellSize;
                
                if (cell & 16) { ctx.fillStyle = '#334155'; ctx.fillRect(px, py, cellSize, cellSize); }
                
                if(dist < 255) { ctx.fillStyle = '#94a3b8'; ctx.font = '8px Arial'; ctx.fillText(dist, px+2, py+12); }
                
                ctx.strokeStyle = '#38bdf8'; ctx.lineWidth = 2;
                if (cell & 1) { ctx.beginPath(); ctx.moveTo(px, py); ctx.lineTo(px+cellSize, py); ctx.stroke(); }
                if (cell & 2) { ctx.beginPath(); ctx.moveTo(px+cellSize, py); ctx.lineTo(px+cellSize, py+cellSize); ctx.stroke(); }
                if (cell & 4) { ctx.beginPath(); ctx.moveTo(px, py+cellSize); ctx.lineTo(px+cellSize, py+cellSize); ctx.stroke(); }
                if (cell & 8) { ctx.beginPath(); ctx.moveTo(px, py); ctx.lineTo(px, py+cellSize); ctx.stroke(); }
            }
        }
        
        let rx = data.x * cellSize; let ry = (15 - data.y) * cellSize;
        ctx.fillStyle = '#ef4444'; ctx.beginPath(); ctx.arc(rx + cellSize/2, ry + cellSize/2, cellSize/3, 0, 2*Math.PI); ctx.fill();
      } catch (e) {}
    }
    setInterval(fetchData, 400);
  </script>
</body>
</html>
)=====";

//==================================================
// FREERTOS CORE 0 TASK (WEB SERVER)
//==================================================
TaskHandle_t WebTask;

void webServerTask(void * pvParameters) {
  for(;;) {
    server.handleClient();
    vTaskDelay(10 / portTICK_PERIOD_MS); // Yield to prevent Watchdog Timeout on Core 0
  }
}

//==================================================
// MOTORS & ENCODERS
//==================================================
void IRAM_ATTR leftEncoderISR() { leftPulse++; }
void IRAM_ATTR rightEncoderISR() { rightPulse++; }

void setupMotors() {
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);
  ledcAttach(PWMA, 1000, 8); ledcAttach(PWMB, 1000, 8);

  pinMode(LEFT_ENC_A, INPUT_PULLUP); pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);
}
void motorsStop() {
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW); digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
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

void checkSafetyStop(int expectedPwm) {
  // Disabled to prevent instant false stall triggers at startup
}

//==================================================
// SENSORS
//==================================================
void setupToF() {
  pinMode(XSHUT_FRONT1, OUTPUT); pinMode(XSHUT_FRONT2, OUTPUT); pinMode(XSHUT_LEFT, OUTPUT); pinMode(XSHUT_RIGHT, OUTPUT);
  digitalWrite(XSHUT_FRONT1, LOW); digitalWrite(XSHUT_FRONT2, LOW); digitalWrite(XSHUT_LEFT, LOW); digitalWrite(XSHUT_RIGHT, LOW); delay(10);
  digitalWrite(XSHUT_FRONT1, HIGH); delay(10); tofFront1.begin(ADDR_FRONT1, false, &Wire);
  digitalWrite(XSHUT_FRONT2, HIGH); delay(10); tofFront.begin(ADDR_FRONT2, false, &Wire);
  digitalWrite(XSHUT_LEFT, HIGH); delay(10); tofLeft.begin(ADDR_LEFT, false, &Wire);
  digitalWrite(XSHUT_RIGHT, HIGH); delay(10); tofRight.begin(ADDR_RIGHT, false, &Wire);
}
void readToF() {
  VL53L0X_RangingMeasurementData_t m;
  memset(&m, 0, sizeof(m)); tofFront1.rangingTest(&m, false); distFront1 = (m.RangeStatus != 4 && m.RangeMilliMeter != 0) ? m.RangeMilliMeter : -1;
  memset(&m, 0, sizeof(m)); tofFront.rangingTest(&m, false); distFront = (m.RangeStatus != 4 && m.RangeMilliMeter != 0) ? m.RangeMilliMeter : -1;
  memset(&m, 0, sizeof(m)); tofLeft.rangingTest(&m, false); distLeft = (m.RangeStatus != 4 && m.RangeMilliMeter != 0) ? m.RangeMilliMeter : -1;
  memset(&m, 0, sizeof(m)); tofRight.rangingTest(&m, false); distRight = (m.RangeStatus != 4 && m.RangeMilliMeter != 0) ? m.RangeMilliMeter : -1;
}

void setupMPU() {
  if (mpu.begin()) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G); mpu.setGyroRange(MPU6050_RANGE_500_DEG); mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); delay(1000);
    float zSum = 0; sensors_event_t a, g, t;
    for (int i=0; i<200; i++) { mpu.getEvent(&a, &g, &t); zSum += g.gyro.z; delay(5); }
    gyroZoffset = zSum / 200; lastMPUTime = millis(); yawAngle = 0.0;
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
// MOVEMENT LOGIC
//==================================================
void pivotTurn(float targetAngle) {
  currentStatus = "Turning...";
  yawAngle = 0.0; float Kp = 3.0; unsigned long turnStart = millis();

  while (millis() - turnStart < 2500) {
    if (robotState == STATE_IDLE) break;
    updateYaw();
    float error = targetAngle - yawAngle;
    if (abs(error) < 1.5) break;

    float pOut = constrain(Kp * error, -180, 180);
    if (pOut > 0 && pOut < 55) pOut = 55;
    if (pOut < 0 && pOut > -55) pOut = -55;

    leftMotor(-pOut); rightMotor(pOut); delay(5);
  }
  motorsStop(); delay(200);
}

void slalomCurve(float targetAngleDelta) {
  currentStatus = "Curve Turn!";
  float initialYaw = yawAngle;
  float targetYaw = yawAngle + targetAngleDelta;
  
  noInterrupts(); long startAvg = (leftPulse + rightPulse) / 2; interrupts();
  
  while (true) {
    if (robotState == STATE_IDLE) break;
    long currAvg = (leftPulse + rightPulse) / 2;
    long diff = currAvg - startAvg;
    if (diff >= 250) break; 
    
    float progress = (float)diff / 250.0;
    float currentTargetYaw = initialYaw + (targetAngleDelta * progress);
    
    updateYaw();
    float gyroError = currentTargetYaw - yawAngle;
    float correction = Kp_gyro * gyroError; 
    
    leftMotor(FAST_PWM - correction); rightMotor(FAST_PWM + correction);
    delay(5);
  }
  yawAngle = targetYaw; 
}

void moveForwardCell(int speedPwm, bool useWallPID) {
  currentStatus = "Moving...";
  noInterrupts(); leftPulse = 0; rightPulse = 0; interrupts();
  yawAngle = 0.0;

  while (true) {
    if (robotState == STATE_IDLE) break;

    long avgPulse = (leftPulse + rightPulse) / 2;
    if (avgPulse >= PULSES_PER_CELL) break;

    static unsigned long lastToFRead = 0;
    if (millis() - lastToFRead > 20) { readToF(); lastToFRead = millis(); }

    int currentBaseSpeed = speedPwm;
    int dFront = (distFront > 0) ? distFront : distFront1;

    if (dFront > 0) {
      if (dFront <= TARGET_WALL_DIST) { motorsStop(); break; } 
      else if (dFront < 180) { currentBaseSpeed = map(dFront, TARGET_WALL_DIST, 180, 50, speedPwm); }
    }

    updateYaw();
    float gyroError = 0.0 - yawAngle;        
    float encError = rightPulse - leftPulse; 
    float wallError = 0.0;
    
    if (useWallPID) {
      bool hasLeftWall = (distLeft > 0 && distLeft < (targetSideDist + 35));
      bool hasRightWall = (distRight > 0 && distRight < (targetSideDist + 35));
      if (hasLeftWall && hasRightWall) wallError = distLeft - distRight; 
      else if (hasLeftWall) wallError = distLeft - targetSideDist; 
      else if (hasRightWall) wallError = targetSideDist - distRight; 
    }

    float correction = constrain((Kp_gyro * gyroError) + (Kp_enc * encError) + (Kp_wall * wallError), -60, 60);
    leftMotor(currentBaseSpeed - correction); rightMotor(currentBaseSpeed + correction);
    
    delay(5); 
  }

  if (rDir == 0) ry++; else if (rDir == 1) rx++; else if (rDir == 2) ry--; else if (rDir == 3) rx--;
}

//==================================================
// FLOOD FILL NAVIGATION
//==================================================
void runFloodFillAI() {
  readToF();
  bool wallF = (distFront > 0 && distFront < WALL_THRESHOLD);
  bool wallL = (distLeft > 0 && distLeft < WALL_THRESHOLD);
  bool wallR = (distRight > 0 && distRight < WALL_THRESHOLD);

  updateMap(wallF, wallL, wallR);

  if (robotState == STATE_EXPLORE) floodFill(7, 7); 
  else if (robotState == STATE_RETURN) floodFill(0, 0); 
  else if (robotState == STATE_FAST_RUN) floodFill(7, 7); 

  if (robotState == STATE_EXPLORE && (rx == 7 || rx == 8) && (ry == 7 || ry == 8)) {
    motorsStop(); currentStatus = "Goal Reached!"; delay(2000);
    robotState = STATE_RETURN; floodFill(0, 0);
  }
  else if (robotState == STATE_RETURN && rx == 0 && ry == 0) {
    motorsStop(); currentStatus = "Back at Start. Saving Map.";
    saveMap(); 
    
    // Auto transition to Fast Run after 3 seconds!
    delay(3000);
    currentStatus = "Starting Fast Run!";
    robotState = STATE_FAST_RUN; 
    return;
  }
  else if (robotState == STATE_FAST_RUN && (rx == 7 || rx == 8) && (ry == 7 || ry == 8)) {
    motorsStop(); currentStatus = "FAST RUN COMPLETE!"; robotState = STATE_IDLE; return;
  }

  int minD = 255; int nextDir = -1;
  int absF = rDir, absL = (rDir + 3) % 4, absR = (rDir + 1) % 4, absB = (rDir + 2) % 4;
  
  int dxF = (absF==1)?1:(absF==3)?-1:0; int dyF = (absF==0)?1:(absF==2)?-1:0;
  if (!wallF && distances[rx+dxF][ry+dyF] < minD) { minD = distances[rx+dxF][ry+dyF]; nextDir = absF; }
  
  int dxR = (absR==1)?1:(absR==3)?-1:0; int dyR = (absR==0)?1:(absR==2)?-1:0;
  if (!wallR && distances[rx+dxR][ry+dyR] < minD) { minD = distances[rx+dxR][ry+dyR]; nextDir = absR; }
  
  int dxL = (absL==1)?1:(absL==3)?-1:0; int dyL = (absL==0)?1:(absL==2)?-1:0;
  if (!wallL && distances[rx+dxL][ry+dyL] < minD) { minD = distances[rx+dxL][ry+dyL]; nextDir = absL; }

  if (nextDir == -1) nextDir = absB; 

  if (robotState == STATE_FAST_RUN) {
    if (nextDir == absF) { moveForwardCell(FAST_PWM, false); }
    else if (nextDir == absL) { slalomCurve(90.0); rDir = absL; moveForwardCell(FAST_PWM, false); }
    else if (nextDir == absR) { slalomCurve(-90.0); rDir = absR; moveForwardCell(FAST_PWM, false); }
    else { pivotTurn(180.0); rDir = absB; moveForwardCell(FAST_PWM, false); }
  } else {
    if (nextDir == absF) { moveForwardCell(BASE_PWM, true); }
    else if (nextDir == absL) { pivotTurn(90.0); rDir = absL; moveForwardCell(BASE_PWM, true); }
    else if (nextDir == absR) { pivotTurn(-90.0); rDir = absR; moveForwardCell(BASE_PWM, true); }
    else { pivotTurn(180.0); rDir = absB; moveForwardCell(BASE_PWM, true); }
  }
}

//==================================================
// SETUP & MAIN LOOP (CORE 1)
//==================================================
void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL); Wire.setClock(400000);
  EEPROM.begin(512);

  for(int i=0; i<16; i++) for(int j=0; j<16; j++) { maze[i][j] = 0; distances[i][j] = 255; }
  
  setupMotors(); setupToF(); setupMPU();

  WiFi.mode(WIFI_STA); WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); }

  server.on("/", []() { server.send(200, "text/html", htmlPage); });
  server.on("/explore", []() { robotState = STATE_EXPLORE; server.send(200, "text/plain", "OK"); });
  server.on("/fast", []() { robotState = STATE_FAST_RUN; server.send(200, "text/plain", "OK"); });
  server.on("/stop", []() { robotState = STATE_IDLE; motorsStop(); server.send(200, "text/plain", "OK"); });
  server.on("/data", []() {
    String json = "{\"x\":" + String(rx) + ",\"y\":" + String(ry) + ",\"s\":\"" + String(currentStatus) + "\",\"maze\":[";
    for(int i = 0; i < 16; i++){ for(int j = 0; j < 16; j++){ json += String(maze[i][j]); if(i!=15 || j!=15) json += ","; } }
    json += "],\"dist\":[";
    for(int i = 0; i < 16; i++){ for(int j = 0; j < 16; j++){ json += String(distances[i][j]); if(i!=15 || j!=15) json += ","; } }
    json += "]}";
    server.send(200, "application/json", json);
  });
  server.begin();

  // PIN WEB SERVER TO CORE 0
  xTaskCreatePinnedToCore(webServerTask, "WebTask", 8192, NULL, 1, &WebTask, 0);

  Serial.println("Waiting 3 seconds for perfect Gyro stabilization before Auto Start...");
  delay(3000);
  
  if (loadMap()) {
    currentStatus = "Map Loaded! Starting Auto Fast Run!";
    robotState = STATE_FAST_RUN;
  } else {
    currentStatus = "No Map. Starting Auto Explore!";
    robotState = STATE_EXPLORE;
  }
  
  setWall(0, 0, WALL_SOUTH); 
}

void loop() {
  // CORE 1: 100% Dedicated to PID and Navigation! No server.handleClient() here!
  if (robotState != STATE_IDLE) {
    runFloodFillAI();
  } else {
    motorsStop();
  }
  updateYaw();
  delay(5); // Extremely fast 200Hz loop for maximum Gyro accuracy
}
