#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define I2C_SDA 40
#define I2C_SCL 2

#define XSHUT_FRONT1 13
#define XSHUT_FRONT2 12
#define XSHUT_LEFT 11
#define XSHUT_RIGHT 10

#define ADDR_FRONT1 0x30
#define ADDR_FRONT2 0x31
#define ADDR_LEFT 0x32
#define ADDR_RIGHT 0x33

Adafruit_VL53L0X tofFront1 = Adafruit_VL53L0X();
Adafruit_VL53L0X tofFront = Adafruit_VL53L0X();
Adafruit_VL53L0X tofLeft = Adafruit_VL53L0X();
Adafruit_VL53L0X tofRight = Adafruit_VL53L0X();
Adafruit_MPU6050 mpu;
WebServer server(80);

const char *ssid = "Tecno Spark";
const char *password = "20240869kp";

int distFront1 = 8191, distFront = 8191, distLeft = 8191, distRight = 8191;
float yawAngle = 0.0;
float gyroZoffset = 0.0;
unsigned long lastMPUTime = 0;

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
  tofFront.begin(ADDR_FRONT2, false, &Wire);

  digitalWrite(XSHUT_LEFT, HIGH); delay(10);
  tofLeft.begin(ADDR_LEFT, false, &Wire);

  digitalWrite(XSHUT_RIGHT, HIGH); delay(10);
  tofRight.begin(ADDR_RIGHT, false, &Wire);
}

void readToF() {
  VL53L0X_RangingMeasurementData_t m;
  
  tofFront1.rangingTest(&m, false);
  distFront1 = (m.RangeStatus != 4 && m.RangeMilliMeter != 0) ? m.RangeMilliMeter : -1;
  
  tofFront.rangingTest(&m, false);
  distFront = (m.RangeStatus != 4 && m.RangeMilliMeter != 0) ? m.RangeMilliMeter : -1;
  
  tofLeft.rangingTest(&m, false);
  distLeft = (m.RangeStatus != 4 && m.RangeMilliMeter != 0) ? m.RangeMilliMeter : -1;
  
  tofRight.rangingTest(&m, false);
  distRight = (m.RangeStatus != 4 && m.RangeMilliMeter != 0) ? m.RangeMilliMeter : -1;
}

void setupMPU() {
  if (mpu.begin()) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    delay(1000);
    float zSum = 0;
    for (int i = 0; i < 200; i++) {
      sensors_event_t a, g, t;
      mpu.getEvent(&a, &g, &t);
      zSum += g.gyro.z;
      delay(5);
    }
    gyroZoffset = zSum / 200.0;
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
  if (abs(zRate) < 0.03) zRate = 0.0;
  yawAngle += (zRate * 57.2958) * dt;
}

void handleData() {
  String json = "{";
  json += "\"f1_dist\":" + String(distFront1) + ",";
  json += "\"f2_dist\":" + String(distFront) + ",";
  json += "\"l_dist\":" + String(distLeft) + ",";
  json += "\"r_dist\":" + String(distRight) + ",";
  json += "\"yaw\":" + String(yawAngle) + ",";
  json += "\"fw1\":" + String(distFront1 > 0 && distFront1 < 120 ? "true" : "false") + ",";
  json += "\"fw2\":" + String(distFront > 0 && distFront < 120 ? "true" : "false") + ",";
  json += "\"lw\":" + String(distLeft > 0 && distLeft < 150 ? "true" : "false") + ",";
  json += "\"rw\":" + String(distRight > 0 && distRight < 150 ? "true" : "false") + ",";
  json += "\"x\":0,\"y\":0,\"d\":\"N\",\"c\":\"--\",\"s\":\"MANUAL TEST\"";
  json += "}";
  server.send(200, "application/json", json);
}

void setup() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  
  setupToF();
  setupMPU();

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  
  server.on("/data", handleData);
  server.begin();
}

void loop() {
  server.handleClient();
  
  static unsigned long lastToF = 0;
  if (millis() - lastToF > 50) {
    readToF();
    lastToF = millis();
  }
  
  updateYaw();
  delay(5);
}
