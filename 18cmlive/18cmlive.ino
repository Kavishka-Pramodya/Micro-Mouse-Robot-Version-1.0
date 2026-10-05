/*
=========================================
Module 03 + Live Web Dashboard
Move One Cell (18cm)
ESP32-S3 + TB6612FNG + Encoder
=========================================
*/

#include <WiFi.h>
#include <WebServer.h>

// Replace with your network credentials
const char* ssid = "Tecno Spark";
const char* password = "20240869kp";

WebServer server(80);

//--------------- Motor Driver ---------------
#define PWMA 4
#define AIN1 5
#define AIN2 6

#define PWMB 18
#define BIN1 15
#define BIN2 14

#define STBY 17

//--------------- Encoder --------------------
#define LEFT_ENC_A 9
#define RIGHT_ENC_A 36

volatile long leftPulse = 0;
volatile long rightPulse = 0;

//******** TEMPORARY ********
// Change after calibration
const long PULSES_PER_CELL = 388;

//--------------------------------------------
void IRAM_ATTR leftEncoderISR()
{
  leftPulse++;
}

void IRAM_ATTR rightEncoderISR()
{
  rightPulse++;
}

//--------------------------------------------
void motorForward(int pwm)
{
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);

  ledcWrite(PWMA, pwm);
  ledcWrite(PWMB, pwm);
}

//--------------------------------------------
void motorStop()
{
  ledcWrite(PWMA, 0);
  ledcWrite(PWMB, 0);
}

//--- Web Server JSON Endpoint ---
void handleData() {
  long avgPulse = (leftPulse + rightPulse) / 2;
  String json = "{";
  json += "\"left\":" + String(leftPulse) + ",";
  json += "\"right\":" + String(rightPulse) + ",";
  json += "\"avg\":" + String(avgPulse);
  json += "}";
  server.send(200, "application/json", json);
}

//--- Web Server HTML Page ---
void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<title>Micromouse Live Dashboard</title>";
  html += "<style>body{font-family:Arial,sans-serif; text-align:center; background:#f4f4f9; padding:20px;}";
  html += ".card{background:white; padding:20px; border-radius:10px; box-shadow:0 4px 8px rgba(0,0,0,0.1); display:inline-block; margin:10px; min-width:150px;}";
  html += "h1{color:#333;} .value{font-size:24px; font-weight:bold; color:#007bff;}</style></head><body>";
  html += "<h1>Micromouse Live Monitor</h1>";
  html += "<div class='card'><h3>Left Pulse</h3><div id='left' class='value'>0</div></div>";
  html += "<div class='card'><h3>Right Pulse</h3><div id='right' class='value'>0</div></div>";
  html += "<div class='card'><h3>Average Pulse</h3><div id='avg' class='value'>0</div></div>";
  html += "<script>setInterval(function(){";
  html += "fetch('/data').then(response=>response.json()).then(data=>{";
  html += "document.getElementById('left').innerText=data.left;";
  html += "document.getElementById('right').innerText=data.right;";
  html += "document.getElementById('avg').innerText=data.avg;";
  html += "});},100);</script></body></html>";
  
  server.send(200, "text/html", html);
}

//--------------------------------------------
void moveOneCell()
{
  leftPulse = 0;
  rightPulse = 0;

  motorForward(180);

  while (true)
  {
    // Web server handle karanawa loop eka athule live data refresh wenna
    server.handleClient();
    vTaskDelay(1);
    long avgPulse = (leftPulse + rightPulse) / 2;

    Serial.print("Average Pulse : ");
    Serial.println(avgPulse);

    if (avgPulse >= PULSES_PER_CELL)
      break;

    delay(5);
  }

  motorStop();

  Serial.println("Reached One Cell (18cm)");

  // Post-reach delay ekadiath server eka weda karanna mini loops use karala thiyenne
  for(int i=0; i<100; i++) {
    server.handleClient();
    delay(10);
  }
}

//--------------------------------------------
void setup()
{
  Serial.begin(115200);

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

  attachInterrupt(
      digitalPinToInterrupt(LEFT_ENC_A),
      leftEncoderISR,
      RISING);

  attachInterrupt(
      digitalPinToInterrupt(RIGHT_ENC_A),
      rightEncoderISR,
      RISING);

  // Wi-Fi Configuration
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP()); // Mema IP eka browser ekata danna

  // Server Routes
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();

  Serial.println("Module 03 Ready");
}

//--------------------------------------------
void loop()
{
  moveOneCell();

  // Loop delay ekadiath server connections continuous thiyaganna mehema handle karala thiyenne
  for(int i=0; i<300; i++) {
    server.handleClient();
    delay(10);
  }
}
