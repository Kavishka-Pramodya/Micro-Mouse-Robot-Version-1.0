/*
====================================================
MODULE 01
module01_motor_encoder_test.ino
Hardware Bring-up + Motor + Encoder System
ESP32-S3 + TB6612FNG + 2x N20 Encoder Motors
====================================================
*/

#include <Arduino.h>
#include "module01.h"

//==================================================
// TB6612FNG MOTOR DRIVER
//==================================================

// LEFT MOTOR / MOTOR A
#define PWMA  4
#define AIN1  5
#define AIN2  6

// RIGHT MOTOR / MOTOR B
#define PWMB  7
#define BIN1  15
#define BIN2  16

// Standby
#define STBY  17

//==================================================
// ENCODER
//==================================================

// LEFT N20 ENCODER
// C1 = Channel A
// C2 = Channel B
#define LEFT_ENC_A   8
#define LEFT_ENC_B   3

// RIGHT N20 ENCODER
#define RIGHT_ENC_A  36
#define RIGHT_ENC_B  35

//==================================================
// VARIABLES
//==================================================

volatile long leftPulse  = 0;
volatile long rightPulse = 0;

// Current PWM values
int leftPWM  = 180;
int rightPWM = 180;

//==================================================
// ENCODER INTERRUPTS
//==================================================

void IRAM_ATTR leftEncoderISR()
{
  leftPulse++;
}

void IRAM_ATTR rightEncoderISR()
{
  rightPulse++;
}

//==================================================
// MOTOR STOP
//==================================================

void motorStop()
{
  ledcWrite(PWMA, 0);
  ledcWrite(PWMB, 0);
}

//==================================================
// MOTOR FORWARD
//==================================================

void motorForward(int leftSpeed, int rightSpeed)
{
  // LEFT MOTOR FORWARD
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);

  // RIGHT MOTOR FORWARD
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);

  ledcWrite(PWMA, leftSpeed);
  ledcWrite(PWMB, rightSpeed);
}

//==================================================
// MOTOR BACKWARD
//==================================================

void motorBackward(int leftSpeed, int rightSpeed)
{
  // LEFT MOTOR BACKWARD
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);

  // RIGHT MOTOR BACKWARD
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);

  ledcWrite(PWMA, leftSpeed);
  ledcWrite(PWMB, rightSpeed);
}

//==================================================
// RESET ENCODER COUNTS
//==================================================

void resetEncoders()
{
  noInterrupts();
  leftPulse  = 0;
  rightPulse = 0;
  interrupts();
}

//==================================================
// FORWARD TEST
//==================================================

void forwardTest()
{
  Serial.println();
  Serial.println("================================");
  Serial.println("FORWARD TEST START");
  Serial.println("================================");

  resetEncoders();
  motorForward(leftPWM, rightPWM);

  unsigned long startTime = millis();

  while (millis() - startTime < 3000)
  {
    printEncoderValues();
    delay(200);
  }

  motorStop();

  Serial.println("FORWARD TEST COMPLETE");
  printEncoderValues();
}

//==================================================
// BACKWARD TEST
//==================================================

void backwardTest()
{
  Serial.println();
  Serial.println("================================");
  Serial.println("BACKWARD TEST START");
  Serial.println("================================");

  resetEncoders();
  motorBackward(leftPWM, rightPWM);

  unsigned long startTime = millis();

  while (millis() - startTime < 3000)
  {
    printEncoderValues();
    delay(200);
  }

  motorStop();

  Serial.println("BACKWARD TEST COMPLETE");
  printEncoderValues();
}

//==================================================
// MOTOR BALANCE TEST
//==================================================

void balanceTest()
{
  Serial.println();
  Serial.println("================================");
  Serial.println("MOTOR BALANCE TEST");
  Serial.println("================================");

  resetEncoders();
  motorForward(leftPWM, rightPWM);

  unsigned long startTime = millis();

  while (millis() - startTime < 5000)
  {
    long left;
    long right;

    noInterrupts();
    left  = leftPulse;
    right = rightPulse;
    interrupts();

    long average = (left + right) / 2;

    Serial.print("Left : ");
    Serial.print(left);
    Serial.print("    Right : ");
    Serial.print(right);
    Serial.print("    Average : ");
    Serial.println(average);

    delay(200);
  }

  motorStop();

  Serial.println();
  Serial.println("BALANCE TEST COMPLETE");
  printEncoderValues();
}

//==================================================
// SERIAL MENU
//==================================================

void printMenu()
{
  Serial.println();
  Serial.println("================================");
  Serial.println("MICROMOUSE MODULE 01");
  Serial.println("================================");
  Serial.println("F = Forward");
  Serial.println("B = Backward");
  Serial.println("S = Stop");
  Serial.println("T = Encoder Test");
  Serial.println("M = Motor Balance Test");
  Serial.println("R = Reset Encoder");
  Serial.println("P = Print Status");
  Serial.println("C = Move Cell (18cm)");
  Serial.println("================================");
}

//==================================================
// SETUP
//==================================================

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("================================");
  Serial.println("MODULE 01 STARTING...");
  Serial.println("================================");

  // MOTOR DRIVER PIN SETUP
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  // Enable TB6612FNG
  digitalWrite(STBY, HIGH);

  // PWM SETUP
  ledcAttach(PWMA, 1000, 8);
  ledcAttach(PWMB, 1000, 8);

  // ENCODER SETUP
  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  // INTERRUPTS
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);

  motorStop();
  Serial.println("Module 01 Ready");
  printMenu();
}

//==================================================
// LOOP
//==================================================

void loop()
{
  if (Serial.available())
  {
    char command = Serial.read();

    if (command >= 'a' && command <= 'z') command = command - 32;

    if (command == 'F') {
      resetEncoders();
      Serial.println("\nMotor Forward");
      motorForward(leftPWM, rightPWM);
    }
    else if (command == 'B') {
      resetEncoders();
      Serial.println("\nMotor Backward");
      motorBackward(leftPWM, rightPWM);
    }
    else if (command == 'S') {
      motorStop();
      Serial.println("\nMotor STOP");
    }
    else if (command == 'T') {
      forwardTest();
    }
    else if (command == 'M') {
      balanceTest();
    }
    else if (command == 'R') {
      resetEncoders();
      Serial.println("\nEncoder counts RESET");
    }
    else if (command == 'P') {
      printStatus();
    }
    else if (command == 'C') {
      moveCell();
    }
  }

  // LIVE ENCODER MONITOR
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 500)
  {
    lastPrint = millis();
    printEncoderValues();
  }
}
