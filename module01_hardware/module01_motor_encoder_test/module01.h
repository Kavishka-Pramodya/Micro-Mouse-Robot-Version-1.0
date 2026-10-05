#ifndef MODULE01_H
#define MODULE01_H

#include <Arduino.h>

// Shared variables
extern volatile long leftPulse;
extern volatile long rightPulse;

extern int leftPWM;
extern int rightPWM;

// Function Prototypes
void printEncoderValues();
void printStatus();
void checkBatteryVoltage();
void checkHardwareStatus();
void moveCell();

void motorStop();
void motorForward(int leftSpeed, int rightSpeed);
void motorBackward(int leftSpeed, int rightSpeed);
void resetEncoders();
void forwardTest();
void backwardTest();
void balanceTest();
void printMenu();

#endif
