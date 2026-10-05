/*
====================================================
MODULE 01
module01_diagnostics.ino
Hardware Diagnostics, Printing, and Status Checks
====================================================
*/

#include <Arduino.h>
#include "module01.h"

//==================================================
// PRINT ENCODER VALUES
//==================================================

void printEncoderValues()
{
  long left;
  long right;

  noInterrupts();
  left  = leftPulse;
  right = rightPulse;
  interrupts();

  long average = (left + right) / 2;

  Serial.print("Left Pulse : ");
  Serial.print(left);

  Serial.print("    Right Pulse : ");
  Serial.print(right);

  Serial.print("    Average : ");
  Serial.println(average);
}

//==================================================
// CHECK BATTERY VOLTAGE
//==================================================

void checkBatteryVoltage()
{
  // TODO: Implement actual battery voltage monitoring (ADC read)
  // This is a placeholder structure as per requirements
  Serial.println("Battery Voltage : [Placeholder - Not calibrated]");
}

//==================================================
// CHECK HARDWARE STATUS
//==================================================

void checkHardwareStatus()
{
  // TODO: Add any other overall hardware diagnostics checks here
  Serial.println("Hardware Diagnostics : [Placeholder - All OK]");
}

//==================================================
// STATUS
//==================================================

void printStatus()
{
  Serial.println();
  Serial.println("================================");
  Serial.println("MODULE 01 STATUS");
  Serial.println("================================");

  Serial.print("Left PWM  : ");
  Serial.println(leftPWM);

  Serial.print("Right PWM : ");
  Serial.println(rightPWM);

  printEncoderValues();
  
  checkBatteryVoltage();
  checkHardwareStatus();

  Serial.println("================================");
}
