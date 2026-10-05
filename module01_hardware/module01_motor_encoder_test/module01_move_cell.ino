/*
====================================================
MODULE 01
module01_move_cell.ino
Logic for moving a fixed distance (18 cm target cell size)
====================================================
*/

#include <Arduino.h>
#include "module01.h"

// PULSES_PER_CELL is a variable to be calibrated in Module 02.
// 450 is just a placeholder estimate for testing.
int PULSES_PER_CELL = 388; 

//==================================================
// MOVE 1 CELL (18 cm)
//==================================================

void moveCell()
{
  Serial.println();
  Serial.println("================================");
  Serial.println("MOVING 1 CELL (18 CM)");
  Serial.println("================================");

  resetEncoders();
  motorForward(leftPWM, rightPWM);

  bool reachedTarget = false;

  while (!reachedTarget)
  {
    long left;
    long right;
    
    noInterrupts();
    left = leftPulse;
    right = rightPulse;
    interrupts();
    
    long average = (left + right) / 2;
    
    if (average >= PULSES_PER_CELL)
    {
      reachedTarget = true;
    }
    
    delay(5);
  }
  
  motorStop();
  
  Serial.println("CELL MOVEMENT COMPLETE");
  printEncoderValues();
}
