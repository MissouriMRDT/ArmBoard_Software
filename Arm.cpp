#include "Arm.h"

Dynamixel shoulder, elbowLeft, elbowRight, wristLeft, wristRight, dynaAll;

void turnJ1(int speed) {
  uint16_t dynaSpeed = ((speed < 0) ? -1 : 1) / 1000 * 1023;
  if (speed < 0)
    dynaSpeed = dynaSpeed | 1024;
 
  dynaSpeed = speed;
  DynamixelSpinWheel(shoulder, 1023, dynaSpeed);
}

void turnJ3(int speed) {
  uint16_t dynaSpeed = ((speed < 0) ? -1 : 1) / 1000 * 1023;
  if (speed < 0)
    dynaSpeed = dynaSpeed | 1024;
  dynaSpeed = speed;
  DynamixelSpinWheel(elbowLeft, 1023, dynaSpeed);
  DynamixelSpinWheel(elbowRight, 1023, dynaSpeed);
}

void turnJ4(int speed) {
  uint16_t dynaSpeed = ((speed < 0) ? -1 : 1) / 1000 * 1023;
  if (speed < 0)
    dynaSpeed = dynaSpeed | 1024;
  dynaSpeed = speed;
  Serial.println(DynamixelSpinWheel(elbowLeft, 1023, dynaSpeed));
  DynamixelSpinWheel(elbowRight, 1023, dynaSpeed ^ 1024);
}

void turnJ5(int speed) {
  uint16_t dynaSpeed = ((speed < 0) ? -1 : 1) / 1000 * 1023;
  if (speed < 0)
    dynaSpeed = dynaSpeed | 1024;
  dynaSpeed = speed;
  DynamixelSpinWheel(wristLeft, 1023, dynaSpeed);
  DynamixelSpinWheel(wristRight, 1023, dynaSpeed);
}

void turnJ6(int speed) {
  uint16_t dynaSpeed = ((speed < 0) ? -1 : 1) / 1000 * 1023;
  if (speed < 0)
    dynaSpeed = dynaSpeed | 1024;
   dynaSpeed = speed;
  DynamixelSpinWheel(wristLeft, 1023, dynaSpeed);
  DynamixelSpinWheel(wristRight, 1023, dynaSpeed ^ 1024);
}

