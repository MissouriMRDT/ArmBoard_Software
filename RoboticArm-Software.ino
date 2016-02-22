
#include "RoveDynamixel.h"



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

void setup()
{ delay(1000);
  Serial.begin(9600);
  DynamixelInit(&wristRight, MX, 1, 7, 1000000);
  DynamixelInit(&wristLeft, MX, 2, 7, 1000000);
  DynamixelInit(&elbowLeft, MX, 3, 7, 1000000);
  DynamixelInit(&elbowRight, MX, 4, 7, 1000000);
  DynamixelInit(&shoulder, MX, 5, 7, 1000000);
  DynamixelInit(&dynaAll, MX, 0xFE, 7, 1000000);
  
  DynamixelSetMode(dynaAll, Wheel);
}

void loop()
{ 
  if (Serial.available()) {
    char ch = Serial.read();
    switch(ch) {
      case 'q':
        Serial.println("Func 1");
        turnJ5(200);
        delay(1000);
        turnJ5(0);
        delay(5);
        break;
      case 'w':
        Serial.println("Func 2");
        turnJ5(1224);
        delay(1000);
        turnJ5(0);
        delay(5);
        break;
      case 'e':
        Serial.println("Func 3");
        turnJ6(200);
        delay(1000);
        turnJ6(0);
        delay(5);
        break;
      case 'r':
        Serial.println("Func 4");
        turnJ6(1224);
        delay(1000);
        turnJ6(0);
        delay(5);
        break;
      case 'a':
        Serial.println("Func 5");
        turnJ3(200);
        delay(1000);
        turnJ3(0);
        delay(5);
        break;
      case 's':
        Serial.println("Func 6");
        turnJ3(1224);
        delay(1000);
        turnJ3(0);
        delay(5);
        break;
      case 'd':
        Serial.println("Func 7");
        turnJ4(100);
        delay(1000);
        turnJ4(0);
        delay(5);
        break;
      case 'f':
        Serial.println("Func 8");
        turnJ4(1124);
        delay(1000);
        turnJ4(0);
        delay(5);
        break;
      case 'z':
        Serial.println("Func 9");
        turnJ1(100);
        delay(1000);
        turnJ1(0);
        delay(5);
        break;
      case 'x':
        Serial.println("Func 10");
        turnJ1(1124);
        delay(1000);
        turnJ1(0);
        delay(5);
        break;
    } 
  }
  delay(1);
}
