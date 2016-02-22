#include "RoveBoard.h"
#include "RoveDynamixel.h"
#include "Arm.h"


void setup()
{ 
  Serial.begin(9600);
  armInit();
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
