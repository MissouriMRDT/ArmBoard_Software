#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <Servo.h>

#include "RoveBoard.h"
#include "RoveEthernet.h"

#include "RoveComm.h"
#include "RoveDynamixel.h"

#include "Arm.h"


#define TURN_J1_ID 205
#define TURN_J2_ID 207
#define TURN_J3_ID 204
#define TURN_J4_ID 203
#define TURN_J5_ID 202
#define TURN_J6_ID 201

#define ARM_STOP 206

#define ARM_POWER_ON    240
#define ARM_POWER_OFF   241

uint16_t dataID = 0;
size_t size = 0;
char data[8];
int counter;

void setup()
{ 
  roveComm_Begin(192,168,1,131);
  Serial.begin(9600);
  Ethernet.enableLinkLed();
  Ethernet.enableActivityLed();
  armInit();
  pinMode(PK_3, INPUT);
  delay(1000);
  Serial.println(analogRead(PK_3));
}


void loop()
{ 
  roveComm_GetMsg(&dataID, &size, data);
  
  if (dataID != 0) {
    switch(dataID) {
      case TURN_J1_ID:
        turnJ1(*(int16_t*)(&data));
        break;
      case TURN_J2_ID:
        //turnJ2(*(int16_t*)(data));
        break;
      case TURN_J3_ID:
        turnJ3(*(int16_t*)(data));
        break;
      case TURN_J4_ID:
        turnJ4(*(int16_t*)(data));
        break;
      case TURN_J5_ID:
        turnJ5(*(int16_t*)(data));
        break;
      case TURN_J6_ID:
        turnJ6(*(int16_t*)(data));
        break;
      case ARM_STOP:
        stopAllMotors();
        break;
      case ARM_POWER_ON:
        AllPowerON();
        Serial.println("Power ON");
        break;
      case ARM_POWER_OFF:
        AllPowerOff();
        Serial.println("Power Off");
        break;
      default:
        break;
    }  
    counter = 0;
  } else {
    counter++;
    delay(5);
  }
  if (counter > 25) {
    stopAllMotors();
    counter = 0;
  }
  delay(1);
}
