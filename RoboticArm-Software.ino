#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <Servo.h>
#include <driverlib/sysctl.h>
#include <driverlib/watchdog.h>
#include <driverlib/rom_map.h>


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

#define ANGLE_MOVEMENT 784
#define GRIPPER_MOVEMENT 208

#define ARM_POWER_ON    240
#define ARM_POWER_OFF   241

#define ARM_POWER_ALL     816
#define ARM_POWER_MAIN    817
#define ARM_POWER_J1      818
#define ARM_POWER_J2      819
#define ARM_POWER_ELBOW   820
#define ARM_POWER_WRIST   821
#define ARM_POWER_ENDEFF  822

#define POSITION_REQUEST  793
#define POSITION_ANSWER   792

uint16_t dataID = 0;
size_t size = 0;
char data[256];
float pos[6];

const uint32_t ROVECOMM_WATCHDOG_TIMEOUT_TICKS = 16400000; 
const uint32_t ROVECOMM_WATCHDOG = WATCHDOG1_BASE;

void roveWatchdogClear()
{
  WatchdogIntClear(ROVECOMM_WATCHDOG);
}//end fnctn

void RoveCommWatchdog_begin(void (*roveWatchdogIntHandler)(void), const uint32_t time_out_millis)
{   
  // Enable Watchdog Timer 1; supposedly timer 0 has a conflict in Energia, unconfirmed
  SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG1);  
  WatchdogUnlock(ROVECOMM_WATCHDOG);
  WatchdogReloadSet(ROVECOMM_WATCHDOG, ROVECOMM_WATCHDOG_TIMEOUT_TICKS);
  
  WatchdogIntRegister(ROVECOMM_WATCHDOG, roveWatchdogIntHandler);
  
  WatchdogIntEnable(ROVECOMM_WATCHDOG);
  WatchdogResetDisable(ROVECOMM_WATCHDOG);
  WatchdogEnable(ROVECOMM_WATCHDOG);

}

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
  //RoveCommWatchdog_begin(&armReinit, 3);
}


void loop()
{ 
  //getEncoderValues();
  roveComm_GetMsg(&dataID, &size, data);

  if (dataID != 0) {
    switch(dataID) {
      case TURN_J1_ID:
        //Serial.println("J1");
        turnJ1(*(int16_t*)(data));
        break;
      case TURN_J2_ID:
        //Serial.println("J2");
        turnJ2(*(int16_t*)(data));
        break;
      case TURN_J3_ID:
        //Serial.println("J3");
        turnJ3(*(int16_t*)(data));
        break;
      case TURN_J4_ID:
        //Serial.println("J4");
        turnJ4(*(int16_t*)(data));
        break;
      case TURN_J5_ID:
        //Serial.println("J5");
        turnJ5(*(int16_t*)(data));
        break;
      case TURN_J6_ID:
        //Serial.println("J6");
        turnJ6(*(int16_t*)(data));
        break;
      case ARM_STOP:
        stopAllMotors();
        break;
      case ANGLE_MOVEMENT:
       moveToAngle((float*)data);
       break;
      case GRIPPER_MOVEMENT:
        movegripper(*(int16_t*)(data)); 
        break;
      case ARM_POWER_ON:
        AllPowerON();
        Serial.println("Power ON");
        break;
      case ARM_POWER_OFF:
        AllPowerOff();
        Serial.println("Power Off");
        break;
      case ARM_POWER_ALL:
        AllPower(*(uint8_t*)data);
        Serial.println("Power All");
        break;
      case ARM_POWER_MAIN:
        MainPower(*(uint8_t*)data);
        Serial.println("Power Main");
        break;
      case ARM_POWER_J1:
        J1Power(*(uint8_t*)data);
        Serial.println("Power J1");
        break;
      case ARM_POWER_J2:
        J2Power(*(uint8_t*)data);
        Serial.println("Power J2");
        break;
      case ARM_POWER_WRIST:
        WristPower(*(uint8_t*)data);
        Serial.println("Power Wrist");
        break;
      case ARM_POWER_ELBOW:
        ElbowPower(*(uint8_t*)data);
        Serial.println("Power Elbow");
        break;
      case ARM_POWER_ENDEFF:
        ElbowPower(*(uint8_t*)data);
        Serial.println("Power Endeff");
        break;
      case POSITION_REQUEST:
        Serial.println("Position Request");
        storePosition(pos);
        roveComm_SendMsg(POSITION_ANSWER, sizeof(float) *6, pos);
        break;
      default:
        break;
    }  
    //roveWatchdogClear();
  } else {
    
  }
  
  checkPosition();
  //delay(500);
  /*
  Serial.print("Main Current: ");
  Serial.println(getMainCurrent());
  Serial.print("EndEffector Current: ");
  Serial.println(getEndEffCurrent());
  */
}

