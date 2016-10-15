#include <RoveComm.h>
#include <stdint.h>
#include <Ethernet.h>

typedef enum ArmCommandIds
{
  ArmStop = 0x320,
  ArmJ1 = 0x321,
  ArmJ2 = 0x322,
  ArmJ3 = 0x323,
  ArmJ4 = 0x324,
  ArmJ5 = 0x325,
  ArmJ6 = 0x326
} ArmCommandIds;

typedef enum EndefCommandIds
{
  Gripper,
  Drill 
} EndefCommandIds; 

void setup() {
  // put your setup code here, to run once:
  roveComm_Begin(192, 168, 1, 83);
  Serial.begin(9600);
  Ethernet.enableActivityLed();
}

void loop() {
  // put your main code here, to run repeatedly: 
  int16_t dataVal = 1;
  while(1)
  {
    roveComm_SendMsg(ArmJ1, sizeof(dataVal), &dataVal);
    Serial.println("Sent packet");
    delay(1000);
  }
}
