#include "JointControlFramework.h"

/* Programmers: David strickland, Jake Hasenfratz, Drue Satterfield
 * Date of creation: 2/5/17
 * Sub system: arm board
 * 
 * program overhead:
 * Tests closed loop capabilities of the joint control framework using the PI algorithm construct to move a motor while taking in position data from an encoder
 */

JointInterface* openL1;
JointInterface* openL2;

void setup() {} //fuck you setup

void loop() {

  Serial.begin(9600);
  delay(2000);
  initialize(); //control devices initted in here

  while(1)
  {
    openL1->runOutputControl(-1000);
    openL2->runOutputControl(-1000);
    delay(10);
  }
   
  
}

void initialize()
{
 
  
  VNH5019* controller1 = new VNH5019(PG_1, PK_7, PK_6, false);
  VNH5019* controller2 = new VNH5019(PK_4, PH_0, PH_1, false);

  openL1 = new SingleMotorJoint(spd, controller1);
  openL2 = new SingleMotorJoint(spd, controller2);
}


