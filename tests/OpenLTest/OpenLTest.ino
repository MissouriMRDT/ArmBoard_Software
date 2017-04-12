#include "JointControlFramework.h"

/* Programmers: David strickland, Jake Hasenfratz, Drue Satterfield
 * Date of creation: 2/5/17
 * Sub system: arm board
 * 
 * program overhead:
 * Tests closed loop capabilities of the joint control framework using the PI algorithm construct to move a motor while taking in position data from an encoder
 */

JointInterface* J1_tilt;
JointInterface* J2_rot;
JointInterface* Gripper;

void setup() {} //fuck you setup

void loop() {

  Serial.begin(9600);
  delay(2000);
  initialize(); //control devices initted in here

  while(1)
  {
    //J1_tilt->runOutputControl(-1000);
    //J2_rot->runOutputControl(-1000);
    //delay(4000);
    //J1_tilt->runOutputControl(1000);
    //J2_rot->runOutputControl(1000);
    //delay(4000);

    Gripper->runOutputControl(-1000);
  }
   
  
}

void initialize()
{
 
  pinMode(PA_7, OUTPUT);
  pinMode(PL_2, OUTPUT);
  pinMode(PE_4, OUTPUT);
  pinMode(PP_3, OUTPUT);

  digitalWrite(PA_7, HIGH);
  digitalWrite(PL_2, HIGH);
  digitalWrite(PE_4, HIGH);
  digitalWrite(PP_3, LOW);
  
  OutputDevice* controller1 = new DRV8388(PG_1, PP_5, false);
  OutputDevice* controller2 = new DRV8388(PF_3, PL_3, true);
  OutputDevice* GripMot = new DRV8388(PF_2, PQ_2, true);

  J1_tilt = new TiltJoint(spd, controller2, controller1);
  J2_rot = new RotateJoint(spd, controller2, controller1);

  J1_tilt->coupleJoint(J2_rot);

  Gripper = new SingleMotorJoint(spd, GripMot);
}


