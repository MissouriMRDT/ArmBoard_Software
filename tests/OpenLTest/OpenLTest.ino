#include "JointControlFramework.h"
#include "DRV8388.h"

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
JointInterface* genJoint;
JointInterface* genJoint2;
JointInterface* genJoint3;
JointInterface* genJoint4;
JointInterface* genJoint5;
IOAlgorithm* alg;
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

    genJoint->runOutputControl(-1000);
    genJoint2->runOutputControl(1000);
    genJoint3->runOutputControl(1000);
    genJoint4->runOutputControl(1000);
    genJoint5->runOutputControl(1000);
  }
   
  
}

void initialize()
{
  pinMode(PA_7, OUTPUT);
  pinMode(PL_2, OUTPUT);
  pinMode(PE_4, OUTPUT);
  pinMode(PP_3, OUTPUT);
  pinMode(PH_1, OUTPUT);
  pinMode(PD_5, OUTPUT);
  pinMode(PK_3, OUTPUT);

  digitalWrite(PD_5, HIGH);
  digitalWrite(PH_1, HIGH);
  digitalWrite(PA_7, HIGH);
  digitalWrite(PL_2, HIGH);
  digitalWrite(PE_4, HIGH);
  digitalWrite(PK_3, HIGH);
  digitalWrite(PP_3, LOW);
  
  OutputDevice* controller1 = new DRV8388(PG_1, PP_5, false);
  OutputDevice* controller2 = new DRV8388(PF_3, PL_3, true);
  OutputDevice* controller3 = new DRV8388(PK_5, PK_6, true);
  OutputDevice* controller4 = new DRV8388(PK_4, PA_5, true);
  OutputDevice* controller5 = new DRV8388(PG_0, PQ_0, true);
  OutputDevice* GripMot = new DRV8388(PF_2, PQ_2, true);

  J1_tilt = new TiltJoint(spd, controller2, controller1);
  J2_rot = new RotateJoint(spd, controller2, controller1);

  J1_tilt->coupleJoint(J2_rot);

  Gripper = new SingleMotorJoint(spd, GripMot);

  genJoint = new SingleMotorJoint(spd, controller1);
  genJoint2 = new SingleMotorJoint(spd, controller2);
  genJoint3 = new SingleMotorJoint(spd, controller3);
  genJoint4 = new SingleMotorJoint(spd, controller4);
  genJoint5 = new SingleMotorJoint(spd, controller5);
}


