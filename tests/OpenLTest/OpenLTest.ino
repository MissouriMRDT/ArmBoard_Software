#include "JointControlFramework.h"
#include "GenPwmPhaseHBridge.h"

/* Programmers: David strickland, Jake Hasenfratz, Drue Satterfield
 * Date of creation: 2/5/17
 * Sub system: arm board
 * 
 * program overhead:
 * Tests open loop capabilities of the joint control framework
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
    Gripper->runOutputControl(1000);
  }
   
  
}

void initialize()
{
  pinMode(PE_4, OUTPUT); //enable main power output
  digitalWrite(PE_4, HIGH);
  
  OutputDevice* controller1 = new GenPwmPhaseHBridge(PG_1, PP_5, PA_7, true, false);
  OutputDevice* controller2 = new GenPwmPhaseHBridge(PF_3, PL_3, PL_2, true, true);
  OutputDevice* controller3 = new GenPwmPhaseHBridge(PK_5, PK_6, PH_1, true, true);
  OutputDevice* controller4 = new GenPwmPhaseHBridge(PK_4, PA_5, PD_5, true, true);
  OutputDevice* controller5 = new GenPwmPhaseHBridge(PG_0, PQ_0, PK_3, true, true);
  OutputDevice* gripMot = new GenPwmPhaseHBridge(PF_2, PQ_2, PP_3, false, true);
  controller1->setPower(true);
  controller2->setPower(true);
  controller3->setPower(true);
  controller4->setPower(true);
  controller5->setPower(true);
  gripMot->setPower(true);

  J1_tilt = new TiltJoint(spd, controller2, controller1);
  J2_rot = new RotateJoint(spd, controller2, controller1);

  J1_tilt->coupleJoint(J2_rot);

  Gripper = new SingleMotorJoint(spd, gripMot);

  genJoint = new SingleMotorJoint(spd, controller1);
  genJoint2 = new SingleMotorJoint(spd, controller2);
  genJoint3 = new SingleMotorJoint(spd, controller3);
  genJoint4 = new SingleMotorJoint(spd, controller4);
  genJoint5 = new SingleMotorJoint(spd, controller5);
}


