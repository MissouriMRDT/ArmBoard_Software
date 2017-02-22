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
JointInterface* openL3;
JointInterface* openL4;
JointInterface* openL5;

void setup() {} //fuck you setup

void loop() {
  initialize(); //control devices initted in here

    openL1->runOutputControl(-1000);
    openL2->runOutputControl(-1000);
    openL3->runOutputControl(-1000);
    openL4->runOutputControl(-1000);
    openL5->runOutputControl(-1000);

    while(1);
}

void initialize()
{

  pinMode(PA_7, OUTPUT);
  pinMode(PL_2, OUTPUT);
  pinMode(PH_1, OUTPUT);
  pinMode(PD_5, OUTPUT);
  pinMode(PK_3, OUTPUT);

  digitalWrite(PA_7, HIGH);
  digitalWrite(PL_2, HIGH);
  digitalWrite(PH_1, HIGH);
  digitalWrite(PD_5, HIGH);
  digitalWrite(PK_3, HIGH);

  pinMode(PM_7, INPUT);
  pinMode(PL_1, INPUT);
  pinMode(PH_0, INPUT);
  pinMode(PP_4, INPUT);
  pinMode(PK_2, INPUT);
  pinMode(PE_4, INPUT);

  pinMode(PE_4, OUTPUT);
  digitalWrite(PE_4, HIGH);
  
  DRV8388* controller1 = new DRV8388(PG_1, PP_5, false);
  DRV8388* controller2 = new DRV8388(PF_3, PL_3, false);
  DRV8388* controller3 = new DRV8388(PK_5, PK_6, false);
  DRV8388* controller4 = new DRV8388(PK_4, PA_5, false);
  DRV8388* controller5 = new DRV8388(PG_0, PQ_0, false);

  openL1 = new SingleMotorJoint(spd, controller1);
  openL2 = new SingleMotorJoint(spd, controller2);
  openL3 = new SingleMotorJoint(spd, controller3);
  openL4 = new SingleMotorJoint(spd, controller4);
  openL5 = new SingleMotorJoint(spd, controller5);
}


