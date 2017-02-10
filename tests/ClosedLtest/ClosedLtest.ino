#include "JointControlFramework.h"

/* Programmers: David strickland, Jake Hasenfratz, Drue Satterfield
 * Date of creation: 2/5/17
 * Sub system: arm board
 * 
 * program overhead:
 * Tests closed loop capabilities of the joint control framework using the PI algorithm construct to move 5 motors while taking in position data from 5 encoders
 * 
 * The test is set up so that the devices are put in place in theory, so that the algorithms have to process the numbers just as much as if
 * they were really there. But in reality no motors are required nor encoders; all that's really required is for the software to think there is and crunch the numbers.
 */

JointInterface * joint1;
JointInterface * joint2;
JointInterface * joint3;
JointInterface * joint4;
JointInterface * joint5;

uint32_t moveValue = 10000;

void setup() {} //fuck you setup

void loop() 
{
  Serial.begin(9600);
  pinMode(PA_4, OUTPUT);
  delay(1000);
  initialize(); //control devices initted in here

  //use an oscilloscope and compare the amount of time high to the amount of time low. They should be the same except for displacement by the output control's calculations. SO the
  //difference between the two is how long it takes to process each of them
  while(1)
  {
    digitalWrite(PA_4, HIGH);
    delayMicroseconds(400);
    joint1 -> runOutputControl(moveValue);  
    joint2 -> runOutputControl(moveValue);  
    joint3 -> runOutputControl(moveValue);  
    joint4 -> runOutputControl(moveValue);  
    joint5 -> runOutputControl(moveValue);  
    digitalWrite(PA_4, LOW);
    delayMicroseconds(400);
  }
   
  
}

void initialize()
{
 
  //init control devices
  Ma3Encoder12b* fb1 = new Ma3Encoder12b(PM_0); 
  Ma3Encoder12b* fb2 = new Ma3Encoder12b(PM_2); 
  Ma3Encoder12b* fb3 = new Ma3Encoder12b(PM_6); 
  Ma3Encoder12b* fb4 = new Ma3Encoder12b(PD_2); 
  Ma3Encoder12b* fb5 = new Ma3Encoder12b(PM_4); 
  
  PIAlgorithm* alg1 = new PIAlgorithm(20,3,.04);
  alg1 -> setHardStopPositions(150, 210);

  PIAlgorithm* alg2 = new PIAlgorithm(19,2,.04);
  alg2 -> setHardStopPositions(150, 210);

  PIAlgorithm* alg3 = new PIAlgorithm(21,1,.04);
  alg3 -> setHardStopPositions(150, 210);

  PIAlgorithm* alg4 = new PIAlgorithm(21,4,.04);
  alg4 -> setHardStopPositions(150, 210);

  PIAlgorithm* alg5 = new PIAlgorithm(18,5,.04);
  alg5 -> setHardStopPositions(150, 210);
  
  VNH5019* ctl1 = new VNH5019(PG_1, PK_7, PK_6, false);
  VNH5019* ctl2 = new VNH5019(PK_4, PL_4, PL_5, false);
  VNH5019* ctl3 = new VNH5019(PK_5, PL_0, PL_2, false);
  VNH5019* ctl4 = new VNH5019(PF_1, PL_3, PN_2, false);
  VNH5019* ctl5 = new VNH5019(PF_2, PN_3, PP_2, false);

  joint1 = new SingleMotorJoint(pos, alg1, ctl1, fb1);
  joint2 = new SingleMotorJoint(pos, alg2, ctl2, fb2);
  joint3 = new SingleMotorJoint(pos, alg3, ctl3, fb3);
  joint4 = new SingleMotorJoint(pos, alg4, ctl4, fb4);
  joint5 = new SingleMotorJoint(pos, alg5, ctl5, fb5);
}


