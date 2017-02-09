#include "JointControlFramework.h"

/* Programmers: David strickland, Jake Hasenfratz, Drue Satterfield
 * Date of creation: 2/5/17
 * Sub system: arm board
 * 
 * program overhead:
 * Tests closed loop capabilities of the joint control framework using the PI algorithm construct to move a motor while taking in position data from an encoder
 */

JointInterface * inHerFace;
JointInterface* openL;

void setup() {} //fuck you setup

void loop() {

  uint16_t commandId;
  size_t commandSize;
  int16_t commandData;

  Serial.begin(9600);
  delay(2000);
  Serial.println("Entering init");
  initialize(); //control devices initted in here
  Serial.println("Exiting init");
  
  //initPwmRead(PM_0);
  while(1)
  {
    //Serial.println("Hi");
    Serial.println(getOnPeriod_us(PM_0));
    //inHerFace -> runOutputControl(500);
   delay(200);
   while((inHerFace -> runOutputControl(58000) == OutputRunning)){delay(1);}

  while(1){}
   
  }
}

void initialize()
{
 
  //init control devices
  Ma3Encoder12b* feedbackDevice = new Ma3Encoder12b(PM_0); 
  
  PIAlgorithm* piAlgorithm = new PIAlgorithm(20,3,.001);
  piAlgorithm -> setHardStopPositions(150, 210);
  
  VNH5019* controller = new VNH5019(PG_1, PK_7, PK_6, false);

  inHerFace = new SingleMotorJoint(pos, piAlgorithm, controller, feedbackDevice);

  //openL = new SingleMotorJoint(spd, controller);
}


