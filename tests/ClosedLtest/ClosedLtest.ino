#include "JointControlFramework.h"

/* Programmers: David strickland, Jake Hasenfratz, Drue Satterfield
 * Date of creation: 2/5/17
 * Sub system: arm board
 * 
 * program overhead:
 * Tests closed loop capabilities of the joint control framework using the PI algorithm construct to move a motor while taking in position data from an encoder
 */

FeedbackDevice* feedbackDevice;
JointInterface * inHerFace;
OutputDevice * controller;
IOAlgorithm * algorithm;

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

  inHerFace = new SingleMotorJoint(pos, algorithm, controller, feedbackDevice);

  
  //openL = new SingleMotorJoint(spd, controller);
  
  initPwmRead(PM_0);
  while(1)
  {
    //Serial.println("Hi");
    Serial.println(getOnPeriod_us(PM_0));
    //inHerFace -> runOutputControl(500);

   while((inHerFace -> runOutputControl(70000) == OutputRunning)){delay(1);}

  }
}

void initialize()
{
 
  //init control devices
  Serial.println("Setting up feedback device");
  feedbackDevice = new Ma3Encoder12b(PM_0); //used for absolutely nothing, but hey demonstration of setting it up. 
  Serial.println("Setting up algorithm bullshit");
  algorithm = new PIAlgorithm(10,0,.001);


  controller = new VNH5019(PG_1, PK_7, PK_6, false);
}


