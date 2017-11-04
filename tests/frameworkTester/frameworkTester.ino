#include "JointPeriodicUpdater.h"

/* Programmers: Drue Satterfield, David Strickland
 * Date of creation: 10/11/2016
 * Sub system: arm board
 * 
 * program overhead:
 * Basically this is just a model of the main-ish program with the framework incorporated
 */


/*#include "JointControlFramework.h"
#include "arm.h"
#include "Ma3Encoder12b.h"
#include "DirectDiscreteHBridge.h"
#include "DynamixelController.h"
#include "GenPwmPhaseHBridge.h"
#include "RCContinuousServo.h"
#include "VNH5019.h"
#include "Sdc2130.h"
#include "GenPwmPhaseHBridge.h"
#include "PIAlgorithm.h"

FeedbackDevice* feedbackDevice;
JointInterface * inHerFace;
OutputDevice * controller;

void setup() {} //fuck you setup

void loop() {
  CommandResult result;
  uint16_t commandId;
  size_t commandSize;
  int16_t commandData;
  uint32_t watchdogTimer_us = 0; //increment this value everytime we don't get a command. When we've waited for a command for longer than our timeout value, stop all arm movement
  
  initialize(); //control devices initted in here

  while(1)
  {
    inHerFace -> runOutputControl(100);
    delay(1000);
  }
  
}

void initialize()
{
  roveComm_Begin(192, 168, 1, 131);
  Serial.begin(9600);

  //init control devices
  int pinInFwd = 1;
  int pinInRev = 2;
  feedbackDevice = new Ma3Encoder12b(PA_2); //used for absolutely nothing, but hey demonstration of setting it up. 
  controller = new DirectDiscreteHBridge(pinInFwd, pinInRev, 0);
  inHerFace = new SingleMotorJoint(spd, controller);
  
  delete inHerFace;
  delete controller;

  controller = new GenPwmPhaseHBridge(PA_4, PK_2, PA_2, true, false);
  inHerFace = new SingleMotorJoint(spd, controller);

  delete inHerFace;
  
  PIAlgorithm* algorithm = new PIAlgorithm(3,3,3);
  inHerFace = new SingleMotorJoint(pos, algorithm, controller, feedbackDevice);

  controller->setPower(true);
}*/
