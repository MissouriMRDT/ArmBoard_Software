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
  uint32_t watchdogTimer_us = 0; //increment this value everytime we don't get a command. When we've waited for a command for longer than our timeout value, stop all arm movement
  Serial.begin(9600);
  delay(2000);
  Serial.println("Entering init");
  initialize(); //control devices initted in here
  Serial.println("Exiting init");
  initPwmRead(PM_0);
  while(1)
  {
    //Serial.println("Hi");
    Serial.println(getOnPeriod_us(PM_0));
    openL -> runOutputControl(-900);

   //while(inHerFace -> runOutputControl(10000) == OutputRunning){delay(1);}
    
  }
}

void initialize()
{
 
  //init control devices
  Serial.println("Setting up feedback device");
  feedbackDevice = new Ma3Encoder12b(PM_0); //used for absolutely nothing, but hey demonstration of setting it up. 
  Serial.println("Setting up algorithm bullshit");
  algorithm = new PIAlgorithm(3,3,.001);


  controller = new DRV8388(PG_1, PK_4, false);
  Serial.println("Setting up to run in her fucking face");
  //inHerFace = new SingleMotorJoint(pos, algorithm, controller, feedbackDevice);
  Serial.println("Alright we did it in her face. Uh, time to leave before she kills us");

  
  openL = new SingleMotorJoint(spd, controller);
}


