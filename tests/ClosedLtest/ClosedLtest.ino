#include "JointControlFramework.h"

/* Programmers: Drue Satterfield, David Strickland
 * Date of creation: 10/11/2016
 * Sub system: arm board
 * 
 * program overhead:
 * Basically this is just a model of the main-ish program with the framework incorporated
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
  while(1)
  {
    Serial.println("Hi");

   while(inHerFace -> runOutputControl(100) == OutputRunning)
    {
      Serial.println("Fuck");
      inHerFace -> runOutputControl(100);
      delay(1);
    }
    
  }
  
  /*while(1) //main program loop. Listen for communications from the endefector or from base station, and proceed based on that transmission 
  {
    commandSize = 0;
    commandId = 0;
    commandData = 0;//reset variables
    
    roveComm_GetMsg(&commandId, &commandSize, &commandData);
    passEndefToBase();
    if((commandSize == 1 || commandSize == 2) && commandId != 0) //command packets come in 1 or 2 bytes. If it's any other size, there was probably a comm error
    {
      watchdogTimer_us = 0; //reset watchdog timer since we received a command
      
      if(commandId == ArmStop || commandId == LY_ArmStop)
      {
        result = stopArm();
      }
      else if(commandId == ArmJ1 || commandId == LY_ArmJ1)
      {
        result = moveJ1(commandData);  
      }
      else if(commandId == ArmJ2 || commandId == LY_ArmJ2)
      {
        result = moveJ2(commandData);
      }
      else if(commandId == ArmJ3 || commandId == LY_ArmJ3)
      {
        result = moveJ3(commandData);
      }
      else if(commandId == ArmJ4 || commandId == LY_ArmJ4)
      {
        result = moveJ4(commandData);
      }
      else if(commandId == ArmJ5 || commandId == LY_ArmJ5)
      {
        result = moveJ5(commandData);
      }
      else if(commandId == ArmJ6 || commandId == LY_ArmJ6)
      {
        result = moveJ6(commandData);
      }
      else if(commandId == Gripper || commandId == Drill) //if the message is meant for the endefector, pass it along
      {
        sendMsgToEndef(commandId, commandSize, &commandData);
      }
      
      if(result != Success)
      {
        //todo: if there's ever any telemetry about what to do when the command isn't successful, this is where we'll send telemetry back about it
      }
    }//end if
    //if no messages were recieved, increment our watchdog counter. If the counter has gone over a certain period of time since we last got a transmission, cease all movement.
    //This is to keep the arm from committing suicide on the environment/the rover if communications ever get interrupted while it's in the middle of moving
    else 
    {
      uint8_t microsecondDelay = 10;
      delayMicroseconds(microsecondDelay);
      watchdogTimer_us += microsecondDelay;
      if(watchdogTimer_us >= WATCHDOG_TIMEOUT_US) //if more than our timeout period has passed, then kill arm movement
      {
        stopArm();
        watchdogTimer_us = 0;
      }
    }//end else
  }//end while
 */ 
}

void initialize()
{
 
  //init control devices
  /*Serial.println("Setting up feedback device");
  feedbackDevice = new Ma3Encoder12b(PM_0); //used for absolutely nothing, but hey demonstration of setting it up. 
  Serial.println("Setting up this shitty h bridge");
  controller = new DirectDiscreteHBridge(PA_4, PK_2, false);

  Serial.println("Setting up algorithm bullshit");
  algorithm = new PIAlgorithm(3,3,3);*/

  Serial.println("Setting up to run in her fucking face");
  //inHerFace = new SingleMotorJoint(pos, algorithm, controller, feedbackDevice);
  Serial.println("Alright we did it in her face. Uh, time to leave before she kills us");

  controller = new DirectDiscreteHBridge(PG_1, PK_4, false);
  openL = new SingleMotorJoint(spd, controller);
}


