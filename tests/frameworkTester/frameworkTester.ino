/* Programmers: Drue Satterfield, David Strickland
 * Date of creation: 10/11/2016
 * Sub system: arm board
 * 
 * program overhead:
 * Basically this is just a model of the main-ish program with the framework incorporated
 */


#include "JointControlFramework.h"
#include "arm.h"
#include "Ma3Encoder12b.h"
#include "DirectDiscreteHBridge.h"
#include "DRV8388.h"
#include "PIAlgorithm.h"
#include "GenPwmPhaseHBridge.h"

FeedbackDevice* feedbackDevice;
JointInterface * inHerFace;
OutputDevice * controller;
IOAlgorithm * algorithm;

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

  controller = new GenPwmPhaseHBridge(PA_4, PK_2, false);
  inHerFace = new SingleMotorJoint(spd, controller);

  delete inHerFace;
  
  algorithm = new PIAlgorithm(3,3,3);
  inHerFace = new SingleMotorJoint(pos, algorithm, controller, feedbackDevice);
}

CommandResult sendMsgToEndef(uint16_t dataId, size_t dataSize, void * data)
{
  //todo: after deciding how comm between arm and endef boards shall work, implement software to pass the information
}

void passEndefToBase()
{
  //todo: after deciding how comm between arm and endef boards shall work, implement software to pass the information from endefector to base station
}

CommandResult stopArm()
{
  Serial.println("Stop arm");
}

CommandResult moveJ1(int16_t moveValue)
{
  if(moveValue != 0)
  {
  Serial.print("Move J1: ");
  Serial.println(moveValue);
  }
}

CommandResult moveJ2(int16_t moveValue)
{
  if(moveValue != 0)
  {
  Serial.print("Move J2: ");
  Serial.println(moveValue);
  }
}

CommandResult moveJ3(int16_t moveValue)
{
  if(moveValue != 0)
  {
  Serial.print("Move J3: ");
  Serial.println(moveValue);
  }
}

CommandResult moveJ4(int16_t moveValue)
{
  if(moveValue != 0)
  {
  Serial.print("Move J4: ");
  Serial.println(moveValue);
  }
}

CommandResult moveJ5(int16_t moveValue)
{
  if(moveValue != 0)
  {
  Serial.print("Move J5: ");
  Serial.println(moveValue);
  }
}

CommandResult moveJ6(int16_t moveValue)
{
  if(moveValue != 0)
  {
  Serial.print("Move J6: ");
  Serial.println(moveValue);
  }
}
