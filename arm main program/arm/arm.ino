#include "JointControlFramework.h"
#include "arm.h";

JointInterface* joint1;
JointInterface* joint2;
JointInterface* joint3;
JointInterface* joint4;
JointInterface* joint5;


void setup() {} //useless

void loop() {
  CommandResult result;
  uint16_t commandId;
  size_t commandSize;
  int16_t commandData;
  uint32_t watchdogTimer_us = 0; //increment this value everytime we don't get a command. When we've waited for a command for longer than our timeout value, stop all arm movement

  initialize(); //control devices initted in here

  while(1) //main program loop. Listen for communications from the endefector or from base station, and proceed based on that transmission
  {
    commandSize = 0;
    commandId = 0;
    commandData = 0;//reset variables

    roveComm_GetMsg(&commandId, &commandSize, &commandData);
    passEndefToBase();
    if(commandId != 0) //command packets come in 1 or 2 bytes. If it's any other size, there was probably a comm error
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

      //if the message is meant for the endefector, pass it along
      else if(commandId == MoveGripper || commandId == TurnCap || commandId == EnableGripperPower || commandId == DisableGripperPower)
      {
        sendMsgToEndef(commandId, commandSize, commandData);
      }



      if(result != Success)
      {
        //todo: if there's ever any telemetry about what to do when the command isn't successful, this is where we'll send telemetry back about it
        // I'm not quite sure how to do this. Somebody else's addition would be very helpful.
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

    if(checkOvercurrent())
    {
      masterPowerDisable();
      //TODO: send telemetry back to base station
    }

  }//end while

}

void initialize()
{
  roveComm_Begin(IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3]);
  Serial.begin(9600);
  Serial6.begin(GRIPPER_COMM_BAUD_RATE);
  // TODO
  OutputDevice * dev1 = new DRV8388(MOT1_PWN_PIN, HBRIDGE1_PHASE_PIN,false);
  OutputDevice * dev2 = new DRV8388(MOT2_PWN_PIN, HBRIDGE2_PHASE_PIN,true);
  OutputDevice * dev3 = new DRV8388(MOT3_PWN_PIN, HBRIDGE3_PHASE_PIN,false);
  OutputDevice * dev4 = new DRV8388(MOT4_PWN_PIN, HBRIDGE4_PHASE_PIN,true);
  OutputDevice * dev5 = new DRV8388(MOT5_PWN_PIN, HBRIDGE5_PHASE_PIN,false);

  pinMode(HBRIDGE1_NFAULT_PIN,INPUT);
  pinMode(HBRIDGE2_NFAULT_PIN,INPUT);
  pinMode(HBRIDGE3_NFAULT_PIN,INPUT);
  pinMode(HBRIDGE4_NFAULT_PIN,INPUT);
  pinMode(HBRIDGE5_NFAULT_PIN,INPUT);

  pinMode(HBRIDGE1_NSLEEP_PIN,OUTPUT);
  pinMode(HBRIDGE2_NSLEEP_PIN,OUTPUT);
  pinMode(HBRIDGE3_NSLEEP_PIN,OUTPUT);
  pinMode(HBRIDGE4_NSLEEP_PIN,OUTPUT);
  pinMode(HBRIDGE5_NSLEEP_PIN,OUTPUT);

  pinMode(OC_NFAULT_PIN,INPUT);
  pinMode(POWER_LINE_CONTROL_PIN,OUTPUT);

  joint1 = new RotateJoint(spd, dev1, dev2);
  joint2 = new TiltJoint(spd, dev1, dev2);
  joint3 = new SingleMotorJoint(spd, dev3);
  joint4 = new RotateJoint(spd, dev4, dev5);
  joint5 = new TiltJoint(spd, dev4, dev5);

  joint1 -> coupleJoint(joint2);
  joint4 -> coupleJoint(joint5);

  masterPowerDisable();

  enableAllMotors();

}

bool checkOvercurrent()
{
  if(readMasterCurrent() > CURRENT_LIMIT)
  {
    return(true);
  }
  else
  {
    return(false);
  }
}

CommandResult sendMsgToEndef(uint16_t dataId, size_t dataSize, int16_t data)
{
  if(dataId == MoveGripper)
  {
    Serial6.write(Serial_MoveGripper);
    Serial6.write((int8_t)data);
    Serial6.write((int8_t)(data>>8));
  }
  else if(dataId == TurnCap)
  {
    Serial6.write(Serial_TurnCap);
    Serial6.write((int8_t)data);
    Serial6.write((int8_t)(data>>8));
  }
  else if(dataId == EnableGripperPower)
  {
    Serial6.write(Serial_EnableGripperPower);
  }
  else if(dataId == DisableGripperPower)
  {
    Serial6.write(Serial_DisableGripperPower);
  }
}

void passEndefToBase()
{
  uint16_t messageId = Serial6.read(); //read returns -1 if there's nothing to send

  if(messageId == Serial_Overcurrent)
  {
    roveComm_SendMsg(GripperOvercurrent, 0, 0);
  }
}

CommandResult masterPowerEnable()
{
  digitalWrite(POWER_LINE_CONTROL_PIN, HIGH);
}

CommandResult masterPowerDisable()
{
  digitalWrite(POWER_LINE_CONTROL_PIN, LOW);
}



void enableAllMotors()
{
  enableM1();
  enableM2();
  enableM3();
  enableM4();
  enableM5();

}

void disableAllMotors()
{
  disableM1();
  disableM2();
  disableM3();
  disableM4();
  disableM5();
}

void enableM1()
{
  digitalWrite(HBRIDGE1_NSLEEP_PIN,HIGH);
}

void enableM2()
{
  digitalWrite(HBRIDGE2_NSLEEP_PIN,HIGH);
}

void enableM3()
{
  digitalWrite(HBRIDGE3_NSLEEP_PIN,HIGH);
}

void enableM4()
{
  digitalWrite(HBRIDGE4_NSLEEP_PIN,HIGH);
}

void enableM5()
{
  digitalWrite(HBRIDGE5_NSLEEP_PIN,HIGH);
}

void disableM1()
{
  digitalWrite(HBRIDGE1_NSLEEP_PIN,LOW);
}

void disableM2()
{
  digitalWrite(HBRIDGE2_NSLEEP_PIN,LOW);
}

void disableM3()
{
  digitalWrite(HBRIDGE3_NSLEEP_PIN,LOW);
}

void disableM4()
{
  digitalWrite(HBRIDGE4_NSLEEP_PIN,LOW);
}

void disableM5()
{
  digitalWrite(HBRIDGE5_NSLEEP_PIN,LOW);
}

float readMasterCurrent()
{
  //Note this is only an estimation, as it assumes the VCC is currently 3.3V when in reality it tends to be between 3V and 3.3V
  int adc = analogRead(CURRENT_READ_PIN);
  float voltRead = <static_cast>(float)(adc)/1023.0*(3.3); //converts read value (from 0 to 1023) to volts (0 to VCC).
  float ampsRead = voltRead/CURRENT_SENSOR_RATIO;
  return ampsRead;
}


CommandResult stopArm()
{
  joint1->runOutputControl(0);
  joint2->runOutputControl(0);
  joint3->runOutputControl(0);
  joint4->runOutputControl(0);
  joint5->runOutputControl(0);
}

CommandResult moveJ1(int16_t moveValue)
{
  joint1->runOutputControl(moveValue);
}

CommandResult moveJ2(int16_t moveValue)
{
  joint2->runOutputControl(moveValue);
}

CommandResult moveJ3(int16_t moveValue)
{
  joint3->runOutputControl(moveValue);
}

CommandResult moveJ4(int16_t moveValue)
{
  joint4->runOutputControl(moveValue);
}

CommandResult moveJ5(int16_t moveValue)
{
  joint5->runOutputControl(moveValue);
}
