/*
 * main.cpp
 *
 *  Created on: Sep 3, 2017
 *      Author: drue (drscp5@mst.edu), RMC by Drue, David, Timur, Eli, Chris Dutcher. Kinematics by Chris Novatny.
 *
 *  Project: Arm Board Software
 *
 *  Libraries used: Roveware (RoveComm and RMC), Roveboard for the tiva.
 *
 *  Description: This is the program used for all processing, controls and telemetry regarding the robotic arm.
 *  Meant to be ran off of one tiva and used on 2018's arm board.
 *  The program is split into multiple files; main.cpp is the front end, where all general operational logic and communications
 *  occur. RMCInstances h and cpp contain the global instances of the RMC objects used to execute the arm logic (RMC
 *  is the source for the motion controls itself, main.cpp just tells RMC what to do. In this way main.cpp is the front end
 *  that deals with operation while RMC is the backend that deals with execution). Kinematics contains all the math
 *  and functions needed for performing IK operations. ArmModelInfo.h finally just contains any info related to the arm
 *  itself physically.
 *
 *  Everything is fairly standard to the rover template after that; a main switch case for processing commands,
 *  a loop to check for faults in the arm, a loop for reading current sensors, and a loop for sending telemetry. The main
 *  things of note are that the arm can switch between three different control states. Open loop is as described above, but
 *  Closed loop and IKIncrement loop both start a thread on timer 6 and 7 to periodically update the closed loop control
 *  system independent of the main loop.
 *
 *  Hardware used; Timers 6 and 7 are used for threading, a pwm module is used to control the motors, several adc channels
 *  are used for reading currents. An internal watchdog is used that restarts the entire program after about a second
 *  without receiving any base station commands, in open loop mode at least. For a full list of all the pins used,
 *  see main.h
 */


#include "main.h"
#include <stdio.h>

static float baseRotateCurrent = 0;
static float baseTiltCurrent = 0;
static float elbowTiltCurrent = 0;
static float elbowRotateCurrent = 0;
static float wristTiltCurrent = 0;
static float wristRotateCurrent = 0;
static float masterCurrent = 0;

bool initialized = false;  //tracks if program setup is finished. Needed as some closed loop interrupts will fail if parts of their code is run before initialize is finished, so this flag
                           //prevents fragile hardware calls from firing before then
bool watchdogUsed = true;
bool gripperSwapped = false; //tracks if the gripper is inverted or not
bool sendPeriodicPositions = false;

RoveTimer_Handle timer7Handle;
RoveTimer_Handle timer6Handle;
RoveAdc_Handle baseRotateCurrentRead;
RoveAdc_Handle baseTiltCurrentRead;
RoveAdc_Handle elbowTiltCurrentRead;
RoveAdc_Handle elbowRotateCurrentRead;
RoveAdc_Handle wristTiltCurrentRead;
RoveAdc_Handle wristRotateCurrentRead;
RoveAdc_Handle masterCurrentRead;

void init()
{
  roveComm_Begin(192, 168, 1, 131);
  wristRotateJoint.pairDifferentialAxis(&wristTiltJoint);

  //Initialize to open loop control format
  switchToOpenLoop();

  //set timer 7 to fire at a rate where the different PI algorithms will all be updated at their expected timeslice in seconds.
  //There are 6 controls to update independently. They update one at a time, one being serviced every time the timer fires. So it takes 6 timer
  //firings for any individual control to get updated again. Meaning the timeslice of the timer itself must be one sixth of the PI algorithms overall timeslice so that
  //when it cycles back around the overall timeslice will have passed.
  //Timer 6 meanwhile will handle updating all the system's more complex math in parallel.
  timer7Handle = setupTimer(Timer7, TimerPeriodicInterrupt, (PI_TIMESLICE_SECONDS/ArmJointCount) * 1000000.0, &closedLoopUpdateHandler);
  timer6Handle = setupTimer(Timer6, TimerPeriodicInterrupt, (PI_TIMESLICE_SECONDS) * 1000000, &sysStatusUpdater);

  //janky ass solution to changing out pwm reads work in the encoders.
  //todo: just add in constructor to ma3Encoder to allow you to pass in a
  //pwmReadHandle instance that's already modified for it so we don't have to
  //modify it from the outside
  RovePwmRead_Handle handle = initPwmRead(0, ENCODER1_READING_PIN);
  setDisconnectCheckTimeout(handle, 20000);
  /*handle = initPwmRead(1, ENCODER2_READING_PIN);
  setDisconnectCheckTimeout(handle, 20000);
  handle = initPwmRead(2, ENCODER3_READING_PIN);
  setDisconnectCheckTimeout(handle, 20000);
  handle = initPwmRead(3, ENCODER4_READING_PIN);
  setDisconnectCheckTimeout(handle, 20000);
  handle = initPwmRead(4, ENCODER5_READING_PIN);
  setDisconnectCheckTimeout(handle, 20000);
  handle = initPwmRead(5, ENCODER6_READING_PIN);
  setDisconnectCheckTimeout(handle, 20000);*/

  baseRotateCurrentRead  = roveAdc_init(Adc0_Seq0_0, MOTOR1_CURRENT_INPUT);
  baseTiltCurrentRead    = roveAdc_init(Adc0_Seq0_1, MOTOR2_CURRENT_INPUT);
  elbowTiltCurrentRead   = roveAdc_init(Adc0_Seq0_2, MOTOR3_CURRENT_INPUT);
  elbowRotateCurrentRead = roveAdc_init(Adc0_Seq0_3, MOTOR4_CURRENT_INPUT);
  wristTiltCurrentRead   = roveAdc_init(Adc0_Seq0_4, MOTOR5_CURRENT_INPUT);
  wristRotateCurrentRead = roveAdc_init(Adc0_Seq0_5, MOTOR6_CURRENT_INPUT);
  masterCurrentRead      = roveAdc_init(Adc0_Seq0_6, MASTER_CURRENT_INPUT);

  baseRotateJointAlg.setDeadband(BaseRotateDeadband);
  baseTiltJointAlg.setHardStopPositions(BaseTiltHardStopDown, BaseTiltHardStopUp);
  baseTiltJointAlg.setDeadband(BaseTiltDeadband);
  elbowTiltJointAlg.setDeadband(ElbowTiltDeadband);
  elbowTiltJointAlg.setHardStopPositions(ElbowTiltHardStopUp, ElbowTiltHardStopDown);
  elbowRotateJointAlg.setDeadband(ElbowRotateDeadband);
  wristTiltJointAlg.setDeadband(WristTiltDeadband);
  wristRotateJointAlg.setDeadband(WristRotateDeadband);

  baseRotateJointEncoder.setOffsetAngle(BaseRotateOffsetAngle);
  baseTiltJointEncoder.setOffsetAngle(BaseTiltOffsetAngle);
  elbowTiltJointEncoder.setOffsetAngle(ElbowTiltOffsetAngle);
  elbowRotateJointEncoder.setOffsetAngle(ElbowRotateOffsetAngle);
  wristTiltJointEncoder.setOffsetAngle(WristTiltOffsetAngle);
  wristRotateJointEncoder.setOffsetAngle(WristRotateOffsetAngle);

  baseRotateJointEncoder.setMaxPwm(4190); //values just found experimentally for the encoders.
  baseTiltJointEncoder.setMaxPwm(4020);
  elbowTiltJointEncoder.setMaxPwm(4030);
  elbowRotateJointEncoder.setMaxPwm(4040);
  wristTiltJointEncoder.setMaxPwm(4285);
  wristRotateJointEncoder.setMaxPwm(4045);


  baseRotateJointEncoder.setFilterConstant(.2); //value that just worked.
  baseTiltJointEncoder.setFilterConstant(.2);
  elbowTiltJointEncoder.setFilterConstant(.2);
  wristTiltJointEncoder.setFilterConstant(.2);
  wristRotateJointEncoder.setFilterConstant(.2);
  baseRotateJointEncoder.reverseDirection(true);
  elbowRotateJointEncoder.reverseDirection(true);
  //j3Grav.setScalar(ScalarJ3);
  //baseRotateJointAlg.addSupportingAlgorithm(&j1Grav);
  //baseTiltJointAlg.addSupportingAlgorithm(&j2Grav);
  //elbowTiltJointAlg.addSupportingAlgorithm(&j3Grav);
  //wristTiltJointAlg.addSupportingAlgorithm(&j5Grav);

  baseRotateJoint.useStopcap(&baseRotateSwitch);
  baseTiltJoint.useStopcap(&baseTiltSwitches);
  elbowTiltJoint.useStopcap(&elbowTiltSwitches);

  baseRotateJoint.stop();
  baseTiltJoint.stop();
  elbowTiltJoint.stop();
  elbowRotateJoint.stop();
  wristTiltJoint.stop();
  wristRotateJoint.stop();
  gripper.stop();

  delay(1000);

  //let background processes finish before turning on the watchdog. Experimentation found that 2 seconds worked while values such as 1.5 resulted in program failure
  //also take some initial arm readings so that the sensors will converge onto their initial positions through their filters
  float currentPositions[6];
  for(int i = 0; i < (1000/4); i++)
  {
    getArmPositions(currentPositions);
    delay(4);
  }

  if(watchdogUsed)
  {
    initWatchdog(WATCHDOG_TIMEOUT_US);
  }

  masterPowerSet(true);

  allMotorsPowerSet(true);

  initialized = true;
}

int main()
{
  init();

  while(1)
  {
    processBaseStationCommands();
    readArmCurrents();
    processCurrentFaults();
    sendPeriodicTelemetry();
  }
}

void sendPeriodicTelemetry()
{
  static uint64_t mills = millis();

  if(mills > millis()) //clock cycled over
  {
    mills = millis();
  }

  if(millis() - mills > 750)
  {
    roveComm_SendMsg(ArmCurrentMain, sizeof(float), (void*)(&masterCurrent));

    if(sendPeriodicPositions)
    {
      sendArmPositions();
    }

    mills = millis();
  }
}

void processBaseStationCommands()
{
  uint16_t commandId = 0;
  size_t commandSize = 0;
  char commandData[100];
  CommandResult result;

  roveComm_GetMsg(&commandId, &commandSize, commandData);
  if(commandId != 0) //returns commandId == 0 if it didn't get any message
  {
    restartWatchdog(WATCHDOG_TIMEOUT_US); //reset watchdog timer since we received a command
    switch(commandId)
    {
      case ArmStop:
      {
        result = stopArm();
        break;
      }

      case ArmValues:
      {
        int16_t* values = ((int16_t*)(commandData));
        moveBaseRotate(values[0]);
        moveBaseTilt(values[1]);
        moveElbowTilt(values[2]);
        moveElbowRotate(values[3]);
        moveWristTilt(values[4]);
        moveWristRotate(values[5]);
        moveGripper(values[6]);
        movePoker(values[7]);
        break;
      }

      case GripperSwap:
      {
        gripperSwap();
        break;
      }

      case IKRoverIncrement:
      {
        int16_t *data = (int16_t*)(commandData);
        incrementRoverIK(data);
        moveGripper(data[6]);
        movePoker(data[7]);
        break;
      }

      case IKWristIncrement:
      {
        incrementWristIK((int16_t*)(commandData));
        break;
      }

      case ArmEnableAll:
      {
        masterPowerSet((*(bool*)(commandData)));
        allMotorsPowerSet(*(bool*)(commandData));
        break;
      }

      case ArmEnableMain:
      {
        masterPowerSet(*(bool*)(commandData));
        break;
      }

      case ArmAbsoluteAngle:
      {
        setArmDestinationAngles((float*)(commandData));
        break;
      }

      case ArmAbsoluteXYZ:
      {
        float absoluteAngles[ArmJointCount];
        calc_roverIK((float*)(commandData), absoluteAngles);
        setArmDestinationAngles(absoluteAngles);
        break;
      }

      case ArmGetPosition:
      {
        sendArmPositions();
        break;
      }

      case ArmGetXYZ:
      {
        float currentXYZ[IKArgCount];
        calcPresentCoordinates(currentXYZ);
        roveComm_SendMsg(ArmCurrentXYZ, sizeof(float) * 6, currentXYZ);
      }

      case ArmEnableJ1:
      {
        baseRotatePowerSet(*(bool*)commandData);
        break;
      }

      case ArmEnableJ2:
      {
        baseTiltPowerSet(*(bool*)commandData);
        break;
      }

      case ArmEnableJ3:
      {
        elbowTiltPowerSet(*(bool*)commandData);
        break;
      }

      case ArmEnableJ4:
      {
        elbowRotatePowerSet(*(bool*)commandData);
        break;
      }

      case ArmEnableJ5:
      {
        wristPowerSet(*(bool*)commandData);
        break;
      }

      case ArmEnableJ6:
      {
        wristPowerSet(*(bool*)commandData);
        break;
      }

      case ArmEnableEndeff1:
      {
        pokerPowerSet(*(bool*)commandData);
        break;
      }

      case ArmEnableEndeff2:
      {
        gripperPowerSet(*(bool*)commandData);
        break;
      }

      case ArmCurrentMain:
      {
        roveComm_SendMsg(ArmCurrentMain, sizeof(float), (void*)(&masterCurrent));
        break;
      }

      case LimitSwitchUnoveride:
      {
        handleLimits(*(uint8_t*)(commandData), true);
        break;
      }

      case LimitSwitchOveride:
      {
        handleLimits(*(uint8_t*)(commandData), false);
        break;
      }

      case OpPoint:
      {
        float *offsets = ((float*)(commandData));
        float offsetX = offsets[0];
        float offsetY = offsets[1];
        float offsetZ = offsets[2];

        setOpPointOffset(offsetX, offsetY, offsetZ);
        break;
      }

      case ToggleAutoPositionTelem:
      {
        sendPeriodicPositions = !sendPeriodicPositions;
      }

      default:
        break; //do nothing if it's not a known ID

    } //end switch

    if(result != Success)
    {
      //todo: if there's ever any telemetry about what to do when the command isn't successful, this is where we'll send telemetry back about it
    }
  }//end if(commandId != 0)
}

//Read the currents on all the power lines on the arm. This information is stored in global variables
void readArmCurrents()
{
  const float FilterConst = .9;
  uint32_t retVal;
  float newReading = 0;

  //due to the way the hardware works on the tiva, triggering a conversion on just one of these will actually trigger them all
  //since they share the same sequencer
  roveAdc_startConversion(masterCurrentRead);

  while(roveAdc_getConvResults(masterCurrentRead, &retVal) == ROVEADC_INCOMPLETE_CONVERSION);
  newReading = ((roveAdc_toVolts(retVal) - MasterSensorVoltOffset) / MasterSensorVoltPerAmp);
  if(newReading < 0) newReading = 0;
  masterCurrent = masterCurrent * FilterConst + (1.0 - FilterConst) * newReading;

  roveAdc_getConvResults(baseRotateCurrentRead, &retVal);
  newReading = ((roveAdc_toVolts(retVal) - MotorSensorVoltOffset) / MotorSensorVoltPerAmp);
  if(newReading < 0) newReading = 0;
  baseRotateCurrent = baseRotateCurrent * FilterConst + (1.0 - FilterConst) * newReading;

  roveAdc_getConvResults(baseTiltCurrentRead, &retVal);
  newReading = ((roveAdc_toVolts(retVal) - MotorSensorVoltOffset) / MotorSensorVoltPerAmp);
  if(newReading < 0) newReading = 0;
  baseTiltCurrent = baseTiltCurrent * FilterConst + (1.0 - FilterConst) * newReading;

  roveAdc_getConvResults(elbowTiltCurrentRead, &retVal);
  newReading = ((roveAdc_toVolts(retVal) - MotorSensorVoltOffset) / MotorSensorVoltPerAmp);
  if(newReading < 0) newReading = 0;
  elbowTiltCurrent = elbowTiltCurrent * FilterConst + (1.0 - FilterConst) * newReading;

  roveAdc_getConvResults(elbowRotateCurrentRead, &retVal);
  newReading = ((roveAdc_toVolts(retVal) - MotorSensorVoltOffset) / MotorSensorVoltPerAmp);
  if(newReading < 0) newReading = 0;
  elbowRotateCurrent = elbowRotateCurrent * FilterConst + (1.0 - FilterConst) * newReading;

  roveAdc_getConvResults(wristTiltCurrentRead, &retVal);
  newReading = ((roveAdc_toVolts(retVal) - MotorSensorVoltOffset) / MotorSensorVoltPerAmp);
  if(newReading < 0) newReading = 0;
  wristTiltCurrent = wristTiltCurrent * FilterConst + (1.0 - FilterConst) * newReading;

  roveAdc_getConvResults(wristRotateCurrentRead, &retVal);
  newReading = ((roveAdc_toVolts(retVal) - MotorSensorVoltOffset) / MotorSensorVoltPerAmp);
  if(newReading < 0) newReading = 0;
  wristRotateCurrent = wristRotateCurrent * FilterConst + (1.0 - FilterConst) * newReading;
}

//check to see if anything on the arm overcurrented. If it did, a) disable it b) tell base station about it
void processCurrentFaults()
{
  //rovecomm requires a pointer to a variable for its data values, so we can't just pass it the constants directly
  //as constants don't typically have memory addresses and could cause a silent crash
  uint8_t ArmFaultM1 = ArmFault_m1;
  uint8_t ArmFaultM2 = ArmFault_m2;
  uint8_t ArmFaultM3 = ArmFault_m3;
  uint8_t ArmFaultM4 = ArmFault_m4;
  uint8_t ArmFaultM5 = ArmFault_m5;
  uint8_t ArmFaultM6 = ArmFault_m6;
  uint8_t ArmFaultMaster = ArmFault_overcurrent;

  if(baseRotateCurrent > MotorMaxCurrent)
  {
    roveComm_SendMsg(ArmFault, sizeof(ArmFault_m1), (void*)&ArmFaultM1);
    baseRotatePowerSet(false);
  }
  if(baseTiltCurrent > MotorMaxCurrent)
  {
    roveComm_SendMsg(ArmFault, sizeof(ArmFault_m2), (void*)&ArmFaultM2);
    baseTiltPowerSet(false);
  }
  if(elbowTiltCurrent > MotorMaxCurrent)
  {
    roveComm_SendMsg(ArmFault, sizeof(ArmFault_m3), (void*)&ArmFaultM3);
    elbowTiltPowerSet(false);
  }
  if(elbowRotateCurrent > MotorMaxCurrent)
  {
    roveComm_SendMsg(ArmFault, sizeof(ArmFault_m4), (void*)&ArmFaultM4);
    elbowRotatePowerSet(false);
  }
  if(wristTiltCurrent > MotorMaxCurrent)
  {
    roveComm_SendMsg(ArmFault, sizeof(ArmFault_m5), (void*)&ArmFaultM5);
    wristPowerSet(false);
  }
  if(wristRotateCurrent > MotorMaxCurrent)
  {
    roveComm_SendMsg(ArmFault, sizeof(ArmFault_m6), (void*)&ArmFaultM6);
    wristPowerSet(false);
  }
  if(masterCurrent > MasterMaxCurrent)
  {
    roveComm_SendMsg(ArmFault, sizeof(ArmFault_overcurrent), (void*)&ArmFaultMaster);
    masterPowerSet(false);
  }
}

//Turns on or off the main power line
CommandResult masterPowerSet(bool enable)
{
  digitalPinWrite(POWER_LINE_CONTROL_PIN, enable);

  return Success;
}

//turns on or off all the motors
void allMotorsPowerSet(bool enable)
{
  baseRotatePowerSet(enable);
  elbowTiltPowerSet(enable);
  elbowTiltPowerSet(enable);
  elbowRotatePowerSet(enable);
  wristPowerSet(enable);
  pokerPowerSet(enable);
  gripperPowerSet(enable);
}


//stops all arm movement
CommandResult stopArm()
{
  //stop all closed loop movement by setting destination position to where the arm currently is
  baseRotateJointDestination = baseRotateJointEncoder.getFeedback();
  baseTiltJointDestination = baseTiltJointEncoder.getFeedback();
  elbowTiltJointDestination = elbowTiltJointEncoder.getFeedback();
  elbowRotateJointDestination = elbowRotateJointEncoder.getFeedback();
  wristTiltJointDestination = wristTiltJointEncoder.getFeedback();
  wristRotateJointDestination = wristRotateJointEncoder.getFeedback();

  baseRotateJoint.stop();
  baseTiltJoint.stop();
  elbowTiltJoint.stop();
  elbowRotateJoint.stop();
  wristTiltJoint.stop();
  wristRotateJoint.stop();
  gripper.stop();
  poker.stop();
  return Success;
}

//turns on or off joint 1
void baseRotatePowerSet(bool powerOn)
{
  if(powerOn)
  {
    baseRotateJoint.enableAxis();
  }
  else
  {
    baseRotateJoint.disableAxis();
  }
}

//turns on or off joint 2
void baseTiltPowerSet(bool powerOn)
{
  if(powerOn)
  {
    baseTiltJoint.enableAxis();
  }
  else
  {
    baseTiltJoint.disableAxis();
  }
}

//turns on or off joint 3
void elbowTiltPowerSet(bool powerOn)
{
  if(powerOn)
  {
    elbowTiltJoint.enableAxis();
  }
  else
  {
    elbowTiltJoint.disableAxis();
  }
}

//turns on or off joint 4
void elbowRotatePowerSet(bool powerOn)
{
  if(powerOn)
  {
    elbowRotateJoint.enableAxis();
  }
  else
  {
    elbowRotateJoint.disableAxis();
  }
}

//turns on or off joint 5 and 6
void wristPowerSet(bool powerOn)
{
  if(powerOn)
  {
    wristTiltJoint.enableAxis();
    wristRotateJoint.enableAxis();
  }
  else
  {
    wristTiltJoint.disableAxis();
    wristRotateJoint.disableAxis();
  }
}

//turns on or off the poker
void pokerPowerSet(bool powerOn)
{
  if(powerOn)
  {
    poker.enableAxis();
  }
  else
  {
    poker.disableAxis();
  }
}

//turns on or off the gripper
void gripperPowerSet(bool powerOn)
{
  if(powerOn)
  {
    gripper.enableAxis();
  }
  else
  {
    gripper.disableAxis();
  }
}

//moves the first joint
//note that using this function will automatically put the arm into open loop mode
//movevalue: Between 1000 and -1000
CommandResult moveBaseRotate(int16_t moveValue)
{
  if(currentControlSystem != OpenLoop)
  {
    switchToOpenLoop();
  }

  baseRotateJoint.runOutputControl(moveValue);
  return Success;
}

//moves the second joint
//note that using this function will automatically put the arm into open loop mode
//movevalue: Between 1000 and -1000
CommandResult moveBaseTilt(int16_t moveValue)
{
  if(currentControlSystem != OpenLoop)
  {
    switchToOpenLoop();
  }

  baseTiltJoint.runOutputControl(moveValue);
  return Success;
}

//moves the third joint
//note that using this function will automatically put the arm into open loop mode
//movevalue: Between 1000 and -1000
CommandResult moveElbowTilt(int16_t moveValue)
{
  if(currentControlSystem != OpenLoop)
  {
    switchToOpenLoop();
  }

  elbowTiltJoint.runOutputControl(moveValue);
  return Success;
}

//moves the fourth joint
//note that using this function will automatically put the arm into open loop mode
//movevalue: Between 1000 and -1000
CommandResult moveElbowRotate(int16_t moveValue)
{
  if(currentControlSystem != OpenLoop)
  {
    switchToOpenLoop();
  }

  elbowRotateJoint.runOutputControl(moveValue);
  return Success;
}

//moves the fifth joint
//note that using this function will automatically put the arm into open loop mode
//movevalue: Between 1000 and -1000
CommandResult moveWristTilt(int16_t moveValue)
{
  if(currentControlSystem != OpenLoop)
  {
    switchToOpenLoop();
  }

  wristTiltJoint.runOutputControl(moveValue);
  return Success;
}

//moves the sixth joint
//note that using this function will automatically put the arm into open loop mode
//movevalue: Between 1000 and -1000
CommandResult moveWristRotate(int16_t moveValue)
{
  if(currentControlSystem != OpenLoop)
  {
    switchToOpenLoop();
  }

  wristRotateJoint.runOutputControl(moveValue);
  return Success;
}

//moves the gripper.
//movevalue: between 1000 and -1000
CommandResult moveGripper(int16_t moveValue)
{
  if(moveValue > 0)
    moveValue = MaxGripperPower;
  else if(moveValue < 0)
    moveValue = -MaxGripperPower;

  gripper.runOutputControl(moveValue);
  return Success;
}

//moves the poker.
//movevalue: between 1000 and -1000
CommandResult movePoker(int16_t moveValue)
{
  if(moveValue > 0)
    moveValue = 1000;
  else if(moveValue < 0)
    moveValue = -1000;

  poker.runOutputControl(moveValue);
  return Success;
}

//checks to see if the limit switch on the passed pin has been activated.
//Returns true if it's pushed, false otherwise
bool checkLimSwitch(uint32_t switchPin)
{
  return(digitalPinRead(switchPin) == 0); //switch pins active low
}

//switches the arm over to open loop control method; this will disable closed loop functions and functionality
//while enabling open loop functions and functionality
CommandResult switchToOpenLoop()
{
  if(initialized)
  {
    //disable closed loop interrupts before doing any operation to preserve thread safety
    stopTimer(timer7Handle);
    stopTimer(timer6Handle);
    stopArm();
    baseRotateJoint.removeIOConverter(InputPowerPercent);
    baseTiltJoint.removeIOConverter(InputPowerPercent);
    elbowTiltJoint.removeIOConverter(InputPowerPercent);
    elbowRotateJoint.removeIOConverter(InputPowerPercent);
    wristTiltJoint.removeIOConverter(InputPowerPercent);
    wristRotateJoint.removeIOConverter(InputPowerPercent);
  }

  currentControlSystem = OpenLoop;

  return Success;
}

CommandResult switchToIKIncrement()
{
  if(initialized)
  {
    if(currentControlSystem != ClosedLoop)
    {
      switchToClosedLoop();
    }

    delay(100);
    if(currentControlSystem != ClosedLoop)
    {
      return; //means a closed loop error was detected in its interrupt, don't go ahead with it
    }

    initPresentCoordinates();

    currentControlSystem = IKIncrement;
  }

  return Success;
}

//switches the arm over to closed loop control method; this will enable closed loop functions and functionality
//while disabling open loop functions and functionality
CommandResult switchToClosedLoop()
{
  currentControlSystem = ClosedLoop;

  //have default position destination values be the joints' current positions, so they hold still when switchover occurs
  //until base station sends a new position to go towards
  baseRotateJointDestination = baseRotateJointEncoder.getFeedback();
  baseTiltJointDestination = baseTiltJointEncoder.getFeedback();
  elbowTiltJointDestination = elbowTiltJointEncoder.getFeedback();
  elbowRotateJointDestination = elbowRotateJointEncoder.getFeedback();
  wristTiltJointDestination = wristTiltJointEncoder.getFeedback();
  wristRotateJointDestination = wristRotateJointEncoder.getFeedback();

  if(initialized)
  {
    stopArm();
    baseRotateJoint.switchModules(InputPosition, &baseRotateJointAlg);
    baseTiltJoint.switchModules(InputPosition, &baseTiltJointAlg);
    elbowTiltJoint.switchModules(InputPosition, &elbowTiltJointAlg);
    elbowRotateJoint.switchModules(InputPosition, &elbowRotateJointAlg);
    wristTiltJoint.switchModules(InputPosition, &wristTiltJointAlg);
    wristRotateJoint.switchModules(InputPosition, &wristRotateJointAlg);

    /*baseRotateJoint.switchModules(InputPosition, &baseRotateJointPIV);
    baseTiltJoint.switchModules(InputPosition, &baseTiltJointPIV);
    elbowTiltJoint.switchModules(InputPosition, &elbowTiltJointPIV);
    wristTiltJoint.switchModules(InputPosition, &wristTiltJointPIV);*/

    //enable closed loop interrupts, which will begin to move the arm towards its set destinations
    startTimer(timer7Handle);
    startTimer(timer6Handle);
  }

  return Success;
}

//Sets the angles for the joints of the arm to travel to
//Input: an angle array. angles[0] = baseRotateJoint destination, etc. Joints are described in floats from 0 to 360 degrees
//Note that this will cause the arm to enter closed loop mode unless it's in IK increment mode
CommandResult setArmDestinationAngles(float* angles)
{
  if(currentControlSystem != ClosedLoop && currentControlSystem != IKIncrement)
  {
    switchToClosedLoop();
  }

  float temp;

  //for some stupid reason the program crashes if we don't save it to a temp variable before storing it
  //into joint destinations
  temp = angles[0];
  baseRotateJointDestination = temp * DEGREES_TO_POS;
  temp = angles[1];
  baseTiltJointDestination = temp * DEGREES_TO_POS;
  temp = angles[2];
  elbowTiltJointDestination = temp * DEGREES_TO_POS;
  temp = angles[3];
  elbowRotateJointDestination = temp * DEGREES_TO_POS;
  temp = angles[4];
  wristTiltJointDestination = temp * DEGREES_TO_POS;
  temp = angles[5];
  wristRotateJointDestination = temp * DEGREES_TO_POS;

  return Success;
}

void sendArmPositions()
{
  float currentPositions[6];
  getArmPositions(currentPositions);
  roveComm_SendMsg(ArmCurrentPosition, sizeof(float) * 6, currentPositions);
}

//sets up the watchdog timer. Watchdog timer will restart the processor and the program when it times out
//input: timeout value in microseconds
void initWatchdog(uint32_t timeout_us)
{
  watchdogUsed = true;
  uint32_t load = getCpuClockFreq() * (timeout_us/1000000.0); // clock cycle (120 MHz cycle/second) * (microsecond timeout/10000000 to convert it to seconds) = cycles till the timeout passes
  load /=2; //watchdog resets after two timeouts

  //
  // Enable the Watchdog 0 peripheral
  //
  SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG0);

  //
  // Wait for the Watchdog 0 module to be ready.
  //
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_WDOG0));

  //
  // Initialize the watchdog timer.
  //
  WatchdogReloadSet(WATCHDOG0_BASE, load);

  //enable watchdog interrupts
  WatchdogIntRegister(WATCHDOG0_BASE, watchdogISR);
  WatchdogIntEnable(WATCHDOG0_BASE);
  IntEnable(INT_WATCHDOG);

  //
  // Enable the watchdog timer.
  //
  WatchdogEnable(WATCHDOG0_BASE);

  //
  // Enable the reset.
  //
  WatchdogResetEnable(WATCHDOG0_BASE);
}

//tells the watchdog to restart its count. Call this function periodically to keep the watchdog timer
//from firing during desired conditions
void restartWatchdog(uint32_t timeout_us)
{
  if(!watchdogUsed)
    return;
  if(WatchdogRunning(WATCHDOG0_BASE))
  {
    //clock cycle (120 MHz cycle/second) * (microsecond timeout/10000000 to convert it to seconds) = cycles till the timeout passes
    uint32_t load = getCpuClockFreq() * (timeout_us/1000000.0);
    load /=2; //watchdog resets after two timeouts

    WatchdogReloadSet(WATCHDOG0_BASE, load);
  }
}

void watchdogISR()
{
  masterPowerSet(false);
}

//fills a float array with the current positions of the joints.
//Angles are numerically described as 0-360 degrees
CommandResult getArmPositions(float positions[ArmJointCount])
{
  positions[0] = baseRotateJointEncoder.getFeedbackDegrees();
  positions[1] = baseTiltJointEncoder.getFeedbackDegrees();
  positions[2] = elbowTiltJointEncoder.getFeedbackDegrees();
  positions[3] = elbowRotateJointEncoder.getFeedbackDegrees();
  positions[4] = wristTiltJointEncoder.getFeedbackDegrees();
  positions[5] = wristRotateJointEncoder.getFeedbackDegrees();

  return Success;
}

//Timer 7 periodic timeout interrupt.
//In this interrupt, closed loop protocol is serviced by updating the arm joint's destination positions.
//The interrupt doesn't decide the destination positions; that's done by other functions. Instead, it just tells the joints
//to go towards their predetermined positions. This is done because closed loop uses PI logic controls, and PI logic needs to be updated
//on a consistent timeslice for its algorithm to calculate properly.
void closedLoopUpdateHandler()
{
  static int jointUpdated = 1;
  restartWatchdog(WATCHDOG_TIMEOUT_US);
  AxisControlStatus status = OutputRunning;
  int faultMessage;

  if(jointUpdated == 1)
  {
    status = baseRotateJoint.runOutputControl(baseRotateJointDestination);
    faultMessage = ArmFault_encoderBaseRotate;
  }
  else if(jointUpdated == 2)
  {
    status = baseTiltJoint.runOutputControl(baseTiltJointDestination);
    faultMessage = ArmFault_encoderBaseTilt;
  }
  else if(jointUpdated == 3)
  {
    //elbowTiltJoint.runOutputControl(elbowTiltJointDestination);
    status = elbowTiltJoint.runOutputControl(elbowTiltJointDestination);
    faultMessage = ArmFault_encoderElbowTilt;
  }
  else if(jointUpdated == 4)
  {
    status = elbowRotateJoint.runOutputControl(elbowRotateJointDestination);
    faultMessage = ArmFault_encoderElbowRotate;
  }
  else if(jointUpdated == 5)
  {
    status = wristTiltJoint.runOutputControl(wristTiltJointDestination);
    faultMessage = ArmFault_encoderWristTilt;
  }
  else if(jointUpdated == 6)
  {
    status = wristRotateJoint.runOutputControl(wristRotateJointDestination);
    faultMessage = ArmFault_encoderWristRotate;
  }

  jointUpdated += 1;
  if(jointUpdated > 6)
  {
    jointUpdated = 1;
  }

  if(status == FeedbackError)
  {
    switchToOpenLoop();
    roveComm_SendMsg(ArmFault, sizeof(faultMessage), (void*)&faultMessage);
  }
}

void gripperSwap()
{
  float newOffset;

  if(!gripperSwapped)
  {
    newOffset = WristTiltOffsetAngle + 180;
    if(abs(newOffset) > 360)
    {
      newOffset -= (360 * sign(newOffset));
    }

    gripperSwapped = true;
  }
  else
  {
    newOffset = WristTiltOffsetAngle;
    gripperSwapped = false;
  }

  wristTiltJointEncoder.setOffsetAngle(newOffset);
}

void sysStatusUpdater()
{
  //sysStatus.update();
}

void handleLimits(uint8_t jointNumber, bool enable)
{
  if(enable)
  {
    switch(jointNumber)
    {
      case 1:
        baseRotateJoint.useStopcap(&baseRotateSwitch);
        break;

      case 2:
        baseTiltJoint.useStopcap(&baseTiltSwitches);
        break;

      case 3:
        elbowTiltJoint.useStopcap(&elbowTiltSwitches);
        break;
    }
  }
  else
  {
    switch(jointNumber)
    {
      case 1:
        baseRotateJoint.removeStopcap();
        break;

      case 2:
        baseTiltJoint.removeStopcap();
        break;

      case 3:
        elbowTiltJoint.removeStopcap();
        break;
    }
  }
}
