#include "main.h"

Ma3Encoder12b joint1Encoder(ReadModule4, ENCODER1_READING_PIN);
Ma3Encoder12b joint2Encoder(ReadModule1, ENCODER2_READING_PIN);
Ma3Encoder12b joint3Encoder(ReadModule5, ENCODER3_READING_PIN);
Ma3Encoder12b joint4Encoder(ReadModule3, ENCODER4_READING_PIN);
Ma3Encoder12b joint5Encoder(ReadModule2, ENCODER5_READING_PIN);

VelocityDeriver joint1Vel(&joint1Encoder, .9);
VelocityDeriver joint2Vel(&joint2Encoder, .9);
VelocityDeriver joint3Vel(&joint3Encoder, .1);
VelocityDeriver joint5Vel(&joint5Encoder, .9);

PIAlgorithm joint1Alg(BaseRotateKp,BaseRotateKi,PI_TIMESLICE_SECONDS, &joint1Encoder);
PIAlgorithm joint2Alg(BaseTiltKp,BaseTiltKi,PI_TIMESLICE_SECONDS, &joint2Encoder);
PIAlgorithm joint3Alg(ElbowKp,ElbowKi,PI_TIMESLICE_SECONDS, &joint3Encoder);
PIAlgorithm joint5Alg(WristTiltKp,WristTiltKi,PI_TIMESLICE_SECONDS, &joint5Encoder);

PIVConverter joint1PIV(BaseRotateKp, BaseRotateKi, BaseRotateKp, BaseRotateKi, PIV_TIMESLICE_SECONDS, &joint1Encoder, &joint1Vel);
PIVConverter joint2PIV(BaseTiltKp, BaseTiltKi, BaseTiltKp, BaseTiltKi, PIV_TIMESLICE_SECONDS, &joint2Encoder, &joint2Vel);
PIVConverter joint3PIV(1, 0, 1, 0, PIV_TIMESLICE_SECONDS, &joint3Encoder, &joint3Vel);
PIVConverter joint5PIV(WristTiltKp, WristTiltKi, WristTiltKp, WristTiltKi, PIV_TIMESLICE_SECONDS, &joint5Encoder, &joint5Vel);

GravityInertiaSystemStatus sysStatus(gryphonArm, WristWeight, WristLength, WristCoG, ElbowWeight, ElbowLength, ElbowCoG, BaseWeight, BaseLength,
    BaseCoG, &joint1Encoder, &joint2Encoder, &joint3Encoder, &joint4Encoder, &joint5Encoder, &joint5Encoder);

GravityCompensator j1Grav(&sysStatus, TorqueConvert_BrushedDC, J12Kt, J1Resistance, MotorVoltage, 1);
GravityCompensator j2Grav(&sysStatus, TorqueConvert_BrushedDC, J12Kt, J2Resistance, MotorVoltage, 2);
GravityCompensator j3Grav(&sysStatus, TorqueConvert_BrushedDC, J3Kt, J3Resistance, MotorVoltage, 3);
GravityCompensator j4Grav(&sysStatus, TorqueConvert_BrushedDC, J45Kt, J4Resistance, MotorVoltage, 4);
GravityCompensator j5Grav(&sysStatus, TorqueConvert_BrushedDC, J45Kt, J5Resistance, MotorVoltage, 5);

GenPwmPhaseHBridge dev1(PwmGenerator2, MOT1_PWN_PIN, HBRIDGE1_PHASE_PIN, HBRIDGE1_NSLEEP_PIN, true, false);
GenPwmPhaseHBridge dev2(PwmGenerator1, MOT2_PWN_PIN, HBRIDGE2_PHASE_PIN, HBRIDGE2_NSLEEP_PIN, true, true);
GenPwmPhaseHBridge dev3(PwmGenerator3, MOT3_PWN_PIN, HBRIDGE3_PHASE_PIN, HBRIDGE3_NSLEEP_PIN, true, false);
GenPwmPhaseHBridge dev4(PwmGenerator3, MOT4_PWN_PIN, HBRIDGE4_PHASE_PIN, HBRIDGE4_NSLEEP_PIN, true, true);
GenPwmPhaseHBridge dev5(PwmGenerator2, MOT5_PWN_PIN, HBRIDGE5_PHASE_PIN, HBRIDGE5_NSLEEP_PIN, true, false);
GenPwmPhaseHBridge gripMotorDev(PwmGenerator1, GRIPMOT_PWM_PIN, GRIPMOT_PHASE_PIN, GRIPMOT_NENABLE_PIN, false, false);
RCContinuousServo gripServoDev(PwmGenerator0, GRIPPER_SERVO_PWM_PIN, false);

SingleMotorJoint gripperMotor(InputPowerPercent, &gripMotorDev);
SingleMotorJoint gripperServo(InputPowerPercent, &gripServoDev);

DifferentialJoint joint1(DifferentialRotate, InputPowerPercent, &dev1, &dev2); //joints initialized to open loop state
DifferentialJoint joint2(DifferentialTilt, InputPowerPercent, &dev1, &dev2);
SingleMotorJoint  joint3(InputPowerPercent, &dev3);
DifferentialJoint joint4(DifferentialRotate, InputPowerPercent, &dev4, &dev5);
DifferentialJoint joint5(DifferentialTilt, InputPowerPercent, &dev4, &dev5);

//variables used to control joints during closed loop control
unsigned long joint1Destination;
unsigned long joint2Destination;
unsigned long joint3Destination;
unsigned long joint5Destination;

ControlSystems currentControlSystem; //tracks what control system arm is currently using
bool mainPowerOn;
bool m1On;
bool m2On;
bool m3On;
bool m4On;
bool m5On;
bool gripMotOn;
bool initialized = false;  //tracks if program setup is finished. Needed as some closed loop interrupts will fail if parts of their code is run before initialize is finished, so this flag
                           //prevents fragile hardware calls from firing before then
bool limitsEnabled = true; //tracks if hardware limit switches are being used or if they're being overridden
bool watchdogUsed = false;

roveTimer_Handle timer7Handle;
roveTimer_Handle timer6Handle;

void init()
{
  roveComm_Begin(192, 168, 1, 131);
  joint1.pairDifferentialJoint(&joint2);
  joint4.pairDifferentialJoint(&joint5);

  //Initialize to open loop control format
  switchToOpenLoop();

  masterPowerSet(true);

  allMotorsPowerSet(true);

  //set timer 7 to fire at a rate where the different PI algorithms will all be updated at their expected timeslice in seconds.
  //There are 5 controls to update independently. They update one at a time, one being serviced every time the timer fires. So it takes 5 timer
  //firings for any individual control to get updated again. Meaning the timeslice of the timer itself must be one fifth of the PI algorithms overall timeslice so that
  //when it cycles back around the overall timeslice will have passed
  //Update: 4 controls are now used, but the setup still works just fine by firing off 5 times per overall timeslice and
  //changing it would require modifying the interrupt as well, so it's staying the way it is
  timer7Handle = setupTimer(Timer7, TimerPeriodicInterrupt, (PI_TIMESLICE_SECONDS/5.0) * 1000000.0);
  timer6Handle = setupTimer(Timer6, TimerPeriodicInterrupt, (PI_TIMESLICE_SECONDS) * 1000000);
  attachTimerInterrupt(timer6Handle, &sysStatusUpdater);
  attachTimerInterrupt(timer7Handle, &closedLoopUpdateHandler);

  joint1Alg.setDeadband(BaseRotateDeadband);
  //joint1Alg.setHardStopPositions(BaseRotateHardStopUp, BaseRotateHardStopDown);
  joint2Alg.setDeadband(BaseTiltDeadband);
  //joint2Alg.setHardStopPositions(BaseTiltHardStopUp, BaseTiltHardStopDown);
  joint3Alg.setDeadband(ElbowDeadband);
  //joint3Alg.setHardStopPositions(ElbowHardStopUp, ElbowHardStopDown);
  joint5Alg.setDeadband(WristTiltDeadband);
  //joint5Alg.setHardStopPositions(WristTiltHardStopUp, WristTiltHardStopDown);

  joint1Encoder.setOffsetAngle(BaseRotateOffsetAngle);
  joint2Encoder.setOffsetAngle(BaseTiltOffsetAngle);
  joint3Encoder.setOffsetAngle(ElbowOffsetAngle);
  joint4Encoder.setOffsetAngle(WristRotateOffsetAngle);
  joint5Encoder.setOffsetAngle(WristTiltOffsetAngle);

  joint1Alg.addSupportingAlgorithm(&j1Grav);
  joint2Alg.addSupportingAlgorithm(&j2Grav);
  joint3Alg.addSupportingAlgorithm(&j3Grav);
  //joint5Alg.addSupportingAlgorithm(&j5Grav);

  dev3.setRampUp(ElbowRampUp);
  dev3.setRampDown(ElbowRampDown);
  dev1.setRampUp(BaseRampUp - 20);
  dev2.setRampUp(BaseRampUp);
  dev1.setRampDown(BaseRampDown - 20);
  dev2.setRampDown(BaseRampDown);
  //dev4.setRampUp(WristRampUp);
  //dev4.setRampDown(WristRampDown);
  //dev5.setRampUp(WristRampUp);
  //dev5.setRampDown(WristRampDown);

  delay(2000); //let background processes finish before turning on the watchdog. Experimentation found that 2 seconds worked while values such as 1.5 resulted in program failure

  //initWatchdog(WATCHDOG_TIMEOUT_US);
  initialized = true;
}

int main()
{
  init();
  uint16_t commandId = 0;
  size_t commandSize = 0;
  char commandData[20];
  CommandResult result;

  while(1)
  {
    roveComm_GetMsg(&commandId, &commandSize, commandData);
    if(commandId != 0) //returns commandId == 0 if it didn't get any message
    {
      restartWatchdog(WATCHDOG_TIMEOUT_US); //reset watchdog timer since we received a command

      switch(commandId)
      {
        case ArmStop:
          result = stopArm();
          break;

        case ArmJ1:
        case LY_ArmJ1:
          if(currentControlSystem != OpenLoop)
          {
            switchToOpenLoop();
          }
          result = moveJ1(*(int16_t*)(commandData));
          break;

        case ArmJ2:
        case LY_ArmJ2:
          if(currentControlSystem != OpenLoop)
          {
            switchToOpenLoop();
          }
          result = moveJ2(*(int16_t*)(commandData));
          break;

       case ArmJ3:
       case LY_ArmJ3:
          if(currentControlSystem != OpenLoop)
          {
            switchToOpenLoop();
          }
          result = moveJ3(*(int16_t*)(commandData));
          break;

        case ArmJ4:
        case LY_ArmJ4:
          //if(currentControlSystem != OpenLoop)
          //{
           // switchToOpenLoop(); always considered open loop
          //}
          result = moveJ4(*(int16_t*)(commandData));
          break;

        case ArmJ5:
        case LY_ArmJ5:
          if(currentControlSystem != OpenLoop)
          {
            switchToOpenLoop();
          }
          result = moveJ5(*(int16_t*)(commandData));
          break;

        case MoveGripper: //gripper only ever operates in open loop but the rest of the system can be using other controls at the same time
          result = moveGripper(*(int16_t*)(commandData));
          break;

        case MoveGripServo: //gripper only ever operates in open loop but the rest of the system can be using other controls at the same time
          result = moveGripServo(*(int16_t*)(commandData));
          break;

        case ArmEnableAll:
          masterPowerSet((*(bool*)(commandData)));
          allMotorsPowerSet(*(bool*)(commandData));
          break;

        case ArmEnableMain:
          masterPowerSet(*(bool*)(commandData));
          break;

        case ArmAbsoluteAngle:
          if(currentControlSystem != ClosedLoop)
          {
            switchToClosedLoop();
          }
          setArmDestinationAngles(((float*)(commandData)));
          break;

        case ArmAbsoluteXYZ:
          if(currentControlSystem != ClosedLoop)
          {
            switchToClosedLoop();
          }
          float absoluteAngles[ArmJointCount];
          computeIK((float*)(commandData), absoluteAngles);
          setArmDestinationAngles(absoluteAngles);
          break;

        case ArmGetPosition:
          float currentPositions[6]; //six positions in the array because RED expects us to return 6 arguments even though we have 5 joints. 6th is just junk data
          getArmPositions(currentPositions);
          currentPositions[5] = 0;
          roveComm_SendMsg(ArmCurrentPosition, sizeof(float) * 6, currentPositions);
          break;

        case ArmEnableJ1:
          j12PowerSet(*(bool*)commandData); //joint 1 and joint 2 are linked together
          break;

        case ArmEnableJ2:
          j12PowerSet(*(bool*)commandData); //joint 1 and joint 2 are linked together
          break;

        case ArmEnableJ3:
          j3PowerSet(*(bool*)commandData); //joint 3 is on its own
          break;

        case ArmEnableJ4:
          j45PowerSet(*(bool*)commandData); //joint 4 and joint 5 are linked together
          break;

        case ArmEnableJ5:
          j45PowerSet(*(bool*)commandData); //joint 4 and joint 5 are linked together
          break;

        case ArmEnableEndeff:
          gripperMotorPowerSet(*(bool*)commandData);
          break;

        case ArmEnableServo:
          gripperServoPowerSet(*(bool*)commandData);
          break;

        case ArmCurrentMain:
          //float *armCurrent;
          //*armCurrent = readMasterCurrent();
          //roveComm_SendMsg(ArmCurrentMain, sizeof(float), armCurrent);
          break;

        case EnableLimits:
          limitsEnabled = true;
          break;

        case DisableLimits:
          limitsEnabled = false;
          break;

        default:
          break; //do nothing if it's not a known ID

      } //end switch

      if(result != Success)
      {
        //todo: if there's ever any telemetry about what to do when the command isn't successful, this is where we'll send telemetry back about it
      }
    }//end if(commandId != 0)
  }
}

//Turns on or off the main power line
CommandResult masterPowerSet(bool enable)
{
  mainPowerOn = enable;
  digitalPinWrite(POWER_LINE_CONTROL_PIN, enable);

  return Success;
}

//turns on or off all the motors
void allMotorsPowerSet(bool enable)
{
  j12PowerSet(enable);
  j3PowerSet(enable);
  j45PowerSet(enable);
  gripperMotorPowerSet(enable);
  gripperServoPowerSet(enable);
}


//stops all arm movement
CommandResult stopArm()
{
  //stop all closed loop movement by setting destination position to where the arm currently is
  joint1Destination = joint1Encoder.getFeedbackDegrees();
  joint2Destination = joint2Encoder.getFeedbackDegrees();
  joint3Destination = joint3Encoder.getFeedbackDegrees();
  joint5Destination = joint5Encoder.getFeedbackDegrees();

  joint1.stop();
  joint2.stop();
  joint3.stop();
  joint4.stop();
  joint5.stop();
  gripperMotor.stop();
  gripperServo.stop();
  return Success;
}

//turns on or off the motors attached to joint 1 and 2
void j12PowerSet(bool powerOn)
{
  if(powerOn)
  {
    joint1.enableJoint();
    joint2.enableJoint();
  }
  else
  {
    joint1.disableJoint();
    joint2.disableJoint();
  }

  m1On = powerOn;
  m2On = powerOn;
}

//turns on or off the motor attached to joint 3
void j3PowerSet(bool powerOn)
{
  if(powerOn)
  {
    joint3.enableJoint();
  }
  else
  {
    joint3.enableJoint();
  }

  m3On = powerOn;
}

//turns on or off the motors attached to joint 4 and 5
void j45PowerSet(bool powerOn)
{
  if(powerOn)
  {
    joint4.enableJoint();
    joint5.enableJoint();
  }
  else
  {
    joint4.disableJoint();
    joint5.disableJoint();
  }

  m4On = powerOn;
  m5On = powerOn;
}

//turns on or off the motor attached to the gripper
void gripperMotorPowerSet(bool powerOn)
{
  if(powerOn)
  {
    gripperMotor.enableJoint();
  }
  else
  {
    gripperMotor.disableJoint();
  }

  gripMotOn = powerOn;
}

//turns on or off the servo attached to the gripper
void gripperServoPowerSet(bool powerOn)
{
  if(powerOn)
  {
    gripperServo.enableJoint();
  }
  else
  {
    gripperServo.disableJoint();
  }
}

//Sets the angles for the joints of the arm to travel to
//Input: an angle array. angles[0] = joint1 destination, etc. Joints are described in floats from 0 to 360 degrees
//Note that this will only be acted on when the current control system being used is closed loop
CommandResult setArmDestinationAngles(float* angles)
{
  //angles comes in as an array
  joint1Destination = angles[0] * (((float)(POS_MAX - POS_MIN))/(360.0-0.0)); //convert from 0-360 float to framework's POSITION_MIN - POSITION_MAX long
  joint2Destination = angles[1] * (((float)(POS_MAX - POS_MIN))/(360.0-0.0));
  joint3Destination = angles[2] * (((float)(POS_MAX - POS_MIN))/(360.0-0.0));
  joint5Destination = angles[4] * (((float)(POS_MAX - POS_MIN))/(360.0-0.0));

  return Success;
}

//moves the first joint
//note that this function is used for open loop; use the setArmDestinationAngles function for closed loop movement
//note that the moveValue is numerically described using the joint control framework standard
CommandResult moveJ1(int16_t moveValue)
{
  if(moveValue > 0)
    moveValue = BaseMaxSpeed;
  else if(moveValue < 0)
    moveValue = -BaseMaxSpeed; //adjusting for base station

  joint1.runOutputControl(moveValue);

  return Success;
}

//moves the second joint
//note that this function is used for open loop; use the setArmDestinationAngles function for closed loop movement
//note that the moveValue is numerically described using the joint control framework standard.
//This being a tilt joint, limit switches are possibly used
CommandResult moveJ2(int16_t moveValue)
{
  //static bool limitSwitchHit = false;
  //static int moveAllowedDir = 0;

  if(moveValue > 0)
    moveValue = BaseMaxSpeed;
  else if(moveValue < 0)
    moveValue = -BaseMaxSpeed; //adjusting for base station scaling

  /*if(limitsEnabled)
  {
    if(checkLimSwitch(BASE_LIMIT_PIN) && !limitSwitchHit) //first time code detects switch being hit
    {
      float currentPos = joint2Encoder.getFeedbackDegrees();
      limitSwitchHit = true;
      if(350 < currentPos || currentPos < 90) //if we're at the lower end position, restrict movement to the positive direction
      {
        moveAllowedDir = 1;
      }
      else
      {
        moveAllowedDir = -1;
      }
     moveValue = 0;
    }
    else if(checkLimSwitch(BASE_LIMIT_PIN) && limitSwitchHit)
    {
      //if limit switch has already been hit, we need to move away from it. Restrict movement direction to away from the limit switch
      if(!(moveValue > 0 && moveAllowedDir >= 0) && !(moveValue < 0 && moveAllowedDir <= 0))
      {
        moveValue = 0;
      }
    }
    else if(!checkLimSwitch(BASE_LIMIT_PIN))
    {
      limitSwitchHit = false;
    }
  }*/

  joint2.runOutputControl(moveValue);

  return Success;
}

//moves the third joint
//note that this function is used for open loop; use the setArmDestinationAngles function for closed loop movement
//note that the moveValue is numerically described using the joint control framework standard
//This being a tilting joint, limit switches are possibly used
CommandResult moveJ3(int16_t moveValue)
{
  /*static bool limitSwitchHit = false;
  static int moveAllowedDir = 0;

  if(limitsEnabled)
  {
    if(checkLimSwitch(ELBOW_LIMIT_PIN) && !limitSwitchHit) //first time code detects switch being hit
    {
      float currentPos = joint3Encoder.getFeedbackDegrees();
      limitSwitchHit = true;
      if(350 < currentPos || currentPos < 100) //if we're at the lower end position, restrict movement to the positive direction
      {
        moveAllowedDir = 1;
      }
      else
      {
        moveAllowedDir = -1;
      }

     moveValue = 0;
    }
    else if(checkLimSwitch(ELBOW_LIMIT_PIN) && limitSwitchHit)
    {
      //if limit switch has already been hit, we need to move away from it. Restrict movement direction to away from the limit switch
      if(!(moveValue > 0 && moveAllowedDir >= 0) && !(moveValue < 0 && moveAllowedDir <= 0))
      {
        moveValue = 0;
      }
    }
    else if(!checkLimSwitch(ELBOW_LIMIT_PIN))
    {
      limitSwitchHit = false;
    }
  }*/
  joint3.runOutputControl(moveValue);

  return Success;
}

//moves the fourth joint
//note that this function is used for open loop; use the setArmDestinationAngles function for closed loop movement
//note that the moveValue is numerically described using the joint control framework standard
CommandResult moveJ4(int16_t moveValue)
{
  joint4.runOutputControl(moveValue);

  return Success;
}

//moves the fifth joint
//note that this function is used for open loop; use the setArmDestinationAngles function for closed loop movement
//note that the moveValue is numerically described using the joint control framework standard
//This being a tilt joint, limit switches are possibly used
CommandResult moveJ5(int16_t moveValue)
{
  static bool limitSwitchHit = false;
  static int moveAllowedDir = 0;

  if(limitsEnabled)
  {
    if(checkLimSwitch(WRIST_LIMIT_PIN) && !limitSwitchHit) //first time code detects switch being hit
    {
      float currentPos = joint5Encoder.getFeedbackDegrees();
      limitSwitchHit = true;
      if(350 < currentPos || currentPos < 100) //if we're at the lower end position, restrict movement to the positive direction
      {
        moveAllowedDir = 1;
      }
      else
      {
        moveAllowedDir = -1;
      }

     moveValue = 0;
    }
    else if(checkLimSwitch(WRIST_LIMIT_PIN) && limitSwitchHit)
    {
      //if limit switch has already been hit, we need to move away from it. Restrict movement direction to away from the limit switch
      if(!(moveValue > 0 && moveAllowedDir >= 0) && !(moveValue < 0 && moveAllowedDir <= 0))
      {
        moveValue = 0;
      }
    }
    else if(!checkLimSwitch(WRIST_LIMIT_PIN))
    {
      limitSwitchHit = false;
    }
  }

  joint5.runOutputControl(moveValue);

  return Success;
}

//moves the gripper open/closed
//note that this function is used for open loop; use the setArmDestinationAngles function for closed loop movement
//note that the moveValue is numerically described using the joint control framework standard
CommandResult moveGripper(int16_t moveValue)
{
  if(moveValue > 0)
    moveValue = 1000;
  else if(moveValue < 0)
    moveValue = -1000;

  gripperMotor.runOutputControl(moveValue);

  return Success;
}

//spins the gripper servo
//note that this function is used for open loop; use the setArmDestinationAngles function for closed loop movement
//note that the moveValue is numerically described using the joint control framework standard
CommandResult moveGripServo(int16_t moveValue)
{
  if(moveValue > 0)
    moveValue = 600; //largest value found that the servo moves at; starts not working beyond this
  else if(moveValue < 0)
    moveValue = -600;
  gripperServo.runOutputControl(moveValue);

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
    joint1.removeIOConverter(InputPowerPercent);
    joint2.removeIOConverter(InputPowerPercent);
    joint3.removeIOConverter(InputPowerPercent);
    joint5.removeIOConverter(InputPowerPercent);
  }

  currentControlSystem = OpenLoop;

  return Success;
}

//switches the arm over to closed loop control method; this will enable closed loop functions and functionality
//while disabling open loop functions and functionality
CommandResult switchToClosedLoop()
{
  currentControlSystem = ClosedLoop;

  //have default position destination values be the joints' current positions, so they hold still when switchover occurs until base station sends a new position to go towards
  joint1Destination = joint1Encoder.getFeedback();
  joint2Destination = joint2Encoder.getFeedback();
  joint3Destination = joint3Encoder.getFeedback();
  joint5Destination = joint5Encoder.getFeedback();

  if(initialized)
  {
    joint1.switchModules(InputPosition, &joint1Alg);
    joint2.switchModules(InputPosition, &joint2Alg);
    joint3.switchModules(InputPosition, &joint3Alg);
    joint5.switchModules(InputPosition, &joint5Alg);

    /*joint1.switchModules(InputPosition, &joint1PIV);
    joint2.switchModules(InputPosition, &joint2PIV);
    joint3.switchModules(InputPosition, &joint3PIV);
    joint5.switchModules(InputPosition, &joint5PIV);*/

    //enable closed loop interrupts, which will begin to move the arm towards its set destinations
    startTimer(timer7Handle);
    startTimer(timer6Handle);
  }

  return Success;
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
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_WDOG0))
  {
  }

  //
  // Initialize the watchdog timer.
  //
  WatchdogReloadSet(WATCHDOG0_BASE, load);

  //enable watchdog interrupts
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

//tells the watchdog to restart its count. Call this function periodically to keep the watchdog timer from firing during desired conditions
void restartWatchdog(uint32_t timeout_us)
{
  if(!watchdogUsed)
    return;
  if(WatchdogRunning(WATCHDOG0_BASE))
  {
    uint32_t load = getCpuClockFreq() * (timeout_us/1000000.0); // clock cycle (120 MHz cycle/second) * (microsecond timeout/10000000 to convert it to seconds) = cycles till the timeout passes
    load /=2; //watchdog resets after two timeouts

    WatchdogReloadSet(WATCHDOG0_BASE, load);
  }
}

//fills a float array with the current positions of the joints.
//Angles are numerically described as 0-360 degrees
CommandResult getArmPositions(float positions[ArmJointCount])
{
  positions[0] = joint1Encoder.getFeedbackDegrees();
  positions[1] = joint2Encoder.getFeedbackDegrees();
  positions[2] = joint3Encoder.getFeedbackDegrees();
  positions[3] = joint4Encoder.getFeedbackDegrees();
  positions[4] = joint5Encoder.getFeedbackDegrees();

  return Success;
}

//takes in a series of 4 coordinates (x,y,z, and gripper angle) and uses inverse kinematics to compute
//what angle each individual joint of the arm needs to be at for the gripper's endpoint to be at that coordinate (wrist rotate aside,
//it doesn't factor into the positional math).
//input: the 4 coordinates, describing x,y,z coordiantes in inches, and gripper angle in degrees
//output: 5 arm joint angles (for joints 1-5) in 0-360 degrees. Note that joint 4 is junk data, as it's not computed
void computeIK(float coordinates[IKArgCount], float angles[ArmJointCount])
{
 //This is the Position IK solution ripped straight from my Generalized IK matlab code with one modification to support J5(WRIST TILT).
  
  //THE ORIENTATION OF THE ARM WHEN ALL ANGLES ARE 0 IS STRAIGHT UP!!!!!!!!!!!
  
  //D-H Parameters of Arm Model
  //float th1offset=PI/2; //should be 90 in order for origin frame to comply with "Rover Coordinate Standard"
  float d1=2.7835; //height of bicep tilt axis from baseplate/origin
  float a1=0; //forward offset of bicep tilt axis relative to base rotate axis
  //float alpha1=PI/2; //anglular offset of J1 about X1 axis. (SHOULD BE 90 UNLESS ARM DESIGN IS SUPER FUNKY)
  //float th2offset=PI/2;//should be 90 in order to comply with DH convention
  float d2=0;//offset to the right of the bicep relative to the base rotation axis(should probably stay as 0 even if bicep is offset. this offset can also be accounted for using d3)
  float a2=16;//bicep length(distance between bicep tilt axis and elbow tilt axis)
  //float alpha2=0;//angular offset of elbow tilt axis about x2 axis.(SHOULD BE 90 UNLESS ARM DESIGN IS SUPER FUNKY)
  //float th3offset=PI/2;//should be 90
  float d3=.2075;//offset to the right of the forearm relative to the bicep(see d2 comment, if the bicep is offset from the base rotate axis but you have d2 as 0, then d3 must be the offset to the right of the forearm relative to the base rotate axis)
  float a3=2.4745;//offset of forearm twist axis from the elbow tilt axis along the x2 axis. (this is the "vertical" offset of the forearm.  DONT USE THIS if you calculated the actual distance between the elbow axis and wrist center and calculated the th3 offset accordingly. in that case a3 should be 0
  //float alpha3=PI/2;//angular offset of forearm about x3 axis. (SHOULD BE 90 UNLESS ARM DESIGN IS SUPER FUNKY)
  //float th4offset=0; //angular offset of forearm twist. should be 0 for standard spherical wrist orientation. (phoenix, horison, zenith, and gryphon's wrist joints all complied with this)
  float d4=14.0335;//Forearm Length. If a3 is zero but there is a "vertical" offset of the forearm, this value needs to be the center to center distance between the elbow tilt axis and the wrist center.
  //float EEoffset=5.25;//Distance of wrist center to center of gripper fingers
  float cX = coordinates[0];
  float cY = coordinates[1];
  float cZ = coordinates[2];
  float gripperAngle = coordinates[3];
  float joint1Angle;
  float joint2Angle;
  float joint3Angle;
  float joint5Angle;
  
  float loffset=sin(gripperAngle*(PI)/180); //these will be used to adjust the commanded position so that IK is solved for at the gripper
  float zoffset=-cos(gripperAngle*(PI)/180);
  //here comes the actual IK trigonometry! Woo Hoo!!
  float L=sqrt((cX*cX)+(cY*cY)-(d2+d3)*(d2+d3))-a1 + loffset; //this is the horizontal distance the bicep and forearm must reach
  joint1Angle=atan2(cY,cX) - atan2(L,((L/(L+a1)*(d2+d3))));//THIS ONLY SOLVES FOR FORWARD REACH (arm cant reach over head)
  float B=sqrt((a3*a3) + (d4*d4));//center to center distance from J3 to Wrist Center
  float R=sqrt((L*L)+((cZ-d1+zoffset)*(cZ-d1+zoffset)));//Reaching distance of biceb and forearm
  if(R>=(a2+B))//This checks to see if the desired point is within the working envelope of the arm
  {    //Dont update the joint angles for joints 2 or 3 if out of range
  }  
  else
  {
    float D=((R*R)-(a2*a2) -(B*B))/(2*a2*B); //acos of angle between bicep and B
    joint2Angle=(atan2(B*sqrt(1-(D*D)),(a2+B*D))+atan2((cZ-d1+zoffset),L)-(PI/2)); //Theta2
    joint3Angle=(atan2(-sqrt(1-(D*D)),D)+ atan2(d4,a3)-(PI/2));//Theta3
  }
  joint5Angle=gripperAngle-joint2Angle-joint3Angle;
  
  joint1Angle = negativeDegreeCorrection(joint1Angle);
  joint2Angle = negativeDegreeCorrection(joint2Angle);
  joint3Angle = negativeDegreeCorrection(joint3Angle);
  joint5Angle = negativeDegreeCorrection(joint5Angle);
  
  angles[0] = degrees(joint1Angle);
  angles[1] = degrees(joint2Angle);
  angles[2] = degrees(joint3Angle);
  angles[3] = 0;
  angles[4] = degrees(joint5Angle);
}

//converts 0 to -2pi, to 0 to 2pi
float negativeDegreeCorrection(float correctThis)
{
  while(correctThis < 0)
  {
    correctThis += 2.0*PI;
  }

  return(correctThis);
}

//Timer 7 periodic timeout interrupt.
//In this interrupt, closed loop protocol is serviced by updating the arm joint's destination positions.
//The interrupt doesn't decide the destination positions; that's done by other functions. Instead, it just tells the joints
//to go towards their predetermined positions. This is done because closed loop uses PI logic controls, and PI logic needs to be updated
//on a consistent timeslice for its algorithm to calculate properly.
void closedLoopUpdateHandler()
{
  static int jointUpdated = 0;
  restartWatchdog(WATCHDOG_TIMEOUT_US);

  jointUpdated += 1;
  if(jointUpdated > 5)
  {
    jointUpdated = 1;
  }
  if(jointUpdated == 1)
  {
    joint1.runOutputControl(joint1Destination);
  }
  else if(jointUpdated == 2)
  {
    joint2.runOutputControl(joint2Destination);
  }
  else if(jointUpdated == 3)
  {
    joint3.runOutputControl(joint3Destination);
  }
  else if(jointUpdated == 4)
  {
    //joint4.runOutputControl(joint4Destination); //only 4 joints used
  }
  else if(jointUpdated == 5)
  {
    //joint5.runOutputControl(joint5Destination);
  }
}

void sysStatusUpdater()
{
  sysStatus.update();
}
