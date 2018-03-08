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

  j3Grav.setScalar(ScalarJ3);
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


//BEGINNING OF NOVA IK
//Still neeed to include matrixmath and math.h somewhere. im not a compsci

//ANGLES ARE IN RADIANS!!!!!
//DISTANCES ARE IN INCHES!!!

//D-H Parameters of Arm Model
float th1offset=1.57079632679; //should be 90 in order for origin frame to comply with "Rover
// Coordinate Standard"
float d1=2.8937; // height of bicep tilt axis from baseplate/origin
float a1=0; //forward offset of bicep tilt axis relative to base rotate axis
float alpha1=1.57079632679; //anglular offset of J1 about X1 axis. (SHOULD BE 90 UNLESS ARM 
           // DESIGN IS SUPER FUNKY)
float th2offset=1.57079632679;//should be 90 in order to comply with DH convention 
float d2=0;//offset to the right of the bicep relative to the base rotation axis(
     //should probably stay as 0 even if bicep is offset. this offset can 
     //also be accounted for using d3)
float a2=17;//bicep length(distance between bicep tilt axis and elbow tilt axis)
float alpha2=0;//angular offset of elbow tilt axis about x2 axis.(SHOULD BE 90 
         //UNLESS ARM DESIGN IS SUPER FUNKY)
float th3offset=1.57079632679;//should be 90
float d3=0;//offset to the right of the forearm relative to the bicep(see d2 
     //comment, if the bicep is offset from the base rotate axis but you 
     //have d2 as 0, then d3 must be the offset to the right of the forearm 
     //relative to the base rotate axis)
float a3=2.837;//offset of forearm twist axis from the elbow tilt axis along the x2 
     //axis. (this is the "vertical" offset of the forearm.  DONT USE THIS 
     //if you calculated the actual distance between the elbow axis and 
     //wrist center and calculated the th3 offset accordingly. in that case 
     //a3 should be 0
float alpha3=1.57079632679;//angular offset of forearm about x3 axis. (SHOULD BE 90 UNLESS ARM 
         //DESIGN IS SUPER FUNKY)
float th4offset=0; //angular offset of forearm twist. should be 0 for standard 
             //spherical wrist orientation. (phoenix, horison, zenith, and 
             //gryphon's wrist joints all complied with this)
float d4=17;//Forearm Length. If a3 is zero but there is a "vertical" offset of 
      //the forearm, this value needs to be the center to center distance 
      //between the elbow tilt axis and the wrist center.
float a4=0; //needs to be 0 for spherical wrist
float alpha4=-1.57079632679; //should be -90 for standard spherical wrist orientation. 
            //(phoenix, horiZon, zenith, and gryphon's wrist joints all 
            //complied with this)
float th5offset=0; //wrist tilt angle offset. should be 0 unless there is a 
             //"vertical" forearm offset and you chose to use the center to 
             //center distances between the elbow tilt axis and the wrist 
             //center. if this is the case, th4offset needs to be calculated
             //as the angle between the line center line between the elbow 
             //tilt axis and wrist center with the axis of gripper rotate(j6)
float d5=0;//needs to be 0 for spherical wrist
float a5=0;//needs to be 0 for spherical wrist
float alpha5=1.57079632679;//angular offset of gripper rotate axis from gripper tilt axis 
          //about x5 axis. needs to be 90 for spherical wrist 
float th6offset=1.57079632679; //angular twist of gripper from normal orientation. should be 
              //90 for standard spherical wrist orientation. (phoenix, 
              //horiZon, zenith, and gryphon's wrist joints all complied with this)
float d6=0;//keep as 0
float a6=0;//keep as 0
float alpha6=1.57079632679; //angular tilt of gripper from normal orientation. should be 90 
           //for standard spherical wrist orientation. (phoenix, horiZon, 
           //zenith, and gryphon's wrist joints all complied with this)

//CENTER POINT OF GRIPPER
float OpPointoffset[3][1]={{0},  
                           {5.25},
                           {0}};

//important supporting functions

void DHTrans(float th, float d, float a, float alpha, float A1[4][4]){  //Calculate the Homogenous transform from the DH convention
   A1[0][0] = cos(th);
   A1[0][1] =  -sin(th)*cos(alpha);
   A1[0][2] = sin(th)*sin(alpha);
   A1[0][3] = a*cos(th);
   A1[1][0] =sin(th) ;
   A1[1][1] = cos(th)*cos(alpha);
   A1[1][2] = -cos(th)*sin(alpha);
   A1[1][3] = a*sin(th);
   A1[2][0] = 0;
   A1[2][1] = sin(alpha);
   A1[2][2] = cos(alpha);
   A1[2][3] = d;
   A1[3][0] = 0;
   A1[3][1] = 0;
   A1[3][2] = 0;
   A1[3][3] = 1;
}

float angledist(float theta1,float theta2){
   if (abs(theta1-theta2)>3.14159265359){
    float a= (2*3.14159265359)-abs(theta1-theta2);
    return a;
   }
   else{
    float a=abs(theta1-theta2);
    return a;
   }      
}

void Rotx(float t,float Rx[3][3]){   //Calculate a rotation matrix of a rotation in about the X axis by theta
  Rx[0][0]=1;
  Rx[0][1]=0;
  Rx[0][2]=0;
  Rx[1][0]=0;
  Rx[1][1]=cos(t);
  Rx[1][2]=-sin(t);
  Rx[2][0]=0;
  Rx[2][1]=sin(t);
  Rx[2][2]=cos(t);
  }

void Roty(float t,float Ry[3][3]){   //Calculate a rotation matrix of a rotation in about the Y axis by theta
  Ry[0][0]=cos(t);
  Ry[0][1]=0;
  Ry[0][2]=-sin(t);
  Ry[1][0]=0;
  Ry[1][1]=1;
  Ry[1][2]=0;
  Ry[2][0]=sin(t);
  Ry[2][1]=0;
  Ry[2][2]=cos(t);
  }

void Rotz(float t,float Rz[3][3]){   //Calculate a rotation matrix of a rotation in about the X axis by theta
  Rz[0][0]=cos(t);
  Rz[0][1]=-sin(t);
  Rz[0][2]=0;
  Rz[1][0]=sin(t);
  Rz[1][1]=cos(t);
  Rz[1][2]=0;
  Rz[2][0]=0;
  Rz[2][1]=0;
  Rz[2][2]=1;
  }

void Calc_IK(float coordinates[IKArgCount], float angles[ArmJointCount]){//float x,float y,float z,float yaw,float pitch,float roll,float *Joints){
  //HERE COMES THE IK MATH!!!
//operating point location
//x=0;// Desired X coordinate of gripper relative to the Rover (where the arm attaches)
//y=17+5.25;// Desired Y coordinate of gripper relative to the Rover (where the arm attaches)
//z=22.73;// Desired Z coordinate of gripper relative to the Rover (where the arm attaches)
float old1=radians(angles[0]);  //Current Joint angles
float old2=radians(angles[1]);
float old3=radians(angles[2]);
float old4=radians(angles[3]);
float old5=radians(angles[4]);
float old6=radians(angles[5]);

//operating point orientation 
//The Order of these rotations matters for the final orientation. I will 
//choose to Yaw first, Then Pitch, and then//Roll.  this can be changed of 
//course, this is just how i am going to do it
//yaw=0; //Rotation of gripper about Gripper's Z axis
//pitch=0; //Rotation of gripper about Gripper's X axis
//roll=0;//Rotation of gripper about Gripper's Y axis
//Calculate the final desired Rotation matris(Gripper Orientation)
float Rotzyaw[3][3];
float Rotxpitch[3][3];
float Rotyroll[3][3];

Rotz(coordinates[3],Rotzyaw);
Rotx(coordinates[4],Rotxpitch);
Roty(coordinates[5],Rotyroll);
float OpRot[3][3];
float OpRottemp[3][3];
Matrix.Multiply((float*)Rotzyaw, (float*)Rotxpitch, 3, 3, 3, (float*)OpRottemp);
Matrix.Multiply((float*)OpRottemp, (float*)Rotyroll, 3, 3, 3, (float*)OpRot);
//IMPLEMENTED MATH IS EQUIVALENT TO:     OpRot=Rotz(yaw)*Rotx(pitch)*Roty(roll)
//Can add other rotations here if so 
//desired. would need to introduce new variables though.


//Calculate the Wrist Center location from Gripper Location and Orientation
float OpPoint[3][1] = {{coordinates[0]},{coordinates[1]},{coordinates[2]}};
float OpPointtemp[3][1];
Matrix.Multiply((float*)OpRot, (float*)OpPointoffset, 3, 3, 1, (float*)OpPointtemp);
float WristCenter[3][1];
Matrix.Subtract((float*)OpPoint, (float*)OpPointtemp, 3, 1, (float*)WristCenter);
//IMPLEMENTED MATH IS EQUIVALENT TO:   WristCenter = OpPoint-OpRot*OpPointoffset;

//Position IK Problem 
//This you will have likely have to solve yourself.  I will attempt to work it 
//in terms of the DH Model described in the documentation to this, but its not hard. 
//the inverse position problem is just simple Trigonometry (SEE DOCUMENTATION)
float L=(sqrt(pow((WristCenter[0][0]),2)+pow((WristCenter[1][0]),2)-(pow((d2+d3),2)))-a1); // this is the horizontal distance the bicep and forearm must reach 
double th1=(atan2((WristCenter[1][0]),(WristCenter[0][0])) - atan2(L,((L/(L+a1)*(d2+d3)))));//THIS ONLY SOLVES FOR FORWARD REACH (arm cant reach over head)
float B=sqrt((a3*a3) + (d4*d4));//center to center distance from J3 to Wrist Center
float R=sqrt((L*L) +((pow((WristCenter[2][0]) - d1,2))));//Reaching distance of bicep and forearm
float th2;
float th3;
if (R >(a2+B)){  //This checks to see if the desired point is within the working envelope of the arm
   R=(a2+B)-0.01;
   //Serial.println("POSITION OUT OF RANGE");
}
  float D=(((R*R) - (a2*a2) - (B*B))/(2*a2*B)); //acos of angle between bicep and B
  //Serial.println(D);
  th2=atan2(B*sqrt(1-(D*D)),(a2+B*D))+atan2((WristCenter[2][0]-d1),L)-1.57079632679; //Theta2
  th3=(atan2(-sqrt(1-(D*D)),D)) + (atan2(d4,a3)) - 1.57079632679;//Theta3

//WRIST ORIENTATION IK
//Define Transformation Matricies of J1, J2, J3
 float A1[4][4];
DHTrans((th1+th1offset), d1, a1, alpha1,A1);
 float A2[4][4];
DHTrans((th2+th2offset), d2, a2, alpha2,A2);
 float A3[4][4];
DHTrans((th3+th3offset), d3, a3, alpha3,A3);
//float T1[4][4] = A1;
 float T2[4][4];
Matrix.Multiply((float*)A1, (float*)A2, 4, 4, 4, (float*)T2);
 float T3[4][4];
Matrix.Multiply((float*)T2, (float*)A3, 4, 4, 4, (float*)T3);
float T3temp [3][3];
float T3sub[3][3];
T3sub[0][0] = T3[0][0]; //might be able to simplify this code
T3sub[0][1] = T3[0][1];
T3sub[0][2] = T3[0][2];
T3sub[1][0] = T3[1][0];
T3sub[1][1] = T3[1][1];
T3sub[1][2] = T3[1][2];
T3sub[2][0] = T3[2][0];
T3sub[2][1] = T3[2][1];
T3sub[2][2] = T3[2][2];
Matrix.Transpose((float*)T3sub, 3, 3, (float*)T3temp);
float WR[3][3];
Matrix.Multiply((float*)T3temp, (float*)OpRot, 3, 3, 3, (float*)WR);
//Find required rotation matrix R3 to 6(combined rot matrix of J4, J5,J6)
//IMPLEMENTED MATH IS EQUIVALENT TO:      WR=transpose(T3(1:3,1:3))*OpRot; 
//See documentation for description of this


//inorder to choose between wrist-up case and wrist-down case, we need to
//compare the calcualted angles of the 2 solutions and choose the best one
float th51=atan2(sqrt(1-pow(WR[2][1],2)),WR[2][1]);//calculate th5 wrist-up
float th41=atan2(WR[1][1],WR[0][1]);//calculate th4 wrist-up
float th61=atan2(WR[2][0],-(WR[2][2]));//calculate th6 wrist-up
float th52=atan2(-sqrt(1-pow(WR[2][1],2)),WR[2][1]);//calculate th5 wrist-down
float th42=atan2(-(WR[1][1]),-(WR[0][1]));//calculate th4 wrist-down
float th62=atan2(-(WR[2][0]),WR[2][2]);//calculate th6 wrist-down

//The expression below compares the total angular distance the wrist joints
//would have to travel to reach each solution. it then chooses the solution
//requiring the least movement
float th4;
float th5;
float th6;
if ((angledist(old5,th51)+angledist(old4,th41)+angledist(old6,th61))>(angledist(old5,th52)+angledist(old4,th42)+angledist(old6,th62))){
  th5=th52;
  th4=th42;
  th6=th62;
  //orient='D';
}
else{
  th5=th51;
  th4=th41;
  th6=th61;
  //orient='U';
}
 
 //This handles the case if the wrist is at its singularity
 if (abs(th5)<0.005){
  th4=old4;
  th6=(atan2(WR[1][2],WR[0][2])-old4);
 }

  th1 = negativeDegreeCorrection(th1);
  th2 = negativeDegreeCorrection(th2);
  th3 = negativeDegreeCorrection(th3);
  th4 = negativeDegreeCorrection(th4);
  th5 = negativeDegreeCorrection(th5);
  th6 = negativeDegreeCorrection(th6);
  
  angles[0] = degrees(th1);
  angles[1] = degrees(th2);
  angles[2] = degrees(th3);
  angles[3] = degrees(th4);
  angles[4] = degrees(th5);
  angles[5] = degrees(th6);
}


//END OF NEW IK


/*
//Gravity Compensation stuff
void GravityTorqueCalc(float angles[ArmJointCount], float gravtorques[ArmJointCount]){
    fcgx=0; //x coordinate of forearm cg (following RCS when forearm is horizontal)
    fcgy=10;
    fcgz=0;

float AforearmCG[4][4]; 
   AforearmCG[0][0] =1;
   AforearmCG[0][1] = 0;
   AforearmCG[0][2] = 0;
   AforearmCG[0][3] = fcgz-a3;
   AforearmCG[1][0] =0 ;
   AforearmCG[1][1] = 1;
   AforearmCG[1][2] = 0;
   AforearmCG[1][3] = fcgx;
   AforearmCG[2][0] = 0;
   AforearmCG[2][1] = 0;
   AforearmCG[2][2] = 1;
   AforearmCG[2][3] = fcgy;
   AforearmCG[3][0] = 0;
   AforearmCG[3][1] = 0;
   AforearmCG[3][2] = 0;
   AforearmCG[3][3] = 1;
    
 float A1[4][4];
DHTrans((th1+th1offset), d1, a1, alpha1,A1);
 float A2[4][4];
DHTrans((th2+th2offset), d2, a2, alpha2,A2);
 float A3[4][4];
DHTrans((th3+th3offset), d3, a3, alpha3,A3);
//float T1[4][4] = A1;
 float T2[4][4];
Matrix.Multiply((float*)A1, (float*)A2, 4, 4, 4, (float*)T2);
 float T3[4][4];
Matrix.Multiply((float*)T2, (float*)A3, 4, 4, 4, (float*)T3);
 
 float TforearmCG[4][4];
Matrix.Multiply((float*)T3, (float*)AforearmCG, 4, 4, 4, (float*)TforearmCG);

float o0[3];
o0[0] = 0;
o0[1] = 0;
o0[2] = 0;

float o1[3];
o1[0] = A1[0][3];
o1[1] = A1[1][3];
o1[2] = A1[2][3];

float o2[3];
o2[0] = T2[0][3];
o2[1] = T2[1][3];
o2[2] = T2[2][3];

float o3[3];
o3[0] = T3[0][3];
o3[1] = T3[1][3];
o3[2] = T3[2][3];

float z0[3];
z1[0] = 0;
z1[1] = 0;
z1[2] = 1;

float z1[3];
z1[0] = A1[0][2];
z1[1] = A1[1][2];
z1[2] = A1[2][2];

float z2[3];
z2[0] = T2[0][2];
z2[1] = T2[1][2];
z2[2] = T2[2][2];

float z3[3];
z3[0] = T3[0][2];
z3[1] = T3[1][2];
z3[2] = T3[2][2];

float oforearmCG [3];
oforearmCG[0] = TforearmCG[0][3];
oforearmCG[1] = TforearmCG[1][3];
oforearmCG[2] = TforearmCG[2][3];

float J2[6][3];
J2[0][0] = (z0[1] * (oforearmCG[2] - o0[2])) - (z0[2] * (oforearmCG[1] - o0[1])) ;
J2[0][1] = (z1[1] * (oforearmCG[2] - o1[2])) - (z1[2] * (oforearmCG[1] - o1[1])) ;   
J2[0][2] = (z2[1] * (oforearmCG[2] - o2[2])) - (z2[2] * (oforearmCG[1] - o2[1])) ;
J2[1][0] = (z0[2] * (oforearmCG[0] - o0[0])) - (z0[0] * (oforearmCG[2] - o0[2])) ;  
J2[1][1] = (z1[2] * (oforearmCG[0] - o1[0])) - (z1[0] * (oforearmCG[2] - o1[2])) ;
J2[1][2] = (z2[2] * (oforearmCG[0] - o2[0])) - (z2[0] * (oforearmCG[2] - o2[2])) ;
J2[2][0] = (z0[0] * (oforearmCG[1] - o0[1])) - (z0[1] * (oforearmCG[0] - o0[0])) ; 
J2[2][1] = (z1[0] * (oforearmCG[1] - o1[1])) - (z1[1] * (oforearmCG[0] - o1[0])) ;    
J2[2][2] = (z2[0] * (oforearmCG[1] - o2[1])) - (z2[1] * (oforearmCG[0] - o2[0])) ;  
J2[3][0] =  z0[0];
J2[3][1] =  z1[0];
J2[3][2] =  z2[0];
J2[4][0] =  z0[1];
J2[4][1] =  z1[1];
J2[4][2] =  z2[1];
J2[5][0] =  z0[2];
J2[5][1] =  z1[2];
J2[5][2] =  z2[2];    

float weightforearm = 1;

float Fforearm[6] = {0,0,weightforearm,0,0,0};

float J2Transpose[6][3];
float Torque2[3];
Matrix.Transpose((float*)J2,6,3,(float*)J2Transpose);
Matrix.Multiply((float*)J2Transpose, (float*)Fforearm, 3, 3, 3, (float*)Torque2);



   bcgx=0; //x coordinate of forearm cg (following RCS when forearm is horizontal)
   bcgy=10;
   bcgz=0;

float AbicepCG[4][4]; 
   AbicepCG[0][0] =1;
   AbicepCG[0][1] = 0;
   AbicepCG[0][2] = 0;
   AbicepCG[0][3] = bcgz-a2;
   AbicepCG[1][0] =0 ;
   AbicepCG[1][1] = 1;
   AbicepCG[1][2] = 0;
   AbicepCG[1][3] = -bcgy;
   AbicepCG[2][0] = 0;
   AbicepCG[2][1] = 0;
   AbicepCG[2][2] = 1;
   AbicepCG[2][3] = bcgy;
   AbicepCG[3][0] = 0;
   AbicepCG[3][1] = 0;
   AbicepCG[3][2] = 0;
   AbicepCG[3][3] = 1;

float TbicepCG[4][4];
Matrix.Multiply((float*)T2, (float*)AbicepCG, 4, 4, 4, (float*)TbicepCG);  

float obicepCG [3];
obicepCG[0] = TbicepCG[0][3];
obicepCG[1] = TbicepCG[1][3];
obicepCG[2] = TbicepCG[2][3];

float J1[6][2];
J2[0][0] = (z0[1] * (oforearmCG[2] - o0[2])) - (z0[2] * (oforearmCG[1] - o0[1])) ;
J2[0][1] = (z1[1] * (oforearmCG[2] - o1[2])) - (z1[2] * (oforearmCG[1] - o1[1])) ;   
J2[1][0] = (z0[2] * (oforearmCG[0] - o0[0])) - (z0[0] * (oforearmCG[2] - o0[2])) ;  
J2[1][1] = (z1[2] * (oforearmCG[0] - o1[0])) - (z1[0] * (oforearmCG[2] - o1[2])) ;
J2[2][0] = (z0[0] * (oforearmCG[1] - o0[1])) - (z0[1] * (oforearmCG[0] - o0[0])) ; 
J2[2][1] = (z1[0] * (oforearmCG[1] - o1[1])) - (z1[1] * (oforearmCG[0] - o1[0])) ;    
J2[3][0] =  z0[0];
J2[3][1] =  z1[0];
J2[4][0] =  z0[1];
J2[4][1] =  z1[1];
J2[5][0] =  z0[2];
J2[5][1] =  z1[2]; 

float weightbicep = 1;

float Fbicep[6] = {0,0,weightbicep,0,0,0};

float J1Transpose[6][3];
float Torque1[2];
Matrix.Transpose((float*)J1,6,2,(float*)J1Transpose);
Matrix.Multiply((float*)J1Transpose, (float*)Fbicep, 3, 3, 3, (float*)Torque1);

gravtorques[0]= Torque1[0] + Torque2[0];
gravtorques[1]= Torque1[1] + Torque2[1];
gravtorques[2]= Torque2[2];
gravtorques[3]=0;
gravtorques[4]=0;
gravtorques[5]=0;
}
//end grav comp stuff
*/ 



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
