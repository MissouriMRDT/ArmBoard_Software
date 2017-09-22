#include "arm.h"

Ma3Encoder12b joint1Encoder(ENCODER1_READING_PIN);
Ma3Encoder12b joint2Encoder(ENCODER2_READING_PIN);
Ma3Encoder12b joint3Encoder(ENCODER3_READING_PIN);
Ma3Encoder12b joint5Encoder(ENCODER5_READING_PIN);

PIAlgorithm joint1Alg(BaseRotateKp,BaseRotateKi,PI_TIMESLICE_SECONDS, &joint1Encoder);
PIAlgorithm joint2Alg(BaseTiltKp,BaseTiltKi,PI_TIMESLICE_SECONDS, &joint2Encoder);
PIAlgorithm joint3Alg(ElbowKp,ElbowKi,PI_TIMESLICE_SECONDS, &joint3Encoder, ElbowMinMag);
PIAlgorithm joint5Alg(WristTiltKp,WristTiltKi,PI_TIMESLICE_SECONDS, &joint5Encoder, WristTiltMinMag);

GenPwmPhaseHBridge dev1(MOT1_PWN_PIN, HBRIDGE1_PHASE_PIN, HBRIDGE1_NSLEEP_PIN, true, false);
GenPwmPhaseHBridge dev2(MOT2_PWN_PIN, HBRIDGE2_PHASE_PIN, HBRIDGE2_NSLEEP_PIN, true, true);
GenPwmPhaseHBridge dev3(MOT3_PWN_PIN, HBRIDGE3_PHASE_PIN, HBRIDGE3_NSLEEP_PIN, true, false);
GenPwmPhaseHBridge dev4(MOT4_PWN_PIN, HBRIDGE4_PHASE_PIN, HBRIDGE4_NSLEEP_PIN, true, true);
GenPwmPhaseHBridge dev5(MOT5_PWN_PIN, HBRIDGE5_PHASE_PIN, HBRIDGE5_NSLEEP_PIN, true, false);
GenPwmPhaseHBridge gripMotorDev(GRIPMOT_PWM_PIN, GRIPMOT_PHASE_PIN, GRIPMOT_NENABLE_PIN, false, false);
RCContinuousServo gripServoDev(GRIPPER_SERVO_PWM_PIN, false);

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

void setup()
{
  roveComm_Begin(192, 168, 1, 131);

  joint1.pairDifferentialJoint(&joint2);
  joint4.pairDifferentialJoint(&joint5);

  //initialze to open loop control format
  switchToOpenLoop();

  masterPowerSet(true);

  allMotorsPowerSet(true);

  //set timer 0 to fire at a rate where the different PI algorithms will all be updated at their expected timeslice in seconds.
  //There are 5 controls to update independently. They update one at a time, one being serviced every time the timer fires. So it takes 5 timer
  //firings for any individual control to get updated again. Meaning the timeslice of the timer itself must be one fifth of the PI algorithms overall timeslice so that
  //when it cycles back around the overall timeslice will have passed
  //Update: 4 controls are now used, but the setup still works just fine by firing off 5 times per overall timeslice and
  //changing it would require modifying the interrupt as well, so it's staying the way it is
  setupTimer7((PI_TIMESLICE_SECONDS/5.0) * 1000000.0); //function expects microseconds

  joint1Alg.setDeadband(BaseRotateDeadband);
  joint1Alg.setHardStopPositions(BaseRotateHardStopUp, BaseRotateHardStopDown);
  joint2Alg.setDeadband(BaseTiltDeadband);
  joint2Alg.setHardStopPositions(BaseTiltHardStopUp, BaseTiltHardStopDown);
  joint3Alg.setDeadband(ElbowDeadband);
  joint3Alg.setHardStopPositions(ElbowHardStopUp, ElbowHardStopDown);
  joint5Alg.setDeadband(WristTiltDeadband);
  joint5Alg.setHardStopPositions(WristTiltHardStopUp, WristTiltHardStopDown);

  joint1Encoder.setOffsetAngle(BaseRotateOffsetAngle);
  joint2Encoder.setOffsetAngle(BaseTiltOffsetAngle);
  joint3Encoder.setOffsetAngle(ElbowOffsetAngle);
  joint5Encoder.setOffsetAngle(WristTiltOffsetAngle);

  dev3.setRampUp(ElbowRampUp);
  dev3.setRampDown(ElbowRampDown);
  dev1.setRampUp(BaseRampUp);
  dev2.setRampUp(BaseRampUp);
  dev1.setRampDown(BaseRampDown);
  dev2.setRampDown(BaseRampDown);
  dev4.setRampUp(WristRampUp);
  dev4.setRampDown(WristRampDown);
  dev5.setRampUp(WristRampUp);
  dev5.setRampDown(WristRampDown);

  delay(2000); //let background processes finish before turning on the watchdog. Experimentation found that 2 seconds worked while values such as 1.5 resulted in program failure

  initWatchdog(WATCHDOG_TIMEOUT_US);

  initialized = true;
}

void loop()
{
  uint16_t commandId = 0;
  size_t commandSize = 0;
  char commandData[250];
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
          //float absoluteAngles[ArmJointCount];
          //computeIK((float*)(commandData), absoluteAngles);
          //setArmDestinationAngles(absoluteAngles);

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
  if(enable)
  {
    digitalPinWrite(POWER_LINE_CONTROL_PIN, 1);
  }
  else
  {
    digitalPinWrite(POWER_LINE_CONTROL_PIN, 0);
  }

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
  static bool limitSwitchHit = false;
  static int moveAllowedDir = 0;

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
  static bool limitSwitchHit = false;
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
  }
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
    TimerIntClear(TIMER7_BASE, TIMER_TIMA_TIMEOUT);
    TimerDisable(TIMER7_BASE, TIMER_A);
    TimerIntDisable(TIMER7_BASE, TIMER_TIMA_TIMEOUT);

    joint1.removeAlgorithm(InputPowerPercent);
    joint2.removeAlgorithm(InputPowerPercent);
    joint3.removeAlgorithm(InputPowerPercent);
    joint5.removeAlgorithm(InputPowerPercent);
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

    //enable closed loop interrupts, which will begin to move the arm towards its set destinations
    TimerIntClear(TIMER7_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntEnable(TIMER7_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER7_BASE, TIMER_A);
  }

  return Success;
}

//sets up timer 7 so that it can service closed loop functionality;
//closed loop works by periodically updating all of the joints' positional destinations on a consistent timeslice.
//Safest option for this service is to use a timer, and timers 1-5 are in use (timers 1-5 are used to read pwm).
//After setup, timer remains ready but not running. The switchToClosedLoop function turns on the timer and its interrupt, while switchToOpenLoop turns it back off.
void setupTimer7(float timeout_micros)
{
  uint32_t timerLoad = 16000000.0 * (timeout_micros/1000000.0); // timer clock cycle (16Mhz cycle/second) * (microsecond timeout/10000000 to convert it to seconds) = cycles till the timeout passes

  //enable timer hardware
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER7);

  delay(1); //let the periph finish processing

  //set clock to internal precision clock of 16 Mhz
  TimerClockSourceSet(TIMER7_BASE, TIMER_CLOCK_PIOSC);

  //configure timer for count up periodic
  TimerConfigure(TIMER7_BASE, TIMER_CFG_PERIODIC);

  //set timer load based on earlier calculated value
  TimerLoadSet(TIMER7_BASE, TIMER_A, (timerLoad));

  //set up interrupts. The order here is actually important, TI's forums reccomend
  //setting up new interrupts in this exact fashion
  TimerIntClear(TIMER7_BASE, TIMER_TIMA_TIMEOUT);
  TimerIntEnable(TIMER7_BASE, TIMER_TIMA_TIMEOUT);
  IntEnable(INT_TIMER7A);

  //register interrupt functions
  TimerIntRegister(TIMER7_BASE, TIMER_A, &closedLoopUpdateHandler);

  //enable master system interrupt
  IntMasterEnable();

  delay(1);
}

//sets up the watchdog timer. Watchdog timer will restart the processor and the program when it times out
//input: timeout value in microseconds
void initWatchdog(uint32_t timeout_us)
{
  if(watchdogUsed == false)
    return;
  uint32_t load = Fcpu * (timeout_us/1000000.0); // clock cycle (120 MHz cycle/second) * (microsecond timeout/10000000 to convert it to seconds) = cycles till the timeout passes
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
    uint32_t load = Fcpu * (timeout_us/1000000.0); // clock cycle (120 MHz cycle/second) * (microsecond timeout/10000000 to convert it to seconds) = cycles till the timeout passes
    load /=2; //watchdog resets after two timeouts

    WatchdogReloadSet(WATCHDOG0_BASE, load);
  }
}

//fills a float array with the current positions of the joints.
//Angles are numerically described as 0-360 degrees
CommandResult getArmPositions(float positions[ArmJointCount])
{
  positions[0] = joint1Encoder.getFeedback() * ((360.0-0.0)/((float)(POS_MAX - POS_MIN))); //getFeedback returns from POS_MAX to POS_MIN long, convert it to 0-360 degrees float
  positions[1] = joint2Encoder.getFeedback() * ((360.0-0.0)/((float)(POS_MAX - POS_MIN)));
  positions[2] = joint3Encoder.getFeedback() * ((360.0-0.0)/((float)(POS_MAX - POS_MIN)));
  positions[4] = joint5Encoder.getFeedback() * ((360.0-0.0)/((float)(POS_MAX - POS_MIN)));

  return Success;
}

//takes in a series of 4 coordinates (x,y,z, and gripper angle) and uses inverse kinematics to compute
//what angle each individual joint of the arm needs to be at for the gripper's endpoint to be at that coordinate (wrist rotate aside,
//it doesn't factor into the positional math).
//input: the 4 coordinates, describing x,y,z coordiantes in inches, and gripper angle in degrees
//output: 5 arm joint angles (for joints 1-5) in 0-360 degrees. Note that joint 4 is junk data, as it's not computed
void computeIK(float coordinates[IKArgCount], float angles[ArmJointCount])
{
 /* float temp;
  float temp2;
  float tempNum;
  float tempDen;
  float cX = coordinates[0];
  float cY = coordinates[1];
  float cZ = coordinates[2];
  float gripperAngle = coordinates[3];

  float joint1Angle;
  float joint2Angle;
  float joint3Angle;
  float joint4Angle;
  float joint5Angle;
  float d = sqrt((cX*cX) + (cY*cY));
  constrain(d, 0, (ElbowLength + WristLength));
  float w = sqrt(d*d + (cZ-BaseLength)*(cZ-BaseLength));
  constrain(w, (ElbowLength-WristLength), (ElbowLength + WristLength));

  joint1Angle = negativeDegreeCorrection(atan2(cY, cX)); //joints will start being expressed in radians

  temp = ElbowLength*ElbowLength + w*w - WristLength*WristLength; //elbow length = R2, base length = R1, Wrist length = R3
  temp /= (2*ElbowLength*w);
  temp = acos(temp);
  temp2 = negativeDegreeCorrection(atan2(cZ-BaseLength, d));

  joint2Angle = temp + temp2;

  temp = ElbowLength*ElbowLength + WristLength*WristLength - w*w;
  temp /= (2*ElbowLength*WristLength);
  joint3Angle = acos(temp);

  joint5Angle = negativeDegreeCorrection(PI -joint3Angle - joint2Angle + gripperAngle); //M_PI given in math.h

  angles[0] = joint1Angle;
  angles[1] = joint2Angle;
  angles[2] = joint3Angle;
  angles[3] = joint4Angle;
  angles[4] = joint5Angle;

  //convert angles from 0-2pi to 0-360
  int i;
  for(i = 0; i < 5; i++)
  {
    angles[i] = angles[i] * 360.0 / (2.0*PI);
  }

  //make sure values are constrained
  for(i = 0; i < 5; i++)
  {
    while(angles[i] < 0)
    {
      angles[i] += 360;
    }
    while(angles[i] > 360)
    {
      angles[i] -= 360;
    }
  }*/
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
  static int jointUpdated = 1;
  TimerIntClear(TIMER7_BASE, TIMER_TIMA_TIMEOUT);
  restartWatchdog(WATCHDOG_TIMEOUT_US);

  jointUpdated += 1;
  if(jointUpdated > 5)
  {
    jointUpdated = 1;
  }
  if(jointUpdated == 1)
  {
    //joint1.runOutputControl(joint1Destination);
  }
  else if(jointUpdated == 2)
  {
    //joint2.runOutputControl(joint2Destination);
  }
  else if(jointUpdated == 3)
  {
    joint3.runOutputControl(joint3Destination);
  }
  else if(jointUpdated == 4)
  {
    //joint4.runOutputControl(joint4Destination); only 4 joints used
  }
  else if(jointUpdated == 5)
  {
    //joint5.runOutputControl(joint5Destination);
  }
}


