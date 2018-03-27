#include "main.h"
#include <stdio.h>
#include "RoveCommSerial.h"
#include "MatrixMath.h"

Ma3Encoder12b joint1Encoder(ReadModule0, ENCODER1_READING_PIN);
Ma3Encoder12b joint2Encoder(ReadModule1, ENCODER2_READING_PIN);
Ma3Encoder12b joint3Encoder(ReadModule2, ENCODER3_READING_PIN);
Ma3Encoder12b joint4Encoder(ReadModule3, ENCODER4_READING_PIN);
Ma3Encoder12b joint5Encoder(ReadModule4, ENCODER5_READING_PIN);
Ma3Encoder12b joint6Encoder(ReadModule5, ENCODER6_READING_PIN);

VelocityDeriver joint1Vel(&joint1Encoder, .9);
VelocityDeriver joint2Vel(&joint2Encoder, .9);
VelocityDeriver joint3Vel(&joint3Encoder, .1);
VelocityDeriver joint5Vel(&joint5Encoder, .9);

PIAlgorithm joint1Alg(BaseRotateKp,BaseRotateKi,PI_TIMESLICE_SECONDS, &joint1Encoder);
PIAlgorithm joint2Alg(BaseTiltKp,BaseTiltKi,PI_TIMESLICE_SECONDS, &joint2Encoder);
PIAlgorithm joint3Alg(ElbowTiltKp,ElbowTiltKi,PI_TIMESLICE_SECONDS, &joint3Encoder);
PIAlgorithm joint4Alg(ElbowRotateKp,ElbowRotateKi,PI_TIMESLICE_SECONDS, &joint4Encoder);
PIAlgorithm joint5Alg(WristTiltKp,WristTiltKi,PI_TIMESLICE_SECONDS, &joint5Encoder);
PIAlgorithm joint6Alg(WristRotateKp,WristRotateKi,PI_TIMESLICE_SECONDS, &joint6Encoder);

PIVConverter joint1PIV(BaseRotateKp, BaseRotateKi, BaseRotateKp, BaseRotateKi, PIV_TIMESLICE_SECONDS, &joint1Encoder, &joint1Vel);
PIVConverter joint2PIV(BaseTiltKp, BaseTiltKi, BaseTiltKp, BaseTiltKi, PIV_TIMESLICE_SECONDS, &joint2Encoder, &joint2Vel);
PIVConverter joint3PIV(1, 0, 1, 0, PIV_TIMESLICE_SECONDS, &joint3Encoder, &joint3Vel);
PIVConverter joint5PIV(WristTiltKp, WristTiltKi, WristTiltKp, WristTiltKi, PIV_TIMESLICE_SECONDS, &joint5Encoder, &joint5Vel);

AtlasArmConstants atlasConsts(0, 0, 0, 0, 0, 0, th1offset, d1, a1, alpha1, th2offset, d2, a2, alpha2, th3offset, d3, a3, alpha3, th4offset, a4,
                                alpha4, th5offset, d5, a5, alpha5, th6offset, d6, alpha6, OpPointoffset[0], OpPointoffset[1], OpPointoffset[2],
                                &joint1Encoder, &joint2Encoder, &joint3Encoder, &joint4Encoder, &joint5Encoder, &joint6Encoder);

GravityInertiaSystemStatus sysStatus(AtlasArm, &atlasConsts);

GravityCompensator j1Grav(&sysStatus, TorqueConvert_BrushedDC, J12Kt, J1Resistance, MotorVoltage, 1);
GravityCompensator j2Grav(&sysStatus, TorqueConvert_BrushedDC, J12Kt, J2Resistance, MotorVoltage, 2);
GravityCompensator j3Grav(&sysStatus, TorqueConvert_BrushedDC, J3Kt, J3Resistance, MotorVoltage, 3);
GravityCompensator j4Grav(&sysStatus, TorqueConvert_BrushedDC, J45Kt, J4Resistance, MotorVoltage, 4);
GravityCompensator j5Grav(&sysStatus, TorqueConvert_BrushedDC, J45Kt, J5Resistance, MotorVoltage, 5);

VNH5019WithPCA9685 baseRotateDriver(PcaChipAddress, 0, HBRIDGE1_INA, HBRIDGE1_INB, PcaI2cModule, PWM_DRIVER_SCL, PWM_DRIVER_SDA, true);
VNH5019WithPCA9685 baseTiltDriver(PcaChipAddress, 1, HBRIDGE2_INA, HBRIDGE2_INB, PcaI2cModule, PWM_DRIVER_SCL, PWM_DRIVER_SDA, false
    );
VNH5019WithPCA9685 elbowTiltDriver(PcaChipAddress, 2, HBRIDGE3_INA, HBRIDGE3_INB, PcaI2cModule, PWM_DRIVER_SCL, PWM_DRIVER_SDA, false);
VNH5019WithPCA9685 elbowRotateDriver(PcaChipAddress, 3, HBRIDGE4_INA, HBRIDGE4_INB, PcaI2cModule, PWM_DRIVER_SCL, PWM_DRIVER_SDA, true);
VNH5019WithPCA9685 wristTiltDriver(PcaChipAddress, 4, HBRIDGE5_INA, HBRIDGE5_INB, PcaI2cModule, PWM_DRIVER_SCL, PWM_DRIVER_SDA, false);
VNH5019WithPCA9685 wristRotateDriver(PcaChipAddress, 5, HBRIDGE6_INA, HBRIDGE6_INB, PcaI2cModule, PWM_DRIVER_SCL, PWM_DRIVER_SDA, false);
VNH5019WithPCA9685 gripperDriver(PcaChipAddress, 6, HBRIDGE7_INA, HBRIDGE7_INB, PcaI2cModule, PWM_DRIVER_SCL, PWM_DRIVER_SDA, true);
VNH5019WithPCA9685 pokerDriver(PcaChipAddress, 7, HBRIDGE8_INA, HBRIDGE8_INB, PcaI2cModule, PWM_DRIVER_SCL, PWM_DRIVER_SDA, true);

SingleMotorJoint gripper(InputPowerPercent, &gripperDriver);
SingleMotorJoint poker(InputPowerPercent, &pokerDriver);

SingleMotorJoint joint1(InputPowerPercent, &baseRotateDriver); //joints initialized to open loop state
SingleMotorJoint joint2(InputPowerPercent, &baseTiltDriver);
SingleMotorJoint  joint3(InputPowerPercent, &elbowTiltDriver);
SingleMotorJoint  joint4(InputPowerPercent, &elbowRotateDriver);
DifferentialJoint joint5(DifferentialTilt, InputPowerPercent, &wristTiltDriver, &wristRotateDriver);
DifferentialJoint joint6(DifferentialRotate, InputPowerPercent, &wristTiltDriver, &wristRotateDriver);

//variables used to control joints during closed loop control
unsigned long joint1Destination;
unsigned long joint2Destination;
unsigned long joint3Destination;
unsigned long joint4Destination;
unsigned long joint5Destination;
unsigned long joint6Destination;

ControlSystems currentControlSystem; //tracks what control system arm is currently using
bool mainPowerOn;
bool m1On;
bool m2On;
bool m3On;
bool m4On;
bool m5On;
bool m6On;
bool gripMotOn;
bool initialized = false;  //tracks if program setup is finished. Needed as some closed loop interrupts will fail if parts of their code is run before initialize is finished, so this flag
                           //prevents fragile hardware calls from firing before then
bool limitsEnabled = true; //tracks if hardware limit switches are being used or if they're being overridden
bool watchdogUsed = false;

RoveTimer_Handle timer7Handle;
RoveTimer_Handle timer6Handle;

void init()
{
  roveComm_Begin(192, 168, 1, 131);
  //joint1.pairDifferentialJoint(&joint2);
  joint6.pairDifferentialJoint(&joint5);

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
  timer7Handle = setupTimer(Timer7, TimerPeriodicInterrupt, (PI_TIMESLICE_SECONDS/5.0) * 1000000.0, &closedLoopUpdateHandler);
  timer6Handle = setupTimer(Timer6, TimerPeriodicInterrupt, (PI_TIMESLICE_SECONDS) * 1000000, &sysStatusUpdater);

  joint1Alg.setDeadband(BaseRotateDeadband);
  joint2Alg.setHardStopPositions(BaseTiltHardStopDown, BaseTiltHardStopUp);
  joint2Alg.setDeadband(BaseTiltDeadband);
  joint3Alg.setDeadband(ElbowTiltDeadband);
  joint3Alg.setHardStopPositions(ElbowTiltHardStopUp, ElbowTiltHardStopDown);
  joint4Alg.setDeadband(ElbowRotateDeadband);
  joint5Alg.setDeadband(WristTiltDeadband);
  joint6Alg.setDeadband(WristRotateDeadband);

  joint1Encoder.setOffsetAngle(BaseRotateOffsetAngle);
  joint2Encoder.setOffsetAngle(BaseTiltOffsetAngle);
  joint3Encoder.setOffsetAngle(ElbowTiltOffsetAngle);
  joint4Encoder.setOffsetAngle(ElbowRotateOffsetAngle);
  joint5Encoder.setOffsetAngle(WristTiltOffsetAngle);
  joint6Encoder.setOffsetAngle(WristRotateOffsetAngle);

  joint1Encoder.setMaxPwm(4190);
  joint2Encoder.setMaxPwm(4020);
  joint3Encoder.setMaxPwm(4030);
  joint4Encoder.setMaxPwm(4040);
  joint5Encoder.setMaxPwm(4285);
  joint6Encoder.setMaxPwm(4045);

  joint2Encoder.setFilterConstant(.2);
  joint6Encoder.setFilterConstant(.2);
  joint4Encoder.reverseDirection(true);
  j3Grav.setScalar(ScalarJ3);
  //joint1Alg.addSupportingAlgorithm(&j1Grav);
  //joint2Alg.addSupportingAlgorithm(&j2Grav);
  //joint3Alg.addSupportingAlgorithm(&j3Grav);
  //joint5Alg.addSupportingAlgorithm(&j5Grav);

  joint1.stop();
  joint2.stop();
  joint3.stop();
  joint4.stop();
  joint5.stop();
  joint6.stop();
  gripper.stop();

  delay(2000); //let background processes finish before turning on the watchdog. Experimentation found that 2 seconds worked while values such as 1.5 resulted in program failure

  //initWatchdog(WATCHDOG_TIMEOUT_US);
  initialized = true;
}

int main()
{
  init();
  uint16_t commandId = 0;
  size_t commandSize = 0;
  char commandData[40];
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
          result = moveJ6(*(int16_t*)(commandData));
          break;

        case MoveGripServo: //gripper only ever operates in open loop but the rest of the system can be using other controls at the same time
          result = moveGripper(*(int16_t*)(commandData));
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
          getArmPositions(absoluteAngles);
          Calc_IK((float*)(commandData), absoluteAngles);
          setArmDestinationAngles(absoluteAngles);
          break;

        case ArmGetPosition:
          float currentPositions[6];
          getArmPositions(currentPositions);
          roveComm_SendMsg(ArmCurrentPosition, sizeof(float) * 6, currentPositions);
          break;

        case ArmEnableJ1:
          j1PowerSet(*(bool*)commandData);
          break;

        case ArmEnableJ2:
          j2PowerSet(*(bool*)commandData);
          break;

        case ArmEnableJ3:
          j3PowerSet(*(bool*)commandData);
          break;

        case ArmEnableJ4:
          j4PowerSet(*(bool*)commandData);
          break;

        case ArmEnableJ5:
          j56PowerSet(*(bool*)commandData);
          break;

        case ArmEnableJ6:
          j56PowerSet(*(bool*)commandData);
          break;

        case ArmEnableEndeff:
          pokerPowerSet(*(bool*)commandData);
          break;

        case ArmEnableServo:
          gripperPowerSet(*(bool*)commandData);
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
  j1PowerSet(enable);
  j3PowerSet(enable);
  j3PowerSet(enable);
  j4PowerSet(enable);
  j56PowerSet(enable);
  pokerPowerSet(enable);
  gripperPowerSet(enable);
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
  joint6.stop();
  gripper.stop();
  return Success;
}

//turns on or off the motors attached to joint 1
void j1PowerSet(bool powerOn)
{
  if(powerOn)
  {
    joint1.enableJoint();
  }
  else
  {
    joint1.disableJoint();
  }

  m1On = powerOn;
}

//turns on or off the motors attached to joint 2
void j2PowerSet(bool powerOn)
{
  if(powerOn)
  {
    joint2.enableJoint();
  }
  else
  {
    joint2.disableJoint();
  }

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

//turns on or off the motors attached to joint 4
void j4PowerSet(bool powerOn)
{
  if(powerOn)
  {
    joint4.enableJoint();
  }
  else
  {
    joint4.disableJoint();
  }

  m4On = powerOn;
}

//turns on or off the motors attached to joint 5 and 6
void j56PowerSet(bool powerOn)
{
  if(powerOn)
  {
    joint5.enableJoint();
    joint6.enableJoint();
  }
  else
  {
    joint5.disableJoint();
    joint6.disableJoint();
  }

  m5On = powerOn;
  m6On = powerOn;
}

//turns on or off the motor attached to the poker
void pokerPowerSet(bool powerOn)
{
  if(powerOn)
  {
    poker.enableJoint();
  }
  else
  {
    poker.disableJoint();
  }
}

//turns on or off the gripper
void gripperPowerSet(bool powerOn)
{
  if(powerOn)
  {
    gripper.enableJoint();
  }
  else
  {
    gripper.disableJoint();
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
  joint4Destination = angles[3] * (((float)(POS_MAX - POS_MIN))/(360.0-0.0));
  joint5Destination = angles[4] * (((float)(POS_MAX - POS_MIN))/(360.0-0.0));
  joint6Destination = angles[5] * (((float)(POS_MAX - POS_MIN))/(360.0-0.0));

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
  /*static bool limitSwitchHit = false;
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
  }*/

  joint5.runOutputControl(moveValue);

  return Success;
}

CommandResult moveJ6(int16_t moveValue)
{
  moveValue *= 2;

  joint6.runOutputControl(moveValue);

  return Success;
}

CommandResult moveGripper(int16_t moveValue)
{
  if(moveValue > 0)
    moveValue = 1000;
  else if(moveValue < 0)
    moveValue = -1000;
  gripper.runOutputControl(moveValue);

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
    joint4.removeIOConverter(InputPowerPercent);
    joint5.removeIOConverter(InputPowerPercent);
    joint6.removeIOConverter(InputPowerPercent);
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
  joint4Destination = joint4Encoder.getFeedback();
  joint5Destination = joint5Encoder.getFeedback();
  joint6Destination = joint6Encoder.getFeedback();

  if(initialized)
  {
    joint1.switchModules(InputPosition, &joint1Alg);
    joint2.switchModules(InputPosition, &joint2Alg);
    joint3.switchModules(InputPosition, &joint3Alg);
    joint4.switchModules(InputPosition, &joint4Alg);
    joint5.switchModules(InputPosition, &joint5Alg);
    joint6.switchModules(InputPosition, &joint6Alg);

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
  positions[5] = joint6Encoder.getFeedbackDegrees();

  return Success;
}

//BEGINNING OF NOVA IK
//Still neeed to include matrixmath and math.h somewhere. im not a compsci

//ANGLES ARE IN RADIANS!!!!!
//DISTANCES ARE IN INCHES!!!
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
  //float old1=radians(angles[0]);  //Current Joint angles
  //float old2=radians(angles[1]);
  //float old3=radians(angles[2]);
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
  
  float Rotzyaw2[3][3];
  float Rotxpitch2[3][3];
  
  float t = coordinates[3]; //crashes if you pass it directly in for some reason
  Rotz(t,Rotzyaw);
  t = coordinates[4];
  Rotx(t,Rotxpitch);
  t = coordinates[5];
  Roty(t,Rotyroll);
  t = coordinates[6];
  Rotx(t,Rotxpitch2);
  t = coordinates[7];
  Rotz(t,Rotzyaw2);
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

//Forward Kinematics
void Calc_FK(float angles[ArmJointCount],float coordinates[IKArgCount],float T6[4][4]){
 
 float th1 = radians(angles(0));
 float th2 = radians(angles(1));
 float th3 = radians(angles(2));
 float th4 = radians(angles(3));
 float th5 = radians(angles(4));
 float th6 = radians(angles(5));
  
 float A1[4][4];
 DHTrans((th1+th1offset), d1, a1, alpha1,A1);
 float A2[4][4];
 DHTrans((th2+th2offset), d2, a2, alpha2,A2);
 float A3[4][4];
 DHTrans((th3+th3offset), d3, a3, alpha3,A3);
 float A4[4][4];
 DHTrans((th3+th3offset), d3, a3, alpha3,A3);
 float A5[4][4];
 DHTrans((th3+th3offset), d3, a3, alpha3,A3);
 float A6[4][4];
 DHTrans((th3+th3offset), d3, a3, alpha3,A3);
 float EE[4][4];
 EE[0][0] = 1;
 EE[0][1] = 0;
 EE[0][2] = 0; 
 EE[0][3] = OpPointoffset(0);  
 EE[1][0] = 0;
 EE[1][1] = 1;
 EE[1][2] = 0;
 EE[1][3] = OpPointoffset(1);
 EE[2][0] = 0;
 EE[2][1] = 0;
 EE[2][2] = 1;
 EE[2][3] = OpPointoffset(2);
 EE[3][0] = 0;
 EE[3][1] = 0;
 EE[3][2] = 0; 
 EE[3][3] = 1; 
 
  //float T6[4][4];
  Matrix.Multiply((float*)A1, (float*)A2, 4, 4, 4, (float*)T6);
  Matrix.Multiply((float*)T6, (float*)A3, 4, 4, 4, (float*)T6);
  Matrix.Multiply((float*)T6, (float*)A4, 4, 4, 4, (float*)T6);
  Matrix.Multiply((float*)T6, (float*)A5, 4, 4, 4, (float*)T6);
  Matrix.Multiply((float*)T6, (float*)A6, 4, 4, 4, (float*)T6);
  Matrix.Multiply((float*)T6, (float*)EE, 4, 4, 4, (float*)T6);
  
  coordinates(0) = T6[0][3];
  coordinates(1) = T6[1][3];
  coordinates(2) = T6[2][3];
  coordinates(3) = degrees(negativeDegreeCorrection(atan2(-T6[0][1],T6[1][1])));
  coordinates(4) = degrees(negativeDegreeCorrection(atan2(T6[2][1],sqrt(1-pow(T6[2][1],2)))));
  coordinates(5) = degrees(negativeDegreeCorrection(atan2(T6[2][0],T6[2][2])));
  
}

//FOR RELATIVE IK
//run Calc_FK code to get T6 matrix and position/orientation
// multiply first 3x3 matrix from T6 by desired new gripper incremental position vector
//take the result(3X1) and add to current x, y, z position
// use current orientation values in Calc_IK, but add incremental desired Roll to roll argument
//the 2 new input arguments are relative gripper incremental pitch and yaw respectively




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
  //static RoveI2c_Handle handle = i2cInit(2, I2C_STANDARD)
  restartWatchdog(WATCHDOG_TIMEOUT_US);

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
    joint4.runOutputControl(joint4Destination);
  }
  else if(jointUpdated == 5)
  {
    joint5.runOutputControl(joint5Destination);
  }
  else if(jointUpdated == 6)
  {
    joint6.runOutputControl(joint6Destination);
  }

  jointUpdated += 1;
  if(jointUpdated > 6)
  {
    jointUpdated = 1;

  }
}

void sysStatusUpdater()
{
  sysStatus.update();
}
