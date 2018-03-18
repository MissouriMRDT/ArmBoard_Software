/*
 * main.h
 *
 *  Created on: Sep 3, 2017
 *      Author: drue
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "RoveBoard_TivaTM4C1294NCPDT.h"
#include "RoveComm.h"

#include "GenPwmPhaseHBridge.h"
#include "Ma3Encoder12b.h"
#include "PIAlgorithm.h"
#include "RCContinuousServo.h"
#include "RoveJointControl.h"
#include "VelocityDeriver.h"
#include "PIVConverter.h"
#include "GravityInertiaSystemStatus.h"
#include "GravityCompensator.h"
#include "VNH5019.h"
#include "VNH5019WithPCA9685.h"

#include "tm4c1294ncpdt_API/tivaware/inc/hw_ints.h"
#include "tm4c1294ncpdt_API/tivaware/driverlib/interrupt.h"
#include "tm4c1294ncpdt_API/tivaware/driverlib/sysctl.h"
#include "tm4c1294ncpdt_API/tivaware/inc/hw_nvic.h"
#include "tm4c1294ncpdt_API/tivaware/inc/hw_types.h"
#include "tm4c1294ncpdt_API/tivaware/driverlib/watchdog.h"

//enum representing the different arm commands we can receive from base station.
//There is a spreadsheet for these under rovesodrive under software architecture
typedef enum ArmCommandIds
{
  ArmStop = 0x320,
  ArmJ1 = 0x321,
  ArmJ2 = 0x322,
  ArmJ3 = 0x323,
  ArmJ4 = 0x324,
  ArmJ5 = 0x325,
  ArmJ6 = 0x325,
  ArmEnableAll = 0x330,
  ArmEnableMain = 0x331,
  ArmEnableJ1 = 0x332,
  ArmEnableJ2 = 0x333,
  ArmEnableJ3 = 0x334,
  ArmEnableJ4 = 0x335,
  ArmEnableJ5 = 0x336,
  ArmEnableJ6 = 0x337,
  ArmEnableEndeff = 0x338,
  ArmEnableServo = 0x339,
  ArmAbsoluteAngle = 0x310,
  ArmAbsoluteXYZ = 0x311,
  MoveGripper = 0x360,
  ArmGetPosition = 0x319,
  MoveGripServo = 0x364,
  ArmCurrentMain = 0x370,
  DisableLimits = 896,
  EnableLimits = 897
} ArmCommandIds;

//enum representing the different arm commands we can send to base station
typedef enum ArmTelemetryIds
{
  ArmCurrentPosition = 0x318,
  ArmFault = 0x340 //armFault is paired with a specific fault message payload to indicate the specific nature of the fault
} ArmTelemetryIds;

//enum representing the different payloads that couple with arm telemetry ID's that are sent back to base station
typedef enum ArmTelemetryPayloadIds
{
  ArmFault_m1 = 1,
  ArmFault_m2 = 2,
  ArmFault_m3 = 3,
  ArmFault_m4 = 4,
  ArmFault_m5 = 5,
  ArmFault_gripper = 8,
  ArmFault_overcurrent = 16
}ArmTelemetryPayloadIds;

//enum representing arm commands that are outdated, but kept around in case the user is using
//an outdated version of base station
typedef enum ArmCommandIds_LastYear
{
  LY_ArmStop = 206,
  LY_ArmJ1 = 205,
  LY_ArmJ2 = 207,
  LY_ArmJ3 = 204,
  LY_ArmJ4 = 203,
  LY_ArmJ5 = 202,
  LY_MoveGripper = 208
} ArmCommandIds_LastYear;

//enum representing the differnet results we can return when we try to move a component
typedef enum CommandResult
{
  Success,
  Fail
} CommandResult;

//enum representing the control systems the arm is currently using
typedef enum ControlSystems
{
  OpenLoop,
  ClosedLoop
} ControlSystems;

const uint32_t WATCHDOG_TIMEOUT_US = 1000000; //the amount of microseconds that should pass without getting a transmission from base station before the arm ceases moving for safety
const uint8_t IP_ADDRESS [4] = {192, 168, 1, 131};
const uint8_t ArmJointCount = 6;
const uint8_t IKArgCount = 6;

const int BaseMaxSpeed = 1000;

const int ElbowKpp = 10;
const int ElbowKip = 1;
const int ElbowKpv = 10;
const int ElbowKiv = 1;

const int ElbowTiltKp = 60;
const int ElbowTiltKi = 20;
const float ElbowTiltDeadband = 1;
const int ElbowTiltOffsetAngle = -35;
const int ElbowTiltHardStopUp = 180;
const int ElbowTiltHardStopDown = 0;
const float ScalarJ3 = .4;

const int ElbowRotateKp = 20;
const int ElbowRotateKi = 0;
const float ElbowRotateDeadband = 1;
const int ElbowRotateOffsetAngle = -36;
const int ElbowRotateHardStopUp = 355;
const int ElbowRotateHardStopDown = 180;

const int BaseTiltKp = 120;//175;
const int BaseTiltKi = 20; //100 - 15;
const int BaseTiltDeadband = 1;//1.5;
const int BaseTiltOffsetAngle = -263;
const int BaseTiltHardStopUp = 40;
const int BaseTiltHardStopDown = 260;

const int BaseRotateKp = 50;
const int BaseRotateKi = 10;
const int BaseRotateDeadband = 1;
const int BaseRotateOffsetAngle = -252;
const int BaseRotateHardStopUp = 270;
const int BaseRotateHardStopDown = 90;

const int WristRotateKp = 80;
const int WristRotateKi = 0;
const float WristRotateDeadband = 1;
const int WristRotateOffsetAngle = -230;

const int WristTiltKp = 45;
const int WristTiltKi = 4;
const float WristTiltDeadband = 1;
const int WristTiltOffsetAngle = -113;
const int WristTiltHardStopUp = 300;
const int WristTiltHardStopDown = 350;

//hardware pin assignments
const uint32_t HBRIDGE1_INA = PQ_2;
const uint32_t HBRIDGE1_INB = PQ_3;
const uint32_t HBRIDGE2_INA = PK_4;
const uint32_t HBRIDGE2_INB = PK_5;
const uint32_t HBRIDGE3_INA = PC_4;
const uint32_t HBRIDGE3_INB = PC_5;
const uint32_t HBRIDGE4_INA = PC_6;
const uint32_t HBRIDGE4_INB = PE_5;
const uint32_t HBRIDGE5_INA = PE_3;
const uint32_t HBRIDGE5_INB = PM_5;
const uint32_t HBRIDGE6_INA = PF_1;
const uint32_t HBRIDGE6_INB = PF_2;
const uint32_t HBRIDGE7_INA = PH_2;
const uint32_t HBRIDGE7_INB = PH_3;
const uint32_t HBRIDGE8_INA = PD_1;
const uint32_t HBRIDGE8_INB = PD_0;

const uint32_t PWM_DRIVER_SDA = PN_4;
const uint32_t PWM_DRIVER_SCL = PN_5;

const uint32_t SELECT_BTN_INPUT = PM_2;
const uint32_t FORWARD_BTN_INPUT = PM_1;
const uint32_t REVERSE_BTN_INPUT = PM_0;
const uint32_t MANUAL_CONTROL_SWITCH_INPUT = PH_0;

const uint32_t LED1_OUTPUT = PB_4;
const uint32_t LED2_OUTPUT = PB_5;
const uint32_t LED3_OUTPUT = PD_4;
const uint32_t LED4_OUTPUT = PD_5;
const uint32_t LED5_OUTPUT = PQ_0;
const uint32_t LED6_OUTPUT = PP_4;

const uint32_t MOTOR1_CURRENT_INPUT = PK_2;
const uint32_t MOTOR2_CURRENT_INPUT = PK_1;
const uint32_t MOTOR3_CURRENT_INPUT = PK_0;
const uint32_t MOTOR4_CURRENT_INPUT = PE_2;
const uint32_t MOTOR5_CURRENT_INPUT = PE_1;
const uint32_t MOTOR6_CURRENT_INPUT = PE_0;
const uint32_t MASTER_CURRENT_INPUT = PK_3;

const uint32_t ENCODER1_READING_PIN = PL_4;
const uint32_t ENCODER2_READING_PIN = PD_2;
const uint32_t ENCODER3_READING_PIN = PA_4;
const uint32_t ENCODER4_READING_PIN = PA_6;
const uint32_t ENCODER5_READING_PIN = PM_4;
const uint32_t ENCODER6_READING_PIN = PB_2;

const uint32_t POWER_LINE_CONTROL_PIN = PQ_1;

const uint32_t BASE_HIGH_LIMIT_PIN = PB_3;
const uint32_t BASE_LOW_LIMIT_PIN = PC_7;
const uint32_t ELBOW_HIGH_LIMIT_PIN = PD_3;
const uint32_t ELBOW_LOW_LIMIT_PIN = PL_1;
const uint32_t WRIST_HIGH_LIMIT_PIN = PL_2;
const uint32_t WRIST_LOW_LIMIT_PIN = PL_3;

const float PI_TIMESLICE_SECONDS = .04;
const float PIV_TIMESLICE_SECONDS = .004;

const float J12Kt = (0.014 * 672) * 2; //newton-meters per amp
const float J3Kt = 0.014 * 672;
const float J45Kt = (0.353 * 131) * 2;
const int MotorVoltage = 12000;
const int J1Resistance = 800;
const int J2Resistance = 800;
const int J3Resistance = 800;
const int J4Resistance = 2400;
const int J5Resistance = 2400;

const uint8_t PcaChipAddress = 0b01000000;
const uint8_t PcaI2cModule = I2C_Module2;

//D-H Parameters of Arm Model
const float th1offset=1.57079632679; //should be 90 in order for origin frame to comply with "Rover
// Coordinate Standard"
const float d1=2.8937; // height of bicep tilt axis from baseplate/origin
const float a1=0; //forward offset of bicep tilt axis relative to base rotate axis
const float alpha1=1.57079632679; //anglular offset of J1 about X1 axis. (SHOULD BE 90 UNLESS ARM
           // DESIGN IS SUPER FUNKY)
const float th2offset=1.57079632679;//should be 90 in order to comply with DH convention
const float d2=0;//offset to the right of the bicep relative to the base rotation axis(
     //should probably stay as 0 even if bicep is offset. this offset can
     //also be accounted for using d3)
const float a2=17;//bicep length(distance between bicep tilt axis and elbow tilt axis)
const float alpha2=0;//angular offset of elbow tilt axis about x2 axis.(SHOULD BE 90
         //UNLESS ARM DESIGN IS SUPER FUNKY)
const float th3offset=1.57079632679;//should be 90
const float d3=0;//offset to the right of the forearm relative to the bicep(see d2
     //comment, if the bicep is offset from the base rotate axis but you
     //have d2 as 0, then d3 must be the offset to the right of the forearm
     //relative to the base rotate axis)
const float a3=2.837;//offset of forearm twist axis from the elbow tilt axis along the x2
     //axis. (this is the "vertical" offset of the forearm.  DONT USE THIS
     //if you calculated the actual distance between the elbow axis and
     //wrist center and calculated the th3 offset accordingly. in that case
     //a3 should be 0
const float alpha3=1.57079632679;//angular offset of forearm about x3 axis. (SHOULD BE 90 UNLESS ARM
         //DESIGN IS SUPER FUNKY)
const float th4offset=0; //angular offset of forearm twist. should be 0 for standard
             //spherical wrist orientation. (phoenix, horison, zenith, and
             //gryphon's wrist joints all complied with this)
const float d4=17;//Forearm Length. If a3 is zero but there is a "vertical" offset of
      //the forearm, this value needs to be the center to center distance
      //between the elbow tilt axis and the wrist center.
const float a4=0; //needs to be 0 for spherical wrist
const float alpha4=-1.57079632679; //should be -90 for standard spherical wrist orientation.
            //(phoenix, horiZon, zenith, and gryphon's wrist joints all
            //complied with this)
const float th5offset=0; //wrist tilt angle offset. should be 0 unless there is a
             //"vertical" forearm offset and you chose to use the center to
             //center distances between the elbow tilt axis and the wrist
             //center. if this is the case, th4offset needs to be calculated
             //as the angle between the line center line between the elbow
             //tilt axis and wrist center with the axis of gripper rotate(j6)
const float d5=0;//needs to be 0 for spherical wrist
const float a5=0;//needs to be 0 for spherical wrist
const float alpha5=1.57079632679;//angular offset of gripper rotate axis from gripper tilt axis
          //about x5 axis. needs to be 90 for spherical wrist
const float th6offset=1.57079632679; //angular twist of gripper from normal orientation. should be
              //90 for standard spherical wrist orientation. (phoenix,
              //horiZon, zenith, and gryphon's wrist joints all complied with this)
const float d6=0;//keep as 0
const float a6=0;//keep as 0
const float alpha6=1.57079632679; //angular tilt of gripper from normal orientation. should be 90
           //for standard spherical wrist orientation. (phoenix, horiZon,
           //zenith, and gryphon's wrist joints all complied with this)

//CENTER POINT OF GRIPPER
const double OpPointoffset[3]={0, 5.25, 0};


CommandResult masterPowerSet(bool enable);
void allMotorsPowerSet(bool enable);
void j1PowerSet(bool powerOn);
void j2PowerSet(bool powerOn);
void j3PowerSet(bool powerOn);
void j4PowerSet(bool powerOn);
void j56PowerSet(bool powerOn);
void pokerPowerSet(bool powerOn);
void gripperPowerSet(bool powerOn);
bool checkLimSwitch(uint32_t switchPin);

CommandResult stopArm();
CommandResult moveJ1(int16_t moveValue);
CommandResult moveJ2(int16_t moveValue);
CommandResult moveJ3(int16_t moveValue);
CommandResult moveJ4(int16_t moveValue);
CommandResult moveJ5(int16_t moveValue);
CommandResult moveJ6(int16_t moveValue);
CommandResult moveGripper(int16_t moveValue);

CommandResult setArmDestinationAngles(float* angles);
CommandResult getArmPositions(float positions[ArmJointCount]);
void Calc_IK(float coordinates[IKArgCount], float angles[ArmJointCount]);
float negativeDegreeCorrection(float correctThis);

CommandResult switchToOpenLoop();
CommandResult switchToClosedLoop();

void setupTimer7(float timeout_micros);
void closedLoopUpdateHandler();
void sysStatusUpdater();
void initWatchdog(uint32_t timeout_us);
void restartWatchdog(uint32_t timeout_us);

#endif /* MAIN_H_ */

