/*
 * main.h
 *
 *  Created on: Sep 3, 2017
 *      Author: drue (drscp5@mst.edu), RMC by Drue, David, Timur, Eli, Chris Dutcher. Kinematics by Chris Novatny.
 *
 *
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "ArmModelInfo.h"
#include "RoveBoard_TivaTM4C1294NCPDT.h"
#include "RoveWare/RoveComm.h"
#include "Kinematics.h"
#include "RMCInstances.h"
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
  ArmJ6 = 0x326,
  MoveEndeff1 = 0x360,
  MoveEndeff2 = 0x364,
  GripperSwap = 0x365,
  ArmValues = 0x327,
  ArmEnableAll = 0x330,
  ArmEnableMain = 0x331,
  ArmEnableJ1 = 0x332,
  ArmEnableJ2 = 0x333,
  ArmEnableJ3 = 0x334,
  ArmEnableJ4 = 0x335,
  ArmEnableJ5 = 0x336,
  ArmEnableJ6 = 0x337,
  ArmEnableEndeff1 = 0x338,
  ArmEnableEndeff2 = 0x339,
  ArmAbsoluteAngle = 0x310,
  ArmAbsoluteXYZ = 0x311,
  IKRoverIncrement = 0x312,
  IKWristIncrement = 0x313,
  ArmGetPosition = 0x319,
  ArmGetXYZ = 0x328,
  ArmCurrentMain = 0x370,
  LimitSwitchOveride = 897,
  LimitSwitchUnoveride = 896,
  OpPoint = 0x366,
  ToggleAutoPositionTelem = 871
} ArmCommandIds;

//enum representing the different arm commands we can send to base station
typedef enum ArmTelemetryIds
{
  ArmCurrentPosition = 0x318,
  ArmFault = 0x340, //armFault is paired with a specific fault message payload to indicate the specific nature of the fault
  ArmCurrentXYZ = 0x31A
} ArmTelemetryIds;

//enum representing the different payloads that couple with arm telemetry ID's that are sent back to base station
typedef enum ArmTelemetryPayloadIds
{
  ArmFault_m1 = 1,
  ArmFault_m2 = 2,
  ArmFault_m3 = 3,
  ArmFault_m4 = 4,
  ArmFault_m5 = 5,
  ArmFault_m6 = 6,
  ArmFault_overcurrent = 7,
  ArmFault_encoderBaseRotate = 8,
  ArmFault_encoderBaseTilt = 9,
  ArmFault_encoderElbowTilt = 10,
  ArmFault_encoderElbowRotate = 11,
  ArmFault_encoderWristTilt = 12,
  ArmFault_encoderWristRotate = 13
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
  ClosedLoop,
  IKIncrement
} ControlSystems;

const uint32_t WATCHDOG_TIMEOUT_US = 1000000; //the amount of microseconds that should pass without getting a transmission from base station before the arm ceases moving for safety
const uint8_t IP_ADDRESS [4] = {192, 168, 1, 131};

const float MotorSensorVoltPerAmp = .066;
const float MotorSensorVoltOffset = 3.3*.5;
const float MasterSensorVoltPerAmp = .0396;
const float MasterSensorVoltOffset = 3.3/10.0;
const float MotorMaxCurrent = 35; //amps. Shut it down after that. Huge cause the sensors aren't actually that good so give it some leeway
const float MasterMaxCurrent = 50;
const int   MaxGripperPower = 500;

const int ElbowKpp = 10;
const int ElbowKip = 1;
const int ElbowKpv = 10;
const int ElbowKiv = 1;

const int ElbowTiltKp = 55;//85;
const int ElbowTiltKi = 2;//5;
const float ElbowTiltDeadband = 0.5;
const int ElbowTiltOffsetAngle = -52;
const int ElbowTiltHardStopUp = 180;
const int ElbowTiltHardStopDown = 0;
const float ScalarJ3 = .4;

const int ElbowRotateKp = 20;
const int ElbowRotateKi = 0;
const float ElbowRotateDeadband = 0.8;
const int ElbowRotateOffsetAngle = -14;
const int ElbowRotateHardStopUp = 355;
const int ElbowRotateHardStopDown = 180;

const int BaseTiltKp = 100;//100;
const int BaseTiltKi = 10; //10;
const float BaseTiltDeadband = 0.5;//1.2;
const int BaseTiltOffsetAngle = -335;
const int BaseTiltHardStopUp = 40;
const int BaseTiltHardStopDown = 260;

const int BaseRotateKp = 45;
const int BaseRotateKi = 0;
const float BaseRotateDeadband = 0.3;
const int BaseRotateOffsetAngle = -226;
const int BaseRotateHardStopUp = 270;
const int BaseRotateHardStopDown = 90;

const int WristRotateKp = 80;
const int WristRotateKi = 0;
const float WristRotateDeadband = 0.8;
const int WristRotateOffsetAngle = -199;//-203;

const int WristTiltKp = 45;
const int WristTiltKi = 0;
const float WristTiltDeadband = 0.8;
const int WristTiltOffsetAngle = -132;
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

const uint32_t BASE_ROTATE_LIMIT_PIN = PB_3;
const uint32_t BASE_HIGH_LIMIT_PIN = PC_7;
const uint32_t BASE_LOW_LIMIT_PIN = PD_3;
const uint32_t ELBOW_HIGH_LIMIT_PIN = PL_1;
const uint32_t ELBOW_LOW_LIMIT_PIN = PL_2;

const float PI_TIMESLICE_SECONDS = .04;
const float PIV_TIMESLICE_SECONDS = .004;

const uint8_t PcaChipAddress = 0b01000000;
const uint8_t PcaI2cModule = I2C_Module2;

void processBaseStationCommands();
void readArmCurrents();
void processCurrentFaults();
void sendPeriodicTelemetry();

CommandResult masterPowerSet(bool enable);
void allMotorsPowerSet(bool enable);
void baseRotatePowerSet(bool powerOn);
void baseTiltPowerSet(bool powerOn);
void elbowTiltPowerSet(bool powerOn);
void elbowRotatePowerSet(bool powerOn);
void wristPowerSet(bool powerOn);
void pokerPowerSet(bool powerOn);
void gripperPowerSet(bool powerOn);
bool checkLimSwitch(uint32_t switchPin);

CommandResult stopArm();
CommandResult moveBaseRotate(int16_t moveValue);
CommandResult moveBaseTilt(int16_t moveValue);
CommandResult moveElbowTilt(int16_t moveValue);
CommandResult moveElbowRotate(int16_t moveValue);
CommandResult moveWristTilt(int16_t moveValue);
CommandResult moveWristRotate(int16_t moveValue);
CommandResult moveGripper(int16_t moveValue);
CommandResult movePoker(int16_t moveValue);

CommandResult setArmDestinationAngles(float* angles);
CommandResult getArmPositions(float positions[ArmJointCount]);
void sendArmPositions();

CommandResult switchToOpenLoop();
CommandResult switchToClosedLoop();
CommandResult switchToIKIncrement();

void closedLoopUpdateHandler();
void sysStatusUpdater();
void initWatchdog(uint32_t timeout_us);
void restartWatchdog(uint32_t timeout_us);
void watchdogISR();
void gripperSwap();
void handleLimits(uint8_t jointNumber, bool enable);

//variables used to control joints during closed loop control
unsigned long baseRotateJointDestination;
unsigned long baseTiltJointDestination;
unsigned long elbowTiltJointDestination;
unsigned long elbowRotateJointDestination;
unsigned long wristTiltJointDestination;
unsigned long wristRotateJointDestination;

ControlSystems currentControlSystem; //tracks what control system arm is currently using

#endif /* MAIN_H_ */

