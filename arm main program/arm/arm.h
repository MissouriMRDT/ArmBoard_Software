#include <RoveBoard.h>
#include <RoveComm.h>
#include <stdint.h>
#include <math.h>
#include "JointControlFramework.h"
#include "GenPwmPhaseHBridge.h"
#include "PIAlgorithm.h"
#include "Ma3Encoder12b.h"
#include "RCContinuousServo.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/sysctl.h"

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
  ArmEnableAll = 0x330,
  ArmEnableMain = 0x331,
  ArmEnableJ1 = 0x332,
  ArmEnableJ2 = 0x333,
  ArmEnableJ3 = 0x334,
  ArmEnableJ4 = 0x335,
  ArmEnableJ5 = 0x336,
  ArmEnableEndeff = 0x338,
  ArmEnableServo = 0x339,
  ArmAbsoluteAngle = 0x310,
  ArmAbsoluteXYZ = 0x311,
  MoveGripper = 0x360, 
  ArmGetPosition = 0x319,
  MoveGripServo = 0x364,
  ArmCurrentMain = 0x370
} ArmCommandIds;

//enum representing the different arm commands we can send to base station
typedef enum ArmTelemetryIds
{
  ArmCurrentPosition = 0x318,
  ArmFault = 0x340 //armFault is paired with a specific fault message payload to indicate the specific nature of the fault
} ArmTelemetryIds;

//enum representing the different payloads that couple with arm telemetry ID's that are sent back to base station
typedef enum ArmTelemetryPayloadIds: int
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

const uint32_t WATCHDOG_TIMEOUT_US = 2000000; //the amount of microseconds that should pass without getting a transmission from base station before the arm ceases moving for safety
const uint8_t IP_ADDRESS [4] = {192, 168, 1, 131};
const uint8_t ArmJointCount = 5;
const uint8_t IKArgCount = 5;
const int BaseMaxSpeed = 1000;
const int BaseRampUp = 30;
const int BaseRampDown = 60;
const int ElbowRampUp = 75;
const int ElbowRampDown = 100;


//hardware pin assignments
const uint32_t MOT1_PWN_PIN = PG_1;
const uint32_t MOT2_PWN_PIN = PF_3;
const uint32_t MOT3_PWN_PIN = PK_5;
const uint32_t MOT4_PWN_PIN = PK_4;
const uint32_t MOT5_PWN_PIN = PG_0;
const uint32_t GRIPMOT_PWM_PIN = PF_2;
const uint32_t GRIPPER_SERVO_PWM_PIN = PF_2;

const uint32_t HBRIDGE1_NFAULT_PIN = PM_7;
const uint32_t HBRIDGE1_NSLEEP_PIN = PA_7;
const uint32_t HBRIDGE1_PHASE_PIN = PP_5;

const uint32_t HBRIDGE2_NFAULT_PIN = PL_1;
const uint32_t HBRIDGE2_NSLEEP_PIN = PL_2;
const uint32_t HBRIDGE2_PHASE_PIN = PL_3;

const uint32_t HBRIDGE3_NFAULT_PIN = PH_0;
const uint32_t HBRIDGE3_NSLEEP_PIN = PH_1;
const uint32_t HBRIDGE3_PHASE_PIN = PK_6;

const uint32_t HBRIDGE4_NFAULT_PIN = PP_4;
const uint32_t HBRIDGE4_NSLEEP_PIN = PD_5;
const uint32_t HBRIDGE4_PHASE_PIN = PA_5;

const uint32_t HBRIDGE5_NFAULT_PIN = PK_2;
const uint32_t HBRIDGE5_NSLEEP_PIN = PK_3;
const uint32_t HBRIDGE5_PHASE_PIN = PQ_0;

const uint32_t GRIPMOT_NFAULT_PIN = PQ_3;
const uint32_t GRIPMOT_NENABLE_PIN = PP_3;
const uint32_t GRIPMOT_PHASE_PIN = PQ_2;

const uint32_t ENCODER1_READING_PIN = PM_4;
const uint32_t ENCODER2_READING_PIN = PD_2;
const uint32_t ENCODER3_READING_PIN = PM_6;
const uint32_t ENCODER4_READING_PIN = PM_2;
const uint32_t ENCODER5_READING_PIN = PM_0;

const uint32_t CURRENT_READ_PIN = PD_3;
const uint32_t POWER_LINE_CONTROL_PIN = PE_4;

const float CURRENT_SENSOR_RATIO = .066; //current sensor ratio of outputted signal voltage/the current it's currently reading
const float CURRENT_LIMIT = 18; //actual limit we want is 17, but because the calculations are just an estimate we overshoot it slightly for manual checks
const float VCC = 3.3; //usually the V input is 3.3V
const float PI_TIMESLICE_SECONDS = .04;

const float ElbowLength = 0;
const float BaseLength = 0;
const float WristLength = 0;

void initialize();
void motorFaultHandling();
void processBaseStationCommands();
void armOvercurrentHandling();
float readMasterCurrent();

CommandResult masterPowerSet(bool enable);
void allMotorsPowerSet(bool enable);
void j12PowerSet(bool powerOn);
void j3PowerSet(bool powerOn);
void j45PowerSet(bool powerOn);
void gripperMotorPowerSet(bool powerOn);
void gripperServoPowerSet(bool powerOn);

CommandResult stopArm();
CommandResult moveJ1(int16_t moveValue);
CommandResult moveJ2(int16_t moveValue);
CommandResult moveJ3(int16_t moveValue);
CommandResult moveJ4(int16_t moveValue);
CommandResult moveJ5(int16_t moveValue);
CommandResult moveGripper(int16_t moveValue);
CommandResult moveGripServo(int16_t moveValue);

CommandResult setArmDestinationAngles(float* angles);
CommandResult getArmPositions(float positions[ArmJointCount]);
void computeIK(float coordinates[IKArgCount], float angles[ArmJointCount]);
float negativeDegreeCorrection(float correctThis);

CommandResult switchToOpenLoop();
CommandResult switchToClosedLoop();

void setupTimer0(float timeout_micros);
void closedLoopUpdateHandler();
