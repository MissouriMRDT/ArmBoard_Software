#include <RoveBoard.h>
#include <RoveComm.h>
#include <stdint.h>
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
  ArmEnableJ34 = 0x334,
  ArmEnableJ5 = 0x335,
  ArmEnableEndeff = 0x336,
  ArmAbsoluteAngle = 0x310,
  MoveGripper = 0x360, 
  TurnCap = 0x325,  //Incorrect Command ID
  UseOpenLoop = 0x325, //Incorrect Command ID
  UseClosedLoop = 0x326 //Incorrect Command ID
  
} ArmCommandIds;

typedef enum ArmCommandIds_LastYear
{
  LY_ArmStop = 206,
  LY_ArmJ1 = 205,
  LY_ArmJ2 = 207,
  LY_ArmJ3 = 204,
  LY_ArmJ4 = 203,
  LY_ArmJ5 = 202,
  LY_MoveGripper = 202
  
} ArmCommandIds_LastYear;


typedef enum EndefCommandIdsToRed
{
  GripperOvercurrent
}EndefCommandIdsToRed;

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
const uint8_t MAX_PACKET_SIZE = 6;

const uint32_t MOT1_PWN_PIN = PG_1;
const uint32_t MOT2_PWN_PIN = PF_3;
const uint32_t MOT3_PWN_PIN = PK_5;
const uint32_t MOT4_PWN_PIN = PK_4;
const uint32_t MOT5_PWN_PIN = PG_0;
const uint32_t MOT6_PWM_PIN = PF_2;

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

const uint32_t HBRIDGE6_NFAULT_PIN = PQ_3;
const uint32_t HBRIDGE6_NENABLE_PIN = PP_3;
const uint32_t HBRIDGE6_PHASE_PIN = PQ_2;

const uint32_t ENCODER1_READING_PIN = PM_4;
const uint32_t ENCODER2_READING_PIN = PD_2;
const uint32_t ENCODER3_READING_PIN = PM_6;
const uint32_t ENCODER4_READING_PIN = PM_2;
const uint32_t ENCODER5_READING_PIN = PM_0;

const uint32_t GRIPPER_SERVO_PWM_PIN = PF_2;

const uint32_t OC_NFAULT_PIN = PE_5;
const uint32_t CURRENT_READ_PIN = PD_3;

const uint32_t POWER_LINE_CONTROL_PIN = PE_4;

const float CURRENT_SENSOR_RATIO = .066; //current sensor ratio of outputted signal voltage/the current it's currently reading

const float CURRENT_LIMIT = 18; //actual limit we want is 17, but because the calculations are just an estimate we overshoot it slightly for manual checks

const float VCC = 3.3; //usually the V input is 3.3V

const float PI_TIMESLICE_SECONDS = .04;

void initialize();

bool checkOvercurrent();

CommandResult masterPowerSet(bool enable);

void allMotorsPowerSet(bool enable);

void j12PowerSet(bool powerOn);

void j3PowerSet(bool powerOn);

void j45PowerSet(bool powerOn);

void gripperPowerSet(bool powerOn);

float readMasterCurrent();

CommandResult stopArm();

CommandResult moveJ1(int16_t moveValue);

CommandResult moveJ2(int16_t moveValue);

CommandResult moveJ3(int16_t moveValue);

CommandResult moveJ4(int16_t moveValue);

CommandResult moveJ5(int16_t moveValue);

CommandResult moveGripper(int16_t moveValue);

CommandResult turnCap(int16_t moveValue);

CommandResult setArmAngles(float* angles);

CommandResult switchToOpenLoop();

CommandResult switchToClosedLoop();

void setupTimer0(float timeout_micros);

void closedLoopUpdateHandler();

