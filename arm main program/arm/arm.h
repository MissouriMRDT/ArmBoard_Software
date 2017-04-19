#include <RoveBoard.h>
#include <RoveComm.h>
#include <stdint.h>
#include "JointControlFramework.h"
#include "GenPwmPhaseHBridge.h"
#include "PIAlgorithm.h"
#include "Ma3Encoder12b.h"

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
  MoveGripper = 0x325, //Incorrect Command ID
  TurnCap = 0x325  //Incorrect Command ID
  
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

const uint32_t WATCHDOG_TIMEOUT_US = 2000000; //the amount of microseconds that should pass without getting a transmission from base station before the arm ceases moving for safety
const uint8_t IP_ADDRESS [4] = {192, 168, 1, 131};
const uint32_t GRIPPER_COMM_BAUD_RATE = 115200;

const uint32_t MOT1_PWN_PIN = PG_1;
const uint32_t MOT2_PWN_PIN = PF_3;
const uint32_t MOT3_PWN_PIN = PK_5;
const uint32_t MOT4_PWN_PIN = PK_4;
const uint32_t MOT5_PWN_PIN = PG_0;
const uint32_t MOT6_PWM_PIN = PF_2;

const uint32_t GRIPP_TX6_PIN = PP_0;
const uint32_t GRIPP_RX6_PIN = PP_1;

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
const uint32_t HBRIDGE6_NSLEEP_PIN = PP_3;
const uint32_t HBRIDGE6_PHASE_PIN = PQ_2;

const uint32_t ENCODER1_READING_PIN = PM_4;
const uint32_t ENCODER2_READING_PIN = PA_6;
const uint32_t ENCODER3_READING_PIN = PM_6;
const uint32_t ENCODER4_READING_PIN = PM_2;
const uint32_t ENCODER5_READING_PIN = PM_0;

const uint32_t OC_NFAULT_PIN = PE_5;
const uint32_t CURRENT_READ_PIN = PD_3;

const uint32_t POWER_LINE_CONTROL_PIN = PE_4;

const float CURRENT_SENSOR_RATIO = .066; //current sensor ratio of outputted signal voltage/the current it's currently reading

const float CURRENT_LIMIT = 18; //actual limit we want is 17, but because the calculations are just an estimate we overshoot it slightly for manual checks

const float VCC = 3.0; //usually the V input is 3V

void initialize();

bool checkOvercurrent();

CommandResult masterPowerEnable();

CommandResult masterPowerDisable();

void enableAllMotors();

void disableAllMotors();

float readMasterCurrent();

CommandResult stopArm();

CommandResult moveJ1(int16_t moveValue);

CommandResult moveJ2(int16_t moveValue);

CommandResult moveJ3(int16_t moveValue);

CommandResult moveJ4(int16_t moveValue);

CommandResult moveJ5(int16_t moveValue);

CommandResult moveGripper(int16_t moveValue);

CommandResult turnCap(int16_t moveValue);
