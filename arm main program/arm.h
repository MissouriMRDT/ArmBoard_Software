#include <RoveBoard.h>
#include <RoveComm.h>
#include <stdint.h>

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
  ArmJ6 = 0x326
} ArmCommandIds;

typedef enum ArmCommandIds_LastYear
{
  LY_ArmStop = 206,
  LY_ArmJ1 = 205,
  LY_ArmJ2 = 207,
  LY_ArmJ3 = 204,
  LY_ArmJ4 = 203,
  LY_ArmJ5 = 202,
  LY_ArmJ6 = 201
} ArmCommandIds_LastYear;

//enum representing the different endefector commands we can receive from base station.
//There is a spreadsheet for these under rovesodrive under software architecture 
typedef enum EndefCommandIds
{
  Gripper,
  Drill 
} EndefCommandIds; 

//enum representing the differnet results we can return when we try to move a component
typedef enum CommandResult
{
  Success,
  Fail
} CommandResult;

const uint32_t WATCHDOG_TIMEOUT_US = 2000000; //the amount of microseconds that should pass without getting a transmission from base station before the arm ceases moving for safety
const uint8_t IP_ADDRESS [4] = {192, 168, 1, 131};
