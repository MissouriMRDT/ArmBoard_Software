#include <stdint.h>
#include <Servo.h>

typedef enum GripperCommandIds
{
  MoveGripper = 1,
  SpinCap = 2,
  PowerEnable = 3,
  PowerDisable = 4
} GripperCommandIds;

typedef enum GripperFlagMessages
{
  GripperOvercurrent = 5 //flag to send base station if the gripper experiences an overcurrent
} GripperFlagMessages;

typedef enum EndefectorCommandIds
{
  Gripper,
  GripperCurrent
} EndefectorCommandIds;

typedef enum CommandResult
{
  Success,
  Failure
} CommandResult;

const uint32_t WATCHDOG_TIMEOUT_US = 2000000; //the amount of microseconds that should pass without getting a transmission from base station before the arm ceases moving for safety
const uint8_t SERIAL_INPUT_PIN = 0; //the arduino pin used when taking serial input to armboard
const uint8_t SERIAL_OUTPUT_PIN = 1; //the arduino pin used when taking serial output to armboard
const uint8_t DRIVER_DIRECTION_PIN = 2; //the arduino pin used for motor driver direction control
const uint8_t POWER_LINE_CONTROL_PIN = 3; //the arduino pin used for turning the power line on/off
const uint8_t NFAULT_ALERT_PIN = 4; //the arduino pin used when logic low on OC fault line
const uint8_t DRIVER_MAGNITUDE_PIN = 5; //the arduino pin used for motor driver magnitude control
const uint8_t SERVO_CONTROL_PIN = 6; //the arduino pin used for servo control
const uint8_t SERVO_ZERO_VALUE = 90; //the zero speed value used when controlling a continuous servo
const uint32_t SERIAL_BAUD_RATE = 115200; //the baud rate for serial comm between arm and gripper board
