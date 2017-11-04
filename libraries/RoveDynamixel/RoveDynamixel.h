// RoveDynamixel.h
// Author: Gbenga Osibodu

#ifndef DYNAMIXEL_H
#define DYNAMIXEL_H

#include "../RoveBoard/RoveBoard.h"

// DYNAMIXEL EEPROM AREA
#define DYNAMIXEL_MODEL_NUMBER_L           0
#define DYNAMIXEL_MODEL_NUMBER_H           1
#define DYNAMIXEL_VERSION                  2
#define DYNAMIXEL_ID                       3
#define DYNAMIXEL_BAUD_RATE                4
#define DYNAMIXEL_RETURN_DELAY_TIME        5
#define DYNAMIXEL_CW_ANGLE_LIMIT_L         6
#define DYNAMIXEL_CW_ANGLE_LIMIT_H         7
#define DYNAMIXEL_CCW_ANGLE_LIMIT_L        8
#define DYNAMIXEL_CCW_ANGLE_LIMIT_H        9
#define DYNAMIXEL_LIMIT_TEMPERATURE        11
#define DYNAMIXEL_DOWN_LIMIT_VOLTAGE       12
#define DYNAMIXEL_UP_LIMIT_VOLTAGE         13
#define DYNAMIXEL_MAX_TORQUE_L             14
#define DYNAMIXEL_MAX_TORQUE_H             15
#define DYNAMIXEL_RETURN_LEVEL             16
#define DYNAMIXEL_ALARM_LED                17
#define DYNAMIXEL_ALARM_SHUTDOWN           18

// MX SERIES EEPROM
#define MX_MULTI_TURN_OFFSET_L             20
#define MX_MULTI_TURN_OFFSET_H             21
#define MX_RESOLUTION_DIVIDER              22

// DYNAMIXEL RAM AREA
#define DYNAMIXEL_TORQUE_ENABLE            24
#define DYNAMIXEL_LED                      25
#define DYNAMIXEL_GOAL_POSITION_L          30
#define DYNAMIXEL_GOAL_POSITION_H          31
#define DYNAMIXEL_MOVING_SPEED_L           32
#define DYNAMIXEL_MOVING_SPEED_H           33
#define DYNAMIXEL_TORQUE_LIMIT_L           34
#define DYNAMIXEL_TORQUE_LIMIT_H           35
#define DYNAMIXEL_PRESENT_POSITION_L       36
#define DYNAMIXEL_PRESENT_POSITION_H       37
#define DYNAMIXEL_PRESENT_SPEED_L          38
#define DYNAMIXEL_PRESENT_SPEED_H          39
#define DYNAMIXEL_PRESENT_LOAD_L           40
#define DYNAMIXEL_PRESENT_LOAD_H           41
#define DYNAMIXEL_PRESENT_VOLTAGE          42
#define DYNAMIXEL_PRESENT_TEMPERATURE      43
#define DYNAMIXEL_REGISTERED_INSTRUCTION   44
#define DYNAMIXEL_MOVING                   46
#define DYNAMIXEL_LOCK                     47
#define DYNAMIXEL_PUNCH_L                  48
#define DYNAMIXEL_PUNCH_H                  49

// AX SERIES RAM
#define AX_CW_COMPLIANCE_MARGIN            26
#define AX_CCW_COMPLIANCE_MARGIN           27
#define AX_CW_COMPLIANCE_SLOPE             28
#define AX_CCW_COMPLIANCE_SLOPE            29

// MX SERIES RAM
#define MX_D_GAIN                          26
#define MX_I_GAIN                          27
#define MX_P_GAIN                          28
#define MX_GOAL_ACCELERATION               73

// Instructions
#define DYNAMIXEL_PING                     1
#define DYNAMIXEL_READ_DATA                2
#define DYNAMIXEL_WRITE_DATA               3
#define DYNAMIXEL_REG_WRITE                4
#define DYNAMIXEL_ACTION                   5
#define DYNAMIXEL_RESET                    6

#define MX_HIGH_BYTE_MASK                  0x0F
#define AX_HIGH_BYTE_MASK                  0x03

#define TXDELAY 2000

typedef enum {
  AX,
  MX
} DynamixelType;

typedef enum {
  Wheel = 0,
  Joint = 1,
  MultiTurn = 2
} DynamixelMode;

typedef struct {
  uint8_t id;
  DynamixelType type;
  roveUART_Handle uart;
} Dynamixel;

typedef enum {
  DYNAMIXEL_ERROR_SUCCESS = 0,
  DYNAMIXEL_ERROR_VOLTAGE = 1,
  DYNAMIXEL_ERROR_ANGLE_LIMIT = 2,
  DYNAMIXEL_ERROR_OVERHEATING = 4,
  DYNAMIXEL_ERROR_RANGE = 8,
  DYNAMIXEL_ERROR_CHECKSUM = 16,
  DYNAMIXEL_ERROR_OVERLOAD = 32,
  DYNAMIXEL_ERROR_UNKNOWN = 64
} Dynamixel_Error;

void DynamixelInit(Dynamixel* dyna, DynamixelType type, uint8_t id, uint8_t uartIndex, int baud);

void DynamixelSendPacket(Dynamixel dyna, uint8_t length, uint8_t* instruction);
uint8_t DynamixelGetReturnPacket(Dynamixel dyna, uint8_t* buffer, size_t bufferSize);
uint8_t DynamixelGetError(Dynamixel dyna);

uint8_t DynamixelPing(Dynamixel dyna);
void DynamixelSendWriteCommand(Dynamixel dyna, uint8_t dynamixelRegister, uint8_t dataLength, uint8_t* data);
void DynamixelSendReadCommand(Dynamixel dyna, uint8_t dynamixelRegister, uint8_t readLength);

uint8_t DynamixelRotateJoint(Dynamixel dyna, uint16_t position);
uint8_t DynamixelSpinWheel(Dynamixel dyna, uint16_t speed);

uint8_t DynamixelSetId(Dynamixel* dyna, uint8_t id);
uint8_t DynamixelSetBaudRate(Dynamixel dyna, uint8_t baudByte);
uint8_t DynamixelSetReturnDelayTime(Dynamixel dyna, uint8_t returnDelayByte);
uint8_t DynamixelSetMaxTorque(Dynamixel dyna, uint16_t maxTorque);
uint8_t DynamixelSetStatusReturnLevel(Dynamixel dyna, uint8_t level);
uint8_t DynamixelSetMode(Dynamixel dyna, DynamixelMode mode);

uint8_t DynamixelGetMode(Dynamixel dyna, DynamixelMode* mode);
uint8_t DynamixelGetPresentPosition(Dynamixel dyna, uint16_t* pos);
uint8_t DynamixelGetPresentSpeed(Dynamixel dyna, uint16_t* speed);
uint8_t DynamixelGetLoad(Dynamixel dyna, uint16_t* load);
uint8_t DynamixelGetVoltage(Dynamixel dyna, uint8_t* voltage);
uint8_t DynamixelGetTemperature(Dynamixel dyna, uint8_t* temp);

#endif
