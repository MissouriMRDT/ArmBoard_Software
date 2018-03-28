/*
 * VNH5019WithPCA9685.cpp
 *
 *  Created on: Mar 16, 2018
 *      Author: drue
 */

#include "VNH5019WithPCA9685.h"
#include "RoveJointUtilities.h"

static bool initialized = false;
static const uint8_t RESET_ADDRESS = 0b00000000;
static const uint8_t RESET_MSG =     0b000001100;
static const uint8_t M1REGISTER_ADDRESS = 0b00000000;
static const uint8_t M1REGISTER_MSG =     0b00100000; //this will turn off sleep mode, turn on auto increment
static const uint16_t PWM_MAX = 4095;
static const uint8_t MAX_MOTOR_CHANNELS = 7;

VNH5019WithPCA9685::VNH5019WithPCA9685(uint8_t chipAdd, uint8_t motorInd, uint8_t motorInaPin, uint8_t motorInbPin, uint8_t i2cModuleIndex, uint8_t clockPin, uint8_t dataPin, bool inverted)
  : OutputDevice(InputPowerPercent, inverted), ChipAddress(chipAdd), MotorIndex(motorInd), InaPin(motorInaPin), InbPin(motorInbPin),
    I2cModule(i2cModuleIndex), DataPin(dataPin), ClockPin(clockPin)
{
  i2cHandle = i2cInit(i2cModuleIndex, I2CSPEED_FAST, clockPin, dataPin);

  if(motorInd > MAX_MOTOR_CHANNELS)
  {
    debugFault("Error in BTM7752GwithPCA9685 constructor: motor index out of bounds");
  }

  if(!initialized)
  {
    roveI2cSend(i2cHandle, RESET_ADDRESS, RESET_MSG);
    delay(1);
    roveI2cSendReg(i2cHandle, ChipAddress, M1REGISTER_ADDRESS, M1REGISTER_MSG);
    delay(1);
    digitalPinWrite(InaPin, LOW);
    digitalPinWrite(InbPin, LOW);
    initialized = true;
  }
}

void VNH5019WithPCA9685::move(const long movement)
{
  if(enabled && movement != 0)
  {
    currentMove = movement;
    long mov = movement;

    long offRegNum; //the off register determines when the high pulse turns off in pwm
    long onRegNum; //on register is when high pulse turns on. Usually make it 0 so it just starts on, then turns off when offRegNum says to

    uint8_t channelRegister;

    if(invert)
    {
      mov *= -1;
    }

    if(abs(mov) == POWERPERCENT_MAX) //special value used for 100%
    {
      onRegNum = 4096;
      offRegNum = 0;
    }
    else
    {
      offRegNum = (abs(mov) * PWM_MAX) / POWERPERCENT_MAX;
      onRegNum = 0;
    }

    channelRegister = 6 + 4 * MotorIndex;

    if(mov > 0)
    {
      digitalPinWrite(InaPin, HIGH);
      digitalPinWrite(InbPin, LOW);
    }
    else if(mov < 0)
    {
      digitalPinWrite(InaPin, LOW);
      digitalPinWrite(InbPin, HIGH);
    }

    RoveI2C_Error error;
    uint8_t msg[4] = {(uint8_t)onRegNum, (uint8_t)(onRegNum>>8), (uint8_t)offRegNum, ((uint8_t)(offRegNum>>8))};
    error = roveI2cSendBurstReg(i2cHandle, ChipAddress, channelRegister, msg, 4);
    uint8_t count = 0;
    while(error != I2CERROR_NONE)
    {
      if(count > 3)
      {
        break;
      }

      //reset i2c if an error occurred
      digitalPinWrite(ClockPin, HIGH);
      digitalPinWrite(ClockPin, LOW);
      digitalPinWrite(ClockPin, HIGH);
      digitalPinWrite(ClockPin, LOW);
      i2cHandle = i2cInit(I2cModule, I2CSPEED_FAST, ClockPin, DataPin);
      error = roveI2cSendBurstReg(i2cHandle, ChipAddress, channelRegister, msg, 4);
      count++;
      if(error != I2CERROR_NONE)
      {
        delayMicroseconds(5);
      }
    }
  }
  else
  {
    stop();
  }
}

void VNH5019WithPCA9685::setPower(bool power)
{
  enabled = power;
  if(power == false)
  {
    digitalPinWrite(InaPin, HIGH); //setting both pins high to brakes the motor
    digitalPinWrite(InbPin, HIGH);
  }
}

void VNH5019WithPCA9685::stop()
{
  //moving the motor at 0 speed, aka "stopping" it

  uint8_t channelRegisterBaseOffset = 6 + 4*MotorIndex;
  uint8_t msg[4] = {0, 0, 0, (uint8_t)(4096>>8)}; //special value for full off

  RoveI2C_Error error = roveI2cSendBurstReg(i2cHandle, ChipAddress, channelRegisterBaseOffset, msg, 4);
  if(error != I2CERROR_NONE)
  {
    //reset i2c if an error occurred
    i2cHandle = i2cInit(I2cModule, I2CSPEED_FAST, ClockPin, DataPin);
    roveI2cSendBurstReg(i2cHandle, ChipAddress, channelRegisterBaseOffset, msg, 4);
  }

  digitalPinWrite(InaPin, HIGH);
  digitalPinWrite(InbPin, HIGH);
  currentMove = 0;
}

long VNH5019WithPCA9685::getCurrentMove()
{
  return currentMove;
}
