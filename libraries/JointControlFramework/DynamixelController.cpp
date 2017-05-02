#include "DynamixelController.h"
#include "Energia.h"

DynamixelController::DynamixelController(const int Tx, const int Rx, bool upsideDown, DynamixelType type, uint8_t id, uint8_t uartIndex, uint32_t baud, DynamixelMode mode) : OutputDevice()
{
  Tx_PIN = Tx;
  Rx_PIN = Rx;
  baudRate = baud;
  invert = upsideDown;
  
  if(mode == Wheel)
    inType = spd;
  else if(mode == Joint)
    inType = pos;

  DynamixelInit(&dynamixel, type, id, uartIndex, baud);
  DynamixelSetMode(dynamixel, mode);
}

void DynamixelController::move(const long movement)
{
  if(!enabled) return;

  int mov = invert ? -movement : movement;
  
  //stores the error returned by the spin wheel function
  uint8_t errorMessageIgnore;

  //if supposed to move backwards(ccw)
  if(mov < 0)
  {
    uint16_t send = map(mov, 0, SPEED_MAX, DYNA_SPEED_CCW_MAX, DYNA_SPEED_CCW_MIN);
    errorMessageIgnore = DynamixelSpinWheel(dynamixel, send);
  }

  //if forwards (cw)
  else if(mov > 0)
  {
    uint16_t send = map(mov, 0, SPEED_MAX, DYNA_SPEED_CW_MAX, DYNA_SPEED_CW_MIN);
    errorMessageIgnore = DynamixelSpinWheel(dynamixel, send);
  }

  //stop
  else if(mov == 0)
  {
    errorMessageIgnore = DynamixelSpinWheel(dynamixel, 0);
  }
}

void DynamixelController::setPower(bool powerOn)
{
  enabled = powerOn;
  
  if(!enabled)
  {
    move(0);
  }
}