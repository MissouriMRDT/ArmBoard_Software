#include "DynamixelController.h"
#include "RoveBoard.h"

//constructor for a dynamixel for any mode
//Calls the init function from RoveDynamixel.h to initialize the dynamixel
/*Inputs: TX -> tx pin numerical id, as defined by energia's pinmapping
          RX -> tx pin numerical id, as defined by energia's pinmapping
          upsideDown -> Whether or not the dyna is mounted in reverse and thus the inputs need to be reversed as well
          type -> Instance of the DynamixelType enum that defines the dynamixel brand such as AX, MX, etc
          id -> id of the dynamixel. Note that if you aren't sure, you'll need to use a different program to set the id yourself
          uartIndex -> which hardware uart to use when talking to it, 0-7
          baud -> baud rate of the serial communication
          mode -> instance of the DynamixelMode enum that defines the dynamixel's mode such as Wheel, Joint, etc
*/
/*DynamixelController::DynamixelController(const int Tx, const int Rx, bool upsideDown, DynamixelType type, uint8_t id, uint8_t uartIndex, uint32_t baud, DynamixelMode mode) : OutputDevice()
{
  //assignments
  Tx_PIN = Tx;
  Rx_PIN = Rx;
  baudRate = baud;
  invert = upsideDown;

  if(mode == Wheel)
    inType = InputPower;

  else if(mode == Joint)
    inType = InputPosition;

  //view RoveDynamixel.h for details on all functions called here
  //note: no comments in RoveDynamixel
  DynamixelInit(&dynamixel, type, id, uartIndex, baud);

  //sets dynamixel to whatever mode it's supposed to be in
  DynamixelSetMode(dynamixel, mode);
}

//sends the move command for the wheel mode based on power, an int constrained between the POWER_MIN and POWER_MAX constants.
//clockwise will be considered forward and ccw is reverse
void DynamixelController::move(const long movement)
{
  //stores the error returned by the spin wheel function
  uint8_t errorMessageIgnore;
  long mov = movement;
  uint16_t send;

  if(enabled) //if the manager disabled this device, disable output
  {
    //if mounted upside down then invert the signal passed to it and move accordingly
    if (invert)
    {
      //inverts the input easily
      mov = -mov;
    }
    
    currentPower = mov;

    //if supposed to move backwards(ccw)
    if(mov < 0)
    {
      send = map(-mov, 0, POWER_MAX, DYNA_SPEED_CCW_MAX, DYNA_SPEED_CCW_MIN);

      //calls spin wheel function from RoveDynamixel
      //can take up to a uint16_t which exceeds a standard int but
      errorMessageIgnore = DynamixelSpinWheel(dynamixel, send);
    }

    //if forwards (cw)
    else if(mov > 0)
    {
      send = map(mov, 0, POWER_MAX, DYNA_SPEED_CW_MAX, DYNA_SPEED_CW_MIN);

    //calls spin wheel function from RoveDynamixel
      //can take up to a uint16_t which exceeds a standard int but
      errorMessageIgnore = DynamixelSpinWheel(dynamixel, send);
    }

    //stop
    else if(mov == 0)
    {
      stop();
    }
  }
  return;
}

//Instructs the dynamixel class to behave as if it is off or on; IE if it's off it'll refuse to send any output
void DynamixelController::setPower(bool powerOn)
{
  if(powerOn == false)
  {
    stop(); 
  }
  
  enabled = powerOn;
}

long DynamixelController::getCurrentMove()
{
  if(invert) //if we're inverted, then we technically move negatively even if we're moving in the 'positive' direction. The direction is the important part
  {
    return(currentPower * -1); 
  }
  else
  {
    return(currentPower);
  }
}

void DynamixelController::stop()
{
  //calls spin wheel function from RoveDynamixel
  //can take up to a uint16_t which exceeds a standard int but
  DynamixelSpinWheel(dynamixel, 0);
  
  currentPower = 0;
}*/
