#ifndef DYNAMIXELCONTROLLER_H_
#define DYNAMIXELCONTROLLER_H_

//module unimplemented for now due to bugs, and for RoveDynamixel being untested on roveboard

/*#include <RoveDynamixel.h>
#include <stdint.h>
#include "AbstractFramework.h"

class DynamixelController : public OutputDevice
{
  private:
    //Only Important pin for dynamixel is the Tx/Rx pin other two are just power and not important logically
    int Tx_PIN;
    int Rx_PIN;
    uint32_t baudRate;
    int currentPower = 0;

    //values which the dynamixel expects the speed to be between. CW = clockwise/positive/forward  CCW = counter-clockwise/negative/backwards
    const int DYNA_SPEED_CW_MAX = 1023;
    const int DYNA_SPEED_CW_MIN = 0;
    const int DYNA_SPEED_CCW_MAX = 2047;
    const int DYNA_SPEED_CCW_MIN = 1024;

    //need to have in order to interface with RoveDynamixel since it uses structs and pointers as opposed to classes
    Dynamixel dynamixel;

  protected:
    //movement command based on a speed input and wheel mode. Input should be in the range between SPEED_MIN and SPEED_MAX
    //this will be the defaut assumption, other modes with other methods of movement will be made into other functions not simply move.
    //clockwise will be considered forward and ccw is reverse
    void move(const long movement);
    
    //tells the device to power on or off.
    void setPower(bool powerOn);
    
    //tells device to stop moving
    void stop();
    
  public:

    //constructor for a dynamixel which takes in a move type and other things needed for dynamixels
	//Calls the init function from RoveDynamixel.h to initialize the dynamixel
    //Sets up uart on the board and baud rate at the same time
    //Inputs: TX -> tx pin numerical id, as defined by energia's pinmapping
              //RX -> tx pin numerical id, as defined by energia's pinmapping
              //upsideDown -> Whether or not the dyna is mounted in reverse and thus the inputs need to be reversed as well
              //type -> Instance of the DynamixelType enum that defines the dynamixel brand such as AX, MX, etc
              //id -> id of the dynamixel. Note that if you aren't sure, you'll need to use a different program to set the id yourself
              //uartIndex -> which hardware uart to use when talking to it, 0-7
              //baud -> baud rate of the serial communication
              //mode -> instance of the DynamixelMode enum that defines the dynamixel's mode such as Wheel, Joint, etc

    DynamixelController(const int Tx, const int Rx, bool upsideDown, DynamixelType type, uint8_t id, uint8_t uartIndex, uint32_t baud, DynamixelMode mode);
    
    //gets how quickly the dynamixel is moving, in terms of power, or what positino it's moving to, if set up in position mode
    //returns: value between POWER_MIN and POWER_MAX or value between POS_MIN and POS_MAX
    long getCurrentMove();

};*/

#endif
