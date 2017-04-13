#ifndef SDC2130_H_
#define SDC2130_H_

#include "JointFrameworkUtilities.h"
#include "FirmwareIncludes.h"
#include "AbstractFramework.h"

//SDC2130 motor controller. Capable of moving based on torque, speed,
//or position. Unfortunately it's kind of shitty, best we have right now
//is budging it slowly with input speed commands that output just making
//it move position slightly. Only implemented modes are controlling it
//via pwm and moving by taking in a speed position
class Sdc2130: public OutputDevice
{
  private:
    int PWM_PIN, TX_PIN, RX_PIN;
    int pwmVal;

    const int PWM_MIN = 0, PWM_MAX = 255;
    const int POS_INC = 2;

    enum ControlTypes {Pwm, c_Serial};
    ControlTypes controlType;

  protected:

    //general move command. Pass in either speed or position values
    void move(const long movement);

    //move command if inputting speed, input movement is in the range of SPEED_MIN to SPEED_MAX
    void moveSpeed(const int movement);

    //move command if inputting position, input movement is in the range of POS_MIN to POS_MAX
    //void movePos(const long movement); not implemented

  public:
    //constructor for controlling it via pwm
    //inputs:
    //pwmPin: GPIO pin that's connected to the pwm input of the SDC2130 device. GPIO pin number id's are defined by energia's pinmapping
    //inType: instance of the ValueType enum that defines what kind of input the device should take, currently pos or spd. The motor controller can move based on either, so pick one
    //upside down: whether or not the motor is mounted in reverse so the input values would also need to be inverted
    Sdc2130(const int pwmPin, ValueType inType, bool upsideDown);

    //constructor for controlling it via serial
    //Sdc2130(const int txPin, const int rxPin, ValueType inType, bool upsideDown); not implemented
};

#endif