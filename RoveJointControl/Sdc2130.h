#ifndef ROVEJOINTCONTROL_SDC2130_H_
#define ROVEJOINTCONTROL_SDC2130_H_

#include "AbstractFramework.h"
#include "RoveBoard.h"

//represents the sdc2130 motor controller.
//see the readme.md for more info.
class Sdc2130: public OutputDevice
{
  private:
    const RovePwmWrite_Handle PwmHandle;
    int pwmVal;
    int currentPower;
    long currentPosition;

    enum ControlTypes {Pwm, c_Serial};
    ControlTypes controlType;

  protected:

    //general move command. Selects the specific move function.
    //Input: Can be either position or power values constrained between POWERPERCENT_MIN and POWERPERCENT_MAX
    void move(const long movement);

    //move command if inputting speed, input movement is in the range of POWERPERCENT_MIN to POWERPERCENT_MAX
    //Simply causes the joint to move a couple of positional ticks, as true speed based movement is not implemented due to strange delays in sdc2130 response
    void moveSpeed(const int movement);

    //move command if inputting position, input movement is in the range of POS_MIN to POS_MAX
    //void movePos(const long movement); not implemented
    
    //tells the device to power on or off.
    void setPower(bool powerOn);
    
    //tells device to stop moving
    void stop();

  public:

    //constructor for control via pwm
    //
    //inputs: pwmGen: The index of the pwm hardware generator
    //        pwmPin: GPIO pin that's connected to the pwm input of the SDC2130 device.
    //                GPIO pin number id's are defined by roveboard's pinmapping
    //        inType: instance of the ValueType enum that defines what kind of input the device should take.
    //                Currently just InputPowerPercent
    //        upside down: whether or not the motor is mounted in reverse so the input values would also need to be inverted
    Sdc2130(const int pwmGen, const int pwmPin, ValueType inType, bool upsideDown);

    //constructor for controlling it via serial
    //Sdc2130(const int txPin, const int rxPin, ValueType inType, bool upsideDown); not implemented
    
    //returns how much the device is moving in power% if it's set up to use powerpercent control,
    //or what it's destination position is if it's set up to use positional control
    long getCurrentMove();
};

#endif
