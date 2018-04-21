#ifndef ROVEJOINTCONTROL_VNH5019_H_
#define ROVEJOINTCONTROL_VNH5019_H_

#include "AbstractFramework.h"
#include "RoveBoard.h"

//represents the physical vnh5019 motor driver chip.
//see the readme.md for more info
class VNH5019 : public OutputDevice
{
  private:
    const int INA_PIN, INB_PIN;
    const RovePwmWrite_Handle PwmHandle;
    int currentPower;

  protected:

    //move function which passes in power percent (which is converted to phase and PWM) to move device
    void move(const long movement); 
    
    //tells the device to power on or off. 
    void setPower(bool powerOn);
    
    //tells device to stop
    void stop();
    
  public:
  
    //Inputs: pwm generator: reference to output a pwm wave on the pwm pin,
    //        pin assignments for hardware pins,
    //        a bool to determine the orientation of the motor.
    //Pin numbers based on the roveboard standard
    VNH5019(const int PwmGen, const int PwmPin, const int InaPin, const int InbPin, bool upsideDown);
    
    //returns the last move command this device got commanded.
    long getCurrentMove();
};

#endif
