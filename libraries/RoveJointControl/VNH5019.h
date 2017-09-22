#ifndef VNH5019_H_
#define VNH5019_H_

#include "AbstractFramework.h"

class VNH5019 : public OutputDevice
{
  private:
    int PWM_PIN, INA_PIN, INB_PIN;
    int currentPower;

  protected:

    //move function which passes in power percent (which is converted to phase and PWM) to move device
    void move(const long movement); 
    
    //tells the device to power on or off. 
    void setPower(bool powerOn);
    
    //tells device to stop
    void stop();
    
  public:
  
    //pin asignments for hardware pins, also a bool to determine the orientation of the motor.
    //Pin numbers based on the roveboard standard
    VNH5019 (const int PwmPin, const int InaPin, const int InbPin, bool upsideDown);
    
    //returns the last move command this device got commanded.
    long getCurrentMove();
};

#endif
