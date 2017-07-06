#ifndef VNH5019_H_
#define VNH5019_H_

#include "AbstractFramework.h"

class VNH5019 : public OutputDevice
{
  private:
    int PWM_PIN, INA_PIN, INB_PIN;
    const int PWM_MIN = 0, PWM_MAX = 255;
    int currentSpeed = 0;

  protected:
    //move function which passes in speed (which is converted to phase and PWM) to move device
    void move(const long movement); 
    
  public:
    //pin asignments for hardware pins, also a bool to determine the orientation of the motor
    VNH5019 (const int PwmPin, const int InaPin, const int InbPin, bool upsideDown);
    
    //tells the device to power on or off. Note that on setup, device assumes power is off
    void setPower(bool powerOn);
    
    long getCurrentMove();
};

#endif