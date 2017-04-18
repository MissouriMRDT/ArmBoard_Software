#ifndef VNH5019_H_
#define VNH5019_H_

#include "AbstractFramework.h"

//VNH5019 H bridge IC
class VNH5019 : public OutputDevice
{
  private:
    //constants for hardware pins
    //value ranges for min/max PWM 
    int PWM_PIN, INA_PIN, INB_PIN;
    const int PWM_MIN = 0, PWM_MAX = 255;


  protected:
    //move function which passes in speed to move device
    void move(const long movement); 
    
  public:
    //constructor here
    //pin asignments for hardware pins, also a bool to determine the orientation of da motor
    VNH5019 (const int PwmPin, const int InaPin, const int InbPin, bool upsideDown);
    
    //tells the device to power on or off. Note that on setup, device assumes power is off
    void togglePower(bool powerOn);
};

#endif