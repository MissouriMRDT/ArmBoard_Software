#ifndef VNH5019_H_
#define VNH5019_H_

#include "JointFrameworkUtilities.h"
#include "FirmwareIncludes.h"
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
    //move function which passes in speed ( which is converted to phase and PWM) to move device
    void move(const long movement); 
    
    //cloning function, used to return a pointer to an exactly copy of this device
    virtual OutputDevice* clone();
    
    //blank constructor, useful for cloning
    VNH5019(){};
    
  public:

    //constructor here
    //pin asignments for hardware pins, also a bool to determine the orientation of da motor
    VNH5019 (const int PwmPin, const int InaPin, const int InbPin, bool upsideDown);
};

#endif