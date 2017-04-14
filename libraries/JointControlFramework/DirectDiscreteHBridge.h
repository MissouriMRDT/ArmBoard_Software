#ifndef DIRECTDISCRETEHBRIDGE_H_
#define DIRECTDISCRETEHBRIDGE_H_

#include "JointFrameworkUtilities.h"
#include "FirmwareIncludes.h"
#include "AbstractFramework.h"

//Discrete H Bridge controlled directly by the microcontroller, which has only two inputs to control forward and backwards.
//Has two pins (one forward, one backwards). Outputs a pwm signal on one pin or another.
class DirectDiscreteHBridge : public OutputDevice
{
  private:
    //Value ranges for conversting input to expected output when sending
    const int PWM_MIN = 0, PWM_MAX = 255;

  protected:
    //function which directly moves the device.
    void move(const long movement);
    
    //FPWM_PIN the forward PWM and RPWM is the reverse PWM
    int FPWM_PIN, RPWM_PIN;
    
    //cloning function, used to return a pointer to an exactly copy of this device
    OutputDevice* clone();
    
    //empty constructor, useful for cloning
    DirectDiscreteHBridge(){};

  public:
    //constructor, user must give pin assignments with the first pin being the forward pin and the second being the reverse pin.
    //Must specify the physical orientation of the device as well, if it is mounted in reverse or not
    DirectDiscreteHBridge(const int FPIN, const int RPIN, bool upsideDown);

};

#endif