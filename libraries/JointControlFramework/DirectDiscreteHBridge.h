#ifndef DIRECTDISCRETEHBRIDGE_H_
#define DIRECTDISCRETEHBRIDGE_H_

#include "AbstractFramework.h"

class DirectDiscreteHBridge : public OutputDevice
{
  private:

    //FPWM_PIN the forward PWM and RPWM is the reverse PWM
    int FPWM_PIN, RPWM_PIN;

    //Value ranges for conversting input to expected output when sending
    const int PWM_MIN = 0, PWM_MAX = 255;

  protected:
    //function which directly moves the device.
    void move(const long movement);

  public:
    //constructor, user must give pin assignments with the first pin being the forward pin and the second being the reverse pin.
    //Must specify the physical orientation of the device as well, if it is mounted in reverse or not
    DirectDiscreteHBridge(const int FPIN, const int RPIN, bool upsideDown);
    
    //turns device on or off. Note on startup device is considered off, user must call this function and enable device before joints will move
    void setPower(bool powerOn);

};

#endif