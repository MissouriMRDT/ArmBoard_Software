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
    //moves by passing a pwm signal to the H bridge.
    //Input: expects int values constrained between the SPEED_MIN and SPEED_MAX constants
    void move(const long movement);

  public:
    //Creates the device. Assigns the pins correctly.
    //  int FPIN: the GPIO pin connected to the H bridge's forward transistor, pin number is defined by energia's pinmapping
    //  int RPIN: the GPIO pin connected to the H bridge's forward transistor, pin number is defined by energia's pinmapping
    //  bool upside down: Whether or not the motor is mounted in reverse and as such the inputs also need to be reversed
    DirectDiscreteHBridge(const int FPIN, const int RPIN, bool upsideDown);
    
    //turns device on or off. Note on startup device is considered off, user must call this function and enable device before joints will move
    void setPower(bool powerOn);

};

#endif