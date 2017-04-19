#ifndef RCCONTINUOUSSERVO_H
#define RCCONTINUOUSSERVO_H

// Generic class for any RC Continuous Servo device.

#include "AbstractFramework.h"


class RCContinuousServo : public OutputDevice
{
  private:
    int PWM_PIN;
    const int PWM_MAX_FWD = 2500, PWM_MAX_REV = 500, PWM_STOP = 1500; //values represent microseconds in pwm pulse width
    const int PWM_PERIOD = 20000; // this is in microseconds


  protected:
    // move function which passes in speed to move the device
    void move(const long movement);


  public:
    // constructor for a RC Continuous Servo device.
    // inputs are: pin assignments for the PWM pin, and whether or not the device is inverted
    // pin assignment masks are based on energia pin standard(s)
    RCContinuousServo(const int pwnPin, bool upsideDown);

    // tells the device to power on or off. Note that on setup, device assumes power is off
    void setPower(bool powrOn);
};

#endif
