#ifndef RCCONTINUOUSSERVO_H
#define RCCONTINUOUSSERVO_H

#include "AbstractFramework.h"


class RCContinuousServo : public OutputDevice
{
  private:
    int PWM_PIN;
    int PWM_STOP;  //some devices stop at values a little bit different from others, so it's modifiable
    int currentPower;

  protected:
    // move function which passes in speed to move the device
    void move(const long movement);

    //tells the device to power on or off.
    void setPower(bool powrOn);
    
    //tells device to stop moving
    void stop();

  public:
    // constructor for a RC Continuous Servo device.
    // inputs are: pin assignments for the PWM pin, and whether or not the device is inverted
    // pin assignment masks are based on roveboard's pin standard
    RCContinuousServo(const int pwnPin, bool upsideDown);

    //returns how much power the device was last told to move at
    long getCurrentMove();
    
    //sets the period of the pwm wave meant to stop the servo from moving, in microseconds
    void setStopPeriod(long stopPeriod_us);
};

#endif
