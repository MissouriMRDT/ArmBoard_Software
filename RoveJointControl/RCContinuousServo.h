#ifndef RCCONTINUOUSSERVO_H
#define RCCONTINUOUSSERVO_H

#include "AbstractFramework.h"
#include "RoveBoard.h"

class RCContinuousServo : public OutputDevice
{
  private:
    const rovePwmWrite_Handle PwmHandle;
    int pwm_stop_us;  //some devices stop at values a little bit different from others, so it's modifiable
    int currentPower;

  protected:
    // move function which passes in speed to move the device
    void move(const long movement);

    //tells the device to power on or off.
    void setPower(bool powrOn);
    
    //tells device to stop moving
    void stop();

  public:
    // overview: constructor for a RC Continuous Servo device.
    // inputs:   pwmGen: pwm generator reference
    //           pwmPin: pin assignment for the PWM pin
    //           upsideDown: whether or not the device is inverted
    //           pin assignment masks are based on roveboard's pin standard
    RCContinuousServo(const int pwmGen, const int pwmPin, bool upsideDown);

    //returns how much power the device was last told to move at
    long getCurrentMove();
    
    //sets the period of the pwm wave meant to stop the servo from moving, in microseconds
    void setStopPeriod(long stopPeriod_us);
};

#endif
