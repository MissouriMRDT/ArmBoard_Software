#include "RCContinuousServo.h"

//static const int PWM_MAX_FWD = 2500, PWM_MAX_REV = 500; //values represent microseconds in pwm pulse width
static const int PWM_PERIOD = 20000; // also in microseconds
static const int DEFAULT_STOP_US = 1495; //default pulse width needed to make servo stop moving

RCContinuousServo::RCContinuousServo(const int pwmGen, const int pwmPin, bool upsideDown)
  : OutputDevice(InputPowerPercent, upsideDown), pwm_stop_us(DEFAULT_STOP_US), currentPower(0), PwmHandle(setupPwmWrite(pwmGen, pwmPin))
{
  setPwmTotalPeriod(PwmHandle, PWM_PERIOD);
}

void RCContinuousServo::move(const long movement) {
  int mov = movement;
  int pwm = 0;

  if (enabled) {
    if (invert) // inverts the input
    {
      mov = -mov;
    }
      
    currentPower = mov;
    
    // adjust the mov value to fit the range.
    // mov is a scale from -1000 to 0 to 1000. the PWM range is 500 to 1500 to 2500
    // thus, the two can directly add up since both use a range of 2000. Just adjust for the
    // offset of where the two values use a 'o' value. 0 for speed, 1500 for pwm_stop.
    // Unless PMW_STOP has been modified by the user; keep the different ranges
    // in case the servo with the different stop PWM pulse also has modified
    // max and min values correspondingly
    pwm = mov+pwm_stop_us;
    pwmWriteWidth(PwmHandle, pwm);
  }
}

void RCContinuousServo::setPower(bool powerOn) {
  if (powerOn == false)
    stop();

  enabled = powerOn;
}

long RCContinuousServo::getCurrentMove()
{
  if(invert)
  {
    return(currentPower * -1); 
  }
  else
  {
    return(currentPower);
  }
}

void RCContinuousServo::stop()
{
  pwmWriteWidth(PwmHandle, pwm_stop_us);
  
  currentPower = 0;
}

void RCContinuousServo::setStopPeriod(long stopPeriod_us)
{
  pwm_stop_us = stopPeriod_us;
}
