#include "RCContinuousServo.h"
#include "RoveBoard.h"

//static const int PWM_MAX_FWD = 2500, PWM_MAX_REV = 500; //values represent microseconds in pwm pulse width
static const int PWM_PERIOD = 20000; // also in microseconds

RCContinuousServo::RCContinuousServo(const int pwmPin, bool upsideDown)
{
  PWM_PIN = pwmPin;
  inType = InputPowerPercent;
  invert = upsideDown;

  PWM_STOP = 1495;  //some devices stop at values a little bit different from others, so it's modifiable
  currentPower = 0;
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
    pwm = mov+PWM_STOP;
    pwmWrite(PWM_PIN, pwm, PWM_PERIOD);
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
  pwmWrite(PWM_PIN, PWM_STOP, PWM_PERIOD);
  
  currentPower = 0;
}

void RCContinuousServo::setStopPeriod(long stopPeriod_us)
{
  PWM_STOP = stopPeriod_us;
}
