#include "RCContinuousServo.h"
#include "pwmWriter.h"

RCContinuousServo::RCContinuousServo(const int pwmPin, bool upsideDown)
{
  PWM_PIN = pwmPin;
  inType = spd;
  invert = upsideDown;
}

void RCContinuousServo::move(const long movement) {
  int mov = movement;
  int pwm = 0;

  if (enabled) {
    if (invert) // inverts the input
    {
      mov = -mov;
    }
      
    currentSpeed = mov;
    
    // adjust the mov value to fit the range.
    // mov is a scale from -1000 to 0 to 1000. the PWM range is 500 to 1500 to 2500
    // thus, the two can directly add up since both use a range of 2000. Just adjust for the
    // offset of where the two values use a 'o' value. 0 for speed, 1500 for pwm_stop.
    pwm = mov+PWM_STOP;
    PwmWrite(PWM_PIN, pwm, PWM_PERIOD);
  }
}

void RCContinuousServo::setPower(bool powerOn) {
  if (powerOn == false)
    move(0);

  enabled = powerOn;
}

long RCContinuousServo::getCurrentMove()
{
  return(currentSpeed);
}